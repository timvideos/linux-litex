/*
 * LiteX Liteeth Ethernet
 *
 * Copyright 2017 Joel Stanley <joel@jms.id.au>
 *
 */

#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#include <linux/iopoll.h>

#define DRV_NAME	"liteeth"
#define DRV_VERSION	"0.1"

#define LITEETH_WRITER_SLOT		0x00
#define LITEETH_WRITER_LENGTH		0x04
#define LITEETH_WRITER_ERRORS		0x14
#define LITEETH_WRITER_EV_STATUS	0x24
#define LITEETH_WRITER_EV_PENDING	0x28
#define LITEETH_WRITER_EV_ENABLE	0x2c
#define LITEETH_READER_START		0x30
#define LITEETH_READER_READY		0x34
#define LITEETH_READER_LEVEL		0x38
#define LITEETH_READER_SLOT		0x3c
#define LITEETH_READER_LENGTH		0x40
#define LITEETH_READER_EV_STATUS	0x48
#define LITEETH_READER_EV_PENDING	0x4c
#define LITEETH_READER_EV_ENABLE	0x50
#define LITEETH_PREAMBLE_CRC		0x54
#define LITEETH_PREAMBLE_ERRORS		0x58
#define LITEETH_CRC_ERRORS		0x68

#define LITEETH_PHY_CRG_RESET		0x00
#define LITEETH_MDIO_W			0x04
#define LITEETH_MDIO_R			0x08

#define LITEETH_BUFFER_SIZE		0x800
#define MAX_PKT_SIZE			LITEETH_BUFFER_SIZE

struct liteeth {
	void __iomem *base;
	void __iomem *mdio_base;
	struct net_device *netdev;
	struct device *dev;
	struct mii_bus *mii_bus;

	/* Link management */
	int cur_duplex;
	int cur_speed;

	/* Tx */
	int tx_slot;
	int num_tx_slots;
	void __iomem *tx_base;

	/* Rx */
	int rx_slot;
	int num_rx_slots;
	void __iomem *rx_base;
};

/* Helper routines for accessing MMIO over a wishbone bus.
 * Each 32 bit memory location contains a single byte of data, stored
 * big endian
 */
static inline void outreg8(u8 val, void __iomem *addr)
{
	iowrite32be(val, addr);
}

static inline void outreg16(u16 val, void __iomem *addr)
{
	outreg8(val >> 8, addr);
	outreg8(val, addr + 4);
}

static inline u8 inreg8(void __iomem *addr)
{
	return ioread32be(addr);
}

static inline u32 inreg32(void __iomem *addr)
{
	return (inreg8(addr) << 24) |
		(inreg8(addr + 0x4) << 16) |
		(inreg8(addr + 0x8) <<  8) |
		(inreg8(addr + 0xc) <<  0);
}

static int liteeth_rx(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct sk_buff *skb;
	unsigned char *data;
	u8 rx_slot;
	int len;

	rx_slot = inreg8(priv->base + LITEETH_WRITER_SLOT);
	len = inreg32(priv->base + LITEETH_WRITER_LENGTH);

	skb = netdev_alloc_skb(netdev, len);
	if (!skb) {
		netdev_err(netdev, "couldn't get memory");
		netdev->stats.rx_dropped++;
		return NET_RX_DROP;
	}

	/* Ensure alignemnt of the ip header within the skb */
	skb_reserve(skb, NET_IP_ALIGN);

	data = skb_put(skb, len);
	memcpy_fromio(data, priv->rx_base + rx_slot * LITEETH_BUFFER_SIZE, len);
	skb->protocol = eth_type_trans(skb, netdev);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	return netif_rx(skb);
}

static void liteeth_tx_done(struct net_device *netdev)
{
	netdev->stats.tx_packets++;
}

static irqreturn_t liteeth_interrupt(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct liteeth *priv = netdev_priv(netdev);
	u8 reg;

	reg = inreg8(priv->base + LITEETH_READER_EV_PENDING);
	if (reg) {
		liteeth_tx_done(netdev);
		outreg8(reg, priv->base + LITEETH_READER_EV_PENDING);
	}

	reg = inreg8(priv->base + LITEETH_WRITER_EV_PENDING);
	if (reg) {
		liteeth_rx(netdev);
		outreg8(reg, priv->base + LITEETH_WRITER_EV_PENDING);
	}

	return IRQ_HANDLED;
}

static int liteeth_open(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	int err;

	/* TODO: Remove these once we have working mdio support */
	priv->cur_duplex = DUPLEX_FULL;
	priv->cur_speed = SPEED_100;
	netif_carrier_on(netdev);

	err = request_irq(netdev->irq, liteeth_interrupt, 0, netdev->name, netdev);
	if (err) {
		netdev_err(netdev, "failed to request irq %d\n", netdev->irq);
		goto err_irq;
	}

	/* Clear pending events? */
	outreg8(1, priv->base + LITEETH_WRITER_EV_PENDING);
	outreg8(1, priv->base + LITEETH_READER_EV_PENDING);

	/* Enable IRQs? */
	outreg8(1, priv->base + LITEETH_WRITER_EV_ENABLE);
	outreg8(1, priv->base + LITEETH_READER_EV_ENABLE);

	netif_start_queue(netdev);

	return 0;

err_irq:
	netif_carrier_off(netdev);
	return err;
}

static int liteeth_stop(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);

	outreg8(0, priv->base + LITEETH_WRITER_EV_ENABLE);
	outreg8(0, priv->base + LITEETH_READER_EV_ENABLE);

	free_irq(netdev->irq, netdev);

	return 0;
}

static int liteeth_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	void *txbuffer;
	int ret;
	u8 val;

	/* Reject oversize packets */
	if (unlikely(skb->len > MAX_PKT_SIZE)) {
		if (net_ratelimit())
			netdev_dbg(netdev, "tx packet too big\n");
		goto drop;
	}

	txbuffer = priv->tx_base + priv->tx_slot * LITEETH_BUFFER_SIZE;
	memcpy_fromio(txbuffer, skb->data, skb->len);
	outreg8(priv->tx_slot, priv->base + LITEETH_READER_SLOT);
	outreg16(skb->len, priv->base + LITEETH_READER_LENGTH);

	ret = readb_poll_timeout_atomic(priv->base + LITEETH_READER_READY,
			val, !val, 5, 1000);
	if (ret == -ETIMEDOUT) {
		netdev_err(netdev, "LITEETH_READER_READY timed out\n");
		goto drop;
	}

	outreg8(1, priv->base + LITEETH_READER_START);

	priv->tx_slot = (priv->tx_slot + 1) % priv->num_tx_slots;

	return NETDEV_TX_OK;
drop:
	/* Drop the packet */
	dev_kfree_skb_any(skb);
	netdev->stats.tx_dropped++;

	return NETDEV_TX_OK;
}

static void liteeth_get_drvinfo(struct net_device *netdev,
				struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev_name(&netdev->dev), sizeof(info->bus_info));
}

static const struct net_device_ops liteeth_netdev_ops = {
	.ndo_open		= liteeth_open,
	.ndo_stop		= liteeth_stop,
	.ndo_start_xmit         = liteeth_start_xmit,
};

static const struct ethtool_ops liteeth_ethtool_ops = {
	.get_drvinfo		= liteeth_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.nway_reset		= phy_ethtool_nway_reset,
};

static void liteeth_reset_hw(struct liteeth *priv)
{
	/* Reset, twice */
	outreg8(0, priv->base + LITEETH_PHY_CRG_RESET);
	udelay(10);
	outreg8(1, priv->base + LITEETH_PHY_CRG_RESET);
	udelay(10);
	outreg8(0, priv->base + LITEETH_PHY_CRG_RESET);
	udelay(10);
}

static int liteeth_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct net_device *netdev;
	void __iomem *buf_base;
	struct resource *res;
	struct liteeth *priv;
	const char *mac_addr;
	int irq, err;

	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->dev = &pdev->dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		err = PTR_ERR(priv->base);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->mdio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->mdio_base)) {
		err = PTR_ERR(priv->mdio_base);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	buf_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(buf_base)) {
		err = PTR_ERR(buf_base);
		goto err;
	}

	err = of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth",
			&priv->num_rx_slots);
	if (err) {
		dev_err(&pdev->dev, "unable to get rx-fifo-depth\n");
		goto err;
	}

	err = of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth",
			&priv->num_tx_slots);
	if (err) {
		dev_err(&pdev->dev, "unable to get tx-fifo-depth\n");
		goto err;
	}

	/* Rx slots */
	priv->rx_base = buf_base;
	priv->rx_slot = 0;

	/* Tx slots come after Rx slots */
	priv->tx_base = buf_base + priv->num_rx_slots * LITEETH_BUFFER_SIZE;
	priv->tx_slot = 0;

	mac_addr = of_get_mac_address(np);
	if (mac_addr && is_valid_ether_addr(mac_addr))
		memcpy(netdev->dev_addr, mac_addr, ETH_ALEN);
	else
		eth_hw_addr_random(netdev);

	SET_NETDEV_DEV(netdev, &pdev->dev);
	platform_set_drvdata(pdev, netdev);

	netdev->netdev_ops = &liteeth_netdev_ops;
	netdev->ethtool_ops = &liteeth_ethtool_ops;
	netdev->irq = irq;

	liteeth_reset_hw(priv);

	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev\n");
		goto err;
	}

	netdev_info(netdev, "irq %d, mapped at %p\n", netdev->irq, priv->base);

	return 0;
err:
	free_netdev(netdev);
	return err;
}

static int liteeth_remove(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct liteeth *priv;

	netdev = platform_get_drvdata(pdev);
	priv = netdev_priv(netdev);

	unregister_netdev(netdev);

	free_netdev(netdev);

	return 0;
}

static const struct of_device_id liteeth_of_match[] = {
	{ .compatible = "litex,liteeth" },
	{ }
};
MODULE_DEVICE_TABLE(of, liteeth_of_match);

static struct platform_driver liteeth_driver = {
	.probe = liteeth_probe,
	.remove = liteeth_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = liteeth_of_match,
	},
};
module_platform_driver(liteeth_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_LICENSE("GPL");
