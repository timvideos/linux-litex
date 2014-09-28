/*
 * DMA driver for the Wishbone Streamer softcore
 * (https://github.com/olofk/wb_streamer)
 *
 * Copyright (C) 2014 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 *
 * Based on dma-jz4740
 * Copyright (C) 2013, Lars-Peter Clausen <lars@metafoo.de>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_dma.h>

#include "virt-dma.h"

#define WBSTREAMER_CSR			0x00
#define WBSTREAMER_START_ADDR		0x04
#define WBSTREAMER_BUF_SIZE		0x08
#define WBSTREAMER_BURST_SIZE		0x0c
#define WBSTREAMER_TRANSFERRED		0x10

#define WBSTREAMER_CSR_ENABLE	(1 << 0)
#define WBSTREAMER_CSR_IRQ	(1 << 1)

struct wbstream_dma_sg {
	dma_addr_t addr;
	unsigned int len;
};

struct wbstream_dma_desc {
	struct virt_dma_desc vdesc;

	enum dma_transfer_direction direction;
	bool cyclic;

	unsigned int num_sgs;
	struct wbstream_dma_sg sg[];
};

struct wbstream_dma;

struct wbstream_chan {
	struct virt_dma_chan vchan;
	unsigned int slave_id;
	struct wbstream_dma_desc *desc;
	unsigned int next_sg;
};

struct wbstream_dma {
	struct dma_device dma_dev;
	void __iomem *regs;
	struct wbstream_chan chan;
};

static void wbstream_write_reg(struct wbstream_dma *wdma, loff_t offset,
			u32 value)
{
	iowrite32be(value, wdma->regs + offset);
}

static u32 wbstream_read_reg(struct wbstream_dma *wdma, loff_t offset)
{
	return ioread32be(wdma->regs + offset);
}

static struct wbstream_dma_desc *wbstream_dma_alloc_desc(unsigned int num_sgs)
{
	return kzalloc(sizeof(struct wbstream_dma_desc) +
		       sizeof(struct wbstream_dma_sg) * num_sgs, GFP_ATOMIC);
}

static struct wbstream_chan *to_wbstream_chan(struct dma_chan *chan)
{
	return container_of(chan, struct wbstream_chan, vchan.chan);
}

static struct wbstream_dma *to_wbstream_dma(struct dma_device *device)
{
	return container_of(device, struct wbstream_dma, dma_dev);
}

static struct wbstream_dma_desc
*to_wbstream_dma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct wbstream_dma_desc, vdesc);
}

static struct dma_chan *wbstream_dma_of_xlate(struct of_phandle_args *dma_spec,
					      struct of_dma *ofdma)
{
	struct wbstream_dma *wdma = ofdma->of_dma_data;
	struct dma_chan *chan;
	struct wbstream_chan *wchan;

	chan = dma_get_any_slave_channel(&wdma->dma_dev);

	if (chan) {
		wchan = to_wbstream_chan(chan);
		wchan->slave_id = dma_spec->args[0];
	}

	return chan;
}

static int wbstream_dma_alloc_chan_resources(struct dma_chan *dc)
{
	return 0;
}

static void wbstream_dma_free_chan_resources(struct dma_chan *dc)
{
	vchan_free_chan_resources(to_virt_chan(dc));
}

static size_t wbstream_dma_desc_residue(struct wbstream_chan *chan,
	struct wbstream_dma_desc *desc, unsigned int next_sg)
{
	struct wbstream_dma *wdma = to_wbstream_dma(chan->vchan.chan.device);
	unsigned int residue;
	unsigned int i;

	residue = 0;

	for (i = next_sg; i < desc->num_sgs; i++)
		residue += desc->sg[i].len;

	if (next_sg != 0) {
		residue += wbstream_read_reg(wdma, WBSTREAMER_BUF_SIZE) -
			wbstream_read_reg(wdma, WBSTREAMER_TRANSFERRED);
	}

	return residue;
}

static enum dma_status wbstream_dma_tx_status(struct dma_chan *dc,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct wbstream_chan *chan = to_wbstream_chan(dc);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(dc, cookie, txstate);
	if (status == DMA_COMPLETE || !txstate)
		return status;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	vdesc = vchan_find_desc(&chan->vchan, cookie);
	if (cookie == chan->desc->vdesc.tx.cookie) {
		txstate->residue = wbstream_dma_desc_residue(chan, chan->desc,
				chan->next_sg);
	} else if (vdesc) {
		txstate->residue = wbstream_dma_desc_residue(chan,
				to_wbstream_dma_desc(vdesc), 0);
	} else {
		txstate->residue = 0;
	}
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return status;
}

static struct dma_async_tx_descriptor *wbstream_dma_prep_slave_sg(
	struct dma_chan *dc, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags,
	void *context)
{
	struct wbstream_chan *chan = to_wbstream_chan(dc);
	struct wbstream_dma_desc *desc;
	struct scatterlist *sg;
	unsigned int i;

	desc = wbstream_dma_alloc_desc(sg_len);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		desc->sg[i].addr = sg_dma_address(sg);
		desc->sg[i].len = sg_dma_len(sg);
	}

	desc->num_sgs = sg_len;
	desc->direction = direction;
	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *wbstream_dma_prep_dma_cyclic(
	struct dma_chan *dc, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct wbstream_chan *chan = to_wbstream_chan(dc);
	struct wbstream_dma_desc *desc;
	unsigned int num_periods, i;

	if (buf_len % period_len)
		return NULL;

	num_periods = buf_len / period_len;

	desc = wbstream_dma_alloc_desc(num_periods);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		desc->sg[i].addr = buf_addr;
		desc->sg[i].len = period_len;
		buf_addr += period_len;
	}

	desc->num_sgs = num_periods;
	desc->direction = direction;
	desc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static int wbstream_dma_terminate_all(struct dma_chan *dc)
{
	struct wbstream_chan *chan = to_wbstream_chan(dc);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	chan->desc = NULL;
	vchan_get_all_descriptors(&chan->vchan, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

static int wbstream_dma_control(struct dma_chan *dc, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	switch (cmd) {
	case DMA_TERMINATE_ALL:
		return wbstream_dma_terminate_all(dc);
	case DMA_SLAVE_CONFIG:
		/* Nothing to be done(?) */
		return 0;
	default:
		return -ENOSYS;
	}

	return -EINVAL;
}

static int wbstream_dma_start_transfer(struct wbstream_chan *chan)
{
	struct wbstream_dma *wdma = to_wbstream_dma(chan->vchan.chan.device);
	struct virt_dma_desc *vdesc;
	struct wbstream_dma_sg *sg;

	if (!chan->desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return 0;
		chan->desc = to_wbstream_dma_desc(vdesc);
		chan->next_sg = 0;
	}

	if (chan->next_sg == chan->desc->num_sgs)
		chan->next_sg = 0;

	sg = &chan->desc->sg[chan->next_sg];

	wbstream_write_reg(wdma, WBSTREAMER_START_ADDR, sg->addr);
	wbstream_write_reg(wdma, WBSTREAMER_BUF_SIZE, sg->len);
	wbstream_write_reg(wdma, WBSTREAMER_BURST_SIZE, 8);

	chan->next_sg++;
	wbstream_write_reg(wdma, WBSTREAMER_CSR, WBSTREAMER_CSR_ENABLE);

	return 0;
}

static irqreturn_t wbstream_irq_handler(int irq, void *dev_id)
{
	struct wbstream_dma *wdma = dev_id;
	struct wbstream_chan *chan = &wdma->chan;

	wbstream_write_reg(wdma, WBSTREAMER_CSR, WBSTREAMER_CSR_IRQ);

	spin_lock(&chan->vchan.lock);
	if (chan->desc) {
		if (chan->desc->cyclic) {
			vchan_cyclic_callback(&chan->desc->vdesc);
		} else {
			if (chan->next_sg == chan->desc->num_sgs) {
				chan->desc = NULL;
				vchan_cookie_complete(&chan->desc->vdesc);
			}
		}
	}
	wbstream_dma_start_transfer(chan);
	spin_unlock(&chan->vchan.lock);

	return IRQ_HANDLED;
}

static void wbstream_dma_issue_pending(struct dma_chan *dc)
{
	struct wbstream_chan *chan = to_wbstream_chan(dc);
	unsigned long flags;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (vchan_issue_pending(&chan->vchan) && !chan->desc)
		wbstream_dma_start_transfer(chan);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

}

static void wbstream_dma_desc_free(struct virt_dma_desc *vdesc)
{
	kfree(container_of(vdesc, struct wbstream_dma_desc, vdesc));
}

static int wbstream_dma_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct wbstream_dma *wdma;
	int irq;
	int ret;

	wdma = devm_kzalloc(&pdev->dev, sizeof(*wdma), GFP_KERNEL);
	if (!wdma)
		return -EINVAL;

	platform_set_drvdata(pdev, wdma);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdma->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdma->regs))
		return PTR_ERR(wdma->regs);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, wbstream_irq_handler, 0,
			       pdev->name, wdma);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return ret;
	}

	dma_cap_set(DMA_SLAVE, wdma->dma_dev.cap_mask);
	dma_cap_set(DMA_PRIVATE, wdma->dma_dev.cap_mask);

	wdma->dma_dev.device_alloc_chan_resources =
		wbstream_dma_alloc_chan_resources;
	wdma->dma_dev.device_free_chan_resources =
		wbstream_dma_free_chan_resources;
	wdma->dma_dev.device_prep_slave_sg = wbstream_dma_prep_slave_sg;
	wdma->dma_dev.device_prep_dma_cyclic = wbstream_dma_prep_dma_cyclic;
	wdma->dma_dev.device_control = wbstream_dma_control;
	wdma->dma_dev.device_tx_status = wbstream_dma_tx_status;
	wdma->dma_dev.device_issue_pending = wbstream_dma_issue_pending;

	wdma->dma_dev.dev = &pdev->dev;
	INIT_LIST_HEAD(&wdma->dma_dev.channels);

	wdma->chan.vchan.desc_free = wbstream_dma_desc_free;
	vchan_init(&wdma->chan.vchan, &wdma->dma_dev);

	ret = dma_async_device_register(&wdma->dma_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Wishbone streamer DMA driver registration failed %d\n",
			ret);
		return ret;
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 wbstream_dma_of_xlate, wdma);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Wishbone streamer DMA OF registration failed %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int wbstream_dma_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id wbstream_dma_of_match[] = {
	{ .compatible = "wb-streamer-dma", },
}
MODULE_DEVICE_TABLE(of, wbstream_dma_of_match);

static struct platform_driver wbstream_dma_driver = {
	.driver = {
		.name	= "wb-streamer-dma",
		.owner = THIS_MODULE,
		.of_match_table = wbstream_dma_of_match,
	},
	.probe		= wbstream_dma_probe,
	.remove		= wbstream_dma_remove,
};
module_platform_driver(wbstream_dma_driver);

MODULE_ALIAS("platform:wbstream-dma");
MODULE_DESCRIPTION("Wishbone streamer DMA Controller driver");
MODULE_AUTHOR("Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>");
MODULE_LICENSE("GPL v2");
