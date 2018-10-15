/*
 * Litex SoCs Build Info Driver
 *
 * Copyright (C) 2018 Stafford Horne
 *
 * Author: Stafford Horne <shorne@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define DRV_NAME	"litexinfo"
#define DRV_VERSION	"0.1"

#define LITEX_INFO_REVISION_OFFSET	0x20
#define LITEX_INFO_REVISION_SIZE	20
#define LITEX_INFO_PLATFORM_OFFSET	0x70
#define LITEX_INFO_PLATFORM_SIZE	8
#define LITEX_INFO_TARGET_OFFSET	0x90
#define LITEX_INFO_TARGET_SIZE		8

static void __iomem *litexinfo_base;

static ssize_t litexinfo_str_read(char *buf, void __iomem *addr, size_t size)
{
	size_t i;

	for (i = 0; i < size; i++) {
		buf[i] = (char) ioread32be(addr + (i*4));
		if (buf[i] == '\0')
			return i;
	}
	return -1;
}

static ssize_t litexinfo_hex_read(char *buf, void __iomem *addr, size_t size)
{
	size_t i;

	for (i = 0; i < size; i++) {
		sprintf(buf + (i*2), "%02x", ioread32be(addr + (i*4)));
	}
	return -1;
}

static int litexinfo_show(struct seq_file *m, void *v)
{
	char strbuf[64];

	seq_printf(m, "Litex version info\n");
	litexinfo_str_read(strbuf,
			    litexinfo_base + LITEX_INFO_PLATFORM_OFFSET,
			    LITEX_INFO_PLATFORM_SIZE);
	seq_printf(m, "        platform: %s\n", strbuf);
	litexinfo_str_read(strbuf,
			    litexinfo_base + LITEX_INFO_TARGET_OFFSET,
			    LITEX_INFO_TARGET_SIZE);
	seq_printf(m, "        target: %s\n", strbuf);
	litexinfo_hex_read(strbuf,
			    litexinfo_base + LITEX_INFO_REVISION_OFFSET,
			    LITEX_INFO_REVISION_SIZE);
	seq_printf(m, "        revision: %s\n", strbuf);
	return 0;
}

static int litexinfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, litexinfo_show, NULL);
}

static const struct file_operations litexinfo_fops = {
	.open		= litexinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int litexinfo_probe(struct platform_device *pdev)
{
	struct resource res;
	struct device_node *node;

	if (litexinfo_base) {
		pr_err("litexinfo: multiple litex info's are not supported");
		return -EEXIST;
	}

	node = pdev->dev.of_node;

	if (of_address_to_resource(node, 0, &res)) {
		pr_err("litexinfo: reg property requires an address and size");
		return -EINVAL;
	}

	litexinfo_base = devm_ioremap_resource(&pdev->dev, &res);
	if (IS_ERR(litexinfo_base))
		return PTR_ERR(litexinfo_base);

	/* TODO: covert to upstream api proc_create_single */
	proc_create("litex", 0644, NULL, &litexinfo_fops);
	return 0;
}

static int litexinfo_remove(struct platform_device *pdev)
{
	remove_proc_entry("litex", NULL);

	return 0;
}

static const struct of_device_id litexinfo_of_match[] = {
	{ .compatible = "litex,litexinfo" },
	{ }
};
MODULE_DEVICE_TABLE(of, litexinfo_of_match);

static struct platform_driver litexinfo_driver = {
	.probe = litexinfo_probe,
	.remove = litexinfo_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = litexinfo_of_match,
	},
};
module_platform_driver(litexinfo_driver);

MODULE_AUTHOR("Stafford Horne <shorne@gmail.com>>");
MODULE_DESCRIPTION("Litex Info Driver");
MODULE_LICENSE("GPL");
