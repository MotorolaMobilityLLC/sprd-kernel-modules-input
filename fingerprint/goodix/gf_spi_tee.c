/* Goodix's GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206/GF5216/GF5208
 *  fingerprint sensor linux driver for TEE
 *
 * 2010 - 2015 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/pm_wakeup.h>
 #include <linux/of_gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <linux/clk.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/platform_device.h>//shasha

#include "gf_spi_tee.h"


/**************************defination******************************/
#define GF_DEV_NAME "goodix_fp"
#define GF_DEV_MAJOR 0	/* assigned */

#define GF_CLASS_NAME "goodix_fp"
#define GF_INPUT_NAME "gf-keys"

#define GF_LINUX_VERSION "V1.01.06"

#define GF_NETLINK_ROUTE 29   /* for GF test temporary */
#define MAX_NL_MSG_LEN 16

#define WAKELOCK_HOLD_TIME 500 /* in ms */
/*************************************************************/

/* debug log setting */
u8 g_debug_level = DEBUG_LOG;

/* align=2, 2 bytes align */
/* align=4, 4 bytes align */
/* align=8, 8 bytes align */
#define ROUND_UP(x, align)		((x+(align-1))&~(align-1))


/*************************************************************/
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned int bufsiz = (25 * 1024);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "maximum data bytes for SPI message");

#ifdef CONFIG_OF
static const struct of_device_id gf_of_match[] = {
	{ .compatible = "sprd,goodix-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, gf_of_match);
#endif

static struct gf_device *ggf_dev = NULL;

/* for netlink use */
static int pid = 0;

static u8 g_vendor_id = 0;
static u8 gpio_ready = 0;
static ssize_t gf_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t gf_debug_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, gf_debug_show, gf_debug_store);

static ssize_t fpname_get(struct device *device,
        struct device_attribute *attribute,
        char* buffer)
{
	struct gf_device *gf_dev = dev_get_drvdata(device);
	gf_debug(DEBUG_LOG, "gf_dev %s get fpname:%s", __func__, gf_dev->fpname);
	return snprintf(buffer, sizeof(gf_dev->fpname), "%s\n", gf_dev->fpname);
}

static DEVICE_ATTR(fpname, S_IRUSR | S_IRGRP | S_IROTH, fpname_get, NULL);

static struct attribute *gf_debug_attrs[] = {
	&dev_attr_debug.attr,
	&dev_attr_fpname.attr,
	NULL
};

static const struct attribute_group gf_debug_attr_group = {
	.attrs = gf_debug_attrs,
	.name = "debug"
};


/* -------------------------------------------------------------------- */
/* timer function								*/
/* -------------------------------------------------------------------- */
#define TIME_START	   0
#define TIME_STOP	   1

static long int prev_time, cur_time;

long int kernel_time(unsigned int step)
{
	cur_time = ktime_to_us(ktime_get());
	if (step == TIME_START) {
		prev_time = cur_time;
		return 0;
	} else if (step == TIME_STOP) {
		gf_debug(DEBUG_LOG, "%s, use: %ld us\n", __func__, (cur_time - prev_time));
		return cur_time - prev_time;
	}
	prev_time = cur_time;
	return -1;
}
//shasha
int gf_parse_dts(struct gf_device *gf_dev)
{
	int rc = 0;
	struct device *dev = &(gf_dev->pldev->dev);
	gf_debug(DEBUG_LOG, "%s, from gpio count = %d,gpio ready = %d\n", __func__,gpiod_count(dev,"gfrst"),gpio_ready);
	if(gpio_ready)
	{
		gf_debug(DEBUG_LOG, "%s, gpio ready\n", __func__);
		return 0;
	}
	gf_dev->rst_gpio_desc = devm_gpiod_get(dev, "gfrst", GPIOD_OUT_LOW);
	rc = gpiod_direction_output(gf_dev->rst_gpio_desc,0);
	if (rc) {
		pr_err("failed to set reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
#if 0
	gf_dev->avdd_gpio_desc = devm_gpiod_get(dev, "gfvdd", GPIOD_OUT_LOW);
	rc = gpiod_direction_output(gf_dev->avdd_gpio_desc,0);
	if (rc) {
		pr_err("failed to set avdd gpio, rc = %d\n", rc);
		goto err_reset;
	}
#endif
	return rc;

err_reset:
	return rc;
}

void gf_cleanup(struct gf_device *gf_dev)
{
	pr_info("[info] %s\n", __func__);
	gpio_ready = 0;
#if 0
	if (NULL != gf_dev->avdd_gpio_desc) {
		gpiod_put(gf_dev->avdd_gpio_desc);
		pr_info("remove avdd_gpio success\n");
	}
#endif
	if (NULL != gf_dev->rst_gpio_desc) {
		gpiod_put(gf_dev->rst_gpio_desc);
		pr_info("remove reset_gpio success\n");
	}
	if (NULL != gf_dev->irq_gpio_desc) {
		gpiod_put(gf_dev->irq_gpio_desc);
		pr_info("remove irq_gpio success\n");
	}
}
int gf_hw_reset(struct gf_device *gf_dev, unsigned int delay_ms)
{
	int rc = 0;
	if (gf_dev->rst_gpio_desc == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	rc = gpiod_direction_output(gf_dev->rst_gpio_desc,0);
	if (rc) {
		pr_err("failed to set reset gpio, rc = %d\n", rc);
		return rc;
	}
	gpiod_set_value(gf_dev->rst_gpio_desc, 0);
	mdelay(3);
	gpiod_set_value(gf_dev->rst_gpio_desc, 1);
	mdelay(delay_ms);
	return 0;
}

static int gf_get_sensor_dts_info(void)
{
	struct device_node *node = NULL;
	int value;

	node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
	if (node) {
		of_property_read_u32(node, "netlink-event", &value);
		gf_debug(DEBUG_LOG, "%s, get netlink event[%d] from dts\n", __func__, value);
	} else {
		gf_debug(ERR_LOG, "%s failed to get device node!\n", __func__);
		return -ENODEV;
	}

	return 0;
}

#ifdef USE_REGULATOR
static int gf_set_ldo_enable(struct gf_device *gf_dev, u8 onoff)
{
	int rc;
	struct regulator *vreg;
	struct device *pdev = gf_dev->pdev;
	char ldo_name[24] = "vdd_io";

	vreg = gf_dev->vreg;
	if (onoff) {
		if(!vreg) {
			vreg = devm_regulator_get(pdev, "vdd_io");
			if (IS_ERR(vreg)) {
				pr_err("Unable to get %s\n", ldo_name);
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf.vmin, vreg_conf.vmax);
			if (rc) {
				pr_err( "Unable to set voltage on %s, %d\n", ldo_name, rc);
				return rc;
			}
		}

		rc = regulator_set_load(vreg, vreg_conf.ua_load);
		if (rc < 0) {
			pr_err("Unable to set current on %s, %d\n", ldo_name, rc);
			return rc;
		}

		rc = regulator_enable(vreg);
		if (rc) {
			pr_err("error enabling %s: %d\n", ldo_name, rc);
			return rc;
		}
		gf_dev->vreg = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				rc = regulator_disable(vreg);
				if (rc) {
					pr_err("disabled %s fail\n", ldo_name);
					return rc;
				}
			}
			pr_err("wtd post 5\n");
			gf_dev->vreg = NULL;
		}
		rc = 0;
	}
	return rc;
}
#endif

//#define GF_POWER_GPIO	   1
#define GF_POWER_EXT_LDO	   1
static void gf_hw_power_enable(struct gf_device *gf_dev, u8 onoff)
{
	/* TODO: LDO configure */
	static int enable = 1;
	int rc = 0;
	if (onoff && enable) {
	/* TODO:  set power  according to actual situation  */
		enable = 0;

#ifdef GF_POWER_GPIO
		gpiod_direction_output(gf_dev->avdd_gpio_desc, 1);
		pr_info("set pwr_gpio on");
#elif GF_POWER_EXT_LDO
		//rc = wl2868c_set_ldo_enable(LDO4, 3000);
		//pr_info("---- power_on external ldo ---- rc = %d\n",rc);
#ifdef USE_REGULATOR
		rc = gf_set_ldo_enable(gf_dev, 1);
		if (rc)
			pr_err("set ldo power on failed rc = %d\n", rc);
		else
			pr_info("set ldo power on success\n");
#endif
#else
		rc = -1;
		pr_info("---- power on mode not set !!! ----\n");
#endif
		gf_hw_reset(gf_dev, 15);
	} else if (!onoff && !enable) {
		enable = 1;

#ifdef GF_POWER_GPIO
		gpiod_direction_output(gf_dev->avdd_gpio_desc, 0);
		gpiod_set_value(gf_dev->rst_gpio_desc, 0);
		pr_info("set pwr_gpio off");
#elif GF_POWER_EXT_LDO
		//rc = wl2868c_set_ldo_disable(LDO4);
		//pr_info("---- power_off external ldo ---- rc = %d\n",rc);
#ifdef USE_REGULATOR
		rc = gf_set_ldo_enable(gf_dev, 0);
		if (rc)
			pr_err("set ldo power off failed rc = %d\n", rc);
		else
			pr_info("set ldo power off success\n");
#endif
#else
		rc = -1;
		pr_info("---- power on mode not set !!! ----\n");
#endif
	}
}

static void gf_bypass_flash_gpio_cfg(void)
{
	/* TODO: by pass flash IO config, default connect to GND */
}

static void gf_irq_gpio_cfg(struct gf_device *gf_dev)
{
	int ret = 0;
	struct device *dev = &(gf_dev->pldev->dev);
	gf_debug(DEBUG_LOG, "%s, gpio ready=%d\n", __func__,gpio_ready);
	if(gpio_ready == 0)
	{
		gf_dev->irq_gpio_desc = devm_gpiod_get(dev, "gfirq", GPIOD_IN);
	}
	ret = gpiod_direction_input(gf_dev->irq_gpio_desc);
	if (ret) {
		pr_err("failed to set irq gpio, ret = %d\n", ret);
		//return;
	}
	gf_dev->irq_num = gpiod_to_irq(gf_dev->irq_gpio_desc);
	if (gf_dev->irq_num < 0) {
		pr_err("failed to set irq num, ret = %d\n", gf_dev->irq_num);
		//return;
	}
	gf_debug(INFO_LOG, "%s, gf_irq = %d\n", __func__, gf_dev->irq_num);
	gf_dev->irq = gf_dev->irq_num;
	gpio_ready = 1;
	gf_debug(INFO_LOG, "[%s] : irq config success\n", __func__);
}

/*
static void gf_reset_gpio_cfg(struct gf_device *gf_dev)
{
#ifdef CONFIG_OF
	//gpio_set_value(gf_dev->reset_gpio, 1);
#endif

}
*/


static void gf_enable_irq(struct gf_device *gf_dev)
{
	if (1 == gf_dev->irq_count) {
		gf_debug(ERR_LOG, "%s, irq already enabled\n", __func__);
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_count = 1;
		gf_debug(DEBUG_LOG, "%s enable interrupt!\n", __func__);
	}
}

static void gf_disable_irq(struct gf_device *gf_dev)
{
	if (0 == gf_dev->irq_count) {
		gf_debug(ERR_LOG, "%s, irq already disabled\n", __func__);
	} else {
		disable_irq(gf_dev->irq);
		gf_dev->irq_count = 0;
		gf_debug(DEBUG_LOG, "%s disable interrupt!\n", __func__);
	}
}


/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
void gf_netlink_send(struct gf_device *gf_dev, const int command)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret;

	gf_debug(INFO_LOG, "[%s] : enter, send command %d\n", __func__, command);
	if (NULL == gf_dev->nl_sk) {
		gf_debug(ERR_LOG, "[%s] : invalid socket\n", __func__);
		return;
	}

	if (0 == pid) {
		gf_debug(ERR_LOG, "[%s] : invalid native process pid\n", __func__);
		return;
	}

	/*alloc data buffer for sending to native*/
	/*malloc data space at least 1500 bytes, which is ethernet data length*/
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL) {
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		gf_debug(ERR_LOG, "[%s] : nlmsg_put failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	ret = netlink_unicast(gf_dev->nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret == 0) {
		gf_debug(ERR_LOG, "[%s] : send failed\n", __func__);
		return;
	}

	gf_debug(INFO_LOG, "[%s] : send done, data length is %d\n", __func__, ret);
}

static void gf_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];
	uint32_t data_len;

	gf_debug(INFO_LOG, "[%s] : enter \n", __func__);

	skb = skb_get(__skb);
	if (skb == NULL) {
		gf_debug(ERR_LOG, "[%s] : skb_get return NULL\n", __func__);
		return;
	}

	/* presume there is 5byte payload at leaset */
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		data_len = NLMSG_PAYLOAD(nlh, 0);
		memcpy(str, NLMSG_DATA(nlh), min(data_len, (uint32_t)sizeof(str)));
		pid = nlh->nlmsg_pid;
		gf_debug(INFO_LOG, "[%s] : pid: %d, msg: %s\n", __func__, pid, str);

	} else {
		gf_debug(ERR_LOG, "[%s] : not enough data length\n", __func__);
	}

	kfree_skb(skb);
}

static int gf_netlink_init(struct gf_device *gf_dev)
{
	struct netlink_kernel_cfg cfg;

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = gf_netlink_recv;

	gf_dev->nl_sk = netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
	if (gf_dev->nl_sk == NULL) {
		gf_debug(ERR_LOG, "[%s] : netlink create failed\n", __func__);
		return -1;
	}

	gf_debug(INFO_LOG, "[%s] : netlink create success\n", __func__);
	return 0;
}

static int gf_netlink_destroy(struct gf_device *gf_dev)
{
	if (gf_dev->nl_sk != NULL) {
		netlink_kernel_release(gf_dev->nl_sk);
		gf_dev->nl_sk = NULL;
		return 0;
	}

	gf_debug(ERR_LOG, "[%s] : no netlink socket yet\n", __func__);
	return -1;
}

/* -------------------------------------------------------------------- */
/* early suspend callback and suspend/resume functions          */
/* -------------------------------------------------------------------- */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);
	gf_debug(INFO_LOG, "[%s] enter\n", __func__);

	gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
}

static void gf_late_resume(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);
	gf_debug(INFO_LOG, "[%s] enter\n", __func__);

	gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
}
#else

static int gf_fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct gf_device *gf_dev = NULL;
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;
	FUNC_ENTRY();

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */)
		return 0;

	gf_dev = container_of(self, struct gf_device, notifier);
	blank = *(int *)evdata->data;

	gf_debug(INFO_LOG, "[%s] : enter, blank=0x%x\n", __func__, blank);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		gf_debug(INFO_LOG, "[%s] : lcd on notify\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
		break;

	case FB_BLANK_POWERDOWN:
		gf_debug(INFO_LOG, "[%s] : lcd off notify\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
		break;

	default:
		gf_debug(INFO_LOG, "[%s] : other notifier, ignore\n", __func__);
		break;
	}
	FUNC_EXIT();
	return retval;
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

void gf_fb_set_screen_status(bool st)
{
	if (!strstr(ggf_dev->fpname, "gc3976")) {
		gf_debug(INFO_LOG, "[%s] : gc3976 fingerprint not exist\n", __func__);
		return;
	}
	if (st) {
		gf_debug(INFO_LOG, "[%s] : lcd on notify\n", __func__);
		gf_netlink_send(ggf_dev, GF_NETLINK_SCREEN_ON);
	} else {
		gf_debug(INFO_LOG, "[%s] : lcd off notify\n", __func__);
		gf_netlink_send(ggf_dev, GF_NETLINK_SCREEN_OFF);
	}
}
EXPORT_SYMBOL(gf_fb_set_screen_status);
/* -------------------------------------------------------------------- */
/* file operation function                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;

	FUNC_EXIT();
	return retval;
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	gf_debug(ERR_LOG, "%s: Not support write opertion in TEE mode\n", __func__);
	return -EFAULT;
}

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_device *gf_dev = (struct gf_device *)handle;
	FUNC_ENTRY();

	//__pm_wakeup_event(&gf_dev->fp_wakesrc, WAKELOCK_HOLD_TIME);
	gf_netlink_send(gf_dev, GF_NETLINK_IRQ);
	gf_dev->sig_count++;

	FUNC_EXIT();
	return IRQ_HANDLED;
}


static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_device *gf_dev = NULL;
	struct gf_key gf_key;
	gf_nav_event_t nav_event = GF_NAV_NONE;
	uint32_t nav_input = 0;
	uint32_t key_input = 0;

	int retval = 0;
	u8  buf    = 0;
	u8 netlink_route = GF_NETLINK_ROUTE;
	struct gf_ioc_chip_info info;

	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -EINVAL;

	/* Check access direction once here; don't repeat below.
	* IOC_DIR is from the user perspective, while access_ok is
	* from the kernel perspective; so they look reversed.
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));

	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));

	if (retval)
		return -EINVAL;

	gf_dev = (struct gf_device *)filp->private_data;
	if (!gf_dev) {
		gf_debug(ERR_LOG, "%s: gf_dev IS NULL ======\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case GF_IOC_INIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_INIT gf init======\n", __func__);
		gf_debug(INFO_LOG, "%s: Linux Version %s\n", __func__, GF_LINUX_VERSION);

		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8))) {
			retval = -EFAULT;
			break;
		}
		if (gf_dev->system_status) {
			gf_debug(INFO_LOG, "%s: system re-started======\n", __func__);
			gf_disable_irq(gf_dev);
			disable_irq_wake(gf_dev->irq);
			if (gf_dev->irq) {
				free_irq(gf_dev->irq, gf_dev);
				gf_dev->irq_count = 0;
				gf_dev->irq = 0;
			}
			//break;
		}
		gf_irq_gpio_cfg(gf_dev);
		retval = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "goodix_fp_irq", gf_dev);
		if (!retval)
			gf_debug(INFO_LOG, "%s irq thread request success!\n", __func__);
		else
			gf_debug(ERR_LOG, "%s irq thread request failed, retval=%d\n", __func__, retval);


		gf_dev->irq_count = 1;
		enable_irq_wake(gf_dev->irq);
		gf_disable_irq(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
		gf_debug(INFO_LOG, "[%s] : register_early_suspend\n", __func__);
		gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
		gf_dev->early_suspend.suspend = gf_early_suspend,
		gf_dev->early_suspend.resume = gf_late_resume,
		register_early_suspend(&gf_dev->early_suspend);
#else
		/* register screen on/off callback */
		gf_dev->notifier.notifier_call = gf_fb_notifier_callback;
		fb_register_client(&gf_dev->notifier);
#endif

		gf_dev->sig_count = 0;
		gf_dev->system_status = 1;

		gf_debug(INFO_LOG, "%s: gf init finished======\n", __func__);
		break;

	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg, sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		g_vendor_id = info.vendor_id;
		snprintf(gf_dev->fpname, sizeof(gf_dev->fpname), "gc3976");
		gf_debug(INFO_LOG, "%s: vendor_id 0x%x\n", __func__, g_vendor_id);
		gf_debug(INFO_LOG, "%s: mode 0x%x\n", __func__, info.mode);
		gf_debug(INFO_LOG, "%s: operation 0x%x\n", __func__, info.operation);
		break;

	case GF_IOC_EXIT:
		gf_debug(INFO_LOG, "%s: GF_IOC_EXIT ======\n", __func__);
		gf_disable_irq(gf_dev);
		disable_irq_wake(gf_dev->irq);
		if (gf_dev->irq) {
			free_irq(gf_dev->irq, gf_dev);
			gf_dev->irq_count = 0;
			gf_dev->irq = 0;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		if (gf_dev->early_suspend.suspend)
			unregister_early_suspend(&gf_dev->early_suspend);
#else
		fb_unregister_client(&gf_dev->notifier);
#endif

		gf_dev->system_status = 0;
		gf_debug(INFO_LOG, "%s: gf exit finished ======\n", __func__);
		break;

	case GF_IOC_RESET:
		gf_debug(INFO_LOG, "%s: chip reset command\n", __func__);
		gf_hw_reset(gf_dev, 60);
		break;

	case GF_IOC_ENABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_IRQ ======\n", __func__);
		gf_enable_irq(gf_dev);
		break;

	case GF_IOC_DISABLE_IRQ:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_IRQ ======\n", __func__);
		gf_disable_irq(gf_dev);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_SPI_CLK ======\n", __func__);
		break;

	case GF_IOC_DISABLE_SPI_CLK:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_SPI_CLK ======\n", __func__);
		break;

	case GF_IOC_ENABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENABLE_POWER ======\n", __func__);
		gf_hw_power_enable(gf_dev, 1);
		break;

	case GF_IOC_DISABLE_POWER:
		gf_debug(INFO_LOG, "%s: GF_IOC_DISABLE_POWER ======\n", __func__);
		gf_hw_power_enable(gf_dev, 0);
		break;

	case GF_IOC_INPUT_KEY_EVENT:
		if (copy_from_user(&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
			gf_debug(ERR_LOG, "Failed to copy input key event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		if (GF_KEY_HOME == gf_key.key) {
			key_input = GF_KEY_INPUT_HOME;
		} else if (GF_KEY_POWER == gf_key.key) {
			key_input = GF_KEY_INPUT_POWER;
		} else if (GF_KEY_CAMERA == gf_key.key) {
			key_input = GF_KEY_INPUT_CAMERA;
		} else {
			/* add special key define */
			key_input = gf_key.key;
		}
		gf_debug(INFO_LOG, "%s: received key event[%d], key=%d, value=%d\n",
				__func__, key_input, gf_key.key, gf_key.value);

		if ((GF_KEY_POWER == gf_key.key || GF_KEY_CAMERA == gf_key.key) && (gf_key.value == 1)) {
			input_report_key(gf_dev->input, key_input, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, key_input, 0);
			input_sync(gf_dev->input);
		}

		if (GF_KEY_HOME == gf_key.key) {
		    input_report_key(gf_dev->input, key_input, gf_key.value);
		    input_sync(gf_dev->input);
		}

		break;

	case GF_IOC_NAV_EVENT:
	    gf_debug(ERR_LOG, "nav event");
		if (copy_from_user(&nav_event, (gf_nav_event_t *)arg, sizeof(gf_nav_event_t))) {
			gf_debug(ERR_LOG, "Failed to copy nav event from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		switch (nav_event) {
		case GF_NAV_FINGER_DOWN:
			gf_debug(ERR_LOG, "nav finger down");
			break;

		case GF_NAV_FINGER_UP:
			gf_debug(ERR_LOG, "nav finger up");
			break;

		case GF_NAV_DOWN:
			nav_input = GF_NAV_INPUT_DOWN;
			gf_debug(ERR_LOG, "nav down");
			break;

		case GF_NAV_UP:
			nav_input = GF_NAV_INPUT_UP;
			gf_debug(ERR_LOG, "nav up");
			break;

		case GF_NAV_LEFT:
			nav_input = GF_NAV_INPUT_LEFT;
			gf_debug(ERR_LOG, "nav left");
			break;

		case GF_NAV_RIGHT:
			nav_input = GF_NAV_INPUT_RIGHT;
			gf_debug(ERR_LOG, "nav right");
			break;

		case GF_NAV_CLICK:
			nav_input = GF_NAV_INPUT_CLICK;
			gf_debug(ERR_LOG, "nav click");
			break;

		case GF_NAV_HEAVY:
			nav_input = GF_NAV_INPUT_HEAVY;
			break;

		case GF_NAV_LONG_PRESS:
			nav_input = GF_NAV_INPUT_LONG_PRESS;
			break;

		case GF_NAV_DOUBLE_CLICK:
			nav_input = GF_NAV_INPUT_DOUBLE_CLICK;
			break;

		default:
			gf_debug(INFO_LOG, "%s: not support nav event nav_event: %d ======\n", __func__, nav_event);
			break;
		}

		if ((nav_event != GF_NAV_FINGER_DOWN) && (nav_event != GF_NAV_FINGER_UP)) {
		    input_report_key(gf_dev->input, nav_input, 1);
		    input_sync(gf_dev->input);
		    input_report_key(gf_dev->input, nav_input, 0);
		    input_sync(gf_dev->input);
		}
		break;

	case GF_IOC_ENTER_SLEEP_MODE:
		gf_debug(INFO_LOG, "%s: GF_IOC_ENTER_SLEEP_MODE ======\n", __func__);
		break;

	case GF_IOC_GET_FW_INFO:
		gf_debug(INFO_LOG, "%s: GF_IOC_GET_FW_INFO ======\n", __func__);
		buf = gf_dev->need_update;

		gf_debug(DEBUG_LOG, "%s: firmware info  0x%x\n", __func__, buf);
		if (copy_to_user((void __user *)arg, (void *)&buf, sizeof(u8))) {
			gf_debug(ERR_LOG, "Failed to copy data to user\n");
			retval = -EFAULT;
		}

		break;
	case GF_IOC_REMOVE:
		gf_debug(INFO_LOG, "%s: GF_IOC_REMOVE ======\n", __func__);

		gf_netlink_destroy(gf_dev);

		mutex_lock(&gf_dev->release_lock);
		if (gf_dev->input == NULL) {
			mutex_unlock(&gf_dev->release_lock);
			break;
		}
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		cdev_del(&gf_dev->cdev);
		//sysfs_remove_group(&gf_dev->device->kobj, &gf_debug_attr_group);
		device_destroy(gf_dev->class, gf_dev->devno);
		list_del(&gf_dev->device_entry);
		unregister_chrdev_region(gf_dev->devno, 1);
		class_destroy(gf_dev->class);
		//gf_hw_power_enable(gf_dev, 0);

		gf_cleanup(gf_dev);
		gf_dev->spi = NULL;
		mutex_unlock(&gf_dev->release_lock);

		//spi_set_drvdata(gf_dev->spi, NULL);
		//gf_dev->spi = NULL;
		mutex_destroy(&gf_dev->buf_lock);
		mutex_destroy(&gf_dev->release_lock);

		break;

	default:
		gf_debug(ERR_LOG, "gf doesn't support this command(%x)\n", cmd);
		break;
	}

	FUNC_EXIT();
	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	FUNC_ENTRY();

	retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);

	FUNC_EXIT();
	return retval;
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
	gf_debug(ERR_LOG, "Not support poll opertion in TEE version\n");
	return -EFAULT;
}


/* -------------------------------------------------------------------- */
/* devfs                                                              */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	gf_debug(INFO_LOG, "%s: Show debug_level = 0x%x\n", __func__, g_debug_level);
	return sprintf(buf, "vendor id 0x%x\n", g_vendor_id);
}

static ssize_t gf_debug_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	struct gf_device *gf_dev =  dev_get_drvdata(dev);
	//u8 flag = 0;

	if (!strncmp(buf, "-8", 2)) {
		gf_debug(INFO_LOG, "%s: parameter is -8, lcd on notify===============\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
	} else if (!strncmp(buf, "-9", 2)) {
		gf_debug(INFO_LOG, "%s: parameter is -9, lcd off notify===============\n", __func__);
		gf_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);

	}else {
		gf_debug(ERR_LOG, "%s: wrong parameter!===============\n", __func__);
	}

	return count;
}

/* -------------------------------------------------------------------- */
/* device function								  */
/* -------------------------------------------------------------------- */
static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devno == inode->i_rdev) {
			gf_debug(INFO_LOG, "%s, Found\n", __func__);
			status = 0;
			break;
		}
	}
	mutex_unlock(&device_list_lock);
	status = gf_parse_dts(gf_dev);
	if (status){
		gf_debug(INFO_LOG, "%s,fail gf_parse_dts", __func__);
	}

	//gf_get_gpio_dts_info(gf_dev);
	/*if(!IS_ERR(gf_dev->pinstate_spi_func)) {
		pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pinstate_spi_func);
	}*/
	if (status == 0) {
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		gf_debug(INFO_LOG, "%s, Success to open device. irq = %d\n", __func__, gf_dev->irq);
	} else {
		gf_debug(ERR_LOG, "%s, No device for minor %d\n", __func__, iminor(inode));
	}
	FUNC_EXIT();
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int    status = 0;

	FUNC_ENTRY();
	gf_dev = filp->private_data;
	if (gf_dev->irq)
		gf_disable_irq(gf_dev);
	gf_dev->need_update = 0;
	FUNC_EXIT();
	return status;
}


static const struct file_operations gf_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.	It'll simplify things
	* too, except for the locking.
	*/
	.write =	gf_write,
	.read =		gf_read,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif
	.open =		gf_open,
	.release =	gf_release,
	.poll	= gf_poll,
};

/*-------------------------------------------------------------------------*/
static int gf_platform_probe(struct platform_device *pldev)
{
	int status = -EINVAL;
	struct gf_device *gf_dev = NULL;
	struct device *dev = &pldev->dev;

	FUNC_ENTRY();

	/* Allocate driver data */
	gf_dev = kzalloc(sizeof(struct gf_device), GFP_KERNEL);
	if (!gf_dev) {
		status = -ENOMEM;
		goto err;
	}

	mutex_init(&gf_dev->release_lock);

	INIT_LIST_HEAD(&gf_dev->device_entry);

	gf_dev->device_count     = 0;
	gf_dev->probe_finish     = 0;
	gf_dev->system_status    = 0;
	gf_dev->need_update      = 0;

	/*setup gf configurations.*/
	gf_debug(INFO_LOG, "%s, Setting gf device configuration==========\n", __func__);

	dev_set_drvdata(dev, gf_dev);
	gf_dev->pldev = pldev;
	gf_dev->device = dev;
#ifdef USE_REGULATOR
	gf_dev->pdev = dev;
#endif
	/* get gpio info from dts or defination */
	//gf_get_gpio_dts_info(gf_dev);
	gf_get_sensor_dts_info();

	/*enable the power*/
	//gf_hw_power_enable(gf_dev, 1);
	gf_bypass_flash_gpio_cfg();

	/* check firmware Integrity */
	//gf_debug(INFO_LOG, "%s, Sensor type : %s.\n", __func__, CONFIG_GOODIX_SENSOR_TYPE);

	/* create class */
	gf_dev->class = class_create(THIS_MODULE, GF_CLASS_NAME);
	if (IS_ERR(gf_dev->class)) {
		gf_debug(ERR_LOG, "%s, Failed to create class.\n", __func__);
		status = -ENODEV;
		goto err_class;
	}

	/* get device no */
	if (GF_DEV_MAJOR > 0) {
		gf_dev->devno = MKDEV(GF_DEV_MAJOR, gf_dev->device_count++);
		status = register_chrdev_region(gf_dev->devno, 1, GF_DEV_NAME);
	} else {
		status = alloc_chrdev_region(&gf_dev->devno, gf_dev->device_count++, 1, GF_DEV_NAME);
	}
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed to alloc devno.\n", __func__);
		goto err_devno;
	} else {
		gf_debug(INFO_LOG, "%s, major=%d, minor=%d\n", __func__, MAJOR(gf_dev->devno), MINOR(gf_dev->devno));
	}

	/* create device */
	gf_dev->device = device_create(gf_dev->class, &pldev->dev, gf_dev->devno, gf_dev, GF_DEV_NAME);
	if (IS_ERR(gf_dev->device)) {
		gf_debug(ERR_LOG, "%s, Failed to create device.\n", __func__);
		status = -ENODEV;
		goto err_device;
	} else {
		mutex_lock(&device_list_lock);
		list_add(&gf_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
		gf_debug(INFO_LOG, "%s, device create success.\n", __func__);
	}

	/* create sysfs */
	status = sysfs_create_group(&dev->kobj, &gf_debug_attr_group);
	if (status) {
		gf_debug(ERR_LOG, "%s, Failed to create sysfs file.\n", __func__);
		status = -ENODEV;
		goto err_sysfs;
	} else {
		gf_debug(INFO_LOG, "%s, Success create sysfs file.\n", __func__);
	}

	/* cdev init and add */
	cdev_init(&gf_dev->cdev, &gf_fops);
	gf_dev->cdev.owner = THIS_MODULE;
	status = cdev_add(&gf_dev->cdev, gf_dev->devno, 1);
	if (status) {
		gf_debug(ERR_LOG, "%s, Failed to add cdev.\n", __func__);
		goto err_cdev;
	}

	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if (gf_dev->input == NULL) {
		gf_debug(ERR_LOG, "%s, Failed to allocate input device.\n", __func__);
		status = -ENOMEM;
		goto err_input;
	}

	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(GF_KEY_INPUT_HOME, gf_dev->input->keybit);

	__set_bit(GF_KEY_INPUT_MENU, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_BACK, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_POWER, gf_dev->input->keybit);

	__set_bit(GF_NAV_INPUT_UP, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_DOWN, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_RIGHT, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_LEFT, gf_dev->input->keybit);
	__set_bit(GF_KEY_INPUT_CAMERA, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_CLICK, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_DOUBLE_CLICK, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_LONG_PRESS, gf_dev->input->keybit);
	__set_bit(GF_NAV_INPUT_HEAVY, gf_dev->input->keybit);

	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input)) {
		gf_debug(ERR_LOG, "%s, Failed to register input device.\n", __func__);
		status = -ENODEV;
		goto err_input_2;
	}

	/* netlink interface init */
	status = gf_netlink_init(gf_dev);
	if (status == -1) {
		mutex_lock(&gf_dev->release_lock);
		input_unregister_device(gf_dev->input);
		gf_dev->input = NULL;
		mutex_unlock(&gf_dev->release_lock);
		goto err_input;
	}
	//wakeup_source_add(&gf_dev->fp_wakesrc);

	gf_dev->probe_finish = 1;
	gf_dev->is_sleep_mode = 0;
	sprintf(gf_dev->fpname,"%s","NULL");
	gf_debug(INFO_LOG, "%s probe finished\n", __func__);
	ggf_dev = gf_dev;
	FUNC_EXIT();
	return 0;

err_input_2:
	mutex_lock(&gf_dev->release_lock);
	input_free_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);

err_input:
	cdev_del(&gf_dev->cdev);

err_cdev:
	sysfs_remove_group(&dev->kobj, &gf_debug_attr_group);

err_sysfs:
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

err_device:
	unregister_chrdev_region(gf_dev->devno, 1);

err_devno:
	class_destroy(gf_dev->class);

err_class:
	gf_hw_power_enable(gf_dev, 0);
	mutex_destroy(&gf_dev->release_lock);
	dev_set_drvdata(dev, NULL);
	kfree(gf_dev);
	gf_dev = NULL;
err:

	FUNC_EXIT();
	return status;
}

static int gf_platform_remove(struct platform_device *pldev)
{
	struct device *dev = &pldev->dev;
	struct gf_device *gf_dev = dev_get_drvdata(dev);

	FUNC_ENTRY();

	//wakeup_source_remove(&gf_dev->fp_wakesrc);
	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
		gf_dev->irq_count = 0;
		gf_dev->irq = 0;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (gf_dev->early_suspend.suspend)
		unregister_early_suspend(&gf_dev->early_suspend);
#else
	fb_unregister_client(&gf_dev->notifier);
#endif

	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->input == NULL) {
		kfree(gf_dev);
		mutex_unlock(&gf_dev->release_lock);
		FUNC_EXIT();
		return 0;
	}
	input_unregister_device(gf_dev->input);
	gf_dev->input = NULL;
	mutex_unlock(&gf_dev->release_lock);

	mutex_lock(&gf_dev->release_lock);

	mutex_unlock(&gf_dev->release_lock);

	gf_netlink_destroy(gf_dev);
	cdev_del(&gf_dev->cdev);
	sysfs_remove_group(&pldev->dev.kobj, &gf_debug_attr_group);
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

	unregister_chrdev_region(gf_dev->devno, 1);
	class_destroy(gf_dev->class);
	gf_hw_power_enable(gf_dev, 0);

	dev_set_drvdata(dev, NULL);
	mutex_destroy(&gf_dev->release_lock);

	kfree(gf_dev);
	FUNC_EXIT();
	return 0;
}

/*-------------------------------------------------------------------------*/
static struct platform_driver goodix_fp_driver = {
	.driver = {
		   .name = "goodix_fp",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gf_of_match,
#endif
		   },
	.probe = gf_platform_probe,
	.remove = gf_platform_remove
};

static int __init gf_init(void)
{
	int status = 0;

	FUNC_ENTRY();
	pr_err("%s %d\n", __func__, __LINE__);

	status = platform_driver_register(&goodix_fp_driver);
	if (status < 0) {
		gf_debug(ERR_LOG, "%s, Failed to register goodix_fp driver.\n", __func__);
		return -EINVAL;
	}

	FUNC_EXIT();
	return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	FUNC_ENTRY();
	platform_driver_unregister(&goodix_fp_driver);

	FUNC_EXIT();
}
module_exit(gf_exit);


MODULE_AUTHOR("goodix");
MODULE_DESCRIPTION("Goodix Fingerprint chip GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206/GF5216/GF5208/GF3626ZS2 TEE driver");
MODULE_LICENSE("GPL");
