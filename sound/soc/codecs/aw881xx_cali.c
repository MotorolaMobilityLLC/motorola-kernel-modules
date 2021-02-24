/*
 * aw881xx_cali.c cali_module
 *
 * Version: v0.2.0
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include "aw881xx.h"
#include "aw881xx_reg.h"
#include "aw881xx_cali.h"
#include "aw881xx_monitor.h"

/******************************************************
 *
 * aw881xx cali store
 *
 ******************************************************/

/*write cali to persist file example */
#ifdef AW_CALI_STORE_EXAMPLE

#define AWINIC_CALI_FILE  "/mnt/vendor/persist/factory/audio/aw_cali.bin"
#define AW_INT_DEC_DIGIT 10

static int aw881xx_write_cali_re_to_file(uint32_t cali_re, uint8_t channel)
{
	struct file *fp = NULL;
	char buf[50] = { 0 };
	loff_t pos = 0;
	mm_segment_t fs;

	pos = channel * AW_INT_DEC_DIGIT;

	fp = filp_open(AWINIC_CALI_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			__func__, channel, AWINIC_CALI_FILE);
		return -EINVAL;
	}
	cali_re = ((cali_re * 1000) >> 12);
	snprintf(buf, sizeof(buf), "%10u", cali_re);

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, strlen(buf), &pos);

	set_fs(fs);

	pr_info("%s: channel:%d  buf:%s cali_re:%d\n",
		__func__, channel, buf, cali_re);

	filp_close(fp, NULL);
	return 0;
}

static int aw881xx_get_cali_re_from_file(uint32_t *cali_re, uint8_t channel)
{
	struct file *fp = NULL;
	int f_size;
	char *buf = NULL;
	int32_t int_cali_re = 0;
	loff_t pos = 0;
	mm_segment_t fs;

	pos = channel * AW_INT_DEC_DIGIT;

	fp = filp_open(AWINIC_CALI_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s:channel:%d open %s failed!\n",
			__func__, channel, AWINIC_CALI_FILE);
		return -EINVAL;
	}

	f_size = AW_INT_DEC_DIGIT;

	buf = kzalloc(f_size + 1, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: channel:%d malloc mem %d failed!\n",
			__func__, channel, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, f_size, &pos);

	set_fs(fs);

	if (sscanf(buf, "%d", &int_cali_re) == 1)
		*cali_re = ((int_cali_re << 12) / 1000);
	else
		*cali_re = AW_ERRO_CALI_VALUE;

	pr_info("%s: channel:%d buf:%s int_cali_re: %d\n",
		__func__, channel, buf, int_cali_re);

	kfree(buf);
	buf = NULL;
	filp_close(fp, NULL);

	return 0;

}
#endif


static int aw881xx_get_cali_re_from_phone(struct aw881xx *aw881xx)
{
	/* customer add, get re from nv or persist or cali file */
#ifdef AW_CALI_STORE_EXAMPLE
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;

	return aw881xx_get_cali_re_from_file(&cali_attr->cali_re,
					aw881xx->channel);
#else
	return -EBUSY;
#endif
}

void aw881xx_get_cali_re(struct aw881xx_cali_attr *cali_attr)
{
	int ret;
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);

	ret = aw881xx_get_cali_re_from_phone(aw881xx);
	if (ret < 0) {
		cali_attr->cali_re = AW_ERRO_CALI_VALUE;
		aw_dev_err(aw881xx->dev, "%s: get re failed, use default\n", __func__);
	}
}

static int aw881xx_set_cali_re_to_phone(struct aw881xx *aw881xx)
{
	/* customer add, set re to nv or persist or cali file */
#ifdef AW_CALI_STORE_EXAMPLE
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;

	return aw881xx_write_cali_re_to_file(cali_attr->cali_re,
					aw881xx->channel);
#else
	return -EBUSY;
#endif
}

void aw881xx_set_cali_re_to_dsp(struct aw881xx_cali_attr *cali_attr)
{
	uint16_t cali_re = 0;
	uint16_t dsp_ra = 0;
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);

	aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RA, &dsp_ra);

	cali_re = cali_attr->cali_re + dsp_ra;

	/* set cali re to aw881xx */
	aw881xx_dsp_write(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RE,
			cali_re);

}

static int aw881xx_read_cali_re_from_dsp(struct aw881xx *aw881xx, uint32_t *read_re)
{
	uint16_t dsp_re = 0;
	uint16_t dsp_ra = 0;
	int ret;

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RA, &dsp_ra);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read ra fail\n", __func__);
		return ret;
	}

	ret = aw881xx_dsp_read(aw881xx, AW881XX_DSP_REG_CFG_ADPZ_RE, &dsp_re);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read re fail\n", __func__);
		return ret;
	}

	*read_re = dsp_re - dsp_ra;

	return 0;
}

static int aw881xx_set_cali_re(struct aw881xx_cali_attr *cali_attr)
{
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);
	int ret;

	ret = aw881xx_set_cali_re_to_phone(aw881xx);
	if (ret < 0)
		return ret;

	/* set cali re to aw881xx */
	aw881xx_set_cali_re_to_dsp(cali_attr);

	return 0;
}

static ssize_t aw881xx_cali_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->cali_re = databuf[0] * (1 << AW881XX_DSP_RE_SHIFT) / 1000;
	ret = aw881xx_set_cali_re(cali_attr);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t aw881xx_cali_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cali_re=%umohm\n",
			(uint32_t) (cali_attr->cali_re * 1000) /
			(1 << AW881XX_DSP_RE_SHIFT));

	return len;
}

static ssize_t aw881xx_re_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->re = databuf[0];

	return count;
}

static ssize_t aw881xx_re_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"re=%umohm\n", cali_attr->re);

	return len;
}

static ssize_t aw881xx_f0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->f0 = databuf[0];

	return count;
}

static ssize_t aw881xx_f0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "f0=%uHz\n", cali_attr->f0);

	return len;
}

static ssize_t aw881xx_q_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	int ret = -1;
	unsigned int databuf[2] = { 0 };

	ret = kstrtouint(buf, 0, &databuf[0]);
	if (ret < 0)
		return ret;

	cali_attr->q = databuf[0];

	return count;
}

static ssize_t aw881xx_q_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	struct aw881xx_cali_attr *cali_attr = &aw881xx->cali_attr;
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "q=%u\n", cali_attr->q);

	return len;
}

static ssize_t aw881xx_dsp_re_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw881xx *aw881xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int ret = 0;
	uint32_t read_re;

	aw881xx_get_dsp_config(aw881xx);
	if (aw881xx->dsp_cfg == AW881XX_DSP_BYPASS) {
		len += snprintf((char *)(buf + len), PAGE_SIZE - len,
				"%s: aw881xx dsp bypass\n", __func__);
		return len;
	}

	ret = aw881xx_get_iis_status(aw881xx);
	if (ret < 0) {
		len += snprintf((char *)(buf + len),
				PAGE_SIZE - len,
				"%s: aw881xx no iis signal\n",
				__func__);
		return len;
	}

	ret = aw881xx_read_cali_re_from_dsp(aw881xx, &read_re);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev, "%s:read dsp re fail\n", __func__);
		return ret;
	}

	len += snprintf((char *)(buf + len),
		PAGE_SIZE - len,
		"dsp_re: %d\n", ((read_re * 1000) >> 12));

	return len;
}

static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO,
			aw881xx_cali_show, aw881xx_cali_store);
static DEVICE_ATTR(re, S_IWUSR | S_IRUGO,
			aw881xx_re_show, aw881xx_re_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO,
			aw881xx_f0_show, aw881xx_f0_store);
static DEVICE_ATTR(q, S_IWUSR | S_IRUGO,
			aw881xx_q_show, aw881xx_q_store);
static DEVICE_ATTR(dsp_re, S_IRUGO,
			aw881xx_dsp_re_show, NULL);


static struct attribute *aw881xx_cali_attr[] = {
	&dev_attr_cali.attr,
	&dev_attr_re.attr,
	&dev_attr_f0.attr,
	&dev_attr_q.attr,
	&dev_attr_dsp_re.attr,
	NULL
};

static struct attribute_group aw881xx_cali_attr_group = {
	.attrs = aw881xx_cali_attr
};

void aw881xx_cali_init(struct aw881xx_cali_attr *cali_attr)
{
	int ret = -1;
	struct aw881xx *aw881xx =
		container_of(cali_attr, struct aw881xx, cali_attr);

	ret = sysfs_create_group(&aw881xx->dev->kobj, &aw881xx_cali_attr_group);
	if (ret < 0) {
		aw_dev_err(aw881xx->dev,
			"%s error creating sysfs attr files\n", __func__);
	}
}
