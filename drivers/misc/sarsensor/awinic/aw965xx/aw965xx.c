// SPDX-License-Identifier: GPL-2.0
#include "aw965xx.h"
#include "aw_sar.h"

#define AW965XX_I2C_NAME "aw965xx_sar"
#define AW965XX_DRIVER_VERSION "v0.7.0"

static struct aw_sar *g_p_sar = NULL;

static void aw965xx_set_cs_as_irq(struct aw_sar *p_sar, int flag);

static int32_t aw965xx_load_reg_bin(struct aw_bin *aw_bin, void *load_bin_para)
{
	int32_t ret = 0;
	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;
	struct aw965xx *aw965xx = (struct aw965xx *)p_sar->priv_data;

	AWLOGE(p_sar->dev, "reg chip name: %s, soc chip name: %s, len = %d",
			p_sar->chip_name, aw_bin->header_info[0].chip_type, aw_bin->info.len);

	//ret = strncmp(p_sar->chip_name, aw_bin->header_info[0].chip_type, sizeof(aw_bin->header_info[0].chip_type));
	ret = strncmp(p_sar->chip_name, aw_bin->header_info[0].chip_type, strlen("AW96508"));
	if (ret != 0) {
		AWLOGE(p_sar->dev, "load_binname(%s) incompatible with chip type(%s)",
			p_sar->chip_name, aw_bin->header_info[0].chip_type);
		//return -AW_ERR;
	}

	p_sar->load_bin.bin_data_ver = aw_bin->header_info[0].bin_data_ver;
	AWLOGI(p_sar->dev, "Bin_data_ver = 0x%x", p_sar->load_bin.bin_data_ver);

	aw_sar_i2c_write(p_sar->i2c, REG_ENABLE_RAM, RAM_ACCESS_ENABLE);

	ret = aw_sar_load_reg(aw_bin, p_sar->i2c);

	aw_sar_i2c_write(p_sar->i2c, REG_ENABLE_RAM, RAM_ACCESS_DISABLE);
	aw965xx_set_cs_as_irq(p_sar, aw965xx->irq_mux);

	return ret;
}

static void aw965xx_irq_handle_func(uint32_t irq_status, void *data)
{
	int8_t i = 0;
	int8_t j = 0;
	int32_t ret = 0;
	uint32_t reg_stat0 = 0;
	uint32_t curr_status_val[4] = { 0 };

	struct aw_sar *p_sar = (struct aw_sar *)data;
	uint32_t ch_th[AW965XX_CHANNEL_NUM_MAX] = { 0 };

	AWLOGD(p_sar->dev, "IRQSRC = 0x%x", irq_status);
	p_sar->irq_status = irq_status;

	if (((irq_status & 0x01) == 1) && (p_sar->driver_code_initover_flag == 1)) {
		pr_info("%s not healthy!\n", __func__);
		p_sar->fault_flag = AW_SAR_UNHEALTHY;
	}

	ret = aw_sar_i2c_read(p_sar->i2c, REG_STAT0, &reg_stat0);

	for (i = 0; i < AW965XX_VALID_TH; i++)
		curr_status_val[i] = ((reg_stat0 >> (i * AW_BIT8)) & ONE_WORD);

	for (j = 0; j < AW965XX_CHANNEL_NUM_MAX; j++) {
		if (p_sar->channels_arr[j].input == NULL)
			continue;

		for (i = 0; i < AW965XX_VALID_TH; i++) {
			ch_th[j] |= ((curr_status_val[i] >> j) & 0x01) << i;
			AWLOGD(p_sar->dev, "ch= %d, th = %d ch_th = 0x%x", j, i, ch_th[j]);
		}
		AWLOGD(p_sar->dev, "ch = %d last_th=0x%x th = 0x%x", j, p_sar->channels_arr[j].last_channel_info, ch_th[j]);

		if (p_sar->channels_arr[j].last_channel_info != ch_th[j]) {
			if ((ch_th[j] >> 3 & 0x01) == 1) {	//th3
				input_report_abs(p_sar->channels_arr[j].input, ABS_DISTANCE, 4);
			} else if ((ch_th[j] >> 2 & 0x01) == 1) { //th2
				input_report_abs(p_sar->channels_arr[j].input, ABS_DISTANCE, 3);
			} else if ((ch_th[j] >> 1 & 0x01) == 1) { //th1
				input_report_abs(p_sar->channels_arr[j].input, ABS_DISTANCE, 2);
			} else if ((ch_th[j] >> 0 & 0x01) == 1) { //th0
				input_report_abs(p_sar->channels_arr[j].input, ABS_DISTANCE, 1);
			} else {	//far
				input_report_abs(p_sar->channels_arr[j].input, ABS_DISTANCE, 0);
			}
			input_sync(p_sar->channels_arr[j].input);
			p_sar->channels_arr[j].last_channel_info = ch_th[j];
		}
	}
}

static ssize_t aw965xx_operation_mode_get(void *data, char *buf)
{
	ssize_t len = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	if (p_sar->last_mode == AW965XX_ACTIVE_MODE)
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: Active\n");
	else if (p_sar->last_mode == AW965XX_SLEEP_MODE)
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: Sleep\n");
	else if (p_sar->last_mode == AW965XX_DEEPSLEEP_MODE)
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: DeepSleep\n");
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: Unconfirmed\n");

	return len;
}

static void aw965xx_sar_chip_info_get(void *data, char *buf, ssize_t *p_len)
{
	uint32_t reg_data = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "sar%u, aw965xx chip driver version:%s\n",
				p_sar->dts_info.sar_num, AW965XX_DRIVER_VERSION);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "The driver supports UI\n");

	aw_sar_i2c_read(p_sar->i2c, REG_CHIPID, &reg_data);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "chipid is 0x%08x\n", reg_data);

	aw_sar_i2c_read(p_sar->i2c, REG_IRQEN, &reg_data);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "REG_HOSTIRQEN is 0x%08x\n", reg_data);

	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "aw965xx Bin data version:0x%08x\n",
							p_sar->load_bin.bin_data_ver);
}


static ssize_t aw965xx_get_cap_offset(void *data, char *buf, char *tcmd_buf)
{
	int i = 0;
	uint8_t  mul = 1;
	uint32_t rough = 0;
	uint32_t fine = 0;
	int ret = 0;
	ssize_t len = 0;
	ssize_t tcmd_buf_len = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;
	uint32_t cap_ofst = 0;
	uint32_t send_tcmd_offset = 0;
	uint32_t afe_soft_cfg0_ch[AW965XX_CHANNEL_NUM_MAX] = { 0 };

	for (i = 0; i < AW965XX_CHANNEL_NUM_MAX; i++) {
		ret = aw_sar_i2c_read(p_sar->i2c, REG_AFE_SOFT_CFG0_CH0 + i * (REG_AFE_SOFT_CFG0_CH1 - REG_AFE_SOFT_CFG0_CH0), (uint32_t *)&afe_soft_cfg0_ch[i]);
		if (ret != AW_OK)
			AWLOGE(p_sar->dev, "check REG_AFE_SOFT_CFG0_CH%d error\n", i);

		if (((afe_soft_cfg0_ch[i] >> AW965XX_AFE_CV_MODE_SEL_CH0_START_BIT) & AW965XX_AFE_SOFT_CV_MASK) == AW965XX_AFE_CV_MODE_SEL_CH0_X1NORMAL) {
			mul = 1;
		} else if ((((afe_soft_cfg0_ch[i] >> AW965XX_AFE_CV_MODE_SEL_CH0_START_BIT) & AW965XX_AFE_SOFT_CV_MASK) == AW965XX_AFE_CV_MODE_SEL_CH0_X2VCM)
			&& (((afe_soft_cfg0_ch[i]  >> AW965XX_AFE_CV_VREF_SEL_CH0_START_BIT) & AW965XX_AFE_SOFT_CV_MASK) >= AW965XX_AFE_CV_VREF_SEL_CH0_VREF_IS_0P5VCC_0P5VCC_600PF)) {
			mul = 2;
		} else {
			AWLOGE(p_sar->dev, "%s ch%d: offset configure error !\n",  __func__, i);
			continue;
		}

		rough = ((afe_soft_cfg0_ch[i] >> AW965XX_AFE_CV_OFFSET_C_CH0_START_BIT) & ONE_WORD) * AW965XX_STEP_LEN_UNSIGNED_CAP_ROUGH_ADJ;
		fine = ((afe_soft_cfg0_ch[i] >> AW965XX_AFE_CV_OFFSET_F_CH0_START_BIT) & ONE_WORD) * AW965XX_STEP_LEN_UNSIGNED_CAP_FINE_ADJ;
		cap_ofst = (rough + fine) * mul;
		send_tcmd_offset = cap_ofst ;
		if (buf != NULL) {
			len += snprintf(buf + len, PAGE_SIZE - len,
						"unsigned cap ofst ch%d: %u.%u pf\r\n",
						i,
						cap_ofst / AW965XX_STEP_LEN_UNSIGNED_CAP_ENLARGE,
						cap_ofst % AW965XX_STEP_LEN_UNSIGNED_CAP_ENLARGE);
		}

		if (tcmd_buf != NULL) {
			tcmd_buf[i * 4 + 0] = (uint8_t)((send_tcmd_offset >> 0) & 0xff);
			tcmd_buf[i * 4 + 1] = (uint8_t)((send_tcmd_offset >> 8) & 0xff);
			tcmd_buf[i * 4 + 2] = (uint8_t)((send_tcmd_offset >> 16) & 0xff);
			tcmd_buf[i * 4 + 3] = (uint8_t)((send_tcmd_offset >> 24) & 0xff);
			tcmd_buf_len += 4;
		}
	}

	if(tcmd_buf != NULL) {
		return tcmd_buf_len;
	} else {
		return len;
	} 
}

static ssize_t aw965xx_get_cap_offset_send_to_tcmd(void *data, char *tcmd_buf)
{
	//Note: The format needs to be the same as that of tcmd
	if (tcmd_buf != NULL) {
		return aw965xx_get_cap_offset(data, NULL, tcmd_buf);
	}

	return 0;
}

static ssize_t aw965xx_get_cap_offset_send_to_debug(void *data, char *debug_buf)
{
	//Note: That debugging uses string output
	if (debug_buf != NULL) {
		return aw965xx_get_cap_offset(data, debug_buf, NULL);
	}

	return 0;
}

static void aw965xx_set_cs_as_irq(struct aw_sar *p_sar, int flag)
{
	if (flag == AW965XX_CS4_IRQ) {
		aw_sar_i2c_write(p_sar->i2c, 0xfff4, 0x3c00ffff);
		aw_sar_i2c_write(p_sar->i2c, 0x4024, 0x00000000);
		aw_sar_i2c_write(p_sar->i2c, 0x4028, 0x00000002);
	}  else {
		aw_sar_i2c_write(p_sar->i2c, 0xfff4, 0x3c00ffff);
		aw_sar_i2c_write(p_sar->i2c, 0x4024, 0x00000080);
		aw_sar_i2c_write(p_sar->i2c, 0x4028, 0x00000000);
	}
}

int32_t aw965xx_check_chipid(void *data)
{
	int32_t ret = -AW_ERR;
	uint32_t reg_val = 0;
	uint32_t reg_data = 0;

	struct aw_sar *p_sar = (struct aw_sar *)data;

	if (p_sar == NULL)
		return -AW_BIN_PARA_INVALID;

	ret = aw_sar_i2c_read(p_sar->i2c, REG_EFUSE_WORD, &reg_data);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "check REG_EFUSE_WORD error\n");
		return -AW_ERR;
	}

	if (((reg_data >> AW_EFUSE_DATA_CHECK_BIT) & 0x1) == 0) {
		AWLOGE(p_sar->dev, "efuse program error\n");
		return -AW_ERR;
	}

	ret = aw_sar_i2c_read(p_sar->i2c, REG_CHIPID, &reg_val);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read CHIP ID failed: %d", ret);
		return ret;
	}

	switch (reg_val) {
	case AW965XX_SFR_ID_CHIPID_AW96503CSR:
		AWLOGI(p_sar->dev, "aw96503CSR detected, 0x%04x", reg_val);
		memcpy(p_sar->chip_name, AW96503, 8);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96505CSR:
		AWLOGI(p_sar->dev, "aw96505CSR detected, 0x%04x", reg_val);
		memcpy(p_sar->chip_name, AW96505, 8);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96505DNR:
		AWLOGI(p_sar->dev, "aw96505DNR detected, 0x%04x", reg_val);
		memcpy(p_sar->chip_name, AW96505, 8);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96506QNR:
		AWLOGI(p_sar->dev, "aw96506QNR detected, 0x%04x", reg_val);
		memcpy(p_sar->chip_name, AW96506, 8);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96508QNR:
		AWLOGI(p_sar->dev, "aw96508QNR detected, 0x%04x", reg_val);
		memcpy(p_sar->chip_name, AW96508, 8);
		ret = AW_OK;
		break;
	default:
		AWLOGI(p_sar->dev, "chip id error, 0x%04x", reg_val);
		ret =  -AW_ERR;
		break;
	}

	return ret;
}

/**********************mode operation start*******************************/

static uint32_t aw965xx_rc_irqscr(void *i2c)
{
	uint32_t val = 0;

	aw_sar_i2c_read(i2c, REG_IRQSRC, &val);
	return val;
}

static void aw965xx_set_active_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW965XX_ACTIVE_MODE);
}

static void aw965xx_set_sleep_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW965XX_SLEEP_MODE);
}

static void aw965xx_set_deepsleep_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW965XX_DEEPSLEEP_MODE);
}

static const struct aw_sar_mode_set_t g_aw965xx_mode_set[] = {
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_ACTIVE_MODE,
			.last_mode = AW965XX_DEEPSLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_active_cmd,
		},
	},
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_ACTIVE_MODE,
			.last_mode = AW965XX_SLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_active_cmd,
		},
	},
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_ACTIVE_MODE,
			.last_mode = AW965XX_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_active_cmd,
		},
	},
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_SLEEP_MODE,
			.last_mode = AW965XX_DEEPSLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_sleep_cmd,
		},
	},
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_SLEEP_MODE,
			.last_mode = AW965XX_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_sleep_cmd,
		},
	},
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_DEEPSLEEP_MODE,
			.last_mode = AW965XX_SLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_deepsleep_cmd,
		},
	},
	{
		.chip_id = AW_SAR_NONE_CHECK_CHIP,
		.chip_mode = {
			.curr_mode = AW965XX_DEEPSLEEP_MODE,
			.last_mode = AW965XX_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw965xx_set_deepsleep_cmd,
		},
	},
};


static int32_t aw965xx_parse_dts(void *data)
{
	int32_t val = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw965xx *aw965xx = (struct aw965xx *)p_sar->priv_data;
	struct device_node *np = p_sar->i2c->dev.of_node;

	val = of_property_read_u32(np, "irq-mux", &aw965xx->irq_mux);
	if (val != 0)
		AWLOGE(p_sar->dev, "irq-mux not detected");
	else
		AWLOGI(p_sar->dev, "irq-mux =  %d", aw965xx->irq_mux);

	val = of_property_read_u32(np, "start-mode", &aw965xx->start_mode);
	if (val != 0)
		AWLOGE(p_sar->dev, "start-mode not detected");
	else
		AWLOGI(p_sar->dev, "start-mode =  %d", aw965xx->start_mode);

	return AW_OK;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t name_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#else 
static ssize_t name_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#endif
{
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	AWLOGE(p_sar->dev, "name = capsense%d", p_sar->dts_info.sar_num);
	return snprintf(buf, PAGE_SIZE, "capsense%d\n", p_sar->dts_info.sar_num);
}

static DEVICE_ATTR_RO(name);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#else
static ssize_t reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#endif
{
	u32 temp = 0;
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	if (!strncmp(buf, "cal", 3) ) {
		AWLOGI(p_sar->dev, "capsense_reset_store msg: cal");
		aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff00, 0xff00);
	}

	if (!strncmp(buf, "flip_near", 9)) {
		AWLOGI(p_sar->dev, "capsense_reset_store msg:sar%d flip_near", p_sar->dts_info.sar_num);
		if(p_sar->dts_info.sar_num == 0) {
			aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff00, 0xBf00);
		} else {
			aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff00, 0xff00);
		}
	}

	if (!strncmp(buf, "flip_far", 8)) {
		AWLOGI(p_sar->dev, "capsense_reset_store msg: sar%d flip_far", p_sar->dts_info.sar_num);
		if(p_sar->dts_info.sar_num == 0) {
			aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff00, 0xBf00);
		} else {
			aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff00, 0xff00);
		}
	}

	aw_sar_i2c_read(p_sar->i2c, REG_PST, &temp);
	if (!strncmp(buf, "reset", 5) || !strncmp(buf, "1", 1)) {
		AWLOGI(p_sar->dev, "capsense_reset_store msg: reset");
		if (((temp >> 24) & 0x00000003) == 1) {
			AWLOGE(p_sar->dev, "temp:0X%x", temp);
			aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff, 0xff);
		}
	}

	return count;
}
static DEVICE_ATTR_WO(reset);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t raw_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#else
static ssize_t raw_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#endif
{
	char *p = buf;
	int csx;
	s32 useful, average, diff;
	u32 uData = 0;
	u32 offset = 0;
	char offset_buf[4 * AW965XX_CHANNEL_NUM_MAX] = {0};

	u32 cap_integer;
	u32 cap_fraction;

	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL) {
		pr_err("%s: aw965xx is NULL, no drvdata set for this device\n", __func__);
		return -EINVAL;
	}

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL) {
		pr_err("%s: p_sar is NULL, no drvdata set for this device\n", __func__);
		return -EINVAL;
	}
	if (!p_sar->i2c || !p_sar->dev) {
		pr_err("%s: invalid driver private data member\n", __func__);
		return -EINVAL;
	}

	if(p_sar) {
		for(csx =0; csx<8; csx++) {
			aw_sar_i2c_read(p_sar->i2c, REG_SAT_CH0 + csx*(REG_SAT_CH1 - REG_SAT_CH0), &uData);
			useful = (s32)uData>>10;
			aw_sar_i2c_read(p_sar->i2c, REG_BASELINE_MULTI_CH0 + csx*(REG_BASELINE_MULTI_CH1 - REG_BASELINE_MULTI_CH0), &uData);
			average = (s32)uData>>10;
			aw_sar_i2c_read(p_sar->i2c, REG_DIFF_CH0 + csx*(REG_DIFF_CH1 - REG_DIFF_CH0), &uData);
			diff = (s32)uData>>10;
			aw965xx_get_cap_offset_send_to_tcmd(p_sar, offset_buf);
			offset = offset_buf[csx * 4 + 0] + (offset_buf[csx * 4 + 1]<<8) + (offset_buf[csx * 4 + 2]<<16) + (offset_buf[csx * 4 + 3]<<24);

			cap_integer = offset / AW965XX_STEP_LEN_UNSIGNED_CAP_ENLARGE;
			cap_fraction = (offset * 1000 / AW965XX_STEP_LEN_UNSIGNED_CAP_ENLARGE) % 1000;

			AWLOGE(p_sar->dev, "ph=%d useful=%d average=%d diff=%d offset=%d cap=%u.%03u",
								csx, useful, average, diff, offset, cap_integer, cap_fraction);

			p += snprintf(p, PAGE_SIZE, "PH= %d Useful= %d Average= %d DIFF= %d Offset= %d CAP= %u.%03u\n",
					csx,useful,average,diff,offset, cap_integer, cap_fraction);
		}
	}
	return (p-buf);
}
static DEVICE_ATTR_RO(raw_data);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t register_write_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
#else
static ssize_t register_write_store(struct device *dev,
		struct device_attribute *attr,
		char *buf, size_t count)
#endif
{
	uint16_t regaddr = 0;
	uint32_t val = 0;
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;
	
	if (sscanf(buf, "%hx,%x", &regaddr, &val) != 2)
	{
		AWLOGE(p_sar->dev, "The number of data are wrong\n");
		return -EINVAL;
	}

	aw_sar_i2c_write(p_sar->i2c, regaddr, val);

	return count;
}
static DEVICE_ATTR_WO(register_write);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t register_read_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#else
static ssize_t register_read_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#endif
{
	int temp_regist = 0;
	u32 temp_val = 0;
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	if (sscanf(buf, "%x", &temp_regist) != 1)
	{
		AWLOGE(p_sar->dev, " The number of data are wrong\n");
		return -EINVAL;
	}

	AWLOGE(p_sar->dev, "%d aw_sar_fac_cal_store\n", p_sar->dts_info.sar_num);
	aw_sar_i2c_read(p_sar->i2c, temp_regist, &temp_val);

	AWLOGE(p_sar->dev, "%d Register(0x%2x) data(0x%4x)\n", p_sar->dts_info.sar_num, temp_regist, temp_val);
	return count;
}
static DEVICE_ATTR_WO(register_read);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t fac_irq_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#else
static ssize_t fac_irq_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#endif
{
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	AWLOGE(p_sar->dev, "%d Reading INT line state %d\n",p_sar->dts_info.sar_num, p_sar->irq_status);
	return sprintf(buf, "%d\n",  p_sar->irq_status);
}
static DEVICE_ATTR_RO(fac_irq_state);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t fac_detect_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#else
static ssize_t fac_detect_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#endif
{
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;
	uint32_t reg_val = 0;
	int ret = 0;
	
	if (!aw965xx) {
		pr_err("%s: aw965xx is NULL, no drvdata set for this device\n", __func__);
		return -EINVAL;
	}

	p_sar = aw965xx->p_aw_sar;

	if (!p_sar) {
		pr_err("%s: p_sar is NULL, no drvdata set for this device\n", __func__);
		return -EINVAL;
	}

	if (!p_sar->i2c || !p_sar->dev) {
		pr_err("%s: invalid driver private data member\n", __func__);
		return -EINVAL;
	}

	ret = aw_sar_i2c_read(p_sar->i2c, REG_CHIPID, &reg_val);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read CHIP ID failed: %d", ret);
		return ret;
	}

	switch (reg_val) {
	case AW965XX_SFR_ID_CHIPID_AW96503CSR:
		AWLOGI(p_sar->dev, "aw96503CSR detected, 0x%04x", reg_val);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96505CSR:
		AWLOGI(p_sar->dev, "aw96505CSR detected, 0x%04x", reg_val);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96505DNR:
		AWLOGI(p_sar->dev, "aw96505DNR detected, 0x%04x", reg_val);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96506QNR:
		AWLOGI(p_sar->dev, "aw96506QNR detected, 0x%04x", reg_val);
		ret = AW_OK;
		break;
	case AW965XX_SFR_ID_CHIPID_AW96508QNR:
		AWLOGI(p_sar->dev, "aw96508QNR detected, 0x%04x", reg_val);
		ret = AW_OK;
		break;
	default:
		AWLOGI(p_sar->dev, "chip id error, 0x%04x", reg_val);
		ret =  -AW_ERR;
		break;
	}
	
	if(ret == AW_OK ){
		AWLOGE(p_sar->dev, "Detect ic aw965xx\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", 1);

	}else{

		AWLOGE(p_sar->dev, "Not found ic aw965xx\n");
		return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
}
static DEVICE_ATTR_RO(fac_detect);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t fac_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#else
static ssize_t fac_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#endif

{
	u32 temp = 0;
	int ret = 0;

	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;
	
	if ( !strncmp(buf, "1", 1)) {
		AWLOGI(p_sar->dev, "enable cap sensor\n");
		aw_sar_i2c_read(p_sar->i2c, REG_SCANCTRL0, &temp);
		temp = temp | 0x000000FF;
		AWLOGE(p_sar->dev, "set reg 0x%x val 0x%x\n", REG_SCANCTRL0, temp);
		ret = aw_sar_i2c_write(p_sar->i2c, REG_SCANCTRL0, temp);
		if(ret <0){
			AWLOGE(p_sar->dev, "enable write enable aw965xx error ret =%d\n",ret);
			return -EINVAL;
		}
	}
	if (!strncmp(buf, "0", 1)) {
		AWLOGE(p_sar->dev, "disnable cap sensor\n");
		aw_sar_i2c_read(p_sar->i2c,REG_SCANCTRL0, &temp);
		temp = temp & 0xFFFFFF00;
		AWLOGE(p_sar->dev, "set reg 0x%x val 0x%x\n", REG_SCANCTRL0, temp);
		ret = aw_sar_i2c_write(p_sar->i2c, REG_SCANCTRL0, temp);
		if(ret <0){
			AWLOGE(p_sar->dev, "enable write enable aw965xx error ret =%d\n",ret);
			return -EINVAL;
		}
	}
	return count;
}
static DEVICE_ATTR_WO(fac_enable);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t fac_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#else
static ssize_t fac_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#endif

{
	int ret = 0;
	u32 temp = 0;
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	AWLOGE(p_sar->dev, "%d aw965xx_fac_cal_store\n", p_sar->dts_info.sar_num);
	if ( !strncmp(buf, "1", 1)) {
		temp = temp | 0x0000FF00;
		ret = aw_sar_i2c_write(p_sar->i2c, REG_SCANCTRL0, temp);
		AWLOGE(p_sar->dev, "set reg 0x%x val 0xff00\n", REG_SCANCTRL0);
	}

	return count;
}
static DEVICE_ATTR_WO(fac_cal);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t fac_comp_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#else
static ssize_t fac_comp_show(struct device *dev,,
		struct device_attribute *attr,
		char *buf)
#endif
{
	int j= 0;
	u8 reg_data[AW965XX_CHANNEL_NUM_MAX*4] = {0};
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	AWLOGE(p_sar->dev, "%d aw_sar_fac_comp_show\n", p_sar->dts_info.sar_num);

	aw965xx_get_cap_offset_send_to_tcmd(p_sar, reg_data);

	for(j=0; j < sizeof(reg_data); j++){
		buf[j] = reg_data[j];
	}
	return sizeof(reg_data);
}
static DEVICE_ATTR_RO(fac_comp);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t fac_raw_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
#else
static ssize_t fac_raw_show(struct device *dev,,
		struct device_attribute *attr,
		char *buf)
#endif
{
	u16 reg_addr;
	u32 temp_val;
	int diff_val;
	int read_ret;
	int i;
	u8 data[4 * AW965XX_CHANNEL_NUM_MAX] = {0};
	reg_addr = REG_DIFF_CH0;
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;

	AWLOGE(p_sar->dev, "%d aw_sar_fac_raw_show\n", p_sar->dts_info.sar_num);
	for ( i = 0; i < AW965XX_CHANNEL_NUM_MAX; i++) {
		read_ret=aw_sar_i2c_read(p_sar->i2c, REG_DIFF_CH0 + (REG_DIFF_CH1 - REG_DIFF_CH0) * i, &temp_val);

		if(read_ret<0){
			AWLOGI(p_sar->dev, "failed to read reg data 0x%x", reg_addr);
		}
		AWLOGE(p_sar->dev, "aw965xx i==%d,reg_addr:0x%x",i, reg_addr + 0x4 * i);
		diff_val = ((int)temp_val) >> 10;
		data[4 * i] = (u8)(diff_val >> 24);
		data[1 + 4 * i] = (u8)(diff_val >> 16);
		data[2 + 4 * i] = (u8)(diff_val >> 8);
		data[3 + 4 * i] = (u8)(diff_val);
		AWLOGE(p_sar->dev, "aw965xx diff_val==%x,data[%d]==%x,data[%d]==%x,data[%d]==%x,data[%d]==%x",
				diff_val,
				(4*i), data[4 * i],
				(1+4*i), data[1+4 * i],
				(2+4*i), data[2+4 * i],
				(3+4*i), data[3+4 * i]);
	}
	for(i=0; i<sizeof(data); i++){
		buf[i] = data[i];
	}
	return sizeof(data);
}
static DEVICE_ATTR_RO(fac_raw);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static ssize_t reinitialize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#else
static ssize_t reinitialize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
#endif
{
	struct aw965xx *aw965xx = dev_get_drvdata(dev);
	struct aw_sar *p_sar = NULL;

	if (aw965xx == NULL)
		return 0;

	p_sar = aw965xx->p_aw_sar;
	if (p_sar == NULL)
		return 0;
	AWLOGE(p_sar->dev, "sys/bus/i2c/drivers/aw_sar/x-0012/update");

	return count;
}
static DEVICE_ATTR_WO(reinitialize);

#ifdef USE_SENSORS_CLASS
static int capsensor_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	uint8_t i = 0;
	struct aw965xx *aw965xx = (struct aw965xx *)(sensors_cdev->dev->platform_data);
	struct aw_sar *p_sar = NULL;
	uint8_t set_mode = 0;

	pr_err("%s enter\n",__func__);

	pr_err("%s aw965xx = %p\n",__func__, aw965xx);
	if (aw965xx == NULL) {
		pr_err("%s aw965xx is null\n",__func__);
		return 0;
	}

	p_sar = aw965xx->p_aw_sar;
	pr_err("%s p_sar = %p\n",__func__, p_sar);
	if (p_sar == NULL) {
		pr_err("%s p_sar is null\n",__func__);
		return 0;
	}

	pr_err("%s enable %d\n", __func__, enable);

	for (i = 0; i < AW965XX_CHANNEL_NUM_MAX; i++) {
		pr_err("%s sar-num=%d, channel %d: used=%d, input=%p, name=%s\n",__func__, p_sar->dts_info.sar_num, i,
				p_sar->channels_arr[i].used, p_sar->channels_arr[i].input, p_sar->channels_arr[i].name);

		if ((p_sar->channels_arr[i].used == AW_FALSE) ||
			(p_sar->channels_arr[i].input == NULL))
				continue;

		if (strcmp(sensors_cdev->name, p_sar->channels_arr[i].name) == 0) {
			if (enable == 0x01) {
				pr_err("%s enable cap sensor[%d] : %s\n", __func__, i, sensors_cdev->name);
				if (aw965xx->sar_cali_flag ==  false) {
					aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~0xff, 0xff);
					set_mode = AW965XX_ACTIVE_MODE;
					aw_sar_mode_set(p_sar, set_mode);
					aw965xx->sar_cali_flag = true;
				}
				input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, 0);
				input_sync(p_sar->channels_arr[i].input);
			} else if (enable == 0) {
				pr_err("%s disable cap sensor[%d] : %s\n", __func__, i, sensors_cdev->name);
				if (aw965xx->sar_cali_flag == true) {
					set_mode = AW965XX_SLEEP_MODE;
					aw_sar_mode_set(p_sar, set_mode);
					aw965xx->sar_cali_flag = false;
				}
				input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, -1);
				input_sync(p_sar->channels_arr[i].input);
				p_sar->channels_arr[i].last_channel_info = -1;
			} else {
				AWLOGE(p_sar->dev, "unknown enable symbol");
			}
			break;
		}
	}

	AWLOGE(p_sar->dev, "enable over %d", enable);

	return 0;
}
#endif
//moto_customization
#ifdef USE_SENSORS_CLASS
static struct class *g_capsense_class_ptr = NULL;
// recorde tow devices capsense0 and capsense1
static struct device *g_capsense_devs[2] = {NULL, NULL};
static int g_capsense_refcount = 0;

static struct attribute *capsense_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_reset.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_register_write.attr,
	&dev_attr_register_read.attr,
	&dev_attr_fac_irq_state.attr,
	&dev_attr_fac_detect.attr,
	&dev_attr_fac_enable.attr,
	&dev_attr_fac_cal.attr,
	&dev_attr_fac_comp.attr,
	&dev_attr_fac_raw.attr,
	&dev_attr_reinitialize.attr,
	NULL,
};
ATTRIBUTE_GROUPS(capsense);

static struct class g_capsense_class = {
	.name = "capsense",
	.dev_groups = capsense_groups,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0))
	.owner = THIS_MODULE,
#endif
};

static const char *g_aw965xx_ch_name[] = {
	"Moto CapSense Ch0", "Moto CapSense Ch1", "Moto CapSense Ch2",
	"Moto CapSense Ch3", "Moto CapSense Ch4", "Moto CapSense Ch5",
	"Moto CapSense Ch6", "Moto CapSense Ch7", "Moto CapSense Ch8",
	"Moto CapSense Ch9", "Moto CapSense Ch10", "Moto CapSense Ch11",
	"Moto CapSense Ch12", "Moto CapSense Ch13", "Moto CapSense Ch14"
};
#endif
static int32_t aw_sar_custom_flie_node_create(void *data)
{
	struct aw_sar *p_sar = NULL;
	int32_t ret = 0;
	struct aw965xx *aw965xx = NULL;
#ifdef USE_SENSORS_CLASS
	int i = 0;
#endif

	int dev_idx = 0;

	if (data == NULL) {
		return -1;
	}

	p_sar = (struct aw_sar *)data;
	aw965xx = (struct aw965xx *)p_sar->priv_data;

	dev_idx = p_sar->dts_info.sar_num;
	if (dev_idx < 0 || dev_idx > 1) {
		AWLOGE(p_sar->dev, "Invalid sar_num: %d\n", dev_idx);
		return -EINVAL;
	}

	AWLOGD(p_sar->dev, "aw_sar_custom_flie_node for capsense%d", dev_idx);
	// check if the class node is avaliable
	if (!g_capsense_class_ptr) {
		ret = class_register(&g_capsense_class);
		if (ret < 0) {
			AWLOGE(p_sar->dev, "Create fsys class failed (%d)\n", ret);
			return ret;
		}

		g_capsense_class_ptr = &g_capsense_class;
		AWLOGD(p_sar->dev, "Create fsys class success");
	}

	if (!g_capsense_devs[dev_idx]) {
		g_capsense_devs[dev_idx] = device_create(g_capsense_class_ptr,
													NULL,
													0,
													aw965xx,
													"capsense%d",
													dev_idx);
		if (IS_ERR_OR_NULL(g_capsense_devs[dev_idx])) {
			ret = g_capsense_devs[dev_idx] ? PTR_ERR(g_capsense_devs[dev_idx]) : -ENODEV;
			AWLOGE(p_sar->dev, "Failed to create capsense%d dev: %d\n", dev_idx, ret);
			g_capsense_devs[dev_idx] = NULL;
			return ret;
		}
		g_capsense_refcount++;
		AWLOGD(p_sar->dev, "Create capsense%d dev success", dev_idx);
	}

	struct device *curr_dev = g_capsense_devs[dev_idx];

	if (curr_dev && aw965xx) {
		if (!IS_ERR_OR_NULL(curr_dev)) {
			dev_set_drvdata(curr_dev, aw965xx);
			AWLOGI(p_sar->dev, "Set drvdata for capsense%d\n", dev_idx);
		} else {
			AWLOGE(p_sar->dev, "Failed to get drvdata for capsense%d\n", dev_idx);
		}
	} else {
		pr_err("[aw965xx] %s: curr_dev=%p, aw965xx=%p, invalid pointers!\n", __func__, curr_dev, aw965xx);
	}

	if (dev_get_drvdata(curr_dev) != aw965xx) {
		AWLOGE(p_sar->dev, "Failed to set drvdata for capsense%d\n", dev_idx);
		return -EINVAL;
	}
#if 0
	ret = device_create_file(curr_dev, &dev_attr_name);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create name file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_reset);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create rest file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_raw_data);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create raw_data file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_register_write);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create register_write file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_register_read);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create register_read file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_fac_irq_state);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create fac_irq_state file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_fac_detect);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create fac_detect file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_fac_enable);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create fac_enable file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_fac_cal);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create fac_cal file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_fac_comp);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create fac_comp file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_fac_raw);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create fac_raw file failed (%d)\n", ret);
		return ret;
	}

	ret = device_create_file(curr_dev, &dev_attr_reinitialize);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "Create reinitialize file failed (%d)\n", ret);
		return ret;
	}
#endif

#ifdef USE_SENSORS_CLASS
	for (i = 0; i < AW965XX_CHANNEL_NUM_MAX; i++) {
		if ((p_sar->channels_arr[i].used == AW_FALSE) ||
			(p_sar->channels_arr[i].input == NULL)) {
			continue;
		}

		aw965xx->sensors_capsensor_chs[i].sensors_enable = capsensor_set_enable;
		aw965xx->sensors_capsensor_chs[i].sensors_poll_delay = NULL;
		aw965xx->sensors_capsensor_chs[i].name = g_aw965xx_ch_name[i];
		aw965xx->sensors_capsensor_chs[i].vendor = "awinic";
		aw965xx->sensors_capsensor_chs[i].version = 1;
		aw965xx->sensors_capsensor_chs[i].type = SENSOR_TYPE_MOTO_CAPSENSE;
		aw965xx->sensors_capsensor_chs[i].max_range = "5";
		aw965xx->sensors_capsensor_chs[i].resolution = "5.0";
		aw965xx->sensors_capsensor_chs[i].sensor_power = "0.1";
		aw965xx->sensors_capsensor_chs[i].min_delay = 0;
		aw965xx->sensors_capsensor_chs[i].fifo_reserved_event_count = 0;
		aw965xx->sensors_capsensor_chs[i].fifo_max_event_count = 0;
		aw965xx->sensors_capsensor_chs[i].delay_msec = 100;
		aw965xx->sensors_capsensor_chs[i].enabled = 0;

		struct device *input_dev_parent = &p_sar->channels_arr[i].input->dev;

		AWLOGD(p_sar->dev, "cap sensor_class channel_name:%s", g_aw965xx_ch_name[i]);
		ret = sensors_classdev_register(input_dev_parent, &aw965xx->sensors_capsensor_chs[i]);
		if (ret < 0) {
			AWLOGE(p_sar->dev, "create ch%d cap sensor_class file failed (%d)\n", i, ret);
			pr_err("%s create ch%d cap sensor_class file failed (%d)\n", __func__, i, ret);
			continue;
		} else {
			AWLOGE(p_sar->dev, "create ch%d cap sensor_class file success", i);
			pr_err("%s create ch%d cap sensor_class file success", __func__, i);
		}

		if (aw965xx->sensors_capsensor_chs[i].dev) {
			aw965xx->sensors_capsensor_chs[i].dev->platform_data = aw965xx;
		}
	}
#endif

	return ret;
}

static void aw_sar_custom_flie_node_free(void *data)
{
	struct aw_sar *p_sar = NULL;
	struct aw965xx *aw965xx = NULL;
#ifdef USE_SENSORS_CLASS
	int i = 0;
#endif

	int dev_idx = 0;

	pr_info("%s enter\n",__func__);

	if (data == NULL) {
		return;
	}

	p_sar = (struct aw_sar *)data;
	aw965xx = (struct aw965xx *)p_sar->priv_data;
	dev_idx = p_sar->dts_info.sar_num;

#ifdef USE_SENSORS_CLASS

	for (i = 0; i < AW965XX_CHANNEL_NUM_MAX; i++){
		if ((p_sar->channels_arr[i].used == AW_FALSE) ||
			(p_sar->channels_arr[i].input == NULL)) {
			continue;
		}
		sensors_classdev_unregister(&aw965xx->sensors_capsensor_chs[i]);
	}
#endif

	if (g_capsense_devs[dev_idx]) {
		device_destroy(g_capsense_class_ptr, g_capsense_devs[dev_idx]->devt);
		g_capsense_devs[dev_idx] = NULL;
		g_capsense_refcount--;
	}

	if(g_capsense_class_ptr && g_capsense_refcount <= 0){
		class_unregister(g_capsense_class_ptr);
		g_capsense_class_ptr = NULL;
	}

}

static const struct aw_sar_mode_t g_aw965xx_mode = {
	.mode_set_arr = &g_aw965xx_mode_set[0],
	.mode_set_arr_len = ARRAY_SIZE(g_aw965xx_mode_set),
	.p_set_mode_node_fn = NULL,
	.p_get_mode_node_fn = aw965xx_operation_mode_get,
};

static const struct aw_sar_diff_t g_aw965xx_diff = {
	.diff0_reg = REG_DIFF_CH0,
	.diff_step = REG_DIFF_CH1 - REG_DIFF_CH0,
	.rm_float = AW965XX_DATA_PROCESS_FACTOR,
	.p_get_diff_node_fn = NULL,
};

static const struct aw_sar_offset_t g_aw965xx_offset = {
	.p_get_offset_node_fn = aw965xx_get_cap_offset_send_to_debug,
};

static const struct aw_sar_aot_t g_aw965xx_aot = {
	.aot_reg = REG_SCANCTRL0,
	.aot_mask = ~(0xff << 8),
	.aot_flag = 0xff << 8,
};

static const struct aw_sar_para_load_t g_aw965xx_reg_arr_para = {
	.reg_arr = aw965xx_reg_default,
	.reg_arr_len = ARRAY_SIZE(aw965xx_reg_default),
};

static const struct aw_sar_regulator_config_t g_regulator_config = {
	.vcc_name = "vcc",
	.min_uV = AW9620X_SAR_VCC_MIN_UV,
	.max_uV = AW9620X_SAR_VCC_MAX_UV,
};

static const struct aw_sar_reg_list_t g_aw965xx_reg_list = {
	.reg_none_access = REG_NONE_ACCESS,
	.reg_rd_access = REG_RD_ACCESS,
	.reg_wd_access = REG_WR_ACCESS,
	.reg_perm = (struct aw_sar_reg_data *)&g_aw965xx_reg_access[0],
	.reg_num = ARRAY_SIZE(g_aw965xx_reg_access),
};

static const struct aw_sar_chip_mode_t g_aw965xx_chip_mode = {
	.init_mode = AW965XX_ACTIVE_MODE,
	.active = AW965XX_ACTIVE_MODE,
	.pre_init_mode = AW965XX_SLEEP_MODE,
};

static const struct aw_sar_load_bin_t g_aw965xx_load_reg_bin = {
	.bin_name = "aw965xx_reg",
	.bin_opera_func = aw965xx_load_reg_bin,
	.p_update_fn = NULL,
};


static const struct aw_sar_get_chip_info_t g_aw965xx_get_chip_info = {
	.p_get_chip_info_node_fn = aw965xx_sar_chip_info_get,
};

static const struct aw_sar_check_chipid_t g_aw965xx_check_chipid = {
	.p_check_chipid_fn = aw965xx_check_chipid,
};

static const struct aw_sar_irq_init_t g_aw965xx_irq_init = {
	.flags = GPIOD_IN | GPIOD_OUT_HIGH,
	.irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.handler = NULL,
	.thread_fn = NULL,
	.rc_irq_fn = aw965xx_rc_irqscr,
	.irq_spec_handler_fn = aw965xx_irq_handle_func,

	.p_irq_init_fn = NULL,
	.p_irq_deinit_fn = NULL,
};

static const struct aw_sar_soft_rst_t g_aw965xx_soft_rst = {
	.reg_rst = REG_SA_RSTNALL,
	.reg_rst_val = AW965XX_SOFT_RST_EN,
	.delay_ms = AW965XX_CHIP_INIT_MAX_TIME_MS,
	.p_soft_reset_fn = NULL,
};

static const struct aw_sar_init_over_irq_t g_aw965xx_init_over_irq = {
	.wait_times = 100,
	.daley_step = 1,
	.reg_irqsrc = REG_IRQSRC,
	.irq_offset_bit = 0,
	.irq_mask = 0x1,
	.irq_flag = 0x1,

	.p_check_init_over_irq_fn = NULL,
	.p_get_err_type_fn = NULL,
};

static const struct aw_sar_pm_t g_aw965xx_pm_chip_mode = {
	.suspend_set_mode = AW965XX_SLEEP_MODE,
	.resume_set_mode = AW965XX_ACTIVE_MODE,
	.shutdown_set_mode = AW965XX_SLEEP_MODE,
};

static const struct aw_sar_platform_config g_aw965xx_platform_config = {
	.p_add_parse_dts_fn = &aw965xx_parse_dts,
	.p_regulator_config = &g_regulator_config,
	.p_irq_init = &g_aw965xx_irq_init,
	.p_pm_chip_mode = &g_aw965xx_pm_chip_mode,
};

static int32_t aw965xx_effuse_check(void *data)
{
	struct aw_sar *p_sar = (struct aw_sar *)data;
	int32_t ret = 0;
	uint32_t val = 0;

	ret = aw_sar_i2c_read(p_sar->i2c, REG_STAT1, &val);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "sar: %s read stat1 error\n", __func__);
		return -AW_ERR;
	}

	if ((val >> AW_EFUSE_STATUS_CHECK_BIT) & 0x1) {
		AWLOGE(p_sar->dev, "sar: %s effuse load  error\n", __func__);
		return -AW_ERR;
	}
	return ret;
}

static const struct aw_sar_chip_config g_aw965xx_chip_config = {
	.ch_num_max = AW965XX_CHANNEL_NUM_MAX,
	.p_platform_config = &g_aw965xx_platform_config,

	.p_check_chipid = &g_aw965xx_check_chipid,
	.p_soft_rst = &g_aw965xx_soft_rst,
	.p_init_over_irq = &g_aw965xx_init_over_irq,
	.p_fw_bin =	NULL,
	.p_reg_bin = &g_aw965xx_load_reg_bin,
	.p_chip_mode = &g_aw965xx_chip_mode,

	//Node usage parameters
	.p_reg_list = &g_aw965xx_reg_list,
	.p_reg_arr = &g_aw965xx_reg_arr_para,
	.p_aot = &g_aw965xx_aot,
	.p_diff = &g_aw965xx_diff,
	.p_offset = &g_aw965xx_offset,
	.p_mode = &g_aw965xx_mode,
	.p_prox_fw = NULL,
	.p_get_chip_info = &g_aw965xx_get_chip_info,
	.p_aw_sar_awrw = NULL,
	.p_boot_bin = NULL,

	.p_effuse_check = aw965xx_effuse_check,
	.power_on_prox_detection = NULL,

	.p_other_operation = aw_sar_custom_flie_node_create,
	.p_other_opera_free = aw_sar_custom_flie_node_free,
};

int32_t aw965xx_init(struct aw_sar *p_sar)
{
	struct aw965xx *aw965xx = NULL;

	if (p_sar == NULL)
		return -AW_ERR;

	p_sar->priv_data = devm_kzalloc(p_sar->dev, sizeof(struct aw965xx), GFP_KERNEL);
	if (p_sar->priv_data == NULL) {
		AWLOGE(p_sar->dev, "priv_data failed to malloc memory!");
		return -AW_ERR;
	}

	//Chip private function operation
	p_sar->p_sar_para = &g_aw965xx_chip_config;

	aw965xx = (struct aw965xx *)p_sar->priv_data;

	aw965xx->p_aw_sar = p_sar;

	g_p_sar = p_sar;

	return AW_OK;
}

void aw965xx_deinit(struct aw_sar *p_sar)
{
	struct aw965xx *aw965xx = NULL;

	if ((p_sar == NULL) || (p_sar->priv_data == NULL))
		return;

	aw965xx = (struct aw965xx *)p_sar->priv_data;


	if (p_sar->priv_data != NULL)
		devm_kfree(p_sar->dev, p_sar->priv_data);

	AWLOGE(p_sar->dev, "%s ok!", __func__);
}
