#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>	
#include <linux/mman.h>
#include "fshaptic3001_reg.h"
#include "fshaptic3001.h"
#include "fshaptic.h"

static unsigned char ERROR_CODE = 0xFF;
static char* FSERROR = "FSERROR";
static char* FSREAD = "FSREAD";
static char* FSWRITE = "FSWRITE";
static char* FSWRITEBULK = "FSWRITEBULK";
static unsigned char fs3001_haptic_osc_read_status(struct fs3001 *fs3001);
static void fs3001_haptic_diagnostic_sequence(struct fs3001 *fs3001);
static void fs3001_haptic_set_diagnostic_reg_to_default_value(struct fs3001 *fs3001);
static int fs3001_haptic_offset_calibration(struct fs3001 *fs3001);
static void fs3001_haptic_duration_ram_play_config(struct fs3001 *fs3001,int duration);




static char *fs3001_ram_name = "fs3001_haptic.bin";
static char fs3001_rtp_name[][FS3001_RTP_NAME_MAX] = {
	{"fs3001_osc_rtp_12K_10s.bin"},
	{"fs3001_1.bin"},
	{"fs3001_2.bin"},
};

struct pm_qos_request fs3001_pm_qos_req_vb;

static int wf_repeat[8] = { 1, 2, 4, 8, 16, 32, 64, 128 };

struct foursemi *g_foursemi = NULL;

static void pm_qos_enable(bool b_enable)
{

#ifdef FS_KERNEL_VER_OVER_5_10
	if(b_enable)
	{
		if(!cpu_latency_qos_request_active(&fs3001_pm_qos_req_vb))
		{
			cpu_latency_qos_add_request(&fs3001_pm_qos_req_vb, 0);
		}
	}		
	else
	{
		if (cpu_latency_qos_request_active(&fs3001_pm_qos_req_vb))
		{
			cpu_latency_qos_remove_request(&fs3001_pm_qos_req_vb);
		}		
	}
#else
	if(b_enable)
	{
		if(!pm_qos_request_active(&fs3001_pm_qos_req_vb))
		{
			pm_qos_add_request(&fs3001_pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY,FS3001_PM_QOS_VALUE_VB);
		}
	}
	else
	{
		if (pm_qos_request_active(&fs3001_pm_qos_req_vb))
		{
			pm_qos_remove_request(&fs3001_pm_qos_req_vb);
		}		
	}
#endif
}


//fs3001 i2c write/read
static unsigned char pow2(unsigned char value)
{
	unsigned char ret = 1;
	int i=0;

	for(i=0;i<value;i++)
	{
		ret = ret*2;
	}	
	
	return ret;
}

static unsigned char util_get_bit(unsigned char value, int i_pos)
{
	return ((value >> i_pos) & 1);
}

static unsigned char util_get_bits(unsigned char value, int i_start, int i_stop)
{
	if(i_start<i_stop || (i_start-i_stop+1)>8)
	{
		pr_err("%s:i_start=%d,i_stop=%d\n",FSERROR,i_start,i_stop);
		return ERROR_CODE;
	}
	return ((value >> i_stop) & (pow2(i_start - i_stop + 1) - 1));
}

static int fs3001_i2c_write(struct fs3001 *fs3001,unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < FS3001_I2C_RETRIES) 
	{
		ret = i2c_smbus_write_byte_data(fs3001->i2c, reg_addr, reg_data);
		if (ret < 0) 
		{
			pr_err("%s:i2c_write addr=0x%02X, data=0x%02X, cnt=%d, error=%d\n", FSERROR,reg_addr, reg_data, cnt, ret);
		} 
		else 
		{
			if(fs3001->fs3001_debug_enable)
				pr_info("%s,addr=0x%02X, data=0x%02X\n",FSWRITE,reg_addr, reg_data);
			break;
		}
		cnt++;
		usleep_range(FS3001_I2C_RETRY_DELAY * 1000,FS3001_I2C_RETRY_DELAY * 1000 + 500);
	}
	return ret;
}

static int fs3001_i2c_read(struct fs3001 *fs3001,unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < FS3001_I2C_RETRIES) 
	{
		ret = i2c_smbus_read_byte_data(fs3001->i2c, reg_addr);
		if (ret < 0) 
		{
			pr_err("%s:i2c_read addr=0x%02X, cnt=%d error=%d\n", FSERROR,reg_addr, cnt, ret);
		} 
		else 
		{
			*reg_data = ret;
			if(fs3001->fs3001_debug_enable)
				pr_info("%s,addr=0x%02X, data=0x%02X\n",FSREAD,reg_addr, ret);
			break;
		}
		cnt++;
		usleep_range(FS3001_I2C_RETRY_DELAY * 1000,FS3001_I2C_RETRY_DELAY * 1000 + 500);
	}
	return ret;
}

static int fs3001_i2c_writes(struct fs3001 *fs3001,unsigned char reg_addr, unsigned char *buf,unsigned int len)
{
	int ret = -1;
	unsigned char *data = NULL;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) 
	{
		pr_err("%s:can not allocate memory\n",FSERROR);
		return -ENOMEM;
	}
	data[0] = reg_addr;
	memcpy(&data[1], buf, len);
	ret = i2c_master_send(fs3001->i2c, data, len + 1);
	if (ret < 0)
		pr_err("%s:i2c master send error\n",FSERROR);
	else
		pr_info("%s,addr=0x%02X, data=[%d]\n",FSWRITEBULK,reg_addr, len);
	kfree(data);
	return ret;
}

static int fs3001_i2c_write_bits_1(struct fs3001 *fs3001,unsigned char reg_addr,unsigned char reg_data, unsigned char i_start, unsigned char i_stop)
{
	int ret = -1;
	unsigned char reg_val = 0;
	unsigned char t = 0;

	if(i_start<i_stop || (i_start-i_stop+1)>8)
	{
		pr_err("%s:i_start=%d,i_stop=%d\n",FSERROR,i_start,i_stop);
		return ret;
	}

	ret = fs3001_i2c_read(fs3001, reg_addr, &reg_val);
	if (ret < 0) 
	{
		pr_err("%s:ret=%d\n", FSERROR,ret);
		return ret;
	}
	t = ~((pow2(i_start - i_stop + 1) - 1)<<i_stop);
	reg_val = (reg_val & t) | (reg_data << i_stop);
	ret = fs3001_i2c_write(fs3001, reg_addr, reg_val);
	if (ret < 0) 
	{
		pr_err("%s:ret=%d\n", FSERROR,ret);
		return ret;
	}
	return 0;
}

static unsigned char fs3001_i2c_read_bits(struct fs3001 *fs3001,unsigned char reg_addr, int i_start, int i_stop)
{
	int ret = -1;
	unsigned char reg_val = 0;

	if(i_start<i_stop || (i_start-i_stop+1)>8)
	{
		pr_err("%s:i_start=%d,i_stop=%d\n",FSERROR,i_start,i_stop);
		return ERROR_CODE;
	}

	ret = fs3001_i2c_read(fs3001, reg_addr, &reg_val);
	if (ret < 0) 
	{
		return ERROR_CODE;
	}

	return util_get_bits(reg_val,i_start,i_stop);
}

unsigned char fs3001_haptic_rtp_get_fifo_afs(struct fs3001 *fs3001)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;
	
	pr_info("enter\n");
	fs3001_i2c_read(fs3001, FS3001_INTSTAT2, &reg_val);
	reg_val &= FS3001_INTSTAT2_MASK_B1_AF;
	ret = (reg_val >> 1) & 0x1;
	return ret;
}

unsigned char fs3001_haptic_rtp_get_fifo_afs_0xAF(struct fs3001 *fs3001)
{
	unsigned char reg_val = 0;
	
	pr_info("enter\n");
	reg_val = fs3001_i2c_read_bits(fs3001, FS3001_INTSTATR2, 1,1);
	return reg_val;
}



//rtp
void fs3001_haptic_set_rtp_aei(struct fs3001 *fs3001, bool flag)
{
	pr_info("enter flag=%d\n",flag);
	if (flag) 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_INTMASK2, 0, 0, 0);
	}
	else 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_INTMASK2, 1, 0, 0);
	}
}


int fs3001_parse_dt(struct device *dev, struct fs3001 *fs3001,struct device_node *np)
{
	unsigned int val = 0;
	unsigned int reg_inits[10];
	unsigned int duration_time[3];
	unsigned int rtp_time[FS3001_RTP_MAX_DEFAULT];
	struct qti_hap_config *config = &fs3001->config;
	struct device_node *child_node;
	struct qti_hap_effect *effect;
	int rc = 0, tmp = 0, i = 0, j;
	
	pr_info("enter\n");

	//fs3001_f0_ref
	val = of_property_read_u32(np,"fs3001_f0_ref",&fs3001->dts_info.fs3001_f0_ref);
	if (val != 0)
	{
		pr_err("%s:fs3001_f0_ref not found\n", FSERROR);
		fs3001->dts_info.fs3001_f0_ref = FS3001_F0_REF_DEFAULT;
	}
	else
		pr_info("fs3001_f0_ref=%d\n",fs3001->dts_info.fs3001_f0_ref);

    //fs3001_auto_brake
	val = of_property_read_u32(np,"fs3001_auto_brake",&fs3001->dts_info.fs3001_auto_brake);
	if (val != 0)
	{
		pr_err("%s:fs3001_auto_brake not found\n", FSERROR);
		fs3001->dts_info.fs3001_auto_brake = FS3001_AUTO_BRAKE_DISABLE;
	}		
	else
	{
		pr_info("fs3001_auto_brake=%d\n",fs3001->dts_info.fs3001_auto_brake);
	}

	//fs3001_f0_cali_mode
	val = of_property_read_u32(np,"fs3001_f0_cali_mode",&fs3001->dts_info.fs3001_f0_cali_mode);
	if (val != 0)
	{
		pr_err("%s:fs3001_f0_cali_mode not found\n", FSERROR);
		fs3001->dts_info.fs3001_f0_cali_mode = FS3001_F0_CALI_MODE_AUTO;
	}		
	else
		pr_info("fs3001_f0_cali_mode=%d\n",fs3001->dts_info.fs3001_f0_cali_mode);

	//fs3001_cont_drv1_lvl
	val = of_property_read_u32(np, "fs3001_cont_drv1_lvl", &fs3001->dts_info.fs3001_cont_drv1_lvl);
	if (val != 0)
	{
		pr_err("%s:fs3001_cont_drv1_lvl not found\n", FSERROR);
		fs3001->dts_info.fs3001_cont_drv1_lvl = FS3001_CONT_DRV1_LVL_DEFAULT;
	}
	else
		pr_info("fs3001_cont_drv1_lvl=%d\n",fs3001->dts_info.fs3001_cont_drv1_lvl);

	//fs3001_cont_drv2_lvl
	val = of_property_read_u32(np, "fs3001_cont_drv2_lvl",&fs3001->dts_info.fs3001_cont_drv2_lvl);
	if (val != 0)
	{
		pr_err("%s:fs3001_cont_drv2_lvl not found\n", FSERROR);
		fs3001->dts_info.fs3001_cont_drv2_lvl = FS3001_CONT_DRV2_LVL_DEFAULT;
	}
	else
		pr_info("fs3001_cont_drv2_lvl=%d\n",fs3001->dts_info.fs3001_cont_drv2_lvl);

	//fs3001_cont_drv1_time
	val = of_property_read_u32(np, "fs3001_cont_drv1_time",&fs3001->dts_info.fs3001_cont_drv1_time);
	if (val != 0)
	{
		pr_err("%s:fs3001_cont_drv1_time not found\n", FSERROR);
		fs3001->dts_info.fs3001_cont_drv1_time = FS3001_CONT_DRV1_TIME_DEFAULT;
	}
	else
		pr_info("fs3001_cont_drv1_time = %d\n",fs3001->dts_info.fs3001_cont_drv1_time);

	//fs3001_cont_drv2_time
	val = of_property_read_u32(np, "fs3001_cont_drv2_time",&fs3001->dts_info.fs3001_cont_drv2_time);
	if (val != 0)
	{
		pr_err("%s:fs3001_cont_drv2_time not found\n", FSERROR);
		fs3001->dts_info.fs3001_cont_drv2_time = FS3001_CONT_DRV2_TIME_DEFAULT;
	}
	else
		pr_info("fs3001_cont_drv2_time = %d\n",fs3001->dts_info.fs3001_cont_drv2_time);

	//fs3001_cont_1_period
	val = of_property_read_u32(np,"fs3001_cont_1_period",&fs3001->dts_info.fs3001_cont_1_period);
	if (val != 0)
	{
		pr_err("%s:fs3001_cont_1_period not found\n", FSERROR);
		fs3001->dts_info.fs3001_cont_1_period = FS3001_CONT_1_PERIOD_DEFAULT;
	}
	else
		pr_info("fs3001_cont_1_period=%d\n",fs3001->dts_info.fs3001_cont_1_period);

	//fs3001_brk_slopeth
	val = of_property_read_u32(np,"fs3001_brk_slopeth",&fs3001->dts_info.fs3001_brk_slopeth);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_slopeth not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_slopeth = FS3001_BRK_SLOPETH_DEFAULT;
	}
	else
		pr_info("fs3001_brk_slopeth=%d\n",fs3001->dts_info.fs3001_brk_slopeth);

	//fs3001_brk_gain
	val = of_property_read_u32(np, "fs3001_brk_gain", &fs3001->dts_info.fs3001_brk_gain);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_gain not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_gain = FS3001_BRK_GAIN_DEFAULT;
	}
	else
		pr_info("fs3001_brk_gain = %d\n",fs3001->dts_info.fs3001_brk_gain);

	//fs3001_brk_times
	val = of_property_read_u32(np, "fs3001_brk_times",&fs3001->dts_info.fs3001_brk_times);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_times not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_times = FS3001_BRK_TIMES_DEFAULT;
	}		
	else
		pr_info("fs3001_brk_times = %d\n",fs3001->dts_info.fs3001_brk_times);

	//fs3001_brk_noise_gate
	val = of_property_read_u32(np, "fs3001_brk_noise_gate",&fs3001->dts_info.fs3001_brk_noise_gate);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_noise_gate not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_noise_gate = FS3001_BRK_NOISE_GATE_DEFAULT;
	}		
	else
		pr_info("fs3001_brk_noise_gate = %d\n",fs3001->dts_info.fs3001_brk_noise_gate);

	//fs3001_brk_1_period
	val = of_property_read_u32(np, "fs3001_brk_1_period",&fs3001->dts_info.fs3001_brk_1_period);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_1_period not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_1_period = FS3001_BRK_1_PERIOD_DEFAULT;
	}		
	else
		pr_info("fs3001_brk_1_period = %d\n",fs3001->dts_info.fs3001_brk_1_period);

	//fs3001_gain_adjust
	val =of_property_read_u32(np, "fs3001_gain_adjust",&fs3001->dts_info.fs3001_gain_adjust);
	if (val != 0)
	{
		pr_err("%s:fs3001_gain_adjust not found\n", FSERROR);
		fs3001->dts_info.fs3001_gain_adjust = FS3001_GAIN_ADJUST_DEFAULT;
	}	
	else
		pr_info("fs3001_gain_adjust = %d\n",fs3001->dts_info.fs3001_gain_adjust);		

	//fs3001_brk_pgagain
	val =of_property_read_u32(np, "fs3001_brk_pgagain",&fs3001->dts_info.fs3001_brk_pgagain);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_pgagain not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_pgagain = FS3001_BRK_PGAGAIN_DEFAULT;
	}	
	else
		pr_info("fs3001_brk_pgagain = %d\n",fs3001->dts_info.fs3001_brk_pgagain);

	//fs3001_brk_margin
	val =of_property_read_u32(np, "fs3001_brk_margin",&fs3001->dts_info.fs3001_brk_margin);
	if (val != 0)
	{
		pr_err("%s:fs3001_brk_margin not found\n", FSERROR);
		fs3001->dts_info.fs3001_brk_margin = FS3001_BRK_MARGIN_DEFAULT;
	}	
	else
		pr_info("fs3001_brk_margin = %d\n",fs3001->dts_info.fs3001_brk_margin);


	//fs3001_play_ram_srate
	val =of_property_read_u32(np, "fs3001_play_ram_srate",&fs3001->dts_info.fs3001_play_ram_srate);
	if (val != 0)
	{
		pr_err("%s:fs3001_play_ram_srate not found\n", FSERROR);
		fs3001->dts_info.fs3001_play_ram_srate = FS3001_PLAY_RAM_SRATE_DEFAULT;
	}	
	else
		pr_info("fs3001_play_ram_srate = %d\n",fs3001->dts_info.fs3001_play_ram_srate);

	
	//fs3001_play_rtp_srate
	val =of_property_read_u32(np, "fs3001_play_rtp_srate",&fs3001->dts_info.fs3001_play_rtp_srate);
	if (val != 0)
	{
		pr_err("%s:fs3001_play_rtp_srate not found\n", FSERROR);
		fs3001->dts_info.fs3001_play_rtp_srate = FS3001_PLAY_RTP_SRATE_DEFAULT;
	}	
	else
		pr_info("fs3001_play_rtp_srate = %d\n",fs3001->dts_info.fs3001_play_rtp_srate);




	//	fs3001_default_vib_mode
	val = of_property_read_u32(np,"fs3001_default_vib_mode",&fs3001->dts_info.fs3001_default_vib_mode);
	if (val != 0)
	{
		pr_err("%s:fs3001_default_vib_mode not found\n", FSERROR);
		fs3001->dts_info.fs3001_default_vib_mode = FS3001_DEFAULT_VIB_MODE;
	}
	else
		pr_info("fs3001_default_vib_mode=%d\n",fs3001->dts_info.fs3001_default_vib_mode);

	//	fs3001_vbat_mode
	val = of_property_read_u32(np, "fs3001_vbat_mode",&fs3001->dts_info.fs3001_vbat_mode);
	if (val != 0)
	{
		pr_err("%s:fs3001_vbat_mode not found\n", FSERROR);
		fs3001->dts_info.fs3001_vbat_mode = FS3001_VBAT_MODE_DEFAULT;
	}
	else
		pr_info("fs3001_vbat_mode = %d\n",fs3001->dts_info.fs3001_vbat_mode);

	// fs3001_lk_f0_cali
	val = of_property_read_u32(np, "fs3001_lk_f0_cali", &fs3001->dts_info.fs3001_lk_f0_cali);
	if (val != 0)
	{
		pr_err("%s:fs3001_lk_f0_cali not found\n", FSERROR);
		fs3001->dts_info.fs3001_lk_f0_cali = FS3001_LK_F0_CALI_DEFAULT;
	}		
	else
		pr_info("fs3001_lk_f0_cali = 0x%02x\n", fs3001->dts_info.fs3001_lk_f0_cali);

	//	fs3001_bypass_system_gain
	val = of_property_read_u32(np,"fs3001_bypass_system_gain",&fs3001->dts_info.fs3001_bypass_system_gain);
	if (val != 0)
	{
		pr_err("%s:fs3001_bypass_system_gain not found\n", FSERROR);
		fs3001->dts_info.fs3001_bypass_system_gain = FS3001_BYPASS_SYSTEM_GAIN_DEFAULT;
	}
	else
		pr_info("fs3001_bypass_system_gain=%d\n",fs3001->dts_info.fs3001_bypass_system_gain);

	//	fs3001_f0_cali_data_mode
	val = of_property_read_u32(np,"fs3001_f0_cali_data_mode",&fs3001->dts_info.fs3001_f0_cali_data_mode);
	if (val != 0)
	{
		pr_err("%s:fs3001_f0_cali_data_mode not found\n", FSERROR);
		fs3001->dts_info.fs3001_f0_cali_data_mode = FS3001_F0_CALI_DATA_MODE_DEFAULT;
	}
	else
		pr_info("fs3001_f0_cali_data_mode=%d\n",fs3001->dts_info.fs3001_f0_cali_data_mode);	
	
	//	fs3001_lr_pgagain
	val = of_property_read_u32(np,"fs3001_lr_pgagain",&fs3001->dts_info.fs3001_lr_pgagain);
	if (val != 0)
	{
		pr_err("%s:fs3001_lr_pgagain not found\n", FSERROR);
		fs3001->dts_info.fs3001_lr_pgagain = FS3001_LR_PGAGAIN_DEFAULT;
	}
	else
		pr_info("fs3001_lr_pgagain=%d\n",fs3001->dts_info.fs3001_lr_pgagain);	




	//fs3001_duration_time = < 30 60 90>;
	val = of_property_read_u32_array(np, "fs3001_duration_time",duration_time,ARRAY_SIZE(duration_time));
	if (val != 0)
	{
		pr_err("%s:fs3001_duration_time not found\n", FSERROR);
		for (i = 0;i < ARRAY_SIZE(duration_time);i++)
		{
			fs3001->dts_info.fs3001_duration_time[i] = FS3001_DURATION_TIME_DEFAULT *i + FS3001_DURATION_TIME_DEFAULT;
		}
	}
	else
	{
		for (i = 0;i < ARRAY_SIZE(duration_time);i++)
		{
			pr_info("fs3001_duration_time[%d]=0x%x\n",i,duration_time[i]);
		}
	}
	memcpy(fs3001->dts_info.fs3001_duration_time, duration_time,sizeof(duration_time));

	//fs3001_reg_inits = < 0x5f0001 0x5f3411 0x5f7080 >;
	val = of_property_read_u32_array(np, "fs3001_reg_inits",reg_inits,ARRAY_SIZE(reg_inits));
	if (val != 0)
	{
		pr_err("%s:fs3001_reg_inits not found\n", FSERROR);
		for (i = 0;i < ARRAY_SIZE(reg_inits);i++)
		{
			reg_inits[i] = FS3001_REG_INITS_DEFAULT;
		}
	}
	else
	{
		for (i = 0;i < ARRAY_SIZE(reg_inits);i++)
		{
			if(reg_inits[i] != FS3001_REG_INITS_DEFAULT)
			{
				pr_info("reg_inits[%d]=0x%x\n",i,reg_inits[i]);
			}
		}
	}
	memcpy(fs3001->dts_info.fs3001_reg_inits, reg_inits,sizeof(reg_inits));

	//	fs3001_rtp_id_boundary
	val = of_property_read_u32(np, "fs3001_rtp_id_boundary",&fs3001->dts_info.fs3001_rtp_id_boundary);
	if (val != 0)
	{
		pr_err("%s:fs3001_rtp_id_boundary not found\n", FSERROR);
		fs3001->dts_info.fs3001_rtp_id_boundary = FS3001_RTP_ID_BOUNDARY_DEFAULT;
	}
	else
		pr_info("fs3001_rtp_id_boundary = %d\n",fs3001->dts_info.fs3001_rtp_id_boundary);

	//	fs3001_rtp_max
	val = of_property_read_u32(np, "fs3001_rtp_max",&fs3001->dts_info.fs3001_rtp_max);
	if (val != 0)
	{
		pr_err("%s:fs3001_rtp_max not found\n", FSERROR);
		fs3001->dts_info.fs3001_rtp_max = FS3001_RTP_MAX_DEFAULT;
	}		
	else
		pr_info("fs3001_rtp_max = %d\n",fs3001->dts_info.fs3001_rtp_max);

	//fs3001_rtp_time
	val = of_property_read_u32_array(np, "fs3001_rtp_time", rtp_time,ARRAY_SIZE(rtp_time));
	if (val != 0)
	{
		pr_err("%s:fs3001_rtp_time not found\n", FSERROR);
		for (i = 0;i < ARRAY_SIZE(rtp_time);i++)
		{
			fs3001->dts_info.fs3001_rtp_time[i] = FS3001_RTP_TIME_DEFAULT;
		}
	}		
	else
		pr_info("fs3001_rtp_time number = %d\n",(int)ARRAY_SIZE(rtp_time));
	memcpy(fs3001->dts_info.fs3001_rtp_time, rtp_time, sizeof(rtp_time));

	



	//xxxx_play_rate_us(not find it in dts)
	config->play_rate_us = HAP_PLAY_RATE_US_DEFAULT;
	rc = of_property_read_u32(np, "qcom,play-rate-us", &tmp);
	if (!rc)
	{
		config->play_rate_us = (tmp >= HAP_PLAY_RATE_US_MAX) ? HAP_PLAY_RATE_US_MAX : tmp;
		pr_err("%s:qcom,play-rate-us not found,use default value=%d\n", FSERROR,config->play_rate_us);
	}
	else
		pr_info("qcom,play-rate-us = %d\n",tmp);
	

	//4 related struct��3 need to alloc space(predefined,constant�� and pattern��alloc in next code����and pattern in predefined[alloc space in for_each_available_child_of_node])
	//struct qti_hap_config config;
	//struct qti_hap_play_info play;
	//struct qti_hap_effect *predefined;
	//struct qti_hap_effect constant;
       
	//local variable: qti_hap_config *config = &fs3001->config;
	//local variable: effect = &fs3001->predefined[i++];
	//save common value in config(for example:config->play_rate_us��read "qcom,play-rate-us" in dts��no config->vmax_mv value in dts;so)��
	//effect pointer array(same as fs3001->config)   effect->vmax_mv = config->vmax_mv; the operation is no effect,get the value qcom,wf-vmax-mv in dts
	fs3001->constant.pattern = devm_kcalloc(fs3001->dev,HAP_WAVEFORM_BUFFER_MAX,sizeof(u8), GFP_KERNEL);
	if (!fs3001->constant.pattern)
		return -ENOMEM;

	tmp = of_get_available_child_count(np);
	//devm_kcalloc alloc tmp size of sizeof(*fs3001->predefined)
	fs3001->predefined = devm_kcalloc(fs3001->dev, tmp,sizeof(*fs3001->predefined),GFP_KERNEL);
	if (!fs3001->predefined)
		return -ENOMEM;

	fs3001->effects_count = tmp;
	pr_info("fs3001->effects_count=%d\n",fs3001->effects_count);
	i = 0;
	for_each_available_child_of_node(np, child_node) 
	{
		effect = &fs3001->predefined[i++];
		//qcom,effect-id = <0>;
		rc = of_property_read_u32(child_node, "qcom,effect-id",&effect->id);
		if (rc != 0) 
		{
			pr_err("%s:Read qcom,effect-id failed\n",FSERROR);
		}
		else
		{
			pr_info("effect_id: %d\n", effect->id);
		}

		//qcom,wf-vmax-mv = <3600>;
		effect->vmax_mv = config->vmax_mv;
		rc = of_property_read_u32(child_node, "qcom,wf-vmax-mv", &tmp);
		if (rc != 0)
			pr_err("%s:Read qcom,wf-vmax-mv failed !\n",FSERROR);
		else
		{
			effect->vmax_mv = tmp;
			pr_info("effect->vmax_mv =%d \n",effect->vmax_mv);
		}

		//get qcom,wf-pattern = [3e 3e] number
		rc = of_property_count_elems_of_size(child_node,"qcom,wf-pattern",sizeof(u8));
		if (rc < 0) 
		{
			pr_err("%s:Count qcom,wf-pattern property failed !\n",FSERROR);
		}
		else if (rc == 0)
		{
			pr_err("%s:qcom,wf-pattern has no data\n",FSERROR);
		}
		else
		{
			pr_info("effect_id %d,qcom,wf-pattern count =0x%x \n",effect->id,rc);
		}

		//get qcom,wf-pattern = [3e 3e]
		effect->pattern_length = rc;
		effect->pattern = devm_kcalloc(fs3001->dev,effect->pattern_length,sizeof(u8), GFP_KERNEL);

		rc = of_property_read_u8_array(child_node, "qcom,wf-pattern",effect->pattern,effect->pattern_length);
		if (rc < 0) 
		{
			pr_err("%s:Read qcom,wf-pattern property failed !\n",FSERROR);
		}
		else
		{
			for (j = 0; j < effect->pattern_length; j++)
			{
				pr_info("effect->pattern[%d] = =%d \n",j,effect->pattern[j]);
			}
		}

		//qcom,wf-play-rate-us = <20000>;
		effect->play_rate_us = config->play_rate_us;
		rc = of_property_read_u32(child_node, "qcom,wf-play-rate-us",&tmp);
		if (rc < 0)
			pr_err("%s:Read qcom,wf-play-rate-us failed !\n",FSERROR);
		else
		{
			effect->play_rate_us = tmp;
			pr_info("effect->play_rate_us=%d \n",effect->play_rate_us);
		}

		//not find it in dts
		rc = of_property_read_u32(child_node, "qcom,wf-repeat-count",&tmp);
		if (rc < 0) 
		{
			pr_err("%s,Read  qcom,wf-repeat-count failed !\n",FSERROR);
		}
		else 
		{
			for (j = 0; j < ARRAY_SIZE(wf_repeat); j++)
			{
				if (tmp <= wf_repeat[j])
				{
					break;
				}
			}
			effect->wf_repeat_n = j;
		}
		
		//"qcom,lra-auto-resonance-disable"
		effect->lra_auto_res_disable = of_property_read_bool(child_node,"qcom,lra-auto-resonance-disable");
		pr_info("qcom,lra-auto-resonance-disable = %d\n",effect->lra_auto_res_disable);

		//qcom,wf-brake-pattern = [02 01 00 00]; get number
		tmp = of_property_count_elems_of_size(child_node,"qcom,wf-brake-pattern",sizeof(u8));
		if (tmp <= 0)
		{
			continue;
		}

		if (tmp > HAP_BRAKE_PATTERN_MAX) 
		{
			pr_err("%s:wf-brake-pattern shouldn't be more than %d bytes\n",FSERROR, HAP_BRAKE_PATTERN_MAX);
		}
		else
		{
			pr_info("qcom,wf-brake-pattern count=%d \n",tmp);
		}

		//get qcom,wf-brake-pattern = [02 01 00 00] details
		rc = of_property_read_u8_array(child_node,"qcom,wf-brake-pattern",effect->brake, tmp);
		if (rc < 0) 
		{
			pr_err("%s:Failed to get wf-brake-pattern !\n",FSERROR);
		}
		else
		{
			for (j = 0; j < tmp; j++)
			{
				pr_info("effect->brake[%d] = =%d \n",j,effect->brake[j]);
			}
		}

		effect->brake_pattern_length = tmp;
	}
	return 0;
}

//set osc
static void fs3001_haptic_upload_lra(struct fs3001 *fs3001, unsigned int flag)
{
	pr_info("enter flag=%d\n",flag);
	switch (flag) 
	{
		case FS3001_WRITE_ZERO:
			pr_info("write zero to trim_lra!\n");
			fs3001_i2c_write(fs3001, FS3001_FTUNECFG,0x90);
			break;
		case FS3001_F0_CALI:
			pr_info("write f0_cali_data to trim_lra = 0x%02X\n", fs3001->f0_cali_data);
			fs3001_i2c_write(fs3001, FS3001_FTUNECFG,(char)fs3001->f0_cali_data);
			break;
		case FS3001_OSC_CALI:
			pr_info("write osc_cali_data to trim_lra = 0x%02X\n", fs3001->osc_cali_data);
			fs3001_i2c_write(fs3001, FS3001_FTUNECFG,(char)fs3001->osc_cali_data);
			break;
		default:
			break;
	}
}

static int fs3001_haptic_stop(struct fs3001 *fs3001)
{
	bool force_flag = true;
	int i_count = 40;
	unsigned char reg_val_00 = 0;
	
	pr_info("enter\n");

	if (fs3001->vib_stop_flag == true)
	{
		return 0;
	}
	
	fs3001->play_mode = FS3001_HAPTIC_STANDBY_MODE;
	fs3001_i2c_write(fs3001, FS3001_OPCTRL, 0);
	//fs3001_i2c_write_bits_1(fs3001, FS3001_TRGCTRL,1,0,0);//stop trig play

	fs3001->vib_stop_flag = true;

	while(i_count)
	{
		fs3001_i2c_read(fs3001, FS3001_STATUS, &reg_val_00);
		if((reg_val_00 & 0x01) == 0)
		{
			force_flag = false;
			pr_info("standby state, FS3001_STATUS = %d\n",reg_val_00);
			break;
		}
		i_count = i_count - 1;
		pr_info("not in standby state, FS3001_STATUS = %d\n",reg_val_00);
		usleep_range(2000,2500);
	}
	if(force_flag)
	{
		pr_info("enter standby mode again\n");
		fs3001_i2c_write(fs3001, FS3001_OPCTRL, 0);
	}	

	return 0;
}

static void fs3001_haptic_raminit(struct fs3001 *fs3001, bool flag)
{
	pr_info("enter flag = %d\n",flag);

	if (flag) 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_RAMACC,0x03,1,0);
	} 
	else 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_RAMACC,0x00,1,0);
	}
}

static void fs3001_haptic_enable_key(struct fs3001 *fs3001, bool flag)
{
	pr_info("enter flag = %d\n",flag);

	if (flag) 
	{
		fs3001_i2c_write(fs3001, FS3001_ACCKEY,0x91);
	} 
	else 
	{
		fs3001_i2c_write(fs3001, FS3001_ACCKEY,0);
	}
}

static int fs3001_haptic_get_vbat(struct fs3001 *fs3001)
{
	unsigned char reg_val = 0;
	unsigned int vbat_code = 0;
	
	pr_info("enter\n");

	fs3001_haptic_stop(fs3001);

	fs3001_i2c_read(fs3001, FS3001_BATS_L, &reg_val);
	fs3001_i2c_read(fs3001, FS3001_BATS_L, &reg_val);
	vbat_code = reg_val;
	reg_val = fs3001_i2c_read_bits(fs3001, FS3001_BATS_H, 1,0);
	vbat_code = vbat_code | (reg_val << 8);
	
	fs3001->vbat = 6000 * vbat_code / 1024;
	if (fs3001->vbat > FS3001_VBAT_MAX) 
	{
		fs3001->vbat = FS3001_VBAT_MAX;
		pr_info("vbat max limit = %dmV\n", fs3001->vbat);
	}
	if (fs3001->vbat < FS3001_VBAT_MIN) 
	{
		fs3001->vbat = FS3001_VBAT_MIN;
		pr_info("vbat min limit = %dmV\n", fs3001->vbat);
	}
	pr_info("fs3001->vbat=%dmV, vbat_code=0x%02X\n", fs3001->vbat, vbat_code);
	return 0;
}

static void fs3001_interrupt_clear(struct fs3001 *fs3001)
{
	unsigned char reg_val = 0;

	pr_info("enter\n");
	fs3001_i2c_read(fs3001, FS3001_INTSTAT1, &reg_val);
	pr_info("reg SYSINT=0x%02X\n", reg_val);
	fs3001_i2c_read(fs3001, FS3001_INTSTAT2, &reg_val);
	pr_info("reg SYSINT=0x%02X\n", reg_val);
}

static int fs3001_haptic_set_gain(struct fs3001 *fs3001, unsigned char gain)
{
	unsigned char comp_gain = 0;
	unsigned char uc_VBD = 0;

	pr_info("enter gain = 0x%x\n",gain);

	switch (fs3001->dts_info.fs3001_vbat_mode) 
	{
		//set the gain normally in these 3 situations
		case FS3001_HAPTIC_VBAT_COMP_DISABLE:
		case FS3001_HAPTIC_VBAT_COMP_PLAY_AUTO_BRK_DISABLE:
		case FS3001_HAPTIC_VBAT_COMP_PLAY_AUTO_BRK_ENABLE:
			pr_info("vbat mode = %d, gain=0x%x", fs3001->dts_info.fs3001_vbat_mode,gain);
			fs3001_i2c_write(fs3001, FS3001_GAINCFG, gain);
			break;
		//by adjust the gain
		case FS3001_HAPTIC_VBAT_COMP_PLAY_GAIN_BRK_DISABLE:
			fs3001_haptic_get_vbat(fs3001);
			pr_info("vbat mode = %d, vbat=%d, vbat_min=%d, vbat_ref=%d", fs3001->dts_info.fs3001_vbat_mode,fs3001->vbat, FS3001_VBAT_MIN, FS3001_VBAT_REFER);
			comp_gain = fs3001->gain * FS3001_VBAT_REFER / fs3001->vbat;
			if (comp_gain > (128 * FS3001_VBAT_REFER / FS3001_VBAT_MIN)) 
			{
				comp_gain = 128 * FS3001_VBAT_REFER / FS3001_VBAT_MIN;
				pr_info("gain limit=%d\n",comp_gain);
			}
			pr_info("enable vbat comp, level = %x comp level = %x",gain, comp_gain);
			fs3001_i2c_write(fs3001, FS3001_GAINCFG, comp_gain);
			break;
		//manu vbat
		case FS3001_HAPTIC_VBAT_COMP_PLAY_REG_BRK_DISABLE:
		case FS3001_HAPTIC_VBAT_COMP_PLAY_REG_BRK_ENABLE:
			fs3001_i2c_write(fs3001, FS3001_GAINCFG, gain);
			
			fs3001_haptic_get_vbat(fs3001);
			uc_VBD = 64*fs3001->vbat/6;
			pr_info("vbat mode = %d, vbat=%d, VBD=%d", fs3001->dts_info.fs3001_vbat_mode,fs3001->vbat, uc_VBD);
			if(uc_VBD>0)
			{
				fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCFG,uc_VBD,5,0);
			}
			break;
		default:
			pr_info("unsupported vbat mode (0x%x)\n", fs3001->dts_info.fs3001_vbat_mode);
			fs3001_i2c_write(fs3001, FS3001_GAINCFG, gain);
			break;
	}	
	return 0;
}

//vbat mode
static int fs3001_haptic_vbat_mode_config(struct fs3001 *fs3001,unsigned int mode)
{
	pr_info("enter mode = %d\n",mode);

	switch (mode) 
	{
		case FS3001_HAPTIC_VBAT_COMP_DISABLE:
		case FS3001_HAPTIC_VBAT_COMP_PLAY_GAIN_BRK_DISABLE:
			fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCTRL,0x00,4,0);
			break;
		case FS3001_HAPTIC_VBAT_COMP_PLAY_REG_BRK_DISABLE:
			fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCTRL,0x12,4,0);
			break;
		case FS3001_HAPTIC_VBAT_COMP_PLAY_AUTO_BRK_DISABLE:
			fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCTRL,0x02,4,0);
			break;
		case FS3001_HAPTIC_VBAT_COMP_PLAY_REG_BRK_ENABLE:
			fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCTRL,0x13,4,0);
			break;
		case FS3001_HAPTIC_VBAT_COMP_PLAY_AUTO_BRK_ENABLE:
			fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCTRL,0x03,4,0);
			break;
		default:
			pr_info("unsupported vbat mode (0x%x)\n", mode);
			fs3001->dts_info.fs3001_vbat_mode = FS3001_HAPTIC_VBAT_COMP_DISABLE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_VCOMPCTRL,0x00,4,0);
			break;
	}

	return 0;
}

//vbat compensate
static int fs3001_haptic_ram_vbat_compensate(struct fs3001 *fs3001,bool flag)
{
	pr_info("enter flag = %d\n",flag);

	if (flag)
		fs3001_haptic_vbat_mode_config(fs3001,fs3001->dts_info.fs3001_vbat_mode);
	else
		fs3001_haptic_vbat_mode_config(fs3001,FS3001_HAPTIC_VBAT_COMP_DISABLE);

	return 0;
}

static int fs3001_haptic_play_mode(struct fs3001 *fs3001,unsigned char play_mode)
{
	pr_info("enter\n");

	switch (play_mode) 
	{
		case FS3001_HAPTIC_STANDBY_MODE:
			pr_info("enter standby mode\n");
			fs3001->play_mode = FS3001_HAPTIC_STANDBY_MODE;
			fs3001_haptic_stop(fs3001);
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,0,1,0);
			// disable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);
			break;
		case FS3001_HAPTIC_RAM_MODE:
			pr_info("enter ram mode\n");
			fs3001->play_mode = FS3001_HAPTIC_RAM_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,0,1,0);
			// disable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);			
			break;
		case FS3001_HAPTIC_RAM_LOOP_MODE:
			pr_info("enter ram loop mode\n");
			fs3001->play_mode = FS3001_HAPTIC_RAM_LOOP_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,0,1,0);
			// disable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);			
			break;
		case FS3001_HAPTIC_RTP_MODE:
			pr_info("enter rtp mode\n");
			fs3001->play_mode = FS3001_HAPTIC_RTP_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,1,1,0);
			// disable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);			
			break;
		case FS3001_HAPTIC_TRIG_MODE:
			pr_info("enter trig mode\n");
			fs3001->play_mode = FS3001_HAPTIC_TRIG_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,0,1,0);
			// disable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);
			break;
		case FS3001_HAPTIC_CONT_MODE:
			pr_info("enter cont mode\n");
			fs3001->play_mode = FS3001_HAPTIC_CONT_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,2,1,0);
			// disable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);			
	        break;
		case FS3001_HAPTIC_F0_DETECT_MODE:
			pr_info("enter F0 detect mode\n");
			fs3001->play_mode = FS3001_HAPTIC_F0_DETECT_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,2,1,0);
			// enable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,7,7);
			// disable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,4,4);
			break;
		case FS3001_HAPTIC_F0_CALI_MODE:
			pr_info("enter F0 cali mode\n");
			fs3001->play_mode = FS3001_HAPTIC_F0_CALI_MODE;
			fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,3,1,0);
			// enable f0 detect
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,1,7,7);
			// enable f0 self tuning
			fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL,0,4,4);
			break;
		default:
			pr_err("%s:play mode %d error", FSERROR, play_mode);
			break;
	}
	return 0;
}

static int fs3001_haptic_play_go(struct fs3001 *fs3001)
{
	unsigned char reg_val = 0;

	pr_info("enter\n");

	fs3001_i2c_write(fs3001, FS3001_OPCTRL, 0x00);//Do a stop action regardless of whether the flag is false
			

	fs3001_haptic_enable_key(fs3001,true);
	fs3001_i2c_write_bits_1(fs3001, FS3001_ANACTRL,1,6,6);
	fs3001_i2c_write(fs3001, FS3001_OPCTRL, 0x01);
	fs3001_i2c_read(fs3001, FS3001_STATUS, &reg_val);//read 0x0 to ensure osc off
	fs3001_i2c_write_bits_1(fs3001, FS3001_ANACTRL,0,6,6);
	fs3001_haptic_enable_key(fs3001,false);

	mdelay(2);
	fs3001->vib_stop_flag = false;

	return 0;
}

static int fs3001_haptic_set_wav_seq(struct fs3001 *fs3001,unsigned char wav, unsigned char seq)
{
	pr_info("enter wav=%d  seq=%d\n",wav,seq);
	fs3001_i2c_write_bits_1(fs3001, FS3001_WFSCFG1 + wav, seq, 6, 0);
	return 0;
}

static int fs3001_haptic_set_wav_loop(struct fs3001 *fs3001,unsigned char wav, unsigned char loop)
{
	pr_info("enter  wav=%d   loop=%d\n",wav,loop);
	if (wav % 2 == 0) 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_WFSLOOP1 + (wav / 2),loop,3,0);
	} 
	else 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_WFSLOOP1 + (wav / 2),loop,7,4);
	}
	return 0;
}

//haptic f0 cali
static int fs3001_haptic_read_lra_f0(struct fs3001 *fs3001)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_info("enter = %d\n",ret);
	//F_LRA_F0_L
	ret = fs3001_i2c_read(fs3001, FS3001_F0CALB_L, &reg_val);
	ret = fs3001_i2c_read(fs3001, FS3001_F0CALB_L, &reg_val);
	f0_reg = reg_val;
	//F_LRA_F0_H
	ret = fs3001_i2c_read(fs3001, FS3001_F0CALB_H, &reg_val);
	f0_reg = f0_reg | (reg_val << 8);
	if (!f0_reg) 
	{
		pr_err("%s:didn't get lra f0 because f0_reg value is 0!\n", FSERROR);
		fs3001->f0 = fs3001->dts_info.fs3001_f0_ref;
		return -1;
	} 
	else 
	{
		f0_tmp = FS3001_BASE_FRE * 10 / f0_reg;
		fs3001->f0 = (unsigned int)f0_tmp;
		pr_info("lra_f0=%d\n",fs3001->f0);
	}

	return 0;

}

static int fs3001_haptic_read_cont_f0(struct fs3001 *fs3001)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_info("enter = %d\n",ret);
	ret = fs3001_i2c_read(fs3001, FS3001_F0TRK_L, &reg_val);
	ret = fs3001_i2c_read(fs3001, FS3001_F0TRK_L, &reg_val);
	f0_reg = reg_val;
	ret = fs3001_i2c_read(fs3001, FS3001_F0TRK_H, &reg_val);
	f0_reg = f0_reg | (reg_val << 8);

	if (!f0_reg) 
	{
		pr_err("%s:didn't get cont f0 because f0_reg value is 0!\n", FSERROR);
		fs3001->cont_f0 = fs3001->dts_info.fs3001_f0_ref;
		return -1;
	} 
	else 
	{
		f0_tmp = FS3001_BASE_FRE * 10 / f0_reg;
		fs3001->cont_f0 = (unsigned int)f0_tmp;
		pr_info("cont_f0=%d\n",fs3001->cont_f0);
	}
	return 0;

}

//get f0, no cali operation
static int fs3001_haptic_cont_get_f0(struct fs3001 *fs3001)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int cnt = 200;
	bool get_f0_flag = false;

	pr_info("enter\n");
	fs3001->f0 = fs3001->dts_info.fs3001_f0_ref;
	// enter standby mode
	fs3001_haptic_stop(fs3001);
	
	// cont play go
	fs3001_haptic_play_go(fs3001);
	// 300ms
	while (cnt) 
	{
		fs3001_i2c_read(fs3001, FS3001_DIGSTAT, &reg_val);
		if ((reg_val >> 4) == FS3001_DIGSTAT_B7_4_OPS_OFF) 
		{
			get_f0_flag = true;
			pr_info("entered standby mode! state=0x%02X,count=%d\n", reg_val,cnt);
			cnt = 0;
		} 
		else 
		{
			cnt--;
			pr_info("waitting for standby, state=0x%02X\n", reg_val);
		}
		usleep_range(10000, 10500);
	}
	
	if (get_f0_flag) 
	{
		fs3001_haptic_read_lra_f0(fs3001);
		fs3001_haptic_read_cont_f0(fs3001);
	} 
	else 
	{
		pr_err("%s:enter standby mode failed, stop reading f0!\n", FSERROR);
	}
	return ret;
}

static int fs3001_haptic_rtp_init(struct fs3001 *fs3001)
{
	unsigned int buf_len = 0;
	unsigned char DIGSTAT_OPS = 0;

	pr_info("enter fs3001->play_mode=%d\n",fs3001->play_mode);
	pm_qos_enable(true);
	fs3001->rtp_cnt = 0;
	mutex_lock(&fs3001->rtp_lock);
	while ((!fs3001_haptic_rtp_get_fifo_afs(fs3001)) && (fs3001->play_mode == FS3001_HAPTIC_RTP_MODE) &&  !atomic_read(&fs3001->exit_in_rtp_loop)) 
	{
		pr_info("rtp cnt = %d\n",fs3001->rtp_cnt);
		if (!fs3001->rtp_container) 
		{
			pr_err("%s,fs3001->rtp_container is null, break!\n",FSERROR);
			break;
		}
		if (fs3001->rtp_cnt < (fs3001->ram.base_addr)) 
		{
			if ((fs3001->rtp_container->len - fs3001->rtp_cnt) < (fs3001->ram.base_addr)) 
			{
				buf_len = fs3001->rtp_container->len - fs3001->rtp_cnt;
			}
			else 
			{
				buf_len = fs3001->ram.base_addr;
			}
		} 
		else if ((fs3001->rtp_container->len - fs3001->rtp_cnt) < (fs3001->ram.base_addr >> 2)) 
		{
			buf_len = fs3001->rtp_container->len - fs3001->rtp_cnt;
		}
		else 
		{
			buf_len = fs3001->ram.base_addr >> 2;
		}
		pr_info("buf_len = %d\n",buf_len);
		fs3001_i2c_writes(fs3001,FS3001_RTPWDATA,&fs3001->rtp_container->data[fs3001->rtp_cnt],buf_len);
		fs3001->rtp_cnt += buf_len;
		fs3001_i2c_read(fs3001, FS3001_DIGSTAT, &DIGSTAT_OPS);//DIGSTAT_OPS == 0x00  inactive
		DIGSTAT_OPS = DIGSTAT_OPS >> 4;
		if ((fs3001->rtp_cnt == fs3001->rtp_container->len) || (DIGSTAT_OPS == FS3001_DIGSTAT_B7_4_OPS_OFF)) 
		{
			if (fs3001->rtp_cnt == fs3001->rtp_container->len)
				pr_info("rtp load completely! DIGSTAT_OPS=0x%02x fs3001->rtp_cnt=%d\n", DIGSTAT_OPS,fs3001->rtp_cnt);
			else
				pr_err("%s:rtp load failed!! DIGSTAT_OPS=0x%02x fs3001->rtp_cnt=%d\n", FSERROR, DIGSTAT_OPS,fs3001->rtp_cnt);
			fs3001->rtp_cnt = 0;
			pm_qos_enable(false);
			fs3001_haptic_raminit(fs3001,false);
			mutex_unlock(&fs3001->rtp_lock);
			return 0;
		}
	}
	mutex_unlock(&fs3001->rtp_lock);

	if (fs3001->play_mode == FS3001_HAPTIC_RTP_MODE && !atomic_read(&fs3001->exit_in_rtp_loop))
		fs3001_haptic_set_rtp_aei(fs3001, true);

	pr_info("exit\n");
	pm_qos_enable(false);
	return 0;
}

static unsigned char fs3001_haptic_osc_read_status(struct fs3001 *fs3001)
{
	unsigned char reg_val = 0;
	pr_info("enter\n");

	fs3001_i2c_read(fs3001, FS3001_INTSTATR2, &reg_val);
	return reg_val;
}

static int fs3001_haptic_set_repeat_wav_seq(struct fs3001 *fs3001,unsigned char seq)
{
	pr_info("enter repeat wav seq %d\n", seq);
	fs3001_haptic_set_wav_seq(fs3001, 0x00, seq);
	fs3001_haptic_set_wav_loop(fs3001, 0x00,FS3001_WFSLOOP_INIFINITELY);
	return 0;
}

static int fs3001_haptic_set_pgagain(struct fs3001 *fs3001,unsigned char pgagain)
{
	pr_info("enter pgagain=0x%2x\n", pgagain);
	fs3001_i2c_write_bits_1(fs3001, FS3001_PGACTRL,pgagain,3,0);
	return 0;
}


static int fs3001_haptic_set_pwm(struct fs3001 *fs3001, unsigned char mode)
{
	pr_info("enter mode=%d\n",mode);

	fs3001_i2c_write_bits_1(fs3001, FS3001_SYSCTRL,mode,5,4);
	return 0;
}

static int16_t fs3001_haptic_effect_strength(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	pr_info("fs3001->play.vmax_mv =0x%x\n",fs3001->play.vmax_mv);
#if 0
	switch (fs3001->play.vmax_mv) 
	{
		case FS3001_LIGHT_MAGNITUDE:
			fs3001->level = 0x30;
			break;	
		case FS3001_MEDIUM_MAGNITUDE:
			fs3001->level = 0x50;
			break;
		case FS3001_STRONG_MAGNITUDE:
			fs3001->level = 0x80;
			break;
		default:
			break;
	}
#else
	if (fs3001->play.vmax_mv >= 0x7FFF)
		fs3001->level = 0x80;	//128 
	else if (fs3001->play.vmax_mv <= 0x3FFF)
		fs3001->level = 0x1E;	//30
	else
		fs3001->level = (fs3001->play.vmax_mv - 16383) / 128;
	if (fs3001->level < 0x1E)
		fs3001->level = 0x1E;	//30
#endif

	pr_info("fs3001->level =0x%x\n", fs3001->level);
	return 0;
}

static void fs3001_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int cnt = 200;
	unsigned char reg_val = 0;
	bool rtp_work_flag = false;
	struct fs3001 *fs3001 = container_of(work, struct fs3001, rtp_work);

	pr_info("enter fs3001->effect_id=%d,fs3001->dts_info.fs3001_rtp_id_boundary=%d, fs3001->dts_info.fs3001_rtp_max=%d,fs3001->dts_info.fs3001_bypass_system_gain=%d\n",fs3001->effect_id,fs3001->dts_info.fs3001_rtp_id_boundary,fs3001->dts_info.fs3001_rtp_max,fs3001->dts_info.fs3001_bypass_system_gain);

	if ((fs3001->effect_id < fs3001->dts_info.fs3001_rtp_id_boundary) && (fs3001->effect_id > fs3001->dts_info.fs3001_rtp_max))
	{
		return;
	}
		
	pr_info("effect_id =%d state = %d\n", fs3001->effect_id,fs3001->state);
	mutex_lock(&fs3001->lock);
	fs3001_haptic_upload_lra(fs3001, FS3001_OSC_CALI);
	fs3001_haptic_set_rtp_aei(fs3001, false);
	fs3001_interrupt_clear(fs3001);
	//wait for irq to exit
	atomic_set(&fs3001->exit_in_rtp_loop, 1);
	while (atomic_read(&fs3001->is_in_rtp_loop)) 
	{
		pr_info("goint to waiting irq exit\n");
		ret = wait_event_interruptible(fs3001->wait_q,atomic_read(&fs3001->is_in_rtp_loop) == 0);
		pr_info("wakeup \n");
		if (ret == -ERESTARTSYS) 
		{
			atomic_set(&fs3001->exit_in_rtp_loop, 0);
			wake_up_interruptible(&fs3001->stop_wait_q);
			mutex_unlock(&fs3001->lock);
			pr_err("%s:wake up by signal return error\n",FSERROR);
			return;
		}
	}
	atomic_set(&fs3001->exit_in_rtp_loop, 0);
	wake_up_interruptible(&fs3001->stop_wait_q);
	fs3001_haptic_stop(fs3001);

	if (fs3001->state) 
	{
		pm_stay_awake(fs3001->dev);
		fs3001->wk_lock_flag = 1;
		
		//xxxx  ??
		//fs3001->rtp_file_num = fs3001->effect_id - 20;
		
		if (fs3001->rtp_file_num < 0)
			fs3001->rtp_file_num = 0;
		if (fs3001->rtp_file_num > ((sizeof(fs3001_rtp_name) / FS3001_RTP_NAME_MAX) - 1))
			fs3001->rtp_file_num = (sizeof(fs3001_rtp_name) / FS3001_RTP_NAME_MAX) - 1;
		pr_info("fs3001->rtp_file_num =%d\n",fs3001->rtp_file_num);


		//fw loaded
		ret = request_firmware(&rtp_file,fs3001_rtp_name[fs3001->rtp_file_num],fs3001->dev);
		if (ret < 0) 
		{
			pr_err("%s:failed to read %s\n", FSERROR,fs3001_rtp_name[fs3001->rtp_file_num]);
			if (fs3001->wk_lock_flag == 1) 
			{
				pm_relax(fs3001->dev);
				fs3001->wk_lock_flag = 0;
			}
			mutex_unlock(&fs3001->lock);
			return;
		}
		fs3001->rtp_init = 0;
		vfree(fs3001->rtp_container);
		fs3001->rtp_container = vmalloc(rtp_file->size + sizeof(int));
		if (!fs3001->rtp_container) 
		{
			release_firmware(rtp_file);
			pr_err("%s:error allocating memory\n", FSERROR);
		    if (fs3001->wk_lock_flag == 1) 
			{
				pm_relax(fs3001->dev);
				fs3001->wk_lock_flag = 0;
			}
			mutex_unlock(&fs3001->lock);
			return;
		}
		fs3001->rtp_container->len = rtp_file->size;
		pr_info("rtp file:[%s] size = %dbytes\n",fs3001_rtp_name[fs3001->rtp_file_num],fs3001->rtp_container->len);
		memcpy(fs3001->rtp_container->data, rtp_file->data, rtp_file->size);
		release_firmware(rtp_file);
		fs3001->rtp_init = 1;//memcpy(fs3001->rtp_container->data, rtp_file->data, rtp_file->size);
		fs3001_haptic_upload_lra(fs3001, FS3001_OSC_CALI);
		fs3001_haptic_set_pwm(fs3001, fs3001->dts_info.fs3001_play_rtp_srate);//20210601
		//gain
		fs3001_haptic_ram_vbat_compensate(fs3001, false);
		if(fs3001->dts_info.fs3001_bypass_system_gain == 0)
		{
			fs3001_haptic_effect_strength(fs3001);
			fs3001_haptic_set_gain(fs3001, fs3001->level);//20210716
		}
		
		//rtp mode config
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RTP_MODE);
		//enable ram access and bulk
		fs3001_haptic_raminit(fs3001,true);
		//haptic go
		fs3001_haptic_play_go(fs3001);
		//The AE and E interrupts of FS3001 need to be clear again: here, although all masks of INT are closed and will not enter FS3001_irq, we will open the mask of almost empty when almost full occurs
		//It does not matter if the FS3001_interrupt_clear operation is not performed here. After all, the mask is turned off and will not cause an actual interrupt
		//fs3001_interrupt_clear(fs3001);
		mutex_unlock(&fs3001->lock);
		usleep_range(2000, 2500);
		while (cnt) 
		{
			fs3001_i2c_read(fs3001, FS3001_DIGSTAT, &reg_val);
			if ((reg_val >> 4) == FS3001_DIGSTAT_B7_4_OPS_GO) //DIGSTAT_OPS == 0x20  go
			{
				cnt = 0;
				rtp_work_flag = true;
				pr_info("RTP_GO! OPS = 2\n");
			} 
			else 
			{
				cnt--;
				pr_info("wait for RTP_GO, OPS=%d\n",(reg_val>>4));
			}
			usleep_range(2000, 2500);
		}
		if (rtp_work_flag) 
		{
			fs3001_haptic_rtp_init(fs3001);
		}
		else 
		{
			//enter standby mode
			fs3001_haptic_stop(fs3001);
			pr_err("%s:failed to enter RTP_GO status!\n", FSERROR);
		}
	}
	else 
	{
		pr_info("fs3001->state=%d, fs3001->wk_lock_flag=%d\n",fs3001->state,fs3001->wk_lock_flag);
		if (fs3001->wk_lock_flag == 1) 
		{
			pm_relax(fs3001->dev);
			fs3001->wk_lock_flag = 0;
		}
		fs3001->rtp_cnt = 0;
		fs3001->rtp_init = 0;
		mutex_unlock(&fs3001->lock);
	}
}

//play a 10s rtp file, then get fs3001->microsecond
static int fs3001_rtp_osc_calibration(struct fs3001 *fs3001)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char osc_int_state = 0;

	fs3001->rtp_cnt = 0;
	fs3001->timeval_flags = 1;

	pr_info("enter\n");
	//fw loaded
	ret = request_firmware(&rtp_file, fs3001_rtp_name[0], fs3001->dev);
	if (ret < 0) 
	{
		pr_err("%s:failed to read %s\n", FSERROR,fs3001_rtp_name[0]);
		return ret;
	}
	//foursemi add stop,for irq interrupt during calibrate
	fs3001_haptic_stop(fs3001);
	fs3001->rtp_init = 0;
	mutex_lock(&fs3001->rtp_lock);
	vfree(fs3001->rtp_container);
	fs3001->rtp_container = vmalloc(rtp_file->size + sizeof(int));
	if (!fs3001->rtp_container) 
	{
		release_firmware(rtp_file);
		mutex_unlock(&fs3001->rtp_lock);
		pr_err("%s:error allocating memory\n", FSERROR);
		return -ENOMEM;
	}
	fs3001->rtp_container->len = rtp_file->size;
	fs3001->rtp_len = rtp_file->size;
	pr_info("rtp file:[%s] size = %dbytes\n",fs3001_rtp_name[0], fs3001->rtp_container->len);

	memcpy(fs3001->rtp_container->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&fs3001->rtp_lock);
	// gain
	fs3001_haptic_ram_vbat_compensate(fs3001, false);
	// rtp mode config
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RTP_MODE);
	//burst mode enable
	fs3001_haptic_raminit(fs3001,true);

	//xxxx_set the INT mode here?
	//fs3001_i2c_write_bits(fs3001, FS3001_REG_SYSCTRL7,FS3001_BIT_SYSCTRL7_INT_MODE_MASK,FS3001_BIT_SYSCTRL7_INT_MODE_EDGE);
	disable_irq(gpio_to_irq(fs3001->irq_gpio));
	//haptic go
	fs3001_haptic_play_go(fs3001);
	fs3001_interrupt_clear(fs3001);//need to do it immediately, otherwise fs3001_haptic_rtp_get_fifo_afs will get wrong result
	//require latency of CPU & DMA not more then PM_QOS_VALUE_VB us
	pm_qos_enable(true);
	while (1) 
	{
		//!almost full
		if (!fs3001_haptic_rtp_get_fifo_afs_0xAF(fs3001)) 
		{
			mutex_lock(&fs3001->rtp_lock);
			if ((fs3001->rtp_container->len - fs3001->rtp_cnt) < (fs3001->ram.base_addr >> 2))
				buf_len = fs3001->rtp_container->len - fs3001->rtp_cnt;
			else
				buf_len = (fs3001->ram.base_addr >> 2);

			//transfered file len != total file len
			if (fs3001->rtp_cnt != fs3001->rtp_container->len) 
			{
				if (fs3001->timeval_flags == 1) 
				{
					//start
					ktime_get_real_ts64(&fs3001->start);
					fs3001->timeval_flags = 0;
				}
				fs3001->rtp_update_flag = fs3001_i2c_writes(fs3001,FS3001_RTPWDATA,&fs3001->rtp_container->data[fs3001->rtp_cnt],buf_len);
				fs3001->rtp_cnt += buf_len;
			}
			mutex_unlock(&fs3001->rtp_lock);
		}
		osc_int_state = fs3001_haptic_osc_read_status(fs3001);
		if (util_get_bit(osc_int_state,2)==1) 
		{
			ktime_get_real_ts64(&fs3001->end);
			pr_info("osc trim playback done fs3001->rtp_cnt= %d\n", fs3001->rtp_cnt);
			break;
		}
		ktime_get_real_ts64(&fs3001->end);
		fs3001->microsecond = (fs3001->end.tv_sec - fs3001->start.tv_sec) * 1000000 + (fs3001->end.tv_nsec - fs3001->start.tv_nsec) / 1000000;
		if (fs3001->microsecond > FS3001_OSC_CALI_MAX_LENGTH) 
		{
			pr_err("%s:osc trim time out! fs3001->rtp_cnt %d osc_int_state %02x\n", FSERROR, fs3001->rtp_cnt, osc_int_state);
			break;
		}
	}
	pm_qos_enable(false);
	enable_irq(gpio_to_irq(fs3001->irq_gpio));

	//burst mode disable
	fs3001_haptic_raminit(fs3001,false);

	fs3001->microsecond =(fs3001->end.tv_sec - fs3001->start.tv_sec) * 1000000 +(fs3001->end.tv_nsec - fs3001->start.tv_nsec)  / 1000000;
	//calibration osc
	pr_info("foursemi_microsecond: %ld\n",fs3001->microsecond);
	pr_info("exit\n");
	return 0;
}

static int fs3001_osc_trim_calculation(unsigned long int theory_time,unsigned long int real_time)
{
	unsigned int lra_code = 0;
	unsigned int DFT_LRA_TRIM_CODE = 0x90;

	pr_info("enter\n");
	if (theory_time == real_time) 
	{
		pr_info("theory_time == real_time: %ld, no need to calibrate!\n", real_time);
		return DFT_LRA_TRIM_CODE;
	}
	else if(abs(real_time - theory_time)> (theory_time / 50))
	{
		pr_info("|real_time - theory_time| > (theory_time/50), can't calibrate!\n");
		return DFT_LRA_TRIM_CODE;
	}
	else
	{
		 lra_code = (100 * (real_time - theory_time) + 257064068) / 1776596;
	}
	
	pr_info("real_time: %ld, theory_time: %ld, lra_code:0x%02X\n", real_time,theory_time,lra_code);
	return lra_code;
}

static unsigned int get_d2sgain(unsigned char uc_0x2A)
{
	switch (uc_0x2A)
	{
	case 0:
		return 25;
	case 1:
		return 50;
	case 2:
		return 100;
	case 3:
		return 200;
	case 4:
		return 400;
	case 5:
	case 6:
	case 7:
		return 800;
	case 8:
		return 125;
	case 9:
		return 250;
	case 10:
		return 500;
	case 11:
		return 1000;
	case 12:
		return 2000;
	case 13:
	case 14:
	case 15:
		return 4000;
	default:
		return 800;
	}
}


static int fs3001_haptic_get_lra_resistance(struct fs3001 *fs3001)
{
	unsigned char uc_0xBC = 0;
	unsigned char uc_0xBD = 0;
 	unsigned int ui_BCBD = 0;
	unsigned char uc_0x2A = 0;
	int i = 0;
	
	pr_info("enter\n");

	mutex_lock(&fs3001->lock);
	fs3001_haptic_stop(fs3001);
	fs3001_haptic_diagnostic_sequence(fs3001);

	fs3001_i2c_write(fs3001, FS3001_SWDIAG1, 0xFA);//0xB5 = 0xFA		// Turn of S2F, turn on RL detection current sink
	usleep_range(1000, 2000);//usleep_range(100, 110); //Wait > 100 us(100);                                                           //Wait > 100 us 


	ui_BCBD = 0;
	for(i=0;i<FS3001_OFFSET_RETRIES;i++)
	{
		fs3001_i2c_write(fs3001, FS3001_SARCTRL, 0x90);//0xBB = 0x90		// SAR capture data
		usleep_range(1000, 2000);//usleep_range(10, 20); //Wait > 10 us
		fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
		fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
		uc_0xBD = fs3001_i2c_read_bits(fs3001, FS3001_SARTS_H, 1,0);//0xBD//high 2 offset value
		ui_BCBD = (uc_0xBD<<8 | uc_0xBC) + ui_BCBD;
	}
	ui_BCBD = ui_BCBD/FS3001_OFFSET_RETRIES;
	uc_0x2A = fs3001_i2c_read_bits(fs3001, FS3001_PGACTRL, 3,0);

	fs3001->lra = (fs3001->offset - ui_BCBD) * 428600 / (1024 * get_d2sgain(uc_0x2A));
	pr_info("fs3001->offset=%d,ui_BCBD=%d,uc_0x2A=%d,fs3001->lra=%d\n",fs3001->offset,ui_BCBD,uc_0x2A,fs3001->lra);
	fs3001_haptic_set_diagnostic_reg_to_default_value(fs3001);
	mutex_unlock(&fs3001->lock);
	return 0;
}

static int fs3001_haptic_juge_RTP_is_going_on(struct fs3001 *fs3001)
{
	unsigned char rtp_state = 0;
	unsigned char mode = 0;
	unsigned char glb_st = 0;
	pr_info("enter\n");
	
	fs3001_i2c_read(fs3001, FS3001_SYSCTRL, &mode);
	fs3001_i2c_read(fs3001, FS3001_DIGSTAT, &glb_st);
	if (((mode & 0x3) == FS3001_SYSCTRL_B1_0_OPMODE_RTP) && ((glb_st >> 4) ==  FS3001_DIGSTAT_B7_4_OPS_GO))
	{
		rtp_state = 1;
	}
	return rtp_state;
}

static int fs3001_container_update(struct fs3001 *fs3001,struct fs3001_container *fs3001_cont)
{
	unsigned char reg_val = 0;
	unsigned int shift = 0;
	unsigned int temp = 0;
	//int i = 0;
	int ret = 0;
#ifdef FS_CHECK_RAM_DATA
	unsigned short check_sum = 0;
#endif

	pr_info("enter\n");
	mutex_lock(&fs3001->lock);
	fs3001->ram.baseaddr_shift = 2;
	fs3001->ram.ram_shift = 4;
	//RAMINIT Enable
	fs3001_haptic_raminit(fs3001, true);

	//Enter standby mode
	fs3001_haptic_stop(fs3001);
	//base addr
	shift = fs3001->ram.baseaddr_shift;
	fs3001->ram.base_addr = (unsigned int)((fs3001_cont->data[0 + shift] << 8) |(fs3001_cont->data[1 + shift]));

	//BASE_ADDRH  WFSBASE_H 
	fs3001_i2c_write_bits_1(fs3001, FS3001_WFSBASE_H,fs3001_cont->data[0 + shift],3,0);

	//BASE_ADDRL  WFSBASE_L 
	fs3001_i2c_write(fs3001, FS3001_WFSBASE_L,fs3001_cont->data[1 + shift]);

	//FIFO_AEH  1/2 of fifo
	fs3001_i2c_write_bits_1(fs3001, FS3001_RTPFIFOAE_H,(unsigned char)(((fs3001->ram.base_addr >> 1) >> 8) & 0x0F),3,0);
	//FIFO AEL
	fs3001_i2c_write(fs3001, FS3001_RTPFIFOAE_L,(unsigned char)(((fs3001->ram.base_addr >> 1) & 0xFF)));

	
	// FIFO_AFH  1/4 of fifo
	fs3001_i2c_write_bits_1(fs3001, FS3001_RTPFIFOAF_H,(unsigned char)(((fs3001->ram.base_addr -(fs3001->ram.base_addr >> 2)) >> 8) & 0x0F),3,0);
	// FIFO_AFL
	fs3001_i2c_write(fs3001, FS3001_RTPFIFOAF_L,(unsigned char)(((fs3001->ram.base_addr -(fs3001->ram.base_addr >> 2)) & 0xFF)));

	fs3001_i2c_read(fs3001, FS3001_RTPFIFOAE_L, &reg_val);
	temp = fs3001_i2c_read_bits(fs3001, FS3001_RTPFIFOAE_H, 3,0);
	temp = (temp<<8) | reg_val;
	pr_info("almost_empty_threshold = %d\n", temp);

	fs3001_i2c_read(fs3001, FS3001_RTPFIFOAF_L, &reg_val);
	temp = fs3001_i2c_read_bits(fs3001, FS3001_RTPFIFOAF_H, 3,0);
	temp = (temp<<8) | reg_val;
	pr_info("almost_full_threshold = %d\n", temp);
	
	//ram
	shift = fs3001->ram.baseaddr_shift;

	//RAMADDR_H
	fs3001_i2c_write_bits_1(fs3001, FS3001_RAMADDR_H,fs3001_cont->data[0 + shift],3,0);
	//RAMADDR_L
	fs3001_i2c_write(fs3001, FS3001_RAMADDR_L,fs3001_cont->data[1 + shift]);

	shift = fs3001->ram.ram_shift;
	pr_info("ram_len = %d\n",fs3001_cont->len - shift);

	fs3001->ram_update_flag = fs3001_i2c_writes(fs3001,FS3001_RAMWDATA,&fs3001_cont->data[shift],fs3001_cont->len - shift);
	//for (i = shift; i < fs3001_cont->len; i++) 
	//{
	//	fs3001->ram_update_flag = fs3001_i2c_write(fs3001,FS3001_RAMWDATA,fs3001_cont->data[i]);
	//}
#ifdef	FS_CHECK_RAM_DATA
	shift = fs3001->ram.baseaddr_shift;
	//RAMADDR_H
	fs3001_i2c_write_bits_1(fs3001, FS3001_RAMADDR_H,fs3001_cont->data[0 + shift],3,0);
	//RAMADDR_H
	fs3001_i2c_write(fs3001, FS3001_RAMADDR_L,fs3001_cont->data[1 + shift]);

	shift = fs3001->ram.ram_shift;
	for (i = shift; i < fs3001_cont->len; i++) 
	{
		fs3001_i2c_read(fs3001, FS3001_RAMRDATA, &reg_val);
		//for debug
		//pr_info("fs3001_cont->data=0x%02X, ramdata=0x%02X\n",fs3001_cont->data[i],reg_val);
		
		if (reg_val != fs3001_cont->data[i]) 
		{
			pr_err("%s:ram check error addr=0x%04x, file_data=0x%02X, ram_data=0x%02X\n", FSERROR, i, fs3001_cont->data[i], reg_val);
			ret = -1;
			break;
		}
		check_sum += reg_val;
	}
	if (!ret) 
	{
		fs3001_i2c_read(fs3001, FS3001_WFSBASE_L, &reg_val);
		check_sum += reg_val;
		fs3001_i2c_read(fs3001, FS3001_WFSBASE_H, &reg_val);
		check_sum += reg_val & 0x0f;

		if (check_sum != fs3001->ram.check_sum) 
		{
			pr_err("%s:ram data check sum error, check_sum=0x%04x\n", FSERROR, check_sum);
			ret = -1;
		} 
		else 
		{
			pr_info("ram data check sum pass, check_sum=0x%04x\n", check_sum);
		}
	}

#endif
	//RAMINIT Disable
	fs3001_haptic_raminit(fs3001, false);
	mutex_unlock(&fs3001->lock);
	pr_info("exit\n");
	return ret;
}

static void fs3001_ram_loaded(const struct firmware *cont, void *context)
{
	struct fs3001 *fs3001 = context;
	struct fs3001_container *fs3001_fw;
	unsigned short check_sum = 0;
	int i = 0;
	int ret = 0;
#ifdef FS_READ_BIN_FLEXBALLY
	static unsigned char load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	pr_info("enter\n");
	if (!cont) 
	{
		pr_err("%s:failed to read %s\n", FSERROR, fs3001_ram_name);
		release_firmware(cont);
#ifdef FS_READ_BIN_FLEXBALLY
		if (load_cont <= 20) 
		{
			schedule_delayed_work(&fs3001->ram_work,msecs_to_jiffies(ram_timer_val));
			pr_info("start hrtimer: load_cont=%d\n", load_cont);
		}
#endif
		return;
	}
	pr_info("loaded %s - size: %zu bytes\n",fs3001_ram_name, cont ? cont->size : 0);
	
//for debug	xxxx, print bin file content
//	for(i=0; i < cont->size; i++) {
//		pr_info("addr: 0x%04x, data: 0x%02X\n",i, *(cont->data+i));
//	}

	// check sum
	for (i = 2; i < cont->size; i++)
	{
		check_sum += cont->data[i];
	}

	if (check_sum != (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) 
	{
		pr_err("%s:check sum err: check_sum=0x%04x\n", FSERROR,check_sum);
		return;
	} 
	else 
	{
		pr_info("check sum pass: 0x%04x\n", check_sum);
		fs3001->ram.check_sum = check_sum;
	}

	//fs3001 ram update less then 128kB
	fs3001_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!fs3001_fw) 
	{
		release_firmware(cont);
		pr_err("%s:error allocating memory\n", FSERROR);
		return;
	}
	fs3001_fw->len = cont->size;
	memcpy(fs3001_fw->data, cont->data, cont->size);
	release_firmware(cont);
	ret = fs3001_container_update(fs3001, fs3001_fw);
	if (ret) 
	{
		kfree(fs3001_fw);
		fs3001->ram.len = 0;
		pr_err("%s:ram firmware update failed!\n", FSERROR);
	} 
	else 
	{
		fs3001->ram_init = 1;
		fs3001->ram.len = fs3001_fw->len;
		kfree(fs3001_fw);
		pr_info("ram firmware update complete!\n");
	}
}

static int fs3001_ram_update(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	fs3001->ram_init = 0;
	fs3001->rtp_init = 0;

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,fs3001_ram_name, fs3001->dev,GFP_KERNEL, fs3001, fs3001_ram_loaded);
}

//compare fs3001->microsecond and theory_time, get fs3001->osc_cali_data and set this value to osc
static int fs3001_rtp_trim_lra_calibration(struct fs3001 *fs3001)
{
	unsigned char fre_val = 0;
	unsigned int theory_time = 0;
	unsigned int lra_trim_code = 0;
	pr_info("enter\n");

	fre_val = fs3001_i2c_read_bits(fs3001, FS3001_SYSCTRL, 5,4);

	if (fre_val == 2 || fre_val == 3)
		theory_time = (fs3001->rtp_len / 48000) * 1000000;	//48K
	if (fre_val == 0)
		theory_time = (fs3001->rtp_len / 12000) * 1000000;	//12K
	if (fre_val == 1)
		theory_time = (fs3001->rtp_len / 24000) * 1000000;	//24K

	pr_info("microsecond:%ld  theory_time = %d\n", fs3001->microsecond, theory_time);

	lra_trim_code = fs3001_osc_trim_calculation(theory_time,fs3001->microsecond);
	if (lra_trim_code >= 0) 
	{
		fs3001->osc_cali_data = lra_trim_code;
		fs3001_haptic_upload_lra(fs3001, FS3001_OSC_CALI);
	}
	return 0;
}

static enum hrtimer_restart fs3001_vibrator_timer_func(struct hrtimer *timer)
{
	struct fs3001 *fs3001 = container_of(timer, struct fs3001, timer);

	pr_info("enter\n");
	fs3001->state = 0;
	schedule_work(&fs3001->long_vibrate_work);

	return HRTIMER_NORESTART;
}

static int fs3001_haptic_play_repeat_seq(struct fs3001 *fs3001,unsigned char flag)
{
	pr_info("enter\n");

	if (flag) 
	{
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RAM_LOOP_MODE);
		fs3001_haptic_play_go(fs3001);
	}
	return 0;
}

static int fs3001_haptic_trig_config(struct fs3001 *fs3001)
{

	pr_info("enter\n");

	if (fs3001->isUsedIntn == false) 
	{
		fs3001_i2c_write_bits_1(fs3001, FS3001_INTCTRL,0,7,7);
	}

	return 0;
}

//motor protect
static int fs3001_haptic_swicth_motor_protect_config(struct fs3001 *fs3001,bool flag)
{
	pr_info("enter\n");
	fs3001_i2c_write_bits_1(fs3001, FS3001_DCCTRL,flag? 1:0,7,7);
	return 0;
}

static void fs3001_haptic_set_f0_ref(struct fs3001 *fs3001)
{
	unsigned int i_temp = FS3001_BASE_FRE * 10 / fs3001->dts_info.fs3001_f0_ref;
	pr_info("enter\n");
	fs3001_i2c_write(fs3001, FS3001_F0SET_L, i_temp & 0xFF);
	fs3001_i2c_write(fs3001, FS3001_F0SET_H, (i_temp>>8) & 0xFF);
}

static unsigned int fs3001_haptic_cal_0x7B_value(long long ll_f0,long long ll_f0_ref)
{
	unsigned int ui_0x7B = 0x90;
	long long ll_osc_deviation = 0;

	pr_info("enter: f0=%lld,f0_ref=%lld\n", ll_f0, ll_f0_ref);

	ll_f0 = ll_f0 * 1000000000;

	ll_osc_deviation = (ll_f0 / ll_f0_ref) - 1000000000;
	ui_0x7B = (unsigned int)((ll_osc_deviation + 257064068) / 1776596);

	if (ui_0x7B<0 || ui_0x7B>255)
	{
		pr_err("%s:f0_ref setting is over current haptic range\n",FSERROR);
		return 0x90;
	}
	pr_info("need to set 0x7B=0x%02X\n", ui_0x7B);
	return ui_0x7B;

}


static int fs3001_haptic_f0_calibration(struct fs3001 *fs3001)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int ui_0x7B = 0x90;

	pr_info("enter\n");

	//set f0 ref
	fs3001_haptic_set_f0_ref(fs3001);
	fs3001_i2c_write(fs3001, FS3001_FTUNECFG, 0x90);//cali will fail if  FTUNECFG == 0 or 0xff

	fs3001_haptic_upload_lra(fs3001,FS3001_WRITE_ZERO);
	if(fs3001->dts_info.fs3001_f0_cali_mode == FS3001_F0_CALI_MODE_AUTO)
	{
		// f0 calibrate work mode
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_F0_CALI_MODE);
		fs3001_haptic_cont_get_f0(fs3001);

		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_STANDBY_MODE);

		//save f0_cali_data
		fs3001_i2c_read(fs3001, FS3001_FTUNEDS, &reg_val);
		fs3001->f0_cali_data = reg_val;
		
		fs3001_haptic_upload_lra(fs3001, FS3001_F0_CALI);
		pr_info("f0_cali_data=0x%02X\n",fs3001->f0_cali_data);
	}
	else if(fs3001->dts_info.fs3001_f0_cali_mode == FS3001_F0_CALI_MODE_FORMULA)
	{
		// enter DETECT_MODE
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_F0_DETECT_MODE);
	
		fs3001_haptic_cont_get_f0(fs3001);//get f0
		ui_0x7B = fs3001_haptic_cal_0x7B_value(fs3001->f0, fs3001->dts_info.fs3001_f0_ref);
		//save f0_cali_data
		fs3001->f0_cali_data = ui_0x7B;
		//set new osc
		fs3001_haptic_upload_lra(fs3001, FS3001_F0_CALI);
		fs3001_haptic_cont_get_f0(fs3001);//get new f0
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_STANDBY_MODE);
		
	}

	fs3001_haptic_stop(fs3001);
	return ret;
}

//haptic cont
static int fs3001_haptic_cont_config(struct fs3001 *fs3001)
{
	pr_info("enter\n");

	//work mode
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_CONT_MODE);
	//cont play go
	fs3001_haptic_play_go(fs3001);
	return 0;
}

static int fs3001_haptic_play_wav_seq(struct fs3001 *fs3001, unsigned char flag)
{
	pr_info("enter\n");
	if (flag) 
	{
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RAM_MODE);
		fs3001_haptic_play_go(fs3001);
	}
	return 0;
}

#ifdef TIMED_OUTPUT
static int fs3001_vibrator_get_time(struct timed_output_dev *dev)
{
	struct fs3001 *fs3001 = container_of(dev, struct fs3001, vib_dev);
	pr_info("enter\n");
	if (hrtimer_active(&fs3001->timer)) 
	{
		ktime_t r = hrtimer_get_remaining(&fs3001->timer);
		return ktime_to_ms(r);
	}
	return 0;
}

static void fs3001_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct fs3001 *fs3001 = container_of(dev, struct fs3001, vib_dev);

	pr_info( "enter\n");
	mutex_lock(&fs3001->lock);
	fs3001_haptic_stop(fs3001);
	if (value > 0) 
	{
		fs3001_haptic_ram_vbat_compensate(fs3001, false);
		fs3001_haptic_play_wav_seq(fs3001, value);
	}
	mutex_unlock(&fs3001->lock);
	pr_info("exit\n");
}
#else
static enum led_brightness fs3001_haptic_brightness_get(struct led_classdev*cdev)
{
	struct fs3001 *fs3001 = container_of(cdev, struct fs3001, vib_dev);
	pr_info("enter\n");
	return fs3001->amplitude;
}

static void fs3001_haptic_brightness_set(struct led_classdev *cdev, enum led_brightness level)
{
	struct fs3001 *fs3001 = container_of(cdev, struct fs3001, vib_dev);

	pr_info("enter\n");
	if (!fs3001->ram_init) 
	{
		pr_err("%s:ram init failed, not allow to play!\n", FSERROR);
		return;
	}
	if (fs3001->ram_update_flag < 0)
	{
		return;
	}
		
	fs3001->amplitude = level;
	mutex_lock(&fs3001->lock);
	fs3001_haptic_stop(fs3001);
	if (fs3001->amplitude > 0) 
	{
		fs3001_haptic_upload_lra(fs3001, FS3001_F0_CALI);
		fs3001_haptic_ram_vbat_compensate(fs3001, false);
		fs3001_haptic_play_wav_seq(fs3001, fs3001->amplitude);
	}
	mutex_unlock(&fs3001->lock);
}
#endif

static int
fs3001_haptic_audio_ctr_list_insert(struct haptic_audio *haptic_audio,struct haptic_ctr *haptic_ctr,struct device *dev)
{
	struct haptic_ctr *p_new = NULL;
	pr_info("enter\n");

	p_new = (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr), GFP_KERNEL);
	if (p_new == NULL) 
	{
		pr_err("%s:kzalloc memory fail\n", FSERROR);
		return -1;
	}
	//update new list info
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;

	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));
	return 0;
}

static int
fs3001_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	pr_info("enter\n");

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,&(haptic_audio->ctr_list),list) 
	{
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}

	return 0;
}

static int fs3001_haptic_audio_off(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	mutex_lock(&fs3001->lock);
	fs3001_haptic_set_gain(fs3001, 0x80);
	fs3001_haptic_stop(fs3001);
	fs3001->gun_type = 0xff;
	fs3001->bullet_nr = 0;
	fs3001_haptic_audio_ctr_list_clear(&fs3001->haptic_audio);
	mutex_unlock(&fs3001->lock);
	return 0;
}

static int fs3001_haptic_audio_init(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	fs3001_haptic_set_wav_seq(fs3001, 0x01, 0x00);

	return 0;
}

static int fs3001_haptic_activate(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	fs3001_interrupt_clear(fs3001);
	//xxxx_?? open UV INT
	fs3001_i2c_write_bits_1(fs3001, FS3001_INTMASK1,0,6,6);
	return 0;
}

static int fs3001_haptic_start(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	fs3001_haptic_activate(fs3001);
	fs3001_haptic_play_go(fs3001);
	return 0;
}


static void fs3001_haptic_audio_work_routine(struct work_struct *work)
{
	struct fs3001 *fs3001 = container_of(work,struct fs3001,haptic_audio.work);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	unsigned int ctr_list_flag = 0;
	unsigned int ctr_list_input_cnt = 0;
	unsigned int ctr_list_output_cnt = 0;
	unsigned int ctr_list_diff_cnt = 0;
	unsigned int ctr_list_del_cnt = 0;
	int rtp_is_going_on = 0;

	pr_info("enter\n");

	haptic_audio = &(fs3001->haptic_audio);
	mutex_lock(&fs3001->haptic_audio.lock);
	memset(&fs3001->haptic_audio.ctr, 0, sizeof(struct haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,&(haptic_audio->ctr_list), list) 
	{
		ctr_list_flag = 1;
		break;
	}
	if (ctr_list_flag == 0)
		pr_info("ctr list empty\n");

	if (ctr_list_flag == 1) 
	{
		list_for_each_entry_safe(p_ctr, p_ctr_bak,&(haptic_audio->ctr_list), list) 
		{
			ctr_list_input_cnt =  p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,&(haptic_audio->ctr_list), list) 
		{
			ctr_list_output_cnt =  p_ctr->cnt;
			break;
		}
		if (ctr_list_input_cnt > ctr_list_output_cnt)
			ctr_list_diff_cnt = ctr_list_input_cnt - ctr_list_output_cnt;

		if (ctr_list_input_cnt < ctr_list_output_cnt)
			ctr_list_diff_cnt = 32 + ctr_list_input_cnt - ctr_list_output_cnt;

		pr_info("ctr_list_input_cnt=%d,ctr_list_output_cnt=%d\n",ctr_list_input_cnt,ctr_list_output_cnt);
		if (ctr_list_diff_cnt > 2) 
		{
			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,&(haptic_audio->ctr_list), list) 
			{
				if ((p_ctr->play == FS3001_HAPTIC_CMD_NULL) &&(FS3001_HAPTIC_CMD_ENABLE ==(FS3001_HAPTIC_CMD_HAPTIC & p_ctr->cmd))) 
				{
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt == ctr_list_diff_cnt)
					break;
			}
		}
	}

	//get the last data from list
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,&(haptic_audio->ctr_list), list) 
	{
		fs3001->haptic_audio.ctr.cnt = p_ctr->cnt;
		fs3001->haptic_audio.ctr.cmd = p_ctr->cmd;
		fs3001->haptic_audio.ctr.play = p_ctr->play;
		fs3001->haptic_audio.ctr.wavseq = p_ctr->wavseq;
		fs3001->haptic_audio.ctr.loop = p_ctr->loop;
		fs3001->haptic_audio.ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}

	if (fs3001->haptic_audio.ctr.play) 
	{
		pr_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",fs3001->haptic_audio.ctr.cnt,fs3001->haptic_audio.ctr.cmd,fs3001->haptic_audio.ctr.play,fs3001->haptic_audio.ctr.wavseq,fs3001->haptic_audio.ctr.loop,fs3001->haptic_audio.ctr.gain);
	}

	//rtp mode jump
	rtp_is_going_on = fs3001_haptic_juge_RTP_is_going_on(fs3001);
	if (rtp_is_going_on) 
	{
		mutex_unlock(&fs3001->haptic_audio.lock);
		return;
	}
	mutex_unlock(&fs3001->haptic_audio.lock);

	//haptic play control
	if (FS3001_HAPTIC_CMD_ENABLE == (FS3001_HAPTIC_CMD_HAPTIC & fs3001->haptic_audio.ctr.cmd)) 
	{
		if (FS3001_HAPTIC_PLAY_ENABLE == fs3001->haptic_audio.ctr.play) 
		{
			pr_info("haptic_audio_play_start\n");
			pr_info("normal haptic start\n");
			mutex_lock(&fs3001->lock);
			fs3001_haptic_stop(fs3001);
			fs3001_haptic_play_mode(fs3001,FS3001_HAPTIC_RAM_MODE);
			fs3001_haptic_set_wav_seq(fs3001, 0x00,fs3001->haptic_audio.ctr.wavseq);
			fs3001_haptic_set_wav_loop(fs3001, 0x00,fs3001->haptic_audio.ctr.loop);
			fs3001_haptic_set_gain(fs3001,fs3001->haptic_audio.ctr.gain);
			fs3001_haptic_start(fs3001);
			mutex_unlock(&fs3001->lock);
		} 
		else if (FS3001_HAPTIC_PLAY_STOP == fs3001->haptic_audio.ctr.play) 
		{
			mutex_lock(&fs3001->lock);
			fs3001_haptic_stop(fs3001);
			mutex_unlock(&fs3001->lock);
		} 
		else if (FS3001_HAPTIC_PLAY_GAIN == fs3001->haptic_audio.ctr.play) 
		{
			mutex_lock(&fs3001->lock);
			fs3001_haptic_set_gain(fs3001,fs3001->haptic_audio.ctr.gain);
			mutex_unlock(&fs3001->lock);
		}
	}
}

// show lra f0, make a cont go to get f0 and cont f0, but only return f0(cont f0 not returned)
// Why set FTUNECFG (7Bh) to 0x90? Because the OSC corresponding to 0x90 is trimmed and accurate
//vibrate
// cat f0
// 1723
static ssize_t fs3001_f0_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;

	pr_info("enter");

	mutex_lock(&fs3001->lock);

	fs3001_haptic_upload_lra(fs3001, FS3001_WRITE_ZERO);
	// f0 detect mode
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_F0_DETECT_MODE);
	fs3001_haptic_cont_get_f0(fs3001);
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_STANDBY_MODE);
	
	mutex_unlock(&fs3001->lock);
	len += snprintf(buf + len, PAGE_SIZE - len,"%d\n", fs3001->f0);
	return len;
}


//show fs3001->f0 with f0_cali_data osc
//In this case, F0 is detected again with the new OSC after Cali, and the result obtained at this time should be F0_ref
//cat cali
//2350
static ssize_t fs3001_cali_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	mutex_lock(&fs3001->lock);
	
	fs3001_haptic_upload_lra(fs3001, FS3001_F0_CALI);
	// f0 detect mode
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_F0_DETECT_MODE);
	fs3001_haptic_cont_get_f0(fs3001);
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_STANDBY_MODE);
	mutex_unlock(&fs3001->lock);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", fs3001->f0);
	return len;
}

//Perform an F0 calibration
//echo 1 > cali   [hex is ok]
static ssize_t fs3001_cali_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) 
	{
		mutex_lock(&fs3001->lock);
		fs3001_haptic_f0_calibration(fs3001);
		mutex_unlock(&fs3001->lock);
	}
	return count;
}


//Show fs3001->f0_cali_data
//cat f0_save
//f0_cali_data = 0x83
static ssize_t fs3001_f0_save_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 0x%02X\n",fs3001->f0_cali_data);

	return len;
}

//show fs3001->dts_info.fs3001_f0_cali_mode
//cat cali_mode
//cali_mode = 0
static ssize_t fs3001_cali_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"cali_mode = %d\n",fs3001->dts_info.fs3001_f0_cali_mode);
	return len;

}

//set cali_mode
//cat cali_mode
//cali_mode = 0
//echo 1 > cali_mode		[hex is ok:echo 0x01 > cali_mode ]
//cali_mode = 1
static ssize_t fs3001_cali_mode_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	
	mutex_lock(&fs3001->lock);
	if(val == 0 || val == 1)
	{
		fs3001->dts_info.fs3001_f0_cali_mode = val;
	}
	else
	{
		pr_err("%s:input value out of range (val=%d, should be 0 or 1)\n", FSERROR, val);
	}
	
	mutex_unlock(&fs3001->lock);

	return count;
}


//set fs3001->f0_cali_data
//cat f0_save
//f0_cali_data = 0x83
//echo 144>f0_save		[hex is ok: echo 0x90 > f0_save		]
//cat f0_save
//f0_cali_data = 0x90
static ssize_t fs3001_f0_save_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	fs3001->f0_cali_data = val;
	return count;
}

//Show fs3001->osc_cali_data
//cat osc_save
//osc_cali_data = 0x90
static ssize_t fs3001_osc_save_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "osc_cali_data = 0x%02X\n",fs3001->osc_cali_data);

	return len;
}

//set fs3001->osc_cali_data
//cat osc_save
//osc_cali_data = 0x90
//echo 143 > osc_save    [hex is ok: echo 0x8f > osc_save]
//cat osc_save
//osc_cali_data = 0x8F
static ssize_t fs3001_osc_save_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	fs3001->osc_cali_data = val;
	return count;
}


//show f0_ref
//cat f0_ref
//f0_ref = 2350
static ssize_t fs3001_f0_ref_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"f0_ref = %d\n",fs3001->dts_info.fs3001_f0_ref);
	return len;

}

//set f0_ref[generally, after this step, we need to call "echo 1 > cali" to calibrate the motor again]
//cat f0_ref
//f0_ref = 2350
//echo 2222 > f0_ref		[hex is ok:echo 0x8AE > f0_ref ]
//f0_ref = 2222
static ssize_t fs3001_f0_ref_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	
	mutex_lock(&fs3001->lock);
	fs3001->dts_info.fs3001_f0_ref = val;
	mutex_unlock(&fs3001->lock);

	return count;
}

//play a 10s rtp file, then get fs3001->microsecond
//compare fs3001->microsecond and theory_time, get fs3001->osc_cali_data and set this value to osc
//cat osc_cali
//osc_cali_data = 0x90
//echo 1 > osc_cali
//cat osc_cali
//osc_cali_data = 0x90
static ssize_t fs3001_osc_cali_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&fs3001->lock);
	fs3001->osc_cali_run = 1;
	if (val == 1) 
	{
		//set osc to default value(FS3001_FTUNECFG=0x90,osc to 384000)
		fs3001_haptic_upload_lra(fs3001, FS3001_WRITE_ZERO);
		//play a 10s rtp file, then get fs3001->microsecond
		fs3001_rtp_osc_calibration(fs3001);
		//compare fs3001->microsecond and theory_time, get fs3001->osc_cali_data and set this value to osc
		fs3001_rtp_trim_lra_calibration(fs3001);
	}
	//set osc to osc_cali_data, then play a 10s rtp file, get fs3001->microsecond
	//just for observing the debug message
	else if (val == 2) 
	{
		fs3001_haptic_upload_lra(fs3001, FS3001_OSC_CALI);
		fs3001_rtp_osc_calibration(fs3001);
	}
	else 
	{
		pr_err("%s:input value out of range\n", FSERROR);
	}
	fs3001->osc_cali_run = 0;
	//osc calibration flag end, other behaviors are permitted
	mutex_unlock(&fs3001->lock);

	return count;
}

static ssize_t fs3001_state_show(struct device *dev,struct device_attribute *attr, char *buf)
{
       struct fs3001 *fs3001 = g_foursemi->fs3001;
 
       return snprintf(buf, PAGE_SIZE, "%d\n", fs3001->state);
}
 
static ssize_t fs3001_state_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
       return count;
}


//show fs3001->activate_mode and fs3001->state
//cat activate
//fs3001->activate = 3 fs3001->state=0
static ssize_t fs3001_activate_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;

	pr_info("enter");

	//For now nothing to show
	return snprintf(buf, PAGE_SIZE, "fs3001->activate_mode = %d fs3001->state=%d\n", fs3001->activate_mode,fs3001->state);
}

//schedule_work(&fs3001->long_vibrate_work);-->fs3001_long_vibrate_work_routine
// By setting FS3001 ->duration, you can control the duration of RAM loop playback
//echo 2000 > duration   set 2000ms
//echo 3 > activate_mode    [hex is ok:echo 0x02 > activate_mode]
//echo 1 > activate			[hex is ok:echo 0x01 > activate]

//enum fs3001_haptic_activate_mode {
//	FS3001_HAPTIC_ACTIVATE_RAM_MODE = 0,
//	FS3001_HAPTIC_ACTIVATE_CONT_MODE = 1,
//    FS3001_HAPTIC_ACTIVATE_RTP_MODE = 2,
//	FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE = 3,
//};

static ssize_t fs3001_activate_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;

	pr_info("enter");

	if (!fs3001->ram_init) 
	{
		pr_err("%s:ram init failed, not allow to play!\n", FSERROR);
		return count;
	}
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
	{
		pr_err("%s:kstrtouint fail\n",FSERROR);
		return rc;
	}
	
	pr_info("value=%d\n", val);
	if (val != 0 && val != 1)
	{
		pr_err("%s:fs3001_activate_store value must be 0 or 1\n",FSERROR);
		return count;
	}

	mutex_lock(&fs3001->lock);
	hrtimer_cancel(&fs3001->timer);
	fs3001->state = val;
	mutex_unlock(&fs3001->lock);
	schedule_work(&fs3001->long_vibrate_work);
	return count;
}



//show fs3001->activate_mode
//cat activate_mode
//fs3001->activate_mode = 2
static ssize_t fs3001_activate_mode_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	pr_info("enter\n");

	return snprintf(buf, PAGE_SIZE, "fs3001->activate_mode = %d\n",fs3001->activate_mode);
}


//echo 2 > activate_mode			//set fs3001->activate_mode
//cat activate_mode
//fs3001->activate_mode = 2			[hex is ok:echo 0x02 > activate_mode]

//enum fs3001_haptic_activate_mode {
//	FS3001_HAPTIC_ACTIVATE_RAM_MODE = 0,
//	FS3001_HAPTIC_ACTIVATE_CONT_MODE = 1,
//    FS3001_HAPTIC_ACTIVATE_RTP_MODE = 2,
//	FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE = 3,
//};

static ssize_t fs3001_activate_mode_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
	{
		pr_err("%s:input value out of range\n", FSERROR);
		return rc;
	}
		
	mutex_lock(&fs3001->lock);
	fs3001->activate_mode = val;
	mutex_unlock(&fs3001->lock);
	return count;
}


// Just show tracking F0 (no vibration), only read the register and get tracking F0
//cat cont
//1723 (Because can't handle decimals, need to divide the result by 10)

static ssize_t fs3001_cont_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;

	pr_info("enter");

	fs3001_haptic_read_cont_f0(fs3001);
	len += snprintf(buf + len, PAGE_SIZE - len,"%d\n", fs3001->cont_f0);
	return len;
}

// Make a cont go
//echo 1 > cont
static ssize_t fs3001_cont_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;

	pr_info("enter");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	fs3001_haptic_stop(fs3001);
	if (val)
		fs3001_haptic_cont_config(fs3001);
	return count;
}


// Displays the number of transmitted bytes of the current RTP fs3001-> rtp_cnt
//cat rtp
//rtp_cnt = 0
//echo 172 > rtp
//cat rtp
//rtp_cnt = 37376    //it's playing, so it can show a not zero number
//cat rtp
//rtp_cnt = 0			//The rtp playing is over, so it shows 0
static ssize_t fs3001_rtp_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"rtp_cnt = %d\n",fs3001->rtp_cnt);
	return len;
}

//example:echo 41 > rtp [hex is ok: echo 0x29 > rtp]
// Play an RTP
//sizeof(fs3001_rtp_name) / FS3001_RTP_NAME_MAX is total rtp files number 173(0--172) 
static ssize_t fs3001_rtp_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) 
	{
		pr_err("%s:kstrtouint fail\n", FSERROR);
		return rc;
	}
	mutex_lock(&fs3001->lock);
	fs3001_haptic_stop(fs3001);
	// Set the INT of almost emtpy. This is set in the fs3001_rtp_work_routine function, so it is not needed here
	//fs3001_haptic_set_rtp_aei(fs3001, false);
	//fs3001_interrupt_clear(fs3001);
	if (val < (sizeof(fs3001_rtp_name) / FS3001_RTP_NAME_MAX)) 
	{
		fs3001->rtp_file_num = val;
		pr_info("fs3001_rtp_name[%d]: %s\n",val, fs3001_rtp_name[val]);
		schedule_work(&fs3001->rtp_work);
	}
	else
	{
		pr_err("%s:rtp_file_num 0x%02X over max value\n", FSERROR, fs3001->rtp_file_num);
	}
	mutex_unlock(&fs3001->lock);
	return count;
}


// Display all register contents
//cat register
static ssize_t fs3001_reg_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	pr_info("enter");
	fs3001_haptic_enable_key(fs3001,true);
	for (i = 0; i <= FS3001_REG_MAX; i++) 
	{
		fs3001_i2c_read(fs3001, i, &reg_val);
		if((i+1) % 8 == 0)
		{
			len += snprintf(buf + len, PAGE_SIZE - len,"0x%02X=0x%02X\n", i, reg_val);
		}
		else
		{
			len += snprintf(buf + len, PAGE_SIZE - len,"0x%02X=0x%02X ", i, reg_val);
		}
	}
	fs3001_haptic_enable_key(fs3001,false);
	pr_info("PAGE_SIZE=%d len=%d\n",(int)PAGE_SIZE,(int)len);
	return len;
}

//set a register  [only hex]
//echo 9 2 > register  (set 0x2 to address 0x9)
static ssize_t fs3001_reg_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[2] = { 0, 0 };

	pr_info("enter");
	fs3001_haptic_enable_key(fs3001,true);

	if(sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
	{
		fs3001_i2c_write(fs3001, (unsigned char)databuf[0],(unsigned char)databuf[1]);
	}
	fs3001_haptic_enable_key(fs3001,false);

	return count;
}

//set multi registers  [only hex]
//echo abcd 1234 ffff ffff ffff ffff ffff ffff ffff ffff  > registers  (set 0xcd to address 0xab, set 0x34 to address 0x12
//reg count must be 10, ffff is dull commnd
static ssize_t fs3001_regs_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int i = 0;
	pr_info("enter");

	if(sscanf(buf, "%x %x %x %x %x %x %x %x %x %x", &databuf[0], &databuf[1], &databuf[2], &databuf[3], &databuf[4], &databuf[5], &databuf[6], &databuf[7], &databuf[8], &databuf[9]) == 10)
	{
		for (i = 0;i < ARRAY_SIZE(databuf);i++)
		{
			if(databuf[i] != 0xffff)
			{
				fs3001_i2c_write(fs3001, (unsigned char)((databuf[i]>>8) & 0xff),(unsigned char)(databuf[i] & 0xff));
			}
		}
	}
	else
		pr_info("cmd count must be 10");

	return count;
}


//show fs3001->dts_info.fs3001_reg_inits contents
//cat reg_inits
//0x927000
//0x934003
//0x5f7080
//0xffffff
//0xffffff
//0xffffff
//0xffffff
//0xffffff
//0xffffff
//0xffffff

//0x5f0001			write 0x01 to 0x5f, start bit=0, len=1(0x1->bit0)
//0x5f3403			write 0x03 to 0x5f, start bit=4, len=4(0x3->bit7-4)
//0x5f7080			write 0x80 to 0x5f, start bit=0, len=8(0x80->bit7-0)

static ssize_t fs3001_reg_inits_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	int i = 0;
	pr_info("enter\n");
	for (i = 0;i < ARRAY_SIZE(fs3001->dts_info.fs3001_reg_inits);i++)
	{
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%x\n", fs3001->dts_info.fs3001_reg_inits[i]);
	}

	return len;
}


 
// show duration and fs3001->timer remaining
//cat duration
//fs3001->timer remaining = 0, fs3001->duration = 2000
static ssize_t fs3001_duration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ktime_t time_rem;
	s64 time_ms = 0;

	pr_info("enter");

	// If the current timer is in active state
	if (hrtimer_active(&fs3001->timer)) 
	{
		time_rem = hrtimer_get_remaining(&fs3001->timer);
		time_ms = ktime_to_ms(time_rem);
	}
	return snprintf(buf, PAGE_SIZE, "fs3001->timer remaining = %lld, fs3001->duration = %d\n", time_ms,fs3001->duration);
}

// By setting FS3001 ->duration, you can control the duration of RAM loop playback
//enum fs3001_haptic_activate_mode {
//	FS3001_HAPTIC_ACTIVATE_RAM_MODE = 0,
//	FS3001_HAPTIC_ACTIVATE_CONT_MODE = 1,
//    FS3001_HAPTIC_ACTIVATE_RTP_MODE = 2,
//	FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE = 3,
//};
//echo 2000 > duration   set 2000ms [hex is ok:echo 0xff > duration]
//echo 3 > activate_mode
//echo 1 > activate
static ssize_t fs3001_duration_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;

	pr_info("enter");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	//setting 0 on duration is NOP for now
	if (val <= 0)
		return count;
	fs3001_haptic_duration_ram_play_config(fs3001,val);
	fs3001->duration = val;
	return count;
}


//show current seq
//cat seq
//seq1: 0x01
//seq2: 0x00
//seq3: 0x00
//seq4: 0x00
//seq5: 0x00
//seq6: 0x00
//seq7: 0x00
//seq8: 0x00
static ssize_t fs3001_seq_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	pr_info("enter\n");

	for (i = 0; i < FS3001_SEQUENCER_SIZE; i++) 
	{
		fs3001_i2c_read(fs3001, FS3001_WFSCFG1 + i, &reg_val);
		count += snprintf(buf + count, PAGE_SIZE - count,"seq%d: 0x%02x\n", i + 1, reg_val & 0x7f);
		fs3001->seq[i] |= reg_val;
	}
	return count;
}

//[only hex]
// Set seq. Note that the first parameter ranges from 0 to 7
//cat seq
//seq1: 0x01
//seq2: 0x00
//seq3: 0x00
//seq4: 0x00
//seq5: 0x00
//seq6: 0x00
//seq7: 0x00
//seq8: 0x00
//echo 1 1f > seq
//cat seq
//seq1: 0x01
//seq2: 0x1f
//seq3: 0x00
//seq4: 0x00
//seq5: 0x00
//seq6: 0x00
//seq7: 0x00
//seq8: 0x00

static ssize_t fs3001_seq_store(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[2] = { 0, 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) 
	{
		if (databuf[0] >= FS3001_SEQUENCER_SIZE) 
		{
			pr_err("%s:input value out of range\n", FSERROR);
			return count;
		}
		pr_info("seq%d=0x%02X\n",databuf[0], databuf[1]);
		mutex_lock(&fs3001->lock);
		fs3001->seq[databuf[0]] = (unsigned char)databuf[1];
		fs3001_haptic_set_wav_seq(fs3001, (unsigned char)databuf[0],fs3001->seq[databuf[0]]);
		mutex_unlock(&fs3001->lock);
	}
	return count;
}

//[only hex]
// Cat loop displays the value of 8 RAM play loops
//seq1_loop = 0x0f
//seq2_loop = 0x00
//seq3_loop = 0x00
//seq4_loop = 0x00
//seq5_loop = 0x00
//seq6_loop = 0x00
//seq7_loop = 0x00
//seq8_loop = 0x00
static ssize_t fs3001_loop_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	pr_info("enter\n");

	for (i = 0; i < FS3001_SEQUENCER_LOOP_SIZE; i++) 
	{
		fs3001_i2c_read(fs3001, FS3001_WFSLOOP1 + i, &reg_val);
		fs3001->loop[i * 2 + 0] = (reg_val >> 0) & 0x0F;
		fs3001->loop[i * 2 + 1] = (reg_val >> 4) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,"seq%d_loop = 0x%02x\n", i * 2 + 1,fs3001->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,"seq%d_loop = 0x%02x\n", i * 2 + 2,fs3001->loop[i * 2 + 1]);
	}
	return count;
}

// Set loop. Note that the first parameter ranges from 0 to 7
//Cat loop
//seq1_loop = 0x0f
//seq2_loop = 0x00
//seq3_loop = 0x00
//seq4_loop = 0x00
//seq5_loop = 0x00
//seq6_loop = 0x00
//seq7_loop = 0x00
//seq8_loop = 0x00
//echo 1 e > loop
//cat loop
//seq1_loop = 0x0f
//seq2_loop = 0x0e
//seq3_loop = 0x00
//seq4_loop = 0x00
//seq5_loop = 0x00
//seq6_loop = 0x00
//seq7_loop = 0x00
//seq8_loop = 0x00
static ssize_t fs3001_loop_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[2] = { 0, 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) 
	{
		if (databuf[0] >= FS3001_SEQUENCER_SIZE) 
		{
			pr_err("%s:input value out of range\n", FSERROR);
			return count;
		}
		pr_info("seq%d loop=0x%02X\n",databuf[0], databuf[1]);
		mutex_lock(&fs3001->lock);
		fs3001->loop[databuf[0]] = (unsigned char)databuf[1];
		fs3001_haptic_set_wav_loop(fs3001, (unsigned char)databuf[0],fs3001->loop[databuf[0]]);
		mutex_unlock(&fs3001->lock);
	}
	return count;
}


//show seq1
//cat index
//index = 1

static ssize_t fs3001_index_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned char reg_val = 0;
	pr_info("enter\n");

	reg_val = fs3001_i2c_read_bits(fs3001, FS3001_WFSCFG1, 6, 0);
	return snprintf(buf, PAGE_SIZE, "index = %d\n", reg_val);
}

// Set seq1 and set loop1 to 0xf
//cat index
//index = 1
//echo 2 > index [hex is ok:echo 0x02 > index]
//cat index
//index = 2
//cat loop
//seq1_loop = 0x0f
//seq2_loop = 0x00
//seq3_loop = 0x00
//seq4_loop = 0x00
//seq5_loop = 0x00
//seq6_loop = 0x00
//seq7_loop = 0x00
//seq8_loop = 0x00
static ssize_t fs3001_index_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
	{
		pr_err("%s:input value out of range\n", FSERROR);
		return rc;
	}

	pr_info("value=%d\n", val);
	mutex_lock(&fs3001->lock);
	fs3001_haptic_set_repeat_wav_seq(fs3001, val);
	mutex_unlock(&fs3001->lock);
	return count;
}

// Display the SRAM size
//cat sram_size
//sram_size = 3K
static ssize_t fs3001_sram_size_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	pr_info("enter\n");
	return snprintf(buf, PAGE_SIZE, "sram_size = 3K\n");
}


// Display the Settings of the GAIN (register FS3001 GAINCFG)
//cat gain
//0x80
static ssize_t fs3001_gain_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char reg = 0;
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	pr_info("enter\n");

	fs3001_i2c_read(fs3001, FS3001_GAINCFG, &reg);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", reg);
}

// set GAIN to FS3001 -> GAIN, but there is a conversion process (voltage compensation factor)
//cat gain
//0x80
//echo 126 > gain		[hex is ok: echo 0x7E > gain]
//cat gain
//0x7E
static ssize_t fs3001_gain_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
	{
		return rc;
	}

	pr_info("value=%d\n", val);
	if (val >= 0x80)
		val = 0x80;
	mutex_lock(&fs3001->lock);
	fs3001->gain = val + fs3001->dts_info.fs3001_gain_adjust;
	fs3001_haptic_set_gain(fs3001, fs3001->gain);
	mutex_unlock(&fs3001->lock);
	return count;
}

// Display the contents of the RAM bin (read from RAM, not directly read from the bin file)
//cat ram_update
//haptic_ram len = 893
//...
//...
static ssize_t fs3001_ram_update_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	unsigned int ram_shift = 4;
	pr_info("enter\n");

	//set to ram mode
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RAM_MODE);

	//RAMINIT Enable
	fs3001_haptic_raminit(fs3001, true);
	fs3001_haptic_stop(fs3001);

	fs3001_i2c_write_bits_1(fs3001, FS3001_RAMADDR_H,(unsigned char)(fs3001->ram.base_addr >> 8),3,0);
	fs3001_i2c_write(fs3001, FS3001_RAMADDR_L,(unsigned char)(fs3001->ram.base_addr & 0x00ff));

	len += snprintf(buf + len, PAGE_SIZE - len,"haptic_ram len = %d\n", fs3001->ram.len);
	for (i = 0; i < fs3001->ram.len - ram_shift; i++) 
	{
		fs3001_i2c_read(fs3001, FS3001_RAMRDATA, &reg_val);
		if ((i+1) % 16 == 0)
			len += snprintf(buf + len,PAGE_SIZE - len, "%02X\n", reg_val);
		else
			len += snprintf(buf + len,PAGE_SIZE - len, "%02X,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	// RAMINIT Disable
	fs3001_haptic_raminit(fs3001, false);
	return len;
}

// Reload the RAM bin file
//echo 1 > ram_update
static ssize_t fs3001_ram_update_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val)
	{
		fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RAM_MODE);
		fs3001_ram_update(fs3001);
	}
	return count;
}

//show fs3001->dts_info.fs3001_vbat_mode
//cat vbat_mode
//vbat_mode = 0
static ssize_t fs3001_vbat_mode_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_mode = %d\n",fs3001->dts_info.fs3001_vbat_mode);

	return len;
}

//0:disable  
//1:play enable(software gain)+brake disable  
//2:play enable(register configuration)+brake disable   
//3:play enable(auto-detected)+brake disable  
//4:play enable+brake enable (register configuration)  
//5:play enable+brake enable (auto-detected)

//set fs3001->dts_info.fs3001_vbat_mode
//cat vbat_mode
//vbat_mode = 0
//echo 1 > vbat_mode   [hex is ok]
//cat vbat_mode
//vbat_mode = 1
static ssize_t fs3001_vbat_mode_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&fs3001->lock);
	fs3001->dts_info.fs3001_vbat_mode = val;
	mutex_unlock(&fs3001->lock);

	return count;
}


//show fs3001_cont_drv1_lvl��fs3001_cont_drv2_lvl
//cat cont_drv_lvl
//cont_drv1_lvl = 0x7F
//cont_drv2_lvl = 0x36
static ssize_t fs3001_cont_drv_lvl_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"cont_drv1_lvl = 0x%02X\n",fs3001->dts_info.fs3001_cont_drv1_lvl);
	len += snprintf(buf + len, PAGE_SIZE - len,"cont_drv2_lvl = 0x%02X\n",fs3001->dts_info.fs3001_cont_drv2_lvl);
	return len;
}

//[only hex]
//set fs3001_cont_drv1_lvl��fs3001_cont_drv2_lvl
//cat cont_drv_lvl
//cont_drv1_lvl = 0x7F
//cont_drv2_lvl = 0x36
//echo 0x7f 044 > cont_drv_lvl
//cat cont_drv_lvl
//cont_drv1_lvl = 0x7F
//cont_drv2_lvl = 0x44
static ssize_t fs3001_cont_drv_lvl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[2] = { 0, 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) 
	{
		fs3001->dts_info.fs3001_cont_drv1_lvl = databuf[0];
		fs3001->dts_info.fs3001_cont_drv2_lvl = databuf[1];
		fs3001_i2c_write(fs3001, FS3001_DRVCFG1,fs3001->dts_info.fs3001_cont_drv1_lvl);
		fs3001_i2c_write(fs3001, FS3001_DRVCFG2,fs3001->dts_info.fs3001_cont_drv2_lvl);
	}
	return count;
}

//show cont_drv1_time��cont_drv2_time
//cat cont_drv_time
//cont_drv1_lvl = 0x04
//cont_drv2_time = 0x14
static ssize_t fs3001_cont_drv_time_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"cont_drv1_time = 0x%02X\n",fs3001->dts_info.fs3001_cont_drv1_time);
	len += snprintf(buf + len, PAGE_SIZE - len,"cont_drv2_time = 0x%02X\n",fs3001->dts_info.fs3001_cont_drv2_time);
	return len;
}

//[only hex]
//set cont_drv1_time��cont_drv2_time
//cat cont_drv_time
//cont_drv1_lvl = 0x04
//cont_drv2_time = 0x14
//echo 4 6 > cont_drv_time
//cont_drv1_time = 0x04
//cont_drv2_time = 0x06
static ssize_t fs3001_cont_drv_time_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[2] = { 0, 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) 
	{
		fs3001->dts_info.fs3001_cont_drv1_time = databuf[0];
		fs3001->dts_info.fs3001_cont_drv2_time = databuf[1];
		fs3001_i2c_write(fs3001, FS3001_DRVCFG3,fs3001->dts_info.fs3001_cont_drv1_time);
		fs3001_i2c_write(fs3001, FS3001_DRVCFG4,fs3001->dts_info.fs3001_cont_drv2_time);
	}
	return count;
}

//show fs3001->dts_info.fs3001_brk_times
//cat cont_brk_times
//cont_brk_times = 0x06
static ssize_t fs3001_brk_times_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len, "brk_times = 0x%02X\n",fs3001->dts_info.fs3001_brk_times);
	return len;
}

//[only hex]
//set brk cont brk times
//cat brk_times
//brk_times = 0x06
//echo 5 > brk_times
//brk_times = 0x05
static ssize_t fs3001_brk_times_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[1] = { 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x", &databuf[0]) == 1) 
	{
		fs3001->dts_info.fs3001_brk_times = databuf[0];
		fs3001_i2c_write(fs3001, FS3001_BRKCFG5,fs3001->dts_info.fs3001_brk_times);
	}
	return count;
}

//show  the current battery voltage
//cat vbat_monitor
//vbat_monitor = 3433
static ssize_t fs3001_vbat_monitor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	mutex_lock(&fs3001->lock);
	fs3001_haptic_get_vbat(fs3001);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat_monitor = %d\n",fs3001->vbat);
	mutex_unlock(&fs3001->lock);

	return len;
}

//show lra_resistance
//cat lra_resistance
//lra_resistance = 226
static ssize_t fs3001_lra_resistance_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");
	
	if(fs3001->dts_info.fs3001_lr_pgagain != fs3001->dts_info.fs3001_brk_pgagain)
	{
		fs3001_haptic_set_pgagain(fs3001, fs3001->dts_info.fs3001_lr_pgagain);
		fs3001_haptic_offset_calibration(fs3001);
	}
	
	fs3001_haptic_get_lra_resistance(fs3001);
	len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %d\n",fs3001->lra);

	if(fs3001->dts_info.fs3001_lr_pgagain != fs3001->dts_info.fs3001_brk_pgagain)
	{
		fs3001_haptic_set_pgagain(fs3001, fs3001->dts_info.fs3001_brk_pgagain);
		fs3001_haptic_offset_calibration(fs3001);
	}
	return len;
}

//show fs3001->gun_type(This parameter is not used at all in driver)
//cat gun_type
//0xff
static ssize_t fs3001_gun_type_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	pr_info("enter\n");

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", fs3001->gun_type);

}

//set fs3001->gun_type
//cat gun_type
//0xff
//echo 254 > gun_type [hex is ok: echo 0xfe > gun_type]
//cat gun_type
//0xfe
static ssize_t fs3001_gun_type_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	pr_info("value=%d\n", val);
	mutex_lock(&fs3001->lock);
	fs3001->gun_type = val;
	mutex_unlock(&fs3001->lock);
	return count;
}

//cat version
//v0.1.0.2
static ssize_t fs3001_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	pr_info("enter\n");

	return snprintf(buf, PAGE_SIZE, "%s\n", FOURSEMI_DRIVER_VERSION);
}


//show fs3001->bullet_nr(This parameter is not used at all in driver)
//cat bullet_nr
//0x00
static ssize_t fs3001_bullet_nr_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	pr_info("enter\n");

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", fs3001->bullet_nr);
}

//set fs3001->bullet_nr
//cat bullet_nr
//0x00
//echo 1 > bullet_nr	[hex is ok]
//cat bullet_nr
//0x01
static ssize_t fs3001_bullet_nr_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int val = 0;
	int rc = 0;
	pr_info("enter\n");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	pr_info("value=%d\n", val);
	mutex_lock(&fs3001->lock);
	fs3001->bullet_nr = val;
	mutex_unlock(&fs3001->lock);
	return count;
}

//show fs3001->haptic_audio.ctr.cnt
//cat haptic_audio
//0
static ssize_t fs3001_haptic_audio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf+len, PAGE_SIZE-len,"%d\n", fs3001->haptic_audio.ctr.cnt);
	return len;
}

//it's for ram play
//CMD==0��null
//CMD==1��play
//CMD==255��stop

//echo 1 1 1 1 0 
static ssize_t fs3001_haptic_audio_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[6] = {0};
	int rtp_is_going_on = 0;
	struct haptic_ctr *hap_ctr = NULL;
	
	pr_info("enter\n");

	rtp_is_going_on = fs3001_haptic_juge_RTP_is_going_on(fs3001);
	if (rtp_is_going_on) 
	{
		pr_info("RTP is runing, stop audio haptic\n");
		return count;
	}
	if (!fs3001->ram_init)
		return count;

	if (sscanf(buf, "%d %d %d %d %d %d",&databuf[0], &databuf[1], &databuf[2],&databuf[3], &databuf[4], &databuf[5]) == 6) 
	{
		if (databuf[2] != FS3001_HAPTIC_CMD_NULL) 
		{
			pr_info("cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",databuf[0], databuf[1], databuf[2],databuf[3], databuf[4], databuf[5]);
			hap_ctr = (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr),GFP_KERNEL);
			if (hap_ctr == NULL) 
			{
				pr_err("%s:kzalloc memory fail\n", FSERROR);
				return count;
			}
			mutex_lock(&fs3001->haptic_audio.lock);
			hap_ctr->cnt = (unsigned char)databuf[0];
			hap_ctr->cmd = (unsigned char)databuf[1];
			hap_ctr->play = (unsigned char)databuf[2];
			hap_ctr->wavseq = (unsigned char)databuf[3];
			hap_ctr->loop = (unsigned char)databuf[4];
			hap_ctr->gain = (unsigned char)databuf[5];
			fs3001_haptic_audio_ctr_list_insert(&fs3001->haptic_audio,hap_ctr, fs3001->dev);
			if (hap_ctr->cmd == 0xff) 
			{
				pr_info("haptic_audio stop\n");
				if (hrtimer_active(&fs3001->haptic_audio.timer)) 
				{
					pr_info("cancel haptic_audio_timer\n");
					hrtimer_cancel(&fs3001->haptic_audio.timer);
					fs3001->haptic_audio.ctr.cnt = 0;
					fs3001_haptic_audio_off(fs3001);
				}
			}
			else 
			{
				if (hrtimer_active(&fs3001->haptic_audio.timer)) 
				{
				
				} 
				else 
				{
					pr_info("start haptic_audio_timer\n");
					fs3001_haptic_audio_init(fs3001);
					hrtimer_start(&fs3001->haptic_audio.timer,ktime_set(fs3001->haptic_audio.delay_val/1000000,(fs3001->haptic_audio.delay_val%1000000)*1000),HRTIMER_MODE_REL);
				}
			}
		}
		mutex_unlock(&fs3001->haptic_audio.lock);
		kfree(hap_ctr);
	}
	return count;
}

//show fs3001->dts_info.fs3001_bypass_system_gain
//cat bypass_system_gain
//bypass_system_gain = 0
static ssize_t fs3001_bypass_system_gain_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	pr_info("enter\n");

	return snprintf(buf, PAGE_SIZE, "bypass_system_gain = %d\n", fs3001->dts_info.fs3001_bypass_system_gain);
}

//set fs3001->dts_info.fs3001_bypass_system_gain
//cat bypass_system_gain
//bypass_system_gain = 0
//echo 1 > bypass_system_gain
//cat bypass_system_gain
//bypass_system_gain = 1
static ssize_t fs3001_bypass_system_gain_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	int databuf[1] = { 0 };
	pr_info("enter\n");
	if (sscanf(buf, "%x", &databuf[0]) == 1) 
	{
		pr_info("bypass_system_gain = %d\n",databuf[0]);
		if (databuf[0] != 0 && databuf[0] != 1)
		{
			pr_info("1:input value_0=%d out of range\n",databuf[0]);
			return count;
		}
		
		fs3001->dts_info.fs3001_bypass_system_gain = databuf[0];
		pr_info("2:bypass_system_gain = %d\n",fs3001->dts_info.fs3001_bypass_system_gain);
	}
	return count;
}

//reset the chip
//echo 1 > reset
static ssize_t fs3001_reset_store(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	pr_info("enter");

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val)
	{
		foursemi_haptic_softreset(g_foursemi);
	}
	return count;
}

static ssize_t fs3001_debug_enable_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"debug_enable = 0x%02X\n",fs3001->fs3001_debug_enable);
	return len;
}

static ssize_t fs3001_debug_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[1] = { 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x", &databuf[0]) == 1) 
	{
		pr_info("debug_enable = %d\n",databuf[0]);
		if (databuf[0] != 0 && databuf[0] != 1)
		{
			pr_info("1:input value_0=%d out of range\n",databuf[0]);
			return count;
		}
		
		fs3001->fs3001_debug_enable = databuf[0];
		pr_info("2:debug_enable = %d\n",fs3001->fs3001_debug_enable);
	}

	return count;
}

static ssize_t fs3001_gain_adjust_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"gain_adjust = 0x%02X\n",fs3001->dts_info.fs3001_gain_adjust);
	return len;
}

static ssize_t fs3001_gain_adjust_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[1] = { 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x", &databuf[0]) == 1) 
	{
		pr_info("debug_enable = %d\n",databuf[0]);
		if (databuf[0] < 0 || databuf[0] >= 128)
		{
			pr_info("1:input value_0=%d out of range\n",databuf[0]);
			return count;
		}
		
		fs3001->dts_info.fs3001_gain_adjust = databuf[0];
		pr_info("2:gain_adjust = %d\n",fs3001->dts_info.fs3001_gain_adjust);
	}

	return count;
}

static ssize_t fs3001_auto_brake_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	ssize_t len = 0;
	pr_info("enter\n");

	len += snprintf(buf + len, PAGE_SIZE - len,"auto_brake = 0x%02X\n",fs3001->dts_info.fs3001_auto_brake);
	return len;
}

static ssize_t fs3001_auto_brake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	unsigned int databuf[1] = { 0 };
	pr_info("enter\n");

	if (sscanf(buf, "%x", &databuf[0]) == 1) 
	{
		pr_info("debug_enable = %d\n",databuf[0]);
		if (databuf[0] != 0 && databuf[0] != 1)
		{
			pr_info("1:input value=%d out of range\n",databuf[0]);
			return count;
		}
		
		fs3001->dts_info.fs3001_auto_brake = databuf[0];
		pr_info("2:fs3001_auto_brake = %d\n",fs3001->dts_info.fs3001_auto_brake);
	}

	return count;
}



static DEVICE_ATTR(f0, 0644, fs3001_f0_show, NULL);
static DEVICE_ATTR(f0_ref, 0644, fs3001_f0_ref_show, fs3001_f0_ref_store);
static DEVICE_ATTR(cont, 0644, fs3001_cont_show, fs3001_cont_store);
static DEVICE_ATTR(register, 0644, fs3001_reg_show, fs3001_reg_store);
static DEVICE_ATTR(registers, 0644, NULL, fs3001_regs_store);
static DEVICE_ATTR(duration, 0644, fs3001_duration_show,fs3001_duration_store);
static DEVICE_ATTR(index, 0644, fs3001_index_show, fs3001_index_store);
static DEVICE_ATTR(activate, 0644, fs3001_activate_show,fs3001_activate_store);
static DEVICE_ATTR(activate_mode, 0644, fs3001_activate_mode_show,fs3001_activate_mode_store);
static DEVICE_ATTR(state, 0644, fs3001_state_show, fs3001_state_store);
static DEVICE_ATTR(seq, 0644, fs3001_seq_show, fs3001_seq_store);
static DEVICE_ATTR(loop, 0644, fs3001_loop_show, fs3001_loop_store);
static DEVICE_ATTR(rtp, 0644, fs3001_rtp_show, fs3001_rtp_store);
static DEVICE_ATTR(sram_size, 0644, fs3001_sram_size_show,NULL);
static DEVICE_ATTR(osc_cali, 0644, NULL,fs3001_osc_cali_store);
static DEVICE_ATTR(gain, 0644, fs3001_gain_show, fs3001_gain_store);
static DEVICE_ATTR(ram_update, 0644, fs3001_ram_update_show,fs3001_ram_update_store);
static DEVICE_ATTR(f0_save, 0644, fs3001_f0_save_show, fs3001_f0_save_store);
static DEVICE_ATTR(osc_save, 0644, fs3001_osc_save_show,fs3001_osc_save_store);
static DEVICE_ATTR(vbat_mode, 0644, fs3001_vbat_mode_show,fs3001_vbat_mode_store);
static DEVICE_ATTR(cali, 0644, fs3001_cali_show, fs3001_cali_store);
static DEVICE_ATTR(cali_mode, 0644, fs3001_cali_mode_show, fs3001_cali_mode_store);
static DEVICE_ATTR(cont_drv_lvl, 0644, fs3001_cont_drv_lvl_show,fs3001_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, 0644, fs3001_cont_drv_time_show,fs3001_cont_drv_time_store);
static DEVICE_ATTR(brk_times, 0644, fs3001_brk_times_show,fs3001_brk_times_store);
static DEVICE_ATTR(vbat_monitor, 0644, fs3001_vbat_monitor_show,NULL);
static DEVICE_ATTR(lra_resistance, 0644, fs3001_lra_resistance_show,NULL);
static DEVICE_ATTR(gun_type, 0644, fs3001_gun_type_show,fs3001_gun_type_store);
static DEVICE_ATTR(bullet_nr, 0644, fs3001_bullet_nr_show,fs3001_bullet_nr_store);
static DEVICE_ATTR(haptic_audio, 0644, fs3001_haptic_audio_show,fs3001_haptic_audio_store);
static DEVICE_ATTR(bypass_system_gain, 0644, fs3001_bypass_system_gain_show,fs3001_bypass_system_gain_store);
static DEVICE_ATTR(version, 0644, fs3001_version_show, NULL);
static DEVICE_ATTR(reg_inits, 0644, fs3001_reg_inits_show,NULL);
static DEVICE_ATTR(reset, 0644, NULL,fs3001_reset_store);
static DEVICE_ATTR(debug_enable, 0644, fs3001_debug_enable_show, fs3001_debug_enable_store);
static DEVICE_ATTR(gain_adjust, 0644, fs3001_gain_adjust_show, fs3001_gain_adjust_store);
static DEVICE_ATTR(auto_brake, 0644, fs3001_auto_brake_show, fs3001_auto_brake_store);



static struct attribute *fs3001_vibrator_attributes[] = 
{
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_register.attr,
	&dev_attr_registers.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_f0_ref.attr,
	&dev_attr_cali.attr,
	&dev_attr_cali_mode.attr,
	&dev_attr_f0_save.attr,
	&dev_attr_osc_save.attr,
	&dev_attr_cont.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_brk_times.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_sram_size.attr,
	&dev_attr_vbat_mode.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_gun_type.attr,
	&dev_attr_bullet_nr.attr,
	&dev_attr_haptic_audio.attr,
	&dev_attr_bypass_system_gain.attr,
	&dev_attr_version.attr,
	&dev_attr_reg_inits.attr,
	&dev_attr_reset.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_gain_adjust.attr,
    &dev_attr_auto_brake.attr,
	NULL
};

struct attribute_group fs3001_vibrator_attribute_group = 
{
	.attrs = fs3001_vibrator_attributes
};


static int fs3001_haptic_play_effect_seq(struct fs3001 *fs3001, unsigned char flag)
{
	pr_info("enter flag = %d,fs3001->effect_id =%d,fs3001->dts_info.fs3001_rtp_id_boundary = %d,fs3001->activate_mode =%d,fs3001->dts_info.fs3001_bypass_system_gain=%d\n",flag,fs3001->effect_id,fs3001->dts_info.fs3001_rtp_id_boundary,fs3001->activate_mode,fs3001->dts_info.fs3001_bypass_system_gain);
	
	if (fs3001->effect_id > fs3001->dts_info.fs3001_rtp_id_boundary)
		return 0;
	
	if (flag) 
	{
		if (fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RAM_MODE) 
		{
			//20210719
		    if((fs3001->effect_id == 1)||(fs3001->effect_id == 2)|| (fs3001->effect_id == 9))
			{
				fs3001->effect_id = 3;
			}
			else if(fs3001->effect_id == 4)
			{
				fs3001->effect_id = 8;
			}else if(fs3001->effect_id == 5)
			{
				fs3001->effect_id = 6;
			}

			//20210719
			fs3001_haptic_set_wav_seq(fs3001, 0x00,(char)fs3001->effect_id + 1);//set seq0
			fs3001_haptic_set_pwm(fs3001, fs3001->dts_info.fs3001_play_ram_srate);//20210601
			fs3001_haptic_set_wav_seq(fs3001, 0x01, 0x00);//set seq1 stop
			fs3001_haptic_set_wav_loop(fs3001, 0x00, 0x00);
			fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RAM_MODE);
			if(fs3001->dts_info.fs3001_bypass_system_gain == 0)
			{
				fs3001_haptic_effect_strength(fs3001);
				fs3001_haptic_set_gain(fs3001, fs3001->level);
			}
			fs3001_haptic_start(fs3001);

		}
		if (fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE) 
		{
			//has been set in fs3001_duration_store->fs3001_haptic_duration_ram_play_config, so remove it if it's mediatek platform
			//fs3001_haptic_set_repeat_wav_seq(fs3001,(fs3001->dts_info.fs3001_rtp_id_boundary +1));
			fs3001_haptic_set_pwm(fs3001,  fs3001->dts_info.fs3001_play_ram_srate);//20210601
			if(fs3001->dts_info.fs3001_bypass_system_gain == 0)
			{
				fs3001_haptic_set_gain(fs3001, fs3001->level);
			}
			fs3001_haptic_play_repeat_seq(fs3001, true);
		}
	}
	pr_info("exit\n");
	return 0;
}

static void fs3001_long_vibrate_work_routine(struct work_struct *work)
{
	struct fs3001 *fs3001 = container_of(work, struct fs3001,long_vibrate_work);

	pr_info("enter,state=%d activate_mode = %d duration = %d\n",fs3001->state, fs3001->activate_mode, fs3001->duration);

	pr_info("bbbbbbbbbbbbbbbefore mutex_lock(&fs3001->lock)\n");
	mutex_lock(&fs3001->lock);
	pr_info("aaaaaaaaaaaaaaaafter  mutex_lock(&fs3001->lock)\n");
	//Enter standby mode
	fs3001_haptic_stop(fs3001);
	if (fs3001->state) 
	{
		fs3001_haptic_upload_lra(fs3001, FS3001_F0_CALI);
		if (fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RAM_MODE) 
		{
			fs3001_haptic_ram_vbat_compensate(fs3001, false);//20210716
			//fs3001_haptic_play_repeat_seq(fs3001, true);
			fs3001_haptic_play_effect_seq(fs3001, true);
		}
		else if (fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE) 
		{
			fs3001_haptic_ram_vbat_compensate(fs3001, true);
			fs3001_haptic_play_effect_seq(fs3001, true);
			//fs3001_haptic_play_repeat_seq(fs3001, true);
			// run ms timer
			hrtimer_start(&fs3001->timer, ktime_set(fs3001->duration / 1000,(fs3001->duration % 1000) * 1000000), HRTIMER_MODE_REL);
			pm_stay_awake(fs3001->dev);
			fs3001->wk_lock_flag = 1;
		}
		else if (fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_CONT_MODE) 
		{
			pr_info("mode:%s\n","FS3001_HAPTIC_ACTIVATE_CONT_MODE");
			fs3001_haptic_cont_config(fs3001);
			// run ms timer
			hrtimer_start(&fs3001->timer,ktime_set(fs3001->duration / 1000,(fs3001->duration % 1000) * 1000000),HRTIMER_MODE_REL);
		}
		else 
		{
			pr_err("%s:activate_mode error\n", FSERROR);
		}
	}
	else 
	{
		if (fs3001->wk_lock_flag == 1) 
		{
			pm_relax(fs3001->dev);
			fs3001->wk_lock_flag = 0;
		}
	}
	pr_info("bbbbbbbbbbbbbbbefore mutex_unlock(&fs3001->lock)\n");
	mutex_unlock(&fs3001->lock);
	pr_info("aaaaaaaaaaaaaaaafter  mutex_unlock(&fs3001->lock)\n");
}

#ifdef FS_HAPSTREAM

static int fs3001_hapstream_i2c_writes(struct fs3001 *fs3001, struct mmap_buf_format *hapstream_buf)
{
	int ret = -1, i = 0, len = 0;
	char str[200] = { 0 };

	pr_info("enter,hapstream_buf->reg_addr=0x%02X, hapstream_buf->length=%d\n",hapstream_buf->reg_addr,hapstream_buf->length);

	ret = i2c_master_send(fs3001->i2c, &(hapstream_buf->reg_addr), hapstream_buf->length + 1);

	//xxxx for debug
	if(fs3001->fs3001_debug_enable)
	{
		for (i = 0; i < hapstream_buf->length; i++)
		{
			if ((i + 1) % 16 == 0)
			{
				snprintf(str + len, 200 - len, "0x%02X \n", (uint8_t)hapstream_buf->data[i]);
				pr_info("%s", str);
				len = 0;
			}
			else
			{
				len += snprintf(str + len, 200 - len, "0x%02X ", (uint8_t)hapstream_buf->data[i]);
				if(i == hapstream_buf->length -1)
				{
					pr_info("%s", str);
				}
			}
		}
	}
	
	if (ret < 0)
		pr_err("%s, i2c master send error\n", FSERROR);
	return ret;
}

static inline unsigned int fs3001_get_sys_msecs(void)//static inline unsigned int fs3001_get_sys_msecs() -- transsion platform
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	struct timespec64 ts64;
	ktime_get_coarse_real_ts64(&ts64);
#else
	struct timespec64 ts64 = current_kernel_time64();
#endif

	pr_info("enter\n");

	return jiffies_to_msecs(timespec64_to_jiffies(&ts64));
}

static void fs3001_hapstream_clean_buf(struct fs3001 *fs3001, int status)
{
	struct mmap_buf_format *hapstream_buf = fs3001->start_buf;
	int i = 0;

	pr_info("enter\n");

	for (i = 0; i < HAPSTREAM_MMAP_BUF_SUM; i++)
	{
		hapstream_buf->status = status;
		hapstream_buf = hapstream_buf->kernel_next;
	}
}

static void fs3001_rtp_work_hapstream(struct work_struct *work)
{
	//struct fs3001 *fs3001 = container_of(work,struct fs3001,rtp_hapstream);
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	struct mmap_buf_format *hapstream_buf = fs3001->start_buf;
	int count = 100;
	unsigned char reg_val_AF = 0x01;
	unsigned char reg_val_11 = 0;
	unsigned char reg_val_00 = 0;
	unsigned int write_start;
	unsigned int buf_cnt = 0;

	pr_info("enter\n");

	mutex_lock(&fs3001->lock);
	fs3001->hapstream_stop_flag = false;
	while (true && count--)
	{
		if (hapstream_buf->status == MMAP_BUF_DATA_VALID) 
		{
			fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_RTP_MODE);
			fs3001_haptic_play_go(fs3001);
			break;
		} 
		else if (fs3001->hapstream_stop_flag == true) 
		{
			mutex_unlock(&fs3001->lock);
			pr_info("fs3001->hapstream_stop_flag == true, return\n");
			return;
		} 
		else 
		{
			mdelay(1);//mdelay(10);//zzzz for debug, otherwise too much debug message     
		}
	}
	if (count <= 0) 
	{
		pr_err( "%s, error, start_buf->status is not valid\n", FSERROR);
		fs3001->hapstream_stop_flag = true;
		mutex_unlock(&fs3001->lock);
		return;
	}
	mutex_unlock(&fs3001->lock);

	mutex_lock(&fs3001->rtp_lock);
	pm_qos_enable(true);
	fs3001_haptic_raminit(fs3001, true);
	write_start = fs3001_get_sys_msecs();
	reg_val_AF = 0x01;
	while (true)
	{
		if (fs3001_get_sys_msecs() > (write_start + 800)) 
		{
			pr_err( "%s, Failed ! endless loop\n", FSERROR);
			break;
		}
		fs3001_i2c_read(fs3001, FS3001_STATUS, &reg_val_00);
		fs3001_i2c_read(fs3001, FS3001_SYSCTRL, &reg_val_11);
		if (((reg_val_00 & 0x01) != 0x01) && ((reg_val_11 & 0x03) != 0x01))
		{
			pr_info("hapstream break_1, reg_00 = 0x%02X, reg_11 = 0x%02X\n",reg_val_00, reg_val_11);
			break;
		}
		if (reg_val_00 & 0x00 || (fs3001->hapstream_stop_flag == true) || (hapstream_buf->status == MMAP_BUF_DATA_FINISHED) || (hapstream_buf->status == MMAP_BUF_DATA_INVALID)) 
		{
			pr_info("hapstream break_2, reg_00 = 0x%02X, hapstream_stop_flag = %d, hapstream_buf->status = 0x%02X\n",reg_val_00, fs3001->hapstream_stop_flag, hapstream_buf->status);
			break;
		}
		else if (hapstream_buf->status == MMAP_BUF_DATA_VALID && (reg_val_AF & 0x01)) 
		{
			pr_info("buf_cnt = %d, bit = %d, length = %d!\n", buf_cnt, hapstream_buf->bit, hapstream_buf->length);
			fs3001_hapstream_i2c_writes(fs3001, hapstream_buf);
			memset(hapstream_buf->data, 0, hapstream_buf->length);
			hapstream_buf->length = 0;
			hapstream_buf->status = MMAP_BUF_DATA_FINISHED;
			hapstream_buf = hapstream_buf->kernel_next;
			write_start = fs3001_get_sys_msecs();
			buf_cnt++;
		}
		else 
		{
			//pr_info("hapstream wait, reg_AF = 0x%02X, hapstream_buf->status = 0x%02X\n", reg_val_AF, hapstream_buf->status);
			mdelay(1);//mdelay(10);//zzzz for debug, otherwise too much debug message     
		}
		fs3001_i2c_read(fs3001, FS3001_INTSTATR2, &reg_val_AF);
	}
	fs3001_haptic_raminit(fs3001, false);

	pm_qos_enable(false);
	fs3001->hapstream_stop_flag = true;
	mutex_unlock(&fs3001->rtp_lock);
}

static void fs3001_rtp_irq_work_hapstream(struct work_struct*work)
{
	pr_info("enter\n");
}


static long fs3001_hapstream_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	pr_info("enter,cmd=%d,arg=%ld\n",cmd,arg);

	return 0;
}


#define WRITE_RTP_MODE 	1
#define WRITE_STOP_MODE	2

static ssize_t fs3001_buf_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
	struct fs3001 *fs3001 = g_foursemi->fs3001;

	pr_info("enter: count=%d, hapstream_stop_flag=%d, vib_stop_flag=%d\n", (int)count, fs3001->hapstream_stop_flag, fs3001->vib_stop_flag);

	switch (count) 
	{
		case WRITE_RTP_MODE:
			fs3001_hapstream_clean_buf(fs3001, MMAP_BUF_DATA_INVALID);
			fs3001->hapstream_stop_flag = true;
			if (fs3001->vib_stop_flag == false) 
			{
				mutex_lock(&fs3001->lock);
				fs3001_haptic_stop(fs3001);
				mutex_unlock(&fs3001->lock);
			}
			schedule_work(&fs3001->rtp_hapstream);
			break;
		case WRITE_STOP_MODE:
			fs3001->hapstream_stop_flag = true;
			mutex_lock(&fs3001->lock);
			fs3001_haptic_stop(fs3001);
			mutex_unlock(&fs3001->lock);
			break;
		default:
			break;
	}

	return count;
}

static int fs3001_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys;
	struct fs3001 *fs3001 = g_foursemi->fs3001;
	int ret = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,7,0)
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ | PROT_WRITE, 0) | calc_vm_flag_bits(MAP_SHARED);

	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;

	if (vma && (pgprot_val(vma->vm_page_prot) != pgprot_val(vm_get_page_prot(vm_flags)))) {
		pr_info("vm_page_prot error!\n");
		return -EPERM;
	}

	if (vma && ((vma->vm_end - vma->vm_start) != (PAGE_SIZE << HAPSTREAM_MMAP_PAGE_ORDER))) {
		pr_info("mmap size check err!\n");
		return -EPERM;
	}
#endif

	pr_info("enter\n");

	phys = virt_to_phys(fs3001->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT), (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if (ret) {
		pr_info("mmap failed!\n");
		return ret;
	}

	pr_info("success!\n");

	return ret;
}

#ifdef FS_KERNEL_VER_OVER_5_10 
static const struct proc_ops config_proc_ops = {
	.proc_write = fs3001_buf_write_proc,
	.proc_ioctl = fs3001_hapstream_unlocked_ioctl,
	.proc_mmap = fs3001_file_mmap,
};
#else
static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.write = fs3001_buf_write_proc,
	.unlocked_ioctl = fs3001_hapstream_unlocked_ioctl,
	.mmap = fs3001_file_mmap,
};
#endif

#endif


//irq
//1:register as led driver
//2:init fs3001->timer(hrtimer_init)
//3:init work,wait_queue_head_t
int fs3001_vibrator_init(struct fs3001 *fs3001)
{
       int ret = 0;
 
       pr_info("enter\n");
 
#ifdef TIMED_OUTPUT
       pr_info("TIMED_OUT FRAMEWORK!\n");
       fs3001->vib_dev.name = "foursemi_vibrator";
       fs3001->vib_dev.get_time = fs3001_vibrator_get_time;
       fs3001->vib_dev.enable = fs3001_vibrator_enable;
 
 
       ret = timed_output_dev_register(&(fs3001->vib_dev));
       if (ret < 0)
       {
              pr_err("%s:fail to create timed output dev\n", FSERROR);
              return ret;
       }
       ret = sysfs_create_group(&fs3001->vib_dev.dev->kobj, &fs3001_vibrator_attribute_group);
       if (ret < 0)
       {
              pr_err("%s:error creating bus sysfs attr files\n", FSERROR);
              return ret;
       }
#else
       pr_info("loaded in leds_cdev framework!\n");
       fs3001->vib_dev.name = "vibrator";
       fs3001->vib_dev.brightness_get = fs3001_haptic_brightness_get;
       fs3001->vib_dev.brightness_set = fs3001_haptic_brightness_set;
 
 
       ret = devm_led_classdev_register(&fs3001->i2c->dev, &fs3001->vib_dev);
       if (ret < 0)
       {
              pr_err("%s:fail to create led dev\n", FSERROR);
              return ret;
       }
      
       ret = sysfs_create_group(&fs3001->vib_dev.dev->kobj, &fs3001_vibrator_attribute_group);
       if (ret < 0)
       {
              pr_err("%s:error creating bus sysfs attr files\n", FSERROR);
              return ret;
       }
 
      
       //ret = sysfs_create_link(&fs3001->vib_dev.dev->kobj,&fs3001->i2c->dev.kobj, "vibrator");
       //if (ret < 0)
       //{
       //       pr_err("%s:error creating class sysfs link attr files\n", FSERROR);
       //       return ret;
       //}
#endif

#ifdef FS_HAPSTREAM
	   fs3001->fs_config_proc = NULL;
	   fs3001->fs_config_proc = proc_create(FS_HAPSTREAM_NAME, 0666, NULL, &config_proc_ops);
	   if (fs3001->fs_config_proc == NULL)
		   dev_err(fs3001->dev, "create_proc_entry %s failed\n", FS_HAPSTREAM_NAME);
	   else
		   dev_info(fs3001->dev, "create proc entry %s success\n", FS_HAPSTREAM_NAME);
   
	   fs3001->start_buf = (struct mmap_buf_format *)__get_free_pages(GFP_KERNEL, HAPSTREAM_MMAP_PAGE_ORDER);
	   if (fs3001->start_buf == NULL)
	   {
		   pr_info( "Error _get_free_pages failed\n");
		   return -ENOMEM;
	   }
   
	   SetPageReserved(virt_to_page(fs3001->start_buf));
	   {
		   struct mmap_buf_format *temp;
		   uint32_t i = 0;
		   temp = fs3001->start_buf;
		   for (i = 1; i < HAPSTREAM_MMAP_BUF_SUM; i++)
		   {
			   temp->kernel_next = (fs3001->start_buf + i);
			   temp = temp->kernel_next;
		   }
		   temp->kernel_next = fs3001->start_buf;
   
		   temp = fs3001->start_buf;
		   for (i = 0; i < HAPSTREAM_MMAP_BUF_SUM; i++)
		   {
			   temp->bit = i;
			   temp->reg_addr = FS3001_RTPWDATA;
			   temp = temp->kernel_next;
		   }
	   }
	   fs3001->hapstream_stop_flag = true;
	   fs3001->vib_stop_flag = false;
	   
	   INIT_WORK(&fs3001->rtp_irq_hapstream, fs3001_rtp_irq_work_hapstream);
	   INIT_WORK(&fs3001->rtp_hapstream, fs3001_rtp_work_hapstream);
#endif

       hrtimer_init(&fs3001->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
       fs3001->timer.function = fs3001_vibrator_timer_func;
       INIT_WORK(&fs3001->long_vibrate_work,fs3001_long_vibrate_work_routine);
       INIT_WORK(&fs3001->rtp_work, fs3001_rtp_work_routine);
       mutex_init(&fs3001->lock);
       mutex_init(&fs3001->rtp_lock);
       atomic_set(&fs3001->is_in_rtp_loop, 0);
       atomic_set(&fs3001->exit_in_rtp_loop, 0);
       init_waitqueue_head(&fs3001->wait_q);
       init_waitqueue_head(&fs3001->stop_wait_q);
       return 0;
}


static void fs3001_haptic_misc_para_init(struct fs3001 *fs3001)
{
	pr_info("enter\n");

	//	set Waveform select to be square
	fs3001_i2c_write(fs3001, FS3001_BWAVCTRL, FS3001_BWAVCTRL_B1_0_SQUARE);
}

static void fs3001_hack_init(struct fs3001 *fs3001)
{
	pr_info("enter\n");
	fs3001_i2c_write(fs3001, FS3001_HDCTRL, 0xB1);//for A1 short 
	fs3001_haptic_enable_key(fs3001,true);//enable key
	fs3001_i2c_write_bits_1(fs3001, FS3001_ANACTRL,1,6,6);//Just in case of special circumstances, so enable it
	fs3001_i2c_write(fs3001, FS3001_OTPPG0W2B4, 0x00);//for A1 short
	fs3001_i2c_write_bits_1(fs3001, FS3001_ANACTRL,0,6,6);
	fs3001_haptic_enable_key(fs3001,false);//disable key
}


static void fs3001_haptic_duration_ram_play_config(struct fs3001 *fs3001,int duration)
{
	unsigned char seq = 0;
	unsigned char loop = 0;
	
	pr_info( "enter %d\n",duration);

	if(duration == 200)
	{
		duration = 100;//wrong
	}
	
	if(duration == 12)
	{
		duration = 0;//ring
	}

	if(duration == 65 || duration == 95)
	{
		seq = 1;//short-strong
		loop = 0;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_MODE;
	}
	else if((duration < fs3001->dts_info.fs3001_duration_time[0]) && duration != 12)
	{
		seq = 3;//0-30 short-weak
		loop = 0;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_MODE;
	}
	else if((duration >= fs3001->dts_info.fs3001_duration_time[0]) && (duration < fs3001->dts_info.fs3001_duration_time[1]))
	{
		seq = 1;//30-60 short-strong
		loop = 0;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_MODE;
	}
	else if((duration >= fs3001->dts_info.fs3001_duration_time[1]) && (duration < fs3001->dts_info.fs3001_duration_time[2]) && duration != 65)
	{
		seq = 5;//60-90 long-weak
		loop = 0x0E;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE;
	}
	else if((duration >= fs3001->dts_info.fs3001_duration_time[2]) && duration != 95)
	{
		seq = 4;//over 90 long-strong
		loop = 0x0E;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE;
	}
	else
	{
		seq = 0;
		loop = 0;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_MODE;
	}

	fs3001_haptic_set_wav_seq(fs3001,0,seq);
	fs3001_haptic_set_wav_loop(fs3001,0,loop);
	fs3001_haptic_set_wav_seq(fs3001,1,0);
	fs3001_haptic_set_wav_loop(fs3001,1,0);
}


static void fs3001_haptic_diagnostic_sequence(struct fs3001 *fs3001)
{
	pr_info( "enter\n");
	fs3001_haptic_enable_key(fs3001,true);//enable key
	fs3001_i2c_write_bits_1(fs3001,FS3001_INTCTRL,0,7,7);// Disable interrupt output
	fs3001_i2c_write(fs3001, FS3001_ANACTRL, 0x40);//0xC0 = 0x40		// Enable OSC
	fs3001_i2c_write(fs3001, FS3001_SWDIAG1, 0xC0);//0xB5 = 0xC0		// Enable Software diagnose mode and VCM buffer
	usleep_range(1000, 2000);//usleep_range(10, 20); //Wait > 5 us
	fs3001_i2c_write(fs3001, FS3001_SWDIAG1, 0xD2);//0xB5 = 0xD2		// Turn on S1FN & S1FP & PGA
	usleep_range(1000, 2000);//usleep_range(100, 150);//Wait > 100 us														 //Wait > 100 us  �����100ms
}

static void fs3001_haptic_set_diagnostic_reg_to_default_value(struct fs3001 *fs3001)
{
	pr_info( "enter\n");
	//reset to default value //0xB5, 0xBB, 0x2A(15), 0xC0, 0xA9(80)
	fs3001_i2c_write(fs3001, FS3001_SWDIAG1, 0x00);//0xB5 = 0x00
	fs3001_i2c_write(fs3001, FS3001_SARCTRL, 0x00);//0xBB = 0x00
	fs3001_i2c_write(fs3001, FS3001_ANACTRL, 0x00);//0xC0 = 0x00
	fs3001_i2c_write_bits_1(fs3001,FS3001_INTCTRL,1,7,7);// enable interrupt output
	fs3001_haptic_enable_key(fs3001,false);//disable key													 //Wait > 100 us  �����100ms
}


//offset calibration
static int fs3001_haptic_offset_calibration(struct fs3001 *fs3001)
{
	unsigned char uc_0xBC = 0;
	unsigned char uc_0xBD = 0;
 	unsigned int ui_BCBD = 0;
	int i = 0;

	pr_info( "enter\n");
	fs3001_haptic_diagnostic_sequence(fs3001);

	//outp
	fs3001_i2c_write(fs3001, FS3001_SARCTRL, 0x92);//0xBB = 0x92		// SAR capture data
	usleep_range(1000, 2000);//usleep_range(10, 20); //Wait > 5 us
	fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
	fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
	uc_0xBD = fs3001_i2c_read_bits(fs3001, FS3001_SARTS_H, 1,0);//0xBD//high 2 offset value
	ui_BCBD = uc_0xBD<<8 | uc_0xBC;
	if (100 <= ui_BCBD && ui_BCBD <= 350)            //Check if SAR_OUTN_READING ��[100, 350] (TBD, 0.58V ~2.05V). 
	{
		pr_info( "outp = %d\n",ui_BCBD);
	}
	else
	{
		pr_err( "%s,outp is out of range =  %d\n",FSERROR,ui_BCBD);
		fs3001_haptic_set_diagnostic_reg_to_default_value(fs3001);
		return ui_BCBD;
	}

	//outn
	fs3001_i2c_write(fs3001, FS3001_SARCTRL, 0x93);//0xBB = 0x93		// SAR capture data
	usleep_range(1000, 2000);//usleep_range(10, 20); //Wait > 5 us
	fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
	fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
	uc_0xBD = fs3001_i2c_read_bits(fs3001, FS3001_SARTS_H, 1,0);//0xBD//high 2 offset value
	ui_BCBD = uc_0xBD<<8 | uc_0xBC;
	if (100 <= ui_BCBD && ui_BCBD <= 350)            //Check if SAR_OUTN_READING ��[100, 350] (TBD, 0.58V ~2.05V). 
	{
		pr_info("outn = %d\n",ui_BCBD);
	}
	else
	{
		pr_err("%s,outn is out of range =  %d\n",FSERROR,ui_BCBD);
		fs3001_haptic_set_diagnostic_reg_to_default_value(fs3001);
		return ui_BCBD;
	}

	//offset
	fs3001_i2c_write(fs3001, FS3001_SWDIAG1, 0xDE);//0xB5 = 0xDE		// Turn on S2F
	usleep_range(1000, 2000);//usleep_range(10, 20); //Wait > 5 us
	ui_BCBD = 0;
	for(i=0;i<FS3001_OFFSET_RETRIES;i++)
	{
		fs3001_i2c_write(fs3001, FS3001_SARCTRL, 0x90);//0xBB = 0x90		// SAR capture data
		usleep_range(1000, 2000);//usleep_range(10, 20); //Wait > 5 us
		fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
		fs3001_i2c_read(fs3001, FS3001_SARTS_L, &uc_0xBC);//0xBC//low 8 offset value
		uc_0xBD = fs3001_i2c_read_bits(fs3001, FS3001_SARTS_H, 1,0);//0xBD//high 2 offset value
		ui_BCBD = (uc_0xBD<<8 | uc_0xBC) + ui_BCBD;
	}
	ui_BCBD = ui_BCBD/FS3001_OFFSET_RETRIES;
	if(ui_BCBD>(512+6) || ui_BCBD<(512-6))
	{
		pr_err("%s:offset = %d\n",FSERROR,ui_BCBD);
	}
	else
	{
		fs3001->offset = ui_BCBD;
		pr_info("offset = %d\n",fs3001->offset);
		//Write back SAR_OS_READING to {0xB3<1:0>, 0xB2<7:0>}
		fs3001_i2c_write(fs3001, FS3001_DIAGLRA_L, (ui_BCBD & 0xff));
		fs3001_i2c_write_bits_1(fs3001,FS3001_DIAGLRA_H,(ui_BCBD >> 8) & 0x3,1,0);
	}

	fs3001_haptic_set_diagnostic_reg_to_default_value(fs3001);

	return 0;
}



static void fs3001_ram_work_routine(struct work_struct *work)
{
	struct fs3001 *fs3001 = container_of(work, struct fs3001,ram_work.work);

	pr_info("enter\n");
	fs3001_ram_update(fs3001);
}

int fs3001_ram_work_init(struct fs3001 *fs3001)
{
	int ram_timer_val = 8000;

	pr_info("enter\n");
	INIT_DELAYED_WORK(&fs3001->ram_work, fs3001_ram_work_routine);
	schedule_delayed_work(&fs3001->ram_work,msecs_to_jiffies(ram_timer_val));
	return 0;
}

static enum hrtimer_restart fs3001_haptic_audio_timer_func(struct hrtimer *timer)
{
	struct fs3001 *fs3001 = container_of(timer,struct fs3001, haptic_audio.timer);

	pr_info("enter\n");
	schedule_work(&fs3001->haptic_audio.work);

	hrtimer_start(&fs3001->haptic_audio.timer,ktime_set(fs3001->haptic_audio.timer_val/1000000,(fs3001->haptic_audio.timer_val%1000000)*1000),HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void fs3001_haptic_auto_brk_enable(struct fs3001 *fs3001, unsigned char flag)
{
	pr_info("enter, flag = %d\n", flag);

	fs3001_i2c_write_bits_1(fs3001, FS3001_BRKCTRL,flag,7,7);
}

void fs3001_reg_init(struct fs3001 *fs3001)
{
	int i = 0;
	unsigned char uc_reg_addr = 0, uc_reg_val =0, uc_start = 0, uc_end = 0;
	//0x927000 fs3001_i2c_write_bits_1(fs3001, 0x92,00,7,0);
	//0x934003 fs3001_i2c_write_bits_1(fs3001, 0x93,3,4,0);

	pr_info("enter\n");

	mutex_lock(&fs3001->lock);
	for (i = 0;i < ARRAY_SIZE(fs3001->dts_info.fs3001_reg_inits);i++)
	{
		if(fs3001->dts_info.fs3001_reg_inits[i] != 0xffffff)
		{
			uc_reg_addr = (fs3001->dts_info.fs3001_reg_inits[i] >> 16) & 0xff;
			uc_reg_val = fs3001->dts_info.fs3001_reg_inits[i]  & 0xff;
			uc_end = (fs3001->dts_info.fs3001_reg_inits[i] >> 8) & 0xf;
			uc_start = ((fs3001->dts_info.fs3001_reg_inits[i] >> 12) & 0xf) + uc_end;
			
			fs3001_i2c_write_bits_1(fs3001, uc_reg_addr,uc_reg_val,uc_start,uc_end);
		}
	}
	mutex_unlock(&fs3001->lock);
}


void fs3001_f0_cali_setting_init(struct fs3001 *fs3001)
{
	pr_info("enter\n");

	mutex_lock(&fs3001->lock);

	fs3001_haptic_set_f0_ref(fs3001);
	//	DRV1_lvl
	fs3001_i2c_write(fs3001, FS3001_DRVCFG1,fs3001->dts_info.fs3001_cont_drv1_lvl);
	//	DRV2_lvl
	fs3001_i2c_write(fs3001, FS3001_DRVCFG2,fs3001->dts_info.fs3001_cont_drv2_lvl);
	//  DRV1_TIME
	fs3001_i2c_write(fs3001, FS3001_DRVCFG3,fs3001->dts_info.fs3001_cont_drv1_time);
	//  DRV2_TIME
	fs3001_i2c_write(fs3001, FS3001_DRVCFG4,fs3001->dts_info.fs3001_cont_drv2_time);
	//  cont_1_period
	fs3001_i2c_write(fs3001, FS3001_TRKCFG2, fs3001->dts_info.fs3001_cont_1_period & 0xff);
	fs3001_i2c_write_bits_1(fs3001, FS3001_TRKCFG3, fs3001->dts_info.fs3001_cont_1_period>>8,4,0);
	//	brk_slopeth
	fs3001_i2c_write(fs3001, FS3001_BRKCFG1, fs3001->dts_info.fs3001_brk_slopeth & 0xff);
	//	brk_gain
	fs3001_i2c_write(fs3001, FS3001_BRKCFG4, fs3001->dts_info.fs3001_brk_gain & 0xff);
	//	brk_times
	fs3001_i2c_write_bits_1(fs3001, FS3001_BRKCFG5,fs3001->dts_info.fs3001_brk_times,4,0);
	//	brk_noise_gate
	fs3001_i2c_write(fs3001, FS3001_BEMFDCFG4, fs3001->dts_info.fs3001_brk_noise_gate & 0xff);
	//	brk_1_period
	fs3001_i2c_write(fs3001, FS3001_TRKCFG6, fs3001->dts_info.fs3001_brk_1_period & 0xff);
	fs3001_i2c_write_bits_1(fs3001, FS3001_TRKCFG7, fs3001->dts_info.fs3001_brk_1_period>>8,4,0);
	//	brk_pgagain
	fs3001_haptic_set_pgagain(fs3001, fs3001->dts_info.fs3001_brk_pgagain);
	//	brk_margin
	fs3001_i2c_write(fs3001, FS3001_TRKCFG4, fs3001->dts_info.fs3001_brk_margin & 0xff);

	
	//fs3001_i2c_write_bits_1(fs3001, FS3001_FDETCTRL, fs3001->dts_info.fs3001_average_number,3,0);
	//fs3001_i2c_write(fs3001, FS3001_FDETCFG1, fs3001->dts_info.fs3001_try_number & 0xff);
	//fs3001_i2c_write(fs3001, FS3001_TRKCFG5, fs3001->dts_info.fs3001_apt_step & 0xff);
	//fs3001_i2c_write(fs3001, FS3001_BEMFDCFG14, fs3001->dts_info.fs3001_zc_timeout_f0 & 0xff);
	//fs3001_i2c_write(fs3001, FS3001_BEMFDCFG9, fs3001->dts_info.fs3001_zc_timeout & 0xff);
	//fs3001_i2c_write(fs3001, FS3001_BEMFDCFG10, fs3001->dts_info.fs3001_tra_margin & 0xff);
	mutex_unlock(&fs3001->lock);
}


int fs3001_haptic_init(struct fs3001 *fs3001)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	//char*ptr;zzzz

	pr_info("enter\n");
	//haptic audio
	fs3001->haptic_audio.delay_val = 1;
	fs3001->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(fs3001->haptic_audio.ctr_list));
	hrtimer_init(&fs3001->haptic_audio.timer,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fs3001->haptic_audio.timer.function = fs3001_haptic_audio_timer_func;
	INIT_WORK(&fs3001->haptic_audio.work,fs3001_haptic_audio_work_routine);
	mutex_init(&fs3001->haptic_audio.lock);
	
	fs3001->gun_type = 0xff;
	fs3001->bullet_nr = 0x00;

	mutex_lock(&fs3001->lock);
	//haptic init
	fs3001->fs3001_debug_enable = 0;
	fs3001->activate_mode = fs3001->dts_info.fs3001_default_vib_mode;
	ret = fs3001_i2c_read(fs3001, FS3001_GAINCFG, &reg_val);
	fs3001->gain = reg_val;
	pr_info("fs3001->gain = 0x%02X\n",fs3001->gain);
	for (i = 0; i < FS3001_SEQUENCER_SIZE; i++) 
	{
		reg_val = fs3001_i2c_read_bits(fs3001, FS3001_WFSCFG1 + i, 6, 0);
		fs3001->seq[i] = reg_val;
	}

	//FS3001_HAPTIC_STANDBY_MODE:just call fs3001_haptic_stop(fs3001);
	fs3001_haptic_play_mode(fs3001, FS3001_HAPTIC_STANDBY_MODE);
	
	fs3001_haptic_set_pwm(fs3001, fs3001->dts_info.fs3001_play_ram_srate);
	
	// misc value init: Write registers according to the Settings in DTS
	//1: cont setting
	//2: brake setting
	fs3001_haptic_misc_para_init(fs3001);

	fs3001_hack_init(fs3001);

	//set motor protect
	fs3001_haptic_swicth_motor_protect_config(fs3001, true);
	//Based on the IRQ-GPIO Settings in DTS, decide whether to turn TRIG on or not
	fs3001_haptic_trig_config(fs3001);

	//offset_calibration
	fs3001_haptic_offset_calibration(fs3001);
	//config auto_brake
	fs3001_haptic_auto_brk_enable(fs3001,fs3001->dts_info.fs3001_auto_brake);

	//f0 calibration
	if (fs3001->dts_info.fs3001_f0_cali_data_mode == FS3001_F0_CALI_DATA_SELF_MODE)
	{
		fs3001_haptic_f0_calibration(fs3001);
	}
/*zzzz
	else if(fs3001->dts_info.fs3001_f0_cali_data_mode == FS3001_F0_CALI_DATA_CMDLINE_MODE)
	{
		ptr = strstr(saved_command_line, "fs3001_lk_f0_cali=");
		if (ptr != NULL) 
		{
			ptr += strlen("fs3001_lk_f0_cali=");
			fs3001->f0_cali_data = simple_strtol(ptr, NULL, 0);
			pr_info("fs3001->f0_cali_data = 0x%x\n", fs3001->f0_cali_data);
		}
		else
		{
			pr_err("%s: FS3001_F0_CALI_DATA_CMDLINE_MODE fail\n",FSERROR);
			fs3001_haptic_f0_calibration(fs3001);
		}
	}
	*/
	else if(fs3001->dts_info.fs3001_f0_cali_data_mode == FS3001_F0_CALI_DATA_DTS_MODE)
	{
		if (fs3001->dts_info.fs3001_lk_f0_cali != 0) 
		{
			fs3001->f0_cali_data = fs3001->dts_info.fs3001_lk_f0_cali;
			pr_info("fs3001->f0_cali_data = 0x%x\n", fs3001->f0_cali_data);
		}
		else
		{
			pr_err("%s: FS3001_F0_CALI_DATA_DTS_MODE fail\n",FSERROR);
			fs3001_haptic_f0_calibration(fs3001);
		}
	}
	else
	{
		pr_err("%s: fs3001->dts_info.fs3001_f0_cali_data_mode=%d\n",FSERROR, fs3001->dts_info.fs3001_f0_cali_data_mode);
		fs3001_haptic_f0_calibration(fs3001);
	}

	

	mutex_unlock(&fs3001->lock);
	return ret;
}

void fs3001_interrupt_setup(struct fs3001 *fs3001)
{
	unsigned char reg_val = 0;

	pr_info("enter\n");

	//INT clear 0xAC
	fs3001_i2c_read(fs3001, FS3001_INTSTAT1, &reg_val);
	pr_info("reg INTSTAT1=0x%02X\n", reg_val);
	//INT clear 0xAD
	fs3001_i2c_read(fs3001, FS3001_INTSTAT2, &reg_val);
	pr_info("reg INTSTAT2=0x%02X\n", reg_val);


	//INTM7	7	RW	1h	Interrupt source request mask - OC
	//INTM6	6	RW	1h	Interrupt source request mask - UV
	//INTM5	5	RW	1h	Interrupt source request mask - OV
	//INTM4	4	RW	1h	Interrupt source request mask - OTP
	//INTM3	3	RW	1h	Interrupt source request mask - DC
	//INTM2	2	RW	1h	Interrupt source request mask - VCM fault
	//INTM1	1	RW	1h	Interrupt source request mask - reserved
	//INTM0	0	RW	1h	Interrupt source request mask - a fault removal
	
	//Since ae and empty are always 1, if there is a non-sticky interrupt source, 
	//then the interruption will occur all the time, and the default is that it is 
	//sticky, so we need to read the above first to clear out ae and empty. The following 
	//sentence opens all interrupt sources and marks them first, which is not useful, 
	//because even if the interrupt is triggered, there is no special 
	//processing (if the chip is not working properly, just read the status register).
	//xxxx 20220913: close all INT source
	//fs3001_i2c_write_bits_1(fs3001, FS3001_INTMASK1,0b00000010,7,0);//enable OC,UV,OV,OTP,DC,VCM, reserved,fault removal
}

irqreturn_t fs3001_irq(int irq, void *data)
{
	struct fs3001 *fs3001 = data;
	unsigned char reg_val = 0;
	unsigned int buf_len = 0;
	unsigned char DIGSTAT_OPS = 0;

	pr_info("enter\n");

	if (fs3001->hapstream_stop_flag == false) 
	{
		return IRQ_HANDLED;
	}
	
	atomic_set(&fs3001->is_in_rtp_loop, 1);
	fs3001_i2c_read(fs3001, FS3001_INTSTAT1, &reg_val);
	if(reg_val != 0)
	{
		pr_err("%s:INTSTAT1=0x%02X\n", FSERROR,reg_val);
	}
	
	fs3001_i2c_read(fs3001, FS3001_INTSTAT2, &reg_val);
	pr_info("reg INTSTAT2=0x%02X\n", reg_val);
		
	if (reg_val & FS3001_INTSTAT2_MASK_B7_DONE)
	{
		pr_info("chip playback done\n");
	}			

	if (reg_val & FS3001_INTSTAT2_MASK_B1_AE) 
	{
		pr_info("fs3001 rtp fifo almost empty\n");
		if (fs3001->rtp_init) 
		{
			//almost full?
			while ((!fs3001_haptic_rtp_get_fifo_afs(fs3001)) && (fs3001->play_mode == FS3001_HAPTIC_RTP_MODE) && !atomic_read(&fs3001->exit_in_rtp_loop)) 
			{
				mutex_lock(&fs3001->rtp_lock);
				pr_info( "fs3001 rtp mode fifo update, cnt=%d\n", fs3001->rtp_cnt);
				if (!fs3001->rtp_container) 
				{
					pr_err("%s:fs3001->rtp_container is null, break!\n", FSERROR);
					mutex_unlock(&fs3001->rtp_lock);
					break;
				}
				//total file len - transfered file len < 1/2 file size
				if ((fs3001->rtp_container->len - fs3001->rtp_cnt) < (fs3001->ram.base_addr >> 2)) 
				{
					buf_len = fs3001->rtp_container->len - fs3001->rtp_cnt;
				} 
				else 
				{
					buf_len = (fs3001->ram.base_addr >> 2);
				}
				fs3001->rtp_update_flag = fs3001_i2c_writes(fs3001,FS3001_RTPWDATA,&fs3001->rtp_container->data[fs3001->rtp_cnt],buf_len);
				fs3001->rtp_cnt += buf_len;
				fs3001_i2c_read(fs3001, FS3001_DIGSTAT, &DIGSTAT_OPS);
				DIGSTAT_OPS = DIGSTAT_OPS >> 4;//get 0x4 high 4 bits(7-4)
				if ((fs3001->rtp_cnt == fs3001->rtp_container->len) || ((DIGSTAT_OPS & 0x0f) == 0)) 
				{
					if (fs3001->rtp_cnt == fs3001->rtp_container->len)
					{
						pr_info("rtp load completely! DIGSTAT_OPS=%02x fs3001->rtp_cnt=%d\n",DIGSTAT_OPS,fs3001->rtp_cnt);
					}					
					else
					{
						pr_err("%s:rtp load failed!! DIGSTAT_OPS=%02x fs3001->rtp_cnt=%d\n", FSERROR, DIGSTAT_OPS,fs3001->rtp_cnt);
					}
					fs3001_haptic_raminit(fs3001,false);
					fs3001_haptic_set_rtp_aei(fs3001,false);
					fs3001->rtp_cnt = 0;
					fs3001->rtp_init = 0;
					mutex_unlock(&fs3001->rtp_lock);
					break;
				}
				mutex_unlock(&fs3001->rtp_lock);
			}
		}
		else 
		{
			pr_err("%s:fs3001 rtp init = %d, init error\n", FSERROR,fs3001->rtp_init);
		}
	}

	if (reg_val & FS3001_INTSTAT2_MASK_B1_AF)
	{
		pr_info("fs3001 rtp mode fifo almost full!\n");
	}			
		

	if (fs3001->play_mode != FS3001_HAPTIC_RTP_MODE || atomic_read(&fs3001->exit_in_rtp_loop))
	{
		fs3001_haptic_set_rtp_aei(fs3001, false);
	}

	atomic_set(&fs3001->is_in_rtp_loop, 0);
	wake_up_interruptible(&fs3001->wait_q);
	pr_info("exit\n");

	return IRQ_HANDLED;
}

//fs3001_check_qualify
char fs3001_check_qualify(struct fs3001 *fs3001)
{
	unsigned char reg = 0;
	int ret = 0;
	
	pr_info("enter\n");

	ret = fs3001_i2c_read(fs3001, FS3001_CHIPINI, &reg);
	if (ret < 0) 
	{
		pr_err("%s:failed to read register 0x0E: %d\n", FSERROR,ret);
		return ret;
	}
	if (util_get_bit(reg,1)==1)
	{
		return 1;//otped
	}
	pr_err("%s:register 0x0E error: 0x%02x\n", FSERROR,reg);
	return 0;
}

//fs3001_haptics_upload_effect
//1: case: fs3001->effect_type == FF_CONSTANT
//    fs3001->duration = effect->replay.length;
//    fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE;
//    fs3001->effect_id = fs3001->dts_info.fs3001_rtp_id_boundary;
//    In this case, the vibration can be played for a fixed duration. The duration is effect->replay.length. The mode is recorded as Ram loop, vibration mode, and fs3001_rtp_id_boundary (10 in DTS). How do you control the loop time? The essence is to use a timer, the time is up, the RAM play to stop.

//2��fs3001->effect_type == FF_PERIODIC
//    Custom_data stores the user-specified play information in this effect-> u.putiodic.custom_data. Use copy_from_user to store this information in the variable data[]
//    fs3001->effect_id = data[0]    //get effect_id
//     A��If effect_id is between 0 and 9 (i.e., <boundary), then activate_mode is recorded as FS3001_HAPTIC_ACTIVATE_RAM_MODE
//     B��If effect_id>=10, then activate_mode is recorded as FS3001_HAPTIC_ACTIVATE_RTP_MODE
//     C��Both A and B need to save the duration for effect_id (obtained from DTS) to data[1], data[2]
//     Finally, use copy_to_user to return the data[] information to effect->u.periodic.custom_data
int fs3001_haptics_upload_effect (struct input_dev *dev,struct ff_effect *effect,struct ff_effect *old)
{
	struct fs3001 *fs3001 = input_get_drvdata(dev);
	s16 data[CUSTOM_DATA_LEN];
	ktime_t rem;
	s64 time_us;
	int ret;

	pr_info("enter\n");

	if (fs3001->osc_cali_run != 0)
		return 0;

	if (hrtimer_active(&fs3001->timer)) 
	{
		rem = hrtimer_get_remaining(&fs3001->timer);
		time_us = ktime_to_us(rem);
		pr_info("waiting for playing clear sequence: %lld us\n",time_us);
		usleep_range(time_us, time_us + 100);
	}

	pr_info("effect->type=0x%x,FF_CONSTANT=0x%x,FF_PERIODIC=0x%x\n",effect->type, FF_CONSTANT, FF_PERIODIC);
	fs3001->effect_type = effect->type;
	pr_info("bbbbbbbbbbbbbbbefore mutex_lock(&fs3001->lock)\n");
	mutex_lock(&fs3001->lock);
	pr_info("aaaaaaaaaaaaaaaaafter mutex_lock(&fs3001->lock)\n");
	while (atomic_read(&fs3001->exit_in_rtp_loop)) 
	{
		pr_info("goint to waiting rtp  exit\n");
		mutex_unlock(&fs3001->lock);
		ret = wait_event_interruptible(fs3001->stop_wait_q,atomic_read(&fs3001->exit_in_rtp_loop) == 0);
		pr_info("wakeup \n");
		if (ret == -ERESTARTSYS) 
		{
			mutex_unlock(&fs3001->lock);
			pr_err("%s:wake up by signal return error\n", FSERROR);
			return ret;
		}
		mutex_lock(&fs3001->lock);
	}

	if (fs3001->effect_type == FF_CONSTANT) 
	{
		pr_info("effect_type is  FF_CONSTANT! \n");
		//set duration
		fs3001->duration = effect->replay.length;
		fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE;
		fs3001->effect_id = fs3001->dts_info.fs3001_rtp_id_boundary;

	} 
	else if (fs3001->effect_type == FF_PERIODIC) 
	{
		if (fs3001->effects_count == 0) 
		{
			mutex_unlock(&fs3001->lock);
			return -EINVAL;
		}

		pr_info("effect_type is  FF_PERIODIC! \n");
		if (copy_from_user(data, effect->u.periodic.custom_data,sizeof(s16) * CUSTOM_DATA_LEN)) 
		{
			mutex_unlock(&fs3001->lock);
			return -EFAULT;
		}

		fs3001->effect_id = data[0];//yyyy_get effect_id from data[0] as effect_type is FF_PERIODIC
		pr_info("fs3001->effect_id =%d \n",fs3001->effect_id);
		fs3001->play.vmax_mv = effect->u.periodic.magnitude;	//vmax level

		if (fs3001->effect_id < 0 || fs3001->effect_id > fs3001->dts_info.fs3001_rtp_max) 
		{
			mutex_unlock(&fs3001->lock);
			return 0;
		}

		if (fs3001->effect_id < fs3001->dts_info.fs3001_rtp_id_boundary) 
		{
			fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RAM_MODE;
			pr_info("fs3001->effect_id=%d , fs3001->activate_mode = %d\n", fs3001->effect_id,fs3001->activate_mode);
			data[1] = fs3001->predefined[fs3001->effect_id].play_rate_us / 1000000;	//second data
			data[2] = fs3001->predefined[fs3001->effect_id].play_rate_us / 1000;	//millisecond data
			pr_info("fs3001->predefined[fs3001->effect_id].play_rate_us/1000 = %d\n",fs3001->predefined[fs3001->effect_id].play_rate_us / 1000);
		}
		
		if (fs3001->effect_id >= fs3001->dts_info.fs3001_rtp_id_boundary) 
		{
			fs3001->activate_mode = FS3001_HAPTIC_ACTIVATE_RTP_MODE;
			pr_info("fs3001->effect_id=%d , fs3001->activate_mode = %d\n", fs3001->effect_id,fs3001->activate_mode);
			data[1] = fs3001->dts_info.fs3001_rtp_time[fs3001->effect_id] / 1000;	//second data
			data[2] = fs3001->dts_info.fs3001_rtp_time[fs3001->effect_id];	//millisecond data
			pr_info("data[1] = %d data[2] = %d\n",data[1], data[2]);
		}

		if (copy_to_user(effect->u.periodic.custom_data, data,sizeof(s16) * CUSTOM_DATA_LEN)) 
		{
			mutex_unlock(&fs3001->lock);
			return -EFAULT;
		}
	} 
	else 
	{
		pr_err("%s:Unsupported effect type: %d\n", FSERROR,effect->type);
	}
	pr_info("bbbbbbbbbbbbbbbefore mutex_unlock(&fs3001->lock)     fs3001->effect_type= 0x%x\n", fs3001->effect_type);
	
	mutex_unlock(&fs3001->lock);
	pr_info("aaaaaaaaaaaaaaaaafter unlock,fs3001->effect_type= 0x%x\n",fs3001->effect_type);
	return 0;
}

int fs3001_haptics_playback(struct input_dev *dev, int effect_id, int val)
{
	struct fs3001 *fs3001 = input_get_drvdata(dev);
	int rc = 0;
	pr_info("enter effect_id=%d,val=%d\n",effect_id,val);

	pr_info("effect_id=%d , val = %d\n", effect_id, val);
	pr_info("fs3001->effect_id=%d , fs3001->activate_mode = %d\n", fs3001->effect_id, fs3001->activate_mode);

	//for osc calibration
	if (fs3001->osc_cali_run != 0)
	{
		return 0;
	}

	if (val > 0)
	{
		fs3001->state = 1;
	}
	if (val <= 0)
	{
		fs3001->state = 0;
	}
	hrtimer_cancel(&fs3001->timer);
	pr_info("hrtimer_cancel(&fs3001->timer)\n");

	if (fs3001->effect_type == FF_CONSTANT && fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RAM_LOOP_MODE) 
	{
		pr_info("enter ram_loop_mode \n");
		queue_work(fs3001->work_queue, &fs3001->long_vibrate_work);
	} 
	else if (fs3001->effect_type == FF_PERIODIC && fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RAM_MODE) 
	{
		pr_info("enter  ram_mode\n");
		queue_work(fs3001->work_queue, &fs3001->long_vibrate_work);
	} 
	else if (fs3001->effect_type == FF_PERIODIC && fs3001->activate_mode == FS3001_HAPTIC_ACTIVATE_RTP_MODE) 
	{
		pr_info("enter  rtp_mode\n");
		queue_work(fs3001->work_queue, &fs3001->rtp_work);
		//if we are in the play mode, force to exit
		if (val == 0) 
		{
			atomic_set(&fs3001->exit_in_rtp_loop, 1);
		}
	} 
	else 
	{
		//other mode
	}

	return rc;
}

int fs3001_haptics_erase(struct input_dev *dev, int effect_id)
{
	struct fs3001 *fs3001 = input_get_drvdata(dev);
	int rc = 0;
	pr_info("enter\n");

	//for osc calibration
	if (fs3001->osc_cali_run != 0)
	{
		return 0;
	}
	
	fs3001->effect_type = 0;
	fs3001->duration = 0;
	return rc;
}

void fs3001_haptics_set_gain_work_routine(struct work_struct *work)
{
	struct fs3001 *fs3001 = container_of(work, struct fs3001, set_gain_work);

	pr_info("enter\n");

	if (fs3001->new_gain >= 0x7FFF)
	{		
		fs3001->level = 0x80;	//128
	}	
	else if (fs3001->new_gain <= 0x3FFF)
	{		
		fs3001->level = 0x1E;	//30
	}		
	else
	{		
		fs3001->level = (fs3001->new_gain - 16383) / 128;
	}		

	if (fs3001->level < 0x1E)
	{
		fs3001->level = 0x1E;	//30
	}

	pr_info("set_gain queue work, new_gain = %x level = %x \n",fs3001->new_gain, fs3001->level);

	fs3001_haptic_set_gain(fs3001,fs3001->level);
}

void fs3001_haptics_set_gain(struct input_dev *dev, u16 gain)
{
	struct fs3001 *fs3001 = input_get_drvdata(dev);
	pr_info("enter\n");
	pr_info("enter set gain=%d\n",gain);
	fs3001->new_gain = gain;
	queue_work(fs3001->work_queue, &fs3001->set_gain_work);
}

