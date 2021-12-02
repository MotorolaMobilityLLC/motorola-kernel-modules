/*****************************************************************************
* Copyright(c) BMT, 2021. All rights reserved.
*       
* BMT [oz8806] Source Code Reference Design
* File:[bmulib.c]
*
* This Source Code Reference Design for BMT [oz8806] access
* ("Reference Design") is solely for the use of PRODUCT INTEGRATION REFERENCE ONLY,
* and contains confidential and privileged information of BMT International
* Limited. BMT shall have no liability to any PARTY FOR THE RELIABILITY,
* SERVICEABILITY FOR THE RESULT OF PRODUCT INTEGRATION, or results from: (i) any
* modification or attempted modification of the Reference Design by any party, or
* (ii) the combination, operation or use of the Reference Design with non-BMT
* Reference Design. Use of the Reference Design is at user's discretion to qualify
* the final work result.
*****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/suspend.h>
#include <asm/div64.h>
//you must add these here for BMT
#include "parameter.h"
#include "table.h"
#include "battery_config.h"
#include "oz8806_regdef.h"

/*****************************************************************************
* static variables/functions section 
****************************************************************************/
static int fg_hw_init_done = 0;
static uint8_t 	bmu_init_done = 0;
static int oz8806_suspend = 0;
static DEFINE_MUTEX(update_mutex);
static bmu_data_t 	*batt_info_ptr = NULL;
static gas_gauge_t *gas_gauge_ptr = NULL;
static uint8_t charger_finish = 0;
static unsigned long ic_wakeup_time = 0;
static int init_soc = 0;
static void  (*bmu_polling_loop_callback)(void) = NULL;
static void  (*bmu_wake_up_chip_callback)(void) = NULL;
static void  (*bmu_power_down_chip_callback)(void) = NULL;
static void  (*bmu_charge_end_process_callback)(void) = NULL;
static void  (*bmu_discharge_end_process_callback)(void) = NULL;
static int32_t (*oz8806_temp_read_callback)(int32_t *voltage) = NULL;
static int32_t (*oz8806_current_read_callback)(int32_t *voltage) = NULL;
static int32_t (*oz8806_voltage_read_callback)(int32_t *voltage) = NULL;

static int oz8806_update_batt_info(struct oz8806_data *data);
static int oz8806_update_batt_temp(struct oz8806_data *data);
static int32_t oz8806_write_byte(struct oz8806_data *data, uint8_t index, uint8_t dat);
static int32_t oz8806_read_byte(struct oz8806_data *data, uint8_t index, uint8_t *dat);

static struct oz8806_data *the_oz8806 = NULL;
static int8_t adapter_status = O2_CHARGER_BATTERY;
static int32_t rsoc_pre = 0;
#ifdef EXT_THERMAL_READ
static uint8_t ext_thermal_read = 1;
#else
static uint8_t ext_thermal_read = 0;
#endif
static int capacity_init_ok = 0;

static int save_capacity = INIT_CAP;

//-1 means not ready
int32_t age_soh = -1;
int32_t cycle_soh = -1;
#define NAME	"oz8806"
#define BATT_PHY "bms"

static enum power_supply_property oz8806_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	//POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

/*-------------------------------------------------------------------------*/
/*****************************************************************************
* Description:
*		below function is linux power section
* Parameters:
*		description for each argument, new argument starts at new line
* Return:
*		what does this function returned?
*****************************************************************************/
/*-------------------------------------------------------------------------*/

static int oz8806_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct oz8806_data *data = (struct oz8806_data *)power_supply_get_drvdata(psy);

	switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:

		if (adapter_status == O2_CHARGER_BATTERY)
		{
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING; /*discharging*/
		}
		else if(adapter_status == O2_CHARGER_USB ||
		        adapter_status == O2_CHARGER_AC )
		{
			if (data->batt_info.batt_soc == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;	/*charging*/

		}
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = data->batt_info.batt_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bmu_init_done;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = data->batt_info.batt_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = data->batt_info.batt_soc;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = data->batt_info.batt_temp * 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = data->batt_info.batt_fcc_data;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = data->batt_info.batt_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = data->batt_info.batt_fcc_data;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = data->batt_info.batt_capacity;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void oz8806_external_power_changed(struct power_supply *psy)
{
	struct oz8806_data *data = (struct oz8806_data *)power_supply_get_drvdata(psy);

	power_supply_changed(data->bat);
	cancel_delayed_work(&data->work);
	schedule_delayed_work(&data->work, 0);
}

static void oz8806_powersupply_init(struct oz8806_data *data)
{
	data->bat_desc.name = BATT_PHY;
	data->bat_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	data->bat_desc.properties = oz8806_battery_props;
	data->bat_desc.num_properties = ARRAY_SIZE(oz8806_battery_props);
	data->bat_desc.get_property = oz8806_battery_get_property;
	data->bat_desc.external_power_changed = oz8806_external_power_changed;
	data->bat_desc.no_thermal = 1;
}

/*****************************************************************************
 *write 0x20 into register 0x09
 *example:echo 0920 > /sys/class/i2c-dev/i2c-2/device/2-002f/registers
 *****************************************************************************/
static ssize_t oz8806_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t _count)
{
	char data[num_3];
	uint8_t address;
	uint8_t value = 0;
	char *endp; 
	int len;
	struct oz8806_data *oz8806;

	len = strlen(buf);
	if(len < 5)
		return -EINVAL;

	if (&(the_oz8806->myclient->dev) == dev)
	    oz8806 = dev_get_drvdata(dev);
	else //this device is the private device of power supply 
	    oz8806 = dev_get_drvdata(dev->parent);

	data[num_0] = buf[num_0];
 	data[num_1] = buf[num_1];
 	data[num_2] = num_0;

	address = simple_strtoul(data, &endp, 16); 

	if (address != OZ8806_OP_PEC_CTRL
	 && address != OZ8806_OP_CTRL
	 && address != OZ8806_OP_CAR
	 && address != OZ8806_OP_CAR+1
	 && address != OZ8806_OP_BOARD_OFFSET
	 && address != OZ8806_OP_BOARD_OFFSET+1)
	{
		bmt_dbg("register[0x%.2x] is read-only\n", address);
		return _count;
	}

	data[num_0] = buf[num_2];
	data[num_1] = buf[num_3];
	data[num_2] = num_0;

	value = simple_strtoul(data, &endp, 16); 

	oz8806_write_byte(oz8806, address, value);

  	bmt_dbg("write 0x%.2x into register[0x%.2x]\n", value, address);

	return _count;
}

/*
	example:cat /sys/class/i2c-dev/i2c-2/device/2-002f/registers
*/
static ssize_t oz8806_register_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct oz8806_data *oz8806;
	u8 i = 0;
	u8 data = 0;
	int result = 0;

	if (&(the_oz8806->myclient->dev) == dev)
	    oz8806 = dev_get_drvdata(dev);
	else //this device is the private device of power supply 
	    oz8806 = dev_get_drvdata(dev->parent);

	//CHIP ID and Reversion
	oz8806_read_byte(oz8806, OZ8806_OP_IDREV, &data);
	result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", OZ8806_OP_IDREV, data);

	//I2C config register
	oz8806_read_byte(oz8806, OZ8806_OP_I2CCONFIG, &data);
	result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", OZ8806_OP_I2CCONFIG, data);

	for (i=0x00; i<=0x1a; i++)
	{
		oz8806_read_byte(oz8806, i, &data);
		result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", i, data);
	}

	return result;
}

// chip id: 0x38
static ssize_t oz8806_chip_id_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct oz8806_data *oz8806;
	u8 data = 0;
	int result = 0;

	if (&(the_oz8806->myclient->dev) == dev)
	    oz8806 = dev_get_drvdata(dev);
	else //this device is the private device of power supply 
	    oz8806 = dev_get_drvdata(dev->parent);

	//CHIP ID and Reversion
	oz8806_read_byte(oz8806, OZ8806_OP_IDREV, &data);
	result = sprintf(buf, "0x%.2x = 0x%.2x\n", OZ8806_OP_IDREV, data);
	return result;
}

static ssize_t oz8806_debug_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"%d\n", config_data.debug);
}
/*****************************************************************************
 * Description:
 *		oz8806_debug_store
 * Parameters:
 *		write example: echo 1 > debug ---open debug
 *****************************************************************************/
static ssize_t oz8806_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t _count)
{
	int val;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if(val == 1)
	{
		config_data.debug = 1;
		bmt_dbg("DEBUG ON \n");
	}
	else if (val == 0)
	{
		config_data.debug = 0;
		bmt_dbg("DEBUG CLOSE \n");
	}
	else
	{
		bmt_dbg("invalid command\n");
		return -EINVAL;
	}

	return _count;
}
static ssize_t oz8806_save_capacity_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	if (capacity_init_ok && gas_gauge_ptr && gas_gauge_ptr->stored_capacity)
		return sprintf(buf,"%d\n", gas_gauge_ptr->stored_capacity);

	return sprintf(buf,"%d\n", save_capacity);
}

static ssize_t oz8806_bmu_init_done_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"%d\n", bmu_init_done);
}
static ssize_t oz8806_bmu_init_done_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t _count)
{
	int val = 0;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	bmu_init_done = 0;
	ic_wakeup_time = jiffies;

	cancel_delayed_work(&the_oz8806->work);
	msleep(1000);

	if(val == 1)
	{
		bmt_dbg("reinit BMT from ocv\n");
		bmu_reinit(1);//bmu_init_done will be setted
	}
	else if (val == 0)
	{
		bmt_dbg("reinit BMT,just wakeup ic\n");
		bmu_reinit(0);
		bmu_init_done = 1;
	}
	else
	{
		bmt_dbg("invalid command\n");
		return -EINVAL;
	}
	schedule_delayed_work(&the_oz8806->work, 0);
	return _count;
}

static ssize_t batt_cycle_soh_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	cycle_soh = val;
	accmulate_data_update();

	batt_dbg("batt_cycle_count: %d ", cycle_soh);
	return count;
}

static ssize_t batt_cycle_soh_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", cycle_soh);
}

static ssize_t batt_age_soh_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	age_soh = val;

	batt_dbg("batt_age: %d ", age_soh);
	return count;
}

static ssize_t batt_age_soh_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", age_soh);
}

static ssize_t oz8806_name_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", NAME);
}

static DEVICE_ATTR(registers, S_IRUGO | (S_IWUSR|S_IWGRP), oz8806_register_show, oz8806_register_store);
static DEVICE_ATTR(chip_id, S_IRUGO, oz8806_chip_id_show, NULL);
static DEVICE_ATTR(bmt_debug, S_IRUGO | (S_IWUSR|S_IWGRP), oz8806_debug_show, oz8806_debug_store);
static DEVICE_ATTR(save_capacity, S_IRUGO, oz8806_save_capacity_show, NULL);
static DEVICE_ATTR(bmu_init_done, S_IRUGO| (S_IWUSR|S_IWGRP), oz8806_bmu_init_done_show, oz8806_bmu_init_done_store);
static DEVICE_ATTR(cycle_count, S_IRUGO| (S_IWUSR|S_IWGRP), batt_cycle_soh_show, batt_cycle_soh_store);
static DEVICE_ATTR(age, S_IRUGO| (S_IWUSR|S_IWGRP), batt_age_soh_show, batt_age_soh_store);
static DEVICE_ATTR(name, S_IRUGO, oz8806_name_show, NULL);
static struct attribute *oz8806_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_bmt_debug.attr,
	&dev_attr_bmu_init_done.attr,
	&dev_attr_save_capacity.attr,
	&dev_attr_cycle_count.attr,
	&dev_attr_age.attr,
	&dev_attr_name.attr,
	NULL,
};

static struct attribute_group oz8806_attribute_group = {
	.attrs = oz8806_attributes,
};

static int oz8806_create_sys(struct device *dev, const struct attribute_group * grp)
{
	int err = 0;
	struct power_supply *ply;

	bmt_dbg("BMT_create_sysfs\n");
	ply = power_supply_get_by_name(BATT_PHY);

	if(NULL == dev){
		bmt_dbg("[BATT]: failed to register battery\n");
		return -EINVAL;
	}

	if(ply)
	{
		err = sysfs_create_group(&ply->dev.kobj, grp);

		if (!err)
		{
			bmt_dbg("creat BMT sysfs group ok\n");
		}
		else
		{
			bmt_dbg("creat BMT sysfs group fail\n");
			err =  -EIO;
		}
		power_supply_put(ply);
	}
	else
	{
		bmt_dbg("don't have /sys/class/power_supply/battery dir\n");
	}
	return err;
}

static int32_t oz8806_read_byte(struct oz8806_data *data, uint8_t index, uint8_t *dat)
{
	int32_t ret;
	uint8_t i;
	struct i2c_client *client = data->myclient;

	for(i = 0; i < 4; i++){
		ret = i2c_smbus_read_byte_data(client, index);

		if(ret >= 0) break;
		else
			dev_err(&client->dev, "%s: err %d, %d times\n", __func__, ret, i);
	}
	if(i >= 4)
	{
		return ret;
	} 
	*dat = (uint8_t)ret;

	return ret;
}

static int32_t oz8806_write_byte(struct oz8806_data *data, uint8_t index, uint8_t dat)
{
	int32_t ret;
	uint8_t i;
	struct i2c_client *client = data->myclient;

	for(i = 0; i < 4; i++){
		ret = i2c_smbus_write_byte_data(client, index, dat);
		if(ret >= 0) break;
		else
			dev_err(&client->dev, "%s: err %d, %d times\n", __func__, ret, i);
	}
	if(i >= 4)
	{
		return ret;
	}

	return ret;
}

static int32_t oz8806_update_bits(struct oz8806_data *data, uint8_t reg, uint8_t mask, uint8_t dat)
{
	int32_t ret;
	uint8_t tmp;

	ret = oz8806_read_byte(data, reg, &tmp);
	if (ret < 0)
		return ret;

	if ((tmp & mask) != dat)
	{
		tmp &= ~mask;
		tmp |= dat & mask;
		return oz8806_write_byte(data, reg, tmp);

	}
	else
		return 0;
}

void oz8806_reset_wkuptime(void)
{
	ic_wakeup_time = 0;
}
int oz8806_wakeup_full_power(void)
{
	uint8_t val = 0;
	int32_t ret = 0;
	struct oz8806_data *data = the_oz8806;

	ret = oz8806_read_byte(data, OZ8806_OP_CTRL, &val);
	bmt_dbg("BMT_OP_CTRL: 0x%02x\n", val);

	if ((val & SLEEP_MODE) != 0<<6)
	{
		ret = oz8806_update_bits(data, OZ8806_OP_CTRL, SLEEP_MODE, 0 << 6);

		ret = oz8806_read_byte(data, OZ8806_OP_CTRL, &val);

		bmt_dbg("BMT_OP_CTRL: 0x%02x after writing\n", val);

		if ((val & SLEEP_MODE) != (0 << 6))
			bmt_dbg("fail to wake up BMT to full power mode\n");
		else {
			if (oz8806_get_boot_up_time() < 2000)
				ic_wakeup_time = jiffies + msecs_to_jiffies(2000 - oz8806_get_boot_up_time());
			else
				ic_wakeup_time = jiffies;
			bmt_dbg("wake up BMT to full power mode\n");
		}
	}
	bmt_dbg("ic_wakeup_time %lu\n",ic_wakeup_time);

	return ret;
}
EXPORT_SYMBOL(oz8806_wakeup_full_power);

static void discharge_end_fun(struct oz8806_data *data)
{
	//End discharge
	//this may jump 2%
	if (!gas_gauge_ptr || !batt_info_ptr)
		return;
  /*  
    if(batt_info_ptr->fCellTemp < -10)
		config_data.discharge_end_voltage = OZ8806_EOD + 350;
	else if (batt_info_ptr->fCellTemp < 0)
		config_data.discharge_end_voltage = OZ8806_EOD + 300;
	else if (batt_info_ptr->fCellTemp < 5)
		config_data.discharge_end_voltage = OZ8806_EOD + 200;
	else if (batt_info_ptr->fCellTemp < 10)
		config_data.discharge_end_voltage = OZ8806_EOD + 100;
    else
        config_data.discharge_end_voltage = OZ8806_EOD;

*/

	if(batt_info_ptr->fVolt < (config_data.discharge_end_voltage - 100))
	{
        	if(batt_info_ptr->fRSOC == 1)
        	{
            		batt_info_ptr->fRSOC = 0;
            		batt_info_ptr->sCaMAH = data->batt_info.batt_fcc_data / num_100 -1;
            		discharge_end_process();
        	}
        	else if(batt_info_ptr->fRSOC > 0){
            		batt_info_ptr->sCaMAH = batt_info_ptr->fRSOC * data->batt_info.batt_fcc_data / num_100 - 1;
            		batt_info_ptr->fRSOC--;
		}
    }
}

static int check_charger_full(void)
{
	int ret = 0;

	//from charge ic
	return ret;
}

static void charge_end_fun(void)
{
#define CHG_END_PERIOD_MS	(MSEC_PER_SEC * 60) //60 s = 1 minutes 
#define FORCE_FULL_MS	(MSEC_PER_SEC * 120) //120s = 2 minutes 
#define CHG_END_PURSUE_STEP    (MSEC_PER_SEC * 30) //30s

	static unsigned int time_accumulation;
	static unsigned long start_jiffies;
	int oz8806_eoc = 0;
	static unsigned long chgr_full_soc_pursue_start;
	static unsigned int chgr_full_soc_pursue_accumulation;

#ifdef ENABLE_10MIN_END_CHARGE_FUN
	static unsigned long start_record_jiffies;
	static uint8_t start_record_flag = 0;
#endif

	if (!gas_gauge_ptr || !batt_info_ptr)
		return;

	if (adapter_status != O2_CHARGER_BATTERY && check_charger_full())
	{
		charger_finish = 1;
		bmt_dbg("charger is full, enter external charger finish\n");
		goto charger_full;
	}

	if(adapter_status == O2_CHARGER_USB)
		oz8806_eoc = 100;
	else if(adapter_status == O2_CHARGER_AC)
		oz8806_eoc = config_data.charge_end_current;

	if((adapter_status == O2_CHARGER_BATTERY) ||
		(batt_info_ptr->fCurr < DISCH_CURRENT_TH) || 
		(batt_info_ptr->fCurr >  oz8806_eoc))
	{
		charger_finish   = 0;
		time_accumulation = 0;
		start_jiffies = 0;
		chgr_full_soc_pursue_start = 0;
#ifdef ENABLE_10MIN_END_CHARGE_FUN
		start_record_jiffies = 0;
		start_record_flag = 0;
#endif
		return;
	}

	if((batt_info_ptr->fVolt >= (config_data.charge_cv_voltage - 50))&&(batt_info_ptr->fCurr >= DISCH_CURRENT_TH) &&
		(batt_info_ptr->fCurr < oz8806_eoc)&& (!gas_gauge_ptr->charge_end))
	{
		if (!start_jiffies)
			start_jiffies = jiffies;

		time_accumulation = jiffies_to_msecs(jiffies - start_jiffies);

		//time accumulation is over 5 minutes
		if (time_accumulation >= CHG_END_PERIOD_MS)
		{
			charger_finish	 = 1;
			bmt_dbg("enter external charger finish\n");
		}
		else
		{
			charger_finish	= 0;
		}
	}
	else
	{
		time_accumulation = 0;
		start_jiffies = 0;
		charger_finish = 0;
	}

charger_full:
	bmt_dbg("%s, time_accumulation:%d, charger_finish:%d\n",
			__func__, time_accumulation, charger_finish);

	bmt_dbg("voltage:%d, cv:%d, fcurr:%d, BMT eoc:%d, gas_gauge_ptr->charge_end:%d\n",
			batt_info_ptr->fVolt, config_data.charge_cv_voltage,
			batt_info_ptr->fCurr, oz8806_eoc, gas_gauge_ptr->charge_end);

#ifdef ENABLE_10MIN_END_CHARGE_FUN
	if((batt_info_ptr->fRSOC == 99) &&(!start_record_flag) &&(batt_info_ptr->fCurr > oz8806_eoc))
	{
		start_record_jiffies = jiffies;
		start_record_flag = 1;
		bmt_dbg("start_record_flag: %d, at %d ms\n",start_record_flag, jiffies_to_msecs(jiffies));
	}
	if((batt_info_ptr->fRSOC != 99) ||(batt_info_ptr->fCurr < oz8806_eoc))
	{
		start_record_flag = 0;
	}

	if((start_record_flag) && (batt_info_ptr->fCurr > oz8806_eoc))
	{
		if(jiffies_to_msecs(jiffies - start_record_jiffies) > FORCE_FULL_MS)
		{
			bmt_dbg("start_record_flag: %d, at %d ms\n",start_record_flag, jiffies_to_msecs(jiffies));
			charger_finish	 = 1;
			start_record_flag = 0;
			bmt_dbg("enter charge timer finish\n");
		}
	}
#endif
	if(charger_finish)
	{
		if(!gas_gauge_ptr->charge_end)
		{
			if(batt_info_ptr->fRSOC < 100)
			{
				static int32_t fRSOC_extern = 0;

				if(batt_info_ptr->fRSOC <= rsoc_pre){
					bmt_dbg("fRSOC_extern:%d, soc:%d \n",fRSOC_extern,batt_info_ptr->fRSOC);

					if (!chgr_full_soc_pursue_start || fRSOC_extern != batt_info_ptr->fRSOC) {
						chgr_full_soc_pursue_start = jiffies;
						fRSOC_extern = batt_info_ptr->fRSOC ;
					}

					chgr_full_soc_pursue_accumulation =
						jiffies_to_msecs(jiffies - chgr_full_soc_pursue_start);

					if (chgr_full_soc_pursue_accumulation >= CHG_END_PURSUE_STEP){

						chgr_full_soc_pursue_start = 0;
						batt_info_ptr->fRSOC++;
						batt_info_ptr->sCaMAH = batt_info_ptr->fRSOC * the_oz8806->batt_info.batt_fcc_data / num_100;
						batt_info_ptr->sCaMAH +=  the_oz8806->batt_info.batt_fcc_data / 200;  // + 0.5%
					}
				}

				if(batt_info_ptr->fRSOC > 100) {
					batt_info_ptr->fRSOC = 100;
					batt_info_ptr->sCaMAH = batt_info_ptr->fRSOC * the_oz8806->batt_info.batt_fcc_data / num_100;
					batt_info_ptr->sCaMAH ++;
				}
				//update fRSOC_extern
				fRSOC_extern = batt_info_ptr->fRSOC ;
				bmt_dbg("enter charger finsh update soc:%d\n",batt_info_ptr->fRSOC);
			}
			else
			{
				bmt_dbg("enter charger charge end\n");
				gas_gauge_ptr->charge_end = 1;
				charge_end_process();

				charger_finish = 0;
				chgr_full_soc_pursue_start = 0;
			}
		}
		else {
			charger_finish = 0;
			chgr_full_soc_pursue_start = 0;
		}
	} else chgr_full_soc_pursue_start = 0;

}
static void oz8806_wakeup_event(struct oz8806_data *data)
{
	static int ws_active = 0;

	if (adapter_status == O2_CHARGER_BATTERY && ws_active) {
		pm_relax(&data->myclient->dev);
		ws_active = 0;
	}

	if (adapter_status != O2_CHARGER_BATTERY && !ws_active) {
		pm_stay_awake(&data->myclient->dev);
		ws_active = 1;
	}
}
//this is very important code customer need change
//customer should change charge discharge status according to your system
static void system_charge_discharge_status(struct oz8806_data *data)
{
	int8_t adapter_status_temp = O2_CHARGER_BATTERY;

	union power_supply_propval val_ac = {0};
	union power_supply_propval val_usb = {0};

	if (!data->ac_psy)
		data->ac_psy = power_supply_get_by_name ("ac"); 

	if (!data->usb_psy)
		data->usb_psy= power_supply_get_by_name ("usb");

//---------------------------------------------
#ifdef QUALCOMM_MACH_SUPPORT
	if (data->usb_psy)
	{
		if(0 == power_supply_get_property(data->usb_psy, POWER_SUPPLY_PROP_REAL_TYPE, &val_usb))
		{
			if(POWER_SUPPLY_TYPE_USB_ACA == val_usb.intval|| POWER_SUPPLY_TYPE_USB_CDP == val_usb.intval|| POWER_SUPPLY_TYPE_USB_DCP == val_usb.intval)
				adapter_status_temp = O2_CHARGER_AC;
			else if(POWER_SUPPLY_TYPE_USB == val_usb.intval) 
				adapter_status_temp = O2_CHARGER_USB;
			else 
				adapter_status_temp = O2_CHARGER_BATTERY;
		}
	}

#else
	if (data->usb_psy)
	{
		if(0 == power_supply_get_property(data->usb_psy, POWER_SUPPLY_PROP_ONLINE, &val_usb))
		{
			if (val_usb.intval)
				adapter_status_temp = O2_CHARGER_USB;
		}
	}

	if (data->ac_psy)
	{
		if(0 == power_supply_get_property(data->ac_psy, POWER_SUPPLY_PROP_ONLINE, &val_ac))
		{
			if (val_ac.intval)
				adapter_status_temp = O2_CHARGER_AC;
		}
	}
	batt_dbg("val_ac.intval %d, val_usb.intval %d,adapter_status_temp %d\n",val_ac.intval,val_usb.intval,adapter_status_temp);
#endif

	adapter_status = adapter_status_temp;
	batt_dbg("adapter_status:%d\n", adapter_status);
}

static void oz8806_lock_soc(struct oz8806_data *data)
{
	unsigned long car = 0;
	unsigned short temp = 0;

	//bmu ok
	if(bmu_init_done)
	{
		if(rsoc_pre < 0)	rsoc_pre = 0;
		if(rsoc_pre > 100)	rsoc_pre = 100;

		//adapter is online,and current > 0, lock to 0 or pre soc
		if( (data->batt_info.batt_current > 0) 	&& (O2_CHARGER_USB == adapter_status || O2_CHARGER_AC == adapter_status)  )
		{
			if(data->batt_info.batt_voltage <= O2_SOC_START_THRESHOLD_VOL)
			{
				bmt_dbg("charge 1,lock soc to pre_soc,pre_soc=%d\n",rsoc_pre);
				if(0 == rsoc_pre)
					car = data->batt_info.batt_fcc_data / 100 -1;
				else
					car = rsoc_pre * data->batt_info.batt_fcc_data / 100;

				temp = (car * config_data.fRsense) / config_data.dbCARLSB;		//transfer to CAR
				temp /= 1000;

				i2c_smbus_write_word_data(data->myclient, OZ8806_OP_CAR,(unsigned short)temp);
			}
		}

		//adapter is online,do not consider current, lock soc to 100 if voltage >= cv - recharge 
		if(O2_CHARGER_USB == adapter_status || O2_CHARGER_AC == adapter_status)
		{
			if(	(100 == rsoc_pre) && (data->batt_info.batt_voltage >= (config_data.charge_cv_voltage - O2_CONFIG_RECHARGE)) )
			{
				bmt_dbg("charge 2,lock soc to pre_soc,pre_soc=%d\n",rsoc_pre);
				car = 101 * data->batt_info.batt_fcc_data / 100 - 2;
				batt_info_ptr->sCaMAH = car;
				batt_info_ptr->fRC = car;
				batt_info_ptr->fRCPrev = car;
				temp = (car * config_data.fRsense) / config_data.dbCARLSB;		//transfer to CAR
				temp /= 1000;

				i2c_smbus_write_word_data(data->myclient, OZ8806_OP_CAR,(unsigned short)temp);
			}
		}
	}
}

static void oz8806_battery_func(struct oz8806_data *data)
{
	unsigned long time_since_last_update_ms = 0;
	static unsigned long cur_jiffies = 0;

	if(0 == cur_jiffies)
		cur_jiffies = jiffies;

	batt_dbg("BMT battery func: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	time_since_last_update_ms = jiffies_to_msecs(jiffies - cur_jiffies);
	cur_jiffies = jiffies;

	//get charging type: battery, AC, or USB
	system_charge_discharge_status(data);

	rsoc_pre = data->batt_info.batt_soc;

	//you must add this code here for BMT
	//Notice: don't nest mutex
#ifdef EXT_THERMAL_READ
	oz8806_update_batt_temp(data);
#endif
	oz8806_lock_soc(data);

	/**************mutex_lock*********************/
	mutex_lock(&update_mutex);
	if (!oz8806_suspend)
		bmu_polling_loop();

	oz8806_update_batt_info(data);

	charge_end_fun();

	if(adapter_status == O2_CHARGER_BATTERY)
		discharge_end_fun(data);

	mutex_unlock(&update_mutex);
	/**************mutex_unlock*********************/


	oz8806_wakeup_event(data);

	batt_dbg("l=%d v=%d t=%d c=%d ch=%d\n",
			data->batt_info.batt_soc, data->batt_info.batt_voltage, 
			data->batt_info.batt_temp, data->batt_info.batt_current, adapter_status);

	power_supply_changed(data->bat);
	batt_dbg("since last batt update = %lu ms\n", time_since_last_update_ms);
}

static void oz8806_battery_work(struct work_struct *work)
{
	struct oz8806_data *data = container_of(work, struct oz8806_data, work.work);

	oz8806_battery_func(data);

	if (!bmu_init_done) 
	{
	//	data->interval = 2000;
        schedule_delayed_work(&data->work, msecs_to_jiffies(200));
	} 
	else 
	{
		data->interval = BATTERY_WORK_INTERVAL * 1000;
	        schedule_delayed_work(&data->work, msecs_to_jiffies(data->interval));

	}
	batt_dbg("interval:%d ms\n", data->interval);
}

int32_t is_battery_exchanged(void)
{
	return 0;
}
EXPORT_SYMBOL(is_battery_exchanged);

static int oz8806_suspend_notifier(struct notifier_block *nb,
				unsigned long event,
				void *dummy)
{
	struct oz8806_data *data = container_of(nb, struct oz8806_data, pm_nb);

	switch (event) {

	case PM_SUSPEND_PREPARE:
		bmt_dbg("BMT PM_SUSPEND_PREPARE \n");
		cancel_delayed_work_sync(&data->work);
		system_charge_discharge_status(data);
		oz8806_suspend = 1;
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		bmt_dbg("BMT PM_POST_SUSPEND \n");
		system_charge_discharge_status(data);
		mutex_lock(&update_mutex);
		// if AC charge can't wake up every 1 min,you must remove the if.
		if(adapter_status == O2_CHARGER_BATTERY)
				bmu_wake_up_chip();

		oz8806_update_batt_info(data);
		mutex_unlock(&update_mutex);

		schedule_delayed_work(&data->work, 0);
		//this code must be here,very carefull.
		if(adapter_status == O2_CHARGER_BATTERY)
		{
			if(data->batt_info.batt_current >= data->batt_info.discharge_current_th)
			{
				if (batt_info_ptr) batt_info_ptr->fCurr = -20;

				data->batt_info.batt_current = -20;
				bmt_dbg("drop current\n");
			}
		}
		oz8806_suspend = 0;
		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static int32_t oz8806_hw_ic_detect(struct oz8806_data *data)
{

	uint8_t chip_id = 0;
	int32_t ret = 0;
	int32_t i = 0;

	for(i=0; i<5; i++)
	{
		ret = i2c_smbus_read_word_data(data->myclient, OZ8806_OP_IDREV);
		if(ret >= 0)
		{
			bmt_dbg("BMT fgu read chip id value 0x%.2x\n",ret);
			break;
		}
		msleep(5);
	}
	if(i >= 5)
	{
		bmt_dbg("BMT fgu hw ic not detect, reg value 0x%.2x\n",ret);
		return -1;
	}
	chip_id = (uint8_t)ret;
	if(OZ8806_CHIP_ID != chip_id)
		return -1;

	return 0;
}

static void oz8806_init_soc(struct oz8806_data *data)
{
	int32_t ret = 0;
	int32_t temp = 0;
	uint32_t i = 0;

	for(i=0; i<5; i++)
	{
		ret = i2c_smbus_read_word_data(data->myclient, OZ8806_OP_CAR);
		if(ret >= 0)
			break;
		msleep(5);
	}
	if(i >= 5)
	{
		bmt_dbg("BMT fgu init soc failed, reg value %x\n",ret);
		init_soc = 50;
		return;
	}

	temp = (int16_t)ret;
	temp = temp * config_data.dbCARLSB;
	temp = temp * 1000 / config_data.fRsense;
	if(temp > 5)
	{
		init_soc = temp * 100 / config_data.design_capacity - 1;
		if(init_soc >= 100)
			init_soc = 100;
		if(init_soc <= 0)
			init_soc = 0;
	}
	else
		init_soc = 0;

	bmt_dbg("BMT fgu init soc succeed, init soc is %d\n",init_soc);
}
static int oz8806_init_batt_info(struct oz8806_data *data)
{
	data->batt_info.batt_soc = 50; 
	data->batt_info.batt_voltage = 3999;
	data->batt_info.batt_current = -300;
	data->batt_info.batt_temp = 27;
	data->batt_info.batt_capacity = 4000;

	data->batt_info.batt_fcc_data = config_data.design_capacity;
	data->batt_info.discharge_current_th = DISCH_CURRENT_TH;
	data->batt_info.charge_end = 0;
	oz8806_init_soc(data);
	return 0;
}
static int oz8806_update_batt_info(struct oz8806_data *data)
{
	int ret = 0;

	//Notice: before call this function, use mutex_lock(&update_mutex)
	//Notice: don't nest mutex
	if (!batt_info_ptr || !gas_gauge_ptr)
	{
		ret = -EINVAL;
		goto end;
	}

	data->batt_info.batt_soc = batt_info_ptr->fRSOC; 
	if (data->batt_info.batt_soc > 100)
		data->batt_info.batt_soc = 100;

	data->batt_info.batt_voltage = batt_info_ptr->fVolt;
	data->batt_info.batt_current = batt_info_ptr->fCurr;
	data->batt_info.batt_capacity = batt_info_ptr->sCaMAH;

	data->batt_info.charge_end = gas_gauge_ptr->charge_end;
	bmu_init_done = gas_gauge_ptr->bmu_init_ok;
#ifndef EXT_THERMAL_READ
	oz8806_update_batt_temp(data);
#endif
end:
	return ret;
}

static int oz8806_update_batt_temp(struct oz8806_data *data)
{
	int ret = 0;

	if (!batt_info_ptr || !gas_gauge_ptr)
	{
		ret = -EINVAL;
		goto end;
	}

	data->batt_info.batt_temp = batt_info_ptr->fCellTemp;
	if (batt_info_ptr->fCellTemp != data->batt_info.batt_temp)
		batt_info_ptr->fCellTemp = data->batt_info.batt_temp;
end:
	return ret;
}

int oz8806_get_remaincap(void)
{
	int ret = -1;

	mutex_lock(&update_mutex);

	if (batt_info_ptr)
		ret = batt_info_ptr->sCaMAH;

	mutex_unlock(&update_mutex);

	return ret;
}
EXPORT_SYMBOL(oz8806_get_remaincap);

int oz8806_get_soc_from_ext(void)
{
	int ret = -1;

	return ret;
}
EXPORT_SYMBOL(oz8806_get_soc_from_ext);

int oz8806_get_soc(void)
{
	int ret = -1;

	mutex_lock(&update_mutex);

	if (!bmu_init_done)
	{
		ret = init_soc;
	}
	else if (batt_info_ptr)
		ret = batt_info_ptr->fRSOC;

	mutex_unlock(&update_mutex);

	return ret;
}
EXPORT_SYMBOL(oz8806_get_soc);

int oz8806_get_battry_current(void)
{
	int ret = -1;

	if (!bmu_init_done) return -1;

	mutex_lock(&update_mutex);

	if (batt_info_ptr)
		ret = afe_read_current(&batt_info_ptr->fCurr);

	if (ret < 0)
	{
		ret = -1;
		bmt_dbg("BMT current adc error\n");
	}

	if (batt_info_ptr && ret >= 0)
		ret = batt_info_ptr->fCurr;

	mutex_unlock(&update_mutex);

	return ret;
}
EXPORT_SYMBOL(oz8806_get_battry_current);

int oz8806_get_battery_voltage(void)
{
	int ret = -1;

	if (!bmu_init_done) return -1;

	mutex_lock(&update_mutex);

	if (batt_info_ptr)
		ret = afe_read_cell_volt(&batt_info_ptr->fVolt);

	if (ret < 0)
	{
		ret = -1;
		bmt_dbg("BMT voltage adc error\n");
	}

	if (batt_info_ptr && ret >= 0)
		ret = batt_info_ptr->fVolt;

	mutex_unlock(&update_mutex);

	return ret;
}
EXPORT_SYMBOL(oz8806_get_battery_voltage);

int oz8806_get_battery_temp(void)
{
	int ret = -1;

	if (!bmu_init_done) return -1;

	mutex_lock(&update_mutex);

	if (batt_info_ptr)
		ret = batt_info_ptr->fCellTemp;

	mutex_unlock(&update_mutex);

	return ret;
}
EXPORT_SYMBOL(oz8806_get_battery_temp);

void oz8806_battery_update_data(void)
{
	bmt_dbg("enter BMT fgu battery update data");
	if (fg_hw_init_done && the_oz8806 && bmu_init_done)
		oz8806_battery_func(the_oz8806);
}
EXPORT_SYMBOL(oz8806_battery_update_data);

int32_t oz8806_vbus_voltage(void)
{
	int32_t vbus_voltage = 0;
	int ret = -1;

	ret =oz8806_temp_read(&vbus_voltage);

	if (ret < 0)
	{
		bmt_dbg("BMT temp adc error\n");
		return ret;
	}

	bmt_dbg("voltage from BMT:%d\n", vbus_voltage);
	vbus_voltage =  vbus_voltage * (RPULL + RDOWN) / RDOWN;

	return vbus_voltage;
}
EXPORT_SYMBOL(oz8806_vbus_voltage);

int32_t oz8806_get_simulated_temp(void)
{
	return 25;

}
EXPORT_SYMBOL(oz8806_get_simulated_temp);

int32_t oz8806_get_init_status(void)
{
	return bmu_init_done;
}
EXPORT_SYMBOL(oz8806_get_init_status);

int32_t gauge_int_done(void)
{
    return fg_hw_init_done; 
}
EXPORT_SYMBOL(gauge_int_done);

struct i2c_client * oz8806_get_client(void)
{
	if (the_oz8806)
		return the_oz8806->myclient;
	else
	{
		bmt_dbg("BMT is NULL, BMT_probe didn't call\n");
		return NULL;
	}
}
EXPORT_SYMBOL(oz8806_get_client);

int8_t get_adapter_status(void)
{
	return adapter_status;
}
EXPORT_SYMBOL(get_adapter_status);

void oz8806_register_bmu_callback(void *bmu_polling_loop_func,
		void *bmu_wake_up_chip_func,
		void *bmu_power_down_chip_func,
		void *charge_end_process_func,
		void *discharge_end_process_func,
		void *oz8806_temp_read_func,
		void *oz8806_current_read_func,
		void *oz8806_voltage_read_func)
{
	mutex_lock(&update_mutex);

	bmu_polling_loop_callback = bmu_polling_loop_func;
	bmu_wake_up_chip_callback = bmu_wake_up_chip_func;
	bmu_power_down_chip_callback = bmu_power_down_chip_func;
	bmu_charge_end_process_callback = charge_end_process_func;
	bmu_discharge_end_process_callback = discharge_end_process_func;

	oz8806_temp_read_callback = oz8806_temp_read_func;
	oz8806_current_read_callback = oz8806_current_read_func;
	oz8806_voltage_read_callback = oz8806_voltage_read_func;

	bmu_polling_loop();

	mutex_unlock(&update_mutex);
}
EXPORT_SYMBOL(oz8806_register_bmu_callback);

void unregister_bmu_callback(void)
{
	mutex_lock(&update_mutex);

	bmu_polling_loop_callback = NULL;
	bmu_wake_up_chip_callback = NULL;
	bmu_power_down_chip_callback = NULL;
	bmu_charge_end_process_callback = NULL;
	bmu_discharge_end_process_callback = NULL;

	oz8806_temp_read_callback = NULL;
	oz8806_current_read_callback = NULL;
	oz8806_voltage_read_callback = NULL;

	oz8806_init_batt_info(the_oz8806);

	batt_info_ptr = NULL;
	gas_gauge_ptr = NULL;

	mutex_unlock(&update_mutex);
}
EXPORT_SYMBOL(unregister_bmu_callback);

void oz8806_set_batt_info_ptr(bmu_data_t  *batt_info)
{
	mutex_lock(&update_mutex);

	if (!batt_info)
	{
		bmt_dbg("BMT fgu batt_info NULL\n");
		mutex_unlock(&update_mutex);
		return;
	}

	batt_info_ptr = batt_info;

	mutex_unlock(&update_mutex);
}
EXPORT_SYMBOL(oz8806_set_batt_info_ptr);

void oz8806_set_gas_gauge(gas_gauge_t *gas_gauge)
{
	mutex_lock(&update_mutex);

	if (!gas_gauge)
	{
		bmt_dbg("BMT fgu gas_gauge NULL\n");
        mutex_unlock(&update_mutex);
		return;
	}

	gas_gauge_ptr = gas_gauge;

	if (gas_gauge_ptr->fcc_data != 0)
		the_oz8806->batt_info.batt_fcc_data = gas_gauge_ptr->fcc_data;

	if (gas_gauge_ptr->discharge_current_th != 0)
		the_oz8806->batt_info.discharge_current_th = gas_gauge_ptr->discharge_current_th;

	bmt_dbg("batt_fcc_data:%d, discharge_current_th:%d\n",
			the_oz8806->batt_info.batt_fcc_data, the_oz8806->batt_info.discharge_current_th);

	gas_gauge_ptr->ext_temp_measure = ext_thermal_read;

	mutex_unlock(&update_mutex);
}
EXPORT_SYMBOL(oz8806_set_gas_gauge);

int oz8806_get_save_capacity(void)
{
	return save_capacity;
}
EXPORT_SYMBOL(oz8806_get_save_capacity);

unsigned long oz8806_get_system_boot_time(void)
{
	unsigned long long time_sec;
	unsigned long mod;
	time_sec = ktime_to_ns(ktime_get_boottime());
	mod = do_div(time_sec,1000000);
	batt_dbg("system boottime: %lu ms\n", (unsigned long)time_sec);

	return time_sec;
}
EXPORT_SYMBOL(oz8806_get_system_boot_time);

unsigned long oz8806_get_boot_up_time(void)
{
	unsigned long long t;
	struct timespec time;
	ktime_get_ts(&time);
	t = time.tv_sec * 1000 + time.tv_nsec / 1000000;
	bmt_dbg("boot up time: %lu ms\n", (unsigned long) t);

	return t;
}
EXPORT_SYMBOL(oz8806_get_boot_up_time);

unsigned long oz8806_get_power_on_time(void)
{
	unsigned int t = 0;
	bmt_dbg("jiffies %lu,ic_wakeup_time %lu\n",jiffies,ic_wakeup_time);
	t = jiffies_to_msecs(jiffies - ic_wakeup_time);

	bmt_dbg("IC wakeup time: %u ms\n", t);

	return (unsigned long)t;
}
EXPORT_SYMBOL(oz8806_get_power_on_time);

static int oz8806_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int32_t ret = 0;
	struct oz8806_data* data = NULL;

	struct power_supply_config psy_cfg = {
		.of_node = client->dev.of_node,
	};

	bmt_dbg("BMT fgu probe start\n");
	data = devm_kzalloc(&client->dev, sizeof(struct oz8806_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Can't alloc BMT_data struct\n");
		return -ENOMEM;
	}

	// Init real i2c_client 
	i2c_set_clientdata(client, data);

	the_oz8806 = data;
	data->myclient = client;

	ret = oz8806_hw_ic_detect(data);
	if(ret < 0)
	{
		pr_err("BMT fgu do not detect ic, exit\n");
		goto register_fail;
	}

	//init battery information as soon as possible
	oz8806_wakeup_full_power();

	oz8806_init_batt_info(data);

	INIT_DELAYED_WORK(&data->work, oz8806_battery_work);
	/*****************************************************************************
	*  BMT Warning: This is a very important code here(bmulib_init).
	* The chip needs i2c client processing opt mapping and self init first in bmulib_init.
	* You must not return before this code even if you read i2c error.
	* Remind you again that it will make serious mistake if you return before bmulib_init.
	*****************************************************************************/
	if (oz8806_get_boot_up_time() < 2000)
		ic_wakeup_time = jiffies + msecs_to_jiffies(2000 - oz8806_get_boot_up_time());
	else
		ic_wakeup_time = jiffies;//msecs_to_jiffies(2000);

#ifndef CONFIG_OZ8806_M
	bmulib_init();
#else
	/*
	 * Wake up oz8806 to full power mode here.
	 * Then, after at least 2.5s, detection of current, voltage, and temperature can be stable
	 *
	 * when IC start with start-up mode, after at least 2s, IC can enter Full Power Mode.
	 */
	oz8806_wakeup_full_power();
#endif

	data->interval = BATTERY_WORK_INTERVAL * 1000;

	oz8806_powersupply_init(data);
	psy_cfg.drv_data = data,
	data->bat = power_supply_register(&client->dev, &the_oz8806->bat_desc,&psy_cfg);
	if (!data->bat) {
		bmt_dbg("failed to register power_supply battery\n");
		goto register_fail;
	}
	/*
	 * /sys/class/i2c-dev/i2c-2/device/2-002f/
	 */
	ret = oz8806_create_sys(&(client->dev), &oz8806_attribute_group);
	if(ret){
		bmt_dbg("[BATT]: Err failed to creat BMT attributes\n");
		goto bat_failed;
	}

	//alternative suspend/resume method
	data->pm_nb.notifier_call = oz8806_suspend_notifier;
	register_pm_notifier(&data->pm_nb);

	ret = device_init_wakeup(&client->dev, true);
	if (ret) 
		dev_err(&client->dev, "wakeup source init failed.\n");

	fg_hw_init_done = 1; 

	schedule_delayed_work(&data->work, 0);

	bmt_dbg("BMT fgu probe end\n");
	return 0;
bat_failed:
	power_supply_unregister(the_oz8806->bat);
register_fail:
	bmt_dbg("BMT fgu probe () fail: return %d\n",ret);
	the_oz8806 = NULL;
	return ret;
}


static int oz8806_remove(struct i2c_client *client)
{
	struct oz8806_data *data = i2c_get_clientdata(client);

	cancel_delayed_work(&data->work);

	sysfs_remove_group(&(data->myclient->dev.kobj), &oz8806_attribute_group);

	power_supply_unregister(data->bat);

	mutex_lock(&update_mutex);

	/*
	 * It should be noted here that during the shutdown process, the shutdown callback must be called,
	 * otherwise the chip may not sleep normally, resulting in abnormal capacity at the next startup.
	 *
	 * And be careful after the callback, if there is work or thread running,
	 * it may wake up the chip again, to avoid this situation.
	 */
	fg_hw_init_done = 0;//be carefull other thread call bmu_polling_loop_func and wake up chip again
	bmu_power_down_chip();

	oz8806_update_batt_info(data);

	mutex_unlock(&update_mutex);
#ifndef CONFIG_OZ8806_M
	bmulib_exit();
#endif

	return 0;
}

static void oz8806_shutdown(struct i2c_client *client)
{
	struct oz8806_data *data = i2c_get_clientdata(client);
	bmt_dbg("BMT fgu shutdown\n");

	cancel_delayed_work(&data->work);

	sysfs_remove_group(&(data->myclient->dev.kobj), &oz8806_attribute_group);

	mutex_lock(&update_mutex);

	/*
	 * It should be noted here that during the shutdown process, the shutdown callback must be called,
	 * otherwise the chip may not sleep normally, resulting in abnormal capacity at the next startup.
	 *
	 * And be careful after the callback, if there is work or thread running,
	 * it may wake up the chip again, to avoid this situation.
	 */
	fg_hw_init_done = 0; 
	bmu_power_down_chip();

	oz8806_update_batt_info(data);
	mutex_unlock(&update_mutex);
}
/*-------------------------------------------------------------------------*/

static const struct i2c_device_id oz8806_id[] = {
	{ MYDRIVER, 0 },							//string, id??
	{ }
};
MODULE_DEVICE_TABLE(i2c, oz8806_id);

static const struct of_device_id oz8806_of_match[] = {
	{.compatible = "o2micro,oz8806",},
	{},
};
MODULE_DEVICE_TABLE(of, oz8806_of_match);


static struct i2c_driver oz8806_driver = {
	.driver = {
		.name	= MYDRIVER,
		.of_match_table = oz8806_of_match,
	},
	.probe			= oz8806_probe,
	.remove		= oz8806_remove,
	.shutdown		= oz8806_shutdown,
	.id_table		= oz8806_id,
};


static int __init oz8806_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&oz8806_driver);

	if(ret != 0)
		bmt_dbg("failed to register BMT i2c driver.\n");
	else
		bmt_dbg("Success to register BMT i2c driver.\n");

	return ret;
}

static void __exit oz8806_exit(void)
{
	bmt_dbg("BMT fgu exit\n");
	i2c_del_driver(&oz8806_driver);
}

MODULE_DESCRIPTION("oz8806 Battery Monitor IC Driver");
MODULE_LICENSE("GPL");

//subsys_initcall_sync(oz8806_init);
module_init(oz8806_init);
module_exit(oz8806_exit);
