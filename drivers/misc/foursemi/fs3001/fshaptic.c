#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>	
#include "fshaptic.h"
#include "fshaptic3001.h"

static char* FSERROR = "FSERROR";
static char* FSREAD = "FSREAD";
static char* FSWRITE = "FSWRITE";


#define FOURSEMI_I2C_NAME		("fshaptic")
#define FOURSEMI_HAPTIC_NAME      ("fshaptic")
#define FS_READ_CHIPID_RETRIES	(5)
#define FS_I2C_RETRIES		(2)
#define FS3001_CHIP_ID		(0x25)
#define FS_REG_DEVID		(0x01)
#define FS_REG_REVID		(0x02)
#define FS_REG_RESET		(0x10)


#ifdef ENABLE_PIN_CONTROL
const char * const pctl_names[] = 
{
	"foursemi_reset_reset",
	"foursemi_reset_active",
	"foursemi_interrupt_active",
};
#endif

extern struct foursemi *g_foursemi;

static int foursemi_i2c_read(struct foursemi *foursemi,unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < FS_I2C_RETRIES) 
	{
		ret = i2c_smbus_read_byte_data(foursemi->i2c, reg_addr);
		if (ret < 0) 
		{
			pr_err("%s:i2c_read addr=0x%02X, cnt=%d error=%d\n", FSERROR,reg_addr, cnt, ret);
		} 
		else 
		{
			*reg_data = ret;
			pr_info("%s,addr=0x%02X, data=0x%02X\n",FSREAD,reg_addr, ret);
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int foursemi_i2c_write(struct foursemi *foursemi,unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < FS_I2C_RETRIES) 
	{
		ret = i2c_smbus_write_byte_data(foursemi->i2c, reg_addr, reg_data);
		if (ret < 0) 
		{
			pr_err("%s:i2c_write addr=0x%02X, data=0x%02X, cnt=%d, error=%d\n", FSERROR,reg_addr, reg_data, cnt, ret);
		} 
		else 
		{
			pr_info("%s,addr=0x%02X, data=0x%02X\n",FSWRITE,reg_addr, reg_data);
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

#ifdef ENABLE_PIN_CONTROL
static int select_pin_ctl(struct foursemi *foursemi, const char *name)
{
	size_t i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(foursemi->pinctrl_state); i++) 
	{
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) 
		{
			rc = pinctrl_select_state(foursemi->foursemi_pinctrl,foursemi->pinctrl_state[i]);
			if (rc)
				pr_err("%s:cannot select '%s'\n", FSERROR,name);
			else
				pr_info("selected '%s'\n", name);
			goto exit;
		}
	}

	rc = -EINVAL;
	pr_info("%s: not found\n", name);

exit:
	return rc;
}

//use select_pin_ctl to set pin as foursemi_interrupt_active setting
static int foursemi_set_interrupt(struct foursemi *foursemi)
{
	int rc = select_pin_ctl(foursemi, "foursemi_interrupt_active");
	return rc;
}
#endif

static int foursemi_hw_reset(struct foursemi *foursemi)
{
#ifdef ENABLE_PIN_CONTROL
	int rc  = 0;
#endif
	pr_info("enter\n");
#ifdef ENABLE_PIN_CONTROL
	rc = select_pin_ctl(foursemi, "foursemi_reset_active");
	msleep(5);
	rc = select_pin_ctl(foursemi, "foursemi_reset_reset");
	msleep(5);
	rc = select_pin_ctl(foursemi, "foursemi_reset_active");
#endif
	if (foursemi && gpio_is_valid(foursemi->reset_gpio)) 
	{
		gpio_set_value_cansleep(foursemi->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(foursemi->reset_gpio, 1);
		usleep_range(3500, 4000);
		pr_info("hw_reset success\n");
	} 
	else 
	{
		pr_err("%s:hw_reset fail\n",FSERROR);
	}
	return 0;
}

int foursemi_haptic_softreset(struct foursemi *foursemi)
{
	pr_info("enter\n");
	foursemi_i2c_write(foursemi, FS_REG_RESET, 0x02);
	usleep_range(2000, 2500);
	return 0;
}

static int foursemi_read_chipid(struct foursemi *foursemi,unsigned char *reg, unsigned char type)
{
	int ret = -1;
	unsigned char cnt = 0;

	pr_info("foursemi i2c addr = 0x%02x", foursemi->i2c->addr);
	while (cnt < FS_I2C_RETRIES) 
	{
		ret = i2c_smbus_read_byte_data(foursemi->i2c, FS_REG_DEVID);
		if (ret < 0) 
		{
			if (type == FS_FIRST_TRY) 
			{
				pr_info("reading chip id\n");
			} 
			else if (type == FS_LAST_TRY) 
			{
				pr_err("%s:i2c_read cnt=%d error=%d\n", FSERROR,cnt, ret);
			} 
			else 
			{
				pr_err("%s:type is error\n",FSERROR);
			}
		} 
		else 
		{
			*reg = ret;
			break;
		}
		cnt++;
		usleep_range(2000, 3000);
	}

	return ret;
}

static int foursemi_parse_chipid(struct foursemi *foursemi)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;
	unsigned char rev_id = 0xff;

	pr_info("enter\n");
	while (cnt < FS_READ_CHIPID_RETRIES) 
	{
		//hardware reset
		foursemi_hw_reset(foursemi);
		ret = foursemi_read_chipid(foursemi, &reg, FS_FIRST_TRY);
		if (ret < 0) 
		{
			//i2c->addr is from fshaptic@58, maybe it's different from fs3001_i2c_addr
			foursemi->i2c->addr = (u16)foursemi->fs3001_i2c_addr;
			pr_info("try to replace i2c addr [(0x%02X)] to read chip id again\n",foursemi->i2c->addr);
			ret = foursemi_read_chipid(foursemi, &reg, FS_LAST_TRY);
			if (ret < 0)
				break;
		}
		//rev_id
		switch (reg) 
		{
			case FS3001_CHIP_ID:
				foursemi_i2c_read(foursemi, FS_REG_REVID, &rev_id);
				if (rev_id == FS3001_A1) 
				{
					foursemi->name = FS3001_A1;
					pr_info("FS3001_A1 detected\n");
					foursemi_haptic_softreset(foursemi);
					return 0;
				} 
				else if (rev_id == FS3001_A2) 
				{
					foursemi->name = FS3001_A2;
					pr_info("FS3001_A2 detected\n");
					foursemi_haptic_softreset(foursemi);
					return 0;
				} 
				else if (rev_id == FS3001_A3) 
				{
					foursemi->name = FS3001_A3;
					pr_info("FS3001_A3 detected\n");
					foursemi_haptic_softreset(foursemi);
					return 0;
				} 
				else 
				{
					pr_info("unsupported ef_id = (0x%02X)\n",rev_id);
					break;
				}
			default:
				pr_info("unsupported device revision (0x%x)\n", reg);
				break;
		}
		cnt++;

		usleep_range(2000, 3000);
	}

	return -EINVAL;
}

static int foursemi_parse_dt(struct device *dev, struct foursemi *foursemi,struct device_node *np) 
{
	unsigned int val = 0;

	pr_info("enter\n");
	//0 should match 0 in <&tlmm 156 0>;
	foursemi->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (foursemi->reset_gpio >= 0) 
	{
		pr_info("reset gpio provided ok\n");
	} 
	else 
	{
		foursemi->reset_gpio = -1;
		pr_err("%s:no reset gpio provided, will not HW reset device\n",FSERROR);
		return -1;
	}
	foursemi->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (foursemi->irq_gpio < 0) 
	{
		dev_err(dev, "no irq gpio provided.\n");
		foursemi->IsUsedIRQ = false;
	} 
	else 
	{
		pr_info("irq gpio provided ok.\n");
		foursemi->IsUsedIRQ = true;
	}
	//fs3001_i2c_addr = < 0x34 >;
	val = of_property_read_u32(np,"fs3001_i2c_addr", &foursemi->fs3001_i2c_addr);
	if (val)
		pr_err("%s:configure fs3001_i2c_addr error\n",FSERROR);
	else
		pr_info("configure fs3001_i2c_addr ok\n");
	return 0;
}

static int
foursemi_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
// * struct ff_device - force-feedback part of an input device
// * @upload: Called to upload an new effect into device
// * @erase: Called to erase an effect from device
// * @playback: Called to request device to start playing specified effect
// * @set_gain: Called to set specified gain
// * @set_autocenter: Called to auto-center device
// * @destroy: called by input core when parent input device is being
// *	destroyed
// * @private: driver-specific data, will be freed automatically
// * @ffbit: bitmap of force feedback capabilities truly supported by
// *	device (not emulated like ones in input_dev->ffbit)
// * @mutex: mutex for serializing access to the device
// * @max_effects: maximum number of effects supported by device
// * @effects: pointer to an array of effects currently loaded into device
// * @effect_owners: array of effect owners; when file handle owning
// *	an effect gets closed the effect is automatically erased
// *
// * Every force-feedback device must implement upload() and playback()
// * methods; erase() is optional. set_gain() and set_autocenter() need
// * only be implemented if driver sets up FF_GAIN and FF_AUTOCENTER
// * bits.
// *
// * Note that playback(), set_gain() and set_autocenter() are called with
// * dev->event_lock spinlock held and interrupts off and thus may not
// * sleep.
// struct ff_device {
//	int (*upload)(struct input_dev *dev, struct ff_effect *effect,
//		      struct ff_effect *old);
//	int (*erase)(struct input_dev *dev, int effect_id);
//
//	int (*playback)(struct input_dev *dev, int effect_id, int value);
//	void (*set_gain)(struct input_dev *dev, u16 gain);
//	void (*set_autocenter)(struct input_dev *dev, u16 magnitude);
//
//	void (*destroy)(struct ff_device *);
//
//	void *private;
//
//	unsigned long ffbit[BITS_TO_LONGS(FF_CNT)];
//
//	struct mutex mutex;
//
//	int max_effects;
//	struct ff_effect *effects;
//	struct file *effect_owners[];
//};
	struct foursemi *foursemi;
	struct input_dev *input_dev;
	struct device_node *np = i2c->dev.of_node;
	struct ff_device *ff;
	int rc = 0;
	int effect_count_max;
	int ret = -1;
	int irq_flags = 0;
#ifdef ENABLE_PIN_CONTROL
	int i;
#endif

	pr_info("enter\n");
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("%s:check_functionality failed\n",FSERROR);
		return -EIO;
	}

	foursemi = devm_kzalloc(&i2c->dev, sizeof(struct foursemi), GFP_KERNEL);
	if (foursemi == NULL)
		return -ENOMEM;

	//input dev
	input_dev = devm_input_allocate_device(&i2c->dev);
	if (!input_dev)
		return -ENOMEM;

	//save i2c_client,device to foursemi struct
	foursemi->dev = &i2c->dev;
	foursemi->i2c = i2c;
	//save foursemi to i2c->dev->driver_data  (Driver data, set and get with dev_set/get_drvdata)
	i2c_set_clientdata(i2c, foursemi);
	//get data from dts
	if (np) 
	{
		ret = foursemi_parse_dt(&i2c->dev, foursemi, np);
		if (ret) 
		{
			pr_err("%s:failed to parse device tree node\n",FSERROR);
			goto err_parse_dt;
		}
	}
  
	foursemi->enable_pin_control = 0;
#ifdef ENABLE_PIN_CONTROL
	//devm_pinctrl_get gets pinctrl info, reture value is a struct pinctrl *foursemi_pinctrl;
	foursemi->foursemi_pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(foursemi->foursemi_pinctrl )) 
	{
		if (PTR_ERR(foursemi->foursemi_pinctrl ) == -EPROBE_DEFER) 
		{
			pr_err("%s:pinctrl not ready\n",FSERROR);
			return -EPROBE_DEFER;
		}
		pr_err("%s:target does not use pinctrl\n",FSERROR);
		foursemi->foursemi_pinctrl  = NULL;
		rc = -EINVAL;
		return rc;
	}
	for (i = 0; i < ARRAY_SIZE(foursemi->pinctrl_state); i++) 
	{
		const char *n = pctl_names[i];
		//get pin_state(get it from different pctl_names setting)
		struct pinctrl_state *state = pinctrl_lookup_state(foursemi->foursemi_pinctrl, n);
		if (!IS_ERR(state)) 
		{
			pr_info("found pin control %s\n", n);
			foursemi->pinctrl_state[i] = state;
			foursemi->enable_pin_control = 1;
			foursemi_set_interrupt(foursemi);
			continue;
		}
		pr_err("%s:cannot find '%s'\n",FSERROR, n);
	}
#endif
	//if not find pin_control or ENABLE_PIN_CONTROL not defined
	//pull it down and set INT as input
	if (!foursemi->enable_pin_control) 
	{
		pr_info("foursemi->enable_pin_control==1\n");
		if (gpio_is_valid(foursemi->reset_gpio)) 
		{
			pr_info("gpio_is_valid(foursemi->reset_gpio)==1\n");
			//will be pull up by foursemi_hw_reset
			ret = devm_gpio_request_one(&i2c->dev, foursemi->reset_gpio,GPIOF_OUT_INIT_LOW, "foursemi_rst");
			if (ret) 
			{
				pr_err("%s:rst request failed\n",FSERROR);
				goto err_reset_gpio_request;
			}
		}
	}
	//irq-gpio = <&tlmm 157 0>;
	if (gpio_is_valid(foursemi->irq_gpio)) 
	{
		pr_info("gpio_is_valid(foursemi->irq_gpio)==1\n");
		ret = devm_gpio_request_one(&i2c->dev, foursemi->irq_gpio, GPIOF_DIR_IN, "foursemi_int");
		if (ret)
		{
			pr_err("%s:int request failed\n",FSERROR);
			goto err_irq_gpio_request;
		}
	}
	//get device id
	ret = foursemi_parse_chipid(foursemi);
	if (ret < 0) 
	{
		pr_err("%s:foursemi parse chipid failed\n",FSERROR);
		goto err_id;
	}
	//alloc fs3001 struct
	if (foursemi->name == FS3001_A1 || foursemi->name == FS3001_A2 || foursemi->name == FS3001_A3)
	{
		foursemi->fs3001 = devm_kzalloc(&i2c->dev, sizeof(struct fs3001),GFP_KERNEL);
		if (foursemi->fs3001 == NULL) 
		{
			if (gpio_is_valid(foursemi->irq_gpio))
			{
				//devm_gpio_free(&i2c->dev, foursemi->irq_gpio);
			}
				
			if (gpio_is_valid(foursemi->reset_gpio) && (!foursemi->enable_pin_control))
            {
				//devm_gpio_free(&i2c->dev, foursemi->reset_gpio);
			}
			
			devm_kfree(&i2c->dev, foursemi);
			foursemi = NULL;
			return -ENOMEM;
		}

		//save settings
		foursemi->fs3001->dev = foursemi->dev;
		foursemi->fs3001->i2c = foursemi->i2c;
		foursemi->fs3001->reset_gpio = foursemi->reset_gpio;
		foursemi->fs3001->irq_gpio = foursemi->irq_gpio;
		foursemi->fs3001->isUsedIntn = foursemi->IsUsedIRQ;
		//xxxx_fs3001->gain ? fs3001->level
		foursemi->fs3001->level = 0x80;
		foursemi->fs3001->f0_cali_data = 0x90;
		foursemi->fs3001->osc_cali_data= 0x90;
		foursemi->fs3001->offset = 512;

		//otp?
		if (!fs3001_check_qualify(foursemi->fs3001)) 
		{
			pr_err("%s:unqualified chip!\n",FSERROR);
			goto err_fs3001_check_qualify;  //zzzz: remove it for fpga test
		}

		//load fs3001 dts setting to foursemi->fs3001
		if (np) 
		{
			ret = fs3001_parse_dt(&i2c->dev, foursemi->fs3001, np);
			if (ret) 
			{
				pr_err("%s:failed to parse device tree node\n",FSERROR);
				goto err_fs3001_parse_dt;
			}
		}

		//irq
		if (gpio_is_valid(foursemi->fs3001->irq_gpio) && !(foursemi->fs3001->flags & FS3001_FLAG_SKIP_INTERRUPTS)) {
			//register irq handler
			fs3001_interrupt_setup(foursemi->fs3001);
			irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
			ret = devm_request_threaded_irq(&i2c->dev,gpio_to_irq(foursemi->fs3001->irq_gpio),NULL, fs3001_irq, irq_flags,"fs3001", foursemi->fs3001);
			if (ret != 0) 
			{
				pr_err("%s:failed to request IRQ %d: %d\n",FSERROR,gpio_to_irq(foursemi->fs3001->irq_gpio),ret);
				goto err_fs3001_irq;
			}
		} 
		else 
		{
			pr_err("%s:skipping IRQ registration\n",FSERROR);
			//disable feature support if gpio was invalid 
			foursemi->fs3001->flags |= FS3001_FLAG_SKIP_INTERRUPTS;
		}

		//fs3001_vibrator_init
		//1:init fs3001->timer
		//2:init fs3001->long_vibrate_work,fs3001->rtp_work
		//3:init fs3001->lock,fs3001->rtp_lock
		//4:init fs3001->wait_q,fs3001->stop_wait_q
		fs3001_vibrator_init(foursemi->fs3001);
		//fs3001 reg init
		fs3001_reg_init(foursemi->fs3001);
		//fs3001 f0 cali setting
		fs3001_f0_cali_setting_init(foursemi->fs3001);
		//fs3001_haptic_init
		//1:init fs3001->haptic_audio.timer
		//2:init fs3001->haptic_audio.ctr_list(list_head)
		//3:init fs3001->haptic_audio.work
		//4:init fs3001->haptic_audio.lock
		//5:set gain,pwm,cont,brake,trig
		//6:offset
		//7:vbat
		//8:auto brake
		//9:f0 cali
		fs3001_haptic_init(foursemi->fs3001);
		//load ram bin
		fs3001_ram_work_init(foursemi->fs3001);

		//fs3001 input config
		input_dev->name = "fshaptic";
		input_set_drvdata(input_dev, foursemi->fs3001);
		foursemi->fs3001->input_dev = input_dev;
		input_set_capability(input_dev, EV_FF, FF_CONSTANT);
		input_set_capability(input_dev, EV_FF, FF_GAIN);
		if (foursemi->fs3001->effects_count != 0) 
		{
			input_set_capability(input_dev, EV_FF, FF_PERIODIC);
			input_set_capability(input_dev, EV_FF, FF_CUSTOM);
		}

		if (foursemi->fs3001->effects_count + 1 > FF_EFFECT_COUNT_MAX)
			effect_count_max = foursemi->fs3001->effects_count + 1;
		else
			effect_count_max = FF_EFFECT_COUNT_MAX;
		
		rc = input_ff_create(input_dev, effect_count_max);
		if (rc < 0) 
		{
			dev_err(foursemi->fs3001->dev, "create FF input device failed, rc=%d\n",rc);
			return rc;
		}
		else
		{
			pr_info("input_ff_create ok, rc=%d\n",rc);
		}

		foursemi->fs3001->work_queue = create_singlethread_workqueue("fs3001_vibrator_work_queue");
		if (!foursemi->fs3001->work_queue) 
		{
			dev_err(&i2c->dev,"Error creating fs3001_vibrator_work_queue\n");
			goto err_fs3001_sysfs;
		}
		INIT_WORK(&foursemi->fs3001->set_gain_work,fs3001_haptics_set_gain_work_routine);
		ff = input_dev->ff;
		ff->upload = fs3001_haptics_upload_effect;
		ff->playback = fs3001_haptics_playback;
		ff->erase = fs3001_haptics_erase;
		ff->set_gain = fs3001_haptics_set_gain;
		rc = input_register_device(input_dev);
		if (rc < 0) 
		{
			pr_err("%s:register input device failed, rc=%d\n",FSERROR,rc);
			goto fs3001_destroy_ff;
		}
		else
		{
			pr_info("input_register_device ok, rc=%d\n",rc);
		}
	} 
	else
	{
		goto err_parse_dt;
	}

	dev_set_drvdata(&i2c->dev, foursemi);
	g_foursemi = foursemi;
	pr_info("probe completed successfully!\n");
	return 0;


err_fs3001_sysfs:
	if (foursemi->name == FS3001_A1 || foursemi->name == FS3001_A2 || foursemi->name == FS3001_A3)
		devm_free_irq(&i2c->dev, gpio_to_irq(foursemi->fs3001->irq_gpio),foursemi->fs3001);
fs3001_destroy_ff:
	if (foursemi->name == FS3001_A1 || foursemi->name == FS3001_A2 || foursemi->name == FS3001_A3)
		input_ff_destroy(foursemi->fs3001->input_dev);
err_fs3001_irq:
err_fs3001_parse_dt:
err_fs3001_check_qualify:   //for fpga test + //
	if (foursemi->name == FS3001_A1 || foursemi->name == FS3001_A2 || foursemi->name == FS3001_A3)
	{
		devm_kfree(&i2c->dev, foursemi->fs3001);
		foursemi->fs3001 = NULL;
	}
err_id:
	if (gpio_is_valid(foursemi->irq_gpio))
    {
		//devm_gpio_free(&i2c->dev, foursemi->irq_gpio);
	}			
err_irq_gpio_request:
	if (gpio_is_valid(foursemi->reset_gpio) && (!foursemi->enable_pin_control))
    {
		//devm_gpio_free(&i2c->dev, foursemi->reset_gpio);
	}		

err_reset_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, foursemi);
	foursemi = NULL;
	return ret;

}

static void foursemi_i2c_remove(struct i2c_client *i2c)
{
	struct foursemi *foursemi = i2c_get_clientdata(i2c);

	pr_info("enter \n");

	if (foursemi->name == FS3001_A1 || foursemi->name == FS3001_A2 || foursemi->name == FS3001_A3)
	{
		pr_info("remove fs3001\n");
		cancel_delayed_work_sync(&foursemi->fs3001->ram_work);
		cancel_work_sync(&foursemi->fs3001->haptic_audio.work);
		hrtimer_cancel(&foursemi->fs3001->haptic_audio.timer);
		if (foursemi->fs3001->isUsedIntn)
			cancel_work_sync(&foursemi->fs3001->rtp_work);
		cancel_work_sync(&foursemi->fs3001->long_vibrate_work);
#ifdef FS_HAPSTREAM
		proc_remove(foursemi->fs3001->fs_config_proc);
		free_pages((unsigned long)foursemi->fs3001->start_buf, HAPSTREAM_MMAP_PAGE_ORDER);
		foursemi->fs3001->start_buf = NULL;
#endif
		hrtimer_cancel(&foursemi->fs3001->timer);
		mutex_destroy(&foursemi->fs3001->lock);
		mutex_destroy(&foursemi->fs3001->rtp_lock);
		mutex_destroy(&foursemi->fs3001->haptic_audio.lock);
		sysfs_remove_group(&foursemi->fs3001->i2c->dev.kobj,&fs3001_vibrator_attribute_group);
		devm_free_irq(&i2c->dev, gpio_to_irq(foursemi->fs3001->irq_gpio),foursemi->fs3001);
#ifdef TIMED_OUTPUT
		timed_output_dev_unregister(&foursemi->fs3001->vib_dev);
#endif
		devm_kfree(&i2c->dev, foursemi->fs3001);
		foursemi->fs3001 = NULL;
	}
	else 
	{
		pr_err("%s:no chip\n",FSERROR);
		return;
	}

	pr_info("exit\n");
	return;
}


static const struct i2c_device_id foursemi_i2c_id[] = 
{
	{ FOURSEMI_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, foursemi_i2c_id);
static const struct of_device_id foursemi_dt_match[] = 
{
	{ .compatible = "foursemi,fshaptic" },
	{ },
};
static struct i2c_driver foursemi_i2c_driver = 
{
	.driver = 
	{
		//yyyy_search 38980991 in doc
		.name = FOURSEMI_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(foursemi_dt_match),//match with dts "compatible"
	},
	.probe = foursemi_i2c_probe,
	.remove = foursemi_i2c_remove,
	.id_table = foursemi_i2c_id,
};

static int __init foursemi_i2c_init(void)
{
	int ret = 0;

	pr_info("foursemi driver version %s\n", FOURSEMI_DRIVER_VERSION);
	//add i2c driver
	ret = i2c_add_driver(&foursemi_i2c_driver);
	if (ret) 
	{
		pr_err("%s:fail to add foursemi device into i2c\n",FSERROR);
		return ret;
	}

	return 0;
}
module_init(foursemi_i2c_init);

static void __exit foursemi_i2c_exit(void)
{
	//delete i2c driver
	i2c_del_driver(&foursemi_i2c_driver);
}

module_exit(foursemi_i2c_exit);

MODULE_DESCRIPTION("Foursemi Haptic Driver");
MODULE_LICENSE("GPL");