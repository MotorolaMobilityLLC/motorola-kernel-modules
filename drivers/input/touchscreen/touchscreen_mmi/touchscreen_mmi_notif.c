/*
 * Copyright (C) 2019 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/power_supply.h>
#include <linux/touchscreen_mmi.h>
#include <linux/fingerprint_mmi.h>

#if defined(CONFIG_DRM_DYNAMIC_REFRESH_RATE)
extern struct blocking_notifier_head dsi_freq_head;
#define register_dynamic_refresh_rate_notifier(_nb) \
	blocking_notifier_chain_register(&dsi_freq_head, _nb)
#define unregister_dynamic_refresh_rate_notifier(_nb) \
	blocking_notifier_chain_unregister(&dsi_freq_head, _nb)
#else
#define register_dynamic_refresh_rate_notifier(...) ret
#define unregister_dynamic_refresh_rate_notifier(...)
#endif

#define IS_GESTURE_MODE (touch_cdev->pm_mode == TS_MMI_PM_GESTURE ? 1 : 0)
#define IS_DEEPSLEEP_MODE (touch_cdev->pm_mode == TS_MMI_PM_DEEPSLEEP ? 1 : 0)
#define IS_ACTIVE_MODE (touch_cdev->pm_mode == TS_MMI_PM_ACTIVE ? 1 : 0)
#define NEED_TO_SET_PINCTRL \
					(touch_cdev->pdata.power_off_suspend && \
					touch_cdev->mdata->pinctrl && \
					IS_DEEPSLEEP_MODE)
#define NEED_TO_SET_POWER \
					(touch_cdev->pdata.power_off_suspend && \
					touch_cdev->mdata->power && \
					IS_DEEPSLEEP_MODE)

enum ts_mmi_work {
	TS_MMI_DO_RESUME,
	TS_MMI_DO_PS,
	TS_MMI_DO_REFRESH_RATE,
	TS_MMI_DO_FPS,
};

static int ts_mmi_panel_off(struct ts_mmi_dev *touch_cdev) {
	int ret = 0;

	if (atomic_cmpxchg(&touch_cdev->touch_stopped, 0, 1) == 1)
		return 0;

	atomic_set(&touch_cdev->resume_should_stop, 1);

	TRY_TO_CALL(pre_suspend);
	if (touch_cdev->pdata.gestures_enabled) {
		if(ts_mmi_is_sensor_enable()) {
			dev_info(DEV_MMI, "%s: try to enter Gesture mode\n", __func__);
			TRY_TO_CALL(panel_state, touch_cdev->pm_mode, TS_MMI_PM_GESTURE);
			touch_cdev->pm_mode = TS_MMI_PM_GESTURE;
		}
	}
	if (IS_ACTIVE_MODE) {
		/* IC power is off. IRQ pin status is floated. So disable IRQ. */
		dev_info(DEV_MMI, "%s: try to enter Deepsleep mode\n", __func__);
		TRY_TO_CALL(panel_state, touch_cdev->pm_mode, TS_MMI_PM_DEEPSLEEP);
		TRY_TO_CALL(drv_irq, TS_MMI_IRQ_OFF);
		touch_cdev->pm_mode = TS_MMI_PM_DEEPSLEEP;
	}

#ifdef TS_MMI_TOUCH_EDGE_GESTURE
	if (touch_cdev->pdata.gestures_enabled)
		ts_mmi_gesture_suspend(touch_cdev);
#endif
	TRY_TO_CALL(post_suspend);

	dev_info(DEV_MMI, "%s: done\n", __func__);

	return 0;
}

static int inline ts_mmi_panel_on(struct ts_mmi_dev *touch_cdev) {
	atomic_set(&touch_cdev->resume_should_stop, 0);
	kfifo_put(&touch_cdev->cmd_pipe, TS_MMI_DO_RESUME);
	/* schedule_delayed_work returns true if work has been scheduled */
	/* and false otherwise, thus return 0 on success to comply POSIX */
	return schedule_delayed_work(&touch_cdev->work, 0) == false;
}

static int ts_mmi_panel_cb(struct notifier_block *nb,
		unsigned long event, void *evd)
{
	int idx = -1;
	struct ts_mmi_dev *touch_cdev =
		container_of(nb, struct ts_mmi_dev, panel_nb);
	int ret = 0;

	GET_CONTROL_DSI_INDEX;

	if (!touch_cdev)
		return 0;

	dev_dbg(DEV_MMI,"%s: %s event(%lu), ctrl_dsi=%d, idx=%d\n", __func__,
		EVENT_PRE_DISPLAY_OFF ? "EVENT_PRE_DISPLAY_OFF" :
		(EVENT_DISPLAY_OFF ?     "EVENT_DISPLAY_OFF" :
		(EVENT_PRE_DISPLAY_ON ?   "EVENT_PRE_DISPLAY_ON" :
		(EVENT_DISPLAY_ON ?       "EVENT_DISPLAY_ON" : "Unknown"))),
		event, touch_cdev->pdata.ctrl_dsi, idx);

	if (touch_cdev->pdata.ctrl_dsi != idx)
		return 0;

	/* entering suspend upon early blank event */
	/* to ensure shared power supply is still on */
	/* for in-cell design touch solutions */
	if (EVENT_PRE_DISPLAY_OFF) {
		cancel_delayed_work_sync(&touch_cdev->work);
		ts_mmi_panel_off(touch_cdev);
		if (NEED_TO_SET_PINCTRL) {
			dev_dbg(DEV_MMI, "%s: touch pinctrl off\n", __func__);
			TRY_TO_CALL(pinctrl, TS_MMI_PINCTL_OFF);
		}
	} else if (EVENT_DISPLAY_OFF) {
		if (NEED_TO_SET_POWER) {
			/* then proceed with de-powering */
			TRY_TO_CALL(power, TS_MMI_POWER_OFF);
			dev_dbg(DEV_MMI, "%s: touch powered off\n", __func__);
		}
	} else if (EVENT_PRE_DISPLAY_ON) {
		if (NEED_TO_SET_POWER) {
			/* powering on early */
			TRY_TO_CALL(power, TS_MMI_POWER_ON);
			dev_dbg(DEV_MMI, "%s: touch powered on\n", __func__);
		} else if (touch_cdev->pdata.reset &&
			touch_cdev->mdata->reset) {
			/* Power is not off in previous suspend.
			 * But need reset IC in resume.
			 */
			dev_dbg(DEV_MMI, "%s: resetting...\n", __func__);
			TRY_TO_CALL(reset, TS_MMI_RESET_HARD);
		}
	} else if (EVENT_DISPLAY_ON) {
		/* out of reset to allow wait for boot complete */
		if (NEED_TO_SET_PINCTRL) {
			TRY_TO_CALL(pinctrl, TS_MMI_PINCTL_ON);
			dev_dbg(DEV_MMI, "%s: touch pinctrl_on\n", __func__);
		}
		ts_mmi_panel_on(touch_cdev);
	} else {
		dev_dbg(DEV_TS, "%s: function not implemented\n", __func__);
	}

	return 0;
}

static inline void ts_mmi_restore_settings(struct ts_mmi_dev *touch_cdev)
{
	int ret = 0;

	if (touch_cdev->pdata.usb_detection)
		TRY_TO_CALL(charger_mode, (int)touch_cdev->ps_is_present);
	if (touch_cdev->pdata.update_refresh_rate)
		TRY_TO_CALL(refresh_rate, (int)touch_cdev->refresh_rate);
	if (touch_cdev->pdata.suppression_ctrl)
		TRY_TO_CALL(suppression, (int)touch_cdev->suppression);
	if (touch_cdev->pdata.pill_region_ctrl)
		TRY_TO_CALL(pill_region, (unsigned int *)touch_cdev->pill_region);
	if (touch_cdev->pdata.hold_distance_ctrl)
		TRY_TO_CALL(hold_distance, (int)touch_cdev->hold_distance);
	if (touch_cdev->pdata.gs_distance_ctrl)
		TRY_TO_CALL(gs_distance, (int)touch_cdev->gs_distance);

	dev_dbg(DEV_MMI, "%s: done\n", __func__);
}

static void ts_mmi_queued_resume(struct ts_mmi_dev *touch_cdev)
{
	bool wait4_boot_complete = true;
	int ret = 0;

	if (atomic_cmpxchg(&touch_cdev->touch_stopped, 1, 0) == 0)
		return;

	TRY_TO_CALL(pre_resume);

	/* touch IC baseline update always done when IC resume.
	 * So touchscreen class need let vendor driver know baseline update work need to be done
	 * or not before vendor resume is called.
	 */
	if (touch_cdev->pdata.fps_detection) {
		if (touch_cdev->fps_state) {
			TRY_TO_CALL(update_baseline, TS_MMI_UPDATE_BASELINE_OFF);
			touch_cdev->delay_baseline_update = true;
		}
		if (!touch_cdev->fps_state) {
			TRY_TO_CALL(update_baseline, TS_MMI_UPDATE_BASELINE_ON);
			touch_cdev->delay_baseline_update = false;
		}
	}

	if (NEED_TO_SET_POWER) {
		/* power turn on in PANEL_EVENT_PRE_DISPLAY_ON.
		 * IC need some time to boot up.
		 * Check IC is ready or not.
		 */
		TRY_TO_CALL(wait_for_ready);
	} else if (!touch_cdev->pdata.reset) {
		/* IC power is not down in suspend.
		 * IC also do not need reset in resume.
		 * So IC RAM is not lost, just change IC working mode to normal mode.
		 */
		TRY_TO_CALL(panel_state, touch_cdev->pm_mode, TS_MMI_PM_ACTIVE);
		wait4_boot_complete = false;
	}

	if (wait4_boot_complete) {
		/* IC is just power on or reseted.
		 * Need setup post resume work.
		 */
		if (touch_cdev->pdata.power_off_suspend)
			TRY_TO_CALL(drv_irq, TS_MMI_IRQ_ON);
	}

	if (IS_DEEPSLEEP_MODE)
		TRY_TO_CALL(drv_irq, TS_MMI_IRQ_ON);

	TRY_TO_CALL(post_resume);

	/* Incase user space interface is R/W during restore cached value,
	 * hold extif mutex when restore those values.
	 */
	mutex_lock(&touch_cdev->extif_mutex);
	ts_mmi_restore_settings(touch_cdev);
	touch_cdev->pm_mode = TS_MMI_PM_ACTIVE;
	mutex_unlock(&touch_cdev->extif_mutex);
	dev_info(DEV_MMI, "%s: done\n", __func__);
}

static void ts_mmi_worker_func(struct work_struct *w)
{
	struct delayed_work *dw =
		container_of(w, struct delayed_work, work);
	struct ts_mmi_dev *touch_cdev =
		container_of(dw, struct ts_mmi_dev, work);
	int ret, cmd = 0;

	while (kfifo_get(&touch_cdev->cmd_pipe, &cmd)) {
		switch (cmd) {
		case TS_MMI_DO_RESUME:
			ret = atomic_read(&touch_cdev->resume_should_stop);
			if (ret) {
				dev_info(DEV_MMI, "%s: resume cancelled\n", __func__);
				break;
			}
			ts_mmi_queued_resume(touch_cdev);
				break;

		case TS_MMI_DO_PS:
			TRY_TO_CALL(charger_mode, (int)touch_cdev->ps_is_present);
				break;

		case TS_MMI_DO_REFRESH_RATE:
			TRY_TO_CALL(refresh_rate, (int)touch_cdev->refresh_rate);
				break;
		case TS_MMI_DO_FPS:
			if (touch_cdev->fps_state) {/* on */
				TRY_TO_CALL(update_baseline, TS_MMI_UPDATE_BASELINE_OFF);
				touch_cdev->delay_baseline_update = true;
			} else { /* off */
				if (touch_cdev->delay_baseline_update) {
					TRY_TO_CALL(update_baseline, TS_MMI_UPDATE_BASELINE_ON);
					touch_cdev->delay_baseline_update = false;
				}
			}
				break;
		default:
			dev_dbg(DEV_MMI, "%s: unknown command %d\n", __func__, cmd);
		}
	}
}

static int ts_mmi_refresh_rate_cb(struct notifier_block *nb,
		unsigned long refresh_rate, void *ptr)
{
	struct ts_mmi_dev *touch_cdev =
		container_of(nb, struct ts_mmi_dev, freq_nb);
	bool do_calibration = false;

	if (!touch_cdev)
		return 0;

	if (!touch_cdev->refresh_rate ||
		(touch_cdev->refresh_rate != (refresh_rate & 0xFF))) {
		touch_cdev->refresh_rate = refresh_rate & 0xFF;
		if (is_touch_active)
			do_calibration = true;
	}

	if (do_calibration) {
		kfifo_put(&touch_cdev->cmd_pipe, TS_MMI_DO_REFRESH_RATE);
		schedule_delayed_work(&touch_cdev->work, 0);
	}

	return 0;
}

static int ts_mmi_ps_get_state(struct power_supply *psy, bool *present)
{
	union power_supply_propval pval = {0};
	int ret;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT, &pval);
	if (ret)
		return ret;
	*present = !pval.intval ? false : true;
	return 0;
}

static int ts_mmi_charger_cb(struct notifier_block *self,
				unsigned long event, void *ptr)
{
	struct ts_mmi_dev *touch_cdev = container_of(
					self, struct ts_mmi_dev, ps_notif);
	struct power_supply *psy = ptr;
	int ret;
	bool present;

	if (!((event == PSY_EVENT_PROP_CHANGED) && psy &&
			psy->desc->get_property && psy->desc->name &&
			!strncmp(psy->desc->name, "usb", sizeof("usb"))))
		return 0;

	ret = ts_mmi_ps_get_state(psy, &present);
	if (ret) {
		dev_err(DEV_MMI, "%s: failed to get usb status: %d\n",
				__func__, ret);
		return ret;
	}

	dev_dbg(DEV_MMI, "%s: event=%lu, usb status: cur=%d, prev=%d\n",
				__func__, event, present, touch_cdev->ps_is_present);

	if (touch_cdev->ps_is_present != present) {
		touch_cdev->ps_is_present = present;
		if (is_touch_active) {
			kfifo_put(&touch_cdev->cmd_pipe, TS_MMI_DO_PS);
			schedule_delayed_work(&touch_cdev->work, 0);
		}
	}

	return 0;
}

static int ts_mmi_fps_cb(struct notifier_block *self,
				unsigned long event, void *p)
{
	int fps_state = *(int *)p;
	struct ts_mmi_dev *touch_cdev = container_of(
					self, struct ts_mmi_dev, fps_notif);

	if (touch_cdev && event == 0xBEEF && fps_state != touch_cdev->fps_state) {
		touch_cdev->fps_state = fps_state;
		dev_info(DEV_MMI, "FPS: state is %s\n", touch_cdev->fps_state ? "ON" : "OFF");
		if (is_touch_active) {
			kfifo_put(&touch_cdev->cmd_pipe, TS_MMI_DO_FPS);
			schedule_delayed_work(&touch_cdev->work, 0);
		}
	}

	return 0;
}

static int ts_mmi_fps_notifier_register(struct ts_mmi_dev *touch_cdev, bool enable) {
	int (*register_link)(struct notifier_block *nb, unsigned long stype, bool report);
	int (*unregister_link)(struct notifier_block *nb, unsigned long stype);
	int ret;

	if (enable) {
		register_link = symbol_get(FPS_register_notifier);
		if (register_link) {
			touch_cdev->fps_notif.notifier_call = ts_mmi_fps_cb;
			ret = register_link(&touch_cdev->fps_notif, 0xBEEF, false);
			symbol_put(FPS_register_notifier);
			if (ret < 0) {
				dev_err(DEV_TS,
					"Failed to register fps_notifier: %d\n", ret);
				return ret;
			}
			touch_cdev->is_fps_registered = true;
			dev_info(DEV_TS, "Register fps_notifier OK\n");
		} else
			dev_err(DEV_TS, "no FPS_register_notifier exported.\n");
	} else if (touch_cdev->is_fps_registered) {
		unregister_link = symbol_get(FPS_unregister_notifier);
		if (unregister_link)
			unregister_link(&touch_cdev->fps_notif, 0xBEEF);
		else
			dev_err(DEV_TS, "no FPS_unregister_notifier exported.\n");
		touch_cdev->is_fps_registered = false;
	}

	return 0;
}

int ts_mmi_notifiers_register(struct ts_mmi_dev *touch_cdev)
{
	int ret = 0;

	if (!touch_cdev->class_dev) {
		return -ENODEV;
	}

	dev_info(DEV_TS, "%s: Start notifiers init.\n", __func__);

	INIT_DELAYED_WORK(&touch_cdev->work, ts_mmi_worker_func);
	ret = kfifo_alloc(&touch_cdev->cmd_pipe,
				sizeof(unsigned int)* 10, GFP_KERNEL);
	if (ret)
		goto FIFO_ALLOC_FAILED;

	if (touch_cdev->pdata.usb_detection) {
		touch_cdev->ps_notif.notifier_call = ts_mmi_charger_cb;
		ret = power_supply_reg_notifier(&touch_cdev->ps_notif);
		if (ret)
			goto PS_NOTIF_REGISTER_FAILED;
	}

	/*
	Because the touch pm_mode default is TS_MMI_PM_DEEPSLEEP, when the first suspend occurs,
	the interrupt will not be turned off, which results in I2C error.
	Need to set the initial pm_mode of the touch to PM_ACTIVE.
	*/
	touch_cdev->pm_mode = TS_MMI_PM_ACTIVE;

	touch_cdev->panel_nb.notifier_call = ts_mmi_panel_cb;
	ret = register_panel_notifier(&touch_cdev->panel_nb);
	if (ret)
		goto PANEL_NOTIF_REGISTER_FAILED;

	if (touch_cdev->pdata.update_refresh_rate) {
		touch_cdev->freq_nb.notifier_call = ts_mmi_refresh_rate_cb;
		ret = register_dynamic_refresh_rate_notifier(&touch_cdev->freq_nb);
		if (ret)
			goto FREQ_NOTIF_REGISTER_FAILED;
	}

	if (touch_cdev->pdata.fps_detection) {
		ret = ts_mmi_fps_notifier_register(touch_cdev, true);
		if (ret < 0)
			dev_err(DEV_TS,
				"Failed to register fps_notifier: %d\n", ret);
	}

	dev_info(DEV_TS, "%s: Notifiers init OK.\n", __func__);
	return 0;

FREQ_NOTIF_REGISTER_FAILED:
	unregister_panel_notifier(&touch_cdev->panel_nb);
PANEL_NOTIF_REGISTER_FAILED:
	cancel_delayed_work(&touch_cdev->work);
PS_NOTIF_REGISTER_FAILED:
	kfifo_free(&touch_cdev->cmd_pipe);
FIFO_ALLOC_FAILED:
	return ret;
}

void ts_mmi_notifiers_unregister(struct ts_mmi_dev *touch_cdev)
{
	if (touch_cdev->class_dev == NULL) {
		dev_err(DEV_MMI, "%s:touch_cdev->class_dev == NULL", __func__);
		return;
	}

	if (touch_cdev->pdata.fps_detection)
		ts_mmi_fps_notifier_register(touch_cdev, false);

	if (touch_cdev->pdata.update_refresh_rate)
		unregister_dynamic_refresh_rate_notifier(&touch_cdev->freq_nb);

	if (touch_cdev->pdata.usb_detection)
		power_supply_unreg_notifier(&touch_cdev->ps_notif);

	unregister_panel_notifier(&touch_cdev->panel_nb);
	cancel_delayed_work(&touch_cdev->work);
	kfifo_free(&touch_cdev->cmd_pipe);
	dev_info(DEV_MMI, "%s:notifiers_unregister finish", __func__);
}
