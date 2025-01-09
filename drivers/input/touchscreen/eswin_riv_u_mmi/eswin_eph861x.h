

#ifndef __LINUX_PLATFORM_DATA_ESWIN_EPH_H
#define __LINUX_PLATFORM_DATA_ESWIN_EPH_H

int eph_update_fw(struct device *dev, const char* name);
int eph_fod_mode_enable(struct device *dev, bool enable);
int eph_deepsleep_enable(struct device *dev, int enable);
int eph_gesture_mode_enable(struct device *dev, u8 gesture_mode);
int eph_screen_on_reporting(struct device *dev, int enable);
#if defined EPH_ESD_RECOVERY
void heartbeat_work_start(struct eph_data *ephdata);
void heartbeat_work_stop(struct eph_data *ephdata);
#endif
#endif /* __LINUX_PLATFORM_DATA_ESWIN_EPH_ */
