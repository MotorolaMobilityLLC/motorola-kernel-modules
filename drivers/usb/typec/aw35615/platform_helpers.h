/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __AW_PLATFORM_HELPERS_H_
#define __AW_PLATFORM_HELPERS_H_

#include "aw35615_global.h"

#define RETRIES_I2C		 (3) /* Number of retries for I2C reads/writes */

#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)
#define AW35615_REG_MAX		(0x44)

typedef enum {
	SET_VOUT_0000MV = 0,      // 0V
	SET_VOUT_5000MV = 5000,   // 5V
	SET_VOUT_9000MV = 9000,   // 9V
	SET_VOUT_12000MV = 12000, // 12V
	SET_VOUT_15000MV = 15000, // 15V
	SET_VOUT_20000MV = 20000, // 20V
} SET_VOUT_MV;

/**
 * Events that platform uses to notify modules listening to the events.
 * The subscriber to event signal can subscribe to individual events or
 * a event in group.
 */
typedef enum {
	CC1_ORIENT			= 0x1,
	CC2_ORIENT			= 0x2,
	CC_AUDIO_ORIENT		= 0x3,
	CC_NO_ORIENT		= 0x4,
	CC_AUDIO_OPEN		= 0x5,
	CC_ORIENT_ALL		= CC1_ORIENT | CC2_ORIENT | CC_NO_ORIENT,
	PD_NEW_CONTRACT		= 0x8,
	PD_NO_CONTRACT		= 0x9,
	PD_CONTRACT_ALL		= PD_NEW_CONTRACT | PD_NO_CONTRACT,
	PD_STATE_CHANGED	= 0xa,
	ACC_UNSUPPORTED		= 0xb,
	BIST_DISABLED		= 0xc,
	BIST_ENABLED		= 0xd,
	BIST_ALL			= BIST_ENABLED | BIST_DISABLED,
	ALERT_EVENT			= 0x10,
	POWER_ROLE			= 0x11,
	DATA_ROLE			= 0x12,
	AUDIO_ACC			= 0x13,
	CUSTOM_SRC			= 0x14,
	VBUS_OK				= 0x15,
	VBUS_DIS			= 0x16,
	WATERPROOFING		= 0x17,
	CC1_AND_CC2			= 0x18,
	EVENT_ALL			= 0xFF,
} Events_t;

/*****************************************************************************/
/*****************************************************************************/
/************************		GPIO Interface		 **********************/
/*****************************************************************************/
/*****************************************************************************/
/*******************************************************************************
 * Function:		aw_parse_dts
 * Input:		   none
 * Return:		  0 on success, error code otherwise
 * Description:	 Initializes the platform's dtsi
 *****************************************************************************/
AW_S32 aw_parse_dts(void);

/*******************************************************************************
 * Function:		aw_GPIO_Get_VBus5v/Other/IntN
 * Input:		   none
 * Return:		  true if set, false if clear
 * Description:	 Returns the value of the GPIO pin
 *****************************************************************************/

AW_BOOL aw_GPIO_Get_IntN(void);


/*******************************************************************************
 * Function:		aw_GPIO_Cleanup
 * Input:		   none
 * Return:		  none
 * Description:	 Frees any GPIO resources we may have claimed (eg. INT_N, VBus5V)
 *****************************************************************************/
void aw_GPIO_Cleanup(void);
int DeviceWrite(Port_t *port, AW_U8 reg_addr, AW_U8 length, AW_U8 *data);
int DeviceRead(Port_t *port, AW_U8 reg_addr, AW_U8 length, AW_U8 *data);

/*****************************************************************************/
/*****************************************************************************/
/************************		Timer Interface		**********************/
/*****************************************************************************/
/*****************************************************************************/

/*******************************************************************************
 * Function:		aw_Delay10us
 * Input:		   delay10us: Time to delay, in 10us increments
 * Return:		  none
 * Description:	 Delays the given amount of time
 *****************************************************************************/
void aw_Delay10us(AW_U32 delay10us);

/**
 * @brief Set or return programmable supply (PPS) voltage and current limit.
 *
 * @param port ID for multiple port controls
 * @param mv Voltage in millivolts
 * @return None or Value in mv/ma.
 */
void platform_set_pps_voltage(AW_U8 port, AW_U32 mv);

/**
 * @brief The function gets the current VBUS level supplied by PPS supply
 *
 * If VBUS is not enabled by the PPS supply the return type is undefined.
 *
 * @param port ID for multiple port controls
 * @return VBUS level supplied by PPS in milivolt resolution
 */
AW_U16 platform_get_pps_voltage(AW_U8 port);

/**
 * @brief Set the maximum current that can be supplied by PPS source
 * @param port ID for multiple port controls
 * @param ma Current in milliamps
 * @return None
 */
void platform_set_pps_current(AW_U8 port, AW_U32 ma);

/**
 * @brief The current state of the device interrupt pin
 *
 * @param port ID for multiple port controls
 * @return AW_TRUE if interrupt condition present.  Note: pin is active low.
 */
AW_BOOL platform_get_device_irq_state(AW_U8 port);

/**
 * @brief Perform a blocking delay.
 *
 * @param delayCount - Number of 10us delays to wait
 * @return None
 */
void platform_delay_10us(AW_U32 delayCount);

#ifdef AW_HAVE_DP
/******************************************************************************
 * Function:    platform_dp_enable_pins
 * Input:       enable - If false put dp pins to safe state and config is
 *              don't care. When true configure the pins with valid config.
 *              config - 32-bit port partner config. Same as type in
 *              DisplayPortConfig_t in display_port_types.h.
 * Return:      AW_TRUE - pin config succeeded, AW_FALSE - pin config failed
 * Description: enable/disable display port pins. If enable is true, check
 *              the configuration bits[1:0] and the pin assignment
 *              bits[15:8] to decide the appropriate configuration.
 ******************************************************************************/
AW_BOOL platform_dp_enable_pins(AW_BOOL enable, AW_U32 config);

/******************************************************************************
 * Function:		platform_dp_status_update
 * Input:		status - 32-bit status value. Same as DisplayPortStatus_t
 *			in display_port_types.h
 * Return:		None
 * Description:	 Called when new status is available from port partner
 ******************************************************************************/
void platform_dp_status_update(AW_U32 status);
#endif

AW_U64 get_system_time_ms(void);

/*****************************************************************************/
/*****************************************************************************/
/************************		SysFS Interface		**********************/
/*****************************************************************************/
/*****************************************************************************/
/*******************************************************************************
 * Function:		aw_Sysfs_Init
 * Input:		   none
 * Return:		  none
 * Description:	 Initializes the sysfs objects (used for user-space access)
 *****************************************************************************/
void aw_Sysfs_Init(void);

/*****************************************************************************/
/*****************************************************************************/
/************************		Driver Helpers		 **********************/
/*****************************************************************************/
/*****************************************************************************/
/*******************************************************************************
 * Function:		aw_InitializeCore
 * Input:		   none
 * Return:		  none
 * Description:	 Calls core initialization functions
 *****************************************************************************/
void aw_InitializeCore(void);

/*******************************************************************************
 * Function:		aw_IsDeviceValid
 * Input:		   none
 * Return:		  true if device is valid, false otherwise
 * Description:	 Verifies device contents via I2C read
 *****************************************************************************/
AW_BOOL aw_IsDeviceValid(struct aw35615_chip *chip);

/*******************************************************************************
 * Function:		aw_InitChipData
 * Input:		   none
 * Return:		  none
 * Description:	 Initializes our driver chip struct data
 *****************************************************************************/
void aw_InitChipData(void);

/*****************************************************************************/
/*****************************************************************************/
/************************     Threading Helpers         **********************/
/*****************************************************************************/
/*****************************************************************************/

/*******************************************************************************
 * Function: aw_EnableInterrupts
 * Input: none
 * Return: 0 on success, error code on failure
 * Description:	 Initializaes and enables the INT_N interrupt.
 * NOTE: The interrupt may be triggered at any point once this is called.
 * NOTE: The Int_N GPIO must be intialized prior to using this function.
 *****************************************************************************/
AW_S32 aw_EnableInterrupts(void);
void aw35615_init_event_handler(void);

void stop_otg_vbus(void);
AW_BOOL aw_set_pdo(AW_U16 obj_num, AW_U16 obj_cur);
AW_BOOL aw_set_apdo(AW_U16 apdo_vol, AW_U16 apdo_cur);
void stop_usb_host(struct aw35615_chip *chip);
void start_usb_host(struct aw35615_chip *chip, bool ss);
void stop_usb_peripheral(struct aw35615_chip *chip);
void start_usb_peripheral(struct aw35615_chip *chip);
void handle_core_event(AW_U32 event, AW_U8 portId, void *usr_ctx, void *app_ctx);
#endif  // __AW_PLATFORM_HELPERS_H_
