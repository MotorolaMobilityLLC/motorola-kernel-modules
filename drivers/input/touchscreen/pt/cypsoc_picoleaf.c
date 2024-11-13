/*
  * COPYRIGHT(C) FCNT LIMITED 2021
  *
  * This program is free software; you can redistribute it and/or
  * modify it under the terms of the GNU General Public License
  * as published by the Free Software Foundation; version 2
  * of the License.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/

#include "cypsoc_picoleaf.h"

typedef enum {
	I2C_READ,
	I2C_WRITE
} i2c_rw;

typedef enum {
	GPIO_LOW,
	GPIO_HIGH
} gpio_lh;

/* Enum for debug reporting levels */
enum CYPSOC_DEBUG_LEVEL {
	CDL_QUIET   = 0,
	CDL_ERROR   = 1,
	CDL_WARN    = 2,
	CDL_INFO    = 3,
	CDL_DEBUG   = 4,
	CDL_MAX
};

struct cypsoc_picoleaf_data *cpd_global;

/*
 * Print out all debug prints that are less then or equal to set level.
 */
#define cyp_debug(dev, dlevel, format, arg...)	 \
	do { \
		struct cypsoc_picoleaf_data *cpd_tmp = dev_get_drvdata(dev);\
		if (cpd_tmp->debug_level >= dlevel) {\
			if (dlevel == CDL_ERROR)\
				dev_err(dev, "[%d] "format, dlevel, ##arg);\
			else\
				dev_info(dev, "[%d] "format, dlevel, ##arg);\
		} \
	} while (0)

#define CYPSOC_INITIAL_DEBUG_LEVEL CDL_INFO

////////////////////////////////////////////////////
///            For Hardware control              ///

/// cypsoc_picoleaf_force_power_on_hw() function ///
/// has been written in pt_core.c because touch  ///
/// dirver have to turn on Cypress PSoC in order ///
/// to use I2C connection.                       ///

int cypsoc_picoleaf_power_off_hw(struct cypsoc_picoleaf_data *cpd)
{
	int rc = 0;

	cyp_debug(cpd->dev, CDL_INFO,
			"%s: Turn off PSoC pwr: VDD, VREF\n",
			__func__);

	if (cpd->rst_gpio)
		gpio_set_value(cpd->rst_gpio, GPIO_HIGH);

	if(cpd->vdd_gpio){
		rc = gpio_request(cpd->vdd_gpio, NULL);
		if (rc < 0) {
			gpio_free(cpd->vdd_gpio);
			rc = gpio_request(cpd->vdd_gpio, NULL);
		}
		if (rc < 0) {
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: Failed requesting VDD GPIO %d\n",
					__func__,
					cpd->vdd_gpio);
		}
		rc = gpio_direction_output(cpd->vdd_gpio, GPIO_LOW);
		if (rc){
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: setcfg for VDD GPIO %d failed\n",
					__func__,
					cpd->vdd_gpio);
		}
		gpio_free(cpd->vdd_gpio);
	}

	// at the same time as VDD
	if(cpd->vref_gpio){
		rc = gpio_request(cpd->vref_gpio, NULL);
		if (rc < 0) {
			gpio_free(cpd->vref_gpio);
			rc = gpio_request(cpd->vref_gpio, NULL);
		}
		if (rc < 0) {
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: Failed requesting VDD GPIO %d\n",
					__func__,
					cpd->vref_gpio);
		}
		rc = gpio_direction_output(cpd->vref_gpio, GPIO_LOW);
		if (rc){
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: setcfg for VDD GPIO %d failed\n",
					__func__,
					cpd->vref_gpio);
		}
		gpio_free(cpd->vref_gpio);
	}
	return rc;
}

static void cypsoc_picoleaf_reset_hw(struct cypsoc_picoleaf_data *cpd)
{
	int rc;
	cyp_debug(cpd->dev, CDL_INFO,
			"%s: PSoC HW Reset\n",
			__func__);

	if (cpd->rst_gpio){
		rc = gpio_request(cpd->rst_gpio, NULL);

		if(rc<0){
			gpio_free(cpd->rst_gpio);
			rc = gpio_request(cpd->rst_gpio, NULL);
		}
		if(rc<0){
			gpio_free(cpd->rst_gpio);
			cyp_debug(cpd->dev, CDL_ERROR, "%s: reset GPIO request ERROR\n", __func__);
			return;
		}

		rc = gpio_direction_output(cpd->rst_gpio, GPIO_HIGH);
		if(rc) rc = gpio_direction_output(cpd->rst_gpio, GPIO_HIGH);
		if(rc){
			gpio_free(cpd->rst_gpio);
			cyp_debug(cpd->dev, CDL_ERROR, "%s: cannot set reset GPIO to high\n", __func__);
			return;
		}

		// 1ms //
		usleep_range(1000, 2000);

		rc = gpio_direction_output(cpd->rst_gpio, GPIO_LOW);
		if(rc) rc = gpio_direction_output(cpd->rst_gpio, GPIO_LOW);
		if(rc) cyp_debug(cpd->dev, CDL_ERROR, "%s: cannot set reset GPIO to low\n", __func__);

		gpio_free(cpd->rst_gpio);
	}
}

static void cypsoc_picoleaf_esd_err_hw(struct cypsoc_picoleaf_data *cpd)
{
	int rc;
	cyp_debug(cpd->dev, CDL_INFO,
			"%s: ESD retry is failed. res_gpio becomes LOW\n",
			__func__);

	if (cpd->rst_gpio){
		rc = gpio_request(cpd->rst_gpio, NULL);

		if(rc<0){
			gpio_free(cpd->rst_gpio);
			rc = gpio_request(cpd->rst_gpio, NULL);
		}
		if(rc<0){
			gpio_free(cpd->rst_gpio);
			cyp_debug(cpd->dev, CDL_ERROR, "%s: reset GPIO request ERROR\n", __func__);
			return;
		}

		rc = gpio_direction_output(cpd->rst_gpio, GPIO_HIGH);
		if(rc) rc = gpio_direction_output(cpd->rst_gpio, GPIO_HIGH);
		if(rc){
			gpio_free(cpd->rst_gpio);
			cyp_debug(cpd->dev, CDL_ERROR, "%s: cannot set reset GPIO to high\n", __func__);
			return;
		}
		gpio_free(cpd->rst_gpio);
	}
}

static int cypsoc_picoleaf_esd_retry(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client){
	int rc;
	int i;
	struct        i2c_msg msgs[2];
	const uint8_t msg_count = 2;
	uint8_t       reg_addr_arr[2];
	uint8_t       version;

	reg_addr_arr[0] = (uint8_t)(CYPSOC_PICOLEAF_REG_FIRMWARE_VER >> 8);   // reg Address H
	reg_addr_arr[1] = (uint8_t)(CYPSOC_PICOLEAF_REG_FIRMWARE_VER & 0xFF); // reg Address L

	msgs[0].addr  = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len   = 2;
	msgs[0].buf   = reg_addr_arr;

	msgs[1].addr  = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len   = 1;
	msgs[1].buf   = &version;

	for (i=0; i<CYPSOC_DRIVER_I2C_RETRY_MAX; i++){
		cypsoc_picoleaf_reset_hw(cpd);

		// 150ms //
		usleep_range(150000, 160000);

		rc = i2c_transfer(client->adapter, msgs, msg_count);
		if (rc == msg_count){
			if(version == 0xFFU){
				cyp_debug(cpd->dev, CDL_ERROR,
						"%s: PSoC firm had been broken (read PSoC version=%d)\n", __func__, version);
			}else{
				cyp_debug(cpd->dev, CDL_INFO,
						"%s: PSoC has recoverd (read PSoC version=%d)\n",
						__func__,
						version);
				return CYPSOC_PICOLEAF_RET_OK;
			}
		}
	}
	return CYPSOC_PICOLEAF_RET_NG;
}

static int cypsoc_picoleaf_i2c_access_hw(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client, struct i2c_msg *msgs, uint8_t msg_count)
{
	/// ESD retry takes a detour of this function.
	int rc;
	uint8_t status;

	if(unlikely(cpd->i2c_pull_up)){
		return CYPSOC_PICOLEAF_RET_NG_I2C_PULL_UP;
	}

	mutex_lock(&cpd->psoc_status_lock);
	status = cpd->psoc_status;
	mutex_unlock(&cpd->psoc_status_lock);

	switch (status)
	{
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
		break;
	case CYPSOC_PICOLEAF_STATUS_RAISE_ERROR:
	case CYPSOC_PICOLEAF_STATUS_BROKEN:
		return CYPSOC_PICOLEAF_RET_NG_POWER_OFF;
	case CYPSOC_PICOLEAF_STATUS_I2C_RETRYING:
		return CYPSOC_PICOLEAF_RET_NG_PROCESSING;
	default:
		break;
	}

	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc == msg_count){
		return CYPSOC_PICOLEAF_RET_OK;
	}else{
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: Failed I2C transfer rc=%d\n",
				__func__,
				rc);
		schedule_work(&cpd->i2c_retry_work);
		return CYPSOC_PICOLEAF_RET_NG;
	}
}

static int cypsoc_picoleaf_i2c_read(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client, uint16_t reg_addr, uint8_t arr_len, uint8_t *data)
{
	struct        i2c_msg msgs[2];
	const uint8_t msg_count = 2;
	uint8_t       reg_addr_arr[2];

	reg_addr_arr[0] = (uint8_t)(reg_addr >> 8);   // reg Address H
	reg_addr_arr[1] = (uint8_t)(reg_addr & 0xFF); // reg Address L

	msgs[0].addr  = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len   = 2;
	msgs[0].buf   = reg_addr_arr;

	msgs[1].addr  = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len   = arr_len;
	msgs[1].buf   = data;

	return cypsoc_picoleaf_i2c_access_hw(cpd, client, msgs, msg_count);
}

static int cypsoc_picoleaf_i2c_write(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client, uint16_t reg_addr, uint8_t arr_len, uint8_t *data)
{
	struct        i2c_msg msgs[1];
	const uint8_t msg_count = 1;
	int           pack_arr_size = 2 + arr_len;
	uint8_t       *reg_addr_arr;
	int           rc;

	reg_addr_arr = kzalloc(pack_arr_size*sizeof(uint8_t), GFP_KERNEL);
	if(!reg_addr_arr) return CYPSOC_PICOLEAF_RET_NG;

	reg_addr_arr[0] = (uint8_t)(reg_addr >> 8);   // reg Address H
	reg_addr_arr[1] = (uint8_t)(reg_addr & 0xFF); // reg Address L

	memcpy(&reg_addr_arr[2], data, arr_len);

	msgs[0].addr  = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len   = pack_arr_size;
	msgs[0].buf   = reg_addr_arr;

	rc = cypsoc_picoleaf_i2c_access_hw(cpd, client, msgs, msg_count);
	kfree(reg_addr_arr);
	return rc;
}

/////////////////////////////////////////////////////
////        Cypress PSoC driver functions        ////

static int cypsoc_picoleaf_read_press_register(struct cypsoc_picoleaf_data *cpd, int *press)
{
	struct i2c_client *client = to_i2c_client(cpd->dev);
	int     rc;
	uint8_t press_hl[2];

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_INTEGRATED_PRESS_L, 2, press_hl);
    if(rc){
		goto read_press_error;
	}
	*press = (press_hl[1] << 8) + press_hl[0];

	return rc;

read_press_error:
	cyp_debug(cpd->dev, CDL_ERROR,
			"%s: press read ERROR (rc = %d\n",
			__func__, rc);
	*press = CYPSOC_NOTIFY_FW_TO_RK_OFF;
	return rc;
}

static int _cypsoc_picoleaf_get_press_z(struct cypsoc_picoleaf_data *cpd, int *press)
{
	int status;

	mutex_lock(&cpd->psoc_status_lock);
	status = cpd->psoc_status;
	mutex_unlock(&cpd->psoc_status_lock);

	switch (status)
	{
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
		break;
	case CYPSOC_PICOLEAF_STATUS_RAISE_ERROR:
		*press = CYPSOC_NOTIFY_FW_TO_RK_OFF;
		return CYPSOC_PICOLEAF_RET_NG_POWER_OFF;
	default:
		break;
	}

	return cypsoc_picoleaf_read_press_register(cpd, press);
}

static int cypsoc_picoleaf_get_status(struct cypsoc_picoleaf_data *cpd, char *buf)
{
	int ret;
	uint8_t status;

	mutex_lock(&cpd->psoc_status_lock);
	status = cpd->psoc_status;
	mutex_unlock(&cpd->psoc_status_lock);

	switch (status)
	{
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"%s\n",
			CYPSOC_ON);
		break;
	case CYPSOC_PICOLEAF_STATUS_BROKEN:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"broken\n");
		break;
	case CYPSOC_PICOLEAF_STATUS_INITIALIZING:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"initializing\n");
		break;
	case CYPSOC_PICOLEAF_STATUS_FW_UPDATING:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"firmware updating\n");
		break;
	case CYPSOC_PICOLEAF_STATUS_I2C_RETRYING:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"i2c retrying\n");
		break;
	case CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"%s\n",
			CYPSOC_SLEEP);
		break;
	case CYPSOC_PICOLEAF_STATUS_RAISE_ERROR:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"%s\n",
			"raise error");
		break;
	case CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"%s%s\n",
			CYPSOC_ON, "[testmode]");
		break;
	default:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"INVALID state!!\n");
		break;
	}

	return ret;
}

static int cypsoc_picoleaf_set_deep_sleep_mode_enable(struct cypsoc_picoleaf_data *cpd){
	struct i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t values[1] = {CYPSOC_PICOLEAF_DEEP_SLEEP_ENABLE};

	cyp_debug(cpd->dev, CDL_INFO,
			"%s: set deep sleep mode enable\n",
			__func__);

	return cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_RESET, 1, values);
}

static int cypsoc_picoleaf_set_deep_sleep_mode_disable(struct cypsoc_picoleaf_data *cpd){
	struct i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t values[1] = {CYPSOC_PICOLEAF_DEEP_SLEEP_DISABLE};

	cyp_debug(cpd->dev, CDL_INFO,
			"%s: set disable deep sleep mode\n",
			__func__);

	return cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_RESET, 1, values);
}

static int cypsoc_picoleaf_clear_intagration(struct cypsoc_picoleaf_data *cpd){
	struct  i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t values[1] = {CYPSOC_PICOLEAF_CLEAR_INTEGRA};

	cyp_debug(cpd->dev, CDL_INFO,
			"%s: set register to clear intagration\n",
			__func__);

	return cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_CLEAR_INTEGRA, 1, values);
}

static int cypsoc_picoleaf_clear_offset_intagration(struct cypsoc_picoleaf_data *cpd){
	struct  i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t values[1] = {CYPSOC_PICOLEAF_CLEAR_OFFSET_INTEGRA};

	cyp_debug(cpd->dev, CDL_INFO,
			"%s: set register to clear offet intagration\n",
			__func__);

	return cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_CLEAR_INTEGRA, 1, values);
}

static void cypsoc_picoleaf_rakuraku_on(struct cypsoc_picoleaf_data *cpd)
{
	int status;

	mutex_lock(&cpd->prev_status_lock);
	cpd->prev_status = CYPSOC_PICOLEAF_STATUS_AVAILABLE; // for FW updating
	mutex_unlock(&cpd->prev_status_lock);

	mutex_lock(&cpd->psoc_status_lock);
	status = cpd->psoc_status;
	mutex_unlock(&cpd->psoc_status_lock);

	switch (status)
	{
	case CYPSOC_PICOLEAF_STATUS_FW_UPDATING:
		cyp_debug(cpd->dev, CDL_INFO,
					"%s: rakuraku touch will be ON later (FW updating...)\n", __func__);
		break;
	case CYPSOC_PICOLEAF_STATUS_BROKEN:
		mutex_lock(&cpd->psoc_status_lock);
		cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_RAISE_ERROR;
		mutex_unlock(&cpd->psoc_status_lock);
		break;
	case CYPSOC_PICOLEAF_STATUS_INITIALIZING:
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
	case CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP:
		if (cypsoc_picoleaf_set_deep_sleep_mode_disable(cpd)){
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: Failed to set deep sleep\n", __func__);
		}
		else{
			mutex_lock(&cpd->psoc_status_lock);
			cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_AVAILABLE;
			mutex_unlock(&cpd->psoc_status_lock);
			cyp_debug(cpd->dev, CDL_INFO, "psoc status is changed to Available.\n");
		}
		break;
	default:
		break;
	}
}

static void cypsoc_picoleaf_rakuraku_off(struct cypsoc_picoleaf_data *cpd)
{
	uint8_t status;

	mutex_lock(&cpd->prev_status_lock);
	cpd->prev_status = CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP;// for FW updating
	mutex_unlock(&cpd->prev_status_lock);

	mutex_lock(&cpd->psoc_status_lock);
	status = cpd->psoc_status;
	mutex_unlock(&cpd->psoc_status_lock);

	switch (status)
	{
	case CYPSOC_PICOLEAF_STATUS_FW_UPDATING:
		cyp_debug(cpd->dev, CDL_INFO,
					"%s: rakuraku touch will be OFF later (FW updating...)\n", __func__);
		break;
	case CYPSOC_PICOLEAF_STATUS_RAISE_ERROR:
		mutex_lock(&cpd->psoc_status_lock);
		cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_BROKEN;
		mutex_unlock(&cpd->psoc_status_lock);
		break;
	case CYPSOC_PICOLEAF_STATUS_INITIALIZING:
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
	case CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP:
		if (cypsoc_picoleaf_set_deep_sleep_mode_enable(cpd)){
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: Failed to exit deep sleep\n", __func__);
		}
		else{
			mutex_lock(&cpd->psoc_status_lock);
			cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP;
			mutex_unlock(&cpd->psoc_status_lock);
			cyp_debug(cpd->dev, CDL_INFO, "psoc status is changed to Sleep.\n");
		}
		break;
	default:
		break;
	}
}

static int cypsoc_picoleaf_state_recover(struct cypsoc_picoleaf_data *cpd)
{
	int rc = CYPSOC_PICOLEAF_RET_OK;
	int pstatus;

	cyp_debug(cpd->dev, CDL_WARN, "%s: psoc status is revocerd.\n",
			__func__);

	mutex_lock(&cpd->prev_status_lock);
	mutex_lock(&cpd->psoc_status_lock);
	pstatus = cpd->prev_status;
	if (cpd->psoc_status == CYPSOC_PICOLEAF_STATUS_BROKEN){
		if (pstatus == CYPSOC_PICOLEAF_STATUS_AVAILABLE){
			pstatus = CYPSOC_PICOLEAF_STATUS_RAISE_ERROR;
		}
		else{
			pstatus = CYPSOC_PICOLEAF_STATUS_BROKEN;
		}
	}
	cpd->psoc_status = pstatus;
	mutex_unlock(&cpd->psoc_status_lock);
	mutex_unlock(&cpd->prev_status_lock);

	switch (pstatus)
	{
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
		cyp_debug(cpd->dev, CDL_WARN, "%s: psoc status will be on.\n",
				__func__);
		rc = cypsoc_picoleaf_set_deep_sleep_mode_disable(cpd);
		break;
	case CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP:
	case CYPSOC_PICOLEAF_STATUS_INITIALIZING: // default OFF
		cyp_debug(cpd->dev, CDL_WARN, "%s: psoc status will be off.\n",
				__func__);
		rc = cypsoc_picoleaf_set_deep_sleep_mode_enable(cpd);
	default:
		break;
	}
	return rc;
}

static void cypsoc_picoleaf_i2c_esd_retry_work(struct cypsoc_picoleaf_data *cpd)
{
	struct i2c_client *client;
	uint8_t hrest_release = CYPSOC_PICOLEAF_HRST_RELEASE;
	int rc;

	client = to_i2c_client(cpd->dev);

	cyp_debug(cpd->dev, CDL_WARN,
			"%s: STARTs to hardware reset work.\n",
			__func__);

	mutex_lock(&cpd->psoc_status_lock);
	mutex_lock(&cpd->prev_status_lock);
	switch (cpd->psoc_status)
	{
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
	case CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP:
		cpd->prev_status = cpd->psoc_status;
		break;
	default:
		break;
	}
	cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_I2C_RETRYING;
	mutex_unlock(&cpd->prev_status_lock);
	mutex_unlock(&cpd->psoc_status_lock);

	rc = cypsoc_picoleaf_esd_retry(cpd, client);

	if(rc){
		cyp_debug(cpd->dev, CDL_WARN,
				"%s: ESD retry is failed.\n",
				__func__);
		mutex_lock(&cpd->psoc_status_lock);
		cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_BROKEN;
		mutex_unlock(&cpd->psoc_status_lock);
		(void) cypsoc_picoleaf_esd_err_hw(cpd);
	}else{
		cyp_debug(cpd->dev, CDL_WARN,
				"%s: ESD retry is success. state is recovered.\n",
				__func__);
	}
	(void)cypsoc_picoleaf_state_recover(cpd);
	if(!rc) cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_HREST_STATUS, 1, &hrest_release);

	cyp_debug(cpd->dev, CDL_WARN,
			"%s: ENDs to hardware reset work.\n",
			__func__);
}

/////  The functions for Firmware Update   /////
static int cypsoc_picoleaf_init_i2c_cmds(struct cypsoc_picoleaf_i2c_cmds *cmds)
{
	const int ext_len = 3;

	const uint8_t bootload[] = {
			7U,  // command bytes
			15U, // size of receive buf
			20U, // waiting time (mili) for receive cmd
			0x01U, 0x38U, 0x00U, 0x00U,
			0xC7U, 0xFFU, 0x17U};

	const uint8_t erase_row[] = {
			10U, // command bytes
			7U,  // size of receive buf
			20U, // waiting time (mili) for receive cmd
			0x01U, 0x34U, 0x03U, 0x00U,
			0x00U, 0x00U, 0x00U, 0x00U,
			0x00U, 0x17U};

	const uint8_t send_data[] = {
			2U, // command bytes
			7U, // size of receive buf
			5U, // waiting time (mili) for receive cmd
			0x01U, 0x37U};

	const uint8_t prog_row[] = {
			2U, // command bytes
			7U, // size of receive buf
			50U,// waiting time (mili) for receive cmd
			0x01U, 0x39U};

	const uint8_t verify_row[] = {
			10U, // command bytes
			8U,  // size of receive buf
			20U, // waiting time (mili) for receive cmd
			0x01U, 0x3AU, 0x03U, 0x00U,
			0x00U, 0x00U, 0x00U, 0x00U,
			0x00U, 0x17U};

	const uint8_t verify_chksum[] = {
			7U,  // command bytes
			8U,  // size of receive buf
			20U, // waiting time (mili) for receive cmd
			0x01U, 0x31U, 0x00U, 0x00U,
			0xCEU, 0xFFU, 0x17U};

	const uint8_t exit[] = {
			7U,   // command bytes
			0U,   // size of receive buf
			100U, // waiting time (mili) for receive cmd
			0x01U, 0x3BU, 0x00U, 0x00U,
			0xC4U, 0xFFU, 0x17U};

	cmds->bootload_cmd      = kzalloc((bootload[0]      + ext_len)*sizeof(uint8_t), GFP_KERNEL);
	cmds->erase_row_cmd     = kzalloc((erase_row[0]     + ext_len)*sizeof(uint8_t), GFP_KERNEL);
	cmds->send_data_cmd     = kzalloc((send_data[0]     + ext_len)*sizeof(uint8_t), GFP_KERNEL);
	cmds->prog_row_cmd      = kzalloc((prog_row[0]      + ext_len)*sizeof(uint8_t), GFP_KERNEL);
	cmds->verify_row_cmd    = kzalloc((verify_row[0]    + ext_len)*sizeof(uint8_t), GFP_KERNEL);
	cmds->verify_chksum_cmd = kzalloc((verify_chksum[0] + ext_len)*sizeof(uint8_t), GFP_KERNEL);
	cmds->exit_cmd          = kzalloc((exit[0]          + ext_len)*sizeof(uint8_t), GFP_KERNEL);

	if (!cmds->bootload_cmd)      goto error;
	if (!cmds->erase_row_cmd)     goto error;
	if (!cmds->send_data_cmd)     goto error;
	if (!cmds->prog_row_cmd)      goto error;
	if (!cmds->verify_row_cmd)    goto error;
	if (!cmds->verify_chksum_cmd) goto error;
	if (!cmds->exit_cmd)          goto error;

	memcpy(cmds->bootload_cmd,      bootload,      bootload[0] + ext_len);
	memcpy(cmds->erase_row_cmd,     erase_row,     erase_row[0] + ext_len);
	memcpy(cmds->send_data_cmd,     send_data,     send_data[0] + ext_len);
	memcpy(cmds->prog_row_cmd,      prog_row,      prog_row[0] + ext_len);
	memcpy(cmds->verify_row_cmd,    verify_row,    verify_row[0] + ext_len);
	memcpy(cmds->verify_chksum_cmd, verify_chksum, verify_chksum[0] + ext_len);
	memcpy(cmds->exit_cmd,          exit,          exit[0] + ext_len);
	return 0;
error:
	pr_err("%s: Failed to allocate region of i2c cmds!\n",
			__func__);
	return -1;
}

static void cypsoc_picoleaf_release_i2c_cmds(struct cypsoc_picoleaf_i2c_cmds *cmds)
{
	if(cmds->bootload_cmd)      kfree(cmds->bootload_cmd);
	if(cmds->erase_row_cmd)     kfree(cmds->erase_row_cmd);
	if(cmds->send_data_cmd)     kfree(cmds->send_data_cmd);
	if(cmds->prog_row_cmd)      kfree(cmds->prog_row_cmd);
	if(cmds->verify_row_cmd)    kfree(cmds->verify_row_cmd);
	if(cmds->verify_chksum_cmd) kfree(cmds->verify_chksum_cmd);
	if(cmds->exit_cmd)          kfree(cmds->exit_cmd);
}

static void cypsoc_picoleaf_hexarr_to_string(char *buf, int buf_size, int cmd_len, uint8_t *hexarr)
{
	int i;
	for (i=0; i<(buf_size/2); i++){
		if (cmd_len == i) break;
		buf[i*2] = '0' + ((hexarr[i] >> 4) & 0x0FU) + (((hexarr[i] >> 4) & 0x0FU) < 10 ? 0 : 7);

		if ( (i+1)*2 >= buf_size) break;
		buf[i*2+1] = '0' + (hexarr[i] & 0x0FU) + ((hexarr[i] & 0x0FU) < 10 ? 0 : 7);
	}
	i = buf_size > i*2 ? i*2 : (buf_size - 1);
	buf[i] = '\0';
}

static int cypsoc_picoleaf_i2c_cmd_send(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client, uint8_t *cmd)
{
	struct i2c_msg msgs[1];
	const  uint8_t  msg_count = 1;
	char   view_buf[100];
	const  uint8_t view_buf_len = 100;
	const int ext_len = 3;

	msgs[0].addr  = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len   = cmd[0];
	msgs[0].buf   = &cmd[ext_len];

	cypsoc_picoleaf_hexarr_to_string(view_buf, view_buf_len, cmd[0], &cmd[ext_len]);
	cyp_debug(cpd->dev, CDL_DEBUG,
			"sends command to PSoC (cmd: %s)\n",
			view_buf);

	return cypsoc_picoleaf_i2c_access_hw(cpd, client, msgs, msg_count);
}

static int cypsoc_picoleaf_i2c_cmd_receive(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client, uint8_t receive_size, uint8_t *receive_buf)
{
	struct i2c_msg msgs[1];
	const uint8_t  msg_count = 1;
	char view_buf[100];
	const uint8_t view_buf_len = 100;
	int rc;

	msgs[0].addr  = client->addr;
	msgs[0].flags = I2C_M_RD;
	msgs[0].len   = receive_size;
	msgs[0].buf   = receive_buf;

	rc = cypsoc_picoleaf_i2c_access_hw(cpd, client, msgs, msg_count);

	if(rc){
		cyp_debug(cpd->dev, CDL_WARN,
				"%s: access error\n",
				__func__);
	}else{
		cypsoc_picoleaf_hexarr_to_string(view_buf, view_buf_len, receive_size, receive_buf);
		cyp_debug(cpd->dev, CDL_DEBUG,
				"received command from PSoC (cmd: %s)\n",
				view_buf);
	}
	return rc;
}

static int cypsoc_picoleaf_i2c_cmd_sr(struct cypsoc_picoleaf_data *cpd,
		struct i2c_client *client, uint8_t *send_cmd, uint8_t *receive_buf)
{
	int rc;
	int receive_size = send_cmd[1];
	int wait_receive = send_cmd[2];
	int i;
	int retry = 5;

	rc = cypsoc_picoleaf_i2c_cmd_send(cpd, client, send_cmd);
	if(rc){
		cyp_debug(cpd->dev, CDL_WARN,
				"%s: I2C command send error! (host --> PSoC)\n",
				__func__);
		return rc;
	}

	if(receive_size == 0){
		return rc;
	}

	for (i=0; i<retry; i++){
		usleep_range(1000*wait_receive, 1000*(wait_receive+1));
		rc = cypsoc_picoleaf_i2c_cmd_receive(cpd, client, receive_size, receive_buf);
		if(rc){
			cyp_debug(cpd->dev, CDL_WARN,
					"%s: I2C command receive error! (host <-- PSoC)\n",
					__func__);
			continue;
		}
		if(receive_size > 1){
			if(receive_buf[1] == 0xFFU){
				continue;
			}
			if(receive_buf[1] != 0x0U){
				cyp_debug(cpd->dev, CDL_ERROR,
						"%s: Received command send err=%d\n",
						__func__,
						receive_buf[1]);
				return CYPSOC_PICOLEAF_RET_NG;
			}
		}
		break;
	}
	return rc;
}

static int cypsoc_picoleaf_readline(uint8_t **buf, int *remain, char *line, int max_line_len)
{
	// This func removes '\n' at the end of line.
	char c;
	int  i;
	int  i_max = *remain;

	for(i=0; i<i_max; i++){
		if(i == max_line_len - 1) {
			*buf += i;
			*remain -= i;
			line[i] = '\0';
			return -1;
		}

		c = buf[0][i];

		switch (c)
		{
		case '\n':
			*buf += (i + 1);
			*remain -= (i + 1);
			line[i] = '\0';
			return i + 1;
		case '\0':
			*buf += i;
			*remain -= i;
			line[i] = '\0';
			return 0;
		default:
			line[i] = c;
			break;
		}
	}
	*buf += i;
	*remain -= i;
	line[i] = '\0';
	return 0;
}

static char cypsoc_picoleaf_from_hex(char c)
{
	if ('0' <= c && c <= '9')
		return c - '0';
	else if ('a' <= c && c <= 'f')
		return 10 + (c - 'a');
	else if ('A' <= c && c <= 'F')
		return 10 + (c - 'A');
	else
		return 0;
}

static int cypsoc_picoleaf_parse_fw_version(int* version,
		uint8_t *file_header_buf)
{
	int fw_ver = 0;

	if (strlen(file_header_buf) < 4){
		pr_err("size err first line size=%lu (needs >= 3)\n",
				strlen(file_header_buf));
		return CYPSOC_PICOLEAF_RET_NG;
	}

	fw_ver =
			cypsoc_picoleaf_from_hex(file_header_buf[1]) << 4 |
			cypsoc_picoleaf_from_hex(file_header_buf[2]);

	*version = fw_ver;
	return CYPSOC_PICOLEAF_RET_OK;
}

static int  cypsoc_picoleaf_silicon_id_check(uint8_t *dev_receive_buf,
		uint8_t *file_header_buf)
{
	unsigned long dev_silicon_id;
	uint8_t  dev_rev_id;
	unsigned long dev_bl_ver;

	unsigned long file_silicon_id;
	int8_t   file_rev_id;

	// needs length of dev_receive_buf >= 12
	if (strlen(file_header_buf) < 10){
		pr_err("size err file_header size=%lu (needs >= 10)\n",
				strlen(file_header_buf));
		return -1;
	}

	dev_bl_ver =
			dev_receive_buf[11] << 16 |
			dev_receive_buf[10] <<  8 |
			dev_receive_buf[9]  <<  0;
	dev_rev_id =
			dev_receive_buf[8] <<  0;
	dev_silicon_id =
			dev_receive_buf[7] << 24 |
			dev_receive_buf[6] << 16 |
			dev_receive_buf[5] <<  8 |
			dev_receive_buf[4] <<  0;

	printk("dev bl_ver=%08lX dev rev_id=%02X dev silicon_id=%08lX\n",
			dev_bl_ver,
			dev_rev_id,
			dev_silicon_id);

	file_silicon_id =
			cypsoc_picoleaf_from_hex(file_header_buf[0]) << 28 |
			cypsoc_picoleaf_from_hex(file_header_buf[1]) << 24 |
			cypsoc_picoleaf_from_hex(file_header_buf[2]) << 20 |
			cypsoc_picoleaf_from_hex(file_header_buf[3]) << 16 |
			cypsoc_picoleaf_from_hex(file_header_buf[4]) << 12 |
			cypsoc_picoleaf_from_hex(file_header_buf[5]) <<  8 |
			cypsoc_picoleaf_from_hex(file_header_buf[6]) <<  4 |
			cypsoc_picoleaf_from_hex(file_header_buf[7]) <<  0;
	file_rev_id =
			cypsoc_picoleaf_from_hex(file_header_buf[8]) <<  4 |
			cypsoc_picoleaf_from_hex(file_header_buf[9]) <<  0;

	printk("file silicon_id=%08lX file rev_id=%02X\n",
			file_silicon_id,
			file_rev_id);

	if ((dev_silicon_id != file_silicon_id) ||
		(dev_rev_id != file_rev_id)) {
		pr_err("%s: Error: Silicon ID  or Rev mismatch\n",
				__func__);
		return -1;
	}

	return 0;
}

static uint16_t cypsoc_picoleaf_compute_chksum(uint8_t *buf, int buf_size)
{
	int i;
	uint16_t checksum = 0;

	if(buf_size<0){
		pr_err("%s: buf_size < 0!!\n",
				__func__);
	}

	for (i = 0; i < buf_size; i++)
		checksum += buf[i];
	checksum = 1 + ~checksum;

	return checksum;
}


static int cypsoc_picoleaf_parse_fw_row(char *line,	struct cypsoc_fw_parsed_row *parsed)
{
	int min_buf_size = 13;
	int line_size;
	uint16_t i, j;
	uint16_t checksum = 0;

	if (!line) {
		pr_err("parse row error - buf is null\n");
		return -1;
	}

	line_size = strlen(line);
	if (line_size < min_buf_size) {
		pr_err("parse row error - bufsize(%d) < min(%d)\n",
				line_size,
				min_buf_size);
		return -1;
	}

	if (line[0] == ':') {

		for (i = 1; i < (288*2)+10; i++){
			checksum += cypsoc_picoleaf_from_hex(line[i]);
		}
		checksum = ~checksum;

		parsed->array_id =
				cypsoc_picoleaf_from_hex(line[1]) <<  4 |
				cypsoc_picoleaf_from_hex(line[2]) <<  0;
		parsed->row_num =
				cypsoc_picoleaf_from_hex(line[3]) << 12 |
				cypsoc_picoleaf_from_hex(line[4]) <<  8 |
				cypsoc_picoleaf_from_hex(line[5]) <<  4 |
				cypsoc_picoleaf_from_hex(line[6]) <<  0;
		parsed->row_size =
				cypsoc_picoleaf_from_hex(line[7]) << 12 |
				cypsoc_picoleaf_from_hex(line[8]) <<  8 |
				cypsoc_picoleaf_from_hex(line[9]) <<  4 |
				cypsoc_picoleaf_from_hex(line[10]) <<  0;

		checksum = parsed->array_id + parsed->row_num + parsed->row_size;

		for (i = 0, j = 11; i < line_size; i++) {
			parsed->row_data[i] =
					cypsoc_picoleaf_from_hex(line[j+0]) << 4 |
					cypsoc_picoleaf_from_hex(line[j+1]) << 0;
			j += 2;
			checksum += parsed->row_data[i];
		}
		checksum = 1 + ~checksum;

		parsed->row_chksum =
				cypsoc_picoleaf_from_hex(line[j+0]) << 4 |
				cypsoc_picoleaf_from_hex(line[j+1]) << 0;
		return 0;
	} else {
		pr_err("%s: parse row error - buf[0]=%d\n",
				__func__,
				line[0]);
		return -1;
	}
}

static int cypsoc_picoleaf_erase_row(struct cypsoc_picoleaf_data *cpd,
		struct cypsoc_picoleaf_i2c_cmds *cmds, struct cypsoc_fw_parsed_row *parsed)
{
	struct i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t   receive_buf[64];
	uint16_t  checksum;
	const int ext_len = 3;
	uint8_t  *cmd  = kzalloc(cmds->erase_row_cmd[0] + ext_len, GFP_KERNEL);
	int rc;

	if(!cmd) return CYPSOC_PICOLEAF_RET_NG;

	memcpy(cmd, cmds->erase_row_cmd, cmds->erase_row_cmd[0] + ext_len);

	cmd[4+ext_len] = parsed->array_id;
	cmd[5+ext_len] = (uint8_t) parsed->row_num;
	cmd[6+ext_len] = (uint8_t)(parsed->row_num >> 8);

	checksum = cypsoc_picoleaf_compute_chksum(&cmd[ext_len], (int)cmd[0] - 3);

	cmd[7+ext_len] = (uint8_t) checksum;
	cmd[8+ext_len] = (uint8_t)(checksum >> 8);

	rc = cypsoc_picoleaf_i2c_cmd_sr(cpd, client, cmd, receive_buf);
	if(cmd) kfree(cmd);
	return rc;
}

static int cypsoc_picoleaf_prog_row(struct cypsoc_picoleaf_data *cpd,
	struct cypsoc_picoleaf_i2c_cmds *cmds, struct cypsoc_fw_parsed_row *parsed)
{
	struct i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t   receive_buf[64];
	const uint8_t eop = 0x17;
	uint16_t      data_row_size = 128;
	uint16_t      data_len = 32;
	uint16_t      max_xfer_size = 128;
	int           rc;
	uint16_t      checksum;
	uint16_t      i, j, k, l;
	uint16_t      row_sum = 0;
	int           ext_len = 3;

	uint8_t *cmd = kzalloc(max_xfer_size, GFP_KERNEL);

	if (cmd) {
		i = 0;
		rc = CYPSOC_PICOLEAF_RET_OK;
		row_sum = 0;
		for (l = 0; l < (data_row_size/data_len) - 1; l++) {
			j = ext_len;
			cmd[  j] = cmds->send_data_cmd[j];
			j++;
			cmd[  j] = cmds->send_data_cmd[j];
			cmd[++j] = (uint8_t)data_len;
			cmd[++j] = (uint8_t)(data_len >> 8);
			j++; // 7

			for (k = 0; k < data_len; k++) {
				cmd[j] = parsed->row_data[i];
				row_sum += cmd[j];
				i++;
				j++;
			}

			checksum = cypsoc_picoleaf_compute_chksum(cmd+ext_len, j-ext_len);
			cmd[j+0] = (uint8_t)checksum;
			cmd[j+1] = (uint8_t)(checksum >> 8);
			cmd[j+2] = eop;

			cmd[0] = (uint8_t)(j+2-ext_len+1); // command length
			cmd[1] = cmds->send_data_cmd[1];   // receive length
			if(cypsoc_picoleaf_i2c_cmd_sr(cpd, client, cmd, receive_buf)){
				rc = CYPSOC_PICOLEAF_RET_NG;
			}
		}

		j = ext_len;
		cmd[  j] = cmds->prog_row_cmd[j];
		j++;
		cmd[  j] = cmds->prog_row_cmd[j];
		/* include array id size and row id size in data_len */
		cmd[++j] = (uint8_t)(data_len + 3);
		cmd[++j] = (uint8_t)((data_len+3) >> 8);
		cmd[++j] = parsed->array_id;
		cmd[++j] = (uint8_t)parsed->row_num;
		cmd[++j] = (uint8_t)(parsed->row_num >> 8);
		j++; // 10

		for (k = 0; k < data_len; k++) {
			cmd[j] = parsed->row_data[i];
			row_sum += cmd[j];
			i++;
			j++;
		}

		checksum = cypsoc_picoleaf_compute_chksum(cmd+ext_len, j-ext_len);
		cmd[j+0] = (uint8_t)checksum;
		cmd[j+1] = (uint8_t)(checksum >> 8);
		cmd[j+2] = eop;

		cmd[0] = (uint8_t)(j+2-ext_len+1); // command length
		cmd[1] = cmds->prog_row_cmd[1];    // receive length
		if(cypsoc_picoleaf_i2c_cmd_sr(cpd, client, cmd, receive_buf)){
			rc = CYPSOC_PICOLEAF_RET_NG;
		}

		kfree(cmd);
		return rc;
	} else {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: prog row error - cmd buf is NULL\n",
				__func__);
		return CYPSOC_PICOLEAF_RET_NG;
	}
}

static int cypsoc_picoleaf_verify_row(struct cypsoc_picoleaf_data *cpd,
	struct cypsoc_picoleaf_i2c_cmds *cmds, struct cypsoc_fw_parsed_row *parsed)
{
	struct i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t   receive_buf[64];
	uint16_t  checksum;
	const int ext_len = 3;
	uint8_t  *cmd = kzalloc(cmds->verify_row_cmd[0] + ext_len, GFP_KERNEL);
	int rc;

	if (!cmd) return CYPSOC_PICOLEAF_RET_NG;

	memcpy(cmd, cmds->verify_row_cmd, cmds->verify_row_cmd[0] + ext_len);

	cmd[4+ext_len] = parsed->array_id;
	cmd[5+ext_len] = (uint8_t)parsed->row_num;
	cmd[6+ext_len] = (uint8_t)(parsed->row_num >> 8);

	checksum = cypsoc_picoleaf_compute_chksum(&cmd[ext_len], (int)cmd[0] - 3);

	cmd[7+ext_len] = (uint8_t)checksum;
	cmd[8+ext_len] = (uint8_t)(checksum >> 8);

	rc = cypsoc_picoleaf_i2c_cmd_sr(cpd, client, cmd, receive_buf);
	if(cmd) kfree(cmd);
	return rc;
}

static int cypsoc_picoleaf_enter_bootloader_mode(struct cypsoc_picoleaf_data *cpd)
{
	struct i2c_client *client = to_i2c_client(cpd->dev);
	uint8_t            value  = CYPSOC_PICOLEAF_BOOTLOADER_MODE;
	return cypsoc_picoleaf_i2c_write(cpd, client,
			CYPSOC_PICOLEAF_REG_ENTER_BOOTLOADER, 1, &value);
}

static int _cypsoc_picoleaf_firmware_update_cont(struct cypsoc_picoleaf_data *cpd,
		uint8_t force_update)
{
	struct i2c_client *client   = to_i2c_client(cpd->dev);
	uint8_t           *fw_img   = NULL;
	uint8_t           *cursor   = NULL;
	int                fw_size  = 0;
	char              *line_buf = NULL;
	int                line_buf_size = 1024;
	int                st = 0;
	int                fw_file_ver = -1;
	uint8_t            fw_micon_ver= -1;
	int                rc;
	int                tries = 0;
	const struct firmware *fw_entry = NULL;

	uint8_t   receive_buf[64];
	const int parsed_rowsize = 512;
	uint8_t   parsed_row[512];
	struct cypsoc_picoleaf_i2c_cmds *cmds = NULL;
	struct cypsoc_fw_parsed_row parsed = {
			.array_id   = 0,
			.row_num    = 0,
			.row_data   = parsed_row,
			.row_size   = 0,
			.row_chksum = 0};

	cyp_debug(cpd->dev, CDL_INFO, "%s: STARTs\n", __func__);

	rc = request_firmware(&fw_entry, CYPSOC_FW_FILE_NAME, cpd->dev);
	if (rc < 0 || !fw_entry) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: No firmware provided to load\n",
				__func__);
		rc = CYPSOC_PICOLEAF_RET_NG;
		goto exit;
	}

	fw_size = fw_entry->size;
	fw_img = (uint8_t*)&(fw_entry->data[0]);

	if (!fw_img || !fw_size) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"Invalid fw or file size=%d\n",
				(int)fw_size);
		return CYPSOC_PICOLEAF_RET_NG;
	}
	if (fw_img[0] >= (fw_size + 1)) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: Firmware format is invalid\n",
				__func__);
		return CYPSOC_PICOLEAF_RET_NG;
	}

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_FIRMWARE_VER, 1, &fw_micon_ver);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: Firm version of picoleaf micro controller read err (ret=%d)\n", __func__, rc);
		return CYPSOC_PICOLEAF_RET_NG_I2C_RETRY;
	}

	// allocation
	cmds = kzalloc(sizeof(*cmds), GFP_KERNEL);
	if (!cmds){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Failed to allocate cmds struct!\n");
		return CYPSOC_PICOLEAF_RET_NG;
	}
	rc = cypsoc_picoleaf_init_i2c_cmds(cmds);
	if (rc) {
		cypsoc_picoleaf_release_i2c_cmds(cmds);
		kfree(cmds);
		return CYPSOC_PICOLEAF_RET_NG;
	}
	line_buf = kzalloc(line_buf_size, GFP_KERNEL);
	if (!line_buf){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Failed to allocate line buf!\n");
		cypsoc_picoleaf_release_i2c_cmds(cmds);
		kfree(cmds);
		return CYPSOC_PICOLEAF_RET_NG;
	}

	cursor = fw_img;

	// Read in first line (version info) of CYACD file.
	st = cypsoc_picoleaf_readline(&cursor, &fw_size, line_buf, line_buf_size);
	if (st < 1){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Invaild fw file or failed to read fw file! (fw_size=%d)\n",
				fw_size);
		rc = CYPSOC_PICOLEAF_RET_NG;
		goto exit;
	}
	if (cypsoc_picoleaf_parse_fw_version(&fw_file_ver, line_buf)){
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: cannot read file version.\n",
				__func__);
		rc = CYPSOC_PICOLEAF_RET_NG;
		goto exit;
	}

	if(fw_micon_ver == 0xFF) force_update = 1;
	if(!force_update){
		if(fw_file_ver < 0 || fw_file_ver <= fw_micon_ver){
			cyp_debug(cpd->dev, CDL_ERROR,
					"PSoC already has latest FW. (file, micon)=(%d, %d)\n", fw_file_ver, fw_micon_ver);
			rc = CYPSOC_PICOLEAF_RET_OK;
			goto exit;
		}
	}

	// Read in second line (header info) of CYACD file.
	st = cypsoc_picoleaf_readline(&cursor, &fw_size, line_buf, line_buf_size);
	if (st < 1){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Invaild fw file or failed to read fw file! (fw_size=%d)\n",
				fw_size);
		rc = CYPSOC_PICOLEAF_RET_NG;
		goto exit;
	}

	cyp_debug(cpd->dev, CDL_INFO,
			"Firmware header : %s\n",
			line_buf);


	// Bootloader mode
	rc = cypsoc_picoleaf_enter_bootloader_mode(cpd);
	if (rc){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Failed to enter bootloader mode!\n");
		return CYPSOC_PICOLEAF_RET_NG_I2C_RETRY;
	}

	// 150ms //
	usleep_range(150000, 160000);

	// Enter Bootload command
	rc = cypsoc_picoleaf_i2c_cmd_sr(cpd, client, cmds->bootload_cmd, receive_buf);
	if (rc){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Failed to send/receive I2C command (Enter Bootload) to PSoC!\n");
		rc = CYPSOC_PICOLEAF_RET_NG_I2C_RETRY;
		goto exit;
	}

	rc = cypsoc_picoleaf_silicon_id_check(receive_buf, line_buf);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: firm header err (rc=%d)\n",
				__func__,
				rc);
		goto bl_exit;
	}

	// Skip Get flash size command

	while (fw_size > 0) {
		// parse row //
		memset(parsed_row, 0, parsed_rowsize);
		st = cypsoc_picoleaf_readline(&cursor, &fw_size, line_buf, line_buf_size);
		if(!st){
			break;
		}
		rc = cypsoc_picoleaf_parse_fw_row(line_buf, &parsed);
		if (rc) {
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: Parse Row Error (arr_id=%d row_num=%d chk=%02X ret=%d)\n",
					__func__,
					parsed.array_id,
					parsed.row_num,
					parsed.row_chksum,
					rc);
			goto bl_exit;
		}

		// erase row //
		tries = 0;
		do {
			rc = cypsoc_picoleaf_erase_row(cpd, cmds, &parsed);
			if (rc) {
				cyp_debug(cpd->dev, CDL_ERROR,
						"%s: Erase Row Error (array=%d row=%d ret=%d)\n",
						__func__,
						parsed.array_id,
						parsed.row_num,
						rc);
			}
		} while (rc && tries++ < 5);
		if (rc) goto bl_exit;

		tries = 0;
		do {
			rc = cypsoc_picoleaf_prog_row(cpd, cmds, &parsed);
			if (rc) {
				cyp_debug(cpd->dev, CDL_ERROR,
						"%s: Program Row Error (array=%d row=%d ret=%d)\n",
						__func__,
						parsed.array_id,
						parsed.row_num,
						rc);
			}
		} while (rc && tries++ < 5);
		if (rc) goto bl_exit;

	}
	// last line of fw file is ignored

	rc = cypsoc_picoleaf_verify_row(cpd, cmds, &parsed);
	if (rc){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Failed to send/receive I2C command (verify_row) to PSoC!\n");
		rc = CYPSOC_PICOLEAF_RET_NG_I2C_RETRY;
		goto exit;
	}

	// verify app checksum
	rc = cypsoc_picoleaf_i2c_cmd_sr(cpd, client, cmds->verify_chksum_cmd, receive_buf);
	if (rc){
		cyp_debug(cpd->dev, CDL_ERROR,
				"Failed to send/receive I2C command (verify_chksum) to PSoC!\n");
		rc = CYPSOC_PICOLEAF_RET_NG_I2C_RETRY;
		goto exit;
	}
	if(receive_buf[4] == 0){
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: Error: Checksum err\n",
				__func__);
		rc = CYPSOC_PICOLEAF_RET_NG_FW_FF;
		goto exit;
	}

	cyp_debug(cpd->dev, CDL_INFO, "%s: completed!\n", __func__);

	// exit loader //
bl_exit:
	rc = cypsoc_picoleaf_i2c_cmd_send(cpd, client, cmds->exit_cmd);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"Error on stop PSoC Loader (rc=%d)\n",
				rc);
		rc = CYPSOC_PICOLEAF_RET_NG_I2C_RETRY;

	}

	// 150ms //
	usleep_range(150000, 160000);

	// check firmware version //
	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_FIRMWARE_VER, 1, &fw_micon_ver);
	cyp_debug(cpd->dev, CDL_INFO, "%s: updates to ver=%d\n", __func__, fw_micon_ver);

	if(!rc && (fw_micon_ver == 0xFFU)){
		cyp_debug(cpd->dev, CDL_WARN, "%s:hw_reset (at FW update)\n", __func__);
		// hw reset
		cypsoc_picoleaf_reset_hw(cpd);

		// 150ms //
		usleep_range(150000, 160000);

		rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_FIRMWARE_VER, 1, &fw_micon_ver);
		if(!rc && (fw_micon_ver == 0xFFU)){
			rc = CYPSOC_PICOLEAF_RET_NG_FW_FF;
			cyp_debug(cpd->dev, CDL_ERROR, "%s: firmware update probably FAILED. (version == %d)\n",
					__func__, fw_micon_ver);

		}
		cyp_debug(cpd->dev, CDL_WARN, "%s: updates to ver=%d (HW reset was retried)\n", __func__, fw_micon_ver);
	}


exit:
	if (fw_entry) release_firmware(fw_entry);
	if (line_buf) kfree(line_buf);
	if (cmds) {
		cypsoc_picoleaf_release_i2c_cmds(cmds);
		kfree(cmds);
	}
	cyp_debug(cpd->dev, CDL_INFO,
			"%s: ENDs\n",
			__func__);
	return rc;
}

static int _cypsoc_picoleaf_firmware_update(struct cypsoc_picoleaf_data *cpd,
		uint8_t force_update)
{
	int rc = CYPSOC_PICOLEAF_RET_OK;
	int status;

	if (!force_update) {
		mutex_lock(&cpd->psoc_status_lock);
		status = cpd->psoc_status;
		mutex_unlock(&cpd->psoc_status_lock);
		switch (status)
		{
		case CYPSOC_PICOLEAF_STATUS_INITIALIZING:
		case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
		case CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP:
			break;
		default:
			rc = CYPSOC_PICOLEAF_RET_NG;
			break;
		}
	}

	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: skips PSoC firmware update!\n",
				__func__);
	}
	else {
		mutex_lock(&cpd->psoc_status_lock);
		mutex_lock(&cpd->prev_status_lock);
		cpd->prev_status = status;
		cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_FW_UPDATING;
		mutex_unlock(&cpd->prev_status_lock);
		mutex_unlock(&cpd->psoc_status_lock);

		rc = _cypsoc_picoleaf_firmware_update_cont(cpd, force_update);

		if (rc) {
			cyp_debug(cpd->dev, CDL_WARN,
					"%s: firmware update retrying (rc=%d)\n",
					__func__, rc);
			// 150ms //
			usleep_range(150000, 160000);

			mutex_lock(&cpd->psoc_status_lock);
			mutex_lock(&cpd->prev_status_lock);
			cpd->prev_status = status;
			cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_FW_UPDATING;
			mutex_unlock(&cpd->prev_status_lock);
			mutex_unlock(&cpd->psoc_status_lock);
			rc = _cypsoc_picoleaf_firmware_update_cont(cpd, force_update);
		}

		if (rc) {
			cyp_debug(cpd->dev, CDL_ERROR,
					"%s: Failed to PSoC firmware update! (rc=%d)\n",
					__func__,
					rc);
			if(rc == CYPSOC_PICOLEAF_RET_NG_FW_FF){
				mutex_lock(&cpd->psoc_status_lock);
				cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_BROKEN;
				mutex_unlock(&cpd->psoc_status_lock);
			}
		}else{
			cyp_debug(cpd->dev, CDL_INFO,
					"%s: Succeeded at PSoC firmware update\n",
					__func__);
		}

		mutex_lock(&cpd->psoc_status_lock);
		status = cpd->psoc_status;
		mutex_unlock(&cpd->psoc_status_lock);
		if (status == CYPSOC_PICOLEAF_STATUS_FW_UPDATING)
			(void)cypsoc_picoleaf_state_recover(cpd);
	}
	return rc;
}

//////////////////////////////////////////////
////        Schedule work handlers        ////

static void cypsoc_picoleaf_i2c_esd_retry_work_handler(struct work_struct *work)
{
	struct cypsoc_picoleaf_data *cpd;
	cpd = container_of(work, struct cypsoc_picoleaf_data, i2c_retry_work);
	cypsoc_picoleaf_i2c_esd_retry_work(cpd);
}

//////////////////////////////////////////////////
////         For sysfs SHOW, STORE            ////

static ssize_t cypsoc_picoleaf_sysfs_psoc_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	int rc=0;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return size;
	}

	if (!strncmp(buf, CYPSOC_ON, strlen(CYPSOC_ON))){
		if (cpd->psoc_status == CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP){
			rc = cypsoc_picoleaf_set_deep_sleep_mode_disable(cpd);
			if (rc){
				goto error;
			}
			mutex_lock(&cpd->psoc_status_lock);
			cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_AVAILABLE;
			mutex_unlock(&cpd->psoc_status_lock);
			rc = cypsoc_picoleaf_clear_offset_intagration(cpd);
			if (rc){
				goto error;
			}
		}else{
			cyp_debug(cpd->dev, CDL_INFO,
					"PSoC status is not SLEEP\n");
		}
	}
	else {
		if (cpd->psoc_status == CYPSOC_PICOLEAF_STATUS_AVAILABLE){
			rc = cypsoc_picoleaf_set_deep_sleep_mode_enable(cpd);
			if (rc){
				goto error;
			}
			mutex_lock(&cpd->psoc_status_lock);
			cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_DEEP_SLEEP;
			mutex_unlock(&cpd->psoc_status_lock);
		}else{
			cyp_debug(cpd->dev, CDL_INFO,
					"PSoC status is not ON\n");
		}
	}
	return size;

error:
	cyp_debug(cpd->dev, CDL_ERROR,
			"%s: FAILED: I2C access err:%d\n",
			__func__,
			rc);
	return size;
}

static ssize_t cypsoc_picoleaf_sysfs_psoc_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	ssize_t ret;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}

	ret = cypsoc_picoleaf_get_status(cpd, buf);
	return ret;
}
static DEVICE_ATTR(psoc_state, 0664,
		cypsoc_picoleaf_sysfs_psoc_state_show,
		cypsoc_picoleaf_sysfs_psoc_state_store);

static ssize_t cypsoc_picoleaf_sysfs_press_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	int value;
	int rc;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return size;
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = kstrtoint(buf, 10 , &value);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: Invalid press mode value\n",
				__func__);
		return size;
	}

	switch (value)
	{
	case 0:
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: rakuraku OFF\n", __func__);
		cypsoc_picoleaf_rakuraku_off(cpd);
		break;
	case 1:
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: rakuraku ON\n",
				__func__);
		cypsoc_picoleaf_rakuraku_on(cpd);
		break;
	default:
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: Nothing such press mode (mode=%d)\n",
				__func__,
				value);
		break;
	}

	return size;
}

static ssize_t cypsoc_picoleaf_sysfs_press_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	ssize_t ret;
	int status;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	mutex_lock(&cpd->psoc_status_lock);
	status = cpd->psoc_status;
	mutex_unlock(&cpd->psoc_status_lock);

	switch (status)
	{
	case CYPSOC_PICOLEAF_STATUS_AVAILABLE:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "1\n");
		break;
	default:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "0\n");
		break;
	}
	return ret;
}
static DEVICE_ATTR(press_mode, 0660,
		cypsoc_picoleaf_sysfs_press_mode_show,
		cypsoc_picoleaf_sysfs_press_mode_store);

static ssize_t cypsoc_picoleaf_sysfs_offset_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	ssize_t ret;
	int rc = 0;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = cypsoc_picoleaf_clear_offset_intagration(cpd);

	if(rc){
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "error\n");
	}else{
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "%s\n",
				CYPSOC_OK_STATE);
	}
	return ret;
}
static DEVICE_ATTR(offset_reset, 0440, cypsoc_picoleaf_sysfs_offset_reset_show, NULL);

static ssize_t cypsoc_picoleaf_sysfs_press_integra_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	ssize_t ret;
	int rc = 0;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = cypsoc_picoleaf_clear_intagration(cpd);

	if(rc){
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "error\n");
	}else{
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "%s\n",
				CYPSOC_OK_STATE);
	}
	return ret;
}
static DEVICE_ATTR(press_integra_reset, 0440, cypsoc_picoleaf_sysfs_press_integra_reset_show, NULL);

static ssize_t cypsoc_picoleaf_noise_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	ssize_t ret;
	uint8_t noise_test;
	int     rc;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_TEST_COMMAND, 1, &noise_test);

	if (rc) {
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%s\n",
				CYPSOC_I2C_ERROR);
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: i2c read error; rc = %d\n",
				__func__,
				rc);
	}
	else{
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%d\n",
				noise_test);
	}
	return ret;
}

static ssize_t cypsoc_picoleaf_noise_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	struct i2c_client           *client;
	int value;
	int rc;
	uint8_t warg;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return size;
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);
	client = (struct i2c_client*)to_i2c_client(cpd->dev);

	rc = kstrtoint(buf, 10 , &value);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: sent invalid value\n",
				__func__);
		return size;
	}

	if(value == 1){
		cyp_debug(cpd->dev, CDL_INFO, "%s: starts noise test\n", __func__);
		warg = CYPSOC_PICOLEAF_START_NOISE_TEST;
		cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_TEST_COMMAND, 1, &warg);
	}
	return size;
}

static DEVICE_ATTR(noise_test, 0664, cypsoc_picoleaf_noise_test_show, cypsoc_picoleaf_noise_test_store);

static ssize_t cypsoc_picoleaf_test_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	struct i2c_client           *client;
	int value;
	int rc;
	int next_state;
	uint8_t warg;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return size;
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);
	client = (struct i2c_client*)to_i2c_client(cpd->dev);

	rc = kstrtoint(buf, 10 , &value);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: sent invalid value\n",
				__func__);
		return size;
	}

	if(cpd->psoc_status == CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE ||
			cpd->psoc_status == CYPSOC_PICOLEAF_STATUS_AVAILABLE){
		if(value == 0){
			cyp_debug(cpd->dev, CDL_INFO, "%s: test mode OFF\n", __func__);
			warg = CYPSOC_PICOLEAF_TEST_MODE_OFF;
			next_state = CYPSOC_PICOLEAF_STATUS_AVAILABLE;
		}else{
			cyp_debug(cpd->dev, CDL_INFO, "%s: test mode ON\n", __func__);
			warg = CYPSOC_PICOLEAF_TEST_MODE_ON;
			next_state = CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE;
		}
	}else{
		cyp_debug(cpd->dev, CDL_INFO, "%s: driver state is not \"on\" or \"on[testmode]\" (skipped what change test mode)\n", __func__);
		return size;
	}

	rc = cypsoc_picoleaf_i2c_write(cpd, client, CYPSOC_PICOLEAF_REG_TEST_MODE, 1, &warg);

	if(rc) cyp_debug(cpd->dev, CDL_ERROR, "%s: FAILED to switch test mode!\n", __func__);
	else{
		mutex_lock(&cpd->psoc_status_lock);
		cpd->psoc_status = next_state;
		mutex_unlock(&cpd->psoc_status_lock);
	}

	return size;
}

static DEVICE_ATTR(test_mode, 0664, NULL, cypsoc_picoleaf_test_mode_store);

static ssize_t cypsoc_picoleaf_noise_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	ssize_t ret;
	uint8_t noise_data_hl[2];
	int32_t noise_data;
	int     rc;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_NOISE_DATA_L, 2, noise_data_hl);


	if (rc) {
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%s\n",
				CYPSOC_I2C_ERROR);
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: i2c read error; rc = %d\n",
				__func__,
				rc);
	}
	else{
		noise_data = (noise_data_hl[1] << 8) + noise_data_hl[0];
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%d\n",
				noise_data);
	}
	return ret;
}
static DEVICE_ATTR(noise_data, 0444, cypsoc_picoleaf_noise_data_show, NULL);

static ssize_t cypsoc_picoleaf_offset_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	ssize_t ret;
	uint8_t offset_hl[2];
	int32_t offset;
	int     rc;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_ADC_OFFSET_L, 2, offset_hl);

	if (rc) {
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%s\n",
				CYPSOC_I2C_ERROR);
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: i2c read error; rc = %d\n",
				__func__,
				rc);
	}
	else{
		offset = (offset_hl[1] << 8) + offset_hl[0];
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%d\n",
				offset);
	}

	return ret;
}
static DEVICE_ATTR(offset_data, 0444, cypsoc_picoleaf_offset_data_show, NULL);

static ssize_t cypsoc_picoleaf_press_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct  cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	ssize_t ret;
	int     press;
	int     rc;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = _cypsoc_picoleaf_get_press_z(cpd, &press);

	if (rc) {
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%s\n",
				CYPSOC_I2C_ERROR);
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: i2c read error; rc = %d\n",
				__func__,
				rc);
	}
	else{
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%d\n",
				press);
	}

	return ret;
}
static DEVICE_ATTR(press_z, 0444, cypsoc_picoleaf_press_z_show, NULL);

static ssize_t cypsoc_picoleaf_firmware_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	ssize_t ret;
	uint8_t fw_ver = 0;
	int     rc;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_FIRMWARE_VER, 1, &fw_ver);

	if (rc) {
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%s\n",
				CYPSOC_I2C_ERROR);
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: i2c read error; rc = %d\n",
				__func__,
				rc);
	}
	else{
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
				"%d\n",
				fw_ver);
	}
	return ret;
}
static DEVICE_ATTR(firmware_version, 0444, cypsoc_picoleaf_firmware_version_show, NULL);

static ssize_t cypsoc_picoleaf_stabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	ssize_t ret;
	uint8_t stabled_byte = 0;
	int     stabled;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);

	cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_MICON_STATUS, 1, &stabled_byte);

	cyp_debug(cpd->dev, CDL_INFO,
				"%s: PSoC status reg = 0x%x, status = %d\n",
				__func__,
				stabled_byte,
				cpd->psoc_status);

	stabled = (stabled_byte >> 7U) & 0x1U;

	ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"%d\n",
			stabled);

	return ret;
}
static DEVICE_ATTR(stabled, 0444, cypsoc_picoleaf_stabled_show, NULL);

static ssize_t cypsoc_picoleaf_reg_access_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client;
	int rc;
	ssize_t ret;
	uint8_t *data = NULL;
	int i;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "NULL drvdata...\n");
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);
	client = (struct i2c_client*)to_i2c_client(cpd->dev);

	switch (cpd->acc_mode)
	{
	case CYPSOC_ACCESS_MODE_READ:
		data = kzalloc(cpd->acc_arg*sizeof(uint8_t), GFP_KERNEL);
		if(data){
			rc = cypsoc_picoleaf_i2c_read(cpd, client, (uint16_t)cpd->acc_add, (uint8_t)cpd->acc_arg, data);

			if(rc){
				ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
						"%s\n", CYPSOC_I2C_ERROR);
			}else{
				ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "");
				for(i=0; i<cpd->acc_arg; i++){
					ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "%s%02X ", buf, data[i]);
				}
				ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "%s\n", buf);
			}
			kfree(data);
		}else{
			ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "mem_error\n");
		}
		break;
	case CYPSOC_ACCESS_MODE_SUCCESS:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "ok; writing was successful\n");
		break;
	default:
		ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE, "command_error\n");
	}

	return ret;
}

static ssize_t cypsoc_picoleaf_reg_access_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	struct i2c_client           *client;
	int value;
	int rc;
	char mode;
	char reg_add_str[4];
	char arg_str[11];
	int index = 0;
	int j=0;
	uint8_t warg;

	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return size;
	}
	cyp_debug(cpd->dev, CDL_INFO, "%s: driver status = %d\n",
			__func__,
			cpd->psoc_status);
	client = (struct i2c_client*)to_i2c_client(cpd->dev);

	cpd->acc_mode = CYPSOC_ACCESS_MODE_WRITE; //for error state

	if(strlen(buf) == 0){
		cyp_debug(cpd->dev, CDL_ERROR, "%s: too few command args (0)\n", __func__);
		return size;
	}
	mode = buf[0];

	if(mode != CYPSOC_ACCESS_MODE_READ && mode != CYPSOC_ACCESS_MODE_WRITE){
		cyp_debug(cpd->dev, CDL_ERROR, "%s: invalid command args [mode = %c]\n", __func__, mode);
		return size;
	}

	if(buf[1] != ' ' || strlen(buf)<3){
		cyp_debug(cpd->dev, CDL_ERROR, "%s: too few command args (1)\n", __func__);
		return size;
	}

	memset(reg_add_str, 0, sizeof(reg_add_str));
	rc = CYPSOC_PICOLEAF_RET_NG;
	for(index = 2; index<strlen(buf); index++){
		if(buf[index] == ' '){
			if(!rc) break;
			j++; // seek
		}else{
			if(index-2-j > 3){ // over array size
				rc = CYPSOC_PICOLEAF_RET_NG;
				break;
			}
			rc = CYPSOC_PICOLEAF_RET_OK;
			reg_add_str[index-2-j] = buf[index];
		}
	}
	if(rc) {
		cyp_debug(cpd->dev, CDL_ERROR, "%s: command syntax error (a)\n", __func__);
		return size;
	}

	rc = CYPSOC_PICOLEAF_RET_NG;
	index++;
	while(index<strlen(buf)){
		if(buf[index] != ' '){
			memcpy(arg_str, &buf[index], 10);
			rc = CYPSOC_PICOLEAF_RET_OK;
			break;
		}
		index++;
	}
	if(rc) {
		cyp_debug(cpd->dev, CDL_ERROR, "%s: too few command args (b)\n", __func__);
		return size;
	}

	// store args to cpd
	cyp_debug(cpd->dev, CDL_INFO, "%s: reg_add_str = %s\n", __func__, reg_add_str);
	rc = kstrtoint(reg_add_str, 16, &value);
	if(rc) {
		cyp_debug(cpd->dev, CDL_ERROR, "%s: invalid address arg (3)\n", __func__);
		return size;
	}
	cpd->acc_add = value;

	cyp_debug(cpd->dev, CDL_INFO, "%s: arg_str = %s\n", __func__, arg_str);
	if(mode == CYPSOC_ACCESS_MODE_READ){
		rc = kstrtoint(arg_str, 10, &value);
		if(value > 255){
			value = 255;
		}else if(value < 0){
			value = 0;
		}
	}else{
		rc = kstrtoint(arg_str, 16, &value);
	}

	if(rc) {
		cyp_debug(cpd->dev, CDL_ERROR, "%s: command syntax error (c)\n", __func__);
		return size;
	}
	cpd->acc_arg = value;

	cpd->acc_mode = mode;

	if(cpd->acc_mode == CYPSOC_ACCESS_MODE_WRITE){
		warg = (uint8_t)(cpd->acc_arg & 0xFF);
		cyp_debug(cpd->dev, CDL_INFO, "%s: writes to PSoC that mode[%c], addr=0x%04X, param=0x%02X\n",
				__func__, cpd->acc_mode, cpd->acc_add, warg);
		rc = cypsoc_picoleaf_i2c_write(cpd, client, (uint16_t)cpd->acc_add, 1U, &warg);
		if(!rc){
			cpd->acc_mode = CYPSOC_ACCESS_MODE_SUCCESS;
		}
	}

	return size;
}
static DEVICE_ATTR(reg_access, 0600, cypsoc_picoleaf_reg_access_show, cypsoc_picoleaf_reg_access_store);

////      (sysfs For debug)      ////
static ssize_t cypsoc_picoleaf_log_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	ssize_t ret;

	ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"log_level = %d\n"
			"\n"
			"--------------\n"
			"CDL_QUIET = %d\n"
			"CDL_ERROR = %d\n"
			"CDL_WARN  = %d\n"
			"CDL_INFO  = %d\n"
			"CDL_DEBUG = %d\n",
			cpd->debug_level,
			CDL_QUIET,
			CDL_ERROR,
			CDL_WARN,
			CDL_INFO,
			CDL_DEBUG);

	return ret;
}

static ssize_t cypsoc_picoleaf_log_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = kstrtoint(buf, 10 , &value);
	if (rc) {
		cyp_debug(dev, CDL_ERROR, "%s: Invalid value\n", __func__);
		return size;
	}

	if (value < CDL_MAX){
		cpd->debug_level = value;
	}else{
		cyp_debug(dev, CDL_ERROR, "%s: Invalid value\n", __func__);
	}

	return size;
}
static DEVICE_ATTR(log_level, 0664, cypsoc_picoleaf_log_level_show, cypsoc_picoleaf_log_level_store);

static ssize_t cypsoc_picoleaf_i2c_get_adc_data_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	int     rc;
	ssize_t ret;
	uint8_t adc_data_hl[2];
	int32_t adc_data;

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_ADC_DATA_L, 2, adc_data_hl);
	adc_data = (adc_data_hl[1] << 8) + adc_data_hl[0];

	ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"adc_data = %d (H, L) = (0x%x, 0x%x), err = %d\n",
			adc_data,
			adc_data_hl[1],
			adc_data_hl[0],
			rc);

	return ret;
}
static DEVICE_ATTR(i2c_get_adc_data_debug, 0444, cypsoc_picoleaf_i2c_get_adc_data_debug_show, NULL);

static ssize_t cypsoc_picoleaf_i2c_get_press_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypsoc_picoleaf_data *cpd    = dev_get_drvdata(dev);
	struct i2c_client           *client = to_i2c_client(cpd->dev);
	int     rc;
	ssize_t ret;
	uint8_t press_hl[2];
	int32_t press;

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_INTEGRATED_PRESS_L, 2, press_hl);
	press = (press_hl[1] << 8) + press_hl[0];

	ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			"press = %d (H, L) = (0x%x, 0x%x), err = %d\n",
			press,
			press_hl[1],
			press_hl[0],
			rc);

	return ret;
}
static DEVICE_ATTR(i2c_get_press_debug, 0444, cypsoc_picoleaf_i2c_get_press_debug_show, NULL);

static ssize_t cypsoc_picoleaf_function_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	ret = snprintf(buf, CYPSOC_PICOLEAF_MAX_PRBUF_SIZE,
			" 1: _cypsoc_picoleaf_firmware_update(force_update=false)\n"
			" 2: cypsoc_picoleaf_clear_offset_intagration()\n"
			" 3: cypsoc_picoleaf_i2c_esd_retry_work()\n"
			" 4: _cypsoc_picoleaf_firmware_update(force_update=true)\n"
			);

	return ret;
}

static ssize_t cypsoc_picoleaf_function_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = kstrtoint(buf, 10 , &value);
	if (rc) {
		//cyp_debug(dev, DL_ERROR, "%s: Invalid value\n", __func__);
		return size;
	}

	switch (value)
	{
	case 1:
		_cypsoc_picoleaf_firmware_update(cpd, 0);
		break;
	case 2:
		cypsoc_picoleaf_clear_offset_intagration(cpd);
		break;
	case 3:
		cypsoc_picoleaf_i2c_esd_retry_work(cpd);
		break;
	case 4:
		_cypsoc_picoleaf_firmware_update(cpd, 1);
		break;
	default:
		break;
	}

	return size;
}
static DEVICE_ATTR(function_debug, 0600, cypsoc_picoleaf_function_debug_show, cypsoc_picoleaf_function_debug_store);

static ssize_t cypsoc_picoleaf_state_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	int value, rc;
	value = 0;

	rc = kstrtoint(buf, 10 , &value);
	if (rc) return size;

	cpd->psoc_status = value;
	return size;
}
static DEVICE_ATTR(state_debug, 0600, NULL, cypsoc_picoleaf_state_debug_store);

static ssize_t cypsoc_picoleaf_prev_state_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cypsoc_picoleaf_data *cpd = dev_get_drvdata(dev);
	int value, rc;
	value = 0;

	rc = kstrtoint(buf, 10 , &value);
	if (rc) return size;

	cpd->prev_status = value;
	return size;
}
static DEVICE_ATTR(prev_state_debug, 0600, NULL, cypsoc_picoleaf_prev_state_debug_store);

static struct attribute *cypsoc_picoleaf_attrs[] = {
	&dev_attr_i2c_get_adc_data_debug.attr,
	&dev_attr_i2c_get_press_debug.attr,
	&dev_attr_psoc_state.attr,
	&dev_attr_press_mode.attr,
	&dev_attr_offset_reset.attr,
	&dev_attr_press_integra_reset.attr,
	&dev_attr_log_level.attr,
	&dev_attr_noise_test.attr,
	&dev_attr_noise_data.attr,
	&dev_attr_test_mode.attr,
	&dev_attr_offset_data.attr,
	&dev_attr_press_z.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_stabled.attr,
	&dev_attr_reg_access.attr,
	&dev_attr_function_debug.attr,
	&dev_attr_state_debug.attr,
	&dev_attr_prev_state_debug.attr,
	NULL
};

static struct attribute_group cypsoc_picoleaf_attr_group = {
	.attrs	= cypsoc_picoleaf_attrs,
};

static int cypsoc_picoleaf_initialize_driver_data(struct cypsoc_picoleaf_data *cpd)
{
	// Initialize device info //
	cpd->i2c_pull_up     = CYPSOC_PICOLEAF_PT_I2C_PULL_UP_NG;
	cpd->psoc_status     = CYPSOC_PICOLEAF_STATUS_INITIALIZING;
	cpd->prev_status     = CYPSOC_PICOLEAF_STATUS_INITIALIZING;
	cpd->debug_level     = CYPSOC_INITIAL_DEBUG_LEVEL;
	cpd->acc_mode        = CYPSOC_ACCESS_MODE_WRITE;
	cpd->acc_add         = 0;
	cpd->acc_arg         = 0;
	cpd->pt_core_data    = NULL;

	// Initialize mutexes and spinlocks //
	mutex_init(&cpd->sysfs_lock);
	mutex_init(&cpd->psoc_status_lock);
	mutex_init(&cpd->prev_status_lock);
	mutex_init(&cpd->i2c_power_changed_lock);

	// Initialize works //
	INIT_WORK(&cpd->i2c_retry_work, cypsoc_picoleaf_i2c_esd_retry_work_handler);

	// Initialize GPIO //
	// GPIO is set at pt_cypsoc_picoleaf_i2c_probe() in pt_i2c.c
	cpd->rst_gpio = 0;
	cpd->vdd_gpio = 0;
	cpd->vref_gpio = 0;

	return 0;
}

static void cypsoc_picoleaf_release_device_data(struct cypsoc_picoleaf_data *cpd)
{
	struct device *dev = cpd->dev;

	if (cpd->sysfs_class){
		device_destroy(cpd->sysfs_class, 0);
		class_destroy(cpd->sysfs_class);
	}

	dev_set_drvdata(dev, NULL);
	if(cpd) kfree(cpd);
}

/////////////////////////////////////////////////
////    Interfaces For touchscreen driver    ////

int cypsoc_picoleaf_probe_cont(struct cypsoc_picoleaf_data *cpd)
{
	int rc=0;

	pr_info("cypsoc_picoleaf_probe_cont() starts\n");

	if (!cpd){
		printk("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return CYPSOC_PICOLEAF_RET_NG;
	}

	cpd->sysfs_class = class_create(THIS_MODULE, "rakuraku_touch");
	if(!cpd->sysfs_class){
		printk("sysfs_class could not be created\n");
		rc = CYPSOC_PICOLEAF_RET_NG;
	}


	if(!rc){

		cpd->sysfs_dev = device_create(cpd->sysfs_class, NULL, 0, cpd, "psoc_dev");
		if(!cpd->sysfs_dev){
			printk("sysfs_dev could not be created\n");
			rc = CYPSOC_PICOLEAF_RET_NG;
			class_destroy(cpd->sysfs_class);
			cpd->sysfs_class = NULL;
		}
	}
	if(!rc){
		rc = sysfs_create_group(&(cpd->sysfs_dev->kobj), &cypsoc_picoleaf_attr_group);
		if(rc) {
			rc = CYPSOC_PICOLEAF_RET_NG;
			device_destroy(cpd->sysfs_class, 0);
			cpd->sysfs_dev = NULL;
			class_destroy(cpd->sysfs_class);
			cpd->sysfs_class = NULL;
		}
	}

	rc = cypsoc_picoleaf_initialize_driver_data(cpd);
	if(rc) goto err_probe;
	dev_set_drvdata(cpd->dev, cpd);

	cpd_global = cpd;

	printk("cypsoc_picoleaf_probe() reached normal END\n");
	pr_info("cypsoc_picoleaf_probe_cont() end\n");

	return rc;

err_probe:
	mutex_lock(&cpd->psoc_status_lock);
	cpd->psoc_status = CYPSOC_PICOLEAF_STATUS_BROKEN;
	mutex_unlock(&cpd->psoc_status_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_probe_cont);

void cypsoc_picoleaf_i2c_power_turned_on(struct cypsoc_picoleaf_data *cpd)
{
	if(cpd == NULL){
		pr_err("%s: NULL illegal\n", __func__);
		return;
	}
	mutex_lock(&cpd->i2c_power_changed_lock);
	cpd->i2c_pull_up = CYPSOC_PICOLEAF_PT_I2C_PULL_UP_OK;
	mutex_unlock(&cpd->i2c_power_changed_lock);
	cyp_debug(cpd->dev, CDL_INFO,
			"%s: changes pcoleaf driver i2c state to I2C_PULL_UP_OK\n",
			__func__);
}

void cypsoc_picoleaf_i2c_power_turned_off(struct cypsoc_picoleaf_data *cpd)
{
	if(cpd == NULL){
		pr_err("%s: NULL illegal\n", __func__);
		return;
	}
	mutex_lock(&cpd->i2c_power_changed_lock);
	cpd->i2c_pull_up = CYPSOC_PICOLEAF_PT_I2C_PULL_UP_NG;
	mutex_unlock(&cpd->i2c_power_changed_lock);
	cyp_debug(cpd->dev, CDL_INFO,
			"%s: changes pcoleaf driver i2c state to I2C_PULL_UP_NG\n",
			__func__);
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_i2c_power_turned_off);

void cypsoc_picoleaf_shutdown_cont(struct cypsoc_picoleaf_data *cpd)
{
	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return;
	}

	(void)cypsoc_picoleaf_power_off_hw(cpd);

	if (cpd->sysfs_dev) {
		sysfs_remove_group(&(cpd->sysfs_dev->kobj), &cypsoc_picoleaf_attr_group);
	}
	cypsoc_picoleaf_release_device_data(cpd);

	pr_info("%s: removes cypress PSoC data and sysfs.", __func__);
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_shutdown_cont);

/*******************************************************************************
 * FUNCTION: cypsoc_picoleaf_i2c_readied
 *
 * SUMMARY: trantsitions PSoC state to INITIALIZING and I2C_PULL_UP_OK
 * if necessary.
 *
 * RETURN:
 *
 * PARAMETERS:
 *	*cpd   - pointer to cypress PSoC core data
 ******************************************************************************/
void cypsoc_picoleaf_i2c_readied(struct cypsoc_picoleaf_data *cpd)
{
	uint8_t firmware_ver = 0;
	int     rc;
	struct i2c_client *client;

	if(cpd == NULL){
		pr_err("%s: cypsoc_picoleaf_data is still NULL\n", __func__);
		return;
	}
	client = (struct i2c_client *)to_i2c_client(cpd->dev);

	cypsoc_picoleaf_i2c_power_turned_on(cpd);

	rc = cypsoc_picoleaf_i2c_read(cpd, client, CYPSOC_PICOLEAF_REG_FIRMWARE_VER, 1, &firmware_ver);
	if (rc) {
		cyp_debug(cpd->dev, CDL_ERROR,
				"%s: PSoC (Picoleaf) first I2C access ERROR\n", __func__);
	}
	else {
		cyp_debug(cpd->dev, CDL_INFO,
				"%s: PSoC (Picoleaf) firmware version = %d\n",
				__func__,
				firmware_ver);
	}
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_i2c_readied);

/*******************************************************************************
 * FUNCTION: cypsoc_picoleaf_failure_check
 *
 * SUMMARY: gets "press Z" data of cypress PSoC.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *	*cpd   - pointer to cypress PSoC core data
 *  *press - pointer to store the retrieved press Z
 ******************************************************************************/
int cypsoc_picoleaf_get_press_z(int *press)
{
	int rc;
	if (!cpd_global){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		*press = CYPSOC_NOTIFY_FW_TO_RK_OFF;
		return CYPSOC_PICOLEAF_RET_NG;
	}

	if(cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE){
		*press = CYPSOC_TEST_MODE_PRESS_Z;
		rc = CYPSOC_PICOLEAF_RET_OK;
	}else{
		rc = _cypsoc_picoleaf_get_press_z(cpd_global, press);
	}

	if (!rc){
		if (*press > 0){
			return rc;
		} else if (*press == 0){
			if(cpd_global->psoc_status != CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE)
				*press = CYPSOC_DEFAULT_PRESS_Z;
		} else {
			rc = CYPSOC_PICOLEAF_RET_NG;
		}
	}
	return rc;
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_get_press_z);

/*******************************************************************************
 * FUNCTION: cypsoc_picoleaf_notification_enabled
 *
 * SUMMARY: returns either cypress PSoC is broken or not.
 *
 * RETURN:
 *   0 = normal
 *	!0 = malfunction
 *
 * PARAMETERS:
 *	*cpd  - pointer to cypress PSoC core data
 ******************************************************************************/
int cypsoc_picoleaf_notification_enabled(void){

	if (!cpd_global){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return CYPSOC_PICOLEAF_RET_NG;
	}
	return cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_AVAILABLE ||
			cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_RAISE_ERROR ||
			cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE;
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_notification_enabled);

/*******************************************************************************
 * FUNCTION: cypsoc_picoleaf_firmware_update
 *
 * SUMMARY: starts to update cypress PSoC firmware if the firmware image is
 * newer then current it.
 *
 * RETURN:
 *   0 = success
 *  !0 = failure
 *
 * PARAMETERS:
 *	*cpd  - pointer to cypress PSoC core data
 ******************************************************************************/
int cypsoc_picoleaf_firmware_update(struct cypsoc_picoleaf_data *cpd){
	if (!cpd){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return CYPSOC_PICOLEAF_RET_NG;
	}
	return _cypsoc_picoleaf_firmware_update(cpd, 0);
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_firmware_update);

void cypsoc_picoleaf_suspend(void){
	if (!cpd_global){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return;
	}
	cyp_debug(cpd_global->dev, CDL_INFO,
			"%s: Picoleaf PSoC suspends\n",
			__func__);
	if (cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_AVAILABLE){
		cyp_debug(cpd_global->dev, CDL_INFO,
				"%s: PSoC rakuraku OFF for suspend\n",
				__func__);
		cypsoc_picoleaf_set_deep_sleep_mode_enable(cpd_global);
	}
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_suspend);

void cypsoc_picoleaf_resume(void){
	if (!cpd_global){
		pr_err("%s: Error: cypsoc_picoleaf_data is NULL\n", __func__);
		return;
	}
	cyp_debug(cpd_global->dev, CDL_INFO,
			"%s: Picoleaf PSoC resumes\n",
			__func__);
	if (cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_ON_TEST_MODE){
		mutex_lock(&cpd_global->psoc_status_lock);
		cpd_global->psoc_status = CYPSOC_PICOLEAF_STATUS_AVAILABLE;
		mutex_unlock(&cpd_global->psoc_status_lock);
		cyp_debug(cpd_global->dev, CDL_INFO, "%s: driver state was changed ON[TESTMODE] --> ON[normal]\n", __func__);
	}else if (cpd_global->psoc_status == CYPSOC_PICOLEAF_STATUS_AVAILABLE){
		cyp_debug(cpd_global->dev, CDL_INFO,
				"%s: PSoC rakuraku ON for resume\n",
				__func__);
		cypsoc_picoleaf_set_deep_sleep_mode_disable(cpd_global);
	}
}
EXPORT_SYMBOL_GPL(cypsoc_picoleaf_resume);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("cypress PSoC i2c driver module for Picoleaf");
MODULE_AUTHOR("FCNT LIMITED");
