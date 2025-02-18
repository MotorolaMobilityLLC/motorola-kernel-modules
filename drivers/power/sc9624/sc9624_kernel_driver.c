// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
*/

#define pr_fmt(fmt)	"[sc9624] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/sched/clock.h>
#include <uapi/linux/sched/types.h>

#include "sc9624_kernel_reg.h"

#define SC9624_IRQ_WAKE_TIME    (500) /* ms */
int sc9624_send_event(struct sc9624 *sc, struct wls_event_msg *msg);

/**********************************IIC API*********************************/
static const struct regmap_config sc9624_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

static int __sc9624_read_block(struct sc9624 *sc, uint16_t reg,
        uint8_t *data, uint8_t length)
{
    int ret;

    ret = regmap_raw_read(sc->regmap, reg, data, length);
    if (ret < 0) {
        sc_err("i2c read fail: can't read from reg 0x%04X\n", reg);
    }

    return ret;
}

static int __sc9624_write_block(struct sc9624 *sc, uint16_t reg,
        uint8_t *data, uint8_t length)
{
    int ret;

    ret = regmap_raw_write(sc->regmap, reg, data, length);
    if (ret < 0) {
        sc_err("i2c write fail: can't write 0x%04X: %d\n", reg, ret);
    }

    return ret;
}

static int sc9624_read_block(struct sc9624 *sc, uint16_t reg,
        uint8_t *data, uint8_t len)
{
    int ret;

    if (sc->fw_program) {
        sc_err("firmware programming\n");
        return -1;
    }

    mutex_lock(&sc->i2c_rw_lock);
    ret = __sc9624_read_block(sc, reg, data, len);
    mutex_unlock(&sc->i2c_rw_lock);

    return ret;
}

static int sc9624_write_block(struct sc9624 *sc, uint16_t reg,
        uint8_t *data, uint8_t len)
{
    int ret;

    if (sc->fw_program) {
        sc_err("firmware programming\n");
        return -1;
    }

    mutex_lock(&sc->i2c_rw_lock);
    ret = __sc9624_write_block(sc, reg, data, len);
    mutex_unlock(&sc->i2c_rw_lock);

    return ret;
}

//-------------------sc9624 system interface-------------------
static int sc9624_rx_set_cmd(struct sc9624 *sc, RX_CMD cmd);

static int sc9624_get_chipid(struct sc9624 *sc, uint16_t *chip_id)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, ChipID, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)chip_id,
            (uint8_t)sizeof(((RXCustType *)0)->ChipID));
    if (ret) {
        sc_err("sc9624 get chip id fail\n");
    }

    return ret;
}

int sc9624_online(struct sc9624 *sc)
{
    uint16_t chip_id;

    return sc9624_get_chipid(sc, &chip_id);
}

static int sc9624_get_custid(struct sc9624 *sc, uint16_t *cust_id)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, CustID, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)cust_id,
            (uint8_t)sizeof(((RXCustType *)0)->CustID));
    if (ret) {
        sc_err("sc9624 get cust id fail\n");
    }

    return ret;
}

int sc9624_get_fwver(struct sc9624 *sc, uint32_t *fw_ver)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, FwVer, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)fw_ver,
            (uint8_t)sizeof(((RXCustType *)0)->FwVer));
    if (ret) {
        sc_err("sc9624 get fw ver fail\n");
    }

    return ret;
}

static int sc9624_get_hwver(struct sc9624 *sc, uint32_t *hw_ver)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, HwVer, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)hw_ver,
            (uint8_t)sizeof(((RXCustType *)0)->HwVer));
    if (ret) {
        sc_err("sc9624 get hw ver fail\n");
    }

    return ret;
}

static int sc9624_get_gitver(struct sc9624 *sc, uint32_t *git_ver)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, GitVer, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)git_ver,
            (uint8_t)sizeof(((RXCustType *)0)->GitVer));
    if (ret) {
        sc_err("sc9624 get git ver fail\n");
    }

    return ret;
}

int sc9624_get_mcode(struct sc9624 *sc, uint32_t *mcode)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, mCode, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)mcode,
            (uint8_t)sizeof(((RXCustType *)0)->mCode));
    if (ret) {
        sc_err("sc9624 get mcode fail\n");
    }

    return ret;
}

int sc9624_get_frequecy(struct sc9624 *sc, uint16_t *freq)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Frequecy, offset);

    //refresh
    ret = sc9624_rx_set_cmd(sc, RX_REFRESH);
    if (ret) {
        sc_err("sc9624 set cmd refresh fail\n");
    }

    ret = sc9624_read_block(sc, offset, (uint8_t *)freq,
            (uint8_t)sizeof(((RXCustType *)0)->Frequecy));
    if (ret) {
        sc_err("sc9624 get mcode fail\n");
    }

    return ret;
}

int sc9624_get_vrect(struct sc9624 *sc, uint16_t *vrect)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Vrect, offset);

    //refresh
    ret = sc9624_rx_set_cmd(sc, RX_REFRESH);
    if (ret) {
        sc_err("sc9624 set cmd refresh fail\n");
    }

    ret = sc9624_read_block(sc, offset, (uint8_t *)vrect,
            (uint8_t)sizeof(((RXCustType *)0)->Vrect));
    if (ret) {
        sc_err("sc9624 get vrect fail\n");
    }

    return ret;
}

int sc9624_get_vping(struct sc9624 *sc, uint32_t *vping)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Vping, offset);

    ret = sc9624_read_block(sc, offset, (uint8_t *)vping,
            (uint8_t)sizeof(((RXCustType *)0)->Vping));
    if (ret) {
        sc_err("sc9624 get Vping fail\n");
    }

    return ret;
}

int sc9624_get_voltage(struct sc9624 *sc, uint32_t *volt)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Vout, offset);

    //refresh
    ret = sc9624_rx_set_cmd(sc, RX_REFRESH);
    if (ret) {
        sc_err("sc9624 set cmd refresh fail\n");
    }

    ret = sc9624_read_block(sc, offset, (uint8_t *)volt,
            (uint8_t)sizeof(((RXCustType *)0)->Vout));
    if (ret) {
        sc_err("sc9624 get voltage fail\n");
    }

    return ret;
}

int sc9624_get_voltage_setting(struct sc9624 *sc, uint32_t *volt)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, VoutSet, offset);

    //refresh
    ret = sc9624_rx_set_cmd(sc, RX_REFRESH);
    if (ret) {
        sc_err("sc9624 set cmd refresh fail\n");
    }

    ret = sc9624_read_block(sc, offset, (uint8_t *)volt,
            (uint8_t)sizeof(((RXCustType *)0)->VoutSet));
    if (ret) {
        sc_err("sc9624 get voltage_setting fail\n");
    }

    return ret;
}

int sc9624_get_current(struct sc9624 *sc, uint32_t *curr)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Iout, offset);

    //refresh
    ret = sc9624_rx_set_cmd(sc, RX_REFRESH);
    if (ret) {
        sc_err("sc9624 set cmd refresh fail\n");
    }

    ret = sc9624_read_block(sc, offset, (uint8_t *)curr,
            (uint8_t)sizeof(((RXCustType *)0)->Iout));
    if (ret) {
        sc_err("sc9624 get current fail\n");
    }

    return ret;
}

int sc9624_get_tdie(struct sc9624 *sc, uint32_t *tdie)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Tdie, offset);

    //refresh
    ret = sc9624_rx_set_cmd(sc, RX_REFRESH);
    if (ret) {
        sc_err("sc9624 set cmd refresh fail\n");
    }

    ret = sc9624_read_block(sc, offset, (uint8_t *)tdie,
            (uint8_t)sizeof(((RXCustType *)0)->Tdie));
    if (ret) {
        sc_err("sc9624 get tdie fail\n");
    }

    return ret;
}

static int sc9624_get_nego_power(struct sc9624 *sc, uint32_t *power)
{
    int ret;
    uint16_t offset = 0;
    ContractType type = {0x00};

    OFFSET(RXCustType, ReqContract, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)&type,
            (uint8_t)sizeof(((RXCustType *)0)->ReqContract));
    if (ret) {
        sc_err(" fail, ret:%d\n", ret);
    } else {
        *power = type.guaranteed_power / 2;
    }

    return ret;
}

int sc9624_get_rx_ce(struct sc9624 *sc, int *ce)
{
	int ret = -1;
	uint16_t offset = 0;
	PTpktType pkt = {0x00};

	OFFSET(RXCustType, PTPkt, offset);
	ret = sc9624_read_block(sc, offset, (uint8_t *)&pkt,
			(uint8_t)sizeof(((RXCustType *)0)->PTPkt));
	if (ret) {
		sc_err(" fail, ret:%d\n", ret);
	} else {
		*ce = (int) pkt.cep;
	}

	return ret;
}

int sc9624_get_tx_mcode(struct sc9624 *sc, uint16_t *mcode)
{
	int ret = -1;
	uint16_t offset = 0;
	TxInfoType info = {0x00};

	OFFSET(RXCustType, TxInfo, offset);
	ret = sc9624_read_block(sc, offset, (uint8_t *)&info,
			(uint8_t)sizeof(((RXCustType *)0)->TxInfo));
	if (ret) {
		sc_err(" fail, ret:%d\n", ret);
	} else {
		*mcode = (int) info.mcode;
	}

	return ret;
}

//-------------------sc9624 RX interface-------------------
static int sc9624_rx_set_cmd(struct sc9624 *sc, RX_CMD cmd)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, CMD, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&cmd,
            (uint8_t)sizeof(((RXCustType *)0)->CMD));
    if (ret) {
        sc_err("sc9624 rx set cmd fail\n");
    }

    return ret;
}

static int sc9624_rx_set_Vout(struct sc9624 *sc, uint32_t vout)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, VoutSet, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&vout,
            (uint8_t)sizeof(((RXCustType *)0)->VoutSet));
    if (ret) {
        sc_err("sc9624 set vout fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_VOUT_CHANGE);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

static int sc9624_rx_set_Ilimit(struct sc9624 *sc, uint16_t ilimt)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Ilimit, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&ilimt,
            (uint8_t)sizeof(((RXCustType *)0)->Ilimit));
    if (ret) {
        sc_err("sc9624 set ilimit fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_ILIMIT_CHANGE);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

int sc9624_rx_set_lvp(struct sc9624 *sc, uint16_t vlvp)
{


    return 0;
}

int sc9624_rx_set_vout_ovp(struct sc9624 *sc, uint16_t vovp)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, OVP, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&vovp,
            (uint8_t)sizeof(((RXCustType *)0)->OVP));
    if (ret) {
        sc_err("sc9624 set vout ovp fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_OVP_CHANGE);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}


int sc9624_rx_set_vout_ocp(struct sc9624 *sc, uint16_t iocp)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, OCP, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&iocp,
            (uint8_t)sizeof(((RXCustType *)0)->OCP));
    if (ret) {
        sc_err("sc9624 set iout ocp fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_OCP_CHANGE);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

int sc9624_rx_set_otp(struct sc9624 *sc, uint32_t otp)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, OTP, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&otp,
            (uint8_t)sizeof(((RXCustType *)0)->OTP));
    if (ret) {
        sc_err("sc9624 set otp fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_OTP_CHANGE);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

int sc9624_rx_vdd_enable(struct sc9624 *sc, bool enable)
{
    if (enable) {
        return sc9624_rx_set_cmd(sc, RX_VDD_ENABLE);
    } else {
        return sc9624_rx_set_cmd(sc, RX_VDD_DISABLE);
    }
}

int sc9624_rx_clr_int(struct sc9624 *sc, uint32_t rxint)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, IntClr, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&rxint,
            (uint8_t)sizeof(((RXCustType *)0)->IntClr));
    if (ret) {
        sc_err("sc9624 clear rx int fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_INT_CLR);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

int sc9624_rx_get_int(struct sc9624 *sc, uint32_t *rxint)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, IntFlag, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)rxint,
            (uint8_t)sizeof(((RXCustType *)0)->IntFlag));
    if (ret) {
        sc_err("sc9624 clear rx int fail\n");
    }

    return ret;
}

int sc9624_rx_get_sysmode(struct sc9624 *sc, uint32_t *sysmode)
{
	int ret = 0;
	uint16_t offset = 0;

	OFFSET(RXCustType, SYSMode, offset);
	ret = sc9624_read_block(sc, offset, (uint8_t *)sysmode,
			(uint8_t)sizeof(((RXCustType *)0)->SYSMode));
	if (ret) {
		sc_err("sc9624 read sysmode fail\n");
	}

	return ret;
}

int sc9624_rx_send_ept(struct sc9624 *sc, EPT_RESON ept_v)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, EPTReson, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&ept_v,
            (uint8_t)sizeof(((RXCustType *)0)->EPTReson));
    if (ret) {
        sc_err("sc9624 clear rx int fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_SEND_EPT);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

static int sc9624_rx_recv_fsk_pkt(struct sc9624 *sc, FskType *fsk)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Fsk, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)(fsk->buf),
            MAX_FSK_SIZE);
    if (ret) {
        sc_err("sc9624 read recv fsk pkt fail\n");
    }

    return ret;
}

int sc9624_rx_send_ask_pkt(struct sc9624 *sc, AskType *ask)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, Ask, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)(ask->buf),
            MAX_ASK_SIZE);
    if (ret) {
        sc_err("sc9624 write send ask pkt fail\n");
    }

    ret = sc9624_rx_set_cmd(sc, RX_SEND_PPP);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

int sc9624_rx_config_fod(struct sc9624 *sc, FodType *fod)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(RXCustType, FOD, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)fod, 16);
    if (ret) {
        sc_err("sc9624 write send ask pkt fail\n");
    }

    return ret;
}

//-------------------sc9624 RX interface-------------------
static int sc9624_tx_set_cmd(struct sc9624 *sc, TX_CMD cmd)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(TXCustType, CMD, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&cmd,
            (uint8_t)sizeof(((TXCustType *)0)->CMD));
    if (ret) {
        sc_err("sc9624 tx set cmd fail\n");
    }

    return ret;
}

/*
static int sc9624_tx_func_enable(struct sc9624 *sc)
{
    return sc9624_tx_set_cmd(sc, TX_ENABLE);
}

static int sc9624_tx_fod_enable(struct sc9624 *sc, bool enable)
{
    if (enable) {
        return sc9624_tx_set_cmd(sc, TX_FOD_ENABLE);
    } else {
        return sc9624_tx_set_cmd(sc, TX_FOD_DISABLE);
    }
}
*/

int sc9624_tx_clr_int(struct sc9624 *sc, uint32_t txint)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(TXCustType, IntClr, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)&txint,
            (uint8_t)sizeof(((TXCustType *)0)->IntClr));
    if (ret) {
        sc_err("sc9624 clear tx int fail\n");
    }

    ret = sc9624_tx_set_cmd(sc, TX_INT_CLR);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }


    return ret;
}

int sc9624_tx_get_int(struct sc9624 *sc, uint32_t *txint)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(TXCustType, IntFlag, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)txint,
            (uint8_t)sizeof(((TXCustType *)0)->IntFlag));
    if (ret) {
        sc_err("sc9624 clear tx int fail\n");
    }

    return ret;
}

int sc9624_tx_send_fsk_pkt(struct sc9624 *sc, FskType *fsk)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(TXCustType, Fsk, offset);
    ret = sc9624_write_block(sc, offset, (uint8_t *)(fsk->buf),
            MAX_FSK_SIZE);
    if (ret) {
        sc_err("sc9624 write send fsk pkt fail\n");
    }

    ret = sc9624_tx_set_cmd(sc, TX_SEND_PPP);
    if (ret) {
        sc_err("sc9624 set cmd fail\n");
    }

    return ret;
}

static int sc9624_tx_recv_ask_pkt(struct sc9624 *sc, AskType *ask)
{
    int ret;
    uint16_t offset = 0;

    OFFSET(TXCustType, Ask, offset);
    ret = sc9624_read_block(sc, offset, (uint8_t *)(ask->buf),
            MAX_ASK_SIZE);
    if (ret) {
        sc_err("sc9624 read recv ask pkt fail\n");
    }

    return ret;
}

//-------------------Interrupt interface-------------------
static int ocp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");

    return 0;
}

static int vout_ovp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int clamp_ovp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int ngage_ovp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int vout_lvp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int otp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int otp_160c_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int sleep_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int mode_change_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

//rx
static int rx_fsk_recv_irq_handler(struct sc9624 *sc)
{
    int ret;
    FskType fsk_pkt;
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");

    ret = sc9624_rx_recv_fsk_pkt(sc, &fsk_pkt);
    sc_info("ret:%d header:%d\n", ret , fsk_pkt.header);
    if (ret == 0) {
        msg.event = WLS_EVENT_RX_FSK_PKT;
        msg.len = wls_get_message_size((int)fsk_pkt.header);
        memcpy(&msg.data, fsk_pkt.msg, msg.len);
        ret = sc9624_send_event(sc, &msg);
    }

    return ret;
}

static int rx_ppp_timeout_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_ppp_success_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_afc_det_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_power_profile_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_poweron_irq_handler(struct sc9624 *sc)
{
    int ret;
    uint32_t regval;
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");
    msg.event = WLS_EVENT_RX_POWER_ON;
    ret = sc9624_send_event(sc, &msg);

    ret = sc9624_get_custid(sc, (uint16_t *)&regval);
    sc_info(" cust id : 0x%08x\n", regval);
    ret = sc9624_get_fwver(sc, &regval);
    sc_info(" fw ver : 0x%08x\n", regval);
    ret = sc9624_get_hwver(sc, &regval);
    sc_info(" hw ver : 0x%08x\n", regval);
    ret = sc9624_get_gitver(sc, &regval);
    sc_info(" git ver : 0x%08x\n", regval);
    ret = sc9624_get_vping(sc, &regval);
    sc_info(" Vping : %dmV\n", regval);

    return ret;
}

static int rx_ss_pkt_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_id_pkt_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_config_pkt_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_ready_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_ldo_on_irq_handler(struct sc9624 *sc)
{
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");
    msg.event = WLS_EVENT_RX_LDO_ON;
    sc9624_send_event(sc, &msg);
    return 0;
}

static int rx_ldo_off_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_pldo_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_scp_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int rx_hs_ok_irq_handler(struct sc9624 *sc)
{
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");
    msg.event = WLS_EVENT_HS_OK;
    sc9624_send_event(sc, &msg);
    return 0;
}

static int rx_hs_fail_irq_handler(struct sc9624 *sc)
{
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");
    msg.event = WLS_EVENT_HS_FAIL;
    sc9624_send_event(sc, &msg);
    return 0;
}

static int rx_nego_power_irq_handler(struct sc9624 *sc)
{
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");
    msg.event = WLS_EVENT_RX_NEGO_POWER_READY;
    sc9624_send_event(sc, &msg);
    return 0;
}

static int rx_backpower_mode_irq_handler(struct sc9624 *sc)
{
    struct wls_event_msg msg = {0x00};

    sc_info(":trigger\n");
    msg.event = WLS_EVENT_RX_BACKPOWER_MODE;
    sc9624_send_event(sc, &msg);
    return 0;
}

//tx
static int tx_ask_recv_irq_handler(struct sc9624 *sc)
{
    int ret;
    AskType ask_pkt;

    sc_info(":trigger\n");

    ret = sc9624_tx_recv_ask_pkt(sc, &ask_pkt);
    return ret;
}

static int tx_ppp_timeout_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_ppp_success_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_afc_det_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_epp_det_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_detect_rx_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_remove_power_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_fod_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_detect_tx_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_cep_timeout_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

static int tx_rpp_timeout_irq_handler(struct sc9624 *sc)
{
    sc_info(":trigger\n");
    return 0;
}

struct interrupt_handler {
	uint32_t bit_mask;
	int (*handler)(struct sc9624 *sc);
};

#define DECL_INTERRUPT_HANDLER(xbit, xhandler) {\
		.bit_mask = 1 << xbit,\
		.handler = xhandler, \
	}

static const struct interrupt_handler rx_irq_handlers[] = {
    DECL_INTERRUPT_HANDLER(0, ocp_irq_handler),
    DECL_INTERRUPT_HANDLER(1, vout_ovp_irq_handler),
    DECL_INTERRUPT_HANDLER(2, clamp_ovp_irq_handler),
    DECL_INTERRUPT_HANDLER(3, ngage_ovp_irq_handler),
    DECL_INTERRUPT_HANDLER(4, vout_lvp_irq_handler),
    DECL_INTERRUPT_HANDLER(5, otp_irq_handler),
    DECL_INTERRUPT_HANDLER(6, otp_160c_irq_handler),
    DECL_INTERRUPT_HANDLER(7, sleep_irq_handler),
    DECL_INTERRUPT_HANDLER(8, mode_change_irq_handler),
    DECL_INTERRUPT_HANDLER(9, rx_fsk_recv_irq_handler),
    DECL_INTERRUPT_HANDLER(10, rx_ppp_timeout_irq_handler),
    DECL_INTERRUPT_HANDLER(11, rx_ppp_success_irq_handler),
    DECL_INTERRUPT_HANDLER(12, rx_afc_det_irq_handler),
    DECL_INTERRUPT_HANDLER(13, rx_power_profile_irq_handler),
//    DECL_INTERRUPT_HANDLER(12, rx_epp_det_irq_handler),
    DECL_INTERRUPT_HANDLER(14, rx_poweron_irq_handler),
    DECL_INTERRUPT_HANDLER(15, rx_ss_pkt_irq_handler),
    DECL_INTERRUPT_HANDLER(16, rx_id_pkt_irq_handler),
    DECL_INTERRUPT_HANDLER(17, rx_config_pkt_irq_handler),
    DECL_INTERRUPT_HANDLER(18, rx_ready_irq_handler),
    DECL_INTERRUPT_HANDLER(19, rx_ldo_on_irq_handler),
    DECL_INTERRUPT_HANDLER(20, rx_ldo_off_irq_handler),
    DECL_INTERRUPT_HANDLER(21, rx_pldo_irq_handler),
    DECL_INTERRUPT_HANDLER(22, rx_scp_irq_handler),
    DECL_INTERRUPT_HANDLER(26, rx_hs_ok_irq_handler),
    DECL_INTERRUPT_HANDLER(27, rx_hs_fail_irq_handler),
    DECL_INTERRUPT_HANDLER(28, rx_nego_power_irq_handler),
    DECL_INTERRUPT_HANDLER(29, rx_backpower_mode_irq_handler),
};

static const struct interrupt_handler tx_irq_handlers[] = {
    DECL_INTERRUPT_HANDLER(0, ocp_irq_handler),
    DECL_INTERRUPT_HANDLER(1, vout_ovp_irq_handler),
    DECL_INTERRUPT_HANDLER(2, clamp_ovp_irq_handler),
    DECL_INTERRUPT_HANDLER(3, ngage_ovp_irq_handler),
    DECL_INTERRUPT_HANDLER(4, vout_lvp_irq_handler),
    DECL_INTERRUPT_HANDLER(5, otp_irq_handler),
    DECL_INTERRUPT_HANDLER(6, otp_160c_irq_handler),
    DECL_INTERRUPT_HANDLER(7, sleep_irq_handler),
    DECL_INTERRUPT_HANDLER(8, mode_change_irq_handler),
    DECL_INTERRUPT_HANDLER(9, tx_ask_recv_irq_handler),
    DECL_INTERRUPT_HANDLER(10, tx_ppp_timeout_irq_handler),
    DECL_INTERRUPT_HANDLER(11, tx_ppp_success_irq_handler),
    DECL_INTERRUPT_HANDLER(12, tx_afc_det_irq_handler),
    DECL_INTERRUPT_HANDLER(13, tx_epp_det_irq_handler),
    DECL_INTERRUPT_HANDLER(14, tx_detect_rx_irq_handler),
    DECL_INTERRUPT_HANDLER(15, tx_remove_power_irq_handler),
    DECL_INTERRUPT_HANDLER(16, tx_fod_irq_handler),
    DECL_INTERRUPT_HANDLER(17, tx_detect_tx_irq_handler),
    DECL_INTERRUPT_HANDLER(18, tx_cep_timeout_irq_handler),
    DECL_INTERRUPT_HANDLER(19, tx_rpp_timeout_irq_handler),
};

/*********************************************************************/
static ssize_t sc9624_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct sc9624 *sc = dev_get_drvdata(dev);
    int i = 0;
    uint32_t data;
    uint8_t tmpbuf[500];
    int len;
    int idx = 0;
    int ret;

    for (i = 0; i < 0x200 / 4; i++) {
        ret = sc9624_read_block(sc, i * 4, (uint8_t *)&data, 4);
        if (!ret)
        {
            len = snprintf(tmpbuf, PAGE_SIZE - idx,
                    "Reg[%04X] = 0x%08x\n", i * 4, data);
            memcpy(&buf[idx], tmpbuf, len);
            idx += len;
        }
    }

    return idx;
}

static ssize_t sc9624_store_register(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
    struct sc9624 *sc = dev_get_drvdata(dev);
    int ret;
    int reg;
    int len;
    uint32_t val;

    ret = sscanf(buf, "%x %x %x", &reg, &val, &len);
    if (ret == 3 && reg >= 0x0000 && reg <= 0x0200)
    {
        sc9624_write_block(sc, reg, (uint8_t *)&val, len);
    }

    return count;
}

static DEVICE_ATTR(registers, 0660, sc9624_show_registers,
        sc9624_store_register);

//-----------------------------reg addr----------------------------------
static ssize_t show_reg_addr(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sc9624 *sc = dev_get_drvdata(dev);
	return sprintf(buf, "reg addr 0x%08x\n", sc->reg_addr);
}

static ssize_t store_reg_addr(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc9624 *sc = dev_get_drvdata(dev);

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_addr = tmp;

	return count;
}
static DEVICE_ATTR(reg_addr, 0664, show_reg_addr, store_reg_addr);

//-----------------------------reg data----------------------------------
static ssize_t show_reg_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t data[4] = {0x00};
	struct sc9624 *sc = dev_get_drvdata(dev);

	ret = sc9624_read_block(sc, sc->reg_addr, data, 4);
	if (ret == 0)
		sc->reg_data = *((uint32_t *)data);
	return sprintf(buf, "reg addr 0x%08x -> 0x%08x\n", sc->reg_addr, sc->reg_data);
}

static ssize_t store_reg_data(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp;
	struct sc9624 *sc = dev_get_drvdata(dev);

	tmp = simple_strtoul(buf, NULL, 0);
	sc->reg_data = tmp;
	if (sc->reg_addr >= 0x0000 && sc->reg_addr <= 0x0200)
		sc9624_write_block(sc, sc->reg_addr, (uint8_t *)&sc->reg_data, 4);

	return count;
}
static DEVICE_ATTR(reg_data, 0664, show_reg_data, store_reg_data);

static ssize_t wireless_fw_update_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;
	struct sc9624 *sc = dev_get_drvdata(dev);

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	mtp_program(sc);

	return count;
}
static DEVICE_ATTR(wireless_fw_update, 0220, NULL, wireless_fw_update_store);

static ssize_t wireless_fw_erase_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	bool val;
	struct sc9624 *sc = dev_get_drvdata(dev);

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (val)
		mtp_erase(sc);

	return count;
}
static DEVICE_ATTR(wireless_fw_erase, 0220, NULL, wireless_fw_erase_store);


static void sc9624_create_device_node(struct device *dev)
{
    device_create_file(dev, &dev_attr_registers);
    device_create_file(dev, &dev_attr_reg_addr);
    device_create_file(dev, &dev_attr_reg_data);
    device_create_file(dev, &dev_attr_wireless_fw_update);
    device_create_file(dev, &dev_attr_wireless_fw_erase);
}

static const struct wireless_properties sc9624_wls_props = {
	.alias_name = "SC9624",
};

int sc9624_get_chip_id(struct wireless_device *wls_dev, int *chip_id)
{
	int rt = 0;
	uint16_t id = 0;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_get_chipid(sc, &id);
	pr_info("%s rt=%d chip_id=0x%04X\n", __func__, rt, id);
	if (!IS_ERR_OR_NULL(chip_id))
		*chip_id = id;

	return rt;
}

int sc9624_rx_get_fw_version(struct wireless_device *wls_dev, int *fw_version)
{
	int rt = 0;
	struct sc9624 *sc = NULL;
	uint32_t ver = 0;

	sc = dev_get_drvdata(&wls_dev->dev);

	rt = sc9624_get_fwver(sc, &ver);
	if (rt == 0 && !IS_ERR_OR_NULL(fw_version))
		*fw_version = ver ;

	return rt;
}

int sc9624_rx_get_rx_nego_power(struct wireless_device *wls_dev, int *power)
{
	uint32_t nego_power = 0;
	int rt = 0;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_get_nego_power(sc, &nego_power);

	if (!IS_ERR_OR_NULL(power))
		*power = nego_power ;

	return rt;
}

int sc9624_rx_get_op_mode(struct wireless_device *wls_dev, int *op_mode)
{
	int rt = 0;
	struct sc9624 *sc = NULL;
	SYSMODE sysmode = {0x00};

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_rx_get_sysmode(sc, &sysmode.value);

	if (rt == 0 && sysmode.RECEIVER) {
		if (sysmode.BPP_MODE) {
			*op_mode = Sys_Op_Mode_BPP;
		} else if (sysmode.EPP_MODE) {
			*op_mode = Sys_Op_Mode_EPP;
		}
	}

	return rt;
}

int sc9624_rx_get_sys_mode(struct wireless_device *wls_dev, int *sys_mode)
{
	int rt = 0;
	struct sc9624 *sc = NULL;
	SYSMODE sysmode = {0x00};
	sc = dev_get_drvdata(&wls_dev->dev);

	rt = sc9624_rx_get_sysmode(sc, &sysmode.value);
	if (rt) {
		return rt;
	}

	if (sysmode.RECEIVER) {
		*sys_mode = SYS_MODE_RX;
	} else if (sysmode.TRANSMITTER) {
		*sys_mode = SYS_MODE_TX;
	}

	return rt;
}

int sc9624_get_rx_irect(struct wireless_device *wls_dev, int *cur)
{
	int rt = 0;
	struct sc9624 *sc = NULL;
	uint32_t curr = 0;

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_get_current(sc, &curr);
	if (rt == 0 && !IS_ERR_OR_NULL(cur))
		*cur = curr ;

	return rt;
}

int sc9624_get_rx_vrect(struct wireless_device *wls_dev, int *voltage)
{
	int rt = 0;
	uint16_t vrect = 0;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_get_vrect(sc, &vrect);
	if (rt == 0 && !IS_ERR_OR_NULL(voltage))
		*voltage = vrect;

	return rt;
}

int sc9624_get_rx_vout(struct wireless_device *wls_dev, int *voltage)
{
	int rt = 0;
	uint32_t vout = 0;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_get_voltage(sc, &vout);
	if (rt == 0 && !IS_ERR_OR_NULL(voltage))
		*voltage = vout ;

	return rt;
}

int sc9624_get_rx_vout_setting(struct wireless_device *wls_dev, int *voltage)
{
	int rt = 0;
	uint32_t vout = 0;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_get_voltage_setting(sc, &vout);
	if (rt == 0 && !IS_ERR_OR_NULL(voltage))
		*voltage = vout ;

	return rt;
}

bool sc9624_check_ldo_on(struct wireless_device *wls_dev)
{
	uint32_t voltage = 0;
	int rt = 0;
	struct sc9624 *sc = NULL;
	SYSMODE sysmode = {0x00};

	sc = dev_get_drvdata(&wls_dev->dev);
	rt = sc9624_rx_get_sysmode(sc, &sysmode.value);
	if (rt == 0 && sysmode.RECEIVER) {
		rt = sc9624_get_voltage(sc, &voltage);
	}

	return (voltage > 4500);
}

int sc9624_send_ask_packet(struct wireless_device *wls_dev, uint8_t *data, int data_len)
{
	struct sc9624 *sc = NULL;
	AskType ask = {0x00};
	int rt = 0;
	int i = 0;

	sc = dev_get_drvdata(&wls_dev->dev);
	if (data_len > MAX_ASK_SIZE) {
		sc_err("data_len(%d) > MAX_ASK_SIZE\n", data_len);
		return -1;
	}

	pr_info("%s data_len=%d:", __func__, data_len);
	while (i < data_len) {
		pr_info("%02X ", data[i]);
		i ++;
	}
	pr_info("\n");
	memcpy(&ask.buf, data, data_len);
	if (0 == sc9624_rx_send_ask_pkt(sc, &ask)) {
		rt = data_len; //if send successed, need to return data len.
	}
	return rt;
}

int sc9624_set_mode_select(struct wireless_device *wls_dev, bool on)
{
	int rt = -1;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	if (gpio_is_valid(sc->wls_mode_select)) {
		gpio_set_value(sc->wls_mode_select, on);
		rt = 0;
	}

	sc_info("sc9624_wls_mode_select mode:%d rt:%d\n", on, rt);

	return rt;
}

int sc9624_get_mode_select(struct wireless_device *wls_dev, int *mode_sel)
{
	int rt = -1;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	if (gpio_is_valid(sc->wls_mode_select)) {
		*mode_sel = gpio_get_value(sc->wls_mode_select);
		if (*mode_sel >= 0) {
			rt = 0;
		}
	}

	return rt;
}

int sc9624_get_ce(struct wireless_device *wls_dev, int *ce)
{
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	return sc9624_get_rx_ce(sc, ce);
}

int sc9624_get_txinfo_mcode(struct wireless_device *wls_dev, uint16_t *mcode)
{
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	return sc9624_get_tx_mcode(sc, mcode);
}

int sc9624_set_mc_det(struct wireless_device *wls_dev, bool on)
{
	int rt = -1;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	if (gpio_is_valid(sc->wls_mc_det)) {
		gpio_set_value(sc->wls_mc_det, on);
		rt = 0;
	}

	sc_info("status:%d rt:%d\n", on, rt);

	return rt;
}

int sc9624_set_fw_update(struct wireless_device *wls_dev, bool on)
{
	int rt = -1;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	sc->fw_update_force = on;
	rt = mtp_program(sc);

	sc_info(" update:%d rt:%d\n", on, rt);

	return rt;
}

int sc9624_get_fw_update_status(struct wireless_device *wls_dev, int *status)
{
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);
	*status = sc->fw_update_status;
	sc_info(" status:%d\n", *status);

	return 0;
}

bool sc9624_dump_info(struct wireless_device *wls_dev)
{
	struct sc9624 *sc = NULL;
	int ret = -1;
	int ce = 0;
	uint16_t Vrect = 0;
	uint32_t Vout = 0;
	uint16_t Frequecy = 0;
	uint32_t Tdie = 0;
	SYSMODE sysmode = {0x00};

	sc = dev_get_drvdata(&wls_dev->dev);
	if (IS_ERR_OR_NULL(sc)) {
		sc_err("sc9624 is err or null\n");
		return false;
	}

	ret = sc9624_rx_get_sysmode(sc, &sysmode.value);
	if (ret) {
		sc_err("get_sysmode failed\n");
		return false;
	}

	if (!sysmode.RECEIVER) {
		sc_err("not in RX mode\n");
		return false;
	}

	ret = sc9624_get_rx_ce(sc, &ce);
	ret |= sc9624_get_vrect(sc, &Vrect);
	ret |= sc9624_get_voltage(sc, &Vout);
	ret |= sc9624_get_frequecy(sc, &Frequecy);
	ret |= sc9624_get_tdie(sc, &Tdie);

	sc_info("Vrect:%d,Vout:%d,Freq:%d,ce:%d,dieTemp:%d,ret:%d\n",
			Vrect, Vout, Frequecy, ce, Tdie, ret);

	return (ret == 0);
}

int sc9624_set_lowpower_mode(struct wireless_device *wls_dev, bool on)
{
	int ret = -1;
	struct sc9624 *sc = NULL;

	sc = dev_get_drvdata(&wls_dev->dev);

	if (on) {
		ret = sc9624_rx_set_cmd(sc, RX_LOWPOWER_MODE);
		if (ret) {
			sc_err("sc9624 set low power mode fail\n");
		}
	}
	sc_info(" %d ret:%d\n", on, ret);

	return ret;
}

int sc9624_send_event(struct sc9624 *sc, struct wls_event_msg *msg)
{
	int rt = -1;

	sc_info("event=%d\n", msg->event);
	if (IS_ERR_OR_NULL(sc->wls_dev) ||
		IS_ERR_OR_NULL(sc->wls_dev->callback_ops)||
		IS_ERR_OR_NULL(sc->wls_dev->callback_ops->event_handler)) {
		return rt;
	}

	mutex_lock(&sc->event_lock);
	rt = sc->wls_dev->callback_ops->event_handler(sc->wls_dev, msg);
	mutex_unlock(&sc->event_lock);

	return rt;
}

static int sc9624_wlc2_init(struct sc9624 *sc)
{
	int ret = 0;

	sc->rx_ops.get_chip_id = sc9624_get_chip_id;
	sc->rx_ops.get_fw_version = sc9624_rx_get_fw_version;
	sc->rx_ops.get_op_mode = sc9624_rx_get_op_mode;
	sc->rx_ops.get_sys_mode = sc9624_rx_get_sys_mode;
	sc->rx_ops.get_rx_neg_power = sc9624_rx_get_rx_nego_power;
	sc->rx_ops.get_rx_irect = sc9624_get_rx_irect;
	sc->rx_ops.get_rx_iout = sc9624_get_rx_irect;
	sc->rx_ops.get_rx_vrect = sc9624_get_rx_vrect;
	sc->rx_ops.get_rx_vout = sc9624_get_rx_vout;
	sc->rx_ops.get_rx_vout_setting = sc9624_get_rx_vout_setting;
	sc->rx_ops.check_ldo_on = sc9624_check_ldo_on;
	sc->rx_ops.send_ask_packet = sc9624_send_ask_packet;
	sc->rx_ops.set_mode_select = sc9624_set_mode_select;
	sc->rx_ops.get_mode_select = sc9624_get_mode_select;
	sc->rx_ops.set_fw_update = sc9624_set_fw_update;
	sc->rx_ops.get_fw_update_status = sc9624_get_fw_update_status;
	sc->rx_ops.set_mc_det = sc9624_set_mc_det;
	sc->rx_ops.get_ce = sc9624_get_ce;
	sc->rx_ops.get_mcode = sc9624_get_txinfo_mcode;
	sc->rx_ops.check_dump_info = sc9624_dump_info;
    sc->rx_ops.set_lowpower_mode = sc9624_set_lowpower_mode;

	sc->wls_dev = wireless_device_register("moto_wlc2",
							&sc->client->dev, (void*)sc,
							&sc->rx_ops,
							NULL,
							&sc9624_wls_props);
	if (IS_ERR_OR_NULL(sc->wls_dev)) {
		ret = PTR_ERR(sc->wls_dev);
		sc_err("failed to register battery: %d\n", ret);
	} else {
		sc_info("sc->wls_dev=%p\n", sc->wls_dev);
	}

	return ret;
}

static enum power_supply_property sc9624_charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
    //POWER_SUPPLY_PROP_SC_VOLTAGE,
    //POWER_SUPPLY_PROP_SC_CURRENT,
    //POWER_SUPPLY_PROP_SC_ILIMIT,
    //POWER_SUPPLY_PROP_SC_VRECT,
    //POWER_SUPPLY_PROP_SC_TX_ENABLE,
    //POWER_SUPPLY_PROP_SC_TX_FOD_ENABLE,
    //POWER_SUPPLY_PROP_SC_WORK_MODE,
    //POWER_SUPPLY_PROP_SC_MTP_PROGRAM,
};

static int sc9624_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
    struct sc9624 *sc = power_supply_get_drvdata(psy);
    int ret;
    uint32_t regval;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        ret = sc9624_online(sc);
        if (ret) {
            val->intval = 0;
        } else {
            val->intval = 1;
        }
        sc_info("online:%d wls_det_gpio:%d\n",
                ret, gpio_get_value(sc->wls_det_gpio));
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc9624_get_voltage(sc, &regval);
        if (!ret) {
            val->intval = regval;
        }
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sc9624_get_current(sc, &regval);
        if (!ret) {
            val->intval = regval;
        }
        break;
#if 0
    case POWER_SUPPLY_PROP_SC_VRECT:
        ret = sc9624_get_vrect(sc, &regval);
        if (!ret) {
            val->intval = regval;
        }
        break;
    case POWER_SUPPLY_PROP_SC_ILIMIT:
    case POWER_SUPPLY_PROP_SC_TX_ENABLE:
    case POWER_SUPPLY_PROP_SC_TX_FOD_ENABLE:
    case POWER_SUPPLY_PROP_SC_MTP_PROGRAM:
        val->intval = 0;
        break;
    case POWER_SUPPLY_PROP_SC_WORK_MODE:
        val->intval = sc->work_mode;
        break;
#endif
    default:
        return -EINVAL;

    }
    return 0;
}


static int sc9624_charger_set_property(struct power_supply *psy,
                        enum power_supply_property prop,
                        const union power_supply_propval *val)
{
    struct sc9624 *sc = power_supply_get_drvdata(psy);
    int ret = 0;

    switch (prop) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sc9624_rx_set_Vout(sc, val->intval);
        break;
    case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
        ret = sc9624_rx_set_Ilimit(sc, val->intval);
    break;
#if 0
    case POWER_SUPPLY_PROP_SC_TX_ENABLE:
        ret = sc9624_tx_func_enable(sc);
    break;
    case POWER_SUPPLY_PROP_SC_TX_FOD_ENABLE:
        ret = sc9624_tx_fod_enable(sc, val->intval);
    break;
    case POWER_SUPPLY_PROP_SC_WORK_MODE:
        sc->work_mode = val->intval;
    break;
    case POWER_SUPPLY_PROP_SC_MTP_PROGRAM:
        ret = mtp_program(sc);
    break;
#endif
    default:
        return -EINVAL;
    }

    return ret;
}


static int sc9624_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
    int ret;

    switch (prop) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
    case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
        ret = 1;
        break;
    default:
        ret = 0;
        break;
    }
    return ret;
}

static int sc9624_psy_register(struct sc9624 *sc)
{
    sc->psy_cfg.drv_data = sc;
    sc->psy_cfg.of_node = sc->dev->of_node;

    sc->psy_desc.name = "wireless-sc9624";
    sc->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
    sc->psy_desc.properties = sc9624_charger_props;
    sc->psy_desc.num_properties = ARRAY_SIZE(sc9624_charger_props);
    sc->psy_desc.get_property = sc9624_charger_get_property;
    sc->psy_desc.set_property = sc9624_charger_set_property;
    sc->psy_desc.property_is_writeable = sc9624_charger_is_writeable;

    sc->wl_psy = devm_power_supply_register(sc->dev,
            &sc->psy_desc, &sc->psy_cfg);
    if (IS_ERR(sc->wl_psy)) {
        sc_err("failed to register wl_psy\n");
        return PTR_ERR(sc->wl_psy);
    }

    sc_info("%s power supply register successfully\n", sc->psy_desc.name);

    return 0;
}

static int sc9624_parse_dt(struct sc9624 *sc, struct device *dev)
{
    int ret = 0;
    struct device_node *np = dev->of_node;

    if (!np)
        return -ENOMEM;

    ret = of_get_named_gpio(np, "wls-int", 0);
    if (ret < 0) {
        sc_err("no wls-int gpio info\n");
        return ret;
    }
    sc->irq_gpio = ret;

    ret = of_get_named_gpio(np, "wls-det-int", 0);
    if (ret < 0) {
        sc_err("no wl-det-int gpio info\n");
        return ret;
    }
    sc->wls_det_gpio = ret;
    sc_info("irq_gpio=%d, wls_det_gpio=%d\n", sc->irq_gpio, sc->wls_det_gpio);

    sc->wls_mode_select = of_get_named_gpio(np, "wls-mode-select", 0);
    if (!gpio_is_valid(sc->wls_mode_select)) {
        sc_err("wls_mode_select(%d) is invalid\n", sc->wls_mode_select);
    }

    sc->wls_mc_det = of_get_named_gpio(np, "wls-mc-det", 0);
    if (!gpio_is_valid(sc->wls_mc_det)) {
        sc_info("wls_mc_det(%d) is invalid\n", sc->wls_mc_det);
    }

    of_property_read_string(np, "wireless-fw-name", &sc->wls_fw_name);
    sc_info("wls_fw_name: %s\n", sc->wls_fw_name);

    return 0;
}

/*
* interrupt does nothing, just info event chagne, other module could get info
* through power supply interface
*/
static void sc9624_irq_work_handler(struct kthread_work *work)
{
    struct sc9624 *sc =
            container_of(work, struct sc9624, irq_work);
    int ret;
    int i;
    uint32_t regval;

    /* make sure I2C bus had resumed */
    down(&sc->suspend_lock);
    ret = sc9624_rx_get_sysmode(sc, &sc->sys_mode.value);
    if (ret) {
        sc_err("Can't get sys mode\n");
        up(&sc->suspend_lock);
        return;
    }
    sc_info("irq trigger mode RX:%d TX:%d STANDBY:%d\n",
            sc->sys_mode.RECEIVER,
            sc->sys_mode.TRANSMITTER,
            sc->sys_mode.STANDBY);

    if (sc->sys_mode.RECEIVER || sc->sys_mode.STANDBY) {
        ret = sc9624_rx_get_int(sc, &regval);
        if (ret < 0) {
            sc_err("get rx int flag fail\n");
            up(&sc->suspend_lock);
            return;
        }
        sc_info("get rx int flag:0x%08X\n", regval);
        //clear intflag
        ret = sc9624_rx_clr_int(sc, regval);
        for (i = 0; i < ARRAY_SIZE(rx_irq_handlers); i++) {
            if (rx_irq_handlers[i].bit_mask & regval) {
                if (rx_irq_handlers[i].handler != 0)
                    rx_irq_handlers[i].handler(sc);
            }
        }
    } else if (sc->sys_mode.TRANSMITTER){
        ret = sc9624_tx_get_int(sc, &regval);
        if (ret < 0) {
            sc_err("get tx int flag fail\n");
            up(&sc->suspend_lock);
            return;
        }

        for (i = 0; i < ARRAY_SIZE(tx_irq_handlers); i++) {
                if (tx_irq_handlers[i].bit_mask & regval) {
                    if (tx_irq_handlers[i].handler != 0)
                        tx_irq_handlers[i].handler(sc);
            }
        }

        //clear intflag
        ret = sc9624_tx_clr_int(sc, regval);
    }

    up(&sc->suspend_lock);

    power_supply_changed(sc->wl_psy);
}

static irqreturn_t sc9624_intr_handler(int irq, void *dev_id)
{
    struct sc9624 *sc = dev_id;

    __pm_wakeup_event(sc->irq_wake_lock, SC9624_IRQ_WAKE_TIME);

    kthread_queue_work(&sc->irq_worker, &sc->irq_work);

    return IRQ_HANDLED;
}

static int sc9624_irq_init(struct sc9624 *sc)
{
    int ret;
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

    ret = devm_gpio_request(sc->dev, sc->irq_gpio, "sc9624-irq-gpio");
    if (ret < 0) {
        sc_err("Error: failed to request GPIO%d (ret = %d)\n",
        sc->irq_gpio, ret);
        return -EINVAL;
    }

    ret = gpio_direction_input(sc->irq_gpio);
    if (ret < 0) {
        sc_err("Error: failed to set GPIO%d as input pin(ret = %d)\n",
        sc->irq_gpio, ret);
        return -EINVAL;
    }

    sc->irq = gpio_to_irq(sc->irq_gpio);
    if (sc->irq <= 0) {
        sc_err("gpio to irq fail, chip->irq(%d)\n",
                        sc->irq);
        return -EINVAL;
    }

    sc_info(" : IRQ number = %d\n", sc->irq);

    kthread_init_worker(&sc->irq_worker);
    sc->irq_worker_task = kthread_run(kthread_worker_fn,
            &sc->irq_worker, "%s", "sc9624_irq_thread");
    if (IS_ERR(sc->irq_worker_task)) {
        sc_err("Error: Could not create tcpc task\n");
        return -EINVAL;
    }

    sched_setscheduler(sc->irq_worker_task, SCHED_FIFO, &param);
    kthread_init_work(&sc->irq_work, sc9624_irq_work_handler);

    ret = request_irq(sc->irq, sc9624_intr_handler,
        IRQF_TRIGGER_FALLING | IRQF_NO_THREAD, "sc9624_irq", sc);
    if (ret < 0) {
        sc_err("Error: failed to request irq%d (gpio = %d, ret = %d)\n",
            sc->irq, sc->irq_gpio, ret);
        return -EINVAL;
    }

    enable_irq_wake(sc->irq);
    return 0;
}

static void sc9624_wls_det_work_handler(struct kthread_work *work)
{
    struct sc9624 *sc =
            container_of(work, struct sc9624, wls_det_work);
    struct wls_event_msg msg = {0x00};

    /* make sure I2C bus had resumed */
    down(&sc->wls_det_lock);
    msg.event = WLS_EVENT_TX_DETECTED;
    msg.data[0] = gpio_get_value(sc->wls_det_gpio);
    msg.len = 1;
    sc_info("wls_det_gpio:%d\n", msg.data[0]);
    sc9624_send_event(sc, &msg);
    up(&sc->wls_det_lock);
    power_supply_changed(sc->wl_psy);
}

static irqreturn_t sc9624_wls_det_handler(int irq, void *dev_id)
{
    struct sc9624 *sc = dev_id;

    sc_info("");
    __pm_wakeup_event(sc->wls_det_wake_lock, SC9624_IRQ_WAKE_TIME);
    kthread_queue_work(&sc->wls_det_worker, &sc->wls_det_work);

    return IRQ_HANDLED;
}

static int sc9624_wls_det_irq_init(struct sc9624 *sc)
{
    int ret;
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

    if (!gpio_is_valid(sc->wls_det_gpio)) {
        sc_err("Error: wls_det_gpio(%d) is invalid\n", sc->wls_det_gpio);
        return -EINVAL;
    }

    sema_init(&sc->wls_det_lock, 1);

    ret = devm_gpio_request(sc->dev, sc->wls_det_gpio, "sc9624-wls_det-gpio");
    if (ret < 0) {
        sc_err("Error: failed to request GPIO%d (ret = %d)\n",
            sc->wls_det_gpio, ret);
        return -EINVAL;
    }

    ret = gpio_direction_input(sc->wls_det_gpio);
    if (ret < 0) {
        sc_err("Error: failed to set GPIO%d as input pin(ret = %d)\n",
            sc->wls_det_gpio, ret);
        return -EINVAL;
    }

    sc->wls_det_irq = gpio_to_irq(sc->wls_det_gpio);
    if (sc->wls_det_irq < 0) {
        sc_err("failed get wls_det irq num %d", sc->wls_det_irq);
        return -EINVAL;
    }

    sc_info(" : IRQ number = %d\n", sc->wls_det_irq);

    kthread_init_worker(&sc->wls_det_worker);
    sc->wls_det_worker_task = kthread_run(kthread_worker_fn,
            &sc->wls_det_worker, "%s", "sc9624_wls_det_thread");
    if (IS_ERR(sc->wls_det_worker_task)) {
        sc_err("Error: Could not create wls_det task\n");
        return -EINVAL;
    }

    sched_setscheduler(sc->wls_det_worker_task, SCHED_FIFO, &param);
    kthread_init_work(&sc->wls_det_work, sc9624_wls_det_work_handler);

    ret = request_irq(sc->wls_det_irq, sc9624_wls_det_handler,
        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "sc9624_wls_det_irq", sc);
    if (ret < 0) {
        sc_err("Error: failed to request wls_det irq%d (gpio = %d, ret = %d)\n",
            sc->wls_det_irq, sc->wls_det_gpio, ret);
        return -EINVAL;
    }

    enable_irq_wake(sc->wls_det_irq);
    return 0;
}

static struct of_device_id sc9624_charger_match_table[] = {
    {
        .compatible = "sc,sc9624-wireless-charger",
        .data = NULL,
    },
    {},
};

static int sc9624_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
    struct sc9624 *sc;
    int ret;

    pr_err("%s start\n", __func__);

    sc = devm_kzalloc(&client->dev, sizeof(struct sc9624), GFP_KERNEL);
    if (!sc)
        return -ENOMEM;

    sc->dev = &client->dev;
    sc->client = client;


    mutex_init(&sc->i2c_rw_lock);
    mutex_init(&sc->data_lock);
    mutex_init(&sc->event_lock);
    sema_init(&sc->suspend_lock, 1);

    i2c_set_clientdata(client, sc);

    sc->regmap = devm_regmap_init_i2c(client, &sc9624_regmap_config);

    sc9624_create_device_node(&(client->dev));

    sc->irq_wake_lock =
        wakeup_source_register(sc->dev, "sc9624_irq_wake_lock");

    sc->wls_det_wake_lock =
        wakeup_source_register(sc->dev, "sc9624_wls_det_wake_lock");

    ret = sc9624_parse_dt(sc, &client->dev);
    if (ret)
        return -EIO;

    ret = sc9624_psy_register(sc);
    if (ret)
        goto err_1;

    ret = sc9624_irq_init(sc);
    ret = sc9624_wls_det_irq_init(sc);

    device_init_wakeup(sc->dev, 1);

    sc9624_wlc2_init(sc);
    sc_info("sc9624 probe successfully!\n");

    return 0;

err_1:
    wakeup_source_unregister(sc->wls_det_wake_lock);
    wakeup_source_unregister(sc->irq_wake_lock);
    power_supply_unregister(sc->wl_psy);
    return ret;
}

static int sc9624_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc9624 *sc = i2c_get_clientdata(client);

    if (sc)
        down(&sc->suspend_lock);

    sc_err("Suspend successfully!");

    return 0;
}

static int sc9624_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sc9624 *sc = i2c_get_clientdata(client);

    if (sc)
        up(&sc->suspend_lock);
	
    sc_err("Resume successfully!");

    return 0;
}
static void sc9624_charger_remove(struct i2c_client *client)
{
    struct sc9624 *sc = i2c_get_clientdata(client);

    power_supply_unregister(sc->wl_psy);

    mutex_destroy(&sc->data_lock);
    mutex_destroy(&sc->i2c_rw_lock);
}

static void sc9624_charger_shutdown(struct i2c_client *client)
{
    //struct sc9624 *sc = i2c_get_clientdata(client);
    return;
}

static const struct dev_pm_ops sc9624_pm_ops = {
    .resume     = sc9624_resume,
    .suspend    = sc9624_suspend,
};

static const struct i2c_device_id sc9624_charger_id[] = {
    {"sc9624-wls-chg", 0},
    {},
};

static struct i2c_driver sc9624_wireless_charger_driver = {
    .driver     = {
        .name   = "sc-wireless-charger",
        .owner  = THIS_MODULE,
        .of_match_table = sc9624_charger_match_table,
        .pm     = &sc9624_pm_ops,
    },
    .id_table   = sc9624_charger_id,

    .probe      = sc9624_charger_probe,
    .remove     = sc9624_charger_remove,
    .shutdown   = sc9624_charger_shutdown,
};

module_i2c_driver(sc9624_wireless_charger_driver);

MODULE_DESCRIPTION("SC SC9624 Wireless Charge Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aiden-yu@southchip.com");
