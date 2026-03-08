#include <linux/fs.h>
#include <asm/termbits.h>
#include <asm-generic/ioctls.h>
#include "mmi_earbud.h"
#include <linux/compat.h>

#define UART_ENABLE_WAITTIME 1000 //1 second
#define UART_DISABLE_WAITTIME 100 //100ms

#define CMD_WRITE      _IOWR('M', 1, struct ebud_tlv)
#define CMD_READ       _IOW('M', 1, struct ebud_tlv)

S_EBUD_CMD left_cmdset[EBUD_CMDSET_COUNT] = {
    // earphone binary parring instructio
    { {0x24, 0x01, 0x08, 0xD7, 0x53, 0xCE, 0x50, 0x84, 0x54, 0x41, 0x00, 0x2A, 0xFB}, 13 },
    // Check paring status of ear phone, s3-s7
    { {0x24, 0x02, 0x01, 0x41, 0x7C, 0xCB}, 6},
     // Battery inquery instruction on open
    { {0x24, 0x06, 0x02, 0x1E, 0x59, 0x32, 0xE7}, 7},
    // Battery inquery instruction on close
    { {0x24, 0x06, 0x02, 0x9E, 0x59, 0x29, 0x7F}, 7},
    // Battery inquery instruction on open parring command S1
    { {0x24, 0x06, 0x02, 0x1E, 0x41, 0xA1, 0xDE}, 7},
    // Battery inquery instruction on closed parring command S9
    { {0x24, 0x06, 0x02, 0x9E, 0x41, 0xBA, 0x46}, 7},
    // Shutdown command
    { {0x24, 0x05, 0x00, 0x69, 0x6F}, 5}
};

S_EBUD_CMD right_cmdset[EBUD_CMDSET_COUNT] = {
    // earphone binary parring instruction, on S2
    { {0x25, 0x01, 0x08, 0xD1, 0x47, 0xCE, 0x50, 0x84, 0x54, 0x41, 0x00, 0x84, 0xC4}, 13 },
    // Check paring status of ear phone, s3-s7
    { {0x25, 0x02, 0x01, 0x41, 0x0A, 0x7F}, 6},
     // Battery inquery instruction on open
    { {0x25, 0x06, 0x02, 0x1E, 0x59, 0x98, 0xB6}, 7},
    // Battery inquery instruction on close
    { {0x25, 0x06, 0x02, 0x9E, 0x59, 0x83, 0x2E}, 7},
    // Battery inquery instruction on open parring command S1
    { {0x25, 0x06, 0x02, 0x1E, 0x41, 0x0B, 0x8F}, 7},
    // Battery inquery instruction on closed parring command S9
    { {0x25, 0x06, 0x02, 0x9E, 0x41, 0x10, 0x17}, 7},
    // Shutdown command
    { {0x25, 0x05, 0x00, 0x5E, 0x5F}, 5}
};

extern struct ebud_tlv rx_data;

int mmi_uart_setup(struct mmi_earbud_chg_data *pdata, unsigned int baudrate) {
    return 0;
}

void mmi_uart_enable(struct mmi_earbud_chg_data *pdata, int channel, bool enable)
{
    if (channel == LEFT) {
        if(enable) {
            mmi_earbud_lchg_enable(pdata, EBUD_GPIO_DISABLE);
            msleep(UART_ENABLE_WAITTIME);
            mmi_earbud_lchg_enable(pdata, EBUD_GPIO_ENABLE);
            msleep(UART_ENABLE_WAITTIME);
            mmi_earbud_lchg_enable(pdata, EBUD_GPIO_DISABLE);
        } else mmi_earbud_lchg_enable(pdata, pdata->enable_lchg);
        msleep(500);
        mmi_earbud_uart_enable(pdata, enable);
        msleep(UART_DISABLE_WAITTIME);
    } else if (channel == RIGHT) {
        if(enable) {
            mmi_earbud_rchg_enable(pdata, EBUD_GPIO_DISABLE);
            msleep(UART_ENABLE_WAITTIME);
            mmi_earbud_rchg_enable(pdata, EBUD_GPIO_ENABLE);
            msleep(UART_ENABLE_WAITTIME);
            mmi_earbud_rchg_enable(pdata, EBUD_GPIO_DISABLE);
        } else mmi_earbud_rchg_enable(pdata, pdata->enable_rchg);
        msleep(500);
        mmi_earbud_uart_enable(pdata, enable);
        msleep(UART_DISABLE_WAITTIME);
    }
}

int mmi_uart_tx(struct mmi_earbud_chg_data *pdata, int cmd, int channel) {
    struct file *file;
    loff_t pos = 0;
    char devnode[32];
    int ret = 0, cnt = 0; //, retry = 0;
    //ssize_t wsize = 0;
    pr_info("%s: Entered", __func__);
    struct ebud_tlv data = {0};

    if((cmd < EBUD_CMD_PARING) || (cmd > EBUD_CMDSET_COUNT))
        return -EINVAL;

    mmi_uart_enable(pdata, channel, EBUD_GPIO_ENABLE);
    if(channel == LEFT) strscpy(devnode, pdata->dev_lchan, 32);
    else if(channel == RIGHT) strscpy(devnode, pdata->dev_rchan, 32);
    //Clear the global buffer
    memset(&rx_data, 0, sizeof(struct ebud_tlv));
    //file = filp_open(devnode, O_WRONLY|O_NOCTTY|O_NONBLOCK, 0);
    file = filp_open(devnode, O_RDWR, 0644);
    if (!IS_ERR(file)) {
	 mdelay(500);
         if(channel == LEFT) {
	         data.size = left_cmdset[cmd-1].size;
	         memcpy(data.buf, left_cmdset[cmd-1].cmdbuf, data.size);
                 // Call the driver's ioctl function directly
                 ret = file->f_op->unlocked_ioctl(file, CMD_WRITE, (unsigned long)&data);
		 pr_info("%s: Received data LChannel : ", __func__);
		 for(cnt = 0; cnt < data.size; cnt++) pr_info("0x%X ", data.buf[cnt]);
	 }
         else if(channel == RIGHT) {
                 data.size = right_cmdset[cmd-1].size;
                 memcpy(data.buf, right_cmdset[cmd-1].cmdbuf, data.size);
                 // Call the driver's ioctl function directly
                 ret = file->f_op->unlocked_ioctl(file, CMD_WRITE, (unsigned long)&data);
		 pr_info("%s: Received data RChannel : ", __func__);
                 for(cnt = 0; cnt < data.size; cnt++) pr_info("0x%X ", data.buf[cnt]);
	 }
         pos = 0;
         if(ret > 0) {
              memcpy(rx_data.buf, data.buf, data.size);
              rx_data.size = data.size;
         }

	 mdelay(500);
         filp_close(file, NULL);
    } else {
         pr_info("%s: File %s open failed", __func__, devnode);
         ret = -1;
    }
    msleep(UART_ENABLE_WAITTIME);
    mmi_uart_enable(pdata, channel, EBUD_GPIO_DISABLE);
    msleep(UART_ENABLE_WAITTIME);
    pr_info("%s: Exit", __func__);
    return ret;
}

int mmi_uart_rx(struct mmi_earbud_chg_data *pdata, char *recv, int *size) {
    *size = 0;
    if(rx_data.size > 0) {
         memcpy(recv, rx_data.buf, rx_data.size);
         *size = rx_data.size;
    }
    return 0;
}
