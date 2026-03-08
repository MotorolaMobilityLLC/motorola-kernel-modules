#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define EBUD_CMDSET_COUNT                    7

#define EBUD_CMD_PARING                      0x01
#define EBUD_CMD_PARING_GET_STS              0x02
#define EBUD_CMD_INQUIRY_LID_OPEN            0x03
#define EBUD_CMD_INQUIRY_LID_CLOSE           0x04
#define EBUD_CMD_INQUIRY_LID_OPEN_PARING     0x05
#define EBUD_CMD_INQUIRY_LID_CLOSE_PARING    0x06
#define EBUD_CMD_SHUTDOWN                    0x07
#define EBUD_GPIO_DISABLE                    0
#define EBUD_GPIO_ENABLE                     1

#define EBUD_CASE_CLOSED                     0
#define EBUD_CASE_OPENED                     1

enum charging_state {
    NO_CHG,
    L_CHG,
    R_CHG,
    LR_CHG,
};

enum chip_idx {
    LEFT,
    RIGHT,
};

typedef struct earbud_cmd {
    unsigned char cmdbuf[16];
    unsigned char size;
}S_EBUD_CMD;

struct mmi_ichg_chip {
    int (*read_ichg)(void *pdata, int chanindex, int *out_val);
    void *pdata;
};

struct mmi_earbud_chg_data {
    int enable_chg;  /* enable charging */
    int enable_lchg; /* enable left earbud charging */
    int enable_rchg; /* enable right earbud charging*/

    int en_chg_gpio;
    int en_lchg_gpio;
    int en_rchg_gpio;
    struct gpio_desc *chg_irq_gpio;
    int chg_irq;
    const char *dev_lchan;
    const char *dev_rchan;

    int termination_current;
    enum charging_state chg_state;
    int hb_interval; /* in milliseconds */
    int user_opt;
    struct mutex lock;
    struct delayed_work heartbeat_work;

    struct mmi_ichg_chip *ichg_chip;
    struct device *dev;
};

struct ebud_tlv {
   unsigned char buf[64];
   unsigned int size;
};

static inline void mmi_earbud_uart_enable(struct mmi_earbud_chg_data *pdata, bool enable) {
    gpio_set_value(pdata->en_chg_gpio, enable);
    pr_debug("%s: Get GPIO_VLAUE %d", __func__, gpio_get_value(pdata->en_chg_gpio));
}

static inline void mmi_earbud_lchg_enable(struct mmi_earbud_chg_data *pdata, bool enable) {
    gpio_set_value(pdata->en_lchg_gpio, enable);
    pr_debug("%s: Get LCHG GPIO_VLAUE %d", __func__, gpio_get_value(pdata->en_lchg_gpio));
}
static inline void mmi_earbud_rchg_enable(struct mmi_earbud_chg_data *pdata, bool enable) {
    gpio_set_value(pdata->en_rchg_gpio, enable);
    pr_debug("%s: Get RCHG GPIO_VLAUE %d", __func__, gpio_get_value(pdata->en_rchg_gpio));
}

static inline void mmi_earbud_get_irq_gpio_state(struct mmi_earbud_chg_data *pdata, int *val) {
    *val =  gpiod_get_value(pdata->chg_irq_gpio);
    pr_debug("%s: Get Charging irq GPIO_VLAUE %d", __func__, *val);
}

int mmi_uart_setup(struct mmi_earbud_chg_data *pdata, unsigned int baudrate);
int mmi_uart_tx(struct mmi_earbud_chg_data *pdata, int cmd, int channel);
int mmi_uart_rx(struct mmi_earbud_chg_data *pdata, char *recv, int *size);
