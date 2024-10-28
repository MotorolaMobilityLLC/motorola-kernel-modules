#ifndef __GH_CUSTOMER_H
#define __GH_CUSTOMER_H


typedef struct power_cofing {
	int load_uA;
	int min_uV;
	int max_uV;
} gh_power_cfg;

/*
* for dts config
*/
#define GH_COMPATILBE        "goodix,health"
#define GH_GPIO_RESET        "goodix,gpio_reset"
#define GH_GPIO_IRQ          "goodix,gpio_irq"
#define GH_POWER_VDD_LEDS    "goodix,gh_vdd_leds" //5V leds power
#define GH_POWER_VDD         "goodix,gh_vdd"     //3.3V g_vdd_cfg
#define GH_POWER_VDD_IO      "goodix,gh_vdd_io"  //1.8V g_vdd_io_cfg

/*
* need defined in include/uapi/linux/netlink.h
* cat /proc/net/netlink, and set a unused value
* notice : the value is lower then MAX_LINKS
*/
#define GH_NETLINK_ROUTE    (25)

/*
* choose SPI or I2C
*/
#define GH_SUPPORT_BUS_I2C  (0)
#define GH_SUPPORT_BUS_SPI  (1)
#define GH_SUPPORT_BUS      (GH_SUPPORT_BUS_I2C)


/*
* 0 : local test
* 1 : for customer project
*/
#define GH_CUSTOMIZATION_POWER   (1)

/*
* for vdd and vdd_io config
*/
extern gh_power_cfg g_vdd_cfg;
extern gh_power_cfg g_vdd_io_cfg;

#endif	/* __GH_CUSTOMER_H */
