//
// Description: I2C Adapter for Boreas Haptic Driver on Linux Kernel
//
// Copyright (c) 2020 Boreas Technologies All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#define pr_fmt(fmt) "bos0614: %s: " fmt, __func__

#define DEBUG
//#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "i2cLinux.h"

#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_err
#undef dev_dbg
#define dev_dbg dev_err
#endif

typedef struct
{
    I2c driver;
    struct i2c_client *client;
} Context;

static void initDriverFct(Context *ctx);

#define NBR_CHAR_PER_BYTE (5)
#define ESCAPE_CHAR (1)

int process_params(ParamsLst *params, int numP, const char *buffer)
{
	char *arg, *buf, *p;
	int n, err;
	unsigned int value;

	p = buf = kstrdup(buffer, GFP_KERNEL);
	for (n = 0; n < numP && p && *p; n++, params++) {
		arg = strsep(&p, " ");
		if (!arg || !*arg)
			break;

		pr_debug("arg=[%s] rest=[%s]\n", arg, p);

		switch (params->tid) {
		case PARAM_UCHAR8:
			err = kstrtou8(arg, 0, params->data_ptr);
			value = (unsigned int)*(unsigned char *)params->data_ptr;
				break;
		case PARAM_INT16:
			err = kstrtos16(arg, 0, params->data_ptr);
			value = (unsigned int)*(short *)params->data_ptr;
				break;
		case PARAM_UINT16:
			err = kstrtou16(arg, 0, params->data_ptr);
			value = (unsigned int)*(unsigned short *)params->data_ptr;
				break;
		case PARAM_INT32:
			err = kstrtoint(arg, 0, params->data_ptr);
			value = (unsigned int)*(int *)params->data_ptr;
				break;
		case PARAM_UINT32:
			err = kstrtouint(arg, 0, params->data_ptr);
			value = *(unsigned int *)params->data_ptr;
				break;
		}

		if (err) {
			n = err;
			break;
		} else
			pr_debug("[%d]=%u\n", n, value);
	}
	kfree(buf);
	pr_debug("processed %d input parameters\n", n);
	return n;
}
#if 0
static int readRegsConfig(Context *ctx, struct device_node *parent, const char *suffix)
{
	Bos0614RegisterStruct *config = getAllRegsPtr();
	struct device_node *node;
	char node_name[64];
	u32 *temp, length = 0;
	int npairs, i, index, ret = -EIO;

	scnprintf(node_name, 63, "config-%s", suffix);
	node = of_find_node_by_name(parent, node_name);
	if (!node)
		return -ENODEV;

	if (!of_find_property(node, "config-data", &length)) {
		dev_err(&ctx->client->dev, "(config-%s) prop config-data not found\n", suffix);
		goto out;
	}

	npairs = length / 2;
	dev_info(&ctx->client->dev, "(config-%s) array size %d\n", suffix, npairs);

	temp = kzalloc(length, GFP_KERNEL);
	if (!temp)
		goto out;

	ret = of_property_read_u32_array(node, "config-data", temp, sizeof(u32) * npairs * 2);
	if (ret) {
		dev_err(&ctx->client->dev, "error reading config-data (config-%s)\n", suffix);
		goto release_mem;
	}

	for (i = 0; i < npairs; i++) {
		index =(uint16_t)*temp++;
		config[index].value = (uint16_t)*temp++;
		dev_info(&ctx->client->dev, "[%d] addr=0x%02x, val=0x%04x\n",
				i, config[index].addr, config[index].value);
	}

release_mem:
	kfree(temp);
out:
	of_node_put(node);

	return ret;
}

static int readDevTree(Context *ctx)
{
	int ret, verno;
	struct device_node *np = ctx->client->dev.of_node;
	struct device_node *config_np;

	config_np = of_find_node_by_name(np, "configs");
	if (!config_np) {
		dev_info(&ctx->client->dev, "does not support configs\n");
		return 0;
	}

	if (!of_property_read_u32(config_np, "config-ver", &verno))
		dev_info(&ctx->client->dev, "dt config Rev.%u\n", verno);

	ret = readRegsConfig(ctx, config_np, "default");
	if (ret > 0)
		dev_info(&ctx->client->dev, "has default config\n");

	of_node_put(config_np);

	return 0;
}
#endif
static void logBuffer(const char *message, const void *data, size_t length)
{
    size_t bufferLength = length * NBR_CHAR_PER_BYTE + ESCAPE_CHAR;
    char *buf = kzalloc(bufferLength, GFP_KERNEL);

    if (buf != NULL)
    {
        size_t index;
        char *ptr = buf;
        char *end = buf + bufferLength;

        uint8_t *_data = (uint8_t *) data;

        for (index = 0; index < length && ptr < end; index++)
        {
            ptr += sprintf(ptr, index < (length - 1) ? "0x%02x " : "0x%02x", _data[index]);
        }

        pr_debug("%s length: %zu [%s]\n", message, length, buf);

        kfree(buf);
    }

}

/*
 * Private Section
 */

static int32_t i2cKernelSend(I2c *i2c, uint8_t address, const void *data, size_t num)
{
    int32_t res = -EIO;

    if (i2c != NULL && data != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);
        char *buf = kzalloc(num, GFP_KERNEL);

        dev_dbg(&ctx->client->dev, "I2C Write Address: 0x%x\n", address);

        logBuffer("Write: ", data, num);

        if (buf != NULL)
        {
	    int status;

            memcpy(buf, data, num);
            status = i2c_master_send(ctx->client, (const char *) buf, (int) num);
            res = status == num ? ARM_DRIVER_OK : status;

            kfree(buf);
        }
    }

    return res;
}

int32_t i2cKernelRead(I2c *i2c, uint8_t address, void *data, size_t num)
{
    int32_t res = -EIO;

    if (i2c != NULL && data != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);
        char *buf = kzalloc(num, GFP_KERNEL);

        dev_dbg(&ctx->client->dev, "I2C Read Address: 0x%x \n", address);

        if (buf != NULL)
        {
            int status = i2c_master_recv(ctx->client, (char *) buf, num);

            res = status == num ? 0 : status;

            if (res == ARM_DRIVER_OK)
            {
                memcpy(data, buf, num);
                logBuffer("Read: ", data, num);
            }

            kfree(buf);
        }


    }

    return res;
}

/*
 * Public Section
 */

I2c *i2cBoreasLinuxInit(struct i2c_client *client)
{
    I2c *driver = NULL;

    if (client != NULL)
    {
        Context *ctx = kzalloc(sizeof(Context), GFP_KERNEL);

        if (ctx != NULL)
        {
            ctx->client = client;
            initDriverFct(ctx);
            driver = &ctx->driver;
#if 0
            if (ctx->client->dev.of_node)
               readDevTree(ctx);
#endif
        }
    }

    pr_debug("I2C client: %p, driver: %p\n", client, driver);

    return driver;
}

bool i2cBoreasLinuxFree(I2c *i2c)
{
    bool res = false;

    if (i2c != NULL)
    {
        Context *ctx = container_of(i2c, Context, driver);

        dev_info(&ctx->client->dev, "I2C Linux Wrapper Free\n");

        kfree(ctx);

        res = true;
    }

    return res;
}

/*
 * Private Section
 */

void initDriverFct(Context *ctx)
{
    ctx->driver.write = i2cKernelSend;
    ctx->driver.read = i2cKernelRead;
}
