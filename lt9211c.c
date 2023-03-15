// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAXIM9286 Deserializer for BST Deserializer Driver
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/usb.h>
#include <linux/power_supply.h>
#include "lt9211c.h"


#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/of_gpio.h>
#include <media/media-entity.h>
#include <media/media-device.h>
#include "ti_deser_hub.h"
#include "camera_common_op.h"
#include "maxim_deser_hub.h"

#define MODULE_NAME "bst,lt9211c"
#define	FAIL 0
#define	SUCCESS 1
#define lt9211c_N_LINKS 4
#define RETRY_MAX_TIMES         3
#define I2C_MAX_TRANSFER_SIZE   255
#define lt9211_ADDR_LENGTH      1

#define PRINT_DEG(args...) \
	do { \
		if (1) { \
			printk(args); \
		} \
	} while (0)
	

struct lt9211c_priv {
	struct i2c_client *client;
	int pclk;
	int rst_gpio;

	// int des_addr;
	// int n_links;
	// int links_mask;
	// long pixel_rate;
	// const char *fsync_mode;
	// int fsync_period;

	

	// int him;
	// int hsync;
	// int vsync;
	// int bws;
	// int dbl;
	// int dt;
	// int timeout;
	// u64 crossbar;
	// char cb[16];
	// int ser_addr[4];
};


static struct lt9211c_priv *priv = NULL;



static unsigned char register_read(struct i2c_client *client, uint8_t reg)
{   

    unsigned char buf[2] = {reg};

    int ret = i2c_smbus_read_byte_data(client, reg);

    if (ret < 0)
        dev_err(&client->dev, "~~~~~~~9211  %s: read 0x%02x failed\n", __func__, reg);

    if(ret == 0) {
        return buf[1];
    } else {
        return 0;
    }
}



// unsigned char lt9211_read(struct i2c_client *client, unsigned char addr)
// {
//     unsigned char buf[2] = {addr};
//     int ret = -1;
//     ret = lt9211_i2c_read(client, buf, 2);
//     if(ret == 0) {
//         return buf[1];
//     } else {
//         return 0;
//     }
// }


static int register_write(struct i2c_client *client, uint8_t reg, uint8_t value)
{
    int ret = i2c_smbus_write_byte_data(client, reg, value); //-- 本质就是对 i2c_transfer 的封装?

    if (ret < 0)
        dev_err(&client->dev, "~~~~~~~9211 %s: write 0x%02x failed\n", __func__, reg);

    return ret;
}


/*******************************************************
    Function:
    Write data to the i2c slave device.
    Input:
    client: i2c device.
    buf[0]: write start address.
    buf[1~len-1]: data buffer
    len: lt9211_ADDR_LENGTH + write bytes count
    Output:
    numbers of i2c_msgs to transfer:
        0: succeed, otherwise: failed
 *********************************************************/
// write 8bit data to 8bit register
static int bst_i2c_write_byte_data_byte_reg00(struct i2c_client *client, uint8_t slave_address, uint8_t reg_offset, uint8_t value)
{
    int ret;
    struct i2c_msg msg;
    unsigned char data[2];

    msg.addr = slave_address; /* I2C address of chip */
    msg.flags = 0;
    msg.len = 2;
    msg.buf = data;

    data[0] = reg_offset; /* register num */
    data[1] = value; /* register data */
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret != 1) {
        pr_info("i2c_transfer error, slave = 0x%x, reg = 0x%x, ret = %d\n",
            slave_address, reg_offset, ret);
        return -1;
    }

    return 0;
}


// read 8bit data from 8bit register
static unsigned char bst_i2c_read_byte_data_byte_reg00(struct i2c_client *client
            , uint8_t slave_address
        , uint8_t reg_offset)
{
    int ret;
    uint8_t value;
    struct i2c_msg msg[2] = { {
                      .addr = slave_address,
                      .flags = 0,
                      .len = 1,
                      .buf = &reg_offset,
                  },
                    {
                      .addr = slave_address,
                      .flags = I2C_M_RD,
                      .len = 1,
                      .buf = &value,
                  } };

    ret = i2c_transfer(client->adapter, &msg[0], 1);
    if (ret != 1) {
        pr_info("i2c_transfer send error, slave = 0x%x, reg = 0x%x, ret = %d\n",
            slave_address, reg_offset, ret);
        return -EIO;
    }

    ret = i2c_transfer(client->adapter, &msg[1], 1);
    if (ret != 1) {
        pr_info("i2c_transfer read error, slave = 0x%x, reg = 0x%x, ret = %d\n",
            slave_address, reg_offset, ret);
        return -EIO;
    }
    // if (out_value)
        // *out_value = value;
    return value;

    // return 0;
}




int lt9211_i2c_write(struct i2c_client *client, unsigned char *buf, int len)
{
    unsigned int pos = 0, transfer_length = 0;
    unsigned char address = buf[0];
    unsigned char put_buf[64];
    int retry, ret = 0;
    struct i2c_msg msg = {
        .addr = client->addr,
        .flags = !I2C_M_RD,
    };
    if(likely(len < sizeof(put_buf))) {
        /* code optimize,use stack memory*/
        msg.buf = &put_buf[0];
    } else {
        msg.buf = kmalloc(len > I2C_MAX_TRANSFER_SIZE
                          ? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
        if(!msg.buf)
            return -ENOMEM;
    }
    len -= lt9211_ADDR_LENGTH;
    while(pos != len) {
        if(unlikely(len - pos > I2C_MAX_TRANSFER_SIZE - lt9211_ADDR_LENGTH))
            transfer_length = I2C_MAX_TRANSFER_SIZE - lt9211_ADDR_LENGTH;
        else
            transfer_length = len - pos;


        msg.buf[0] = address;
        msg.len = transfer_length + lt9211_ADDR_LENGTH;

        // printk("~~9211 msg.buf[0]: %02x", msg.buf[0]); //ff
        // printk("~~9211 msg.len: %02x", msg.len);       //02

        memcpy(&msg.buf[lt9211_ADDR_LENGTH], &buf[lt9211_ADDR_LENGTH + pos], transfer_length);
        for(retry = 0; retry < RETRY_MAX_TIMES; retry++) {
            if(likely(i2c_transfer(client->adapter, &msg, 1) == 1)) {
                pos += transfer_length;
                address += transfer_length;
                break;
            }
            dev_info(&client->dev, "I2C write retry[%d]\n", retry + 1);
            usleep_range(1000,2000);
        }
        if(unlikely(retry == RETRY_MAX_TIMES)) {
            dev_err(&client->dev,
                    "I2c write failed,dev:%02x,reg:%02x,size:%u\n",
                    client->addr, address, len);
            ret = -EAGAIN;
            goto write_exit;
        }
    }
write_exit:
    if(len + lt9211_ADDR_LENGTH >= sizeof(put_buf))
        kfree(msg.buf);
    return ret;
}

/*******************************************************
    Function:
    Read data from the i2c slave device.
    Input:
    client: i2c device.
    buf[0]: read start address.
    buf[1~len-1]: data buffer
    len: lt9211_ADDR_LENGTH + read bytes count
    Output:
    numbers of i2c_msgs to transfer:
        0: succeed, otherwise: failed
 *********************************************************/
int lt9211_i2c_read(struct i2c_client *client, unsigned char *buf, int len)
{
    unsigned int transfer_length = 0;
    unsigned int pos = 0;
    unsigned char address = buf[0];
    unsigned char get_buf[64], addr_buf[2];
    int retry, ret = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = client->addr,
            .flags = !I2C_M_RD,
            .buf = &addr_buf[0],
            .len = lt9211_ADDR_LENGTH,
        }, {
            .addr = client->addr,
            .flags = I2C_M_RD,
        }
    };
    len -= lt9211_ADDR_LENGTH;
    if(likely(len < sizeof(get_buf))) {
        /* code optimize, use stack memory */
        msgs[1].buf = &get_buf[0];
    } else {
        msgs[1].buf = kzalloc(len > I2C_MAX_TRANSFER_SIZE
                              ? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
        if(!msgs[1].buf)
            return -ENOMEM;
    }
    while(pos != len) {
        if(unlikely(len - pos > I2C_MAX_TRANSFER_SIZE))
            transfer_length = I2C_MAX_TRANSFER_SIZE;
        else
            transfer_length = len - pos;
        msgs[0].buf[0] = address;
        msgs[1].len = transfer_length;
        for(retry = 0; retry < RETRY_MAX_TIMES; retry++) {
            if(likely(i2c_transfer(client->adapter, msgs, 2) == 2)) {
                memcpy(&buf[lt9211_ADDR_LENGTH + pos], msgs[1].buf, transfer_length);
                pos += transfer_length;
                address += transfer_length;
                break;
            }
            dev_info(&client->dev, "I2c read retry[%d]:0x%x\n",
                     retry + 1, address);
            usleep_range(1000,2000);
        }
        if(unlikely(retry == RETRY_MAX_TIMES)) {
            dev_err(&client->dev,
                    "I2c read failed,dev:%02x,reg:%02x,size:%u\n",
                    client->addr, address, len);
            ret = -EAGAIN;
            goto read_exit;
        }
    }
read_exit:
    if(len >= sizeof(get_buf))
        kfree(msgs[1].buf);
    return ret;
}



int lt9211_write(struct i2c_client *client, unsigned char addr, unsigned char data)
{
    unsigned char buf[2] = {addr, data};
    int ret = -1;
    ret = lt9211_i2c_write(client, buf, 2);
	if(ret < 0){
		printk("lt9211_i2c_write error\n");
	}
    return ret;
}

unsigned char lt9211_read(struct i2c_client *client, unsigned char addr)
{
    unsigned char buf[2] = {addr};
    int ret = -1;
    ret = lt9211_i2c_read(client, buf, 2);
    if(ret == 0) {
        return buf[1];
    } else {
        return 0;
    }
}


int HDMI_WriteI2C_Byte(unsigned char addr, unsigned char data)
{
	int ret = -1;
	// ret =  lt9211_write(priv->client, addr, data);
    // ret =  register_write(priv->client, addr, data);
    ret =   bst_i2c_write_byte_data_byte_reg00(priv->client, priv->client->addr, addr, data);
    
	return ret;
}


unsigned char HDMI_ReadI2C_Byte(unsigned char addr)
{
    // unsigned char ret = lt9211_read(priv->client, addr);
   	// unsigned char ret = register_read(priv->client, addr);
    unsigned char ret = bst_i2c_read_byte_data_byte_reg00(priv->client, priv->client->addr,  addr);

   	return ret;
}







// static char fsync_mode_default[20] = "automatic"; /* manual, automatic, semi-automatic, external */
// static unsigned long crossbar = 0xba9876543210;   /* default crossbar */

// static int register_read(struct deser_hub_dev *hub, uint8_t reg)
// {
// 	int ret = i2c_smbus_read_byte_data(hub->i2c_client, reg);

// 	if (ret < 0)
// 		dev_err(hub->dev, "%s: read 0x%02x failed\n", __func__, reg);

// 	return ret;
// }

// static int register_write(struct deser_hub_dev *hub, uint8_t reg, uint8_t value)
// {
// 	int ret = i2c_smbus_write_byte_data(hub->i2c_client, reg, value);

// 	if (ret < 0)
// 		dev_err(hub->dev, "%s: write 0x%02x failed\n", __func__, reg);

// 	return ret;
// }




// static int lt9211c_initialize(struct lt9211c_priv *priv)
// {
// 	int i;
// 	// struct deser_hub_dev *hub = &priv->hub;


// 	return 0;
// }

static int lt9211_parse_dt(struct device *dev, struct lt9211c_priv *pdata)
{
	// int ret = 0;
    struct device_node *np = dev->of_node;  //指定的设备树节点
    //pdata->pwr_gpio = of_get_named_gpio(np, "power-gpio", 0);
    //if(!gpio_is_valid(pdata->pwr_gpio)) {
    //    dev_err(dev, "No valid pwr gpio");
       // return -1;
    //}

    // 指定的设备树节点
    // GPIO属性名="reset-gpio"
    // index 引脚索引值，若一个属性有多个属性值，index表示选第几个属性值，取 0 表示选第一个属性值
	pdata->rst_gpio = of_get_named_gpio(np, "reset-gpio", 0);  

    if(!gpio_is_valid(pdata->rst_gpio)) {
        dev_err(dev, "No valid rst gpio");
        return -1;
    }
	
	return 0;
}


static int lt9211_request_io_port(struct lt9211c_priv *pdata)
{
    int ret = 0;
    //if(gpio_is_valid(pdata->pwr_gpio)) {
    //    ret = gpio_request(pdata->pwr_gpio, "pdata_pwr");
    //    if(ret < 0) {
    //        dev_err(&pdata->client->dev,
    //                "Failed to request GPIO:%d, ERRNO:%d\n",
    //                (s32)pdata->pwr_gpio, ret);
    //        return -ENODEV;
    //    }
    //    gpio_direction_input(pdata->pwr_gpio);
    //    dev_info(&pdata->client->dev, "Success request pwr-gpio\n");
    //}
    if(gpio_is_valid(pdata->rst_gpio)) {

    // int gpio_request(unsigned gpio, const char *label)  
    // gpio_request 函数用于申请一个 GPIO 管脚，在使用一个 GPIO 之前一定要使用 gpio_request进行申请
    // gpio:使用 of_get_named_gpio 从设备树获取指定 GPIO 属性信息，此函数会返回这个 GPIO 的标号。
    // label: 给 gpio 设置个名字。
        ret = gpio_request(pdata->rst_gpio, "lt9211_rst");
        if(ret < 0) {
            dev_err(&pdata->client->dev,
                    "Failed to request GPIO:%d, ERRNO:%d\n",
                    (s32)pdata->rst_gpio, ret);
            // if(gpio_is_valid(pdata->pwr_gpio))
            //     gpio_free(pdata->pwr_gpio);
            return -ENODEV;
        }
 //       gpio_direction_input(pdata->rst_gpio);
        // dev_info(&pdata->client->dev,  "Success request rst-gpio\n");
    }

    return 0;
}


void lt9211_read_ID(void)  // ??
{
    PRINT_DEG("%s\n",__func__);
	unsigned char ID[3];
    HDMI_WriteI2C_Byte(0xff,0x81);//register bank
    ID[0] =HDMI_ReadI2C_Byte(0x00);
	ID[1] =HDMI_ReadI2C_Byte(0x01);
	ID[2] =HDMI_ReadI2C_Byte(0x02);

    pr_info("lt9211 read ID[0]=0x%2x ID[1]=0x%2x ID[2]=0x%2x\n", ID[0], ID[1], ID[2]);

}



static int lt9211c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	// u8 val = 0;

    PRINT_DEG("LT9211 I2C Address: 0x%02x\n", client->addr);

	//lt9211 复位低100ms高100ms
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {     // ok
        dev_err(&client->dev, "~~~~~~~~~Failed check I2C functionality");
        return -ENODEV;
    }

	// struct lt9211c_priv *priv;
	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);   // ok 
	if (!priv){
		dev_err(&client->dev, "~~~~~~~~~Failed devm_kzalloc priv");
		return -ENOMEM;
	}


    if(client->dev.of_node) {  // ok 
        ret = lt9211_parse_dt(&client->dev, priv);
        if(ret < 0) {
            dev_err(&client->dev, "~~~~~~~~~Failed parse lt9211\n");
            return -ENODEV;
        }
    }


    priv->client = client;        //数据结构 i2c_client 赋值
	i2c_set_clientdata(client, priv);
	

	

    //使用一个 GPIO 之前, 申请一个 GPIO 管脚
    ret = lt9211_request_io_port(priv);  // ok 

    if(ret < 0) {
        dev_err(&client->dev, "~~~~~~~~~Failed request IO port\n");
        return -ENODEV;
    }


    // //测试设置 低电平
    gpio_direction_output(priv->rst_gpio, 0);      
    usleep_range(5000, 10000);   
    gpio_direction_output(priv->rst_gpio, 1); 
    usleep_range(5000, 10000); 
    pr_info("~~~~~~~~~lt9211c reset the priv->rst_gpio!");  

    lt9211_read_ID();

	pr_info("~~~~~~~~~lt9211c probe ok !!!!\n");
	return 0;
}

static int lt9211c_remove(struct i2c_client *client)
{
    struct lt9211c_priv *lt9211 = i2c_get_clientdata(client);
	return 0;
}

static const struct i2c_device_id lt9211c_id[] = {
	{ MODULE_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, lt9211c_id);

static const struct of_device_id lt9211c_of_match[] = {
	{
		.compatible = MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, lt9211c_of_match);

static struct i2c_driver lt9211c_driver = {
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(lt9211c_of_match),
	},
	.probe		= lt9211c_probe,
	.remove		= lt9211c_remove,
	.id_table	= lt9211c_id,
};

module_i2c_driver(lt9211c_driver);

MODULE_DESCRIPTION("GMSL driver for lt9211c");
MODULE_AUTHOR("kanghua.li@bst.ai");
MODULE_LICENSE("GPL v2");
