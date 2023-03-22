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
// #include <stdio.h>

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


#define         LT9211_MODE_SEL             TTL_IN_MIPI_OUT // ---------- 输入模型设置 ？？？？？？

#define     TTLRX_DATARATE_MODE         SDR             //SDR/DDR  单沿采样  双沿采样
#define     TTLRX_PHASE_SEL             10               //0~9/10: print all phase
#define     TTLRX_DE_SYNC_MODE          0               //0-normal mode 1-sync mode(gen de) 2-de mode(gen sync)  //use in separate sync mode
#define     TTLRX_VIDEO_FORMAT          P_FORMAT        //P_FORMAT/I_FORMAT


#if TTLRX_DATARATE_MODE == SDR
/*==============TTLRX SDR MODE SETTING==============*/
    #define     TTLRX_SYNC_INTER_MODE       ENABLED        //ENABLED/DISABLED  内同步信号则打开
    #define     TTLRX_COLORSPACE            YUV422             //RGB/YUV444/YUV422, YUV422内同步模式时ENABLED，其他模式时DISABLED。
    #define     TTLRX_INPUT_MODE            INPUT_BT1120_16BIT    //INPUT_RGB888    INPUT_RGB666    INPUT_RGB565    INPUT_YCBCR444
                                                            //INPUT_YCBCR422_8BIT   INPUT_YCBCR422_10BIT    INPUT_YCBCR422_12BIT
                                                            //INPUT_YCBCR422_16BIT  INPUT_YCBCR422_20BIT    INPUT_YCBCR422_24BIT
                                                            //INPUT_BT656_8BIT      INPUT_BT656_10BIT       INPUT_BT656_12BIT
                                                            //INPUT_BT1120_8BIT     INPUT_BT1120_10BIT      INPUT_BT1120_12BIT
                                                            //INPUT_BTA_T1004_16BIT INPUT_BTA_T1004_20BIT   INPUT_BTA_T1004_24BIT
                                                            //INPUT_BT1120_16BIT    INPUT_BT1120_20BIT      INPUT_BT1120_24BIT ？？   8bit， 16bit。  2971 接出来是哪种格式。
/*==============TTLTX SDR MODE SETTING==============*/
// BT1120的话，设置成16bit
#endif

// #if TTLRX_DATARATE_MODE == DDR
// /*==============TTLRX DDR MODE SETTING==============*/
//     #define     TTLRX_SYNC_INTER_MODE       ENABLED        //ENABLED/DISABLED
//     #define     TTLRX_COLORSPACE            YUV422             //RGB/YUV444/YUV422
//     #define     TTLRX_HALF_CLOCK_MODE       ENABLED        //ENABLED/DISABLED
//     #define     TTLRX_HALF_WIDTH_MODE       ENABLED        //ENABLED/DISABLED
//     #define     TTLRX_INPUT_MODE            INPUT_BT1120_16BIT    //INPUT_RGB888    INPUT_RGB666    INPUT_RGB565    INPUT_YCBCR444
// //                                                             //INPUT_YCBCR422_8BIT   INPUT_YCBCR422_10BIT    INPUT_YCBCR422_12BIT
// //                                                             //INPUT_YCBCR422_16BIT  INPUT_YCBCR422_20BIT    INPUT_YCBCR422_24BIT
// //                                                             //INPUT_BT656_8BIT      INPUT_BT656_10BIT       INPUT_BT656_12BIT
// //                                                             //INPUT_BT1120_8BIT     INPUT_BT1120_10BIT      INPUT_BT1120_12BIT
// //                                                             //INPUT_BTA_T1004_16BIT INPUT_BTA_T1004_20BIT   INPUT_BTA_T1004_24BIT
// //                                                             //INPUT_BT1120_16BIT    INPUT_BT1120_20BIT      INPUT_BT1120_24BIT
// /*==============TTLTX DDR MODE SETTING==============*/
// #endif


// //vedio settings
// #define     TTLTX_VIDEO_FORMAT          I_FORMAT        //P_FORMAT/I_FORMAT
// #define     TTLTX_VSYNC_POLARITY        PLUS            //PLUS/MINUS
// #define     TTLTX_HSYNC_POLARITY        PLUS            //PLUS/MINUS

/*==============MIPITX PATTERN SETTING==============*/
// #define         MIPITX_PATTERN_OUT_SEL      MIPI_CSI //MIPI_DSI         //MIPI_DSI/MIPI_CSI   mf mod
// #define         MIPITX_PATTERN_COLORSPACE   RGB              //RGB/YUV422/YUV420
// #define         MIPITX_PATTERN_OUTPUT_LANE  MIPITX_3LANE     //MIPITX_4LANE/MIPITX_3LANE/MIPITX_2LANE/MIPITX_1LANE
// #define         MIPITX_PATTERN_CLOCK_BURST  DISABLED         //ENABLED/DISABLED
// #define         MIPITX_PATTERN_OUTPUT_PORT  PORTB            //PORTA/PORTB


#define     MIPITX_OUT_SEL             MIPI_CSI          //MIPI_DSI/MIPI_CSI        ？？
#define     MIPITX_PHY_SEL             MIPI_DPHY         //MIPI_DPHY/MIPI_CPHY
#define     MIPITX_CMD_SEL             MIPITX_DSC          //MIPITX_DSC/MIPITX_CSC
#define     MIPITX_PORT_SEL            PORTA               //PORTA/PORTB/DOU_PORT   ？？ 
#define     MIPITX_LANE_NUM            MIPITX_4LANE        //MIPITX_4LANE/MIPITX_3LANE/MIPITX_2LANE/MIPITX_1LANE, no MIPITX_8LANE  ？？
#define     MIPITX_PORT_COPY           NO_COPY            //ENABLED/NO_COPY         ？？
#define     MIPI_CLOCK_BURST           ENABLED             //ENABLED/DISABLED      ？？


// #define FAIL 0
// #define SUCCESS 1
// #define lt9211_ADDR_LENGTH      1
// #define I2C_MAX_TRANSFER_SIZE   255
// #define RETRY_MAX_TIMES         3
// #define lt9211_I2C_NAME      "lt9211"
// #define lt9211_DRIVER_VERSION  "1.0.0"

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
	

// void pr_info(u8 ucLvl, const char* fmt, ...)
// {
    
//     printk(fmt);

// }



void Ocm_Timer0_Delay1ms(int tt)
{
    mdelay(tt);
}


enum lc9211_pads {
    lc9211_SINK_LINK0,
    lc9211_SINK_LINK1,
    lc9211_SINK_LINK2,
    lc9211_SINK_LINK3,
    lc9211_SOURCE,
    lc9211_N_PADS,
};


struct lt9211c_priv {
	struct i2c_client *client;
	int pclk;
	int rst_gpio;

    struct deser_hub_dev hub;
    struct media_pad pads[lc9211_N_PADS];
    // int des_addr;
    // int n_links;
    // int links_mask;
    // long pixel_rate;
    const char *fsync_mode;

	int des_addr;
	int n_links;
	// int links_mask;
	// long pixel_rate;
	// const char *fsync_mode;
	int fsync_period;

	

	// int him;
	// int hsync;
	// int vsync;
	// int bws;
	int dbl;
	int dt;
	// int timeout;
	// u64 crossbar;
	// char cb[16];
	int ser_addr[4];
};


static struct lt9211c_priv *priv = NULL;



// static unsigned char register_read(struct i2c_client *client, uint8_t reg)
// {   

//     unsigned char buf[2] = {reg};

//     int ret = i2c_smbus_read_byte_data(client, reg);

//     if (ret < 0)
//         dev_err(&client->dev, "~~~~~~~9211  %s: read 0x%02x failed\n", __func__, reg);

//     if(ret == 0) {
//         return buf[1];
//     } else {
//         return 0;
//     }
// }



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


// static int register_write(struct i2c_client *client, uint8_t reg, uint8_t value)
// {
//     int ret = i2c_smbus_write_byte_data(client, reg, value); //-- 本质就是对 i2c_transfer 的封装?

//     if (ret < 0)
//         dev_err(&client->dev, "~~~~~~~9211 %s: write 0x%02x failed\n", __func__, reg);

//     return ret;
// }

static int register_write(struct deser_hub_dev *hub, uint8_t reg, uint8_t value)
{
    int ret = i2c_smbus_write_byte_data(hub->i2c_client, reg, value);

    if (ret < 0)
        dev_err(hub->dev, "%s: write 0x%02x failed\n", __func__, reg);

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








static int parse_input_dt(struct deser_hub_dev *hub, struct device_node *node)
{
    int i;

    struct lt9211c_priv *priv = container_of(hub, struct lt9211c_priv, hub);
    pr_info("vv9211 n_links: %d", priv->n_links );

    for (i = 0; i < priv->n_links; i++) {
        struct device_node *port;
        struct device_node *remote;

        port = of_graph_get_port_by_id(node, i);
        if (!port) {
            dev_err(hub->dev, "%s: vv9211 input port%d not found\n ",
                __func__, i);
            break;
        }

        remote = of_graph_get_remote_node(node, i, 0);
        if (!remote) {
            dev_err(hub->dev, "%s: vv9211 input device%d not found\n",
                __func__, i);
            break;
        }

        hub->chn[i].index = i;
        hub->chn[i].camera_node = remote;
        hub->chn[i].camera_fwnode = of_fwnode_handle(remote);
        hub->num_cameras++;
    }

    return 0;
}


static int parse_output_dt(struct deser_hub_dev *hub, struct device_node *node)
{
    struct device_node *csi2 = of_get_child_by_name(node, "csi-link");

    if (!csi2) {
        dev_err(hub->dev, "vv9211: csi-link not found\n");
        return -EINVAL;
    }

    hub->subdev.fwnode = of_fwnode_handle(csi2);

    return 0;
}


static int lt9211_parse_dt(struct device *dev, struct lt9211c_priv *pdaprivta,  struct i2c_client *client)
{   
    int err, i;
    u32 addrs[4], naddrs;
	// int ret = 0;
    struct device_node *np = dev->of_node;  //指定的设备树节点

    // if (!np)
    //     return -EINVAL;

    struct deser_hub_dev *hub = &pdaprivta->hub;


    if (of_property_read_s32(np, "reg", &pdaprivta->des_addr)) {
        dev_err(&client->dev, "Invalid DT reg property\n");
        pr_info("vv9211 error:  Invalid DT reg property\n");
        // return -EINVAL;
    }

    naddrs = of_property_count_elems_of_size(np, "regs", sizeof(u32));
    err = of_property_read_u32_array(client->dev.of_node, "regs", addrs, naddrs);
    priv->n_links = naddrs;

    pr_info("vv9211 naddrs: %u", naddrs);
    pr_info("vv9211 err: %u", err);

    // pr_info("vv9211 2naddrs: %u", of_property_count_elems_of_size(np, "regs", sizeof(u32)));
    // pr_info("vv9211 2err: %u", of_property_read_u32_array(client->dev.of_node, "regs", addrs, naddrs));
    if (parse_input_dt(hub, np)) {
        dev_err(hub->dev, "vv9211 parse input dt failed\n");
        return -1;
    }


    if (parse_output_dt(hub, np)) {
        dev_err(hub->dev, "vv9211 parse output dt failed\n");
        return -1;
    }


    // memcpy(priv->ser_addr, addrs, naddrs * sizeof(u32));

    // if (err < 0) {
    //     dev_err(&client->dev, "Invalid DT regs property\n");
    //     pr_info("vv9211 error:  Invalid DT reg property\n");
    //     // return -EINVAL;
    // }
    // 指定的设备树节点
    // GPIO属性名="reset-gpio"
    // index 引脚索引值，若一个属性有多个属性值，index表示选第几个属性值，取 0 表示选第一个属性值
	pdaprivta->rst_gpio = of_get_named_gpio(np, "reset-gpio", 0);  

    if(!gpio_is_valid(pdaprivta->rst_gpio)) {
        dev_err(dev, "No valid rst gpio");
        return -1;
    }
	
	return 0;
}


static int lt9211_request_io_port(struct lt9211c_priv *pdaprivta)
{
    int ret = 0;
    if(gpio_is_valid(pdaprivta->rst_gpio)) {

    // int gpio_request(unsigned gpio, const char *label)  
    // gpio_request 函数用于申请一个 GPIO 管脚，在使用一个 GPIO 之前一定要使用 gpio_request进行申请
    // gpio:使用 of_get_named_gpio 从设备树获取指定 GPIO 属性信息，此函数会返回这个 GPIO 的标号。
    // label: 给 gpio 设置个名字。
        ret = gpio_request(pdaprivta->rst_gpio, "lt9211_rst");
        if(ret < 0) {
            dev_err(&pdaprivta->client->dev,
                    "Failed to request GPIO:%d, ERRNO:%d\n",
                    (s32)pdaprivta->rst_gpio, ret);
            return -ENODEV;
        }
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



// **************************************************************************************************************************************
// -  init
// **************************************************************************************************************************************
void Drv_TtlRx_PhyPowerOn(void)
{

    PRINT_DEG("~~~9211 √√√√√%s\n",__func__);
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x01,0x22); //TTL enable
    HDMI_WriteI2C_Byte(0x60,0x0a); //tx phy at TtlRx phy
    HDMI_WriteI2C_Byte(0x61,0x23);
    HDMI_WriteI2C_Byte(0x61,0x33);
    HDMI_WriteI2C_Byte(0x62,0x08);
}

void Drv_TtlRxClk_Sel(void)
{
    PRINT_DEG("~~~9211  √√√√√%s\n",__func__);
        /* CLK sel */
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0xe9,0x88); //sys clk sel from XTAL
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x80,0x00); 
    HDMI_WriteI2C_Byte(0x08,0x7f);  //top ctrl reg
    
}

void Drv_SystemActRx_Sel(IN u8 ucSrc)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) & 0xf8));

    switch(ucSrc)
    {
        case LVDSRX:
            PRINT_DEG("~~~9211 LVDSRX\n");
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | LVDSRX));
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) & 0xcf)); //[5:4]00: LVDSRX
        break;
        case MIPIRX:
            PRINT_DEG("~~~9211 MIPIRX\n");
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | MIPIRX));
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | BIT4_1)); //[5:4]01: MIPIRX
        break;
        case TTLRX:
            PRINT_DEG("~~~9211 TTLRX  √√√√√\n");
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | TTLRX));
        break;
        case PATTERN:
            PRINT_DEG("~~~9211 PATTERN\n");
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | PATTERN));
        break;
        default:
            PRINT_DEG("~~~9211 default\n");
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | LVDSRX));
        break;
        
    }
}



// **************************************************************************************************************************************
// -  STATE_CHIPRX_VIDTIMING_CONFIG
// **************************************************************************************************************************************

void Drv_TtlRxHalfBit_Set(void)
{   PRINT_DEG("%s\n",__func__);
    
    #if   ((TTLRX_INPUT_MODE == INPUT_YCBCR422_8BIT) || (TTLRX_INPUT_MODE == INPUT_YCBCR422_10BIT) || (TTLRX_INPUT_MODE == INPUT_YCBCR422_12BIT))
        #define     TTLRX_HALF_BIT_MODE         ENABLED
    #elif ((TTLRX_INPUT_MODE == INPUT_BT656_8BIT)    || (TTLRX_INPUT_MODE == INPUT_BT656_10BIT)    || (TTLRX_INPUT_MODE == INPUT_BT656_12BIT))
        #define     TTLRX_HALF_BIT_MODE         ENABLED
    #elif ((TTLRX_INPUT_MODE == INPUT_BT1120_8BIT)   || (TTLRX_INPUT_MODE == INPUT_BT1120_10BIT)   || (TTLRX_INPUT_MODE == INPUT_BT1120_12BIT))
        #define     TTLRX_HALF_BIT_MODE         ENABLED
    #else
        #define     TTLRX_HALF_BIT_MODE         DISABLED
    #endif

}

void Drv_TtlRxDig_Set(void)
{PRINT_DEG("%s\n",__func__);
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0xc1,0x00);
    HDMI_WriteI2C_Byte(0xc4,0x00); 
    HDMI_WriteI2C_Byte(0xc5,0x00);
    HDMI_WriteI2C_Byte(0xc6,0x00);     
    HDMI_WriteI2C_Byte(0xc7,0x07);
    HDMI_WriteI2C_Byte(0xc8,0x04);
    HDMI_WriteI2C_Byte(0xe8,0x00); 
    

    //ttlrx sync mode set
    #if TTLRX_SYNC_INTER_MODE == ENABLED
    
        HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x08); //embedded sync mode enable
        pr_info("~~~9211 Sync Mode:    Embedded");
        
        //embedded video format set
        #if TTLRX_VIDEO_FORMAT == I_FORMAT
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x80); //Interlace format
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x10); //rgd_ttlrx_bt_sync_gen_en
            PRINT_DEG("~~~9211 Video Format:    I Format");
        #else
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) & 0x7f); //Progressive format
            PRINT_DEG("~~~9211 Video Format:    P Format");
        #endif
        
        //embedded Half Bit Mode set
        #if TTLRX_HALF_BIT_MODE == ENABLED
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x40); //8Bit embedded enable
        #endif     
    #endif

    
    #if TTLRX_DATARATE_MODE == DDR
        #if TTLRX_HALF_WIDTH_MODE == ENABLED
            HDMI_WriteI2C_Byte(0xc1,(HDMI_ReadI2C_Byte(0xc1) | 0x80)); //ddr half width enable
            HDMI_WriteI2C_Byte(0xc0,(HDMI_ReadI2C_Byte(0xc0) | 0x08)); //fifo_w_en
            pr_info("~~~9211 DDR Half Width Mode:    Enable");
            //DDR Trans Mode set
            #if 0
                HDMI_WriteI2C_Byte(0xc1,HDMI_ReadI2C_Byte(0xc1) | 0x40); //high 12 Bit first
                pr_info("DDR Trans Mode:    High 12Bit First");
            #else
                HDMI_WriteI2C_Byte(0xc1,HDMI_ReadI2C_Byte(0xc1) & 0xbf); //low 12 Bit first
                pr_info("~~~9211 DDR Trans Mode:    Low 12Bit First");
            #endif
            //DDR Input Mode set
            #if 1
                HDMI_WriteI2C_Byte(0xc1,HDMI_ReadI2C_Byte(0xc1) | 0x0d);
                pr_info("~~~9211 DDR Input Mode:    12Bit Mode");
            #endif
        #endif
    #endif
    
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x0d,0x7e); //ttl rx reset
    HDMI_WriteI2C_Byte(0x0d,0x7f); //ttl rx release
}
void Drv_TtlRxMapping_Set(void)
{
    u8 ucRxVidFmt;
    
    HDMI_WriteI2C_Byte(0xff,0x85);
    
    switch(TTLRX_INPUT_MODE)
    {
        case INPUT_RGB888:
            //RGB swap case
            #if 0   //input RGB  R: D23->D16    G: D15->D8    B:D7->D0    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input RBG  R: D23->D16    B: D15->D8    G:D7->D0    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input GRB  G: D23->D16    R: D15->D8    B:D7->D0    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input GBR  G: D23->D16    B: D15->D8    R:D7->D0    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input BRG  B: D23->D16    R: D15->D8    G:D7->D0    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 1   //input BGR  B: D23->D16    G: D15->D8    R:D7->D0    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input BGR  B: D23->D16    G: D15->D8    R:D0->D7    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input BGR  B: D23->D16    G: D8->D15    R:D7->D0    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input BGR  B: D16->D23    G: D15->D8    R:D7->D0    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            ucRxVidFmt = RGB_8Bit;
            pr_info("~~~9211 TTLRx Input Mode:    RGB888");
            break;
            
        case INPUT_RGB666:
            //RGB swap case
            #if 1   //input RGB  R: D23->D18    G: D15->D10    B:D7->D2    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input RBG  R: D23->D18    B: D15->D10    G:D7->D2    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input GRB  G: D23->D18    R: D15->D10    B:D7->D2    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input GBR  G: D23->D18    B: D15->D10    R:D7->D2    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input BRG  B: D23->D18    R: D15->D10    G:D7->D2    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input BGR  B: D23->D18    G: D15->D10    R:D7->D2    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input BGR  B: D23->D18    G: D15->D10    R:D0->D5    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input BGR  B: D23->D18    G: D8->D13    R:D7->D2    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input BGR  B: D16->D21    G: D15->D10    R:D7->D2    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            //RGB666 swap case
            #if 0   //input BGR  B: D17->D12    G: D11->D6    R:D5->D0    //8619C output  [606d]=a0 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x05);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0   //input BGR  B: D23->D18    G: D17->D12    R:D11->D6    //8619C output  [606d]=80 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x04);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            ucRxVidFmt = RGB_6Bit;
            pr_info("~~~9211 TTLRx Input Mode:    RGB666");
            break;
            
        case INPUT_RGB565:
            //RGB swap case
            #if 0   //input RGB  R: D23->D19    G: D15->D10    B:D7->D3    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input RBG  R: D23->D19    B: D15->D11    G:D7->D2    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input GRB  G: D23->D18    R: D15->D11    B:D7->D3    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input GBR  G: D23->D18    B: D15->D11    R:D7->D3    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input BRG  B: D23->D19    R: D15->D11    G:D7->D2    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input BGR  B: D23->D19    G: D15->D10    R:D7->D3    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input BGR  B: D23->D19    G: D15->D10    R:D0->D4    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input BGR  B: D23->D19    G: D8->D13    R:D7->D3    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input BGR  B: D16->D20    G: D15->D10    R:D7->D3    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            //RGB565 case
            #if 1   //input RGB  B: D15->D11    G: D10->D5    R: D4->D0    //8619C output  [606d]=e0 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x07);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0   //input RGB  B: D23->D19    G: D18->D13    R: D12->D8    //8619C output  [606d]=c0 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x06);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            ucRxVidFmt = RGB_6Bit;
            pr_info("~~~9211 TTLRx Input Mode:    RGB565");
            break;
        
        case INPUT_YCBCR444:
            //RGB swap case
            #if 1   //input YUV  V: D23->D16    Y: D15->D8    U:D7->D0    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input YUV  V: D23->D16    U: D15->D8    Y:D7->D0    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input YUV  Y: D23->D16    V: D15->D8    U:D7->D0    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input YUV  Y: D23->D16    U: D15->D8    V:D7->D0    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input YUV  U: D23->D16    V: D15->D8    Y:D7->D0    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input YUV  U: D23->D16    Y: D15->D8    V:D7->D0    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input YUV  U: D23->D16    Y: D15->D8    V:D0->D7    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input YUV  U: D23->D16    Y: D8->D15    V:D7->D0    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            #if 0   //input YUV  U: D16->D23    Y: D15->D8    V:D7->D0    //8619C output  [606d]=00 [606e]=80
                HDMI_WriteI2C_Byte(0xc5,0x74);
            #endif
            ucRxVidFmt = YUV444_8Bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR444");
            break;
        
        case INPUT_YCBCR422_8BIT:
            ucRxVidFmt = YUV422_8bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR422_8BIT");
            break;
        
        case INPUT_YCBCR422_10BIT:
            ucRxVidFmt = YUV422_10bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR422_10BIT");
            break;
        
        case INPUT_YCBCR422_12BIT:
            ucRxVidFmt = YUV422_12bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR422_12BIT");
            break;
        
        case INPUT_YCBCR422_16BIT:
            //RGB swap case
            #if 1   //input YUV  C: D23->D16    Y: D15->D8    //8619C output  [606d]=07 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0   //input YUV  C: D23->D16    Y:D7->D0    //8619C output  [606d]=06 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 0   //input YUV  Y: D23->D16    C: D15->D8    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0   //input YUV  Y: D23->D16    C:D7->D0    //8619C output  [606d]=04 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x60);
            #endif
            #if 0   //input YUV  C: D15->D8    Y:D7->D0    //8619C output  [606d]=03 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0   //input YUV  Y: D15->D8    C:D7->D0    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            //RGB high/low swap case
            #if 0   //input YUV  Y: D15->D8    C:D0->D7    //8619C output  [606d]=00 [606e]=20
                HDMI_WriteI2C_Byte(0xc5,0x71);
            #endif
            #if 0   //input YUV  Y: D8->D15    C:D7->D0    //8619C output  [606d]=00 [606e]=40
                HDMI_WriteI2C_Byte(0xc5,0x72);
            #endif
            ucRxVidFmt = YUV422_8bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR422_16BIT");
            break;
        
        case INPUT_YCBCR422_20BIT:
            //20Bit swap case
            #if 1  //input YUV  Y: D23->D14    C:D13->D4    //8619C output  [606d]=40 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x02);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0  //input YUV  Y: D19->D10    C:D9->D0    //8619C output  [606d]=60 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x03);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            //RGB swap case
            #if 0  //input YUV  Y: D15->D8  D19->D18    C:D8->D0  D23->D22    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D23->D16  D3->D2    C:D15->D6    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_10bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR422_20BIT");
            break;
        
        case INPUT_YCBCR422_24BIT:
            //24Bit swap case
            #if 1  //input YUV  Y: D23->D12    C:D11->D0    //8619C output  [606d]=20 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 0  //input YUV  C: D23->D12    Y:D11->D0    //8619C output  [606d]=28 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            //RGB swap case
            #if 0  //input YUV  Y: D15->D8  D19->D16    C:D7->D0  D23->D20    //8619C output  [606d]=00 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D23->D16  D3->D0    C:D15->D4    //8619C output  [606d]=05 [606e]=00
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_12bit;
            pr_info("~~~9211 TTLRx Input Mode:    YCBCR422_24BIT");
            break;
        
        case INPUT_BT656_8BIT:
            //RGB swap case
            #if 0  //input YUV  D7-D0    //8619C output  low 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 1  //input YUV  D15-D8    //8619C output middle 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0  //input YUV  D23-D16    //8619C output  high 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_8bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT656_8BIT");
            break;
        
        case INPUT_BT656_10BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  D9-D0    //8619C output  low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  D13-D4    //8619C output middle-low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            #if 0  //input YUV  D19-D10    //8619C output  middle-high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  D23-D14    //8619C output  high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            //other case
            #if 1  //input YUV  D15-D6    
                HDMI_WriteI2C_Byte(0xc6,0x01);
            #endif
            #if 0  //input YUV  D11-D2 
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_10bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT656_10BIT");
            break;
        
        case INPUT_BT656_12BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case
            #if 0  //input YUV  D11-D0    //8619C output  low 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0  //input YUV  D23-D12   //8619C output  high 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            //other case
            #if 1  //input YUV  D15-D4    
                HDMI_WriteI2C_Byte(0xc6,0x08);
            #endif
            ucRxVidFmt = YUV422_12bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT656_12BIT");
            break;
        
        case INPUT_BT1120_8BIT:
            //RGB swap case
            #if 0  //input YUV  D7-D0    //8619C output  low 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x30);
            #endif
            #if 1  //input YUV  D15-D8    //8619C output middle 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            #if 0  //input YUV  D23-D16    //8619C output  high 8Bit
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            ucRxVidFmt = YUV422_8bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT1120_8BIT");
            break;
        
        case INPUT_BT1120_10BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  D9-D0    //8619C output  low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  D13-D4    //9211C output  //8619C output middle-low 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            #if 0  //input YUV  D19-D10    //8619C output  middle-high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  D23-D14    //8619C output  high 10Bit
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            //other case
            #if 1  //input YUV  D15-D6    
                HDMI_WriteI2C_Byte(0xc6,0x01);
            #endif
            #if 0  //input YUV  D11-D2 
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_10bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT1120_10BIT");
            break;
        
        case INPUT_BT1120_12BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case
            #if 0  //input YUV  D11-D0    //8619C output  low 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0  //input YUV  D23-D12   //8619C output  high 12Bit
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            //other case
            #if 1  //input YUV  D15-D4    
                HDMI_WriteI2C_Byte(0xc6,0x08);
            #endif
            ucRxVidFmt = YUV422_12bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT1120_12BIT");
            break;
        
        case INPUT_BTA_T1004_16BIT:
            //RGB swap case
            #if 0  //input YUV  C: D7-D0    Y: D15-D8    //8619C /YC no swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D7-D0    C: D15-D8    //8619C /YC swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 0  //input YUV  Y: D23-D16    C: D15-D8    //8619C /YC no swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);
            #endif
            #if 1  //input YUV  C: D23-D16    Y: D15-D8    //8619C /YC swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            //other case
            #if 0  //input YUV  C: D23-D16    Y: D11-D4    
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            ucRxVidFmt = YUV422_8bit;
            pr_info("~~~9211 TTLRx Input Mode:    BTA_T1004_16BIT");
            break;
        
        case INPUT_BTA_T1004_20BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  C: D9-D0    Y: D19-D10    //8619C /YC no swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  Y: D9-D0    C: D19-D10    //8619C /YC swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  Y: D23-D14    C: D13-D4    //8619C /YC no swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            #if 0  //input YUV  C: D23-D14    Y: D13-D4    //8619C /YC swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            //other case
            #if 1 //input YUV  C: D23-D14    Y: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0 //input YUV  Y: D23-D14    C: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            ucRxVidFmt = YUV422_10bit;
            pr_info("~~~9211 TTLRx Input Mode:    BTA_T1004_20BIT"); // √√√√√√√√
            break;
        
        case INPUT_BTA_T1004_24BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case 
            #if 0 //input YUV  Y: D23-D12    C: D11-D0    //8619C /YC no swap output
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            #if 1 //input YUV  C: D23-D12    Y: D11-D0    //8619C /YC swap output
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_12bit;
            pr_info("~~~9211 TTLRx Input Mode:    BTA_T1004_24BIT");
            break;
        
        case INPUT_BT1120_16BIT: 
            //RGB swap case
            #if 0  //input YUV  C: D7-D0    Y: D15-D8    //8619C /YC no swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x70);
            #endif
            #if 0  //input YUV  Y: D7-D0    C: D15-D8    //8619C /YC swap /low 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x50);
            #endif
            #if 1  //input YUV  Y: D23-D16    C: D15-D8    //8619C /YC no swap /high 16bit output   // color die
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x40);        
            #endif
            #if 0  //input YUV  C: D23-D16    Y: D15-D8    //8619C /YC swap /high 16bit output
                HDMI_WriteI2C_Byte(0xc4,0x00);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            //other case
            #if 0  //input YUV  C: D23-D16    Y: D11-D4    
                HDMI_WriteI2C_Byte(0xc4,0x01);
                HDMI_WriteI2C_Byte(0xc5,0x00);
            #endif
            ucRxVidFmt = YUV422_8bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT1120_16BIT");
            break;
        
        case INPUT_BT1120_20BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x01); //rgd_bt_cd_10b_en
            //YC swap, 20Bit swap case
            #if 0  //input YUV  C: D9-D0    Y: D19-D10    //8619C /YC no swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x45);
            #endif
            #if 0  //input YUV  Y: D9-D0    C: D19-D10    //8619C /YC swap /low 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x05);
            #endif
            #if 0  //input YUV  Y: D23-D14    C: D13-D4    //8619C /YC no swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x44);
            #endif
            #if 0  //input YUV  C: D23-D14    Y: D13-D4    //8619C /YC swap /high 20bit output
                HDMI_WriteI2C_Byte(0xc6,0x04);
            #endif
            //other case
            #if 1 //input YUV  C: D23-D14    Y: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            #if 0 //input YUV  Y: D23-D14    C: D11-D2    
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            ucRxVidFmt = YUV422_10bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT1120_20BIT");
            break;
        
        case INPUT_BT1120_24BIT:
            HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x02); //rgd_bt_cd_12b_en
            //YC swap case 
            #if 0 //input YUV  Y: D23-D12    C: D11-D0    //8619C /YC no swap output
                HDMI_WriteI2C_Byte(0xc6,0x40);
            #endif
            #if 1 //input YUV  C: D23-D12    Y: D11-D0    //8619C /YC swap output
                HDMI_WriteI2C_Byte(0xc6,0x00);
            #endif
            ucRxVidFmt = YUV422_12bit;
            pr_info("~~~9211 TTLRx Input Mode:    BT1120_24BIT");
            break;
    }
    
    g_stChipRx.ucRxFormat = ucRxVidFmt;
    
}


void Drv_TtlRxCsc_Set(void)   // no ttl in mipi out ???
{PRINT_DEG("%s\n",__func__);
    #if ((LT9211_MODE_SEL == TTL_IN_LVDS_OUT)||(LT9211_MODE_SEL == TTL_IN_TTL_OUT))
        //ttlrx colorspace set
        #if TTLRX_COLORSPACE == RGB
            pr_info("~~~9211 Color Format:    RGB");
        #elif TTLRX_COLORSPACE == YUV444
            HDMI_WriteI2C_Byte(0xff,0x86);
            HDMI_WriteI2C_Byte(0x87,HDMI_ReadI2C_Byte(0x87) | 0x10);
            pr_info("~~~9211 Color Format:    YUV444");
        #elif TTLRX_COLORSPACE == YUV422
            HDMI_WriteI2C_Byte(0xff,0x86);
            HDMI_WriteI2C_Byte(0x87,HDMI_ReadI2C_Byte(0x87) | 0x10);
            HDMI_WriteI2C_Byte(0x86,HDMI_ReadI2C_Byte(0x86) | 0x04);
            pr_info("~~~9211 Color Format:    YUV422");
        #endif
    #endif
}


void Mod_TtlRx_DigSet(void)
{
    Drv_TtlRxHalfBit_Set();
    Drv_TtlRxDig_Set();                  
    Drv_TtlRxMapping_Set();       
    Drv_TtlRxCsc_Set();            
}


u8 Drv_TtlRx_Pll_Set(void)
{
    
    u8 ucloopx;         // Ñ­»·ÏµÊý
    u8 ucrtn = FALSE;   // rxpll×´Ì¬·µ»ØÖµ
        
    //ÅäÖÃRXPLL
    //»ù±¾ÅäÖÃ£ºTTL mode 1.8GHz/V VCO,3 order pll mode
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x1e,0x40);
    HDMI_WriteI2C_Byte(0x20,0x02);
    HDMI_WriteI2C_Byte(0x22,0x00);
    HDMI_WriteI2C_Byte(0x61,(HDMI_ReadI2C_Byte(0x61) | 0x02)); //rg_ttlrx_phdly_en = 1
    
    //DDR Ä£Ê½ÏÂ£¬ÐèÒª´ò¿ªDDR mode£¬¶ÔRxPLLÊä³öµÄd_clk½øÐÐ³Ë2´¦Àí
    #if TTLRX_DATARATE_MODE == DDR
        HDMI_WriteI2C_Byte(0x61,(HDMI_ReadI2C_Byte(0x61) | 0x04)); //DLL DDR mode enable
        HDMI_WriteI2C_Byte(0x23,(HDMI_ReadI2C_Byte(0x23) | 0x10)); //double dclk as pix clk
        //Ò»°ãDDRÄ£Ê½
        #if TTLRX_HALF_CLOCK_MODE == ENABLED
            pr_info("DDR Half Clock Mode:    Enable");
        #endif
        //DPIÄ£Ê½£¨ÐèÒª¶Ôdecddr_clkºÍpix_clk½øÐÐ³ý2´¦Àí£©
        #if TTLRX_HALF_WIDTH_MODE == ENABLED
            HDMI_WriteI2C_Byte(0x23,(HDMI_ReadI2C_Byte(0x23) | 0x04)); //rg_rxpll_decddr_divsel: div 2
        pr_info("DDR Half Width Mode:    Enable");
        #endif
    #endif
    
    //BT656 8BIT Ä£Ê½ÏÂ£¬ÐèÒª¶ÔRXPLLÊä³öµÄpix_clk½øÐÐ³ý2´¦Àí
    #if TTLRX_HALF_BIT_MODE == ENABLED
        HDMI_WriteI2C_Byte(0x21,HDMI_ReadI2C_Byte(0x21) | 0x01); //RxPll half pix clock output
        pr_info("Half Bit Mode:    Enable");
    #endif
    
    
    //rxpll ¸´Î»
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x03, (HDMI_ReadI2C_Byte(0x03) & 0x7f)); //rgd_rxpll_rst_n=0
    Ocm_Timer0_Delay1ms(1);
    HDMI_WriteI2C_Byte(0x03, (HDMI_ReadI2C_Byte(0x03) | 0x80)); //rgd_rxpll_rst_n=1
    Ocm_Timer0_Delay1ms(20);
    
    
    //¼ì²ârxpllµÄlockºÍcal×´Ì¬£¬3´Î»ú»á
    for(ucloopx = 0; ucloopx < 50; ucloopx++)
    {
        //rxpll ¸´Î»
        HDMI_WriteI2C_Byte(0xff,0x81);
        HDMI_WriteI2C_Byte(0x03,0x7f);
        Ocm_Timer0_Delay1ms(1);
        HDMI_WriteI2C_Byte(0x03,0xff);
        Ocm_Timer0_Delay1ms(10);
        
        //rxpll Ð£×¼vcocur
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x24,0x02);  //vcocurÑ¡ÔñÓ²¼þ¿ØÖÆ
        HDMI_WriteI2C_Byte(0xff,0x87);
        HDMI_WriteI2C_Byte(0x06,(HDMI_ReadI2C_Byte(0x06) | 0x01)); //rgd_rxpll_cal_en=1
        Ocm_Timer0_Delay1ms(300);
        
        //¼ì²ârxpllµÄlockºÍcal×´Ì¬
        // pr_info("9211 ---> 0x%2x", HDMI_ReadI2C_Byte(0x31)); // == 0x03
        if ((HDMI_ReadI2C_Byte(0x31) & 0x05) == 0x05)
        {
            pr_info("Rx Pll Cal done and Lock");
            pr_info("vmax, vmin = 0x%02bx 0x%02bx",HDMI_ReadI2C_Byte(0x30) & 0x3f,HDMI_ReadI2C_Byte(0x2f) & 0x3f);
            pr_info("vcoset = 0x%02bx",HDMI_ReadI2C_Byte(0x32) & 0x3f); //´òÓ¡vco_cur×Ô¶¯Ð£×¼Öµ
            ucrtn = TRUE;
            break;
        }
    }  
    
    return ucrtn;
}



void Drv_System_VidChkClk_SrcSel(u8 ucSrc)
{
    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x80,(HDMI_ReadI2C_Byte(0x80) & 0xfc));

    switch (ucSrc)
    {
        case RXPLL_PIX_CLK:
            HDMI_WriteI2C_Byte(0x80,(HDMI_ReadI2C_Byte(0x80) | RXPLL_PIX_CLK));
        break;
        case DESSCPLL_PIX_CLK:
            HDMI_WriteI2C_Byte(0x80,(HDMI_ReadI2C_Byte(0x80) | DESSCPLL_PIX_CLK));
        break;
        case RXPLL_DEC_DDR_CLK:
            HDMI_WriteI2C_Byte(0x80,(HDMI_ReadI2C_Byte(0x80) | RXPLL_DEC_DDR_CLK));
        break;
        case MLRX_BYTE_CLK:
            HDMI_WriteI2C_Byte(0x80,(HDMI_ReadI2C_Byte(0x80) | MLRX_BYTE_CLK));
        break;    
        
    }

}


void Drv_System_VidChk_SrcSel(u8 ucSrc)
{
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0x3f,(HDMI_ReadI2C_Byte(0x80) & 0xf8));

    switch (ucSrc)
    {
        case LVDSRX:
            HDMI_WriteI2C_Byte(0x3f,LVDSRX);
        break;
        case MIPIRX:
            HDMI_WriteI2C_Byte(0x3f,MIPIRX);
        break;
        case TTLRX:
            HDMI_WriteI2C_Byte(0x3f,TTLRX);
        break;
        case PATTERN:
            HDMI_WriteI2C_Byte(0x3f,PATTERN);
        break;
        case LVDSDEBUG:
            HDMI_WriteI2C_Byte(0x3f,LVDSDEBUG);
        case MIPIDEBUG:
            HDMI_WriteI2C_Byte(0x3f,MIPIDEBUG);
        break;
        case TTLDEBUG:
            HDMI_WriteI2C_Byte(0x3f,TTLDEBUG);
        break;    
        
    }

}


void Drv_TtlRx_TimingSet(StructChipRxVidTiming *VidTiming)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
    
    HDMI_WriteI2C_Byte(0xd0,(u8)(VidTiming->usHact>>8));       
    HDMI_WriteI2C_Byte(0xd1,(u8)(VidTiming->usHact));          
    HDMI_WriteI2C_Byte(0xd2,(u8)(VidTiming->usHbp>>8));        
    HDMI_WriteI2C_Byte(0xd3,(u8)(VidTiming->usHbp));           
    HDMI_WriteI2C_Byte(0xd4,(u8)(VidTiming->usHfp>>8));    
    HDMI_WriteI2C_Byte(0xd5,(u8)(VidTiming->usHfp)); 
    HDMI_WriteI2C_Byte(0xd6,(u8)(VidTiming->usHs>>8));    
    HDMI_WriteI2C_Byte(0xd7,(u8)(VidTiming->usHs));       
    HDMI_WriteI2C_Byte(0xd8,(u8)(VidTiming->usHtotal>>8));    
    HDMI_WriteI2C_Byte(0xd9,(u8)(VidTiming->usHtotal));   
    HDMI_WriteI2C_Byte(0xda,(u8)(VidTiming->usVact>>8));       
    HDMI_WriteI2C_Byte(0xdb,(u8)(VidTiming->usVact));          
    HDMI_WriteI2C_Byte(0xdc,(u8)(VidTiming->usVbp>>8));        
    HDMI_WriteI2C_Byte(0xdd,(u8)(VidTiming->usVbp));           
    HDMI_WriteI2C_Byte(0xde,(u8)(VidTiming->usVfp>>8));    
    HDMI_WriteI2C_Byte(0xdf,(u8)(VidTiming->usVfp)); 
    HDMI_WriteI2C_Byte(0xe0,(u8)(VidTiming->usVs>>8));    
    HDMI_WriteI2C_Byte(0xe1,(u8)(VidTiming->usVs));       

    #if TTLRX_VIDEO_FORMAT == I_FORMAT
        if(VidTiming->usVact == 1080)
        {
            HDMI_WriteI2C_Byte(0xe6,0x03);
            HDMI_WriteI2C_Byte(0xe7,0x8c);
        }
    #endif
}


void Drv_TtlRx_TimingSel(u8 uci)
{
    StructChipRxVidTiming VidTiming;
    
    VidTiming.usVtotal = resolution[uci].usVtotal;
    VidTiming.usVact   = resolution[uci].usVact;
    VidTiming.usVs     = resolution[uci].usVs;
    VidTiming.usVfp    = resolution[uci].usVfp;
    VidTiming.usVbp    = resolution[uci].usVbp;
    
    VidTiming.usHtotal = resolution[uci].usHtotal;
    VidTiming.usHact   = resolution[uci].usHact;
    VidTiming.usHs     = resolution[uci].usHs;
    VidTiming.usHfp    = resolution[uci].usHfp;
    VidTiming.usHbp    = resolution[uci].usHbp;
    
    Drv_TtlRx_TimingSet(&VidTiming);
    
    pr_info("self timing set:");
    pr_info("hfp, hs, hbp, hact, htotal = %d %d %d %d %d",VidTiming.usHfp, VidTiming.usHs, VidTiming.usHbp, VidTiming.usHact, VidTiming.usHtotal);
    pr_info("vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",VidTiming.usVfp, VidTiming.usVs, VidTiming.usVbp, VidTiming.usVact, VidTiming.usVtotal);
}



void Drv_TtlRx_SelfTimingSet(void)
{
    u16 ushtotal, usvtotal;
    u16 ushact, usvact;
    u8 uci;
    u8 ucResolutionnum = 0;
    u8 ucphase;
    
    //timing phase sel
    #if TTLRX_PHASE_SEL == 10
    pr_info("~~~9211 self timing phase check:");
    for(ucphase = 0; ucphase < 10; ucphase++)
    {
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x22,ucphase);
        
        HDMI_WriteI2C_Byte(0xff,0x86); 
        Ocm_Timer0_Delay1ms(100);
        usvtotal = HDMI_ReadI2C_Byte(0x62);
        usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);
        ushtotal = HDMI_ReadI2C_Byte(0x60);
        ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);
        usvact = HDMI_ReadI2C_Byte(0x5e);
        usvact = (usvact << 8) + HDMI_ReadI2C_Byte(0x5f);
        ushact = HDMI_ReadI2C_Byte(0x5c);
        ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);
        
        pr_info("phase: 0x%02bx", ucphase);
        pr_info("hact, htotal, vact, vtotal = %d %d %d %d",ushact, ushtotal, usvact, usvtotal);
    }
    #else
        ucphase = TTLRX_PHASE_SEL;
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x22,ucphase);
        
        HDMI_WriteI2C_Byte(0xff,0x86); 
        Ocm_Timer0_Delay1ms(100);
        usvtotal = HDMI_ReadI2C_Byte(0x62);
        usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);
        ushtotal = HDMI_ReadI2C_Byte(0x60);
        ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);
        usvact = HDMI_ReadI2C_Byte(0x5e);
        usvact = (usvact << 8) + HDMI_ReadI2C_Byte(0x5f);
        ushact = HDMI_ReadI2C_Byte(0x5c);
        ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);
        
        pr_info("************timing set phase: 0x%02bx", ucphase);
    #endif
    
    //flag check
    HDMI_WriteI2C_Byte(0xff,0x85); 
    if(HDMI_ReadI2C_Byte(0xbf) & 0x01)
    {
        ushact = (ushact - 2);
        pr_info("flag check:   BTA_T1004");
    }
    else if(HDMI_ReadI2C_Byte(0xbf) & 0x02)
    {
        ushact = (ushact - 2);
        pr_info("flag check:   BT656");
    }
    else if(HDMI_ReadI2C_Byte(0xbf) & 0x04)
    {
        ushact = (ushact - 4);
        pr_info("flag check:   8bit BT1120");
    }
    else if(HDMI_ReadI2C_Byte(0xbf) & 0x08)
    {
        ushact = (ushact - 4);
        pr_info("flag check:   16bit BT1120");
    }
    else
    {
        pr_info("No flag fit!");
    }
    pr_info("~~9211 debug:     hact, htotal, vact, vtotal = %d %d %d %d",ushact, ushtotal, usvact, usvtotal);
    
    //timing fit search
    ucResolutionnum = sizeof(resolution) / sizeof(resolution[0]);
    #if TTLRX_VIDEO_FORMAT == P_FORMAT
    for (uci = 0; uci < ucResolutionnum; uci++)
    {
        if (( ushact == resolution[uci].usHact ) && 
            ( usvact == resolution[uci].usVact ))
        {            
            pr_info("uci fit = 0x%02bx",uci);
            pr_info("ucihact, ucivact = %d %d", resolution[uci].usHact, resolution[uci].usVact);
            Drv_TtlRx_TimingSel(uci);
            break;
        }
    }
    #elif TTLRX_VIDEO_FORMAT == I_FORMAT
    for (uci = 0; uci < ucResolutionnum; uci++)
    {
        if ( (( ushtotal == resolution[uci].usHtotal ) && ( usvtotal == resolution[uci].usVtotal/2 )) ||
             (( ushtotal == resolution[uci].usHtotal ) && ( usvtotal == resolution[uci].usVtotal/2 + 1 )) )
        {
            Drv_TtlRx_TimingSel(uci);
            break;
        }
    }
    #endif
    
    if(uci == ucResolutionnum)
    {
        pr_info("~~9211 debug:    No timing fit!");
    }
 
}



#if TTLRX_DE_SYNC_MODE != 0
void Drv_TtlRx_DeSyncModeSet(void)
{
    u16 usVtotal,usVact,usVs,usVfp,usVbp;
    u16 usHtotal,usHact,usHs,usHfp,usHbp;
    u8 uci,ucResolutionnum;
    
    usHact = 3840;
    usVact = 2160;

    ucResolutionnum = sizeof(resolution) / sizeof(resolution[0]);
    
    for (uci = 0; uci < ucResolutionnum; uci++)
    {
        if (( usHact == resolution[uci].usHact ) && 
            ( usVact == resolution[uci].usVact ))
        {            
            pr_info("uci fit = 0x%02bx",uci);
            pr_info("ucihact, ucivact = %d %d", resolution[uci].usHact, resolution[uci].usVact);
            break;
        }
    }
    
    usVtotal = resolution[uci].usVtotal;
    usVact   = resolution[uci].usVact;
    usVs     = resolution[uci].usVs;
    usVfp    = resolution[uci].usVfp;
    usVbp    = resolution[uci].usVbp;
    
    usHtotal = resolution[uci].usHtotal;
    usHact   = resolution[uci].usHact;
    usHs     = resolution[uci].usHs;
    usHfp    = resolution[uci].usHfp;
    usHbp    = resolution[uci].usHbp;
    
    HDMI_WriteI2C_Byte(0xff,0x85);
    
    HDMI_WriteI2C_Byte(0xd0,(u8)(usHact>>8));       
    HDMI_WriteI2C_Byte(0xd1,(u8)(usHact));          
    HDMI_WriteI2C_Byte(0xd2,(u8)(usHbp>>8));        
    HDMI_WriteI2C_Byte(0xd3,(u8)(usHbp));           
    HDMI_WriteI2C_Byte(0xd4,(u8)(usHfp>>8));    
    HDMI_WriteI2C_Byte(0xd5,(u8)(usHfp)); 
    HDMI_WriteI2C_Byte(0xd6,(u8)(usHs>>8));    
    HDMI_WriteI2C_Byte(0xd7,(u8)(usHs));       
    HDMI_WriteI2C_Byte(0xd8,(u8)(usHtotal>>8));    
    HDMI_WriteI2C_Byte(0xd9,(u8)(usHtotal));   
    HDMI_WriteI2C_Byte(0xda,(u8)(usVact>>8));       
    HDMI_WriteI2C_Byte(0xdb,(u8)(usVact));          
    HDMI_WriteI2C_Byte(0xdc,(u8)(usVbp>>8));        
    HDMI_WriteI2C_Byte(0xdd,(u8)(usVbp));           
    HDMI_WriteI2C_Byte(0xde,(u8)(usVfp>>8));    
    HDMI_WriteI2C_Byte(0xdf,(u8)(usVfp)); 
    HDMI_WriteI2C_Byte(0xe0,(u8)(usVs>>8));    
    HDMI_WriteI2C_Byte(0xe1,(u8)(usVs)); 
    
    pr_info("self timing set:");
    pr_info("hfp, hs, hbp, hact, htotal = %d %d %d %d %d",usHfp, usHs, usHbp, usHact, usHtotal);
    pr_info("vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usVfp, usVs, usVbp, usVact, usVtotal);
    
    #if TTLRX_DE_SYNC_MODE == 1
        HDMI_WriteI2C_Byte(0xc7,HDMI_ReadI2C_Byte(0xc7) | 0x80); //de_gen enable
        pr_info("sync mode enable, generate de");
    #elif TTLRX_DE_SYNC_MODE == 2
        HDMI_WriteI2C_Byte(0xc7,HDMI_ReadI2C_Byte(0xc7) | 0x40); //sync_gen enable
        pr_info("de mode enable, generate sync");
    #endif
    #if TTLRX_VIDEO_FORMAT == P_FORMAT
        HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) & 0x7f);
    #elif TTLRX_VIDEO_FORMAT == I_FORMAT
        HDMI_WriteI2C_Byte(0xc8,HDMI_ReadI2C_Byte(0xc8) | 0x80);
    #endif

}
#endif




void Drv_TtlRx_VidChkDebug(void)
{
    u16 ushact, usvact;
    u16 ushs, usvs;
    u16 ushbp, usvbp;
    u16 ushtotal, usvtotal;
    u16 ushfp, usvfp;
    u8 ucsync_polarity;
    u8 ucphase;
    
    #if TTLRX_PHASE_SEL == 10
        for(ucphase = 0; ucphase < 10; ucphase++)
        {
            HDMI_WriteI2C_Byte(0xff,0x82);
            HDMI_WriteI2C_Byte(0x22,ucphase);
            
            Ocm_Timer0_Delay1ms(100);
            
            HDMI_WriteI2C_Byte(0xff,0x86);
        
            ucsync_polarity = HDMI_ReadI2C_Byte(0x4f);
            
            usvs = HDMI_ReadI2C_Byte(0x52); //[15:8]
            usvs = (usvs <<8 ) + HDMI_ReadI2C_Byte(0x53);

            ushs = HDMI_ReadI2C_Byte(0x50);
            ushs = (ushs << 8) + HDMI_ReadI2C_Byte(0x51);
            
            usvbp = HDMI_ReadI2C_Byte(0x57);
            usvfp = HDMI_ReadI2C_Byte(0x5b);

            ushbp = HDMI_ReadI2C_Byte(0x54);
            ushbp = (ushbp << 8) + HDMI_ReadI2C_Byte(0x55);

            ushfp = HDMI_ReadI2C_Byte(0x58);
            ushfp = (ushfp << 8) + HDMI_ReadI2C_Byte(0x59);

            usvtotal = HDMI_ReadI2C_Byte(0x62);
            usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);

            ushtotal = HDMI_ReadI2C_Byte(0x60);
            ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);

            usvact = HDMI_ReadI2C_Byte(0x5e);
            usvact = (usvact << 8)+ HDMI_ReadI2C_Byte(0x5f);

            ushact = HDMI_ReadI2C_Byte(0x5c);
            ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);

            pr_info("~~9211 debug:  phase: 0x%02bx", ucphase);
            pr_info("~~9211 debug:  sync_polarity = 0x%02bx", ucsync_polarity);
            pr_info("~~9211 debug:  hfp, hs, hbp, hact, htotal = %d %d %d %d %d",ushfp, ushs, ushbp, ushact, ushtotal);
            pr_info("~~9211 debug:  vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usvfp, usvs, usvbp, usvact, usvtotal);
        }
    #else
        ucphase = TTLRX_PHASE_SEL;
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x22,ucphase);
        
        Ocm_Timer0_Delay1ms(100);
        
        HDMI_WriteI2C_Byte(0xff,0x86);
    
        ucsync_polarity = HDMI_ReadI2C_Byte(0x4f);
        
        usvs = HDMI_ReadI2C_Byte(0x52); //[15:8]
        usvs = (usvs <<8 ) + HDMI_ReadI2C_Byte(0x53);

        ushs = HDMI_ReadI2C_Byte(0x50);
        ushs = (ushs << 8) + HDMI_ReadI2C_Byte(0x51);
        
        usvbp = HDMI_ReadI2C_Byte(0x57);
        usvfp = HDMI_ReadI2C_Byte(0x5b);

        ushbp = HDMI_ReadI2C_Byte(0x54);
        ushbp = (ushbp << 8) + HDMI_ReadI2C_Byte(0x55);

        ushfp = HDMI_ReadI2C_Byte(0x58);
        ushfp = (ushfp << 8) + HDMI_ReadI2C_Byte(0x59);

        usvtotal = HDMI_ReadI2C_Byte(0x62);
        usvtotal = (usvtotal << 8) + HDMI_ReadI2C_Byte(0x63);

        ushtotal = HDMI_ReadI2C_Byte(0x60);
        ushtotal = (ushtotal << 8) + HDMI_ReadI2C_Byte(0x61);

        usvact = HDMI_ReadI2C_Byte(0x5e);
        usvact = (usvact << 8)+ HDMI_ReadI2C_Byte(0x5f);

        ushact = HDMI_ReadI2C_Byte(0x5c);
        ushact = (ushact << 8) + HDMI_ReadI2C_Byte(0x5d);

        pr_info("~~9211 debug:  ************vedio check phase: 0x%02bx", ucphase);
        pr_info("~~9211 debug:  sync_polarity = 0x%02bx", ucsync_polarity);
        pr_info("~~9211 debug:  hfp, hs, hbp, hact, htotal = %d %d %d %d %d",ushfp, ushs, ushbp, ushact, ushtotal);
        pr_info("~~9211 debug:  vfp, vs, vbp, vact, vtotal = %d %d %d %d %d",usvfp, usvs, usvbp, usvact, usvtotal);
    #endif
    
}


static void Drv_SystemTxSram_Sel(void)
{
    //[7:6]2'b00: TX Sram sel MIPITX; others sel LVDSTX
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) & 0x3f)); 

    // switch(ucSrc)
    // {
    //     case LVDSTX:
    //         HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) | BIT6_1));
    //     break;
        // case MIPITX:
            HDMI_WriteI2C_Byte(0x30,(HDMI_ReadI2C_Byte(0x30) & BIT6_0));
        // break;
    // }
}




static void Mod_MipiTxVidFmt_Get(void)
{
    g_stMipiTx.ucTxFormat = g_stChipRx.ucRxFormat;
}

u32 Drv_System_FmClkGet(IN u8 ucSrc)
{
    u32 ulRtn = 0;
    HDMI_WriteI2C_Byte(0xff,0x86);
    HDMI_WriteI2C_Byte(0X90,ucSrc);
    Ocm_Timer0_Delay1ms(5);
    HDMI_WriteI2C_Byte(0x90,(ucSrc | BIT7_1));
    ulRtn = (HDMI_ReadI2C_Byte(0x98) & 0x0f);
    ulRtn = (ulRtn << 8) + HDMI_ReadI2C_Byte(0x99);
    ulRtn = (ulRtn << 8) + HDMI_ReadI2C_Byte(0x9a);
    HDMI_WriteI2C_Byte(0x90,(HDMI_ReadI2C_Byte(0x90) & BIT7_0));
    return ulRtn;
}

//========================================================================
// Func Name   : Drv_MipiTx_GetMipiInClk
// Description : Get Mipi In Clk
// Input       : INOUT StructMipiTx* pstMipiTx  
// Output      : INOUT StructMipiTx* pstMipiTx  
// Return      : u32
//========================================================================

u32 Drv_MipiTx_GetMipiInClk(INOUT StructMipiTx* pstMipiTx)
{
    u32 ulHalfPixClk = 0;
#if LT9211_MODE_SEL == MIPI_IN_MIPI_OUT
    ulHalfPixClk = Drv_System_FmClkGet(AD_DESSCPLL_PCR_CLK);
#endif

#if (LT9211_MODE_SEL == LVDS_IN_MIPI_OUT || LT9211_MODE_SEL == TTL_IN_MIPI_OUT)
    ulHalfPixClk = Drv_System_FmClkGet(AD_DESSCPLL_PIX_CLK) / 2;
#endif    
    
    if ((pstMipiTx->ulMipiInClk != ulHalfPixClk) && (0x00 != ulHalfPixClk))
    {
        pstMipiTx->ulMipiInClk = ulHalfPixClk;
    }
    return ulHalfPixClk;
}

//========================================================================
// Func Name   : Mod_MipiTx_ParaSet
// Description : ÉèÖÃ port, lane, clockburst, 3dmode
//               get MipiInClk
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Mod_MipiTx_ParaSet(void)
{
    u32 ulTmpClk ;

    Mod_MipiTxVidFmt_Get();
    g_stMipiTx.ucTxPortNum = SPort;
    g_stMipiTx.ucTxLaneNum = MIPITX_LANE_NUM;
    g_stMipiTx.b1MipiClockburst = MIPI_CLOCK_BURST;
    pr_info( "burst:0x%02bx", g_stMipiTx.b1MipiClockburst);

    switch (g_stMipiTx.ucTxFormat)
    {
        case RGB_6Bit:
            g_stMipiTx.ucBpp = 18;
            break;
        case RGB_8Bit:
        case YUV444_8Bit:
        case YUV420_8bit:
            g_stMipiTx.ucBpp = 24;
            break;
        case RGB_10Bit:
        case YUV444_10Bit:
        case YUV420_10bit:
            g_stMipiTx.ucBpp = 30;
            break;
        case RGB_12Bit:
            g_stMipiTx.ucBpp = 36;
            break;
        case YUV422_8bit:
            g_stMipiTx.ucBpp = 16;
            break;
        case YUV422_10bit:
            g_stMipiTx.ucBpp = 20;
            break;
        case YUV422_12bit:
            g_stMipiTx.ucBpp = 24;
            break;
        default:
            g_stMipiTx.ucBpp = 24;
            break;
    }

    Drv_MipiTx_GetMipiInClk(&g_stMipiTx);

    //MIPI D-PHY check either use 2port or not
    ulTmpClk = g_stMipiTx.ulMipiInClk * g_stMipiTx.ucBpp * 2 / (g_stMipiTx.ucTxLaneNum);
    if (ulTmpClk > MIPITX_PLL_HIGH)
    {
        pr_info("LOG_ERROR 9211:  Over Maximum MIPITX Datarate!!! error");
    }

    //csi port adjust
//    Mod_MipiTx_PortLane_Adj();
}

//========================================================================
// Func Name   : Mod_MipiTx_DataRateAdj
// Description : calc DPHY data rate, limit up && low
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Mod_MipiTx_DataRateAdj(void)
{
    //MIPI D-PHY
    //DPHY is 8bit to 8bit
    g_stMipiTx.ulMipiDataRate = g_stMipiTx.ulMipiInClk * g_stMipiTx.ucBpp * 2 / (g_stMipiTx.ucTxPortNum * g_stMipiTx.ucTxLaneNum);

    //clkÁ¬ÐøÊ±£¬¼Ó¿ì80M£¬clk·ÇÁ¬ÐøÊ±£¬¼Ó¿ì120M£¬Èç¹ûË«portÔÙ¼Ó¿ì40M£¬ÒÔÂú×ãDPHY timing LPºÍHSµÄÇÐ»»µÄÐèÇó
    if(g_stMipiTx.b1MipiClockburst)
    {
        g_stMipiTx.ulMipiDataRate += 120000;
    }
    else
    {
        g_stMipiTx.ulMipiDataRate += 80000;
    }


    if(g_stMipiTx.ulMipiDataRate < MIPITX_PLL_LOW) //set byteclk minium value to 50M , phyclk minium value is 80M 
    {
        g_stMipiTx.ulMipiDataRate = MIPITX_PLL_LOW ;
    }
    if(g_stMipiTx.ulMipiDataRate > MIPITX_PLL_HIGH)//set byteclk maxmum value to 312.5M , phyclk maxmum value is 2.5G 
    {
        g_stMipiTx.ulMipiDataRate = MIPITX_PLL_HIGH ;
    }
}


//========================================================================
// Func Name   : Drv_MipiTx_DPhySet
// Description : MIPI Tx DPHY Setting
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_DPhySet(void)
{
    HDMI_WriteI2C_Byte(0xff,0x82);
    #if (MIPITX_PORT_SEL == PORTA)
    HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) | 0x01)); //0x00: disabled, 0x01: portA en
                                                                //0x02: portB en, 0x03: portA & B en
    pr_info("~~9211 debug:  MIPI Output PortA");
    #elif (MIPITX_PORT_SEL == PORTB)
    HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) | 0x02));
    pr_info("~~9211 debug:  MIPI Output PortB");
    #elif (MIPITX_PORT_SEL == DOU_PORT)
    HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) | 0x03));
    pr_info("~~9211 debug:  MIPI Output PortA & B");
    #endif
    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x53,0xee);

    
    //mipi tx phy cts test require
    //below setting are used for mipitx D-phy cts 1.3.10 when datarate 2.5gbps
    if ((g_stMipiTx.ulMipiDataRate > 1500000) || (g_stMipiTx.ulMipiDataRate <= 1050000)) //mipitx D-Dphy cts 1.4.17 test fail when datarate less than 1.5Gbps if use those setting
    {
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x37,0x11);
        HDMI_WriteI2C_Byte(0x39,0x32);
        HDMI_WriteI2C_Byte(0x3a,0xc6);
        HDMI_WriteI2C_Byte(0x3b,0x21);

        HDMI_WriteI2C_Byte(0x46,0x4c);
        HDMI_WriteI2C_Byte(0x47,0x4c);
        HDMI_WriteI2C_Byte(0x48,0x48);
        HDMI_WriteI2C_Byte(0x49,0x4c);
        HDMI_WriteI2C_Byte(0x4a,0x4c);
        HDMI_WriteI2C_Byte(0x4b,0x4c);
        HDMI_WriteI2C_Byte(0x4c,0x4c);
        HDMI_WriteI2C_Byte(0x4d,0x48);
        HDMI_WriteI2C_Byte(0x4e,0x4c);
        HDMI_WriteI2C_Byte(0x4f,0x4c);
    }
}




//========================================================================
// Func Name   : Drv_MipiTx_PllSet
// Description : pll calc loop divider according calc_datarate
// Input       : IN StructMipiTx* pstMipiTx  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_PllSet(IN StructMipiTx* pstMipiTx)
{
    u32 ulMpiTXPhyClk;
    u8 ucSericlkDiv , ucPreDiv, ucDivSet;

    //MIPI D-PHY datarate use full speed <= 1.5Gbps for decrease clk jitter, to solve mipi tx phy cts_1.4.18.
    //use half speed when datarate > 1.5Gbps
    if (g_stMipiTx.ulMipiDataRate <= 1500000)
    {
        ulMpiTXPhyClk = pstMipiTx->ulMipiDataRate;
        HDMI_WriteI2C_Byte(0xff,0x82);
        HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) & BIT4_0));
    }
    else
    {
        ulMpiTXPhyClk = pstMipiTx->ulMipiDataRate / 2;
    }
    
    //txpll sericlk div default use 0x00(DIV2) because mipi tx phy use half rate mode
    HDMI_WriteI2C_Byte(0xff,0x82);

    if (ulMpiTXPhyClk >= 640000 )//640M~1.28G
    {
        ucSericlkDiv = 1; //sericlk div1 [6:4]:0x40
        HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & 0x8f) | 0x40);
    }
    else if (ulMpiTXPhyClk >= 320000 && ulMpiTXPhyClk < 640000)
    {
        ucSericlkDiv = 2; //sericlk div2 [6:4]:0x00
        HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & 0x8f));
    }
    else if (ulMpiTXPhyClk >= 160000 && ulMpiTXPhyClk < 320000)
    {
        ucSericlkDiv = 4; //sericlk div4 [6:4]:0x10
        HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & 0x8f) | 0x10);
    }
    else if (ulMpiTXPhyClk >= 80000 && ulMpiTXPhyClk < 160000)
    {
        ucSericlkDiv = 8; //sericlk div8 [6:4]:0x20
        HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & 0x8f) | 0x20);
    }
    else //40M~80M
    {
        ucSericlkDiv = 16; //sericlk div16 [6:4]:0x30
        HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & 0x8f) | 0x30);
    }

    pr_info( "~~9211 debug:  ulMipiDataRate:%ld, ulHalfPixClk:%ld, ulMpiTXPhyClk:%ld, ucBpp:%bd, ucTxPortNum:0x%02bx, ucTxLaneNum:0x%02bx",
                pstMipiTx->ulMipiDataRate,pstMipiTx->ulMipiInClk, ulMpiTXPhyClk, pstMipiTx->ucBpp, pstMipiTx->ucTxPortNum,pstMipiTx->ucTxLaneNum);

    //prediv_set N1 = 1
    ucPreDiv = 1;
    
    //div set
    //Vcoclk=byte_clk*4*M3=25M*M1*M2*ucSericlkDiv(Òì²½Ä£Ê½), M2 default value is 2;
    ucDivSet = (u8)(ulMpiTXPhyClk * ucSericlkDiv / 25000);

    HDMI_WriteI2C_Byte(0xff,0x82);
    HDMI_WriteI2C_Byte(0x30,0x02); //[7]0:txpll normal work
                                   //[2:1]Lmtxpll reference clock selection:Xtal clock;
    HDMI_WriteI2C_Byte(0x31,0x28);
    HDMI_WriteI2C_Byte(0x32,(HDMI_ReadI2C_Byte(0x32) & 0xf3)); //tx pll post div set DIV1
    HDMI_WriteI2C_Byte(0x34,0x01);
    HDMI_WriteI2C_Byte(0x35,ucDivSet);

    pr_info("~~9211 debug:  ucSericlkDiv N1:0x%02bx, ucDivSet M2:0x%02bx",ucSericlkDiv, ucDivSet);
}



//========================================================================
// Func Name   : Drv_MipiTx_PllCali
// Description : first pll cali, then phy set
// Input       : void  
// Output      : None
// Return      : u8
//========================================================================
u8 Drv_MipiTx_PllCali(void)
{
    u8 ucPllScanCnt = 0;
    u8 ucRtn = FALSE;
    Ocm_Timer0_Delay1ms(20);
    do 
    {    
        HDMI_WriteI2C_Byte(0xff,0x81);//register bank   
        HDMI_WriteI2C_Byte(0x0c,0x78);//tx pll rest cal_rst =0
        HDMI_WriteI2C_Byte(0xff,0x87);//register bank
        HDMI_WriteI2C_Byte(0x0f,0x00);//tx pll cal = 0;
        
        HDMI_WriteI2C_Byte(0xff,0x81);//register bank
        HDMI_WriteI2C_Byte(0x0c,0xf9);//tx pll rest cal_rst =1
        HDMI_WriteI2C_Byte(0xff,0x87);//register bank
        HDMI_WriteI2C_Byte(0x0f,0x01);//tx pll cal = 0;
        Ocm_Timer0_Delay1ms(10);
        ucPllScanCnt++;
    }while((ucPllScanCnt < 3) && ((HDMI_ReadI2C_Byte(0x39) & 0x07) != 0x05));//PLL calibration done status

    if((HDMI_ReadI2C_Byte(0x39) & 0x07)== 0x05)
    {
        ucRtn = SUCCESS;
        pr_info("~~9211 debug:  Tx Pll Lock");
    }
    else
    {
        ucRtn = FAIL;
        pr_info("~~9211 LOG_ERROR: Tx Pll Unlocked");
    }
    return ucRtn;
}

//========================================================================
// Func Name   : Drv_MipiTx_HalfWrClkSrc_Sel
// Description : MPTX write clk select:
//                0 = Select half write clk;
//                1 = Select write clk.
// Input       : IN u8 clock  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_HalfWrClkSrc_Sel(IN u8 clock)
{
    HDMI_WriteI2C_Byte(0xff,0x82);
    
    if (clock == HALF_WRITE_CLK)
    {
        HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) | 0x80));
    }
    else
    {
        HDMI_WriteI2C_Byte(0x36,(HDMI_ReadI2C_Byte(0x36) & 0x7f));
    }
}

u16 Drv_VidChkSingle_Get(u8 ucPara)
{ 
    u16 usRtn = 0;

    HDMI_WriteI2C_Byte(0xff,0x81);
    HDMI_WriteI2C_Byte(0x0b,0x7f);
    HDMI_WriteI2C_Byte(0x0b,0xff);
    Ocm_Timer0_Delay1ms(80);
    HDMI_WriteI2C_Byte(0xff,0x86);
    switch(ucPara)
    {
        case HTOTAL_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x60) << 8) + HDMI_ReadI2C_Byte(0x61);
        break;
        case HACTIVE_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x5c) << 8) + HDMI_ReadI2C_Byte(0x5d);  
        break;
        case HFP_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x58) << 8) + HDMI_ReadI2C_Byte(0x59);
        break;
        case HSW_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x50) << 8) + HDMI_ReadI2C_Byte(0x51);
        break;    
        case HBP_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x54) << 8) + HDMI_ReadI2C_Byte(0x55);
        break;
        case VTOTAL_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x62) << 8) + HDMI_ReadI2C_Byte(0x63);
        break;
        case VACTIVE_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x5e) << 8) + HDMI_ReadI2C_Byte(0x5f);
        break;
        case VFP_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x5a) << 8) + HDMI_ReadI2C_Byte(0x5b);
        break;
        case VSW_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x52) << 8) + HDMI_ReadI2C_Byte(0x53);
        break;
        case VBP_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x56) << 8) + HDMI_ReadI2C_Byte(0x57);
        break;
        case HSPOL_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x4f) & 0x01);
        break;
        case VSPOL_POS:
            usRtn = (HDMI_ReadI2C_Byte(0x4f) & 0x02);
        break;
        default:
        break;
    }
    return usRtn;
}
unsigned char Drv_VidChk_FrmRt_Get(void)
{
    u8 ucframerate = 0; 
    u32 ulframetime = 0;

    HDMI_WriteI2C_Byte(0xff,0x86);
    ulframetime = HDMI_ReadI2C_Byte(0x43);
    ulframetime = (ulframetime << 8) + HDMI_ReadI2C_Byte(0x44);
    ulframetime = (ulframetime << 8) + HDMI_ReadI2C_Byte(0x45);
    // ucframerate = (u8)(((float)25000000 / (float)(ulframetime)) + (float)(0.5)); //2500000/ulframetime
    ucframerate = (unsigned char)(((unsigned int)25000000 / (unsigned int)(ulframetime))); //2500000/ulframetime

    return ucframerate;
}

// unsigned char Drv_VidChk_FrmRt_Get(void)
// {
//     unsigned char ucframerate = 0; 
//     unsigned int ulframetime = 0;
//     lt9211_write(pdata->client,0xff,0x86);
//     ulframetime = lt9211_read(pdata->client,0x43);
//     ulframetime = (ulframetime << 8) + lt9211_read(pdata->client,0x44);
//     ulframetime = (ulframetime << 8) + lt9211_read(pdata->client,0x45);
//     ucframerate = (unsigned char)(((unsigned int)25000000 / (unsigned int)(ulframetime))); //2500000/ulframetime
//     return ucframerate;
// }



void Drv_VidChkAll_Get(OUT StructVidChkTiming *video_time)
{
    video_time->usHtotal    =     Drv_VidChkSingle_Get(HTOTAL_POS);
    video_time->usHact      =     Drv_VidChkSingle_Get(HACTIVE_POS);
    video_time->usHfp       =     Drv_VidChkSingle_Get(HFP_POS);
    video_time->usHs        =     Drv_VidChkSingle_Get(HSW_POS);
    video_time->usHbp       =     Drv_VidChkSingle_Get(HBP_POS);
    
    video_time->usVtotal    =     Drv_VidChkSingle_Get(VTOTAL_POS);
    video_time->usVact      =     Drv_VidChkSingle_Get(VACTIVE_POS);
    video_time->usVfp       =     Drv_VidChkSingle_Get(VFP_POS);
    video_time->usVs        =     Drv_VidChkSingle_Get(VSW_POS);
    video_time->usVbp       =     Drv_VidChkSingle_Get(VBP_POS);
    
    video_time->ucHspol     =     Drv_VidChkSingle_Get(HSPOL_POS);
    video_time->ucVspol     =     Drv_VidChkSingle_Get(VSPOL_POS);        
    video_time->ucFrameRate =     Drv_VidChk_FrmRt_Get(); 
    PRINT_DEG("~~9211 debug:  Htotal=%u Hact=%u Hfp=%u Hs=%u Hbp=%u Vtotal=%u Vact=%u Vfp=%u Vs=%u Vbp=%u FrameRate=%u \n",
        video_time->usHtotal, video_time->usHact, video_time->usHfp, video_time->usHs, video_time->usHbp, 
        video_time->usVtotal,  video_time->usVact, video_time->usVfp, video_time->usVs, video_time->usVbp, video_time->ucFrameRate);
}
//========================================================================
// Func Name   : Drv_MipiTx_TimingSet
// Description : mipi tx timing set
// Input       : IN StructVidChkTiming *pstVidTiming  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_TimingSet(IN StructVidChkTiming *pstVidTiming)
{
    u16 ushss, usvss;
    u16 us3d_dly;
 
    ushss = pstVidTiming->usHs + pstVidTiming->usHbp;
    usvss = pstVidTiming->usVs + pstVidTiming->usVbp;
    
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x7e,(u8)(pstVidTiming->usHact >> 8));
    HDMI_WriteI2C_Byte(0x7f,(u8)pstVidTiming->usHact);
    HDMI_WriteI2C_Byte(0x7c,(u8)(pstVidTiming->usVact >> 8));
    HDMI_WriteI2C_Byte(0x7d,(u8)pstVidTiming->usVact);
    HDMI_WriteI2C_Byte(0x84,(u8)(HDMI_ReadI2C_Byte(0x84) | ((ushss >> 8) & 0xfc)));
    HDMI_WriteI2C_Byte(0x85,(u8)ushss);
    HDMI_WriteI2C_Byte(0x80,(u8)(usvss >> 8));
    HDMI_WriteI2C_Byte(0x81,(u8)usvss);

    //3d dly
    us3d_dly = pstVidTiming->usHact / 4;
    HDMI_WriteI2C_Byte(0x7a,(u8)(us3d_dly >> 8));
    HDMI_WriteI2C_Byte(0x7b,(u8)us3d_dly);
}


//========================================================================
// Func Name   : Drv_MipiTx_InHSyncPol_Sel
// Description : Rx     hsync/vsync 1:positive 0:negative
//               MIPITX hsync/vsync 1:negative 0:positive
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_InHSyncPol_Sel(IN u8 b1SyncPol)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    if (b1SyncPol == NEGITVE)
    {
        HDMI_WriteI2C_Byte(0x70,(HDMI_ReadI2C_Byte(0x70) | BIT1_1));
    }
    else
    {
        HDMI_WriteI2C_Byte(0x70,(HDMI_ReadI2C_Byte(0x70) & BIT1_0));
    }
}

//========================================================================
// Func Name   : Drv_MipiTx_InVSyncPol_Sel
// Description : Rx     hsync/vsync 1:positive 0:negative
//               MIPITX hsync/vsync 1:negative 0:positive
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_InVSyncPol_Sel(IN u8 b1SyncPol)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    if (b1SyncPol == NEGITVE)
    {
        HDMI_WriteI2C_Byte(0x70,(HDMI_ReadI2C_Byte(0x70) | BIT0_1));
    }
    else
    {
        HDMI_WriteI2C_Byte(0x70,(HDMI_ReadI2C_Byte(0x70) & BIT0_0));
    }
}

//========================================================================
// Func Name   : Mod_MipiTx_Resolution_Config
// Description : get vid chk timing, set mipi resolution config
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Mod_MipiTx_Resolution_Config(void)
{
    Drv_VidChkAll_Get(&g_stVidChk);
    Drv_MipiTx_TimingSet(&g_stVidChk);
    Drv_MipiTx_InHSyncPol_Sel(g_stVidChk.ucHspol);
    Drv_MipiTx_InVSyncPol_Sel(g_stVidChk.ucVspol);
}


//========================================================================
// Func Name   : Drv_MipiTx_PhyTimingParaSet
// Description : mipi phy timing set
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_PhyTimingParaSet(INOUT StructMipiTx* pstMipiTx, INOUT StructMipiTxDPhy* pstMipiTxDPhy)
{

    u32 ulrdbyteclk  = 0;
    
    ulrdbyteclk = Drv_System_FmClkGet(AD_MLTX_WRITE_CLK);
    ulrdbyteclk = ulrdbyteclk / 1000;
    pr_info( "~~9211 debug:  byteclk: %ldM", ulrdbyteclk);

    g_stMipiTxDPhy.ucClkPre = 0x02;
    pstMipiTxDPhy->ucHsLpx   = ulrdbyteclk * 6 / 100 + 1; //hs lpx > 50ns
    pstMipiTxDPhy->ucHsPrep  = ulrdbyteclk * 6 / 100; //hs prep : (40ns + 4*UI)~(85ns + 6*UI) , clk_prepare
    pstMipiTxDPhy->ucHsTrail = ulrdbyteclk * 7 / 100 + 4; //hs_trail and clk_trail: max(8UI , 60ns + 4UI),   +2->+3
    pstMipiTxDPhy->ucClkPost = ulrdbyteclk * 7 / 100 + 7;//ck_post > 60ns + 52UI,  +4->+15->+1
        
    if(pstMipiTx->b1MipiClockburst)
    {

        //·ÇÁ¬ÐøÊ±ÖÓ´Ó1.94G~2.5GÏÂ£¬clk_zero¸ù¾Ý¹Ì¶¨¹«Ê½ÎÞ·¨³öÍ¼
        if (pstMipiTx->ulMipiDataRate > CTS_DATARATE)
        {
            pstMipiTxDPhy->ucClkZero = 0x05;
        }
        else
        {
            pstMipiTxDPhy->ucClkZero = ulrdbyteclk * 6 / 25;    //ck_zero > 300 - ck_prpr , old: rdbyteclk/4      
        }
        
        pstMipiTxDPhy->ucHsRqStPre = g_stMipiTxDPhy.ucHsLpx + g_stMipiTxDPhy.ucHsPrep + g_stMipiTxDPhy.ucClkZero + g_stMipiTxDPhy.ucClkPre;
    }
    else
    {
        pstMipiTxDPhy->ucClkZero   = ulrdbyteclk * 6 / 25;    //ck_zero > 300 - ck_prpr , old: rdbyteclk/4      
        pstMipiTxDPhy->ucHsRqStPre = ulrdbyteclk / 10;
    }
    
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x8a,pstMipiTxDPhy->ucHsRqStPre);
    HDMI_WriteI2C_Byte(0xa4,pstMipiTxDPhy->ucHsLpx);
    HDMI_WriteI2C_Byte(0xa5,pstMipiTxDPhy->ucHsPrep);
    HDMI_WriteI2C_Byte(0xa6,pstMipiTxDPhy->ucHsTrail);
    HDMI_WriteI2C_Byte(0xa7,pstMipiTxDPhy->ucClkZero);
    HDMI_WriteI2C_Byte(0xa9,pstMipiTxDPhy->ucClkPost);

    pr_info( "~~9211 debug:  ck_post (0xD4A9) = 0x%02bx", pstMipiTxDPhy->ucClkPost);
    pr_info( "~~9211 debug:  ck_zero (0xD4A7) = 0x%02bx", pstMipiTxDPhy->ucClkZero);
    pr_info( "~~9211 debug:  hs_lpx  (0xD4A4) = 0x%02bx", pstMipiTxDPhy->ucHsLpx);
    pr_info( "~~9211 debug:  hs_prep (0xD4A5) = 0x%02bx", pstMipiTxDPhy->ucHsPrep);
    pr_info( "~~9211 debug:  hs_trail(0xD4A6) = 0x%02bx", pstMipiTxDPhy->ucHsTrail);
    pr_info( "~~9211 debug:  hs_rqst (0xD48A) = 0x%02bx", pstMipiTxDPhy->ucHsRqStPre);
}



//========================================================================
// Func Name   : Drv_MipiTx_LaneSet
// Description : mipi tx lane set
//                    MIPI lane mode:
//                    2'b00 = 4lane;
//                    2'b01 = 1lane;
//                    2'b10 = 2lane;
//                    2'b11 = 3lane.
// Input       : u8 LaneNum  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_LaneSet(IN u8 LaneNum)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) & 0xfc));
    if(LaneNum != MIPITX_4LANE)
    {
        HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) | LaneNum));
    }
}

//========================================================================
void Drv_MipiTx_PortCopy(void)
{

    #if ((MIPITX_PORT_COPY == ENABLED) || (MIPITX_PORT_SEL == PORTA))
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x4a,(HDMI_ReadI2C_Byte(0x4a) | 0x40));
    HDMI_WriteI2C_Byte(0x50,(HDMI_ReadI2C_Byte(0x50) | 0x40));
    #endif
}
void Drv_MipiTx_PortDataEnable(void)
{
    HDMI_WriteI2C_Byte(0xff,0x85);
    HDMI_WriteI2C_Byte(0x4b,(HDMI_ReadI2C_Byte(0x4b) | 0x02)); //portA lane0
    HDMI_WriteI2C_Byte(0x4c,(HDMI_ReadI2C_Byte(0x4c) | 0x01)); //portA lane1
    HDMI_WriteI2C_Byte(0x4d,(HDMI_ReadI2C_Byte(0x4d) | 0x01)); //portA lane2
    HDMI_WriteI2C_Byte(0x4e,(HDMI_ReadI2C_Byte(0x4e) | 0x01)); //portA lane3
    HDMI_WriteI2C_Byte(0x4f,(HDMI_ReadI2C_Byte(0x4f) | 0x01)); //portA lane4

    HDMI_WriteI2C_Byte(0x51,(HDMI_ReadI2C_Byte(0x51) | 0x02)); //portB lane0
    HDMI_WriteI2C_Byte(0x52,(HDMI_ReadI2C_Byte(0x52) | 0x01)); //portB lane1
    HDMI_WriteI2C_Byte(0x53,(HDMI_ReadI2C_Byte(0x53) | 0x01)); //portB lane2
    HDMI_WriteI2C_Byte(0x54,(HDMI_ReadI2C_Byte(0x54) | 0x01)); //portB lane3
    HDMI_WriteI2C_Byte(0x55,(HDMI_ReadI2C_Byte(0x55) | 0x01)); //portB lane4
}

void Drv_MipiTx_DPHYClkData_Set(void)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0xaa,0xAA);
}

//========================================================================
// Func Name   : Drv_MipiTx_DPHYClkMode_Sel
// Description : CLOCK LANE non-burst enable.
// Input       : IN u8 b1MipiClockburst  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_DPHYClkMode_Sel(IN u8 b1IsMipiClockburst)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    if (b1IsMipiClockburst)
    {
        HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) | 0x80));
    }
    else
    {
        HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) & 0x7f));
    }
}

void Drv_MipiTx_DPHYCSI8Lane_En(IN u8 b1IsEn)
{
    HDMI_WriteI2C_Byte(0Xff,0xd4);

    if (b1IsEn)
    {
        HDMI_WriteI2C_Byte(0Xa0,(HDMI_ReadI2C_Byte(0xa0) | 0x40));
    }
    else
    {
        HDMI_WriteI2C_Byte(0Xa0,(HDMI_ReadI2C_Byte(0xa0) & 0xbf));
    }
}

void Drv_MipiTx_PortSet(IN u8 ucTxPortNum)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x87,(HDMI_ReadI2C_Byte(0x87) | ucTxPortNum)); //single port
}



//========================================================================
// Func Name   : Drv_MipiTx_CsiDataTypeSet
// Description : CSI DataType Set
// Input       : IN u8 ucTxFormat  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_CsiDataTypeSet(IN u8 ucTxFormat)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    switch (ucTxFormat)
    {
        case RGB_6Bit : //bpp18
            HDMI_WriteI2C_Byte(0x88,0x11);
            HDMI_WriteI2C_Byte(0x86,0x23); //csi_rgb666         
            break;
        case RGB_8Bit : //bpp24
            HDMI_WriteI2C_Byte(0x88,0x25);
            HDMI_WriteI2C_Byte(0x86,0x24); //csi_rgb888         
            break;
        case RGB_10Bit : //bpp30
            break;
        case YUV422_8bit ://bpp16
            HDMI_WriteI2C_Byte(0x88,0x01);
            HDMI_WriteI2C_Byte(0x86,0x1e); //yuv16
            break;
        case YUV422_10bit ://bpp20
            HDMI_WriteI2C_Byte(0x88,0x31);
            HDMI_WriteI2C_Byte(0x86,0x1f); //Y422_1_10
            break;
        case YUV422_12bit: //bpp24
        case YUV444_8Bit : //bpp24
            HDMI_WriteI2C_Byte(0x88,0x23);
            HDMI_WriteI2C_Byte(0x86,0x1c); //yuv24
            break;
        case YUV420_8bit :
            HDMI_WriteI2C_Byte(0x88,0x20);
            HDMI_WriteI2C_Byte(0x86,0x1a); //Y420_3_legacy
            break;
        case YUV420_10bit :
            HDMI_WriteI2C_Byte(0x88,0x41);
            HDMI_WriteI2C_Byte(0x86,0x19); //Y420_3_10bit
            break;
    }
}


char* g_szStrTxFormat[MIPITX_FORMAT_CNT] = 
{
    "RGB 6bit",
    "RGB 8bit",
    "RGB 10bit",
    "RGB 12bit",
    "YUV444 8Bit",
    "YUV444 10bit",
    "YUV422 8bit",
    "YUV422 10bit",
    "YUV422 12bit",
    "YUV420 8bit",
    "YUV420 10bit",
};

//========================================================================
// Func Name   : Mod_MipiTx_Digital_Config
// Description : mipi tx digital config
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Mod_MipiTx_Digital_Config(void)
{
    Drv_MipiTx_LaneSet(g_stMipiTx.ucTxLaneNum);
    Drv_MipiTx_PortCopy();
    Drv_MipiTx_PortDataEnable();
    Drv_MipiTx_DPHYClkData_Set();
    Drv_MipiTx_DPHYClkMode_Sel(g_stMipiTx.b1MipiClockburst);

    if (g_stMipiTx.b1DphyCsi8Lane == ENABLED)
    {
        Drv_MipiTx_HalfWrClkSrc_Sel(WRITE_CLK);
        Drv_MipiTx_DPHYCSI8Lane_En(ENABLED);
    }

    Drv_MipiTx_PortSet(g_stMipiTx.ucTxPortNum);

    #if (MIPITX_OUT_SEL == MIPI_DSI)
    Drv_MipiTx_DsiDataTypeSet(g_stMipiTx.ucTxFormat);
    Drv_MipiTx_DcsAutoMode();
    #else
    Drv_MipiTx_CsiDataTypeSet(g_stMipiTx.ucTxFormat);
    #endif
    pr_info( "~~9211 debug: MipiTx Output Format: %s",g_szStrTxFormat[g_stMipiTx.ucTxFormat]);
}

//========================================================================
// Func Name   : Drv_MipiTx_Hss_Set
// Description : Hss Set
// Input       : IN u16 value  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_Hss_Set(IN u16 usVal)
{
    if( usVal > 0x3FF )
    {
        usVal = 0x3FF;
    }

    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x84,(u8)(HDMI_ReadI2C_Byte(0x84) | (usVal >> 8)));
    HDMI_WriteI2C_Byte(0x85,(u8)usVal);
}


void Mod_MipiTx_HssSet(void)
{
    u16 Hss;
    #if (MIPITX_OUT_SEL == MIPI_DSI)
    Hss = 0x0A; //Hss
    #else 
    if(g_stMipiTx.b1MipiClockburst == ENABLED)
    {
        Hss = (3 * g_stMipiTxDPhy.ucHsRqStPre + g_stMipiTxDPhy.ucHsTrail + 9) / 2 + 22; //Hss
    }
    else
    {
        Hss = (g_stMipiTxDPhy.ucHsRqStPre + (g_stMipiTxDPhy.ucHsTrail + 13) / 2) + 20;
    }    
    #endif
    Drv_MipiTx_Hss_Set(Hss);
}


//========================================================================
// Func Name   : Drv_MipiTx_Hss_Get
// Description : Hss Get
// Input       : void  
// Output      : None
// Return      : u16
//========================================================================
u16 Drv_MipiTx_Hss_Get(void)
{
    u16 rdhss = 0;

    HDMI_WriteI2C_Byte(0xff,0xd4);
    rdhss = (HDMI_ReadI2C_Byte(0x84) & 0x03); 
    rdhss = (rdhss << 8); 
    rdhss = rdhss + (HDMI_ReadI2C_Byte(0x85));
    return rdhss;
}

//========================================================================
// Func Name   : Drv_MipiTx_FSMHact_Get
// Description : Get Fsm Hact
// Input       : void  
// Output      : None
// Return      : u16
//========================================================================
u16 Drv_MipiTx_FSMHact_Get(void)
{
    u16 rgod_hact = 0;
    HDMI_WriteI2C_Byte(0xff,0xd4);
    rgod_hact = (HDMI_ReadI2C_Byte(0xc1) & 0x3f);
    rgod_hact = (rgod_hact << 8);
    rgod_hact = rgod_hact + HDMI_ReadI2C_Byte(0xc2);
    return rgod_hact;
}

//========================================================================
// Func Name   : Drv_MipiTx_FifoDelay_Set
// Description : MIPI Tx rddly set
// Input       : IN u16 rddly  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_FifoDelay_Set(IN u16 rddly)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0x82,(u8)(rddly >> 8));
    HDMI_WriteI2C_Byte(0x83,rddly);
}


void Mod_MipiTx_FifoRddly_Config(void)
{
    u16 ushss, usrgodhact;
    u16 ulRdHalfpixclk, usrdbyteclk;
    u32 ulTemp, ulrddly_max, ulrddly_min1, ulrddly_min2;
    u32 ulrddly = 0;

    //MIPI_DPHY Dphy is double-byte design , Cphy not
    ulRdHalfpixclk =  (u16)(g_stMipiTx.ulMipiInClk / 1000); //half pix
//    usrdbyteclk = (Drv_System_FmClkGet(AD_MLTX_WRITE_CLK) / 2); //Ê¹ÓÃÊµ¼ÊµÄbyteclkÔËËã»áÆ«Ð¡

    usrdbyteclk = (u16)(g_stMipiTx.ulMipiDataRate / 2000);
    usrdbyteclk = usrdbyteclk / 8;
  
    usrdbyteclk *= g_stMipiTx.ucTxPortNum;
    
    if((g_stMipiTx.ucTxLaneNum == 8)&&(g_stMipiTx.ucTxPortNum == 1))
    {
        usrdbyteclk <<= 1;
    }
    
    ushss = Drv_MipiTx_Hss_Get();
    usrgodhact = Drv_MipiTx_FSMHact_Get();
    ulTemp = (usrdbyteclk * ((g_stVidChk.usHs >> 1) + (g_stVidChk.usHbp >> 1)) / ulRdHalfpixclk); 
    
    if(ulTemp > ushss)
    {
        ulrddly_min1 = (usrdbyteclk * ((g_stVidChk.usHs >> 1) + (g_stVidChk.usHbp >> 1)) / ulRdHalfpixclk) - ushss;
    }
    else
    {
        ulrddly_min1 = 0;
    }
    
    if(g_stMipiTx.ucTxPortNum == 1)
    {
        ulrddly_min2 = (u32)usrdbyteclk * ((g_stVidChk.usHs >> 1) + (g_stVidChk.usHbp >> 1) + (g_stVidChk.usHact >> 1));
        ulTemp = ((u32)ulrddly_min2 / (u32)ulRdHalfpixclk);
        if(ulTemp > (ushss + usrgodhact))
        {
            ulrddly_min2 =(u32)(((u32)ulrddly_min2 / (u32)ulRdHalfpixclk) - ushss - usrgodhact);
        }
        else
        {
            ulrddly_min2 = 0;
        }
    }
    else
    {
        ulrddly_min2 = ulrddly_min1;
    }
    
    if (ulrddly_min1 > ulrddly_min2)
    {
        ulrddly_min2 = ulrddly_min1;
    }

    ulrddly_max = 0x8000 / (g_stMipiTx.ucBpp << 1);  //0x8000: 512fifo * 8 byte * 8 bit
    ulrddly_max = usrdbyteclk * (ulrddly_max + (g_stVidChk.usHs >> 1) + (g_stVidChk.usHbp >> 1));
    ulrddly_max = ulrddly_max / (u32)ulRdHalfpixclk - ushss;
//    ulrddly = (ulrddly_max - ulrddly_min2) / 20 + ulrddly_min2;
    ulrddly = (ulrddly_max / 7) + ulrddly_min2;

    Drv_MipiTx_FifoDelay_Set(ulrddly);
    pr_info( "rddly is 0x%04lx;",ulrddly);          
}

//========================================================================
// Func Name   : Drv_MipiTx_DPhyClkHsTrig
// Description : DPHY HS Clk Trig
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_DPhyClkHsTrig(void)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0xab,(HDMI_ReadI2C_Byte(0xab) & 0xdf));
    Ocm_Timer0_Delay1ms(5);
    HDMI_WriteI2C_Byte(0xab,(HDMI_ReadI2C_Byte(0xab) | BIT5_1)); //[5]:Dphy clk lane hs mode initial trigger
    Ocm_Timer0_Delay1ms(1);
    HDMI_WriteI2C_Byte(0xab,(HDMI_ReadI2C_Byte(0xab) & 0xdf));
}

//========================================================================
// Func Name   : Drv_MipiTx_DPhySkewCali
// Description : DPHY Skew Cali
// Input       : void  
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_DPhySkewCali(void)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);

    HDMI_WriteI2C_Byte(0xab,(HDMI_ReadI2C_Byte(0xab) | 0x0F)); //RGD_SKEW_CALI_PT_CFG[3:0]
    HDMI_WriteI2C_Byte(0xac,0x18); //RGD_SKEW_CALI_LEN[15:8]
    HDMI_WriteI2C_Byte(0xad,0x00); //RGD_SKEW_CALI_LEN[7:0]
    HDMI_WriteI2C_Byte(0xae,0x20); //RGD_SKEW_CALI_HS_ZERO[7:0]
    HDMI_WriteI2C_Byte(0xab,(HDMI_ReadI2C_Byte(0xab) | BIT4_1));
    Ocm_Timer0_Delay1ms(10);
    HDMI_WriteI2C_Byte(0xab,(HDMI_ReadI2C_Byte(0xab) & BIT4_0));
    HDMI_WriteI2C_Byte(0xae,0x00);
}

void Drv_MipiTx_DcsPktWrite(u8 DCS_DI, u8 ucLen, u8* Ptr )
{
    u8 i = 0;
    
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0xa0,0x00);
    
    if(ucLen == 2)
    {   
        if (DCS_DI == 0x29)
        {
            HDMI_WriteI2C_Byte( 0x9a, 0xe1 );
            HDMI_WriteI2C_Byte( 0x99, ucLen + 6 );
            HDMI_WriteI2C_Byte( 0x98, DCS_DI );
            HDMI_WriteI2C_Byte( 0x98, ucLen );
            HDMI_WriteI2C_Byte( 0x98, 0x00 );
        
            for(i = 0; i < ucLen; i++)
            {
                HDMI_WriteI2C_Byte( 0x98, *Ptr );
                Ptr++;                                                         
            }
        }
        else
        {
            HDMI_WriteI2C_Byte( 0x9a, 0xc1 );
            HDMI_WriteI2C_Byte( 0x99, 0x04 );
            HDMI_WriteI2C_Byte( 0x98, DCS_DI );
            HDMI_WriteI2C_Byte( 0x98, *Ptr );
            HDMI_WriteI2C_Byte( 0x98, *( Ptr + 1 ) );
        }
    }
    else
    {
        HDMI_WriteI2C_Byte( 0x9a, 0xe1 );
        HDMI_WriteI2C_Byte( 0x99, ucLen + 6 );
        HDMI_WriteI2C_Byte( 0x98, DCS_DI );
        HDMI_WriteI2C_Byte( 0x98, ucLen );
        HDMI_WriteI2C_Byte( 0x98, 0x00 );
    
        for(i = 0; i < ucLen; i++)
        {
            HDMI_WriteI2C_Byte( 0x98, *Ptr );
            Ptr++;                                                         
        }
    }
    Ocm_Timer0_Delay1ms(1);

    HDMI_WriteI2C_Byte( 0xab, 0x00 );

}

void Drv_MipiTx_DcsPktRead(u8 DCS_DI, u8 ucLen, u8* Ptr )
{
    u8 i = 0;
    
    HDMI_WriteI2C_Byte(0xff,0xd4);
    HDMI_WriteI2C_Byte(0xa0,0x10);
    
    if(ucLen == 2)
    {   
        HDMI_WriteI2C_Byte( 0x9a, 0xc2 );
        HDMI_WriteI2C_Byte( 0x99, 0x04 );
        HDMI_WriteI2C_Byte( 0x98, DCS_DI );
        HDMI_WriteI2C_Byte( 0x98, *Ptr );
        HDMI_WriteI2C_Byte( 0x98, *( Ptr + 1 ) );
    }
    else
    {
        HDMI_WriteI2C_Byte( 0x9a, 0xe2 );
        HDMI_WriteI2C_Byte( 0x99, ucLen + 6 );
        HDMI_WriteI2C_Byte( 0x98, DCS_DI );
        HDMI_WriteI2C_Byte( 0x98, ucLen );
        HDMI_WriteI2C_Byte( 0x98, 0x00 );
    
        for(i = 0; i < ucLen; i++)
        {
            HDMI_WriteI2C_Byte( 0x98, *Ptr );
            Ptr++;                                                         
        }
    }
    Ocm_Timer0_Delay1ms(1);

    HDMI_WriteI2C_Byte( 0xab, 0x00 );

}


void Drv_MipiTx_PanelInit(void)
{   

    const u8 boe_1600x1600_dcs0[] = {0x05, 0x01, 0x00};
    const u8 boe_1600x1600_dcs1[] = {LPT_DI, 0xb0, 0x04};
    const u8 boe_1600x1600_dcs2[] = {LPT_DI, 0xb9, 0x0f, 0x7f, 0x04, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x05, 0xdc, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00};
    const u8 boe_1600x1600_dcs3[] = {LPT_DI, 0xd6, 0x00};
    const u8 boe_1600x1600_dcs4[] = {LPT_DI, 0xec, 0x04, 0x72, 0x00, 0x00, 0x00};
    const u8 boe_1600x1600_dcs5[] = {0x15, 0x35, 0x00};
    const u8 boe_1600x1600_dcs6[] = {0x05, 0x11, 0x00};
    const u8 boe_1600x1600_dcs7[] = {0x05, 0x29, 0x00};


    HDMI_WriteI2C_Byte( 0xff, 0xd4 );
    HDMI_WriteI2C_Byte( 0xA2, 0x01 ); //LPRX CMD PORT SEL
    
    HDMI_WriteI2C_Byte( 0xab, 0x00 );
    HDMI_WriteI2C_Byte( 0xb6, 0x10 );
    HDMI_WriteI2C_Byte( 0xa3, 0x04 );
    HDMI_WriteI2C_Byte( 0x9a, 0xc2 ); 
    HDMI_WriteI2C_Byte( 0x9b, 0x46 );
    HDMI_WriteI2C_Byte( 0x9c, 0x02 );
    HDMI_WriteI2C_Byte( 0x9d, 0x50 );
    HDMI_WriteI2C_Byte( 0x9e, 0x10 );
    HDMI_WriteI2C_Byte( 0x9f, 0x50 );

    Ocm_Timer0_Delay1ms(5);

    pr_info("~~~~9211 --> Start initial panel");
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs0[0], (sizeof(boe_1600x1600_dcs0)/sizeof(boe_1600x1600_dcs0[0]) - 1), (u8*)&(boe_1600x1600_dcs0[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs1[0], (sizeof(boe_1600x1600_dcs1)/sizeof(boe_1600x1600_dcs1[0]) - 1), (u8*)&(boe_1600x1600_dcs1[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktRead(boe_1600x1600_dcs1[0], (sizeof(boe_1600x1600_dcs1)/sizeof(boe_1600x1600_dcs1[0]) - 1), (u8*)&(boe_1600x1600_dcs1[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs2[0], (sizeof(boe_1600x1600_dcs2)/sizeof(boe_1600x1600_dcs2[0]) - 1), (u8*)&(boe_1600x1600_dcs2[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs3[0], (sizeof(boe_1600x1600_dcs3)/sizeof(boe_1600x1600_dcs3[0]) - 1), (u8*)&(boe_1600x1600_dcs3[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs4[0], (sizeof(boe_1600x1600_dcs4)/sizeof(boe_1600x1600_dcs4[0]) - 1), (u8*)&(boe_1600x1600_dcs4[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs5[0], (sizeof(boe_1600x1600_dcs5)/sizeof(boe_1600x1600_dcs5[0]) - 1), (u8*)&(boe_1600x1600_dcs5[1]));
    Ocm_Timer0_Delay1ms(10);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs6[0], (sizeof(boe_1600x1600_dcs6)/sizeof(boe_1600x1600_dcs6[0]) - 1), (u8*)&(boe_1600x1600_dcs6[1]));
    Ocm_Timer0_Delay1ms(120);
    Drv_MipiTx_DcsPktWrite(boe_1600x1600_dcs7[0], (sizeof(boe_1600x1600_dcs7)/sizeof(boe_1600x1600_dcs7[0]) - 1), (u8*)&(boe_1600x1600_dcs7[1]));
    Ocm_Timer0_Delay1ms(200);
    pr_info("~~~~9211 --> Finish initial panel");
}


//========================================================================
// Func Name   : Drv_MipiTx_VideoSet
// Description : Mipi Set dsi/csi
// Input       : u8 b1Opt  On/Off
// Output      : None
// Return      : void
//========================================================================
void Drv_MipiTx_VideoSet(u8 b1Opt)
{
    HDMI_WriteI2C_Byte(0xff,0xd4);
    if (b1Opt == ON)
    {
        #if (MIPITX_OUT_SEL == MIPI_DSI)
        HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) | BIT4_1));
        pr_info( "Mipi DSI Out");
        #else
        HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) | BIT5_1));
        pr_info( "~~~~9211 --> Mipi CSI Out");
        #endif
    }
    else
    {
        HDMI_WriteI2C_Byte(0x89,(HDMI_ReadI2C_Byte(0x89) & 0x30));
    }

}














void lt9211_init_config(void)  // ttl in mipi out
{ 
    int ret = -1;

in_init:
    Drv_TtlRx_PhyPowerOn();  
    Drv_TtlRxClk_Sel();  
    Drv_SystemActRx_Sel(TTLRX);  

STATE_CHIPRX_VIDTIMING_CONFIG:
    Mod_TtlRx_DigSet();

STATE_CHIPRX_PLL_CONFIG:
    if(Drv_TtlRx_Pll_Set() == FAIL)   
    {   
        PRINT_DEG("~~~~9211 Drv_TtlRx_Pll_Set fail!  go in_init!!\n");
        goto in_init;
    } 
        
    else
    {

        PRINT_DEG("~~~~9211 Drv_TtlRx_Pll_Set sucess!!!\n");
        #if TTLRX_SYNC_INTER_MODE == ENABLED
            Drv_System_VidChkClk_SrcSel(RXPLL_PIX_CLK);                      // DrvSystem.c |  line= 7.    √ √ √ √ √
            Drv_System_VidChk_SrcSel(TTLDEBUG);                             // DrvSystem.c  |  line= 82.   √ √ √ √ √
            Drv_TtlRx_SelfTimingSet();                                      // DrvttlRx.c   |  line= 814.  √ √ √ √ √
        #endif

        #if TTLRX_DE_SYNC_MODE != 0
            Drv_TtlRx_DeSyncModeSet();                                      // DrvttlRx.c   |  line= 926.  √ √ √ √ √
        #endif
    }

STATE_CHIPRX_VIDEO_CHECK:
    Drv_System_VidChkClk_SrcSel(DESSCPLL_PIX_CLK);     
    Drv_System_VidChk_SrcSel(TTLRX);                                            // DrvSystem.c  |  line= 82.   √ √ √ √ √

    Drv_TtlRx_VidChkDebug();   
    // g_stChipRx.pHdmiRxNotify(MIPIRX_VIDEO_ON_EVENT);  //  ?  ?  ? 


    // mipi
    Drv_SystemTxSram_Sel();
    Mod_MipiTx_ParaSet();
    Mod_MipiTx_DataRateAdj();
    Drv_MipiTx_DPhySet();

    Drv_MipiTx_PllSet(&g_stMipiTx);

    if (Drv_MipiTx_PllCali() == SUCCESS)
    {
        PRINT_DEG("~~9211 debug:  Drv_MipiTx_PllCali SUCCESS ~~~~~~~~~~\n");
        Drv_MipiTx_HalfWrClkSrc_Sel(HALF_WRITE_CLK);
        Mod_MipiTx_Resolution_Config();
        Drv_MipiTx_PhyTimingParaSet(&g_stMipiTx, &g_stMipiTxDPhy);
        Mod_MipiTx_Digital_Config();
        Mod_MipiTx_HssSet();
        Mod_MipiTx_FifoRddly_Config();
        Drv_MipiTx_DPhyClkHsTrig();
        Drv_MipiTx_DPhySkewCali();
        // Mod_SystemTx_SetState(STATE_CHIPTX_VIDEO_OUT);
 
    }

    else{
        PRINT_DEG("~~9211 debug:  Drv_MipiTx_PllCali error ~~~~~~~~~~\n");
        return ;
    }
    
    Drv_MipiTx_PanelInit();
    Drv_MipiTx_VideoSet(ON);

    pr_info("lt9211_init_config done!!!!\n");

}

// EXPORT_SYMBOL_GPL(lt9211_init_config);






// ------------------------------------------------- for video probe ---------------------------------------------------------
// ------------------------------------------------- for video probe ---------------------------------------------------------
// ------------------------------------------------- for video probe ---------------------------------------------------------
// static char fsync_mode_default[20] = "automatic"; /* manual, automatic, semi-automatic, external */


// static void lc9211_initial_setup(struct lt9211c_priv *priv)
// {
//     struct deser_hub_dev *hub = &priv->hub;

//     priv->fsync_mode = fsync_mode_default;

//     if (strcmp(priv->fsync_mode, "manual") == 0) {
//         register_write(
//             hub, 0x01,
//             0x00); /* manual: FRAMESYNC set manually via [0x06:0x08] regs */
//     } else if (strcmp(priv->fsync_mode, "automatic") == 0) {
//         register_write(
//             hub, 0x01,
//             0x02); /* automatic: FRAMESYNC taken from the slowest Link */
//     } else if (strcmp(priv->fsync_mode, "semi-automatic") == 0) {
//         register_write(
//             hub, 0x01,
//             0x01); /* semi-automatic: FRAMESYNC taken from the slowest Link */
//     } else if (strcmp(priv->fsync_mode, "external") == 0) {
//         register_write(
//             hub, 0x01,
//             0xc0); /* ECU (aka MCU) based FrameSync using GPI-to-GPO */
//     }

//     register_write(hub, 0x63, 0); /* disable overlap window */
//     register_write(hub, 0x64, 0);
//     register_write(hub, 0x06, priv->fsync_period & 0xff);
//     register_write(hub, 0x07, (priv->fsync_period >> 8) & 0xff);
//     register_write(hub, 0x08, priv->fsync_period >> 16);

//     mdelay(64);
// }



// static int lc9211_postinit(struct lt9211c_priv *priv)
// {
//     struct deser_hub_dev *hub = &priv->hub;

//     register_write(hub, 0x12,
//                ((4 - 1) << 6) | (priv->dbl ? 0x30 : 0) |
//                    (priv->dt &
//                 0xf)); /* setup lanes, DBL mode, DataType */

//     lc9211_initial_setup(priv);

//     //register_write(hub, 0x0a, 0xff);

//     register_write(hub, 0x62, 0x1f);
//     register_write(hub, 0x61, 0xff);
//     register_write(hub, 0x5f, 0x0f);

//     ser_write(hub, MAX96705_BROADCAST, 0x04,
//           0x83); //enable all max96705 output
//     mdelay(64);
//     register_write(hub, 0x15, 0x9b); //enable lc9211 output

//     return 0;
// }


static int lc9211_s_stream(struct v4l2_subdev *subdev, int enable)
{
    return 0;
}

static int lc9211_s_power(struct v4l2_subdev *sd, int enable)
{

    struct deser_hub_dev *hub = container_of(sd, struct deser_hub_dev, subdev);

    struct lt9211c_priv *priv = container_of(hub, struct lt9211c_priv, hub);

    // lc9211_postinit(priv);
    dev_err(hub->dev, "vv9211:  %s() line:%d\n", __func__, __LINE__);

    hub->deser_boot_flag = true;

    return 0;
}



static struct v4l2_subdev_video_ops lc9211_video_ops = {
    .s_stream = lc9211_s_stream,
};

static struct v4l2_subdev_core_ops lc9211_core_ops = {
    .s_power = lc9211_s_power,
};



static struct v4l2_subdev_ops lc9211_ops = {
    .core = &lc9211_core_ops,
    .video = &lc9211_video_ops,
};


static int register_subdev(struct lt9211c_priv *priv)
{
    int ret;
    struct deser_hub_dev *hub;
    struct v4l2_subdev *sd;

    hub = &priv->hub;

    sd = &hub->subdev; // de, 

    v4l2_subdev_init(sd, &lc9211_ops);  //  √ ok
    v4l2_set_subdevdata(sd, hub);       // √ ok

    sd->dev = hub->dev;
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    snprintf(sd->name, sizeof(sd->name), "%s", dev_name(hub->dev));

    PRINT_DEG("vv9211c_03 sd: %s \n", sd->name); // 0-0029
    // PRINT_DEG("vv9211 hub: %s \n", hub->name);
    PRINT_DEG("vv9211_03: %s \n", dev_name(hub->dev));   // 0-0029

    sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
    priv->pads[lc9211_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
    priv->pads[lc9211_SINK_LINK0].flags = MEDIA_PAD_FL_SINK;
    priv->pads[lc9211_SINK_LINK1].flags = MEDIA_PAD_FL_SINK;
    priv->pads[lc9211_SINK_LINK2].flags = MEDIA_PAD_FL_SINK;
    priv->pads[lc9211_SINK_LINK3].flags = MEDIA_PAD_FL_SINK;


    ret = media_entity_pads_init(&sd->entity, lc9211_N_PADS, priv->pads);
    if (ret)
        return ret;

    ret = v4l2_async_register_subdev(sd);
    if (ret) {
        dev_err(hub->dev, "register subdev failed\n");
        return -1;
    }

    pr_info("~~vv9211_03 v4l2: registe_subdev ok!!");
    return 0;
}


static int deser_notify_bound(struct v4l2_async_notifier *notifier,
                  struct v4l2_subdev *sd,
                  struct v4l2_async_subdev *asd)
{
    struct camera_dev *cam_dev;
    struct deser_channel *deser_chn;

    cam_dev = container_of(sd, struct camera_dev, subdev);
    deser_chn = container_of(asd, struct deser_channel, async_dev);

    cam_dev->sd_state = BST_SUBDEV_STATE_BOUND;
    cam_dev->deser_parent = deser_chn->deser_dev;
    cam_dev->index_in_serdes = deser_chn->index;
    deser_chn->cam_dev = cam_dev;
    deser_chn->camera_bound = true;

    return 0;
}


static void deser_notify_unbind(struct v4l2_async_notifier *notifier,
                struct v4l2_subdev *subdev,
                struct v4l2_async_subdev *asd)
{
}


static const struct v4l2_async_notifier_operations deser_async_ops = {
    .bound = deser_notify_bound,
    .unbind = deser_notify_unbind,
};






static int register_subdev_notifier(struct lt9211c_priv *priv)
{
    int ret;
    int i;
    int index;
    struct deser_hub_dev *hub = &priv->hub;

    // hub->num_cameras = 2;
    if (!hub->num_cameras) {
        dev_err(hub->dev, "%s: no input device found\n", __func__);
        // return -1;
        pr_info("vv9211: error1  no input device found");
    }



    v4l2_async_notifier_init(&hub->notifier);
    hub->notifier.ops = &deser_async_ops;
    index = 0;
    for (i = 0; i < priv->n_links; i++) {
        if (!hub->chn[i].camera_fwnode)
            continue;

        hub->chn[i].deser_dev = hub;
        hub->chn[i].async_dev.match_type = V4L2_ASYNC_MATCH_FWNODE;
        hub->chn[i].async_dev.match.fwnode = hub->chn[i].camera_fwnode;
        v4l2_async_notifier_add_subdev(&hub->notifier,
                           &(hub->chn[i].async_dev));
        index++;
    }

    ret = v4l2_async_subdev_notifier_register(&hub->subdev, &hub->notifier);
    if (ret) {
        dev_err(hub->dev, "%s: register subdev notifier failed\n",
            __func__);
        // return -1;
        pr_info("vv9211:register subdev notifier failed");
    }

    return 0;
}



/*

struct i2c_client {
    unsigned short addr; //设备地址
    char name[I2C_NAME_SIZE]; //设备名称
    struct i2c_adapter *adapter; //设配器，值I2C控制器
    struct i2c_driver *driver; //设备对应的驱动
    struct device dev; //表明这是一个设备
    int irq; //中断号
    struct list_head detected; //节点
};
*/

// i2c_client就是描述设备信息的（将设备树节点转化为i2c_client）
static int lt9211c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	// u8 val = 0;

    PRINT_DEG("LT9211 I2C Address: 0x%02x\n", client->addr);   // 0x30 //设备地址

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

   
    priv->client = client;        //数据结构 i2c_client 赋值
	i2c_set_clientdata(client, priv);
	priv->hub.i2c_client = client;
    priv->hub.dev = &client->dev;           // same as client
    priv->hub.deser_boot_flag = false;

    PRINT_DEG("vv9211 priv->hub Address: 0x%02x\n", priv->hub.i2c_client->addr);   // 0x29 //hup 设备地址


    ret = register_subdev(priv);       // ok
    if (ret) {
        dev_err(priv->hub.dev, "%s: register subdev failed\n",
            __func__);
        return -EINVAL;
    }

    if(client->dev.of_node) {  // ok 
        ret = lt9211_parse_dt(&client->dev, priv, client);
        if(ret < 0) {
            dev_err(&client->dev, "~~~~~~~~~Failed parse lt9211\n");
            return -ENODEV;
        }
    }
	

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
    usleep_range(5000, 10000); 
    
    lt9211_read_ID();


    lt9211_init_config();


    ret = register_subdev_notifier(priv);
    if (ret) {
        dev_err(priv->hub.dev, "%s: register subdev notifier failed\n",
            __func__);
        pr_info("vv9211c::: register subdev notifier failed!");  
        // return -EINVAL;
    }

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
