/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-01     tyustli     the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <string.h>
#include <stdlib.h>

#define DBG_TAG "gt9147"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "touch.h"
#include "gt9147.h"
#include "gt9147_reg.h"

/* hardware section */
static const rt_uint8_t GT9147_CFG_TBL[] =
{
    0X00, 0XE0, 0X01, 0X10, 0X01, 0X05, 0X3C, 0X00, 0X02, 0X08,
    0X1E, 0X08, 0X50, 0X3C, 0X0F, 0X05, 0X00, 0X00, 0XFF, 0X67,
    0X50, 0X00, 0X00, 0X18, 0X1A, 0X1E, 0X14, 0X89, 0X28, 0X0A,
    0X30, 0X2E, 0XBB, 0X0A, 0X03, 0X00, 0X00, 0X02, 0X33, 0X1D,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X32, 0X00, 0X00,
    0X2A, 0X1C, 0X5A, 0X94, 0XC5, 0X02, 0X07, 0X00, 0X00, 0X00,
    0XB5, 0X1F, 0X00, 0X90, 0X28, 0X00, 0X77, 0X32, 0X00, 0X62,
    0X3F, 0X00, 0X52, 0X50, 0X00, 0X52, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X0F,
    0X0F, 0X03, 0X06, 0X10, 0X42, 0XF8, 0X0F, 0X14, 0X00, 0X00,
    0X00, 0X00, 0X1A, 0X18, 0X16, 0X14, 0X12, 0X10, 0X0E, 0X0C,
    0X0A, 0X08, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X29, 0X28, 0X24, 0X22, 0X20, 0X1F, 0X1E, 0X1D,
    0X0E, 0X0C, 0X0A, 0X08, 0X06, 0X05, 0X04, 0X02, 0X00, 0XFF,
    0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
    0X00, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF,
    0XFF, 0XFF, 0XFF, 0XFF,
};

static rt_err_t gt9147_write_reg(struct gt9147_device *dev, rt_uint8_t write_len, rt_uint8_t *write_data)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = dev->i2c_addr;                /* slave address */
    msgs.flags = RT_I2C_WR;                    /* write flag */
    msgs.buf   = write_data;                   /* send data pointer */
    msgs.len   = write_len;                    /* send data len */

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }

    return res;
}

static rt_err_t gt9147_read_regs(struct gt9147_device *dev, rt_uint8_t *cmd_buf, rt_uint8_t cmd_len, rt_uint8_t read_len, rt_uint8_t *read_buf)
{
    rt_int8_t res = 0;

    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = dev->i2c_addr;    /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = cmd_buf;        /* Slave register address */
    msgs[0].len   = cmd_len;                /* Number of bytes sent */

    msgs[1].addr  = dev->i2c_addr;    /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = read_buf;              /* Read data pointer */
    msgs[1].len   = read_len;              /* Number of bytes read */

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }

    return res;
}

/**
 * This function read the product id
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for gt9xx
 * @param read data len
 * @param read data pointer
 *
 * @return the read status, RT_EOK reprensents  read the value of the register successfully.
 */
static rt_err_t gt9147_get_product_id(struct gt9147_device *dev, rt_uint8_t read_len, rt_uint8_t *read_data)
{
    rt_uint8_t cmd_buf[2];

    cmd_buf[0] = (rt_uint8_t)(GT9XX_PRODUCT_ID >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT9XX_PRODUCT_ID & 0xff);

    if (gt9147_read_regs(dev, cmd_buf, 2, read_len, read_data) != RT_EOK)
    {
        LOG_D("read id failed \n");

        return RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t gt9147_soft_reset(struct gt9147_device *dev)
{
    rt_uint8_t buf[3];

    buf[0] = (rt_uint8_t)(GT9147_COMMAND >> 8);
    buf[1] = (rt_uint8_t)(GT9147_COMMAND & 0xFF);
    buf[2] = 0x02;

    if (gt9147_write_reg(dev, 3, buf) != RT_EOK)
    {
        LOG_D("soft reset gt9147 failed\n");
        return RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t gt9147_control(struct gt9147_device *dev, void *cmd, void *data)
{
    rt_uint8_t buf[4];
    rt_uint8_t i = 0;
    rt_uint8_t *config;
    rt_uint8_t choose_cmd = 0xff;

    config = (rt_uint8_t *)rt_malloc(sizeof(GT9147_CFG_TBL) + GTP_ADDR_LENGTH);

    if (config == RT_NULL)
    {
        LOG_D("malloc config memory failed\n");
        return RT_ERROR;
    }

    config[0] = (rt_uint8_t)((GT9147_CONFIG >> 8) & 0xFF); /* config reg */
    config[1] = (rt_uint8_t)(GT9147_CONFIG & 0xFF);

    memcpy(&config[2], GT9147_CFG_TBL, sizeof(GT9147_CFG_TBL)); /* config table */

    if (cmd != RT_NULL)
        choose_cmd = *(rt_uint8_t *)cmd;

    switch(choose_cmd)
    {
    case CMD_CONFIG_VERSION: /* set version */
    {
        config[2] = *(rt_uint8_t *)data;
        break;
    }
    case CMD_CONFIG_X_OUTPUT_MAX: /* set x range */
    {
        rt_uint16_t x_ran;

        x_ran = *(rt_uint16_t *)data;
        config[4] = (rt_uint8_t)(x_ran >> 8);
        config[3] = (rt_uint8_t)(x_ran & 0xff);

        break;
    }
    case CMD_CONFIG_Y_OUTPUT_MAX: /* set y range */
    {
        rt_uint16_t y_ran;

        y_ran = *(rt_uint16_t *)data;
        config[6] = (rt_uint8_t)(y_ran >> 8);
        config[5] = (rt_uint8_t)(y_ran & 0xff);

        break;
    }
    case CMD_CONFIG_X_TO_Y: /* change x y */
    {
        config[8] = config[8] ^= (1 << 3);
        break;
    }
    case CMD_CONFIG_INT_TYPE: /* change int trig type */
    {
        rt_uint8_t trig_type;
        trig_type = *(rt_uint8_t *)data;

        switch (trig_type)
        {
        case CMD_INT_RISING_TRIG:
            config[8] &= 0xFC;
            break;
        case CMD_INT_FALLING_TRIG:
            config[8] &= 0xFC;
            config[8] |= 0x01;
            break;
        case CMD_INT_CHECK_LOW:
            config[8] &= 0xFC;
            config[8] |= 0x02;
            break;
        case CMD_INT_CHECK_HIGH:
            config[8] &= 0xFC;
            config[8] |= 0x03;
            break;
        default:
            break;
        }
        break;
    }
    default:
    {
        break;
    }
    }

    if (gt9147_write_reg(dev, sizeof(GT9147_CFG_TBL) + GTP_ADDR_LENGTH, config) != RT_EOK)  /* send config */
    {
        LOG_D("send config failed\n");
        return RT_ERROR;
    }

    buf[0] = (rt_uint8_t)((GT9147_CHECK_SUM >> 8) & 0xFF);
    buf[1] = (rt_uint8_t)(GT9147_CHECK_SUM & 0xFF);
    buf[2] = 0;       /* Config_Checksum */

    for(i = GTP_ADDR_LENGTH; i < sizeof(GT9147_CFG_TBL) + GTP_ADDR_LENGTH; i++)
        buf[GTP_ADDR_LENGTH] += config[i]; /* calc sum*/

    buf[2] = (~buf[2]) + 1;
    buf[3] = 1;	      /* Config_Flash */

    gt9147_write_reg(dev, 4, buf); /* send check sum */

    /* test config write */
#if 1
    {
        rt_uint16_t i;
        rt_uint8_t buf[200];
        rt_uint8_t cmd[2];

        cmd[0] = (rt_uint8_t)((GT9147_CONFIG >> 8) & 0xFF); /* config reg */
        cmd[1] = (rt_uint8_t)(GT9147_CONFIG & 0xFF);

        gt9147_read_regs(dev, cmd, 2, sizeof(GT9147_CFG_TBL), buf);

        for(i = 0; i < sizeof(GT9147_CFG_TBL); i++)
        {
            if(config[i + 2] != buf[i])
            {
                LOG_D("Config fail ! i = %d \r\n", i);
                return 0;
            }
        }

        LOG_D("Config success\r\n", i);
    }
#endif
    rt_free(config);

    return RT_EOK;

}

static rt_err_t gt9147_read_point(struct gt9147_device *dev, touch_message_t msg)
{
    rt_uint8_t read_status = 0;
    rt_uint8_t touch_num = 0;
    rt_uint8_t write_buf[3];
    rt_uint8_t cmd[2];
    rt_uint8_t read_buf[8] = {0};
    static rt_uint8_t s_tp_down1 = 0;

    cmd[0] = (rt_uint8_t)((GT9147_READ_XY >> 8) & 0xFF);
    cmd[1] = (rt_uint8_t)(GT9147_READ_XY & 0xFF);

    if (gt9147_read_regs(dev, cmd, 2, 8, read_buf) != RT_EOK)
    {
        LOG_D("read point failed\n");
        return RT_ERROR;
    }
    read_status = read_buf[0];

    if (read_status == 0)
    {
        return 0;
    }

    if ((read_status & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = read_status & 0x0f;

    if(touch_num == 0)
    {
        if(s_tp_down1)
        {
            s_tp_down1 = 0;
            msg->event = TOUCH_EVENT_UP;
        }
        else
        {
            msg->event = TOUCH_EVENT_NONE;
        }
    }
    else
    {
        msg->id = read_buf[1] & 0x0f;
        msg->x = ((rt_uint16_t)read_buf[3] << 8) | read_buf[2];
        msg->y = ((rt_uint16_t)read_buf[5] << 8) | read_buf[4];
        msg->width = ((rt_uint16_t)read_buf[7] << 8) | read_buf[6];

        if(s_tp_down1 == 1)
        {
            msg->event = TOUCH_EVENT_MOVE;
        }
        else
        {
            msg->event = TOUCH_EVENT_DOWN;
            s_tp_down1 = 1;
        }
    }

exit_work_func:
    write_buf[0] = (rt_uint8_t)((GT9147_READ_XY >> 8));
    write_buf[1] = (rt_uint8_t)(GT9147_READ_XY & 0xFF);
    write_buf[2] = 0x00;

    gt9147_write_reg(dev, 3, write_buf);

    return RT_EOK;
}


static rt_err_t gt9147_get_info(struct gt9147_device *dev, struct gt9147_info *info)
{
    rt_uint8_t opr_buf[7] = {0};
    rt_uint8_t cmd_buf[2];

    cmd_buf[0] = (rt_uint8_t)(GT9147_CONFIG >> 8);
    cmd_buf[1] = (rt_uint8_t)(GT9147_CONFIG & 0xff);

    if (gt9147_read_regs(dev, cmd_buf, 2, 7, opr_buf) != RT_EOK)
    {
        LOG_D("read id failed \n");

        return RT_ERROR;
    }

    info->version = opr_buf[0];
    info->x_range = (opr_buf[2] << 8) + opr_buf[1];
    info->y_range = (opr_buf[4] << 8) + opr_buf[3];
    info->touch_num = opr_buf[5] & 0x0f;
    info->trig_type = opr_buf[6] & 0x03;

    return RT_EOK;

}

static struct gt9147_device *gt9147_init(const char *dev_name, rt_uint8_t param)
{
    struct gt9147_device *dev = RT_NULL;
    rt_uint8_t reg[4];

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct gt9147_device));

    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for gt9147 device on '%s' \n", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);

    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'\n", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* find gt9147 device at address: 0x5d */
            dev->i2c_addr = GT9147_ADDRESS_HIGH;

            if (gt9147_get_product_id(dev, 4, reg) != RT_EOK)
            {
                /* find gt9147 device at address 14 */
                dev->i2c_addr = GT9147_ADDRESS_LOW;

                if (gt9147_get_product_id(dev, 4, reg) != RT_EOK)
                {
                    LOG_E("Can't find device at '%s'!", dev_name);
                    goto __exit;
                }
            }

            LOG_E("Device i2c address is:'0x%x'!\n", dev->i2c_addr);
        }
    }
    else
    {
        LOG_E("Unsupported device:'%s'!\n", dev_name);
        goto __exit;
    }

    if (rt_device_open(dev->bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open device failed\n");
    }

    LOG_D("gt947 init success\n");

    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }

    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
static RT_UNUSED void gt9147_deinit(struct gt9147_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

/* docking to rt-thread touch framework section */

static struct gt9147_device *gt9147_dev;

static rt_err_t _gt9147_init(struct rt_touch_intf *intf)
{
    struct device_data *appli_data = (struct device_data *)(intf->user_data);

    rt_pin_mode(appli_data->rst_pin, PIN_MODE_OUTPUT);
    rt_pin_mode(appli_data->int_pin, PIN_MODE_OUTPUT);

    rt_pin_write(appli_data->rst_pin, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(appli_data->rst_pin, PIN_HIGH);
    rt_thread_mdelay(10);
    rt_pin_mode(appli_data->int_pin, PIN_MODE_INPUT);
    rt_thread_mdelay(100);

    gt9147_dev = gt9147_init(intf->dev_name, appli_data->i2c_addr);

    if (gt9147_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    gt9147_soft_reset(gt9147_dev);

    return RT_EOK;
}

static rt_size_t _gt9147_get_data(rt_touch_t touch, rt_off_t pos, struct rt_touch_data *data, rt_size_t len)
{
    touch_message_t msg;

    gt9147_read_point(gt9147_dev, msg);

    data->event = msg->event;
    data->track_id = msg->id;
    data->timestamp = rt_touch_get_ts();
    data->width = msg->width;
    data->x_coordinate = msg->x;
    data->y_doordinate = msg->y;

    return 1;
}


static rt_size_t _gt9147_fetch_data(struct rt_touch_device *touch, rt_off_t pos, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    return _gt9147_get_data(touch, pos, buf, len);
}

static rt_err_t _gt9147_control(struct rt_touch_device *touch, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_TOUCH_CTRL_GET_ID:
    {
        gt9147_get_product_id(gt9147_dev, 4, args);
        break;
    }
    case RT_TOUCH_CTRL_GET_INFO:
    {
        gt9147_get_info(gt9147_dev, args);
        break;
    }
    case RT_TOUCH_CTRL_SET_MODE:
    {
        rt_uint8_t cmd = CMD_CONFIG_INT_TYPE;
        gt9147_control(gt9147_dev, &cmd, args);
        break;
    }
    case RT_TOUCH_CTRL_SET_POWER:
    {
        result = RT_EOK;
        break;
    }
    case RT_TOUCH_CTRL_SET_X_RANGE:
    {
        rt_uint8_t cmd = CMD_CONFIG_X_OUTPUT_MAX;
        gt9147_control(gt9147_dev, &cmd, args);
        break;
    }
    case RT_TOUCH_CTRL_SET_Y_RANGE:
    {
        rt_uint8_t cmd = CMD_CONFIG_Y_OUTPUT_MAX;
        gt9147_control(gt9147_dev, &cmd, args);
        break;
    }
    case RT_TOUCH_CTRL_SET_X_TO_Y:
    {
        rt_uint8_t cmd = CMD_CONFIG_X_TO_Y;
        gt9147_control(gt9147_dev, &cmd, args);
        break;
    }
    default:
    {
        return -RT_ERROR;
    }
    }

    return result;
}

static struct rt_touch_ops touch_ops =
{
    .touch_readpoint = _gt9147_fetch_data,
    .touch_control = _gt9147_control,
};

int rt_hw_gt9147_init(const char *name, struct rt_touch_config *cfg)
{
    rt_uint8_t result;
    rt_touch_t touch_device = RT_NULL;

    touch_device = rt_calloc(1, sizeof(struct rt_touch_device));

    if (touch_device == RT_NULL)
        return -1;

    /* touch ic information */
    touch_device->info.type = RT_TOUCH_TYPE_CAPACITANCE;
    touch_device->info.vendor = RT_TOUCH_VENDOR_GT;
    touch_device->info.intf_type = RT_TOUCH_INTF_I2C;
    touch_device->info.range_x = 480;
    touch_device->info.range_y = 272;

    rt_memcpy(&touch_device->config, cfg, sizeof(struct rt_touch_config));
    touch_device->ops = &touch_ops;

    result = rt_hw_touch_register(touch_device, name, RT_DEVICE_FLAG_INT_RX, RT_NULL);

    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d\n", result);
        goto __exit;
    }

    result = _gt9147_init(&cfg->intf);

    if (result != RT_EOK)
    {
        LOG_E("GT9147 init err code: %d\n", result);
        goto __exit;
    }

    LOG_I("touch init success\n");

    return RT_EOK;

__exit:
    rt_free(touch_device);
    LOG_E("gt9147 init failed\n");
    return -RT_ERROR;
}



