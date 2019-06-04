/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-01     tyustli     the first version
 */

#ifndef __GT9147_H_
#define __GT9147_H_
#include <rtthread.h>
#include "touch.h"

#define TOUCH_EVENT_UP      (0x01)
#define TOUCH_EVENT_DOWN    (0x02)
#define TOUCH_EVENT_MOVE    (0x03)
#define TOUCH_EVENT_NONE    (0x80)

#define CMD_CONFIG_VERSION        (0x00)
#define CMD_CONFIG_X_OUTPUT_MAX   (0x01)
#define CMD_CONFIG_Y_OUTPUT_MAX   (0x02)
#define CMD_CONFIG_X_TO_Y         (0x03)
#define CMD_CONFIG_INT_TYPE       (0x04)

#define CMD_INT_RISING_TRIG       (0x00)
#define CMD_INT_FALLING_TRIG      (0x01)
#define CMD_INT_CHECK_LOW         (0x02)
#define CMD_INT_CHECK_HIGH        (0x03)

struct gt9147_info
{
    rt_uint16_t x_range;
    rt_uint16_t y_range;
    rt_uint8_t touch_num;
    rt_uint8_t version;
    rt_uint8_t trig_type;
};

struct gt9147_device
{
    rt_device_t bus;
    rt_uint32_t id;
    rt_uint8_t i2c_addr;
};

struct touch_message
{
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t width;
    rt_uint8_t id;
    rt_uint8_t event;
};
typedef struct touch_message *touch_message_t;

struct device_data
{
    rt_uint8_t i2c_addr;
    rt_uint8_t rst_pin;
    rt_uint8_t int_pin;
};

int rt_hw_gt9147_init(const char *name, struct rt_touch_config *cfg);

#endif
