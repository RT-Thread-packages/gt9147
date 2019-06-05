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
#include "gt9147.h"

/* Test function */
static int gt9147_test()
{
    rt_device_t dev;
    void *id;

    dev = rt_device_find("touch_gt");

    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device:%s\n", "acce_icm");
        return -1;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("open device failed!");
        return -1;
    }

    rt_device_control(dev, RT_TOUCH_CTRL_GET_ID, id);

    rt_uint8_t * read_id = (rt_uint8_t *)id;

    rt_kprintf("id = %d %d %d %d \n", read_id[0] - '0', read_id[1] - '0', read_id[2] - '0', read_id[3] - '0');

    rt_device_control(dev, RT_TOUCH_CTRL_GET_INFO, id);

    rt_kprintf("range_x = %d ", (*(struct rt_touch_info*)id).range_x);
    rt_kprintf("range_y = %d \n", (*(struct rt_touch_info*)id).range_y);

    rt_device_close(dev);
    
    return 0;
}
MSH_CMD_EXPORT(gt9147_test, gt9147 sensor test function);

