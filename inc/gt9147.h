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

int rt_hw_gt9147_init(const char *name, struct rt_touch_config *cfg);

#endif
