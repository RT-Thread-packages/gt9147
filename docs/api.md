# API 说明

在 RT-Thread 上编程，需要用到 gt9147 等触摸芯片时，使用 gt9147 软件包就可以轻松完成传感器的配置以及传感器数据的读取，本章介绍 gt9147 软件包提供的常用 API。

### 初始化函数

```{.c}
struct gt9147_device *gt9147_init(const char *dev_name, rt_uint8_t param)
```

使用指定的通信设备 I2C 初始化 gt9147 ，并返回控制句柄。

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev_name               | 用于同 gt9147 通信的设备名（支持 I2C 总线设备） |
|param | I2C 通信，根据此处传入的 I2C 地址寻找设备（例如：0x5D） |
| **返回**          | **描述**                                |
|NULL                 | 失败                                |

### 反初始化函数

```{.c}
void gt9147_deinit(struct gt9147_device *dev)
```

释放 gt9147 设备占据的内存空间

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | gt9147_device 结构体的指针 |
| **返回** | **描述**                    |
| 无返回值 |                             |

### 获取产品 ID

```{.c}
rt_err_t gt9147_get_product_id(struct gt9147_device *dev, rt_uint8_t read_len, rt_uint8_t *read_data);
```

获取挂载在总线上的 gt9147 的 ID 号

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | gt9147_device 结构体的指针 |
|read_len | 要读的数据的长度 |
|read_data |读到的数据的指针|
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|RT_ERROR                 | 失败                                |

### 获取 gt9147 配置信息
```{.c}
rt_err_t gt9147_get_info(struct gt9147_device *dev, struct gt9147_info *info);
```

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | gt9147_device 结构体的指针 |
|info |获取到的 gt9147 配置信息的结构体指针|
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|RT_ERROR                 | 失败                                |

### 软复位 gt9147
```{.c}
rt_err_t gt9147_soft_reset(struct gt9147_device *dev);
```
| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | gt9147_device 结构体的指针 |
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|RT_ERROR                 | 失败                                |

### 设定参数
```{.c}
rt_err_t gt9147_control(struct gt9147_device *dev, void *cmd, void *data);
```
为挂载上的 gt9147 设备设定参数
| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | gt9147_device 结构体的指针 |
|cmd | 支持的配置选项，详见下面介绍 |
|data |设定的具体参数值|
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|RT_ERROR                 | 失败                                |
cmd 参数指要配置的选项，data 参数表示设定的参数的具体值。详情如下：

**支持的配置选项** 

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|CMD_CONFIG_VERSION               | 配置触摸芯片的版本信息 |
|CMD_CONFIG_X_OUTPUT_MAX | 配置触摸芯片的 X 轴输出位置的最大值 |
|CMD_CONFIG_Y_OUTPUT_MAX | 配置触摸芯片的 Y 轴输出位置的最大值 |
| CMD_CONFIG_X_TO_Y | 翻转 X 轴和 Y 轴坐标 |
|CMD_CONFIG_INT_TYPE                  | 配置触摸芯片中断触发方式 |

** CMD_CONFIG_INT_TYPE 参数的值** 

```{.c}
CMD_INT_RISING_TRIG    //上升沿触发
CMD_INT_FALLING_TRIG   //下降沿触发
CMD_INT_CHECK_LOW      //低电平查询
CMD_INT_CHECK_HIGH     //高电平查询
```

### 读取触摸芯片坐标信息   

```{.c}
rt_err_t gt9147_read_point(struct gt9147_device *dev, touch_message_t msg);
```

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | gt9147_device 结构体的指针             |
| msg     | 存储 GT9147 触摸芯片数据的结构体指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| RT_ERROR     | 失败                                    |

触摸芯片数据的结构体定义如下

```{.c}
struct touch_message
{
    rt_uint16_t x;          /* X 轴坐标 */
    rt_uint16_t y;          /* Y 轴坐标 */
    rt_uint8_t width;       /* 触摸点宽度 */
    rt_uint8_t id;          /* 触摸点 ID 号 */
    rt_uint8_t event;       /* 触摸事件 */
};
```
触摸事件定义如下：
```{.c}
TOUCH_EVENT_UP             /* 抬起事件 */
TOUCH_EVENT_DOWN           /* 按下事件 */
TOUCH_EVENT_MOVE           /* 移动事件 */
TOUCH_EVENT_NONE           /* 无触摸事件 */
```
