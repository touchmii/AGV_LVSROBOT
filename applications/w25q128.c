/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-21     cisco       the first version
 */

#include "drv_spi.h"
#include <rtdevice.h>

// 注册 spi 设备
static int rt_hw_spi_flash_init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_PIN_6);
    if (RT_NULL == rt_sfud_flash_probe("W25Q128", "spi10"))
    {
        return -RT_ERROR;
    };
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_spi_flash_init);
