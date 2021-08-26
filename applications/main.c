/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-21     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include "cm_backtrace.h"
#include <rtdbg.h>

#define APPNAME "AGV_YS-F4"
#define HARDWARE_VERSION "V1.0.0"
#define SOFTWARE_VERSION "V0.0.1"

void w25q128_mount(void) {
  rt_device_t dev;
  dev = rt_device_find("W25Q128");
  if (dev != RT_NULL) {
    if (dfs_mount("W25Q128", "/", "elm", 0, 0) == 0) {
      rt_kprintf("spi_flash mount to spi!\n");
    } else {
      rt_kprintf("spi_flash mount to spi failed!\n");
    }
  }
}
int main(void) {
  rt_kprintf("CmBacktrace Test...\r\n");
  cm_backtrace_init(APPNAME, HARDWARE_VERSION, SOFTWARE_VERSION);
  w25q128_mount();
  int count = 1;

  while (count++) {
    // LOG_D("Hello RT-Thread!");
    rt_thread_mdelay(1000);
    // rt_kprintf("xxxxx");
  }

  return RT_EOK;
}
