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


#define APPNAME "AGV_RTOS"
#define HARDWARE_VERSION "V1.0.0"
#define SOFTWARE_VERSION "V0.0.1"

#define LOG_TAG     "main"     // 该模块对应的标签。不定义时，默认：NO_TAG
#define LOG_LVL     LOG_LVL_DBG   // 该模块对应的日志输出级别。不定义时，默认：调试级别
#include <ulog.h>                 // 必须在 LOG_TAG 与 LOG_LVL 下面
#include <rtdbg.h>
#include "cm_backtrace.h"

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
    // LOG_D("LOG_D(%d): RT-Thread is an open source IoT operating system from China.", count);
    // LOG_I("LOG_I(%d): RT-Thread is an open source IoT operating system from China.", count);
    // LOG_W("LOG_W(%d): RT-Thread is an open source IoT operating system from China.", count);
    // LOG_E("LOG_E(%d): RT-Thread is an open source IoT operating system from China.", count);
    // LOG_RAW("LOG_RAW(%d): RT-Thread is an open source IoT operating system from China.", count);

  }

  return RT_EOK;
}
