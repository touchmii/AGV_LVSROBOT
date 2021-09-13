/*
 * 程序清单：这是一个 PIN 设备使用例程
 * 例程导出了 pin_beep_sample 命令到控制终端
 * 命令调用格式：pin_beep_sample
 * 程序功能：通过按键控制蜂鸣器对应引脚的电平状态控制蜂鸣器
*/

#include <rtthread.h>
#include <rtdevice.h>

/* 引脚编号，通过查看设备驱动文件drv_gpio.c确定 */
#ifndef BEEP_PIN_NUM
    #define BEEP_PIN_NUM            138  /* PI10 */
#endif
#ifndef KEY0_PIN_NUM
    #define SENSOR0_PIN_NUM            106  /* PG10 */
#endif
#ifndef KEY1_PIN_NUM
    #define SENSOR1_PIN_NUM            111  /* PG15 */
#endif

#define RELAY_BK 92 //PF12

void sensor0_on(void *args)
{
    rt_kprintf("sensor0 on beep!\n");

    rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
    rt_pin_write(RELAY_BK, PIN_HIGH);
}

void sensor1_off(void *args)
{
    rt_kprintf("sensor1n off beep!\n");

    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);
    rt_pin_write(RELAY_BK, PIN_LOW);
}

static void sensor_sample(void)
{
    rt_pin_mode(RELAY_BK, PIN_MODE_OUTPUT);
    rt_pin_write(RELAY_BK, PIN_LOW);
    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_LOW);

    /* 按键0引脚为输入模式 */
    rt_pin_mode(SENSOR0_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_on */
    rt_pin_attach_irq(SENSOR0_PIN_NUM, PIN_IRQ_MODE_FALLING, sensor0_on, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(SENSOR0_PIN_NUM, PIN_IRQ_ENABLE);

    /* 按键1引脚为输入模式 */
    rt_pin_mode(SENSOR1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    /* 绑定中断，下降沿模式，回调函数名为beep_off */
    rt_pin_attach_irq(SENSOR1_PIN_NUM, PIN_IRQ_MODE_FALLING, sensor1_off, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(SENSOR1_PIN_NUM, PIN_IRQ_ENABLE);
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(sensor_sample, pin beep sample);