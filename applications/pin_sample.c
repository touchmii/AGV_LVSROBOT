#include "rtthread.h"
#include "rtdevice.h"
#include <drv_common.h>
#include "finsh.h"


#define     LEDR_PIN        GET_PIN(E, 6)
#define     KEY1_PIN        GET_PIN(E, 0)
#define     KEY2_PIN        GET_PIN(E, 1)

/*****************************************************************
*   函数：  void ledr_on(void *parameter)
*   参数：  空指针
*   返回值：无
*   说明：  led亮
******************************************************************/
void ledr_on(void *parameter)
{
    
    rt_kprintf("key1 press down!\n");
    rt_pin_write(LEDR_PIN, PIN_LOW);
}

/*****************************************************************
*   函数：  void ledr_off(void *parameter)
*   参数：  空指针
*   返回值：无
*   说明：  led灭
******************************************************************/
void ledr_off(void *parameter)
{
    
    rt_kprintf("key2 press down!\n");
    rt_pin_write(LEDR_PIN, PIN_HIGH);
}

/*****************************************************************
*   函数：  static void key_led_entry(void *paraemter)
*   参数：  空指针
*   返回值：无
*   说明：  按键中断控制灯亮灭线程
******************************************************************/
static void key_led_entry(void *paraemter)
{
    
    while (1)
    {
    
        rt_thread_mdelay(10);
        rt_pin_attach_irq(KEY1_PIN, PIN_IRQ_MODE_RISING, ledr_on, RT_NULL);     /* 绑定按键1中断回调函数ledr_on，上升沿触发*/
        rt_pin_attach_irq(KEY2_PIN, PIN_IRQ_MODE_RISING, ledr_off, RT_NULL);    /* 绑定按键2中断回调函数ledr_off，上升沿触发*/

        rt_pin_irq_enable(KEY1_PIN, PIN_IRQ_ENABLE);    /* 使能按键1中断 */
        rt_pin_irq_enable(KEY2_PIN, PIN_IRQ_ENABLE);    /* 使能按键2中断 */
    }
    
}

/***************************************************************
*   函数：  static void key_led_sample(int argc, char *argv[])
*   参数：  空指针
*   返回值：无  
*   说明：  启动线程
*****************************************************************/
static void key_led_sample(int argc, char *argv[])
{
    
    static rt_thread_t key_led_tid = RT_NULL;       /* 定义线程控制块 */

    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLDOWN); /* 设置按键1对应引脚的工作模式 */
    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT_PULLDOWN); /* 设置按键2对应引脚的工作模式 */
    rt_pin_mode(LEDR_PIN, PIN_MODE_OUTPUT);         /* 设置ledr对应引脚的工作模式 */
    rt_pin_write(LEDR_PIN, PIN_HIGH);               /* 设置ledr默认输出高电平，灯不亮 */

    key_led_tid = rt_thread_create("key_led_tid", key_led_entry, RT_NULL,
                                    512, 8, 10);     /* 创建线程 */

    if (key_led_tid != RT_NULL)
    {
    
        rt_thread_startup(key_led_tid);         /* 启动线程 */
    }
    
}


MSH_CMD_EXPORT(key_led_sample, key led sample); /* 导出至FinSH控制台 */