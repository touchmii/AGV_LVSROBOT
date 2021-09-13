#include <rtthread.h>
#include <rtdevice.h>
#include "ps2.h"
#include "string.h"
#include "stdlib.h"

#define CAN_DEV_NAME       "can1"      /* CAN 设备名称 */

static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */
static rt_device_t can_dev;            /* CAN 设备句柄 */

#define SAMPLE_UART_NAME       "uart4"

#define TRUN_INC 3313676/130

/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
static rt_device_t serial;

struct serial_configure uart_config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

static rt_int32_t steer_speed = 0;
static rt_int32_t trun_loc = 0;
static char datt[32];


#define CS_PIN 37 // PC5
#define CLK_PIN 80 // PF0
#define DO_PIN 41 //PC9
#define DI_PIN 108 // PG12

#define STEER_ID  2
#define TURN_ID  3
#define LOC_S 1000*17896
rt_uint8_t RESET_ERROR[8] = {0x2b, 0x40, 0x60, 0x00, 0x86, 0x00, 0x00, 0x00};
rt_uint8_t CTRL_F[8] = {0x2b, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
rt_uint8_t SPEED_MODE[8] = {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
rt_uint8_t LOC_MODE[8] = {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
rt_uint8_t SPEED_150[8] = {0x23, 0xFF, 0x60, 0x00, 0x00, 0x40, 0x06, 0x00};
rt_uint8_t SPEED_200[8] = {0x2b, 0xF0, 0x2F, 0x09, 0xc8, 0x00, 0x00, 0x00};
rt_uint8_t LOC[8] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
int LOC_SPEED[8] = {0x23, 0x81, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};



/* 串口接收消息结构*/
struct rx_msg
{
  rt_device_t dev;
  rt_size_t size;
};
/* 串口设备句柄 */
static rt_device_t serial;
/* 消息队列控制块 */
static struct rt_messagequeue rx_mq;

static void steer_motor_init(rt_device_t can) {
    struct rt_can_msg msg = {0};
    int size = 0;
    msg.id = 0x600 + STEER_ID;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 6;
    struct rt_can_msg speed_msg = {0};
    speed_msg.id = 0x600 + STEER_ID;
    speed_msg.ide = RT_CAN_STDID;
    speed_msg.rtr = RT_CAN_DTR;
    speed_msg.len = 8;
    for(int i=0; i<8; i++){
      speed_msg.data[i] = SPEED_200[i];
    }
    struct rt_can_msg loc_msg = {0};
    loc_msg.id = 0x600 + TURN_ID;
    loc_msg.ide = RT_CAN_STDID;
    loc_msg.rtr = RT_CAN_DTR;
    loc_msg.len = 8;
    for(int i=0; i<8; i++){
      loc_msg.data[i] = LOC[i];
    }
//    msg.data = SPEED_MODE;
    for(int i=0; i<8; i++){
      msg.data[i] = SPEED_MODE[i];
    }
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    // HOME mode
    rt_thread_delay(100);
    msg.data[4] = 6;
    msg.id = 0x600 + TURN_ID;
    size = rt_device_write(can, 0, &msg, sizeof(msg));

    rt_thread_delay(1000);
//    msg.data = RESET_ERROR;
    for(int i=0; i<8; i++){
      msg.data[i] = RESET_ERROR[i];
    }
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    rt_thread_delay(100);
    msg.id = 0x600 + TURN_ID;
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    rt_thread_delay(1000);
//    msg.data = CTRL_F;
    for(int i=0; i<8; i++){
      msg.data[i] = CTRL_F[i];
    }
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    // go home F -> 1F
    rt_thread_delay(100);
    msg.id = 0x600 + TURN_ID;
    msg.data[4] = 0xF;
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    rt_thread_delay(100);
    msg.data[4] = 0x1F;
//    msg.id = 0x600 + TURN_ID;
    size = rt_device_write(can, 0, &msg, sizeof(msg));

    rt_thread_delay(20000);

    //    msg.data = LOC_MODE;
    for(int i=0; i<8; i++){
      msg.data[i] = LOC_MODE[i];
    }
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    rt_thread_delay(1000);
    // set loc speed
    for(int i=0; i<8; i++){
      msg.data[i] = LOC_SPEED[i];
    }
    msg.data[4] = LOC_S & 0xFF;
    msg.data[5] = LOC_S >> 8 & 0xFF;
    msg.data[6] = LOC_S >> 16 & 0xFF;
    msg.data[7] = LOC_S >> 24 & 0xFF;
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    rt_thread_delay(1000);



    //    2F->3F
    for(int i=0; i<8; i++){
      msg.data[i] = CTRL_F[i];
    }
    msg.data[4] = 0x2F;
    size = rt_device_write(can, 0, &msg, sizeof(msg));
    // go home F -> 1F
    rt_thread_delay(100);

    msg.data[4] = 0x3F;
    size = rt_device_write(can, 0, &msg, sizeof(msg));


//    rt_thread_delay(1000);
//    msg.len = 8;
////    msg.data = SPEED_150;
//    for(int i=0; i<8; i++){
//      msg.data[i] = SPEED_150[i];
//    }
//    size = rt_device_write(can, 0, &msg, sizeof(msg));
//    rt_thread_delay(10000);
//    msg.len = 8;
//    for(int i=0; i<8; i++){
//      msg.data[i] = SPEED_200[i];
//    }
//    size = rt_device_write(can, 0, &msg, sizeof(msg));
    int dd = 0;
    int loc_inc = 0;
    while(1) {
      rt_thread_delay(100);
      rt_kprintf("speed: %d, loc: %d \n", steer_speed, trun_loc);
      speed_msg.data[4] = steer_speed*10 & 0xFF;
      speed_msg.data[5] = steer_speed*10 >> 8 & 0xFF;
      size = rt_device_write(can, 0, &speed_msg, sizeof(msg));
//      dd++;
      loc_inc = trun_loc * TRUN_INC;
      loc_msg.data[4] = loc_inc & 0xFF;
      loc_msg.data[5] = loc_inc >> 8 & 0xFF;
      loc_msg.data[6] = loc_inc >> 16 & 0xFF;
      loc_msg.data[7] = loc_inc >> 24 & 0xFF;
      size = rt_device_write(can, 0, &loc_msg, sizeof(msg));
    }
    rt_device_close(can_dev);
    
}

void turn_motor_init(rt_device_t can) {

}

//int pair_stick(char cmdd[32], rt_int32_t *x_, rt_int32_t *y_) {
//  int is_left = 0;
//  int size_cmd = sizeof(cmdd);
//  rt_kprintf("cmd[1] : %s, sizeof: %d", cmdd, size_cmd);
//  for(int i = 0; i < size_cmd; i++) {
//    if (cmdd[i] == 'S' || cmdd[i] == 'P') {
//      if (i == 4) {
//        x_stick = atoi(cmdd[1]) * 100 + atoi(cmdd[2]) * 10 + atoi(cmdd[3]);
//      }
//      else if (i == 3) {
//        x_stick = atoi(cmdd[1]) * 10 + atoi(cmdd[2]);
//      }
//      else if (i == 2) {
//        x_stick = atoi(cmdd[1]);
//      }
//      if ((size_cmd - i) == 4) {
//        y_stick = atoi(cmdd[5]) * 100 + atoi(cmdd[6]) * 10 + atoi(cmdd[7]);
//      }
//      else if ((size_cmd - i) == 3) {
//        y_stick = atoi(cmdd[5]) * 10 + atoi(cmdd[6]);
//      }
//      else if ((size_cmd - i) == 2) {
//        y_stick =  atoi(cmdd[5]);
//      }
//
//    }
//  }
//  if (cmdd[0] == 'W') {
//    is_left = 1;
//  }
//  rt_kprintf("left x: %d, y: %d \n", x_stick, y_stick);
//  return is_left;
//}

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
  struct rx_msg msg;
  rt_err_t result;
  msg.dev = dev;
  msg.size = size;

  result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
  if ( result == -RT_EFULL)
  {
    /* 消息队列满 */
    rt_kprintf("message queue full！\n");
  }
  return result;
}

static void serial_thread_entry(void *parameter)
{
  char ch;
  char data[32];
  static char i = 0;
  int a = 127;
  int b = 127;
  int c = 127;
  int d = 127;

  struct rx_msg msg;
  rt_err_t result;
  rt_uint32_t rx_length;
  static char rx_buffer[RT_SERIAL_RB_BUFSZ + 1];

  rt_bool_t revc_ok = 0;
  char temp[6];
  int xx = 0;
  int tt = 0;
  rt_bool_t speed_button;


  while (1)
  {
    rt_memset(&msg, 0, sizeof(msg));
    /* 从消息队列中读取消息*/
    result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
    if (tt % 20 == 0) {
      a,b,c,d = 127,127,127,127;
      speed_button = 0;
      if (tt > 60000) {
        tt = 1;
      }
    }
    if (result == RT_EOK)
    {
      /* 从串口读取数据*/
      rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
//      rt_kprintf("rx_length: %d \n", rx_length);
      rx_buffer[rx_length] = '\0';
      if((rx_buffer[0] < 79 && rx_buffer[0] > 64)) {

        /* 通过串口设备 serial 输出读取到的消息 */
        // rt_device_write(serial, 0, rx_buffer, rx_length);
        /* 打印数据 */
        if( rx_buffer[0] == 'M') {

          speed_button = 1;
        }
        rt_kprintf("%s\n",rx_buffer);
      } else if ( rx_buffer[rx_length-1] == '\n') {
        rt_strncpy(data+xx, rx_buffer, rx_length+1);
        xx = 0;
//        int temp_len = strlen(temp);
//        for(int i = 0; i < temp_len; i++) {
//          data[i] = temp[i];
//        }
//        for(int i = 0; i < strlen(rx_buffer); i++) {
//          data[i+temp_len] = temp[i];
//        }
//        rt_kprintf("data: %s \n", data);
        if (data[0] == 'W') {

          char *p = strtok(data+1, 'P');
          a = atoi(p);
          int bb = 0;
          for(int i = 0; i < 8; i ++) {
            if(data[i] == 'P') {
              bb = i;
            }
          }
          //        rt_kprintf("strlen: %d", bb);
          char *pp = p + strlen(p) - 3;
          b = atoi(data+bb+1);


          //        rt_kprintf(*p);
        }
        else if (data[0] == 'Q') {

          char *p = strtok(data+1, 'S');
          c = atoi(p);
          int bb = 0;
          for(int i = 0; i < 8; i ++) {
            if(data[i] == 'S') {
              bb = i;
            }
          }
          //        rt_kprintf("strlen: %d", bb);
          char *pp = p + strlen(p) - 3;
          d = atoi(data+bb+1);


          //        rt_kprintf(*p);
        }



//        rt_kprintf("left x: %d, left y: %d, right x: %d, right y: %d \n", a, b, c, d);
        steer_speed = (127 - a)*(1+speed_button);
        trun_loc = d - 127;
      } else {
        rt_strncpy(data+xx, rx_buffer, rx_length);
        xx += rx_length;
      }
    }
    tt++;


  }
}

int motor_controller(int argc, char *argv[]) {

    //  ps2_init(CS_PIN, CLK_PIN, DO_PIN, DI_PIN);
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    static char msg_pool[256];
    char str[] = "hello RT-Thread!\r\n";


    /* 查找系统中的串口设备 */
    serial = rt_device_find("uart4");
    if (!serial)
    {
      rt_kprintf("find uart4 failed!\n");
      return RT_ERROR;
    }

    /* 初始化信号量 */
//    rt_sem_init(&rx_sem, "rx_sem2", 0, RT_IPC_FLAG_FIFO);
    /* 初始化消息队列 */
    rt_mq_init(&rx_mq, "rx_mq",
               msg_pool,                 /* 存放消息的缓冲区 */
               sizeof(struct rx_msg),    /* 一条消息的最大长度 */
               sizeof(msg_pool),         /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);        /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    /* 以中断接收及轮询发送模式打开串口设备 */
    /* step2：修改串口配置参数 */
    uart_config.baud_rate = BAUD_RATE_9600;        //修改波特率为 9600
    uart_config.data_bits = DATA_BITS_8;           //数据位 8
    uart_config.stop_bits = STOP_BITS_1;           //停止位 1
    uart_config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
    uart_config.parity    = PARITY_NONE;           //无奇偶校验位

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
//    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &uart_config);
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
    /* 发送字符串 */
//    rt_device_write(serial, 0, str, (sizeof(str) - 1));

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 15, 10);
    /* 创建成功则启动线程 */
  if (thread != RT_NULL)
    {
      rt_thread_startup(thread);
    }
    else
    {
      ret = RT_ERROR;
    }
  /* 查找 CAN 设备 */
  can_dev = rt_device_find(CAN_DEV_NAME);
  if (!can_dev)
    {
      rt_kprintf("find %s failed!\n", CAN_DEV_NAME);
      return RT_ERROR;
    }
  /* 以中断接收及发送方式打开 CAN 设备 */
  int res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX);
  rt_thread_t thread2 = rt_thread_create("motor", steer_motor_init, can_dev, 1024, 25, 10);
  if (thread != RT_NULL)
  {
    rt_thread_startup(thread2);
  }
  else
  {
    ret = RT_ERROR;
  }

//    steer_motor_init(can_dev);


}
//MSH_CMD_EXPORT(motor_controller, motor controller sample)
INIT_APP_EXPORT(motor_controller)