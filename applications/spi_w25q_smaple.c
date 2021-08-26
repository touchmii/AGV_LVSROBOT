/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-21     cisco       the first version
 */
#include <rtdevice.h>

#define W25Q_SPI_DEVICE_NAME     "spi20"

static void spi_w25q_sample(int argc, char *argv[])
{
    struct rt_spi_device *spi_dev_w25q;
    char name[RT_NAME_MAX];
    rt_uint8_t w25x_read_id = 0x90;
    rt_uint8_t id[5] = {0};

    if(argc == 2){
        rt_strncpy(name, argv[1], RT_NAME_MAX);
    } else {
        rt_strncpy(name, W25Q_SPI_DEVICE_NAME, RT_NAME_MAX);
    }

    spi_dev_w25q = (struct rt_spi_device *)rt_device_find(name);
    if(!spi_dev_w25q){
        rt_kprintf("spi sample run failed! can't find %s device!\n", name);
    } else {

         struct rt_spi_configuration cfg;
         cfg.data_width = 8;
         cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
         cfg.max_hz = 50 * 1000 * 6 ; /* 50M */
         rt_spi_configure(spi_dev_w25q, &cfg);

        rt_spi_send_then_recv(spi_dev_w25q, &w25x_read_id, 1, id, 5);
        rt_kprintf("use rt_spi_send_then_recv() read w25q ID is %x%x\n", id[3], id[4]);

        struct rt_spi_message msg1, msg2;
        msg1.send_buf = &w25x_read_id;
        msg1.recv_buf = RT_NULL;
        msg1.length = 1;
        msg1.cs_take = 1;
        msg1.cs_release = 0;
        msg1.next = &msg2;

        msg2.send_buf = RT_NULL;
        msg2.recv_buf = id;
        msg2.length = 5;
        msg2.cs_take = 0;
        msg2.cs_release = 1;
        msg2.next = RT_NULL;

        rt_spi_transfer_message(spi_dev_w25q, &msg1);
        rt_kprintf("use rt_spi_transfer_mesage() read w25q ID is :%x%x\n", id[3], id[4]);
    }

}
MSH_CMD_EXPORT(spi_w25q_sample, spi_w25q_sample samole);
