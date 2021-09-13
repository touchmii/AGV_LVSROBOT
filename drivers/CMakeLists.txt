set(xdrivers_SRCS
  ${CMAKE_CURRENT_LIST_DIR}/drv_common.c
  ${CMAKE_CURRENT_LIST_DIR}/board.c
  ${CMAKE_CURRENT_LIST_DIR}/drv_clk.c
)

if(DEFINED RT_USING_PIN)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_gpio.c
  )
endif()
 
if(DEFINED RT_USING_SERIAL)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_usart.c
  )
endif()
 
if(DEFINED RT_USING_HWTIMER)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_hwtimer.c
  )
endif()
 
if(DEFINED RT_USING_PWM)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_pwm.c
  )
endif()
 
if(DEFINED RT_USING_SPI)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_spi.c
  )
endif()
 
if(DEFINED RT_USING_QSPI)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_qspi.c
  )
endif()
 
if(DEFINED RT_USING_I2C AND DEFINED RT_USING_I2C_BITOPS)
  if(DEFINED BSP_USING_I2C1 OR DEFINED BSP_USING_I2C2 OR DEFINED BSP_USING_I2C3 OR DEFINED BSP_USING_I2C4)
    list(APPEND xdrivers_SRCS 
      ${CMAKE_CURRENT_LIST_DIR}/drv_soft_i2c.c
    )
  endif()
endif()
 
if(DEFINED RT_USING_ETH AND DEFINED RT_USING_LWIP)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_eth.c
  )
endif()
 
if(DEFINED RT_USING_ADC)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_adc.c
  )
endif()
   
if(DEFINED RT_USING_DAC)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_dac.c
  )
endif()

if(DEFINED RT_USING_CAN)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_can.c
  )
endif()
     
if(DEFINED RT_USING_PM AND DEFINED SOC_SERIES_STM32L4)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_pm.c 
    ${CMAKE_CURRENT_LIST_DIR}/drv_lptim.c
  )
endif()
     
if(DEFINED RT_USING_SDRAM)
  list(APPEND xdrivers_SRCS 
   ${CMAKE_CURRENT_LIST_DIR}/drv_sdram.c
  )
endif()
       
if(DEFINED RT_USING_LCD)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_lcd.c
  )
endif()
       
if(DEFINED RT_USING_LCD_MIPI)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_lcd_mipi.C
  )
endif()
         
if(DEFINED RT_USING_ONCHIP_RTC)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_rtc.c
  )
endif()
           
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32F0)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_f0.c 
  )
endif()
               
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32F1)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_f1.c 
  )
endif()
               
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32F2)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_f2.c 
  )
endif()
               
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32F3)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_f3.c 
  )
endif()
                    
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32F4)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_f4.c 
  )
endif()
                      
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32F7)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_f7.c 
  )
endif()
                      
if(DEFINED RT_USING_ONCHIP_FLASH AND DEFINED SOC_SERIES_STM32L4)
  list(APPEND xdrivers_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash/drv_flash_l4.c 
  )
endif()
         
if(DEFINED RT_USING_HWCRYPTO)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_crypto.c
  )
endif()
           
if(DEFINED RT_USING_WDT)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_wdt.c
  )
endif()
             
if(DEFINED RT_USING_SDIO)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_sdio.c
  )
endif()
             
if(DEFINED RT_USING_USBD)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_usbd.c
  )
endif()
                 
if(DEFINED RT_USING_USBH)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_usbh.c
  )
endif()
               
if(DEFINED RT_USING_PULSE_ENCODER)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_pulse_encoder.c
  )
endif()

if(DEFINED RT_USING_USING_ON_CHIP_FLASH)
  list(APPEND xdrivers_SRCS 
    ${CMAKE_CURRENT_LIST_DIR}/drv_flash.c
  )
endif()

set(xdrivers_SRCS
  ${xdrivers_SRCS}
  CACHE INTERNAL ""
)  

set(xdrivers_INCS
  ${CMAKE_CURRENT_LIST_DIR} 
  ${CMAKE_CURRENT_LIST_DIR}/include 
  ${CMAKE_CURRENT_LIST_DIR}/include/config
  CACHE INTERNAL ""
)
