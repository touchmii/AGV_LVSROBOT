set(Drivers_DIR ${CMAKE_CURRENT_LIST_DIR})
set(xdrivers_SRC
    ${Drivers_DIR}/board.c
    ${Drivers_DIR}/drv_gpio.c
    ${Drivers_DIR}/drv_usart.c
    ${Drivers_DIR}/drv_hwtimer.c
    ${Drivers_DIR}/drv_pwm.c
    ${Drivers_DIR}/drv_pulse_encoderulse.c
    ${Drivers_DIR}/drv_spi.c
    # ${Drivers_DIR}/drv_qspi.c ${Drivers_DIR}/drv_soft_i2c.c
    ${Drivers_DIR}/drv_eth.c
    ${Drivers_DIR}/drv_adc.c
    ${Drivers_DIR}/drv_dac.c
    ${Drivers_DIR}/drv_can.c
    ${Drivers_DIR}/drv_clk.c
    # ${Drivers_DIR}/drv_pm.c ${Drivers_DIR}/drv_lptim.c
    # ${Drivers_DIR}/drv_sdram.c ${Drivers_DIR}/drv_lcd.c
    # ${Drivers_DIR}/drv_lcd_mipi.c
    ${Drivers_DIR}/drv_rtc.c
    # ${Drivers_DIR}/drv_flash/drv_flash_f0.c
    # ${Drivers_DIR}/drv_flash/drv_flash_f1.c
    # ${Drivers_DIR}/drv_flash/drv_flash_f2.c
    # ${Drivers_DIR}/drv_flash/drv_flash_f4.c
    # ${Drivers_DIR}/drv_flash/drv_flash_f7.c
    # ${Drivers_DIR}/drv_flash/drv_flash_l4.c ${Drivers_DIR}/drv_crypto.c
    ${Drivers_DIR}/drv_wdt.c 
    # ${Drivers_DIR}/drv_sdio.c
    # ${Drivers_DIR}/drv_usbd.c
    ${Drivers_DIR}/drv_usbh.c
    ${Drivers_DIR}/drv_pulse_encoder.c
    ${Drivers_DIR}/drv_common.c
    CACHE INTERNAL "")

if(DEFINED RT_USING_CAN)
  # message("RT_USING_CAN")
  list(APPEND xdrivers_SRCS ${Drivers_DIR}/drv_can.c)
  # message(${xdrivers_SRCS})
endif()

set(xdrivers_INCS
    ${Drivers_DIR} ${Drivers_DIR}/include ${Drivers_DIR}/include/config
    CACHE INTERNAL "")
