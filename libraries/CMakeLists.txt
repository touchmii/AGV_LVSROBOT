# set(GLOB_RECURSE HAL_SRCS ./STM32F4xx_HAL_Driver/Src/*/c )
set(HAL_DIR ${CMAKE_CURRENT_LIST_DIR})

set(HAL_SRCS
    ${HAL_DIR}/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
    ${HAL_DIR}/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407.s
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    # ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/Legacy/stm32f4xx_hal_can.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cec.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_qspi.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pccard.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_lptim.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai_ex.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nor.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nand.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc_ex.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma2d.c
    # STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dsi.c
    CACHE INTERNAL "")

if(DEFINED RT_USING_CAN)
  # message("RT_USING_CAN")
  list(APPEND HAL_SRCS ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c)
  # message(${HAL_SRCS})
endif()

set(HAL_INCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Inc
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Inc/Legacy
    ${HAL_DIR}/CMSIS/Device/ST/STM32F4xx/Include ${HAL_DIR}/CMSIS/Include
    CACHE INTERNAL "")

# add_library(HAL ${HAL_SRCS}) target_include_directories(HAL PUBLIC
# ${HAL_INCS}) target_include_directories(HAL PUBLIC
# ${PROJECT_SOURCE_DIR}/drivers)
