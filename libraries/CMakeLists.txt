# set(GLOB_RECURSE HAL_SRCS ./STM32F4xx_HAL_Driver/Src/*/c )
set(HAL_DIR ${CMAKE_CURRENT_LIST_DIR})

set(HAL_SRCS
    ${HAL_DIR}/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
    # ${HAL_DIR}/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407.s
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
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
)
    
if (DEFINED RT_USING_SERIAL)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
    )
endif()

if (DEFINED RT_USING_I2C)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    )
endif()

if (DEFINED RT_USING_SPI)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_qspi.c
    )
endif()

if (DEFINED RT_USING_USB_HOST OR DEFINED RT_USING_USB_HOST)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pccard.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    )
endif()

if (DEFINED RT_USING_CAN)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    )
endif()

if (DEFINED RT_USING_HWTIMER OR DEFINED RT_USING_PWM OR DEFINED RT_USING_PULSE_ENCODER)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_lptim.c
    )
endif()

if (DEFINED BSP_USING_ETH)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c
    )
endif()

if (DEFINED RT_USING_ADC)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    )
endif()

if (DEFINED RT_USING_DAC)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c
    )
endif()

if (DEFINED RT_USING_RTC)
list(APPEND HAL_SRCS
${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
)
endif()

if (DEFINED RT_USING_WDT)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c
    )
endif()

if (DEFINED RT_USING_SDIO)
list(APPEND HAL_SRCS
  ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
  ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
    )
endif()

if (DEFINED RT_USING_AUDIO)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai_ex.c
    )
endif()

if (DEFINED RT_USING_MTD_NOR)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nor.c
    )
endif()

if (DEFINED RT_USING_MTD_NAND)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nand.c
    )
endif()

if (DEFINED BSP_USING_FMC)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c
    )
endif()

if (DEFINED BSP_USING_SDRAM)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c
    
    )
    endif()
    
if (DEFINED BSP_USING_EXT_FMC_IO)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c
    )
endif()

if (DEFINED BSP_USING_ON_CHIP_FLASH)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    )
endif()

if (DEFINED BSP_USING_LTDC)
  list(APPEND HAL_SRCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc_ex.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma2d.c
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dsi.c
    )
endif()

set(HAL_INCS
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Inc
    ${HAL_DIR}/STM32F4xx_HAL_Driver/Inc/Legacy
    ${HAL_DIR}/CMSIS/Device/ST/STM32F4xx/Include ${HAL_DIR}/CMSIS/Include
    CACHE INTERNAL "")

set(HAL_SRCS
  ${HAL_SRCS}
  CACHE INTERNAL ""  
)    
add_library(HAL STATIC ${HAL_SRCS}) 
target_include_directories(HAL PUBLIC ${HAL_INCS}) 
target_include_directories(HAL PUBLIC ${PROJECT_SOURCE_DIR}/drivers)
