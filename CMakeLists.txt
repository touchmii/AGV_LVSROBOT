cmake_minimum_required(VERSION 3.1)

project(AGV_RTOS C CXX ASM)

set(DEVICE_FAMILY STM32F40_41xxx)
set(Cortex_FAMILY ARM_MATH_CM4)

# (?<=exp2)exp1：查找 exp2 后面的 exp1。
file(STRINGS rtconfig.h RT_CONFIG REGEX "^#define.*$")
foreach(define ${RT_CONFIG})
  # string(STRIP "${define}" output)
  string(REPLACE " " ";" D_LIST "${define}")
  # separate_arguments(${define}) string(REGEX MATCHALL "[a-zA-Z]+\ |[a-zA-Z]+$"
  # SEXY_LIST "${define}")
  list(LENGTH D_LIST define_len)
  # message("define_len: ${define_len}") message("1 define ${D_LIST}")
  if(${define_len} STREQUAL 2)
    # list操作不需要${}
    list(GET D_LIST 1 dd1)
    set("${dd1}" CACHE INTERNAL "")
    # message( "one define: ${dd1}")
  elseif(${define_len} STREQUAL 3)
    list(GET D_LIST 1 dd1)
    list(GET D_LIST 2 dd2)
    set("${dd1}"
        ${dd2}
        CACHE INTERNAL "")
    # message("two define: ${dd1}=${dd2}")
  endif()

  # foreach(xx ${D_LIST}) message(${xx}) endforeach() message(${define})
endforeach()

# message(${RT_CONFIG})


# if(${CMAKE_BUILD_TYPE} STREQUAL RelWithDebInfo)
#   set(mfloat-flag "soft")
# else()
# endif()
set(mfloat-flag "hard")

# 一定要在添加子目录前定义参数， 相当于使用#define STM32F407xx，一定要定义此参数，否则HAL库无法判断芯片类型
# -DSOC_SERIES_STM32F4 -DSTM32F407xx -DUSE_HAL_DRIVER
# 非常重要的宏定义,没有的话使用HAL会产生各种未定义错误
add_definitions(
  -DARM_MATH_CM4 -DSOC_SERIES_STM32F4 -DSTM32F407xx -DUSE_HAL_DRIVER
  -Dflash_layout -DDEBUG_DEFAULT_INTERRUPT_HANDLERS)
set(CMAKE_C_FLAGS
    "-std=c99 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=${mfloat-flag} -ffunction-sections -fdata-sections"
)
set(CMAKE_CXX_FLAGS
    " -std=c++11 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=${mfloat-flag} -ffunction-sections -fdata-sections"
)
set(CMAKE_ASM_FLAGS
    " -c -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=${mfloat-flag} -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb"
)

# set(CMAKE_EXE_LINKER_FLAGS " -Wl,-Map=${CMAKE_PROJECT_NAME}.map
# -Wl,-gc-sections --specs=nano.specs --specs=nosys.specs,-cref,-u,Reset_Handler
# -T${CMAKE_SOURCE_DIR}/linkerscripts/STM32F407IG/link.lds" )
set(CMAKE_EXE_LINKER_FLAGS
    " -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=${CMAKE_PROJECT_NAME}.map,-cref,-u,Reset_Handler -T ${CMAKE_SOURCE_DIR}/linkscripts/STM32F407IG/link.lds"
)
# set(CMAKE_EXE_LINKER_FLAGS " -Wl,-Map=${CMAKE_PROJECT_NAME}.map
# -Wl,-gc-sections --specs=nano.specs --specs=nosys.specs
# -T${CMAKE_SOURCE_DIR}/LinkerScripts/STM32F407VE_flash.lds" )
# set(additional_flags -O0 -g) if(CMAKE_BUILD_TYPE MATCHES Debug)
# set(additional_flags -O0 -g)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s") else()
# set(additional_flags -O3 -s) endif() include(drivers/Drivers.cmake)
# include(libraries/HAL.cmake) include(rt-thread/rt-thread.cmake)
add_subdirectory(drivers)
add_subdirectory(libraries)
add_subdirectory(rt-thread)
add_subdirectory(packages)

include_directories(
  ${PROJECT_SOURCE_DIR} ${rt-thread_INCS} ${HAL_INCS} ${packages_INCS}
  ${xdrivers_INCS} ${PROJECT_SOURCE_DIR}/applications)

# message("rt thread include path ${rt-thread_INCS}")

# 晶振频率8Mhz stm32f4xx_hal_conf.h已经定义 set(HSE_VALUE 8000000)

file(
  GLOB
  source_files
  ${HAL_SRCS}
  ${rt-thread_SRCS}
  ${packages_SRCS}
  ${xdrivers_SRC}
  ${PROJECT_SOURCE_DIR}/applications/*.c
  ${PROJECT_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m4/context_gcc.S
  ${PROJECT_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m4/cpuport.c
  ${PROJECT_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
)

add_executable(${CMAKE_PROJECT_NAME}.elf ${source_files})

set(elf_file ${CMAKE_PROJECT_NAME}.elf)
set(hex_file ${CMAKE_PROJECT_NAME}.hex)
set(bin_file ${CMAKE_PROJECT_NAME}.bin)
set(lss_file ${CMAKE_PROJECT_NAME}.lss)
# show size of resulting firmware image
add_custom_target(
  ${elf_file}-size
  DEPENDS ${elf_file}
  COMMAND ${ARM_SIZE_EXECUTABLE} -B ${elf_file})

# generate extended listing
add_custom_target(
  ${lss_file}
  DEPENDS ${elf_file}
  COMMAND ${CMAKE_OBJDUMP} -S ${elf_file} > ${lss_file})

# create binary and hex files
add_custom_target(
  ${hex_file}
  DEPENDS ${elf_file}
  COMMAND ${CMAKE_OBJCOPY} -Oihex ${elf_file} ${hex_file})
add_custom_target(
  ${bin_file}
  DEPENDS ${elf_file}
  COMMAND ${CMAKE_OBJCOPY} -Obinary ${elf_file} ${bin_file})
add_custom_target(${application_name} ALL DEPENDS ${elf_file}-size ${bin_file}
                                                  ${hex_file} ${lss_file})

# target_link_libraries( ${CMAKE_PROJECT_NAME}.elf # PUBLIC RT-Thread PUBLIC
# stm32f4xx PUBLIC #
# ${PROJECT_SOURCE_DIR}/STM32F4xx_DSP_StdPeriph_Lib/DSP_Lib/libarm_cortexM4lf_math.a
# )

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
  # find_program(JLINK JLink PATHS "C:/ProgramFiles (x86)/SEGGER/JLink" DOC
  # "Path to `JLink.exe` command line executable")
  find_program(JLINKGDBSERVER JLinkGDBServerCL
               DOC "Path to `JLinkGDBServerCL.exe` command line executable")
  find_program(JLINKRTTCLIENT JLinkRTTClient
               DOC "Path to `JLinkRTTClient.exe` command line executable")
  if(NOT JLINK)
    set(JLINK "$ENV{ProgramFiles\(x86\)}/SEGGER/JLink/JLink.exe")
  endif()
  if(NOT JLINKGDBSERVER)
    set(JLINKGDBSERVER
        "$ENV{ProgramFiles\(x86\)}/SEGGER/JLink/JLinkGDBServerCL.exe")
  endif()
  if(NOT JLINKRTTCLIENT)
    set(JLINKRTTCLIENT
        "$ENV{ProgramFiles\(x86\)}/SEGGER/JLink/JLinkRTTClient.exe")
  endif()
else()
  find_program(JLINK JLinkExe DOC "Path to `JLinkExe` command line executable")
  find_program(JLINKGDBSERVER JLinkGDBServer
               DOC "Path to `JLinkGDBServer` command line executable")
  find_program(JLINKRTTCLIENT JLinkRTTClient
               DOC "Path to `JLinkRTTClient` command line executable")
endif()

if(JLINK)
  message("-- Found JLinkExe: ${JLINK}")
endif()
if(JLINKGDBSERVER)
  message("-- Found JLinkGDBServer: ${JLINKGDBSERVER}")
endif()
if(JLINKRTTCLIENT)
  message("-- Found JLinkRTTClient: ${JLINKRTTCLIENT}")
endif()

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
  set(TERMINAL "open")
  set(COMMAND_SUFFIX "")
elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
  find_program(CMD cmd)
  set(TERMINAL
      ${CMD}
      /c
      start
      powershell
      -noexit
      -ExecutionPolicy
      Bypass
      -File)
  set(POST_OPTIONS -JLinkPath ${JLINK} -JLinkGDBServerPath ${JLINKGDBSERVER}
                   -JLinkRTTClientPath ${JLINKRTTCLIENT})
  set(COMMAND_SUFFIX ".ps1")
else()
  set(TERMINAL "gnome-terminal" --)
  set(COMMAND_SUFFIX "")
endif()

if(EXISTS "${JLINK}")

  file(WRITE ${PROJECT_BINARY_DIR}/download.jlink
       "loadbin \"${hex_file}\" 0x08000000\nr\ng\nqc")
  add_custom_command(
    OUTPUT jlink_download
    COMMAND # ${TERMINAL}
            ${JLINK} -ExitOnError 1 -device STM32F407IG -Speed 4000 -IF SWD
            "download.jlink"
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    VERBATIM)
  add_custom_target(START_JLINK_DOWNLOAD DEPENDS ${hex_file} jlink_download)
  if(EXISTS "${JLINKGDBSERVER}" AND EXISTS "${JLINKRTTCLIENT}")
    add_custom_target(
      START_JLINK_ALL ALL
      COMMAND
        ${TERMINAL} "${PROJECT_SOURCE_DIR}/runJLinkGDBServer${COMMAND_SUFFIX}"
        ${POST_OPTIONS}
      COMMAND ${TERMINAL} "${PROJECT_SOURCE_DI}/runJLinkExe${COMMAND_SUFFIX}"
              ${POST_OPTIONS}
      COMMAND cmake -E sleep 2
      COMMAND
        ${TERMINAL} "${PROJECT_SOURCE_DI}/runJLinkRTTClient${COMMAND_SUFFIX}"
        ${POST_OPTIONS}
      COMMENT "started JLink commands"
      VERBATIM)
  endif()
  if(EXISTS "${JLINKRTTCLIENT}")
    add_custom_target(
      START_JLINK_RTT ALL
      COMMAND ${TERMINAL} "${PROJECT_SOURCE_DI}/runJLinkExe${COMMAND_SUFFIX}"
              ${POST_OPTIONS}
      COMMAND cmake -E sleep 2
      COMMAND
        ${TERMINAL} "${PROJECT_SOURCE_DI}/runJLinkRTTClient${COMMAND_SUFFIX}"
        ${POST_OPTIONS}
      COMMENT "started JLink RTT terminal"
      VERBATIM)
  else()
    message(
      WARNING
        "The path to the JLinkRTTClient utility (JLINKRTTCLIENT) is not set or does not exist, so START_JLINK_RTT and START_JLINK_ALL targets will not be available"
    )
  endif()
  if(EXISTS "${JLINKGDBSERVER}")
    add_custom_target(
      START_JLINK_GDBSERVER ALL
      COMMAND ${TERMINAL} "${PROJECT_SOURCE_DI}/runJLinkExe${COMMAND_SUFFIX}"
              ${POST_OPTIONS}
      COMMAND cmake -E sleep 2
      COMMAND
        ${TERMINAL} "${PROJECT_SOURCE_DI}/runJLinkGDBServer${COMMAND_SUFFIX}"
        ${POST_OPTIONS}
      COMMENT "started JLink GDB server"
      VERBATIM)
  else()
    message(
      WARNING
        "The path to the JLinkGDBServer utility (JLINKGDBSERVER) is not set or does not exist, so START_JLINK_GDBSERVER and START_JLINK_ALL targets will not be available"
    )
  endif()
else()
  message(
    WARNING
      "The path to the JLink utility (JLINK) is not set or does not exist, so START_JLINK_* targets will not be available"
  )
endif()
