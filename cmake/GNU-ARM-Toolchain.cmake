set(CMAKE_SYSTEM_NAME Generic)
# 设置ARM 会引用-isysroot目录
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_VERBOSE_MAKEFILE ON)

# set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# INCLUDE(CMakeForceCompiler)
if(NOT DEFINED gcc_path)
  set(gcc_path "C:/Users/cisco/gcc-arm-none-eabi-5_4-2016q3")
  # set(gcc_path /home/cisco/Documents/gcc-arm-none-eabi-5_4-2016q3)
  # set(gcc_path /home/cisco/Documents/gcc-arm-none-eabi-9-2020-q2-update)
endif()

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
 set(COMMAND_SUFFIX ".exe")
else()
 set(COMMAND_SUFFIX "") 
endif()

set(CMAKE_C_COMPILER ${gcc_path}/bin/arm-none-eabi-gcc${COMMAND_SUFFIX})
set(CMAKE_CXX_COMPILER ${gcc_path}/bin/arm-none-eabi-g++${COMMAND_SUFFIX})
set(CMAKE_ASM_COMPILER ${gcc_path}/bin/arm-none-eabi-gcc${COMMAND_SUFFIX})
set(CMAKE_AR ${gcc_path}/bin/arm-none-eabi-ar${COMMAND_SUFFIX})
set(CMAKE_RANLIB ${gcc_path}/bin/arm-none-eabi-ranlib${COMMAND_SUFFIX})
set(CMAKE_OBJCOPY ${gcc_path}/bin/arm-none-eabi-objcopy${COMMAND_SUFFIX})
set(CMAKE_OBJDUMP ${gcc_path}/bin/arm-none-eabi-objdump${COMMAND_SUFFIX})
set(ARM_SIZE_EXECUTABLE ${gcc_path}/bin/arm-none-eabi-size${COMMAND_SUFFIX})
# SET(TOOLCHAIN) SET(CMAKE_SYSROOT
# "/Users/touchmii/Documents/gcc-arm-none-eabi-5_4-2016q3")

# SET (CMAKE_C_COMPILER_WORKS 1) SET (CMAKE_CXX_COMPILER_WORKS 1)

# if ( APPLE ) string ( REPLACE "-Wl,-search_paths_first" "" CMAKE_C_LINK_FLAGS
# ${CMAKE_C_LINK_FLAGS} ) string ( REPLACE "-Wl,-search_paths_first" ""
# CMAKE_CXX_LINK_FLAGS ${CMAKE_CXX_LINK_FLAGS} ) endif () search for programs in
# the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
