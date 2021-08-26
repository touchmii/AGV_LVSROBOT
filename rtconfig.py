import os

# toolchains options
ARCH = 'arm'
CPU = 'cortex-m4'
CROSS_TOOL = 'gcc'

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery, Keil MDK, IAR
PLATFORM = 'gcc'
# EXEC_PATH = 'C:/RT-ThreadStudio/repo/Extract/ToolChain_Support_Packages/ARM/GNU_Tools_for_ARM_Embedded_Processors/5.4.1/bin'
EXEC_PATH = '/home/cisco/Documents/gcc-arm-none-eabi-5_4-2016q3/bin'

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

PREFIX = 'arm-none-eabi-'
CC = PREFIX + 'gcc'
AS = PREFIX + 'gcc'
AR = PREFIX + 'ar'
CXX = PREFIX + 'g++'
LINK = PREFIX + 'gcc'
TARGET_EXT = 'elf'
SIZE = PREFIX + 'size'
OBJDUMP = PREFIX + 'objdump'
OBJCPY = PREFIX + 'objcopy'
DEVICE = ''
CFLAGS = ''
AFLAGS = ''
LFLAGS = '-T linkscripts//STM32F407IG//link.lds'
CPATH = ''
LPATH = ''
CXXFLAGS = ''
POST_ACTION = ''


# modified by rtthread.studio.vscode
DEVICE = ' -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections'
CFLAGS = DEVICE + ' -Dgcc'
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb '
LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=rtthread.map,-cref,-u,Reset_Handler -T linkscripts//STM32F407IG//link.lds'
CFLAGS += ' -O0 -gdwarf-2 -g'
AFLAGS += ' -gdwarf-2'
CXXFLAGS = CFLAGS
POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'
