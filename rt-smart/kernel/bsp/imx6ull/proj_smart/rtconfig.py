import os

import uuid
def get_mac_address():
    mac=uuid.UUID(int = uuid.getnode()).hex[-12:]
    return "#define AUTOMAC".join([str(int(e/2) + 1) + '  0x' + mac[e:e+2] + '\n' for e in range(5,11,2)])

header = '''
#ifndef __MAC_AUTO_GENERATE_H__
#define __MAC_AUTO_GENERATE_H__

/* Automatically generated file; DO NOT EDIT. */
/* mac configure file for RT-Thread qemu */

#define AUTOMAC0  0x52
#define AUTOMAC1  0x54
#define AUTOMAC2  0x00
#define AUTOMAC'''

end = '''
#endif
'''

automac_h_fn = os.path.join(os.path.dirname(__file__), '../drivers', 'automac.h')
with open(automac_h_fn, 'w') as f:
    f.write(header + get_mac_address() + end)

# toolchains options
ARCH        = 'arm'
CPU         = 'cortex-a'
CROSS_TOOL  = 'gcc'
PLATFORM    = 'gcc'
EXEC_PATH   = os.getenv('RTT_EXEC_PATH') or '/usr/bin'
TOOL_PATH   = os.getenv('RTT_TOOL_PATH') or '.'
BUILD       = 'debug'

PROJ_TYPE   = os.getenv('RTT_PROJ') or 'rt-thread'

if   PROJ_TYPE == 'rt-smart':
    LINK_FILE = 'link_rtsmart.lds'
elif PROJ_TYPE == 'rt-thread':
    LINK_FILE = 'link_rtthread.lds'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX  = os.getenv('RTT_CC_PREFIX') or 'arm-none-eabi-'
    CC      = PREFIX + 'gcc'
    CXX     = PREFIX + 'g++'
    AS      = PREFIX + 'gcc'
    AR      = PREFIX + 'ar'
    LINK    = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE    = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY  = PREFIX + 'objcopy'
    STRIP   = PREFIX + 'strip'

    if   PROJ_TYPE == 'rt-smart':
        CFPFLAGS = ' -msoft-float'
        AFPFLAGS = ' -mfloat-abi=softfp -mfpu=neon'
        DEVICE   = ' -march=armv7-a -mtune=cortex-a7 -ftree-vectorize -ffast-math -funwind-tables -fno-strict-aliasing'

        CXXFLAGS= DEVICE + CFPFLAGS + ' -Wall'
        CFLAGS  = DEVICE + CFPFLAGS + ' -Wall -std=gnu99'
        AFLAGS  = ' -c' + AFPFLAGS + ' -x assembler-with-cpp'
        LFLAGS  = DEVICE + ' -Wl,--gc-sections,-Map=output.map,-cref,-u,system_vectors -T ' + LINK_FILE + ' -lsupc++ -lgcc'
    elif PROJ_TYPE == 'rt-thread':
        CFPFLAGS = ' -msoft-float'
        AFPFLAGS = ' -mfloat-abi=softfp -mfpu=vfpv3-d16'
        DEVICE   = ' -march=armv7-a -mtune=cortex-a7 -ftree-vectorize -ffast-math'

        CXXFLAGS= DEVICE + CFPFLAGS + ' -Wall'
        CFLAGS  = DEVICE + CFPFLAGS + ' -Wall -std=gnu99'
        AFLAGS  = ' -c' + AFPFLAGS + ' -x assembler-with-cpp -D__ASSEMBLY__ -I.'
        LFLAGS  = DEVICE + ' -Wl,--gc-sections,-Map=output.map,-cref,-u,system_vectors -T ' + LINK_FILE + ' -lsupc++ -lgcc'

    CPATH   = ''
    LPATH   = ''

    if BUILD == 'debug':
        CFLAGS   += ' -O0 -gdwarf-2'
        CXXFLAGS += ' -O0 -gdwarf-2'
        AFLAGS   += ' -gdwarf-2'
    else:
        CFLAGS   += ' -Os'
        CXXFLAGS += ' -Os'

    CXXFLAGS += ' -Woverloaded-virtual -fno-exceptions -fno-rtti'

DUMP_ACTION = OBJDUMP + ' -D -S $TARGET > output.DEBUG \n'

MKIMAGE_PATH = TOOL_PATH
MKIMAGE_CFG_FILE = MKIMAGE_PATH + '/imximage.cfg.cfgtmp'

POST_ACTION = OBJCPY + ' -O binary $TARGET output.bin \n' + \
              SIZE + ' $TARGET \n' + \
              'mkimage.exe -n ' + MKIMAGE_CFG_FILE + ' -T imximage -e 0x80010000 -d output.bin output.imx \n'