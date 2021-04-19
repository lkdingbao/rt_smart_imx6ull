import os

# toolchains options
ARCH        = 'arm'
CPU         = 'cortex-a'
CROSS_TOOL  = 'gcc'
PLATFORM    = 'gcc'
EXEC_PATH   = os.getenv('RTT_EXEC_PATH') or '/usr/bin'
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

MKIMAGE_PATH = '../scripts'
MKIMAGE_CFG_FILE = MKIMAGE_PATH + '/imximage.cfg.cfgtmp'

POST_ACTION = OBJCPY + ' -O binary $TARGET output.bin \n' + \
              SIZE + ' $TARGET \n' + \
              'mkimage.exe -n ' + MKIMAGE_CFG_FILE + ' -T imximage -e 0x80010000 -d output.bin output.imx \n'