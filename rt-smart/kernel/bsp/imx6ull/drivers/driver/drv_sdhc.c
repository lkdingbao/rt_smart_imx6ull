/*
 * SD-Card
 *   SD-Card driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_SDHC

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_pin.h"
#include "drv_sdhc.h"
#include "skt.h"

#define DBG_TAG "drv.sdhc"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define SDCARD_SHOW_CARD_INFO 0

/*
 * There are 512M Bytes reserved for boot image.
 * Modify these parameter manually.
 */
#define SDCARD_PART1_START_SECTOR 8192 //no fs!
#define SDCARD_PART2_START_SECTOR 1056768 //use fs.

_internal_rw struct skt_sddev _s_sdcard_device = {
    .name = "sd0",
    .periph.paddr = BOARD_SD_HOST_BASEADDR,
    .irqno = BOARD_SD_HOST_IRQ, //not use irq yet!
    .gpio = {
        {IOMUXC_SD1_CLK_USDHC1_CLK,         0U, 0x17049},
        {IOMUXC_SD1_CMD_USDHC1_CMD,         0U, 0x17089},
        
        {IOMUXC_SD1_DATA0_USDHC1_DATA0,     0U, 0x17089},
        {IOMUXC_SD1_DATA1_USDHC1_DATA1,     0U, 0x17089},
        {IOMUXC_SD1_DATA2_USDHC1_DATA2,     0U, 0x17089},
        {IOMUXC_SD1_DATA3_USDHC1_DATA3,     0U, 0x17089},
    },
    .flag = 0,
};

_internal_rw sd_card_t _s_sd;

#if defined(SDCARD_SHOW_CARD_INFO) && (SDCARD_SHOW_CARD_INFO)
static void _sdcard_show_info( sd_card_t *card )
{
    LOG_RAW("------------------------------\n");
    LOG_RAW("sd card infomation\n");
    LOG_RAW("------------------------------\n");

    LOG_RAW("Version:\n");
    LOG_RAW("  version: %d\n", card->version);

    LOG_RAW("Bus frequency:\n");
    LOG_RAW("  freq: %d Hz\n", card->busClock_Hz);

    LOG_RAW("Capacity information:\n");

    LOG_RAW("  sector size:  %d Bytes\n", card->blockSize);
    LOG_RAW("  sector count: %d\n", card->blockCount);
    LOG_RAW("  block size:   %d\n", card->csd.eraseSectorSize);

    LOG_RAW("Working condition:\n");

    LOG_RAW("  voltage: ");
    if (card->operationVoltage == kCARD_OperationVoltage330V) {
        LOG_RAW("3.3V");
    } else if (card->operationVoltage == kCARD_OperationVoltage180V) {
        LOG_RAW("1.8V");
    }
    LOG_RAW("\n");

    LOG_RAW("  timing mode: ");
    if (card->currentTiming == kSD_TimingSDR12DefaultMode)
    {
        if (card->operationVoltage == kCARD_OperationVoltage330V) {
            LOG_RAW("Default");
        } else if (card->operationVoltage == kCARD_OperationVoltage180V) {
            LOG_RAW("SDR12");
        }
    } else if (card->currentTiming == kSD_TimingSDR25HighSpeedMode) {
        if (card->operationVoltage == kCARD_OperationVoltage180V) {
            LOG_RAW("SDR25");
        } else {
            LOG_RAW("High Speed");
        }
    } else if (card->currentTiming == kSD_TimingSDR50Mode) {
        LOG_RAW("SDR50");
    } else if (card->currentTiming == kSD_TimingSDR104Mode) {
        LOG_RAW("SDR104");
    } else if (card->currentTiming == kSD_TimingDDR50Mode) {
        LOG_RAW("DDR50");
    }
    LOG_RAW("\n");
}
#endif //#if defined(SDCARD_SHOW_CARD_INFO) && (SDCARD_SHOW_CARD_INFO)

static rt_err_t sdcard_init_clock( struct skt_sddev *sddev )
{
    RT_ASSERT(RT_NULL != sddev);

    if (REALVIEW_USDHC1_BASE == sddev->periph.paddr)
    {
        CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
        CLOCK_EnableClock(kCLOCK_Usdhc1);
    } else if (REALVIEW_USDHC2_BASE == sddev->periph.paddr) {
        CLOCK_SetDiv(kCLOCK_Usdhc2Div, 0U);
        CLOCK_EnableClock(kCLOCK_Usdhc2);
    }

    return RT_EOK;
}

static void _sdcard_gpio_init( struct skt_sddev *sddev )
{
    RT_ASSERT(RT_NULL != sddev);

    for (int i=0; i<SDCARD_CONTROL_PIN_NUM; i++)
    {
        gpio_set_iomux(&sddev->gpio[i]);
    }
}

static rt_err_t _sdcard_device_init( struct skt_sddev *sddev )
{
    sd_card_t *card = RT_NULL;

    RT_ASSERT(RT_NULL != sddev);
    RT_ASSERT(0 != sddev->periph.vaddr);

    sdcard_init_clock(sddev);

    card = &_s_sd;

    card->host.base = (USDHC_Type*)(sddev->periph.vaddr);
    if (REALVIEW_USDHC1_BASE == sddev->periph.paddr)
    {
        card->host.sourceClock_Hz = BOARD_USDHC1_CLK_FREQ;
    } else if (REALVIEW_USDHC2_BASE == sddev->periph.paddr) {
        card->host.sourceClock_Hz = BOARD_USDHC2_CLK_FREQ;
    }

    if (kStatus_Success != SD_Init(card))
    {
        SD_Deinit(card);
        rt_memset(card, 0U, sizeof(_s_sd));

        LOG_W("sd card init failed.");
        return -RT_ERROR;
    }

    sddev->flag = 1;

    LOG_D("sd card init finished.");
    LOG_D("sd card has %d sector, sector size %d Bytes.", card->blockCount, card->blockSize);

#if defined(SDCARD_SHOW_CARD_INFO) && (SDCARD_SHOW_CARD_INFO)
    _sdcard_show_info(card);
#endif

    return RT_EOK;
}

static rt_err_t _sdcard_ops_init( rt_device_t dev )
{
    return RT_EOK;
}

static rt_err_t _sdcard_ops_open( rt_device_t dev,
                                  rt_uint16_t oflag )
{
    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_W("only support rd/wr option!");
        return -RT_ERROR;
    }

    dev->ref_count++;

    return RT_EOK;
}

static rt_err_t _sdcard_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    dev->ref_count = (0 == dev->ref_count) ? 0 : (dev->ref_count - 1);

    return RT_EOK;
}

static rt_size_t _sdcard_ops_read( rt_device_t dev, 
                                   rt_off_t pos, 
                                   void *buffer, 
                                   rt_size_t size )
{
    RT_ASSERT(RT_NULL != dev);

    if (0 == dev->ref_count)
    {
        return 0;
    }

    if (kStatus_Success != SD_ReadBlocks(&_s_sd, buffer, SDCARD_PART2_START_SECTOR+pos, size))
    {
        return 0;
    }

    return size;
}

static rt_size_t _sdcard_ops_write( rt_device_t dev, 
                                    rt_off_t pos, 
                                    const void *buffer, 
                                    rt_size_t size )
{
    RT_ASSERT(RT_NULL != dev);

    if (0 == dev->ref_count)
    {
        return 0;
    }

    if (kStatus_Success != SD_WriteBlocks(&_s_sd, buffer, SDCARD_PART2_START_SECTOR+pos, size))
    {
        return 0;
    }

    return size;
}

static rt_err_t _sdcard_ops_control( rt_device_t dev, 
                                     int cmd,
                                     void *args )
{
    struct rt_device_blk_geometry *pgeometry = RT_NULL;
    rt_err_t result;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_BLK_GETGEOME:
            if (args)
            {
                pgeometry = (struct rt_device_blk_geometry*)args;

                pgeometry->sector_count = _s_sd.blockCount;
                pgeometry->bytes_per_sector = _s_sd.blockSize;
                pgeometry->block_size = _s_sd.csd.eraseSectorSize;

                result = RT_EOK;
            }
            else
            {
                result = -RT_EINVAL;
            }
            break;

        case RT_DEVICE_CTRL_BLK_SYNC:
            result = RT_EOK;
            break;

        default :
            result = -RT_EINVAL;
            break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_sdcard_ops =
{
    _sdcard_ops_init,       /* init */
    _sdcard_ops_open,       /* open */
    _sdcard_ops_close,      /* close */
    _sdcard_ops_read,       /* read */
    _sdcard_ops_write,      /* write */
    _sdcard_ops_control,    /* control */
};
#endif

int rt_hw_sdcard_init(void)
{
    struct rt_device *device = RT_NULL;

    device = &_s_sdcard_device.parent;

    device->type    = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_sdcard_ops;
#else
    device->init    = _sdcard_ops_init;
    device->open    = _sdcard_ops_open;
    device->close   = _sdcard_ops_close;
    device->read    = _sdcard_ops_read;
    device->write   = _sdcard_ops_write;
    device->control = _sdcard_ops_control;
#endif
    device->user_data = RT_NULL;

    rt_device_register(device, _s_sdcard_device.name, RT_DEVICE_FLAG_RDWR);

    _s_sdcard_device.periph.vaddr = platform_get_periph_vaddr(_s_sdcard_device.periph.paddr);

    _s_sdcard_device.flag = 0; //wait for sdhc init!

    _sdcard_gpio_init(&_s_sdcard_device);
    _sdcard_device_init(&_s_sdcard_device);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_sdcard_init);

#endif //#ifdef RT_USING_SDHC

