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

#ifdef RT_USING_SDCARD

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_pin.h"
#include "drv_sd.h"
#include "skt.h"

#define DBG_TAG "SDCARD"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define SDCARD_SHOW_CARD_INFO 0

_internal_rw struct skt_sddev _s_sdcard_device = {
    .name = "sd",
    .periph.paddr = BOARD_SD_HOST_BASEADDR,
    .irqno = BOARD_SD_HOST_IRQ,
    .gpio = {
        {IOMUXC_SD1_CLK_USDHC1_CLK,         0U, 0x17049},
        {IOMUXC_SD1_CMD_USDHC1_CMD,         0U, 0x17089},
        
        {IOMUXC_SD1_DATA0_USDHC1_DATA0,     0U, 0x17089},
        {IOMUXC_SD1_DATA1_USDHC1_DATA1,     0U, 0x17089},
        {IOMUXC_SD1_DATA2_USDHC1_DATA2,     0U, 0x17089},
        {IOMUXC_SD1_DATA3_USDHC1_DATA3,     0U, 0x17089},
        
        {IOMUXC_UART1_RTS_B_GPIO1_IO19,     0U, 0x18000},
    },
    .cd_pin = GET_PIN(0,19), //detect not used yet.
    .flag = 0,
};

_internal_rw sd_card_t _s_sd;

#if defined(SDCARD_SHOW_CARD_INFO) && (SDCARD_SHOW_CARD_INFO)
static void _sdcard_show_info( sd_card_t *card )
{
    LOG_RAW("sd card infomation\n");
    LOG_RAW("------------------------------\n");

    LOG_RAW("Card size: %d * %d bytes\n", card->blockCount, card->blockSize);
    LOG_RAW("Working condition:\n");

    if (card->operationVoltage == kCARD_OperationVoltage330V) {
        LOG_RAW("  Voltage : 3.3V\n");
    } else if (card->operationVoltage == kCARD_OperationVoltage180V) {
        LOG_RAW("  Voltage : 1.8V\n");
    }

    if (card->currentTiming == kSD_TimingSDR12DefaultMode)
    {
        if (card->operationVoltage == kCARD_OperationVoltage330V) {
            LOG_RAW("  Timing mode: Default mode\n");
        } else if (card->operationVoltage == kCARD_OperationVoltage180V) {
            LOG_RAW("  Timing mode: SDR12 mode\n");
        }
    } else if (card->currentTiming == kSD_TimingSDR25HighSpeedMode) {
        if (card->operationVoltage == kCARD_OperationVoltage180V) {
            LOG_RAW("  Timing mode: SDR25\n");
        } else {
            LOG_RAW("  Timing mode: High Speed\n");
        }
    } else if (card->currentTiming == kSD_TimingSDR50Mode) {
        LOG_RAW("  Timing mode: SDR50\n");
    } else if (card->currentTiming == kSD_TimingSDR104Mode) {
        LOG_RAW("  Timing mode: SDR104\n");
    } else if (card->currentTiming == kSD_TimingDDR50Mode) {
        LOG_RAW("  Timing mode: DDR50\n");
    }

    LOG_RAW("  Freq : %d Hz\n", card->busClock_Hz);
    LOG_RAW("------------------------------\n");
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

    if (SD_Init(card))
    {
        LOG_D("sd card init failed.");
        return -RT_ERROR;
    }

    LOG_D("sd card init finished.");
    LOG_D("sd card has %d blk, blk size %d B.",  card->blockCount, card->blockSize);

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
    return RT_EOK;
}

static rt_err_t _sdcard_ops_close( rt_device_t dev )
{
    return RT_EOK;
}

static rt_size_t _sdcard_ops_read( rt_device_t dev, 
                                   rt_off_t pos, 
                                   void *buffer, 
                                   rt_size_t size )
{
    return 0;
}

static rt_size_t _sdcard_ops_write( rt_device_t dev, 
                                    rt_off_t pos, 
                                    const void *buffer, 
                                    rt_size_t size )
{
    return 0;
}

static rt_err_t _sdcard_ops_control( rt_device_t dev, 
                                     int cmd,
                                     void *args )
{
    struct skt_sddev *sddev = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    sddev = (struct skt_sddev*)(dev);

    switch (cmd)
    {
        default :
            break;
    }

    UNUSED(sddev);
    return RT_EOK;
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
    _s_sdcard_device.periph.vaddr = platform_get_periph_vaddr(_s_sdcard_device.periph.paddr);

    _sdcard_gpio_init(&_s_sdcard_device);
    _sdcard_device_init(&_s_sdcard_device);

    UNUSED(_k_sdcard_ops);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_sdcard_init);

#endif //#ifdef RT_USING_SDCARD

