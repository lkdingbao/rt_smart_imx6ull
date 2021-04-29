/*
 * OV2640
 *   OV2640 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-28     Lyons        first version
 */

#include <rtconfig.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_OV2640

#include <board.h>
#ifdef RT_USING_LWP
#include <lwp.h>
#include <lwp_user_mm.h>
#endif

#include "__def.h"
#include "realview.h"
#include "drv_i2c.h"
#include "drv_pin.h"
#include "drv_ov2640.h"
#include "bsp_lcdapi.h"
#include "skt.h"

#define DBG_TAG "drv.ov2640"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define _DEVICE_NAME            "ov2640"
#define _BUS_NAME               "i2c2"

/* set this to 1 to enable camera test demo. */
#define CAMERA_DEBUG_EN 1

#define _CAMERA_WIDTH           BSP_LCD_WIDTH
#define _CAMERA_HEIGHT          BSP_LCD_HEIGHT
#define _CAMERA_PIXEL_SIZE      2
#define _CAMERA_FRAME_COUNT     2

#define _BUS_I2C_ADDR           0x30

#define _OV2640_DELAY_US(t)     rt_hw_us_delay(t)
#define _OV2640_DELAY_MS(t)     rt_hw_ms_delay(t)

#include "ov2640cfg.h"

#define _OV2640_INIT_REG_TBL    ov2640_sxga_init_reg_tbl
//ov2640_sxga_init_reg_tbl
//ov2640_svga_init_reg_tbl

#define _OV2640_JPEG_TBL        ov2640_jpeg_reg_tbl
#define _OV2640_RGB565_TBL      ov2640_rgb565_reg_tbl
#define _OV2640_YUV422_TBL      ov2640_yuv422_reg_tbl

//ov2640 register list
#define OV2640_SENSOR_MIDH      0x1C
#define OV2640_SENSOR_MIDL      0x1D
#define OV2640_SENSOR_PIDH      0x0A
#define OV2640_SENSOR_PIDL      0x0B

_internal_rw struct skt_cameradev _s_ov2640 = {
    .name = "ov2640",
    .periph.paddr = REALVIEW_CSI_BASE,
    .irqno = CSI_IRQn,
    .gpio = {
        {IOMUXC_CSI_DATA00_CSI_DATA02,      0U, 0x10B0},
        {IOMUXC_CSI_DATA01_CSI_DATA03,      0U, 0x10B0},
        {IOMUXC_CSI_DATA02_CSI_DATA04,      0U, 0x10B0},
        {IOMUXC_CSI_DATA03_CSI_DATA05,      0U, 0x10B0},
        {IOMUXC_CSI_DATA04_CSI_DATA06,      0U, 0x10B0},
        {IOMUXC_CSI_DATA05_CSI_DATA07,      0U, 0x10B0},
        {IOMUXC_CSI_DATA06_CSI_DATA08,      0U, 0x10B0},
        {IOMUXC_CSI_DATA07_CSI_DATA09,      0U, 0x10B0},
        {IOMUXC_CSI_HSYNC_CSI_HSYNC,        0U, 0x10B0},
        {IOMUXC_CSI_PIXCLK_CSI_PIXCLK,      0U, 0x10B0},
        {IOMUXC_CSI_VSYNC_CSI_VSYNC,        0U, 0x10B0},
    },
    .rst_pin = GET_PIN(1,2),
    .pwdn_pin = GET_PIN(1,4),
    .flag = 0,
};

/* only used for csi operate. not editable! */
_internal_rw csi_resource_t csi_resource;
_internal_rw csi_private_data_t csi_privateData;

_internal_rw camera_receiver_handle_t _s_camera_receiver = {
    .resource = &csi_resource,
    .ops = &csi_ops,
    .privateData = &csi_privateData,
};

_internal_rw camera_config_t _s_camera_config = {
    .pixelFormat = kVIDEO_PixelFormatRGB565,
    .bytesPerPixel = _CAMERA_PIXEL_SIZE,
    .resolution = FSL_VIDEO_RESOLUTION(_CAMERA_WIDTH, _CAMERA_HEIGHT),
    .frameBufferLinePitch_Bytes = _CAMERA_WIDTH * _CAMERA_PIXEL_SIZE,
    .interface = kCAMERA_InterfaceGatedClock,
    .controlFlags = (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge),
    .framePerSec = 30,
};

_internal_rw AT_NONCACHEABLE_SECTION_ALIGN( uint16_t _s_frame_buf[_CAMERA_FRAME_COUNT][_CAMERA_HEIGHT][_CAMERA_WIDTH],
                                        64 );

static void _write_data( rt_device_t i2cdev, rt_uint16_t reg, rt_uint8_t *data, rt_uint16_t len )
{
    rt_uint8_t bus_addr = _BUS_I2C_ADDR;

    rt_device_control(i2cdev, RT_I2C_DEV_CTRL_ADDR, &bus_addr);
    rt_device_write(i2cdev, reg, data, len);
}

static void _read_data( rt_device_t i2cdev, rt_uint16_t reg, rt_uint8_t *data, rt_uint16_t len )
{
    rt_uint8_t bus_addr = _BUS_I2C_ADDR;

    rt_device_control(i2cdev, RT_I2C_DEV_CTRL_ADDR, &bus_addr);
    rt_device_read(i2cdev, reg, data, len);
}

static void _write_one_data( rt_device_t i2cdev, rt_uint16_t reg, rt_uint8_t data )
{
    _write_data(i2cdev, reg, &data, 1);
}

static rt_uint8_t _read_one_data( rt_device_t i2cdev, rt_uint16_t reg )
{
    rt_uint8_t data;

    _read_data(i2cdev, reg, &data, 1);
    return data;
}

static rt_uint16_t _get_verison( rt_device_t i2cdev )
{
    rt_uint8_t pid[2];
    rt_uint16_t data;

    pid[0] = _read_one_data(i2cdev, OV2640_SENSOR_PIDH);
    pid[1] = _read_one_data(i2cdev, OV2640_SENSOR_PIDL);

    data = (pid[0] << 8) | pid[1];

    LOG_D("product id: %04x", data);

    return data;
}

static void _csi_int_isr( int irqno, void* parameter )
{
    extern void CSI_DriverIRQHandler(void);
    CSI_DriverIRQHandler();
}

static void _csi_clock_init( void )
{
    /* CSI MCLK select 24M. */
    CLOCK_SetMux(kCLOCK_CsiMux, 0);
    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);

    CLOCK_EnableClock(kCLOCK_CsiMclk);
}

static void _csi_gpio_init( void )
{
    gpio_pin_config_t config;

    for (int i=0; i<CAMERA_CONTROL_PIN_NUM; i++)
    {
        gpio_set_iomux(&_s_ov2640.gpio[i]);
    }

    config.direction = kGPIO_DigitalOutput;
    config.interruptMode = kGPIO_NoIntmode;
    config.outputLogic = PIN_HIGH; //keep high default

    gpio_config(_s_ov2640.rst_pin, &config);
    gpio_config(_s_ov2640.pwdn_pin, &config);
}

static void _csi_device_init( void )
{
    rt_hw_interrupt_install(_s_ov2640.irqno, _csi_int_isr, RT_NULL, _s_ov2640.name);
    rt_hw_interrupt_umask(_s_ov2640.irqno);

    CAMERA_RECEIVER_Init(&_s_camera_receiver, &_s_camera_config, NULL, NULL);

    for (int i = 0; i < _CAMERA_FRAME_COUNT; i++)
    {
        uint32_t paddr = mem_map_v2p((uint32_t)(_s_frame_buf[i]));
        CAMERA_RECEIVER_SubmitEmptyBuffer(&_s_camera_receiver, paddr);
    }

    CAMERA_RECEIVER_Start(&_s_camera_receiver);

    _s_ov2640.flag = 1;
}

static void _set_output_mode( rt_device_t i2cdev, eCameraOutputMode mode )
{
    int i;

    switch (mode)
    {
        case eOutputMode_Jpeg:
            for (i=0; i<GET_ARRAY_NUM(_OV2640_JPEG_TBL); i++) {
                _write_one_data(i2cdev, _OV2640_JPEG_TBL[i][0], _OV2640_JPEG_TBL[i][1]);
            }
            break;

        case eOutputMode_Rgb565:
            for (i=0; i<GET_ARRAY_NUM(_OV2640_RGB565_TBL); i++) {
                _write_one_data(i2cdev, _OV2640_RGB565_TBL[i][0], _OV2640_RGB565_TBL[i][1]);
            }
            break;

        case eOutputMode_Yuv422:
            for (i=0; i<GET_ARRAY_NUM(_OV2640_YUV422_TBL); i++) {
                _write_one_data(i2cdev, _OV2640_YUV422_TBL[i][0], _OV2640_YUV422_TBL[i][1]);
            }
            break;

        default:
            break;
    }
}

static void _set_light_mode( rt_device_t i2cdev, eCameraLightMode mode )
{
    uint8_t regVal[3];

    if (mode >= eLightMode_MaxNum) {
        return;
    }

    switch(mode)
    { 
        case eLightMode_Auto:
            regVal[0] = 0x5E;
            regVal[1] = 0x41;
            regVal[2] = 0x54;
            break;

        case eLightMode_Cloudy:
            regVal[0] = 0x65;
            regVal[1] = 0x41;
            regVal[2] = 0x4F;
            break;

        case eLightMode_Office:
            regVal[0] = 0x52;
            regVal[1] = 0x41;
            regVal[2] = 0x66;
            break;

        case eLightMode_Home:
            regVal[0] = 0x42;
            regVal[1] = 0x3F;
            regVal[2] = 0x71;
            break;

        default:
            break;
    }

    _write_one_data(i2cdev, 0xFF, 0x00);
    if (eLightMode_Auto == mode)
    {
        _write_one_data(i2cdev, 0xC7, 0x10); //AWB ON
    } else {
        _write_one_data(i2cdev, 0xC7, 0x40); //AWB OFF
    }
    _write_one_data(i2cdev, 0xCC, regVal[0]);
    _write_one_data(i2cdev, 0xCD, regVal[1]);
    _write_one_data(i2cdev, 0xCE, regVal[2]);
}

static void _set_output_image_size( rt_device_t i2cdev, uint16_t width, uint16_t height )
{
    uint16_t outh, outw;
    uint8_t dummy;

    if ( (width % 4) || (height % 4) ) {
        return;
    }

    outw = width / 4;
    outh = height / 4;

    _write_one_data(i2cdev, 0xFF, 0x00);
    _write_one_data(i2cdev, 0xE0, 0x04);

    _write_one_data(i2cdev, 0x5A, outw&0xFF);
    _write_one_data(i2cdev, 0x5B, outh&0xFF);

    dummy  = (outw >> 8) & 0x03;
    dummy |= (outh >> 6) & 0x04;

    _write_one_data(i2cdev, 0x5C, dummy);
    _write_one_data(i2cdev, 0xE0, 0x00);
}

static void _ov2640_device_init( rt_device_t i2cdev )
{
    RT_ASSERT(RT_NULL != i2cdev);

    gpio_output(_s_ov2640.pwdn_pin, 0);
    _OV2640_DELAY_MS(5);
    gpio_output(_s_ov2640.rst_pin, 0);
    _OV2640_DELAY_MS(5);
    gpio_output(_s_ov2640.rst_pin, 1);
    _OV2640_DELAY_MS(5);

    _write_one_data(i2cdev, 0xFF, 0x01);
    _write_one_data(i2cdev, 0x12, 0x80);
    _OV2640_DELAY_MS(10);

    _get_verison(i2cdev);

    for (int i=0; i<GET_ARRAY_NUM(_OV2640_INIT_REG_TBL); i++) {
        _write_one_data(i2cdev, _OV2640_INIT_REG_TBL[i][0], _OV2640_INIT_REG_TBL[i][1]);
    }

    _set_output_mode(i2cdev, eOutputMode_Rgb565);
    _set_light_mode(i2cdev, eLightMode_Auto);
    _set_output_image_size(i2cdev, _CAMERA_WIDTH, _CAMERA_HEIGHT);
}

static rt_err_t _ov2640_ops_open( rt_device_t dev,
                                  rt_uint16_t oflag )
{
    struct rt_device *i2c_bus = RT_NULL;

    RT_ASSERT(RT_NULL != dev);

    if (!(dev->flag & RT_DEVICE_FLAG_RDWR))
    {
        LOG_D("only support wr/rd option!");
        return -RT_ERROR;
    }

    i2c_bus = rt_device_find(_BUS_NAME);
    if (RT_NULL == i2c_bus)
    {
        LOG_D("not find bus device[%s]", i2c_bus);
        return -RT_EIO;
    }

    rt_device_open(i2c_bus, RT_DEVICE_OFLAG_RDWR);
    dev->user_data = i2c_bus;

    _csi_gpio_init();

    _ov2640_device_init(dev->user_data);

    _csi_clock_init();
    _csi_device_init();

    return RT_EOK;
}

static rt_err_t _ov2640_ops_close( rt_device_t dev )
{
    RT_ASSERT(RT_NULL != dev);

    rt_device_close(dev->user_data);
    dev->user_data = RT_NULL;

    return RT_EOK;
}

static rt_size_t _ov2640_ops_read( rt_device_t dev,
                                   rt_off_t pos,
                                   void *buffer,
                                   rt_size_t size )
{
    return RT_EOK;
}

static rt_err_t _ov2640_ops_control( rt_device_t dev,
                                     int cmd,
                                     void *args )
{
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
_internal_ro struct rt_device_ops _k_ov2640_ops =
{
    RT_NULL,                /* init */
    _ov2640_ops_open,       /* open */
    _ov2640_ops_close,      /* close */
    _ov2640_ops_read,       /* read */
    RT_NULL,                /* write */
    _ov2640_ops_control,    /* control */
};
#endif

int rt_hw_ov2640_init(void)
{
    struct rt_device *device = RT_NULL;

    device = &_s_ov2640.parent;

    device->type    = RT_Device_Class_Sensor;

#ifdef RT_USING_DEVICE_OPS
    device->ops     = &_k_ov2640_ops;
#else
    device->init    = RT_NULL;
    device->open    = _ov2640_ops_open;
    device->close   = _ov2640_ops_close;
    device->read    = _ov2640_ops_read;
    device->write   = RT_NULL;
    device->control = _ov2640_ops_control;
#endif

    device->user_data = RT_NULL;

    _s_ov2640.periph.vaddr = platform_get_periph_vaddr(_s_ov2640.periph.paddr);
    csi_resource.csiBase = (CSI_Type*)_s_ov2640.periph.vaddr;

    _s_ov2640.flag = 0; //wait for enet init!

    rt_device_register(device, _s_ov2640.name, RT_DEVICE_FLAG_RDWR);

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_ov2640_init);

#if defined(CAMERA_DEBUG_EN) && (CAMERA_DEBUG_EN)
int camera(int argc, char **argv)
{
    rt_device_t ovdev = RT_NULL;
    uint32_t *tgtAddr = RT_NULL;
    uint8_t *srcAddr = RT_NULL;

    ovdev = rt_device_find("ov2640");
    if (RT_NULL == ovdev)
    {
        LOG_RAW("not find device [%s].\n", "ov2640");
        return -1;
    }

    rt_device_open(ovdev, RT_DEVICE_FLAG_RDWR);

    tgtAddr = (uint32_t*)_g_lcd_info.fb_virt;

    LOG_RAW("wait and display an image from camera.\n");
    LOG_RAW("you can quit only by hardware reset!\n");

    while (1)
    {
        uint32_t activeFrameAddr = 0;
        while ( kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&_s_camera_receiver, &activeFrameAddr) );

        srcAddr = (uint8_t*)mem_map_p2v(activeFrameAddr);

        for (uint32_t i=0; i<(_CAMERA_HEIGHT*_CAMERA_WIDTH); i++)
        {
            uint16_t rgb565 = (srcAddr[2*i+1]<<8) | srcAddr[2*i+0];

            lcd_color32_t rgb24;
            rgb24.ch.red = (rgb565 & 0xF800) >> 8;
            rgb24.ch.green = (rgb565 & 0x07E0) >> 3;
            rgb24.ch.blue = (rgb565 & 0x001F) << 3;
            rgb24.ch.alpha = 0;

            tgtAddr[i] = rgb24.full;
        }

        CAMERA_RECEIVER_SubmitEmptyBuffer(&_s_camera_receiver, mem_map_v2p(activeFrameAddr));
    }

    rt_device_close(ovdev);

    return 0;
}
MSH_CMD_EXPORT_ALIAS(camera, camera, <usr> camera capture test);
#endif //#if defined(CAMERA_DEBUG_EN) && (CAMERA_DEBUG_EN)

#endif //#ifdef RT_USING_OV2640

