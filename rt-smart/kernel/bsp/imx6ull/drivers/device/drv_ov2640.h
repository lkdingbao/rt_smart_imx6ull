/*
 * OV2640
 *   OV2640 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-28     Lyons        first version
 */

#ifndef __DRV_OV2640_H__
#define __DRV_OV2640_H__

#include "skt.h"

#ifdef RT_USING_OV2640

/* Fixed value, not edit! */
#define CAMERA_CONTROL_PIN_NUM      (11)

struct skt_cameradev
{
    struct rt_device                parent;

    const char                      *name;
    struct skt_periph               periph;
    rt_uint32_t                     irqno;

    struct skt_gpio                 gpio[CAMERA_CONTROL_PIN_NUM];
    uint32_t                        rst_pin;
    uint32_t                        pwdn_pin;

    rt_uint32_t                     flag;
};

typedef enum
{
    eLightMode_Auto                 = 0,
    eLight_Sunny,
    eLightMode_Cloudy,
    eLightMode_Office,
    eLightMode_Home,

    eLightMode_MaxNum,
}eCameraLightMode;

typedef enum
{
    eOutputMode_Jpeg                = 0,
    eOutputMode_Rgb565,
    eOutputMode_Yuv422,

    eOutputMode_MaxNum,
}eCameraOutputMode;

#endif //#ifdef RT_USING_OV2640
#endif //#ifndef __DRV_OV2640_H__

