/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-03     Lyons        first version
 */

#ifndef __TASK_GRAPHIC_3D_H__
#define __TASK_GRAPHIC_3D_H__

#ifdef cplusplus
extern "C" {
#endif

#include <rtthread.h>

#ifdef RT_USING_GRAPHIC3D

#include "rasterization.h"

//-----------------------------------------------
//      宏定义
//-----------------------------------------------

//-----------------------------------------------
//      结构体，共用体，枚举
//-----------------------------------------------
typedef struct
{
    uint32_t edge_color;
    uint32_t face_color;
    uint32_t *vertex_color;

}TriangleInfo_t;

typedef struct
{
    uint32_t edge_color;
    uint32_t face_color;
    uint32_t *vertex_color;

}TripyramidInfo_t;

typedef struct
{
    int         fd;

    uint32_t    Width;
    uint32_t    Height;

    uint32_t    ZipMode;

    uint32_t    Offset;

} BMPHeadInfo_t;

//-----------------------------------------------
//      变量声明
//-----------------------------------------------
extern TriangleInfo_t _g_TriangleInfo;
extern TripyramidInfo_t _g_TripyramidInfo;
extern BMPHeadInfo_t _g_BMPHeadInfo;

//-----------------------------------------------
//      函数声明
//-----------------------------------------------
void G3D_InitDeepBuffer( void );
void G3D_GrawAxis( float *ppoint, uint16_t num, uint32_t *pcolor );
void G3D_DrawTriangle( uint8_t type, float *ppoint, float *pvn, float *pvt, uint16_t num );
void G3D_DrawTripyramid( uint8_t type, float *ppoint, uint16_t num );

void G3D_GetBMPInfo( const char *ppath, BMPHeadInfo_t *pInfo );
uint32_t G3D_GetBMPPointColor( float x, float y, BMPHeadInfo_t *pInfo );

#endif //#ifdef RT_USING_GRAPHIC3D

#ifdef cplusplus
}
#endif

#endif //#ifndef __TASK_GRAPHIC_3D_H__
