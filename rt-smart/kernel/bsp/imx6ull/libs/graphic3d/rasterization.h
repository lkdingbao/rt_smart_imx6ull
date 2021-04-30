/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-03     Lyons        first version
 */

#ifndef __RASTERIZATION_H__
#define __RASTERIZATION_H__

#ifdef cplusplus
extern "C" {
#endif

#include <rtthread.h>

#ifdef RT_USING_GRAPHIC3D

//-----------------------------------------------
//      宏定义
//-----------------------------------------------
//定义顶点数据大小（字节）
#define VERTEX_DATA_SIZE_2F         (sizeof(float)*2)
#define VERTEX_DATA_SIZE_3F         (sizeof(float)*3)

//-----------------------------------------------
//      结构体，共用体，枚举
//-----------------------------------------------
typedef struct
{
    float       angle[3];

    float       eye_fov;
    float       aspect_ratio;
    float       znear;
    float       zfar;

    float       eye_point[3];
    float       target_point[3];
    float       up_vector[3];

    uint16_t    width;
    uint16_t    height;

}TransferMatrixInfo_t;

//-----------------------------------------------
//      变量声明
//-----------------------------------------------
extern TransferMatrixInfo_t g_TransferMatrixInfo;

//-----------------------------------------------
//      函数声明
//-----------------------------------------------
void create_transfer_matrix( int Type, TransferMatrixInfo_t *pInfo, float *pcenter, float *pOut );
void get_object_center( const float *ppoint, int num, float *pOut );

void transfer_triangle( const float *ppoint, float *tmatrix, float *pOut );
void transfer_tripyramid( const float *ppoint, float *tmatrix, float *pOut );

#endif //#ifdef RT_USING_GRAPHIC3D

#ifdef cplusplus
}
#endif

#endif //#ifndef __RASTERIZATION_H__
