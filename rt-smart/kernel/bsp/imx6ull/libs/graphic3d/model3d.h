/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-17     Lyons        first version
 */

#ifndef __OBJECT_DEF_H__
#define __OBJECT_DEF_H__

#ifdef cplusplus
extern "C" {
#endif

#include <rtthread.h>

#ifdef RT_USING_GRAPHIC3D

#include "__def.h"

//-----------------------------------------------
//      宏定义
//-----------------------------------------------
#define OBJECT_MODEL_NONE           0
#define OBJECT_MODEL_ROCK           1
#define OBJECT_MODEL_COWS           2

#define OBJECT_MODEL                BSP_GRAPHIC3D_OBJECT_MODEL

#if( OBJECT_MODEL == OBJECT_MODEL_ROCK )
#define _MODEL_FACE_NUM     192
#elif( OBJECT_MODEL == OBJECT_MODEL_COWS )
#define _MODEL_FACE_NUM     5856
#endif

//-----------------------------------------------
//      结构体，共用体，枚举
//-----------------------------------------------

//-----------------------------------------------
//      变量声明
//-----------------------------------------------
#if( OBJECT_MODEL == OBJECT_MODEL_ROCK )
const float _k_model3d_v[98][3]; //vertex
const float _k_model3d_vn[98][3]; //vector normal
const float _k_model3d_vt[165][2]; //texture coordinate
const uint16_t _k_model3d_f[_MODEL_FACE_NUM*3][3]; //face index
#elif( OBJECT_MODEL == OBJECT_MODEL_COWS )
const float _k_model3d_v[3225][3]; //vertex
const float _k_model3d_vn[3225][3]; //vector normal
const float _k_model3d_vt[3225][2]; //texture coordinate
const uint16_t _k_model3d_f[_MODEL_FACE_NUM*3][3]; //face index
#endif

//-----------------------------------------------
//      函数声明
//-----------------------------------------------

#endif //#ifdef RT_USING_GRAPHIC3D

#ifdef cplusplus
}
#endif

#endif //#ifndef __OBJECT_DEF_H__
