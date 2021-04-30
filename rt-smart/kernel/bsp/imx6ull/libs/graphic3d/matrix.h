/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-03     Lyons        first version
 */

#ifndef __MATRIX_LIB_H__
#define __MATRIX_LIB_H__

#ifdef cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <math.h>

//-----------------------------------------------
//      宏定义
//-----------------------------------------------
#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

//-----------------------------------------------
//      结构体，共用体，枚举
//-----------------------------------------------

//-----------------------------------------------
//      变量声明
//-----------------------------------------------

//-----------------------------------------------
//      函数声明
//-----------------------------------------------
void calc_vector_normalized( uint32_t sz, float *pu, float *pOut );
void calc_vector_maxlimit( uint32_t sz, float *pu, float maxv, float *pOut );
void calc_vector_minlimit( uint32_t sz, float *pu, float maxv, float *pOut );

void calc_vector_minus( uint32_t sz, float *pu, float *pv, float *pOut );
void calc_vector_plus( uint32_t sz, float *pu, float *pv, float *pOut );

void calc_vector_cwisemulti( uint32_t sz, float *pu, float *pv, float *pOut );
void calc_vector_nummulti( uint32_t sz, float *pu, float n, float *pOut );
float calc_vector_multi( uint32_t sz, float *pu, float *pv );

void calc_vector_cross( uint32_t sz, float *pu, float *pv, float *pOut );

void calc_matrix_multi_N_1( uint32_t sz, float *pM, float *pv, float *pOut );
void calc_matrix_multi_N_N( uint32_t sz, float *pM, float *pN, float *pOut );

#ifdef cplusplus
}
#endif

#endif //#ifndef __MATRIX_LIB_H__
