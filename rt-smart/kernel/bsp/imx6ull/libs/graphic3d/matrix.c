/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-03     Lyons        first version
 */

#include "__def.h"
#include "matrix.h"

//-----------------------------------------------
//      本文件使用的宏定义
//-----------------------------------------------

//-----------------------------------------------
//      本文件使用的结构体，共用体，枚举
//-----------------------------------------------

//-----------------------------------------------
//      全局使用的变量，常量
//-----------------------------------------------

//-----------------------------------------------
//      本文件使用的变量，常量
//-----------------------------------------------

//-----------------------------------------------
//      内部函数声明
//-----------------------------------------------

//-----------------------------------------------
//      函数定义
//-----------------------------------------------
//-----------------------------------------------
//功能描述：向量归一化
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          *pOut[out], Nx1列向量
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_normalized( uint32_t sz, float *pu, float *pOut )
{
    float len;

    len = calc_vector_multi(sz, pu, pu);
    len = sqrt(len);

    calc_vector_nummulti(sz, pu, 1.0f/len, pOut);
}

//-----------------------------------------------
//功能描述：对向量每一维max操作
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          minv[in],   比较值
//          *pOut[out], Nx1列向量
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_maxlimit( uint32_t sz, float *pu, float maxv, float *pOut )
{
    uint32_t i;

    for (i=0; i<sz; i++)
    {
        pOut[i] = MAX(pu[i], maxv);
    }
}

//-----------------------------------------------
//功能描述：对向量每一维min操作
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          minv[in],   比较值
//          *pOut[out], Nx1列向量
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_minlimit( uint32_t sz, float *pu, float maxv, float *pOut )
{
    uint32_t i;

    for (i=0; i<sz; i++)
    {
        pOut[i] = MIN(pu[i], maxv);
    }
}

//-----------------------------------------------
//功能描述：计算向量加法：u+v
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          *pv[in],    Nx1列向量
//          *pOut[out], Nx1列向量（计算结果u+v）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_plus( uint32_t sz, float *pu, float *pv, float *pOut )
{
    uint32_t i;

    for (i=0; i<sz; i++)
    {
        pOut[i] = pu[i] + pv[i];
    }
}

//-----------------------------------------------
//功能描述：计算向量减法：u-v
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          *pv[in],    Nx1列向量
//          *pOut[out], Nx1列向量（计算结果u-v）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_minus( uint32_t sz, float *pu, float *pv, float *pOut )
{
    uint32_t i;

    for (i=0; i<sz; i++)
    {
        pOut[i] = pu[i] - pv[i];
    }
}

//-----------------------------------------------
//功能描述：计算向量乘法：u·v
//
//参    数：sz[in],  向量维数
//          *pu[in], Nx1列向量
//          *pv[in], Nx1列向量
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
float calc_vector_multi( uint32_t sz, float *pu, float *pv )
{
    float result;
    uint32_t i;

    result = 0;
    for (i=0; i<sz; i++)
    {
        result += (pu[i] * pv[i]);
    }

    return result;
}

//-----------------------------------------------
//功能描述：计算向量叉乘：u×v
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          *pv[in],    Nx1列向量
//          *pOut[out], Nx1列向量（计算结果u×v）
//
//返 回 值：void
//
//备注内容：目前sz只支持3！
//-----------------------------------------------
void calc_vector_cross( uint32_t sz, float *pu, float *pv, float *pOut )
{
    if (3 != sz)
        return;

    pOut[0] = pu[1]*pv[2] - pv[1]*pu[2];
    pOut[1] = -(pu[0]*pv[2] - pv[0]*pu[2]);
    pOut[2] = pu[0]*pv[1] - pv[0]*pu[1];
}

//-----------------------------------------------
//功能描述：计算向量对应位置相乘：u·v
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          *pv[in],    Nx1列向量
//          *pOut[out], Nx1列向量（计算结果u·n）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_cwisemulti( uint32_t sz, float *pu, float *pv, float *pOut )
{
    uint32_t i;

    for (i=0; i<sz; i++)
    {
        pOut[i] = pu[i] * pv[i];
    }
}

//-----------------------------------------------
//功能描述：计算向量数乘：u·n
//
//参    数：sz[in],     向量维数
//          *pu[in],    Nx1列向量
//          n[in],      除数
//          *pOut[out], Nx1列向量（计算结果M·n）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_vector_nummulti( uint32_t sz, float *pu, float n, float *pOut )
{
    uint32_t i;

    for (i=0; i<sz; i++)
    {
        pOut[i] = pu[i] * n;
    }
}

//-----------------------------------------------
//功能描述：计算矩阵乘法：M·v
//
//参    数：sz[in],     矩阵或向量维数
//          *pM[in],    NxN矩阵
//          *pv[in],    Nx1列向量
//          *pOut[out], Nx1列向量（计算结果M·v）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_matrix_multi_N_1( uint32_t sz, float *pM, float *pv, float *pOut )
{
    uint32_t i,k;

    for (i=0; i<sz; i++)
    {
        pOut[i] = 0;
        for (k=0; k<sz; k++)
        {
            pOut[i] += (pM[sz*i+k] * pv[k]);
        }
    }
}

//-----------------------------------------------
//功能描述：计算矩阵乘法：M·N
//
//参    数：sz[in],     矩阵或向量维数
//          *pM[in],    4X4矩阵
//          *pN[in],    4X4矩阵
//          *pOut[out], 4X4矩阵（计算结果M·N）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void calc_matrix_multi_N_N( uint32_t sz, float *pM, float *pN, float *pOut )
{
    uint32_t i,j,k;

    for (i=0; i<sz; i++)
    {
        for (j=0; j<sz; j++)
        {
            pOut[sz*i+j] = 0;
            for (k=0; k<sz; k++)
            {
                pOut[sz*i+j] += (pM[sz*i+k] * pN[sz*k+j]);
            }
        }
    }
}
