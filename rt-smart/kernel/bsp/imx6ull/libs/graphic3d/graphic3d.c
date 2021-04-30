/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-03     Lyons        first version
 */

#include <rtthread.h>

#ifdef RT_USING_GRAPHIC3D

#include <dfs_posix.h>

#include "__def.h"
#include "matrix.h"
#include "graphic3d.h"

#include "bsp_lcdapi.h"
#include "drv_lcd.h"

//-----------------------------------------------
//      本文件使用的宏定义
//-----------------------------------------------
#define POINT_XY(idx,sz) g_TransferMatrixInfo.width-(uint16_t)ppoint[(idx)*(sz)+0], (uint16_t)ppoint[(idx)*(sz)+1]

//-----------------------------------------------
//      本文件使用的结构体，共用体，枚举
//-----------------------------------------------

//-----------------------------------------------
//      全局使用的变量，常量
//-----------------------------------------------
TriangleInfo_t _g_TriangleInfo = {
    .edge_color = RGB_COLOR_BLACK,
    .face_color = RGB_COLOR_WHITE,
    .vertex_color = RT_NULL,
};

TripyramidInfo_t _g_TripyramidInfo = {
    .edge_color = RGB_COLOR_BLACK,
    .face_color = RGB_COLOR_RED,
    .vertex_color = RT_NULL,
};

BMPHeadInfo_t _g_BMPHeadInfo = {0};

//-----------------------------------------------
//      本文件使用的变量，常量
//-----------------------------------------------
_internal_rw float _s_DeepBuffer[BSP_GRAPHIC3D_BUFFER_WIDTH*BSP_GRAPHIC3D_BUFFER_HEIGHT];

_internal_ro uint32_t _k_Axis3DColor[3] = {
    RGB_COLOR_RED, RGB_COLOR_GREEN, RGB_COLOR_BLUE,
};

//点光源位置（世界坐标系下）
_internal_rw float _s_Light_Position[][3] = {
    {300, 80, -120},
    {-120, 300, 0},
};

//点光源强度（显示区域越大，强度值越大）
_internal_rw float _s_Light_Intensity[][3] = {
    {24000, 24000, 24000},
    {24000, 24000, 24000},
};

//环境光强度（显示区域越大，强度值越大）
_internal_rw float _s_Amb_Light_Intensity[3] = {
    5000, 5000, 5000,
};

//镜面反射指数因子的幂
_internal_ro float _k_SpecularFactorPower = 150.0f;

//-----------------------------------------------
//      内部函数声明
//-----------------------------------------------

//-----------------------------------------------
//      函数定义
//-----------------------------------------------
//-----------------------------------------------
//功能描述：判断点是否在三角形内
//
//参    数：x,y[in],      屏幕上要判断的点的坐标（计算像素点的中点，像素点差值为1）
//          *ppointp[in], 三角形顶点坐标，[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]
//
//返 回 值：1, 在内部
//          0, 在外部
//
//备注内容：无
//-----------------------------------------------
static int _IsInsideTriangle( float x, float y, float *ptpoint )
{
    float AB[2];
    float BC[2];
    float CA[2];

    float AP[2];
    float BP[2];
    float CP[2];

    int is_inside[2];

    AB[0] = ptpoint[3*1+0] - ptpoint[3*0+0];
    AB[1] = ptpoint[3*1+1] - ptpoint[3*0+1];

    BC[0] = ptpoint[3*2+0] - ptpoint[3*1+0];
    BC[1] = ptpoint[3*2+1] - ptpoint[3*1+1];

    CA[0] = ptpoint[3*0+0] - ptpoint[3*2+0];
    CA[1] = ptpoint[3*0+1] - ptpoint[3*2+1];

    AP[0] = x - ptpoint[3*0+0];
    AP[1] = y - ptpoint[3*0+1];

    BP[0] = x - ptpoint[3*1+0];
    BP[1] = y - ptpoint[3*1+1];

    CP[0] = x - ptpoint[3*2+0];
    CP[1] = y - ptpoint[3*2+1];

    is_inside[0] = ((AB[0]*AP[1] - AB[1]*AP[0]) > 0)
                && ((BC[0]*BP[1] - BC[1]*BP[0]) > 0)
                && ((CA[0]*CP[1] - CA[1]*CP[0]) > 0);
    is_inside[1] = ((AB[0]*AP[1] - AB[1]*AP[0]) < 0)
                && ((BC[0]*BP[1] - BC[1]*BP[0]) < 0)
                && ((CA[0]*CP[1] - CA[1]*CP[0]) < 0);

    return (is_inside[0] || is_inside[1]);
}

//-----------------------------------------------
//功能描述：计算三角形重心坐标
//
//参    数：x,y[in],      屏幕上要判断的点的坐标（计算像素点的中点，像素点差值为1）
//          *ppointp[in], 三角形顶点坐标，[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]
//          *pOut[out],   重心坐标，[α,β,γ]
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
static void _ComputeBarycentric2D( float x, float y, float *ptpoint,
                                   float *pOut )
{
    pOut[0] = (x*(ptpoint[3*1+1] - ptpoint[3*2+1]) + y*(ptpoint[3*2+0] - ptpoint[3*1+0]) + ptpoint[3*1+0]*ptpoint[3*2+1] - ptpoint[3*2+0]*ptpoint[3*1+1])
            / (ptpoint[3*0+0]*(ptpoint[3*1+1] - ptpoint[3*2+1]) + (ptpoint[3*2+0] - ptpoint[3*1+0])*ptpoint[3*0+1] + ptpoint[3*1+0]*ptpoint[3*2+1] - ptpoint[3*2+0]*ptpoint[3*1+1]);

    pOut[1] = (x*(ptpoint[3*2+1] - ptpoint[3*0+1]) + y*(ptpoint[3*0+0] - ptpoint[3*2+0]) + ptpoint[3*2+0]*ptpoint[3*0+1] - ptpoint[3*0+0]*ptpoint[3*2+1])
            / (ptpoint[3*1+0]*(ptpoint[3*2+1] - ptpoint[3*0+1]) + (ptpoint[3*0+0] - ptpoint[3*2+0])*ptpoint[3*1+1] + ptpoint[3*2+0]*ptpoint[3*0+1] - ptpoint[3*0+0]*ptpoint[3*2+1]);

    pOut[2] = (x*(ptpoint[3*0+1] - ptpoint[3*1+1]) + y*(ptpoint[3*1+0] - ptpoint[3*0+0]) + ptpoint[3*0+0]*ptpoint[3*1+1] - ptpoint[3*1+0]*ptpoint[3*0+1])
            / (ptpoint[3*2+0]*(ptpoint[3*0+1] - ptpoint[3*1+1]) + (ptpoint[3*1+0] - ptpoint[3*0+0])*ptpoint[3*2+1] + ptpoint[3*0+0]*ptpoint[3*1+1] - ptpoint[3*1+0]*ptpoint[3*0+1]);
}

//-----------------------------------------------
//功能描述：根据重心坐标对三角形内插值
//
//参    数：*pbarycentric[in], 重心坐标（SIZE=3）
//          *pIn[in],          输入信息（SIZE=3×SIZE），[x1, y1, z1], [x2, y2, z2], ...
//          *pOut[out],        输出信息（SIZE=3）
//          weight[in],        权值
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
static void _interpolate( float *pbarycentric, float *pIn, float *pOut, float weight )
{
    for (int i=0; i<3; i++) {
        pOut[i] = 0.0f;
    }

    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            pOut[j] += (pbarycentric[i] * pIn[3*i+j]);
        }
    }

    for (int i=0; i<3; i++) {
        pOut[i] /= weight;
    }

}

//-----------------------------------------------
//功能描述：填充单个三棱锥
//
//参    数：*ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          *pcolor[in], 顶点颜色信息
//          num[in],     ppoint大小（目前num必须为9！）
//
//返 回 值：void
//
//备注内容：三棱锥的每个面内颜色来自其顶点（重心坐标插值）
//-----------------------------------------------
static void _draw_sub_fill_tripyramid( float *ppoint, uint32_t *pcolor, uint16_t num )
{
    float min_x = MIN(ppoint[3*0+0], MIN(ppoint[3*1+0], ppoint[3*2+0]));
    float max_x = MAX(ppoint[3*0+0], MAX(ppoint[3*1+0], ppoint[3*2+0]));
    float min_y = MIN(ppoint[3*0+1], MIN(ppoint[3*1+1], ppoint[3*2+1]));
    float max_y = MAX(ppoint[3*0+1], MAX(ppoint[3*1+1], ppoint[3*2+1]));

    if ((!ppoint) || (9 != num) )
        return;

    min_x = (int)floor(min_x);
    max_x = (int)ceil(max_x);
    min_y = (int)floor(min_y);
    max_y = (int)ceil(max_y);

    min_x = MAX(0, min_x);
    max_x = MIN(1000, max_x);
    min_y = MAX(0, min_y);
    max_y = MIN(1000, max_y);

    for (int x=min_x; x<=max_x; x++)
    {
        for (int y=min_y; y<=max_y; y++)
        {
            if (_IsInsideTriangle(x+0.5f, y+0.5f, ppoint))
            {
                float barycentric[3];
                _ComputeBarycentric2D(x+0.5f, y+0.5f, ppoint, barycentric); //1.59us

                float interpolate_point[3];
                _interpolate(barycentric, ppoint, interpolate_point, 1.0f);

                uint32_t index = y*g_TransferMatrixInfo.width + x;
                if ( (index < GET_ARRAY_NUM(_s_DeepBuffer))
                  && (_s_DeepBuffer[index] > interpolate_point[2]) )
                {
                    _s_DeepBuffer[index] = interpolate_point[2];

                    if (_g_TripyramidInfo.vertex_color)
                    {
                        lcd_color32_t color;
                        float color24[9];

                        for (int i=0; i<3; i++)
                        {
                            color.full = pcolor[i];
                            color24[3*i+0] = color.ch.red;
                            color24[3*i+1] = color.ch.green;
                            color24[3*i+2] = color.ch.blue;
                        }

                        float result_color[3];
                        _interpolate(barycentric, color24, result_color, 1.0f);

                        color.ch.red = result_color[0];
                        color.ch.green = result_color[1];
                        color.ch.blue = result_color[2];

                        lcd_draw_point(g_TransferMatrixInfo.width-x, y, color.full);
                    }
                }
            }
        }
    }
}

//-----------------------------------------------
//功能描述：对单个三角形进行Phong着色
//
//参    数：*ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          *pvn[in],    法线信息，[vnx, vny, vnz],
//          *pvt[in],    纹理坐标，[vnt, vty]
//          num[in],     ppoint大小（目前num必须为9！）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
static void _phong_fragment_shader( float *ppoint, float *pvn, float *pvt, uint16_t num )
{
    float min_x = MIN(ppoint[3*0+0], MIN(ppoint[3*1+0], ppoint[3*2+0]));
    float max_x = MAX(ppoint[3*0+0], MAX(ppoint[3*1+0], ppoint[3*2+0]));
    float min_y = MIN(ppoint[3*0+1], MIN(ppoint[3*1+1], ppoint[3*2+1]));
    float max_y = MAX(ppoint[3*0+1], MAX(ppoint[3*1+1], ppoint[3*2+1]));

    if ((!ppoint) || (9 != num) )
        return;

    min_x = (int)floor(min_x);
    max_x = (int)ceil(max_x);
    min_y = (int)floor(min_y);
    max_y = (int)ceil(max_y);

    min_x = MAX(0, min_x);
    max_x = MIN(1000, max_x);
    min_y = MAX(0, min_y);
    max_y = MIN(1000, max_y);

    for (int x=min_x; x<=max_x; x++)
    {
        for (int y=min_y; y<=max_y; y++)
        {
            if (_IsInsideTriangle(x+0.5f, y+0.5f, ppoint))
            {
                float barycentric[3];
                _ComputeBarycentric2D(x+0.5f, y+0.5f, ppoint, barycentric); //1.59us

                float interpolate_point[3], interpolate_texture[3];
                _interpolate(barycentric, ppoint, interpolate_point, 1.0f);
                _interpolate(barycentric, pvt, interpolate_texture, 1.0f);

                uint32_t index = y*g_TransferMatrixInfo.width + x;
                if ( (index < GET_ARRAY_NUM(_s_DeepBuffer))
                  && (_s_DeepBuffer[index] > interpolate_point[2]) )
                {
                    _s_DeepBuffer[index] = interpolate_point[2];

                    lcd_color32_t color;
                    float color24[9];

                    if (pvt)
                        _g_TriangleInfo.face_color = G3D_GetBMPPointColor(interpolate_texture[0], interpolate_texture[1], &_g_BMPHeadInfo);

                    color.full = _g_TriangleInfo.face_color;

                    for (int i=0; i<3; i++)
                    {
                        color24[3*i+0] = color.ch.red;
                        color24[3*i+1] = color.ch.green;
                        color24[3*i+2] = color.ch.blue;
                    }

                    float vn_inpolated[3],v_inpolated[3],c_inpolated[3];

                    _interpolate(barycentric, color24, c_inpolated, 1.0f);
                    _interpolate(barycentric, pvn, vn_inpolated, 1.0f);
                    _interpolate(barycentric, ppoint, v_inpolated, 1.0f);

                    float Ka[3] = {0.000f, 0.000f, 0.000f};
                    float Kd[3] = {1.0f, 1.0f, 1.0f};
                    calc_vector_nummulti(3, c_inpolated, 1.0f/255.0f, Kd);
                    float Ks[3] = {0.7937f, 0.7937f, 0.7937f};

                    float result_color[3] = {0.0f, 0.0f, 0.0f};
                    for (int i=0; i<GET_ARRAY_NUM(_s_Light_Position); i++)
                    {
                        float dummy_data[3];

                        float v[3],l[3],h[3];
                        float nn[3],vn[3],ln[3],hn[3];

                        calc_vector_minus(3, g_TransferMatrixInfo.eye_point, v_inpolated, v);
                        calc_vector_minus(3, _s_Light_Position[i], v_inpolated, l);

                        calc_vector_normalized(3, vn_inpolated, nn);
                        calc_vector_normalized(3, v, vn);
                        calc_vector_normalized(3, l, ln);

                        calc_vector_plus(3, ln, vn, h);
                        calc_vector_normalized(3, h, hn);

                        float r = 1.0f / calc_vector_multi(3, l, l);
                        float decay[3];
                        calc_vector_nummulti(3, _s_Light_Intensity[i], r, decay);

                        float La[3],Ld[3],Ls[3];
                        float dump;

                        //环境光分量
                        calc_vector_cwisemulti(3, Ka, _s_Amb_Light_Intensity, La);

                        //漫反射分量
                        calc_vector_cwisemulti(3, Kd, decay, Ld);
                        dump = MAX(0.0f, calc_vector_multi(3, nn, ln));
                        calc_vector_nummulti(3, Ld, dump, Ld);

                        //镜面反射分量
                        calc_vector_cwisemulti(3, Ks, decay, Ls);
                        dump = pow(MAX(0.0f, calc_vector_multi(3, nn, hn)), _k_SpecularFactorPower);
                        calc_vector_nummulti(3, Ls, dump, Ls);

                        //求和
                        calc_vector_plus(3, La, Ld, dummy_data);
                        calc_vector_plus(3, Ls, dummy_data, dummy_data);

                        calc_vector_plus(3, dummy_data, result_color, result_color);
                    }

                    calc_vector_nummulti(3, result_color, 255.0f/GET_ARRAY_NUM(_s_Light_Position), result_color);
                    calc_vector_minlimit(3, result_color, 255.0f, result_color);

                    color.ch.red = (uint8_t)result_color[0];
                    color.ch.green = (uint8_t)result_color[1];
                    color.ch.blue = (uint8_t)result_color[2];
                    color.ch.alpha = 0;

                    lcd_draw_point(g_TransferMatrixInfo.width-x, y, color.full);
                }
            }
        }
    }
}

//-----------------------------------------------
//功能描述：绘制单个三角形
//
//参    数：ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          num[in],    ppoint大小（对于三角形，num必须为9）
//          color[in],  线条颜色
//
//返 回 值：void
//
//备注内容：只能由draw_triangle()调用！
//-----------------------------------------------
static void _draw_sub_triangle( float *ppoint, uint16_t num, uint32_t color )
{
    if ((!ppoint) || (9 != num) )
        return;

    lcd_draw_line( POINT_XY(0,3), POINT_XY(1,3), color );
    lcd_draw_line( POINT_XY(1,3), POINT_XY(2,3), color );
    lcd_draw_line( POINT_XY(2,3), POINT_XY(0,3), color );
}

//-----------------------------------------------
//功能描述：绘制单个三棱锥
//
//参    数：ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          num[in],    ppoint大小（对于三棱锥，num必须为12）
//          color[in],  线条颜色
//
//返 回 值：void
//
//备注内容：只能由draw_tripyramid()调用！
//-----------------------------------------------
static void _draw_sub_tripyramid( float *ppoint, uint16_t num, uint32_t color )
{
    if ((!ppoint) || (12 != num) )
        return;

    lcd_draw_line( POINT_XY(3,3), POINT_XY(0,3), color );
    lcd_draw_line( POINT_XY(3,3), POINT_XY(1,3), color );
    lcd_draw_line( POINT_XY(3,3), POINT_XY(2,3), color );

    lcd_draw_line( POINT_XY(0,3), POINT_XY(1,3), color );
    lcd_draw_line( POINT_XY(1,3), POINT_XY(2,3), color );
    lcd_draw_line( POINT_XY(2,3), POINT_XY(0,3), color );
}

//-----------------------------------------------
//功能描述：绘制三棱锥
//
//参    数：ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          num[in],    ppoint大小（对于三棱锥，num必须为12的整数倍）
//
//返 回 值：void
//
//备注内容：可绘制多个三棱锥
//-----------------------------------------------
void G3D_InitDeepBuffer(void)
{
    uint32_t i;

    for (i=0; i<GET_ARRAY_NUM(_s_DeepBuffer); i++)
    {
        _s_DeepBuffer[i] = 1/0.0L;
    }
}

//-----------------------------------------------
//功能描述：绘制坐标系图示
//
//参    数：ppoint[in],  顶点坐标信息，[x0, y0, z0], [x1, y1, z1], [x2, y2, z2], [x3, y3, z3]
//          num[in],     ppoint大小（num必须为12）
//          *pcolor[in], 线条颜色（如果传入空指针，则使用默认颜色）
//
//返 回 值：void
//
//备注内容：无
//-----------------------------------------------
void G3D_GrawAxis( float *ppoint, uint16_t num, uint32_t *pcolor )
{
    uint32_t color;

    if ((!ppoint) || (12 != num) )
        return;

    for (int i=0; i<3; i++)
    {
        color = (!pcolor) ? _k_Axis3DColor[i] : *pcolor;
        lcd_draw_line( POINT_XY(0,3), POINT_XY(i+1,3), color );
    }
}

//-----------------------------------------------
//功能描述：绘制三角形
//
//参    数：type[in],    0,绘制边框/1,Phong着色/2,法线贴图
//          *ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          *pvn[in],    顶点法线信息，[vnx1, vny1, vnz1], [vnx2, vny2, vnz2], ...
//          *pvt[in],    纹理坐标，[vtx1, vty1, 0], [vtx2, vty2, 0], ...
//          num[in],     ppoint大小（对于三角形，num必须为9的整数倍）
//
//返 回 值：void
//
//备注内容：可绘制多个三角形
//-----------------------------------------------
void G3D_DrawTriangle( uint8_t type, float *ppoint, float *pvn, float *pvt, uint16_t num )
{
    uint16_t i;

    if (0 != (num%9))
    {
        return;
    }

    switch (type)
    {
        case 0:
            //绘制棱
            for (i=0; i<num; i+=9)
            {
                _draw_sub_triangle( &ppoint[i], 9, _g_TriangleInfo.edge_color );
            }
            break;

        case 1:
            //Phong着色
            for (i=0; i<num; i+=9)
            {
                _phong_fragment_shader( &ppoint[i], &pvn[i], RT_NULL, 9 );
            }
            break;

        case 2:
            //法线贴图
            for (i=0; i<num; i+=9)
            {
                pvt[i+2] = pvt[i+3+2] = pvt[i+6+2] = 0.0f;
                _phong_fragment_shader( &ppoint[i], &pvn[i], &pvt[i], 9 );
            }
            break;

        default:
            break;
    }
}

//-----------------------------------------------
//功能描述：绘制三棱锥
//
//参    数：type[in],    0,绘制边框/1,绘制填充
//          *ppoint[in], 顶点坐标信息，[x1, y1, z1], [x2, y2, z2], ...
//          num[in],     ppoint大小（对于三棱锥，num必须为12的整数倍）
//
//返 回 值：void
//
//备注内容：可绘制多个三棱锥
//-----------------------------------------------
void G3D_DrawTripyramid( uint8_t type, float *ppoint, uint16_t num )
{
    float point_buf[9];
    uint32_t color_buf[3];
    uint16_t i,j;

    if (0 != (num%12))
    {
        return;
    }

    switch (type)
    {
        case 0:
            //绘制棱
            for (i=0; i<num; i+=12)
            {
                _draw_sub_tripyramid( &ppoint[i], 12, _g_TripyramidInfo.edge_color );
            }
            break;

        case 1:
            //填充面
            for (i=0; i<4; i++) //1个三棱锥需要绘制4个三角形
            {
                for (j=0; j<3; j++)
                {
                    rt_memcpy( &point_buf[3*j], &ppoint[3*((i+j)%4)], sizeof(float)*3 );
                    color_buf[j] = _g_TripyramidInfo.vertex_color[(i+j)%4];
                }

                _draw_sub_fill_tripyramid( point_buf, color_buf, 9 ); //13.65ms
            }
            break;

        default:
            break;
    }
}

//-----------------------------------------------
//功能描述：获取BMP文件的文件头
//
//参    数：*ppath[in],  图像文件的路径
//          *pInfo[out], 图像的头部信息
//
//返 回 值：void
//
//备注内容：目前图片存储在sd卡中
//-----------------------------------------------
void G3D_GetBMPInfo( const char *ppath, BMPHeadInfo_t *pInfo )
{
    int fd;
    uint8_t dummy_buf[54];

    if (!ppath || !pInfo)
        return;

    rt_memset(pInfo, 0, sizeof(BMPHeadInfo_t));
    pInfo->fd = -1;

    fd = open(ppath, O_RDONLY);
    if (fd < 0)
        return;

    read(fd, dummy_buf, 54);
    if( (dummy_buf[0] != 'B') && (dummy_buf[1] != 'M') )
        return;

    pInfo->fd = fd;

    pInfo->Width = (dummy_buf[0x15]<<24) | (dummy_buf[0x14]<<16) | (dummy_buf[0x13]<<8) | dummy_buf[0x12]; //图片宽度和高度，各4字节，高字节在前
    pInfo->Height = (dummy_buf[0x19]<<24) | (dummy_buf[0x18]<<16) | (dummy_buf[0x17]<<8) | dummy_buf[0x16];

    pInfo->Offset = 54;
}

//-----------------------------------------------
//功能描述：获取BMP某一点的颜色
//
//参    数：*ppath[in],  图像文件的路径
//          *pInfo[out], 图像的头部信息
//
//返 回 值：颜色值
//
//备注内容：目前图片存储在sd卡中
//-----------------------------------------------
static void _get_bmp_point_color( uint32_t u, uint32_t v, BMPHeadInfo_t *pInfo, float *pOut )
{
    uint32_t offset;
    uint8_t dummy_buf[3] = {0};

    offset = v*pInfo->Width + u;
    lseek(pInfo->fd, pInfo->Offset+offset*3, SEEK_SET);

    read(pInfo->fd, dummy_buf, sizeof(dummy_buf));

    for (int i=0; i<3; i++)
        pOut[i] = dummy_buf[2-i];
}

//-----------------------------------------------
//功能描述：获取BMP某一点的颜色
//
//参    数：u,v[in],     纹理坐标
//          *pInfo[out], 图像的头部信息
//
//返 回 值：RGB24颜色值
//
//备注内容：使用双线性插值
//-----------------------------------------------
uint32_t G3D_GetBMPPointColor( float u, float v, BMPHeadInfo_t *pInfo )
{
    float u_img,v_img;
    float s,t;
    float u0[3],u1[3];

    float u00[2],u01[2],u10[2],u11[2];
    float dummy_data[3];

    if ( (!pInfo) || (!pInfo->fd) )
        return _g_TriangleInfo.face_color;

    u = (u < 0.0f) ? 0.0f : u;
    u = (u > 1.0f) ? 1.0f : u;
    v = (v < 0.0f) ? 0.0f : v;
    v = (v > 1.0f) ? 1.0f : v;

    u_img = u * (pInfo->Width - 1);
    v_img = v * (pInfo->Height - 1);

    u00[0] = floor(u_img);
    u00[1] = floor(v_img);

    u01[0] = ceil(u_img);
    u01[1] = floor(v_img);

    u10[0] = floor(u_img);
    u10[1] = ceil(v_img);

    u11[0] = ceil(u_img);
    u11[1] = ceil(v_img);

    s = u_img - floor(u_img);
    t = v_img - floor(v_img);

    _get_bmp_point_color(u01[0], u01[1], pInfo, dummy_data);
    _get_bmp_point_color(u00[0], u00[1], pInfo, u0);
    calc_vector_minus(3, dummy_data, u0, dummy_data);
    calc_vector_nummulti(3, dummy_data, s, dummy_data);
    calc_vector_plus(3, u0, dummy_data, u0);

    _get_bmp_point_color(u11[0], u11[1], pInfo, dummy_data);
    _get_bmp_point_color(u10[0], u10[1], pInfo, u1);
    calc_vector_minus(3, dummy_data, u1, dummy_data);
    calc_vector_nummulti(3, dummy_data, s, dummy_data);
    calc_vector_plus(3, u1, dummy_data, u1);

    calc_vector_minus(3, u1, u0, dummy_data);
    calc_vector_nummulti(3, dummy_data, t, dummy_data);
    calc_vector_plus(3, u0, dummy_data, dummy_data);

    lcd_color32_t color;

    color.ch.red =  (uint8_t)dummy_data[0];
    color.ch.green =  (uint8_t)dummy_data[1];
    color.ch.blue =  (uint8_t)dummy_data[2];
    color.ch.alpha = 0;

    return color.full;
}

#endif //#ifdef RT_USING_GRAPHIC3D
