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
#include "graphic3d.h"
#include "model3d.h"

#include "bsp_lcdapi.h"
#include "drv_lcd.h"

//定义坐标原点（在世界坐标系下）
_internal_ro float _k_origin_point[3] = {
    0, 0, 0,
};

//定义视点坐标（在世界坐标系下）
_internal_ro float _k_eye_point[3] = {
#if( OBJECT_MODEL == OBJECT_MODEL_COWS )
    0, 0, 3,
#else
    3, 3, 3,
#endif
};

//定义目标点坐标（在世界坐标系下）
_internal_ro float _k_target_point[3] = {
    0, 0, 0,
};

//定义相机向上方向
_internal_ro float _k_up_vector[3] = {
    -1, 1, 0,
};

//定义坐标轴坐标
_internal_ro float _k_axis_point_buf[12] = {
    0, 0, 0,
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

//定义三角形顶点默认颜色
_internal_ro uint32_t _k_triangle_vertex_color[3] = {
    RGB_COLOR_WHITE, RGB_COLOR_WHITE, RGB_COLOR_WHITE,
};

//定义Phong着色是模型的旋转角度
_internal_rw float _s_phong_model_angle_xyz[3] = {
    0, 140, 0,
};

int graphic3d(int argc, char **argv)
{
    float matrix_mvpvp[16];
    float dummy_result[12];

    float dummy_v[GET_ARRAY_NUM(dummy_result)];
    float dummy_vn[GET_ARRAY_NUM(dummy_result)];
    float dummy_vt[GET_ARRAY_NUM(dummy_result)];

    rt_kprintf("graphics rendering ...\n");
    rt_kprintf("this will take several minutes.\n");

    lcd_clear(RGB_COLOR_BLACK);

    G3D_InitDeepBuffer();

    g_TransferMatrixInfo.width = BSP_GRAPHIC3D_BUFFER_WIDTH;
    g_TransferMatrixInfo.height = BSP_GRAPHIC3D_BUFFER_HEIGHT;

    g_TransferMatrixInfo.eye_fov = 45;
    g_TransferMatrixInfo.aspect_ratio = 1;
    g_TransferMatrixInfo.znear = 0.1;
    g_TransferMatrixInfo.zfar = 50;

    //加载相机坐标系参数（如果需要严格在物理空间中旋转，则必须设置以下参数）
    rt_memcpy(g_TransferMatrixInfo.target_point, _k_target_point, VERTEX_DATA_SIZE_3F);
    rt_memcpy(g_TransferMatrixInfo.up_vector, _k_up_vector, VERTEX_DATA_SIZE_3F);

    rt_memcpy(g_TransferMatrixInfo.eye_point, _k_eye_point, VERTEX_DATA_SIZE_3F);
    rt_memcpy(g_TransferMatrixInfo.angle, _s_phong_model_angle_xyz, VERTEX_DATA_SIZE_3F);

    create_transfer_matrix(0, &g_TransferMatrixInfo, (float*)_k_origin_point, matrix_mvpvp);

    transfer_tripyramid( _k_axis_point_buf, matrix_mvpvp, dummy_result );
    G3D_GrawAxis( dummy_result, 12, RT_NULL);

    G3D_GetBMPInfo( "/models/cows/spot_texture1.bmp", &_g_BMPHeadInfo );

    _g_TriangleInfo.edge_color = RGB_COLOR_BLACK;
    _g_TriangleInfo.face_color = RGB_COLOR_WHITE;
    _g_TriangleInfo.vertex_color = (uint32_t*)_k_triangle_vertex_color;
    
    for (int i=0; i<_MODEL_FACE_NUM; i++)
    {
        for (int j=0; j<9; j++)
        {
            dummy_v[j] = _k_model3d_v[_k_model3d_f[3*i+(j/3)][0]-1][j%3];
            dummy_vn[j] = _k_model3d_vn[_k_model3d_f[3*i+(j/3)][2]-1][j%3];
            dummy_vt[j] = _k_model3d_vt[_k_model3d_f[3*i+(j/3)][1]-1][(j%3)%2]; //注意，只有前两个数据，第三个数据无效！
        }
    
        transfer_triangle( dummy_v, matrix_mvpvp, dummy_result );
        G3D_DrawTriangle( 2, dummy_result, dummy_vn, dummy_vt, 9 );
    }

    if (_g_BMPHeadInfo.fd > 0) {
        close(_g_BMPHeadInfo.fd);
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(graphic3d, graphic3d, <usr> graphic 3d test);

#endif //#ifdef RT_USING_GRAPHIC3D

