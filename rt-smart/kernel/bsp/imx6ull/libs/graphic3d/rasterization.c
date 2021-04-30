/*
 * Copyright (c) 2019
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-03     Lyons        first version
 */

#include <rtthread.h>

#ifdef RT_USING_GRAPHIC3D

#include "__def.h"
#include "matrix.h"
#include "rasterization.h"

//-----------------------------------------------
//      ���ļ�ʹ�õĺ궨��
//-----------------------------------------------
#ifndef PI
#define PI      3.1415926f
#endif

//-----------------------------------------------
//      ���ļ�ʹ�õĽṹ�壬�����壬ö��
//-----------------------------------------------

//-----------------------------------------------
//      ȫ��ʹ�õı���������
//-----------------------------------------------
TransferMatrixInfo_t g_TransferMatrixInfo;

//-----------------------------------------------
//      ���ļ�ʹ�õı���������
//-----------------------------------------------
//���¾�����Ҫʵʱ���£�
_internal_rw float _s_M_rotate_x[16] = {
    1, 0, 0, 0,
    0, 1, 1, 0,
    0, 1, 1, 0,
    0, 0, 0, 1
};

_internal_rw float _s_M_rotate_y[16] = {
    1, 0, 1, 0,
    0, 1, 0, 0,
    1, 0, 1, 0,
    0, 0, 0, 1
};

_internal_rw float _s_M_rotate_z[16] = {
    1, 1, 0, 0,
    1, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};

_internal_rw float _s_M_view[16] = {
    1, 0, 0, 1,
    0, 1, 0, 1,
    0, 0, 1, 1,
    0, 0, 0, 1
};

_internal_rw float _s_M_persp2ortho[16] = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 1,
    0, 0, 1, 0
};

_internal_rw float _s_M_ortho_scale[16] = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};

_internal_rw float _s_M_ortho_trans[16] = {
    1, 0, 0, 1,
    0, 1, 0, 1,
    0, 0, 1, 1,
    0, 0, 0, 1
};

_internal_rw float _s_M_viewport[16] = {
    1, 0, 0, 1,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1
};

//-----------------------------------------------
//      �ڲ���������
//-----------------------------------------------

//-----------------------------------------------
//      ��������
//-----------------------------------------------
//��ȡm����
static void _get_model_matrix( float angle_x, float angle_y, float angle_z,
                               float *pOut )
{
    float matrix_temp[16];

    float f_a2r = PI / 180.0f;

    float cos_data_x = cos(angle_x*f_a2r);
    float cos_data_y = cos(angle_y*f_a2r);
    float cos_data_z = cos(angle_z*f_a2r);

    float sin_data_x = sin(angle_x*f_a2r);
    float sin_data_y = sin(angle_y*f_a2r);
    float sin_data_z = sin(angle_z*f_a2r);

    _s_M_rotate_x[5]  = cos_data_x;
    _s_M_rotate_x[6]  = -sin_data_x;
    _s_M_rotate_x[9]  = sin_data_x;
    _s_M_rotate_x[10] = cos_data_x;

    _s_M_rotate_y[0]  = cos_data_y;
    _s_M_rotate_y[2]  = sin_data_y;
    _s_M_rotate_y[8]  = -sin_data_y;
    _s_M_rotate_y[10] = cos_data_y;

    _s_M_rotate_z[0]  = cos_data_z;
    _s_M_rotate_z[1]  = -sin_data_z;
    _s_M_rotate_z[4]  = sin_data_z;
    _s_M_rotate_z[5]  = cos_data_z;

    calc_matrix_multi_N_N(4, _s_M_rotate_y, _s_M_rotate_z, matrix_temp);
    calc_matrix_multi_N_N(4, _s_M_rotate_x, matrix_temp, pOut);
}

//��ȡv����
static void _get_view_matrix( float *peye,
                              float *ptarget,
                              float *vup,
                              float *pOut )
{
    float xyz[3][3];

    rt_memset(_s_M_view, 0, sizeof(_s_M_view));
    _s_M_view[0] = _s_M_view[5] = _s_M_view[10] = _s_M_view[15] = 1;

    if (!ptarget || !vup)
    {
        _s_M_view[3]  = -peye[0];
        _s_M_view[7]  = -peye[1];
        _s_M_view[11] = -peye[2];
    }
    else
    {
        calc_vector_minus(3, peye, ptarget, xyz[2]); //z=xyz[2]
        calc_vector_normalized(3, xyz[2], xyz[2]);

        calc_vector_cross(3, xyz[2], vup, xyz[0]); //x=xyz[0]
        calc_vector_normalized(3, xyz[0], xyz[0]);

        calc_vector_cross(3, xyz[0], xyz[2], xyz[1]); //y=xyz[1]

        for (int i=0; i<3; i++)
        {
            _s_M_view[4*i+0]  = xyz[i][0];
            _s_M_view[4*i+1]  = xyz[i][1];
            _s_M_view[4*i+2]  = xyz[i][2];
            _s_M_view[4*i+3]  = -calc_vector_multi(3, xyz[i], peye);
        }
    }

    rt_memcpy(pOut, _s_M_view, sizeof(_s_M_view));
}

//��ȡp����
static void _get_projection_matrix( float eye_fov, float aspect_ratio, float zNear, float zFar,
                                    float *pOut )
{
    float matrix_temp[16];

    float angle = eye_fov / 180.0f * PI;

    float t = zNear * tan(angle/2.0f);
    float b = -t;

    float r = t * aspect_ratio;
    float l = -r;

    _s_M_persp2ortho[0]  = zNear;
    _s_M_persp2ortho[5]  = zNear;
    _s_M_persp2ortho[10] = zNear+zFar;
    _s_M_persp2ortho[11] = -zNear*zFar;

    _s_M_ortho_scale[0]  = 2/(r-l);
    _s_M_ortho_scale[5]  = 2/(t-b);
    _s_M_ortho_scale[10] = 2/(zNear-zFar);

    _s_M_ortho_trans[3]  = -(r+l)/2;
    _s_M_ortho_trans[7]  = -(t+b)/2;
    _s_M_ortho_trans[11] = -(zNear+zFar)/2;

    calc_matrix_multi_N_N(4, _s_M_ortho_trans, _s_M_persp2ortho, matrix_temp);
    calc_matrix_multi_N_N(4, _s_M_ortho_scale, matrix_temp, pOut);
}

//��ȡvp����
static void _get_viewport_matrix( uint16_t width, uint16_t height,
                                  float *pOut )
{
    _s_M_viewport[0] = width/2;
    _s_M_viewport[3] = width/2;
    _s_M_viewport[5] = height/2;
    _s_M_viewport[7] = height/2;

    rt_memcpy(pOut, _s_M_viewport, sizeof(_s_M_viewport));
}

//-----------------------------------------------
//������������ȡ������������
//
//��    ����*ppoint[in], �任ǰ�Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//          num[in],     �������
//          *pOut[out],  �任��Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//
//�� �� ֵ��void
//
//��ע���ݣ�Ŀǰֻ֧����άͼ�Σ�
//-----------------------------------------------
void get_object_center( const float *ppoint, int num, float *pOut )
{
    if (num < 1)
        return;

    for (int i=0; i<3; i++) //ֻ֧����άͼ�εļ��㣡
    {
        pOut[i] = 0.0f;
        for (int j=0; j<num; j++)
        {
            pOut[i] += ppoint[3*j+i];
        }
        pOut[i] /= num;
    }
}

//-----------------------------------------------
//��������������������ԭ����ת�ı任����
//
//��    ����Type[in],     View����Ļ�ȡ���ͣ�0-�������ϵ����������ϵ��������ת/1-������ת��
//          *pInfo[in],   TransferMatrixInfo_tָ��
//          *pcenter[in], ��ת����
//          *pOut[out],   �任����Mvp��Mp��Mv��Mm��
//
//�� �� ֵ��void
//
//��ע���ݣ���
//-----------------------------------------------
void create_transfer_matrix( int Type, TransferMatrixInfo_t *pInfo, float *pcenter, float *pOut )
{
    float matrix_c[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    };
    float matrix_m[16];
    float matrix_v[16];
    float matrix_p[16];
    float matrix_vp[16];

    float matrix_mvpvp[2][16];

    _get_model_matrix(pInfo->angle[0], pInfo->angle[1], pInfo->angle[2], matrix_m);
    if (0 == Type) {
        _get_view_matrix(pInfo->eye_point, RT_NULL, RT_NULL, matrix_v);
    } else {
        _get_view_matrix(pInfo->eye_point, pInfo->target_point, pInfo->up_vector, matrix_v);
    }
    _get_projection_matrix(pInfo->eye_fov, pInfo->aspect_ratio, pInfo->znear, pInfo->zfar, matrix_p);
    _get_viewport_matrix(pInfo->width, pInfo->height, matrix_vp);

    for (int i=0; i<3; i++) {
        matrix_c[i*4+3] = -pcenter[i];
    }
    calc_matrix_multi_N_N(4, matrix_m, matrix_c, matrix_mvpvp[0]);
    for (int i=0; i<3; i++) {
        matrix_c[i*4+3] = pcenter[i];
    }
    calc_matrix_multi_N_N(4, matrix_c, matrix_mvpvp[0], matrix_mvpvp[1]);
    calc_matrix_multi_N_N(4, matrix_v, matrix_mvpvp[1], matrix_mvpvp[0]);
    calc_matrix_multi_N_N(4, matrix_p, matrix_mvpvp[0], matrix_mvpvp[1]);
    calc_matrix_multi_N_N(4, matrix_vp, matrix_mvpvp[1], pOut);
}

//-----------------------------------------------
//�����������任������
//
//��    ����*ppoint[in], �任ǰ�Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//          vn[in],      �������
//          *tmatri[in], �任����4x4��
//          *pOut[out],  �任��Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//
//�� �� ֵ��void
//
//��ע���ݣ��任ǰ�Ķ���Ϊ�ռ��е����꣨[x,y,z]����float��
//          �任��Ķ���Ϊ��Ļ��ʾ�����꣨��[x,y]��ʹ�ã�z���õ������У���float��
//-----------------------------------------------
static void _transfer_sub_triangle( const float *ppoint, int vn, float *tmatrix, float *pOut )
{
    float point_buf[2][4];

    for (int i=0; i<vn; i++)
    {
        point_buf[0][0] = ppoint[3*i + 0];
        point_buf[0][1] = ppoint[3*i + 1];
        point_buf[0][2] = ppoint[3*i + 2];
        point_buf[0][3] = 1.0f;

        calc_matrix_multi_N_1(4, tmatrix, point_buf[0], point_buf[1]);

        pOut[3*i+0] = point_buf[1][0] / point_buf[1][3];
        pOut[3*i+1] = point_buf[1][1] / point_buf[1][3];
        pOut[3*i+2] = point_buf[1][2] / point_buf[1][3];
    }
}

//-----------------------------------------------
//�����������任������
//
//��    ����*ppoint[in], �任ǰ�Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//          *tmatri[in], �任����4x4��
//          *pOut[out],  �任��Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//
//�� �� ֵ��void
//
//��ע���ݣ���
//-----------------------------------------------
void transfer_triangle( const float *ppoint, float *tmatrix, float *pOut )
{
    _transfer_sub_triangle( ppoint, 3, tmatrix, pOut );
}

//-----------------------------------------------
//�����������任����׶
//
//��    ����*ppoint[in], �任ǰ�Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//          *tmatri[in], �任����4x4��
//          *pOut[out],  �任��Ķ���������Ϣ��[x1, y1, z1], [x2, y2, z2], ...
//
//�� �� ֵ��void
//
//��ע���ݣ���
//-----------------------------------------------
void transfer_tripyramid( const float *ppoint, float *tmatrix, float *pOut )
{
    _transfer_sub_triangle( ppoint, 4, tmatrix, pOut );
}

#endif //#ifdef RT_USING_GRAPHIC3D
