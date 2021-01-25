/*
 * GT9147
 *   GT9147 driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-25     Lyons        first version
 */

#ifndef __DRV_GT9147_H__
#define __DRV_GT9147_H__

#include <board.h>

#define GT_CTRL_REG             0X8040  /* Ctrl Register                */
#define GT_MODSW_REG            0X804D  /* Mode Switch Register         */
#define GT_CFGS_REG             0X8047  /* Config Address Start         */
#define GT_CHECK_REG            0X80FF  /* Check Sum Register           */
#define GT_PID_REG              0X8140  /* Product ID Register          */
#define GT_RESULT_REG           0X814E  /* Result Register              */

#define GT_TP1_REG              0X8150  /* 1st Touch Point, length 6Byte */
#define GT_TP2_REG              0X8158  /* 2nd Touch Point, length 6Byte */
#define GT_TP3_REG              0X8160  /* 3rd Touch Point, length 6Byte */
#define GT_TP4_REG              0X8168  /* 4th Touch Point, length 6Byte */
#define GT_TP5_REG              0X8170  /* 5th Touch Point, length 6Byte */

#define GT_FLAG_NEW_DATA        (1 << 0)

struct TouchPointData
{
    rt_uint16_t                 xPos;
    rt_uint16_t                 yPos;

    rt_uint16_t                 pointSz;

    rt_uint8_t                  flag; //1-having update
};

extern rt_uint8_t _g_gt9147_flag;

#endif //#ifndef __DRV_GT9147_H__

