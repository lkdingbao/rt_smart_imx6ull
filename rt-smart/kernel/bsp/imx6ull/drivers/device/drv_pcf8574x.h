/*
 * PCF8574x
 *   PCF8574x driver file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-17     Lyons        first version
 */

#ifndef __DRV_PCF8574X_H__
#define __DRV_PCF8574X_H__

#include "skt.h"

#ifdef RT_USING_PCF8574

//* lcm模块说明文档：
//* 
//* 支持的指令：
//* 1.0000 0001, 0x01,  清屏指令
//*                     将空格字符送入全部DDRAM地址中，光标归零
//* 2.0000 001X, 0x02,  光标归零
//* 3.0000 01AB,     ,  设置输入模式
//*                     A=1 光标自动右移，地址加一  A=0 光标自动左移，光标减一
//*                     B=1 全部显示移动，光标不动  B=0 不移动(注意：此指令并非移屏！)，一般B为0！
//* 4.0000 1DBC      ,  显示开/关控制
//*                     D=1 开显示    D=0 关显示
//*                     C=1 光标显示  C=0 光标不显示
//*                     B=1 光标闪烁  B=0 光标不闪烁
//* 5.0001 ABXX,     ,  光标或显示移位元
//*                     AB  功能说明
//*                     00  光标向左移动，AC自动减一
//*                     01  光标向右移动，AC自动加一
//*                     10  光标和显示一起左移动，AC不变
//*                     11  光标和显示一起右移动，AC不变
//* 6.001A BCXX,     ,  功能设置
//*                     A=1 8位总线宽度   A=0 4位总线宽度
//*                     B=1 2行显示模式   B=0 1行显示模式 
//*                     C=1 字符大小5*10  C=0 字符大小5*7
//* 7.01AB CDEF,     ,  CGRAM地址设置
//*                     ABCDEF  用户自定义字符存储地址
//* 8.1ABC DEFG,     ,  DDRAM地址设置
//*                     ABCDEFG 显示地址，会被送到AC中
//* 9.BAAA AAAA,     ,  读忙标志和AC
//*                     B       忙标志
//*                     AAAAAAA 地址

#endif //#ifdef RT_USING_PCF8574
#endif //#ifndef __DRV_PCF8574X_H__

