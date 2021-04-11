/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-07-06     Bernard    the first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtconfig.h>
#include <realview.h>
#include "imx6ull.h"

#include "mmu.h"

#ifndef RT_USING_SMART
#define KERNEL_VADDR_START 0x80000000
#endif

#if defined(__CC_ARM)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void*)&Image$$RW_IRAM1$$ZI$$Limit)
#elif defined(__GNUC__)
extern int __bss_end;
#define HEAP_BEGIN      ((void*)&__bss_end)
#endif

#ifdef RT_USING_USERSPACE
#define HEAP_END        (void*)(KERNEL_VADDR_START + 16 * 1024 * 1024)
#define PAGE_START      HEAP_END
#define PAGE_END        (void*)(KERNEL_VADDR_START + 128 * 1024 * 1024)
#else
#define HEAP_END        (void*)(0x80000000 + 64 * 1024 * 1024)
#endif

void rt_hw_board_init(void);

#ifdef RT_USING_SMART
extern rt_mmu_info mmu_info;
#endif

#endif
