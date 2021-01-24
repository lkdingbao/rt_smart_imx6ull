/*
 * USER
 *   user def file
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-09     Lyons        first version
 */

#ifndef __USER_DEF_H__
#define __USER_DEF_H__

#ifndef UNUSED
#define UNUSED(X) 					((void)X)
#endif 

#define CLEAR_ARRAY(ins,data)		memset((uint8_t*)ins,data,sizeof(ins))
#define CLEAR_STRUCT(ins,data)		memset((uint8_t*)ins,data,sizeof(ins))

#define GET_ARRAY_NUM(ins)			((uint32_t)(sizeof(ins)/sizeof(ins[0])))
#define GET_STRUCT_SIZE(ins)		((uint32_t)(sizeof(ins)))

#define GET_WORD_BYTE0(w)			((uint8_t)((w)     & 0xFF))
#define GET_WORD_BYTE1(w)			((uint8_t)((w>>8)  & 0xFF))

#define GET_DWORD_BYTE0(d)			GET_WORD_BYTE0(d)
#define GET_DWORD_BYTE1(d)			GET_WORD_BYTE1(d)
#define GET_DWORD_BYTE2(d)			((uint8_t)(((d)>>16) & 0xFF))
#define GET_DWORD_BYTE3(d)			((uint8_t)(((d)>>24) & 0xFF))

#define BUILD_WORD(a,b)				((uint16_t)(((a)<<8 ) |  (b)))
#define BUILD_DWORD(a,b,c,d)		((uint32_t)(((a)<<24) | ((b)<<16) | ((c)<<8) | (d)))
   
#define BIT0                        0x0001
#define BIT1                        0x0002
#define BIT2                        0x0004
#define BIT3                        0x0008
#define BIT4                        0x0010
#define BIT5                        0x0020
#define BIT6                        0x0040
#define BIT7                        0x0080
#define BIT8                        0x0100
#define BIT9                        0x0200
#define BIT10                       0x0400
#define BIT11                       0x0800
#define BIT12                       0x1000
#define BIT13                       0x2000
#define BIT14                       0x4000
#define BIT15                       0x8000

#define B_0000_0000					0x00
#define B_0000_0001					0x01
#define B_0000_0010					0x02
#define B_0000_0100					0x04
#define B_0000_1000					0x08
#define B_0001_0000					0x10
#define B_0010_0000					0x20
#define B_0100_0000					0x40
#define B_1000_0000					0x80

#define _internal_ro				static const
#define _internal_rw				static
#define _internal_zi				static

#endif //#ifndef __USER_DEF_H__

