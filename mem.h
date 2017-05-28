#ifndef MEM_H
#define MEM_H

#include "cpu.h"

#define RAM_SIZE 128
#define CART_SIZE (1<<12)
#define TIA_SIZE 0x3E

#if MOS_6502
#define MEM_MAX 0xffff
#else
#define MEM_MAX (1<<13)
#endif

#define ADDR_BAD 0x7777

#define SWCHA 0
#define SWACNT 1
#define SWCHB 2
#define SWBCNT 3
#define INTIM 4
#define INSTAT 5
#define TIM1T 4
#define TIM8T 5
#define TIM64T 6
#define T1024T 7

extern byte riot_mem[RAM_SIZE];
extern byte cart_mem[CART_SIZE];
extern byte pia_mem[8];

extern void mem_init();
//extern void mem_map(ushrt guest);

extern void mem_set8(ushrt addr, byte value);

extern void mem_copy(byte* host, ushrt guest, ushrt size);

extern byte mem_get8(ushrt addr);
extern short mem_get16(ushrt addr);
extern short mem_get16_zpg(ushrt addr);
extern byte hex_to_bcd(byte value);
extern byte bcd_to_hex(byte bcd);

#endif
