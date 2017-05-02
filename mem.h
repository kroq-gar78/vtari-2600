#ifndef MEM_H
#define MEM_H

#include "cpu.h"

#define RAM_SIZE 128
#define CART_SIZE (1<<12)
#define MEM_MAX (1<<13)
#define TIA_SIZE 0xF

#define ADDR_BAD 0x7777

extern byte pia_mem[RAM_SIZE];
extern byte cart_mem[CART_SIZE];

extern void mem_init();
//extern void mem_map(ushrt guest);

extern void mem_set8(ushrt addr, byte value);

extern void mem_copy(byte* host, ushrt guest, ushrt size);

extern byte mem_get8(ushrt addr);
extern short mem_get16(ushrt addr);
extern short mem_get16_zpg(ushrt addr);
extern int mem_get32(ushrt addr);

#endif
