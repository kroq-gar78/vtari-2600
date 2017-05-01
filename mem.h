#ifndef MEM_H
#define MEM_H

#include "cpu.h"

#define MEM_SIZE 128

extern byte mem[];

extern void mem_init();
//extern void mem_map(ushrt guest);

extern void mem_set8(ushrt addr, byte value);

extern void mem_copy(byte* host, ushrt guest, ushrt size);

extern byte mem_get8(ushrt addr);
extern short mem_get16(ushrt addr);
extern int mem_get32(ushrt addr);

#endif
