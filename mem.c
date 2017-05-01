#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

#include "mem.h"

byte mem[MEM_SIZE];

void mem_init()
{
}

//void mem_map(ushrt guest, ushrt count);

void mem_set8(ushrt addr, byte value)
{
    mem[addr] = value;
}

/*void mem_copy(byte* host, ushrt guest, ushrt size)
{
    for(int i = 0; i < size; i++)
    {
        mem_set8(guest+i, host[i]);
    }
}*/

byte mem_get8(ushrt addr)
{
    return mem[addr];
}
short mem_get16(ushrt addr)
{
    byte b0 = mem_get8(addr);
    byte b1 = mem_get8(addr+1);
    return (b1<<8) | (b0 & 0xff);
}
int mem_get32(ushrt addr)
{
    short n0 = mem_get16(addr);
    short n1 = mem_get16(addr+2);
    return (n1<<16) | (n0 & USHRT_MAX);
}
