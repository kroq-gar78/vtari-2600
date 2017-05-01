#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

#include "cpu.h"
#include "mem.h"

byte pia_mem[RAM_SIZE];
byte cart_mem[CART_SIZE];
byte tia_mem[TIA_SIZE];

void mem_init()
{
}

//void mem_map(ushrt guest, ushrt count);

void mem_set8(ushrt addr, byte value)
{
    /*mem[addr] = value;*/
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
    // all memory is mirrored every 0x2000 bytes
    addr &= (MEM_MAX-1);

    // TIA mirrors
    if((addr & 0x1080) == 0)
    {
        MISSING();
    }
    // PIA RAM mirrors
    else if((addr & 0x1280) == 0x80)
    {
        addr &= RAM_SIZE-1;
        return pia_mem[addr];
    }
    // PIA I/O mirrors
    else if((addr & 0x1280) == 0x280)
    {
        MISSING();
    }
    // cartridge mirrors
    else if((addr & 0x2000) == 0x2000)
    {
        addr &= 0xfff;
        return cart_mem[addr];
    }

    return addr;
}
short mem_get16(ushrt addr)
{
    // all memory is mirrored every 0x2000 bytes
    addr &= (MEM_MAX-1);

    byte b0 = mem_get8(addr);
    byte b1 = mem_get8(addr+1);

    return (b1<<8) | (b0 & 0xff);
}

// because zero-page addressing can wrap around, we need a separate method
short mem_get16_zpg(ushrt addr)
{
    addr &= 0xff; // get the lower byte (because zero-page)

    byte b0 = mem_get8(addr);
    byte b1 = mem_get8((addr+1)&0xff);

    return (b1<<0) | (b0 & 0xff);
}

int mem_get32(ushrt addr)
{
    short n0 = mem_get16(addr);
    short n1 = mem_get16(addr+2);
    return (n1<<16) | (n0 & USHRT_MAX);
}
