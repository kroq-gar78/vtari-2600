#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

#include "cpu.h"
#include "mem.h"
#include "tia.h"

byte riot_mem[RAM_SIZE];
byte cart_mem[CART_SIZE];
byte pia_mem[8];

void mem_init()
{
}

//void mem_map(ushrt guest, ushrt count);

void mem_set8(ushrt addr, byte value)
{
    addr &= (MEM_MAX-1);

    // TIA mirrors
    if((addr & 0x1080) == 0)
    {
        // TODO: implement
        //MISSING();
        addr &= 0b01111111;
        tia_write(addr, value);
        return;
    }
    // PIA RAM mirrors
    else if((addr & 0x1280) == 0x80)
    {
        addr &= RAM_SIZE-1;
        riot_mem[addr] = value;
    }
    // PIA I/O mirrors
    else if((addr & 0x1280) == 0x280)
    {
        addr &= 8-1;

        switch(addr)
        {
            case SWCHA:
                break;
            case SWACNT:
                break;
            case SWCHB:
                break;
            case SWBCNT:
                break;
            case TIM1T:
                pia_mem[INTIM] = value;
                timer_int = 1;
                pia_mem[INSTAT] &= ~(1<<7);
                printf("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
            case TIM8T:
                pia_mem[INTIM] = value;
                timer_int = 8;
                pia_mem[INSTAT] &= ~(1<<7);
                printf("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
            case TIM64T:
                pia_mem[INTIM] = value;
                timer_int = 64;
                pia_mem[INSTAT] &= ~(1<<7);
                printf("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
            case T1024T:
                pia_mem[INTIM] = value;
                timer_int = 1024;
                pia_mem[INSTAT] &= ~(1<<7);
                printf("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
        }

        if(addr != INSTAT)
        {
            pia_mem[addr] = value;
        }
    }
    // cartridge mirrors
    else if((addr & 0x1000) == 0x1000)
    {
        MISSING();
    }
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
    //printf("addr %x\n", addr);

    // TIA mirrors
    if((addr & 0x1080) == 0)
    {
        // TODO: implement
        //MISSING();
        addr &= 0b01111111;
        //printf("TIA: read from 0x%x\n", addr);
        return tia_read(addr);
    }
    // PIA RAM mirrors
    else if((addr & 0x1280) == 0x80)
    {
        addr &= RAM_SIZE-1;
        return riot_mem[addr];
    }
    // PIA I/O mirrors
    else if((addr & 0x1280) == 0x280)
    {
        addr &= 8-1;
        //if(addr == INSTAT)
        if(addr == 5 || addr == 7) // INSTAT mirroring
        {
            byte old = pia_mem[addr];
            pia_mem[INSTAT] &= ~(1<<6);
            return old;
        }
        if(addr == 4 || addr == 6) // INTIM mirroring
        {
            return pia_mem[INTIM];
        }
        return pia_mem[addr];
    }
    // cartridge mirrors
    else if((addr & 0x1000) == 0x1000)
    {
        addr &= 0xfff;
        //printf("ROM addr %x\n", addr);
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

// convert hex number to packed BCD
byte hex_to_bcd(byte value)
{
    // we can assume that each nibble is < 10
    return (value>>4)*10 + (value & 0xf);
}

// convert from a hex representation of a number to packed BCD
byte bcd_to_hex(byte bcd)
{
    // we assume the number is < 100
    return ((bcd/10)<<4) + (bcd%10);
}

void print_ram()
{
    for(int i = 0; i < RAM_SIZE; i++)
    {
        if(i%16 == 0) printf("\n");
        printf("%02x ", riot_mem[i]);
    }
    printf("\n");
}
