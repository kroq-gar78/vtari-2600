#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

#include "cpu.h"
#include "mem.h"
#include "tia.h"

ushrt cart_size = CART_SIZE_ATARI;
ushrt cart_start;

byte riot_mem[RAM_SIZE];
byte* cart_mem;
byte pia_mem[8];

void mem_init()
{
    pia_mem[SWCHA] = 0xFF;
    pia_mem[SWCHB] = 0b00111111;

    if(args.start_given)
    {
        cart_start = args.start_arg;

        // pad trailing unused bytes with 0's
        int cart_end = (int)cart_start + cart_size;
        if(cart_end < MEM_MAX)
        {
            int new_size = MEM_MAX - (int)cart_start;

            // realloc logic based off of:
            // https://stackoverflow.com/a/21006795
            byte* tmp = realloc(cart_mem, new_size);
            if(cart_mem == NULL)
            {
                fprintf(stderr, "Realloc failed\n");
                free(cart_mem);
                exit(1);
            }
            else
            {
                cart_mem = tmp;
            }

            // memset gives us unitialized memory, so set new bytes to 0
            memset(cart_mem+cart_size, 0, new_size - cart_size);
        }
    }
    else
    {
#ifdef ATARI_2600
        cart_start = CART_START_ATARI;
#else
        cart_start = MEM_MAX - cart_size;
#endif
    }
}

//void mem_map(ushrt guest, ushrt count);

void mem_set8(ushrt addr, byte value)
{
    // regardless of settings, should be guaranteed to be a power of 2
    addr &= (MEM_MAX-1);

#ifdef ATARI_2600
    // TIA mirrors
    if((addr & 0x1080) == 0)
    {
        addr &= 0b01111111;
        tia_write(addr, value);
        return;
    }
    // PIA RAM mirrors
    else if((addr & 0x1280) == 0x80)
#elif defined(MOS_6502)
    // in the general case (MOS 6502), everything that isn't cartridge/ROM is RAM
    if(addr < cart_start)
#endif
    {
#ifdef ATARI_2600
        addr &= RAM_SIZE-1;
#endif
        riot_mem[addr] = value;
    }
#ifdef ATARI_2600
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
                timer_underflow = false;
                pia_mem[INSTAT] &= ~(1<<7);
                printfv("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
            case TIM8T:
                pia_mem[INTIM] = value;
                timer_int = 8;
                timer_underflow = false;
                pia_mem[INSTAT] &= ~(1<<7);
                printfv("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
            case TIM64T:
                pia_mem[INTIM] = value;
                timer_int = 64;
                timer_underflow = false;
                pia_mem[INSTAT] &= ~(1<<7);
                printfv("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
            case T1024T:
                pia_mem[INTIM] = value;
                timer_int = 1024;
                timer_underflow = false;
                pia_mem[INSTAT] &= ~(1<<7);
                printfv("PIA timer set val %d timer_int %d pc %x\n", value, timer_int, pc);
                break;
        }

        if(addr != INSTAT)
        {
            pia_mem[addr] = value;
        }
    }
#endif
    // cartridge mirrors
    else if(addr >= cart_start)
    {
#ifdef MOS_6502_TEST
        // make ROM writeable for tests
        // TODO: needs commandline option
        addr -= cart_start;
        cart_mem[addr] = value;
#else
        MISSING();
#endif
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
    //printfv("addr %x\n", addr);

#ifdef ATARI_2600
    // TIA mirrors
    if((addr & 0x1080) == 0)
    {
        addr &= 0b01111111;
        //printfv("TIA: read from 0x%x\n", addr);
        return tia_read(addr);
    }
    // PIA RAM mirrors
    else if((addr & 0x1280) == 0x80)
#elif defined(MOS_6502)
    // in the general case (MOS 6502), everything that isn't cartridge/ROM is RAM
    if(addr < cart_start)
#endif
    {
#ifdef ATARI_2600
        addr &= RAM_SIZE-1;
#endif
        return riot_mem[addr];
    }
#ifdef ATARI_2600
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
            timer_underflow = false; // apparently resume the old interval once read
            return pia_mem[INTIM];
        }
        return pia_mem[addr];
    }
#endif
    // cartridge mirrors
    else if(addr >= cart_start)
    {
#ifdef ATARI_2600
        addr &= (cart_size-1);
#else
        addr -= cart_start; // no memory mirroring, doesn't assume power of 2
#endif
        //printfv("ROM addr %x\n", addr);
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

    return (b1<<8) | (b0 & 0xff);
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
        if(i%16 == 0) printfv("\n");
        printfv("%02x ", riot_mem[i]);
    }
    printfv("\n");
}

// writes as a CSV
// address,value
void write_ram(FILE* file)
{
#ifdef ATARI_2600
    int length = RAM_SIZE;
#else
    int length = cart_start; // since all memory below ROM is RAM
#endif
    for(int i = 0; i < length; i++)
    {
        fprintf(file, "0x%04x,%02x\n", (RAM_START+i), riot_mem[i]);
    }
    fprintf(file, "\n");
}

// based off of: https://stackoverflow.com/a/11574035
void write_ram_path(char* path)
{
    fprintf(stderr, "Writing RAM to '%s'\n", path);
    FILE* f = fopen(path, "w");
    if(f == NULL)
    {
        fprintf(stderr, "Unable to open file for writing RAM\n");
        exit(1);
    }

    write_ram(f);
    fclose(f);
}
