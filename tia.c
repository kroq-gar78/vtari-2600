#include <stdlib.h>
#include <stdio.h>

#include "mem.h"
#include "tia.h"

int tia_state;
int tia_x = 0;
int tia_y = 0;

byte tia_mem[TIA_SIZE];
byte tia_display[NTSC_HEIGHT][NTSC_WIDTH];


// from javatari.js: src/main/atari/tia/TiaPalettes.js
// divide the color by two, then index into this array
// it's not actually RGB; it's "BGR" (from left to right)
int ntsc_rgb[128] = {0x000000, 0x404040, 0x6c6c6c, 0x909090, 0xb0b0b0, 0xc8c8c8, 0xdcdcdc, 0xf4f4f4, 0x004444, 0x106464, 0x248484, 0x34a0a0, 0x40b8b8, 0x50d0d0, 0x5ce8e8, 0x68fcfc, 0x002870, 0x144484, 0x285c98, 0x3c78ac, 0x4c8cbc, 0x5ca0cc, 0x68b4dc, 0x78c8ec, 0x001884, 0x183498, 0x3050ac, 0x4868c0, 0x5c80d0, 0x7094e0, 0x80a8ec, 0x94bcfc, 0x000088, 0x20209c, 0x3c3cb0, 0x5858c0, 0x7070d0, 0x8888e0, 0xa0a0ec, 0xb4b4fc, 0x5c0078, 0x74208c, 0x883ca0, 0x9c58b0, 0xb070c0, 0xc084d0, 0xd09cdc, 0xe0b0ec, 0x780048, 0x902060, 0xa43c78, 0xb8588c, 0xcc70a0, 0xdc84b4, 0xec9cc4, 0xfcb0d4, 0x840014, 0x982030, 0xac3c4c, 0xc05868, 0xd0707c, 0xe08894, 0xeca0a8, 0xfcb4bc, 0x880000, 0x9c201c, 0xb04038, 0xc05c50, 0xd07468, 0xe08c7c, 0xeca490, 0xfcb8a4, 0x7c1800, 0x90381c, 0xa85438, 0xbc7050, 0xcc8868, 0xdc9c7c, 0xecb490, 0xfcc8a4, 0x5c2c00, 0x784c1c, 0x906838, 0xac8450, 0xc09c68, 0xd4b47c, 0xe8cc90, 0xfce0a4, 0x2c3c00, 0x485c1c, 0x647c38, 0x809c50, 0x94b468, 0xacd07c, 0xc0e490, 0xd4fca4, 0x003c00, 0x205c20, 0x407c40, 0x5c9c5c, 0x74b474, 0x8cd08c, 0xa4e4a4, 0xb8fcb8, 0x003814, 0x1c5c34, 0x387c50, 0x50986c, 0x68b484, 0x7ccc9c, 0x90e4b4, 0xa4fcc8, 0x00302c, 0x1c504c, 0x347068, 0x4c8c84, 0x64a89c, 0x78c0b4, 0x88d4cc, 0x9cece0, 0x002844, 0x184864, 0x306884, 0x4484a0, 0x589cb8, 0x6cb4d0, 0x7ccce8, 0x8ce0fc};

byte tia_read(ushrt addr)
{
    switch(addr)
    {
        case CXM0P:
            break;
        case CXM1P:
            break;
        case CXP0FB:
            break;
        case CXP1FB:
            break;
        case CXM0FB:
            break;
        case CXM1FB:
            break;
        case CXBLPF:
            break;
        case CXPPMM:
            break;
        case INPT0:
            break;
        case INPT1:
            break;
        case INPT2:
            break;
        case INPT3:
            break;
        case INPT4:
            break;
        default:
            printf("TIA: unknown read addr 0x%x\n", addr);
            return 0;
    }

    return tia_mem[addr];
}

void tia_write(ushrt addr, byte value)
{
    printf("TIA: write to 0x%x val %x\n", addr, value);

    // regex (based off of `#define`s in `mem.h`):
    // s/^.*\ \([0-9A-Z]\+\)\ .*$/case \1:\r\tbreak;/g
    switch(addr)
    {
        case VSYNC:
            break;
        case VBLANK:
            if(value != 0) printf("TIA: VBLANK at (%d,%d)\n", tia_x, tia_y);
            break;
        case WSYNC:
            cpu_halted = true;
            tia_state = TIA_STATE_WSYNC;
            break;
        case RSYNC:
            break;
        case NUSIZ0:
            break;
        case NUSIZ1:
            break;
        case COLUP0:
            break;
        case COLUP1:
            break;
        case COLUPF:
            break;
        case COLUBK:
            break;
        case CTRLPF:
            break;
        case REFP0:
            break;
        case REFP1:
            break;
        case PF0:
            break;
        case PF1:
            break;
        case PF2:
            break;
        case RESP0:
            break;
        case RESP1:
            break;
        case RESM0:
            break;
        case RESM1:
            break;
        case RESBL:
            break;
        case AUDC0:
            break;
        case AUDC1:
            break;
        case AUDF0:
            break;
        case AUDF1:
            break;
        case AUDV0:
            break;
        case AUDV1:
            break;
        case GRP0:
            break;
        case GRP1:
            break;
        case ENAM0:
            break;
        case ENAM1:
            break;
        case ENABL:
            break;
        case HMP0:
            break;
        case HMP1:
            break;
        case HMM0:
            break;
        case HMM1:
            break;
        case HMBL:
            break;
        case VDELP0:
            break;
        case VDELP1:
            break;
        case VDELBL:
            break;
        case RESMP0:
            break;
        case RESMP1:
            break;
        case HMOVE:
            break;
        case HMCLR:
            break;
        case CXCLR:
            break;
        default:
            printf("TIA: unknown write addr 0x%x val 0x%x\n", addr, value);
            return;
    }
    tia_mem[addr] = value;
}

void tia_init()
{
    tia_state = TIA_STATE_NORMAL;
    tia_x = 0;
    tia_y = 0;
}

void tia_tick()
{
    // do rendering things
    // asdfadsf
    // asdfasdf
    //MISSING();
    
    // TODO: more things than just background
    byte bgcolor = tia_mem[COLUBK];
    if(tia_mem[VBLANK])
    {
        bgcolor = tia_mem[0];
    }
    tia_display[tia_y][tia_x] = bgcolor;

    tia_x = (tia_x+1)%NTSC_WIDTH;
    // new scanline
    if(tia_x == 0)
    {
        tia_y++;
        if(tia_state == TIA_STATE_WSYNC)
        {
            cpu_halted = false;
            tia_state = TIA_STATE_NORMAL;
        }
    }
    tia_y %= NTSC_HEIGHT;
}
