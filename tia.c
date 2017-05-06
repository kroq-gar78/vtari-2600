#include <stdlib.h>
#include <stdio.h>
#include <math.h>

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

// from: http://stackoverflow.com/a/2603254
static byte reverse_lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };
byte bitwise_reverse(byte n) {
   // Reverse the top and bottom nibble then swap them.
   return (reverse_lookup[n&0b1111] << 4) | reverse_lookup[n>>4];
}

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
            //printf("TIA: VSYNC at (%d,%d)\n", tia_x, tia_y);
            if(value & (1<<1))
            {
                tia_x = 0;
                tia_y = 0;
            }
            break;
        case VBLANK:
            //if(value != 0) printf("TIA: VBLANK at (%d,%d)\n", tia_x, tia_y);
            break;
        case WSYNC:
            cpu_halted = true;
            tia_state |= TIA_STATE_WSYNC;
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
            //printf("TIA: write PF0 val %x\n", value);
            printf("TIA: pc 0x%x val %x y %d\n", pc, value, tia_y);
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

    // if a sprite reset, use the memory location to store the starting x-coordinate
    if(addr != RESP0 && addr != RESP1 && addr != RESM0 && addr != RESM1 && addr != RESBL)
    {
        tia_mem[addr] = value;
    }
    else
    {
        tia_mem[addr] = tia_x;
    }
}

void tia_init()
{
    tia_state = TIA_STATE_NORMAL;
    tia_x = 0;
    tia_y = 0;
}

// pos = reset sprite; data = sprite data; color = color; ref = reflect
void sprite_draw(byte pos, byte data, byte color, byte ref)
{
    int cycles_since = tia_x - pos;
    int bit;
    if(cycles_since >= 0 && cycles_since < 8) // render the sprite if we're at the horiz. position
    {
        if(ref & (1<<3))
        {
            bit = cycles_since;
        }
        else
        {
            bit = 8-cycles_since-1;
        }

        if(data & 1<<(bit))
        {
            tia_display[tia_y][tia_x] = color;
            //printf("TIA: RESP0\n");
        }
    }
}

void tia_tick()
{
    // correct for the blanks and overscan
    int x_ctrd = tia_x-DISPLAY_H_START;
    int y_ctrd = tia_y-DISPLAY_V_START;

    // BACKGROUND
    byte bgcolor = tia_mem[COLUBK];
    if(tia_mem[VBLANK] & 2) // TODO: for all graphics, or just BG?
    {
        bgcolor = tia_mem[0];
        //printf("TIA VBLANK tia_y %d\n", tia_y);
    }
    tia_display[tia_y][tia_x] = bgcolor;

    // PLAYFIELD
    // TODO: normalize pixels so that they only render in the middle part of the screen
    uint64_t pf = (bitwise_reverse(tia_mem[PF0])<<16) | (tia_mem[PF1]<<8) | bitwise_reverse(tia_mem[PF2]);
    int bit = 0;
    int x_pf_pixel = x_ctrd/PF_PIXEL;
    if(x_pf_pixel < (PF_WIDTH>>1) || (tia_mem[CTRLPF] & 1) == 0)
    {
        bit = (PF_WIDTH>>1)-x_pf_pixel%20-1; // off by one error!!!
    }
    else // this happens if we're in the 2nd half, and mirroring/reflecting is on
    {
        bit = x_pf_pixel%20;
    }
    //printf("TIA: pf bits 0x%lx y_ctrd %d x_ctrd %d\n", pf&0xfffff, y_ctrd, x_ctrd);
    if(tia_y < DISPLAY_H_END && tia_y > 220)
    {
        printf("TIA: playfield (%d, %d) pf 0x%lx TIA_STATE %x\n", tia_x, tia_y, pf, tia_state);
        //if(tia_state & TIA_STATE_RESP0) printf("(%d, %d) RESP0\n", tia_x, tia_y);
    }

    if(1<<bit & pf)
    {
        if(tia_mem[CTRLPF] & (1<<1)) // left half COLUP0, right half COLUP1
        {
            MISSING();
        }
        else // all PF same color
        {
            tia_display[tia_y][tia_x] = tia_mem[COLUPF];
        }
        if(tia_y < DISPLAY_H_END && tia_y > 220) printf("TIA: playfield (%d, %d)\n", tia_x, tia_y);
    }

    if(tia_mem[CTRLPF] & (1<<2)) // playfield priority over sprites
    {
        MISSING();
    }

    
    // SPRITES
    // TODO: NUSIZx (number and size of players)
    sprite_draw(tia_mem[RESP0], tia_mem[GRP0], tia_mem[COLUP0], tia_mem[REFP0]);
    sprite_draw(tia_mem[RESP1], tia_mem[GRP1], tia_mem[COLUP1], tia_mem[REFP1]);
    byte m0_size = (tia_mem[ENAM0] & 2) ? (tia_mem[NUSIZ0]>>4)&(0b11) : 0;
    byte m1_size = (tia_mem[ENAM1] & 2) ? (tia_mem[NUSIZ1]>>4)&(0b11) : 0;
    sprite_draw(tia_mem[RESM0], (1<<m0_size)-1, tia_mem[COLUP0], 0);
    sprite_draw(tia_mem[RESM1], (1<<m1_size)-1, tia_mem[COLUP1], 0);
    byte ball_size = (tia_mem[ENABL] & 2) ? (tia_mem[CTRLPF]>>4)&(0b11) : 0;
    sprite_draw(tia_mem[RESBL], (1<<ball_size)-1, tia_mem[COLUPF], 0); // TODO: color of ball

    // update TIA beam position
    tia_x = (tia_x+1)%NTSC_WIDTH;
    if(tia_x == 0) // new scanline
    {
        tia_y++;
        if(tia_state & TIA_STATE_WSYNC)
        {
            cpu_halted = false;
            //tia_state = TIA_STATE_NORMAL;
            tia_state &= ~TIA_STATE_WSYNC;
        }
        // if we were rendering sprite, stop rendering
        //tia_state &= (~TIA_STATE_RESP0 & ~TIA_STATE_RESP1 & ~TIA_STATE_RESM0 & ~TIA_STATE_RESM1 & TIA_STATE_RESBL);
    }
    tia_y %= NTSC_HEIGHT;
}
