#ifndef TIA_H
#define TIA_H

#include <stdint.h>

#include "cpu.h"
#include "mem.h"

// TIA write mappings
// from: http://problemkaputt.de/2k6specs.htm#memoryandiomap
#define VSYNC 0x00
#define VBLANK 0x01
#define WSYNC 0x02
#define RSYNC 0x03
#define NUSIZ0 0x04
#define NUSIZ1 0x05
#define COLUP0 0x06
#define COLUP1 0x07
#define COLUPF 0x08
#define COLUBK 0x09
#define CTRLPF 0x0A
#define REFP0 0x0B
#define REFP1 0x0C
#define PF0 0x0D
#define PF1 0x0E
#define PF2 0x0F
#define RESP0 0x10
#define RESP1 0x11
#define RESM0 0x12
#define RESM1 0x13
#define RESBL 0x14
#define AUDC0 0x15
#define AUDC1 0x16
#define AUDF0 0x17
#define AUDF1 0x18
#define AUDV0 0x19
#define AUDV1 0x1A
#define GRP0 0x1B
#define GRP1 0x1C
#define ENAM0 0x1D
#define ENAM1 0x1E
#define ENABL 0x1F
#define HMP0 0x20
#define HMP1 0x21
#define HMM0 0x22
#define HMM1 0x23
#define HMBL 0x24
#define VDELP0 0x25
#define VDELP1 0x26
#define VDELBL 0x27
#define RESMP0 0x28
#define RESMP1 0x29
#define HMOVE 0x2A
#define HMCLR 0x2B
#define CXCLR 0x2C

// TIA read mappings
#define CXM0P 0x30
#define CXM1P 0x31
#define CXP0FB 0x32
#define CXP1FB 0x33
#define CXM0FB 0x34
#define CXM1FB 0x35
#define CXBLPF 0x36
#define CXPPMM 0x37
#define INPT0 0x38
#define INPT1 0x39
#define INPT2 0x3A
#define INPT3 0x3B
#define INPT4 0x3C

#define NTSC_WIDTH 228
#define NTSC_HEIGHT 262
#define WINDOW_ZOOM 2

#define TIA_STATE_NORMAL 0
#define TIA_STATE_WSYNC 1

extern int tia_state;
extern int tia_x;
extern int tia_y;

extern byte tia_mem[TIA_SIZE];
extern byte tia_display[NTSC_HEIGHT][NTSC_WIDTH];
extern int ntsc_rgb[128];

extern byte tia_read(ushrt addr);
extern void tia_write(ushrt addr, byte value);

extern void tia_init();
extern void tia_tick();

#endif
