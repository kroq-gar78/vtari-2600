#ifndef CPU_H
#define CPU_H

#include <stdlib.h>

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmdline.h"

// from: pa (kernel/vmm.c)
#define MISSING() do { \
    printf("%s:%d is missing\n", __FILE__, __LINE__); \
    exit(1); \
} while (0)

// verbose printing, based on the 'verbose_on' variable
// from: https://stackoverflow.com/a/10017285
#define printfv(format, ...) do { \
    if (verbose_on) \
        printf(format, ##__VA_ARGS__); \
} while (0)

// debug from: http://stackoverflow.com/a/1644898
#define debug_print(fmt, ...) \
    do { if (DEBUG_TEST) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

typedef unsigned char byte;
typedef unsigned short ushrt;
typedef ushrt (*F1)(ushrt a0); // from p6
typedef short (*F2)(ushrt a0, int addr_mode); // from p6

#define FLAGS_CARRY (byte)(1<<0)
#define FLAGS_ZERO (byte)(1<<1)
#define FLAGS_IRQ_DISABLE (byte)(1<<2)
#define FLAGS_DECIMAL (byte)(1<<3)
#define FLAGS_BREAK (byte)(1<<4)
#define FLAGS_OVERFLOW (byte)(1<<6)
#define FLAGS_NEGATIVE (byte)(1<<7)

#define OPCODE_BAD 0x777

#define ADDRMODE_ABS 0
#define ADDRMODE_ABS_X 1
#define ADDRMODE_ABS_Y 2
#define ADDRMODE_IND 3
#define ADDRMODE_X_IND 4
#define ADDRMODE_IND_Y 5
#define ADDRMODE_REL 6
#define ADDRMODE_ZPG 7
#define ADDRMODE_ZPG_X 8
#define ADDRMODE_ZPG_Y 9
#define ADDRMODE_IMM 10
#define ADDRMODE_IMPL 11

// CPU registers
extern ushrt pc;
extern byte reg_a;
extern byte reg_x;
extern byte reg_y;
extern byte sp;
extern byte reg_p; // status register

extern unsigned int cycle; // current cycle number
extern unsigned int cpu_cycles_left; // cycles left in current instruction

extern int timer_int; // timer interval
extern bool timer_underflow;

extern bool cpu_halted;

extern void cpu_init();
extern void cpu_exec();

// graphics-related things
// these really should really be in a separate header
extern unsigned int frame_num;
extern void draw_frame();
extern struct gengetopt_args_info args;
extern bool verbose_on;

#endif
