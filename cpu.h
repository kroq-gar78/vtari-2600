#ifndef CPU_H
#define CPU_H

#include <stdlib.h>

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

// from: pa (kernel/vmm.c)
#define MISSING() do { \
    printf("%s:%d is missing\n", __FILE__, __LINE__); \
    exit(1); \
} while (0)

// debug from: http://stackoverflow.com/a/1644898
#define debug_print(fmt, ...) \
    do { if (DEBUG_TEST) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

typedef unsigned char byte;
typedef unsigned short ushrt;
typedef int (*F1)(short a0); // from p6

#define FLAGS_CARRY (byte)(1<<0)
#define FLAGS_ZERO (byte)(1<<1)
#define FLAGS_IRQ_DISABLE (byte)(1<<2)
#define FLAGS_DECIMAL (byte)(1<<3)
#define FLAGS_BREAK (byte)(1<<4)
#define FLAGS_OVERFLOW (byte)(1<<6)
#define FLAGS_NEGATIVE (byte)(1<<7)

#define OPCODE_BAD 0x777

extern int pc;
extern byte reg_a;
extern byte reg_x;
extern byte reg_y;
extern byte sp;
extern byte reg_p; // status register

#endif
