#ifndef CPU_H
#define CPU_H

#include <limits.h>
#include <stdlib.h>

#define FLAGS_CARRY 1<<0
#define FLAGS_ZERO 1<<1
#define FLAGS_IRQ_DISABLE 1<<2
#define FLAGS_DECIMAL 1<<3
#define FLAGS_BREAK 1<<4
#define FLAGS_OVERFLOW 1<<6
#define FLAGS_NEGATIVE 1<<7

#define OPCODE_BAD 0x777

typedef unsigned char byte;
typedef int (*F1)(short a0); // from p6

extern int pc;
extern byte reg_a;
extern byte reg_x;
extern byte reg_y;
extern byte sp;
extern byte reg_p; // status register

#endif
