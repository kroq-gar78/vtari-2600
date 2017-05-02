#include <stdlib.h>
#include <stdio.h>

#include "mem.h"
#include "tia.h"

void tia_write(ushrt addr, byte value)
{
    printf("TIA: write to 0x%x val %x\n", addr, value);

    // regex (based off of `#define`s in `mem.h`):
    // s/^.*\ \([0-9A-Z]\+\)\ .*$/case \1:\r\tbreak;/g
    switch(value)
    {
        case VSYNC:
            break;
        case VBLANK:
            break;
        case WSYNC:
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
    }
}
