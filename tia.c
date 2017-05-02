#include <stdlib.h>
#include <stdio.h>

#include "mem.h"
#include "tia.h"

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
    }

    return 0;
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
        default:
            printf("TIA: unknown write addr 0x%x val 0x%x\n", addr, value);
    }
}
