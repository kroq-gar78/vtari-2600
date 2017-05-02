#include <stdio.h>
#include <sys/mman.h> // mmap
#include <sys/stat.h> // filesize

#include <fcntl.h> // file control

#include "cpu.h"
#include "mem.h"

int pc = 0x1000;
byte reg_a = 0;
byte reg_x = 0;
byte reg_y = 0;
byte sp = 0;
byte reg_p = 0x20; // unused bit is 1
byte* mmap_p; // pointer to mmap'd file

ushrt addr_abs(ushrt addr);
ushrt addr_abs_x(ushrt addr);
ushrt addr_abs_y(ushrt addr);
ushrt addr_ind(ushrt addr);
ushrt addr_x_ind(ushrt addr);
ushrt addr_ind_y(ushrt addr);
ushrt addr_rel(ushrt addr);
ushrt addr_zpg(ushrt addr);
ushrt addr_zpg_x(ushrt addr);
ushrt addr_zpg_y(ushrt addr);
ushrt addr_imm(ushrt addr); // for compatibility
ushrt addr_impl(ushrt addr); // for compatibility

// map of address mode to function pointer
F1 addr_mode_f[12] = {addr_abs, addr_abs_x, addr_abs_y, addr_ind, addr_x_ind, addr_ind_y, addr_rel, addr_zpg, addr_zpg_x, addr_zpg_y, addr_imm, addr_impl};

// len of an instruction with a given address mode
ushrt addr_mode_len[12] = {3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1};

short inst_adc(ushrt addr, int addr_mode);
short inst_and(ushrt addr, int addr_mode);
short inst_asl(ushrt addr, int addr_mode);
short inst_bcc(ushrt addr, int addr_mode);
short inst_bcs(ushrt addr, int addr_mode);
short inst_beq(ushrt addr, int addr_mode);
short inst_bit(ushrt addr, int addr_mode);
short inst_bmi(ushrt addr, int addr_mode);
short inst_bne(ushrt addr, int addr_mode);
short inst_bpl(ushrt addr, int addr_mode);
short inst_brk(ushrt addr, int addr_mode);
short inst_bvc(ushrt addr, int addr_mode);
short inst_bvs(ushrt addr, int addr_mode);
short inst_clc(ushrt addr, int addr_mode);
short inst_cld(ushrt addr, int addr_mode);
short inst_cli(ushrt addr, int addr_mode);
short inst_clv(ushrt addr, int addr_mode);
short inst_cmp(ushrt addr, int addr_mode);
short inst_cpx(ushrt addr, int addr_mode);
short inst_cpy(ushrt addr, int addr_mode);
short inst_dec(ushrt addr, int addr_mode);
short inst_dex(ushrt addr, int addr_mode);
short inst_dey(ushrt addr, int addr_mode);
short inst_eor(ushrt addr, int addr_mode);
short inst_inc(ushrt addr, int addr_mode);
short inst_inx(ushrt addr, int addr_mode);
short inst_iny(ushrt addr, int addr_mode);
short inst_jmp(ushrt addr, int addr_mode);
short inst_jsr(ushrt addr, int addr_mode);
short inst_lda(ushrt addr, int addr_mode);
short inst_ldx(ushrt addr, int addr_mode);
short inst_ldy(ushrt addr, int addr_mode);
short inst_lsr(ushrt addr, int addr_mode);
short inst_nop(ushrt addr, int addr_mode);
short inst_ora(ushrt addr, int addr_mode);
short inst_pha(ushrt addr, int addr_mode);
short inst_php(ushrt addr, int addr_mode);
short inst_pla(ushrt addr, int addr_mode);
short inst_plp(ushrt addr, int addr_mode);
short inst_rol(ushrt addr, int addr_mode);
short inst_ror(ushrt addr, int addr_mode);
short inst_rti(ushrt addr, int addr_mode);
short inst_rts(ushrt addr, int addr_mode);
short inst_sbc(ushrt addr, int addr_mode);
short inst_sec(ushrt addr, int addr_mode);
short inst_sed(ushrt addr, int addr_mode);
short inst_sei(ushrt addr, int addr_mode);
short inst_sta(ushrt addr, int addr_mode);
short inst_stx(ushrt addr, int addr_mode);
short inst_sty(ushrt addr, int addr_mode);
short inst_tax(ushrt addr, int addr_mode);
short inst_tay(ushrt addr, int addr_mode);
short inst_tsx(ushrt addr, int addr_mode);
short inst_txa(ushrt addr, int addr_mode);
short inst_txs(ushrt addr, int addr_mode);
short inst_tya(ushrt addr, int addr_mode);


F2 opcodes[256] = {inst_brk, inst_ora, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_ora, inst_asl, (F2) OPCODE_BAD, inst_php, inst_ora, inst_asl, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_ora, inst_asl, (F2) OPCODE_BAD,
inst_bpl, inst_ora, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_ora, inst_asl, (F2) OPCODE_BAD, inst_clc, inst_ora, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_ora, inst_asl, (F2) OPCODE_BAD,
inst_jsr, inst_and, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_bit, inst_and, inst_rol, (F2) OPCODE_BAD, inst_plp, inst_and, inst_rol, (F2) OPCODE_BAD, inst_bit, inst_and, inst_rol, (F2) OPCODE_BAD,
inst_bmi, inst_and, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_and, inst_rol, (F2) OPCODE_BAD, inst_sec, inst_and, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_and, inst_rol, (F2) OPCODE_BAD,
inst_rti, inst_eor, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_eor, inst_lsr, (F2) OPCODE_BAD, inst_pha, inst_eor, inst_lsr, (F2) OPCODE_BAD, inst_jmp, inst_eor, inst_lsr, (F2) OPCODE_BAD,
inst_bvc, inst_eor, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_eor, inst_lsr, (F2) OPCODE_BAD, inst_cli, inst_eor, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_eor, inst_lsr, (F2) OPCODE_BAD,
inst_rts, inst_adc, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_adc, inst_ror, (F2) OPCODE_BAD, inst_pla, inst_adc, inst_ror, (F2) OPCODE_BAD, inst_jmp, inst_adc, inst_ror, (F2) OPCODE_BAD,
inst_bvs, inst_adc, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_adc, inst_ror, (F2) OPCODE_BAD, inst_sei, inst_adc, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_adc, inst_ror, (F2) OPCODE_BAD,
(F2) OPCODE_BAD, inst_sta, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_sty, inst_sta, inst_stx, (F2) OPCODE_BAD, inst_dey, (F2) OPCODE_BAD, inst_txa, (F2) OPCODE_BAD, inst_sty, inst_sta, inst_stx, (F2) OPCODE_BAD,
inst_bcc, inst_sta, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_sty, inst_sta, inst_stx, (F2) OPCODE_BAD, inst_tya, inst_sta, inst_txs, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_sta, (F2) OPCODE_BAD, (F2) OPCODE_BAD,
inst_ldy, inst_lda, inst_ldx, (F2) OPCODE_BAD, inst_ldy, inst_lda, inst_ldx, (F2) OPCODE_BAD, inst_tay, inst_lda, inst_tax, (F2) OPCODE_BAD, inst_ldy, inst_lda, inst_ldx, (F2) OPCODE_BAD,
inst_bcs, inst_lda, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_ldy, inst_lda, inst_ldx, (F2) OPCODE_BAD, inst_clv, inst_lda, inst_tsx, (F2) OPCODE_BAD, inst_ldy, inst_lda, inst_ldx, (F2) OPCODE_BAD,
inst_cpy, inst_cmp, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_cpy, inst_cmp, inst_dec, (F2) OPCODE_BAD, inst_iny, inst_cmp, inst_dex, (F2) OPCODE_BAD, inst_cpy, inst_cmp, inst_dec, (F2) OPCODE_BAD,
inst_bne, inst_cmp, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_cmp, inst_dec, (F2) OPCODE_BAD, inst_cld, inst_cmp, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_cmp, inst_dec, (F2) OPCODE_BAD,
inst_cpx, inst_sbc, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_cpx, inst_sbc, inst_inc, (F2) OPCODE_BAD, inst_inx, inst_sbc, inst_nop, (F2) OPCODE_BAD, inst_cpx, inst_sbc, inst_inc, (F2) OPCODE_BAD,
inst_beq, inst_sbc, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_sbc, inst_inc, (F2) OPCODE_BAD, inst_sed, inst_sbc, (F2) OPCODE_BAD, (F2) OPCODE_BAD, (F2) OPCODE_BAD, inst_sbc, inst_inc, (F2) OPCODE_BAD};

int addr_modes[256] = {ADDRMODE_IMPL, ADDRMODE_X_IND, -1, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, -1, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, -1, -1, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, -1,
ADDRMODE_ABS, ADDRMODE_X_IND, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, ADDRMODE_ABS, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, -1, -1, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, -1,
ADDRMODE_IMPL, ADDRMODE_X_IND, -1, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, ADDRMODE_ABS, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, -1, -1, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, -1,
ADDRMODE_IMPL, ADDRMODE_X_IND, -1, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, ADDRMODE_IND, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, -1, -1, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, -1,
-1, ADDRMODE_X_IND, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, -1, ADDRMODE_IMPL, -1, ADDRMODE_ABS, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, ADDRMODE_ZPG_Y, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, ADDRMODE_IMPL, -1, -1, ADDRMODE_ABS_X, -1, -1,
ADDRMODE_IMM, ADDRMODE_X_IND, ADDRMODE_IMM, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, ADDRMODE_ABS, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, ADDRMODE_ZPG_Y, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, ADDRMODE_IMPL, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, ADDRMODE_ABS_Y, -1,
ADDRMODE_IMM, ADDRMODE_X_IND, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, ADDRMODE_ABS, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, -1, -1, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, -1,
ADDRMODE_IMM, ADDRMODE_X_IND, -1, -1, ADDRMODE_ZPG, ADDRMODE_ZPG, ADDRMODE_ZPG, -1, ADDRMODE_IMPL, ADDRMODE_IMM, ADDRMODE_IMPL, -1, ADDRMODE_ABS, ADDRMODE_ABS, ADDRMODE_ABS, -1,
ADDRMODE_REL, ADDRMODE_IND_Y, -1, -1, -1, ADDRMODE_ZPG_X, ADDRMODE_ZPG_X, -1, ADDRMODE_IMPL, ADDRMODE_ABS_Y, -1, -1, -1, ADDRMODE_ABS_X, ADDRMODE_ABS_X, -1};


struct stat st;
byte* ldr_mmap_file(char *filename)
{
    stat(filename, &st);

    int fd = open(filename, O_RDONLY);

    byte* p = mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);

    // pick the minimum of the max cartrige size, and the filesize
    int length = st.st_size < CART_SIZE ? st.st_size : CART_SIZE;
    for(int i = 0; i < length; i++)
    {
        cart_mem[i] = p[i];
    }

    // mirror onto second half if it's <=2K
    if(length <= CART_SIZE/2)
    {
        for(int i = 0; i < length; i++)
        {
            cart_mem[i+2048] = p[i];
        }
    }

    if(p == MAP_FAILED)
    {
        fprintf(stderr, "mmap failed\n");
        perror("mmap");
    }
    return p;
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        fprintf(stderr, "Not enough arguments\n");
        exit(1);
    }

    mmap_p = ldr_mmap_file(argv[1]);
    

    // EXECUTION
    while(opcodes[mem_get8(pc)] != inst_brk)
    {
        printf("pc %x inst %x\n", pc, mem_get8(pc));
        pc += addr_mode_len[mem_get8(pc)];
    }



    if(munmap(mmap_p, st.st_size) == -1)
    {
        perror("mmap");
        exit(1);
    }

    return 0;
}



// flag setter helper methods
bool setflag_z(byte num)
{
    if(num == 0)
    {
        reg_p |= FLAGS_ZERO;
        return true;
    }
    else
    {
        reg_p &= ~FLAGS_ZERO;
        return false;
    }
}
bool setflag_n(byte num)
{
    if(((num >> 7) & 1) == 1)
    {
        reg_p |= FLAGS_NEGATIVE;
        return true;
    }
    else
    {
        reg_p &= ~FLAGS_NEGATIVE;
        return false;
    }
}
bool setflag_nz(byte num)
{
    return setflag_n(num) | setflag_z(num);
}
// just set the flag to whatever you want (nothing smart like above)
bool setflag_c_direct(bool status) // carry
{
    if(status)
    {
        reg_p |= FLAGS_CARRY;
        return true;
    }
    else
    {
        reg_p &= ~FLAGS_CARRY;
        return false;
    }
}
bool setflag_v_direct(bool status) // overflow
{
    if(status)
    {
        reg_p |= FLAGS_OVERFLOW;
        return true;
    }
    else
    {
        reg_p &= ~FLAGS_OVERFLOW;
        return false;
    }
}

int _adc(byte a, byte b)
{
    if(reg_p & FLAGS_DECIMAL) MISSING();
    byte result = a+b;

    // check overflow
    if(result >= USHRT_MAX)
    {
        reg_p |= FLAGS_CARRY | FLAGS_OVERFLOW;
    }
    else
    {
        reg_p &= (~FLAGS_CARRY) & (~FLAGS_OVERFLOW);
    }

    setflag_z(result);

    return result;
}

int _and(byte a, byte b)
{
    byte result = a&b;

    setflag_n(result);
    setflag_z(result);

    return result;
}

// `b` is the number of shifts
int _asl(byte a, byte b)
{
    byte result = a<<b;

    setflag_n(result);
    setflag_z(result);

    // carry flag
    setflag_c_direct( (result>>(8-b)) != 0 );

    return result;
}

int _bit(byte a, byte b)
{
    byte result = a&b;

    reg_p &= (~FLAGS_NEGATIVE) & (~FLAGS_OVERFLOW);
    reg_p |= (b & FLAGS_NEGATIVE);
    reg_p |= (b & FLAGS_OVERFLOW);

    setflag_z(result);

    return result;
}

// register in `a`
int _cmp(byte a, byte b)
{
    byte result = a >= b;

    setflag_c_direct(a >= b); // TODO: is this a signed comparison?
    setflag_z(a-b);
    setflag_n(a);

    return result;
}

int _dec(byte a)
{
    byte result = a-1;

    setflag_nz(result);

    return result;
}

int _eor(byte a, byte b)
{
    byte result = a^b;

    setflag_nz(result);

    return result;
}

int _inc(byte a)
{
    byte result = a+1;

    setflag_nz(result);

    return result;
}

int _lsr(byte a)
{
    byte result = (a>>1);

    setflag_z(result);
    setflag_c_direct(a&1);

    return result;
}

int _or(byte a, byte b)
{
    byte result = a|b;

    setflag_nz(result);

    return result;
}

int _rol(byte a)
{
    byte result = a<<1;
    result &= (byte)(~1);
    result |= ((reg_p & FLAGS_CARRY) != 0);

    setflag_c_direct(result>>7);

    return result;
}

int _ror(byte a)
{
    byte result = a>>1;
    result &= ((reg_p & FLAGS_CARRY) != 0) << 7;
    
    setflag_c_direct(result&1);

    return result;
}

int _sbc(byte a, byte b)
{
    if(reg_p & FLAGS_DECIMAL) MISSING();

    byte result = a - b - ((reg_p & FLAGS_CARRY) != 0);

    // TODO: overflow, carry flags
    setflag_nz(result);

    return result;
}

int _transfer(byte* a, byte* b)
{
    *a = *b;

    setflag_nz(*a);

    return *a;
}

// addressing modes
// they take in the address of the start of the operands
ushrt addr_abs(ushrt addr)
{
    ushrt b0 = mem_get8(addr);
    ushrt b1 = mem_get8(addr);

    return (b1<<8) | (b0 & 0xff);
}
ushrt addr_abs_x(ushrt addr)
{
    return addr_abs(addr) + reg_x;
}
ushrt addr_abs_y(ushrt addr)
{
    return addr_abs(addr) + reg_y;
}
ushrt addr_ind(ushrt addr)
{
    return mem_get16(mem_get16(addr));
}
// TODO: carry?
ushrt addr_x_ind(ushrt addr)
{
    return mem_get16_zpg(mem_get8(addr)+reg_x);
}
ushrt addr_ind_y(ushrt addr)
{
    return mem_get16_zpg(mem_get8(addr))+reg_y;
}
ushrt addr_rel(ushrt addr)
{
    short rel = mem_get8(addr);
    return pc + rel;
}
ushrt addr_zpg(ushrt addr)
{
    return mem_get8(addr);
}
ushrt addr_zpg_x(ushrt addr)
{
    return (mem_get8(addr)+reg_x) & 0xff;
}
ushrt addr_zpg_y(ushrt addr)
{
    return (mem_get8(addr)+reg_y) & 0xff;
}
ushrt addr_imm(ushrt addr) // for compatibility
{
    return addr;
}
ushrt addr_impl(ushrt addr) // for compatibility
{
    return addr;
}

// instructions
short inst_adc(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = _adc(reg_a, val_8);
    return 0;
}
short inst_and(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = _and(reg_a, val_8);
    return 0;
}
short inst_asl(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = _asl(reg_a, 1);
    return 0;
}
short inst_bcc(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_bcs(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_beq(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_bit(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _bit(reg_a, val_8);
    return 0;
}
short inst_bmi(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_bne(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_bpl(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_brk(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_bvc(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_bvs(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_clc(ushrt addr, int addr_mode)
{
    reg_p &= (~FLAGS_CARRY);
    return 0;
}
short inst_cld(ushrt addr, int addr_mode)
{
    reg_p &= (~FLAGS_DECIMAL);
    return 0;
}
short inst_cli(ushrt addr, int addr_mode)
{
    reg_p &= (~FLAGS_IRQ_DISABLE);
    return 0;
}
short inst_clv(ushrt addr, int addr_mode)
{
    reg_p &= (~FLAGS_OVERFLOW);
    return 0;
}
short inst_cmp(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _cmp(reg_a, val_8);
    return 0;
}
short inst_cpx(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _cmp(reg_x, val_8);
    return 0;
}
short inst_cpy(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _cmp(reg_y, val_8);
    return 0;
}
short inst_dec(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    mem_set8(addr_e, _dec(val_8));
    return 0;
}
short inst_dex(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_x = _dec(reg_x);
    return 0;
}
short inst_dey(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_y = _dec(reg_y);
    return 0;
}
short inst_eor(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = _eor(reg_a, val_8);
    return 0;
}
short inst_inc(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    mem_set8(addr_e, _inc(val_8));
    return 0;
}
short inst_inx(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_x = _inc(reg_x);
    return 0;
}
short inst_iny(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_y = _inc(reg_y);
    return 0;
}
short inst_jmp(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_jsr(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_lda(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = val_8;
    return 0;
}
short inst_ldx(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_x = val_8;
    return 0;
}
short inst_ldy(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_y = val_8;
    return 0;
}
short inst_lsr(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    if(addr_mode == ADDRMODE_IMPL)
    {
        reg_a = _lsr(reg_a);
    }
    else
    {
        mem_set8(addr_e, _lsr(val_8));
    }
    return 0;
}
short inst_nop(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    return 0;
}
short inst_ora(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = _or(reg_a, val_8);
    return 0;
}
short inst_pha(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_php(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_pla(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_plp(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_rol(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    if(addr_mode == ADDRMODE_IMPL)
    {
        reg_a = _rol(reg_a);
    }
    else
    {
        mem_set8(addr_e, _rol(val_8));
    }

    return 0;
}
short inst_ror(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    if(addr_mode == ADDRMODE_IMPL)
    {
        reg_a = _ror(reg_a);
    }
    else
    {
        mem_set8(addr_e, _ror(val_8));
    }
    return 0;
}
short inst_rti(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_rts(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    MISSING();
    return 0;
}
short inst_sbc(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    reg_a = _sbc(reg_a, val_8);
    return 0;
}
short inst_sec(ushrt addr, int addr_mode)
{
    reg_p |= FLAGS_CARRY;
    return 0;
}
short inst_sed(ushrt addr, int addr_mode)
{
    reg_p |= FLAGS_DECIMAL;
    return 0;
}
short inst_sei(ushrt addr, int addr_mode)
{
    reg_p |= FLAGS_IRQ_DISABLE;
    return 0;
}
short inst_sta(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    mem_set8(addr_e, reg_a);
    return 0;
}
short inst_stx(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    mem_set8(addr_e, reg_x);
    return 0;
}
short inst_sty(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    mem_set8(addr_e, reg_y);
    return 0;
}
short inst_tax(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _transfer(&reg_a, &reg_x);
    return 0;
}
short inst_tay(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _transfer(&reg_a, &reg_y);
    return 0;
}
short inst_tsx(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _transfer(&sp, &reg_x);
    return 0;
}
short inst_txa(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _transfer(&reg_x, &reg_a);
    return 0;
}
short inst_txs(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _transfer(&reg_x, &sp);
    return 0;
}
short inst_tya(ushrt addr, int addr_mode)
{
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr+1);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    }
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    _transfer(&reg_y, &reg_a);
    return 0;
}
