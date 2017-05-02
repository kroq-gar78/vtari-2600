#include <stdio.h>
#include <sys/mman.h> // mmap
#include <sys/stat.h> // filesize

#include <fcntl.h> // file control

#include "cpu.h"
#include "mem.h"

int pc;
byte reg_a;
byte reg_x;
byte reg_y;
byte sp;
byte reg_p = 0x20;
byte* mmap_p; // pointer to mmap'd file

int adc_abs(short addr);
int adc_abs_x(short addr);
int adc_abs_y(short addr);
int adc_imm(short addr);
int adc_ind_y(short addr);
int adc_x_ind(short addr);
int adc_zpg(short addr);
int adc_zpg_x(short addr);
int and_abs(short addr);
int and_abs_x(short addr);
int and_abs_y(short addr);
int and_imm(short addr);
int and_ind_y(short addr);
int and_x_ind(short addr);
int and_zpg(short addr);
int and_zpg_x(short addr);
int asl_a(short addr);
int asl_abs(short addr);
int asl_abs_x(short addr);
int asl_zpg(short addr);
int asl_zpg_x(short addr);
int bcc_rel(short addr);
int bcs_rel(short addr);
int beq_rel(short addr);
int bit_abs(short addr);
int bit_zpg(short addr);
int bmi_rel(short addr);
int bne_rel(short addr);
int bpl_rel(short addr);
int brk_impl(short addr);
int bvc_rel(short addr);
int bvs_rel(short addr);
int clc_impl(short addr);
int cld_impl(short addr);
int cli_impl(short addr);
int clv_impl(short addr);
int cmp_abs(short addr);
int cmp_abs_x(short addr);
int cmp_abs_y(short addr);
int cmp_imm(short addr);
int cmp_ind_y(short addr);
int cmp_x_ind(short addr);
int cmp_zpg(short addr);
int cmp_zpg_x(short addr);
int cpx_abs(short addr);
int cpx_imm(short addr);
int cpx_zpg(short addr);
int cpy_abs(short addr);
int cpy_imm(short addr);
int cpy_zpg(short addr);
int dec_abs(short addr);
int dec_abs_x(short addr);
int dec_zpg(short addr);
int dec_zpg_x(short addr);
int dex_impl(short addr);
int dey_impl(short addr);
int eor_abs(short addr);
int eor_abs_x(short addr);
int eor_abs_y(short addr);
int eor_imm(short addr);
int eor_ind_y(short addr);
int eor_x_ind(short addr);
int eor_zpg(short addr);
int eor_zpg_x(short addr);
int inc_abs(short addr);
int inc_abs_x(short addr);
int inc_zpg(short addr);
int inc_zpg_x(short addr);
int inx_impl(short addr);
int iny_impl(short addr);
int jmp_abs(short addr);
int jmp_ind(short addr);
int jsr_abs(short addr);
int lda_abs(short addr);
int lda_abs_x(short addr);
int lda_abs_y(short addr);
int lda_imm(short addr);
int lda_ind_y(short addr);
int lda_x_ind(short addr);
int lda_zpg(short addr);
int lda_zpg_x(short addr);
int ldx_abs(short addr);
int ldx_abs_y(short addr);
int ldx_imm(short addr);
int ldx_zpg(short addr);
int ldx_zpg_y(short addr);
int ldy_abs(short addr);
int ldy_abs_x(short addr);
int ldy_imm(short addr);
int ldy_zpg(short addr);
int ldy_zpg_x(short addr);
int lsr_a(short addr);
int lsr_abs(short addr);
int lsr_abs_x(short addr);
int lsr_zpg(short addr);
int lsr_zpg_x(short addr);
int nop_impl(short addr);
int ora_abs(short addr);
int ora_abs_x(short addr);
int ora_abs_y(short addr);
int ora_imm(short addr);
int ora_ind_y(short addr);
int ora_x_ind(short addr);
int ora_zpg(short addr);
int ora_zpg_x(short addr);
int pha_impl(short addr);
int php_impl(short addr);
int pla_impl(short addr);
int plp_impl(short addr);
int rol_a(short addr);
int rol_abs(short addr);
int rol_abs_x(short addr);
int rol_zpg(short addr);
int rol_zpg_x(short addr);
int ror_a(short addr);
int ror_abs(short addr);
int ror_abs_x(short addr);
int ror_zpg(short addr);
int ror_zpg_x(short addr);
int rti_impl(short addr);
int rts_impl(short addr);
int sbc_abs(short addr);
int sbc_abs_x(short addr);
int sbc_abs_y(short addr);
int sbc_imm(short addr);
int sbc_ind_y(short addr);
int sbc_x_ind(short addr);
int sbc_zpg(short addr);
int sbc_zpg_x(short addr);
int sec_impl(short addr);
int sed_impl(short addr);
int sei_impl(short addr);
int sta_abs(short addr);
int sta_abs_x(short addr);
int sta_abs_y(short addr);
int sta_ind_y(short addr);
int sta_x_ind(short addr);
int sta_zpg(short addr);
int sta_zpg_x(short addr);
int stx_abs(short addr);
int stx_zpg(short addr);
int stx_zpg_y(short addr);
int sty_abs(short addr);
int sty_zpg(short addr);
int sty_zpg_x(short addr);
int tax_impl(short addr);
int tay_impl(short addr);
int tsx_impl(short addr);
int txa_impl(short addr);
int txs_impl(short addr);
int tya_impl(short addr);


F1 opcodes[256] = {brk_impl, ora_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, ora_zpg, asl_zpg, (F1) OPCODE_BAD, php_impl, ora_imm, asl_a, (F1) OPCODE_BAD, (F1) OPCODE_BAD, ora_abs, asl_abs, (F1) OPCODE_BAD, 
bpl_rel, ora_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, ora_zpg_x, asl_zpg_x, (F1) OPCODE_BAD, clc_impl, ora_abs_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, ora_abs_x, asl_abs_x, (F1) OPCODE_BAD, 
jsr_abs, and_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, bit_zpg, and_zpg, rol_zpg, (F1) OPCODE_BAD, plp_impl, and_imm, rol_a, (F1) OPCODE_BAD, bit_abs, and_abs, rol_abs, (F1) OPCODE_BAD, 
bmi_rel, and_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, and_zpg_x, rol_zpg_x, (F1) OPCODE_BAD, sec_impl, and_abs_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, and_abs_x, rol_abs_x, (F1) OPCODE_BAD, 
rti_impl, eor_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, eor_zpg, lsr_zpg, (F1) OPCODE_BAD, pha_impl, eor_imm, lsr_a, (F1) OPCODE_BAD, jmp_abs, eor_abs, lsr_abs, (F1) OPCODE_BAD, 
bvc_rel, eor_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, eor_zpg_x, lsr_zpg_x, (F1) OPCODE_BAD, cli_impl, eor_abs_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, eor_abs_x, lsr_abs_x, (F1) OPCODE_BAD, 
rts_impl, adc_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, adc_zpg, ror_zpg, (F1) OPCODE_BAD, pla_impl, adc_imm, ror_a, (F1) OPCODE_BAD, jmp_ind, adc_abs, ror_abs, (F1) OPCODE_BAD, 
bvs_rel, adc_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, adc_zpg_x, ror_zpg_x, (F1) OPCODE_BAD, sei_impl, adc_abs_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, adc_abs_x, ror_abs_x, (F1) OPCODE_BAD, 
(F1) OPCODE_BAD, sta_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, sty_zpg, sta_zpg, stx_zpg, (F1) OPCODE_BAD, dey_impl, (F1) OPCODE_BAD, txa_impl, (F1) OPCODE_BAD, sty_abs, sta_abs, stx_abs, (F1) OPCODE_BAD, 
bcc_rel, sta_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, sty_zpg_x, sta_zpg_x, stx_zpg_y, (F1) OPCODE_BAD, tya_impl, sta_abs_y, txs_impl, (F1) OPCODE_BAD, (F1) OPCODE_BAD, sta_abs_x, (F1) OPCODE_BAD, (F1) OPCODE_BAD, 
ldy_imm, lda_x_ind, ldx_imm, (F1) OPCODE_BAD, ldy_zpg, lda_zpg, ldx_zpg, (F1) OPCODE_BAD, tay_impl, lda_imm, tax_impl, (F1) OPCODE_BAD, ldy_abs, lda_abs, ldx_abs, (F1) OPCODE_BAD, 
bcs_rel, lda_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, ldy_zpg_x, lda_zpg_x, ldx_zpg_y, (F1) OPCODE_BAD, clv_impl, lda_abs_y, tsx_impl, (F1) OPCODE_BAD, ldy_abs_x, lda_abs_x, ldx_abs_y, (F1) OPCODE_BAD, 
cpy_imm, cmp_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, cpy_zpg, cmp_zpg, dec_zpg, (F1) OPCODE_BAD, iny_impl, cmp_imm, dex_impl, (F1) OPCODE_BAD, cpy_abs, cmp_abs, dec_abs, (F1) OPCODE_BAD, 
bne_rel, cmp_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, cmp_zpg_x, dec_zpg_x, (F1) OPCODE_BAD, cld_impl, cmp_abs_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, cmp_abs_x, dec_abs_x, (F1) OPCODE_BAD, 
cpx_imm, sbc_x_ind, (F1) OPCODE_BAD, (F1) OPCODE_BAD, cpx_zpg, sbc_zpg, inc_zpg, (F1) OPCODE_BAD, inx_impl, sbc_imm, nop_impl, (F1) OPCODE_BAD, cpx_abs, sbc_abs, inc_abs, (F1) OPCODE_BAD, 
beq_rel, sbc_ind_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, sbc_zpg_x, inc_zpg_x, (F1) OPCODE_BAD, sed_impl, sbc_abs_y, (F1) OPCODE_BAD, (F1) OPCODE_BAD, (F1) OPCODE_BAD, sbc_abs_x, inc_abs_x, (F1) OPCODE_BAD};

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

int adc(byte a, byte b)
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

int and(byte a, byte b)
{
    byte result = a&b;

    setflag_n(result);
    setflag_z(result);

    return result;
}

// `b` is the number of shifts
int asl(byte a, byte b)
{
    byte result = a<<b;

    setflag_n(result);
    setflag_z(result);

    // carry flag
    setflag_c_direct( (result>>(8-b)) != 0 );

    return result;
}

int bit(byte a, byte b)
{
    byte result = a&b;

    reg_p &= (~FLAGS_NEGATIVE) & (~FLAGS_OVERFLOW);
    reg_p |= (b & FLAGS_NEGATIVE);
    reg_p |= (b & FLAGS_OVERFLOW);

    setflag_z(result);

    return result;
}

// register in `a`
int cmp(byte a, byte b)
{
    byte result = a >= b;

    setflag_c_direct(a >= b); // TODO: is this a signed comparison?
    setflag_z(a-b);
    setflag_n(a);

    return result;
}

int dec(byte a)
{
    byte result = a-1;

    setflag_nz(result);

    return result;
}

int eor(byte a, byte b)
{
    byte result = a^b;

    setflag_nz(result);

    return result;
}

int inc(byte a)
{
    byte result = a+1;

    setflag_nz(result);

    return result;
}

int lsr(byte a)
{
    byte result = (a>>1);

    setflag_z(result);
    setflag_c_direct(a&1);

    return result;
}

int or(byte a, byte b)
{
    byte result = a|b;

    setflag_nz(result);

    return result;
}

int rol(byte a)
{
    byte result = a<<1;
    result &= (byte)(~1);
    result |= ((reg_p & FLAGS_CARRY) != 0);

    setflag_c_direct(result>>7);

    return result;
}

int ror(byte a)
{
    byte result = a>>1;
    result &= ((reg_p & FLAGS_CARRY) != 0) << 7;
    
    setflag_c_direct(result&1);

    return result;
}

int sbc(byte a, byte b)
{
    if(reg_p & FLAGS_DECIMAL) MISSING();

    byte result = a - b - ((reg_p & FLAGS_CARRY) != 0);

    // TODO: overflow, carry flags
    setflag_nz(result);

    return result;
}

int transfer(byte* a, byte* b)
{
    *a = *b;

    setflag_nz(*a);

    return *a;
}

// addressing modes
// they take in the address of the start of the operands
ushrt addr_abs(short addr)
{
    ushrt b0 = mem_get8(addr);
    ushrt b1 = mem_get8(addr);

    return (b1<<8) | (b0 & 0xff);
}
ushrt addr_abs_x(short addr)
{
    return addr_abs(addr) + reg_x;
}
ushrt addr_abs_y(short addr)
{
    return addr_abs(addr) + reg_y;
}
ushrt addr_ind(short addr)
{
    return mem_get16(mem_get16(addr));
}
// TODO: carry?
ushrt addr_x_ind(short addr)
{
    return mem_get16_zpg(mem_get8(addr)+reg_x);
}
ushrt addr_ind_y(short addr)
{
    return mem_get16_zpg(mem_get8(addr))+reg_y;
}
ushrt addr_rel(short addr)
{
    short rel = mem_get8(addr);
    return pc + rel;
}
ushrt addr_zpg(short addr)
{
    return mem_get8(addr);
}
ushrt addr_zpg_x(short addr)
{
    return (mem_get8(addr)+reg_x) & 0xff;
}
ushrt addr_zpg_y(short addr)
{
    return (mem_get8(addr)+reg_y) & 0xff;
}
ushrt addr_imm(short addr) // for compatibility
{
    return addr;
}
ushrt addr_impl(short addr) // for compatibility
{
    return addr;
}

// vim regex: s/;/\r{\r    return 0;\r}/g
int brk_impl(short addr)
{
    return 0;
}
int clc_impl(short addr)
{
    reg_p &= (~FLAGS_CARRY);
    return 0;
}
int cld_impl(short addr)
{
    reg_p &= (~FLAGS_DECIMAL);
    return 0;
}
int cli_impl(short addr)
{
    reg_p &= (~FLAGS_IRQ_DISABLE);
    return 0;
}
int clv_impl(short addr)
{
    reg_p &= (~FLAGS_OVERFLOW);
    return 0;
}
int dex_impl(short addr)
{
    return 0;
}
int dey_impl(short addr)
{
    return 0;
}
int inx_impl(short addr)
{
    return 0;
}
int iny_impl(short addr)
{
    return 0;
}
int nop_impl(short addr)
{
    return 0;
}
int pha_impl(short addr)
{
    return 0;
}
int php_impl(short addr)
{
    return 0;
}
int pla_impl(short addr)
{
    return 0;
}
int plp_impl(short addr)
{
    return 0;
}
int rti_impl(short addr)
{
    return 0;
}
int rts_impl(short addr)
{
    return 0;
}
int sec_impl(short addr)
{
    reg_p |= FLAGS_CARRY;
    return 0;
}
int sed_impl(short addr)
{
    reg_p |= FLAGS_DECIMAL;
    return 0;
}
int sei_impl(short addr)
{
    reg_p |= FLAGS_IRQ_DISABLE;
    return 0;
}
int tax_impl(short addr)
{
    return 0;
}
int tay_impl(short addr)
{
    return 0;
}
int tsx_impl(short addr)
{
    return 0;
}
int txa_impl(short addr)
{
    return 0;
}
int txs_impl(short addr)
{
    return 0;
}
int tya_impl(short addr)
{
    return 0;
}
