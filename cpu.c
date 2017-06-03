#include "cpu.h"
#include "fps.h"
#include "mem.h"
#include "tia.h"

ushrt pc = 0x1000;
ushrt next_pc = 0x1002;
byte reg_a = 0;
byte reg_x = 0;
byte reg_y = 0;
byte sp = 0xff;
byte reg_p = 0x20; // unused bit is 1
int timer_int = 1; // timer interval
bool timer_underflow = false;
bool cpu_halted = false;

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

// length of an instruction (in bytes) with a given address mode
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

// from ruby2600: lib/ruby2600/cpu.rb
int opcodes_cycles[256] =
 {7, 6, 0, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  6, 6, 0, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  6, 6, 0, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
  2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  6, 6, 0, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
  2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
  2, 6, 0, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
  2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
  2, 5, 0, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
  2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 0, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7};

unsigned int cpu_cycles_left;
unsigned int cycle;

// for more about these operations, see:
// http://www.pagetable.com/?p=410
ushrt vec_nmi;
ushrt vec_reset;
ushrt vec_irq; // also for BRK (vec_brk)

void cpu_init()
{
    vec_nmi = mem_get16(0xfffa);
    vec_reset = mem_get16(0xfffc);
    vec_irq = mem_get16(0xfffe);

    if(args.entry_given) pc = args.entry_arg;
    else pc = vec_reset;
    printfv("entrypoint 0x%x\n", pc);

    cpu_cycles_left = opcodes_cycles[mem_get8(pc)];
    cycle = 0;
}

// CPU EXECUTION
// also handles TIA and SDL rendering
void cpu_exec()
{
    SDL_Event event;

    while(opcodes[mem_get8(pc)] != inst_brk)
    {
        tia_tick();

        // first, process the interval timer
        if(cycle%3 == 0)
        {
            if(cycle%(3*(timer_underflow ? 1 : timer_int)) == 0)
            {
                if(pia_mem[INTIM] == 0) // underflow
                {
                    timer_underflow = true;
                    pia_mem[INSTAT] |= (1<<7) | (1<<6);
                    //printfv("PIA timer overflow\n");
                }
                //printfv("timer int %d cycle %d preval %d\n", timer_int, cycle, pia_mem[INTIM]);
                pia_mem[INTIM] -= 1;
            }
        }

        // tick the CPU
        if((cycle%3 == 0) && !cpu_halted && (--cpu_cycles_left <= 0))
        {
            byte inst_opcode = mem_get8(pc);
            int addr_mode = addr_modes[inst_opcode];
            printfv("pc %x inst %x tia_x %d tia_y %d\n", pc, mem_get8(pc), tia_x, tia_y);
            printfv("A %x X %x Y %x\n", reg_a, reg_x, reg_y);
            next_pc = pc + addr_mode_len[addr_mode];
            cpu_cycles_left = opcodes_cycles[inst_opcode];
            printfv("SWCHA 0x%02x INPT4 0x%02x IPT5 0x%02x\n", pia_mem[SWCHA], tia_mem[INPT4], tia_mem[INPT5]);

            // fetch and execute
            F2 inst = opcodes[inst_opcode];
            (*inst)(pc, addr_mode);

            //printfv("len %d\n", addr_mode_len[addr_mode]);
            pc = next_pc;
        }

        if((cycle % (NTSC_HEIGHT*NTSC_WIDTH)) == 0)
        {
            fpsthink();
            draw_frame();
            frame_num++;
            SDL_Delay(17); // 17 ms ~= 60Hz; guarantee max 60Hz framerate
        }

        // render once every full frame drawn; multiply by 3 to guarantee its divisibility by 3
        // multiply by 1024 for divisibility for the interval timer
        cycle = (cycle + 1)%(NTSC_HEIGHT*NTSC_WIDTH*3*1024);
        if(SDL_PollEvent(&event) && (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)))
        {
            break;
        }
        else if(event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
        {
            tia_handleKeyboard(&event.key);
        }
    }
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
bool setflag_n_direct(bool status) // negative
{
    if(status)
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

// the "core" functionality of various instructions
int _adc(byte a, byte b)
{
    if(reg_p & FLAGS_DECIMAL)
    {
        a = hex_to_bcd(a);
        b = hex_to_bcd(b);
    }
    byte c = (reg_p & FLAGS_CARRY) != 0;
    ushrt result_ushrt = a+b+c;
    byte result = a+b+c;

    // check overflow
    if(result_ushrt >= USHRT_MAX)
    {
        setflag_c_direct(true);
        setflag_v_direct(true);
    }
    else
    {
        reg_p &= (~FLAGS_CARRY) & (~FLAGS_OVERFLOW);
    }

    setflag_nz(result);

    if(reg_p & FLAGS_DECIMAL)
    {
        result = bcd_to_hex(result);
    }
    return result;
}

int _and(byte a, byte b)
{
    byte result = a&b;

    setflag_nz(result);

    return result;
}

// `b` is the number of shifts
int _asl(byte a, byte b)
{
    byte result = a<<b;

    setflag_nz(result);

    // carry flag
    setflag_c_direct( (result>>(8-b)) != 0 );

    return result;
}

int _bit(byte a, byte b)
{
    byte result = a&b;

    setflag_n_direct(b & FLAGS_NEGATIVE);
    setflag_v_direct(b & FLAGS_OVERFLOW);
    setflag_z(result);

    return result;
}

// register in `a`
int _cmp(byte a, byte b)
{
    byte result = a >= b;

    setflag_c_direct(a >= b); // TODO: is this a signed comparison?
    setflag_nz(a-b);

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
    if(reg_p & FLAGS_DECIMAL)
    {
        a = hex_to_bcd(a);
        b = hex_to_bcd(b);
    }

    byte c_inv = ((reg_p & FLAGS_CARRY) == 0);
    byte result = a - b - c_inv; // need ~C (https://www.dwheeler.com/6502/oneelkruns/asm1step.html)

    // TODO: overflow, carry flags
    setflag_nz(result);

    if(reg_p & FLAGS_DECIMAL)
    {
        result = bcd_to_hex(result);
    }

    setflag_v_direct((result>>7) != (a>>7)); // set V on sign change
    setflag_c_direct((char)a >= (char)((char)b+c_inv));

    return result;
}

int _transfer(byte* a, byte* b)
{
    *b = *a;

    setflag_nz(*a);

    return *a;
}


// stack operations
ushrt _push(byte val)
{
    if((sp & 0xff) == 0x80)
    {
        fprintf(stderr, "Stack overrun (pull)\n");
    }
    mem_set8(sp, val);
    sp--;

    return sp;
}
ushrt _push16(ushrt val) // for pushing the PC
{
    _push(val>>8);
    _push(val&0xff);
    return sp;
}

byte _pull() // like pop?
{
    if((sp & 0xff) == 0xff)
    {
        fprintf(stderr, "Stack overrun (pull)\n");
    }
    byte result = mem_get8(++sp);

    return result;
}
ushrt _pull16()
{
    ushrt ret = _pull(); // lower byte
    ret |= (_pull() << 8); // higher byte
    return ret;
}


// interrupts
// for more on interrupts, see: http://www.pagetable.com/?p=410
ushrt _brk()
{
    _push16(pc);
    _push(reg_p | FLAGS_BREAK);
    next_pc = vec_irq;
    return next_pc;
}
ushrt _irq()
{
    _push16(pc);
    _push(reg_p & ~FLAGS_BREAK);
    next_pc = vec_irq;
    return next_pc;
}
ushrt _nmi()
{
    _push16(pc);
    _push(reg_p & ~FLAGS_BREAK);
    next_pc = vec_nmi;
    return next_pc;
}

// addressing modes
// they take in the address of the start of the operands
ushrt addr_abs(ushrt addr)
{
    /*ushrt b0 = mem_get8(addr);
    ushrt b1 = mem_get8(addr);

    return (b1<<8) | (b0 & 0xff);*/
    return mem_get16(addr);
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
    return mem_get16_zpg(mem_get8(addr))+reg_y; // TODO: carry?
}
ushrt addr_rel(ushrt addr)
{
    short rel = (char)mem_get8(addr);
    return next_pc + rel;
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

    bool take = (reg_p & FLAGS_CARRY) == 0;
    if(take)
    {
        next_pc = addr_e;
    }
    return take;
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

    bool take = (reg_p & FLAGS_CARRY) != 0;
    if(take)
    {
        next_pc = addr_e;
    }
    return take;
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

    bool take = (reg_p & FLAGS_ZERO) != 0;
    if(take)
    {
        next_pc = addr_e;
    }
    return take;
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

    bool take = (reg_p & FLAGS_NEGATIVE) != 0;
    if(take)
    {
        next_pc = addr_e;
    }
    printfv("BMI take %d next_pc %x\n", take, next_pc);
    return take;
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

    bool take = (reg_p & FLAGS_ZERO) == 0;
    if(take)
    {
        next_pc = addr_e;
    }
    return take;
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

    bool take = (reg_p & FLAGS_NEGATIVE) == 0;
    if(take)
    {
        next_pc = addr_e;
    }
    printfv("bpl take %d target %x\n", take, addr_e);
    return take;
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

    _brk();
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

    bool take = (reg_p & FLAGS_OVERFLOW) == 0;
    if(take)
    {
        next_pc = addr_e;
    }
    return take;
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

    bool take = (reg_p & FLAGS_OVERFLOW) != 0;
    if(take)
    {
        next_pc = addr_e;
    }
    return take;
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

    printfv("jmp to %x addr_e %x\n", val_16, addr_e);
    next_pc = addr_e;
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

    _push16(next_pc);
    printfv("jsr to %x addr_e %x\n", val_16, addr_e);
    next_pc = addr_e;
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

    //printfv("LDA pc %x val %x\n", pc, );
    setflag_nz(val_8);
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

    setflag_nz(val_8);
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

    setflag_nz(val_8);
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

    _push(reg_a);
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

    _push(reg_p);
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

    reg_a = _pull();
    setflag_nz(reg_a);
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

    reg_p = _pull();
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

    next_pc = _pull16();
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
    printfv("STA addr 0x%x\n", addr);
    printfv("STA addr_e 0x%x\n", addr_e);

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
