#!/usr/bin/env python2

# print out a parsed table of instructions

from collections import defaultdict
import re
import sys

f = open("inst.txt",'r')
s = f.read().strip()

lines = s.split('\n')

# get rid of first row and first column (opcodes)
lines_sep = [map(lambda y: y.split(), x.split('\t')[1:]) for x in lines[1:]]

# print lines_sep

d = defaultdict(set)
addr_modes = set()

opcodes_table = []
addrmodes_table = []
sums = 0

# print
# sys.stdout.write('F0[16][16] opcodes = {')
for a in xrange(0x10):
    l_opcodes = []
    l_addrmodes = []
    for b in xrange(0x10):
        inst, addr = lines_sep[a][b]
        inst = inst.lower()
        addr = re.sub('#','imm',addr)
        addr = re.sub(',','_',addr)
        addr = addr.lower()


        if inst == "???" or "addr" == "---":
            l_opcodes.append("(F2) OPCODE_BAD")
            l_addrmodes.append("-1");
            continue

        if(addr == "a"): addr = "impl"
        d[inst].add(addr)
        addr_modes.add(addr)

        # l.append("%s_%s" % (inst, addr))
        l_opcodes.append("inst_%s" % inst)
        l_addrmodes.append("ADDRMODE_%s" % addr.upper())
    # print "row", len(l)
    opcodes_table.append(', '.join(l_opcodes))
    addrmodes_table.append(', '.join(l_addrmodes))
    sums += len(l_opcodes)

opcodes_table_str = ', \n'.join(opcodes_table)
addrmodes_table_str = ', \n'.join(addrmodes_table)

print
print
# print(sums)

for inst in sorted(d.iterkeys()):
    print "short inst_%s(ushrt addr, int addr_mode);" % inst

print
print

print "F2 opcodes[256] = {%s};" % opcodes_table_str
print
print "int addr_modes[256] = {%s};" % addrmodes_table_str

print
print

for inst in sorted(d.iterkeys()):
    print "int inst_%s(short addr, int addr_mode)" % inst
    print """
    F1 addr_f = addr_mode_f[addr_mode];
    ushrt addr_e = addr_f(addr);

    // get operands, using zero-page if necessary
    byte val_8 = mem_get8(addr_e);
    ushrt val_16 = 0;
    if(addr_mode == ADDRMODE_ZPG || addr_mode == ADDRMODE_ZPG_X || addr_mode == ADDRMODE_ZPG_Y)
    {
        byte val_16 = mem_get16_zpg(addr_e);
    else
    {
        byte val_16 = mem_get16(addr_e);
    }

    return 0;
}"""

