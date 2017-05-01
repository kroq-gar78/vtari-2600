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
sums = 0

# print
# sys.stdout.write('F0[16][16] opcodes = {')
for a in xrange(0x10):
    l = []
    for b in xrange(0x10):
        inst, addr = lines_sep[a][b]
        inst = inst.lower()
        addr = re.sub('#','imm',addr)
        addr = re.sub(',','_',addr)
        addr = addr.lower()


        if inst == "???" or "addr" == "---":
            l.append("(F1) OPCODE_BAD")
            continue

        d[inst].add(addr)
        addr_modes.add(addr)

        l.append("%s_%s" % (inst, addr))
    # print "row", len(l)
    opcodes_table.append(', '.join(l))
    sums += len(l)

opcodes_table_str = ', \n'.join(opcodes_table)

print
print
# print(sums)

for inst in sorted(d.iterkeys()):
    for addr in sorted(d[inst]):
        print "int %s_%s(short addr);" % (inst, addr)

print "F1 opcodes[256] = {%s};" % opcodes_table_str
