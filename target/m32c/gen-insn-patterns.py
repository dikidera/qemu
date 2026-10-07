#!/usr/bin/env python3
# Generate insn-patterns.c.inc from Ghidra's M16C_80.slaspec.
# SPDX-License-Identifier: GPL-2.0-or-later
import re, sys, collections
src = open(sys.argv[1]).read()
cons = []
for line in src.split('\n'):
    if not line.startswith(':') or line.startswith(':^instruction'):
        continue
    m = re.match(r':(\S+)\s*(.*?)\s+is\s+(.*?)(\{|$)', line)
    if not m:
        print('NOMATCH', line, file=sys.stderr); continue
    mnem, ops, pat = m.group(1), m.group(2), m.group(3)
    mnem = mnem.replace('^".', '.').replace('"', '').replace('^cnd', 'cnd').replace('^b2cnd','cnd').replace('^b1cnd','cnd')
    cons.append((mnem, ops.strip(), pat.strip()))
FIELDS = {
 'b0_0007': (0, 0, 7),
 'b1_s5': (1, 4, 6), 'b1_s5_4': (1, 6, 6), 'b1_d5': (1, 1, 3), 'b1_d5_4': (1, 3, 3),
 'b1_d2': (1, 4, 5), 'b1_d1_regAx': (1, 0, 0), 'b1_size_5': (1, 5, 5),
 'b1_size_4': (1, 4, 4), 'b1_size_0': (1, 0, 0),
 'b2_d5': (2, 6, 7), 'b2_s5': (2, 4, 5), 'b2_d5_1': (2, 7, 7), 'b2_d5_0': (2, 6, 6),
 'b2_s5_1': (2, 5, 5), 'b2_s5_0': (2, 4, 4), 'b2_shiftSign': (2, 3, 3),
}
def field(name):
    if name in FIELDS: return FIELDS[name]
    m = re.match(r'b([012])_(\d\d)(\d\d)$', name)
    if m: return (int(m.group(1)), int(m.group(2)), int(m.group(3)))
    return None
out = []
for mnem, ops, pat in cons:
    # Ax-only duplicate constructors are covered by the generic dst5 handling
    if 'DST5AX' in pat or 'BITBASE_AX' in pat or 'reloffset_dst5Ax' in ops:
        continue
    mask = [0, 0, 0]; val = [0, 0, 0]; has = [False, False, False]
    for name, v in re.findall(r'\b(b[012]_\w+)\s*=\s*(0x[0-9a-fA-F]+|\d+)', pat):
        f = field(name)
        if not f: print('UNK', name, mnem, file=sys.stderr); continue
        b, lo, hi = f
        w = hi - lo + 1
        mask[b] |= ((1 << w) - 1) << lo
        val[b] |= int(v, 0) << lo
        has[b] = True
    # byte 2 is present if any b2_ field or src5/dst5/bitbase appears
    if re.search(r'\bb2_|SRC5|DST5|BITBASE|dst5|bitbase', pat): has[2] = True
    has[1] = True
    prefix = val[0] if has[0] else -1
    out.append((mnem, ops, prefix, mask[1], val[1], mask[2] if has[2] else 0, val[2], has[2]))
forms = collections.Counter()
for o in out:
    forms[re.sub(r'\s+', ' ', o[1])] += 1
if len(sys.argv) > 2 and sys.argv[2] == 'forms':
    for f, n in sorted(forms.items()): print(n, f)
elif len(sys.argv) <= 2:
    for o in out:
        print('%-10s %-40s pre=%3d b1 %02x/%02x b2 %02x/%02x %s' % (o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7]))

FORMS = {
 '': 'F_NONE', '"C"': 'F_BMC',
 'R0, dst2W': 'F_R0_DST2', 'R0L, dst2B': 'F_R0_DST2',
 'R0L, dst5B': 'F_R0L_DST5',
 'R1H, dst5B': 'F_R1H_DST5', 'R1H, dst5L': 'F_R1H_DST5', 'R1H, dst5W': 'F_R1H_DST5',
 'abs16offset, abs24offset': 'F_CTX', 'abs24offset': 'F_ABS24',
 'b, bitbaseAbs16': 'F_BTST_S',
 'b2_creg16': 'F_CREG16', 'b2_creg24': 'F_CREG24',
 'b2_creg16, dst5W': 'F_CREG16_DST5', 'b2_creg24, dst5L': 'F_CREG24_DST5',
 'b2_dreg24, dst5L': 'F_DREG24_DST5',
 'b2_reg16, dst5W': 'F_XCHG', 'b2_reg8, dst5B': 'F_XCHG',
 'b2_regAx, dst5B': 'F_XCHG_AX', 'b2_regAx, dst5W': 'F_XCHG_AX',
 'bit, bitbase': 'F_BIT',
 'dsp8spB, dst5B_afterDsp8': 'F_SP_DST5', 'dsp8spW, dst5W_afterDsp8': 'F_SP_DST5',
 'dst2B, R0L': 'F_DST2_R0', 'dst2W, R0': 'F_DST2_R0',
 'dst2B, R1L': 'F_DST2_R1', 'dst2W, R1': 'F_DST2_R1',
 'dst2L, b1_d1_regAx': 'F_DST2_AX',
 'dst5A': 'F_DST5A', 'dst5A, b2_reg32': 'F_DST5A_REG', 'dst5A, b2_regAx': 'F_DST5A_REG',
 'dst5B': 'F_DST5', 'dst5W': 'F_DST5', 'dst5L': 'F_DST5',
 'dst5B, R0L': 'F_DST5_R0L',
 'dst5B, dsp8spB': 'F_DST5_SP', 'dst5W, dsp8spW': 'F_DST5_SP',
 'dst5L, b2_creg24': 'F_DST5_CREG24', 'dst5L, b2_dreg24': 'F_DST5_DREG24',
 'dst5W, b2_creg16': 'F_DST5_CREG16',
 'flagBit': 'F_FLAG', 'popRegList': 'F_REGLIST', 'pushRegList': 'F_REGLIST',
 'rel16offset1': 'F_REL16', 'rel3offset2': 'F_REL3', 'rel8offset1': 'F_REL8',
 'reloffset_dst5L': 'F_DST5', 'reloffset_dst5W': 'F_DST5',
 'src5B, dst5B_afterSrc5': 'F_SRC5_DST5', 'src5B, dst5L_afterSrc5': 'F_SRC5_DST5',
 'src5B, dst5W_afterSrc5': 'F_SRC5_DST5', 'src5L, dst5L_afterSrc5': 'F_SRC5_DST5',
 'src5W, dst5W_afterSrc5': 'F_SRC5_DST5',
 'srcImm16': 'F_IMM', 'srcImm8': 'F_IMM', 'srcImm32': 'F_IMM', 'srcSimm16': 'F_IMM',
 'srcSimm8': 'F_IMM',
 'srcImm16, b1_d1_regAx': 'F_IMM_AX', 'srcImm24, b1_d1_regAx': 'F_IMM_AX',
 'srcImm16, b2_creg16': 'F_IMM_CREG16', 'srcImm24, b2_creg24': 'F_IMM_CREG24',
 'srcImm24, b2_dreg24': 'F_IMM_DREG24',
 'srcImm16, dst2W': 'F_IMM_DST2', 'srcImm8, dst2B': 'F_IMM_DST2',
 'srcSimm16, dst2W': 'F_IMM_DST2', 'srcSimm8, dst2B': 'F_IMM_DST2',
 'srcImm16, dst5W': 'F_IMM_DST5', 'srcImm8, dst5B': 'F_IMM_DST5',
 'srcImm32, dst5L': 'F_IMM_DST5', 'srcSimm16, dst5W': 'F_IMM_DST5',
 'srcSimm8, dst5B': 'F_IMM_DST5', 'srcSimm32, dst5L': 'F_IMM_DST5',
 'srcSimm8, dst5L': 'F_IMM8_DST5',
 'srcImm16, srcImm16a, dst5W': 'F_IMM2_DST5', 'srcImm8, srcImm8a, dst5B': 'F_IMM2_DST5',
 'srcSimm16, srcSimm16a, dst5W': 'F_IMM2_DST5', 'srcSimm8, srcSimm8a, dst5B': 'F_IMM2_DST5',
 'srcImm1p, b1_d1_regAx': 'F_IMM1P_AX', 'srcImm3p, SP': 'F_IMM3P_SP',
 'srcSimm16, SP': 'F_IMM_SP', 'srcSimm8, SP': 'F_IMM_SP',
 'srcImm3': 'F_IMM3', 'srcIntNum': 'F_INTNUM',
 'srcSimm4, dst5B': 'F_Q4_DST5', 'srcSimm4, dst5W': 'F_Q4_DST5', 'srcSimm4, dst5L': 'F_Q4_DST5',
 'srcSimm4, dst5B, rel8offset2': 'F_ADJNZ', 'srcSimm4, dst5W, rel8offset2': 'F_ADJNZ',
 'srcSimm4Shift, dst5B': 'F_SH4_DST5', 'srcSimm4Shift, dst5W': 'F_SH4_DST5',
 'srcZero16, dst2W': 'F_ZERO_DST2', 'srcZero8, dst2B': 'F_ZERO_DST2',
}

def gen_c(rows):
    lines = []
    ops = []
    for mnem, opstr, pre, m1, v1, m2, v2, has2 in rows:
        opstr = re.sub(r'\s+', ' ', opstr)
        form = FORMS[opstr]
        base = mnem.split('.')[0].split(':')[0]
        size = 0
        mm = re.search(r'\.([BWLA])', mnem)
        if mm:
            size = {'B': 1, 'W': 2, 'L': 4, 'A': 3}[mm.group(1)]
        # quirks of the source table
        if base == 'MOV' and opstr == 'dst2L, b1_d1_regAx':
            size = 4
        if base == 'PUSH' and opstr == 'srcImm16':
            size = 2
        if base in ('JMPI', 'JSRI') and size == 3:
            size = 4
        if form == 'F_BTST_S':
            has2 = False; m2 = 0; v2 = 0
        opname = 'OP_' + base.replace('cnd', '_CND')
        if opname not in ops:
            ops.append(opname)
        lines.append('    { %d, 0x%02x, 0x%02x, 0x%02x, 0x%02x, %d, %s, %s, %d }, /* %s %s */'
                     % (1 if pre == 1 else 0, m1, v1, m2, v2, 1 if has2 else 0,
                        opname, form, size, mnem, opstr))
    return ops, lines

if len(sys.argv) > 2 and sys.argv[2] == 'c':
    ops, lines = gen_c(out)
    print('/* Generated from Ghidra M16C_80.slaspec constructor patterns */')
    print('enum {\n    OP_INVALID,')
    for o in ops:
        print('    %s,' % o)
    print('};\n')
    print('static const M32CInsnPattern m32c_patterns[] = {')
    print('\n'.join(lines))
    print('};')
