#!/usr/bin/env python3
# Self checking M32C/80 instruction test ROM for the ecu-m32c87 machine.
# Prints "CPU OK" on UART0, or "F<n>" for each failing check.
# SPDX-License-Identifier: GPL-2.0-or-later
import sys
from m32casm import *

a = Asm2(0xF00000, 0x100000)
VECT = 0xFFFE00
MEM = 0x1000            # scratch RAM
n = [0]


def check_w(op, exp):
    """compare a word operand with an expected value"""
    n[0] += 1
    lbl = 'ok%d' % n[0]
    a.cmp_w_imm(exp, op)
    a.j(EQ, lbl)
    a.mov_w(op, ABS16(0x0802))
    fail(n[0], True)
    a.label(lbl)


def check_cond(cnd):
    """verify that condition cnd holds"""
    n[0] += 1
    lbl = 'ok%d' % n[0]
    a.j(cnd, lbl)
    fail(n[0])
    a.label(lbl)


def fail(i, val=False):
    a.mov_b_imm(ord('F'), R0L)
    a.jsr('putc')
    a.mov_w_imm(i, R0)
    a.jsr('puthex')
    if val:
        a.mov_b_imm(ord('='), R0L)
        a.jsr('putc')
        a.mov_w(ABS16(0x0802), R0)
        a.jsr('puthex')
    a.mov_b_imm(0x0a, R0L)
    a.jsr('putc')
    a.inc_w(ABS16(0x0800))


a.org(0xFFFFFC)
a.addr24('start')
a.org(VECT + 40 * 4)
a.addr24('int40')

a.org(0xFF0000)
a.label('start')
a.ldc24(0x00c000, 'ISP')
a.ldc24(VECT, 'INTB')
a.mov_w_imm(0, ABS16(0x0800))       # failure count
a.mov_b_imm(0x05, ABS16(0x0368))
a.mov_b_imm(0x10, ABS16(0x0369))
a.mov_b_imm(0x05, ABS16(0x036d))

# --- add / carry / overflow
a.mov_w_imm(0xfff0, R0)
a.add_w_imm(0x0020, R0)
check_cond(GEU)                      # C set (carry out)
check_w(R0, 0x0010)
a.mov_w_imm(0x7fff, R1)
a.add_w_imm(1, R1)
check_cond(O)                        # signed overflow
check_w(R1, 0x8000)
# adc uses C
a.mov_w_imm(0xffff, R0)
a.add_w_imm(1, R0)                   # C=1, R0=0
a.adc_w_imm(5, R0)                   # R0 = 0 + 5 + 1
check_w(R0, 6)
# --- sub / cmp: C = no borrow
a.mov_w_imm(5, R0)
a.cmp_w_imm(3, R0)
check_cond(GTU)
a.cmp_w_imm(5, R0)
check_cond(EQ)
a.cmp_w_imm(9, R0)
check_cond(LTU)
a.mov_w_imm(0xfffe, R2)              # -2
a.cmp_w_imm(1, R2)
check_cond(LT)                       # signed -2 < 1
check_cond(GTU)                      # unsigned 0xfffe > 1
a.mov_w_imm(10, R0)
a.sub_w_imm(3, R0)
check_w(R0, 7)
# sbb: dst - src - !C
a.mov_w_imm(10, R0)
a.cmp_w_imm(20, R0)                  # C=0 (borrow)
a.sbb_w_imm(3, R0)                   # 10 - 3 - 1
check_w(R0, 6)
a.neg_w(R0)
check_w(R0, 0xfffa)
a.not_w(R0)
check_w(R0, 0x0005)

# --- multiply / divide
a.mov_w_imm(0xfffd, R0)              # -3
a.mul_w_imm(1000, R0)                # R2R0 = -3000
check_w(R0, (-3000) & 0xffff)
check_w(R2, 0xffff)
a.mov_w_imm(300, R0)
a.mulu_w_imm(1000, R0)               # 300000 = 0x493e0
check_w(R0, 0x93e0)
check_w(R2, 0x0004)
# R2R0 = 300000 / 7 = 42857 r 1  (DIVU)
a.divu_w_imm(7)
check_w(R0, 42857)
check_w(R2, 1)
# DIV: -7 / 2 = -3 r -1;  DIVX: -7 / 2 = -4 r 1
a.mov_w_imm(0xfff9, R0)
a.mov_w_imm(0xffff, R2)
a.div_w_imm(2)
check_w(R0, 0xfffd)
check_w(R2, 0xffff)
a.mov_w_imm(0xfff9, R0)
a.mov_w_imm(0xffff, R2)
a.divx_w_imm(2)
check_w(R0, 0xfffc)
check_w(R2, 0x0001)

# --- shifts / rotates
a.mov_w_imm(0x8001, R0)
a.shl_w(1, R0)
check_cond(GEU)                      # C = old bit 15
check_w(R0, 0x0002)
a.mov_w_imm(0x8000, R0)
a.sha_w(-3, R0)
check_w(R0, 0xf000)
a.mov_w_imm(0x8000, R0)
a.shl_w(-3, R0)
check_w(R0, 0x1000)
a.mov_w_imm(0x8001, R0)
a.rot_w(4, R0)
check_w(R0, 0x0018)
a.mov_w_imm(0x0001, R0)
a.rot_w(-1, R0)
check_w(R0, 0x8000)
a.fset('C')
a.mov_w_imm(0x4000, R0)
a.rolc_w(R0)
check_w(R0, 0x8001)
a.fclr('C')
a.rorc_w(R0)
check_w(R0, 0x4000)
check_cond(GEU)                      # bit 0 went to C

# --- extensions
a.mov_w_imm(0x0080, R0)
a.exts_b(R0L)
check_w(R0, 0xff80)
a.mov_b_imm(0xfe, ABS16(MEM))
a.extz_b(ABS16(MEM), R1)
check_w(R1, 0x00fe)

# --- memory addressing: [A0], dsp:8[SB], indirect, INDEX
a.emit(bytes([0x9c]) + (MEM).to_bytes(2, 'little'))      # MOV.W:S #MEM, A0
a.mov_w_imm(0x1234, IA0)
check_w(ABS16(MEM), 0x1234)
a.ldc24(MEM - 0x10, 'SB')
check_w(SB8(0x10), 0x1234)
# pointer at MEM+8 -> MEM, read through [abs16] indirect
a.mov_w_imm(MEM, ABS16(MEM + 8))
a.mov_w_imm(0, ABS16(MEM + 10))
a.mov_w_ind_src(ABS16(MEM + 8), R3)
check_w(R3, 0x1234)
# INDEXW.W R1 (=3): next instruction reads MEM + 6
a.mov_w_imm(0xbeef, ABS16(MEM + 6))
a.mov_w_imm(3, R1)
a.indexw_w(R1)
a.mov_w(ABS16(MEM), R2)
check_w(R2, 0xbeef)

# --- stack, PUSHM/POPM, ENTER/EXITD via a call
a.mov_w_imm(0x1111, R0)
a.mov_w_imm(0x2222, R1)
a.pushm(0xc0)
a.mov_w_imm(0, R0)
a.mov_w_imm(0, R1)
a.popm(0x03)
check_w(R0, 0x1111)
check_w(R1, 0x2222)
a.mov_w_imm(0x0055, R0)
a.jsr('frame_fn')
check_w(R0, 0x00aa)

# --- register bank 1
a.mov_w_imm(0x0a0a, R0)
a.fset('B')
a.mov_w_imm(0x0b0b, R0)
a.fclr('B')
check_w(R0, 0x0a0a)
a.fset('B')
check_w(R0, 0x0b0b)
a.fclr('B')

# --- ADJNZ loop: 5 iterations
a.mov_w_imm(5, R1)
a.mov_w_imm(0, R0)
a.label('adj')
a.add_w_imm(2, R0)
a.adjnz_w(-1, R1, 'adj')
check_w(R0, 10)

# --- string ops: SSTR.B fills, SMOVF.W copies
a.emit(bytes([0x9d]) + (MEM + 0x20).to_bytes(2, 'little'))   # A1
a.mov_w_imm(4, R3)
a.mov_b_imm(0x5a, R0L)
a.sstr_b()
check_w(ABS16(MEM + 0x22), 0x5a5a)
a.emit(bytes([0x9c]) + (MEM + 0x20).to_bytes(2, 'little'))   # A0 src
a.emit(bytes([0x9d]) + (MEM + 0x40).to_bytes(2, 'little'))   # A1 dst
a.mov_w_imm(2, R3)
a.smovf_w()
check_w(ABS16(MEM + 0x42), 0x5a5a)

# --- software interrupt and REIT
a.mov_w_imm(0, R0)
a.int_(40)
check_w(R0, 0x0040)

# --- bit ops
a.mov_w_imm(0x0000, ABS16(MEM))
a.bset(3, ABS16(MEM))
check_w(ABS16(MEM), 0x0008)
a.btst(3, ABS16(MEM))
check_cond(NE)                       # Z = !bit
a.bclr(3, ABS16(MEM))
a.btst(3, ABS16(MEM))
check_cond(EQ)

a.cmp_w_imm(0, ABS16(0x0800))
a.j(NE, 'done')
for ch in b'CPU OK\n':
    a.mov_b_imm(ch, R0L)
    a.jsr('putc')
a.label('done')
a.jmp('done')

# frame_fn: ENTER #4, store arg to a local, double it, EXITD
a.label('frame_fn')
a.enter(4)
a.mov_w(R0, FB8(-2))
a.add_w(FB8(-2), R0)
a.exitd()

a.label('int40')
a.mov_w_imm(0x0040, R0)
a.reit()

a.label('putc')
a.btst(1, ABS16(0x036d))
a.j(EQ, 'putc')
a.mov_b(R0L, ABS16(0x036a))
a.rts()

a.label('puthex')
a.pushm(0xc0)
a.mov_w(R0, R1)
for sh in (12, 8, 4, 0):
    a.mov_w(R1, R0)
    for _ in range(sh // 4):
        a.shl_w(-4, R0)
    a.and_w_imm(0x0f, R0)
    a.cmp_w_imm(10, R0)
    a.j(LTU, 'hx%d' % sh)
    a.add_w_imm(7, R0)
    a.label('hx%d' % sh)
    a.add_w_imm(0x30, R0)
    a.jsr('putc')
a.popm(0x03)
a.rts()

open(sys.argv[1], 'wb').write(a.build())
