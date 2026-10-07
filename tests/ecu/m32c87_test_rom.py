#!/usr/bin/env python3
# Build a small M32C/87 test ROM (1 MiB flash image, 0xF00000..0xFFFFFF)
#
#   UART0 : boot banner and results
#   A/D0  : AN0 (map) and AN2 (clt) one-shot conversions, printed raw
#   TA0   : 1 ms timer interrupt; every 20 ms starts TA1 one-shot
#           (TA1OUT = inj1, 2.5 ms)
#   TB0   : crank period measurement on TB0IN, counts teeth in its ISR
#   CAN0  : transmits id 0x123 from slot 0
#
# SPDX-License-Identifier: GPL-2.0-or-later
import sys
from m32casm import *

a = Asm(0xF00000, 0x100000)
TICKS = ABS16(0x0400)
TEETH = ABS16(0x0402)
INJT = ABS16(0x0404)
VECT = 0xFFFE00

# fixed vectors
a.org(0xFFFFFC)
a.addr24('start')
# variable vector table (INTB)
a.org(VECT + 12 * 4)
a.addr24('ta0_isr')
a.org(VECT + 21 * 4)
a.addr24('tb0_isr')

a.org(0xFF0000)
a.label('start')
a.ldc24(0x00c000, 'ISP')
a.ldc24(VECT, 'INTB')
a.mov_w_imm(0, TICKS)
a.mov_w_imm(0, TEETH)
a.mov_w_imm(0, INJT)

# UART0: 8N1 internal clock, f1/16/(BRG+1)
a.mov_b_imm(0x05, ABS16(0x0368))       # U0MR
a.mov_b_imm(0x10, ABS16(0x0369))       # U0BRG
a.mov_b_imm(0x00, ABS16(0x036c))       # U0C0
a.mov_b_imm(0x05, ABS16(0x036d))       # U0C1: TE | RE
for ch in b'BOOT\n':
    a.mov_b_imm(ch, R0L)
    a.jsr('putc')

# A/D0 one-shot, 10 bit: AN0, AN2
a.mov_b_imm(0x08, ABS16(0x0397))       # AD0CON1: BITS=1
for ch in (0, 2):
    a.mov_b_imm(0x40 | ch, ABS16(0x0396))
    a.label('adw%d' % ch)
    a.btst(6, ABS16(0x0396))
    a.j(NE, 'adw%d' % ch)
    a.mov_w(ABS16(0x0380 + 2 * ch), R0)
    a.jsr('puthex')
    a.mov_b_imm(0x20, R0L)
    a.jsr('putc')

# TA0: timer mode, f1 = 32 MHz, 1 ms; TA0IC level 5
a.mov_b_imm(0x00, ABS16(0x0356))
a.mov_w_imm(31999, ABS16(0x0346))
a.mov_b_imm(0x05, ABS16(0x006c))
# TA1: one-shot, f8 (4 MHz), output enabled, 2.5 ms
a.mov_b_imm(0x46, ABS16(0x0357))
a.mov_w_imm(10000, ABS16(0x0348))
# TB0: period measurement (falling edges), f1; TB0IC level 6
a.mov_b_imm(0x02, ABS16(0x035b))
a.mov_b_imm(0x06, ABS16(0x0094))
a.mov_b_imm(0x23, ABS16(0x0340))       # TABSR: TA0, TA1, TB0

# CAN0: leave reset, slot 0 = std id 0x123, 8 bytes, transmit
a.mov_w_imm(0x0000, ABS16(0x0200))
a.mov_b_imm(0x00, ABS16(0x0220))       # slot buffer select: slot 0
a.mov_b_imm(0x123 >> 6, ABS16(0x01e0))
a.mov_b_imm(0x123 & 0x3f, ABS16(0x01e1))
a.mov_b_imm(8, ABS16(0x01e5))
for i, ch in enumerate(b'QEMU-M32'):
    a.mov_b_imm(ch, ABS16(0x01e6 + i))
# CAN1 (base 0x0280, slot window 0x0260): slot 0 receives std id 0x123
a.mov_w_imm(0x0000, ABS16(0x0280))
a.mov_b_imm(0x00, ABS16(0x02a0))       # C1SBS
for i in range(6):
    a.mov_b_imm(0xff, ABS16(0x02a8 + i))   # C1GMR: compare all bits
a.mov_b_imm(0x123 >> 6, ABS16(0x0260))
a.mov_b_imm(0x123 & 0x3f, ABS16(0x0261))
a.mov_b_imm(0x40, ABS16(0x02b0))       # C1MCTL0: RECREQ
a.mov_b_imm(0x80, ABS16(0x0230))       # C0MCTL0: TRMREQ
a.btst(0, ABS16(0x0230))
a.j(EQ, 'cantxfail')
for ch in b'CANT':
    a.mov_b_imm(ch, R0L)
    a.jsr('putc')
a.label('cantxfail')
a.btst(0, ABS16(0x02b0))               # C1MCTL0 NEWDATA
a.j(EQ, 'canrxfail')
for ch in b'RX1=':
    a.mov_b_imm(ch, R0L)
    a.jsr('putc')
a.mov_w(ABS16(0x0266), R0)
a.jsr('puthex')
a.mov_b_imm(0x0a, R0L)
a.jsr('putc')
a.label('canrxfail')

a.fset('I')

a.label('main')
a.cmp_w_imm(1000, TICKS)
a.j(LTU, 'main')
a.mov_w_imm(0, TICKS)
a.mov_b_imm(ord('T'), R0L)
a.jsr('putc')
a.mov_w(TEETH, R0)
a.jsr('puthex')
a.mov_b_imm(0x0a, R0L)
a.jsr('putc')
a.jmp('main')

# putc(R0L)
a.label('putc')
a.btst(1, ABS16(0x036d))               # TI
a.j(EQ, 'putc')
a.mov_b(R0L, ABS16(0x036a))
a.rts()

# puthex(R0): 4 hex digits
a.label('puthex')
a.pushm(0xc0)                          # R0, R1
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

# TA0 ISR: count ms, start TA1 one-shot every 20 ms
a.label('ta0_isr')
a.inc_w(TICKS)
a.inc_w(INJT)
a.cmp_w_imm(20, INJT)
a.j(LTU, 'ta0_done')
a.mov_w_imm(0, INJT)
a.mov_b_imm(0x02, ABS16(0x0342))       # ONSF: TA1OS
a.label('ta0_done')
a.reit()

# TB0 ISR: crank tooth
a.label('tb0_isr')
a.inc_w(TEETH)
a.reit()

open(sys.argv[1], 'wb').write(a.build())
