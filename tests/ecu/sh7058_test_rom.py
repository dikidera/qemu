#!/usr/bin/env python3
# Build a small SH7058 test ROM exercising the ECU emulator peripherals.
#
#   SCI0  : prints a boot banner and results (115200-ish, BRR=4)
#   ADC0  : converts AN0 (map) and AN2 (clt), prints raw 10-bit values
#   CMT0  : 1 ms interrupt; every 20 ms fires TO8A (inj1, 2.5 ms) and
#           TO8I (ign1, 3.0 ms dwell) one-shot pulses
#   ATU0  : counts crank teeth on TI0A with the ICI0A interrupt
#   HCAN0 : transmits id 0x123 from mailbox 1, receives 0x7E0 in mailbox 2
#
# SPDX-License-Identifier: GPL-2.0-or-later
import sys
from shasm import Asm, Pool

a = Asm(1 << 20)
p = Pool(a, 'p')

RAM = 0xFFFF0000
TICKS = RAM + 0x100       # CMT 1 ms ticks
TEETH = RAM + 0x104       # crank edges

SCI0 = 0xFFFFF000
SSR0 = SCI0 + 4
TDR0 = SCI0 + 3

# vector table
a.org(0)
a.long('reset')
a.long(0xFFFFBFF0)
for v in range(2, 256):
    a.long('unhandled')
a.org(84 * 4)
a.long('ici0a_isr')
a.org(188 * 4)
a.long('cmt0_isr')

a.org(0x1000)
a.label('reset')
# --- SCI0: async 8N1, BRR = 4 (125000 baud @ 20 MHz), TE|RE
a.lit(SCI0, 8, p)
a.mov_i(0, 0); a.raw(0x8082)        # mov.b r0,@(2,r8)  SCR = 0
a.mov_i(0, 0); a.raw(0x8080)        # SMR = 0
a.mov_i(4, 0); a.raw(0x8081)        # BRR = 4
a.mov_i(0x30, 0); a.raw(0x8082)     # SCR = TE|RE
a.lit(0, 0, p)
a.lit(TICKS, 1, p); a.st('l', 0, 1)
a.lit(TEETH, 1, p); a.st('l', 0, 1)

a.lit(ord('B') << 24 | ord('O') << 16 | ord('O') << 8 | ord('T'), 4, p)
a.bsr('putc4'); a.nop()
a.mov_i(10, 4); a.bsr('putc'); a.nop()

# --- ADC0 single conversions on AN0 and AN2
for ch in (0, 2):
    a.lit(0xFFFFF818, 9, p)            # ADCSR0
    a.mov_i(ch, 0); a.st('b', 0, 9)    # single mode, CH=ch
    a.lit(0xFFFFF819, 10, p)
    a.mov_i(0x20, 0); a.st('b', 0, 10) # ADST
    lbl = 'adwait%d' % ch
    a.label(lbl)
    a.ld('b', 9, 0); a.tst_i(0x80); a.bt(lbl)
    a.mov_i(ch, 0); a.st('b', 0, 9)    # clear ADF
    a.lit(0xFFFFF800 + 2 * ch, 9, p)
    # r4 = ADDR >> 6 (10-bit result is left aligned)
    a.ld('w', 9, 4); a.extuw(4, 4); a.shlr(4, 2); a.shlr(4, 2); a.shlr(4, 2)
    a.bsr('puthex'); a.nop()
    a.mov_i(32, 4); a.bsr('putc'); a.nop()

# --- INTC priorities: IPRC.ATU02 = 6, IPRJ.CMT0 = 5
a.lit(0xFFFFED04, 9, p); a.lit(0x0006, 0, p); a.st('w', 0, 9)
a.lit(0xFFFFED12, 9, p); a.lit(0x0500, 0, p); a.st('w', 0, 9)

# --- CMT0: 20 MHz / 8 = 2.5 MHz, 1 ms = 2500 counts
a.lit(0xFFFFF716, 9, p); a.lit(2499, 0, p); a.st('w', 0, 9)     # CMCOR0
a.lit(0xFFFFF712, 9, p); a.lit(0x0040, 0, p); a.st('w', 0, 9)   # CMIE
a.lit(0xFFFFF710, 9, p); a.mov_i(1, 0); a.st('w', 0, 9)         # CMSTR

# --- ATU: PSCR1 = 0, PSCR4 = 19 (1 MHz for channel 8)
a.lit(0xFFFFF404, 9, p); a.mov_i(0, 0); a.st('b', 0, 9)
a.lit(0xFFFFF40A, 9, p); a.mov_i(19, 0); a.st('b', 0, 9)
a.lit(0xFFFFF42A, 9, p); a.mov_i(1, 0); a.st('b', 0, 9)        # TIOR0: A rising
a.lit(0xFFFFF42E, 9, p); a.mov_i(1, 0); a.st('w', 0, 9)        # TIER0: ICEA
a.lit(0xFFFFF668, 9, p); a.mov_i(0, 0); a.st('b', 0, 9)        # TCR8
a.lit(0xFFFFF401, 9, p); a.mov_i(1, 0); a.st('b', 0, 9)        # TSTR1: STR0

# --- HCAN0 (HCAN2): leave reset, MB1 tx id 0x123, MB2 rx id 0x7E0
H = 0xFFFFD000
a.lit(H, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)                 # MCR = 0
a.label('canwait')
a.lit(H + 2, 9, p); a.ld('w', 9, 0); a.tst_i(0x08); a.bf('canwait')
MB1 = H + 0x100 + 0x20
a.lit(MB1, 9, p); a.lit(0x123 << 4, 0, p); a.st('w', 0, 9)
a.lit(MB1 + 2, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)
a.lit(MB1 + 4, 9, p); a.mov_i(8, 0); a.st('w', 0, 9)            # MBC=0, DLC=8
a.lit(MB1 + 8, 9, p); a.lit(0x51454D55, 0, p); a.st('l', 0, 9)  # "QEMU"
a.lit(MB1 + 12, 9, p); a.lit(0x45435521, 0, p); a.st('l', 0, 9) # "ECU!"
MB2 = H + 0x100 + 0x40
a.lit(MB2, 9, p); a.lit(0x7E0 << 4, 0, p); a.st('w', 0, 9)
a.lit(MB2 + 2, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)
a.lit(MB2 + 4, 9, p); a.lit(0x0408, 0, p); a.st('w', 0, 9)      # MBC=4 rx data
a.lit(MB2 + 0x10, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)        # LAFM exact
a.lit(MB2 + 0x12, 9, p); a.st('w', 0, 9)
a.lit(H + 0x22, 9, p); a.mov_i(2, 0); a.st('w', 0, 9)          # TXPR0 MB1
a.lit(H + 0x32, 9, p); a.ld('w', 9, 0); a.tst_i(2); a.bt('cantxfail')
a.lit(ord('C') << 24 | ord('A') << 16 | ord('N') << 8 | ord('T'), 4, p)
a.bsr('putc4'); a.nop()
a.label('cantxfail')

# enable interrupts (SR.I = 0)
a.mov_i(0, 0); a.ldc_sr(0)

# --- main loop: every 1000 ticks print teeth count; report CAN rx
a.label('main')
a.lit(TICKS, 9, p)
a.label('wait')
a.ld('l', 9, 0); a.lit(1000, 1, p); a.cmphs(1, 0); a.bf('wait2')
a.mov_i(0, 0); a.st('l', 0, 9)
a.mov_i(ord('T'), 4); a.bsr('putc'); a.nop()
a.lit(TEETH, 9, p); a.ld('l', 9, 4); a.bsr('puthex'); a.nop()
a.mov_i(10, 4); a.bsr('putc'); a.nop()
a.bra('main'); a.nop()
a.label('wait2')
a.lit(H + 0x42, 10, p); a.ld('w', 10, 0); a.tst_i(4); a.bt('wait')
a.mov_i(4, 0); a.st('w', 0, 10)                                 # clear RXPR
a.lit(ord('R') << 24 | ord('X') << 16 | ord('O') << 8 | ord('K'), 4, p)
a.bsr('putc4'); a.nop()
a.lit(MB2 + 8, 10, p); a.ld('l', 10, 4); a.bsr('puthex'); a.nop()
a.mov_i(10, 4); a.bsr('putc'); a.nop()
a.bra('wait'); a.nop()
p.emit()

# --- putc(r4): wait TDRE, write TDR, clear TDRE
q = Pool(a, 'q')
a.label('putc')
a.lit(SSR0, 1, q)
a.label('putc_w')
a.ld('b', 1, 0); a.tst_i(0x80); a.bt('putc_w')
a.lit(TDR0, 2, q); a.st('b', 4, 2)
a.and_i(0x7f); a.st('b', 0, 1)
a.rts(); a.nop()

# putc4: print the 4 characters packed in r4 (msb first)
a.label('putc4')
a.sts_pr_push()
a.push(5); a.push(6)
a.mov(4, 5); a.mov_i(4, 6)
a.label('putc4_l')
a.mov(5, 4); a.shlr(4, 16); a.shlr(4, 8)
a.bsr('putc'); a.nop()
a.shll(5, 8)
a.dt(6); a.bf('putc4_l')
a.pop(6); a.pop(5)
a.lds_pr_pop()
a.rts(); a.nop()

# puthex(r4): 8 hex digits
a.label('puthex')
a.sts_pr_push()
a.push(5); a.push(6)
a.mov(4, 5); a.mov_i(8, 6)
a.label('puthex_l')
a.mov(5, 0); a.shlr(0, 16); a.shlr(0, 8); a.shlr(0, 2); a.shlr(0, 2)
a.and_i(0x0f)
a.mov_i(10, 1); a.cmphs(1, 0); a.bf('puthex_d')
a.add_i(7, 0)
a.label('puthex_d')
a.add_i(0x30, 0); a.mov(0, 4)
a.bsr('putc'); a.nop()
a.shll(5, 2); a.shll(5, 2)
a.dt(6); a.bf('puthex_l')
a.pop(6); a.pop(5)
a.lds_pr_pop()
a.rts(); a.nop()

# --- CMT0 ISR: clear CMF, count ticks, every 20 ms fire inj1/ign1
a.label('cmt0_isr')
for r in (0, 1, 2):
    a.push(r)
a.lit(0xFFFFF712, 1, q); a.ld('w', 1, 0); a.and_i(0x7f); a.st('w', 0, 1)
a.lit(TICKS, 1, q); a.ld('l', 1, 0); a.add_i(1, 0); a.st('l', 0, 1)
a.lit(RAM + 0x108, 1, q); a.ld('l', 1, 0); a.add_i(1, 0); a.st('l', 0, 1)
a.cmpeq_i(20); a.bf('cmt_done')
a.mov_i(0, 0); a.st('l', 0, 1)
a.lit(0xFFFFF640, 1, q); a.lit(2500, 0, q); a.st('w', 0, 1)    # DCNT8A
a.lit(0xFFFFF650, 1, q); a.lit(3000, 0, q); a.st('w', 0, 1)    # DCNT8I
a.lit(0xFFFFF666, 1, q); a.lit(0x0101, 0, q); a.st('w', 0, 1)  # DSTR A|I
a.label('cmt_done')
for r in (2, 1, 0):
    a.pop(r)
a.rte(); a.nop()

# --- ICI0A ISR: clear ICF0A, count crank edges
a.label('ici0a_isr')
a.push(0); a.push(1)
a.lit(0xFFFFF42C, 1, q); a.lit(0xfffe, 0, q); a.st('w', 0, 1)
a.lit(TEETH, 1, q); a.ld('l', 1, 0); a.add_i(1, 0); a.st('l', 0, 1)
a.pop(1); a.pop(0)
a.rte(); a.nop()

a.label('unhandled')
a.mov_i(ord('!'), 4); a.bsr('putc'); a.nop()
a.label('hang')
a.bra('hang'); a.nop()
q.emit()

open(sys.argv[1], 'wb').write(a.build())
