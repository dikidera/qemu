#!/usr/bin/env python3
# Build an MC68376 QADC / CTM4 test ROM (512 KiB boot flash image).
#
#   SYNCR  : 16.78 MHz; watchdog off; SRAM at $100000; SCI 9600 8N1
#   QADC   : QCLK = fsys / 16; queue 1 software single-scan of AN2 (clt),
#            AN0 (map), VRH, VRL, VDDA/2 with the completion interrupt
#            (IRLQ1 = 4, vector $63); results printed in the three
#            formats.  Queue 2 (BQ2 = 8) external trigger rising edge
#            continuous scan on ETRIG2 (cam bound to it), completion
#            interrupt level 3 (vector $61) counted.
#   CTM4   : PRUN, VECT = %10 ($80), IARB = 5.
#            FCSM12 on PCLK3 (fsys / 8) drives TBB2, overflow interrupt
#            level 5 (vector $8C) used as a time base.
#            DASM3 IPM on TBB2 measures the crank tooth period (crank
#            bound to CTD3), interrupt level 6 (vector $83): min / max.
#            DASM10 IPWM on TBB2 measures the crank high time (CTD10).
#            PWMSM5 (CPWM5): fsys / 16, period 8192, width 2048
#            -> 128 Hz, 1.953125 ms (bound to inj1 and isc).
#            MCSM2 on PCLK3, modulus $F800, drives TBB4; DASM4 OPWM on
#            TBB4: A = $F800, B = $FA00 -> 1024 Hz 25 % on CTD4 (egr).
#
# SPDX-License-Identifier: GPL-2.0-or-later
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m68kasm import *  # noqa: E402,F403

SYNCR = absw(0xfa04)
SYPCR = absw(0xfa21)
RAMMCR = absw(0xfb40)
RAMBAH = absw(0xfb44)
RAMBAL = absw(0xfb46)
SCCR0 = absw(0xfc08)
SCCR1 = absw(0xfc0a)
SCSR = absw(0xfc0c)
SCDR = absw(0xfc0e)

# QADC (Table D-24)
QADCMCR = absw(0xf200)
QADCINT = absw(0xf204)
QACR0 = absw(0xf20a)
QACR1 = absw(0xf20c)
QACR2 = absw(0xf20e)
QASR = absw(0xf210)


def CCW(n):
    return absw(0xf230 + 2 * n)


def RJURR(n):
    return absw(0xf2b0 + 2 * n)


def LJSRR(n):
    return absw(0xf330 + 2 * n)


def LJURR(n):
    return absw(0xf3b0 + 2 * n)


# CTM4 (Table D-35)
BIUMCR = absw(0xf400)
CPCR = absw(0xf408)
MCSM2SIC = absw(0xf410)
MCSM2CNT = absw(0xf412)
DASM3SIC = absw(0xf418)
DASM3A = absw(0xf41a)
DASM3B = absw(0xf41c)
DASM4SIC = absw(0xf420)
DASM4A = absw(0xf422)
DASM4B = absw(0xf424)
PWM5SIC = absw(0xf428)
PWM5A = absw(0xf42a)
PWM5B = absw(0xf42c)
DASM10SIC = absw(0xf450)
DASM10A = absw(0xf452)
DASM10B = absw(0xf454)
FCSM12SIC = absw(0xf460)

SRAM = 0x100000
Q1F = absl(SRAM + 0x10)
Q2N = absl(SRAM + 0x14)
OVF = absl(SRAM + 0x18)
NCAP = absl(SRAM + 0x1c)
MINP = absl(SRAM + 0x20)
MAXP = absl(SRAM + 0x22)

DASM3_CFG = 0x6102      # IL = 6, BSL (TBB2), EDPOL = 0, IPM
FCSM_CFG = 0x5102       # IL = 5, DRVB (TBB2), PCLK3

a = Asm(0, 0x80000)

a.org(0)
a.dc_l(SRAM + 0x1000, 'start')
for v in range(2, 256):
    a.dc_l('unexpected')
a.org(0x61 * 4)
a.dc_l('q2_isr')                # QADC queue 2 completion (IVB | %01)
a.org(0x63 * 4)
a.dc_l('q1_isr')                # QADC queue 1 completion (IVB | %11)
a.org(0x83 * 4)
a.dc_l('dasm3_isr')             # CTM4 VECT = %10, submodule 3
a.org(0x8c * 4)
a.dc_l('fcsm_isr')              # CTM4 VECT = %10, submodule 12

a.org(0x1000)
a.label('start')
a.move_b(imm(0x00), SYPCR)      # software watchdog off
a.move_w(imm(0x7f00), SYNCR)    # 16.78 MHz
a.move_w(imm(SRAM >> 16), RAMBAH)
a.move_w(imm(SRAM & 0xffff), RAMBAL)
a.move_w(imm(0x0000), RAMMCR)
a.lea(absl(SRAM + 0x1000), SP)
a.clr_l(Q1F)
a.clr_l(Q2N)
a.clr_l(OVF)
a.clr_l(NCAP)
a.move_w(imm(0xffff), MINP)
a.clr_w(MAXP)

a.move_w(imm(55), SCCR0)        # 9600 baud
a.move_w(imm(0x000c), SCCR1)
a.lea(absl('s_boot'), A0)
a.bsr('puts')

# --- QADC -------------------------------------------------------------
a.move_w(imm(0x0003), QADCMCR)  # SUPV = 0, IARB = 3
a.move_w(imm(0x4360), QADCINT)  # IRLQ1 = 4, IRLQ2 = 3, IVB = $60
a.move_w(imm(0x0077), QACR0)    # PSH = 7, PSL = 7: QCLK = fsys / 16
for i, ccw in enumerate((0x0002, 0x0000, 0x003d, 0x003c, 0x003e, 0x003f)):
    a.move_w(imm(ccw), CCW(i))
a.move_w(imm(0x0002), CCW(8))   # queue 2: AN2, end of table follows
a.move_w(imm(0x003f), CCW(9))
a.move_w(imm(0x2000), SR)
# queue 1: CIE1, SSE1, software triggered single-scan
a.move_w(imm(0xa100), QACR1)
a.move_l(imm(0x40000), D1)
a.label('q1_wait')
a.tst_l(Q1F)
a.bne('q1_done')
a.subq_l(1, D1)
a.bne('q1_wait')
a.lea(absl('s_q1to'), A0)
a.bsr('puts')
a.label('q1_done')
a.lea(absl('s_clt'), A0)
a.bsr('puts')
a.move_w(RJURR(0), D0)
a.bsr('puthex4')
for lab, src in (('s_map', RJURR(1)), ('s_vrh', RJURR(2)),
                 ('s_vrl', RJURR(3)), ('s_mid', RJURR(4)),
                 ('s_ljs', LJSRR(0)), ('s_lju', LJURR(0))):
    a.lea(absl(lab), A0)
    a.bsr('puts')
    a.move_w(src, D0)
    a.bsr('puthex4')
a.bsr('newline')
# queue 2: CIE2, external trigger rising edge continuous, BQ2 = 8
a.move_w(imm(0x9208), QACR2)

# --- CTM4 -------------------------------------------------------------
a.move_w(imm(0x1500), BIUMCR)   # VECT = %10, IARB = 5
a.move_w(imm(FCSM_CFG), FCSM12SIC)
a.move_w(imm(0x0202), MCSM2SIC)  # DRVA (TBB4), PCLK3
a.move_w(imm(0xf800), MCSM2CNT)  # counter and modulus latch
a.move_w(imm(DASM3_CFG), DASM3SIC)
a.move_w(imm(0x0111), DASM10SIC)  # BSL, EDPOL = 1, IPWM: high time
a.move_w(imm(0xf800), DASM4A)     # DIS: prepare A and B1
a.move_w(imm(0xfa00), DASM4B)
a.move_w(imm(0x0008), DASM4SIC)   # TBB4, OPWM 16 bits
a.move_w(imm(0x2000), PWM5A)
a.move_w(imm(0x0800), PWM5B)
a.move_w(imm(0x000b), PWM5SIC)    # EN, POL = 0, fsys / 16
a.move_w(imm(0x0008), CPCR)       # PRUN, DIV23 = 0

# wait 8 FCSM overflows (8 x 65536 x 8 / fsys = 250 ms)
a.label('ovf_wait')
a.cmpi_l(8, OVF)
a.blo('ovf_wait')
a.move_w(imm(0x0000), DASM3SIC)   # stop the captures

a.lea(absl('s_ipm'), A0)
a.bsr('puts')
a.move_w(MINP, D0)
a.bsr('puthex4')
a.lea(absl('s_max'), A0)
a.bsr('puts')
a.move_w(MAXP, D0)
a.bsr('puthex4')
a.lea(absl('s_ipwm'), A0)
a.bsr('puts')
a.move_w(DASM10A, D0)
a.sub_w(DASM10B, D0)
a.move_w(D0, D4)
a.bsr('puthex4')
a.lea(absl('s_q2'), A0)
a.bsr('puts')
a.move_l(Q2N, D0)
a.bsr('puthex4')
a.bsr('newline')

# tooth 555.6 us = 1165 counts at fsys / 8, gap 3 teeth = 3495
a.cmpi_w(1100, MINP)
a.blo('crank_bad')
a.cmpi_w(1230, MINP)
a.bhi('crank_bad')
a.cmpi_w(3300, MAXP)
a.blo('crank_bad')
a.cmpi_w(3700, MAXP)
a.bhi('crank_bad')
a.lea(absl('s_crank'), A0)
a.bsr('puts')
a.label('crank_bad')
# high time 277.8 us = 582.5 counts
a.cmpi_w(550, D4)
a.blo('ipwm_bad')
a.cmpi_w(615, D4)
a.bhi('ipwm_bad')
a.lea(absl('s_ipwmok'), A0)
a.bsr('puts')
a.label('ipwm_bad')
# cam at 3000 rpm: 25 Hz, ~6 queue 2 scans in 250 ms
a.cmpi_l(3, Q2N)
a.blo('etrig_bad')
a.cmpi_w(0x0204, RJURR(8))
a.bne('etrig_bad')
a.lea(absl('s_etrig'), A0)
a.bsr('puts')
a.label('etrig_bad')
a.lea(absl('s_done'), A0)
a.bsr('puts')
a.label('idle')
a.bra('idle')

# --- interrupt handlers -----------------------------------------------
a.label('q1_isr')
a.tst_w(QASR)                   # read CF1 as one ...
a.move_w(imm(0x7fff), QASR)     # ... then write it zero
a.addq_l(1, Q1F)
a.rte()

a.label('q2_isr')
a.tst_w(QASR)
a.move_w(imm(0xdfff), QASR)     # clear CF2
a.addq_l(1, Q2N)
a.rte()

a.label('fcsm_isr')
a.tst_w(FCSM12SIC)
a.move_w(imm(FCSM_CFG), FCSM12SIC)   # COF = 0
a.addq_l(1, OVF)
a.rte()

a.label('dasm3_isr')
a.movem_l([D0], predec(SP))
a.tst_w(DASM3SIC)
a.move_w(imm(DASM3_CFG), DASM3SIC)   # FLAG = 0
a.move_w(DASM3A, D0)
a.sub_w(DASM3B, D0)             # period = A - B (previous capture)
a.addq_l(1, NCAP)
a.cmpi_l(3, NCAP)               # skip the first captures
a.blo('dasm3_end')
a.cmp_w(MINP, D0)
a.bhs('dasm3_nomin')
a.move_w(D0, MINP)
a.label('dasm3_nomin')
a.cmp_w(MAXP, D0)
a.bls('dasm3_end')
a.move_w(D0, MAXP)
a.label('dasm3_end')
a.movem_l(postinc(SP), [D0])
a.rte()

a.label('unexpected')
a.lea(absl('s_exc'), A0)
a.bsr('puts')
a.label('dead')
a.bra('dead')

# --- SCI output -------------------------------------------------------
a.label('putc')
a.btst(0, SCSR)                 # TDRE
a.beq('putc')
a.andi_w(0xff, D0)
a.move_w(D0, SCDR)
a.rts()

a.label('puts')
a.move_b(postinc(A0), D0)
a.beq('puts_end')
a.bsr('putc')
a.bra('puts')
a.label('puts_end')
a.rts()

a.label('newline')
a.moveq(10, D0)
a.bra('putc')

a.label('puthex4')
a.move_w(D0, D2)
a.lsr_w(8, D0)
a.bsr('puthex2')
a.move_w(D2, D0)
a.label('puthex2')
a.move_b(D0, D3)
a.lsr_b(4, D0)
a.bsr('nibble')
a.move_b(D3, D0)
a.label('nibble')
a.andi_w(0x0f, D0)
a.move_b(pcidx('hexdig', D0, 'w', 1), D0)
a.bra('putc')
a.label('hexdig')
a.ascii('0123456789ABCDEF')
a.align(2)

for name, text in (('s_boot', 'MC68376 ADC/CTM TEST\n'),
                   ('s_q1to', 'QADC TIMEOUT\n'),
                   ('s_clt', 'QADC CLT='), ('s_map', ' MAP='),
                   ('s_vrh', ' VRH='), ('s_vrl', ' VRL='),
                   ('s_mid', ' MID='), ('s_ljs', ' LJS='),
                   ('s_lju', ' LJU='),
                   ('s_ipm', 'IPM MIN='), ('s_max', ' MAX='),
                   ('s_ipwm', ' IPWM='), ('s_q2', ' Q2N='),
                   ('s_crank', 'CRANK OK\n'), ('s_ipwmok', 'IPWM OK\n'),
                   ('s_etrig', 'ETRIG OK\n'), ('s_done', 'DONE\n'),
                   ('s_exc', 'UNEXPECTED EXCEPTION\n')):
    a.label(name)
    a.asciz(text)
    a.align(2)

with open(sys.argv[1] if len(sys.argv) > 1 else 'mc68376_adc_ctm.bin',
          'wb') as f:
    f.write(a.bytes())
