#!/usr/bin/env python3
# MC68376 TouCAN test ROM (512 KiB boot flash image on CSBOOT at 0).
#
#   SYNCR  : 16.78 MHz; watchdog off; PIT ~1 ms for time-outs
#   TouCAN : PRESDIV = 1, PROPSEG = 7, PSEG1 = 3, PSEG2 = 2 -> 16 time
#            quanta of 2 system clocks = 524288 bit/s
#   checks : reset values, debug mode (HALT/FRZACK/NOTRDY), loop back
#            transmit/receive with time stamps and frame time, the
#            uninitialised ($0F) and IVBA based interrupt vectors, buffer
#            locking and release by TIMER, no self-reception into a FULL
#            buffer, transmit priority by ID and by buffer (LBUF),
#            extended IDs with the buffer 14 mask, remote frame request /
#            automatic response, TSYNC, a transmission on the CAN bus
#            (LOOP = 0) received back by self-reception, error counter
#            writes in debug mode, SOFTRST and low-power stop.
#
# Every check prints "<NAME> OK" or "FAIL <NAME>=<value>"; the last line
# is "CAN DONE FAILS=<n>".
#
# SPDX-License-Identifier: GPL-2.0-or-later
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m68kasm import *  # noqa: E402,F403

SYNCR = absw(0xfa04)
SYPCR = absw(0xfa21)
PICR = absw(0xfa22)
PITR = absw(0xfa24)
RAMBAH = absw(0xfb44)
RAMBAL = absw(0xfb46)
RAMMCR = absw(0xfb40)
SCCR0 = absw(0xfc08)
SCCR1 = absw(0xfc0a)
SCSR = absw(0xfc0c)
SCDR = absw(0xfc0e)

CANMCR = absw(0xf080)
CANMCR_H = absw(0xf080)         # byte: STOP FRZ - HALT NOTRDY WAKEMSK SOFTRST FRZACK
CANMCR_L = absw(0xf081)
CANICR = absw(0xf084)
CANICR_H = absw(0xf084)
CANCTRL01 = absw(0xf086)
PRESDIV2 = absw(0xf088)
TIMER = absw(0xf08a)
RXGMSKHI = absw(0xf090)
RXGMSKLO = absw(0xf092)
RX14MSKHI = absw(0xf094)
RX14MSKLO = absw(0xf096)
ESTAT = absw(0xf0a0)
IMASK = absw(0xf0a2)
IFLAG = absw(0xf0a4)
ECTR = absw(0xf0a6)


def MB(n, w=0):
    """word w (0 = control/status, 1 = ID_HIGH, 2 = ID_LOW, 3.. data)"""
    return absw(0xf100 + 16 * n + 2 * w)


def std_idh(i, rtr=0):
    return (i & 0x7ff) << 5 | rtr << 4


def ext_idh(i):
    return ((i >> 18) & 0x7ff) << 5 | 0x18 | ((i >> 15) & 7)


def ext_idl(i, rtr=0):
    return (i & 0x7fff) << 1 | rtr


IARB = 5
MCR_RUN = 0x4000 | IARB         # FRZ, HALT clear
MCR_HALT = 0x5000 | IARB        # FRZ, HALT
CTRL1 = 0x07                    # PROPSEG = 7
LOOP = 0x40
TSYNC = 0x20
LBUF = 0x10
CTRL2 = 3 << 3 | 2              # PSEG1 = 3, PSEG2 = 2
ICR = 4 << 8 | 3 << 5           # ILCAN = 4, IVBA = 3 -> vectors $60-$72

SRAM = 0x100000
TICKS = absl(SRAM + 0x10)
FAILS = absl(SRAM + 0x14)
VEC15 = absl(SRAM + 0x18)       # uninitialised vector interrupts
VEC61 = absl(SRAM + 0x1c)       # buffer 1 interrupts
VECBAD = absl(SRAM + 0x20)
T0 = absl(SRAM + 0x24)
T1 = absl(SRAM + 0x28)

a = Asm(0, 0x80000)
strings = {}
nlabel = [0]


def s(text):
    if text not in strings:
        strings[text] = 'str%d' % len(strings)
    return strings[text]


def L():
    nlabel[0] += 1
    return '_l%d' % nlabel[0]


def say(text):
    a.lea(absl(s(text)), A0)
    a.bsr('puts')


def fail_d0(name):
    """print FAIL name=D0.w and count it"""
    a.move_w(D0, D4)
    say('FAIL %s=' % name)
    a.move_w(D4, D0)
    a.bsr('puthex4')
    a.bsr('newline')
    a.addq_l(1, FAILS)


def expect(ea, val, name, mask=0xffff, ok=True):
    """D0 = ea & mask must equal val"""
    good = L()
    a.move_w(ea, D0)
    if mask != 0xffff:
        a.andi_w(mask, D0)
    a.cmpi_w(val, D0)
    a.beq(good)
    fail_d0(name)
    done = L()
    a.bra(done)
    a.label(good)
    if ok:
        say('%s OK\n' % name)
    a.label(done)


def expect_range(lo, hi, name):
    """lo <= D0.w <= hi (unsigned)"""
    bad, done = L(), L()
    a.cmpi_w(lo, D0)
    a.blo(bad)
    a.cmpi_w(hi, D0)
    a.bhi(bad)
    say('%s OK\n' % name)
    a.bra(done)
    a.label(bad)
    fail_d0(name)
    a.label(done)


def wait_iflag(mask, name):
    """wait until all IFLAG bits in mask are set (50 ms time-out)"""
    a.move_w(imm(mask), D1)
    a.bsr('wait_iflag')
    ok = L()
    a.beq(ok)
    a.move_w(IFLAG, D0)
    fail_d0(name + '_TIMEOUT')
    a.label(ok)


def clear_iflag():
    a.move_w(IFLAG, D0)
    a.move_w(imm(0), IFLAG)


# vector table
a.org(0)
a.dc_l(SRAM + 0x1000, 'start')
for v in range(2, 256):
    a.dc_l('unexpected')
a.org(15 * 4)
a.dc_l('isr_uninit')
a.org(0x40 * 4)
a.dc_l('pit_isr')
for v in range(0x60, 0x73):
    a.org(v * 4)
    a.dc_l('isr_bad')
a.org(0x61 * 4)
a.dc_l('isr_mb1')

a.org(0x1000)
a.label('start')
a.move_b(imm(0x00), SYPCR)                  # watchdog off
a.move_w(imm(0x7f00), SYNCR)                # 16.78 MHz
a.move_w(imm(SRAM >> 16), RAMBAH)
a.move_w(imm(SRAM & 0xffff), RAMBAL)
a.move_w(imm(0x0000), RAMMCR)
a.lea(absl(SRAM + 0x1000), SP)
for v in (TICKS, FAILS, VEC15, VEC61, VECBAD):
    a.clr_l(v)
a.move_w(imm(55), SCCR0)                    # 9600 baud
a.move_w(imm(0x000c), SCCR1)
a.move_w(imm(0x0640), PICR)                 # PIT level 6, vector $40
a.move_w(imm(0x0008), PITR)
a.move_w(imm(0x2000), SR)
say('CAN TEST\n')

# --- reset state (D.10.1, D.10.3, D.10.9, 13.5.1) -----------------------
expect(CANMCR, 0x5980, 'RESET MCR')
expect(CANICR, 0x000f, 'RESET ICR')
expect(RXGMSKHI, 0xffef, 'RESET GMSKHI')
expect(RXGMSKLO, 0xfffe, 'RESET GMSKLO')
# RTR mask bits read 0, IDE mask bit reads 1
a.move_w(imm(0x0010), RX14MSKHI)
expect(RX14MSKHI, 0x0008, 'MASK FIXED BITS')

# --- configuration in debug mode ----------------------------------------
a.move_w(imm(MCR_HALT), CANMCR)
a.move_w(imm(LOOP | CTRL1), CANCTRL01)      # CANCTRL0 = 0
a.move_w(imm(0x0100 | CTRL2), PRESDIV2)     # PRESDIV = 1
for n in range(16):
    a.move_w(imm(0), MB(n))
a.move_w(imm(0xffff), RX14MSKHI)            # ID[28:15] checked
a.move_w(imm(0xffe0), RX14MSKLO)            # ID[3:0] don't care
# MB0: transmit, ID $123, 8 bytes
a.move_w(imm(0x0080), MB(0))
a.move_w(imm(std_idh(0x123)), MB(0, 1))
for i, w in enumerate((0x1122, 0x3344, 0x5566, 0x7788)):
    a.move_w(imm(w), MB(0, 3 + i))
# MB1: receive, ID $123
a.move_w(imm(std_idh(0x123)), MB(1, 1))
a.move_w(imm(0x0040), MB(1))
a.move_w(imm(0x0002), IMASK)                # MB1 interrupt
# CANICR level only (byte write): IVBA not initialised -> vector $0F
a.move_b(imm(4), CANICR_H)
# timer does not run in debug mode
a.move_w(TIMER, D5)
a.move_l(TICKS, D1)
a.addq_l(3, D1)
a.label('frz_wait')
a.cmp_l(TICKS, D1)
a.bhi('frz_wait')
a.move_w(TIMER, D0)
a.sub_w(D5, D0)
expect_range(0, 0, 'TIMER FROZEN')

# leave debug mode: NOTRDY clears after 11 recessive bits
a.move_w(imm(MCR_RUN), CANMCR)
a.bsr('wait_ready')
expect(CANMCR, MCR_RUN, 'SYNC')
expect(ESTAT, 0x0080, 'ESTAT IDLE')

# bit rate: TIMER over 20 PIT periods (20 x 976.6 us x 524288 = 10240)
a.move_l(TICKS, D1)
a.label('tick_edge')
a.cmp_l(TICKS, D1)
a.beq('tick_edge')
a.move_w(TIMER, D5)
a.addi_l(21, D1)
a.label('tick_20')
a.cmp_l(TICKS, D1)
a.bhi('tick_20')
a.move_w(TIMER, D0)
a.sub_w(D5, D0)
expect_range(9200, 11300, 'BIT RATE')

# --- loop back transmission ---------------------------------------------
a.move_w(TIMER, D0)
a.move_w(D0, T0)
a.move_w(imm(0x00c8), MB(0))                # transmit once, 8 bytes
a.bsr('wait_irq15')
a.move_w(TIMER, D0)
a.move_w(D0, T1)
a.tst_l(VEC15)
ok = L()
a.bne(ok)
a.move_w(IFLAG, D0)
fail_d0('UNINIT VECTOR')
a.label(ok)
say('UNINIT VECTOR OK\n')
say('LPB CS=')
a.move_w(MB(1), D0)
a.bsr('puthex4')
say(' ID=')
a.move_w(MB(1, 1), D0)
a.bsr('puthex4')
say(' D=')
for i in range(4):
    a.move_w(MB(1, 3 + i), D0)
    a.bsr('puthex4')
a.bsr('newline')
expect(MB(1), 0x0028, 'RX FULL', mask=0x00ff)
expect(MB(1, 3), 0x1122, 'RX DATA0', ok=False)
expect(MB(1, 6), 0x7788, 'RX DATA3', ok=False)
expect(MB(0), 0x0088, 'TX NOTREADY', mask=0x00ff)
a.move_w(TIMER, D0)                         # release the lock
expect(IFLAG, 0x0001, 'IFLAG')
# time stamp: TIMER at the identifier field, 16 bits in ID_LOW
a.move_w(MB(1, 2), D0)
a.cmp_w(MB(0, 2), D0)
ok = L()
a.beq(ok)
fail_d0('TX/RX STAMP')
a.label(ok)
a.move_w(MB(1, 2), D0)
a.lsr_w(8, D0)
a.move_w(MB(1), D1)
a.lsr_w(8, D1)
a.cmp_b(D1, D0)
ok = L()
a.beq(ok)
fail_d0('STAMP HIGH BYTE')
a.label(ok)
clear_iflag()

# --- vector from IVBA, locking ------------------------------------------
a.move_w(imm(ICR), CANICR)
a.move_w(imm(0x0040), MB(1))                # EMPTY again
a.move_w(MB(1), D0)                         # lock MB1
a.move_w(imm(0xa1a2), MB(0, 3))
a.move_w(imm(0x00c8), MB(0))
wait_iflag(0x0001, 'LOCK TX')
expect(IFLAG, 0x0001, 'LOCKED NO FLAG', ok=False)
expect(MB(1, 3), 0x1122, 'LOCKED NO DATA', ok=False)
a.move_w(TIMER, D0)                         # release: frame moves in
a.bsr('wait_irq61')
expect(MB(1, 3), 0xa1a2, 'LOCK RELEASE')
a.move_w(TIMER, D0)
a.tst_l(VEC61)
ok = L()
a.bne(ok)
fail_d0('IVBA VECTOR')
a.label(ok)
say('IVBA VECTOR OK\n')
clear_iflag()

# --- own frames are not received into a FULL buffer (13.5.3.2) ----------
a.move_w(imm(0x00c8), MB(0))
wait_iflag(0x0001, 'SELF TX')
expect(MB(1), 0x0028, 'NO SELF OVERRUN', mask=0x00ff)
a.move_w(TIMER, D0)
clear_iflag()

# --- transmit priority: lowest ID, then lowest buffer (LBUF) ------------
a.move_w(imm(0x0000), MB(1))
a.bsr('halt')
for lbuf, name in ((0, 'ARB ID'), (LBUF, 'ARB LBUF')):
    a.move_w(imm(LOOP | lbuf | CTRL1), CANCTRL01)
    for n, i in ((2, 0x200), (3, 0x100)):
        a.move_w(imm(0x0080), MB(n))
        a.move_w(imm(std_idh(i)), MB(n, 1))
        a.move_w(imm(0x00c1), MB(n))
    a.bsr('run')
    wait_iflag(0x000c, name)
    # back to back: the stamps differ by one frame (44 + 8 + 3 = 55 bits)
    a.move_w(MB(2, 2), D0)                  # stamp MB2 - stamp MB3
    a.sub_w(MB(3, 2), D0)
    a.move_w(TIMER, D1)
    a.move_w(D0, D5)
    if lbuf:
        expect(D5, (-55) & 0xffff, name)    # MB2 first
    else:
        expect(D5, 55, name)                # MB3 (ID $100) first
    clear_iflag()
    a.bsr('halt')

# --- extended IDs, buffer 14 mask ---------------------------------------
EXT_RX, EXT_TX = 0x12345670, 0x12345675
a.move_w(imm(LOOP | CTRL1), CANCTRL01)
a.move_w(imm(ext_idh(EXT_RX)), MB(5, 1))    # global mask: all bits
a.move_w(imm(ext_idl(EXT_RX)), MB(5, 2))
a.move_w(imm(0x0040), MB(5))
a.move_w(imm(ext_idh(EXT_RX)), MB(14, 1))   # RX14MSK: ID[3:0] ignored
a.move_w(imm(ext_idl(EXT_RX)), MB(14, 2))
a.move_w(imm(0x0040), MB(14))
a.move_w(imm(0x0080), MB(4))
a.move_w(imm(ext_idh(EXT_TX)), MB(4, 1))
a.move_w(imm(ext_idl(EXT_TX)), MB(4, 2))
a.move_w(imm(0xabcd), MB(4, 3))
a.move_w(imm(0x00c2), MB(4))
a.bsr('run')
wait_iflag(0x4010, 'EXT')
expect(MB(14), 0x0022, 'EXT RX14', mask=0x00ff)
a.move_w(TIMER, D0)
expect(MB(14, 2), ext_idl(EXT_TX), 'EXT ID STORED')
expect(MB(14, 3), 0xabcd, 'EXT DATA')
expect(IFLAG, 0x4010, 'EXT MB5 NO MATCH')
a.move_w(TIMER, D0)
clear_iflag()

# --- remote frame and automatic response (13.5.5) -----------------------
a.move_w(imm(0x0080), MB(6))
a.move_w(imm(std_idh(0x300)), MB(6, 1))
a.move_w(imm(0x5aa5), MB(6, 3))
a.move_w(imm(0x00a2), MB(6))                # respond to remote frames
a.move_w(imm(0x0080), MB(7))
a.move_w(imm(std_idh(0x300, 1)), MB(7, 1))
a.move_w(imm(0x00c0), MB(7))                # remote frame once
wait_iflag(0x00c0, 'REMOTE')
wait_iflag(0x0040, 'REMOTE RESPONSE')
expect(MB(6), 0x00a2, 'RESPONSE CODE', mask=0x00ff)
expect(MB(7), 0x0022, 'REMOTE RX', mask=0x00ff)
a.move_w(TIMER, D0)
expect(MB(7, 3), 0x5aa5, 'REMOTE DATA')
expect(MB(7, 1), std_idh(0x300), 'REMOTE ID', ok=False)
a.move_w(TIMER, D0)
clear_iflag()

# --- TSYNC: reception into MB0 resets the timer -------------------------
a.bsr('halt')
a.move_w(imm(LOOP | TSYNC | CTRL1), CANCTRL01)
a.move_w(imm(0x0000), MB(0))
a.move_w(imm(std_idh(0x050)), MB(0, 1))
a.move_w(imm(0x0040), MB(0))
a.move_w(imm(0x0080), MB(10))
a.move_w(imm(std_idh(0x050)), MB(10, 1))
a.move_w(imm(0x00c0), MB(10))
a.move_w(imm(0x8000), TIMER)
a.bsr('run')
wait_iflag(0x0001, 'TSYNC')
a.move_w(TIMER, D0)
expect_range(0, 0x3fff, 'TSYNC')
clear_iflag()

# --- CAN bus (no loop back): sent on the bus, received back -------------
a.bsr('halt')
a.move_w(imm(CTRL1), CANCTRL01)
a.move_w(imm(0x0080), MB(8))
a.move_w(imm(std_idh(0x7e0)), MB(8, 1))
for i, w in enumerate((0x5145, 0x4d55, 0x2d43, 0x414e)):   # "QEMU-CAN"
    a.move_w(imm(w), MB(8, 3 + i))
a.move_w(imm(std_idh(0x7e0)), MB(9, 1))
a.move_w(imm(0x0040), MB(9))
a.bsr('run')
a.move_w(imm(0x00c8), MB(8))
wait_iflag(0x0300, 'BUS')
expect(MB(9), 0x0028, 'BUS SELF RX', mask=0x00ff)
a.move_w(TIMER, D0)
clear_iflag()

# --- error counters writable in debug mode, ESTAT -----------------------
a.move_w(imm(0x1234), ECTR)                 # ignored while running
expect(ECTR, 0x0000, 'ECTR RO')
a.bsr('halt')
a.move_w(imm(0x8060), ECTR)
expect(ECTR, 0x8060, 'ECTR DEBUG', ok=False)
# TXWARN, RXWARN, IDLE, FCS = error passive
expect(ESTAT, 0x0390, 'ESTAT PASSIVE')
a.move_w(imm(0x0000), ECTR)

# --- SOFTRST ------------------------------------------------------------
a.move_w(imm(0x0200 | MCR_RUN), CANMCR)
expect(CANMCR, 0x5980, 'SOFTRST MCR', ok=False)
expect(CANICR, 0x000f, 'SOFTRST ICR', ok=False)
expect(IMASK, 0x0000, 'SOFTRST')

# --- low-power stop: only CANMCR accessible -----------------------------
a.move_w(imm(0x8000 | IARB), CANMCR)
expect(CANMCR, 0x8810 | IARB, 'STOPACK', ok=False)
expect(CANCTRL01, 0x0000, 'STOP ACCESS')
a.move_w(imm(IARB), CANMCR)
a.bsr('wait_ready')
expect(CANCTRL01, CTRL1, 'STOP EXIT')

a.tst_l(VECBAD)
ok = L()
a.beq(ok)
say('FAIL BAD VECTOR\n')
a.addq_l(1, FAILS)
a.label(ok)
say('CAN DONE FAILS=')
a.move_l(FAILS, D0)
a.bsr('puthex2')
a.bsr('newline')
a.label('idle')
a.bra('idle')

# --- subroutines ------------------------------------------------------
# wait_iflag: D1.w = mask; Z set on success
a.label('wait_iflag')
a.move_l(TICKS, D2)
a.addi_l(50, D2)
a.label('wif_loop')
a.move_w(IFLAG, D0)
a.and_w(D1, D0)
a.cmp_w(D1, D0)
a.beq('wif_done')
a.cmp_l(TICKS, D2)
a.bhi('wif_loop')
a.moveq(1, D0)                  # clears Z
a.label('wif_done')
a.rts()


def wait_flag(name, var):
    a.label(name)
    a.move_l(TICKS, D2)
    a.addi_l(50, D2)
    a.label(name + '_l')
    a.tst_l(var)
    a.bne(name + '_d')
    a.cmp_l(TICKS, D2)
    a.bhi(name + '_l')
    a.label(name + '_d')
    a.rts()


wait_flag('wait_irq15', VEC15)
wait_flag('wait_irq61', VEC61)

# halt: enter debug mode, wait for FRZACK
a.label('halt')
a.move_w(imm(MCR_HALT), CANMCR)
a.label('halt_l')
a.btst(0, CANMCR_H)
a.beq('halt_l')
a.rts()

# run: leave debug mode, wait until synchronised
a.label('run')
a.move_w(imm(MCR_RUN), CANMCR)
a.label('wait_ready')
a.move_l(TICKS, D2)
a.addi_l(20, D2)
a.label('rdy_l')
a.btst(3, CANMCR_H)
a.beq('rdy_d')
a.cmp_l(TICKS, D2)
a.bhi('rdy_l')
a.label('rdy_d')
a.rts()

a.label('putc')
a.btst(0, SCSR)
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

a.label('pit_isr')
a.addq_l(1, TICKS)
a.rte()

# interrupt handlers clear IFLAG bit 1 (read as one, write zero)
for name, var in (('isr_uninit', VEC15), ('isr_mb1', VEC61)):
    a.label(name)
    a.move_w(IFLAG, D7)
    a.move_w(imm(0xfffd), IFLAG)
    a.addq_l(1, var)
    a.rte()

a.label('isr_bad')
a.move_w(imm(0), IMASK)
a.addq_l(1, VECBAD)
a.rte()

a.label('unexpected')
say('UNEXPECTED EXCEPTION\n')
a.label('dead')
a.bra('dead')

for text, name in strings.items():
    a.label(name)
    a.asciz(text)
    a.align(2)

with open(sys.argv[1] if len(sys.argv) > 1 else 'mc68376_can.bin', 'wb') as f:
    f.write(a.bytes())
