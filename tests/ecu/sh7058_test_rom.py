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

# usage: sh7058_test_rom.py out.bin [7058|7055|7054|7052] [180|350]
#   the last argument selects the flash generation the ROM tests (default:
#   180 nm on the SH7058, 350 nm on the others; nothing on the SH7052)
VARIANT = sys.argv[2] if len(sys.argv) > 2 else '7058'
FLASH_NODE = sys.argv[3] if len(sys.argv) > 3 else \
    {'7058': '180', '7052': None}.get(VARIANT, '350')
RAM, RAM_TOP, ROM_SIZE, HCAN2 = {
    '7058': (0xFFFF0000, 0xFFFFC000, 1 << 20, True),
    '7055': (0xFFFF6000, 0xFFFFE000, 512 << 10, False),
    '7054': (0xFFFF8000, 0xFFFFC000, 384 << 10, False),
    '7052': (0xFFFF8000, 0xFFFFB000, 256 << 10, False),
}[VARIANT]

a = Asm(ROM_SIZE)
p = Pool(a, 'p')
TICKS = RAM + 0x100       # CMT 1 ms ticks
TEETH = RAM + 0x104       # crank edges

SCI0 = 0xFFFFF000
SSR0 = SCI0 + 4
TDR0 = SCI0 + 3

# vector table
a.org(0)
a.long('reset')
a.long(RAM_TOP - 0x10)
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

# --- flash erase/program, the way the npkern reflash kernel does it
FLASH_TABLE = 0x7000          # 128 bytes of data to program
FLASH_OLD = 0x12345678        # initial contents of the test unit


def flash_test():
    f = Pool(a, 'f')
    n = [0]

    def expect(ok_if_t, code):
        n[0] += 1
        lbl = 'fl_ok%d' % n[0]
        (a.bt if ok_if_t else a.bf)(lbl)
        a.mov_i(code, 4); a.bra('flash_err'); a.nop()
        a.label(lbl)

    def ldb(disp):              # r0 = sign-extended byte @(disp, r9)
        a.raw(0x8490 | disp)

    def stb(disp, val=None):    # byte @(disp, r9) = r0 (or val)
        if val is not None:
            a.mov_i(val if val < 0x80 else val - 0x100, 0)
        a.raw(0x8090 | disp)

    def modb(disp, setb=0, clrb=0):
        ldb(disp)
        if setb:
            a.or_i(setb)
        if clrb:
            a.and_i(0xff & ~clrb)
        stb(disp)

    def call(base_reg, off):    # jsr @(base_reg + off)
        a.mov(base_reg, 1); a.add_i(off, 1); a.jsr(1); a.nop()

    def loop(name, count, body):
        a.mov_i(count, 3)
        a.label(name)
        body()
        a.dt(3); a.bf(name)

    if VARIANT == '7058':
        dest, blk, ftdar_e, ftdar_w = 0xE0080, 15, 2, 4
    elif VARIANT == '7055':
        dest, blk, ftdar_e, ftdar_w = 0x70080, 15, 4, 5
    else:
        dest, blk, ftdar_e, ftdar_w = 0x50080, 13, None, None
    src = RAM + 0x400

    a.lit(0xFFFFE800, 9, f)
    a.lit(dest, 13, f)
    if FLASH_NODE == '180':
        # 180/350 nm detection (FKEY is RW), FCCS == FWE, RAMER == 0
        stb(4, 0x33); ldb(4); a.cmpeq_i(0x33); expect(True, 1)
        ldb(0); a.cmpeq_i(0x80); expect(True, 2)
        a.lit(0xFFFFEC26, 1, f); a.ld('w', 1, 0); a.tst(0, 0)
        expect(True, 3)
        # download the erase and the write program
        dl = ((10, RAM + ftdar_e * 0x800, 0, 1, ftdar_e, 4),
              (12, RAM + ftdar_w * 0x800, 1, 0, ftdar_w, 5))
        for reg, base, fpcs, fecs, ftdar, code in dl:
            a.lit(base, reg, f)
            a.mov_i(-1, 0); a.st('b', 0, reg)       # DPFR = H'FF
            stb(1, fpcs); stb(2, fecs); stb(6, ftdar)
            stb(4, 0xA5)
            modb(0, setb=1)                         # FCCS.SCO = 1
            for i in range(8):
                a.nop()
            a.ld('b', reg, 0); a.tst_i(0xff); expect(True, code)
        stb(4, 0)
        # initialise both programs, FPEFEQ = 40 MHz
        for reg, code in ((10, 6), (12, 7)):
            a.lit(4000, 4, f); a.mov_i(0, 5)
            call(reg, 32)
            a.tst(0, 0); expect(True, code)
    else:
        # npkern's 350 nm detection: FLMCR2.SWE2 settable, no FKEY
        modb(1, setb=0x40); ldb(1); a.tst_i(0x40); expect(False, 1)
        modb(1, clrb=0x40)
        stb(4, 0x33); ldb(4); a.cmpeq_i(0x33); expect(False, 2)
        ldb(0); a.tst_i(0x80); expect(False, 3)     # FWE
        ldb(1); a.tst_i(0x80); expect(True, 4)      # FLER

    # original contents, and plain CPU writes must not change them
    a.ld('l', 13, 0); a.lit(FLASH_OLD, 1, f); a.cmpeq(1, 0)
    expect(True, 8)
    a.mov_i(0, 0); a.st('l', 0, 13)
    a.ld('l', 13, 0); a.cmpeq(1, 0); expect(True, 9)

    # erase the block
    if FLASH_NODE == '180':
        stb(4, 0x5A)
        a.mov_i(blk, 4); call(10, 16)
        a.mov(0, 11); stb(4, 0)
        a.tst(11, 11); expect(True, 10)
        a.mov(13, 2)

        def body():
            a.raw(0x6026); a.cmpeq_i(0xff); expect(True, 11)
        loop('fl_ev', 32, body)
    else:
        modb(1, setb=0x40)                          # SWE2
        stb(3, 0); stb(2, 0); stb(3, 1 << (blk - 8))
        modb(1, setb=0x20); modb(1, setb=0x02)      # ESU2, E2
        modb(1, clrb=0x02); modb(1, clrb=0x20)
        stb(2, 0); stb(3, 0)
        modb(1, setb=0x08)                          # EV2
        a.mov(13, 2)

        def body():
            a.mov_i(-1, 1); a.st('l', 1, 2)         # dummy write
            a.raw(0x6026); a.cmpeq_i(0xff); expect(True, 11)
        loop('fl_ev', 32, body)
        modb(1, clrb=0x08)

    # program 128 bytes
    if FLASH_NODE == '180':
        a.lit(FLASH_TABLE, 2, f); a.lit(src, 5, f)

        def body():
            a.raw(0x6026); a.st('l', 0, 5); a.add_i(4, 5)
        loop('fl_cp', 32, body)
        # FKEY not H'5A: must fail
        a.lit(src, 4, f); a.mov(13, 5); call(12, 16)
        a.tst(0, 0); expect(False, 12)
        stb(4, 0x5A)
        a.lit(src, 4, f); a.mov(13, 5); call(12, 16)
        a.mov(0, 11); stb(4, 0)
        a.tst(11, 11); expect(True, 13)
    else:
        a.lit(FLASH_TABLE, 2, f); a.mov(13, 5)

        def body():
            a.raw(0x6024); a.st('b', 0, 5); a.add_i(1, 5)
        a.mov_i(0x7f, 3); a.add_i(1, 3)             # 128 bytes
        a.label('fl_lat')
        body()
        a.dt(3); a.bf('fl_lat')
        modb(1, setb=0x10); modb(1, setb=0x01)      # PSU2, P2
        modb(1, clrb=0x01); modb(1, clrb=0x10)
        modb(1, setb=0x04)                          # PV2
        a.mov(13, 2); a.lit(FLASH_TABLE, 6, f)

        def body():
            a.mov_i(-1, 1); a.st('l', 1, 2)         # dummy write
            a.raw(0x6026); a.raw(0x6166); a.cmpeq(1, 0)
            expect(True, 13)
        loop('fl_pv', 32, body)
        modb(1, clrb=0x04)
        modb(1, clrb=0x40)

    # read back
    a.mov(13, 2); a.lit(FLASH_TABLE, 6, f)

    def body():
        a.raw(0x6026); a.raw(0x6166); a.cmpeq(1, 0); expect(True, 14)
    loop('fl_rd', 32, body)

    a.lit(ord('F') << 24 | ord('L') << 16 | ord('A') << 8 | ord('S'), 4, f)
    a.bsr('putc4'); a.nop()
    a.lit(ord('H') << 24 | ord(' ') << 16 | ord('O') << 8 | ord('K'), 4, f)
    a.bsr('putc4'); a.nop()
    a.mov_i(10, 4); a.bsr('putc'); a.nop()
    a.bra('flash_done'); a.nop()

    a.label('flash_err')
    a.mov(4, 11)
    a.lit(ord('F') << 24 | ord('L') << 16 | ord('A') << 8 | ord('S'), 4, f)
    a.bsr('putc4'); a.nop()
    a.lit(ord('H') << 24 | ord(' ') << 16 | ord('E') << 8 | ord('R'), 4, f)
    a.bsr('putc4'); a.nop()
    a.lit(ord('R') << 24 | ord(' ') << 16 | ord('0') << 8 | ord('x'), 4, f)
    a.bsr('putc4'); a.nop()
    a.mov(11, 4); a.bsr('puthex'); a.nop()
    a.mov_i(10, 4); a.bsr('putc'); a.nop()
    a.bra('flash_done'); a.nop()
    f.emit()
    a.label('flash_done')

    here = a.pc
    a.org(FLASH_TABLE)
    a.bytes_([(i * 7 + 3) & 0xff for i in range(128)])
    a.org(dest)
    for i in range(32):
        a.long(FLASH_OLD)
    a.org(here)


if FLASH_NODE:
    flash_test()

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

if HCAN2:
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
    # HCAN1: MB2 receives id 0x123 (bus loopback test, needs both on one bus)
    H1 = 0xFFFFD800
    a.lit(H1, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)
    H1MB2 = H1 + 0x100 + 0x40
    a.lit(H1MB2, 9, p); a.lit(0x123 << 4, 0, p); a.st('w', 0, 9)
    a.lit(H1MB2 + 2, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)
    a.lit(H1MB2 + 4, 9, p); a.lit(0x0408, 0, p); a.st('w', 0, 9)
    a.lit(H1MB2 + 0x10, 9, p); a.mov_i(0, 0); a.st('w', 0, 9)
    a.lit(H1MB2 + 0x12, 9, p); a.st('w', 0, 9)
    a.lit(H + 0x22, 9, p); a.mov_i(2, 0); a.st('w', 0, 9)          # TXPR0 MB1
    a.lit(H + 0x32, 9, p); a.ld('w', 9, 0); a.tst_i(2); a.bt('cantxfail')
    a.lit(ord('C') << 24 | ord('A') << 16 | ord('N') << 8 | ord('T'), 4, p)
    a.bsr('putc4'); a.nop()
    a.label('cantxfail')
    a.lit(H1 + 0x42, 9, p); a.ld('w', 9, 0); a.tst_i(4); a.bt('canrxfail')
    a.lit(ord('R') << 24 | ord('X') << 16 | ord('1') << 8 | ord('='), 4, p)
    a.bsr('putc4'); a.nop()
    a.lit(H1MB2 + 8, 9, p); a.ld('l', 9, 4); a.bsr('puthex'); a.nop()
    a.mov_i(10, 4); a.bsr('putc'); a.nop()
    a.label('canrxfail')


else:
    # --- HCAN0 (HCAN, 16 mailboxes): MB1 tx id 0x123; HCAN1 MB2 rx 0x123
    H = 0xFFFFE400
    H1 = 0xFFFFE600
    a.lit(H, 9, p); a.mov_i(0, 0); a.st('b', 0, 9)                 # MCR = 0
    a.lit(H1, 9, p); a.st('b', 0, 9)
    a.label('canwait')
    a.lit(H + 1, 9, p); a.ld('b', 9, 0); a.tst_i(0x08); a.bf('canwait')
    # MC1: DLC 8, std id 0x123 (MCx[5]: id[2:0]<<5, MCx[6]: id[10:3])
    a.lit(H + 0x28, 9, p); a.mov_i(8, 0); a.st('b', 0, 9)
    a.lit(H + 0x2c, 9, p); a.lit((0x123 & 7) << 5, 0, p); a.st('b', 0, 9)
    a.lit(H + 0x2d, 9, p); a.lit(0x123 >> 3, 0, p); a.st('b', 0, 9)
    a.lit(H + 0xb8, 9, p); a.lit(0x51454D55, 0, p); a.st('l', 0, 9)  # MD1
    a.lit(H + 0xbc, 9, p); a.lit(0x45435521, 0, p); a.st('l', 0, 9)
    # HCAN1 MB2 receive (MBCR bit for MB2 = 0x0400), id 0x123
    a.lit(H1 + 4, 9, p); a.lit(0x0400, 0, p); a.st('w', 0, 9)
    a.lit(H1 + 0x34, 9, p); a.lit((0x123 & 7) << 5, 0, p); a.st('b', 0, 9)
    a.lit(H1 + 0x35, 9, p); a.lit(0x123 >> 3, 0, p); a.st('b', 0, 9)
    a.lit(H + 6, 9, p); a.lit(0x0200, 0, p); a.st('w', 0, 9)       # TXPR MB1
    a.lit(H + 0xa, 9, p); a.ld('w', 9, 0); a.lit(0x0200, 1, p); a.tst(1, 0)
    a.bt('cantxfail')
    a.lit(ord('C') << 24 | ord('A') << 16 | ord('N') << 8 | ord('T'), 4, p)
    a.bsr('putc4'); a.nop()
    a.label('cantxfail')
    a.lit(H1 + 0xe, 9, p); a.ld('w', 9, 0); a.lit(0x0400, 1, p); a.tst(1, 0)
    a.bt('canrxfail')
    a.lit(ord('R') << 24 | ord('X') << 16 | ord('1') << 8 | ord('='), 4, p)
    a.bsr('putc4'); a.nop()
    a.lit(H1 + 0xc0, 9, p); a.ld('l', 9, 4); a.bsr('puthex'); a.nop()
    a.mov_i(10, 4); a.bsr('putc'); a.nop()
    a.label('canrxfail')
    MB2 = None


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
if not HCAN2:
    a.bra('wait'); a.nop()
a.lit(0xFFFFD042, 10, p); a.ld('w', 10, 0); a.tst_i(4); a.bt('wait')
a.mov_i(4, 0); a.st('w', 0, 10)                                 # clear RXPR
a.lit(ord('R') << 24 | ord('X') << 16 | ord('O') << 8 | ord('K'), 4, p)
a.bsr('putc4'); a.nop()
a.lit(0xFFFFD148, 10, p); a.ld('l', 10, 4); a.bsr('puthex'); a.nop()
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
