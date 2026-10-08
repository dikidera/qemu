#!/usr/bin/env python3
# CPU32 core self-test ROM for the "cpu32-test" machine.
#
#   cpu32_test_rom.py out.bin
#   qemu-system-m68k -M cpu32-test -bios out.bin -serial stdio -display none
#
# Prints "CPU32 OK" and exits with status 0, or prints "FAIL nn" (nn = hex
# test number) / "UNEXP vvvv" (unexpected exception, format/vector word)
# and exits with status 1.  Expected values come from the CPU32 Reference
# Manual (CPU32RM); see the comments at each test.
# SPDX-License-Identifier: GPL-2.0-or-later
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m68kasm import *   # noqa: E402,F403

OUT = 0xF000            # (xxx).W -> 0xFFFFF000 output port
EXIT = 0xF004           # exit port
IRQ = 0xF008            # interrupt request port: level << 8 | vector

SSP = 0x80000
USTACK = 0x70000
VBR2 = 0x8000           # relocated vector table

# variables (absolute short addresses)
RESUME = 0x1000         # where the generic handler returns to
EXC_SR = 0x1004
EXC_PC = 0x1006
EXC_FV = 0x100A
EXC_X1 = 0x100C         # frame + 8
EXC_X2 = 0x1010         # frame + $10
EXC_SSW = 0x1014        # frame + $16
EXC_CNT = 0x1018
IRQ_CNT = 0x101C
IRQ_SR = 0x1020
IRQ_CLR = 0x1022
VBR_HIT = 0x1024
SAVE_SP = 0x1028
SCRATCH = 0x2000

a = Asm(0, 0x10000, fill=0xff)


def sync_irq(lbl):
    """end the translation block so a just-raised interrupt is taken"""
    a.bra_w(lbl)
    a.label(lbl)


def fail_ne():
    a.bne('fail')


def chk_l(val, reg):
    a.cmpi_l(val, reg)
    fail_ne()


def chk_w(val, reg):
    a.cmpi_w(val, reg)
    fail_ne()


def chk_mem_l(val, addr):
    a.cmpi_l(val, absw(addr))
    fail_ne()


def chk_mem_w(val, addr):
    a.cmpi_w(val, absw(addr))
    fail_ne()


n_test = [0]


def test(n):
    """start test n: the number is kept in D7 for the failure report"""
    n_test[0] = n
    a.moveq(n, D7)
    a.clr_l(absw(EXC_CNT))


def trigger(label_resume, emit):
    """arm the generic handler, emit the faulting code, define resume"""
    a.move_l(imm(label_resume), absw(RESUME))
    emit()
    a.label(label_resume)


def expect_exc(fv, pc=None, x1=None):
    """exactly one exception with format/vector word fv was taken"""
    chk_mem_l(1, EXC_CNT)
    chk_mem_w(fv, EXC_FV)
    if pc is not None:
        chk_mem_l(pc, EXC_PC)
    if x1 is not None:
        chk_mem_l(x1, EXC_X1)


# --------------------------------------------------------------------------
# vectors: SSP, PC, then everything to the generic handler
a.org(0)
a.dc_l(SSP, 'start')
for v in range(2, 256):
    a.dc_l('exc')

a.org(0x400)
# generic exception handler: records the frame, returns to RESUME in
# supervisor mode.  An exception with RESUME == 0 is unexpected.
a.label('exc')
a.move_w(ind(SP), absw(EXC_SR))
a.move_l(disp(2, SP), absw(EXC_PC))
a.move_w(disp(6, SP), absw(EXC_FV))
a.move_l(disp(8, SP), absw(EXC_X1))
a.move_l(disp(0x10, SP), absw(EXC_X2))
a.move_w(disp(0x16, SP), absw(EXC_SSW))
a.addq_l(1, absw(EXC_CNT))
a.tst_l(absw(RESUME))
a.beq('unexpected')
a.ori_w(0x2000, ind(SP))
a.move_l(absw(RESUME), disp(2, SP))
a.clr_l(absw(RESUME))
a.rte()

a.label('unexpected')
a.lea(pcrel('s_unexp'), A0)
a.bsr('puts')
a.move_w(absw(EXC_FV), D0)
a.bsr('puthex4')
a.moveq(0x20, D0)
a.bsr('putc')
a.bra('fail')

# interrupt handler (vectors 64..66): count, record SR, optionally clear
a.label('irq')
a.move_w(SR, absw(IRQ_SR))
a.andi_w(0xff00, absw(IRQ_SR))
a.addq_l(1, absw(IRQ_CNT))
a.tst_w(absw(IRQ_CLR))
a.beq_s('irq_keep')
a.clr_w(absw(IRQ))
a.label('irq_keep')
a.rte()

# handler installed in the relocated vector table
a.label('vbr_h')
a.addq_l(1, absw(VBR_HIT))
a.rte()

# -- I/O helpers -------------------------------------------------------------
a.label('putc')                     # D0.b
a.move_b(D0, absw(OUT))
a.rts()

a.label('puts')                     # A0 -> NUL-terminated string
a.label('puts_l')
a.move_b(postinc(A0), D0)
a.beq_s('puts_e')
a.bsr('putc')
a.bra_s('puts_l')
a.label('puts_e')
a.rts()

a.label('puthex4')                  # D0.w as 4 hex digits
a.move_l(D2, predec(SP))
a.move_w(D0, D2)
a.moveq(3, D1)
a.label('ph_l')
a.rol_w(4, D2)
a.move_w(D2, D0)
a.andi_w(0xf, D0)
a.lea(absl('hexdig'), A1)
a.move_b(idx(0, A1, D0, 'w'), D0)
a.bsr('putc')
a.dbra(D1, 'ph_l')
a.move_l(postinc(SP), D2)
a.rts()

a.label('fail')
a.lea(pcrel('s_fail'), A0)
a.bsr('puts')
a.move_w(D7, D0)
a.lsl_w(8, D0)
a.moveq(1, D1)
a.move_w(D0, D2)
a.label('pf_l')
a.rol_w(4, D2)
a.move_w(D2, D0)
a.andi_w(0xf, D0)
a.lea(absl('hexdig'), A1)
a.move_b(idx(0, A1, D0, 'w'), D0)
a.bsr('putc')
a.dbra(D1, 'pf_l')
a.label('die')
a.moveq(10, D0)
a.bsr('putc')
a.move_l(imm(1), absw(EXIT))
a.label('die_l')
a.bra_s('die_l')

# --------------------------------------------------------------------------
a.label('start')
# 1: reset state: supervisor, mask 7, VBR 0, SSP from vector 0
test(1)
a.move_w(SR, D0)
a.andi_w(0xf700, D0)                # T1 T0 S - - I2 I1 I0
chk_w(0x2700, D0)
a.movec(VBR, D0)
chk_l(0, D0)
a.move_l(SP, D0)
chk_l(SSP, D0)
a.clr_l(absw(RESUME))
a.clr_w(absw(IRQ_CLR))

# 2: EXTB.L (CPU32RM EXT/EXTB)
test(2)
a.move_l(imm(0x12345680), D0)
a.extb_l(D0)
chk_l(0xFFFFFF80, D0)
a.move_l(imm(0x8000017F), D0)
a.extb_l(D0)
chk_l(0x7F, D0)

# 3: MULU.L / MULS.L, 32 and 64 bit products
test(3)
a.move_l(imm(0x10000), D0)
a.move_l(D0, D1)
a.mulu_l(D1, D0)                    # overflow: low 32 bits, V set
a.bvc('fail')
chk_l(0, D0)
a.moveq(-1, D0)
a.moveq(-1, D1)
a.mulu_l(D1, D0, D2)                # D2:D0 = FFFFFFFE:00000001
chk_l(1, D0)
chk_l(0xFFFFFFFE, D2)
a.moveq(-3, D0)
a.muls_l(imm(7), D0)
chk_l((-21) & 0xffffffff, D0)
a.moveq(-2, D0)
a.muls_l(imm(0x40000000), D0, D2)   # -0x80000000
chk_l(0x80000000, D0)
chk_l(0xFFFFFFFF, D2)

# 4: DIVU.L / DIVS.L / DIVUL.L / DIVSL.L / 64-bit dividend, DIVU.W
test(4)
a.moveq(100, D0)
a.divu_l(imm(7), D0)
chk_l(14, D0)
a.moveq(100, D0)
a.divul_l(imm(7), D1, D0)           # DIVUL.L #7,D1:D0
chk_l(14, D0)
chk_l(2, D1)
a.moveq(100, D0)
a.divsl_l(imm(-7), D1, D0)
chk_l((-14) & 0xffffffff, D0)
chk_l(2, D1)
a.moveq(-100, D0)
a.divsl_l(imm(7), D1, D0)           # remainder has the dividend's sign
chk_l((-14) & 0xffffffff, D0)
chk_l((-2) & 0xffffffff, D1)
a.moveq(1, D1)
a.moveq(0, D0)
a.divu_l(imm(0x10), D0, D1)         # D1:D0 = 1:00000000 / 16
chk_l(0x10000000, D0)
chk_l(0, D1)
a.moveq(100, D0)
a.divu_w(imm(7), D0)
chk_l(0x0002000E, D0)

# 5: scaled index, full-format base displacement (no memory indirect)
test(5)
a.lea(absl('ltab'), A0)
a.moveq(2, D1)
a.move_l(idx(0, A0, D1, 'w', 4), D0)
chk_l(0x33333333, D0)
a.moveq(3, D1)
a.move_l(full('ltab', None, D1, 'l', 4), D0)    # (ltab.l,ZA0,D1.l*4)
chk_l(0x44444444, D0)
a.lea(absl('ltab-0x12340'), A1)
a.moveq(1, D1)
a.move_l(full(0x12340, A1, D1, 'w', 4), D0)     # (bd.l,A1,D1.w*4)
chk_l(0x22222222, D0)
a.moveq(4, D1)
a.move_w(idx(-2, A0, D1, 'l', 2), D0)
chk_w(0x2222, D0)

# 6: memory indirect / reserved BD SIZE: illegal on CPU32 (CPU32RM 6.2.8)
test(6)
trigger('r6a', lambda: (a.label('i6a'), a.raw(0x2010 | 0x20, 0x0151)))
expect_exc(0x0010, pc='i6a')        # move.l ([A0]),D0
test(6)
trigger('r6b', lambda: (a.label('i6b'), a.raw(0x2030, 0x0140)))
expect_exc(0x0010, pc='i6b')        # BD SIZE = 00
test(6)
trigger('r6c', lambda: (a.label('i6c'), a.raw(0x2030, 0x1158)))
expect_exc(0x0010, pc='i6c')        # bit 3 set

# 7: TRAP #5 -> vector 37, format $0, PC of next instruction (6.2.4)
test(7)
trigger('r7', lambda: a.trap(5))
expect_exc(0x0094, pc='r7')
chk_mem_w(0x2700, EXC_SR)

# 8: CHK -> vector 6, format $2, faulted instruction address (6.4.2)
test(8)
a.moveq(10, D1)
trigger('r8', lambda: (a.label('i8'), a.chk_w(imm(5), D1)))
expect_exc(0x2018, pc='r8', x1='i8')
test(8)
a.moveq(5, D1)
a.chk_w(imm(5), D1)                 # in bounds: no exception
a.moveq(-1, D1)
trigger('r8b', lambda: (a.label('i8b'), a.chk_l(imm(100), D1)))
expect_exc(0x2018, pc='r8b', x1='i8b')

# 9: CHK2/CMP2 (CPU32RM CHK2, CMP2)
test(9)
a.moveq(3, D1)
a.cmp2_w(absl('bounds'), D1)        # 0 <= 3 <= 5
a.beq('fail')
a.bcs('fail')
a.moveq(5, D1)
a.cmp2_w(absl('bounds'), D1)        # equal to bound: Z
a.bne('fail')
a.bcs('fail')
a.moveq(6, D1)
a.cmp2_w(absl('bounds'), D1)        # out of bounds: C
a.bcc('fail')
a.moveq(4, D1)
a.chk2_w(absl('bounds'), D1)
chk_mem_l(0, EXC_CNT)
a.moveq(-1, D1)
trigger('r9', lambda: (a.label('i9'), a.chk2_w(absl('bounds'), D1)))
expect_exc(0x2018, pc='r9', x1='i9')
test(9)
a.move_l(imm(0x80), A1)             # address register: compared as long
a.cmp2_b(absl('bbounds'), A1)       # bounds 0x10..0x7f (sign-extended)
a.bcc('fail')

# 10: divide by zero -> vector 5, format $2
test(10)
a.moveq(100, D0)
trigger('r10', lambda: (a.label('i10'), a.divu_w(imm(0), D0)))
expect_exc(0x2014, pc='r10', x1='i10')
test(10)
trigger('r10b', lambda: (a.label('i10b'), a.divs_l(imm(0), D0)))
expect_exc(0x2014, pc='r10b', x1='i10b')

# 11: TRAPcc / TRAPV -> vector 7, format $2
test(11)
a.moveq(0, D0)                      # Z = 1
a.trapcc('ne')                      # not taken
a.trapcc('ne', 0x1234, 'w')
chk_mem_l(0, EXC_CNT)
trigger('r11', lambda: (a.move_w(imm(0x04), CCR), a.label('i11'),
                         a.trapcc('eq', 0x12345678, 'l')))
expect_exc(0x201C, pc='r11', x1='i11')
test(11)
trigger('r11b', lambda: (a.move_w(imm(0x02), CCR),     # V
                          a.label('i11b'), a.trapv()))
expect_exc(0x201C, pc='r11b', x1='i11b')

# 12: illegal / BGND / BKPT / line A / line F -> format $0, PC of insn
test(12)
trigger('r12a', lambda: (a.label('i12a'), a.illegal()))
expect_exc(0x0010, pc='i12a')
test(12)
# BGND with background mode disabled: illegal instruction (CPU32RM 4-41)
trigger('r12b', lambda: (a.label('i12b'), a.bgnd()))
expect_exc(0x0010, pc='i12b')
test(12)
# BKPT: breakpoint acknowledge terminated by BERR -> illegal (6.2.5)
trigger('r12c', lambda: (a.label('i12c'), a.bkpt(3)))
expect_exc(0x0010, pc='i12c')
test(12)
trigger('r12d', lambda: (a.label('i12d'), a.raw(0xA123)))
expect_exc(0x0028, pc='i12d')
test(12)
trigger('r12e', lambda: (a.label('i12e'), a.raw(0xF200, 0x0000)))
expect_exc(0x002C, pc='i12e')
test(12)                            # MOVEC with an undefined register
trigger('r12f', lambda: (a.label('i12f'), a.raw(0x4E7B, 0x0002)))
expect_exc(0x0010, pc='i12f')
test(12)                            # CAS is not a CPU32 instruction
trigger('r12g', lambda: (a.label('i12g'), a.raw(0x0CD0, 0x0000)))
expect_exc(0x0010, pc='i12g')


# 13: privilege violations in user mode -> vector 8, format $0, PC of insn
def user_priv(n, label, emit):
    test(13)
    a.move_l(imm(USTACK), A0)
    a.move_l(A0, USP)
    a.andi_sr(0xDFFF)               # enter user mode
    trigger('r13' + label, lambda: (a.label('i13' + label), emit()))
    expect_exc(0x0020, pc='i13' + label)
    a.move_w(absw(EXC_SR), D0)      # stacked SR: user mode
    a.btst(13, D0)
    a.bne('fail')


user_priv(13, 'a', lambda: a.stop(0x2700))
user_priv(13, 'b', lambda: a.lpstop(0x2700))
user_priv(13, 'c', lambda: a.move_w(SR, D0))
user_priv(13, 'd', lambda: a.movec(VBR, D0))
user_priv(13, 'e', lambda: a.rte())
user_priv(13, 'f', lambda: a.move_l(USP, A0))
user_priv(13, 'g', lambda: a.ori_sr(0x0700))
# user mode can read the CCR
test(13)
a.move_l(imm(USTACK), A0)
a.move_l(A0, USP)
a.andi_sr(0xDFFF)
a.move_w(imm(0x1F), CCR)
a.move_w(CCR, D0)
chk_w(0x1F, D0)
a.move_l(SP, D0)                    # user stack pointer in use
chk_l(USTACK, D0)
trigger('r13z', lambda: a.trap(0))  # back to supervisor
expect_exc(0x0080, pc='r13z')

# 14: LPSTOP with S clear in the immediate data: privilege violation
test(14)
trigger('r14', lambda: (a.label('i14'), a.lpstop(0x0700)))
expect_exc(0x0020, pc='i14')
a.move_w(SR, D0)
a.btst(13, D0)
a.beq('fail')

# 15: RTE frame format checks (6.2.7, 6.2.12)
test(15)
a.move_l(SP, absw(SAVE_SP))
a.move_w(imm(0x8000), predec(SP))   # format $8: not a CPU32 frame
a.pea(absl('fail'))
a.move_w(imm(0x2700), predec(SP))
trigger('r15', lambda: (a.label('i15'), a.rte()))
expect_exc(0x0038, pc='i15')        # format error, PC = the RTE
a.addq_l(8, SP)                     # faulty frame left intact
a.move_l(SP, D0)
chk_l(SSP, D0)
test(15)                            # format $2: six words
a.move_l(imm(0x11111111), predec(SP))
a.move_w(imm(0x2018), predec(SP))
a.pea(absl('r15b'))
a.move_w(imm(0x2704), predec(SP))   # Z set
a.rte()
a.bra('fail')
a.label('r15b')
a.bne('fail')
a.move_l(SP, D0)
chk_l(SSP, D0)
test(15)                            # format $0: four words
a.move_w(imm(0x0080), predec(SP))
a.pea(absl('r15c'))
a.move_w(imm(0x2700), predec(SP))
a.rte()
a.bra('fail')
a.label('r15c')
a.move_l(SP, D0)
chk_l(SSP, D0)

# 16: MOVEC VBR relocation (CPU32RM 6.1.1)
test(16)
a.lea(absw(0), A0)
a.lea(absl(VBR2), A1)
a.move_w(imm(255), D1)
a.label('cpvec')
a.move_l(postinc(A0), postinc(A1))
a.dbra(D1, 'cpvec')
a.move_l(imm('vbr_h'), absl(VBR2 + 33 * 4))
a.move_l(imm(VBR2), D0)
a.movec(D0, VBR)
a.movec(VBR, D1)
chk_l(VBR2, D1)
a.clr_l(absw(VBR_HIT))
a.trap(1)                           # -> vbr_h via the new table
chk_mem_l(1, VBR_HIT)
chk_mem_l(0, EXC_CNT)
trigger('r16', lambda: a.trap(2))   # generic handler via the copy
expect_exc(0x0088, pc='r16')
a.moveq(0, D0)
a.movec(D0, VBR)
a.moveq(5, D0)
a.movec(D0, SFC)
a.movec(SFC, D1)
chk_l(5, D1)
a.moveq(5, D0)
a.movec(D0, DFC)
a.move_l(imm(0x1234), A3)
a.movec(A3, USP)
a.move_l(USP, A4)
a.cmpa_l(imm(0x1234), A4)
fail_ne()

# 17: TBLU/TBLS/TBLUN/TBLSN, table forms (CPU32RM 4.6 examples 2 and 3)
test(17)
a.move_l(imm(0xABCD018E), D0)       # entry 1, fraction $8E
a.tblu_w(absl('tw'), D0)            # 1311 + 142*(1966-1311)/256 = 1674
chk_l(0xABCD068A, D0)
a.move_l(imm(0xABCD018E), D0)
a.tbls_w(pcrel('tw'), D0)
chk_l(0xABCD068A, D0)
a.move_l(imm(0xABCD018E), D0)
a.tblun_w(absl('tw'), D0)           # 1311*256 + 93010, zero extended
chk_l(0x00068A52, D0)
a.bvs('fail')
a.bmi('fail')
a.move_l(imm(0x12340BD0), D0)       # example 3: Y = 80+208*(64-80)/256
a.lea(absl('tb3'), A0)
a.tblu_b(ind(A0), D0)
chk_l(0x12340B43, D0)
a.moveq(0, D1)
a.move_l(imm(0x12340BD0), D0)
a.tblu_b(idx(0, A0, D1, 'w', 1), D0)
chk_l(0x12340B43, D0)
a.move_l(imm(0x1), D0)              # long table, entry 0, fraction 1
a.tblu_l(absl('tl'), D0)            # 0x10000000 + 0x100 = 0x10000100
chk_l(0x10000100, D0)

# 18: TBLS rounding (round half away from zero), TBLSN sign extension, CCs
test(18)
a.move_l(imm(0x55555580), D0)       # tbs: [0, -1], fraction 1/2
a.tbls_b(absl('tbs'), D0)           # -1/2 -> -1
chk_l(0x555555FF, D0)
a.bpl('fail')                       # N from bit 7
a.move_l(imm(0x5555557F), D0)
a.tbls_b(absl('tbs'), D0)           # -127/256 -> 0
a.bne('fail')                       # Z
chk_l(0x55555500, D0)
a.move_l(imm(0x55555180), D0)       # entry 1: [-1, 0]: +1/2 -> +1 = 0
a.tbls_b(absl('tbs'), D0)
chk_l(0x55555500, D0)
a.move_l(imm(0x55555340), D0)       # entry 3: [0x10, 0x20], 1/4
a.tblsn_b(absl('tbs'), D0)          # 0x1000 + 16*64
chk_l(0x00001400, D0)
a.move_l(imm(0x55555540), D0)       # entry 5: [-16, -32], 1/4
a.tblsn_b(absl('tbs'), D0)          # -4096 - 1024, sign extended
chk_l(0xFFFFEC00, D0)
a.bpl('fail')
a.move_w(imm(0x1F), CCR)
a.move_l(imm(0x55555340), D0)
a.tbls_b(absl('tbs'), D0)           # X unaffected, V and C cleared
a.bcs('fail')
a.bvs('fail')
a.move_w(CCR, D1)
a.btst(4, D1)
a.beq('fail')

# 19: register interpolate forms, long overflow (V)
test(19)
a.move_l(imm(1000), D1)
a.move_l(imm(2000), D2)
a.move_l(imm(0x5555AA40), D0)       # Dx[15:8] ignored
a.tblu_w((D1, D2), D0)              # 1000 + 1000*64/256
chk_l(0x555504E2, D0)
a.move_l(imm(0x7FFFFFFF), D1)
a.move_l(D1, D2)
a.move_l(imm(0x10), D0)
a.tblsn_l((D1, D2), D0)             # integer part 0x7FFFFFFF > 2^23-1
a.bvc('fail')
chk_l(0xFFFFFF00, D0)
a.move_l(imm(0x00FFFFFF), D1)
a.move_l(D1, D2)
a.move_l(imm(0x10), D0)
a.tblun_l((D1, D2), D0)             # 0x00FFFFFF fits 24 bits
a.bvs('fail')
a.bpl('fail')
chk_l(0xFFFFFF00, D0)
a.move_l(imm(0xFFFFFFF0), D1)       # TBLS.L register form, -16 .. 16
a.moveq(16, D2)
a.move_l(imm(0x80), D0)
a.tbls_l((D1, D2), D0)              # -16 + 32/2 = 0
a.bne('fail')
test(19)                            # invalid size 11 -> illegal
trigger('r19', lambda: (a.label('i19'), a.raw(0xF810, 0x01C0)))
expect_exc(0x0010, pc='i19')

# 20: address errors: odd word/long operands and odd instruction fetch,
# format $C twelve-word frame (6.2.3, 6.4.3), RTE of the $C frame
test(20)
a.lea(absl(SCRATCH + 1), A0)
a.move_b(ind(A0), D0)               # byte access at odd address is fine
chk_mem_l(0, EXC_CNT)
trigger('r20a', lambda: (a.label('i20a'), a.move_w(ind(A0), D0)))
expect_exc(0xC00C, pc='i20a', x1=SCRATCH + 1)
chk_mem_l('i20a', EXC_X2)           # current instruction PC
chk_mem_w(0x0055, EXC_SSW)          # RW, SIZ=word, FC=5 (supervisor data)
test(20)
trigger('r20b', lambda: (a.label('i20b'), a.move_l(D0, disp(2, A0))))
expect_exc(0xC00C, pc='i20b', x1=SCRATCH + 3)
a.move_w(absw(EXC_SSW), D0)
a.andi_w(0xffc7, D0)                # write, FC=5 (SIZ not checked)
chk_w(0x0005, D0)
test(20)
trigger('r20c', lambda: a.jmp(ind(A0)))
expect_exc(0xC00C, pc=SCRATCH + 1, x1=SCRATCH + 1)
chk_mem_w(0x00D6, EXC_SSW)          # IN, RW, SIZ=word, FC=6 (program)
test(20)
a.lea(absl(SCRATCH + 1), A0)        # (An)+ not updated on a fault
trigger('r20d', lambda: a.move_w(postinc(A0), D0))
expect_exc(0xC00C)
a.cmpa_l(imm(SCRATCH + 1), A0)
fail_ne()

# 21: interrupts: masking, mask set to the request level, LPSTOP/STOP
# wake-up, edge-triggered level 7 (6.2.11)
test(21)
a.move_l(imm('irq'), absw(64 * 4))
a.move_l(imm('irq'), absw(65 * 4))
a.move_l(imm('irq'), absw(66 * 4))
a.move_l(imm('irq'), absw(27 * 4))
a.clr_l(absw(IRQ_CNT))
a.move_w(imm(1), absw(IRQ_CLR))
a.move_w(imm(0x0340), absw(IRQ))    # level 3, vector 64, masked
a.nop()
a.nop()
chk_mem_l(0, IRQ_CNT)
a.lpstop(0x2000)                    # wakes on the level 3 request
a.move_w(SR, D6)
chk_mem_l(1, IRQ_CNT)
chk_mem_w(0x2300, IRQ_SR)           # mask = level of the interrupt
chk_w(0x2000, D6)                   # SR loaded by LPSTOP
test(21)
a.move_w(imm(0x2100), SR)           # mask 1, request level 2
a.clr_l(absw(IRQ_CNT))
a.move_w(imm(0x0241), absw(IRQ))
sync_irq('s21b')
chk_mem_l(1, IRQ_CNT)
chk_mem_w(0x2200, IRQ_SR)           # replaced, not OR-ed (would be 3)
test(21)
a.move_w(imm(0x2700), SR)
a.clr_l(absw(IRQ_CNT))
a.move_w(imm(0x031B), absw(IRQ))    # level 3 autovector (27)
a.stop(0x2200)
chk_mem_l(1, IRQ_CNT)
chk_mem_w(0x2300, IRQ_SR)
test(21)                            # NMI: level 7 ignores mask 7, once
a.move_w(imm(0x2700), SR)
a.clr_l(absw(IRQ_CNT))
a.clr_w(absw(IRQ_CLR))              # keep the request asserted
a.move_w(imm(0x0742), absw(IRQ))
sync_irq('s21d')
sync_irq('s21e')
chk_mem_l(1, IRQ_CNT)
chk_mem_w(0x2700, IRQ_SR)
a.move_w(imm(1), absw(IRQ_CLR))
a.move_w(imm(0x2600), SR)           # mask 7 -> 6 with level 7 held: NMI
a.nop()
chk_mem_l(2, IRQ_CNT)
a.move_w(imm(0x2700), SR)

# 22: LINK.L / UNLK / RTD / Bcc.L / DBcc / Scc / MOVEM / MOVEP / MOVES
test(22)
a.move_l(SP, D5)
a.link_l(A6, -0x10000)
a.move_l(SP, D0)
a.sub_l(D5, D0)
chk_l((-0x10004) & 0xffffffff, D0)
a.unlk(A6)
a.cmp_l(SP, D5)
fail_ne()
a.move_l(imm(0x11), predec(SP))
a.move_l(imm(0x22), predec(SP))
a.bsr('rtd_sub')
a.cmp_l(SP, D5)
fail_ne()
a.bra_l('bl_ok')
a.bra('fail')
a.label('rtd_sub')
a.rtd(8)
a.label('bl_ok')
a.moveq(0, D0)
a.moveq(9, D1)
a.label('dbl')
a.add_l(D1, D0)
a.dbra(D1, 'dbl')
chk_l(45, D0)
a.moveq(1, D0)
a.seq(D1)
a.cmpi_b(0, D1)
fail_ne()
a.sne(D1)
a.cmpi_b(0xFF, D1)
fail_ne()
a.movem_l([D0, D1, A0], predec(SP))
a.moveq(0, D0)
a.moveq(0, D1)
a.movem_l(postinc(SP), [D2, D3, A1])
chk_l(1, D2)
a.cmp_l(SP, D5)
fail_ne()
a.lea(absl(SCRATCH), A0)
a.move_l(imm(0x11223344), D0)
a.movep_l(D0, disp(0, A0))
a.move_b(disp(2, A0), D1)
a.cmpi_b(0x22, D1)
fail_ne()
a.movep_w(disp(4, A0), D1)
chk_w(0x3344, D1)
a.moves_l(absl(SCRATCH), D2)        # SFC = 5 (supervisor data)
a.move_l(absl(SCRATCH), D3)
a.cmp_l(D3, D2)
fail_ne()

# 23: bus error: access to unmapped memory -> vector 2, format $C
test(23)
a.lea(absl(0x00F00000), A0)
trigger('r23', lambda: (a.label('i23'), a.move_w(ind(A0), D0)))
expect_exc(0xC008, pc='i23', x1=0x00F00000)
chk_mem_w(0x0055, EXC_SSW)

# done
a.lea(pcrel('s_ok'), A0)
a.bsr('puts')
a.clr_l(absw(EXIT))
a.label('halt')
a.bra_s('halt')

# -- data --------------------------------------------------------------------
a.align(4)
a.label('ltab')
a.dc_l(0x11111111, 0x22222222, 0x33333333, 0x44444444)
a.label('bounds')
a.dc_w(0, 5)
a.label('bbounds')
a.dc_b(0x10, 0x7f)
a.align(2)
a.label('tw')
a.dc_w(1000, 1311, 1966, 2000)
a.label('tb3')                      # CPU32RM figure 4-5 table
a.dc_b(*[0, 16, 32, 48, 64, 80, 96, 112, 128,
         112, 96, 80, 64, 48, 32, 16, 0])
a.label('tbs')
a.dc_b(0, 0xFF, 0, 0x10, 0x20, 0xF0, 0xE0, 0)
a.align(2)
a.label('tl')
a.dc_l(0x10000000, 0x10010000)
a.label('hexdig')
a.ascii('0123456789ABCDEF')
a.label('s_ok')
a.asciz('CPU32 OK\n')
a.label('s_fail')
a.asciz('FAIL ')
a.label('s_unexp')
a.asciz('UNEXP ')

if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'cpu32_test.bin'
    with open(out, 'wb') as f:
        f.write(a.bytes())
