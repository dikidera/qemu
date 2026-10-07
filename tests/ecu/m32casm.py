#!/usr/bin/env python3
# Minimal M32C/80 assembler for the ECU emulator test ROMs.
# Operands use the 5-bit addressing codes (hi, lo) of the M32C/80 encoding.
# SPDX-License-Identifier: GPL-2.0-or-later
import struct

# 5-bit codes
R0, R1, R2, R3 = (4, 2), (4, 3), (4, 0), (4, 1)          # word registers
R0L, R1L, R0H, R1H = (4, 2), (4, 3), (4, 0), (4, 1)      # byte registers
R2R0, R3R1 = (4, 2), (4, 3)
A0, A1 = (0, 2), (0, 3)
IA0, IA1 = (0, 0), (0, 1)                                # [A0], [A1]


def ABS16(a):
    return (3, 3, a, 2)


def ABS24(a):
    return (3, 2, a, 3)


def SB8(d):
    return (1, 2, d, 1)


def FB8(d):
    return (1, 3, d & 0xff, 1)


def addon(op):
    if len(op) == 2:
        return b''
    return (op[2] & ((1 << (8 * op[3])) - 1)).to_bytes(op[3], 'little')


# condition codes
LTU, LEU, NE, PZ, NO, GT, GE, GEU, GTU, EQ, N, O, LE, LT = \
    0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14


class Asm:
    def __init__(self, base, size, fill=0xff):
        self.base = base
        self.img = bytearray([fill]) * size
        self.pc = base
        self.labels = {}
        self.fixups = []

    def org(self, a):
        self.pc = a

    def label(self, n):
        self.labels[n] = self.pc

    def emit(self, b):
        off = self.pc - self.base
        self.img[off:off + len(b)] = b
        self.pc += len(b)

    def addr24(self, v):
        if isinstance(v, str):
            self.fixups.append((self.pc, 'a24', v, 0))
            v = 0
        self.emit(struct.pack('<I', v)[:3] + b'\0')

    # -- generic encoders --------------------------------------------------
    def g1(self, b1base, b2base, dst, size_bit, extra=b''):
        """single dst5 operand: op1 = b1base|dhi<<1|z, op2 = dlo<<6|b2base"""
        self.emit(bytes([b1base | dst[0] << 1 | size_bit,
                         dst[1] << 6 | b2base]) + addon(dst) + extra)

    def g2(self, b2op, src, dst, size_bit, prefix=b''):
        self.emit(prefix + bytes([0x80 | src[0] << 4 | dst[0] << 1 | size_bit,
                                  dst[1] << 6 | src[1] << 4 | b2op]) +
                  addon(src) + addon(dst))

    # -- instructions ------------------------------------------------------
    def mov_b_imm(self, imm, dst):
        self.g1(0x90, 0x2f, dst, 0, bytes([imm & 0xff]))

    def mov_w_imm(self, imm, dst):
        self.g1(0x90, 0x2f, dst, 1, struct.pack('<H', imm & 0xffff))

    def mov_l_imm(self, imm, dst):
        self.g1(0xb0, 0x31, dst, 0, struct.pack('<I', imm & 0xffffffff))

    def mov_b(self, src, dst):
        self.g2(0xb, src, dst, 0)

    def mov_w(self, src, dst):
        self.g2(0xb, src, dst, 1)

    def cmp_w_imm(self, imm, dst):
        self.g1(0x90, 0x2e, dst, 1, struct.pack('<H', imm & 0xffff))

    def cmp_b_imm(self, imm, dst):
        self.g1(0x90, 0x2e, dst, 0, bytes([imm & 0xff]))

    def add_w_imm(self, imm, dst):
        self.g1(0x80, 0x2e, dst, 1, struct.pack('<H', imm & 0xffff))

    def add_w(self, src, dst):
        self.g2(0x8, src, dst, 1)

    def sub_w(self, src, dst):
        self.g2(0xa, src, dst, 1)

    def and_b_imm(self, imm, dst):
        self.g1(0x80, 0x3f, dst, 0, bytes([imm & 0xff]))

    def or_b_imm(self, imm, dst):
        self.g1(0x80, 0x2f, dst, 0, bytes([imm & 0xff]))

    def and_w_imm(self, imm, dst):
        self.g1(0x80, 0x3f, dst, 1, struct.pack('<H', imm & 0xffff))

    def inc_w(self, dst):
        self.g1(0xa0, 0x0e, dst, 1)

    def shl_w(self, n, dst):
        """SHL.W #n, dst  (n = -8..-1, 1..8)"""
        code = (n - 1) if n > 0 else (8 | (-n - 1))
        self.g1(0xe0, 0x00 | code, dst, 1)

    def mulu_w_imm(self, imm, dst):
        self.g1(0x80, 0x0f, dst, 1, struct.pack('<H', imm & 0xffff))

    def divu_w_imm(self, imm):
        self.emit(bytes([0xb0, 0x13]) + struct.pack('<H', imm & 0xffff))

    def btst(self, bit, dst):
        self.g1(0xd0, 0x00 | bit, dst, 0)

    def bset(self, bit, dst):
        self.g1(0xd0, 0x38 | bit, dst, 0)

    def bclr(self, bit, dst):
        self.g1(0xd0, 0x30 | bit, dst, 0)

    def ldc24(self, imm, creg):
        regs = {'INTB': 0, 'SP': 1, 'SB': 2, 'FB': 3, 'ISP': 7}
        self.emit(bytes([0xd5, 0x28 | regs[creg]]))
        self.addr24(imm) if isinstance(imm, str) else \
            self.emit(struct.pack('<I', imm)[:3])

    def ldc_flg(self, imm):
        self.emit(bytes([0xd5, 0xaa]) + struct.pack('<H', imm))

    def fset(self, f):
        self.emit(bytes([0xd1, 0xe8 | 'CDZSBOIU'.index(f)]))

    def fclr(self, f):
        self.emit(bytes([0xd3, 0xe8 | 'CDZSBOIU'.index(f)]))

    def pushm(self, mask):
        self.emit(bytes([0x8f, mask]))

    def popm(self, mask):
        self.emit(bytes([0x8e, mask]))

    def nop(self):
        self.emit(b'\xde')

    def rts(self):
        self.emit(b'\xdf')

    def reit(self):
        self.emit(b'\x9e')

    def wait(self):
        self.emit(b'\xb2\x03')

    def jmp(self, lbl):
        self.fixups.append((self.pc, 'rel16', lbl, 1))
        self.emit(b'\xce\0\0')

    def jsr(self, lbl):
        self.fixups.append((self.pc, 'rel16', lbl, 1))
        self.emit(b'\xcf\0\0')

    def j(self, cnd, lbl):
        b1 = 0x8a | ((cnd >> 1) << 4) | (cnd & 1)
        self.fixups.append((self.pc, 'rel8', lbl, 1))
        self.emit(bytes([b1, 0]))

    def build(self):
        for at, kind, lbl, extra in self.fixups:
            t = self.labels[lbl]
            off = at - self.base
            if kind == 'rel16':
                d = t - (at + 1)
                assert -32768 <= d < 32768
                struct.pack_into('<h', self.img, off + 1, d)
            elif kind == 'rel8':
                d = t - (at + 1)
                assert -128 <= d < 128, (lbl, d)
                struct.pack_into('<b', self.img, off + 1, d)
            elif kind == 'a24':
                self.img[off:off + 3] = struct.pack('<I', t)[:3]
        return bytes(self.img)


class Asm2(Asm):
    """more instructions for the CPU self test"""

    def cmp_w(self, src, dst):
        self.g2(0x6, src, dst, 1)

    def adc_w_imm(self, imm, dst):
        self.emit(b'\x01')
        self.g1(0x80, 0x2e, dst, 1, struct.pack('<H', imm & 0xffff))

    def sbb_w_imm(self, imm, dst):
        self.emit(b'\x01')
        self.g1(0x90, 0x2e, dst, 1, struct.pack('<H', imm & 0xffff))

    def sub_w_imm(self, imm, dst):
        self.g1(0x80, 0x3e, dst, 1, struct.pack('<H', imm & 0xffff))

    def mul_w_imm(self, imm, dst):
        self.g1(0x80, 0x1f, dst, 1, struct.pack('<H', imm & 0xffff))

    def div_w_imm(self, imm):
        self.emit(bytes([0xb0, 0x53]) + struct.pack('<H', imm & 0xffff))

    def divx_w_imm(self, imm):
        self.emit(bytes([0xb2, 0x53]) + struct.pack('<H', imm & 0xffff))

    def sha_w(self, n, dst):
        code = (n - 1) if n > 0 else (8 | (-n - 1))
        self.g1(0xf0, code, dst, 1)

    def rot_w(self, n, dst):
        code = (n - 1) if n > 0 else (8 | (-n - 1))
        self.g1(0xe0, 0x20 | code, dst, 1)

    def rolc_w(self, dst):
        self.g1(0xb0, 0x2e, dst, 1)

    def rorc_w(self, dst):
        self.g1(0xa0, 0x2e, dst, 1)

    def neg_w(self, dst):
        self.g1(0xa0, 0x2f, dst, 1)

    def not_w(self, dst):
        self.g1(0xa0, 0x1e, dst, 1)

    def exts_b(self, dst):
        self.g1(0xc0, 0x1e, dst, 0)

    def extz_b(self, src, dst):
        self.g2(0xb, src, dst, 0, prefix=b'\x01')

    def push_w(self, dst):
        self.g1(0xc0, 0x0e, dst, 1)

    def pop_w(self, dst):
        self.g1(0xb0, 0x2f, dst, 1)

    def enter(self, n):
        self.emit(bytes([0xec, n]))

    def exitd(self):
        self.emit(b'\xfc')

    def indexw_w(self, dst):
        self.g1(0x80, 0x33, dst, 0)

    def smovf_w(self):
        self.emit(b'\xb0\x93')

    def sstr_b(self):
        self.emit(b'\xb8\x03')

    def int_(self, n):
        self.emit(bytes([0xbe, n << 2]))

    def adjnz_w(self, imm, dst, lbl):
        at = self.pc
        self.g1(0xf0, 0x10 | (imm & 0xf), dst, 1)
        self.fixups.append((self.pc, 'adjnz', lbl, at))
        self.emit(b'\0')

    def mov_w_ind_src(self, src, dst):
        """MOV.W [src], dst (indirect source prefix)"""
        self.g2(0xb, src, dst, 1, prefix=b'\x41')

    def build(self):
        for i, (at, kind, lbl, extra) in enumerate(self.fixups):
            if kind == 'adjnz':
                d = self.labels[lbl] - (extra + 2)
                assert -128 <= d < 128
                struct.pack_into('<b', self.img, at - self.base, d)
        self.fixups = [f for f in self.fixups if f[1] != 'adjnz']
        return super().build()
