#!/usr/bin/env python3
# Minimal two-pass SH-2 assembler used to build the ECU emulator test ROMs.
# SPDX-License-Identifier: GPL-2.0-or-later
import struct


class Asm:
    def __init__(self, size, fill=0xff):
        self.size = size
        self.fill = fill
        self.labels = {}
        self.items = []      # (addr, kind, args)
        self.pc = 0

    # -- layout --------------------------------------------------------
    def org(self, addr):
        self.pc = addr

    def align(self, n):
        while self.pc % n:
            self.items.append((self.pc, 'b', 0))
            self.pc += 1

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = self.pc

    def long(self, v):
        self.items.append((self.pc, 'L', v))
        self.pc += 4

    def word(self, v):
        self.items.append((self.pc, 'W', v))
        self.pc += 2

    def bytes_(self, data):
        for b in data:
            self.items.append((self.pc, 'b', b))
            self.pc += 1

    def op(self, fn):
        """fn(addr, labels) -> 16-bit opcode"""
        self.items.append((self.pc, 'op', fn))
        self.pc += 2

    def raw(self, v):
        self.op(lambda a, l: v)

    # -- instructions ----------------------------------------------------
    def mov_i(self, imm, n):
        assert -128 <= imm < 256
        self.raw(0xE000 | n << 8 | (imm & 0xff))

    def mov(self, m, n):
        self.raw(0x6003 | n << 8 | m << 4)

    def movl_pc(self, lbl, n):
        def f(a, l):
            t = l[lbl]
            d = (t - ((a & ~3) + 4)) // 4
            assert t % 4 == 0 and 0 <= d < 256, (lbl, hex(a), hex(t))
            return 0xD000 | n << 8 | d
        self.op(f)

    def lit(self, value, n, pool):
        """load 32-bit constant via named literal in pool"""
        name = '%s_%x' % (pool.name, value & 0xffffffff)
        pool.add(name, value)
        self.movl_pc(name, n)

    def st(self, size, m, n):           # mov.x Rm,@Rn
        self.raw(0x2000 | n << 8 | m << 4 | {'b': 0, 'w': 1, 'l': 2}[size])

    def ld(self, size, m, n):           # mov.x @Rm,Rn
        self.raw(0x6000 | n << 8 | m << 4 | {'b': 0, 'w': 1, 'l': 2}[size])

    def push(self, m):
        self.raw(0x2F06 | m << 4)

    def pop(self, n):
        self.raw(0x60F6 | n << 8)

    def add(self, m, n):
        self.raw(0x300C | n << 8 | m << 4)

    def add_i(self, imm, n):
        self.raw(0x7000 | n << 8 | (imm & 0xff))

    def sub(self, m, n):
        self.raw(0x3008 | n << 8 | m << 4)

    def cmpeq(self, m, n):
        self.raw(0x3000 | n << 8 | m << 4)

    def cmphs(self, m, n):
        self.raw(0x3002 | n << 8 | m << 4)

    def cmpeq_i(self, imm):
        self.raw(0x8800 | (imm & 0xff))

    def tst_i(self, imm):
        self.raw(0xC800 | (imm & 0xff))

    def tst(self, m, n):
        self.raw(0x2008 | n << 8 | m << 4)

    def and_i(self, imm):
        self.raw(0xC900 | (imm & 0xff))

    def or_i(self, imm):
        self.raw(0xCB00 | (imm & 0xff))

    def and_(self, m, n):
        self.raw(0x2009 | n << 8 | m << 4)

    def or_(self, m, n):
        self.raw(0x200B | n << 8 | m << 4)

    def extuw(self, m, n):
        self.raw(0x600D | n << 8 | m << 4)

    def extub(self, m, n):
        self.raw(0x600C | n << 8 | m << 4)

    def shlr(self, n, k):
        codes = {1: 0x01, 2: 0x09, 8: 0x19, 16: 0x29}
        self.raw(0x4000 | n << 8 | codes[k])

    def shll(self, n, k):
        codes = {1: 0x00, 2: 0x08, 8: 0x18, 16: 0x28}
        self.raw(0x4000 | n << 8 | codes[k])

    def dt(self, n):
        self.raw(0x4010 | n << 8)

    def _br(self, base, lbl, bits):
        def f(a, l):
            d = (l[lbl] - (a + 4)) // 2
            lim = 1 << (bits - 1)
            assert -lim <= d < lim, (lbl, hex(a))
            return base | (d & ((1 << bits) - 1))
        self.op(f)

    def bt(self, lbl):
        self._br(0x8900, lbl, 8)

    def bf(self, lbl):
        self._br(0x8B00, lbl, 8)

    def bra(self, lbl):
        self._br(0xA000, lbl, 12)

    def bsr(self, lbl):
        self._br(0xB000, lbl, 12)

    def jsr(self, m):
        self.raw(0x400B | m << 8)

    def rts(self):
        self.raw(0x000B)

    def rte(self):
        self.raw(0x002B)

    def nop(self):
        self.raw(0x0009)

    def sleep(self):
        self.raw(0x001B)

    def ldc_sr(self, m):
        self.raw(0x400E | m << 8)

    def ldc_vbr(self, m):
        self.raw(0x402E | m << 8)

    def stc_sr(self, n):
        self.raw(0x0002 | n << 8)

    def sts_pr_push(self):
        self.raw(0x4F22)        # sts.l pr,@-r15

    def lds_pr_pop(self):
        self.raw(0x4F26)        # lds.l @r15+,pr

    def trapa(self, imm):
        self.raw(0xC300 | imm)

    # -- output ----------------------------------------------------------
    def build(self):
        img = bytearray([self.fill]) * self.size
        for addr, kind, v in self.items:
            if kind == 'op':
                v = v(addr, self.labels)
                struct.pack_into('>H', img, addr, v)
            elif kind == 'W':
                struct.pack_into('>H', img, addr, v & 0xffff)
            elif kind == 'L':
                if isinstance(v, str):
                    v = self.labels[v]
                struct.pack_into('>I', img, addr, v & 0xffffffff)
            else:
                img[addr] = v
        return bytes(img)


class Pool:
    """literal pool; call emit() at a reachable, unreached location"""
    def __init__(self, asm, name):
        self.asm = asm
        self.name = name
        self.entries = {}

    def add(self, name, value):
        self.entries[name] = value

    def emit(self):
        a = self.asm
        if a.pc % 4:
            a.nop()
        for name, value in self.entries.items():
            a.label(name)
            a.long(value)
