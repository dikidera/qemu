#!/usr/bin/env python3
# Minimal 68000 / CPU32 assembler for the ECU emulator test ROMs.
# SPDX-License-Identifier: GPL-2.0-or-later
"""
m68kasm -- a small Python assembler for the 68000/CPU32 subset needed by
the ECU test ROMs (MC68332/MC68336/MC68376 class parts).

Usage
-----
    from m68kasm import *            # Asm, registers, addressing helpers

    a = Asm(base=0, size=0x10000)    # image covers [base, base+size)
    a.org(0)
    a.dc_l(0x00100000, 'start')      # reset vectors: SSP, PC (labels ok)
    a.org(0x400)
    a.label('start')
    a.move_l(imm(0x12345678), D0)
    a.lea(absl('table'), A0)
    a.move_w(idx(2, A0, D1, 'w', 2), D2)
    a.bra('start')
    a.label('table')
    a.dc_w(1, 2, 3)
    image = a.bytes()                # resolves labels, returns bytes

Operands
--------
  D0..D7, A0..A7 (SP == A7)          data / address register direct
  ind(An)                            (An)
  postinc(An), predec(An)            (An)+, -(An)
  disp(d16, An)                      (d16,An)
  idx(d8, An, Xn, 'w'|'l', scale)    (d8,An,Xn.size*scale)  brief format
  full(bd, An|None|'pc', Xn|None, 'w'|'l', scale)
                                     (bd,An,Xn.size*scale)  full format
                                     (CPU32: no memory indirection)
  absw(a), absl(a)                   (xxx).W / (xxx).L
  pcrel(target)                      (d16,PC)
  pcidx(target, Xn, 'w'|'l', scale)  (d8,PC,Xn.size*scale)
  imm(v)                             #v

Wherever an address or immediate value is expected, a label name (str)
may be used, optionally with an offset: 'table+4', 'end-2'.  Labels are
resolved when bytes() is called (forward references are fine).

Methods
-------
Methods are named after the instruction, with the size as a suffix where
the instruction is sized: move_l, add_w, cmpi_b, mulu_l, tbls_w, ...
Branches: bra/bsr/b<cc>(target) use a word displacement; append _s or _l
for byte/long displacement (bra_s, beq_l).  Generic forms: bcond(cond, t,
size='w'), dbcond(cond, Dn, t), scond(cond, ea), trapcc(cond, imm=None,
size=None); per-condition names exist too (bcc, bhs, dbne, dbra, seq).
Conditions: 't f hi ls cc hs cs lo ne eq vc vs pl mi ge lt gt le'.  Shifts: asl_w(count, Dn) where count is 1..8 or a data
register; memory shifts asl_m(ea).  Bit operations take a bit number
(int) or data register: btst(3, D0), bset(D1, ind(A0)).
MOVEM: movem_l([D0, D1, A2], predec(SP)) / movem_l(postinc(SP), [...]).
MOVEC: movec(D0, VBR) (to control register) / movec(VBR, D0) (from).
Control registers: SFC, DFC, USP, VBR.  move_w(x, SR) / move_w(SR, x),
likewise CCR, and move_l(An, USP) / move_l(USP, An) select the special
MOVE forms.
TBL: tblu_w(ea, Dx) memory form, tblu_w((Dym, Dyn), Dx) register form;
likewise tbls_*, tblun_*, tblsn_*.

Data and layout: org(addr), label(name), align(n), dc_b(*v), dc_w(*v),
dc_l(*v), ascii(s), asciz(s), here (current address), sym(name) (label
value, valid after bytes()), raw(*words) (emit raw opcode words).
"""
import struct

__all__ = ['Asm', 'Reg', 'EA',
           'D0', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7',
           'A0', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'SP',
           'SR', 'CCR', 'USP', 'VBR', 'SFC', 'DFC',
           'ind', 'postinc', 'predec', 'disp', 'idx', 'full',
           'absw', 'absl', 'pcrel', 'pcidx', 'imm', 'COND']


class Reg:
    def __init__(self, kind, n):
        self.kind = kind        # 'd' or 'a'
        self.n = n

    def __repr__(self):
        return '%s%d' % (self.kind.upper(), self.n)

    @property
    def is_d(self):
        return self.kind == 'd'

    @property
    def is_a(self):
        return self.kind == 'a'

    @property
    def num16(self):
        """register number 0..15 (D0..D7, A0..A7)"""
        return self.n + (8 if self.kind == 'a' else 0)


D0, D1, D2, D3, D4, D5, D6, D7 = [Reg('d', i) for i in range(8)]
A0, A1, A2, A3, A4, A5, A6, A7 = [Reg('a', i) for i in range(8)]
SP = A7


class Special:
    def __init__(self, name, code=None):
        self.name = name
        self.code = code

    def __repr__(self):
        return self.name


SR = Special('SR')
CCR = Special('CCR')
USP = Special('USP', 0x800)
VBR = Special('VBR', 0x801)
SFC = Special('SFC', 0x000)
DFC = Special('DFC', 0x001)


class EA:
    """A memory / immediate effective address."""
    def __init__(self, kind, **kw):
        self.kind = kind
        self.__dict__.update(kw)

    def __repr__(self):
        return 'EA(%s, %r)' % (self.kind, self.__dict__)


def ind(a):
    return EA('ind', an=a)


def postinc(a):
    return EA('postinc', an=a)


def predec(a):
    return EA('predec', an=a)


def disp(d, a):
    return EA('disp', d=d, an=a)


def idx(d, a, x, size='l', scale=1):
    return EA('idx', d=d, an=a, x=x, xsize=size, scale=scale)


def full(bd, base, x=None, size='l', scale=1):
    return EA('full', bd=bd, base=base, x=x, xsize=size, scale=scale)


def absw(a):
    return EA('absw', a=a)


def absl(a):
    return EA('absl', a=a)


def pcrel(t):
    return EA('pcrel', t=t)


def pcidx(t, x, size='l', scale=1):
    return EA('pcidx', t=t, x=x, xsize=size, scale=scale)


def imm(v):
    return EA('imm', v=v)


COND = {'t': 0, 'f': 1, 'hi': 2, 'ls': 3, 'cc': 4, 'hs': 4, 'cs': 5,
        'lo': 5, 'ne': 6, 'eq': 7, 'vc': 8, 'vs': 9, 'pl': 10, 'mi': 11,
        'ge': 12, 'lt': 13, 'gt': 14, 'le': 15}

SIZE2 = {'b': 0, 'w': 1, 'l': 2}          # standard 2-bit size field
NBYTES = {'b': 1, 'w': 2, 'l': 4}
SCALE = {1: 0, 2: 1, 4: 2, 8: 3}


def _cond(c):
    if isinstance(c, int):
        return c
    return COND[c.lower()]


class Asm:
    def __init__(self, base=0, size=0x10000, fill=0xff):
        self.base = base
        self.img = bytearray([fill]) * size
        self.pc = base
        self.labels = {}
        self.fixups = []        # (pos, kind, expr, ref)

    # -- layout ----------------------------------------------------------
    @property
    def here(self):
        return self.pc

    def org(self, addr):
        self.pc = addr

    def align(self, n, fill=0):
        while self.pc % n:
            self._emit(bytes([fill]))

    def label(self, name):
        assert name not in self.labels, 'duplicate label ' + name
        self.labels[name] = self.pc

    def sym(self, name):
        return self._value(name)

    def _emit(self, b):
        off = self.pc - self.base
        assert 0 <= off and off + len(b) <= len(self.img), hex(self.pc)
        self.img[off:off + len(b)] = b
        self.pc += len(b)

    def _w(self, v):
        self._emit(struct.pack('>H', v & 0xffff))

    def _l(self, v):
        self._emit(struct.pack('>I', v & 0xffffffff))

    def _value(self, expr):
        if isinstance(expr, int):
            return expr
        e = expr.replace(' ', '')
        for i in range(len(e) - 1, 0, -1):
            if e[i] in '+-':
                return self._value(e[:i]) + (int(e[i + 1:], 0) *
                                             (1 if e[i] == '+' else -1))
        if e not in self.labels:
            raise KeyError('undefined label %r' % expr)
        return self.labels[e]

    def _fix(self, kind, expr, ref=0, size=None):
        """emit a placeholder and record a fixup"""
        n = {'b8': 2, 'w16': 2, 'l32': 4, 'pc8': 0, 'pc16': 2,
             'pc32': 4, 'pcx8': 0}[kind] if size is None else size
        if isinstance(expr, int) and kind in ('b8', 'w16', 'l32'):
            v = expr
            if kind == 'b8':
                self._w(v & 0xff)
            elif kind == 'w16':
                self._w(v)
            else:
                self._l(v)
            return
        self.fixups.append((self.pc, kind, expr, ref))
        self._emit(bytes(n))

    def bytes(self):
        for pos, kind, expr, ref in self.fixups:
            off = pos - self.base
            v = self._value(expr)
            if kind == 'b8':
                struct.pack_into('>H', self.img, off, v & 0xff)
            elif kind == 'w16':
                struct.pack_into('>H', self.img, off, v & 0xffff)
            elif kind == 'l32':
                struct.pack_into('>I', self.img, off, v & 0xffffffff)
            elif kind == 'pc16':
                d = v - ref
                assert -0x8000 <= d < 0x8000, (expr, hex(pos))
                struct.pack_into('>h', self.img, off, d)
            elif kind == 'pc32':
                struct.pack_into('>I', self.img, off, (v - ref) & 0xffffffff)
            elif kind == 'pc8':            # low byte of opcode word
                d = v - ref
                assert -0x80 <= d < 0x80 and d not in (0, -1), \
                    ('short branch out of range', expr, hex(pos))
                self.img[off + 1] = d & 0xff
            elif kind == 'pcx8':           # low byte of brief ext word
                d = v - ref
                assert -0x80 <= d < 0x80, (expr, hex(pos))
                self.img[off + 1] = d & 0xff
        return bytes(self.img)

    # -- data ------------------------------------------------------------
    def dc_b(self, *vals):
        for v in vals:
            if isinstance(v, (bytes, bytearray)):
                self._emit(v)
            elif isinstance(v, str):
                self._emit(v.encode())
            else:
                self._emit(bytes([v & 0xff]))

    def dc_w(self, *vals):
        for v in vals:
            self._fix('w16', v)

    def dc_l(self, *vals):
        for v in vals:
            self._fix('l32', v)

    def ascii(self, s):
        self._emit(s.encode())

    def asciz(self, s):
        self._emit(s.encode() + b'\0')

    def raw(self, *words):
        for w in words:
            self._w(w)

    # -- effective addresses ----------------------------------------------
    def _ea(self, op, size='w'):
        """return (mode, reg, extension emitter)"""
        if isinstance(op, Reg):
            return (0 if op.is_d else 1), op.n, None
        assert isinstance(op, EA), op
        k = op.kind
        if k == 'ind':
            return 2, op.an.n, None
        if k == 'postinc':
            return 3, op.an.n, None
        if k == 'predec':
            return 4, op.an.n, None
        if k == 'disp':
            return 5, op.an.n, lambda: self._fix('w16', op.d)
        if k == 'idx':
            def e():
                self._w(self._brief(op.x, op.xsize, op.scale) |
                        (op.d & 0xff))
            assert -128 <= op.d < 128
            return 6, op.an.n, e
        if k == 'full':
            return self._full(op)
        if k == 'absw':
            return 7, 0, lambda: self._fix('w16', op.a)
        if k == 'absl':
            return 7, 1, lambda: self._fix('l32', op.a)
        if k == 'pcrel':
            return 7, 2, lambda: self._fix('pc16', op.t, self.pc)
        if k == 'pcidx':
            def e():
                ref = self.pc
                self._w(self._brief(op.x, op.xsize, op.scale))
                if isinstance(op.t, int):
                    d = op.t - ref
                    assert -128 <= d < 128
                    self.img[ref - self.base + 1] = d & 0xff
                else:
                    self.fixups.append((ref, 'pcx8', op.t, ref))
            return 7, 3, e
        if k == 'imm':
            if size == 'b':
                return 7, 4, lambda: self._fix('b8', op.v)
            if size == 'w':
                return 7, 4, lambda: self._fix('w16', op.v)
            return 7, 4, lambda: self._fix('l32', op.v)
        raise ValueError(op)

    @staticmethod
    def _brief(x, xsize, scale):
        return ((0x8000 if x.is_a else 0) | x.n << 12 |
                (0x800 if xsize == 'l' else 0) | SCALE[scale] << 9)

    def _full(self, op):
        ext = 0x100
        if op.x is None:
            ext |= 0x40                     # IS
        else:
            ext |= self._brief(op.x, op.xsize, op.scale)
        pcbase = isinstance(op.base, str) and op.base.lower() == 'pc'
        if op.base is None:
            ext |= 0x80                     # BS
            mode, reg = 6, 0
        elif pcbase:
            mode, reg = 7, 3
        else:
            mode, reg = 6, op.base.n
        bd = op.bd
        if isinstance(bd, int) and bd == 0:
            ext |= 0x10                     # null displacement
        elif isinstance(bd, int) and -0x8000 <= bd < 0x8000 and not pcbase:
            ext |= 0x20
        else:
            ext |= 0x30

        def e():
            ref = self.pc
            self._w(ext)
            if ext & 0x30 == 0x20:
                self._w(bd)
            elif ext & 0x30 == 0x30:
                if pcbase:
                    self._fix('pc32', bd, ref)
                else:
                    self._fix('l32', bd)
        return mode, reg, e

    def _op_ea(self, opword, ea, size='w', ext_words=()):
        """emit opcode | ea, extension words, then EA extension"""
        mode, reg, e = self._ea(ea, size)
        self._w(opword | mode << 3 | reg)
        for w in ext_words:
            if callable(w):
                w()
            else:
                self._w(w)
        if e:
            e()

    # -- data movement ---------------------------------------------------
    def _move(self, size, src, dst):
        if dst is SR:
            return self.move_to_sr(src)
        if dst is CCR:
            return self.move_to_ccr(src)
        if src is SR:
            return self.move_from_sr(dst)
        if src is CCR:
            return self.move_from_ccr(dst)
        if dst is USP:
            return self.move_to_usp(src)
        if src is USP:
            return self.move_from_usp(dst)
        sm, sr, se = self._ea(src, size)
        dm, dr, de = self._ea(dst, size)
        code = {'b': 1, 'w': 3, 'l': 2}[size]
        assert size != 'b' or dm != 1
        self._w(code << 12 | dr << 9 | dm << 6 | sm << 3 | sr)
        if se:
            se()
        if de:
            de()

    def move_b(self, src, dst):
        self._move('b', src, dst)

    def move_w(self, src, dst):
        self._move('w', src, dst)

    def move_l(self, src, dst):
        self._move('l', src, dst)

    def movea_w(self, src, an):
        assert an.is_a
        self._move('w', src, an)

    def movea_l(self, src, an):
        assert an.is_a
        self._move('l', src, an)

    def moveq(self, v, dn):
        assert dn.is_d and -128 <= v < 256
        self._w(0x7000 | dn.n << 9 | (v & 0xff))

    def lea(self, ea, an):
        assert an.is_a
        self._op_ea(0x41c0 | an.n << 9, ea)

    def pea(self, ea):
        self._op_ea(0x4840, ea)

    def exg(self, rx, ry):
        if rx.is_d and ry.is_d:
            self._w(0xc140 | rx.n << 9 | ry.n)
        elif rx.is_a and ry.is_a:
            self._w(0xc148 | rx.n << 9 | ry.n)
        else:
            d, a = (rx, ry) if rx.is_d else (ry, rx)
            self._w(0xc188 | d.n << 9 | a.n)

    def swap(self, dn):
        self._w(0x4840 | dn.n)

    def move_to_sr(self, ea):
        self._op_ea(0x46c0, ea, 'w')

    def move_from_sr(self, ea):
        self._op_ea(0x40c0, ea, 'w')

    def move_to_ccr(self, ea):
        self._op_ea(0x44c0, ea, 'w')

    def move_from_ccr(self, ea):
        self._op_ea(0x42c0, ea, 'w')

    def move_to_usp(self, an):
        self._w(0x4e60 | an.n)

    def move_from_usp(self, an):
        self._w(0x4e68 | an.n)

    def movec(self, src, dst):
        """movec(Rn, VBR) writes a control register, movec(VBR, Rn) reads"""
        if isinstance(src, Special):
            self._w(0x4e7a)
            self._w(dst.num16 << 12 | src.code)
        else:
            self._w(0x4e7b)
            self._w(src.num16 << 12 | dst.code)

    def _moves(self, size, src, dst):
        if isinstance(src, Reg):
            self._op_ea(0x0e00 | SIZE2[size] << 6, dst, size,
                        [src.num16 << 12 | 0x800])
        else:
            self._op_ea(0x0e00 | SIZE2[size] << 6, src, size,
                        [dst.num16 << 12])

    def moves_b(self, src, dst):
        self._moves('b', src, dst)

    def moves_w(self, src, dst):
        self._moves('w', src, dst)

    def moves_l(self, src, dst):
        self._moves('l', src, dst)

    def _movem(self, size, src, dst):
        s = 0x40 if size == 'l' else 0
        if isinstance(src, (list, tuple)):
            mask = 0
            for r in src:
                b = r.num16
                mask |= 1 << (15 - b if dst.kind == 'predec' else b)
            self._op_ea(0x4880 | s, dst, size, [mask])
        else:
            mask = 0
            for r in dst:
                mask |= 1 << r.num16
            self._op_ea(0x4c80 | s, src, size, [mask])

    def movem_w(self, src, dst):
        self._movem('w', src, dst)

    def movem_l(self, src, dst):
        self._movem('l', src, dst)

    def _movep(self, size, src, dst):
        if isinstance(src, Reg):    # Dx -> (d16,Ay)
            op = 0x0188 if size == 'w' else 0x01c8
            self._w(op | src.n << 9 | dst.an.n)
            self._fix('w16', dst.d)
        else:
            op = 0x0108 if size == 'w' else 0x0148
            self._w(op | dst.n << 9 | src.an.n)
            self._fix('w16', src.d)

    def movep_w(self, src, dst):
        self._movep('w', src, dst)

    def movep_l(self, src, dst):
        self._movep('l', src, dst)

    # -- arithmetic / logic ------------------------------------------------
    def _arith(self, base, size, src, dst, adda=None):
        """ADD/SUB/AND/OR/CMP family; base = 0xd000 etc."""
        if isinstance(dst, Reg) and dst.is_a:
            assert adda is not None and size != 'b'
            self._op_ea(base | dst.n << 9 | (0x1c0 if size == 'l' else 0xc0),
                        src, size)
        elif isinstance(dst, Reg) and dst.is_d:
            self._op_ea(base | dst.n << 9 | SIZE2[size] << 6, src, size)
        else:
            assert isinstance(src, Reg) and src.is_d and base != 0xb000
            self._op_ea(base | src.n << 9 | (4 + SIZE2[size]) << 6, dst,
                        size)

    def _imm_op(self, base, size, v, dst):
        mode, reg, e = self._ea(dst, size)
        self._w(base | SIZE2[size] << 6 | mode << 3 | reg)
        if size == 'b':
            self._fix('b8', v)
        elif size == 'w':
            self._fix('w16', v)
        else:
            self._fix('l32', v)
        if e:
            e()

    def _quick(self, sub, size, n, dst):
        assert 1 <= n <= 8
        self._op_ea(0x5000 | (n & 7) << 9 | sub << 8 | SIZE2[size] << 6,
                    dst, size)

    def _unary(self, base, size, ea):
        self._op_ea(base | SIZE2[size] << 6, ea, size)

    def _ccr_sr(self, base, v):
        self._w(base)
        self._w(v)

    # generated below: add_b/w/l, sub_*, and_*, or_*, cmp_* ...
    def adda_w(self, src, an):
        self._arith(0xd000, 'w', src, an, True)

    def adda_l(self, src, an):
        self._arith(0xd000, 'l', src, an, True)

    def suba_w(self, src, an):
        self._arith(0x9000, 'w', src, an, True)

    def suba_l(self, src, an):
        self._arith(0x9000, 'l', src, an, True)

    def cmpa_w(self, src, an):
        self._arith(0xb000, 'w', src, an, True)

    def cmpa_l(self, src, an):
        self._arith(0xb000, 'l', src, an, True)

    def _eor(self, size, dn, dst):
        assert dn.is_d
        self._op_ea(0xb100 | dn.n << 9 | SIZE2[size] << 6, dst, size)

    def eor_b(self, dn, dst):
        self._eor('b', dn, dst)

    def eor_w(self, dn, dst):
        self._eor('w', dn, dst)

    def eor_l(self, dn, dst):
        self._eor('l', dn, dst)

    def cmpm_b(self, ay, ax):
        self._w(0xb108 | ax.an.n << 9 | ay.an.n)

    def cmpm_w(self, ay, ax):
        self._w(0xb148 | ax.an.n << 9 | ay.an.n)

    def cmpm_l(self, ay, ax):
        self._w(0xb188 | ax.an.n << 9 | ay.an.n)

    def _x(self, base, size, src, dst):
        if isinstance(src, Reg):
            self._w(base | dst.n << 9 | SIZE2[size] << 6 | src.n)
        else:
            self._w(base | 8 | dst.an.n << 9 | SIZE2[size] << 6 | src.an.n)

    def addx_b(self, s, d):
        self._x(0xd100, 'b', s, d)

    def addx_w(self, s, d):
        self._x(0xd100, 'w', s, d)

    def addx_l(self, s, d):
        self._x(0xd100, 'l', s, d)

    def subx_b(self, s, d):
        self._x(0x9100, 'b', s, d)

    def subx_w(self, s, d):
        self._x(0x9100, 'w', s, d)

    def subx_l(self, s, d):
        self._x(0x9100, 'l', s, d)

    def andi_ccr(self, v):
        self._ccr_sr(0x023c, v & 0xff)

    def ori_ccr(self, v):
        self._ccr_sr(0x003c, v & 0xff)

    def eori_ccr(self, v):
        self._ccr_sr(0x0a3c, v & 0xff)

    def andi_sr(self, v):
        self._ccr_sr(0x027c, v)

    def ori_sr(self, v):
        self._ccr_sr(0x007c, v)

    def eori_sr(self, v):
        self._ccr_sr(0x0a7c, v)

    def ext_w(self, dn):
        self._w(0x4880 | dn.n)

    def ext_l(self, dn):
        self._w(0x48c0 | dn.n)

    def extb_l(self, dn):
        self._w(0x49c0 | dn.n)

    def tas(self, ea):
        self._op_ea(0x4ac0, ea, 'b')

    def mulu_w(self, ea, dn):
        self._op_ea(0xc0c0 | dn.n << 9, ea, 'w')

    def muls_w(self, ea, dn):
        self._op_ea(0xc1c0 | dn.n << 9, ea, 'w')

    def divu_w(self, ea, dn):
        self._op_ea(0x80c0 | dn.n << 9, ea, 'w')

    def divs_w(self, ea, dn):
        self._op_ea(0x81c0 | dn.n << 9, ea, 'w')

    def _mull(self, sign, ea, dl, dh=None):
        ext = dl.n << 12 | (0x800 if sign else 0)
        if dh is not None:
            ext |= 0x400 | dh.n
        self._op_ea(0x4c00, ea, 'l', [ext])

    def mulu_l(self, ea, dl, dh=None):
        """MULU.L <ea>,Dl  or  MULU.L <ea>,Dh:Dl (pass dh)"""
        self._mull(False, ea, dl, dh)

    def muls_l(self, ea, dl, dh=None):
        self._mull(True, ea, dl, dh)

    def _divl(self, sign, quad, ea, dq, dr):
        ext = dq.n << 12 | (0x800 if sign else 0) | (0x400 if quad else 0)
        ext |= (dr if dr is not None else dq).n
        self._op_ea(0x4c40, ea, 'l', [ext])

    def divu_l(self, ea, dq, dr=None):
        """DIVU.L <ea>,Dq (32/32) or DIVU.L <ea>,Dr:Dq (64/32, pass dr)"""
        self._divl(False, dr is not None, ea, dq, dr)

    def divs_l(self, ea, dq, dr=None):
        self._divl(True, dr is not None, ea, dq, dr)

    def divul_l(self, ea, dr, dq):
        """DIVUL.L <ea>,Dr:Dq  (32/32 -> 32r:32q)"""
        self._divl(False, False, ea, dq, dr)

    def divsl_l(self, ea, dr, dq):
        self._divl(True, False, ea, dq, dr)

    def _chk(self, size, ea, dn):
        self._op_ea(0x4000 | dn.n << 9 | (0x180 if size == 'w' else 0x100),
                    ea, size)

    def chk_w(self, ea, dn):
        self._chk('w', ea, dn)

    def chk_l(self, ea, dn):
        self._chk('l', ea, dn)

    def _chk2(self, size, ea, rn, chk):
        self._op_ea(0x00c0 | SIZE2[size] << 9, ea, size,
                    [rn.num16 << 12 | (0x800 if chk else 0)])

    def chk2_b(self, ea, rn):
        self._chk2('b', ea, rn, True)

    def chk2_w(self, ea, rn):
        self._chk2('w', ea, rn, True)

    def chk2_l(self, ea, rn):
        self._chk2('l', ea, rn, True)

    def cmp2_b(self, ea, rn):
        self._chk2('b', ea, rn, False)

    def cmp2_w(self, ea, rn):
        self._chk2('w', ea, rn, False)

    def cmp2_l(self, ea, rn):
        self._chk2('l', ea, rn, False)

    # -- shifts / rotates -------------------------------------------------
    def _shift(self, typ, left, size, cnt, dy):
        op = 0xe000 | SIZE2[size] << 6 | typ << 3 | dy.n | (0x100 if left
                                                          else 0)
        if isinstance(cnt, Reg):
            op |= 0x20 | cnt.n << 9
        else:
            assert 1 <= cnt <= 8
            op |= (cnt & 7) << 9
        self._w(op)

    def _shift_m(self, typ, left, ea):
        self._op_ea(0xe0c0 | typ << 9 | (0x100 if left else 0), ea, 'w')

    # -- bit operations -----------------------------------------------------
    def _bit(self, tt, bit, ea):
        if isinstance(bit, Reg):
            self._op_ea(0x0100 | bit.n << 9 | tt << 6, ea, 'b')
        else:
            self._op_ea(0x0800 | tt << 6, ea, 'b', [bit & 0xff])

    def btst(self, bit, ea):
        self._bit(0, bit, ea)

    def bchg(self, bit, ea):
        self._bit(1, bit, ea)

    def bclr(self, bit, ea):
        self._bit(2, bit, ea)

    def bset(self, bit, ea):
        self._bit(3, bit, ea)

    # -- program control ---------------------------------------------------
    def bcond(self, cond, target, size='w'):
        op = 0x6000 | _cond(cond) << 8
        ref = self.pc + 2
        if size == 's':
            if isinstance(target, int):
                d = target - ref
                assert -128 <= d < 128 and d not in (0, -1)
                self._w(op | (d & 0xff))
            else:
                self.fixups.append((self.pc, 'pc8', target, ref))
                self._w(op)
        elif size == 'w':
            self._w(op)
            self._fix('pc16', target, ref)
        else:
            self._w(op | 0xff)
            self._fix('pc32', target, ref)

    def dbcond(self, cond, dn, target):
        self._w(0x50c8 | _cond(cond) << 8 | dn.n)
        self._fix('pc16', target, self.pc)

    def scond(self, cond, ea):
        self._op_ea(0x50c0 | _cond(cond) << 8, ea, 'b')

    def trapcc(self, cond, v=None, size=None):
        op = 0x50f8 | _cond(cond) << 8
        if size is None:
            self._w(op | 4)
        elif size == 'w':
            self._w(op | 2)
            self._w(v)
        else:
            self._w(op | 3)
            self._l(v)

    def jmp(self, ea):
        if isinstance(ea, (str, int)):
            ea = absl(ea)
        self._op_ea(0x4ec0, ea)

    def jsr(self, ea):
        if isinstance(ea, (str, int)):
            ea = absl(ea)
        self._op_ea(0x4e80, ea)

    def rts(self):
        self._w(0x4e75)

    def rte(self):
        self._w(0x4e73)

    def rtr(self):
        self._w(0x4e77)

    def rtd(self, d):
        self._w(0x4e74)
        self._w(d)

    def nop(self):
        self._w(0x4e71)

    def reset(self):
        self._w(0x4e70)

    def illegal(self):
        self._w(0x4afc)

    def bgnd(self):
        self._w(0x4afa)

    def bkpt(self, n):
        self._w(0x4848 | (n & 7))

    def trap(self, n):
        self._w(0x4e40 | (n & 15))

    def trapv(self):
        self._w(0x4e76)

    def stop(self, sr):
        self._w(0x4e72)
        self._w(sr)

    def lpstop(self, sr):
        self._w(0xf800)
        self._w(0x01c0)
        self._w(sr)

    def link_w(self, an, d):
        self._w(0x4e50 | an.n)
        self._w(d)

    def link_l(self, an, d):
        self._w(0x4808 | an.n)
        self._l(d)

    def unlk(self, an):
        self._w(0x4e58 | an.n)

    # -- CPU32 table lookup and interpolate ----------------------------------
    def _tbl(self, signed, rnd, size, src, dx):
        ext = (dx.n << 12 | (0x800 if signed else 0) |
               (0 if rnd else 0x400) | SIZE2[size] << 6)
        if isinstance(src, tuple):          # (Dym, Dyn) register form
            dym, dyn = src
            self._w(0xf800 | dym.n)
            self._w(ext | dyn.n)
        else:
            self._op_ea(0xf800, src, size, [ext | 0x100])


def _gen():
    """generate the regular sized method families"""
    def mk(fn, *a):
        return lambda self, *args: fn(self, *a, *args)

    for sz in 'bwl':
        for name, base in (('add', 0xd000), ('sub', 0x9000),
                           ('and', 0xc000), ('or', 0x8000),
                           ('cmp', 0xb000)):
            setattr(Asm, '%s_%s' % (name, sz),
                    mk(lambda self, base, sz, s, d:
                       self._arith(base, sz, s, d,
                                   True if base in (0xd000, 0x9000, 0xb000)
                                   else None), base, sz))
        for name, base in (('addi', 0x0600), ('subi', 0x0400),
                           ('andi', 0x0200), ('ori', 0x0000),
                           ('eori', 0x0a00), ('cmpi', 0x0c00)):
            setattr(Asm, '%s_%s' % (name, sz),
                    mk(lambda self, base, sz, v, d:
                       self._imm_op(base, sz, v, d), base, sz))
        setattr(Asm, 'addq_' + sz,
                mk(lambda self, sz, n, d: self._quick(0, sz, n, d), sz))
        setattr(Asm, 'subq_' + sz,
                mk(lambda self, sz, n, d: self._quick(1, sz, n, d), sz))
        for name, base in (('negx', 0x4000), ('clr', 0x4200),
                           ('neg', 0x4400), ('not', 0x4600),
                           ('tst', 0x4a00)):
            setattr(Asm, '%s_%s' % (name, sz),
                    mk(lambda self, base, sz, ea: self._unary(base, sz, ea),
                       base, sz))
        for name, typ in (('as', 0), ('ls', 1), ('rox', 2), ('ro', 3)):
            for d, left in (('l', True), ('r', False)):
                setattr(Asm, '%s%s_%s' % (name, d, sz),
                        mk(lambda self, typ, left, sz, c, dy:
                           self._shift(typ, left, sz, c, dy), typ, left, sz))
        for name, signed, rnd in (('tblu', False, True),
                                  ('tblun', False, False),
                                  ('tbls', True, True),
                                  ('tblsn', True, False)):
            setattr(Asm, '%s_%s' % (name, sz),
                    mk(lambda self, signed, rnd, sz, src, dx:
                       self._tbl(signed, rnd, sz, src, dx), signed, rnd, sz))
    for name, typ in (('as', 0), ('ls', 1), ('rox', 2), ('ro', 3)):
        for d, left in (('l', True), ('r', False)):
            setattr(Asm, '%s%s_m' % (name, d),
                    mk(lambda self, typ, left, ea:
                       self._shift_m(typ, left, ea), typ, left))
    for c in COND:
        if c not in ('t', 'f'):
            setattr(Asm, 'b' + c, mk(lambda self, c, t: self.bcond(c, t, 'w'),
                                     c))
            setattr(Asm, 'b%s_s' % c,
                    mk(lambda self, c, t: self.bcond(c, t, 's'), c))
            setattr(Asm, 'b%s_w' % c,
                    mk(lambda self, c, t: self.bcond(c, t, 'w'), c))
            setattr(Asm, 'b%s_l' % c,
                    mk(lambda self, c, t: self.bcond(c, t, 'l'), c))
        setattr(Asm, 'db' + c, mk(lambda self, c, dn, t: self.dbcond(c, dn, t),
                                  c))
        setattr(Asm, 's' + c, mk(lambda self, c, ea: self.scond(c, ea), c))
    for nm, c, sz in (('bra', 't', 'w'), ('bra_s', 't', 's'),
                      ('bra_w', 't', 'w'), ('bra_l', 't', 'l'),
                      ('bsr', 'f', 'w'), ('bsr_s', 'f', 's'),
                      ('bsr_w', 'f', 'w'), ('bsr_l', 'f', 'l')):
        setattr(Asm, nm, mk(lambda self, c, sz, t: self.bcond(c, t, sz), c, sz))
    Asm.dbra = Asm.dbf


_gen()
