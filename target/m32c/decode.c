/*
 * Renesas M32C/80 instruction decoder
 *
 * The instruction set uses variable length CISC encodings:
 *
 *   [09|41|49 indirect prefix] [01 page prefix] op1 [op2]
 *   [src add-on] [dst add-on] [immediate(s)]
 *
 * Operands are mostly given by a 5-bit code split between op1 and op2
 * (src: op1[6:4]/op2[5:4], dst: op1[3:1]/op2[7:6]) or a 2-bit "dst2"
 * code in op1[5:4].  The opcode patterns themselves come from
 * insn-patterns.c.inc; the most specific matching pattern wins, as in
 * SLEIGH.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "cpu.h"
#include "decode.h"

bool m32c_cond(uint32_t flg, int cnd)
{
    bool c = flg & FLG_C, z = flg & FLG_Z, s = flg & FLG_S, o = flg & FLG_O;

    switch (cnd & 0xf) {
    case 0x0: return !c;                /* LTU / NC */
    case 0x1: return !c || z;           /* LEU */
    case 0x2: return !z;                /* NE */
    case 0x3: return !s;                /* PZ */
    case 0x4: return !o;                /* NO */
    case 0x5: return !((s ^ o) || z);   /* GT */
    case 0x6: return !(s ^ o);          /* GE */
    case 0x8: return c;                 /* GEU / C */
    case 0x9: return c && !z;           /* GTU */
    case 0xa: return z;                 /* EQ */
    case 0xb: return s;                 /* N */
    case 0xc: return o;                 /* O */
    case 0xd: return (s ^ o) || z;      /* LE */
    case 0xe: return s ^ o;             /* LT */
    default:  return false;
    }
}

typedef struct DecodeCtx {
    M32CFetchFn fetch;
    void *opaque;
    uint32_t pos;
} DecodeCtx;

static uint32_t get(DecodeCtx *c, int n)
{
    uint32_t v = 0;

    for (int i = 0; i < n; i++) {
        v |= (uint32_t)c->fetch(c->opaque, (c->pos + i) & M32C_ADDR_MASK)
             << (8 * i);
    }
    c->pos += n;
    return v;
}

static int32_t sext(uint32_t v, int bytes)
{
    int sh = 32 - 8 * bytes;

    return (int32_t)(v << sh) >> sh;
}

/* operand code validity, see the SRC5/DST5 macros of the SLEIGH spec */
static bool code_valid(int hi, int lo, int size)
{
    if (hi > 4) {
        return false;
    }
    if (hi == 4 && size == 4 && !(lo & 2)) {
        return false;
    }
    return true;
}

static bool pattern_valid(const M32CInsnPattern *p, uint8_t b1, uint8_t b2)
{
    int dhi = (b1 >> 1) & 7, dlo = b2 >> 6;
    int shi = (b1 >> 4) & 7, slo = (b2 >> 4) & 3;
    int size = p->size ? p->size : 1;

    switch (p->form) {
    case F_SRC5_DST5: {
        int ssize = size, dsize = size;
        if (p->op == OP_ADDX || p->op == OP_SUBX || p->op == OP_EXTS ||
            p->op == OP_EXTZ) {
            ssize = 1;
        }
        return code_valid(shi, slo, ssize) && code_valid(dhi, dlo, dsize);
    }
    case F_DST5: case F_R0L_DST5: case F_R1H_DST5: case F_CREG16_DST5:
    case F_CREG24_DST5: case F_DREG24_DST5: case F_XCHG: case F_XCHG_AX:
    case F_SP_DST5: case F_DST5_R0L: case F_DST5_SP: case F_DST5_CREG24:
    case F_DST5_DREG24: case F_DST5_CREG16: case F_IMM_DST5: case F_IMM2_DST5:
    case F_Q4_DST5: case F_ADJNZ: case F_SH4_DST5: case F_BIT:
        if (p->form == F_CREG24_DST5 || p->form == F_DREG24_DST5 ||
            p->form == F_DST5_CREG24 || p->form == F_DST5_DREG24) {
            size = 4;
        } else if (p->form == F_CREG16_DST5 || p->form == F_DST5_CREG16) {
            size = 2;
        } else if (p->form == F_BIT) {
            size = 1;
        }
        return code_valid(dhi, dlo, size);
    case F_IMM8_DST5:
        return code_valid(dhi, dlo, 4);
    case F_DST5A: case F_DST5A_REG:
        return dhi >= 1 && dhi <= 3;
    default:
        return true;
    }
}

/* add-on bytes of a 5-bit operand code */
static void read_addon(DecodeCtx *c, M32COperand *o)
{
    switch (o->hi) {
    case 1:
        o->disp = o->lo == 3 ? sext(get(c, 1), 1) : get(c, 1);
        break;
    case 2:
        o->disp = o->lo == 3 ? sext(get(c, 2), 2) : get(c, 2);
        break;
    case 3:
        if (o->lo == 3) {
            o->disp = get(c, 2);
        } else {
            o->disp = get(c, 3);
        }
        break;
    default:
        o->disp = 0;
        break;
    }
}

/* 2-bit dst code in op1[5:4], mapped onto the 5-bit representation */
static void read_dst2(DecodeCtx *c, M32COperand *o, uint8_t b1)
{
    switch ((b1 >> 4) & 3) {
    case 0:
        o->hi = 4;
        o->lo = 2;              /* R0L / R0 / R2R0 */
        o->disp = 0;
        break;
    case 1:
        o->hi = 3;
        o->lo = 3;              /* abs16 */
        o->disp = get(c, 2);
        break;
    case 2:
        o->hi = 1;
        o->lo = 2;              /* dsp:8[SB] */
        o->disp = get(c, 1);
        break;
    case 3:
        o->hi = 1;
        o->lo = 3;              /* dsp:8[FB] */
        o->disp = sext(get(c, 1), 1);
        break;
    }
}

static bool op_ends_tb(int op)
{
    switch (op) {
    case OP_INVALID:
    case OP_J_CND: case OP_JMP: case OP_JMPI: case OP_JMPS: case OP_JSR:
    case OP_JSRI: case OP_JSRS: case OP_RTS: case OP_REIT: case OP_FREIT:
    case OP_EXITD: case OP_INT: case OP_INTO: case OP_BRK: case OP_BRK2:
    case OP_WAIT: case OP_ADJNZ: case OP_LDC: case OP_POPC: case OP_FSET:
    case OP_FCLR: case OP_LDIPL: case OP_LDCTX: case OP_STCTX:
    case OP_INDEXB: case OP_INDEXBD: case OP_INDEXBS: case OP_INDEXL:
    case OP_INDEXLD: case OP_INDEXLS: case OP_INDEXW: case OP_INDEXWD:
    case OP_INDEXWS: case OP_BITINDEX:
    case OP_SMOVB: case OP_SMOVF: case OP_SMOVU: case OP_SSTR: case OP_SIN:
    case OP_SOUT: case OP_RMPA: case OP_SCMPU:
        return true;
    default:
        return false;
    }
}

bool m32c_decode(M32CInsn *d, uint32_t pc, M32CFetchFn fetch, void *opaque)
{
    DecodeCtx c = { .fetch = fetch, .opaque = opaque, .pos = pc };
    const M32CInsnPattern *best = NULL;
    int best_bits = -1;
    uint8_t b;

    memset(d, 0, sizeof(*d));
    d->pc = pc;

    b = get(&c, 1);
    if (b == 0x09 || b == 0x41 || b == 0x49) {
        d->ind_dst = b & 0x08;
        d->ind_src = b & 0x40;
        b = get(&c, 1);
    }
    if (b == 0x01) {
        d->page1 = true;
        b = get(&c, 1);
    }
    d->base = c.pos - 1;
    d->b1 = b;
    d->b2 = fetch(opaque, c.pos & M32C_ADDR_MASK);

    for (size_t i = 0; i < ARRAY_SIZE(m32c_patterns); i++) {
        const M32CInsnPattern *p = &m32c_patterns[i];
        int bits;

        if (p->page != d->page1 || (d->b1 & p->m1) != p->v1) {
            continue;
        }
        if (p->has_b2 && (d->b2 & p->m2) != p->v2) {
            continue;
        }
        if (!pattern_valid(p, d->b1, d->b2)) {
            continue;
        }
        bits = ctpop8(p->m1) + (p->has_b2 ? ctpop8(p->m2) : 0);
        if (bits > best_bits) {
            best = p;
            best_bits = bits;
        }
    }

    if (!best) {
        d->op = OP_INVALID;
        d->len = c.pos - pc;
        d->next = (pc + d->len) & M32C_ADDR_MASK;
        d->ends_tb = true;
        return false;
    }

    d->op = best->op;
    d->form = best->form;
    d->size = best->size ? best->size : 1;
    d->src_size = d->size;
    if (best->has_b2) {
        c.pos++;
    }

    d->dst.hi = (d->b1 >> 1) & 7;
    d->dst.lo = d->b2 >> 6;
    d->src.hi = (d->b1 >> 4) & 7;
    d->src.lo = (d->b2 >> 4) & 3;

    switch (d->form) {
    case F_SRC5_DST5:
        if (d->op == OP_ADDX || d->op == OP_SUBX || d->op == OP_EXTS ||
            d->op == OP_EXTZ) {
            d->src_size = 1;
        }
        read_addon(&c, &d->src);
        read_addon(&c, &d->dst);
        break;
    case F_DST5: case F_R0L_DST5: case F_R1H_DST5: case F_CREG16_DST5:
    case F_CREG24_DST5: case F_DREG24_DST5: case F_XCHG: case F_XCHG_AX:
    case F_DST5_R0L: case F_DST5_CREG24: case F_DST5_DREG24:
    case F_DST5_CREG16: case F_Q4_DST5: case F_SH4_DST5: case F_DST5A:
    case F_DST5A_REG:
        read_addon(&c, &d->dst);
        break;
    case F_BIT:
        read_addon(&c, &d->dst);
        if (d->op == OP_BM_CND) {
            d->extra = get(&c, 1);
        }
        break;
    case F_IMM_DST5:
        read_addon(&c, &d->dst);
        d->imm = get(&c, d->size);
        break;
    case F_IMM8_DST5:
        read_addon(&c, &d->dst);
        d->imm = get(&c, 1);
        d->src_size = 1;
        d->size = 4;
        break;
    case F_IMM2_DST5:
        read_addon(&c, &d->dst);
        d->imm = get(&c, d->size);
        d->imm2 = get(&c, d->size);
        break;
    case F_ADJNZ:
        read_addon(&c, &d->dst);
        d->target = (d->base + 2 + sext(get(&c, 1), 1)) & M32C_ADDR_MASK;
        break;
    case F_SP_DST5:
        d->src.hi = OPND_SPREL;
        d->src.disp = sext(get(&c, 1), 1);
        read_addon(&c, &d->dst);
        break;
    case F_DST5_SP:
        read_addon(&c, &d->dst);
        d->src.hi = OPND_SPREL;
        d->src.disp = sext(get(&c, 1), 1);
        break;
    case F_DST2_R0: case F_DST2_R1: case F_R0_DST2: case F_ZERO_DST2:
    case F_DST2_AX:
        read_dst2(&c, &d->dst, d->b1);
        break;
    case F_IMM_DST2:
        read_dst2(&c, &d->dst, d->b1);
        d->imm = get(&c, d->size);
        break;
    case F_IMM:
        if (d->op == OP_ENTER || d->op == OP_JMPS || d->op == OP_JSRS) {
            d->size = 1;
        }
        d->imm = get(&c, d->size);
        break;
    case F_IMM_AX:
        d->imm = get(&c, d->size == 2 ? 2 : 3);
        break;
    case F_IMM_SP:
        d->imm = d->b2 == 0x13 ? sext(get(&c, 2), 2) : sext(get(&c, 1), 1);
        break;
    case F_IMM_CREG16:
        d->imm = get(&c, 2);
        break;
    case F_IMM_CREG24: case F_IMM_DREG24:
        d->imm = get(&c, 3);
        break;
    case F_INTNUM:
        d->extra = get(&c, 1) >> 2;
        break;
    case F_REGLIST:
        d->extra = get(&c, 1);
        break;
    case F_REL8:
        d->target = (d->base + 1 + sext(get(&c, 1), 1)) & M32C_ADDR_MASK;
        break;
    case F_REL16:
        d->target = (d->base + 1 + sext(get(&c, 2), 2)) & M32C_ADDR_MASK;
        break;
    case F_REL3:
        d->target = (d->base + 2 + (((d->b1 >> 4) & 3) << 1) + (d->b1 & 1)) &
                    M32C_ADDR_MASK;
        break;
    case F_ABS24:
        d->target = get(&c, 3);
        break;
    case F_BTST_S:
        d->dst.hi = 3;
        d->dst.lo = 3;
        d->dst.disp = get(&c, 2);
        break;
    case F_CTX:
        d->imm = get(&c, 2);
        d->imm2 = get(&c, 3);
        break;
    default:
        break;
    }

    d->len = c.pos - pc;
    d->next = (pc + d->len) & M32C_ADDR_MASK;
    d->ends_tb = op_ends_tb(d->op);
    return true;
}
