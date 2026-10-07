/*
 * Renesas M32C/80 instruction decoder
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef M32C_DECODE_H
#define M32C_DECODE_H

/* operand forms, see gen-insn-patterns.py */
enum {
    F_NONE, F_BMC, F_R0_DST2, F_R0L_DST5, F_R1H_DST5, F_CTX, F_ABS24,
    F_BTST_S, F_CREG16, F_CREG24, F_CREG16_DST5, F_CREG24_DST5,
    F_DREG24_DST5, F_XCHG, F_XCHG_AX, F_BIT, F_SP_DST5, F_DST2_R0,
    F_DST2_R1, F_DST2_AX, F_DST5A, F_DST5A_REG, F_DST5, F_DST5_R0L,
    F_DST5_SP, F_DST5_CREG24, F_DST5_DREG24, F_DST5_CREG16, F_FLAG,
    F_REGLIST, F_REL16, F_REL3, F_REL8, F_SRC5_DST5, F_IMM, F_IMM_AX,
    F_IMM_CREG16, F_IMM_CREG24, F_IMM_DREG24, F_IMM_DST2, F_IMM_DST5,
    F_IMM8_DST5, F_IMM2_DST5, F_IMM1P_AX, F_IMM3P_SP, F_IMM_SP, F_IMM3,
    F_INTNUM, F_Q4_DST5, F_ADJNZ, F_SH4_DST5, F_ZERO_DST2,
};

typedef struct M32CInsnPattern {
    uint8_t page;
    uint8_t m1, v1, m2, v2;
    uint8_t has_b2;
    uint8_t op;
    uint8_t form;
    uint8_t size;
} M32CInsnPattern;

#include "insn-patterns.c.inc"

/* operand "hi" values beyond the 5-bit encoding */
#define OPND_SPREL  8       /* dsp:8[SP] */

typedef struct M32COperand {
    uint8_t hi;             /* 0..4 per the 5-bit encoding, or OPND_* */
    uint8_t lo;
    int32_t disp;           /* displacement or absolute address */
} M32COperand;

typedef struct M32CInsn {
    uint32_t pc;            /* first byte, including prefixes */
    uint32_t base;          /* address of the first opcode byte */
    uint32_t next;          /* address of the following instruction */
    int len;
    int op;
    int form;
    int size;               /* operation size: 1, 2, 3 (address), 4 */
    int src_size;
    uint8_t b1, b2;
    bool page1;
    bool ind_src, ind_dst;
    M32COperand src, dst;
    uint32_t imm, imm2;
    uint32_t target;        /* branch target of PC relative forms */
    uint8_t extra;          /* cnd byte, register list, INT number */
    bool ends_tb;
} M32CInsn;

typedef uint8_t (*M32CFetchFn)(void *opaque, uint32_t addr);

/* Returns false for an undefined instruction (op == OP_INVALID). */
bool m32c_decode(M32CInsn *d, uint32_t pc, M32CFetchFn fetch, void *opaque);

/* Condition code evaluation, @cnd per the 4-bit "cnd" encoding */
bool m32c_cond(uint32_t flg, int cnd);

#endif
