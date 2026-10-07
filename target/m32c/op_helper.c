/*
 * Renesas M32C/80 instruction execution
 *
 * Instructions are executed by an interpreter helper: the translator
 * decodes each instruction (to find its length and whether it ends the
 * translation block) and emits a call to helper_exec, which decodes it
 * again and executes it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "accel/tcg/cpu-loop.h"
#include "qemu/plugin.h"
#include "decode.h"

typedef struct ExecCtx {
    CPUM32CState *env;
    uintptr_t ra;
    /* INDEX / BITINDEX prefix state for the current instruction */
    uint32_t src_index, dst_index;
    bool bit_index;
    uint32_t bit_index_val;
} ExecCtx;

/* ---------------------------------------------------------------------- */
/* Memory                                                                 */
/* ---------------------------------------------------------------------- */

static uint32_t ld(ExecCtx *x, uint32_t addr, int size)
{
    CPUM32CState *env = x->env;

    addr &= M32C_ADDR_MASK;
    switch (size) {
    case 1:
        return cpu_ldub_data_ra(env, addr, x->ra);
    case 2:
        return cpu_lduw_le_data_ra(env, addr, x->ra);
    case 3:
        return cpu_lduw_le_data_ra(env, addr, x->ra) |
               (cpu_ldub_data_ra(env, (addr + 2) & M32C_ADDR_MASK, x->ra)
                << 16);
    default:
        return cpu_ldl_le_data_ra(env, addr, x->ra);
    }
}

static void st(ExecCtx *x, uint32_t addr, int size, uint32_t v)
{
    CPUM32CState *env = x->env;

    addr &= M32C_ADDR_MASK;
    switch (size) {
    case 1:
        cpu_stb_data_ra(env, addr, v, x->ra);
        break;
    case 2:
        cpu_stw_le_data_ra(env, addr, v, x->ra);
        break;
    case 3:
        cpu_stw_le_data_ra(env, addr, v, x->ra);
        cpu_stb_data_ra(env, (addr + 2) & M32C_ADDR_MASK, v >> 16, x->ra);
        break;
    default:
        cpu_stl_le_data_ra(env, addr, v, x->ra);
        break;
    }
}

/* stack slots are 2 bytes for 8/16-bit pushes and 4 bytes otherwise */
static void push(ExecCtx *x, int size, uint32_t v)
{
    int slot = size <= 2 ? 2 : 4;
    uint32_t sp = (m32c_get_sp(x->env) - slot) & M32C_ADDR_MASK;

    st(x, sp, size, v);
    m32c_set_sp(x->env, sp);
}

static uint32_t pop(ExecCtx *x, int size)
{
    int slot = size <= 2 ? 2 : 4;
    uint32_t sp = m32c_get_sp(x->env);
    uint32_t v = ld(x, sp, size);

    m32c_set_sp(x->env, sp + slot);
    return v;
}

/* ---------------------------------------------------------------------- */
/* Registers and operands                                                 */
/* ---------------------------------------------------------------------- */

enum { LOC_MEM, LOC_REG8, LOC_REG16, LOC_REG32, LOC_AX };

typedef struct Loc {
    int kind;
    int idx;
    uint32_t addr;
} Loc;

#define R(x, n)  ((x)->env->r[m32c_bank((x)->env)][n])
#define A(x, n)  ((x)->env->a[m32c_bank((x)->env)][n])
#define FB(x)    ((x)->env->fb[m32c_bank((x)->env)])
#define SB(x)    ((x)->env->sb[m32c_bank((x)->env)])

enum { R0 = 0, R1 = 1, R2 = 2, R3 = 3 };

static uint32_t size_mask(int size)
{
    return size >= 4 ? 0xffffffffu : (1u << (8 * size)) - 1;
}

static uint32_t sign_bit(int size)
{
    return 1u << (8 * (size >= 4 ? 4 : size) - 1);
}

static int32_t sx(uint32_t v, int size)
{
    int sh = 32 - 8 * (size >= 4 ? 4 : size);

    return (int32_t)(v << sh) >> sh;
}

static uint32_t base_reg(ExecCtx *x, int lo)
{
    switch (lo) {
    case 0: return A(x, 0);
    case 1: return A(x, 1);
    case 2: return SB(x);
    default: return FB(x);
    }
}

static Loc resolve(ExecCtx *x, const M32COperand *o, int size, bool ind,
                   uint32_t index)
{
    Loc l = { .kind = LOC_MEM };
    uint32_t addr;

    switch (o->hi) {
    case 4:
        l.kind = size == 1 ? LOC_REG8 : size == 2 ? LOC_REG16 : LOC_REG32;
        l.idx = o->lo;
        return l;
    case 0:
        if (o->lo & 2) {
            if (ind) {
                l.addr = A(x, o->lo & 1);
                return l;
            }
            l.kind = LOC_AX;
            l.idx = o->lo & 1;
            return l;
        }
        addr = A(x, o->lo) + index;
        break;
    case 1:
    case 2:
        addr = base_reg(x, o->lo) + o->disp + index;
        break;
    case 3:
        if (o->lo < 2) {
            addr = A(x, o->lo) + o->disp + index;
        } else {
            addr = o->disp + index;
        }
        break;
    case OPND_SPREL:
        l.addr = (m32c_get_sp(x->env) + o->disp) & M32C_ADDR_MASK;
        return l;
    default:
        g_assert_not_reached();
    }
    addr &= M32C_ADDR_MASK;
    if (ind) {
        addr = ld(x, addr, 3);
    }
    l.addr = addr;
    return l;
}

static uint32_t rd(ExecCtx *x, const Loc *l, int size)
{
    switch (l->kind) {
    case LOC_REG8: {
        /* 0 R0H, 1 R1H, 2 R0L, 3 R1L */
        uint16_t r = R(x, l->idx & 1);
        return l->idx & 2 ? r & 0xff : r >> 8;
    }
    case LOC_REG16: {
        static const int map[4] = { R2, R3, R0, R1 };
        return R(x, map[l->idx]);
    }
    case LOC_REG32:
        return l->idx & 1 ? ((uint32_t)R(x, R3) << 16) | R(x, R1)
                          : ((uint32_t)R(x, R2) << 16) | R(x, R0);
    case LOC_AX:
        return A(x, l->idx) & size_mask(size);
    default:
        return ld(x, l->addr, size);
    }
}

static void wr(ExecCtx *x, const Loc *l, int size, uint32_t v)
{
    switch (l->kind) {
    case LOC_REG8: {
        uint16_t *r = &R(x, l->idx & 1);
        if (l->idx & 2) {
            *r = (*r & 0xff00) | (v & 0xff);
        } else {
            *r = (*r & 0x00ff) | ((v & 0xff) << 8);
        }
        break;
    }
    case LOC_REG16: {
        static const int map[4] = { R2, R3, R0, R1 };
        R(x, map[l->idx]) = v;
        break;
    }
    case LOC_REG32:
        if (l->idx & 1) {
            R(x, R1) = v;
            R(x, R3) = v >> 16;
        } else {
            R(x, R0) = v;
            R(x, R2) = v >> 16;
        }
        break;
    case LOC_AX:
        A(x, l->idx) = v & size_mask(size) & M32C_ADDR_MASK;
        break;
    default:
        st(x, l->addr, size, v);
        break;
    }
}

static Loc dst_loc(ExecCtx *x, const M32CInsn *d, int size)
{
    return resolve(x, &d->dst, size, d->ind_dst, x->dst_index);
}

static Loc src_loc(ExecCtx *x, const M32CInsn *d, int size)
{
    return resolve(x, &d->src, size, d->ind_src, x->src_index);
}

/* ---------------------------------------------------------------------- */
/* Flags                                                                  */
/* ---------------------------------------------------------------------- */

static void setf(ExecCtx *x, uint32_t f, bool v)
{
    if (v) {
        x->env->flg |= f;
    } else {
        x->env->flg &= ~f;
    }
}

static void set_sz(ExecCtx *x, int size, uint32_t v)
{
    setf(x, FLG_S, v & sign_bit(size));
    setf(x, FLG_Z, !(v & size_mask(size)));
}

static uint32_t do_add(ExecCtx *x, int size, uint32_t a, uint32_t b,
                       uint32_t cin)
{
    uint32_t m = size_mask(size);
    uint64_t r = (uint64_t)(a & m) + (b & m) + cin;
    uint32_t res = r & m;

    setf(x, FLG_C, r > m);
    setf(x, FLG_O, (~(a ^ b) & (a ^ res)) & sign_bit(size));
    set_sz(x, size, res);
    return res;
}

/* a - b - borrow; C = no borrow */
static uint32_t do_sub(ExecCtx *x, int size, uint32_t a, uint32_t b,
                       uint32_t borrow)
{
    uint32_t m = size_mask(size);
    uint32_t res = ((a & m) - (b & m) - borrow) & m;

    setf(x, FLG_C, (uint64_t)(a & m) >= (uint64_t)(b & m) + borrow);
    setf(x, FLG_O, ((a ^ b) & (a ^ res)) & sign_bit(size));
    set_sz(x, size, res);
    return res;
}

/* ---------------------------------------------------------------------- */
/* Control registers                                                      */
/* ---------------------------------------------------------------------- */

/* creg16: DCT0 DCT1 FLG SVF DRC0 DRC1 DMD0 DMD1 */
static uint32_t *creg16(CPUM32CState *env, int n)
{
    switch (n) {
    case 0: return &env->dct[0];
    case 1: return &env->dct[1];
    case 2: return &env->flg;
    case 3: return &env->svf;
    case 4: return &env->drc[0];
    case 5: return &env->drc[1];
    case 6: return &env->dmd[0];
    default: return &env->dmd[1];
    }
}

/* creg24: INTB SP SB FB SVP VCT - ISP */
static uint32_t get_creg24(ExecCtx *x, int n)
{
    CPUM32CState *env = x->env;

    switch (n) {
    case 0: return env->intb;
    case 1: return m32c_get_sp(env);
    case 2: return SB(x);
    case 3: return FB(x);
    case 4: return env->svp;
    case 5: return env->vct;
    case 7: return env->isp;
    default: return 0;
    }
}

static void set_creg24(ExecCtx *x, int n, uint32_t v)
{
    CPUM32CState *env = x->env;

    v &= M32C_ADDR_MASK;
    switch (n) {
    case 0: env->intb = v; break;
    case 1: m32c_set_sp(env, v); break;
    case 2: SB(x) = v; break;
    case 3: FB(x) = v; break;
    case 4: env->svp = v; break;
    case 5: env->vct = v; break;
    case 7: env->isp = v; break;
    }
}

/* dreg24: - - DMA0 DMA1 DRA0 DRA1 DSA0 DSA1 */
static uint32_t *dreg24(CPUM32CState *env, int n)
{
    static uint32_t dummy;

    switch (n) {
    case 2: return &env->dma[0];
    case 3: return &env->dma[1];
    case 4: return &env->dra[0];
    case 5: return &env->dra[1];
    case 6: return &env->dsa[0];
    case 7: return &env->dsa[1];
    default: return &dummy;
    }
}

/* ---------------------------------------------------------------------- */
/* Interrupts                                                             */
/* ---------------------------------------------------------------------- */

static void enter_int(ExecCtx *x, uint32_t ret_pc, uint32_t vector_addr,
                      bool clear_u, int ipl)
{
    CPUM32CState *env = x->env;
    uint32_t flg = env->flg;

    env->flg &= ~(FLG_I | FLG_D);
    if (clear_u) {
        env->flg &= ~FLG_U;
    }
    push(x, 2, flg);
    push(x, 4, ret_pc);
    if (ipl >= 0) {
        env->flg = (env->flg & ~FLG_IPL_MASK) | (ipl << FLG_IPL_SHIFT);
    }
    env->pc = ld(x, vector_addr, 3);
}

static void sw_int(ExecCtx *x, uint32_t ret_pc, int num)
{
    enter_int(x, ret_pc, x->env->intb + num * 4, num < 32, -1);
}

void m32c_cpu_do_interrupt(CPUState *cs)
{
    /* all interrupt entry happens in m32c_cpu_exec_interrupt */
    cs->exception_index = -1;
}

bool m32c_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    CPUM32CState *env = cpu_env(cs);
    M32CIrqRequest req;
    ExecCtx x = { .env = env, .ra = 0 };
    uint64_t last_pc = env->pc;

    if (!(interrupt_request & CPU_INTERRUPT_HARD) || !env->irq_query) {
        return false;
    }
    if (!env->irq_query(env->irq_opaque, &req)) {
        return false;
    }
    if (req.level < 8) {
        int ipl = (env->flg & FLG_IPL_MASK) >> FLG_IPL_SHIFT;
        if (!(env->flg & FLG_I) || req.level <= ipl) {
            return false;
        }
    }
    if (env->irq_ack) {
        env->irq_ack(env->irq_opaque, &req);
    }
    cs->halted = 0;

    qemu_log_mask(CPU_LOG_INT, "m32c: interrupt %d level %d at pc=0x%06x\n",
                  req.vec, req.level, env->pc);

    if (req.fast) {
        env->svf = env->flg;
        env->svp = env->pc;
        env->flg &= ~(FLG_I | FLG_D | FLG_U);
        env->flg = (env->flg & ~FLG_IPL_MASK) | (7 << FLG_IPL_SHIFT);
        env->pc = env->vct;
    } else if (req.level >= 8) {
        enter_int(&x, env->pc, req.fixed_vec, true, 7);
    } else {
        enter_int(&x, env->pc, env->intb + req.vec * 4, true, req.level);
    }
    qemu_plugin_vcpu_interrupt_cb(cs, last_pc);
    return true;
}

/* ---------------------------------------------------------------------- */
/* Shifts                                                                 */
/* ---------------------------------------------------------------------- */

static uint32_t do_shift(ExecCtx *x, int size, uint32_t v, int n,
                         bool arith)
{
    int bits = 8 * (size >= 4 ? 4 : size);
    uint32_t m = size_mask(size);
    uint32_t res;

    v &= m;
    if (n == 0) {
        return v;
    }
    if (n > 0) {
        if (n >= bits) {
            setf(x, FLG_C, n == bits && (v & 1));
            if (arith) {
                setf(x, FLG_O, v != 0);
            }
            set_sz(x, size, 0);
            return 0;
        }
        setf(x, FLG_C, n <= bits && ((uint64_t)v >> (bits - n)) & 1);
        res = ((uint64_t)v << n) & m;
        if (arith) {
            /* overflow when the bits shifted out differ from the sign */
            uint64_t outmask = (((uint64_t)1 << (n + 1)) - 1) << (bits - n - 1);
            uint64_t out = (uint64_t)v & outmask & m;
            setf(x, FLG_O, out != 0 && out != (outmask & m));
        }
    } else {
        n = -n;
        if (n > bits) {
            n = bits;
        }
        setf(x, FLG_C, ((uint64_t)v >> (n - 1)) & 1);
        if (arith) {
            res = (uint32_t)((int64_t)sx(v, size) >> n) & m;
            setf(x, FLG_O, false);
        } else {
            res = (uint32_t)((uint64_t)v >> n);
        }
    }
    set_sz(x, size, res);
    return res;
}

static uint32_t do_rot(ExecCtx *x, int size, uint32_t v, int n)
{
    int bits = 8 * size;
    uint32_t m = size_mask(size);

    v &= m;
    n %= bits;
    if (n > 0) {
        v = ((v << n) | (v >> (bits - n))) & m;
        setf(x, FLG_C, v & 1);
    } else if (n < 0) {
        n = -n;
        v = ((v >> n) | (v << (bits - n))) & m;
        setf(x, FLG_C, v & sign_bit(size));
    }
    set_sz(x, size, v);
    return v;
}

/* ---------------------------------------------------------------------- */
/* Decimal                                                                */
/* ---------------------------------------------------------------------- */

static uint32_t bcd_add(ExecCtx *x, int size, uint32_t a, uint32_t b,
                        int cin)
{
    uint32_t res = 0;
    int carry = cin;

    for (int i = 0; i < 2 * size; i++) {
        int d = ((a >> (4 * i)) & 0xf) + ((b >> (4 * i)) & 0xf) + carry;
        carry = d > 9;
        if (carry) {
            d -= 10;
        }
        res |= (uint32_t)d << (4 * i);
    }
    setf(x, FLG_C, carry);
    set_sz(x, size, res);
    return res;
}

static uint32_t bcd_sub(ExecCtx *x, int size, uint32_t a, uint32_t b,
                        int bin)
{
    uint32_t res = 0;
    int borrow = bin;

    for (int i = 0; i < 2 * size; i++) {
        int d = ((a >> (4 * i)) & 0xf) - ((b >> (4 * i)) & 0xf) - borrow;
        borrow = d < 0;
        if (borrow) {
            d += 10;
        }
        res |= (uint32_t)d << (4 * i);
    }
    setf(x, FLG_C, !borrow);
    set_sz(x, size, res);
    return res;
}

/* ---------------------------------------------------------------------- */
/* Bit operations                                                         */
/* ---------------------------------------------------------------------- */

static Loc bit_loc(ExecCtx *x, const M32CInsn *d, int *bit)
{
    Loc l;

    *bit = d->b2 & 7;
    l = resolve(x, &d->dst, 1, false, 0);
    if (x->bit_index && l.kind == LOC_MEM) {
        l.addr = (l.addr + x->bit_index_val / 8) & M32C_ADDR_MASK;
        *bit = x->bit_index_val % 8;
    }
    return l;
}

/* ---------------------------------------------------------------------- */
/* Execution                                                              */
/* ---------------------------------------------------------------------- */

static uint8_t fetch_code(void *opaque, uint32_t addr)
{
    return cpu_ldb_code_mmu(opaque, addr, make_memop_idx(MO_UB, 0), 0);
}

static void undefined(ExecCtx *x, const M32CInsn *d)
{
    qemu_log_mask(LOG_GUEST_ERROR, "m32c: undefined instruction at 0x%06x\n",
                  d->pc);
    enter_int(x, d->next, M32C_VEC_UND, true, -1);
}

static void exec_insn(ExecCtx *x, const M32CInsn *d)
{
    CPUM32CState *env = x->env;
    int size = d->size;
    uint32_t next = d->next;
    Loc dl, sl;
    uint32_t a, b, r;
    int32_t sa, sb;

    env->pc = next;

    switch (d->op) {
    case OP_NOP:
        break;

    /* ---- moves ---- */
    case OP_MOV:
        switch (d->form) {
        case F_IMM_DST5: case F_IMM_DST2:
            dl = dst_loc(x, d, size);
            wr(x, &dl, size, d->imm);
            set_sz(x, size, d->imm);
            break;
        case F_Q4_DST5:
            r = sx(d->b2 & 0xf, 1) & size_mask(size);
            dl = dst_loc(x, d, size);
            wr(x, &dl, size, r);
            set_sz(x, size, r);
            break;
        case F_ZERO_DST2:
            dl = dst_loc(x, d, size);
            wr(x, &dl, size, 0);
            set_sz(x, size, 0);
            break;
        case F_IMM_AX:
            A(x, d->b1 & 1) = d->imm & M32C_ADDR_MASK;
            set_sz(x, size == 2 ? 2 : 3, d->imm);
            break;
        case F_SRC5_DST5: case F_SP_DST5:
            sl = src_loc(x, d, size);
            r = rd(x, &sl, size);
            dl = dst_loc(x, d, size);
            wr(x, &dl, size, r);
            set_sz(x, size, r);
            break;
        case F_DST5_SP:
            dl = dst_loc(x, d, size);
            r = rd(x, &dl, size);
            sl = src_loc(x, d, size);
            wr(x, &sl, size, r);
            set_sz(x, size, r);
            break;
        case F_R0_DST2: {       /* MOV R0L/R0, dst2 */
            Loc r0 = { .kind = size == 1 ? LOC_REG8 : LOC_REG16, .idx = 2 };
            r = rd(x, &r0, size);
            dl = dst_loc(x, d, size);
            wr(x, &dl, size, r);
            set_sz(x, size, r);
            break;
        }
        case F_DST2_R0: case F_DST2_R1: {
            int idx = d->form == F_DST2_R0 ? 2 : 3;
            Loc rr = { .kind = size == 1 ? LOC_REG8 : LOC_REG16, .idx = idx };
            dl = dst_loc(x, d, size);
            r = rd(x, &dl, size);
            wr(x, &rr, size, r);
            set_sz(x, size, r);
            break;
        }
        case F_DST2_AX:
            dl = dst_loc(x, d, 4);
            r = rd(x, &dl, 4);
            A(x, d->b1 & 1) = r & M32C_ADDR_MASK;
            set_sz(x, 4, r);
            break;
        default:
            undefined(x, d);
        }
        break;
    case OP_MOVX:
        r = sx(d->imm, 1);
        dl = dst_loc(x, d, 4);
        wr(x, &dl, 4, r);
        set_sz(x, 4, r);
        break;
    case OP_MOVA:
        dl = dst_loc(x, d, 3);
        if (d->b2 & 0x02) {     /* MOVA dst, Ax */
            A(x, d->b2 & 1) = dl.addr;
        } else {
            Loc rr = { .kind = LOC_REG32, .idx = d->b2 & 1 };
            wr(x, &rr, 4, dl.addr);
        }
        break;
    case OP_MOVLL: case OP_MOVHL: case OP_MOVLH: case OP_MOVHH: {
        Loc r0l = { .kind = LOC_REG8, .idx = 2 };
        uint32_t rv = rd(x, &r0l, 1);
        dl = dst_loc(x, d, 1);
        a = rd(x, &dl, 1);
        if (d->form == F_R0L_DST5) {    /* R0L -> dst */
            switch (d->op) {
            case OP_MOVLL: r = (a & 0xf0) | (rv & 0x0f); break;
            case OP_MOVHL: r = (a & 0xf0) | (rv >> 4); break;
            case OP_MOVLH: r = (a & 0x0f) | (rv << 4); break;
            default:       r = (a & 0x0f) | (rv & 0xf0); break;
            }
            wr(x, &dl, 1, r);
        } else {                        /* dst -> R0L */
            switch (d->op) {
            case OP_MOVLL: r = (rv & 0xf0) | (a & 0x0f); break;
            case OP_MOVHL: r = (rv & 0xf0) | (a >> 4); break;
            case OP_MOVLH: r = (rv & 0x0f) | (a << 4); break;
            default:       r = (rv & 0x0f) | (a & 0xf0); break;
            }
            wr(x, &r0l, 1, r);
        }
        break;
    }
    case OP_STZ: case OP_STNZ:
        if (!!(env->flg & FLG_Z) == (d->op == OP_STZ)) {
            dl = dst_loc(x, d, size);
            wr(x, &dl, size, d->imm);
        }
        break;
    case OP_STZX:
        dl = dst_loc(x, d, size);
        wr(x, &dl, size, (env->flg & FLG_Z) ? d->imm : d->imm2);
        break;
    case OP_XCHG: {
        Loc rr;
        if (d->form == F_XCHG_AX) {
            rr = (Loc) { .kind = LOC_AX, .idx = d->b2 & 1 };
        } else if (size == 1) {
            /* R0L R1L - - R0H R1H */
            rr = (Loc) { .kind = LOC_REG8,
                         .idx = ((d->b2 & 4) ? 0 : 2) | (d->b2 & 1) };
        } else {
            static const int map[8] = { 2, 3, 0, 0, 0, 1, 0, 0 };
            rr = (Loc) { .kind = LOC_REG16, .idx = map[d->b2 & 7] };
        }
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size);
        b = rd(x, &rr, size);
        wr(x, &dl, size, b);
        wr(x, &rr, size, a);
        break;
    }

    /* ---- arithmetic ---- */
    case OP_ADD: case OP_SUB: case OP_CMP: case OP_ADC: case OP_SBB:
    case OP_AND: case OP_OR: case OP_XOR: case OP_TST:
    case OP_ADDX: case OP_SUBX: case OP_CMPX: {
        bool is_sp = false;
        int ssize = d->src_size;

        switch (d->form) {
        case F_IMM_DST5: case F_IMM_DST2: case F_IMM8_DST5:
            b = d->imm;
            if (d->form == F_IMM8_DST5) {
                b = sx(b, 1);
            }
            break;
        case F_Q4_DST5:
            b = sx(d->b2 & 0xf, 1);
            break;
        case F_SRC5_DST5:
            sl = src_loc(x, d, ssize);
            b = rd(x, &sl, ssize);
            if (ssize < size) {
                b = sx(b, ssize);
            }
            break;
        case F_DST2_R0: {       /* CMP dst2, R0 */
            Loc r0 = { .kind = size == 1 ? LOC_REG8 : LOC_REG16, .idx = 2 };
            dl = dst_loc(x, d, size);
            a = rd(x, &r0, size);
            b = rd(x, &dl, size);
            do_sub(x, size, a, b, 0);
            goto done;
        }
        case F_IMM1P_AX:        /* ADD.L:S #1/#2, Ax */
            b = ((d->b1 >> 5) & 1) + 1;
            a = A(x, d->b1 & 1);
            r = do_add(x, 4, sx(a, 3), b, 0);
            A(x, d->b1 & 1) = r & M32C_ADDR_MASK;
            goto done;
        case F_IMM3P_SP:        /* ADD.L:Q #1..8, SP */
            b = (((d->b1 >> 4) & 3) << 1) + (d->b1 & 1) + 1;
            is_sp = true;
            break;
        case F_IMM_SP:
            b = d->imm;
            is_sp = true;
            break;
        default:
            undefined(x, d);
            goto done;
        }
        if (is_sp) {
            r = do_add(x, 3, m32c_get_sp(env), b, 0);
            m32c_set_sp(env, r);
            break;
        }
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size);
        switch (d->op) {
        case OP_ADD: case OP_ADDX:
            r = do_add(x, size, a, b, 0);
            wr(x, &dl, size, r);
            break;
        case OP_ADC:
            r = do_add(x, size, a, b, !!(env->flg & FLG_C));
            wr(x, &dl, size, r);
            break;
        case OP_SUB: case OP_SUBX:
            r = do_sub(x, size, a, b, 0);
            wr(x, &dl, size, r);
            break;
        case OP_SBB:
            r = do_sub(x, size, a, b, !(env->flg & FLG_C));
            wr(x, &dl, size, r);
            break;
        case OP_CMP: case OP_CMPX:
            do_sub(x, size, a, b, 0);
            break;
        case OP_AND:
            r = a & b;
            wr(x, &dl, size, r);
            set_sz(x, size, r);
            break;
        case OP_OR:
            r = a | b;
            wr(x, &dl, size, r);
            set_sz(x, size, r);
            break;
        case OP_XOR:
            r = a ^ b;
            wr(x, &dl, size, r);
            set_sz(x, size, r);
            break;
        case OP_TST:
            set_sz(x, size, a & b);
            break;
        }
        break;
    }
    case OP_ADCF:
        dl = dst_loc(x, d, size);
        r = do_add(x, size, rd(x, &dl, size), 0, !!(env->flg & FLG_C));
        wr(x, &dl, size, r);
        break;
    case OP_INC: case OP_DEC:
        dl = dst_loc(x, d, size);
        r = (rd(x, &dl, size) + (d->op == OP_INC ? 1 : -1)) & size_mask(size);
        wr(x, &dl, size, r);
        set_sz(x, size, r);
        break;
    case OP_NEG:
        dl = dst_loc(x, d, size);
        r = do_sub(x, size, 0, rd(x, &dl, size), 0);
        wr(x, &dl, size, r);
        break;
    case OP_NOT:
        dl = dst_loc(x, d, size);
        r = ~rd(x, &dl, size) & size_mask(size);
        wr(x, &dl, size, r);
        set_sz(x, size, r);
        break;
    case OP_ABS:
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size);
        setf(x, FLG_O, (a & size_mask(size)) == sign_bit(size));
        r = sx(a, size) < 0 ? (-a) & size_mask(size) : a;
        wr(x, &dl, size, r);
        set_sz(x, size, r);
        break;
    case OP_EXTS:
        if (d->form == F_SRC5_DST5) {
            sl = src_loc(x, d, 1);
            r = sx(rd(x, &sl, 1), 1) & 0xffff;
            dl = dst_loc(x, d, 2);
            wr(x, &dl, 2, r);
            set_sz(x, 2, r);
        } else {
            int big = size == 1 ? 2 : 4;
            dl = dst_loc(x, d, size);
            r = sx(rd(x, &dl, size), size) & size_mask(big);
            dl = dst_loc(x, d, big);
            wr(x, &dl, big, r);
            set_sz(x, big, r);
        }
        break;
    case OP_EXTZ:
        sl = src_loc(x, d, 1);
        r = rd(x, &sl, 1);
        dl = dst_loc(x, d, 2);
        wr(x, &dl, 2, r);
        set_sz(x, 2, r);
        break;
    case OP_MAX: case OP_MIN: case OP_CLIP:
        dl = dst_loc(x, d, size);
        sa = sx(rd(x, &dl, size), size);
        if (d->op == OP_CLIP) {
            int32_t lo = sx(d->imm, size), hi = sx(d->imm2, size);
            r = sa < lo ? lo : sa > hi ? hi : sa;
        } else {
            if (d->form == F_SRC5_DST5) {
                sl = src_loc(x, d, size);
                sb = sx(rd(x, &sl, size), size);
            } else {
                sb = sx(d->imm, size);
            }
            r = d->op == OP_MAX ? MAX(sa, sb) : MIN(sa, sb);
        }
        wr(x, &dl, size, r);
        break;
    case OP_MUL: case OP_MULU: {
        int big = size == 1 ? 2 : 4;
        uint64_t p;
        if (d->form == F_SRC5_DST5) {
            sl = src_loc(x, d, size);
            b = rd(x, &sl, size);
        } else {
            b = d->imm;
        }
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size);
        if (d->op == OP_MUL) {
            p = (int64_t)sx(a, size) * sx(b, size);
        } else {
            p = (uint64_t)(a & size_mask(size)) * (b & size_mask(size));
        }
        if (dl.kind == LOC_AX) {
            wr(x, &dl, big, p);
        } else {
            dl = dst_loc(x, d, big);
            wr(x, &dl, big, p);
        }
        break;
    }
    case OP_MULEX: {
        int64_t p;
        dl = dst_loc(x, d, 2);
        p = (int64_t)(int32_t)(((uint32_t)R(x, R2) << 16) | R(x, R0)) *
            sx(rd(x, &dl, 2), 2);
        R(x, R0) = p;
        R(x, R2) = p >> 16;
        R(x, R1) = p >> 32;
        break;
    }
    case OP_DIV: case OP_DIVU: case OP_DIVX: {
        int64_t dividend, divisor, q, rem;
        if (d->form == F_IMM) {
            b = d->imm;
        } else {
            dl = dst_loc(x, d, size);
            b = rd(x, &dl, size);
        }
        if (size == 1) {
            dividend = d->op == OP_DIVU ? R(x, R0) : (int16_t)R(x, R0);
        } else {
            uint32_t v = ((uint32_t)R(x, R2) << 16) | R(x, R0);
            dividend = d->op == OP_DIVU ? (int64_t)v : (int64_t)(int32_t)v;
        }
        divisor = d->op == OP_DIVU ? (int64_t)(b & size_mask(size))
                                   : (int64_t)sx(b, size);
        if (divisor == 0) {
            setf(x, FLG_O, true);
            break;
        }
        q = dividend / divisor;
        rem = dividend % divisor;
        if (d->op == OP_DIVX && rem != 0 && ((rem < 0) != (divisor < 0))) {
            q -= 1;
            rem += divisor;
        }
        if (d->op == OP_DIVU) {
            setf(x, FLG_O, (uint64_t)q > size_mask(size));
        } else {
            setf(x, FLG_O, q != sx(q, size));
        }
        if (size == 1) {
            R(x, R0) = (q & 0xff) | ((rem & 0xff) << 8);
        } else {
            R(x, R0) = q;
            R(x, R2) = rem;
        }
        break;
    }
    case OP_DADD: case OP_DADC: case OP_DSUB: case OP_DSBB:
        if (d->form == F_SRC5_DST5) {
            sl = src_loc(x, d, size);
            b = rd(x, &sl, size);
        } else {
            b = d->imm;
        }
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size);
        if (d->op == OP_DADD || d->op == OP_DADC) {
            r = bcd_add(x, size, a, b,
                        d->op == OP_DADC ? !!(env->flg & FLG_C) : 0);
        } else {
            r = bcd_sub(x, size, a, b,
                        d->op == OP_DSBB ? !(env->flg & FLG_C) : 0);
        }
        wr(x, &dl, size, r);
        break;

    /* ---- shifts / rotates ---- */
    case OP_SHL: case OP_SHA: case OP_ROT: {
        int n;
        if (d->form == F_SH4_DST5) {
            n = (d->b2 & 7) + 1;
            if (d->b2 & 8) {
                n = -n;
            }
        } else if (d->form == F_R1H_DST5) {
            n = (int8_t)(R(x, R1) >> 8);
        } else {
            n = (int8_t)d->imm;
        }
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size);
        if (n == 0) {
            break;
        }
        if (d->op == OP_ROT) {
            r = do_rot(x, size, a, n);
        } else {
            r = do_shift(x, size, a, n, d->op == OP_SHA);
        }
        wr(x, &dl, size, r);
        break;
    }
    case OP_ROLC: case OP_RORC: {
        bool c = env->flg & FLG_C;
        dl = dst_loc(x, d, size);
        a = rd(x, &dl, size) & size_mask(size);
        if (d->op == OP_ROLC) {
            setf(x, FLG_C, a & sign_bit(size));
            r = ((a << 1) | c) & size_mask(size);
        } else {
            setf(x, FLG_C, a & 1);
            r = (a >> 1) | (c ? sign_bit(size) : 0);
        }
        wr(x, &dl, size, r);
        set_sz(x, size, r);
        break;
    }

    /* ---- bit operations ---- */
    case OP_BTST: case OP_BNTST: case OP_BSET: case OP_BCLR: case OP_BNOT:
    case OP_BTSTC: case OP_BTSTS: case OP_BAND: case OP_BNAND: case OP_BOR:
    case OP_BNOR: case OP_BXOR: case OP_BNXOR: case OP_BM_CND: {
        int bit;
        bool v, c = env->flg & FLG_C;
        if (d->form == F_BMC) {
            /* BMcnd C */
            setf(x, FLG_C, m32c_cond(env->flg,
                                     ((d->b2 >> 6) & 1) * 8 + (d->b2 & 7)));
            break;
        }
        if (d->form == F_BTST_S) {
            bit = (((d->b1 >> 4) & 3) << 1) | (d->b1 & 1);
            dl = (Loc) { .kind = LOC_MEM, .addr = d->dst.disp };
        } else {
            dl = bit_loc(x, d, &bit);
        }
        a = rd(x, &dl, 1);
        v = (a >> bit) & 1;
        switch (d->op) {
        case OP_BTST:
            setf(x, FLG_C, v);
            setf(x, FLG_Z, !v);
            break;
        case OP_BNTST:
            setf(x, FLG_C, !v);
            setf(x, FLG_Z, !v);
            break;
        case OP_BTSTC: case OP_BTSTS:
            setf(x, FLG_C, v);
            setf(x, FLG_Z, !v);
            /* fall through */
        case OP_BSET: case OP_BCLR: case OP_BNOT: case OP_BM_CND: {
            bool nv;
            switch (d->op) {
            case OP_BSET: case OP_BTSTS: nv = true; break;
            case OP_BCLR: case OP_BTSTC: nv = false; break;
            case OP_BNOT: nv = !v; break;
            default: nv = m32c_cond(env->flg, d->extra & 0xf); break;
            }
            r = (a & ~(1u << bit)) | ((uint32_t)nv << bit);
            wr(x, &dl, 1, r);
            break;
        }
        case OP_BAND:  setf(x, FLG_C, c && v); break;
        case OP_BNAND: setf(x, FLG_C, c && !v); break;
        case OP_BOR:   setf(x, FLG_C, c || v); break;
        case OP_BNOR:  setf(x, FLG_C, c || !v); break;
        case OP_BXOR:  setf(x, FLG_C, c ^ v); break;
        case OP_BNXOR: setf(x, FLG_C, c ^ !v); break;
        }
        break;
    }
    case OP_FSET: case OP_FCLR:
        setf(x, 1u << (d->b2 & 7), d->op == OP_FSET);
        break;
    case OP_SC_CND:
        dl = dst_loc(x, d, 2);
        wr(x, &dl, 2, m32c_cond(env->flg, d->b2 & 0xf));
        break;

    /* ---- control flow ---- */
    case OP_JMP:
        env->pc = d->target;
        break;
    case OP_J_CND:
        if (m32c_cond(env->flg, ((d->b1 >> 4) & 7) * 2 + (d->b1 & 1))) {
            env->pc = d->target;
        }
        break;
    case OP_JSR:
        push(x, 4, next);
        env->pc = d->target;
        break;
    case OP_JMPI: case OP_JSRI:
        dl = dst_loc(x, d, size);
        r = rd(x, &dl, size);
        if (d->op == OP_JSRI) {
            push(x, 4, next);
        }
        env->pc = (size == 2 ? d->base + sx(r, 2) : r) & M32C_ADDR_MASK;
        break;
    case OP_JMPS: case OP_JSRS:
        if (d->op == OP_JSRS) {
            push(x, 4, next);
        }
        r = ld(x, 0xfffe - (d->imm & 0xff) * 2, 2);
        env->pc = 0xff0000 | r;
        break;
    case OP_RTS:
        env->pc = pop(x, 4) & M32C_ADDR_MASK;
        break;
    case OP_REIT:
        env->pc = pop(x, 4) & M32C_ADDR_MASK;
        env->flg = pop(x, 2);
        break;
    case OP_FREIT:
        env->flg = env->svf;
        env->pc = env->svp;
        break;
    case OP_ENTER:
        push(x, 4, FB(x));
        FB(x) = m32c_get_sp(env);
        m32c_set_sp(env, m32c_get_sp(env) - (d->imm & 0xff));
        break;
    case OP_EXITD:
        m32c_set_sp(env, FB(x));
        FB(x) = pop(x, 4) & M32C_ADDR_MASK;
        env->pc = pop(x, 4) & M32C_ADDR_MASK;
        break;
    case OP_ADJNZ:
        dl = dst_loc(x, d, size);
        r = (rd(x, &dl, size) + sx(d->b2 & 0xf, 1)) & size_mask(size);
        wr(x, &dl, size, r);
        if (r) {
            env->pc = d->target;
        }
        break;
    case OP_INT:
        sw_int(x, next, d->extra);
        break;
    case OP_INTO:
        if (env->flg & FLG_O) {
            enter_int(x, next, M32C_VEC_INTO, true, -1);
        }
        break;
    case OP_BRK: case OP_BRK2:
        enter_int(x, next, M32C_VEC_BRK, true, -1);
        break;
    case OP_WAIT:
        env->pc = next;
        {
            CPUState *cs = env_cpu(env);
            cs->halted = 1;
            cs->exception_index = EXCP_HLT;
            cpu_loop_exit(cs);
        }
        break;

    /* ---- stack ---- */
    case OP_PUSH:
        if (d->form == F_IMM) {
            push(x, size, d->imm);
        } else {
            dl = dst_loc(x, d, size);
            push(x, size, rd(x, &dl, size));
        }
        break;
    case OP_POP:
        r = pop(x, size);
        dl = dst_loc(x, d, size);
        wr(x, &dl, size, r);
        break;
    case OP_PUSHA:
        dl = dst_loc(x, d, 3);
        push(x, 4, dl.addr);
        break;
    case OP_PUSHM:
        /* bit7 R0 .. bit3 A0, bit2 A1, bit1 SB, bit0 FB; FB pushed first */
        if (d->extra & 0x01) push(x, 4, FB(x));
        if (d->extra & 0x02) push(x, 4, SB(x));
        if (d->extra & 0x04) push(x, 4, A(x, 1));
        if (d->extra & 0x08) push(x, 4, A(x, 0));
        if (d->extra & 0x10) push(x, 2, R(x, R3));
        if (d->extra & 0x20) push(x, 2, R(x, R2));
        if (d->extra & 0x40) push(x, 2, R(x, R1));
        if (d->extra & 0x80) push(x, 2, R(x, R0));
        break;
    case OP_POPM:
        /* bit0 R0 .. bit3 R3, bit4 A0, bit5 A1, bit6 SB, bit7 FB */
        if (d->extra & 0x01) R(x, R0) = pop(x, 2);
        if (d->extra & 0x02) R(x, R1) = pop(x, 2);
        if (d->extra & 0x04) R(x, R2) = pop(x, 2);
        if (d->extra & 0x08) R(x, R3) = pop(x, 2);
        if (d->extra & 0x10) A(x, 0) = pop(x, 4) & M32C_ADDR_MASK;
        if (d->extra & 0x20) A(x, 1) = pop(x, 4) & M32C_ADDR_MASK;
        if (d->extra & 0x40) SB(x) = pop(x, 4) & M32C_ADDR_MASK;
        if (d->extra & 0x80) FB(x) = pop(x, 4) & M32C_ADDR_MASK;
        break;
    case OP_PUSHC:
        if (d->form == F_CREG16) {
            push(x, 2, *creg16(env, d->b2 & 7));
        } else {
            push(x, 4, get_creg24(x, d->b2 & 7));
        }
        break;
    case OP_POPC:
        if (d->form == F_CREG16) {
            *creg16(env, d->b2 & 7) = pop(x, 2) & 0xffff;
        } else {
            set_creg24(x, d->b2 & 7, pop(x, 4));
        }
        break;

    /* ---- control registers ---- */
    case OP_LDC:
        switch (d->form) {
        case F_IMM_CREG16:
            *creg16(env, d->b2 & 7) = d->imm & 0xffff;
            break;
        case F_IMM_CREG24:
            set_creg24(x, d->b2 & 7, d->imm);
            break;
        case F_IMM_DREG24:
            *dreg24(env, d->b2 & 7) = d->imm & M32C_ADDR_MASK;
            break;
        case F_DST5_CREG16:
            dl = dst_loc(x, d, 2);
            *creg16(env, d->b2 & 7) = rd(x, &dl, 2);
            break;
        case F_DST5_CREG24:
            dl = dst_loc(x, d, 4);
            set_creg24(x, d->b2 & 7, rd(x, &dl, 4));
            break;
        case F_DST5_DREG24:
            dl = dst_loc(x, d, 4);
            *dreg24(env, d->b2 & 7) = rd(x, &dl, 4) & M32C_ADDR_MASK;
            break;
        default:
            undefined(x, d);
        }
        break;
    case OP_STC:
        switch (d->form) {
        case F_CREG16_DST5:
            dl = dst_loc(x, d, 2);
            wr(x, &dl, 2, *creg16(env, d->b2 & 7));
            break;
        case F_CREG24_DST5:
            dl = dst_loc(x, d, 4);
            wr(x, &dl, 4, get_creg24(x, d->b2 & 7));
            break;
        case F_DREG24_DST5:
            dl = dst_loc(x, d, 4);
            wr(x, &dl, 4, *dreg24(env, d->b2 & 7));
            break;
        default:
            undefined(x, d);
        }
        break;
    case OP_LDIPL:
        env->flg = (env->flg & ~FLG_IPL_MASK) |
                   ((d->b2 & 7) << FLG_IPL_SHIFT);
        break;
    case OP_LDCTX: case OP_STCTX: {
        /* register save/restore by task table (OS support) */
        uint32_t task = ld(x, d->imm, 1);
        uint32_t ent = d->imm2 + task * 2;
        uint8_t regs = ld(x, ent, 1);
        uint32_t sp = m32c_get_sp(env);
        static const int sizes[8] = { 2, 2, 2, 2, 4, 4, 4, 4 };
        if (d->op == OP_STCTX) {
            uint8_t spc = ld(x, ent + 1, 1);
            sp -= spc;
            m32c_set_sp(env, sp);
        }
        for (int i = 0; i < 8; i++) {
            uint32_t *p32 = NULL;
            uint16_t *p16 = NULL;
            if (!(regs & (1 << i))) {
                continue;
            }
            switch (i) {
            case 0: case 1: case 2: case 3: p16 = &R(x, i); break;
            case 4: p32 = &A(x, 0); break;
            case 5: p32 = &A(x, 1); break;
            case 6: p32 = &SB(x); break;
            default: p32 = &FB(x); break;
            }
            if (d->op == OP_LDCTX) {
                if (p16) {
                    *p16 = ld(x, sp, 2);
                } else {
                    *p32 = ld(x, sp, 4) & M32C_ADDR_MASK;
                }
            } else {
                st(x, sp, sizes[i], p16 ? *p16 : *p32);
            }
            sp += sizes[i];
        }
        if (d->op == OP_LDCTX) {
            m32c_set_sp(env, sp);
        }
        break;
    }

    /* ---- string instructions ---- */
    case OP_SMOVF: case OP_SMOVB: case OP_SIN: case OP_SOUT: case OP_SSTR: {
        int step = d->op == OP_SMOVB ? -size : size;
        while (R(x, R3)) {
            switch (d->op) {
            case OP_SSTR:
                st(x, A(x, 1), size, size == 1 ? R(x, R0) & 0xff : R(x, R0));
                A(x, 1) = (A(x, 1) + size) & M32C_ADDR_MASK;
                break;
            case OP_SIN:
                st(x, A(x, 1), size, ld(x, A(x, 0), size));
                A(x, 1) = (A(x, 1) + size) & M32C_ADDR_MASK;
                break;
            case OP_SOUT:
                st(x, A(x, 1), size, ld(x, A(x, 0), size));
                A(x, 0) = (A(x, 0) + size) & M32C_ADDR_MASK;
                break;
            default:
                st(x, A(x, 1), size, ld(x, A(x, 0), size));
                A(x, 0) = (A(x, 0) + step) & M32C_ADDR_MASK;
                A(x, 1) = (A(x, 1) + step) & M32C_ADDR_MASK;
                break;
            }
            R(x, R3)--;
        }
        break;
    }
    case OP_SMOVU:
        do {
            a = ld(x, A(x, 0), size);
            st(x, A(x, 1), size, a);
            A(x, 0) = (A(x, 0) + size) & M32C_ADDR_MASK;
            A(x, 1) = (A(x, 1) + size) & M32C_ADDR_MASK;
        } while ((size == 1 ? a : (a & 0xff) && (a & 0xff00)) != 0);
        break;
    case OP_SCMPU:
        do {
            a = ld(x, A(x, 0), size);
            b = ld(x, A(x, 1), size);
            do_sub(x, size, a, b, 0);
            A(x, 0) = (A(x, 0) + size) & M32C_ADDR_MASK;
            A(x, 1) = (A(x, 1) + size) & M32C_ADDR_MASK;
        } while (a == b && (size == 1 ? a : (a & 0xff) && (a & 0xff00)));
        break;
    case OP_RMPA: {
        int64_t acc;
        if (size == 1) {
            acc = (int16_t)R(x, R0);
        } else {
            acc = (int64_t)(((uint64_t)R(x, R1) << 32) |
                            ((uint64_t)R(x, R2) << 16) | R(x, R0));
            acc = (acc << 16) >> 16;
        }
        while (R(x, R3)) {
            int64_t p = (int64_t)sx(ld(x, A(x, 0), size), size) *
                        sx(ld(x, A(x, 1), size), size);
            acc += p;
            A(x, 0) = (A(x, 0) + size) & M32C_ADDR_MASK;
            A(x, 1) = (A(x, 1) + size) & M32C_ADDR_MASK;
            R(x, R3)--;
        }
        if (size == 1) {
            R(x, R0) = acc;
        } else {
            R(x, R0) = acc;
            R(x, R2) = acc >> 16;
            R(x, R1) = acc >> 32;
        }
        break;
    }

    default:
        undefined(x, d);
        break;
    }
done:
    return;
}

static bool is_index(int op)
{
    switch (op) {
    case OP_INDEXB: case OP_INDEXBD: case OP_INDEXBS: case OP_INDEXL:
    case OP_INDEXLD: case OP_INDEXLS: case OP_INDEXW: case OP_INDEXWD:
    case OP_INDEXWS: case OP_BITINDEX:
        return true;
    default:
        return false;
    }
}

static void exec_index(ExecCtx *x, const M32CInsn *d)
{
    Loc l = dst_loc(x, d, d->size);
    uint32_t v = rd(x, &l, d->size);
    int mul = 1;

    switch (d->op) {
    case OP_INDEXW: case OP_INDEXWD: case OP_INDEXWS:
        mul = 2;
        break;
    case OP_INDEXL: case OP_INDEXLD: case OP_INDEXLS:
        mul = 4;
        break;
    }
    v *= mul;
    switch (d->op) {
    case OP_BITINDEX:
        x->bit_index = true;
        x->bit_index_val = v / mul;
        break;
    case OP_INDEXB: case OP_INDEXW: case OP_INDEXL:
        x->src_index = x->dst_index = v;
        break;
    case OP_INDEXBD: case OP_INDEXWD: case OP_INDEXLD:
        x->dst_index = v;
        break;
    default:
        x->src_index = v;
        break;
    }
}

void helper_exec(CPUM32CState *env, uint32_t pc)
{
    ExecCtx x = { .env = env, .ra = GETPC() };
    M32CInsn d;

    env->pc = pc;
    m32c_decode(&d, pc, fetch_code, env);
    if (is_index(d.op)) {
        /* the prefix applies to (and is atomic with) the next insn */
        exec_index(&x, &d);
        m32c_decode(&d, d.next, fetch_code, env);
    }
    if (d.op == OP_INVALID) {
        env->pc = d.next;
        undefined(&x, &d);
        return;
    }
    exec_insn(&x, &d);
}
