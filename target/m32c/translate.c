/*
 * Renesas M32C/80 translation
 *
 * Each instruction is executed by helper_exec(); the translator only
 * decodes instructions to know their length and whether they end the
 * translation block (branches, flag/IPL changes, prefixes, string ops).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "tcg/tcg-op.h"
#include "exec/helper-proto.h"
#include "exec/helper-gen.h"
#include "exec/translation-block.h"
#include "exec/translator.h"
#include "exec/target_page.h"
#include "exec/log.h"
#include "decode.h"

#define HELPER_H "helper.h"
#include "exec/helper-info.c.inc"
#undef  HELPER_H

#define M32C_MAX_TB_INSNS 32

#define DISAS_JUMP    DISAS_TARGET_0

typedef struct DisasContext {
    DisasContextBase base;
    CPUM32CState *env;
    uint32_t page;
} DisasContext;

static TCGv_i32 cpu_pc;

void m32c_translate_init(void)
{
    cpu_pc = tcg_global_mem_new_i32(tcg_env, offsetof(CPUM32CState, pc),
                                    "pc");
}

static uint8_t tr_fetch(void *opaque, uint32_t addr)
{
    DisasContext *ctx = opaque;

    return translator_ldub(ctx->env, &ctx->base, addr);
}

static void m32c_tr_init_disas_context(DisasContextBase *dcbase,
                                       CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    ctx->env = cpu_env(cs);
    ctx->page = ctx->base.pc_first & TARGET_PAGE_MASK;
    ctx->base.max_insns = MIN(ctx->base.max_insns, M32C_MAX_TB_INSNS);
}

static void m32c_tr_tb_start(DisasContextBase *dcbase, CPUState *cs)
{
}

static void m32c_tr_insn_start(DisasContextBase *dcbase, CPUState *cs)
{
    tcg_gen_insn_start(dcbase->pc_next, 0, 0);
}

static void m32c_tr_translate_insn(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);
    M32CInsn d;

    m32c_decode(&d, ctx->base.pc_next, tr_fetch, ctx);
    gen_helper_exec(tcg_env, tcg_constant_i32(ctx->base.pc_next));
    ctx->base.pc_next += d.len;
    if (d.ends_tb) {
        ctx->base.is_jmp = DISAS_JUMP;
    } else if ((ctx->base.pc_next & TARGET_PAGE_MASK) != ctx->page ||
               (ctx->base.pc_next & ~TARGET_PAGE_MASK) >
               TARGET_PAGE_SIZE - 16) {
        ctx->base.is_jmp = DISAS_TOO_MANY;
    }
}

static void m32c_tr_tb_stop(DisasContextBase *dcbase, CPUState *cs)
{
    DisasContext *ctx = container_of(dcbase, DisasContext, base);

    switch (ctx->base.is_jmp) {
    case DISAS_NEXT:
    case DISAS_TOO_MANY:
        if (translator_use_goto_tb(&ctx->base, ctx->base.pc_next)) {
            tcg_gen_goto_tb(0);
            tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
            tcg_gen_exit_tb(ctx->base.tb, 0);
        } else {
            tcg_gen_movi_i32(cpu_pc, ctx->base.pc_next);
            tcg_gen_lookup_and_goto_ptr();
        }
        break;
    case DISAS_JUMP:
        /* the helper has set pc; exit so pending interrupts are seen */
        tcg_gen_exit_tb(NULL, 0);
        break;
    case DISAS_NORETURN:
        break;
    default:
        g_assert_not_reached();
    }
}

static const TranslatorOps m32c_tr_ops = {
    .init_disas_context = m32c_tr_init_disas_context,
    .tb_start           = m32c_tr_tb_start,
    .insn_start         = m32c_tr_insn_start,
    .translate_insn     = m32c_tr_translate_insn,
    .tb_stop            = m32c_tr_tb_stop,
};

void m32c_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc)
{
    DisasContext ctx;

    translator_loop(cs, tb, max_insns, pc, host_pc, &m32c_tr_ops,
                    &ctx.base, TCG_TYPE_VA);
}
