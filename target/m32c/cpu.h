/*
 * Renesas M32C/80 (M16C/80 instruction set) CPU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef M32C_CPU_H
#define M32C_CPU_H

#include "cpu-qom.h"
#include "exec/cpu-common.h"
#include "exec/cpu-interrupt.h"

#define M32C_ADDR_MASK  0x00ffffff

/* FLG bits */
#define FLG_C   0x0001
#define FLG_D   0x0002
#define FLG_Z   0x0004
#define FLG_S   0x0008
#define FLG_B   0x0010
#define FLG_O   0x0020
#define FLG_I   0x0040
#define FLG_U   0x0080
#define FLG_IPL_SHIFT 12
#define FLG_IPL_MASK  (7 << FLG_IPL_SHIFT)

/* Fixed vector table */
#define M32C_VEC_UND    0xffffdc
#define M32C_VEC_INTO   0xffffe0
#define M32C_VEC_BRK    0xffffe4
#define M32C_VEC_AMATCH 0xffffe8
#define M32C_VEC_WDT    0xfffff0
#define M32C_VEC_DBC    0xfffff4
#define M32C_VEC_NMI    0xfffff8
#define M32C_VEC_RESET  0xfffffc

/*
 * Interrupt controller interface.  The SoC returns the software interrupt
 * number (variable vector table index) of its highest priority request
 * and its priority level 1..7, or a negative value if nothing is pending.
 * A level of 8 means non-maskable; @fixed_vec then holds the address of
 * the fixed vector (NMI, watchdog).  @fast requests the high-speed
 * interrupt (SVF/SVP/VCT).
 */
typedef struct M32CIrqRequest {
    int vec;
    int level;
    uint32_t fixed_vec;
    bool fast;
} M32CIrqRequest;

typedef bool (*M32CIrqQueryFn)(void *opaque, M32CIrqRequest *req);
typedef void (*M32CIrqAckFn)(void *opaque, const M32CIrqRequest *req);

typedef struct CPUArchState {
    uint16_t r[2][4];       /* R0..R3, two banks */
    uint32_t a[2][2];       /* A0, A1 */
    uint32_t fb[2];
    uint32_t sb[2];
    uint32_t pc;
    uint32_t usp;
    uint32_t isp;
    uint32_t intb;
    uint32_t flg;
    uint32_t svf;
    uint32_t svp;
    uint32_t vct;
    uint32_t dmd[2], dct[2], drc[2], dma[2], dsa[2], dra[2];

    /* Fields up to this point are cleared by a CPU reset */
    struct {} end_reset_fields;

    M32CIrqQueryFn irq_query;
    M32CIrqAckFn irq_ack;
    void *irq_opaque;
} CPUM32CState;

struct ArchCPU {
    CPUState parent_obj;

    CPUM32CState env;
};

struct M32CCPUClass {
    CPUClass parent_class;

    DeviceRealize parent_realize;
    ResettablePhases parent_phases;
};

#define CPU_RESOLVING_TYPE TYPE_M32C_CPU

void m32c_translate_init(void);
void m32c_translate_code(CPUState *cs, TranslationBlock *tb,
                         int *max_insns, vaddr pc, void *host_pc);
void m32c_cpu_dump_state(CPUState *cs, FILE *f, int flags);
int m32c_cpu_gdb_read_register(CPUState *cs, GByteArray *buf, int reg);
int m32c_cpu_gdb_write_register(CPUState *cs, uint8_t *buf, int reg);

#ifndef CONFIG_USER_ONLY
void m32c_cpu_do_interrupt(CPUState *cs);
bool m32c_cpu_exec_interrupt(CPUState *cs, int interrupt_request);
hwaddr m32c_cpu_get_phys_page_debug(CPUState *cs, vaddr addr);
bool m32c_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr);
#endif

/* Set PC and ISP as after a hardware reset (reset vector at 0xfffffc) */
void m32c_cpu_load_reset_vector(M32CCPU *cpu);

static inline int m32c_bank(CPUM32CState *env)
{
    return (env->flg & FLG_B) ? 1 : 0;
}

static inline uint32_t m32c_get_sp(CPUM32CState *env)
{
    return (env->flg & FLG_U) ? env->usp : env->isp;
}

static inline void m32c_set_sp(CPUM32CState *env, uint32_t v)
{
    if (env->flg & FLG_U) {
        env->usp = v & M32C_ADDR_MASK;
    } else {
        env->isp = v & M32C_ADDR_MASK;
    }
}

#endif
