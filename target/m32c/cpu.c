/*
 * Renesas M32C/80 CPU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/qemu-print.h"
#include "qapi/error.h"
#include "cpu.h"
#include "migration/vmstate.h"
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/translation-block.h"
#include "exec/target_page.h"
#include "hw/core/loader.h"
#include "tcg/debug-assert.h"
#include "accel/tcg/cpu-ops.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "gdbstub/helpers.h"

static void m32c_cpu_set_pc(CPUState *cs, vaddr value)
{
    cpu_env(cs)->pc = value & M32C_ADDR_MASK;
}

static vaddr m32c_cpu_get_pc(CPUState *cs)
{
    return cpu_env(cs)->pc;
}

static TCGTBCPUState m32c_get_tb_cpu_state(CPUState *cs)
{
    return (TCGTBCPUState){ .pc = cpu_env(cs)->pc, .flags = 0 };
}

static void m32c_cpu_synchronize_from_tb(CPUState *cs,
                                         const TranslationBlock *tb)
{
    tcg_debug_assert(!tcg_cflags_has(cs, CF_PCREL));
    cpu_env(cs)->pc = tb->pc;
}

static void m32c_restore_state_to_opc(CPUState *cs,
                                      const TranslationBlock *tb,
                                      const uint64_t *data)
{
    cpu_env(cs)->pc = data[0];
}

static bool m32c_cpu_has_work(CPUState *cs)
{
    return cpu_test_interrupt(cs, CPU_INTERRUPT_HARD);
}

static int m32c_cpu_mmu_index(CPUState *cs, bool ifetch)
{
    return 0;
}

void m32c_cpu_load_reset_vector(M32CCPU *cpu)
{
    CPUM32CState *env = &cpu->env;
    uint8_t *vec = rom_ptr_for_as(&address_space_memory, M32C_VEC_RESET, 3);

    if (vec) {
        env->pc = vec[0] | (vec[1] << 8) | (vec[2] << 16);
    } else {
        env->pc = address_space_lduw_le(&address_space_memory,
                                        M32C_VEC_RESET,
                                        MEMTXATTRS_UNSPECIFIED, NULL) |
                  (address_space_ldub(&address_space_memory,
                                      M32C_VEC_RESET + 2,
                                      MEMTXATTRS_UNSPECIFIED, NULL) << 16);
    }
}

static void m32c_cpu_reset_hold(Object *obj, ResetType type)
{
    CPUState *cs = CPU(obj);
    M32CCPUClass *mcc = M32C_CPU_GET_CLASS(obj);
    CPUM32CState *env = cpu_env(cs);

    if (mcc->parent_phases.hold) {
        mcc->parent_phases.hold(obj, type);
    }
    memset(env, 0, offsetof(CPUM32CState, end_reset_fields));
    env->flg = 0;
}

static ObjectClass *m32c_cpu_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;
    g_autofree char *typename = NULL;

    oc = object_class_by_name(cpu_model);
    if (oc && object_class_dynamic_cast(oc, TYPE_M32C_CPU)) {
        return oc;
    }
    typename = g_strdup_printf(M32C_CPU_TYPE_NAME("%s"), cpu_model);
    return object_class_by_name(typename);
}

static void m32c_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    M32CCPUClass *mcc = M32C_CPU_GET_CLASS(dev);
    Error *local_err = NULL;

    cpu_common_realize(cs, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    qemu_init_vcpu(cs);
    cpu_reset(cs);
    mcc->parent_realize(dev, errp);
}

bool m32c_cpu_tlb_fill(CPUState *cs, vaddr addr, int size,
                       MMUAccessType access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    uint32_t page = addr & TARGET_PAGE_MASK & M32C_ADDR_MASK;

    tlb_set_page(cs, addr & TARGET_PAGE_MASK, page,
                 PAGE_READ | PAGE_WRITE | PAGE_EXEC, mmu_idx,
                 TARGET_PAGE_SIZE);
    return true;
}

hwaddr m32c_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    return addr & M32C_ADDR_MASK;
}

void m32c_cpu_dump_state(CPUState *cs, FILE *f, int flags)
{
    CPUM32CState *env = cpu_env(cs);
    int b = m32c_bank(env);

    qemu_fprintf(f, "pc=%06x flg=%04x [%c%c%c%c%c%c%c%c ipl=%d] bank=%d\n",
                 env->pc, env->flg,
                 env->flg & FLG_U ? 'U' : '-', env->flg & FLG_I ? 'I' : '-',
                 env->flg & FLG_O ? 'O' : '-', env->flg & FLG_B ? 'B' : '-',
                 env->flg & FLG_S ? 'S' : '-', env->flg & FLG_Z ? 'Z' : '-',
                 env->flg & FLG_D ? 'D' : '-', env->flg & FLG_C ? 'C' : '-',
                 (env->flg & FLG_IPL_MASK) >> FLG_IPL_SHIFT, b);
    qemu_fprintf(f, "r0=%04x r1=%04x r2=%04x r3=%04x a0=%06x a1=%06x\n",
                 env->r[b][0], env->r[b][1], env->r[b][2], env->r[b][3],
                 env->a[b][0], env->a[b][1]);
    qemu_fprintf(f, "fb=%06x sb=%06x usp=%06x isp=%06x intb=%06x\n",
                 env->fb[b], env->sb[b], env->usp, env->isp, env->intb);
}

/*
 * gdb raw register layout of GDB's m32c-tdep.c (make_regs) for
 * bfd_mach_m32c, in 'g' packet order, little endian.  24-bit registers
 * are 3 bytes wide (24-bit pointer types):
 *
 *   0-7   r0 r1 r2 r3, bank 0 and bank 1 of each (16 bit)
 *   8-15  a0 a1 fb sb, bank 0 and bank 1 of each (24 bit)
 *   16-19 usp isp intb pc (24 bit)
 *   20    flg (16 bit)
 *   21-23 svf (16 bit), svp vct (24 bit)
 *   24-35 dmd0 dmd1 (8 bit), dct0 dct1 drc0 drc1 (16 bit),
 *         dma0 dma1 dsa0 dsa1 dra0 dra1 (24 bit)
 */
#define M32C_GDB_NUM_REGS 36

static int m32c_gdb_reg_size(int n)
{
    switch (n) {
    case 0 ... 7: case 20: case 21: case 26 ... 29:
        return 2;
    case 24: case 25:
        return 1;
    default:
        return 3;
    }
}

static uint32_t *m32c_gdb_reg_ptr(CPUM32CState *env, int n)
{
    switch (n) {
    case 8 ... 11:
        return &env->a[(n - 8) & 1][(n - 8) >> 1];
    case 12: case 13: return &env->fb[n - 12];
    case 14: case 15: return &env->sb[n - 14];
    case 16: return &env->usp;
    case 17: return &env->isp;
    case 18: return &env->intb;
    case 19: return &env->pc;
    case 20: return &env->flg;
    case 21: return &env->svf;
    case 22: return &env->svp;
    case 23: return &env->vct;
    case 24: case 25: return &env->dmd[n - 24];
    case 26: case 27: return &env->dct[n - 26];
    case 28: case 29: return &env->drc[n - 28];
    case 30: case 31: return &env->dma[n - 30];
    case 32: case 33: return &env->dsa[n - 32];
    case 34: case 35: return &env->dra[n - 34];
    }
    g_assert_not_reached();
}

int m32c_cpu_gdb_read_register(CPUState *cs, GByteArray *buf, int n)
{
    CPUM32CState *env = cpu_env(cs);
    uint32_t v;
    uint8_t b[4];
    int size;

    if (n < 0 || n >= M32C_GDB_NUM_REGS) {
        return 0;
    }
    if (n < 8) {
        v = env->r[n & 1][n >> 1];
    } else {
        v = *m32c_gdb_reg_ptr(env, n);
    }
    size = m32c_gdb_reg_size(n);
    stl_le_p(b, v);
    g_byte_array_append(buf, b, size);
    return size;
}

int m32c_cpu_gdb_write_register(CPUState *cs, uint8_t *buf, int n)
{
    CPUM32CState *env = cpu_env(cs);
    int size;
    uint32_t v;

    if (n < 0 || n >= M32C_GDB_NUM_REGS) {
        return 0;
    }
    size = m32c_gdb_reg_size(n);
    v = size == 1 ? ldub_p(buf) : size == 2 ? lduw_le_p(buf)
                                            : lduw_le_p(buf) | (buf[2] << 16);
    if (n < 8) {
        env->r[n & 1][n >> 1] = v;
    } else if (n == 20) {
        env->flg = v & FLG_MASK;
    } else {
        *m32c_gdb_reg_ptr(env, n) = v;
    }
    return size;
}

static const VMStateDescription vmstate_m32c_cpu = {
    .name = "cpu",
    .unmigratable = 1,
};

#include "hw/core/sysemu-cpu-ops.h"

static const struct SysemuCPUOps m32c_sysemu_ops = {
    .has_work = m32c_cpu_has_work,
    .get_phys_addr_debug = m32c_cpu_get_phys_page_debug,
    .legacy_vmsd = &vmstate_m32c_cpu,
};

static const TCGCPUOps m32c_tcg_ops = {
    .guest_default_memory_order = TCG_MO_ALL,
    .mttcg_supported = false,

    .initialize = m32c_translate_init,
    .translate_code = m32c_translate_code,
    .get_tb_cpu_state = m32c_get_tb_cpu_state,
    .synchronize_from_tb = m32c_cpu_synchronize_from_tb,
    .restore_state_to_opc = m32c_restore_state_to_opc,
    .mmu_index = m32c_cpu_mmu_index,
    .tlb_fill = m32c_cpu_tlb_fill,
    .pointer_wrap = cpu_pointer_wrap_uint32,

    .cpu_exec_interrupt = m32c_cpu_exec_interrupt,
    .cpu_exec_halt = m32c_cpu_has_work,
    .cpu_exec_reset = cpu_reset,
    .do_interrupt = m32c_cpu_do_interrupt,
};

static void m32c_cpu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    CPUClass *cc = CPU_CLASS(klass);
    M32CCPUClass *mcc = M32C_CPU_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_parent_realize(dc, m32c_cpu_realize,
                                    &mcc->parent_realize);
    resettable_class_set_parent_phases(rc, NULL, m32c_cpu_reset_hold, NULL,
                                       &mcc->parent_phases);

    cc->class_by_name = m32c_cpu_class_by_name;
    cc->dump_state = m32c_cpu_dump_state;
    cc->set_pc = m32c_cpu_set_pc;
    cc->get_pc = m32c_cpu_get_pc;
    cc->sysemu_ops = &m32c_sysemu_ops;
    cc->gdb_read_register = m32c_cpu_gdb_read_register;
    cc->gdb_write_register = m32c_cpu_gdb_write_register;
    cc->gdb_num_core_regs = M32C_GDB_NUM_REGS;
    cc->tcg_ops = &m32c_tcg_ops;
}

static const TypeInfo m32c_cpu_info[] = {
    {
        .name = TYPE_M32C_CPU,
        .parent = TYPE_CPU,
        .instance_size = sizeof(M32CCPU),
        .instance_align = __alignof(M32CCPU),
        .abstract = true,
        .class_size = sizeof(M32CCPUClass),
        .class_init = m32c_cpu_class_init,
    }, {
        .name = TYPE_M32C80_CPU,
        .parent = TYPE_M32C_CPU,
    },
};

DEFINE_TYPES(m32c_cpu_info)
