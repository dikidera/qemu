/*
 * Minimal test machine for the Motorola CPU32 core.
 *
 * Not a real board: it exists so that CPU32 self-test ROMs
 * (tests/ecu/cpu32_test_rom.py) can run without a full SoC model.
 *
 *   0x00000000  RAM (default 1 MiB), loaded from -bios; the reset vector
 *               (SSP, PC) is read from addresses 0 and 4.
 *   0xFFFFF000  output port: a byte written here is sent to serial 0.
 *   0xFFFFF004  exit port: writing N terminates QEMU with exit status N.
 *   0xFFFFF008  interrupt request port: writing (level << 8) | vector
 *               sets the CPU interrupt request level (0 = none) and the
 *               vector supplied on acknowledge.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "target/m68k/cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "chardev/char-fe.h"
#include "system/system.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/qtest.h"
#include "system/address-spaces.h"

#define CPU32_TEST_IO_BASE  0xfffff000

typedef struct Cpu32TestIO {
    MemoryRegion iomem;
    CharFrontend chr;
    M68kCPU *cpu;
} Cpu32TestIO;

static uint64_t cpu32_test_io_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void cpu32_test_io_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    Cpu32TestIO *s = opaque;
    uint8_t ch;

    switch (addr & ~3) {
    case 0x0:
        ch = val;
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        break;
    case 0x4:
        qemu_system_shutdown_request_with_code(SHUTDOWN_CAUSE_GUEST_SHUTDOWN,
                                               val & 0xff);
        break;
    case 0x8:
        m68k_set_irq_level(s->cpu, (val >> 8) & 7, val & 0xff);
        break;
    }
}

static const MemoryRegionOps cpu32_test_io_ops = {
    .read = cpu32_test_io_read,
    .write = cpu32_test_io_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void cpu32_test_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void cpu32_test_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    Cpu32TestIO *io = g_new0(Cpu32TestIO, 1);
    M68kCPU *cpu;

    cpu = M68K_CPU(cpu_create(machine->cpu_type));
    io->cpu = cpu;
    qemu_register_reset(cpu32_test_reset, cpu);

    memory_region_add_subregion(sysmem, 0, machine->ram);

    memory_region_init_io(&io->iomem, NULL, &cpu32_test_io_ops, io,
                          "cpu32-test.io", 0x10);
    memory_region_add_subregion(sysmem, CPU32_TEST_IO_BASE, &io->iomem);
    qemu_chr_fe_init(&io->chr, serial_hd(0), &error_fatal);

    if (machine->firmware) {
        if (load_image_targphys(machine->firmware, 0, machine->ram_size,
                                NULL) < 0) {
            error_report("could not load ROM image '%s'", machine->firmware);
            exit(1);
        }
    } else if (!qtest_enabled()) {
        error_report("a ROM image must be given with -bios");
        exit(1);
    }
}

static void cpu32_test_machine_init(MachineClass *mc)
{
    mc->desc = "CPU32 core test machine (RAM, output and exit ports)";
    mc->init = cpu32_test_init;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("cpu32");
    mc->default_ram_size = 1 * MiB;
    mc->default_ram_id = "cpu32-test.ram";
}

DEFINE_MACHINE("cpu32-test", cpu32_test_machine_init)
