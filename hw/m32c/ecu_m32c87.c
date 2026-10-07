/*
 * ECU board built around a Renesas M32C/87.
 *
 * The flash image (raw dump, as long as the flash: 512 KiB, 768 KiB or
 * 1 MiB; it is mapped to the top of the 16 MiB address space so the reset
 * vector at 0xfffffc is its last word) is given with -bios.  Override the
 * flash size with -global m32c87-soc.rom-size=0x80000.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/loader.h"
#include "system/system.h"
#include "system/qtest.h"
#include "net/can_emu.h"
#include "hw/m32c/m32c87.h"

#define TYPE_ECU_M32C87_MACHINE MACHINE_TYPE_NAME("ecu-m32c87")
OBJECT_DECLARE_SIMPLE_TYPE(EcuM32C87MachineState, ECU_M32C87_MACHINE)

struct EcuM32C87MachineState {
    MachineState parent_obj;

    M32C87State *soc;
    DeviceState *engine;
    CanBusState *canbus[M32C87_NUM_CAN];
};

/*
 * Starting point for the engine bindings: crank period measured by timer
 * B0 and an INT0 edge interrupt, cam on INT1, injectors on the timer A
 * one-shot outputs, coils on port 3.  Adjust to the ECU schematic.
 */
static const char m32c87_default_bind[] =
    "crank=TB0IN+INT0,cam=INT1,vss=TB1IN,"
    "map=AN0,maf=AN1,clt=AN2,iat=AN3,tps=AN4,o2=AN5,vbat=AN6,knock=AN7,"
    "baro=AN8,app=AN9,app2=AN10,fuel_temp=AN11,o2b=AN12,"
    "inj1=TA1OUT,inj2=TA2OUT,inj3=TA3OUT,inj4=TA4OUT,"
    "ign1=P3_0,ign2=P3_1,ign3=P3_2,ign4=P3_3,"
    "isc=TA0OUT,fpump=P3_4,main_relay=P3_5,cel=P3_6,fan1=P3_7";

static void ecu_m32c87_init(MachineState *machine)
{
    EcuM32C87MachineState *m = ECU_M32C87_MACHINE(machine);
    const char *rom = machine->firmware ? machine->firmware
                                        : machine->kernel_filename;
    DeviceState *dev;

    dev = qdev_new(TYPE_M32C87_SOC);
    m->soc = M32C87_SOC(dev);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    for (int i = 0; i < M32C87_NUM_UART; i++) {
        char name[8];
        snprintf(name, sizeof(name), "uart%d", i);
        qdev_prop_set_chr(dev, name, serial_hd(i));
    }
    for (int i = 0; i < M32C87_NUM_CAN; i++) {
        if (m->canbus[i]) {
            char name[16];
            snprintf(name, sizeof(name), "canbus%d", i);
            object_property_set_link(OBJECT(dev), name,
                                     OBJECT(m->canbus[i]), &error_fatal);
        }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    if (rom) {
        if (load_image_targphys(rom, 0x1000000 - m->soc->rom_size,
                                m->soc->rom_size, NULL) < 0) {
            error_report("could not load ECU flash image '%s'", rom);
            exit(1);
        }
    } else if (!qtest_enabled()) {
        warn_report("no ECU flash image given (use -bios flash.bin)");
    }

    m->engine = qdev_new(TYPE_ECU_ENGINE);
    object_property_add_child(OBJECT(machine), "engine", OBJECT(m->engine));
    qdev_prop_set_string(m->engine, "board-bind", m32c87_default_bind);
    qdev_realize_and_unref(m->engine, NULL, &error_fatal);
}

static void ecu_m32c87_instance_init(Object *obj)
{
    EcuM32C87MachineState *m = ECU_M32C87_MACHINE(obj);

    for (int i = 0; i < M32C87_NUM_CAN; i++) {
        char name[16];
        snprintf(name, sizeof(name), "canbus%d", i);
        object_property_add_link(obj, name, TYPE_CAN_BUS,
                                 (Object **)&m->canbus[i],
                                 object_property_allow_set_link, 0);
    }
}

static void ecu_m32c87_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Engine control unit with a Renesas M32C/87";
    mc->init = ecu_m32c87_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 0;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

static const TypeInfo ecu_m32c87_type = {
    .name = TYPE_ECU_M32C87_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(EcuM32C87MachineState),
    .instance_init = ecu_m32c87_instance_init,
    .class_init = ecu_m32c87_class_init,
};

static void ecu_m32c87_register(void)
{
    type_register_static(&ecu_m32c87_type);
}

type_init(ecu_m32c87_register)
