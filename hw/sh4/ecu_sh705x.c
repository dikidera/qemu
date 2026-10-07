/*
 * ECU boards built around Renesas SH705x microcontrollers.
 *
 *   ecu-sh7052  SH-2,  256 KiB flash, 12 KiB RAM, 1 HCAN
 *   ecu-sh7054  SH-2,  384 KiB flash, 16 KiB RAM, 1 HCAN
 *   ecu-sh7055  SH-2E, 512 KiB flash, 32 KiB RAM, 2 HCAN
 *   ecu-sh7058  SH-2E,   1 MiB flash, 48 KiB RAM, 2 HCAN2
 *
 * The ROM image (raw dump of the internal flash) is given with -bios (or
 * -kernel).  The ECU engine simulator drives the MCU pins; its default
 * bindings below are only a starting point, every ECU wires its sensors
 * differently, override them with "-global ecu-engine.bind=..." or the
 * "ecu bind" monitor command.
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
#include "hw/sh4/sh705x.h"

#define TYPE_ECU_SH705X_MACHINE MACHINE_TYPE_NAME("ecu-sh705x")
OBJECT_DECLARE_TYPE(EcuSH705xMachineState, EcuSH705xMachineClass,
                    ECU_SH705X_MACHINE)

struct EcuSH705xMachineState {
    MachineState parent_obj;

    SH705xState *soc;
    DeviceState *engine;
    CanBusState *canbus[SH705X_NUM_HCAN];
};

struct EcuSH705xMachineClass {
    MachineClass parent_class;

    SH705xVariant variant;
    uint32_t pclk_hz;
};

/*
 * Generic starting point for the engine bindings: crank on the channel 10
 * angle input and channel 0 capture A, cam on capture B, injectors and
 * coils on the channel 8 one-shot outputs, sensors on the first ANx inputs.
 */
static const char sh705x_default_bind[] =
    "crank=TI10+TI0A,cam=TI0B,vss=TI9A,"
    "map=AN0,maf=AN1,clt=AN2,iat=AN3,tps=AN4,o2=AN5,vbat=AN6,knock=AN7,"
    "baro=AN8,app=AN9,app2=AN10,fuel_temp=AN11,o2b=AN12,fuel_press=AN13,"
    "oil_temp=AN14,ref5=AN15,"
    "inj1=TO8A,inj2=TO8B,inj3=TO8C,inj4=TO8D,"
    "inj5=TO8E,inj6=TO8F,inj7=TO8G,inj8=TO8H,"
    "ign1=TO8I,ign2=TO8J,ign3=TO8K,ign4=TO8L,"
    "ign5=TO8M,ign6=TO8N,ign7=TO8O,ign8=TO8P,"
    "isc=TO6A,boost=TO6B,vvt=TO6C,purge=TO6D";

static void ecu_sh705x_init(MachineState *machine)
{
    EcuSH705xMachineState *m = ECU_SH705X_MACHINE(machine);
    EcuSH705xMachineClass *mc = ECU_SH705X_MACHINE_GET_CLASS(machine);
    const char *rom = machine->firmware ? machine->firmware
                                        : machine->kernel_filename;
    DeviceState *dev;

    dev = qdev_new(TYPE_SH705X_SOC);
    m->soc = SH705X_SOC(dev);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_prop_set_uint32(dev, "variant", mc->variant);
    qdev_prop_set_uint32(dev, "pclk-hz", mc->pclk_hz);
    for (int i = 0; i < SH705X_NUM_SCI; i++) {
        char name[8];
        snprintf(name, sizeof(name), "sci%d", i);
        qdev_prop_set_chr(dev, name, serial_hd(i));
    }
    for (int i = 0; i < SH705X_NUM_HCAN; i++) {
        if (m->canbus[i]) {
            char name[16];
            snprintf(name, sizeof(name), "canbus%d", i);
            object_property_set_link(OBJECT(dev), name,
                                     OBJECT(m->canbus[i]), &error_fatal);
        }
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    if (rom) {
        if (load_image_targphys(rom, 0, m->soc->rom_size, NULL) < 0) {
            error_report("could not load ECU ROM image '%s'", rom);
            exit(1);
        }
    } else if (!qtest_enabled()) {
        warn_report("no ECU ROM image given (use -bios rom.bin)");
    }

    m->engine = qdev_new(TYPE_ECU_ENGINE);
    object_property_add_child(OBJECT(machine), "engine", OBJECT(m->engine));
    qdev_prop_set_string(m->engine, "board-bind", sh705x_default_bind);
    qdev_realize_and_unref(m->engine, NULL, &error_fatal);
}

static void ecu_sh705x_instance_init(Object *obj)
{
    EcuSH705xMachineState *m = ECU_SH705X_MACHINE(obj);

    for (int i = 0; i < SH705X_NUM_HCAN; i++) {
        char name[16];
        snprintf(name, sizeof(name), "canbus%d", i);
        object_property_add_link(obj, name, TYPE_CAN_BUS,
                                 (Object **)&m->canbus[i],
                                 object_property_allow_set_link, 0);
    }
}

static void ecu_sh705x_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = ecu_sh705x_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 0;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->default_nic = NULL;
}

#define ECU_SH705X_VARIANT(name_, variant_, pclk_, desc_)                  \
    static void ecu_##name_##_class_init(ObjectClass *oc,               \
                                         const void *data)              \
    {                                                                   \
        MachineClass *mc = MACHINE_CLASS(oc);                           \
        EcuSH705xMachineClass *emc = ECU_SH705X_MACHINE_CLASS(oc);      \
                                                                        \
        mc->desc = desc_;                                               \
        emc->variant = variant_;                                        \
        emc->pclk_hz = pclk_;                                           \
    }

ECU_SH705X_VARIANT(sh7052, SH705X_7052, 20000000,
                   "Engine control unit with a Renesas SH7052 (SH-2)")
ECU_SH705X_VARIANT(sh7054, SH705X_7054, 20000000,
                   "Engine control unit with a Renesas SH7054 (SH-2)")
ECU_SH705X_VARIANT(sh7055, SH705X_7055, 20000000,
                   "Engine control unit with a Renesas SH7055 (SH-2E)")
ECU_SH705X_VARIANT(sh7058, SH705X_7058, 20000000,
                   "Engine control unit with a Renesas SH7058 (SH-2E)")

static const TypeInfo ecu_sh705x_types[] = {
    {
        .name = TYPE_ECU_SH705X_MACHINE,
        .parent = TYPE_MACHINE,
        .abstract = true,
        .instance_size = sizeof(EcuSH705xMachineState),
        .instance_init = ecu_sh705x_instance_init,
        .class_size = sizeof(EcuSH705xMachineClass),
        .class_init = ecu_sh705x_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ecu-sh7052"),
        .parent = TYPE_ECU_SH705X_MACHINE,
        .class_init = ecu_sh7052_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ecu-sh7054"),
        .parent = TYPE_ECU_SH705X_MACHINE,
        .class_init = ecu_sh7054_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ecu-sh7055"),
        .parent = TYPE_ECU_SH705X_MACHINE,
        .class_init = ecu_sh7055_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("ecu-sh7058"),
        .parent = TYPE_ECU_SH705X_MACHINE,
        .class_init = ecu_sh7058_class_init,
    },
};

DEFINE_TYPES(ecu_sh705x_types)
