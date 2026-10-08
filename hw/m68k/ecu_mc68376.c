/*
 * ECU board built around a Motorola MC68376 (CPU32 core, SIM, QSM, QADC,
 * CTM4, TPU, TouCAN).
 *
 * The firmware is a raw image of the external boot flash on CSBOOT,
 * given with -bios (or -kernel).  At reset CSBOOT maps it at $000000
 * (1 MiB block, mirrored if smaller), so the reset vector (SSP, PC) is
 * its first 8 bytes.  Flash size: -global mc68376-soc.flash-size=...
 * (default 512 KiB).  Optional external RAM: -global
 * mc68376-soc.ext-ram-size=... with either ext-ram-cs=N (decoded by
 * chip select N as programmed by the firmware) or ext-ram-base=ADDR
 * (always mapped).
 *
 * The ECU engine simulator drives the MCU pins; its default bindings are
 * placeholders, re-bind them to the ECU's schematic.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "system/system.h"
#include "system/qtest.h"
#include "net/can_emu.h"
#include "hw/m68k/mc68376.h"

#define MC68376_CPU_TYPE M68K_CPU_TYPE_NAME("cpu32")

#define TYPE_ECU_MC68376_MACHINE MACHINE_TYPE_NAME("ecu-mc68376")
OBJECT_DECLARE_SIMPLE_TYPE(EcuMC68376MachineState, ECU_MC68376_MACHINE)

struct EcuMC68376MachineState {
    MachineState parent_obj;

    MC68376State *soc;
    DeviceState *engine;
    CanBusState *canbus;
};

/*
 * Placeholder bindings: crank/cam/vehicle speed on TPU channels 0-2,
 * injectors on TPU4-7, coils on TPU8-11, PWM actuators on the CTM4 PWM
 * channels, sensors on the QADC inputs (AN0-AN3 on port B, AN48-AN59 on
 * ports A/B, Table 5-16).  The TPU, CTM4 and QADC models register these
 * pin names.
 */
static const char mc68376_default_bind[] =
    "crank=TPU0,cam=TPU1,vss=TPU2,"
    "map=AN0,maf=AN1,clt=AN2,iat=AN3,tps=AN48,o2=AN49,vbat=AN50,"
    "knock=AN51,baro=AN52,app=AN53,app2=AN54,fuel_temp=AN55,o2b=AN56,"
    "fuel_press=AN57,oil_temp=AN58,ref5=AN59,"
    "inj1=TPU4,inj2=TPU5,inj3=TPU6,inj4=TPU7,"
    "ign1=TPU8,ign2=TPU9,ign3=TPU10,ign4=TPU11,"
    "isc=CPWM5,boost=CPWM6,vvt=CPWM7,purge=CPWM8";

static void ecu_mc68376_init(MachineState *machine)
{
    EcuMC68376MachineState *m = ECU_MC68376_MACHINE(machine);
    const char *rom = machine->firmware ? machine->firmware
                                        : machine->kernel_filename;
    DeviceState *dev;

    dev = qdev_new(TYPE_MC68376_SOC);
    m->soc = MC68376_SOC(dev);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(dev));
    qdev_prop_set_string(dev, "cpu-type", MC68376_CPU_TYPE);
    qdev_prop_set_chr(dev, "sci", serial_hd(0));
    if (m->canbus) {
        object_property_set_link(OBJECT(dev), "canbus0", OBJECT(m->canbus),
                                 &error_fatal);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    if (rom) {
        /*
         * Copied into the flash array (not a ROM blob), so the image is
         * in place before the SoC reset fetches the reset vector.
         */
        if (!mc68376_flash_load(m->soc, rom)) {
            error_report("could not load ECU flash image '%s'", rom);
            exit(1);
        }
    } else if (!qtest_enabled()) {
        warn_report("no ECU flash image given (use -bios flash.bin)");
    }

    m->engine = qdev_new(TYPE_ECU_ENGINE);
    object_property_add_child(OBJECT(machine), "engine", OBJECT(m->engine));
    qdev_prop_set_string(m->engine, "board-bind", mc68376_default_bind);
    qdev_realize_and_unref(m->engine, NULL, &error_fatal);
}

static void ecu_mc68376_instance_init(Object *obj)
{
    EcuMC68376MachineState *m = ECU_MC68376_MACHINE(obj);

    object_property_add_link(obj, "canbus0", TYPE_CAN_BUS,
                             (Object **)&m->canbus,
                             object_property_allow_set_link, 0);
}

static void ecu_mc68376_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Engine control unit with a Motorola MC68376 (CPU32)";
    mc->init = ecu_mc68376_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 0;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->default_nic = NULL;
}

static const TypeInfo ecu_mc68376_type = {
    .name = TYPE_ECU_MC68376_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(EcuMC68376MachineState),
    .instance_init = ecu_mc68376_instance_init,
    .class_init = ecu_mc68376_class_init,
};

static void ecu_mc68376_register(void)
{
    type_register_static(&ecu_mc68376_type);
}

type_init(ecu_mc68376_register)
