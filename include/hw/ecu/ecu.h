/*
 * ECU emulation: shared I/O layer between MCU peripheral models and the
 * engine/vehicle simulator.
 *
 * MCU models expose their physical pins by name ("TI10", "TO8A", "PA3",
 * "AN0", "IRQ0" ...).  The engine simulator binds its signals (crank, cam,
 * map, maf, inj1, ign1 ...) to those names, so the firmware sees the same
 * electrical signals it would on the real board.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ECU_ECU_H
#define HW_ECU_ECU_H

typedef struct EcuPin EcuPin;

/* Called on the MCU side when an external source changes an input pin. */
typedef void (*EcuPinInputFn)(void *opaque, int level);
/* Called on the simulator side when the MCU drives an output pin. */
typedef void (*EcuPinWatchFn)(void *opaque, EcuPin *pin, int level);

/* Look up (creating on first use) a pin by case-insensitive name. */
EcuPin *ecu_pin(const char *name);
const char *ecu_pin_name(EcuPin *pin);
int ecu_pin_level(EcuPin *pin);

/* MCU side */
void ecu_pin_set_input_handler(EcuPin *pin, EcuPinInputFn fn, void *opaque);
void ecu_pin_mcu_drive(EcuPin *pin, int level);
/*
 * Same, for an edge that happened at virtual time @when_ns (peripheral
 * models processing events late can report the exact edge time).
 */
void ecu_pin_mcu_drive_at(EcuPin *pin, int level, int64_t when_ns);
/* virtual time (ns) of the last level change of @pin */
int64_t ecu_pin_change_ns(EcuPin *pin);

/* Simulator side */
void ecu_pin_external_drive(EcuPin *pin, int level);
void ecu_pin_add_watch(EcuPin *pin, EcuPinWatchFn fn, void *opaque);
void ecu_pin_remove_watch(EcuPin *pin, EcuPinWatchFn fn, void *opaque);

/* Iterate over every known pin. */
void ecu_pin_foreach(void (*fn)(EcuPin *pin, void *opaque), void *opaque);

/*
 * Analog inputs.  ADC models call ecu_analog_read("AN<n>") and get the
 * pin voltage in volts.  The engine simulator installs the provider.
 */
typedef double (*EcuAnalogFn)(void *opaque, const char *channel);
void ecu_analog_set_provider(EcuAnalogFn fn, void *opaque);
double ecu_analog_read(const char *channel);

/*
 * Convert a voltage into an ADC result.
 */
static inline unsigned ecu_adc_convert(double volts, double vref, int bits)
{
    double max = (double)((1u << bits) - 1);
    double v = volts / vref * max;

    if (v < 0) {
        v = 0;
    } else if (v > max) {
        v = max;
    }
    return (unsigned)(v + 0.5);
}

/*
 * Text command interface ("set rpm 3000", "bind map AN0", "status" ...)
 * shared by the HMP "ecu" command and the simulator's chardev.  The
 * output callback is invoked for each line of output.
 */
typedef void (*EcuOutputFn)(void *opaque, const char *line);
typedef void (*EcuCommandFn)(void *handler_opaque, const char *cmd,
                             EcuOutputFn out, void *out_opaque);
void ecu_set_command_handler(EcuCommandFn fn, void *opaque);
bool ecu_run_command(const char *cmd, EcuOutputFn out, void *out_opaque);

#define TYPE_ECU_ENGINE "ecu-engine"

#endif
