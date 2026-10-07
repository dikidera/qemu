/*
 * ECU emulation: named pin / analog channel registry and the "ecu"
 * monitor command.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "qemu/timer.h"
#include "qobject/qdict.h"
#include "monitor/monitor.h"
#include "monitor/hmp.h"
#include "hw/ecu/ecu.h"
#include "trace.h"

typedef struct EcuPinWatch {
    EcuPinWatchFn fn;
    void *opaque;
    QLIST_ENTRY(EcuPinWatch) next;
} EcuPinWatch;

struct EcuPin {
    char *name;
    int level;
    int64_t change_ns;
    EcuPinInputFn input_fn;
    void *input_opaque;
    QLIST_HEAD(, EcuPinWatch) watches;
};

static GHashTable *ecu_pins;
static GPtrArray *ecu_pin_list;

static EcuAnalogFn analog_fn;
static void *analog_opaque;

static EcuCommandFn command_fn;
static void *command_opaque;

EcuPin *ecu_pin(const char *name)
{
    g_autofree char *key = g_ascii_strup(name, -1);
    EcuPin *pin;

    if (!ecu_pins) {
        ecu_pins = g_hash_table_new(g_str_hash, g_str_equal);
        ecu_pin_list = g_ptr_array_new();
    }
    pin = g_hash_table_lookup(ecu_pins, key);
    if (!pin) {
        pin = g_new0(EcuPin, 1);
        pin->name = g_steal_pointer(&key);
        QLIST_INIT(&pin->watches);
        g_hash_table_insert(ecu_pins, pin->name, pin);
        g_ptr_array_add(ecu_pin_list, pin);
    }
    return pin;
}

const char *ecu_pin_name(EcuPin *pin)
{
    return pin->name;
}

int ecu_pin_level(EcuPin *pin)
{
    return pin->level;
}

void ecu_pin_set_input_handler(EcuPin *pin, EcuPinInputFn fn, void *opaque)
{
    pin->input_fn = fn;
    pin->input_opaque = opaque;
}

static void ecu_pin_notify(EcuPin *pin)
{
    EcuPinWatch *w, *next;

    QLIST_FOREACH_SAFE(w, &pin->watches, next, next) {
        w->fn(w->opaque, pin, pin->level);
    }
}

int64_t ecu_pin_change_ns(EcuPin *pin)
{
    return pin->change_ns;
}

void ecu_pin_mcu_drive(EcuPin *pin, int level)
{
    ecu_pin_mcu_drive_at(pin, level, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

void ecu_pin_mcu_drive_at(EcuPin *pin, int level, int64_t when_ns)
{
    level = !!level;
    if (pin->level == level) {
        return;
    }
    pin->level = level;
    pin->change_ns = when_ns;
    trace_ecu_pin_mcu_drive(pin->name, level);
    ecu_pin_notify(pin);
}

void ecu_pin_external_drive(EcuPin *pin, int level)
{
    level = !!level;
    if (pin->level == level) {
        return;
    }
    pin->level = level;
    pin->change_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (pin->input_fn) {
        pin->input_fn(pin->input_opaque, level);
    }
    ecu_pin_notify(pin);
}

void ecu_pin_add_watch(EcuPin *pin, EcuPinWatchFn fn, void *opaque)
{
    EcuPinWatch *w = g_new0(EcuPinWatch, 1);

    w->fn = fn;
    w->opaque = opaque;
    QLIST_INSERT_HEAD(&pin->watches, w, next);
}

void ecu_pin_remove_watch(EcuPin *pin, EcuPinWatchFn fn, void *opaque)
{
    EcuPinWatch *w, *next;

    QLIST_FOREACH_SAFE(w, &pin->watches, next, next) {
        if (w->fn == fn && w->opaque == opaque) {
            QLIST_REMOVE(w, next);
            g_free(w);
        }
    }
}

void ecu_pin_foreach(void (*fn)(EcuPin *pin, void *opaque), void *opaque)
{
    if (!ecu_pin_list) {
        return;
    }
    for (guint i = 0; i < ecu_pin_list->len; i++) {
        fn(g_ptr_array_index(ecu_pin_list, i), opaque);
    }
}

void ecu_analog_set_provider(EcuAnalogFn fn, void *opaque)
{
    analog_fn = fn;
    analog_opaque = opaque;
}

double ecu_analog_read(const char *channel)
{
    return analog_fn ? analog_fn(analog_opaque, channel) : 0.0;
}

void ecu_set_command_handler(EcuCommandFn fn, void *opaque)
{
    command_fn = fn;
    command_opaque = opaque;
}

bool ecu_run_command(const char *cmd, EcuOutputFn out, void *out_opaque)
{
    if (!command_fn) {
        return false;
    }
    command_fn(command_opaque, cmd, out, out_opaque);
    return true;
}

static void hmp_ecu_out(void *opaque, const char *line)
{
    monitor_hmp_printf(opaque, "%s\n", line);
}

void hmp_ecu(MonitorHMP *hmp, const QDict *qdict)
{
    const char *cmd = qdict_get_try_str(qdict, "cmd");

    if (!ecu_run_command(cmd ? cmd : "help", hmp_ecu_out, hmp)) {
        monitor_hmp_printf(hmp, "this machine has no ECU engine simulator\n");
    }
}
