/*
 * ECU emulation: engine and vehicle simulator.
 *
 * Generates the electrical signals an engine control unit sees on its
 * connector (crank/cam wheel pulses, vehicle speed pulses, analog sensor
 * voltages, switch inputs) and measures what the ECU drives back
 * (injector pulse widths and timing, ignition dwell and advance, relays,
 * PWM outputs).  Everything is expressed as named signals bound to MCU
 * pin names through the hw/ecu/ecu-io.c registry, so the firmware runs
 * unmodified against the emulated MCU peripherals.
 *
 * Control: HMP "ecu <command>", the optional "chardev" property (line
 * oriented text protocol, same commands), QOM property "script".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "hw/ecu/ecu.h"
#include "qom/object.h"
#include <math.h>

OBJECT_DECLARE_SIMPLE_TYPE(EcuEngineState, ECU_ENGINE)

#define ECU_MAX_CYL     12
#define ECU_MAX_EDGES   2048
#define ECU_TICK_NS     (10 * SCALE_MS)
#define ECU_MAX_CURVE   64

/* ---------------------------------------------------------------------- */
/* Signals                                                                */
/* ---------------------------------------------------------------------- */

typedef enum {
    SK_ANALOG,      /* sensor voltage presented on an analog input */
    SK_DIGITAL,     /* digital input driven by the simulator */
    SK_OUTPUT,      /* ECU output measured by the simulator */
} SignalKind;

enum {
    /* analog sensors */
    SIG_MAP, SIG_MAF, SIG_CLT, SIG_IAT, SIG_TPS, SIG_TPS2, SIG_APP, SIG_APP2,
    SIG_O2, SIG_O2B, SIG_WBO2, SIG_VBAT, SIG_KNOCK, SIG_KNOCK2, SIG_BARO,
    SIG_FUEL_TEMP, SIG_FUEL_PRESS, SIG_OIL_PRESS, SIG_OIL_TEMP, SIG_EGT,
    SIG_FUEL_LEVEL, SIG_AC_PRESS, SIG_REF5, SIG_GND,
    SIG_ANAUX1, SIG_ANAUX2, SIG_ANAUX3, SIG_ANAUX4,
    /* digital inputs */
    SIG_CRANK, SIG_CAM, SIG_CAM2, SIG_VSS,
    SIG_IGN_SW, SIG_START_SW, SIG_NEUTRAL_SW, SIG_CLUTCH_SW, SIG_BRAKE_SW,
    SIG_AC_SW, SIG_PS_SW, SIG_DIN1, SIG_DIN2, SIG_DIN3, SIG_DIN4,
    /* outputs */
    SIG_INJ1,
    SIG_IGN1 = SIG_INJ1 + ECU_MAX_CYL,
    SIG_FPUMP = SIG_IGN1 + ECU_MAX_CYL,
    SIG_FAN1, SIG_FAN2, SIG_CEL, SIG_TACH, SIG_ISC, SIG_ISC2, SIG_PURGE,
    SIG_AC_CLUTCH, SIG_MAIN_RELAY, SIG_VVT, SIG_EGR, SIG_BOOST,
    SIG_O2_HEATER, SIG_AUX1, SIG_AUX2, SIG_AUX3, SIG_AUX4,
    SIG_COUNT
};

typedef struct SignalInfo {
    const char *name;
    SignalKind kind;
    const char *help;
} SignalInfo;

static SignalInfo sig_info[SIG_COUNT] = {
    [SIG_MAP]        = { "map", SK_ANALOG, "manifold pressure sensor (kPa)" },
    [SIG_MAF]        = { "maf", SK_ANALOG, "mass air flow sensor (g/s)" },
    [SIG_CLT]        = { "clt", SK_ANALOG, "coolant temperature NTC (degC)" },
    [SIG_IAT]        = { "iat", SK_ANALOG, "intake air temperature NTC (degC)" },
    [SIG_TPS]        = { "tps", SK_ANALOG, "throttle position (%)" },
    [SIG_TPS2]       = { "tps2", SK_ANALOG, "throttle position, 2nd track" },
    [SIG_APP]        = { "app", SK_ANALOG, "accelerator pedal (%)" },
    [SIG_APP2]       = { "app2", SK_ANALOG, "accelerator pedal, 2nd track" },
    [SIG_O2]         = { "o2", SK_ANALOG, "front narrowband O2 (lambda)" },
    [SIG_O2B]        = { "o2b", SK_ANALOG, "rear (post-cat) narrowband O2" },
    [SIG_WBO2]       = { "wbo2", SK_ANALOG, "wideband O2, 0-5V linear" },
    [SIG_VBAT]       = { "vbat", SK_ANALOG, "battery voltage via divider (V)" },
    [SIG_KNOCK]      = { "knock", SK_ANALOG, "knock sensor (V)" },
    [SIG_KNOCK2]     = { "knock2", SK_ANALOG, "knock sensor 2 (V)" },
    [SIG_BARO]       = { "baro", SK_ANALOG, "barometric pressure (kPa)" },
    [SIG_FUEL_TEMP]  = { "fuel_temp", SK_ANALOG, "fuel temperature NTC (degC)" },
    [SIG_FUEL_PRESS] = { "fuel_press", SK_ANALOG, "fuel pressure (kPa)" },
    [SIG_OIL_PRESS]  = { "oil_press", SK_ANALOG, "oil pressure (kPa)" },
    [SIG_OIL_TEMP]   = { "oil_temp", SK_ANALOG, "oil/ATF temperature NTC (degC)" },
    [SIG_EGT]        = { "egt", SK_ANALOG, "exhaust gas temp, 5mV/degC" },
    [SIG_FUEL_LEVEL] = { "fuel_level", SK_ANALOG, "fuel level (%)" },
    [SIG_AC_PRESS]   = { "ac_press", SK_ANALOG, "A/C refrigerant pressure (kPa)" },
    [SIG_REF5]       = { "ref5", SK_ANALOG, "constant reference voltage (vref)" },
    [SIG_GND]        = { "gnd", SK_ANALOG, "constant 0V" },
    [SIG_ANAUX1]     = { "anaux1", SK_ANALOG, "spare analog input (V)" },
    [SIG_ANAUX2]     = { "anaux2", SK_ANALOG, "spare analog input (V)" },
    [SIG_ANAUX3]     = { "anaux3", SK_ANALOG, "spare analog input (V)" },
    [SIG_ANAUX4]     = { "anaux4", SK_ANALOG, "spare analog input (V)" },
    [SIG_CRANK]      = { "crank", SK_DIGITAL, "crankshaft position wheel" },
    [SIG_CAM]        = { "cam", SK_DIGITAL, "camshaft position (phase)" },
    [SIG_CAM2]       = { "cam2", SK_DIGITAL, "second camshaft / bank 2" },
    [SIG_VSS]        = { "vss", SK_DIGITAL, "vehicle speed pulses" },
    [SIG_IGN_SW]     = { "ign_sw", SK_DIGITAL, "ignition switch" },
    [SIG_START_SW]   = { "start_sw", SK_DIGITAL, "starter signal" },
    [SIG_NEUTRAL_SW] = { "neutral_sw", SK_DIGITAL, "neutral/park switch" },
    [SIG_CLUTCH_SW]  = { "clutch_sw", SK_DIGITAL, "clutch switch" },
    [SIG_BRAKE_SW]   = { "brake_sw", SK_DIGITAL, "brake switch" },
    [SIG_AC_SW]      = { "ac_sw", SK_DIGITAL, "A/C request" },
    [SIG_PS_SW]      = { "ps_sw", SK_DIGITAL, "power steering switch" },
    [SIG_DIN1]       = { "din1", SK_DIGITAL, "spare digital input" },
    [SIG_DIN2]       = { "din2", SK_DIGITAL, "spare digital input" },
    [SIG_DIN3]       = { "din3", SK_DIGITAL, "spare digital input" },
    [SIG_DIN4]       = { "din4", SK_DIGITAL, "spare digital input" },
    [SIG_FPUMP]      = { "fpump", SK_OUTPUT, "fuel pump relay" },
    [SIG_FAN1]       = { "fan1", SK_OUTPUT, "radiator fan 1" },
    [SIG_FAN2]       = { "fan2", SK_OUTPUT, "radiator fan 2" },
    [SIG_CEL]        = { "cel", SK_OUTPUT, "check engine lamp" },
    [SIG_TACH]       = { "tach", SK_OUTPUT, "tachometer output" },
    [SIG_ISC]        = { "isc", SK_OUTPUT, "idle speed control (PWM/stepper)" },
    [SIG_ISC2]       = { "isc2", SK_OUTPUT, "idle speed control, 2nd coil" },
    [SIG_PURGE]      = { "purge", SK_OUTPUT, "canister purge solenoid" },
    [SIG_AC_CLUTCH]  = { "ac_clutch", SK_OUTPUT, "A/C compressor clutch" },
    [SIG_MAIN_RELAY] = { "main_relay", SK_OUTPUT, "main relay" },
    [SIG_VVT]        = { "vvt", SK_OUTPUT, "variable valve timing solenoid" },
    [SIG_EGR]        = { "egr", SK_OUTPUT, "EGR valve" },
    [SIG_BOOST]      = { "boost", SK_OUTPUT, "wastegate/boost control solenoid" },
    [SIG_O2_HEATER]  = { "o2_heater", SK_OUTPUT, "O2 sensor heater" },
    [SIG_AUX1]       = { "aux1", SK_OUTPUT, "spare output" },
    [SIG_AUX2]       = { "aux2", SK_OUTPUT, "spare output" },
    [SIG_AUX3]       = { "aux3", SK_OUTPUT, "spare output" },
    [SIG_AUX4]       = { "aux4", SK_OUTPUT, "spare output" },
};

static char inj_names[ECU_MAX_CYL][8];
static char ign_names[ECU_MAX_CYL][8];

static void sig_info_init(void)
{
    static bool done;

    if (done) {
        return;
    }
    done = true;
    for (int i = 0; i < ECU_MAX_CYL; i++) {
        snprintf(inj_names[i], sizeof(inj_names[i]), "inj%d", i + 1);
        snprintf(ign_names[i], sizeof(ign_names[i]), "ign%d", i + 1);
        sig_info[SIG_INJ1 + i] = (SignalInfo) {
            inj_names[i], SK_OUTPUT, "fuel injector driver" };
        sig_info[SIG_IGN1 + i] = (SignalInfo) {
            ign_names[i], SK_OUTPUT, "ignition coil driver" };
    }
}

static int sig_lookup(const char *name)
{
    for (int i = 0; i < SIG_COUNT; i++) {
        if (sig_info[i].name && !g_ascii_strcasecmp(sig_info[i].name, name)) {
            return i;
        }
    }
    return -1;
}

/* ---------------------------------------------------------------------- */
/* State                                                                  */
/* ---------------------------------------------------------------------- */

typedef struct EcuEdge {
    double angle;           /* 0..720 crank degrees */
    uint8_t sig;            /* SIG_CRANK, SIG_CAM, SIG_CAM2 */
    uint8_t level;
} EcuEdge;

typedef struct EcuOutput {
    EcuEngineState *s;
    int sig;
    int level;              /* electrical level of the pin */
    bool active;            /* logical on state */
    int64_t t_on;           /* ns, start of current activation */
    double angle_on;
    int64_t t_last_on;      /* ns, start of previous activation */
    double last_width_ms;   /* duration of the last activation */
    double last_angle_on;
    double last_angle_off;
    double period_ms;       /* time between activations */
    double duty;            /* percent, last period */
    uint64_t count;
    int64_t accum_eff_ns;   /* injectors: effective open time this cycle */
    bool fired_this_cycle;
} EcuOutput;

typedef struct EcuParams {
    /* operating point */
    double rpm, mode, tps, app, clt, iat, baro, vbat, speed;
    double knock, knock2, fuel_temp, fuel_press, oil_press, oil_temp, egt;
    double fuel_level, ac_press, anaux1, anaux2, anaux3, anaux4;
    double idle_rpm;
    /* engine geometry */
    double cyl, disp, ve, stoich, inj_flow, inj_dead, boost_ratio;
    double teeth, missing, tdc, vss_ppk;
    /* electrical behaviour */
    double vref, map_v0, map_gain, maf_v0, maf_gain;
    double ntc_r25, ntc_beta, ntc_pullup, vbat_div;
    double tps_v0, tps_v100, app_v0, app_v100;
    double inj_active, ign_active, out_active, crank_inv, cam_inv, vss_inv;
} EcuParams;

struct EcuEngineState {
    DeviceState parent_obj;

    CharFrontend chr;
    char *board_bind;
    char *user_bind;
    char *script;

    EcuParams p;

    /* overrides of modelled quantities (NAN = modelled) */
    double map_forced, maf_forced, lambda_forced;
    double forced_volts[SIG_COUNT];
    GHashTable *raw_volts;          /* "AN3" -> double* */

    /* digital input levels for static signals */
    int din_level[SIG_COUNT];

    /* bindings */
    GPtrArray *bound[SIG_COUNT];    /* of EcuPin* */
    GHashTable *analog_map;         /* "AN3" -> GINT_TO_POINTER(sig + 1) */

    char firing[ECU_MAX_CYL + 1];
    char *crank_pattern;
    char *cam_pattern;
    char *cam2_pattern;
    double maf_curve_v[ECU_MAX_CURVE];
    double maf_curve_g[ECU_MAX_CURVE];
    int maf_curve_n;

    /* wheel generator */
    EcuEdge edges[ECU_MAX_EDGES];
    int n_edges;
    int next_edge;
    double cycle_base;          /* absolute angle of the current 720 cycle */
    double a0;                  /* absolute crank angle at time t0 */
    int64_t t0;
    double cur_rpm;             /* rpm actually used by the generator */
    QEMUTimer *edge_timer;
    int64_t cycle_start_ns;
    double cycle_ms;

    QEMUTimer *vss_timer;
    int vss_level;

    QEMUTimer *tick_timer;
    int64_t last_tick_ns;

    /* model outputs */
    double map, maf, lambda, fuel_gps;
    int firing_cyl;
    bool running;

    EcuOutput out[SIG_COUNT];

    /* chardev protocol */
    char line[512];
    int line_len;
    int64_t stream_ms;
    QEMUTimer *stream_timer;
};

typedef struct ParamInfo {
    const char *name;
    size_t offset;
    const char *help;
    unsigned flags;
} ParamInfo;

#define PF_WHEEL   1    /* rebuild crank/cam edge table */
#define PF_RPM     2    /* re-time the wheel generator */
#define PF_VSS     4
#define PF_INPUTS  8    /* re-drive static digital inputs */

#define P(n, h, f) { #n, offsetof(EcuParams, n), h, f }
static const ParamInfo param_info[] = {
    P(rpm, "engine speed (fixed mode) / initial speed", PF_RPM),
    P(mode, "0 = fixed rpm, 1 = dynamic (rpm follows fuel/spark/tps)", 0),
    P(tps, "throttle opening %", 0),
    P(app, "accelerator pedal %", 0),
    P(clt, "coolant temperature degC", 0),
    P(iat, "intake air temperature degC", 0),
    P(baro, "barometric pressure kPa", 0),
    P(vbat, "battery voltage V", 0),
    P(speed, "vehicle speed km/h", PF_VSS),
    P(knock, "knock sensor voltage", 0),
    P(knock2, "knock sensor 2 voltage", 0),
    P(fuel_temp, "fuel temperature degC", 0),
    P(fuel_press, "fuel pressure kPa", 0),
    P(oil_press, "oil pressure kPa", 0),
    P(oil_temp, "oil temperature degC", 0),
    P(egt, "exhaust gas temperature degC", 0),
    P(fuel_level, "fuel level %", 0),
    P(ac_press, "A/C pressure kPa", 0),
    P(anaux1, "spare analog volts", 0),
    P(anaux2, "spare analog volts", 0),
    P(anaux3, "spare analog volts", 0),
    P(anaux4, "spare analog volts", 0),
    P(idle_rpm, "dynamic mode: idle speed at closed throttle", 0),
    P(cyl, "number of cylinders", PF_WHEEL),
    P(disp, "displacement in litres", 0),
    P(ve, "volumetric efficiency 0..1", 0),
    P(stoich, "stoichiometric AFR", 0),
    P(inj_flow, "injector flow cc/min", 0),
    P(inj_dead, "injector dead time ms", 0),
    P(boost_ratio, "max boost as fraction of baro at WOT (0 = NA)", 0),
    P(teeth, "crank wheel teeth (incl. missing)", PF_WHEEL),
    P(missing, "missing teeth", PF_WHEEL),
    P(tdc, "crank angle of cyl 1 TDC after the first tooth (deg)", 0),
    P(vss_ppk, "speed sensor pulses per km", PF_VSS),
    P(vref, "ADC reference / sensor supply voltage", 0),
    P(map_v0, "MAP sensor volts at 0 kPa", 0),
    P(map_gain, "MAP sensor volts per kPa", 0),
    P(maf_v0, "MAF volts at 0 g/s (when no maf_curve)", 0),
    P(maf_gain, "MAF volts per sqrt(g/s) (when no maf_curve)", 0),
    P(ntc_r25, "temperature sensors: NTC resistance at 25C (ohm)", 0),
    P(ntc_beta, "temperature sensors: NTC beta (K)", 0),
    P(ntc_pullup, "temperature sensors: pull-up resistor (ohm)", 0),
    P(vbat_div, "battery sense divider ratio", 0),
    P(tps_v0, "TPS volts at 0%", 0),
    P(tps_v100, "TPS volts at 100%", 0),
    P(app_v0, "pedal volts at 0%", 0),
    P(app_v100, "pedal volts at 100%", 0),
    P(inj_active, "injector driver active level (1 = high opens)", 0),
    P(ign_active, "coil driver active level (1 = high charges)", 0),
    P(out_active, "other outputs active level", 0),
    P(crank_inv, "invert crank signal", PF_WHEEL),
    P(cam_inv, "invert cam signals", PF_WHEEL),
    P(vss_inv, "invert speed signal", PF_VSS),
};
#undef P

static double *param_ptr(EcuEngineState *s, const ParamInfo *pi)
{
    return (double *)((char *)&s->p + pi->offset);
}

static const ParamInfo *param_lookup(const char *name)
{
    for (size_t i = 0; i < ARRAY_SIZE(param_info); i++) {
        if (!g_ascii_strcasecmp(param_info[i].name, name)) {
            return &param_info[i];
        }
    }
    return NULL;
}

/* ---------------------------------------------------------------------- */
/* Sensor models                                                          */
/* ---------------------------------------------------------------------- */

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static double ntc_volts(EcuEngineState *s, double degc)
{
    double t = degc + 273.15;
    double r = s->p.ntc_r25 * exp(s->p.ntc_beta * (1.0 / t - 1.0 / 298.15));

    return s->p.vref * r / (r + s->p.ntc_pullup);
}

static double maf_volts(EcuEngineState *s, double gps)
{
    if (s->maf_curve_n >= 2) {
        /* curve is volts -> g/s, monotonic; invert by interpolation */
        for (int i = 1; i < s->maf_curve_n; i++) {
            double g0 = s->maf_curve_g[i - 1], g1 = s->maf_curve_g[i];
            if (gps <= g1 || i == s->maf_curve_n - 1) {
                double v0 = s->maf_curve_v[i - 1], v1 = s->maf_curve_v[i];
                double f = g1 != g0 ? (gps - g0) / (g1 - g0) : 0;
                return clampd(v0 + f * (v1 - v0), 0, s->p.vref);
            }
        }
    }
    return clampd(s->p.maf_v0 + s->p.maf_gain * sqrt(MAX(gps, 0)),
                  0, s->p.vref);
}

static double signal_volts(EcuEngineState *s, int sig)
{
    EcuParams *p = &s->p;
    double v;

    if (!isnan(s->forced_volts[sig])) {
        return s->forced_volts[sig];
    }
    switch (sig) {
    case SIG_MAP:
        v = p->map_v0 + s->map * p->map_gain;
        break;
    case SIG_BARO:
        v = p->map_v0 + p->baro * p->map_gain;
        break;
    case SIG_MAF:
        v = maf_volts(s, s->maf);
        break;
    case SIG_CLT:
        v = ntc_volts(s, p->clt);
        break;
    case SIG_IAT:
        v = ntc_volts(s, p->iat);
        break;
    case SIG_FUEL_TEMP:
        v = ntc_volts(s, p->fuel_temp);
        break;
    case SIG_OIL_TEMP:
        v = ntc_volts(s, p->oil_temp);
        break;
    case SIG_TPS:
        v = p->tps_v0 + (p->tps_v100 - p->tps_v0) * p->tps / 100.0;
        break;
    case SIG_TPS2:
        v = p->vref - (p->tps_v0 + (p->tps_v100 - p->tps_v0) * p->tps / 100.0);
        break;
    case SIG_APP:
        v = p->app_v0 + (p->app_v100 - p->app_v0) * p->app / 100.0;
        break;
    case SIG_APP2:
        v = (p->app_v0 + (p->app_v100 - p->app_v0) * p->app / 100.0) / 2;
        break;
    case SIG_O2:
        v = s->running ? 0.45 + 0.42 * tanh((1.0 - s->lambda) * 25.0) : 0.45;
        break;
    case SIG_O2B:
        v = s->running ? 0.60 + 0.10 * tanh((1.0 - s->lambda) * 10.0) : 0.45;
        break;
    case SIG_WBO2:
        v = (s->lambda - 0.68) / (1.36 - 0.68) * 5.0;
        break;
    case SIG_VBAT:
        v = p->vbat * p->vbat_div;
        break;
    case SIG_KNOCK:
        v = p->knock;
        break;
    case SIG_KNOCK2:
        v = p->knock2;
        break;
    case SIG_FUEL_PRESS:
        v = 0.5 + p->fuel_press * 4.0 / 1000.0;
        break;
    case SIG_OIL_PRESS:
        v = 0.5 + p->oil_press * 4.0 / 1000.0;
        break;
    case SIG_AC_PRESS:
        v = 0.5 + p->ac_press * 4.0 / 3000.0;
        break;
    case SIG_EGT:
        v = p->egt * 0.005;
        break;
    case SIG_FUEL_LEVEL:
        v = 0.5 + 4.0 * p->fuel_level / 100.0;
        break;
    case SIG_REF5:
        v = p->vref;
        break;
    case SIG_ANAUX1:
        v = p->anaux1;
        break;
    case SIG_ANAUX2:
        v = p->anaux2;
        break;
    case SIG_ANAUX3:
        v = p->anaux3;
        break;
    case SIG_ANAUX4:
        v = p->anaux4;
        break;
    case SIG_GND:
    default:
        v = 0;
        break;
    }
    return clampd(v, 0, p->vref);
}

static double engine_analog_read(void *opaque, const char *channel)
{
    EcuEngineState *s = opaque;
    g_autofree char *key = g_ascii_strup(channel, -1);
    double *raw = g_hash_table_lookup(s->raw_volts, key);
    gpointer sig;

    if (raw) {
        return *raw;
    }
    sig = g_hash_table_lookup(s->analog_map, key);
    if (sig) {
        return signal_volts(s, GPOINTER_TO_INT(sig) - 1);
    }
    return 0.0;
}

/* ---------------------------------------------------------------------- */
/* Crank / cam wheel generator                                            */
/* ---------------------------------------------------------------------- */

static void drive_signal(EcuEngineState *s, int sig, int level)
{
    GPtrArray *pins = s->bound[sig];

    s->din_level[sig] = level;
    if (!pins) {
        return;
    }
    for (guint i = 0; i < pins->len; i++) {
        ecu_pin_external_drive(g_ptr_array_index(pins, i), level);
    }
}

static int edge_cmp(const void *a, const void *b)
{
    const EcuEdge *ea = a, *eb = b;

    return ea->angle < eb->angle ? -1 : ea->angle > eb->angle ? 1 : 0;
}

static void add_edge(EcuEngineState *s, double angle, int sig, int level)
{
    if (s->n_edges >= ECU_MAX_EDGES) {
        return;
    }
    angle = fmod(angle, 720.0);
    if (angle < 0) {
        angle += 720.0;
    }
    s->edges[s->n_edges++] = (EcuEdge) { angle, sig, level };
}

/* "start:width,start:width" (degrees) repeated every @period degrees */
static bool add_pattern(EcuEngineState *s, const char *pat, int sig,
                        double period, int inv)
{
    g_auto(GStrv) items = g_strsplit(pat, ",", -1);

    for (int i = 0; items[i]; i++) {
        double a, w;
        if (!*g_strstrip(items[i])) {
            continue;
        }
        if (sscanf(items[i], "%lf:%lf", &a, &w) != 2 || w <= 0) {
            return false;
        }
        for (double base = 0; base < 720.0; base += period) {
            add_edge(s, base + a, sig, 1 ^ inv);
            add_edge(s, base + a + w, sig, 0 ^ inv);
        }
    }
    return true;
}

static void build_wheel(EcuEngineState *s)
{
    int inv = s->p.crank_inv != 0;
    int cinv = s->p.cam_inv != 0;

    s->n_edges = 0;
    if (s->crank_pattern && *s->crank_pattern) {
        if (!add_pattern(s, s->crank_pattern, SIG_CRANK, 360.0, inv)) {
            warn_report("ecu-engine: bad crank pattern '%s'", s->crank_pattern);
        }
    } else {
        int n = (int)s->p.teeth, m = (int)s->p.missing;
        double pitch;

        if (n < 1) {
            n = 1;
        }
        if (m < 0 || m >= n) {
            m = 0;
        }
        pitch = 360.0 / n;
        for (int rev = 0; rev < 2; rev++) {
            for (int k = 0; k < n - m; k++) {
                add_edge(s, rev * 360.0 + k * pitch, SIG_CRANK, 1 ^ inv);
                add_edge(s, rev * 360.0 + k * pitch + pitch / 2, SIG_CRANK,
                         0 ^ inv);
            }
        }
    }
    if (s->cam_pattern && *s->cam_pattern &&
        !add_pattern(s, s->cam_pattern, SIG_CAM, 720.0, cinv)) {
        warn_report("ecu-engine: bad cam pattern '%s'", s->cam_pattern);
    }
    if (s->cam2_pattern && *s->cam2_pattern &&
        !add_pattern(s, s->cam2_pattern, SIG_CAM2, 720.0, cinv)) {
        warn_report("ecu-engine: bad cam2 pattern '%s'", s->cam2_pattern);
    }
    qsort(s->edges, s->n_edges, sizeof(EcuEdge), edge_cmp);
}

static double angle_at(EcuEngineState *s, int64_t now)
{
    return s->a0 + (now - s->t0) * s->cur_rpm * 6e-9;
}

static void schedule_edge(EcuEngineState *s)
{
    double target, dt;

    if (s->cur_rpm <= 0 || s->n_edges == 0) {
        timer_del(s->edge_timer);
        return;
    }
    target = s->cycle_base + s->edges[s->next_edge].angle;
    dt = (target - s->a0) / (s->cur_rpm * 6e-9);
    timer_mod(s->edge_timer, s->t0 + (int64_t)MAX(dt, 0));
}

/* Position the generator at absolute angle @a (keeping the edge index). */
static void wheel_resync(EcuEngineState *s, int64_t now)
{
    double a = angle_at(s, now);
    double rel;

    s->a0 = a;
    s->t0 = now;
    s->cycle_base = floor(a / 720.0) * 720.0;
    rel = a - s->cycle_base;
    s->next_edge = 0;
    while (s->next_edge < s->n_edges && s->edges[s->next_edge].angle <= rel) {
        s->next_edge++;
    }
    if (s->next_edge >= s->n_edges) {
        s->next_edge = 0;
        s->cycle_base += 720.0;
    }
}

static void set_rpm(EcuEngineState *s, double rpm)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (rpm < 0) {
        rpm = 0;
    }
    s->a0 = angle_at(s, now);
    s->t0 = now;
    s->cur_rpm = rpm;
    schedule_edge(s);
}

static void cycle_complete(EcuEngineState *s, int64_t now)
{
    double flow_gps = s->p.inj_flow / 60.0 * 0.74;   /* gasoline density */
    double fuel_g = 0;
    int ncyl = (int)clampd(s->p.cyl, 1, ECU_MAX_CYL);
    int firing = 0;
    bool any_inj_bound = false, any_ign_bound = false;

    s->cycle_ms = (now - s->cycle_start_ns) / 1e6;
    s->cycle_start_ns = now;

    for (int i = 0; i < ncyl; i++) {
        EcuOutput *inj = &s->out[SIG_INJ1 + i];
        EcuOutput *ign = &s->out[SIG_IGN1 + i];

        any_inj_bound |= s->bound[SIG_INJ1 + i] != NULL;
        any_ign_bound |= s->bound[SIG_IGN1 + i] != NULL;
        fuel_g += inj->accum_eff_ns / 1e9 * flow_gps;
        if ((inj->accum_eff_ns > 0 || !s->bound[SIG_INJ1 + i]) &&
            (ign->fired_this_cycle || !s->bound[SIG_IGN1 + i])) {
            firing++;
        }
        inj->accum_eff_ns = 0;
        ign->fired_this_cycle = false;
    }
    if (!any_inj_bound && !any_ign_bound) {
        firing = 0;
    }
    s->fuel_gps = s->cycle_ms > 0 ? fuel_g / (s->cycle_ms / 1000.0) : 0;
    s->firing_cyl = firing;
}

static void edge_cb(void *opaque)
{
    EcuEngineState *s = opaque;
    EcuEdge *e;
    double target;

    if (s->cur_rpm <= 0 || s->n_edges == 0) {
        return;
    }
    e = &s->edges[s->next_edge];
    target = s->cycle_base + e->angle;
    /* advance the time base exactly to the edge to avoid drift */
    s->t0 = s->t0 + (int64_t)((target - s->a0) / (s->cur_rpm * 6e-9));
    s->a0 = target;
    drive_signal(s, e->sig, e->level);

    if (++s->next_edge >= s->n_edges) {
        s->next_edge = 0;
        s->cycle_base += 720.0;
        cycle_complete(s, s->t0);
    }
    schedule_edge(s);
}

static void vss_cb(void *opaque)
{
    EcuEngineState *s = opaque;
    double hz = s->p.speed * s->p.vss_ppk / 3600.0;

    if (hz <= 0.01) {
        drive_signal(s, SIG_VSS, (s->p.vss_inv != 0));
        return;
    }
    s->vss_level ^= 1;
    drive_signal(s, SIG_VSS, s->vss_level ^ (s->p.vss_inv != 0));
    timer_mod(s->vss_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              (int64_t)(1e9 / hz / 2));
}

/* ---------------------------------------------------------------------- */
/* Output measurement                                                     */
/* ---------------------------------------------------------------------- */

static double cyl_tdc(EcuEngineState *s, int cyl)
{
    int ncyl = (int)clampd(s->p.cyl, 1, ECU_MAX_CYL);
    int pos = cyl;

    for (int i = 0; s->firing[i]; i++) {
        int c = s->firing[i] >= 'a' ? s->firing[i] - 'a' + 10
                                    : s->firing[i] - '1';
        if (c == cyl) {
            pos = i;
            break;
        }
    }
    return fmod(s->p.tdc + pos * 720.0 / ncyl, 720.0);
}

/* degrees before TDC of @cyl at which absolute angle @a lies, (-360,360] */
static double btdc(EcuEngineState *s, int cyl, double a)
{
    double d = fmod(cyl_tdc(s, cyl) - fmod(a, 720.0) + 1440.0, 720.0);

    return d > 360.0 ? d - 720.0 : d;
}

static double active_level(EcuEngineState *s, int sig)
{
    if (sig >= SIG_INJ1 && sig < SIG_INJ1 + ECU_MAX_CYL) {
        return s->p.inj_active;
    }
    if (sig >= SIG_IGN1 && sig < SIG_IGN1 + ECU_MAX_CYL) {
        return s->p.ign_active;
    }
    return s->p.out_active;
}

static void output_watch(void *opaque, EcuPin *pin, int level)
{
    EcuOutput *o = opaque;
    EcuEngineState *s = o->s;
    int64_t now = MIN(ecu_pin_change_ns(pin),
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    bool active = level == (int)(active_level(s, o->sig) != 0);
    double angle = angle_at(s, now);

    o->level = level;
    if (active == o->active) {
        return;
    }
    o->active = active;
    if (active) {
        if (o->count) {
            o->period_ms = (now - o->t_on) / 1e6;
            o->duty = o->period_ms > 0 ?
                      o->last_width_ms / o->period_ms * 100.0 : 0;
        }
        o->t_last_on = o->t_on;
        o->t_on = now;
        o->angle_on = angle;
        o->count++;
    } else {
        int64_t width = now - o->t_on;

        o->last_width_ms = width / 1e6;
        o->last_angle_on = o->angle_on;
        o->last_angle_off = angle;
        if (o->sig >= SIG_INJ1 && o->sig < SIG_INJ1 + ECU_MAX_CYL) {
            int64_t eff = width - (int64_t)(s->p.inj_dead * 1e6);
            if (eff > 0) {
                o->accum_eff_ns += eff;
            }
        } else if (o->sig >= SIG_IGN1 && o->sig < SIG_IGN1 + ECU_MAX_CYL) {
            o->fired_this_cycle = true;
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Engine model tick                                                      */
/* ---------------------------------------------------------------------- */

static void update_model(EcuEngineState *s, double dt)
{
    EcuParams *p = &s->p;
    double rpm = s->cur_rpm;
    int ncyl = (int)clampd(p->cyl, 1, ECU_MAX_CYL);

    if (p->mode != 0) {
        bool cranking = s->din_level[SIG_START_SW] && rpm < 400;
        double target = 0, tau;

        if (s->firing_cyl > 0 && s->lambda > 0.5 && s->lambda < 1.6) {
            target = (p->idle_rpm + MAX(p->tps, p->app) * 65.0) *
                     s->firing_cyl / ncyl;
        }
        if (cranking && target < 250) {
            target = 250;
        }
        tau = target > rpm ? 0.4 : 0.8;
        rpm += (target - rpm) * MIN(dt / tau, 1.0);
        if (rpm < 30 && target == 0) {
            rpm = 0;
        }
        if (fabs(rpm - s->cur_rpm) > 0.5 || (rpm == 0 && s->cur_rpm != 0)) {
            set_rpm(s, rpm);
        }
        p->rpm = rpm;
    }

    s->running = rpm > 50;
    if (!s->running) {
        s->cycle_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->fuel_gps = 0;
        s->firing_cyl = 0;
    }

    /* manifold pressure */
    if (!isnan(s->map_forced)) {
        s->map = s->map_forced;
    } else if (!s->running) {
        s->map = p->baro;
    } else {
        double t = clampd(MAX(p->tps, 0) / 100.0, 0, 1);
        s->map = p->baro * clampd(0.22 + 0.78 * pow(t, 0.6), 0, 1) *
                 (1.0 + p->boost_ratio * t);
    }

    /* air mass flow: speed-density */
    if (!isnan(s->maf_forced)) {
        s->maf = s->maf_forced;
    } else {
        double rho = s->map * 1000.0 / (287.05 * (p->iat + 273.15));
        s->maf = rho * (p->disp / 1000.0) * p->ve * rpm / 120.0 * 1000.0;
    }

    /* mixture */
    if (!isnan(s->lambda_forced)) {
        s->lambda = s->lambda_forced;
    } else if (!s->running) {
        s->lambda = 1.0;
    } else if (s->fuel_gps > 1e-6) {
        s->lambda = clampd(s->maf / s->fuel_gps / p->stoich, 0.4, 4.0);
    } else {
        s->lambda = 4.0;
    }
}

static void tick_cb(void *opaque)
{
    EcuEngineState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    double dt = (now - s->last_tick_ns) / 1e9;

    s->last_tick_ns = now;
    update_model(s, dt);
    timer_mod(s->tick_timer, now + ECU_TICK_NS);
}

/* ---------------------------------------------------------------------- */
/* Bindings                                                               */
/* ---------------------------------------------------------------------- */

static void unbind_signal(EcuEngineState *s, int sig)
{
    GPtrArray *pins = s->bound[sig];

    if (!pins) {
        return;
    }
    for (guint i = 0; i < pins->len; i++) {
        EcuPin *pin = g_ptr_array_index(pins, i);
        if (sig_info[sig].kind == SK_OUTPUT) {
            ecu_pin_remove_watch(pin, output_watch, &s->out[sig]);
        } else if (sig_info[sig].kind == SK_ANALOG) {
            g_hash_table_remove(s->analog_map, ecu_pin_name(pin));
        }
    }
    g_ptr_array_free(pins, true);
    s->bound[sig] = NULL;
}

static void bind_signal(EcuEngineState *s, int sig, const char *pinlist)
{
    g_auto(GStrv) names = g_strsplit_set(pinlist, "+ ", -1);

    unbind_signal(s, sig);
    for (int i = 0; names[i]; i++) {
        EcuPin *pin;

        if (!*names[i] || !g_ascii_strcasecmp(names[i], "none")) {
            continue;
        }
        pin = ecu_pin(names[i]);
        if (!s->bound[sig]) {
            s->bound[sig] = g_ptr_array_new();
        }
        g_ptr_array_add(s->bound[sig], pin);
        switch (sig_info[sig].kind) {
        case SK_OUTPUT:
            ecu_pin_add_watch(pin, output_watch, &s->out[sig]);
            s->out[sig].level = ecu_pin_level(pin);
            break;
        case SK_ANALOG:
            g_hash_table_insert(s->analog_map, (gpointer)ecu_pin_name(pin),
                                GINT_TO_POINTER(sig + 1));
            break;
        case SK_DIGITAL:
            ecu_pin_external_drive(pin, s->din_level[sig]);
            break;
        }
    }
}

static bool apply_bind_list(EcuEngineState *s, const char *list, Error **errp)
{
    g_auto(GStrv) items = NULL;

    if (!list) {
        return true;
    }
    items = g_strsplit(list, ",", -1);
    for (int i = 0; items[i]; i++) {
        char *eq;
        int sig;

        g_strstrip(items[i]);
        if (!*items[i]) {
            continue;
        }
        eq = strchr(items[i], '=');
        if (!eq) {
            error_setg(errp, "ecu-engine: bad binding '%s' (want sig=PIN)",
                       items[i]);
            return false;
        }
        *eq = 0;
        sig = sig_lookup(items[i]);
        if (sig < 0) {
            error_setg(errp, "ecu-engine: unknown signal '%s'", items[i]);
            return false;
        }
        bind_signal(s, sig, eq + 1);
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* Command interpreter                                                    */
/* ---------------------------------------------------------------------- */

static void outf(EcuOutputFn out, void *op, const char *fmt, ...)
    G_GNUC_PRINTF(3, 4);

static void outf(EcuOutputFn out, void *op, const char *fmt, ...)
{
    va_list ap;
    g_autofree char *str = NULL;

    va_start(ap, fmt);
    str = g_strdup_vprintf(fmt, ap);
    va_end(ap);
    out(op, str);
}

static void cmd_status(EcuEngineState *s, EcuOutputFn out, void *op)
{
    int ncyl = (int)clampd(s->p.cyl, 1, ECU_MAX_CYL);
    GString *str = g_string_new(NULL);

    g_string_append_printf(str, "rpm=%.0f map=%.1fkPa maf=%.2fg/s "
                           "lambda=%.3f afr=%.2f tps=%.1f clt=%.1f iat=%.1f "
                           "vbat=%.2f speed=%.1f fuel=%.3fg/s firing=%d/%d",
                           s->cur_rpm, s->map, s->maf, s->lambda,
                           s->lambda * s->p.stoich, s->p.tps, s->p.clt,
                           s->p.iat, s->p.vbat, s->p.speed, s->fuel_gps,
                           s->firing_cyl, ncyl);
    out(op, str->str);
    g_string_truncate(str, 0);
    for (int i = 0; i < ncyl; i++) {
        EcuOutput *inj = &s->out[SIG_INJ1 + i];
        EcuOutput *ign = &s->out[SIG_IGN1 + i];

        if (!s->bound[SIG_INJ1 + i] && !s->bound[SIG_IGN1 + i]) {
            continue;
        }
        g_string_append_printf(str, "cyl%d:", i + 1);
        if (s->bound[SIG_INJ1 + i]) {
            g_string_append_printf(str, " inj pw=%.3fms eoi=%.1fbtdc n=%"
                                   PRIu64, inj->last_width_ms,
                                   btdc(s, i, inj->last_angle_off),
                                   inj->count);
        }
        if (s->bound[SIG_IGN1 + i]) {
            g_string_append_printf(str, " ign adv=%.1fbtdc dwell=%.2fms n=%"
                                   PRIu64, btdc(s, i, ign->last_angle_off),
                                   ign->last_width_ms, ign->count);
        }
        out(op, str->str);
        g_string_truncate(str, 0);
    }
    for (int i = SIG_FPUMP; i < SIG_COUNT; i++) {
        EcuOutput *o = &s->out[i];
        if (!s->bound[i]) {
            continue;
        }
        g_string_append_printf(str, "%s=%s ", sig_info[i].name,
                               o->active ? "ON" : "off");
        if (o->count > 1 && o->period_ms > 0) {
            g_string_append_printf(str, "(%.1fHz %.1f%%) ",
                                   1000.0 / o->period_ms, o->duty);
        }
    }
    if (str->len) {
        out(op, str->str);
    }
    g_string_free(str, true);
}

static void print_bindings(EcuEngineState *s, EcuOutputFn out, void *op)
{
    for (int i = 0; i < SIG_COUNT; i++) {
        GString *str;

        if (!s->bound[i]) {
            continue;
        }
        str = g_string_new(NULL);
        g_string_append_printf(str, "%-11s ->", sig_info[i].name);
        for (guint j = 0; j < s->bound[i]->len; j++) {
            g_string_append_printf(str, " %s",
                ecu_pin_name(g_ptr_array_index(s->bound[i], j)));
        }
        out(op, str->str);
        g_string_free(str, true);
    }
}

typedef struct PinPrint {
    EcuOutputFn out;
    void *op;
} PinPrint;

static void print_pin(EcuPin *pin, void *opaque)
{
    PinPrint *pp = opaque;

    outf(pp->out, pp->op, "%-8s %d", ecu_pin_name(pin), ecu_pin_level(pin));
}

static bool parse_curve(EcuEngineState *s, const char *str)
{
    g_auto(GStrv) items = g_strsplit(str, ",", -1);
    int n = 0;

    for (int i = 0; items[i] && n < ECU_MAX_CURVE; i++) {
        double v, g;
        if (sscanf(items[i], "%lf:%lf", &v, &g) != 2) {
            return false;
        }
        s->maf_curve_v[n] = v;
        s->maf_curve_g[n] = g;
        n++;
    }
    s->maf_curve_n = n;
    return true;
}

static void apply_param_change(EcuEngineState *s, const ParamInfo *pi)
{
    if (pi->flags & PF_WHEEL) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        build_wheel(s);
        wheel_resync(s, now);
        schedule_edge(s);
    }
    if (pi->flags & PF_RPM) {
        set_rpm(s, s->p.rpm);
    }
    if (pi->flags & PF_VSS) {
        vss_cb(s);
    }
}

static const char ecu_help[] =
    "commands:\n"
    "  status                     engine state + measured outputs\n"
    "  set <name> <value>         set a parameter (rpm, tps, clt ...),\n"
    "                             a switch (ign_sw 1), a modelled value\n"
    "                             (map, maf, lambda) or a raw analog pin\n"
    "                             voltage (set AN3 1.25)\n"
    "  auto <name>                return map/maf/lambda/forced voltage/raw\n"
    "                             pin to the model\n"
    "  force <signal> <volts>     force a sensor signal voltage\n"
    "  get <name>                 read parameter, signal or pin\n"
    "  bind <signal> <PIN[+PIN]>  connect a signal to MCU pin(s)\n"
    "  unbind <signal>\n"
    "  bindings | signals | params | pins\n"
    "  wheel <teeth> <missing>    regular wheel, e.g. wheel 36 2\n"
    "  pattern crank|cam|cam2 <start:width,...>  custom tooth pattern\n"
    "                             (crank: degrees per rev, cam: per 720)\n"
    "  firing <order>             e.g. firing 1342\n"
    "  maf_curve <V:gps,...>      MAF transfer function\n"
    "  stream <ms>                periodic status on the chardev (0=off)";

static void ecu_command(void *opaque, const char *cmdline,
                        EcuOutputFn out, void *op)
{
    EcuEngineState *s = opaque;
    g_auto(GStrv) argv = g_strsplit_set(cmdline, " \t\r\n", -1);
    char *a[8] = { 0 };
    int argc = 0;
    const ParamInfo *pi;
    int sig;

    for (int i = 0; argv[i] && argc < 8; i++) {
        if (*argv[i]) {
            a[argc++] = argv[i];
        }
    }
    if (argc == 0) {
        return;
    }

    if (!strcmp(a[0], "help") || !strcmp(a[0], "?")) {
        g_auto(GStrv) lines = g_strsplit(ecu_help, "\n", -1);
        for (int i = 0; lines[i]; i++) {
            out(op, lines[i]);
        }
    } else if (!strcmp(a[0], "status")) {
        cmd_status(s, out, op);
    } else if (!strcmp(a[0], "bindings")) {
        print_bindings(s, out, op);
    } else if (!strcmp(a[0], "signals")) {
        static const char *kinds[] = { "analog", "input", "output" };
        for (int i = 0; i < SIG_COUNT; i++) {
            outf(out, op, "%-11s %-6s %s", sig_info[i].name,
                 kinds[sig_info[i].kind], sig_info[i].help);
        }
    } else if (!strcmp(a[0], "params")) {
        for (size_t i = 0; i < ARRAY_SIZE(param_info); i++) {
            outf(out, op, "%-11s %-10g %s", param_info[i].name,
                 *param_ptr(s, &param_info[i]), param_info[i].help);
        }
        outf(out, op, "%-11s %-10s %s", "firing", s->firing, "firing order");
    } else if (!strcmp(a[0], "pins")) {
        PinPrint pp = { out, op };
        ecu_pin_foreach(print_pin, &pp);
    } else if (!strcmp(a[0], "set") && argc >= 3) {
        double v = strtod(a[2], NULL);

        if ((pi = param_lookup(a[1]))) {
            *param_ptr(s, pi) = v;
            apply_param_change(s, pi);
        } else if (!g_ascii_strcasecmp(a[1], "map")) {
            s->map_forced = v;
        } else if (!g_ascii_strcasecmp(a[1], "maf")) {
            s->maf_forced = v;
        } else if (!g_ascii_strcasecmp(a[1], "lambda")) {
            s->lambda_forced = v;
        } else if (!g_ascii_strcasecmp(a[1], "afr")) {
            s->lambda_forced = v / s->p.stoich;
        } else if ((sig = sig_lookup(a[1])) >= 0 &&
                   sig_info[sig].kind == SK_DIGITAL) {
            drive_signal(s, sig, v != 0);
        } else if ((sig = sig_lookup(a[1])) >= 0 &&
                   sig_info[sig].kind == SK_ANALOG) {
            s->forced_volts[sig] = v;
        } else if (!g_ascii_strncasecmp(a[1], "AN", 2)) {
            double *d = g_new(double, 1);
            *d = v;
            g_hash_table_insert(s->raw_volts, g_ascii_strup(a[1], -1), d);
        } else {
            outf(out, op, "error: unknown name '%s'", a[1]);
            return;
        }
        update_model(s, 0);
        out(op, "ok");
    } else if (!strcmp(a[0], "force") && argc >= 3) {
        sig = sig_lookup(a[1]);
        if (sig < 0 || sig_info[sig].kind != SK_ANALOG) {
            outf(out, op, "error: '%s' is not an analog signal", a[1]);
            return;
        }
        s->forced_volts[sig] = strtod(a[2], NULL);
        out(op, "ok");
    } else if (!strcmp(a[0], "auto") && argc >= 2) {
        if (!g_ascii_strcasecmp(a[1], "map")) {
            s->map_forced = NAN;
        } else if (!g_ascii_strcasecmp(a[1], "maf")) {
            s->maf_forced = NAN;
        } else if (!g_ascii_strcasecmp(a[1], "lambda") ||
                   !g_ascii_strcasecmp(a[1], "afr")) {
            s->lambda_forced = NAN;
        } else if ((sig = sig_lookup(a[1])) >= 0) {
            s->forced_volts[sig] = NAN;
        } else {
            g_autofree char *k = g_ascii_strup(a[1], -1);
            g_hash_table_remove(s->raw_volts, k);
        }
        update_model(s, 0);
        out(op, "ok");
    } else if (!strcmp(a[0], "get") && argc >= 2) {
        if ((pi = param_lookup(a[1]))) {
            outf(out, op, "%s=%g", pi->name, *param_ptr(s, pi));
        } else if (!g_ascii_strcasecmp(a[1], "map")) {
            outf(out, op, "map=%g kPa (%.3f V)", s->map,
                 signal_volts(s, SIG_MAP));
        } else if (!g_ascii_strcasecmp(a[1], "maf")) {
            outf(out, op, "maf=%g g/s (%.3f V)", s->maf,
                 signal_volts(s, SIG_MAF));
        } else if (!g_ascii_strcasecmp(a[1], "lambda")) {
            outf(out, op, "lambda=%g", s->lambda);
        } else if ((sig = sig_lookup(a[1])) >= 0) {
            EcuOutput *o = &s->out[sig];
            switch (sig_info[sig].kind) {
            case SK_ANALOG:
                outf(out, op, "%s=%.4f V", a[1], signal_volts(s, sig));
                break;
            case SK_DIGITAL:
                outf(out, op, "%s=%d", a[1], s->din_level[sig]);
                break;
            case SK_OUTPUT:
                outf(out, op, "%s=%s width=%.3fms period=%.3fms duty=%.1f%% "
                     "on@%.1f off@%.1f count=%" PRIu64, a[1],
                     o->active ? "ON" : "off", o->last_width_ms,
                     o->period_ms, o->duty, fmod(o->last_angle_on, 720),
                     fmod(o->last_angle_off, 720), o->count);
                break;
            }
        } else {
            EcuPin *pin = ecu_pin(a[1]);
            if (!g_ascii_strncasecmp(a[1], "AN", 2)) {
                outf(out, op, "%s=%.4f V", ecu_pin_name(pin),
                     ecu_analog_read(a[1]));
            } else {
                outf(out, op, "%s=%d", ecu_pin_name(pin), ecu_pin_level(pin));
            }
        }
    } else if (!strcmp(a[0], "bind") && argc >= 3) {
        sig = sig_lookup(a[1]);
        if (sig < 0) {
            outf(out, op, "error: unknown signal '%s'", a[1]);
            return;
        }
        bind_signal(s, sig, a[2]);
        out(op, "ok");
    } else if (!strcmp(a[0], "unbind") && argc >= 2) {
        sig = sig_lookup(a[1]);
        if (sig < 0) {
            outf(out, op, "error: unknown signal '%s'", a[1]);
            return;
        }
        unbind_signal(s, sig);
        out(op, "ok");
    } else if (!strcmp(a[0], "wheel") && argc >= 3) {
        s->p.teeth = strtod(a[1], NULL);
        s->p.missing = strtod(a[2], NULL);
        g_free(s->crank_pattern);
        s->crank_pattern = NULL;
        apply_param_change(s, param_lookup("teeth"));
        out(op, "ok");
    } else if (!strcmp(a[0], "pattern") && argc >= 3) {
        char **dst = !strcmp(a[1], "crank") ? &s->crank_pattern :
                     !strcmp(a[1], "cam") ? &s->cam_pattern :
                     !strcmp(a[1], "cam2") ? &s->cam2_pattern : NULL;
        if (!dst) {
            out(op, "error: pattern crank|cam|cam2 <start:width,...>");
            return;
        }
        g_free(*dst);
        *dst = !strcmp(a[2], "none") ? NULL : g_strdup(a[2]);
        apply_param_change(s, param_lookup("teeth"));
        out(op, "ok");
    } else if (!strcmp(a[0], "firing") && argc >= 2) {
        pstrcpy(s->firing, sizeof(s->firing), a[1]);
        out(op, "ok");
    } else if (!strcmp(a[0], "maf_curve") && argc >= 2) {
        if (!parse_curve(s, a[1])) {
            out(op, "error: maf_curve V:gps,V:gps,...");
            return;
        }
        update_model(s, 0);
        out(op, "ok");
    } else if (!strcmp(a[0], "stream") && argc >= 2) {
        s->stream_ms = strtoll(a[1], NULL, 0);
        if (s->stream_ms > 0) {
            timer_mod(s->stream_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + s->stream_ms);
        } else {
            timer_del(s->stream_timer);
        }
        out(op, "ok");
    } else {
        outf(out, op, "error: unknown command '%s' (try help)", a[0]);
    }
}

static void script_out(void *opaque, const char *line)
{
    if (g_str_has_prefix(line, "error")) {
        warn_report("ecu-engine script: %s", line);
    }
}

static void run_script(EcuEngineState *s, const char *script)
{
    g_auto(GStrv) cmds = g_strsplit(script, ";", -1);

    for (int i = 0; cmds[i]; i++) {
        ecu_command(s, cmds[i], script_out, NULL);
    }
}

/* ---------------------------------------------------------------------- */
/* chardev                                                                */
/* ---------------------------------------------------------------------- */

static void chr_out(void *opaque, const char *line)
{
    EcuEngineState *s = opaque;

    qemu_chr_fe_write_all(&s->chr, (const uint8_t *)line, strlen(line));
    qemu_chr_fe_write_all(&s->chr, (const uint8_t *)"\r\n", 2);
}

static int chr_can_read(void *opaque)
{
    return 64;
}

static void chr_read(void *opaque, const uint8_t *buf, int size)
{
    EcuEngineState *s = opaque;

    for (int i = 0; i < size; i++) {
        if (buf[i] == '\n' || buf[i] == '\r') {
            if (s->line_len) {
                s->line[s->line_len] = 0;
                ecu_command(s, s->line, chr_out, s);
                s->line_len = 0;
            }
        } else if (s->line_len < (int)sizeof(s->line) - 1) {
            s->line[s->line_len++] = buf[i];
        }
    }
}

static void stream_cb(void *opaque)
{
    EcuEngineState *s = opaque;

    if (s->stream_ms <= 0) {
        return;
    }
    if (qemu_chr_fe_backend_connected(&s->chr)) {
        cmd_status(s, chr_out, s);
    }
    timer_mod(s->stream_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + s->stream_ms);
}

/* ---------------------------------------------------------------------- */
/* Device                                                                 */
/* ---------------------------------------------------------------------- */

static void ecu_engine_realize(DeviceState *dev, Error **errp)
{
    EcuEngineState *s = ECU_ENGINE(dev);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    sig_info_init();
    for (int i = 0; i < SIG_COUNT; i++) {
        s->out[i].s = s;
        s->out[i].sig = i;
        s->forced_volts[i] = NAN;
    }
    s->map_forced = s->maf_forced = s->lambda_forced = NAN;
    s->raw_volts = g_hash_table_new_full(g_str_hash, g_str_equal,
                                         g_free, g_free);
    s->analog_map = g_hash_table_new(g_str_hash, g_str_equal);
    s->din_level[SIG_IGN_SW] = 1;

    s->edge_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, edge_cb, s);
    s->vss_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vss_cb, s);
    s->tick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tick_cb, s);
    s->stream_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, stream_cb, s);

    if (!apply_bind_list(s, s->board_bind, errp) ||
        !apply_bind_list(s, s->user_bind, errp)) {
        return;
    }

    build_wheel(s);
    s->a0 = 0;
    s->t0 = now;
    s->cycle_base = 0;
    s->next_edge = 0;
    s->cycle_start_ns = now;
    s->last_tick_ns = now;

    ecu_analog_set_provider(engine_analog_read, s);
    ecu_set_command_handler(ecu_command, s);

    if (s->script) {
        run_script(s, s->script);
    }
    set_rpm(s, s->p.rpm);
    vss_cb(s);
    update_model(s, 0);
    timer_mod(s->tick_timer, now + ECU_TICK_NS);

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, chr_can_read, chr_read, NULL, NULL,
                                 s, NULL, true);
    }
}

static void ecu_engine_init(Object *obj)
{
    EcuEngineState *s = ECU_ENGINE(obj);

    /* A healthy, warm 4 cylinder 2.0l at idle. */
    s->p = (EcuParams) {
        .rpm = 800, .mode = 0, .tps = 0, .app = 0, .clt = 85, .iat = 25,
        .baro = 101.3, .vbat = 14.0, .speed = 0, .knock = 0.05,
        .knock2 = 0.05, .fuel_temp = 30, .fuel_press = 300,
        .oil_press = 250, .oil_temp = 85, .egt = 450, .fuel_level = 50,
        .ac_press = 800, .idle_rpm = 750,
        .cyl = 4, .disp = 2.0, .ve = 0.85, .stoich = 14.7, .inj_flow = 380,
        .inj_dead = 0.9, .boost_ratio = 0,
        .teeth = 36, .missing = 2, .tdc = 90, .vss_ppk = 2548,
        .vref = 5.0, .map_v0 = 0.2, .map_gain = 0.016,
        .maf_v0 = 0.6, .maf_gain = 0.3,
        .ntc_r25 = 2250, .ntc_beta = 3500, .ntc_pullup = 2700,
        .vbat_div = 0.25,
        .tps_v0 = 0.5, .tps_v100 = 4.5, .app_v0 = 0.8, .app_v100 = 4.2,
        .inj_active = 1, .ign_active = 1, .out_active = 1,
        .crank_inv = 0, .cam_inv = 0, .vss_inv = 0,
    };
    pstrcpy(s->firing, sizeof(s->firing), "1342");
    s->cam_pattern = g_strdup("40:20");
}

static const Property ecu_engine_props[] = {
    DEFINE_PROP_CHR("chardev", EcuEngineState, chr),
    DEFINE_PROP_STRING("board-bind", EcuEngineState, board_bind),
    DEFINE_PROP_STRING("bind", EcuEngineState, user_bind),
    DEFINE_PROP_STRING("script", EcuEngineState, script),
};

static void ecu_engine_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ecu_engine_realize;
    dc->desc = "ECU engine/vehicle simulator";
    dc->user_creatable = false;
    device_class_set_props(dc, ecu_engine_props);
}

static const TypeInfo ecu_engine_info = {
    .name = TYPE_ECU_ENGINE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(EcuEngineState),
    .instance_init = ecu_engine_init,
    .class_init = ecu_engine_class_init,
};

static void ecu_engine_register_types(void)
{
    type_register_static(&ecu_engine_info);
}

type_init(ecu_engine_register_types)
