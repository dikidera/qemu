/*
 * MC68376 configurable timer module 4 (CTM4).
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D), section 10 and
 * appendix D.7.  Section and table numbers below refer to that manual.
 *
 * Submodules (Figure 10-1, Table 10-7): BIUSM (0) with the CPSM (1),
 * MCSM2, DASM3, DASM4, PWMSM5-8, DASM9, DASM10, MCSM11, FCSM12.
 * Pins (ECU pin names as in the manual): CTD3, CTD4, CTD9, CTD10 (DASM),
 * CPWM5..CPWM8 (PWMSM), CTM2C (external counter clock).  CTD9 is also the
 * MCSM modulus load input (D.7.8).
 *
 * Timing: all counters run on a common system clock cycle grid derived
 * from the virtual clock and SYNCR (mc68376_sysclk_hz()).  Counter values
 * are computed from the time (no per-tick work); compare matches,
 * overflows and PWM edges are found by sweeping every counter from the
 * previous sweep to now, and output edges are reported with the exact
 * virtual time of the counter tick that caused them
 * (ecu_pin_mcu_drive_at()), so timer callback latency does not distort
 * pulse widths.  One QEMU timer is armed for the earliest upcoming event.
 *
 * Time base buses (10.3, Table 10-1): TBB1, TBB2, TBB4.  MCSM2 drives
 * TBB4 (bus A) / TBB2 (bus B), MCSM11 and FCSM12 TBB1 / TBB2; DASM3/4
 * use TBB4 / TBB2, DASM9/10 TBB1 / TBB2.  A bus driven by several
 * counters reads as the OR of their values (wired-OR); compares follow
 * the first driver.  An undriven bus reads zero.
 *
 * DASM operating modes (Table 10-2, D.7.11-D.7.13) follow the CTM
 * reference manual behaviour as summarised in appendix D; see
 * docs/system/ecu-emulator.rst for the details that are assumptions.
 *
 * Not modelled: freeze, WOR (open drain) electrical effect, test
 * registers (stored only).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/m68k/mc68376.h"

/* BIUMCR (D.7.1) */
#define BIUMCR_STOP     0x8000
#define BIUMCR_WMASK    0xdf21      /* STOP FRZ VECT IARB TBRS1 TBRS0 */
#define BIUMCR_VECT(v)  (((v) >> 11) & 3)
#define BIUMCR_IARB(v)  (((v) >> 8) & 7)

/* CPCR (D.7.4) */
#define CPCR_PRUN       0x0008
#define CPCR_DIV23      0x0004
#define CPCR_WMASK      0x000f

/* common SIC bits */
#define SIC_FLAG        0x8000      /* COF / FLAG */
#define SIC_IL(v)       (((v) >> 12) & 7)
#define SIC_IARB3       0x0800
#define SIC_DRVA        0x0200
#define SIC_DRVB        0x0100

/* FCSMSIC / MCSMSIC (D.7.6, D.7.8) */
#define FCSM_WMASK      0x7b07
#define MCSM_WMASK      0x7b37
#define CSM_IN          0x0080      /* IN (FCSM) / IN2 (MCSM): CTM2C */
#define MCSM_IN1        0x0040      /* CTD9 */
#define MCSM_EDGEN      0x0020
#define MCSM_EDGEP      0x0010

/* DASMSIC (D.7.11) */
#define DASM_WMASK      0x7b1f      /* IL IARB3 WOR BSL EDPOL MODE */
#define DASM_BSL        0x0100
#define DASM_IN         0x0080
#define DASM_FORCA      0x0040
#define DASM_FORCB      0x0020
#define DASM_EDPOL      0x0010

enum { DM_DIS, DM_IPWM, DM_IPM, DM_IC, DM_OCB, DM_OCAB, DM_RSV6, DM_RSV7,
       DM_OPWM };

/* PWMSIC (D.7.14) */
#define PWM_WMASK       0x781f      /* IL IARB3 POL EN CLK */
#define PWM_PIN         0x0080
#define PWM_LOAD        0x0020
#define PWM_POL         0x0010
#define PWM_EN          0x0008

#define NEVER           UINT64_MAX

/* time base bus numbers */
#define TBB1 1
#define TBB2 2
#define TBB4 4

typedef struct MC68376CTM4 CTM;

typedef struct CtmCounter {
    int n;                  /* submodule number */
    bool mcsm;
    int bus_a, bus_b;
    uint16_t sic, ml;
    uint16_t armed;         /* SIC bits read as one */
    /* counter state: value after tick e is f(anchor_v, e - anchor_e) */
    uint64_t div;           /* system clocks per count, 0: not time clocked */
    bool ext;               /* clocked by CTM2C edges */
    uint64_t ext_edges;
    uint64_t anchor_e;
    uint16_t anchor_v;
    uint64_t last_e;        /* tick index at the previous sweep */
    IMBIrq irq;
} CtmCounter;

typedef struct CtmDasm {
    struct MC68376CTM4 *ctm;
    int n;
    int bus_a, bus_b;
    uint16_t sic, a, b1, b2;
    uint16_t armed;
    bool ff;                /* output flip-flop */
    bool arm_a, arm_b;      /* OCB/OCAB comparators enabled */
    bool first;             /* IPM: no capture yet */
    bool lead;              /* IPWM: leading edge captured */
    int level;              /* input pin level */
    EcuPin *pin;
    IMBIrq irq;
} CtmDasm;

typedef struct CtmPwm {
    int n;
    uint16_t sic, a1, b1, a2, b2;
    uint16_t armed;
    bool ff;
    bool running;
    bool cleared;           /* the pulse of this period has ended */
    uint64_t ps;            /* system clock cycle of the period start */
    uint64_t div;
    uint16_t held;          /* counter while not running */
    EcuPin *pin;
    IMBIrq irq;
} CtmPwm;

struct MC68376CTM4 {
    MC68376State *soc;
    uint16_t biumcr, biutest, cpcr, cptr;

    /* system clock cycle grid */
    int64_t base_ns;
    uint64_t base_cyc;
    uint32_t hz;
    uint64_t prun_cyc;      /* prescaler released (grid origin) */
    Notifier clock_notifier;

    QEMUTimer *timer;

    CtmCounter mcsm2, mcsm11, fcsm12;
    CtmCounter *cnt[3];
    CtmDasm dasm[4];        /* 3, 4, 9, 10 */
    CtmPwm pwm[4];          /* 5..8 */
    EcuPin *ctm2c;
    int ctm2c_level;
};

static inline uint16_t merge16(uint16_t old, uint16_t val, uint16_t mask)
{
    return (old & ~mask) | (val & mask);
}

/* ---------------------------------------------------------------------- */
/* System clock grid                                                       */
/* ---------------------------------------------------------------------- */

static uint64_t now_cyc(CTM *m, int64_t ns)
{
    if (ns <= m->base_ns || !m->hz) {
        return m->base_cyc;
    }
    return m->base_cyc + muldiv64(ns - m->base_ns, m->hz,
                                  NANOSECONDS_PER_SECOND);
}

/* first virtual time (ns) at which cycle @c has been reached */
static int64_t cyc_ns(CTM *m, uint64_t c)
{
    uint64_t d, ns;

    if (c <= m->base_cyc || !m->hz) {
        return m->base_ns;
    }
    d = c - m->base_cyc;
    ns = muldiv64(d, NANOSECONDS_PER_SECOND, m->hz);
    if (muldiv64(ns, m->hz, NANOSECONDS_PER_SECOND) < d) {
        ns++;
    }
    return m->base_ns + ns;
}

static bool clocks_running(CTM *m)
{
    return (m->cpcr & CPCR_PRUN) && !(m->biumcr & BIUMCR_STOP);
}

/* CPSM outputs PCLK1..6 in system clocks (Figure 10-2, Table D-38) */
static uint64_t pclk_div(CTM *m, int k)
{
    uint64_t base = (m->cpcr & CPCR_DIV23) ? 3 : 2;

    if (k == 6) {
        return base * (32 << (m->cpcr & 3));
    }
    return base << (k - 1);
}

/* ---------------------------------------------------------------------- */
/* Counters (FCSM, MCSM)                                                   */
/* ---------------------------------------------------------------------- */

static uint16_t cnt_modulus(CtmCounter *c)
{
    return c->mcsm ? c->ml : 0;
}

static uint64_t cnt_cur_e(CTM *m, CtmCounter *c, uint64_t now_c)
{
    if (c->ext) {
        return c->ext_edges;
    }
    if (!c->div) {
        return c->anchor_e;
    }
    return now_c > m->prun_cyc ? (now_c - m->prun_cyc) / c->div : 0;
}

/* counter value after tick @e: up to $FFFF, then from the modulus */
static uint16_t cnt_value(CtmCounter *c, uint64_t e)
{
    uint64_t k = e - c->anchor_e;
    uint64_t first = 0x10000 - c->anchor_v;     /* ticks to the overflow */
    uint64_t period;

    if (e <= c->anchor_e) {
        return c->anchor_v;
    }
    if (k < first) {
        return c->anchor_v + k;
    }
    period = 0x10000 - cnt_modulus(c);
    return cnt_modulus(c) + (k - first) % period;
}

/*
 * First tick e > @from at which (value & @mask) == (@t & @mask), or NEVER.
 * Walks at most the rest of the current count and one modulus cycle.
 */
static uint64_t cnt_next_hit(CtmCounter *c, uint64_t from, uint16_t t,
                             uint16_t mask)
{
    uint64_t e = from;
    uint32_t v = cnt_value(c, from);

    for (int i = 0; i < 3; i++) {
        if (v != 0xffff) {
            uint32_t start = v + 1;
            uint32_t d = (t - start) & mask;

            if (start + d <= 0xffff) {
                return e + 1 + d;
            }
            e += 0xffff - v;
            v = 0xffff;
        }
        e += 1;                     /* overflow: reload from the modulus */
        v = cnt_modulus(c);
        if (((v ^ t) & mask) == 0) {
            return e;
        }
    }
    return NEVER;
}

/* first tick e > @from at which the counter overflows */
static uint64_t cnt_next_ovf(CtmCounter *c, uint64_t from)
{
    uint64_t e;

    if (cnt_value(c, from) == 0xffff) {
        return from + 1;
    }
    e = cnt_next_hit(c, from, 0xffff, 0xffff);
    return e == NEVER ? NEVER : e + 1;
}

/* virtual time of tick @e of a time-clocked counter */
static int64_t cnt_tick_ns(CTM *m, CtmCounter *c, uint64_t e, int64_t now)
{
    if (c->ext || !c->div) {
        return now;
    }
    return MIN(cyc_ns(m, m->prun_cyc + e * c->div), now);
}

static uint16_t cnt_read(CTM *m, CtmCounter *c, uint64_t now_c)
{
    return cnt_value(c, cnt_cur_e(m, c, now_c));
}

/* (re)derive the clock of @c; call cnt_freeze() before a change */
static void cnt_config(CTM *m, CtmCounter *c, uint64_t now_c)
{
    int clk = c->sic & 7;

    c->ext = false;
    c->div = 0;
    if (clocks_running(m)) {
        if (clk < 6) {
            c->div = pclk_div(m, clk + 1);
        } else {
            c->ext = true;
        }
    }
    c->anchor_e = c->last_e = cnt_cur_e(m, c, now_c);
}

static void cnt_freeze(CTM *m, CtmCounter *c, uint64_t now_c)
{
    c->anchor_v = cnt_read(m, c, now_c);
}

static void cnt_load(CTM *m, CtmCounter *c, uint64_t now_c, uint16_t v)
{
    c->anchor_v = v;
    c->anchor_e = c->last_e = cnt_cur_e(m, c, now_c);
}

static bool cnt_drives(CtmCounter *c, int bus)
{
    return ((c->sic & SIC_DRVA) && c->bus_a == bus) ||
           ((c->sic & SIC_DRVB) && c->bus_b == bus);
}

static CtmCounter *bus_driver(CTM *m, int bus)
{
    for (int i = 0; i < 3; i++) {
        if (cnt_drives(m->cnt[i], bus)) {
            return m->cnt[i];
        }
    }
    return NULL;
}

static uint16_t bus_value(CTM *m, int bus, uint64_t now_c)
{
    uint16_t v = 0;

    for (int i = 0; i < 3; i++) {
        if (cnt_drives(m->cnt[i], bus)) {
            v |= cnt_read(m, m->cnt[i], now_c);
        }
    }
    return v;
}

/* ---------------------------------------------------------------------- */
/* DASM (10.8, D.7.11-D.7.13)                                              */
/* ---------------------------------------------------------------------- */

static int dasm_mode(CtmDasm *d)
{
    int mode = d->sic & 0xf;

    return mode >= DM_OPWM ? DM_OPWM : mode;
}

static bool dasm_output(CtmDasm *d)
{
    int mode = dasm_mode(d);

    return mode == DM_OCB || mode == DM_OCAB || mode == DM_OPWM;
}

/* OPWM resolution: time base bits ignored (Table D-46) */
static uint16_t dasm_mask(CtmDasm *d)
{
    static const uint16_t masks[8] = {
        0xffff, 0x7fff, 0x3fff, 0x1fff, 0x0fff, 0x07ff, 0x01ff, 0x007f,
    };

    return dasm_mode(d) == DM_OPWM ? masks[d->sic & 7] : 0xffff;
}

static int dasm_bus(CtmDasm *d)
{
    return (d->sic & DASM_BSL) ? d->bus_b : d->bus_a;
}

/* pin level: the flip-flop after EDPOL (Table D-45) */
static int dasm_pin_out(CtmDasm *d)
{
    return d->ff ^ !!(d->sic & DASM_EDPOL);
}

static void dasm_drive(CtmDasm *d, int64_t when)
{
    if (dasm_output(d)) {
        ecu_pin_mcu_drive_at(d->pin, dasm_pin_out(d), when);
    }
}

static void dasm_sweep(CTM *m, CtmDasm *d, uint64_t now_c, int64_t now)
{
    CtmCounter *c = bus_driver(m, dasm_bus(d));
    uint16_t mask = dasm_mask(d);
    int mode = dasm_mode(d);
    uint64_t from, to;

    if (!dasm_output(d) || !c || !clocks_running(m)) {
        return;
    }
    from = c->last_e;
    to = cnt_cur_e(m, c, now_c);
    for (int i = 0; i < 256 && from < to; i++) {
        bool use_a = mode == DM_OPWM || d->arm_a;
        bool use_b = mode == DM_OPWM || d->arm_b;
        uint64_t ea = use_a ? cnt_next_hit(c, from, d->a, mask) : NEVER;
        uint64_t eb = use_b ? cnt_next_hit(c, from, d->b2, mask) : NEVER;
        uint64_t e = MIN(ea, eb);
        int before = dasm_pin_out(d);
        uint16_t v;

        if (e > to) {
            break;
        }
        v = cnt_value(c, e) & mask;
        if (use_a && v == (d->a & mask)) {
            /* channel A: set the flip-flop */
            d->ff = true;
            if (mode == DM_OPWM) {
                d->b2 = d->b1;          /* B double buffer (Table 10-3) */
                d->sic |= SIC_FLAG;
            } else {
                d->arm_a = false;
                if (mode == DM_OCAB) {
                    d->sic |= SIC_FLAG;
                }
            }
        }
        if (use_b && v == (d->b2 & mask)) {
            /* channel B: clear the flip-flop (B wins a tie: 0% duty) */
            d->ff = false;
            if (mode != DM_OPWM) {
                d->arm_b = false;
                d->sic |= SIC_FLAG;
            }
        }
        if (dasm_pin_out(d) != before) {
            ecu_pin_mcu_drive_at(d->pin, dasm_pin_out(d),
                                 cnt_tick_ns(m, c, e, now));
        }
        from = e;
    }
}

static uint64_t dasm_next(CTM *m, CtmDasm *d, uint64_t now_c, int64_t now)
{
    CtmCounter *c = bus_driver(m, dasm_bus(d));
    uint16_t mask = dasm_mask(d);
    int mode = dasm_mode(d);
    uint64_t cur, e = NEVER;

    if (!dasm_output(d) || !c || c->ext || !c->div) {
        return NEVER;
    }
    cur = cnt_cur_e(m, c, now_c);
    if (mode == DM_OPWM || d->arm_a) {
        e = MIN(e, cnt_next_hit(c, cur, d->a, mask));
    }
    if (mode == DM_OPWM || d->arm_b) {
        e = MIN(e, cnt_next_hit(c, cur, d->b2, mask));
    }
    return e == NEVER ? NEVER : m->prun_cyc + e * c->div;
}

/* input edge on a DASM pin (Tables D-44, D-45) */
static void dasm_input(CTM *m, CtmDasm *d, int level, uint64_t now_c)
{
    int mode = dasm_mode(d);
    bool rising = level, edpol = d->sic & DASM_EDPOL;
    uint16_t tb;

    if (!clocks_running(m)) {
        return;
    }
    tb = bus_value(m, dasm_bus(d), now_c);
    switch (mode) {
    case DM_IPWM:
        /* EDPOL = 0: A captures rising, B falling edges */
        if (rising != edpol) {
            if (d->lead) {
                d->a = tb;              /* trailing edge */
                d->b2 = d->b1;
                d->sic |= SIC_FLAG;
                d->lead = false;
            }
        } else {
            d->b1 = tb;                 /* leading edge */
            d->lead = true;
        }
        break;
    case DM_IPM:
    case DM_IC:
        if (rising != edpol) {
            d->b1 = d->a;
            d->b2 = d->b1;              /* previous capture */
            d->a = tb;
            if (mode == DM_IC || !d->first) {
                d->sic |= SIC_FLAG;
            }
            d->first = false;
        }
        break;
    }
}

static void dasm_set_mode(CTM *m, CtmDasm *d, int old_mode, int64_t now)
{
    int mode = dasm_mode(d);

    if (mode == old_mode) {
        return;
    }
    switch (mode) {
    case DM_DIS:
        d->sic &= ~SIC_FLAG;            /* selecting DIS clears FLAG */
        break;
    case DM_IPWM:
        d->lead = false;
        break;
    case DM_IPM:
    case DM_IC:
        d->first = true;
        break;
    case DM_OCB:
    case DM_OCAB:
        d->arm_a = d->arm_b = true;
        break;
    case DM_OPWM:
        d->b2 = d->b1;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.ctm4: DASM%d reserved "
                      "mode %d\n", d->n, mode);
        break;
    }
    dasm_drive(d, now);
}

/* ---------------------------------------------------------------------- */
/* PWMSM (10.9, D.7.14-D.7.17)                                             */
/* ---------------------------------------------------------------------- */

static uint64_t pwm_div(CTM *m, CtmPwm *p)
{
    static const uint16_t pre[8] = { 1, 2, 4, 8, 16, 32, 64, 256 };

    return pclk_div(m, 1) * pre[p->sic & 7];        /* Table D-50 */
}

static uint32_t pwm_period(CtmPwm *p)
{
    return p->a2 ? p->a2 : 0x10000;
}

static int pwm_pin_out(CtmPwm *p)
{
    if (!(p->sic & PWM_EN)) {
        return !!(p->sic & PWM_POL);
    }
    return p->ff ^ !!(p->sic & PWM_POL);
}

static bool pwm_steady(CtmPwm *p)
{
    return p->b2 == 0 || p->b2 >= pwm_period(p);
}

/* nothing visible happens at a period end: 0%/100% with stable values */
static bool pwm_idle_periods(CtmPwm *p)
{
    return pwm_steady(p) && (p->sic & SIC_FLAG) && p->a1 == p->a2 &&
           p->b1 == p->b2;
}

static uint16_t pwm_counter(CTM *m, CtmPwm *p, uint64_t now_c)
{
    uint64_t k;

    if (!p->running) {
        return p->held;
    }
    k = now_c > p->ps ? (now_c - p->ps) / p->div : 0;
    return 1 + k % pwm_period(p);
}

static void pwm_new_period(CtmPwm *p, uint64_t at)
{
    p->a2 = p->a1;
    p->b2 = p->b1;
    p->ps = at;
    p->cleared = false;
    p->ff = p->b2 != 0;
    p->sic |= SIC_FLAG;
}

static void pwm_sweep(CTM *m, CtmPwm *p, uint64_t now_c)
{
    for (int i = 0; i < 100000 && p->running; i++) {
        uint64_t per = pwm_period(p) * p->div;
        uint64_t end = p->ps + per;

        if (p->ff && !p->cleared && !pwm_steady(p)) {
            uint64_t clr = p->ps + p->b2 * p->div;
            if (clr <= now_c) {
                p->ff = false;
                p->cleared = true;
                ecu_pin_mcu_drive_at(p->pin, pwm_pin_out(p), cyc_ns(m, clr));
            }
        }
        if (end > now_c) {
            break;
        }
        if (pwm_idle_periods(p)) {
            /* skip whole periods without visible effect */
            p->ps += (now_c - p->ps) / per * per;
            continue;
        }
        pwm_new_period(p, end);
        ecu_pin_mcu_drive_at(p->pin, pwm_pin_out(p), cyc_ns(m, end));
    }
}

static uint64_t pwm_next(CtmPwm *p)
{
    uint64_t e = NEVER;

    if (!p->running) {
        return NEVER;
    }
    if (p->ff && !p->cleared && !pwm_steady(p)) {
        e = p->ps + p->b2 * p->div;
    }
    if (!pwm_idle_periods(p)) {
        e = MIN(e, p->ps + pwm_period(p) * p->div);
    }
    return e;
}

/* re-evaluate run state / clock; call after sweeping to now */
static void pwm_config(CTM *m, CtmPwm *p, uint64_t now_c, int64_t now,
                       bool was_en)
{
    bool en = p->sic & PWM_EN;
    bool run = en && clocks_running(m);
    uint64_t div = pwm_div(m, p);

    if (!en) {
        /* disabled: A1/B1 pass to A2/B2, flip-flop reset, counter 1 */
        p->running = false;
        p->a2 = p->a1;
        p->b2 = p->b1;
        p->ff = false;
        p->held = 1;
    } else if (!was_en) {
        /* EN 0 -> 1: start the first pulse, set FLAG (D.7.14) */
        pwm_new_period(p, now_c);
        p->div = div;
        p->running = run;
        p->held = 1;
    } else if (run != p->running) {
        if (run) {
            /* clocks restarted: continue from the held count */
            p->div = div;
            p->ps = now_c - (uint64_t)(p->held - 1) * div;
        } else {
            p->held = pwm_counter(m, p, now_c);
        }
        p->running = run;
    } else if (run && div != p->div) {
        uint16_t cv = pwm_counter(m, p, now_c);
        p->div = div;
        p->ps = now_c - (uint64_t)(cv - 1) * div;
    }
    ecu_pin_mcu_drive_at(p->pin, pwm_pin_out(p), now);
}

/* ---------------------------------------------------------------------- */
/* Sweep, schedule, interrupts                                             */
/* ---------------------------------------------------------------------- */

static void ctm_sweep(CTM *m, int64_t now)
{
    uint64_t now_c = now_cyc(m, now);

    for (int i = 0; i < 4; i++) {
        dasm_sweep(m, &m->dasm[i], now_c, now);
    }
    for (int i = 0; i < 4; i++) {
        pwm_sweep(m, &m->pwm[i], now_c);
    }
    for (int i = 0; i < 3; i++) {
        CtmCounter *c = m->cnt[i];
        uint64_t to = cnt_cur_e(m, c, now_c);

        if (to > c->last_e && cnt_next_ovf(c, c->last_e) <= to) {
            c->sic |= SIC_FLAG;         /* COF */
        }
        c->last_e = to;
    }
}

static void ctm_schedule(CTM *m, int64_t now)
{
    uint64_t now_c = now_cyc(m, now);
    uint64_t next = NEVER;

    for (int i = 0; i < 3; i++) {
        CtmCounter *c = m->cnt[i];
        if (c->div && !c->ext && !(c->sic & SIC_FLAG)) {
            uint64_t e = cnt_next_ovf(c, cnt_cur_e(m, c, now_c));
            if (e != NEVER) {
                next = MIN(next, m->prun_cyc + e * c->div);
            }
        }
    }
    for (int i = 0; i < 4; i++) {
        next = MIN(next, dasm_next(m, &m->dasm[i], now_c, now));
        next = MIN(next, pwm_next(&m->pwm[i]));
    }
    if (next == NEVER) {
        timer_del(m->timer);
    } else {
        timer_mod(m->timer, MAX(cyc_ns(m, next), now + 1));
    }
}

static void ctm_irq_one(CTM *m, IMBIrq *irq, int n, uint16_t sic)
{
    irq->level = SIC_IL(sic);
    irq->vector = (BIUMCR_VECT(m->biumcr) << 6) | n;    /* Table 10-7 */
    irq->iarb = BIUMCR_IARB(m->biumcr) | ((sic & SIC_IARB3) ? 8 : 0);
    irq->pending = irq->level && (sic & SIC_FLAG);
}

static void ctm_update_irq(CTM *m)
{
    for (int i = 0; i < 3; i++) {
        ctm_irq_one(m, &m->cnt[i]->irq, m->cnt[i]->n, m->cnt[i]->sic);
    }
    for (int i = 0; i < 4; i++) {
        ctm_irq_one(m, &m->dasm[i].irq, m->dasm[i].n, m->dasm[i].sic);
        ctm_irq_one(m, &m->pwm[i].irq, m->pwm[i].n, m->pwm[i].sic);
    }
    imb_irq_update(m->soc);
}

static void ctm_timer_cb(void *opaque)
{
    CTM *m = opaque;
    int64_t now = mc68376_now();

    ctm_sweep(m, now);
    ctm_update_irq(m);
    ctm_schedule(m, now);
}

/* re-derive every counter clock after a clock affecting change */
static void ctm_freeze_all(CTM *m, uint64_t now_c)
{
    for (int i = 0; i < 3; i++) {
        cnt_freeze(m, m->cnt[i], now_c);
    }
}

static void ctm_config_all(CTM *m, uint64_t now_c, int64_t now,
                           bool prun_rise)
{
    if (prun_rise) {
        m->prun_cyc = now_c;    /* prescaler released: new grid origin */
    }
    for (int i = 0; i < 3; i++) {
        cnt_config(m, m->cnt[i], now_c);
    }
    for (int i = 0; i < 4; i++) {
        CtmPwm *p = &m->pwm[i];
        pwm_config(m, p, now_c, now, p->sic & PWM_EN);
    }
}

static void ctm_clock_changed(Notifier *n, void *data)
{
    CTM *m = container_of(n, CTM, clock_notifier);
    int64_t now = mc68376_now();
    uint64_t now_c;

    ctm_sweep(m, now);
    now_c = now_cyc(m, now);
    ctm_freeze_all(m, now_c);
    /* continue the cycle count at the new rate */
    m->base_cyc = now_c;
    m->base_ns = now;
    m->hz = mc68376_sysclk_hz(m->soc);
    ctm_config_all(m, now_c, now, false);
    ctm_update_irq(m);
    ctm_schedule(m, now);
}

/* ---------------------------------------------------------------------- */
/* Pins                                                                    */
/* ---------------------------------------------------------------------- */

static void ctd_pin_cb(void *opaque, int level)
{
    CtmDasm *d = opaque;
    CTM *m = d->ctm;
    int64_t now = mc68376_now();
    uint64_t now_c;
    int old = d->level;

    ctm_sweep(m, now);
    now_c = now_cyc(m, now);
    d->level = !!level;
    if (old == d->level) {
        return;
    }
    if (getenv("CTMDBG") && d->n == 3) {
        fprintf(stderr, "CTD3 %d now=%" PRId64 " c=%" PRIu64 " tb=%04x\n",
                level, now, now_c, bus_value(m, TBB2, now_c));
    }
    if (!dasm_output(d)) {
        dasm_input(m, d, d->level, now_c);
    }
    if (d->n == 9) {
        /* modulus load input of the MCSMs (D.7.8, Table D-42) */
        CtmCounter *mc[2] = { &m->mcsm2, &m->mcsm11 };
        for (int i = 0; i < 2; i++) {
            CtmCounter *c = mc[i];
            if (clocks_running(m) &&
                ((level && (c->sic & MCSM_EDGEP)) ||
                 (!level && (c->sic & MCSM_EDGEN)))) {
                cnt_load(m, c, now_c, c->ml);
            }
        }
    }
    ctm_update_irq(m);
    ctm_schedule(m, now);
}

static void ctm2c_pin_cb(void *opaque, int level)
{
    CTM *m = opaque;
    int64_t now = mc68376_now();
    int old = m->ctm2c_level;

    ctm_sweep(m, now);
    m->ctm2c_level = !!level;
    if (old == m->ctm2c_level) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        CtmCounter *c = m->cnt[i];
        int clk = c->sic & 7;
        /* CLK = 6: negative edge, 7: positive edge (Table D-40) */
        if (c->ext && ((clk == 7 && level) || (clk == 6 && !level))) {
            c->ext_edges++;
        }
    }
    ctm_sweep(m, now);
    ctm_update_irq(m);
    ctm_schedule(m, now);
}

/* ---------------------------------------------------------------------- */
/* Register interface (Table D-35)                                         */
/* ---------------------------------------------------------------------- */

static CtmCounter *counter_of(CTM *m, int n)
{
    return n == 2 ? &m->mcsm2 : n == 11 ? &m->mcsm11 :
           n == 12 ? &m->fcsm12 : NULL;
}

static CtmDasm *dasm_of(CTM *m, int n)
{
    return n == 3 ? &m->dasm[0] : n == 4 ? &m->dasm[1] :
           n == 9 ? &m->dasm[2] : n == 10 ? &m->dasm[3] : NULL;
}

static CtmPwm *pwm_of(CTM *m, int n)
{
    return n >= 5 && n <= 8 ? &m->pwm[n - 5] : NULL;
}

static uint16_t ctm_read16(CTM *m, hwaddr off)
{
    int64_t now = mc68376_now();
    uint64_t now_c;
    int n = off >> 3, reg = (off & 7) >> 1;
    CtmCounter *c;
    CtmDasm *d;
    CtmPwm *p;
    uint16_t v = 0;

    ctm_sweep(m, now);
    now_c = now_cyc(m, now);

    if (n == 0) {
        switch (reg) {
        case 0:
            return m->biumcr;
        case 1:
            return m->biutest;
        case 2: {
            /* TBRS1:TBRS0 = TBB1, TBB2, TBB3 (none on CTM4), TBB4 */
            int sel = ((m->biumcr >> 4) & 2) | (m->biumcr & 1);
            static const int bus[4] = { TBB1, TBB2, 0, TBB4 };
            return bus[sel] ? bus_value(m, bus[sel], now_c) : 0;
        }
        }
        return 0;
    }
    if (n == 1) {
        return reg == 0 ? m->cpcr : reg == 1 ? m->cptr : 0;
    }
    if ((c = counter_of(m, n))) {
        switch (reg) {
        case 0:
            v = c->sic & ~(CSM_IN | MCSM_IN1);
            v |= m->ctm2c_level ? CSM_IN : 0;
            if (c->mcsm && m->dasm[2].level) {
                v |= MCSM_IN1;
            }
            c->armed |= v & SIC_FLAG;
            return v;
        case 1:
            return cnt_read(m, c, now_c);
        case 2:
            return c->mcsm ? c->ml : 0;
        }
        return 0;
    }
    if ((d = dasm_of(m, n))) {
        int mode = dasm_mode(d);
        switch (reg) {
        case 0:
            v = d->sic & ~(DASM_IN | DASM_FORCA | DASM_FORCB);
            if (dasm_output(d) ? dasm_pin_out(d) : d->level) {
                v |= DASM_IN;
            }
            d->armed |= v & SIC_FLAG;
            return v;
        case 1:
            return d->a;
        case 2:
            /* Table D-48: B1 in DIS and OPWM, B2 otherwise */
            return (mode == DM_DIS || mode == DM_OPWM) ? d->b1 : d->b2;
        }
        return 0;
    }
    if ((p = pwm_of(m, n))) {
        switch (reg) {
        case 0:
            v = p->sic & ~(PWM_PIN | PWM_LOAD);
            if (pwm_pin_out(p)) {
                v |= PWM_PIN;
            }
            p->armed |= v & SIC_FLAG;
            return v;
        case 1:
            return p->a1;
        case 2:
            return p->b1;
        case 3:
            return pwm_counter(m, p, now_c);
        }
    }
    return 0;           /* reserved: reads zero */
}

/* flags clear by writing zero after reading one */
static uint16_t sic_write(uint16_t sic, uint16_t *armed, uint16_t val,
                          uint16_t mask, uint16_t wmask)
{
    uint16_t v = merge16(sic, val, mask & wmask);

    if ((mask & SIC_FLAG) && !(val & SIC_FLAG) && (*armed & SIC_FLAG)) {
        v &= ~SIC_FLAG;
    }
    if (mask & SIC_FLAG) {
        *armed &= ~SIC_FLAG;
    }
    return v;
}

static void ctm_write16(CTM *m, hwaddr off, uint16_t val, uint16_t mask)
{
    int64_t now = mc68376_now();
    uint64_t now_c;
    int n = off >> 3, reg = (off & 7) >> 1;
    bool prun_rise = false;
    CtmCounter *c;
    CtmDasm *d;
    CtmPwm *p;

    ctm_sweep(m, now);
    now_c = now_cyc(m, now);
    ctm_freeze_all(m, now_c);

    if (n == 0) {
        if (reg == 0) {
            m->biumcr = merge16(m->biumcr, val, mask & BIUMCR_WMASK);
        } else if (reg == 1) {
            m->biutest = merge16(m->biutest, val, mask);
        }
    } else if (n == 1) {
        if (reg == 0) {
            uint16_t old = m->cpcr;
            m->cpcr = merge16(m->cpcr, val, mask & CPCR_WMASK);
            prun_rise = (m->cpcr & CPCR_PRUN) && !(old & CPCR_PRUN);
        } else if (reg == 1) {
            m->cptr = merge16(m->cptr, val, mask);
        }
    } else if ((c = counter_of(m, n))) {
        switch (reg) {
        case 0:
            c->sic = sic_write(c->sic, &c->armed, val, mask,
                               c->mcsm ? MCSM_WMASK : FCSM_WMASK);
            break;
        case 1:
            /* MCSM: a counter write also loads the modulus latch */
            cnt_load(m, c, now_c, merge16(cnt_read(m, c, now_c), val, mask));
            if (c->mcsm) {
                c->ml = c->anchor_v;
            }
            break;
        case 2:
            if (c->mcsm) {
                c->ml = merge16(c->ml, val, mask);
            }
            break;
        }
    } else if ((d = dasm_of(m, n))) {
        int old_mode = dasm_mode(d), mode;
        uint16_t old_sic = d->sic;

        switch (reg) {
        case 0:
            d->sic = sic_write(d->sic, &d->armed, val, mask, DASM_WMASK);
            mode = dasm_mode(d);
            dasm_set_mode(m, d, old_mode, now);
            if (dasm_output(d) && (mask & 0x00ff) &&
                (val & (DASM_FORCA | DASM_FORCB))) {
                /* FORCA sets, FORCB (or both) resets the flip-flop */
                d->ff = (val & (DASM_FORCA | DASM_FORCB)) == DASM_FORCA;
            }
            if (mode == old_mode && ((old_sic ^ d->sic) & DASM_EDPOL)) {
                d->lead = false;
                d->first = true;
            }
            dasm_drive(d, now);
            break;
        case 1:
            d->a = merge16(d->a, val, mask);
            if (old_mode == DM_OCB || old_mode == DM_OCAB) {
                d->arm_a = true;
            }
            break;
        case 2:
            switch (old_mode) {
            case DM_DIS:
                d->b1 = d->b2 = merge16(d->b1, val, mask);
                break;
            case DM_OPWM:
                d->b1 = merge16(d->b1, val, mask);
                break;
            case DM_OCB:
            case DM_OCAB:
                d->b2 = merge16(d->b2, val, mask);
                d->arm_b = true;
                break;
            default:
                d->b2 = merge16(d->b2, val, mask);
                break;
            }
            break;
        }
    } else if ((p = pwm_of(m, n))) {
        bool was_en = p->sic & PWM_EN;

        switch (reg) {
        case 0:
            p->sic = sic_write(p->sic, &p->armed, val, mask, PWM_WMASK);
            pwm_config(m, p, now_c, now, was_en);
            if ((mask & PWM_LOAD) && (val & PWM_LOAD) && (p->sic & PWM_EN)) {
                /* LOAD: new period now, without a glitch (D.7.14) */
                pwm_new_period(p, now_c);
                ecu_pin_mcu_drive_at(p->pin, pwm_pin_out(p), now);
            }
            break;
        case 1:
            p->a1 = merge16(p->a1, val, mask);
            break;
        case 2:
            p->b1 = merge16(p->b1, val, mask);
            break;
        }
        if (!(p->sic & PWM_EN)) {
            p->a2 = p->a1;
            p->b2 = p->b1;
        }
    }

    ctm_config_all(m, now_c, now, prun_rise);
    ctm_update_irq(m);
    ctm_schedule(m, now);
}

static uint64_t ctm_read(void *opaque, hwaddr addr, unsigned size)
{
    uint16_t v = ctm_read16(opaque, addr & ~1);

    if (size == 2) {
        return v;
    }
    return addr & 1 ? v & 0xff : v >> 8;
}

static void ctm_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    if (size == 2) {
        ctm_write16(opaque, addr, val, 0xffff);
    } else if (addr & 1) {
        ctm_write16(opaque, addr & ~1, val & 0xff, 0x00ff);
    } else {
        ctm_write16(opaque, addr, (val & 0xff) << 8, 0xff00);
    }
}

static const MemoryRegionOps ctm_ops = {
    .read = ctm_read,
    .write = ctm_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

void mc68376_ctm4_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    CTM *m = g_new0(CTM, 1);
    static const int dasm_n[4] = { 3, 4, 9, 10 };
    static const char *const dasm_irq[4] = {
        "ctm4-dasm3", "ctm4-dasm4", "ctm4-dasm9", "ctm4-dasm10",
    };
    static const char *const pwm_irq[4] = {
        "ctm4-pwm5", "ctm4-pwm6", "ctm4-pwm7", "ctm4-pwm8",
    };
    char name[8];

    m->soc = s;
    s->ctm4 = m;
    memory_region_init_io(mr, OBJECT(s), &ctm_ops, m, "mc68376.ctm4",
                          MC68376_CTM4_SIZE);
    m->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ctm_timer_cb, m);
    m->clock_notifier.notify = ctm_clock_changed;
    mc68376_add_clock_notifier(s, &m->clock_notifier);

    m->mcsm2 = (CtmCounter) { .n = 2, .mcsm = true, .bus_a = TBB4,
                              .bus_b = TBB2 };
    m->mcsm11 = (CtmCounter) { .n = 11, .mcsm = true, .bus_a = TBB1,
                               .bus_b = TBB2 };
    m->fcsm12 = (CtmCounter) { .n = 12, .bus_a = TBB1, .bus_b = TBB2 };
    m->cnt[0] = &m->mcsm2;
    m->cnt[1] = &m->mcsm11;
    m->cnt[2] = &m->fcsm12;

    for (int i = 0; i < 4; i++) {
        CtmDasm *d = &m->dasm[i];
        CtmPwm *p = &m->pwm[i];

        d->n = dasm_n[i];
        d->bus_a = d->n <= 4 ? TBB4 : TBB1;     /* Table 10-1 */
        d->bus_b = TBB2;
        snprintf(name, sizeof(name), "CTD%d", d->n);
        d->pin = ecu_pin(name);
        d->level = ecu_pin_level(d->pin);
        d->ctm = m;
        ecu_pin_set_input_handler(d->pin, ctd_pin_cb, d);

        p->n = 5 + i;
        snprintf(name, sizeof(name), "CPWM%d", p->n);
        p->pin = ecu_pin(name);
    }
    m->ctm2c = ecu_pin("CTM2C");
    m->ctm2c_level = ecu_pin_level(m->ctm2c);
    ecu_pin_set_input_handler(m->ctm2c, ctm2c_pin_cb, m);

    /* submodule number order: the lowest number wins (10.10) */
    imb_irq_register(s, &m->mcsm2.irq, "ctm4-mcsm2");
    imb_irq_register(s, &m->dasm[0].irq, dasm_irq[0]);
    imb_irq_register(s, &m->dasm[1].irq, dasm_irq[1]);
    for (int i = 0; i < 4; i++) {
        imb_irq_register(s, &m->pwm[i].irq, pwm_irq[i]);
    }
    imb_irq_register(s, &m->dasm[2].irq, dasm_irq[2]);
    imb_irq_register(s, &m->dasm[3].irq, dasm_irq[3]);
    imb_irq_register(s, &m->mcsm11.irq, "ctm4-mcsm11");
    imb_irq_register(s, &m->fcsm12.irq, "ctm4-fcsm12");
}

void mc68376_ctm4_reset(MC68376State *s)
{
    CTM *m = s->ctm4;
    int64_t now = mc68376_now();

    timer_del(m->timer);
    m->hz = mc68376_sysclk_hz(s);
    m->base_ns = now;
    m->base_cyc = 0;
    m->prun_cyc = 0;
    m->biumcr = 0x1800;             /* VECT = %11 (D.7.1) */
    m->biutest = m->cpcr = m->cptr = 0;

    for (int i = 0; i < 3; i++) {
        CtmCounter *c = m->cnt[i];
        c->sic = c->ml = c->armed = 0;
        c->div = 0;
        c->ext = false;
        c->ext_edges = c->anchor_e = c->last_e = 0;
        c->anchor_v = 0;
        c->irq.pending = false;
    }
    for (int i = 0; i < 4; i++) {
        CtmDasm *d = &m->dasm[i];
        CtmPwm *p = &m->pwm[i];

        d->sic = d->armed = 0;      /* DIS: the pin is not driven */
        d->ff = d->arm_a = d->arm_b = d->lead = false;
        d->first = true;
        d->irq.pending = false;

        p->sic = p->armed = 0;
        p->a2 = p->a1;
        p->b2 = p->b1;
        p->ff = p->running = p->cleared = false;
        p->held = 1;
        p->irq.pending = false;
        /* EN = 0, POL = 0: always low (Table D-49) */
        ecu_pin_mcu_drive(p->pin, 0);
    }
    ctm_update_irq(m);
}
