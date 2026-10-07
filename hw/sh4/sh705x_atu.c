/*
 * Renesas SH705x Advanced Timer Unit II (ATU-II)
 *
 * Counters are derived from the virtual clock; compare matches, overflows
 * and one-shot pulse ends are found by "sweeping" every counter from its
 * value at the previous sweep to its current value, and a single QEMU timer
 * is armed for the earliest upcoming event.
 *
 * Pins (see hw/ecu): TI0A..TI0D, TIO1A..TIO1H, TIO2A..TIO2H, TIO3A..TIO5D,
 * TO6A..TO7D, TO8A..TO8P, TI9A..TI9F, TI10, TIO11A, TIO11B.
 *
 * Simplifications: the channel 10 angle clock (AGCK) is approximated from
 * the last tooth period, noise cancellers are ignored, DMA/A-D triggers
 * other than the channel 0 interval timer are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/sh4/sh705x.h"

/* IO field decoding for channels 1-5 and 11 (3-bit fields) */
#define IO_IS_OC(io)    ((io) >= 1 && (io) <= 3)
#define IO_IS_IC(io)    ((io) >= 4 && (io) <= 6)

enum {
    PIN_TI0 = 0,        /* 4 */
    PIN_TIO1 = 4,       /* 8 */
    PIN_TIO2 = 12,      /* 8 */
    PIN_TIO3 = 20,      /* 4 x 3 */
    PIN_TI9 = 32,       /* 6 */
    PIN_TI10 = 38,
    PIN_TIO11 = 39,     /* 2 */
    PIN_COUNT = 41,
};

typedef struct AtuPinCtx {
    SH705xATU *a;
    int id;
} AtuPinCtx;

static EcuPin *atu_pins[PIN_COUNT];
static EcuPin *to6_pins[2][4];
static EcuPin *to8_pins[16];

static int64_t phi_ps(SH705xATU *a, int psc)
{
    uint32_t div = (a->pscr[psc] & 0x1f) + 1;

    return 1000000000000LL * div / a->soc->pclk_hz;
}

/* period for an internal clock select (0..5 = phi'/1..phi'/32) */
static int64_t clk_period(SH705xATU *a, int psc, int cksel)
{
    if (cksel > 5) {
        return 0;   /* external clock / angle clock: handled separately */
    }
    return phi_ps(a, psc) << cksel;
}

static void update_clocks(SH705xATU *a, int64_t now)
{
    uint8_t t1 = a->tstr[1], t2 = a->tstr[0], t3 = a->tstr[2];

    atu_cnt_set_period(&a->cnt0, now, (t1 & 0x01) ? phi_ps(a, 0) : 0);
    for (int c = 0; c < 2; c++) {
        int64_t pa = (t1 & (c ? 0x04 : 0x02)) ?
                     clk_period(a, 0, a->ch12[c].tcra & 0xf) : 0;
        int64_t pb = (t1 & 0x08) ? clk_period(a, 0, a->ch12[c].tcrb & 0xf)
                                 : 0;

        if ((a->ch12[c].tcra & 0xf) == 8 && (t1 & (c ? 0x04 : 0x02)) &&
            a->ch10.tooth_ns) {
            /* angle clock from channel 10 */
            int mult = 1 << ((a->ch10.tior >> 4) & 3);
            pa = a->ch10.tooth_ns * 1000 / mult;
        }
        atu_cnt_set_period(&a->ch12[c].cnta, now, pa);
        atu_cnt_set_period(&a->ch12[c].cntb, now, pb);
    }
    for (int c = 0; c < 3; c++) {
        atu_cnt_set_period(&a->ch345[c].cnt, now, (t1 & (0x10 << c)) ?
                           clk_period(a, 0, a->ch345[c].tcr & 0xf) : 0);
    }
    for (int c = 0; c < 2; c++) {
        for (int x = 0; x < 4; x++) {
            uint8_t tcr = x < 2 ? a->ch67[c].tcra : a->ch67[c].tcrb;
            int cksel = (x & 1) ? (tcr >> 4) & 7 : tcr & 7;
            bool run = t2 & (1 << (4 * c + x));
            atu_cnt_set_period(&a->ch67[c].cnt[x], now,
                               run ? clk_period(a, 1 + c, cksel) : 0);
        }
    }
    atu_cnt_set_period(&a->ch10.cnta, now, (t1 & 0x80) ? phi_ps(a, 0) : 0);
    atu_cnt_set_period(&a->ch11.cnt, now, (t3 & 0x01) ?
                       clk_period(a, 0, a->ch11.tcr & 7) : 0);
}

/* ---------------------------------------------------------------------- */
/* Channel 8 one-shot pulse                                               */
/* ---------------------------------------------------------------------- */

static int64_t ch8_period(SH705xATU *a, int n)
{
    int cksel = n < 8 ? a->ch8.tcr & 7 : (a->ch8.tcr >> 4) & 7;

    return clk_period(a, 3, cksel);
}

static int64_t ch8_end_ns(SH705xATU *a, int n)
{
    int64_t p = ch8_period(a, n);

    if (!(a->ch8.dstr & (1 << n)) || !p) {
        return INT64_MAX;
    }
    return a->ch8.start_ns[n] + a->ch8.start_val[n] * p / 1000;
}

static uint16_t ch8_dcnt(SH705xATU *a, int n, int64_t now)
{
    int64_t p = ch8_period(a, n);
    int64_t done;

    if (!(a->ch8.dstr & (1 << n)) || !p) {
        return a->ch8.dcnt[n];
    }
    done = (now - a->ch8.start_ns[n]) * 1000 / p;
    if (done >= a->ch8.start_val[n]) {
        return 0;
    }
    return a->ch8.start_val[n] - done;
}

static void ch8_start(SH705xATU *a, int n, int64_t now)
{
    if (a->ch8.dcnt[n] == 0) {
        return;
    }
    a->ch8.dstr |= 1 << n;
    a->ch8.start_ns[n] = now;
    a->ch8.start_val[n] = a->ch8.dcnt[n];
    ecu_pin_mcu_drive_at(to8_pins[n], 1, now);
}

static void ch8_stop(SH705xATU *a, int n, int64_t when)
{
    a->ch8.dstr &= ~(1 << n);
    a->ch8.dcnt[n] = 0;
    a->ch8.tsr |= 1 << n;
    ecu_pin_mcu_drive_at(to8_pins[n], 0, when);
}

/* compare match on GR1x (n = 0..7) / GR2x (n = 8..15) */
static void ch8_link(SH705xATU *a, int n, int64_t now)
{
    if (a->ch8.tcnr & (1 << n)) {
        if (a->ch8.rldenr & 0x80) {
            a->ch8.dcnt[n] = a->ch8.rldr;
        }
        ch8_start(a, n, now);
    }
    if ((a->ch8.otr & (1 << n)) && (a->ch8.dstr & (1 << n))) {
        ch8_stop(a, n, now);
    }
}

/* ---------------------------------------------------------------------- */
/* Output compare helpers                                                 */
/* ---------------------------------------------------------------------- */

static void oc_action(EcuPin *pin, int io, int64_t when)
{
    switch (io) {
    case 1:
        ecu_pin_mcu_drive_at(pin, 0, when);
        break;
    case 2:
        ecu_pin_mcu_drive_at(pin, 1, when);
        break;
    case 3:
        ecu_pin_mcu_drive_at(pin, !ecu_pin_level(pin), when);
        break;
    }
}

/* time at which counter @c stepped onto @v since the previous sweep */
static int64_t hit_time(SH705xATU *a, AtuCounter *c, uint32_t v, int64_t now)
{
    return MIN(atu_cnt_time_to(c, a->last_sweep_ns, v), now);
}

static int ch12_io(SH705xATU *a, int c, int i)
{
    /* tior[0] = TIORA (B|A), [1] = TIORB (D|C), [2] = TIORC (F|E), ... */
    uint8_t r = a->ch12[c].tior[i / 2];

    return i & 1 ? (r >> 4) & 7 : r & 7;
}

static int ch345_io(SH705xATU *a, int c, int i)
{
    uint8_t r = i < 2 ? a->ch345[c].tiora : a->ch345[c].tiorb;

    return i & 1 ? (r >> 4) & 7 : r & 7;
}

/* ---------------------------------------------------------------------- */
/* Interrupts                                                             */
/* ---------------------------------------------------------------------- */

static void atu_irq(SH705xATU *a)
{
    SH705xState *s = a->soc;
    uint16_t f;

    f = a->tsr0 & a->tier0;
    for (int i = 0; i < 4; i++) {
        sh705x_set_irq(s, SH705X_VEC_ICI0A + 2 * i, f & (1 << i));
    }
    sh705x_set_irq(s, SH705X_VEC_OVI0, f & 0x10);
    sh705x_set_irq(s, SH705X_VEC_ITV, a->tsr0 & 0xe0);

    for (int c = 0; c < 2; c++) {
        uint16_t fa = a->ch12[c].tsra & a->ch12[c].tiera;
        uint16_t fb = a->ch12[c].tsrb & a->ch12[c].tierb;
        int base = c ? SH705X_VEC_IMI2A : SH705X_VEC_IMI1A;

        for (int i = 0; i < 8; i++) {
            bool req = fa & (1 << i);
            if (c == 1) {
                req |= fb & (1 << i);
            }
            sh705x_set_irq(s, base + i, req);
        }
        sh705x_set_irq(s, c ? SH705X_VEC_OVI2 : SH705X_VEC_OVI1,
                       (fa | fb) & 0x100);
    }

    f = a->tsr3 & a->tier3;
    for (int c = 0; c < 3; c++) {
        int base = SH705X_VEC_IMI3A + 8 * c;
        for (int i = 0; i < 4; i++) {
            sh705x_set_irq(s, base + i, f & (1 << (5 * c + i)));
        }
        sh705x_set_irq(s, base + 4, f & (1 << (5 * c + 4)));
    }

    for (int c = 0; c < 2; c++) {
        f = a->ch67[c].tsr & a->ch67[c].tier;
        for (int i = 0; i < 4; i++) {
            sh705x_set_irq(s, SH705X_VEC_CMI6A + 4 * c + i, f & (1 << i));
        }
    }

    f = a->ch8.tsr & a->ch8.tier;
    for (int i = 0; i < 16; i++) {
        sh705x_set_irq(s, SH705X_VEC_OSI8A + i, f & (1 << i));
    }

    f = a->ch9.tsr & a->ch9.tier;
    for (int i = 0; i < 4; i++) {
        sh705x_set_irq(s, SH705X_VEC_CMI9A + i, f & (1 << i));
    }
    sh705x_set_irq(s, SH705X_VEC_CMI9E, f & 0x10);
    sh705x_set_irq(s, SH705X_VEC_CMI9F, f & 0x20);

    f = a->ch10.tsr & a->ch10.tier;
    sh705x_set_irq(s, SH705X_VEC_CMI10A, f & 0x01);
    sh705x_set_irq(s, SH705X_VEC_CMI10B, f & 0x04);
    sh705x_set_irq(s, SH705X_VEC_ICI10A, f & 0x0a);

    f = a->ch11.tsr & a->ch11.tier;
    sh705x_set_irq(s, SH705X_VEC_IMI11A, f & 0x01);
    sh705x_set_irq(s, SH705X_VEC_IMI11B, f & 0x02);
    sh705x_set_irq(s, SH705X_VEC_OVI11, f & 0x100);
}

/* ---------------------------------------------------------------------- */
/* Sweep                                                                  */
/* ---------------------------------------------------------------------- */

static void sweep_itv(SH705xATU *a, uint32_t from, uint32_t to)
{
    static const struct { uint8_t *reg; int first_bit; uint16_t flag; } itv[] = {
        { NULL, 6, 0x20 }, { NULL, 10, 0x40 }, { NULL, 10, 0x80 },
    };
    uint8_t regs[3] = { a->itvrr1, a->itvrr2a, a->itvrr2b };
    uint32_t moved = to - from;

    for (int r = 0; r < 3; r++) {
        for (int i = 0; i < 4; i++) {
            int b = itv[r].first_bit + i;
            uint32_t step = 1u << (b + 1), p;
            bool hit;

            if (!(regs[r] & (0x11 << i))) {
                continue;
            }
            /* next value after @from with bit b rising */
            p = (from & ~(step - 1)) + (1u << b);
            if ((int32_t)(p - from) <= 0) {
                p += step;
            }
            hit = (uint32_t)(p - from) <= moved && moved != 0;
            if (!hit) {
                continue;
            }
            if (regs[r] & (0x01 << i)) {
                a->tsr0 |= itv[r].flag;
            }
            if (regs[r] & (0x10 << i)) {
                sh705x_adc_trigger(a->soc);
            }
        }
    }
}

static int64_t next_itv(SH705xATU *a, int64_t now)
{
    uint8_t regs[3] = { a->itvrr1, a->itvrr2a, a->itvrr2b };
    int first[3] = { 6, 10, 10 };
    uint32_t cur = atu_cnt_get(&a->cnt0, now);
    int64_t next = INT64_MAX;

    for (int r = 0; r < 3; r++) {
        for (int i = 0; i < 4; i++) {
            int b = first[r] + i;
            uint32_t step = 1u << (b + 1), p;
            if (!(regs[r] & (0x11 << i))) {
                continue;
            }
            p = (cur & ~(step - 1)) + (1u << b);
            if ((int32_t)(p - cur) <= 0) {
                p += step;
            }
            next = MIN(next, atu_cnt_time_to(&a->cnt0, now, p));
        }
    }
    return next;
}

static void atu_sweep(SH705xATU *a, int64_t now)
{
    uint32_t from, to;

    /* channel 0 */
    from = a->cnt0.last;
    to = atu_cnt_get(&a->cnt0, now);
    if (atu_cnt_passed(&a->cnt0, from, to, 0)) {
        a->tsr0 |= 0x10;
    }
    sweep_itv(a, from, to);
    a->cnt0.last = to;

    /* channels 1, 2 */
    for (int c = 0; c < 2; c++) {
        AtuCounter *ca = &a->ch12[c].cnta, *cb = &a->ch12[c].cntb;
        EcuPin **pins = &atu_pins[c ? PIN_TIO2 : PIN_TIO1];

        from = ca->last;
        to = atu_cnt_get(ca, now);
        for (int i = 0; i < 8; i++) {
            int io = ch12_io(a, c, i);
            if (IO_IS_OC(io) &&
                atu_cnt_passed(ca, from, to, a->ch12[c].gr[i])) {
                a->ch12[c].tsra |= 1 << i;
                int64_t t = hit_time(a, ca, a->ch12[c].gr[i], now);
                oc_action(pins[i], io, t);
                ch8_link(a, 8 * c + i, t);
            }
        }
        if (atu_cnt_passed(ca, from, to, 0)) {
            a->ch12[c].tsra |= 0x100;
        }
        ca->last = to;

        from = cb->last;
        to = atu_cnt_get(cb, now);
        if (c == 0) {
            if (atu_cnt_passed(cb, from, to, a->ch12[0].ocr[0])) {
                a->ch12[0].tsrb |= 1;
            }
        } else {
            for (int i = 0; i < 8; i++) {
                if (atu_cnt_passed(cb, from, to, a->ch12[1].ocr[i])) {
                    a->ch12[1].tsrb |= 1 << i;
                }
            }
        }
        if (atu_cnt_passed(cb, from, to, 0)) {
            a->ch12[c].tsrb |= 0x100;
        }
        cb->last = to;
    }

    /* channels 3, 4, 5 */
    for (int c = 0; c < 3; c++) {
        AtuCounter *cn = &a->ch345[c].cnt;
        EcuPin **pins = &atu_pins[PIN_TIO3 + 4 * c];
        bool pwm = a->tmdr & (1 << c);

        from = cn->last;
        to = atu_cnt_get(cn, now);
        for (int i = 0; i < 4; i++) {
            int io = ch345_io(a, c, i);
            if (!atu_cnt_passed(cn, from, to, a->ch345[c].gr[i])) {
                continue;
            }
            if (pwm) {
                a->tsr3 |= 1 << (5 * c + i);
                if (i == 3) {
                    /* cycle register: clear counter, outputs high */
                    int64_t t = atu_cnt_time_to(cn, a->last_sweep_ns,
                                                a->ch345[c].gr[3]);
                    atu_cnt_set(cn, MIN(t, now), 0);
                    to = atu_cnt_get(cn, now);
                    for (int j = 0; j < 3; j++) {
                        ecu_pin_mcu_drive_at(pins[j], 1, MIN(t, now));
                    }
                } else {
                    ecu_pin_mcu_drive_at(pins[i], 0,
                        hit_time(a, cn, a->ch345[c].gr[i], now));
                }
            } else if (IO_IS_OC(io)) {
                a->tsr3 |= 1 << (5 * c + i);
                oc_action(pins[i], io, hit_time(a, cn, a->ch345[c].gr[i], now));
            }
        }
        if (atu_cnt_passed(cn, from, to, 0) && !pwm) {
            a->tsr3 |= 1 << (5 * c + 4);
        }
        cn->last = to;
    }

    /* channels 6, 7: PWM timers */
    for (int c = 0; c < 2; c++) {
        for (int x = 0; x < 4; x++) {
            AtuCounter *cn = &a->ch67[c].cnt[x];
            EcuPin *pin = to6_pins[c][x];

            from = cn->last;
            to = atu_cnt_get(cn, now);
            if (atu_cnt_passed(cn, from, to, a->ch67[c].dtr[x])) {
                ecu_pin_mcu_drive_at(pin, 0,
                                     hit_time(a, cn, a->ch67[c].dtr[x], now));
            }
            if (atu_cnt_passed(cn, from, to, a->ch67[c].cylr[x])) {
                int64_t t = atu_cnt_time_to(cn, a->last_sweep_ns,
                                            a->ch67[c].cylr[x]);
                atu_cnt_set(cn, MIN(t, now), 0);
                to = atu_cnt_get(cn, now);
                a->ch67[c].tsr |= 1 << x;
                a->ch67[c].dtr[x] = a->ch67[c].bfr[x];
                ecu_pin_mcu_drive_at(pin, a->ch67[c].dtr[x] != 0, MIN(t, now));
            }
            cn->last = to;
        }
    }

    /* channel 8 */
    for (int n = 0; n < 16; n++) {
        int64_t end = ch8_end_ns(a, n);
        if (end <= now) {
            ch8_stop(a, n, end);
        }
    }

    /* channel 10 */
    from = a->ch10.cnta.last;
    to = atu_cnt_get(&a->ch10.cnta, now);
    if (atu_cnt_passed(&a->ch10.cnta, from, to, a->ch10.ocra)) {
        a->ch10.tsr |= 0x01;
    }
    a->ch10.cnta.last = to;

    /* channel 11 */
    from = a->ch11.cnt.last;
    to = atu_cnt_get(&a->ch11.cnt, now);
    for (int i = 0; i < 2; i++) {
        int io = i ? (a->ch11.tior >> 4) & 7 : a->ch11.tior & 7;
        uint16_t gr = i ? a->ch11.grb : a->ch11.gra;
        if (IO_IS_OC(io) && atu_cnt_passed(&a->ch11.cnt, from, to, gr)) {
            a->ch11.tsr |= 1 << i;
            oc_action(atu_pins[PIN_TIO11 + i], io,
                      hit_time(a, &a->ch11.cnt, gr, now));
        }
    }
    if (atu_cnt_passed(&a->ch11.cnt, from, to, 0)) {
        a->ch11.tsr |= 0x100;
    }
    a->ch11.cnt.last = to;

    a->last_sweep_ns = now;
}

static int64_t cnt_next(AtuCounter *c, int64_t now, uint32_t v)
{
    return atu_cnt_time_to(c, now, v);
}

/* guarantee at least one sweep per half wrap so no event is missed */
static int64_t cnt_guard(AtuCounter *c, int64_t now)
{
    return atu_cnt_time_to(c, now, atu_cnt_get(c, now) + (c->mask >> 1));
}

static void atu_schedule(SH705xATU *a, int64_t now)
{
    int64_t next = INT64_MAX;

    if (a->cnt0.period_ps) {
        next = MIN(next, cnt_guard(&a->cnt0, now));
        next = MIN(next, cnt_next(&a->cnt0, now, 0));
        next = MIN(next, next_itv(a, now));
    }
    for (int c = 0; c < 2; c++) {
        AtuCounter *ca = &a->ch12[c].cnta, *cb = &a->ch12[c].cntb;
        if (ca->period_ps) {
            next = MIN(next, cnt_guard(ca, now));
            for (int i = 0; i < 8; i++) {
                if (IO_IS_OC(ch12_io(a, c, i))) {
                    next = MIN(next, cnt_next(ca, now, a->ch12[c].gr[i]));
                }
            }
        }
        if (cb->period_ps) {
            next = MIN(next, cnt_guard(cb, now));
            for (int i = 0; i < (c ? 8 : 1); i++) {
                next = MIN(next, cnt_next(cb, now, a->ch12[c].ocr[i]));
            }
        }
    }
    for (int c = 0; c < 3; c++) {
        AtuCounter *cn = &a->ch345[c].cnt;
        if (!cn->period_ps) {
            continue;
        }
        next = MIN(next, cnt_guard(cn, now));
        for (int i = 0; i < 4; i++) {
            if ((a->tmdr & (1 << c)) || IO_IS_OC(ch345_io(a, c, i))) {
                next = MIN(next, cnt_next(cn, now, a->ch345[c].gr[i]));
            }
        }
    }
    for (int c = 0; c < 2; c++) {
        for (int x = 0; x < 4; x++) {
            AtuCounter *cn = &a->ch67[c].cnt[x];
            if (!cn->period_ps) {
                continue;
            }
            next = MIN(next, cnt_guard(cn, now));
            next = MIN(next, cnt_next(cn, now, a->ch67[c].dtr[x]));
            next = MIN(next, cnt_next(cn, now, a->ch67[c].cylr[x]));
        }
    }
    for (int n = 0; n < 16; n++) {
        next = MIN(next, ch8_end_ns(a, n));
    }
    if (a->ch10.cnta.period_ps) {
        next = MIN(next, cnt_guard(&a->ch10.cnta, now));
        next = MIN(next, cnt_next(&a->ch10.cnta, now, a->ch10.ocra));
    }
    if (a->ch11.cnt.period_ps) {
        next = MIN(next, cnt_guard(&a->ch11.cnt, now));
        next = MIN(next, cnt_next(&a->ch11.cnt, now, a->ch11.gra));
        next = MIN(next, cnt_next(&a->ch11.cnt, now, a->ch11.grb));
    }

    if (next == INT64_MAX) {
        timer_del(a->timer);
    } else {
        timer_mod(a->timer, MAX(next, now + 1));
    }
}

static void atu_update(SH705xATU *a)
{
    int64_t now = sh705x_now();

    atu_sweep(a, now);
    atu_irq(a);
    atu_schedule(a, now);
}

static void atu_timer_cb(void *opaque)
{
    atu_update(opaque);
}

/* ---------------------------------------------------------------------- */
/* Input pins                                                             */
/* ---------------------------------------------------------------------- */

static bool edge_match(int sel, int old, int level)
{
    /* sel: 1 rising, 2 falling, 3 both */
    if (old == level) {
        return false;
    }
    return (sel == 1 && level) || (sel == 2 && !level) || sel == 3;
}

static void ch10_edge(SH705xATU *a, int64_t now)
{
    uint32_t cnt = atu_cnt_get(&a->ch10.cnta, now);
    int mult = 1 << ((a->ch10.tior >> 4) & 3);

    a->ch10.icra = cnt;
    a->ch10.tsr |= 0x02;
    a->ch10.rldc = cnt - a->ch10.last_edge_cnt;
    a->ch10.tcntc = a->ch10.rldc;
    a->ch10.last_edge_cnt = cnt;
    if (a->ch10.last_edge_ns) {
        a->ch10.tooth_ns = now - a->ch10.last_edge_ns;
    }
    a->ch10.last_edge_ns = now;

    a->ch10.tcntb++;
    if (a->ch10.tcntb == a->ch10.ocrb) {
        a->ch10.tsr |= 0x04;
    }
    a->ch10.tcntd++;
    a->ch10.tcntf += mult;
    a->ch10.tcnte += mult;
    for (int i = 0; i < mult; i++) {
        a->ch10.tcntg++;
        if (a->ch10.tcntg == a->ch10.grg) {
            a->ch10.tsr |= 0x08;
        }
    }
    /* angle clocked channel 1/2 counters follow the new tooth period */
    update_clocks(a, now);
}

static void atu_pin_cb(void *opaque, int level)
{
    AtuPinCtx *ctx = opaque;
    SH705xATU *a = ctx->a;
    int id = ctx->id;
    int old = a->pin_level[id];
    int64_t now = sh705x_now();

    atu_sweep(a, now);
    a->pin_level[id] = level;

    if (id >= PIN_TI0 && id < PIN_TI0 + 4) {
        int i = id - PIN_TI0;
        if (edge_match((a->tior0 >> (2 * i)) & 3, old, level)) {
            uint32_t v = atu_cnt_get(&a->cnt0, now);
            if (i == 3) {
                a->icr0[3] = v;
            } else {
                a->icr0[i] = v;
            }
            a->tsr0 |= 1 << i;
        }
    } else if (id >= PIN_TIO1 && id < PIN_TIO1 + 16) {
        int c = (id - PIN_TIO1) / 8, i = (id - PIN_TIO1) % 8;
        int io = ch12_io(a, c, i);
        if (IO_IS_IC(io) && edge_match(io - 3, old, level)) {
            a->ch12[c].gr[i] = atu_cnt_get(&a->ch12[c].cnta, now);
            a->ch12[c].tsra |= 1 << i;
        }
    } else if (id >= PIN_TIO3 && id < PIN_TIO3 + 12) {
        int c = (id - PIN_TIO3) / 4, i = (id - PIN_TIO3) % 4;
        int io = ch345_io(a, c, i);
        if (IO_IS_IC(io) && edge_match(io - 3, old, level)) {
            a->ch345[c].gr[i] = atu_cnt_get(&a->ch345[c].cnt, now);
            a->tsr3 |= 1 << (5 * c + i);
        }
    } else if (id >= PIN_TI9 && id < PIN_TI9 + 6) {
        int i = id - PIN_TI9;
        uint8_t tcr = i < 2 ? a->ch9.tcra : i < 4 ? a->ch9.tcrb : a->ch9.tcrc;
        int sel = i & 1 ? (tcr >> 4) & 3 : tcr & 3;
        if (edge_match(sel, old, level)) {
            a->ch9.ecnt[i]++;
            if (a->ch9.ecnt[i] == a->ch9.gr[i]) {
                a->ch9.tsr |= 1 << i;
                a->ch9.ecnt[i] = 0;
            }
        }
    } else if (id == PIN_TI10) {
        int sel = a->ch10.tcr & 3;
        if ((a->tstr[1] & 0x80) && edge_match(sel ? sel : 1, old, level)) {
            ch10_edge(a, now);
        }
    } else if (id >= PIN_TIO11 && id < PIN_TIO11 + 2) {
        int i = id - PIN_TIO11;
        int io = i ? (a->ch11.tior >> 4) & 7 : a->ch11.tior & 7;
        if (IO_IS_IC(io) && edge_match(io - 3, old, level)) {
            uint16_t v = atu_cnt_get(&a->ch11.cnt, now);
            if (i) {
                a->ch11.grb = v;
            } else {
                a->ch11.gra = v;
            }
            a->ch11.tsr |= 1 << i;
        }
    }
    atu_irq(a);
    atu_schedule(a, now);
}

/* ---------------------------------------------------------------------- */
/* Register access                                                        */
/* ---------------------------------------------------------------------- */

#define HI(v)       ((uint16_t)(v) << 8)
#define B_HI(val)   ((uint8_t)((val) >> 8))
#define B_LO(val)   ((uint8_t)(val))

static uint16_t rd32h(uint32_t v, bool high)
{
    return high ? v >> 16 : v & 0xffff;
}

static uint32_t wr32h(uint32_t old, bool high, uint16_t val, uint16_t mask)
{
    if (high) {
        return (old & 0xffff) |
               ((uint32_t)sh705x_merge(old >> 16, val, mask) << 16);
    }
    return (old & 0xffff0000) | sh705x_merge(old & 0xffff, val, mask);
}

uint16_t sh705x_atu_read(SH705xATU *a, uint32_t addr)
{
    int64_t now = sh705x_now();
    uint32_t off = addr - 0xFFFFF400;

    atu_sweep(a, now);

    switch (off) {
    case 0x00:
        return HI(a->tstr[0]) | a->tstr[1];
    case 0x02:
        return HI(a->tstr[2]);
    case 0x04: case 0x06: case 0x08: case 0x0a:
        return HI(a->pscr[(off - 4) / 2]);
    /* channel 0 */
    case 0x20: case 0x22:
        return rd32h(a->icr0[3], off == 0x20);
    case 0x24:
        return HI(a->itvrr1);
    case 0x26:
        return HI(a->itvrr2a);
    case 0x28:
        return HI(a->itvrr2b);
    case 0x2a:
        return HI(a->tior0);
    case 0x2c:
        return a->tsr0;
    case 0x2e:
        return a->tier0;
    case 0x30: case 0x32:
        return rd32h(atu_cnt_get(&a->cnt0, now), off == 0x30);
    case 0x34: case 0x36: case 0x38: case 0x3a: case 0x3c: case 0x3e:
        return rd32h(a->icr0[(off - 0x34) / 4], !(off & 2));
    /* channels 3..5 shared */
    case 0x80:
        return a->tsr3;
    case 0x82:
        return a->tier3;
    case 0x84:
        return HI(a->tmdr);
    }

    /* channel 1 (0x40..0x67) and channel 2 (0x200..0x233) */
    if ((off >= 0x40 && off < 0x68) || (off >= 0x200 && off < 0x234)) {
        int c = off >= 0x200;
        uint32_t o = off - (c ? 0x200 : 0x40);
        uint32_t ctl = c ? 0x26 : 0x18;     /* offset of TIOR?B */

        if (o == 0) {
            return atu_cnt_get(&a->ch12[c].cnta, now);
        } else if (o == 2) {
            return atu_cnt_get(&a->ch12[c].cntb, now);
        } else if (o >= 4 && o < 0x14) {
            return a->ch12[c].gr[(o - 4) / 2];
        } else if (!c && o == 0x14) {
            return a->ch12[0].ocr[0];
        } else if (!c && o == 0x16) {
            return a->ch12[0].osbr;
        } else if (c && o >= 0x14 && o < 0x24) {
            return a->ch12[1].ocr[(o - 0x14) / 2];
        } else if (c && o == 0x24) {
            return a->ch12[1].osbr;
        } else if (o == ctl) {
            return HI(a->ch12[c].tior[1]) | a->ch12[c].tior[0];
        } else if (o == ctl + 2) {
            return HI(a->ch12[c].tior[3]) | a->ch12[c].tior[2];
        } else if (o == ctl + 4) {
            return HI(a->ch12[c].tcrb) | a->ch12[c].tcra;
        } else if (o == ctl + 6) {
            return a->ch12[c].tsra;
        } else if (o == ctl + 8) {
            return a->ch12[c].tsrb;
        } else if (o == ctl + 10) {
            return a->ch12[c].tiera;
        } else if (o == ctl + 12) {
            return a->ch12[c].tierb;
        } else if (!c && o == ctl + 14) {
            return HI(a->ch12[0].trgmdr);
        }
        return 0;
    }

    /* channels 3..5 */
    if (off >= 0xa0 && off < 0x100) {
        int c = (off - 0xa0) / 0x20;
        uint32_t o = (off - 0xa0) % 0x20;

        if (o == 0) {
            return atu_cnt_get(&a->ch345[c].cnt, now);
        } else if (o >= 2 && o < 0xa) {
            return a->ch345[c].gr[(o - 2) / 2];
        } else if (o == 0xa) {
            return HI(a->ch345[c].tiorb) | a->ch345[c].tiora;
        } else if (o == 0xc) {
            return HI(a->ch345[c].tcr);
        }
        return 0;
    }

    /* channels 6 (0x100) and 7 (0x180) */
    if ((off >= 0x100 && off < 0x128) || (off >= 0x180 && off < 0x1a6)) {
        int c = off >= 0x180;
        uint32_t o = off - (c ? 0x180 : 0x100);

        if (o < 0x08) {
            return atu_cnt_get(&a->ch67[c].cnt[o / 2], now);
        } else if (o < 0x10) {
            return a->ch67[c].cylr[(o - 8) / 2];
        } else if (o < 0x18) {
            return a->ch67[c].bfr[(o - 0x10) / 2];
        } else if (o < 0x20) {
            return a->ch67[c].dtr[(o - 0x18) / 2];
        } else if (o == 0x20) {
            return HI(a->ch67[c].tcrb) | a->ch67[c].tcra;
        } else if (o == 0x22) {
            return a->ch67[c].tsr;
        } else if (o == 0x24) {
            return a->ch67[c].tier;
        } else if (o == 0x26) {
            return HI(a->ch67[c].pmdr);
        }
        return 0;
    }

    /* channel 11 */
    if (off >= 0x1c0 && off < 0x1ce) {
        switch (off - 0x1c0) {
        case 0x0: return atu_cnt_get(&a->ch11.cnt, now);
        case 0x2: return a->ch11.gra;
        case 0x4: return a->ch11.grb;
        case 0x6: return HI(a->ch11.tior);
        case 0x8: return HI(a->ch11.tcr);
        case 0xa: return a->ch11.tsr;
        case 0xc: return a->ch11.tier;
        }
        return 0;
    }

    /* channel 8 */
    if (off >= 0x240 && off < 0x270) {
        uint32_t o = off - 0x240;
        if (o < 0x20) {
            return ch8_dcnt(a, o / 2, now);
        }
        switch (o) {
        case 0x20: return a->ch8.rldr;
        case 0x22: return a->ch8.tcnr;
        case 0x24: return a->ch8.otr;
        case 0x26: return a->ch8.dstr;
        case 0x28: return HI(a->ch8.tcr);
        case 0x2a: return a->ch8.tsr;
        case 0x2c: return a->ch8.tier;
        case 0x2e: return HI(a->ch8.rldenr);
        }
        return 0;
    }

    /* channel 9 */
    if (off >= 0x280 && off < 0x2a2) {
        uint32_t o = off - 0x280;
        if (o < 0x0c) {
            return HI(a->ch9.ecnt[o / 2]);
        } else if (o < 0x18) {
            return HI(a->ch9.gr[(o - 0x0c) / 2]);
        }
        switch (o) {
        case 0x18: return HI(a->ch9.tcra);
        case 0x1a: return HI(a->ch9.tcrb);
        case 0x1c: return HI(a->ch9.tcrc);
        case 0x1e: return a->ch9.tsr;
        case 0x20: return a->ch9.tier;
        }
        return 0;
    }

    /* channel 10 */
    if (off >= 0x2c0 && off < 0x2ec) {
        switch (off - 0x2c0) {
        case 0x00: case 0x02:
            return rd32h(atu_cnt_get(&a->ch10.cnta, now), off == 0x2c0);
        case 0x04: return HI(a->ch10.tcntb);
        case 0x06: return a->ch10.tcntc;
        case 0x08: return HI(a->ch10.tcntd);
        case 0x0a: return a->ch10.tcnte;
        case 0x0c: return a->ch10.tcntf;
        case 0x0e: return a->ch10.tcntg;
        case 0x10: case 0x12: return rd32h(a->ch10.icra, off == 0x2d0);
        case 0x14: case 0x16: return rd32h(a->ch10.ocra, off == 0x2d4);
        case 0x18: return HI(a->ch10.ocrb);
        case 0x1a: return a->ch10.rldc;
        case 0x1c: return a->ch10.grg;
        case 0x1e: return HI(a->ch10.tcnth);
        case 0x20: return HI(a->ch10.ncr);
        case 0x22: return HI(a->ch10.tior);
        case 0x24: return HI(a->ch10.tcr);
        case 0x26: return a->ch10.tcclr;
        case 0x28: return a->ch10.tsr;
        case 0x2a: return a->ch10.tier;
        }
        return 0;
    }

    qemu_log_mask(LOG_UNIMP, "sh705x-atu: read of unknown reg 0x%08x\n",
                  addr);
    return 0;
}

static void wr_hi8(uint8_t *reg, uint16_t val, uint16_t mask)
{
    if (mask & 0xff00) {
        *reg = B_HI(val);
    }
}

static void wr_pair8(uint8_t *hi, uint8_t *lo, uint16_t val, uint16_t mask)
{
    if (mask & 0xff00) {
        *hi = B_HI(val);
    }
    if (mask & 0x00ff) {
        *lo = B_LO(val);
    }
}

void sh705x_atu_write(SH705xATU *a, uint32_t addr, uint16_t val,
                      uint16_t mask)
{
    int64_t now = sh705x_now();
    uint32_t off = addr - 0xFFFFF400;
    uint16_t tmp;

    atu_sweep(a, now);

    switch (off) {
    case 0x00:
        wr_pair8(&a->tstr[0], &a->tstr[1], val, mask);
        update_clocks(a, now);
        goto out;
    case 0x02:
        wr_hi8(&a->tstr[2], val, mask);
        update_clocks(a, now);
        goto out;
    case 0x04: case 0x06: case 0x08: case 0x0a:
        wr_hi8(&a->pscr[(off - 4) / 2], val, mask);
        update_clocks(a, now);
        goto out;
    case 0x20: case 0x22:
        a->icr0[3] = wr32h(a->icr0[3], off == 0x20, val, mask);
        goto out;
    case 0x24:
        wr_hi8(&a->itvrr1, val, mask);
        goto out;
    case 0x26:
        wr_hi8(&a->itvrr2a, val, mask);
        goto out;
    case 0x28:
        wr_hi8(&a->itvrr2b, val, mask);
        goto out;
    case 0x2a:
        wr_hi8(&a->tior0, val, mask);
        goto out;
    case 0x2c:
        a->tsr0 = sh705x_w0c(a->tsr0, val, mask) & 0xff;
        goto out;
    case 0x2e:
        a->tier0 = sh705x_merge(a->tier0, val, mask) & 0x1f;
        goto out;
    case 0x30: case 0x32:
        atu_cnt_set(&a->cnt0, now,
                    wr32h(atu_cnt_get(&a->cnt0, now), off == 0x30, val,
                          mask));
        goto out;
    case 0x34: case 0x36: case 0x38: case 0x3a: case 0x3c: case 0x3e:
        a->icr0[(off - 0x34) / 4] = wr32h(a->icr0[(off - 0x34) / 4],
                                          !(off & 2), val, mask);
        goto out;
    case 0x80:
        a->tsr3 = sh705x_w0c(a->tsr3, val, mask) & 0x7fff;
        goto out;
    case 0x82:
        a->tier3 = sh705x_merge(a->tier3, val, mask) & 0x7fff;
        goto out;
    case 0x84:
        wr_hi8(&a->tmdr, val, mask);
        goto out;
    }

    if ((off >= 0x40 && off < 0x68) || (off >= 0x200 && off < 0x234)) {
        int c = off >= 0x200;
        uint32_t o = off - (c ? 0x200 : 0x40);
        uint32_t ctl = c ? 0x26 : 0x18;

        if (o == 0) {
            tmp = sh705x_merge(atu_cnt_get(&a->ch12[c].cnta, now), val, mask);
            atu_cnt_set(&a->ch12[c].cnta, now, tmp);
        } else if (o == 2) {
            tmp = sh705x_merge(atu_cnt_get(&a->ch12[c].cntb, now), val, mask);
            atu_cnt_set(&a->ch12[c].cntb, now, tmp);
        } else if (o >= 4 && o < 0x14) {
            uint16_t *r = &a->ch12[c].gr[(o - 4) / 2];
            *r = sh705x_merge(*r, val, mask);
        } else if (!c && o == 0x14) {
            a->ch12[0].ocr[0] = sh705x_merge(a->ch12[0].ocr[0], val, mask);
        } else if (!c && o == 0x16) {
            a->ch12[0].osbr = sh705x_merge(a->ch12[0].osbr, val, mask);
        } else if (c && o >= 0x14 && o < 0x24) {
            uint16_t *r = &a->ch12[1].ocr[(o - 0x14) / 2];
            *r = sh705x_merge(*r, val, mask);
        } else if (c && o == 0x24) {
            a->ch12[1].osbr = sh705x_merge(a->ch12[1].osbr, val, mask);
        } else if (o == ctl) {
            wr_pair8(&a->ch12[c].tior[1], &a->ch12[c].tior[0], val, mask);
        } else if (o == ctl + 2) {
            wr_pair8(&a->ch12[c].tior[3], &a->ch12[c].tior[2], val, mask);
        } else if (o == ctl + 4) {
            wr_pair8(&a->ch12[c].tcrb, &a->ch12[c].tcra, val, mask);
            update_clocks(a, now);
        } else if (o == ctl + 6) {
            a->ch12[c].tsra = sh705x_w0c(a->ch12[c].tsra, val, mask) & 0x1ff;
        } else if (o == ctl + 8) {
            a->ch12[c].tsrb = sh705x_w0c(a->ch12[c].tsrb, val, mask) & 0x1ff;
        } else if (o == ctl + 10) {
            a->ch12[c].tiera = sh705x_merge(a->ch12[c].tiera, val, mask);
        } else if (o == ctl + 12) {
            a->ch12[c].tierb = sh705x_merge(a->ch12[c].tierb, val, mask);
        } else if (!c && o == ctl + 14) {
            wr_hi8(&a->ch12[0].trgmdr, val, mask);
        }
        goto out;
    }

    if (off >= 0xa0 && off < 0x100) {
        int c = (off - 0xa0) / 0x20;
        uint32_t o = (off - 0xa0) % 0x20;

        if (o == 0) {
            tmp = sh705x_merge(atu_cnt_get(&a->ch345[c].cnt, now), val, mask);
            atu_cnt_set(&a->ch345[c].cnt, now, tmp);
        } else if (o >= 2 && o < 0xa) {
            uint16_t *r = &a->ch345[c].gr[(o - 2) / 2];
            *r = sh705x_merge(*r, val, mask);
        } else if (o == 0xa) {
            wr_pair8(&a->ch345[c].tiorb, &a->ch345[c].tiora, val, mask);
        } else if (o == 0xc) {
            wr_hi8(&a->ch345[c].tcr, val, mask);
            update_clocks(a, now);
        }
        goto out;
    }

    if ((off >= 0x100 && off < 0x128) || (off >= 0x180 && off < 0x1a6)) {
        int c = off >= 0x180;
        uint32_t o = off - (c ? 0x180 : 0x100);

        if (o < 0x08) {
            AtuCounter *cn = &a->ch67[c].cnt[o / 2];
            atu_cnt_set(cn, now, sh705x_merge(atu_cnt_get(cn, now), val,
                                              mask));
        } else if (o < 0x10) {
            uint16_t *r = &a->ch67[c].cylr[(o - 8) / 2];
            *r = sh705x_merge(*r, val, mask);
        } else if (o < 0x18) {
            uint16_t *r = &a->ch67[c].bfr[(o - 0x10) / 2];
            *r = sh705x_merge(*r, val, mask);
        } else if (o < 0x20) {
            uint16_t *r = &a->ch67[c].dtr[(o - 0x18) / 2];
            *r = sh705x_merge(*r, val, mask);
        } else if (o == 0x20) {
            wr_pair8(&a->ch67[c].tcrb, &a->ch67[c].tcra, val, mask);
            update_clocks(a, now);
        } else if (o == 0x22) {
            a->ch67[c].tsr = sh705x_w0c(a->ch67[c].tsr, val, mask) & 0xff;
        } else if (o == 0x24) {
            a->ch67[c].tier = sh705x_merge(a->ch67[c].tier, val, mask) & 0xf;
        } else if (o == 0x26) {
            wr_hi8(&a->ch67[c].pmdr, val, mask);
        }
        goto out;
    }

    if (off >= 0x1c0 && off < 0x1ce) {
        switch (off - 0x1c0) {
        case 0x0:
            tmp = sh705x_merge(atu_cnt_get(&a->ch11.cnt, now), val, mask);
            atu_cnt_set(&a->ch11.cnt, now, tmp);
            break;
        case 0x2:
            a->ch11.gra = sh705x_merge(a->ch11.gra, val, mask);
            break;
        case 0x4:
            a->ch11.grb = sh705x_merge(a->ch11.grb, val, mask);
            break;
        case 0x6:
            wr_hi8(&a->ch11.tior, val, mask);
            break;
        case 0x8:
            wr_hi8(&a->ch11.tcr, val, mask);
            update_clocks(a, now);
            break;
        case 0xa:
            a->ch11.tsr = sh705x_w0c(a->ch11.tsr, val, mask) & 0x103;
            break;
        case 0xc:
            a->ch11.tier = sh705x_merge(a->ch11.tier, val, mask) & 0x103;
            break;
        }
        goto out;
    }

    if (off >= 0x240 && off < 0x270) {
        uint32_t o = off - 0x240;
        if (o < 0x20) {
            int n = o / 2;
            uint16_t v = sh705x_merge(ch8_dcnt(a, n, now), val, mask);
            a->ch8.dcnt[n] = v;
            if (a->ch8.dstr & (1 << n)) {
                a->ch8.start_ns[n] = now;
                a->ch8.start_val[n] = v;
                if (!v) {
                    ch8_stop(a, n, now);
                }
            }
            goto out;
        }
        switch (o) {
        case 0x20:
            a->ch8.rldr = sh705x_merge(a->ch8.rldr, val, mask);
            break;
        case 0x22:
            a->ch8.tcnr = sh705x_merge(a->ch8.tcnr, val, mask);
            break;
        case 0x24:
            a->ch8.otr = sh705x_merge(a->ch8.otr, val, mask);
            break;
        case 0x26:
            for (int n = 0; n < 16; n++) {
                if ((mask & val & (1 << n)) && !(a->ch8.dstr & (1 << n))) {
                    if (a->ch8.rldenr & 0x80) {
                        a->ch8.dcnt[n] = a->ch8.rldr;
                    }
                    ch8_start(a, n, now);
                }
            }
            break;
        case 0x28:
            /* freeze running counters before changing their clock */
            for (int n = 0; n < 16; n++) {
                if (a->ch8.dstr & (1 << n)) {
                    a->ch8.start_val[n] = ch8_dcnt(a, n, now);
                    a->ch8.start_ns[n] = now;
                }
            }
            wr_hi8(&a->ch8.tcr, val, mask);
            break;
        case 0x2a:
            a->ch8.tsr = sh705x_w0c(a->ch8.tsr, val, mask);
            break;
        case 0x2c:
            a->ch8.tier = sh705x_merge(a->ch8.tier, val, mask);
            break;
        case 0x2e:
            wr_hi8(&a->ch8.rldenr, val, mask);
            break;
        }
        goto out;
    }

    if (off >= 0x280 && off < 0x2a2) {
        uint32_t o = off - 0x280;
        if (o < 0x0c) {
            wr_hi8(&a->ch9.ecnt[o / 2], val, mask);
        } else if (o < 0x18) {
            wr_hi8(&a->ch9.gr[(o - 0x0c) / 2], val, mask);
        } else if (o == 0x18) {
            wr_hi8(&a->ch9.tcra, val, mask);
        } else if (o == 0x1a) {
            wr_hi8(&a->ch9.tcrb, val, mask);
        } else if (o == 0x1c) {
            wr_hi8(&a->ch9.tcrc, val, mask);
        } else if (o == 0x1e) {
            a->ch9.tsr = sh705x_w0c(a->ch9.tsr, val, mask) & 0x3f;
        } else if (o == 0x20) {
            a->ch9.tier = sh705x_merge(a->ch9.tier, val, mask) & 0x3f;
        }
        goto out;
    }

    if (off >= 0x2c0 && off < 0x2ec) {
        switch (off - 0x2c0) {
        case 0x00: case 0x02:
            atu_cnt_set(&a->ch10.cnta, now,
                        wr32h(atu_cnt_get(&a->ch10.cnta, now), off == 0x2c0,
                              val, mask));
            break;
        case 0x04: wr_hi8(&a->ch10.tcntb, val, mask); break;
        case 0x06: a->ch10.tcntc = sh705x_merge(a->ch10.tcntc, val, mask);
            break;
        case 0x08: wr_hi8(&a->ch10.tcntd, val, mask); break;
        case 0x0a: a->ch10.tcnte = sh705x_merge(a->ch10.tcnte, val, mask);
            break;
        case 0x0c: a->ch10.tcntf = sh705x_merge(a->ch10.tcntf, val, mask);
            break;
        case 0x0e: a->ch10.tcntg = sh705x_merge(a->ch10.tcntg, val, mask);
            break;
        case 0x10: case 0x12:
            a->ch10.icra = wr32h(a->ch10.icra, off == 0x2d0, val, mask);
            break;
        case 0x14: case 0x16:
            a->ch10.ocra = wr32h(a->ch10.ocra, off == 0x2d4, val, mask);
            break;
        case 0x18: wr_hi8(&a->ch10.ocrb, val, mask); break;
        case 0x1a: a->ch10.rldc = sh705x_merge(a->ch10.rldc, val, mask);
            break;
        case 0x1c: a->ch10.grg = sh705x_merge(a->ch10.grg, val, mask); break;
        case 0x1e: wr_hi8(&a->ch10.tcnth, val, mask); break;
        case 0x20: wr_hi8(&a->ch10.ncr, val, mask); break;
        case 0x22: wr_hi8(&a->ch10.tior, val, mask); break;
        case 0x24: wr_hi8(&a->ch10.tcr, val, mask); break;
        case 0x26: a->ch10.tcclr = sh705x_merge(a->ch10.tcclr, val, mask);
            break;
        case 0x28: a->ch10.tsr = sh705x_w0c(a->ch10.tsr, val, mask) & 0xf;
            break;
        case 0x2a: a->ch10.tier = sh705x_merge(a->ch10.tier, val, mask) & 0x1f;
            break;
        }
        goto out;
    }

    qemu_log_mask(LOG_UNIMP, "sh705x-atu: write of unknown reg 0x%08x\n",
                  addr);
out:
    atu_irq(a);
    atu_schedule(a, now);
}

/* ---------------------------------------------------------------------- */
/* Init / reset                                                           */
/* ---------------------------------------------------------------------- */

void sh705x_atu_reset(SH705xATU *a)
{
    SH705xState *s = a->soc;
    QEMUTimer *t = a->timer;
    int64_t now = sh705x_now();

    memset(a, 0, sizeof(*a));
    a->soc = s;
    a->timer = t;
    timer_del(t);

    atu_cnt_init(&a->cnt0, 0xffffffff);
    for (int c = 0; c < 2; c++) {
        atu_cnt_init(&a->ch12[c].cnta, 0xffff);
        atu_cnt_init(&a->ch12[c].cntb, 0xffff);
        for (int i = 0; i < 8; i++) {
            a->ch12[c].gr[i] = 0xffff;
            a->ch12[c].ocr[i] = 0xffff;
        }
        for (int x = 0; x < 4; x++) {
            atu_cnt_init(&a->ch67[c].cnt[x], 0xffff);
            a->ch67[c].cylr[x] = 0xffff;
            a->ch67[c].bfr[x] = 0xffff;
            a->ch67[c].dtr[x] = 0xffff;
        }
    }
    for (int c = 0; c < 3; c++) {
        atu_cnt_init(&a->ch345[c].cnt, 0xffff);
        for (int i = 0; i < 4; i++) {
            a->ch345[c].gr[i] = 0xffff;
        }
    }
    for (int i = 0; i < 6; i++) {
        a->ch9.gr[i] = 0xff;
    }
    atu_cnt_init(&a->ch10.cnta, 0xffffffff);
    a->ch10.ocra = 0xffffffff;
    a->ch10.ocrb = 0xff;
    a->ch10.grg = 0xffff;
    atu_cnt_init(&a->ch11.cnt, 0xffff);
    a->ch11.gra = a->ch11.grb = 0xffff;
    a->last_sweep_ns = now;

    for (int i = 0; i < PIN_COUNT; i++) {
        if (atu_pins[i]) {
            a->pin_level[i] = ecu_pin_level(atu_pins[i]);
        }
    }
}

static void atu_add_pin(SH705xATU *a, int id, const char *fmt, int n)
{
    char name[16];
    AtuPinCtx *ctx = g_new(AtuPinCtx, 1);

    snprintf(name, sizeof(name), fmt, n);
    ctx->a = a;
    ctx->id = id;
    atu_pins[id] = ecu_pin(name);
    ecu_pin_set_input_handler(atu_pins[id], atu_pin_cb, ctx);
}

void sh705x_atu_init(SH705xState *s)
{
    SH705xATU *a = &s->atu;
    char name[16];

    a->soc = s;
    a->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, atu_timer_cb, a);

    for (int i = 0; i < 4; i++) {
        atu_add_pin(a, PIN_TI0 + i, "TI0%c", 'A' + i);
    }
    for (int i = 0; i < 8; i++) {
        atu_add_pin(a, PIN_TIO1 + i, "TIO1%c", 'A' + i);
        atu_add_pin(a, PIN_TIO2 + i, "TIO2%c", 'A' + i);
    }
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < 4; i++) {
            snprintf(name, sizeof(name), "TIO%d%%c", 3 + c);
            atu_add_pin(a, PIN_TIO3 + 4 * c + i, name, 'A' + i);
        }
    }
    for (int i = 0; i < 6; i++) {
        atu_add_pin(a, PIN_TI9 + i, "TI9%c", 'A' + i);
    }
    atu_add_pin(a, PIN_TI10, "TI10", 0);
    atu_add_pin(a, PIN_TIO11, "TIO11%c", 'A');
    atu_add_pin(a, PIN_TIO11 + 1, "TIO11%c", 'B');

    for (int c = 0; c < 2; c++) {
        for (int x = 0; x < 4; x++) {
            snprintf(name, sizeof(name), "TO%d%c", 6 + c, 'A' + x);
            to6_pins[c][x] = ecu_pin(name);
        }
    }
    for (int n = 0; n < 16; n++) {
        snprintf(name, sizeof(name), "TO8%c", 'A' + n);
        to8_pins[n] = ecu_pin(name);
    }
    sh705x_atu_reset(a);
}
