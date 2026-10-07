/*
 * Renesas SH705x automotive microcontrollers (SH7052/SH7054/SH7055/SH7058)
 *
 * SoC container, I/O register dispatcher, interrupt controller, watchdog,
 * compare-match timer, I/O ports, A/D converters and SCI.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "system/runstate.h"
#include "system/memory.h"
#include "system/reset.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "hw/sh4/sh705x.h"

/* ---------------------------------------------------------------------- */
/* Time based counters                                                    */
/* ---------------------------------------------------------------------- */

int64_t sh705x_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

void atu_cnt_init(AtuCounter *c, uint32_t mask)
{
    memset(c, 0, sizeof(*c));
    c->mask = mask;
}

static uint64_t atu_cnt_ticks(AtuCounter *c, int64_t now)
{
    if (!c->period_ps || now <= c->base_ns) {
        return 0;
    }
    return (uint64_t)(now - c->base_ns) * 1000 / c->period_ps;
}

uint32_t atu_cnt_get(AtuCounter *c, int64_t now)
{
    return (c->base + atu_cnt_ticks(c, now)) & c->mask;
}

void atu_cnt_set(AtuCounter *c, int64_t now, uint32_t val)
{
    c->base = val & c->mask;
    c->base_ns = now;
    c->last = c->base;
}

void atu_cnt_set_period(AtuCounter *c, int64_t now, int64_t period_ps)
{
    if (c->period_ps == period_ps) {
        return;
    }
    c->base = atu_cnt_get(c, now);
    c->base_ns = now;
    c->period_ps = period_ps;
}

/* Absolute time (ns) at which the counter next equals @target. */
int64_t atu_cnt_time_to(AtuCounter *c, int64_t now, uint32_t target)
{
    uint64_t cur_ticks, delta;
    uint32_t cur;

    if (!c->period_ps) {
        return INT64_MAX;
    }
    cur_ticks = atu_cnt_ticks(c, now);
    cur = (c->base + cur_ticks) & c->mask;
    delta = (target - cur) & c->mask;
    if (delta == 0) {
        delta = (uint64_t)c->mask + 1;
    }
    return c->base_ns +
           (int64_t)(((cur_ticks + delta) * c->period_ps + 999) / 1000);
}

/* Did the counter step onto @v while moving from @from to @to? */
bool atu_cnt_passed(AtuCounter *c, uint32_t from, uint32_t to, uint32_t v)
{
    uint32_t moved = (to - from) & c->mask;

    if (moved == 0) {
        return false;
    }
    return ((v - from) & c->mask) != 0 && ((v - from) & c->mask) <= moved;
}

/* ---------------------------------------------------------------------- */
/* Interrupt controller                                                   */
/* ---------------------------------------------------------------------- */

static int vec_priority(SH705xState *s, int vec)
{
    int n;

    if (vec == SH705X_VEC_NMI) {
        return 16;
    }
    n = s->vec_ipr[vec];
    if (n < 0) {
        return 0;
    }
    return (s->ipr[n / 4] >> (12 - 4 * (n % 4))) & 0xf;
}

static int sh705x_irq_query(void *opaque, int imask, int *level)
{
    SH705xState *s = opaque;
    int best = -1, best_prio = imask;

    for (int v = 0; v < SH705X_NUM_VECTORS; v++) {
        if (s->pending[v]) {
            int p = vec_priority(s, v);
            if (p > best_prio || (v == SH705X_VEC_NMI && best_prio < 16)) {
                best = v;
                best_prio = p;
            }
        }
    }
    if (best >= 0) {
        *level = best_prio > 15 ? 15 : best_prio;
    }
    return best;
}

static void sh705x_update_irq(SH705xState *s)
{
    CPUState *cs = CPU(s->cpu);
    int level;

    if (sh705x_irq_query(s, 0, &level) >= 0) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

void sh705x_set_irq(SH705xState *s, int vec, bool level)
{
    if (s->pending[vec] == level) {
        return;
    }
    s->pending[vec] = level;
    sh705x_update_irq(s);
}

static void intc_irq_pins_update(SH705xState *s)
{
    for (int i = 0; i < 8; i++) {
        bool edge = s->icr & (0x80 >> i);
        bool req = edge ? (s->isr & (0x80 >> i)) : !s->irq_level[i];
        sh705x_set_irq(s, SH705X_VEC_IRQ0 + i, req);
    }
}

typedef struct IrqPinCtx {
    SH705xState *s;
    int n;
} IrqPinCtx;

static void intc_irq_pin_cb(void *opaque, int level)
{
    IrqPinCtx *ctx = opaque;
    SH705xState *s = ctx->s;
    int n = ctx->n;

    if ((s->icr & (0x80 >> n)) && s->irq_level[n] && !level) {
        s->isr |= 0x80 >> n;    /* falling edge */
    }
    s->irq_level[n] = level;
    intc_irq_pins_update(s);
}

static void intc_nmi_cb(void *opaque, int level)
{
    SH705xState *s = opaque;
    bool rising = s->icr & 0x0100;

    if ((rising && !s->nmi_level && level) ||
        (!rising && s->nmi_level && !level)) {
        sh705x_set_irq(s, SH705X_VEC_NMI, true);
    }
    s->nmi_level = level;
}

static void intc_init_table(SH705xState *s)
{
    static const struct { int first, last, nibble; } map[] = {
        { 64, 64, 0 }, { 65, 65, 1 }, { 66, 66, 2 }, { 67, 67, 3 },
        { 68, 68, 4 }, { 69, 69, 5 }, { 70, 70, 6 }, { 71, 71, 7 },
        { 72, 75, 8 }, { 76, 79, 9 }, { 80, 83, 10 }, { 84, 87, 11 },
        { 88, 91, 12 }, { 92, 95, 13 }, { 96, 99, 14 }, { 100, 103, 15 },
        { 104, 107, 16 }, { 108, 111, 17 }, { 112, 115, 18 },
        { 116, 119, 19 }, { 120, 123, 20 }, { 124, 127, 21 },
        { 128, 131, 22 }, { 132, 135, 23 }, { 136, 139, 24 },
        { 140, 143, 25 }, { 144, 147, 26 }, { 148, 151, 27 },
        { 152, 155, 28 }, { 156, 159, 29 }, { 160, 163, 30 },
        { 164, 167, 31 }, { 168, 171, 32 }, { 172, 175, 33 },
        { 176, 179, 34 }, { 180, 183, 35 }, { 184, 187, 36 },
        { 188, 191, 37 }, { 192, 195, 38 }, { 196, 199, 39 },
        { 200, 203, 40 }, { 204, 207, 41 }, { 208, 211, 42 },
        { 212, 215, 43 }, { 216, 219, 44 }, { 220, 223, 45 },
        { 224, 227, 46 }, { 228, 231, 47 },
    };

    memset(s->vec_ipr, -1, sizeof(s->vec_ipr));
    for (size_t i = 0; i < ARRAY_SIZE(map); i++) {
        for (int v = map[i].first; v <= map[i].last; v++) {
            s->vec_ipr[v] = map[i].nibble;
        }
    }
}

static uint16_t intc_read(SH705xState *s, uint32_t off)
{
    if (off < 0x18) {
        return s->ipr[off / 2];
    } else if (off == 0x18) {
        return (s->icr & 0x01ff) | (s->nmi_level ? 0x8000 : 0);
    } else if (off == 0x1a) {
        return s->isr;
    }
    return 0;
}

static void intc_write(SH705xState *s, uint32_t off, uint16_t val,
                       uint16_t mask)
{
    if (off < 0x18) {
        s->ipr[off / 2] = sh705x_merge(s->ipr[off / 2], val, mask);
    } else if (off == 0x18) {
        s->icr = sh705x_merge(s->icr, val, mask) & 0x01ff;
    } else if (off == 0x1a) {
        s->isr = sh705x_w0c(s->isr, val, mask) & 0xff;
    }
    intc_irq_pins_update(s);
    sh705x_update_irq(s);
}

/* ---------------------------------------------------------------------- */
/* Watchdog timer (0xFFFFEC10)                                            */
/* ---------------------------------------------------------------------- */

static void wdt_schedule(SH705xState *s)
{
    static const int div[8] = { 2, 64, 128, 256, 512, 1024, 4096, 8192 };
    SH705xWDT *w = &s->wdt;
    int64_t now = sh705x_now();

    if (!(w->tcsr & 0x20)) {
        atu_cnt_set_period(&w->cnt, now, 0);
        timer_del(w->timer);
        return;
    }
    atu_cnt_set_period(&w->cnt, now,
                       1000000000000LL / s->pclk_hz * div[w->tcsr & 7]);
    timer_mod(w->timer, atu_cnt_time_to(&w->cnt, now, 0));
}

static void wdt_timer_cb(void *opaque)
{
    SH705xState *s = opaque;
    SH705xWDT *w = &s->wdt;

    w->tcsr |= 0x80;
    if (w->tcsr & 0x40) {
        w->rstcsr |= 0x80;
        if ((w->rstcsr & 0x40) && s->wdt_reset) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh705x: watchdog reset\n");
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            return;
        }
    } else {
        sh705x_set_irq(s, SH705X_VEC_WDT_ITI, true);
    }
    wdt_schedule(s);
}

static uint16_t wdt_read(SH705xState *s, uint32_t off)
{
    SH705xWDT *w = &s->wdt;

    if (off == 0) {
        return (w->tcsr | 0x18) << 8 | atu_cnt_get(&w->cnt, sh705x_now());
    }
    return 0xff00 | w->rstcsr | 0x1f;
}

static void wdt_write(SH705xState *s, uint32_t off, uint16_t val,
                      uint16_t mask)
{
    SH705xWDT *w = &s->wdt;
    uint8_t key = val >> 8, data = val & 0xff;

    if (mask != 0xffff) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x: WDT needs word writes\n");
        return;
    }
    if (off == 0) {
        if (key == 0xa5) {
            /* OVF can only be cleared */
            w->tcsr = (data & 0x67) | (w->tcsr & data & 0x80);
            if (!(w->tcsr & 0x80)) {
                sh705x_set_irq(s, SH705X_VEC_WDT_ITI, false);
            }
        } else if (key == 0x5a) {
            atu_cnt_set(&w->cnt, sh705x_now(), data);
        }
    } else {
        if (key == 0xa5) {
            if (!(data & 0x80)) {
                w->rstcsr &= ~0x80;
            }
        } else if (key == 0x5a) {
            w->rstcsr = (w->rstcsr & 0x80) | (data & 0x60);
        }
    }
    wdt_schedule(s);
}

/* ---------------------------------------------------------------------- */
/* Compare match timer (0xFFFFF710)                                       */
/* ---------------------------------------------------------------------- */

static void cmt_update(SH705xState *s)
{
    SH705xCMT *c = &s->cmt;
    int64_t now = sh705x_now(), next = INT64_MAX;

    for (int i = 0; i < 2; i++) {
        int64_t period = 0;

        if (c->cmstr & (1 << i)) {
            period = 1000000000000LL / s->pclk_hz *
                     (8 << (2 * (c->ch[i].cmcsr & 3)));
        }
        atu_cnt_set_period(&c->ch[i].cnt, now, period);
        if (period) {
            next = MIN(next, atu_cnt_time_to(&c->ch[i].cnt, now,
                                             c->ch[i].cmcor));
        }
        sh705x_set_irq(s, i ? SH705X_VEC_CMTI1 : SH705X_VEC_CMTI0,
                       (c->ch[i].cmcsr & 0xc0) == 0xc0);
    }
    if (next == INT64_MAX) {
        timer_del(c->timer);
    } else {
        timer_mod(c->timer, next);
    }
}

static void cmt_timer_cb(void *opaque)
{
    SH705xState *s = opaque;
    SH705xCMT *c = &s->cmt;
    int64_t now = sh705x_now();

    for (int i = 0; i < 2; i++) {
        AtuCounter *cnt = &c->ch[i].cnt;
        int64_t t;

        if (!(c->cmstr & (1 << i)) || !cnt->period_ps) {
            continue;
        }
        /* compare match(es) since the counter was last (re)started */
        while ((t = atu_cnt_time_to(cnt, cnt->base_ns, c->ch[i].cmcor))
               <= now) {
            int64_t cycle = (int64_t)(c->ch[i].cmcor + 1) *
                            cnt->period_ps / 1000;
            c->ch[i].cmcsr |= 0x80;
            if (cycle > 0 && now - t > 16 * cycle) {
                t += (now - t) / cycle * cycle;
            }
            atu_cnt_set(cnt, t, 0);
        }
    }
    cmt_update(s);
}

static uint16_t cmt_read(SH705xState *s, uint32_t off)
{
    SH705xCMT *c = &s->cmt;
    int i = off >= 8;

    switch (off) {
    case 0:
        return c->cmstr;
    case 2: case 8:
        return c->ch[i].cmcsr;
    case 4: case 10:
        return atu_cnt_get(&c->ch[i].cnt, sh705x_now());
    case 6: case 12:
        return c->ch[i].cmcor;
    }
    return 0;
}

static void cmt_write(SH705xState *s, uint32_t off, uint16_t val,
                      uint16_t mask)
{
    SH705xCMT *c = &s->cmt;
    int i = off >= 8;

    switch (off) {
    case 0:
        c->cmstr = sh705x_merge(c->cmstr, val, mask) & 3;
        break;
    case 2: case 8:
        c->ch[i].cmcsr = (sh705x_w0c(c->ch[i].cmcsr, val, mask) & 0x80) |
                         (sh705x_merge(c->ch[i].cmcsr, val, mask) & 0x43);
        break;
    case 4: case 10:
        atu_cnt_set(&c->ch[i].cnt, sh705x_now(),
                    sh705x_merge(atu_cnt_get(&c->ch[i].cnt, sh705x_now()),
                                 val, mask));
        break;
    case 6: case 12:
        c->ch[i].cmcor = sh705x_merge(c->ch[i].cmcor, val, mask);
        break;
    }
    cmt_update(s);
}

/* ---------------------------------------------------------------------- */
/* I/O ports (0xFFFFF720..)                                               */
/* ---------------------------------------------------------------------- */

static const struct {
    const char *name;
    uint16_t ior, dr, pins;
} port_desc[SH705X_NUM_PORTS] = {
    { "A", 0x720, 0x726, 0xffff }, { "B", 0x730, 0x738, 0xffff },
    { "C", 0x73a, 0x73e, 0x001f }, { "D", 0x740, 0x746, 0x3fff },
    { "E", 0x750, 0x754, 0xffff }, { "F", 0x748, 0x74e, 0xffff },
    { "G", 0x760, 0x764, 0x000f }, { "H", 0x728, 0x72c, 0xffff },
    { "J", 0x766, 0x76c, 0xffff }, { "K", 0x770, 0x778, 0xffff },
    { "L", 0x756, 0x75e, 0x3fff },
};

static void port_drive(SH705xPort *p)
{
    for (int b = 0; b < 16; b++) {
        if ((p->pins & p->ior) & (1 << b)) {
            ecu_pin_mcu_drive(p->pin[b], (p->dr >> b) & 1);
        }
    }
}

static uint16_t port_read_dr(SH705xPort *p)
{
    uint16_t v = p->dr & p->ior;

    for (int b = 0; b < 16; b++) {
        if ((p->pins & ~p->ior) & (1 << b)) {
            v |= ecu_pin_level(p->pin[b]) << b;
        }
    }
    return v & p->pins;
}

/* returns true if handled */
static bool port_access(SH705xState *s, uint32_t off, bool write,
                        uint16_t val, uint16_t mask, uint16_t *ret)
{
    for (int i = 0; i < SH705X_NUM_PORTS; i++) {
        SH705xPort *p = &s->port[i];

        if (off == port_desc[i].dr) {
            if (write) {
                p->dr = sh705x_merge(p->dr, val, mask);
                port_drive(p);
            } else {
                *ret = port_read_dr(p);
            }
            return true;
        }
        if (off == port_desc[i].ior) {
            if (write) {
                p->ior = sh705x_merge(p->ior, val, mask) & p->pins;
                port_drive(p);
            } else {
                *ret = p->ior;
            }
            return true;
        }
    }
    return false;
}

/* ---------------------------------------------------------------------- */
/* A/D converters                                                         */
/* ---------------------------------------------------------------------- */

#define ADCSR_ADF   0x80
#define ADCSR_ADIE  0x40
#define ADCR_TRGE   0x80
#define ADCR_CKS    0x40
#define ADCR_ADST   0x20
#define ADCR_ADCS   0x10

static void adc_irq(SH705xADC *a)
{
    static const int vec[3] = {
        SH705X_VEC_ADI0, SH705X_VEC_ADI1, SH705X_VEC_ADI2
    };
    sh705x_set_irq(a->soc, vec[a->index],
                   (a->adcsr & (ADCSR_ADF | ADCSR_ADIE)) ==
                   (ADCSR_ADF | ADCSR_ADIE));
}

static int64_t adc_conv_ns(SH705xADC *a)
{
    int states = (a->adcr & ADCR_CKS) ? 66 : 134;

    return (int64_t)states * 1000000000LL / a->soc->pclk_hz;
}

static int adc_first(SH705xADC *a)
{
    int ch = a->adcsr & 0xf;

    switch ((a->adcsr >> 4) & 3) {
    case 0:
        return ch;
    case 1:
        return ch & ~3;
    case 2:
        return ch & ~7;
    default:
        return 0;
    }
}

static void adc_convert(SH705xADC *a, int ch)
{
    char name[8];
    double v;

    if (ch >= a->nchan) {
        return;
    }
    snprintf(name, sizeof(name), "AN%d", a->first_chan + ch);
    v = ecu_analog_read(name);
    a->addr[ch] = ecu_adc_convert(v, a->soc->avref_mv / 1000.0, 10) << 6;
}

static void adc_timer_cb(void *opaque)
{
    SH705xADC *a = opaque;
    int last = a->adcsr & 0xf;

    if (!(a->adcr & ADCR_ADST)) {
        return;
    }
    adc_convert(a, a->cur);
    if (a->cur < last && ((a->adcsr >> 4) & 3)) {
        a->cur++;
        timer_mod(a->timer, sh705x_now() + adc_conv_ns(a));
        return;
    }
    /* end of single conversion or of one scan */
    a->adcsr |= ADCSR_ADF;
    adc_irq(a);
    if (((a->adcsr >> 4) & 3) && (a->adcr & ADCR_ADCS)) {
        a->cur = adc_first(a);
        timer_mod(a->timer, sh705x_now() + adc_conv_ns(a));
    } else {
        a->adcr &= ~ADCR_ADST;
    }
}

static void adc_start(SH705xADC *a)
{
    a->adcr |= ADCR_ADST;
    a->cur = adc_first(a);
    timer_mod(a->timer, sh705x_now() + adc_conv_ns(a));
}

void sh705x_adc_trigger(SH705xState *s)
{
    for (int i = 0; i < s->nadc; i++) {
        SH705xADC *a = &s->adc[i];
        if ((a->adcr & ADCR_TRGE) && !(a->adcr & ADCR_ADST)) {
            adc_start(a);
        }
    }
}

static uint8_t adc_readb(SH705xADC *a, int reg)
{
    /* reg: 0..23 data bytes, 24 ADCSR, 25 ADCR */
    if (reg < 24) {
        uint16_t d = a->addr[reg / 2];
        return reg & 1 ? d & 0xff : d >> 8;
    } else if (reg == 24) {
        return a->adcsr;
    } else if (reg == 25) {
        return a->adcr | 0x0f;
    }
    return 0xff;
}

static void adc_writeb(SH705xADC *a, int reg, uint8_t val)
{
    if (reg == 24) {
        a->adcsr = (a->adcsr & val & ADCSR_ADF) | (val & 0x7f);
        adc_irq(a);
    } else if (reg == 25) {
        bool start = (val & ADCR_ADST) && !(a->adcr & ADCR_ADST);
        a->adcr = (a->adcr & ADCR_ADST) | (val & 0xd0);
        if (!(val & ADCR_ADST)) {
            a->adcr &= ~ADCR_ADST;
            timer_del(a->timer);
        } else if (start) {
            adc_start(a);
        }
    }
}

/* map an absolute byte address in the ADC area to (module, reg) */
static SH705xADC *adc_lookup(SH705xState *s, uint32_t addr, int *reg)
{
    static const uint32_t data[3] = { 0xFFFFF800, 0xFFFFF820, 0xFFFFF840 };
    static const uint32_t ctl[3] = { 0xFFFFF818, 0xFFFFF838, 0xFFFFF858 };

    for (int i = 0; i < s->nadc; i++) {
        if (addr >= data[i] && addr < data[i] + 2 * s->adc[i].nchan) {
            *reg = addr - data[i];
            return &s->adc[i];
        }
        if (addr == ctl[i] || addr == ctl[i] + 1) {
            *reg = 24 + addr - ctl[i];
            return &s->adc[i];
        }
    }
    return NULL;
}

/* ---------------------------------------------------------------------- */
/* Serial communication interface                                         */
/* ---------------------------------------------------------------------- */

#define SSR_TDRE 0x80
#define SSR_RDRF 0x40
#define SSR_ORER 0x20
#define SSR_FER  0x10
#define SSR_PER  0x08
#define SSR_TEND 0x04
#define SCR_TIE  0x80
#define SCR_RIE  0x40
#define SCR_TE   0x20
#define SCR_RE   0x10
#define SCR_TEIE 0x04

static void sci_irq(SH705xSCI *c)
{
    int vec = SH705X_VEC_SCI0 + 4 * c->index;

    sh705x_set_irq(c->soc, vec, (c->scr & SCR_RIE) &&
                   (c->ssr & (SSR_ORER | SSR_FER | SSR_PER)));
    sh705x_set_irq(c->soc, vec + 1, (c->scr & SCR_RIE) &&
                   (c->ssr & SSR_RDRF));
    sh705x_set_irq(c->soc, vec + 2, (c->scr & SCR_TIE) &&
                   (c->ssr & SSR_TDRE));
    sh705x_set_irq(c->soc, vec + 3, (c->scr & SCR_TEIE) &&
                   (c->ssr & SSR_TEND));
}

static int64_t sci_byte_ns(SH705xSCI *c)
{
    int n = c->smr & 3;
    int bits = 1 + ((c->smr & 0x40) ? 7 : 8) + ((c->smr & 0x20) ? 1 : 0) +
               ((c->smr & 0x08) ? 2 : 1);
    double baud;

    if (c->smr & 0x80) {        /* clocked synchronous */
        baud = (double)c->soc->pclk_hz / (8.0 * (1 << (2 * n)) *
                                          (c->brr + 1));
        bits = 8;
    } else {
        baud = (double)c->soc->pclk_hz / (32.0 * (1 << (2 * n)) *
                                          (c->brr + 1));
    }
    return (int64_t)(bits * 1e9 / baud);
}

static void sci_rx_push(SH705xSCI *c, uint8_t b)
{
    if (c->rx_count < (int)sizeof(c->rx_fifo)) {
        c->rx_fifo[(c->rx_head + c->rx_count) % sizeof(c->rx_fifo)] = b;
        c->rx_count++;
    }
    if (!timer_pending(c->rx_timer)) {
        timer_mod(c->rx_timer, sh705x_now() + sci_byte_ns(c));
    }
}

static void sci_tx_start(SH705xSCI *c)
{
    c->tx_shift = c->tdr;
    c->tx_busy = true;
    c->ssr |= SSR_TDRE;
    c->ssr &= ~SSR_TEND;
    qemu_chr_fe_write_all(&c->chr, &c->tx_shift, 1);
    if (c->kline_echo && (c->scr & SCR_RE)) {
        sci_rx_push(c, c->tx_shift);
    }
    timer_mod(c->tx_timer, sh705x_now() + sci_byte_ns(c));
}

static void sci_tx_timer_cb(void *opaque)
{
    SH705xSCI *c = opaque;

    c->tx_busy = false;
    if (!(c->ssr & SSR_TDRE) && (c->scr & SCR_TE)) {
        sci_tx_start(c);
    } else {
        c->ssr |= SSR_TEND;
    }
    sci_irq(c);
}

static void sci_rx_timer_cb(void *opaque)
{
    SH705xSCI *c = opaque;

    if (!c->rx_count) {
        return;
    }
    if ((c->scr & SCR_RE) && !(c->ssr & SSR_RDRF)) {
        c->rdr = c->rx_fifo[c->rx_head];
        c->rx_head = (c->rx_head + 1) % sizeof(c->rx_fifo);
        c->rx_count--;
        c->ssr |= SSR_RDRF;
        sci_irq(c);
    } else if (!(c->scr & SCR_RE)) {
        /* receiver disabled: data on the line is lost */
        c->rx_head = (c->rx_head + 1) % sizeof(c->rx_fifo);
        c->rx_count--;
    }
    if (c->rx_count) {
        timer_mod(c->rx_timer, sh705x_now() + sci_byte_ns(c));
    }
    qemu_chr_fe_accept_input(&c->chr);
}

static int sci_can_receive(void *opaque)
{
    SH705xSCI *c = opaque;

    return sizeof(c->rx_fifo) - c->rx_count;
}

static void sci_receive(void *opaque, const uint8_t *buf, int size)
{
    SH705xSCI *c = opaque;

    for (int i = 0; i < size; i++) {
        sci_rx_push(c, buf[i]);
    }
}

static uint8_t sci_readb(SH705xSCI *c, int reg)
{
    switch (reg) {
    case 0: return c->smr;
    case 1: return c->brr;
    case 2: return c->scr;
    case 3: return c->tdr;
    case 4: return c->ssr;
    case 5: return c->rdr;
    case 6: return c->sdcr | 0xf2;
    }
    return 0xff;
}

static void sci_writeb(SH705xSCI *c, int reg, uint8_t val)
{
    uint8_t old;

    switch (reg) {
    case 0:
        c->smr = val;
        break;
    case 1:
        c->brr = val;
        break;
    case 2:
        old = c->scr;
        c->scr = val;
        if (!(val & SCR_TE)) {
            c->ssr |= SSR_TDRE;
        }
        if ((val & SCR_RE) && !(old & SCR_RE)) {
            qemu_chr_fe_accept_input(&c->chr);
        }
        break;
    case 3:
        c->tdr = val;
        break;
    case 4:
        old = c->ssr;
        /* TDRE, RDRF, ORER, FER, PER are cleared by writing 0; MPBT r/w */
        c->ssr = (old & val & 0xf8) | (old & 0x06) | (val & 0x01);
        if ((old & SSR_TDRE) && !(c->ssr & SSR_TDRE) && (c->scr & SCR_TE) &&
            !c->tx_busy) {
            sci_tx_start(c);
        }
        if ((old & SSR_RDRF) && !(c->ssr & SSR_RDRF) && c->rx_count &&
            !timer_pending(c->rx_timer)) {
            timer_mod(c->rx_timer, sh705x_now() + sci_byte_ns(c));
        }
        break;
    case 6:
        c->sdcr = val & 0x08;
        break;
    }
    sci_irq(c);
}

/* ---------------------------------------------------------------------- */
/* I/O dispatcher                                                         */
/* ---------------------------------------------------------------------- */

static uint16_t storage_read16(SH705xState *s, uint32_t off)
{
    return (s->regs[off] << 8) | s->regs[off + 1];
}

static void storage_write16(SH705xState *s, uint32_t off, uint16_t val,
                            uint16_t mask)
{
    uint16_t v = sh705x_merge(storage_read16(s, off), val, mask);

    s->regs[off] = v >> 8;
    s->regs[off + 1] = v;
}

static SH705xHCAN *hcan_lookup(SH705xState *s, uint32_t addr, uint32_t *off)
{
    for (int i = 0; i < s->nhcan; i++) {
        SH705xHCAN *h = &s->hcan[i];
        uint32_t base, size;

        if (h->v2) {
            base = 0xFFFFD000 + 0x800 * i;
            size = 0x800;
        } else {
            base = 0xFFFFE400 + 0x200 * i;
            size = 0x200;
        }
        if (addr >= base && addr < base + size) {
            *off = addr - base;
            return h;
        }
    }
    return NULL;
}

/* Halfword read at absolute @addr (even). */
static uint16_t io_read16(SH705xState *s, uint32_t addr)
{
    uint32_t off = addr - SH705X_IO_BASE;
    uint32_t hoff;
    SH705xHCAN *h;
    SH705xADC *a;
    uint16_t ret;
    int reg;

    if ((h = hcan_lookup(s, addr, &hoff))) {
        return sh705x_hcan_read(h, hoff);
    }
    if (addr >= 0xFFFFEC10 && addr < 0xFFFFEC14) {
        return wdt_read(s, addr - 0xFFFFEC10);
    }
    if (addr >= 0xFFFFED00 && addr < 0xFFFFED1C) {
        return intc_read(s, addr - 0xFFFFED00);
    }
    if (addr >= 0xFFFFF000 && addr < 0xFFFFF028) {
        SH705xSCI *c = &s->sci[(addr - 0xFFFFF000) / 8];
        int r = (addr - 0xFFFFF000) % 8;
        return (sci_readb(c, r) << 8) | sci_readb(c, r + 1);
    }
    if (addr >= 0xFFFFF400 && addr < 0xFFFFF700) {
        return sh705x_atu_read(&s->atu, addr);
    }
    if (addr >= 0xFFFFF710 && addr < 0xFFFFF71E) {
        return cmt_read(s, addr - 0xFFFFF710);
    }
    if (addr >= 0xFFFFF800 && addr < 0xFFFFF860) {
        int reg2;
        SH705xADC *a2 = adc_lookup(s, addr + 1, &reg2);
        a = adc_lookup(s, addr, &reg);
        return ((a ? adc_readb(a, reg) : 0xff) << 8) |
               (a2 ? adc_readb(a2, reg2) : 0xff);
    }
    if (addr == 0xFFFFF72E) {       /* ADTRGR1/ADTRGR2 */
        return (s->adc[1].adtrgr << 8) | s->adc[2].adtrgr;
    }
    if (addr == 0xFFFFF76E) {
        return (s->adc[0].adtrgr << 8) | s->regs[off + 1];
    }
    if (addr >= 0xFFFFF720 && addr < 0xFFFFF790 &&
        port_access(s, addr - 0xFFFFF000, false, 0, 0, &ret)) {
        return ret;
    }
    return storage_read16(s, off);
}

static void io_write16(SH705xState *s, uint32_t addr, uint16_t val,
                       uint16_t mask)
{
    uint32_t off = addr - SH705X_IO_BASE;
    uint32_t hoff;
    SH705xHCAN *h;
    uint16_t dummy;
    int reg;

    if ((h = hcan_lookup(s, addr, &hoff))) {
        sh705x_hcan_write(h, hoff, val, mask);
        return;
    }
    if (addr >= 0xFFFFEC10 && addr < 0xFFFFEC14) {
        wdt_write(s, addr - 0xFFFFEC10, val, mask);
        return;
    }
    if (addr >= 0xFFFFED00 && addr < 0xFFFFED1C) {
        intc_write(s, addr - 0xFFFFED00, val, mask);
        return;
    }
    if (addr >= 0xFFFFF000 && addr < 0xFFFFF028) {
        SH705xSCI *c = &s->sci[(addr - 0xFFFFF000) / 8];
        int r = (addr - 0xFFFFF000) % 8;
        if (mask & 0xff00) {
            sci_writeb(c, r, val >> 8);
        }
        if (mask & 0x00ff) {
            sci_writeb(c, r + 1, val);
        }
        return;
    }
    if (addr >= 0xFFFFF400 && addr < 0xFFFFF700) {
        sh705x_atu_write(&s->atu, addr, val, mask);
        return;
    }
    if (addr >= 0xFFFFF710 && addr < 0xFFFFF71E) {
        cmt_write(s, addr - 0xFFFFF710, val, mask);
        return;
    }
    if (addr >= 0xFFFFF800 && addr < 0xFFFFF860) {
        SH705xADC *a;
        if ((mask & 0xff00) && (a = adc_lookup(s, addr, &reg))) {
            adc_writeb(a, reg, val >> 8);
        }
        if ((mask & 0x00ff) && (a = adc_lookup(s, addr + 1, &reg))) {
            adc_writeb(a, reg, val);
        }
        return;
    }
    if (addr == 0xFFFFF72E) {
        if (mask & 0xff00) {
            s->adc[1].adtrgr = (val >> 8) & 0x80;
        }
        if (mask & 0x00ff) {
            s->adc[2].adtrgr = val & 0x80;
        }
        return;
    }
    if (addr == 0xFFFFF76E && (mask & 0xff00)) {
        s->adc[0].adtrgr = (val >> 8) & 0x80;
        mask &= 0x00ff;
        if (!mask) {
            return;
        }
    }
    if (addr >= 0xFFFFF720 && addr < 0xFFFFF790 &&
        port_access(s, addr - 0xFFFFF000, true, val, mask, &dummy)) {
        return;
    }
    storage_write16(s, off, val, mask);
}

static uint64_t sh705x_io_read(void *opaque, hwaddr offset, unsigned size)
{
    SH705xState *s = opaque;
    uint32_t addr = SH705X_IO_BASE + offset;
    uint16_t v;

    switch (size) {
    case 4:
        return ((uint32_t)io_read16(s, addr) << 16) | io_read16(s, addr + 2);
    case 2:
        if (!(addr & 1)) {
            return io_read16(s, addr);
        }
        return (sh705x_io_read(s, offset, 1) << 8) |
               sh705x_io_read(s, offset + 1, 1);
    default:
        v = io_read16(s, addr & ~1);
        return addr & 1 ? v & 0xff : v >> 8;
    }
}

static void sh705x_io_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    SH705xState *s = opaque;
    uint32_t addr = SH705X_IO_BASE + offset;

    switch (size) {
    case 4:
        io_write16(s, addr, val >> 16, 0xffff);
        io_write16(s, addr + 2, val, 0xffff);
        break;
    case 2:
        if (!(addr & 1)) {
            io_write16(s, addr, val, 0xffff);
        } else {
            sh705x_io_write(s, offset, val >> 8, 1);
            sh705x_io_write(s, offset + 1, val & 0xff, 1);
        }
        break;
    default:
        if (addr & 1) {
            io_write16(s, addr & ~1, val & 0xff, 0x00ff);
        } else {
            io_write16(s, addr, (val & 0xff) << 8, 0xff00);
        }
        break;
    }
}

static const MemoryRegionOps sh705x_io_ops = {
    .read = sh705x_io_read,
    .write = sh705x_io_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
};

/* ---------------------------------------------------------------------- */
/* SoC device                                                             */
/* ---------------------------------------------------------------------- */

static void sh705x_cpu_reset(void *opaque)
{
    SH705xState *s = opaque;
    CPUState *cs = CPU(s->cpu);
    CPUSH4State *env = &s->cpu->env;
    uint8_t *vec;

    cpu_reset(cs);
    /* power-on reset: PC = @0, SP = @4 (big endian) */
    vec = rom_ptr_for_as(&address_space_memory, 0, 8);
    if (vec) {
        env->pc = ldl_be_p(vec);
        env->gregs[15] = ldl_be_p(vec + 4);
    } else {
        env->pc = address_space_ldl_be(&address_space_memory, 0,
                                       MEMTXATTRS_UNSPECIFIED, NULL);
        env->gregs[15] = address_space_ldl_be(&address_space_memory, 4,
                                              MEMTXATTRS_UNSPECIFIED, NULL);
    }
}

static void sh705x_reset_hold(Object *obj, ResetType type)
{
    SH705xState *s = SH705X_SOC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->pending, 0, sizeof(s->pending));
    memset(s->ipr, 0, sizeof(s->ipr));
    s->icr = 0;
    s->isr = 0;

    s->wdt.tcsr = 0x18;
    s->wdt.rstcsr = 0x1f;
    atu_cnt_set(&s->wdt.cnt, sh705x_now(), 0);
    s->wdt.cnt.period_ps = 0;
    timer_del(s->wdt.timer);

    s->cmt.cmstr = 0;
    for (int i = 0; i < 2; i++) {
        s->cmt.ch[i].cmcsr = 0;
        s->cmt.ch[i].cmcor = 0xffff;
        atu_cnt_init(&s->cmt.ch[i].cnt, 0xffff);
    }
    timer_del(s->cmt.timer);

    for (int i = 0; i < SH705X_NUM_PORTS; i++) {
        s->port[i].dr = 0;
        s->port[i].ior = 0;
    }

    for (int i = 0; i < SH705X_NUM_SCI; i++) {
        SH705xSCI *c = &s->sci[i];
        c->smr = c->scr = c->tdr = c->rdr = c->sdcr = 0;
        c->brr = 0xff;
        c->ssr = SSR_TDRE | SSR_TEND;
        c->tx_busy = false;
        c->rx_count = 0;
        timer_del(c->tx_timer);
        timer_del(c->rx_timer);
    }

    for (int i = 0; i < s->nadc; i++) {
        SH705xADC *a = &s->adc[i];
        memset(a->addr, 0, sizeof(a->addr));
        a->adcsr = a->adcr = a->adtrgr = 0;
        timer_del(a->timer);
    }

    sh705x_atu_reset(&s->atu);
    for (int i = 0; i < s->nhcan; i++) {
        sh705x_hcan_reset(&s->hcan[i]);
    }
}

static void sh705x_realize(DeviceState *dev, Error **errp)
{
    SH705xState *s = SH705X_SOC(dev);
    MemoryRegion *sysmem = get_system_memory();
    const char *cpu_type;
    bool hcan_v2 = false;
    int hcan_mb = 16;

    switch (s->variant) {
    case SH705X_7052:
        cpu_type = TYPE_SH2_CPU;
        s->rom_size = 256 * KiB;
        s->ram_base = 0xFFFF8000;
        s->ram_size = 12 * KiB;
        s->nhcan = 1;
        s->nadc = 2;
        break;
    case SH705X_7054:
        cpu_type = TYPE_SH2_CPU;
        s->rom_size = 384 * KiB;
        s->ram_base = 0xFFFF8000;
        s->ram_size = 16 * KiB;
        s->nhcan = 1;
        s->nadc = 2;
        break;
    case SH705X_7055:
        cpu_type = TYPE_SH2E_CPU;
        s->rom_size = 512 * KiB;
        s->ram_base = 0xFFFF6000;
        s->ram_size = 32 * KiB;
        s->nhcan = 2;
        s->nadc = 3;
        break;
    case SH705X_7058:
    default:
        cpu_type = TYPE_SH2E_CPU;
        s->rom_size = 1 * MiB;
        s->ram_base = 0xFFFF0000;
        s->ram_size = 48 * KiB;
        s->nhcan = 2;
        s->nadc = 3;
        hcan_v2 = true;
        hcan_mb = 32;
        break;
    }

    s->cpu = SUPERH_CPU(cpu_create(cpu_type));
    s->cpu->env.sh2_irq_query = sh705x_irq_query;
    s->cpu->env.sh2_irq_opaque = s;

    memory_region_init_rom(&s->rom, OBJECT(s), "sh705x.rom", s->rom_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0, &s->rom);
    memory_region_init_ram(&s->ram, OBJECT(s), "sh705x.ram", s->ram_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, s->ram_base, &s->ram);
    memory_region_init_io(&s->io, OBJECT(s), &sh705x_io_ops, s,
                          "sh705x.io", SH705X_IO_SIZE);
    memory_region_add_subregion_overlap(sysmem, SH705X_IO_BASE, &s->io, -1);

    intc_init_table(s);
    for (int i = 0; i < 8; i++) {
        char name[8];
        EcuPin *pin;
        IrqPinCtx *ctx = g_new(IrqPinCtx, 1);

        snprintf(name, sizeof(name), "IRQ%d", i);
        pin = ecu_pin(name);
        ctx->s = s;
        ctx->n = i;
        ecu_pin_external_drive(pin, 1);
        ecu_pin_set_input_handler(pin, intc_irq_pin_cb, ctx);
        s->irq_level[i] = 1;
    }
    ecu_pin_external_drive(ecu_pin("NMI"), 1);
    s->nmi_level = 1;
    ecu_pin_set_input_handler(ecu_pin("NMI"), intc_nmi_cb, s);

    s->wdt.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_timer_cb, s);
    atu_cnt_init(&s->wdt.cnt, 0xff);
    s->cmt.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cmt_timer_cb, s);

    for (int i = 0; i < SH705X_NUM_PORTS; i++) {
        SH705xPort *p = &s->port[i];
        p->name = port_desc[i].name;
        p->pins = port_desc[i].pins;
        for (int b = 0; b < 16; b++) {
            char name[8];
            snprintf(name, sizeof(name), "P%s%d", p->name, b);
            p->pin[b] = ecu_pin(name);
        }
    }

    for (int i = 0; i < s->nadc; i++) {
        SH705xADC *a = &s->adc[i];
        a->soc = s;
        a->index = i;
        a->first_chan = 12 * i;
        a->nchan = i < 2 ? 12 : 8;
        a->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, adc_timer_cb, a);
    }

    for (int i = 0; i < SH705X_NUM_SCI; i++) {
        SH705xSCI *c = &s->sci[i];
        c->soc = s;
        c->index = i;
        c->kline_echo = s->kline_echo;
        c->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sci_tx_timer_cb, c);
        c->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sci_rx_timer_cb, c);
        qemu_chr_fe_set_handlers(&c->chr, sci_can_receive, sci_receive,
                                 NULL, NULL, c, NULL, true);
    }

    for (int i = 0; i < s->nhcan; i++) {
        s->hcan[i].canbus = s->canbus[i];
        sh705x_hcan_init(&s->hcan[i], s, i, hcan_v2, hcan_mb,
                         i ? SH705X_VEC_HCAN1 : SH705X_VEC_HCAN0);
    }

    sh705x_atu_init(s);

    qemu_register_reset(sh705x_cpu_reset, s);
}

static const Property sh705x_props[] = {
    DEFINE_PROP_UINT32("variant", SH705xState, variant, SH705X_7058),
    DEFINE_PROP_UINT32("pclk-hz", SH705xState, pclk_hz, 20000000),
    DEFINE_PROP_UINT32("avref-mv", SH705xState, avref_mv, 5000),
    DEFINE_PROP_BOOL("wdt-reset", SH705xState, wdt_reset, true),
    DEFINE_PROP_BOOL("kline-echo", SH705xState, kline_echo, false),
    DEFINE_PROP_CHR("sci0", SH705xState, sci[0].chr),
    DEFINE_PROP_CHR("sci1", SH705xState, sci[1].chr),
    DEFINE_PROP_CHR("sci2", SH705xState, sci[2].chr),
    DEFINE_PROP_CHR("sci3", SH705xState, sci[3].chr),
    DEFINE_PROP_CHR("sci4", SH705xState, sci[4].chr),
    DEFINE_PROP_LINK("canbus0", SH705xState, canbus[0], TYPE_CAN_BUS,
                     CanBusState *),
    DEFINE_PROP_LINK("canbus1", SH705xState, canbus[1], TYPE_CAN_BUS,
                     CanBusState *),
};

static void sh705x_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = sh705x_realize;
    dc->user_creatable = false;
    rc->phases.hold = sh705x_reset_hold;
    device_class_set_props(dc, sh705x_props);
}

static const TypeInfo sh705x_info = {
    .name = TYPE_SH705X_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SH705xState),
    .class_init = sh705x_class_init,
};

static void sh705x_register_types(void)
{
    type_register_static(&sh705x_info);
}

type_init(sh705x_register_types)
