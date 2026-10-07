/*
 * Renesas M32C/87 microcontroller
 *
 * SFR addresses follow the M32C/80 series layout (interrupt control
 * registers, timers, UART2-4, ports and A/D0 match Ghidra's M16C_80
 * processor definition).  The CAN module register layout and the CAN
 * interrupt vector numbers are best effort; they are collected in
 * can_layout/m32c87_icr_map below so they can be adjusted against the
 * M32C/87 hardware manual.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/address-spaces.h"
#include "system/runstate.h"
#include "system/memory.h"
#include "system/reset.h"
#include "chardev/char-fe.h"
#include "hw/m32c/m32c87.h"

int64_t m32c87_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint16_t rd16(M32C87State *s, uint32_t a)
{
    return s->regs[a] | (s->regs[a + 1] << 8);
}

/* ---------------------------------------------------------------------- */
/* Interrupt control registers                                            */
/* ---------------------------------------------------------------------- */

#define ICR_ILVL    0x07
#define ICR_IR      0x08
#define ICR_POL     0x10
#define ICR_LVS     0x20
#define RLVL_ADDR   0x009f
#define RLVL_FSIT   0x08

static const struct { int vec; uint16_t icr; } m32c87_icr_map[] = {
    { 8, 0x68 }, { 9, 0x88 }, { 10, 0x6a }, { 11, 0x8a },     /* DMA0-3 */
    { 12, 0x6c }, { 13, 0x8c }, { 14, 0x6e }, { 15, 0x8e },   /* TA0-3 */
    { 16, 0x70 },                                             /* TA4 */
    { 17, 0x90 }, { 18, 0x72 }, { 19, 0x92 }, { 20, 0x74 },   /* UART0/1 */
    { 21, 0x94 }, { 22, 0x76 }, { 23, 0x96 }, { 24, 0x78 },   /* TB0-3 */
    { 25, 0x98 },                                             /* TB4 */
    { 26, 0x7a }, { 27, 0x9a }, { 28, 0x7c }, { 29, 0x9c },   /* INT5-2 */
    { 30, 0x7e }, { 31, 0x9e },                               /* INT1-0 */
    { 32, 0x69 },                                             /* TB5 */
    { 33, 0x89 }, { 34, 0x6b }, { 35, 0x8b }, { 36, 0x6d },   /* UART2/3 */
    { 37, 0x8d }, { 38, 0x6f },                               /* UART4 */
    { 39, 0x8f }, { 40, 0x71 }, { 41, 0x91 },                 /* bus coll. */
    { 42, 0x73 }, { 43, 0x93 },                               /* AD0, KEY */
    { 44, 0x75 }, { 45, 0x95 }, { 46, 0x77 }, { 47, 0x97 },   /* IIO0-3 */
    { 48, 0x79 }, { 49, 0x99 }, { 50, 0x7b }, { 51, 0x9b },   /* IIO4-7 */
    { 52, 0x7d }, { 53, 0x9d }, { 54, 0x7f },                 /* IIO8-10 */
    /* best effort: IIO11 / CAN interrupt control registers */
    { 57, 0x81 }, { 58, 0x83 }, { 59, 0x85 }, { 60, 0x87 },
};

static void m32c87_update_irq(M32C87State *s)
{
    CPUState *cs = CPU(s->cpu);
    bool any = s->nmi_pending || s->wdt_pending;

    for (int v = 0; v < M32C87_NUM_VEC && !any; v++) {
        int a = s->vec_icr[v];
        if (a >= 0 && (s->regs[a] & ICR_IR) && (s->regs[a] & ICR_ILVL)) {
            any = true;
        }
    }
    if (any) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

void m32c87_set_ir(M32C87State *s, int vec)
{
    int a = s->vec_icr[vec];

    if (a < 0) {
        return;
    }
    s->regs[a] |= ICR_IR;
    m32c87_update_irq(s);
}

static bool m32c87_irq_query(void *opaque, M32CIrqRequest *req)
{
    M32C87State *s = opaque;
    int best = -1, best_lvl = 0;

    if (s->nmi_pending || s->wdt_pending) {
        req->vec = -1;
        req->level = 8;
        req->fixed_vec = s->nmi_pending ? M32C_VEC_NMI : M32C_VEC_WDT;
        req->fast = false;
        return true;
    }
    for (int v = 0; v < M32C87_NUM_VEC; v++) {
        int a = s->vec_icr[v];
        int lvl;
        if (a < 0 || !(s->regs[a] & ICR_IR)) {
            continue;
        }
        lvl = s->regs[a] & ICR_ILVL;
        if (lvl > best_lvl) {
            best = v;
            best_lvl = lvl;
        }
    }
    if (best < 0) {
        return false;
    }
    req->vec = best;
    req->level = best_lvl;
    req->fixed_vec = 0;
    req->fast = best_lvl == 7 && (s->regs[RLVL_ADDR] & RLVL_FSIT);
    return true;
}

static void int_pin_eval(M32C87State *s, int n);

static void m32c87_irq_ack(void *opaque, const M32CIrqRequest *req)
{
    M32C87State *s = opaque;

    if (req->level >= 8) {
        if (req->fixed_vec == M32C_VEC_NMI) {
            s->nmi_pending = false;
        } else {
            s->wdt_pending = false;
        }
    } else if (s->vec_icr[req->vec] >= 0) {
        s->regs[s->vec_icr[req->vec]] &= ~ICR_IR;
        if (req->vec >= M32C87_VEC_INT5 && req->vec <= M32C87_VEC_INT5 + 5) {
            /* level sensitive INT pins request again while active */
            int_pin_eval(s, 5 - (req->vec - M32C87_VEC_INT5));
        }
    }
    m32c87_update_irq(s);
}

/* INTn pins: vector 31 - n */
static void int_pin_eval(M32C87State *s, int n)
{
    int vec = M32C87_VEC_INT5 + (5 - n);
    uint8_t icr = s->regs[s->vec_icr[vec]];
    bool active_high = icr & ICR_POL;

    if ((icr & ICR_LVS) && s->int_level[n] == active_high) {
        s->regs[s->vec_icr[vec]] |= ICR_IR;
    }
}

typedef struct PinCtx {
    M32C87State *s;
    int n;
} PinCtx;

static void int_pin_cb(void *opaque, int level)
{
    PinCtx *p = opaque;
    M32C87State *s = p->s;
    int n = p->n;
    int vec = M32C87_VEC_INT5 + (5 - n);
    uint8_t icr = s->regs[s->vec_icr[vec]];
    int old = s->int_level[n];

    s->int_level[n] = level;
    if (icr & ICR_LVS) {
        int_pin_eval(s, n);
    } else if (old != level && level == !!(icr & ICR_POL)) {
        s->regs[s->vec_icr[vec]] |= ICR_IR;
    }
    m32c87_update_irq(s);
}

static void nmi_pin_cb(void *opaque, int level)
{
    M32C87State *s = opaque;

    if (s->nmi_level && !level) {
        s->nmi_pending = true;
        m32c87_update_irq(s);
    }
    s->nmi_level = level;
}

/* ---------------------------------------------------------------------- */
/* Watchdog (WDTS 0x000e, WDC 0x000f)                                     */
/* ---------------------------------------------------------------------- */

static int64_t wdt_period_ns(M32C87State *s)
{
    int presc = (s->regs[0x0f] & 0x80) ? 128 : 16;

    return (int64_t)32768 * presc * 1000000000LL / s->pclk_hz;
}

static void wdt_cb(void *opaque)
{
    M32C87State *s = opaque;

    if (!s->wdt_running) {
        return;
    }
    if ((s->regs[0x05] & 0x04) && s->wdt_reset) {     /* PM12 */
        qemu_log_mask(LOG_GUEST_ERROR, "m32c87: watchdog reset\n");
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    s->wdt_pending = true;
    m32c87_update_irq(s);
    s->wdt_start_ns = m32c87_now();
    timer_mod(s->wdt_timer, s->wdt_start_ns + wdt_period_ns(s));
}

static void wdt_restart(M32C87State *s)
{
    s->wdt_running = true;
    s->wdt_start_ns = m32c87_now();
    timer_mod(s->wdt_timer, s->wdt_start_ns + wdt_period_ns(s));
}

/* ---------------------------------------------------------------------- */
/* Timers A and B                                                         */
/* ---------------------------------------------------------------------- */

#define TABSR   0x0340
#define ONSF    0x0342
#define TBSR    0x0300

static const uint16_t ta_addr[5] = { 0x0346, 0x0348, 0x034a, 0x034c, 0x034e };
static const uint16_t ta_mr[5] = { 0x0356, 0x0357, 0x0358, 0x0359, 0x035a };
static const uint16_t tb_addr[6] = { 0x0350, 0x0352, 0x0354,
                                     0x0310, 0x0312, 0x0314 };
static const uint16_t tb_mr[6] = { 0x035b, 0x035c, 0x035d,
                                   0x031b, 0x031c, 0x031d };

static bool timer_started(M32C87State *s, M32C87Timer *t)
{
    if (!t->is_b) {
        return s->regs[TABSR] & (1 << t->index);
    }
    if (t->index < 3) {
        return s->regs[TABSR] & (0x20 << t->index);
    }
    return s->regs[TBSR] & (0x20 << (t->index - 3));
}

static int64_t timer_tick_ps(M32C87State *s, M32C87Timer *t)
{
    static const int div[4] = { 1, 8, 2, 0 };
    int tck = (t->mr >> 6) & 3;

    if (tck == 3) {
        return 1000000000000LL / 1024;          /* fC32 */
    }
    return 1000000000000LL * div[tck] / s->pclk_hz;
}

static int timer_vec(M32C87Timer *t)
{
    if (!t->is_b) {
        return M32C87_VEC_TA0 + t->index;
    }
    return t->index < 5 ? M32C87_VEC_TB0 + t->index : M32C87_VEC_TB5;
}

static void timer_out(M32C87Timer *t, int level, int64_t when)
{
    t->out = level;
    if (t->out_pin && (t->mr & 0x04)) {
        ecu_pin_mcu_drive_at(t->out_pin, level, when);
    }
}

static int timer_mode(M32C87Timer *t)
{
    return t->mr & 3;
}

/* (re)arm the channel timer from start_ns */
static void timer_schedule(M32C87State *s, M32C87Timer *t)
{
    int64_t len;

    timer_del(t->timer);
    if (!t->running || !t->period_ps) {
        return;
    }
    switch (timer_mode(t)) {
    case 0:                                     /* timer mode */
        len = (int64_t)(t->reload + 1) * t->period_ps / 1000;
        break;
    case 2:
        if (t->is_b) {                          /* measurement: overflow */
            len = (int64_t)0x10000 * t->period_ps / 1000;
            break;
        }
        if (!t->one_shot_active) {
            return;
        }
        len = (int64_t)t->reload * t->period_ps / 1000;
        break;
    case 3: {                                   /* PWM */
        int64_t high, period;
        if (t->mr & 0x20) {                     /* 8-bit PWM */
            int m = t->reload & 0xff, n = t->reload >> 8;
            period = (int64_t)(m + 1) * 255;
            high = (int64_t)n * (m + 1);
        } else {
            period = 0xffff;
            high = t->reload;
        }
        len = (t->pwm_high ? high : period - high) * t->period_ps / 1000;
        break;
    }
    default:
        return;                                 /* event counter */
    }
    t->deadline_ns = t->start_ns + MAX(len, 1);
    timer_mod(t->timer, t->deadline_ns);
}

/* the real callback needs the SoC pointer; keep it next to the timer */
typedef struct TimerCtx {
    M32C87State *s;
    M32C87Timer *t;
} TimerCtx;

static void timer_event(void *opaque)
{
    TimerCtx *c = opaque;
    M32C87State *s = c->s;
    M32C87Timer *t = c->t;
    int64_t now = m32c87_now();
    int64_t end = MIN(t->deadline_ns, now);
    switch (timer_mode(t)) {
    case 0:
        t->start_ns = end;
        if (t->mr & 0x04) {
            timer_out(t, !t->out, end);
        }
        m32c87_set_ir(s, timer_vec(t));
        break;
    case 2:
        if (t->is_b) {
            t->start_ns = end;                  /* counter overflow */
            t->mr |= 0x20;
            s->regs[tb_mr[t->index]] = t->mr;
            break;
        }
        t->one_shot_active = false;
        timer_out(t, 0, end);
        m32c87_set_ir(s, timer_vec(t));
        break;
    case 3:
        t->start_ns = end;
        t->pwm_high = !t->pwm_high;
        timer_out(t, t->pwm_high, end);
        if (!t->pwm_high) {
            m32c87_set_ir(s, timer_vec(t));
        }
        break;
    }
    timer_schedule(s, t);
}

static uint16_t timer_count(M32C87State *s, M32C87Timer *t)
{
    int64_t now = m32c87_now();
    uint64_t ticks;

    if (!t->running || !t->period_ps) {
        return t->value;
    }
    ticks = (uint64_t)(now - t->start_ns) * 1000 / t->period_ps;
    switch (timer_mode(t)) {
    case 0:
        return t->reload - (ticks % (t->reload + 1));
    case 2:
        if (t->is_b) {
            return t->measured;
        }
        return t->one_shot_active ? t->reload - MIN(ticks, t->reload) : 0;
    default:
        return t->value;
    }
}

static void timer_update(M32C87State *s, M32C87Timer *t)
{
    bool run = timer_started(s, t);
    int64_t now = m32c87_now();

    t->period_ps = timer_tick_ps(s, t);
    if (run && !t->running) {
        t->running = true;
        t->start_ns = now;
        t->value = t->reload;
        t->pwm_high = false;
        if (timer_mode(t) == 3) {
            t->pwm_high = true;
            timer_out(t, 1, now);
        }
    } else if (!run && t->running) {
        t->value = timer_count(s, t);
        t->running = false;
        t->one_shot_active = false;
    }
    timer_schedule(s, t);
}

static void timer_one_shot_start(M32C87State *s, M32C87Timer *t)
{
    if (!t->running || timer_mode(t) != 2 || t->is_b) {
        return;
    }
    t->one_shot_active = true;
    t->start_ns = m32c87_now();
    timer_out(t, 1, t->start_ns);
    timer_schedule(s, t);
}

static void timer_write_reg(M32C87State *s, M32C87Timer *t, uint16_t v)
{
    t->reload = v;
    if (!t->running) {
        t->value = v;
    }
}

static void timer_in_cb(void *opaque, int level)
{
    TimerCtx *c = opaque;
    M32C87State *s = c->s;
    M32C87Timer *t = c->t;
    int old = t->in_level;
    int64_t now = m32c87_now();

    t->in_level = level;
    if (!t->running || old == level) {
        return;
    }
    if (timer_mode(t) == 1) {
        /* event counter: MR1 (bit3) set counts rising edges on TA */
        bool rising = t->is_b ? (t->mr & 0x04) : (t->mr & 0x08);
        if (level == rising) {
            if (t->value == 0) {
                t->value = t->reload;
                m32c87_set_ir(s, timer_vec(t));
            } else {
                t->value--;
            }
        }
    } else if (t->is_b && timer_mode(t) == 2) {
        int sel = (t->mr >> 2) & 3;
        bool hit = (sel == 0 && !level) || (sel == 1 && level) || sel >= 2;
        if (hit) {
            t->measured = (uint64_t)(now - t->start_ns) * 1000 / t->period_ps;
            t->start_ns = now;
            m32c87_set_ir(s, timer_vec(t));
            timer_schedule(s, t);
        }
    }
}

static M32C87Timer *timer_by_reg(M32C87State *s, uint32_t a, bool *is_mr)
{
    for (int i = 0; i < M32C87_NUM_TA; i++) {
        if (a == ta_addr[i]) {
            *is_mr = false;
            return &s->ta[i];
        }
        if (a == ta_mr[i]) {
            *is_mr = true;
            return &s->ta[i];
        }
    }
    for (int i = 0; i < M32C87_NUM_TB; i++) {
        if (a == tb_addr[i]) {
            *is_mr = false;
            return &s->tb[i];
        }
        if (a == tb_mr[i]) {
            *is_mr = true;
            return &s->tb[i];
        }
    }
    return NULL;
}

/* ---------------------------------------------------------------------- */
/* UART                                                                   */
/* ---------------------------------------------------------------------- */

#define UC1_TE  0x01
#define UC1_TI  0x02
#define UC1_RE  0x04
#define UC1_RI  0x08
#define UC1_IRS 0x10
#define UC0_TXEPT 0x08

static const uint16_t uart_base[5] = { 0x0364, 0x02e4, 0x0334, 0x0324, 0x02f4 };

static int64_t uart_byte_ns(M32C87UART *u)
{
    static const int div[4] = { 1, 8, 2, 1 };
    double f = (double)u->soc->pclk_hz / div[u->c0 & 3];
    double baud = f / (16.0 * (u->brg + 1));
    int bits = 1 + ((u->mr & 7) == 4 ? 7 : (u->mr & 7) == 6 ? 9 : 8) +
               ((u->mr & 0x40) ? 1 : 0) + ((u->mr & 0x10) ? 2 : 1);

    return (int64_t)(bits * 1e9 / baud);
}

static void uart_rx_push(M32C87UART *u, uint8_t b)
{
    if (u->rx_count < (int)sizeof(u->rx_fifo)) {
        u->rx_fifo[(u->rx_head + u->rx_count) % sizeof(u->rx_fifo)] = b;
        u->rx_count++;
    }
    if (!timer_pending(u->rx_timer)) {
        timer_mod(u->rx_timer, m32c87_now() + uart_byte_ns(u));
    }
}

static void uart_tx_start(M32C87UART *u)
{
    uint8_t b = u->tb;

    u->tx_busy = true;
    u->c1 |= UC1_TI;
    u->c0 &= ~UC0_TXEPT;
    qemu_chr_fe_write_all(&u->chr, &b, 1);
    if (u->soc->kline_echo && (u->c1 & UC1_RE)) {
        uart_rx_push(u, b);
    }
    if (!(u->c1 & UC1_IRS)) {
        m32c87_set_ir(u->soc, u->vec_tx);
    }
    timer_mod(u->tx_timer, m32c87_now() + uart_byte_ns(u));
}

static void uart_tx_cb(void *opaque)
{
    M32C87UART *u = opaque;

    u->tx_busy = false;
    if (!(u->c1 & UC1_TI) && (u->c1 & UC1_TE)) {
        uart_tx_start(u);
        return;
    }
    u->c0 |= UC0_TXEPT;
    if (u->c1 & UC1_IRS) {
        m32c87_set_ir(u->soc, u->vec_tx);
    }
}

static void uart_rx_cb(void *opaque)
{
    M32C87UART *u = opaque;

    if (!u->rx_count) {
        return;
    }
    if ((u->c1 & UC1_RE) && !(u->c1 & UC1_RI)) {
        u->rb = u->rx_fifo[u->rx_head];
        u->rx_head = (u->rx_head + 1) % sizeof(u->rx_fifo);
        u->rx_count--;
        u->c1 |= UC1_RI;
        m32c87_set_ir(u->soc, u->vec_rx);
    } else if (!(u->c1 & UC1_RE)) {
        u->rx_head = (u->rx_head + 1) % sizeof(u->rx_fifo);
        u->rx_count--;
    }
    if (u->rx_count) {
        timer_mod(u->rx_timer, m32c87_now() + uart_byte_ns(u));
    }
    qemu_chr_fe_accept_input(&u->chr);
}

static int uart_can_receive(void *opaque)
{
    M32C87UART *u = opaque;

    return sizeof(u->rx_fifo) - u->rx_count;
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    M32C87UART *u = opaque;

    for (int i = 0; i < size; i++) {
        uart_rx_push(u, buf[i]);
    }
}

static uint8_t uart_read(M32C87UART *u, int off)
{
    switch (off) {
    case 0: case 1: case 2: case 3:
        return u->smr[off];
    case 4: return u->mr;
    case 5: return u->brg;
    case 6: return u->tb;
    case 7: return u->tb >> 8;
    case 8: return u->c0;
    case 9: return u->c1;
    case 10: {
        uint8_t v = u->rb;
        u->c1 &= ~UC1_RI;
        if (u->rx_count && !timer_pending(u->rx_timer)) {
            timer_mod(u->rx_timer, m32c87_now() + uart_byte_ns(u));
        }
        return v;
    }
    case 11: return (u->rb >> 8) & 0x01;
    }
    return 0;
}

static void uart_write(M32C87UART *u, int off, uint8_t v)
{
    switch (off) {
    case 0: case 1: case 2: case 3:
        u->smr[off] = v;
        break;
    case 4:
        u->mr = v;
        break;
    case 5:
        u->brg = v;
        break;
    case 6:
        u->tb = (u->tb & 0xff00) | v;
        u->c1 &= ~UC1_TI;
        if ((u->c1 & UC1_TE) && !u->tx_busy) {
            uart_tx_start(u);
        }
        break;
    case 7:
        u->tb = (u->tb & 0x00ff) | (v << 8);
        break;
    case 8:
        u->c0 = (v & ~UC0_TXEPT) | (u->c0 & UC0_TXEPT);
        break;
    case 9:
        u->c1 = (v & ~(UC1_TI | UC1_RI)) | (u->c1 & (UC1_TI | UC1_RI));
        if (v & UC1_RE) {
            qemu_chr_fe_accept_input(&u->chr);
        }
        break;
    }
}

/* ---------------------------------------------------------------------- */
/* A/D converter 0                                                        */
/* ---------------------------------------------------------------------- */

#define AD0CON2 0x0394
#define AD0CON0 0x0396
#define AD0CON1 0x0397
#define AD_ADST 0x40

static int ad_mode(M32C87State *s)
{
    return ((s->regs[AD0CON0] >> 3) & 3) | ((s->regs[AD0CON1] & 4) ? 4 : 0);
}

static int ad_last(M32C87State *s)
{
    if (ad_mode(s) < 2) {
        return s->regs[AD0CON0] & 7;
    }
    return 2 * ((s->regs[AD0CON1] & 3) + 1) - 1;
}

static int64_t ad_conv_ns(M32C87State *s)
{
    return 49LL * 2 * 1000000000LL / s->pclk_hz;
}

static void ad_convert(M32C87State *s, int ch)
{
    static const int group_base[4] = { 0, 24, 8, 16 };
    int aps = (s->regs[AD0CON2] >> 1) & 3;
    char name[8];
    double v;
    bool bits10 = s->regs[AD0CON1] & 0x08;

    snprintf(name, sizeof(name), "AN%d", group_base[aps] + ch);
    v = ecu_analog_read(name);
    s->ad[ch] = bits10 ? ecu_adc_convert(v, s->avref_mv / 1000.0, 10)
                       : ecu_adc_convert(v, s->avref_mv / 1000.0, 8);
}

static void ad_cb(void *opaque)
{
    M32C87State *s = opaque;
    int mode = ad_mode(s);

    if (!(s->regs[AD0CON0] & AD_ADST)) {
        return;
    }
    ad_convert(s, s->ad_cur);
    switch (mode) {
    case 0:                             /* one-shot */
        s->regs[AD0CON0] &= ~AD_ADST;
        m32c87_set_ir(s, M32C87_VEC_AD0);
        return;
    case 1:                             /* repeat */
        break;
    default:                            /* sweeps */
        if (s->ad_cur < ad_last(s)) {
            s->ad_cur++;
        } else if (mode == 2) {
            s->regs[AD0CON0] &= ~AD_ADST;
            m32c87_set_ir(s, M32C87_VEC_AD0);
            return;
        } else {
            s->ad_cur = 0;
        }
        break;
    }
    timer_mod(s->ad_timer, m32c87_now() + ad_conv_ns(s));
}

static void ad_start(M32C87State *s)
{
    s->ad_cur = ad_mode(s) < 2 ? (s->regs[AD0CON0] & 7) : 0;
    timer_mod(s->ad_timer, m32c87_now() + ad_conv_ns(s));
}

/* ---------------------------------------------------------------------- */
/* I/O ports                                                              */
/* ---------------------------------------------------------------------- */

static const uint16_t port_p[16] = {
    0x3e0, 0x3e1, 0x3e4, 0x3e5, 0x3e8, 0x3e9, 0x3c0, 0x3c1,
    0x3c4, 0x3c5, 0x3c8, 0x3c9, 0x3cc, 0x3cd, 0x3d0, 0x3d1,
};

static int port_by_addr(uint32_t a, bool *is_dir)
{
    for (int i = 0; i < 16; i++) {
        if (a == port_p[i]) {
            *is_dir = false;
            return i;
        }
        if (a == port_p[i] + 2) {
            *is_dir = true;
            return i;
        }
    }
    return -1;
}

static void port_drive(M32C87State *s, int n)
{
    uint8_t dir = s->regs[port_p[n] + 2], dat = s->regs[port_p[n]];

    for (int b = 0; b < 8; b++) {
        if (dir & (1 << b)) {
            ecu_pin_mcu_drive(s->port_pin[n][b], (dat >> b) & 1);
        }
    }
}

/* ---------------------------------------------------------------------- */
/* CAN                                                                    */
/* ---------------------------------------------------------------------- */

/*
 * Register layout relative to the channel base (best effort):
 *   +0x00 CiCTLR  +0x02 CiSTR  +0x04 CiSSTR  +0x06 CiICR  +0x08 CiIDR
 *   +0x0a CiCONR  +0x0c CiRECR +0x0d CiTECR  +0x0e CiTSR
 *   +0x20 CiSBS   +0x28 CiGMR (6)  +0x30 CiMCTL0..15  +0x40 CiLMAR (6)
 *   +0x50 CiLMBR (6)
 * Slot buffer windows (selected by CiSBS) at win and win + 0x10.
 */
static const struct { uint16_t base, win; } can_layout[M32C87_NUM_CAN] = {
    { 0x0200, 0x01e0 },
    { 0x0280, 0x0260 },
};

#define CTLR_RESET      0x0001
#define CTLR_LOOPBACK   0x0002
#define STR_TRMSUCC     0x0010
#define STR_RECSUCC     0x0020
#define STR_RESET       0x0100
#define MCTL_NEWDATA    0x01
#define MCTL_TRMACTIVE  0x02
#define MCTL_MSGLOST    0x04
#define MCTL_REMOTE     0x20
#define MCTL_RECREQ     0x40
#define MCTL_TRMREQ     0x80

static void can_slot_to_frame(M32C87CAN *c, int n, qemu_can_frame *f)
{
    uint8_t *b = c->slot[n];
    uint32_t sid = ((b[0] & 0x1f) << 6) | (b[1] & 0x3f);
    uint32_t eid = ((b[2] & 0x0f) << 14) | (b[3] << 6) | (b[4] & 0x3f);

    memset(f, 0, sizeof(*f));
    if (c->idr & (1 << n)) {
        f->can_id = (sid << 18) | eid | QEMU_CAN_EFF_FLAG;
    } else {
        f->can_id = sid;
    }
    if (c->mctl[n] & MCTL_REMOTE) {
        f->can_id |= QEMU_CAN_RTR_FLAG;
    }
    f->can_dlc = MIN(b[5] & 0xf, 8);
    memcpy(f->data, &b[6], 8);
}

static void can_irq(M32C87CAN *c, int slot)
{
    if (c->icr & (1 << slot)) {
        m32c87_set_ir(c->soc, c->vec_trx);
    }
}

static void can_transmit(M32C87CAN *c, int n)
{
    qemu_can_frame f;

    can_slot_to_frame(c, n, &f);
    if (c->canbus) {
        can_bus_client_send(&c->bus_client, &f, 1);
    }
    c->mctl[n] = (c->mctl[n] & ~MCTL_TRMACTIVE) | MCTL_NEWDATA;
    c->sstr |= 1 << n;
    c->str = (c->str & ~0x0f) | n | STR_TRMSUCC;
    can_irq(c, n);
}

static bool can_match(M32C87CAN *c, int n, const qemu_can_frame *f)
{
    const uint8_t *mask = n == 14 ? c->lmar : n == 15 ? c->lmbr : c->gmr;
    const uint8_t *b = c->slot[n];
    bool ext = f->can_id & QEMU_CAN_EFF_FLAG;
    uint32_t sid = ext ? (f->can_id >> 18) & 0x7ff : f->can_id & 0x7ff;
    uint32_t eid = ext ? f->can_id & 0x3ffff : 0;
    uint32_t msid = ((mask[0] & 0x1f) << 6) | (mask[1] & 0x3f);
    uint32_t meid = ((mask[2] & 0x0f) << 14) | (mask[3] << 6) |
                    (mask[4] & 0x3f);
    uint32_t ssid = ((b[0] & 0x1f) << 6) | (b[1] & 0x3f);
    uint32_t seid = ((b[2] & 0x0f) << 14) | (b[3] << 6) | (b[4] & 0x3f);

    if (ext != !!(c->idr & (1 << n))) {
        return false;
    }
    if ((sid ^ ssid) & msid) {
        return false;
    }
    return !ext || !((eid ^ seid) & meid);
}

static void can_rx_frame(M32C87CAN *c, const qemu_can_frame *f)
{
    bool rtr = f->can_id & QEMU_CAN_RTR_FLAG;

    for (int n = 0; n < M32C87_CAN_SLOTS; n++) {
        uint8_t *b = c->slot[n];
        bool ext;
        uint32_t sid, eid;

        if (!(c->mctl[n] & MCTL_RECREQ) || (c->mctl[n] & MCTL_TRMREQ)) {
            continue;
        }
        if (!!(c->mctl[n] & MCTL_REMOTE) != rtr || !can_match(c, n, f)) {
            continue;
        }
        if (c->mctl[n] & MCTL_NEWDATA) {
            c->mctl[n] |= MCTL_MSGLOST;
        }
        ext = f->can_id & QEMU_CAN_EFF_FLAG;
        sid = ext ? (f->can_id >> 18) & 0x7ff : f->can_id & 0x7ff;
        eid = ext ? f->can_id & 0x3ffff : 0;
        b[0] = sid >> 6;
        b[1] = sid & 0x3f;
        b[2] = eid >> 14;
        b[3] = eid >> 6;
        b[4] = eid & 0x3f;
        b[5] = MIN(f->can_dlc, 8);
        memset(&b[6], 0, 8);
        memcpy(&b[6], f->data, MIN(f->can_dlc, 8));
        c->mctl[n] |= MCTL_NEWDATA;
        c->sstr |= 1 << n;
        c->str = (c->str & ~0x0f) | n | STR_RECSUCC;
        can_irq(c, n);
        return;
    }
}

static bool can_can_receive(CanBusClientState *client)
{
    M32C87CAN *c = container_of(client, M32C87CAN, bus_client);

    return !(c->ctlr & CTLR_RESET);
}

static ssize_t can_receive(CanBusClientState *client,
                           const qemu_can_frame *frames, size_t n)
{
    M32C87CAN *c = container_of(client, M32C87CAN, bus_client);

    if (c->ctlr & CTLR_RESET) {
        return n;
    }
    for (size_t i = 0; i < n; i++) {
        if (!(frames[i].can_id & QEMU_CAN_ERR_FLAG) &&
            !(frames[i].flags & QEMU_CAN_FRMF_TYPE_FD)) {
            can_rx_frame(c, &frames[i]);
        }
    }
    return n;
}

static CanBusClientInfo m32c87_can_info = {
    .can_receive = can_can_receive,
    .receive = can_receive,
};

static bool can_access(M32C87State *s, uint32_t a, bool write, uint8_t v,
                       uint8_t *ret)
{
    for (int i = 0; i < M32C87_NUM_CAN; i++) {
        M32C87CAN *c = &s->can[i];
        uint32_t o;
        uint16_t *r16 = NULL;

        if (a >= can_layout[i].win && a < can_layout[i].win + 0x20) {
            /* slot buffer windows */
            int w = (a - can_layout[i].win) / 16;
            int slot = (c->sbs >> (4 * w)) & 0xf;
            uint8_t *b = &c->slot[slot][(a - can_layout[i].win) % 16];
            if (write) {
                *b = v;
            } else {
                *ret = *b;
            }
            return true;
        }
        if (a < c->base || a >= c->base + 0x60) {
            continue;
        }
        o = a - c->base;
        switch (o & ~1) {
        case 0x00: r16 = &c->ctlr; break;
        case 0x02: r16 = &c->str; break;
        case 0x04: r16 = &c->sstr; break;
        case 0x06: r16 = &c->icr; break;
        case 0x08: r16 = &c->idr; break;
        case 0x0a: r16 = &c->conr; break;
        case 0x0e: r16 = &c->tsr; break;
        }
        if (r16) {
            int sh = (o & 1) * 8;
            if (!write) {
                *ret = *r16 >> sh;
                return true;
            }
            if ((o & ~1) == 0x02) {
                return true;                /* status: read only */
            }
            if ((o & ~1) == 0x04) {
                *r16 &= ~(v << sh);         /* SSTR: write 1 clears */
                return true;
            }
            *r16 = (*r16 & ~(0xff << sh)) | (v << sh);
            if ((o & ~1) == 0x00) {
                if (c->ctlr & CTLR_RESET) {
                    c->str |= STR_RESET;
                } else {
                    c->str &= ~STR_RESET;
                }
            }
            return true;
        }
        if (o == 0x0c || o == 0x0d) {
            uint8_t *r = o == 0x0c ? &c->recr : &c->tecr;
            if (write) {
                *r = v;
            } else {
                *ret = *r;
            }
            return true;
        }
        if (o == 0x20) {
            if (write) {
                c->sbs = v;
            } else {
                *ret = c->sbs;
            }
            return true;
        }
        if (o >= 0x28 && o < 0x2e) {
            if (write) {
                c->gmr[o - 0x28] = v;
            } else {
                *ret = c->gmr[o - 0x28];
            }
            return true;
        }
        if (o >= 0x30 && o < 0x40) {
            int n = o - 0x30;
            if (!write) {
                *ret = c->mctl[n];
                return true;
            }
            /* status bits are cleared by writing 0 */
            c->mctl[n] = (v & 0xf8) | (c->mctl[n] & v & 0x07);
            if (!(v & MCTL_NEWDATA)) {
                c->sstr &= ~(1 << n);
            }
            if ((v & MCTL_TRMREQ) && !(c->ctlr & CTLR_RESET)) {
                c->mctl[n] |= MCTL_TRMACTIVE;
                can_transmit(c, n);
            }
            return true;
        }
        if (o >= 0x40 && o < 0x46) {
            if (write) {
                c->lmar[o - 0x40] = v;
            } else {
                *ret = c->lmar[o - 0x40];
            }
            return true;
        }
        if (o >= 0x50 && o < 0x56) {
            if (write) {
                c->lmbr[o - 0x50] = v;
            } else {
                *ret = c->lmbr[o - 0x50];
            }
            return true;
        }
        if (o >= 0x10 && o < 0x20) {
            if (write) {
                c->misc[o - 0x10] = v;
            } else {
                *ret = c->misc[o - 0x10];
            }
            return true;
        }
        return false;
    }
    return false;
}

/* ---------------------------------------------------------------------- */
/* SFR dispatcher                                                         */
/* ---------------------------------------------------------------------- */

static uint8_t sfr_readb(M32C87State *s, uint32_t a)
{
    bool flag;
    int n;
    uint8_t v;
    M32C87Timer *t;

    if (a >= 0x0380 && a < 0x0390) {
        uint16_t ad = s->ad[(a - 0x0380) / 2];
        return a & 1 ? ad >> 8 : ad;
    }
    for (int i = 0; i < M32C87_NUM_UART; i++) {
        if (a >= uart_base[i] && a < uart_base[i] + 12) {
            return uart_read(&s->uart[i], a - uart_base[i]);
        }
    }
    if ((t = timer_by_reg(s, a & ~1, &flag)) && !flag) {
        uint16_t cnt = timer_count(s, t);
        return a & 1 ? cnt >> 8 : cnt;
    }
    if ((n = port_by_addr(a, &flag)) >= 0 && !flag) {
        uint8_t dir = s->regs[port_p[n] + 2];
        v = s->regs[a] & dir;
        for (int b = 0; b < 8; b++) {
            if (!(dir & (1 << b))) {
                v |= ecu_pin_level(s->port_pin[n][b]) << b;
            }
        }
        return v;
    }
    if (a == 0x000f) {                  /* WDC: counter high bits */
        uint32_t cnt = 0x7fff;
        if (s->wdt_running) {
            int64_t el = m32c87_now() - s->wdt_start_ns;
            cnt = 0x7fff - (uint32_t)(el * 0x8000 / wdt_period_ns(s));
        }
        return (s->regs[a] & 0xe0) | ((cnt >> 10) & 0x1f);
    }
    if (can_access(s, a, false, 0, &v)) {
        return v;
    }
    return s->regs[a];
}

static void sfr_writeb(M32C87State *s, uint32_t a, uint8_t v)
{
    bool flag;
    int n;
    uint8_t dummy;
    M32C87Timer *t;

    for (int i = 0; i < M32C87_NUM_UART; i++) {
        if (a >= uart_base[i] && a < uart_base[i] + 12) {
            uart_write(&s->uart[i], a - uart_base[i], v);
            return;
        }
    }
    if (can_access(s, a, true, v, &dummy)) {
        return;
    }
    if (a >= 0x0380 && a < 0x0390) {
        return;                         /* A/D results are read only */
    }

    /* interrupt control registers: IR can only be cleared by software */
    for (int vec = 0; vec < M32C87_NUM_VEC; vec++) {
        if (s->vec_icr[vec] == (int)a) {
            s->regs[a] = (v & ~ICR_IR) | (s->regs[a] & v & ICR_IR);
            m32c87_update_irq(s);
            return;
        }
    }

    s->regs[a] = v;

    if (a == RLVL_ADDR) {
        m32c87_update_irq(s);
    } else if (a == 0x000e) {           /* WDTS */
        wdt_restart(s);
    } else if (a == TABSR || a == TBSR || a == 0x0341) {
        for (int i = 0; i < M32C87_NUM_TA; i++) {
            timer_update(s, &s->ta[i]);
        }
        for (int i = 0; i < M32C87_NUM_TB; i++) {
            timer_update(s, &s->tb[i]);
        }
    } else if (a == ONSF) {
        for (int i = 0; i < M32C87_NUM_TA; i++) {
            if (v & (1 << i)) {
                timer_one_shot_start(s, &s->ta[i]);
            }
        }
        s->regs[a] &= ~0x1f;
    } else if ((t = timer_by_reg(s, a, &flag)) != NULL && flag) {
        t->mr = v;
        timer_update(s, t);
    } else if ((t = timer_by_reg(s, a & ~1, &flag)) != NULL && !flag) {
        timer_write_reg(s, t, rd16(s, a & ~1));
    } else if ((n = port_by_addr(a, &flag)) >= 0) {
        port_drive(s, n);
    } else if (a == AD0CON0) {
        if ((v & AD_ADST) && !timer_pending(s->ad_timer)) {
            ad_start(s);
        } else if (!(v & AD_ADST)) {
            timer_del(s->ad_timer);
        }
    }
}

static uint64_t sfr_read(void *opaque, hwaddr addr, unsigned size)
{
    M32C87State *s = opaque;
    uint64_t v = 0;

    for (unsigned i = 0; i < size; i++) {
        v |= (uint64_t)sfr_readb(s, (addr + i) & (M32C87_SFR_SIZE - 1))
             << (8 * i);
    }
    return v;
}

static void sfr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    M32C87State *s = opaque;

    /* low byte first; 16-bit registers act once their high byte lands */
    for (unsigned i = 0; i < size; i++) {
        sfr_writeb(s, (addr + i) & (M32C87_SFR_SIZE - 1), val >> (8 * i));
    }
}

static const MemoryRegionOps sfr_ops = {
    .read = sfr_read,
    .write = sfr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4,
               .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 4,
              .unaligned = true },
};

/* ---------------------------------------------------------------------- */
/* Device                                                                 */
/* ---------------------------------------------------------------------- */

static void m32c87_cpu_reset(void *opaque)
{
    M32C87State *s = opaque;

    cpu_reset(CPU(s->cpu));
    m32c_cpu_load_reset_vector(s->cpu);
}

static void m32c87_reset_hold(Object *obj, ResetType type)
{
    M32C87State *s = M32C87_SOC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0x0377] = 0x01;             /* FMR0: flash ready */
    s->nmi_pending = s->wdt_pending = false;
    s->wdt_running = false;
    timer_del(s->wdt_timer);
    timer_del(s->ad_timer);
    memset(s->ad, 0, sizeof(s->ad));

    for (int i = 0; i < M32C87_NUM_TA + M32C87_NUM_TB; i++) {
        M32C87Timer *t = i < M32C87_NUM_TA ? &s->ta[i]
                                           : &s->tb[i - M32C87_NUM_TA];
        t->mr = 0;
        t->reload = t->value = 0;
        t->running = t->one_shot_active = t->pwm_high = false;
        t->out = 0;
        timer_del(t->timer);
    }
    for (int i = 0; i < M32C87_NUM_UART; i++) {
        M32C87UART *u = &s->uart[i];
        u->mr = u->brg = 0;
        u->c0 = UC0_TXEPT;
        u->c1 = UC1_TI;
        u->tx_busy = false;
        u->rx_count = 0;
        timer_del(u->tx_timer);
        timer_del(u->rx_timer);
    }
    for (int i = 0; i < M32C87_NUM_CAN; i++) {
        M32C87CAN *c = &s->can[i];
        c->ctlr = CTLR_RESET;
        c->str = STR_RESET;
        c->sstr = c->icr = c->idr = c->conr = c->tsr = 0;
        c->recr = c->tecr = c->sbs = 0;
        memset(c->mctl, 0, sizeof(c->mctl));
        memset(c->slot, 0, sizeof(c->slot));
    }
}

static void m32c87_realize(DeviceState *dev, Error **errp)
{
    M32C87State *s = M32C87_SOC(dev);
    MemoryRegion *sysmem = get_system_memory();

    s->cpu = M32C_CPU(cpu_create(TYPE_M32C80_CPU));
    s->cpu->env.irq_query = m32c87_irq_query;
    s->cpu->env.irq_ack = m32c87_irq_ack;
    s->cpu->env.irq_opaque = s;

    memory_region_init_io(&s->sfr, OBJECT(s), &sfr_ops, s, "m32c87.sfr",
                          M32C87_SFR_SIZE);
    memory_region_add_subregion(sysmem, 0, &s->sfr);
    memory_region_init_ram(&s->ram, OBJECT(s), "m32c87.ram", s->ram_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, M32C87_SFR_SIZE, &s->ram);
    memory_region_init_rom(&s->rom, OBJECT(s), "m32c87.rom", s->rom_size,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0x1000000 - s->rom_size, &s->rom);

    for (int v = 0; v < M32C87_NUM_VEC; v++) {
        s->vec_icr[v] = -1;
    }
    for (size_t i = 0; i < ARRAY_SIZE(m32c87_icr_map); i++) {
        s->vec_icr[m32c87_icr_map[i].vec] = m32c87_icr_map[i].icr;
    }

    for (int i = 0; i < 6; i++) {
        char name[8];
        PinCtx *p = g_new(PinCtx, 1);
        EcuPin *pin;

        p->s = s;
        p->n = i;
        snprintf(name, sizeof(name), "INT%d", i);
        pin = ecu_pin(name);
        ecu_pin_external_drive(pin, 1);
        s->int_level[i] = 1;
        ecu_pin_set_input_handler(pin, int_pin_cb, p);
    }
    ecu_pin_external_drive(ecu_pin("NMI"), 1);
    s->nmi_level = 1;
    ecu_pin_set_input_handler(ecu_pin("NMI"), nmi_pin_cb, s);

    s->wdt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_cb, s);
    s->ad_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ad_cb, s);

    for (int i = 0; i < M32C87_NUM_TA + M32C87_NUM_TB; i++) {
        bool is_b = i >= M32C87_NUM_TA;
        int idx = is_b ? i - M32C87_NUM_TA : i;
        M32C87Timer *t = is_b ? &s->tb[idx] : &s->ta[idx];
        TimerCtx *c = g_new(TimerCtx, 1);
        char name[16];

        t->is_b = is_b;
        t->index = idx;
        c->s = s;
        c->t = t;
        t->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_event, c);
        snprintf(name, sizeof(name), "T%c%dIN", is_b ? 'B' : 'A', idx);
        t->in_pin = ecu_pin(name);
        ecu_pin_set_input_handler(t->in_pin, timer_in_cb, c);
        if (!is_b) {
            snprintf(name, sizeof(name), "TA%dOUT", idx);
            t->out_pin = ecu_pin(name);
        }
    }

    for (int n = 0; n < M32C87_NUM_PORTS; n++) {
        for (int b = 0; b < 8; b++) {
            char name[8];
            snprintf(name, sizeof(name), "P%d_%d", n, b);
            s->port_pin[n][b] = ecu_pin(name);
        }
    }

    for (int i = 0; i < M32C87_NUM_UART; i++) {
        static const int vt[5] = { M32C87_VEC_S0T, M32C87_VEC_S1T,
                                   M32C87_VEC_S2T, M32C87_VEC_S3T,
                                   M32C87_VEC_S4T };
        M32C87UART *u = &s->uart[i];
        u->soc = s;
        u->index = i;
        u->base = uart_base[i];
        u->vec_tx = vt[i];
        u->vec_rx = vt[i] + 1;
        u->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_tx_cb, u);
        u->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_rx_cb, u);
        qemu_chr_fe_set_handlers(&u->chr, uart_can_receive, uart_receive,
                                 NULL, NULL, u, NULL, true);
    }

    for (int i = 0; i < M32C87_NUM_CAN; i++) {
        M32C87CAN *c = &s->can[i];
        c->soc = s;
        c->index = i;
        c->base = can_layout[i].base;
        c->vec_trx = s->can_vec[i][0];
        c->vec_err = s->can_vec[i][1];
        c->canbus = s->canbus[i];
        c->bus_client.info = &m32c87_can_info;
        if (c->canbus &&
            can_bus_insert_client(c->canbus, &c->bus_client) < 0) {
            error_setg(errp, "m32c87: cannot attach CAN%d", i);
            return;
        }
    }

    qemu_register_reset(m32c87_cpu_reset, s);
}

static const Property m32c87_props[] = {
    DEFINE_PROP_UINT32("pclk-hz", M32C87State, pclk_hz, 32000000),
    DEFINE_PROP_UINT32("rom-size", M32C87State, rom_size, 1 * MiB),
    DEFINE_PROP_UINT32("ram-size", M32C87State, ram_size, 48 * KiB),
    DEFINE_PROP_UINT32("avref-mv", M32C87State, avref_mv, 5000),
    DEFINE_PROP_BOOL("wdt-reset", M32C87State, wdt_reset, true),
    DEFINE_PROP_BOOL("kline-echo", M32C87State, kline_echo, false),
    DEFINE_PROP_UINT32("can0-vec", M32C87State, can_vec[0][0], 53),
    DEFINE_PROP_UINT32("can0-err-vec", M32C87State, can_vec[0][1], 57),
    DEFINE_PROP_UINT32("can1-vec", M32C87State, can_vec[1][0], 54),
    DEFINE_PROP_UINT32("can1-err-vec", M32C87State, can_vec[1][1], 58),
    DEFINE_PROP_CHR("uart0", M32C87State, uart[0].chr),
    DEFINE_PROP_CHR("uart1", M32C87State, uart[1].chr),
    DEFINE_PROP_CHR("uart2", M32C87State, uart[2].chr),
    DEFINE_PROP_CHR("uart3", M32C87State, uart[3].chr),
    DEFINE_PROP_CHR("uart4", M32C87State, uart[4].chr),
    DEFINE_PROP_LINK("canbus0", M32C87State, canbus[0], TYPE_CAN_BUS,
                     CanBusState *),
    DEFINE_PROP_LINK("canbus1", M32C87State, canbus[1], TYPE_CAN_BUS,
                     CanBusState *),
};

static void m32c87_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = m32c87_realize;
    dc->user_creatable = false;
    rc->phases.hold = m32c87_reset_hold;
    device_class_set_props(dc, m32c87_props);
}

static const TypeInfo m32c87_info = {
    .name = TYPE_M32C87_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(M32C87State),
    .class_init = m32c87_class_init,
};

static void m32c87_register_types(void)
{
    type_register_static(&m32c87_info);
}

type_init(m32c87_register_types)
