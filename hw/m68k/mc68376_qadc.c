/*
 * MC68376 queued analog-to-digital converter (QADC).
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D), section 8 and
 * appendix D.5.  Section and table numbers below refer to that manual.
 *
 * Model:
 *  - two conversion queues in the 40-entry CCW table (queue 1 from CCW0,
 *    queue 2 from BQ2), all queue operating modes of Tables D-25/D-26
 *    (software, external trigger and periodic/interval timer, single and
 *    continuous scan), pause/sub-queues, the end-of-queue conditions of
 *    8.12.2, queue 1 priority with queue 2 trigger pending / suspension
 *    and the RES resume point (Table 8-3), trigger overruns;
 *  - conversion timing from QCLK (PSH/PSL, 8.12.4) and the CCW input
 *    sample time / amplifier bypass (8.11.1): 16 + IST QCLKs, 10 + IST
 *    with BYP; the periodic/interval timer counts 2^7..2^17 QCLKs;
 *  - 10-bit results (ecu_analog_read("AN<n>") referred to VRH, the SoC
 *    "vrh-mv" property, VRL = 0 V) readable right justified, left
 *    justified signed and left justified unsigned (D.5.9);
 *  - special channels 60 (VRL), 61 (VRH), 62 (VDDA/2, VDDA taken equal to
 *    VRH); invalid/reserved channels convert VRL (D.5.8); external
 *    multiplexing (MUX): channels 0-31 read "AN0".."AN31" and the channel
 *    address is driven on MA[2:0] = PQA[2:0] (8.9);
 *  - ports QA/QB as ECU pins PQA0..PQA7 (open drain outputs, a released
 *    output reads high) and PQB0..PQB7; ETRIG1/ETRIG2 are the PQA3/PQA4
 *    pins (either name may be driven).  An input reads the ECU pin level
 *    OR'ed with "analog voltage >= VRH/2" so analog-only signals also have
 *    a digital value;
 *  - four interrupt sources (queue 1/2 completion/pause, 8.13) through the
 *    IMB with levels IRLQ1/IRLQ2, vector IVB[7:2] + source code and the
 *    QADCMCR IARB.
 *
 * Not modelled: freeze mode (no BDM), supervisor/user access restriction
 * (SUPV), conversion accuracy/sample-and-hold effects (the voltage is read
 * at the end of the conversion), wait states.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/m68k/mc68376.h"

/* QADCMCR (D.5.1) */
#define QADCMCR_STOP    0x8000
#define QADCMCR_WMASK   0xc08f      /* STOP FRZ SUPV IARB */

/* QACR0 (D.5.6) */
#define QACR0_MUX       0x8000
#define QACR0_WMASK     0x81ff      /* MUX PSH PSA PSL */

/* QACR1 / QACR2 (D.5.6) */
#define QACR_CIE        0x8000
#define QACR_PIE        0x4000
#define QACR_SSE        0x2000
#define QACR1_WMASK     0xc700      /* CIE1 PIE1 MQ1 (SSE1 reads 0) */
#define QACR2_WMASK     0xdfbf      /* CIE2 PIE2 MQ2 RES BQ2 */
#define QACR2_RES       0x0080

/* QASR (D.5.7) */
#define QASR_CF1        0x8000
#define QASR_PF1        0x4000
#define QASR_CF2        0x2000
#define QASR_PF2        0x1000
#define QASR_TOR1       0x0800
#define QASR_TOR2       0x0400
#define QASR_FLAGS      0xfc00

/* CCW (D.5.8) */
#define CCW_P           0x0200
#define CCW_BYP         0x0100
#define CCW_IST(v)      (((v) >> 6) & 3)
#define CCW_CHAN(v)     ((v) & 0x3f)
#define CCW_MASK        0x03ff

#define QADC_NCCW       40
#define CHAN_EOQ        63

/* register offsets inside the module (Table D-24) */
#define R_MCR           0x00
#define R_TEST          0x02
#define R_INT           0x04
#define R_PORT          0x06
#define R_DDR           0x08
#define R_ACR0          0x0a
#define R_ACR1          0x0c
#define R_ACR2          0x0e
#define R_ASR           0x10
#define R_CCW           0x30
#define R_RJURR         0xb0
#define R_LJSRR         0x130
#define R_LJURR         0x1b0

enum { QS_IDLE, QS_PAUSED, QS_ACTIVE, QS_PENDING, QS_SUSPENDED };

/* decoded queue operating mode */
enum {
    QM_DISABLED,
    QM_SW_SINGLE,
    QM_EXT_SINGLE,
    QM_TIMER_SINGLE,    /* interval timer single-scan (queue 2) */
    QM_SW_CONT,
    QM_EXT_CONT,
    QM_TIMER_CONT,      /* periodic timer continuous-scan (queue 2) */
    QM_RESERVED,
};

typedef struct QadcMode {
    int kind;
    bool falling;       /* external trigger edge */
    int timer_exp;      /* periodic/interval timer: 2^n QCLKs */
} QadcMode;

typedef struct QadcQueue {
    int state;
    bool sse;           /* single-scan enable (reads as zero) */
    int ptr;            /* next CCW */
    int sub_start;      /* first CCW of the current sub-queue */
    int resume;         /* CCW aborted by a queue 1 trigger */
    QadcMode mode;
    IMBIrq irq_cf, irq_pf;
} QadcQueue;

struct MC68376QADC {
    MC68376State *soc;

    uint16_t mcr, test, intr, acr0, acr1, acr2, asr;
    uint16_t asr_armed;         /* QASR flags read as one */
    bool ivb_written;
    uint8_t portqa, ddrqa;
    uint8_t pa_level, pb_level; /* input levels seen on the pins */
    uint8_t ma;                 /* multiplexed address latch MA[2:0] */
    uint16_t ccw[QADC_NCCW];
    uint16_t result[QADC_NCCW];
    int cwp;

    QadcQueue q[3];             /* index 1, 2 */

    /* converter */
    int conv_q;                 /* queue being converted, 0 = idle */
    int conv_idx;
    QEMUTimer *conv_timer;
    QEMUTimer *itrig_timer;     /* internal (software mode) triggers */
    int64_t itrig_at[3];

    /* periodic/interval timer (8.12.5) */
    QEMUTimer *pit_timer;
    bool pit_running;
    int64_t pit_start_ns;
    uint64_t pit_k;

    EcuPin *pqa[8], *pqb[8], *etrig[2];
};

static inline uint16_t merge16(uint16_t old, uint16_t val, uint16_t mask)
{
    return (old & ~mask) | (val & mask);
}

/* ---------------------------------------------------------------------- */
/* Clocks (8.12.4)                                                         */
/* ---------------------------------------------------------------------- */

/* QCLK period in system clocks: high (1 + PSH) + low (1 + PSL) */
static uint32_t qclk_cycles(MC68376QADC *m)
{
    return ((m->acr0 >> 4) & 0x1f) + (m->acr0 & 7) + 2;
}

static int64_t cycles_ns(MC68376QADC *m, uint64_t cycles)
{
    uint32_t hz = mc68376_sysclk_hz(m->soc);

    return hz ? (int64_t)muldiv64(cycles, NANOSECONDS_PER_SECOND, hz) : 0;
}

static int64_t qclk_ns(MC68376QADC *m, uint64_t n)
{
    return cycles_ns(m, n * qclk_cycles(m));
}

/* conversion time in QCLKs (8.11.1, 8.11.1.1) */
static int conv_qclks(uint16_t ccw)
{
    int ist = 2 << CCW_IST(ccw);

    return (ccw & CCW_BYP) ? 10 + ist : 16 + ist;
}

/* ---------------------------------------------------------------------- */
/* Interrupts (8.13)                                                       */
/* ---------------------------------------------------------------------- */

static void qadc_update_irq(MC68376QADC *m)
{
    uint8_t ivb = m->ivb_written ? (m->intr & 0xfc) : 0x0f;
    uint8_t iarb = m->mcr & 0xf;
    static const uint16_t cie[3] = { 0, QASR_CF1, QASR_CF2 };
    static const uint16_t pie[3] = { 0, QASR_PF1, QASR_PF2 };

    for (int n = 1; n <= 2; n++) {
        QadcQueue *q = &m->q[n];
        uint16_t acr = n == 1 ? m->acr1 : m->acr2;
        uint8_t level = n == 1 ? (m->intr >> 12) & 7 : (m->intr >> 8) & 7;

        q->irq_cf.level = q->irq_pf.level = level;
        q->irq_cf.iarb = q->irq_pf.iarb = iarb;
        /* Figure 8-11: %11 Q1 completion, %10 Q1 pause, %01, %00 Q2 */
        q->irq_cf.vector = m->ivb_written ? ivb | (n == 1 ? 3 : 1) : ivb;
        q->irq_pf.vector = m->ivb_written ? ivb | (n == 1 ? 2 : 0) : ivb;
        q->irq_cf.pending = level && (acr & QACR_CIE) && (m->asr & cie[n]);
        q->irq_pf.pending = level && (acr & QACR_PIE) && (m->asr & pie[n]);
    }
    imb_irq_update(m->soc);
}

/* ---------------------------------------------------------------------- */
/* Ports (8.8, D.5.4, D.5.5)                                               */
/* ---------------------------------------------------------------------- */

static bool ext_trigger_mode(QadcMode *mode)
{
    return mode->kind == QM_EXT_SINGLE || mode->kind == QM_EXT_CONT;
}

/* port A pins driven by the module: DDRQA outputs and MA[2:0] */
static uint8_t pa_outputs(MC68376QADC *m)
{
    uint8_t out = m->ddrqa;

    if (m->acr0 & QACR0_MUX) {
        out |= 0x07;
    }
    if (ext_trigger_mode(&m->q[1].mode)) {
        out &= ~0x08;
    }
    if (ext_trigger_mode(&m->q[2].mode)) {
        out &= ~0x10;
    }
    return out;
}

static uint8_t pa_latch(MC68376QADC *m)
{
    if (m->acr0 & QACR0_MUX) {
        return (m->portqa & ~0x07) | (m->ma & 7);
    }
    return m->portqa;
}

static void pa_drive(MC68376QADC *m)
{
    uint8_t out = pa_outputs(m), latch = pa_latch(m);

    for (int b = 0; b < 8; b++) {
        if (out & (1 << b)) {
            /* open drain: a one releases the pin (external pull-up) */
            ecu_pin_mcu_drive(m->pqa[b], (latch >> b) & 1);
        }
    }
}

static double vrh_volts(MC68376QADC *m)
{
    return m->soc->vrh_mv / 1000.0;
}

static const uint8_t pa_chan[8] = { 52, 53, 54, 55, 56, 57, 58, 59 };
static const uint8_t pb_chan[8] = { 0, 1, 2, 3, 48, 49, 50, 51 };

static bool analog_high(MC68376QADC *m, int chan)
{
    char name[8];

    snprintf(name, sizeof(name), "AN%d", chan);
    return ecu_analog_read(name) >= vrh_volts(m) / 2;
}

static uint8_t read_porta(MC68376QADC *m)
{
    uint8_t out = pa_outputs(m), latch = pa_latch(m), v = 0;

    for (int b = 0; b < 8; b++) {
        if (out & (1 << b)) {
            v |= latch & (1 << b);
        } else if ((m->pa_level & (1 << b)) || analog_high(m, pa_chan[b])) {
            v |= 1 << b;
        }
    }
    return v;
}

static uint8_t read_portb(MC68376QADC *m)
{
    uint8_t v = 0;

    for (int b = 0; b < 8; b++) {
        if ((m->pb_level & (1 << b)) || analog_high(m, pb_chan[b])) {
            v |= 1 << b;
        }
    }
    return v;
}

/* ---------------------------------------------------------------------- */
/* Queue operating modes (Tables D-25, D-26)                               */
/* ---------------------------------------------------------------------- */

static QadcMode decode_mode(int n, uint16_t acr)
{
    QadcMode md = { QM_RESERVED, false, 0 };
    int mq = n == 1 ? (acr >> 8) & 7 : (acr >> 8) & 0x1f;

    if (n == 1) {
        static const int k1[8] = {
            QM_DISABLED, QM_SW_SINGLE, QM_EXT_SINGLE, QM_EXT_SINGLE,
            QM_RESERVED, QM_SW_CONT, QM_EXT_CONT, QM_EXT_CONT,
        };
        md.kind = k1[mq];
        md.falling = mq == 3 || mq == 7;
        return md;
    }
    if (mq == 0) {
        md.kind = QM_DISABLED;
    } else if (mq == 1) {
        md.kind = QM_SW_SINGLE;
    } else if (mq == 2 || mq == 3) {
        md.kind = QM_EXT_SINGLE;
        md.falling = mq == 3;
    } else if (mq >= 4 && mq <= 14) {
        md.kind = QM_TIMER_SINGLE;
        md.timer_exp = 7 + mq - 4;
    } else if (mq == 17) {
        md.kind = QM_SW_CONT;
    } else if (mq == 18 || mq == 19) {
        md.kind = QM_EXT_CONT;
        md.falling = mq == 19;
    } else if (mq >= 20 && mq <= 30) {
        md.kind = QM_TIMER_CONT;
        md.timer_exp = 7 + mq - 20;
    }
    return md;
}

static bool single_scan(QadcMode *md)
{
    return md->kind == QM_SW_SINGLE || md->kind == QM_EXT_SINGLE ||
           md->kind == QM_TIMER_SINGLE;
}

static bool sw_mode(QadcMode *md)
{
    return md->kind == QM_SW_SINGLE || md->kind == QM_SW_CONT;
}

static int bq2(MC68376QADC *m)
{
    return m->acr2 & 0x3f;
}

static int queue_start(MC68376QADC *m, int n)
{
    return n == 1 ? 0 : bq2(m);
}

/* end-of-queue conditions (8.12.2, 8.12.7) */
static bool is_eoq(MC68376QADC *m, int n, int idx)
{
    return idx >= QADC_NCCW || (n == 1 && idx >= bq2(m)) ||
           CCW_CHAN(m->ccw[idx]) == CHAN_EOQ;
}

/* ---------------------------------------------------------------------- */
/* Periodic/interval timer (8.12.5)                                        */
/* ---------------------------------------------------------------------- */

static int64_t pit_period_ns(MC68376QADC *m)
{
    return qclk_ns(m, 1ULL << m->q[2].mode.timer_exp);
}

static void pit_schedule(MC68376QADC *m)
{
    int64_t p = pit_period_ns(m);

    if (!m->pit_running || p <= 0) {
        timer_del(m->pit_timer);
        return;
    }
    timer_mod(m->pit_timer, m->pit_start_ns + (int64_t)(m->pit_k + 1) * p);
}

static void pit_start(MC68376QADC *m)
{
    m->pit_running = true;
    m->pit_start_ns = mc68376_now();
    m->pit_k = 0;
    pit_schedule(m);
}

static void pit_stop(MC68376QADC *m)
{
    m->pit_running = false;
    timer_del(m->pit_timer);
}

/* ---------------------------------------------------------------------- */
/* Conversion sequencing (8.12)                                            */
/* ---------------------------------------------------------------------- */

static void qadc_run(MC68376QADC *m);
static void qadc_trigger(MC68376QADC *m, int n, bool external);

static void itrig_schedule(MC68376QADC *m)
{
    int64_t t = INT64_MAX;

    for (int n = 1; n <= 2; n++) {
        if (m->itrig_at[n]) {
            t = MIN(t, m->itrig_at[n]);
        }
    }
    if (t == INT64_MAX) {
        timer_del(m->itrig_timer);
    } else {
        timer_mod(m->itrig_timer, t);
    }
}

/* software modes: the next internal trigger comes two QCLKs later */
static void itrig_after_pause(MC68376QADC *m, int n)
{
    m->itrig_at[n] = mc68376_now() + MAX(qclk_ns(m, 2), 1);
    itrig_schedule(m);
}

static void itrig_cb(void *opaque)
{
    MC68376QADC *m = opaque;
    int64_t now = mc68376_now();

    for (int n = 1; n <= 2; n++) {
        if (m->itrig_at[n] && m->itrig_at[n] <= now) {
            m->itrig_at[n] = 0;
            qadc_trigger(m, n, false);
        }
    }
    itrig_schedule(m);
}

static void abort_conversion(MC68376QADC *m)
{
    m->conv_q = 0;
    timer_del(m->conv_timer);
}

static void queue_complete(MC68376QADC *m, int n)
{
    QadcQueue *q = &m->q[n];

    m->asr |= n == 1 ? QASR_CF1 : QASR_CF2;
    q->state = QS_IDLE;
    q->ptr = q->sub_start = queue_start(m, n);
    if (single_scan(&q->mode)) {
        q->sse = false;
        if (q->mode.kind == QM_TIMER_SINGLE) {
            pit_stop(m);            /* timer held in reset */
        }
    }
    if (q->mode.kind == QM_SW_CONT) {
        itrig_after_pause(m, n);
    }
}

static uint16_t convert(MC68376QADC *m, int chan)
{
    double vrh = vrh_volts(m), v = 0.0;
    bool mux = m->acr0 & QACR0_MUX;
    char name[8];
    int b;

    if ((chan <= 3 && !mux) || (chan <= 31 && mux) ||
        (chan >= 48 && chan <= 59 && !(mux && chan >= 52 && chan <= 54))) {
        b = chan >= 52 && chan <= 59 ? chan - 52 : -1;
        if (b >= 0 && (pa_outputs(m) & (1 << b))) {
            /* an output pin converts the level of its driver (8.8.2) */
            v = (pa_latch(m) >> b) & 1 ? vrh : 0.0;
        } else {
            snprintf(name, sizeof(name), "AN%d", chan);
            v = ecu_analog_read(name);
        }
    } else if (chan == 61) {
        v = vrh;
    } else if (chan == 62) {
        v = vrh / 2;
    }
    /* channel 60 (VRL), invalid and reserved channels: VRL (D.5.8) */
    return ecu_adc_convert(v, vrh, 10);
}

static void start_conversion(MC68376QADC *m, int n, int idx)
{
    uint16_t ccw = m->ccw[idx];
    int chan = CCW_CHAN(ccw);

    m->conv_q = n;
    m->conv_idx = idx;
    m->cwp = idx;
    if ((m->acr0 & QACR0_MUX) && chan <= 31) {
        /* MA[2:0] select the input of the external multiplexer */
        m->ma = (chan >> 1) & 7;
        pa_drive(m);
    }
    timer_mod(m->conv_timer, mc68376_now() +
              MAX(qclk_ns(m, conv_qclks(ccw)), 1));
}

/* start the highest priority runnable queue if the converter is free */
static void qadc_run(MC68376QADC *m)
{
    for (;;) {
        QadcQueue *q1 = &m->q[1], *q2 = &m->q[2];
        int n;

        if (m->conv_q || (m->mcr & QADCMCR_STOP)) {
            return;
        }
        if (q1->state == QS_ACTIVE) {
            n = 1;
        } else if (q2->state == QS_ACTIVE || q2->state == QS_PENDING ||
                   q2->state == QS_SUSPENDED) {
            if (q2->state == QS_SUSPENDED) {
                q2->ptr = (m->acr2 & QACR2_RES) ? q2->resume : q2->sub_start;
            }
            q2->state = QS_ACTIVE;
            n = 2;
        } else {
            return;
        }
        if (is_eoq(m, n, m->q[n].ptr)) {
            m->cwp = MIN(m->q[n].ptr, QADC_NCCW - 1);
            queue_complete(m, n);
            continue;
        }
        start_conversion(m, n, m->q[n].ptr);
        return;
    }
}

static void conv_cb(void *opaque)
{
    MC68376QADC *m = opaque;
    int n = m->conv_q, idx = m->conv_idx;
    QadcQueue *q;
    uint16_t ccw;

    if (!n) {
        return;
    }
    q = &m->q[n];
    ccw = m->ccw[idx];
    m->result[idx] = convert(m, CCW_CHAN(ccw));
    m->conv_q = 0;
    q->ptr = idx + 1;

    if (ccw & CCW_P) {
        m->asr |= n == 1 ? QASR_PF1 : QASR_PF2;
        q->sub_start = q->ptr;
        if (is_eoq(m, n, q->ptr)) {
            /* pause followed by end of queue: completion, idle (8.12.2) */
            queue_complete(m, n);
        } else {
            q->state = QS_PAUSED;
            if (sw_mode(&q->mode)) {
                itrig_after_pause(m, n);
            }
        }
    } else if (is_eoq(m, n, q->ptr)) {
        queue_complete(m, n);
    }
    qadc_run(m);
    qadc_update_irq(m);
}

/*
 * A trigger event for queue @n (Table 8-3).  @external: an ETRIG edge or
 * the periodic/interval timer, which can be recorded as an overrun.
 */
static void qadc_trigger(MC68376QADC *m, int n, bool external)
{
    QadcQueue *q = &m->q[n];

    if (m->mcr & QADCMCR_STOP || q->mode.kind == QM_DISABLED ||
        q->mode.kind == QM_RESERVED) {
        return;
    }
    if (single_scan(&q->mode) && !q->sse) {
        return;
    }
    if (n == 1) {
        if (q->state == QS_ACTIVE) {
            if (external) {
                m->asr |= QASR_TOR1;
            }
            return;
        }
        q->state = QS_ACTIVE;
        if (m->conv_q == 2) {
            /* queue 2 is suspended, its conversion aborted */
            m->q[2].resume = m->conv_idx;
            m->q[2].state = QS_SUSPENDED;
            abort_conversion(m);
        }
    } else {
        if (q->state == QS_ACTIVE || q->state == QS_PENDING ||
            q->state == QS_SUSPENDED) {
            if (external) {
                m->asr |= QASR_TOR2;
            }
            return;
        }
        q->state = m->q[1].state == QS_ACTIVE ? QS_PENDING : QS_ACTIVE;
    }
    qadc_run(m);
    qadc_update_irq(m);
}

static void pit_cb(void *opaque)
{
    MC68376QADC *m = opaque;

    m->pit_k++;
    pit_schedule(m);
    qadc_trigger(m, 2, true);
}

/* QACR1/QACR2 written: mode changes abort the queue (8.12.3.2, 8.12.7) */
static void queue_control(MC68376QADC *m, int n, uint16_t old, bool sse)
{
    QadcQueue *q = &m->q[n];
    uint16_t acr = n == 1 ? m->acr1 : m->acr2;
    int mqmask = n == 1 ? 0x0700 : 0x1f00;
    QadcMode md = decode_mode(n, acr);
    bool changed = (old ^ acr) & mqmask;

    if (changed) {
        if (m->conv_q == n) {
            abort_conversion(m);
        }
        q->mode = md;
        q->state = QS_IDLE;
        q->ptr = q->sub_start = queue_start(m, n);
        q->sse = sse;
        m->itrig_at[n] = 0;
        itrig_schedule(m);
        if (n == 2) {
            if (md.kind == QM_TIMER_CONT ||
                (md.kind == QM_TIMER_SINGLE && sse)) {
                pit_start(m);       /* includes the pulsed reset */
            } else {
                pit_stop(m);
            }
        }
        pa_drive(m);
        if (md.kind == QM_SW_CONT || (md.kind == QM_SW_SINGLE && sse)) {
            qadc_trigger(m, n, false);
        }
        qadc_run(m);
        return;
    }
    if (sse && !q->sse && single_scan(&q->mode) && q->state == QS_IDLE) {
        q->sse = true;
        if (q->mode.kind == QM_SW_SINGLE) {
            qadc_trigger(m, n, false);
        } else if (q->mode.kind == QM_TIMER_SINGLE && !m->pit_running) {
            pit_start(m);
        }
    }
}

/* ---------------------------------------------------------------------- */
/* External trigger and port pins                                          */
/* ---------------------------------------------------------------------- */

typedef struct QadcPinCtx {
    MC68376QADC *m;
    int port;           /* 0 = A, 1 = B */
    int bit;
} QadcPinCtx;

static void qadc_pin_cb(void *opaque, int level)
{
    QadcPinCtx *ctx = opaque;
    MC68376QADC *m = ctx->m;
    uint8_t *lv = ctx->port ? &m->pb_level : &m->pa_level;
    int old = (*lv >> ctx->bit) & 1;

    *lv = (*lv & ~(1 << ctx->bit)) | (!!level << ctx->bit);
    if (ctx->port == 0 && (ctx->bit == 3 || ctx->bit == 4) && old != !!level) {
        int n = ctx->bit - 2;       /* ETRIG1 = PQA3, ETRIG2 = PQA4 */
        QadcMode *md = &m->q[n].mode;

        if (ext_trigger_mode(md) && md->falling == !level) {
            qadc_trigger(m, n, true);
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Register interface (D.5)                                                */
/* ---------------------------------------------------------------------- */

static void qadc_stop_reset(MC68376QADC *m)
{
    /* low-power stop resets QACR0-2 and QASR, aborts conversions (8.6.1) */
    abort_conversion(m);
    pit_stop(m);
    m->itrig_at[1] = m->itrig_at[2] = 0;
    itrig_schedule(m);
    m->acr0 = 0x0033;
    m->acr1 = 0;
    m->acr2 = 0x0027;
    m->asr = m->asr_armed = 0;
    for (int n = 1; n <= 2; n++) {
        m->q[n].mode = decode_mode(n, n == 1 ? m->acr1 : m->acr2);
        m->q[n].state = QS_IDLE;
        m->q[n].sse = false;
        m->q[n].ptr = m->q[n].sub_start = queue_start(m, n);
    }
    m->cwp = 0;
}

static uint16_t qs_field(MC68376QADC *m)
{
    static const uint8_t q1[] = {
        [QS_IDLE] = 0, [QS_PAUSED] = 1, [QS_ACTIVE] = 2,
    };
    static const uint8_t q2[] = {
        [QS_IDLE] = 0, [QS_PAUSED] = 1, [QS_ACTIVE] = 2,
        [QS_PENDING] = 3, [QS_SUSPENDED] = 2,
    };

    return (q1[m->q[1].state] << 2) | q2[m->q[2].state];
}

static uint16_t qadc_read16(MC68376QADC *m, hwaddr off)
{
    bool stopped = m->mcr & QADCMCR_STOP;
    int i;

    if (off >= R_CCW && off < R_CCW + 2 * QADC_NCCW) {
        return stopped ? 0 : m->ccw[(off - R_CCW) / 2] & CCW_MASK;
    }
    if (off >= R_RJURR && off < R_RJURR + 2 * QADC_NCCW) {
        i = (off - R_RJURR) / 2;
        return stopped ? 0 : m->result[i];
    }
    if (off >= R_LJSRR && off < R_LJSRR + 2 * QADC_NCCW) {
        i = (off - R_LJSRR) / 2;
        return stopped ? 0 : ((m->result[i] << 6) ^ 0x8000);
    }
    if (off >= R_LJURR && off < R_LJURR + 2 * QADC_NCCW) {
        i = (off - R_LJURR) / 2;
        return stopped ? 0 : m->result[i] << 6;
    }
    switch (off) {
    case R_MCR:
        return m->mcr;
    case R_TEST:
        return m->test;
    case R_INT:
        return m->ivb_written ? m->intr & 0x77fc : m->intr & 0x77ff;
    case R_PORT:
        return (read_porta(m) << 8) | read_portb(m);
    case R_DDR:
        return m->ddrqa << 8;
    case R_ACR0:
        return m->acr0;
    case R_ACR1:
        return m->acr1;
    case R_ACR2:
        return m->acr2;
    case R_ASR:
        m->asr_armed |= m->asr & QASR_FLAGS;
        return (m->asr & QASR_FLAGS) | (qs_field(m) << 6) | m->cwp;
    }
    return 0;           /* reserved: reads zero */
}

static void qadc_write16(MC68376QADC *m, hwaddr off, uint16_t val,
                         uint16_t mask)
{
    bool stopped = m->mcr & QADCMCR_STOP;
    uint16_t old;
    int i;

    if (off >= R_CCW && off < R_CCW + 2 * QADC_NCCW) {
        i = (off - R_CCW) / 2;
        if (!stopped) {
            m->ccw[i] = merge16(m->ccw[i], val, mask & CCW_MASK);
        }
        return;
    }
    if ((off >= R_RJURR && off < R_RJURR + 2 * QADC_NCCW) ||
        (off >= R_LJSRR && off < R_LJSRR + 2 * QADC_NCCW) ||
        (off >= R_LJURR && off < R_LJURR + 2 * QADC_NCCW)) {
        /* all result writes are right justified (8.12.8) */
        i = ((off - R_RJURR) & 0x7f) / 2;
        if (!stopped) {
            m->result[i] = merge16(m->result[i], val, mask & 0x3ff);
        }
        return;
    }

    switch (off) {
    case R_MCR:
        old = m->mcr;
        m->mcr = merge16(m->mcr, val, mask & QADCMCR_WMASK);
        if ((m->mcr & QADCMCR_STOP) && !(old & QADCMCR_STOP)) {
            qadc_stop_reset(m);
        }
        break;
    case R_TEST:
        m->test = merge16(m->test, val, mask);
        break;
    case R_INT:
        m->intr = merge16(m->intr, val, mask & 0x77fc);
        if (mask & 0x00ff) {
            m->ivb_written = true;
        }
        break;
    case R_PORT:
        if (mask & 0xff00) {
            m->portqa = val >> 8;     /* port B is input only */
            pa_drive(m);
        }
        break;
    case R_DDR:
        if (mask & 0xff00) {
            m->ddrqa = val >> 8;
            pa_drive(m);
        }
        break;
    case R_ACR0:
        if (!stopped) {
            m->acr0 = merge16(m->acr0, val, mask & QACR0_WMASK);
            pa_drive(m);
        }
        break;
    case R_ACR1:
        if (!stopped) {
            old = m->acr1;
            m->acr1 = merge16(m->acr1, val, mask & QACR1_WMASK);
            queue_control(m, 1, old, (mask & QACR_SSE) && (val & QACR_SSE));
        }
        break;
    case R_ACR2:
        if (!stopped) {
            old = m->acr2;
            m->acr2 = merge16(m->acr2, val, mask & QACR2_WMASK);
            if (((old ^ m->acr2) & 0x3f) && m->q[2].state == QS_IDLE) {
                m->q[2].ptr = m->q[2].sub_start = bq2(m);
            }
            queue_control(m, 2, old, (mask & QACR_SSE) && (val & QACR_SSE));
        }
        break;
    case R_ASR:
        if (!stopped) {
            /* flags clear by writing zero after reading one (8.12.6.4) */
            uint16_t clr = ~val & mask & m->asr_armed & QASR_FLAGS;
            m->asr &= ~clr;
            m->asr_armed &= ~(mask & QASR_FLAGS);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.qadc: write to reserved "
                      "register $%03x\n", (unsigned)off);
        break;
    }
    qadc_update_irq(m);
}

static uint64_t qadc_read(void *opaque, hwaddr addr, unsigned size)
{
    uint16_t v = qadc_read16(opaque, addr & ~1);

    if (size == 2) {
        return v;
    }
    return addr & 1 ? v & 0xff : v >> 8;
}

static void qadc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    if (size == 2) {
        qadc_write16(opaque, addr, val, 0xffff);
    } else if (addr & 1) {
        qadc_write16(opaque, addr & ~1, val & 0xff, 0x00ff);
    } else {
        qadc_write16(opaque, addr, (val & 0xff) << 8, 0xff00);
    }
}

static const MemoryRegionOps qadc_ops = {
    .read = qadc_read,
    .write = qadc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

void mc68376_qadc_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    MC68376QADC *m = g_new0(MC68376QADC, 1);
    char name[16];

    m->soc = s;
    s->qadc = m;
    memory_region_init_io(mr, OBJECT(s), &qadc_ops, m, "mc68376.qadc",
                          MC68376_QADC_SIZE);

    m->conv_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, conv_cb, m);
    m->itrig_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, itrig_cb, m);
    m->pit_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pit_cb, m);

    /* the order sets the priority among equal levels and IARB */
    imb_irq_register(s, &m->q[1].irq_cf, "qadc-cf1");
    imb_irq_register(s, &m->q[1].irq_pf, "qadc-pf1");
    imb_irq_register(s, &m->q[2].irq_cf, "qadc-cf2");
    imb_irq_register(s, &m->q[2].irq_pf, "qadc-pf2");

    for (int b = 0; b < 8; b++) {
        QadcPinCtx *ca = g_new0(QadcPinCtx, 1);
        QadcPinCtx *cb = g_new0(QadcPinCtx, 1);

        *ca = (QadcPinCtx) { m, 0, b };
        *cb = (QadcPinCtx) { m, 1, b };
        snprintf(name, sizeof(name), "PQA%d", b);
        m->pqa[b] = ecu_pin(name);
        ecu_pin_set_input_handler(m->pqa[b], qadc_pin_cb, ca);
        snprintf(name, sizeof(name), "PQB%d", b);
        m->pqb[b] = ecu_pin(name);
        ecu_pin_set_input_handler(m->pqb[b], qadc_pin_cb, cb);
        m->pa_level |= ecu_pin_level(m->pqa[b]) << b;
        m->pb_level |= ecu_pin_level(m->pqb[b]) << b;
        if (b == 3 || b == 4) {
            /* ETRIG1/ETRIG2 are other names of PQA3/PQA4 */
            snprintf(name, sizeof(name), "ETRIG%d", b - 2);
            m->etrig[b - 3] = ecu_pin(name);
            ecu_pin_set_input_handler(m->etrig[b - 3], qadc_pin_cb, ca);
        }
    }
}

void mc68376_qadc_reset(MC68376State *s)
{
    MC68376QADC *m = s->qadc;

    m->mcr = 0x0080;                /* SUPV = 1 (D.5.1) */
    m->test = 0;
    m->intr = 0x000f;               /* IVB = $0F (8.13.3) */
    m->ivb_written = false;
    m->ddrqa = 0;                   /* PORTQA/PORTQB are not reset */
    m->ma = 0;
    qadc_stop_reset(m);
    m->q[1].irq_cf.pending = m->q[1].irq_pf.pending = false;
    m->q[2].irq_cf.pending = m->q[2].irq_pf.pending = false;
    qadc_update_irq(m);
}
