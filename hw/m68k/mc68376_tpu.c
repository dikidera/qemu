/*
 * MC68376 time processor unit (TPU, section 11 and appendix D.8).
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D).  Section and
 * table numbers below refer to it.
 *
 * Host interface (D.8): TPUMCR, TICR, CIER, CFSR0-3, HSQR0/1, HSRR0/1,
 * CPR0/1, CISR, DSCR/DSSR (stored), the factory test registers (read
 * zero) and the parameter RAM ($YFFF00, 100 words: six per channel for
 * channels 0-13, eight for channels 14 and 15, D.8.15).  Interrupts go
 * through one IMB source: level CIRL, arbitration IARB (TPUMCR), vector
 * CIBV:channel (11.3.7).
 *
 * Time bases (11.6.1): TCR1 = fsys / (PSCK ? 4 : 32) / 2^TCR1P, derived
 * from the virtual clock.  TCR2 counts T2CLK pin rising edges / 2^TCR2P
 * (T2CG = 0) or fsys / 8 / 2^TCR2P while T2CLK is high (T2CG = 1).
 *
 * Channels: the hardware part of each channel (pin direction, match
 * register with greater-or-equal comparator, capture register,
 * transition / match / link / host service latches) is modelled the way
 * section 11.2-11.3 describes it, and the factory A mask set ROM
 * functions are behavioural C models that run as "microcode" when the
 * scheduler services a channel.  Service latency is zero: a request is
 * serviced at the virtual time of the event that caused it (requests
 * of one instant are served in priority order, high > middle > low,
 * then by channel number), so the HSR field of a channel with a
 * non-zero priority reads %00 again right after the host writes it.
 * Matches are computed as exact virtual times (never from the expiry
 * time of the QEMU timer that noticed them) and output edges are
 * reported with ecu_pin_mcu_drive_at().
 *
 * IMPORTANT: the user's manual does not describe the parameter RAM
 * layouts, host sequence and host service request encodings of the ROM
 * functions; it refers to the TPU programming notes TPUPN11-TPUPN20,
 * which were not available.  The function models follow second-hand
 * summaries of those notes; every detail that could not be confirmed is
 * marked "UNVERIFIED" below and listed in docs/system/ecu-emulator.rst.
 *
 * Not modelled: the stepper motor function (SM, $D; its interface is
 * not documented in the available sources), TPU microcode emulation
 * (TPUMCR.EMU: the microinstruction format is not public; the TPURAM is
 * handed over to the TPU but nothing executes it), function codes $0-$5
 * (no ROM function), the development support functions (breakpoints,
 * FREEZE, single step), the T2CLK digital filter and real microcode
 * execution times.
 *
 * Pins: "TPU0".."TPU15" (channels), "T2CLK" (TCR2 clock / gate input).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/m68k/mc68376.h"

#define TPU_NCH         16
#define TPU_PRAM_OFF    0x100

/* register offsets (Table D-51) */
#define R_TPUMCR        0x00
#define R_TCR           0x02
#define R_DSCR          0x04
#define R_DSSR          0x06
#define R_TICR          0x08
#define R_CIER          0x0a
#define R_CFSR0         0x0c
#define R_HSQR0         0x14
#define R_HSRR0         0x18
#define R_CPR0          0x1c
#define R_CISR          0x20
#define R_LR            0x22
#define R_SGLR          0x24
#define R_DCNR          0x26

/* TPUMCR (D.8.1) */
#define MCR_STOP        0x8000
#define MCR_TCR1P(v)    (((v) >> 13) & 3)
#define MCR_TCR2P(v)    (((v) >> 11) & 3)
#define MCR_EMU         0x0400
#define MCR_T2CG        0x0200
#define MCR_STF         0x0100
#define MCR_SUPV        0x0080
#define MCR_PSCK        0x0040
#define MCR_WMASK       0xfecf      /* all but STF and the zero bits 5:4 */

#define TICR_CIRL(v)    (((v) >> 8) & 7)
#define TICR_WMASK      0x07f0

/* A mask set function codes (Table 11-3) */
enum {
    FN_QDEC = 0x6,
    FN_SPWM = 0x7,
    FN_DIO = 0x8,
    FN_PWM = 0x9,
    FN_ITC = 0xa,
    FN_PMX = 0xb,       /* PMA / PMM */
    FN_PSP = 0xc,
    FN_SM = 0xd,
    FN_OC = 0xe,
    FN_PPWA = 0xf,
};

enum { TCR1 = 0, TCR2 = 1 };

/* output action on a match (output PAC encoding, UNVERIFIED) */
enum { ACT_NONE = 0, ACT_HIGH = 1, ACT_LOW = 2, ACT_TOGGLE = 3 };

/* why a channel is serviced */
enum { WHY_HSR, WHY_LINK, WHY_EVENT };

/*
 * A time base derived from the system clock: absolute tick count
 * base + (cycles since base_ns) / div.  div = 0: stopped.
 */
typedef struct TpuCounter {
    uint32_t hz;
    uint32_t div;
    int64_t base_ns;
    uint64_t base;
} TpuCounter;

typedef struct TpuChan {
    struct MC68376TPU *m;
    int n;
    EcuPin *pin;

    /* channel hardware (11.2.2) */
    bool out;               /* pin direction is output */
    int level;              /* pin level (driven, or last sampled input) */
    uint8_t edge;           /* transition detection: 1 rising, 2 falling */
    int cap_tcr;            /* TCR captured on a transition */
    int match_tcr;          /* TCR the match register is compared with */
    bool match_en;
    bool match_noimm;       /* fire only when the TCR steps onto mr */
    uint16_t mr;            /* match register */
    int64_t match_ns;       /* time of the match (TCR1 / gated TCR2) */
    int out_action;         /* pin action on match (ACT_*) */
    uint16_t cap;           /* capture register */
    int64_t ev_ns;          /* time of the last transition / match */
    int tdl_level;          /* pin level after the latched transition */
    bool mrl, tdl, lsl;     /* match, transition, link service latches */
    bool hsr_blocked;       /* HSR the model cannot service */
    int64_t req_ns;         /* time of the oldest pending request */

    /* angle watch on TCR2 (PSP), behaves like a second match register */
    bool aw_en;
    uint8_t aw_kind;
    uint16_t aw_val;
    bool awl;

    /* function state */
    int st;
    uint64_t last_ticks;    /* absolute TCR1 ticks of the last edge */
    uint32_t ref_period;    /* PMx: last normal period */
    bool have_last, have_ref, gap_seen, synced, err, skip;
    int tooth_cnt;
    bool fall_pending;
    uint16_t fall_val;
} TpuChan;

struct MC68376TPU {
    MC68376State *soc;
    QEMUTimer *timer;
    QEMUBH *bh;
    Notifier clock_notifier;
    IMBIrq irq;
    TpuChan ch[TPU_NCH];
    EcuPin *t2clk_pin;

    /* host registers */
    uint16_t mcr, dscr, ticr, cier, cisr;
    uint16_t cfsr[4], hsqr[2], hsrr[2], cpr[2];
    uint16_t cisr_armed;    /* CISR bits read as one since the last write */
    bool mcr_written;
    uint16_t pram[128];

    /* time bases */
    TpuCounter tcr1;
    TpuCounter tcr2c;       /* gated mode (T2CG = 1) */
    uint64_t tcr2_edges;    /* external mode: prescaled T2CLK edges */
    uint16_t tcr2_off;      /* TCR2 = abs + off (microcode writes) */
    int t2clk;
    int t2_presc;

    int64_t lt;             /* logical time of the last processed event */
    bool in_advance;
    uint32_t unimp_logged;  /* per channel: unsupported function logged */
    bool emu_logged;
};

static void tpu_advance(MC68376TPU *m, int64_t now, bool inclusive);

/* ---------------------------------------------------------------------- */
/* Time bases                                                             */
/* ---------------------------------------------------------------------- */

static uint64_t cnt_ticks(TpuCounter *c, int64_t t)
{
    if (!c->div || !c->hz || t <= c->base_ns) {
        return c->base;
    }
    return c->base + muldiv64(t - c->base_ns, c->hz,
                              NANOSECONDS_PER_SECOND) / c->div;
}

static void cnt_rebase(TpuCounter *c, int64_t t, uint32_t hz, uint32_t div)
{
    c->base = cnt_ticks(c, t);
    c->base_ns = t;
    c->hz = hz;
    c->div = div;
}

/* earliest virtual time at which the absolute count reaches @k */
static int64_t cnt_time_of(TpuCounter *c, uint64_t k)
{
    uint64_t cyc;

    if (!c->div || !c->hz) {
        return INT64_MAX;
    }
    if (k <= c->base) {
        return c->base_ns;
    }
    cyc = (k - c->base) * c->div;
    return c->base_ns + muldiv64_round_up(cyc, NANOSECONDS_PER_SECOND,
                                          c->hz);
}

static bool tpu_stopped(MC68376TPU *m)
{
    return m->mcr & MCR_STOP;
}

static uint32_t tcr1_div(MC68376TPU *m)
{
    if (tpu_stopped(m)) {
        return 0;
    }
    return ((m->mcr & MCR_PSCK) ? 4 : 32) << MCR_TCR1P(m->mcr);
}

static uint32_t tcr2_gated_div(MC68376TPU *m)
{
    if (tpu_stopped(m) || !(m->mcr & MCR_T2CG) || !m->t2clk) {
        return 0;
    }
    return 8 << MCR_TCR2P(m->mcr);
}

static uint64_t tcr2_abs(MC68376TPU *m, int64_t t)
{
    return (m->mcr & MCR_T2CG) ? cnt_ticks(&m->tcr2c, t) : m->tcr2_edges;
}

static uint16_t tcr_value(MC68376TPU *m, int which, int64_t t)
{
    if (which == TCR1) {
        return cnt_ticks(&m->tcr1, t);
    }
    return tcr2_abs(m, t) + m->tcr2_off;
}

/* greater-or-equal comparison of a free running 16-bit count */
static bool tcr_ge(uint16_t tcr, uint16_t v)
{
    return (uint16_t)(tcr - v) < 0x8000;
}

/* ---------------------------------------------------------------------- */
/* Parameter RAM                                                          */
/* ---------------------------------------------------------------------- */

/* words 6 and 7 of channels 0-13 are not implemented (Table D-57) */
static bool pram_impl(unsigned addr)
{
    return (addr & 0x0f) < 0x0c || (addr >> 4) >= 14;
}

static uint16_t pram_rd(MC68376TPU *m, unsigned addr)
{
    addr &= 0xfe;
    return pram_impl(addr) ? m->pram[addr >> 1] : 0;
}

static void pram_wr(MC68376TPU *m, unsigned addr, uint16_t v)
{
    addr &= 0xfe;
    if (pram_impl(addr)) {
        m->pram[addr >> 1] = v;
    }
}

static uint8_t pram_rdb(MC68376TPU *m, unsigned addr)
{
    uint16_t w = pram_rd(m, addr);

    return (addr & 1) ? w : w >> 8;
}

static void pram_wrb(MC68376TPU *m, unsigned addr, uint8_t v)
{
    uint16_t w = pram_rd(m, addr);

    w = (addr & 1) ? (w & 0xff00) | v : (w & 0x00ff) | (v << 8);
    pram_wr(m, addr, w);
}

/* channel parameter at byte offset @off (0, 2, ... 0xE) */
static uint16_t P(TpuChan *c, unsigned off)
{
    return pram_rd(c->m, c->n * 16 + off);
}

static void SETP(TpuChan *c, unsigned off, uint16_t v)
{
    pram_wr(c->m, c->n * 16 + off, v);
}

static uint8_t PHI(TpuChan *c, unsigned off)
{
    return P(c, off) >> 8;
}

static uint8_t PLO(TpuChan *c, unsigned off)
{
    return P(c, off);
}

static void SETPHI(TpuChan *c, unsigned off, uint8_t v)
{
    SETP(c, off, (P(c, off) & 0x00ff) | (v << 8));
}

static void SETPLO(TpuChan *c, unsigned off, uint8_t v)
{
    SETP(c, off, (P(c, off) & 0xff00) | v);
}

/* ---------------------------------------------------------------------- */
/* Host register fields                                                   */
/* ---------------------------------------------------------------------- */

static int cfsr_get(MC68376TPU *m, int n)
{
    return (m->cfsr[3 - n / 4] >> ((n % 4) * 4)) & 0xf;
}

static int field2(const uint16_t *r, int n)
{
    return (r[1 - n / 8] >> ((n % 8) * 2)) & 3;
}

static void field2_set(uint16_t *r, int n, int v)
{
    int sh = (n % 8) * 2;

    r[1 - n / 8] = (r[1 - n / 8] & ~(3 << sh)) | (v << sh);
}

static int hsq(TpuChan *c)
{
    return field2(c->m->hsqr, c->n);
}

static int hsr_get(TpuChan *c)
{
    return field2(c->m->hsrr, c->n);
}

static int prio(TpuChan *c)
{
    return field2(c->m->cpr, c->n);
}

/* ---------------------------------------------------------------------- */
/* Interrupts (11.3.7)                                                    */
/* ---------------------------------------------------------------------- */

/*
 * UNVERIFIED: which channel's vector the TPU supplies when several
 * channel interrupts are pending is not documented in the manual; the
 * lowest numbered channel is used.
 */
static uint8_t tpu_vector(MC68376TPU *m)
{
    uint16_t act = m->cisr & m->cier;

    return (m->ticr & 0xf0) | (act ? ctz32(act) : 0);
}

static void tpu_update_irq(MC68376TPU *m)
{
    int level = TICR_CIRL(m->ticr);

    m->irq.level = level;
    m->irq.iarb = m->mcr & 0xf;
    m->irq.vector = tpu_vector(m);
    imb_irq_set(&m->irq, level && (m->cisr & m->cier));
    imb_irq_update(m->soc);
}

static void tpu_iack(IMBIrq *irq)
{
    MC68376TPU *m = irq->opaque;

    irq->vector = tpu_vector(m);
}

static void ch_irq(TpuChan *c)
{
    c->m->cisr |= 1 << c->n;
}

/* ---------------------------------------------------------------------- */
/* Channel hardware                                                       */
/* ---------------------------------------------------------------------- */

static bool ch_pending(TpuChan *c)
{
    return (hsr_get(c) && !c->hsr_blocked) || c->lsl || c->mrl || c->tdl ||
           c->awl;
}

static void ch_latch(TpuChan *c, bool *latch, int64_t t)
{
    if (!ch_pending(c)) {
        c->req_ns = t;
    }
    *latch = true;
}

static void ch_link(MC68376TPU *m, int n, int64_t t)
{
    TpuChan *d = &m->ch[n & 15];

    ch_latch(d, &d->lsl, t);
}

/* link to a block of @count channels starting at @start */
static void link_block(MC68376TPU *m, int start, int count, int64_t t)
{
    for (int i = 0; i < MIN(count, 8); i++) {
        ch_link(m, start + i, t);
    }
}

static void ch_drive(TpuChan *c, int level, int64_t t)
{
    c->out = true;
    level = !!level;
    if (c->level != level || ecu_pin_level(c->pin) != level) {
        c->level = level;
        ecu_pin_mcu_drive_at(c->pin, level, t);
    }
}

static void ch_action(TpuChan *c, int act, int64_t t)
{
    switch (act) {
    case ACT_HIGH:
        ch_drive(c, 1, t);
        break;
    case ACT_LOW:
        ch_drive(c, 0, t);
        break;
    case ACT_TOGGLE:
        ch_drive(c, !c->level, t);
        break;
    }
}

static void ch_input(TpuChan *c, uint8_t edge)
{
    c->out = false;
    c->edge = edge;
    c->level = ecu_pin_level(c->pin);
}

static void ch_calc_match(MC68376TPU *m, TpuChan *c, int64_t t)
{
    TpuCounter *k = NULL;
    uint16_t cur;
    uint32_t d;

    c->match_ns = INT64_MAX;
    if (!c->match_en) {
        return;
    }
    cur = tcr_value(m, c->match_tcr, t);
    if (!c->match_noimm && tcr_ge(cur, c->mr)) {
        c->match_ns = t;
        return;
    }
    if (c->match_tcr == TCR1) {
        k = &m->tcr1;
    } else if (m->mcr & MCR_T2CG) {
        k = &m->tcr2c;
    } else {
        return;     /* external TCR2 clock: checked on every T2CLK edge */
    }
    d = (uint16_t)(c->mr - cur);
    if (!d) {
        d = 0x10000;
    }
    c->match_ns = cnt_time_of(k, cnt_ticks(k, t) + d);
}

static void ch_arm(MC68376TPU *m, TpuChan *c, int tcr, uint16_t val,
                   int action, int64_t t)
{
    c->match_en = true;
    c->match_noimm = false;
    c->match_tcr = tcr;
    c->mr = val;
    c->out_action = action;
    ch_calc_match(m, c, t);
}

static void ch_disarm(TpuChan *c)
{
    c->match_en = false;
    c->match_ns = INT64_MAX;
}

/* stop all channel activity and drop the pending events (re-init) */
static void ch_quiesce(TpuChan *c)
{
    ch_disarm(c);
    c->edge = 0;
    c->aw_en = false;
    c->mrl = c->tdl = c->awl = false;
}

static void do_match(MC68376TPU *m, TpuChan *c, int64_t t)
{
    ch_disarm(c);
    if (c->out) {
        ch_action(c, c->out_action, t);
    }
    c->ev_ns = t;
    ch_latch(c, &c->mrl, t);
}

static void ch_watch_angle(TpuChan *c, uint8_t kind, uint16_t val)
{
    c->aw_en = true;
    c->aw_kind = kind;
    c->aw_val = val;
}

/* TCR2 changed from @old at @t: edge-driven matches and angle watches */
static void tcr2_changed(MC68376TPU *m, uint16_t old, int64_t t)
{
    uint16_t now = tcr_value(m, TCR2, t);

    for (int n = 0; n < TPU_NCH; n++) {
        TpuChan *c = &m->ch[n];
        if (c->match_en && c->match_tcr == TCR2) {
            if (m->mcr & MCR_T2CG) {
                ch_calc_match(m, c, t);
            } else if (!tcr_ge(old, c->mr) && tcr_ge(now, c->mr)) {
                do_match(m, c, t);
            }
        }
        if (c->aw_en && !tcr_ge(old, c->aw_val) && tcr_ge(now, c->aw_val)) {
            c->aw_en = false;
            c->ev_ns = t;
            ch_latch(c, &c->awl, t);
        }
    }
}

/* microcode write of TCR2 (PMA / PMM) */
static void tcr2_write(MC68376TPU *m, uint16_t v, int64_t t)
{
    uint16_t old = tcr_value(m, TCR2, t);

    m->tcr2_off = v - (uint16_t)tcr2_abs(m, t);
    tcr2_changed(m, old, t);
}

static void tpu_recalc_matches(MC68376TPU *m, int64_t t)
{
    for (int n = 0; n < TPU_NCH; n++) {
        if (m->ch[n].match_en) {
            ch_calc_match(m, &m->ch[n], t);
        }
    }
}

/* re-derive the time bases after a TPUMCR, SYNCR or T2CLK gate change */
static void tpu_clocks(MC68376TPU *m, int64_t t)
{
    uint32_t hz = mc68376_sysclk_hz(m->soc);

    cnt_rebase(&m->tcr1, t, hz, tcr1_div(m));
    cnt_rebase(&m->tcr2c, t, hz, tcr2_gated_div(m));
    tpu_recalc_matches(m, t);
}

/*
 * CHANNEL_CONTROL (common parameter, UNVERIFIED encodings beyond those
 * listed in the docs): TBS[8:5] (bit 3 = no change, bit 2 = output,
 * bit 1 = capture TCR2, bit 0 = match TCR2), PAC[4:2] (bit 2 = no
 * change; input: edges 1 rising, 2 falling, 3 both; output: match action
 * 1 high, 2 low, 3 toggle), PSC[1:0] (1 force high, 2 force low, 3 do
 * not force, 0 force per PAC).  The pin direction is set by the function
 * model (input or output function), not by TBS bit 2.
 */
static void cc_apply(TpuChan *c, uint16_t cc, bool output, int64_t t)
{
    int tbs = (cc >> 5) & 0xf, pac = (cc >> 2) & 7, psc = cc & 3;

    if (!(tbs & 8)) {
        c->cap_tcr = (tbs & 2) ? TCR2 : TCR1;
        c->match_tcr = (tbs & 1) ? TCR2 : TCR1;
    }
    if (!output) {
        ch_input(c, (pac & 4) ? c->edge : (pac & 3));
        return;
    }
    c->edge = 0;
    if (!(pac & 4)) {
        c->out_action = pac & 3;
    }
    c->out = true;
    switch (psc) {
    case 0:
        ch_action(c, c->out_action, t);
        break;
    case 1:
        ch_drive(c, 1, t);
        break;
    case 2:
        ch_drive(c, 0, t);
        break;
    }
}

static void log_unimp(TpuChan *c, const char *what)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.tpu: channel %d: %s\n", c->n, what);
}

/* ---------------------------------------------------------------------- */
/* DIO: discrete input/output ($8, TPUPN18)                               */
/* ---------------------------------------------------------------------- */
/*
 * W0 CHANNEL_CONTROL, W2 PIN_LEVEL (bit 15 newest, UM 11.4.1),
 * W4 MATCH_RATE.  HSQ (input): %00 transition, %01 match rate, %10 on
 * request.  HSR %11 input / read, %01 drive high, %10 drive low.
 * UNVERIFIED: the interrupt request on every service; PIN_LEVEL updated
 * at the input initialisation and on output requests.
 */
static void dio_sample(TpuChan *c, int level)
{
    SETP(c, 2, (P(c, 2) >> 1) | (level ? 0x8000 : 0));
}

static bool fn_dio(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    switch (why) {
    case WHY_HSR:
        ch_quiesce(c);
        if (hsr == 3) {
            cc_apply(c, P(c, 0), false, t);
            c->st = hsq(c);
            if (c->st != 0) {
                c->edge = 0;
            }
            if (c->st == 1) {
                /* the first interval always uses TCR1 */
                ch_arm(m, c, TCR1, tcr_value(m, TCR1, t) + P(c, 4),
                       ACT_NONE, t);
            } else if (c->st == 3) {
                qemu_log_mask(LOG_GUEST_ERROR, "mc68376.tpu: DIO channel %d: "
                              "HSR %%11 with HSQ %%11\n", c->n);
            }
            dio_sample(c, c->level);
        } else {
            c->st = 2;
            ch_drive(c, hsr == 1, t);
            dio_sample(c, c->level);
        }
        ch_irq(c);
        return true;
    case WHY_EVENT:
        if (c->out) {
            return true;
        }
        if (c->tdl && c->st == 0) {
            dio_sample(c, c->tdl_level);
            ch_irq(c);
        }
        if (c->mrl && c->st == 1) {
            dio_sample(c, c->level);
            ch_arm(m, c, c->match_tcr, c->mr + P(c, 4), ACT_NONE, t);
            ch_irq(c);
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* ITC: input capture / input transition counter ($A, TPUPN16)            */
/* ---------------------------------------------------------------------- */
/*
 * W0 CHANNEL_CONTROL, W2 START_LINK_CHANNEL[15:12] LINK_CHANNEL_COUNT
 * [11:8] BANK_ADDRESS[7:0] (bit split UNVERIFIED), W4 MAX_COUNT,
 * W6 TRANS_COUNT, W8 FINAL_TRANS_TIME, WA LAST_TRANS_TIME.
 * HSQ bit 0: continual, bit 1: links.  HSR %01 initialises.
 * UNVERIFIED: the bank byte is incremented only when BANK_ADDRESS is
 * non-zero; HSR %10/%11 do nothing.
 */
static bool fn_itc(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    uint16_t max;

    switch (why) {
    case WHY_HSR:
        if (hsr != 1) {
            log_unimp(c, "ITC host service request other than %01");
            return true;
        }
        ch_quiesce(c);
        cc_apply(c, P(c, 0), false, t);
        SETP(c, 6, 0);
        c->st = 1;
        return true;
    case WHY_EVENT:
        if (!c->tdl || c->st != 1) {
            return true;
        }
        SETP(c, 0xa, c->cap);
        SETP(c, 6, P(c, 6) + 1);
        max = MAX(P(c, 4), 1);
        if (P(c, 6) < max) {
            return true;
        }
        SETP(c, 8, c->cap);
        ch_irq(c);
        if (PLO(c, 2)) {
            pram_wrb(m, PLO(c, 2), pram_rdb(m, PLO(c, 2)) + 1);
        }
        if (hsq(c) & 2) {
            link_block(m, PHI(c, 2) >> 4, PHI(c, 2) & 0xf, t);
        }
        if (hsq(c) & 1) {
            SETP(c, 6, 0);
        } else {
            c->st = 0;
            c->edge = 0;
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* OC: output compare ($E, TPUPN12)                                       */
/* ---------------------------------------------------------------------- */
/*
 * UNVERIFIED layout (the note's parameter table was not obtained):
 * W0 CHANNEL_CONTROL, W2 OFFSET, W4 RATIO[15:8] REF_ADDR1[7:0],
 * W6 REF_ADDR2[15:8] REF_ADDR3[7:0], W8 REF_TIME, WA ACTUAL_MATCH_TIME.
 * HSR %01 host-initiated pulse (copies TCR1/TCR2 to $EC/$EE, partially
 * confirmed), %11 continuous mode, %10 not modelled.  HSQ bit 1 set:
 * no immediate PSC force (UNVERIFIED).  Continuous mode: OFFSET =
 * RATIO x (REF_ADDR2) / 256 (scaling UNVERIFIED), no interrupts.
 */
enum { OC_IDLE, OC_PULSE, OC_WAIT_LINK, OC_CONT };

static void oc_offset(MC68376TPU *m, TpuChan *c)
{
    uint32_t period = pram_rd(m, PHI(c, 6));

    SETP(c, 2, period * PHI(c, 4) / 256);
}

static bool fn_oc(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    uint16_t cc = P(c, 0);

    switch (why) {
    case WHY_HSR:
        if (hsr == 2) {
            log_unimp(c, "OC host service request %10");
            return true;
        }
        ch_quiesce(c);
        if (hsr == 1) {
            pram_wr(m, 0xec, tcr_value(m, TCR1, t));
            pram_wr(m, 0xee, tcr_value(m, TCR2, t));
            cc_apply(c, (hsq(c) & 2) ? (cc | 3) : cc, true, t);
            ch_arm(m, c, c->match_tcr,
                   pram_rd(m, PLO(c, 4)) + P(c, 2), c->out_action, t);
            c->st = OC_PULSE;
        } else {
            cc_apply(c, cc, true, t);
            c->st = OC_WAIT_LINK;
        }
        return true;
    case WHY_LINK:
        if (c->st == OC_WAIT_LINK) {
            oc_offset(m, c);
            SETP(c, 8, pram_rd(m, PLO(c, 6)));
            ch_arm(m, c, c->match_tcr, P(c, 8), ACT_TOGGLE, t);
            c->st = OC_CONT;
        } else if (c->st == OC_CONT) {
            oc_offset(m, c);
        }
        return true;
    case WHY_EVENT:
        if (!c->mrl) {
            return true;
        }
        SETP(c, 0xa, c->mr);
        if (c->st == OC_PULSE) {
            ch_irq(c);
            c->st = OC_IDLE;
        } else if (c->st == OC_CONT) {
            SETP(c, 8, c->mr + P(c, 2));
            ch_arm(m, c, c->match_tcr, P(c, 8), ACT_TOGGLE, t);
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* PWM: pulse-width modulation ($9, TPUPN17)                              */
/* ---------------------------------------------------------------------- */
/*
 * W0 CHANNEL_CONTROL, W2 OLDRIS, W4 PWMHI, W6 PWMPER, W8 PWMRIS.
 * HSR %10 initialise, %01 immediate update.  Interrupt request at the
 * start of every period and after an immediate update.  PWMHI = 0 or
 * PWMHI >= PWMPER: level mode (no transition, match at OLDRIS + PWMPER).
 */
enum { PWM_IDLE, PWM_HIGH, PWM_LOW, PWM_LEVEL };

/* start of a period at TCR value @rise (time @t), shared with SPWM */
static void pwm_period(MC68376TPU *m, TpuChan *c, unsigned old_off,
                       unsigned next_off, uint16_t rise, int64_t t)
{
    uint16_t hi = P(c, 4), per = P(c, 6);

    SETP(c, old_off, rise);
    SETP(c, next_off, rise + per);
    if (!per) {
        /* no period: stay at the level, nothing to schedule */
        ch_drive(c, hi != 0, t);
        c->st = PWM_IDLE;
    } else if (hi == 0 || hi >= per) {
        ch_drive(c, hi != 0, t);
        ch_arm(m, c, c->match_tcr, rise + per, ACT_NONE, t);
        c->st = PWM_LEVEL;
    } else {
        ch_drive(c, 1, t);
        ch_arm(m, c, c->match_tcr, rise + hi, ACT_LOW, t);
        c->st = PWM_HIGH;
    }
    ch_irq(c);
}

/* re-plan the current period with new PWMHI / PWMPER */
static void pwm_update(MC68376TPU *m, TpuChan *c, unsigned old_off,
                       unsigned next_off, int64_t t)
{
    uint16_t rise = P(c, old_off), hi = P(c, 4), per = P(c, 6);

    SETP(c, next_off, rise + per);
    if (c->st == PWM_IDLE) {
        return;
    }
    if (!per) {
        ch_disarm(c);
        ch_drive(c, hi != 0, t);
        c->st = PWM_IDLE;
    } else if (hi == 0 || hi >= per) {
        ch_drive(c, hi != 0, t);
        ch_arm(m, c, c->match_tcr, rise + per, ACT_NONE, t);
        c->st = PWM_LEVEL;
    } else if (c->level) {
        ch_arm(m, c, c->match_tcr, rise + hi, ACT_LOW, t);
        c->st = PWM_HIGH;
    } else {
        ch_arm(m, c, c->match_tcr, rise + per, ACT_HIGH, t);
        c->st = PWM_LOW;
    }
}

static bool fn_pwm(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    switch (why) {
    case WHY_HSR:
        if (hsr == 2) {
            ch_quiesce(c);
            cc_apply(c, P(c, 0), true, t);
            pwm_period(m, c, 2, 8, tcr_value(m, c->match_tcr, t), t);
        } else if (hsr == 1) {
            pwm_update(m, c, 2, 8, t);
            ch_irq(c);
        } else {
            log_unimp(c, "PWM host service request %11");
        }
        return true;
    case WHY_EVENT:
        if (!c->mrl) {
            return true;
        }
        if (c->st == PWM_HIGH) {
            ch_arm(m, c, c->match_tcr, P(c, 8), ACT_HIGH, t);
            c->st = PWM_LOW;
        } else if (c->st == PWM_LOW || c->st == PWM_LEVEL) {
            pwm_period(m, c, 2, 8, c->mr, t);
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* SPWM: synchronized PWM ($7, TPUPN19)                                   */
/* ---------------------------------------------------------------------- */
/*
 * W0 CHANNEL_CONTROL, overwritten with LASTRISE; W2 NEXTRISE,
 * W4 HIGH_TIME, W6 PERIOD, W8 REF_ADDR1[7:0] (mode 2: START_LINK_CHANNEL
 * [15:12], LINK_CHANNEL_COUNT [11:8], UNVERIFIED packing), WA DELAY.
 * HSQ %00 mode 0, %10 mode 2; mode 1 is not modelled.  HSR %10
 * initialises.  On a link (modes 0, 2): NEXTRISE = (REF_ADDR1) + DELAY +
 * PERIOD (as summarised from the note, UNVERIFIED).  Mode 2 links to
 * the channel block at each rising edge.  Interrupt at each rising edge.
 */
static void spwm_rise(MC68376TPU *m, TpuChan *c, uint16_t rise, int64_t t)
{
    pwm_period(m, c, 0, 2, rise, t);
    if (hsq(c) == 2) {
        link_block(m, PHI(c, 8) >> 4, PHI(c, 8) & 0xf, t);
    }
}

static bool fn_spwm(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    switch (why) {
    case WHY_HSR:
        if (hsr != 2) {
            log_unimp(c, "SPWM host service request other than %10");
            return true;
        }
        ch_quiesce(c);
        c->st = PWM_IDLE;
        if (hsq(c) & 1) {
            log_unimp(c, "SPWM mode 1");
            return true;
        }
        cc_apply(c, P(c, 0), true, t);
        spwm_rise(m, c, tcr_value(m, c->match_tcr, t), t);
        return true;
    case WHY_LINK:
        if (c->st == PWM_IDLE) {
            return true;
        }
        SETP(c, 2, pram_rd(m, PLO(c, 8)) + P(c, 0xa) + P(c, 6));
        if (c->st == PWM_LOW || c->st == PWM_LEVEL) {
            ch_arm(m, c, c->match_tcr, P(c, 2),
                   c->st == PWM_LOW ? ACT_HIGH : ACT_NONE, t);
        }
        return true;
    case WHY_EVENT:
        if (!c->mrl) {
            return true;
        }
        if (c->st == PWM_HIGH) {
            ch_arm(m, c, c->match_tcr, P(c, 2), ACT_HIGH, t);
            c->st = PWM_LOW;
        } else if (c->st == PWM_LOW || c->st == PWM_LEVEL) {
            spwm_rise(m, c, c->mr, t);
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* PMA / PMM: period measurement, additional / missing transition         */
/* detect ($B, TPUPN15A / TPUPN15B)                                       */
/* ---------------------------------------------------------------------- */
/*
 * HSQ bit 1: 0 PMA, 1 PMM; bit 0: 0 bank mode, 1 count mode.
 * W0 CHANNEL_CONTROL (normally $0007: rising edges, TCR1)
 * W2 MAX_MISSING / MAX_ADDITIONAL [15:8], NUM_OF_TEETH [7:0]
 * W4 BANK_SIGNAL [15:8], MISSING / ADDITIONAL_COUNT [7:0] (UNVERIFIED)
 * W6 RATIO [15:8], TCR2_MAX_VALUE [7:0] (UNVERIFIED byte order)
 * W8 TCR2_VALUE [15:8] (UNVERIFIED), PERIOD_HIGH_WORD [7:0] (bits 22:16)
 * WA PERIOD_LOW_WORD
 *
 * The tooth signal also drives T2CLK, so TCR2 counts teeth in hardware;
 * the function resets TCR2 to $FFFF on the reference gap so that the
 * next tooth makes it 0.  Model (UNVERIFIED details):
 *  - init (HSR %01, and %10 which a conflicting summary gives): TCR2 =
 *    $C0FF, the first period is not measured
 *  - period = TCR1 time between active edges (23 bits); longer than
 *    $7EFFFF: period error, TCR2 = $C0FF, TCR2_VALUE = $C0
 *  - missing tooth (PMM): period >= last normal period x RATIO / 128;
 *    additional tooth (PMA): period < last normal period x RATIO / 128,
 *    the following (short) period is not checked
 *  - a detected gap is accepted when NUM_OF_TEETH + 1 teeth were seen
 *    since the previous one (always for the first gap after init or an
 *    error); otherwise TCR2 = $80FF, TCR2_VALUE = $80 and an interrupt
 *    on every tooth until the next accepted gap
 *  - count mode: COUNT++, when it reaches MAX: TCR2 = $FFFF, COUNT = 0,
 *    interrupt; bank mode: BANK_SIGNAL non-zero: TCR2 = $FFFF,
 *    BANK_SIGNAL = 0, interrupt; zero: interrupt only
 *  - while synchronised, TCR2 above TCR2_MAX_VALUE (when non-zero) is an
 *    error as well
 *  - TCR2_VALUE follows the low byte of TCR2
 */
static void pmx_error(MC68376TPU *m, TpuChan *c, uint8_t code, int64_t t)
{
    tcr2_write(m, (code << 8) | 0xff, t);
    SETPHI(c, 8, code);
    c->gap_seen = c->synced = false;
    c->err = code == 0x80;
    ch_irq(c);
}

static void pmx_init(MC68376TPU *m, TpuChan *c, int64_t t)
{
    ch_quiesce(c);
    cc_apply(c, P(c, 0), false, t);
    c->have_last = c->have_ref = c->gap_seen = c->synced = false;
    c->err = c->skip = false;
    c->tooth_cnt = 0;
    tcr2_write(m, 0xc0ff, t);
    SETPHI(c, 8, 0xc0);
    c->st = 1;
}

static void pmx_edge(MC68376TPU *m, TpuChan *c, int64_t t)
{
    bool pmm = hsq(c) & 2, count_mode = hsq(c) & 1;
    uint64_t now = cnt_ticks(&m->tcr1, c->ev_ns);
    uint64_t period;
    bool det = false;

    if (!c->have_last) {
        c->have_last = true;
        c->last_ticks = now;
        return;
    }
    period = now - c->last_ticks;
    c->last_ticks = now;
    if (period > 0x7effff) {
        SETPLO(c, 8, 0x7f);
        SETP(c, 0xa, 0xffff);
        c->have_ref = false;
        c->tooth_cnt = 0;
        pmx_error(m, c, 0xc0, t);
        c->err = false;
        return;
    }
    SETPLO(c, 8, period >> 16);
    SETP(c, 0xa, period);

    if (c->have_ref && !c->skip) {
        uint64_t lim = (uint64_t)c->ref_period * PHI(c, 6);
        det = pmm ? period * 128 >= lim : period * 128 < lim;
    }
    c->skip = false;

    if (!det) {
        uint16_t tcr2 = tcr_value(m, TCR2, t);
        uint8_t max = PLO(c, 6);

        c->ref_period = period;
        c->have_ref = true;
        if (c->gap_seen) {
            c->tooth_cnt++;
        }
        if (c->err) {
            SETPHI(c, 8, PHI(c, 8) + 1);
            ch_irq(c);
        } else if (c->synced && max && tcr2 > max && tcr2 != 0xffff) {
            pmx_error(m, c, 0x80, t);
        } else if (c->synced) {
            SETPHI(c, 8, tcr2);
        }
        return;
    }

    /* missing / additional transition detected */
    if (!pmm) {
        c->skip = true;
    }
    if (c->gap_seen && !c->err && c->tooth_cnt != PLO(c, 2) + 1) {
        c->tooth_cnt = 1;
        pmx_error(m, c, 0x80, t);
        c->gap_seen = true;
        return;
    }
    c->gap_seen = true;
    c->err = false;
    c->tooth_cnt = 1;
    if (count_mode) {
        uint8_t cnt = PLO(c, 4) + 1;
        if (cnt >= PHI(c, 2)) {
            tcr2_write(m, 0xffff, t);
            c->synced = true;
            cnt = 0;
            ch_irq(c);
        }
        SETPLO(c, 4, cnt);
    } else {
        if (PHI(c, 4)) {
            tcr2_write(m, 0xffff, t);
            c->synced = true;
            SETPHI(c, 4, 0);
        }
        ch_irq(c);
    }
    if (c->synced) {
        SETPHI(c, 8, tcr_value(m, TCR2, t));
    }
}

static bool fn_pmx(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    switch (why) {
    case WHY_HSR:
        if (hsr == 3) {
            log_unimp(c, "PMA/PMM host service request %11");
            return true;
        }
        pmx_init(m, c, t);
        return true;
    case WHY_EVENT:
        if (c->tdl && c->st == 1) {
            pmx_edge(m, c, t);
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* PSP: position-synchronized pulse generator ($C, TPUPN14)               */
/* ---------------------------------------------------------------------- */
/*
 * W0 PERIOD_ADDRESS + CHANNEL_CONTROL: the period word address is taken
 *    from bits 15:9 (an even byte address, UNVERIFIED), CHANNEL_CONTROL
 *    from bits 8:0 (only used in force mode)
 * W2 R2_A2_TEMP, W4 ANGLE_TIME (TCR1 at the angle), W6 RATIO_TEMP
 * W8 RATIO1 [15:8] / ANGLE1 [7:0]
 * WA RATIO2 [15:8] / ANGLE2 [7:0] (HSQ bit 0 = 0, angle-angle) or
 *    HIGH_TIME (HSQ bit 0 = 1, angle-time)
 * HSR %10 init, %01 immediate update, %11 force (pin per PSC, stop).
 *
 * When TCR2 (the tooth count reset by PMA/PMM) steps onto ANGLE1 the
 * rising edge is scheduled at TCR1 + RATIO1 x period / 128 (period = the
 * word at PERIOD_ADDRESS, normally PMx PERIOD_LOW_WORD); the falling
 * edge HIGH_TIME later or at ANGLE2 + RATIO2 x period / 128.
 * UNVERIFIED: active-high pulses (pin low after init), an angle matches
 * only when TCR2 steps onto it (not when it is already past), immediate
 * update re-plans the pending edge(s) ignoring HSQ bit 1, an interrupt
 * request at the end of each pulse.
 */
enum { PSP_IDLE, PSP_WAIT_A1, PSP_WAIT_RISE, PSP_HIGH };

static uint16_t psp_offset(MC68376TPU *m, TpuChan *c, uint8_t ratio)
{
    uint32_t period = pram_rd(m, (P(c, 0) >> 8) & 0xfe);

    return period * ratio / 128;
}

static void psp_arm_a1(TpuChan *c)
{
    c->st = PSP_WAIT_A1;
    c->fall_pending = false;
    ch_watch_angle(c, 1, PLO(c, 8));
}

static bool fn_psp(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    bool angle_time = hsq(c) & 1;

    switch (why) {
    case WHY_HSR:
        if (hsr == 2) {
            ch_quiesce(c);
            c->match_tcr = TCR1;
            ch_drive(c, 0, t);
            psp_arm_a1(c);
        } else if (hsr == 3) {
            ch_quiesce(c);
            c->st = PSP_IDLE;
            switch (P(c, 0) & 3) {
            case 1:
                ch_drive(c, 1, t);
                break;
            case 2:
                ch_drive(c, 0, t);
                break;
            }
        } else if (c->st == PSP_WAIT_RISE) {
            ch_arm(m, c, TCR1, P(c, 4) + psp_offset(m, c, PHI(c, 8)),
                   ACT_HIGH, t);
        } else if (c->st == PSP_HIGH && angle_time && c->match_en) {
            ch_arm(m, c, TCR1, c->fall_val + P(c, 0xa), ACT_LOW, t);
        }
        return true;
    case WHY_EVENT:
        if (c->awl) {
            c->awl = false;
            if (c->aw_kind == 1 && c->st == PSP_WAIT_A1) {
                uint16_t at = tcr_value(m, TCR1, c->ev_ns);
                SETP(c, 4, at);
                ch_arm(m, c, TCR1, at + psp_offset(m, c, PHI(c, 8)),
                       ACT_HIGH, t);
                c->st = PSP_WAIT_RISE;
                if (!angle_time) {
                    if (PLO(c, 0xa) == PLO(c, 8)) {
                        c->fall_val = at + psp_offset(m, c, PHI(c, 0xa));
                        c->fall_pending = true;
                    } else {
                        ch_watch_angle(c, 2, PLO(c, 0xa));
                    }
                }
            } else if (c->aw_kind == 2 &&
                       (c->st == PSP_WAIT_RISE || c->st == PSP_HIGH)) {
                uint16_t at = tcr_value(m, TCR1, c->ev_ns);
                c->fall_val = at + psp_offset(m, c, PHI(c, 0xa));
                SETP(c, 2, c->fall_val);
                if (c->st == PSP_HIGH) {
                    ch_arm(m, c, TCR1, c->fall_val, ACT_LOW, t);
                } else {
                    c->fall_pending = true;
                }
            }
        }
        if (c->mrl) {
            c->mrl = false;
            if (c->st == PSP_WAIT_RISE) {
                c->st = PSP_HIGH;
                if (angle_time) {
                    c->fall_val = c->mr;
                    ch_arm(m, c, TCR1, c->mr + P(c, 0xa), ACT_LOW, t);
                } else if (c->fall_pending) {
                    c->fall_pending = false;
                    ch_arm(m, c, TCR1, c->fall_val, ACT_LOW, t);
                }
            } else if (c->st == PSP_HIGH) {
                ch_irq(c);
                psp_arm_a1(c);
            }
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* PPWA: period / pulse-width accumulator ($F, TPUPN11)                   */
/* ---------------------------------------------------------------------- */
/*
 * W0 START_LINK_CHANNEL [15:12], LINK_CHANNEL_COUNT [11:9] (0 = 8,
 *    UNVERIFIED split), CHANNEL_CONTROL [8:0]
 * W2 MAX_COUNT [15:8], PERIOD_COUNT [7:0] (UNVERIFIED byte split)
 * W4 LAST_ACCUM, W6 ACCUM, W8 ACCUM_RATE [15:8] / PPWA_UB [7:0], WA PPWA_LW
 * HSQ: %00 24-bit periods, %01 16-bit periods + links, %10 24-bit pulse
 * widths, %11 16-bit pulse widths + links.  HSR %10 initialises
 * (UNVERIFIED).  After MAX_COUNT periods/pulses: PPWA_LW = ACCUM,
 * ACCUM = 0, PERIOD_COUNT = 0, interrupt, links in modes 1 and 3.
 * 24-bit modes carry into PPWA_UB, which the CPU must clear.
 * Not modelled: the periodic ACCUM updates at ACCUM_RATE (the sum is
 * added once per period, which gives the same final result); 16-bit
 * modes wrap (UNVERIFIED).
 */
static void ppwa_add(MC68376TPU *m, TpuChan *c, uint32_t v, int64_t t)
{
    uint32_t sum = P(c, 6) + v;
    uint8_t cnt;

    if (!(hsq(c) & 1)) {
        SETPLO(c, 8, PLO(c, 8) + (sum >> 16));
    }
    SETP(c, 6, sum);
    cnt = PLO(c, 2) + 1;
    if (cnt >= MAX(PHI(c, 2), 1)) {
        SETP(c, 0xa, P(c, 6));
        SETP(c, 6, 0);
        cnt = 0;
        ch_irq(c);
        if (hsq(c) & 1) {
            int count = (P(c, 0) >> 9) & 7;
            link_block(m, P(c, 0) >> 12, count ? count : 8, t);
        }
    }
    SETPLO(c, 2, cnt);
}

static bool fn_ppwa(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    bool pulse = hsq(c) & 2;
    uint64_t now;
    uint32_t v;

    switch (why) {
    case WHY_HSR:
        if (hsr != 2) {
            log_unimp(c, "PPWA host service request other than %10");
            return true;
        }
        ch_quiesce(c);
        cc_apply(c, P(c, 0) & 0x1ff, false, t);
        if (pulse) {
            c->edge = 3;
        }
        SETPLO(c, 2, 0);
        SETP(c, 6, 0);
        c->st = 1;
        return true;
    case WHY_EVENT:
        if (!c->tdl || !c->st) {
            return true;
        }
        now = cnt_ticks(&m->tcr1, c->ev_ns);
        if (pulse && !c->tdl_level) {
            if (c->st != 2) {
                return true;
            }
        } else if (c->st == 1 || pulse) {
            SETP(c, 4, c->cap);
            c->last_ticks = now;
            c->st = 2;
            return true;
        }
        if (c->cap_tcr == TCR1) {
            v = now - c->last_ticks;
        } else {
            v = (uint16_t)(c->cap - P(c, 4));
        }
        if (pulse) {
            c->st = 3;
        } else {
            SETP(c, 4, c->cap);
            c->last_ticks = now;
        }
        ppwa_add(m, c, v, t);
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* QDEC: quadrature decode ($6, TPUPN20)                                  */
/* ---------------------------------------------------------------------- */
/*
 * Per channel: W0 EDGE_TIME, W2 POSITION_COUNT, W4 TCR1_VALUE,
 * W6 CHAN_PINSTATE, W8 CORR_PINSTATE_ADDR, WA EDGE_TIME_LSB_ADDR.
 * HSQ bit 0: 0 primary, 1 secondary channel.  HSR %11 initialises.
 * UNVERIFIED: the shared EDGE_TIME / POSITION_COUNT words are found at
 * EDGE_TIME_LSB_ADDR - 1 and + 1; CHAN_PINSTATE holds 0 or 1; a
 * primary edge with the pins different (secondary edge: equal) counts
 * up; HSR %01 copies TCR1 to TCR1_VALUE; the interrupt request is made
 * when the counter wraps.
 */
static bool fn_qdec(MC68376TPU *m, TpuChan *c, int why, int hsr, int64_t t)
{
    unsigned lsb = PLO(c, 0xa);
    uint16_t pos;
    bool up;

    switch (why) {
    case WHY_HSR:
        if (hsr == 3) {
            ch_quiesce(c);
            cc_apply(c, 0, false, t);
            c->edge = 3;
            SETP(c, 6, c->level);
            SETP(c, 4, tcr_value(m, TCR1, t));
            c->st = 1;
        } else if (hsr == 1) {
            SETP(c, 4, tcr_value(m, TCR1, t));
        } else {
            log_unimp(c, "QDEC host service request %10");
        }
        return true;
    case WHY_EVENT:
        if (!c->tdl || c->st != 1) {
            return true;
        }
        SETP(c, 6, c->tdl_level);
        if (hsq(c) & 1) {
            up = c->tdl_level == !!pram_rd(m, PLO(c, 8));
        } else {
            up = c->tdl_level != !!pram_rd(m, PLO(c, 8));
        }
        pos = pram_rd(m, lsb + 1) + (up ? 1 : -1);
        pram_wr(m, lsb + 1, pos);
        pram_wr(m, lsb - 1, tcr_value(m, TCR1, c->ev_ns));
        if ((up && pos == 0) || (!up && pos == 0xffff)) {
            ch_irq(c);
        }
        return true;
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* Scheduler                                                              */
/* ---------------------------------------------------------------------- */

typedef bool (*TpuFn)(MC68376TPU *m, TpuChan *c, int why, int hsr,
                      int64_t t);

static TpuFn tpu_function(int code)
{
    switch (code) {
    case FN_QDEC:
        return fn_qdec;
    case FN_SPWM:
        return fn_spwm;
    case FN_DIO:
        return fn_dio;
    case FN_PWM:
        return fn_pwm;
    case FN_ITC:
        return fn_itc;
    case FN_PMX:
        return fn_pmx;
    case FN_PSP:
        return fn_psp;
    case FN_OC:
        return fn_oc;
    case FN_PPWA:
        return fn_ppwa;
    }
    return NULL;
}

static void tpu_service(MC68376TPU *m, TpuChan *c, int64_t t)
{
    int code = cfsr_get(m, c->n);
    TpuFn fn = tpu_function(code);
    int hsr = hsr_get(c);

    if (hsr && !c->hsr_blocked) {
        if (!fn) {
            /* no ROM function: the request is never serviced */
            if (!(m->unimp_logged & (1 << c->n))) {
                m->unimp_logged |= 1 << c->n;
                qemu_log_mask(LOG_UNIMP, "mc68376.tpu: channel %d: function "
                              "$%X %s, channel stays idle\n", c->n, code,
                              code == FN_SM ? "(stepper motor) not modelled"
                              : "is not in the A mask ROM (custom microcode "
                              "is not supported)");
            }
            c->hsr_blocked = true;
            return;
        }
        fn(m, c, WHY_HSR, hsr, t);
        field2_set(m->hsrr, c->n, 0);
        return;
    }
    if (c->lsl) {
        c->lsl = false;
        if (fn) {
            fn(m, c, WHY_LINK, 0, t);
        }
        return;
    }
    if (fn) {
        fn(m, c, WHY_EVENT, 0, t);
    }
    c->mrl = c->tdl = c->awl = false;
}

/*
 * Process match events and channel services in time order up to @now
 * (events at @now itself only when @inclusive; pin input handlers
 * leave them for later so that every edge of one instant, e.g. a tooth
 * on both a channel pin and T2CLK, is in before the microcode runs).
 */
static void tpu_advance(MC68376TPU *m, int64_t now, bool inclusive)
{
    int guard = 0;
    int64_t next = INT64_MAX;
    bool left = false;

    if (m->in_advance) {
        return;
    }
    m->in_advance = true;
    for (;;) {
        TpuChan *cm = NULL, *cs = NULL;
        int64_t tm = INT64_MAX, ts = INT64_MAX;

        for (int n = 0; n < TPU_NCH; n++) {
            TpuChan *c = &m->ch[n];
            if (c->match_en && c->match_ns < tm) {
                tm = c->match_ns;
                cm = c;
            }
        }
        if (!tpu_stopped(m)) {
            for (int p = 3; p >= 1; p--) {
                for (int n = 0; n < TPU_NCH; n++) {
                    TpuChan *c = &m->ch[n];
                    int64_t r;
                    if (prio(c) != p || !ch_pending(c)) {
                        continue;
                    }
                    r = MAX(c->req_ns, m->lt);
                    if (r < ts) {
                        ts = r;
                        cs = c;
                    }
                }
            }
        }
        if (!cm && !cs) {
            break;
        }
        if (MIN(tm, ts) > now || (!inclusive && MIN(tm, ts) == now)) {
            left = ts <= now;
            break;
        }
        if (++guard > 100000) {
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.tpu: runaway channel "
                          "activity, matches disabled\n");
            for (int n = 0; n < TPU_NCH; n++) {
                ch_disarm(&m->ch[n]);
            }
            break;
        }
        if (tm <= ts) {
            m->lt = MAX(m->lt, tm);
            do_match(m, cm, tm);
        } else {
            m->lt = MAX(m->lt, ts);
            tpu_service(m, cs, ts);
        }
    }
    m->in_advance = false;

    for (int n = 0; n < TPU_NCH; n++) {
        if (m->ch[n].match_en) {
            next = MIN(next, m->ch[n].match_ns);
        }
    }
    if (next == INT64_MAX) {
        timer_del(m->timer);
    } else {
        timer_mod(m->timer, MAX(next, now));
    }
    if (left) {
        qemu_bh_schedule(m->bh);
    }
    tpu_update_irq(m);
}

static void tpu_timer_cb(void *opaque)
{
    MC68376TPU *m = opaque;

    tpu_advance(m, mc68376_now(), true);
}

static void tpu_bh_cb(void *opaque)
{
    MC68376TPU *m = opaque;

    tpu_advance(m, mc68376_now(), true);
}

static void tpu_clock_changed(Notifier *n, void *data)
{
    MC68376TPU *m = container_of(n, MC68376TPU, clock_notifier);
    int64_t now = mc68376_now();

    /* the counters still hold the old rate: catch up first */
    tpu_advance(m, now, true);
    tpu_clocks(m, now);
    tpu_advance(m, now, true);
}

/* ---------------------------------------------------------------------- */
/* Pins                                                                   */
/* ---------------------------------------------------------------------- */

static void tpu_pin_cb(void *opaque, int level)
{
    TpuChan *c = opaque;
    MC68376TPU *m = c->m;
    int64_t now = mc68376_now();
    uint8_t e = level ? 1 : 2;

    tpu_advance(m, now, false);
    if (c->out || c->level == level) {
        return;
    }
    c->level = level;
    if (c->edge & e) {
        c->cap = tcr_value(m, c->cap_tcr, now);
        c->tdl_level = level;
        c->ev_ns = now;
        ch_latch(c, &c->tdl, now);
        qemu_bh_schedule(m->bh);
    }
}

/*
 * T2CLK: TCR2 clock (T2CG = 0, counted on rising edges; the manual does
 * not name the edge, rising is assumed) or DIV8 clock gate (T2CG = 1).
 * The synchronizer and digital filter delay is not modelled.
 */
static void tpu_t2clk_cb(void *opaque, int level)
{
    MC68376TPU *m = opaque;
    int64_t now = mc68376_now();
    int old = m->t2clk;

    tpu_advance(m, now, false);
    m->t2clk = level;
    if (m->mcr & MCR_T2CG) {
        cnt_rebase(&m->tcr2c, now, mc68376_sysclk_hz(m->soc),
                   tcr2_gated_div(m));
        for (int n = 0; n < TPU_NCH; n++) {
            TpuChan *c = &m->ch[n];
            if (c->match_en && c->match_tcr == TCR2) {
                ch_calc_match(m, c, now);
            }
        }
    } else if (level && !old && !tpu_stopped(m)) {
        if (++m->t2_presc >= (1 << MCR_TCR2P(m->mcr))) {
            uint16_t o = tcr_value(m, TCR2, now);
            m->t2_presc = 0;
            m->tcr2_edges++;
            tcr2_changed(m, o, now);
        }
    }
    qemu_bh_schedule(m->bh);
}

/* ---------------------------------------------------------------------- */
/* Host interface                                                         */
/* ---------------------------------------------------------------------- */

static uint16_t tpu_reg_read(MC68376TPU *m, hwaddr addr, uint16_t bytes)
{
    if (addr >= TPU_PRAM_OFF) {
        return pram_rd(m, addr - TPU_PRAM_OFF);
    }
    switch (addr) {
    case R_TPUMCR:
        return m->mcr | (tpu_stopped(m) ? MCR_STF : 0);
    case R_DSCR:
        return m->dscr;
    case R_TICR:
        return m->ticr;
    case R_CIER:
        return m->cier;
    case R_CFSR0 ... R_CFSR0 + 6:
        return m->cfsr[(addr - R_CFSR0) / 2];
    case R_HSQR0:
    case R_HSQR0 + 2:
        return m->hsqr[(addr - R_HSQR0) / 2];
    case R_HSRR0:
    case R_HSRR0 + 2:
        return m->hsrr[(addr - R_HSRR0) / 2];
    case R_CPR0:
    case R_CPR0 + 2:
        return m->cpr[(addr - R_CPR0) / 2];
    case R_CISR:
        m->cisr_armed |= m->cisr & bytes;
        return m->cisr;
    default:
        /* TCR, DSSR, LR, SGLR, DCNR (factory test), unused: zero */
        return 0;
    }
}

static void tpu_write_mcr(MC68376TPU *m, uint16_t val, uint16_t mask,
                          int64_t now)
{
    uint16_t wm = MCR_WMASK & mask;

    if (m->mcr_written) {
        wm &= ~MCR_EMU;     /* EMU can be written only once (11.6.1.3) */
    }
    m->mcr_written = true;
    m->mcr = (m->mcr & ~wm) | (val & wm);
    if (m->mcr & MCR_EMU) {
        mc68376_tpuram_set_emulation(m->soc, true);
        if (!m->emu_logged) {
            m->emu_logged = true;
            qemu_log_mask(LOG_UNIMP, "mc68376.tpu: TPU microcode emulation "
                          "not supported (TPUMCR.EMU set): TPURAM microcode "
                          "is not executed, A mask functions keep running "
                          "as ROM models\n");
        }
    }
    tpu_clocks(m, now);
}

static void tpu_reg_write(MC68376TPU *m, hwaddr addr, uint16_t val,
                          uint16_t mask, int64_t now)
{
    uint16_t old;

    if (addr >= TPU_PRAM_OFF) {
        addr -= TPU_PRAM_OFF;
        pram_wr(m, addr, (pram_rd(m, addr) & ~mask) | (val & mask));
        return;
    }
    switch (addr) {
    case R_TPUMCR:
        tpu_write_mcr(m, val, mask, now);
        break;
    case R_DSCR:
        m->dscr = (m->dscr & ~mask) | (val & mask & 0x87ff);
        break;
    case R_TICR:
        m->ticr = (m->ticr & ~mask) | (val & mask & TICR_WMASK);
        break;
    case R_CIER:
        m->cier = (m->cier & ~mask) | (val & mask);
        break;
    case R_CFSR0 ... R_CFSR0 + 6: {
        int r = (addr - R_CFSR0) / 2;
        m->cfsr[r] = (m->cfsr[r] & ~mask) | (val & mask);
        for (int i = 0; i < 4; i++) {
            m->ch[(3 - r) * 4 + i].hsr_blocked = false;
        }
        break;
    }
    case R_HSQR0:
    case R_HSQR0 + 2: {
        int r = (addr - R_HSQR0) / 2;
        m->hsqr[r] = (m->hsqr[r] & ~mask) | (val & mask);
        break;
    }
    case R_HSRR0:
    case R_HSRR0 + 2: {
        /*
         * A non-zero field requests service; writing %00 leaves a
         * pending request alone (UNVERIFIED, the manual only says the
         * host writes one of the three non-zero states).
         */
        int r = (addr - R_HSRR0) / 2;
        for (int i = 0; i < 8; i++) {
            int n = (1 - r) * 8 + i;
            int v = (val >> (2 * i)) & 3;
            TpuChan *c = &m->ch[n];
            if (!v || !((mask >> (2 * i)) & 3)) {
                continue;
            }
            if (!ch_pending(c)) {
                c->req_ns = now;
            }
            field2_set(m->hsrr, n, v);
            c->hsr_blocked = false;
        }
        break;
    }
    case R_CPR0:
    case R_CPR0 + 2: {
        int r = (addr - R_CPR0) / 2;
        old = m->cpr[r];
        m->cpr[r] = (m->cpr[r] & ~mask) | (val & mask);
        for (int i = 0; i < 8; i++) {
            TpuChan *c = &m->ch[(1 - r) * 8 + i];
            if (!((old >> (2 * i)) & 3) && prio(c)) {
                c->req_ns = MAX(c->req_ns, now);
            }
        }
        break;
    }
    case R_CISR:
        /* clear: read the flag as one, then write zero (11.6.2.1) */
        m->cisr &= ~(m->cisr_armed & mask & ~val);
        m->cisr_armed &= ~mask;
        break;
    default:
        break;
    }
}

static uint64_t tpu_read(void *opaque, hwaddr addr, unsigned size)
{
    MC68376TPU *m = opaque;
    uint64_t v;

    tpu_advance(m, mc68376_now(), true);
    if (size == 4) {
        v = (uint32_t)tpu_reg_read(m, addr, 0xffff) << 16;
        v |= tpu_reg_read(m, addr + 2, 0xffff);
    } else if (size == 2) {
        v = tpu_reg_read(m, addr, 0xffff);
    } else {
        if (addr < TPU_PRAM_OFF && (addr & ~1) != R_CISR) {
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.tpu: byte read of "
                          "word register 0x%03" HWADDR_PRIx "\n", addr);
        }
        v = tpu_reg_read(m, addr & ~1, (addr & 1) ? 0x00ff : 0xff00);
        v = (addr & 1) ? v & 0xff : v >> 8;
    }
    tpu_update_irq(m);
    return v;
}

static void tpu_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    MC68376TPU *m = opaque;
    int64_t now = mc68376_now();

    tpu_advance(m, now, true);
    if (size == 4) {
        tpu_reg_write(m, addr, val >> 16, 0xffff, now);
        tpu_reg_write(m, addr + 2, val, 0xffff, now);
    } else if (size == 2) {
        tpu_reg_write(m, addr, val, 0xffff, now);
    } else {
        if (addr < TPU_PRAM_OFF && (addr & ~1) != R_CISR) {
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.tpu: byte write of "
                          "word register 0x%03" HWADDR_PRIx "\n", addr);
        }
        tpu_reg_write(m, addr & ~1, (addr & 1) ? val : val << 8,
                      (addr & 1) ? 0x00ff : 0xff00, now);
    }
    tpu_advance(m, now, true);
}

static const MemoryRegionOps tpu_ops = {
    .read = tpu_read,
    .write = tpu_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

void mc68376_tpu_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    MC68376TPU *m = g_new0(MC68376TPU, 1);
    char name[8];

    m->soc = s;
    s->tpu = m;
    memory_region_init_io(mr, OBJECT(s), &tpu_ops, m, "mc68376.tpu",
                          MC68376_TPU_SIZE);
    m->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tpu_timer_cb, m);
    m->bh = qemu_bh_new(tpu_bh_cb, m);
    m->clock_notifier.notify = tpu_clock_changed;
    mc68376_add_clock_notifier(s, &m->clock_notifier);
    imb_irq_register(s, &m->irq, "tpu");
    m->irq.iack = tpu_iack;
    m->irq.opaque = m;

    for (int n = 0; n < TPU_NCH; n++) {
        TpuChan *c = &m->ch[n];
        c->m = m;
        c->n = n;
        snprintf(name, sizeof(name), "TPU%d", n);
        c->pin = ecu_pin(name);
        ecu_pin_set_input_handler(c->pin, tpu_pin_cb, c);
    }
    m->t2clk_pin = ecu_pin("T2CLK");
    ecu_pin_set_input_handler(m->t2clk_pin, tpu_t2clk_cb, m);
}

void mc68376_tpu_reset(MC68376State *s)
{
    MC68376TPU *m = s->tpu;
    int64_t now = mc68376_now();

    timer_del(m->timer);
    m->mcr = MCR_SUPV;
    m->dscr = m->ticr = m->cier = m->cisr = m->cisr_armed = 0;
    memset(m->cfsr, 0, sizeof(m->cfsr));
    memset(m->hsqr, 0, sizeof(m->hsqr));
    memset(m->hsrr, 0, sizeof(m->hsrr));
    memset(m->cpr, 0, sizeof(m->cpr));
    m->mcr_written = false;
    m->emu_logged = false;
    m->unimp_logged = 0;
    m->tcr1 = (TpuCounter) { .base_ns = now };
    m->tcr2c = (TpuCounter) { .base_ns = now };
    m->tcr2_edges = 0;
    m->tcr2_off = 0;
    m->t2_presc = 0;
    m->t2clk = ecu_pin_level(m->t2clk_pin);
    m->lt = now;
    for (int n = 0; n < TPU_NCH; n++) {
        TpuChan *c = &m->ch[n];
        memset(&c->out, 0, sizeof(*c) - offsetof(TpuChan, out));
        c->level = ecu_pin_level(c->pin);
        c->match_ns = INT64_MAX;
    }
    tpu_clocks(m, now);
    m->irq.level = 0;
    m->irq.iarb = 0;
    m->irq.vector = 0;
    imb_irq_set(&m->irq, false);
}
