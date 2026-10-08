/*
 * Motorola MC68376 microcontroller: CPU32 core, system integration module
 * (SIM), standby RAM (SRAM), TPU emulation RAM (TPURAM), masked ROM
 * module (MRM), intermodule bus (IMB) interrupt arbitration and the
 * module register block.  QSM, QADC, CTM4, TPU and TouCAN live in their
 * own files (mc68376_*.c).
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D), sections 5
 * (SIM), 6 (SRAM), 7 (MRM), 12 (TPURAM) and appendix D (register
 * summary).  Section and table numbers below refer to that manual.
 *
 * Model assumptions (mode select pins at reset, 5.7.3): all data bus
 * pins high (weak pull-ups: 16-bit CSBOOT, CS[10:0] are chip selects,
 * port E/F pins are bus control/IRQ pins), MODCLK high (PLL enabled,
 * fsys synthesized from the reference on EXTAL), BKPT high (no BDM),
 * DATA14 high unless the "mrm-enabled" property is cleared.
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
#include "hw/core/cpu.h"
#include "accel/tcg/cpu-ops.h"
#include "system/address-spaces.h"
#include "system/runstate.h"
#include "system/memory.h"
#include "hw/m68k/mc68376.h"

int64_t mc68376_now(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static inline uint16_t merge16(uint16_t old, uint16_t val, uint16_t mask)
{
    return (old & ~mask) | (val & mask);
}

static void mc68376_update_memmap(MC68376State *s);

/* ---------------------------------------------------------------------- */
/* Clock synthesizer (5.3, D.2.3)                                         */
/* ---------------------------------------------------------------------- */

#define SYNCR_W         0x8000
#define SYNCR_X         0x4000
#define SYNCR_Y_SHIFT   8
#define SYNCR_EDIV      0x0080
#define SYNCR_SLOCK     0x0008
#define SYNCR_WMASK     0xff83      /* W X Y EDIV STSIM STEXT */

/*
 * fsys = fref / 128 * [4 (Y + 1) 2^(2W + X)] (5.3.2).  The reset value
 * $3F00 gives 8.388 MHz from the 4.194304 MHz reference (Table 5-3).
 */
uint32_t mc68376_sysclk_hz(MC68376State *s)
{
    uint64_t y = (s->syncr >> SYNCR_Y_SHIFT) & 0x3f;
    int w = !!(s->syncr & SYNCR_W), x = !!(s->syncr & SYNCR_X);

    return ((uint64_t)s->extal_hz * 4 * (y + 1)) << (2 * w + x) >> 7;
}

void mc68376_add_clock_notifier(MC68376State *s, Notifier *n)
{
    notifier_list_add(&s->clock_notifiers, n);
}

/* ---------------------------------------------------------------------- */
/* IMB interrupt arbitration (5.2.2, 5.8)                                 */
/* ---------------------------------------------------------------------- */

void imb_irq_register(MC68376State *s, IMBIrq *irq, const char *name)
{
    assert(s->nirqs < ARRAY_SIZE(s->irqs));
    irq->soc = s;
    irq->name = name;
    s->irqs[s->nirqs++] = irq;
}

/*
 * Arbitration among the requests of one level: the highest IARB wins.
 * Equal non-zero IARB values are a programming error on the real chip
 * (5.8.3 WARNING); here the source registered first wins, which gives
 * the documented SIM order (PIT before the IRQ pins, 5.8.3) and the QSM
 * order (QSPI before SCI, 9.2.1.3).  Returns NULL when no request of
 * that level can win (none pending, or all with IARB = 0).
 */
static IMBIrq *imb_arbitrate(MC68376State *s, int level, bool *any)
{
    IMBIrq *best = NULL;

    *any = false;
    for (int i = 0; i < s->nirqs; i++) {
        IMBIrq *q = s->irqs[i];
        if (!q->pending || q->level != level) {
            continue;
        }
        *any = true;
        if (q->iarb && (!best || q->iarb > best->iarb)) {
            best = q;
        }
    }
    return best;
}

void imb_irq_update(MC68376State *s)
{
    int level = 0;
    uint8_t vector = MC68376_VEC_SPURIOUS;
    IMBIrq *w;
    bool any;

    for (int i = 0; i < s->nirqs; i++) {
        IMBIrq *q = s->irqs[i];
        if (q->pending && q->level > level && q->level <= 7) {
            level = q->level;
        }
    }
    if (level) {
        w = imb_arbitrate(s, level, &any);
        if (w) {
            vector = w->vector;
        }
    }
    if (s->cpu) {
        m68k_set_irq_level(s->cpu, level, vector);
    }
}

void imb_irq_set(IMBIrq *irq, bool pending)
{
    if (irq->pending == pending) {
        return;
    }
    irq->pending = pending;
    if (!irq->soc->in_iack) {
        imb_irq_update(irq->soc);
    }
}

/*
 * Interrupt acknowledge cycle for @level: IARB contention picks the
 * source that supplies the vector (5.8.3).  If nobody can contend
 * (IARB = 0) the spurious interrupt monitor terminates the cycle with
 * BERR and the CPU uses the spurious interrupt vector (5.4.4, 5.8.4 E).
 */
static int imb_iack(MC68376State *s, int level)
{
    IMBIrq *w;
    bool any;

    s->in_iack = true;
    w = imb_arbitrate(s, level, &any);
    if (w && w->iack) {
        w->iack(w);
    }
    s->in_iack = false;
    if (!w) {
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376: spurious interrupt at "
                      "level %d (%s)\n", level,
                      any ? "requesting module has IARB = 0" : "no request");
        return MC68376_VEC_SPURIOUS;
    }
    return w->vector;
}

/*
 * QEMU's m68k core samples the vector when the request is raised
 * (m68k_set_irq_level).  The CPU32 gets it in the IACK cycle, and the
 * PIT negates its request in that cycle, so the cpu32 core calls back
 * into the SIM when it takes the interrupt (env->iack).
 *
 * Level 7 is non-maskable and transition sensitive (5.8.2); the cpu32
 * core implements that (m68k_cpu32_irq_ready()), the IMB only reports
 * the current request level.
 */
static int mc68376_cpu_iack(void *opaque, int level)
{
    return imb_iack(opaque, level);
}

static void mc68376_cpu_iack_done(void *opaque)
{
    imb_irq_update(opaque);
}

static void mc68376_install_iack(MC68376State *s)
{
    CPUM68KState *env = &s->cpu->env;

    env->iack = mc68376_cpu_iack;
    env->iack_done = mc68376_cpu_iack_done;
    env->iack_opaque = s;
}

/* ---------------------------------------------------------------------- */
/* Periodic interrupt timer (5.4.6, 5.4.7, D.2.13, D.2.14)                */
/* ---------------------------------------------------------------------- */

#define PICR_PIRQL(v)   (((v) >> 8) & 7)
#define PITR_PTP        0x0100

/*
 * PIT period = 128 x PITM x (512 if PTP) x 4 / fref with the PLL enabled
 * (fast reference, 5.4.6).  The modulus counter is clocked from fref/128,
 * independent of SYNCR.
 */
static int64_t pit_period_ns(MC68376State *s, uint8_t pitm, bool ptp)
{
    uint64_t div = 128ULL * pitm * (ptp ? 512 : 1) * 4;

    return muldiv64(div, NANOSECONDS_PER_SECOND, s->extal_hz);
}

static void pit_start(MC68376State *s, int64_t from)
{
    s->pit_modulus = s->pitr & 0xff;
    s->pit_ptp = s->pitr & PITR_PTP;
    if (!s->pit_modulus) {
        timer_del(s->pit_timer);
        return;
    }
    s->pit_deadline_ns = from + pit_period_ns(s, s->pit_modulus, s->pit_ptp);
    timer_mod(s->pit_timer, s->pit_deadline_ns);
}

static void pit_cb(void *opaque)
{
    MC68376State *s = opaque;

    /* the timer keeps running while the interrupt is disabled (5.4.7) */
    if (PICR_PIRQL(s->picr)) {
        imb_irq_set(&s->pit_irq, true);
    }
    /* a new PITR value is loaded when the current count completes */
    pit_start(s, s->pit_deadline_ns);
}

/* the PIT has no status flag: its request is negated by the IACK cycle */
static void pit_iack(IMBIrq *irq)
{
    imb_irq_set(irq, false);
}

static void pit_update_irq(MC68376State *s)
{
    s->pit_irq.level = PICR_PIRQL(s->picr);
    s->pit_irq.vector = s->picr & 0xff;
    if (!s->pit_irq.level) {
        s->pit_irq.pending = false;
    }
    imb_irq_update(s);
}

/* ---------------------------------------------------------------------- */
/* Software watchdog (5.4.5, D.2.12, D.2.15)                              */
/* ---------------------------------------------------------------------- */

#define SYPCR_SWE       0x80
#define SYPCR_SWP       0x40
#define SYPCR_SWT(v)    (((v) >> 4) & 3)

#define RSR_EXT         0x80
#define RSR_POW         0x40
#define RSR_SW          0x20
#define RSR_HLT         0x10
#define RSR_SYS         0x02
#define RSR_TST         0x01

/*
 * Time-out = 128 x ratio / fref for the fast reference (5.4.5), ratio =
 * 2^9, 2^11, 2^13, 2^15 (SWP = 0) or 2^18 .. 2^24 (SWP = 1) (Table 5-6).
 * Table 5-6/D-7 write the ratios as "/ fsys", but the formula and Figure
 * 5-7 show the watchdog clocked from fref / 128 like the PIT; the
 * formula is used here.
 */
static int64_t swt_period_ns(MC68376State *s)
{
    int shift = (s->sypcr & SYPCR_SWP ? 18 : 9) + 2 * SYPCR_SWT(s->sypcr);

    return muldiv64(128ULL << shift, NANOSECONDS_PER_SECOND, s->extal_hz);
}

static void swt_restart(MC68376State *s)
{
    if (s->sypcr & SYPCR_SWE) {
        timer_mod(s->swt_timer, mc68376_now() + swt_period_ns(s));
    } else {
        timer_del(s->swt_timer);
    }
}

static void swt_cb(void *opaque)
{
    MC68376State *s = opaque;

    if (!(s->sypcr & SYPCR_SWE)) {
        return;
    }
    if (s->wdt_reset) {
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376: software watchdog reset\n");
        s->rsr_next = RSR_SW;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        return;
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "mc68376: software watchdog time-out (reset disabled)\n");
    swt_restart(s);
}

/* ---------------------------------------------------------------------- */
/* Ports E, F, C and the IRQ pins (5.9.1.4, 5.10, D.2.6-D.2.11, D.2.16)   */
/* ---------------------------------------------------------------------- */

/*
 * A data register read returns the pin level only for a pin configured
 * as discrete input (PAR = 0, DDR = 0), the latch otherwise (5.10.3).
 */
static uint8_t port_read(uint8_t latch, uint8_t ddr, uint8_t par,
                         EcuPin **pins)
{
    uint8_t v = 0;

    for (int b = 0; b < 8; b++) {
        if (!(par & (1 << b)) && !(ddr & (1 << b))) {
            v |= ecu_pin_level(pins[b]) << b;
        } else {
            v |= latch & (1 << b);
        }
    }
    return v;
}

/* returns the mask of pins driven as outputs */
static uint8_t port_drive(uint8_t latch, uint8_t ddr, uint8_t par,
                          EcuPin **pins)
{
    uint8_t driven = 0;

    for (int b = 0; b < 8; b++) {
        if (!(par & (1 << b)) && (ddr & (1 << b))) {
            ecu_pin_mcu_drive(pins[b], (latch >> b) & 1);
            driven |= 1 << b;
        }
    }
    return driven;
}

static void portf_drive(MC68376State *s)
{
    s->pf_driven |= port_drive(s->portf, s->ddrf, s->pfpar, s->pf_pin);
}

/* chip-select pin assignment field (Table 5-19); n = -1 for CSBOOT */
static int cs_pin_field(MC68376State *s, int n)
{
    if (n < 0) {
        return s->cspar0 & 3;
    }
    if (n < 6) {
        return (s->cspar0 >> (2 * n + 2)) & 3;
    }
    return (s->cspar1 >> (2 * (n - 6))) & 3;
}

/* PC[6:0] are the discrete outputs of CS[9:3] (5.9.1.4) */
static void portc_drive(MC68376State *s)
{
    for (int b = 0; b < 7; b++) {
        if (cs_pin_field(s, b + 3) == 0) {
            ecu_pin_mcu_drive(s->pc_pin[b], (s->portc >> b) & 1);
        }
    }
}

/*
 * IRQ[7:1] are active-low level-sensitive inputs (5.8.2); the transition
 * sensitivity of level 7 is a CPU32 property and handled by the cpu32
 * core.  The external requests come from the SIM and use its IARB.  The
 * vector: the IACK cycle for an external request goes to the external
 * bus, where a device supplies a vector or AVEC is asserted (pin or
 * chip-select logic, 5.8.3, 5.9.3); external devices are not modelled,
 * so every IRQ pin request is autovectored (vector 24 + n).
 *
 * The physical pin can be driven through either of its names, "IRQn" or
 * "PFn"; it reads low when either is low (both idle high, as with the
 * usual pull-ups).
 */
static int irq_pin_level(MC68376State *s, int n)
{
    return ecu_pin_level(s->pf_pin[n]) && ecu_pin_level(s->irq_ecu_pin[n]);
}

static void irq_pins_eval(MC68376State *s)
{
    for (int n = 1; n <= 7; n++) {
        s->irq_pin_level[n] = irq_pin_level(s, n);
        s->irq_pin[n].pending = (s->pfpar & (1 << n)) &&
                                !s->irq_pin_level[n];
    }
    imb_irq_update(s);
}

static void irq_pin_cb(void *opaque, int level)
{
    irq_pins_eval(opaque);
}

/* ---------------------------------------------------------------------- */
/* SIM register block ($YFFA00, Table D-3)                                */
/* ---------------------------------------------------------------------- */

#define SIMCR_MM        0x0040
#define SIMCR_WMASK     0xe3cf      /* EXOFF FRZSW FRZBM SHEN SUPV MM IARB */

static void sim_set_iarb(MC68376State *s)
{
    uint8_t iarb = s->simcr & 0xf;

    s->pit_irq.iarb = iarb;
    for (int n = 1; n <= 7; n++) {
        s->irq_pin[n].iarb = iarb;
    }
    imb_irq_update(s);
}

static uint16_t sim_read16(MC68376State *s, hwaddr off)
{
    switch (off) {
    case 0x00:
        return s->simcr;
    case 0x02:
        return s->simtr;
    case 0x04:
        /* the VCO locks instantly */
        return s->syncr | SYNCR_SLOCK;
    case 0x06:
        return s->rsr;
    case 0x08:
        return s->simtre;
    case 0x10: case 0x12:
        return port_read(s->porte, s->ddre, s->pepar, s->pe_pin);
    case 0x14:
        return s->ddre;
    case 0x16:
        return s->pepar;
    case 0x18: case 0x1a:
        return port_read(s->portf, s->ddrf, s->pfpar, s->pf_pin);
    case 0x1c:
        return s->ddrf;
    case 0x1e:
        return s->pfpar;
    case 0x20:
        return s->sypcr;
    case 0x22:
        return s->picr;
    case 0x24:
        return s->pitr;
    case 0x26:
        return 0;               /* SWSR reads zero (D.2.15) */
    case 0x30 ... 0x3a:
        return s->sim_test[(off - 0x30) / 2];
    case 0x40:
        return s->portc;
    case 0x44:
        return s->cspar0;
    case 0x46:
        return s->cspar1;
    case 0x48:
        return s->csbarbt;
    case 0x4a:
        return s->csorbt;
    case 0x4c ... 0x76:
        if ((off - 0x4c) & 2) {
            return s->csor[(off - 0x4c) / 4];
        }
        return s->csbar[(off - 0x4c) / 4];
    }
    return 0;                   /* not used */
}

static void sim_write16(MC68376State *s, hwaddr off, uint16_t val,
                        uint16_t mask)
{
    uint16_t old;

    switch (off) {
    case 0x00:
        old = s->simcr;
        s->simcr = merge16(old, val, mask & SIMCR_WMASK);
        /* MM can only be written once (D.2.1) */
        if (mask & SIMCR_MM) {
            if (s->mm_written) {
                s->simcr = (s->simcr & ~SIMCR_MM) | (old & SIMCR_MM);
            }
            s->mm_written = true;
        }
        if ((old ^ s->simcr) & SIMCR_MM) {
            mc68376_update_memmap(s);
        }
        sim_set_iarb(s);
        break;
    case 0x02:
        s->simtr = merge16(s->simtr, val, mask);
        break;
    case 0x04:
        old = s->syncr;
        s->syncr = merge16(s->syncr, val, mask & SYNCR_WMASK);
        if (old != s->syncr) {
            notifier_list_notify(&s->clock_notifiers, s);
        }
        break;
    case 0x06:
        break;                  /* RSR: writes have no effect */
    case 0x08:
        s->simtre = merge16(s->simtre, val, mask);
        break;
    case 0x10: case 0x12:
        s->porte = merge16(s->porte, val, mask & 0xff);
        port_drive(s->porte, s->ddre, s->pepar, s->pe_pin);
        break;
    case 0x14:
        s->ddre = merge16(s->ddre, val, mask & 0xff);
        port_drive(s->porte, s->ddre, s->pepar, s->pe_pin);
        break;
    case 0x16:
        s->pepar = merge16(s->pepar, val, mask & 0xff);
        port_drive(s->porte, s->ddre, s->pepar, s->pe_pin);
        break;
    case 0x18: case 0x1a:
        s->portf = merge16(s->portf, val, mask & 0xff);
        portf_drive(s);
        break;
    case 0x1c:
        s->ddrf = merge16(s->ddrf, val, mask & 0xff);
        portf_drive(s);
        break;
    case 0x1e:
        s->pfpar = merge16(s->pfpar, val, mask & 0xff);
        portf_drive(s);
        irq_pins_eval(s);
        break;
    case 0x20:
        /* SYPCR can be written once after reset (D.2.12) */
        if (!(mask & 0xff)) {
            break;
        }
        if (s->sypcr_written) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "mc68376: SYPCR already written, ignored\n");
            break;
        }
        s->sypcr_written = true;
        old = s->sypcr;
        s->sypcr = val & 0xff;
        /*
         * A new SWT/SWP value only takes effect with the next service
         * sequence (5.4.5); enabling/disabling acts at once.
         */
        if (!(s->sypcr & SYPCR_SWE)) {
            timer_del(s->swt_timer);
        } else if (!(old & SYPCR_SWE)) {
            swt_restart(s);
        }
        break;
    case 0x22:
        s->picr = merge16(s->picr, val, mask & 0x07ff);
        pit_update_irq(s);
        break;
    case 0x24: {
        bool was_running = s->pit_modulus != 0;
        s->pitr = merge16(s->pitr, val, mask & 0x01ff);
        if (!(s->pitr & 0xff)) {
            /* a zero modulus turns the timer off */
            s->pit_modulus = 0;
            timer_del(s->pit_timer);
        } else if (!was_running) {
            pit_start(s, mc68376_now());
        }
        /* else: loaded when the current count completes (5.4.6) */
        break;
    }
    case 0x26:
        /*
         * Service sequence $55, $AA (5.4.5).  Other values are ignored
         * and do not break a started sequence (the manual only requires
         * the two writes in order).
         */
        if (!(mask & 0xff)) {
            break;
        }
        if ((val & 0xff) == 0x55) {
            s->swsr_armed = true;
        } else if ((val & 0xff) == 0xaa && s->swsr_armed) {
            s->swsr_armed = false;
            swt_restart(s);
        }
        break;
    case 0x30 ... 0x3a:
        s->sim_test[(off - 0x30) / 2] =
            merge16(s->sim_test[(off - 0x30) / 2], val, mask);
        break;
    case 0x40:
        s->portc = merge16(s->portc, val, mask & 0x7f);
        portc_drive(s);
        break;
    case 0x44:
        /* bits 15:14 read zero, bit 1 always reads one (D.2.17) */
        s->cspar0 = merge16(s->cspar0, val, mask & 0x3fff) | 0x0002;
        mc68376_update_memmap(s);
        portc_drive(s);
        break;
    case 0x46:
        s->cspar1 = merge16(s->cspar1, val, mask & 0x03ff);
        mc68376_update_memmap(s);
        portc_drive(s);
        break;
    case 0x48:
        s->csbarbt = merge16(s->csbarbt, val, mask);
        mc68376_update_memmap(s);
        break;
    case 0x4a:
        s->csorbt = merge16(s->csorbt, val, mask);
        mc68376_update_memmap(s);
        break;
    case 0x4c ... 0x76:
        if ((off - 0x4c) & 2) {
            uint16_t *r = &s->csor[(off - 0x4c) / 4];
            *r = merge16(*r, val, mask);
        } else {
            uint16_t *r = &s->csbar[(off - 0x4c) / 4];
            *r = merge16(*r, val, mask);
        }
        mc68376_update_memmap(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.sim: write to unused "
                      "register $YFFA%02x\n", (unsigned)off);
        break;
    }
}

/* ---------------------------------------------------------------------- */
/* SRAM, TPURAM and MRM control registers (D.3, D.9, D.4)                 */
/* ---------------------------------------------------------------------- */

#define RAMMCR_STOP     0x8000
#define RAMMCR_RLCK     0x0800
#define RAMMCR_RASP     0x0300

static uint16_t sram_ctl_read16(MC68376State *s, hwaddr off)
{
    switch (off) {
    case 0: return s->sram_mcr;
    case 4: return s->sram_bar_h;
    case 6: return s->sram_bar_l;
    }
    return 0;                   /* RAMTST: factory test */
}

static void sram_ctl_write16(MC68376State *s, hwaddr off, uint16_t val,
                             uint16_t mask)
{
    bool bar_ok = (s->sram_mcr & RAMMCR_STOP) &&
                  !(s->sram_mcr & RAMMCR_RLCK);

    switch (off) {
    case 0: {
        /* RLCK can be written once only to a value of one (6.2) */
        uint16_t rlck = s->sram_mcr & RAMMCR_RLCK;
        s->sram_mcr = merge16(s->sram_mcr, val,
                              mask & (RAMMCR_STOP | RAMMCR_RASP | RAMMCR_RLCK));
        s->sram_mcr |= rlck;
        break;
    }
    case 4:
    case 6:
        /* only while in low-power stop and unlocked (D.3.4) */
        if (!bar_ok) {
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.sram: RAMBAH/RAMBAL "
                          "write ignored (STOP = 0 or RLCK = 1)\n");
            return;
        }
        if (off == 4) {
            s->sram_bar_h = merge16(s->sram_bar_h, val, mask & 0x00ff);
        } else {
            s->sram_bar_l = merge16(s->sram_bar_l, val, mask & 0xf000);
        }
        break;
    default:
        return;
    }
    mc68376_update_memmap(s);
}

#define TRAMMCR_STOP    0x8000
#define TRAMMCR_RASP    0x0100
#define TRAMBAR_RAMDS   0x0001

static uint32_t tpuram_base(MC68376State *s)
{
    return (uint32_t)(s->tpuram_bar & 0xfff0) << 8;
}

static uint32_t modules_base(MC68376State *s)
{
    return s->simcr & SIMCR_MM ? 0xfff000 : 0x7ff000;
}

/*
 * The array is disabled out of reset and stays disabled if the base
 * address overlaps the module register block (12.3).
 */
static bool tpuram_enabled(MC68376State *s)
{
    return s->tpuram_bar_written && tpuram_base(s) != modules_base(s);
}

static uint16_t tpuram_ctl_read16(MC68376State *s, hwaddr off)
{
    switch (off) {
    case 0:
        return s->tpuram_mcr;
    case 4:
        /*
         * D.9.3 shows RAMDS = 0 in the reset line, but the text (12.3,
         * D.9.3) says the array is disabled at reset; RAMDS reads 1 until
         * a valid base address has been written.
         */
        return s->tpuram_bar | (tpuram_enabled(s) ? 0 : TRAMBAR_RAMDS);
    }
    return 0;
}

static void tpuram_ctl_write16(MC68376State *s, hwaddr off, uint16_t val,
                               uint16_t mask)
{
    switch (off) {
    case 0:
        s->tpuram_mcr = merge16(s->tpuram_mcr, val,
                                mask & (TRAMMCR_STOP | TRAMMCR_RASP));
        break;
    case 4:
        /*
         * TRAMBAR can be written only once after reset; the first write
         * (even of one byte) locks it (12.3).
         */
        if (s->tpuram_bar_written) {
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.tpuram: TRAMBAR already "
                          "written, ignored\n");
            return;
        }
        s->tpuram_bar = merge16(s->tpuram_bar, val, mask & 0xfff0);
        s->tpuram_bar_written = true;
        break;
    default:
        return;
    }
    mc68376_update_memmap(s);
}

#define MRMCR_STOP      0x8000
#define MRMCR_BOOT      0x1000
#define MRMCR_LOCK      0x0800
#define MRMCR_ASPC      0x0300
#define MRMCR_WAIT      0x00c0

static uint16_t mrm_ctl_read16(MC68376State *s, hwaddr off)
{
    switch (off) {
    case 0x00: return s->mrm_mcr;
    case 0x04: return s->mrm_bah;
    case 0x06: return s->mrm_bal;
    case 0x08: return s->mrm_sig[0];
    case 0x0a: return s->mrm_sig[1];
    case 0x10: case 0x12: case 0x14: case 0x16:
        return s->mrm_bsw[(off - 0x10) / 2];
    }
    return 0;                   /* not implemented: reads zero (7.1) */
}

static void mrm_ctl_write16(MC68376State *s, hwaddr off, uint16_t val,
                            uint16_t mask)
{
    bool unlocked = (s->mrm_mcr & MRMCR_STOP) && !(s->mrm_mcr & MRMCR_LOCK);
    uint16_t wmask;

    switch (off) {
    case 0x00:
        /*
         * STOP is always writable, LOCK once to one; ASPC and WAIT only
         * while LOCK = 0 and STOP = 1; BOOT is read only and EMUL reads
         * zero on the MC68376 (D.4.1).
         */
        wmask = MRMCR_STOP | MRMCR_LOCK;
        if (unlocked) {
            wmask |= MRMCR_ASPC | MRMCR_WAIT;
        }
        s->mrm_mcr = merge16(s->mrm_mcr, val, mask & wmask) |
                     (s->mrm_mcr & MRMCR_LOCK);
        break;
    case 0x04:
    case 0x06:
        if (!unlocked) {
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.mrm: ROMBAH/ROMBAL "
                          "write ignored (STOP = 0 or LOCK = 1)\n");
            return;
        }
        if (off == 0x04) {
            s->mrm_bah = merge16(s->mrm_bah, val, mask & 0x00ff);
        } else {
            s->mrm_bal = merge16(s->mrm_bal, val, mask & 0xe000);
        }
        break;
    default:
        return;                 /* signature/bootstrap words: read only */
    }
    mc68376_update_memmap(s);
}

/* ---------------------------------------------------------------------- */
/* 16-bit register access helpers                                         */
/* ---------------------------------------------------------------------- */

/*
 * The modules are 16-bit IMB slaves.  Byte accesses reach one half of a
 * register; a long-word access is split into two word accesses by the
 * memory core (impl.max_access_size = 2).
 */
typedef uint16_t (*Reg16Read)(MC68376State *s, hwaddr off);
typedef void (*Reg16Write)(MC68376State *s, hwaddr off, uint16_t val,
                           uint16_t mask);

static uint64_t reg16_read(MC68376State *s, Reg16Read fn, hwaddr addr,
                           unsigned size)
{
    uint16_t v = fn(s, addr & ~1);

    if (size == 2) {
        return v;
    }
    return addr & 1 ? v & 0xff : v >> 8;
}

static void reg16_write(MC68376State *s, Reg16Write fn, hwaddr addr,
                        uint64_t val, unsigned size)
{
    if (size == 2) {
        fn(s, addr, val, 0xffff);
    } else if (addr & 1) {
        fn(s, addr & ~1, val & 0xff, 0x00ff);
    } else {
        fn(s, addr, (val & 0xff) << 8, 0xff00);
    }
}

#define REG16_OPS(name)                                                 \
    static uint64_t name##_mmio_read(void *opaque, hwaddr addr,         \
                                     unsigned size)                     \
    {                                                                   \
        return reg16_read(opaque, name##_read16, addr, size);           \
    }                                                                   \
    static void name##_mmio_write(void *opaque, hwaddr addr,            \
                                  uint64_t val, unsigned size)          \
    {                                                                   \
        reg16_write(opaque, name##_write16, addr, val, size);           \
    }                                                                   \
    static const MemoryRegionOps name##_ops = {                         \
        .read = name##_mmio_read,                                       \
        .write = name##_mmio_write,                                     \
        .endianness = DEVICE_BIG_ENDIAN,                                \
        .valid = { .min_access_size = 1, .max_access_size = 4 },        \
        .impl = { .min_access_size = 1, .max_access_size = 2 },         \
    }

REG16_OPS(sim);
REG16_OPS(sram_ctl);
REG16_OPS(tpuram_ctl);
REG16_OPS(mrm_ctl);

static uint64_t modules_bg_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376: read from unimplemented module "
                  "register $YFF%03x\n", (unsigned)addr);
    return 0;
}

static void modules_bg_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376: write to unimplemented module "
                  "register $YFF%03x\n", (unsigned)addr);
}

static const MemoryRegionOps modules_bg_ops = {
    .read = modules_bg_read,
    .write = modules_bg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---------------------------------------------------------------------- */
/* Address map: module block, internal arrays, chip-select decoding       */
/* ---------------------------------------------------------------------- */

/*
 * Priorities inside the 16 MiB bus: internal modules and arrays take
 * precedence over chip-select windows (5.9.1).  An SRAM array placed over
 * the register block hides the registers (6, "overlap makes the
 * registers inaccessible"); the MRM register block takes precedence over
 * an overlapping MRM array (7.2).
 */
#define PRIO_CS         0
#define PRIO_ARRAY      10
#define PRIO_MODULES    20
#define PRIO_SRAM       30

static const uint32_t cs_block_size[8] = {
    2 * KiB, 8 * KiB, 16 * KiB, 64 * KiB,
    128 * KiB, 256 * KiB, 512 * KiB, 1 * MiB,       /* Table D-13 */
};

/*
 * Chip-select n (-1 = CSBOOT) decodes a block of normal (user and/or
 * supervisor) space when its pin is assigned as a chip select (CSxPA =
 * %10 or %11), the option register's BYTE and R/W fields are not
 * "disable" and SPACE is not CPU space (5.9.1, 5.9.2).
 */
static bool cs_decode(MC68376State *s, int n, uint32_t *base, uint32_t *size)
{
    uint16_t bar = n < 0 ? s->csbarbt : s->csbar[n];
    uint16_t opt = n < 0 ? s->csorbt : s->csor[n];
    int byte = (opt >> 13) & 3, rw = (opt >> 11) & 3, space = (opt >> 4) & 3;

    if (cs_pin_field(s, n) < 2 || !byte || !rw || !space) {
        return false;
    }
    *size = cs_block_size[bar & 7];
    /* only the bits above the block size are compared (5.9.1.2) */
    *base = ((uint32_t)(bar & 0xfff8) << 8) & ~(*size - 1);
    return true;
}

/*
 * Map a device of @devsize bytes into a chip-select block: a smaller
 * device repeats through the block (its upper address lines are not
 * connected), up to MC68376_CS_MIRRORS copies; a larger one is cut to
 * the block size.
 */
static void cs_map(MemoryRegion *win, uint64_t devsize, bool enable,
                   uint32_t base, uint32_t size)
{
    uint64_t len = MIN(devsize, size);

    for (int i = 0; i < MC68376_CS_MIRRORS; i++) {
        bool on = enable && devsize && (uint64_t)i * len < size;
        if (on) {
            memory_region_set_size(&win[i], len);
            memory_region_set_address(&win[i], base + i * len);
        }
        memory_region_set_enabled(&win[i], on);
    }
}

static void mc68376_update_memmap(MC68376State *s)
{
    uint32_t base, size;
    bool on;

    memory_region_transaction_begin();

    memory_region_set_address(&s->modules, modules_base(s));

    /* external boot flash on CSBOOT */
    on = cs_decode(s, -1, &base, &size);
    cs_map(s->flash_win, s->flash_size, on, base, size);

    /* optional external RAM: on a chip select or at a fixed address */
    if (s->ext_ram_cs >= 0) {
        on = cs_decode(s, s->ext_ram_cs, &base, &size);
        cs_map(s->ext_ram_win, s->ext_ram_size, on, base, size);
    } else {
        cs_map(s->ext_ram_win, s->ext_ram_size, s->ext_ram_size != 0,
               s->ext_ram_base & MC68376_ADDR_MASK, s->ext_ram_size);
    }

    /* standby RAM: enabled when not in low-power stop (6.2, 6.5) */
    memory_region_set_address(&s->sram,
                              ((uint32_t)(s->sram_bar_h & 0xff) << 16) |
                              (s->sram_bar_l & 0xf000));
    memory_region_set_enabled(&s->sram, !(s->sram_mcr & RAMMCR_STOP));

    /* TPURAM: lower 3.5 KiB of the 4 KiB page in TRAMBAR (12.3) */
    memory_region_set_address(&s->tpuram, tpuram_base(s));
    memory_region_set_enabled(&s->tpuram, tpuram_enabled(s) &&
                              !(s->tpuram_mcr & TRAMMCR_STOP) &&
                              !s->tpuram_emul);

    /* MRM: 8 KiB boundary from ROMBAH/ROMBAL, off in low-power stop */
    memory_region_set_address(&s->mrm,
                              ((uint32_t)(s->mrm_bah & 0xff) << 16) |
                              (s->mrm_bal & 0xe000));
    memory_region_set_enabled(&s->mrm, !(s->mrm_mcr & MRMCR_STOP));

    memory_region_transaction_commit();
}

static MemTxResult bus_mirror_read(void *opaque, hwaddr addr, uint64_t *data,
                                   unsigned size, MemTxAttrs attrs)
{
    MC68376State *s = opaque;
    uint8_t buf[8];
    MemTxResult r;

    r = address_space_read(&s->bus_as, addr & MC68376_ADDR_MASK, attrs,
                           buf, size);
    *data = ldn_be_p(buf, size);
    return r;
}

static MemTxResult bus_mirror_write(void *opaque, hwaddr addr, uint64_t data,
                                    unsigned size, MemTxAttrs attrs)
{
    MC68376State *s = opaque;
    uint8_t buf[8];

    stn_be_p(buf, size, data);
    return address_space_write(&s->bus_as, addr & MC68376_ADDR_MASK, attrs,
                               buf, size);
}

static const MemoryRegionOps bus_mirror_ops = {
    .read_with_attrs = bus_mirror_read,
    .write_with_attrs = bus_mirror_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4,
               .unaligned = true },
    .impl = { .min_access_size = 1, .max_access_size = 4,
              .unaligned = true },
};

uint8_t *mc68376_tpuram_ptr(MC68376State *s)
{
    return memory_region_get_ram_ptr(&s->tpuram);
}

bool mc68376_tpuram_emulation(MC68376State *s)
{
    return s->tpuram_emul;
}

void mc68376_tpuram_set_emulation(MC68376State *s, bool on)
{
    if (s->tpuram_emul != on) {
        s->tpuram_emul = on;
        mc68376_update_memmap(s);
    }
}

bool mc68376_flash_load(MC68376State *s, const char *filename)
{
    g_autofree char *data = NULL;
    gsize len;

    if (!g_file_get_contents(filename, &data, &len, NULL)) {
        return false;
    }
    if (len > s->flash_size) {
        warn_report("mc68376: flash image '%s' is larger than the flash "
                    "(%u bytes), truncated", filename, s->flash_size);
        len = s->flash_size;
    }
    memcpy(memory_region_get_ram_ptr(&s->flash), data, len);
    memory_region_set_dirty(&s->flash, 0, len);
    return true;
}

/* ---------------------------------------------------------------------- */
/* Device                                                                 */
/* ---------------------------------------------------------------------- */

static void mc68376_reset_hold(Object *obj, ResetType type)
{
    MC68376State *s = MC68376_SOC(obj);

    /* SIM (appendix D.2 reset values, mode select pins as noted above) */
    s->simcr = 0x00cf;              /* SUPV = 1, MM = 1, IARB = $F */
    s->syncr = 0x3f00;
    s->rsr = s->rsr_next;
    s->rsr_next = RSR_EXT;
    s->simtr = s->simtre = 0;
    memset(s->sim_test, 0, sizeof(s->sim_test));
    s->porte = s->portf = 0;        /* undefined at reset */
    s->ddre = s->ddrf = 0;
    s->pepar = 0xff;                /* DATA8 high */
    s->pfpar = 0xff;                /* DATA9 high */
    s->sypcr = SYPCR_SWE;           /* SWP = !MODCLK = 0 */
    s->picr = 0x000f;
    s->pitr = 0x0000;               /* PTP = !MODCLK = 0 (Table 5-7) */
    s->swsr = 0;
    s->portc = 0x7f;
    s->cspar0 = 0x3fff;             /* DATA[2:0] high: chip selects */
    s->cspar1 = 0x03ff;             /* DATA[7:3] high: CS[10:6] */
    s->csbarbt = 0x0007;            /* $000000, 1 MiB */
    s->csorbt = 0x7b70;             /* both bytes, R/W, 13 waits, S/U */
    memset(s->csbar, 0, sizeof(s->csbar));
    memset(s->csor, 0, sizeof(s->csor));
    s->mm_written = s->sypcr_written = s->swsr_armed = false;

    /*
     * Port F outputs are released at reset; IRQ1-7/PF1-7 return to
     * their idle (pulled-up) high level.
     */
    for (int n = 1; n <= 7; n++) {
        if (s->pf_driven & (1 << n)) {
            ecu_pin_mcu_drive(s->pf_pin[n], 1);
        }
    }
    s->pf_driven = 0;

    timer_del(s->pit_timer);
    s->pit_modulus = 0;
    s->pit_irq.pending = false;
    pit_update_irq(s);
    for (int n = 1; n <= 7; n++) {
        s->irq_pin[n].pending = false;
        s->irq_pin_level[n] = 1;
    }
    sim_set_iarb(s);
    swt_restart(s);

    /* SRAM: low-power stop, base 0, unlocked, RASP = %11 (D.3) */
    s->sram_mcr = RAMMCR_STOP | RAMMCR_RASP;
    s->sram_bar_h = s->sram_bar_l = 0;

    /* TPURAM (D.9; STOP reset value as in D.9.1) */
    s->tpuram_mcr = TRAMMCR_RASP;
    s->tpuram_bar = 0;
    s->tpuram_bar_written = false;
    s->tpuram_emul = false;

    /*
     * MRM, generic (blank ROM) device values (D.4): BOOT = 1 (vectors
     * from CSBOOT), LOCK = 0, ASPC = %11, WAIT = %11, base $FF0000,
     * STOP = complement of DATA14.
     */
    s->mrm_mcr = 0x13c0 | (s->mrm_enabled ? 0 : MRMCR_STOP);
    s->mrm_bah = 0x00ff;
    s->mrm_bal = 0;
    memset(s->mrm_sig, 0, sizeof(s->mrm_sig));
    memset(s->mrm_bsw, 0, sizeof(s->mrm_bsw));

    mc68376_qsm_reset(s);
    mc68376_qadc_reset(s);
    mc68376_ctm4_reset(s);
    mc68376_tpu_reset(s);
    mc68376_toucan_reset(s);

    mc68376_update_memmap(s);
    irq_pins_eval(s);
}

/*
 * After all devices (and ROM blobs) have been reset: reset the CPU and
 * fetch the reset vector, SSP from $000000 and PC from $000004, through
 * the bus as it is mapped now (CSBOOT, 5.7.9).
 */
static void mc68376_reset_exit(Object *obj, ResetType type)
{
    MC68376State *s = MC68376_SOC(obj);
    CPUM68KState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->aregs[7] = address_space_ldl_be(&address_space_memory, 0,
                                         MEMTXATTRS_UNSPECIFIED, NULL);
    env->pc = address_space_ldl_be(&address_space_memory, 4,
                                   MEMTXATTRS_UNSPECIFIED, NULL);
    imb_irq_update(s);
}

static void mc68376_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    MC68376State *s = MC68376_SOC(dev);
    Object *obj = OBJECT(dev);
    MemoryRegion *sysmem = get_system_memory();
    uint8_t *p;

    if (!s->extal_hz) {
        error_setg(errp, "mc68376: extal-hz must not be zero");
        return;
    }
    if (s->ext_ram_cs > 10) {
        error_setg(errp, "mc68376: ext-ram-cs must be -1 or 0..10");
        return;
    }

    s->cpu = M68K_CPU(cpu_create(s->cpu_type));
    mc68376_install_iack(s);
    notifier_list_init(&s->clock_notifiers);

    /*
     * 24-bit bus (ADDR[23:0]), mirrored over the 4 GiB CPU address space.
     * The mirrors at 0 and at $FF000000 (reached by absolute short
     * addresses $8000-$FFFF) are real aliases; the other 254 are served
     * by a slower forwarding region, because 256 aliases make every
     * memory map change (chip selects, RAM relocation) take ~15 ms.
     * Code cannot be executed from the forwarded mirrors.
     */
    memory_region_init(&s->bus, obj, "mc68376.bus", 16 * MiB);
    address_space_init(&s->bus_as, &s->bus, "mc68376.bus");
    memory_region_init_io(&s->bus_mirror, obj, &bus_mirror_ops, s,
                          "mc68376.bus-mirror", 4 * GiB);
    /* it only forwards to the device's own regions, which do the I/O */
    s->bus_mirror.disable_reentrancy_guard = true;
    memory_region_add_subregion_overlap(sysmem, 0, &s->bus_mirror, -1);
    for (int i = 0; i < 2; i++) {
        memory_region_init_alias(&s->bus_alias[i], obj, "mc68376.bus-alias",
                                 &s->bus, 0, 16 * MiB);
        memory_region_add_subregion(sysmem, i ? 0xff000000 : 0,
                                    &s->bus_alias[i]);
    }

    /* module register block */
    memory_region_init(&s->modules, obj, "mc68376.modules",
                       MC68376_MOD_BLOCK_SIZE);
    memory_region_init_io(&s->modules_bg, obj, &modules_bg_ops, s,
                          "mc68376.unimp", MC68376_MOD_BLOCK_SIZE);
    memory_region_add_subregion_overlap(&s->modules, 0, &s->modules_bg, -1);
    memory_region_init_io(&s->sim_mr, obj, &sim_ops, s, "mc68376.sim", 0x80);
    memory_region_add_subregion(&s->modules, MC68376_SIM_OFF, &s->sim_mr);
    memory_region_init_io(&s->sram_ctl_mr, obj, &sram_ctl_ops, s,
                          "mc68376.sram-ctl", 8);
    memory_region_add_subregion(&s->modules, MC68376_SRAM_OFF,
                                &s->sram_ctl_mr);
    memory_region_init_io(&s->tpuram_ctl_mr, obj, &tpuram_ctl_ops, s,
                          "mc68376.tpuram-ctl", 0x40);
    memory_region_add_subregion(&s->modules, MC68376_TPURAM_OFF,
                                &s->tpuram_ctl_mr);
    memory_region_init_io(&s->mrm_ctl_mr, obj, &mrm_ctl_ops, s,
                          "mc68376.mrm-ctl", 0x20);
    memory_region_add_subregion(&s->modules, MC68376_MRM_OFF, &s->mrm_ctl_mr);

    /* SIM interrupt sources: PIT first (wins ties with the IRQ pins) */
    imb_irq_register(s, &s->pit_irq, "pit");
    s->pit_irq.iack = pit_iack;
    for (int n = 7; n >= 1; n--) {
        static const char *const names[8] = {
            NULL, "irq1", "irq2", "irq3", "irq4", "irq5", "irq6", "irq7"
        };
        imb_irq_register(s, &s->irq_pin[n], names[n]);
        s->irq_pin[n].level = n;
        s->irq_pin[n].vector = MC68376_VEC_AUTOVEC(n);
    }

    mc68376_qsm_init(s, &s->qsm_mr, errp);
    if (*errp) {
        return;
    }
    mc68376_qadc_init(s, &s->qadc_mr, errp);
    if (*errp) {
        return;
    }
    mc68376_ctm4_init(s, &s->ctm4_mr, errp);
    if (*errp) {
        return;
    }
    mc68376_tpu_init(s, &s->tpu_mr, errp);
    if (*errp) {
        return;
    }
    mc68376_toucan_init(s, &s->toucan_mr, errp);
    if (*errp) {
        return;
    }
    memory_region_add_subregion(&s->modules, MC68376_QSM_OFF, &s->qsm_mr);
    memory_region_add_subregion(&s->modules, MC68376_QADC_OFF, &s->qadc_mr);
    memory_region_add_subregion(&s->modules, MC68376_CTM4_OFF, &s->ctm4_mr);
    memory_region_add_subregion(&s->modules, MC68376_TPU_OFF, &s->tpu_mr);
    memory_region_add_subregion(&s->modules, MC68376_TOUCAN_OFF,
                                &s->toucan_mr);
    memory_region_add_subregion_overlap(&s->bus, 0xfff000, &s->modules,
                                        PRIO_MODULES);

    /* external boot flash (ROM: writes are ignored) */
    if (!s->flash_size) {
        error_setg(errp, "mc68376: flash-size must not be zero");
        return;
    }
    if (!memory_region_init_rom(&s->flash, obj, "mc68376.flash",
                                s->flash_size, errp)) {
        return;
    }
    memset(memory_region_get_ram_ptr(&s->flash), 0xff, s->flash_size);
    for (int i = 0; i < MC68376_CS_MIRRORS; i++) {
        memory_region_init_alias(&s->flash_win[i], obj, "mc68376.csboot",
                                 &s->flash, 0, s->flash_size);
        memory_region_set_enabled(&s->flash_win[i], false);
        memory_region_add_subregion_overlap(&s->bus, 0, &s->flash_win[i],
                                            PRIO_CS);
    }

    if (s->ext_ram_size) {
        if (!memory_region_init_ram(&s->ext_ram, obj, "mc68376.ext-ram",
                                    s->ext_ram_size, errp)) {
            return;
        }
        for (int i = 0; i < MC68376_CS_MIRRORS; i++) {
            memory_region_init_alias(&s->ext_ram_win[i], obj,
                                     "mc68376.ext-ram-cs", &s->ext_ram, 0,
                                     s->ext_ram_size);
            memory_region_set_enabled(&s->ext_ram_win[i], false);
            memory_region_add_subregion_overlap(&s->bus, 0,
                                                &s->ext_ram_win[i], PRIO_CS);
        }
    }

    /* internal arrays */
    if (!memory_region_init_ram(&s->sram, obj, "mc68376.sram",
                                MC68376_SRAM_SIZE, errp) ||
        !memory_region_init_ram(&s->tpuram, obj, "mc68376.tpuram",
                                MC68376_TPURAM_SIZE, errp) ||
        !memory_region_init_rom(&s->mrm, obj, "mc68376.mrm",
                                MC68376_MRM_SIZE, errp)) {
        return;
    }
    memory_region_set_enabled(&s->sram, false);
    memory_region_set_enabled(&s->tpuram, false);
    memory_region_add_subregion_overlap(&s->bus, 0, &s->sram, PRIO_SRAM);
    memory_region_add_subregion_overlap(&s->bus, 0, &s->tpuram, PRIO_ARRAY);
    memory_region_add_subregion_overlap(&s->bus, 0xff0000, &s->mrm,
                                        PRIO_ARRAY);

    /*
     * The masked ROM contents are factory programmed and unknown; read
     * them from "mrm-image" if given, otherwise the array reads $FF.
     */
    p = memory_region_get_ram_ptr(&s->mrm);
    memset(p, 0xff, MC68376_MRM_SIZE);
    if (s->mrm_image) {
        g_autofree char *data = NULL;
        gsize len;

        if (!g_file_get_contents(s->mrm_image, &data, &len, NULL)) {
            error_setg(errp, "mc68376: cannot read mrm-image '%s'",
                       s->mrm_image);
            return;
        }
        memcpy(p, data, MIN(len, MC68376_MRM_SIZE));
    }

    /* pins */
    for (int b = 0; b < 8; b++) {
        char name[8];
        snprintf(name, sizeof(name), "PE%d", b);
        s->pe_pin[b] = ecu_pin(name);
        snprintf(name, sizeof(name), "PF%d", b);
        s->pf_pin[b] = ecu_pin(name);
        if (b < 7) {
            snprintf(name, sizeof(name), "PC%d", b);
            s->pc_pin[b] = ecu_pin(name);
        }
    }
    for (int n = 1; n <= 7; n++) {
        char name[8];

        snprintf(name, sizeof(name), "IRQ%d", n);
        s->irq_ecu_pin[n] = ecu_pin(name);
        /* idle high (pull-up) */
        ecu_pin_external_drive(s->irq_ecu_pin[n], 1);
        ecu_pin_external_drive(s->pf_pin[n], 1);
        ecu_pin_set_input_handler(s->irq_ecu_pin[n], irq_pin_cb, s);
        ecu_pin_set_input_handler(s->pf_pin[n], irq_pin_cb, s);
        s->irq_pin_level[n] = 1;
    }

    s->pit_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pit_cb, s);
    s->swt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, swt_cb, s);
    s->rsr_next = RSR_POW;
}

static const Property mc68376_props[] = {
    DEFINE_PROP_STRING("cpu-type", MC68376State, cpu_type),
    /* PLL reference; 4.194 MHz is the typical crystal (5.3.1) */
    DEFINE_PROP_UINT32("extal-hz", MC68376State, extal_hz, 4194304),
    DEFINE_PROP_UINT32("flash-size", MC68376State, flash_size, 512 * KiB),
    DEFINE_PROP_UINT32("ext-ram-size", MC68376State, ext_ram_size, 0),
    DEFINE_PROP_UINT32("ext-ram-base", MC68376State, ext_ram_base, 0x100000),
    DEFINE_PROP_INT32("ext-ram-cs", MC68376State, ext_ram_cs, -1),
    DEFINE_PROP_UINT32("vrh-mv", MC68376State, vrh_mv, 5000),
    DEFINE_PROP_BOOL("wdt-reset", MC68376State, wdt_reset, true),
    DEFINE_PROP_BOOL("kline-echo", MC68376State, kline_echo, false),
    DEFINE_PROP_BOOL("mrm-enabled", MC68376State, mrm_enabled, true),
    DEFINE_PROP_STRING("mrm-image", MC68376State, mrm_image),
    DEFINE_PROP_CHR("sci", MC68376State, sci_chr),
    DEFINE_PROP_LINK("canbus0", MC68376State, canbus, TYPE_CAN_BUS,
                     CanBusState *),
};

static void mc68376_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = mc68376_realize;
    dc->user_creatable = false;
    rc->phases.hold = mc68376_reset_hold;
    rc->phases.exit = mc68376_reset_exit;
    device_class_set_props(dc, mc68376_props);
}

static const TypeInfo mc68376_info = {
    .name = TYPE_MC68376_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MC68376State),
    .class_init = mc68376_class_init,
};

static void mc68376_register_types(void)
{
    type_register_static(&mc68376_info);
}

type_init(mc68376_register_types)
