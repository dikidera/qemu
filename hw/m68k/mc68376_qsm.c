/*
 * MC68376 queued serial module (QSM): SCI (asynchronous serial port),
 * QSPI (queued synchronous serial interface) and port QS.
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D), section 9 and
 * appendix D.6.  Section and table numbers below refer to that manual.
 *
 * The SCI is connected to the SoC's "sci" chardev.  Bytes are paced at
 * the programmed baud rate; received bytes are held back (flow control)
 * while RDR is full, so the overrun flag never sets.  Line conditions a
 * byte stream cannot carry (noise, framing errors, break, the ninth data
 * bit) are not modelled.
 *
 * The QSPI executes its command queue in master mode with the programmed
 * timing; the data shifted in comes from a device attached with
 * mc68376_qspi_attach() or, with nothing connected, reads as all ones
 * (MISO floating high).  Slave mode needs an external master and does
 * nothing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "hw/m68k/mc68376.h"

/* QSMCR (D.6.1) */
#define QSMCR_WMASK     0xe08f      /* STOP FRZ1 FRZ0 SUPV IARB */

/* SCCR1 (D.6.6) */
#define SCCR1_LOOPS     0x4000
#define SCCR1_PT        0x0800
#define SCCR1_PE        0x0400
#define SCCR1_M         0x0200
#define SCCR1_WAKE      0x0100
#define SCCR1_TIE       0x0080
#define SCCR1_TCIE      0x0040
#define SCCR1_RIE       0x0020
#define SCCR1_ILIE      0x0010
#define SCCR1_TE        0x0008
#define SCCR1_RE        0x0004
#define SCCR1_RWU       0x0002
#define SCCR1_SBK       0x0001

/* SCSR (D.6.7) */
#define SCSR_TDRE       0x0100
#define SCSR_TC         0x0080
#define SCSR_RDRF       0x0040
#define SCSR_RAF        0x0020
#define SCSR_IDLE       0x0010
#define SCSR_OR         0x0008
#define SCSR_NF         0x0004
#define SCSR_FE         0x0002
#define SCSR_PF         0x0001
#define SCSR_RXFLAGS    (SCSR_RDRF | SCSR_IDLE | SCSR_OR | SCSR_NF | \
                         SCSR_FE | SCSR_PF)

/* SPCR0..3, SPSR (D.6.11 - D.6.15) */
#define SPCR0_MSTR      0x8000
#define SPCR1_SPE       0x8000
#define SPCR2_SPIFIE    0x8000
#define SPCR2_WREN      0x4000
#define SPCR2_WRTO      0x2000
#define SPCR3_LOOPQ     0x04
#define SPCR3_HMIE      0x02
#define SPCR3_HALT      0x01
#define SPSR_SPIF       0x80
#define SPSR_MODF       0x40
#define SPSR_HALTA      0x20

/* command RAM byte (D.6.18) */
#define CR_CONT         0x80
#define CR_BITSE        0x40
#define CR_DT           0x20
#define CR_DSCK         0x10

static inline uint16_t merge16(uint16_t old, uint16_t val, uint16_t mask)
{
    return (old & ~mask) | (val & mask);
}

#define QSPI_RAM_OFF    0x100       /* $YFFD00 */
#define QSPI_RAM_SIZE   0x50
#define QSPI_RR         0x00
#define QSPI_TR         0x20
#define QSPI_CR         0x40

struct MC68376QSM {
    MC68376State *soc;

    uint16_t qsmcr, qtest;
    uint8_t qilr, qivr;
    IMBIrq qspi_irq, sci_irq;

    /* SCI */
    uint16_t sccr0, sccr1, scsr, rdr, tdr;
    uint16_t scsr_armed;        /* flags seen set by the last SCSR read */
    bool tx_busy, tx_preamble, preamble_pending, rx_active, idle_armed;
    QEMUTimer *tx_timer, *rx_timer, *idle_timer;
    uint8_t rx_fifo[256];
    int rx_head, rx_count;

    /* port QS */
    uint8_t portqs, pqspar, ddrqs;
    EcuPin *pin[8];

    /* QSPI */
    uint16_t spcr0, spcr1, spcr2;
    uint8_t spcr3, spsr, spsr_armed;
    uint8_t ram[QSPI_RAM_SIZE];
    int qp;                     /* working queue pointer */
    bool qspi_running, newqp_pending;
    QEMUTimer *qspi_timer;
    MC68376SPIXferFn spi_fn;
    void *spi_opaque;
};

/* ---------------------------------------------------------------------- */
/* Interrupts (9.2.1.3, D.6.3, D.6.4)                                     */
/* ---------------------------------------------------------------------- */

static void qsm_update_irq(MC68376QSM *q)
{
    bool sci, spi;
    uint16_t f = q->scsr, c = q->sccr1;

    /* while RWU is set the receiver status flags/interrupts are off */
    sci = ((c & SCCR1_TIE) && (f & SCSR_TDRE)) ||
          ((c & SCCR1_TCIE) && (f & SCSR_TC)) ||
          (!(c & SCCR1_RWU) && (c & SCCR1_RIE) &&
           (f & (SCSR_RDRF | SCSR_OR))) ||
          (!(c & SCCR1_RWU) && (c & SCCR1_ILIE) && (f & SCSR_IDLE));
    spi = ((q->spcr2 & SPCR2_SPIFIE) && (q->spsr & SPSR_SPIF)) ||
          ((q->spcr3 & SPCR3_HMIE) && (q->spsr & (SPSR_HALTA | SPSR_MODF)));

    q->sci_irq.level = q->qilr & 7;
    q->qspi_irq.level = (q->qilr >> 3) & 7;
    q->sci_irq.vector = q->qivr & 0xfe;     /* INTV0 = 0 for the SCI */
    q->qspi_irq.vector = q->qivr | 0x01;    /* INTV0 = 1 for the QSPI */
    q->sci_irq.iarb = q->qspi_irq.iarb = q->qsmcr & 0xf;
    q->sci_irq.pending = sci;
    q->qspi_irq.pending = spi;
    imb_irq_update(q->soc);
}

/* ---------------------------------------------------------------------- */
/* Port QS (9.2.2, D.6.9, D.6.10)                                         */
/* ---------------------------------------------------------------------- */

static bool qspi_enabled(MC68376QSM *q)
{
    return q->spcr1 & SPCR1_SPE;
}

/* is pin @b used by the QSPI/SCI rather than general purpose I/O? */
static bool pqs_dedicated(MC68376QSM *q, int b)
{
    if (b == 7) {
        return q->sccr1 & SCCR1_TE;         /* TXD */
    }
    if (b == 2) {
        return qspi_enabled(q);             /* SCK */
    }
    return qspi_enabled(q) && (q->pqspar & (1 << b));
}

static void pqs_drive(MC68376QSM *q)
{
    for (int b = 0; b < 8; b++) {
        if (b == 7 && (q->sccr1 & SCCR1_TE)) {
            /* TXD idles at mark while the transmitter owns it */
            ecu_pin_mcu_drive(q->pin[7], 1);
        } else if (q->ddrqs & (1 << b)) {
            /*
             * General purpose outputs, and QSPI outputs between
             * transfers, are driven from PORTQS (9.3.5.1).
             */
            ecu_pin_mcu_drive(q->pin[b], (q->portqs >> b) & 1);
        }
    }
}

static uint8_t pqs_read(MC68376QSM *q)
{
    uint8_t v = 0;

    for (int b = 0; b < 8; b++) {
        if (q->ddrqs & (1 << b) && !pqs_dedicated(q, b)) {
            v |= q->portqs & (1 << b);
        } else {
            v |= ecu_pin_level(q->pin[b]) << b;
        }
    }
    return v;
}

/* ---------------------------------------------------------------------- */
/* SCI (9.4)                                                              */
/* ---------------------------------------------------------------------- */

/*
 * Frame time: 10 or 11 bits (M) at fsys / (32 x SCBR) baud (9.4.3.3).
 * SCBR = 0 disables the baud rate generator: returns 0.
 */
static int64_t sci_frame_ns(MC68376QSM *q)
{
    uint32_t scbr = q->sccr0 & 0x1fff;
    uint32_t fsys = mc68376_sysclk_hz(q->soc);
    int bits = (q->sccr1 & SCCR1_M) ? 11 : 10;

    if (!scbr || !fsys) {
        return 0;
    }
    return muldiv64((uint64_t)bits * 32 * scbr, NANOSECONDS_PER_SECOND,
                    fsys);
}

static bool parity_odd(uint8_t v)
{
    return __builtin_parity(v);
}

static void sci_rx_schedule(MC68376QSM *q)
{
    int64_t t = sci_frame_ns(q);

    if (!q->rx_count || timer_pending(q->rx_timer) || !t) {
        return;
    }
    q->rx_active = q->sccr1 & SCCR1_RE;
    timer_mod(q->rx_timer, mc68376_now() + t);
}

static void sci_rx_push(MC68376QSM *q, uint8_t b)
{
    if (q->rx_count < (int)sizeof(q->rx_fifo)) {
        q->rx_fifo[(q->rx_head + q->rx_count) % sizeof(q->rx_fifo)] = b;
        q->rx_count++;
    }
    sci_rx_schedule(q);
}

/*
 * The byte on the wire for TDR: with parity enabled the MSB of the frame
 * data is the parity bit (Table 9-6).  With M = 1 the ninth bit (T8 or
 * the parity bit) cannot be represented on the chardev and is dropped.
 */
static uint8_t sci_tx_wire_byte(MC68376QSM *q, uint16_t d)
{
    uint8_t b = d & 0xff;

    if ((q->sccr1 & (SCCR1_PE | SCCR1_M)) == SCCR1_PE) {
        /* PT = 0: even parity (even number of ones), PT = 1: odd */
        bool odd = q->sccr1 & SCCR1_PT;
        b &= 0x7f;
        if (parity_odd(b) != odd) {
            b |= 0x80;
        }
    }
    return b;
}

static void sci_tx_kick(MC68376QSM *q);

static void sci_tx_start_preamble(MC68376QSM *q)
{
    int64_t t = sci_frame_ns(q);

    q->preamble_pending = false;
    if (!t) {
        return;
    }
    q->tx_busy = true;
    q->tx_preamble = true;
    q->scsr &= ~SCSR_TC;
    timer_mod(q->tx_timer, mc68376_now() + t);
}

/* move TDR to the shifter if possible */
static void sci_tx_kick(MC68376QSM *q)
{
    MC68376State *s = q->soc;
    int64_t t = sci_frame_ns(q);
    uint8_t b;

    if (q->tx_busy || !(q->sccr1 & SCCR1_TE) || !t) {
        return;
    }
    if (q->preamble_pending) {
        sci_tx_start_preamble(q);
        return;
    }
    if (q->scsr & SCSR_TDRE) {
        q->scsr |= SCSR_TC;             /* nothing left to send */
        return;
    }
    b = sci_tx_wire_byte(q, q->tdr);
    q->scsr |= SCSR_TDRE;
    q->scsr &= ~SCSR_TC;
    q->tx_busy = true;
    q->tx_preamble = false;
    if (q->sccr1 & SCCR1_LOOPS) {
        /* internal loop: TXD stays idle (9.4.3.9) */
        if (q->sccr1 & SCCR1_RE) {
            sci_rx_push(q, b);
        }
    } else {
        qemu_chr_fe_write_all(&s->sci_chr, &b, 1);
        if (s->kline_echo && (q->sccr1 & SCCR1_RE)) {
            sci_rx_push(q, b);
        }
    }
    timer_mod(q->tx_timer, mc68376_now() + t);
}

static void sci_tx_timer_cb(void *opaque)
{
    MC68376QSM *q = opaque;

    q->tx_busy = false;
    q->tx_preamble = false;
    if (q->sccr1 & SCCR1_TE) {
        sci_tx_kick(q);
    } else {
        /* transmitter disabled once pending frames are out */
        q->scsr |= SCSR_TC;
        pqs_drive(q);
    }
    qsm_update_irq(q);
}

static void sci_rx_timer_cb(void *opaque)
{
    MC68376QSM *q = opaque;
    MC68376State *s = q->soc;
    uint8_t b;

    q->rx_active = false;
    if (!q->rx_count) {
        return;
    }
    if (!(q->sccr1 & SCCR1_RE)) {
        /* receiver disabled: data on the line is lost */
        q->rx_head = (q->rx_head + 1) % sizeof(q->rx_fifo);
        q->rx_count--;
        sci_rx_schedule(q);
        qemu_chr_fe_accept_input(&s->sci_chr);
        return;
    }
    if (q->scsr & SCSR_RDRF) {
        /* held back until the firmware reads RDR (see header comment) */
        return;
    }
    b = q->rx_fifo[q->rx_head];
    q->rx_head = (q->rx_head + 1) % sizeof(q->rx_fifo);
    q->rx_count--;

    if (q->sccr1 & SCCR1_RWU) {
        /*
         * Wake-up mode (9.4.3.8): with address-mark wake-up a frame with
         * its MSB set wakes the receiver and is received normally;
         * otherwise frames are ignored until woken (idle-line wake-up is
         * handled by the idle timer).
         */
        if ((q->sccr1 & SCCR1_WAKE) && (b & 0x80)) {
            q->sccr1 &= ~SCCR1_RWU;
        } else {
            goto next;
        }
    }
    q->rdr = b;
    q->scsr |= SCSR_RDRF;
    if ((q->sccr1 & (SCCR1_PE | SCCR1_M)) == SCCR1_PE) {
        bool odd = q->sccr1 & SCCR1_PT;
        if (parity_odd(b) != odd) {
            q->scsr |= SCSR_PF;
        }
    }
    q->idle_armed = true;
next:
    if (q->rx_count) {
        sci_rx_schedule(q);
    } else {
        int64_t t = sci_frame_ns(q);
        if (t) {
            timer_mod(q->idle_timer, mc68376_now() + t);
        }
    }
    qsm_update_irq(q);
    qemu_chr_fe_accept_input(&s->sci_chr);
}

/*
 * Idle line: one frame time of mark after the last frame.  IDLE is set
 * again only after a new frame has been received (9.4.3.7).  Short and
 * long idle-line detection (ILT) are not distinguished.
 */
static void sci_idle_timer_cb(void *opaque)
{
    MC68376QSM *q = opaque;

    if (q->rx_count || !(q->sccr1 & SCCR1_RE)) {
        return;
    }
    if ((q->sccr1 & SCCR1_RWU) && !(q->sccr1 & SCCR1_WAKE)) {
        q->sccr1 &= ~SCCR1_RWU;          /* idle-line wake-up */
        return;
    }
    if (q->idle_armed) {
        q->idle_armed = false;
        q->scsr |= SCSR_IDLE;
        qsm_update_irq(q);
    }
}

static int sci_can_receive(void *opaque)
{
    MC68376QSM *q = opaque;

    return sizeof(q->rx_fifo) - q->rx_count;
}

static void sci_receive(void *opaque, const uint8_t *buf, int size)
{
    MC68376QSM *q = opaque;

    for (int i = 0; i < size; i++) {
        sci_rx_push(q, buf[i]);
    }
}

static void sci_write_sccr1(MC68376QSM *q, uint16_t v)
{
    uint16_t old = q->sccr1;

    q->sccr1 = v & 0x7fff;
    if ((v & SCCR1_TE) && !(old & SCCR1_TE)) {
        /*
         * Enabling the transmitter queues an idle preamble; TC is set
         * at its end (9.4.3.5).
         */
        q->preamble_pending = true;
        sci_tx_kick(q);
    }
    if ((v & SCCR1_RE) && !(old & SCCR1_RE)) {
        sci_rx_schedule(q);
        qemu_chr_fe_accept_input(&q->soc->sci_chr);
    }
    if (v & SCCR1_SBK) {
        qemu_log_mask(LOG_UNIMP, "mc68376.qsm: SCI break not modelled\n");
    }
    pqs_drive(q);
}

static uint16_t sci_read_scsr(MC68376QSM *q)
{
    uint16_t v = q->scsr | (q->rx_active ? SCSR_RAF : 0);

    /* reading either byte arms clearing of all flags set (D.6.7) */
    q->scsr_armed = v & ~SCSR_RAF;
    return v;
}

static uint16_t sci_read_scdr(MC68376QSM *q)
{
    uint16_t v = q->rdr;

    q->scsr &= ~(q->scsr_armed & SCSR_RXFLAGS);
    q->scsr_armed &= ~SCSR_RXFLAGS;
    sci_rx_schedule(q);
    qsm_update_irq(q);
    return v;
}

static void sci_write_scdr(MC68376QSM *q, uint16_t v)
{
    q->tdr = v & 0x1ff;
    if (q->scsr & SCSR_TDRE) {
        if (!(q->scsr_armed & SCSR_TDRE)) {
            /*
             * TDRE must be cleared (read SCSR with TDRE set, then write
             * SCDR) or the data is not transmitted (9.4.3.5).
             */
            qemu_log_mask(LOG_GUEST_ERROR, "mc68376.qsm: SCDR written "
                          "without reading SCSR first, not transmitted\n");
            return;
        }
        q->scsr &= ~SCSR_TDRE;
    }
    /* TC is cleared by reading SCSR with TC set, then writing SCDR */
    q->scsr &= ~(q->scsr_armed & SCSR_TC);
    q->scsr_armed &= ~(SCSR_TDRE | SCSR_TC);
    sci_tx_kick(q);
    qsm_update_irq(q);
}

/* ---------------------------------------------------------------------- */
/* QSPI (9.3)                                                             */
/* ---------------------------------------------------------------------- */

void mc68376_qspi_attach(MC68376State *s, MC68376SPIXferFn fn, void *opaque)
{
    s->qsm->spi_fn = fn;
    s->qsm->spi_opaque = opaque;
}

static int qspi_bits(MC68376QSM *q, uint8_t cmd)
{
    int bits = (q->spcr0 >> 10) & 0xf;

    if (!(cmd & CR_BITSE)) {
        return 8;
    }
    if (bits == 0) {
        return 16;
    }
    return bits < 8 ? 8 : bits;     /* reserved values: eight bits */
}

/*
 * Duration of one queue entry: PCS-to-SCK delay, the bits at
 * fsys / (2 x SPBR), then the delay after transfer (D.6.11, D.6.12).
 * Returns 0 when the baud rate generator is off (SPBR < 2: no transfers).
 */
static int64_t qspi_entry_ns(MC68376QSM *q, uint8_t cmd, bool first)
{
    uint64_t fsys = mc68376_sysclk_hz(q->soc);
    uint32_t spbr = q->spcr0 & 0xff;
    uint32_t dsckl = (q->spcr1 >> 8) & 0x7f;
    uint32_t dtl = q->spcr1 & 0xff;
    uint64_t clocks;

    if (spbr < 2 || !fsys) {
        return 0;
    }
    clocks = (uint64_t)qspi_bits(q, cmd) * 2 * spbr;
    /* DSCKL = 0 is outside the documented 1..127 range; taken as 128 */
    clocks += (cmd & CR_DSCK) ? (dsckl ? dsckl : 128) : spbr;
    if (!first) {
        /* delay after the previous transfer */
        clocks += (cmd & CR_DT) ? (dtl ? 32 * dtl : 8192) : 17;
    }
    return muldiv64(clocks, NANOSECONDS_PER_SECOND, fsys);
}

static void qspi_schedule(MC68376QSM *q, bool first)
{
    int64_t t = qspi_entry_ns(q, q->ram[QSPI_CR + q->qp], first);

    if (!t) {
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.qsm: QSPI enabled with "
                      "SPBR < 2, no transfers\n");
        return;
    }
    timer_mod(q->qspi_timer, mc68376_now() + t);
}

static void qspi_stop(MC68376QSM *q)
{
    q->qspi_running = false;
    timer_del(q->qspi_timer);
    pqs_drive(q);
}

static void qspi_start(MC68376QSM *q)
{
    if (!(q->spcr0 & SPCR0_MSTR)) {
        qemu_log_mask(LOG_UNIMP, "mc68376.qsm: QSPI slave mode needs an "
                      "external master, not modelled\n");
        return;
    }
    q->qp = q->spcr2 & 0xf;
    q->newqp_pending = false;
    q->qspi_running = true;
    q->spsr &= ~SPSR_HALTA;
    qspi_schedule(q, true);
}

static void qspi_drive_pcs(MC68376QSM *q, uint8_t pcs)
{
    for (int i = 0; i < 4; i++) {
        int b = 3 + i;
        if ((q->pqspar & (1 << b)) && (q->ddrqs & (1 << b))) {
            ecu_pin_mcu_drive(q->pin[b], (pcs >> i) & 1);
        }
    }
}

static void qspi_timer_cb(void *opaque)
{
    MC68376QSM *q = opaque;
    uint8_t cmd = q->ram[QSPI_CR + q->qp];
    int bits = qspi_bits(q, cmd);
    uint16_t mask = (1u << bits) - 1;
    uint16_t tx = ((q->ram[QSPI_TR + 2 * q->qp] << 8) |
                   q->ram[QSPI_TR + 2 * q->qp + 1]) & mask;
    uint16_t rx;
    uint8_t endqp = (q->spcr2 >> 8) & 0xf;

    if (!q->qspi_running) {
        return;
    }
    qspi_drive_pcs(q, cmd & 0xf);
    if (q->spcr3 & SPCR3_LOOPQ) {
        rx = tx;                        /* feedback path */
    } else if (q->spi_fn) {
        rx = q->spi_fn(q->spi_opaque, cmd & 0xf, tx, bits);
    } else {
        rx = 0xffff;                    /* nothing connected: MISO high */
    }
    rx &= mask;                         /* unused bits read zero */
    q->ram[QSPI_RR + 2 * q->qp] = rx >> 8;
    q->ram[QSPI_RR + 2 * q->qp + 1] = rx;
    if (!(cmd & CR_CONT)) {
        pqs_drive(q);                   /* PORTQS between transfers */
    }

    /* CPTQP <- working pointer (9.3.4) */
    q->spsr = (q->spsr & 0xf0) | q->qp;
    if (q->qp == endqp) {
        q->spsr |= SPSR_SPIF;
        if (q->spcr2 & SPCR2_WREN) {
            q->qp = (q->spcr2 & SPCR2_WRTO) ? (q->spcr2 & 0xf) : 0;
        } else {
            /* end of queue: clear SPE and stop */
            q->spcr1 &= ~SPCR1_SPE;
            qspi_stop(q);
            qsm_update_irq(q);
            return;
        }
    } else {
        q->qp = (q->qp + 1) & 0xf;
    }
    if (q->newqp_pending) {
        q->newqp_pending = false;
        q->qp = q->spcr2 & 0xf;
    }
    if (q->spcr3 & SPCR3_HALT) {
        /* halt on the transfer boundary (D.6.14) */
        q->spsr |= SPSR_HALTA;
        q->qspi_running = false;
        pqs_drive(q);
    } else {
        qspi_schedule(q, false);
    }
    qsm_update_irq(q);
}

/* SS low while the QSPI is master: mode fault (9.3.5) */
static void pqs3_cb(void *opaque, int level)
{
    MC68376QSM *q = opaque;

    if (!level && qspi_enabled(q) && (q->spcr0 & SPCR0_MSTR) &&
        (q->pqspar & 0x08) && !(q->ddrqs & 0x08)) {
        q->spsr |= SPSR_MODF;
        qsm_update_irq(q);
    }
}

/* ---------------------------------------------------------------------- */
/* Register access ($YFFC00, Table D-31)                                  */
/* ---------------------------------------------------------------------- */

static uint16_t qsm_read16(MC68376QSM *q, hwaddr off)
{
    switch (off) {
    case 0x00: return q->qsmcr;
    case 0x02: return q->qtest;
    case 0x04: return (q->qilr << 8) | q->qivr | 0x01;  /* INTV0 reads 1 */
    case 0x08: return q->sccr0;
    case 0x0a: return q->sccr1;
    case 0x0c: return sci_read_scsr(q);
    case 0x0e: return sci_read_scdr(q);
    case 0x14: return pqs_read(q);
    case 0x16: return (q->pqspar << 8) | q->ddrqs;
    case 0x18: return q->spcr0;
    case 0x1a: return q->spcr1;
    case 0x1c: return q->spcr2;
    case 0x1e:
        q->spsr_armed = q->spsr & 0xe0;
        return (q->spcr3 << 8) | q->spsr;
    }
    return 0;                       /* not used */
}

static void qsm_write16(MC68376QSM *q, hwaddr off, uint16_t val,
                        uint16_t mask)
{
    uint16_t old;

    switch (off) {
    case 0x00:
        q->qsmcr = merge16(q->qsmcr, val, mask & QSMCR_WMASK);
        if (q->qsmcr & 0x8000) {
            qemu_log_mask(LOG_UNIMP, "mc68376.qsm: STOP not modelled\n");
        }
        break;
    case 0x02:
        q->qtest = merge16(q->qtest, val, mask);
        break;
    case 0x04:
        if (mask & 0xff00) {
            q->qilr = (val >> 8) & 0x3f;
        }
        if (mask & 0x00ff) {
            q->qivr = val & 0xfe;   /* writes to INTV0 have no effect */
        }
        break;
    case 0x08:
        q->sccr0 = merge16(q->sccr0, val, mask & 0x1fff);
        break;
    case 0x0a:
        sci_write_sccr1(q, merge16(q->sccr1, val, mask));
        break;
    case 0x0c:
        break;                      /* SCSR: cleared by the sequences */
    case 0x0e:
        sci_write_scdr(q, merge16(q->tdr, val, mask));
        break;
    case 0x14:
        q->portqs = merge16(q->portqs, val, mask & 0xff);
        pqs_drive(q);
        break;
    case 0x16:
        if (mask & 0xff00) {
            q->pqspar = (val >> 8) & 0x7b;
        }
        if (mask & 0x00ff) {
            q->ddrqs = val & 0xff;
        }
        pqs_drive(q);
        break;
    case 0x18:
        q->spcr0 = merge16(q->spcr0, val, mask);
        break;
    case 0x1a:
        old = q->spcr1;
        q->spcr1 = merge16(q->spcr1, val, mask);
        if ((q->spcr1 & SPCR1_SPE) && !(old & SPCR1_SPE)) {
            qspi_start(q);
        } else if (!(q->spcr1 & SPCR1_SPE) && (old & SPCR1_SPE)) {
            qspi_stop(q);
        }
        break;
    case 0x1c:
        /*
         * SPCR2 is buffered: a new NEWQP takes effect after the current
         * transfer (D.6.13).  The other fields are used as read.
         */
        q->spcr2 = merge16(q->spcr2, val, mask & 0xef0f);
        if ((mask & 0x000f) && q->qspi_running) {
            q->newqp_pending = true;
        }
        break;
    case 0x1e:
        if (mask & 0xff00) {
            uint8_t o3 = q->spcr3;
            q->spcr3 = (val >> 8) & 0x07;
            if ((o3 & SPCR3_HALT) && !(q->spcr3 & SPCR3_HALT) &&
                qspi_enabled(q) && !q->qspi_running &&
                (q->spsr & SPSR_HALTA)) {
                /* restart after a halt */
                q->spsr &= ~SPSR_HALTA;
                q->qspi_running = true;
                qspi_schedule(q, true);
            }
        }
        if (mask & 0x00ff) {
            /* flags read as one, then written zero, are cleared */
            uint8_t clr = q->spsr_armed & ~val & 0xe0;
            q->spsr &= ~clr;
            q->spsr_armed &= ~clr;
        }
        break;
    case 0x06: case 0x10: case 0x12:
        break;                      /* not used */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.qsm: write to unused "
                      "register $YFFC%02x\n", (unsigned)off);
        return;
    }
    qsm_update_irq(q);
}

static uint64_t qsm_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MC68376QSM *q = opaque;
    uint16_t v;

    if (addr >= QSPI_RAM_OFF) {
        hwaddr o = addr - QSPI_RAM_OFF;
        if (o >= QSPI_RAM_SIZE) {
            return 0;
        }
        if (size == 2) {
            return (q->ram[o] << 8) | (o + 1 < QSPI_RAM_SIZE ?
                                       q->ram[o + 1] : 0);
        }
        return q->ram[o];
    }
    v = qsm_read16(q, addr & ~1);
    if (size == 2) {
        return v;
    }
    return addr & 1 ? v & 0xff : v >> 8;
}

static void qsm_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    MC68376QSM *q = opaque;

    if (addr >= QSPI_RAM_OFF) {
        hwaddr o = addr - QSPI_RAM_OFF;
        if (o >= QSPI_RAM_SIZE) {
            return;
        }
        if (size == 2) {
            q->ram[o] = val >> 8;
            if (o + 1 < QSPI_RAM_SIZE) {
                q->ram[o + 1] = val;
            }
        } else {
            q->ram[o] = val;
        }
        return;
    }
    if (size == 2) {
        qsm_write16(q, addr, val, 0xffff);
    } else if (addr & 1) {
        qsm_write16(q, addr & ~1, val & 0xff, 0x00ff);
    } else {
        qsm_write16(q, addr, (val & 0xff) << 8, 0xff00);
    }
}

static const MemoryRegionOps qsm_ops = {
    .read = qsm_mmio_read,
    .write = qsm_mmio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

void mc68376_qsm_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    MC68376QSM *q = g_new0(MC68376QSM, 1);

    q->soc = s;
    s->qsm = q;
    memory_region_init_io(mr, OBJECT(s), &qsm_ops, q, "mc68376.qsm",
                          MC68376_QSM_SIZE);

    /* the QSPI wins over the SCI on equal levels (9.2.1.3) */
    imb_irq_register(s, &q->qspi_irq, "qspi");
    imb_irq_register(s, &q->sci_irq, "sci");

    for (int b = 0; b < 8; b++) {
        char name[8];
        snprintf(name, sizeof(name), "PQS%d", b);
        q->pin[b] = ecu_pin(name);
    }
    ecu_pin_set_input_handler(q->pin[3], pqs3_cb, q);

    q->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sci_tx_timer_cb, q);
    q->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sci_rx_timer_cb, q);
    q->idle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sci_idle_timer_cb, q);
    q->qspi_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, qspi_timer_cb, q);
    qemu_chr_fe_set_handlers(&s->sci_chr, sci_can_receive, sci_receive,
                             NULL, NULL, q, NULL, true);
}

void mc68376_qsm_reset(MC68376State *s)
{
    MC68376QSM *q = s->qsm;

    /* reset values from D.6 */
    q->qsmcr = 0x0080;
    q->qtest = 0;
    q->qilr = 0;
    q->qivr = 0x0f;
    q->sccr0 = 0x0004;
    q->sccr1 = 0;
    q->scsr = SCSR_TDRE | SCSR_TC;
    q->scsr_armed = 0;
    q->rdr = q->tdr = 0;
    q->tx_busy = q->tx_preamble = q->preamble_pending = false;
    q->rx_active = q->idle_armed = false;
    q->rx_count = 0;
    timer_del(q->tx_timer);
    timer_del(q->rx_timer);
    timer_del(q->idle_timer);

    q->portqs = q->pqspar = q->ddrqs = 0;
    q->spcr0 = 0x0104;
    q->spcr1 = 0x0404;
    q->spcr2 = 0;
    q->spcr3 = 0;
    q->spsr = q->spsr_armed = 0;
    q->qp = 0;
    q->qspi_running = q->newqp_pending = false;
    timer_del(q->qspi_timer);
    qsm_update_irq(q);
}
