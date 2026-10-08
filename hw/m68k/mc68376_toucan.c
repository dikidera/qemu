/*
 * MC68376 TouCAN: CAN 2.0B controller module (section 13, appendix D.10).
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D).  Section numbers
 * in the comments refer to it.
 *
 * Model summary
 * -------------
 * - 16 message buffers at offset $80 of the module window ($YFF100),
 *   16 bytes each (13.4.1, Figures 13-3/13-4): control/status (time
 *   stamp high byte, CODE, LENGTH), ID_HIGH, ID_LOW, 8 data bytes and a
 *   reserved word.
 * - Receive: every error-free frame is matched against the active
 *   receive buffers (codes EMPTY/FULL/OVERRUN) with RXGMSK (buffers
 *   0-13), RX14MSK, RX15MSK (13.4.2, D.10.9-11).  The frame goes to the
 *   lowest matching EMPTY buffer; if no EMPTY buffer matches, to the
 *   lowest matching FULL/OVERRUN buffer, which becomes OVERRUN (Table
 *   13-2).  Frames the TouCAN transmitted itself are only received into
 *   an EMPTY buffer (13.5.3.2).  Remote frames are never stored; they
 *   turn transmit buffers with code %1010 and the exact same ID into
 *   %1110 (13.5.5, Table 13-3).
 * - Lock/release (13.4.1.6, 13.5.4.2): reading a buffer's control/status
 *   word locks it (and releases the previously locked one), reading TIMER
 *   releases it.  A frame for a locked buffer waits in the serial message
 *   buffer (the last one wins) and is moved in on release.  The BUSY code
 *   is never shown: transfers are instantaneous.
 * - Transmit (13.5.3): buffers with code %1100 or %1110 take part in the
 *   internal arbitration, lowest ID first (CAN arbitration order,
 *   standard before extended with the same base ID) or lowest buffer
 *   first with CANCTRL1.LBUF (D.10.5).  A frame takes its full length at
 *   the programmed bit rate (no stuff bits): 47 + 8n bits standard,
 *   67 + 8n bits extended, including the 3-bit intermission.  The
 *   transmission is always acknowledged: there is no error modelling, so
 *   ESTAT shows error active and the error bits never set.  The time
 *   stamp (TIMER at the start of the ID field) goes to the buffer on
 *   completion, IFLAG is set and the code updated per Table 13-3.
 * - Bit timing (13.4.3, D.10.5-7): S-clock = fsys / (PRESDIV + 1), bit
 *   time = (1 + PROPSEG+1 + PSEG1+1 + PSEG2+1) S-clocks.  TIMER counts
 *   at the bit rate while the prescaler runs (not in debug mode or
 *   low-power stop); TSYNC resets it on a reception into buffer 0.
 * - CANMCR (D.10.1): out of reset the module is in debug mode (HALT, FRZ,
 *   FRZACK, NOTRDY set, 13.5.1).  HALT with FRZ = 1 enters debug mode
 *   (FRZACK, NOTRDY) after the frame in progress; leaving it, NOTRDY
 *   clears after 11 recessive bit times (13.6.1).  STOP enters low-power
 *   stop (STOPACK, NOTRDY; only CANMCR is accessible); a frame on the
 *   bus then sets WAKEINT, and with SELFWAKE clears STOP and is received
 *   (13.6.2).  SOFTRST resets CANMCR, CANICR, CANTCR, IMASK, IFLAG,
 *   ESTAT, the error counters and the timer (not the bit timing, masks
 *   or buffers).  APS is stored only.  The IMB FREEZE line (background
 *   debug mode) is not modelled.
 * - Loop back (CANCTRL1.LOOP): frames are not sent to the CAN bus, frames
 *   from the bus are ignored, own frames are received.  Without LOOP,
 *   frames go to the QEMU CAN bus (-machine canbus0=...) and are also
 *   self-received.
 * - Interrupts (13.7): one request at CANICR.ILCAN with CANMCR.IARB; the
 *   vector is IVBA:source, source 0-15 = buffers (IFLAG & IMASK), 16 =
 *   bus off, 17 = error, 18 = wake-up, lower number wins (Table 13-9).
 *   Until CANICR is written after reset the vector is $0F (D.10.3).
 * - Not modelled: user/supervisor restriction (SUPV), CANTCR, pin
 *   polarity (RXMODE/TXMODE), sampling mode, RJW, bus errors and the
 *   bus-off/error interrupts (never requested), arbitration against
 *   frames arriving from the bus while transmitting.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "net/can_emu.h"
#include "hw/m68k/mc68376.h"

/* register offsets inside the module window (Table D-59) */
#define R_CANMCR        0x00
#define R_CANTCR        0x02
#define R_CANICR        0x04
#define R_CANCTRL01     0x06    /* CANCTRL0 (high) / CANCTRL1 (low) */
#define R_PRESDIV2      0x08    /* PRESDIV (high) / CANCTRL2 (low) */
#define R_TIMER         0x0a
#define R_RXGMSKHI      0x10
#define R_RXGMSKLO      0x12
#define R_RX14MSKHI     0x14
#define R_RX14MSKLO     0x16
#define R_RX15MSKHI     0x18
#define R_RX15MSKLO     0x1a
#define R_ESTAT         0x20
#define R_IMASK         0x22
#define R_IFLAG         0x24
#define R_ECTR          0x26    /* RXECTR (high) / TXECTR (low) */
#define R_MB            0x80    /* $YFF100: message buffers */

#define NMB             16

/* CANMCR (D.10.1) */
#define MCR_STOP        0x8000
#define MCR_FRZ         0x4000
#define MCR_HALT        0x1000
#define MCR_NOTRDY      0x0800
#define MCR_WAKEMSK     0x0400
#define MCR_SOFTRST     0x0200
#define MCR_FRZACK      0x0100
#define MCR_SUPV        0x0080
#define MCR_SELFWAKE    0x0040
#define MCR_APS         0x0020
#define MCR_STOPACK     0x0010
#define MCR_IARB        0x000f
#define MCR_RESET       0x5980
#define MCR_WMASK       (MCR_STOP | MCR_FRZ | MCR_HALT | MCR_WAKEMSK | \
                         MCR_SUPV | MCR_SELFWAKE | MCR_APS | MCR_IARB)

/* CANICR (D.10.3): reserved bits 3:0 read as reset ($000F) */
#define ICR_RESET       0x000f
#define ICR_WMASK       0x07e0
#define ICR_ILCAN(v)    (((v) >> 8) & 7)
#define ICR_IVBA(v)     (((v) >> 5) & 7)

/* CANCTRL0 (D.10.4), CANCTRL1 (D.10.5), CANCTRL2 (D.10.7) */
#define CTRL0_BOFFMSK   0x80
#define CTRL0_ERRMSK    0x40
#define CTRL0_WMASK     0xcf
#define CTRL1_SAMP      0x80
#define CTRL1_LOOP      0x40
#define CTRL1_TSYNC     0x20
#define CTRL1_LBUF      0x10
#define CTRL1_WMASK     0xf7

/* ESTAT (D.10.12) */
#define ESTAT_ERRBITS   0xfc00  /* BITERR, ACKERR, CRCERR, FORMERR, STUFF */
#define ESTAT_TXWARN    0x0200
#define ESTAT_RXWARN    0x0100
#define ESTAT_IDLE      0x0080
#define ESTAT_TXRX      0x0040
#define ESTAT_FCS_PASS  0x0010
#define ESTAT_BOFFINT   0x0004
#define ESTAT_ERRINT    0x0002
#define ESTAT_WAKEINT   0x0001
#define ESTAT_INTS      0x0007

/* message buffer codes (Tables 13-2, 13-3) */
#define CODE_RX_NOTACT  0x0
#define CODE_RX_EMPTY   0x4
#define CODE_RX_FULL    0x2
#define CODE_RX_OVERRUN 0x6
#define CODE_TX_NOTRDY  0x8
#define CODE_TX_ONCE    0xc
#define CODE_TX_RESP    0xa
#define CODE_TX_ONCERSP 0xe

/* ID_HIGH layout (Figures 13-3, 13-4) */
#define IDH_SRR         0x0010  /* extended: SRR; standard: RTR */
#define IDH_IDE         0x0008

/* interrupt sources above the 16 buffers (Table 13-9) */
#define SRC_BOFF        16
#define SRC_ERR         17
#define SRC_WAKE        18

typedef struct TouCANFrame {
    bool ide, rtr;
    uint32_t id;            /* 11 or 29 bits */
    uint8_t dlc;            /* DLC as sent (0..15) */
    uint8_t data[8];
} TouCANFrame;

struct MC68376TouCAN {
    MC68376State *soc;
    CanBusClientState bus_client;
    IMBIrq irq;
    Notifier clock_notifier;
    QEMUTimer *tx_timer;        /* end of the frame being transmitted */
    QEMUTimer *sync_timer;      /* 11 recessive bits after debug/stop */

    uint16_t mcr, tcr, icr;
    bool icr_written;           /* IVBA initialised since reset */
    uint8_t ctrl0, ctrl1, presdiv, ctrl2;
    uint16_t rxgmsk[2], rx14msk[2], rx15msk[2];
    uint16_t estat;             /* latched error bits and interrupt flags */
    uint16_t estat_rd;          /* interrupt flags read as one */
    uint16_t imask, iflag;
    uint16_t iflag_rd;          /* IFLAG bits read as one */
    uint8_t rxectr, txectr;
    uint16_t mb[NMB][8];

    /* free-running timer: value = base + elapsed bits while running */
    bool timer_on;
    uint16_t timer_base;
    int64_t timer_base_ns;

    /* lock / serial message buffer (13.4.1.6) */
    int locked;                 /* locked buffer or -1 */
    bool smb_pending;
    int smb_mb;
    TouCANFrame smb;
    uint16_t smb_stamp;

    /*
     * Time of the event being handled by a timer callback (end of frame,
     * end of resynchronisation), 0 otherwise: a transmission that follows
     * starts there, not when the callback happens to run.
     */
    int64_t evt_ns;
    int64_t sync_ns;            /* end of the resynchronisation */

    /* transmission in progress */
    bool tx_active;
    int64_t tx_end_ns;
    int tx_mb;                  /* -1 once the buffer was rewritten */
    TouCANFrame tx;
    uint16_t tx_stamp;
};

static void toucan_update_mode(MC68376TouCAN *m);
static void toucan_try_tx(MC68376TouCAN *m);

/* ---------------------------------------------------------------------- */
/* Bit timing and free-running timer (13.4.3, D.10.6-8)                   */
/* ---------------------------------------------------------------------- */

/* CAN bit time in picoseconds */
static uint64_t toucan_bit_ps(MC68376TouCAN *m)
{
    uint32_t fsys = mc68376_sysclk_hz(m->soc);
    unsigned propseg = m->ctrl1 & 7;
    unsigned pseg1 = (m->ctrl2 >> 3) & 7;
    unsigned pseg2 = m->ctrl2 & 7;
    /* SYNC_SEG (1) + PROP_SEG + PHASE_SEG1 + PHASE_SEG2 time quanta */
    uint64_t tq_per_bit = 1 + (propseg + 1) + (pseg1 + 1) + (pseg2 + 1);
    uint64_t sclk_div = m->presdiv + 1;

    if (!fsys) {
        fsys = 1;
    }
    /* muldiv64() takes a 32-bit multiplier: 10^12 = 10^6 x 10^6 */
    return muldiv64(tq_per_bit * sclk_div * 1000000, 1000000, fsys);
}

static int64_t toucan_bits_ns(MC68376TouCAN *m, uint64_t bits)
{
    return muldiv64(bits, toucan_bit_ps(m), 1000);
}

static uint64_t toucan_timer_ticks(MC68376TouCAN *m, int64_t now)
{
    int64_t el = now - m->timer_base_ns;

    if (!m->timer_on || el <= 0) {
        return 0;
    }
    return muldiv64(el, 1000, toucan_bit_ps(m));
}

static uint16_t toucan_timer_at(MC68376TouCAN *m, int64_t t)
{
    return m->timer_base + toucan_timer_ticks(m, t);
}

static uint16_t toucan_timer_get(MC68376TouCAN *m)
{
    return toucan_timer_at(m, mc68376_now());
}

/* fold the elapsed bits into the base (before the rate changes) */
static void toucan_timer_rebase(MC68376TouCAN *m)
{
    int64_t now = mc68376_now();
    uint64_t t = toucan_timer_ticks(m, now);

    if (m->timer_on && t) {
        m->timer_base += t;
        m->timer_base_ns += muldiv64(t, toucan_bit_ps(m), 1000);
    } else if (!m->timer_on) {
        m->timer_base_ns = now;
    }
}

static void toucan_timer_set(MC68376TouCAN *m, uint16_t v)
{
    m->timer_base = v;
    m->timer_base_ns = m->evt_ns ? m->evt_ns : mc68376_now();
}

static void toucan_timer_run(MC68376TouCAN *m, bool on)
{
    if (on == m->timer_on) {
        return;
    }
    if (!on) {
        m->timer_base = toucan_timer_get(m);
    }
    m->timer_on = on;
    m->timer_base_ns = mc68376_now();
}

static void toucan_clock_changed(Notifier *n, void *data)
{
    MC68376TouCAN *m = container_of(n, MC68376TouCAN, clock_notifier);

    /* the SoC already switched the clock; the error is a fraction of a bit */
    toucan_timer_rebase(m);
}

/* ---------------------------------------------------------------------- */
/* Interrupts (13.7)                                                      */
/* ---------------------------------------------------------------------- */

static void toucan_irq_update(MC68376TouCAN *m)
{
    uint32_t src = m->iflag & m->imask;
    uint8_t level = ICR_ILCAN(m->icr);
    uint8_t iarb = m->mcr & MCR_IARB;
    bool reprio;

    if ((m->estat & ESTAT_BOFFINT) && (m->ctrl0 & CTRL0_BOFFMSK)) {
        src |= 1u << SRC_BOFF;
    }
    if ((m->estat & ESTAT_ERRINT) && (m->ctrl0 & CTRL0_ERRMSK)) {
        src |= 1u << SRC_ERR;
    }
    if ((m->estat & ESTAT_WAKEINT) && (m->mcr & MCR_WAKEMSK)) {
        src |= 1u << SRC_WAKE;
    }
    /* Table 13-9: buffer 0 highest priority ... wake-up lowest */
    if (src) {
        m->irq.vector = m->icr_written
                        ? (ICR_IVBA(m->icr) << 5) | ctz32(src)
                        : MC68376_VEC_UNINIT;
    }
    reprio = m->irq.level != level || m->irq.iarb != iarb;
    m->irq.level = level;
    m->irq.iarb = iarb;
    if (reprio && m->irq.pending) {
        m->irq.pending = src && level;
        imb_irq_update(m->soc);
    } else {
        imb_irq_set(&m->irq, src && level);
    }
}

static void toucan_set_iflag(MC68376TouCAN *m, int n)
{
    m->iflag |= 1u << n;
    /* a new event between the read and the write keeps the flag */
    m->iflag_rd &= ~(1u << n);
}

/* ---------------------------------------------------------------------- */
/* Message buffers                                                        */
/* ---------------------------------------------------------------------- */

static inline unsigned mb_code(MC68376TouCAN *m, int n)
{
    return (m->mb[n][0] >> 4) & 0xf;
}

static inline void mb_set_code(MC68376TouCAN *m, int n, unsigned code)
{
    m->mb[n][0] = (m->mb[n][0] & ~0x00f0) | (code << 4);
}

/* active receive buffer: EMPTY, FULL or OVERRUN (BUSY bit ignored) */
static inline bool mb_is_rx(MC68376TouCAN *m, int n)
{
    unsigned c = mb_code(m, n);

    return !(c & 8) && (c & 6);
}

/* frame ID in the message buffer ID_HIGH/ID_LOW layout */
static void frame_to_idw(const TouCANFrame *f, uint16_t *hi, uint16_t *lo)
{
    if (f->ide) {
        *hi = ((f->id >> 18) & 0x7ff) << 5 | IDH_SRR | IDH_IDE |
              ((f->id >> 15) & 7);
        *lo = (f->id & 0x7fff) << 1 | f->rtr;
    } else {
        *hi = (f->id & 0x7ff) << 5 | (f->rtr ? IDH_SRR : 0);
        *lo = 0;
    }
}

static void mb_to_frame(MC68376TouCAN *m, int n, TouCANFrame *f)
{
    uint16_t hi = m->mb[n][1], lo = m->mb[n][2];

    f->ide = hi & IDH_IDE;
    if (f->ide) {
        f->id = (uint32_t)(hi >> 5) << 18 | (uint32_t)(hi & 7) << 15 |
                lo >> 1;
        f->rtr = lo & 1;
    } else {
        f->id = hi >> 5;
        f->rtr = hi & IDH_SRR;
    }
    f->dlc = m->mb[n][0] & 0xf;
    for (int i = 0; i < 4; i++) {
        f->data[2 * i] = m->mb[n][3 + i] >> 8;
        f->data[2 * i + 1] = m->mb[n][3 + i];
    }
}

/* frame length in bits including the intermission, without stuff bits */
static unsigned frame_bits(const TouCANFrame *f, bool with_ifs)
{
    unsigned n = f->rtr ? 0 : MIN(f->dlc, 8);

    return (f->ide ? 64 : 44) + 8 * n + (with_ifs ? 3 : 0);
}

/* acceptance filter (13.4.2, D.10.9) */
static bool mb_match(MC68376TouCAN *m, int n, const TouCANFrame *f)
{
    const uint16_t *mask = n < 14 ? m->rxgmsk : n == 14 ? m->rx14msk
                                                         : m->rx15msk;
    uint16_t fhi, flo;
    /* RTR/SRR never compared, IDE always compared */
    uint16_t mhi = (mask[0] & ~IDH_SRR) | IDH_IDE;
    uint16_t mlo = mask[1] & ~1;

    frame_to_idw(f, &fhi, &flo);
    if (!f->ide) {
        /* MID[17:0] mask only extended frames */
        return !((fhi ^ m->mb[n][1]) & mhi & 0xffe8);
    }
    return !((fhi ^ m->mb[n][1]) & mhi) && !((flo ^ m->mb[n][2]) & mlo);
}

/* exact ID match for remote frames (13.5.5): masks not used */
static bool mb_match_exact(MC68376TouCAN *m, int n, const TouCANFrame *f)
{
    uint16_t fhi, flo;

    frame_to_idw(f, &fhi, &flo);
    if (!f->ide) {
        return !((fhi ^ m->mb[n][1]) & 0xffef);
    }
    return !((fhi ^ m->mb[n][1]) & 0xffef) && !((flo ^ m->mb[n][2]) & 0xfffe);
}

/* move a received frame into buffer n (13.5.4 steps 1-5) */
static void mb_store(MC68376TouCAN *m, int n, const TouCANFrame *f,
                     uint16_t stamp)
{
    unsigned code = mb_code(m, n) & 0xe;
    uint16_t hi, lo;

    code = code == CODE_RX_EMPTY ? CODE_RX_FULL : CODE_RX_OVERRUN;
    frame_to_idw(f, &hi, &lo);
    m->mb[n][0] = (stamp & 0xff00) | code << 4 | (f->dlc & 0xf);
    m->mb[n][1] = hi;
    /* standard format: ID_LOW holds the 16-bit time stamp (Table 13-5) */
    m->mb[n][2] = f->ide ? lo : stamp;
    for (int i = 0; i < MIN(f->dlc, 8); i++) {
        uint16_t *w = &m->mb[n][3 + i / 2];
        *w = (i & 1) ? (*w & 0xff00) | f->data[i]
                     : (*w & 0x00ff) | f->data[i] << 8;
    }
    toucan_set_iflag(m, n);
    /* successful reception (13.4.4) */
    if (m->rxectr > 127) {
        m->rxectr = 119;
    } else if (m->rxectr) {
        m->rxectr--;
    }
    if (n == 0 && (m->ctrl1 & CTRL1_TSYNC)) {
        toucan_timer_set(m, 0);
    }
    qemu_log_mask(CPU_LOG_INT, "mc68376.toucan: rx mb%d id=0x%x%s dlc=%d\n",
                  n, f->id, f->ide ? "x" : "", f->dlc);
}

/* receive process for one error-free frame (13.5.4, 13.5.5) */
static void toucan_rx(MC68376TouCAN *m, const TouCANFrame *f, uint16_t stamp,
                      bool own)
{
    int target = -1;

    if (f->rtr) {
        bool any = false;
        for (int n = 0; n < NMB; n++) {
            if (mb_code(m, n) == CODE_TX_RESP && mb_match_exact(m, n, f)) {
                mb_set_code(m, n, CODE_TX_ONCERSP);
                any = true;
            }
        }
        if (any) {
            toucan_try_tx(m);
        }
        return;
    }
    for (int n = 0; n < NMB; n++) {
        if ((mb_code(m, n) & 0xe) == CODE_RX_EMPTY && mb_match(m, n, f)) {
            target = n;
            break;
        }
    }
    if (target < 0 && !own) {
        for (int n = 0; n < NMB; n++) {
            if (mb_is_rx(m, n) && mb_match(m, n, f)) {
                target = n;
                break;
            }
        }
    }
    if (target < 0) {
        return;
    }
    if (target == m->locked) {
        /* kept in the serial message buffer, the last one wins */
        m->smb_pending = true;
        m->smb_mb = target;
        m->smb = *f;
        m->smb_stamp = stamp;
        return;
    }
    mb_store(m, target, f, stamp);
}

static void toucan_unlock(MC68376TouCAN *m)
{
    int n = m->locked;

    m->locked = -1;
    if (n >= 0 && m->smb_pending && m->smb_mb == n) {
        m->smb_pending = false;
        if (mb_is_rx(m, n)) {
            mb_store(m, n, &m->smb, m->smb_stamp);
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Operating modes (13.5.1, 13.6)                                         */
/* ---------------------------------------------------------------------- */

static bool toucan_ready(MC68376TouCAN *m)
{
    return !(m->mcr & (MCR_NOTRDY | MCR_FRZACK | MCR_STOPACK));
}

static void toucan_update_mode(MC68376TouCAN *m)
{
    bool freeze = (m->mcr & MCR_HALT) && (m->mcr & MCR_FRZ);

    if (m->tx_active) {
        /* debug / stop mode is entered at the end of the frame */
        return;
    }
    if (m->mcr & MCR_STOP) {
        if (!(m->mcr & MCR_STOPACK)) {
            m->mcr |= MCR_STOPACK | MCR_NOTRDY;
            m->mcr &= ~MCR_FRZACK;
            timer_del(m->sync_timer);
            toucan_timer_run(m, false);
        }
    } else if (freeze) {
        if (!(m->mcr & MCR_FRZACK)) {
            m->mcr |= MCR_FRZACK | MCR_NOTRDY;
            m->mcr &= ~MCR_STOPACK;
            timer_del(m->sync_timer);
            toucan_timer_run(m, false);
        }
    } else {
        m->mcr &= ~(MCR_FRZACK | MCR_STOPACK);
        toucan_timer_run(m, true);
        if ((m->mcr & MCR_NOTRDY) && !timer_pending(m->sync_timer)) {
            /* resynchronise: 11 consecutive recessive bits */
            m->sync_ns = mc68376_now() + toucan_bits_ns(m, 11);
            timer_mod(m->sync_timer, m->sync_ns);
        }
    }
}

static void toucan_sync_done(void *opaque)
{
    MC68376TouCAN *m = opaque;

    if (!(m->mcr & (MCR_FRZACK | MCR_STOPACK))) {
        m->mcr &= ~MCR_NOTRDY;
        m->evt_ns = m->sync_ns;
        toucan_try_tx(m);
        m->evt_ns = 0;
    }
}

/* ---------------------------------------------------------------------- */
/* Transmit process (13.5.3)                                              */
/* ---------------------------------------------------------------------- */

/* CAN arbitration field as a number: lower wins */
static uint32_t frame_arb_key(const TouCANFrame *f)
{
    if (f->ide) {
        return (f->id >> 18) << 21 | 1u << 20 | 1u << 19 |
               (f->id & 0x3ffff) << 1 | f->rtr;
    }
    return f->id << 21 | (uint32_t)f->rtr << 20;
}

static void toucan_try_tx(MC68376TouCAN *m)
{
    int best = -1;
    uint32_t best_key = 0;
    int64_t start = m->evt_ns ? m->evt_ns : mc68376_now();
    TouCANFrame f;

    if (m->tx_active || !toucan_ready(m)) {
        return;
    }
    for (int n = 0; n < NMB; n++) {
        unsigned c = mb_code(m, n);
        if (c != CODE_TX_ONCE && c != CODE_TX_ONCERSP) {
            continue;
        }
        if (m->ctrl1 & CTRL1_LBUF) {
            best = n;
            break;
        }
        mb_to_frame(m, n, &f);
        if (best < 0 || frame_arb_key(&f) < best_key) {
            best = n;
            best_key = frame_arb_key(&f);
        }
    }
    if (best < 0) {
        return;
    }
    mb_to_frame(m, best, &m->tx);
    m->tx_mb = best;
    m->tx_active = true;
    /* time stamp: TIMER at the start of the identifier field (13.4.5) */
    m->tx_stamp = toucan_timer_at(m, start) + 1;
    m->tx_end_ns = start + toucan_bits_ns(m, frame_bits(&m->tx, true));
    timer_mod(m->tx_timer, m->tx_end_ns);
}

static void toucan_tx_done(void *opaque)
{
    MC68376TouCAN *m = opaque;
    TouCANFrame f = m->tx;
    int n = m->tx_mb;

    if (!m->tx_active) {
        return;
    }
    m->tx_active = false;
    m->evt_ns = m->tx_end_ns;

    if (!(m->ctrl1 & CTRL1_LOOP) && m->soc->canbus) {
        qemu_can_frame cf = { 0 };

        cf.can_id = f.id | (f.ide ? QEMU_CAN_EFF_FLAG : 0) |
                    (f.rtr ? QEMU_CAN_RTR_FLAG : 0);
        cf.can_dlc = MIN(f.dlc, 8);
        if (!f.rtr) {
            memcpy(cf.data, f.data, cf.can_dlc);
        }
        can_bus_client_send(&m->bus_client, &cf, 1);
    }
    qemu_log_mask(CPU_LOG_INT, "mc68376.toucan: tx mb%d id=0x%x%s%s dlc=%d%s\n",
                  n, f.id, f.ide ? "x" : "", f.rtr ? " rtr" : "", f.dlc,
                  (m->ctrl1 & CTRL1_LOOP) ? " (loop back)" : "");

    /* a buffer rewritten meanwhile gets no update and no flag (13.5.3.1) */
    if (n >= 0) {
        unsigned c = mb_code(m, n);

        m->mb[n][0] = (m->mb[n][0] & 0x00ff) | (m->tx_stamp & 0xff00);
        if (!f.ide) {
            m->mb[n][2] = m->tx_stamp;
        }
        if (c == CODE_TX_ONCE) {
            mb_set_code(m, n, f.rtr ? CODE_RX_EMPTY : CODE_TX_NOTRDY);
        } else {
            mb_set_code(m, n, CODE_TX_RESP);
        }
        toucan_set_iflag(m, n);
    }
    if (m->txectr) {
        m->txectr--;
    }
    /* reception of transmitted frames (13.5.3.2) */
    toucan_rx(m, &f, m->tx_stamp, true);

    toucan_update_mode(m);
    toucan_try_tx(m);
    m->evt_ns = 0;
    toucan_irq_update(m);
}

/* ---------------------------------------------------------------------- */
/* CAN bus client                                                         */
/* ---------------------------------------------------------------------- */

static bool toucan_can_receive(CanBusClientState *client)
{
    return true;
}

static ssize_t toucan_receive(CanBusClientState *client,
                              const qemu_can_frame *frames, size_t cnt)
{
    MC68376TouCAN *m = container_of(client, MC68376TouCAN, bus_client);

    for (size_t i = 0; i < cnt; i++) {
        const qemu_can_frame *cf = &frames[i];
        TouCANFrame f = { 0 };

        if (cf->can_id & QEMU_CAN_ERR_FLAG || cf->flags & QEMU_CAN_FRMF_TYPE_FD) {
            continue;
        }
        if (m->mcr & MCR_STOPACK) {
            /* recessive to dominant edge in low-power stop (13.6.2) */
            m->estat |= ESTAT_WAKEINT;
            m->estat_rd &= ~ESTAT_WAKEINT;
            if (!(m->mcr & MCR_SELFWAKE)) {
                continue;
            }
            m->mcr &= ~(MCR_STOP | MCR_STOPACK | MCR_NOTRDY);
            toucan_update_mode(m);
            timer_del(m->sync_timer);
            /* it tries to receive the frame that woke it up */
        }
        if (!toucan_ready(m) || (m->ctrl1 & CTRL1_LOOP)) {
            continue;
        }
        f.ide = cf->can_id & QEMU_CAN_EFF_FLAG;
        f.rtr = cf->can_id & QEMU_CAN_RTR_FLAG;
        f.id = cf->can_id & (f.ide ? QEMU_CAN_EFF_MASK : QEMU_CAN_SFF_MASK);
        f.dlc = MIN(cf->can_dlc, 8);
        memcpy(f.data, cf->data, f.dlc);
        /* the frame has just ended: stamp at its identifier field */
        toucan_rx(m, &f, toucan_timer_get(m) - (frame_bits(&f, false) - 1),
                  false);
    }
    toucan_try_tx(m);
    toucan_irq_update(m);
    return cnt;
}

static CanBusClientInfo toucan_bus_client_info = {
    .can_receive = toucan_can_receive,
    .receive = toucan_receive,
};

/* ---------------------------------------------------------------------- */
/* Reset                                                                  */
/* ---------------------------------------------------------------------- */

/* SOFTRST (D.10.1): host interface registers and internal state */
static void toucan_soft_reset(MC68376TouCAN *m)
{
    timer_del(m->tx_timer);
    timer_del(m->sync_timer);
    m->tx_active = false;
    m->tx_mb = -1;
    m->mcr = MCR_RESET;
    m->tcr = 0;
    m->icr = ICR_RESET;
    m->icr_written = false;
    m->imask = 0;
    m->iflag = 0;
    m->iflag_rd = 0;
    m->estat = 0;
    m->estat_rd = 0;
    m->rxectr = 0;
    m->txectr = 0;
    m->locked = -1;
    m->smb_pending = false;
    m->timer_on = false;
    toucan_timer_set(m, 0);
    toucan_update_mode(m);
    toucan_irq_update(m);
}

void mc68376_toucan_reset(MC68376State *s)
{
    MC68376TouCAN *m = s->toucan;

    m->ctrl0 = 0;
    m->ctrl1 = 0;
    m->presdiv = 0;
    m->ctrl2 = 0;
    m->rxgmsk[0] = m->rx14msk[0] = m->rx15msk[0] = 0xffef;
    m->rxgmsk[1] = m->rx14msk[1] = m->rx15msk[1] = 0xfffe;
    /* the message buffers keep their contents (13.5.1) */
    toucan_soft_reset(m);
}

/* ---------------------------------------------------------------------- */
/* Register access                                                        */
/* ---------------------------------------------------------------------- */

static uint16_t toucan_estat(MC68376TouCAN *m)
{
    uint16_t v = m->estat;

    if (m->txectr >= 96) {
        v |= ESTAT_TXWARN;
    }
    if (m->rxectr >= 96) {
        v |= ESTAT_RXWARN;
    }
    if (m->txectr >= 128 || m->rxectr >= 128) {
        v |= ESTAT_FCS_PASS;
    }
    if (m->tx_active) {
        v |= ESTAT_TXRX;
    } else {
        v |= ESTAT_IDLE;
    }
    return v;
}

/* @bm: bytes being read (0xff00 high, 0x00ff low) */
static uint16_t toucan_reg_read(MC68376TouCAN *m, hwaddr off, uint16_t bm)
{
    uint16_t v;

    if (off >= R_MB) {
        int n = (off - R_MB) >> 4, w = (off >> 1) & 7;
        if (w == 0) {
            /* reading the control/status word locks the buffer */
            if (m->locked != n) {
                toucan_unlock(m);
                m->locked = n;
            }
            toucan_irq_update(m);
        }
        return m->mb[n][w];
    }
    switch (off) {
    case R_CANMCR:
        return m->mcr;
    case R_CANTCR:
        return m->tcr;
    case R_CANICR:
        return m->icr;
    case R_CANCTRL01:
        return m->ctrl0 << 8 | m->ctrl1;
    case R_PRESDIV2:
        return m->presdiv << 8 | m->ctrl2;
    case R_TIMER:
        /* global release of the locked buffer */
        toucan_unlock(m);
        toucan_irq_update(m);
        return toucan_timer_get(m);
    case R_RXGMSKHI:
    case R_RXGMSKLO:
        return m->rxgmsk[(off >> 1) & 1];
    case R_RX14MSKHI:
    case R_RX14MSKLO:
        return m->rx14msk[(off >> 1) & 1];
    case R_RX15MSKHI:
    case R_RX15MSKLO:
        return m->rx15msk[(off >> 1) & 1];
    case R_ESTAT:
        v = toucan_estat(m);
        /* error bits clear on read, flags arm for the write of zero */
        m->estat &= ~(ESTAT_ERRBITS & bm);
        m->estat_rd |= m->estat & ESTAT_INTS & bm;
        return v;
    case R_IMASK:
        return m->imask;
    case R_IFLAG:
        m->iflag_rd |= m->iflag & bm;
        return m->iflag;
    case R_ECTR:
        return m->rxectr << 8 | m->txectr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.toucan: read of reserved "
                      "offset 0x%03" HWADDR_PRIx "\n", off + 0x80);
        return 0;
    }
}

static uint16_t merge(uint16_t old, uint16_t val, uint16_t mask)
{
    return (old & ~mask) | (val & mask);
}

static void toucan_reg_write(MC68376TouCAN *m, hwaddr off, uint16_t val,
                             uint16_t bm)
{
    uint16_t v, clr;

    if (off >= R_MB) {
        int n = (off - R_MB) >> 4, w = (off >> 1) & 7;
        m->mb[n][w] = merge(m->mb[n][w], val, bm);
        if (w == 0) {
            if (m->tx_active && m->tx_mb == n) {
                /* sent anyway, but no flag and no code update */
                m->tx_mb = -1;
            }
            toucan_try_tx(m);
        }
        return;
    }
    switch (off) {
    case R_CANMCR:
        if (val & bm & MCR_SOFTRST) {
            toucan_soft_reset(m);
            return;
        }
        m->mcr = merge(m->mcr, val, bm & MCR_WMASK);
        toucan_update_mode(m);
        break;
    case R_CANTCR:
        m->tcr = merge(m->tcr, val, bm);
        if (m->tcr) {
            qemu_log_mask(LOG_UNIMP, "mc68376.toucan: CANTCR test modes "
                          "not modelled (0x%04x)\n", m->tcr);
        }
        break;
    case R_CANICR:
        m->icr = merge(m->icr, val, bm & ICR_WMASK);
        if (bm & 0x00e0) {
            m->icr_written = true;
        }
        break;
    case R_CANCTRL01:
        toucan_timer_rebase(m);
        v = merge(m->ctrl0 << 8 | m->ctrl1, val,
                  bm & (CTRL0_WMASK << 8 | CTRL1_WMASK));
        m->ctrl0 = v >> 8;
        m->ctrl1 = v;
        break;
    case R_PRESDIV2:
        toucan_timer_rebase(m);
        v = merge(m->presdiv << 8 | m->ctrl2, val, bm);
        m->presdiv = v >> 8;
        m->ctrl2 = v;
        break;
    case R_TIMER:
        toucan_timer_set(m, merge(toucan_timer_get(m), val, bm));
        break;
    case R_RXGMSKHI:
    case R_RXGMSKLO:
    case R_RX14MSKHI:
    case R_RX14MSKLO:
    case R_RX15MSKHI:
    case R_RX15MSKLO: {
        uint16_t *r = (off < R_RX14MSKHI ? m->rxgmsk :
                       off < R_RX15MSKHI ? m->rx14msk : m->rx15msk) +
                      ((off >> 1) & 1);
        *r = merge(*r, val, bm);
        /* RTR mask bits (20, 0) read 0, the IDE mask bit (19) reads 1 */
        if (off & 2) {
            *r &= ~1;
        } else {
            *r = (*r & ~IDH_SRR) | IDH_IDE;
        }
        break;
    }
    case R_ESTAT:
        /* only BOFFINT, ERRINT, WAKEINT: cleared by writing 0 after 1 */
        clr = ~val & bm & m->estat_rd;
        m->estat &= ~clr;
        m->estat_rd &= ~(bm & ESTAT_INTS);
        break;
    case R_IMASK:
        m->imask = merge(m->imask, val, bm);
        break;
    case R_IFLAG:
        /* written to zeros only, and only bits read as one (D.10.14) */
        clr = ~val & bm & m->iflag_rd;
        m->iflag &= ~clr;
        m->iflag_rd &= ~bm;
        break;
    case R_ECTR:
        /* read-only except in test or debug mode (D.10.15) */
        if (m->mcr & MCR_FRZACK) {
            v = merge(m->rxectr << 8 | m->txectr, val, bm);
            m->rxectr = v >> 8;
            m->txectr = v;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.toucan: write to reserved "
                      "offset 0x%03" HWADDR_PRIx "\n", off + 0x80);
        return;
    }
    toucan_irq_update(m);
}

static uint64_t toucan_read(void *opaque, hwaddr addr, unsigned size)
{
    MC68376TouCAN *m = opaque;
    hwaddr off = addr & ~1;
    uint16_t bm = size == 2 ? 0xffff : (addr & 1) ? 0x00ff : 0xff00;
    uint16_t v;

    if ((m->mcr & MCR_STOPACK) && off != R_CANMCR) {
        /* low-power stop: only CANMCR is accessible (13.6.2) */
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.toucan: read @ 0x%03"
                      HWADDR_PRIx " in low-power stop\n", addr);
        return 0;
    }
    v = toucan_reg_read(m, off, bm);
    if (size == 2) {
        return v;
    }
    return (addr & 1) ? (v & 0xff) : (v >> 8);
}

static void toucan_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    MC68376TouCAN *m = opaque;
    hwaddr off = addr & ~1;
    uint16_t bm = size == 2 ? 0xffff : (addr & 1) ? 0x00ff : 0xff00;

    if (size == 1) {
        val = (addr & 1) ? (val & 0xff) : (val & 0xff) << 8;
    }
    if ((m->mcr & MCR_STOPACK) && off != R_CANMCR) {
        qemu_log_mask(LOG_GUEST_ERROR, "mc68376.toucan: write @ 0x%03"
                      HWADDR_PRIx " in low-power stop\n", addr);
        return;
    }
    toucan_reg_write(m, off, val, bm);
}

static const MemoryRegionOps toucan_ops = {
    .read = toucan_read,
    .write = toucan_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

void mc68376_toucan_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    MC68376TouCAN *m = g_new0(MC68376TouCAN, 1);

    m->soc = s;
    s->toucan = m;
    m->locked = -1;
    memory_region_init_io(mr, OBJECT(s), &toucan_ops, m, "mc68376.toucan",
                          MC68376_TOUCAN_SIZE);
    m->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, toucan_tx_done, m);
    m->sync_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, toucan_sync_done, m);
    imb_irq_register(s, &m->irq, "toucan");
    m->irq.opaque = m;
    m->clock_notifier.notify = toucan_clock_changed;
    mc68376_add_clock_notifier(s, &m->clock_notifier);

    m->bus_client.info = &toucan_bus_client_info;
    if (s->canbus && can_bus_insert_client(s->canbus, &m->bus_client) < 0) {
        error_setg(errp, "mc68376: cannot attach TouCAN to CAN bus");
    }
}
