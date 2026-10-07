/*
 * Renesas SH705x Hitachi Controller Area Network: HCAN (SH7052/SH7054/
 * SH7055, 16 mailboxes) and HCAN2 (SH7058, 32 mailboxes).
 *
 * Frames are exchanged with a QEMU "can-bus" object, which can be bridged
 * to a host SocketCAN interface with "can-host-socketcan".  Bit timing,
 * error counters and the timer/time stamp unit are not modelled;
 * transmissions complete immediately and are always acknowledged.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/sh4/sh705x.h"

/* IRR bits in the "logical" numbering IRRn of the manuals */
enum {
    IRR_RST = 0,        /* v1: reset, v2: reset/halt/sleep */
    IRR_RXM = 1,        /* data frame received */
    IRR_RFR = 2,        /* remote frame received */
    IRR_TOW = 3,
    IRR_ROW = 4,
    IRR_EP = 5,
    IRR_BOF = 6,
    IRR_OLF = 7,
    IRR_MBE = 8,        /* mailbox empty */
    IRR_URI = 9,        /* unread message overwrite */
    IRR_WAKE = 12,
};

/* HCAN (v1) swaps the bytes of 16-bit mailbox/IRR registers */
static uint16_t v1_swap(uint32_t bits)
{
    return ((bits & 0xff) << 8) | ((bits >> 8) & 0xff);
}

static void hcan_irq(SH705xHCAN *h)
{
    uint16_t irr = h->irr & ~h->imr;   /* logical numbering */
    SH705xState *s = h->soc;
    uint32_t rx_pend = h->rxpr & ~h->mbimr;
    uint32_t rf_pend = h->rfpr & ~h->mbimr;

    if (rx_pend) {
        h->irr |= 1 << IRR_RXM;
    } else {
        h->irr &= ~(1 << IRR_RXM);
    }
    if (rf_pend) {
        h->irr |= 1 << IRR_RFR;
    } else {
        h->irr &= ~(1 << IRR_RFR);
    }
    irr = h->irr & ~h->imr;

    sh705x_set_irq(s, h->vec_base + 0, irr & 0x0078);           /* ERS */
    sh705x_set_irq(s, h->vec_base + 1, irr & 0xfe81);           /* OVR */
    sh705x_set_irq(s, h->vec_base + 2, irr & 0x0006);           /* RM */
    sh705x_set_irq(s, h->vec_base + 3, irr & 0x0100);           /* SLE */
}

static void hcan_enter_reset(SH705xHCAN *h)
{
    h->gsr = 0x000c;
    h->irr |= 1 << IRR_RST;
    h->txpr = 0;
    h->tec = h->rec = 0;
}

void sh705x_hcan_reset(SH705xHCAN *h)
{
    h->mcr = 0x0001;
    h->irr = 0;
    h->imr = 0xffff;
    h->txpr = h->txcr = h->txack = h->aback = 0;
    h->rxpr = h->rfpr = h->umsr = 0;
    h->mbimr = 0xffffffff;
    h->bcr = h->mbcr = h->lafml = h->lafmh = 0;
    h->bcr1 = h->bcr0 = 0;
    memset(h->mc, 0, sizeof(h->mc));
    memset(h->md, 0, sizeof(h->md));
    memset(h->mb, 0, sizeof(h->mb));
    memset(h->timer_regs, 0, sizeof(h->timer_regs));
    for (int i = 0; i < SH705X_HCAN_MB_MAX; i++) {
        h->mb[i][4] = 0x07;     /* MBC = 111: mailbox inactive */
    }
    hcan_enter_reset(h);
    hcan_irq(h);
}

static bool hcan_active(SH705xHCAN *h)
{
    return !(h->mcr & 0x0003) && !(h->gsr & 0x0008);
}

/* ---------------------------------------------------------------------- */
/* Mailbox <-> frame conversion                                           */
/* ---------------------------------------------------------------------- */

typedef struct MbId {
    uint32_t std;       /* 11 bits */
    uint32_t ext;       /* 18 bits */
    bool ide, rtr;
} MbId;

static MbId v1_get_id(const uint8_t *mc)
{
    MbId id;

    id.std = ((uint32_t)mc[5] << 3) | (mc[4] >> 5);
    id.ext = ((uint32_t)(mc[4] & 3) << 16) | ((uint32_t)mc[7] << 8) | mc[6];
    id.rtr = mc[4] & 0x10;
    id.ide = mc[4] & 0x08;
    return id;
}

static MbId v2_get_id(const uint8_t *mb)
{
    uint16_t h = (mb[0] << 8) | mb[1];
    uint16_t m = (mb[2] << 8) | mb[3];
    MbId id;

    id.std = (h >> 4) & 0x7ff;
    id.ext = ((uint32_t)(h & 3) << 16) | m;
    id.rtr = h & 0x08;
    id.ide = h & 0x04;
    return id;
}

static void frame_to_id(const qemu_can_frame *f, MbId *id)
{
    id->rtr = f->can_id & QEMU_CAN_RTR_FLAG;
    id->ide = f->can_id & QEMU_CAN_EFF_FLAG;
    if (id->ide) {
        id->std = (f->can_id >> 18) & 0x7ff;
        id->ext = f->can_id & 0x3ffff;
    } else {
        id->std = f->can_id & 0x7ff;
        id->ext = 0;
    }
}

static void id_to_frame(const MbId *id, qemu_can_frame *f)
{
    if (id->ide) {
        f->can_id = (id->std << 18) | id->ext | QEMU_CAN_EFF_FLAG;
    } else {
        f->can_id = id->std;
    }
    if (id->rtr) {
        f->can_id |= QEMU_CAN_RTR_FLAG;
    }
}

static bool id_match(const MbId *mb, const MbId *rx, uint32_t std_mask,
                     uint32_t ext_mask)
{
    /* masks: 1 = must match */
    if (mb->ide != rx->ide) {
        return false;
    }
    if ((mb->std ^ rx->std) & std_mask) {
        return false;
    }
    if (rx->ide && ((mb->ext ^ rx->ext) & ext_mask)) {
        return false;
    }
    return true;
}

static void hcan_transmit(SH705xHCAN *h, int n)
{
    qemu_can_frame f = { 0 };
    MbId id;
    int dlc;

    if (h->v2) {
        id = v2_get_id(h->mb[n]);
        dlc = h->mb[n][5] & 0xf;
        memcpy(f.data, &h->mb[n][8], 8);
    } else {
        id = v1_get_id(h->mc[n]);
        dlc = h->mc[n][0] & 0xf;
        memcpy(f.data, h->md[n], 8);
    }
    id_to_frame(&id, &f);
    f.can_dlc = MIN(dlc, 8);
    if (h->canbus) {
        can_bus_client_send(&h->bus_client, &f, 1);
    }
    qemu_log_mask(CPU_LOG_INT, "hcan%d: tx mb%d id=0x%x dlc=%d\n", h->index,
                  n, f.can_id, f.can_dlc);
}

static void hcan_process_tx(SH705xHCAN *h)
{
    if (!hcan_active(h) || !h->txpr) {
        return;
    }
    /* lower mailbox number first */
    for (int n = 1; n < h->nmb; n++) {
        if (h->txpr & (1u << n)) {
            hcan_transmit(h, n);
            h->txpr &= ~(1u << n);
            h->txack |= 1u << n;
        }
    }
    h->irr |= 1 << IRR_MBE;
}

static bool hcan_is_rx_mb(SH705xHCAN *h, int n, bool rtr)
{
    if (h->v2) {
        int mbc = (h->mb[n][4] >> 0) & 7;
        switch (mbc) {
        case 1:     /* transmit remote, receive data */
            return !rtr;
        case 2:     /* receive data and remote */
            return true;
        case 3:     /* receive remote only */
            return rtr;
        case 4:     /* receive data only */
            return !rtr;
        case 5:
            return !rtr;
        default:
            return false;
        }
    }
    return n == 0 || (h->mbcr & (1u << n));
}

static void hcan_store(SH705xHCAN *h, int n, const qemu_can_frame *f,
                       bool rtr)
{
    uint32_t bit = 1u << n;
    int dlc = MIN(f->can_dlc, 8);

    if ((h->rxpr | h->rfpr) & bit) {
        h->umsr |= bit;
        h->irr |= 1 << IRR_URI;
        if (h->v2 && !(h->mb[n][4] & 0x20)) {
            return;     /* NMC = 0: keep the old message */
        }
    }
    if (h->v2) {
        MbId id;
        uint16_t ctrlh, ctrlm;

        frame_to_id(f, &id);
        ctrlh = (id.std << 4) | (id.rtr ? 8 : 0) | (id.ide ? 4 : 0) |
                ((id.ext >> 16) & 3);
        ctrlm = id.ext & 0xffff;
        h->mb[n][0] = ctrlh >> 8;
        h->mb[n][1] = ctrlh;
        h->mb[n][2] = ctrlm >> 8;
        h->mb[n][3] = ctrlm;
        h->mb[n][5] = (h->mb[n][5] & 0xf0) | dlc;
        memset(&h->mb[n][8], 0, 8);
        memcpy(&h->mb[n][8], f->data, dlc);
    } else {
        MbId id;

        frame_to_id(f, &id);
        if (n == 0) {
            /* MB0 stores the received identifier (filtered by LAFM) */
            h->mc[0][4] = ((id.std & 7) << 5) | (id.rtr ? 0x10 : 0) |
                          (id.ide ? 0x08 : 0) | ((id.ext >> 16) & 3);
            h->mc[0][5] = id.std >> 3;
            h->mc[0][6] = id.ext;
            h->mc[0][7] = id.ext >> 8;
        }
        h->mc[n][0] = (h->mc[n][0] & 0xf0) | dlc;
        memset(h->md[n], 0, 8);
        memcpy(h->md[n], f->data, dlc);
    }
    if (rtr) {
        h->rfpr |= bit;
    } else {
        h->rxpr |= bit;
    }
}

static void hcan_rx_frame(SH705xHCAN *h, const qemu_can_frame *f)
{
    MbId rx;
    bool rtr = f->can_id & QEMU_CAN_RTR_FLAG;

    if (f->can_id & QEMU_CAN_ERR_FLAG || (f->flags & QEMU_CAN_FRMF_TYPE_FD)) {
        return;
    }
    frame_to_id(f, &rx);

    for (int n = h->nmb - 1; n >= 0; n--) {
        MbId mb;
        uint32_t std_mask = 0x7ff, ext_mask = 0x3ffff;

        if (!hcan_is_rx_mb(h, n, rtr)) {
            continue;
        }
        if (h->v2) {
            uint16_t lh = (h->mb[n][0x10] << 8) | h->mb[n][0x11];
            uint16_t ll = (h->mb[n][0x12] << 8) | h->mb[n][0x13];
            mb = v2_get_id(h->mb[n]);
            std_mask = ~(lh >> 4) & 0x7ff;
            ext_mask = ~((((uint32_t)lh & 3) << 16) | ll) & 0x3ffff;
        } else {
            mb = v1_get_id(h->mc[n]);
            if (n == 0) {
                uint8_t l5 = h->lafmh >> 8, l6 = h->lafmh & 0xff;
                uint8_t l7 = h->lafml >> 8, l8 = h->lafml & 0xff;
                std_mask = ~(((uint32_t)l6 << 3) | (l5 >> 5)) & 0x7ff;
                ext_mask = ~(((uint32_t)(l5 & 3) << 16) |
                             ((uint32_t)l8 << 8) | l7) & 0x3ffff;
            }
        }
        if (id_match(&mb, &rx, std_mask, ext_mask)) {
            hcan_store(h, n, f, rtr);
            qemu_log_mask(CPU_LOG_INT, "hcan%d: rx id=0x%x -> mb%d\n",
                          h->index, f->can_id, n);
            return;
        }
    }
}

static bool hcan_can_receive(CanBusClientState *client)
{
    SH705xHCAN *h = container_of(client, SH705xHCAN, bus_client);

    return hcan_active(h);
}

static ssize_t hcan_receive(CanBusClientState *client,
                            const qemu_can_frame *frames, size_t n)
{
    SH705xHCAN *h = container_of(client, SH705xHCAN, bus_client);

    if (!hcan_active(h)) {
        return n;
    }
    for (size_t i = 0; i < n; i++) {
        hcan_rx_frame(h, &frames[i]);
    }
    hcan_irq(h);
    return n;
}

static CanBusClientInfo hcan_bus_client_info = {
    .can_receive = hcan_can_receive,
    .receive = hcan_receive,
};

/* ---------------------------------------------------------------------- */
/* Register access                                                        */
/* ---------------------------------------------------------------------- */

static uint32_t mb_reg_get(SH705xHCAN *h, uint32_t v, bool high)
{
    return high ? v >> 16 : v & 0xffff;
}

static void mb_w1c(uint32_t *reg, bool high, uint16_t val, uint16_t mask)
{
    uint32_t bits = (uint32_t)(val & mask) << (high ? 16 : 0);

    *reg &= ~bits;
}

static void mcr_write(SH705xHCAN *h, uint16_t mcr)
{
    bool was_reset = h->mcr & 1;

    h->mcr = mcr;
    if (mcr & 1) {
        if (!was_reset) {
            hcan_enter_reset(h);
        }
    } else if (mcr & 2) {
        h->gsr = (h->gsr & ~0x0008) | 0x0010;     /* halt */
    } else {
        h->gsr &= ~0x0018;                       /* bus idle, active */
        h->gsr |= 0x0004;
        hcan_process_tx(h);
    }
}

static uint16_t v2_read(SH705xHCAN *h, uint32_t off)
{
    if (off >= 0x100 && off < 0x100 + 0x20 * (uint32_t)h->nmb) {
        uint8_t *b = &h->mb[(off - 0x100) / 0x20][off & 0x1f];
        return (b[0] << 8) | b[1];
    }
    if (off >= 0x80 && off < 0xa0) {
        return (h->timer_regs[off - 0x80] << 8) | h->timer_regs[off - 0x7f];
    }
    switch (off) {
    case 0x00: return h->mcr;
    case 0x02: return h->gsr;
    case 0x04: return h->bcr1;
    case 0x06: return h->bcr0;
    case 0x08: return h->irr;
    case 0x0a: return h->imr;
    case 0x0c: return (h->tec << 8) | h->rec;
    case 0x20: case 0x22: return mb_reg_get(h, h->txpr, off == 0x20);
    case 0x28: case 0x2a: return mb_reg_get(h, h->txcr, off == 0x28);
    case 0x30: case 0x32: return mb_reg_get(h, h->txack, off == 0x30);
    case 0x38: case 0x3a: return mb_reg_get(h, h->aback, off == 0x38);
    case 0x40: case 0x42: return mb_reg_get(h, h->rxpr, off == 0x40);
    case 0x48: case 0x4a: return mb_reg_get(h, h->rfpr, off == 0x48);
    case 0x50: case 0x52: return mb_reg_get(h, h->mbimr, off == 0x50);
    case 0x58: case 0x5a: return mb_reg_get(h, h->umsr, off == 0x58);
    }
    return 0;
}

static void v2_write(SH705xHCAN *h, uint32_t off, uint16_t val,
                     uint16_t mask)
{
    bool high = !(off & 2);
    uint32_t bits = (uint32_t)(val & mask) << (high ? 16 : 0);

    if (off >= 0x100 && off < 0x100 + 0x20 * (uint32_t)h->nmb) {
        uint8_t *b = &h->mb[(off - 0x100) / 0x20][off & 0x1f];
        if (mask & 0xff00) {
            b[0] = val >> 8;
        }
        if (mask & 0x00ff) {
            b[1] = val;
        }
        return;
    }
    if (off >= 0x80 && off < 0xa0) {
        if (mask & 0xff00) {
            h->timer_regs[off - 0x80] = val >> 8;
        }
        if (mask & 0x00ff) {
            h->timer_regs[off - 0x7f] = val;
        }
        return;
    }
    switch (off) {
    case 0x00:
        mcr_write(h, sh705x_merge(h->mcr, val, mask));
        break;
    case 0x04:
        h->bcr1 = sh705x_merge(h->bcr1, val, mask);
        break;
    case 0x06:
        h->bcr0 = sh705x_merge(h->bcr0, val, mask);
        break;
    case 0x08:
        /* write 1 to clear; receive flags follow RXPR/RFPR */
        h->irr &= ~(val & mask & ~0x0006);
        break;
    case 0x0a:
        h->imr = sh705x_merge(h->imr, val, mask);
        break;
    case 0x0c:
        if (mask & 0xff00) {
            h->tec = val >> 8;
        }
        if (mask & 0x00ff) {
            h->rec = val;
        }
        break;
    case 0x20: case 0x22:
        h->txpr |= bits & ~1u;
        hcan_process_tx(h);
        break;
    case 0x28: case 0x2a:
        bits &= h->txpr;
        h->txpr &= ~bits;
        h->aback |= bits;
        break;
    case 0x30: case 0x32:
        mb_w1c(&h->txack, high, val, mask);
        break;
    case 0x38: case 0x3a:
        mb_w1c(&h->aback, high, val, mask);
        break;
    case 0x40: case 0x42:
        mb_w1c(&h->rxpr, high, val, mask);
        break;
    case 0x48: case 0x4a:
        mb_w1c(&h->rfpr, high, val, mask);
        break;
    case 0x50: case 0x52:
        if (high) {
            h->mbimr = (h->mbimr & 0xffff) |
                       ((uint32_t)sh705x_merge(h->mbimr >> 16, val, mask)
                        << 16);
        } else {
            h->mbimr = (h->mbimr & 0xffff0000) |
                       sh705x_merge(h->mbimr, val, mask);
        }
        break;
    case 0x58: case 0x5a:
        mb_w1c(&h->umsr, high, val, mask);
        break;
    }
}

/* HCAN (v1) */
static uint16_t v1_read(SH705xHCAN *h, uint32_t off)
{
    if (off >= 0x20 && off < 0xa0) {
        uint8_t *b = &h->mc[(off - 0x20) / 8][off & 7];
        return (b[0] << 8) | b[1];
    }
    if (off >= 0xb0 && off < 0x130) {
        uint8_t *b = &h->md[(off - 0xb0) / 8][off & 7];
        return (b[0] << 8) | b[1];
    }
    switch (off) {
    case 0x00: return (h->mcr << 8) | (h->gsr & 0xff);
    case 0x02: return h->bcr;
    case 0x04: return v1_swap(h->mbcr);
    case 0x06: return v1_swap(h->txpr);
    case 0x08: return v1_swap(h->txcr);
    case 0x0a: return v1_swap(h->txack);
    case 0x0c: return v1_swap(h->aback);
    case 0x0e: return v1_swap(h->rxpr);
    case 0x10: return v1_swap(h->rfpr);
    case 0x12: return v1_swap(h->irr);
    case 0x14: return v1_swap(h->mbimr);
    case 0x16: return v1_swap(h->imr);
    case 0x18: return (h->rec << 8) | h->tec;
    case 0x1a: return v1_swap(h->umsr);
    case 0x1c: return h->lafml;
    case 0x1e: return h->lafmh;
    }
    return 0;
}

static void v1_write(SH705xHCAN *h, uint32_t off, uint16_t val,
                     uint16_t mask)
{
    uint16_t sv = v1_swap(val), smask = v1_swap(mask);
    uint32_t bits = sv & smask;

    if (off >= 0x20 && off < 0xa0) {
        uint8_t *b = &h->mc[(off - 0x20) / 8][off & 7];
        if (mask & 0xff00) {
            b[0] = val >> 8;
        }
        if (mask & 0x00ff) {
            b[1] = val;
        }
        return;
    }
    if (off >= 0xb0 && off < 0x130) {
        uint8_t *b = &h->md[(off - 0xb0) / 8][off & 7];
        if (mask & 0xff00) {
            b[0] = val >> 8;
        }
        if (mask & 0x00ff) {
            b[1] = val;
        }
        return;
    }
    switch (off) {
    case 0x00:
        if (mask & 0xff00) {
            mcr_write(h, val >> 8);
        }
        break;
    case 0x02:
        h->bcr = sh705x_merge(h->bcr, val, mask);
        break;
    case 0x04:
        /* stored with bit n = mailbox n */
        h->mbcr = v1_swap(sh705x_merge(v1_swap(h->mbcr), val, mask));
        break;
    case 0x06:
        h->txpr |= bits & ~1u;
        hcan_process_tx(h);
        break;
    case 0x08:
        bits &= h->txpr;
        h->txpr &= ~bits;
        h->aback |= bits;
        break;
    case 0x0a:
        h->txack &= ~bits;
        break;
    case 0x0c:
        h->aback &= ~bits;
        break;
    case 0x0e:
        h->rxpr &= ~bits;
        break;
    case 0x10:
        h->rfpr &= ~bits;
        break;
    case 0x12:
        h->irr &= ~(bits & ~0x0006);
        break;
    case 0x14:
        h->mbimr = (h->mbimr & ~(uint32_t)smask) | bits;
        break;
    case 0x16:
        h->imr = (h->imr & ~smask) | bits;
        break;
    case 0x18:
        if (mask & 0xff00) {
            h->rec = val >> 8;
        }
        if (mask & 0x00ff) {
            h->tec = val;
        }
        break;
    case 0x1a:
        h->umsr &= ~bits;
        break;
    case 0x1c:
        h->lafml = sh705x_merge(h->lafml, val, mask);
        break;
    case 0x1e:
        h->lafmh = sh705x_merge(h->lafmh, val, mask);
        break;
    }
}

uint16_t sh705x_hcan_read(SH705xHCAN *h, uint32_t off)
{
    return h->v2 ? v2_read(h, off) : v1_read(h, off);
}

void sh705x_hcan_write(SH705xHCAN *h, uint32_t off, uint16_t val,
                       uint16_t mask)
{
    if (h->v2) {
        v2_write(h, off, val, mask);
    } else {
        v1_write(h, off, val, mask);
    }
    hcan_irq(h);
}

void sh705x_hcan_init(SH705xHCAN *h, SH705xState *s, int index, bool v2,
                      int nmb, int vec_base)
{
    h->soc = s;
    h->index = index;
    h->v2 = v2;
    h->nmb = nmb;
    h->vec_base = vec_base;
    h->bus_client.info = &hcan_bus_client_info;
    if (h->canbus && can_bus_insert_client(h->canbus, &h->bus_client) < 0) {
        error_report("sh705x: cannot attach HCAN%d to CAN bus", index);
    }
    sh705x_hcan_reset(h);
}
