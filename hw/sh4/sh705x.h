/*
 * Renesas SH705x automotive microcontrollers (SH7052/SH7054/SH7055/SH7058)
 *
 * Internal definitions shared by the SoC model files.
 *
 * Register addresses, vector numbers and bit layouts follow the Renesas
 * hardware manuals / iodefine headers (as collected by the nissutils
 * project).  Peripheral behaviour is modelled closely enough to run engine
 * management firmware against the ECU engine simulator (hw/ecu).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SH4_SH705X_H
#define HW_SH4_SH705X_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "net/can_emu.h"
#include "target/sh4/cpu.h"
#include "hw/ecu/ecu.h"

#define TYPE_SH705X_SOC "sh705x-soc"
OBJECT_DECLARE_SIMPLE_TYPE(SH705xState, SH705X_SOC)

typedef enum {
    SH705X_7052,
    SH705X_7054,
    SH705X_7055,
    SH705X_7058,
} SH705xVariant;

#define SH705X_NUM_VECTORS  256
#define SH705X_NUM_SCI      5
#define SH705X_NUM_ADC      3
#define SH705X_NUM_HCAN     2
#define SH705X_NUM_PORTS    11
#define SH705X_HCAN_MB_MAX  32

/* Peripheral register space covered by the I/O dispatcher */
#define SH705X_IO_BASE      0xFFFFD000u
#define SH705X_IO_SIZE      0x3000u

/* Interrupt vector numbers */
enum {
    SH705X_VEC_NMI      = 11,
    SH705X_VEC_IRQ0     = 64,
    SH705X_VEC_ITV      = 80,
    SH705X_VEC_ICI0A    = 84,   /* +2 per channel: A,B,C,D */
    SH705X_VEC_OVI0     = 92,
    SH705X_VEC_IMI1A    = 96,   /* A..H = 96..103 */
    SH705X_VEC_OVI1     = 104,
    SH705X_VEC_IMI2A    = 108,  /* A..H = 108..115 */
    SH705X_VEC_OVI2     = 116,
    SH705X_VEC_IMI3A    = 120,
    SH705X_VEC_OVI3     = 124,
    SH705X_VEC_IMI4A    = 128,
    SH705X_VEC_OVI4     = 132,
    SH705X_VEC_IMI5A    = 136,
    SH705X_VEC_OVI5     = 140,
    SH705X_VEC_CMI6A    = 144,
    SH705X_VEC_CMI7A    = 148,
    SH705X_VEC_OSI8A    = 152,  /* A..P = 152..167 */
    SH705X_VEC_CMI9A    = 168,  /* A..D = 168..171 */
    SH705X_VEC_CMI9E    = 172,
    SH705X_VEC_CMI9F    = 174,
    SH705X_VEC_CMI10A   = 176,
    SH705X_VEC_CMI10B   = 178,
    SH705X_VEC_ICI10A   = 180,
    SH705X_VEC_IMI11A   = 184,
    SH705X_VEC_IMI11B   = 186,
    SH705X_VEC_OVI11    = 187,
    SH705X_VEC_CMTI0    = 188,
    SH705X_VEC_ADI0     = 190,
    SH705X_VEC_CMTI1    = 192,
    SH705X_VEC_ADI1     = 194,
    SH705X_VEC_ADI2     = 196,
    SH705X_VEC_SCI0     = 200,  /* ERI, RXI, TXI, TEI; +4 per channel */
    SH705X_VEC_HCAN0    = 220,  /* ERS, OVR, RM, SLE */
    SH705X_VEC_WDT_ITI  = 224,
    SH705X_VEC_HCAN1    = 228,
};

typedef struct SH705xState SH705xState;

/* ATU-II: generic up-counter derived from the virtual clock */
typedef struct AtuCounter {
    uint32_t mask;
    uint32_t base;          /* value at base_ns */
    int64_t base_ns;
    int64_t period_ps;      /* picoseconds per count, 0 = stopped */
    uint32_t last;          /* value at the previous sweep */
} AtuCounter;

typedef struct SH705xATU {
    SH705xState *soc;
    QEMUTimer *timer;
    int64_t last_sweep_ns;

    uint8_t tstr[3];
    uint8_t pscr[4];

    /* channel 0 */
    AtuCounter cnt0;
    uint32_t icr0[4];
    uint8_t itvrr1, itvrr2a, itvrr2b, tior0;
    uint16_t tsr0, tier0;

    /* channels 1 and 2 */
    struct {
        AtuCounter cnta, cntb;
        uint16_t gr[8];
        uint16_t ocr[8];        /* ch1 uses ocr[0] only */
        uint16_t osbr;
        uint8_t tior[4];        /* TIORA..TIORD: [ioB|ioA], ... */
        uint8_t tcra, tcrb;
        uint16_t tsra, tsrb, tiera, tierb;
        uint8_t trgmdr;
    } ch12[2];

    /* channels 3, 4, 5 */
    struct {
        AtuCounter cnt;
        uint16_t gr[4];
        uint8_t tiora, tiorb, tcr;
    } ch345[3];
    uint16_t tsr3, tier3;
    uint8_t tmdr;

    /* channels 6, 7 (PWM) */
    struct {
        AtuCounter cnt[4];
        uint16_t cylr[4], bfr[4], dtr[4];
        uint8_t tcra, tcrb;
        uint16_t tsr, tier;
        uint8_t pmdr;
        int out[4];
    } ch67[2];

    /* channel 8 (one-shot pulse) */
    struct {
        uint16_t dcnt[16];      /* value when stopped */
        int64_t start_ns[16];
        uint16_t start_val[16];
        uint16_t rldr, tcnr, otr, dstr, tsr, tier;
        uint8_t tcr, rldenr;
    } ch8;

    /* channel 9 (event counters) */
    struct {
        uint8_t ecnt[6], gr[6];
        uint8_t tcra, tcrb, tcrc;
        uint16_t tsr, tier;
    } ch9;

    /* channel 10 (crank angle) */
    struct {
        AtuCounter cnta;
        uint8_t tcntb;
        uint16_t tcntc, tcnte, tcntf, tcntg;
        uint8_t tcntd, tcnth;
        uint32_t icra, ocra;
        uint8_t ocrb, ncr, tior, tcr;
        uint16_t rldc, grg, tcclr, tsr, tier;
        uint32_t last_edge_cnt;
        int64_t last_edge_ns;
        int64_t tooth_ns;
    } ch10;

    /* channel 11 */
    struct {
        AtuCounter cnt;
        uint16_t gra, grb;
        uint8_t tior, tcr;
        uint16_t tsr, tier;
    } ch11;

    /* input pin edge tracking */
    int pin_level[64];
} SH705xATU;

typedef struct SH705xSCI {
    SH705xState *soc;
    int index;
    CharFrontend chr;
    uint8_t smr, brr, scr, tdr, ssr, rdr, sdcr;
    bool tx_busy;
    uint8_t tx_shift;
    QEMUTimer *tx_timer;
    QEMUTimer *rx_timer;
    uint8_t rx_fifo[256];
    int rx_head, rx_count;
    bool kline_echo;
} SH705xSCI;

typedef struct SH705xADC {
    SH705xState *soc;
    int index;
    int first_chan;
    int nchan;
    uint16_t addr[12];
    uint8_t adcsr, adcr, adtrgr;
    QEMUTimer *timer;
    int cur;                /* channel being converted (module relative) */
} SH705xADC;

typedef struct SH705xHCAN {
    SH705xState *soc;
    int index;
    bool v2;                /* HCAN2 (SH7058) vs HCAN (SH705[245]) */
    int nmb;
    int vec_base;
    CanBusClientState bus_client;
    CanBusState *canbus;
    /* common */
    uint16_t mcr, gsr, irr, imr;
    uint8_t tec, rec;
    uint32_t txpr, txcr, txack, aback, rxpr, rfpr, mbimr, umsr;
    /* HCAN (v1) */
    uint16_t bcr, mbcr, lafml, lafmh;
    uint8_t mc[16][8], md[16][8];
    /* HCAN2 */
    uint16_t bcr1, bcr0;
    uint8_t mb[SH705X_HCAN_MB_MAX][0x20];
    uint8_t timer_regs[0x20];
} SH705xHCAN;

typedef struct SH705xCMT {
    uint16_t cmstr;
    struct {
        uint16_t cmcsr, cmcor;
        AtuCounter cnt;
    } ch[2];
    QEMUTimer *timer;
} SH705xCMT;

typedef struct SH705xWDT {
    uint8_t tcsr, tcnt, rstcsr;
    AtuCounter cnt;
    QEMUTimer *timer;
} SH705xWDT;

/*
 * On-chip flash memory controller (sh705x_flash.c).  Two generations:
 * 350 nm parts (SH7052/SH7054/SH7055F) are programmed by the CPU through
 * FLMCR1/FLMCR2/EBR1/EBR2; 180 nm parts (SH7055S, SH7058) use the
 * "download" method (FCCS/FPCS/FECS/FKEY/FMATS/FTDAR).
 */
#define SH705X_FLASH_MAX_BLOCKS 16
#define SH705X_FLASH_LATCH      128     /* program unit, bytes */

typedef struct SH705xFlash {
    SH705xState *soc;
    bool is_180nm;
    uint8_t *mem;               /* host pointer to the flash array */
    uint32_t size;
    const uint32_t *blocks;     /* erase block start offsets + end */
    int nblocks;                /* 0: erase/program not modelled */
    uint32_t flmcr2_begin;      /* 350 nm: first byte under FLMCR2 */

    /* 350 nm */
    uint8_t flmcr[2];
    uint8_t ebr[2];
    uint8_t ebr2_mask;
    uint8_t latch[SH705X_FLASH_LATCH];
    int64_t latch_base;         /* -1: program latches empty */

    /* 180 nm */
    uint8_t fccs, fpcs, fecs, fkey, fmats, ftdar;
    uint32_t dl_base[2];        /* download address, [0] write, [1] erase */
    bool dl_valid[2];
    bool init_done[2];
    uint32_t arg[2];            /* emulated microcode interface */
    uint32_t cmd;
    uint32_t fpfr;

    uint16_t ramer;
} SH705xFlash;

typedef struct SH705xPort {
    const char *name;       /* "A" */
    uint16_t dr, ior;
    uint16_t pins;          /* mask of implemented pins */
    EcuPin *pin[16];
} SH705xPort;

struct SH705xState {
    SysBusDevice parent_obj;

    /* properties */
    uint32_t variant;
    uint32_t pclk_hz;
    uint32_t avref_mv;
    bool wdt_reset;
    bool kline_echo;
    CanBusState *canbus[SH705X_NUM_HCAN];

    SuperHCPU *cpu;
    MemoryRegion rom;
    MemoryRegion ram;
    MemoryRegion io;
    uint32_t rom_size;
    uint32_t ram_base, ram_size;

    /* INTC */
    uint16_t ipr[12];
    uint16_t icr, isr;
    uint8_t pending[SH705X_NUM_VECTORS];
    int8_t vec_ipr[SH705X_NUM_VECTORS];     /* ipr nibble index, -1 none */
    int irq_level[8];
    int nmi_level;

    SH705xATU atu;
    SH705xSCI sci[SH705X_NUM_SCI];
    SH705xADC adc[SH705X_NUM_ADC];
    int nadc;
    SH705xHCAN hcan[SH705X_NUM_HCAN];
    int nhcan;
    SH705xCMT cmt;
    SH705xWDT wdt;
    SH705xPort port[SH705X_NUM_PORTS];
    SH705xFlash flash;
    uint32_t flash_node;        /* property: 0 (variant default), 180, 350 */

    /* plain storage for registers without a behavioural model */
    uint8_t regs[SH705X_IO_SIZE];
};

/* sh705x.c */
void sh705x_set_irq(SH705xState *s, int vec, bool level);
int64_t sh705x_now(void);
void sh705x_adc_trigger(SH705xState *s);

/* counter helpers */
void atu_cnt_init(AtuCounter *c, uint32_t mask);
uint32_t atu_cnt_get(AtuCounter *c, int64_t now);
void atu_cnt_set(AtuCounter *c, int64_t now, uint32_t val);
void atu_cnt_set_period(AtuCounter *c, int64_t now, int64_t period_ps);
int64_t atu_cnt_time_to(AtuCounter *c, int64_t now, uint32_t target);
bool atu_cnt_passed(AtuCounter *c, uint32_t from, uint32_t to, uint32_t v);

/* sh705x_atu.c */
void sh705x_atu_init(SH705xState *s);
void sh705x_atu_reset(SH705xATU *a);
uint16_t sh705x_atu_read(SH705xATU *a, uint32_t addr);
void sh705x_atu_write(SH705xATU *a, uint32_t addr, uint16_t val,
                      uint16_t mask);

/* sh705x_hcan.c */
void sh705x_hcan_init(SH705xHCAN *h, SH705xState *s, int index, bool v2,
                      int nmb, int vec_base);
void sh705x_hcan_reset(SH705xHCAN *h);
uint16_t sh705x_hcan_read(SH705xHCAN *h, uint32_t off);
void sh705x_hcan_write(SH705xHCAN *h, uint32_t off, uint16_t val,
                       uint16_t mask);

/* sh705x_flash.c */
bool sh705x_flash_in_range(uint32_t addr);
bool sh705x_flash_init(SH705xFlash *f, SH705xState *s, Error **errp);
void sh705x_flash_reset(SH705xFlash *f);
bool sh705x_flash_load(SH705xState *s, const char *filename);
uint16_t sh705x_flash_read16(SH705xFlash *f, uint32_t addr);
void sh705x_flash_write16(SH705xFlash *f, uint32_t addr, uint16_t val,
                          uint16_t mask);

static inline uint16_t sh705x_merge(uint16_t old, uint16_t val, uint16_t mask)
{
    return (old & ~mask) | (val & mask);
}

/* Write-0-to-clear status flags: only bits written as 0 are cleared. */
static inline uint16_t sh705x_w0c(uint16_t old, uint16_t val, uint16_t mask)
{
    return old & (val | ~mask);
}

#endif
