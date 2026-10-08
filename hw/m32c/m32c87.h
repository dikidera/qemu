/*
 * Renesas M32C/87 microcontroller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_M32C_M32C87_H
#define HW_M32C_M32C87_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "net/can_emu.h"
#include "target/m32c/cpu.h"
#include "hw/ecu/ecu.h"

#define TYPE_M32C87_SOC "m32c87-soc"
OBJECT_DECLARE_SIMPLE_TYPE(M32C87State, M32C87_SOC)

#define M32C87_SFR_SIZE     0x400
#define M32C87_NUM_UART     5
#define M32C87_NUM_TA       5
#define M32C87_NUM_TB       6
#define M32C87_NUM_PORTS    16
#define M32C87_NUM_VEC      64
#define M32C87_NUM_CAN      2
#define M32C87_CAN_SLOTS    16

/* software interrupt numbers (variable vector table) */
enum {
    M32C87_VEC_TA0 = 12,        /* TA0..TA4 = 12..16 */
    M32C87_VEC_S0T = 17, M32C87_VEC_S0R = 18,
    M32C87_VEC_S1T = 19, M32C87_VEC_S1R = 20,
    M32C87_VEC_TB0 = 21,        /* TB0..TB4 = 21..25 */
    M32C87_VEC_INT5 = 26,       /* INT5..INT0 = 26..31 */
    M32C87_VEC_TB5 = 32,
    M32C87_VEC_S2T = 33, M32C87_VEC_S2R = 34,
    M32C87_VEC_S3T = 35, M32C87_VEC_S3R = 36,
    M32C87_VEC_S4T = 37, M32C87_VEC_S4R = 38,
    M32C87_VEC_AD0 = 42,
    M32C87_VEC_KEY = 43,
};

typedef struct M32CCounter {
    uint32_t base;
    int64_t base_ns;
    int64_t period_ps;
} M32CCounter;

typedef struct M32C87Timer {
    bool is_b;
    int index;
    uint8_t mr;
    uint16_t reload;
    uint16_t value;         /* count when stopped */
    int64_t start_ns;       /* start of the current period */
    int64_t deadline_ns;    /* time of the next scheduled event */
    int64_t period_ps;      /* one count */
    bool running;
    bool one_shot_active;
    bool pwm_high;
    int out;
    QEMUTimer *timer;
    EcuPin *out_pin;
    EcuPin *in_pin;
    int in_level;
    /* timer B measurement */
    int64_t last_edge_ns;
    uint16_t measured;
} M32C87Timer;

typedef struct M32C87UART {
    struct M32C87State *soc;
    int index;
    uint16_t base;
    CharFrontend chr;
    uint8_t mr, brg, c0, c1, smr[4];
    uint16_t tb, rb;
    bool tx_busy;
    QEMUTimer *tx_timer;
    QEMUTimer *rx_timer;
    uint8_t rx_fifo[256];
    int rx_head, rx_count;
    int vec_tx, vec_rx;
} M32C87UART;

typedef struct M32C87CAN {
    struct M32C87State *soc;
    int index;
    CanBusClientState bus_client;
    CanBusState *canbus;
    uint16_t ctlr0, str, idr, conr, tsr, sistr, simkr, ssctlr, ssstr, afs;
    uint8_t ctlr1, slpr, sbs, recr, tecr, eimkr, eistr, efr, mdr;
    uint8_t mctl[M32C87_CAN_SLOTS];
    uint8_t slot[M32C87_CAN_SLOTS][16];
    uint8_t gmr[5], lmar[5], lmbr[5];
    int irq_slot, irq_err;      /* CAN interrupt numbers 0..5 */
} M32C87CAN;

typedef struct M32C87State {
    SysBusDevice parent_obj;

    uint32_t pclk_hz;
    uint32_t rom_size;
    uint32_t ram_size;
    uint32_t avref_mv;
    bool wdt_reset;
    bool kline_echo;
    uint32_t can_irq[M32C87_NUM_CAN][2];
    CanBusState *canbus[M32C87_NUM_CAN];

    M32CCPU *cpu;
    MemoryRegion rom;
    MemoryRegion ram;
    MemoryRegion sfr;

    uint8_t regs[M32C87_SFR_SIZE];
    int16_t vec_icr[M32C87_NUM_VEC];    /* SFR address of the ICR */

    M32C87Timer ta[M32C87_NUM_TA];
    M32C87Timer tb[M32C87_NUM_TB];
    M32C87UART uart[M32C87_NUM_UART];
    M32C87CAN can[M32C87_NUM_CAN];

    /* A/D0 */
    uint16_t ad[8];
    QEMUTimer *ad_timer;
    int ad_cur;

    /* ports */
    EcuPin *port_pin[M32C87_NUM_PORTS][8];

    /* INT pins */
    int int_level[6];
    int key_level[4];               /* KI0..KI3 = P10_4..P10_7 */
    int nmi_level;
    bool nmi_pending;

    /* watchdog */
    QEMUTimer *wdt_timer;
    bool wdt_running;
    int64_t wdt_start_ns;
    bool wdt_pending;
} M32C87State;

void m32c87_set_ir(M32C87State *s, int vec);
int64_t m32c87_now(void);

#endif
