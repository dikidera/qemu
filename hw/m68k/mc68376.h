/*
 * Motorola MC68376 (CPU32 + SIM, QSM, QADC, CTM4, TPU, TPURAM, SRAM,
 * MRM, TouCAN) ECU microcontroller.
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D, Rev. 15).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_M68K_MC68376_H
#define HW_M68K_MC68376_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "net/can_emu.h"
#include "qemu/notify.h"
#include "target/m68k/cpu.h"
#include "hw/ecu/ecu.h"

#define TYPE_MC68376_SOC "mc68376-soc"
OBJECT_DECLARE_SIMPLE_TYPE(MC68376State, MC68376_SOC)

/*
 * Module control registers live in a 4 KiB block at $7FF000 (SIMCR MM=0)
 * or $FFF000 (MM=1, the reset state).  Offsets of each module inside
 * that block (Table D-1).
 */
#define MC68376_MOD_BLOCK_SIZE  0x1000
#define MC68376_TOUCAN_OFF      0x080   /* 384 bytes */
#define MC68376_QADC_OFF        0x200   /* 512 bytes */
#define MC68376_CTM4_OFF        0x400   /* 256 bytes */
#define MC68376_MRM_OFF         0x820   /* 32 bytes */
#define MC68376_SIM_OFF         0xa00   /* 128 bytes */
#define MC68376_TPURAM_OFF      0xb00   /* 64 bytes */
#define MC68376_SRAM_OFF        0xb40   /* 8 bytes */
#define MC68376_QSM_OFF         0xc00   /* 512 bytes */
#define MC68376_TPU_OFF         0xe00   /* 512 bytes */

#define MC68376_TOUCAN_SIZE     0x180
#define MC68376_QADC_SIZE       0x200
#define MC68376_CTM4_SIZE       0x100
#define MC68376_QSM_SIZE        0x200
#define MC68376_TPU_SIZE        0x200

#define MC68376_ADDR_MASK       0x00ffffff  /* 24-bit external bus */

#define MC68376_SRAM_SIZE       0x1000      /* 4 KiB standby RAM (sec. 6) */
#define MC68376_TPURAM_SIZE     0x0e00      /* 3.5 KiB TPURAM (sec. 12) */
#define MC68376_MRM_SIZE        0x2000      /* 8 KiB masked ROM (sec. 7) */

/* CPU32 exception vector numbers used by the interrupt logic */
#define MC68376_VEC_UNINIT      15      /* uninitialized interrupt */
#define MC68376_VEC_SPURIOUS    24
#define MC68376_VEC_AUTOVEC(n)  (24 + (n))  /* level 1..7 autovector */

/*
 * One interrupt source on the intermodule bus.  A module sets level
 * (1..7, 0 = disabled), the vector it supplies during IACK and its IARB
 * arbitration number (1..15), then calls imb_irq_set().
 *
 * IARB = 0: the request is still seen by the CPU32, but no module wins
 * the arbitration during the interrupt acknowledge cycle, so the CPU
 * takes the spurious interrupt exception (vector 24) (manual 5.2.2,
 * 5.8.4 E.1).  Requests are level sensitive: they stay asserted until
 * the module negates them (usually when software clears the flag).
 *
 * iack (optional) is called when the CPU acknowledges this request and
 * it won the arbitration; sources without a status flag (the PIT, the
 * edge-sensitive IRQ7 pin) negate their request there.  It must not call
 * imb_irq_update() itself (imb_irq_set() from it is fine).
 */
typedef struct IMBIrq {
    struct MC68376State *soc;
    const char *name;
    bool pending;
    uint8_t level;
    uint8_t vector;
    uint8_t iarb;
    void (*iack)(struct IMBIrq *irq);
    void *opaque;
} IMBIrq;

void imb_irq_register(struct MC68376State *s, IMBIrq *irq, const char *name);
void imb_irq_set(IMBIrq *irq, bool pending);
/* re-evaluate after a module changed level/vector/iarb */
void imb_irq_update(struct MC68376State *s);

/* virtual time helpers */
int64_t mc68376_now(void);

/* forward declarations of per-module state (see each module's .c) */
typedef struct MC68376QSM MC68376QSM;
typedef struct MC68376QADC MC68376QADC;
typedef struct MC68376CTM4 MC68376CTM4;
typedef struct MC68376TPU MC68376TPU;
typedef struct MC68376TouCAN MC68376TouCAN;

/*
 * Module interface: each module provides
 *   void mc68376_xxx_init(MC68376State *s, MemoryRegion *mr, Error **errp);
 *     -> initialises *mr (an MMIO region of the module's register size)
 *        and allocates its state, stored in s->xxx
 *   void mc68376_xxx_reset(MC68376State *s);
 * The SoC maps *mr into the module register block.  init is called from
 * the SoC realize (after the CPU exists, so s->cpu is valid), reset from
 * the SoC reset (hold phase), in the order QSM, QADC, CTM4, TPU, TouCAN.
 */
void mc68376_qsm_init(struct MC68376State *s, MemoryRegion *mr, Error **errp);
void mc68376_qsm_reset(struct MC68376State *s);
void mc68376_qadc_init(struct MC68376State *s, MemoryRegion *mr, Error **errp);
void mc68376_qadc_reset(struct MC68376State *s);
void mc68376_ctm4_init(struct MC68376State *s, MemoryRegion *mr, Error **errp);
void mc68376_ctm4_reset(struct MC68376State *s);
void mc68376_tpu_init(struct MC68376State *s, MemoryRegion *mr, Error **errp);
void mc68376_tpu_reset(struct MC68376State *s);
void mc68376_toucan_init(struct MC68376State *s, MemoryRegion *mr,
                         Error **errp);
void mc68376_toucan_reset(struct MC68376State *s);

/* TPU emulation mode needs the TPURAM array (3.5 KiB) */
uint8_t *mc68376_tpuram_ptr(struct MC68376State *s);
bool mc68376_tpuram_emulation(struct MC68376State *s);
/*
 * Called by the TPU when TPUMCR EMU changes.  In emulation mode the
 * TPURAM array is connected to the TPU only: CPU accesses through the IMB
 * are inhibited and the TPURAM control registers have no effect
 * (manual 12.9).
 */
void mc68376_tpuram_set_emulation(struct MC68376State *s, bool on);

/*
 * QADC external trigger inputs, CTM4/TPU driven signals etc. go through
 * named ECU pins (hw/ecu/ecu.h); see each module for the pin names.
 */

/*
 * QSPI peripheral hook.  Called once per completed queue entry in master
 * mode with the peripheral chip-select pattern of the command (PCS[3:0],
 * as driven on the pins), the transmit word and the number of bits; the
 * return value is the word shifted in on MISO (right justified).  With no
 * device attached MISO is assumed to float high (pulled up), so every
 * received bit is one.
 */
typedef uint16_t (*MC68376SPIXferFn)(void *opaque, uint8_t pcs,
                                     uint16_t tx, int bits);
void mc68376_qspi_attach(struct MC68376State *s, MC68376SPIXferFn fn,
                         void *opaque);

/* number of mirror aliases used to decode one chip-select window */
#define MC68376_CS_MIRRORS      16

struct MC68376State {
    SysBusDevice parent_obj;

    /* properties */
    uint32_t extal_hz;          /* PLL reference fref (4.194304 MHz typ.) */
    uint32_t flash_size;        /* external boot flash on CSBOOT */
    uint32_t ext_ram_size;      /* optional external RAM */
    uint32_t ext_ram_base;      /* fixed base when ext_ram_cs < 0 */
    int32_t ext_ram_cs;         /* chip select (0..10) decoding the RAM */
    uint32_t vrh_mv;            /* QADC reference high */
    bool wdt_reset;
    bool kline_echo;
    bool mrm_enabled;           /* DATA14 high at reset: MRM STOP = 0 */
    char *mrm_image;
    char *cpu_type;
    CanBusState *canbus;
    CharFrontend sci_chr;

    M68kCPU *cpu;
    MemoryRegion bus;           /* 16 MiB IMB/external address space */
    AddressSpace bus_as;
    MemoryRegion bus_alias[2];  /* mirrors at $00000000 and $FF000000 */
    MemoryRegion bus_mirror;    /* other mirrors, forwarded to bus_as */
    MemoryRegion modules;       /* 4 KiB module register block */
    MemoryRegion modules_bg;    /* unimplemented parts of the block */
    MemoryRegion flash;
    MemoryRegion ext_ram;
    MemoryRegion flash_win[MC68376_CS_MIRRORS];
    MemoryRegion ext_ram_win[MC68376_CS_MIRRORS];
    MemoryRegion sram;          /* 4 KiB standby RAM array */
    MemoryRegion tpuram;        /* 3.5 KiB TPURAM array */
    MemoryRegion mrm;           /* 8 KiB masked ROM array */

    MemoryRegion sim_mr, sram_ctl_mr, tpuram_ctl_mr, mrm_ctl_mr;
    MemoryRegion qsm_mr, qadc_mr, ctm4_mr, tpu_mr, toucan_mr;

    MC68376QSM *qsm;
    MC68376QADC *qadc;
    MC68376CTM4 *ctm4;
    MC68376TPU *tpu;
    MC68376TouCAN *toucan;

    /* SIM */
    uint16_t simcr, syncr, sypcr, picr, pitr, swsr;
    uint8_t porte, ddre, pepar, portf, ddrf, pfpar, portc;
    uint16_t cspar0, cspar1, csbarbt, csorbt, csbar[11], csor[11];
    uint16_t sigr, simtr, simtre, rsr;
    uint16_t sim_test[6];       /* TSTMSRA..DREG, factory test only */
    bool mm_written, sypcr_written, swsr_armed;
    uint8_t rsr_next;           /* RSR value latched at the next reset */
    QEMUTimer *pit_timer;
    QEMUTimer *swt_timer;
    IMBIrq pit_irq;
    IMBIrq irq_pin[8];          /* IRQ1..7 pins (index 1..7) */
    int irq_pin_level[8];       /* IRQ1..7 pins (index 1..7) */
    int64_t pit_deadline_ns;
    uint8_t pit_modulus;        /* PITM loaded in the modulus counter */
    uint8_t pf_driven;          /* port F pins driven as outputs */
    bool pit_ptp;
    EcuPin *pe_pin[8], *pf_pin[8], *pc_pin[7], *irq_ecu_pin[8];
    NotifierList clock_notifiers;

    /* SRAM / TPURAM / MRM control */
    uint16_t sram_mcr, sram_bar_h, sram_bar_l;
    uint16_t tpuram_mcr, tpuram_bar;
    bool tpuram_bar_written, tpuram_emul;
    uint16_t mrm_mcr, mrm_bah, mrm_bal, mrm_sig[2], mrm_bsw[4];

    /* interrupt sources */
    IMBIrq *irqs[64];
    int nirqs;
    bool in_iack;
};

/* system clock in Hz derived from SYNCR (SIM clock synthesizer) */
uint32_t mc68376_sysclk_hz(struct MC68376State *s);
/*
 * Notified (data = MC68376State *) after SYNCR changes the system clock,
 * so modules can rescale running timers.
 */
void mc68376_add_clock_notifier(struct MC68376State *s, Notifier *n);

/* copy a raw image into the external boot flash (machine helper) */
bool mc68376_flash_load(struct MC68376State *s, const char *filename);

#endif
