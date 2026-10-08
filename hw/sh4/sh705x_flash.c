/*
 * Renesas SH705x on-chip flash memory: array and programming controller.
 *
 * The flash array is a ROM device: reads (and instruction fetches) go
 * straight to the host buffer, CPU writes reach the controller, which only
 * acts on them when a programming sequence allows it.
 *
 * Two controller generations are modelled.  Register addresses come from
 * the Renesas iodefine headers (nissutils renesas_includes, npkern
 * reg_defines); the sequences the model accepts are the ones the npkern
 * reflash kernel (tested on real ECUs) uses.
 *
 * 350 nm parts (SH7052, SH7054, SH7055F), npkern pl_flash_7055_350nm.c:
 *   FLMCR1 (H'FFFFE800): FWE SWE1 ESU1 PSU1 EV1 PV1 E1 P1, controls the
 *                        flash below H'40000
 *   FLMCR2 (H'FFFFE801): FLER SWE2 ESU2 PSU2 EV2 PV2 E2 P2, controls the
 *                        flash from H'40000
 *   EBR1/EBR2 (H'FFFFE802/3): erase block select EB0..EB7 / EB8..EB15
 *   Programming: with SWE set, byte writes to a 128-byte unit of flash
 *   load the program latches; setting P while PSU is set programs the
 *   latched zero bits.  Erase: setting E while ESU is set erases the blocks
 *   selected in EBR1/EBR2.  In program-/erase-verify mode (PV/EV) the
 *   dummy writes are ignored and reads return the array contents.  Pulse
 *   lengths are not checked: one pulse programs/erases completely.
 *
 * 180 nm parts (SH7055S, SH7058), npkern pl_flash_705x_180nm.c:
 *   FCCS (H'FFFFE800) FWE . . FLER . . . SCO, FPCS (E801) PPVS,
 *   FECS (E802) EPVB, FKEY (E804), FMATS (E805), FTDAR (E806) TDER TDA[6:0]
 *   Writing SCO = 1 "downloads" the on-chip program selected by PPVS
 *   (write) or EPVB (erase) to on-chip RAM at RAM start + TDA * 2 KiB and
 *   stores the result in the first byte there (DPFR).  The program is then
 *   initialised by calling download address + 32 (R4 = FPEFEQ, R5 = FUBRA)
 *   and run by calling download address + 16 (write: R4 = FMPDR source,
 *   R5 = FMPAR flash destination, 128 bytes; erase: R4 = FEBS block
 *   number) with FKEY = H'5A; both return FPFR in R0 (0 = success).
 *
 *   The real downloaded programs are Renesas internal and not available.
 *   They are emulated behind the same calling interface: the download
 *   writes a small SH-2 stub at download address + 16 / + 32 which stores
 *   R4/R5 and a command code to private registers at H'FFFFE810..E81B
 *   (unused on the real chip) and loads the result from H'FFFFE81C.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/loader.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/sh4/sh705x.h"

#define FLASH_REG_BASE      0xFFFFE800u
#define FLASH_REG_END       0xFFFFEC00u
#define FLASH_RAMER         0xFFFFEC26u

/* FLMCR1/FLMCR2 bits (350 nm) */
#define FLMCR_FWE           0x80    /* FLMCR1: FWE pin level */
#define FLMCR_FLER          0x80    /* FLMCR2: flash error */
#define FLMCR_SWE           0x40
#define FLMCR_ESU           0x20
#define FLMCR_PSU           0x10
#define FLMCR_EV            0x08
#define FLMCR_PV            0x04
#define FLMCR_E             0x02
#define FLMCR_P             0x01

/* 180 nm registers (offsets from H'FFFFE800) and bits */
#define R_FCCS              0
#define R_FPCS              1
#define R_FECS              2
#define R_FKEY              4
#define R_FMATS             5
#define R_FTDAR             6
#define FCCS_FWE            0x80
#define FCCS_FLER           0x10
#define FCCS_SCO            0x01
#define FPCS_PPVS           0x01
#define FECS_EPVB           0x01
#define FTDAR_TDER          0x80
#define FTDAR_TDA           0x7f
#define FKEY_DOWNLOAD       0xa5
#define FKEY_PROGRAM        0x5a
#define DL_AREA_SIZE        0x800   /* FTDAR step: 2 KiB */

/* DPFR (first byte of the download area) */
#define DPFR_SF             0x01
#define DPFR_FK             0x02
#define DPFR_SS             0x04

/*
 * FPFR returned by the emulated programs.  npkern only relies on 0 =
 * success and reports bits 1..2; the bit assignment below follows the
 * usual Renesas download-method layout and is not verified against the
 * SH7058 manual.
 */
#define FPFR_SF             0x01    /* failed */
#define FPFR_WA             0x02    /* write: bad destination (FMPAR) */
#define FPFR_FQ             0x04    /* init: bad frequency (FPEFEQ) */
#define FPFR_WD             0x04    /* write: bad source (FMPDR) */
#define FPFR_EB             0x08    /* erase: bad block number (FEBS) */
#define FPFR_FK             0x10    /* FKEY was not H'5A */

/* FPEFEQ: operating frequency in units of 10 kHz, npkern passes 40 * 100 */
#define FPEFEQ_MAX          4000

/* Private interface used by the emulated download programs */
#define R_PRIV_ARG0         0x10    /* R4 */
#define R_PRIV_ARG1         0x14    /* R5 */
#define R_PRIV_CMD          0x18
#define R_PRIV_RESULT       0x1c
#define PRIV_ENTRY_PROCESS  0x01
#define PRIV_ENTRY_INIT     0x02
#define PROG_WRITE          0
#define PROG_ERASE          1
#define PRIV_CMD(prog, entry) ((((prog) + 1) << 4) | (entry))

/* Erase block start offsets (+ end delimiter), from npkern's fblocks[] */
static const uint32_t blocks_7058[] = {
    0x00000, 0x01000, 0x02000, 0x03000, 0x04000, 0x05000, 0x06000, 0x07000,
    0x08000, 0x20000, 0x40000, 0x60000, 0x80000, 0xA0000, 0xC0000, 0xE0000,
    0x100000,
};

/*
 * SH7055 (both generations).  SH7054 uses the first 14 blocks: npkern
 * builds its 7054 kernel with this table, and the SH7054 EBR2 only has
 * EB8..EB13, which ends exactly at the end of its 384 KiB.
 */
static const uint32_t blocks_7055[] = {
    0x00000, 0x01000, 0x02000, 0x03000, 0x04000, 0x05000, 0x06000, 0x07000,
    0x08000, 0x10000, 0x20000, 0x30000, 0x40000, 0x50000, 0x60000, 0x70000,
    0x80000,
};

static bool flash_ram_range(SH705xFlash *f, uint32_t addr, uint32_t len)
{
    SH705xState *s = f->soc;

    return addr >= s->ram_base && len <= s->ram_size &&
           addr - s->ram_base <= s->ram_size - len;
}

static void flash_changed(SH705xFlash *f, uint32_t off, uint32_t len)
{
    memory_region_flush_rom_device(&f->soc->rom, off, len);
}

/* ---------------------------------------------------------------------- */
/* 350 nm: FLMCR1/FLMCR2                                                  */
/* ---------------------------------------------------------------------- */

static int flash_region_of(SH705xFlash *f, uint32_t off)
{
    return off >= f->flmcr2_begin ? 1 : 0;
}

static void flash_latch_clear(SH705xFlash *f)
{
    memset(f->latch, 0xff, sizeof(f->latch));
    f->latch_base = -1;
}

static void flash_program_pulse(SH705xFlash *f, int r)
{
    uint32_t base;

    if (f->latch_base < 0) {
        return;
    }
    base = f->latch_base;
    if (flash_region_of(f, base) != r) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sh705x-flash: program pulse in FLMCR%d but the "
                      "latched data is at 0x%05x\n", r + 1, base);
        return;
    }
    for (int i = 0; i < SH705X_FLASH_LATCH; i++) {
        f->mem[base + i] &= f->latch[i];
    }
    flash_changed(f, base, SH705X_FLASH_LATCH);
    flash_latch_clear(f);
}

static void flash_erase_pulse(SH705xFlash *f, int r)
{
    uint16_t sel = f->ebr[0] | (f->ebr[1] << 8);

    for (int b = 0; b < f->nblocks; b++) {
        uint32_t start = f->blocks[b], len = f->blocks[b + 1] - start;

        if (!(sel & (1 << b))) {
            continue;
        }
        if (flash_region_of(f, start) != r) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "sh705x-flash: erase pulse in FLMCR%d ignores "
                          "EB%d\n", r + 1, b);
            continue;
        }
        memset(f->mem + start, 0xff, len);
        flash_changed(f, start, len);
    }
}

static void flash_flmcr_write(SH705xFlash *f, int r, uint8_t val)
{
    uint8_t old = f->flmcr[r];
    uint8_t rise;

    if (r == 1 && f->flmcr2_begin >= f->size) {
        /* FLMCR2 has only FLER on parts without a second flash area */
        return;
    }
    val &= 0x7f;
    f->flmcr[r] = val;
    rise = val & ~old;

    if (!(val & FLMCR_SWE)) {
        if (old & FLMCR_SWE) {
            flash_latch_clear(f);
        }
        return;
    }
    if (!f->nblocks && (rise & (FLMCR_P | FLMCR_E))) {
        qemu_log_mask(LOG_UNIMP, "sh705x-flash: programming/erasing is not "
                      "modelled on this variant\n");
        return;
    }
    if ((rise & FLMCR_P) && (val & FLMCR_PSU)) {
        flash_program_pulse(f, r);
    }
    if ((rise & FLMCR_E) && (val & FLMCR_ESU)) {
        flash_erase_pulse(f, r);
    }
}

/* Returns false if the access is not a program latch write. */
static bool flash_array_write_350(SH705xFlash *f, uint32_t off, uint8_t val)
{
    uint8_t flmcr = f->flmcr[flash_region_of(f, off)];
    uint32_t base = off & ~(SH705X_FLASH_LATCH - 1);

    if (!(flmcr & FLMCR_SWE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: write to flash 0x%05x "
                      "without SWE ignored\n", off);
        return false;
    }
    if (flmcr & (FLMCR_PV | FLMCR_EV)) {
        return false;           /* verify mode dummy write */
    }
    if (flmcr & (FLMCR_PSU | FLMCR_P | FLMCR_ESU | FLMCR_E)) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: write to flash 0x%05x "
                      "during program/erase setup ignored\n", off);
        return false;
    }
    if (f->latch_base != base) {
        if (f->latch_base >= 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: latched data for "
                          "0x%05x discarded\n", (uint32_t)f->latch_base);
        }
        flash_latch_clear(f);
        f->latch_base = base;
    }
    f->latch[off - base] = val;
    return true;
}

/* ---------------------------------------------------------------------- */
/* 180 nm: download method                                                */
/* ---------------------------------------------------------------------- */

/*
 * Stub placed in the download area.  Process entry (+16) and init entry
 * (+32) load their command code into R0 and branch to common code at +48:
 *   mov #-24,r1; shll8 r1           r1 = H'FFFFE800
 *   mov.l r4,@(16,r1); mov.l r5,@(20,r1); mov.l r0,@(24,r1)
 *   mov.l @(28,r1),r0; rts; nop     r0 = FPFR
 */
static void flash_download_stub(uint8_t *buf, int prog)
{
    static const uint16_t common[] = {
        0xE1E8, 0x4118, 0x1144, 0x1155, 0x1106, 0x5017, 0x000B, 0x0009,
    };

    for (int i = 16; i < 48; i += 2) {
        stw_be_p(buf + i, 0x0009);
    }
    stw_be_p(buf + 16, 0xA00E);                                 /* bra +48 */
    stw_be_p(buf + 18, 0xE000 | PRIV_CMD(prog, PRIV_ENTRY_PROCESS));
    stw_be_p(buf + 32, 0xA006);                                 /* bra +48 */
    stw_be_p(buf + 34, 0xE000 | PRIV_CMD(prog, PRIV_ENTRY_INIT));
    for (int i = 0; i < ARRAY_SIZE(common); i++) {
        stw_be_p(buf + 48 + 2 * i, common[i]);
    }
}

static void flash_download(SH705xFlash *f)
{
    SH705xState *s = f->soc;
    uint8_t buf[64];
    uint32_t base = s->ram_base + (f->ftdar & FTDAR_TDA) * DL_AREA_SIZE;
    uint8_t dpfr = 0;
    int prog;

    if (!flash_ram_range(f, base, DL_AREA_SIZE)) {
        /* download address outside on-chip RAM: DPFR left unchanged */
        f->ftdar |= FTDAR_TDER;
        return;
    }
    if (f->fkey != FKEY_DOWNLOAD) {
        dpfr |= DPFR_FK;
    }
    if (!(f->fpcs & FPCS_PPVS) == !(f->fecs & FECS_EPVB)) {
        dpfr |= DPFR_SS;
    }
    if (dpfr) {
        dpfr |= DPFR_SF;
        address_space_write(&address_space_memory, base,
                            MEMTXATTRS_UNSPECIFIED, &dpfr, 1);
        return;
    }

    prog = (f->fpcs & FPCS_PPVS) ? PROG_WRITE : PROG_ERASE;
    if (f->dl_valid[!prog] && f->dl_base[!prog] == base) {
        f->dl_valid[!prog] = false;     /* overwritten */
    }
    f->dl_base[prog] = base;
    f->dl_valid[prog] = true;
    f->init_done[prog] = false;

    address_space_read(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED,
                       buf, sizeof(buf));
    buf[0] = 0;                         /* DPFR: success */
    flash_download_stub(buf, prog);
    address_space_write(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED,
                        buf, sizeof(buf));
}

static uint32_t flash_do_write(SH705xFlash *f)
{
    uint32_t src = f->arg[0], dest = f->arg[1];
    uint8_t buf[SH705X_FLASH_LATCH];

    if (f->fkey != FKEY_PROGRAM) {
        return FPFR_SF | FPFR_FK;
    }
    if ((dest & (SH705X_FLASH_LATCH - 1)) || dest >= f->size) {
        return FPFR_SF | FPFR_WA;
    }
    if (address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                           buf, sizeof(buf)) != MEMTX_OK) {
        return FPFR_SF | FPFR_WD;
    }
    for (int i = 0; i < sizeof(buf); i++) {
        f->mem[dest + i] &= buf[i];
    }
    flash_changed(f, dest, sizeof(buf));
    /* the program verifies: bits that should be 1 but were not erased */
    return memcmp(f->mem + dest, buf, sizeof(buf)) ? FPFR_SF : 0;
}

static uint32_t flash_do_erase(SH705xFlash *f)
{
    uint32_t blk = f->arg[0];
    uint32_t start, len;

    if (f->fkey != FKEY_PROGRAM) {
        return FPFR_SF | FPFR_FK;
    }
    if (blk >= f->nblocks) {
        return FPFR_SF | FPFR_EB;
    }
    start = f->blocks[blk];
    len = f->blocks[blk + 1] - start;
    memset(f->mem + start, 0xff, len);
    flash_changed(f, start, len);
    return 0;
}

static void flash_command(SH705xFlash *f, uint32_t cmd)
{
    int prog = ((cmd >> 4) & 0xf) - 1;
    int entry = cmd & 0xf;

    f->fpfr = FPFR_SF;
    if ((prog != PROG_WRITE && prog != PROG_ERASE) || !f->dl_valid[prog]) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: call of a flash "
                      "program that was not downloaded (0x%x)\n", cmd);
        return;
    }
    if (f->fmats) {
        qemu_log_mask(LOG_UNIMP, "sh705x-flash: FMATS=0x%02x: only the user "
                      "MAT is modelled\n", f->fmats);
    }
    switch (entry) {
    case PRIV_ENTRY_INIT:
        /* R4 = FPEFEQ, R5 = FUBRA (user branch, not modelled) */
        if (f->arg[0] == 0 || f->arg[0] > FPEFEQ_MAX) {
            f->fpfr = FPFR_SF | FPFR_FQ;
            return;
        }
        f->init_done[prog] = true;
        f->fpfr = 0;
        break;
    case PRIV_ENTRY_PROCESS:
        if (!f->init_done[prog]) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: flash program "
                          "called before initialisation\n");
            return;
        }
        f->fpfr = prog == PROG_WRITE ? flash_do_write(f) : flash_do_erase(f);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: bad command 0x%x\n",
                      cmd);
        break;
    }
}

/* ---------------------------------------------------------------------- */
/* Register access                                                        */
/* ---------------------------------------------------------------------- */

static uint8_t flash_readb(SH705xFlash *f, uint32_t off)
{
    if (!f->is_180nm) {
        switch (off) {
        case 0:
            return FLMCR_FWE | f->flmcr[0];     /* FWE pin high */
        case 1:
            return f->flmcr[1];                 /* FLER never set */
        case 2:
        case 3:
            return f->ebr[off - 2];
        default:
            return 0;                           /* FKEY etc. undefined */
        }
    }
    switch (off) {
    case R_FCCS:
        return FCCS_FWE;                        /* FWE pin high, no FLER */
    case R_FPCS:
        return f->fpcs;
    case R_FECS:
        return f->fecs;
    case R_FKEY:
        return f->fkey;
    case R_FMATS:
        return f->fmats;
    case R_FTDAR:
        return f->ftdar;
    case R_PRIV_ARG0 ... R_PRIV_ARG0 + 3:
        return f->arg[0] >> (8 * (3 - (off & 3)));
    case R_PRIV_ARG1 ... R_PRIV_ARG1 + 3:
        return f->arg[1] >> (8 * (3 - (off & 3)));
    case R_PRIV_RESULT ... R_PRIV_RESULT + 3:
        return f->fpfr >> (8 * (3 - (off & 3)));
    default:
        return 0;
    }
}

static void flash_set_byte(uint32_t *reg, uint32_t off, uint8_t val)
{
    int sh = 8 * (3 - (off & 3));

    *reg = (*reg & ~(0xffu << sh)) | ((uint32_t)val << sh);
}

static void flash_writeb(SH705xFlash *f, uint32_t off, uint8_t val)
{
    if (!f->is_180nm) {
        switch (off) {
        case 0:
        case 1:
            flash_flmcr_write(f, off, val);
            break;
        case 2:
            f->ebr[0] = val;
            break;
        case 3:
            f->ebr[1] = val & f->ebr2_mask;
            break;
        default:
            break;
        }
        return;
    }
    switch (off) {
    case R_FCCS:
        if (val & FCCS_SCO) {
            flash_download(f);
        }
        break;
    case R_FPCS:
        f->fpcs = val & FPCS_PPVS;
        break;
    case R_FECS:
        f->fecs = val & FECS_EPVB;
        break;
    case R_FKEY:
        f->fkey = val;
        break;
    case R_FMATS:
        f->fmats = val;
        break;
    case R_FTDAR:
        f->ftdar = val & FTDAR_TDA;
        break;
    case R_PRIV_ARG0 ... R_PRIV_ARG0 + 3:
        flash_set_byte(&f->arg[0], off, val);
        break;
    case R_PRIV_ARG1 ... R_PRIV_ARG1 + 3:
        flash_set_byte(&f->arg[1], off, val);
        break;
    case R_PRIV_CMD ... R_PRIV_CMD + 3:
        flash_set_byte(&f->cmd, off, val);
        if (off == R_PRIV_CMD + 3) {
            flash_command(f, f->cmd);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: write to reserved "
                      "register 0x%08x\n", FLASH_REG_BASE + off);
        break;
    }
}

bool sh705x_flash_in_range(uint32_t addr)
{
    return (addr >= FLASH_REG_BASE && addr < FLASH_REG_END) ||
           (addr & ~1u) == FLASH_RAMER;
}

uint16_t sh705x_flash_read16(SH705xFlash *f, uint32_t addr)
{
    uint32_t off = addr - FLASH_REG_BASE;

    if (addr == FLASH_RAMER) {
        return f->ramer;
    }
    return (flash_readb(f, off) << 8) | flash_readb(f, off + 1);
}

void sh705x_flash_write16(SH705xFlash *f, uint32_t addr, uint16_t val,
                          uint16_t mask)
{
    uint32_t off = addr - FLASH_REG_BASE;

    if (addr == FLASH_RAMER) {
        f->ramer = sh705x_merge(f->ramer, val, mask) & 0x000f;
        if (f->ramer) {
            qemu_log_mask(LOG_UNIMP, "sh705x-flash: RAMER=0x%04x: flash RAM "
                          "emulation is not modelled\n", f->ramer);
        }
        return;
    }
    if (mask & 0xff00) {
        flash_writeb(f, off, val >> 8);
    }
    if (mask & 0x00ff) {
        flash_writeb(f, off + 1, val);
    }
}

/* ---------------------------------------------------------------------- */
/* Flash array                                                            */
/* ---------------------------------------------------------------------- */

static uint64_t flash_array_read(void *opaque, hwaddr off, unsigned size)
{
    SH705xFlash *f = opaque;

    /* only reached outside ROMD mode, which the model never leaves */
    return ldn_be_p(f->mem + off, size);
}

static void flash_array_write(void *opaque, hwaddr off, uint64_t val,
                              unsigned size)
{
    SH705xFlash *f = opaque;

    if (f->is_180nm) {
        qemu_log_mask(LOG_GUEST_ERROR, "sh705x-flash: CPU write to flash "
                      "0x%05x ignored (use the download programs)\n",
                      (uint32_t)off);
        return;
    }
    for (unsigned i = 0; i < size; i++) {
        if (!flash_array_write_350(f, off + i, val >> (8 * (size - 1 - i)))) {
            break;
        }
    }
}

static const MemoryRegionOps flash_array_ops = {
    .read = flash_array_read,
    .write = flash_array_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

void sh705x_flash_reset(SH705xFlash *f)
{
    f->flmcr[0] = f->flmcr[1] = 0;
    f->ebr[0] = f->ebr[1] = 0;
    flash_latch_clear(f);
    f->fpcs = f->fecs = f->fkey = f->fmats = f->ftdar = 0;
    for (int i = 0; i < 2; i++) {
        f->dl_valid[i] = false;
        f->init_done[i] = false;
        f->arg[i] = 0;
    }
    f->cmd = 0;
    f->fpfr = 0;
    f->ramer = 0;
}

bool sh705x_flash_init(SH705xFlash *f, SH705xState *s, Error **errp)
{
    uint32_t node;

    f->soc = s;
    f->size = s->rom_size;
    f->flmcr2_begin = 0x40000;

    switch (s->variant) {
    case SH705X_7052:
        node = 350;
        f->blocks = NULL;       /* block layout not available */
        f->nblocks = 0;
        f->ebr2_mask = 0x3f;    /* EB8..EB13 (7052S.H) */
        break;
    case SH705X_7054:
        node = 350;
        f->blocks = blocks_7055;
        f->nblocks = 14;
        f->ebr2_mask = 0x3f;
        break;
    case SH705X_7055:
        node = s->flash_node ? s->flash_node : 350;
        f->blocks = blocks_7055;
        f->nblocks = 16;
        f->ebr2_mask = 0xff;
        break;
    case SH705X_7058:
    default:
        node = 180;
        f->blocks = blocks_7058;
        f->nblocks = 16;
        f->ebr2_mask = 0xff;
        break;
    }
    if (s->flash_node && s->flash_node != node) {
        error_setg(errp, "flash-node=%u is not supported on this SH705x "
                   "variant (use 180 or 350 on the SH7055)", s->flash_node);
        return false;
    }
    f->is_180nm = node == 180;
    assert(!f->nblocks || f->blocks[f->nblocks] == f->size);

    if (!memory_region_init_rom_device(&s->rom, OBJECT(s), &flash_array_ops,
                                       f, "sh705x.flash", f->size, errp)) {
        return false;
    }
    f->mem = memory_region_get_ram_ptr(&s->rom);
    memset(f->mem, 0xff, f->size);      /* erased */
    flash_latch_clear(f);
    return true;
}

bool sh705x_flash_load(SH705xState *s, const char *filename)
{
    SH705xFlash *f = &s->flash;
    int64_t size = get_image_size(filename, NULL);

    if (size < 0 || size > f->size ||
        load_image_size(filename, f->mem, f->size) != size) {
        return false;
    }
    flash_changed(f, 0, f->size);
    return true;
}
