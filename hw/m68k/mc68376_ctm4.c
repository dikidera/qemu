/*
 * MC68376 CTM4 (configurable timer module 4, section 10) -- placeholder stub.
 *
 * Maps the module's register window, logs every access as unimplemented
 * and reads zero.  To be replaced by the full model.
 *
 * Reference: MC68336/376 User's Manual (MC68336376UM/D).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/m68k/mc68376.h"

struct MC68376CTM4 {
    MC68376State *soc;
};

static uint64_t ctm4_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.ctm4: unimplemented read%u @ 0x%03"
                  HWADDR_PRIx "\n", size * 8, addr);
    return 0;
}

static void ctm4_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.ctm4: unimplemented write%u @ 0x%03"
                  HWADDR_PRIx " = 0x%" PRIx64 "\n", size * 8, addr, val);
}

static const MemoryRegionOps ctm4_ops = {
    .read = ctm4_read,
    .write = ctm4_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

void mc68376_ctm4_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    MC68376CTM4 *m = g_new0(MC68376CTM4, 1);

    m->soc = s;
    s->ctm4 = m;
    memory_region_init_io(mr, OBJECT(s), &ctm4_ops, m, "mc68376.ctm4",
                          MC68376_CTM4_SIZE);
}

void mc68376_ctm4_reset(MC68376State *s)
{
}
