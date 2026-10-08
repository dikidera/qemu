/*
 * MC68376 QADC (queued A/D converter, section 8) -- placeholder stub.
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

struct MC68376QADC {
    MC68376State *soc;
};

static uint64_t qadc_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.qadc: unimplemented read%u @ 0x%03"
                  HWADDR_PRIx "\n", size * 8, addr);
    return 0;
}

static void qadc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.qadc: unimplemented write%u @ 0x%03"
                  HWADDR_PRIx " = 0x%" PRIx64 "\n", size * 8, addr, val);
}

static const MemoryRegionOps qadc_ops = {
    .read = qadc_read,
    .write = qadc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

void mc68376_qadc_init(MC68376State *s, MemoryRegion *mr, Error **errp)
{
    MC68376QADC *m = g_new0(MC68376QADC, 1);

    m->soc = s;
    s->qadc = m;
    memory_region_init_io(mr, OBJECT(s), &qadc_ops, m, "mc68376.qadc",
                          MC68376_QADC_SIZE);
}

void mc68376_qadc_reset(MC68376State *s)
{
}
