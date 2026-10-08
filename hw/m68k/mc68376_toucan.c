/*
 * MC68376 TouCAN (CAN 2.0B controller, section 13) -- placeholder stub.
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

struct MC68376TouCAN {
    MC68376State *soc;
};

static uint64_t toucan_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.toucan: unimplemented read%u @ 0x%03"
                  HWADDR_PRIx "\n", size * 8, addr);
    return 0;
}

static void toucan_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "mc68376.toucan: unimplemented write%u @ 0x%03"
                  HWADDR_PRIx " = 0x%" PRIx64 "\n", size * 8, addr, val);
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
    memory_region_init_io(mr, OBJECT(s), &toucan_ops, m, "mc68376.toucan",
                          MC68376_TOUCAN_SIZE);
}

void mc68376_toucan_reset(MC68376State *s)
{
}
