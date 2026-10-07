/*
 * Renesas M32C/80 CPU parameters
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef M32C_CPU_PARAM_H
#define M32C_CPU_PARAM_H

/* 16 MiB address space; SFRs live in the first KiB next to RAM */
#define TARGET_PAGE_BITS 10
#define TARGET_VIRT_ADDR_SPACE_BITS 32

#endif
