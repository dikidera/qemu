/*
 * Renesas M32C/80 CPU QOM header (target agnostic)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef M32C_CPU_QOM_H
#define M32C_CPU_QOM_H

#include "hw/core/cpu.h"

#define TYPE_M32C_CPU "m32c-cpu"

#define TYPE_M32C80_CPU M32C_CPU_TYPE_NAME("m32c80")

OBJECT_DECLARE_CPU_TYPE(M32CCPU, M32CCPUClass, M32C_CPU)

#define M32C_CPU_TYPE_SUFFIX "-" TYPE_M32C_CPU
#define M32C_CPU_TYPE_NAME(model) model M32C_CPU_TYPE_SUFFIX

#endif
