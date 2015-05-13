/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

typedef enum {
    VAR_UINT8 = (1 << 0),
    VAR_INT8 = (1 << 1),
    VAR_UINT16 = (1 << 2),
    VAR_INT16 = (1 << 3),
    VAR_UINT32 = (1 << 4),
    VAR_FLOAT = (1 << 5),

    MASTER_VALUE = (1 << 6),
    PROFILE_VALUE = (1 << 7),
    CONTROL_RATE_VALUE = (1 << 8)
} cliValueFlag_e;

#define CONFIG_VALUE_TYPE_MASK (VAR_UINT8 | VAR_INT8 | VAR_UINT16 | VAR_INT16 | VAR_UINT32 | VAR_FLOAT)
#define CONFIG_SECTION_MASK (MASTER_VALUE | PROFILE_VALUE | CONTROL_RATE_VALUE)

typedef struct configValue_t {
    const char *name;
    const uint16_t type; // cliValueFlag_e - specify one of each from CONFIG_VALUE_TYPE_MASK and CONFIG_SECTION_MASK
    void *ptr;
    const int32_t min;
    const int32_t max;
} configValue_t;

void* configVarResolveValuePointer(const configValue_t *value);

extern const configValue_t configVarDefs[];
extern const uint16_t configVarDefCount;
