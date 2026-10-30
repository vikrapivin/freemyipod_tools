/*
 * Copyright (C) 2026 Vikotr Krapivin
 *
 * This file is part of emCORE.
 *
 * emCORE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * emCORE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with emCORE. If not, see <http://www.gnu.org/licenses/>.
 */

/* Mapping of the Rockbox kernel API used by the ported nano 3G NAND driver
 * (nand-swnand.c) and FTL (ftl-nano3g.c) onto emCORE. */
#ifndef __NAND_COMPAT_H__
#define __NAND_COMPAT_H__

#include "global.h"
#include "panic.h"

#define STORAGE_ALIGN_ATTR __attribute__((aligned(32)))
#define logf(...) ((void)0)
/* An unrecoverable FTL/VFL write failure: stop the NAND worker thread only */
#define NAND_PANIC(...) panicf(PANIC_KILLTHREAD, __VA_ARGS__)

/* ARM926EJ-S data cache, 32-byte lines. The controller reads and writes
 * buffers behind the cache's back. */
#define NAND_DLINE 32u

static inline void nand_cp15_dsb(void)
{
    __asm__ volatile("mcr p15, 0, %0, c7, c10, 4" : : "r"(0) : "memory");
}

/* write dirty lines back */
static inline void commit_dcache_range(const void *addr, uint32_t len)
{
    uint32_t a = (uint32_t)addr & ~(NAND_DLINE - 1u), e = (uint32_t)addr + len;
    for (; a < e; a += NAND_DLINE)
        __asm__ volatile("mcr p15, 0, %0, c7, c10, 1" : : "r"(a) : "memory");
    nand_cp15_dsb();
}

/* write back and invalidate */
static inline void commit_discard_dcache_range(const void *addr, uint32_t len)
{
    uint32_t a = (uint32_t)addr & ~(NAND_DLINE - 1u), e = (uint32_t)addr + len;
    for (; a < e; a += NAND_DLINE)
        __asm__ volatile("mcr p15, 0, %0, c7, c14, 1" : : "r"(a) : "memory");
    nand_cp15_dsb();
}

/* invalidate (edge lines are cleaned first, so neighbours survive) */
static inline void discard_dcache_range(const void *addr, uint32_t len)
{
    uint32_t a = (uint32_t)addr & ~(NAND_DLINE - 1u), e = (uint32_t)addr + len;
    for (; a < e; a += NAND_DLINE)
        __asm__ volatile("mcr p15, 0, %0, c7, c6, 1" : : "r"(a) : "memory");
    nand_cp15_dsb();
}

#endif
