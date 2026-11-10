/* SPDX-License-Identifier: GPL-2.0-or-later
 * ARM1176 cache services for the emCORE M2 probe and copied Rockbox driver. */
#include "global.h"
#include "nand_hal_emcore.h"
#define DLINE 32u
#define WHOLE_CACHE_BYTES 0x8000u

static void cp15_dsb(void)
{
    __asm__ volatile("mcr p15, 0, %0, c7, c10, 4" : : "r"(0) : "memory");
}

static void hal_cache_wb(void *addr, uint32_t len)
{
    if (len >= WHOLE_CACHE_BYTES)
        __asm__ volatile("mcr p15, 0, %0, c7, c10, 0" : : "r"(0) : "memory");
    else {
        uint32_t a = (uint32_t)addr & ~(DLINE - 1u), e = (uint32_t)addr + len;
        for (; a < e; a += DLINE) __asm__ volatile("mcr p15, 0, %0, c7, c10, 1" : : "r"(a) : "memory");
    }
    cp15_dsb();
}

static void hal_cache_wb_inv(void *addr, uint32_t len)
{
    if (len >= WHOLE_CACHE_BYTES)
        __asm__ volatile("mcr p15, 0, %0, c7, c14, 0" : : "r"(0) : "memory");
    else {
        uint32_t a = (uint32_t)addr & ~(DLINE - 1u), e = (uint32_t)addr + len;
        for (; a < e; a += DLINE) __asm__ volatile("mcr p15, 0, %0, c7, c14, 1" : : "r"(a) : "memory");
    }
    cp15_dsb();
}

void n4g_emcore_cache_inv(void *addr, uint32_t len)
{
    uint32_t a = (uint32_t)addr & ~(DLINE - 1u), e = (uint32_t)addr + len;
    for (; a < e; a += DLINE) __asm__ volatile("mcr p15, 0, %0, c7, c6, 1" : : "r"(a) : "memory");
    cp15_dsb();
}

const struct n4g_probe_hal g_hal = { hal_cache_wb, hal_cache_wb_inv };
