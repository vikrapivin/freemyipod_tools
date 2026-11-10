/* SPDX-License-Identifier: GPL-2.0-or-later
 * emCORE services for the copied Rockbox physical NAND driver. */
#ifndef N4G_ROCKBOX_COMPAT_H
#define N4G_ROCKBOX_COMPAT_H
#include "global.h"
#include "timer.h"
#include "clockgates.h"
#include "mmu.h"
#include "console.h"
#include "rockbox-fmc.h"
#include "../nand_hal_emcore.h"

#define CLOCKGATE_NAND 5
#define CLOCKGATE_NANDECC 9
#define current_tick (read_usec_timer() / 10000u)
#define logf(...) cprintf(CONSOLE_BOOT, __VA_ARGS__)
static inline void commit_dcache_range(const void *p, uint32_t n)
{ clean_dcache_range((uint32_t)p, n); }
static inline void discard_dcache_range(void *p, uint32_t n)
{ invalidate_dcache_range((uint32_t)p, n); }
void commit_discard_dcache_range(void *p, uint32_t n);
int lcd_wait_write_dma(void);
#include "fmss-controller.h"
#endif
