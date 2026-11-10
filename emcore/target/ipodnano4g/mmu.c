//
//
//    Copyright 2010 TheSeven
//
//
//    This file is part of emCORE.
//
//    emCORE is free software: you can redistribute it and/or
//    modify it under the terms of the GNU General Public License as
//    published by the Free Software Foundation, either version 2 of the
//    License, or (at your option) any later version.
//
//    emCORE is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with emCORE.  If not, see <http://www.gnu.org/licenses/>.
//
//


#include "global.h"
#include "mmu.h"


/* WEDGE-FIX-2026-09-20 (M0.4, bit0): real RANGE-based D-cache ops.
   The legacy clean_dcache()/invalidate_dcache() below are FULL-cache ops
   (c7,c10,0 = clean-all / c7,c14,0 = clean+invalidate-all) and their callers in
   the USB driver (synopsysotg.c) passed a buf/len that was silently ignored.
   Running a full-cache op from inside the USB IRQ handler on EVERY transfer
   (a) costs a long IRQ hold (all cache lines) and (b) discards/rewrites the
   whole working set incl. live USB-stack state -> the accumulated-USB-desync
   wedge class.  These range versions invalidate/clean ONLY the requested
   region (ARMv6, 32-byte D-cache lines):
     MCR p15,0,Rd,c7,c10,1 = D-cache clean line by MVA (ARM1176; was c7,c11,1 = ARMv7-only, fixed Oct 2026)
     MCR p15,0,Rd,c7,c6,1  = D-cache invalidate line by MVA
     MCR p15,0,Rd,c7,c10,4 = DSB / drain write buffer
   New symbols: clean_dcache_range, invalidate_dcache_range (declared in mmu.h). */

void clean_dcache_range(uint32_t addr, uint32_t size)
{
    uint32_t end = addr + size;
    addr &= ~31u;                       /* 32-byte line granularity */
    for (; addr < end; addr += 32u)
        __asm volatile("mcr p15, 0, %0, c7, c10, 1" :: "r"(addr));
    __asm volatile("mcr p15, 0, %0, c7, c10, 4" :: "r"(0));   /* DSB */
}

void invalidate_dcache_range(uint32_t addr, uint32_t size)
{
    uint32_t end = addr + size;
    addr &= ~31u;
    for (; addr < end; addr += 32u)
        __asm volatile("mcr p15, 0, %0, c7, c6, 1" :: "r"(addr));
    __asm volatile("mcr p15, 0, %0, c7, c10, 4" :: "r"(0));   /* DSB */
}


void clean_dcache()
{
    asm volatile(
        "MOV R0, #0                \n\t"
        "MCR p15, 0, R0,c7,c10,0   \n\t"
        "MCR p15, 0, R0,c7,c10,4   \n\t"
        "MOV PC, LR                \n\t"
    );
}

void invalidate_dcache()
{
    asm volatile(
        "MOV R0, #0                \n\t"
        "MCR p15, 0, R0,c7,c14,0   \n\t"
        "MCR p15, 0, R0,c7,c10,4   \n\t"
        "MOV PC, LR                \n\t"
    );
}

void invalidate_icache()
{
    asm volatile(
        "MOV R0, #0                \n\t"
        "MCR p15, 0, R0,c7,c5,0    \n\t"
        "MCR p15, 0, R0,c7,c5,4    \n\t"
        "MOV PC, LR                \n\t"
    );
}
