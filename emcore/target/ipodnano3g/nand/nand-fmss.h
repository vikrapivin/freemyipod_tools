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

/* Options of the "fmss" backend (nand-fmss.c) */
#ifndef __NAND_FMSS_H__
#define __NAND_FMSS_H__

/* FMC_CTRL0 bus timing the sequencer programs use. SAFE (the default) is the
 * word the CPU-sequenced "sw" backend runs with (setup 4, hold 3, no EDO),
 * validated on hardware. CHIP derives setup/hold/EDO from the chip table's
 * nanosecond figures and the bus clock given in MHz, by the formula of the
 * original driver; the bus clock of the nano 3G is not established, so this
 * mode is for experiments. Takes effect immediately if a chip is matched,
 * else at the next init. */
#define NAND_FMSS_TIMING_SAFE 0
#define NAND_FMSS_TIMING_CHIP 1
void nand_fmss_set_timing(unsigned int mode, unsigned int bus_mhz);

/* Microseconds fmss_start_wait() sleeps between polls of a running program's end flags, giving the CPU to other
 * threads (default 100); 0 = spin, as the "sw" backend does. Only call the backend from a thread. */
extern uint32_t fmss_poll_us;
/* bus timing override for both program sets: (setup << 16) | (hold << 12) [| EDO << 28], 0 = configured mode */
void nand_fmss_set_raw_timing(uint32_t word);

/* the sequencer's state at the last fault / timeout (see nand-fmss.c fmss_snapshot) */
struct fmss_fault_info {
    uint32_t magic, why, set, prog, pc, r[8], c3c, c50, cs_irq, fmc[20], ecc[4], count;
    uint32_t pc_last, pcs[8], pcs_n[8];     /* PC samples taken while the program ran */
};
/* 1: spin instead of sleeping while a program runs, so the PC samples are dense (diagnosis) */
extern uint32_t fmss_pc_watch;
extern struct fmss_fault_info fmss_fault;

#endif
