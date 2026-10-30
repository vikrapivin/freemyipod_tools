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

/* Probe: run one caller-supplied sequencer program and record what the CPU can see (the registers r0..r7 at 0xc18,
 * the PC at 0xc38, the register windows before and after, samples taken while it runs). For learning the engine. */
#define FMSS_PROBE_REQ_MAGIC  0x51525046u   /* "FPRQ" */
#define FMSS_PROBE_OUT_MAGIC  0x4f525046u   /* "FPRO" */
#define FMSS_PROBE_MAXPARAMS  96
#define FMSS_PROBE_MAXSAMPLES 1024
#define FMSS_PROBE_NWIN       5             /* windows 0x000, 0x400, 0x800, 0xc00, 0xd00; 64 words each */
#define FMSS_PROBE_KEEP_CTRL0 (1u << 0)     /* leave FMC_CTRL0 as it is (default: 0, no chip enable, before start) */
#define FMSS_PROBE_WIN400     (1u << 1)     /* also read the 0x400 window (unknown block) */
#define FMSS_PROBE_SAMPLE_REGS (1u << 2)    /* also read r0..r7 while it runs (default: PC and CS_IRQ only, as the driver) */
/* start and end steps, each on by default; a flag skips or changes one (experiments on the sequencer's state) */
#define FMSS_PROBE_NO_CLEAN     (1u << 3)   /* no start-clean of C58 / C4C / C5C before the run */
#define FMSS_PROBE_NO_PREFLIGHT (1u << 4)   /* no END-only program before the run */
#define FMSS_PROBE_NO_RESET     (1u << 5)   /* no sequencer reset before the run */
#define FMSS_PROBE_NO_400       (1u << 6)   /* no FMC +0x400 = 0 before the run */
#define FMSS_PROBE_END_NOCLEAN  (1u << 7)   /* after the run: park without the clean (reset, END-only program) */
#define FMSS_PROBE_END_RESET    (1u << 8)   /* after the run: only a sequencer reset (no park at all) */
#define FMSS_PROBE_WAIT_END   (1u << 14)    /* do not stop at a fault: wait for END (or the timeout); first fault time
                                               in fault_at_us */
#define FMSS_PROBE_PROG_SRAM  (1u << 15)    /* run a copy of the program from internal SRAM (at most
                                               FMSS_PROBE_SRAM_BYTES) instead of from the request buffer */
#define FMSS_PROBE_SRAM_BYTES 4096
#define FMSS_PROBE_FULLMAP    (1u << 17)    /* also read all 1024 words of the window (CPU view) into fullmap[0]
                                               before the start and fullmap[1] after the run (ID FIFO words read 0) */
#define FMSS_PROBE_CH1_PINS   (1u << 16)    /* switch FMC channel 1's pins (the 0x400 set) to the controller, as the boot
                                               ROM's channel-1 NAND boot does; restored after the run */
#define FMSS_PROBE_SNAP_SKIP_SHIFT 9        /* bits 9..13: leave window n (0x000, 0x400, 0x800, 0xc00, 0xd00) out
                                               of the before-snapshot (reads between the reset and the start) */
struct fmss_probe_req {                     /* staged by the host */
    uint32_t magic, version, flags, timeout_us, nparams, prog_bytes;
    uint32_t late_us, nlate;                /* the last nlate of params[] are written once late_us after the start */
    uint32_t start_value;                   /* FMSS_CTRL word that starts the program (0: the driver's 0xfff5) */
    uint32_t late2_us, nlate2;              /* a second batch: the last nlate2 of params[], at late2_us (after the
                                               first batch, which then is the nlate before them) */
    struct { uint32_t off, val; } params[FMSS_PROBE_MAXPARAMS];   /* FMC_BASE + off = val, in order, before start */
};
struct fmss_probe_sample { uint32_t t_us, pc, r[8]; };
struct fmss_probe_out {
    uint32_t magic, version, why, elapsed_us, cs_irq, seq_result, pc, c50;
    uint32_t prog, nsamples, dropped, polls;
    uint32_t pre_ok, pre_us, pre_cs_irq, pre_c00;   /* preflight: the END-only program after the setup */
    uint32_t late_at_us, fault_at_us;       /* when the late writes went out; first CS_IRQ b2 */
    uint32_t irq_seen, irq_end;             /* VIC1 raw status: OR of every poll / at the end (bit 22 is source 54) */
    uint32_t late2_at_us, vic0_seen;        /* VIC0 raw status: OR of every poll */
    uint32_t before[FMSS_PROBE_NWIN][64], after[FMSS_PROBE_NWIN][64];
    struct fmss_probe_sample s[FMSS_PROBE_MAXSAMPLES];
    uint32_t fullmap[2][1024];              /* FMSS_PROBE_FULLMAP: the whole window before the start / after the run */
};
#define FMSS_PROBE_END      1               /* why: END (CS_IRQ b0) */
#define FMSS_PROBE_FAULT    2               /* CS_IRQ b2 */
#define FMSS_PROBE_TIMEOUT  3               /* neither within timeout_us: the sequencer was reset */
/* prog must be 32-byte aligned and outlive the call; leaves the sequencer parked (END-only program, no chip
 * enable). Returns 0, or -1 for a bad request. */
int fmss_probe(const struct fmss_probe_req *q, const uint32_t *prog, struct fmss_probe_out *o);

#endif
