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

/* M2 probe ABI: request v4, output v14, nano4g-specific magic. */
#ifndef N4G_FMSS_PROBE_H
#define N4G_FMSS_PROBE_H
/* Probe: run one caller-supplied sequencer program and record what the CPU can see (the registers r0..r7 at 0xc18,
 * the PC at 0xc38, the register windows before and after, samples taken while it runs). For learning the engine. */
#define FMSS_PROBE_REQ_MAGIC  0x34525046u   /* "FPRQ" */
#define FMSS_PROBE_OUT_MAGIC  0x3452504fu   /* "FPRO" */
#define FMSS_PROBE_MAXPARAMS  96
#define FMSS_PROBE_MAXSAMPLES 1024
#define FMSS_PROBE_NWIN       4             /* windows 0x000, 0x800, 0xc00, 0xd00; 64 words each */
#define FMSS_PROBE_KEEP_CTRL0 (1u << 0)     /* leave FMC_CTRL0 as it is (default: 0, no chip enable, before start) */
#define FMSS_PROBE_WIN400     (1u << 1)     /* also read the 0x400 window (unknown block) */
#define FMSS_PROBE_SAMPLE_REGS (1u << 2)    /* also read r0..r7 while it runs (default: PC and CS_IRQ only, as the driver) */
/* start and end steps, each on by default; a flag skips or changes one (experiments on the sequencer's state) */
#define FMSS_PROBE_NO_CLEAN     (1u << 3)   /* no start-clean of C58 / C4C before the run */
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
#define FMSS_PROBE_MUTATE     (1u << 18)    /* explicit NAND program/erase experiment */
#define FMSS_PROBE_DISCOVERY  (1u << 19)    /* exploratory opcodes/register sets */
#define FMSS_PROBE_CH1_PINS   (1u << 16)    /* switch FMC channel 1's pins (the 0x400 set) to the controller, as the boot
                                               ROM's channel-1 NAND boot does; restored after the run */
#define FMSS_PROBE_VICSOFT    (1u << 28)    /* params with off = FMSS_PROBE_VIC_PARAM(v, 0x18 | 0x1c) write VIC v's
                                               SOFTINT (lines enabled in VICINTENABLE masked off) or SOFTINTCLEAR;
                                               both controllers' soft lines are cleared after the run */
#define FMSS_PROBE_RELOADED   (1u << 29)    /* capability only: kernel entered through EXECFIRMWARE (n4g.py reload) */
#define FMSS_PROBE_STALL_SAFE (1u << 31)    /* request + capability: during the run the CPU reads no FMSS register (only its
                                               timer) until timeout_us, then cycles FMSS clock gates 5 and 9 before any
                                               access; out.polls = 0, out.dropped = FMSS_PROBE_STALL_MARK. For cases whose
                                               sequencer access may stall the bus (unmapped addresses). No late writes. */
#define FMSS_PROBE_STALL_MARK 0x5afe5afeu
#define FMSS_PROBE_CPU_NAND   (1u << 30)    /* capability only: CPU-direct NAND jobs (NPOC_CMD_CPU_NAND, cpu_nand.h) */
#define FMSS_PROBE_VIC_PARAM(v, reg) (0x80000000u | ((v) << 12) | (reg))
/* Aligned parameter addresses 0x1000..0x3ffffffc write absolute system memory.
 * Round-6 request options. Their settings are params with off = FMSS_PROBE_CFG(id, k) (never written anywhere). */
#define FMSS_PROBE_SAMPLER    (1u << 20)    /* every poll the CPU reads up to 8 host-chosen sources (CFG_SMP k = 0..7)
                                               into s[].r[k], s[].pc = C38, s[].t_us = us since the start; k = 8:
                                               minimum period in us (0: every poll), 9: first sample time (us),
                                               10: mode bit 0 = only when a sampled value changes, bit 1 = also on a
                                               PC change. Sources: aligned FMSS offsets or absolute addresses, or
                                               FMSS_PROBE_VIC_PARAM(v, 0 | 8). Reads retain hardware side effects.
                                               Excludes SAMPLE_REGS, STALL_SAFE */
#define FMSS_PROBE_GATEWIN    (1u << 21)    /* once, at CFG_GATE k = 5 us after the start: IRQs off, the enabled gates
                                               among the PWRCON(k) masks k = 0..4 turned off for k = 6 CPU delay iterations;
                                               FMSS sample offset k = 7 (default C1C), restored exactly, IRQs on;
                                               three samples with pc FMSS_PROBE_GATE_MARK | 1 / | 2 / | 3 (layout in fmss_probe.c) */
#define FMSS_PROBE_EIC        (1u << 22)    /* params FMSS_PROBE_EIC_PARAM(op, reg) set (op 0), clear (op 1) bits of or
                                               write (op 2) GPIO interrupt controller word 0x39700000 + reg (0x80..0xfc);
                                               the EIC words are restored and the touched status bits cleared after the run */
#define FMSS_PROBE_PADS       (1u << 23)    /* run the program from a fresh SDRAM region of CFG_PADS k = 0 bytes whose
                                               rest is filled, per 16 bytes at offset x, with movi r2,#(0xfa000000 |
                                               x >> 4); end. No region: rc -5 (nothing ran). Excludes PROG_SRAM */
#define FMSS_PROBE_CAP_SAMPLER (1u << 12)   /* capability only: FMSS_PROBE_SAMPLER */
#define FMSS_PROBE_CAP_STIMULI (1u << 13)   /* capability only: FMSS_PROBE_GATEWIN, FMSS_PROBE_EIC, FMSS_PROBE_PADS */
#define FMSS_PROBE_CFG(id, k) (0x40000000u | ((id) << 8) | ((k) << 2))
#define FMSS_PROBE_CFG_SMP    1
#define FMSS_PROBE_CFG_GATE   2
#define FMSS_PROBE_CFG_PADS   3
#define FMSS_PROBE_EIC_PARAM(op, reg) (0x90000000u | ((op) << 8) | (reg))
#define FMSS_PROBE_EIC_BASE   0x39700000u
#define FMSS_PROBE_GATE_MARK  0xfeed0000u
#define FMSS_PROBE_PAD_TAG    0xfa000000u
#define FMSS_PROBE_PAD_MAX    0x01100000u   /* largest PADS region: 2^24 + 1 MiB of the 32 MiB SDRAM */
#define FMSS_PROBE_SNAP_SKIP_SHIFT 9        /* bits 9..13: leave window n (0x000, 0x400, 0x800, 0xc00, 0xd00) out
                                               of the before-snapshot (reads between the reset and the start) */
struct fmss_probe_req {                     /* staged by the host */
    uint32_t magic, version, flags, timeout_us, nparams, prog_bytes;
    uint32_t late_us, nlate;                /* the last nlate of params[] are written once late_us after the start */
    uint32_t start_value;                   /* FMSS_CTRL word that starts the program (0: polling word 0xffb5) */
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
    uint32_t irq_seen, irq_end;             /* VIC1 raw status: OR of every poll / at the end; no IRQ assumed */
    uint32_t late2_at_us, vic0_seen;        /* VIC0 raw status: OR of every poll */
    uint32_t before[FMSS_PROBE_NWIN][64], after[FMSS_PROBE_NWIN][64];
    struct fmss_probe_sample s[FMSS_PROBE_MAXSAMPLES];
    uint32_t fullmap[2][1024];              /* FMSS_PROBE_FULLMAP: the whole window before the start / after the run */
};
#define FMSS_PROBE_END      1               /* why: END (CS_IRQ b0) */
#define FMSS_PROBE_FAULT    2               /* CS_IRQ b2 */
#define FMSS_PROBE_TIMEOUT  3               /* neither within timeout_us: the sequencer was reset */
/* Batch output v3 appends reset phases without changing probe v4/v14. */
#include "nand/fmss-controller.h"
struct fmss_reset_diag {
    /* method low byte: actions. Bit24 marks C08 snapshots valid:
     * bits15:8 after abort, bits23:16 after forced reset (p14 correction).
     * No layout change; older outputs leave bit24 clear. */
    uint32_t method, status, irq, pc, after_status, after_irq, rc, elapsed_us;
    struct n4g_fmss_reset_state first_failure, final_state;
};
struct fmss_probe_diag {
    uint32_t stage; /* 1 initial reset, 2 preflight, 3 start, 4 cleanup, 5 complete */
    struct fmss_reset_diag initial, cleanup;
};
extern struct fmss_probe_diag fmss_probe_diagnostics;
/* prog must be 32-byte aligned and outlive the call; leaves the sequencer parked (END-only program, no chip
 * enable). Returns 0, or -1 for a bad request. */
struct n4g_probe_hal;
int fmss_probe(const struct n4g_probe_hal *hal, const struct fmss_probe_req *q, const uint32_t *prog, struct fmss_probe_out *o);

#endif
