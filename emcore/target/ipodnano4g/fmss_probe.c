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

/* Nano4g M2 probe. Sampling/layout derived from the nano3g experiment harness.
 * Reset/start: bl_NandReadOnly mem:0x502c / 0x51b8.
 * This experimental harness uses bounded waits; the library stays unchanged. */
#include "global.h"
#include "timer.h"
#include "s5l8720.h"
#include "nand_hal_emcore.h"
#include "nand/fmss-controller.h"
#include "fmss_probe.h"
#include "clockgates.h"
#include "malloc.h"
#include "contextswitch.h"
#include "nand_poc.h"
#define FMSS_REG(o) (*(volatile uint32_t *)(0x38a00000u + (o)))
#define FMSS_CTRL FMSS_REG(0xc00)
#define FMSS_PROG FMSS_REG(0xc04)
#define FMSS_CS_IRQ FMSS_REG(0xc0c)
#define FMSS_RESULT FMSS_REG(0xc30)
#define FMCTRL0 FMSS_REG(0)
#define FMSS_CTRL_START 0xffb5
#define CS_IRQ_ALL 0xff
#define CS_IRQ_DONE 1
#define CS_IRQ_FAULT 4
#undef USEC_TIMER
#define USEC_TIMER read_usec_timer()
static const uint32_t fmss_idle[2] __attribute__((aligned(32))) = {0, 0};
struct fmss_probe_diag fmss_probe_diagnostics;
static void fmss_clear_irq(void) { FMSS_CS_IRQ = CS_IRQ_ALL; }

/* Experiments may leave opcode12 paused (p06: C08=a, IRQ=8). The normal
 * driver reset waits for idle and cannot leave that state. CPU handoff uses
 * PC write, IRQ8 acknowledgement, CTRL|1 (bl_NandReadOnly mem:0x4d08).
 * Redirect to our END-only program instead of executing the candidate's tail.
 * Program-base redirection on resume is a testable inference: require END
 * and then the ordinary reset before claiming recovery. Capture precedes this
 * operation, so cleanup cannot turn a paused observation into an END result. */
static int probe_reset(const struct n4g_probe_hal *hal, struct fmss_reset_diag *d)
{
    uint32_t start = USEC_TIMER, control, until, status;
    int rc;
    d->status = FMSS_REG(0xc08); d->irq = FMSS_CS_IRQ; d->pc = FMSS_REG(0xc38);
    if (d->status == 0xa)
    {
        d->method = 1; /* redirect paused execution to trusted END */
        control = FMSS_CTRL;
        hal->cache_wb((void *)fmss_idle, sizeof(fmss_idle));
        FMSS_PROG = (uint32_t)(uintptr_t)fmss_idle;
        FMSS_REG(0xc38) = 0;
        FMSS_CS_IRQ = CS_IRQ_ALL;
        /* STOP/abort may have been left by a preceding failed cleanup. */
        FMSS_CTRL = (control & 0xffff0000u) | FMSS_CTRL_START;
        until = USEC_TIMER;
        while (!(FMSS_CS_IRQ & CS_IRQ_DONE) && USEC_TIMER - until < 1000) ;
        if (!(FMSS_CS_IRQ & CS_IRQ_DONE))
        {
            /* Bounded abort candidate; report it separately from END parking. */
            d->method |= 2;
            FMSS_CTRL = 2;
            until = USEC_TIMER;
            while (FMSS_REG(0xc08) != 0 && FMSS_REG(0xc08) != 3 && USEC_TIMER - until < 1000) ;
        }
    }
    rc = n4g_fmss_reset_traced(&d->final_state);
    if (rc) {
        d->first_failure = d->final_state;
        /* P13 ctrlfx/start=1/evjmp: STOP leaves C08=b, PC=20, IRQ=0.
         * The ordinary driver only aborts C08=4, so it never reaches the
         * sequencer reset in this state. Do not let that omission poison
         * every subsequent probe. Try abort, then the M2 sequencer-reset
         * write. Both are bounded candidates until device qualification;
         * success requires the complete ordinary reset to acknowledge.
         * The next probe also executes its END-only preflight. */
        /* P26/P28: a CPU write C60 = 1 (buffer-reset request) during a run
         * leaves C08 = 1 after the run ends; STOP then never reaches idle and
         * the hold survives an emCORE reload. Clearing C60 while running let
         * the program finish and cleanup succeed (p28 restore-2500). Clear the
         * request, then abort and reset the sequencer if C08 still is not idle;
         * success still requires the complete ordinary reset (method 0x20,
         * 0x40 when the abort/reset fallback was needed). */
        if (d->final_state.phase == 1 && d->final_state.bufrst) {
            d->method |= 0x20;
            FMSS_REG(0xc60) = 0;
            until = USEC_TIMER;
            do { status = FMSS_REG(0xc08); }
            while (status != 0 && status != 3 && USEC_TIMER - until < 1000);
            if (status != 0 && status != 3) {
                d->method |= 0x40;
                FMSS_CTRL = 2;
                until = USEC_TIMER;
                while (FMSS_REG(0xc08) != 0 && FMSS_REG(0xc08) != 3 && USEC_TIMER - until < 1000) ;
                FMSS_CTRL = 0x80000;
                until = USEC_TIMER;
                while (FMSS_REG(0xc08) != 0 && FMSS_REG(0xc08) != 3 && USEC_TIMER - until < 1000) ;
            }
            rc = n4g_fmss_reset_traced(&d->final_state);
        }
        if (rc && d->final_state.phase == 1 && d->final_state.status == 0xb) {
            d->method |= 8;
            FMSS_CTRL = 2;
            until = USEC_TIMER;
            do { status = FMSS_REG(0xc08); }
            while (status != 0 && status != 3 && USEC_TIMER - until < 1000);
            /* P14: abort can satisfy this poll without clearing the latched
             * wait; the next STOP restores b. Always reset while held, before
             * another STOP. Record the intermediate states instead of
             * interpreting 0/3 as proof of recovery. Method bit24 marks the
             * status bytes valid: abort at bits15:8, reset at bits23:16. */
            d->method |= 0x01000010u | ((status & 0xff) << 8);
            FMSS_CTRL = 0x80000;
            until = USEC_TIMER;
            do { status = FMSS_REG(0xc08); }
            while (status != 0 && status != 3 && USEC_TIMER - until < 1000);
            d->method |= (status & 0xff) << 16;
            rc = n4g_fmss_reset_traced(&d->final_state);
        }
        /* A timed-out status-latch experiment can leave FMC active even after
         * sequencer END (p07 ev5). Original status polling releases it with
         * 0x30020 then acknowledges 0x1800000. Try that only after capturing
         * the failed reset, with the sequencer idle/aborted; never rerun the
         * candidate. The next ordinary reset must actually acknowledge. */
        if ((d->final_state.status == 0 || d->final_state.status == 3) &&
            (d->final_state.phase == 3 || d->final_state.phase == 4)) {
            d->method |= 4;
            FMSS_REG(0x004) = 0x30020;
            FMSS_REG(0x048) = 0x1800000;
            rc = n4g_fmss_reset_traced(&d->final_state);
        }
    }
    d->after_status = FMSS_REG(0xc08); d->after_irq = FMSS_CS_IRQ;
    /* No silent success if the reset leaves a paused/running engine behind. */
    if (!rc && d->after_status != 0 && d->after_status != 3) rc = -3;
    d->rc = rc; d->elapsed_us = USEC_TIMER - start;
    return rc;
}

/* Buffer reset must precede every start, including the END-only preflight. */
static int probe_start(uint32_t word)
{
    if (n4g_fmss_start(word)) { n4g_fmss_reset(); return -1; }
    return 0;
}

static int probe_write_ok(uint32_t off, uint32_t value, uint32_t flags)
{
    (void)value;
    if (flags & FMSS_PROBE_DISCOVERY) off &= 0xfff;
    return off <= 0xffc && !(off & 3);
}

static int is_cfg(uint32_t off);
static int is_eic(uint32_t off);
static int is_absolute(uint32_t off) { return off >= 0x1000 && off < 0x40000000 && !(off & 3); }

/* Params that are not FMSS window writes: VIC soft lines, EIC words, round-6 settings (checked by probe_config). */
static int probe_vic_param(uint32_t off, uint32_t flags)
{
    if (is_absolute(off)) return 1;
    if (is_cfg(off)) return (flags & (FMSS_PROBE_SAMPLER | FMSS_PROBE_GATEWIN | FMSS_PROBE_PADS)) != 0;
    if (is_eic(off)) return (flags & FMSS_PROBE_EIC) != 0;
    return (flags & FMSS_PROBE_VICSOFT) && (off == FMSS_PROBE_VIC_PARAM(0, 0x18) || off == FMSS_PROBE_VIC_PARAM(1, 0x18)
                                            || off == FMSS_PROBE_VIC_PARAM(0, 0x1c) || off == FMSS_PROBE_VIC_PARAM(1, 0x1c));
}

static void eic_write(uint32_t off, uint32_t val);

/* FMSS window write, or a VIC soft-interrupt raise/clear. A raise never touches
 * a line the CPU has enabled, so no handler runs for it. */
static void probe_param_write(uint32_t off, uint32_t val)
{
    if (is_cfg(off)) return;
    if (is_absolute(off)) { *(volatile uint32_t *)off = val; return; }
    if (is_eic(off)) { eic_write(off, val); return; }
    if (off & 0x80000000u)
    {
        uint32_t v = (off >> 12) & 1;
        if ((off & 0xff) == 0x18) VICSOFTINT(v) = val & ~VICINTENABLE(v);
        else VICSOFTINTCLEAR(v) = val;
    }
    else
        FMSS_REG(off) = val;
}

static int snapshot_skip(uint32_t off);

/* ---- round-6 options: CPU sampler, clock-gate window, GPIO interrupt controller, far landing pads ---- */
#define EIC_REG(o) (*(volatile uint32_t *)(FMSS_PROBE_EIC_BASE + (o)))
#define CFG_ID(off) (((off) >> 8) & 0xffu)
#define CFG_K(off) (((off) >> 2) & 0x3fu)
struct probe_cfg {
    uint32_t nsrc, src[8], period, from, mode;          /* sampler */
    uint32_t gmask[5], gat, giters, goff, checkpoint;   /* gate window */
    uint32_t span;                                      /* pads */
};
static uint32_t eic_saved[32], eic_touched[8];

static int is_cfg(uint32_t off) { return (off & 0xffff0003u) == 0x40000000u && CFG_ID(off) >= 1 && CFG_ID(off) <= 3; }
static int is_eic(uint32_t off) { return (off & 0xfffffc03u) == 0x90000000u && ((off >> 8) & 3) < 3 && (off & 0xff) >= 0x80; }

static int smp_source_ok(uint32_t src, uint32_t flags)
{
    (void)flags;
    return !(src & 3);
}

static uint32_t smp_read(uint32_t src)
{
    if (src <= 0xffc) return FMSS_REG(src);
    if (src == FMSS_PROBE_VIC_PARAM(0, 0) || src == FMSS_PROBE_VIC_PARAM(0, 8) ||
        src == FMSS_PROBE_VIC_PARAM(1, 0) || src == FMSS_PROBE_VIC_PARAM(1, 8))
        return (src & 0xff) ? VICRAWINTR((src >> 12) & 1) : VICIRQSTATUS((src >> 12) & 1);
    return *(volatile uint32_t *)src;
}

/* Collect and check the CFG params of the requested options; -1 for an inconsistent request. */
static int probe_config(const struct fmss_probe_req *q, struct probe_cfg *c)
{
    uint32_t i, have = 0;
    memset(c, 0, sizeof(*c));
    c->goff = 0xc1c;
    for (i = 0; i < q->nparams; i++)
    {
        uint32_t off = q->params[i].off, v = q->params[i].val, id, k;
        if (!is_cfg(off)) continue;
        if (i >= q->nparams - q->nlate - q->nlate2) return -1;
        id = CFG_ID(off); k = CFG_K(off);
        have |= 1u << id;
        if (id == FMSS_PROBE_CFG_SMP)
        {
            if (k < 8) { if (k != c->nsrc || !smp_source_ok(v, q->flags)) return -1; c->src[c->nsrc++] = v; }
            else if (k == 8) c->period = v;
            else if (k == 9) c->from = v;
            else if (k == 10) c->mode = v;
            else return -1;
        }
        else if (id == FMSS_PROBE_CFG_GATE)
        {
            if (k < 5) c->gmask[k] = v;
            else if (k == 5) c->gat = v;
            else if (k == 6) c->giters = v;
            else if (k == 7) { if (!smp_source_ok(v, q->flags) || v > 0xffc) return -1; c->goff = v; }
            else if (k == 8 && v <= 1) c->checkpoint = v;
            else return -1;
        }
        else if (k == 0) c->span = v;
        else return -1;
    }
    if (!!(have & (1u << FMSS_PROBE_CFG_SMP)) != !!(q->flags & FMSS_PROBE_SAMPLER) ||
        !!(have & (1u << FMSS_PROBE_CFG_GATE)) != !!(q->flags & FMSS_PROBE_GATEWIN) ||
        !!(have & (1u << FMSS_PROBE_CFG_PADS)) != !!(q->flags & FMSS_PROBE_PADS)) return -1;
    if ((q->flags & FMSS_PROBE_SAMPLER) &&
        (!c->nsrc || (q->flags & (FMSS_PROBE_SAMPLE_REGS | FMSS_PROBE_STALL_SAFE)))) return -1;
    if ((q->flags & FMSS_PROBE_GATEWIN) &&
        (!c->giters || (q->flags & FMSS_PROBE_STALL_SAFE))) return -1;
    if ((q->flags & FMSS_PROBE_PADS) &&
        (c->span < q->prog_bytes + 16 || c->span > FMSS_PROBE_PAD_MAX || (c->span & 15) ||
         (q->flags & FMSS_PROBE_PROG_SRAM))) return -1;
    return 0;
}

static void eic_save(void)
{
    uint32_t i;
    for (i = 0; i < 32; i++) eic_saved[i] = EIC_REG(0x80 + 4 * i);
    memset(eic_touched, 0, sizeof(eic_touched));
}

static void eic_write(uint32_t off, uint32_t val)
{
    uint32_t reg = off & 0xfc, op = (off >> 8) & 3;
    eic_touched[(reg >> 2) & 7] |= val;
    if (op == 0) EIC_REG(reg) |= val;
    else if (op == 1) EIC_REG(reg) &= ~val;
    else EIC_REG(reg) = val;
}

/* Enables off, level and type back, the touched status bits cleared (W1C), then the saved enables. */
static void eic_restore(void)
{
    uint32_t g;
    for (g = 0; g < 8; g++) EIC_REG(0xc0 + 4 * g) = 0;
    for (g = 0; g < 8; g++) { EIC_REG(0x80 + 4 * g) = eic_saved[g]; EIC_REG(0xe0 + 4 * g) = eic_saved[24 + g]; }
    for (g = 0; g < 8; g++) EIC_REG(0xa0 + 4 * g) = eic_touched[g];
    for (g = 0; g < 8; g++) EIC_REG(0xc0 + 4 * g) = eic_saved[16 + g];
}

extern void fmss_gate_pulse(const uint32_t *before, const uint32_t *mask, uint32_t iterations);

/* IRQs off; the assembly pulse touches only PWRCON while any gate is off.
 * Sample GATE_MARK|1: t = entry, r0..4 = PWRCON(0..4) before, r5/r6 = VIC0/VIC1 raw before, r7 = word before.
 * Sample GATE_MARK|2: t = entry + elapsed after restore, r2 = iterations,
 * r0/r1/r3/r4/r5 = 0xffffffff (no MMIO samples while clocks are disabled),
 * r6 = word after restore, r7 = 1 if every PWRCON word reads its saved value.
 * Sample GATE_MARK|3: r0/r1 = restored VIC0/VIC1 raw, r2 = word, r3 = C08, r7 = restore verified.
 * Gates already off are left as they are. */
static void __attribute__((noinline)) ICODE_ATTR gate_window(const struct probe_cfg *c, struct fmss_probe_out *o, uint32_t t_us)
{
    uint32_t before[5], t0, t1, k, ok = 1, mode;
    uint32_t iterations = c->giters;
    struct fmss_probe_sample *a = o->nsamples < FMSS_PROBE_MAXSAMPLES ? &o->s[o->nsamples++] : NULL, *b;
    mode = enter_critical_section();
    for (k = 0; k < 5; k++) before[k] = PWRCON(k);
    if (a) {
        a->t_us = t_us; a->pc = FMSS_PROBE_GATE_MARK | 1;
        for (k = 0; k < 5; k++) a->r[k] = before[k];
        a->r[5] = VIC0RAWINTR; a->r[6] = VICRAWINTR(1); a->r[7] = FMSS_REG(c->goff);
    }
    t0 = USEC_TIMER;
    fmss_gate_pulse(before, c->gmask, iterations);
    t1 = USEC_TIMER;
    for (k = 0; k < 5; k++) if (PWRCON(k) != before[k]) ok = 0;
    b = o->nsamples < FMSS_PROBE_MAXSAMPLES ? &o->s[o->nsamples++] : NULL;
    if (b) {
        b->t_us = t_us + (t1 - t0); b->pc = FMSS_PROBE_GATE_MARK | 2;
        b->r[0] = b->r[1] = b->r[3] = b->r[4] = b->r[5] = 0xffffffffu;
        b->r[2] = iterations; b->r[6] = FMSS_REG(c->goff); b->r[7] = ok;
    }
    else o->dropped++;
    if (!a) o->dropped++;
    b = o->nsamples < FMSS_PROBE_MAXSAMPLES ? &o->s[o->nsamples++] : NULL;
    if (b) {
        memset(b, 0, sizeof(*b));
        b->t_us = t_us + (USEC_TIMER - t0); b->pc = FMSS_PROBE_GATE_MARK | 3;
        b->r[0] = VIC0RAWINTR; b->r[1] = VICRAWINTR(1);
        b->r[2] = FMSS_REG(c->goff); b->r[3] = FMSS_REG(0xc08); b->r[7] = ok;
    }
    else o->dropped++;
    leave_critical_section(mode);
}

/* The pad pattern: 16-byte units of movi r2,#(TAG | x >> 4) and END. */
static void pads_fill(uint32_t *region, uint32_t from, uint32_t span)
{
    uint32_t x;
    for (x = from; x < span; x += 16)
    {
        region[x / 4] = 0x05020000u;
        region[x / 4 + 1] = FMSS_PROBE_PAD_TAG | (x >> 4);
        region[x / 4 + 2] = 0;
        region[x / 4 + 3] = 0;
    }
}

static void fmss_start_clean(void)
{
    FMSS_REG(0xc58) = 0;
    FMSS_REG(0xc4c) = 0;
}

static int snapshot_skip(uint32_t off)
{
    /* M2 ID/meta latches and the raw PIO FIFO. fw_restore m2FmcFifoRead
     * mem:0x0802a640 repeatedly reads +0x080 to consume data. No M1 FIFO
     * mirror at +0x280/+0x480/+0x680 is established for this controller. */
    return (off >= 0x060 && off <= 0x068) || off == 0x080;
}

static const uint16_t fmss_probe_win[FMSS_PROBE_NWIN] = { 0x000, 0x800, 0xc00, 0xd00 };

static void fmss_probe_snap(uint32_t win[FMSS_PROBE_NWIN][64], uint32_t flags, uint32_t skip)
{
    unsigned int w, i;
    for (w = 0; w < FMSS_PROBE_NWIN; w++)
        for (i = 0; i < 64; i++)
        {
            uint32_t off = fmss_probe_win[w] + 4 * i;
            if (snapshot_skip(off) || (skip >> (w ? w + 1 : 0) & 1))
                win[w][i] = 0;
            else
                win[w][i] = FMSS_REG(off);
        }
}

static void fmss_probe_fullmap(uint32_t *dst)
{
    unsigned int i;
    for (i = 0; i < 1024; i++)
        dst[i] = snapshot_skip(4 * i) ? 0 : FMSS_REG(4 * i);
}

int fmss_probe(const struct n4g_probe_hal *hal, const struct fmss_probe_req *q, const uint32_t *prog, struct fmss_probe_out *o)
{
    static uint32_t sram_prog[FMSS_PROBE_SRAM_BYTES / 4] __attribute__((section(".idata"), aligned(32)));
    uint32_t start, now, irq = 0, timeout, i, last_pc = 0xffffffffu, r[8], last_r[8], late_done = 0, late2_done = 0;
    uint32_t early, l1, gate_done = 0, last_smp = 0, smp_any = 0;
    struct probe_cfg cfg;
    uint32_t *pads = NULL;
    int rc = 0;
    memset(&fmss_probe_diagnostics, 0, sizeof(fmss_probe_diagnostics));
    if (q->magic != FMSS_PROBE_REQ_MAGIC || q->version != 4 ||
        q->flags & (FMSS_PROBE_WIN400 | FMSS_PROBE_CH1_PINS) || q->nparams > FMSS_PROBE_MAXPARAMS || q->nlate > q->nparams || q->nlate2 > q->nparams - q->nlate
        || !q->prog_bytes
        || ((q->flags & FMSS_PROBE_STALL_SAFE) && (q->nlate || q->nlate2))
        || (q->prog_bytes & 7) || ((uintptr_t)prog & 31))
        return -1;
    for (i = 0; i < q->nparams; i++)
        if (!probe_vic_param(q->params[i].off, q->flags) && (q->params[i].off > 0xffc || (q->params[i].off & 3))) return -1;
    if ((q->flags & FMSS_PROBE_PROG_SRAM) && q->prog_bytes > FMSS_PROBE_SRAM_BYTES) return -1;
    if (probe_config(q, &cfg)) return -1;
    for (i = 0; i < q->nparams; i++)
        if (!probe_vic_param(q->params[i].off, q->flags) && !probe_write_ok(q->params[i].off, q->params[i].val, q->flags)) return -1;
    for (i = 0; i < q->prog_bytes / 4; i += 2)
    {
        uint32_t op = prog[i] >> 24, off = prog[i] & 0xffff;
        if (op == 1 && !probe_write_ok(off, prog[i + 1], q->flags)) return -1;
        if (op == 2 && !probe_write_ok(off, 0x100, q->flags)) return -1;
    }
    hal->cache_wb((void *)fmss_idle, sizeof(fmss_idle));
    early = q->nparams - q->nlate - q->nlate2;      /* params: early..., first late batch..., second batch... */
    l1 = early + q->nlate;
    timeout = q->timeout_us ? (q->timeout_us < 2000000 ? q->timeout_us : 2000000) : 100000;
    memset(o, 0, sizeof(*o));
    o->magic = FMSS_PROBE_OUT_MAGIC;
    o->version = 14;                                 /* explicit mutation/discovery flags; full map */
    o->prog = (uint32_t)(uintptr_t)prog;
    /* Gates are an explicit host step. No M1 channel-1 pin or +0x400 writes.
     * The END-only preflight must complete before the requested experiment. */
    fmss_probe_diagnostics.stage = 1;
    if (!(q->flags & FMSS_PROBE_NO_RESET) && probe_reset(hal, &fmss_probe_diagnostics.initial)) return -2;
    if (!(q->flags & FMSS_PROBE_NO_CLEAN)) fmss_start_clean();
    fmss_clear_irq();
    fmss_probe_diagnostics.stage = 2;
    if (q->flags & FMSS_PROBE_NO_PREFLIGHT)
        o->pre_ok = 2;                              /* preflight skipped on request */
    else
    {
        FMSS_PROG = (uint32_t)(uintptr_t)fmss_idle;
        FMSS_CS_IRQ = CS_IRQ_ALL;
        start = USEC_TIMER;
        if (probe_start(FMSS_CTRL_START)) return -2;
        while (!(FMSS_CS_IRQ & (CS_IRQ_DONE | CS_IRQ_FAULT)) && USEC_TIMER - start < 10000)
            ;
        o->pre_us = USEC_TIMER - start;
        o->pre_cs_irq = FMSS_CS_IRQ;
        o->pre_c00 = FMSS_CTRL;
        o->pre_ok = (o->pre_cs_irq & (CS_IRQ_DONE | CS_IRQ_FAULT)) == CS_IRQ_DONE;
        fmss_clear_irq();
    }
    if (!o->pre_ok) { o->why = FMSS_PROBE_TIMEOUT; probe_reset(hal, &fmss_probe_diagnostics.cleanup); FMCTRL0 = 0; return -2; }
    if (q->flags & FMSS_PROBE_PADS)
    {
        pads = memalign(32, cfg.span);
        if (!pads) { probe_reset(hal, &fmss_probe_diagnostics.cleanup); FMCTRL0 = 0; return -5; }
    }
    if (q->flags & FMSS_PROBE_EIC) eic_save();
    if (!(q->flags & FMSS_PROBE_KEEP_CTRL0)) FMCTRL0 = 0;
    for (i = 0; i < early; i++)
        probe_param_write(q->params[i].off, q->params[i].val);
    if (q->flags & FMSS_PROBE_PROG_SRAM)
    {
        memcpy(sram_prog, prog, q->prog_bytes);
        prog = sram_prog;
        o->prog = (uint32_t)(uintptr_t)prog;
    }
    if (pads)
    {
        memcpy(pads, prog, q->prog_bytes);
        pads_fill(pads, (q->prog_bytes + 15) & ~15u, cfg.span);
        hal->cache_wb(pads, cfg.span);
        prog = pads;
        o->prog = (uint32_t)(uintptr_t)prog;
    }
    fmss_probe_snap(o->before, q->flags, q->flags >> FMSS_PROBE_SNAP_SKIP_SHIFT & 0x1f);   /* the starting state */
    if (q->flags & FMSS_PROBE_FULLMAP) fmss_probe_fullmap(o->fullmap[0]);
    hal->cache_wb((void *)prog, q->prog_bytes);
    for (i = 0; i < 8; i++) last_r[i] = 0xffffffffu;
    FMSS_PROG = (uint32_t)(uintptr_t)prog;
    FMSS_CS_IRQ = CS_IRQ_ALL;
    if (cfg.checkpoint && nand_poc_gate_checkpoint(cfg.gmask, cfg.giters)) {
        o->why = FMSS_PROBE_TIMEOUT;
        if (q->flags & FMSS_PROBE_EIC) eic_restore();
        if (pads) free(pads);
        probe_reset(hal, &fmss_probe_diagnostics.cleanup);
        FMCTRL0 = 0;
        return -6;
    }
    start = USEC_TIMER;
    fmss_probe_diagnostics.stage = 3;
    if (probe_start(q->start_value ? q->start_value : FMSS_CTRL_START))
    {
        if (q->flags & FMSS_PROBE_EIC) eic_restore();
        if (pads) free(pads);                       /* probe_start reset the engine before returning */
        return -2;
    }
    if (q->flags & FMSS_PROBE_STALL_SAFE)
    {
        /* Touch nothing on the FMSS slave port while a possibly stalled access is outstanding:
         * wait on the CPU timer, then cycle the FMSS clock gates before the first register read. */
        while (USEC_TIMER - start < timeout)
            ;
        clockgate_enable(5, false);
        clockgate_enable(9, false);
        now = USEC_TIMER;
        while (USEC_TIMER - now < 100)
            ;
        clockgate_enable(5, true);
        clockgate_enable(9, true);
        o->dropped = FMSS_PROBE_STALL_MARK;
        irq = FMSS_CS_IRQ & (CS_IRQ_DONE | CS_IRQ_FAULT);
        if (irq & CS_IRQ_FAULT) o->fault_at_us = timeout | 1;
    }
    else for (;;)
    {
        uint32_t pc = FMSS_REG(0xc38), changed = pc != last_pc;
        now = USEC_TIMER;
        o->polls++;
        if (q->nlate && !late_done && now - start >= q->late_us)
        {
            for (i = early; i < l1; i++)
                probe_param_write(q->params[i].off, q->params[i].val);
            late_done = 1;
            o->late_at_us = now - start;
        }
        if (q->nlate2 && late_done && !late2_done && now - start >= q->late2_us)
        {
            for (i = l1; i < q->nparams; i++)
                probe_param_write(q->params[i].off, q->params[i].val);
            late2_done = 1;
            o->late2_at_us = now - start;
        }
        o->irq_seen |= VIC1RAWINTR;
        o->vic0_seen |= VIC0RAWINTR;
        if (q->flags & FMSS_PROBE_SAMPLER)
        {
            /* host-chosen sources; record per the period/from/mode settings */
            uint32_t vchanged = !smp_any;
            for (i = 0; i < 8; i++)
            {
                r[i] = i < cfg.nsrc ? smp_read(cfg.src[i]) : 0;
                if (r[i] != last_r[i]) vchanged = 1;
            }
            changed = now - start >= cfg.from && (!smp_any || now - last_smp >= cfg.period) &&
                      ((cfg.mode & 1) ? (vchanged || ((cfg.mode & 2) && changed))
                                      : 1);
            if (changed) { smp_any = 1; last_smp = now; }
        }
        else for (i = 0; i < 8; i++)
        {
            /* r0..r7 only on request; by default r[7] carries CS_IRQ (what the driver itself polls) */
            r[i] = (q->flags & FMSS_PROBE_SAMPLE_REGS) ? FMSS_REG(0xc18 + 4 * i) : i == 7 ? FMSS_CS_IRQ : 0;
            if (r[i] != last_r[i]) changed = 1;
        }
        if (changed)
        {
            if (o->nsamples < FMSS_PROBE_MAXSAMPLES)
            {
                struct fmss_probe_sample *p = &o->s[o->nsamples++];
                p->t_us = now - start;
                p->pc = pc;
                memcpy(p->r, r, sizeof(r));
            }
            else o->dropped++;
            last_pc = pc;
            memcpy(last_r, r, sizeof(r));
        }
        if ((q->flags & FMSS_PROBE_GATEWIN) && !gate_done && now - start >= cfg.gat)
        {
            gate_window(&cfg, o, now - start);
            if (cfg.checkpoint) nand_poc_gate_restored();
            gate_done = 1;
        }
        irq = FMSS_CS_IRQ & (CS_IRQ_DONE | CS_IRQ_FAULT);
        if ((irq & CS_IRQ_FAULT) && !o->fault_at_us) o->fault_at_us = (now - start) | 1;
        if (q->flags & FMSS_PROBE_WAIT_END ? (irq & CS_IRQ_DONE) : irq) break;
        if (now - start >= timeout) break;
    }
    o->elapsed_us = USEC_TIMER - start;
    o->why = (irq & CS_IRQ_FAULT) ? FMSS_PROBE_FAULT : (irq & CS_IRQ_DONE) ? FMSS_PROBE_END : FMSS_PROBE_TIMEOUT;
    o->cs_irq = FMSS_CS_IRQ;
    o->seq_result = FMSS_RESULT;
    o->pc = FMSS_REG(0xc38);
    o->c50 = FMSS_REG(0xc50);
    o->irq_end = VIC1RAWINTR;
    if (q->flags & FMSS_PROBE_VICSOFT) { VICSOFTINTCLEAR(0) = 0xffffffffu; VICSOFTINTCLEAR(1) = 0xffffffffu; }
    if (q->flags & FMSS_PROBE_EIC) eic_restore();
    fmss_probe_snap(o->after, q->flags, 0);
    if (q->flags & FMSS_PROBE_FULLMAP) fmss_probe_fullmap(o->fullmap[1]);
    /* Always stop before returning the DMA buffer to the host. */
    fmss_probe_diagnostics.stage = 4;
    if (probe_reset(hal, &fmss_probe_diagnostics.cleanup)) rc = -2;
    /* The engine is stopped (or reported held): only then the pad region goes back to the heap. A held engine
     * keeps it allocated, as its program memory. */
    if (pads && !rc) free(pads);
    if (rc) return rc;
    if (!(q->flags & (FMSS_PROBE_END_RESET | FMSS_PROBE_END_NOCLEAN))) fmss_start_clean();
    FMCTRL0 = 0;
    fmss_clear_irq();
    fmss_probe_diagnostics.stage = 5;
    return 0;
}
