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

/* "fmss" backend of the nano 3G target: the S5L8702's microcode sequencer
 * (FMSS) runs the NAND operations. The CPU builds the lists a program works
 * through in memory, writes the parameter RAM, starts the program and waits
 * for its end; the programs (the .s files in fmss/, assembled at build time) do the
 * chip-enable interleaving, ECC, spare metadata and DMA themselves, so the
 * on-flash format is the one of the "sw" backend and of the original
 * firmware.
 *
 * Start: FMSS_PROG = program bus address, CS_IRQ = 0xff, FMSS_CTRL = 0xfff5.
 * Completion: the raw status of VIC1 line 54 (not enabled, polled) or CS_IRQ
 * bit 0 (end) / bit 2 (fault), bounded to two seconds. A fault or a timeout
 * parks the sequencer by running an END-only program.
 *
 * The chip table and geometry are shared with the "sw" backend (nand-chip.h);
 * the job planning is in nand-fmss-plan.h.
 */
#include "global.h"
#include "timer.h"
#include "../s5l8702.h"
#include "nand-compat.h"
#include "nand-target.h"
#include "nand-backend.h"
#include "nand-chip.h"
#include "nand-fmss.h"
#include "nand-fmss-plan.h"
#include "thread.h"
#include "malloc.h"
#include "fmss-nano3g.h"

/* The sequencer programs (nand/fmss/), assembled at build time */
struct fmss_progs { const uint32_t *readid, *read, *program, *erase; size_t nreadid, nread, nprogram, nerase; };
#define FMSS_SET(p) { p##readid, p##read, p##program, p##erase, \
                      sizeof(p##readid), sizeof(p##read), sizeof(p##program), sizeof(p##erase) }
static const struct fmss_progs fmss_sets[1] = { FMSS_SET(fmss3g_) };

static unsigned int fmss_set;

#define FMSS_REG(off)   (*(volatile uint32_t *)(FMC_BASE + (off)))
#define FMSS_CTRL       FMSS_REG(0xc00)
#define FMSS_PROG       FMSS_REG(0xc04)
#define FMSS_CS_IRQ     FMSS_REG(0xc0c)
#define FMSS_RESULT     FMSS_REG(0xc30)
#define FMC_REG_400     FMSS_REG(0x400)

#define FMSS_CTRL_RESET 0x30000
#define FMSS_CTRL_START 0xfff5
#define CS_IRQ_ALL      0xff
#define CS_IRQ_DONE     (1 << 0)
#define CS_IRQ_FAULT    (1 << 2)
#define VIC1_FMSS_BIT   (1 << 22)       /* line 54 */

/* SEQ_RESULT of the program ends */
#define RES_FAIL        (1u << 28)      /* NAND FAIL, or a fault */
#define RES_CE_MASK     0xffu

#define FMSS_WAIT_US    2000000         /* a run may not take longer */
#define FMSS_CLEAR_US   100             /* CS_IRQ clear handshake */

/* The hardware-validated FMC_CTRL0 timing of the "sw" backend */
#define FMSS_TIMING_SAFE_WORD ((4u << 16) | (3u << 12))

static const uint32_t fmss_idle[2] __attribute__((aligned(32))) = { 0, 0 };
static struct fmss_job job;

static const struct nand_chip_info *fmss_chip;
static uint32_t fmss_id;
static unsigned int fmss_banks;
static bool fmss_ready;
static struct nand_geometry fmss_geo;
static uint32_t fmss_rtiming = FMSS_TIMING_SAFE_WORD;
static uint32_t fmss_wtiming = FMSS_TIMING_SAFE_WORD;
static unsigned int fmss_tmode = NAND_FMSS_TIMING_SAFE;
static unsigned int fmss_bus_mhz;
static uint32_t fmss_last_result;      /* SEQ_RESULT of the last failed run */
/* Snapshot of the sequencer at the last fault or timeout (nand-fmss.h), taken before fmss_park() resets it:
 * magic, which program set, the program's bus address, PC (0xc38), r0..r7 (0xc18..0xc34), C3C, C50, CS_IRQ,
 * FMC registers 0x00..0x4c (control/status/ECC side), ECC 0x80c..0x814 and 0x840, a fault counter. */
struct fmss_fault_info fmss_fault;
uint32_t fmss_poll_us = 100;           /* sleep between polls of a running program (nand-fmss.h) */

static bool fmss_elapsed(uint32_t start, uint32_t us)
{
    return (uint32_t)(USEC_TIMER - start) >= us;
}

static void fmss_apply_timing(void);

/* the benchmark's timing override: word = (setup << 16) | (hold << 12) [| EDO << 28] for reads and writes;
   0 = back to the configured mode */
void nand_fmss_set_raw_timing(uint32_t word)
{
    if (word)
        fmss_rtiming = fmss_wtiming = word & 0x10077000u;
    else
        fmss_apply_timing();
}

static void fmss_apply_timing(void)
{
    if (fmss_tmode == NAND_FMSS_TIMING_CHIP && fmss_bus_mhz && fmss_chip)
    {
        fmss_rtiming = fmss_timing_word(fmss_chip->read_ns[0],
                                        fmss_chip->read_ns[1],
                                        fmss_chip->read_ns[2], fmss_bus_mhz);
        fmss_wtiming = fmss_timing_word(fmss_chip->write_ns[0],
                                        fmss_chip->write_ns[1],
                                        fmss_chip->write_ns[2], fmss_bus_mhz);
    }
    else
        fmss_rtiming = fmss_wtiming = FMSS_TIMING_SAFE_WORD;
}

void nand_fmss_set_timing(unsigned int mode, unsigned int bus_mhz)
{
    fmss_tmode = mode;
    fmss_bus_mhz = bus_mhz;
    fmss_apply_timing();
}

/* Acknowledge end and fault, wait for the flags to drop */
static int fmss_clear_irq(void)
{
    uint32_t start = USEC_TIMER;

    FMSS_CS_IRQ = CS_IRQ_DONE | CS_IRQ_FAULT;
    while (FMSS_CS_IRQ & (CS_IRQ_DONE | CS_IRQ_FAULT))
        if (fmss_elapsed(start, FMSS_CLEAR_US))
            return -1;
    return 0;
}

static void fmss_park(void);

/* PC sampling while a program runs (the fault handler resets the PC, so a snapshot after the fault cannot tell
 * where the program hung): up to 8 distinct PCs with sample counts, plus the last PC seen before the fault.
 * fmss_pc_watch = 1 makes the wait spin (no sleep) so the samples are dense. */
uint32_t fmss_pc_watch;
static uint32_t pcs_pc[8], pcs_n[8], pcs_last;

static void fmss_pcs_reset(void)
{
    int i;
    for (i = 0; i < 8; i++) pcs_pc[i] = pcs_n[i] = 0;
    pcs_last = 0xffffffff;
}

static void fmss_pcs_sample(uint32_t pc)
{
    int i, low = 0;
    if (!(FMSS_CS_IRQ & CS_IRQ_FAULT)) pcs_last = pc;
    for (i = 0; i < 8; i++)
    {
        if (pcs_n[i] && pcs_pc[i] == pc) { pcs_n[i]++; return; }
        if (pcs_n[i] < pcs_n[low]) low = i;
    }
    pcs_pc[low] = pc;
    pcs_n[low] = 1;
}

static void fmss_snapshot(const uint32_t *prog, uint32_t why)
{
    int i;
    fmss_fault.magic = 0x46464d46;          /* "FMFF" */
    fmss_fault.why = why;                   /* 1 fault (CS_IRQ b2), 2 timeout */
    fmss_fault.set = fmss_set;
    fmss_fault.prog = (uint32_t)(uintptr_t)prog;
    fmss_fault.pc = FMSS_REG(0xc38);
    for (i = 0; i < 8; i++) fmss_fault.r[i] = FMSS_REG(0xc18 + 4 * i);
    fmss_fault.c3c = FMSS_REG(0xc3c);
    fmss_fault.c50 = FMSS_REG(0xc50);
    fmss_fault.cs_irq = FMSS_CS_IRQ;
    for (i = 0; i < 20; i++) fmss_fault.fmc[i] = FMSS_REG(4 * i);
    fmss_fault.ecc[0] = FMSS_REG(0x80c);
    fmss_fault.ecc[1] = FMSS_REG(0x810);
    fmss_fault.ecc[2] = FMSS_REG(0x814);
    fmss_fault.ecc[3] = FMSS_REG(0x840);
    for (i = 0; i < 8; i++) { fmss_fault.pcs[i] = pcs_pc[i]; fmss_fault.pcs_n[i] = pcs_n[i]; }
    fmss_fault.pc_last = pcs_last;
    fmss_fault.count++;
}

/* Start the program at bus address prog and wait for its end. Returns 0 on a
 * clean end, -1 on a fault or a timeout (the sequencer is parked afterwards
 * and SEQ_RESULT is kept in fmss_last_result). */
static int fmss_start_wait(const uint32_t *prog)
{
    uint32_t start, irq;

    FMSS_PROG = (uint32_t)(uintptr_t)prog;
    FMSS_CS_IRQ = CS_IRQ_ALL;
    FMSS_CTRL = FMSS_CTRL_START;

    start = USEC_TIMER;
    fmss_pcs_reset();
    for (;;)
    {
        fmss_pcs_sample(FMSS_REG(0xc38));
        /* the raw VIC status is the completion event; CS_IRQ says which */
        if ((VIC1RAWINTR & VIC1_FMSS_BIT) || (FMSS_CS_IRQ & (CS_IRQ_DONE | CS_IRQ_FAULT)))
        {
            irq = FMSS_CS_IRQ & (CS_IRQ_DONE | CS_IRQ_FAULT);
            if (irq)
                break;
        }
        if (fmss_elapsed(start, FMSS_WAIT_US))
        {
            fmss_last_result = FMSS_RESULT;
            fmss_snapshot(prog, 2);
            fmss_park();
            return -1;
        }
        /* the sequencer does the work: block between polls so other threads get the CPU (the scheduler is
           fair-share, a yield() would hand the CPU straight back). The original driver polls the raw VIC
           status too, it does not take the interrupt. */
        if (fmss_poll_us && !fmss_pc_watch)
            sleep(fmss_poll_us);
    }
    if (irq & CS_IRQ_FAULT)
    {
        fmss_last_result = FMSS_RESULT;
        fmss_snapshot(prog, 1);
        fmss_park();
        return -1;
    }
    fmss_clear_irq();
    return 0;
}

/* FMSS_C58, the fault vector FMSS_C4C and FMSS_C5C survive the sequencer reset; with C58 = 1 the next start begins
 * at the fault vector instead of instruction 0 (device: a probe left C58 = 1 and every later program lost its first
 * instructions). Put them back to their power-on value 0 before a start whose program does not set them first. */
static void fmss_start_clean(void)
{
    FMSS_REG(0xc58) = 0;
    FMSS_REG(0xc4c) = 0;
    FMSS_REG(0xc5c) = 0;
}

/* Stop whatever runs: reset the sequencer, run the END-only program, leave
 * the chip enables deselected */
static void fmss_park(void)
{
    FMSS_CTRL = FMSS_CTRL_RESET;
    udelay(1000);
    fmss_start_clean();
    FMSS_PROG = (uint32_t)(uintptr_t)fmss_idle;
    FMSS_CS_IRQ = CS_IRQ_ALL;
    FMSS_CTRL = FMSS_CTRL_START;
    udelay(1000);
    fmss_clear_irq();
    FMCTRL0 = 0;
}

/* Write the parameter words, make the job visible to the controller, run a
 * program. *result gets SEQ_RESULT after a clean end. */
static int fmss_exec(const uint32_t *prog, size_t bytes,
                     const struct fmss_plan *pl, uint32_t *result)
{
    unsigned int i;
    int rc;

    commit_dcache_range(prog, bytes);
    commit_discard_dcache_range(&job, sizeof(job));
    for (i = 0; i < pl->np; i++)
        FMSS_REG(pl->p[i].off) = pl->p[i].val;
    rc = fmss_start_wait(prog);
    discard_dcache_range(&job, sizeof(job));
    if (rc)
        return rc;
    if (result)
        *result = FMSS_RESULT;
    return 0;
}

#define EXEC(name, pl, res) fmss_exec(fmss_sets[fmss_set].name, fmss_sets[fmss_set].n##name, pl, res)

/* Reset every chip and read the ID of CEs 0..n-1 into job.idbuf */
static int fmss_readid(unsigned int n)
{
    struct fmss_plan pl;

    fmss_plan_readid(&pl, &job, (uint32_t)(uintptr_t)&job, n,
                     FMSS_READID_DELAY);
    return EXEC(readid, &pl, NULL);
}

static int fmss_reset(uint32_t bank)
{
    if (bank >= NAND_MAX_BANKS)
        return NAND_ERR_NOINIT;
    nand_hw_enable();
    return fmss_readid(bank + 1) ? NAND_ERR_RESET_CMD : 0;
}

static int fmss_identify(uint32_t bank, uint32_t *id, uint32_t *ext)
{
    if (bank >= NAND_MAX_BANKS)
        return NAND_ERR_NOINIT;
    nand_hw_enable();
    if (fmss_readid(bank + 1))
        return NAND_ERR_IDENTIFY;
    *id = job.idbuf[bank];
    if (ext)
        *ext = 0;               /* ID bytes 4..7 are not kept by the program */
    return 0;
}

static bool fmss_id_valid(uint32_t id)
{
    return id && id != 0xffffffffu;
}

static int fmss_init(void)
{
    uint32_t id0, id;
    unsigned int bank;

    fmss_ready = false;
    fmss_banks = 0;
    fmss_chip = NULL;

    nand_hw_enable();
    fmss_park();
    FMC_REG_400 = 0;                    /* as the original driver's init */

    if (fmss_readid(NAND_MAX_BANKS))
        return NAND_ERR_IDENTIFY;
    if (!fmss_id_valid(job.idbuf[0]))
    {
        /* the broadcast RESET may not have settled: once more */
        udelay(5000);
        if (fmss_readid(NAND_MAX_BANKS) || !fmss_id_valid(job.idbuf[0]))
            return NAND_ERR_IDENTIFY;
    }
    id0 = job.idbuf[0];
    fmss_banks = 1;
    for (bank = 1; bank < NAND_MAX_BANKS; bank++)
    {
        id = job.idbuf[bank];
        if (!fmss_id_valid(id))
            continue;
        if (id != id0 || bank != fmss_banks)
            return NAND_ERR_MIXED;
        fmss_banks++;
    }
    fmss_id = id0;
    fmss_chip = nand_chip_find(id0, fmss_banks);
    if (!fmss_chip)
        return NAND_ERR_UNSUPPORTED;
    fmss_apply_timing();
    fmss_ready = true;
    return 0;
}

static const struct nand_geometry *fmss_get_geometry(void)
{
    if (!fmss_ready)
        return NULL;
    nand_chip_geometry(fmss_chip, fmss_banks, &fmss_geo);
    return &fmss_geo;
}

static uint32_t fmss_get_id(void)
{
    return fmss_id;
}

static unsigned int fmss_units(void)
{
    return fmss_chip->pagesize / FMSS_UNIT_BYTES;
}

static bool fmss_page_ok(uint32_t bank, uint32_t page)
{
    return bank < fmss_banks
        && page < (uint32_t)fmss_chip->blocks * fmss_chip->pagesperblock;
}

/* The bank a failed run reported: the lowest CE of the result's mask */
static uint32_t fmss_fail_bank(uint32_t result, uint32_t dflt)
{
    return (result & RES_CE_MASK) ? (uint32_t)__builtin_ctz(result & RES_CE_MASK)
                                  : dflt;
}

static int fmss_read_pages(struct nand_read *r, unsigned int n)
{
    uint32_t bank[FMSS_MAXPAGES], page[FMSS_MAXPAGES], buf[FMSS_MAXPAGES];
    unsigned int first, cnt, i, units, size;
    struct fmss_plan pl;

    if (!fmss_ready || !n)
        return -1;
    size = fmss_chip->pagesize;
    units = fmss_units();
    for (i = 0; i < n; i++)
    {
        if (!fmss_page_ok(r[i].bank, r[i].page))
            return -1;
        r[i].ecc = NAND_ECC_FAILED;
    }

    for (first = 0; first < n; first += cnt)
    {
        cnt = n - first < FMSS_MAXPAGES ? n - first : FMSS_MAXPAGES;
        for (i = 0; i < cnt; i++)
        {
            bank[i] = r[first + i].bank;
            page[i] = r[first + i].page;
            buf[i] = (uint32_t)(uintptr_t)r[first + i].buf;
            /* the controller writes the buffer behind the cache's back */
            commit_discard_dcache_range(r[first + i].buf, size);
        }
        fmss_plan_read(&pl, &job, (uint32_t)(uintptr_t)&job, fmss_banks,
                       units, cnt, bank, page, buf, fmss_rtiming);
        if (EXEC(read, &pl, NULL))
            return -1;
        for (i = 0; i < cnt; i++)
        {
            struct nand_read *p = &r[first + i];
            uint32_t res = job.res[i];

            discard_dcache_range(p->buf, size);
            p->result = res;
            p->ecc = (res & (1u << 30)) ? NAND_ECC_FAILED
                   : (res & 0xff) ? NAND_ECC_CORRECTED : NAND_ECC_CLEAN;
            if (p->meta)
            {
                p->meta[0] = job.meta[i * 3 + 0];
                p->meta[1] = job.meta[i * 3 + 1];
                p->meta[2] = job.meta[i * 3 + 2];
            }
        }
    }
    return 0;
}

static int fmss_write_pages(const struct nand_write *w, unsigned int n,
                            bool two_plane, uint32_t *failbank)
{
    uint32_t bank[FMSS_MAXPAGES], page[FMSS_MAXPAGES], buf[FMSS_MAXPAGES];
    const uint32_t *meta[FMSS_MAXPAGES];
    unsigned int first, cnt, i, units, size;
    struct fmss_plan pl;
    uint32_t result;
    int rc;

    if (!fmss_ready || !n || (two_plane && (n & 1)))
        return -1;
    size = fmss_chip->pagesize;
    units = fmss_units();
    for (i = 0; i < n; i++)
    {
        if (!fmss_page_ok(w[i].bank, w[i].page) || !w[i].meta
            || (two_plane && (i & 1) && w[i].bank != w[i - 1].bank))
            return -1;
        /* the controller reads the data behind the cache's back */
        commit_dcache_range(w[i].buf, size);
    }

    for (first = 0; first < n; first += cnt)
    {
        cnt = n - first < FMSS_MAXPAGES ? n - first : FMSS_MAXPAGES;
        for (i = 0; i < cnt; i++)
        {
            bank[i] = w[first + i].bank;
            page[i] = w[first + i].page;
            buf[i] = (uint32_t)(uintptr_t)w[first + i].buf;
            meta[i] = w[first + i].meta;
        }
        fmss_plan_write(&pl, &job, (uint32_t)(uintptr_t)&job, fmss_banks,
                        units, cnt, bank, page, buf, meta, two_plane,
                        fmss_wtiming);
        fmss_last_result = 0;
        rc = EXEC(program, &pl, &result);
        if (rc)
        {
            if (failbank)
                *failbank = fmss_fail_bank(fmss_last_result, bank[0]);
            return -1;
        }
        if (result & RES_FAIL)
        {
            if (failbank)
                *failbank = fmss_fail_bank(result, bank[0]);
            return NAND_OP_FAILED;
        }
    }
    return 0;
}

static int fmss_erase_blocks(const struct nand_erase *e, unsigned int n,
                             unsigned int *failidx)
{
    uint32_t bank[FMSS_NCE], block[FMSS_NCE];
    unsigned int first, cnt, i, used;
    struct fmss_plan pl;
    uint32_t result;
    int rc;

    if (!fmss_ready || !n)
        return -1;
    for (i = 0; i < n; i++)
        if (e[i].bank >= fmss_banks || e[i].block >= fmss_chip->blocks)
            return -1;

    /* A run takes every bank at most once: a failure then names its entry */
    for (first = 0; first < n; first += cnt)
    {
        used = 0;
        for (cnt = 0; first + cnt < n; cnt++)
        {
            if (used & (1u << e[first + cnt].bank))
                break;
            used |= 1u << e[first + cnt].bank;
            bank[cnt] = e[first + cnt].bank;
            block[cnt] = e[first + cnt].block;
        }
        fmss_plan_erase(&pl, &job, (uint32_t)(uintptr_t)&job, fmss_banks,
                        fmss_chip->pagesperblock, cnt, bank, block,
                        fmss_rtiming);
        fmss_last_result = 0;
        rc = EXEC(erase, &pl, &result);
        if (rc || (result & RES_FAIL))
        {
            uint32_t mask = rc ? fmss_last_result : result;

            if (failidx)
            {
                *failidx = first;
                for (i = 0; i < cnt; i++)
                    if ((mask & RES_CE_MASK) & (1u << bank[i]))
                    {
                        *failidx = first + i;
                        break;
                    }
            }
            return rc ? -1 : NAND_OP_FAILED;
        }
    }
    return 0;
}

static int fmss_init_set0(void) { fmss_set = 0; return fmss_init(); }

const struct nand_backend nand_backend_fmss =
{
    .name = "fmss",
    .init = fmss_init_set0,
    .reset = fmss_reset,
    .identify = fmss_identify,
    .read_pages = fmss_read_pages,
    .write_pages = fmss_write_pages,
    .erase_blocks = fmss_erase_blocks,
    .get_geometry = fmss_get_geometry,
    .get_id = fmss_get_id,
};
