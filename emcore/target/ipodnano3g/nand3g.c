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


//
/* Nano3g NAND worker. USB commands queue jobs and return immediately; hosts
 * poll the status record and retrieve data from the result buffer. Boot only
 * creates the worker. Backend initialization and FTL mounting occur on
 * explicit commands. Everything is read-only unless a command says
 * otherwise: the FTL mounts read-only unless step 4 gets argument 1, and the
 * raw write benchmark needs N3G_BF_WRITE. */
#include "global.h"
#include "thread.h"
#include "timer.h"
#include "malloc.h"
#include "console.h"
#include "interrupt.h"
#include "clockgates.h"
#include "crc32.h"
#include "nand3g.h"
#include "nand/nand-compat.h"
#include "nand/nand-target.h"
#include "nand/nand-backend.h"
#include "nand/ftl-target.h"
#include "nand/nand-chip.h"
#include "nand/nand-fmss.h"
void nand_sw_set_timing(uint32_t word);

volatile struct n3g_nand_status n3g_nand_status;
#define S n3g_nand_status

_Static_assert(sizeof(struct n3g_nand_status) == N3G_STATUS_BYTES, "status record layout (nano3g/nand.py mirrors it)");
_Static_assert(sizeof(struct n3g_bench_params) == N3G_BENCH_PARAM_BYTES, "bench params layout");
_Static_assert(N3G_PROBE_OUT + sizeof(struct fmss_probe_out) <= N3G_PROBE_SCRATCH, "probe output fits before the scratch area");
_Static_assert(N3G_PROBE_PROG >= sizeof(struct fmss_probe_req), "probe request fits before the program");
_Static_assert(sizeof(struct n3g_bench_result) == N3G_BENCH_RESULT_BYTES, "bench result layout");
_Static_assert(N3G_BENCH_CPU_OFF + sizeof(struct n3g_bench_cpu) <= N3G_BENCH_REP_OFF, "bench cpu block");

#define BENCH_BUF_BYTES (N3G_BENCH_MAXBATCH * 4096u)

static struct scheduler_thread g_thread;
static struct wakeup g_wake;
static uint8_t *g_result;                 /* N3G_RESULT_BYTES */
static uint8_t *g_bbuf;                   /* benchmark data buffer */
static uint32_t *g_meta;                  /* N3G_BENCH_MAXBATCH * 4 words of spare metadata */
static uint32_t g_entries[N3G_BENCH_MAXENTRIES];   /* bank << 24 | page */
static struct nand_read g_rd[N3G_BENCH_MAXBATCH];
static struct nand_write g_wr[N3G_BENCH_MAXBATCH];
static struct nand_erase g_er[N3G_BENCH_MAXBATCH];
static int g_ready, g_ftl_ready;
static int g_write_ready, g_write_poisoned;   /* writer state of the current mount (jobs 7..9) */
static struct scheduler_thread g_load_thread;
static volatile uint32_t g_load_on, g_load_count;   /* CPU meter, see struct n3g_bench_cpu */
static volatile struct n3g_bench_cpu *g_cpu;
static uint32_t g_ph, g_mt_c0;
static struct { uint32_t kind, arg[3]; } g_job;

/* ------------------------------------------------------------------ status record */
static void publish(void)
{
    S.fmss_fault_addr = (uint32_t)&fmss_fault;
    S.fmss_fault_count = fmss_fault.count;
    S.fmss_fault_pc = fmss_fault.pc;
    commit_dcache_range((const void *)&n3g_nand_status, sizeof(n3g_nand_status));
}

static void refresh_live(void)
{
    S.now_us = read_usec_timer();
    S.hb_word = (S.busy ? 0x80000000u : 0u) | (S.state & 0x7fu) << 24 | (S.job_kind & 0xffu) << 16 |
                (S.step_ok_mask & 0xffu) << 8 | (S.jobs_done & 0xffu);
}

static void geom_from_driver(void)
{
    const struct nand_geometry *g = nand_get_geometry();
    const struct nand_backend *b = nand_backend_active();
    uint32_t i;
    for (i = 0; i < 4; i++) { S.chip_ids[i] = 0; S.chip_ext[i] = 0; }
    for (i = 0; i < 12; i++) S.geom[i] = 0;
    S.chip_row = 0xffffffffu;
    S.backend = nand_backend_active_index();
    if (!g) {
        S.chip_ids[0] = b->get_id ? b->get_id() : 0;   /* what bank 0 answered, even when the chip is unsupported */
        return;
    }
    for (i = 0; i < g->banks && i < 4; i++) {
        uint32_t id = 0, ext = 0;
        if (b->identify && b->identify(i, &id, &ext) == 0) { S.chip_ids[i] = id; S.chip_ext[i] = ext; }
    }
    S.geom[0] = g->banks;
    S.geom[1] = g->blocks;
    S.geom[2] = g->pagesperblock;
    S.geom[3] = g->planes;
    S.geom[4] = g->userblocks;
    S.geom[5] = g->vflspares;
    S.geom[6] = g->twoplane;
    S.geom[7] = g->pagesize;
    S.geom[8] = g->mode;
    S.geom[9] = g->layout;
    S.geom[10] = g->validated;
    S.geom[11] = b->get_id ? b->get_id() : 0;
    S.chip_row = (uint32_t)nand_chip_row(S.geom[11], g->banks);
}

static void snap_pcon(void)
{
    S.pcon[0] = PCON(8);
    S.pcon[1] = PCON(9);
    S.pcon[2] = PCON(10);
}

static void invalidate_ftl(void)
{
    g_ftl_ready = 0;
    S.step_ok_mask &= ~(1u << N3G_STEP_FTL);
    S.ftl_sectors = 0;
    S.ftl_sector_bytes = 0;
    S.ftl_writable = 0;
}

/* ------------------------------------------------------------------ steps */
static int need(uint32_t mask) { return (S.step_ok_mask & mask) == mask; }

static uint32_t step_gates(void)
{
    S.pwrcon0_before = PWRCON(0);
    clockgate_enable(CLOCKGATE_NAND, true);
    clockgate_enable(CLOCKGATE_NANDECC, true);
    S.pwrcon0_after = PWRCON(0);
    return (!(S.pwrcon0_after & (1u << CLOCKGATE_NAND)) && !(S.pwrcon0_after & (1u << CLOCKGATE_NANDECC))) ? 0u : 1u;
}

static uint32_t step_phys(void)
{
    int rc;
    invalidate_ftl();
    rc = nand_backend_init();
    geom_from_driver();
    snap_pcon();
    S.job_detail[0] = S.geom[0];
    S.job_detail[1] = (uint32_t)rc;
    return (uint32_t)rc;
}

static uint32_t step_ftl(uint32_t writable)
{
    int rc;
    ftl_mount_writable = (writable & 1) != 0;
    ftl_allow_unvalidated = (writable & 2) != 0;
    g_write_ready = 0;          /* every mount starts a new writer session */
    g_write_poisoned = 0;
    rc = ftl_init();
    if (rc == 0) {
        S.ftl_sectors = ftl_num_sectors();
        S.ftl_sector_bytes = N3G_SECTOR_BYTES;
        S.ftl_writable = !ftl_readonly_mount();
        S.job_detail[0] = S.ftl_sectors;
        S.job_detail[1] = N3G_SECTOR_BYTES;
        S.job_detail[2] = S.ftl_writable;
        g_ftl_ready = S.ftl_sectors != 0;
        if (!g_ftl_ready) rc = NAND_ERR_FTL;
    } else {
        g_ftl_ready = 0;
    }
    return (uint32_t)rc;
}

static uint32_t job_step(uint32_t step, uint32_t arg, uint32_t *result)
{
    uint32_t rc;
    if (step == 0 || step > N3G_STEP_FTL) { *result = N3G_R_BAD_ARG; return 0; }
    if ((step == N3G_STEP_PHYS && !need(1u << N3G_STEP_GATES)) ||
        (step == N3G_STEP_FTL && !need(1u << N3G_STEP_PHYS))) { *result = N3G_R_PRECOND; return 0; }
    S.step_status[step] = N3G_STEP_RUNNING;
    publish();
    switch (step) {
    case N3G_STEP_GATES: rc = step_gates(); break;
    case N3G_STEP_PINS:  snap_pcon(); rc = 0; break;
    case N3G_STEP_PHYS:  rc = step_phys(); break;
    default:             rc = step_ftl(arg); break;
    }
    S.step_status[step] = rc;
    if (rc == 0) S.step_ok_mask |= 1u << step;
    else { S.step_ok_mask &= ~(1u << step); *result = N3G_R_LIB_ERROR; }
    return rc;
}

static uint32_t job_backend(uint32_t idx, uint32_t *result)
{
    if (nand_backend_select(idx)) { *result = N3G_R_BAD_ARG; return 0; }
    invalidate_ftl();
    S.step_ok_mask &= ~(1u << N3G_STEP_PHYS);
    S.step_status[N3G_STEP_PHYS] = N3G_STEP_NOT_RUN;
    S.backend = idx;
    geom_from_driver();
    return 0;
}

/* ------------------------------------------------------------------ plain reads */
static uint32_t job_read_page(uint32_t bank, uint32_t page, uint32_t *result)
{
    const struct nand_geometry *g = nand_get_geometry();
    struct nand_read r;
    uint32_t i;
    int rc;
    if (!need(1u << N3G_STEP_PHYS) || !g) { *result = N3G_R_PRECOND; return 0; }
    if (bank >= g->banks || page >= g->blocks * g->pagesperblock) { *result = N3G_R_BAD_ARG; return 0; }
    memset(g_meta, 0xa5, 16);
    commit_dcache_range(g_meta, 16);
    r.bank = bank; r.page = page; r.buf = g_result + N3G_RP_DATA; r.meta = g_meta; r.ecc = 0; r.result = 0;
    rc = nand_read_pages(&r, 1);
    for (i = 0; i < 16; i++) g_result[N3G_RP_META + i] = i < 12 ? ((uint8_t *)g_meta)[i] : 0;
    *(uint32_t *)(g_result + N3G_RP_ECC) = (uint32_t)r.ecc;
    *(uint32_t *)(g_result + N3G_RP_RESULT) = r.result;
    S.job_detail[0] = (uint32_t)rc;
    S.job_detail[1] = (uint32_t)r.ecc;
    S.result_len = N3G_RP_BYTES;
    if (rc != 0 || r.ecc == NAND_ECC_FAILED) *result = N3G_R_LIB_ERROR;
    return (uint32_t)rc;
}

/* Spare records of every page of blocks [first, first + n) on every bank, as Rockbox's NAND check serves them
 * (nand-check-nano3g.c nand_check_load_spares): 8 pages per bank per call, the banks in parallel. Read only. */
static uint32_t job_meta_scan(uint32_t first, uint32_t nblocks, uint32_t *result)
{
    const struct nand_geometry *g = nand_get_geometry();
    uint32_t blk, p, q, bank, i, n, ppb;
    if (!need(1u << N3G_STEP_PHYS) || !g) { *result = N3G_R_PRECOND; return 0; }
    ppb = g->pagesperblock;
    if (!nblocks || first >= g->blocks || nblocks > g->blocks - first || g->banks * 8 > N3G_BENCH_MAXBATCH ||
        (uint64_t)nblocks * g->banks * ppb * N3G_MS_RECORD > N3G_RESULT_BYTES) { *result = N3G_R_BAD_ARG; return 0; }
    S.progress_total = nblocks;
    for (blk = 0; blk < nblocks; blk++) {
        for (p = 0; p < ppb; p += 8) {
            int rc;
            n = 0;
            for (q = p; q < p + 8 && q < ppb; q++)
                for (bank = 0; bank < g->banks; bank++, n++) {
                    g_rd[n].bank = bank;
                    g_rd[n].page = (first + blk) * ppb + q;
                    g_rd[n].buf = g_bbuf + n * 4096u;
                    g_rd[n].meta = g_meta + n * 4;
                    g_rd[n].ecc = 0;
                    g_rd[n].result = 0;
                }
            rc = nand_read_pages(g_rd, n);
            for (i = 0; i < n; i++) {
                uint8_t *rec = g_result + ((blk * g->banks + g_rd[i].bank) * ppb + (g_rd[i].page % ppb)) * N3G_MS_RECORD;
                int32_t ecc = rc ? -1 : g_rd[i].ecc;
                if (ecc >= 0) {
                    uint32_t bits = 8;
                    while (bits && !(g_rd[i].result & (1u << (bits - 1)))) bits--;
                    ecc |= bits << 8;
                    if (g_rd[i].result & (1u << 29)) ecc |= 0x10000;
                }
                memcpy(rec, g_meta + i * 4, NAND_META_WORDS * 4);
                memcpy(rec + 12, &ecc, 4);
                if (rc) S.job_detail[1]++;
            }
        }
        S.progress_done = blk + 1;
    }
    S.job_detail[0] = nblocks;
    S.result_len = nblocks * g->banks * ppb * N3G_MS_RECORD;
    return 0;
}

/* run one caller-supplied sequencer program (struct fmss_probe_req + program staged in the result buffer) and
   record the state; the physical layer is re-initialised afterwards (the program may have changed anything) */
static uint32_t job_fmss_probe(uint32_t bytes, uint32_t crc, uint32_t *result)
{
    const struct fmss_probe_req *q = (const struct fmss_probe_req *)g_result;
    struct fmss_probe_out *o = (struct fmss_probe_out *)(g_result + N3G_PROBE_OUT);
    int rc;
    if (!need(1u << N3G_STEP_GATES)) { *result = N3G_R_PRECOND; return 0; }
    if (bytes < N3G_PROBE_PROG || bytes > N3G_PROBE_OUT) { *result = N3G_R_BAD_ARG; return 0; }
    discard_dcache_range(g_result, bytes);
    if (crc32(g_result, bytes) != crc || q->prog_bytes + N3G_PROBE_PROG > bytes) { *result = N3G_R_BAD_ARG; return 0; }
    commit_discard_dcache_range(g_result + N3G_PROBE_SCRATCH, N3G_PROBE_SCRATCH_BYTES);
    rc = fmss_probe(q, (const uint32_t *)(g_result + N3G_PROBE_PROG), o);
    discard_dcache_range(g_result + N3G_PROBE_SCRATCH, N3G_PROBE_SCRATCH_BYTES);
    commit_dcache_range(o, sizeof(*o));
    invalidate_ftl();
    S.step_ok_mask &= ~(1u << N3G_STEP_PHYS);
    nand_backend_select(nand_backend_active_index());
    if (rc) { *result = N3G_R_BAD_ARG; return 0; }
    S.job_detail[0] = o->why;
    S.job_detail[1] = o->elapsed_us;
    S.job_detail[2] = o->nsamples;
    return 0;
}

static uint32_t job_ftl_read(uint32_t lpn, uint32_t count, uint32_t *result)
{
    uint32_t done = 0, worst = 0, total = S.ftl_sectors;
    if (!g_ftl_ready) { *result = N3G_R_PRECOND; return 0; }
    if (count == 0 || count * N3G_SECTOR_BYTES > N3G_RESULT_BYTES || lpn >= total || count > total - lpn) {
        *result = N3G_R_BAD_ARG; return 0;
    }
    S.progress_total = count;
    S.job_detail[1] = 0xffffffffu;
    while (done < count) {
        uint32_t n = count - done > N3G_FTL_CHUNK ? N3G_FTL_CHUNK : count - done;
        int rc = ftl_read(lpn + done, n, g_result + done * N3G_SECTOR_BYTES);
        if (rc != 0) {
            worst = (uint32_t)rc;
            S.job_detail[1] = lpn + done;
            break;
        }
        done += n;
        S.progress_done = done;
    }
    S.job_detail[0] = worst;
    S.result_len = done * N3G_SECTOR_BYTES;
    if (worst != 0) *result = N3G_R_LIB_ERROR;
    return worst;
}

#if N3G_NAND_WRITE
static uint32_t job_write(uint32_t kind, const uint32_t *arg, uint32_t *result)
{
    int rc = 0;
    if (!g_ftl_ready || g_write_poisoned || (kind != N3G_JOB_WRITE_OPEN && !g_write_ready)) {
        *result = N3G_R_PRECOND; return 0;
    }
    if (kind == N3G_JOB_BLOCK_WRITE) {
        uint32_t lba = arg[0], count = arg[1];
        if (!count || count > N3G_RESULT_BYTES / N3G_SECTOR_BYTES || lba >= S.ftl_sectors ||
            count > S.ftl_sectors - lba || crc32(g_result, count * N3G_SECTOR_BYTES) != arg[2]) {
            *result = N3G_R_BAD_ARG; return 0;
        }
        S.progress_total = count;
        rc = ftl_write(lba, count, g_result);
        if (!rc) S.progress_done = count;
    } else if (kind == N3G_JOB_WRITE_OPEN) {
        if (ftl_readonly_mount()) { *result = N3G_R_NOT_ALLOWED; return 0; }
        g_write_ready = 1;
    } else rc = ftl_sync();
    if (rc) { g_write_poisoned = 1; *result = N3G_R_LIB_ERROR; }
    return (uint32_t)rc;
}
#endif

/* ------------------------------------------------------------------ CPU meter */
static void load_thread(void *a0, void *a1, void *a2, void *a3)
{
    (void)a0; (void)a1; (void)a2; (void)a3;
    for (;;) {
        if (g_load_on) g_load_count++;
        else sleep(50000);
    }
}

/* every timed backend call: t0 = mt_start(); call; dt = mt_stop(t0) */
static uint32_t mt_start(void)
{
    g_mt_c0 = g_load_count;
    return read_usec_timer();
}

static uint32_t mt_stop(uint32_t t0)
{
    uint32_t dt = read_usec_timer() - t0;
    if (g_cpu && g_ph < 8) {
        g_cpu->phase_us[g_ph] += dt;
        g_cpu->phase_load[g_ph] += g_load_count - g_mt_c0;
    }
    return dt;
}

static void mt_corr(uint32_t result)
{
    if (g_cpu && g_ph < 8 && (result & 0xffu)) {
        uint32_t m = 8;
        while (m && !(result & (1u << (m - 1)))) m--;
        if (m > g_cpu->phase_maxcorr[g_ph]) g_cpu->phase_maxcorr[g_ph] = m;
    }
}

/* ------------------------------------------------------------------ benchmark */
struct bench {
    struct n3g_bench_params p;
    volatile struct n3g_bench_result *r;
    volatile uint32_t *rep;
    const struct nand_geometry *geo;
    uint32_t nbanks, entries;
};

static uint32_t xorshift(uint32_t *s)
{
    uint32_t x = *s ? *s : 0x2545f491u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}

static uint32_t pat_word(uint32_t seed, uint32_t bank, uint32_t page, uint32_t i)
{
    return seed ^ (bank << 24) ^ (page * 0x9e3779b1u) ^ (i * 0x85ebca6bu);
}

static void pat_fill(uint32_t *dst, uint32_t words, uint32_t seed, uint32_t bank, uint32_t page)
{
    uint32_t i;
    for (i = 0; i < words; i++) dst[i] = pat_word(seed, bank, page, i);
}

static uint32_t pat_mismatch(const uint32_t *src, uint32_t words, uint32_t seed, uint32_t bank, uint32_t page)
{
    uint32_t i, bad = 0;
    for (i = 0; i < words; i++) if (src[i] != pat_word(seed, bank, page, i)) bad++;
    return bad;
}

static void note_err(struct bench *b, int rc, uint32_t entry)
{
    if (!b->r->status) {
        b->r->status = (uint32_t)rc;
        b->r->first_err_rc = (uint32_t)rc;
        b->r->first_err_bank = entry >> 24;
        b->r->first_err_page = entry & 0xffffffu;
    }
}

/* page list of one repeat: entries[], returns N3G_R_* */
static uint32_t bench_entries(struct bench *b)
{
    const struct n3g_bench_params *p = &b->p;
    const struct nand_geometry *g = b->geo;
    uint32_t banks[8], nb = 0, bk, blk, pg, n = 0, i;
    for (bk = 0; bk < g->banks && bk < 8; bk++) if (p->bank_mask & (1u << bk)) banks[nb++] = bk;
    if (!nb || (p->bank_mask >> g->banks) != 0 || !p->nblocks || !p->pages || p->pages > g->pagesperblock ||
        p->first_block >= g->blocks || p->nblocks > g->blocks - p->first_block ||
        (uint64_t)nb * p->nblocks * p->pages > N3G_BENCH_MAXENTRIES) return N3G_R_BAD_ARG;
    b->nbanks = nb;
    if (p->flags & N3G_BF_INTERLEAVE) {
        for (blk = 0; blk < p->nblocks; blk++)
            for (pg = 0; pg < p->pages; pg++)
                for (i = 0; i < nb; i++)
                    g_entries[n++] = banks[i] << 24 | ((p->first_block + blk) * g->pagesperblock + pg);
    } else {
        for (i = 0; i < nb; i++)
            for (blk = 0; blk < p->nblocks; blk++)
                for (pg = 0; pg < p->pages; pg++)
                    g_entries[n++] = banks[i] << 24 | ((p->first_block + blk) * g->pagesperblock + pg);
    }
    if (p->op == N3G_BOP_READ && (p->flags & N3G_BF_SCATTER)) {
        uint32_t s = p->seed;
        for (i = n - 1; i > 0; i--) {
            uint32_t j = xorshift(&s) % (i + 1), t = g_entries[i];
            g_entries[i] = g_entries[j];
            g_entries[j] = t;
        }
    }
    b->entries = n;
    b->r->entries = n;
    return N3G_R_OK;
}

/* one pass of read batches over entries[]; compares against the pattern when verify */
static int bench_read_pass(struct bench *b, int verify, uint32_t *us_acc)
{
    const struct nand_geometry *g = b->geo;
    uint32_t words = g->pagesize / 4, off, i, bad = 0;
    for (off = 0; off < b->entries; off += b->p.batch) {
        uint32_t n = b->entries - off < b->p.batch ? b->entries - off : b->p.batch, t0, dt;
        int rc;
        for (i = 0; i < n; i++) {
            uint32_t e = g_entries[off + i];
            g_rd[i].bank = e >> 24;
            g_rd[i].page = e & 0xffffffu;
            g_rd[i].buf = g_bbuf + i * 4096u;
            g_rd[i].meta = g_meta + i * 4;
            g_rd[i].ecc = 0;
            g_rd[i].result = 0;
        }
        t0 = mt_start();
        rc = nand_read_pages(g_rd, n);
        dt = mt_stop(t0);
        *us_acc += dt;
        b->r->batches++;
        if (verify) b->r->verify_pages += n; else b->r->read_pages += n;
        if (!verify) {
            if (dt < b->r->batch_us_min) b->r->batch_us_min = dt;
            if (dt > b->r->batch_us_max) b->r->batch_us_max = dt;
        }
        if (rc != 0) {
            if (verify) b->r->verify_errors++; else b->r->read_errors++;
            note_err(b, rc, g_entries[off]);
            bad = 1;
            continue;
        }
        for (i = 0; i < n; i++) {
            uint32_t res = g_rd[i].result;
            if (g_rd[i].ecc == NAND_ECC_FAILED) { b->r->ecc_failed++; if (verify) b->r->verify_errors++; note_err(b, NAND_ECC_FAILED, g_entries[off + i]); }
            else if (g_rd[i].ecc == NAND_ECC_CORRECTED) b->r->ecc_corrected++;
            else b->r->ecc_clean++;
            if (res & (1u << 29)) b->r->erased_pages++;
            mt_corr(res);
            if (!verify && g_cpu && b->p.op == N3G_BOP_READ) {
                g_cpu->data_sig = g_cpu->data_sig * 31u + crc32(g_bbuf + i * 4096u, b->geo->pagesize)
                                + crc32(g_meta + i * 4, 12);
                g_cpu->sig_pages++;
                if (g_rd[i].ecc == NAND_ECC_FAILED) g_cpu->ecc_fail_pages++;
            }
            if (res & 0xffu) {
                uint32_t m = 8;
                while (m && !(res & (1u << (m - 1)))) m--;
                if (m > b->r->max_corrected) b->r->max_corrected = m;
            }
            if (verify) {
                uint32_t e = g_entries[off + i];
                if (pat_mismatch((const uint32_t *)(g_bbuf + i * 4096u), words, b->p.seed, e >> 24, e & 0xffffffu))
                    b->r->verify_mismatch++;
            }
        }
    }
    return bad;
}

static uint32_t bench_read(struct bench *b)
{
    uint32_t rep;
    b->r->batch_us_min = 0xffffffffu;
    for (rep = 0; rep < b->p.repeats; rep++) {
        uint32_t t0 = b->r->read_us, acc = t0;
        bench_read_pass(b, 0, &acc);
        b->r->read_us = acc;
        if (rep < N3G_BENCH_MAXREP) { b->rep[rep] = acc - t0; b->r->reps_recorded = rep + 1; }
        b->r->repeats_done = rep + 1;
        S.progress_done = rep + 1;
    }
    b->r->read_bytes = b->r->read_pages * b->geo->pagesize;
    if (b->r->batch_us_min == 0xffffffffu) b->r->batch_us_min = 0;
    return N3G_R_OK;
}

#if N3G_NAND_WRITE
/* erase g_er[0..n); returns 1 on failure */
static int bench_erase_run(struct bench *b, unsigned int n)
{
    unsigned int failidx = 0;
    uint32_t t0 = mt_start(), dt;
    int rc = nand_erase_blocks(g_er, n, &failidx);
    dt = mt_stop(t0);
    b->r->erase_us += dt;
    b->r->erase_blocks += rc ? failidx : n;
    if (rc) {
        b->r->erase_fail++;
        note_err(b, rc, g_er[failidx].bank << 24 | (g_er[failidx].block * b->geo->pagesperblock));
        return 1;
    }
    return 0;
}

static int bench_erase_all(struct bench *b)
{
    const struct n3g_bench_params *p = &b->p;
    uint32_t bk, blk;
    unsigned int n = 0;
    for (bk = 0; bk < b->geo->banks; bk++) {
        if (!(p->bank_mask & (1u << bk))) continue;
        for (blk = 0; blk < p->nblocks; blk++) {
            g_er[n].bank = bk;
            g_er[n].block = p->first_block + blk;
            if (++n == p->batch) {
                if (bench_erase_run(b, n)) return 1;
                n = 0;
            }
        }
    }
    return n ? bench_erase_run(b, n) : 0;
}

static int bench_program_all(struct bench *b)
{
    const struct nand_geometry *g = b->geo;
    uint32_t words = g->pagesize / 4, off, i;
    int two = (b->p.flags & N3G_BF_TWOPLANE) != 0;
    for (off = 0; off < b->entries; off += b->p.batch) {
        uint32_t n = b->entries - off < b->p.batch ? b->entries - off : b->p.batch, t0, dt, failbank = 0xffffffffu;
        int rc;
        for (i = 0; i < n; i++) {
            uint32_t e = g_entries[off + i];
            uint32_t *d = (uint32_t *)(g_bbuf + i * 4096u);
            pat_fill(d, words, b->p.seed, e >> 24, e & 0xffffffu);
            g_meta[i * 4 + 0] = 0xffffffffu;
            g_meta[i * 4 + 1] = 0;
            g_meta[i * 4 + 2] = 0x4e334257u;    /* "N3BW" marks bench pages */
            g_wr[i].bank = e >> 24;
            g_wr[i].page = e & 0xffffffu;
            g_wr[i].buf = d;
            g_wr[i].meta = g_meta + i * 4;
        }
        t0 = mt_start();
        rc = nand_write_pages(g_wr, n, two, &failbank);
        dt = mt_stop(t0);
        b->r->prog_us += dt;
        b->r->batches++;
        if (rc) {
            b->r->prog_fail++;
            note_err(b, rc, failbank != 0xffffffffu ? failbank << 24 : g_entries[off]);
            return 1;
        }
        b->r->prog_pages += n;
    }
    return 0;
}

/* entries for the two-plane program: plane pairs (block b, b+1, same page), page ascending */
static void bench_pair_entries(struct bench *b)
{
    const struct n3g_bench_params *p = &b->p;
    const struct nand_geometry *g = b->geo;
    uint32_t n = 0, bk, blk, pg;
    for (bk = 0; bk < g->banks; bk++) {
        if (!(p->bank_mask & (1u << bk))) continue;
        for (blk = 0; blk < p->nblocks; blk += 2)
            for (pg = 0; pg < p->pages; pg++) {
                uint32_t base = (p->first_block + blk) * g->pagesperblock + pg;
                g_entries[n++] = bk << 24 | base;
                g_entries[n++] = bk << 24 | (base + g->pagesperblock);
            }
    }
    b->entries = n;
    b->r->entries = n;
}

static uint32_t bench_write(struct bench *b)
{
    uint32_t rep;
    int two = (b->p.flags & N3G_BF_TWOPLANE) != 0;
    if (two) {
        if (!b->geo->twoplane || (b->p.first_block & 1) || (b->p.nblocks & 1)) return N3G_R_BAD_ARG;
        bench_pair_entries(b);
        if (b->p.batch & 1) b->p.batch &= ~1u;
        if (!b->p.batch) return N3G_R_BAD_ARG;
    }
    for (rep = 0; rep < b->p.repeats; rep++) {
        uint32_t t0 = read_usec_timer(), dummy = 0;
        if (bench_erase_all(b)) break;
        if (bench_program_all(b)) break;
        if (b->p.flags & N3G_BF_VERIFY) {
            bench_read_pass(b, 1, &dummy);
            b->r->verify_us += dummy;
        }
        if (rep < N3G_BENCH_MAXREP) { b->rep[rep] = read_usec_timer() - t0; b->r->reps_recorded = rep + 1; }
        b->r->repeats_done = rep + 1;
        S.progress_done = rep + 1;
    }
    b->r->prog_bytes = b->r->prog_pages * b->geo->pagesize;
    if (b->p.flags & N3G_BF_ERASE_AFTER) bench_erase_all(b);
    return N3G_R_OK;
}

/* Op CYCLE over whole blocks, page-major across the banks (entries[]):
 *   1 save:     read every page (data + spare metadata) into an SDRAM copy; an uncorrectable page or a failed
 *               read stops here, before anything is written (such a page could not be put back as it was)
 *   2,3,4:      erase, program the pattern (bench_program_all), read back and compare every page
 *   5,6,7:      erase, program the saved pages back (pages that were erased stay erased), read back and compare
 *               data and metadata with the copy
 * The restore runs whenever phase 2 started, also after a failure in 2..4. */
static void cyc_fail(struct bench *b, uint32_t phase)
{
    if (!b->r->cyc_failed_phase) b->r->cyc_failed_phase = phase;
}

/* restore-erase failures, one byte per (selected bank, block of the range) */
static uint32_t rbad_slot(struct bench *b, uint32_t bank, uint32_t block)
{
    uint32_t bk, bi = 0;
    for (bk = 0; bk < bank; bk++) if (b->p.bank_mask & (1u << bk)) bi++;
    return bi * b->p.nblocks + (block - b->p.first_block);
}
static void rbad_mark(struct bench *b, uint8_t *rbad, uint32_t bank, uint32_t block)
{
    rbad[rbad_slot(b, bank, block)] = 1;
}
static int rbad_get(struct bench *b, const uint8_t *rbad, uint32_t bank, uint32_t block)
{
    return rbad[rbad_slot(b, bank, block)];
}

/* erase g_er[0..cnt) for the restore; a failing block is marked and the rest of the run is still erased */
static void rst_erase_run(struct bench *b, uint8_t *rbad, uint32_t cnt)
{
    uint32_t start = 0;
    while (start < cnt) {
        unsigned int failidx = 0;
        uint32_t t0 = mt_start();
        int rc = nand_erase_blocks(g_er + start, cnt - start, &failidx);
        const struct nand_erase *f;
        b->r->rst_erase_us += mt_stop(t0);
        if (!rc) { b->r->rst_erase_blocks += cnt - start; return; }
        f = &g_er[start + failidx];
        b->r->rst_erase_blocks += failidx;
        b->r->rst_errors++;
        note_err(b, rc, f->bank << 24 | f->block * b->geo->pagesperblock);
        cyc_fail(b, N3G_CYC_RST_ERASE);
        rbad_mark(b, rbad, f->bank, f->block);
        start += failidx + 1;
    }
}

static uint32_t bench_cycle(struct bench *b)
{
    const struct nand_geometry *g = b->geo;
    uint32_t ps = g->pagesize, n = b->entries, off, i, k, acc;
    uint8_t *save = memalign(64, n * ps);
    uint32_t *smeta = memalign(64, n * 16u);
    uint8_t *serased = malloc(n);
    uint8_t *rbad = NULL;
    if (!save || !smeta || !serased) {
        if (save) free(save);
        if (smeta) free(smeta);
        if (serased) free(serased);
        return N3G_R_NO_MEMORY;
    }
    b->r->save_addr = (uint32_t)save;
    b->r->save_bytes = n * ps;
    S.progress_total = 7;

    /* 1: save */
    g_ph = S.progress_done = 1;
    for (off = 0; off < n && !b->r->cyc_failed_phase; off += b->p.batch) {
        uint32_t cnt = n - off < b->p.batch ? n - off : b->p.batch, t0;
        int rc;
        for (i = 0; i < cnt; i++) {
            uint32_t e = g_entries[off + i];
            g_rd[i].bank = e >> 24;
            g_rd[i].page = e & 0xffffffu;
            g_rd[i].buf = save + (off + i) * ps;
            g_rd[i].meta = smeta + (off + i) * 4;
            g_rd[i].ecc = 0;
            g_rd[i].result = 0;
        }
        t0 = mt_start();
        rc = nand_read_pages(g_rd, cnt);
        b->r->save_us += mt_stop(t0);
        b->r->batches++;
        if (rc) { note_err(b, rc, g_entries[off]); cyc_fail(b, N3G_CYC_SAVE); break; }
        for (i = 0; i < cnt; i++) {
            serased[off + i] = (g_rd[i].result >> 29) & 1;
            mt_corr(g_rd[i].result);
            if (serased[off + i]) b->r->save_erased++;
            if (g_rd[i].ecc == NAND_ECC_FAILED) {
                b->r->ecc_failed++;
                note_err(b, NAND_ECC_FAILED, g_entries[off + i]);
                cyc_fail(b, N3G_CYC_SAVE);
            }
            else if (g_rd[i].ecc == NAND_ECC_CORRECTED) b->r->ecc_corrected++;
            else b->r->ecc_clean++;
        }
        b->r->save_pages += cnt;
    }
    if (b->r->cyc_failed_phase) goto out;

    /* 2..4: pattern */
    g_ph = S.progress_done = 2;
    if (bench_erase_all(b)) { cyc_fail(b, N3G_CYC_ERASE); goto restore; }
    g_ph = S.progress_done = 3;
    if (bench_program_all(b)) { cyc_fail(b, N3G_CYC_PROGRAM); goto restore; }
    b->r->prog_bytes = b->r->prog_pages * ps;
    g_ph = S.progress_done = 4;
    acc = 0;
    if (bench_read_pass(b, 1, &acc) || b->r->verify_mismatch) cyc_fail(b, N3G_CYC_VERIFY);
    b->r->verify_us += acc;

restore:
    /* 5: erase again. A block that fails is skipped (not given up on): every block that erased gets its pages back
     * in step 6, and the failed ones count in rst_errors and show up as mismatches in step 7. */
    g_ph = S.progress_done = 5;
    rbad = malloc(b->nbanks * b->p.nblocks);
    if (!rbad) { cyc_fail(b, N3G_CYC_RST_ERASE); b->r->rst_errors++; goto out; }
    memset(rbad, 0, b->nbanks * b->p.nblocks);
    {
        uint32_t bk, blk, cnt = 0;
        for (bk = 0; bk < g->banks; bk++) {
            if (!(b->p.bank_mask & (1u << bk))) continue;
            for (blk = 0; blk < b->p.nblocks; blk++) {
                g_er[cnt].bank = bk;
                g_er[cnt].block = b->p.first_block + blk;
                if (++cnt == b->p.batch) { rst_erase_run(b, rbad, cnt); cnt = 0; }
            }
        }
        if (cnt) rst_erase_run(b, rbad, cnt);
    }

    /* 6: program the saved pages, in entry order, skipping the ones that were erased and those of blocks that
     * did not erase */
    g_ph = S.progress_done = 6;
    for (off = 0; off < n; ) {
        uint32_t cnt = 0, t0, failbank = 0xffffffffu;
        int rc;
        while (off < n && cnt < b->p.batch) {
            uint32_t e = g_entries[off];
            if (!serased[off] && !rbad_get(b, rbad, e >> 24, (e & 0xffffffu) / g->pagesperblock)) {
                g_wr[cnt].bank = e >> 24;
                g_wr[cnt].page = e & 0xffffffu;
                g_wr[cnt].buf = save + off * ps;
                g_wr[cnt].meta = smeta + off * 4;
                cnt++;
            }
            off++;
        }
        if (!cnt) continue;
        t0 = mt_start();
        rc = nand_write_pages(g_wr, cnt, false, &failbank);
        b->r->rst_prog_us += mt_stop(t0);
        if (rc) {
            b->r->rst_errors++;
            note_err(b, rc, failbank != 0xffffffffu ? failbank << 24 : (g_wr[0].bank << 24 | g_wr[0].page));
            cyc_fail(b, N3G_CYC_RST_PROGRAM);
            continue;   /* put back as much as possible */
        }
        b->r->rst_prog_pages += cnt;
    }

    /* 7: read back and compare with the copy */
    g_ph = S.progress_done = 7;
    for (off = 0; off < n; off += b->p.batch) {
        uint32_t cnt = n - off < b->p.batch ? n - off : b->p.batch, t0;
        int rc;
        for (i = 0; i < cnt; i++) {
            uint32_t e = g_entries[off + i];
            g_rd[i].bank = e >> 24;
            g_rd[i].page = e & 0xffffffu;
            g_rd[i].buf = g_bbuf + i * 4096u;
            g_rd[i].meta = g_meta + i * 4;
            g_rd[i].ecc = 0;
            g_rd[i].result = 0;
        }
        t0 = mt_start();
        rc = nand_read_pages(g_rd, cnt);
        b->r->rst_verify_us += mt_stop(t0);
        b->r->rst_verify_pages += cnt;
        if (rc) { b->r->rst_errors++; b->r->rst_mismatch += cnt; note_err(b, rc, g_entries[off]); continue; }
        for (i = 0; i < cnt; i++) {
            uint32_t idx = off + i;
            int bad;
            mt_corr(g_rd[i].result);
            if (g_rd[i].ecc == NAND_ECC_FAILED) { b->r->rst_errors++; b->r->rst_mismatch++; continue; }
            if (serased[idx]) bad = !((g_rd[i].result >> 29) & 1);
            else bad = memcmp(g_bbuf + i * 4096u, save + idx * ps, ps) != 0
                    || memcmp(g_meta + i * 4, smeta + idx * 4, NAND_META_WORDS * 4) != 0;
            if (bad) b->r->rst_mismatch++;
        }
    }
    if (b->r->rst_mismatch) cyc_fail(b, N3G_CYC_RST_VERIFY);

out:
    if (rbad) free(rbad);
    free(serased);
    free(smeta);
    free(save);
    b->r->repeats_done = 1;
    return N3G_R_OK;
}
#endif

static uint32_t bench_ftl_read(struct bench *b)
{
    const struct n3g_bench_params *p = &b->p;
    uint32_t total = ftl_num_sectors(), rep, win = p->sectors, s = p->seed;
    if (!win || win > total || p->first_sector > total - win || !p->batch || p->batch > N3G_BENCH_MAXFTLBATCH)
        return N3G_R_BAD_ARG;
    for (rep = 0; rep < p->repeats; rep++) {
        uint32_t off = 0, acc = 0;
        while (off < win) {
            uint32_t n = win - off < p->batch ? win - off : p->batch, start, t0, dt;
            int rc;
            if (p->flags & N3G_BF_SCATTER) start = p->first_sector + xorshift(&s) % (win - n + 1);
            else start = p->first_sector + off;
            t0 = mt_start();
            rc = ftl_read(start, n, g_bbuf);
            dt = mt_stop(t0);
            acc += dt;
            b->r->batches++;
            if (rc) { b->r->ftl_errors++; note_err(b, rc, start & 0xffffffu); }
            else b->r->ftl_sectors += n;
            off += n;
        }
        b->r->ftl_us += acc;
        if (rep < N3G_BENCH_MAXREP) { b->rep[rep] = acc; b->r->reps_recorded = rep + 1; }
        b->r->repeats_done = rep + 1;
        S.progress_done = rep + 1;
    }
    b->r->ftl_bytes = b->r->ftl_sectors * N3G_SECTOR_BYTES;
    return N3G_R_OK;
}

#if N3G_NAND_WRITE
static uint32_t bench_ftl_cycle(struct bench *b);
#endif

static uint32_t job_bench(uint32_t *result)
{
    struct bench b;
    uint32_t t0, rc = N3G_R_OK, i;
    const struct nand_geometry *g;
    discard_dcache_range(g_result, N3G_BENCH_PARAM_BYTES);
    memcpy(&b.p, g_result, sizeof(b.p));
    b.r = (volatile struct n3g_bench_result *)(g_result + N3G_BENCH_RES_OFF);
    b.rep = (volatile uint32_t *)(g_result + N3G_BENCH_REP_OFF);
    for (i = 0; i < N3G_BENCH_RESULT_BYTES / 4; i++) ((volatile uint32_t *)b.r)[i] = 0;
    b.r->magic = N3G_BENCH_RESULT_MAGIC;
    b.r->version = N3G_BENCH_VERSION;
    b.r->op = b.p.op;
    b.r->flags = b.p.flags;
    S.result_len = N3G_BENCH_REP_OFF + 4u * N3G_BENCH_MAXREP;
    if (b.p.magic != N3G_BENCH_PARAM_MAGIC || b.p.version != N3G_BENCH_VERSION || !b.p.repeats ||
        b.p.op < N3G_BOP_READ || b.p.op > N3G_BOP_FTL_CYCLE) { *result = N3G_R_BAD_ARG; return 0; }
    if (b.p.op == N3G_BOP_WRITE || b.p.op == N3G_BOP_CYCLE || b.p.op == N3G_BOP_FTL_CYCLE) {
#if N3G_NAND_WRITE
        if (!(b.p.flags & N3G_BF_WRITE)) { *result = N3G_R_NOT_ALLOWED; return 0; }
#else
        *result = N3G_R_NOT_ALLOWED; return 0;
#endif
    }
    /* backend selection / init for this run */
    if (b.p.backend != 0xffffffffu && (b.p.backend != nand_backend_active_index() || !nand_backend_ready())) {
        int irc;
        if (!need(1u << N3G_STEP_GATES)) { *result = N3G_R_PRECOND; return 0; }
        if (nand_backend_select(b.p.backend)) { *result = N3G_R_BAD_ARG; return 0; }
        invalidate_ftl();
        irc = nand_backend_init();
        S.backend = b.p.backend;
        geom_from_driver();
        if (irc) {
            S.step_ok_mask &= ~(1u << N3G_STEP_PHYS);
            b.r->backend = b.p.backend;
            b.r->status = (uint32_t)irc;
            *result = N3G_R_LIB_ERROR;
            return (uint32_t)irc;
        }
        S.step_ok_mask |= 1u << N3G_STEP_PHYS;
    }
    /* an FTL read needs a mount on this backend: a backend switch (or a physical cycle) drops it, and the backend
       may already be the active one, so mount whenever there is none (read-only) */
    if (b.p.op == N3G_BOP_FTL_READ && !g_ftl_ready && (S.step_ok_mask & (1u << N3G_STEP_PHYS))) {
        int irc = (int)step_ftl(0);
        if (irc) { b.r->status = (uint32_t)irc; *result = N3G_R_LIB_ERROR; return (uint32_t)irc; }
        S.step_ok_mask |= 1u << N3G_STEP_FTL;
    }
    b.r->backend = nand_backend_active_index();
    g = nand_get_geometry();
    if (!need(1u << N3G_STEP_PHYS) || !g) { *result = N3G_R_PRECOND; return 0; }
#if N3G_NAND_WRITE
    if (b.p.op == N3G_BOP_FTL_CYCLE) {
        /* this run's own writable mount (with the explicit override on an unvalidated row) */
        int irc;
        if (!g->validated && !(b.p.flags & N3G_BF_UNVALIDATED)) { *result = N3G_R_NOT_ALLOWED; return 0; }
        invalidate_ftl();
        irc = (int)step_ftl(1u | ((b.p.flags & N3G_BF_UNVALIDATED) ? 2u : 0u));
        if (irc) { b.r->status = (uint32_t)irc; *result = N3G_R_LIB_ERROR; return (uint32_t)irc; }
        S.step_ok_mask |= 1u << N3G_STEP_FTL;
        if (ftl_readonly_mount()) { *result = N3G_R_NOT_ALLOWED; return 0; }
    }
#endif
    b.geo = g;
    b.r->pages_per_block = g->pagesperblock;
    b.r->page_size = g->pagesize;
    if (!b.p.batch) b.p.batch = 1;
    if (b.p.op == N3G_BOP_FTL_READ || b.p.op == N3G_BOP_FTL_CYCLE) {
        if (!g_ftl_ready) { *result = N3G_R_PRECOND; return 0; }
    } else if (b.p.batch > N3G_BENCH_MAXBATCH) { *result = N3G_R_BAD_ARG; return 0; }
    if (b.p.op == N3G_BOP_CYCLE) {
        /* whole blocks, page-major across the banks; the FTL's view of these blocks is dropped */
        if (b.p.pages != g->pagesperblock) { *result = N3G_R_BAD_ARG; return 0; }
        b.p.flags = (b.p.flags | N3G_BF_INTERLEAVE) & ~(N3G_BF_SCATTER | N3G_BF_TWOPLANE);
        invalidate_ftl();
    }
    if (b.p.op != N3G_BOP_FTL_READ && b.p.op != N3G_BOP_FTL_CYCLE) {
        uint32_t er = bench_entries(&b);
        if (er != N3G_R_OK) { *result = er; return 0; }
    }
    if (b.p.op == N3G_BOP_WRITE || b.p.op == N3G_BOP_CYCLE) {
        if (!g->validated && !(b.p.flags & N3G_BF_UNVALIDATED)) { *result = N3G_R_NOT_ALLOWED; return 0; }
        if (b.p.flags & N3G_BF_SCATTER) { *result = N3G_R_BAD_ARG; return 0; }
    }
    S.progress_total = b.p.repeats;
    /* CPU meter: phase 0 unless a cycle sets it; with N3G_BF_LOAD calibrate the load thread alone first */
    {
        volatile struct n3g_bench_cpu *c = (volatile struct n3g_bench_cpu *)(g_result + N3G_BENCH_CPU_OFF);
        uint32_t c0, u0, du;
        for (i = 0; i < sizeof(*c) / 4; i++) ((volatile uint32_t *)c)[i] = 0;
        c->magic = N3G_BENCH_CPU_MAGIC;
        if (b.p.fmss_poll_us != 0xffffffffu) fmss_poll_us = b.p.fmss_poll_us;
        c->timing = b.p.timing;
        nand_sw_set_timing(b.p.timing);
        nand_fmss_set_raw_timing(b.p.timing);
        c->fmss_poll_us = fmss_poll_us;
        g_ph = 0;
        if (b.p.flags & N3G_BF_LOAD) {
            g_load_on = 1;
            sleep(20000);                        /* let it start */
            c0 = g_load_count; u0 = read_usec_timer();
            sleep(200000);                       /* the worker blocked: the load thread runs alone */
            du = read_usec_timer() - u0;
            c->load = 1;
            c->load_rate_x1000 = du >= 1000 ? (g_load_count - c0) / (du / 1000) : 0;   /* counts per ms */
        }
        g_cpu = c;
    }
    t0 = read_usec_timer();
    switch (b.p.op) {
    case N3G_BOP_READ:     rc = bench_read(&b); break;
#if N3G_NAND_WRITE
    case N3G_BOP_WRITE:    rc = bench_write(&b); break;
    case N3G_BOP_CYCLE:    rc = bench_cycle(&b); break;
    case N3G_BOP_FTL_CYCLE:
        rc = bench_ftl_cycle(&b);
        invalidate_ftl();                       /* leave it read-only: mount again without the write flag */
        if (!step_ftl(0)) S.step_ok_mask |= 1u << N3G_STEP_FTL;
        break;
#endif
    default:               rc = bench_ftl_read(&b); break;
    }
    b.r->total_us = read_usec_timer() - t0;
    g_load_on = 0;
    g_cpu = NULL;
    if (b.p.timing) { nand_sw_set_timing(0); nand_fmss_set_raw_timing(0); }
    if (rc != N3G_R_OK) *result = rc;
    else if (b.r->status) *result = N3G_R_LIB_ERROR;
    S.job_detail[0] = b.r->status;
    S.job_detail[1] = b.r->repeats_done;
    return b.r->status;
}

#if N3G_NAND_WRITE
/* Op FTL_CYCLE: logical sectors through the FTL (ftl_read / ftl_write / ftl_sync), see nand3g.h. The FTL is
 * mounted writable by job_bench before; a failed ftl_write or ftl_sync makes the FTL refuse further writes, so
 * the restore can then only be attempted, and the result says so. */
static uint32_t bench_ftl_cycle(struct bench *b)
{
    const struct n3g_bench_params *p = &b->p;
    uint32_t n = p->sectors, first = p->first_sector, off, i, t0;
    uint8_t *save;
    int rc;
    if (!n || first >= ftl_num_sectors() || n > ftl_num_sectors() - first || !p->batch ||
        p->batch > N3G_BENCH_MAXFTLBATCH || p->batch * N3G_SECTOR_BYTES > BENCH_BUF_BYTES) return N3G_R_BAD_ARG;
    save = memalign(64, n * N3G_SECTOR_BYTES);
    if (!save) return N3G_R_NO_MEMORY;
    b->r->save_addr = (uint32_t)save;
    b->r->save_bytes = n * N3G_SECTOR_BYTES;
    S.progress_total = 7;

    /* 1: save */
    g_ph = S.progress_done = 1;
    for (off = 0; off < n; off += p->batch) {
        uint32_t cnt = n - off < p->batch ? n - off : p->batch;
        t0 = mt_start();
        rc = ftl_read(first + off, cnt, save + off * N3G_SECTOR_BYTES);
        b->r->ftl_us += mt_stop(t0);
        b->r->batches++;
        if (rc) { b->r->ftl_errors++; note_err(b, rc, first + off); cyc_fail(b, N3G_CYC_SAVE); goto out; }
        b->r->ftl_sectors += cnt;
    }
    b->r->ftl_bytes = b->r->ftl_sectors * N3G_SECTOR_BYTES;

    /* 3, 2: pattern write, sync */
    g_ph = S.progress_done = 3;
    for (off = 0; off < n; off += p->batch) {
        uint32_t cnt = n - off < p->batch ? n - off : p->batch;
        for (i = 0; i < cnt; i++)
            pat_fill((uint32_t *)(g_bbuf + i * N3G_SECTOR_BYTES), N3G_SECTOR_BYTES / 4, p->seed, 0xff, first + off + i);
        t0 = mt_start();
        rc = ftl_write(first + off, cnt, g_bbuf);
        b->r->ftl_wr_us += mt_stop(t0);
        if (rc) { b->r->prog_fail++; note_err(b, rc, first + off); cyc_fail(b, N3G_CYC_PROGRAM); goto restore; }
        b->r->prog_pages += cnt;
    }
    g_ph = S.progress_done = 2;
    t0 = mt_start();
    rc = ftl_sync();
    b->r->ftl_sync_us = mt_stop(t0);
    if (rc) { note_err(b, rc, first); cyc_fail(b, N3G_CYC_ERASE); goto restore; }

    /* 4: read back */
    g_ph = S.progress_done = 4;
    for (off = 0; off < n; off += p->batch) {
        uint32_t cnt = n - off < p->batch ? n - off : p->batch;
        t0 = mt_start();
        rc = ftl_read(first + off, cnt, g_bbuf);
        b->r->ftl_vfy_us += mt_stop(t0);
        if (rc) { b->r->verify_errors++; b->r->ftl_vfy_mismatch += cnt; note_err(b, rc, first + off); continue; }
        for (i = 0; i < cnt; i++)
            if (pat_mismatch((const uint32_t *)(g_bbuf + i * N3G_SECTOR_BYTES), N3G_SECTOR_BYTES / 4, p->seed, 0xff,
                             first + off + i))
                b->r->ftl_vfy_mismatch++;
    }
    if (b->r->ftl_vfy_mismatch) cyc_fail(b, N3G_CYC_VERIFY);

restore:
    /* 6, 5: write the saved sectors back, sync */
    g_ph = S.progress_done = 6;
    for (off = 0; off < n; off += p->batch) {
        uint32_t cnt = n - off < p->batch ? n - off : p->batch;
        t0 = mt_start();
        rc = ftl_write(first + off, cnt, save + off * N3G_SECTOR_BYTES);
        b->r->ftl_rst_wr_us += mt_stop(t0);
        if (rc) { b->r->rst_errors++; note_err(b, rc, first + off); cyc_fail(b, N3G_CYC_RST_PROGRAM); break; }
        b->r->rst_prog_pages += cnt;
    }
    g_ph = S.progress_done = 5;
    t0 = mt_start();
    rc = ftl_sync();
    b->r->ftl_rst_sync_us = mt_stop(t0);
    if (rc) { b->r->rst_errors++; note_err(b, rc, first); cyc_fail(b, N3G_CYC_RST_ERASE); }

    /* 7: read back and compare with the copy */
    g_ph = S.progress_done = 7;
    for (off = 0; off < n; off += p->batch) {
        uint32_t cnt = n - off < p->batch ? n - off : p->batch;
        t0 = mt_start();
        rc = ftl_read(first + off, cnt, g_bbuf);
        b->r->ftl_rst_vfy_us += mt_stop(t0);
        if (rc) { b->r->rst_errors++; b->r->ftl_rst_mismatch += cnt; note_err(b, rc, first + off); continue; }
        for (i = 0; i < cnt; i++)
            if (memcmp(g_bbuf + i * N3G_SECTOR_BYTES, save + (off + i) * N3G_SECTOR_BYTES, N3G_SECTOR_BYTES))
                b->r->ftl_rst_mismatch++;
    }
    b->r->rst_mismatch = b->r->ftl_rst_mismatch;
    if (b->r->ftl_rst_mismatch) cyc_fail(b, N3G_CYC_RST_VERIFY);
out:
    free(save);
    b->r->repeats_done = 1;
    return N3G_R_OK;
}
#endif

/* ------------------------------------------------------------------ job runner */
static void run_job(uint32_t kind, const uint32_t *arg)
{
    uint32_t result = N3G_R_OK, rc = 0;
    S.state = N3G_STATE_RUNNING;               /* record BEFORE the first register access */
    S.job_t_start = read_usec_timer();
    S.job_t_end = 0;
    S.progress_done = 0;
    S.progress_total = 1;
    S.result_len = 0;
    publish();

    switch (kind) {
    case N3G_JOB_STEP:       rc = job_step(arg[0], arg[1], &result); break;
    case N3G_JOB_READ_PAGE:  rc = job_read_page(arg[0], arg[1], &result); break;
    case N3G_JOB_FTL_READ:   rc = job_ftl_read(arg[0], arg[1], &result); break;
    case N3G_JOB_BACKEND:    rc = job_backend(arg[0], &result); break;
    case N3G_JOB_BENCH:      rc = job_bench(&result); break;
    case N3G_JOB_META_SCAN:  rc = job_meta_scan(arg[0], arg[1], &result); break;
    case N3G_JOB_FMSS_PROBE: rc = job_fmss_probe(arg[0], arg[1], &result); break;
#if N3G_NAND_WRITE
    case N3G_JOB_WRITE_OPEN:
    case N3G_JOB_BLOCK_WRITE:
    case N3G_JOB_FLUSH:      rc = job_write(kind, arg, &result); break;
#endif
    default:                 result = N3G_R_BAD_ARG; break;
    }
    if (S.result_len) commit_dcache_range(g_result, S.result_len);
    S.job_lib_rc = rc;
    S.job_result = result;
    if (S.progress_total == 1 && result == N3G_R_OK) S.progress_done = 1;
    S.job_t_end = read_usec_timer();
    refresh_live();
    S.state = N3G_STATE_DONE;
    S.jobs_done = S.jobs_started;
    S.busy = 0;
    publish();
}

static void worker(void *a0, void *a1, void *a2, void *a3)
{
    (void)a0; (void)a1; (void)a2; (void)a3;
    while (1) {
        uint32_t kind, arg[3], mode;
        wakeup_wait(&g_wake, TIMEOUT_BLOCK);
        mode = enter_critical_section();
        kind = g_job.kind; arg[0] = g_job.arg[0]; arg[1] = g_job.arg[1]; arg[2] = g_job.arg[2];
        g_job.kind = N3G_JOB_NONE;
        leave_critical_section(mode);
        if (kind != N3G_JOB_NONE) run_job(kind, arg);
    }
}

/* ------------------------------------------------------------------ public */
int n3g_nand_init(void)
{
    uint32_t i;
    void *stack;
    if (g_ready) return 0;
    for (i = 0; i < N3G_NSTEPS; i++) S.step_status[i] = N3G_STEP_NOT_RUN;
    S.magic = N3G_NAND_MAGIC;
    S.version = N3G_NAND_VERSION;
    S.size = N3G_STATUS_BYTES;
    g_result = memalign(64, N3G_RESULT_BYTES);
    g_bbuf = memalign(64, BENCH_BUF_BYTES);
    g_meta = memalign(64, N3G_BENCH_MAXBATCH * 16u);
    stack = memalign(32, 0x8000);
    if (!g_result || !g_bbuf || !g_meta || !stack) {
        cprintf(CONSOLE_BOOT, "[nand] harness: out of memory\n");
        return -1;
    }
    reownalloc(g_result, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    reownalloc(g_bbuf, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    reownalloc(g_meta, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    reownalloc(stack, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    memset(g_result, 0, N3G_RESULT_BYTES);
    commit_dcache_range(g_result, N3G_RESULT_BYTES);
    S.result_addr = (uint32_t)g_result;
    S.result_cap = N3G_RESULT_BYTES;
    wakeup_init(&g_wake);
    /* below the USB monitor worker: a long FTL mount leaves the async monitor responsive */
    if (!thread_create(&g_thread, "nand worker", worker, stack, 0x8000, CORE_THREAD, 64, true, NULL, NULL, NULL, NULL)) {
        cprintf(CONSOLE_BOOT, "[nand] harness: thread_create failed\n");
        return -1;
    }
    {
        /* CPU meter load thread: sleeps unless a benchmark asks for load (N3G_BF_LOAD); same priority as the worker */
        void *lstack = memalign(32, 0x1000);
        if (lstack) {
            reownalloc(lstack, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
            thread_create(&g_load_thread, "nand cpu load", load_thread, lstack, 0x1000, CORE_THREAD, 64, true,
                          NULL, NULL, NULL, NULL);
        }
    }
    g_ready = 1;
    publish();
    return 0;
}

void n3g_nand_heartbeat(uint32_t *w0, uint32_t *w1)
{
    *w0 = (S.busy ? 0x80000000u : 0u) | (S.state & 0x7fu) << 24 | (S.job_kind & 0xffu) << 16 |
          (S.step_ok_mask & 0xffu) << 8 | (S.jobs_done & 0xffu);
    *w1 = S.jobs_started;
}

static uint32_t queue(uint32_t kind, uint32_t a0, uint32_t a1, uint32_t a2)
{
    uint32_t mode, i;
    if (!g_ready) return N3G_ACCEPT_NOT_INIT;
    mode = enter_critical_section();
    if (S.busy) { leave_critical_section(mode); return N3G_ACCEPT_BUSY; }
    S.busy = 1;
    S.state = N3G_STATE_QUEUED;
    S.jobs_started++;
    S.job_kind = kind;
    S.job_arg[0] = a0; S.job_arg[1] = a1; S.job_arg[2] = a2;
    S.job_result = 0; S.job_lib_rc = 0;
    for (i = 0; i < 4; i++) S.job_detail[i] = 0;
    g_job.kind = kind; g_job.arg[0] = a0; g_job.arg[1] = a1; g_job.arg[2] = a2;
    leave_critical_section(mode);
    wakeup_signal(&g_wake);
    return N3G_ACCEPT_OK;
}

void n3g_nand_usb_command(const uint32_t *buf, uint32_t *reply)
{
    uint32_t acc;
    reply[0] = 1;
    switch (buf[0]) {
    case N3G_NAND_WRITE_CAPS:
        reply[1] = 1; reply[2] = N3G_NAND_WRITE ? 1 : 0;
        reply[3] = N3G_RESULT_BYTES / N3G_SECTOR_BYTES; return;
#if N3G_NAND_WRITE
    case N3G_NAND_WRITE_OPEN:  acc = queue(N3G_JOB_WRITE_OPEN, 0, 0, 0); break;
    case N3G_NAND_BLOCK_WRITE: acc = queue(N3G_JOB_BLOCK_WRITE, buf[1], buf[2], buf[3]); break;
    case N3G_NAND_FLUSH:       acc = queue(N3G_JOB_FLUSH, 0, 0, 0); break;
#endif
    case N3G_NAND_STATUS:     /* -> {status record address, size, result buffer address} */
        if (g_ready) refresh_live();
        reply[1] = (uint32_t)&n3g_nand_status;
        reply[2] = N3G_STATUS_BYTES;
        reply[3] = S.result_addr;
        return;
    case N3G_NAND_STEP:       acc = queue(N3G_JOB_STEP, buf[1], buf[2], 0); break;
    case N3G_NAND_READ_PAGE:  acc = queue(N3G_JOB_READ_PAGE, buf[1], buf[2], 0); break;
    case N3G_NAND_FTL_OPEN:   acc = queue(N3G_JOB_STEP, N3G_STEP_FTL, buf[1], 0); break;
    case N3G_NAND_FTL_READ:   acc = queue(N3G_JOB_FTL_READ, buf[1], buf[2], 0); break;
    case N3G_NAND_BACKEND:    acc = queue(N3G_JOB_BACKEND, buf[1], 0, 0); break;
    case N3G_NAND_BENCH:      acc = queue(N3G_JOB_BENCH, 0, 0, 0); break;
    case N3G_NAND_META_SCAN:  acc = queue(N3G_JOB_META_SCAN, buf[1], buf[2], 0); break;
    case N3G_NAND_FMSS_PROBE: acc = queue(N3G_JOB_FMSS_PROBE, buf[1], buf[2], 0); break;
    default: reply[0] = 2; return;
    }
    reply[1] = acc;
    reply[2] = S.jobs_started;            /* job id when accepted */
    reply[3] = (uint32_t)&n3g_nand_status;
}
