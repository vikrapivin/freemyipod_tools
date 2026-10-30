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

volatile struct n3g_nand_status n3g_nand_status;
#define S n3g_nand_status

_Static_assert(sizeof(struct n3g_nand_status) == N3G_STATUS_BYTES, "status record layout (nano3g/nand.py mirrors it)");


static struct scheduler_thread g_thread;
static struct wakeup g_wake;
static uint8_t *g_result;                 /* N3G_RESULT_BYTES */
static uint32_t *g_meta;                  /* 4 words of spare metadata */
static int g_ready, g_ftl_ready;
static int g_write_ready, g_write_poisoned;   /* writer state of the current mount (jobs 7..9) */
static struct { uint32_t kind, arg[3]; } g_job;

/* ------------------------------------------------------------------ status record */
static void publish(void)
{
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
    g_meta = memalign(64, 16u);
    stack = memalign(32, 0x8000);
    if (!g_result || !g_meta || !stack) {
        cprintf(CONSOLE_BOOT, "[nand] harness: out of memory\n");
        return -1;
    }
    reownalloc(g_result, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
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
    default: reply[0] = 2; return;
    }
    reply[1] = acc;
    reply[2] = S.jobs_started;            /* job id when accepted */
    reply[3] = (uint32_t)&n3g_nand_status;
}
