/* SPDX-License-Identifier: GPL-2.0-or-later
 * nano4g physical NAND/M2 experiment worker, using the adapted Rockbox driver.
 * USB handlers queue jobs; only the worker accesses NAND. No mount at startup.
 * The legacy 0x250-byte status layout remains stable for existing host tools.
 */
#include "global.h"
#include "thread.h"
#include "timer.h"
#include "malloc.h"
#include "console.h"
#include "clockgates.h"
#include "contextswitch.h"
#include "i2c.h"
#include "crc32.h"
#include "nand_poc.h"
#include "fmss_probe.h"
#include "fmss_batch.h"
#include "cpu_nand.h"
#include "cpu_nand_stack.h"
#include "nand_compare.h"
#include "nand_hal_emcore.h"
#include "nand/nand-target.h"
#include "nand/rockbox-compat.h"

extern int ftl_init(void);
extern int ftl_read(uint32_t sector, uint32_t count, void *buffer);
extern uint32_t ftl_num_sectors(void);

volatile struct nand_poc_status nand_poc_status;
#define S nand_poc_status
_Static_assert(sizeof(S) == NPOC_STATUS_BYTES, "status ABI");
_Static_assert(sizeof(struct fmss_probe_req) == 44 + 96 * 8, "request ABI");
_Static_assert(sizeof(struct fmss_probe_req) <= N4G_PROBE_PROG, "request overlap");
_Static_assert(__builtin_offsetof(struct fmss_probe_out, before) == 88, "header ABI");
_Static_assert(__builtin_offsetof(struct fmss_probe_out, s) == 2136, "sample offset ABI");
_Static_assert(sizeof(struct fmss_probe_sample) == 40, "sample ABI");
_Static_assert(__builtin_offsetof(struct fmss_probe_out, fullmap) == 43096, "fullmap ABI");
_Static_assert(N4G_PROBE_OUT + sizeof(struct fmss_probe_out) <= N4G_PROBE_SCRATCH, "output overlap");
_Static_assert(N4G_PROBE_SCRATCH + N4G_PROBE_SCRATCH_BYTES <= NPOC_RESULT_BYTES, "scratch overflow");

static struct scheduler_thread g_thread;
static struct wakeup g_wake;
static uint8_t *g_result;
static int g_ready;
static uint8_t *g_batch_input, *g_batch_output;
static uint32_t g_batch_input_cap, g_batch_output_cap, g_batch_next;
static struct { uint32_t kind, arg[3]; } g_job;
static volatile uint32_t g_gate_checkpoint[23], g_gate_ack;
static uint32_t g_gate_token, g_firmware_id[10];
extern uint8_t _sramstart[], _sram_code_end[], _sdramstart[], _sdram_code_end[];

/* Kept separate so native tests can deliver an emulated host ACK via IRQ. */
void __attribute__((noinline)) nand_poc_gate_wait(void)
{
    uint32_t start = read_usec_timer();
    while (g_gate_ack != g_gate_token && read_usec_timer() - start < 30000000u) ;
}

int nand_poc_gate_checkpoint(const uint32_t *mask, uint32_t iterations)
{
    volatile uint32_t *d = g_gate_checkpoint;
    unsigned int i;
    if (++g_gate_token == 0) ++g_gate_token;
    g_gate_ack = 0;
    d[0] = NPOC_GATE_CHECKPOINT; d[1] = 1; d[2] = sizeof(g_gate_checkpoint);
    d[3] = g_gate_token; d[4] = 1; /* waiting, before candidate start */
    d[5] = S.jobs_started; d[6] = S.job_kind == NPOC_JOB_BATCH_RUN ? S.progress_done : 0;
    d[7] = iterations;
    for (i = 0; i < 5; i++) { d[8+i] = mask[i]; d[13+i] = PWRCON(i); }
    d[18] = VIC0RAWINTR; d[19] = VICRAWINTR(1);
    d[20] = *(volatile uint32_t *)0x38a00c08;
    d[21] = *(volatile uint32_t *)0x38a00c38;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r" (i)); d[22] = i;
    g_hal.cache_wb((void *)d, sizeof(g_gate_checkpoint));
    nand_poc_gate_wait();
    d[4] = g_gate_ack == g_gate_token ? 2 : 4;
    g_hal.cache_wb((void *)d, sizeof(g_gate_checkpoint));
    return d[4] == 2 ? 0 : -6; /* missing transport ACK: no candidate start */
}

void nand_poc_gate_restored(void)
{
    g_gate_checkpoint[4] = 3;
    g_hal.cache_wb((void *)g_gate_checkpoint, sizeof(g_gate_checkpoint));
}

static void firmware_identity(void)
{
    uint32_t *d = g_firmware_id;
    d[0] = NPOC_FIRMWARE_ID; d[1] = 1; d[2] = sizeof(g_firmware_id);
    d[3] = (uint32_t)_sramstart; d[4] = _sram_code_end - _sramstart;
    d[5] = (uint32_t)_sdramstart; d[6] = _sdram_code_end - _sdramstart;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r" (d[7]));
    __asm__ volatile("mrc p15, 0, %0, c2, c0, 0" : "=r" (d[8]));
    d[9] = n4g_reload_boot;
    g_hal.cache_wb(d, sizeof(g_firmware_id));
}

/* NAND_STATUS(arg0=NWD1): optional live scheduler diagnostics. The ordinary
 * 0x250-byte status and old hosts are unchanged. Capture before any recovery. */
#define NPOC_WORKER_DIAG 0x3144574eu
static uint32_t g_worker_diag[29];
static uint32_t g_dispatches, g_queued_state, g_queued_current, g_queued_vic0;
extern bool scheduler_frozen;
extern volatile uint32_t dbg_unhandled_irq_last, dbg_unhandled_irq_count;

static void worker_diagnostics(void)
{
    uint32_t *d = g_worker_diag;
    d[0] = NPOC_WORKER_DIAG; d[1] = 1; d[2] = sizeof(g_worker_diag);
    d[3] = S.jobs_started; d[4] = S.state; d[5] = S.job_kind;
    d[6] = g_thread.state; d[7] = g_thread.block_type;
    d[8] = g_thread.regs[15]; d[9] = g_thread.regs[13];
    d[10] = g_thread.stack ? *g_thread.stack : 0;
    d[11] = g_wake.signalled; d[12] = (uint32_t)g_wake.waiter;
    d[13] = (uint32_t)current_thread;
    d[14] = current_thread ? current_thread->state : THREAD_FREE;
    d[15] = scheduler_frozen;
    d[16] = VIC0INTENABLE; d[17] = VIC0RAWINTR;
    d[18] = VICINTENABLE(1); d[19] = VICRAWINTR(1);
    d[20] = TBCON; d[21] = TBCMD; d[22] = TBCNT;
    d[23] = dbg_unhandled_irq_last; d[24] = dbg_unhandled_irq_count;
    d[25] = g_dispatches; d[26] = g_queued_state;
    d[27] = g_queued_current; d[28] = g_queued_vic0;
    g_hal.cache_wb(d, sizeof(g_worker_diag));
}

static void publish(void) { g_hal.cache_wb((void *)&S, sizeof(S)); }
static int need(uint32_t mask) { return (S.step_ok_mask & mask) == mask; }

static void geometry(void)
{
    const struct nand_geometry *g = nand_get_geometry();
    unsigned int i;
    if (!g) return;
    for (i = 0; i < g->banks; i++) S.chip_ids[i] = nand_chip_id();
    S.geom[0] = g->pagesperblock;
    S.geom[1] = g->banks;
    S.geom[3] = g->blocks;
    S.geom[4] = NAND_PAGE_SIZE;
    S.geom[8] = nand_get_ecc_bits();
    S.geom[5] = S.geom[8] == 16 ? 218 : 128;
    S.geom[9] = 512;
    S.geom[10] = 0x600;
    S.geom[11] = nand_chip_id();
}

static int step(uint32_t n, uint32_t arg)
{
    int rc = 0;
    switch (n) {
    case NPOC_STEP_GATES:
        S.pwrcon0_before = PWRCON(0);
        clockgate_enable(5, true);
        clockgate_enable(9, true);
        S.pwrcon0_after = PWRCON(0);
        rc = (S.pwrcon0_after & 0x220) ? -1 : 0;
        break;
    case NPOC_STEP_PINS:
        if (!need(1u << NPOC_STEP_GATES) || arg) return -1;
        /* Same BootROM pinmux as Rockbox nand_device_init(). */
        PCON6 = (PCON6 & 0xf) | 0x22222220;
        PCON7 = 0x22222222;
        PCON(8) = (PCON(8) & 0xff0000f0) | 0x00333302;
        S.gpio_after[0] = PCON6;
        S.gpio_after[1] = PCON7;
        S.gpio_after[2] = PCON(8);
        break;
    case NPOC_STEP_POWER: {
        /* bl_DxeD1759 mem:0xdac, optional NAND rail request. */
        uint8_t v = i2c_recvbyte(0, 0xe6, 0);
        S.pmu[0] = v;
        if (!(v & 15) || (v & 0x80)) {
            i2c_sendbyte(0, 0xe6, 0x10, i2c_recvbyte(0, 0xe6, 0x10) | 4);
            S.pmu[1] = i2c_recvbyte(0, 0xe6, 0x10);
        } else GPIOCMD = 0x5050f;
        break;
    }
    case NPOC_STEP_FIL:
        if (!need((1u << NPOC_STEP_GATES) | (1u << NPOC_STEP_PINS))) return -1;
        rc = nand_device_init();
        if (!rc) geometry();
        break;
    case NPOC_STEP_FTL:
        if (!need(1u << NPOC_STEP_FIL)) return -1;
        rc = ftl_init();
        S.ftl_pages = rc ? 0 : ftl_num_sectors();
        S.ftl_bpp = 4096;
        S.disk_mount_rc = rc ? (uint32_t)rc : 0;
        if (!rc && !S.ftl_pages) rc = -1;
        break;
    default: return -1;
    }
    S.step_status[n] = rc;
    if (!rc) {
        S.step_ok_mask |= 1u << n;
        if (n > S.step_reached) S.step_reached = n;
    } else S.step_ok_mask &= ~(1u << n);
    return rc;
}

static uint32_t probe_job(uint32_t bytes, uint32_t crc, uint32_t *result)
{
    const struct fmss_probe_req *q = (const void *)g_result;
    int rc;
    if (!need(1u << NPOC_STEP_GATES)) { *result = NPOC_R_PRECOND; return 0; }
    if (bytes < N4G_PROBE_PROG || bytes > N4G_PROBE_OUT ||
        q->prog_bytes > bytes - N4G_PROBE_PROG || crc32(g_result, bytes) != crc)
    { *result = NPOC_R_BAD_ARG; return 0; }
    g_hal.cache_wb_inv(g_result + N4G_PROBE_SCRATCH, N4G_PROBE_SCRATCH_BYTES);
    rc = fmss_probe(&g_hal, q, (const void *)(g_result + N4G_PROBE_PROG),
                    (void *)(g_result + N4G_PROBE_OUT));
    n4g_emcore_cache_inv(g_result + N4G_PROBE_SCRATCH, N4G_PROBE_SCRATCH_BYTES);
    S.step_ok_mask &= ~(1u << NPOC_STEP_FIL);
    S.result_len = NPOC_RESULT_BYTES;
    if (rc) *result = rc == -1 ? NPOC_R_BAD_ARG : NPOC_R_LIB_ERROR;
    return rc;
}

static void batch_free(void)
{
    if (g_batch_input) free(g_batch_input);
    if (g_batch_output) free(g_batch_output);
    g_batch_input = g_batch_output = NULL;
    g_batch_input_cap = g_batch_output_cap = g_batch_next = 0;
}

static int batch_alloc(uint32_t in, uint32_t out)
{
    if (in < 32 || in > FMSS_BATCH_INPUT_MAX || out < 128 * 1024 || out > FMSS_BATCH_OUTPUT_MAX || in + out > 26u * 1024 * 1024) return -1;
    batch_free();
    g_batch_input = memalign(64, in);
    g_batch_output = memalign(64, out);
    if (!g_batch_input || !g_batch_output) { batch_free(); return -1; }
    reownalloc(g_batch_input, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    reownalloc(g_batch_output, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    g_batch_input_cap = in; g_batch_output_cap = out;
    S.job_detail[0] = (uint32_t)g_batch_input; S.job_detail[1] = in;
    S.job_detail[2] = (uint32_t)g_batch_output; S.job_detail[3] = out;
    return 0;
}

static int batch_prepare(uint32_t id, uint32_t flags)
{
    const struct fmss_batch_output *out = (const void *)g_batch_output;
    S.job_detail[0] = id;
    S.job_detail[1] = out->bytes;
    S.job_detail[2] = (uint32_t)g_batch_output;
    S.job_detail[3] = out->errors;
    S.progress_done = id;
    publish();
    cprintf(CONSOLE_BOOT, "PLAN: M2 batch case %u\n", id);
    if (!need(1u << NPOC_STEP_GATES) && step(NPOC_STEP_GATES, 0)) return -3;
    if ((flags & FMSS_BATCH_PINS) && !need(1u << NPOC_STEP_PINS) && step(NPOC_STEP_PINS, 0)) return -3;
    return 0;
}

static int batch_run(uint32_t bytes, uint32_t crc, uint32_t first)
{
    struct fmss_batch_run b;
    struct fmss_batch_output *o = (void *)g_batch_output;
    int rc;
    if (!g_batch_input || !g_batch_output || bytes != g_batch_input_cap ||
        first != g_batch_next || crc32(g_batch_input, bytes) != crc) return -1;
    b.input = g_batch_input; b.input_bytes = bytes;
    b.output = g_batch_output; b.output_bytes = g_batch_output_cap;
    b.work = g_result; b.first = first; b.prepare = batch_prepare;
    S.progress_total = ((const struct fmss_batch_header *)g_batch_input)->count;
    if (((const struct fmss_batch_header *)g_batch_input)->magic == CPU_NAND_BATCH_MAGIC) {
        if (!need((1u << NPOC_STEP_GATES) | (1u << NPOC_STEP_PINS) | (1u << NPOC_STEP_FIL))) return -3;
        rc = cpu_nand_batch_run(&b);
    } else rc = fmss_batch_run(&g_hal, &b);
    if (rc) return rc;
    g_batch_next = o->next;
    if (((const struct fmss_batch_header *)g_batch_input)->magic != CPU_NAND_BATCH_MAGIC)
        S.step_ok_mask &= ~(1u << NPOC_STEP_FIL);
    S.progress_done = o->next;
    S.job_detail[0] = o->next; S.job_detail[1] = o->bytes;
    S.job_detail[2] = (uint32_t)g_batch_output; S.job_detail[3] = o->errors;
    /* Packed output is separate from the stable probe scratch/result allocation. */
    return 0;
}

/* Host wrote 4096 data bytes and 12 metadata bytes (10 used) at g_result. Backend stays FMSS for this call. */
static int fmss_nand_job(const uint32_t *arg, uint32_t *result)
{
    const struct nand_geometry *g = nand_get_geometry();
    uint32_t op = arg[0] & 0xff, ce = (arg[0] >> 8) & 0xf, row = arg[1];
    unsigned int previous;
    int rc;

    *result = NPOC_R_OK;
    if ((arg[0] >> 12) || arg[2] || (op != FMSS_NAND_ERASE && op != FMSS_NAND_PROGRAM)) {
        *result = NPOC_R_BAD_ARG;
        return 0;
    }
    if (!g || ce >= g->banks || !g->pagesperblock || row >= g->blocks * g->pagesperblock ||
        (op == FMSS_NAND_ERASE && row % g->pagesperblock)) {
        *result = NPOC_R_BAD_ARG;
        return 0;
    }
    previous = nand_get_backend();
    nand_set_backend(NAND_BACKEND_M2);
    if (op == FMSS_NAND_ERASE)
        rc = nand_erase_block(ce, row / g->pagesperblock);
    else {
        g_hal.cache_wb_inv(g_result, 0x1010);
        rc = nand_write_page(ce, row, g_result, (const uint32_t *)(g_result + 0x1000));
    }
    nand_set_backend(previous);
    return rc;
}

static void run_job(uint32_t kind, const uint32_t *arg)
{
    uint32_t result = NPOC_R_OK;
    int rc = 0;
    S.state = NPOC_STATE_RUNNING;
    S.job_t_start = read_usec_timer();
    S.result_len = 0;
    publish();
    cprintf(CONSOLE_BOOT, "PLAN: NAND/M2 job %u args %08x %08x\n", kind, arg[0], arg[1]);
    if (kind == NPOC_JOB_FMSS_PROBE) rc = probe_job(arg[0], arg[1], &result);
    else if (kind == NPOC_JOB_BATCH_ALLOC) rc = batch_alloc(arg[0], arg[1]);
    else if (kind == NPOC_JOB_BATCH_RUN) rc = batch_run(arg[0], arg[1], arg[2]);
    else if (kind == NPOC_JOB_BATCH_FREE) batch_free();
    else if (kind == NPOC_JOB_STEP) rc = step(arg[0], arg[1]);
    else if (kind == NPOC_JOB_NAND_COMPARE) {
        uint32_t len = 0;
        if (!need((1u << NPOC_STEP_GATES) | (1u << NPOC_STEP_PINS) | (1u << NPOC_STEP_FIL)))
            result = NPOC_R_PRECOND;
        else {
            rc = nand_compare(g_result, NPOC_RESULT_BYTES, arg, &len);
            if (rc == -31) result = NPOC_R_BAD_ARG;
        }
        S.result_len = len;
    }
    else if (kind == NPOC_JOB_CPU_NAND && (arg[0] & 0xff) >= CNS_OP_INFO) {
        /* CPU-only NAND driver sub-ops (cpu_nand_stack.h); ops 0..5 below keep their ABI. */
        uint32_t len, det[2] = { 0, 0 };
        rc = cpu_nand_stack_job(g_result, arg, need((1u << NPOC_STEP_GATES) | (1u << NPOC_STEP_PINS) |
                                                    (1u << NPOC_STEP_FIL)), &result, det, &len);
        S.job_detail[0] = det[0]; S.job_detail[1] = det[1];
        S.result_len = len;
    }
    else if (kind == NPOC_JOB_CPU_NAND) {
        uint32_t len, det[2] = { 0, 0 };
        if (!need((1u << NPOC_STEP_GATES) | (1u << NPOC_STEP_PINS)) ||
            ((arg[0] & 0xff) && (arg[0] & 0xff) != 5 && !need(1u << NPOC_STEP_FIL))) result = NPOC_R_PRECOND;
        else {
            rc = cpu_nand_job(g_result, arg, &result, det, &len);
            S.job_detail[0] = det[0]; S.job_detail[1] = det[1];
            S.result_len = len;
        }
    }
    else if (kind == NPOC_JOB_FMSS_NAND) {
        if (!need((1u << NPOC_STEP_GATES) | (1u << NPOC_STEP_PINS) | (1u << NPOC_STEP_FIL)))
            result = NPOC_R_PRECOND;
        else
            rc = fmss_nand_job(arg, &result);
    }
    else if (kind == NPOC_JOB_FTL_READ) {
        uint32_t sector = arg[0], count = arg[1];
        if (!need(1u << NPOC_STEP_FTL)) result = NPOC_R_PRECOND;
        else if (!count || count > NPOC_RESULT_BYTES / 4096u || sector >= S.ftl_pages
                 || count > S.ftl_pages - sector)
            result = NPOC_R_BAD_ARG;
        else {
            rc = ftl_read(sector, count, g_result);
            if (rc) result = NPOC_R_LIB_ERROR;
            else S.result_len = count * 4096u;
        }
    }
    else if (!need(1u << NPOC_STEP_FIL)) result = NPOC_R_PRECOND;
    else if (kind == NPOC_JOB_READ_RESERVED) {
        const struct nand_geometry *g = nand_get_geometry();
        if (!g || arg[0] >= g->blocks * g->pagesperblock) result = NPOC_R_BAD_ARG;
        else {
            rc = nand_read_boot_page(0, arg[0], g_result);
            S.result_len = 0x600;
        }
    } else if (kind == NPOC_JOB_READ_PAGE) {
        const struct nand_geometry *g = nand_get_geometry();
        if (!g || arg[0] >= g->banks || arg[1] >= g->blocks * g->pagesperblock || arg[2])
            result = NPOC_R_BAD_ARG;
        else {
            memset(g_result, 0, NPOC_RP_BYTES);
            rc = nand_read_page(arg[0], arg[1], g_result, (void *)(g_result + NPOC_RP_META));
            nand_copy_sector_status(g_result + NPOC_RP_SECSTAT);
            S.fil_pages++;
            if (rc == NAND_ECC_FAILED) S.fil_uecc++;
            S.result_len = NPOC_RP_BYTES;
            geometry();
        }
    } else result = NPOC_R_BAD_ARG;
    if (result == NPOC_R_OK && rc && !(kind != NPOC_JOB_STEP && rc == NAND_ECC_CORRECTED))
        result = NPOC_R_LIB_ERROR;
    S.job_lib_rc = rc;
    S.job_result = result;
    S.job_t_end = read_usec_timer();
    if (kind != NPOC_JOB_BATCH_RUN) { S.progress_done = 1; S.progress_total = 1; }
    if (result) S.err_job_seq = S.jobs_started;
    if (S.result_len) g_hal.cache_wb(g_result, S.result_len);
    S.jobs_done = S.jobs_started;
    S.state = NPOC_STATE_DONE;
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
        g_job.kind = NPOC_JOB_NONE;
        leave_critical_section(mode);
        if (kind != NPOC_JOB_NONE) run_job(kind, arg);
    }
}

static uint32_t queue(uint32_t kind, uint32_t a0, uint32_t a1, uint32_t a2)
{
    uint32_t mode, i;
    if (!g_ready) return NPOC_ACCEPT_NOT_INIT;
    mode = enter_critical_section();
    if (S.busy) { leave_critical_section(mode); return NPOC_ACCEPT_BUSY; }
    S.busy = 1;
    S.state = NPOC_STATE_QUEUED;
    S.jobs_started++;
    S.job_kind = kind;
    S.job_arg[0] = a0; S.job_arg[1] = a1; S.job_arg[2] = a2;
    S.job_result = 0; S.job_lib_rc = 0;
    for (i = 0; i < 4; i++) S.job_detail[i] = 0;
    g_job.kind = kind; g_job.arg[0] = a0; g_job.arg[1] = a1; g_job.arg[2] = a2;
    wakeup_signal(&g_wake);
    g_queued_state = g_thread.state;
    g_queued_current = (uint32_t)current_thread;
    g_queued_vic0 = VIC0INTENABLE;
    /* Called only by the USB IRQ handler. wakeup_signal normally preempts
     * only idle; a ready worker otherwise depends on the scheduler timer.
     * Select it for IRQ return even if a clock experiment disturbed that
     * timer. Never run NAND in IRQ context or replay a running job. Only a
     * READY worker is selected; the scheduler's freeze state still applies. */
    if (g_thread.state == THREAD_READY && !scheduler_frozen) {
        g_dispatches++;
        scheduler_switch(&g_thread, NULL);
    }
    leave_critical_section(mode);
    return NPOC_ACCEPT_OK;
}


int nand_poc_init(void)
{
    void *stack;
    unsigned int i;
    if (g_ready) return 0;
    memset((void *)&S, 0, sizeof(S));
    S.magic = NPOC_MAGIC;
    S.version = NPOC_VERSION;
    S.size = NPOC_STATUS_BYTES;
    S.disk_mount_rc = 0xffffffffu;
    S.pmu[0] = S.pmu[1] = 0xffffffffu;
    for (i = 0; i < NPOC_NSTEPS; i++) S.step_status[i] = NPOC_STEP_NOT_RUN;
    g_result = memalign(64, NPOC_RESULT_BYTES);
    stack = memalign(32, 0x8000);
    if (!g_result || !stack) { S.alloc_fail++; return -1; }
    reownalloc(g_result, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    reownalloc(stack, KERNEL_OWNER(KERNEL_OWNER_UNKNOWN));
    memset(g_result, 0, NPOC_RESULT_BYTES);
    g_hal.cache_wb(g_result, NPOC_RESULT_BYTES);
    S.alloc_count = 2;
    S.alloc_bytes = NPOC_RESULT_BYTES + 0x8000;
    S.result_addr = (uint32_t)g_result;
    S.result_cap = NPOC_RESULT_BYTES;
    wakeup_init(&g_wake);
    if (!thread_create(&g_thread, "nand worker", worker, stack, 0x8000,
                       CORE_THREAD, 64, true, NULL, NULL, NULL, NULL)) return -1;
    g_ready = 1;
    publish();
    return 0;
}

void nand_poc_heartbeat(uint32_t *w0, uint32_t *w1)
{
    *w0 = (S.busy ? 0x80000000u : 0) | (S.state & 0x7f) << 24 |
          (S.job_kind & 0xff) << 16 | (S.step_reached & 0xff) << 8 | (S.jobs_done & 0xff);
    *w1 = S.fil_pages;
}

void nand_poc_usb_command(const uint32_t *buf, uint32_t *reply, const void **rep, uint32_t *replen)
{
    uint32_t acc;
    *rep = NULL;
    *replen = 0;
    reply[0] = 1;
    switch (buf[0]) {
    case NPOC_CMD_FMSS_INFO:
        reply[1] = 0x4d325052;
        reply[2] = 14;
        reply[3] = FMSS_PROBE_FULLMAP | FMSS_PROBE_DISCOVERY | FMSS_BATCH_CAP | (1u << 22) | (1u << 23) | (1u << 24) | (1u << 25) | (1u << 26) | (1u << 27) | FMSS_PROBE_VICSOFT; /* bulk, recovery, phases, progress, state-b reset, reset while held, VIC soft lines */
        reply[3] |= FMSS_PROBE_CPU_NAND | FMSS_PROBE_STALL_SAFE;                  /* CPU-direct NAND jobs (cpu_nand.c) */
        reply[3] |= FMSS_PROBE_CAP_SAMPLER | FMSS_PROBE_CAP_STIMULI;          /* round 6: CPU sampler; gate window, EIC, far pads */
        reply[3] |= FMSS_PROBE_MUTATE;                    /* writes allowed after DFU and after reload */
        if (n4g_reload_boot) reply[3] |= FMSS_PROBE_RELOADED;
        return;
    case 120:
        if (buf[1] == NPOC_GATE_CHECKPOINT) {
            if (g_gate_checkpoint[4] == 1 && buf[2] == g_gate_token)
                g_gate_ack = buf[2];
            reply[1] = (uint32_t)g_gate_checkpoint;
            reply[2] = sizeof(g_gate_checkpoint);
            reply[3] = NPOC_GATE_CHECKPOINT;
            return;
        }
        if (buf[1] == NPOC_FIRMWARE_ID) {
            firmware_identity();
            reply[1] = (uint32_t)g_firmware_id;
            reply[2] = sizeof(g_firmware_id);
            reply[3] = NPOC_FIRMWARE_ID;
            return;
        }
        if (buf[1] == NPOC_WORKER_DIAG) {
            worker_diagnostics();
            reply[1] = (uint32_t)g_worker_diag;
            reply[2] = sizeof(g_worker_diag);
            reply[3] = NPOC_WORKER_DIAG;
            return;
        }
        S.now_us = read_usec_timer();
        reply[1] = (uint32_t)&S;
        reply[2] = NPOC_STATUS_BYTES;
        reply[3] = S.result_addr;
        return;
    case 121:
        if (buf[1] < 1 || buf[1] > 5) acc = NPOC_ACCEPT_BAD_ARG;
        else acc = queue(NPOC_JOB_STEP, buf[1], buf[2], 0);
        break;
    case 122: acc = queue(NPOC_JOB_READ_RESERVED, buf[1], buf[2], 0); break;
    case 123: acc = queue(NPOC_JOB_READ_PAGE, buf[1], buf[2], buf[3]); break;
    case NPOC_CMD_FTL_OPEN: acc = queue(NPOC_JOB_STEP, NPOC_STEP_FTL, 0, 0); break;
    case NPOC_CMD_FTL_READ: acc = queue(NPOC_JOB_FTL_READ, buf[1], buf[2], 0); break;
    case NPOC_CMD_BATCH_ALLOC: acc = queue(NPOC_JOB_BATCH_ALLOC, buf[1], buf[2], 0); break;
    case NPOC_CMD_BATCH_RUN: acc = queue(NPOC_JOB_BATCH_RUN, buf[1], buf[2], buf[3]); break;
    case NPOC_CMD_BATCH_FREE: acc = queue(NPOC_JOB_BATCH_FREE, 0, 0, 0); break;
    case NPOC_CMD_FMSS_PROBE: acc = queue(NPOC_JOB_FMSS_PROBE, buf[1], buf[2], 0); break;
    case NPOC_CMD_NAND_COMPARE: acc = queue(NPOC_JOB_NAND_COMPARE, buf[1], buf[2], buf[3]); break;
    case NPOC_CMD_CPU_NAND: acc = queue(NPOC_JOB_CPU_NAND, buf[1], buf[2], buf[3]); break;
    case NPOC_CMD_FMSS_NAND: acc = queue(NPOC_JOB_FMSS_NAND, buf[1], buf[2], buf[3]); break;
    case NPOC_CMD_WRITE_INFO:
        reply[1] = reply[2] = reply[3] = 0; /* no legacy FTL/write staging API */
        return;
    case 126:
        acc = NPOC_ACCEPT_BAD_ARG; break;
    case 129: case 130: case 131: case 132: case 133: case 134: case 135: case 136:
        acc = NPOC_ACCEPT_NO_WRITE; break;
    case NPOC_CMD_WATCHDOG:
        S.wdt_request = buf[1];
        reply[1] = buf[1] ? NPOC_ACCEPT_UNKNOWN_HW : NPOC_ACCEPT_OK;
        reply[2] = buf[1]; reply[3] = 0;
        return;
    default: reply[0] = 2; return;
    }
    reply[1] = acc;
    reply[2] = S.jobs_started;
    reply[3] = (uint32_t)&S;
}
