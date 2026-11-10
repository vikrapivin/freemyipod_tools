/* Physical NAND/M2 monitor inside emCORE (iPod nano 4G).
 * Status layout is fixed. Legacy FTL/write commands are unsupported.
 *
 * A single NAND worker thread executes one job at a time; the USB sync commands 120..126 (usbdebug.c) only queue
 * a job and return. The host polls the status record (struct nand_poc_status, exported as `nand_poc_status`,
 * address/size returned by NAND_STATUS) with READ_MEM and reads job output from the result buffer
 * (status.result_addr, 64-byte aligned, status.result_cap bytes). Nothing touches the NAND at boot.
 *
 * Layout of the status record is fixed (version NPOC_VERSION); emCORE/tools/n4g_nand.py mirrors it.
 * NAND_STATUS with arg0=0x3144574e (NWD1) instead returns a read-only worker
 * diagnostic record: pointer, byte size, NWD1. Ordinary status is unchanged;
 * older kernels ignore the argument and return ordinary status. The host
 * saves this optional record when an accepted job times out in QUEUED state.
 */
#ifndef NAND_POC_H
#define NAND_POC_H
#include "global.h"

#define NPOC_MAGIC    0x434f504eu   /* "NPOC" */
#define NPOC_VERSION  1u

/* steps (NAND_STEP n) */
#define NPOC_STEP_GATES 1u /* clockgate_enable(5), (9): PWRCON0 0x3c500048 &= ~0x20, &= ~0x200 */
#define NPOC_STEP_PINS 2u /* NAND pad mux: arg 0 = ROM values; other args refused */
#define NPOC_STEP_POWER 3u /* optional PMU "NAND power 0xb" request, off unless run */
#define NPOC_STEP_FIL     4u   /* nand_device_init: reset, READ ID, table match -> IDs/geometry */
#define NPOC_STEP_FTL     5u   /* WMR_Dispatch(0x70): VSVFL + FTL open (long), then disk_mount(0) */
#define NPOC_NSTEPS       8u   /* step_status[] slots, index = step id */

/* job kinds */
#define NPOC_CMD_FMSS_PROBE 137u
#define NPOC_CMD_FMSS_INFO 138u
#define NPOC_JOB_FMSS_PROBE 16u
#define NPOC_CMD_BATCH_ALLOC 139u
#define NPOC_CMD_BATCH_RUN 140u
#define NPOC_CMD_BATCH_FREE 141u
#define NPOC_JOB_BATCH_ALLOC 17u
#define NPOC_JOB_BATCH_RUN 18u
#define NPOC_JOB_BATCH_FREE 19u
#define NPOC_CMD_CPU_NAND 142u     /* CPU-direct NAND operation (cpu_nand.h) */
#define NPOC_JOB_CPU_NAND 20u
#define NPOC_CMD_NAND_COMPARE 143u
#define NPOC_JOB_NAND_COMPARE 21u
#define NPOC_CMD_NAND_BACKEND 145u /* 0 FMSS, 1 CPU, 2 query. Changing it closes the FTL. */
#define NPOC_CMD_FMSS_NAND 144u    /* program one page or erase one block with program.s / erase.s */
#define NPOC_JOB_FMSS_NAND 22u
#define NPOC_CMD_FTL_OPEN 124u     /* step 5: mount the VFL/FTL, no writes */
#define NPOC_CMD_FTL_READ 125u     /* (sector, count) 4096-byte logical sectors */
#define NPOC_JOB_FTL_READ 23u
#define FMSS_NAND_ERASE 1u
#define FMSS_NAND_PROGRAM 2u
#define N4G_PROBE_PROG 0x400u
#define N4G_PROBE_OUT 0x20000u
#define N4G_PROBE_SCRATCH 0x38000u
#define N4G_PROBE_SCRATCH_BYTES 0x8000u

#define NPOC_JOB_NONE          0u
#define NPOC_JOB_STEP          1u   /* arg0 = step, arg1 = step argument */
#define NPOC_JOB_READ_RESERVED 2u   /* arg0 = page (bank 0), arg1 reserved (ignored) */
#define NPOC_JOB_READ_PAGE     3u   /* arg0 = bank (CE index), arg1 = page (row), arg2 must be zero */
/* Legacy commands 124..126 and 129..136 are reserved, but unsupported.
 * WRITE_INFO returns zero; FMSS_INFO carries the reload mutation capability. */
#define NPOC_CMD_WRITE_INFO 128u

/* job states */
#define NPOC_STATE_IDLE     0u      /* no job since boot */
#define NPOC_STATE_QUEUED   1u      /* accepted by the USB handler, worker not yet started */
#define NPOC_STATE_RUNNING  2u      /* worker executing (record written before the first register access) */
#define NPOC_STATE_DONE     3u      /* finished; job_result / job_lib_rc valid */

/* job_result (harness-level outcome; job_lib_rc carries the library's own return code) */
#define NPOC_R_OK           0u
#define NPOC_R_LIB_ERROR    1u      /* library returned an error code (see job_lib_rc) */
#define NPOC_R_PRECOND      2u      /* a prerequisite step has not succeeded (nothing touched) */
#define NPOC_R_BAD_ARG      3u
#define NPOC_R_NO_MEMORY    4u
#define NPOC_R_PANIC        5u      /* the library reported a panic/assert during the job (hal->panic) */
#define NPOC_R_GATE         6u      /* destructive job refused: see job_detail[0] (NPOC_GATE_*) */
#define NPOC_GATE_NO_SNAPSHOT 1u
#define NPOC_GATE_NO_STEP     2u    /* prerequisite step not done */
#define NPOC_GATE_NO_RW       3u    /* write stack not open */
#define NPOC_GATE_BLOCK0      4u    /* bank 0 block 0 (ROM reserved region) without force */
#define NPOC_GATE_IN_USE      5u    /* block in use / unknown, job_detail[1] = reason, without force */
#define NPOC_GATE_RANGE       6u

/* accept codes returned in reply word 1 by the queuing commands */
#define NPOC_ACCEPT_OK       0u
#define NPOC_ACCEPT_BUSY     1u     /* a job is queued or running: one command in flight */
#define NPOC_ACCEPT_BAD_ARG  2u
#define NPOC_ACCEPT_NOT_INIT 3u     /* worker thread not created (storage init has not run yet) */
#define NPOC_ACCEPT_UNKNOWN_HW 4u /* NAND_WATCHDOG: watchdog register semantics not established */
#define NPOC_ACCEPT_BAD_TOKEN  5u   /* destructive command without the right confirmation token */
#define NPOC_ACCEPT_NO_WRITE   6u   /* read-only build: write commands compiled out */

#define NPOC_STEP_NOT_RUN 0xffffffffu
#define NPOC_RESULT_BYTES 0x40000u

/* READ_PAGE result layout inside the result buffer */
#define NPOC_RP_DATA     0x0000u   /* 4096 data bytes */
#define NPOC_RP_META     0x1000u   /* 12 meta bytes */
#define NPOC_RP_MAXCORR  0x1010u   /* 1 byte: max corrected bits + 1 (0 = none reported) */
#define NPOC_RP_SECSTAT  0x1020u   /* 8 per-sector status bytes, then 8 reserved zero bytes */
#define NPOC_RP_BYTES    0x1040u

#define NPOC_CMD_WATCHDOG 127u     /* NAND_WATCHDOG(seconds), 0 = disarm */

struct nand_poc_status {
    uint32_t magic, version, size;                 /* 0x000 */
    uint32_t busy;                                 /* 0x00c 1 = a job is queued or running */
    uint32_t state;                                /* 0x010 NPOC_STATE_* of the current/last job */
    uint32_t step_reached;                         /* 0x014 highest step id whose last run succeeded */
    uint32_t step_ok_mask;                         /* 0x018 bit n = step n succeeded */
    uint32_t jobs_started, jobs_done;              /* 0x01c, 0x020 sequence numbers (job id = jobs_started) */
    uint32_t job_kind;                             /* 0x024 */
    uint32_t job_arg[3];                           /* 0x028 */
    uint32_t job_result;                           /* 0x034 NPOC_R_* */
    uint32_t job_lib_rc;                           /* 0x038 raw library return value */
    uint32_t job_detail[4]; /* 0x03c job-specific (see ) */
    uint32_t job_t_start, job_t_end;               /* 0x04c µs timer */
    uint32_t progress_done, progress_total;        /* 0x054 */
    uint32_t step_status[NPOC_NSTEPS];             /* 0x05c per-step status (raw code) */
    uint32_t chip_ids[8];                          /* 0x07c READ ID words per physical CE (m2 idbuf) */
    uint32_t geom[12];                             /* 0x09c FIL_GetDeviceInfo 0x64,0x65,0x66,0x67,0x68,0x69,0x6b,
                                                      0x6c,0x73,0x74,0x72 + di.chipid */
    uint32_t ftl_pages, ftl_bpp;                   /* 0x0cc FTL dwUserPagesTotal / bytes per page from cmd 0x70 */
    uint32_t disk_mount_rc;                        /* 0x0d4 disk_mount(0) after FTL open (0xffffffff = not run) */
    uint32_t result_addr, result_cap, result_len;  /* 0x0d8 */
    uint32_t ring_addr, ring_bytes;                /* 0x0e4 platform ring buffer (n4g_rec records) */
    uint32_t clk_mhz;                              /* 0x0ec n4g_clk_mhz() at step 4 */
    uint32_t pwrcon0_before, pwrcon0_after;        /* 0x0f0 step 1 */
    uint32_t gpio_after[3];                        /* 0x0f8 0x3cf000c0/e0/100 after step 2 */
    uint32_t pmu[2];                               /* 0x104 step 3: PMU reg 0, reg 0x10 after (0xffffffff = n/a) */
    uint32_t fil_pages, fil_uecc, fil_timeouts;    /* 0x10c live FIL statistics (low 32 bits) */
    uint32_t alloc_bytes, alloc_count, alloc_fail; /* 0x118 hal->alloc */
    uint32_t panic_count, panic_line, panic_t_us;  /* 0x124 last hal->panic */
    char     panic_file[32];                       /* 0x130 */
    uint32_t wmr_asserts;                          /* 0x150 wmr_assert_failures() */
    uint32_t err_job_seq;                          /* 0x154 job id of the last failed job */
    uint32_t snap_valid;                           /* 0x158 1 = register snapshot below belongs to err_job_seq */
    uint32_t fmc_snap[24];                         /* 0x15c FMC 0x38a00000..0x38a0005c */
    uint32_t ecc_snap[6];                          /* 0x1bc FMC 0x38a00800..0x38a00814 */
    uint32_t fmss_snap[28];                        /* 0x1d4 FMSS 0x38a00c00..0x38a00c6c */
    uint32_t now_us;                               /* 0x244 refreshed by NAND_STATUS */
    uint32_t hb_word;                              /* 0x248 nand_poc_heartbeat word 0 */
    uint32_t wdt_request;                          /* 0x24c last NAND_WATCHDOG seconds (never armed, see §10) */
};
#define NPOC_STATUS_BYTES 0x250u

extern volatile struct nand_poc_status nand_poc_status;

/* Automatic USB evidence delivery for explicitly checkpointed gate probes.
 * The host records the checkpoint before acknowledging its token. */
#define NPOC_GATE_CHECKPOINT 0x31504347u /* GCP1 */
#define NPOC_FIRMWARE_ID 0x3149464eu     /* NFI1: immutable running code ranges */
int nand_poc_gate_checkpoint(const uint32_t *mask, uint32_t iterations);
void nand_poc_gate_restored(void);

/* boot: create the worker thread and allocate the result buffer. No hardware access. */
int  nand_poc_init(void);
/* usbdebug.c sync handler (IRQ context): queue a job / report the status record. buf = request words,
 * reply = dbgbuf (reply[0] = 1, reply[1..3] command-specific), rep/replen = optional reply payload (pointer, length). */
void nand_poc_usb_command(const uint32_t *buf, uint32_t *reply, const void **rep, uint32_t *replen);
/* heartbeat (async cmd 200): w0 = bit31 busy | state<<24 | job_kind<<16 | step_reached<<8 | jobs_done & 0xff,
 * w1 = progress counter (FIL pages read since boot, live). */
void nand_poc_heartbeat(uint32_t *w0, uint32_t *w1);

#endif
