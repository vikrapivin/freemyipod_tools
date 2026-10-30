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

/* NAND harness inside emCORE for the iPod nano 3G (S5L8702), on top of the
 * Rockbox nano 3G NAND driver and FTL (nand/, see nand3g.c for job semantics).
 * ONE worker thread executes one job at a time; the USB sync commands
 * 120..136 (usb/usbdebug.c, IRQ context) only queue a job and return. The
 * host polls the status record (struct n3g_nand_status, address and size come
 * back from N3G_NAND_STATUS) with READ_MEM and reads job output from the
 * result buffer (status.result_addr, status.result_cap bytes). NOTHING touches
 * the NAND at boot. FTL writes (129..131) are built in unless N3G_NAND_WRITE=0.
 * tools/nano3g/protocol.py and nand.py mirror this file (N3G_NAND_VERSION).
 *
 * Physical backends (nand/nand-backend.h) are selectable: command 135 picks
 * the active one, the benchmark can pick one per run.
 */
#ifndef N3G_NAND_H
#define N3G_NAND_H
#include "libc/include/stdint.h"

#ifndef N3G_NAND_WRITE
#define N3G_NAND_WRITE 1
#endif

#define N3G_NAND_MAGIC    0x4e47334eu   /* "N3GN" little endian */
#define N3G_NAND_VERSION  1u

/* steps (N3G_NAND_STEP n, arg) */
#define N3G_STEP_GATES    1u   /* FMC + ECC clock gates open (PWRCON0 bits 8 and 12) */
#define N3G_STEP_PINS     2u   /* snapshot of the NAND pad words PCON(8..10); read only */
#define N3G_STEP_PHYS     3u   /* active backend init: reset + READ ID per bank, chip table match, geometry */
#define N3G_STEP_FTL      4u   /* ftl_init(); arg bit 0: allow the mount to write (restore) and ftl_write;
                                  bit 1: also on a chip row not validated on hardware */
#define N3G_NSTEPS        8u

#define N3G_JOB_NONE       0u
#define N3G_JOB_STEP       1u   /* arg0 = step, arg1 = step argument */
#define N3G_JOB_READ_PAGE  3u   /* arg0 = bank, arg1 = page (row within the bank) */
#define N3G_JOB_FTL_READ   4u   /* arg0 = sector, arg1 = count (2048-byte FTL sectors) */
#define N3G_JOB_WRITE_OPEN 7u
#define N3G_JOB_BLOCK_WRITE 8u
#define N3G_JOB_FLUSH      9u
#define N3G_JOB_BACKEND    10u  /* arg0 = backend index */
#define N3G_JOB_BENCH      11u  /* parameters in the result buffer (struct n3g_bench_params) */
#define N3G_JOB_META_SCAN  12u  /* arg0 = first block, arg1 = blocks: spare records of every page (read only) */

/* USB monitor commands (usbdebug.c); numbering follows the nano 5G target.
 * 122, 126, 127 and 132..134 (5G ROM page, reserved, block read, modules)
 * are not implemented here. */
#define N3G_NAND_STATUS     120   /* -> {1, &n3g_nand_status, size, result buffer} */
#define N3G_NAND_STEP       121   /* (step, arg) */
#define N3G_NAND_READ_PAGE  123   /* (bank, page) */
#define N3G_NAND_FTL_OPEN   124   /* (writable) == step 4 */
#define N3G_NAND_FTL_READ   125   /* (sector, count) */
#define N3G_NAND_WRITE_CAPS 128   /* -> {protocol=1, flags(bit0=writer), max sectors per write} */
#define N3G_NAND_WRITE_OPEN 129   /* requires an FTL opened writable */
#define N3G_NAND_BLOCK_WRITE 130  /* {sector, count, CRC32(staged bytes)} */
#define N3G_NAND_FLUSH      131   /* ftl_sync() */
#define N3G_NAND_BACKEND    135   /* (index): select the physical backend; invalidates init/FTL */
#define N3G_NAND_BENCH      136   /* run the benchmark described in the result buffer */
#define N3G_NAND_META_SCAN  137   /* (first block, blocks): read every page of those blocks on every bank */

#define N3G_NAND_HEARTBEAT  200   /* {1, 0, w0, w1} */

#define N3G_STATE_IDLE     0u
#define N3G_STATE_QUEUED   1u
#define N3G_STATE_RUNNING  2u
#define N3G_STATE_DONE     3u

#define N3G_R_OK          0u
#define N3G_R_LIB_ERROR   1u
#define N3G_R_PRECOND     2u    /* prerequisite step missing: nothing touched */
#define N3G_R_BAD_ARG     3u
#define N3G_R_NO_MEMORY   4u
#define N3G_R_NOT_ALLOWED 5u    /* destructive operation without its explicit flag, or unvalidated chip */

#define N3G_ACCEPT_OK       0u
#define N3G_ACCEPT_BUSY     1u
#define N3G_ACCEPT_BAD_ARG  2u
#define N3G_ACCEPT_NOT_INIT 3u

#define N3G_STEP_NOT_RUN  0xffffffffu
#define N3G_STEP_RUNNING  0xfffffffeu

#define N3G_RESULT_BYTES  (256u * 1024u)
#define N3G_SECTOR_BYTES  2048u
#define N3G_FTL_CHUNK     16u   /* FTL sectors per ftl_read() call of the read job */

/* READ_PAGE result layout inside the result buffer */
#define N3G_RP_DATA     0x0000u   /* page data: pagesize (2048 / 4096) */
#define N3G_RP_META     0x1000u   /* 12 spare metadata bytes (3 words), 4 zero pad bytes */
#define N3G_RP_ECC      0x1010u   /* u32 NAND_ECC_* verdict (0 clean, 1 corrected, 3 failed) */
#define N3G_RP_RESULT   0x1014u   /* u32 read program bitmap (bit30 uncorrectable, bit29 erased, bit n-1 n-bit fix) */
#define N3G_RP_BYTES    0x1020u

struct n3g_nand_status {
    uint32_t magic, version, size;                 /* 0x000 */
    uint32_t busy;                                 /* 0x00c 1 = a job is queued or running */
    uint32_t state;                                /* 0x010 N3G_STATE_* */
    uint32_t step_ok_mask;                         /* 0x014 bit n = step n succeeded */
    uint32_t jobs_started, jobs_done;              /* 0x018, 0x01c */
    uint32_t job_kind;                             /* 0x020 */
    uint32_t job_arg[3];                           /* 0x024 */
    uint32_t job_result;                           /* 0x030 N3G_R_* */
    uint32_t job_lib_rc;                           /* 0x034 raw library return value */
    uint32_t job_detail[4];                        /* 0x038 job specific */
    uint32_t job_t_start, job_t_end;               /* 0x048 us counter */
    uint32_t progress_done, progress_total;        /* 0x050 */
    uint32_t step_status[N3G_NSTEPS];              /* 0x058 per-step raw code */
    uint32_t backend;                              /* 0x078 active backend index (0 sw, 1 fmss) */
    uint32_t chip_ids[4];                          /* 0x07c READ ID word 0 per bank (ad d3 14 a5 -> 0xa514d3ad) */
    uint32_t chip_ext[4];                          /* 0x08c READ ID word 1 per bank */
    uint32_t geom[12];                             /* 0x09c banks, blocks/bank, pages/block, planes, userblocks, vflspares,
                                                      twoplane, pagesize, mode, layout, validated, chip id */
    uint32_t ftl_sectors, ftl_sector_bytes;        /* 0x0cc */
    uint32_t ftl_writable;                         /* 0x0d4 1 = mounted writable */
    uint32_t result_addr, result_cap, result_len;  /* 0x0d8 */
    uint32_t now_us;                               /* 0x0e4 refreshed by N3G_NAND_STATUS */
    uint32_t hb_word;                              /* 0x0e8 heartbeat word 0 */
    uint32_t pwrcon0_before, pwrcon0_after;        /* 0x0ec step 1 */
    uint32_t pcon[3];                              /* 0x0f4 PCON(8), PCON(9), PCON(10) after step 2/3 */
    uint32_t chip_row;                             /* 0x100 index in the chip table, 0xffffffff none */
    uint32_t fmss_fault_addr;                      /* 0x104 &fmss_fault (struct fmss_fault_info, nand-fmss.h) */
    uint32_t fmss_fault_count, fmss_fault_pc;      /* 0x108 faults/timeouts so far, PC of the last one */
};
#define N3G_STATUS_BYTES 0x110u

extern volatile struct n3g_nand_status n3g_nand_status;

/* boot (init.c, after USB): create the worker thread and the result buffer. No hardware access. */
int  n3g_nand_init(void);
/* usbdebug.c sync handler (IRQ context): queue a job / report the status record. */
void n3g_nand_usb_command(const uint32_t *buf, uint32_t *reply);
/* heartbeat (usbdebug cmd 200): w0 = bit31 busy | state<<24 | job_kind<<16 | step_ok_mask&0xff<<8 | jobs_done&0xff,
 * w1 = jobs started. */
void n3g_nand_heartbeat(uint32_t *w0, uint32_t *w1);

/* META_SCAN result layout, the records of Rockbox's nano 3G NAND check (nand-check-nano3g.c): for each block of the
 * range, for each bank, pages_per_block records of 16 bytes: the 12 spare metadata bytes, then an i32 result: -1 if
 * the read call failed, else the ECC class (0 clean, 1 corrected, 3 failed) | the most bits corrected in a chunk << 8
 * | 0x10000 if the page reads erased. blocks x banks x pages_per_block x 16 must fit N3G_RESULT_BYTES. */
#define N3G_MS_RECORD     16u

/* ------------------------------------------------------------------ benchmark
 * The host writes struct n3g_bench_params (32 words) at result_addr, then sends
 * N3G_NAND_BENCH. The worker writes struct n3g_bench_result at result_addr+0x100
 * and the per-repeat microseconds (u32 x N3G_BENCH_MAXREP) at result_addr+0x400.
 * All counters are u32 and wrap. Default is read-only: the write op needs
 * N3G_BF_WRITE. */
#define N3G_BENCH_PARAM_MAGIC  0x4247334eu   /* "N3GB" */
#define N3G_BENCH_RESULT_MAGIC 0x5247334eu   /* "N3GR" */
#define N3G_BENCH_VERSION      1u
#define N3G_BENCH_RES_OFF      0x100u
#define N3G_BENCH_CPU_OFF      0x200u        /* struct n3g_bench_cpu */
#define N3G_BENCH_REP_OFF      0x400u
#define N3G_BENCH_MAXREP       1024u
#define N3G_BENCH_MAXBATCH     32u           /* pages per backend call */
#define N3G_BENCH_MAXFTLBATCH  64u           /* sectors per ftl_read() call */
#define N3G_BENCH_MAXENTRIES   8192u         /* pages of one repeat (16 MiB of 2 KiB pages for op CYCLE) */

#define N3G_BOP_READ      1u   /* raw page reads through the backend */
#define N3G_BOP_WRITE     2u   /* raw erase + program (+ optional verify read) of scratch blocks */
#define N3G_BOP_FTL_READ  3u   /* FTL sector reads (sequential or scattered) */
#define N3G_BOP_CYCLE     4u   /* save-read whole blocks to SDRAM, erase + program a pattern, verify every page,
                                  erase + program the saved pages back, verify them (needs N3G_BF_WRITE; pages =
                                  pages per block; always page-major across the banks) */
#define N3G_BOP_FTL_CYCLE 5u   /* the same through the FTL on logical sectors [first_sector, +sectors): save-read,
                                  ftl_write a pattern + ftl_sync, read back, ftl_write the saved sectors + ftl_sync,
                                  read back (needs N3G_BF_WRITE; mounts the FTL writable for the run, read-only after;
                                  a chip row not validated on hardware needs N3G_BF_UNVALIDATED) */

#define N3G_BF_WRITE        (1u << 0)   /* destructive operations allowed (op WRITE) */
#define N3G_BF_INTERLEAVE   (1u << 1)   /* page-major across the banks of the mask (default bank-major) */
#define N3G_BF_SCATTER      (1u << 2)   /* shuffle the read order (op READ) / random start per call (op FTL_READ) */
#define N3G_BF_TWOPLANE     (1u << 3)   /* program in plane pairs (needs geometry.twoplane, even first block/count) */
#define N3G_BF_UNVALIDATED  (1u << 4)   /* allow writes on a chip table row not validated on hardware */
#define N3G_BF_VERIFY       (1u << 5)   /* op WRITE: read back and compare after programming */
#define N3G_BF_ERASE_AFTER  (1u << 6)   /* op WRITE: erase the scratch blocks again at the end */
#define N3G_BF_LOAD         (1u << 7)   /* run a CPU-bound thread beside the benchmark and measure the CPU it gets */

struct n3g_bench_params {
    uint32_t magic, version;           /* 0x00 */
    uint32_t op;                       /* 0x08 N3G_BOP_* */
    uint32_t backend;                  /* 0x0c index, 0xffffffff = keep the active one */
    uint32_t flags;                    /* 0x10 N3G_BF_* */
    uint32_t bank_mask;                /* 0x14 */
    uint32_t first_block;              /* 0x18 first physical block per bank */
    uint32_t nblocks;                  /* 0x1c blocks per bank */
    uint32_t pages;                    /* 0x20 pages per block */
    uint32_t batch;                    /* 0x24 pages (sectors for FTL_READ) per backend call */
    uint32_t repeats;                  /* 0x28 */
    uint32_t seed;                     /* 0x2c shuffle / data pattern seed */
    uint32_t first_sector;             /* 0x30 FTL_READ: first sector of the window */
    uint32_t sectors;                  /* 0x34 FTL_READ: window size in sectors read per repeat */
    uint32_t fmss_poll_us;             /* 0x38 fmss backend: sleep between polls (0 = spin), 0xffffffff = keep */
    uint32_t timing;                   /* 0x3c NAND bus timing for this run, every backend: (setup << 16) | (hold << 12)
                                          [| EDO << 28]; 0 = the default (setup 4, hold 3), restored after the run */
    uint32_t reserved[16];             /* 0x40 */
};
#define N3G_BENCH_PARAM_BYTES 0x80u

struct n3g_bench_result {
    uint32_t magic, version;           /* 0x00 */
    uint32_t op, backend;              /* 0x08 */
    uint32_t status;                   /* 0x10 0 = ok, else the first library error code (as u32) */
    uint32_t flags;                    /* 0x14 echo */
    uint32_t repeats_done, batches;    /* 0x18 */
    uint32_t read_pages, read_bytes, read_us;            /* 0x20 */
    uint32_t ecc_clean, ecc_corrected, ecc_failed;       /* 0x2c */
    uint32_t read_errors;                                /* 0x38 batches whose call failed */
    uint32_t batch_us_min, batch_us_max;                 /* 0x3c over read batches */
    uint32_t erase_blocks, erase_us, erase_fail;         /* 0x44 */
    uint32_t prog_pages, prog_bytes, prog_us, prog_fail; /* 0x50 */
    uint32_t verify_pages, verify_us, verify_mismatch, verify_errors; /* 0x60 */
    uint32_t ftl_sectors, ftl_bytes, ftl_us, ftl_errors; /* 0x70 */
    uint32_t total_us;                                   /* 0x80 wall time of the whole run */
    uint32_t first_err_rc, first_err_bank, first_err_page; /* 0x84 */
    uint32_t erased_pages;                               /* 0x90 read results with bit 29 (erased) */
    uint32_t max_corrected;                              /* 0x94 largest n of an n-bit correction seen */
    uint32_t pages_per_block, page_size, entries;        /* 0x98 */
    uint32_t reps_recorded;                              /* 0xa4 valid entries of the per-repeat array */
    /* op CYCLE (the pattern phase uses erase_*, prog_*, verify_* above) */
    uint32_t cyc_failed_phase;                           /* 0xa8 N3G_CYC_* where it stopped, 0 = completed */
    uint32_t save_pages, save_us;                        /* 0xac save read of the original pages */
    uint32_t save_erased;                                /* 0xb4 original pages that were erased (left erased) */
    uint32_t rst_erase_blocks, rst_erase_us;             /* 0xb8 restore: erase */
    uint32_t rst_prog_pages, rst_prog_us;                /* 0xc0 restore: program of the saved pages */
    uint32_t rst_verify_pages, rst_verify_us;            /* 0xc8 restore: read back */
    uint32_t rst_mismatch;                               /* 0xd0 restored pages whose data or metadata differ */
    uint32_t rst_errors;                                 /* 0xd4 failed calls / ECC failures in the restore */
    uint32_t save_addr, save_bytes;                      /* 0xd8 SDRAM copy of the original pages */
    /* op FTL_CYCLE (save read in ftl_sectors / ftl_us / ftl_errors above; phases N3G_CYC_* as for CYCLE with
       2 = pattern sync and 5 = restore sync) */
    uint32_t ftl_wr_us, ftl_sync_us, ftl_vfy_us;         /* 0xe0 pattern: ftl_write, ftl_sync, read back */
    uint32_t ftl_rst_wr_us, ftl_rst_sync_us, ftl_rst_vfy_us; /* 0xec restore */
    uint32_t ftl_vfy_mismatch, ftl_rst_mismatch;         /* 0xf8 sectors that differ */
};
/* CPU measurement (result_addr + N3G_BENCH_CPU_OFF), filled by every op. The timed backend calls are accounted to
 * a phase (ops CYCLE / FTL_CYCLE: N3G_CYC_*, other ops: 0). With N3G_BF_LOAD a thread at the worker's priority
 * counts in a loop the whole run; load_rate_x1000 is its count per millisecond when it runs alone (measured at the
 * start while the worker sleeps), so phase_load / (load_rate_x1000 * phase_us / 1e6) is the share of the CPU
 * that was left for other work during that phase's backend calls. */
#define N3G_BENCH_CPU_MAGIC 0x4347334eu  /* "N3GC" */
struct n3g_bench_cpu {
    uint32_t magic, load;              /* load: 1 if the load thread ran */
    uint32_t load_rate_x1000;          /* load-thread counts per ms, alone */
    uint32_t fmss_poll_us;             /* the fmss backend's poll sleep during the run */
    uint32_t phase_us[8];              /* wall time of the backend calls per phase */
    uint32_t phase_load[8];            /* load-thread counts during them */
    uint32_t phase_maxcorr[8];         /* largest n-bit correction seen per phase */
    uint32_t timing;                   /* the bus timing word in effect (0 = default) */
    uint32_t data_sig, sig_pages;      /* op READ: signature of every page's data + 12 meta bytes in read order
                                          (sig = sig * 31 + crc32(page) + crc32(meta)), pages included */
    uint32_t ecc_fail_pages;           /* op READ: pages that read uncorrectable */
};
#define N3G_CYC_SAVE        1u   /* nothing was written */
#define N3G_CYC_ERASE       2u
#define N3G_CYC_PROGRAM     3u
#define N3G_CYC_VERIFY      4u
#define N3G_CYC_RST_ERASE   5u
#define N3G_CYC_RST_PROGRAM 6u
#define N3G_CYC_RST_VERIFY  7u
#define N3G_BENCH_RESULT_BYTES 0x100u

#endif
