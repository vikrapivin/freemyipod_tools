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

/* NAND worker protocol. Commands queue work and return promptly; hosts poll
 * the fixed-size status record and read the result buffer over USB.
 * Boot creates the worker without accessing the NAND hardware.
 * tools/nano3g/protocol.py and nand.py mirror these layouts. */
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

#endif
