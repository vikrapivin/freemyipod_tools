/* SPDX-License-Identifier: GPL-2.0-or-later
 * CPU-only NAND driver for the iPod nano 4G (S5L8720 FMC): read, program and
 * erase without running any FMSS sequencer program. The register sequences
 * are those of cpu_nand_ll.c; ECC is either the hardware ECC engine kicked
 * by the CPU or the software BCH of bch_sw.c (no ECC engine access at all),
 * selected per call.
 *
 * Every public operation resets the engine first (the drivers' rule),
 * bounds every wait, and on every exit releases the status latch,
 * deselects (FMC_CTRL0 = 0), acknowledges and runs the ordinary M2 reset;
 * the idle state after that reset is recorded in struct cnand.
 *
 * Page formats:
 *   CNAND_FMT_MAIN  Whimory page: meta[10] | par | data[512], then
 *                   7 x (par | data[512]); BCH8 (13-byte parity, 128-byte
 *                   spare) or BCH16 (26 bytes, 218-byte spare) by geometry;
 *                   sector 0's code word covers data then the ten meta bytes.
 *   CNAND_FMT_BOOT  boot-ROM reserved-region page: 3 x (par[26] | data[512]),
 *                   BCH16, no metadata; the rest of the raw page is 0xff.
 *
 * Program and erase accept every block of every CE, block 0 included.
 */
#ifndef N4G_CPU_NAND_STACK_H
#define N4G_CPU_NAND_STACK_H
#include "global.h"
#include "cpu_nand_ll.h"

#define CNAND_FMT_MAIN 0u
#define CNAND_FMT_BOOT 1u
#define CNAND_ECC_RAW  0u
#define CNAND_ECC_HW   1u
#define CNAND_ECC_SW   2u
#define CNAND_PATH_DMA  0u
#define CNAND_PATH_FIFO 1u

#define CNAND_OK         0
#define CNAND_ESTEP     (-1)    /* a controller step timed out */
#define CNAND_EREADY    (-2)    /* chip ready (status bit 6) timed out */
#define CNAND_EECC      (-3)    /* ECC engine completion or metadata latch timed out */
#define CNAND_ESTATUS   (-10)   /* chip status FAIL after program/erase */
#define CNAND_ERESET    (-20)   /* engine reset before the operation failed */
#define CNAND_EARG      (-31)
#define CNAND_ENOMEM    (-32)   /* software BCH tables could not be allocated */
#define CNAND_EDMA      (-33)   /* LCD DMA still busy */

#define CNAND_MAX_SECTORS   8u
#define CNAND_RAW_BUF       0x1200u /* raw page buffers: 4608 bytes (DMA moves whole 512-byte chunks) */

/* per-sector result flags */
#define CNAND_S_OK      1u      /* decoded: clean or corrected */
#define CNAND_S_UNCORR  2u      /* uncorrectable, data left as read */
#define CNAND_S_ERASED  4u      /* erased sector (hardware: ECC_STATUS bit 6; software: all bytes 0xff) */
#define CNAND_S_HW      8u
#define CNAND_S_SW      16u

struct cnand_sector {
    uint32_t flags, corrected;          /* corrected bits (data, metadata and parity) */
    uint32_t r810, r80c, r818, t_us;    /* hardware: ECC_STATUS, ring entry, totals after the decode/encode */
    uint32_t loc[7];                    /* corrected locations, 13-bit fields ascending (0x820.. layout) */
};
struct cnand_report {
    uint32_t nsectors, corrected_total, max_corrected, uncorrectable_mask, erased_mask, sw_us;
    uint32_t meta[3];                   /* main format: the 10 metadata bytes (12-byte container) */
    struct cnand_sector sector[CNAND_MAX_SECTORS];
};
struct cnand {
    struct cnll ll;
    bool optimized;                    /* shared API: fast polls, no diagnostic reads, no trailing reset */
    uint32_t pre_reset_rc, pre[3];      /* C08, FMC_STATUS, ECC_STATUS before */
    uint32_t cleanup[3];                /* FMC_CTRL0, FMCTRL1, FMC_STATUS after release */
    uint32_t post_reset_rc, post[7];    /* after reset: C08 C0C 000 004 048 810 818 */
    uint32_t elapsed_us;
    uint32_t strength;                  /* 0 (FIL), 8 or 16 */
    uint32_t read_ctrl0;                /* hardware-read bus timing/path; initialized to existing default */
};

/* Main-format code: n->strength 8 or 16, or 0 = the FIL's current choice (nand_get_ecc_bits(): BCH8
 * until a programmed page decoded only with BCH16). 0 before the FIL found the chips. */
uint32_t cnand_strength(const struct cnand *n);
uint32_t cnand_rawsize(const struct cnand *n);  /* raw page bytes: 4096 + 128 (BCH8) or 4096 + 218 (BCH16) */
uint32_t cnand_datasize(uint32_t fmt);          /* 4096 or 0x600 */

/* n: caller-owned context (step log may be NULL with max 0). raw buffers hold CNAND_RAW_BUF bytes,
 * 32-byte aligned (DMA, cache maintenance). */
void cnand_init(struct cnand *n, struct cpu_nand_step *steps, uint32_t max_steps, uint32_t strength);
int cnand_read_raw(struct cnand *n, uint32_t ce, uint32_t row, uint8_t *raw, uint32_t len, uint32_t path);
/* ecc HW: engine decode per sector (DMA path; raw unused). SW: raw read by path into raw, then
 * software decode. data: cnand_datasize(fmt) bytes (512-byte DMA granules). */
int cnand_read_page(struct cnand *n, uint32_t ce, uint32_t row, uint32_t fmt, uint32_t ecc, uint32_t path,
                    uint8_t *data, uint8_t *raw, struct cnand_report *rep);
/* One main-format page in a multi-page read. data is the 4096-byte DMA target. */
#define CNAND_READ_SPAN 32u
struct cnand_main_page {
    uint32_t ce, row;
    uint8_t *data;
    uint32_t *meta;
};
/* Called after each page's data, metadata and sector report are complete. */
typedef int (*cnand_page_done)(void *ctx, uint32_t index, const struct cnand_report *rep);
/* One controller session for several main-format pages. Every chip in a wave
 * is given its array read (00h-30h) before any of those pages is decoded, and
 * the next page on a chip is started once that chip has clocked its data out,
 * while the last sector is still in ECC and DMA. */
int cnand_read_main_pages(struct cnand *n, const struct cnand_main_page *pages, uint32_t count,
                          cnand_page_done done, void *ctx);
/* Encode only: data (+ meta[10] for MAIN) -> raw page image (rawsize bytes, tail 0xff). No NAND access
 * beyond the ECC engine for HW. */
int cnand_build_page(struct cnand *n, uint32_t fmt, uint32_t ecc, const uint8_t *data, const uint8_t *meta,
                     uint8_t *raw, struct cnand_report *rep);
/* ecc RAW: program raw as given. SW: build into raw, then program that image.
 * HW: DMA, encode and stream each sector. raw may be NULL for HW; otherwise it
 * receives the host page image (diagnostic jobs). */
int cnand_program_page(struct cnand *n, uint32_t ce, uint32_t row, uint32_t fmt, uint32_t ecc,
                       const uint8_t *data, const uint8_t *meta, uint8_t *raw);
int cnand_erase_block(struct cnand *n, uint32_t ce, uint32_t block);

/* ---------------------------------------------------------------- USB job ABI
 * Sub-operations of NPOC_CMD_CPU_NAND (the m2_cpu_nand ops 0..5 keep their ABI):
 *   word1 = op | ce << 8 | ecc << 12 | fmt << 14 | path << 16 | bch << 17 (bits 19..31 zero)
 *           ecc CNAND_ECC_*, fmt CNAND_FMT_*, path CNAND_PATH_* (raw and software-ECC reads),
 *           bch 0 = the FIL's current main-format code, 1 = BCH8, 2 = BCH16
 *   word2 = row (page; ERASE: any page of the block)
 *   word3 = 0
 * Result buffer: record at 0, step log at CNS_STEPS, data at CNS_DATA, input at
 * CNS_IN (data, or the raw image for ecc RAW; metadata at CNS_IN + CNS_IN_META),
 * raw page image at CNS_RAW (raw reads, software-decoded reads, built pages). */
#define CNS_OP_INFO    0x10u    /* no NAND or register access: version, geometry */
#define CNS_OP_READ    0x11u
#define CNS_OP_PROGRAM 0x12u
#define CNS_OP_ERASE   0x13u
#define CNS_OP_BUILD   0x14u    /* encode only (HW or SW) into CNS_RAW */
#define CNS_MAGIC      0x53555043u  /* "CPUS" */
#define CNS_VERSION    1u
#define CNS_STEPS      0x400u
#define CNS_MAX_STEPS  256u
#define CNS_DATA       0x2000u
#define CNS_IN         0x4000u
#define CNS_IN_META    0x1200u
#define CNS_RAW        0x6000u
#define CNS_BYTES      0x7200u

struct cns_out {
    uint32_t magic, version, op, word, ce, row, unused0[2];                    /* 0x00 */
    uint32_t rc, unused1, strength, rawsize, elapsed_us, nsteps, dropped, nand_status; /* 0x20 */
    uint32_t ctrl0, bytes, raw_bytes, ecc, fmt, path, max_steps, reserved;      /* 0x40 */
    uint32_t pre_reset_rc, pre[3], cleanup[3], post_reset_rc, post[7];         /* 0x60 */
    uint32_t reserved2;                                                        /* 0x9c */
    struct cnand_report rep;                                                   /* 0xa0 */
};

/* ready: the FIL step succeeded (clock gates, pins, chip IDs). Returns the operation rc; *result gets an
 * NPOC_R_* code. */
int cpu_nand_stack_job(uint8_t *buf, const uint32_t *arg, int ready, uint32_t *result, uint32_t *detail,
                       uint32_t *len);
#endif
