/* SPDX-License-Identifier: GPL-2.0-or-later
 * CPU-direct NAND operations on the S5L8720 FMC (no FMSS sequencer program).
 * Each operation replicates the register sequence of a source program
 * (target/s5l87xx/fmss/erase.s, nand/fmss/program.s, read.s, boot_read.s,
 * tools/nano4g/sequences.py) as CPU writes, polling FMC_STATUS (0x048) or
 * ECC_STATUS (0x810) where the program uses WAIT. Every wait is bounded; the
 * controller is released and FMC_CTRL0 cleared on every exit, then the
 * ordinary M2 reset runs and the idle state is recorded.
 *
 * Request (USB NPOC_CMD_CPU_NAND, queued as a worker job):
 *   word1 = op | ce << 8 | slot << 12, word2 = row (page), word3 = param.
 * Input in the result buffer at CPU_NAND_IN (uploaded by the host first);
 * output record at offset 0, data at CPU_NAND_DATA.
 * Ops 0x10..0x14 of the same command are the CPU-only NAND driver
 * (cpu_nand_stack.h); nand_poc.c routes them there.
 */
#ifndef N4G_CPU_NAND_H
#define N4G_CPU_NAND_H
#include "global.h"
#include "cpu_nand_ll.h"

#define CPU_NAND_MAGIC     0x4e555043u   /* "CPUN" */
#define CPU_NAND_VERSION   1u
#define CPU_NAND_DATA      0x2000u
#define CPU_NAND_IN        0x4000u
#define CPU_NAND_IN_BYTES  0x1400u       /* raw page (<= 4608 incl. DMA tail) + 12 metadata bytes at +0x1200 */
#define CPU_NAND_IN_META   0x1200u
#define CPU_NAND_MAX_RAW   (4096u + 512u)
#define CPU_NAND_MAX_STEPS 512u

/* Generic CPU sequence. Input header at IN: magic, count, CTRL0, payload bytes;
 * then count four-word instructions. Payload at IN+0x1000; output at 0x20000.
 * Neither commands nor physical blocks have an allow-list. */
#define CPU_NAND_SEQ_MAGIC 0x31514553u
#define CPU_NAND_SEQ_MAX   128u
#define CPU_NAND_SEQ_PAYLOAD 0x1000u
#define CPU_NAND_SEQ_BYTES 0x10000u
#define CPU_NAND_SEQ_DATA  0x20000u
#define CPU_NAND_SEQ_TEMP  0x18000u
enum cpu_nand_seq_kind {
    CPU_NAND_SEQ_CMD = 1,       /* command byte */
    CPU_NAND_SEQ_ADDR = 2,      /* byte count 1..8, low bytes, high bytes */
    CPU_NAND_SEQ_OUT = 3,       /* byte count, payload offset, reserved=0 */
    CPU_NAND_SEQ_IN = 4,        /* byte count, output offset, path (0 DMA / 1 FIFO) */
    CPU_NAND_SEQ_READY = 5,     /* timeout us, status command (0x100 = already sent), mask */
    CPU_NAND_SEQ_TIMING = 6,    /* full CTRL0 without CE selection */
    CPU_NAND_SEQ_DELAY = 7,     /* delay us */
    CPU_NAND_SEQ_SELECT = 8,    /* chip enable a (0..7) for the following steps */
};
struct cpu_nand_seq_insn { uint32_t kind, a, b, c; };
struct cpu_nand_seq {
    uint32_t magic, count, ctrl0, payload_bytes;
    struct cpu_nand_seq_insn insn[CPU_NAND_SEQ_MAX];
};
struct cpu_nand_map_page {
    uint32_t row, rc, flags, crc;
    uint8_t prefix[32], marker[8];
    uint32_t formats;          /* exact BCH parity masks: boot bits0..2, main8 8..15, main16 16..23 */
    uint8_t tail[4];
};

enum cpu_nand_op {
    CPU_NAND_STATE = 0,     /* no NAND access: reset and idle snapshot */
    CPU_NAND_ERASE = 1,     /* row = first page of the block */
    CPU_NAND_PROGRAM = 2,   /* param = raw bytes (DMA path, 512-byte chunks from CPU_NAND_IN) */
    CPU_NAND_READ_RAW = 3,  /* param = raw bytes | path << 16 (0 DMA, 1 FIFO) */
    CPU_NAND_READ_ECC = 4,  /* param = format (CPU_NAND_FMT_*): CPU-kicked decode per sector */
    CPU_NAND_ENCODE = 5,    /* param = ECC_CTRL word (encode); 512 bytes at CPU_NAND_IN, slot from word1 */
    CPU_NAND_SEQUENCE = 6,  /* generic commands/address/data/wait steps, uploaded at CPU_NAND_IN */
    CPU_NAND_MAP = 7,       /* read-only row..row+param-1, <=256 pages; compact map at SEQ_DATA */
};
#define CPU_NAND_FMT_MAIN8  0u  /* meta[10] | par13 | data, 7 x (par13 | data); 0xa01 / 0x001 */
#define CPU_NAND_FMT_MAIN16 1u  /* same with par26; 0xa05 / 0x005 */
#define CPU_NAND_FMT_ROM    2u  /* boot-ROM reserved region: 3 x (par26 | data); 0x808 = 0x80, 0x005 */

/* step tags and struct cpu_nand_step: cpu_nand_ll.h */

struct cpu_nand_ecc {           /* one sector, read right after its WAIT4-equivalent */
    uint32_t r810, r80c, r818, t_us;
    uint32_t loc[8];            /* 0x820..0x83c */
    uint32_t par[7];            /* 0x840..0x858 (encode only) */
    uint32_t r810_ack;          /* after acknowledge 1 */
};
struct cpu_nand_out {
    uint32_t magic, version, op, ce, row, param, rc, unused;        /* 0x00 */
    uint32_t elapsed_us, nsteps, dropped, nand_status;              /* 0x20 */
    uint32_t ctrl0, bytes, nsectors, meta[3];                       /* 0x30 */
    uint32_t pre_reset_rc, pre[3];                                  /* 0x48 C08, FMC_STATUS, ECC_STATUS before */
    uint32_t cleanup[3];                                            /* 0x58 FMC_CTRL0, FMCTRL1, FMC_STATUS after release */
    uint32_t post_reset_rc, post[7];                                /* 0x64 after reset: C08 C0C 000 004 048 810 818 */
    struct cpu_nand_ecc ecc[8];                                     /* 0x84 */
    struct cpu_nand_step step[CPU_NAND_MAX_STEPS];
};

/* Returns the operation rc (0, or negative); *result gets an NPOC_R_* code. */
int cpu_nand_job(uint8_t *buf, const uint32_t *arg, uint32_t *result, uint32_t *detail, uint32_t *len);
/* CPU batch v1 uses the M2 allocation/run/free commands with its own magic.
 * Header: fmss_batch_header. Case: id, word1, row, param, upload offset/bytes,
 * two reserved zeros. Output header: fmss_batch_output; records contain
 * id, result, rc, head bytes, data bytes, detail[2], reserved zero, followed
 * by the CPUN header/used steps and data (no fixed-offset padding). */
#define CPU_NAND_BATCH_MAGIC 0x31425443u /* CTB1 */
#define CPU_NAND_BATCH_OUTPUT 0x314f5443u /* CTO1 */
struct fmss_batch_run;
int cpu_nand_batch_run(const struct fmss_batch_run *b);
#endif
