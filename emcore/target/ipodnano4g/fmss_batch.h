/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef N4G_FMSS_BATCH_H
#define N4G_FMSS_BATCH_H
#include "global.h"
#include "nand_hal_emcore.h"
#define FMSS_BATCH_CAP ((1u << 20) | (1u << 21))
#define FMSS_BATCH_MAGIC 0x3142544d /* MTB1 */
#define FMSS_BATCH_OUTPUT 0x314f544d /* MTO1 */
#define FMSS_BATCH_INPUT_MAX (24u * 1024 * 1024)
#define FMSS_BATCH_OUTPUT_MAX (16u * 1024 * 1024)
#define FMSS_BATCH_COUNT_MAX 4096u
#define FMSS_BATCH_PINS 1u
#define FMSS_BATCH_BLOCKS 2u
/* v2 header.reserved[0]: count pairs {action_offset, action_count}.
 * Each action is {kind, source, destination, bytes}: 1 saves previous work
 * into dependency storage; 2 copies dependency bytes into new work; 3 XORs
 * destination scratch byte with source. Work offsets are allocation-relative. */
struct fmss_batch_header { uint32_t magic, version, count, table, bytes, reserved[3]; };
struct fmss_batch_case {
    uint32_t id, request, request_bytes, scratch, scratch_bytes, scratch_read, samples, flags;
};
struct fmss_batch_output { uint32_t magic, version, total, first, next, bytes, errors, elapsed_us; };
struct fmss_batch_record { uint32_t id, rc, bytes, head_bytes, samples, map_bytes, scratch_bytes, reserved; };
struct fmss_batch_run {
    const uint8_t *input;
    uint32_t input_bytes;
    uint8_t *output;
    uint32_t output_bytes;
    uint8_t *work; /* existing 256 KiB probe allocation; scratch addresses stay fixed */
    uint32_t first;
    int (*prepare)(uint32_t id, uint32_t flags); /* runs before each case, may publish progress */
};
/* Validate the whole manifest before executing. Return 0 on a complete or
 * output-full page, -1 on malformed input. out.next is the exact resume cursor.
 * Individual probe errors are records and do not stop subsequent cases. */
int fmss_batch_run(const struct n4g_probe_hal *hal, const struct fmss_batch_run *b);
#endif
