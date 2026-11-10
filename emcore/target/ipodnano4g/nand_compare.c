/* SPDX-License-Identifier: GPL-2.0-or-later
 * One resident workload calls the shared NAND API with either physical backend.
 * Timing encloses only the API call, excluding pattern generation, verification,
 * checksums and USB. Both modes use the controller's hardware BCH engine.
 */
#include "nand_compare.h"
#include "nand/nand-target.h"
#include "nand/rockbox-compat.h"
#include "nand/nand-profile.h"
#include "crc32.h"
#include "util.h"

struct compare_header {
    uint32_t magic, version, bytes, record_bytes, count, ce, block, pages;
    uint32_t repetitions, strength, failures, previous_backend, previous_ecc, elapsed_us, complete, reserved;
};
struct compare_record {
    uint32_t op, backend, writer, row;
    int32_t rc;
    uint32_t elapsed_us, compared_bytes, mismatch_bytes, first_offset, expected, actual;
    uint32_t data_crc, meta_crc, status[2], meta[3], reserved[6];
};
static uint8_t data[NAND_PAGE_SIZE] CACHEALIGN_ATTR;
static uint8_t expected[NAND_PAGE_SIZE] CACHEALIGN_ATTR;
static uint32_t meta[NAND_META_WORDS] CACHEALIGN_ATTR;
static uint32_t expected_meta[NAND_META_WORDS] CACHEALIGN_ATTR;

/* Owned by the serialized NAND worker; disabled outside profiled API calls. */
static bool profile_enabled;
static unsigned int profile_phase;
static uint32_t profile_last, profile_times[NAND_PROFILE_PHASES];
void nand_profile_begin(bool enabled)
{
    memset(profile_times, 0, sizeof(profile_times));
    profile_phase = NAND_PROFILE_OTHER;
    profile_enabled = enabled;
    if (enabled) profile_last = read_usec_timer();
}
unsigned int nand_profile_enter(unsigned int phase)
{
    unsigned int previous = profile_phase;
    if (profile_enabled) {
        uint32_t now = read_usec_timer();
        profile_times[previous] += now - profile_last;
        profile_last = now;
        profile_phase = phase;
    }
    return previous;
}
void nand_profile_leave(unsigned int previous) { nand_profile_enter(previous); }
void nand_profile_end(uint32_t *times)
{
    profile_enabled = false;
    memcpy(times, profile_times + 1, (NAND_PROFILE_PHASES - 1) * sizeof(*times));
}

static void pattern(uint32_t ce, uint32_t page, int erased)
{
    uint32_t x = 0x4d320001u ^ (ce << 24) ^ (page * 0x9e3779b9u);
    uint8_t *m = (uint8_t *)expected_meta;
    for (uint32_t i = 0; i < sizeof(expected); i++) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        expected[i] = erased ? 0xff : page % 3 == 0 ? (uint8_t)(i + ce) :
                      page % 3 == 1 ? (uint8_t)(i & 1 ? 0xaa : 0x55) : (uint8_t)x;
    }
    memset(expected_meta, 0xff, sizeof(expected_meta));
    if (!erased) {
        m[0] = 0x4d; m[1] = 0x32; m[2] = ce; m[3] = page;
        m[4] = 0; m[5] = 0xff; m[6] = 0x55; m[7] = 0xaa;
        m[8] = page >> 8; m[9] = ce ^ page ^ 0x81;
    }
}

static int operation(struct compare_header *h, uint32_t op, uint32_t backend,
                     uint32_t writer, uint32_t row, uint32_t index, int erased, int profiled, int baseline)
{
    struct compare_record *r = (void *)((uint8_t *)(h + 1) + h->count * h->record_bytes);
    uint32_t start;
    memset(r, 0, sizeof(*r));
    r->op = op; r->backend = backend; r->writer = writer; r->row = row;
    r->first_offset = 0xffffffff;
    profiled = profiled && h->reserved;
    baseline = baseline && h->reserved && op == 1 && backend == NAND_BACKEND_CPU;
    r->reserved[0] = erased | (profiled << 8) | (baseline << 9);
    nand_set_cpu_optimized(!baseline);
    nand_set_backend(backend);
    pattern(h->ce, index, erased);
    if (op == 2) { memcpy(data, expected, sizeof(data)); memcpy(meta, expected_meta, sizeof(meta)); }
    else { memset(data, 0xa5, sizeof(data)); memset(meta, 0xa5, sizeof(meta)); }
    nand_profile_begin(profiled);
    start = read_usec_timer();
    if (op == 3) r->rc = nand_erase_block(h->ce, row / nand_get_geometry()->pagesperblock);
    else if (op == 2) r->rc = nand_write_page(h->ce, row, data, meta);
    else r->rc = nand_read_page(h->ce, row, data, meta);
    r->elapsed_us = read_usec_timer() - start;
    nand_profile_end(r->reserved + 1);
    if (op == 1) {
        uint8_t *got_meta = (uint8_t *)meta, *want_meta = (uint8_t *)expected_meta;
        nand_copy_sector_status(r->status);
        memcpy(r->meta, meta, sizeof(meta));
        r->data_crc = crc32(data, sizeof(data));
        r->meta_crc = crc32(meta, 10);
        r->compared_bytes = sizeof(data) + 10;
        for (uint32_t i = 0; i < r->compared_bytes; i++) {
            uint32_t got = i < sizeof(data) ? data[i] : got_meta[i - sizeof(data)];
            uint32_t want = i < sizeof(data) ? expected[i] : want_meta[i - sizeof(data)];
            if (got != want) {
                if (!r->mismatch_bytes) { r->first_offset = i; r->expected = want; r->actual = got; }
                r->mismatch_bytes++;
            }
        }
    }
    h->count++;
    if (r->mismatch_bytes || (r->rc && !(op == 1 && r->rc == NAND_ECC_CORRECTED))) h->failures++;
    /* A failed/timed-out operation is recorded once. Do not replay or keep
     * issuing traffic through a controller whose completion is unknown. */
    return r->rc < 0 || (op != 1 && r->rc) ? -1 : 0;
}

int nand_compare(uint8_t *out, uint32_t capacity, const uint32_t *args, uint32_t *length)
{
    const struct nand_geometry *g = nand_get_geometry();
    struct compare_header *h = (void *)out;
    uint32_t ce = args[0], block = args[1], pages = args[2] & 0xffff;
    uint32_t repetitions = (args[2] >> 16) & 0xff, strength = (args[2] >> 24) & 0x7f;
    uint32_t profile = args[2] >> 31;
    uint64_t count = 6ull + 2ull * pages + 4ull * pages * repetitions;
    uint32_t start = read_usec_timer();
    *length = 0;
    if (!g || ce >= g->banks || block >= g->blocks - 1 || !pages || pages > g->pagesperblock ||
        !repetitions || (profile && repetitions % 4) || (strength != 8 && strength != 16) ||
        sizeof(*h) + count * sizeof(struct compare_record) > capacity) return -31;
    memset(h, 0, sizeof(*h));
    h->magic = 0x3143424e; /* NBC1 */
    h->version = profile ? 2 : 1; h->record_bytes = sizeof(struct compare_record);
    h->reserved = profile;
    h->ce = ce; h->block = block; h->pages = pages; h->repetitions = repetitions; h->strength = strength;
    h->previous_backend = nand_get_backend(); h->previous_ecc = nand_get_ecc_bits();
    nand_set_ecc_bits(strength);
    for (uint32_t n = 0; n < 2; n++) {
        uint32_t writer = n ^ (ce & 1), base = (block + writer) * g->pagesperblock;
        if (operation(h, 3, writer, writer, base, 0, 1, 1, 0)) goto done;
        for (uint32_t reader = 0; reader < 2; reader++)
            if (operation(h, 1, reader, writer, base, 0, 1, 1, 0)) goto done;
        for (uint32_t page = 0; page < pages; page++)
            if (operation(h, 2, writer, writer, base + page, page, 0, 1, 0)) goto done;
    }
    for (uint32_t repeat = 0; repeat < repetitions; repeat++)
        for (uint32_t writer = 0; writer < 2; writer++)
            for (uint32_t n = 0; n < 2; n++) {
                uint32_t reader = n ^ ((ce + repeat + writer) & 1);
                for (uint32_t page = 0; page < pages; page++)
                    if (operation(h, 1, reader, writer, (block + writer) * g->pagesperblock + page,
                                  page, 0, !(repeat & 2), (repeat ^ ce ^ writer) & 1)) goto done;
            }
    h->complete = 1;
done:
    nand_set_cpu_optimized(true);
    nand_set_backend(h->previous_backend);
    nand_set_ecc_bits(h->previous_ecc);
    h->elapsed_us = read_usec_timer() - start;
    h->bytes = sizeof(*h) + h->count * sizeof(struct compare_record);
    *length = h->bytes;
    return h->complete && !h->failures ? 0 : -1;
}
