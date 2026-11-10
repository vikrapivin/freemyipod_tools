/* SPDX-License-Identifier: GPL-2.0-or-later
 * Run a resident experiment manifest without a host handshake between cases.
 * The ordinary probe still owns reset, preflight, execution and cleanup.
 */
#include "global.h"
#include "timer.h"
#include "nand_poc.h"
#include "fmss_probe.h"
#include "fmss_batch.h"
#define HEAD_BYTES __builtin_offsetof(struct fmss_probe_out, s)
#define ALIGN4(n) (((n) + 3u) & ~3u)
static int range(uint32_t off, uint32_t len, uint32_t cap)
{ return off <= cap && len <= cap - off; }

/* One worker owns these arrays. Neither is DMA-visible. */
static uint8_t initial_scratch[N4G_PROBE_SCRATCH_BYTES];
static uint8_t dependency[N4G_PROBE_SCRATCH_BYTES];
static int dependency_valid;

static int scratch_blocks(const struct fmss_batch_run *b, const struct fmss_batch_case *d, uint8_t *dst)
{
    const uint32_t *p = (const void *)(b->input + d->scratch);
    uint32_t i, done = 0, total, parts;
    if ((d->scratch & 3) || d->scratch_bytes < 8) return -1;
    total = p[0]; parts = p[1];
    if (total > N4G_PROBE_SCRATCH_BYTES || parts > 512 || d->scratch_bytes != 8 + parts * 8) return -1;
    for (i = 0; i < parts; i++) {
        uint32_t repeat = p[2 + i * 2], off = p[3 + i * 2], j;
        if (!repeat || repeat > 512 || !range(off, 64, b->input_bytes) ||
            repeat * 64 > ((total + 63u) & ~63u) - done) return -1;
        for (j = 0; j < repeat; j++) {
            uint32_t n = total - done; if (n > 64) n = 64;
            if (dst) memcpy(dst + done, b->input + off, n);
            done += 64;
        }
    }
    return done == ((total + 63) & ~63u) ? (int)total : -1;
}

static int action_valid(const uint32_t *p)
{
    uint32_t kind = p[0], src = p[1], dst = p[2], n = p[3];
    if (!n) return 0;
    if (kind == 1) return range(src, n, NPOC_RESULT_BYTES) && range(dst, n, sizeof(dependency)) &&
        ((src >= N4G_PROBE_OUT && src + n <= N4G_PROBE_OUT + sizeof(struct fmss_probe_out)) || src >= N4G_PROBE_SCRATCH);
    if (kind == 2) return range(src, n, sizeof(dependency)) && range(dst, n, NPOC_RESULT_BYTES) &&
        ((dst >= N4G_PROBE_PROG && dst + n <= N4G_PROBE_OUT) || dst >= N4G_PROBE_SCRATCH);
    return kind == 3 && src <= 255 && n == 1 && dst >= N4G_PROBE_SCRATCH && dst < NPOC_RESULT_BYTES;
}

int fmss_batch_run(const struct n4g_probe_hal *hal, const struct fmss_batch_run *b)
{
    const struct fmss_batch_header *h = (const void *)b->input;
    struct fmss_batch_output *out = (void *)b->output;
    const struct fmss_batch_case *d;
    uint32_t i, started = read_usec_timer();
    if (b->input_bytes < sizeof(*h) || b->input_bytes > FMSS_BATCH_INPUT_MAX ||
        b->output_bytes < sizeof(*out) || b->output_bytes > FMSS_BATCH_OUTPUT_MAX ||
        ((uintptr_t)b->input & 3) || ((uintptr_t)b->output & 3) || ((uintptr_t)b->work & 31)) return -1;
    memset(out, 0, sizeof(*out));
    out->magic = FMSS_BATCH_OUTPUT; out->version = 3;
    out->bytes = sizeof(*out); out->first = out->next = b->first;
    if (h->magic != FMSS_BATCH_MAGIC || (h->version != 1 && h->version != 2) || !h->count ||
        h->count > FMSS_BATCH_COUNT_MAX || h->bytes != b->input_bytes || b->first > h->count ||
        h->table < sizeof(*h) || (h->table & 3) ||
        !range(h->table, h->count * sizeof(*d), b->input_bytes)) return -1;
    d = (const void *)(b->input + h->table);
    out->total = h->count;
    if (h->version == 2 && ((h->reserved[0] & 3) ||
        !range(h->reserved[0], h->count * 8, b->input_bytes))) return -1;
    if (!b->first) dependency_valid = 0;
    for (i = 0; i < h->count; i++) {
        const struct fmss_probe_req *q;
        uint32_t qbytes, max_record;
        if (d[i].id != i || (d[i].request & 3) || d[i].request_bytes < 44 ||
            !range(d[i].request, d[i].request_bytes, b->input_bytes) ||
            !range(d[i].scratch, d[i].scratch_bytes, b->input_bytes) ||
            d[i].scratch_bytes > N4G_PROBE_SCRATCH_BYTES || d[i].scratch_read > N4G_PROBE_SCRATCH_BYTES ||
            d[i].samples > FMSS_PROBE_MAXSAMPLES || (d[i].flags & ~(FMSS_BATCH_PINS | (h->version == 2 ? FMSS_BATCH_BLOCKS : 0)))) return -1;
        if (d[i].flags & FMSS_BATCH_BLOCKS && scratch_blocks(b, &d[i], NULL) < 0) return -1;
        if (h->version == 2) {
            const uint32_t *at = (const void *)(b->input + h->reserved[0] + i * 8);
            uint32_t j;
            if ((at[0] & 3) || at[1] > 4096 || !range(at[0], at[1] * 16, b->input_bytes)) return -1;
            for (j = 0; j < at[1]; j++)
                if (!action_valid((const void *)(b->input + at[0] + j * 16))) return -1;
        }
        q = (const void *)(b->input + d[i].request);
        if (q->nparams > FMSS_PROBE_MAXPARAMS || !q->prog_bytes || (q->prog_bytes & 7) ||
            q->prog_bytes > N4G_PROBE_OUT - N4G_PROBE_PROG) return -1;
        qbytes = 44 + q->nparams * 8;
        if (qbytes > d[i].request_bytes || q->prog_bytes != d[i].request_bytes - qbytes) return -1;
        max_record = sizeof(struct fmss_batch_record) + sizeof(struct fmss_probe_diag) + HEAD_BYTES + d[i].samples * 40 +
            ((q->flags & FMSS_PROBE_FULLMAP) ? 8192 : 0) + d[i].scratch_read;
        if (ALIGN4(max_record) > b->output_bytes - sizeof(*out)) return -1;
    }
    hal->cache_wb(out, sizeof(*out));
    for (i = b->first; i < h->count; i++) {
        const struct fmss_probe_req *q = (const void *)(b->input + d[i].request);
        struct fmss_probe_out *o = (void *)(b->work + N4G_PROBE_OUT);
        struct fmss_batch_record *r;
        uint32_t qbytes = 44 + q->nparams * 8, ns, maps = (q->flags & FMSS_PROBE_FULLMAP) ? 8192 : 0;
        uint32_t reserve = ALIGN4(sizeof(*r) + sizeof(struct fmss_probe_diag) + HEAD_BYTES + d[i].samples * 40 + maps + d[i].scratch_read);
        uint8_t *p;
        int rc = 0, initial_bytes;
        uint32_t j, action_count = 0;
        const uint32_t *actions = NULL;
        if (h->version == 2) {
            const uint32_t *at = (const void *)(b->input + h->reserved[0] + i * 8);
            actions = (const void *)(b->input + at[0]); action_count = at[1];
        }
        if (reserve > b->output_bytes - out->bytes) break; /* no case replay on resume */
        for (j = 0; j < action_count; j++) {
            const uint32_t *a = actions + j * 4;
            if (a[0] != 1) continue;
            dependency_valid = o->magic == 0x3452504f && o->pre_ok && o->polls && o->why == FMSS_PROBE_END;
            if (dependency_valid) memcpy(dependency + a[2], b->work + a[1], a[3]);
        }
        memset(b->work, 0, N4G_PROBE_PROG);
        memcpy(b->work, q, qbytes);
        memcpy(b->work + N4G_PROBE_PROG, (const uint8_t *)q + qbytes, q->prog_bytes);
        initial_bytes = d[i].flags & FMSS_BATCH_BLOCKS ? scratch_blocks(b, &d[i], initial_scratch) : (int)d[i].scratch_bytes;
        if (!(d[i].flags & FMSS_BATCH_BLOCKS) && initial_bytes)
            memcpy(initial_scratch, b->input + d[i].scratch, initial_bytes);
        if (initial_bytes) memcpy(b->work + N4G_PROBE_SCRATCH, initial_scratch, initial_bytes);
        for (j = 0; j < action_count; j++) {
            const uint32_t *a = actions + j * 4;
            if (a[0] == 1) continue;
            if (!dependency_valid) { rc = -4; break; }
            if (a[0] == 2) memcpy(b->work + a[2], dependency + a[1], a[3]);
            else b->work[a[2]] ^= a[1];
        }
        memset(o, 0, sizeof(*o)); /* rejected requests must not expose a preceding capture */
        memset(&fmss_probe_diagnostics, 0, sizeof(fmss_probe_diagnostics));
        if (!rc) rc = b->prepare ? b->prepare(i, d[i].flags) : 0;
        if (!rc) {
            hal->cache_wb_inv(b->work + N4G_PROBE_SCRATCH, N4G_PROBE_SCRATCH_BYTES);
            rc = fmss_probe(hal, (const void *)b->work, (const void *)(b->work + N4G_PROBE_PROG), o);
            n4g_emcore_cache_inv(b->work + N4G_PROBE_SCRATCH, N4G_PROBE_SCRATCH_BYTES);
        }
        ns = o->nsamples < d[i].samples ? o->nsamples : d[i].samples;
        o->dropped += o->nsamples - ns; o->nsamples = ns;
        r = (void *)(b->output + out->bytes);
        r->id = i; r->rc = rc; r->head_bytes = HEAD_BYTES; r->samples = ns;
        r->map_bytes = maps; r->scratch_bytes = d[i].scratch_read; r->reserved = 0;
        p = (void *)(r + 1);
        memcpy(p, o, HEAD_BYTES + ns * 40); p += HEAD_BYTES + ns * 40;
        if (maps) { memcpy(p, o->fullmap, maps); p += maps; }
        if (d[i].scratch_read) {
            const uint8_t *sc = b->work + N4G_PROBE_SCRATCH;
            const uint8_t *initial = initial_scratch;
            uint32_t k, n, changes = 0;
            /* Lossless word patches against the uploaded initializer avoid
             * sending identical 8 KiB patterns back for every opcode. */
            if ((uint32_t)initial_bytes >= d[i].scratch_read) {
                for (k = 0; k < d[i].scratch_read; k += 4) {
                    n = d[i].scratch_read - k; if (n > 4) n = 4;
                    if (memcmp(sc + k, initial + k, n)) changes++;
                }
            } else changes = d[i].scratch_read;
            if (changes * 8 < d[i].scratch_read) {
                r->reserved = 1; r->scratch_bytes = changes * 8;
                for (k = 0; k < d[i].scratch_read; k += 4) {
                    n = d[i].scratch_read - k; if (n > 4) n = 4;
                    if (!memcmp(sc + k, initial + k, n)) continue;
                    memcpy(p, &k, 4); memset(p + 4, 0, 4); memcpy(p + 4, sc + k, n); p += 8;
                }
            } else { memcpy(p, sc, d[i].scratch_read); p += d[i].scratch_read; }
        }
        memcpy(p, &fmss_probe_diagnostics, sizeof(fmss_probe_diagnostics)); p += sizeof(fmss_probe_diagnostics);
        r->bytes = ALIGN4(p - (uint8_t *)r);
        memset(p, 0, r->bytes - (p - (uint8_t *)r));
        /* Commit completed records before advertising the prefix. This permits
         * inspection during a later stuck case without a per-case USB exchange. */
        hal->cache_wb(r, r->bytes);
        out->bytes += r->bytes; out->next = i + 1;
        if (rc) out->errors++;
        hal->cache_wb(out, sizeof(*out));
    }
    out->elapsed_us = read_usec_timer() - started;
    hal->cache_wb(out, out->bytes);
    return 0;
}
