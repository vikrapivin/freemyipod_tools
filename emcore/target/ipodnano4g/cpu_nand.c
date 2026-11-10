/* SPDX-License-Identifier: GPL-2.0-or-later
 * CPU-direct NAND operations for the nano4g M2 campaign (see cpu_nand.h).
 *
 * Register sequences, as the source programs issue them:
 *   erase   target/s5l87xx/fmss/erase.s: 60h, 3 row cycles, D0h, status
 *   program nand/fmss/program.s, sequences.write_page(raw): 80h, 5 cycles,
 *           per 512 bytes DMA RAM->buffer (0x2e0) then data out (0xe4), 10h
 *   read    sequences.read_page: 00h, 5 cycles, 30h, status, 00h, then
 *           per 512 bytes data in (0xe2) + DMA buffer->RAM (0x100), or the
 *           32-byte CPU FIFO transfer of nand-nano4g.c nand_fifo_read
 *   decode  read.s / boot_read.s: parity in (0x32), data in (0xe2),
 *           ECC_CTRL kick, ECC_STATUS bit 2, result words, DMA out
 *   encode  sequences.ecc: DMA in, metadata 0x060..0x068 + 0x040 = 0xc1,
 *           ECC_CTRL kick, ECC_STATUS bit 1, parity 0x840..
 * Each WAIT N of a program becomes a bounded CPU poll of the flag the book
 * gives for it (FMC_STATUS bits 1, 2, 3, 20, 23; ECC_STATUS bits 1/2;
 * WAIT12 = latched status byte 0x04c bit 6). The flag is then acknowledged
 * exactly as the program does.
 */
#include "global.h"
#include "timer.h"
#include "nand_hal_emcore.h"
#include "nand_poc.h"
#include "cpu_nand.h"
#include "fmss_batch.h"
#include "cpu_nand_ll.h"
#include "bch_sw.h"
#include "crc32.h"
#include "nand/fmss-controller.h"
#include "nand/nand-target.h"

#define R CNLL_REG
#define CTRL0_BASE   CNLL_CTRL0_BASE
#define CTRL0_DMA    CNLL_CTRL0_DMA
#define T_ECC_US     CNLL_T_ECC_US

int lcd_wait_write_dma(void);

static struct cpu_nand_out *o;
static struct cnll L;

static uint32_t now(void) { return cnll_now(&L); }

static void note(uint32_t kind, uint32_t index, uint32_t argument, uint32_t value, int timeout)
{
    cnll_note(&L, kind, index, argument, value, timeout);
}

static int poll(uint32_t reg, uint32_t bit, uint32_t ack, uint32_t us, uint32_t kind, uint32_t index, uint32_t argument)
{
    return cnll_poll(&L, reg, bit, ack, us, kind, index, argument);
}

static int transfer(uint32_t n, uint32_t sel, uint32_t op, uint32_t i) { return cnll_transfer(&L, n, sel, op, i); }
static int dma(uint32_t addr, uint32_t sel, int into, uint32_t i) { return cnll_dma(&L, addr, sel, into, i); }
static void setup(uint32_t ce, uint32_t ctrl0) { cnll_setup(&L, ce, ctrl0); }

static int ecc_wait(struct cpu_nand_ecc *e, uint32_t bit, uint32_t i)
{
    uint32_t k;
    int rc = poll(0x810, bit, 0, T_ECC_US, CPU_NAND_K_ECC, i, bit);
    e->t_us = now();
    e->r810 = R(0x810);
    e->r818 = R(0x818);
    for (k = 0; k < 8; k++) e->loc[k] = bit == 4 ? R(0x820 + 4 * k) : 0;
    for (k = 0; k < 7; k++) e->par[k] = bit == 2 ? R(0x840 + 4 * k) : 0;
    e->r80c = R(0x80c);      /* one ring read per operation, after 0x81c reset */
    R(0x810) = 1;
    e->r810_ack = R(0x810);
    return rc;
}

static int op_read_ecc(uint32_t ce, uint32_t row, uint8_t *dst, uint32_t fmt)
{
    uint32_t plen = fmt == CPU_NAND_FMT_MAIN8 ? 13 : 26, sectors = fmt == CPU_NAND_FMT_ROM ? 3 : 8;
    uint32_t bch = fmt == CPU_NAND_FMT_MAIN8 ? 0 : 4, s, k;
    setup(ce, CTRL0_BASE | CTRL0_DMA);
    if (cnll_open_page(&L, row)) return -1;
    g_hal.cache_wb_inv(dst, 0x1000);
    for (s = 0; s < sectors; s++) {
        uint32_t meta = fmt != CPU_NAND_FMT_ROM && s == 0 ? 10 : 0;
        struct cpu_nand_ecc *e = &o->ecc[s];
        if (transfer(plen + meta, 0x10, 0x32, s) || transfer(512, 1, 0xe2, s)) return -1;
        R(0x81c) = 1;                   /* ring and totals at slot 0: one 0x80c read per decode */
        R(0x810) = 0x1ff;
        R(0x01c) = 0;
        if (fmt == CPU_NAND_FMT_ROM) R(0x808) = 0x80;
        R(0x804) = (meta << 8) | bch | 1;
        if (ecc_wait(e, 4, s)) return -2;
        o->nsectors = s + 1;
        if (dma((uint32_t)(dst + 512 * s), 0x100, 0, s)) return -1;
        if (meta) {
            uint32_t start = read_usec_timer(), v;
            R(0x040) = 0xc2;
            while ((v = R(0x040)) & 2)
                if (read_usec_timer() - start > T_ECC_US) { note(CPU_NAND_K_META, s, 0xc2, v, 1); return -3; }
            note(CPU_NAND_K_META, s, 0xc2, v, 0);
            for (k = 0; k < 3; k++) o->meta[k] = R(0x060 + 4 * k);
        }
    }
    n4g_emcore_cache_inv(dst, 0x1000);
    o->bytes = 512 * sectors;
    return 0;
}

static int op_encode(uint32_t slot, uint32_t kick, const uint8_t *src)
{
    const uint32_t *meta = (const uint32_t *)(src + CPU_NAND_IN_META);
    uint32_t k;
    setup(8, CTRL0_BASE | CTRL0_DMA);   /* buffer-only: no chip enable */
    R(0x038) = 7;
    if (dma((uint32_t)src, 1u << slot, 1, 0)) return -1;
    if (kick & 0xff00) {
        for (k = 0; k < 3; k++) R(0x060 + 4 * k) = meta[k];
        R(0x040) = 0xc1;
    }
    R(0x810) = 0x1ff;
    R(0x01c) = slot;
    R(0x804) = kick;
    if (ecc_wait(&o->ecc[0], 2, 0)) return -2;
    o->nsectors = 1;
    o->bytes = 512;
    return 0;
}

/* Reject malformed descriptors before issuing any NAND command. Byte offsets
 * address uploaded payload/output only; NAND opcodes and row-address bytes are
 * deliberately unrestricted. */
static int sequence_valid(const struct cpu_nand_seq *q)
{
    uint32_t i;
    if (q->magic != CPU_NAND_SEQ_MAGIC || !q->count || q->count > CPU_NAND_SEQ_MAX ||
        q->payload_bytes > CPU_NAND_SEQ_BYTES) return 0;
    for (i = 0; i < q->count; i++) {
        const struct cpu_nand_seq_insn *p = &q->insn[i];
        switch (p->kind) {
        case CPU_NAND_SEQ_CMD: if (p->a > 255 || p->b || p->c) return 0; break;
        case CPU_NAND_SEQ_ADDR: if (!p->a || p->a > 8) return 0; break;
        case CPU_NAND_SEQ_OUT:
            if (!p->a || p->b > q->payload_bytes || p->a > q->payload_bytes - p->b || p->c) return 0;
            break;
        case CPU_NAND_SEQ_IN:
            if (!p->a || p->b > CPU_NAND_SEQ_BYTES || p->a > CPU_NAND_SEQ_BYTES - p->b || p->c > 1) return 0;
            break;
        case CPU_NAND_SEQ_READY:
            if (!p->a || p->b > 0x100 || !p->c || p->c > 255) return 0;
            break;
        case CPU_NAND_SEQ_TIMING: case CPU_NAND_SEQ_DELAY:
            if (p->b || p->c) return 0;
            break;
        case CPU_NAND_SEQ_SELECT: if (p->a > 7 || p->b || p->c) return 0; break;
        default: return 0;
        }
    }
    return 1;
}

static int op_sequence(uint8_t *buf, uint32_t ce)
{
    const struct cpu_nand_seq *q = (const void *)(buf + CPU_NAND_IN);
    const uint8_t *payload = buf + CPU_NAND_IN + CPU_NAND_SEQ_PAYLOAD;
    uint8_t *tmp = buf + CPU_NAND_SEQ_TEMP, *dst = buf + CPU_NAND_SEQ_DATA;
    uint32_t i, pos, count, ready_left = 0;
    int rc = 0;
    for (i = 0; i < q->count; i++)
        if (q->insn[i].kind == CPU_NAND_SEQ_READY) ready_left++;
    setup(ce, q->ctrl0);
    R(0x038) = 7;
    for (i = 0; i < q->count; i++) {
        const struct cpu_nand_seq_insn *p = &q->insn[i];
        L.max_steps = CPU_NAND_MAX_STEPS;
        note(CPU_NAND_K_BEGIN, i, p->kind, p->a, 0);
        /* Reserve the remaining descriptor markers even when detailed DMA/FIFO
         * logs fill the record. Completion timestamps must not disappear. */
        L.max_steps = CPU_NAND_MAX_STEPS - (2 * (q->count - i) - 1 + ready_left);
        switch (p->kind) {
        case CPU_NAND_SEQ_CMD: rc = cnll_command(&L, p->a, i); break;
        case CPU_NAND_SEQ_ADDR: rc = cnll_address_bytes(&L, p->b, p->c, p->a, i); break;
        case CPU_NAND_SEQ_OUT:
            for (pos = 0; pos < p->a && !rc; pos += count) {
                count = p->a - pos < 512 ? p->a - pos : 512;
                memset(tmp, 0xff, 512);
                memcpy(tmp, payload + p->b + pos, count);
                g_hal.cache_wb_inv(tmp, 512);
                rc = dma((uint32_t)tmp, 1, 1, i);
                if (!rc) rc = transfer(count, 0x100, 0xe4, i);
            }
            break;
        case CPU_NAND_SEQ_IN:
            for (pos = 0; pos < p->a && !rc; pos += count) {
                count = p->a - pos < 512 ? p->a - pos : 512;
                rc = cnll_read_data(&L, tmp, count, p->c);
                if (!rc) {
                    memcpy(dst + p->b + pos, tmp, count);
                    if (o->bytes < p->b + pos + count) o->bytes = p->b + pos + count;
                }
            }
            break;
        case CPU_NAND_SEQ_READY: rc = cnll_ready_status(&L, i, p->a, p->b, p->c); break;
        case CPU_NAND_SEQ_TIMING:
            L.ctrl0 = p->a | (2u << ce);
            R(0x000) = L.ctrl0;
            break;
        case CPU_NAND_SEQ_DELAY:
            pos = read_usec_timer();
            while (read_usec_timer() - pos < p->a) ;
            break;
        case CPU_NAND_SEQ_SELECT:
            /* program.s interleave: start an operation on one CE, then another */
            ce = p->a;
            L.ctrl0 = (L.ctrl0 & ~0x1feu) | (2u << ce);
            R(0x000) = L.ctrl0;
            break;
        }
        L.max_steps = CPU_NAND_MAX_STEPS;
        if (p->kind == CPU_NAND_SEQ_READY) {
            /* A later status poll must not hide an earlier program/erase FAIL.
             * Preserve each status even after the detailed log fills. */
            note(CPU_NAND_K_STATUS, i, p->kind, L.nand_status, rc != 0);
            ready_left--;
        }
        note(CPU_NAND_K_DONE, i, p->kind, rc, rc != 0);
        o->unused = i + 1;
        if (rc) break;
    }
    L.bytes = 0; /* generic reads may use discontiguous output offsets */
    return rc;
}

static int all_ff(const uint8_t *p, uint32_t n)
{
    while (n--) if (*p++ != 0xff) return 0;
    return 1;
}

/* Exact parity agreement identifies on-flash formats without correcting any
 * bytes. A zero mask does not distinguish another format from damaged data. */
static uint32_t map_formats(const uint8_t *raw, uint32_t n)
{
    uint8_t msg[522], parity[26];
    uint32_t k, t, off, plen, mask = 0;
    for (k = 0; k < 3; k++) {
        off = 538 * k;
        bch_sw_encode(16, raw + off + 26, 512, parity);
        if (!memcmp(parity, raw + off, 26)) mask |= 1u << k;
    }
    for (t = 8; t <= 16; t += 8) {
        plen = t == 8 ? 13 : 26;
        for (k = 0; k < 8; k++) {
            off = 10 + k * (512 + plen);
            if (off + plen + 512 > n) break;
            memcpy(msg, raw + off + plen, 512);
            if (!k) memcpy(msg + 512, raw, 10);
            bch_sw_encode(t, msg, k ? 512 : 522, parity);
            if (!memcmp(parity, raw + off, plen)) mask |= 1u << (t + k);
        }
    }
    return mask;
}

static int op_map(uint8_t *buf, uint32_t ce, uint32_t row, uint32_t count)
{
    uint8_t *raw = buf + CPU_NAND_SEQ_TEMP;
    struct cpu_nand_map_page *p = (void *)(buf + CPU_NAND_SEQ_DATA);
    uint32_t n = nand_get_ecc_bits() == 16 ? 4096 + 218 : 4096 + 128, i;
    int rc;
    o->unused = n;
    if (bch_sw_init()) return -30;
    for (i = 0; i < count; i++) {
        memset(&p[i], 0, sizeof(p[i]));
        p[i].row = row + i;
        rc = cnll_read_raw(&L, ce, row + i, raw, n, 0);
        p[i].rc = rc;
        o->bytes = (i + 1) * sizeof(p[i]);
        if (rc) { L.bytes = 0; return rc; }
        p[i].flags = (all_ff(raw, n) ? 1 : 0) | (all_ff(raw, 4096) ? 2 : 0) |
                     (all_ff(raw + 4096, n - 4096) ? 4 : 0);
        p[i].crc = crc32(raw, n);
        memcpy(p[i].prefix, raw, 32);
        memcpy(p[i].marker, raw + 4096, 8);
        memcpy(p[i].tail, raw + n - 4, 4);
        if (!(p[i].flags & 1)) p[i].formats = map_formats(raw, n);
    }
    L.bytes = 0;
    return 0;
}

/* Release the status latch, deselect, acknowledge, then the ordinary reset. */
static void finish(void)
{
    cnll_release();
    o->cleanup[0] = R(0x000); o->cleanup[1] = R(0x004); o->cleanup[2] = R(0x048);
    o->post_reset_rc = n4g_fmss_reset();
    o->post[0] = R(0xc08); o->post[1] = R(0xc0c); o->post[2] = R(0x000); o->post[3] = R(0x004);
    o->post[4] = R(0x048); o->post[5] = R(0x810); o->post[6] = R(0x818);
}

int cpu_nand_job(uint8_t *buf, const uint32_t *arg, uint32_t *result, uint32_t *detail, uint32_t *len)
{
    const struct nand_geometry *g = nand_get_geometry();
    uint32_t op = arg[0] & 0xff, ce = (arg[0] >> 8) & 0xf, slot = (arg[0] >> 12) & 0xf;
    uint32_t row = arg[1], param = arg[2], n = param & 0xffff, path = param >> 16;
    uint8_t *in = buf + CPU_NAND_IN, *data = buf + CPU_NAND_DATA;
    int rc = 0;

    *result = NPOC_R_OK;
    *len = 0;
    if (op > CPU_NAND_MAP || (op == CPU_NAND_READ_RAW && (path > 1 || !n || n > CPU_NAND_MAX_RAW))
        || (op == CPU_NAND_PROGRAM && (!n || n > CPU_NAND_MAX_RAW))
        || (op == CPU_NAND_READ_ECC && param > CPU_NAND_FMT_ROM)
        || (op == CPU_NAND_ENCODE && (slot > 1 || (param & 3) != 2))
        || (op == CPU_NAND_SEQUENCE && !sequence_valid((const void *)in))
        || (op == CPU_NAND_MAP && (!param || param > 256))) {
        *result = NPOC_R_BAD_ARG;
        return 0;
    }
    if (op != CPU_NAND_STATE && op != CPU_NAND_ENCODE &&
        (!g || ce >= g->banks || row >= g->blocks * g->pagesperblock ||
         (op == CPU_NAND_MAP && param > g->blocks * g->pagesperblock - row))) {
        *result = NPOC_R_BAD_ARG;
        return 0;
    }
    o = (struct cpu_nand_out *)buf;
    memset(o, 0, CPU_NAND_DATA);
    memset(data, 0xee, 0x1200);
    if (op == CPU_NAND_SEQUENCE || op == CPU_NAND_MAP)
        memset(buf + CPU_NAND_SEQ_DATA, 0xee, CPU_NAND_SEQ_BYTES);
    o->magic = CPU_NAND_MAGIC; o->version = CPU_NAND_VERSION;
    o->op = op; o->ce = ce; o->row = row; o->param = param;
    if (op == CPU_NAND_PROGRAM || op == CPU_NAND_ENCODE) g_hal.cache_wb_inv(in, CPU_NAND_IN_BYTES);
    if (lcd_wait_write_dma()) { *result = NPOC_R_LIB_ERROR; return -3; }
    cnll_start(&L, o->step, CPU_NAND_MAX_STEPS);
    o->pre[0] = R(0xc08); o->pre[1] = R(0x048); o->pre[2] = R(0x810);
    /* The drivers' rule: reset the engine before any controller use. */
    o->pre_reset_rc = n4g_fmss_reset();
    if (o->pre_reset_rc) rc = -20;
    else if (op == CPU_NAND_ERASE) rc = cnll_erase(&L, ce, row);
    else if (op == CPU_NAND_PROGRAM) rc = cnll_program(&L, ce, row, in, n);
    else if (op == CPU_NAND_READ_RAW) rc = cnll_read_raw(&L, ce, row, data, n, path);
    else if (op == CPU_NAND_READ_ECC) rc = op_read_ecc(ce, row, data, param);
    else if (op == CPU_NAND_ENCODE) rc = op_encode(slot, param, in);
    else if (op == CPU_NAND_SEQUENCE) rc = op_sequence(buf, ce);
    else if (op == CPU_NAND_MAP) rc = op_map(buf, ce, row, param);
    o->elapsed_us = now();
    o->nsteps = L.nsteps; o->dropped = L.dropped;
    o->ctrl0 = L.ctrl0; o->nand_status = L.nand_status;
    if (L.bytes) o->bytes = L.bytes;
    finish();
    o->rc = (uint32_t)rc;
    if (op == CPU_NAND_SEQUENCE || op == CPU_NAND_MAP) {
        /* All descriptors have been consumed: compact the staged output for USB. */
        memmove(data, buf + CPU_NAND_SEQ_DATA, o->bytes);
        *len = CPU_NAND_DATA + o->bytes;
    } else *len = CPU_NAND_DATA + (o->bytes ? 0x1200 : 0);
    return rc;
}

/* Resident CPU sequences/maps share the single worker and its stable DMA work
 * allocation with probes. Validate the entire manifest before touching NAND. */
struct cpu_batch_case { uint32_t id, arg[3], upload, bytes, reserved[2]; };
struct cpu_batch_record { uint32_t id, result, rc, head, data, detail[2], reserved; };
#define CPU_ALIGN4(n) (((n) + 3u) & ~3u)
static int cpu_batch_range(uint32_t off, uint32_t n, uint32_t cap)
{ return off <= cap && n <= cap - off; }

static uint32_t cpu_batch_data(const struct fmss_batch_run *b, const struct cpu_batch_case *d)
{
    uint32_t i, n = 0;
    const struct cpu_nand_seq *q = (const void *)(b->input + d->upload);
    if ((d->arg[0] & 255) == CPU_NAND_MAP) return d->arg[2] * sizeof(struct cpu_nand_map_page);
    for (i = 0; i < q->count; i++)
        if (q->insn[i].kind == CPU_NAND_SEQ_IN && q->insn[i].a + q->insn[i].b > n)
            n = q->insn[i].a + q->insn[i].b;
    return n;
}

int cpu_nand_batch_run(const struct fmss_batch_run *b)
{
    const struct fmss_batch_header *h = (const void *)b->input;
    struct fmss_batch_output *out = (void *)b->output;
    const struct cpu_batch_case *d;
    const struct nand_geometry *g = nand_get_geometry();
    uint32_t i, started = read_usec_timer();
    if (b->input_bytes < sizeof(*h) || b->input_bytes > FMSS_BATCH_INPUT_MAX ||
        b->output_bytes < sizeof(*out) || b->output_bytes > FMSS_BATCH_OUTPUT_MAX ||
        ((uintptr_t)b->input & 3) || ((uintptr_t)b->output & 3) || ((uintptr_t)b->work & 31)) return -1;
    if (h->magic != CPU_NAND_BATCH_MAGIC || h->version != 1 || !h->count ||
        h->count > FMSS_BATCH_COUNT_MAX || h->bytes != b->input_bytes || b->first >= h->count ||
        h->table < sizeof(*h) || (h->table & 3) || h->reserved[0] || h->reserved[1] || h->reserved[2] ||
        !cpu_batch_range(h->table, h->count * sizeof(*d), b->input_bytes)) return -1;
    d = (const void *)(b->input + h->table);
    for (i = 0; i < h->count; i++) {
        uint32_t op = d[i].arg[0] & 255, ce = d[i].arg[0] >> 8;
        uint32_t row = d[i].arg[1], param = d[i].arg[2];
        if (d[i].id != i || d[i].reserved[0] || d[i].reserved[1] || (d[i].upload & 3) ||
            !cpu_batch_range(d[i].upload, d[i].bytes, b->input_bytes) ||
            !g || ce >= g->banks || row >= g->blocks * g->pagesperblock) return -1;
        if (op == CPU_NAND_SEQUENCE) {
            const struct cpu_nand_seq *q = (const void *)(b->input + d[i].upload);
            if (param || d[i].bytes < CPU_NAND_SEQ_PAYLOAD ||
                !sequence_valid(q) || d[i].bytes != CPU_NAND_SEQ_PAYLOAD + q->payload_bytes) return -1;
        } else if (op == CPU_NAND_MAP) {
            if (d[i].bytes || !param || param > 256 || param > g->blocks * g->pagesperblock - row) return -1;
        } else return -1;
        if (CPU_ALIGN4(sizeof(struct cpu_batch_record) + sizeof(struct cpu_nand_out) + cpu_batch_data(b, &d[i])) >
            b->output_bytes - sizeof(*out)) return -1;
    }
    memset(out, 0, sizeof(*out));
    out->magic = CPU_NAND_BATCH_OUTPUT; out->version = 1; out->total = h->count;
    out->first = out->next = b->first; out->bytes = sizeof(*out);
    g_hal.cache_wb(out, sizeof(*out));
    for (i = b->first; i < h->count; i++) {
        struct cpu_batch_record *r;
        struct cpu_nand_out *capture = (void *)b->work;
        uint32_t result = NPOC_R_OK, detail[2] = {0, 0}, len = 0, size;
        int rc;
        size = CPU_ALIGN4(sizeof(*r) + sizeof(*capture) + cpu_batch_data(b, &d[i]));
        if (size > b->output_bytes - out->bytes) break; /* reserve BEFORE execution */
        rc = b->prepare ? b->prepare(i, FMSS_BATCH_PINS) : 0;
        if (!rc) {
            if (d[i].bytes) memcpy(b->work + CPU_NAND_IN, b->input + d[i].upload, d[i].bytes);
            rc = cpu_nand_job(b->work, d[i].arg, &result, detail, &len);
        }
        if (rc && !result) result = NPOC_R_LIB_ERROR;
        r = (void *)(b->output + out->bytes);
        memset(r, 0, sizeof(*r));
        r->id = i; r->result = result; r->rc = rc;
        r->detail[0] = detail[0]; r->detail[1] = detail[1];
        if (len) {
            r->head = __builtin_offsetof(struct cpu_nand_out, step) + capture->nsteps * sizeof(struct cpu_nand_step);
            r->data = capture->bytes;
            memcpy(r + 1, capture, r->head);
            memcpy((uint8_t *)(r + 1) + r->head, b->work + CPU_NAND_DATA, r->data);
        }
        size = CPU_ALIGN4(sizeof(*r) + r->head + r->data);
        memset((uint8_t *)r + sizeof(*r) + r->head + r->data, 0, size - sizeof(*r) - r->head - r->data);
        g_hal.cache_wb(r, size);
        out->bytes += size; out->next = i + 1;
        if (result || rc) out->errors++;
        g_hal.cache_wb(out, sizeof(*out));
    }
    out->elapsed_us = read_usec_timer() - started;
    g_hal.cache_wb(out, out->bytes);
    return 0;
}
