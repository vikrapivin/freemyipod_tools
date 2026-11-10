/* SPDX-License-Identifier: GPL-2.0-or-later
 * CPU-only NAND driver (see cpu_nand_stack.h). No FMSS sequencer program runs.
 *
 * Hardware ECC read follows read.s's two-buffer overlap, with the CPU
 * opcodes (DATA_SEL + FMCTRL1 0x32/0xe2). Sector 0 is loaded and waited.
 * Each later sector's parity is waited into the other buffer; DATA_SEL is
 * then set once to that buffer's data bit (0/1) OR the current buffer's
 * DMA-out bit (8/9), the DMA address is stored, and 0xe2 starts the 512-byte
 * read. ECC runs on the current slot (0x81c = 1, 0x810 = 0x1ff, 0x01c =
 * slot, boot 0x808 = 0x80, ECC_CTRL = metalen << 8 | BCH16 << 2 | 1). One
 * 0x80c read, then acknowledge 1, then FMCTRL1 0x100. DATA_SEL and DNUM are
 * not written again until that read's bit 3 is acknowledged. The sector-0
 * metadata latch (0x040 = 0xc2 -> 0x060..0x068) sits in that window.
 * Hardware program follows program.s: DMA into the sector buffer, encode
 * (ECC_CTRL = metalen << 8 | BCH16 << 2 | 2, ECC_STATUS bit 1), stream the
 * parity buffer (sector 0 includes the ten metadata bytes) then the data.
 * A non-NULL raw buffer is filled from 0x840 for the diagnostic image.
 * Software ECC: raw page read (DMA or FIFO path), bch_sw per sector.
 */
#include "global.h"
#include "timer.h"
#include "nand_hal_emcore.h"
#include "nand_poc.h"
#include "bch_sw.h"
#include "cpu_nand_stack.h"
#include "nand/fmss-controller.h"
#include "nand/nand-target.h"
#include "nand/nand-profile.h"

#define R CNLL_REG
#define META_BYTES 10u
#define BOOT_STRIDE 538u

int lcd_wait_write_dma(void);

typedef char cns_out_fits[(sizeof(struct cns_out) <= CNS_STEPS) ? 1 : -1];
typedef char cns_rep_at_a0[(__builtin_offsetof(struct cns_out, rep) == 0xa0 &&
                            __builtin_offsetof(struct cns_out, rep.sector) == 0xc4 &&
                            sizeof(struct cnand_sector) == 52) ? 1 : -1];   /* tools/nano4g/nand.py parse_cpu_stack */

uint32_t cnand_strength(const struct cnand *n)
{
    if (!nand_get_geometry()) return 0;
    if (n->strength) return n->strength;
    return nand_get_ecc_bits() == 16 ? 16 : 8;
}

uint32_t cnand_rawsize(const struct cnand *n)
{
    uint32_t t = cnand_strength(n);
    return t ? 4096u + (t == 16 ? 218u : 128u) : 0;
}

uint32_t cnand_datasize(uint32_t fmt) { return fmt == CNAND_FMT_BOOT ? 0x600u : 4096u; }

static uint32_t sectors(uint32_t fmt) { return fmt == CNAND_FMT_BOOT ? 3 : 8; }
static uint32_t plen(const struct cnand *n, uint32_t fmt)
{
    return fmt == CNAND_FMT_BOOT || cnand_strength(n) == 16 ? 26 : 13;
}
static uint32_t bch_t(const struct cnand *n, uint32_t fmt) { return plen(n, fmt) == 26 ? 16 : 8; }

/* Raw offset of sector s's parity; its data follows; main sector 0's metadata is at 0. */
static uint32_t par_off(const struct cnand *n, uint32_t fmt, uint32_t s)
{
    return fmt == CNAND_FMT_BOOT ? s * BOOT_STRIDE : META_BYTES + s * (plen(n, fmt) + 512);
}

void cnand_init(struct cnand *n, struct cpu_nand_step *steps, uint32_t max_steps, uint32_t strength)
{
    memset(n, 0, sizeof(*n));
    n->strength = strength == 8 || strength == 16 ? strength : 0;
    n->read_ctrl0 = CNLL_CTRL0_BASE | CNLL_CTRL0_DMA;
    n->ll.step = steps;
    n->ll.max_steps = steps ? max_steps : 0;
}

/* The drivers' rule: reset the engine before any controller use. */
static int begin(struct cnand *n)
{
    struct cpu_nand_step *steps = n->ll.step;
    uint32_t max = n->ll.max_steps;
    unsigned int previous = nand_profile_enter(NAND_PROFILE_SETUP);
    if (lcd_wait_write_dma()) {
        nand_profile_leave(previous);
        return CNAND_EDMA;
    }
    cnll_start(&n->ll, steps, max);
    n->ll.fast = n->optimized;
    if (!n->optimized) {
        n->pre[0] = R(0xc08); n->pre[1] = R(0x048); n->pre[2] = R(0x810);
    }
    n->pre_reset_rc = n4g_fmss_reset();
    nand_profile_leave(previous);
    return n->pre_reset_rc ? CNAND_ERESET : 0;
}

/* Release the status latch, deselect and acknowledge. Diagnostic jobs then
 * take the ordinary reset and record the idle image. Shared calls set
 * optimized: the next begin() resets, and this second reset is skipped.
 * post_reset_rc stays 0 from cnand_init. */
static int end(struct cnand *n, int rc)
{
    unsigned int previous = nand_profile_enter(NAND_PROFILE_CLEANUP);
    if (!n->optimized) n->elapsed_us = cnll_now(&n->ll);
    cnll_release();
    if (!n->optimized) {
        n->cleanup[0] = R(0x000); n->cleanup[1] = R(0x004); n->cleanup[2] = R(0x048);
        n->post_reset_rc = n4g_fmss_reset();
        n->post[0] = R(0xc08); n->post[1] = R(0xc0c); n->post[2] = R(0x000); n->post[3] = R(0x004);
        n->post[4] = R(0x048); n->post[5] = R(0x810); n->post[6] = R(0x818);
    }
    nand_profile_leave(previous);
    return rc;
}

static int ll_rc(int rc)
{
    return rc == -1 ? CNAND_ESTEP : rc == -2 ? CNAND_EREADY : rc == -10 ? CNAND_ESTATUS : rc;
}

static void summarize(struct cnand_report *rep)
{
    uint32_t s;
    rep->corrected_total = rep->max_corrected = rep->uncorrectable_mask = rep->erased_mask = 0;
    for (s = 0; s < rep->nsectors; s++) {
        const struct cnand_sector *e = &rep->sector[s];
        if (e->flags & CNAND_S_UNCORR) rep->uncorrectable_mask |= 1u << s;
        if (e->flags & CNAND_S_ERASED) rep->erased_mask |= 1u << s;
        rep->corrected_total += e->corrected;
        if (e->corrected > rep->max_corrected) rep->max_corrected = e->corrected;
    }
}

static void meta_words(uint32_t *w, const uint8_t *m)
{
    w[0] = m[0] | m[1] << 8 | m[2] << 16 | (uint32_t)m[3] << 24;
    w[1] = m[4] | m[5] << 8 | m[6] << 16 | (uint32_t)m[7] << 24;
    w[2] = m[8] | m[9] << 8 | 0xffff0000u;
}

int cnand_read_raw(struct cnand *n, uint32_t ce, uint32_t row, uint8_t *raw, uint32_t len, uint32_t path)
{
    int rc;
    if (!len || len > CNAND_RAW_BUF || path > CNAND_PATH_FIFO) return CNAND_EARG;
    rc = begin(n);
    if (!rc) rc = ll_rc(cnll_read_raw(&n->ll, ce, row, raw, len, path));
    return end(n, rc);
}

/* ---------------------------------------------------------------- hardware ECC */
/* 0x840 word k holds bits 32k.. of the big-endian parity. */
static void store_parity(uint8_t *dst, uint32_t p)
{
    uint32_t j, par[7];
    for (j = 0; j < 7; j++) par[j] = R(0x840 + 4 * j);
    for (j = 0; j < p; j++) {
        uint32_t bit = 8 * (p - 1 - j);
        dst[j] = (uint8_t)(par[bit / 32] >> (bit % 32));
    }
}

static int latch_meta(struct cnll *c, uint32_t s, uint32_t *meta)
{
    uint32_t start = read_usec_timer(), v, k;
    R(0x040) = 0xc2;
    while ((v = R(0x040)) & 2)
        if (read_usec_timer() - start > CNLL_T_ECC_US) {
            cnll_note(c, CPU_NAND_K_META, s, 0xc2, v, 1);
            return CNAND_EECC;
        }
    cnll_note(c, CPU_NAND_K_META, s, 0xc2, v, 0);
    for (k = 0; k < 3; k++) meta[k] = R(0x060 + 4 * k);
    return 0;
}

static void select_ce(struct cnll *c, uint32_t ce)
{
    c->ctrl0 = (c->ctrl0 & ~0x1feu) | (2u << ce);
    R(0x000) = c->ctrl0;
}

/* 00h, five address cycles, 30h. Returns while the chip's array read runs. */
static int issue_array_read(struct cnll *c, uint32_t ce, uint32_t row)
{
    select_ce(c, ce);
    if (cnll_command(c, 0x00, 0) || cnll_address(c, row, 5, 0) || cnll_command(c, 0x30, 1))
        return CNAND_ESTEP;
    return 0;
}

/* The array read was already started. Wait until the page register is ready
 * and switch the chip back to data output. */
static int begin_data_out(struct cnll *c, uint32_t ce)
{
    select_ce(c, ce);
    if (cnll_ready(c, 2, CNLL_T_READY_US) || cnll_command(c, 0x00, 3))
        return CNAND_ESTEP;
    return 0;
}

/* opened: 30h has already been issued for this page. prefetch_row, when
 * prefetch is set, is the next row on the same chip; it is issued after this
 * page's last NAND byte is in a sector buffer and before that sector's ECC. */
static int hw_decode(struct cnand *n, uint32_t ce, uint32_t row, uint32_t fmt, uint8_t *data,
                     struct cnand_report *rep, int opened, uint32_t prefetch_row, int prefetch,
                     int *did_prefetch)
{
    struct cnll *c = &n->ll;
    uint32_t p = plen(n, fmt), bch = p == 26 ? 4 : 0, ns = sectors(fmt), s, k;
    uint32_t meta0 = fmt == CNAND_FMT_MAIN ? META_BYTES : 0;
    if (!opened) {
        cnll_setup(c, ce, n->read_ctrl0);
        if (cnll_open_page(c, row)) return CNAND_ESTEP;
    } else if (begin_data_out(c, ce))
        return CNAND_ESTEP;
    g_hal.cache_wb_inv(data, 512 * ns);
    /* Sector 0 into slot 0, fully waited, so the first ECC has a stable buffer. */
    if (cnll_transfer(c, p + meta0, 0x10, 0x32, 0) || cnll_transfer(c, 512, 1, 0xe2, 0)) return CNAND_ESTEP;
    for (s = 0; s < ns; s++) {
        uint32_t slot = s & 1, other = slot ^ 1;
        uint32_t meta = fmt == CNAND_FMT_MAIN && s == 0 ? META_BYTES : 0;
        struct cnand_sector *e = &rep->sector[s];
        bool pending = false;
        if (s + 1 < ns) {
            if (cnll_transfer(c, p, 0x10u << other, 0x32, s + 1)) return CNAND_ESTEP;
            /* One DATA_SEL for the in-flight read and the DMA that follows it.
             * Bits 0/1 name the NAND destination; bits 8/9 name the DMA source.
             * 0x100 must not write DATA_SEL or DNUM again. */
            R(0x034) = (uint32_t)(data + 512 * s);
            cnll_transfer_start(512, (1u << other) | (0x100u << slot), 0xe2);
            pending = true;
        }
        R(0x81c) = 1;
        R(0x810) = 0x1ff;
        R(0x01c) = slot;
        if (fmt == CNAND_FMT_BOOT) R(0x808) = 0x80;
        R(0x804) = (meta << 8) | bch | 1;
        e->flags = CNAND_S_HW;
        if (cnll_poll(c, 0x810, 4, 0, CNLL_T_ECC_US, CPU_NAND_K_ECC, s, 4)) return CNAND_EECC;
        if (!n->optimized) e->t_us = cnll_now(c);
        e->r810 = R(0x810);
        if (!n->optimized) e->r818 = R(0x818);
        e->r80c = R(0x80c);                         /* exactly one ring read per decode */
        if (e->r810 & 0x40) e->flags |= CNAND_S_ERASED;
        else if (e->r80c & 1) e->flags |= CNAND_S_UNCORR;
        else {
            e->flags |= CNAND_S_OK;
            e->corrected = (e->r80c >> 16) & 0x1f;
            if (!n->optimized)
                for (k = 0; k < (e->corrected * 13 + 31) / 32 && k < 7; k++) e->loc[k] = R(0x820 + 4 * k);
        }
        R(0x810) = 1;                               /* before any DMA into this slot */
        rep->nsectors = s + 1;
        if (pending) {
            cnll_dma_out_start();
            if (meta && latch_meta(c, s, rep->meta)) return CNAND_EECC;
            if (cnll_dma_out_wait(c, s) || cnll_transfer_wait(c, s + 1, 0xe2)) return CNAND_ESTEP;
            /* The chip has clocked out every byte of this page. Start its next
             * array read now; the last sector's ECC and DMA do not need it. */
            if (prefetch && s + 1 == ns - 1) {
                if (issue_array_read(c, ce, prefetch_row)) return CNAND_ESTEP;
                if (did_prefetch) *did_prefetch = 1;
            }
        } else {
            if (cnll_dma(c, (uint32_t)(data + 512 * s), 0x100u << slot, 0, s)) return CNAND_ESTEP;
            if (meta && latch_meta(c, s, rep->meta)) return CNAND_EECC;
        }
    }
    n4g_emcore_cache_inv(data, 512 * ns);
    return 0;
}

static int hw_encode(struct cnand *n, uint32_t fmt, const uint8_t *data, const uint8_t *meta, uint8_t *raw,
                     struct cnand_report *rep)
{
    struct cnll *c = &n->ll;
    uint32_t p = plen(n, fmt), bch = p == 26 ? 4 : 0, s, j, w[3];
    g_hal.cache_wb_inv((void *)data, 512 * sectors(fmt));
    cnll_setup(c, CNLL_NO_CE, CNLL_CTRL0_BASE | CNLL_CTRL0_DMA);   /* buffer-only: no chip enable */
    R(0x038) = 7;
    for (s = 0; s < sectors(fmt); s++) {
        uint32_t ml = fmt == CNAND_FMT_MAIN && s == 0 ? META_BYTES : 0;
        struct cnand_sector *e = &rep->sector[s];
        if (cnll_dma(c, (uint32_t)(data + 512 * s), 1, 1, s)) return CNAND_ESTEP;
        if (ml) {
            meta_words(w, meta);
            for (j = 0; j < 3; j++) R(0x060 + 4 * j) = w[j];
            R(0x040) = 0xc1;
        }
        R(0x810) = 0x1ff;
        R(0x01c) = 0;
        R(0x804) = (ml << 8) | bch | 2;
        e->flags = CNAND_S_HW;
        if (cnll_poll(c, 0x810, 2, 0, CNLL_T_ECC_US, CPU_NAND_K_ECC, s, 2)) return CNAND_EECC;
        e->t_us = cnll_now(c);
        e->r810 = R(0x810);
        store_parity(raw + par_off(n, fmt, s), p);
        R(0x810) = 1;
        e->flags |= CNAND_S_OK;
        rep->nsectors = s + 1;
    }
    return 0;
}

/* program.s, one sector at a time: DMA into slot s&1, encode there, stream the
 * parity buffer (sector 0's buffer starts with the ten metadata bytes) then
 * the 512 data bytes. The page register is filled with 0xff by command 80h,
 * so the unread tail of a short page stays erased. */
static int hw_program(struct cnand *n, uint32_t ce, uint32_t row, uint32_t fmt,
                      const uint8_t *data, const uint8_t *meta, uint8_t *raw)
{
    struct cnll *c = &n->ll;
    uint32_t p = plen(n, fmt), bch = p == 26 ? 4 : 0, ns = sectors(fmt), s, j, w[3];
    uint32_t start, v;
    g_hal.cache_wb_inv((void *)data, 512 * ns);
    if (raw) {
        memset(raw, 0xff, CNAND_RAW_BUF);
        if (fmt == CNAND_FMT_MAIN) memcpy(raw, meta, META_BYTES);
        for (s = 0; s < ns; s++) memcpy(raw + par_off(n, fmt, s) + p, data + 512 * s, 512);
    }
    cnll_setup(c, ce, CNLL_CTRL0_BASE);
    if (cnll_command(c, 0x80, 0) || cnll_address(c, row, 5, 0)) return CNAND_ESTEP;
    if (fmt == CNAND_FMT_MAIN) {
        meta_words(w, meta);
        for (j = 0; j < 3; j++) R(0x060 + 4 * j) = w[j];
        R(0x040) = 0xc1;
        start = read_usec_timer();
        while ((v = R(0x040)) & 1)
            if (read_usec_timer() - start > CNLL_T_ECC_US) {
                cnll_note(c, CPU_NAND_K_META, 0, 0xc1, v, 1);
                return CNAND_EECC;
            }
        cnll_note(c, CPU_NAND_K_META, 0, 0xc1, v, 0);
    }
    c->ctrl0 |= CNLL_CTRL0_DMA;
    R(0x000) = c->ctrl0;
    R(0x038) = 7;
    R(0x028) = 0;
    for (s = 0; s < ns; s++) {
        uint32_t slot = s & 1;
        uint32_t ml = fmt == CNAND_FMT_MAIN && s == 0 ? META_BYTES : 0;
        if (cnll_dma(c, (uint32_t)(data + 512 * s), 1u << slot, 1, s)) return CNAND_ESTEP;
        R(0x810) = 0x1ff;
        R(0x01c) = slot;
        if (fmt == CNAND_FMT_BOOT) R(0x808) = 0x80;
        R(0x804) = (ml << 8) | bch | 2;
        if (cnll_poll(c, 0x810, 2, 0, CNLL_T_ECC_US, CPU_NAND_K_ECC, s, 2)) return CNAND_EECC;
        if (raw) store_parity(raw + par_off(n, fmt, s), p);
        R(0x810) = 1;
        /* Sector 0 streams meta||parity from the parity buffer. Do not also
         * write the metadata through the FIFO. */
        if (cnll_transfer(c, p + ml, 0x1000u << slot, 0x34, s) ||
            cnll_transfer(c, 512, 0x100u << slot, 0xe4, s))
            return CNAND_ESTEP;
    }
    if (cnll_command(c, 0x10, 1)) return CNAND_ESTEP;
    if (cnll_ready(c, 2, CNLL_T_READY_US)) return CNAND_EREADY;
    return c->nand_status & 1 ? CNAND_ESTATUS : 0;
}

/* ---------------------------------------------------------------- software ECC */
static int sw_encode(const struct cnand *n, uint32_t fmt, const uint8_t *data, const uint8_t *meta, uint8_t *raw, struct cnand_report *rep)
{
    uint8_t msg[512 + META_BYTES];
    uint32_t s, start = read_usec_timer();
    if (bch_sw_init()) return CNAND_ENOMEM;
    for (s = 0; s < sectors(fmt); s++) {
        uint32_t ml = fmt == CNAND_FMT_MAIN && s == 0 ? META_BYTES : 0;
        memcpy(msg, data + 512 * s, 512);
        if (ml) memcpy(msg + 512, meta, ml);
        if (bch_sw_encode(bch_t(n, fmt), msg, 512 + ml, raw + par_off(n, fmt, s))) return CNAND_EARG;
        rep->sector[s].flags = CNAND_S_SW | CNAND_S_OK;
        rep->nsectors = s + 1;
    }
    rep->sw_us = read_usec_timer() - start;
    return 0;
}

static int sw_decode(const struct cnand *n, uint32_t fmt, const uint8_t *raw, uint8_t *data, struct cnand_report *rep)
{
    uint8_t msg[512 + META_BYTES], par[26];
    uint16_t loc[BCH_SW_MAX_T];
    uint32_t p = plen(n, fmt), s, k, start = read_usec_timer();
    if (bch_sw_init()) return CNAND_ENOMEM;
    for (s = 0; s < sectors(fmt); s++) {
        uint32_t ml = fmt == CNAND_FMT_MAIN && s == 0 ? META_BYTES : 0, po = par_off(n, fmt, s), erased = 1;
        struct cnand_sector *e = &rep->sector[s];
        int got;
        memcpy(msg, raw + po + p, 512);
        if (ml) memcpy(msg + 512, raw, ml);
        memcpy(par, raw + po, p);
        for (k = 0; k < 512 + ml && erased; k++) erased = msg[k] == 0xff;
        for (k = 0; k < p && erased; k++) erased = par[k] == 0xff;
        e->flags = CNAND_S_SW;
        if (erased) e->flags |= CNAND_S_ERASED;
        else {
            memset(loc, 0, sizeof(loc));
            got = bch_sw_decode(bch_t(n, fmt), msg, 512 + ml, par, loc);
            if (got == BCH_SW_EARG) return CNAND_EARG;
            if (got < 0) e->flags |= CNAND_S_UNCORR;
            else {
                e->flags |= CNAND_S_OK;
                e->corrected = (uint32_t)got;
                for (k = 0; k < (uint32_t)got; k++) {
                    uint32_t bit = 13 * k;
                    e->loc[bit / 32] |= (uint32_t)loc[k] << (bit % 32);
                    if (bit % 32 > 19 && bit / 32 + 1 < 7) e->loc[bit / 32 + 1] |= (uint32_t)loc[k] >> (32 - bit % 32);
                }
            }
        }
        memcpy(data + 512 * s, msg, 512);
        if (ml) {
            uint8_t m[12];
            memcpy(m, msg + 512, ml);
            m[10] = m[11] = 0xff;
            memcpy(rep->meta, m, 12);
        }
        rep->nsectors = s + 1;
    }
    rep->sw_us = read_usec_timer() - start;
    return 0;
}

/* ---------------------------------------------------------------- public operations */
int cnand_read_page(struct cnand *n, uint32_t ce, uint32_t row, uint32_t fmt, uint32_t ecc, uint32_t path,
                    uint8_t *data, uint8_t *raw, struct cnand_report *rep)
{
    uint32_t size = cnand_rawsize(n);
    int rc;
    if (fmt > CNAND_FMT_BOOT || (ecc != CNAND_ECC_HW && ecc != CNAND_ECC_SW) || path > CNAND_PATH_FIFO ||
        (ecc == CNAND_ECC_HW && path) || !size)
        return CNAND_EARG;
    memset(rep, 0, sizeof(*rep));
    if (ecc == CNAND_ECC_SW && bch_sw_init()) return CNAND_ENOMEM;
    rc = begin(n);
    if (!rc) {
        if (ecc == CNAND_ECC_HW) rc = hw_decode(n, ce, row, fmt, data, rep, 0, 0, 0, NULL);
        else rc = ll_rc(cnll_read_raw(&n->ll, ce, row, raw, size, path));
    }
    rc = end(n, rc);
    /* Software decode after the controller is idle: no register access. */
    if (!rc && ecc == CNAND_ECC_SW) rc = sw_decode(n, fmt, raw, data, rep);
    summarize(rep);
    return rc;
}

static int next_same_ce(const struct cnand_main_page *pages, uint32_t count, uint32_t j,
                        const uint8_t *issued)
{
    uint32_t k;
    for (k = j + 1; k < count; k++)
        if (pages[k].ce == pages[j].ce && !issued[k])
            return (int)k;
    return -1;
}

int cnand_read_main_pages(struct cnand *n, const struct cnand_main_page *pages, uint32_t count,
                          cnand_page_done done, void *ctx)
{
    const struct nand_geometry *g = nand_get_geometry();
    struct cnll *c = &n->ll;
    struct cnand_report rep;
    uint8_t issued[CNAND_READ_SPAN];
    uint32_t i, first;
    int rc;

    if (!g || !count || count > CNAND_READ_SPAN || !pages) return CNAND_EARG;
    for (i = 0; i < count; i++)
        if (pages[i].ce >= g->banks || !pages[i].data
            || pages[i].row >= g->blocks * g->pagesperblock)
            return CNAND_EARG;
    rc = begin(n);
    if (rc) return end(n, rc);
    memset(issued, 0, count);
    cnll_setup(c, pages[0].ce, n->read_ctrl0);
    for (first = 0; first < count && !rc; ) {
        uint32_t used = 0, end = first, j;
        /* One page per chip: start every array read before decoding any of them. */
        while (end < count && !(used & (1u << pages[end].ce))) {
            used |= 1u << pages[end].ce;
            end++;
        }
        for (j = first; j < end && !rc; j++) {
            if (issued[j]) continue;
            rc = issue_array_read(c, pages[j].ce, pages[j].row);
            if (!rc) issued[j] = 1;
        }
        for (j = first; j < end && !rc; j++) {
            int next = next_same_ce(pages, count, j, issued);
            int did = 0;
            memset(&rep, 0, sizeof(rep));
            rc = hw_decode(n, pages[j].ce, pages[j].row, CNAND_FMT_MAIN, pages[j].data, &rep,
                           1, next >= 0 ? pages[next].row : 0, next >= 0, &did);
            if (did) issued[next] = 1;
            if (rc) break;
            summarize(&rep);
            if (pages[j].meta) memcpy(pages[j].meta, rep.meta, 12);
            if (done && done(ctx, j, &rep)) rc = CNAND_EARG;
        }
        first = end;
    }
    return end(n, rc);
}

static int build(struct cnand *n, uint32_t fmt, uint32_t ecc, const uint8_t *data, const uint8_t *meta,
                 uint8_t *raw, struct cnand_report *rep)
{
    uint32_t s, p = plen(n, fmt);
    memset(raw, 0xff, CNAND_RAW_BUF);
    if (fmt == CNAND_FMT_MAIN) memcpy(raw, meta, META_BYTES);
    for (s = 0; s < sectors(fmt); s++) memcpy(raw + par_off(n, fmt, s) + p, data + 512 * s, 512);
    return ecc == CNAND_ECC_HW ? hw_encode(n, fmt, data, meta, raw, rep) : sw_encode(n, fmt, data, meta, raw, rep);
}

int cnand_build_page(struct cnand *n, uint32_t fmt, uint32_t ecc, const uint8_t *data, const uint8_t *meta,
                     uint8_t *raw, struct cnand_report *rep)
{
    int rc;
    if (fmt > CNAND_FMT_BOOT || (ecc != CNAND_ECC_HW && ecc != CNAND_ECC_SW) || !cnand_rawsize(n)) return CNAND_EARG;
    memset(rep, 0, sizeof(*rep));
    if (ecc == CNAND_ECC_SW) {
        rc = build(n, fmt, ecc, data, meta, raw, rep);   /* no controller access at all */
    } else {
        rc = begin(n);
        if (!rc) rc = build(n, fmt, ecc, data, meta, raw, rep);
        rc = end(n, rc);
    }
    summarize(rep);
    return rc;
}

int cnand_program_page(struct cnand *n, uint32_t ce, uint32_t row, uint32_t fmt, uint32_t ecc,
                       const uint8_t *data, const uint8_t *meta, uint8_t *raw)
{
    const struct nand_geometry *g = nand_get_geometry();
    uint32_t size = cnand_rawsize(n);
    struct cnand_report rep;
    int rc;
    if (fmt > CNAND_FMT_BOOT || ecc > CNAND_ECC_SW || !g || !size || ce >= g->banks ||
        row >= g->blocks * g->pagesperblock) return CNAND_EARG;
    memset(&rep, 0, sizeof(rep));
    if (ecc != CNAND_ECC_HW && !raw) return CNAND_EARG;
    if (ecc == CNAND_ECC_SW && (rc = build(n, fmt, ecc, data, meta, raw, &rep))) return rc;
    rc = begin(n);
    if (!rc && ecc == CNAND_ECC_HW) rc = hw_program(n, ce, row, fmt, data, meta, raw);
    if (!rc && ecc != CNAND_ECC_HW) {
        g_hal.cache_wb_inv(raw, CNAND_RAW_BUF);
        rc = ll_rc(cnll_program(&n->ll, ce, row, raw, size));
    }
    return end(n, rc);
}

int cnand_erase_block(struct cnand *n, uint32_t ce, uint32_t block)
{
    const struct nand_geometry *g = nand_get_geometry();
    int rc;
    if (!g || ce >= g->banks || block >= g->blocks) return CNAND_EARG;
    rc = begin(n);
    if (!rc) rc = ll_rc(cnll_erase(&n->ll, ce, block * g->pagesperblock));
    return end(n, rc);
}

/* ---------------------------------------------------------------- USB job (NPOC_CMD_CPU_NAND sub-ops) */
int cpu_nand_stack_job(uint8_t *buf, const uint32_t *arg, int ready, uint32_t *result, uint32_t *detail,
                       uint32_t *len)
{
    static struct cnand n;
    const struct nand_geometry *g = nand_get_geometry();
    struct cns_out *o = (struct cns_out *)buf;
    uint32_t op = arg[0] & 0xff, ce = (arg[0] >> 8) & 0xf, ecc = (arg[0] >> 12) & 3, fmt = (arg[0] >> 14) & 3;
    uint32_t path = (arg[0] >> 16) & 1, bch = (arg[0] >> 17) & 3, row = arg[1];
    uint8_t *in = buf + CNS_IN, *data = buf + CNS_DATA, *raw = buf + CNS_RAW;
    int rc = 0;

    *result = NPOC_R_OK;
    *len = 0;
    if (op < CNS_OP_INFO || op > CNS_OP_BUILD || arg[0] >> 19 || bch > 2 || ecc > CNAND_ECC_SW || fmt > CNAND_FMT_BOOT ||
        (op == CNS_OP_READ && ecc == CNAND_ECC_HW && path) ||
        (op != CNS_OP_READ && path) ||
        (op == CNS_OP_ERASE && (ecc || fmt)) ||
        (op == CNS_OP_BUILD && ecc == CNAND_ECC_RAW) ||
        (op == CNS_OP_INFO && (arg[0] & 0x1ff00u)) ||
        arg[2]) {
        *result = NPOC_R_BAD_ARG;
        return 0;
    }
    if (op != CNS_OP_INFO && !ready) { *result = NPOC_R_PRECOND; return 0; }
    if (op != CNS_OP_INFO && op != CNS_OP_BUILD && (!g || ce >= g->banks || row >= g->blocks * g->pagesperblock)) {
        *result = NPOC_R_BAD_ARG;
        return 0;
    }
    memset(o, 0, CNS_DATA);
    o->magic = CNS_MAGIC; o->version = CNS_VERSION;
    o->op = op; o->word = arg[0]; o->ce = ce; o->row = row;
    o->ecc = ecc; o->fmt = fmt; o->path = path; o->max_steps = CNS_MAX_STEPS;
    cnand_init(&n, (struct cpu_nand_step *)(buf + CNS_STEPS), CNS_MAX_STEPS, bch == 1 ? 8 : bch == 2 ? 16 : 0);
    o->strength = cnand_strength(&n);
    o->rawsize = cnand_rawsize(&n);
    if (op == CNS_OP_INFO) { *len = CNS_STEPS; return 0; }
    memset(data, 0xee, 0x1000);
    memset(raw, 0xee, CNAND_RAW_BUF);
    if (op == CNS_OP_READ && ecc == CNAND_ECC_RAW) {
        rc = cnand_read_raw(&n, ce, row, raw, o->rawsize, path);
        if (!rc) o->raw_bytes = o->rawsize;
    } else if (op == CNS_OP_READ) {
        rc = cnand_read_page(&n, ce, row, fmt, ecc, path, data, raw, &o->rep);
        o->bytes = cnand_datasize(fmt);
        if (ecc == CNAND_ECC_SW) o->raw_bytes = o->rawsize;
    } else if (op == CNS_OP_BUILD) {
        rc = cnand_build_page(&n, fmt, ecc, in, in + CNS_IN_META, raw, &o->rep);
        o->raw_bytes = o->rawsize;
    } else if (op == CNS_OP_PROGRAM) {
        g_hal.cache_wb_inv(in, 0x1400);
        if (ecc == CNAND_ECC_RAW) memcpy(raw, in, CNAND_RAW_BUF);
        rc = cnand_program_page(&n, ce, row, fmt, ecc, in, in + CNS_IN_META, raw);
        o->raw_bytes = o->rawsize;
    } else {
        rc = cnand_erase_block(&n, ce, row / g->pagesperblock);
    }
    o->rc = (uint32_t)rc;
    o->elapsed_us = n.elapsed_us;
    o->nsteps = n.ll.nsteps; o->dropped = n.ll.dropped;
    o->nand_status = n.ll.nand_status; o->ctrl0 = n.ll.ctrl0;
    o->pre_reset_rc = n.pre_reset_rc;
    memcpy(o->pre, n.pre, sizeof(n.pre));
    memcpy(o->cleanup, n.cleanup, sizeof(n.cleanup));
    o->post_reset_rc = n.post_reset_rc;
    memcpy(o->post, n.post, sizeof(n.post));
    *len = CNS_BYTES;
    return rc;
}
