/* SPDX-License-Identifier: GPL-2.0-or-later
 * CPU-direct FMC primitives (see cpu_nand_ll.h). Each WAIT N of a program
 * becomes a bounded CPU poll of the flag the book gives for it (FMC_STATUS
 * bits 1, 2, 3, 20, 23; WAIT12 = latched status byte 0x04c bit 6). The flag
 * is then acknowledged exactly as the program does.
 */
#include "global.h"
#include "timer.h"
#include "nand_hal_emcore.h"
#include "cpu_nand_ll.h"

#define R CNLL_REG

void cnll_start(struct cnll *c, struct cpu_nand_step *steps, uint32_t max_steps)
{
    memset(c, 0, sizeof(*c));
    c->step = steps;
    c->max_steps = max_steps;
    c->t0 = read_usec_timer();
}

uint32_t cnll_now(const struct cnll *c) { return read_usec_timer() - c->t0; }

void cnll_note(struct cnll *c, uint32_t kind, uint32_t index, uint32_t argument, uint32_t value, int timeout)
{
    struct cpu_nand_step *s;
    if (c->fast) return;
    if (c->nsteps >= c->max_steps) { c->dropped++; return; }
    s = &c->step[c->nsteps++];
    s->tag = (timeout ? 0x80000000u : 0) | kind << 24 | (index & 0xff) << 16 | (argument & 0xffff);
    s->value = value;
    s->t_us = cnll_now(c);
}

int cnll_poll(struct cnll *c, uint32_t reg, uint32_t bit, uint32_t ack, uint32_t us,
              uint32_t kind, uint32_t index, uint32_t argument)
{
    uint32_t start, v;
    /* Most short controller operations have finished by this first read.
     * Keep the bounded timer loop for a genuinely pending operation. */
    if (c->fast) {
        v = R(reg);
        if (v & bit) goto ready;
    }
    start = read_usec_timer();
    do {
        v = R(reg);
        if (v & bit) goto ready;
    } while (read_usec_timer() - start < us);
    cnll_note(c, kind, index, argument, v, 1);
    return -1;
ready:
    if (!c->fast) cnll_note(c, kind, index, argument, v, 0);
    if (ack) R(reg) = ack;
    return 0;
}

int cnll_command(struct cnll *c, uint32_t cmd, uint32_t i)
{
    R(0x008) = cmd;
    return cnll_poll(c, 0x048, 2, 2, CNLL_T_STEP_US, CPU_NAND_K_CMD, i, cmd);
}

int cnll_address(struct cnll *c, uint32_t row, uint32_t cycles, uint32_t i)
{
    return cnll_address_bytes(c, cycles == 5 ? row << 16 : row, cycles == 5 ? row >> 16 : 0, cycles, i);
}

int cnll_address_bytes(struct cnll *c, uint32_t lo, uint32_t hi, uint32_t cycles, uint32_t i)
{
    R(0x02c) = cycles - 1;
    R(0x00c) = lo;
    R(0x010) = hi;
    R(0x004) = 1;
    return cnll_poll(c, 0x048, 4, 4, CNLL_T_STEP_US, CPU_NAND_K_ADDR, i, cycles);
}

void cnll_transfer_start(uint32_t n, uint32_t sel, uint32_t op)
{
    R(0x030) = n - 1;
    R(0x014) = sel;
    R(0x004) = op;
}

int cnll_transfer_wait(struct cnll *c, uint32_t i, uint32_t op)
{
    return cnll_poll(c, 0x048, 8, 8, CNLL_T_STEP_US, CPU_NAND_K_XFER, i, op);
}

int cnll_transfer(struct cnll *c, uint32_t n, uint32_t sel, uint32_t op, uint32_t i)
{
    cnll_transfer_start(n, sel, op);
    return cnll_transfer_wait(c, i, op);
}

int cnll_dma(struct cnll *c, uint32_t addr, uint32_t sel, int into, uint32_t i)
{
    R(0x030) = 0x1ff;
    R(0x014) = sel;
    R(0x034) = addr;
    R(0x004) = into ? 0x2e0 : 0x100;
    return cnll_poll(c, 0x048, 0x100000, 0x100000, CNLL_T_STEP_US, CPU_NAND_K_DMA, i, into ? 0x2e0 : 0x100);
}

void cnll_dma_out_start(void)
{
    R(0x004) = 0x100;
}

int cnll_dma_out_wait(struct cnll *c, uint32_t i)
{
    return cnll_poll(c, 0x048, 0x100000, 0x100000, CNLL_T_STEP_US, CPU_NAND_K_DMA, i, 0x100);
}

void cnll_setup(struct cnll *c, uint32_t ce, uint32_t ctrl0)
{
    R(0x004) = 0x33338e0;
    R(0x048) = 0xff00ffe;
    c->ctrl0 = ctrl0 | (ce < 8 ? 2u << ce : 0);
    R(0x000) = c->ctrl0;
}

/* 70h, status latch (FMCTRL1 0xca), the latched byte's ready bit 6 as
 * WAIT12 arg=mask=0x40, then the WRITE/ERASE release. */
int cnll_ready(struct cnll *c, uint32_t i, uint32_t us)
{
    return cnll_ready_status(c, i, us, 0x70, 0x40);
}

int cnll_ready_status(struct cnll *c, uint32_t i, uint32_t us, uint32_t cmd, uint32_t mask)
{
    int rc = cmd == 0x100 ? 0 : cnll_command(c, cmd, i);
    if (rc) return rc;
    R(0x004) = 0xca;
    cnll_poll(c, 0x048, 0x800000, 0, CNLL_T_STEP_US, CPU_NAND_K_LATCH, i, 0xca);   /* recorded, not required */
    rc = cnll_poll(c, 0x04c, mask, 0, us, CPU_NAND_K_READY, i, mask);
    c->nand_status = R(0x04c);
    R(0x004) = 0x30020;
    R(0x048) = 0x1800000;
    return rc;
}

int cnll_open_page(struct cnll *c, uint32_t row)
{
    if (cnll_command(c, 0x00, 0) || cnll_address(c, row, 5, 0) || cnll_command(c, 0x30, 1) ||
        cnll_ready(c, 2, CNLL_T_READY_US) || cnll_command(c, 0x00, 3))
        return -1;
    return 0;
}

int cnll_erase(struct cnll *c, uint32_t ce, uint32_t row)
{
    cnll_setup(c, ce, CNLL_CTRL0_BASE);
    if (cnll_command(c, 0x60, 0) || cnll_address(c, row, 3, 0) || cnll_command(c, 0xd0, 1)) return -1;
    if (cnll_ready(c, 2, CNLL_T_READY_US)) return -2;
    return c->nand_status & 1 ? -10 : 0;
}

int cnll_program(struct cnll *c, uint32_t ce, uint32_t row, const uint8_t *src, uint32_t n)
{
    uint32_t pos, i = 0;
    cnll_setup(c, ce, CNLL_CTRL0_BASE | CNLL_CTRL0_DMA);
    R(0x038) = 7;                       /* program.s: memory-to-controller DMA */
    if (cnll_command(c, 0x80, 0) || cnll_address(c, row, 5, 0)) return -1;
    for (pos = 0; pos < n; pos += 512, i++) {
        uint32_t len = n - pos < 512 ? n - pos : 512;
        if (cnll_dma(c, (uint32_t)(src + pos), 1, 1, i) || cnll_transfer(c, len, 0x100, 0xe4, i)) return -1;
    }
    c->bytes = n;
    if (cnll_command(c, 0x10, 1)) return -1;
    if (cnll_ready(c, 2, CNLL_T_READY_US)) return -2;
    return c->nand_status & 1 ? -10 : 0;
}

int cnll_read_raw(struct cnll *c, uint32_t ce, uint32_t row, uint8_t *dst, uint32_t n, uint32_t path)
{
    cnll_setup(c, ce, CNLL_CTRL0_BASE | (path ? 0 : CNLL_CTRL0_DMA));
    if (cnll_open_page(c, row)) return -1;
    return cnll_read_data(c, dst, n, path);
}

int cnll_read_data(struct cnll *c, uint8_t *dst, uint32_t n, uint32_t path)
{
    uint32_t pos, i = 0, k;
    uint32_t ctrl0 = R(0x000);
    int rc = 0;
    /* A generic sequence can switch from DMA to FIFO without setup(). With
     * bit 24 still set the FIFO control words also start memory transfers at
     * the advanced DMA address, corrupting earlier sequence output (p48).
     * Select the requested data path, retaining the caller's timing and CE. */
    R(0x000) = path ? ctrl0 & ~CNLL_CTRL0_DMA : ctrl0 | CNLL_CTRL0_DMA;
    if (!path) {
        g_hal.cache_wb_inv(dst, (n + 511) & ~511u);
        for (pos = 0; pos < n; pos += 512, i++) {
            uint32_t len = n - pos < 512 ? n - pos : 512;
            if (cnll_transfer(c, len, 1, 0xe2, i) || cnll_dma(c, (uint32_t)(dst + pos), 0x100, 0, i)) { rc = -1; goto done; }
        }
        n4g_emcore_cache_inv(dst, (n + 511) & ~511u);
    } else {
        uint32_t *w = (uint32_t *)dst;
        /* nand-nano4g.c nand_fifo_read: 32 bytes per transfer through 0x080 */
        for (pos = 0; pos < n; pos += 32, i++) {
            uint32_t len = n - pos < 32 ? n - pos : 32;
            R(0x030) = len - 1;
            R(0x014) = 1;
            R(0x048) = 8;
            R(0x004) = 0x1c2;
            if (cnll_poll(c, 0x048, 8, 0, CNLL_T_STEP_US, CPU_NAND_K_FIFO, i, len)) { rc = -1; goto done; }
            R(0x014) = 0x100;
            R(0x004) = 0x340;
            for (k = 0; k < (len + 3) / 4; k++) *w++ = R(0x080);
        }
    }
    c->bytes = n;
done:
    R(0x000) = ctrl0;
    return rc;
}

void cnll_release(void)
{
    R(0x004) = 0x20;
    R(0x000) = 0;
    R(0x048) = 0xffffffff;
    R(0x810) = 0x1ff;
}
