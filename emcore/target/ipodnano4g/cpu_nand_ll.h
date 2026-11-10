/* SPDX-License-Identifier: GPL-2.0-or-later
 * CPU-direct FMC primitives shared by cpu_nand.c (m2_cpu_nand experiment jobs)
 * and cpu_nand_stack.c (the CPU-only NAND driver). No FMSS sequencer program
 * runs: each WAIT of the source programs is a bounded CPU poll of its flag,
 * acknowledged as the program does, and every primitive records one step.
 *
 * Register sequences (the source programs):
 *   erase   target/s5l87xx/fmss/erase.s: 60h, 3 row cycles, D0h, status
 *   program nand/fmss/program.s: 80h, 5 cycles, per 512 bytes DMA
 *           RAM->buffer (0x2e0) then data out (0xe4), 10h, status
 *   read    00h, 5 cycles, 30h, status, 00h, then per 512 bytes data in
 *           (0xe2) + DMA buffer->RAM (0x100), or the 32-byte CPU FIFO
 *           transfer of nand-nano4g.c nand_fifo_read
 */
#ifndef N4G_CPU_NAND_LL_H
#define N4G_CPU_NAND_LL_H
#include "global.h"

#define CNLL_REG(o) (*(volatile uint32_t *)(0x38a00000u + (o)))
#define CNLL_CTRL0_BASE 0x00077801u     /* timing 0x77000 | 0x800 | data path, as the programs */
#define CNLL_CTRL0_DMA  0x01000000u
#define CNLL_T_STEP_US  10000u
#define CNLL_T_READY_US 50000u
#define CNLL_T_ECC_US   2000u
#define CNLL_NO_CE      8u              /* setup() chip enable: none (buffer-only operations) */

/* step tags: kind << 24 | index << 16 | argument; bit 31 = timed out */
#define CPU_NAND_K_CMD   1u
#define CPU_NAND_K_ADDR  2u
#define CPU_NAND_K_XFER  3u
#define CPU_NAND_K_DMA   4u
#define CPU_NAND_K_LATCH 5u
#define CPU_NAND_K_READY 6u
#define CPU_NAND_K_ECC   7u
#define CPU_NAND_K_META  8u
#define CPU_NAND_K_FIFO  9u
#define CPU_NAND_K_BEGIN 10u
#define CPU_NAND_K_DONE  11u
#define CPU_NAND_K_STATUS 12u

struct cpu_nand_step { uint32_t tag, value, t_us; };

/* One operation's context: step log, time base and the last chip status. */
struct cnll {
    bool fast;                        /* no step log, immediate-completion polling fast path */
    struct cpu_nand_step *step;
    uint32_t max_steps, nsteps, dropped;
    uint32_t t0, ctrl0, nand_status, bytes;
};

void cnll_start(struct cnll *c, struct cpu_nand_step *steps, uint32_t max_steps);
uint32_t cnll_now(const struct cnll *c);
void cnll_note(struct cnll *c, uint32_t kind, uint32_t index, uint32_t argument, uint32_t value, int timeout);
/* Poll reg & bit for at most us, record the register, then acknowledge (W1C, ack != 0). 0 or -1. */
int cnll_poll(struct cnll *c, uint32_t reg, uint32_t bit, uint32_t ack, uint32_t us,
              uint32_t kind, uint32_t index, uint32_t argument);
int cnll_command(struct cnll *c, uint32_t cmd, uint32_t i);
int cnll_address(struct cnll *c, uint32_t row, uint32_t cycles, uint32_t i);
int cnll_address_bytes(struct cnll *c, uint32_t lo, uint32_t hi, uint32_t cycles, uint32_t i);
int cnll_transfer(struct cnll *c, uint32_t n, uint32_t sel, uint32_t op, uint32_t i);
/* Post DNUM, DATA_SEL and FMCTRL1 and return. Those two fields stay in force
 * until cnll_transfer_wait: an in-flight NAND read is also the DMA source
 * selection, and rewriting either field would redirect it. */
void cnll_transfer_start(uint32_t n, uint32_t sel, uint32_t op);
int cnll_transfer_wait(struct cnll *c, uint32_t i, uint32_t op);
int cnll_dma(struct cnll *c, uint32_t addr, uint32_t sel, int into, uint32_t i);
/* DMA-out of the buffer named by DATA_SEL bits 8/9. Writes FMCTRL1 0x100
 * only: DATA_SEL, DNUM and the DMA address were set with the NAND read. */
void cnll_dma_out_start(void);
int cnll_dma_out_wait(struct cnll *c, uint32_t i);
void cnll_setup(struct cnll *c, uint32_t ce, uint32_t ctrl0);
/* 70h, status latch, ready bit 6 of the latched byte, release; c->nand_status = the byte. */
int cnll_ready(struct cnll *c, uint32_t i, uint32_t us);
int cnll_ready_status(struct cnll *c, uint32_t i, uint32_t us, uint32_t cmd, uint32_t mask);
int cnll_open_page(struct cnll *c, uint32_t row);
/* Whole operations: 0, -1 step timeout, -2 ready timeout, -10 chip status FAIL. */
int cnll_erase(struct cnll *c, uint32_t ce, uint32_t row);
int cnll_program(struct cnll *c, uint32_t ce, uint32_t row, const uint8_t *src, uint32_t n);
/* path 0: DMA (dst must hold n rounded up to 512), 1: 32-byte FIFO (dst word aligned). */
int cnll_read_raw(struct cnll *c, uint32_t ce, uint32_t row, uint8_t *dst, uint32_t n, uint32_t path);
int cnll_read_data(struct cnll *c, uint8_t *dst, uint32_t n, uint32_t path);
/* Release the status latch, deselect, acknowledge FMC_STATUS and ECC_STATUS. */
void cnll_release(void);
#endif
