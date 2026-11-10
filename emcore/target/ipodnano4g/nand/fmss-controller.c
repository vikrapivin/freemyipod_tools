/* SPDX-License-Identifier: GPL-2.0-or-later
 * S5L8720 CPU-side binding of the shared nano4g/nano5g M2 lifecycle.
 * bl_NandReadOnly mem:0x502c/0x51b8; osos mem:0x08069ba4/0x080daf1c.
 * No nano5g pinmux, clock, geometry or parameter-list setup is used here. */
#include "rockbox-compat.h"
#include "fmss-controller.h"
#include "nand-profile.h"
static uint32_t reset_phase;
#define FMSS_RESET_PHASE(n) (reset_phase = (n))

#define FMSS_RD(off) (*(volatile uint32_t *)(FMC_BASE + (off)))
#define FMSS_WR(off, v) (FMSS_RD(off) = (v))
#define FMC_CTRL1 0x004
#define FMCTRL1_RESET 0x800
#define FMCTRL1_IDLE (1u << 30)
#define FMCTRL1_RELEASE 0x80000000u
#define ECC_RESET 0x81c
#define FMSS_CTRL 0xc00
#define FMSS_CTRL_START 0xffb5
#define FMSS_CTRL_RESUME 1
#define FMSS_CTRL_ABORT 2
#define FMSS_CTRL_STOP 8
#define FMSS_CTRL_FMCRESET 0x80000
#define FMSS_PROG 0xc04
#define FMSS_STATUS 0xc08
#define FMSS_IRQ 0xc0c
#define FMSS_IRQ_DONE 1
#define FMSS_IRQ_FAULT 4
#define FMSS_IRQ_PAUSE 8
#define FMSS_IRQ_ALL 0xff
#define FMSS_IRQ_MASK 0xd
#define FMSS_PC 0xc38
#define FMSS_BUFRST 0xc60
#define FMSS_BUFRST_OK 0xc64
#define FMSS_BUFCFG 0xc6c
#define FMSS_BUFCFG_BYPASS 0x10
#define NAND_ERR_TIMEOUT (-3)
#define POLL_TIMEOUT_US 100
#define PROG_TIMEOUT_US 2000000
#undef USEC_TIMER
#define USEC_TIMER read_usec_timer()
#include "target/s5l87xx/fmss-controller.h"

int n4g_fmss_reset(void)
{
    unsigned int previous = nand_profile_enter(NAND_PROFILE_RESET);
    int rc = fmss_reset();
    nand_profile_leave(previous);
    return rc;
}
int n4g_fmss_reset_traced(struct n4g_fmss_reset_state *s)
{
    int rc = fmss_reset();
    s->phase = reset_phase;
    s->control = FMSS_RD(FMSS_CTRL); s->status = FMSS_RD(FMSS_STATUS);
    s->bufrst = FMSS_RD(FMSS_BUFRST); s->bufok = FMSS_RD(FMSS_BUFRST_OK);
    s->bufcfg = FMSS_RD(FMSS_BUFCFG);
    s->fmc_control = FMSS_RD(FMC_CTRL1); s->fmc_status = FMSS_RD(0x048);
    return rc;
}
int n4g_fmss_start(uint32_t control) { return fmss_start_word(control); }
int n4g_fmss_run(const void *program, uint32_t len)
{
    /* Caller resets before writing parameter RAM. Resetting here discards
     * those inputs (p11 native-probe reset boundary). */
    unsigned int previous = nand_profile_enter(NAND_PROFILE_SETUP);
    int rc = -2;
    if (!fmss_clear_irq()) {
        commit_dcache_range(program, len);
        FMSS_WR(FMSS_PROG, (uint32_t)program);
        nand_profile_enter(NAND_PROFILE_EXEC);
        if (!fmss_start()) rc = fmss_wait();
    }
    nand_profile_leave(previous);
    return rc;
}

void commit_discard_dcache_range(void *p, uint32_t n)
{
    clean_dcache_range((uint32_t)p, n);
    invalidate_dcache_range((uint32_t)p, n);
}

int lcd_wait_write_dma(void)
{
    /* Same serialization as Rockbox; bounded to keep the monitor responsive. */
    unsigned int previous = nand_profile_enter(NAND_PROFILE_INTERLOCK);
    uint32_t start = read_usec_timer();
    int rc = 0;
    while (DMAC0C4CONFIG & 1)
        if ((uint32_t)(read_usec_timer() - start) > 100000) { rc = -3; break; }
    nand_profile_leave(previous);
    return rc;
}
