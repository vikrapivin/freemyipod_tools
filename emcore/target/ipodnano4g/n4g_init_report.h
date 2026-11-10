/* Report filled by target/ipodnano4g/targetinit.c (defensive init, A). The host reads it through the `n4g_init_report` symbol of the build's map. All fields uint32 so the layout is trivial. */
#ifndef N4G_INIT_REPORT_H
#define N4G_INIT_REPORT_H
#include "global.h"
#define N4G_INIT_REPORT_MAGIC 0x54494e49u   /* 'INIT' */
struct n4g_init_report
{
    uint32_t magic, stage;              /* stage 1 = veryearly done, 2 = reported on the console, 3 = PMU step done */
    uint32_t fuse;
    uint32_t clkcon_before[2], clkcon_after[2], pll0pms;
    int32_t  clock_drop_rc;             /* 0 ok, 1/2 wait timed out, -1 disabled (N4G_KEEP_FCLK) */
    uint32_t vic_raw[2], dmac0_active;
    uint32_t gpio_pull_before[2];
    uint32_t sdram_tested_bytes, sdram_fail_count, sdram_first_fail, sdram_expected, sdram_got;
    uint32_t pmu[4];                    /* regs 2, 4, 0x0a, 0x33 */
    uint32_t step_mask;                 /* N4G_INIT_MASK the image was built with (round-8 bisect) */
};
extern struct n4g_init_report n4g_init_report;
#endif
