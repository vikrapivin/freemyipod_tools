//
// iPod nano 4G: defensive device init for the DFU -> SRAM bootstub -> emCORE path (
//    section 3 A). Apple's SecCore/PreEfi chain is skipped in this path; the bootstub (patched to v2 by
//    tools/patch_bootstub.py) does the parts that must run from SRAM (PLL, SDRAM controller, page table). Everything in
//    this file is safe to run from SDRAM with the caches on, is reported on the console before the next step, and
//    never touches the PLL, the SDRAM controller or the live page-table entries.
//
//    targetinit_veryearly(): first thing in init(), interrupts still masked, nothing else initialised:
//      1. VIC0/VIC1 fully masked and acknowledged (bootrom:0x200000a4 does the same; PL192 registers)
//      2. DMAC0: idle channels cleared, their pending status cleared, controller enabled (running channels are left alone)
//      3. L1 page-table entries for VA >= 0x40000000 (0x2202d000..0x2202ffff) set to fault, cleaned, TLB invalidated
//      4. FIQ mode gets a stack (crt0 sets IRQ/ABT/UND/SYS only)
//      5. fuse word read as bl_PreEfi:0x2200e6ec does (reported; not acted upon)
//      6. GPIO pull words as stock (bl_PreEfi .data 0x22010284): port 0 = 0, port 1 = 0x40
//      7. the stock VBUS clock drop (bl_PreEfi:0x2200e524): FCLK 266 -> 133 MHz, HCLK unchanged; run from SRAM with
//         IRQ/FIQ off and bounded waits (N4G_KEEP_FCLK=1 disables it). DFU boots are always USB powered.
//      8. an SDRAM pattern test of the free regions (heap and bss..framebuffer gap), with the caches cleaned and
// invalidated between the write and the read pass, so D1/D2 of show up as data errors here, not later
//    targetinit_early(): console exists -> print the report (lands on the LCD and in the USB console backlog).
//    targetinit_late(): I2C is up -> PMU regs 2/4/0x0a/0x33 reported, hibernate flag (reg 2 bit 7) cleared like PreEfi.
//
#include "global.h"
#include "targetinit.h"
#include "console.h"
#include "i2c.h"
#include "s5l8720.h"
#include "n4g_init_report.h"

#define REG32(a) (*(volatile uint32_t *)(a))
#define VIC_BASE(v) (0x38e00000u + 0x1000u * (v))
#define DMAC0_BASE  0x38200000u
#define PWRCON1_REG 0x3c50004cu
#define FUSE_REG    0x3d100008u
#define CLKCON0_REG 0x3c500000u
#define CLKCON1_REG 0x3c500004u
#ifndef N4G_INIT_MASK
#define N4G_INIT_MASK 0xffu   /* make N4G_INIT_MASK=0xNN: bisect the veryearly steps (bit list in target.mk) */
#endif
#define STEP(bit) ((N4G_INIT_MASK) & (bit))
#define L1_TAIL_START 0x2202d000u   /* entries for VA 0x40000000.. (table base 0x2202c000, bootstub:0x22000040) */
#define L1_TAIL_END   0x22030000u

extern char _inittailend, _poolend, _bssend, _lcdfbstart;
extern void clean_dcache(void);        /* mmu.c: c7,c10,0 + DSB */
extern void invalidate_dcache(void);   /* mmu.c: c7,c14,0 (clean+invalidate) + DSB */

struct n4g_init_report n4g_init_report;   /* read by the host via nand_poc status / READ_MEM; layout in targetinit.h */

static uint32_t fiq_stack[64] IBSS_ATTR;

static void dsb(void) { __asm__ volatile("mcr p15, 0, %0, c7, c10, 4" : : "r"(0) : "memory"); }

/* ---- 1. interrupt controllers ---------------------------------------------------------------------------------- */
static void vic_quiesce(void)
{
    int v;
    n4g_init_report.vic_raw[0] = REG32(VIC_BASE(0) + 0x08);
    n4g_init_report.vic_raw[1] = REG32(VIC_BASE(1) + 0x08);
    for (v = 0; v < 2; v++) {
        REG32(VIC_BASE(v) + 0x14) = 0xffffffffu;   /* INTENCLEAR */
        REG32(VIC_BASE(v) + 0x0c) = 0;             /* INTSELECT: nothing on FIQ */
        REG32(VIC_BASE(v) + 0x1c) = 0xffffffffu;   /* SOFTINTCLEAR */
        REG32(VIC_BASE(v) + 0xf00) = 0;            /* VICADDRESS acknowledge */
    }
    REG32(0x38e02008u) = 0xffffffffu; /* per-IRQ acknowledge words the stock handlers use */
    REG32(0x38e0200cu) = 0xffffffffu;
}

/* ---- 2. DMAC0 (DMAC1 is left alone: its clock gate state is unknown and nothing here uses it) ------------------- */
static void dmac0_quiesce(void)
{
    uint32_t c, idle = 0;
    for (c = 0; c < 8; c++) {
        uint32_t cfg = DMAC0_BASE + 0x110u + 0x20u * c;
        if (REG32(cfg) & 1u) { n4g_init_report.dmac0_active |= 1u << c; continue; }   /* running: do not touch */
        REG32(cfg) = 0;
        REG32(DMAC0_BASE + 0x10cu + 0x20u * c) = 0;
        idle |= 1u << c;
    }
    REG32(DMAC0_BASE + 0x08) = idle;   /* INTTCCLR  (only the idle channels) */
    REG32(DMAC0_BASE + 0x10) = idle;   /* INTERRCLR */
    REG32(DMAC0_BASE + 0x30) = 1;      /* controller enabled (the stub does this too) */
}

/* ---- 3. page-table hygiene ------------------------------------------------------------------------------------- */
static void l1_tail_fault(void)
{
    uint32_t a;
    for (a = L1_TAIL_START; a < L1_TAIL_END; a += 4) REG32(a) = 0;
    for (a = L1_TAIL_START; a < L1_TAIL_END; a += 32) __asm__ volatile("mcr p15, 0, %0, c7, c10, 1" : : "r"(a) : "memory");
    dsb();
    __asm__ volatile("mcr p15, 0, %0, c8, c7, 0" : : "r"(0) : "memory");   /* invalidate unified TLB */
    dsb();
}

/* ---- 4. FIQ stack ---------------------------------------------------------------------------------------------- */
static void fiq_stack_init(void)
{
    uint32_t top = (uint32_t)&fiq_stack[64], old;
    __asm__ volatile("mrs %0, cpsr\n\tmsr cpsr_c, #0xd1\n\tmov sp, %1\n\tmsr cpsr_c, %0" : "=&r"(old) : "r"(top) : "memory");
}

/* ---- 5. fuse --------------------------------------------------------------------------------------------------- */
static void fuse_read(void)
{
    REG32(PWRCON1_REG) &= ~0x4000u;
    n4g_init_report.fuse = REG32(FUSE_REG);
    REG32(PWRCON1_REG) |= 0x4000u;
}

/* ---- 6. GPIO pulls --------------------------------------------------------------------------------------------- */
static void gpio_pulls(void)
{
    n4g_init_report.gpio_pull_before[0] = REG32(0x3cf00010u);
    n4g_init_report.gpio_pull_before[1] = REG32(0x3cf00030u);
    REG32(0x3cf00010u) = 0;
    REG32(0x3cf00030u) = 0x40u;
}

/* ---- 7. VBUS clock drop (runs from SRAM) ----------------------------------------------------------------------- */
static int clock_drop(void) ICODE_ATTR;
static int clock_drop(void)
{
    uint32_t n, old;
    __asm__ volatile("mrs %0, cpsr\n\torr r1, %0, #0xc0\n\tmsr cpsr_c, r1" : "=&r"(old) : : "r1", "memory");
    REG32(CLKCON0_REG) = 0x40001001u;
    for (n = 0; n < 1000000u && (REG32(CLKCON0_REG) & 0xfu) != 1u; n++) { }
    if (n == 1000000u) { __asm__ volatile("msr cpsr_c, %0" : : "r"(old) : "memory"); return 1; }
    REG32(CLKCON1_REG) = 0;
    for (n = 0; n < 1000000u && (REG32(CLKCON1_REG) & (0x3e0000u | 0x3e00u | 0x3eu)); n++) { }
    __asm__ volatile("msr cpsr_c, %0" : : "r"(old) : "memory");
    return n == 1000000u ? 2 : 0;
}

/* ---- 8. SDRAM pattern test ------------------------------------------------------------------------------------- */
static void sdram_test_range(uint32_t start, uint32_t end, uint32_t key)
{
    volatile uint32_t *p;
    uint32_t a;
    start = (start + 31u) & ~31u; end &= ~31u;
    if (end <= start) return;
    for (a = start; a < end; a += 4) REG32(a) = a ^ key;
    invalidate_dcache();                     /* clean + invalidate: the read pass must come from the DRAM */
    for (a = start; a < end; a += 4) {
        uint32_t got = REG32(a);
        if (got != (a ^ key)) {
            if (!n4g_init_report.sdram_fail_count) {
                n4g_init_report.sdram_first_fail = a;
                n4g_init_report.sdram_expected = a ^ key;
                n4g_init_report.sdram_got = got;
            }
            n4g_init_report.sdram_fail_count++;
        }
    }
    n4g_init_report.sdram_tested_bytes += end - start;
    (void)p;
}

void targetinit_veryearly(void)
{
    n4g_init_report.magic = N4G_INIT_REPORT_MAGIC;
    n4g_init_report.step_mask = N4G_INIT_MASK;
    if (STEP(0x01)) vic_quiesce();
    if (STEP(0x02)) dmac0_quiesce();
    if (STEP(0x04)) l1_tail_fault();
    if (STEP(0x08)) fiq_stack_init();
    if (STEP(0x10)) fuse_read();
    if (STEP(0x20)) gpio_pulls();
    n4g_init_report.clkcon_before[0] = REG32(CLKCON0_REG);
    n4g_init_report.clkcon_before[1] = REG32(CLKCON1_REG);
    n4g_init_report.pll0pms = REG32(0x3c500024u);
    n4g_init_report.clock_drop_rc = -1;
#ifndef N4G_KEEP_FCLK
    if (STEP(0x40)) n4g_init_report.clock_drop_rc = clock_drop();
#endif
    n4g_init_report.clkcon_after[0] = REG32(CLKCON0_REG);
    n4g_init_report.clkcon_after[1] = REG32(CLKCON1_REG);
    if (STEP(0x80)) {
        sdram_test_range((uint32_t)&_inittailend, (uint32_t)&_poolend, 0x5aa5f00fu);
        sdram_test_range((uint32_t)&_bssend, (uint32_t)&_lcdfbstart, 0xa55a0ff0u);
    }
    n4g_init_report.stage = 1;
}

void targetinit_early(void)
{
    const struct n4g_init_report *r = &n4g_init_report;
    cprintf(CONSOLE_BOOT, "[init] mask %02X fuse %08X clk %08X/%08X -> %08X/%08X drop=%d pll %08X\n", r->step_mask, r->fuse, r->clkcon_before[0],
            r->clkcon_before[1], r->clkcon_after[0], r->clkcon_after[1], r->clock_drop_rc, r->pll0pms);
    if (r->sdram_fail_count)
        cprintf(CONSOLE_BOOT, "[init] SDRAM FAIL x%u @%08X exp %08X got %08X (%u KiB tested)\n", r->sdram_fail_count,
                r->sdram_first_fail, r->sdram_expected, r->sdram_got, r->sdram_tested_bytes >> 10);
    else
        cprintf(CONSOLE_BOOT, "[init] SDRAM ok (%u KiB) vic %08X/%08X dma-active %02X pull %X/%X\n", r->sdram_tested_bytes >> 10,
                r->vic_raw[0], r->vic_raw[1], r->dmac0_active, r->gpio_pull_before[0], r->gpio_pull_before[1]);
    n4g_init_report.stage = 2;
}

void targetinit_late(void)
{
    uint8_t r2 = i2c_recvbyte(0, 0xe6, 2), r4 = i2c_recvbyte(0, 0xe6, 4);
    uint8_t r0a = i2c_recvbyte(0, 0xe6, 0x0a), r33 = i2c_recvbyte(0, 0xe6, 0x33);
    n4g_init_report.pmu[0] = r2; n4g_init_report.pmu[1] = r4; n4g_init_report.pmu[2] = r0a; n4g_init_report.pmu[3] = r33;
    cprintf(CONSOLE_BOOT, "[init] pmu r2=%02X r4=%02X r0a=%02X r33=%02X%s\n", r2, r4, r0a, r33,
            (r2 & 0x80) ? " (warm flag set: clearing, SDRAM was not cold-initialised this boot)" : "");
    if (r2 & 0x80) i2c_sendbyte(0, 0xe6, 2, r2 & 0x7f);   /* bl_PreEfi:0x2200eb68 */
    n4g_init_report.stage = 3;
}
