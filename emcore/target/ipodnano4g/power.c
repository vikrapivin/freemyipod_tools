//
//
//    Copyright 2010 TheSeven
//
//
//    This file is part of emCORE.
//
//    emCORE is free software: you can redistribute it and/or
//    modify it under the terms of the GNU General Public License as
//    published by the Free Software Foundation, either version 2 of the
//    License, or (at your option) any later version.
//
//    emCORE is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with emCORE.  If not, see <http://www.gnu.org/licenses/>.
//
//


#include "global.h"
#include "i2c.h"
#include "timer.h"
#include "power.h"
#ifdef N4G_LOWPOWER
#include "console.h"
#include "thread.h"
#endif


long power_last_update;
bool power_last_state;


void reset();


void power_off(void)
{
    reset();
}

#ifdef N4G_LOWPOWER
static void n4g_lowpower_init(void);
#endif

void power_init(void)
{
    power_last_update = 0;
#ifdef N4G_LOWPOWER
    n4g_lowpower_init();   /* initthread, right after i2c_init(), before usbmanager_init()/backlight_init()/nand_poc_init() */
#endif
}

bool charging_state(void)
{
    return false;
}

bool external_power_state(void)
{
    return vbus_state();
}

bool vbus_state(void)
{
    return true;
    if (TIMEOUT_EXPIRED(power_last_update, 200000))
    {
        power_last_update = USEC_TIMER;
        power_last_state = !!(i2c_recvbyte(0, 0xe6, 4) & 0x40);
    }
    return power_last_state;
}

int read_battery_voltage(int battery)
{
    return -1;
}

int read_battery_current(int battery)
{
    return -1;
}

int read_battery_mwh_design(int battery)
{
    return -1;
}

int read_battery_mwh_full(int battery)
{
    return -1;
}

int read_battery_mwh_current(int battery)
{
    return -1;
}

int read_battery_mw(int battery)
{
    return -1;
}

enum battery_state read_battery_state(int battery)
{
    if (battery != 0) return BATTERY_STATE_INVALID;
    return BATTERY_STATE_UNKNOWN;
}

int read_input_voltage(int input)
{
    return -1;
}

int read_input_current(int input)
{
    return -1;
}

int read_input_mw(int input)
{
    return -1;
}

enum input_state read_input_state(int input)
{
    if (input != 0) return INPUT_STATE_INVALID;
    return INPUT_STATE_UNKNOWN;
}


#ifdef N4G_LOWPOWER
/*
 * N4G_LOWPOWER (make N4G_LOWPOWER=1): stock USB power policy for the dead-battery unit.
 * Order (each step prints its plan on CONSOLE_BOOT BEFORE the write; nothing survives a reset on that unit):
 *  1. power_init() (init thread, I2C just up, USB not started): report PMU 02/04/0a/30/31/33/34.
 *  2. backlight clamp to 0x64 = 100 (bl_DxeD1759:0xa54 clamps to 100 on a computer port; the stub wrote 0xfa,
 *     bootstub:0x2200034c table 0x220005e2). backlight_set_brightness() clamps too (backlight.c).
 *  3. N4G_LOWPOWER_CLKDROP (default on within N4G_LOWPOWER): the stock VBUS clock drop of PreEfi_pmu_config
 *     bl_PreEfi:0x2200e524 (literals 0x2200e5d4 = 0x40001001, 0x2200e5d8 = 0x3c500000; Ghidra, Oct 18 2026):
 *     if PMU reg 4 b6: CLKCON0 (0x3c500000) = 0x40001001, wait (CLKCON0 & 0xf) == 1; CLKCON1 (0x3c500004) = 0,
 * wait [21:17], [13:9], [5:1] == 0. FCLK 266 -> 133 MHz, HCLK/PCLK end at 133 as before.
 *     Run from SRAM (.icode), leaf, no stack, IRQ/FIQ off, D-cache cleaned+invalidated, DMAC0 idle (or only
 *     channels proven stalled), before the USB core (and its DMA) is started. Bounded waits (harness).
 *  4. a small CORE thread polls the USB configuration flag (set in IRQ context by usbglue.c set/unset_configuration)
 *     and applies the restore OS's power mode in thread context: SET_CONFIGURATION -> mode 6 (fw_restore:0x08027360
 *     -> 0x08026d98(500) -> set_usb_power 0x08015164 -> 0x0802ccc8: 0a &= ~8, 33 [1:0] = 2; 0x0802ce74(on): 34 = 04);
 *     unconfigure / bus reset -> 33 [1:0] = 0 (100 mA; fw mode 4 keeps 34 = 04). [1:0] = 3 (1 A, Apple charger only)
 *     is never written; 0a bits 0..3 are never set; nothing is raised before the host configured the device.
 */
#define PMU_BUS 0
#define PMU_ADDR 0xe6
#define LP_BACKLIGHT_MAX 0x64
#define LP_CLKCON0 0x3c500000u
#define LP_CLKCON1 0x3c500004u
#define LP_DMAC0_CFG(c) (0x38200110u + 0x20u * (c))
#define LP_DMAC0_SRC(c) (0x38200100u + 0x20u * (c))
#define LP_REG32(a) (*(volatile uint32_t *)(a))

static volatile int n4g_usb_budget_ma = 100;   /* written by the USB IRQ (usbglue.c), read by the power thread */
static volatile int n4g_power_applied_ma = 100; /* what the PMU is really set to (read back after each write) */

/* Oct 18 2026 : on the dead-battery unit the automatic NAND bring-up died when it ran right after boot, i.e.
 * still on the 100 mA budget, while the same steps driven by the host AFTER enumeration (500 mA) all passed. The NAND
 * worker asks this before its automatic bring-up. */
int n4g_power_budget_applied_ma(void)
{
    return n4g_power_applied_ma;
}
static struct scheduler_thread n4g_power_thread_handle;
static uint32_t n4g_power_stack[0x200];

static uint8_t pmu_rd(uint32_t reg) { return i2c_recvbyte(PMU_BUS, PMU_ADDR, reg); }
static void pmu_wr(uint32_t reg, uint32_t v) { i2c_sendbyte(PMU_BUS, PMU_ADDR, reg, v & 0xff); }

/* IRQ context (usb.c SET_CONFIGURATION / usb_unconfigure, also on bus reset): only a store, no I2C. */
void n4g_power_usb_budget(int ma)
{
    n4g_usb_budget_ma = ma;
}

#ifdef N4G_LOWPOWER_CLKDROP
/* r0 = DMAC0 channels allowed to stay enabled (proven stalled). Returns 0 ok, 1 CLKCON0 wait timeout,
 * 2 CLKCON1 wait timeout (CLKCON1 was written), 3 refused: another DMAC0 channel enabled (nothing written).
 * Leaf, no stack, code + literals in SRAM: no SDRAM access between the two CLKCON writes (HCLK dips there). */
int n4g_clkdrop(uint32_t allowed) __attribute__((naked, noinline, section(".icode")));
int n4g_clkdrop(uint32_t allowed)
{
    __asm__ volatile(
        "    mrs   r12, cpsr\n"
        "    orr   r3, r12, #0xc0\n"
        "    msr   cpsr_c, r3\n"                 /* IRQ + FIQ off */
        "    ldr   r2, =0x38200110\n"            /* DMAC0 C0CONFIG; channel stride 0x20 */
        "    mov   r1, #0\n"
        "1:  ldr   r3, [r2, r1, lsl #5]\n"
        "    tst   r3, #1\n"
        "    beq   2f\n"
        "    mov   r3, #1\n"
        "    tst   r0, r3, lsl r1\n"
        "    moveq r0, #3\n"
        "    beq   9f\n"
        "2:  add   r1, r1, #1\n"
        "    cmp   r1, #8\n"
        "    blt   1b\n"
        "    mov   r0, #0\n"
        "    mcr   p15, 0, r0, c7, c14, 0\n"     /* clean + invalidate the whole D-cache */
        "    mcr   p15, 0, r0, c7, c10, 4\n"     /* DSB: all write-backs reached SDRAM */
        "    ldr   r2, =0x3c500000\n"
        "    ldr   r3, =0x40001001\n"
        "    str   r3, [r2]\n"                   /* CLKCON0 = 0x40001001 (bl_PreEfi:0x2200e524) */
        "    ldr   r1, =1000000\n"
        "3:  ldr   r3, [r2]\n"
        "    and   r3, r3, #0xf\n"
        "    cmp   r3, #1\n"
        "    beq   4f\n"
        "    subs  r1, r1, #1\n"
        "    bne   3b\n"
        "    mov   r0, #1\n"
        "    b     9f\n"
        "4:  mov   r3, #0\n"
        "    str   r3, [r2, #4]\n"               /* CLKCON1 = 0 */
        "    ldr   r1, =1000000\n"
        "5:  ldr   r3, [r2, #4]\n"
        "    tst   r3, #0x3e0000\n"              /* [21:17] */
        "    bne   6f\n"
        "    tst   r3, #0x3e00\n"                /* [13:9] */
        "    bne   6f\n"
        "    tst   r3, #0x3e\n"                  /* [5:1] */
        "    beq   9f\n"
        "6:  subs  r1, r1, #1\n"
        "    bne   5b\n"
        "    mov   r0, #2\n"
        "9:  mov   r1, #0\n"
        "    mcr   p15, 0, r1, c7, c10, 4\n"
        "    msr   cpsr_c, r12\n"
        "    bx    lr\n"
        "    .ltorg\n");
}

static void n4g_lowpower_clkdrop(uint8_t r4)
{
    uint32_t c0 = LP_REG32(LP_CLKCON0), c1 = LP_REG32(LP_CLKCON1), en = 0, src[8], stalled = 0, c;
    int i, rc;
    if (!(r4 & 0x40)) { cprintf(CONSOLE_BOOT, "[power] clk: no VBUS, skip\n"); return; }
    if (c0 == 0x40001001u && c1 == 0) return;
    cprintf(CONSOLE_BOOT, "[power] clk plan %08X/%08X->40001001/0\n", c0, c1);
    /* DMA must be idle: let the LCD console frame finish (~2.2 ms); a channel that stays enabled for 100 ms without
     * moving its source pointer is stalled and moves no data. */
    for (c = 0; c < 8; c++) src[c] = LP_REG32(LP_DMAC0_SRC(c));
    for (i = 0; i < 100; i++) {
        en = 0;
        for (c = 0; c < 8; c++) if (LP_REG32(LP_DMAC0_CFG(c)) & 1u) en |= 1u << c;
        if (!en) break;
        sleep(1000);
    }
    for (c = 0; c < 8; c++)
        if ((en & (1u << c)) && LP_REG32(LP_DMAC0_SRC(c)) == src[c]) stalled |= 1u << c;
    if (en & ~stalled) { cprintf(CONSOLE_BOOT, "[power] clk: dma %02X busy, skip\n", en); return; }
    cprintf(CONSOLE_BOOT, "[power] clk go (stalled %02X)\n", stalled);
    sleep(20000);   /* give the LCD console a frame to show the line above before the risky write */
    rc = n4g_clkdrop(stalled);
    cprintf(CONSOLE_BOOT, "[power] clk rc=%d %08X/%08X\n", rc, LP_REG32(LP_CLKCON0), LP_REG32(LP_CLKCON1));
}
#endif

/* thread context only */
static void n4g_lowpower_usb_500(void)
{
    uint8_t r4 = pmu_rd(0x04), r0a = pmu_rd(0x0a), r33 = pmu_rd(0x33), r34 = pmu_rd(0x34);
    uint8_t v33 = (r33 & 0xfc) | 0x02;   /* [1:0] = 2 = 500 mA (fw_restore:0x0802ccc8 mode 6 -> 0x52) */
    cprintf(CONSOLE_BOOT, "[power] cfg 04=%02X 0a=%02X 33=%02X 34=%02X\n", r4, r0a, r33, r34);
    if (!(r4 & 0x40) || (r0a & 0x03) || (v33 & 3) != 2) {
        cprintf(CONSOLE_BOOT, "[power] precond fail: 100 mA\n");
        return;
    }
    cprintf(CONSOLE_BOOT, "[power] plan 0a=%02X 33=%02X 34=04\n", r0a & 0xf7, v33);
    if (r0a & 0x08) pmu_wr(0x0a, r0a & 0xf7);   /* 0x0802ccc8: clear b3 only (input on) */
    pmu_wr(0x33, v33);
    pmu_wr(0x34, 0x04);                          /* 0x0802ce74(on) */
    r4 = pmu_rd(0x04); r0a = pmu_rd(0x0a); r33 = pmu_rd(0x33); r34 = pmu_rd(0x34);
    cprintf(CONSOLE_BOOT, "[power] USB limit %s mA (04=%02X 0a=%02X 33=%02X 34=%02X)\n",
            (r33 & 3) == 2 ? "500" : "??", r4, r0a, r33, r34);
    if ((r33 & 3) == 2) n4g_power_applied_ma = 500;
}

static void n4g_lowpower_usb_100(void)
{
    uint8_t r33 = pmu_rd(0x33);
    cprintf(CONSOLE_BOOT, "[power] uncfg plan 33 %02X->%02X\n", r33, r33 & 0xfc);
    pmu_wr(0x33, r33 & 0xfc);                    /* back to 100 mA (stock budget < 500 -> mode 4: 33 = 0x50) */
    n4g_power_applied_ma = 100;
    cprintf(CONSOLE_BOOT, "[power] USB limit 100 mA (33=%02X)\n", pmu_rd(0x33));
}

static void n4g_power_thread(void* a0, void* a1, void* a2, void* a3)
{
    int applied = 100;
    (void)a0; (void)a1; (void)a2; (void)a3;
    while (true) {
        int want;
        sleep(20000);
        want = n4g_usb_budget_ma >= 500 ? 500 : 100;   /* never more than the 500 mA emCORE declares */
        if (want == applied) continue;
        applied = want;
        if (want == 500) n4g_lowpower_usb_500();
        else n4g_lowpower_usb_100();
    }
}

static void n4g_lowpower_init(void)
{
    uint8_t r2 = pmu_rd(0x02), r4 = pmu_rd(0x04), r0a = pmu_rd(0x0a), r30 = pmu_rd(0x30), r31 = pmu_rd(0x31);
    uint8_t r33 = pmu_rd(0x33), r34 = pmu_rd(0x34);
    cprintf(CONSOLE_BOOT, "[power] 02=%02X 04=%02X 0a=%02X 30=%02X 31=%02X 33=%02X 34=%02X clk %08X/%08X\n",
            r2, r4, r0a, r30, r31, r33, r34, LP_REG32(LP_CLKCON0), LP_REG32(LP_CLKCON1));
    if (r30 > LP_BACKLIGHT_MAX) {
        cprintf(CONSOLE_BOOT, "[power] plan 30 %02X->%02X\n", r30, LP_BACKLIGHT_MAX);
        pmu_wr(0x30, LP_BACKLIGHT_MAX);
        cprintf(CONSOLE_BOOT, "[power] 30=%02X\n", pmu_rd(0x30));
    }
#ifdef N4G_LOWPOWER_CLKDROP
    n4g_lowpower_clkdrop(r4);
#endif
    if (!thread_create(&n4g_power_thread_handle, "power", n4g_power_thread, n4g_power_stack, sizeof(n4g_power_stack),
                       CORE_THREAD, 100, true, NULL, NULL, NULL, NULL))
        cprintf(CONSOLE_BOOT, "[power] no thread\n");
}
#endif /* N4G_LOWPOWER */
