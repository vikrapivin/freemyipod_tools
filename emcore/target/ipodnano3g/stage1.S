@
@    iPod nano 3G (S5L8702) first-stage SRAM loader: DFU hand-off -> clocks, timer, SDRAM -> emCORE.
@
@    This file is part of emCORE.
@
@    emCORE is free software: you can redistribute it and/or
@    modify it under the terms of the GNU General Public License as
@    published by the Free Software Foundation, either version 2 of the
@    License, or (at your option) any later version.
@
@    emCORE is distributed in the hope that it will be useful,
@    but WITHOUT ANY WARRANTY; without even the implied warranty of
@    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
@    See the GNU General Public License for more details.
@
@    Runs where the mask ROM's DFU puts the body of the IMG1 "8702" image tools/nano3g/image.py builds
@    (0x22000000, header stripped, entry offset 0). Position independent (the kernel is found relative to
@    the code), so it does not depend on the exact load address.
@
@    Values: the clock and SDRAM sequence is Rockbox's S5L8702 bootloader preinit (syscon_preinit and
@    miu_preinit, IPOD_NANO3G branches, firmware/target/arm/s5l8702/system-s5l8702.c, GPLv2+); PLL2, the system
@    dividers and the CG16 words are the same as emCORE's S5L8702 loader (loader/ipodclassic), which also gives
@    the SM1 divider and the timer E setup the kernel expects (USEC_TIMER = TECNT, 1 MHz). The pin functions are
@    Rockbox's nano 3G gpio_preinit table. PMU and LCD setup are left to the kernel; the clock gates are left as
@    the ROM set them except SMx, SM1, timer and GPIO.
@
@    Layout after this stage: kernel (emcore.bin) copied to 0x08000000 and entered there, SVC mode, IRQ/FIQ
@    masked, VIC sources disabled, MMU and caches off, translation table base = 0x2203c000 holding an identity
@    section map (crt0 patches it and turns the MMU on).
@
@    Failure policy: every poll is bounded; a timeout or a failed SDRAM check ends in stage1_fail, a spin
@    with the step code in r7 (nothing survives a reset: a unit that neither shows emCORE nor re-enumerates
@    as DFU died here).

.syntax unified
.cpu arm926ej-s
.arm

#define SYSCON      0x3c500000
#define CLKCON0     0x00
#define CLKCON1     0x04
#define CLKCON2     0x08        /* CG16_2L | CG16_SVID << 16 */
#define CLKCON3     0x0c        /* CG16_AUD0 | CG16_AUD1 << 16 */
#define CLKCON4     0x10        /* CG16_AUD2 | CG16_RTIME << 16 */
#define CLKCON5     0x14        /* CG16_5L */
#define PLL2PMS     0x28
#define PLL2LCNT    0x38
#define PLLLOCK     0x40
#define PLLMODE     0x44
#define PWRCON0     0x48
#define PWRCON1     0x4c
#define PLLMOD2     0x60

#define SM1_DIV     0x38501000
#define TIMERBASE   0x3c700000
#define MIUBASE     0x38100000
#define VIC0        0x38e00000
#define VIC1        0x38e01000
#define OTGBASE     0x38400000
#define TTB_BASE    0x2203c000          /* crt0.S builds on this table */
#define GPIOBASE    0x3cf00000

#define KERNEL_BASE 0x08000000
#define KERNEL_MAX  0x01f80000  /* INIT region of ls.x */
#define WAIT_LOOPS  0x200000

.section .text.entry,"ax",%progbits
.global _stage1_entry
_stage1_entry:
    msr     cpsr_c, #0xd3               /* SVC, IRQ and FIQ masked */

    /* clean the data cache, then MMU, caches and protection off (Rockbox crt0 for this SoC) */
1:  mrc     p15, 0, APSR_nzcv, c7, c10, 3
    bne     1b
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 4      /* drain write buffer */
    mrc     p15, 0, r1, c1, c0, 0
    bic     r1, r1, #0x1000             /* I */
    bic     r1, r1, #0x5                /* C, M */
    mcr     p15, 0, r1, c1, c0, 0
    mcr     p15, 0, r0, c7, c7, 0       /* invalidate both caches */
    mcr     p15, 0, r0, c8, c7, 0       /* invalidate TLBs */

    /* disable every VIC source; the kernel programs the VIC itself */
    mvn     r1, #0
    ldr     r2, =VIC0
    str     r1, [r2, #0x14]
    ldr     r2, =VIC1
    str     r1, [r2, #0x14]

    /* The ROM calls us from its DFU loop (bootrom mem:0x200034a0) with its USB session still live: the PHY is only
       powered down if the image returns (0x2000a26c). Stop the core before anything reuses SRAM it may DMA into:
       soft disconnect, global interrupt and DMA enable off, interrupts masked, core soft reset (bounded wait; the
       kernel's synopsysotg_init resets the core again). Same step as the nano 5G stage 1. */
    ldr     r2, =OTGBASE
    mov     r1, #2
    str     r1, [r2, #0x804]            /* DCTL: soft disconnect */
    ldr     r1, [r2, #0x008]
    bic     r1, r1, #0x21
    str     r1, [r2, #0x008]            /* GAHBCFG: GlblIntrMsk, DMAEn off */
    mov     r1, #0
    str     r1, [r2, #0x018]            /* GINTMSK */
    mov     r1, #1
    str     r1, [r2, #0x010]            /* GRSTCTL: CSftRst */
    ldr     r3, =WAIT_LOOPS
13: ldr     r1, [r2, #0x010]
    tst     r1, #1
    beq     14f
    subs    r3, r3, #1
    bne     13b
14:

    /* ---- clocks: system on the oscillator, PLLs off, PLL2 = 12 MHz * 36 / 1 / 2, back to PLL2 ---- */
    mov     r7, #1
    ldr     r0, =SYSCON
    mov     r1, #0
    str     r1, [r0, #CLKCON0]          /* CG16_SYS = OSC /1, SDR_DISABLE clear */
    str     r1, [r0, #CLKCON1]          /* cdiv = hdiv = pdiv = 1 */
    ldr     r3, =WAIT_LOOPS
2:  ldr     r2, [r0, #CLKCON1]
    movs    r2, r2, lsr #8
    beq     3f
    subs    r3, r3, #1
    bne     2b
    b       stage1_fail
3:  ldr     r1, [r0, #PLLMODE]
    bic     r1, r1, #0x70000            /* PLLOUT 0..2 off (slow mode) */
    str     r1, [r0, #PLLMODE]
    mov     r3, #0x4000                 /* settle, > 50 us on the oscillator */
4:  subs    r3, r3, #1
    bne     4b
    bic     r1, r1, #0x7                /* EN 0..2 off */
    bic     r1, r1, #0x100              /* OSCSEL = OSC0 */
    bic     r1, r1, #0x40               /* PLL2 divisor mode (DM) */
    str     r1, [r0, #PLLMODE]
    mov     r1, #0
    str     r1, [r0, #PLLMOD2]
    ldr     r1, =0x01002401             /* P 1, M 36, S 1 */
    str     r1, [r0, #PLL2PMS]
    ldr     r1, =32400                  /* lock time, PClk ticks */
    str     r1, [r0, #PLL2LCNT]
    ldr     r1, [r0, #PLLMODE]
    orr     r1, r1, #0x4                /* EN2 */
    str     r1, [r0, #PLLMODE]
    mov     r7, #2
    ldr     r3, =WAIT_LOOPS
5:  ldr     r2, [r0, #PLLLOCK]
    tst     r2, #0x4
    bne     6f
    subs    r3, r3, #1
    bne     5b
    b       stage1_fail
6:  orr     r1, r1, #0x40000            /* PLLOUT2: PLL2 out of slow mode */
    str     r1, [r0, #PLLMODE]
    mov     r7, #3
    ldr     r1, =0x00404101             /* cdiv 1, hdiv 2, pdiv 4 (hprat 2) */
    str     r1, [r0, #CLKCON1]
    ldr     r3, =WAIT_LOOPS
7:  ldr     r2, [r0, #CLKCON1]
    eor     r2, r2, r1
    movs    r2, r2, lsr #8
    beq     8f
    subs    r3, r3, #1
    bne     7b
    b       stage1_fail
8:  mov     r1, #0x3000                 /* CG16_SYS = PLL2 /1 */
    str     r1, [r0, #CLKCON0]
    ldr     r1, =0x80008000             /* 2L, SVID off */
    str     r1, [r0, #CLKCON2]
    str     r1, [r0, #CLKCON3]          /* AUD0, AUD1 off */
    mov     r1, #0x8000                 /* AUD2 off, RTIME = OSC /1 on (EClk: timer, DRAM refresh) */
    str     r1, [r0, #CLKCON4]
    str     r1, [r0, #CLKCON5]          /* 5L off */
    ldr     r2, =SM1_DIV
    mov     r1, #3
    str     r1, [r2]

    /* clock gates: SMx (3), SM1 (4) and the timer (37) on, the rest as the ROM left them */
    ldr     r1, [r0, #PWRCON0]
    bic     r1, r1, #0x18
    str     r1, [r0, #PWRCON0]
    ldr     r1, [r0, #PWRCON1]
    bic     r1, r1, #0x20
    str     r1, [r0, #PWRCON1]

    /* GPIO gate (44) on and the pin functions: Rockbox's nano 3G gpio_preinit table (PCON per port, pull-up
       registers +0x0c/+0x10 cleared), which its DFU-loaded bootloader applies on this hardware */
    ldr     r1, [r0, #PWRCON1]
    bic     r1, r1, #0x1000
    str     r1, [r0, #PWRCON1]
    ldr     r2, =GPIOBASE
    adr     r3, gpio_pcon
    mov     r4, #16
    mov     r5, #0
15: ldr     r1, [r3], #4
    str     r1, [r2, #0x00]
    str     r5, [r2, #0x0c]
    str     r5, [r2, #0x10]
    add     r2, r2, #0x20
    subs    r4, r4, #1
    bne     15b

    /* timer E: free running 1 MHz counter (TECNT = USEC_TIMER) */
    ldr     r2, =TIMERBASE
    mov     r1, #0x440
    str     r1, [r2, #0xa0]             /* TECON */
    mov     r1, #0xb
    str     r1, [r2, #0xb0]             /* TEPRE: 12 MHz / 12 */
    mvn     r1, #0
    str     r1, [r2, #0xa8]             /* TEDATA0 */
    mov     r1, #3
    str     r1, [r2, #0xa4]             /* TECMD: clear + enable */

    /* ---- SDRAM: Rockbox miu_preinit(selfrefreshing = false), nano 3G values ---- */
    ldr     r0, =MIUBASE
    ldr     r1, =0x1000100d
    str     r1, [r0, #0x00]             /* MIUCON */
    mov     r1, #0
    str     r1, [r0, #0xf0]
    ldr     r1, =0x0004105d
    str     r1, [r0, #0x08]             /* MIUAREF */
    ldr     r1, =0x001fb621
    str     r1, [r0, #0x10]             /* MIUSDPARA */
    ldr     r1, =0x1845
    ldr     r2, =0x1800
    ldr     r3, =0x1885
    str     r1, [r0, #0x200]
    str     r1, [r0, #0x204]
    str     r2, [r0, #0x210]
    str     r2, [r0, #0x214]
    str     r1, [r0, #0x220]
    str     r1, [r0, #0x224]
    str     r3, [r0, #0x230]
    str     r3, [r0, #0x234]
    mov     r1, #0x19
    str     r1, [r0, #0x14]
    str     r1, [r0, #0x18]
    ldr     r1, =0x0790682b
    str     r1, [r0, #0x1c]
    ldr     r1, [r0, #0x314]
    bic     r1, r1, #0x10
    str     r1, [r0, #0x314]
    add     r2, r0, #0x2c
    mov     r3, #0x24
9:  ldr     r1, [r2]
    bic     r1, r1, #0x01000000
    str     r1, [r2], #4
    subs    r3, r3, #1
    bne     9b
    mov     r1, #0x540
    str     r1, [r0, #0x1cc]
    ldr     r1, [r0, #0x1d4]
    orr     r1, r1, #0x80
    str     r1, [r0, #0x1d4]

    adr     r2, miu_cmds                /* MIUCOM / MIUMRS sequence: {offset, value} pairs, 0 ends */
10: ldmia   r2!, {r3, r4}
    cmp     r3, #0
    strne   r4, [r0, r3]
    bne     10b

    ldr     r1, [r0, #0x08]
    orr     r1, r1, #0x61000            /* auto refresh on */
    str     r1, [r0, #0x08]

    /* SDRAM check: two patterns at the start and the end of the kernel region */
    mov     r7, #4
    ldr     r0, =KERNEL_BASE
    ldr     r1, =0x5a5aa5a5
    mvn     r2, r1
    ldr     r3, =(KERNEL_BASE + KERNEL_MAX - 8)
    str     r1, [r0]
    str     r2, [r0, #4]
    str     r2, [r3]
    str     r1, [r3, #4]
    ldr     r4, [r0]
    cmp     r4, r1
    ldreq   r4, [r0, #4]
    cmpeq   r4, r2
    ldreq   r4, [r3]
    cmpeq   r4, r2
    ldreq   r4, [r3, #4]
    cmpeq   r4, r1
    bne     stage1_fail

    /* ---- first-level page table for the kernel's crt0 ----
       crt0.S only rewrites entries of a table at 0x2203c000 (VA 0 -> SRAM, unmapped holes) and turns the MMU on;
       it neither fills the SDRAM, ROM, SRAM and peripheral sections nor loads the table base. emCORE's S5L8702
       loader (loader/ipodclassic) does both; the ROM hands over with the MMU off and an unrelated table base.
       Identity map of all 4096 sections: 0xc1e (AP 3, cached, buffered) below 0x38000000, 0xc12 from there. */
    ldr     r0, =TTB_BASE
    mov     r1, r0
    ldr     r2, =0xc1e
    mov     r3, #0x1000
16: cmp     r2, #0x38000000
    bichs   r4, r2, #0xc
    movlo   r4, r2
    str     r4, [r1], #4
    add     r2, r2, #0x100000
    subs    r3, r3, #1
    bne     16b
    mcr     p15, 0, r0, c2, c0, 0       /* translation table base */
    mov     r1, #0
    mcr     p15, 0, r1, c8, c7, 0       /* invalidate TLBs */

    /* ---- copy {u32 length, kernel} from _stage1_end to 0x08000000 and enter it ---- */
    mov     r7, #5
    ldr     r1, stage1_end_off      /* position independent: _stage1_end relative to this code */
12: add     r1, pc, r1
    ldr     r2, [r1], #4                /* length, multiple of 4 */
    cmp     r2, #0
    beq     stage1_fail
    ldr     r3, =KERNEL_MAX
    cmp     r2, r3
    bhi     stage1_fail
    tst     r2, #3
    bne     stage1_fail
    mov     r0, #KERNEL_BASE
    mov     r3, r0
11: ldr     r4, [r1], #4
    str     r4, [r3], #4
    subs    r2, r2, #4
    bne     11b
    mov     r1, #0
    mcr     p15, 0, r1, c7, c5, 0       /* invalidate icache (caches are off, for the record) */
    bx      r0

stage1_fail:
    b       stage1_fail

stage1_end_off:
    .word   _stage1_end - (12b + 8)

    .align 2
gpio_pcon:
    .word   0xe322222f, 0xeeeeee00, 0x2332eeee, 0x3333e222
    .word   0xeee33333, 0x3ee0eeee, 0x0f00ee33, 0xeeeeeee0
    .word   0x22222222, 0x22222222, 0x33322222, 0xeeeeeeee
    .word   0xeeeeeeee, 0xeeeeeeee, 0xee2222ee, 0xeeee0eee

miu_cmds:
    .word   0x04, 0x33                  /* no action */
    .word   0x04, 0x33
    .word   0x04, 0x233                 /* precharge all banks */
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x333                 /* auto refresh */
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x333                 /* auto refresh */
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x0c, 0x33                  /* MRS: burst length 8, CAS 3 */
    .word   0x04, 0x133                 /* mode register set */
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x0c, 0x8040                /* EMRS: strength 1/4, self refresh area full */
    .word   0x04, 0x133                 /* mode register set */
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0x04, 0x33
    .word   0, 0
    .ltorg
