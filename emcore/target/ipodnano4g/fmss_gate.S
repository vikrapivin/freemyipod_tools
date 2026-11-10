/* SPDX-License-Identifier: GPL-2.0-or-later
 * Clock pulse with no stack, literal, SDRAM or peripheral reads while gated.
 * The caller masks IRQ/FIQ and supplies five saved PWRCON words and masks.
 * No clock identities are assumed. A gate which stops ARM itself still
 * requires external recovery; software cannot promise to restore that gate.
 */
    .section .icode,"ax",%progbits
    .arm
    .balign 256
    .global fmss_gate_pulse
    .type fmss_gate_pulse,%function
fmss_gate_pulse:
    stmdb sp!, {r4-r11, lr}
    ldmia r0, {r3-r7}
    ldmia r1, {r8-r12}
    orr r8, r8, r3
    orr r9, r9, r4
    orr r10, r10, r5
    orr r11, r11, r6
    orr r12, r12, r7
    mov r0, #0x3c000000
    orr r0, r0, #0x00500000
    cmp r2, #0
    moveq r2, #1
    /* All inputs are resident in registers before the first disable. */
    str r8, [r0, #0x48]
    str r9, [r0, #0x4c]
    str r10, [r0, #0x58]
    str r11, [r0, #0x68]
    str r12, [r0, #0x6c]
1:  subs r2, r2, #1
    bne 1b
    str r3, [r0, #0x48]
    str r4, [r0, #0x4c]
    str r5, [r0, #0x58]
    str r6, [r0, #0x68]
    str r7, [r0, #0x6c]
    mov r0, #0
    mcr p15, 0, r0, c7, c10, 4
    ldmia sp!, {r4-r11, pc}
    .size fmss_gate_pulse, .-fmss_gate_pulse
