@ On-device SDRAM test routine for the nano4g emCORE harness (round 13). Position independent, ARM mode, runs as
@ an emCORE system thread: r0 = parameter block (see n4g_memtest2.py PARAMS). Writes 4 patterns over [base, base+size),
@ clean+invalidates the D-cache between the write and the read pass, counts mismatches, records the first one, updates a
@ progress word every 64 KiB, sets done=1 at the end and returns (the thread exits). No USB traffic while it runs.
    .syntax unified
    .arm
    .global memtest
memtest:
    push    {r4-r11, lr}
    mov     r9, r0
    mov     r10, #0                 @ pass index
pass_loop:
    ldr     r4, [r9, #0]            @ base
    ldr     r5, [r9, #4]            @ size
    add     r5, r4, r5              @ end
    cmp     r10, #0
    ldreq   r6, =0x5aa5f00f
    moveq   r7, #0                  @ pass 0: value = addr ^ key
    movne   r7, #1
    cmp     r10, #1
    moveq   r6, #0
    cmp     r10, #2
    mvneq   r6, #0
    cmp     r10, #3
    ldreq   r6, =0xa5a5a5a5
    mov     r1, r4
w_loop:
    cmp     r7, #0
    eoreq   r2, r1, r6
    movne   r2, r6
    str     r2, [r1], #4
    cmp     r1, r5
    blo     w_loop
    mov     r0, #0
    mcr     p15, 0, r0, c7, c14, 0  @ clean + invalidate entire D-cache
    mcr     p15, 0, r0, c7, c10, 4  @ drain write buffer
    mov     r1, r4
    mov     r8, #0                  @ bad words in this pass
r_loop:
    cmp     r7, #0
    eoreq   r2, r1, r6
    movne   r2, r6
    ldr     r3, [r1]
    cmp     r3, r2
    beq     r_ok
    add     r8, r8, #1
    ldr     r0, [r9, #16]
    cmp     r0, #0
    bne     r_count
    str     r1, [r9, #20]           @ first bad address
    str     r2, [r9, #24]           @ expected
    str     r3, [r9, #28]           @ got
r_count:
    add     r0, r0, #1
    str     r0, [r9, #16]           @ bad total
r_ok:
    add     r1, r1, #4
    tst     r1, #0xff00
    bne     r_next
    tst     r1, #0xff
    bne     r_next
    sub     r0, r1, r4
    str     r0, [r9, #12]           @ progress (bytes read back in this pass)
r_next:
    cmp     r1, r5
    blo     r_loop
    add     r0, r9, r10, lsl #2
    str     r8, [r0, #36]           @ per-pass bad count
    add     r10, r10, #1
    str     r10, [r9, #52]          @ passes completed
    cmp     r10, #4
    blo     pass_loop
    mov     r0, #1
    str     r0, [r9, #32]           @ done
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 0  @ clean D-cache so the host sees the results
    mcr     p15, 0, r0, c7, c10, 4
    pop     {r4-r11, pc}
    .ltorg
