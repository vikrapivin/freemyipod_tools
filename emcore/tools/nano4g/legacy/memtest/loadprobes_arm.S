@ Three tiny position-independent ARM routines for n4g_ladder.py (round 14). r0 = param block:
@   +0 base, +4 size (bytes) or count, +8 passes, +12 result (sum), +32 done flag. Each returns to lr (= thread_exit).
    .syntax unified
    .arm
    .global cpuload
cpuload:                            @ pure CPU: count [r0,#4] down to zero, no memory traffic in the loop
    ldr     r1, [r0, #4]
1:  subs    r1, r1, #1
    bne     1b
    mov     r1, #1
    str     r1, [r0, #32]
    bx      lr
    .global cpuload_end
cpuload_end:

    .global readsweep
readsweep:                          @ read [base, base+size) 'passes' times, sum the words into +12
    push    {r4, lr}
    ldr     r4, [r0, #8]
    mov     r3, #0
2:  ldr     r1, [r0, #0]
    ldr     r2, [r0, #4]
    add     r2, r1, r2
3:  ldr     ip, [r1], #4
    add     r3, r3, ip
    cmp     r1, r2
    blo     3b
    subs    r4, r4, #1
    bne     2b
    str     r3, [r0, #12]
    mov     r1, #1
    str     r1, [r0, #32]
    pop     {r4, pc}
    .global readsweep_end
readsweep_end:

    .global writesweep
writesweep:                         @ write addr^0x5aa5f00f over [base, base+size) 'passes' times, no cache op
    push    {r4, r5, lr}
    ldr     r4, [r0, #8]
    ldr     r5, key
4:  ldr     r1, [r0, #0]
    ldr     r2, [r0, #4]
    add     r2, r1, r2
5:  eor     r3, r1, r5
    str     r3, [r1], #4
    cmp     r1, r2
    blo     5b
    subs    r4, r4, #1
    bne     4b
    mov     r1, #1
    str     r1, [r0, #32]
    pop     {r4, r5, pc}
key:
    .word   0x5aa5f00f
    .global writesweep_end
writesweep_end:

    .global writeslow
writeslow:                          @ like writesweep, but after every 32-B line spin [r0,#16] iterations (throttled rate)
    push    {r4, r5, r6, lr}
    ldr     r4, [r0, #8]
    ldr     r5, key2
    ldr     ip, [r0, #20]           @ mode: 0 = addr^key, 1 = constant value from +24
    cmp     ip, #0
    ldrne   r5, [r0, #24]
6:  ldr     r1, [r0, #0]
    ldr     r2, [r0, #4]
    add     r2, r1, r2
7:  cmp     ip, #0
    eoreq   r3, r1, r5              @ mode 0: addr ^ key
    movne   r3, r5                  @ mode 1: constant
    str     r3, [r1], #4
    tst     r1, #0x1c
    bne     8f
    ldr     r6, [r0, #16]
    cmp     r6, #0
    beq     8f                      @ throttle 0 = no spin (a count-down from 0 would spin 2^32 times)
9:  subs    r6, r6, #1
    bne     9b
8:  cmp     r1, r2
    blo     7b
    subs    r4, r4, #1
    bne     6b
    mov     r1, #1
    str     r1, [r0, #32]
    pop     {r4, r5, r6, pc}
key2:
    .word   0x5aa5f00f
    .global writeslow_end
writeslow_end:
