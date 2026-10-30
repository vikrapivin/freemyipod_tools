;             __________               __   ___.
;   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
;   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
;   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
;   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
;                     \/            \/     \/    \/            \/
;
; Copyright (C) 2026 Viktor Krapivin
;
; This program is free software; you can redistribute it and/or
; modify it under the terms of the GNU General Public License
; as published by the Free Software Foundation; either version 2
; of the License, or (at your option) any later version.
;
; This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
; KIND, either express or implied.
;
;
; nano 3G (S5L8702) sequencer program: erase P_COUNT blocks.
;
; Parameter RAM (shared convention with nand-fmss.c):
;   P_ROWS   u32 rows[P_COUNT]: first page of each block (block * pages/block)
;   P_CELIST u32 list[P_COUNT]: index into P_CEMAP per block
;   P_CEMAP  u32 cemap[8]: index -> CE number
;   P_COUNT  blocks to erase (>= 1)
;   P_TIMING FMC_CTRL0 timing field ORed into the control word
; Every block gets 60h, three row address cycles, D0h. A chip is not waited
; for until it is given its next block (several CEs erase in parallel); every
; chip used is checked once more at the end, CE 0..7 in order.
; r6 returns 0 (success), or on a NAND FAIL / fault: bit 28 | selected CE mask,
; plus bit 27 if the sequencer itself faulted.
    wr   [CS_IRQ], #0xff
    wr   [FMSS_C10], #0x1
    wr   [FMSS_C58], #0x4
    wr   [FMSS_C4C], #fault
    wr   [FMCTRL1], #0xff3f8e0
    wr   [FMC_STATUS], #0xff00ffe
    movi r0, #0x801
    ld   r1, [P_TIMING]
    or   r0, r1
    st   [FMC_CTRL0], r0
    movi r6, #0x0                   ; CEs with an erase in flight
next_block:
; ---- r3 = CE bit (1 << CE) of this block, CE selected in FMC_CTRL0
    ld   r1, [P_CELIST]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_CELIST], r1
    shl  r2, r2, #0x2
    ld   r0, [P_CEMAP]
    add  r0, r2
    ldi  r2, [r0]
    movi r3, #0x1
    shl  r3, r2
    shl  r0, r3, #0x1
    ld   r1, [FMC_CTRL0], 0xfffffe01
    or   r1, r0
    st   [FMC_CTRL0], r1
    mov  r2, r3
    and  r2, r6
    bz   r2, issue                  ; chip idle
    movi r5, #0x0                   ; check, then continue at issue
    movi r0, #0x1
    bnz  r0, check
issue:
    or   r6, r3
    wr   [FMC_CMD], #0x60           ; BLOCK ERASE setup
    wait 0
    wr   [FMC_STATUS], #0x2
    ld   r1, [P_ROWS]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_ROWS], r1
    wr   [FMC_ANUM], #0x2           ; three row address cycles
    st   [FMC_ADDR0], r2
    wr   [FMCTRL1], #0x1
    wait 1
    wr   [FMC_STATUS], #0x4
    wr   [FMC_CMD], #0xd0           ; BLOCK ERASE confirm
    wait 0
    wr   [FMC_STATUS], #0x2
    ld   r2, [P_COUNT]
    sub  r2, r2, #0x1
    st   [P_COUNT], r2
    bnz  r2, next_block
; ---- final sweep over the chips used
    movi r2, #0x1                   ; CE bit
    movi r7, #0x8                   ; CEs left
sweep:
    mov  r0, r2
    and  r0, r6
    bz   r0, sweep_next
    shl  r0, r2, #0x1
    ld   r1, [FMC_CTRL0], 0xfffffe01
    or   r1, r0
    st   [FMC_CTRL0], r1
    movi r5, #0x1                   ; check, then continue at sweep_next
    movi r0, #0x1
    bnz  r0, check
sweep_next:
    sub  r7, r7, #0x1
    bz   r7, finish
    shl  r2, r2, #0x1
    movi r0, #0x1
    bnz  r0, sweep
finish:
    ld   r1, [FMC_CTRL0], 0xfffffe01
    st   [FMC_CTRL0], r1            ; deselect
    end
; ---- READ STATUS of the selected chip until ready, then test FAIL. An erase
;      is slow: the first two ready waits run with the event timeout scale 0
;      and are retried, the third is the fatal one.
check:
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMC_CMD], #0x70
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMCTRL1], #0xca           ; latch the status byte
    ld   r0, [FMC_NSTAT], 0x40
    bnz  r0, ready
    movi r4, #0x2
    wr   [FMSS_C58], #0x0
retry:
    wait 5
    wr   [FMSS_C14], #0xff
    wr   [CS_IRQ], #0x4
    ld   r0, [FMC_NSTAT], 0x40
    bnz  r0, ready
    sub  r4, r4, #0x1
    bnz  r4, retry
    wr   [FMSS_C58], #0x4
    wait 5
ready:
    wr   [FMSS_C58], #0x4
    wr   [FMC_STATUS], #0x800000
    ld   r4, [FMC_NSTAT], 0x1       ; FAIL
    wr   [FMCTRL1], #0x20
    bnz  r4, fault
    bz   r5, issue
    movi r0, #0x1
    bnz  r0, sweep_next
fault:
    ld   r2, [FMSS_C3C]
    ld   r0, [FMC_CTRL0], 0x1fe
    shr  r0, r0, #0x1               ; selected CE mask
    and  r6, r6, #0xffffff00
    or   r6, r0
    ld   r0, [FMSS_C50], 0x1
    bz   r0, nofault
    wr   [FMSS_C50], #0x1           ; acknowledge the fault flag
    or   r6, r6, #0x8000000         ; controller fault
nofault:
    ld   r1, [FMC_CTRL0], 0xfffffe01
    st   [FMC_CTRL0], r1            ; deselect
    or   r6, r6, #0x10000000        ; failed
    end
