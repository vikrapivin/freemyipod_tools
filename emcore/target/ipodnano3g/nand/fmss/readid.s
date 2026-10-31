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
; nano 3G (S5L8702) sequencer program: RESET every chip enable, then READ ID.
;
; Parameter RAM (shared convention with nand-fmss.c):
;   P_IDBUF  out: u32 ids[P_NIDS], ID bytes 0..3 (byte 0 = maker) of each chip
;   P_NIDS   number of chip enables to probe
;   P_CELIST u32 CE numbers (0..7) to probe, in output order
;   P_DELAY  loop count that waits out tRST after the broadcast RESET (>= 1)
; The program resets all eight CEs itself (the CPU does not), so it doubles as
; the "reset" operation. Bytes 4..7 of the ID answer are read and dropped.
; r6 returns 0, or 0x08000000 | selected CE mask after a sequencer fault.
    wr   [CS_IRQ], #0xff            ; clear sequencer interrupts
    wr   [FMSS_C10], #0x1           ; sequencer enable (inf)
    wr   [FMSS_C58], #0x4           ; event timeout scale (inf)
    wr   [FMSS_C4C], #fault         ; fault vector
    wr   [FMCTRL1], #0xff3f8e0      ; transfer engine idle setup
    wr   [FMC_STATUS], #0xff00ffe   ; clear FMC status
    ld   r7, [P_IDBUF]              ; r7 = output pointer
    ld   r6, [P_NIDS]               ; r6 = chips left
    ld   r5, [P_CELIST]             ; r5 = CE list pointer
; ---- RESET all CEs at once with slow timing
    wr   [FMC_CTRL0], #0x33201
    wr   [FMC_DATA_SEL], #0x0
    movi r0, #0x1fe                 ; CE bits 1..8
    ld   r1, [FMC_CTRL0], 0xfffffe01
    or   r1, r0
    st   [FMC_CTRL0], r1
    wr   [FMC_CMD], #0xff           ; RESET
    wait 0                          ; command sent
    wr   [FMC_STATUS], #0x2
; ---- wait out tRST: two instructions per count, then deselect
    ld   r1, [P_DELAY]
settle:
    sub  r1, r1, #1
    bnz  r1, settle
    ld   r1, [FMC_CTRL0], 0xfffffe01
    st   [FMC_CTRL0], r1
next_ce:
    wr   [FMC_DATA_SEL], #0x0
    ldi  r2, [r5]                   ; CE number
    movi r3, #0x2
    shl  r3, r2                     ; CE bit in FMC_CTRL0
    ld   r1, [FMC_CTRL0], 0xfffffe01
    or   r1, r3
    st   [FMC_CTRL0], r1            ; select the CE
    wr   [FMC_CMD], #0x90           ; READ ID
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMC_ANUM], #0x0           ; one address cycle
    wr   [FMC_ADDR0], #0x0
    wr   [FMCTRL1], #0x1            ; send address
    wait 1
    wr   [FMC_STATUS], #0x4
    wr   [FMC_DNUM], #0x7           ; 8 answer bytes into the FIFO
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMCTRL1], #0x1c2
    wait 3
    wr   [FMC_STATUS], #0x8
    wr   [FMC_DATA_SEL], #0x100
    wr   [FMCTRL1], #0x340          ; FIFO read mode
    ld   r0, [FMC_FIFO]             ; ID bytes 0..3
    sti  [r7], r0
    ld   r0, [FMC_FIFO]             ; ID bytes 4..7, dropped
    ld   r1, [FMC_CTRL0], 0xfffffe01
    st   [FMC_CTRL0], r1            ; deselect
    add  r5, r5, #0x4
    add  r7, r7, #0x4
    sub  r6, r6, #0x1
    bnz  r6, next_ce
    end
fault:
    ld   r2, [FMSS_C3C]
    ld   r0, [FMC_CTRL0], 0x1fe
    shr  r0, r0, #0x1               ; selected CE mask
    and  r6, r6, #0xffffff00
    or   r6, r0
    wr   [FMSS_C50], #0x1           ; acknowledge the fault flag
    or   r6, r6, #0x8000000         ; controller fault
    end
