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
; nano 3G (S5L8702) sequencer program: program P_COUNT pages, each with its
; 12 bytes of spare metadata.
;
; Parameter RAM (shared convention with nand-fmss.c):
;   P_ROWS    u32 rows[P_COUNT]: page number of each page
;   P_CELIST  u32 list[P_COUNT]: index into P_CEMAP per page
;   P_CEMAP   u32 cemap[8]: index -> CE number
;   P_COUNT   pages to program (>= 1)
;   P_META    in: 12 bytes per page
;   P_DMALIST u32 bus address per 2 KiB unit of each page, in programming order
;   P_CHUNKS  2 KiB units per page (1 for 2 KiB pages, 2 for 4 KiB pages)
;   P_FLAGS   bit 0: two-plane mode. Pages come in pairs on one CE: the first
;             is confirmed with 11h (queued), the second is set up with 81h and
;             confirmed with 10h. P_COUNT must be even then.
;   P_WTIMING FMC_CTRL0 timing field ORed into the control word
; Scratch: V_UNITS.
; A page goes out as, per 512-byte sector: 16 bytes of ECC and metadata, then
; 512 bytes of data. The controller fetches the next sector from memory while
; the current one goes to the chip; the first sector of the next page is
; fetched during the last sector of the current one. A chip is not waited for
; until it is given its next page; every chip used is checked at the end.
; r6 returns the mask of chips used, or on a NAND FAIL / fault: bit 28 |
; selected CE mask, plus bit 27 if the sequencer itself faulted.
;
; Registers: r3 CE bit of the page in progress, r4 sector index within a unit
; (0..3), r5 two-plane phase (0 first plane / single, 1 second plane), r6 chips
; used, r7 units still to program.
    wr   [CS_IRQ], #0xff
    wr   [FMSS_C10], #0x1
    wr   [FMSS_C58], #0x4
    wr   [FMSS_C4C], #fault
    wr   [FMCTRL1], #0xff3f8e0
    wr   [FMC_STATUS], #0xff00ffe
    movi r0, #0x801
    ld   r1, [P_WTIMING]
    or   r0, r1
    st   [FMC_CTRL0], r0
    movi r6, #0x0
    movi r5, #0x0
; ---- first page: chip, sector 0 into buffer 0, metadata, ECC of sector 0
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
    or   r6, r3
    ld   r1, [P_DMALIST]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_DMALIST], r1
    st   [FMC_DMA_ADDR], r2
    ld   r0, [FMC_CTRL0]
    or   r0, r0, #0x1000000         ; controller fetches data from memory itself
    st   [FMC_CTRL0], r0
    wr   [FMC_DNUM], #0x1ff
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMCTRL1], #0x2e0          ; memory -> buffer 0
    ld   r1, [P_META]
    ldi  r0, [r1]
    add  r1, r1, #0x4
    st   [FMC_META0], r0
    ldi  r0, [r1]
    add  r1, r1, #0x4
    st   [FMC_META1], r0
    ldi  r0, [r1]
    add  r1, r1, #0x4
    st   [FMC_META2], r0
    st   [P_META], r1
    wr   [FMC_METACFG], #0x3210
    wr   [FMC_METACTL], #0x1        ; encode the metadata into the ECC slices
meta_first:
    ld   r0, [FMC_METACTL], 0x1
    bnz  r0, meta_first
    wait 2                          ; sector 0 is in buffer 0
    wr   [FMC_STATUS], #0x100000
    wr   [ECC_DONE], #0x1ff
    wr   [ECC_MODE], #0x180
    wr   [ECC_KICK], #0x1102        ; encode sector 0 (buffer 0, slice 0)
    wr   [FMC_CMD], #0x80           ; PAGE PROGRAM setup
    wait 0
    wr   [FMC_STATUS], #0x2
    ld   r1, [P_ROWS]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_ROWS], r1
    wr   [FMC_ANUM], #0x4           ; five address cycles
    shl  r1, r2, #0x10
    st   [FMC_ADDR0], r1
    shr  r1, r2, #0x10
    st   [FMC_ADDR1], r1
    wr   [FMCTRL1], #0x1
    wait 1
    wr   [FMC_STATUS], #0x4
    ld   r0, [P_CHUNKS]
    st   [V_UNITS], r0
    wait 4
    wr   [ECC_DONE], #0x1
    movi r4, #0x0
; ---- sector k is in buffer k & 1, its ECC in slice k. Each step sends slice
;      k, then sends the data while the controller fetches sector k + 1 into
;      the other buffer, and encodes that sector.
sector:
    wr   [FMC_DNUM], #0xf
    wr   [FMC_DATA_SEL], #0x1000
    shl  r0, r4, #0x4
    st   [FMC_28], r0
    wr   [FMCTRL1], #0x34           ; ECC slice -> chip
    sub  r0, r4, #0x3
    bnz  r0, slice_sent
    ld   r1, [P_DMALIST]            ; next unit's (or page's) address
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_DMALIST], r1
    st   [FMC_DMA_ADDR], r2
slice_sent:
    wait 3
    wr   [FMC_STATUS], #0x8
    and  r1, r4, #0x1               ; b = k & 1
    shl  r0, r1, #0x8
    sub  r0, r1
    add  r0, r0, #0x102             ; 0x102 (send buffer 0) or 0x201 (send buffer 1)
    st   [FMC_DATA_SEL], r0
    wr   [FMC_DNUM], #0x1ff
    wr   [FMCTRL1], #0x2e4          ; buffer k -> chip, memory -> other buffer
    sub  r0, r4, #0x3
    bnz  r0, mid_page
    bz   r7, next_page              ; last sector of the page
mid_page:
    wait 2
    wr   [FMC_STATUS], #0x100000
    add  r2, r4, #0x1               ; encode sector (k + 1) & 3
    and  r2, r2, #0x3
    and  r1, r2, #0x1
    movi r0, #0x100
    shl  r0, r1
    shl  r2, r2, #0x10
    add  r2, r0
    add  r2, r2, #0x1002
    wr   [ECC_DONE], #0x1ff
    wr   [ECC_MODE], #0x180
    st   [ECC_KICK], r2
    wait 4
    wr   [ECC_DONE], #0x1
    wait 3
    wr   [FMC_STATUS], #0x8
    sub  r0, r4, #0x2
    bnz  r0, advance
    ld   r7, [V_UNITS]              ; sector 2 sent: one unit less to go
    sub  r7, r7, #0x1
    st   [V_UNITS], r7
    bnz  r7, advance
    ld   r0, [P_CHUNKS]             ; last unit of the page: count the page
    st   [V_UNITS], r0
    ld   r2, [P_COUNT]
    sub  r2, r2, #0x1
    st   [P_COUNT], r2
    bz   r2, last_page
advance:
    add  r4, r4, #0x1
    and  r4, r4, #0x3
    movi r0, #0x1
    bnz  r0, sector
; ---- last sector of a page that is not the last one: load the next page's
;      metadata, encode its sector 0, confirm this page, select the next chip
;      (checking it first if it is still busy) and set the next page up
next_page:
    ld   r1, [P_META]
    ldi  r0, [r1]
    add  r1, r1, #0x4
    st   [FMC_META0], r0
    ldi  r0, [r1]
    add  r1, r1, #0x4
    st   [FMC_META1], r0
    ldi  r0, [r1]
    add  r1, r1, #0x4
    st   [FMC_META2], r0
    st   [P_META], r1
    wr   [FMC_METACFG], #0x3210
    wr   [FMC_METACTL], #0x1
meta_next:
    ld   r0, [FMC_METACTL], 0x1
    bnz  r0, meta_next
    wait 2                          ; next page's sector 0 is in buffer 0
    wr   [FMC_STATUS], #0x100000
    wr   [ECC_DONE], #0x1ff
    wr   [ECC_MODE], #0x180
    wr   [ECC_KICK], #0x1102
    wait 3                          ; this page's last sector is out
    wr   [FMC_STATUS], #0x8
    ld   r0, [P_FLAGS]
    bz   r0, confirm_10
    bnz  r5, confirm_10
    wr   [FMC_CMD], #0x11           ; first plane of a pair: queue it
    movi r5, #0x1
    movi r0, #0x1
    bnz  r0, confirmed
confirm_10:
    wr   [FMC_CMD], #0x10           ; PAGE PROGRAM confirm
    movi r5, #0x0
confirmed:
    wait 0
    wr   [FMC_STATUS], #0x2
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
    mov  r0, r3
    and  r0, r6                     ; chip still programming?
    or   r6, r3
    ld   r1, [P_ROWS]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_ROWS], r1
    bz   r0, setup
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMC_CMD], #0x70
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMCTRL1], #0xca
    ld   r0, [FMC_NSTAT], 0x40
    bnz  r0, busy_done
    wait 5
busy_done:
    wr   [FMC_STATUS], #0x800000
    ld   r4, [FMC_NSTAT], 0x1
    wr   [FMCTRL1], #0x20
    bnz  r4, fault
setup:
    movi r0, #0x80                  ; 80h, or 81h for the second plane
    add  r0, r5
    st   [FMC_CMD], r0
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMC_ANUM], #0x4
    shl  r1, r2, #0x10
    st   [FMC_ADDR0], r1
    shr  r1, r2, #0x10
    st   [FMC_ADDR1], r1
    wr   [FMCTRL1], #0x1
    wait 1
    wr   [FMC_STATUS], #0x4
    wait 4
    wr   [ECC_DONE], #0x1
    movi r4, #0x0
    movi r0, #0x1
    bnz  r0, sector
; ---- last page: stop the controller's memory fetch, send the last slice and
;      sector without a following fetch, confirm, then check every chip used
last_page:
    ld   r0, [FMC_CTRL0], 0xfeffffff
    st   [FMC_CTRL0], r0
    wr   [FMC_DNUM], #0xf
    wr   [FMC_DATA_SEL], #0x1000
    wr   [FMC_28], #0x30
    wr   [FMCTRL1], #0x34
    wait 3
    wr   [FMC_STATUS], #0x8
    wr   [FMC_DMA_CTRL], #0x7
    wr   [FMC_DATA_SEL], #0x200
    wr   [FMC_DNUM], #0x1ff
    wr   [FMCTRL1], #0xe4           ; buffer 1 -> chip
    wait 3
    wr   [FMC_STATUS], #0x8
    wr   [FMC_CMD], #0x10
    wait 0
    wr   [FMC_STATUS], #0x2
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
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMC_CMD], #0x70
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMCTRL1], #0xca
    ld   r0, [FMC_NSTAT], 0x40
    bnz  r0, sweep_ready
    wait 5
sweep_ready:
    wr   [FMC_STATUS], #0x800000
    ld   r4, [FMC_NSTAT], 0x1
    wr   [FMCTRL1], #0x20
    bnz  r4, fault
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
