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
; nano 3G (S5L8702) sequencer program: read P_COUNT pages with per-sector ECC
; decode, DMA and spare metadata decode.
;
; Parameter RAM (shared convention with nand-fmss.c):
;   P_ROWS    u32 rows[P_COUNT]: page number of each page
;   P_CELIST  u32 list[P_COUNT]: index into P_CEMAP per page
;   P_CEMAP   u32 cemap[8]: index -> CE number
;   P_COUNT   pages to read (>= 1)
;   P_META    out: 12 bytes per page (the decoded spare metadata words)
;   P_DMALIST u32 bus address per 2 KiB unit of each page, in reading order
;   P_RESULTS out: one u32 per page: bit 30 uncorrectable, bit 29 no error
;             flagged by the engine (erased or clean), bit n-1 set when some
;             512-byte sector needed n corrections
;   P_CHUNKS  2 KiB units per page (1 for 2 KiB pages, 2 for 4 KiB pages)
;   P_TIMING  FMC_CTRL0 timing field ORed into the control word
; Scratch: V_QCE (CE list walk of the second pass), V_UNITS, V_QLEN.
; Pages are taken in batches: the longest run of entries whose CEs are all
; different. Every page of a batch has its load (00h, address, 30h) started
; first, so the chips read in parallel; then the pages are transferred in
; order, the next page's first sector being fetched while the current page's
; last sector is decoded and moved to memory.
; r6 returns the OR of all page results, or on a fault the selected CE mask
; | 0x08000000.
;
; Registers: r4 sector index within a 2 KiB unit (0..3), r5 result of the
; page in progress, r6 OR of the page results, r7 units still to read.
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
    movi r6, #0x0
batch:
; ---- pass 1: start the page load of every entry up to the first CE repeat
    movi r2, #0x0                   ; CE bits already loading
    movi r3, #0x0                   ; pages started
    ld   r0, [P_CELIST]
    st   [V_QCE], r0                ; pass 2 walks the same list again
load_next:
    ld   r4, [P_COUNT]
    bz   r4, loads_done
    ld   r1, [P_CELIST]
    ldi  r7, [r1]
    shl  r7, r7, #0x2
    ld   r0, [P_CEMAP]
    add  r0, r7
    ldi  r7, [r0]
    movi r0, #0x1
    shl  r0, r7                     ; r0 = CE bit
    mov  r1, r0
    and  r1, r2
    bnz  r1, loads_done             ; CE busy: the entry starts the next batch
    or   r2, r0
    ld   r1, [P_CELIST]
    add  r1, r1, #0x4
    st   [P_CELIST], r1
    sub  r4, r4, #0x1
    st   [P_COUNT], r4
    add  r3, r3, #0x1
    shl  r1, r0, #0x1
    ld   r7, [FMC_CTRL0], 0xfffffe01
    or   r7, r1
    st   [FMC_CTRL0], r7            ; select the CE
    wr   [FMC_CMD], #0x0
    wait 0
    wr   [FMC_STATUS], #0x2
    ld   r1, [P_ROWS]
    ldi  r7, [r1]
    add  r1, r1, #0x4
    st   [P_ROWS], r1
    wr   [FMC_ANUM], #0x4           ; five address cycles
    shl  r1, r7, #0x10              ; column 0, row low bits
    st   [FMC_ADDR0], r1
    shr  r1, r7, #0x10
    st   [FMC_ADDR1], r1
    wr   [FMCTRL1], #0x1
    wait 1
    wr   [FMC_STATUS], #0x4
    wr   [FMC_CMD], #0x30           ; READ confirm: the chip loads the page
    wait 0
    wr   [FMC_STATUS], #0x2
    movi r0, #0x1
    bnz  r0, load_next
loads_done:
; ---- pass 2: transfer. First page of the batch: wait until its chip is ready,
;      back to data output, read sector 0 into buffer 0.
    st   [V_QLEN], r3
    movi r5, #0x0
    ld   r1, [V_QCE]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [V_QCE], r1
    shl  r2, r2, #0x2
    ld   r0, [P_CEMAP]
    add  r0, r2
    ldi  r2, [r0]
    movi r3, #0x2
    shl  r3, r2
    ld   r1, [FMC_CTRL0], 0xfffffe01
    or   r1, r3
    st   [FMC_CTRL0], r1
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMC_CMD], #0x70           ; READ STATUS
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMCTRL1], #0xca           ; latch the status byte
    ld   r0, [FMC_NSTAT], 0x40
    bnz  r0, first_ready
    wait 5                          ; not ready yet: wait for the ready event
first_ready:
    wr   [FMC_STATUS], #0x800000
    wr   [FMCTRL1], #0x20
    wr   [FMCTRL1], #0x100e0
    wr   [FMC_CMD], #0x0            ; back to data output
    wait 0
    wr   [FMC_STATUS], #0x2
    ld   r0, [P_CHUNKS]
    st   [V_UNITS], r0
    wr   [FMC_DNUM], #0xf           ; 16 bytes of ECC and spare
    wr   [FMC_DATA_SEL], #0x10
    wr   [FMC_24], #0x0
    wr   [FMCTRL1], #0x32
    wait 3
    wr   [FMC_STATUS], #0x8
    wr   [FMC_DNUM], #0x1ff         ; 512 bytes of data into buffer 0
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMCTRL1], #0xe2
    wait 3
    wr   [FMC_STATUS], #0x8
    ld   r0, [FMC_CTRL0]
    or   r0, r0, #0x1000000         ; controller moves data to memory itself
    st   [FMC_CTRL0], r0
    ld   r1, [P_DMALIST]
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_DMALIST], r1
    st   [FMC_DMA_ADDR], r2
    movi r4, #0x0
; ---- sector k is in buffer k & 1 with its ECC in parity slice k. Each step
;      fetches the next sector's ECC (slice (k+1) & 3) and data into the other
;      buffer while sector k is decoded and moved to memory. The step of
;      sector 3 fetches sector 0 of the next unit of the page.
sector:
    sub  r0, r4, #0x3
    bnz  r0, step
    bnz  r7, step                   ; more units: sector 3 is an ordinary step
    movi r0, #0x1                   ; last unit: page end
    bnz  r0, page_end
step:
    add  r0, r4, #0x1
    and  r0, r0, #0x3
    shl  r0, r0, #0x4               ; ECC slice offset of sector k + 1
    wr   [FMC_DNUM], #0xf
    wr   [FMC_DATA_SEL], #0x10
    st   [FMC_24], r0
    wr   [FMCTRL1], #0x32
    wait 3
    wr   [FMC_STATUS], #0x8
    and  r1, r4, #0x1               ; b = k & 1
    shl  r0, r1, #0x8
    sub  r0, r1
    add  r0, r0, #0x102             ; 0x102 (fill buffer 1) or 0x201 (fill buffer 0)
    st   [FMC_DATA_SEL], r0
    wr   [FMC_DNUM], #0x1ff
    wr   [FMCTRL1], #0xe2
    shl  r2, r4, #0x10              ; decode: slice k, buffer b
    movi r0, #0x100
    shl  r0, r1
    or   r2, r0
    or   r2, r2, #0x1001
    wr   [ECC_DONE], #0x1ff
    wr   [ECC_MODE], #0x180
    st   [ECC_KICK], r2
    wait 4
    wr   [ECC_DONE], #0x1
    ld   r1, [ECC_STATUS]
    and  r0, r1, #0x1
    bz   r0, count_a
    or   r5, r5, #0x40000000        ; uncorrectable
    bnz  r0, counted_a
count_a:
    shr  r1, r1, #0x10
    and  r1, r1, #0xf               ; corrections in this sector
    movi r0, #0x1
    shl  r0, r1
    shr  r1, r0, #0x1               ; (1 << n) >> 1 = bit n-1
    or   r5, r1
counted_a:
    wr   [FMCTRL1], #0x100          ; buffer k -> memory
    wait 2
    wr   [FMC_STATUS], #0x100000
    sub  r0, r4, #0x3
    bnz  r0, no_new_unit
    ld   r1, [P_DMALIST]            ; next unit's address
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_DMALIST], r1
    st   [FMC_DMA_ADDR], r2
no_new_unit:
    wait 3
    wr   [FMC_STATUS], #0x8
    sub  r0, r4, #0x2
    bnz  r0, advance
    ld   r7, [V_UNITS]              ; sector 2 done: one unit less to go
    sub  r7, r7, #0x1
    st   [V_UNITS], r7
advance:
    add  r4, r4, #0x1
    and  r4, r4, #0x3
    movi r0, #0x1
    bnz  r0, sector
; ---- last sector of the page, another page of the batch follows: decode it
;      while that page's chip is polled, collect the metadata, then start the
;      next page's sector 0
page_end:
    ld   r0, [P_CHUNKS]
    st   [V_UNITS], r0
    ld   r2, [V_QLEN]
    sub  r2, r2, #0x1
    st   [V_QLEN], r2
    bz   r2, last_page
    ld   r0, [FMC_STATUS], 0x8000000
    wr   [FMC_STATUS], #0x8000000   ; engine flagged errors on this page?
    bnz  r0, flagged_b
    movi r5, #0x20000000            ; no: erased or clean
flagged_b:
    wr   [ECC_DONE], #0x1ff
    wr   [ECC_MODE], #0x180
    wr   [ECC_KICK], #0x31201       ; decode sector 3 (buffer 1, slice 3)
    ld   r1, [V_QCE]                ; next page's chip
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [V_QCE], r1
    shl  r2, r2, #0x2
    ld   r0, [P_CEMAP]
    add  r0, r2
    ldi  r2, [r0]
    movi r3, #0x2
    shl  r3, r2
    ld   r1, [FMC_CTRL0], 0xfffffe01
    or   r1, r3
    st   [FMC_CTRL0], r1
    wr   [FMC_DATA_SEL], #0x1
    wr   [FMC_CMD], #0x70
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMCTRL1], #0xca
    ld   r0, [FMC_NSTAT], 0x40
    bnz  r0, next_ready
    wait 5
next_ready:
    wr   [FMC_STATUS], #0x800000
    wr   [FMCTRL1], #0x20
    wait 4
    wr   [ECC_DONE], #0x1
    ld   r1, [ECC_STATUS]
    and  r0, r1, #0x1
    bz   r0, count_b
    or   r5, r5, #0x40000000
    bnz  r0, counted_b
count_b:
    shr  r1, r1, #0x10
    and  r1, r1, #0xf
    movi r0, #0x1
    shl  r0, r1
    shr  r1, r0, #0x1
    or   r5, r1
counted_b:
    wr   [FMC_METACFG], #0x3210     ; decode the metadata of the page
    wr   [FMC_METACTL], #0x2
meta_b:
    ld   r0, [FMC_METACTL], 0x2
    bnz  r0, meta_b
    ld   r1, [P_META]
    ld   r0, [FMC_META0]
    sti  [r1], r0
    add  r1, r1, #0x4
    ld   r0, [FMC_META1]
    sti  [r1], r0
    add  r1, r1, #0x4
    ld   r0, [FMC_META2]
    sti  [r1], r0
    add  r1, r1, #0x4
    st   [P_META], r1
    wr   [FMCTRL1], #0x100e0        ; next page: back to data output, sector 0
    wr   [FMC_CMD], #0x0
    wait 0
    wr   [FMC_STATUS], #0x2
    wr   [FMC_DNUM], #0xf
    wr   [FMC_DATA_SEL], #0x10
    wr   [FMC_24], #0x0
    wr   [FMCTRL1], #0x32
    wait 3
    wr   [FMC_STATUS], #0x8
    wr   [FMC_DATA_SEL], #0x201
    wr   [FMC_DNUM], #0x1ff
    wr   [FMCTRL1], #0xe2
    ld   r0, [P_RESULTS]
    sti  [r0], r5
    add  r0, r0, #0x4
    st   [P_RESULTS], r0
    or   r6, r5
    movi r5, #0x0
    wr   [FMCTRL1], #0x100          ; last sector of the page -> memory
    wait 2
    wr   [FMC_STATUS], #0x100000
    ld   r1, [P_DMALIST]            ; next page's address
    ldi  r2, [r1]
    add  r1, r1, #0x4
    st   [P_DMALIST], r1
    st   [FMC_DMA_ADDR], r2
    wait 3
    wr   [FMC_STATUS], #0x8
    movi r4, #0x0
    movi r0, #0x1
    bnz  r0, sector
; ---- last page of the batch
last_page:
    ld   r0, [FMC_STATUS], 0x8000000
    wr   [FMC_STATUS], #0x8000000
    bnz  r0, flagged_c
    movi r5, #0x20000000
flagged_c:
    wr   [ECC_DONE], #0x1ff
    wr   [ECC_MODE], #0x180
    wr   [ECC_KICK], #0x31201
    wait 4
    wr   [ECC_DONE], #0x1
    ld   r1, [ECC_STATUS]
    and  r0, r1, #0x1
    bz   r0, count_c
    or   r5, r5, #0x40000000
    bnz  r0, counted_c
count_c:
    shr  r1, r1, #0x10
    and  r1, r1, #0xf
    movi r0, #0x1
    shl  r0, r1
    shr  r1, r0, #0x1
    or   r5, r1
counted_c:
    wr   [FMC_DMA_CTRL], #0x7       ; last transfer of the batch
    wr   [FMC_DATA_SEL], #0x200
    wr   [FMC_DNUM], #0x1ff
    wr   [FMCTRL1], #0x1e0
    wait 2
    wr   [FMC_STATUS], #0x100000
    wr   [FMC_METACFG], #0x3210
    wr   [FMC_METACTL], #0x2
meta_c:
    ld   r0, [FMC_METACTL], 0x2
    bnz  r0, meta_c
    ld   r1, [P_META]
    ld   r0, [FMC_META0]
    sti  [r1], r0
    add  r1, r1, #0x4
    ld   r0, [FMC_META1]
    sti  [r1], r0
    add  r1, r1, #0x4
    ld   r0, [FMC_META2]
    sti  [r1], r0
    add  r1, r1, #0x4
    st   [P_META], r1
    ld   r0, [P_RESULTS]
    sti  [r0], r5
    add  r0, r0, #0x4
    st   [P_RESULTS], r0
    or   r6, r5
    ld   r0, [FMC_CTRL0], 0xfeffffff
    st   [FMC_CTRL0], r0            ; controller DMA off
    ld   r1, [FMC_CTRL0], 0xfffffe01
    st   [FMC_CTRL0], r1            ; deselect
    ld   r0, [P_COUNT]
    bnz  r0, batch
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
