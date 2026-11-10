; restore OS FIRMWARE.x1243 program WRITE: file+0x386cc..0x3950c = firmware_body+0x380cc = mem:0x080379e8 
; FMSS microcode, 456 instructions, assembled with tools/fmssasm.c
    wr   [CS_IRQ], #0xff                     ; 0000
    wr   [FMSS_C10], #0x1                    ; 0008
    wr   [FMSS_C58], #0x4                    ; 0010
    wr   [FMSS_C4C], #0xdd8                  ; 0018
    wr   [FMCTRL1], #0x33338e0               ; 0020
    wr   [FMC_STATUS], #0xff00ffe            ; 0028
    ld   r0, [P_WTIMING], 0xffffffff         ; 0030
    or   r0, r0, #0x801                      ; 0038
    st   [FMC_CTRL0], r0                     ; 0040
    ld   r0, [P_DMALIST], 0xffffffff         ; 0048
    ldi  r1, [r0]                            ; 0050
    st   [FMC_DMA_ADDR], r1                  ; 0058
    add  r0, r0, #0x4                        ; 0060
    st   [P_DMALIST], r0                     ; 0068
    ld   r0, [FMC_CTRL0], 0xffffffff         ; 0070
    or   r0, r0, #0x1000000                  ; 0078
    st   [FMC_CTRL0], r0                     ; 0080
    wr   [FMC_DMA_CTRL], #0x7                ; 0088
    wr   [FMC_DNUM], #0x1ff                  ; 0090
    wr   [FMC_18], #0x1                      ; 0098
    wr   [FMC_DATA_SEL], #0x1                ; 00a0
    wr   [FMCTRL1], #0x2e0                   ; 00a8
    ld   r0, [P_META], 0xffffffff            ; 00b0
    ldi  r1, [r0]                            ; 00b8
    st   [FMC_DATA0?], r1                    ; 00c0
    add  r0, r0, #0x4                        ; 00c8
    ldi  r1, [r0]                            ; 00d0
    st   [FMC_64], r1                        ; 00d8
    add  r0, r0, #0x4                        ; 00e0
    ldi  r1, [r0]                            ; 00e8
    st   [FMC_68], r1                        ; 00f0
    add  r0, r0, #0x4                        ; 00f8
    wr   [FMC_40], #0xc1                     ; 0100
L_0108:
    ld   r1, [FMC_40], 0x1                   ; 0108
    bnz  r1, L_0108                          ; 0110
    st   [P_META], r0                        ; 0118
    ld   r0, [P_CEMASKS], 0xffffffff         ; 0120
    ldi  r7, [r0]                            ; 0128
    add  r0, r0, #0x4                        ; 0130
    ldi  r2, [r0]                            ; 0138
    add  r0, r0, #0x4                        ; 0140
    st   [P_CEMASKS], r0                     ; 0148
    wr   [FMC_ANUM], #0x4                    ; 0150
    shl  r0, r2, #0x10                       ; 0158
    st   [FMC_ADDR0], r0                     ; 0160
    shr  r0, r2, #0x10                       ; 0168
    st   [FMC_ADDR1], r0                     ; 0170
    and  r3, r7, #0xff                       ; 0178
    shl  r3, r3, #0x1                        ; 0180
    mov  r6, r3                              ; 0188
    ld   r4, [FMC_CTRL0], 0xfffffe01         ; 0190
    or   r4, r3                              ; 0198
    and  r3, r7, #0xff0000                   ; 01a0
    shr  r3, r3, #0x10                       ; 01a8
    wr   [FMC_28], #0x0                      ; 01b0
    ld   r0, [P_CHUNKS], 0xffffffff          ; 01b8
    st   [V_D7C], r0                         ; 01c0
    st   [FMC_CTRL0], r4                     ; 01c8
    wait 2                                   ; 01d0
    wr   [FMC_STATUS], #0x100000             ; 01d8
    ld   r0, [P_ECCCFG], 0xffffffff          ; 01e0
    wr   [ECC_STATUS], #0x1ff                ; 01e8
    wr   [FMC_1C], #0x0                      ; 01f0
    st   [ECC_CTRL], r0                      ; 01f8
    st   [FMC_CMD], r3                       ; 0200
    wait 0                                   ; 0208
    wr   [FMC_STATUS], #0x2                  ; 0210
    wr   [FMCTRL1], #0x1                     ; 0218
    wait 1                                   ; 0220
    wr   [FMC_STATUS], #0x4                  ; 0228
    wait 4                                   ; 0230
    wr   [ECC_STATUS], #0x1                  ; 0238
    ld   r0, [P_SPARELEN], 0xffffffff        ; 0240
    st   [FMC_DNUM], r0                      ; 0248
    wr   [FMC_DATA_SEL], #0x1000             ; 0250
    wr   [FMCTRL1], #0x34                    ; 0258
L_0260:
    wait 3                                   ; 0260
    wr   [FMC_STATUS], #0x8                  ; 0268
    wr   [FMC_DATA_SEL], #0x102              ; 0270
    wr   [FMC_DNUM], #0x1ff                  ; 0278
    wr   [FMCTRL1], #0x2e4                   ; 0280
    wait 2                                   ; 0288
    wr   [FMC_STATUS], #0x100000             ; 0290
    ld   r0, [P_ECCMODE], 0xffffffff         ; 0298
    wr   [ECC_STATUS], #0x1ff                ; 02a0
    wr   [FMC_1C], #0x1                      ; 02a8
    st   [ECC_CTRL], r0                      ; 02b0
    wait 4                                   ; 02b8
    wr   [ECC_STATUS], #0x1                  ; 02c0
    wait 3                                   ; 02c8
    wr   [FMC_STATUS], #0x8                  ; 02d0
    ld   r0, [P_PARLEN], 0xffffffff          ; 02d8
    st   [FMC_DNUM], r0                      ; 02e0
    wr   [FMC_DATA_SEL], #0x2000             ; 02e8
    wr   [FMCTRL1], #0x34                    ; 02f0
    wait 3                                   ; 02f8
    wr   [FMC_STATUS], #0x8                  ; 0300
    wr   [FMC_DATA_SEL], #0x201              ; 0308
    wr   [FMC_DNUM], #0x1ff                  ; 0310
    wr   [FMCTRL1], #0x2e4                   ; 0318
    wait 2                                   ; 0320
    wr   [FMC_STATUS], #0x100000             ; 0328
    ld   r0, [P_ECCMODE], 0xffffffff         ; 0330
    wr   [ECC_STATUS], #0x1ff                ; 0338
    wr   [FMC_1C], #0x0                      ; 0340
    st   [ECC_CTRL], r0                      ; 0348
    wait 4                                   ; 0350
    wr   [ECC_STATUS], #0x1                  ; 0358
    wait 3                                   ; 0360
    wr   [FMC_STATUS], #0x8                  ; 0368
    ld   r0, [P_PARLEN], 0xffffffff          ; 0370
    st   [FMC_DNUM], r0                      ; 0378
    wr   [FMC_DATA_SEL], #0x1000             ; 0380
    wr   [FMCTRL1], #0x34                    ; 0388
    wait 3                                   ; 0390
    wr   [FMC_STATUS], #0x8                  ; 0398
    wr   [FMC_DATA_SEL], #0x102              ; 03a0
    wr   [FMC_DNUM], #0x1ff                  ; 03a8
    wr   [FMCTRL1], #0x2e4                   ; 03b0
    wait 2                                   ; 03b8
    wr   [FMC_STATUS], #0x100000             ; 03c0
    ld   r0, [P_ECCMODE], 0xffffffff         ; 03c8
    wr   [ECC_STATUS], #0x1ff                ; 03d0
    wr   [FMC_1C], #0x1                      ; 03d8
    st   [ECC_CTRL], r0                      ; 03e0
    ld   r0, [P_DMALIST], 0xffffffff         ; 03e8
    ldi  r1, [r0]                            ; 03f0
    st   [FMC_DMA_ADDR], r1                  ; 03f8
    add  r0, r0, #0x4                        ; 0400
    st   [P_DMALIST], r0                     ; 0408
    ld   r0, [V_D7C], 0xffffffff             ; 0410
    sub  r0, r0, #0x1                        ; 0418
    st   [V_D7C], r0                         ; 0420
    bnz  r0, L_0930                          ; 0428
    and  r5, r7, #0xff00                     ; 0430
    shr  r5, r5, #0x8                        ; 0438
    ld   r0, [P_CEMASKS], 0xffffffff         ; 0440
    ldi  r7, [r0]                            ; 0448
    add  r0, r0, #0x4                        ; 0450
    ldi  r2, [r0]                            ; 0458
    add  r0, r0, #0x4                        ; 0460
    st   [P_CEMASKS], r0                     ; 0468
    bz   r7, L_0a10                          ; 0470
    wait 4                                   ; 0478
    wr   [ECC_STATUS], #0x1                  ; 0480
    wait 3                                   ; 0488
    wr   [FMC_STATUS], #0x8                  ; 0490
    ld   r0, [P_PARLEN], 0xffffffff          ; 0498
    st   [FMC_DNUM], r0                      ; 04a0
    wr   [FMC_DATA_SEL], #0x2000             ; 04a8
    wr   [FMCTRL1], #0x34                    ; 04b0
    wait 3                                   ; 04b8
    wr   [FMC_STATUS], #0x8                  ; 04c0
    wr   [FMC_DATA_SEL], #0x201              ; 04c8
    wr   [FMC_DNUM], #0x1ff                  ; 04d0
    wr   [FMCTRL1], #0x2e4                   ; 04d8
    ld   r0, [P_META], 0xffffffff            ; 04e0
    ldi  r1, [r0]                            ; 04e8
    st   [FMC_DATA0?], r1                    ; 04f0
    add  r0, r0, #0x4                        ; 04f8
    ldi  r1, [r0]                            ; 0500
    st   [FMC_64], r1                        ; 0508
    add  r0, r0, #0x4                        ; 0510
    ldi  r1, [r0]                            ; 0518
    st   [FMC_68], r1                        ; 0520
    add  r0, r0, #0x4                        ; 0528
    wr   [FMC_40], #0xc1                     ; 0530
L_0538:
    ld   r1, [FMC_40], 0x1                   ; 0538
    bnz  r1, L_0538                          ; 0540
    st   [P_META], r0                        ; 0548
    wait 2                                   ; 0550
    wr   [FMC_STATUS], #0x100000             ; 0558
    ld   r0, [P_ECCCFG], 0xffffffff          ; 0560
    wr   [ECC_STATUS], #0x1ff                ; 0568
    wr   [FMC_1C], #0x0                      ; 0570
    st   [ECC_CTRL], r0                      ; 0578
    ld   r0, [P_CHUNKS], 0xffffffff          ; 0580
    st   [V_D7C], r0                         ; 0588
    and  r3, r7, #0xff                       ; 0590
    shl  r3, r3, #0x1                        ; 0598
    or   r6, r3                              ; 05a0
    ld   r4, [FMC_CTRL0], 0xfffffe01         ; 05a8
    or   r4, r3                              ; 05b0
    and  r3, r7, #0xff0000                   ; 05b8
    shr  r3, r3, #0x10                       ; 05c0
    shr  r1, r7, #0x18                       ; 05c8
    bz   r1, L_0870                          ; 05d0
    sub  r0, r1, #0x78                       ; 05d8
    bz   r0, L_0718                          ; 05e0
    wr   [FMC_ANUM], #0x4                    ; 05e8
    shl  r0, r2, #0x10                       ; 05f0
    st   [FMC_ADDR0], r0                     ; 05f8
    shr  r0, r2, #0x10                       ; 0600
    st   [FMC_ADDR1], r0                     ; 0608
    wait 4                                   ; 0610
    wr   [ECC_STATUS], #0x1                  ; 0618
    wait 3                                   ; 0620
    wr   [FMC_STATUS], #0x8                  ; 0628
    st   [FMC_CMD], r5                       ; 0630
    wait 0                                   ; 0638
    wr   [FMC_STATUS], #0x2                  ; 0640
    st   [FMC_CTRL0], r4                     ; 0648
    st   [FMC_CMD], r1                       ; 0650
    wait 0                                   ; 0658
    wr   [FMC_STATUS], #0x2                  ; 0660
    wr   [FMCTRL1], #0xca                    ; 0668
    wait 12, arg=0x40, mask=0x40             ; 0670
    ld   r0, [FMC_NSTAT], 0x1                ; 0678
    wr   [FMCTRL1], #0x30020                 ; 0680
    wr   [FMC_STATUS], #0x1800000            ; 0688
    bz   r0, L_06c0                          ; 0690
    or   r6, r6, #0x10000000                 ; 0698
    ld   r1, [FMC_CTRL0], 0x1fe              ; 06a0
    shr  r1, r1, #0x1                        ; 06a8
    st   [V_FAILCE], r1                      ; 06b0
    bnz  r0, L_0ab0                          ; 06b8
L_06c0:
    st   [FMC_CMD], r3                       ; 06c0
    wait 0                                   ; 06c8
    wr   [FMC_STATUS], #0x2                  ; 06d0
    wr   [FMCTRL1], #0x1                     ; 06d8
    wait 1                                   ; 06e0
    wr   [FMC_STATUS], #0x4                  ; 06e8
    ld   r0, [P_SPARELEN], 0xffffffff        ; 06f0
    st   [FMC_DNUM], r0                      ; 06f8
    wr   [FMC_DATA_SEL], #0x1000             ; 0700
    wr   [FMCTRL1], #0x34                    ; 0708
    bnz  r0, L_0260                          ; 0710
L_0718:
    wr   [FMC_ANUM], #0x2                    ; 0718
    st   [FMC_ADDR0], r2                     ; 0720
    wait 4                                   ; 0728
    wr   [ECC_STATUS], #0x1                  ; 0730
    wait 3                                   ; 0738
    wr   [FMC_STATUS], #0x8                  ; 0740
    st   [FMC_CMD], r5                       ; 0748
    wait 0                                   ; 0750
    wr   [FMC_STATUS], #0x2                  ; 0758
    st   [FMC_CTRL0], r4                     ; 0760
    wr   [FMC_CMD], #0x78                    ; 0768
    wait 0                                   ; 0770
    wr   [FMC_STATUS], #0x2                  ; 0778
    wr   [FMCTRL1], #0x1                     ; 0780
    wait 1                                   ; 0788
    wr   [FMC_STATUS], #0x4                  ; 0790
    wr   [FMCTRL1], #0xca                    ; 0798
    wait 12, arg=0x40, mask=0x40             ; 07a0
    ld   r0, [FMC_NSTAT], 0x1                ; 07a8
    wr   [FMCTRL1], #0x30020                 ; 07b0
    wr   [FMC_STATUS], #0x1800000            ; 07b8
    bz   r0, L_07f0                          ; 07c0
    or   r6, r6, #0x10000000                 ; 07c8
    ld   r1, [FMC_CTRL0], 0x1fe              ; 07d0
    shr  r1, r1, #0x1                        ; 07d8
    st   [V_FAILCE], r1                      ; 07e0
    bnz  r0, L_0ab0                          ; 07e8
L_07f0:
    wr   [FMC_ANUM], #0x4                    ; 07f0
    shl  r1, r2, #0x10                       ; 07f8
    st   [FMC_ADDR0], r1                     ; 0800
    shr  r1, r2, #0x10                       ; 0808
    st   [FMC_ADDR1], r1                     ; 0810
    st   [FMC_CMD], r3                       ; 0818
    wait 0                                   ; 0820
    wr   [FMC_STATUS], #0x2                  ; 0828
    wr   [FMCTRL1], #0x1                     ; 0830
    wait 1                                   ; 0838
    wr   [FMC_STATUS], #0x4                  ; 0840
    ld   r0, [P_SPARELEN], 0xffffffff        ; 0848
    st   [FMC_DNUM], r0                      ; 0850
    wr   [FMC_DATA_SEL], #0x1000             ; 0858
    wr   [FMCTRL1], #0x34                    ; 0860
    bnz  r0, L_0260                          ; 0868
L_0870:
    wr   [FMC_ANUM], #0x4                    ; 0870
    shl  r0, r2, #0x10                       ; 0878
    st   [FMC_ADDR0], r0                     ; 0880
    shr  r0, r2, #0x10                       ; 0888
    st   [FMC_ADDR1], r0                     ; 0890
    wait 4                                   ; 0898
    wr   [ECC_STATUS], #0x1                  ; 08a0
    wait 3                                   ; 08a8
    wr   [FMC_STATUS], #0x8                  ; 08b0
    st   [FMC_CMD], r5                       ; 08b8
    wait 0                                   ; 08c0
    wr   [FMC_STATUS], #0x2                  ; 08c8
    st   [FMC_CTRL0], r4                     ; 08d0
    st   [FMC_CMD], r3                       ; 08d8
    wait 0                                   ; 08e0
    wr   [FMC_STATUS], #0x2                  ; 08e8
    wr   [FMCTRL1], #0x1                     ; 08f0
    wait 1                                   ; 08f8
    wr   [FMC_STATUS], #0x4                  ; 0900
    ld   r0, [P_SPARELEN], 0xffffffff        ; 0908
    st   [FMC_DNUM], r0                      ; 0910
    wr   [FMC_DATA_SEL], #0x1000             ; 0918
    wr   [FMCTRL1], #0x34                    ; 0920
    bnz  r0, L_0260                          ; 0928
L_0930:
    wait 4                                   ; 0930
    wr   [ECC_STATUS], #0x1                  ; 0938
    wait 3                                   ; 0940
    wr   [FMC_STATUS], #0x8                  ; 0948
    ld   r0, [P_PARLEN], 0xffffffff          ; 0950
    st   [FMC_DNUM], r0                      ; 0958
    wr   [FMC_DATA_SEL], #0x2000             ; 0960
    wr   [FMCTRL1], #0x34                    ; 0968
    wait 3                                   ; 0970
    wr   [FMC_STATUS], #0x8                  ; 0978
    wr   [FMC_DATA_SEL], #0x201              ; 0980
    wr   [FMC_DNUM], #0x1ff                  ; 0988
    wr   [FMCTRL1], #0x2e4                   ; 0990
    wait 2                                   ; 0998
    wr   [FMC_STATUS], #0x100000             ; 09a0
    ld   r0, [P_ECCMODE], 0xffffffff         ; 09a8
    wr   [ECC_STATUS], #0x1ff                ; 09b0
    wr   [FMC_1C], #0x0                      ; 09b8
    st   [ECC_CTRL], r0                      ; 09c0
    wait 4                                   ; 09c8
    wr   [ECC_STATUS], #0x1                  ; 09d0
    wait 3                                   ; 09d8
    wr   [FMC_STATUS], #0x8                  ; 09e0
    ld   r0, [P_PARLEN], 0xffffffff          ; 09e8
    st   [FMC_DNUM], r0                      ; 09f0
    wr   [FMC_DATA_SEL], #0x1000             ; 09f8
    wr   [FMCTRL1], #0x34                    ; 0a00
    bnz  r0, L_0260                          ; 0a08
L_0a10:
    wait 4                                   ; 0a10
    wr   [ECC_STATUS], #0x1                  ; 0a18
    wait 3                                   ; 0a20
    wr   [FMC_STATUS], #0x8                  ; 0a28
    ld   r0, [P_PARLEN], 0xffffffff          ; 0a30
    st   [FMC_DNUM], r0                      ; 0a38
    wr   [FMC_DATA_SEL], #0x2000             ; 0a40
    wr   [FMCTRL1], #0x34                    ; 0a48
    wait 3                                   ; 0a50
    wr   [FMC_STATUS], #0x8                  ; 0a58
    ld   r0, [FMC_CTRL0], 0xfeffffff         ; 0a60
    st   [FMC_CTRL0], r0                     ; 0a68
    wr   [FMC_DATA_SEL], #0x200              ; 0a70
    wr   [FMC_DNUM], #0x1ff                  ; 0a78
    wr   [FMCTRL1], #0xe4                    ; 0a80
    wait 3                                   ; 0a88
    wr   [FMC_STATUS], #0x8                  ; 0a90
    st   [FMC_CMD], r5                       ; 0a98
    wait 0                                   ; 0aa0
    wr   [FMC_STATUS], #0x2                  ; 0aa8
L_0ab0:
    movi r2, #0x2                            ; 0ab0
    ld   r1, [P_IDBUF], 0xf                  ; 0ab8
L_0ac0:
    mov  r0, r2                              ; 0ac0
    and  r0, r6                              ; 0ac8
    bz   r0, L_0da0                          ; 0ad0
    ld   r0, [FMC_CTRL0], 0xfffffe01         ; 0ad8
    or   r0, r2                              ; 0ae0
    st   [FMC_CTRL0], r0                     ; 0ae8
    ld   r4, [P_STATROW], 0xffffffff         ; 0af0
    bnz  r4, L_0b70                          ; 0af8
    wr   [FMC_CMD], #0x70                    ; 0b00
    wait 0                                   ; 0b08
    wr   [FMC_STATUS], #0x2                  ; 0b10
    wr   [FMCTRL1], #0xca                    ; 0b18
    wait 12, arg=0x40, mask=0x40             ; 0b20
    ld   r0, [FMC_NSTAT], 0x1                ; 0b28
    wr   [FMCTRL1], #0x30020                 ; 0b30
    wr   [FMC_STATUS], #0x1800000            ; 0b38
    bz   r0, L_0da0                          ; 0b40
    or   r6, r6, #0x10000000                 ; 0b48
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0b50
    shr  r0, r0, #0x1                        ; 0b58
    st   [V_FAILCE], r0                      ; 0b60
    bnz  r0, L_0da0                          ; 0b68
L_0b70:
    wr   [FMC_ANUM], #0x2                    ; 0b70
    st   [FMC_ADDR0], r4                     ; 0b78
    wr   [FMC_CMD], #0x78                    ; 0b80
    wait 0                                   ; 0b88
    wr   [FMC_STATUS], #0x2                  ; 0b90
    wr   [FMCTRL1], #0x1                     ; 0b98
    wait 1                                   ; 0ba0
    wr   [FMC_STATUS], #0x4                  ; 0ba8
    wr   [FMCTRL1], #0xca                    ; 0bb0
    wait 12, arg=0x20, mask=0x20             ; 0bb8
    ld   r0, [FMC_NSTAT], 0x1                ; 0bc0
    wr   [FMCTRL1], #0x30020                 ; 0bc8
    wr   [FMC_STATUS], #0x1800000            ; 0bd0
    bz   r0, L_0c00                          ; 0bd8
    or   r6, r6, #0x10000000                 ; 0be0
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0be8
    shr  r0, r0, #0x1                        ; 0bf0
    st   [V_FAILCE], r0                      ; 0bf8
L_0c00:
    add  r4, r4, #0x80                       ; 0c00
    st   [FMC_ADDR0], r4                     ; 0c08
    wr   [FMC_CMD], #0x78                    ; 0c10
    wait 0                                   ; 0c18
    wr   [FMC_STATUS], #0x2                  ; 0c20
    wr   [FMCTRL1], #0x1                     ; 0c28
    wait 1                                   ; 0c30
    wr   [FMC_STATUS], #0x4                  ; 0c38
    wr   [FMCTRL1], #0xca                    ; 0c40
    wait 12, arg=0x20, mask=0x20             ; 0c48
    ld   r0, [FMC_NSTAT], 0x1                ; 0c50
    wr   [FMCTRL1], #0x30020                 ; 0c58
    wr   [FMC_STATUS], #0x1800000            ; 0c60
    bz   r0, L_0c90                          ; 0c68
    or   r6, r6, #0x10000000                 ; 0c70
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0c78
    shr  r0, r0, #0x1                        ; 0c80
    st   [V_FAILCE], r0                      ; 0c88
L_0c90:
    wr   [FMC_ADDR0], #0x0                   ; 0c90
    wr   [FMC_CMD], #0x78                    ; 0c98
    wait 0                                   ; 0ca0
    wr   [FMC_STATUS], #0x2                  ; 0ca8
    wr   [FMCTRL1], #0x1                     ; 0cb0
    wait 1                                   ; 0cb8
    wr   [FMC_STATUS], #0x4                  ; 0cc0
    wr   [FMCTRL1], #0xca                    ; 0cc8
    wait 12, arg=0x20, mask=0x20             ; 0cd0
    ld   r0, [FMC_NSTAT], 0x1                ; 0cd8
    wr   [FMCTRL1], #0x30020                 ; 0ce0
    wr   [FMC_STATUS], #0x1800000            ; 0ce8
    bz   r0, L_0d18                          ; 0cf0
    or   r6, r6, #0x10000000                 ; 0cf8
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0d00
    shr  r0, r0, #0x1                        ; 0d08
    st   [V_FAILCE], r0                      ; 0d10
L_0d18:
    wr   [FMC_ADDR0], #0x80                  ; 0d18
    wr   [FMC_CMD], #0x78                    ; 0d20
    wait 0                                   ; 0d28
    wr   [FMC_STATUS], #0x2                  ; 0d30
    wr   [FMCTRL1], #0x1                     ; 0d38
    wait 1                                   ; 0d40
    wr   [FMC_STATUS], #0x4                  ; 0d48
    wr   [FMCTRL1], #0xca                    ; 0d50
    wait 12, arg=0x20, mask=0x20             ; 0d58
    ld   r0, [FMC_NSTAT], 0x1                ; 0d60
    wr   [FMCTRL1], #0x30020                 ; 0d68
    wr   [FMC_STATUS], #0x1800000            ; 0d70
    bz   r0, L_0da0                          ; 0d78
    or   r6, r6, #0x10000000                 ; 0d80
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0d88
    shr  r0, r0, #0x1                        ; 0d90
    st   [V_FAILCE], r0                      ; 0d98
L_0da0:
    sub  r1, r1, #0x1                        ; 0da0
    bz   r1, L_0dc0                          ; 0da8
    shl  r2, r2, #0x1                        ; 0db0
    bnz  r1, L_0ac0                          ; 0db8
L_0dc0:
    ld   r0, [FMC_CTRL0], 0xfffffe01         ; 0dc0
    st   [FMC_CTRL0], r0                     ; 0dc8
    end                                      ; 0dd0
    ld   r2, [FMSS_C3C], 0xffffffff          ; 0dd8
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0de0
    shr  r0, r0, #0x1                        ; 0de8
    and  r6, r6, #0xffffff00                 ; 0df0
    or   r6, r0                              ; 0df8
    ld   r0, [FMSS_C50], 0x1                 ; 0e00
    bz   r0, L_0e20                          ; 0e08
    wr   [FMSS_C50], #0x1                    ; 0e10
    or   r6, r6, #0x8000000                  ; 0e18
L_0e20:
    ld   r0, [FMC_CTRL0], 0xfffffe01         ; 0e20
    st   [FMC_CTRL0], r0                     ; 0e28
    or   r6, r6, #0x10000000                 ; 0e30
    end                                      ; 0e38
