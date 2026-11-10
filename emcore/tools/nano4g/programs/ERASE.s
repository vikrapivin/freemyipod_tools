; restore OS FIRMWARE.x1243 program ERASE: file+0x3707c..0x3736c = firmware_body+0x36a7c = mem:0x08036398 
; FMSS microcode, 94 instructions, assembled with tools/fmssasm.c
    wr   [CS_IRQ], #0xff                     ; 0000
    wr   [FMSS_C10], #0x1                    ; 0008
    wr   [FMSS_C58], #0x4                    ; 0010
    wr   [FMSS_C4C], #0x288                  ; 0018
    wr   [FMCTRL1], #0x33338e0               ; 0020
    wr   [FMC_STATUS], #0xff00ffe            ; 0028
    movi r0, #0x801                          ; 0030
    ld   r1, [P_TIMING], 0xffffffff          ; 0038
    or   r0, r1                              ; 0040
    st   [FMC_CTRL0], r0                     ; 0048
    movi r6, #0x0                            ; 0050
    wr   [P_STATROW], #0xcafebabe            ; 0058
    wr   [V_D7C], #0xcafebabe                ; 0060
L_0068:
    ld   r0, [P_CEMASKS], 0xffffffff         ; 0068
    ldi  r2, [r0]                            ; 0070
    add  r0, r0, #0x4                        ; 0078
    st   [P_CEMASKS], r0                     ; 0080
    movi r3, #0x1                            ; 0088
    shl  r3, r2                              ; 0090
    shl  r0, r3, #0x1                        ; 0098
    ld   r1, [FMC_CTRL0], 0xfffffe01         ; 00a0
    or   r1, r0                              ; 00a8
    st   [FMC_CTRL0], r1                     ; 00b0
    mov  r2, r3                              ; 00b8
    and  r2, r6                              ; 00c0
    bz   r2, L_0118                          ; 00c8
    wr   [FMC_CMD], #0x70                    ; 00d0
    wait 0                                   ; 00d8
    wr   [FMC_STATUS], #0x2                  ; 00e0
    wr   [FMCTRL1], #0xca                    ; 00e8
    wait 12, arg=0x40, mask=0x40             ; 00f0
    ld   r2, [FMC_NSTAT], 0x1                ; 00f8
    wr   [FMCTRL1], #0x30020                 ; 0100
    wr   [FMC_STATUS], #0x1800000            ; 0108
    bnz  r2, L_0288                          ; 0110
L_0118:
    or   r6, r3                              ; 0118
    wr   [FMC_CMD], #0x60                    ; 0120
    wait 0                                   ; 0128
    wr   [FMC_STATUS], #0x2                  ; 0130
    ld   r1, [P_ROWS], 0xffffffff            ; 0138
    ldi  r2, [r1]                            ; 0140
    add  r1, r1, #0x4                        ; 0148
    st   [P_ROWS], r1                        ; 0150
    wr   [FMC_ANUM], #0x2                    ; 0158
    st   [FMC_ADDR0], r2                     ; 0160
    wr   [FMCTRL1], #0x1                     ; 0168
    wait 1                                   ; 0170
    wr   [FMC_STATUS], #0x4                  ; 0178
    wr   [FMC_CMD], #0xd0                    ; 0180
    wait 0                                   ; 0188
    wr   [FMC_STATUS], #0x2                  ; 0190
    ld   r2, [P_COUNT], 0xffffffff           ; 0198
    sub  r2, r2, #0x1                        ; 01a0
    st   [P_COUNT], r2                       ; 01a8
    bnz  r2, L_0068                          ; 01b0
    movi r2, #0x1                            ; 01b8
    movi r3, #0x8                            ; 01c0
    bnz  r3, L_01e8                          ; 01c8
L_01d0:
    sub  r3, r3, #0x1                        ; 01d0
    bz   r3, L_0270                          ; 01d8
    shl  r2, r2, #0x1                        ; 01e0
L_01e8:
    mov  r0, r2                              ; 01e8
    and  r0, r6                              ; 01f0
    bz   r0, L_01d0                          ; 01f8
    shl  r0, r2, #0x1                        ; 0200
    ld   r1, [FMC_CTRL0], 0xfffffe01         ; 0208
    or   r1, r0                              ; 0210
    st   [FMC_CTRL0], r1                     ; 0218
    wr   [FMC_CMD], #0x70                    ; 0220
    wait 0                                   ; 0228
    wr   [FMC_STATUS], #0x2                  ; 0230
    wr   [FMCTRL1], #0xca                    ; 0238
    wait 12, arg=0x40, mask=0x40             ; 0240
    ld   r4, [FMC_NSTAT], 0x1                ; 0248
    wr   [FMCTRL1], #0x30020                 ; 0250
    wr   [FMC_STATUS], #0x1800000            ; 0258
    bnz  r4, L_0288                          ; 0260
    bnz  r3, L_01d0                          ; 0268
L_0270:
    ld   r1, [FMC_CTRL0], 0xfffffe01         ; 0270
    st   [FMC_CTRL0], r1                     ; 0278
    end                                      ; 0280
L_0288:
    ld   r2, [FMSS_C3C], 0xffffffff          ; 0288
    ld   r0, [FMC_CTRL0], 0x1fe              ; 0290
    shr  r0, r0, #0x1                        ; 0298
    and  r6, r6, #0xffffff00                 ; 02a0
    or   r6, r0                              ; 02a8
    ld   r0, [FMSS_C50], 0x1                 ; 02b0
    bz   r0, L_02d0                          ; 02b8
    wr   [FMSS_C50], #0x1                    ; 02c0
    or   r6, r6, #0x8000000                  ; 02c8
L_02d0:
    ld   r1, [FMC_CTRL0], 0xfffffe01         ; 02d0
    st   [FMC_CTRL0], r1                     ; 02d8
    or   r6, r6, #0x10000000                 ; 02e0
    end                                      ; 02e8
