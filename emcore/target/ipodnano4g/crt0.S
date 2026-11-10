@
@
@    Copyright 2010 TheSeven
@
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
@    You should have received a copy of the GNU General Public License
@    along with emCORE.  If not, see <http://www.gnu.org/licenses/>.
@
@


.section .intvect,"ax",%progbits
	ldr pc, =reset_handler
	ldr pc, =undef_instr_handler
	ldr pc, =syscall_handler
	ldr pc, =prefetch_abort_handler
	ldr pc, =data_abort_handler
	ldr pc, =reserved_handler
	ldr pc, =irq_handler
	ldr pc, =fiq_handler
.ltorg


.section .inithead,"ax",%progbits
.global __start
__start:
	b	_start

.section .initcode,"ax",%progbits
.global _start
_start:
	@ One kernel entry for DFU and EXECFIRMWARE reload. The retained bootstub
	@ enters with SCTLR.M=1; execfirmware clears M+C before its jump.
	@ Preserve that distinction across relocation for the experiment capability.
	mrc	p15, 0, r8,c1,c0
	and	r8, r8, #1
	eor	r8, r8, #1
	cpsid	aif
	ldr	r0, =0x38200110		@ DMAC0 C0CONFIG (PL080, +0x20 per channel)
	ldr	r1, =0x38700110		@ DMAC1 C0CONFIG
	mov	r2, #8
.stage2_dma:
	ldr	r3, [r0]
	bic	r3, r3, #1		@ channel enable off (stage 1 gone, no transfer may continue)
	str	r3, [r0], #0x20
	ldr	r3, [r1]
	bic	r3, r3, #1
	str	r3, [r1], #0x20
	subs	r2, r2, #1
	bne	.stage2_dma
	mov	r0, #0
	mcr	p15, 0, r0,c7,c14,0	@ clean+invalidate D (all lines are clean already: execfirmware cleaned)
	@ invalidate I: the previous kernel's lines for 0x09f8xxxx/0x2200xxxx/0x0800xxxx hold OTHER code at the addresses stage 2
	@ uses. ARM1176 erratum 411920 (an entire-I-cache invalidate coincident with a linefill can miss lines):
	@ 4 x c7,c5,0 + 11 NOPs as Linux arch/arm/mm/cache-v6.S (ARM_ERRATA_411920) does.
	mcr	p15, 0, r0,c7,c5,0
	mcr	p15, 0, r0,c7,c5,0
	mcr	p15, 0, r0,c7,c5,0
	mcr	p15, 0, r0,c7,c5,0
	.rept	11
	nop
	.endr
	mcr	p15, 0, r0,c7,c5,6	@ flush branch target cache
	mcr	p15, 0, r0,c8,c7,0	@ invalidate unified TLB
	mcr	p15, 0, r0,c7,c10,4	@ drain write buffer
	mcr	p15, 0, r0,c7,c5,4	@ flush prefetch buffer
	mrc	p15, 0, r0,c1,c0
	bic	r0, r0, #0x200
	orr	r0, r0, #0x100
	mcr	p15, 0, r0,c1,c0
	mov	r0, #0x7fffffff
	mcr	p15, 0, r0,c3,c0
	@ Keep the bootstub TTBR at 0x2202c000; do not write a second, unused table.
	mrc	p15, 0, r0,c1,c0
	orr	r0, r0, #5
	orr	r0, r0, #0x400000
	mcr	p15, 0, r0,c1,c0
	ldr	r0, =_sramsource
	ldr	r1, =_sramstart
	ldr	r2, =_sramend
.copysram:
	cmp	r2, r1
	ldrhi	r3, [r0], #4
	strhi	r3, [r1], #4
	bhi	.copysram
	ldr	r0, =_sdramsource
	ldr	r1, =_sdramstart
	ldr	r2, =_sdramend
.copysdram:
	cmp	r2, r1
	ldrhi	r3, [r0], #4
	strhi	r3, [r1], #4
	bhi	.copysdram
	mov	r2, #0
	ldr	r0, =_ibssstart
	ldr	r1, =_ibssend
	mov	r2, #0
.clearibss:
	cmp	r1, r0
	strhi	r2, [r0], #4
	bhi	.clearibss
	ldr	r0, =_bssstart
	ldr	r1, =_bssend
.clearbss:
	cmp	r1, r0
	strhi	r2, [r0], #4
	bhi	.clearbss
	ldr	r0, =n4g_reload_boot
	str	r8, [r0]
	mov	r0, #0
	mcr	p15, 0, r0,c7,c10,0 @ clean data cache
	mcr	p15, 0, r0,c7,c10,4 @ drain write buffer
	mcr	p15, 0, r0,c7,c5,0  @ invalidate instruction cache
	mcr	p15, 0, r0,c7,c5,0	@ erratum 411920 workaround (see the stage-2 prologue): .sram/.sdram now hold
	mcr	p15, 0, r0,c7,c5,0	@ stage 2's code at stage 1's addresses
	mcr	p15, 0, r0,c7,c5,0
	.rept	11
	nop
	.endr
	mcr	p15, 0, r0,c7,c5,6	@ flush branch target cache
	mcr	p15, 0, r0,c7,c5,4  @ flush prefetch buffer
	ldr	r1, =0x38e00000
	add	r2, r1, #0x00001000
	add	r3, r1, #0x00002000
	mov	r4, #-1
	str	r4, [r1,#0x14]
	str	r4, [r2,#0x14]
	str	r4, [r1,#0xf00]
	str	r4, [r2,#0xf00]
	str	r4, [r3,#0x08]
	str	r4, [r3,#0x0c]
	msr	cpsr_c, #0xd2
	ldr	sp, =_irqstackend
	msr	cpsr_c, #0xd7
	ldr	sp, =_abortstackend
	msr	cpsr_c, #0xdb
	ldr	sp, =_abortstackend
	msr	cpsr_c, #0x1f
	ldr	sp, =_abortstackend
	bl	init
	bl	yield
	mov	r0, #0
	ldr	pc, =idleloop
.ltorg


.section .icode, "ax", %progbits
.align 2
idleloop:
	mcr	p15, 0, r0,c7,c0,4
	b	idleloop

.global reset
.global hang
.type reset, %function
.type hang, %function
reset:
	msr	cpsr_c, #0xd3
	mov	r0, #0x100000
	mov	r1, #0x3c800000
	str	r0, [r1]
	@ fw_restore mem:0x0802c490: watchdog write alone is not a software reset.
	ldr	r0, =0xaaa5
	ldr	r1, =0x3c500050
	str	r0, [r1]
hang:
	msr	cpsr_c, #0xd3
	mov	r0, #0
	mcr	p15, 0, r0,c7,c0,4
	b	hang
.size reset, .-reset
.size hang, .-hang

.type reset_handler, %function
reset_handler:
	stmfd	sp, {r10-r12}
	mov	r10, sp
	mov	r11, lr
	mrs	r12, cpsr
	msr	cpsr_c, #0xd7
	sub	sp, sp, #0x44
	stmia	sp!, {r0-r9}
	sub	r0, r10, #0xc
	ldmia	r0, {r0-r2}
	mov	r3, r10
	mov	r4, r11
	mov	r5, r11
	mov	r6, r12
	stmia	sp!, {r0-r6}
	sub	sp, sp, #0x44
	mov	r0, #0
	adr	r1, reset_text
	mov	r2, r11
	b	panicf
.size reset_handler, .-reset_handler

.global undef_instr_handler
.type undef_instr_handler, %function
undef_instr_handler:
	sub	sp, sp, #0x44
	stmia	sp!, {r0-r12}
	sub	r2, lr, #4
	mrs	r3, spsr
	mrs	r4, cpsr
	orr	r0, r3, #0xc0
	msr	cpsr_c, r0
	mov	r0, sp
	mov	r1, lr
	msr	cpsr_c, r4
	stmia	sp!, {r0-r3}
	sub	sp, sp, #0x44
	mov	r0, #0
	adr	r1, undef_instr_text
	ldr	r3, [r2]
	b	panicf
.size undef_instr_handler, .-undef_instr_handler

.type prefetch_abort_handler, %function
prefetch_abort_handler:
	sub	sp, sp, #0x44
	stmia	sp!, {r0-r12}
	sub	r2, lr, #4
	mrs	r3, spsr
	mrs	r4, cpsr
	orr	r0, r3, #0xc0
	msr	cpsr_c, r0
	mov	r0, sp
	mov	r1, lr
	msr	cpsr_c, r4
	stmia	sp!, {r0-r3}
	sub	sp, sp, #0x44
	mov	r0, #0
	adr	r1, prefetch_abort_text
	mrc	p15, 0, r3,c5,c0,1
	mov	r4, r3,lsr#4
	and	r4, r4, #0xf
	and	r5, r3, #0xf
	stmfd	sp!, {r4-r5}
	b	panicf
.size prefetch_abort_handler, .-prefetch_abort_handler

.type data_abort_handler, %function
data_abort_handler:
	sub	sp, sp, #0x44
	stmia	sp!, {r0-r12}
	sub	r2, lr, #8
	mrs	r3, spsr
	mrs	r4, cpsr
	orr	r0, r3, #0xc0
	msr	cpsr_c, r0
	mov	r0, sp
	mov	r1, lr
	msr	cpsr_c, r4
	stmia	sp!, {r0-r3}
	sub	sp, sp, #0x44
	mov	r0, #0
	adr	r1, data_abort_text
	mrc	p15, 0, r3,c5,c0
	mov	r4, r3,lsr#4
	and	r4, r4, #0xf
	and	r5, r3, #0xf
	mrc	p15, 0, r6,c6,c0
	stmfd	sp!, {r4-r6}
	b	panicf
.size data_abort_handler, .-data_abort_handler

.type reserved_handler, %function
reserved_handler:
	stmfd	sp, {r10-r12}
	mov	r10, sp
	mov	r11, lr
	mrs	r12, cpsr
	msr	cpsr_c, #0xd7
	sub	sp, sp, #0x44
	stmia	sp!, {r0-r9}
	sub	r0, r10, #0xc
	ldmia	r0, {r0-r2}
	mov	r3, r10
	mov	r4, r11
	mov	r5, r11
	mov	r6, r12
	stmia	sp!, {r0-r6}
	sub	sp, sp, #0x44
	mov	r0, #0
	adr	r1, reserved_text
	mov	r2, r11
	b	panicf
.size reserved_handler, .-reserved_handler

.type fiq_handler, %function
fiq_handler:
	mov	r0, #2
	adr	r1, fiq_text
	b	panic
.size fiq_handler, .-fiq_handler

prefetch_abort_text:
	.ascii	"Prefetch abort at %08X!\nFSR: %08X (domain %d, fault %d)\0"

reset_text:
	.ascii	"Hit reset vector!\n(Last known PC: %08X)\0"

undef_instr_text:
	.ascii	"Undefined instruction at %08X!\n(Opcode: %08X)\0"

data_abort_text:
	.ascii	"Data abort at %08X!\nFSR: %08X (domain %d, fault %d)\nAddress: %08X\0"

fiq_text:
	.ascii	"Unhandled FIQ!\0"

reserved_text:
	.ascii	"Hit reserved exception handler!\n(Last known PC: %08X)\0"

syscall_text:
	.ascii	"Unhandled syscall!\0"


.section .icode.usec_timer, "ax", %progbits
.align 2
.global read_native_timer
.type read_native_timer, %function
read_native_timer:
	ldr	r0, val_3c700000
	ldr	r1, [r0,#0x80]
	ldr	r0, [r0,#0x84]
	bx	lr
.size read_native_timer, .-read_native_timer

.global read_usec_timer
.type read_usec_timer, %function
read_usec_timer:
	ldr	r0, val_3c700000
	ldr	r0, [r0,#0xb4]
	bx	lr
.size read_usec_timer, .-read_usec_timer

val_3c700000:
	.word	0x3c700000

.section .ibss,"aw",%nobits
.align 2
.global n4g_reload_boot
n4g_reload_boot:
	.space 4
