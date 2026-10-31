# Standalone SRAM loader; included by target.mk for kernel builds.
CROSS ?= arm-none-eabi-
N3G_STAGE1_FLAGS = -mcpu=arm926ej-s -marm -ffreestanding -DASM_FILE -DTARGET_ipodnano3g
.PHONY: n3g-stage1
n3g-stage1: build/ipodnano3g/stage1.bin
build/ipodnano3g/stage1.elf: target/ipodnano3g/stage1.S target/ipodnano3g/stage1.ld
	@echo [CC]     $<
	@mkdir -p $(dir $@)
	@$(CROSS)gcc $(N3G_STAGE1_FLAGS) -c $< -o build/ipodnano3g/stage1.o
	@$(CROSS)ld -T target/ipodnano3g/stage1.ld -Map build/ipodnano3g/stage1.map -o $@ build/ipodnano3g/stage1.o
build/ipodnano3g/stage1.bin: build/ipodnano3g/stage1.elf
	@$(CROSS)objcopy -O binary $< $@
