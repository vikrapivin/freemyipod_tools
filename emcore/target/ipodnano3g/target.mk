CFLAGS_ipodnano3g += -mcpu=arm926ej-s
ifeq ($(N3G_USB_PHY_APPLE),1)
CFLAGS_ipodnano3g += -DN3G_USB_PHY_APPLE
endif
# No UCL image: like the nano 5G, the kernel is packaged with a stage-1 SRAM loader (stage1.S) in an
# IMG1 "8702" container built by tools/nano3g/image.py.
TARGET_OUTPUT_ipodnano3g := build/ipodnano3g/emcore-nano3g.dfu build/ipodnano3g/emcore-nano3g.bin build/ipodnano3g/emcore-nano3g.nm
include target/ipodnano3g/stage1.mk

# FMSS sequencer programs of the NAND backend "fmss", assembled on the host
# with tools/fmssasm.c in its nano 3G mode (-3) into a header of word arrays
# (fmss3g_readid, fmss3g_read, fmss3g_program, fmss3g_erase).
N3G_NAND_DIR := target/ipodnano3g/nand
N3G_NAND_BUILD := build/ipodnano3g/nand
FMSSASM3G := build/host/fmssasm3g
FMSS3G_SRC := $(N3G_NAND_DIR)/fmss/readid.s $(N3G_NAND_DIR)/fmss/read.s $(N3G_NAND_DIR)/fmss/program.s $(N3G_NAND_DIR)/fmss/erase.s
CFLAGS_ipodnano3g += -I$(N3G_NAND_BUILD)

$(FMSSASM3G): tools/fmssasm.c
	@echo [HOSTCC] $<
	@mkdir -p $(dir $@)
	@$(HOSTCC) -O2 -o $@ $<

$(N3G_NAND_BUILD)/fmss-nano3g.h: $(FMSS3G_SRC) $(FMSSASM3G)
	@echo [FMSSASM] $@
	@mkdir -p $(dir $@)
	@$(FMSSASM3G) -3 -p fmss3g_ -i '"libc/include/stdint.h"' -s $@ $(FMSS3G_SRC)

build/ipodnano3g/$(N3G_NAND_DIR)/nand-fmss.o: $(N3G_NAND_BUILD)/fmss-nano3g.h

.PHONY: n3g-fmssasm
n3g-fmssasm: $(FMSSASM3G)

# Container for the off-device signing step: format 4 (plain body) or 3 (signer encrypts), certificate slot size.
N3G_IMG1_FORMAT ?= 4
N3G_IMG1_CERT_LEN ?= 0x300

# Grouped outputs ensure either missing image is regenerated (GNU Make 4.3+).
build/ipodnano3g/emcore-nano3g.dfu build/ipodnano3g/emcore-nano3g.bin &: build/ipodnano3g/stage1.bin build/ipodnano3g/emcore.bin tools/nano3g/image.py tools/n3g.py
	@mkdir -p build/ipodnano3g
	$(PYTHON) tools/n3g.py image --stage1 build/ipodnano3g/stage1.bin --payload build/ipodnano3g/emcore.bin --stage1-elf build/ipodnano3g/stage1.elf --kernel-elf build/ipodnano3g/emcore.elf --nm $(CROSS)nm --format $(N3G_IMG1_FORMAT) --cert-len $(N3G_IMG1_CERT_LEN) --raw build/ipodnano3g/emcore-nano3g.bin -o build/ipodnano3g/emcore-nano3g.dfu

build/ipodnano3g/emcore-nano3g.nm: build/ipodnano3g/emcore.elf
	@$(CROSS)nm -n $< > $@

.PHONY: nano3g n3g n3g-image n3g-kernel
nano3g n3g n3g-image: ipodnano3g
n3g-kernel: build/ipodnano3g/emcore.bin
