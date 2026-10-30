CFLAGS_ipodnano3g += -mcpu=arm926ej-s
ifeq ($(N3G_USB_PHY_APPLE),1)
CFLAGS_ipodnano3g += -DN3G_USB_PHY_APPLE
endif
# No UCL image: like the nano 5G, the kernel is packaged with a stage-1 SRAM loader (stage1.S) in an
# IMG1 "8702" container built by tools/nano3g/image.py.
TARGET_OUTPUT_ipodnano3g := build/ipodnano3g/emcore-nano3g.dfu build/ipodnano3g/emcore-nano3g.bin build/ipodnano3g/emcore-nano3g.nm
include target/ipodnano3g/stage1.mk

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
