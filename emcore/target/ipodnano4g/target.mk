CFLAGS_ipodnano4g += -mcpu=arm1176jz-s -mno-unaligned-access
CFLAGS_ipodnano4g += -Ibuild/ipodnano4g/nand

# make N4G_LOWPOWER=1: backlight clamp and 500 mA only after USB configuration.
ifneq ($(N4G_LOWPOWER),)
CFLAGS_ipodnano4g += -DN4G_LOWPOWER
ifneq ($(N4G_LOWPOWER_CLKDROP),0)
CFLAGS_ipodnano4g += -DN4G_LOWPOWER_CLKDROP
endif
endif


N4G_FMSS_DIR := target/ipodnano4g/nand/fmss
N4G_FMSS_SRC := $(N4G_FMSS_DIR)/read.s $(N4G_FMSS_DIR)/program.s $(N4G_FMSS_DIR)/boot_read.s $(N4G_FMSS_DIR)/erase.s
build/ipodnano4g/nand/fmss-nano4g.h: $(N4G_FMSS_SRC) build/host/fmssasm
	@echo "[FMSSASM] $(N4G_FMSS_SRC) -> $@"
	@mkdir -p $(dir $@)
	@build/host/fmssasm -s $@ $(N4G_FMSS_SRC)
build/ipodnano4g/target/ipodnano4g/nand/nand-nano4g.o: build/ipodnano4g/nand/fmss-nano4g.h

TARGET_OUTPUT_ipodnano4g := build/ipodnano4g/emcore-nano4g.dfu build/ipodnano4g/emcore-nano4g.bin build/ipodnano4g/emcore-nano4g.nm build/ipodnano4g/emcore-nano4g.map
include target/ipodnano4g/image.mk
