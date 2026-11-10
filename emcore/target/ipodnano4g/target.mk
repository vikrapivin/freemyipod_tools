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
# The generic recipe expands CFLAGS before a target-specific addition is
# visible. This object is the nano 5G FTL: 4096-byte pages, read only, and
# its NAND calls are renamed so they do not collide with the nano 4G driver.
build/ipodnano4g/target/ipodnano5g/nand/ftl-nano5g.o: target/ipodnano5g/nand/ftl-nano5g.c build/version.h
	@echo [CC] $<
	@mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) $(CFLAGS_ipodnano4g) -DNAND_PAGE_SIZE=4096 -DN4G_FTL_READ_ONLY -include target/ipodnano4g/nand/ftl-rename.h -DTARGET_ipodnano4g -DTARGET=\"ipodnano4g\" -DCONFIG_H=\"target/ipodnano4g/config.h\" -DTARGET_H=\"target/ipodnano4g/target.h\" -o $@ $<

TARGET_OUTPUT_ipodnano4g := build/ipodnano4g/emcore-nano4g.dfu build/ipodnano4g/emcore-nano4g.bin build/ipodnano4g/emcore-nano4g.nm build/ipodnano4g/emcore-nano4g.map
include target/ipodnano4g/image.mk
