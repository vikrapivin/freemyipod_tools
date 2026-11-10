CFLAGS_ipodnano4g += -mcpu=arm1176jz-s -mno-unaligned-access

# make N4G_LOWPOWER=1: backlight clamp and 500 mA only after USB configuration.
ifneq ($(N4G_LOWPOWER),)
CFLAGS_ipodnano4g += -DN4G_LOWPOWER
ifneq ($(N4G_LOWPOWER_CLKDROP),0)
CFLAGS_ipodnano4g += -DN4G_LOWPOWER_CLKDROP
endif
endif

TARGET_OUTPUT_ipodnano4g := build/ipodnano4g/emcore-nano4g.dfu build/ipodnano4g/emcore-nano4g.bin build/ipodnano4g/emcore-nano4g.nm build/ipodnano4g/emcore-nano4g.map
include target/ipodnano4g/image.mk
