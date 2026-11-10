# Existing, unmodified SRAM bootstub from the working nano4g bundle.
DFU_MAX_BYTES ?= 122880
N4G_BOOTSTUB ?= target/ipodnano4g/bootstub/bootstub.bin
N4G_IMG1_FORMAT ?= 4
N4G_IMG1_CERT_LEN ?= 0x300

# Like nano3g, build the unsigned IMG1 and its raw body together.
build/ipodnano4g/emcore-nano4g.dfu build/ipodnano4g/emcore-nano4g.bin &: build/ipodnano4g/emcore.bin $(N4G_BOOTSTUB) tools/nano4g/image.py tools/n4g.py build/ipodnano4g/image-config
	@echo [IMG]    build/ipodnano4g/emcore-nano4g.dfu
	@$(PYTHON) tools/n4g.py image --stage1 $(N4G_BOOTSTUB) --payload build/ipodnano4g/emcore.bin --format $(N4G_IMG1_FORMAT) --cert-len $(N4G_IMG1_CERT_LEN) --max-bytes $(DFU_MAX_BYTES) --raw build/ipodnano4g/emcore-nano4g.bin -o build/ipodnano4g/emcore-nano4g.dfu

build/ipodnano4g/emcore-nano4g.nm: build/ipodnano4g/emcore.elf
	@echo [NM]     $@
	@$(CROSS)nm -n $< > $@

# Keep the filename consumed by the existing ladder and second-stage tools.
build/ipodnano4g/emcore-nano4g.map: build/ipodnano4g/emcore-nano4g.nm
	@echo [CP]     $@
	@cp $< $@

.PHONY: n4g-config-force
build/ipodnano4g/image-config: n4g-config-force
	@mkdir -p $(dir $@)
	@echo '$(N4G_BOOTSTUB) $(N4G_IMG1_FORMAT) $(N4G_IMG1_CERT_LEN) $(DFU_MAX_BYTES)' > $@.tmp
	@cmp -s $@.tmp $@ && rm $@.tmp || mv $@.tmp $@

.PHONY: nano4g n4g n4g-image n4g-kernel
nano4g n4g n4g-image: ipodnano4g
n4g-kernel: build/ipodnano4g/emcore.bin
