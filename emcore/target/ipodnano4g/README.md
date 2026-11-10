# iPod nano 4G target

## Build and load

```
make ipodnano4g
```

| File | Content |
|---|---|
| `build/ipodnano4g/emcore-nano4g.dfu` | unsigned IMG1 `8720` image |
| `build/ipodnano4g/emcore-nano4g.bin` | bootstub, kernel length, kernel |
| `build/ipodnano4g/emcore.bin` | raw kernel |

The DFU body stays within 122880 bytes. The image is unsigned.

From `emcore/`, with the monitor running:

```
python3 tools/n4g.py status
python3 tools/n4g.py cpu-suite --one-page
python3 tools/n4g.py cpu-suite
```

`cpu-suite --one-page` finds one logical page on bank 1 from the page spare
(the logical page is greater than 1023). The FMSS
reader and the CPU driver both read
it, the CPU driver programs a pattern, and both readers must return that
pattern. That block is erased for the program and then restored.
`cpu-suite` without that flag saves the whole block, programs a pattern, and
restores the saved pages. The backup is written under `nand_device_suite/`.

`python3 tools/n4g.py fmss explore` runs the sequencer, NAND and ECC matrices.

Logical sectors are 4096 bytes, the same size the nano 5G reader uses. Open the
volume, then read or dump it. Mount does not program or erase.

```
python3 tools/n4g.py ftl-open --backend fmss
python3 tools/n4g.py dump --backend fmss -o /tmp/disk-fmss.bin
python3 tools/n4g.py ftl-open --backend cpu
python3 tools/n4g.py dump --backend cpu -o /tmp/disk-cpu.bin
```

`--backend` selects the physical driver under the same FTL. `fmss` runs the
sequencer programs. `cpu` runs the CPU driver. Changing it closes the mount,
so open the volume again before `lread` or `dump`. Each line prints the
device time in microseconds.
