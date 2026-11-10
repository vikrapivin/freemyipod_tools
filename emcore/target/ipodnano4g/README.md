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
python3 tools/n4g.py cpu-suite
```

`cpu-suite` picks one row on bank 1, above 1024. It compares the FMSS reader
with the CPU driver, saves the block, programs a pattern from the CPU, and
restores the saved pages. The backup is written under `nand_device_suite/`.

`python3 tools/n4g.py fmss explore` runs the sequencer, NAND and ECC matrices.
