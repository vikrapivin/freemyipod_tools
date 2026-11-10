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
python3 tools/n4g.py cpu-suite --read-only
python3 tools/n4g.py cpu-suite
```

`cpu-suite --read-only` reads one page on bank 1, chosen at run time above
1024, with the FMSS reader and the CPU driver. It does not erase.
`cpu-suite` without that flag also saves the block, programs a pattern from
the CPU, and restores the saved pages. The backup is written under
`nand_device_suite/`.

`python3 tools/n4g.py fmss explore` runs the sequencer, NAND and ECC matrices.
