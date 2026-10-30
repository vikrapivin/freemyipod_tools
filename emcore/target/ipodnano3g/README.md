# iPod nano 3G target

## Build and load

```
make ipodnano3g          # or: make n3g
```

Outputs (no UCL image; packaged like the nano 5G, with a stage-1 SRAM loader in front of the kernel):

| File | Content |
|---|---|
| `build/ipodnano3g/emcore-nano3g.dfu` | IMG1 "8702" image: 0x800-byte header, body, zero signature (0x80) and certificate (0x300) slots |
| `build/ipodnano3g/emcore-nano3g.bin` | the body: `stage1.bin`, padding to `_stage1_end`, u32 kernel length, `emcore.bin` |
| `build/ipodnano3g/emcore-nano3g.nm` | kernel symbol map |

Header (`tools/nano3g/image.py`, layout of the format 4 DFU template, docs/nano3g/02): magic `8702`, version
`1.0`, `+0x07` = 4 (`--format 3` for a signing step that encrypts the body), `+0x08` entry offset 0, `+0x0c` body
length n (16-byte aligned), `+0x14` = n + 0x80, `+0x18` = 0x300, `+0x10` = `+0x14` + `+0x18`. The packager re-checks
the lengths against the mask ROM's rules (`+0x10 >= n + 0x300`, `+0x14 >= n + 0x80`, `+0x14` 16-byte aligned,
`+0x08 <= n`) and limits the file to 0x20000 bytes. The image is UNSIGNED: the header value at `+0x40`, the signature
and the certificates are zero, and the ROM checks them, so it needs the signing step or a loader that accepts it.
The ROM's DFU runs the body at `0x22000000`.

`stage1.S` (at most 4 KiB): masks IRQ/FIQ and the VIC, stops the ROM's still-live DFU USB core (soft disconnect,
core reset), turns MMU and caches off, applies the pin functions (Rockbox's nano 3G table), sets PLL2 and the system
clocks (`CLKCON1 = 0x00404101`, `CG16_SYS` = PLL2), the SM1 divider, the SMx/SM1/timer clock gates, timer E as
the 1 MHz `USEC_TIMER`, initializes the SDRAM (Rockbox's nano 3G `miu_preinit` values), checks two SDRAM
patterns, copies the kernel to `0x08000000` and jumps there. Any timeout or check failure spins with the step
number in r7. It also loads the translation table base (`0x2203c000`) with an identity section map, which the
kernel's crt0 patches before it turns the MMU on: the ROM hands over with an unrelated table base.

## Host tools added for the nano 3G (`tools/n3g.py`)

| Command | What it does |
|---|---|
| `bench cycle [--unvalidated] [--backends sw,fmss] [--load off|on|both] [--seed-run N]` | per backend: physical cycle (save, pattern, verify, restore, verify) and FTL cycle, unloaded and under CPU load; random region (user area, only user-data or erased pages, checked by a read-only spare scan) and a shuffled backend order per test, both from the printed run seed |
| `collect [--out DIR]` | read-only: the archive Rockbox's NAND check produces (`nandcheck-<id>-x<banks>.tar.gz`) for chip validation |
| `write-test [--unvalidated] [--keep] [--resume FILE]` | FTL write test through `ftl_write`/`ftl_sync`: save, pattern, remount + verify, restore, remount + verify (`--keep` / `--resume` across a restart) |
| `nand fault` | the FMSS sequencer's state at its last fault/timeout, including PC samples taken while it ran |
| `reload [--kernel FILE] [--no-verify]` | replace the running emCORE with a new build without DFU: `emcore.bin` is uploaded to the transfer buffer, read back and compared, then the monitor's EXECFIRMWARE (command 24) copies it to 0x08000000 and jumps to it with MMU and caches off; the kernel starts as stage 1 starts it at boot (clocks, SDRAM and the translation table from the first boot stay). Waits for the new emCORE to answer on USB |
| `fmss run PROG.s [--param OFF=VAL] [--timeout-us N]` | run one FMSS program on the sequencer (monitor command 139) and show r0..r7, the PC, CS_IRQ, the register windows 0x000/0x400/0x800/0xc00/0xd00 before and after, and samples taken while it ran. No chip enable is selected unless the program does so, and a program may not write `FMC_CTRL0` / `FMC_CMD` (`--allow-nand`), use `pst` (`--allow-indirect`) or unknown opcodes (`--allow-unknown`). Afterwards the NAND layer needs step 3 again |
| `fmss explore [--only NAME,...] [--json FILE]` | the probe experiment battery: register file mapping, sequencer speed, ALU edge cases (shift kind and range, wrap, the two-operand form), RAM access through `ldi`/`sti`, `pld`/`pst` reach, the control words, `wait` events and their timeout, every opcode, the start word, the register map, DMA, interrupts, the second FMC register set and the ECC engine (strength, `ECC_MODE`, parity-buffer DMA, raw page reads checked against the code in `tools/fmss/bch.py`). Raw records in the JSON. The experiments live in `tools/fmss/` (target-independent, run against a target profile in `tools/fmss/target.py`; this model's is `NANO3G`, `NANO4G` / `NANO5G` are skeletons to fill in); `tools/nano3g/fmssprobe.py` binds them to the nano 3G. `--list` shows which experiments a target can run |
| `nor info [--show-values]` / `nor dump -o FILE [--size N]` | the boot flash (SPI NOR, read only): size (found where the start repeats), which 64 KiB regions hold data, the SysCfg tags (names and sizes; values only with `--show-values`, they can include the serial number) and the file directory at `0x1000`; `dump` writes the whole chip with its SHA-256. Reads through the boot-flash read (monitor command 22) into the NAND worker's result buffer |

Build options: `N3G_USB_PHY_APPLE=1` uses the restore OS's USB PHY power-on instead of the device-verified sequence
(test enumeration before relying on it).

## Startup and memory

`crt0.S` relocates sections and starts emCORE. `ls.x` reserves these regions:

| Region | Address |
|---|---|
| Load image / allocation pool (`INIT`) | `0x08000000`, length `0x01f80000` |
| Kernel SDRAM sections (text, data, bss) | `0x09f80000`, length `0x00080000` |
| SRAM sections and stacks | `0x22000000`, length `0x0003c000` |
| Translation table | `0x2203c000` |

The kernel SDRAM region is only 512 KiB, so large buffers live on the heap
(`memalign`). The NAND FTL tables (about 500 KiB) are allocated by the first
`ftl_init()`; the harness buffers by `n3g_nand_init()`. Register definitions
are in `s5l8702.h` (the FMC block at `0x38a00000` and the NAND clock gates 8 and
12 were added for the NAND code).

The target is 320 x 240 RGB565.

## NAND

`nand/` is a port of the Rockbox nano 3G NAND driver and FTL (GPLv2+, original
copyright headers kept, adapted notes in each file). Nothing touches the NAND at
boot; the worker thread only runs host-requested jobs.

| File | Role |
|---|---|
| `nand/nand-backend.h`, `nand-backend.c` | `struct nand_backend`, backend registry, and the `nand_*` functions of `nand-target.h`, which dispatch to the active backend |
| `nand/nand-swnand.c` | backend `sw`: CPU-sequenced FMC register driver (reset, READ ID, chip table, ECC page reads with bank overlap, program incl. two-plane, erase) |
| `nand/nand-fmss.c`, `nand-fmss.h`, `nand-fmss-plan.h`, `nand/fmss/*.s` | backend `fmss`: the S5L8702's microcode sequencer runs READ ID / read / program / erase (see below) |
| `nand/nand-chip.h` | chip table row, `nand_chip_find()`, `nand_chip_geometry()`, `nand_hw_enable()`: shared by both backends (the table lives in `nand-swnand.c`) |
| `nand/ftl-nano3g.c`, `ftl-target.h` | Whimory VFL + FTL, read and write; 2048-byte sectors |
| `nand/nand-compat.h` | mapping of the Rockbox kernel API (cache ranges, `logf`, panic, alignment) |
| `nand3g.c`, `nand3g.h` | worker thread, status record, monitor commands, benchmark |

Safety defaults: the FTL mounts read-only (`ftl_mount_writable` false) unless step 4 is
given argument 1; chip rows not validated on hardware are never mounted writable and the
raw write benchmark refuses them unless `N3G_BF_UNVALIDATED` is set; the benchmark's write op
needs `N3G_BF_WRITE`. `N3G_NAND_WRITE=0` removes the write jobs; `N3G_NAND_DISABLE`
removes the harness (`HAVE_NAND3G`). A FTL/VFL write failure that cannot be recovered
stops the worker thread only (`PANIC_KILLTHREAD`).

### FMSS backend (`fmss`)

The CPU builds the lists a sequencer program walks (rows, chip-enable indices, DMA addresses per 2 KiB unit,
metadata, result words), writes the parameter RAM (`0x38a00d00 + off`), points `FMSS_PROG` (`0x38a00c04`) at the
program, writes `CS_IRQ = 0xff` and `FMSS_CTRL = 0xfff5`, and waits for the end: the raw VIC1 status bit 22 (line 54,
polled, not enabled) or `CS_IRQ` bit 0 (end) / bit 2 (fault), at most 2 s. A fault or timeout parks the sequencer
(`FMSS_CTRL = 0x30000`, 1 ms, END-only program, 1 ms) and the call returns -1. The programs do the chip-enable
interleaving, ECC, 12-byte spare metadata and DMA themselves, so the on-flash format is the `sw` backend's and the
original firmware's (the FTL mounts data written by either).

| Program | Source | Job |
|---|---|---|
| `fmss3g_readid` | `nand/fmss/readid.s` | RESET all eight CEs, wait `P_DELAY` loops, READ ID of the listed CEs (bytes 0..3) |
| `fmss3g_read` | `nand/fmss/read.s` | batches of pages on distinct CEs: all page loads started first, then per-sector ECC decode, DMA, metadata, result word per page |
| `fmss3g_program` | `nand/fmss/program.s` | single-plane pages, CEs overlapping, status checked only when a CE gets its next page and at the end; `P_FLAGS` bit 0 selects two-plane pairs (80h/11h then 81h/10h, the `sw` backend's mode 8 form) |
| `fmss3g_erase` | `nand/fmss/erase.s` | blocks over several CEs (60h/D0h), status per CE at the end |

The programs are assembled at build time by `tools/fmssasm.c -3` (the FMSS assembler in its nano 3G mode: pld/pst, 3G
register names) through `target.mk` into `build/ipodnano3g/nand/fmss-nano3g.h` (arrays `fmss3g_*`). Batches larger than 32
pages are split into runs; erase runs take each CE at most once so a failure names its entry. `init` probes CE 0..3 like the
`sw` backend (and retries once if CE 0 answers blank), matches the shared chip table, derives the geometry from it.

* Bus timing: `NAND_FMSS_TIMING_SAFE` (default) uses the word the `sw` backend runs with (setup 4, hold 3, no EDO), validated
  on hardware. `nand_fmss_set_timing(NAND_FMSS_TIMING_CHIP, bus_mhz)` instead derives setup/hold/EDO from the chip table's ns
  figures by the original driver's formula; the 3G bus clock is not established, so use it only for experiments.
* `identify()` returns ID bytes 4..7 as 0 (the program keeps bytes 0..3). `reset(bank)` runs the READ ID program (it resets
  every CE).
* Differences from the original driver: no per-run statistics; erase failures name the entry only per run; the read program
  starts each page load as soon as its entry is seen instead of after queueing the whole batch; no cache-program and no
  78h-status program variants (the original selects them per chip mode); no raw (no-ECC) read program.

## Monitor protocol

Common monitor commands are dispatched by `../../usb/usbdebug.c`; host constants are in
`../../tools/nano3g/protocol.py`. Numbering follows the nano 5G target where the operation exists.

| Command | Operation |
|---|---|
| 120 | NAND status: reply `{status record address, size 0x110, result buffer address}` |
| 121 | Step `(n, arg)`: 1 clock gates, 2 pad snapshot, 3 backend init, 4 FTL mount (`arg` 1 = writable) |
| 123 | Read one raw page `(bank, page)`; result: data, 12 meta bytes at `+0x1000`, ECC verdict `+0x1010`, read bitmap `+0x1014` |
| 124 | FTL open `(writable)` (= step 4) |
| 125 | FTL read `(sector, count)`, 2048-byte sectors, at most 128 per call |
| 128 | Writer capabilities |
| 129 | Writer open (needs an FTL opened writable) |
| 130 | FTL write `(sector, count, CRC-32 of the data staged at the result buffer)` |
| 131 | Flush (`ftl_sync`) |
| 135 | Select backend `(0 sw, 1 fmss)`; invalidates steps 3 and 4 |
| 136 | Run the benchmark described in the result buffer |
| 200 | Heartbeat `{busy/state/kind/steps/jobs_done, jobs started}` |

122, 126, 127 and 132-134 (ROM page, reserved, block read, modules) are not implemented and
answer "unknown command". Jobs return promptly from the USB dispatcher; the host polls the status
record. `nand3g.h` defines the status record, job IDs and result layouts.

## Benchmark

`python3 tools/n3g.py bench ...` runs inside the worker, so the numbers exclude USB. The host writes
a 128-byte parameter block (`struct n3g_bench_params`) to the result buffer and sends command 136; the
worker fills `struct n3g_bench_result` at `result+0x100` and per-repeat microseconds at `result+0x400`.
Timing uses the microsecond timer around each backend call only (pattern generation and statistics are
outside the timed region). Counters are 32-bit and wrap; rates are derived on the host.

```
bench read  --backend sw --banks 0,1 --first-block 100 --blocks 1 --pages 16 --batch 16 --repeats 20 --json sw.json
bench read  ... --interleave        # page-major across banks: lets the backend overlap page loads
bench read  ... --scatter --seed N  # shuffled order
bench ftl   --backend sw --first-sector 0 --sectors 4096 --batch 16 --repeats 5 [--scatter]
bench write --backend sw --banks 0 --scratch-blocks 4000:2 --pages 8 --batch 8 --repeats 3 --destructive [--verify] [--two-plane] [--erase-after]
bench compare sw.json fmss.json [--json cmp.json]
```

One command for the whole comparison (op CYCLE, command 136 with `op = 4`):

```
bench cycle [--mib 16] [--backends sw,fmss] [--first-block N] [--batch 32] [--json cycle.json] [--unvalidated]
```

For each backend in turn, on the same region (whole blocks, all banks, page-major across the banks; default 16 MiB
starting in the middle of each bank, never below block 200 where the VFL context lives), inside the worker:
1. save: read every page (data + 12 metadata bytes) into an SDRAM copy; an uncorrectable page or a failed read stops
   the run here, before anything is written;
2. erase the blocks, 3. program a pattern stamped with bank and page, 4. read back and compare every page;
5. erase again, 6. program the saved pages back (pages that were erased stay erased), 7. read back and compare data
   and metadata with the copy.
The restore (5-7) runs whenever step 2 started, also after a failure in 2-4. The tool stops before the next backend
if a restore did not complete, prints each phase's time and rate and the sw-versus-fmss comparison. The FTL's view
is dropped (mount it again afterwards). A power loss between steps 2 and 7 loses the region's original data: the
copy exists only in SDRAM.

Read and FTL runs are read-only. `write` erases and programs the physical blocks `FIRST:COUNT` on each
selected bank and needs `--destructive`; choose blocks that no mounted FTL data or VFL control structure
uses (the tool cannot know). `--two-plane` needs a chip row with two-plane programming (mode 8) and an even
first block and count. `--backend` makes the device select and initialize that backend for the run (and mount
the FTL read-only for `ftl`); without it the already initialized backend is used. Limits: 32 pages per backend
call, 4096 pages per repeat, 64 sectors per FTL call, 1024 recorded repeats.
Reported: pages, bytes, microseconds, pages/s and MB/s (10^6 B), ECC clean/corrected/uncorrectable, erased
pages, largest correction, errors with the first failing bank and page, erase ms/block, program rates, verify
mismatches, FTL sectors/s. `compare` prints B-versus-A speedups (above 1 means B better for every metric) and warns
when the workloads differ.

## Device test procedure

Use the unit with a healthy battery, USB only. Nothing survives a reset: report results before each risky
step. Steps 1-8 are read-only. `n3g.py` lives in `emcore/tools/`; run it from there.

1. Load emCORE (`build/ipodnano3g/emcore-nano3g.dfu`) and confirm the monitor answers: `python3 n3g.py nand status`
   (magic `N3GN`, size 0x110, backend `sw`, no step done).
2. `python3 n3g.py nand step 1`, then `step 2`. Check PWRCON0 bits 8 and 12 clear after step 1 and the
   PCON words (read only).
3. `python3 n3g.py nand step 3` (backend init) then `python3 n3g.py nand id`: the READ ID words per bank,
   geometry and chip row. Stop here if the chip is reported unsupported or a bank is mixed; record
   the IDs.
4. `python3 n3g.py nand rawread 0 0 -o p0.bin` then a page you know holds data (for example bank 0
   page 100 x pages-per-block): check the ECC verdict (clean or corrected, not uncorrectable) and the 12 spare bytes.
5. `python3 n3g.py nand ftl-open` (read-only mount, may take a while; watch progress); the status shows the
   sector count and `writable 0`.
6. `python3 n3g.py nand lread 0 4` and compare sector 0 with the MBR/partition data of the unit.
7. `python3 n3g.py bench read --banks 0,1 --first-block 100 --blocks 1 --pages 16 --batch 16 --repeats 20 --json sw-seq.json`,
   then the same with `--interleave` and `--scatter`; expect zero errors and ECC failures.
8. `python3 n3g.py bench ftl --sectors 4096 --batch 16 --repeats 5 --json sw-ftl.json` (sequential FTL sectors/s).
9. Optional destructive step, only after steps 1-8 are clean and with scratch blocks you have chosen:
   `python3 n3g.py bench write --banks 0 --scratch-blocks FIRST:2 --pages 8 --batch 8 --repeats 2 --verify --erase-after --destructive`.
   Expect `verify_mismatch 0`, `prog_fail 0`. Try `--two-plane` on a mode 8 chip (even first block/count).
10. `fmss` backend, read-only first. Nothing below writes until step 12. Build the target (`make ipodnano3g`), load it, then
    `python3 n3g.py nand backend fmss` (command 135; invalidates steps 3 and 4) and `step 1`, `step 3`:
    expect the same IDs, bank count and chip row as with `sw`. A -7/-8 or a timeout means the engine did not start or
    finish: stop and report (nothing has been written).
11. `rawread` of the pages used in step 4 with `fmss` selected: data, 12 meta bytes, ECC verdict and bitmap must equal the `sw`
    results byte for byte; then `ftl-open` (read-only), `lread 0 4`, and `bench read ... --backend fmss --json fmss-seq.json`
    (plus `--interleave`, `--scatter`), `bench ftl ... --backend fmss`; zero errors/ECC failures, then
    `bench compare sw-seq.json fmss-seq.json`. If a read differs from `sw`, do not continue.
12. Optional, destructive, only after 10-11 are clean: step 9 with `--backend fmss` on the scratch blocks (single plane first,
    then `--two-plane` on a mode 8 chip), `--verify`, `--erase-after`; read the written pages back with the `sw` backend
    (`backend sw`, `step 3`, `rawread`) to confirm the formats agree.
