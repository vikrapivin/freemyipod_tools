# nano4g experiment tools

## P16 results and next visibility check

P16 executes all172 families/31,001 native cases without reset/cleanup failures.
The lone scored mismatch repeats the known M1 source10 difference. All24 DMA/ECC
controls END but fail full payload comparison, including the serial controls.
The next24-case `m2_dma_visibility` checks full output and a pre-cleanup sequencer
checksum with CPU-timed holds. Copy updated tools; keep the existing bit27 kernel:

```sh
python3 tools/n4g.py fmss explore --only m2_dma_visibility --usb bulk \
  --image build/ipodnano4g/emcore.bin --json ../experiments/m2-dma-p17.json.gz
```

One resident batch, both BCH8/BCH16 plus plain DMA, no BCH option or reload.
New default summaries use lossless gzip. Explicit `.json.gz` keeps every decoded
observation without the huge plain JSON; `.json` is still accepted. The raw

## Throughput: fault windows, checkpoints and timing

Campaigns (`--all`, `--followups`, `--opcodes`) now shorten the ~126.5 ms
hardware WAIT timeout for cases where p16 shows that timing is not the
question. The start word's bits 15:8 set that timeout ((count + 1) × ~494 us,
p16 `startbits`); qualified cases start with `0x20b5` (predicted 16.3 ms) chosen by
`fault_window.py` from `fault_window_p16.json`. A case keeps `0xffb5` when its
family is a timing family, has no p16 evidence, its requests changed since
calibration, or when the case has an explicit start word, timed CPU writes,
NAND mutation, a short observation window, pause/tmwait/evjmp/jr/unknown
opcodes or a late-releasing WAIT2 (8, 16, 19). Every 64th eligible case is a
long control. Each affected record has `fault_window` (start word, predicted
and measured fault time); deviations print a warning. `--fault-margin 0`
restores the hardware timeout everywhere; `--fault-floor-us` (default 16000)
sets the shortest window. Regenerate the table from a new capture with
`python3 -m nano4g.fault_window CAPTURE.json[.gz]` (run in `tools/`).

The readable summary is written once at the end (and at most every
`--checkpoint-seconds`, default 300). After every family the journal gets an
append-only `family-result` record; `python3 tools/n4g.py fmss rebuild
RUN.jsonl.gz --json RUN-rebuilt.json.gz` recreates the summary from the journal
alone. `--checkpoint-seconds 0` restores the old per-family rewrite.

The summary now records `timing` (wall time, per-family plan/prepare/upload/
run/download/decode/report/store/checkpoint seconds and device worker/program
time) and per-family `reset_health` (cases whose reset reached phase 6 and whose
END-only preflight completed, cases skipping either by request, and the names
of every failed reset, preflight or cleanup). See

## Full run with stateful recovery

P16 qualifies recovery across the full campaign. The next focused command is
the visibility check above. The general full-registry command remains below.

Build/reload/run from `emCORE/`:

```sh
make ipodnano4g -j4
python3 tools/n4g.py reload
python3 tools/n4g.py fmss explore --all --usb bulk --bch 8 \
  --image build/ipodnano4g/emcore.bin --json ../experiments/m2-all-next.json.gz
```

The kernel must advertise reset-while-held bit 27, reset-phase bit 24, recovery bit 23 and mutation bit
18. Normal reload enables mutation. `--all` runs pause controls, core ISA and
opcode confirmation/discovery early, followed by the remaining families:
173 entries / 31,019 baseline cases. It continues after case errors. Each
experiment runs its cases on the device; there is no per-case USB handshake.

The observation minimum remains 160,000 us, above the ~126.6 ms idle fault
measured on M2. Early END/fault completes immediately; existing longer windows
remain longer. `--observation-min-us 0` preserves original windows for comparison.

Batch output v3 adds a 196-byte diagnostic. Stage identifies initial reset (1),
preflight (2), candidate start (3), cleanup (4) or completion (5). Each reset
retains its original eight words plus `first_failure` and `final_state`, each
containing phase, C00, C08, C60, C64, C6C, FMCTRL1 and FMC_STATUS. Reset phases
are stop/idle (1), abort (2), buffer acknowledgement (3), FMC acknowledgement
(4), ECC reset (5), complete (6). Readers retain v1/v2 compatibility.

Recovery method bits are trusted-END resume (1), abort fallback (2) and status
release (4); zero means ordinary reset. An idle/aborted engine whose reset fails
in phase 3/4 gets one status-release attempt (`FMCTRL1=0x30020`, acknowledge
`0x1800000`), then another bounded normal reset. Failed-phase snapshots survive
successful recovery. Candidate observations are saved before cleanup and the
candidate is never rerun. The release sequence comes from the original driver;
its effectiveness on the p07 physical failure still needs the next capture.

The readable JSON uses `capture_format=compact-v1`. It retains every native
case and all sampled observations, including faults/timeouts. Unexecuted cases
omit invalid snapshot/scratch payloads, explicitly list the omitted fields,
and retain reset diagnostics. The adjacent **`.jsonl.gz`** preserves raw request
and result bytes losslessly. Preserve both files and the raw kernel identified
by `--image`; that option records a hash, it does not upload firmware.

This addresses p07's two distinct problems: a persistent peripheral reset
failure after `ev5`, and large repeated empty captures. On the supplied file,
the derived summary is 27,542,946 bytes versus 641,793,213 bytes originally;
lossless gzip of the original is 13,866,279 bytes. The original evidence is
untouched. A successful campaign with real samples can be larger.

`m2_status_recovery` includes the exact first p07 `ev5` instruction stream,
contrasts reading NSTAT before/after WAIT5, and follows each candidate with a

## Build and protocol reference

The nano 4G startup and SoC code is in this tree. Shared kernel and USB
changes are target-gated. The physical NAND driver and its programs are here
too. Only that physical layer and a small emCORE adapter are included. There
is no imported FTL. Nano 4G and nano 5G share the reset, start and wait
helpers, the host assembler and the erase program. Pin mux, clocks, geometry
and the other parameter values stay local to the 4G.

```sh
make -C emCORE ipodnano4g -j4
python3 emCORE/tools/n4g.py fmss explore --list
```

The normal Make target packages the binary just like nano3g. `nano4g`, `n4g`
and `n4g-image` are aliases; `n4g-kernel` builds only the raw kernel.
Rolling outputs in `emCORE/build/ipodnano4g/`:

| File | Contents |
|---|---|
| `emcore-nano4g.dfu` | Unsigned `8720 2.0` IMG1, format 4, **0x600-byte header** |
| `emcore-nano4g.bin` | Raw IMG1 body: retained 2580-byte bootstub, u32 kernel length, kernel, trailing 16-byte alignment |
| `emcore-nano4g.nm` | Sorted kernel symbols, as in nano3g |
| `emcore-nano4g.map` | Same symbols under the name used by existing device/model tools |
| `emcore.bin`, `emcore.elf` | Same raw kernel used by DFU packaging and reload, and linked ELF |

Nano3g uses an **0x800-byte** `8702 1.0` header. Nano4g's body starts at
file+0x600, with a zeroed 0x80-byte signature slot and 0x300-byte certificate
slot following the aligned body. The packager follows the S5L8720 checks in
, image validation:
certificate offset = body length + 0x80; data length <= 0x24c00.
The raw-body build cap remains 122880 bytes. `N4G_IMG1_FORMAT` and
`N4G_IMG1_CERT_LEN` expose the same packaging choices as nano3g.

Like nano3g's output, the `.dfu` is **unsigned** and needs signing or a loader
that accepts unsigned images. The existing `tools/n4g_sign.py` takes the raw
`.bin`, not the unsigned `.dfu`, and builds the legacy compact pwnage image
using a live emCORE's AES service. These are distinct container layouts.
was not recorded.

Like nano3g, the normal command builds everything needed for both entry paths:

```sh
make -C emCORE ipodnano4g -j4
python3 emCORE/tools/n4g.py reload
```

Reload uploads **the same `emcore.bin`**, reads it back, then uses the shared
nano3g `EXECFIRMWARE` command-24 workflow. No separate stage2 kernel, trampoline,
or running-image map is needed. The common nano4g entry stops DMA, performs the
ARM1176 cache/TLB maintenance and retains the active bootstub translation table.
It records the incoming SCTLR.M: the retained DFU stub enters with M=1;
`execfirmware` enters with M=0. This distinguishes the initial DFU boot from
reload for mutation capability, preserving the existing second-stage write
workflow with one binary. It is an experiment guard, not a security boundary.

`make -C emCORE clean` removes the entire `build/` directory, including all
images, maps, host assemblers, generated FMSS headers, objects, configuration
stamps and logs/results stored there. It also removes host Python caches and
works without a cross compiler. Keep device evidence outside `build/` before
cleaning. The checked-in bootstub and other source inputs remain. Clean does
not change source timestamps: clock-skew warnings that persist after cleaning
require checking the host clock and copied source timestamps.

## Resident follow-up execution

All 172 registry entries run as resident batches by default: upload a complete
experiment family once, run its cases in the worker, collect packed results.
This includes `m2_wedge` (59 cases) and `m2_regbus` (3 cases), which previously
had implicit per-case collection. `--isolate-diagnostics` now explicitly opts
those two families into per-case jobs. `--single-probes` explicitly requests
individual probes for every family; the two options are mutually exclusive.

`--all` uploads one family at a time. The host still compiles manifests, polls
progress and decodes reports. When a result arena fills, it collects that page
and resumes the same resident manifest without re-uploading it. These page
boundaries are not per-case dispatch. Startup and saved `experiment_transport`
metadata identify the selected execution mode. The routing correction alone was host-only. The subsequent p13 reset fix
and its p14 ordering correction require the rebuilt kernel (capability bit 27); old kernels are rejected
before the campaign. Reload the new image first.
dependencies, compression, continuation and validation.

The updated kernel also enables bulk USB for manifest uploads and capture
downloads. Copy the updated host tools and reload; the runner prints
`USB transport: bulk`. `--usb bulk` requires it explicitly; `--usb control`
selects the old slow path for comparison. Normal reload can bootstrap an older
kernel over control transfers. Per-experiment timings separate upload, execution,
for the transport correction and packet-model checks.

## p06 result qualification

The p06 run used resident batches and bulk USB, but its reported capabilities
`0x7a0000` lack mutation bit 18. All 159 entries are present; only 140 retained
records contain sampled execution. Opcode12 exposes a reset/cleanup gap, and
most later output records were initialized but never populated by an executing

The current recovery revision adds probe-side pause cleanup and diagnostics.
The original shared physical-driver reset remains unchanged; normal nano5g
behavior is unaffected.
Preserve the adjacent gzip JSONL as well as the summary: legacy report replay can
omit setup probes or replace duplicate labels. `--image` records a host-supplied
hash; it does not choose or upload firmware. p06 did not record one.

## CPU NAND read/write suite

With the nano 4G in the monitor and writes enabled, from `emCORE/`:

```sh
python3 tools/n4g.py cpu-suite
```

The command chooses one page on bank 1 when it runs. The page is greater
than 1023. It reads that page with the FMSS reader and the CPU driver. It
erases only after those reads match and every page of the block is in
`nand_device_suite/` at the top of this tree. The command prints that path.
The CPU writer programs one pattern; both readers have to return it. The
CPU writer then puts the saved pages back, and the FMSS reader checks the
block. The backup file stays. A failed restore is retried with the path the
command printed:

```sh
python3 tools/n4g.py cpu-suite --restore <printed-backup>.n4bk
```

## Device sequence

Begin with the coprocessor baseline.
READ ID characterizes the attached NAND separately. The full registry combines
sequencer, SoC-peripheral and NAND tests; its geometry flags are workload
parameters, not prerequisites for understanding the ISA or control state.

Use the healthy unit. Device commands below run from `emCORE/`:

1. Start the existing recovery emCORE if necessary; sign the new payload with
   `tools/n4g_sign.py`; enter DFU and send the resulting signed image.
2. Verify transport/memory/cache/thread primitives using the retained
   `tools/n4g_ladder.py` and this build's map. Historical tail-memory ranges are
   not a substitute for checking the current layout; start with rungs 0..4.
3. Run `tools/n4g.py status`. Run `fmss explore --only
   state,regmap,alu,pram,memory` and then `--only readid`.
4. Run `python3 tools/n4g.py reload`. It uses `build/ipodnano4g/emcore.bin` by
   default; `--kernel PATH` selects another raw kernel. Wait for reconnect and
   confirm `tools/n4g.py status` reports reload mutation enabled. The native
   probe and worker reject mutation on the initial DFU boot.
5. Full registry invocation, including write entries:

```sh
python3 tools/n4g.py fmss explore --all --scratch-ce 0 --scratch-block 3000 --bch 8 \
  --image build/ipodnano4g/emcore.bin --json experiments/m2-all.json
```

The user authorized writes on their unit. CE/block selection is explicit;
some matrices also touch the next block or next CE (see the NAND chapter).
An applicability entry records what it tested; completing it does not resolve
the original M1 question automatically. Some probes intentionally investigate
faults/stalls. Completed worker -2 errors retain initialized probe output,
which may have no execution samples;
failed preflights and entry exceptions are recorded. The runner attempts the
next independent experiment without replaying the failed command. New follow-up
matrices also continue after individual case errors. A preflight failure does
not establish execution of the requested program. Connection/session setup
errors still prevent submission; per-experiment transport errors are recorded.

Updated `--all` includes 3,133 additional cases, including 2,136 opcode checks.
`--opcodes` selects the four opcode families alone; see opcode confirmation. To resume the supplied interrupted
campaign, use `--all --from op12`; to run only the new matrices, use `--followups`.
for commands and coverage. The default result directory is `experiments/`
relative to the working directory. Keep the JSON summary and adjacent JSONL:
the latter retains each submitted request and returned result immediately.

## Protocol and records

The current USB transport uses the retained nano4g status ABI (command 120),
step worker (121), read APIs (122/123), FMSS job 137, and capability query 138.
Legacy FTL commands 124..126 and legacy write commands 129..136 are unsupported;
128 reports no legacy staging API. No filesystem/storage stack mounts at boot.
Only clock/pin prerequisites are automatic. Step 4 explicitly resets/identifies
the NAND through the adapted physical driver. Probe jobs invalidate that step
so a subsequent native read must initialize the driver again.

The fixed status ABI remains 0x250 bytes; unused logical-layer fields are zero.
The ECC/spare fields describe the driver's current mode: it starts BCH8 and
retains BCH16 only after a programmed-page retry succeeds. An erased page or
initial ID query alone does not prove spare geometry. READ_PAGE keeps the old
data/metadata offsets, eight per-sector status bytes and a zero reserved
maximum-correction byte; it rejects the old uncached-alias flag.

Within the existing 256 KiB result allocation:

| Offset | Contents |
|---|---|
| `00000` | request v4, magic `34525046`, at most 96 register writes |
| `00400` | assembled program, eight-byte instruction alignment |
| `20000` | output v14, magic `3452504f`, four snapshots, up to 1024 samples |
| after samples | optional before/after 4 KiB CPU maps |
| `38000..3ffff` | 32 KiB DMA/scratch data |

Request CRC is checked by the worker before execution. Cache maintenance wraps
the scratch region. Parameters may include two timed batches of CPU writes.
The probe executes an END-only preflight, uses the M2 buffer handshake, records
PC/r0..r7/IRQ and raw VIC words, then stops the engine before returning memory.
Timeouts are bounded at two seconds. SRAM execution is limited to 4096 bytes.

Mutation flag bit 18 permits NAND program/erase commands. Discovery flag bit 19
permits unqualified computed-register instructions and candidate register sets.
These checks catch accidental runner misuse; raw microcode is **not a security
sandbox**. M1 channel-pin remux and the five-window ABI are rejected. Full-map
capture is supported independently.

Every probe journal has a flushed/fsynced **plan before submission**, containing
the complete request, scratch bytes, SHA-256 and experiment name. Results are
flushed as soon as USB returns them. A failed experiment therefore does not
erase earlier evidence. Image identity is host-supplied and labelled as such;
it is not a firmware-attested hash.

## p09 wedge isolation

After communication is restored and the current kernel is loaded, run from
`emCORE/`:

```sh
python3 tools/n4g.py reload
python3 tools/n4g.py fmss explore --only m2_wedge --isolate-diagnostics --image build/ipodnano4g/emcore.bin --json ../experiments/m2-wedge-isolated.json
```

With the explicit isolation option, this diagnostic runs 59 native requests, collecting each before
submitting the next. The JSON stores the outstanding request and every completed
case; the gzip journal stores exact bytes and transitions. It tests corrected
regnum operands, r8 read locations, mapped RAM loads and following END markers.
No BCH selection is required. Without that option all 59 cases execute in one
resident batch. The current full registry has 173 entries / 31,019 baseline cases.

A USB loss makes later entries `not_submitted`, instead of generating another
timeout for each name. Completed probe errors still allow independent work.
No timed-out job is replayed and live batch arenas are not freed. Capability
bit 25 advertises cache-cleaned completed-prefix progress; status polls journal
the last seen case, which need not be the final case reached before a wedge.
An already wedged device needs the existing DFU boot path before reload works.
and the distinction between a physical cause and injected model faults.

## P12 event campaign (completed in P16)

P12 completes all 72 cases. All six ALUs use self-source through selector 10 in
the tested ordinary-GPR forms, including distinguishing small shifts. STOP
clears PC; STOP then CTRL=80000 clears GPRs/PRAM and the measured control words.
The already-rebuilt adapter correctly resets before setting parameters.

The historical command below selected the157 cases that p13 could not execute.
P16 has now executed all of them; do not repeat this subset as the next run.

```sh
python3 tools/n4g.py fmss explore \
  --only m2_waitcons,m2_status_order,m2_ecc_sequence,m2_dma_ecc_overlap \
  --image build/ipodnano4g/emcore.bin \
  --json ../experiments/m2-events-p16.json
```

**157 cases in four resident device batches**, one command. Both BCH8 and BCH16
encoder configurations are included, so no BCH option is needed. No new build
or reload is needed after p15. Keep the JSON and `.jsonl.gz`.

The event matrices compare repeated waits, acknowledgement after completion,
rearm and status-latch ordering. The ECC/DMA matrices compare repeated encodes
and same/different slots. Model prevalidation uses explicit WAIT15 alternatives;
it does not establish WAIT15 behavior, actual overlap timing or physical parity.

`--registers` still repeats 1,654 cases, retaining the original M1 prediction
mismatch for source 10. There is no need to repeat it for p13.

## P15: reset correction verified

P13 leaves evjmp at C08=b with start=1; STOP fails and the normal reset never
reaches its reset write. P14 showed that abort passed its status check without
clearing the wait. P15 verifies the corrected ordering: abort reports3, forced
reset before another STOP reports0, and the full ordinary reset succeeds.
Both inherited and fresh evjmp states recover; all seven candidates execute.
Method01000318 retains both intermediate status measurements. All four marker
programs reach END with r2=72657365; the three trigger TIMEOUTs are expected.

The seven-case check remains available for reproducing a reset regression:

```sh
python3 tools/n4g.py fmss explore --only m2_resetcheck \
  --image build/ipodnano4g/emcore.bin --json ../experiments/m2-reset-repeat.json
```

It is one resident batch; repeating it is not required before the event campaign
above. Recovery is qualified for the tested zero-field evjmp/start1 sequence,
not for arbitrary control words or operand forms.
