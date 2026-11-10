"""M2 experiment migration ledger. Nano3g results never become M2 evidence."""
from fmss.experiments import EXPERIMENTS
from fmss.probe import assemble, probe
from fmss.target import NANO4G
from dataclasses import replace
from .protocol import validate_probe, DISCOVERY
from .m2_experiments import REGISTRY as M2_EXPERIMENTS
from . import discovery as _discovery  # register M2 event/control replacements
from . import control_experiments as _control
from . import write_experiments as _write
from . import isa_experiments as _isa

NANO4G.validate_probe = validate_probe

# These programs use the shared encoding and the M2 probe's four windows.
# Their interpretation still needs M2 device data. No M1 reset/start conclusions apply.
SHARED = set("regmap clock pram alu memory resume control wait c58 op15 op12 c58ill "
             "oneshot c50 c14 op12ack op10 ctrlbits op0f ctrlhold timeregs c08 regnum "
             "branch waitf pause r8 op0916 pcwrite op12cont tmofield tmobits tmobits2".split())
# These use M1 mnemonic spellings solely to emit exploratory opcode/operand
# bytes. No ECC or NAND source is admitted through this spelling adapter.
DISCOVER = set("indirect unknown opsweep optime opcost fmc28 fields0 stprev tmwait2 c58b1 c58b2 "
               "regsweep holdout fmcmap startseq startseq2 startseq3 pending fwidth".split())
DESTRUCTIVE = set("eccchunk rawwrite writeseq eraseseq op09nand eccchunk2 ev5ready pageorder "
                  "op09w33 ev5arm writeseq2 ev5two finale startdiff progtime startregion rschip "
                  "dmactl evsweep3".split())
SMOKE = ("state", "regmap", "alu", "pram", "memory")

# Registry coverage is not a claim that every M1 case has an M2 equivalent.
# Keep remaining experimental work machine-readable and visible in --list.
APPLICABILITY = {
    **{n: "M2 alternate-code selector unknown; candidate ECC_CTRL sweep only"
       for n in ("b15ecc", "rsdecode", "rsresult", "rschip")},
    **{n: "candidate +400 set only; second-channel GPIO/event mapping unresolved"
       for n in ("chan1", "chan1pins", "ev8", "op09d", "ch1buf")},
}
PARTIAL = {
    "r8": "M1 register-role comparison with local PC targets; direct LDI r8 low-address bus access remains unqualified",
    "writeseq": "serialized M2 pages; overlapped pipeline and two-plane cases remain",
    "writeseq2": "sequential/repeat programming; overlap/last-DMA comparison remains",
    "eraseseq": "single-plane/CE/address-cycle cases; two-plane erase remains",
    "eccchunk": "captured on-flash M2 chunks; parity-buffer DMA and host BCH construction remain",
    "eccchunk2": "captured-codeword corruption; mixed-error and direct parity-buffer cases remain",
    "startregion": "region reads and scratch writes; reserved-block writes not selected",
    "evcons": "single waits in pending states; ordered repeated-wait combinations remain",
    "op09c": "condition ranges ported; CE deselect/CE1 comparison remains",
    "w2state": "basic M2 states ported; cold-boot/enable/control-word sequence remains",
    "evsweep2": "forward state sweeps; reverse ordering remains",
    "evsweep3": "M2 metadata/ECC/program/DMA; candidate second-channel/parity DMA remains",
    "w2vic": "raw VIC comparison; enable-source combinations remain",
    "ev5t": "repeated M2 latch comparison; delay/command-ack timing matrix remains",
    "ev5arm": "M2 rearm candidates; short-read and data-select variants remain",
    "op16b": "C58/pending states; delayed flag and vector target variants remain",
    "c50x": "timeout/opcode/unaligned scratch; arbitrary faulting bus addresses omitted",
    "ctrlfx": "timeout/opcode/event-jump cases; enabled command IRQ case remains",
    "ctrlwords": "word/DMA/ECC matrix; nested fault/vector-offset cases remain",
    "c08self": "M2 operation state captures; pause/continue/vector cases remain",
    "bufport": "candidate port reads; CPU/sequencer FIFO write experiment remains",
    "seqrestart": "M2 resets/acks; within-program three-encode comparison remains",
    "fmcbits": "known-page control/DMA variants; READID variants remain",
    "dmactl": "serialized read variants; original overlapped read remains",
    "restore": "native M2 reset; power-on defaults require cold-device capture",
}


def state(s, t):
    """Snapshot the four M2 windows around END; excludes FIFO reads, not a power-on baseline."""
    rec = probe(s, t, assemble(t, "end\n"))
    return rec, "M2 snapshot: %s; preflight %s" % (rec["why"], rec["pre_ok"])


def readid(s, t):
    """M2 READ ID on CE0..3 using FIFO command +0x40 and data +0x60 (read-only)."""
    from .nand import check
    from .protocol import Cmd
    # Pins are deliberately separate from the no-flash smoke suite.
    check(s.job(Cmd.NAND_STEP, 2, 0))
    records = {}
    for ce in range(4):
        src = t.prologue() + """
    wr [FMSS_C10], #1
    wr [FMCTRL1], #0x33338e0
    wr [FMC_STATUS], #0xff00ffe
    wr [FMC_CTRL0], #0x%x
    wr [FMC_CMD], #0x90
    wait 0
    wr [FMC_STATUS], #2
    wr [FMC_ANUM], #0
    wr [FMC_ADDR0], #0
    wr [FMCTRL1], #1
    wait 1
    wr [FMC_STATUS], #4
    wr [FMC_DNUM], #4
    wr [FMC_DATA_SEL], #0x10
    wr [FMCTRL1], #0xe2
    wait 3
    wr [FMC_STATUS], #8
    wr [0x40], #0x52
fifo:
    ld r1, [0x40], 2
    bnz r1, fifo
    ld r0, [0x60]
    wr [FMC_CTRL0], #0
    movi r6, #0x600d600d
    end
""" % (0x77001 | 2 << ce) + t.trap()
        rec = probe(s, t, assemble(t, src))
        records[ce] = rec
        if rec["why"] != "END":
            from .protocol import ToolError
            raise ToolError("READ ID failed on CE%d; stop this boot" % ce)
    return records, "IDs: " + " ".join("CE%d=%08x" % (ce, r["r"][0]) for ce, r in records.items())


from .sweep_experiments import opsweep

OVERRIDES = {"state": state, "readid": readid, **M2_EXPERIMENTS, "opsweep": opsweep}


def discovery(fn):
    def run(s, t):
        raw_target = replace(t, asm_flags=("-3",), probe_flags=DISCOVERY)
        return fn(s, raw_target)
    return run


def entries(include_followups=False):
    for ex in EXPERIMENTS:
        if ex.name in OVERRIDES:
            status, reason, fn = "ported", "M2-specific replacement", OVERRIDES[ex.name]
        elif ex.name in SHARED:
            status, reason, fn = "ported", "shared ISA; M2 reset/start/snapshots", ex.fn
        elif ex.name in DISCOVER:
            status, reason, fn = "ported", "raw opcode discovery; M1 meanings are hypotheses", discovery(ex.fn)
        elif ex.name in DESTRUCTIVE or "destructive" in (ex.doc or "").lower():
            status, reason, fn = "needs-adaptation", "write experiments authorized; M2 case matrix still needs porting", None
        else:
            missing = ex.missing(NANO4G)
            status, reason, fn = "needs-adaptation", ("M1 assumptions: " + ", ".join(missing) if missing else
                                                     "M1 setup, unsafe sweep operands, or interpretation"), None
        if ex.name in APPLICABILITY:
            status, reason = "applicability", APPLICABILITY[ex.name]
        elif ex.name in PARTIAL:
            status, reason = "partial", PARTIAL[ex.name]
        from .resident_plan import resident
        if fn is not None and ex.name != 'opsweep' and ex.name not in _discovery.EVENT_PLANS:
            fn = resident(ex.name, fn)
        yield {"name": ex.name, "status": status, "reason": reason, "question": ex.doc, "fn": fn,
               "source": ex.fn.__module__.replace(".", "/") + ".py",
               "resident": True}
    yield {"name": "readid", "status": "ported", "reason": "M2 FIFO sequence", "question": readid.__doc__,
           "fn": resident("readid", readid), "resident": True, "source": "nano4g/experiments.py"}

    if include_followups:
        from .followup_experiments import entries as extra_entries
        yield from extra_entries()
        # CPU-direct NAND jobs between resident sequencer phases (non-probe function family)
        from .cpu_nand_experiments import entry as cpu_nand_entry
        yield cpu_nand_entry()
        # CPU-only NAND driver (cpu_nand_stack.c) against the sequencer driver, same family shape
        from .cpu_stack_experiments import entry as cpu_stack_entry
        yield cpu_stack_entry()
        from .cpu_direct_experiments import entry as cpu_direct_entry
        yield cpu_direct_entry()
        from .driver_compare import entry as driver_compare_entry, profile_entry
        yield driver_compare_entry()
        yield profile_entry()


def resolve(names):
    from .protocol import ToolError
    index = {e["name"]: e for e in entries(include_followups=True)}
    selected = []
    for name in names:
        if name not in index:
            raise ToolError("Unknown experiment: " + name)
        if index[name]["fn"] is None:
            raise ToolError("%s: %s" % (name, index[name]["reason"]))
        selected.append(index[name])
    return selected


# Core controls first, then complete opcode families (including their event
# controls), status recovery, and the remaining legacy peripheral matrices.
# Never reorder cases within a family: captured-data dependencies stay valid.
ISA_FIRST = ('m2_cold_boot', 'm2_m1_poweron_values', 'm2_recovery', 'state', 'regmap', 'alu', 'memory', 'indirect',
             'pram', 'm2_shifts', 'm2_pram_alias', 'm2_opcode_confirm',
             'm2_opcode_scan', 'm2_opcode_m1', 'opsweep', 'm2_opcode_states', 'm2_status_recovery',
             'm2_cpu_direct')  # full physical map precedes the round-6 Apple write trials


# CPU writes to C60..C6C while a program runs can hold the engine (p26); the
# cleanup recovers C60 (p29), but these families still run after all others.
RUN_LAST = ('m2_wait_cpu2', 'm2_c60_hold', 'm2_wait_cpu')  # m2_wait_cpu leaves C64..C6C set: very last


def run_last():
    """Risky M1-comparison families (module RUN_LAST tuples), then the C60..C6C families.

    A family that may wedge the engine beyond the probe reset runs after every
    other family, but before the already measured C60..C6C hold families."""
    from . import followup_experiments  # noqa: F401  (registers the modules below; avoids a circular import)
    from . import m1diff_isa, m1diff_fmc, m1diff_ecc, open_experiments, m1diff_followup, round3_experiments, round4_experiments, round5_experiments, round6_experiments
    risky = [n for m in (m1diff_isa, m1diff_fmc, m1diff_ecc, open_experiments, m1diff_followup, round3_experiments, round4_experiments, round5_experiments, round6_experiments) for n in getattr(m, 'RUN_LAST', ())]
    # A bus error to an unmapped address may need a power cycle: last of the risky group.
    risky.sort(key=lambda n: n == 'm2_m1_bus_error')
    return tuple(risky) + RUN_LAST


def campaign_order(selected):
    priority = {name: index for index, name in enumerate(ISA_FIRST)}
    last = {name: index for index, name in enumerate(run_last())}
    return sorted(selected, key=lambda e: (e['name'] in last, last.get(e['name'], 0),
                                           priority.get(e['name'], len(priority))))
