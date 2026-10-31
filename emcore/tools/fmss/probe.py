# Copyright (C) 2026 Vikotr Krapivin
#
# This file is part of emCORE.
#
# emCORE is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# emCORE is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with emCORE. If not, see <http://www.gnu.org/licenses/>.


"""FMSS probe core: request / response packing, probe(), the assembler, snapshots and the soft reset.

The device job (a monitor command per model) runs one program and records r0..r7, the PC and CS_IRQ while it runs,
and the register windows of the target before and after. Everything model-specific comes from the Target passed in.

FMSS_PROGRAM_LOG=FILE (debug): append one JSON line per probe() call (experiment, call number, sha256 of the request,
length of the scratch bytes written); two runs of the tool that print the same log sent the same programs.
"""
import hashlib
import importlib
import json
import os
import struct
import subprocess
import tempfile
import zlib

from nano3g.session import ToolError       # one error type for the host tools; the transport library is shared

# flags of the request header
F_KEEP_CTRL0, F_WIN400, F_SAMPLE_REGS = 1, 2, 4
# start / end steps (device default: reset, clean, +0x400 = 0, preflight before; park after)
S_NO_CLEAN, S_NO_PREFLIGHT, S_NO_RESET, S_NO_400, S_END_NOCLEAN, S_END_RESET = (1 << 3, 1 << 4, 1 << 5, 1 << 6,
                                                                                1 << 7, 1 << 8)
SNAP_SKIP = {0x000: 1 << 9, 0x400: 1 << 10, 0x800: 1 << 11, 0xc00: 1 << 12, 0xd00: 1 << 13}   # before-snapshot
SNAP_NONE = sum(SNAP_SKIP.values())
WAIT_END = 1 << 14                                       # follow a fault that continues to the END
PROG_SRAM = 1 << 15                                      # run a copy of the program from internal SRAM
CH1_PINS = 1 << 16                                       # switch FMC channel 1's pins to the controller for the run
FULLMAP = 1 << 17                                        # CPU dump of all 1024 window words before the start / after
WHY = {1: "END", 2: "FAULT", 3: "TIMEOUT"}

_ASM = None
_LOG_STATE = {"exp": None, "calls": 0}


def set_experiment(name):
    """name recorded in FMSS_PROGRAM_LOG lines"""
    _LOG_STATE["exp"] = name


def assembler():
    """Build emCORE's FMSS assembler (tools/fmssasm.c) once per process"""
    global _ASM
    if _ASM is None:
        src = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fmssasm.c")
        d = tempfile.mkdtemp(prefix="fmssprobe_")
        _ASM = os.path.join(d, "fmssasm")
        subprocess.check_call(["cc", "-O2", "-o", _ASM, src])
    return _ASM


def assemble(t, text):
    """FMSS source text -> program bytes, with the target's assembler mode and tokens"""
    d = tempfile.mkdtemp(prefix="fmssprobe_")
    src, out = os.path.join(d, "probe.s"), os.path.join(d, "probe.bin")
    with open(src, "w") as f:
        f.write(t.expand(text))
    r = subprocess.run([assembler()] + list(t.asm_flags) + ["-b", out, src], capture_output=True, text=True)
    if r.returncode:
        raise ToolError("assembler: %s" % (r.stderr.strip() or r.stdout.strip()))
    return open(out, "rb").read()


def lint(t, code, allow_nand=False, allow_indirect=False, allow_unknown=False):
    """Refuse what could reach the flash, unless allowed"""
    for i in range(0, len(code), 8):
        w, imm = struct.unpack_from("<II", code, i)
        op, arg = w >> 24, w & 0xffff
        where = "instruction %d (0x%x)" % (i // 8, i)
        if op in (0x01, 0x02) and arg in t.nand_regs and not allow_nand:
            raise ToolError("%s writes %s: pass --allow-nand to let a probe drive the flash" % (where, t.nand_regs[arg]))
        if op == 0x19 and not allow_indirect:
            raise ToolError("%s is pst (store to a computed register): pass --allow-indirect" % where)
        if op not in t.known_ops and not allow_unknown:
            raise ToolError("%s has opcode 0x%02x (no known meaning): pass --allow-unknown" % (where, op))


def request(t, code, params=(), timeout_us=100000, flags=0, late=(), late_us=0, start_value=0, late2=(), late2_us=0):
    """params are written before the start; late ones once late_us after it, while the program runs. start_value:
    the FMSS_CTRL word that starts the program (0: the driver's 0xfff5)"""
    t.need("probe_prog", "probe_out", "req_magic", "req_version", "max_params")
    params = list(params) + list(late) + list(late2)
    if late2 and not late:
        raise ToolError("a second late batch needs a first one")
    if len(params) > t.max_params:
        raise ToolError("at most %d parameter writes" % t.max_params)
    if t.probe_prog + len(code) > t.probe_out:
        raise ToolError("program too long")
    head = struct.pack("<11I", t.req_magic, t.req_version, flags, timeout_us, len(params), len(code), late_us, len(late),
                       start_value, late2_us, len(late2))
    body = b"".join(struct.pack("<II", o, v & 0xffffffff) for o, v in params)
    blob = (head + body).ljust(t.probe_prog, b"\0") + code
    return blob


def parse_out(t, head_and_windows, samples_raw):
    t.need("head_fmt", "out_magic", "head_bytes", "sample_fmt")
    h = struct.unpack_from(t.head_fmt, head_and_windows, 0)
    if h[0] != t.out_magic:
        raise ToolError("no probe output (magic %08x)" % h[0])
    W = t.windows
    n = len(W) * 64
    words = struct.unpack_from("<%dI" % (2 * n), head_and_windows, t.head_bytes)
    before = {W[w] + 4 * i: words[w * 64 + i] for w in range(len(W)) for i in range(64)}
    after = {W[w] + 4 * i: words[n + w * 64 + i] for w in range(len(W)) for i in range(64)}
    samples = [struct.unpack_from(t.sample_fmt, samples_raw, 40 * k) for k in range(len(samples_raw) // 40)]
    return {"why": WHY.get(h[2], h[2]), "elapsed_us": h[3], "cs_irq": h[4], "seq_result": h[5], "pc": h[6],
            "c50": h[7], "prog": h[8], "nsamples": h[9], "dropped": h[10], "polls": h[11],
            "pre_ok": h[12], "pre_us": h[13], "pre_cs_irq": h[14], "pre_c00": h[15], "late_at_us": h[16],
            "fault_at_us": h[17] & ~1, "irq_seen": h[18] >> 22 & 1, "irq_end": h[19] >> 22 & 1, "late2_at_us": h[20],
            "vic1_seen": h[18], "vic1_end": h[19], "vic0_seen": h[21],
            "r": [after[0xc18 + 4 * i] for i in range(8)], "before": before, "after": after,
            "samples": [{"t_us": s[0], "pc": s[1], "r": list(s[2:])} for s in samples]}


def probe(s, t, code, params=(), timeout_us=100000, flags=0, scratch=None, clean=True, allow=None, steps=0, late=(),
          late_us=0, start_value=0, scratch_read=256, quiet=False, late2=(), late2_us=0):
    """Run one program on the device. scratch: bytes placed at the scratch area first (its bus address is
    t.scratch_addr(s)). params are written after t.clean_params (so they can override them). Returns the parsed record
    plus the scratch area after the run."""
    t.need("clean_params", "cmd_step", "cmd_probe", "step_ok_bit", "probe_scratch", "head_bytes", "max_samples",
           "need_version")
    host = importlib.import_module(t.host)
    nand = importlib.import_module(t.host + ".nand")
    Cmd = importlib.import_module(t.host + ".protocol").Cmd
    nw = len(t.windows)
    if steps & S_NO_CLEAN:                               # see what the previous run left: no CPU cleanup either
        clean = False
    flags |= steps
    params = (t.clean_params if clean else []) + list(params)
    st = s.status()
    if not st.step_ok_mask & (1 << t.step_ok_bit):
        nand.check(s.job(getattr(Cmd, t.cmd_step), 1, 0))
        st = s.status()
    blob = request(t, code, params, timeout_us, flags, late, late_us, start_value, late2, late2_us)
    _log = os.environ.get("FMSS_PROGRAM_LOG")
    if _log:
        _LOG_STATE["calls"] += 1
        with open(_log, "a") as f:
            f.write(json.dumps({"exp": _LOG_STATE["exp"], "i": _LOG_STATE["calls"],
                                "sha": hashlib.sha256(blob).hexdigest(),
                                "scratch": None if scratch is None else len(scratch)}) + "\n")
    s.write_memory(st.result_addr, blob)
    if scratch is not None:
        s.write_memory(st.result_addr + t.probe_scratch, scratch)
    nand.check(s.job(getattr(Cmd, t.cmd_probe), len(blob), zlib.crc32(blob) & 0xffffffff, timeout=10.0))
    hw = s.read(st.result_addr + t.probe_out, t.head_bytes + 2 * nw * 256)
    nsamp = min(struct.unpack_from("<I", hw, 36)[0], t.max_samples)
    sraw = s.read(st.result_addr + t.probe_out + t.head_bytes + 2 * nw * 256, 40 * nsamp) if nsamp else b""
    rec = parse_out(t, hw, sraw)
    ver = struct.unpack_from("<I", hw, 4)[0]
    if ver < t.need_version:
        raise ToolError("the running emCORE has probe version %d; this tool needs %d or later: run `n3g.py reload` "
                        "with the current build first" % (ver, t.need_version))
    rec["scratch"] = s.read(st.result_addr + t.probe_scratch, scratch_read) if scratch is not None else None
    if flags & FULLMAP:
        raw = s.read(st.result_addr + t.probe_out + t.head_bytes + 2 * nw * 256 + 40 * t.max_samples, 8192)
        rec["fullmap"] = [list(struct.unpack_from("<1024I", raw, 0)), list(struct.unpack_from("<1024I", raw, 4096))]
    want = dict(params)                                  # the last write per offset is what the CPU left
    bad = {o: (v, rec["before"].get(o)) for o, v in want.items()       # control registers only (0xc00 window)
           if 0xc00 <= o < 0xd00 and o not in t.not_readback and rec["before"][o] != v & 0xffffffff}
    rec["params_ok"] = not bad
    if bad and not quiet:
        print("WARNING: CPU register writes did not read back before the start: %s" % ", ".join(
            "0x%03x wrote %08x read %08x" % (o, v, r) for o, (v, r) in sorted(bad.items())))
    return rec


def changes(rec, skip=(0x048,)):
    return {o: (rec["before"][o], rec["after"][o]) for o in rec["before"]
            if rec["before"][o] != rec["after"][o] and o not in skip}


def ctrl_changes(t, rec):
    """changes in the sequencer control window other than the start word, CS_IRQ, PC and r0..r7"""
    return " ".join("0x%03x:%08x->%08x" % (o, a, b) for o, (a, b) in sorted(changes(rec).items())
                    if 0xc00 <= o < 0xd00 and o not in t.ctrl_skip)


def describe(rec):
    L = ["preflight (END-only program after the controller set-up): %s in %d us, CS_IRQ %08x, FMSS_CTRL %08x" % (
        "completed" if rec["pre_ok"] else "DID NOT COMPLETE", rec["pre_us"], rec["pre_cs_irq"], rec["pre_c00"])]
    L += ["%s after %d us (%d polls, %d samples%s); CS_IRQ %08x, SEQ_RESULT %08x, PC 0x%x (instruction %d), C50 %08x" % (
        rec["why"], rec["elapsed_us"], rec["polls"], rec["nsamples"],
        ", %d dropped" % rec["dropped"] if rec["dropped"] else "", rec["cs_irq"], rec["seq_result"], rec["pc"],
        rec["pc"] // 8, rec["c50"])]
    L.append("r0..r7: " + " ".join("%08x" % v for v in rec["r"]))
    ch = changes(rec)
    if ch:
        L.append("registers changed by the run (before -> after):")
        for o in sorted(ch):
            L.append("  0x%03x: %08x -> %08x" % (o, ch[o][0], ch[o][1]))
    if rec["samples"]:
        L.append("samples (t us, pc, r0..r7; without --sample-regs only the last column is set: CS_IRQ):")
        for smp in rec["samples"][:40]:
            L.append("  %6d  0x%04x  %s" % (smp["t_us"], smp["pc"], " ".join("%08x" % v for v in smp["r"])))
        if len(rec["samples"]) > 40:
            L.append("  ... %d more" % (len(rec["samples"]) - 40))
    return "\n".join(L)


def answer(rec):
    """None if the run ended normally, else why it gave no answer"""
    if rec["why"] == "END":
        return None
    return "no answer (%s; preflight %s)" % (rec["why"], "ok" if rec["pre_ok"] else "FAILED: the engine did not run "
                                             "even the END-only program")


def where(rec):
    r0, r1 = rec["r"][0] == 0x57a27000, rec["r"][1] == 0x3a2c3a2c
    return {(True, False): "started at 0", (False, True): "STARTED AT THE VECTOR", (True, True): "ran 0, then the vector",
            (False, False): "neither marker"}[(r0, r1)]


def state(rec):
    return "C58 %x C5C %x C4C %x CS_IRQ %08x" % (rec["after"][0xc58], rec["after"][0xc5c], rec["after"][0xc4c], rec["cs_irq"])


def jsonable(x):
    if isinstance(x, dict):
        return {str(k) if not isinstance(k, int) else "0x%03x" % k: jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, bytes):
        return x.hex()
    return x


# ---- the sequencer's view of the FMC window, snapshots, power-on comparison, soft reset
def fmc_window_dump(s, t):
    """The whole 4 KB FMC window as the sequencer sees it (pld of every word into RAM): registers outside the
    windows the CPU snapshot covers"""
    base = t.scratch_addr(s)
    src = t.prologue(c58=0) + """    movi r0, #0x%08x
    movi r1, #0x0
    movi r2, #0x400
loop:
    pld  r3, [r1]
    sti  [r0], r3
    add  r0, r0, #0x4
    add  r1, r1, #0x4
    sub  r2, r2, #0x1
    bnz  r2, loop
    movi r6, #0x600d600d
    end
""" % base + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=200000, scratch=b"\xee" * 4096, scratch_read=4096)
    if answer(rec):
        return rec, "fmcmap: " + answer(rec)
    words = struct.unpack("<1024I", rec["scratch"])
    rec["map"] = {"0x%03x" % (4 * i): w for i, w in enumerate(words)}
    lines = []
    for blk in range(0, 0x1000, 0x100):
        nz = [(o, w) for o, w in ((blk + 4 * i, words[blk // 4 + i]) for i in range(64)) if w]
        lines.append("  0x%03x..0x%03x: %s" % (blk, blk + 0xfc, "all zero" if not nz else " ".join(
            "%03x:%08x" % x for x in nz[:24]) + (" ... %d more" % (len(nz) - 24) if len(nz) > 24 else "")))
    return rec, "%s; non-zero words per 256-byte block:\n" % rec["why"] + "\n".join(lines)


def power_on_diff(t, snap):
    """words of a snapshot that differ from the power-on values"""
    run, dump = snap["run"], snap["fmcmap"]
    d = {o: v for o, v in run["before"].items() if o < 0xd80 and o not in t.diff_skip and v != t.power_on.get(o, 0)
         and not 0xc60 <= o < 0xd00}
    m = dump.get("map") if isinstance(dump, dict) else None
    if m:
        for k, v in m.items():
            o = int(k, 16)
            if (0x100 <= o < 0x200 or 0x900 <= o < 0xa00) and v != t.power_on_seq.get(o, 0) & 0xffffffff:
                d[o] = v
    return d


def idle_events(s, t):
    """which of the 16 events pass in a program that does nothing else (power-on: only event 11)"""
    ok = []
    for e in range(16):
        src = t.prologue(c58=0) + "    wait %d\n    movi r6, #0x600d600d\n    end\n" % e + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
        if not rec["fault_at_us"]:
            ok.append(e)
    return ok


def snapshot(s, t):
    """The state without changing it: all windows (CPU view, 0x400 included) around an END-only run, and the
    sequencer's view of the whole 4 KB window (pld dump)"""
    rec = probe(s, t, assemble(t, "    end\n"), timeout_us=20000, quiet=True, flags=F_WIN400)
    dump, _ = fmc_window_dump(s, t)
    return {"run": rec, "fmcmap": dump}


def soft_reset(s, t):
    """the writes of `restore` without its snapshots: every writable FMC / ECC / sequencer word to its power-on value,
    status words acknowledged, the 0x100 block rewritten by the sequencer, parameter RAM zeroed"""
    probe(s, t, assemble(t, "    end\n"), params=t.restore, timeout_us=20000, quiet=True, flags=F_WIN400)
    probe(s, t, assemble(t, t.prologue(c58=0) + t.restore_seq() + "    end\n" + t.trap()), params=t.restore_pram,
          timeout_us=20000, quiet=True)
