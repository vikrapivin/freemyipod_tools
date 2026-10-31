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

"""Sequencer control words: C58 / C4C / C5C, start steps and words, C50, C14, C08, parameter RAM, interrupt line."""
import sys

from ..probe import (probe, assemble, answer, ctrl_changes, where, state, F_KEEP_CTRL0, F_SAMPLE_REGS, S_NO_CLEAN,
                     S_NO_PREFLIGHT, S_NO_RESET, S_NO_400, S_END_NOCLEAN, SNAP_SKIP, SNAP_NONE, WAIT_END)
from .registry import experiment


MARKER = ("    movi r0, #0x57a27000\n    movi r6, #0x0\n    movi r2, #0x0\n    end\n"
          "mark:\n    movi r1, #0x3a2c3a2c\n    end\n")                 # instruction 0 sets r0; offset 0x20 sets r1


@experiment("pram")
def ex_pram(s, t):
    """Parameter RAM 0xd00..0xdfc: which words keep a CPU write, which the sequencer can read and write"""
    params = [(0xd00 + 4 * i, 0xa5000000 | (0xd00 + 4 * i)) for i in range(64)]
    reads = [0xd04, 0xd20, 0xd40, 0xd5c, 0xd60, 0xd7c, 0xd80, 0xdfc]
    src = t.prologue() + "".join("    ld   r%d, [0x%03x]\n" % (i, o) for i, o in enumerate(reads)) + \
        "".join("    wr   [0x%03x], #0x%08x\n" % (0xd80 + 4 * i, 0x5e000000 | (0xd80 + 4 * i)) for i in range(32)) + \
        "    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), params=params)
    if answer(rec):
        return rec, answer(rec)
    keep = [o for o, v in params if rec["before"][o] == v]
    seq_w = [0xd80 + 4 * i for i in range(32) if rec["after"][0xd80 + 4 * i] == (0x5e000000 | (0xd80 + 4 * i))]
    def rng(lst):
        if not lst:
            return "none"
        out, a = [], lst[0]
        for x, y in zip(lst, lst[1:] + [None]):
            if y != x + 4:
                out.append("0x%03x" % a if a == x else "0x%03x..0x%03x" % (a, x))
                a = y
        return ", ".join(out)
    return rec, ("CPU write then CPU read keeps: %s\n  sequencer ld of %s -> %s\n  sequencer wr then CPU read keeps "
                 "(0xd80..0xdfc): %s" % (rng(keep), ", ".join("0x%03x" % o for o in reads),
                                         " ".join("%08x" % v for v in rec["r"]), rng(seq_w)))


@experiment("resume")
def ex_resume(s, t):
    """Where does a start begin? FMSS_C58 / FMSS_C5C set by the CPU before the start, the fault vector at a marker"""
    out, lines = {}, []
    # instruction 0 sets r0; the marker (instruction 4) sets r1; END at 3 and 5
    src = "    movi r0, #0x57a27000\n    movi r6, #0x0\n    movi r2, #0x0\n    end\nmark:\n    movi r1, #0x3a2c3a2c\n    end\n"
    code = assemble(t, src)
    for label, params in (("C58 0, C4C mark", [(0xc58, 0), (0xc4c, 0x20)]),
                          ("C58 1, C4C mark", [(0xc58, 1), (0xc4c, 0x20)]),
                          ("C58 2, C4C mark", [(0xc58, 2), (0xc4c, 0x20)]),
                          ("C58 4, C4C mark", [(0xc58, 4), (0xc4c, 0x20)]),
                          ("C5C 1, C58 0, C4C mark", [(0xc5c, 1), (0xc58, 0), (0xc4c, 0x20)]),
                          ("C5C 1, C58 1, C4C mark", [(0xc5c, 1), (0xc58, 1), (0xc4c, 0x20)])):
        rec = probe(s, t, code, params=[(0xc18, 0), (0xc1c, 0)] + params)    # r0, r1 zeroed by the CPU first
        out[label] = rec
        r0, r1 = rec["r"][0], rec["r"][1]
        where = "at 0" if r0 == 0x57a27000 and r1 != 0x3a2c3a2c else "at the fault vector" if r1 == 0x3a2c3a2c and \
            r0 != 0x57a27000 else "r0=%08x r1=%08x" % (r0, r1)
        lines.append("  %-24s -> %-7s started %s; after: C58 %x C5C %x C4C %x CS_IRQ %08x%s" % (
            label, rec["why"], where, rec["after"][0xc58], rec["after"][0xc5c], rec["after"][0xc4c], rec["cs_irq"],
            "" if rec["params_ok"] else "  [CPU writes did not read back]"))
    probe(s, t, assemble(t, "    end\n"))                  # leave the clean state behind
    return out, "start address:\n" + "\n".join(lines)


@experiment("control")
def ex_control(s, t):
    """The sequencer's control words: what reads back after a run (0xc00..0xc5c), can the CPU preload r0..r7?"""
    src = t.prologue() + "    wr   [FMSS_C10], #0x1\n    wr   [FMSS_C14], #0xff\n    st   [0xd6c], r3\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), params=[(0xc24, 0x0badcafe), (0xd6c, 0)])
    if answer(rec):
        return rec, answer(rec)
    c = {o: rec["after"][o] for o in range(0xc00, 0xc60, 4)}
    pre = rec["after"][0xd6c]
    return rec, "control window after a run: %s\n  CPU wrote r3 (0xc24) = 0badcafe before start; the program stored r3 = %08x (%s)" % (
        " ".join("%03x:%08x" % (o, v) for o, v in c.items()), pre,
        "preload works" if pre == 0x0badcafe else "not kept (0)" if pre == 0 else "?")


@experiment("c58")
def ex_c58(s, t):
    """FMSS_C58: what each value does to a WAIT that times out (time, fault flag, vector or continue)"""
    out, lines = {}, []
    for c58 in list(range(0, 32)) + [0x40, 0x80, 0xff, 0x100, 0xffff]:
        src = t.prologue(c58=c58) + "    wait 1\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=1000000)
        out[c58] = rec
        r6 = rec["r"][6]
        how = "continued" if r6 == 0x600d600d else "vector" if r6 == 0xfa17fa17 else "r6=%08x" % r6
        lines.append("  C58 = 0x%-5x -> %-7s %-9s after %7d us, CS_IRQ %08x; %s" % (c58, rec["why"], how,
                     rec["elapsed_us"], rec["cs_irq"], ctrl_changes(t, rec)))
    return out, "C58 sweep (wait 1, nothing pending):\n" + "\n".join(lines)


@experiment("c58ill")
def ex_c58ill(s, t):
    """Do FMSS_C58's bits also govern an illegal-opcode fault (0x09), as they do a WAIT timeout?"""
    out, lines = {}, []
    for c58 in (0, 1, 2, 4, 5):
        src = t.prologue(c58=c58) + "    movi r6, #0x0\n    .insn 0x09, 0, 0, 0\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=50000)
        out[c58] = rec
        r6 = rec["r"][6]
        how = "continued" if r6 == 0x600d600d else "vector" if r6 == 0xfa17fa17 else "r6=%08x" % r6
        lines.append("  C58 = %d -> %-7s %-9s after %5d us, CS_IRQ %08x; %s" % (c58, rec["why"], how, rec["elapsed_us"],
                     rec["cs_irq"], ctrl_changes(t, rec)))
    return out, "illegal opcode 0x09 under each C58:\n" + "\n".join(lines)


@experiment("startseq")
def ex_startseq(s, t):
    """After a sequencer reset, which step lets a program run? (the START is accepted but nothing executes otherwise)"""
    out, lines = {}, []
    marker = assemble(t, MARKER)
    cases = [("reset only", S_NO_CLEAN | S_NO_400 | S_NO_PREFLIGHT),
             ("reset + clean", S_NO_400 | S_NO_PREFLIGHT),
             ("reset + 0x400", S_NO_CLEAN | S_NO_PREFLIGHT),
             ("reset + preflight", S_NO_CLEAN | S_NO_400),
             ("reset + clean + 0x400", S_NO_PREFLIGHT),
             ("reset + all (default)", 0),
             ("no reset, nothing else", S_NO_RESET | S_NO_CLEAN | S_NO_400 | S_NO_PREFLIGHT)]
    for label, steps in cases:
        probe(s, t, assemble(t, "    end\n"))                                   # a normal probe first (ends parked)
        rec = probe(s, t, marker, params=[(0xc18, 0), (0xc1c, 0)], steps=steps, timeout_us=20000)
        out[label] = rec
        lines.append("  %-24s -> %-7s %-22s FMSS_CTRL after %08x, C08 %x, CS_IRQ %08x" % (
            label, rec["why"], where(rec), rec["after"][0xc00], rec["after"][0xc08], rec["cs_irq"]))
    return out, "start after a sequencer reset (marker program):\n" + "\n".join(lines)


@experiment("startseq2")
def ex_startseq2(s, t):
    """A START right after a reset runs; with register accesses in between it does not: which access? (reset, no
    preflight, no cleanup; END in CS_IRQ = the program ran)"""
    out, lines = {}, []
    base = S_NO_CLEAN | S_NO_400 | S_NO_PREFLIGHT
    code = assemble(t, "    movi r0, #0x57a27000\n    end\n")
    keep0 = F_KEEP_CTRL0
    cases = [("nothing in between", base | SNAP_NONE | keep0, []),
             ("FMC_CTRL0 = 0", base | SNAP_NONE, []),
             ("full snapshot", base | keep0, []),
             ("read window 0x000", base | keep0 | (SNAP_NONE & ~SNAP_SKIP[0x000]), []),
             ("read window 0x800", base | keep0 | (SNAP_NONE & ~SNAP_SKIP[0x800]), []),
             ("read window 0xc00", base | keep0 | (SNAP_NONE & ~SNAP_SKIP[0xc00]), []),
             ("read window 0xd00", base | keep0 | (SNAP_NONE & ~SNAP_SKIP[0xd00]), []),
             ("CPU writes r0", base | SNAP_NONE | keep0, [(0xc18, 0)]),
             ("CPU writes 0xd60", base | SNAP_NONE | keep0, [(0xd60, 0)])]
    for label, flags, params in cases:
        probe(s, t, assemble(t, "    end\n"))                                   # a normal probe first (ends parked)
        rec = probe(s, t, code, params=params, steps=flags & ~(F_KEEP_CTRL0), flags=flags & F_KEEP_CTRL0,
                    clean=False, timeout_us=20000)
        out[label] = rec
        lines.append("  %-22s -> %-7s %-8s FMSS_CTRL after %08x, CS_IRQ %08x" % (
            label, rec["why"], "RAN" if rec["why"] == "END" else "no run", rec["after"][0xc00], rec["cs_irq"]))
    return out, "reset, then one kind of access, then START:\n" + "\n".join(lines)


@experiment("startseq3")
def ex_startseq3(s, t):
    """After a reset a START fetches instruction 0 (C40/C44) but does not run it unless it is END: confirm, and find
    what releases it (a CPU write of FMSS_CTRL 2 ms after the start). Reset, no preflight, nothing else in between"""
    out, lines = {}, []
    base = S_NO_CLEAN | S_NO_400 | S_NO_PREFLIGHT | SNAP_NONE
    marker = assemble(t, "    movi r0, #0x57a27000\n    movi r6, #0x600d600d\n    end\n")
    cases = [("END-only program", assemble(t, "    end\n"), []),
             ("no-op (0x08) first", assemble(t, "    .insn 0x08, 0, 0, 0\n    movi r0, #0x57a27000\n    end\n"), []),
             ("wr CS_IRQ first", assemble(t, "    wr   [CS_IRQ], #0xff\n    movi r0, #0x57a27000\n    end\n"), []),
             ("marker (control)", marker, []),
             ("marker, then CTRL fff5", marker, [(0xc00, 0xfff5)]),
             ("marker, then CTRL 4fff5", marker, [(0xc00, 0x4fff5)]),
             ("marker, then CTRL 40000", marker, [(0xc00, 0x40000)]),
             ("marker, then CTRL 0", marker, [(0xc00, 0x0)])]
    for label, code, late in cases:
        probe(s, t, assemble(t, "    end\n"))                                   # a normal probe first (ends parked)
        rec = probe(s, t, code, steps=base, flags=F_KEEP_CTRL0, clean=False, late=late, late_us=2000, timeout_us=20000)
        out[label] = rec
        lines.append("  %-24s -> %-7s r0 %08x r6 %08x  FMSS_CTRL after %08x, C40 %08x C44 %08x, CS_IRQ %08x" % (
            label, rec["why"], rec["r"][0], rec["r"][6], rec["after"][0xc00], rec["after"][0xc40], rec["after"][0xc44],
            rec["cs_irq"]))
    return out, "reset, START (no preflight), optionally a CPU write of FMSS_CTRL 2 ms later:\n" + "\n".join(lines)


@experiment("oneshot")
def ex_oneshot(s, t):
    """FMSS_C58 bit 1 (the next END runs the fault vector once): armed by a sequencer write too, or disarmed by a fault?"""
    out, lines = {}, []
    tail = "    end\nmark:\n    movi r1, #0x3a2c3a2c\n    end\n"
    cases = [("CPU sets bit 1, END (control)", "    wr   [FMSS_C4C], #mark\n    movi r0, #0x57a27000\n" + tail,
              [(0xc58, 2)]),
             ("program sets bit 1, END", "    wr   [FMSS_C4C], #mark\n    wr   [FMSS_C58], #0x2\n"
              "    movi r0, #0x57a27000\n" + tail, []),
             ("CPU sets bit 1, fault, END", "    wr   [FMSS_C4C], #mark\n    movi r0, #0x57a27000\n    wait 1\n" + tail,
              [(0xc58, 2)]),
             ("program sets bit 1, fault, END", "    wr   [FMSS_C4C], #mark\n    wr   [FMSS_C58], #0x2\n"
              "    movi r0, #0x57a27000\n    wait 1\n" + tail, [])]
    for label, src, params in cases:
        rec = probe(s, t, assemble(t, src), params=[(0xc18, 0), (0xc1c, 0)] + params, timeout_us=50000)
        out[label] = rec
        lines.append("  %-32s -> %-7s %-24s %s" % (label, rec["why"], where(rec), state(rec)))
    return out, "C58 bit 1 one-shot:\n" + "\n".join(lines)


@experiment("c50")
def ex_c50(s, t):
    """FMSS_C50 b0 while the fault handler runs (the original handlers test it and write 1): set by a fault? cleared by
    the ack, or at the END?"""
    out, lines = {}, []
    handler = ("\ntrap:\n    ld   r4, [FMSS_C50], 0xffffffff\n%s    ld   r5, [FMSS_C50], 0xffffffff\n"
               "    movi r6, #0xfa17fa17\n    end\n")
    cases = [("timeout, handler acks", "    wait 1\n", "    wr   [FMSS_C50], #0x1\n"),
             ("timeout, no ack", "    wait 1\n", ""),
             ("illegal opcode, handler acks", "    .insn 0x09, 0, 0, 0\n", "    wr   [FMSS_C50], #0x1\n"),
             ("program jumps to the handler (no fault)", "    movi r0, #0x1\n    bnz  r0, trap\n", "")]
    for label, body, ack in cases:
        src = t.prologue(c58=4) + "    movi r4, #0x0\n    movi r5, #0x0\n" + body + "    movi r6, #0x600d600d\n    end\n" + \
            handler % ack
        rec = probe(s, t, assemble(t, src), timeout_us=50000)
        out[label] = rec
        lines.append("  %-40s -> %-7s C50 in the handler %08x, after the ack %08x, after the end %08x" % (
            label, rec["why"], rec["r"][4], rec["r"][5], rec["after"][0xc50]))
    return out, "FMSS_C50 seen by the fault handler:\n" + "\n".join(lines)


@experiment("c50x", needs=("nand_seq",))
def ex_c50x(s, t):
    """Which fault sets FMSS_C50 b0? (run on its own and last: the bus cases may hang the SoC)"""
    out, lines = {}, []
    handler = ("\ntrap:\n    ld   r4, [FMSS_C50], 0xffffffff\n    ld   r5, [FMSS_C3C], 0xffffffff\n"
               "    movi r6, #0xfa17fa17\n    end\n")
    cases = [("wait 3, CE0 selected, nothing started", t.nand_prologue(0) + "    wr   [FMSS_C58], #0x4\n    wait 3\n"),
             ("ldi from 0x50000000", t.prologue(c58=4) + "    movi r1, #0x50000000\n    ldi  r2, [r1]\n"),
             ("DMA to 0x50000000", t.nand_prologue(0) + "    wr   [FMSS_C58], #0x4\n    wr   [FMC_DMA_ADDR], #0x50000000\n"
              "    wr   [FMC_DNUM], #0x1ff\n    wr   [FMCTRL1], #0x100\n    wait 2\n")]
    for label, body in cases:
        src = body.replace("\ntrap:", "") + "    movi r6, #0x600d600d\n    end\n" + handler
        src = src.replace("    wr   [FMSS_C4C], #trap\n", "    wr   [FMSS_C4C], #trap\n    movi r4, #0x0\n    movi r5, #0x0\n")
        print("  running: %s" % label)
        sys.stdout.flush()
        rec = probe(s, t, assemble(t, src), timeout_us=50000)
        out[label] = rec
        lines.append("  %-40s -> %-7s %6d us, C50 in the handler %08x, C3C %08x, CS_IRQ %08x" % (
            label, rec["why"], rec["elapsed_us"], rec["r"][4], rec["r"][5], rec["cs_irq"]))
    return out, "faults and FMSS_C50 b0:\n" + "\n".join(lines)


@experiment("c14")
def ex_c14(s, t):
    """FMSS_C14: does a sequencer write set it, and does opcode 0x15 invert it (twice = unchanged)?"""
    out, lines = {}, []
    cases = [("wr C14 #0x12", "    wr   [FMSS_C14], #0x12\n"),
             ("wr C14 #0x00", "    wr   [FMSS_C14], #0x0\n"),
             ("wr C14 #0xff", "    wr   [FMSS_C14], #0xff\n"),
             ("wr C14 #0x0f; op15", "    wr   [FMSS_C14], #0x0f\n    .insn 0x15, 0, 0, 0\n"),
             ("op15", "    .insn 0x15, 0, 0, 0\n"),
             ("op15; op15", "    .insn 0x15, 0, 0, 0\n    .insn 0x15, 0, 0, 0\n"),
             ("op15 x3", "    .insn 0x15, 0, 0, 0\n" * 3)]
    for label, body in cases:
        src = t.prologue(c58=0) + body + "    ld   r2, [FMSS_C14]\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src))
        out[label] = rec
        lines.append("  %-20s -> C14 before %08x, read by the program %08x, after %08x" % (
            label, rec["before"][0xc14], rec["r"][2], rec["after"][0xc14]))
    return out, "FMSS_C14:\n" + "\n".join(lines)


@experiment("pending")
def ex_pending(s, t):
    """The second-session state: a program leaves C58 = 1 (after a fault or not); where do the next starts begin?
    Recreated with the steps the probe had then (park without the clean after; reset, +0x400, preflight before)"""
    out, lines = {}, []
    firsts = [
        ("C58 1, WAIT times out (fault, continue)", 1, True),
        ("C58 1, no fault (END)", 1, False),
        ("C58 4, WAIT times out (fault, vector)", 4, True),
        ("C58 0, WAIT times out (fault, continue)", 0, True),
        ("C58 2, no fault (END)", 2, False),
    ]
    old = S_NO_CLEAN | S_END_NOCLEAN                                        # the second session's probe
    marker = assemble(t, MARKER)
    for label, c58, fault in firsts:
        src = "    wr   [CS_IRQ], #0xff\n    wr   [FMSS_C4C], #0x20\n    wr   [FMSS_C58], #0x%x\n" % c58 + \
            ("    wait 1\n" if fault else "") + "    movi r6, #0x600d600d\n    end\n"
        probe(s, t, assemble(t, "    end\n"))                                   # clean state
        r1 = probe(s, t, assemble(t, src), steps=old, timeout_us=50000)
        r2 = probe(s, t, marker, params=[(0xc18, 0), (0xc1c, 0)], steps=old, timeout_us=20000)
        r3 = probe(s, t, marker, params=[(0xc18, 0), (0xc1c, 0)], steps=old, timeout_us=20000)
        r4 = probe(s, t, marker, params=[(0xc18, 0), (0xc1c, 0)], timeout_us=20000)    # the current probe: clean first
        out[label] = {"first": r1, "next": r2, "again": r3, "clean": r4}
        lines.append("  first: %s -> %s, %s" % (label, r1["why"], state(r1)))
        lines.append("    next start (old steps)  -> %-24s %s" % (where(r2), state(r2)))
        lines.append("    again (old steps)       -> %-24s %s" % (where(r3), state(r3)))
        lines.append("    current steps (clean)   -> %-24s %s" % (where(r4), state(r4)))
    return out, "start after a program that left state behind:\n" + "\n".join(lines)


@experiment("ctrlhold", needs=("start_value",))
def ex_ctrlhold(s, t):
    """FMSS_CTRL bits 1, 3 and 16 kept a program from running (device). Do they pause it (a later write without them
    releases it), single-step it (each later write runs one instruction), and does bit 17 clear r0..r7?"""
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r0, #1000\nloop:\n    sub  r0, r0, #0x1\n    bnz  r0, loop\n" \
        "    movi r6, #0x600d600d\n    end\n" + t.trap()
    code = assemble(t, src)
    out, lines = {}, []
    cases = [("0x3, no later write", 0x3, []), ("0x3, later 0x1", 0x3, [(0xc00, 0x1)]),
             ("0x3, later 0xfff5", 0x3, [(0xc00, 0xfff5)]), ("0x9, no later write", 0x9, []),
             ("0x9, later 0x9", 0x9, [(0xc00, 0x9)]), ("0x9, later 0x9 x5", 0x9, [(0xc00, 0x9)] * 5),
             ("0x9, later 0x8 x5", 0x9, [(0xc00, 0x8)] * 5), ("0x9, later 0x1", 0x9, [(0xc00, 0x1)]),
             ("0x10001, later 0x1", 0x10001, [(0xc00, 0x1)]), ("0x3, later 0x2 x5", 0x3, [(0xc00, 0x2)] * 5)]
    for label, sv, late in cases:
        rec = probe(s, t, code, timeout_us=20000, start_value=sv, late=late, late_us=2000, flags=F_SAMPLE_REGS)
        out[label] = rec
        pcs = [x["pc"] for x in rec["samples"]]
        lines.append("  %-22s -> %-7s %6d us, r0 %4d, r6 %08x, PC %x, FMSS_CTRL %08x, CS_IRQ %08x, PCs seen %s" % (
            label, rec["why"], rec["elapsed_us"], rec["r"][0], rec["r"][6], rec["pc"], rec["after"][0xc00],
            rec["cs_irq"], " ".join("%x" % p for p in pcs[:12])))
    end = assemble(t, "    end\n")
    pre = [(0xc20, 0x22222222), (0xc24, 0x33333333), (0xc28, 0x44444444), (0xc2c, 0x55555555)]
    for sv in (0xfff5, 0x20001, 0x20000):
        rec = probe(s, t, end, params=pre, timeout_us=20000, start_value=sv)
        out["regs 0x%x" % sv] = rec
        lines.append("  END-only, r2..r5 preloaded, start 0x%-5x -> %-7s r2..r5 %s" % (
            sv, rec["why"], " ".join("%08x" % rec["r"][i] for i in range(2, 6))))
    return out, "start-word bits 1, 3, 16 and 17 (loop of 1000, later writes 2 ms after the start):\n" + "\n".join(lines)


@experiment("timeregs")
def ex_timeregs(s, t):
    """Writable sequencer words with no known use (C10, C3C, C48, C54): does any change the WAIT timeout?"""
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    wait 2\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    code = assemble(t, src)
    cases = [("control", [])] + [("C54 = 0x%x" % v, [(0xc54, v)]) for v in (0, 1, 0x10, 0x100, 0x5a5, 0xfff)] + \
        [("C10 = 0x%x" % v, [(0xc10, v)]) for v in (0, 1, 0x80, 0xff)] + \
        [("C48 = 0x%x" % v, [(0xc48, v)]) for v in (0, 0xffffffff)] + \
        [("C3C = 0x%x" % v, [(0xc3c, v)]) for v in (0, 0xffffffff)]
    out, lines = {}, []
    for label, params in cases:
        rec = probe(s, t, code, params=params, timeout_us=200000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-16s -> %-7s %6d us, first fault at %6d us, r6 %08x, after: C10 %02x C3C %08x C48 %08x C54 %03x" % (
            label, rec["why"], rec["elapsed_us"], rec["fault_at_us"], rec["r"][6], rec["after"][0xc10],
            rec["after"][0xc3c], rec["after"][0xc48], rec["after"][0xc54]))
    return out, "WAIT 2 (times out) with each word set by the CPU before the start:\n" + "\n".join(lines)


@experiment("c08", needs=("start_value",))
def ex_c08(s, t):
    """FMSS C08 as a state code: read by the CPU while a program runs, waits, stalls, blocks, is paused or is held"""
    loop = "    movi r0, #20000\nloop:\n    sub  r0, r0, #0x1\n    bnz  r0, loop\n"
    cases = [("running (loop)", loop, 0xfff5), ("waiting (wait 2)", "    wait 2\n", 0xfff5),
             ("stalled (opcode 0x09)", "    .insn 0x09, 0, 0, 0\n", 0xfff5),
             ("timer (0x15 from 0xff)", "    .insn 0x15, 0, 0, 0\n", 0xfff5),
             ("blocked (0x12)", "    .insn 0x12, 0, 0, 0\n", 0xfff5),
             ("paused (start bit 1)", loop, 0x3), ("held (start bit 3)", loop, 0x9)]
    out, lines = {}, []
    for label, body, sv in cases:
        src = t.prologue(c58=0) + body + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        # the probe gives up after 1 ms and takes the after-snapshot before it stops the sequencer
        rec = probe(s, t, assemble(t, src), timeout_us=1000, start_value=sv)
        out[label] = rec
        lines.append("  %-24s -> %-7s C08 %x, PC %x, FMSS_CTRL %08x, CS_IRQ %08x, C14 %02x" % (
            label, rec["why"], rec["after"][0xc08], rec["pc"], rec["after"][0xc00], rec["cs_irq"], rec["after"][0xc14]))
    return out, "C08 1 ms after the start:\n" + "\n".join(lines)


@experiment("pause")
def ex_pause(s, t):
    """Start-word bit 1 while a program runs: does it pause it mid-way, and does a later start without it resume?"""
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r0, #20000\nloop:\n    sub  r0, r0, #0x1\n    bnz  r0, loop\n" \
        "    movi r6, #0x600d600d\n    end\n" + t.trap()
    code = assemble(t, src)
    cases = [("no writes (control, ~8.6 ms)", [], []),
             ("0xfff7 at 1 ms", [(0xc00, 0xfff7)], []),
             ("0xfff7 at 1 ms, 0xfff5 at 5 ms", [(0xc00, 0xfff7)], [(0xc00, 0xfff5)]),
             ("0xfffd at 1 ms, 0xfff5 at 5 ms", [(0xc00, 0xfffd)], [(0xc00, 0xfff5)]),
             ("0x10001 at 1 ms, 0x1 at 5 ms", [(0xc00, 0x10001)], [(0xc00, 0x1)]),
             ("0xfff5 at 1 ms (restart?)", [(0xc00, 0xfff5)], []),
             ("0xfff4 at 1 ms", [(0xc00, 0xfff4)], [])]
    out, lines = {}, []
    for label, l1, l2 in cases:
        rec = probe(s, t, code, late=l1, late_us=1000, late2=l2, late2_us=5000, timeout_us=30000, flags=F_SAMPLE_REGS)
        out[label] = rec
        r0s = [x["r"][0] for x in rec["samples"]]
        lines.append("  %-32s -> %-7s %6d us, r0 %5d, C08 %x, FMSS_CTRL %08x, r0 sampled %s" % (
            label, rec["why"], rec["elapsed_us"], rec["r"][0], rec["after"][0xc08], rec["after"][0xc00],
            " ".join(str(v) for v in r0s[::max(1, len(r0s) // 8)][:9])))
    return out, "a 20 000-iteration loop with start-word writes while it runs:\n" + "\n".join(lines)


@experiment("ctrlbits", needs=("start_value",))
def ex_ctrlbits(s, t):
    """FMSS_CTRL start word: the driver writes 0xfff5. Each other bit alone (with bit 0) on a 1000-iteration loop:
    does any change speed, stepping, interrupts or the start itself?"""
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r0, #1000\nloop:\n    sub  r0, r0, #0x1\n    bnz  r0, loop\n" \
        "    movi r6, #0x600d600d\n    end\n" + t.trap()
    code = assemble(t, src)
    vals = [0xfff5, 0x1] + [0x1 | 1 << k for k in range(1, 32) if k not in (16, 17)] + \
        [0xfff7, 0xfffd, 0xffff, 0xfff4, 0x10001, 0x20001]
    out, lines = {}, []
    for v in vals:
        rec = probe(s, t, code, timeout_us=50000, start_value=v, flags=F_SAMPLE_REGS)
        out["0x%08x" % v] = rec
        pcs = sorted({x["pc"] for x in rec["samples"]})
        line = "  start 0x%08x -> %-7s %6d us, r0 %4d, r6 %08x, CS_IRQ %08x, FMSS_CTRL after %08x, %d samples%s" % (
            v, rec["why"], rec["elapsed_us"], rec["r"][0], rec["r"][6], rec["cs_irq"], rec["after"][0xc00],
            len(rec["samples"]), ", pcs %s" % " ".join("%x" % p for p in pcs[:8]) if len(pcs) <= 8 else "")
        lines.append(line)
        print(line)
        sys.stdout.flush()
    return out, "start words (loop of 1000 x 2 instructions, ~430 us at 0xfff5):\n" + "\n".join(lines)


@experiment("irqline", needs=("start_value",))
def ex_irqline(s, t):
    """Which sequencer states raise the CPU interrupt line (VIC1 raw bit 22), and is start-word bit 6 its enable?"""
    block = "    .insn 0x12, 0, 0, 0\n"
    cases = [("END, start 0xfff5", "", 0xfff5), ("END, start 0xffb5 (no bit 6)", "", 0xffb5),
             ("END, start 0x41", "", 0x41), ("END, start 0x1", "", 0x1),
             ("0x10 then block, 0xfff5", "    .insn 0x10, 0, 0, 0\n" + block, 0xfff5),
             ("0x10 then block, 0xffb5", "    .insn 0x10, 0, 0, 0\n" + block, 0xffb5),
             ("block only, 0xfff5", block, 0xfff5),
             ("timeout (C58 0) then block, 0xfff5", "    wait 3\n" + block, 0xfff5),
             ("timeout (C58 0) then block, 0xffb5", "    wait 3\n" + block, 0xffb5)]
    out, lines = {}, []
    for label, body, sv in cases:
        src = t.prologue(c58=0) + body + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, start_value=sv, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-36s -> %-7s CS_IRQ %08x, line seen %d, line at the end %d, preflight line %s" % (
            label, rec["why"], rec["cs_irq"], rec["irq_seen"], rec["irq_end"], "-"))
    return out, "VIC1 raw bit 22 while polling and at the end:\n" + "\n".join(lines)
