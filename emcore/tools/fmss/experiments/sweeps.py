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

"""Systematic sweeps: every opcode, every register word, the FMC window, FMC_28 and the 0x100 block."""
import struct
import sys

from ..probe import probe, assemble, answer, changes, fmc_window_dump, F_WIN400
from .registry import experiment


SWEEP_PRAM = [(0xd40 + 4 * i, 0xc0de0000 | (0xd40 + 4 * i)) for i in range(16)]


SWEEP_FORMS = (("reg 3, arg 1, imm 0", 3, 1, 0), ("reg 3, arg 0xd70, imm 0xd74", 3, 0xd70, 0xd74),
               ("reg 0, arg 0, imm 0xffffffff", 0, 0, 0xffffffff))


def sweep_seeds(s, t):
    """r0..r7 (r6 left as the marker): pointers into the scratch area whose low 12 bits name parameter RAM
    (0xd40 + 4 i), so an opcode that turns out to be a memory access (RAM by address or the FMC window by offset)
    lands somewhere harmless and visible"""
    base = (t.scratch_addr(s) + 0xfff) & ~0xfff         # pages 0..6 from here stay inside the 32 KB area
    return {i: base + 0x1000 * (i if i < 6 else 6) + 0xd40 + 4 * i for i in range(8) if i != 6}


def scratch_pattern(t):
    return bytes((i * 7 + (i >> 8)) & 0xff for i in range(t.scratch_bytes))


@experiment("opsweep")
def ex_opsweep(s, t):
    """Every opcode 0x00..0xff without a known meaning, in three operand forms: registers seeded with harmless
    pointers, C58 = 0 (a fault flags and continues), timeout 20 ms. What ends, faults, blocks, stalls or changes?"""
    seeds = sweep_seeds(s, t)
    pat = scratch_pattern(t)
    out, lines, classes = {}, [], {}
    for op in range(256):
        if op in t.known_ops:
            continue
        for label, reg, arg, imm in SWEEP_FORMS:
            src = t.prologue(c58=0) + "".join("    movi r%d, #0x%08x\n" % (i, v) for i, v in sorted(seeds.items())) + \
                "    movi r6, #0x0\n    .insn 0x%02x, %d, 0x%x, 0x%08x\n    movi r6, #0x600d600d\n    end\n" % (
                    op, reg, arg, imm) + t.trap()
            rec = probe(s, t, assemble(t, src), params=SWEEP_PRAM, timeout_us=20000, scratch=pat, scratch_read=t.scratch_bytes)
            sc = rec.pop("scratch") or b""
            rec["scratch_diff"] = [(i, pat[i], sc[i]) for i in range(len(sc)) if sc[i] != pat[i]][:64]
            out["%02x/%s" % (op, label)] = rec
            r6 = rec["r"][6]
            how = "continued" if r6 == 0x600d600d else "vector" if r6 == 0xfa17fa17 else "stopped" if r6 == 0 else \
                "r6=%08x" % r6
            ms = "fast" if rec["elapsed_us"] < 1500 else "%.1f ms" % (rec["elapsed_us"] / 1000.0)
            regs = " ".join("r%d=%08x" % (i, rec["r"][i]) for i in seeds if rec["r"][i] != seeds[i])
            win = " ".join("0x%03x:%08x->%08x" % (o, a, b) for o, (a, b) in sorted(changes(rec).items())
                           if o not in t.ctrl_skip | {0xc4c, 0xc58} and not 0xc18 <= o < 0xc38)   # the prologue sets those
            mem = ("scratch %d bytes changed from +0x%x" % (len(rec["scratch_diff"]), rec["scratch_diff"][0][0])
                   if rec["scratch_diff"] else "")
            cls = "%s %s %s CS_IRQ %02x%s%s%s" % (rec["why"], how, ms, rec["cs_irq"] & 0xff, " regs" if regs else "",
                                                   " window" if win else "", " memory" if mem else "")
            classes.setdefault(cls, []).append("%02x" % op if label == SWEEP_FORMS[0][0] else None)
            line = "  op %02x %-28s -> %s; %s" % (op, label, cls, "; ".join(x for x in (regs, win, mem) if x)
                                                    or "no other change")
            lines.append(line)
            print(line)
            sys.stdout.flush()
    summ = ["  %-50s %s" % (c, " ".join(o for o in ops if o)) for c, ops in sorted(classes.items())]
    return out, "opcode classes (first operand form):\n" + "\n".join(summ) + "\nall runs:\n" + "\n".join(lines)


@experiment("rwmask", needs=("win400",))
def ex_rwmask(s, t):
    """Register map: for every word of the 0x000, 0x800, 0xc00 and 0x400 windows (action registers left out) what a
    CPU write of all ones and of zero reads back before the start (writable, read-only and write-1-to-clear bits)"""
    out, lines, got_idle = {}, [], {}
    code = assemble(t, "    end\n")
    for win in (0x000, 0x800, 0xc00, 0x400):
        for off in range(win, win + 0x100, 4):
            if off in t.rw_skip:
                continue
            print("  0x%03x" % off, end="")
            sys.stdout.flush()
            got = {}
            # the value before this register's writes (they are put back afterwards: device, a5 patterns left in
            # 0x044 / 0x844 enabled FMC and ECC interrupt sources for every later run)
            start = probe(s, t, code, timeout_us=20000, quiet=True, flags=F_WIN400 if win == 0x400 else 0)
            got_idle[off] = start["before"][off]
            for v in (0xffffffff, 0x00000000, 0xa5a5a5a5):
                rec = probe(s, t, code, params=[(off, v)], timeout_us=20000, quiet=True,
                            flags=F_WIN400 if win == 0x400 else 0)
                got[v] = (rec["before"][off], rec["after"][off], rec["why"])
            pre = probe(s, t, code, params=[(off, got_idle[off])], timeout_us=20000, quiet=True,
                        flags=F_WIN400 if win == 0x400 else 0)
            idle = pre["before"][off]
            ones, zeros, alt = got[0xffffffff][0], got[0][0], got[0xa5a5a5a5][0]
            rw = ones & ~zeros & 0xffffffff
            out["0x%03x" % off] = {"idle": idle, "ones": ones, "zeros": zeros, "a5": alt,
                                   "after_ones": got[0xffffffff][1], "after_zeros": got[0][1],
                                   "why": [got[v][2] for v in got]}
            note = "" if all(got[v][2] == "END" for v in got) else "  (runs: %s)" % ",".join(got[v][2] for v in got)
            if rw or ones != idle or zeros != idle:
                lines.append("  0x%03x idle %08x  ones %08x zeros %08x a5 %08x -> writable %08x%s" % (
                    off, idle, ones, zeros, alt, rw, note))
            elif note:
                lines.append("  0x%03x idle %08x  (no bit kept a write)%s" % (off, idle, note))
        print()
    return out, "registers that keep a CPU write or change (others read the same idle value):\n" + "\n".join(lines)


@experiment("fmcmap", needs=("pld_pst",))
def ex_fmcmap(s, t):
    """The whole 4 KB FMC window as the sequencer sees it (pld of every word into RAM): registers outside the
    windows the CPU snapshot covers"""
    return fmc_window_dump(s, t)


@experiment("blk100", needs=("pld_pst",))
def ex_blk100(s, t):
    """The block at 0x100..0x1fc (repeated at 0x300, 0x500, 0x700): its value, what a sequencer write of all ones and
    of zero leaves (read back at 0x100 and at the 0x300 copy), then the old value written back"""
    base = t.scratch_addr(s)
    src = t.prologue(c58=0) + """    movi r0, #0x%08x
    movi r1, #0x100
    movi r2, #0x40
    movi r4, #0xffffffff
    movi r5, #0x0
loop:
    pld  r3, [r1]
    sti  [r0], r3
    mov  r7, r3
    pst  [r1], r4
    pld  r3, [r1]
    add  r0, r0, #0x4
    sti  [r0], r3
    add  r6, r1, #0x200
    pld  r3, [r6]
    add  r0, r0, #0x4
    sti  [r0], r3
    pst  [r1], r5
    pld  r3, [r1]
    add  r0, r0, #0x4
    sti  [r0], r3
    pst  [r1], r7
    add  r0, r0, #0x4
    add  r1, r1, #0x4
    sub  r2, r2, #0x1
    bnz  r2, loop
    movi r6, #0x600d600d
    end
""" % base + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=100000, scratch=b"\xee" * 1024, scratch_read=1024)
    if answer(rec):
        return rec, "blk100: " + answer(rec)
    w = struct.unpack("<256I", rec["scratch"])
    lines = []
    for i in range(64):
        old, ones, mirror, zero = w[4 * i:4 * i + 4]
        if old or ones or zero:
            lines.append("  0x%03x: value %08x, after ~0 %08x (0x%03x: %08x), after 0 %08x%s" % (
                0x100 + 4 * i, old, ones, 0x300 + 4 * i, mirror, zero,
                "  writable %08x" % (ones & ~zero & 0xffffffff) if ones != zero else ""))
    rec["words"] = w
    return rec, "%s; words that are not 0 in any read:\n" % rec["why"] + "\n".join(lines)


@experiment("fmc28", needs=("pld_pst",))
def ex_fmc28(s, t):
    """What does FMC_28 count? 64 instructions of one kind against an empty program"""
    a = t.scratch_addr(s)
    kinds = [("empty", ""), ("movi", "    movi r5, #0x0\n"), ("pld pram", "    pld  r5, [r2]\n"),
             ("pst pram", "    pst  [r2], r5\n"), ("sti RAM", "    sti  [r1], r5\n"), ("ldi RAM", "    ldi  r5, [r1]\n"),
             ("wr pram", "    wr   [0xd70], #0x0\n"), ("ld pram", "    ld   r5, [0xd70]\n"),
             ("ld FMC_STATUS", "    ld   r5, [FMC_STATUS]\n"), ("wr FMC_ADDR0", "    wr   [FMC_ADDR0], #0x0\n"),
             ("ld FMC_28", "    ld   r5, [FMC_28]\n")]
    out, lines = {}, []
    base = None
    for label, body in kinds:
        src = t.prologue(c58=0) + "    movi r1, #0x%x\n    movi r2, #0xd70\n" % a + body * 64 + "    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, scratch=bytes(64))
        out[label] = rec
        d = (rec["after"][0x028] - rec["before"][0x028]) & 0x1ff
        if base is None:
            base = d
        lines.append("  %-14s -> FMC_28 %03x -> %03x: +%3d (%+d vs empty)" % (label, rec["before"][0x028],
                     rec["after"][0x028], d, d - base))
    return out, "FMC_28 across a run (before = after the CPU set-up):\n" + "\n".join(lines)
