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

"""Round 24: every open point that a program on the device can settle. Register numbers 8..255 in every operand role,
C58 bit 1, the wait events and wait2 conditions during chip and engine activity, C50 bits 0 and 8, the unused control
words, CS_STATUS read by the program, the status latch repeated, the 0x044 routing, FMC_CTRL0 bits 9 / 11 / 24,
FMC_DMA_CTRL and the 0x102 drain select on a real page read, the FMC_STATUS power-on bits, the ECC words 0x940..0x99c,
the restart of the sequence of the modes without bit 8, the ECC_DONE bit 5 rule, and the program time of every page
of a block. NAND writes are allowed (scratch blocks)."""
import dataclasses
import struct
import sys

from ..probe import probe, assemble, soft_reset, WAIT_END, FULLMAP
from .registry import experiment
from .followup import _dma_in, _dma_out, _kick, _ready, _address, _send

END = "    movi r6, #0x600d600d\n    end\n"
SEED = [0x5e000000 | i << 16 | 0x5e5e for i in range(8)]          # r0..r7 before each role
PRE = {0xc10: 0x5a, 0xc48: 0x48c48c48, 0xc54: 0x554, 0xc3c: 0x3c3c3c3c}   # control words preloaded by the program
VOLATILE = {0xc00, 0xc04, 0xc08, 0xc0c, 0xc14, 0xc38, 0xc40, 0xc44, 0xc50, 0x028, 0x228, 0x050, 0x250, 0x428, 0x628,
            0x450, 0x650, 0x9a4}


def _seed(vals=SEED):
    return "".join("    movi r%d, #0x%x\n" % (i, v & 0xffffffff) for i, v in enumerate(vals))


def _preload():
    return "".join("    wr   [0x%03x], #0x%x\n" % (o, v) for o, v in sorted(PRE.items()))


def _name(v, extra=None):
    """a value read back, named by where it came from"""
    names = {SEED[i]: "r%d" % i for i in range(8)}
    names.update({v2: "C%02X" % (o & 0xff) for o, v2 in PRE.items()})
    names.update(extra or {})
    names.setdefault(0, "0")
    return names.get(v, "%08x" % v)


def _groups(sig):
    """{N: signature} -> lines 'N, N..M: signature', grouped by signature"""
    by = {}
    for n, g in sorted(sig.items()):
        by.setdefault(g, []).append(n)
    out = []
    for g, ns in sorted(by.items(), key=lambda x: (-len(x[1]), x[1][0])):
        runs, start, prev = [], ns[0], ns[0]
        for n in ns[1:] + [None]:
            if n is not None and n == prev + 1:
                prev = n
                continue
            runs.append("%d" % start if start == prev else "%d..%d" % (start, prev))
            if n is not None:
                start = prev = n
        out.append("  %s (%d numbers): %s" % (", ".join(runs), len(ns), g))
    return out


@experiment("regsweep", needs=("nand_seq", "fullmap", "pld_pst"))
def ex_regsweep(s, t):
    """Register numbers 8..255 in every operand role (only samples were tested: 9, 11..16, 24, 255 as a movi
    destination, 9 / 15 / 255 as an argument-field source, 9 / 14 as a register-field source, 9 as an ldi address; r10
    never as a source). r0..r7 are seeded with distinct values before each role and the program first writes distinct
    values to the writable control words C10, C48, C54 and C3C, so that "reads r0", "reads 0" and "reads a control word
    (0xc18 + 4 N)" can be told apart. Program A (destinations, number 8 left out: a write is a jump): movi, mov and add
    to N, r0..r7 stored to parameter RAM after each, and the whole window compared. Program B (sources): mov / add /
    or with N in the argument field, st / bnz / bz with N in the register field, jr N. Program C (addresses, run for
    all numbers after A and B): ldi with N as the address register (r0..r7 and C48 / C3C hold scratch addresses with
    markers), pld with N as the address register (r0..r7 hold parameter-RAM offsets with markers). Numbers are grouped
    by identical behaviour"""
    a = t.scratch_addr(s)
    out, lines = {}, []
    sigA, sigB, sigC = {}, {}, {}
    ref = None
    for n in range(8, 256):
        # A: destinations
        if n != 8:
            src = t.prologue(c58=0) + _preload()
            for k, role in enumerate(("    .insn 0x05, %d, 0, 0x5eedd00d\n" % n, "    .insn 0x06, %d, 1, 0\n" % n,
                                      "    .insn 0x0c, %d, 1, 0x1\n" % n)):
                src += _seed() + role + "".join("    st   [0x%x], r%d\n" % (0xd00 + 32 * k + 4 * i, i) for i in range(8))
            src += "    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END | FULLMAP, quiet=True)
            out["A/%d" % n] = rec
            fm = rec.get("fullmap") or [[0] * 1024, [0] * 1024]
            if ref is None:
                ref = fm
            win = [(4 * i, fm[1][i]) for i in range(1024) if 4 * i not in VOLATILE and not 0xd00 <= 4 * i < 0xd60
                   and not 0xc18 <= 4 * i < 0xc38 and fm[1][i] != ref[1][i]]
            parts = []
            for k, lab in enumerate(("movi", "mov", "add")):
                regs = [rec["after"][0xd00 + 32 * k + 4 * i] for i in range(8)]
                ch = ["r%d=%s" % (i, _name(v, {0x5eedd00d: "imm", SEED[1] + 1: "r1+1"})) for i, v in enumerate(regs)
                      if v != SEED[i]]
                parts.append("%s: %s" % (lab, " ".join(ch) or "no register"))
            g = "%s; %s" % (rec["why"], "; ".join(parts))
            if win:
                g += "; window " + " ".join("%03x=%08x" % w for w in win[:8])
            sigA[n] = g
        # B: sources
        body = t.prologue(c58=0) + "    bnz  r5, restarted\n" + _preload()
        body += _seed() + "    .insn 0x06, 1, %d, 0\n    st   [0xd00], r1\n" % n
        body += _seed() + "    .insn 0x0c, 1, %d, 0x1\n    st   [0xd04], r1\n" % n
        body += _seed() + "    .insn 0x0b, 1, %d, 0\n    st   [0xd08], r1\n" % n
        body += _seed() + "    .insn 0x02, %d, 0xd0c, 0\n" % n
        for k, (op, r0) in enumerate(((0x0e, SEED[0]), (0x0e, 0), (0x17, SEED[0]), (0x17, 0))):
            body += _seed([r0] + SEED[1:]) + "    movi r1, #0x7a7e7a7e\n    .insn 0x%02x, %d, 0, br%d\n" \
                "    movi r1, #0xf0f0f0f0\nbr%d:\n    st   [0x%x], r1\n" % (op, n, k, k, 0xd10 + 4 * k)
        body += "".join("    movi r%d, #jr%d\n" % (i, i) for i in range(8)) + "    .insn 0x0f, 0, %d, 0\n" % n + \
            "    movi r1, #0xfa11fa11\n    st   [0xd20], r1\n    end\n"
        for i in range(8):
            body += "jr%d:\n    movi r1, #0x%x\n    st   [0xd20], r1\n    end\n" % (i, 0x1a4d0000 | i)
        body += "restarted:\n    movi r1, #0x5ea5ea5e\n    st   [0xd20], r1\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, body), params=[(0xc2c, 0)], timeout_us=20000, flags=WAIT_END, quiet=True)
        out["B/%d" % n] = rec
        aft = rec["after"]
        br = {0x7a7e7a7e: "taken", 0xf0f0f0f0: "not taken"}
        jr = {0xfa11fa11: "fell through", 0x5ea5ea5e: "jumped to 0", 0: "no landing"}
        jr.update({0x1a4d0000 | i: "jumped to r%d" % i for i in range(8)})
        sigB[n] = "%s; mov %s; add+1 %s; or %s; st %s; bnz %s/%s; bz %s/%s (r0 != 0 / r0 = 0); jr %s" % (
            rec["why"], _name(aft[0xd00]), _name(aft[0xd04], {SEED[0] + 1: "r0+1", 1: "0+1"}),
            _name(aft[0xd08], {SEED[1] | SEED[0]: "r1|r0"}), _name(aft[0xd0c]),
            br.get(aft[0xd10], "%08x" % aft[0xd10]), br.get(aft[0xd14], "%08x" % aft[0xd14]),
            br.get(aft[0xd18], "%08x" % aft[0xd18]), br.get(aft[0xd1c], "%08x" % aft[0xd1c]),
            jr.get(aft[0xd20], "%08x" % aft[0xd20]))
        if n % 32 == 31:
            print("regsweep: A/B done to %d" % n)
            sys.stdout.flush()
    lines.append(" destinations (movi / mov r1 / add r1 + 1 into N; registers that changed, named by source):")
    lines += _groups(sigA)
    lines.append(" sources (named: rK = seed of rK, CXX = preloaded control word, 0):")
    lines += _groups(sigB)
    # C: addresses (bus reads through ldi may hit any address: last)
    marks = [0xcafe0000 | i for i in range(8)] + [0xcafe0c48, 0xcafe0c3c]
    scratch = b"".join(struct.pack("<I", m) for m in marks)
    pmark = {0xd40 + 4 * i: 0xd00d0000 | i for i in range(8)}
    for n in range(8, 256):
        src = t.prologue(c58=0) + "    wr   [0xc48], #0x%x\n    wr   [0xc3c], #0x%x\n" % (a + 32, a + 36)
        src += _seed([a + 4 * i for i in range(8)]) + "    .insn 0x03, 1, %d, 0\n    st   [0xd00], r1\n" % n
        src += _seed([0xd40 + 4 * i for i in range(8)]) + "    .insn 0x18, 1, %d, 0\n    st   [0xd04], r1\n" % n
        src += "    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=list(pmark.items()), timeout_us=20000, flags=WAIT_END,
                    scratch=scratch, quiet=True)
        out["C/%d" % n] = rec
        nm = {m: "addr r%d" % i for i, m in enumerate(marks[:8])}
        nm.update({0xcafe0c48: "addr C48", 0xcafe0c3c: "addr C3C"})
        pm = {v: "offset r%d" % ((o - 0xd40) // 4) for o, v in pmark.items()}
        sigC[n] = "%s; ldi %s; pld %s" % (rec["why"], nm.get(rec["after"][0xd00], "%08x" % rec["after"][0xd00]),
                                          pm.get(rec["after"][0xd04], "%08x" % rec["after"][0xd04]))
    lines.append(" addresses (ldi: which scratch marker was read; pld: which parameter-RAM marker; else the word read):")
    lines += _groups(sigC)
    soft_reset(s, t)
    return out, "register numbers 8..255, every role:\n" + "\n".join(lines)


@experiment("c58b1", needs=("nand_seq",))
def ex_c58b1(s, t):
    """C58 bit 1: an END ran the fault vector once after the CPU wrote C58 = 2 and then C4C, never when the program
    wrote C4C. The CPU writes in different orders and repeats (no clean-up writes before them): which write arms the
    one-shot, and is it used up?"""
    src = "    movi r0, #0x57a27000\n    end\n" + "    nop\n" * 6 + "    movi r1, #0x3a2c3a2c\n    end\n"
    code = assemble(t, src)
    vec = 0x40
    prog_c4c = assemble(t, "    wr   [FMSS_C4C], #0x40\n    movi r0, #0x57a27000\n    end\n" + "    nop\n" * 5 +
                        "    movi r1, #0x3a2c3a2c\n    end\n")
    zero = [(0xc58, 0), (0xc4c, 0), (0xc5c, 0), (0xc18, 0), (0xc1c, 0)]
    cases = [("C58=2, C4C", [(0xc58, 2), (0xc4c, vec)], code, 2),
             ("C4C, C58=2", [(0xc4c, vec), (0xc58, 2)], code, 2),
             ("C58=2 only (C4C left from the previous case)", [(0xc58, 2)], code, 1),
             ("C4C only (C58=2 left)", [(0xc4c, vec)], code, 2),
             ("C58=2, C4C, C4C", [(0xc58, 2), (0xc4c, vec), (0xc4c, vec)], code, 2),
             ("C58=2, C4C, C5C=0", [(0xc58, 2), (0xc4c, vec), (0xc5c, 0)], code, 2),
             ("C58=2, C4C, C58=2", [(0xc58, 2), (0xc4c, vec), (0xc58, 2)], code, 2),
             ("C4C=0x40 by CPU, C58=2, program writes C4C", [(0xc4c, vec), (0xc58, 2)], prog_c4c, 2),
             ("C58=3, C4C", [(0xc58, 3), (0xc4c, vec)], code, 2),
             ("C58=6, C4C", [(0xc58, 6), (0xc4c, vec)], code, 2)]
    out, lines = {}, []
    for label, params, prog, runs in cases:
        if not label.startswith(("C58=2 only", "C4C only")):
            probe(s, t, assemble(t, "    end\n"), params=zero, clean=False, timeout_us=20000, quiet=True)
        res = []
        for k in range(runs):
            rec = probe(s, t, prog, params=(params if k == 0 else []) + [(0xc18, 0), (0xc1c, 0)], clean=False,
                        timeout_us=20000, flags=WAIT_END, quiet=True)
            out["%s/%d" % (label, k)] = rec
            r0, r1 = rec["r"][0] == 0x57a27000, rec["r"][1] == 0x3a2c3a2c
            res.append("%s (C5C %x)" % ({(True, False): "END", (True, True): "END, then the vector",
                                         (False, True): "vector only", (False, False): "neither"}[(r0, r1)],
                                        rec["after"][0xc5c]))
        lines.append("  %-46s %s" % (label, " | then: ".join(res)))
    probe(s, t, assemble(t, "    end\n"), params=zero, clean=False, timeout_us=20000, quiet=True)
    return out, "C58 bit 1 (run 1 | run 2 without new writes):\n" + "\n".join(lines)


def _sweep_src(order):
    """wait N (0..15) and wait2 N (0..255) in turn, each followed by the fault flag (CS_IRQ b2) stored to scratch and
    acknowledged; r2 = store address"""
    src = ""
    for kind, n in order:
        src += "    %s %d\n    ld   r1, [CS_IRQ], 0x4\n    sti  [r2], r1\n    add  r2, r2, #0x4\n" \
               "    wr   [CS_IRQ], #0x4\n" % (kind, n)
    return src


@experiment("evsweep2", needs=("nand_seq", "start_value"))
def ex_evsweep2(s, t):
    """wait events 0..15 and wait2 conditions 0..255 in states not tried: during an erase, a page program and a page
    read (chip busy), an ECC encode in flight, a DMA in flight, with every 0x044 / 0x844 interrupt source enabled and a
    command flag pending, after a second-set command, and idle. Short timeout (start word 0x3f15: 64 x 8 cycles), C58 0
    so a timeout continues; a pass = no fault. Each state in forward and in reverse order (the chip state changes
    while the sweep runs)"""
    a, e = t.scratch_addr(s), t.events
    rows = t.scratch_row + 40
    order = [("wait", n) for n in range(16)] + [("wait2", n) for n in range(256)]
    st = {
        "idle, CE0 selected": "",
        "erase busy": t.cmd(0x60) + "    wr   [FMC_ANUM], #0x2\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMCTRL1], #0x1\n"
                      "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % (rows, e["addr"]) + t.cmd(0xd0),
        "program busy (80h addr 10h)": t.cmd(0x80) + _address(t, rows + 1) + t.cmd(0x10),
        "read busy (00h addr 30h)": t.cmd(0x00) + _address(t, rows) + t.cmd(0x30),
        "ECC encode in flight": "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1102\n",
        "DMA in flight": "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
                         "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n" % a,
        "0x044 / 0x844 all on, command flag pending": "    wr   [0x044], #0xffffffff\n    wr   [0x844], #0x1ff\n"
                                                      "    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"],
        "second set: command sent": "    wr   [0x404], #@FMCTRL1_INIT@\n    wr   [0x448], #@STATUS_CLEAR@\n"
                                    "    wr   [0x400], #@CE0@\n    wr   [0x408], #0x70\n",
    }
    out, lines, base = {}, [], None
    for label, act in st.items():
        res = []
        for rev in (False, True):
            soft_reset(s, t)
            o = order[::-1] if rev else order
            src = t.nand_prologue(0) + act + "    movi r2, #0x%x\n" % a + _sweep_src(o) + \
                "    wr   [0x044], #0x0\n    wr   [0x844], #0x0\n    wr   [FMC_CTRL0], #@CE0@\n" + _ready(t, 0) + \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=400000, start_value=0x3f15, flags=WAIT_END,
                        scratch=b"\xee" * (4 * len(o)), scratch_read=4 * len(o), quiet=True)
            out["%s/%s" % (label, "rev" if rev else "fwd")] = rec
            sc = rec["scratch"] or b""
            vals = struct.unpack("<%dI" % len(o), sc) if len(sc) == 4 * len(o) else [0xeeeeeeee] * len(o)
            ok = {o[i] for i in range(len(o)) if vals[i] == 0}
            bad = [o[i] for i in range(len(o)) if vals[i] not in (0, 4)]
            res.append((ok, bad, rec["why"], rec["elapsed_us"]))
        ok = res[0][0] | res[1][0]
        both = res[0][0] & res[1][0]
        if base is None:
            base = ok
        w = sorted(n for k, n in ok if k == "wait")
        w2 = sorted(n for k, n in ok if k == "wait2")
        extra = sorted(ok - base)
        lines.append("  %-44s wait passes %s; wait2 passes %s%s%s; %s/%s %d/%d us" % (
            label, w, w2, "; only in one order: %s" % sorted(ok - both) if ok - both else "",
            "; NOT STORED %d" % len(res[0][1]) if res[0][1] else "", res[0][2], res[1][2], res[0][3], res[1][3]))
        if extra:
            lines.append("      beyond idle: %s" % extra)
    soft_reset(s, t)
    return out, "wait / wait2 sweep per state (short timeout; pass = no fault):\n" + "\n".join(lines)


@experiment("c50fault", needs=("nand_seq",))
def ex_c50fault(s, t):
    """C50 bit 0 (the original fault handlers test it and write 1 to it) and bit 8. A fault (timeout of wait 10, and an
    illegal opcode) while a command flag is pending, with C58 0 and 4; C50 is read before the fault, right after it (C58
    0) or in the handler (C58 4), and after the handler's write of 1. Bit 8: C50 after a soft reset, after an END-only
    run, after each kind of operation"""
    e = t.events
    out, lines = {}, []
    pend = "    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"]
    for c58 in (0, 4):
        for label, flt in (("timeout (wait 10)", "    wait 10\n"), ("illegal 0x30", "    .insn 0x30, 0, 0, 0\n")):
            for p in (False, True):
                src = t.nand_prologue(0) + "    wr   [FMSS_C58], #0x%x\n    wr   [FMSS_C4C], #hnd\n" % c58
                src += (pend if p else "") + "    ld   r1, [0xc50]\n" + flt + "    ld   r2, [0xc50]\n" + \
                    "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + \
                    "hnd:\n    ld   r3, [0xc50]\n    ld   r5, [0xc3c]\n    wr   [0xc50], #0x1\n    ld   r4, [0xc50]\n" \
                    "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x4a4d4a4d\n    end\n" + t.trap()
                rec = probe(s, t, assemble(t, src), params=[(0xc24, 0), (0xc28, 0), (0xc20, 0), (0xc1c, 0)],
                            timeout_us=30000, flags=WAIT_END, quiet=True)
                r = rec["r"]
                out["C58 %d/%s/%s" % (c58, label, p)] = rec
                lines.append("  C58 %d %-18s %-16s before %08x | after %08x | handler %08x, after w1 %08x, C3C %08x | "
                             "end C50 %08x, %s r6 %08x" % (c58, label, "command pending" if p else "nothing pending",
                                                           r[1], r[2], r[3], r[4], r[5], rec["c50"], rec["why"], r[6]))
    for label, src in (("after a soft reset, END only", "    end\n"), ("again", "    end\n"),
                       ("status latch", t.nand_prologue(0) + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait %d\n" %
                        e["status"] + "    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n" + t.trap())):
        if label.startswith("after"):
            soft_reset(s, t)
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, quiet=True)
        out["bit8/" + label] = rec
        lines.append("  C50 bit 8: %-30s before %08x after %08x" % (label, rec["before"][0xc50], rec["after"][0xc50]))
    soft_reset(s, t)
    return out, "C50 around faults (bit 0) and bit 8:\n" + "\n".join(lines)


@experiment("ctrlwords", needs=("nand_seq", "fullmap"))
def ex_ctrlwords(s, t):
    """C10, C48, C54 hold values but have no known effect; C3C holds the vector address inside a handler. The same
    operations (a timeout fault with C58 4, an illegal opcode, wait2 33, a DMA + ECC encode, tmwait) with the three
    words written all ones by the program vs zero; and C3C read in handlers entered from three different faulting
    offsets, and in a second fault inside the handler"""
    a, e = t.scratch_addr(s), t.events
    ops = [("timeout, C58 4", "    wr   [FMSS_C58], #0x4\n    wait 10\n"),
           ("illegal, C58 4", "    wr   [FMSS_C58], #0x4\n    .insn 0x30, 0, 0, 0\n"),
           ("wait2 33", "    wait2 33\n"),
           ("DMA + ECC encode", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x180, e["ecc"]) +
            "    wr   [FMC_CTRL0], #@CE_NONE@\n"),
           ("tmwait twice", "    tmwait\n    tmwait\n")]
    out, lines = {}, []
    for label, body in ops:
        row = []
        for val in (0, 0xffffffff, 0x55555555):
            pre = "    wr   [0xc10], #0x%x\n    wr   [0xc48], #0x%x\n    wr   [0xc54], #0x%x\n" % (val, val, val)
            src = t.prologue(c58=0) + t.state_setup() + pre + body + END + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP, scratch=bytes(512), quiet=True)
            out["%s/%08x" % (label, val)] = rec
            fm = rec.get("fullmap")
            row.append("%s %s %dus fault %dus CS_IRQ %02x r6 %08x 0x900 %08x" % (
                "%08x" % val, rec["why"], rec["elapsed_us"], rec["fault_at_us"], rec["cs_irq"], rec["r"][6],
                fm[1][0x900 // 4] if fm else 0))
        lines.append("  %-18s %s" % (label, " | ".join(row)))
    # C3C in handlers
    for k, at in enumerate((0, 3, 9)):
        src = t.prologue(c58=4) + "    wr   [FMSS_C4C], #hnd\n    movi r2, #f0\n" + "    nop\n" * at + \
            "f0:\n    wait 10\n" + END + "hnd:\n    bnz  r7, again\n    movi r7, #0x1\n    ld   r1, [0xc3c]\n" \
            "    ld   r3, [0xc38]\n    wait 10\nagain:\n    ld   r4, [0xc3c]\n    movi r6, #0x4a4d4a4d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xc34, 0), (0xc1c, 0), (0xc24, 0), (0xc28, 0)], timeout_us=40000,
                    flags=WAIT_END, quiet=True)
        out["c3c/%d" % at] = rec
        r = rec["r"]
        lines.append("  C3C: fault at offset 0x%x: first handler entry C3C %08x (PC read there %08x), second entry C3C "
                     "%08x; %s r6 %08x" % (r[2], r[1], r[3], r[4], rec["why"], r[6]))
    soft_reset(s, t)
    return out, "unused control words (value written: C10 / C48 / C54) and C3C in handlers:\n" + "\n".join(lines)


@experiment("c08self", needs=("nand_seq",))
def ex_c08self(s, t):
    """CS_STATUS as the program reads it (ld 0xc08 and pld) in places the CPU cannot sample: right after a DMA, an ECC
    kick and a chip command are started, inside a fault handler, after sig, after a wait2 33, after a CPU pause and
    continue (C08 read after the pause point)"""
    a, e = t.scratch_addr(s), t.events
    rd = "    ld   r%d, [0xc08]\n"
    cases = [("plain", rd % 1),
             ("after DMA start", "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
              "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n" % a + rd % 1 + "    wait %d\n" % e["dma"] + rd % 2),
             ("after ECC kick", "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1102\n" +
              rd % 1 + "    wait %d\n" % e["ecc"] + rd % 2),
             ("after a chip command", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n" + rd % 1 +
              "    wait %d\n" % e["cmd"] + rd % 2),
             ("after sig", "    sig\n" + rd % 1),
             ("after wait2 33", "    wait2 33\n" + rd % 1),
             ("pld 0xc08", "    movi r4, #0xc08\n    pld  r1, [r4]\n"),
             ("in a fault handler", "    wr   [FMSS_C58], #0x4\n    wr   [FMSS_C4C], #hnd\n    wait 10\n" + END +
              "hnd:\n" + rd % 1 + "    ld   r2, [0xc0c]\n")]
    out, lines = {}, []
    for label, body in cases:
        src = t.prologue(c58=0) + t.state_setup() + body + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xc1c, 0xeeee), (0xc20, 0xeeee)], timeout_us=40000,
                    flags=WAIT_END, scratch=bytes(512), quiet=True)
        out[label] = rec
        lines.append("  %-22s C08 %x, then %x; %s" % (label, rec["r"][1], rec["r"][2], rec["why"]))
    soft_reset(s, t)
    return out, "CS_STATUS read by the program (r1, then r2 after the wait; eeee = not read):\n" + "\n".join(lines)


@experiment("latchrep", needs=("nand_seq",))
def ex_latchrep(s, t):
    """The intermittent results: wait event 5 straight after the 70h latch, the status latch's wait 5 (once timed out
    in two identical runs), FMC_STATUS bit 24 instead of bit 23, and wait event 3 on CE0. Each 40 times, with and
    without a fresh arm (FMCTRL1 0x20 before the latch), with and without 0x044 / 0x844 written; tallies of pass / time
    out and of the FMC_STATUS bits after the latch"""
    e = t.events
    arm = "    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x1\n"
    latch = t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait %d\n    ld   r3, [FMC_STATUS]\n    ld   r4, [FMC_NSTAT]\n" % e["status"]
    cases = [("fresh arm", arm + latch, []), ("no fresh arm", latch, []),
             ("fresh arm, 0x044 / 0x844 written", arm + latch, [(0x044, 0xa5a5), (0x844, 0x25)]),
             ("no fresh arm, 0x044 / 0x844 written", latch, [(0x044, 0xa5a5), (0x844, 0x25)]),
             ("two latches, no arm between", arm + latch + "    wr   [FMC_STATUS], #0x800000\n" + latch, []),
             ("wait 3 on CE0 (16-byte read)", t.cmd(0x00) + "    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x10\n"
              "    wr   [FMC_24], #0x0\n    wr   [FMCTRL1], #0x32\n    wait 3\n    ld   r3, [FMC_STATUS]\n", [])]
    out, lines = {}, []
    for label, body, params in cases:
        tally = {}
        for k in range(40):
            src = t.nand_prologue(0) + body + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), params=params + [(0xc24, 0)], timeout_us=20000, flags=WAIT_END, quiet=True)
            out["%s/%d" % (label, k)] = rec
            key = "%s b23 %d b24 %d" % ("timeout" if rec["fault_at_us"] else "pass", rec["r"][3] >> 23 & 1,
                                         rec["r"][3] >> 24 & 1)
            tally[key] = tally.get(key, 0) + 1
            if params:
                probe(s, t, assemble(t, "    end\n"), params=[(0x044, 0), (0x844, 0)], timeout_us=20000, quiet=True)
        lines.append("  %-38s %s" % (label, ", ".join("%s x%d" % kv for kv in sorted(tally.items()))))
    soft_reset(s, t)
    return out, "status latch and wait 3, 40 runs each:\n" + "\n".join(lines)


@experiment("irq044", needs=("nand_seq",))
def ex_irq044(s, t):
    """0x044 bit by bit (0..31), three runs each after a soft reset, with one program that does a command, an
    address, a 16-byte data read, a status latch and a DMA: which bits raise CS_IRQ bit 4 / bit 5, VIC0 line 20 and
    VIC1 line 22, and is it stable?"""
    a, e = t.scratch_addr(s), t.events
    body = t.cmd(0x70) + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) + \
        "    wr   [FMCTRL1], #0xca\n    wait %d\n    wr   [FMC_STATUS], #0x800000\n    wr   [FMCTRL1], #0x20\n" % e["status"] + \
        t.cmd(0x00) + _address(t, t.scratch_row) + t.cmd(0x30) + _ready(t, 0) + "    wr   [FMCTRL1], #0x100e0\n" + \
        t.cmd(0x00) + "    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x10\n    wr   [FMC_24], #0x0\n" \
        "    wr   [FMCTRL1], #0x32\n    wait 3\n    wr   [FMC_STATUS], #0x8\n    wr   [FMC_CTRL0], #@CE_DMA@\n" + \
        _dma_in(0x1, a, e["dma"])
    code = assemble(t, t.nand_prologue(0) + body + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap())
    out, lines = {}, []
    for bit in [None] + list(range(32)):
        row = []
        for k in range(3):
            soft_reset(s, t)
            rec = probe(s, t, code, params=[(0x044, 0 if bit is None else 1 << bit)], timeout_us=40000, flags=WAIT_END,
                        scratch=bytes(512), quiet=True)
            out["%s/%d" % (bit, k)] = rec
            row.append("CS_IRQ %02x V0.20 %d V1.22 %d %s" % (rec["cs_irq"], rec["vic0_seen"] >> 20 & 1,
                                                          rec["vic1_seen"] >> 22 & 1, rec["why"]))
        lines.append("  %-8s %s" % ("none" if bit is None else "bit %d" % bit, " | ".join(row)))
    soft_reset(s, t)
    return out, "0x044 one bit at a time (three runs):\n" + "\n".join(lines)


def _erase(t, row):
    """60h, two row cycles, D0h, ready poll (NSTAT in r7)"""
    return t.cmd(0x60) + "    wr   [FMC_ANUM], #0x2\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMCTRL1], #0x1\n" \
        "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % (row, t.events["addr"]) + t.cmd(0xd0) + _ready(t, "e")


def _program_page(t, a, row, ce, nsec=4, raw=False, mode=0x180):
    """80h, address, nsec sectors from scratch a + 0x200 k (engine encode unless raw), 10h, ready poll (NSTAT in r7)"""
    e = t.events
    src = t.nand_prologue(ce) + t.cmd(0x80) + _address(t, row) + "    wr   [FMC_CTRL0], #0x%x\n" % (t.ce_sel[ce] | t.ce_dma)
    for k in range(nsec):
        src += _dma_in(0x1, a + 0x200 * k, e["dma"])
        if not raw:
            src += _kick(0x1102 | k << 16, mode, e["ecc"]) + _send(0x1000, 0xf, 0x34, "    wr   [FMC_28], #0x%x\n" % (16 * k))
        src += _send(0x100, 0x1ff, 0xe4)
    return src + "    wr   [FMC_CTRL0], #0x%x\n" % t.ce_sel[ce] + t.cmd(0x10) + _ready(t, 0) + \
        "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()


def _read_page(t, a, row, ctrl, dsel=0x100, dmactl=None, mode=0x180, ce=0):
    """page read of CE0 with FMC_CTRL0 = ctrl (| bit 24 for the DMAs): per sector chunk and data DMA-ed out, decode,
    ECC_RESULT to 0xd70 + 4k; dmactl: FMC_DMA_CTRL written before the last DMA"""
    e = t.events
    src = t.nand_prologue(ce) + "    wr   [FMC_CTRL0], #0x%x\n" % ctrl + t.cmd(0x00) + _address(t, row) + \
        "    wr   [FMC_CMD], #0x30\n    wait %d\n    wr   [FMC_STATUS], #0x2\n" % e["cmd"] + _ready(t, 0) + \
        "    wr   [FMCTRL1], #0x100e0\n" + t.cmd(0x00)
    for k in range(4):
        src += "    wr   [FMC_CTRL0], #0x%x\n" % ctrl + _send(0x10, 0xf, 0x32, "    wr   [FMC_24], #0x%x\n" % (16 * k)) + \
            _send(0x1, 0x1ff, 0xe2) + "    wr   [FMC_CTRL0], #0x%x\n" % (ctrl | t.ce_dma) + \
            _dma_out(0x1000, a + 0x800 + 0x200 * k, e["dma"]) + _kick(0x1101 | k << 16, mode, e["ecc"]) + \
            "    ld   r2, [ECC_RESULT]\n    st   [0x%x], r2\n" % (0xd70 + 4 * k)
        if dmactl is not None and k == 3:
            src += "    wr   [FMC_DMA_CTRL], #0x%x\n" % dmactl
        src += _dma_out(dsel, a + 0x200 * k, e["dma"])
    return src + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()


@experiment("fmcbits", needs=("nand_seq",))
def ex_fmcbits(s, t):
    """DESTRUCTIVE (scratch block + 50). A page written by the engine is read back with FMC_CTRL0 bit 9 set, bit 11
    clear, bit 24 (DMA) set for the whole read, FMC_DMA_CTRL = 0 / 7 / 0xff before the last DMA, and the drain select
    0x102 / 0x1e0 instead of 0x100; READID with bit 9 and without bit 11. Data, decode results, time"""
    a = t.scratch_addr(s)
    row = t.scratch_row + 50 * t.rows_per_block
    data = b"".join(bytes((i * (11 + 2 * k) + k) & 0xff for i in range(512)) for k in range(4))
    out, lines = {}, []
    rec = probe(s, t, assemble(t, t.nand_prologue(0) + _erase(t, row) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" +
                               END + t.trap()), timeout_us=60000, flags=WAIT_END, quiet=True)
    out["erase"] = rec
    rec = probe(s, t, assemble(t, _program_page(t, a, row, 0)), timeout_us=60000, flags=WAIT_END, scratch=data, quiet=True)
    out["program"] = rec
    lines.append("  erase + program: %s, NSTAT %02x" % (rec["why"], rec["r"][7]))
    base = t.ce_sel[0]
    cases = [("base 0x%x" % base, base, 0x100, None), ("bit 9 set", base | 0x200, 0x100, None),
             ("bit 11 clear", base & ~0x800, 0x100, None), ("bit 24 set throughout", base | t.ce_dma, 0x100, None),
             ("FMC_DMA_CTRL 0", base, 0x100, 0), ("FMC_DMA_CTRL 7", base, 0x100, 7), ("FMC_DMA_CTRL 0xff", base, 0x100, 0xff),
             ("drain 0x102", base, 0x102, None), ("drain 0x1e0", base, 0x1e0, None), ("drain 0x1e0 + DMA_CTRL 7", base, 0x1e0, 7)]
    for label, ctrl, dsel, dmactl in cases:
        rec = probe(s, t, assemble(t, _read_page(t, a, row, ctrl, dsel, dmactl)), timeout_us=80000, flags=WAIT_END,
                    scratch=b"\xee" * 0x1000, scratch_read=0x1000, quiet=True)
        out[label] = rec
        sc = rec["scratch"] or bytes(0x1000)
        ok = [sc[0x200 * k:0x200 * k + 0x200] == data[0x200 * k:0x200 * k + 0x200] for k in range(4)]
        lines.append("  %-28s %s %5dus data %s results %s FMC_DMA_CTRL after %08x" % (
            label, rec["why"], rec["elapsed_us"], "".join("=" if x else "x" for x in ok),
            " ".join("%08x" % rec["after"][0xd70 + 4 * k] for k in range(4)), rec["after"][0x038]))
    for label, word in (("READID base", base), ("READID bit 9", base | 0x200), ("READID no bit 11", base & ~0x800),
                        ("READID bit 9, no bit 11", (base | 0x200) & ~0x800)):
        src = t.prologue(c58=0) + t.read_id(word, 0) + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, quiet=True)
        out[label] = rec
        lines.append("  %-28s %s ID %08x %08x status %08x" % (label, rec["why"], rec["r"][2], rec["r"][3], rec["r"][4]))
    soft_reset(s, t)
    return out, "FMC_CTRL0 bits, FMC_DMA_CTRL and drain selects on a page read:\n" + "\n".join(lines)


@experiment("fmcstat", needs=("nand_seq", "fullmap"))
def ex_fmcstat(s, t):
    """FMC_STATUS power-on bits 12, 13, 18, 19: each acknowledged alone (write 1), then which operation sets it again
    (a command, an address, a data read, a status latch, a DMA, an ECC encode, a meta encode, a second-set command);
    and the value right after a soft reset"""
    a, e = t.scratch_addr(s), t.events
    ops = [("nothing", ""), ("command", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"]),
           ("address", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n"
            "    wr   [FMCTRL1], #0x1\n    wait %d\n" % e["addr"]),
           ("data read", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x10\n"
            "    wr   [FMCTRL1], #0x32\n    wait 3\n"),
           ("status latch", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) +
            "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"]),
           ("DMA", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"])),
           ("ECC encode", _kick(0x1102, 0x180, e["ecc"])),
           ("meta encode", "    wr   [FMC_METACFG], #0x3210\n    wr   [FMC_METACTL], #0x1\nmp:\n    ld   r1, [FMC_METACTL], 0x1\n"
            "    bnz  r1, mp\n"),
           ("second set command", "    wr   [0x400], #@CE0@\n    wr   [0x408], #0x70\n    wait %d\n" % e["set1_cmd"])]
    out, lines = {}, []
    soft_reset(s, t)
    rec = probe(s, t, assemble(t, "    end\n"), timeout_us=20000, quiet=True)
    lines.append("  after a soft reset: FMC_STATUS %08x, 0x448 %08x" % (rec["before"][0x048], rec["after"][0x448]))
    for bit in (12, 13, 18, 19):
        row = []
        for label, body in ops:
            soft_reset(s, t)
            src = t.prologue(c58=0) + "    ld   r1, [FMC_STATUS]\n    wr   [FMC_STATUS], #0x%x\n    ld   r2, [FMC_STATUS]\n" \
                % (1 << bit) + body + "    ld   r3, [FMC_STATUS]\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(512), quiet=True)
            out["%d/%s" % (bit, label)] = rec
            r = rec["r"]
            row.append("%s %d>%d>%d" % (label, r[1] >> bit & 1, r[2] >> bit & 1, r[3] >> bit & 1))
        lines.append("  bit %d (start > after w1c > after the operation): %s" % (bit, ", ".join(row)))
    soft_reset(s, t)
    return out, "FMC_STATUS power-on bits:\n" + "\n".join(lines)


@experiment("eccregs", needs=("nand_seq", "fullmap"))
def ex_eccregs(s, t):
    """ECC words without a known effect: 0x940..0x950 written 0 / all ones / 0x55555555 before an encode and a
    decode with 8 bit errors (parity and decode result compared), and 0x990..0x99c after encodes of zeros, ones and a
    ramp in the BCH, Reed-Solomon and no-bit-8 modes"""
    a, e = t.scratch_addr(s), t.events
    ramp = bytes((i * 37 + 11) & 0xff for i in range(512))
    bad = bytearray(ramp)
    for j in range(8):
        bad[j * 61] ^= 1 << (j % 8)
    out, lines = {}, []
    for val in (0, 0xffffffff, 0x55555555):
        pre = "".join("    wr   [0x%x], #0x%x\n" % (o, val) for o in range(0x940, 0x954, 4))
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + pre + _dma_in(0x1, a, e["dma"]) + \
            _kick(0x1102, 0x180, e["ecc"]) + _dma_out(0x1000, a + 0x600, e["dma"]) + _dma_in(0x1, a + 0x200, e["dma"]) + \
            _kick(0x1101, 0x180, e["ecc"]) + "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + \
            _dma_out(0x100, a + 0x400, e["dma"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                    scratch=ramp + bytes(bad) + b"\xee" * 0x400, scratch_read=0x800, quiet=True)
        out["94x/%08x" % val] = rec
        sc = rec["scratch"] or bytes(0x800)
        fm = rec.get("fullmap") or [[0] * 1024] * 2
        lines.append("  0x940..0x950 = %08x: parity %s, decode %08x / %02x, data %s, 0x940.. after %s" % (
            val, sc[0x600:0x610].hex(), rec["r"][2], rec["r"][3], "corrected" if sc[0x400:0x600] == ramp else "other",
            " ".join("%08x" % fm[1][o // 4] for o in range(0x940, 0x954, 4))))
    for mode in (0x180, 0x8100, 0x8108, 0x0, 0x80):
        for lab, d in (("zeros", bytes(512)), ("ones", b"\xff" * 512), ("ramp", ramp)):
            src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
                _kick(0x1102, mode, e["ecc"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP, scratch=d, quiet=True)
            out["99x/%x/%s" % (mode, lab)] = rec
            fm = rec.get("fullmap") or [[0] * 1024] * 2
            lines.append("  mode 0x%04x %-5s 0x980..0x99c %s" % (mode, lab, " ".join("%08x" % fm[1][o // 4]
                                                                                    for o in range(0x980, 0x9a0, 4))))
    soft_reset(s, t)
    return out, "ECC words 0x940..0x950 and 0x980..0x99c:\n" + "\n".join(lines)


@experiment("seqrestart", needs=("nand_seq", "fullmap"))
def ex_seqrestart(s, t):
    """The sequence of the modes without bit 8 restarted once and continued once between runs. Three encodes of the
    same sector in mode 0 per run, with one action between the runs: nothing, a soft reset, an ECC_MODE write, an
    ECC_KICK reset word, a BCH encode, a BCH decode, an ECC_DONE write, a write of 0x940..0x950, an FMC_CTRL0 write, the
    sequencer reset (0x30000); area words 0x900..0x913 of every encode"""
    a, e = t.scratch_addr(s), t.events
    ramp = bytes((i * 37 + 11) & 0xff for i in range(512))
    acts = [("nothing", ""), ("soft reset", None), ("ECC_MODE write", "    wr   [ECC_MODE], #0x180\n"),
            ("ECC_KICK 0", "    wr   [ECC_KICK], #0x0\n"), ("BCH encode", _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x180, e["ecc"])),
            ("BCH decode", _dma_in(0x1, a, e["dma"]) + _kick(0x1101, 0x180, e["ecc"])),
            ("ECC_DONE write", "    wr   [ECC_DONE], #0x1ff\n"),
            ("0x940..0x950 write", "".join("    wr   [0x%x], #0x0\n" % o for o in range(0x940, 0x954, 4))),
            ("FMC_CTRL0 write", "    wr   [FMC_CTRL0], #@CE_NONE@\n"), ("sequencer reset", "RESET")]
    out, lines = {}, []

    def run():
        body = ""
        for k in range(3):
            body += _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x0, e["ecc"]) + "".join(
                "    ld   r1, [0x%x]\n    movi r2, #0x%x\n    sti  [r2], r1\n" % (0x900 + 4 * j, a + 0x200 + 0x20 * k + 4 * j)
                for j in range(5))
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + body + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=ramp + b"\xee" * 0x60,
                    scratch_read=0x260, quiet=True)
        sc = rec["scratch"] or bytes(0x260)
        return rec, [rec["why"]] + [sc[0x200 + 0x20 * k:0x214 + 0x20 * k].hex() for k in range(3)]
    soft_reset(s, t)
    for label, act in acts:
        r1, v1 = run()
        if act is None:
            soft_reset(s, t)
        elif act == "RESET":
            probe(s, t, assemble(t, "    end\n"), params=[(0xc00, 0x30000)], timeout_us=20000, quiet=True)
        elif act:
            probe(s, t, assemble(t, t.prologue(c58=0) + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + act +
                                 "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()), timeout_us=20000,
                  flags=WAIT_END, scratch=ramp, quiet=True)
        r2, v2 = run()
        out[label] = [r1, r2]
        lines.append("  between: %-20s run 1 %s | run 2 %s" % (label, " ".join(v1), " ".join(v2)))
    soft_reset(s, t)
    return out, "mode 0 encodes, area 0x900..0x913 (three per run):\n" + "\n".join(lines)


@experiment("done5", needs=("nand_seq",))
def ex_done5(s, t):
    """ECC_DONE after an encode and after a decode for ECC_MODE bits 9..14 and 16..24 one at a time, on the BCH mode
    0x180, the Reed-Solomon mode 0x8100 and the no-bit-8 mode 0x0: the bit 5 rule is fitted to a few modes"""
    a, e = t.scratch_addr(s), t.events
    ramp = bytes((i * 37 + 11) & 0xff for i in range(512))
    out, lines = {}, []
    for base in (0x180, 0x8100, 0x0):
        row = []
        for bit in [None] + list(range(9, 15)) + list(range(16, 25)):
            mode = base | (0 if bit is None else 1 << bit)
            src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
                _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n" + _kick(0x1101, mode, e["ecc"]) + \
                "    ld   r5, [ECC_DONE]\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0xc28, 0xee), (0xc2c, 0xee)], timeout_us=40000,
                        flags=WAIT_END, scratch=ramp, quiet=True)
            out["%x" % mode] = rec
            row.append("%s %02x/%02x%s" % ("-" if bit is None else "b%d" % bit, rec["r"][4], rec["r"][5],
                                           "" if rec["why"] == "END" else " " + rec["why"]))
        lines.append("  base 0x%04x: %s" % (base, ", ".join(row)))
    soft_reset(s, t)
    return out, "ECC_DONE after encode / decode (ee = the wait timed out before it):\n" + "\n".join(lines)


@experiment("progtime", needs=("nand_seq",))
def ex_progtime(s, t):
    """DESTRUCTIVE (scratch blocks + 60, + 61 on CE0, + 60 on CE1). Every page of a block programmed in order with
    four sectors of a pattern (engine encode), one run per page: run time (the ready poll included) per page; are
    pages 4..11 slower, and what is the pattern over 0..127?"""
    a = t.scratch_addr(s)
    rpb = t.rows_per_block
    data = b"".join(bytes((i * (5 + 2 * k) + 3) & 0xff for i in range(512)) for k in range(4))
    out, lines = {}, []
    for ce, blk in ((0, 60), (0, 61), (1, 60)):
        row0 = t.scratch_row + blk * rpb
        rec = probe(s, t, assemble(t, t.nand_prologue(ce) + _erase(t, row0) +
                                   "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()),
                    timeout_us=60000, flags=WAIT_END, quiet=True)
        out["erase %d/%d" % (ce, blk)] = rec
        times = []
        for p in range(rpb):
            rec = probe(s, t, assemble(t, _program_page(t, a, row0 + p, ce)), timeout_us=60000, flags=WAIT_END,
                        scratch=data, quiet=True)
            out["%d/%d/%d" % (ce, blk, p)] = rec
            times.append(rec["elapsed_us"] if rec["why"] == "END" and rec["r"][7] & 1 == 0 else -rec["elapsed_us"])
        lines.append("  CE%d block +%d (us per page, negative = failed):" % (ce, blk))
        for p in range(0, rpb, 16):
            lines.append("    %3d: %s" % (p, " ".join("%5d" % x for x in times[p:p + 16])))
        print("\n".join(lines[-9:]))
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "page program time per page (one run per page, the same pattern):\n" + "\n".join(lines)


REGION_BLOCKS = (0, 1, 2, 100, 2000, 3000, 3871, 3872, 3873, 3900, 4000, 4064, 4094, 4095)   # user area 0..3871


@experiment("startregion", needs=("nand_seq", "start_value"))
def ex_startregion(s, t):
    """Start-word bit 6 and the parts of the NAND. On the 4G / 5G the bootloader starts with 0xffb5 and reads only its
    own blocks, the disk OS starts with 0xfff5 and works the user area: every 0xfff5 / 0xffb5 comparison so far ran on
    the scratch block (user area). Page 0 of blocks in the user area (0..3871 of this chip: 3872 user blocks of 4096)
    and in the blocks above it, on CE0 and CE1, read under 0xfff5, 0xffb5, 0xffd5 and 0xff95 (the second pair also
    differs in bit 6 alone); per read: end, time, the four decode results and FMC_STATUS words, a hash of the data and
    chunks. Then DESTRUCTIVE: in a user block (scratch + 2) and in the first block above 3871 whose page 0 read erased,
    an erase + program under each start word in turn, each read back under both"""
    import hashlib
    from .followup import _raw_read
    a = t.scratch_addr(s)
    rpb = t.rows_per_block
    words = (0xfff5, 0xffb5, 0xffd5, 0xff95)
    out, lines, erased = {}, [], {}
    for ce in (0, 1):
        for blk in REGION_BLOCKS:
            res = []
            for w in words:
                rec, secs = _raw_read(t, s, blk * rpb, ce, start_value=w)
                out["read ce%d/%d/%04x" % (ce, blk, w)] = rec
                h = hashlib.sha256(b"".join(d + ch for d, ch, _, _, _ in secs)).hexdigest()[:10]
                blank = all(d == b"\xff" * 512 and ch == b"\xff" * 16 for d, ch, _, _, _ in secs)
                res.append((rec["why"], rec["elapsed_us"], tuple(x[3] for x in secs), tuple(x[4] for x in secs),
                            "erased" if blank else h))
            if res[0][4] == "erased" and blk >= 3872:
                erased.setdefault(ce, blk)
            same = all(r[0] == res[0][0] and r[2:] == res[0][2:] and abs(r[1] - res[0][1]) < 30 for r in res)
            lines.append("  CE%d block %4d (%s): %s" % (ce, blk, "user" if blk < 3872 else "above user", (
                "same under all four: %s %dus data %s res %s st %s" % (res[0][0], res[0][1], res[0][4],
                                                                         " ".join("%x" % x for x in res[0][2]),
                                                                         " ".join("%x" % x for x in res[0][3])))
                if same else " | ".join("%04x: %s %dus data %s res %s st %s" % (
                    w, r[0], r[1], r[4], " ".join("%x" % x for x in r[2]), " ".join("%x" % x for x in r[3]))
                    for w, r in zip(words, res))))
        print("\n".join(lines[-len(REGION_BLOCKS):]))
        sys.stdout.flush()
    data = b"".join(bytes((i * (13 + 2 * k) + 5 * k) & 0xff for i in range(512)) for k in range(4))
    targets = [("user block %d" % (t.scratch_block + 2), 0, t.scratch_block + 2)]
    targets += [("block %d (above user, read erased)" % b, ce, b) for ce, b in sorted(erased.items())]
    if not erased:
        lines.append("  no block above 3871 read erased: no write there")
    for label, ce, blk in targets:
        row = blk * rpb
        for w in (0xfff5, 0xffb5):
            src = t.nand_prologue(ce) + _erase(t, row) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            er = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END, start_value=w, quiet=True)
            pr = probe(s, t, assemble(t, _program_page(t, a, row, ce)), timeout_us=60000, flags=WAIT_END,
                       start_value=w, scratch=data, quiet=True)
            out["write %s/%04x" % (label, w)] = [er, pr]
            back = []
            for w2 in (0xfff5, 0xffb5):
                rec, secs = _raw_read(t, s, row, ce, start_value=w2)
                ok = "".join("=" if secs[k][2] == data[0x200 * k:0x200 * k + 0x200] else "x" for k in range(4))
                back.append("%04x %s data %s res %s" % (w2, rec["why"], ok, " ".join("%x" % x[3] for x in secs)))
            lines.append("  CE%d %s, written under %04x: erase %s %dus NSTAT %02x, program %s %dus NSTAT %02x; read %s" % (
                ce, label, w, er["why"], er["elapsed_us"], er["r"][7], pr["why"], pr["elapsed_us"], pr["r"][7],
                " | ".join(back)))
    soft_reset(s, t)
    return out, "start words and NAND regions (page 0 of each block):\n" + "\n".join(lines)


@experiment("bufport", needs=("nand_seq",))
def ex_bufport(s, t):
    """Is FMC_FIFO (0x080) a port onto the data buffers? The original READID fills data buffer 0 from the chip
    (FMC_DATA_SEL 1, FMCTRL1 0x1c2) and reads its 8 bytes through 0x080 after FMC_DATA_SEL 0x100 and FMCTRL1 0x340.
    Here a buffer is filled by DMA with a known pattern and read through 0x080 130 times (one word more than 512 bytes),
    FMC_28 read every 32 words: data buffer 0 (0x100), data buffer 1 (0x200), parity buffer 0 (0x1000), without
    FMCTRL1 0x340, and read twice in a row (does the position restart?). Then 0x080 written 128 times with FMC_DATA_SEL 1
    (plain and after FMCTRL1 0x340) and the buffer DMA-ed out: can a program write a data buffer through the port?"""
    a, e = t.scratch_addr(s), t.events
    pat = bytes((i * 29 + 7) & 0xff for i in range(512))
    out, lines = {}, []

    def fill(sel):
        return "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(sel, a, e["dma"])

    def readport(n, base, mark=True):
        src = "    movi r2, #0x%x\n    movi r4, #0x%x\n" % (a + base, a + base + 0x400)
        for k in range(n):
            src += "    ld   r1, [FMC_FIFO]\n    sti  [r2], r1\n    add  r2, r2, #0x4\n"
            if mark and k % 32 == 0:
                src += "    ld   r1, [FMC_28]\n    sti  [r4], r1\n    add  r4, r4, #0x4\n"
        return src
    cases = [("data buffer 0, 0x340", 0x1, "    wr   [FMC_DATA_SEL], #0x100\n    wr   [FMCTRL1], #0x340\n", 130),
             ("data buffer 1, 0x340", 0x2, "    wr   [FMC_DATA_SEL], #0x200\n    wr   [FMCTRL1], #0x340\n", 130),
             ("data buffer 0, no 0x340", 0x1, "    wr   [FMC_DATA_SEL], #0x100\n", 130),
             ("parity buffer 0, 0x340", 0x10, "    wr   [FMC_DATA_SEL], #0x1000\n    wr   [FMCTRL1], #0x340\n", 20)]
    for label, sel, mode, n in cases:
        for twice in (False, True):
            src = t.prologue(c58=0) + t.state_setup() + fill(sel) + mode + readport(n, 0x200)
            if twice:
                src += mode + readport(n, 0x800, mark=False)
            src += "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END,
                        scratch=pat + b"\xee" * 0xe00, scratch_read=0x1000, quiet=True)
            sc = rec["scratch"] or bytes(0x1000)
            got = sc[0x200:0x200 + 4 * n]
            ref = pat if sel < 0x10 else pat[448:512]
            match = next((k for k in range(0, len(ref), 4) if got[:16] == ref[k:k + 16]), None)
            same = sum(1 for k in range(0, min(len(got), len(ref) - (match or 0)), 4)
                       if match is not None and got[k:k + 4] == ref[(match or 0) + k:(match or 0) + k + 4])
            f28 = [struct.unpack_from("<I", sc, 0x600 + 4 * k)[0] & 0x1ff for k in range((n + 31) // 32)]
            line = "  %-26s %s %s: first words %s; matches the buffer from byte %s for %d words; FMC_28 %s" % (
                label, "twice" if twice else "once ", rec["why"], got[:16].hex(), match, same, f28)
            if twice:
                g2 = sc[0x800:0x800 + 4 * n]
                line += "; second read first words %s (%s)" % (g2[:16].hex(), "restarts" if g2[:16] == got[:16] else
                                                             "continues" if g2[:16] == ref[(4 * n) % len(ref):(4 * n) % len(ref) + 16] else "other")
            out["%s/%s" % (label, twice)] = rec
            lines.append(line)
    # writes through the port
    for label, pre in (("write 0x080, FMC_DATA_SEL 1", "    wr   [FMC_DATA_SEL], #0x1\n"),
                       ("write 0x080, FMC_DATA_SEL 1, FMCTRL1 0x340", "    wr   [FMC_DATA_SEL], #0x1\n    wr   [FMCTRL1], #0x340\n")):
        src = t.prologue(c58=0) + t.state_setup() + pre + "    movi r1, #0x5a000000\n"
        for k in range(128):
            src += "    st   [FMC_FIFO], r1\n    add  r1, r1, #0x1\n"
        src += "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_out(0x100, a + 0x200, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=b"\xee" * 0x400,
                    scratch_read=0x400, quiet=True)
        sc = rec["scratch"] or bytes(0x400)
        words = struct.unpack_from("<128I", sc, 0x200)
        n5a = sum(1 for w in words if w >> 24 == 0x5a)
        out[label] = rec
        lines.append("  %-42s %s: buffer after: first words %s; %d of 128 words carry the written marker" % (
            label, rec["why"], sc[0x200:0x210].hex(), n5a))
    soft_reset(s, t)
    return out, "FMC_FIFO (0x080) as a port onto the buffers (pattern i*29+7):\n" + "\n".join(lines)
