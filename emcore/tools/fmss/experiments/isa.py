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

"""Instruction set questions: ALU, memory, indirect access, branches, register numbers and opcode timing."""
import struct
import re
import sys

from ..probe import (ToolError, probe, assemble, lint, answer, changes, ctrl_changes, where, F_SAMPLE_REGS, WAIT_END,
                     PROG_SRAM)
from .registry import RESULTS, experiment
from .sweeps import SWEEP_FORMS


ALU_CASES = [
    # (label, setup, instruction, what each answer means)
    ("shr 0x80000000 by 1", "movi r1, #0x80000000", "shr  r2, r1, #0x1", {0x40000000: "logical", 0xc0000000: "arithmetic"}),
    ("shl 1 by 32 (imm)", "movi r1, #0x1", "shl  r2, r1, #0x20", {0: "shift >= 32 gives 0", 1: "count masked to 5 bits"}),
    ("shl 1 by 33 (imm)", "movi r1, #0x1", "shl  r2, r1, #0x21", {0: "gives 0", 2: "count masked to 5 bits"}),
    ("shr 0x80000000 by 32", "movi r1, #0x80000000", "shr  r2, r1, #0x20", {0: "gives 0", 0x80000000: "masked"}),
    ("add wraps", "movi r1, #0xffffffff", "add  r2, r1, #0x2", {1: "32-bit wrap"}),
    ("sub below 0", "movi r1, #0x0", "sub  r2, r1, #0x1", {0xffffffff: "32-bit wrap"}),
    ("add imm 0 is the 2-operand form", "movi r1, #0x5\n    movi r2, #0x7", ".insn 0x0c, 2, 1, 0", {12: "r2 = r2 + r1 (2-operand)", 5: "r2 = r1 + 0"}),
    ("shl by a register >= 32", "movi r2, #0x1\n    movi r1, #0x21", ".insn 0x13, 2, 1, 0", {0: "gives 0", 2: "count masked to 5 bits"}),
    ("ld mask", "wr [0xd60], #0x12345678", "ld   r2, [0xd60], 0xff00ff00", {0x12005600: "AND with the mask"}),
]


@experiment("regmap")
def ex_regmap(s, t):
    """Are r0..r7 the CPU-visible words 0xc18 + 4n, and is SEQ_RESULT (0xc30) r6?"""
    vals = [0x11110000 + 0x1111 * i for i in range(8)]
    src = t.prologue() + "".join("    movi r%d, #0x%08x\n" % (i, v) for i, v in enumerate(vals)) + "    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src))
    if answer(rec):
        return rec, answer(rec)
    ok = rec["r"] == vals
    return rec, "%s: r0..r7 at 0xc18..0xc34 %s; SEQ_RESULT %08x %s r6" % (
        rec["why"], "match" if ok else "DIFFER (%s)" % " ".join("%08x" % v for v in rec["r"]),
        rec["seq_result"], "==" if rec["seq_result"] == vals[6] else "!=")


@experiment("clock")
def ex_clock(s, t):
    """How fast does the sequencer execute? (a two-instruction loop at two lengths)"""
    tm = {}
    recs = {}
    for n in (2000, 20000):
        src = t.prologue() + "    movi r0, #%d\nloop:\n    sub  r0, r0, #0x1\n    bnz  r0, loop\n    end\n" % n + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=2000000)
        if answer(rec):
            return rec, answer(rec)
        recs[n], tm[n] = rec, rec["elapsed_us"]
    d = tm[20000] - tm[2000]
    ns = 1000.0 * d / (2 * 18000) if d > 0 else 0
    return recs, "loop of 2 instructions: %d us for 2000, %d us for 20000 iterations -> %.1f ns per instruction (~%.0f MHz if 1 per clock)" % (
        tm[2000], tm[20000], ns, 1000.0 / ns if ns else 0)


@experiment("alu")
def ex_alu(s, t):
    """ALU edge cases: shift kind and range, wrap, the imm == 0 form, ld mask"""
    out, lines = {}, []
    for label, setup, ins, meaning in ALU_CASES:
        src = t.prologue() + "    movi r2, #0xdeadbeef\n    %s\n    %s\n    end\n" % (setup, ins) + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xd60, 0)])
        v = rec["r"][2]
        out[label] = rec
        lines.append("  %-34s -> %s" % (label, answer(rec) or "%08x  %s" % (v, meaning.get(v, "UNEXPECTED"))))
    return out, "ALU:\n" + "\n".join(lines)


@experiment("memory")
def ex_memory(s, t):
    """ldi / sti on RAM (the scratch area): word access, byte order, an unaligned address"""
    base = t.scratch_addr(s)
    pat = bytes(range(256))
    src = t.prologue() + """    movi r1, #0x%08x
    ldi  r2, [r1]
    add  r3, r1, #0x2
    ldi  r4, [r3]
    movi r5, #0xa1b2c3d4
    add  r7, r1, #0x40
    sti  [r7], r5
    add  r7, r1, #0x81
    sti  [r7], r5
    end
""" % base + t.trap()
    rec = probe(s, t, assemble(t, src), scratch=pat)
    if answer(rec):
        return rec, "memory: " + answer(rec)
    sc = rec["scratch"] or b""
    r = rec["r"]
    msg = ["%s" % rec["why"],
           "ldi [base] = %08x (expect 03020100 little endian)" % r[2],
           "ldi [base+2] = %08x (03020100 if the address is word-aligned down, 05040302 if unaligned access works)" % r[4]]
    if sc:
        msg.append("sti [base+0x40] -> bytes %s (expect d4c3b2a1)" % sc[0x40:0x44].hex())
        msg.append("sti [base+0x81] -> bytes 0x80..0x87 %s (aligned down: d4c3b2a1 at 0x80)" % sc[0x80:0x88].hex())
    return rec, "memory: " + "; ".join(msg)


@experiment("indirect", needs=("pld_pst",))
def ex_indirect(s, t):
    """pld / pst: which offsets do they reach? (register file, parameter RAM, beyond 0xfff)"""
    out, lines = {}, []
    cases = [("pld 0xc18 (r0 through the window)", "movi r0, #0x5a5a0000\n    movi r1, #0xc18", "pld  r2, [r1]", 2),
             ("pld 0xd64 (parameter RAM)", "movi r1, #0xd64", "pld  r2, [r1]", 2),
             ("pld 0x1d64 (above the window)", "movi r1, #0x1d64", "pld  r2, [r1]", 2),
             ("pst 0xd68 then ld", "movi r1, #0xd68\n    movi r3, #0xc0ffee", "pst  [r1], r3\n    ld   r2, [0xd68]", 2)]
    for label, setup, ins, reg in cases:
        src = t.prologue() + "    movi r2, #0xdeadbeef\n    %s\n    %s\n    end\n" % (setup, ins) + t.trap()
        lint_ok = True
        try:
            code = assemble(t, src)
            lint(t, code, allow_indirect=True)
        except ToolError:
            lint_ok = False
        rec = probe(s, t, code, params=[(0xd64, 0x77777777), (0xd68, 0)])
        out[label] = rec
        lines.append("  %-36s -> %s%s" % (label, answer(rec) or "r%d = %08x" % (reg, rec["r"][reg]), "" if lint_ok else " [lint]"))
    return out, "indirect access:\n" + "\n".join(lines)


@experiment("unknown")
def ex_unknown(s, t):
    """Opcodes without a known meaning: what changes? (registers seeded, fault vector set)"""
    out, lines = {}, []
    seed = [0x10000001, 0x00000d70, 0x00000003, 0x30000003, 0x40000004, 0x50000005, 0x60000006, 0x70000007]
    ops = [0x0b] + [0x08, 0x09, 0x0f, 0x10, 0x12, 0x15, 0x16] + list(range(0x1a, 0x20)) + [0x20, 0x40, 0x80, 0xff]
    for op in ops:                          # 0x0b (or r3, r1 / or r3, r1, #imm) is the control row: a known op
        for imm in (0, 0x00000d74):
            src = t.prologue(c58=0) + "".join("    movi r%d, #0x%08x\n" % (i, v) for i, v in enumerate(seed)) + \
                "    .insn 0x%02x, 3, 1, 0x%08x\n    movi r6, #0x600d600d\n    end\n" % (op, imm) + t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0xd70, 0x84848484), (0xd74, 0x88888888)], timeout_us=200000)
            out["%02x/%x" % (op, imm)] = rec
            regs = [i for i in range(8) if i != 6 and rec["r"][i] != seed[i]]
            ch = {o: v for o, v in changes(rec).items() if o >= 0xd00 or o < 0x400}
            cc = ctrl_changes(t, rec)
            r6 = rec["r"][6]
            what = "continued" if r6 == 0x600d600d else "fault vector" if r6 == 0xfa17fa17 else "r6=%08x" % r6
            if not rec["pre_ok"]:
                lines.append("  op %02x imm %-6x -> %s" % (op, imm, answer(rec)))
                continue
            lines.append("  op %02x imm %-6x -> %-7s %-12s %s%s%s" % (op, imm, rec["why"], what,
                         " ".join("r%d=%08x" % (i, rec["r"][i]) for i in regs) or "no register changed",
                         "" if not ch else "; " + " ".join("0x%03x:%08x->%08x" % (o, a, b) for o, (a, b) in sorted(ch.items())),
                         "" if not cc else "; control " + cc))
    return out, ("unknown opcodes (.insn op, r3, arg 1, imm; r1 = 0xd70, r3 = 0x30000003; C58 = 0, so a fault flags and "
                 "continues; op 0b is the control: or r3, r1 [, #imm]):\n") + "\n".join(lines)


@experiment("op15")
def ex_op15(s, t):
    """Opcode 0x15 changed FMSS_C14 (0 with imm 0, 0xff with imm 0xd88): what does it write, from which field?"""
    out, lines = {}, []
    for imm in (0, 1, 0x7f, 0x80, 0xfe, 0xff, 0x100, 0x12345678):
        for reg, arg in ((0, 0), (5, 0), (0, 5)):
            src = t.prologue(c58=0) + "    movi r5, #0xa5a5a5a5\n    .insn 0x15, %d, %d, 0x%x\n    end\n" % (reg, arg, imm) + t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0xc14, 0x55)], allow=True)
            out["%x/%d/%d" % (imm, reg, arg)] = rec
            lines.append("  imm %-10x reg %d arg %d -> %-7s C14 %08x  %s" % (imm, reg, arg, rec["why"], rec["after"][0xc14],
                         ctrl_changes(t, rec)))
    return out, "op 0x15 (C14 preset 0x55 by the CPU; r5 = a5a5a5a5):\n" + "\n".join(lines)


@experiment("op12")
def ex_op12(s, t):
    """Opcode 0x12 blocks (no event timeout) and latches itself in 0xc40/0xc44: is it a conditional wait on a register?
    0xd60 is preset to 0x00001234 by the CPU; each case passes (continues) or blocks (probe timeout)"""
    out, lines = {}, []
    cases = [  # (label, reg value r2, reg field, arg, imm)
        ("[0xd60] & ffffffff == r2 (1234)", 0x1234, 2, 0xd60, 0xffffffff),
        ("[0xd60] & ffffffff == r2 (9999)", 0x9999, 2, 0xd60, 0xffffffff),
        ("[0xd60] & 0 == r2 (0)", 0, 2, 0xd60, 0),
        ("[0xd60] & ff00 == r2 (1200)", 0x1200, 2, 0xd60, 0xff00),
        ("[0xd60] & ff00 == r2 (0034)", 0x0034, 2, 0xd60, 0xff00),
        ("[0xd60] & ffffffff != r2 (0)", 0, 2, 0xd60, 0xffffffff),
        ("[0xd64] (0) & ffffffff, r2 0", 0, 2, 0xd64, 0xffffffff),
        ("[0xd64] (0) & ffffffff, r2 1", 1, 2, 0xd64, 0xffffffff),
    ]
    for label, rv, reg, arg, imm in cases:
        src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r2, #0x%x\n    .insn 0x12, %d, 0x%x, 0x%x\n" \
            "    movi r6, #0x600d600d\n    end\n" % (rv, reg, arg, imm) + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xd60, 0x1234), (0xd64, 0)], timeout_us=50000)
        out[label] = rec
        passed = rec["r"][6] == 0x600d600d
        lines.append("  %-34s -> %-8s %s after %6d us; %s" % (label, "passed" if passed else "BLOCKED", rec["why"],
                     rec["elapsed_us"], ctrl_changes(t, rec)))
    return out, "op 0x12 (r2 = expected value, arg = register offset, imm = mask):\n" + "\n".join(lines)


@experiment("op12wait", needs=("nand_seq",))
def ex_op12wait(s, t):
    """Is opcode 0x12 a WAIT without a timeout (event number in the register field, like `wait`)? No NAND access"""
    out, lines = {}, []
    cases = [("0x12 reg 11 (event 11, set at idle)", "    .insn 0x12, 11, 0, 0\n"),
             ("0x12 reg 0 after a command byte (event 0 set)", "    wr   [FMC_CMD], #0x70\n    .insn 0x12, 0, 0, 0\n"),
             ("wait 0 after a command byte (control)", "    wr   [FMC_CMD], #0x70\n    wait 0\n"),
             ("0x12 reg 3 (event 3, not set)", "    .insn 0x12, 3, 0, 0\n"),
             ("0x12 reg 11, arg 5, imm 0x40", "    .insn 0x12, 11, 5, 0x40\n")]
    for label, body in cases:
        src = t.prologue(c58=0) + "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x0\n" + body + \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000)
        out[label] = rec
        lines.append("  %-46s -> %-7s %s after %6d us, CS_IRQ %08x, C08 %x" % (
            label, rec["why"], "passed" if rec["r"][6] == 0x600d600d else "BLOCKED", rec["elapsed_us"], rec["cs_irq"],
            rec["after"][0xc08]))
    return out, "opcode 0x12 with the event in the register field:\n" + "\n".join(lines)


@experiment("op12ack")
def ex_op12ack(s, t):
    """Opcode 0x12 blocks with CS_IRQ b3 set: which CPU write (2 ms after the start) lets it continue?"""
    out, lines = {}, []
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    .insn 0x12, 2, 0xd60, 0xffffffff\n    movi r6, #0x600d600d\n" \
        "    end\n" + t.trap()
    code = assemble(t, src)
    for label, late in (("none (control)", []), ("CS_IRQ = 0x08 (ack b3)", [(0xc0c, 0x08)]),
                        ("CS_IRQ = 0xff", [(0xc0c, 0xff)]), ("C08 = 0", [(0xc08, 0)]), ("C08 = 0xb", [(0xc08, 0xb)]),
                        ("C40 = 0", [(0xc40, 0)]), ("C14 = 0", [(0xc14, 0)]), ("C10 = 0", [(0xc10, 0)])):
        rec = probe(s, t, code, params=[(0xd60, 0x1234)], late=late, late_us=2000, timeout_us=30000)
        out[label] = rec
        passed = rec["r"][6] == 0x600d600d
        lines.append("  %-24s -> %-8s %-7s after %6d us (late write at %d us), CS_IRQ %08x, C08 %x" % (
            label, "CONTINUED" if passed else "blocked", rec["why"], rec["elapsed_us"], rec["late_at_us"],
            rec["cs_irq"], rec["after"][0xc08]))
    return out, "op 0x12 and a CPU write while it blocks:\n" + "\n".join(lines)


@experiment("op12rel", needs=("nand_seq",))
def ex_op12rel(s, t):
    """Opcode 0x12 blocks for good: does any other CPU write release it (2 ms after the start)?"""
    src = t.prologue(c58=0) + "    wr   [FMC_CTRL0], #@CE0@\n    movi r6, #0x0\n    .insn 0x12, 0, 0, 0\n" \
        "    movi r6, #0x600d600d\n    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n" + t.trap()
    code = assemble(t, src)
    cases = [("none (control)", [])] + [("%s" % n, [(o, v)]) for n, o, v in (
        ("C48 = 1", 0xc48, 1), ("C54 = 1", 0xc54, 1), ("C3C = 1", 0xc3c, 1), ("C5C = 1", 0xc5c, 1),
        ("C58 = 0xff", 0xc58, 0xff), ("C4C = 0x40", 0xc4c, 0x40), ("C10 = 0", 0xc10, 0), ("C10 = 0xff", 0xc10, 0xff),
        ("FMSS_CTRL = 0x1", 0xc00, 0x1), ("FMSS_CTRL = 0x3", 0xc00, 0x3), ("FMSS_CTRL = 0x9", 0xc00, 0x9),
        ("FMSS_CTRL = 0x4", 0xc00, 0x4), ("FMSS_CTRL = 0x20001", 0xc00, 0x20001),
        ("CPU sends 70h (FMC_CMD)", 0x008, 0x70), ("FMC_STATUS = ~0", 0x048, 0xffffffff),
        ("pram 0xd00 = 1", 0xd00, 1), ("r0 = 1", 0xc18, 1), ("C14 = 0", 0xc14, 0))]
    out, lines = {}, []
    for label, late in cases:
        rec = probe(s, t, code, late=late, late_us=2000, timeout_us=20000)
        out[label] = rec
        lines.append("  %-26s -> %-9s %-7s after %6d us, CS_IRQ %08x, C08 %x" % (
            label, "CONTINUED" if rec["r"][6] == 0x600d600d else "blocked", rec["why"], rec["elapsed_us"],
            rec["cs_irq"], rec["after"][0xc08]))
    return out, "op 0x12 and one CPU write while it blocks:\n" + "\n".join(lines)


@experiment("op10")
def ex_op10(s, t):
    """Opcode 0x10 ended with CS_IRQ = 3: does it raise CS_IRQ b1 and carry on?"""
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    .insn 0x10, 0, 0, 0\n    ld   r2, [CS_IRQ]\n" \
        "    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src))
    return rec, "op 0x10: %s, r6 %08x (%s), CS_IRQ read by the program right after it %08x, CS_IRQ at the end %08x" % (
        rec["why"], rec["r"][6], "carried on" if rec["r"][6] == 0x600d600d else "stopped", rec["r"][2], rec["cs_irq"])


@experiment("op0f")
def ex_op0f(s, t):
    """Opcode 0x0f: an indirect jump? (device: the PC ended at the seeded register's value + 8.) Which register field
    names the target, is it a byte offset from the program start, is a return address kept anywhere?"""
    out, lines = {}, []
    tail = ("    movi r6, #0xbad0bad0\n    end\n"
            "    .insn 0x08, 0, 0, 0\n"
            "lab1:\n    movi r6, #0x11111111\n    end\n"
            "lab2:\n    movi r6, #0x22222222\n    end\n")
    cases = [("arg 1 -> r1 = lab1", "    movi r1, #lab1\n    movi r0, #lab2\n    .insn 0x0f, 0, 1, 0\n"),
             ("reg 1, arg 0 -> r0 = lab2", "    movi r1, #lab1\n    movi r0, #lab2\n    .insn 0x0f, 1, 0, 0\n"),
             ("arg 9 (r1 if 3 bits)", "    movi r1, #lab1\n    movi r0, #lab2\n    .insn 0x0f, 0, 9, 0\n"),
             ("imm = lab2, r1 = lab1", "    movi r1, #lab1\n    movi r0, #lab2\n    .insn 0x0f, 0, 1, lab2\n"),
             ("r1 = lab1 + 4 (unaligned)", "    movi r1, #lab1\n    add  r1, r1, #0x4\n    movi r0, #lab2\n"
              "    .insn 0x0f, 0, 1, 0\n"),
             ("r1 = lab1 - 8 (the 0x08 no-op)", "    movi r1, #lab1\n    sub  r1, r1, #0x8\n    movi r0, #lab2\n"
              "    .insn 0x0f, 0, 1, 0\n")]
    for label, body in cases:
        src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r2, #0x0\n    movi r3, #0x0\n    movi r4, #0x0\n" \
            "    movi r5, #0x0\n    movi r7, #0x0\n" + body + tail + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xc3c, 0), (0xc48, 0)], timeout_us=20000, flags=WAIT_END)
        out[label] = rec
        r6 = rec["r"][6]
        land = {0x11111111: "lab1", 0x22222222: "lab2", 0xbad0bad0: "fell through", 0: "nothing"}.get(r6, "%08x" % r6)
        lines.append("  %-32s -> %-7s %-12s r2..r5,r7 %s; C3C %08x C48 %08x; CS_IRQ %08x" % (
            label, rec["why"], land, " ".join("%x" % rec["r"][i] for i in (2, 3, 4, 5, 7)), rec["after"][0xc3c],
            rec["after"][0xc48], rec["cs_irq"]))
    return out, "opcode 0x0f (labels are byte offsets from the program start):\n" + "\n".join(lines)


@experiment("regnum")
def ex_regnum(s, t):
    """Register numbers 8..255 in the register and argument fields: which register do they read and write?"""
    seed = "".join("    movi r%d, #0x%08x\n" % (i, 0x10101010 * (i + 1)) for i in range(8) if i != 6) + "    movi r6, #0x0\n"
    base = t.scratch_addr(s)
    cases = [("mov r1, r9", "    .insn 0x06, 1, 9, 0\n"), ("mov r1, r15", "    .insn 0x06, 1, 15, 0\n"),
             ("mov r1, r255", "    .insn 0x06, 1, 255, 0\n"), ("add r1, r9, #1", "    .insn 0x0c, 1, 9, 1\n"),
             ("st [0xd70], r9", "    .insn 0x02, 9, 0xd70, 0\n"), ("st [0xd70], r14", "    .insn 0x02, 14, 0xd70, 0\n")]
    cases += [("movi r%d, #0x5eed00%02x" % (n, n), "    .insn 0x05, %d, 0, 0x5eed00%02x\n" % (n, n))
              for n in (8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 255)]
    cases += [("ldi r1, [r9] (r0 = scratch)", "    movi r0, #0x%x\n    .insn 0x03, 1, 9, 0\n" % base),
              ("bnz r9 taken? (r0 != 0)", "    .insn 0x0e, 9, 0, skip\n    movi r6, #0xbad0bad0\n    end\nskip:\n")]
    out, lines = {}, []
    for label, body in cases:
        src = t.prologue(c58=0) + seed + body + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xd70, 0)], timeout_us=20000, flags=WAIT_END,
                    scratch=struct.pack("<I", 0xcafe0001) * 4)
        out[label] = rec
        regs = " ".join("r%d=%08x" % (i, rec["r"][i]) for i in range(8)
                        if i != 6 and rec["r"][i] != 0x10101010 * (i + 1) and not (i == 0 and label.startswith("ldi")))
        win = " ".join("0x%03x:%08x->%08x" % (o, a, b) for o, (a, b) in sorted(changes(rec).items())
                       if o >= 0xc00 and o not in t.ctrl_skip | {0xc4c, 0xc58} and not 0xc18 <= o < 0xc38)
        lines.append("  %-30s -> %-6s r6 %08x; %s%s; d70 %08x" % (label, rec["why"], rec["r"][6], regs or "no register",
                     "; " + win if win else "", rec["after"][0xd70]))
    return out, "register numbers above 7 (r0..r7 seeded 0x10101010 * (n + 1)):\n" + "\n".join(lines)


@experiment("branch")
def ex_branch(s, t):
    """bz / bnz edge cases: misaligned target, the argument field, a sign bit"""
    tail = ("    movi r6, #0xbad0bad0\n    end\n    .insn 0x08, 0, 0, 0\n"
            "lab:\n    movi r6, #0x11111111\n    end\n")
    cases = [("bnz r1=1 -> lab (control)", "    movi r1, #0x1\n    bnz  r1, lab\n"),
             ("bnz r1=1 -> lab + 4", "    movi r1, #0x1\n    .insn 0x0e, 1, 0, lab + 4\n"),
             ("bnz r1=1 -> lab - 8", "    movi r1, #0x1\n    .insn 0x0e, 1, 0, lab - 8\n"),
             ("bnz r1=1, arg 5 -> lab", "    movi r1, #0x1\n    .insn 0x0e, 1, 5, lab\n"),
             ("bnz r1=0x80000000 -> lab", "    movi r1, #0x80000000\n    bnz  r1, lab\n"),
             ("bz r1=0, arg 3 -> lab", "    movi r1, #0x0\n    .insn 0x17, 1, 3, lab\n"),
             ("bz r1=1 (not taken)", "    movi r1, #0x1\n    bz   r1, lab\n")]
    out, lines = {}, []
    for label, body in cases:
        src = t.prologue(c58=0) + "    movi r6, #0x0\n" + body + tail + t.trap()
        m = re.search(r"lab ([+-]) (\d+)", src)
        if m:                                             # the assembler takes labels, not expressions
            first = assemble(t, src.replace(m.group(0), "lab"))
            mark = struct.pack("<II", 0x05060000, 0x11111111)
            at = next(i for i in range(0, len(first), 8) if first[i:i + 8] == mark)
            src = src.replace(m.group(0), "0x%x" % (at + int(m.group(2)) * (1 if m.group(1) == "+" else -1)))
        code = assemble(t, src)
        rec = probe(s, t, code, timeout_us=20000, flags=WAIT_END)
        out[label] = rec
        r6 = rec["r"][6]
        lines.append("  %-28s -> %-6s %s, CS_IRQ %08x" % (label, rec["why"], {0x11111111: "took the branch",
                     0xbad0bad0: "fell through", 0: "neither (r6 0)"}.get(r6, "r6 %08x" % r6), rec["cs_irq"]))
    return out, "branches:\n" + "\n".join(lines)


@experiment("r8")
def ex_r8(s, t):
    """Register numbers 8 and 10 had effects (movi r8 ended the program early, movi r10 wrote every register): is r8
    the PC, and what is r10?"""
    tail = "    movi r6, #0xbad0bad0\n    end\nlab:\n    movi r6, #0x11111111\n    end\n"
    seed = "".join("    movi r%d, #0x%08x\n" % (i, 0x10101010 * (i + 1)) for i in range(8) if i != 6) + "    movi r6, #0x0\n"
    cases = [("movi r8, #lab", "    .insn 0x05, 8, 0, lab\n"), ("movi r8, #0x5eed0008 (repeat)", "    .insn 0x05, 8, 0, 0x5eed0008\n"),
             ("mov r8, r1 (r1 = lab)", "    movi r1, #lab\n    .insn 0x06, 8, 1, 0\n"),
             ("mov r1, r8 (read r8)", "    .insn 0x06, 1, 8, 0\n"),
             ("movi r10, #0x12345678 (repeat)", "    .insn 0x05, 10, 0, 0x12345678\n"),
             ("mov r10, r2", "    .insn 0x06, 10, 2, 0\n"), ("add r10, r2, #1", "    .insn 0x0c, 10, 2, 1\n"),
             ("movi r26 (10 + 16)", "    .insn 0x05, 26, 0, 0x26262626\n"),
             ("movi r24 (8 + 16)", "    .insn 0x05, 24, 0, lab\n"),
             ("ldi r1, [r8]", "    .insn 0x03, 1, 8, 0\n"), ("ldi r1, [r10]", "    .insn 0x03, 1, 10, 0\n")]
    out, lines = {}, []
    for label, body in cases:
        src = t.prologue(c58=0) + seed + body + tail + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END | F_SAMPLE_REGS)
        out[label] = rec
        r6 = rec["r"][6]
        lines.append("  %-30s -> %-7s %s; r %s; PC %x" % (label, rec["why"], {0x11111111: "LANDED ON lab",
                     0xbad0bad0: "fell through", 0: "ended without either"}.get(r6, "r6 %08x" % r6),
                     " ".join("%08x" % rec["r"][i] for i in range(8)), rec["pc"]))
    return out, "register numbers 8 / 10 (r0..r7 seeded 0x10101010 * (n + 1)):\n" + "\n".join(lines)


@experiment("opcost", needs=("pld_pst", "sram_prog"))
def ex_opcost(s, t):
    """Time per instruction type (100 x 10 in a loop, against movi), from SDRAM and from internal SRAM"""
    a = t.scratch_addr(s)
    pre = "    movi r1, #0x%x\n    movi r2, #0xd70\n    movi r7, #0x0\n" % a
    bodies = [("movi", "    movi r5, #0x0\n"), ("mov", "    mov  r5, r1\n"), ("add imm", "    add  r5, r1, #0x4\n"),
              ("08 no-op", "    .insn 0x08, 0, 0, 0\n"), ("ldi (RAM)", "    ldi  r5, [r1]\n"),
              ("sti (RAM)", "    sti  [r1], r5\n"), ("wr pram", "    wr   [0xd70], #0x1\n"),
              ("st pram", "    st   [0xd70], r5\n"), ("ld pram", "    ld   r5, [0xd70]\n"),
              ("ld FMC_STATUS", "    ld   r5, [FMC_STATUS]\n"), ("wr FMC_ADDR0", "    wr   [FMC_ADDR0], #0x0\n"),
              ("pld pram", "    pld  r5, [r2]\n"), ("pst pram", "    pst  [r2], r5\n"),
              ("bnz not taken", "    bnz  r7, loop\n"), ("wait 11 (passes)", "    wait 11\n")]
    out, lines = {}, []
    for where, fl in (("SDRAM", 0), ("SRAM", PROG_SRAM)):
        ctl = None
        for label, body in bodies:
            src = t.prologue(c58=0) + pre + "    movi r0, #100\nloop:\n" + body * 10 + \
                "    sub  r0, r0, #0x1\n    bnz  r0, loop\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=500000, flags=WAIT_END | fl, scratch=bytes(64))
            out["%s/%s" % (where, label)] = rec
            if ctl is None:
                ctl = rec["elapsed_us"]
            lines.append("  %-5s %-17s -> %-6s %5d us = %4.0f ns per instruction (%+4.0f ns vs movi)%s" % (
                where, label, rec["why"], rec["elapsed_us"], 1000.0 * rec["elapsed_us"] / 1200,
                (rec["elapsed_us"] - ctl), "" if rec["r"][6] == 0x600d600d else "  r6 %08x" % rec["r"][6]))
            print(lines[-1])
            sys.stdout.flush()
    return out, "100 iterations of 10 instructions + 2 loop instructions:\n" + "\n".join(lines)


@experiment("optime")
def ex_optime(s, t):
    """Opcodes with no visible effect: do they take time? 1000 executions in a loop against `movi`, several operand
    forms (also each opcode opsweep found continuing fast with no change, when run in the same battery)"""
    ops = [0x08, 0xff, 0x10]
    sw = (RESULTS.get("opsweep") or {}).get("data") or {}
    for key, rec in sw.items():
        op = int(key.split("/")[0], 16)
        if op not in ops and rec.get("why") == "END" and rec["r"][6] == 0x600d600d and rec["elapsed_us"] < 1500 \
                and key.endswith(SWEEP_FORMS[0][0]):
            ops.append(op)
    forms = [(0, 0, 0), (7, 0xffff, 0xffffffff), (3, 0xd70, 0x1000)]
    out, lines = {}, []

    def run(body):
        src = t.prologue(c58=0) + "    movi r0, #100\nloop:\n" + body * 10 + "    sub  r0, r0, #0x1\n    bnz  r0, loop\n" \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        return probe(s, t, assemble(t, src), timeout_us=500000, flags=WAIT_END)
    ctl = run("    movi r5, #0x0\n")
    out["movi"] = ctl
    lines.append("  movi (control): %d us" % ctl["elapsed_us"])
    for op in ops:
        for reg, arg, imm in forms:
            rec = run("    .insn 0x%02x, %d, 0x%x, 0x%08x\n" % (op, reg, arg, imm))
            out["%02x/%d/%x/%x" % (op, reg, arg, imm)] = rec
            d = (rec["elapsed_us"] - ctl["elapsed_us"]) / 1000.0
            lines.append("  op %02x reg %d arg %-5x imm %-8x -> %-7s %6d us (%+.2f us per execution vs movi), "
                         "CS_IRQ %08x, r6 %08x" % (op, reg, arg, imm, rec["why"], rec["elapsed_us"], d,
                                                   rec["cs_irq"], rec["r"][6]))
            print(lines[-1])
            sys.stdout.flush()
    return out, "1000 executions each:\n" + "\n".join(lines)


@experiment("holdout", needs=("pld_pst",))
def ex_holdout(s, t):
    """Held-out combinations of rules found earlier (the model's predictions were written down before this ran)"""
    out, lines = {}, []
    cases = [
        ("three timeouts, C58 0", t.prologue(c58=0) + "    wait 1\n    wait 2\n    wait 3\n    movi r6, #0x600d600d\n    end\n", []),
        ("shl 1 by 31 / shr by 31", t.prologue() + "    movi r1, #0x1\n    shl  r2, r1, #0x1f\n    shr  r3, r2, #0x1f\n"
         "    movi r6, #0x600d600d\n    end\n", []),
        ("pld 0xc30 reads r6", t.prologue() + "    movi r6, #0x6666aaaa\n    movi r1, #0xc30\n    pld  r2, [r1]\n    end\n", []),
        ("op15 x4", t.prologue(c58=0) + "    .insn 0x15, 0, 0, 0\n" * 4 + "    ld   r2, [FMSS_C14]\n    end\n", []),
        ("loop of 100", t.prologue() + "    movi r0, #100\nl:\n    sub  r0, r0, #0x1\n    bnz  r0, l\n    movi r6, #0x600d600d\n    end\n", []),
        ("wait 11, timeout, op10", t.prologue(c58=0) + "    wait 11\n    wait 0\n    .insn 0x10, 0, 0, 0\n"
         "    movi r6, #0x600d600d\n    end\n", []),
        ("sti then ldi at scratch+0x200", None, []),
    ]
    for label, src, params in cases:
        if src is None:
            base = t.scratch_addr(s) + 0x200
            src = t.prologue() + "    movi r1, #0x%08x\n    movi r2, #0x13572468\n    sti  [r1], r2\n    add  r3, r1, #0x3\n" \
                "    ldi  r4, [r3]\n    movi r6, #0x600d600d\n    end\n" % base
        rec = probe(s, t, assemble(t, src + t.trap()), params=params, timeout_us=50000, scratch=bytes(512), flags=WAIT_END)
        out[label] = rec
        lines.append("  %-30s -> %-7s %7d us  CS_IRQ %08x  r2 %08x r3 %08x r4 %08x r6 %08x  C14 %02x->%02x" % (
            label, rec["why"], rec["elapsed_us"], rec["cs_irq"], rec["r"][2], rec["r"][3], rec["r"][4], rec["r"][6],
            rec["before"][0xc14], rec["after"][0xc14]) + ("  first fault at %d us" % rec["fault_at_us"] if rec["fault_at_us"] else ""))
    return out, "held-out combinations (followed to the END after a fault):\n" + "\n".join(lines)
