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

"""WAIT events and the opcodes that look like waits (09, 12, 16), around NAND activity and pending flags."""
import sys

from ..probe import (probe, assemble, ctrl_changes, where, soft_reset, F_WIN400, WAIT_END)
from .registry import experiment


@experiment("wait")
def ex_wait(s, t):
    """WAIT N with nothing pending: which events are already set, which time out, how long the timeout is"""
    out, lines = {}, []
    for e in list(range(0, 16)):
        src = t.prologue(c58=4) + "    wait %d\n    movi r6, #0x600d600d\n    end\n" % e + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=500000)
        out[e] = rec
        r6 = rec["r"][6]
        verdict = "passed" if r6 == 0x600d600d else "fault vector" if r6 == 0xfa17fa17 else "no end"
        lines.append("  wait %-2d -> %-7s %-12s after %6d us, CS_IRQ %08x; %s" % (e, rec["why"], verdict,
                     rec["elapsed_us"], rec["cs_irq"], ctrl_changes(t, rec) or "no control-word change"))
    return out, "WAIT:\n" + "\n".join(lines)


@experiment("waitf")
def ex_waitf(s, t):
    """WAIT's other fields: the 4G/5G engine waits on a register under a mask; does the 3G use arg / imm, and do
    event numbers above 15 alias?"""
    cases = [("wait 11 (control)", 11, 0, 0), ("wait 3, arg 0x048, imm 0x3000 (bits set)", 3, 0x048, 0x3000),
             ("wait 3, arg 0x048, imm 0x1 (bit clear)", 3, 0x048, 0x1),
             ("wait 3, arg 0xd70, imm ~0 (word set)", 3, 0xd70, 0xffffffff), ("wait 3, imm ~0", 3, 0, 0xffffffff),
             ("wait 11, arg 0xd74, imm ~0 (word 0)", 11, 0xd74, 0xffffffff), ("wait 27 (11 + 16)", 27, 0, 0),
             ("wait 16 (0 + 16)", 16, 0, 0), ("wait 0x8b (11 + 128)", 0x8b, 0, 0), ("wait 255", 255, 0, 0)]
    out, lines = {}, []
    for label, ev, arg, imm in cases:
        src = t.prologue(c58=0) + "    movi r6, #0x0\n    .insn 0x07, %d, 0x%x, 0x%08x\n" % (ev, arg, imm) + \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xd70, 0x12345678), (0xd74, 0)], timeout_us=20000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-42s -> %-6s %s after %5d us" % (label, rec["why"], "TIMED OUT" if rec["fault_at_us"]
                     else "passed", rec["elapsed_us"]))
    return out, "WAIT with its other fields set:\n" + "\n".join(lines)


@experiment("ev11", needs=("nand_seq",))
def ex_ev11(s, t):
    """Event 11: the chips' ready line? WAIT 11 right after starting a block erase on CE0 (destructive, scratch block)"""
    out, lines = {}, []
    cases = [("idle CE0, wait 11 (control)", t.nand_prologue(0) + "    wait 11\n"),
             ("erase CE0, end at once (control)", t.nand_prologue(0) + t.erase),
             ("erase CE0, wait 11", t.nand_prologue(0) + t.erase + "    wait 11\n"),
             ("erase CE0, select CE1, wait 11", t.nand_prologue(0) + t.erase + "    wr   [FMC_CTRL0], #0x%x\n    wait 11\n" % t.ce_sel[1]),
             ("erase CE0, deselect, wait 11", t.nand_prologue(0) + t.erase + "    wr   [FMC_CTRL0], #0x%x\n    wait 11\n" % t.ce_sel[None]),
             ("erase CE0, wait 11, wait 11", t.nand_prologue(0) + t.erase + "    wait 11\n    wait 11\n")]
    for label, body in cases:
        src = body + "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=50000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-34s -> %-7s %6d us, CS_IRQ %08x%s" % (label, rec["why"], rec["elapsed_us"], rec["cs_irq"],
                     "  (fault at %d us)" % rec["fault_at_us"] if rec["fault_at_us"] else ""))
    return out, "event 11 around a block erase (tBERS ~1-2 ms):\n" + "\n".join(lines)


@experiment("ev5", needs=("nand_seq",))
def ex_ev5(s, t):
    """Event 5: status byte refreshed, or chip ready? Status poll (70h, FMCTRL1 0xca) during an erase and when idle"""
    out, lines = {}, []
    poll = (t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    ld   r2, [FMC_NSTAT], 0xff\n    wait 5\n"
            "    wr   [FMC_STATUS], #0x800000\n    ld   r3, [FMC_NSTAT], 0xff\n")
    cases = [("idle: poll, wait 5", t.nand_prologue(0) + poll),
             ("erase, poll, wait 5", t.nand_prologue(0) + t.erase + poll),
             ("erase, poll, wait 5, re-latch x8", t.nand_prologue(0) + t.erase + poll + (
                 "    wr   [FMCTRL1], #0xca\n    wait 5\n    wr   [FMC_STATUS], #0x800000\n") * 8 +
              "    ld   r4, [FMC_NSTAT], 0xff\n"),
             ("erase, wait 11, poll, wait 5", t.nand_prologue(0) + t.erase + "    wait 11\n" + poll)]
    for label, body in cases:
        src = body + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xc20, 0), (0xc24, 0), (0xc28, 0)], timeout_us=50000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-34s -> %-7s %6d us, status before wait 5 %02x, after %02x, last %02x, CS_IRQ %08x" % (
            label, rec["why"], rec["elapsed_us"], rec["r"][2], rec["r"][3], rec["r"][4], rec["cs_irq"]))
    return out, "event 5 and the status byte (b6 = ready, b0 = fail):\n" + "\n".join(lines)


@experiment("evsweep", needs=("nand_seq",))
def ex_evsweep(s, t):
    """All 16 WAIT events around NAND activity (destructive: erases scratch block 3000 of CE0): which pass at once,
    which pass later, which time out; FMC_STATUS after each wait (a sequencer read of FMC_STATUS before the wait
    consumed events on the device, so the program does not read it first)"""
    ctx = [("idle, CE0 selected", ""),
           ("after a command byte (70h)", "    wr   [FMC_CMD], #0x70\n"),
           ("after 70h + status latch", "    wr   [FMC_CMD], #0x70\n    wr   [FMCTRL1], #0xca\n"),
           ("during an erase", t.erase),
           ("2 ms after an erase start", t.erase + "    movi r0, #4500\ndly:\n    sub  r0, r0, #0x1\n    bnz  r0, dly\n")]
    out, lines = {}, []
    for label, body in ctx:
        base = t.nand_prologue(0) + body
        tail = "    ld   r3, [FMC_STATUS]\n    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        ref = probe(s, t, assemble(t, base + tail), timeout_us=50000, flags=WAIT_END)
        out["%s/none" % label] = ref
        row = []
        for e in range(16):
            rec = probe(s, t, assemble(t, base + "    wait %d\n" % e + tail), timeout_us=50000, flags=WAIT_END)
            out["%s/%d" % (label, e)] = rec
            d = rec["elapsed_us"] - ref["elapsed_us"]
            row.append("%d:%s" % (e, "T" if d > 4000 else "%dus" % max(d, 0)))
        lines.append("  %-28s status %08x | %s" % (label, ref["r"][3], " ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    return out, ("wait N after each context (time beyond the same program without the wait; T = timed out):\n" +
                 "\n".join(lines))


@experiment("evclear", needs=("nand_seq",))
def ex_evclear(s, t):
    """Does a sequencer read of FMC_STATUS (or FMC_NSTAT) consume a pending event before WAIT sees it?"""
    status = t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n"
    cases = [("status latch, wait 5", status, "wait 5"),
             ("status latch, ld FMC_STATUS, wait 5", status + "    ld   r2, [FMC_STATUS]\n", "wait 5"),
             ("status latch, ld FMC_NSTAT, wait 5", status + "    ld   r2, [FMC_NSTAT]\n", "wait 5"),
             ("status latch, ld FMC_STATUS x2, wait 5", status + "    ld   r2, [FMC_STATUS]\n    ld   r2, [FMC_STATUS]\n", "wait 5"),
             ("idle, wait 3", "", "wait 3"),
             ("idle, ld FMC_STATUS, wait 3", "    ld   r2, [FMC_STATUS]\n", "wait 3"),
             ("70h, ld FMC_STATUS, wait 0", "    wr   [FMC_CMD], #0x70\n    ld   r2, [FMC_STATUS]\n", "wait 0"),
             ("idle, ld FMC_STATUS, wait 11", "    ld   r2, [FMC_STATUS]\n", "wait 11")]
    out, lines = {}, []
    for label, body, w in cases:
        src = t.nand_prologue(0) + body + "    movi r6, #0x0\n    %s\n    ld   r3, [FMC_STATUS]\n" % w + \
            "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=50000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-40s -> %-7s %6d us (%s), FMC_STATUS read before %08x, after %08x" % (
            label, rec["why"], rec["elapsed_us"], "timed out" if rec["fault_at_us"] else "passed", rec["r"][2],
            rec["r"][3]))
    return out, "pending events and FMC_STATUS reads:\n" + "\n".join(lines)


@experiment("ev5rule", needs=("nand_seq",))
def ex_ev5rule(s, t):
    """Event 5: what arms it? A status latch (FMCTRL1 = 0xca) after 70h passed WAIT 5 when the command-done flag
    (FMC_STATUS b1) had been cleared first, never when it was still set, and never for a second latch"""
    c70 = "    wr   [FMC_CMD], #0x70\n"
    w0, clr1, latch = "    wait 0\n", "    wr   [FMC_STATUS], #0x2\n", "    wr   [FMCTRL1], #0xca\n"
    clr23 = "    wr   [FMC_STATUS], #0x800000\n"
    cases = [("70h, wait 0, clear b1, latch (control)", c70 + w0 + clr1 + latch),
             ("70h, wait 0, latch (b1 still set)", c70 + w0 + latch),
             ("70h, latch, clear b1", c70 + latch + "    wait 0\n" + clr1),
             ("70h, wait 0, latch, clear b1", c70 + w0 + latch + clr1),
             ("control, then clear b23, latch again", c70 + w0 + clr1 + latch + "    wait 5\n" + clr23 + latch),
             ("control, then latch again (b23 kept)", c70 + w0 + clr1 + latch + "    wait 5\n" + latch),
             ("control, then a new 70h cycle", c70 + w0 + clr1 + latch + "    wait 5\n" + clr23 + c70 + w0 + clr1 + latch),
             ("control, but 0x90 instead of 70h", "    wr   [FMC_CMD], #0x90\n" + w0 + clr1 + latch)]
    out, lines = {}, []
    for label, body in cases:
        src = t.nand_prologue(0) + body + "    movi r6, #0x0\n    wait 5\n    ld   r3, [FMC_STATUS]\n" \
            "    ld   r4, [FMC_NSTAT], 0xff\n    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=50000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-40s -> last WAIT 5 %-9s %6d us, FMC_STATUS %08x, status byte %02x, CS_IRQ %08x" % (
            label, "TIMED OUT" if rec["fault_at_us"] else "passed", rec["elapsed_us"], rec["r"][3], rec["r"][4],
            rec["cs_irq"]))
    return out, "what arms event 5 (the last WAIT 5 of each program is the question):\n" + "\n".join(lines)


@experiment("ev5t", needs=("nand_seq",))
def ex_ev5t(s, t):
    """WAIT 5 straight after 70h + status latch timed out (evsweep), but passed with a wait 0 in between (ev5rule):
    timing, the pending command flag, or an order rule?"""
    c70, latch = "    wr   [FMC_CMD], #0x70\n", "    wr   [FMCTRL1], #0xca\n"
    cases = [("70h, latch, wait 5", c70 + latch),
             ("70h, latch, clear b1, wait 5", c70 + latch + "    wr   [FMC_STATUS], #0x2\n"),
             ("70h, latch, wait 0, wait 5", c70 + latch + "    wait 0\n"),
             ("70h, latch, 20 movi, wait 5", c70 + latch + "    movi r5, #0x0\n" * 20),
             ("70h, latch, wait 15, wait 5", c70 + latch + "    wait 15\n"),
             ("70h, 20 movi, latch, wait 5", c70 + "    movi r5, #0x0\n" * 20 + latch),
             ("70h, wait 0, latch, wait 5", c70 + "    wait 0\n" + latch)]
    out, lines = {}, []
    for label, body in cases:
        src = t.nand_prologue(0) + body + "    movi r6, #0x0\n    wait 5\n    ld   r3, [FMC_STATUS]\n" \
            "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=50000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-30s -> %d timeout(s), %6d us, FMC_STATUS after %08x" % (
            label, round(rec["elapsed_us"] / 4870.0), rec["elapsed_us"], rec["r"][3]))
    return out, "70h + status latch and the last WAIT 5:\n" + "\n".join(lines)


@experiment("evcons", needs=("nand_seq",))
def ex_evcons(s, t):
    """Does a WAIT consume its event? ('70h, latch, wait 15, wait 5' timed out once while every other order passed)"""
    c70, latch = "    wr   [FMC_CMD], #0x70\n", "    wr   [FMCTRL1], #0xca\n"
    cases = [("70h, latch, wait 15", c70 + latch, ["wait 15"]),
             ("70h, latch, wait 15, wait 5", c70 + latch, ["wait 15", "wait 5"]),
             ("70h, latch, wait 5, wait 15", c70 + latch, ["wait 5", "wait 15"]),
             ("70h, latch, wait 5, wait 5", c70 + latch, ["wait 5", "wait 5"]),
             ("70h, wait 0, wait 0", c70, ["wait 0", "wait 0"]),
             ("70h, wait 15, wait 0", c70, ["wait 15", "wait 0"]),
             ("70h, wait 0, wait 15", c70, ["wait 0", "wait 15"]),
             ("70h, latch, wait 15, wait 15, wait 15", c70 + latch, ["wait 15", "wait 15", "wait 15"])]
    out, lines = {}, []
    for label, body, waits in cases:
        # C58 = 0: a timeout continues; after each wait r1..r3 = its CS_IRQ fault bit (4 = timed out), then acknowledged
        prog = t.nand_prologue(0) + body
        for k, w in enumerate(waits):
            prog += "    %s\n    ld   r%d, [CS_IRQ], 0x4\n    wr   [CS_IRQ], #0x4\n" % (w, k + 1)
        prog += "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, prog), timeout_us=60000, flags=WAIT_END)
        out[label] = rec
        res = " ".join("%s:%s" % (w, "T" if rec["r"][k + 1] else "ok") for k, w in enumerate(waits))
        lines.append("  %-38s -> %s  (%d us, FMC_STATUS %08x)" % (label, res, rec["elapsed_us"], rec["after"][0x048]))
    return out, "which wait of each program passed (ok) or timed out (T):\n" + \
        "\n".join(lines)


@experiment("ev8", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_ev8(s, t):
    """Event 8 passed at every step after a soft reset (the FMC register set at +0x400 still held a READ ID sent
    through it) and never after a power cycle: do events 6..15 belong to the second set? Its READ ID steps one by one,
    every event after each"""
    steps = [("set 0x400: 90h", t.ch_regs("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
                                         "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x90\n", 0x400)),
             ("set 0x400: + address", t.ch_regs("    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n"
                                               "    wr   [FMCTRL1], #0x1\n", 0x400)),
             ("set 0x400: + ID transfer", t.ch_regs("    wr   [FMC_DNUM], #0x7\n    wr   [FMC_DATA_SEL], #0x1\n"
                                                  "    wr   [FMCTRL1], #0x1c2\n", 0x400))]
    tail = "    ld   r2, [0x448]\n    ld   r3, [0x450]\n    ld   r4, [0x050]\n    wr   [0x400], #@CE_NONE@\n" \
        "    movi r6, #0x600d600d\n    end\n" + t.trap()
    out, lines, acc = {}, [], t.prologue(c58=0)
    for name, body in steps:
        acc += body
        row = []
        for e in list(range(16)) + [None]:
            src = acc + ("    wait %d\n" % e if e is not None else "") + tail
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END | F_WIN400)
            out["%s/%s" % (name, e)] = rec
            if e is not None:
                row.append("%d:%s" % (e, "T" if rec["fault_at_us"] else "ok"))
        lines.append("  %-26s 0x448 %08x 0x450 %08x 0x050 %08x | %s" % (name, rec["r"][2], rec["r"][3], rec["r"][4],
                                                                       " ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    # the soft-reset state itself: a full READ ID through the set, then the events in a fresh program
    probe(s, t, assemble(t, t.prologue(c58=0) + t.read_id(t.ce_sel[0], 0x400) + "    end\n" + t.trap()), timeout_us=60000,
          flags=WAIT_END)
    row = []
    for e in range(16):
        rec = probe(s, t, assemble(t, t.prologue(c58=0) + "    wait %d\n" % e + tail), timeout_us=20000, flags=WAIT_END)
        out["after a READ ID through 0x400, next program/%d" % e] = rec
        row.append("%d:%s" % (e, "T" if rec["fault_at_us"] else "ok"))
    lines.append("  %-26s %s" % ("next program, no NAND step", " ".join(row)))
    return out, "events after each step through the second register set (ok = passed, T = timed out):\n" + "\n".join(lines)


@experiment("irqen", needs=("nand_seq",))
def ex_irqen(s, t):
    """FMC 0x044 and ECC 0x844 as interrupt enables: one bit at a time, then a command byte (FMC_STATUS b1) or an ECC
    decode (ECC_DONE): which CS_IRQ bit and does the CPU line follow?"""
    out, lines = {}, []
    for reg, bits, act in ((0x044, range(16), t.cmd(0x70).replace("    wait 0\n    wr   [FMC_STATUS], #0x2\n", "    wait 0\n")),
                           (0x844, range(6), "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n"
                            "    wr   [ECC_KICK], #0x1101\n    wait 4\n")):
        for b in list(bits) + [None]:
            src = t.nand_prologue(0) + act + "    ld   r2, [FMC_STATUS]\n    ld   r3, [ECC_DONE]\n    ld   r4, [CS_IRQ]\n" \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            v = 0 if b is None else 1 << b
            rec = probe(s, t, assemble(t, src), params=[(reg, v)], timeout_us=20000, flags=WAIT_END)
            out["0x%03x=%x" % (reg, v)] = rec
            lines.append("  0x%03x = %-6x -> %-6s CS_IRQ in program %08x, after %08x, line %d/%d, FMC_STATUS %08x, "
                         "ECC_DONE %02x, 0x054 %x" % (reg, v, rec["why"], rec["r"][4], rec["cs_irq"], rec["irq_seen"],
                                                     rec["irq_end"], rec["r"][2], rec["r"][3], rec["after"][0x054]))
        # put it back
        probe(s, t, assemble(t, "    end\n"), params=[(reg, 0)], timeout_us=20000)
    return out, "one enable bit at a time (then 0), after a 70h command (0x044) or an ECC decode (0x844):\n" + \
        "\n".join(lines)


@experiment("readev", needs=("nand_seq",))
def ex_readev(s, t):
    """All 16 WAIT events at each step of a page read (CE0, scratch block 3000 page 0: 00h, address, 30h, status poll,
    parity and data transfer, ECC decode, DMA): which event belongs to which step, which pass together"""
    tail = "    ld   r3, [FMC_STATUS]\n    ld   r4, [ECC_DONE]\n    wr   [FMCTRL1], #0x20\n" \
        "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    out, lines = {}, []
    for name, acc in t.page_read_steps(s):
        ref = probe(s, t, assemble(t, acc + tail), timeout_us=60000, flags=WAIT_END, scratch=bytes(1024))
        out["%s/none" % name] = ref
        row = []
        for e in range(16):
            rec = probe(s, t, assemble(t, acc + "    wait %d\n" % e + tail), timeout_us=60000, flags=WAIT_END,
                        scratch=bytes(1024))
            out["%s/%d" % (name, e)] = rec
            d = rec["elapsed_us"] - ref["elapsed_us"]
            row.append("%d:%s" % (e, "T" if rec["fault_at_us"] and rec["fault_at_us"] > ref["elapsed_us"] - 50 and
                                  d > 4000 else "%d" % max(d, 0)))
        lines.append("  %-22s %6d us, FMC_STATUS %08x ECC_DONE %x | %s" % (name, ref["elapsed_us"], ref["r"][3],
                                                                       ref["r"][4], " ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    return out, ("page read, wait N after each step (us beyond the program without the wait; T = timed out; the "
                 "step's own wait is not done yet):\n" + "\n".join(lines))


@experiment("op0916")
def ex_op0916(s, t):
    """Are 09 and 16 really illegal, or waits whose condition the sweep never met? Each with operands that would satisfy
    a register-and-mask wait (the 4G/5G WAIT has that form; the 3G WAIT ignores arg / imm), a register-field condition
    or a short delay; 07 (wait) and 1a / 20 (illegal) as references"""
    # idle values: FMC_STATUS 0x000c3000, ECC 0x800 = 0x20060808, parameter word 0xd70 = 0x12345678 (set below)
    forms = [("[0x048] & 0x1000 (bit set)", 0, 0x048, 0x1000), ("[0x048] == 0xc3000 (all set bits)", 0, 0x048, 0xc3000),
             ("[0x048] & 0x2 (bit clear)", 0, 0x048, 0x2), ("[0xd70] == 0x12345678", 0, 0xd70, 0x12345678),
             ("[0xd70] & 0x8 (bit set)", 0, 0xd70, 0x8), ("[0x800] == 0x20060808", 0, 0x800, 0x20060808),
             ("r1 = ~0, imm 1", 1, 0, 0x1), ("r1 = 0, imm 0 (until zero?)", 2, 0, 0x0),
             ("imm 1 (a short delay?)", 0, 0, 0x1), ("imm 0x100", 0, 0, 0x100), ("event field 11, imm 0", 11, 0, 0)]
    out, lines = {}, []
    for op in (0x09, 0x16, 0x07, 0x1a, 0x20):
        row = []
        for label, reg, arg, imm in forms:
            src = t.prologue(c58=0) + "    movi r1, #0xffffffff\n    movi r2, #0x0\n    movi r6, #0x0\n" \
                "    .insn 0x%02x, %d, 0x%x, 0x%x\n    movi r6, #0x600d600d\n    end\n" % (op, reg, arg, imm) + t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0xd70, 0x12345678)], timeout_us=20000, flags=WAIT_END)
            out["%02x/%s" % (op, label)] = rec
            us = rec["elapsed_us"]
            row.append("T" if rec["fault_at_us"] else "%dus" % us)
        lines.append("  op %02x: %s" % (op, " | ".join("%s %s" % (f[0], r) for f, r in zip(forms, row))))
        print(lines[-1])
        sys.stdout.flush()
    return out, ("each form: T = timed out and faulted, otherwise the run time (a pass, or a delay shorter than the "
                 "timeout):\n" + "\n".join(lines))


@experiment("op09", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_op09(s, t):
    """Is 09 'wait until the timeout' (a real instruction) or just illegal? Compared with 1a, 20 and an unmet wait:
    started with the timer C14 at 0 (after tmwait) and at 0xff, timing and state while it waits, and with a pending
    flag named in its event field or its mask"""
    out, lines = {}, []
    refs = [("09", "    .insn 0x09, 0, 0, 0\n"), ("1a", "    .insn 0x1a, 0, 0, 0\n"), ("20", "    .insn 0x20, 0, 0, 0\n"),
            ("wait 3 (unmet)", "    wait 3\n")]
    # 1. timer state: C14 forced to 0 by tmwait (from 0xff it waits the full period and leaves 0), or reloaded to 0xff
    for timer, pre in (("C14 = 0 (tmwait just before)", "    .insn 0x15, 0, 0, 0\n"),
                       ("C14 = 0xff (tmwait twice)", "    .insn 0x15, 0, 0, 0\n    .insn 0x15, 0, 0, 0\n")):
        for name, insn in refs:
            src = t.prologue(c58=0) + pre + "    ld   r2, [FMSS_C14]\n    movi r6, #0x0\n" + insn + \
                "    ld   r3, [FMSS_C14]\n    ld   r4, [CS_IRQ]\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END)
            out["%s/%s" % (timer, name)] = rec
            lines.append("  %-30s %-15s -> %-6s %6d us, fault at %5d us, C14 before %02x after %02x, CS_IRQ %02x" % (
                timer, name, rec["why"], rec["elapsed_us"], rec["fault_at_us"], rec["r"][2], rec["r"][3], rec["r"][4]))
    # 2. state while it waits, read by the CPU 1 ms in (the probe gives up at 1 ms and snapshots before stopping it)
    for name, insn in refs:
        src = t.prologue(c58=0) + "    movi r6, #0x0\n" + insn + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=1000)
        out["1 ms in/%s" % name] = rec
        a = rec["after"]
        lines.append("  %-30s %-15s -> C08 %x, C14 %02x, C3C %08x, C40 %08x, C44 %08x, C50 %08x, PC %x, CS_IRQ %02x" % (
            "state 1 ms in", name, a[0xc08], a[0xc14], a[0xc3c], a[0xc40], a[0xc44], a[0xc50], rec["pc"], rec["cs_irq"]))
    # 3. a pending flag named in 09's event field or mask
    for label, body in t.state_actions(s)[1:5]:
        ev = {"command flag (b1)": 0, "DMA flag (b20)": 2, "ECC_DONE b0": 4, "status flag (b23)": 5}[label]
        for form, insn in (("event field", "    .insn 0x09, %d, 0, 0\n" % ev),
                           ("mask", "    .insn 0x09, 0, 0, 0x%x\n" % (1 << ev)),
                           ("both", "    .insn 0x09, %d, 0x%x, 0x%x\n" % (ev, 1 << ev, 1 << ev))):
            soft_reset(s, t)
            src = t.prologue(c58=0) + t.state_setup() + body + "    movi r6, #0x0\n" + insn + \
                "    movi r6, #0x600d600d\n    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(1024))
            out["%s/%s" % (label, form)] = rec
            lines.append("  %-30s %-15s -> %s, %6d us" % (label, "09 " + form, "TIMED OUT" if rec["fault_at_us"] else
                         "PASSED", rec["elapsed_us"]))
    return out, "opcode 09 against 1a, 20 and an unmet wait:\n" + "\n".join(lines)


@experiment("op09b", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_op09b(s, t):
    """09 with event field 5 continued at once while the status flag (event 5) was pending and timed out for the other
    flags: is it a status wait with a mask test on FMC_NSTAT (the 4G/5G 'wait for ready' form), which event numbers it
    takes, does it need the flag, and does it wait for a busy chip to become ready?"""
    stat = "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait 5\n"
    tail = ("    ld   r3, [FMC_NSTAT], 0xff\n    movi r6, #0x600d600d\n    wr   [FMCTRL1], #0x20\n"
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n" + t.trap())
    out, lines = {}, []
    # A. mask forms with the status flag pending (the latched byte is in FMC_NSTAT, read just before)
    for arg, imm in ((0, 0), (0x40, 0x40), (0x20, 0x20), (0, 0x40), (0, 0x01), (1, 0x01), (0x70, 0xff), (0x71, 0xff),
                     (0x40, 0), (0xff, 0xff)):
        soft_reset(s, t)
        src = t.prologue(c58=0) + t.state_setup() + stat + "    ld   r2, [FMC_NSTAT], 0xff\n    movi r6, #0x0\n" + \
            "    .insn 0x09, 5, 0x%x, 0x%x\n" % (arg, imm) + tail
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
        out["A/arg %x imm %x" % (arg, imm)] = rec
        lines.append("  status pending, 09 ev 5 arg %-4x imm %-4x -> %-9s %6d us; NSTAT before %02x after %02x" % (
            arg, imm, "TIMED OUT" if rec["fault_at_us"] else "passed", rec["elapsed_us"], rec["r"][2], rec["r"][3]))
    # B. event field 0..15 with every flag of both sets pending
    states = dict(t.state_actions(s))
    allflags = states["all four"] + states["second set: command flag"] + states["second set: DMA flag"]
    row = []
    for ev in range(16):
        soft_reset(s, t)
        src = t.prologue(c58=0) + t.state_setup() + allflags + "    movi r6, #0x0\n    .insn 0x09, %d, 0, 0\n" % ev + tail
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(1024))
        out["B/%d" % ev] = rec
        row.append("%d:%s" % (ev, "T" if rec["fault_at_us"] else "ok"))
    lines.append("  every flag of both sets pending, 09 event field 0..15: " + " ".join(row))
    # C. the status byte latched but b23 acknowledged
    soft_reset(s, t)
    src = t.prologue(c58=0) + t.state_setup() + stat + "    wr   [FMC_STATUS], #0x800000\n    movi r6, #0x0\n" \
        "    .insn 0x09, 5, 0, 0\n" + tail
    rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
    out["C"] = rec
    lines.append("  status latched, b23 acknowledged, 09 ev 5    -> %s, %d us" % (
        "TIMED OUT" if rec["fault_at_us"] else "passed", rec["elapsed_us"]))
    # D. a busy chip: erase confirm, status read while busy, then 09 waiting for ready (b6) - or wait 5 as the reference
    for label, insn in (("09 ev 5 arg 0x40 imm 0x40", "    .insn 0x09, 5, 0x40, 0x40\n"),
                        ("09 ev 5 arg 0 imm 0", "    .insn 0x09, 5, 0, 0\n"),
                        ("reference: wait 5 only", "    wait 5\n")):
        soft_reset(s, t)
        src = t.nand_prologue(0) + t.erase + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    ld   r2, [FMC_NSTAT], 0xff\n" \
            "    movi r6, #0x0\n" + insn + tail
        rec = probe(s, t, assemble(t, src), timeout_us=50000, flags=WAIT_END)
        out["D/%s" % label] = rec
        lines.append("  erase running, status latch, %-26s -> %-9s %6d us; NSTAT right after the latch %02x, at the end "
                     "%02x" % (label, "TIMED OUT" if rec["fault_at_us"] else "passed", rec["elapsed_us"], rec["r"][2],
                               rec["r"][3]))
    return out, "opcode 09:\n" + "\n".join(lines)


@experiment("op16", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_op16(s, t):
    """Opcode 16 timed out in the opcode sweep but passed at once in every form after DMA / ECC / page-read runs: which
    condition lets it pass? Each case: soft reset, one action, then 16 (and 09 as a reference)"""
    a = t.scratch_addr(s)
    ecc = "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1101\n    wait 4\n"
    dma = ("    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
           "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 2\n    wr   [FMC_CTRL0], #@CE_NONE@\n" % a)
    cases = [("nothing (clean)", ""), ("ECC decode, ECC_DONE left", ecc),
             ("ECC decode, ECC_DONE acknowledged", ecc + "    wr   [ECC_DONE], #0x1ff\n"),
             ("DMA RAM -> buffer, flag left", dma), ("DMA, flag acknowledged", dma + "    wr   [FMC_STATUS], #0x100000\n"),
             ("70h command, flag left", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n    wait 0\n"),
             ("70h + status latch", "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait 5\n"),
             ("READ ID through the first set", t.read_id(t.ce_sel[0], 0)),
             ("one sector page read (to RAM)", t.page_read_steps(s)[-1][1])]   # with its CE select and set-up
    out, lines = {}, []
    for label, body in cases:
        row = []
        for op in (0x16, 0x09):
            soft_reset(s, t)
            src = t.prologue(c58=0) + "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n" + body + \
                "    ld   r2, [0x050]\n    ld   r3, [FMSS_C50]\n    ld   r4, [ECC_DONE]\n    ld   r5, [FMC_STATUS]\n" \
                "    movi r6, #0x0\n    .insn 0x%02x, 0, 0, 0\n    movi r6, #0x600d600d\n" \
                "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n" % op + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END, scratch=bytes(1024))
            out["%s/%02x" % (label, op)] = rec
            row.append("%02x %s" % (op, "T" if rec["fault_at_us"] else "PASS"))
            if op == 0x16:
                r16 = rec
        lines.append("  %-36s -> %s; just before 16: 0x050 %08x, C50 %08x, ECC_DONE %02x, FMC_STATUS %08x" % (
            label, ", ".join(row), r16["r"][2], r16["r"][3], r16["r"][4], r16["r"][5]))
        print(lines[-1])
        sys.stdout.flush()
    return out, "opcode 16 (and 09) after one action from a clean state:\n" + "\n".join(lines)


@experiment("op16b", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_op16b(s, t):
    """Opcode 16 jumped to the fault vector at once while a completion flag was pending, else timed out. Does it go to
    C4C whatever it holds, does it wait for a flag that arrives later, does C58 matter, is the flag cleared?"""
    a = t.scratch_addr(s)
    tail = ("    movi r6, #0x600d600d\n    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n"
            "alt:\n    ld   r2, [FMC_STATUS]\n    ld   r3, [FMSS_C50]\n    movi r6, #0xa17a17\n    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n")
    cmdlate = "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n"            # no wait: the flag comes later
    dmalate = ("    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
               "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n" % a)
    cmdp = cmdlate + "    wait 0\n"
    cases = [("flag pending, C4C = alt", cmdp, "alt", 0),
             ("flag pending, C4C = trap, C58 = 4", cmdp, "trap", 4),
             ("command written just before (flag arrives later)", cmdlate, "alt", 0),
             ("DMA started just before (~14 us)", dmalate, "alt", 0),
             ("nothing pending, C4C = alt", "", "alt", 0),
             ("flag pending, 16 twice", cmdp + "    .insn 0x16, 0, 0, 0\n", "alt", 0)]
    out, lines = {}, []
    for label, body, vec, c58 in cases:
        soft_reset(s, t)
        src = ("    wr   [CS_IRQ], #0xff\n    wr   [FMSS_C4C], #%s\n    wr   [FMSS_C58], #0x%x\n" % (vec, c58) +
               t.state_setup() + body + "    movi r6, #0x0\n    movi r2, #0x0\n    movi r3, #0x0\n"
               "    .insn 0x16, 0, 0, 0\n" + tail + t.trap())
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(1024))
        out[label] = rec
        r6 = rec["r"][6]
        where = {0x600d600d: "continued", 0xa17a17: "went to alt (C4C)", 0xfa17fa17: "went to trap"}.get(r6, "r6 %08x" % r6)
        lines.append("  %-50s -> %-18s %6d us%s; at the target FMC_STATUS %08x C50 %08x; C3C after %08x" % (
            label, where, rec["elapsed_us"], " (timeout fault)" if rec["fault_at_us"] else "", rec["r"][2], rec["r"][3],
            rec["after"][0xc3c]))
    return out, "opcode 16:\n" + "\n".join(lines)


@experiment("opstate", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_opstate(s, t):
    """Opcodes before 1a that showed nothing or only timed out (08, 09, 10, 12, 15, 16, ff): does any of them react to
    pending completion flags of either FMC set? Each case: soft reset, one state, then the opcode; C4C points to the
    handler, so a jump to the vector shows as 'vector'"""
    out, lines = {}, []
    states = t.state_actions(s)
    for op in (0x08, 0x09, 0x10, 0x12, 0x15, 0x16, 0xff):
        row = []
        for label, body in states:
            soft_reset(s, t)
            src = t.prologue(c58=0) + t.state_setup() + body + "    ld   r3, [FMSS_C50]\n    movi r6, #0x0\n" \
                "    .insn 0x%02x, 0, 0, 0\n    movi r6, #0x600d600d\n    wr   [FMCTRL1], #0x20\n" \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n    wr   [0x400], #@CE_NONE@\n    end\n" % op + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(1024))
            out["%02x/%s" % (op, label)] = rec
            r6 = rec["r"][6]
            what = ("vector" if r6 == 0xfa17fa17 else "on" if r6 == 0x600d600d else
                    "stopped" if rec["why"] != "TIMEOUT" else "blocked")
            row.append("%s:%s%s" % (label.split(" ")[0] if label != "all four" else "all", what,
                                    " T" if rec["fault_at_us"] else ""))
        lines.append("  op %02x | %s" % (op, " | ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    return out, ("after each state: on = continued with the next instruction, vector = went to the fault vector (C4C), "
                 "T = a timeout fault happened, blocked = no end within 20 ms\n" + "\n".join(lines))


@experiment("op09c", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_op09c(s, t):
    """09 passed with event field 5 in every state tried and timed out with 0..15 otherwise: from a clean state with
    nothing pending, which event numbers 0..31 let it pass, with no chip selected and with CE0 / CE1 selected (wait 0..15
    in the same states as the reference)?"""
    out, lines = {}, []
    for label, ctl in (("no chip selected", t.ce_sel[None]), ("CE0 selected", t.ce_sel[0]), ("CE1 selected", t.ce_sel[1])):
        for op, rng in ((0x09, range(32)), (0x07, range(16))):
            row = []
            for ev in rng:
                soft_reset(s, t)
                src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #0x%x\n    movi r6, #0x0\n" % ctl + \
                    "    .insn 0x%02x, %d, 0, 0\n    movi r6, #0x600d600d\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
                    "    end\n" % (op, ev) + t.trap()
                rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
                out["%s/%02x/%d" % (label, op, ev)] = rec
                if not rec["fault_at_us"]:
                    row.append(str(ev))
            lines.append("  %-17s %s passes for: %s" % (label, "09" if op == 9 else "wait", " ".join(row) or "none"))
            print(lines[-1])
            sys.stdout.flush()
    return out, "nothing pending (soft reset first), event numbers that pass:\n" + "\n".join(lines)
