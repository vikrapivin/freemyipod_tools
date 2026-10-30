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

"""Experiments that drive the NAND controller: the second FMC register set and DMA without a chip."""
from ..probe import (probe, assemble, F_WIN400, WAIT_END, CH1_PINS)
from .registry import experiment


@experiment("chan1", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_chan1(s, t):
    """Is the FMC register set at +0x400 a second NAND channel? READ ID through it, per chip enable, against the same
    READ ID through the first set (timed-out waits continue: C58 = 0)"""
    out, lines = {}, []
    cases = [("set 0x000, CE0 (control)", 0x000, t.ce_sel[0]), ("set 0x000, CE1", 0x000, t.ce_sel[1]),
             ("set 0x000, CE2", 0x000, t.ce_word(2)), ("set 0x000, CE3", 0x000, t.ce_word(3))]
    cases += [("set 0x400, CE%d" % k, 0x400, t.ce_word(k)) for k in range(8)]
    for label, base, ce in cases:
        src = t.prologue(c58=0) + "    movi r2, #0x0\n    movi r3, #0x0\n" + t.read_id(ce, base) + \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END | F_WIN400)
        out[label] = rec
        lines.append("  %-20s -> %-6s %6d us, %d timeouts, ID %08x %08x, status %08x, CS_IRQ %08x" % (
            label, rec["why"], rec["elapsed_us"], round(rec["elapsed_us"] / 4870.0), rec["r"][2], rec["r"][3],
            rec["r"][4], rec["cs_irq"]))
    return out, "READ ID (90h 00h, 8 bytes) through each FMC register set:\n" + "\n".join(lines)


@experiment("chan1pins", needs=("nand_seq", "second_fmc_set", "ch1_pins", "win400"))
def ex_chan1pins(s, t):
    """FMC channel 1 (the 0x400 set) has its own pins, which emCORE leaves unswitched (the boot ROM switches them for a
    channel-1 NAND boot). READ ID through it on CE0..7 with those pins switched for the run; the first set's READ ID
    with and without the switch as a control"""
    out, lines = {}, []
    cases = [("set 0x000, CE0, pins as emCORE leaves them", 0x000, t.ce_sel[0], 0),
             ("set 0x000, CE0, channel-1 pins switched", 0x000, t.ce_sel[0], CH1_PINS),
             ("set 0x400, CE0, pins as emCORE leaves them", 0x400, t.ce_word(0), 0)]
    cases += [("set 0x400, CE%d, channel-1 pins switched" % k, 0x400, t.ce_word(k), CH1_PINS) for k in range(8)]
    for label, base, ce, fl in cases:
        src = t.prologue(c58=0) + "    movi r2, #0x0\n    movi r3, #0x0\n" + t.read_id(ce, base) + \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END | F_WIN400 | fl)
        out[label] = rec
        lines.append("  %-44s -> %-6s %6d us, %d timeouts, ID %08x %08x, status %08x" % (
            label, rec["why"], rec["elapsed_us"], round(rec["elapsed_us"] / 4870.0), rec["r"][2], rec["r"][3],
            rec["r"][4]))
    return out, "READ ID (90h 00h, 8 bytes), FMC channel 1 pins switched or not:\n" + "\n".join(lines)


@experiment("dmaloop", needs=("nand_seq", "second_fmc_set", "win400"))
def ex_dmaloop(s, t):
    """DMA without a chip, through each FMC register set: RAM -> controller data buffer (FMCTRL1 0x2e0) -> RAM
    (0x100, or 0x1e0 after DMA_CTRL = 7). Does the 0x400 set have its own buffers and DMA, which event says DMA done,
    and does a DMA need FMC_CTRL0 bit 24 (the read programs set it around their DMA; the probes did not)?"""
    base = t.scratch_addr(s)
    src, dst = base + 0x1000, base + 0x2000
    pat = bytes((i * 13 + 7) & 0xff for i in range(512))
    scratch = bytes(0x1000) + pat + bytes(0x1000 - 512) + b"\xee" * 512
    out, lines = {}, []
    d100, d1e0 = "    wr   [FMCTRL1], #0x100\n", "    wr   [FMC_DMA_CTRL], #0x7\n    wr   [FMCTRL1], #0x1e0\n"
    cases = []
    for ctl, tag in ((t.ce_sel[None], "CTRL0 0x%x" % t.ce_sel[None]), (t.ce_sel[None] | t.ce_dma, "CTRL0 + b24")):
        cases += [("set 0x000, %s, drain 0x100" % tag, 0x000, 2, d100, ctl),
                  ("set 0x000, %s, DMA_CTRL 7, drain 0x1e0" % tag, 0x000, 2, d1e0, ctl),
                  ("set 0x400, %s, drain 0x100, wait 8" % tag, 0x400, 8, d100, ctl),
                  ("set 0x400, %s, drain 0x100, wait 2" % tag, 0x400, 2, d100, ctl)]
    for label, fb, ev, drain, ctl in cases:
        body = ("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n    wr   [FMC_CTRL0], #0x%x\n" % ctl +
                "    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n    wr   [FMC_DMA_ADDR], #0x%x\n"
                "    wr   [FMCTRL1], #0x2e0\n    wait %d\n    ld   r1, [CS_IRQ], 0x4\n    wr   [CS_IRQ], #0x4\n"
                "    ld   r4, [FMC_STATUS]\n    wr   [FMC_STATUS], #0x100000\n"
                "    wr   [FMC_DATA_SEL], #0x100\n    wr   [FMC_DMA_ADDR], #0x%x\n" % (src, ev, dst) + drain +
                "    wait %d\n    ld   r2, [CS_IRQ], 0x4\n    wr   [CS_IRQ], #0x4\n    ld   r5, [FMC_STATUS]\n"
                "    wr   [FMC_STATUS], #0x100000\n    wr   [FMC_CTRL0], #@CE_NONE@\n" % ev)
        prog = t.prologue(c58=0) + t.ch_regs(body.replace("[FMC_DMA_CTRL]", "[0x038]"), fb) + \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        if fb:
            prog = prog.replace("[0x038]", "[0x438]")
        rec = probe(s, t, assemble(t, prog), timeout_us=60000, flags=WAIT_END | F_WIN400, scratch=scratch,
                    scratch_read=0x2200)
        sc = rec.pop("scratch") or b""
        got = sc[0x2000:0x2200]
        rec["dst"] = got.hex()
        same = sum(1 for a, b in zip(got, pat) if a == b)
        lines.append("  %-42s -> fill %s, drain %s; destination %d/512 bytes match%s; status after fill %08x, "
                     "after drain %08x; DMA_ADDR %08x / %08x" % (
                         label, "T" if rec["r"][1] else "ok", "T" if rec["r"][2] else "ok", same,
                         " (untouched)" if got == b"\xee" * 512 else "", rec["r"][4], rec["r"][5],
                         rec["after"][0x034], rec["after"][0x434]))
        out[label] = rec
    return out, "RAM -> buffer -> RAM through each register set (ok = the wait passed, T = timed out):\n" + \
        "\n".join(lines)
