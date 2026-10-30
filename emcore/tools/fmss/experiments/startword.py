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

"""The start word's bits and the pause opcode: what each FMSS_CTRL bit enables, and how the CPU continues a paused
program."""
import struct
import sys

from ..probe import probe, assemble, soft_reset, WAIT_END
from .registry import experiment


@experiment("op12cont", needs=())
def ex_op12cont(s, t):
    """Opcode 12 blocks with CS_IRQ b3. The 4G/5G driver treats CS_IRQ b3 as 'pause' and continues the program with
    FMSS_PC += 8, CS_IRQ = 8, FMSS_CTRL |= 1. Does that sequence (or a part of it) continue it here?"""
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r2, #0x0\npause:\n    .insn 0x12, 0, 0, 0\n" \
        "    movi r2, #0x2222\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    code = assemble(t, src)
    mark = struct.pack("<II", 0x12000000, 0)
    at = next(i for i in range(0, len(code), 8) if code[i:i + 8] == mark)
    start = 0xfff5
    cases = [("none (control)", []),
             ("CS_IRQ = 8, start", [(0xc0c, 8), (0xc00, start)]),
             ("start, CS_IRQ = 8", [(0xc00, start), (0xc0c, 8)]),
             ("PC = pause + 8, CS_IRQ = 8, start (the 4G/5G sequence)", [(0xc38, at + 8), (0xc0c, 8), (0xc00, start)]),
             ("PC = pause + 8, start", [(0xc38, at + 8), (0xc00, start)]),
             ("CS_IRQ = 8, start 0x1", [(0xc0c, 8), (0xc00, 0x1)]),
             ("CS_IRQ = 8, start, start", [(0xc0c, 8), (0xc00, start), (0xc00, start)])]
    out, lines = {}, []
    for label, late in cases:
        rec = probe(s, t, code, late=late, late_us=2000, timeout_us=20000)
        out[label] = rec
        r2, r6 = rec["r"][2], rec["r"][6]
        how = ("CONTINUED after the pause" if r6 == 0x600d600d and r2 == 0x2222 else
               "ran again from the start?" if r6 == 0x600d600d else "still paused" if rec["why"] == "TIMEOUT" else
               "r6 %08x" % r6)
        lines.append("  %-52s -> %-7s %-26s %6d us, PC %x, C08 %x, CS_IRQ %08x" % (
            label, rec["why"], how, rec["elapsed_us"], rec["pc"], rec["after"][0xc08], rec["cs_irq"]))
    return out, "opcode 12 at offset 0x%x, CPU writes 2 ms after the start (in this order):\n" % at + "\n".join(lines)


@experiment("ctrlfx", needs=("start_value", "nand_seq"))
def ex_ctrlfx(s, t):
    """What do the start-word bits 2 and 4..15 enable? Each start word on four behaviours an enable could gate: a wait
    that times out, an illegal opcode, evjmp with a flag pending, and the CPU interrupt line with the FMC
    command-flag source enabled (0x044 b0)"""
    words = [0x1, 0xfff5, 0xffb5] + [0x1 | 1 << k for k in [2] + list(range(4, 16))]
    tests = [("timeout", "    wait 3\n", []),
             ("illegal", "    .insn 0x1a, 0, 0, 0\n", []),
             ("evjmp", t.nand_prologue(0).replace(t.prologue(c58=0), "") + "    wr   [FMC_CMD], #0x70\n    wait 0\n"
              "    .insn 0x16, 0, 0, 0\n", []),
             ("irq", t.nand_prologue(0).replace(t.prologue(c58=0), "") + "    wr   [FMC_CMD], #0x70\n    wait 0\n",
              [(0x044, 1)])]
    out, lines = {}, []
    for w in words:
        row = []
        for name, body, params in tests:
            src = t.prologue(c58=0) + "    movi r6, #0x0\n" + body + "    movi r6, #0x600d600d\n" \
                "    wr   [FMC_CTRL0], #0x43801\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), params=params, timeout_us=20000, start_value=w, flags=WAIT_END)
            out["0x%04x/%s" % (w, name)] = rec
            r6 = rec["r"][6]
            if name == "irq":
                cell = "line %d/%d CS_IRQ %02x" % (rec["irq_seen"], rec["irq_end"], rec["cs_irq"] & 0xff)
                probe(s, t, assemble(t, "    end\n"), params=[(0x044, 0)], timeout_us=20000, quiet=True)
            else:
                cell = "%s%s" % ("vector" if r6 == 0xfa17fa17 else "on" if r6 == 0x600d600d else rec["why"],
                                 " T%d" % round(rec["elapsed_us"] / 4870.0) if rec["fault_at_us"] else "")
            row.append("%s: %s" % (name, cell))
        lines.append("  start 0x%04x | %s" % (w, " | ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, ("on = continued, vector = went to C4C, T<n> = n timeout faults; irq: VIC line seen while polling / at "
                 "the end:\n") + "\n".join(lines)


@experiment("op09d", needs=("nand_seq", "second_fmc_set"))
def ex_op09d(s, t):
    """Is wait2 (opcode 09) the wait on the second FMC set's flags? Each flag of the second set (0x400) pending on its own
    (command, address, transfer, DMA, status latch), then wait2 0..15 and wait 0..15 in that state"""
    a = t.scratch_addr(s)
    pre = ("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
           "    wr   [FMC_CTRL0], #@CE0@\n")
    cmd = "    wr   [FMC_CMD], #0x70\n    wait 6\n"
    adr = ("    wr   [FMC_CMD], #0x90\n    wait 6\n    wr   [FMC_STATUS], #0x2\n    wr   [FMC_ANUM], #0x0\n"
           "    wr   [FMC_ADDR0], #0x0\n    wr   [FMCTRL1], #0x1\n    wait 7\n")
    xfer = (adr + "    wr   [FMC_STATUS], #0x4\n    wr   [FMC_DNUM], #0x7\n    wr   [FMC_DATA_SEL], #0x1\n"
            "    wr   [FMCTRL1], #0x1c2\n    wait 9\n")
    dma = ("    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
           "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 8\n    wr   [FMC_CTRL0], #@CE_NONE@\n" % a)
    stat = cmd + "    wr   [FMC_STATUS], #0x2\n    wr   [FMCTRL1], #0xca\n"
    states = [("nothing pending", ""), ("command flag (b1)", cmd), ("address flag (b2)", adr.replace(
        "    wr   [FMC_STATUS], #0x2\n", "    wr   [FMC_STATUS], #0x2\n", 1)), ("transfer flag (b3)", xfer),
              ("DMA flag (b20)", dma), ("status latch (b23?)", stat)]
    out, lines = {}, []
    for label, body in states:
        setup = t.ch_regs(pre + body, 0x400)
        for op, name in ((0x09, "wait2"), (0x07, "wait")):
            ok = []
            for ev in range(16):
                soft_reset(s, t)
                src = t.prologue(c58=0) + setup + "    ld   r2, [0x448]\n    movi r6, #0x0\n" \
                    "    .insn 0x%02x, %d, 0, 0\n    movi r6, #0x600d600d\n    wr   [0x400], #@CE_NONE@\n    end\n" % (
                        op, ev) + t.trap()
                rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(1024))
                out["%s/%s/%d" % (label, name, ev)] = rec
                if not rec["fault_at_us"]:
                    ok.append(str(ev))
            lines.append("  %-22s 0x448 %08x  %-5s passes for: %s" % (label, rec["r"][2], name, " ".join(ok) or "none"))
            print(lines[-1])
            sys.stdout.flush()
    return out, "second FMC set (0x400), one flag pending at a time (waits inside the set-up that time out are part " \
        "of it; the sweep is the last wait):\n" + "\n".join(lines)
