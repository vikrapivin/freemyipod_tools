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

"""Round 25: the open points of the FMSS documentation that a device run can settle. The sequence value of the first
encode of the session, wait2 conditions 33 / 20 against the controller state, C50 bit 8, the C58 bit 1 one-shot
without the job's clean-up, ECC_DONE bit 5, ECC_RESULT after a Reed-Solomon encode, st with register numbers above 7,
the 0x044 bits per operation, tmwait, Reed-Solomon on the chip, miscorrection, whole 4 KiB pages, FMC_DMA_CTRL and
combined data selects, FMC_STATUS bits 12/13/18/19, the second set's buffers, wait events during more operations,
the 0x100 block and 0x3a8, and the boot strap pins. NAND writes are allowed (scratch blocks 3051, 3070)."""
import random
import struct
import sys

from ..probe import probe, assemble, soft_reset, WAIT_END, FULLMAP, CH1_PINS, S_NO_CLEAN, S_END_NOCLEAN
from .registry import experiment
from .followup import _dma_in, _dma_out, _kick, _ready, _address, _send, _flip
from .probes3 import _program_page, _read_page, _erase, _sweep_src, END

RAMP = bytes((i * 37 + 11) & 0xff for i in range(512))


def _ecc_prog(s, t, body, scratch, read=0x800, params=(), timeout=60000):
    """run a program that starts with the prologue, the set-up word and DMA enable; returns (rec, scratch bytes)"""
    src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + body + \
        "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
    rec = probe(s, t, assemble(t, src), params=list(params), timeout_us=timeout, flags=WAIT_END,
                scratch=scratch, scratch_read=read, quiet=True)
    return rec, rec["scratch"] or bytes(read)


def _passes(sc, order):
    vals = struct.unpack("<%dI" % len(order), sc[:4 * len(order)])
    return sorted(order[i] for i in range(len(order)) if vals[i] == 0), [order[i] for i in range(len(order)) if vals[i] not in (0, 4)]


# ---------------------------------------------------------------- first in the session
@experiment("seqfirst", needs=("nand_seq",))
def ex_seqfirst(s, t):
    """The sequence of the ECC modes without a code restarts at some event between device runs. One encode in mode 0,
    as the very first ECC operation of the session (run it first, right after a power cycle or an emCORE reload, as the
    run notes say): which value of the sequence comes out?"""
    a, e = t.scratch_addr(s), t.events
    src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
        _kick(0x1102, 0x0, e["ecc"]) + "".join("    ld   r%d, [0x%x]\n" % (1 + j, 0x900 + 4 * j) for j in range(4)) + \
        "    ld   r5, [0x990]\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=RAMP, quiet=True)
    area = b"".join(struct.pack("<I", v) for v in rec["r"][1:5])
    return {"rec": rec}, "first mode-0 encode of the session: area %s 0x990 %08x (%s)" % (area.hex()[:26], rec["r"][5], rec["why"])


@experiment("w2state", needs=("nand_seq", "start_value", "ch1_pins"))
def ex_w2state(s, t):
    """wait2 condition 33 was true in every state of runs 29..34 and in none of run 35; condition 20 was true once (every
    interrupt enable on, a command flag pending). wait events 0..15 and wait2 conditions 0..255 (short timeout,
    start word 0x3f15) in the first program of the session and then after one change of state at a time: a soft reset,
    0x044 = 0xa5a5 with 0x844 = 0x25 left set, each 0x044 bit 0..15 alone with a command flag pending, each
    FMC_STATUS / ECC_DONE flag left pending, a second-set READ ID, the channel-1 pins switched, a page read, C10 = 0 /
    0xff, no chip / CE0 / CE1 selected, and the start word 0x3ff5"""
    a, e = t.scratch_addr(s), t.events
    order = [("wait", n) for n in range(16)] + [("wait2", n) for n in range(256)]
    out, lines, base = {}, [], [None]

    def sweep(label, pre="", params=(), flags=0, start=0x3f15, reset=False):
        if reset:
            soft_reset(s, t)
        src = t.prologue(c58=0) + pre + "    movi r2, #0x%x\n" % a + _sweep_src(order) + \
            "    wr   [0x044], #0x0\n    wr   [0x844], #0x0\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), params=list(params), timeout_us=400000, start_value=start,
                    flags=WAIT_END | flags, scratch=b"\xee" * (4 * len(order)), scratch_read=4 * len(order), quiet=True)
        ok, bad = _passes(rec["scratch"] or bytes(4 * len(order)), order)
        out[label] = rec
        w = [n for k, n in ok if k == "wait"]
        w2 = [n for k, n in ok if k == "wait2"]
        lines.append("  %-52s wait %s; wait2 %s%s" % (label, w, w2, "; NOT STORED %d" % len(bad) if bad else ""))
        print(lines[-1])
        sys.stdout.flush()
    cmdp = "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"]
    sweep("first program of the session")
    sweep("again, no reset")
    sweep("after a soft reset", reset=True)
    sweep("0x044 = 0xa5a5, 0x844 = 0x25 (CPU, before)", params=[(0x044, 0xa5a5), (0x844, 0x25)], reset=True)
    for bit in range(16):
        sweep("command flag pending, 0x044 bit %d" % bit, pre=cmdp, params=[(0x044, 1 << bit)], reset=True)
    sweep("command flag pending, 0x844 = 0x2f", pre=cmdp, params=[(0x844, 0x2f)], reset=True)
    flags = [("address flag", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n"
              "    wr   [FMCTRL1], #0x1\n    wait %d\n" % e["addr"]),
             ("DMA flag", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a + 0x800, e["dma"]).replace("    wr   [FMC_STATUS], #0x100000\n", "")),
             ("ECC_DONE", "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1102\n    wait %d\n" % e["ecc"]),
             ("status flag", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) +
              "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"])]
    for label, pre in flags:
        sweep(label + " pending", pre=t.state_setup() + pre, reset=True)
    sweep("after a second-set READ ID", pre=t.read_id(t.ce_sel[0], 0x400).replace("wait 0", "wait 6")
          .replace("wait 1\n", "wait 7\n").replace("wait 3\n", "wait 9\n"), reset=True)
    sweep("channel-1 pins switched", flags=CH1_PINS, reset=True)
    page = t.page_read_steps(s)[-1][1]
    sweep("after a page read of CE0", pre=page + "    wr   [FMC_CTRL0], #@CE_NONE@\n", reset=True)
    for v in (0, 0xff):
        sweep("C10 = 0x%x" % v, pre="    wr   [0xc10], #0x%x\n" % v, reset=True)
    for label, w in (("no chip selected", "@CE_NONE@"), ("CE0 selected", "@CE0@"), ("CE1 selected", "0x%x" % t.ce_sel[1])):
        sweep(label, pre="    wr   [FMC_CTRL0], #%s\n" % w, reset=True)
    sweep("start word 0x3ff5 (step n = 15)", start=0x3ff5, reset=True)
    soft_reset(s, t)
    return out, "wait / wait2 passes per controller state (short timeout):\n" + "\n".join(lines)


@experiment("c50b8", needs=("nand_seq",))
def ex_c50b8(s, t):
    """C50 bit 8 was seen once after a soft reset (0xffff1900). C50 read by the CPU before and after a run, around: the
    first runs of the session, a soft reset, a sequencer reset alone (FMSS_CTRL = 0x30000), a run that faults, a run
    with a second-set command, a run with an ECC decode, ten END-only runs in a row"""
    e = t.events
    out, lines = {}, []
    end = assemble(t, "    end\n")
    cases = [("END only (1)", end, [], 0), ("END only (2)", end, [], 0), ("soft reset, then END", None, [], 0),
             ("sequencer reset, then END", end, [(0xc00, 0x30000)], 0),
             ("fault (wait 10, C58 0)", assemble(t, t.prologue(c58=0) + "    wait 10\n" + END + t.trap()), [], 0),
             ("second-set command", assemble(t, t.prologue(c58=0) + "    wr   [0x404], #@FMCTRL1_INIT@\n    wr   [0x400], #@CE0@\n"
                                             "    wr   [0x408], #0x70\n    wait %d\n    wr   [0x400], #@CE_NONE@\n" % e["set1_cmd"] + END + t.trap()), [], 0),
             ("ECC decode", assemble(t, t.prologue(c58=0) + _kick(0x1101, 0x180, e["ecc"]) + END + t.trap()), [], 0)]
    cases += [("END only, repeat %d" % k, end, [], 0) for k in range(10)]
    for label, code, params, fl in cases:
        if code is None:
            soft_reset(s, t)
            code = end
        rec = probe(s, t, code, params=params, timeout_us=30000, flags=WAIT_END | fl, quiet=True)
        out[label] = rec
        lines.append("  %-30s C50 before %08x after %08x" % (label, rec["before"][0xc50], rec["after"][0xc50]))
    return out, "C50 around runs (bit 8 = 0x100):\n" + "\n".join(lines)


# ---------------------------------------------------------------- sequencer control
@experiment("c58b2", needs=("nand_seq",))
def ex_c58b2(s, t):
    """C58 bit 1 (an END ran the fault vector once in run 8; never in run 35, where the job's park wrote C58, C4C and C5C
    = 0 after every run). The job's clean-up is switched off before and after the run, so C58 stays as written for
    three runs in a row. Cases: CPU writes C58 = 2 then C4C; C4C then C58 = 2; C58 = 2 with the program writing C4C
    (the run-8 control case, also with the job's normal steps); the program writing C58 = 2 and C4C; C58 = 3 and 6"""
    vec = 0x40
    code = assemble(t, "    movi r0, #0x57a27000\n    end\n" + "    nop\n" * 6 + "    movi r1, #0x3a2c3a2c\n    end\n")
    prog_c4c = assemble(t, "    wr   [FMSS_C4C], #mark\n    movi r0, #0x57a27000\n    end\nmark:\n    movi r1, #0x3a2c3a2c\n    end\n")
    prog_both = assemble(t, "    wr   [FMSS_C4C], #mark\n    wr   [FMSS_C58], #0x2\n    movi r0, #0x57a27000\n    end\nmark:\n"
                            "    movi r1, #0x3a2c3a2c\n    end\n")
    nocl = S_NO_CLEAN | S_END_NOCLEAN
    zero = [(0xc58, 0), (0xc4c, 0), (0xc5c, 0)]
    cases = [("CPU C58=2, C4C", [(0xc58, 2), (0xc4c, vec)], code, nocl),
             ("CPU C4C, C58=2", [(0xc4c, vec), (0xc58, 2)], code, nocl),
             ("CPU C58=2, program C4C (run-8 control, no clean-up)", [(0xc58, 2)], prog_c4c, nocl),
             ("CPU C58=2, program C4C (run-8 control, job steps)", [(0xc58, 2)], prog_c4c, 0),
             ("program C58=2 and C4C", [], prog_both, nocl),
             ("CPU C58=3, C4C", [(0xc58, 3), (0xc4c, vec)], code, nocl),
             ("CPU C58=6, C4C", [(0xc58, 6), (0xc4c, vec)], code, nocl)]
    out, lines = {}, []
    for label, params, prog, steps in cases:
        probe(s, t, assemble(t, "    end\n"), params=zero, clean=False, timeout_us=20000, quiet=True)
        res = []
        for k in range(3):
            rec = probe(s, t, prog, params=(params if k == 0 else []) + [(0xc18, 0), (0xc1c, 0)], clean=False,
                        steps=steps, timeout_us=20000, flags=WAIT_END, quiet=True)
            out["%s/%d" % (label, k)] = rec
            r0, r1 = rec["r"][0] == 0x57a27000, rec["r"][1] == 0x3a2c3a2c
            res.append("%s (C58 %x C4C %x C5C %x)" % ({(True, False): "END", (True, True): "END, then the vector",
                                                     (False, True): "vector only", (False, False): "neither"}[(r0, r1)],
                                                    rec["before"][0xc58], rec["before"][0xc4c], rec["before"][0xc5c]))
        lines.append("  %-52s %s" % (label, " | ".join(res)))
    probe(s, t, assemble(t, "    end\n"), params=zero, clean=False, timeout_us=20000, quiet=True)
    return out, "C58 bit 1, three runs per case (C58 / C4C / C5C as read before each start):\n" + "\n".join(lines)


@experiment("stprev", needs=("nand_seq", "pld_pst"))
def ex_stprev(s, t):
    """st with a register number above 7 stored 0 after movi r6, #0 (run 15) and r7's value after movi r7 (run 35). The
    instruction before `st [0xd70], r9` is varied (registers seeded with distinct values first): movi to r0 / r3 / r7,
    add to r2, mov, ld from a window word, pld, ldi, a wr to the window, an st of r5, a wait that passes, nop, and an
    st with r9 directly after the seeding"""
    a = t.scratch_addr(s)
    seed = "".join("    movi r%d, #0x%x\n" % (i, 0x11000000 * (i + 1) + 0x1111 * (i + 1)) for i in range(8))
    cases = [("seed only (last write movi r7)", ""), ("movi r0, #0xa0a0a0a0", "    movi r0, #0xa0a0a0a0\n"),
             ("movi r3, #0x33333333", "    movi r3, #0x33333333\n"), ("movi r7, #0x77777777", "    movi r7, #0x77777777\n"),
             ("add r2, r2, #1", "    add  r2, r2, #0x1\n"), ("mov r4, r1", "    mov  r4, r1\n"),
             ("ld r5, [0xd74] (holds 0x5d5d5d5d)", "    ld   r5, [0xd74]\n"),
             ("pld r6, [r0] (r0 = 0xd78, holds 0x6e6e6e6e)", "    movi r0, #0xd78\n    pld  r6, [r0]\n"),
             ("ldi r1, [r0] (scratch 0xcafe0001)", "    movi r0, #0x%x\n    ldi  r1, [r0]\n" % a),
             ("wr [0xd7c], #0x7c7c7c7c", "    wr   [0xd7c], #0x7c7c7c7c\n"), ("st [0xd7c], r5", "    st   [0xd7c], r5\n"),
             ("wait 11 (passes)", "    wait 11\n"), ("nop", "    nop\n")]
    out, lines = {}, []
    names = {0x11000000 * (i + 1) + 0x1111 * (i + 1): "r%d seed" % i for i in range(8)}
    for label, body in cases:
        for n in (9, 14, 200):
            src = t.prologue(c58=0) + seed + body + "    .insn 0x02, %d, 0xd70, 0\n" % n + END + t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0xd70, 0xeeeeeeee), (0xd74, 0x5d5d5d5d), (0xd78, 0x6e6e6e6e)],
                        timeout_us=20000, flags=WAIT_END, scratch=struct.pack("<I", 0xcafe0001), quiet=True)
            out["%s/%d" % (label, n)] = rec
            v = rec["after"][0xd70]
            lines.append("  N=%-3d after %-44s st stored %08x (%s)" % (n, label, v, names.get(v, "")))
    return out, "st [0xd70], rN with N above 7, by the instruction before it:\n" + "\n".join(lines)


@experiment("tmwait2", needs=("nand_seq",))
def ex_tmwait2(s, t):
    """Two tmwait in one program took 4841, 18 and 3250 us in three runs. C14 is read at the start, after each of four
    tmwait, and the program's time is recorded; the same program three times in a row, then after a run with a
    timeout, then after a soft reset"""
    rd = "    ld   r%d, [FMSS_C14]\n"
    src = t.prologue(c58=0) + rd % 1 + "    tmwait\n" + rd % 2 + "    tmwait\n" + rd % 3 + "    tmwait\n" + rd % 4 + \
        "    tmwait\n" + rd % 5 + END + t.trap()
    code = assemble(t, src)
    out, lines = {}, []
    for label, pre in (("1", None), ("2", None), ("3", None), ("after a timeout run", "tmo"), ("after a soft reset", "reset")):
        if pre == "tmo":
            probe(s, t, assemble(t, t.prologue(c58=0) + "    wait 10\n" + END + t.trap()), timeout_us=20000, flags=WAIT_END, quiet=True)
        elif pre == "reset":
            soft_reset(s, t)
        rec = probe(s, t, code, timeout_us=60000, flags=WAIT_END, quiet=True)
        out[label] = rec
        lines.append("  run %-22s C14 before (CPU) %02x | start %02x, after each tmwait %02x %02x %02x %02x | %d us" % (
            label, rec["before"][0xc14], *[rec["r"][i] & 0xff for i in range(1, 6)], rec["elapsed_us"]))
    return out, "tmwait and C14:\n" + "\n".join(lines)


# ---------------------------------------------------------------- interrupts
@experiment("irqop", needs=("nand_seq",))
def ex_irqop(s, t):
    """Which operation each 0x044 bit takes into CS_IRQ bit 4, and wait event 15. For 0x044 bits 0, 1, 2, 11, 12 (and
    none), one operation per run whose flag is left pending (command, address, 16-byte data read, status latch, DMA):
    CS_IRQ bit 4, VIC0 line 20, and whether `wait` event 15 passes (C58 0, the fault flag read after it)"""
    a, e = t.scratch_addr(s), t.events
    ops = [("command", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"]),
           ("address", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n"
            "    wr   [FMCTRL1], #0x1\n    wait %d\n" % e["addr"]),
           ("data read", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x10\n"
            "    wr   [FMC_24], #0x0\n    wr   [FMCTRL1], #0x32\n    wait 3\n"),
           ("status latch", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) +
            "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"]),
           ("DMA", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]).replace("    wr   [FMC_STATUS], #0x100000\n", ""))]
    out, lines = {}, []
    for bit in (None, 0, 1, 2, 11, 12):
        row = []
        for label, body in ops:
            soft_reset(s, t)
            print("irqop: 0x044 %s, %s ..." % ("none" if bit is None else "bit %d" % bit, label))
            sys.stdout.flush()
            src = t.prologue(c58=0) + t.state_setup() + body + "    wr   [CS_IRQ], #0x4\n    wait 15\n    ld   r3, [CS_IRQ], 0x4\n" \
                "    wr   [FMCTRL1], #0x20\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n    wr   [0x044], #0x0\n    wr   [0x844], #0x0\n" \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0x044, 0 if bit is None else 1 << bit)], timeout_us=40000,
                        flags=WAIT_END, scratch=bytes(512), quiet=True)
            out["%s/%s" % (bit, label)] = rec
            row.append("%s: b4 %d V0.20 %d ev15 %s" % (label, rec["cs_irq"] >> 4 & 1, rec["vic0_seen"] >> 20 & 1,
                                                        "pass" if rec["r"][3] == 0 else "timeout"))
        lines.append("  %-9s %s" % ("none" if bit is None else "bit %d" % bit, " | ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "0x044 bit x operation (flag left pending): CS_IRQ bit 4, VIC0 line 20, wait event 15:\n" + "\n".join(lines)


# ---------------------------------------------------------------- ECC
@experiment("done5b", needs=("nand_seq",))
def ex_done5b(s, t):
    """ECC_DONE bit 5 with ECC_MODE bits 21, 22, 23 alone on 0x180 read 0x33 in run 30 (encode only) and 0x13 in run 35
    (encode, then decode, in one program). Each of bits 21..24: encode only; encode then decode; a decode first, then
    the encode; each after a soft reset"""
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    for bit in (None, 21, 22, 23, 24):
        mode = 0x180 | (0 if bit is None else 1 << bit)
        row = []
        for label, body in (("enc", _dma_in(0x1, a, e["dma"]) + _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n"),
                            ("enc+dec", _dma_in(0x1, a, e["dma"]) + _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n" +
                             _kick(0x1101, mode, e["ecc"]) + "    ld   r5, [ECC_DONE]\n"),
                            ("dec(0x180)+enc", _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x180, e["ecc"]) +
                             _kick(0x1101, 0x180, e["ecc"]) + _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n")):
            soft_reset(s, t)
            rec, _ = _ecc_prog(s, t, body, RAMP, params=[(0xc28, 0xee), (0xc2c, 0xee)])
            out["%s/%s" % (bit, label)] = rec
            row.append("%s %02x%s" % (label, rec["r"][4], "/%02x" % rec["r"][5] if label == "enc+dec" else ""))
        lines.append("  0x%07x: %s" % (mode, ", ".join(row)))
    soft_reset(s, t)
    return out, "ECC_DONE (encode / decode) per mode, each case after a soft reset:\n" + "\n".join(lines)


@experiment("rsresult", needs=("nand_seq",))
def ex_rsresult(s, t):
    """A Reed-Solomon encode (0x8180) left ECC_RESULT = 0x20000 where BCH encodes left the earlier value. ECC_RESULT
    after an RS encode (8, 12, 16 symbols) that follows: a BCH decode with 0 and with 3 corrected bits; an RS decode
    with 0 and with 2 corrected bytes; nothing (after a soft reset)"""
    a, e = t.scratch_addr(s), t.events
    bad3 = _flip(RAMP, [5, 1000, 3000])
    bad2 = bytearray(RAMP); bad2[10] ^= 0xff; bad2[300] ^= 0x0f; bad2 = bytes(bad2)
    pre = {"nothing": "",
           "BCH decode, 0 bits": _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x180, e["ecc"]) + _kick(0x1101, 0x180, e["ecc"]),
           "BCH decode, 3 bits": _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x180, e["ecc"]) + _dma_in(0x1, a + 0x200, e["dma"]) +
           _kick(0x1101, 0x180, e["ecc"]),
           "RS decode, 0 bytes": _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x8100, e["ecc"]) + _kick(0x1101, 0x8100, e["ecc"]),
           "RS decode, 2 bytes": _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x8100, e["ecc"]) + _dma_in(0x1, a + 0x400, e["dma"]) +
           _kick(0x1101, 0x8100, e["ecc"])}
    out, lines = {}, []
    for label, body in pre.items():
        row = []
        for mode in (0x8100, 0x8104, 0x8108, 0x180):
            soft_reset(s, t)
            src = body + "    ld   r3, [ECC_RESULT]\n" + _dma_in(0x1, a, e["dma"]) + _kick(0x1102, mode, e["ecc"]) + \
                "    ld   r4, [ECC_RESULT]\n"
            rec, _ = _ecc_prog(s, t, src, RAMP + bad3 + bad2)
            out["%s/%x" % (label, mode)] = rec
            row.append("0x%x: %08x -> %08x" % (mode, rec["r"][3], rec["r"][4]))
        lines.append("  before: %-20s %s" % (label, " | ".join(row)))
    soft_reset(s, t)
    return out, "ECC_RESULT before -> after an encode:\n" + "\n".join(lines)


def _raw_page(t, a, row, nsec, ce=0):
    """program a page without the ECC engine: per sector a 16-byte chunk (scratch a + 0x200 * (2k)) and 512 data bytes
    (scratch a + 0x200 * (2k + 1)), both sent from data buffer 0"""
    e = t.events
    src = t.nand_prologue(ce) + t.cmd(0x80) + _address(t, row) + "    wr   [FMC_CTRL0], #0x%x\n" % (t.ce_sel[ce] | t.ce_dma)
    for k in range(nsec):
        src += _dma_in(0x1, a + 0x400 * k, e["dma"]) + _send(0x100, 0xf, 0xe4) + \
            _dma_in(0x1, a + 0x400 * k + 0x200, e["dma"]) + _send(0x100, 0x1ff, 0xe4)
    return src + "    wr   [FMC_CTRL0], #0x%x\n" % t.ce_sel[ce] + t.cmd(0x10) + _ready(t, 0) + \
        "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()


@experiment("rschip", needs=("nand_seq",))
def ex_rschip(s, t):
    """DESTRUCTIVE (block 3070 of CE0). Reed-Solomon parity on the chip: pages written by the engine with RS-8 (0x8100),
    RS-12 (0x8104) and BCH, read back raw and decoded in the same mode. Stored byte errors: the page is programmed a
    second time without an erase, with 0x00 at k data bytes and 0xff elsewhere (programming only clears bits), and
    decoded: RS-8 with 4 and 5 bad bytes, RS-12 with 6 and 7. RS-16 in the buffers: an encode into slot 3, both parity
    buffers dumped (where do the 4 extra bytes go?), and a decode of slot 3 with 2 bad bytes"""
    a, e = t.scratch_addr(s), t.events
    row0 = t.scratch_row + 70 * t.rows_per_block
    data = b"".join(bytes((i * (19 + 2 * k) + 3 * k + 1) & 0xff | 0x01 for i in range(512)) for k in range(4))
    out, lines = {}, []
    rec = probe(s, t, assemble(t, t.nand_prologue(0) + _erase(t, row0) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()),
                timeout_us=60000, flags=WAIT_END, quiet=True)
    lines.append("  erase: %s NSTAT %02x" % (rec["why"], rec["r"][7]))
    pages = [(0, 0x8100, None), (1, 0x8104, None), (2, 0x180, None), (3, 0x8100, 4), (4, 0x8100, 5), (5, 0x8104, 6),
             (6, 0x8104, 7)]
    for p, mode, k in pages:
        rec = probe(s, t, assemble(t, _program_page(t, a, row0 + p, 0, mode=mode)), timeout_us=60000, flags=WAIT_END,
                    scratch=data, quiet=True)
        line = "  page %d mode 0x%x: program %s NSTAT %02x" % (p, mode, rec["why"], rec["r"][7])
        if k:
            pos = [(j * 97 + 13) % 512 for j in range(k)]
            secs = b""
            for sct in range(4):
                d = bytearray(b"\xff" * 512)
                for q in pos:
                    d[q] = 0x00
                secs += b"\xff" * 512 + bytes(d)
            rec = probe(s, t, assemble(t, _raw_page(t, a, row0 + p, 4)), timeout_us=60000, flags=WAIT_END, scratch=secs, quiet=True)
            line += "; second program (%d zero bytes per sector) %s NSTAT %02x" % (k, rec["why"], rec["r"][7])
        rec = probe(s, t, assemble(t, _read_page(t, a, row0 + p, t.ce_sel[0], mode=mode)), timeout_us=80000, flags=WAIT_END,
                    scratch=b"\xee" * 0x1000, scratch_read=0x1000, quiet=True)
        sc = rec["scratch"] or bytes(0x1000)
        ok = "".join("=" if sc[0x200 * j:0x200 * j + 0x200] == data[0x200 * j:0x200 * j + 0x200] else "x" for j in range(4))
        line += "; read %s data %s results %s; chunk 0 as stored %s" % (rec["why"], ok, " ".join(
            "%x" % rec["after"][0xd70 + 4 * j] for j in range(4)), sc[0x800:0x810].hex())
        out["page %d" % p] = rec
        lines.append(line)
        print(line)
        sys.stdout.flush()
    # RS-16 slot 3
    bad = bytearray(RAMP); bad[7] ^= 0x55; bad[400] ^= 0xaa
    src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
        _kick(0x31102, 0x8108, e["ecc"]) + _dma_out(0x1000, a + 0x400, e["dma"]) + _dma_out(0x2000, a + 0x600, e["dma"]) + \
        _dma_in(0x1, a + 0x200, e["dma"]) + _kick(0x31101, 0x8108, e["ecc"]) + "    ld   r2, [ECC_RESULT]\n" + \
        _dma_out(0x100, a + 0x800, e["dma"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=RAMP + bytes(bad) + b"\xee" * 0x800,
                scratch_read=0xa00, quiet=True)
    sc = rec["scratch"] or bytes(0xa00)
    out["rs16 slot 3"] = rec
    lines.append("  RS-16 into slot 3: parity buffer 0 %s | parity buffer 1 %s | decode of slot 3 with 2 bad bytes: "
                 "ECC_RESULT %08x, data %s" % (sc[0x400:0x440].hex(), sc[0x600:0x640].hex(), rec["r"][2],
                                               "corrected" if sc[0x800:0xa00] == RAMP else "as sent" if sc[0x800:0xa00] == bytes(bad) else "other"))
    soft_reset(s, t)
    return out, "Reed-Solomon on the chip (block 3070):\n" + "\n".join(lines)


@experiment("miscorr", needs=("nand_seq",))
def ex_miscorr(s, t):
    """Can a decode report success with wrong data? BCH decodes with 9..64 random flipped bits and RS-8 decodes with
    5..40 random bad bytes, 240 sectors each (12 per program, encode and decode in one program): how many are reported
    uncorrectable, how many reported corrected with the original data back, how many reported corrected with other
    data (miscorrection)"""
    a, e = t.scratch_addr(s), t.events
    rnd = random.Random(25)
    out, lines = {}, []
    for code, mode in (("BCH", 0x180), ("RS-8", 0x8100)):
        tally = {"uncorrectable": 0, "corrected, right data": 0, "MISCORRECTED": 0, "UNDETECTED (reported no error)": 0}
        examples = []
        for prog in range(20):
            bads = []
            for k in range(12):
                b = bytearray(RAMP)
                if code == "BCH":
                    for bit in rnd.sample(range(4096), rnd.randint(9, 64)):
                        b[bit // 8] ^= 0x80 >> (bit % 8)
                else:
                    for pos in rnd.sample(range(512), rnd.randint(5, 40)):
                        b[pos] ^= rnd.randint(1, 255)
                bads.append(bytes(b))
            body = ""
            for k in range(12):
                body += _dma_in(0x1, a, e["dma"]) + _kick(0x1102, mode, e["ecc"]) + _dma_in(0x1, a + 0x200 * (1 + k), e["dma"]) + \
                    _kick(0x1101, mode, e["ecc"]) + "    ld   r1, [ECC_RESULT]\n    st   [0x%x], r1\n" % (0xd40 + 4 * k) + \
                    _dma_out(0x100, a + 0x200 * (13 + k), e["dma"])
            src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + body + \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=200000, flags=WAIT_END, scratch=RAMP + b"".join(bads) + b"\xee" * 0x1800,
                        scratch_read=0x200 * 25, quiet=True)
            sc = rec["scratch"] or bytes(0x200 * 25)
            for k in range(12):
                res = rec["after"][0xd40 + 4 * k]
                got = sc[0x200 * (13 + k):0x200 * (14 + k)]
                if res & 1:
                    key = "uncorrectable"
                elif got == RAMP:
                    key = "corrected, right data"
                elif got == bads[k]:
                    key = "UNDETECTED (reported no error)"
                    examples.append("%08x" % res)
                else:
                    key = "MISCORRECTED"
                    examples.append("%08x" % res)
                tally[key] += 1
            out["%s/%d" % (code, prog)] = rec
        lines.append("  %-5s %s%s" % (code, ", ".join("%s %d" % kv for kv in tally.items()),
                                      "; ECC_RESULT of miscorrections %s" % examples[:10] if examples else ""))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "decodes far beyond the code's strength:\n" + "\n".join(lines)


def _raw_read8(t, a, row, ce=0):
    """a whole 4 KiB page (two 2 KiB units, 8 sectors) as stored: chunk k to scratch a + 0x2000 + 16 k (via parity
    buffer 0, slot k % 4), data k to a + 0x200 k"""
    e = t.events
    src = t.nand_prologue(ce) + t.cmd(0x00) + _address(t, row) + "    wr   [FMC_CMD], #0x30\n    wait %d\n" \
        "    wr   [FMC_STATUS], #0x2\n" % e["cmd"] + _ready(t, 0) + "    wr   [FMCTRL1], #0x100e0\n" + t.cmd(0x00)
    for k in range(8):
        src += "    wr   [FMC_CTRL0], #0x%x\n" % t.ce_sel[ce] + _send(0x10, 0xf, 0x32, "    wr   [FMC_24], #0x%x\n" % (16 * (k % 4))) + \
            _send(0x1, 0x1ff, 0xe2) + "    wr   [FMC_CTRL0], #0x%x\n" % (t.ce_sel[ce] | t.ce_dma) + \
            _dma_out(0x100, a + 0x200 * k, e["dma"])
        if k % 4 == 3:
            src += _dma_out(0x1000, a + 0x2000 + 0x200 * (k // 4), e["dma"])
    return src + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()


@experiment("fullpage", needs=("nand_seq",))
def ex_fullpage(s, t):
    """READ-ONLY. Do both 2 KiB units of a 4 KiB page written by the original firmware carry the same metadata bytes?
    Page 0, 1 and 64 of blocks 1, 2, 10, 100, 500, 1000, 2000 of CE0 and CE1 read whole (8 sectors, raw): per page
    whether it is written, and the 3 metadata bytes of each of the 8 chunks"""
    a = t.scratch_addr(s)
    out, lines = {}, []
    for ce in (0, 1):
        for blk in (1, 2, 10, 100, 500, 1000, 2000):
            for pg in (0, 1, 64):
                row = blk * t.rows_per_block + pg
                rec = probe(s, t, assemble(t, _raw_read8(t, a, row, ce)), timeout_us=80000, flags=WAIT_END,
                            scratch=b"\xee" * 0x2400, scratch_read=0x2400, quiet=True)
                sc = rec["scratch"] or bytes(0x2400)
                chunks = [sc[0x2000 + 0x200 * (k // 4) + 16 * (k % 4):0x2000 + 0x200 * (k // 4) + 16 * (k % 4) + 16] for k in range(8)]
                erased = all(c == b"\xff" * 16 for c in chunks) and sc[:0x1000] == b"\xff" * 0x1000
                out["%d/%d/%d" % (ce, blk, pg)] = rec
                if erased:
                    lines.append("  CE%d block %4d page %2d: erased" % (ce, blk, pg))
                    continue
                m0 = b"".join(c[:3] for c in chunks[:4]); m1 = b"".join(c[:3] for c in chunks[4:])
                lines.append("  CE%d block %4d page %2d: %s metadata unit 0 %s unit 1 %s (%s)" % (
                    ce, blk, pg, rec["why"], m0.hex(), m1.hex(), "SAME" if m0 == m1 else "different"))
    return out, "whole 4 KiB pages, metadata bytes of the two units:\n" + "\n".join(lines)


# ---------------------------------------------------------------- NAND controller
@experiment("dmactl", needs=("nand_seq",))
def ex_dmactl(s, t):
    """DESTRUCTIVE (block 3051 of CE0). FMC_DMA_CTRL bit by bit (1 << b, b = 0..7) written before every data DMA of a
    page read, with the drain selects 0x100 and 0x1e0 (as a FMC_DATA_SEL value); and the combined selects of the
    original READ (fill one data buffer while draining the other: 0x102 / 0x201) used as written, alternating the data
    buffers per sector"""
    a, e = t.scratch_addr(s), t.events
    row = t.scratch_row + 51 * t.rows_per_block
    data = b"".join(bytes((i * (23 + 2 * k) + 5 * k) & 0xff for i in range(512)) for k in range(4))
    out, lines = {}, []
    probe(s, t, assemble(t, t.nand_prologue(0) + _erase(t, row) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()),
          timeout_us=60000, flags=WAIT_END, quiet=True)
    rec = probe(s, t, assemble(t, _program_page(t, a, row, 0)), timeout_us=60000, flags=WAIT_END, scratch=data, quiet=True)
    lines.append("  program: %s NSTAT %02x" % (rec["why"], rec["r"][7]))

    def read(dmactl, dsel):
        src = _read_page(t, a, row, t.ce_sel[0], dsel=dsel)
        if dmactl is not None:
            src = src.replace("    wr   [FMC_DATA_SEL], #0x%x\n    wr   [FMC_DMA_ADDR]" % dsel,
                              "    wr   [FMC_DMA_CTRL], #0x%x\n    wr   [FMC_DATA_SEL], #0x%x\n    wr   [FMC_DMA_ADDR]" % (dmactl, dsel))
        rec = probe(s, t, assemble(t, src), timeout_us=80000, flags=WAIT_END, scratch=b"\xee" * 0x1000, scratch_read=0x1000, quiet=True)
        sc = rec["scratch"] or bytes(0x1000)
        ok = "".join("=" if sc[0x200 * k:0x200 * k + 0x200] == data[0x200 * k:0x200 * k + 0x200] else "x" for k in range(4))
        return rec, ok
    for dsel in (0x100, 0x1e0):
        for b in [None] + list(range(8)):
            rec, ok = read(None if b is None else 1 << b, dsel)
            out["%x/%s" % (dsel, b)] = rec
            lines.append("  drain 0x%03x FMC_DMA_CTRL %-5s %-6s %5dus data %s; FMC_DMA_CTRL after %08x" % (
                dsel, "-" if b is None else "0x%x" % (1 << b), rec["why"], rec["elapsed_us"], ok, rec["after"][0x038]))
    # combined selects, alternating buffers
    src = t.nand_prologue(0) + t.cmd(0x00) + _address(t, row) + "    wr   [FMC_CMD], #0x30\n    wait %d\n    wr   [FMC_STATUS], #0x2\n" \
        % e["cmd"] + _ready(t, 0) + "    wr   [FMCTRL1], #0x100e0\n" + t.cmd(0x00)
    for k in range(4):
        b = k % 2                                         # data buffer b receives sector k, the other one is "drained"
        rsel = (1 << b) | (0x100 << (1 - b))              # 0x201 for buffer 0, 0x102 for buffer 1
        dsel = (0x100 << b) | (1 << (1 - b))              # 0x102 for buffer 0, 0x201 for buffer 1
        src += "    wr   [FMC_CTRL0], #@CE0@\n" + _send(0x10, 0xf, 0x32, "    wr   [FMC_24], #0x%x\n" % (16 * k)) + \
            _send(rsel, 0x1ff, 0xe2) + "    wr   [FMC_CTRL0], #@CE0_DMA@\n" + \
            _kick(0x1001 | (0x100 << b) | k << 16, 0x180, e["ecc"]) + \
            "    ld   r2, [ECC_RESULT]\n    st   [0x%x], r2\n" % (0xd70 + 4 * k) + _dma_out(dsel, a + 0x200 * k, e["dma"])
    src += "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=80000, flags=WAIT_END, scratch=b"\xee" * 0x1000, scratch_read=0x1000, quiet=True)
    sc = rec["scratch"] or bytes(0x1000)
    ok = "".join("=" if sc[0x200 * k:0x200 * k + 0x200] == data[0x200 * k:0x200 * k + 0x200] else "x" for k in range(4))
    out["combined"] = rec
    lines.append("  combined selects, data buffers alternating (read 0x201 / 0x102, DMA 0x102 / 0x201): %s data %s results %s" % (
        rec["why"], ok, " ".join("%x" % rec["after"][0xd70 + 4 * k] for k in range(4))))
    soft_reset(s, t)
    return out, "FMC_DMA_CTRL and data selects on a page read (block 3051):\n" + "\n".join(lines)


@experiment("fmcstat2", needs=("nand_seq",))
def ex_fmcstat2(s, t):
    """FMC_STATUS bits 12, 13, 18, 19 (set after a soft reset, not cleared by writing 1, bits 13 and 19 cleared by a
    16-byte data read). After that data read, which operation sets bits 13 / 19 again: a DMA into a data buffer, a DMA
    out of one, a parity-buffer DMA, a chip data read of 512 bytes, an ECC encode, a status latch, a command, a
    second-set data read, the set-up word"""
    a, e = t.scratch_addr(s), t.events
    rd16 = "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x10\n    wr   [FMC_24], #0x0\n" \
           "    wr   [FMCTRL1], #0x32\n    wait 3\n    wr   [FMC_STATUS], #0x8\n"
    ops = [("nothing", ""), ("DMA into data buffer 0", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"])),
           ("DMA out of data buffer 0", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_out(0x100, a, e["dma"])),
           ("DMA into parity buffer 0", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x10, a, e["dma"])),
           ("chip read 512 bytes", "    wr   [FMC_CTRL0], #@CE0@\n" + _send(0x1, 0x1ff, 0xe2)),
           ("ECC encode", _kick(0x1102, 0x180, e["ecc"])),
           ("status latch", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) +
            "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"]),
           ("command", "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x70)),
           ("second-set data read", "    wr   [0x400], #@CE0@\n    wr   [0x430], #0xf\n    wr   [0x414], #0x10\n    wr   [0x404], #0x32\n"
            "    wait 9\n"),
           ("set-up word", t.state_setup())]
    out, lines = {}, []
    for label, body in ops:
        soft_reset(s, t)
        src = t.prologue(c58=0) + t.state_setup() + rd16 + "    ld   r1, [FMC_STATUS]\n" + body + "    ld   r2, [FMC_STATUS]\n" \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=bytes(512), quiet=True)
        out[label] = rec
        bits = lambda v: "".join(str(v >> b & 1) for b in (12, 13, 18, 19))
        lines.append("  %-24s bits 12/13/18/19 after the 16-byte read %s, after the operation %s (%s)" % (
            label, bits(rec["r"][1]), bits(rec["r"][2]), rec["why"]))
    soft_reset(s, t)
    return out, "FMC_STATUS bits 12, 13, 18, 19:\n" + "\n".join(lines)


@experiment("ch1buf", needs=("nand_seq",))
def ex_ch1buf(s, t):
    """Does the second register set (channel 1) have parity buffers and two data buffers? Through the 0x400 set: a
    pattern DMA-ed into a parity buffer (0x414 = 0x10 / 0x20) and back (0x1000 / 0x2000), into data buffer 1
    (0x414 = 2 / 0x200); and whether that changed the first set's parity buffer 0 (filled with another pattern first)"""
    a, e = t.scratch_addr(s), t.events
    p1 = bytes((i * 7 + 1) & 0xff for i in range(512))
    p2 = bytes((i * 13 + 5) & 0xff for i in range(512))

    def s1(fill, drain, dst):
        return ("    wr   [0x404], #@FMCTRL1_INIT@\n    wr   [0x448], #@STATUS_CLEAR@\n    wr   [0x400], #@CE_DMA@\n"
                "    wr   [0x430], #0x1ff\n    wr   [0x414], #0x%x\n    wr   [0x434], #0x%x\n    wr   [0x404], #0x2e0\n    wait %d\n"
                "    wr   [0x448], #0x100000\n    wr   [0x414], #0x%x\n    wr   [0x434], #0x%x\n    wr   [0x404], #0x100\n    wait %d\n"
                "    wr   [0x448], #0x100000\n    wr   [0x400], #@CE_NONE@\n" % (fill, a + 0x200, e["set1_dma"], drain, dst, e["set1_dma"]))
    out, lines = {}, []
    for label, fill, drain in (("parity buffer 0", 0x10, 0x1000), ("parity buffer 1", 0x20, 0x2000), ("data buffer 1", 0x2, 0x200),
                               ("data buffer 0", 0x1, 0x100)):
        soft_reset(s, t)
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x10, a, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n" + s1(fill, drain, a + 0x400) + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + \
            _dma_out(0x1000, a + 0x600, e["dma"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END, scratch=p1 + p2 + b"\xee" * 0x400, scratch_read=0x800, quiet=True)
        sc = rec["scratch"] or bytes(0x800)
        back = sc[0x400:0x600]
        n = 64 if fill in (0x10, 0x20) else 512
        ref = p2[448:512] if n == 64 else p2
        out[label] = rec
        lines.append("  second set, %-15s %s: back %s; first set's parity buffer 0 %s" % (
            label, rec["why"], "= pattern" if back[:n] == ref else "other (%s)" % back[:16].hex(),
            "unchanged" if sc[0x600:0x640] == p1[448:512] else "CHANGED (%s)" % sc[0x600:0x610].hex()))
    soft_reset(s, t)
    return out, "buffers of the second register set (DMA via 0x400):\n" + "\n".join(lines)


@experiment("evsweep3", needs=("nand_seq", "start_value"))
def ex_evsweep3(s, t):
    """wait events 0..15 and wait2 conditions 0..255 (short timeout) in operations not yet swept: a second-set DMA in
    flight, a meta encode and a meta decode in flight, an ECC decode in flight, a chip data write in flight (80h,
    address, 512 bytes), a parity-buffer DMA in flight"""
    a, e = t.scratch_addr(s), t.events
    order = [("wait", n) for n in range(16)] + [("wait2", n) for n in range(256)]
    row = t.scratch_row + 71 * t.rows_per_block
    st = {"second-set DMA in flight": "    wr   [0x404], #@FMCTRL1_INIT@\n    wr   [0x400], #@CE_DMA@\n    wr   [0x430], #0x1ff\n"
                                      "    wr   [0x414], #0x1\n    wr   [0x434], #0x%x\n    wr   [0x404], #0x2e0\n" % a,
          "meta encode in flight": "    wr   [FMC_METACFG], #0x3210\n    wr   [FMC_METACTL], #0x1\n",
          "meta decode in flight": "    wr   [FMC_METACFG], #0x3210\n    wr   [FMC_METACTL], #0x2\n",
          "ECC decode in flight": "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1101\n",
          "chip data write in flight": t.cmd(0x80) + _address(t, row) + "    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x100\n"
                                       "    wr   [FMCTRL1], #0xe4\n",
          "parity-buffer DMA in flight": "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x10\n"
                                         "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n" % a}
    out, lines = {}, []
    for label, act in st.items():
        soft_reset(s, t)
        src = t.nand_prologue(0) + act + "    movi r2, #0x%x\n" % (a + 0x1000) + _sweep_src(order) + \
            "    wr   [0x400], #@CE_NONE@\n    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMCTRL1], #0x20\n" + t.cmd(0xff) + \
            _ready(t, 9) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=400000, start_value=0x3f15, flags=WAIT_END,
                    scratch=bytes(0x1000) + b"\xee" * (4 * len(order)), scratch_read=0x1000 + 4 * len(order), quiet=True)
        ok, bad = _passes((rec["scratch"] or bytes(0x2000))[0x1000:], order)
        out[label] = rec
        lines.append("  %-30s wait %s; wait2 %s%s" % (label, [n for k, n in ok if k == "wait"], [n for k, n in ok if k == "wait2"],
                                                     "; NOT STORED %d" % len(bad) if bad else ""))
    soft_reset(s, t)
    return out, "wait / wait2 passes during further operations (short timeout):\n" + "\n".join(lines)


@experiment("blk100b", needs=("nand_seq", "pld_pst"))
def ex_blk100b(s, t):
    """The 0x100 block's writable words and the words 0x3a8..0x3b4. Each writable word set to 0 (0x1b8 to 1; 0x1cc /
    0x1d0 also to 0xffffff) by the sequencer, then timed: 8 DMAs, 8 ECC encode + decode, a 512-byte chip read, a
    wait that times out, and wait2 33 / 5; the word restored at the end. 0x3a8..0x3b4: written by the
    CPU (params) and by the program (pst), read by both"""
    a, e = t.scratch_addr(s), t.events
    body = "    wr   [FMC_CTRL0], #@CE_DMA@\n" + "".join(_dma_in(0x1, a, e["dma"]) for _ in range(8)) + \
        "".join(_kick(0x1102, 0x180, e["ecc"]) + _kick(0x1101, 0x180, e["ecc"]) for _ in range(8)) + \
        "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x00) + _address(t, t.scratch_row) + t.cmd(0x30) + _ready(t, 0) + \
        "    wr   [FMCTRL1], #0x100e0\n" + t.cmd(0x00) + _send(0x1, 0x1ff, 0xe2) + \
        "    wr   [CS_IRQ], #0x4\n    wait 10\n    ld   r3, [CS_IRQ], 0x4\n    wr   [CS_IRQ], #0x4\n    wait2 33\n" \
        "    ld   r4, [CS_IRQ], 0x4\n    wr   [CS_IRQ], #0x4\n    wait2 5\n    ld   r5, [CS_IRQ], 0x4\n"
    cases = [("default", None, None)] + [("0x%03x = 0" % w, w, 0) for w in [0x1bc, 0x1c0, 0x1c4, 0x1c8, 0x1cc, 0x1d0, 0x1d4, 0x1d8]] + \
        [("0x1b8 = 1", 0x1b8, 1), ("0x1cc = 0xffffff", 0x1cc, 0xffffff), ("0x1d0 = 0xffffff", 0x1d0, 0xffffff)]
    out, lines = {}, []
    for label, w, v in cases:
        soft_reset(s, t)
        pre = "" if w is None else "    movi r6, #0x%x\n    movi r7, #0x%x\n    pst  [r6], r7\n" % (w, v)
        src = t.prologue(c58=0) + t.state_setup() + pre + body + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + t.restore_seq() + END + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=100000, flags=WAIT_END, scratch=RAMP, quiet=True)
        out[label] = rec
        lines.append("  %-18s %s %6dus; wait 10 %s, wait2 33 %s, wait2 5 %s" % (
            label, rec["why"], rec["elapsed_us"], *["timeout" if rec["r"][i] else "pass" for i in (3, 4, 5)]))
    # 0x3a8..0x3b4
    src = t.prologue(c58=0) + "".join("    movi r4, #0x%x\n    pld  r1, [r4]\n    st   [0x%x], r1\n" % (0x3a8 + 4 * i, 0xd40 + 4 * i) for i in range(4)) + \
        "".join("    movi r4, #0x%x\n    movi r1, #0x%x\n    pst  [r4], r1\n" % (0x3a8 + 4 * i, 0x3a83a800 + i) for i in range(4)) + \
        "".join("    movi r4, #0x%x\n    pld  r1, [r4]\n    st   [0x%x], r1\n" % (0x3a8 + 4 * i, 0xd50 + 4 * i) for i in range(4)) + END + t.trap()
    rec = probe(s, t, assemble(t, src), params=[(0x3a8 + 4 * i, 0xc0de0000 + i) for i in range(4)], timeout_us=20000,
                flags=WAIT_END | FULLMAP, quiet=True)
    fm = rec.get("fullmap") or [[0] * 1024] * 2
    lines.append("  0x3a8..0x3b4: CPU wrote c0de000x; sequencer read %s; sequencer wrote 3a83a80x, read back %s; CPU reads after: %s" % (
        " ".join("%08x" % rec["after"][0xd40 + 4 * i] for i in range(4)), " ".join("%08x" % rec["after"][0xd50 + 4 * i] for i in range(4)),
        " ".join("%08x" % fm[1][(0x3a8 + 4 * i) // 4] for i in range(4))))
    soft_reset(s, t)
    return out, "0x100 block words and 0x3a8..0x3b4:\n" + "\n".join(lines)


@experiment("straps", needs=())
def ex_straps(s, t):
    """READ-ONLY, CPU only. The boot ROM chooses its boot device from GPIO port 3 pins 6, 7 and checks port 1 pins 0, 1
    with USB PHY word 0x3c400028 bit 0 for a forced USB boot: the CPU reads PCON / PDAT of ports 1 and 3 and that word"""
    rd = lambda addr: struct.unpack("<I", s.read(addr, 4))[0]
    v = {n: rd(addr) for n, addr in (("PCON1", 0x3cf00020), ("PDAT1", 0x3cf00024), ("PCON3", 0x3cf00060),
                                    ("PDAT3", 0x3cf00064), ("PHY+0x28", 0x3c400028))}
    p3 = (v["PDAT3"] >> 6 & 1, v["PDAT3"] >> 7 & 1)
    dev = {(0, 0): "SPI flash, SPI port 0", (0, 1): "SPI flash, SPI port 2", (1, 0): "NAND channel 0", (1, 1): "NAND channel 1"}[p3]
    return v, "GPIO / PHY words: %s; P1.0 %d P1.1 %d; P3.6 %d P3.7 %d -> boot device %s; PHY+0x28 bit 0 = %d" % (
        " ".join("%s %08x" % kv for kv in v.items()), v["PDAT1"] & 1, v["PDAT1"] >> 1 & 1, p3[0], p3[1], dev, v["PHY+0x28"] & 1)


@experiment("w2vic", needs=("nand_seq", "start_value"))
def ex_w2vic(s, t):
    """Are the wait2 conditions the CPU's interrupt lines? wait2 condition 20 is true exactly when VIC0 line 20 (the NAND
    controller's line) is raised. For N = 0..63, just before each `wait2 N` (short timeout, start word 0x3f15) the program
    reads the raw interrupt status of VIC0 (0x38e00008) and VIC1 (0x38e01008) with ldi; per N the pass / timeout is
    compared with raw line N (VIC0 bit N for N < 32, VIC1 bit N - 32). States: idle; a command flag pending with
    0x044 bit 0 (VIC0 line 20); a DMA flag pending with 0x044 bit 11; an ECC_DONE flag pending with 0x844 bit 0. Then
    N = 64..255 once, idle. Each program acknowledges every flag and clears 0x044 / 0x844 before its end"""
    a, e = t.scratch_addr(s), t.events
    states = [("idle", "", []),
              ("command flag pending, 0x044 bit 0", "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"], [(0x044, 1)]),
              ("DMA flag pending, 0x044 bit 11", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a + 0x1000, e["dma"]).replace(
                  "    wr   [FMC_STATUS], #0x100000\n", ""), [(0x044, 1 << 11)]),
              ("ECC_DONE pending, 0x844 bit 0", _kick(0x1102, 0x180, e["ecc"]), [(0x844, 1)])]
    tail = "    wr   [FMCTRL1], #0x20\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n    wr   [ECC_DONE], #0x1ff\n    wr   [0x044], #0x0\n" \
           "    wr   [0x844], #0x0\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + END + t.trap()
    out, lines = {}, []
    for label, pre, params in states:
        soft_reset(s, t)
        src = t.prologue(c58=0) + t.state_setup() + pre + "    movi r2, #0x%x\n    movi r5, #0x38e00008\n    movi r6, #0x38e01008\n" % a
        for n in range(64):
            src += "    ldi  r3, [r5]\n    ldi  r4, [r6]\n    wr   [CS_IRQ], #0x4\n    wait2 %d\n    ld   r1, [CS_IRQ], 0x4\n" \
                   "    sti  [r2], r1\n    add  r2, r2, #0x4\n    sti  [r2], r3\n    add  r2, r2, #0x4\n    sti  [r2], r4\n" \
                   "    add  r2, r2, #0x4\n" % n
        src += tail
        rec = probe(s, t, assemble(t, src), params=params, timeout_us=400000, start_value=0x3f15, flags=WAIT_END,
                    scratch=b"\xee" * (12 * 64) + bytes(512), scratch_read=12 * 64, quiet=True)
        out[label] = rec
        sc = rec["scratch"] or bytes(12 * 64)
        passed, match, mism, rawset = [], 0, [], set()
        for n in range(64):
            f, v0, v1 = struct.unpack_from("<III", sc, 12 * n)
            raw = (v0 >> n & 1) if n < 32 else (v1 >> (n - 32) & 1)
            if raw: rawset.add(n)
            ok = f == 0
            if ok: passed.append(n)
            if ok == bool(raw): match += 1
            else: mism.append("%d(%s, line %d)" % (n, "pass" if ok else "timeout", raw))
        lines.append("  %-36s %s: wait2 passed %s; raw lines set %s; agree %d of 64%s" % (
            label, rec["why"], passed, sorted(rawset), match, "; differ: " + " ".join(mism[:16]) if mism else ""))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    order = [("wait2", n) for n in range(64, 256)]
    src = t.prologue(c58=0) + "    movi r2, #0x%x\n" % a + _sweep_src(order) + END + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=400000, start_value=0x3f15, flags=WAIT_END,
                scratch=b"\xee" * (4 * len(order)), scratch_read=4 * len(order), quiet=True)
    ok, bad = _passes(rec["scratch"] or bytes(4 * len(order)), order)
    out["64..255"] = rec
    lines.append("  wait2 64..255, idle: passed %s; VIC0 / VIC1 raw seen during the run %08x / %08x" % (
        [n for k, n in ok], rec["vic0_seen"], rec["vic1_seen"]))
    soft_reset(s, t)
    return out, "wait2 conditions against the raw interrupt lines (VIC0 0..31, VIC1 32..63):\n" + "\n".join(lines)
