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

"""Differential tests: the same programs under two start words (start-word bit 6), the reserved parts of the window
from the sequencer, the Reed-Solomon strengths, the interrupt line per source, and what advances FMC_28."""
import struct
import sys

from ..probe import probe, assemble, soft_reset, WAIT_END, FULLMAP
from .registry import experiment
from .followup import _dma_in, _dma_out, _kick

# window ranges with no known register: read 0 / drop writes from the CPU (0xc60.., 0xd80.., 0xe00..), and the
# unexplained words (0x3a8..0x3b4), the unknown block (0x100..0x1fc)
RESERVED = [(0xc60, 0xd00), (0xd80, 0xe00), (0xe00, 0x1000), (0x3a8, 0x3b8), (0x100, 0x200)]
DRIFT = {0x028, 0x228, 0x050, 0x250, 0x428, 0x628, 0x450, 0x650, 0x9a4, 0xc00}     # words that change by themselves; 0xc00 reads back the start word


def _words(lo, hi):
    return list(range(lo, hi, 4))


def _dump_src(t, a, words, base_off):
    """pld every word into r1 and sti it to RAM a + base_off + 4 i"""
    out = "    movi r5, #0x0\n"
    for i, w in enumerate(words):
        out += "    movi r4, #0x%x\n    pld  r1, [r4]\n    movi r2, #0x%x\n    sti  [r2], r1\n" % (w, a + base_off + 4 * i)
    return out


def _fm_diff(r1, r2):
    """window words that one of the two runs changed and that end with different values; a word an earlier program
    left set is unchanged by both runs and drops out"""
    f1, f2 = r1.get("fullmap"), r2.get("fullmap")
    if not f1 or not f2:
        return []
    out = []
    for i in range(1024):
        if 4 * i in DRIFT:
            continue
        changed = f1[1][i] != f1[0][i] or f2[1][i] != f2[0][i]
        if changed and f1[1][i] != f2[1][i]:
            out.append((4 * i, f1[1][i], f2[1][i]))
    return out


@experiment("startdiff", needs=("nand_seq", "fullmap", "start_value"))
def ex_startdiff(s, t):
    """Start-word bit 6: the original operating systems start with 0xfff5, the 4G/5G bootloader with 0xffb5; no
    difference between them has been found. Bit 6 is part of the timeout step n (bits 7:4); n = 15 and 11 (and 13 and 9)
    give the same step, so these pairs differ in bit 6 alone. The same set of programs is run under 0xfff5 / 0xffb5 and
    0xffd5 / 0xff95, each after a soft reset, and every observable is compared: end
    reason, time, CS_IRQ, r0..r7, interrupt lines, the whole window after the run, and the sequencer's reads and writes
    of the reserved parts of the window (0xc60..0xcfc, 0xd80..0xffc, 0x100..0x1fc, 0x3a8..0x3b4)"""
    a, e = t.scratch_addr(s), t.events
    words = [w for lo, hi in RESERVED for w in _words(lo, hi)]
    end = "    movi r6, #0x600d600d\n    end\n" + t.trap()
    progs = [
        ("end only", t.prologue(c58=0) + end),
        ("sig, end", t.prologue(c58=0) + "    sig\n" + end),
        ("loop 2000", t.prologue(c58=0) + "    movi r0, #2000\nl:\n    sub  r0, r0, #0x1\n    bnz  r0, l\n" + end),
        ("wait 10 (never), C58 0", t.prologue(c58=0) + "    wait 10\n" + end),
        ("wait 10 (never), C58 4", t.prologue(c58=4) + "    wait 10\n" + end),
        ("illegal 0x30, C58 4", t.prologue(c58=4) + "    .insn 0x30, 0, 0, 0\n" + end),
        ("evjmp, nothing pending", t.prologue(c58=0) + "    evjmp\n" + end),
        ("tmwait", t.prologue(c58=0) + "    tmwait\n" + end),
        ("DMA in + ECC encode", t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" +
         _dma_in(0x1, a, e["dma"]) + _kick(0x1102, 0x180, e["ecc"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + end),
        ("status latch, wait 5", t.nand_prologue(0) + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"] +
         "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" + end),
        ("reserved: sequencer reads", t.prologue(c58=0) + _dump_src(t, a, words, 0x400) + end),
        ("reserved: sequencer writes all ones, reads back", t.prologue(c58=0) + "".join(
            "    movi r4, #0x%x\n    movi r1, #0xffffffff\n    pst  [r4], r1\n" % w for w in words) +
         _dump_src(t, a, words, 0x400) + end),
    ]
    starts = (0xfff5, 0xffb5, 0xffd5, 0xff95)
    out, lines = {}, []
    recs = {}
    for label, src in progs:
        code = assemble(t, src)
        for w in starts:
            soft_reset(s, t)
            rec = probe(s, t, code, timeout_us=60000, start_value=w, flags=WAIT_END | FULLMAP,
                        scratch=bytes(0x400) + b"\xee" * (4 * len(words)), scratch_read=0x400 + 4 * len(words),
                        quiet=True)
            recs[(label, w)] = rec
            out["%s/0x%04x" % (label, w)] = rec
    soft_reset(s, t)
    for label, _ in progs:
        for w1, w2 in ((0xfff5, 0xffb5), (0xffd5, 0xff95)):
            r1, r2 = recs[(label, w1)], recs[(label, w2)]
            d = []
            for k in ("why", "cs_irq", "vic0_seen", "vic1_seen", "vic1_end", "irq_seen"):
                if r1.get(k) != r2.get(k):
                    d.append("%s %s / %s" % (k, r1.get(k), r2.get(k)))
            if abs(r1["elapsed_us"] - r2["elapsed_us"]) > max(20, 0.05 * max(r1["elapsed_us"], r2["elapsed_us"])):
                d.append("time %d / %d us" % (r1["elapsed_us"], r2["elapsed_us"]))
            if r1["r"] != r2["r"]:
                d.append("r %s / %s" % (r1["r"], r2["r"]))
            fm = _fm_diff(r1, r2)
            if fm:
                d.append("window after (a run changed it) " + ", ".join("%03x %08x/%08x" % x for x in fm[:12]) +
                         (" ..." if len(fm) > 12 else ""))
            if label.startswith("reserved") and r1.get("scratch") and r2.get("scratch"):
                s1, s2 = r1["scratch"][0x400:], r2["scratch"][0x400:]
                diff = [(words[i], struct.unpack_from("<I", s1, 4 * i)[0], struct.unpack_from("<I", s2, 4 * i)[0])
                        for i in range(len(words)) if s1[4 * i:4 * i + 4] != s2[4 * i:4 * i + 4]]
                nz = [(words[i], struct.unpack_from("<I", s1, 4 * i)[0]) for i in range(len(words))
                      if s1[4 * i:4 * i + 4] != bytes(4)]
                if diff:
                    d.append("sequencer reads " + ", ".join("%03x %08x/%08x" % x for x in diff[:12]))
                if w1 == 0xfff5:
                    lines.append("  %-48s non-zero reserved words read by the sequencer (0x%04x): %s" % (
                        label, w1, ", ".join("%03x=%08x" % x for x in nz[:16]) or "none"))
            lines.append("  %-48s 0x%04x vs 0x%04x: %s" % (label, w1, w2, "; ".join(d) if d else "no difference"))
        print("\n".join(lines[-2:]))
        sys.stdout.flush()
    return out, "the same programs under start words differing only in bit 6 (differences only; drifting words " \
        "FMC_28, 0x050, 0x9a4 left out):\n" + "\n".join(lines)


@experiment("rsdecode", needs=("nand_seq", "fullmap"))
def ex_rsdecode(s, t):
    """Reed-Solomon with 12 and 16 parity symbols (ECC_MODE b15 with b2 / b3): the decode strength (encode and decode
    in one program, k bytes changed), and where the 20 parity bytes of the 16-symbol code go (the whole parity buffer,
    all four slots, is copied out after the encode; the slot holds 16)"""
    a, e = t.scratch_addr(s), t.events
    base = bytes((i * 37 + 11) & 0xff for i in range(512))
    out, lines = {}, []
    for mode, ks in ((0x8104, (0, 4, 6, 7)), (0x8108, (0, 4, 8, 9, 12))):
        for k in ks:
            bad = bytearray(base)
            for j in range(k):
                bad[(j * 61 + 7) % 512] ^= 0x5a
            src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
                _kick(0x1102, mode, e["ecc"]) + _dma_out(0x1000, a + 0x600, e["dma"]) + \
                _dma_in(0x1, a + 0x200, e["dma"]) + _kick(0x1101, mode, e["ecc"]) + \
                "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + _dma_out(0x100, a + 0x400, e["dma"]) + \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                        scratch=base + bytes(bad) + b"\xee" * 0x400, scratch_read=0x800, quiet=True)
            sc = rec["scratch"] or bytes(0x800)
            back = sc[0x400:0x600]
            out["0x%x/%d" % (mode, k)] = rec
            line = "  0x%05x, %2d bytes changed: ECC_RESULT %08x ECC_DONE %02x, data %s" % (
                mode, k, rec["r"][2], rec["r"][3], "corrected" if back == base else "as sent" if back == bytes(bad)
                else "other")
            if k == 0:
                area = b"".join(struct.pack("<I", w) for w in rec["fullmap"][1][0x900 // 4:0x918 // 4]) \
                    if rec.get("fullmap") else b""
                line += "\n      parity buffer after the encode (slots 0..3): %s\n      ECC area 0x900..0x914: %s" % (
                    sc[0x600:0x640].hex(), area.hex())
            lines.append(line)
    soft_reset(s, t)
    return out, "Reed-Solomon 12 / 16 parity symbols, decode strength and the parity buffer:\n" + "\n".join(lines)


@experiment("vic20", needs=("nand_seq",))
def ex_vic20(s, t):
    """VIC0 line 20 was seen for different FMC interrupt sources in two runs. Each source with its enable bit, three
    times, each after a soft reset: is the line stable per source, and which CS_IRQ bits come with it?"""
    a, e = t.scratch_addr(s), t.events
    act = {"command": t.cmd(0x70).replace("    wr   [FMC_STATUS], #0x2\n", ""),
           "address": "    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n    wr   [FMCTRL1], #0x1\n    wait %d\n" % e["addr"],
           "status": t.cmd(0x70) + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) +
           "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"],
           "DMA": "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]).replace("    wr   [FMC_STATUS], #0x100000\n", "")}
    out, lines = {}, []
    for src_name, bits in (("command", (0, 1, 2)), ("address", (0, 1)), ("status", (0, 13)), ("DMA", (11,)), ("none", (0,))):
        for bit in bits:
            row = []
            for n in range(3):
                soft_reset(s, t)
                src = t.nand_prologue(0) + act.get(src_name, "") + "    wr   [FMC_CTRL0], #@CE_NONE@\n" \
                    "    movi r6, #0x600d600d\n    end\n" + t.trap()
                rec = probe(s, t, assemble(t, src), params=[(0x044, 1 << bit)], timeout_us=20000, flags=WAIT_END,
                            scratch=bytes(512), quiet=True)
                out["%s/bit %d/%d" % (src_name, bit, n)] = rec
                row.append("CS_IRQ %02x VIC0.20 %d VIC1.22 %d" % (rec["cs_irq"], rec["vic0_seen"] >> 20 & 1,
                                                                rec["vic1_seen"] >> 22 & 1))
            lines.append("  %-8s 0x044 bit %2d: %s" % (src_name, bit, " | ".join(row)))
    soft_reset(s, t)
    return out, "interrupt lines per FMC source, three runs each:\n" + "\n".join(lines)


@experiment("fmc28b", needs=("nand_seq", "fullmap"))
def ex_fmc28b(s, t):
    """FMC_28 stays 0 through every operation after the set-up word but rose ~124 per probe run without it. A series
    of runs, each reading FMC_28 at its start and end, with and without the set-up word, with nothing, a long loop, a
    chip read, a DMA, an ECC encode; the CPU's window dumps give the value between runs"""
    a, e = t.scratch_addr(s), t.events
    rd = "    ld   r%d, [FMC_28]\n"
    end = "    movi r6, #0x600d600d\n    end\n" + t.trap()
    bodies = [("nothing", ""), ("loop 2000", "    movi r0, #2000\nl:\n    sub  r0, r0, #0x1\n    bnz  r0, l\n"),
              ("DMA", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n"),
              ("ECC encode", _kick(0x1102, 0x180, e["ecc"])),
              ("16-byte chip read", "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x00) + "    wr   [FMC_DNUM], #0xf\n"
               "    wr   [FMC_DATA_SEL], #0x10\n    wr   [FMC_24], #0x0\n    wr   [FMCTRL1], #0x32\n    wait 3\n"
               "    wr   [FMC_STATUS], #0x8\n    wr   [FMC_CTRL0], #@CE_NONE@\n")]
    out, lines = {}, []
    for setup in (False, True):
        for label, body in bodies:
            src = t.prologue(c58=0) + (t.state_setup() if setup else "") + rd % 4 + body + rd % 5 + end
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP, scratch=bytes(512), quiet=True)
            out["%s/%s" % ("set-up" if setup else "no set-up", label)] = rec
            fm = rec.get("fullmap")
            cpu_before = fm[0][0x028 // 4] & 0x1ff if fm else -1
            cpu_after = fm[1][0x028 // 4] & 0x1ff if fm else -1
            lines.append("  %-9s %-18s CPU before %3d | program start %3d, end %3d | CPU after %3d" % (
                "set-up" if setup else "no set-up", label, cpu_before, rec["r"][4] & 0x1ff, rec["r"][5] & 0x1ff, cpu_after))
    return out, "FMC_28 (9 bits) around runs:\n" + "\n".join(lines)


@experiment("startbits", needs=("nand_seq", "fullmap", "start_value", "sram_prog"))
def ex_startbits(s, t):
    """What the start-word bits do beyond the known ones (bit 0 start, 1 pause, 2 timeout enable, 3 and 16 hold, 7:4
    timeout step, 15:8 timeout length, 18 read-back status). Each bit 4..31 (16 left out: hold) is flipped alone in
    0xfff5 and a set of programs is run that looks at things not measured before: instruction fetch from SDRAM and from
    internal SRAM, system-memory reads (ldi) and writes (sti), window reads (pld), sig and the interrupt lines, a DMA plus
    an ECC encode, and a fault with vectoring. Every variant is compared with the 0xfff5 baseline, which is run three
    times to measure the run-to-run spread; only differences beyond that spread are reported (timeout lengths change
    with bits 4..15 by design and are shown for the fault program only)"""
    from ..probe import PROG_SRAM
    a, e = t.scratch_addr(s), t.events
    end = "    movi r6, #0x600d600d\n    end\n" + t.trap()
    loop = lambda body: "    movi r0, #2000\nl:\n" + body + "    sub  r0, r0, #0x1\n    bnz  r0, l\n"
    progs = [
        ("loop, SDRAM", t.prologue(c58=0) + loop("") + end, 0),
        ("loop, SRAM", t.prologue(c58=0) + loop("") + end, PROG_SRAM),
        ("2000 ldi", t.prologue(c58=0) + "    movi r2, #0x%x\n" % a + loop("    ldi  r1, [r2]\n") + end, 0),
        ("2000 sti", t.prologue(c58=0) + "    movi r2, #0x%x\n" % (a + 0x100) + loop("    sti  [r2], r0\n") + end, 0),
        ("2000 pld", t.prologue(c58=0) + "    movi r2, #0xd70\n" + loop("    pld  r1, [r2]\n") + end, 0),
        ("sig, end", t.prologue(c58=0) + "    sig\n" + end, 0),
        ("DMA + ECC encode", t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) +
         _kick(0x1102, 0x180, e["ecc"]) + "    wr   [FMC_CTRL0], #@CE_NONE@\n" + end, 0),
        ("fault, C58 4 (vector)", t.prologue(c58=4) + "    .insn 0x30, 0, 0, 0\n" + end, 0),
    ]
    codes = [(label, assemble(t, src), fl) for label, src, fl in progs]

    def run(w):
        recs = {}
        for label, code, fl in codes:
            soft_reset(s, t)
            recs[label] = probe(s, t, code, timeout_us=60000, start_value=w, flags=WAIT_END | FULLMAP | fl,
                                scratch=bytes(0x200), quiet=True)
        return recs
    out, lines = {}, []
    base = [run(0xfff5) for _ in range(3)]
    for n, r in enumerate(base):
        for label, rec in r.items():
            out["0xfff5 #%d/%s" % (n, label)] = rec
    spread = {label: (min(b[label]["elapsed_us"] for b in base), max(b[label]["elapsed_us"] for b in base)) for label, _, _ in codes}
    lines.append("  baseline 0xfff5 x3, run time spread (us): " + ", ".join("%s %d..%d" % (k, v[0], v[1]) for k, v in spread.items()))
    ref = base[0]
    for bit in [b for b in range(4, 32) if b != 16]:
        w = 0xfff5 ^ (1 << bit)
        recs = run(w)
        diffs = []
        for label, rec in recs.items():
            out["0x%08x/%s" % (w, label)] = rec
            r0 = ref[label]
            lo, hi = spread[label]
            tol = max(10, 0.03 * hi)
            if label.startswith("fault"):
                if rec["why"] != r0["why"] or (rec["cs_irq"] & ~0x0) != r0["cs_irq"]:
                    diffs.append("%s: %s cs_irq %x (base %s %x)" % (label, rec["why"], rec["cs_irq"], r0["why"], r0["cs_irq"]))
                if abs(rec["fault_at_us"] - r0["fault_at_us"]) > 20 + (hi - lo):
                    diffs.append("%s: fault at %d us (base %d)" % (label, rec["fault_at_us"], r0["fault_at_us"]))
                continue
            d = []
            if rec["why"] != r0["why"]: d.append("end %s" % rec["why"])
            if rec["elapsed_us"] < lo - tol or rec["elapsed_us"] > hi + tol: d.append("time %d us (base %d..%d)" % (rec["elapsed_us"], lo, hi))
            if rec["cs_irq"] != r0["cs_irq"]: d.append("CS_IRQ %x (base %x)" % (rec["cs_irq"], r0["cs_irq"]))
            for k in ("vic0_seen", "vic1_seen", "vic1_end"):
                if rec.get(k) != r0.get(k): d.append("%s %x (base %x)" % (k, rec.get(k), r0.get(k)))
            if rec["r"] != r0["r"]: d.append("r %s" % rec["r"])
            fm = _fm_diff(r0, rec)
            if fm: d.append("window after (a run changed it) " + ", ".join("%03x %08x/%08x" % x for x in fm[:6]))
            if d: diffs.append("%s: %s" % (label, "; ".join(d)))
        lines.append("  bit %2d (0x%08x): %s" % (bit, w, " | ".join(diffs) or "no difference"))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "start-word bits flipped one at a time from 0xfff5 (differences from the baseline only):\n" + "\n".join(lines)
