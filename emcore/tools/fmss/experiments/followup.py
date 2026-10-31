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

"""Fields the original programs leave 0, the timeout length, the ECC code, and the status latch with interrupt
enables."""
import struct
import sys

from ..probe import probe, assemble, soft_reset, WAIT_END, FULLMAP
from .registry import experiment


@experiment("fields0", needs=("pld_pst",))
def ex_fields0(s, t):
    """The fields the original programs always leave 0: end's R / arg / imm, wr's R, movi's arg, and the imm of st, ldi,
    mov, sti, pld, pst. Each with a mask-like (0xff) and an offset-like (4) value: does anything change?"""
    a = t.scratch_addr(s)
    pram = [(0xd70, 0x11111111), (0xd74, 0x22222222), (0xd78, 0x33333333)]
    scr = struct.pack("<4I", 0xaaaaaaaa, 0xbbbbbbbb, 0xcccccccc, 0xdddddddd) + bytes(240)
    seed = ("    movi r1, #0x%x\n    movi r2, #0x0\n    movi r3, #0x0\n    movi r4, #0x0\n    movi r5, #0x0d70\n"
            "    movi r7, #0x7777cafe\n    movi r6, #0x0\n" % (a + 4))
    cases = []
    for imm in (0xff, 0x4):
        cases += [("st [0xd70], r7, imm 0x%x" % imm, "    .insn 0x02, 7, 0xd70, 0x%x\n" % imm),
                  ("ldi r2, [r1], imm 0x%x" % imm, "    .insn 0x03, 2, 1, 0x%x\n" % imm),
                  ("mov r2, r7, imm 0x%x" % imm, "    .insn 0x06, 2, 7, 0x%x\n" % imm),
                  ("sti [r1], r7, imm 0x%x" % imm, "    .insn 0x11, 7, 1, 0x%x\n" % imm),
                  ("pld r2, [r5], imm 0x%x" % imm, "    .insn 0x18, 2, 5, 0x%x\n" % imm),
                  ("pst [r5], r7, imm 0x%x" % imm, "    .insn 0x19, 7, 5, 0x%x\n" % imm)]
    cases += [("wr [0xd70], #x, R 3", "    .insn 0x01, 3, 0xd70, 0x44444444\n"),
              ("wr [0xd70], #x, R 7", "    .insn 0x01, 7, 0xd70, 0x44444444\n"),
              ("movi r2, #x, arg 0x1234", "    .insn 0x05, 2, 0x1234, 0x55555555\n"),
              ("movi r2, #x, arg 0xd70", "    .insn 0x05, 2, 0xd70, 0x55555555\n")]
    out, lines = {}, []
    for label, body in cases:
        src = t.prologue(c58=0) + seed + body + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=pram, timeout_us=20000, flags=WAIT_END, scratch=scr)
        sc = rec["scratch"] or bytes(16)
        out[label] = rec
        lines.append("  %-28s -> %-6s r2 %08x  0xd70..0xd78 %08x %08x %08x  RAM +0..+12 %s" % (
            label, rec["why"], rec["r"][2], rec["after"][0xd70], rec["after"][0xd74], rec["after"][0xd78],
            " ".join("%08x" % v for v in struct.unpack_from("<4I", sc, 0))))
    # end with fields: which value comes back as SEQ_RESULT?
    for label, ins in (("end", "    end\n"), ("end R 1", "    .insn 0x00, 1, 0, 0\n"),
                       ("end arg 1", "    .insn 0x00, 0, 1, 0\n"), ("end imm 0x12345678", "    .insn 0x00, 0, 0, 0x12345678\n"),
                       ("end R 1, arg 7, imm 0xff", "    .insn 0x00, 1, 7, 0xff\n")):
        src = t.prologue(c58=0) + "    movi r1, #0x11111111\n    movi r7, #0x77777777\n    movi r6, #0x66666666\n" + ins + \
            "    movi r6, #0xbad0bad0\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-28s -> %-6s SEQ_RESULT %08x, r6 %08x, CS_IRQ %08x" % (
            label, rec["why"], rec["seq_result"], rec["r"][6], rec["cs_irq"]))
    return out, ("fields left 0 by the original programs (expected without an effect: r2 = 7777cafe / aaaaaaaa / "
                 "11111111, the word written at its plain place):\n") + "\n".join(lines)


@experiment("tmofield", needs=("start_value",))
def ex_tmofield(s, t):
    """The timeout length: start-word bit 2 enables it; which bits set the length, in what unit?"""
    words = [0x0005, 0x0105, 0x0205, 0x0405, 0x1005, 0x4005, 0x8005, 0xff05, 0x00f5, 0x0ff5, 0x0f05, 0xfff5]
    out, lines = {}, []
    src = t.prologue(c58=0) + "    movi r6, #0x0\n    wait 3\n    ld   r2, [FMSS_C14]\n    movi r6, #0x600d600d\n    end\n" + \
        t.trap()
    code = assemble(t, src)
    for w in words:
        rec = probe(s, t, code, timeout_us=40000, start_value=w, flags=WAIT_END)
        out["0x%04x" % w] = rec
        lines.append("  start 0x%04x -> %-6s fault at %6d us, C14 after the fault %02x" % (
            w, rec["why"], rec["fault_at_us"], rec["r"][2]))
    return out, "time from the start to the timeout of a wait that is never met:\n" + "\n".join(lines)


@experiment("eccstr2", needs=("nand_seq",))
def ex_eccstr2(s, t):
    """ECC strength and code, with encode and decode in one program (the set-up word at a program's start clears the
    buffers): 0..12 bits flipped before the decode; parity of single-bit sectors for matching the code offline"""
    a = t.scratch_addr(s)
    base = bytes((i * 37 + 11) & 0xff for i in range(512))

    def flip(data, bits):
        b = bytearray(data)
        for bit in bits:
            b[bit // 8] ^= 1 << (bit % 8)
        return bytes(b)

    def prog(decode):
        # RAM: +0x000 the sector to encode, +0x200 the sector to decode, +0x400 the decoded result, +0x600 the parity
        dma_in = ("    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
                  "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 2\n    wr   [FMC_STATUS], #0x100000\n")
        kick = ("    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x%x\n    wait 4\n")
        src = t.prologue(c58=0) + "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n" + \
            dma_in % a + kick % 0x1102 + \
            "    wr   [FMC_DATA_SEL], #0x1000\n    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n    wait 2\n" \
            "    wr   [FMC_STATUS], #0x100000\n" % (a + 0x600)
        if decode:
            src += dma_in % (a + 0x200) + kick % 0x1101 + "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" \
                "    wr   [FMC_DATA_SEL], #0x100\n    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n    wait 2\n" \
                "    wr   [FMC_STATUS], #0x100000\n" % (a + 0x400)
        return src + "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    out, lines = {}, []
    dec = assemble(t, prog(True))
    for k in range(13):
        bad = flip(base, [(j * 409 + 13) % 4096 for j in range(k)])
        rec = probe(s, t, dec, timeout_us=40000, flags=WAIT_END, scratch=base + bad + b"\xee" * 1024,
                    scratch_read=0x610, quiet=True)
        sc = rec["scratch"] or bytes(0x610)
        back = sc[0x400:0x600]
        out["flips %d" % k] = rec
        lines.append("  %2d bits flipped -> %-6s ECC_RESULT %08x (uncorrectable %d, corrected %2d), ECC_DONE %02x, "
                     "data %s" % (k, rec["why"], rec["r"][2], rec["r"][2] & 1, rec["r"][2] >> 16 & 0xf, rec["r"][3],
                                  "corrected" if back == base else "as sent" if back == bad else "other"))
    enc = assemble(t, prog(False))
    for label, data in [("zeros", bytes(512)), ("ones", b"\xff" * 512)] + \
            [("bit %d" % b, flip(bytes(512), [b])) for b in (0, 1, 2, 7, 8, 9, 4088, 4095)]:
        rec = probe(s, t, enc, timeout_us=40000, flags=WAIT_END, scratch=data + bytes(0x400) + b"\xee" * 16,
                    scratch_read=0x610, quiet=True)
        par = (rec["scratch"] or bytes(0x610))[0x600:0x610]
        out["parity/%s" % label] = rec
        lines.append("  parity of %-9s: %s" % (label, par.hex()))
    soft_reset(s, t)
    return out, "ECC, encode and decode in one program:\n" + "\n".join(lines)


@experiment("ev5en", needs=("nand_seq",))
def ex_ev5en(s, t):
    """The status latch failed (FMC_STATUS b24 instead of b23, no event 5) in every run with an interrupt enable
    written: repeat with no enable / single enable bits / the ECC enable, three times each, and with the enable written
    by the program instead of the CPU"""
    stat = "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n"
    out, lines = {}, []
    for label, params, pre in (("no enable", [], ""), ("0x044 = 1 (CPU)", [(0x044, 1)], ""),
                               ("0x044 = 0x2000 (CPU)", [(0x044, 0x2000)], ""), ("0x844 = 1 (CPU)", [(0x844, 1)], ""),
                               ("0x044 = 1 (program)", [], "    wr   [0x044], #0x1\n")):
        row = []
        for n in range(3):
            soft_reset(s, t)
            src = t.prologue(c58=0) + "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n" + \
                pre + stat + "    ld   r2, [FMC_STATUS]\n    wait 5\n    ld   r3, [FMC_STATUS]\n    ld   r4, [CS_IRQ]\n" \
                "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), params=params, timeout_us=20000, flags=WAIT_END)
            out["%s/%d" % (label, n)] = rec
            row.append("%s (before wait %08x, after %08x)" % ("T" if rec["fault_at_us"] else "ok", rec["r"][2], rec["r"][3]))
        lines.append("  %-22s %s" % (label, " | ".join(row)))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "status latch and wait 5 (T = wait 5 timed out), FMC_STATUS right after the latch and after the wait:\n" + \
        "\n".join(lines)


@experiment("tmobits", needs=("start_value",))
def ex_tmobits(s, t):
    """Timeout length, second sweep: which of start-word bits 7:4 enable or scale the count (bits 7:4 = 0 gave an
    immediate timeout, 0xf the long one), and the length against bits 15:8 with bits 7:4 = 0xf"""
    words = [0xff15, 0xff25, 0xff45, 0xff85, 0xff35, 0xffc5, 0x0f15, 0x0f25, 0x0f45, 0x0f85,
             0x01f5, 0x03f5, 0x07f5, 0x3ff5, 0x7ff5]
    out, lines = {}, []
    code = assemble(t, t.prologue(c58=0) + "    movi r6, #0x0\n    wait 3\n    ld   r2, [FMSS_C14]\n"
                    "    movi r6, #0x600d600d\n    end\n" + t.trap())                  # as tmofield
    for w in words:
        rec = probe(s, t, code, timeout_us=40000, start_value=w, flags=WAIT_END)
        out["0x%04x" % w] = rec
        lines.append("  start 0x%04x -> %-6s fault at %6d us, C14 after the fault %02x" % (
            w, rec["why"], rec["fault_at_us"], rec["r"][2]))
    return out, "time from the start to the timeout of a wait that is never met:\n" + "\n".join(lines)


def _dma_in(sel, addr, ev):
    return ("    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x%x\n    wr   [FMC_DMA_ADDR], #0x%x\n"
            "    wr   [FMCTRL1], #0x2e0\n    wait %d\n    wr   [FMC_STATUS], #0x100000\n" % (sel, addr, ev))


def _dma_out(sel, addr, ev):
    return ("    wr   [FMC_DATA_SEL], #0x%x\n    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n    wait %d\n"
            "    wr   [FMC_STATUS], #0x100000\n" % (sel, addr, ev))


def _kick(word, mode, ev):
    return ("    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x%x\n    wr   [ECC_KICK], #0x%x\n    wait %d\n" %
            (mode, word, ev))


def _flip(data, bits):
    b = bytearray(data)
    for bit in bits:
        b[bit // 8] ^= 0x80 >> (bit % 8)          # bit 0 = MSB of byte 0, the code's highest power
    return bytes(b)


@experiment("eccmode", needs=("nand_seq", "fullmap"))
def ex_eccmode(s, t):
    """ECC_MODE values other than 0x180: does the parity (slot and 0x900 area, whole ECC block dumped) or the strength
    change? Per mode: encode a sector, then decode it with 0 / 8 / 9 / 12 / 16 flipped data bits in the same program"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    base = bytes((i * 37 + 11) & 0xff for i in range(512))
    out, lines = {}, []
    for mode in (0x180, 0x0, 0x80, 0x100, 0x1c0, 0x181, 0x380, 0x780, 0xe00000, 0x1ffffff):
        row = []
        for k in (0, 8, 9, 12, 16):
            src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + \
                _dma_in(0x1, a, e["dma"]) + _kick(0x1102, mode, e["ecc"]) + \
                "    ld   r4, [ECC_DONE]\n" + _dma_out(0x1000, a + 0x600, e["dma"]) + \
                _dma_in(0x1, a + 0x200, e["dma"]) + _kick(0x1101, mode, e["ecc"]) + \
                "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + _dma_out(0x100, a + 0x400, e["dma"]) + \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            bad = _flip(base, [(j * 409 + 13) % 4096 for j in range(k)])
            rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                        scratch=base + bad + b"\xee" * 0x400, scratch_read=0x810, quiet=True)
            sc = rec["scratch"] or bytes(0x810)
            par, back = sc[0x600:0x640], sc[0x400:0x600]
            out["0x%x/%d" % (mode, k)] = rec
            if k == 0:
                area = b"".join(struct.pack("<I", w) for w in rec["fullmap"][1][0x900 // 4:0x940 // 4]) \
                    if rec.get("fullmap") else b""
                std = par[:16] == bch.chunk(base, par[:3])
                lines.append("  mode 0x%07x: %s; encode done %02x; slot 0 %s (%s); slots 1..3 %s; 0x900..0x93c %s" % (
                    mode, rec["why"], rec["r"][4], par[:16].hex(), "the 0x180 code" if std else "DIFFERENT",
                    par[16:64].hex(), area.hex()))
            row.append("%2d: %08x/%02x %s" % (k, rec["r"][2], rec["r"][3],
                                              "corr" if back == base else "sent" if back == bad else "other"))
        lines.append("      decode " + " | ".join(row))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "ECC_MODE sweep (decode: flipped bits: ECC_RESULT/ECC_DONE, data corrected / as sent / other):\n" + \
        "\n".join(lines)


@experiment("eccchunk", needs=("nand_seq",))
def ex_eccchunk(s, t):
    """(1) Can a parity slot be filled by DMA from memory (needed for raw writes and for chunk tests)? (2) The meta
    encode: FMC_META0..2 + METACFG 0x3210 + METACTL 1, then an ECC encode: where do the 12 bytes land, and are they
    in the parity? (3) Decodes of host-built chunks (the identified code) with errors in the metadata / parity bytes"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    pat = bytes((i * 13 + 5) & 0xff for i in range(512))
    # (1) DMA into the parity buffer, then out again
    for sel in (0x10, 0x20):
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + \
            _dma_in(sel, a, e["dma"]) + _dma_out(0x1000 if sel == 0x10 else 0x2000, a + 0x200, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=pat + b"\xee" * 0x200,
                    scratch_read=0x400, quiet=True)
        got = (rec["scratch"] or bytes(0x400))[0x200:0x240]
        out["pdma 0x%x" % sel] = rec
        lines.append("  DMA memory -> parity buffer (DATA_SEL 0x%x) -> memory: %s, first 64 bytes back %s" % (
            sel, rec["why"], "= sent" if got == pat[:64] else got.hex()))
    # (2) meta encode then ECC encode of slot 0..3
    meta = bytes(range(0xa0, 0xac))
    src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
        "    wr   [FMC_META0], #0x%x\n    wr   [FMC_META1], #0x%x\n    wr   [FMC_META2], #0x%x\n" % \
        struct.unpack("<3I", meta) + "    wr   [FMC_METACFG], #0x3210\n    wr   [FMC_METACTL], #0x1\n" \
        "poll:\n    ld   r1, [FMC_METACTL], 0x1\n    bnz  r1, poll\n" + \
        "".join(_kick(0x1102 | k << 16, 0x180, e["ecc"]) for k in range(4)) + _dma_out(0x1000, a + 0x200, e["dma"]) + \
        "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=pat + b"\xee" * 0x200,
                scratch_read=0x400, quiet=True)
    par = (rec["scratch"] or bytes(0x400))[0x200:0x240]
    out["meta encode"] = rec
    lines.append("  meta encode (META = a0..ab, 0x3210) + encode slots 0..3: %s; slots: %s" % (rec["why"], " ".join(
        "%s(%s)" % (par[16 * k:16 * k + 16].hex(), "code ok" if bch.is_codeword(pat, par[16 * k:16 * k + 16]) else
                    "no") for k in range(4))))
    # (3) host-built chunks into slot 0 by DMA, decode
    m3 = b"\x12\x34\x56"
    good = bch.chunk(pat, m3)
    cases = [("clean", pat, good), ("meta 1 bit", pat, _flip(good, [3])), ("meta 3 bits", pat, _flip(good, [0, 9, 23])),
             ("parity 1 bit", pat, _flip(good, [40])), ("parity 8 bits", pat, _flip(good, range(24, 128, 13))),
             ("data 4 + parity 4", _flip(pat, [7, 900, 2000, 4095]), _flip(good, [30, 60, 90, 120])),
             ("parity 9 bits", pat, _flip(good, range(24, 128, 11))),
             ("meta byte order: chunk built with the bytes reversed", pat, bch.chunk(pat, m3[::-1])[:3][::-1] +
              bch.chunk(pat, m3[::-1])[3:])]
    for label, data, ch in cases:
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + \
            _dma_in(0x1, a, e["dma"]) + _dma_in(0x10, a + 0x200, e["dma"]) + _kick(0x1101, 0x180, e["ecc"]) + \
            "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + _dma_out(0x100, a + 0x400, e["dma"]) + \
            _dma_out(0x1000, a + 0x600, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END,      # the parity buffer keeps the
                    scratch=data + ch.ljust(64, b"\xff").rjust(512, b"\xff") + b"\xee" * 0x400,  # DMA's last 64 bytes
                    scratch_read=0x810, quiet=True)
        sc = rec["scratch"] or bytes(0x810)
        back, slot = sc[0x400:0x600], sc[0x600:0x610]
        out["decode/" + label] = rec
        lines.append("  decode, %-52s -> %-5s ECC_RESULT %08x ECC_DONE %02x, data %s, slot %s" % (
            label, rec["why"], rec["r"][2], rec["r"][3], "clean" if back == pat else "as sent" if back == data else
            "other", "clean" if slot == good else "as sent" if slot == ch else slot.hex()))
    soft_reset(s, t)
    return out, "parity buffer DMA, meta encode, decode of host-built chunks:\n" + "\n".join(lines)


def _ready(t, n):
    """C7-style status poll until the chip reports ready (a fresh 70h per latch); NSTAT ends in r7. FMC_DATA_SEL = 1
    is required (C7): with the drain select a write leaves (0x100), the latch returned the last command byte (0x70,
    whose b6 reads as ready) and the poll ended while the chip was still programming"""
    e = t.events
    return ("rdy%s:\n    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x1\n" % n + t.cmd(0x70) +
            "    wr   [FMCTRL1], #0xca\n    wait %d\n    wr   [FMC_STATUS], #0x800000\n    ld   r1, [FMC_NSTAT], 0x40\n"
            "    bz   r1, rdy%s\n    ld   r7, [FMC_NSTAT], 0xff\n    wr   [FMCTRL1], #0x20\n" % (e["status"], n))


def _address(t, row):
    return ("    wr   [FMC_ANUM], #0x4\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMC_ADDR1], #0x%x\n    wr   [FMCTRL1], #0x1\n"
            "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % ((row & 0xffff) << 16, row >> 16, t.events["addr"]))


def _send(sel, dnum, ctl, extra=""):
    return ("    wr   [FMC_DNUM], #0x%x\n    wr   [FMC_DATA_SEL], #0x%x\n%s    wr   [FMCTRL1], #0x%x\n    wait 3\n"
            "    wr   [FMC_STATUS], #0x8\n" % (dnum, sel, extra, ctl))


def _raw_read(t, s, row, ce=0, start_value=0):
    """page `row` of chip enable `ce`: per sector the chunk and data as stored (DMA before the decode), the decode's ECC_RESULT
    (0xd70 + 4k), FMC_STATUS after the data (0xd60 + 4k), the data after the decode (+0x1000)"""
    a, e = t.scratch_addr(s), t.events
    sel, seld = "0x%x" % t.ce_sel[ce], "0x%x" % (t.ce_sel[ce] | t.ce_dma)
    src = t.nand_prologue(ce) + t.cmd(0x00) + _address(t, row) + "    wr   [FMC_CMD], #0x30\n    wait %d\n" \
        "    wr   [FMC_STATUS], #0x2\n" % e["cmd"] + _ready(t, 0) + "    wr   [FMCTRL1], #0x100e0\n" + t.cmd(0x00)
    for k in range(4):
        src += ("    wr   [FMC_CTRL0], #%s\n" % sel + _send(0x10, 0xf, 0x32, "    wr   [FMC_24], #0x%x\n" % (16 * k)) +
                _send(0x1, 0x1ff, 0xe2) + "    ld   r5, [FMC_STATUS]\n    st   [0x%x], r5\n    wr   [FMC_CTRL0], #%s\n"
                % (0xd60 + 4 * k, seld) + _dma_out(0x100, a + 0x200 * k, e["dma"]) +
                _dma_out(0x1000, a + 0x800 + 0x200 * k, e["dma"]) + _kick(0x1101 | k << 16, 0x180, e["ecc"]) +
                "    ld   r2, [ECC_RESULT]\n    st   [0x%x], r2\n" % (0xd70 + 4 * k) +
                _dma_out(0x100, a + 0x1000 + 0x200 * k, e["dma"]))
    src += "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=80000, flags=WAIT_END, scratch=b"\xee" * 0x1800,
                scratch_read=0x1800, quiet=True, start_value=start_value)
    sc = rec["scratch"] or bytes(0x1800)
    secs = []
    for k in range(4):
        secs.append((sc[0x200 * k:0x200 * k + 0x200], sc[0x800 + 0x210 * k:0x800 + 0x210 * k + 16],
                     sc[0x1000 + 0x200 * k:0x1200 + 0x200 * k], rec["after"][0xd70 + 4 * k], rec["after"][0xd60 + 4 * k]))
    return rec, secs


def _verdict(bch, d, ch):
    if d == b"\xff" * 512 and ch == b"\xff" * 16:
        return "erased"
    if bch.is_codeword(d, ch):
        return "code ok"
    if bch.is_codeword(d, ch[2::-1] + ch[3:]):
        return "code ok, meta reversed"
    return "NOT a code word"


@experiment("rawpage", needs=("nand_seq",))
def ex_rawpage(s, t):
    """Raw read (chunk and data as stored, before the correction) of pages the original firmware wrote, checked against
    the identified code on the host; plus the engine's decode of them"""
    from .. import bch
    out, lines = {}, []
    for row in (0x0, 0x80, 0x80 * 100 + 1, 0x80 * 1000, 0x80 * 2000 + 5, 0x80 * 3500):
        rec, secs = _raw_read(t, s, row)
        out["row 0x%x" % row] = rec
        lines.append("  row 0x%06x %-5s " % (row, rec["why"]) + "\n                   ".join(
            "s%d %-22s res %08x st %08x chunk %s" % (k, _verdict(bch, d, ch), res, st, ch.hex())
            for k, (d, ch, dec, res, st) in enumerate(secs)))
        print(lines[-1])
        sys.stdout.flush()
    return out, "raw page reads, CE0 (chunk as stored, checked against the identified code; res = engine decode, " \
        "st = FMC_STATUS after the sector's data):\n" + "\n".join(lines)


@experiment("rawwrite", needs=("nand_seq",))
def ex_rawwrite(s, t):
    """DESTRUCTIVE (scratch block of CE0): erase, then program pages by every path and read them back raw.
    p0 the engine's own path (meta encode + encode, parity slice from the parity buffer): the on-flash layout and
    metadata order; p1 host-built chunks DMA-ed into the parity buffer; p2 host-built chunks sent from a data buffer
    (no ECC engine at all: a raw write); p3 errors stored on the flash (8 / 9 data bits, a zero chunk, 2 metadata
    bits); p4 left erased"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    base_row = t.scratch_row
    out, lines = {}, []
    src = t.nand_prologue(0) + t.erase + _ready(t, 0) + "    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)
    out["erase"] = rec
    lines.append("  erase block: %s, NSTAT %02x" % (rec["why"], rec["r"][7]))
    data = [bytes((i * (7 + 2 * p + k) + 3 * k + p) & 0xff for i in range(512)) for p in range(4) for k in range(4)]
    meta = [bytes(range(0x10 * p + 0x41, 0x10 * p + 0x4d)) for p in range(4)]

    def head(row):
        return t.nand_prologue(0) + t.cmd(0x80) + _address(t, row) + "    wr   [FMC_CTRL0], #@CE0_DMA@\n"

    def tail():
        return "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x10) + _ready(t, 0) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    expect = {}
    # p0: engine path; scratch: data 0..0x800
    src = head(base_row)
    for k in range(4):
        src += _dma_in(0x1, a + 0x200 * k, e["dma"])
        if k == 0:
            src += ("    wr   [FMC_META0], #0x%x\n    wr   [FMC_META1], #0x%x\n    wr   [FMC_META2], #0x%x\n" %
                    struct.unpack("<3I", meta[0]) + "    wr   [FMC_METACFG], #0x3210\n    wr   [FMC_METACTL], #0x1\n"
                    "mpoll:\n    ld   r1, [FMC_METACTL], 0x1\n    bnz  r1, mpoll\n")
        src += _kick(0x1102 | k << 16, 0x180, e["ecc"]) + _send(0x1000, 0xf, 0x34, "    wr   [FMC_28], #0x%x\n" % (16 * k)) + \
            _send(0x100, 0x1ff, 0xe4)
    rec = probe(s, t, assemble(t, src + tail()), timeout_us=60000, flags=WAIT_END, scratch=b"".join(data[0:4]))
    out["write p0 engine"] = rec
    lines.append("  p0 write (engine: meta encode + encode): %s, NSTAT %02x" % (rec["why"], rec["r"][7]))
    expect[0] = [(data[k], None) for k in range(4)]
    # p1: host chunks into the parity buffer by DMA; scratch: data 0..0x800, chunks 0x800..0x840
    ch1 = [bch.chunk(data[4 + k], meta[1][3 * k:3 * k + 3]) for k in range(4)]
    src = head(base_row + 1) + _dma_in(0x10, a + 0x800, e["dma"])
    for k in range(4):
        src += _send(0x1000, 0xf, 0x34, "    wr   [FMC_28], #0x%x\n" % (16 * k)) + _dma_in(0x1, a + 0x200 * k, e["dma"]) + \
            _send(0x100, 0x1ff, 0xe4)
    rec = probe(s, t, assemble(t, src + tail()), timeout_us=60000, flags=WAIT_END,
                scratch=b"".join(data[4:8]) + b"".join(ch1).rjust(512, b"\xff"))   # last 64 bytes stay in the buffer
    out["write p1 parity DMA"] = rec
    lines.append("  p1 write (host chunks via parity-buffer DMA): %s, NSTAT %02x" % (rec["why"], rec["r"][7]))
    expect[1] = [(data[4 + k], ch1[k]) for k in range(4)]

    # p2 / p3: chunks from a data buffer (no ECC engine); scratch: data 0..0x800, chunk k at 0x800 + 0x200 k
    def raw_page(row, secs):
        src = head(row)
        for k in range(4):
            src += _dma_in(0x1, a + 0x800 + 0x200 * k, e["dma"]) + _send(0x100, 0xf, 0xe4) + \
                _dma_in(0x1, a + 0x200 * k, e["dma"]) + _send(0x100, 0x1ff, 0xe4)
        return probe(s, t, assemble(t, src + tail()), timeout_us=60000, flags=WAIT_END,
                     scratch=b"".join(d for d, c in secs) + b"".join(c.ljust(512, b"\xff") for d, c in secs))
    secs2 = [(data[8 + k], bch.chunk(data[8 + k], meta[2][3 * k:3 * k + 3])) for k in range(4)]
    rec = raw_page(base_row + 2, secs2)
    out["write p2 raw"] = rec
    lines.append("  p2 write (raw: chunks from a data buffer): %s, NSTAT %02x" % (rec["why"], rec["r"][7]))
    expect[2] = secs2
    good = [bch.chunk(data[12 + k], meta[3][3 * k:3 * k + 3]) for k in range(4)]
    secs3 = [(_flip(data[12], [(j * 509 + 1) % 4096 for j in range(8)]), good[0]),
             (_flip(data[13], [(j * 509 + 1) % 4096 for j in range(9)]), good[1]),
             (data[14], bytes(16)),
             (data[15], _flip(good[3], [5, 17]))]
    rec = raw_page(base_row + 3, secs3)
    out["write p3 errors"] = rec
    lines.append("  p3 write (raw, stored errors: 8 / 9 data bits, zero chunk, 2 meta bits): %s, NSTAT %02x" % (
        rec["why"], rec["r"][7]))
    expect[3] = secs3
    clean3 = [data[12 + k] for k in range(4)]
    # read back
    for p in range(5):
        rec, secs = _raw_read(t, s, base_row + p)
        out["read p%d" % p] = rec
        for k, (d, ch, dec, res, st) in enumerate(secs):
            if p == 0:
                cmp = "data %s; meta bytes %s (sent %s)" % ("= sent" if d == data[k] else "DIFFERENT", ch[:3].hex(),
                                                            meta[0][3 * k:3 * k + 3].hex())
            elif p < 4:
                sd, sc_ = expect[p][k]
                cmp = "stored %s" % ("= sent" if (d, ch) == (sd, sc_) else "data %s, chunk %s (sent %s)" % (
                    "=" if d == sd else "DIFFERENT", ch.hex(), sc_.hex()))
                if p == 3:
                    cmp += "; after decode %s" % ("clean" if dec == clean3[k] else "as stored" if dec == d else "other")
            else:
                cmp = ""
            lines.append("  p%d s%d %-22s res %08x st %08x chunk %s %s" % (p, k, _verdict(bch, d, ch), res, st,
                                                                         ch.hex(), cmp))
        print("\n".join(lines[-4:]))
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "scratch block write paths and raw read-back (res = engine decode, st = FMC_STATUS after the data):\n" + \
        "\n".join(lines)


def _erase_src(t, rows, n, two_plane=False):
    """erase of each row's block on the selected chip (60h, 3 row bytes, D0h), NSTAT right after D0h in r3, the
    ready poll's NSTAT in r7; two_plane: 60h row0 60h row1 D0h"""
    e = t.events
    src = ""
    for i, row in enumerate(rows):
        src += t.cmd(0x60) + "    wr   [FMC_ANUM], #0x2\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMCTRL1], #0x1\n" \
            "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % (row, e["addr"])
        if not two_plane or i == len(rows) - 1:
            src += t.cmd(0xd0) + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait %d\n" \
                "    wr   [FMC_STATUS], #0x800000\n    ld   r3, [FMC_NSTAT], 0xff\n" % e["status"] + \
                _ready(t, "%s_%d" % (n, i))
    return src


def _meta_encode(meta, n):
    return ("    wr   [FMC_META0], #0x%x\n    wr   [FMC_META1], #0x%x\n    wr   [FMC_META2], #0x%x\n" % struct.unpack("<3I", meta) +
            "    wr   [FMC_METACFG], #0x3210\n    wr   [FMC_METACTL], #0x1\nmp%s:\n    ld   r1, [FMC_METACTL], 0x1\n"
            "    bnz  r1, mp%s\n" % (n, n))


def _pages(n_pages, salt):
    data = [bytes((i * (5 + 2 * k + salt) + 7 * k + 3 * salt) & 0xff for i in range(512)) for k in range(4 * n_pages)]
    meta = [bytes((0x21 + 13 * p + 3 * salt + i) & 0xff for i in range(12)) for p in range(n_pages)]
    return data, meta


@experiment("writeseq", needs=("nand_seq",))
def ex_writeseq(s, t):
    """DESTRUCTIVE (scratch blocks): the write sequences the original firmware uses, checked by raw read-back.
    (1) four pages in one run with the overlapped transfer 0x2e4 (send one sector while the next is DMA-ed into the
    other buffer), FMC_DMA_CTRL = 7 before the last page, the last sector sent with 0xe4, a meta encode per page and a
    ready poll before the next 80h; (2) one run that programs a page on CE0 and one on CE1 back to back, then polls
    both; (3) a two-plane pair (80h .. 11h, poll, 81h .. 10h) on blocks B and B+1; (4) a page programmed a second
    time without an erase (does NSTAT report FAIL, what is stored?)"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    B, rpb = t.scratch_block, t.rows_per_block
    out, lines = {}, []
    word = {ce: (t.ce_sel[ce], t.ce_sel[ce] | t.ce_dma) for ce in (0, 1)}
    # erase the blocks used: B and B+1 on CE0, B on CE1
    for ce, blk in ((0, B), (0, B + 1), (1, B)):
        src = t.nand_prologue(ce) + _erase_src(t, [blk * rpb], "e") + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)
        out["erase CE%d block %d" % (ce, blk)] = rec
        lines.append("  erase CE%d block %d: %s, NSTAT after D0h %02x, ready %02x, %d us" % (
            ce, blk, rec["why"], rec["r"][3], rec["r"][7], rec["elapsed_us"]))

    def program(ce, row, data, meta, n, confirm=0x10, first=0x80, stage=True):
        """one page; data at RAM a + 0x200 * k (k = 0..3); buffers: sector k drains buffer k & 1"""
        c, cd = word[ce]
        src = "    wr   [FMC_CTRL0], #0x%x\n" % cd
        if stage:
            src += _dma_in(0x1, a + data, e["dma"])                       # WR1: sector 0 into buffer 0
        src += "    wr   [FMC_CTRL0], #0x%x\n" % c + t.cmd(first) + _address(t, row) + _meta_encode(meta, n) + \
            "    wr   [FMC_CTRL0], #0x%x\n" % cd
        for k in range(4):
            cur = k & 1
            src += _kick(0x1002 | (0x100 << cur) | k << 16, 0x180, e["ecc"]) + \
                _send(0x1000, 0xf, 0x34, "    wr   [FMC_28], #0x%x\n" % (16 * k))
            if k < 3:                                                    # drain cur, fill the other with sector k+1
                src += "    wr   [FMC_DMA_ADDR], #0x%x\n" % (a + data + 0x200 * (k + 1)) + \
                    _send((0x100 << cur) | (1 << (1 - cur)), 0x1ff, 0x2e4) + \
                    "    wait %d\n    wr   [FMC_STATUS], #0x100000\n" % e["dma"]
            else:
                src += _send(0x100 << cur, 0x1ff, 0xe4)
        return src + "    wr   [FMC_CTRL0], #0x%x\n" % c + t.cmd(confirm)

    def run(src, scratch):
        return probe(s, t, assemble(t, src + "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" +
                                    t.trap()), timeout_us=100000, flags=WAIT_END, scratch=scratch)

    def check(ce, row, data, meta, label):
        rec, secs = _raw_read(t, s, row, ce)
        bad = [k for k, (d, ch, dec, res, st) in enumerate(secs)
               if d != data[k] or ch != bch.chunk(data[k], meta[3 * k:3 * k + 3])]
        lines.append("  read %-34s %s; ECC_RESULT %s" % (
            label, "all 4 sectors = expected (data + meta + parity)" if not bad else "sectors %s DIFFER: %s" % (
                bad, " ".join(secs[k][1].hex() for k in bad)), " ".join("%x" % x[3] for x in secs)))
        out["read " + label] = rec
        return secs

    # (1) four pages, one run, overlapped transfers, DMA_CTRL 7 before the last DMA
    data, meta = _pages(4, 1)
    src = t.nand_prologue(0)
    for p in range(4):
        if p == 3:
            src += "    wr   [FMC_DMA_CTRL], #0x7\n"
        src += program(0, B * rpb + 10 + p, 0x800 * p, meta[p], "a%d" % p) + _ready(t, "a%d" % p) + \
            "    st   [0x%x], r7\n" % (0xd70 + 4 * p)
    rec = run(src, b"".join(data))
    out["(1) 4 pages"] = rec
    lines.append("  (1) 4 pages, 0x2e4 overlap, DMA_CTRL 7: %s, NSTAT per page %s, %d us" % (
        rec["why"], " ".join("%02x" % rec["after"][0xd70 + 4 * p] for p in range(4)), rec["elapsed_us"]))
    for p in range(4):
        check(0, B * rpb + 10 + p, data[4 * p:4 * p + 4], meta[p], "(1) CE0 page %d" % (10 + p))
    # (2) CE0 and CE1 in one run
    data2, meta2 = _pages(2, 2)
    src = t.nand_prologue(0) + program(0, B * rpb + 20, 0, meta2[0], "b0") + \
        program(1, B * rpb + 20, 0x800, meta2[1], "b1") + \
        "    wr   [FMC_CTRL0], #0x%x\n" % word[0][0] + _ready(t, "b0") + "    st   [0xd70], r7\n" + \
        "    wr   [FMC_CTRL0], #0x%x\n" % word[1][0] + _ready(t, "b1") + "    st   [0xd74], r7\n"
    rec = run(src, b"".join(data2))
    out["(2) CE0 + CE1"] = rec
    lines.append("  (2) CE0 + CE1 in one run: %s, NSTAT %02x / %02x" % (rec["why"], rec["after"][0xd70],
                                                                     rec["after"][0xd74]))
    check(0, B * rpb + 20, data2[0:4], meta2[0], "(2) CE0 page 20")
    check(1, B * rpb + 20, data2[4:8], meta2[1], "(2) CE1 page 20")
    # (3) two-plane pair on blocks B and B+1
    data3, meta3 = _pages(2, 3)
    src = t.nand_prologue(0) + program(0, B * rpb + 30, 0, meta3[0], "c0", confirm=0x11) + _ready(t, "c0") + \
        "    st   [0xd70], r7\n" + program(0, (B + 1) * rpb + 30, 0x800, meta3[1], "c1", first=0x81) + _ready(t, "c1") + \
        "    st   [0xd74], r7\n"
    rec = run(src, b"".join(data3))
    out["(3) two-plane"] = rec
    lines.append("  (3) two-plane 80h..11h / 81h..10h: %s, NSTAT %02x / %02x, %d us" % (
        rec["why"], rec["after"][0xd70], rec["after"][0xd74], rec["elapsed_us"]))
    check(0, B * rpb + 30, data3[0:4], meta3[0], "(3) block B page 30")
    check(0, (B + 1) * rpb + 30, data3[4:8], meta3[1], "(3) block B+1 page 30")
    # (4) program page 10 again without an erase
    data4, meta4 = _pages(1, 4)
    src = t.nand_prologue(0) + program(0, B * rpb + 10, 0, meta4[0], "d0") + _ready(t, "d0")
    rec = run(src, b"".join(data4))
    out["(4) reprogram"] = rec
    lines.append("  (4) page 10 programmed again without an erase: %s, NSTAT %02x (b0 = FAIL)" % (rec["why"], rec["r"][7]))
    rec, secs = _raw_read(t, s, B * rpb + 10, 0)
    out["read (4)"] = rec
    for k, (d, ch, dec, res, st) in enumerate(secs):
        old, new = data[k], data4[k]
        lines.append("    s%d data %s, ECC_RESULT %08x" % (
            k, "= new" if d == new else "= old" if d == old else "= old AND new" if d == bytes(x & y for x, y in zip(old, new))
            else "other", res))
    soft_reset(s, t)
    return out, "write sequences (scratch blocks), raw read-back:\n" + "\n".join(lines)


@experiment("eraseseq", needs=("nand_seq",))
def ex_eraseseq(s, t):
    """DESTRUCTIVE (scratch blocks): the erase. Time of an erase (block with written pages / already erased); NSTAT
    right after D0h (busy?) and at ready; written pages read back erased; a two-plane erase (60h B, 60h B+1, D0h);
    the erase on CE1; an erase sent with 4 address cycles instead of 3 (FMC_ANUM 3: accepted or refused?)"""
    from .. import bch
    B, rpb = t.scratch_block, t.rows_per_block
    out, lines = {}, []
    e = t.events

    def erase(ce, rows, label, two_plane=False):
        src = t.nand_prologue(ce) + _erase_src(t, rows, "x", two_plane) + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)
        out[label] = rec
        lines.append("  %-44s %s, NSTAT after D0h %02x, ready %02x, %5d us" % (label, rec["why"], rec["r"][3],
                                                                              rec["r"][7], rec["elapsed_us"]))

    def page_state(ce, row, label):
        rec, secs = _raw_read(t, s, row, ce)
        out["read " + label] = rec
        lines.append("    %-42s %s" % (label, " ".join(_verdict(bch, d, ch) for d, ch, dec, res, st in secs)))

    for p in (10, 20, 30):
        page_state(0, B * rpb + p, "before: CE0 block B page %d" % p)
    erase(0, [B * rpb], "erase CE0 block B (written pages)")
    for p in (10, 20, 30):
        page_state(0, B * rpb + p, "after: CE0 block B page %d" % p)
    erase(0, [B * rpb], "erase CE0 block B again (already erased)")
    page_state(0, (B + 1) * rpb + 30, "before: CE0 block B+1 page 30")
    erase(0, [B * rpb, (B + 1) * rpb], "two-plane erase B, B+1", two_plane=True)
    page_state(0, (B + 1) * rpb + 30, "after: CE0 block B+1 page 30")
    page_state(1, B * rpb + 20, "before: CE1 block B page 20")
    erase(1, [B * rpb], "erase CE1 block B")
    page_state(1, B * rpb + 20, "after: CE1 block B page 20")
    # write a marker page in B+2 and B+2+(1<<16)/rpb, then erase with 3 row bytes (ANUM 3) of row B+2
    src = t.nand_prologue(0) + t.cmd(0x60) + "    wr   [FMC_ANUM], #0x3\n    wr   [FMC_ADDR0], #0x%x\n" \
        "    wr   [FMCTRL1], #0x1\n    wait %d\n    wr   [FMC_STATUS], #0x4\n" % ((B + 2) * rpb, e["addr"]) + t.cmd(0xd0) + \
        _ready(t, "z") + "    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)
    out["erase, 4 address bytes"] = rec
    lines.append("  %-44s %s, ready %02x, %5d us (a 4th address byte: accepted / ignored?)" % (
        "erase B+2 with FMC_ANUM = 3", rec["why"], rec["r"][7], rec["elapsed_us"]))
    soft_reset(s, t)
    return out, "erase (scratch blocks; per page: four sectors' state):\n" + "\n".join(lines)


@experiment("op09nand", needs=("nand_seq",))
def ex_op09nand(s, t):
    """DESTRUCTIVE (scratch blocks): does wait2 (opcode 09) track write / erase / read activity of the chip? wait2 N
    for N = 0..31 while a block erase runs, right after it finished, while a page program runs, right after it
    finished, and during a page read's tR; N = 32..255 idle and during an erase (never tried before). Per case:
    passed or timed out, the time it waited, and the chip's status right after it (b6 = ready)"""
    a, e = t.scratch_addr(s), t.events
    B, rpb = t.scratch_block, t.rows_per_block
    after = t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait %d\n    wr   [FMC_STATUS], #0x800000\n" \
        "    ld   r3, [FMC_NSTAT], 0xff\n" % e["status"]

    def erase(row):
        return t.cmd(0x60) + "    wr   [FMC_ANUM], #0x2\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMCTRL1], #0x1\n" \
            "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % (row, e["addr"]) + "    wr   [FMC_CMD], #0xd0\n    wait %d\n" \
            "    wr   [FMC_STATUS], #0x2\n" % e["cmd"]

    def program(row):
        return "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + "    wr   [FMC_CTRL0], #@CE0@\n" + \
            t.cmd(0x80) + _address(t, row) + _send(0x100, 0x1ff, 0xe4) + "    wr   [FMC_CMD], #0x10\n    wait %d\n" \
            "    wr   [FMC_STATUS], #0x2\n" % e["cmd"]

    def read(row):
        return t.cmd(0x00) + _address(t, row) + "    wr   [FMC_CMD], #0x30\n    wait %d\n    wr   [FMC_STATUS], #0x2\n" % e["cmd"]
    states = [("erase running", lambda n: erase((B + 6) * rpb)),
              ("erase finished", lambda n: erase((B + 6) * rpb) + _ready(t, "p")),
              ("program running", lambda n: program((B + 4) * rpb + n)),
              ("program finished", lambda n: program((B + 5) * rpb + n) + _ready(t, "p")),
              ("read tR running", lambda n: read((B + 4) * rpb + n))]
    out, lines = {}, []

    def one(label, n, body):
        src = t.nand_prologue(0) + body + "    movi r6, #0x0\n    .insn 0x09, %d, 0, 0\n" % n + after + \
            _ready(t, "q") + "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END, scratch=bytes(range(256)) * 2)
        out["%s/%d" % (label, n)] = rec
        return rec
    # the program's own status polls can time out too (a second poll after a chip operation does): a wait2 that
    # passed is one whose run took a whole timeout less than the slowest runs of the same state
    def passed(recs):
        worst = max(r["elapsed_us"] for r in recs.values())
        return [n for n, r in recs.items() if r["elapsed_us"] < worst - 3000]
    for label, mk in states:
        recs = {n: one(label, n, mk(n)) for n in range(32)}
        row = ["%d (%d us, NSTAT %02x)" % (n, recs[n]["elapsed_us"], recs[n]["r"][3]) for n in passed(recs)]
        ref = out["%s/%d" % (label, 0)]
        lines.append("  %-17s wait2 passes for: %s   [N = 0: %d us, NSTAT after %02x]" % (
            label, ", ".join(row) or "none", ref["elapsed_us"], ref["r"][3]))
        print(lines[-1])
        sys.stdout.flush()
    for label, mk in (("idle", lambda n: ""), ("erase running", states[0][1])):
        recs = {n: one(label, n, mk(n)) for n in range(32, 256)}
        row = ["%d (%d us)" % (n, recs[n]["elapsed_us"]) for n in passed(recs)]
        lines.append("  %-17s wait2 32..255 passes for: %s" % (label, ", ".join(row) or "none"))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "wait2 against chip activity (scratch blocks):\n" + "\n".join(lines)


@experiment("eccchunk2", needs=("nand_seq",))
def ex_eccchunk2(s, t):
    """Decodes of host-built chunks (the identified code) loaded into parity slot 0 by DMA (the chunk in the last 64
    bytes of the 512-byte DMA, which is what the 64-byte buffer keeps): errors in the metadata / parity bytes, mixed
    with data errors, and the metadata byte order"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    pat = bytes((i * 13 + 5) & 0xff for i in range(512))
    m3 = b"\x12\x34\x56"
    good = bch.chunk(pat, m3)
    cases = [("clean", pat, good), ("meta 1 bit", pat, _flip(good, [3])), ("meta 3 bits", pat, _flip(good, [0, 9, 23])),
             ("parity 1 bit", pat, _flip(good, [40])), ("parity 8 bits", pat, _flip(good, range(24, 128, 13))),
             ("data 4 + parity 4", _flip(pat, [7, 900, 2000, 4095]), _flip(good, [30, 60, 90, 120])),
             ("data 5 + meta 3", _flip(pat, [1, 2, 3, 4, 5]), _flip(good, [0, 9, 23])),
             ("parity 9 bits", pat, _flip(good, range(24, 128, 11))),
             ("meta byte order: built with the bytes reversed", pat, m3 + bch.chunk(pat, m3[::-1])[3:])]
    for label, data, ch in cases:
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + \
            _dma_in(0x1, a, e["dma"]) + _dma_in(0x10, a + 0x200, e["dma"]) + _kick(0x1101, 0x180, e["ecc"]) + \
            "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + _dma_out(0x100, a + 0x400, e["dma"]) + \
            _dma_out(0x1000, a + 0x600, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END,
                    scratch=data + ch.ljust(64, b"\xff").rjust(512, b"\xff") + b"\xee" * 0x400,  # slot 0 = byte 448
                    scratch_read=0x810, quiet=True)
        sc = rec["scratch"] or bytes(0x810)
        back, slot = sc[0x400:0x600], sc[0x600:0x610]
        out[label] = rec
        lines.append("  %-48s -> %-5s ECC_RESULT %08x ECC_DONE %02x, data %s, slot %s" % (
            label, rec["why"], rec["r"][2], rec["r"][3], "clean" if back == pat else "as sent" if back == data else
            "other", "clean" if slot == good else "as sent" if slot == ch else slot.hex()))
    soft_reset(s, t)
    return out, "decode of host-built chunks in parity slot 0 (slot = after the decode):\n" + "\n".join(lines)


@experiment("eccmode2", needs=("nand_seq", "fullmap"))
def ex_eccmode2(s, t):
    """ECC_MODE map, encode only (no decode afterwards, so the ECC area shows what the encode left): 0x180 plus each
    single bit 9..24, and 0, 0x80, 0x100, 0xe00000, 0x1ffffff; for the modes that differ, the parity of zeros and of
    single-bit sectors (to fit their code like the 0x180 one)"""
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []

    def enc(mode, data):
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
            _kick(0x1102, mode, e["ecc"]) + "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + \
            _dma_out(0x1000, a + 0x200, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                    scratch=data + b"\xee" * 0x200, scratch_read=0x400, quiet=True)
        slot = (rec["scratch"] or bytes(0x400))[0x200:0x240]
        area = b"".join(struct.pack("<I", w) for w in rec["fullmap"][1][0x900 // 4:0x9c0 // 4]) if rec.get("fullmap") \
            else b""
        return rec, slot, area
    base = bytes((i * 37 + 11) & 0xff for i in range(512))
    ref = None
    differ = []
    modes = [0x180] + [0x180 | 1 << b for b in range(9, 25)] + [0x0, 0x80, 0x100, 0xe00000, 0x1ffffff]
    for mode in modes:
        rec, slot, area = enc(mode, base)
        out["mode 0x%x" % mode] = rec
        if ref is None:
            ref = (slot, area)
        same = (slot, area) == ref
        if not same:
            differ.append(mode)
        lines.append("  0x%07x: %s ECC_DONE %02x ECC_RESULT %08x slot0 %s area %s%s" % (
            mode, rec["why"], rec["r"][3], rec["r"][2], slot[:16].hex(), area.rstrip(b"\0").hex(),
            "  (= 0x180)" if same and mode != 0x180 else ""))
    print("\n".join(lines))
    sys.stdout.flush()
    for mode in [m for m in differ if m in (0x0, 0x80, 0xe00000, 0x1ffffff)] + [m for m in differ if m not in
                                                                                (0x0, 0x80, 0xe00000, 0x1ffffff)][:3]:
        for label, data in [("zeros", bytes(512))] + [("bit %d" % b, _flip(bytes(512), [b])) for b in (0, 1, 8, 9, 4095)]:
            rec, slot, area = enc(mode, data)
            out["fit 0x%x/%s" % (mode, label)] = rec
            lines.append("    fit 0x%07x %-8s slot0 %s area %s" % (mode, label, slot[:16].hex(), area.rstrip(b"\0").hex()))
        print("\n".join(lines[-6:]))
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "ECC_MODE map (encode only; bit numbering of the fit sectors: bit 0 = MSB of byte 0):\n" + "\n".join(lines)


@experiment("tmobits2", needs=("start_value",))
def ex_tmobits2(s, t):
    """Timeout: where does the growth with start-word bits 7:4 stop (n = 5, 6, 7 untested; 8 and above = 2**11)?"""
    words = [0xff55, 0xff65, 0xff75, 0x0f55, 0x0f75]
    out, lines = {}, []
    code = assemble(t, t.prologue(c58=0) + "    movi r6, #0x0\n    wait 3\n    ld   r2, [FMSS_C14]\n"
                    "    movi r6, #0x600d600d\n    end\n" + t.trap())
    for w in words:
        rec = probe(s, t, code, timeout_us=200000, start_value=w, flags=WAIT_END)
        out["0x%04x" % w] = rec
        lines.append("  start 0x%04x -> %-6s fault at %6d us (n = %d; 2**11 per step would be %d us)" % (
            w, rec["why"], rec["fault_at_us"], (w >> 4) & 15, round(((w >> 8) + 1) * 2048 / 108.0)))
    return out, "timeout length, bits 7:4 = 5..7:\n" + "\n".join(lines)


@experiment("ev5ready", needs=("nand_seq",))
def ex_ev5ready(s, t):
    """DESTRUCTIVE (erase of a scratch block). Event 5 (status latch done): an erase program's second status latch
    timed out every time, the first (taken while the erase ran) waited for the end of the erase. Is event 5 the chip
    becoming ready rather than "status byte latched"? Latches idle (after a soft reset / not), twice in one program,
    after a chip reset (FFh), after an erase; FMC_STATUS and NSTAT read before each wait, and Apple's order (NSTAT
    first, wait only while busy)"""
    a, e = t.scratch_addr(s), t.events
    B, rpb = t.scratch_block, t.rows_per_block
    out, lines = {}, []

    def latch(i):
        # r2 = FMC_STATUS right after the latch, r3 = NSTAT then; the wait; r4 = FMC_STATUS after; time stamps via r0
        return ("    wr   [FMCTRL1], #0x20\n" + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    ld   r2, [FMC_STATUS]\n"
                "    ld   r3, [FMC_NSTAT], 0xff\n    st   [0x%x], r2\n    st   [0x%x], r3\n    wait %d\n"
                "    ld   r4, [FMC_STATUS]\n    st   [0x%x], r4\n    wr   [FMC_STATUS], #0x800000\n" % (
                    0xd40 + 12 * i, 0xd44 + 12 * i, e["status"], 0xd48 + 12 * i))
    erase = t.cmd(0x60) + "    wr   [FMC_ANUM], #0x2\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMCTRL1], #0x1\n" \
        "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % ((B + 7) * rpb, e["addr"]) + t.cmd(0xd0)
    cases = [("idle, after a soft reset: 1 latch", True, latch(0)),
             ("idle, no soft reset: 1 latch", False, latch(0)),
             ("idle, no soft reset: 1 latch (again)", False, latch(0)),
             ("idle: 3 latches", True, latch(0) + latch(1) + latch(2)),
             ("FFh reset, then 2 latches", True, t.cmd(0xff) + latch(0) + latch(1)),
             ("erase, then 3 latches", True, erase + latch(0) + latch(1) + latch(2)),
             ("erase, NSTAT polled without waits, then 1 latch", True, erase +
              "    wr   [FMCTRL1], #0x20\n" + t.cmd(0x70) + "pl:\n    wr   [FMCTRL1], #0xca\n"
              "    ld   r1, [FMC_NSTAT], 0x40\n    bz   r1, pl\n    wr   [FMCTRL1], #0x20\n" + latch(0)),
             ("read page (00h .. 30h), then 2 latches", True, t.cmd(0x00) + _address(t, (B + 7) * rpb) +
              "    wr   [FMC_CMD], #0x30\n    wait %d\n    wr   [FMC_STATUS], #0x2\n" % e["cmd"] + latch(0) + latch(1))]
    for label, reset, body in cases:
        if reset:
            soft_reset(s, t)
        src = t.nand_prologue(0) + body + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)
        out[label] = rec
        n = body.count("wr   [FMCTRL1], #0xca\n    ld   r2")
        parts = ["latch %d: STATUS %08x NSTAT %02x -> after the wait %08x" % (
            i, rec["after"][0xd40 + 12 * i], rec["after"][0xd44 + 12 * i], rec["after"][0xd48 + 12 * i]) for i in range(n)]
        lines.append("  %-48s %s, %5d us, %d timeout(s)\n      %s" % (
            label, rec["why"], rec["elapsed_us"], rec["elapsed_us"] // 4500, "\n      ".join(parts)))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "status latch event (5) against chip activity (timeouts ~4.87 ms each):\n" + "\n".join(lines)


@experiment("pageorder", needs=("nand_seq",))
def ex_pageorder(s, t):
    """DESTRUCTIVE (scratch blocks). In writeseq, pages 11 and 13 (programmed after 10 and 12, block otherwise empty)
    read back erased although each program took its busy time. MLC page-order rule? (a) pages 0..7 in order in one run
    (the original firmware's overlapped transfers); (b) page 5 alone in an erased block; (c) page 4 alone; (d) pages
    4, 5; (e) pages 1, 3 (odd only); (f) an erase sent with 4 address cycles (FMC_ANUM 3) of a block holding a page:
    is it erased?"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    B, rpb = t.scratch_block, t.rows_per_block
    out, lines = {}, []
    blocks = {"a": B + 8, "b": B + 9, "c": B + 10, "d": B + 11, "e": B + 12, "f": B + 13}
    for blk in blocks.values():
        src = t.nand_prologue(0) + _erase_src(t, [blk * rpb], "e") + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        out["erase %d" % blk] = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)

    def write(blk, pages, salt):
        data, meta = _pages(len(pages), salt)
        src = t.nand_prologue(0)
        for i, p in enumerate(pages):
            src += "    wr   [FMC_CTRL0], #@CE0_DMA@\n" + _dma_in(0x1, a + 0x800 * i, e["dma"]) + \
                "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x80) + _address(t, blk * rpb + p) + _meta_encode(meta[i], "%d" % i) + \
                "    wr   [FMC_CTRL0], #@CE0_DMA@\n"
            for k in range(4):
                cur = k & 1
                src += _kick(0x1002 | (0x100 << cur) | k << 16, 0x180, e["ecc"]) + \
                    _send(0x1000, 0xf, 0x34, "    wr   [FMC_28], #0x%x\n" % (16 * k))
                if k < 3:
                    src += "    wr   [FMC_DMA_ADDR], #0x%x\n" % (a + 0x800 * i + 0x200 * (k + 1)) + \
                        _send((0x100 << cur) | (1 << (1 - cur)), 0x1ff, 0x2e4) + \
                        "    wait %d\n    wr   [FMC_STATUS], #0x100000\n" % e["dma"]
                else:
                    src += _send(0x100 << cur, 0x1ff, 0xe4)
            src += "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x10) + _ready(t, "w%d" % i) + "    st   [0x%x], r7\n" % (0xd40 + 4 * i)
        src += "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=100000, flags=WAIT_END, scratch=b"".join(data))
        res = []
        for i, p in enumerate(pages):
            r2, secs = _raw_read(t, s, blk * rpb + p, 0)
            ok = all(d == data[4 * i + k] and ch == bch.chunk(data[4 * i + k], meta[i][3 * k:3 * k + 3])
                     for k, (d, ch, dec, r_, st) in enumerate(secs))
            erased = all(d == b"\xff" * 512 and ch == b"\xff" * 16 for d, ch, dec, r_, st in secs)
            res.append("page %d %s (NSTAT %02x)" % (p, "ok" if ok else "ERASED" if erased else "DIFFERENT",
                                                   rec["after"][0xd40 + 4 * i]))
        return rec, res
    for key, pages in (("a", list(range(8))), ("b", [5]), ("c", [4]), ("d", [4, 5]), ("e", [1, 3])):
        rec, res = write(blocks[key], pages, 5 + len(key))
        out["(%s)" % key] = rec
        lines.append("  (%s) block %d pages %s: %s, %d us; %s" % (key, blocks[key], pages, rec["why"], rec["elapsed_us"],
                                                                ", ".join(res)))
        print(lines[-1])
        sys.stdout.flush()
    blk = blocks["f"]
    write(blk, [0], 9)
    src = t.nand_prologue(0) + t.cmd(0x60) + "    wr   [FMC_ANUM], #0x3\n    wr   [FMC_ADDR0], #0x%x\n" \
        "    wr   [FMCTRL1], #0x1\n    wait %d\n    wr   [FMC_STATUS], #0x4\n" % (blk * rpb, e["addr"]) + t.cmd(0xd0) + \
        _ready(t, "z") + "    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)
    out["(f) erase ANUM 3"] = rec
    r2, secs = _raw_read(t, s, blk * rpb, 0)
    lines.append("  (f) erase with 4 address cycles: %s, %d us; page 0 afterwards: %s" % (
        rec["why"], rec["elapsed_us"], " ".join(_verdict(bch, d, ch) for d, ch, dec, r_, st in secs)))
    soft_reset(s, t)
    return out, "page program order (scratch blocks), read back raw:\n" + "\n".join(lines)


@experiment("op09w33", needs=("nand_seq",))
def ex_op09w33(s, t):
    """wait2 33 passed idle and during an erase (5 is the other true one). 33 in the other chip states, and the numbers
    that share bits with 5 / 33 (1, 4, 32, 37, 65, 69, 97, 133, 161, 165) idle, to see a pattern"""
    e = t.events
    out, lines = {}, []
    after = t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    ld   r3, [FMC_NSTAT], 0xff\n    wr   [FMCTRL1], #0x20\n"
    for n in (33, 5, 0, 1, 4, 32, 37, 65, 69, 97, 133, 161, 165):
        src = t.nand_prologue(0) + "    movi r6, #0x0\n    .insn 0x09, %d, 0, 0\n    movi r6, #0x600d600d\n" % n + \
            after + "    wr   [FMC_CTRL0], #@CE_NONE@\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
        out["idle/%d" % n] = rec
        lines.append("  wait2 %3d (0b%08d): %s, fault at %d us" % (n, int(bin(n)[2:]), "passes" if not rec["fault_at_us"]
                                                                    else "times out", rec["fault_at_us"]))
    return out, "wait2, idle chip (no other waits in the program):\n" + "\n".join(lines)


@experiment("ev5arm", needs=("nand_seq",))
def ex_ev5arm(s, t):
    """Event 5 is set by the first status latch of a program only. What re-arms it inside a program? latch, X, latch
    for X = nothing, the set-up word, the status-clear word, CE deselect / reselect, FMCTRL1 0x100e0, a 16-byte read
    from the chip (00h + 0x32 transfer), an ECC kick, a DMA; and a latch with FMC_DATA_SEL 0x100 (what does it set?)"""
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []

    def latch(i, sel=0x1):
        return ("    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x%x\n" % sel + t.cmd(0x70) +
                "    wr   [FMCTRL1], #0xca\n    ld   r2, [FMC_STATUS]\n    ld   r3, [FMC_NSTAT], 0xff\n"
                "    st   [0x%x], r2\n    st   [0x%x], r3\n    wait %d\n    wr   [FMC_STATUS], #0x800000\n" % (
                    0xd40 + 8 * i, 0xd44 + 8 * i, e["status"]))
    xs = [("nothing", ""),
          ("set-up word", "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n"),
          ("status-clear word", "    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"),
          ("set-up + status-clear", "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"),
          ("CE deselect / reselect", "    wr   [FMC_CTRL0], #@CE_NONE@\n    wr   [FMC_CTRL0], #@CE0@\n"),
          ("FMCTRL1 0x100e0", "    wr   [FMCTRL1], #0x20\n    wr   [FMCTRL1], #0x100e0\n"),
          ("16-byte read from the chip", "    wr   [FMCTRL1], #0x20\n" + t.cmd(0x00) + _send(0x10, 0xf, 0x32,
                                                                                         "    wr   [FMC_24], #0x0\n")),
          ("ECC kick", _kick(0x1102, 0x180, e["ecc"])),
          ("DMA", "    wr   [FMC_CTRL0], #@CE0_DMA@\n" + _dma_in(0x1, a, e["dma"]) + "    wr   [FMC_CTRL0], #@CE0@\n")]
    for label, x in xs:
        src = t.nand_prologue(0) + latch(0) + x + latch(1) + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(512))
        out[label] = rec
        lines.append("  latch, %-28s latch: second wait %s (%5d us); STATUS after latch 1 %08x, NSTAT %02x" % (
            label + ",", "PASSED" if rec["elapsed_us"] < 3000 else "timed out", rec["elapsed_us"],
            rec["after"][0xd48], rec["after"][0xd4c]))
    # a latch with the drain select: what does it return / set?
    src = t.nand_prologue(0) + latch(0, 0x100) + latch(1) + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
        "    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
    out["DATA_SEL 0x100 first"] = rec
    lines.append("  latch with DATA_SEL 0x100, then a normal latch: %d us; latch 0 STATUS %08x NSTAT %02x, latch 1 STATUS "
                 "%08x NSTAT %02x" % (rec["elapsed_us"], rec["after"][0xd40], rec["after"][0xd44], rec["after"][0xd48],
                                      rec["after"][0xd4c]))
    soft_reset(s, t)
    return out, "what re-arms event 5 inside a program (each line: latch, X, latch; a timeout ~4.87 ms):\n" + "\n".join(lines)


@experiment("writeseq2", needs=("nand_seq",))
def ex_writeseq2(s, t):
    """DESTRUCTIVE (scratch blocks). Multi-page runs with the status poll done right (FMC_DATA_SEL = 1): eight pages in
    order with overlapped transfers and FMC_DMA_CTRL = 7 before the last DMA; pages 1, 3; a page programmed again
    without an erase (FAIL bit?); the status byte while a program runs"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    B, rpb = t.scratch_block, t.rows_per_block
    out, lines = {}, []
    for blk in (B + 14, B + 15):
        src = t.nand_prologue(0) + _erase_src(t, [blk * rpb], "e") + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        out["erase %d" % blk] = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)

    def write(blk, pages, salt, busy_peek=False):
        data, meta = _pages(len(pages), salt)
        src = t.nand_prologue(0)
        for i, p in enumerate(pages):
            src += "    wr   [FMC_CTRL0], #@CE0_DMA@\n" + _dma_in(0x1, a + 0x800 * i, e["dma"]) + \
                "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x80) + _address(t, blk * rpb + p) + \
                _meta_encode(meta[i], "%d" % i) + "    wr   [FMC_CTRL0], #@CE0_DMA@\n"
            for k in range(4):
                cur = k & 1
                src += _kick(0x1002 | (0x100 << cur) | k << 16, 0x180, e["ecc"]) + \
                    _send(0x1000, 0xf, 0x34, "    wr   [FMC_28], #0x%x\n" % (16 * k))
                if k < 3:
                    if i == len(pages) - 1 and k == 2:
                        src += "    wr   [FMC_DMA_CTRL], #0x7\n"
                    src += "    wr   [FMC_DMA_ADDR], #0x%x\n" % (a + 0x800 * i + 0x200 * (k + 1)) + \
                        _send((0x100 << cur) | (1 << (1 - cur)), 0x1ff, 0x2e4) + \
                        "    wait %d\n    wr   [FMC_STATUS], #0x100000\n" % e["dma"]
                else:
                    src += _send(0x100 << cur, 0x1ff, 0xe4)
            src += "    wr   [FMC_CTRL0], #@CE0@\n" + t.cmd(0x10)
            if busy_peek:                                  # one latch right after 10h, no wait: the busy status byte
                src += "    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n" \
                    "    ld   r5, [FMC_NSTAT], 0xff\n    st   [0x%x], r5\n    wr   [FMCTRL1], #0x20\n" % (0xd60 + 4 * i)
            # re-latch without waits (as the original programs do) until ready
            src += ("    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) + "pw%d:\n    wr   [FMCTRL1], #0xca\n"
                    "    ld   r1, [FMC_NSTAT], 0x40\n    bz   r1, pw%d\n    ld   r7, [FMC_NSTAT], 0xff\n"
                    "    wr   [FMCTRL1], #0x20\n    st   [0x%x], r7\n" % (i, i, 0xd40 + 4 * i))
        src += "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=100000, flags=WAIT_END, scratch=b"".join(data))
        res = []
        for i, p in enumerate(pages):
            r2, secs = _raw_read(t, s, blk * rpb + p, 0)
            ok = all(d == data[4 * i + k] and ch == bch.chunk(data[4 * i + k], meta[i][3 * k:3 * k + 3])
                     for k, (d, ch, dec, r_, st) in enumerate(secs))
            erased = all(d == b"\xff" * 512 and ch == b"\xff" * 16 for d, ch, dec, r_, st in secs)
            res.append("page %d %s (ready %02x%s)" % (p, "ok" if ok else "ERASED" if erased else "DIFFERENT",
                                                     rec["after"][0xd40 + 4 * i], ", just after 10h %02x" %
                                                     rec["after"][0xd60 + 4 * i] if busy_peek else ""))
        return rec, res
    for label, blk, pages, peek in (("8 pages in order", B + 14, list(range(8)), True),
                                    ("pages 9, 11", B + 14, [9, 11], False),
                                    ("page 0 again without an erase", B + 14, [0], True),
                                    ("pages 1, 3 of an empty block", B + 15, [1, 3], False)):
        rec, res = write(blk, pages, 3 + len(label), peek)
        out[label] = rec
        lines.append("  %-32s %s, %5d us: %s" % (label, rec["why"], rec["elapsed_us"], ", ".join(res)))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "multi-page program runs with the status poll on FMC_DATA_SEL = 1 (ready byte; b0 = FAIL):\n" + \
        "\n".join(lines)


@experiment("eccmode3", needs=("nand_seq", "fullmap"))
def ex_eccmode3(s, t):
    """ECC_MODE: (1) the length field b20:16 with 3, 5, 12, 31 and with b21..b24 added (is the field wider?); (2) a
    decode with the length field (does it check only those bytes?); (3) parity samples for the codes not identified
    yet (b15 and the modes without b8): single-bit sectors at bits 0..15 and 4080..4095"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    base = bytes((i * 37 + 11) & 0xff for i in range(512))

    def enc(mode, data, decode_data=None):
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
            _kick(0x1102, mode, e["ecc"]) + _dma_out(0x1000, a + 0x200, e["dma"])
        if decode_data is not None:
            src += _dma_in(0x1, a + 0x400, e["dma"]) + _kick(0x1101, mode, e["ecc"]) + \
                "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n"
        src += "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                    scratch=data + b"\xee" * 0x200 + (decode_data or b"\xee" * 512), scratch_read=0x400, quiet=True)
        slot = (rec["scratch"] or bytes(0x400))[0x200:0x210]
        area = b"".join(struct.pack("<I", w) for w in rec["fullmap"][1][0x900 // 4:0x918 // 4]) if rec.get("fullmap") \
            else b""
        return rec, slot, area

    def length_of(slot):
        for L in range(0, 513):
            if bch.chunk(base[:L] + b"\xff" * (512 - L), b"\xff" * 3) == slot:
                return L
        return None
    for f in (3, 5, 12, 31):
        rec, slot, area = enc(0x180 | f << 16, base)
        out["len %d" % f] = rec
        lines.append("  b20:16 = %2d: encodes the first %s bytes" % (f, length_of(slot)))
    for extra in (1 << 21, 1 << 22, 1 << 24, 0x1f << 16 | 1 << 21):
        rec, slot, area = enc(0x180 | 1 << 16 | extra if extra != (0x1f << 16 | 1 << 21) else 0x180 | extra, base)
        out["len 1 + 0x%x" % extra] = rec
        lines.append("  b20:16 = 1 plus 0x%07x: encodes the first %s bytes" % (extra, length_of(slot)))
    # decode with the length field: data differing only beyond L
    for f, where in ((16, 100), (16, 5)):
        bad = bytearray(base)
        bad[where] ^= 0xff                                 # 8 bits of one byte
        rec, slot, area = enc(0x180 | f << 16, base, bytes(bad))
        out["decode len %d, byte %d" % (f, where)] = rec
        lines.append("  decode with b20:16 = %d, byte %d changed (8 bits): ECC_RESULT %08x ECC_DONE %02x" % (
            f, where, rec["r"][2], rec["r"][3]))
    print("\n".join(lines))
    sys.stdout.flush()
    for mode in (0x8180, 0x0, 0x80):
        for label, data in [("zeros", bytes(512))] + [("bit %d" % b, _flip(bytes(512), [b])) for b in
                                                     list(range(16)) + list(range(4080, 4096))]:
            rec, slot, area = enc(mode, data)
            out["fit 0x%x/%s" % (mode, label)] = rec
        lines.append("  parity samples for 0x%x: zeros, bits 0..15, 4080..4095 (in the JSON)" % mode)
    soft_reset(s, t)
    return out, "ECC_MODE length field and samples:\n" + "\n".join(lines)


@experiment("ev5two", needs=("nand_seq",))
def ex_ev5two(s, t):
    """DESTRUCTIVE (scratch block on CE0 and CE1). Event 5 is re-armed by FMCTRL1 operations. Does a chip becoming ready
    re-arm it too? Program a page on CE0 and on CE1, poll CE0 (its latch waits for ready), then poll CE1 with no FMCTRL1
    operation between (the original WRITE does this). Also: do address cycles (FMCTRL1 = 1) re-arm it?"""
    a, e = t.scratch_addr(s), t.events
    B, rpb = t.scratch_block, t.rows_per_block
    out, lines = {}, []
    for ce in (0, 1):
        src = t.nand_prologue(ce) + _erase_src(t, [(B + 16) * rpb], "e") + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        out["erase CE%d" % ce] = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END)

    def prog(ce, row):
        cd = "0x%x" % (t.ce_sel[ce] | t.ce_dma)
        return ("    wr   [FMC_CTRL0], #%s\n" % cd + _dma_in(0x1, a, e["dma"]) + "    wr   [FMC_CTRL0], #0x%x\n" % t.ce_sel[ce] +
                t.cmd(0x80) + _address(t, row) + _send(0x100, 0x1ff, 0xe4) + t.cmd(0x10))

    def latch(ce, i):
        return ("    wr   [FMC_CTRL0], #0x%x\n    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x1\n" % t.ce_sel[ce] +
                t.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    ld   r3, [FMC_NSTAT], 0xff\n    st   [0x%x], r3\n"
                "    wait %d\n    wr   [FMC_STATUS], #0x800000\n    ld   r3, [FMC_NSTAT], 0xff\n    st   [0x%x], r3\n" % (
                    0xd40 + 8 * i, e["status"], 0xd44 + 8 * i))
    cases = [("program CE0 + CE1, poll CE0, poll CE1", prog(0, (B + 16) * rpb) + prog(1, (B + 16) * rpb) + latch(0, 0) +
              latch(1, 1)),
             ("program CE0 + CE1, poll CE1, poll CE0", prog(0, (B + 16) * rpb + 1) + prog(1, (B + 16) * rpb + 1) +
              latch(1, 0) + latch(0, 1)),
             ("idle: latch, address cycles, latch", latch(0, 0) + "    wr   [FMCTRL1], #0x20\n" + t.cmd(0x00) +
              _address(t, (B + 16) * rpb) + "    wr   [FMCTRL1], #0x20\n" + t.cmd(0xff) + latch(0, 1))]
    for label, body in cases:
        src = t.nand_prologue(0) + body + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END, scratch=bytes(range(256)) * 2)
        out[label] = rec
        lines.append("  %-40s %s, %5d us (%d timeout(s)); poll 1 NSTAT %02x -> %02x, poll 2 NSTAT %02x -> %02x" % (
            label, rec["why"], rec["elapsed_us"], rec["elapsed_us"] // 4500, rec["after"][0xd40], rec["after"][0xd44],
            rec["after"][0xd48], rec["after"][0xd4c]))
    soft_reset(s, t)
    return out, "event 5 and a second chip (each poll waits on event 5):\n" + "\n".join(lines)


@experiment("b15ecc", needs=("nand_seq", "fullmap"))
def ex_b15ecc(s, t):
    """ECC_MODE b15 (0x8180), the GF(2^10) code with 20 parity bytes: its correction strength (encode and decode in one
    program, k flipped bits spread one per 10-bit symbol, and bursts inside a few symbols); and parity samples across
    the sector for identifying the generator"""
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    base = bytes((i * 37 + 11) & 0xff for i in range(512))

    def run(mode, data, bad):
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
            _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n" + _dma_out(0x1000, a + 0x600, e["dma"])
        if bad is not None:
            src += _dma_in(0x1, a + 0x200, e["dma"]) + _kick(0x1101, mode, e["ecc"]) + \
                "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" + _dma_out(0x100, a + 0x400, e["dma"])
        src += "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                    scratch=data + (bad or bytes(512)) + b"\xee" * 0x420, scratch_read=0x610, quiet=True)
        return rec

    def spread(k):          # one bit in each of k different 10-bit symbols (little-endian symbol packing)
        return [(37 * j) * 10 % 4090 + 3 for j in range(k)]

    def le_flip(data, lebits):
        b = bytearray(data)
        for x in lebits:
            b[x // 8] ^= 1 << (x % 8)
        return bytes(b)
    cases = [("%d bits, one per symbol" % k, spread(k)) for k in (0, 1, 2, 4, 8, 9, 10, 12, 16, 17)] + \
            [("burst: 10 bits in 1 symbol", list(range(1000, 1010))),
             ("burst: 40 bits in 4 symbols", list(range(1000, 1040))),
             ("burst: 80 bits in 8 symbols", list(range(1000, 1080))),
             ("burst: 90 bits in 9 symbols", list(range(1000, 1090)))]
    for label, lebits in cases:
        bad = le_flip(base, lebits)
        rec = run(0x8180, base, bad)
        sc = rec["scratch"] or bytes(0x610)
        back = sc[0x400:0x600]
        out[label] = rec
        lines.append("  %-30s -> %-5s ECC_RESULT %08x ECC_DONE %02x, data %s" % (
            label, rec["why"], rec["r"][2], rec["r"][3], "corrected" if back == base else "as sent" if back == bad
            else "other"))
        print(lines[-1])
        sys.stdout.flush()
    for byte in (0, 1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 384, 448, 496, 504, 508, 510, 511):
        for bit in (0, 7):
            rec = run(0x8180, le_flip(bytes(512), [byte * 8 + bit]), None)
            out["sample byte %d bit %d" % (byte, bit)] = rec
    rec = run(0x8180, le_flip(bytes(512), [7, 4095]), None)
    out["sample two bits"] = rec
    lines.append("  parity samples: one bit at bytes 0..511 (bit 0 and 7), and two bits together (in the JSON)")
    soft_reset(s, t)
    return out, "ECC_MODE b15 code, strength (encode and decode in one program) and samples:\n" + "\n".join(lines)


@experiment("eccmode4", needs=("nand_seq", "fullmap"))
def ex_eccmode4(s, t):
    """ECC_MODE leftovers. Modes without b8: is the result reproducible (same sector three times, after a soft reset or
    not), linear over GF(2) (bits 0, 1 and both), and does an earlier operation change it (after a BCH decode / encode)?
    0x1ffffff: which bit beyond b8 + b15 + length 31 changes the code? b15: the upper 10 area bytes after an encode only
    and after a decode"""
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    base = bytes((i * 37 + 11) & 0xff for i in range(512))

    def run(mode, data, pre="", decode=False, reset=True):
        if reset:
            soft_reset(s, t)
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + pre + _dma_in(0x1, a, e["dma"]) + \
            _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n" + _dma_out(0x1000, a + 0x400, e["dma"])
        if decode:
            src += _kick(0x1101, mode, e["ecc"]) + "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n"
        src += "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP,
                    scratch=data + base + b"\xee" * 0x400, scratch_read=0x420, quiet=True)
        fm = rec["fullmap"][1] if rec.get("fullmap") else [0] * 1024
        area = b"".join(struct.pack("<I", w) for w in fm[0x900 // 4:0x914 // 4])
        return rec, area, fm[0x990 // 4]

    def b(x):
        d = bytearray(512)
        for bit in x:
            d[bit // 8] ^= 0x80 >> (bit % 8)
        return bytes(d)
    pre_dec = _dma_in(0x1, a + 0x200, e["dma"]) + _kick(0x1101, 0x180, e["ecc"])
    pre_enc = _dma_in(0x1, a + 0x200, e["dma"]) + _kick(0x1102, 0x180, e["ecc"])
    cases = [("0x0 ramp, after a soft reset (1)", 0x0, base, "", False, True),
             ("0x0 ramp, after a soft reset (2)", 0x0, base, "", False, True),
             ("0x0 ramp, no reset (3)", 0x0, base, "", False, False),
             ("0x0 ramp, no reset (4)", 0x0, base, "", False, False),
             ("0x0 zeros", 0x0, bytes(512), "", False, True),
             ("0x0 bit 0", 0x0, b([0]), "", False, True),
             ("0x0 bit 1", 0x0, b([1]), "", False, True),
             ("0x0 bits 0 + 1", 0x0, b([0, 1]), "", False, True),
             ("0x0 ramp after a BCH decode", 0x0, base, pre_dec, False, True),
             ("0x0 ramp after a BCH encode", 0x0, base, pre_enc, False, True),
             ("0x80 ramp", 0x80, base, "", False, True),
             ("0x80 ramp, then a decode", 0x80, base, "", True, True),
             ("0xe00000 ramp", 0xe00000, base, "", False, True),
             ("0x8180 ramp, encode only", 0x8180, base, "", False, True),
             ("0x8180 ramp, then a decode", 0x8180, base, "", True, True),
             ("0x8180 zeros", 0x8180, bytes(512), "", False, True)]
    for label, mode, data, pre, dec, reset in cases:
        rec, area, w990 = run(mode, data, pre, dec, reset)
        out[label] = rec
        lines.append("  %-34s ECC_DONE %02x area %s 0x990 %08x" % (label, rec["r"][4], area.hex(), w990))
    print("\n".join(lines))
    sys.stdout.flush()
    from .. import bch
    for bit in [None] + [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 21, 22, 23, 24]:
        mode = 0x1f8100 | (1 << bit if bit is not None else 0)
        for lab, data in (("zeros", bytes(512)), ("ramp", base)):
            rec, area, w990 = run(mode, data)
            out["0x%x/%s" % (mode, lab)] = rec
            lines.append("  mode 0x%07x %-5s area %s" % (mode, lab, area.hex()))
    soft_reset(s, t)
    return out, "ECC_MODE leftovers (area = 0x900..0x913 after the run):\n" + "\n".join(lines)


@experiment("finale", needs=("nand_seq", "fullmap"))
def ex_finale(s, t):
    """The last open points. (1) ECC_MODE b1..b5 with the BCH (0x180) and the Reed-Solomon (0x8100) code, with
    single-byte samples for the bits that change something; (2) the sequence of the modes without b8: 48 values in one
    program; (3) what FMC_28 counts (sti, ldi, DMA, chip transfers, ECC operations, plain instructions); (4) do the
    0x100-block words 0x1cc / 0x1d0 change a NAND transfer's time?; (5) which FMC interrupt sources raise VIC0 line 20;
    (6) wait2 N = 0..63 while an ECC operation and a DMA are still running"""
    from .. import bch
    a, e = t.scratch_addr(s), t.events
    out, lines = {}, []
    ramp = bytes((i * 37 + 11) & 0xff for i in range(512))

    def enc(mode, data):
        src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + \
            _kick(0x1102, mode, e["ecc"]) + "    ld   r4, [ECC_DONE]\n" + _dma_out(0x1000, a + 0x200, e["dma"]) + \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END | FULLMAP, scratch=data + b"\xee" * 0x200,
                    scratch_read=0x240, quiet=True)
        fm = rec["fullmap"][1] if rec.get("fullmap") else [0] * 1024
        return rec, b"".join(struct.pack("<I", w) for w in fm[0x900 // 4:0x914 // 4]), (rec["scratch"] or bytes(0x240))[0x200:0x210]
    # (1)
    lines.append("(1) ECC_MODE b1..b5:")
    for base_mode in (0x180, 0x8100):
        _, ref_z, _ = enc(base_mode, bytes(512))
        _, ref_r, _ = enc(base_mode, ramp)
        for bit in (1, 2, 3, 4, 5):
            mode = base_mode | 1 << bit
            rz, az, sz = enc(mode, bytes(512))
            rr, ar, sr = enc(mode, ramp)
            out["(1) 0x%x zeros" % mode], out["(1) 0x%x ramp" % mode] = rz, rr
            changed = (az != ref_z, ar != ref_r)
            lines.append("  0x%05x: zeros %s, ramp %s; ECC_DONE %02x; slot (ramp) %s" % (
                mode, "CHANGED" if changed[0] else "same", "CHANGED" if changed[1] else "same", rr["r"][4], sr.hex()))
            if any(changed):
                for byte in (0, 1, 2, 3, 255, 509, 510, 511):
                    d = bytearray(512)
                    d[byte] = 1
                    r1, a1, s1 = enc(mode, bytes(d))
                    out["(1) 0x%x byte %d" % (mode, byte)] = r1
    print("\n".join(lines))
    sys.stdout.flush()
    # (2) 48 consecutive values in one program
    loop = ""
    for i in range(48):
        loop += _kick(0x1102, 0x0, e["ecc"]) + "".join(
            "    ld   r1, [0x%x]\n    movi r2, #0x%x\n    sti  [r2], r1\n" % (0x900 + 4 * w, a + 0x400 + 20 * i + 4 * w)
            for w in range(4)) + "    ld   r1, [0x990]\n    movi r2, #0x%x\n    sti  [r2], r1\n" % (a + 0x400 + 20 * i + 16)
    src = t.prologue(c58=0) + t.state_setup() + "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) + loop + \
        "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
    rec = probe(s, t, assemble(t, src), timeout_us=200000, flags=WAIT_END, scratch=ramp + bytes(0x200) + bytes(48 * 20),
                scratch_read=0x400 + 48 * 20, quiet=True)
    out["(2) sequence"] = rec
    sc = rec["scratch"] or bytes(0x400 + 48 * 20)
    seq = [sc[0x400 + 20 * i:0x400 + 20 * i + 20] for i in range(48)]
    lines.append("(2) %s; first values: %s ... (48 in the JSON)" % (rec["why"], " ".join(x[:13].hex() for x in seq[:3])))
    # (3) FMC_28
    lines.append("(3) FMC_28 change over 32 repetitions of:")
    acts = [("nothing (32 x movi)", "    movi r3, #0x0\n"),
            ("sti to RAM", "    movi r2, #0x%x\n    sti  [r2], r3\n" % (a + 0x800)),
            ("ldi from RAM", "    movi r2, #0x%x\n    ldi  r3, [r2]\n" % (a + 0x800)),
            ("wr to the parameter RAM", "    wr   [0xd70], #0x1\n"),
            ("DMA RAM -> buffer", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"])),
            ("ECC encode", _kick(0x1102, 0x180, e["ecc"])),
            ("16-byte chip read (CE0)", "    wr   [FMC_CTRL0], #@CE0@\n" + _send(0x10, 0xf, 0x32, "    wr   [FMC_24], #0x0\n"))]
    for label, body in acts:
        src = t.nand_prologue(0) + t.cmd(0x00) + "    ld   r4, [FMC_28]\n" + body * 32 + "    ld   r5, [FMC_28]\n" \
            "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=100000, flags=WAIT_END, scratch=bytes(0x900), quiet=True)
        out["(3) %s" % label] = rec
        lines.append("  %-28s %3d (before %03x after %03x)" % (label, (rec["r"][5] - rec["r"][4]) & 0x1ff, rec["r"][4] & 0x1ff,
                                                             rec["r"][5] & 0x1ff))
    # (4) 0x1cc / 0x1d0 and a page transfer time
    lines.append("(4) a 2112-byte transfer from the chip with 0x1cc / 0x1d0 changed (r2 = sequencer time stamps):")
    for label, extra in (("default", ""), ("0x1cc = 0", "    wr   [0x1cc], #0x0\n"), ("0x1cc = 0xffffff", "    wr   [0x1cc], #0xffffff\n"),
                         ("0x1d0 = 0", "    wr   [0x1d0], #0x0\n"), ("0x1d0 = 0xffffff", "    wr   [0x1d0], #0xffffff\n")):
        src = t.nand_prologue(0) + extra + t.cmd(0x00) + _address(t, 0x80) + "    wr   [FMC_CMD], #0x30\n    wait %d\n" \
            "    wr   [FMC_STATUS], #0x2\n" % e["cmd"] + _ready(t, "t") + "    wr   [FMCTRL1], #0x100e0\n" + t.cmd(0x00) + \
            "    movi r0, #0x0\n" + "".join(_send(0x1, 0x1ff, 0xe2) + _send(0x10, 0xf, 0x32, "    wr   [FMC_24], #0x0\n")
                                         for _ in range(4)) + \
            "    wr   [0x1cc], #0x8060\n    wr   [0x1d0], #0x3010d0\n    wr   [FMC_CTRL0], #@CE_NONE@\n" \
            "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=60000, flags=WAIT_END, quiet=True)
        out["(4) %s" % label] = rec
        lines.append("  %-18s %s, %5d us" % (label, rec["why"], rec["elapsed_us"]))
    # (5) VIC0 line 20 per routed source
    lines.append("(5) VIC0 line 20 / VIC1 line 22 per FMC interrupt source (0x044 enable bit, action):")
    act = {"command": t.cmd(0x70).replace("    wait %d\n    wr   [FMC_STATUS], #0x2\n" % e["cmd"], "    wait %d\n" % e["cmd"]),
           "address": "    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n    wr   [FMCTRL1], #0x1\n    wait %d\n" % e["addr"],
           "status": t.cmd(0x70) + "    wr   [FMCTRL1], #0x20\n    wr   [FMC_DATA_SEL], #0x1\n" + t.cmd(0x70) +
           "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"],
           "DMA": "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]).replace("    wr   [FMC_STATUS], #0x100000\n", "")}
    for src_name, bits in (("command", (0, 1, 2)), ("address", (0, 1)), ("status", (0, 13)), ("DMA", (11,))):
        for bit in bits:
            soft_reset(s, t)
            src = t.nand_prologue(0) + act[src_name] + "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + \
                t.trap()
            rec = probe(s, t, assemble(t, src), params=[(0x044, 1 << bit)], timeout_us=20000, flags=WAIT_END,
                        scratch=bytes(512), quiet=True)
            out["(5) %s/bit %d" % (src_name, bit)] = rec
            lines.append("  %-8s 0x044 bit %2d: CS_IRQ %02x, VIC0 line 20 %s, VIC1 line 22 %s" % (
                src_name, bit, rec["cs_irq"], "SET" if rec["vic0_seen"] >> 20 & 1 else "clear",
                "SET" if rec["vic1_seen"] >> 22 & 1 else "clear"))
    soft_reset(s, t)
    # (6) wait2 during an ECC operation / a DMA in flight
    for label, start in (("ECC encode in flight", "    wr   [FMC_CTRL0], #@CE_DMA@\n" + _dma_in(0x1, a, e["dma"]) +
                          "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1102\n"),
                         ("DMA in flight", "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n"
                          "    wr   [FMC_DATA_SEL], #0x1\n    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n" % a)):
        ok = []
        for n in range(64):
            # the wait2 is the only wait after the start: a run well under one timeout means it passed
            src = t.prologue(c58=0) + t.state_setup() + start + "    movi r6, #0x0\n    .insn 0x09, %d, 0, 0\n" % n + \
                "    wr   [FMC_CTRL0], #@CE_NONE@\n    movi r6, #0x600d600d\n    end\n" + t.trap()
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=bytes(512), quiet=True)
            out["(6) %s/%d" % (label, n)] = rec
            if rec["elapsed_us"] < 3000:
                ok.append(str(n))
        lines.append("(6) wait2 during %s: passes for %s" % (label, " ".join(ok) or "none"))
    soft_reset(s, t)
    return out, "final open points:\n" + "\n".join(lines)
