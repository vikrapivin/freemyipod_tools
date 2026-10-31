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

"""How many bits of the instruction fields are decoded, and which interrupt sources reach CS_IRQ and the CPU."""
import struct
import sys

from ..probe import probe, assemble, soft_reset, WAIT_END
from .registry import experiment

LAB_MARK = struct.pack("<II", 0x05060000, 0x11111111)          # movi r6, #0x11111111


def lab_offset(t, src):
    """byte offset of the instruction 'movi r6, #0x11111111' in the assembled source"""
    code = assemble(t, src)
    return next(i for i in range(0, len(code), 8) if code[i:i + 8] == LAB_MARK)


@experiment("fwidth", needs=())
def ex_fwidth(s, t):
    """Field widths: do wr / st / ld use the upper bits of their 16-bit offset, and branches, jr and r8 writes the upper
    bits of their 32-bit target? (pld / pst wrap at 12 bits, already known)"""
    out, lines = {}, []
    # register offsets: parameter word 0xd70 with bits 12..15 set in the offset
    for label, body, chk in (
            ("wr [0x1d70]", "    .insn 0x01, 0, 0x1d70, 0x11111111\n", "write"),
            ("wr [0x8d70]", "    .insn 0x01, 0, 0x8d70, 0x22222222\n", "write"),
            ("wr [0xfd70]", "    .insn 0x01, 0, 0xfd70, 0x33333333\n", "write"),
            ("st [0x8d70], r1", "    movi r1, #0x44444444\n    .insn 0x02, 1, 0x8d70, 0\n", "write"),
            ("ld r2, [0x1d74]", "    .insn 0x04, 2, 0x1d74, 0xffffffff\n", "read"),
            ("ld r2, [0x8d74]", "    .insn 0x04, 2, 0x8d74, 0xffffffff\n", "read")):
        src = t.prologue(c58=0) + "    movi r2, #0x0\n" + body + "    movi r6, #0x600d600d\n    end\n" + t.trap()
        rec = probe(s, t, assemble(t, src), params=[(0xd70, 0), (0xd74, 0x5a5a5a5a)], timeout_us=20000,
                    flags=WAIT_END)
        out[label] = rec
        got = ("0xd70 now %08x" % rec["after"][0xd70] if chk == "write" else "r2 %08x (0xd74 holds 5a5a5a5a)" % rec["r"][2])
        lines.append("  %-20s -> %-6s %s" % (label, rec["why"], got))
    # targets with upper bits set
    tail = "    movi r6, #0xbad0bad0\n    end\n    .insn 0x08, 0, 0, 0\nlab:\n    movi r6, #0x11111111\n    end\n"
    for hi in (0x1000, 0x10000, 0x1000000, 0x80000000):
        for label, body in (("bnz", "    movi r1, #0x1\n    .insn 0x0e, 1, 0, @T@\n"),
                            ("jr", "    movi r1, #@T@\n    .insn 0x0f, 0, 1, 0\n"),
                            ("movi r8", "    .insn 0x05, 8, 0, @T@\n")):
            src = t.prologue(c58=0) + "    movi r6, #0x0\n" + body + tail + t.trap()
            at = lab_offset(t, src.replace("@T@", "0x0"))
            src = src.replace("@T@", "0x%x" % (at | hi))
            rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
            out["%s/0x%x" % (label, hi)] = rec
            r6 = rec["r"][6]
            lines.append("  %-8s target lab | 0x%-8x -> %-7s %s%s" % (
                label, hi, rec["why"], {0x11111111: "landed on lab (upper bits ignored)", 0xbad0bad0: "fell through",
                                        0: "went elsewhere (no marker)"}.get(r6, "r6 %08x" % r6),
                " (timeout fault)" if rec["fault_at_us"] else ""))
    return out, "field widths:\n" + "\n".join(lines)


@experiment("irqsrc", needs=("nand_seq",))
def ex_irqsrc(s, t):
    """Interrupt sources: each first-set flag pending on its own (command, address, transfer, DMA, ECC, status), with no
    enables, all FMC enables (0x044 = 0xffff) or all ECC enables (0x844 = 0x2f): CS_IRQ, C50 and every VIC0 / VIC1 raw
    line; then which single 0x044 bit routes each flag that reached CS_IRQ"""
    a = t.scratch_addr(s)
    sel = "    wr   [FMC_CTRL0], #@CE0@\n"
    cmd = sel + "    wr   [FMC_CMD], #0x70\n    wait 0\n"
    adr = sel + "    wr   [FMC_CMD], #0x90\n    wait 0\n    wr   [FMC_STATUS], #0x2\n    wr   [FMC_ANUM], #0x0\n" \
        "    wr   [FMC_ADDR0], #0x0\n    wr   [FMCTRL1], #0x1\n    wait 1\n"
    xfer = adr + "    wr   [FMC_STATUS], #0x4\n    wr   [FMC_DNUM], #0x7\n    wr   [FMC_DATA_SEL], #0x1\n" \
        "    wr   [FMCTRL1], #0x1c2\n    wait 3\n"
    dma = "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n" \
        "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 2\n" % a
    ecc = "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1101\n    wait 4\n"
    stat = cmd + "    wr   [FMC_STATUS], #0x2\n    wr   [FMCTRL1], #0xca\n    wait 5\n"
    states = [("nothing", ""), ("command", cmd), ("address", adr), ("transfer", xfer), ("DMA", dma), ("ECC", ecc),
              ("status", stat)]
    head = "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
    tail = ("    ld   r2, [CS_IRQ]\n    ld   r3, [FMSS_C50]\n    ld   r4, [FMC_STATUS]\n    ld   r5, [0x050]\n"
            "    movi r6, #0x600d600d\n    end\n")

    def run(body, params):
        soft_reset(s, t)
        src = t.prologue(c58=0) + head + body + tail + t.trap()
        return probe(s, t, assemble(t, src), params=params, timeout_us=20000, flags=WAIT_END, scratch=bytes(1024))
    out, lines = {}, []
    base = run("", [])
    out["baseline"] = base
    v0, v1 = base["vic0_seen"], base["vic1_seen"]
    lines.append("  baseline (nothing pending, no enables): VIC0 seen %08x, VIC1 seen %08x" % (v0, v1))
    routed = []
    for label, body in states:
        for en, params in (("none", []), ("0x044=ffff", [(0x044, 0xffff)]), ("0x844=2f", [(0x844, 0x2f)])):
            rec = run(body, params)
            out["%s/%s" % (label, en)] = rec
            if en == "0x044=ffff" and rec["r"][2] & 0x70:
                routed.append((label, body))
            lines.append("  %-9s %-11s CS_IRQ %02x C50 %08x FMC_STATUS %08x 0x050 %08x  new VIC0 %08x VIC1 %08x%s" % (
                label, en, rec["r"][2] & 0xff, rec["r"][3], rec["r"][4], rec["r"][5], rec["vic0_seen"] & ~v0,
                rec["vic1_seen"] & ~v1, "" if rec["why"] == "END" else " (" + rec["why"] + ")"))
        print(lines[-1])
        sys.stdout.flush()
    for label, body in routed:
        bits = []
        for b in range(16):
            rec = run(body, [(0x044, 1 << b)])
            out["%s/0x044 bit %d" % (label, b)] = rec
            if rec["r"][2] & 0x70:
                bits.append("b%d->CS_IRQ %02x" % (b, rec["r"][2] & 0x70))
        lines.append("  %-9s single 0x044 bits that reach CS_IRQ: %s" % (label, ", ".join(bits) or "none"))
    soft_reset(s, t)
    return out, "interrupt sources (VIC bits: raw lines seen during the run that were not set in the baseline):\n" + \
        "\n".join(lines)


@experiment("eccpar", needs=("nand_seq", "fullmap"))
def ex_eccpar(s, t):
    """Where do the ECC parity bytes and the decoder's results become visible? Without the flash: a known 512-byte sector
    is DMA-ed into data buffer 0, encoded into parity buffer 0 slot 0, then decoded as is and with one / two bits
    flipped; the whole ECC area (0x800..0x9fc) is dumped after each step"""
    from ..probe import FULLMAP
    a = t.scratch_addr(s)
    pats = [("zeros", bytes(512)), ("ones", b"\xff" * 512), ("ramp", bytes(i & 0xff for i in range(512)))]

    def load_and(kick, data, parity_out=False):
        """DMA data into data buffer 0, then one ECC operation; with parity_out, then also try to DMA parity buffer 0
        (16 bytes, the slot the encode filled) to scratch + 0x400; returns the record (dumps before / after)"""
        src = t.prologue(c58=0) + ("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
            "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
            "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 2\n    wr   [FMC_STATUS], #0x100000\n"
            "    wr   [FMC_CTRL0], #@CE_NONE@\n" % a)
        if kick:
            src += ("    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x%x\n    wait 4\n"
                    "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n" % kick)
        if parity_out:
            src += ("    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x1000\n"
                    "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n    wait 2\n    ld   r4, [CS_IRQ], 0x4\n"
                    "    wr   [FMC_STATUS], #0x100000\n    wr   [FMC_CTRL0], #@CE_NONE@\n" % (a + 0x400))
        src += "    movi r6, #0x600d600d\n    end\n" + t.trap()
        scr = data + b"\xee" * 512 if parity_out else data
        return probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END | FULLMAP, scratch=scr, quiet=True,
                     scratch_read=0x410 if parity_out else 256)
    out, lines = {}, []
    for name, data in pats:
        base = load_and(0, data)                                   # data loaded, no ECC operation
        enc = load_and(0x1102, data, parity_out=True)              # encode: data buffer 0 -> parity buffer 0 slot 0
        out["%s/load" % name], out["%s/encode" % name] = base, enc
        b0, e1 = base["fullmap"][1], enc["fullmap"][1]
        ch = ["%03x:%08x" % (4 * i, e1[i]) for i in range(0x800 // 4, 0xa00 // 4) if e1[i] != b0[i]]
        par = (enc["scratch"] or b"")[0x400:0x410]
        lines.append("  %-6s encode: %s; ECC area words that changed: %s; parity buffer DMA %s: %s" % (
            name, enc["why"], " ".join(ch) or "none", "timed out" if enc["r"][4] else "done",
            "untouched" if par == b"\xee" * 16 else par.hex()))
        for flips, label in (((), "decode, no error"), ((5,), "decode, 1 bit flipped"), ((5, 300 * 8 + 3), "decode, 2 bits flipped")):
            bad = bytearray(data)
            for bit in flips:
                bad[bit // 8] ^= 1 << (bit % 8)
            dec = load_and(0x1101, bytes(bad))                     # parity slot 0 still holds the encode's parity
            out["%s/%s" % (name, label)] = dec
            d1 = dec["fullmap"][1]
            ch = ["%03x:%08x" % (4 * i, d1[i]) for i in range(0x800 // 4, 0xa00 // 4) if d1[i] != e1[i]]
            lines.append("  %-6s %-24s ECC_RESULT %08x ECC_DONE %02x; changed vs after the encode: %s" % (
                name, label, dec["r"][2], dec["r"][3], " ".join(ch) or "none"))
        print("\n".join(lines[-4:]))
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "ECC engine with a known sector (no flash access):\n" + "\n".join(lines)


@experiment("bufsize", needs=("nand_seq",))
def ex_bufsize(s, t):
    """How big is a data buffer? N bytes of a known pattern DMA-ed into data buffer 0 and N bytes back out (FMC_DNUM =
    N - 1), N = 256, 512, 513, 1024, 2048: does a longer transfer wrap, stop or time out at 512?"""
    a = t.scratch_addr(s)
    src_off, dst_off = 0x0, 0x2000
    out, lines = {}, []
    for n in (256, 512, 513, 1024, 2048):
        pat = bytes((i * 7 + (i >> 8) * 3 + 1) & 0xff for i in range(n))
        scr = pat + bytes(dst_off - n) + b"\xee" * 2048
        src = t.prologue(c58=0) + ("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
            "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x%x\n    wr   [FMC_DATA_SEL], #0x1\n"
            "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 2\n    ld   r1, [CS_IRQ], 0x4\n"
            "    wr   [CS_IRQ], #0x4\n    wr   [FMC_STATUS], #0x100000\n    wr   [FMC_DATA_SEL], #0x100\n"
            "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n    wait 2\n    ld   r2, [CS_IRQ], 0x4\n"
            "    ld   r3, [FMC_DMA_ADDR]\n    wr   [FMC_STATUS], #0x100000\n    wr   [FMC_CTRL0], #@CE_NONE@\n"
            "    movi r6, #0x600d600d\n    end\n" % (n - 1, a + src_off, a + dst_off)) + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=40000, flags=WAIT_END, scratch=scr, scratch_read=dst_off + 2048,
                    quiet=True)
        got = (rec["scratch"] or b"")[dst_off:dst_off + 2048]
        rec["dst"] = got.hex()
        out[str(n)] = rec
        same = next((i for i in range(n) if got[i] != pat[i]), n)
        wrap = n > 512 and got[512:n] == pat[:n - 512]
        lines.append("  N = %-5d -> fill %s, drain %s; first %d bytes intact%s; bytes written past N: %s; DMA_ADDR after "
                     "%08x (start %08x)" % (n, "T" if rec["r"][1] else "ok", "T" if rec["r"][2] else "ok", same,
                                            ", bytes 512.. repeat the start (wrap)" if wrap else "",
                                            "none" if got[n:] == b"\xee" * (2048 - n) else "yes", rec["r"][3],
                                            a + dst_off))
    return out, "data buffer size (RAM -> buffer 0 -> RAM, N bytes):\n" + "\n".join(lines)


@experiment("eccstr", needs=("nand_seq",))
def ex_eccstr(s, t):
    """ECC strength, measured: a known sector encoded, then decoded with 1..12 bits flipped (spread over the sector) -
    from how many flipped bits is it uncorrectable? And does ECC_MODE (always 0x180 in the original programs) change
    that: modes tried at 8, 9 and 12 flipped bits"""
    a = t.scratch_addr(s)
    data = bytes((i * 37 + 11) & 0xff for i in range(512))

    def run(kick, mode, buf):
        src = t.prologue(c58=0) + ("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
            "    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
            "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait 2\n    wr   [FMC_STATUS], #0x100000\n"
            "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x%x\n    wr   [ECC_KICK], #0x%x\n    wait 4\n"
            "    ld   r2, [ECC_RESULT]\n    ld   r3, [ECC_DONE]\n    ld   r4, [CS_IRQ], 0x4\n"
            "    wr   [FMC_DATA_SEL], #0x100\n    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n    wait 2\n"
            "    wr   [FMC_STATUS], #0x100000\n    wr   [FMC_CTRL0], #@CE_NONE@\n"
            "    movi r6, #0x600d600d\n    end\n" % (a, mode, kick, a + 0x1000)) + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END, scratch=buf + bytes(0x1000 - 512) +
                    b"\xee" * 512, scratch_read=0x1200, quiet=True)
        rec["back"] = (rec["scratch"] or b"")[0x1000:0x1200]
        return rec

    def flipped(k):
        bad = bytearray(data)
        for j in range(k):
            bit = (j * 409 + 13) % 4096                    # spread over the sector, distinct bits
            bad[bit // 8] ^= 1 << (bit % 8)
        return bytes(bad)
    out, lines = {}, []
    for mode in (0x180,):
        enc = run(0x1102, mode, data)
        out["encode/%x" % mode] = enc
        for k in range(0, 13):
            rec = run(0x1101, mode, flipped(k))
            out["%x/%d" % (mode, k)] = rec
            lines.append("  ECC_MODE %06x, %2d bits flipped -> ECC_RESULT %08x (b0 %d, corrected %2d), data %s" % (
                mode, k, rec["r"][2], rec["r"][2] & 1, rec["r"][2] >> 16 & 0xf,
                "restored" if rec["back"] == data else "as sent" if rec["back"] == flipped(k) else "changed otherwise"))
    for mode in (0xe00000, 0x0, 0x80, 0x100, 0x1c0, 0x200, 0x380):
        run(0x1102, mode, data)                            # encode in this mode first
        for k in (1, 8, 9, 12):
            rec = run(0x1101, mode, flipped(k))
            out["%x/%d" % (mode, k)] = rec
            lines.append("  ECC_MODE %06x, %2d bits flipped -> ECC_RESULT %08x (b0 %d, corrected %2d), data %s" % (
                mode, k, rec["r"][2], rec["r"][2] & 1, rec["r"][2] >> 16 & 0xf,
                "restored" if rec["back"] == data else "as sent" if rec["back"] == flipped(k) else "changed otherwise"))
        print(lines[-1])
        sys.stdout.flush()
    soft_reset(s, t)
    return out, "ECC decode of one known sector with bits flipped (encode in the same mode first):\n" + "\n".join(lines)
