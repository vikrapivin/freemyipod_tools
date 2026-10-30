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

"""Who can read and write which word of the register window: the CPU, the sequencer, or both."""
import sys

from ..probe import probe, assemble, soft_reset, FULLMAP, WAIT_END
from .registry import experiment

BATCH = 16
PATTERNS = (0xffffffff, 0x00000000)


def access_skip(t):
    """words not written in the access test: transfer triggers, command bytes, the ID FIFO and their copies, the ECC
    kick, the start word and program address, the fault routing words (they would divert the test program)"""
    base = {0x004, 0x008, 0x038, 0x07c, 0x080}
    trig = set()
    for o in base:
        for b in (0x000, 0x200, 0x400, 0x600):
            trig.add(o + b)
    return trig | {0x80c, 0xa0c, 0xc00, 0xc04, 0xc4c, 0xc58, 0xc5c}


def seq_skip():
    """also left out of the sequencer-write half: the registers the test program works with, the PC"""
    return set(range(0xc18, 0xc38, 4)) | {0xc38}


def classify(cw1, cw0, cs1, cs0, sw1, sw0, sc1, sc0):
    """readbacks of all ones / zero: CPU write -> CPU read, CPU write -> sequencer read, sequencer write -> sequencer
    read, sequencer write -> CPU read (None = not tested)"""
    def mask(a, b):
        return None if a is None else (a & ~b) & 0xffffffff
    return {"cpu->cpu": mask(cw1, cw0), "cpu->seq": mask(cs1, cs0), "seq->seq": mask(sw1, sw0),
            "seq->cpu": mask(sc1, sc0)}


@experiment("access", needs=("pld_pst", "fullmap"))
def ex_access(s, t):
    """Every word 0x000..0xffc: does a CPU write read back to the CPU and to the sequencer, and a sequencer write to the
    sequencer and to the CPU (all ones and zero, bits that follow the write)? Original values are put back after each
    group of 16 words; a soft reset at the end"""
    skip, sskip = access_skip(t), seq_skip()
    a = t.scratch_addr(s)
    first = probe(s, t, assemble(t, "    end\n"), timeout_us=20000, flags=FULLMAP, quiet=True)
    orig = first["fullmap"][0]
    words = [o for o in range(0, 0x1000, 4) if o not in skip]
    res = {}
    end = assemble(t, "    end\n")
    for i in range(0, len(words), BATCH):
        grp = words[i:i + BATCH]
        r = {o: {} for o in grp}
        for pat in PATTERNS:
            # CPU writes; the sequencer reads each word into the scratch area; the CPU reads before the start
            src = "    movi r0, #0x%x\n" % a + "".join(
                "    movi r1, #0x%x\n    pld  r2, [r1]\n    sti  [r0], r2\n    add  r0, r0, #0x4\n" % o for o in grp) + \
                "    end\n"
            rec = probe(s, t, assemble(t, src), params=[(o, pat) for o in grp], timeout_us=20000,
                        flags=FULLMAP | WAIT_END, scratch=bytes(4 * BATCH), scratch_read=4 * BATCH, quiet=True)
            sc = rec["scratch"] or bytes(4 * BATCH)
            for k, o in enumerate(grp):
                r[o]["cw%x" % (pat & 1)] = rec["fullmap"][0][o // 4]
                r[o]["cs%x" % (pat & 1)] = int.from_bytes(sc[4 * k:4 * k + 4], "little")
            probe(s, t, end, params=[(o, orig[o // 4]) for o in grp], timeout_us=20000, quiet=True)
            # the sequencer writes each word and reads it back; the CPU reads after the run
            sg = [o for o in grp if o not in sskip]
            if sg:
                src = "    movi r0, #0x%x\n    movi r3, #0x%x\n" % (a, pat) + "".join(
                    "    movi r1, #0x%x\n    pst  [r1], r3\n    pld  r2, [r1]\n    sti  [r0], r2\n"
                    "    add  r0, r0, #0x4\n" % o for o in sg) + "    end\n"
                rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=FULLMAP | WAIT_END,
                            scratch=bytes(4 * BATCH), scratch_read=4 * BATCH, quiet=True)
                sc = rec["scratch"] or bytes(4 * BATCH)
                for k, o in enumerate(sg):
                    r[o]["sw%x" % (pat & 1)] = int.from_bytes(sc[4 * k:4 * k + 4], "little")
                    r[o]["sc%x" % (pat & 1)] = rec["fullmap"][1][o // 4]
                probe(s, t, end, params=[(o, orig[o // 4]) for o in grp], timeout_us=20000, quiet=True)
        for o in grp:
            v = r[o]
            res[o] = dict(v, orig=orig[o // 4], **classify(v.get("cw1"), v.get("cw0"), v.get("cs1"), v.get("cs0"),
                                                         v.get("sw1"), v.get("sw0"), v.get("sc1"), v.get("sc0")))
        print("  0x%03x..0x%03x done" % (grp[0], grp[-1]))
        sys.stdout.flush()
    soft_reset(s, t)
    # summary: runs of words with the same four masks
    lines, prev, start = [], None, None

    def fmt(m):
        return "-" if m is None else "%08x" % m
    for o in words + [None]:
        key = None if o is None else tuple(res[o][k] for k in ("cpu->cpu", "cpu->seq", "seq->seq", "seq->cpu"))
        if key != prev:
            if prev is not None:
                lines.append("  0x%03x..0x%03x  cpu->cpu %s  cpu->seq %s  seq->seq %s  seq->cpu %s" % (
                    (start, last) + tuple(fmt(m) for m in prev)))
            prev, start = key, o
        last = o
    return {"words": res, "skipped": sorted(skip), "seq_skipped": sorted(sskip)}, \
        ("bits that follow a write of all ones / zero (writer->reader; - = not tested). Not written: %s; not written by "
         "the sequencer: r0..r7, PC\n" % " ".join("0x%03x" % o for o in sorted(skip))) + "\n".join(lines)


@experiment("pcwrite", needs=())
def ex_pcwrite(s, t):
    """The CPU cannot write the PC (0xc38). Can the sequencer, and does that jump (as a write to r8 does)?"""
    tail = ("    movi r6, #0xbad0bad0\n    end\n" "lab:\n    movi r6, #0x11111111\n    end\n")
    out, lines = {}, []
    for label, body in (("wr [0xc38], #lab", "    wr   [0xc38], #lab\n"),
                        ("st [0xc38], r1 (r1 = lab)", "    movi r1, #lab\n    st   [0xc38], r1\n"),
                        ("ld r2, [0xc38] (read it)", "    ld   r2, [0xc38]\n")):
        src = t.prologue(c58=0) + "    movi r6, #0x0\n    movi r2, #0x0\n" + body + tail + t.trap()
        rec = probe(s, t, assemble(t, src), timeout_us=20000, flags=WAIT_END)
        out[label] = rec
        r6 = rec["r"][6]
        lines.append("  %-28s -> %-6s %s; r2 %08x" % (label, rec["why"], {0x11111111: "JUMPED to lab",
                     0xbad0bad0: "fell through", 0: "ended without either"}.get(r6, "r6 %08x" % r6), rec["r"][2]))
    return out, "the sequencer and its PC:\n" + "\n".join(lines)
