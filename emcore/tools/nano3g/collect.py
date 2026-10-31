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

"""Collect what Rockbox needs to validate the unit's NAND, through emCORE (read only).

Writes the same archive as Rockbox's NAND check (utils/ipodnano3g/nandcheck/nandcheck.py collect), so its host
tests take it as is (test_ftl / test_crash collected:DIR):

  report.txt   chip ids (all eight id bytes of every chip enable), chip table row and geometry, the read-only
               mount result, verdict, model, software and hardware version (no serial number)
  meta.bin     every page's 12 spare bytes and read result (0 erased, 1 written, 3 failed, -1 timed out), bank
               by bank
  results.bin  every page's full read result word (ECC class | most bits corrected << 8 | 0x10000 erased)
  pages.bin    every page that is not user data or erased (FTL and VFL control structures, bad-block records),
               each as {u32 bank, u32 page, page data}
  summary.txt  page counts by type, ECC failures, timeouts, erased-flag agreement, corrected-bit histogram

Device side: monitor command 137 (META_SCAN, target/ipodnano3g/nand3g.c) returns the records of a range of
blocks, built as Rockbox's check builds them; control pages come through the page read (123).

  n3g.py collect [--out DIR] [--backend sw|fmss] [--bulk]
"""
import argparse
import hashlib
import io
import os
import struct
import sys
import tarfile
import time

from . import nand
from .nand import BACKENDS, Cmd, ToolError

MAGIC = "nano3g-nandcheck 1"
RECORD = 16
DATA_TYPES = (0x40, 0x41)
ERASED = 0xff
SYSCFG_MAGIC = 0x53436667          # "SCfg"
TAG_MODN, TAG_SWVR, TAG_HWVR = 0x4d6f6423, 0x53775672, 0x48775672
ERR_UNSUPPORTED, ERR_MIXED, ERR_FTL = -5, -6, -100


def s32(v):
    return v - (1 << 32) if v & 0x80000000 else v


def verdict(nand_rc, have_row, ftl_rc, validated):
    """Rockbox's nand_check_init() wording"""
    if nand_rc == ERR_UNSUPPORTED:
        return "chip not in the chip table"
    if nand_rc == ERR_MIXED:
        return "different chips on the chip enables"
    if not have_row:
        return "NAND did not answer"
    if ftl_rc == 0:
        return "OK, validated chip" if validated else "OK, needs validation"
    return {-2: "geometry beyond FTL", -3: "no VFL context", -4: "layout not supported",
            -5: "VFL does not fit row"}.get(ftl_rc, "mount failed")


def syscfg_lines(nor):
    """model / swvr / hwvr from the SysCfg block (bootflash offset 0), in SysCfg order; never the serial"""
    w = struct.unpack_from("<6I", nor, 0)
    if w[0] != SYSCFG_MAGIC or w[1] > len(nor) or w[5] * 0x14 + 0x18 != w[1]:
        return []
    out = []
    for i in range(w[5]):
        off = 0x18 + i * 0x14
        tag = struct.unpack_from("<I", nor, off)[0]
        data = nor[off + 4:off + 0x14]
        if tag == TAG_MODN:
            out.append("model %s" % data.split(b"\0", 1)[0].decode("ascii", "replace"))
        elif tag == TAG_SWVR:
            out.append("swvr %s" % data.split(b"\0", 1)[0].decode("ascii", "replace"))
        elif tag == TAG_HWVR:
            out.append("hwvr %08x" % struct.unpack_from("<I", data, 4)[0])
    return out


def make_report(st, nand_rc, ftl_rc, notes):
    g = st.geom
    ids = list(st.chip_ids) + [0] * (4 - len(st.chip_ids))
    ext = list(st.chip_ext) + [0] * (4 - len(st.chip_ext))
    have_row = g[0] != 0 and st.chip_row != 0xffffffff
    L = [MAGIC, "version emCORE n3g collect 1",
         "ids %08x %08x %08x %08x" % tuple(ids[:4]),
         "ext %08x %08x %08x %08x" % tuple(ext[:4]),
         "banks %u" % (g[0] if g[0] else sum(1 for i in ids if i)),
         "nand %d" % (nand_rc if nand_rc > ERR_FTL else 0)]
    if not have_row:
        L.append("row none")
    else:
        L += ["row %d" % st.chip_row, "mode %u" % g[8], "pagesize %u" % g[7], "blocks %u" % g[1], "ppb %u" % g[2],
              "userblocks %u" % g[4], "planes %u" % g[3], "vflspares %u" % g[5], "validated %d" % g[10],
              "ftl %d" % ftl_rc, "sectors %u" % st.ftl_sectors]
    L.append("verdict %s" % verdict(nand_rc, have_row, ftl_rc, g[10]))
    L += notes
    return "\n".join(L) + "\n", have_row


def progress(done, total, start, what):
    rate = done / max(time.time() - start, 1e-3)
    eta = (total - done) / rate if rate else 0
    sys.stderr.write("\r%s %5.1f%%  %d/%d  eta %dm%02ds   " % (
        what, 100.0 * done / max(total, 1), done, total, eta // 60, eta % 60))


def scan_meta(s, banks, blocks, ppb, result_cap):
    """meta.bin layout (bank by bank) from META_SCAN jobs of as many blocks as the result buffer holds"""
    per_block = banks * ppb * RECORD
    step = max(1, result_cap // per_block)
    meta = bytearray(banks * blocks * ppb * RECORD)
    start = time.time()
    for first in range(0, blocks, step):
        n = min(step, blocks - first)
        done = s.job(Cmd.NAND_META_SCAN, first, n, timeout=120.0)
        nand.check(done)
        data = s.read(done.result_addr, n * per_block)
        for i in range(n):
            for bank in range(banks):
                src = (i * banks + bank) * ppb * RECORD
                dst = (bank * blocks + first + i) * ppb * RECORD
                meta[dst:dst + ppb * RECORD] = data[src:src + ppb * RECORD]
        progress(first + n, blocks, start, "spare records")
    sys.stderr.write("\n")
    return meta


def summarize(meta, banks, blocks, ppb):
    """nandcheck.py's pass: results.bin, normalised meta.bin, control page list, summary"""
    counts = {"data": 0, "erased": 0, "ftl": 0, "vfl": 0, "other": 0, "ecc-failed": 0, "timeout": 0,
              "erased-flag-but-written": 0, "erased-type-no-flag": 0}
    corrected = [0] * 9
    control = []
    results = bytearray(banks * blocks * ppb * 4)
    for g in range(banks * blocks * ppb):
        off = g * RECORD
        word = struct.unpack_from("<i", meta, off + 12)[0]
        results[g * 4:(g + 1) * 4] = meta[off + 12:off + 16]
        kind = meta[off + 9]
        if word < 0:
            counts["timeout"] += 1
            norm = -1
        else:
            erased = bool(word & 0x10000)
            failed = (word & 0xff) == 3
            if failed:
                counts["ecc-failed"] += 1
            if erased and kind != ERASED:
                counts["erased-flag-but-written"] += 1
            if kind == ERASED and not erased:
                counts["erased-type-no-flag"] += 1
            if not erased:
                corrected[min((word >> 8) & 0xff, 8)] += 1
            norm = 0 if erased else 3 if failed else 1
        struct.pack_into("<i", meta, off + 12, norm)
        if kind in DATA_TYPES:
            counts["data"] += 1
        elif kind == ERASED:
            counts["erased"] += 1
        else:
            counts["ftl" if 0x43 <= kind <= 0x4f else "vfl" if kind == 0x80 else "other"] += 1
            control.append(g)
    summary = "".join("%s %d\n" % kv for kv in counts.items())
    summary += "corrected-bits %s\n" % " ".join("%d:%d" % (b, c) for b, c in enumerate(corrected) if c)
    return bytes(results), control, summary


def read_control(s, control, blocks, ppb, pagesize):
    pages = io.BytesIO()
    start = time.time()
    for i, g in enumerate(control):
        bank, page = divmod(g, blocks * ppb)
        done = s.job(Cmd.NAND_READ_PAGE, bank, page, timeout=10.0)
        # data as the chip gives it, whatever the ECC verdict (meta.bin says which pages failed)
        pages.write(struct.pack("<II", bank, page))
        pages.write(s.read(done.result_addr, pagesize))
        if i % 64 == 0 or i + 1 == len(control):
            progress(i + 1, len(control), start, "control pages")
    if control:
        sys.stderr.write("\n")
    return pages.getvalue()


def write_archive(out_dir, name, files):
    manifest = "".join("%s  %s\n" % (hashlib.sha256(v).hexdigest(), k) for k, v in files.items())
    files = dict(files, SHA256SUMS=manifest.encode())
    out = os.path.join(out_dir, name + ".tar.gz")
    with tarfile.open(out, "w:gz") as tar:
        for k, v in files.items():
            info = tarfile.TarInfo(name + "/" + k)
            info.size = len(v)
            info.mtime = int(time.time())
            tar.addfile(info, io.BytesIO(v))
    return out


def collect(s, out_dir, backend):
    os.makedirs(out_dir, exist_ok=True)
    st = s.status()
    if not st.step_ok_mask & (1 << 1):
        nand.check(s.job(Cmd.NAND_STEP, 1, 0))
    if s.status().backend != BACKENDS[backend] or not s.status().step_ok_mask & (1 << 3):
        nand.check(s.job(Cmd.NAND_BACKEND, BACKENDS[backend]))
    done = s.job(Cmd.NAND_STEP, 3, 0, timeout=30.0)       # may fail: the report says why
    nand_rc = s32(done.step_status[3])
    ftl_rc = 1
    if done.step_ok_mask & (1 << 3):
        done = s.job(Cmd.NAND_STEP, 4, 0, timeout=600.0, progress=True)   # read-only mount
        rc = s32(done.step_status[4])
        ftl_rc = rc - ERR_FTL if rc <= ERR_FTL else rc
    st = s.status()
    notes = []
    try:
        s.em.bootflashread(st.result_addr, 0, 0x1000)
        notes = syscfg_lines(s.read(st.result_addr, 0x1000))
    except Exception as e:      # the report is still useful without the SysCfg lines
        print("SysCfg not read: %s" % e, file=sys.stderr)
    text, have_row = make_report(st, nand_rc, ftl_rc, notes)
    print(text.rstrip())
    rep = dict(l.split(" ", 1) for l in text.splitlines()[1:] if " " in l)
    name = "nandcheck-%s-x%s" % (rep["ids"].split()[0], rep["banks"])
    files = {"report.txt": text.encode()}
    if not have_row:
        print("\nThe chip is not in Apple's table; only the report is saved.")
    else:
        g = st.geom
        banks, blocks, ppb, pagesize = g[0], g[1], g[2], g[7]
        meta = scan_meta(s, banks, blocks, ppb, st.result_cap)
        results, control, summary = summarize(meta, banks, blocks, ppb)
        files["meta.bin"] = bytes(meta)
        files["results.bin"] = results
        files["pages.bin"] = read_control(s, control, blocks, ppb, pagesize)
        files["summary.txt"] = summary.encode()
        print(summary.rstrip())
    out = write_archive(out_dir, name, files)
    print("\nwrote %s" % out)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="n3g.py collect", description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=".", help="directory for nandcheck-<id>-x<banks>.tar.gz")
    ap.add_argument("--backend", choices=tuple(BACKENDS), default="sw",
                    help="physical backend that reads the pages (default sw, the Rockbox driver)")
    ap.add_argument("--bulk", action="store_true", help="read results over the bulk endpoint")
    ap.add_argument("--settle", type=float, default=0.02)
    ap.add_argument("--poll", type=float, default=0.02)
    a = ap.parse_args(argv)
    s = None
    try:
        s = nand.make_dev_session(settle=a.settle, poll=a.poll, ep0=not a.bulk)
        return collect(s, a.out, a.backend)
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()


if __name__ == "__main__":
    sys.exit(main())
