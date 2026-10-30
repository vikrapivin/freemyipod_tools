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

"""FTL write test on the unit (the on-device step of Rockbox's chip validation), through emCORE.

Through the FTL's own write path (ftl_write + ftl_sync, monitor commands 124/129/130/131):
  1. mount writable (step 4; --unvalidated lets a chip row not validated on hardware mount writable)
  2. save the sectors it will overwrite (to the host, and to --save FILE unless --no-save)
  3. write a pattern stamped with each sector number, flush, remount, read back and compare
  4. write the saved sectors back, flush, remount, read back and compare with the saved copy
  5. remount read-only
--keep stops after 3 with the pattern left on the medium: restart the iPod, load emCORE again and run
--resume FILE, which checks that the pattern survived the restart, then does 4 and 5.

A failure between 3 and 4 leaves the pattern in those sectors; the saved copy (--save) puts them back:
  n3g.py write-test --resume FILE

  n3g.py write-test [--first-sector N] [--sectors 1024] [--unvalidated] [--save FILE | --no-save] [--keep]
  n3g.py write-test --resume FILE [--unvalidated]
"""
import argparse
import json
import os
import struct
import sys
import time
import zlib

from . import nand
from .nand import BACKENDS, SECTOR_BYTES, Cmd, ToolError

CHUNK = 128                 # sectors per FTL read / write job (256 KiB result buffer)
TAG = b"N3GWTEST"


def pattern(sector, seed):
    """2048 bytes: tag, sector number, seed, then a sector-dependent word stream"""
    head = TAG + struct.pack("<II", sector, seed)
    words = struct.pack("<%dI" % ((SECTOR_BYTES - len(head)) // 4),
                        *(((sector * 0x9e3779b1) ^ (i * 0x85ebca6b) ^ seed) & 0xffffffff
                          for i in range((SECTOR_BYTES - len(head)) // 4)))
    return head + words


def patterns(first, count, seed):
    return b"".join(pattern(first + i, seed) for i in range(count))


def mount(s, writable, unvalidated):
    arg = (1 if writable else 0) | (2 if unvalidated else 0)
    st = s.job(Cmd.NAND_STEP, 4, arg, timeout=600.0, progress=True)
    nand.check(st)
    st = s.status()
    if writable and not st.ftl_writable:
        if not st.geom[10] and not unvalidated:
            raise ToolError("chip 0x%08x is not validated on hardware, so the FTL mounted read-only: pass --unvalidated "
                            "to run the write test on it" % st.geom[11])
        raise ToolError("the FTL mounted read-only (a commit it would need could not be loaded); nothing was written")
    return st


def ftl_read(s, first, count):
    out = bytearray()
    for off in range(0, count, CHUNK):
        n = min(CHUNK, count - off)
        st = nand.check(s.job(Cmd.NAND_FTL_READ, first + off, n, timeout=120.0))
        out += s.result(st, n * SECTOR_BYTES)
    return bytes(out)


def ftl_write(s, first, data):
    count = len(data) // SECTOR_BYTES
    nand.check(s.job(Cmd.NAND_WRITE_OPEN))
    for off in range(0, count, CHUNK):
        n = min(CHUNK, count - off)
        chunk = data[off * SECTOR_BYTES:(off + n) * SECTOR_BYTES]
        st = s.status()
        s.write_memory(st.result_addr, chunk)
        nand.check(s.job(Cmd.NAND_BLOCK_WRITE, first + off, n, d=zlib.crc32(chunk) & 0xffffffff, timeout=120.0))
    nand.check(s.job(Cmd.NAND_FLUSH, timeout=300.0))


def compare(name, got, want, first):
    bad = [first + i for i in range(len(want) // SECTOR_BYTES)
           if got[i * SECTOR_BYTES:(i + 1) * SECTOR_BYTES] != want[i * SECTOR_BYTES:(i + 1) * SECTOR_BYTES]]
    print("%s: %d sectors, %d differ%s" % (name, len(want) // SECTOR_BYTES, len(bad),
                                           (" (first %d)" % bad[0]) if bad else ""))
    return bad


def timed(what, fn, *args):
    t0 = time.time()
    r = fn(*args)
    print("  %s: %.1f s" % (what, time.time() - t0))
    return r


def bring_up(s, backend):
    st = s.status()
    if not st.step_ok_mask & (1 << 1):
        nand.check(s.job(Cmd.NAND_STEP, 1, 0))
    if s.status().backend != BACKENDS[backend] or not s.status().step_ok_mask & (1 << 3):
        nand.check(s.job(Cmd.NAND_BACKEND, BACKENDS[backend]))
        nand.check(s.job(Cmd.NAND_STEP, 3, 0, timeout=30.0))
    st = s.status()
    print("chip 0x%08x x%d, %s on hardware, backend %s" % (st.geom[11], st.geom[0],
          "validated" if st.geom[10] else "NOT validated", backend))
    return st


def restore(s, first, saved, unvalidated):
    print("4: write the saved sectors back")
    timed("write + flush", ftl_write, s, first, saved)
    mount(s, True, unvalidated)
    got = timed("remount + read", ftl_read, s, first, len(saved) // SECTOR_BYTES)
    return compare("restored data", got, saved, first)


def run(s, a):
    bring_up(s, a.backend)
    if a.resume:
        with open(a.resume + ".json") as f:
            meta = json.load(f)
        saved = open(a.resume, "rb").read()
        first, count, seed = meta["first_sector"], meta["sectors"], meta["seed"]
        if len(saved) != count * SECTOR_BYTES or zlib.crc32(saved) & 0xffffffff != meta["crc32"]:
            raise ToolError("%s does not match its .json (size or CRC)" % a.resume)
        st = mount(s, True, a.unvalidated)
        if st.geom[11] != meta["chip_id"] or st.ftl_sectors != meta["ftl_sectors"]:
            raise ToolError("this unit (chip 0x%08x, %d sectors) is not the one %s was saved from" % (
                st.geom[11], st.ftl_sectors, a.resume))
        got = timed("read", ftl_read, s, first, count)
        persisted = compare("pattern after the restart", got, patterns(first, count, seed), first)
        bad = restore(s, first, saved, a.unvalidated)
        mount(s, False, False)
        print("5: remounted read-only")
        ok = not persisted and not bad
        print("RESULT: %s" % ("PASS (pattern survived the restart, data restored)" if ok else "FAIL"))
        return 0 if ok else 2

    st = mount(s, True, a.unvalidated)
    total = st.ftl_sectors
    count = a.sectors
    first = a.first_sector if a.first_sector is not None else (total // 2) // CHUNK * CHUNK
    if count < 1 or first + count > total:
        raise ToolError("sectors %d..%d outside the FTL's %d" % (first, first + count - 1, total))
    seed = a.seed
    print("FTL %d sectors of %d B, mounted writable; test sectors %d..%d (%.1f MiB)" % (
        total, SECTOR_BYTES, first, first + count - 1, count * SECTOR_BYTES / 1048576.0))

    print("2: save the sectors to be overwritten")
    saved = timed("read", ftl_read, s, first, count)
    save = None if a.no_save else (a.save or "write-test-%08x-%d.bin" % (st.geom[11], first))
    if save:
        with open(save, "wb") as f:
            f.write(saved)
        with open(save + ".json", "w") as f:
            json.dump({"first_sector": first, "sectors": count, "seed": seed, "chip_id": st.geom[11],
                       "ftl_sectors": total, "crc32": zlib.crc32(saved) & 0xffffffff}, f, indent=1)
        print("  saved to %s (+ .json); 'n3g.py write-test --resume %s' puts them back" % (save, save))

    print("3: write the pattern, flush, remount, read back")
    pat = patterns(first, count, seed)
    timed("write + flush", ftl_write, s, first, pat)
    mount(s, True, a.unvalidated)
    got = timed("remount + read", ftl_read, s, first, count)
    pbad = compare("pattern", got, pat, first)
    if a.keep and not pbad:
        mount(s, False, False)
        print("pattern left on the medium (FTL remounted read-only). Restart the iPod (MENU+SELECT), load emCORE "
              "again and run:\n  n3g.py write-test --resume %s%s" % (save, " --unvalidated" if a.unvalidated else ""))
        return 0
    bad = restore(s, first, saved, a.unvalidated)
    mount(s, False, False)
    print("5: remounted read-only")
    ok = not pbad and not bad
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    if bad and save:
        print("the restore differs: the saved copy is in %s" % save)
    return 0 if ok else 2


def main(argv=None):
    ap = argparse.ArgumentParser(prog="n3g.py write-test", description=__doc__.split("\n")[0])
    ap.add_argument("--first-sector", type=nand.ai, default=None, help="default: the middle of the FTL")
    ap.add_argument("--sectors", type=int, default=1024, help="2048-byte sectors (default 1024 = 2 MiB)")
    ap.add_argument("--seed", type=nand.ai, default=0x5eed1234)
    ap.add_argument("--unvalidated", action="store_true", help="allow a chip row not validated on hardware")
    ap.add_argument("--backend", choices=tuple(BACKENDS), default="sw")
    ap.add_argument("--save", help="file for the saved sectors (default write-test-<chip>-<sector>.bin)")
    ap.add_argument("--no-save", action="store_true", help="keep the saved sectors in memory only")
    ap.add_argument("--keep", action="store_true", help="stop after the pattern check (restart test)")
    ap.add_argument("--resume", metavar="FILE", help="after a restart: check the pattern, then restore FILE")
    ap.add_argument("--settle", type=float, default=0.02)
    ap.add_argument("--poll", type=float, default=0.05)
    a = ap.parse_args(argv)
    if a.keep and a.no_save:
        print("ERROR: --keep needs the saved copy on disk (drop --no-save)", file=sys.stderr)
        return 1
    s = None
    try:
        s = nand.make_dev_session(settle=a.settle, poll=a.poll)
        return run(s, a)
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()


if __name__ == "__main__":
    sys.exit(main())
