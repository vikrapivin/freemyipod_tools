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
"""Read and inspect nano3g NAND through the emCORE worker protocol (target/ipodnano3g/nand3g.h)."""
import argparse
import os
import struct
import sys
import time
import zlib

from . import session as transport
from .protocol import Cmd
from .session import ToolError

# ------------------------------------------------------------------------------------------- constants (nand3g.h)
MAGIC, VERSION, STATUS_BYTES = 0x4e47334e, 1, 0x110
STATE_NAMES = {0: "IDLE", 1: "QUEUED", 2: "RUNNING", 3: "DONE"}
RESULT_NAMES = {0: "OK", 1: "LIB_ERROR", 2: "PRECOND", 3: "BAD_ARG", 4: "NO_MEMORY", 5: "NOT_ALLOWED"}
ACCEPT_NAMES = {0: "OK", 1: "BUSY", 2: "BAD_ARG", 3: "NOT_INIT"}
JOB_NAMES = {0: "none", 1: "step", 3: "read_page"}
STEP_NAMES = {1: "clock gates (FMC, ECC)", 2: "NAND pad snapshot", 3: "backend init (reset, READ ID, chip table)"}
BACKENDS = {"sw": 0}
BACKEND_NAMES = {v: k for k, v in BACKENDS.items()}
GEOM_NAMES = ("banks", "blocks_per_bank", "pages_per_block", "planes", "userblocks_per_bank", "vfl_spares",
              "twoplane", "page_bytes", "chip_mode", "layout", "validated", "chip_id")
# driver return codes as the harness stores them (uint32 of a negative int)
LIB_RC_NAMES = {0: "ok", 0xffffffff: "controller timeout / generic (-1)", 0xfffffffe: "chip reported failure / identify failed (-2)", 0xfffffffd: "READ STATUS never completed (-3)",
                0xfffffff9: "NOSYS (backend does not implement it)", 0xfffffff8: "NOINIT",
                0xfffffffb: "unsupported chip", 0xfffffffa: "mixed chips", 0xffffff9c: "FTL"}
ECC_NAMES = {0: "clean", 1: "corrected", 3: "UNCORRECTABLE", 6: "status6"}
WRITE_PROTOCOL = 1
RP_DATA, RP_META, RP_ECC, RP_RESULT, RP_BYTES = 0x0, 0x1000, 0x1010, 0x1014, 0x1020
SECTOR_BYTES = 2048
RESULT_BYTES = 256 * 1024

# status record layout: (name, struct format); must match struct n3g_nand_status (size 0x110)
_FIELDS = [("magic", "I"), ("version", "I"), ("size", "I"), ("busy", "I"), ("state", "I"), ("step_ok_mask", "I"),
           ("jobs_started", "I"), ("jobs_done", "I"), ("job_kind", "I"), ("job_arg", "3I"), ("job_result", "I"),
           ("job_lib_rc", "I"), ("job_detail", "4I"), ("job_t_start", "I"), ("job_t_end", "I"),
           ("progress_done", "I"), ("progress_total", "I"), ("step_status", "8I"), ("backend", "I"),
           ("chip_ids", "4I"), ("chip_ext", "4I"), ("geom", "12I"), ("ftl_sectors", "I"), ("ftl_sector_bytes", "I"),
           ("ftl_writable", "I"), ("result_addr", "I"), ("result_cap", "I"), ("result_len", "I"), ("now_us", "I"),
           ("hb_word", "I"), ("pwrcon0_before", "I"), ("pwrcon0_after", "I"), ("pcon", "3I"), ("chip_row", "I"), ("fmss_fault_addr", "I"), ("fmss_fault_count", "I"), ("fmss_fault_pc", "I")]
_FMT = "<" + "".join(f for _, f in _FIELDS)
assert struct.calcsize(_FMT) == STATUS_BYTES


class Bunch(dict):
    __getattr__ = dict.__getitem__


def parse_status(raw):
    """Decode the 0x110-byte n3g_nand_status record into a Bunch (arrays become tuples)."""
    if len(raw) < STATUS_BYTES:
        raise ToolError("status record too short (%d bytes)" % len(raw))
    vals = struct.unpack(_FMT, raw[:STATUS_BYTES])
    out, i = Bunch(), 0
    for name, f in _FIELDS:
        n = int(f[:-1] or 1)
        if n == 1:
            out[name] = vals[i]
            i += 1
        else:
            out[name] = tuple(vals[i:i + n])
            i += n
    return out


def format_status(st):
    L = ["magic %08x version %d size 0x%x backend %s" % (st.magic, st.version, st.size,
                                                        BACKEND_NAMES.get(st.backend, st.backend)),
         "job #%d/%d %s state %s busy %d result %s lib_rc 0x%08x (%s) args %s detail %s" % (
             st.jobs_done, st.jobs_started, JOB_NAMES.get(st.job_kind, st.job_kind), STATE_NAMES.get(st.state, st.state),
             st.busy, RESULT_NAMES.get(st.job_result, st.job_result), st.job_lib_rc,
             LIB_RC_NAMES.get(st.job_lib_rc, "?"), " ".join("%x" % v for v in st.job_arg),
             " ".join("%x" % v for v in st.job_detail)),
         "steps ok mask 0x%x: %s" % (st.step_ok_mask, ", ".join(
             "%d=%s" % (i, "-" if v == 0xffffffff else "RUN" if v == 0xfffffffe else "ok" if v == 0 else "0x%x" % v)
             for i, v in enumerate(st.step_status) if i)),
         "FTL sectors %d x %d B, writable %d; result buffer 0x%08x cap 0x%x len 0x%x" % (
             st.ftl_sectors, st.ftl_sector_bytes, st.ftl_writable, st.result_addr, st.result_cap, st.result_len)]
    if st.pwrcon0_before or st.pwrcon0_after:
        L.append("PWRCON0 before 0x%08x after 0x%08x; PCON8/9/10 %s" % (
            st.pwrcon0_before, st.pwrcon0_after, " ".join("%08x" % v for v in st.pcon)))
    return "\n".join(L)


# ------------------------------------------------------------------------------------------- sessions
class Session3G:
    """Transport independent part: needs cmd(c, a, b, d) -> (w1, w2, w3), read_chunk(addr, size), sleep(s), now()."""
    settle = 0.05
    poll = 0.05

    def status(self):
        addr, size, _ = self.cmd(Cmd.NAND_STATUS)
        if size != STATUS_BYTES:
            raise ToolError("device status size 0x%x != tool 0x%x (rebuild/update)" % (size, STATUS_BYTES))
        st = parse_status(self.read(addr, size))
        if st.magic != MAGIC or st.version != VERSION:
            raise ToolError("status record magic/version %08x/%d not N3GN/%d" % (st.magic, st.version, VERSION))
        return st

    def job(self, c, a=0, b=0, timeout=10.0, progress=False, d=0):
        """Queue one job, poll until it is done. Raises ToolError (after printing the record) on a stuck job."""
        self.sleep(self.settle)
        acc, job_id, _ = self.cmd(c, a, b, d)
        if acc != 0:
            raise ToolError("command %d not accepted: %s" % (c, ACCEPT_NAMES.get(acc, acc)))
        return self.wait(job_id, timeout, progress)

    def wait(self, job_id, timeout, progress=False):
        t0 = self.now()
        last = -1
        while True:
            self.sleep(self.poll)
            st = self.status()
            if st.jobs_done == job_id and st.state == 3:
                return st
            if progress and st.progress_done != last:
                last = st.progress_done
                print("  ... %s %d/%d  (%.0f s)" % (STATE_NAMES.get(st.state), st.progress_done,
                      st.progress_total, self.now() - t0), file=sys.stderr)
            if self.now() - t0 > timeout:
                print(format_status(st), file=sys.stderr)
                raise ToolError("job %d not done after %.0f s" % (job_id, timeout))

    def result(self, st, n):
        return self.read(st.result_addr, n)

    def read(self, addr, size):          # chunked READ_MEM (<= 0xff0 per control transfer)
        out = b""
        while size:
            n = min(size, 0xf00)
            out += self.read_chunk(addr, n)
            addr += n
            size -= n
        return out


def make_dev_session(**kw):
    class DevSession(Session3G, transport.Session):
        def __init__(self, settle=0.05, poll=0.05, ep0=True):
            self.settle, self.poll = settle, poll
            transport.Session.__init__(self, settle=settle, poll=poll, ep0=ep0)

        def cmd(self, c, a=0, b=0, d=0):
            return transport.Session.cmd(self, c, a, b, d)

        def read_chunk(self, addr, size):
            return transport.Session.read_chunk(self, addr, size)

        def sleep(self, s):
            time.sleep(s)

        def now(self):
            return time.time()
    return DevSession(**kw)


def check(st, ok_lib=(0,)):
    if st.job_result != 0 or st.job_lib_rc not in ok_lib:
        print(format_status(st), file=sys.stderr)
        raise ToolError("job failed: %s, lib rc 0x%08x" % (RESULT_NAMES.get(st.job_result, st.job_result), st.job_lib_rc))
    return st


def hexdump(data, limit=256):
    out = []
    for off in range(0, min(len(data), limit), 16):
        chunk = data[off:off + 16]
        out.append("%04x  %-47s  %s" % (off, " ".join("%02x" % b for b in chunk),
                                        "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)))
    return "\n".join(out)


# ------------------------------------------------------------------------------------------- commands
def cmd_status(s, a):
    st = s.status()
    print(format_status(st))
    return st


def cmd_step(s, a):
    st = check(s.job(Cmd.NAND_STEP, a.n, a.arg, timeout=a.timeout or 120.0, progress=a.n == 4))
    print("step %d (%s): ok, took %d us" % (a.n, STEP_NAMES.get(a.n, "?"), (st.job_t_end - st.job_t_start) & 0xffffffff))
    return st


def cmd_init(s, a):
    for n in (1, 2, 3):
        a.n, a.arg = n, 0
        cmd_step(s, a)
    return cmd_id(s, a)


def cmd_id(s, a):
    st = s.status()
    if not st.step_ok_mask & (1 << 3):
        print("step 3 (backend init) has not succeeded; values below are whatever the last attempt left", file=sys.stderr)
    for bank, (v, e) in enumerate(zip(st.chip_ids, st.chip_ext)):
        print("bank %d: %08x %08x  bytes %s" % (bank, v, e, " ".join("%02x" % x for x in struct.pack("<I", v))))
    print("driver geometry (chip table):")
    for n, v in zip(GEOM_NAMES, st.geom):
        print("  %-20s 0x%x (%d)" % (n, v, v))
    return st


def cmd_rawread(s, a):
    st = check(s.job(Cmd.NAND_READ_PAGE, a.bank, a.page), ok_lib=(0,))
    bpp = st.geom[7]
    r = s.result(st, RP_BYTES)
    data, meta = r[RP_DATA:RP_DATA + bpp], r[RP_META:RP_META + 12]
    ecc, res = struct.unpack_from("<II", r, RP_ECC)
    print("bank %d page %d: ecc %s, read result 0x%08x, meta %s" % (a.bank, a.page, ECC_NAMES.get(ecc, ecc), res, meta.hex()))
    if a.output:
        open(a.output, "wb").write(data + meta)
        print("wrote %s (%d data + 12 meta)" % (a.output, bpp))
    else:
        print(hexdump(data, limit=256))
    return st, data, meta, ecc


FAULT_FIELDS = ["magic", "why", "set", "prog", "pc"] + ["r%d" % i for i in range(8)] + ["c3c", "c50", "cs_irq"] + \
               ["fmc%02x" % (4 * i) for i in range(20)] + ["ecc80c", "ecc810", "ecc814", "ecc840", "count", "pc_last"] + \
               ["pcs%d" % i for i in range(8)] + ["pcsn%d" % i for i in range(8)]


HANDLERS = {'status': cmd_status, 'step': cmd_step, 'init': cmd_init, 'id': cmd_id, 'rawread': cmd_rawread}


def ai(x):
    return int(x, 0)


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--settle", type=float, default=0.05)
    ap.add_argument("--poll", type=float, default=0.05)
    ap.add_argument("--timeout", type=float, default=0.0)
    ap.add_argument("--bulk", action="store_true", help="use the bulk endpoint for reads")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    p = sub.add_parser("step"); p.add_argument("n", type=int); p.add_argument("--arg", type=ai, default=0)
    p = sub.add_parser("init")
    sub.add_parser("id")
    p = sub.add_parser("rawread"); p.add_argument("bank", type=ai); p.add_argument("page", type=ai); p.add_argument("-o", "--output")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    s = None
    try:
        s = make_dev_session(settle=a.settle, poll=a.poll, ep0=not a.bulk)
        HANDLERS[a.cmd](s, a)
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
