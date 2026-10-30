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
"""NAND benchmark host side: parameter packing, result decoding, run, report and compare.

Device side: target/ipodnano3g/nand3g.c (job N3G_JOB_BENCH, command 136). The parameter block goes to the result
buffer with WRITE_MEM, the result struct comes back from result_addr + 0x100 and the per-repeat microseconds from
result_addr + 0x400. Counters are u32 on the device and wrap; throughput is derived here from pages and page size.
"""
import argparse
import json
import os
import random
import struct
import sys
import time

from . import nand
from .nand import BACKENDS, BACKEND_NAMES, Cmd, ToolError

PARAM_MAGIC, RESULT_MAGIC, BENCH_VERSION = 0x4247334e, 0x5247334e, 1
PARAM_BYTES, RESULT_BYTES = 0x80, 0x100
RES_OFF, REP_OFF, MAXREP = 0x100, 0x400, 1024
MAXBATCH, MAXFTLBATCH, MAXENTRIES = 32, 64, 8192
OPS = {"read": 1, "write": 2, "ftl": 3, "cycle": 4, "ftl-cycle": 5}
OP_NAMES = {v: k for k, v in OPS.items()}
F_WRITE, F_INTERLEAVE, F_SCATTER, F_TWOPLANE, F_UNVALIDATED, F_VERIFY, F_ERASE_AFTER, F_LOAD = (1 << i for i in range(8))
FLAG_NAMES = [(F_WRITE, "write"), (F_INTERLEAVE, "interleave"), (F_SCATTER, "scatter"), (F_TWOPLANE, "two-plane"),
              (F_UNVALIDATED, "unvalidated"), (F_VERIFY, "verify"), (F_ERASE_AFTER, "erase-after"), (F_LOAD, "load")]
CPU_OFF, CPU_MAGIC = 0x200, 0x4347334e
_CFMT = "<4I8I8I8I4I"
KEEP_POLL = 0xffffffff
# phase index -> name, per op (struct n3g_bench_cpu); phase 0 = ops without phases
PHASES = {"cycle": {1: "save", 2: "erase", 3: "program", 4: "verify", 5: "rst_erase", 6: "rst_program", 7: "rst_verify"},
          "ftl-cycle": {1: "save", 2: "sync", 3: "write", 4: "verify", 5: "rst_sync", 6: "rst_write", 7: "rst_verify"}}
KEEP_BACKEND = 0xffffffff

# struct n3g_bench_params: 15 named words + 17 reserved
PARAM_FIELDS = ("magic", "version", "op", "backend", "flags", "bank_mask", "first_block", "nblocks", "pages", "batch",
                "repeats", "seed", "first_sector", "sectors", "fmss_poll_us", "timing")
_PFMT = "<16I16I"
# struct n3g_bench_result, in order; "reserved" pads to 0x100 bytes
RESULT_FIELDS = ("magic", "version", "op", "backend", "status", "flags", "repeats_done", "batches",
                 "read_pages", "read_bytes", "read_us", "ecc_clean", "ecc_corrected", "ecc_failed", "read_errors",
                 "batch_us_min", "batch_us_max", "erase_blocks", "erase_us", "erase_fail",
                 "prog_pages", "prog_bytes", "prog_us", "prog_fail",
                 "verify_pages", "verify_us", "verify_mismatch", "verify_errors",
                 "ftl_sectors", "ftl_bytes", "ftl_us", "ftl_errors", "total_us",
                 "first_err_rc", "first_err_bank", "first_err_page", "erased_pages", "max_corrected",
                 "pages_per_block", "page_size", "entries", "reps_recorded",
                 "cyc_failed_phase", "save_pages", "save_us", "save_erased", "rst_erase_blocks", "rst_erase_us",
                 "rst_prog_pages", "rst_prog_us", "rst_verify_pages", "rst_verify_us", "rst_mismatch", "rst_errors",
                 "save_addr", "save_bytes",
                 "ftl_wr_us", "ftl_sync_us", "ftl_vfy_us", "ftl_rst_wr_us", "ftl_rst_sync_us", "ftl_rst_vfy_us",
                 "ftl_vfy_mismatch", "ftl_rst_mismatch")
CYC_PHASES = {0: "completed", 1: "save read (nothing written)", 2: "pattern erase", 3: "pattern program",
              4: "pattern verify", 5: "restore erase", 6: "restore program", 7: "restore verify"}
_RFMT = "<%dI%dI" % (len(RESULT_FIELDS), RESULT_BYTES // 4 - len(RESULT_FIELDS))
assert struct.calcsize(_PFMT) == PARAM_BYTES and struct.calcsize(_RFMT) == RESULT_BYTES


# ------------------------------------------------------------------------------------------- packing
def parse_banks(text):
    """'0,1' or '0-3' or '0,2-3' -> bank mask."""
    mask = 0
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = (int(x, 0) for x in part.split("-", 1))
            if hi < lo:
                raise ToolError("bad bank range %r" % part)
            bits = range(lo, hi + 1)
        else:
            bits = (int(part, 0),)
        for b in bits:
            if not 0 <= b < 8:
                raise ToolError("bank %d out of range 0..7" % b)
            mask |= 1 << b
    if not mask:
        raise ToolError("no banks selected")
    return mask


def mask_banks(mask):
    return [b for b in range(8) if mask >> b & 1]


def flag_names(flags):
    return [n for bit, n in FLAG_NAMES if flags & bit]


def make_params(op, backend="sw", flags=0, bank_mask=1, first_block=0, nblocks=1, pages=1, batch=1, repeats=1,
                seed=0x1234abcd, first_sector=0, sectors=0, fmss_poll_us=KEEP_POLL, timing=0):
    """Pack a parameter block (0x80 bytes). Raises ToolError for values the device would reject."""
    if op not in OPS:
        raise ToolError("unknown benchmark op %r" % (op,))
    if isinstance(backend, str) and backend not in BACKENDS:
        raise ToolError("unknown backend %r" % (backend,))
    be = KEEP_BACKEND if backend is None else BACKENDS[backend] if isinstance(backend, str) else int(backend)
    if be != KEEP_BACKEND and be not in BACKEND_NAMES:
        raise ToolError("unknown backend %r" % (backend,))
    if repeats < 1:
        raise ToolError("repeats must be >= 1")
    if op in ("ftl", "ftl-cycle"):
        if not 1 <= batch <= MAXFTLBATCH:
            raise ToolError("ftl batch must be 1..%d sectors" % MAXFTLBATCH)
        if sectors < 1:
            raise ToolError("--sectors must be >= 1")
    else:
        if not 1 <= batch <= MAXBATCH:
            raise ToolError("batch must be 1..%d pages" % MAXBATCH)
        nb = bin(bank_mask).count("1")
        if nb * nblocks * pages > MAXENTRIES:
            raise ToolError("banks x blocks x pages = %d exceeds %d pages per repeat" % (nb * nblocks * pages, MAXENTRIES))
        if nblocks < 1 or pages < 1:
            raise ToolError("blocks and pages must be >= 1")
    if op == "cycle":
        flags = (flags | F_WRITE | F_INTERLEAVE) & ~(F_SCATTER | F_TWOPLANE)
    elif op == "ftl-cycle":
        flags = (flags | F_WRITE) & ~(F_SCATTER | F_TWOPLANE | F_INTERLEAVE)
    elif op == "write":
        flags |= F_WRITE
        if flags & F_SCATTER:
            raise ToolError("scatter is not supported for programming (pages go in ascending order per block)")
        if flags & F_TWOPLANE and (first_block & 1 or nblocks & 1):
            raise ToolError("two-plane needs an even first block and an even block count")
    else:
        flags &= ~F_WRITE
    vals = [PARAM_MAGIC, BENCH_VERSION, OPS[op], be, flags, bank_mask, first_block, nblocks, pages, batch, repeats,
            seed, first_sector, sectors, fmss_poll_us & 0xffffffff, timing & 0xffffffff]
    return struct.pack(_PFMT, *vals, *([0] * 16))


def unpack_params(raw):
    vals = struct.unpack(_PFMT, raw[:PARAM_BYTES])
    d = dict(zip(PARAM_FIELDS, vals))
    if d["magic"] != PARAM_MAGIC:
        raise ToolError("bad parameter magic %08x" % d["magic"])
    return d


def unpack_result(raw):
    """Decode struct n3g_bench_result (0x100 bytes) into a dict; raises ToolError on a bad magic."""
    if len(raw) < RESULT_BYTES:
        raise ToolError("result block too short (%d bytes)" % len(raw))
    vals = struct.unpack(_RFMT, raw[:RESULT_BYTES])
    d = dict(zip(RESULT_FIELDS, vals))
    if d["magic"] != RESULT_MAGIC or d["version"] != BENCH_VERSION:
        raise ToolError("bench result magic/version %08x/%d not N3GR/%d" % (d["magic"], d["version"], BENCH_VERSION))
    return d


def unpack_cpu(raw):
    """struct n3g_bench_cpu -> dict (None if the device did not fill it)"""
    v = struct.unpack(_CFMT, raw[:struct.calcsize(_CFMT)])
    if v[0] != CPU_MAGIC:
        return None
    return {"load": v[1], "load_rate_per_ms": v[2], "fmss_poll_us": v[3], "phase_us": list(v[4:12]),
            "phase_load": list(v[12:20]), "phase_maxcorr": list(v[20:28]), "timing": v[28], "data_sig": v[29],
            "sig_pages": v[30], "ecc_fail_pages": v[31]}


def cpu_free(cpu, op):
    """{phase name: % of the CPU left for other work during that phase's backend calls} (needs the load thread)"""
    out = {}
    if not cpu or not cpu["load"] or not cpu["load_rate_per_ms"]:
        return out
    for i, name in PHASES.get(op, {0: "all"}).items():
        us = cpu["phase_us"][i]
        if us:
            out[name] = 100.0 * cpu["phase_load"][i] / (cpu["load_rate_per_ms"] * us / 1000.0)
    return out


def unpack_reps(raw, count):
    count = min(count, MAXREP, len(raw) // 4)
    return list(struct.unpack("<%dI" % count, raw[:4 * count]))


# ------------------------------------------------------------------------------------------- derived figures
def _rate(n, us):
    return n * 1e6 / us if us else 0.0


def derive(res):
    """Throughput figures from a decoded result. us are device microseconds; MB/s is 10^6 bytes per second."""
    d = {}
    page = res["page_size"]
    if res["op"] == OPS["ftl-cycle"]:
        sec = nand.SECTOR_BYTES
        n = res["ftl_sectors"]
        if n:
            d["ftl_save_MB_per_s"] = _rate(n * sec, res["ftl_us"]) / 1e6
        if res["prog_pages"]:
            w = res["prog_pages"] * sec
            d["ftl_write_MB_per_s"] = _rate(w, res["ftl_wr_us"]) / 1e6
            d["ftl_write_sync_MB_per_s"] = _rate(w, res["ftl_wr_us"] + res["ftl_sync_us"]) / 1e6
            d["ftl_sync_ms"] = res["ftl_sync_us"] / 1e3
            d["ftl_verify_MB_per_s"] = _rate(w, res["ftl_vfy_us"]) / 1e6
        if res["rst_prog_pages"]:
            w = res["rst_prog_pages"] * sec
            d["ftl_rst_write_MB_per_s"] = _rate(w, res["ftl_rst_wr_us"]) / 1e6
            d["ftl_rst_write_sync_MB_per_s"] = _rate(w, res["ftl_rst_wr_us"] + res["ftl_rst_sync_us"]) / 1e6
            d["ftl_rst_sync_ms"] = res["ftl_rst_sync_us"] / 1e3
            d["ftl_rst_verify_MB_per_s"] = _rate(w, res["ftl_rst_vfy_us"]) / 1e6
        return d
    if res["read_pages"]:
        d["read_pages_per_s"] = _rate(res["read_pages"], res["read_us"])
        d["read_MB_per_s"] = _rate(res["read_pages"] * page, res["read_us"]) / 1e6
        d["read_us_per_page"] = res["read_us"] / res["read_pages"]
        d["read_us_per_batch"] = res["read_us"] / res["batches"] if res["batches"] else 0.0
        d["ecc_corrected_fraction"] = res["ecc_corrected"] / res["read_pages"]
    if res["erase_blocks"]:
        d["erase_ms_per_block"] = res["erase_us"] / res["erase_blocks"] / 1e3
    if res["prog_pages"]:
        d["prog_pages_per_s"] = _rate(res["prog_pages"], res["prog_us"])
        d["prog_MB_per_s"] = _rate(res["prog_pages"] * page, res["prog_us"]) / 1e6
        d["prog_us_per_page"] = res["prog_us"] / res["prog_pages"]
    if res["verify_pages"]:
        d["verify_MB_per_s"] = _rate(res["verify_pages"] * page, res["verify_us"]) / 1e6
    if res.get("save_pages"):
        d["save_MB_per_s"] = _rate(res["save_pages"] * page, res["save_us"]) / 1e6
    if res.get("rst_erase_blocks"):
        d["rst_erase_ms_per_block"] = res["rst_erase_us"] / res["rst_erase_blocks"] / 1e3
    if res.get("rst_prog_pages"):
        d["rst_prog_MB_per_s"] = _rate(res["rst_prog_pages"] * page, res["rst_prog_us"]) / 1e6
    if res.get("rst_verify_pages"):
        d["rst_verify_MB_per_s"] = _rate(res["rst_verify_pages"] * page, res["rst_verify_us"]) / 1e6
    if res["ftl_sectors"]:
        d["ftl_sectors_per_s"] = _rate(res["ftl_sectors"], res["ftl_us"])
        d["ftl_KiB_per_s"] = _rate(res["ftl_sectors"] * nand.SECTOR_BYTES, res["ftl_us"]) / 1024
        d["ftl_us_per_sector"] = res["ftl_us"] / res["ftl_sectors"]
    return d


def make_report(params_raw, res, reps, backend_name=None, cpu=None):
    """The JSON document written by --json (also what 'compare' reads)."""
    p = unpack_params(params_raw)
    be = res["backend"]
    op = OP_NAMES.get(res["op"], str(res["op"]))
    d = derive(res)
    for name, pct in cpu_free(cpu, op).items():
        d["cpu_free_%s_pct" % name] = pct
    return {"tool": "n3g-bench", "version": 1,
            "backend": backend_name or BACKEND_NAMES.get(be, str(be)),
            "op": op,
            "params": dict(p, banks=mask_banks(p["bank_mask"]), flag_names=flag_names(p["flags"])),
            "result": res, "derived": d, "rep_us": reps, "cpu": cpu}


# ------------------------------------------------------------------------------------------- compare
# metric -> (label, True if a larger value is better)
METRICS = {"read_MB_per_s": ("raw read MB/s", True), "read_pages_per_s": ("raw read pages/s", True),
           "read_us_per_page": ("raw read us/page", False), "prog_MB_per_s": ("program MB/s", True),
           "prog_us_per_page": ("program us/page", False), "erase_ms_per_block": ("erase ms/block", False),
           "verify_MB_per_s": ("verify read MB/s", True), "ftl_sectors_per_s": ("FTL sectors/s", True),
           "ftl_KiB_per_s": ("FTL KiB/s", True),
           "save_MB_per_s": ("save read MB/s", True), "rst_erase_ms_per_block": ("restore erase ms/block", False),
           "rst_prog_MB_per_s": ("restore program MB/s", True), "rst_verify_MB_per_s": ("restore read MB/s", True),
           "ftl_save_MB_per_s": ("FTL read MB/s", True), "ftl_write_MB_per_s": ("FTL write MB/s", True),
           "ftl_write_sync_MB_per_s": ("FTL write+sync MB/s", True), "ftl_sync_ms": ("FTL sync ms", False),
           "ftl_verify_MB_per_s": ("FTL read-back MB/s", True),
           "ftl_rst_write_MB_per_s": ("FTL restore write MB/s", True),
           "ftl_rst_write_sync_MB_per_s": ("FTL restore write+sync MB/s", True),
           "ftl_rst_sync_ms": ("FTL restore sync ms", False), "ftl_rst_verify_MB_per_s": ("FTL restore read MB/s", True)}
WORKLOAD_KEYS = ("op", "bank_mask", "first_block", "nblocks", "pages", "batch", "repeats", "flags", "first_sector", "sectors")
for _ph in ("save", "erase", "program", "verify", "rst_erase", "rst_program", "rst_verify", "sync", "write",
            "rst_sync", "rst_write", "all"):
    METRICS["cpu_free_%s_pct" % _ph] = ("CPU free %% during %s" % _ph.replace("_", " "), True)
COUNT_KEYS = ("ecc_failed", "read_errors", "erase_fail", "prog_fail", "verify_mismatch", "verify_errors", "ftl_errors", "status",
              "cyc_failed_phase", "rst_mismatch", "rst_errors", "ftl_vfy_mismatch", "ftl_rst_mismatch")


def compare(a, b):
    """Compare two reports. Returns {'workload_diff', 'metrics', 'errors', 'comparable'}.

    'speedup' of a metric is how much better b is than a (>1: b better, whichever direction is good)."""
    wd = {k: (a["params"].get(k), b["params"].get(k)) for k in WORKLOAD_KEYS if a["params"].get(k) != b["params"].get(k)}
    metrics = {}
    for key, (label, higher) in METRICS.items():
        va, vb = a["derived"].get(key), b["derived"].get(key)
        if va is None or vb is None:
            continue
        if va == 0 or vb == 0:
            speed = None
        else:
            speed = vb / va if higher else va / vb
        metrics[key] = {"label": label, "a": va, "b": vb, "speedup": speed, "higher_is_better": higher}
    errors = {k: (a["result"].get(k, 0), b["result"].get(k, 0)) for k in COUNT_KEYS
              if a["result"].get(k, 0) or b["result"].get(k, 0)}
    return {"a": "%s/%s" % (a["backend"], a["op"]), "b": "%s/%s" % (b["backend"], b["op"]),
            "workload_diff": wd, "metrics": metrics, "errors": errors,
            "comparable": not wd and bool(metrics)}


def format_compare(c):
    L = ["A = %s   B = %s" % (c["a"], c["b"])]
    if c["workload_diff"]:
        L.append("WARNING: workloads differ (A vs B): " + ", ".join(
            "%s %s vs %s" % (k, x, y) for k, (x, y) in sorted(c["workload_diff"].items())))
    if not c["metrics"]:
        L.append("no common metrics")
    for key, m in c["metrics"].items():
        sp = "n/a" if m["speedup"] is None else "%.2fx" % m["speedup"]
        L.append("  %-20s A %12.3f   B %12.3f   B vs A: %s" % (m["label"], m["a"], m["b"], sp))
    for k, (x, y) in c["errors"].items():
        L.append("  error counter %-16s A %d   B %d" % (k, x, y))
    return "\n".join(L)


def format_report(rep):
    r, p, d = rep["result"], rep["params"], rep["derived"]
    L = ["backend %s op %s banks %s blocks %d+%d pages %d batch %d repeats %d flags %s" % (
        rep["backend"], rep["op"], p["banks"], p["first_block"], p["nblocks"], p["pages"], p["batch"], p["repeats"],
        ",".join(p["flag_names"]) or "-"),
        "status %s  repeats done %d  batches %d  total %d us  page %d B" % (
            "ok" if not r["status"] else "0x%08x" % r["first_err_rc"], r["repeats_done"], r["batches"], r["total_us"],
            r["page_size"])]
    if r["read_pages"]:
        L.append("read: %d pages in %d us; ecc clean %d corrected %d failed %d erased %d, max bits %d; errors %d; "
                 "batch us min %d max %d" % (r["read_pages"], r["read_us"], r["ecc_clean"], r["ecc_corrected"],
                                             r["ecc_failed"], r["erased_pages"], r["max_corrected"], r["read_errors"],
                                             r["batch_us_min"], r["batch_us_max"]))
    if r["erase_blocks"] or r["erase_fail"]:
        L.append("erase: %d blocks in %d us, failures %d" % (r["erase_blocks"], r["erase_us"], r["erase_fail"]))
    if (r["prog_pages"] or r["prog_fail"]) and rep["op"] != "ftl-cycle":
        L.append("program: %d pages in %d us, failures %d" % (r["prog_pages"], r["prog_us"], r["prog_fail"]))
    if r["verify_pages"]:
        L.append("verify: %d pages in %d us, mismatching pages %d, errors %d" % (
            r["verify_pages"], r["verify_us"], r["verify_mismatch"], r["verify_errors"]))
    if rep["op"] == "ftl-cycle":
        L.append("ftl cycle: %s; logical sectors %d..%d = %.1f MiB" % (
            CYC_PHASES.get(r["cyc_failed_phase"], r["cyc_failed_phase"]).replace("program", "write").replace(
                "pattern erase", "pattern sync").replace("restore erase", "restore sync"),
            p["first_sector"], p["first_sector"] + p["sectors"] - 1, p["sectors"] * nand.SECTOR_BYTES / 1048576.0))
        L.append("  save read %d sectors in %d us; pattern write %d in %d us + sync %d us; read back %d us, %d differ" % (
            r["ftl_sectors"], r["ftl_us"], r["prog_pages"], r["ftl_wr_us"], r["ftl_sync_us"], r["ftl_vfy_us"],
            r["ftl_vfy_mismatch"]))
        L.append("  restore write %d in %d us + sync %d us; read back %d us, %d differ; errors %d" % (
            r["rst_prog_pages"], r["ftl_rst_wr_us"], r["ftl_rst_sync_us"], r["ftl_rst_vfy_us"], r["ftl_rst_mismatch"],
            r["rst_errors"]))
    if rep["op"] == "cycle":
        L.append("cycle: %s; region %d pages = %.1f MiB (%d were erased and stay erased)" % (
            CYC_PHASES.get(r["cyc_failed_phase"], r["cyc_failed_phase"]), r["entries"],
            r["entries"] * r["page_size"] / 1048576.0, r["save_erased"]))
        L.append("  1 save read:   %d pages in %d us" % (r["save_pages"], r["save_us"]))
        L.append("  restore:       erase %d blocks in %d us; program %d pages in %d us; read back %d pages in %d us; "
                 "mismatching pages %d, errors %d" % (r["rst_erase_blocks"], r["rst_erase_us"], r["rst_prog_pages"],
                                                     r["rst_prog_us"], r["rst_verify_pages"], r["rst_verify_us"],
                                                     r["rst_mismatch"], r["rst_errors"]))
    if (r["ftl_sectors"] or r["ftl_errors"]) and rep["op"] != "ftl-cycle":
        L.append("ftl: %d sectors in %d us, errors %d" % (r["ftl_sectors"], r["ftl_us"], r["ftl_errors"]))
    if r["status"]:
        L.append("first error: rc 0x%08x (%s) bank %d page %d" % (
            r["first_err_rc"], nand.LIB_RC_NAMES.get(r["first_err_rc"], "?"), r["first_err_bank"], r["first_err_page"]))
    cpu = rep.get("cpu")
    if cpu:
        names = PHASES.get(rep["op"], {0: "all"})
        L.append("cpu: fmss poll sleep %d us; %s; largest correction per phase: %s" % (
            cpu["fmss_poll_us"],
            ("load thread %d counts/ms alone; CPU free: " % cpu["load_rate_per_ms"] + ", ".join(
                "%s %.0f%%" % (n, v) for n, v in cpu_free(cpu, rep["op"]).items())) if cpu["load"] else "no load thread",
            ", ".join("%s %d" % (n, cpu["phase_maxcorr"][i]) for i, n in names.items() if cpu["phase_us"][i])))
    L.append("derived: " + ", ".join("%s %.3f" % (k, v) for k, v in d.items() if not k.startswith("cpu_free")))
    return "\n".join(L)


# ------------------------------------------------------------------------------------------- run on the device
def run_bench(s, params_raw, timeout=600.0, progress=True):
    """Write the parameter block, run the job, return the report dict."""
    st = s.status()
    if st.busy:
        raise ToolError("monitor busy")
    s.write_memory(st.result_addr, params_raw)
    done = s.job(Cmd.NAND_BENCH, timeout=timeout, progress=progress)
    res = unpack_result(s.read(done.result_addr + RES_OFF, RESULT_BYTES))
    reps = unpack_reps(s.read(done.result_addr + REP_OFF, 4 * min(res["reps_recorded"], MAXREP)), res["reps_recorded"]) \
        if res["reps_recorded"] else []
    cpu = unpack_cpu(s.read(done.result_addr + CPU_OFF, struct.calcsize(_CFMT)))
    rep = make_report(params_raw, res, reps, cpu=cpu)
    rep["job"] = {"job_result": nand.RESULT_NAMES.get(done.job_result, done.job_result), "lib_rc": done.job_lib_rc,
                  "job_us": (done.job_t_end - done.job_t_start) & 0xffffffff}
    return rep, done


def ensure_ready(s, backend, need_ftl, auto_init):
    st = s.status()
    if not st.step_ok_mask & (1 << 1):
        if not auto_init:
            raise ToolError("clock gates not open: run 'n3g.py nand init' (or pass --init)")
        nand.check(s.job(Cmd.NAND_STEP, 1, 0))
    if need_ftl and not st.ftl_sectors and not auto_init:
        raise ToolError("FTL not mounted: run 'n3g.py nand init --ftl' (or pass --init)")


def _common(p):
    p.add_argument("--backend", choices=tuple(BACKENDS), default=None, help="physical backend for this run (default: active)")
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--batch", type=int, default=1)
    p.add_argument("--seed", type=nand.ai, default=0x1234abcd)
    p.add_argument("--json", help="write the report to this file")
    p.add_argument("--init", action="store_true", help="open the clock gates first if needed")
    p.add_argument("--run-timeout", type=float, default=600.0)


def _raw_args(p):
    p.add_argument("--banks", default="0", help="e.g. 0,1 or 0-3")
    p.add_argument("--first-block", type=nand.ai, default=0)
    p.add_argument("--blocks", type=int, default=1, help="blocks per bank")
    p.add_argument("--pages", type=int, default=1, help="pages per block")
    p.add_argument("--interleave", action="store_true", help="page-major across the banks (overlaps NAND page loads)")


def build_parser():
    ap = argparse.ArgumentParser(prog="n3g.py bench", description=__doc__.split("\n")[0])
    ap.add_argument("--settle", type=float, default=0.05)
    ap.add_argument("--poll", type=float, default=0.2)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("read", help="raw page reads (read-only)")
    _common(p); _raw_args(p)
    p.add_argument("--scatter", action="store_true", help="shuffle the page order")
    p = sub.add_parser("write", help="DESTRUCTIVE: erase + program scratch blocks (needs --destructive)")
    _common(p)
    p.add_argument("--banks", default="0")
    p.add_argument("--scratch-blocks", required=True, metavar="FIRST:COUNT",
                   help="physical blocks per bank to erase and program; must be blocks nothing else uses")
    p.add_argument("--pages", type=int, default=1)
    p.add_argument("--interleave", action="store_true")
    p.add_argument("--destructive", action="store_true", help="explicit acknowledgement that the scratch blocks are erased")
    p.add_argument("--two-plane", action="store_true")
    p.add_argument("--verify", action="store_true", help="read back and compare after programming")
    p.add_argument("--erase-after", action="store_true")
    p.add_argument("--unvalidated", action="store_true", help="allow a chip table row not validated on hardware")
    p = sub.add_parser("ftl", help="FTL sector reads (read-only)")
    _common(p)
    p.add_argument("--first-sector", type=nand.ai, default=0)
    p.add_argument("--sectors", type=int, default=1024, help="window size per repeat (2048-byte sectors)")
    p.add_argument("--scatter", action="store_true", help="random start per call")
    p = sub.add_parser("cycle", help="DESTRUCTIVE but restoring: per backend, read a region to SDRAM, write and check a "
                                      "pattern, write the saved pages back and check them; then compare the backends")
    p.add_argument("--backends", default="sw,fmss", help="backends to run, comma separated (default: both, sw,fmss)")
    p.add_argument("--mib", type=float, default=16.0, help="physical region size in MiB (default 16)")
    p.add_argument("--ftl-mib", type=float, default=16.0,
                   help="logical (FTL) region in MiB, written and restored through the FTL after the physical cycle "
                        "of each backend (default 16; 0 = skip). Mounts the FTL writable for the run.")
    p.add_argument("--first-sector", type=nand.ai, default=None, help="first FTL sector (default: random each run)")
    p.add_argument("--timing", default=None, metavar="S,H[,E]",
                   help="NAND bus timing for every run: setup, hold [, EDO 0/1] (default 4,3,0); pick it with 'bench sweep'")
    p.add_argument("--seed-run", type=int, default=None,
                   help="seed for the random regions and the backend order of each test (printed; default random)")
    p.add_argument("--load", choices=("off", "on", "both"), default="both",
                   help="CPU-bound thread beside the benchmark: off = raw speed only, on = speed under load + CPU "
                        "left for other work, both = each cycle twice (default)")
    p.add_argument("--fmss-poll-us", type=int, default=100,
                   help="fmss backend: sleep between polls of a running program, 0 = spin like sw (default 100)")
    p.add_argument("--banks", default="all", help="banks (default: all chip enables)")
    p.add_argument("--first-block", type=nand.ai, default=None,
                   help="first physical block per bank (default: random each run, in the user area, only blocks "
                        "holding user data or erased pages)")
    p.add_argument("--batch", type=int, default=MAXBATCH, help="pages per backend call (default %d)" % MAXBATCH)
    p.add_argument("--seed", type=nand.ai, default=0x1234abcd)
    p.add_argument("--unvalidated", action="store_true", help="allow a chip table row not validated on hardware")
    p.add_argument("--json", help="write all reports and the comparison to this file")
    p.add_argument("--run-timeout", type=float, default=1800.0)
    p = sub.add_parser("all", help="everything in one command: seqread + sweep (read-only), then cycle (writes and "
                                    "restores); one JSON per stage")
    p.add_argument("--out", default=None, help="JSON prefix (default bench-YYYYmmdd-HHMMSS)")
    p.add_argument("--backends", default="sw,fmss")
    p.add_argument("--seq-mib", type=float, default=32.0, help="seqread physical MiB (default 32)")
    p.add_argument("--mib", type=float, default=16.0, help="cycle physical MiB (default 16)")
    p.add_argument("--ftl-mib", type=float, default=16.0, help="FTL MiB for seqread and cycle (0 = skip FTL tests)")
    p.add_argument("--load", choices=("off", "on", "both"), default="both")
    p.add_argument("--unvalidated", action="store_true", help="cycle: allow a chip table row not validated on hardware")
    p.add_argument("--timing", default=None, metavar="S,H[,E]", help="seqread and cycle: bus timing (default: the safe default; the sweep always tries its grid)")
    p.add_argument("--skip", default="", help="stages to leave out, e.g. sweep or cycle (comma separated)")
    p.add_argument("--keep-going", action="store_true", help="run the remaining stages after one fails")
    p.add_argument("--seed-run", type=int, default=None)
    p = sub.add_parser("seqread", help="read-only: sequential read speed per backend, physical and through the FTL, "
                                        "unloaded and under CPU load, shuffled order, data compared across backends")
    p.add_argument("--backends", default="sw,fmss")
    p.add_argument("--mib", type=float, default=32.0, help="physical MiB (default 32, the most one run holds)")
    p.add_argument("--ftl-mib", type=float, default=32.0, help="FTL MiB (0 = skip the FTL test)")
    p.add_argument("--first-block", type=nand.ai, default=None, help="default: a random plain region")
    p.add_argument("--first-sector", type=nand.ai, default=None, help="default: random")
    p.add_argument("--batch", type=int, default=MAXBATCH, help="pages per backend call (default %d)" % MAXBATCH)
    p.add_argument("--ftl-batch", type=int, default=MAXFTLBATCH, help="sectors per FTL call (default %d)" % MAXFTLBATCH)
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--load", choices=("off", "on", "both"), default="both")
    p.add_argument("--fmss-poll-us", type=int, default=100, help="FMSS wait sleep in us (as cycle)")
    p.add_argument("--timing", default=None, metavar="S,H[,E]")
    p.add_argument("--seed-run", type=int, default=None)
    p.add_argument("--json")
    p.add_argument("--run-timeout", type=float, default=600.0)
    p = sub.add_parser("sweep", help="read-only: find the fastest NAND bus timing per backend whose data is identical "
                                      "to the default timing's (signature over every page + metadata)")
    p.add_argument("--backends", default="sw,fmss")
    p.add_argument("--blocks", type=int, default=2, help="blocks per bank (default 2: 4 MiB with 4 KiB pages on 4 banks)")
    p.add_argument("--first-block", type=nand.ai, default=None, help="default: a random plain region")
    p.add_argument("--setup", type=lambda t: [int(x) for x in t.split(",")], default=[0, 1, 2, 3, 4])
    p.add_argument("--hold", type=lambda t: [int(x) for x in t.split(",")], default=[0, 1, 2, 3])
    p.add_argument("--edo", type=lambda t: [int(x) for x in t.split(",")], default=[0, 1])
    p.add_argument("--fmss-poll-us", type=int, default=100)
    p.add_argument("--seed-run", type=int, default=None)
    p.add_argument("--json")
    p.add_argument("--run-timeout", type=float, default=300.0)
    p = sub.add_parser("table", help="detailed comparison tables (markdown) of a 'bench all' run")
    p.add_argument("prefix", help="the run's JSON prefix, e.g. bench-20261028-191756")
    p.add_argument("--backends", default="sw,fmss", help="column order")
    p.add_argument("--out", help="also write the tables to this .md file")
    p = sub.add_parser("compare", help="compare two --json reports")
    p.add_argument("a"); p.add_argument("b")
    p.add_argument("--json", help="write the comparison to this file")
    return ap


def parse_scratch(text):
    parts = text.replace(",", ":").split(":")
    if len(parts) != 2:
        raise ToolError("--scratch-blocks wants FIRST:COUNT")
    return int(parts[0], 0), int(parts[1], 0)


def params_from_args(a):
    if a.cmd == "read":
        flags = (F_INTERLEAVE if a.interleave else 0) | (F_SCATTER if a.scatter else 0)
        return make_params("read", a.backend, flags, parse_banks(a.banks), a.first_block, a.blocks, a.pages,
                           a.batch, a.repeats, a.seed)
    if a.cmd == "write":
        if not a.destructive:
            raise ToolError("write benchmark erases %s: pass --destructive to confirm these are scratch blocks" %
                            a.scratch_blocks)
        first, count = parse_scratch(a.scratch_blocks)
        flags = F_WRITE | (F_INTERLEAVE if a.interleave else 0) | (F_TWOPLANE if a.two_plane else 0) | \
            (F_VERIFY if a.verify else 0) | (F_ERASE_AFTER if a.erase_after else 0) | (F_UNVALIDATED if a.unvalidated else 0)
        return make_params("write", a.backend, flags, parse_banks(a.banks), first, count, a.pages, a.batch,
                           a.repeats, a.seed)
    return make_params("ftl", a.backend, F_SCATTER if a.scatter else 0, 1, 0, 1, 1, a.batch, a.repeats, a.seed,
                       a.first_sector, a.sectors)


def timing_word(setup, hold, edo=0):
    """FMCTRL0 bus timing word: (setup << 16) | (hold << 12) | EDO << 28 (the default is setup 4, hold 3)"""
    if not (0 <= setup <= 7 and 0 <= hold <= 7):
        raise ToolError("setup and hold are 0..7")
    # bit 31 marks an explicit timing (the device masks it off), so setup 0 / hold 0 / no EDO is not read as 0 = default
    return (1 << 31) | (setup << 16) | (hold << 12) | ((1 << 28) if edo else 0)


def parse_timing(text):
    """'S,H' or 'S,H,E' -> timing word; None/'' -> 0 (default)"""
    if not text:
        return 0
    f = [int(x, 0) for x in text.split(",")]
    return timing_word(f[0], f[1], f[2] if len(f) > 2 else 0)


def run_sweep(s, a):
    """Read-only: the fastest bus timing per backend whose reads are byte-identical to the default timing's"""
    names = [b.strip() for b in a.backends.split(",") if b.strip()]
    st = s.status()
    if not st.step_ok_mask & (1 << 1):
        nand.check(s.job(Cmd.NAND_STEP, 1, 0))
    if not s.status().step_ok_mask & (1 << 3):
        nand.check(s.job(Cmd.NAND_STEP, 3, 0, timeout=30.0))
    geom = list(s.status().geom)
    seed = a.seed_run if a.seed_run is not None else int.from_bytes(os.urandom(4), "little")
    rng = random.Random(seed)
    banks, ppb = geom[0], geom[2]
    first = a.first_block if a.first_block is not None else pick_physical(s, geom, a.blocks, rng)
    mask = (1 << banks) - 1
    print("sweep seed %d; region banks %s blocks %d..%d (%d pages, %.1f MiB), read-only" % (
        seed, mask_banks(mask), first, first + a.blocks - 1, banks * a.blocks * ppb,
        banks * a.blocks * ppb * geom[7] / 1048576.0))

    def read(name, word):
        raw = make_params("read", name, F_INTERLEAVE, mask, first, a.blocks, ppb, MAXBATCH, 1, 0x1234abcd,
                          fmss_poll_us=a.fmss_poll_us, timing=word)
        rep_, done = run_bench(s, raw, timeout=a.run_timeout, progress=False)
        r, c = rep_["result"], rep_["cpu"] or {}
        return {"ok_job": done.job_result == 0, "mbs": rep_["derived"].get("read_MB_per_s", 0.0),
                "sig": c.get("data_sig"), "pages": c.get("sig_pages"), "ecc_fail": c.get("ecc_fail_pages", 0),
                "errors": r["read_errors"], "maxcorr": r["max_corrected"], "corrected": r["ecc_corrected"]}

    ref = read(names[0], 0)
    if not ref["ok_job"] or ref["errors"] or ref["ecc_fail"]:
        raise ToolError("the reference read at the default timing failed: %r" % (ref,))
    print("reference (%s, default timing setup 4 hold 3): %.2f MB/s, signature %08x over %d pages" % (
        names[0], ref["mbs"], ref["sig"], ref["pages"]))
    grid = [(su, ho, e) for e in a.edo for su in a.setup for ho in a.hold]
    results = {}
    for name in names:
        print("\n=== %s" % name)
        best = None
        for su, ho, e in grid:
            word = timing_word(su, ho, e)
            r = read(name, word)
            good = r["ok_job"] and not r["errors"] and not r["ecc_fail"] and r["sig"] == ref["sig"]
            r.update(setup=su, hold=ho, edo=e, word=word, good=good)
            results.setdefault(name, []).append(r)
            print("  setup %d hold %d edo %d: %6.2f MB/s  %s  (max correction %d bits, %d corrected pages)" % (
                su, ho, e, r["mbs"], "OK" if good else "BAD (%s)" % ", ".join(
                    x for x, f in (("job failed", not r["ok_job"]), ("read errors", r["errors"]),
                                   ("uncorrectable", r["ecc_fail"]), ("data differs", r["sig"] != ref["sig"])) if f),
                r["maxcorr"], r["corrected"]))
            if good and (best is None or r["mbs"] > best["mbs"]):
                best = r
        if best:
            print("  fastest good: setup %d hold %d edo %d = %.2f MB/s (%.2fx the default timing)" % (
                best["setup"], best["hold"], best["edo"], best["mbs"],
                best["mbs"] / max(1e-9, next((x["mbs"] for x in results[name] if x["word"] == timing_word(4, 3)), ref["mbs"]))))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"tool": "n3g-bench-sweep", "seed": seed, "first_block": first, "blocks": a.blocks,
                       "reference": ref, "results": results}, f, indent=1)
        print("wrote %s" % a.json)
    return 0


def run_seqread(s, a):
    """Read-only: sequential reads per backend, physical (page-major across all banks) and through the FTL, each test
    in its own shuffled backend order, unloaded and/or under CPU load; checks every backend read the same data."""
    names = [b.strip() for b in a.backends.split(",") if b.strip()]
    for b in names:
        if b not in BACKENDS:
            raise ToolError("unknown backend %r" % b)
    if not s.status().step_ok_mask & (1 << 1):
        nand.check(s.job(Cmd.NAND_STEP, 1, 0))
    if not s.status().step_ok_mask & (1 << 3):
        nand.check(s.job(Cmd.NAND_STEP, 3, 0, timeout=30.0))
    geom = list(s.status().geom)
    banks, ppb, psize = geom[0], geom[2], geom[7]
    seed = a.seed_run if a.seed_run is not None else int.from_bytes(os.urandom(4), "little")
    rng = random.Random(seed)
    print("run seed %d (repeat this run's regions and order with --seed-run %d); read-only" % (seed, seed))
    mask = (1 << banks) - 1
    nblocks = first = 0
    if a.mib > 0:
        nblocks = max(1, min(int(a.mib * 1048576) // (banks * ppb * psize), MAXENTRIES // (banks * ppb)))
        first = a.first_block if a.first_block is not None else pick_physical(s, geom, nblocks, rng)
        print("physical: banks %s, blocks %d..%d, %d pages of %d B = %.1f MiB, page-major across the banks, batch %d" % (
            mask_banks(mask), first, first + nblocks - 1, banks * nblocks * ppb, psize,
            banks * nblocks * ppb * psize / 1048576.0, a.batch))
    ftl_first = ftl_count = 0
    if a.ftl_mib > 0:
        st = s.status()
        if not st.ftl_sectors:
            nand.check(s.job(Cmd.NAND_STEP, 4, 0, timeout=600.0, progress=True))   # read-only mount, to size the FTL
            st = s.status()
        ftl_count = int(a.ftl_mib * 1048576) // nand.SECTOR_BYTES
        ftl_first = a.first_sector if a.first_sector is not None else \
            rng.randrange(0, max(1, (st.ftl_sectors - ftl_count) // 128 + 1)) * 128
        if ftl_count < 1 or ftl_first + ftl_count > st.ftl_sectors:
            raise ToolError("FTL sectors %d..%d outside the FTL's %d" % (ftl_first, ftl_first + ftl_count - 1,
                                                                        st.ftl_sectors))
        print("FTL: logical sectors %d..%d of %d (%.1f MiB), %d sectors per call" % (
            ftl_first, ftl_first + ftl_count - 1, st.ftl_sectors, ftl_count * nand.SECTOR_BYTES / 1048576.0,
            a.ftl_batch))
    loads = {"off": [False], "on": [True], "both": [False, True]}[a.load]
    kinds = (["read"] if nblocks else []) + (["ftl"] if ftl_count else [])
    rows, allreps, sigs, bad = [], [], {}, []
    for loaded in loads:
        for kind in kinds:
            order = names[:]
            rng.shuffle(order)
            print("\n##### sequential %s%s: order %s" % ("physical" if kind == "read" else "FTL",
                                                        " under load" if loaded else "", ", ".join(order)))
            for name in order:
                lf = F_LOAD if loaded else 0
                if kind == "read":
                    raw = make_params("read", name, lf | F_INTERLEAVE, mask, first, nblocks, ppb, a.batch, a.repeats,
                                      0x1234abcd, fmss_poll_us=a.fmss_poll_us, timing=parse_timing(a.timing))
                else:
                    st = s.status()
                    if st.backend == BACKENDS[name] and not st.step_ok_mask & (1 << 4):
                        # older firmware mounts the FTL for a bench run only on a backend switch: mount it here
                        nand.check(s.job(Cmd.NAND_STEP, 4, 0, timeout=600.0))
                    raw = make_params("ftl", name, lf, 1, 0, 1, 1, a.ftl_batch, a.repeats, 0x1234abcd, ftl_first,
                                      ftl_count, fmss_poll_us=a.fmss_poll_us, timing=parse_timing(a.timing))
                rep_, done = run_bench(s, raw, timeout=a.run_timeout, progress=False)
                rep_["loaded"], rep_["order"] = loaded, order
                allreps.append(rep_)
                r, d, c = rep_["result"], rep_["derived"], rep_["cpu"] or {}
                if done.job_result != 0 or r["read_errors"] or r["status"]:
                    bad.append("%s %s%s: job result %d, %d read errors, status %#x" % (
                        name, kind, " loaded" if loaded else "", done.job_result, r["read_errors"], r["status"]))
                if kind == "read":
                    mbs = d.get("read_MB_per_s", 0.0)
                    if c.get("sig_pages"):
                        sigs.setdefault(c["sig_pages"], {})[(name, loaded)] = c["data_sig"]
                else:
                    mbs = d.get("ftl_KiB_per_s", 0.0) * 1024 / 1e6
                free = cpu_free(c, rep_["params"]["op"]).get("all") if loaded else None
                rows.append({"kind": kind, "backend": name, "loaded": loaded, "MB_per_s": mbs, "cpu_free_pct": free,
                             "us_per_unit": d.get("read_us_per_page") if kind == "read" else d.get("ftl_us_per_sector"),
                             "ecc_corrected": r["ecc_corrected"], "max_corrected": r["max_corrected"]})
                print("  %-6s %7.2f MB/s  %7.1f us/%s%s" % (
                    name, mbs, rows[-1]["us_per_unit"] or 0, "page" if kind == "read" else "sector",
                    "" if free is None else "   CPU left for other work %.0f %%" % free))
    same = all(len(set(v.values())) == 1 for v in sigs.values())
    print("\n=== sequential read, MB/s (10^6 B/s)%s" % ("; under load: rate (CPU left for other work)" if True in loads else ""))
    hdr = "%-26s" % "" + "".join("%18s" % n for n in names)
    print(hdr)
    for loaded in loads:
        for kind in kinds:
            line = "%-26s" % ("%s%s" % ("physical (page-major)" if kind == "read" else "FTL (logical sectors)",
                                       ", loaded" if loaded else ""))
            base = None
            for n in names:
                row = next(x for x in rows if (x["kind"], x["backend"], x["loaded"]) == (kind, n, loaded))
                base = base or row["MB_per_s"]
                cell = "%.2f" % row["MB_per_s"]
                if row["cpu_free_pct"] is not None:
                    cell += " (%.0f%%)" % row["cpu_free_pct"]
                if n != names[0] and base:
                    cell += " %+.0f%%" % (100.0 * (row["MB_per_s"] / base - 1))
                line += "%18s" % cell
            print(line)
    print("(the %% after a figure is relative to %s)" % names[0])
    if sigs:
        print("data: %s" % ("identical across backends (signature over every page + metadata)" if same else
                            "DIFFERS between backends: %r" % (sigs,)))
    for b_ in bad:
        print("ERROR: %s" % b_)
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"tool": "n3g-bench-seqread", "version": 1, "seed": seed, "rows": rows, "reports": allreps,
                       "data_identical": same, "errors": bad,
                       "regions": {"physical_first_block": first, "physical_blocks": nblocks,
                                   "ftl_first_sector": ftl_first, "ftl_sectors": ftl_count}}, f, indent=1)
        print("wrote %s" % a.json)
    return 0 if same and not bad else 2


def run_all(s, a):
    """The whole benchmark in one go: seqread and sweep (read-only) first, then cycle (writes, restores).
    Each stage writes PREFIX-<stage>.json; a stage that fails stops the run unless --keep-going."""
    stages = [x for x in ("seqread", "sweep", "cycle") if x not in {t.strip() for t in a.skip.split(",")}]
    common = ["--backends", a.backends] + (["--seed-run", str(a.seed_run)] if a.seed_run is not None else [])
    argv = {
        "seqread": ["seqread", "--mib", str(a.seq_mib), "--ftl-mib", str(a.ftl_mib), "--load", a.load]
                   + (["--timing", a.timing] if a.timing else []),
        "sweep": ["sweep"],
        "cycle": ["cycle", "--mib", str(a.mib), "--ftl-mib", str(a.ftl_mib), "--load", a.load]
                 + (["--unvalidated"] if a.unvalidated else []) + (["--timing", a.timing] if a.timing else []),
    }
    run = {"seqread": run_seqread, "sweep": run_sweep, "cycle": run_cycle}
    rcs = {}
    for st in stages:
        path = "%s-%s.json" % (a.out, st)
        sa = build_parser().parse_args(argv[st] + common + ["--json", path])
        print("\n" + "#" * 78 + "\n# bench %s\n" % " ".join(argv[st] + common + ["--json", path]) + "#" * 78)
        try:
            rcs[st] = run[st](s, sa)
        except ToolError as e:
            print("ERROR: %s: %s" % (st, e))
            rcs[st] = 1
        if rcs[st] and not a.keep_going:
            print("\n%s failed (rc %d); stopping (--keep-going runs the remaining stages anyway)" % (st, rcs[st]))
            break
    print("\n" + "=" * 78 + "\nsummary (%s-*.json)" % a.out)
    for st in stages:
        print("  %-8s %s" % (st, "not run" if st not in rcs else "ok" if rcs[st] == 0 else "FAILED (rc %d)" % rcs[st]))
    try:
        with open("%s-sweep.json" % a.out) as f:
            sw = json.load(f)
        for name, rows in sw["results"].items():
            good = [r for r in rows if r["good"]]
            if good:
                b = max(good, key=lambda r: r["mbs"])
                print("  sweep %-6s fastest good timing %d,%d,%d = %.2f MB/s (cycle --timing %d,%d,%d)" % (
                    name, b["setup"], b["hold"], b["edo"], b["mbs"], b["setup"], b["hold"], b["edo"]))
    except (OSError, ValueError, KeyError):
        pass
    try:
        run_table(argparse.Namespace(prefix=a.out, backends=a.backends, out="%s-table.md" % a.out))
    except ToolError:
        pass
    return max(rcs.values() or [0])


def _fmt(v, unit):
    if v is None:
        return "-"
    return "%.0f" % v if unit in ("us", "%", "us/page", "us/sector", "pages", "bits", "#") else "%.2f" % v


def _ratio(a, b, lower_better):
    """how many times better b is than a (>1 = b better)"""
    if not a or not b:
        return "-"
    return "%.2fx" % ((a / b) if lower_better else (b / a))


def table_rows(prefix):
    """(section, metric, unit, lower_better, {(backend, loaded): value}) from PREFIX-{seqread,cycle,sweep}.json"""
    rows = []

    def load(name):
        try:
            with open("%s-%s.json" % (prefix, name)) as f:
                return json.load(f)
        except OSError:
            return None
    sq = load("seqread")
    if sq:
        for r in sq["rows"]:
            kind = "seq read, physical" if r["kind"] == "read" else "seq read, FTL"
            u = "us/page" if r["kind"] == "read" else "us/sector"
            for metric, unit, lb, v in (("rate", "MB/s", False, r["MB_per_s"]), ("time per unit", u, True, r["us_per_unit"]),
                                        ("CPU left for other work", "%", False, r["cpu_free_pct"])):
                rows.append((kind, metric, unit, lb, r["backend"], r["loaded"], v))
    cy = load("cycle")
    if cy:
        for rep_ in cy["reports"]:
            d, res, name, loaded = rep_["derived"], rep_["result"], rep_["backend"], rep_.get("loaded", False)
            page = res.get("page_size") or 4096
            if rep_["params"]["op"] == OPS["cycle"]:
                sec = "cycle, physical"
                items = [("save read", "MB/s", False, d.get("save_MB_per_s")),
                         ("save read", "us/page", True, page / d["save_MB_per_s"] if d.get("save_MB_per_s") else None),
                         ("erase", "ms/block", True, d.get("erase_ms_per_block")),
                         ("program", "MB/s", False, d.get("prog_MB_per_s")),
                         ("program", "us/page", True, d.get("prog_us_per_page")),
                         ("verify read", "MB/s", False, d.get("verify_MB_per_s")),
                         ("restore erase", "ms/block", True, d.get("rst_erase_ms_per_block")),
                         ("restore program", "MB/s", False, d.get("rst_prog_MB_per_s")),
                         ("restore read back", "MB/s", False, d.get("rst_verify_MB_per_s")),
                         ("ECC: corrected pages, all reads of the run", "pages", True, res.get("ecc_corrected")),
                         ("ECC: largest correction", "bits", True, res.get("max_corrected"))]
                phases = PHASES["cycle"].values()
            else:
                sec = "cycle, FTL (logical)"
                items = [("FTL read (save)", "MB/s", False, d.get("ftl_save_MB_per_s")),
                         ("FTL write", "MB/s", False, d.get("ftl_write_MB_per_s")),
                         ("FTL sync", "ms", True, d.get("ftl_sync_ms")),
                         ("FTL write + sync", "MB/s", False, d.get("ftl_write_sync_MB_per_s")),
                         ("FTL read back", "MB/s", False, d.get("ftl_verify_MB_per_s")),
                         ("restore write", "MB/s", False, d.get("ftl_rst_write_MB_per_s")),
                         ("restore sync", "ms", True, d.get("ftl_rst_sync_ms")),
                         ("restore write + sync", "MB/s", False, d.get("ftl_rst_write_sync_MB_per_s")),
                         ("restore read back", "MB/s", False, d.get("ftl_rst_verify_MB_per_s"))]
                phases = PHASES["ftl-cycle"].values()
            if rep_.get("order") and name in rep_["order"]:
                items.insert(0, ("run order in this test (1 = first: reads the region's older data)", "#", True,
                                 rep_["order"].index(name) + 1))
            for ph in phases:
                if "cpu_free_%s_pct" % ph in d:
                    items.append(("CPU left: " + ph, "%", False, d["cpu_free_%s_pct" % ph]))
            for metric, unit, lb, v in items:
                rows.append((sec, metric, unit, lb, name, loaded, v))
    sw = load("sweep")
    if sw:
        for name, rs in sw["results"].items():
            for r in rs:
                rows.append(("timing sweep, read (setup,hold,EDO)", "%d,%d,%d%s" % (
                    r["setup"], r["hold"], r["edo"], "" if r["good"] else " BAD"), "MB/s", False, name, False,
                    r["mbs"] if r["good"] else None))
    return rows


def run_table(a):
    rows = table_rows(a.prefix)
    if not rows:
        raise ToolError("no %s-{seqread,cycle,sweep}.json" % a.prefix)
    names = []
    for r in rows:
        if r[4] not in names:
            names.append(r[4])
    names = [n for n in a.backends.split(",") if n in names] or names
    pairs = [(names[0], n) for n in names[1:]] + ([(names[1], names[2])] if len(names) > 2 else [])
    out = []
    for loaded in (False, True):
        secs = []
        for r in rows:
            if r[5] == loaded and r[0] not in secs:
                secs.append(r[0])
        for sec in secs:
            out.append("\n### %s%s\n" % (sec, ", under CPU load" if loaded else ""))
            out.append("| metric | unit | " + " | ".join(names) + " | " +
                       " | ".join("%s vs %s" % (b, a_) for a_, b in pairs) + " |")
            out.append("|---|---|" + "---:|" * (len(names) + len(pairs)))
            seen = []
            for r in rows:
                if r[0] == sec and r[5] == loaded and (r[1], r[2]) not in seen:
                    seen.append((r[1], r[2]))
            for metric, unit in seen:
                vals = {r[4]: r[6] for r in rows if (r[0], r[1], r[2], r[5]) == (sec, metric, unit, loaded)}
                lb = next(r[3] for r in rows if (r[0], r[1], r[2]) == (sec, metric, unit))
                cmp_ = ["" if unit == "#" else
                        _ratio(vals.get(a_), vals.get(b), lb) if unit not in ("%", "pages", "bits") else
                        ("-" if vals.get(a_) is None or vals.get(b) is None else "%+.0f" % (vals[b] - vals[a_]))
                        for a_, b in pairs]
                out.append("| %s | %s | %s | %s |" % (metric, unit, " | ".join(_fmt(vals.get(n), unit) for n in names),
                                                      " | ".join(cmp_)))
    out.append("\n'A vs B' columns: how many times better B is than A (>1.00x = B better; for times, lower is better). "
               "For %, pages and bits: B minus A. MB/s = 10^6 bytes/s; us = microseconds.")
    text = "\n".join(out)
    print(text)
    if a.out:
        with open(a.out, "w") as f:
            f.write(text + "\n")
        print("wrote %s" % a.out)
    return 0


def cycle_region(geom, mib, banks_text, first_block):
    """Region of op CYCLE from the driver geometry: whole blocks, page-major across the banks."""
    nbanks, blocks, ppb, pagesize = geom[0], geom[1], geom[2], geom[7]
    if not nbanks or not ppb or not pagesize:
        raise ToolError("no driver geometry: the backend init (step 3) did not succeed")
    mask = (1 << nbanks) - 1 if banks_text == "all" else parse_banks(banks_text)
    nb = bin(mask).count("1")
    pages = int(mib * 1048576) // pagesize
    nblocks = max(1, -(-pages // (nb * ppb)))
    if nb * nblocks * ppb > MAXENTRIES:
        raise ToolError("%.1f MiB is %d pages, more than %d per run" % (mib, nb * nblocks * ppb, MAXENTRIES))
    if first_block is None:
        first_block = (blocks // 2) & ~1
    if first_block < 200 or first_block + nblocks > blocks:
        raise ToolError("blocks %d..%d are outside 200..%d (the first 200 blocks of a bank hold the VFL context)" % (
            first_block, first_block + nblocks - 1, blocks - 1))
    return mask, first_block, nblocks, ppb


def region_is_plain(s, first, nblocks, banks, ppb):
    """Read-only spare scan (META_SCAN) of the candidate blocks: only user data (types 0x40/0x41) and erased pages,
    no control/VFL/bad-block pages, no failed or timed-out reads"""
    done = nand.check(s.job(Cmd.NAND_META_SCAN, first, nblocks, timeout=120.0))
    raw = s.read(done.result_addr, nblocks * banks * ppb * 16)
    for i in range(0, len(raw), 16):
        kind = raw[i + 9]
        word = struct.unpack_from("<i", raw, i + 12)[0]
        if word < 0 or (word & 0xff) == 3 or kind not in (0x40, 0x41, 0xff):
            return False
    return True


def pick_physical(s, geom, nblocks, rng, tries=40):
    """Random even first block in [200, userblocks - nblocks] whose blocks hold only user data / erased pages"""
    banks, ppb, userblocks = geom[0], geom[2], geom[4] or geom[1]
    hi = min(geom[1], userblocks) - nblocks
    if hi < 200:
        raise ToolError("no room for %d blocks in the user area" % nblocks)
    for _ in range(tries):
        first = rng.randrange(200, hi + 1) & ~1
        if region_is_plain(s, first, nblocks, banks, ppb):
            return first
    raise ToolError("no plain region found in %d random tries" % tries)


def cycle_stop(name, kind, rep, done, a, geom):
    """Why the run must stop after this job (None to go on)"""
    r = rep["result"]
    what = "physical" if kind == "cycle" else "FTL"
    if done.job_result != 0 and not r["cyc_failed_phase"]:
        why = ""
        if rep["job"]["job_result"] == "NOT_ALLOWED" and not a.unvalidated and not geom[10]:
            why = ("; chip 0x%08x is not marked validated on hardware in the chip table, so writes are refused: "
                   "pass --unvalidated to run on it" % geom[11])
        return "%s %s: the job did not run (%s), nothing was written%s" % (name, what, rep["job"]["job_result"], why)
    if r["cyc_failed_phase"] == 1:
        return "%s %s: the save read failed (nothing was written)" % (name, what)
    lost = r["rst_mismatch"] if kind == "cycle" else r["ftl_rst_mismatch"]
    if r["cyc_failed_phase"] >= 5 or (r["cyc_failed_phase"] >= 2 and (lost or r["rst_errors"])):
        if lost or (kind == "cycle" and r["cyc_failed_phase"] == 5 and not r["rst_verify_pages"]):
            return ("%s %s: the restore did not complete (%s, %d differ, %d errors): the region no longer holds its "
                    "original data; stopping" % (name, what, CYC_PHASES[r["cyc_failed_phase"]], lost, r["rst_errors"]))
        return ("%s %s: the restore had %d errors (%s) but everything reads back as before; stopping before the next "
                "step" % (name, what, r["rst_errors"], CYC_PHASES[r["cyc_failed_phase"]]))
    return None


def run_cycle(s, a):
    """One command: init, then op CYCLE per backend on the same region, then the comparison."""
    names = [b.strip() for b in a.backends.split(",") if b.strip()]
    for b in names:
        if b not in BACKENDS:
            raise ToolError("unknown backend %r" % b)
    st = s.status()
    if not st.step_ok_mask & (1 << 1):
        nand.check(s.job(Cmd.NAND_STEP, 1, 0))
    if not s.status().step_ok_mask & (1 << 3):
        nand.check(s.job(Cmd.NAND_STEP, 3, 0, timeout=30.0))
    geom = list(s.status().geom)
    seed = a.seed_run if a.seed_run is not None else int.from_bytes(os.urandom(4), "little")
    rng = random.Random(seed)
    print("run seed %d (repeat this run's regions and order with --seed-run %d)" % (seed, seed))
    mask, first, nblocks, ppb = cycle_region(geom, a.mib, a.banks, a.first_block or 200)
    if a.first_block is None:
        first = pick_physical(s, geom, nblocks, rng)
    if not geom[10]:
        print("chip 0x%08x: not validated on hardware in the chip table%s" % (
            geom[11], " (--unvalidated given)" if a.unvalidated else "; the device refuses writes without --unvalidated"))
    print("region: banks %s, blocks %d..%d of %d per bank, %d pages of %d B = %.1f MiB; batch %d" % (
        mask_banks(mask), first, first + nblocks - 1, geom[1], bin(mask).count("1") * nblocks * ppb, geom[7],
        bin(mask).count("1") * nblocks * ppb * geom[7] / 1048576.0, a.batch))
    ftl_first = ftl_count = 0
    if a.ftl_mib > 0:
        st = s.status()
        if not st.ftl_sectors:
            nand.check(s.job(Cmd.NAND_STEP, 4, 0, timeout=600.0, progress=True))   # read-only, to size the FTL
            st = s.status()
        ftl_count = int(a.ftl_mib * 1048576) // nand.SECTOR_BYTES
        ftl_first = a.first_sector if a.first_sector is not None else \
            rng.randrange(0, max(1, (st.ftl_sectors - ftl_count) // 128 + 1)) * 128
        if ftl_count < 1 or ftl_first + ftl_count > st.ftl_sectors:
            raise ToolError("FTL sectors %d..%d outside the FTL's %d" % (ftl_first, ftl_first + ftl_count - 1,
                                                                        st.ftl_sectors))
        print("FTL region: logical sectors %d..%d of %d (%.1f MiB), written and restored through the FTL" % (
            ftl_first, ftl_first + ftl_count - 1, st.ftl_sectors, ftl_count * nand.SECTOR_BYTES / 1048576.0))
    flags = F_UNVALIDATED if a.unvalidated else 0
    loads = {"off": [False], "on": [True], "both": [False, True]}[a.load]
    groups = {}             # (kind, loaded) -> [report per backend]
    allreps = []
    tests = [(kind, loaded) for loaded in loads for kind in (["cycle"] + (["ftl-cycle"] if ftl_count else []))]
    byname = {}
    for kind, loaded in tests:
        order = names[:]
        rng.shuffle(order)
        print("\n##### %s%s: order %s" % (kind, " under load" if loaded else "", ", ".join(order)))
        for name in order:
            lf = flags | (F_LOAD if loaded else 0)
            if kind == "cycle":
                raw = make_params("cycle", name, lf, mask, first, nblocks, ppb, a.batch, 1, a.seed,
                                  fmss_poll_us=a.fmss_poll_us, timing=parse_timing(a.timing))
            else:
                raw = make_params("ftl-cycle", name, lf, 1, 0, 1, 1, MAXFTLBATCH, 1, a.seed, ftl_first, ftl_count,
                                  fmss_poll_us=a.fmss_poll_us, timing=parse_timing(a.timing))
            print("\n=== %s %s%s" % (name, "physical cycle" if kind == "cycle" else "FTL (logical) cycle",
                                      ", under CPU load" if loaded else ""))
            rep, done = run_bench(s, raw, timeout=a.run_timeout)
            rep["loaded"] = loaded
            rep["order"] = order
            print(format_report(rep))
            byname[(kind, loaded, name)] = rep
            allreps.append(rep)
            stop = cycle_stop(name, kind, rep, done, a, geom)
            if stop:
                if a.json:
                    with open(a.json, "w") as f:
                        json.dump({"tool": "n3g-bench-cycle", "version": 2, "seed": seed, "reports": allreps,
                                   "stopped": stop}, f, indent=1)
                    print("wrote %s" % a.json)
                raise ToolError(stop)
    groups = {(k, l): [byname[(k, l, n)] for n in names] for k, l in tests}
    out = {"tool": "n3g-bench-cycle", "version": 2, "seed": seed, "reports": allreps, "compare": {},
           "regions": {"physical_first_block": first, "physical_blocks": nblocks, "ftl_first_sector": ftl_first,
                       "ftl_sectors": ftl_count}}
    pairs = [(0, i) for i in range(1, len(names))] + ([(1, 2)] if len(names) > 2 else [])
    for (kind, loaded), reps in groups.items():
        for ia, ib in pairs:
            if ib >= len(reps):
                continue
            c = compare(reps[ia], reps[ib])
            key = "%s%s:%s-vs-%s" % (kind, "-loaded" if loaded else "", names[ia], names[ib])
            out["compare"][key] = c
            print("\n=== %s vs %s, %s%s" % (names[ia], names[ib],
                  "physical pages" if kind == "cycle" else "FTL (logical sectors, FTL overhead included)",
                  ", under CPU load (rates of the benchmark itself, and CPU left for the load thread)" if loaded else ""))
            print(format_compare(c))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
        print("wrote %s" % a.json)
    return 0 if all(not r["result"]["cyc_failed_phase"] for r in allreps) else 2


def load_report(path):
    with open(path) as f:
        d = json.load(f)
    if d.get("tool") != "n3g-bench":
        raise ToolError("%s is not an n3g-bench report" % path)
    return d


def main(argv=None):
    a = build_parser().parse_args(argv)
    s = None
    try:
        if a.cmd == "table":
            return run_table(a)
        if a.cmd == "compare":
            c = compare(load_report(a.a), load_report(a.b))
            print(format_compare(c))
            if a.json:
                with open(a.json, "w") as f:
                    json.dump(c, f, indent=1)
            return 0
        if a.cmd == "cycle":
            s = nand.make_dev_session(settle=a.settle, poll=a.poll)
            return run_cycle(s, a)
        if a.cmd == "all":
            a.out = a.out or time.strftime("bench-%Y%m%d-%H%M%S")
            s = nand.make_dev_session(settle=a.settle, poll=a.poll)
            return run_all(s, a)
        if a.cmd == "seqread":
            s = nand.make_dev_session(settle=a.settle, poll=a.poll)
            return run_seqread(s, a)
        if a.cmd == "sweep":
            s = nand.make_dev_session(settle=a.settle, poll=a.poll)
            return run_sweep(s, a)
        raw = params_from_args(a)
        s = nand.make_dev_session(settle=a.settle, poll=a.poll)
        ensure_ready(s, a.backend, a.cmd == "ftl", a.init)
        rep, done = run_bench(s, raw, timeout=a.run_timeout)
        print(format_report(rep))
        if a.json:
            with open(a.json, "w") as f:
                json.dump(rep, f, indent=1)
            print("wrote %s" % a.json)
        return 0 if done.job_result == 0 else 2
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()


if __name__ == "__main__":
    sys.exit(main())
