#!/usr/bin/env python3
"""n4g_nand.py -- host tool for the emCORE nandlib proof of concept (iPod nano 4G), 

Talks to target/ipodnano4g/nand_poc.c through the USB commands 120..126 (usb_cmds.py) and READ_MEM.
One command in flight: every job is queued, then the status record is polled until the job is done. The tool
stops at the first error and prints the status record. USB errors are treated as the host control-endpoint halt
(bundle rule): the device handle is reset (dev.reset()) and the session reconnected once.

  n4g_nand.py status                      decoded status record (+ monitor heartbeat)
  n4g_nand.py step N [--arg A]            run bring-up step N (1 gates, 2 pins, 3 PMU power, 4 FIL init, 5 FTL open)
  n4g_nand.py ids                         chip IDs + geometry (after step 4)
  n4g_nand.py read-reserved PAGE [-o F]   ROM-style read of bank 0 PAGE (0x600 bytes)
  n4g_nand.py dump-reserved -o F [--pages N]   pages 0..N (N from the IMG1 header in page 0, as the ROM does)
  n4g_nand.py read-page BANK PAGE [-o F]  FIL page read (4096 data + 12 meta)
  n4g_nand.py ftl-open                    step 5 with progress (FIL pages read) until done
  n4g_nand.py read-lpn LPN [COUNT] [-o F] FTL read (4 KiB logical pages)
  n4g_nand.py dump-disk -o F [--start S --count C]   chunked, resumable logical-disk dump
  n4g_nand.py mbr                         MBR + firmware partition ("STOP sign") directory
  n4g_nand.py get-struct ID [-o F] WMR GetStruct 
  n4g_nand.py ring                        platform ring buffer records (n4g_rec)
  n4g_nand.py watchdog SECONDS            NAND_WATCHDOG (stub: register semantics unknown)
  read-page ... --uncached                uncached-alias read variant (fallback: clean+invalidate after DMA)
  write path (rw build): write-info, snapshot -o DIR, open-rw, erase-block, program-page, write-lpn, flush,
  session-end (FTL wear level + flush(1), what the OS does at sync), bootarea-write, enable-storage-writes, boot-rw,
  free-vbs (FTL free-VB ring, read-only: candidates for the first erase/program test)

The parsers (parse_status, parse_img1_header, parse_mbr, parse_fw_directory, parse_ring) are pure functions.
"""
import argparse
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------------------------------- constants (nand_poc.h)
NPOC_MAGIC = 0x434F504E
NPOC_VERSION = 1
STATE_NAMES = {0: "IDLE", 1: "QUEUED", 2: "RUNNING", 3: "DONE"}
RESULT_NAMES = {0: "OK", 1: "LIB_ERROR", 2: "PRECOND", 3: "BAD_ARG", 4: "NO_MEMORY", 5: "PANIC", 6: "GATE (refused)"}
GATE_NAMES = {1: "no snapshot in this boot", 2: "prerequisite step not done", 3: "write stack not open (open-rw)",
              4: "bank 0 block 0 = ROM reserved region (needs --force)", 5: "block in use/unknown (needs --force)", 6: "out of range"}
IN_USE_WHY = {0: "free", 1: "FTL not open", 2: "VSVFL context/reserve area", 3: "outside FTL VB space", 4: "not in FTL free list",
              5: "remapped block", 6: "marked bad"}
ACCEPT_NAMES = {0: "OK", 1: "BUSY", 2: "BAD_ARG", 3: "NOT_INIT", 4: "UNKNOWN_HW (unknown register semantics)",
                5: "BAD_TOKEN", 6: "NO_WRITE (read-only build)"}
UC_MODES = {0: "cached (default path)", 1: "uncached alias SDRAM|0x80000000", 2: "no uncached alias in the bootstub map: cached buffer, clean+invalidate after DMA"}
JOB_NAMES = {0: "none", 1: "step", 2: "read_reserved", 3: "read_page", 4: "ftl_read", 5: "get_struct", 6: "ftl_read_ext",
             7: "snapshot", 8: "open_rw", 9: "erase_block", 10: "program_page", 11: "ftl_write", 12: "ftl_flush",
             13: "bootarea_write", 14: "storage_writes", 15: "ftl_write_ext"}
STEP_NAMES = {1: "clock gates 5/9", 2: "NAND pin mux", 3: "PMU NAND power", 4: "FIL init (chip detect)",
              5: "FTL open (WMR 0x70)"}
GEOM_NAMES = ("pages_per_block", "num_ce", "vendor", "blocks_per_ce", "bytes_per_page", "spare_per_page",
              "config", "refresh_threshold", "ecc_bits", "sector_bytes", "boot_page_bytes", "chipid")
LIB_RC_NAMES = {0: "ok", 1: "clean/erased", 0x80000001: "FAULT", 0x80000002: "UECC/ECC mark",
                0x80000003: "RANGE", 0x80000004: "PARAM", 0x80000006: "BUF", 0x80000007: "UNSUPPORTED"}
ROM_RC_NAMES = {0: "BLANK (erased sector seen)", 1: "OK", 3: "UNCORRECTABLE", 5: "TIMEOUT"}
CMD_STATUS, CMD_STEP, CMD_READ_RESERVED, CMD_READ_PAGE, CMD_FTL_OPEN, CMD_FTL_READ, CMD_GET_STRUCT, CMD_WATCHDOG = range(120, 128)
(CMD_WRITE_INFO, CMD_SNAPSHOT, CMD_OPEN_RW, CMD_ERASE_BLOCK, CMD_PROGRAM_PAGE, CMD_FTL_WRITE, CMD_FTL_FLUSH,
 CMD_BOOTAREA_WR, CMD_STORAGE_WR) = range(128, 137)
JOB_OPEN_RW, JOB_ERASE_BLOCK, JOB_PROGRAM_PAGE, JOB_FTL_WRITE, JOB_FTL_FLUSH, JOB_BOOTAREA_WR, JOB_STORAGE_WR = 8, 9, 10, 11, 12, 13, 14
CMD_TO_JOB = {CMD_OPEN_RW: JOB_OPEN_RW, CMD_ERASE_BLOCK: JOB_ERASE_BLOCK, CMD_PROGRAM_PAGE: JOB_PROGRAM_PAGE,
              CMD_FTL_WRITE: JOB_FTL_WRITE, CMD_FTL_FLUSH: JOB_FTL_FLUSH, CMD_BOOTAREA_WR: JOB_BOOTAREA_WR,
              CMD_STORAGE_WR: JOB_STORAGE_WR}
FORCE = 0x80000000
STAGE_PAGES = 16
TOKEN_SEED = 0x4E414E44
DESTRUCTIVE_FLAG = "--i-understand-this-is-destructive"
FLUSH_WEARLEVEL = 0x1        # NPOC_FLUSH_WEARLEVEL: FTL_FLUSH a0 bit 0 = FTL_WearLevel (AND 0x74) before the flush
AND_FACTORY_REFORMAT = 0x6c


def and_rc_ok(cmd, rc):
    """Interpret a WMR/AND dispatcher return code. Every command: 0 = success. Exception :
    AND 0x6c (FactoryReformat, fw 0x08012998 returns 0 on success, the dispatcher 0x0800759a maps only 1 to success)
    returns 0x80000001 when the reformat WORKED -> None ('indeterminate: verify by reopening'), never False.
    The harness does not expose 0x6c; this keeps any future caller from reading its success as a failure."""
    rc &= 0xffffffff
    if cmd == AND_FACTORY_REFORMAT and rc in (0, 0x80000001):
        return None
    return rc == 0


def token(kind, a0, a1):
    """Confirmation token the device checks for commands 130..136 (mirrors nand_poc_token in nand_poc.c)."""
    a1 &= 0xffffffff
    return (TOKEN_SEED ^ ((kind & 0xff) << 24) ^ (a0 & 0xffffffff) ^ (((a1 << 8) | (a1 >> 24)) & 0xffffffff)) & 0xffffffff

FTL_MAX_PAGES = 64
ROM_PAGE = 0x600
LPN_BYTES = 4096
RP_DATA, RP_META, RP_MAXCORR, RP_SECSTAT, RP_BYTES = 0x0, 0x1000, 0x1010, 0x1020, 0x1040

# status record layout: (name, struct format); must match struct nand_poc_status (size 0x250)
STATUS_LAYOUT = [
    ("magic", "I"), ("version", "I"), ("size", "I"), ("busy", "I"), ("state", "I"),
    ("step_reached", "I"), ("step_ok_mask", "I"), ("jobs_started", "I"), ("jobs_done", "I"),
    ("job_kind", "I"), ("job_arg", "3I"), ("job_result", "I"), ("job_lib_rc", "I"), ("job_detail", "4I"),
    ("job_t_start", "I"), ("job_t_end", "I"), ("progress_done", "I"), ("progress_total", "I"),
    ("step_status", "8I"), ("chip_ids", "8I"), ("geom", "12I"),
    ("ftl_pages", "I"), ("ftl_bpp", "I"), ("disk_mount_rc", "I"),
    ("result_addr", "I"), ("result_cap", "I"), ("result_len", "I"), ("ring_addr", "I"), ("ring_bytes", "I"),
    ("clk_mhz", "I"), ("pwrcon0_before", "I"), ("pwrcon0_after", "I"), ("gpio_after", "3I"), ("pmu", "2I"),
    ("fil_pages", "I"), ("fil_uecc", "I"), ("fil_timeouts", "I"),
    ("alloc_bytes", "I"), ("alloc_count", "I"), ("alloc_fail", "I"),
    ("panic_count", "I"), ("panic_line", "I"), ("panic_t_us", "I"), ("panic_file", "32s"),
    ("wmr_asserts", "I"), ("err_job_seq", "I"), ("snap_valid", "I"),
    ("fmc_snap", "24I"), ("ecc_snap", "6I"), ("fmss_snap", "28I"),
    ("now_us", "I"), ("hb_word", "I"), ("wdt_request", "I"),
]
STATUS_FMT = "<" + "".join(f for _, f in STATUS_LAYOUT)
STATUS_BYTES = 0x250
assert struct.calcsize(STATUS_FMT) == STATUS_BYTES


class Bunch(dict):
    __getattr__ = dict.__getitem__


class ToolError(Exception):
    pass


# ------------------------------------------------------------------------------------------- pure parsers
def u32(b, off):
    return struct.unpack_from("<I", b, off)[0]


def parse_status(raw):
    """Decode the 0x250-byte nand_poc_status record."""
    if len(raw) < STATUS_BYTES:
        raise ValueError("status record too short (%d bytes)" % len(raw))
    vals = list(struct.unpack_from(STATUS_FMT, raw, 0))
    out = Bunch()
    for name, fmt in STATUS_LAYOUT:
        n = int(fmt[:-1]) if len(fmt) > 1 and fmt[-1] == "I" else 1
        if fmt.endswith("s"):
            out[name] = vals.pop(0).split(b"\0", 1)[0].decode("ascii", "replace")
        elif n == 1:
            out[name] = vals.pop(0)
        else:
            out[name] = [vals.pop(0) for _ in range(n)]
    if out.magic != NPOC_MAGIC:
        raise ValueError("bad status magic 0x%08x" % out.magic)
    if out.version != NPOC_VERSION or out.size != STATUS_BYTES:
        raise ValueError("status version %d size 0x%x (tool expects %d / 0x%x)"
                         % (out.version, out.size, NPOC_VERSION, STATUS_BYTES))
    return out


def parse_img1_header(page0):
    """IMG1 header of the reserved region : magic '8720', version, fmt, lengths; ROM page count."""
    if len(page0) < 0x40:
        raise ValueError("header too short")
    magic = page0[0:4].decode("latin-1")
    version = page0[4:7].decode("latin-1")
    fmt = page0[7]
    datalen = u32(page0, 0x10)
    rom_ok = fmt in (3, 4) and datalen < 0x24c01      # rom_boot_nand's only checks before copying
    return Bunch(magic=magic, version=version, fmt=fmt, entry=u32(page0, 8), bodylen=u32(page0, 0xc),
                 datalen=datalen, certoff=u32(page0, 0x14), certlen=u32(page0, 0x18), rom_accepts=rom_ok,
                 data_pages=(datalen + ROM_PAGE - 1) // ROM_PAGE if rom_ok else 0)


def parse_mbr(sector):
    """MBR at LBA 0 (LBA = 4 KiB FTL page): 4 entries at +0x1be, signature 55 AA at +0x1fe."""
    if len(sector) < 512:
        raise ValueError("sector too short")
    sig = sector[0x1fe] == 0x55 and sector[0x1ff] == 0xAA
    parts = []
    for i in range(4):
        e = sector[0x1be + 16 * i: 0x1be + 16 * (i + 1)]
        parts.append(Bunch(index=i, active=e[0], type=e[4], start=u32(e, 8), size=u32(e, 12)))
    return Bunch(valid=sig, partitions=parts)


def parse_ftl_free_ring(cxt):
    """FTL context head (GetStruct 0x1000300, nand_structs.h FTLCxt): wNumOfFreeVb @0x08, wNextFreeIdx @0x0a,
    swapCounter @0x0c, awFreeVb[20] @0x0e; free VB k = awFreeVb[(wNextFreeIdx + k) % 20]."""
    if len(cxt) < 0x36:
        raise ValueError("FTL context too short")
    n, nxt, swaps = struct.unpack_from("<HHH", cxt, 8)
    ring = struct.unpack_from("<20H", cxt, 0x0e)
    if n > 20 or nxt >= 20:
        raise ValueError("implausible free ring (count %d, next %d)" % (n, nxt))
    return Bunch(count=n, next=nxt, swap_counter=swaps, free=[ring[(nxt + k) % 20] for k in range(n)])


STOP_SIGN_PREFIX = b"{{~~  /-----\\"


def is_stop_sign(block):
    """Firmware partition block 0 starts with the 0x7c-byte iPod 'STOP sign' text."""
    return block[:len(STOP_SIGN_PREFIX)] == STOP_SIGN_PREFIX


def fourcc(v):
    """Directory ids are u32 LE whose big-endian characters spell the name ('osos' = 0x6f736f73)."""
    return struct.pack(">I", v).decode("latin-1")


def fw_dir_offset(block0):
    """dirOffset = u32 at +0x104; the directory sits at partition byte offset dirOffset + 4096."""
    return u32(block0, 0x104)


def parse_fw_directory(buf):
    """Up to 40 entries of 0x28 bytes, ending at the first entry whose word 0 is 0."""
    out = []
    for i in range(40):
        off = i * 0x28
        if off + 0x28 > len(buf):
            break
        w0 = u32(buf, off)
        if w0 == 0:
            break
        out.append(Bunch(index=i, type=w0, id=u32(buf, off + 4), name=fourcc(u32(buf, off + 4)),
                         devoffset=u32(buf, off + 0xc), length=u32(buf, off + 0x10), addr=u32(buf, off + 0x14),
                         entryoffset=u32(buf, off + 0x18), checksum=u32(buf, off + 0x1c),
                         version=u32(buf, off + 0x20), loadaddr=u32(buf, off + 0x24)))
    return out


def fw_file_location(part_start_lpn, entry):
    """(first lpn, byte offset in that page, length) of a file's data: partition offset devOffset + 4096."""
    off = part_start_lpn * LPN_BYTES + entry.devoffset + LPN_BYTES
    return off // LPN_BYTES, off % LPN_BYTES, entry.length


RING_MAGIC = 0x5247344E
TAGS = {0x494E464F: "INFO", 0x50414E43: "PANC"}


def parse_ring(raw):
    """Platform ring buffer (s5l8720_bringup.h): header {magic, head, total, capacity}, 16-byte records."""
    magic, head, total, cap = struct.unpack_from("<4I", raw, 0)
    if magic != RING_MAGIC:
        return Bunch(valid=False, records=[])
    n = min(total, cap)
    first = (head - n) % cap if cap else 0
    recs = []
    for k in range(n):
        i = (first + k) % cap
        tag, a, b, t = struct.unpack_from("<4I", raw, 16 + 16 * i)
        if tag >> 24 in (0x53, 0x45) and tag & 0xffffff < 0x100:
            name = "%s%d" % ("BEGIN " if tag >> 24 == 0x53 else "END   ", tag & 0xff)
        else:
            name = TAGS.get(tag, "0x%08x" % tag)
        recs.append(Bunch(tag=tag, name=name, a=a, b=b, t_us=t))
    return Bunch(valid=True, total=total, records=recs)


def hexdump(data, base=0, width=16, limit=None):
    lines = []
    data = data if limit is None else data[:limit]
    for i in range(0, len(data), width):
        chunk = data[i:i + width]
        asc = "".join(chr(c) if 32 <= c < 127 else "." for c in chunk)
        lines.append("%06x  %-*s %s" % (base + i, width * 3, " ".join("%02x" % c for c in chunk), asc))
    return "\n".join(lines)


def format_status(st):
    rc = lambda v: "0x%08x%s" % (v, " (%s)" % LIB_RC_NAMES[v] if v in LIB_RC_NAMES else "")
    L = []
    L.append("state %s  busy %d  jobs %d/%d  last job %s(%s) -> %s, lib rc %s" % (
        STATE_NAMES.get(st.state, st.state), st.busy, st.jobs_done, st.jobs_started,
        JOB_NAMES.get(st.job_kind, st.job_kind), ", ".join("0x%x" % a for a in st.job_arg),
        RESULT_NAMES.get(st.job_result, st.job_result), rc(st.job_lib_rc)))
    L.append("  detail %s  progress %d/%d  t %d..%d us (now %d)" % (
        " ".join("0x%x" % d for d in st.job_detail), st.progress_done, st.progress_total,
        st.job_t_start, st.job_t_end, st.now_us))
    L.append("steps: reached %d, ok mask 0x%x" % (st.step_reached, st.step_ok_mask))
    for s in range(1, 6):
        v = st.step_status[s]
        L.append("  %d %-24s %s" % (s, STEP_NAMES[s], "not run" if v == 0xffffffff else
                                     "RUNNING" if v == 0xfffffffe else rc(v)))
    L.append("chip ids: " + " ".join("%08x" % i for i in st.chip_ids))
    L.append("geometry: " + ", ".join("%s=0x%x" % (n, v) for n, v in zip(GEOM_NAMES, st.geom)))
    L.append("ftl: pages %d (%.1f MiB), bytes/page %d, disk_mount rc 0x%08x" % (
        st.ftl_pages, st.ftl_pages * 4096 / 1048576.0, st.ftl_bpp, st.disk_mount_rc))
    L.append("clk %d MHz, PWRCON0 0x%08x -> 0x%08x, GPIO c0/e0/100 %s, PMU %s" % (
        st.clk_mhz, st.pwrcon0_before, st.pwrcon0_after, " ".join("%08x" % g for g in st.gpio_after),
        " ".join("%08x" % p for p in st.pmu)))
    L.append("FIL pages %d, uecc sectors %d, timeouts %d; alloc %d B in %d (fail %d); wmr asserts %d" % (
        st.fil_pages, st.fil_uecc, st.fil_timeouts, st.alloc_bytes, st.alloc_count, st.alloc_fail, st.wmr_asserts))
    L.append("panics %d, last %s:%d at %d us" % (st.panic_count, st.panic_file, st.panic_line, st.panic_t_us))
    L.append("result buffer 0x%08x cap 0x%x len 0x%x; ring 0x%08x (%d B)" % (
        st.result_addr, st.result_cap, st.result_len, st.ring_addr, st.ring_bytes))
    if st.snap_valid:
        L.append("register snapshot (job %d):" % st.err_job_seq)
        L.append("  FMC  000: " + " ".join("%08x" % v for v in st.fmc_snap))
        L.append("  ECC  800: " + " ".join("%08x" % v for v in st.ecc_snap))
        L.append("  FMSS c00: " + " ".join("%08x" % v for v in st.fmss_snap))
    return "\n".join(L)


# ------------------------------------------------------------------------------------------- device session
class Session:
    def __init__(self, settle=0.05, poll=0.05, verbose=False, ep0=False):
        self.settle, self.poll, self.verbose, self.ep0 = settle, poll, verbose, ep0
        self.retries = 0          # transient USB errors absorbed by _io / read
        self.em = None
        self._connect()

    def _connect(self):
        if HERE not in sys.path:
            sys.path.insert(0, HERE)
        import libemcore   # lazy: the parsers and their tests need no pyusb
        self.libemcore = libemcore
        self.em = libemcore.Emcore()
        if self.ep0:
            # --ep0: never use the bulk-IN endpoint; libemcore.read() then streams 0xff0-byte control reads
            # (: 5.7 MB of read-lpn over EP0 worked where bulk-IN claim/transfer failed)
            self.em.lib.bulkin = None

    def _recover(self, exc):
        """Bundle rule: a USB error is the host control-endpoint halt, not the device -> dev.reset(), reconnect."""
        print("USB error (%s): resetting the device handle and reconnecting" % exc, file=sys.stderr)
        # Order matters (Oct 18 2026): release + dispose the OLD handle before the port reset, otherwise the
        # new session's claim_interface fails with USBError(16) 'Resource busy' while the device is perfectly alive.
        import usb.util
        dev = None
        try:
            dev = self.em.lib.dev.dev
        except Exception:
            pass
        try:
            self.em.close()
        except Exception:
            pass
        if dev is not None:
            try:
                usb.util.dispose_resources(dev)
            except Exception:
                pass
            try:
                dev.reset()
            except Exception:
                pass
        self.em = None
        last = None
        for attempt in range(5):
            time.sleep(1.0 + attempt)
            try:
                self._connect()
                return
            except Exception as e:  # noqa: BLE001
                last = e
                print("reconnect attempt %u failed: %s" % (attempt + 1, e), file=sys.stderr)
        raise RuntimeError("could not reconnect after the USB error: %s" % last)

    def _io(self, fn, *a):
        """Transient control-transfer errors (EIO, timeouts) are common on this build and clear on an immediate retry
        (: dozens per 250 KiB of reads, every retry succeeded). Retry a few times first; only a persistent
        failure gets the heavy recovery (handle reset + reconnect)."""
        import usb.core
        # Capture the operation's path, not just its bound receiver. Recovery
        # replaces self.em (and self.em.lib); invoking the old bound method
        # afterwards lets PyUSB reclaim the disposed handle while the NEW
        # connection owns the interface, failing with EBUSY.
        owner = getattr(fn, "__self__", None)
        method = getattr(fn, "__name__", None)
        receiver_path = () if owner is self.em else ("lib",) if owner is self.em.lib else None
        last = None
        for attempt in range(8 if self.ep0 else 4):
            try:
                return fn(*a)
            except usb.core.USBError as e:
                last = e
                self.retries += 1
                if e.errno == 19:          # no such device: do not spin, go straight to recovery
                    break
                time.sleep(min(0.1 * (attempt + 1), 1.0))
        self._recover(last)
        if receiver_path is not None:
            receiver = self.em
            for attr in receiver_path:
                receiver = getattr(receiver, attr)
            fn = getattr(receiver, method)
        return fn(*a)

    def cmd(self, c, a=0, b=0, d=0):
        r = self._io(self.em.lib.monitorcommand, struct.pack("<IIII", c, a, b, d), "III", ("w1", "w2", "w3"))
        return r.w1, r.w2, r.w3

    def read(self, addr, size):
        if not self.ep0:
            return bytes(self._io(self.em.read, addr, size))
        # --ep0: 1 KiB control reads. : some pieces of the NAND result buffer fail at 1 KiB but read fine at
        # 512 B or smaller (address/size-specific, not data), so a failing piece is re-read in smaller transfers.
        import usb.core
        out = b""
        while size:
            n = min(size, 1024)
            piece = None
            for sub in (n, 512, 256, 64):
                if sub > n:
                    continue
                try:
                    buf = b""
                    for off in range(0, n, sub):
                        buf += bytes(self._io(self.em._readmem, addr + off, min(sub, n - off)))
                    piece = buf
                    break
                except (usb.core.USBError, RuntimeError):
                    self.retries += 1
                    print("  read %08x/%u failed even with retries; trying %u-byte transfers" % (addr, sub, sub // 2 if sub > 64 else 0), file=sys.stderr)
            if piece is None:
                raise ToolError("READ_MEM %08x/%u failed at every transfer size" % (addr, n))
            out += piece
            addr += n
            size -= n
        return out

    def heartbeat(self):
        return self._io(self.em.get_monitor_heartbeat)

    def status(self):
        addr, size, _ = self.cmd(CMD_STATUS)
        if size != STATUS_BYTES:
            raise ToolError("device status size 0x%x != tool 0x%x (rebuild/update)" % (size, STATUS_BYTES))
        return parse_status(self.read(addr, size))

    def job(self, c, a=0, b=0, timeout=10.0, progress=False, d=0):
        """Queue one job, poll until it is done. Raises ToolError (after printing the record) on failure."""
        time.sleep(self.settle)
        acc, job_id, _ = self.cmd(c, a, b, d)
        if acc != 0:
            raise ToolError("command %d not accepted: %s" % (c, ACCEPT_NAMES.get(acc, acc)))
        t0 = time.time()
        last = -1
        while True:
            time.sleep(self.poll)
            st = self.status()
            if st.jobs_done == job_id and st.state == 3:
                return st
            if progress and st.progress_done != last:
                last = st.progress_done
                print("  ... %s %d/%d  (FIL pages %d, %.0f s)" % (
                    STATE_NAMES.get(st.state), st.progress_done, st.progress_total, st.fil_pages,
                    time.time() - t0), file=sys.stderr)
            if time.time() - t0 > timeout:
                hb = self.heartbeat()
                print(format_status(st), file=sys.stderr)
                raise ToolError("job %d not done after %.0f s (heartbeat %d, nand word 0x%08x, FIL pages %d)"
                                % (job_id, timeout, hb.heartbeat, hb.nand_word, hb.nand_progress))

    def result(self, st, n):
        return self.read(st.result_addr, n)


def check(st, ok_lib=(0,)):
    if st.job_result != 0 or st.job_lib_rc not in ok_lib:
        print(format_status(st), file=sys.stderr)
        raise ToolError("job failed: %s, lib rc 0x%08x" % (RESULT_NAMES.get(st.job_result, st.job_result),
                                                           st.job_lib_rc))
    return st


# ------------------------------------------------------------------------------------------- subcommands
def cmd_status(s, a):
    st = s.status()
    print(format_status(st))
    hb = s.heartbeat()
    print("heartbeat %d, nand word 0x%08x, FIL pages %d" % (hb.heartbeat, hb.nand_word, hb.nand_progress))


def cmd_step(s, a):
    timeout = a.timeout or (900.0 if a.n == 5 else 15.0)
    st = check(s.job(CMD_STEP, a.n, a.arg, timeout=timeout, progress=a.n == 5))
    print("step %d (%s): status 0x%08x" % (a.n, STEP_NAMES.get(a.n, "?"), st.step_status[a.n]))
    if a.n == 4:
        cmd_ids_print(st)
    if a.n == 5:
        print("ftl pages %d, bytes/page %d, disk_mount rc 0x%08x" % (st.ftl_pages, st.ftl_bpp, st.disk_mount_rc))


NAND_MAKERS = {0xad: "Hynix", 0x98: "Toshiba", 0xec: "Samsung", 0x2c: "Micron", 0x89: "Intel", 0x45: "SanDisk", 0x01: "Spansion/AMD"}


def decode_nand_id(word):
    """Decode a 4-byte READ ID answer (little-endian word as the harness stores it) using the legacy Samsung/Hynix/
    Toshiba ID-byte convention (3rd/4th byte layouts; vendor datasheets agree on these bits for 2K/4K/8K parts).
    Returns a list of (field, value) pairs; everything beyond maker/device is marked 'legacy layout'."""
    b = struct.pack("<I", word)
    if word in (0, 0xffffffff):
        return [("present", "no chip (bus %s)" % ("floating high" if word else "low"))]
    maker, dev, b3, b4 = b[0], b[1], b[2], b[3]
    dies = 1 << (b3 & 3)
    levels = 1 << ((b3 >> 2 & 3) + 1)                       # 2 = SLC, 4 = MLC, 8 = TLC
    simul = 1 << (b3 >> 4 & 3)
    page_kib = 1 << (b4 & 3)
    spare_per_512 = 16 if (b4 & 4) else 8
    block_kib = 64 << (b4 >> 4 & 3)
    width = 16 if (b4 & 0x40) else 8
    out = [("maker", "0x%02x %s" % (maker, NAND_MAKERS.get(maker, "unknown"))),
           ("device code", "0x%02x" % dev),
           ("dies per CE", "%u (byte3[1:0])" % dies),
           ("cell type", "%u-level (%s) (byte3[3:2])" % (levels, {2: "SLC", 4: "MLC", 8: "TLC"}.get(levels, "?"))),
           ("simult. pages", "%u (byte3[5:4])" % simul),
           ("interleave/cache", "%s/%s (byte3[6]/[7])" % ("yes" if b3 & 0x40 else "no", "yes" if b3 & 0x80 else "no")),
           ("page size", "%u KiB (byte4[1:0])" % page_kib),
           ("spare", "%u B per 512 B -> %u B per page (byte4[2])" % (spare_per_512, spare_per_512 * page_kib * 2)),
           ("block size", "%u KiB -> %u pages (byte4[5:4])" % (block_kib, block_kib // page_kib)),
           ("bus width", "x%u (byte4[6])" % width)]
    return out


def cmd_ids_print(st):
    for ce, v in enumerate(st.chip_ids):
        print("CE%d: %08x  bytes %s" % (ce, v, " ".join("%02x" % x for x in struct.pack("<I", v))))
    first = next((v for v in st.chip_ids if v not in (0, 0xffffffff)), None)
    if first is not None:
        print("decoded ID (CE0..%u, legacy ID-byte layout):" % max(i for i, v in enumerate(st.chip_ids) if v == first))
        for k, val in decode_nand_id(first):
            print("  %-18s %s" % (k, val))
        geom = dict(zip(GEOM_NAMES, st.geom))
        exp = {"bytes_per_page": 1024 << (struct.pack("<I", first)[3] & 3),
               "pages_per_block": (64 << (struct.pack("<I", first)[3] >> 4 & 3)) * 1024 // (1024 << (struct.pack("<I", first)[3] & 3))}
        for k, e in exp.items():
            if k in geom and geom[k]:
                print("  cross-check %-12s ID says %u, driver table says %u -> %s" % (k, e, geom[k], "match" if e == geom[k] else "MISMATCH"))
    print("driver geometry (chip table):")
    for n, v in zip(GEOM_NAMES, st.geom):
        print("  %-18s 0x%x (%d)" % (n, v, v))


def cmd_ids(s, a):
    st = s.status()
    if not st.step_ok_mask & (1 << 4):
        print("step 4 (FIL init) has not succeeded; IDs below are whatever the last attempt left", file=sys.stderr)
    cmd_ids_print(st)


def read_reserved(s, page, reset):
    st = s.job(CMD_READ_RESERVED, page, 1 if reset else 0)
    check(st, ok_lib=(0, 1))
    return st.job_lib_rc, s.result(st, ROM_PAGE)


def cmd_read_reserved(s, a):
    rc, data = read_reserved(s, a.page, not a.no_reset)
    print("bank 0 page %d: rc %d %s" % (a.page, rc, ROM_RC_NAMES.get(rc, "")))
    if a.page == 0:
        h = parse_img1_header(data)
        print("IMG1: magic %r version %r fmt %d datalen 0x%x -> %d data pages (rom accepts: %s)"
              % (h.magic, h.version, h.fmt, h.datalen, h.data_pages, h.rom_accepts))
    if a.output:
        open(a.output, "wb").write(data)
        print("wrote %s" % a.output)
    else:
        print(hexdump(data, limit=256))


def cmd_dump_reserved(s, a):
    rc0, page0 = read_reserved(s, 0, True)
    h = parse_img1_header(page0)
    print("page 0 rc %d; IMG1 magic %r version %r fmt %d datalen 0x%x -> %d data pages"
          % (rc0, h.magic, h.version, h.fmt, h.datalen, h.data_pages))
    n = a.pages if a.pages is not None else h.data_pages
    if a.pages is None and not h.rom_accepts:
        raise ToolError("header not accepted by the ROM rules (fmt 3/4, datalen < 0x24c01); pass --pages N")
    rcs = [rc0]
    with open(a.output, "wb") as f:
        f.write(page0)
        for p in range(1, n + 1):
            rc, data = read_reserved(s, p, False)
            rcs.append(rc)
            f.write(data)
            if p % 16 == 0 or p == n:
                print("  page %d/%d" % (p, n), file=sys.stderr)
    bad = [i for i, r in enumerate(rcs) if r != 1]
    print("wrote %s: %d pages x 0x600 (0x%x bytes); pages with rc != 1: %s" % (
        a.output, n + 1, (n + 1) * ROM_PAGE, bad if bad else "none"))


def cmd_read_page(s, a):
    st = check(s.job(CMD_READ_PAGE, a.bank, a.page, d=1 if a.uncached else 0), ok_lib=(0, 1))
    r = s.result(st, RP_BYTES)
    if a.uncached:
        print("uncached variant: mode %d = %s (alias descriptor 0x%08x)" % (
            st.job_detail[2], UC_MODES.get(st.job_detail[2], "?"), st.job_detail[3]))
    data, meta = r[RP_DATA:RP_DATA + 4096], r[RP_META:RP_META + 12]
    print("bank %d page %d: rc 0x%x (%s), maxcorr %d, meta %s, secstat %s" % (
        a.bank, a.page, st.job_lib_rc, LIB_RC_NAMES.get(st.job_lib_rc, "?"), r[RP_MAXCORR],
        meta.hex(), r[RP_SECSTAT:RP_SECSTAT + 16].hex()))
    if a.output:
        open(a.output, "wb").write(data + meta)
        print("wrote %s (4096 data + 12 meta)" % a.output)
    else:
        print(hexdump(data, limit=256))


def cmd_ftl_open(s, a):
    a.n, a.arg = 5, 0
    cmd_step(s, a)


def ftl_read(s, lpn, count, keep_ecc=False):
    st = s.job(CMD_FTL_READ, lpn, count, timeout=60.0)
    if not (keep_ecc and st.job_result == 1 and st.job_lib_rc == 0x80000002):
        check(st)                     # stop at the first error (ECC-marked pages only pass with --keep-ecc)
    return st, s.result(st, count * LPN_BYTES)


def cmd_read_lpn(s, a):
    out = b""
    lpn, left = a.lpn, a.count
    while left:
        n = min(left, FTL_MAX_PAGES)
        _, data = ftl_read(s, lpn, n)
        out += data
        lpn += n
        left -= n
    if a.output:
        open(a.output, "wb").write(out)
        print("wrote %s (%d pages)" % (a.output, a.count))
    else:
        print(hexdump(out, base=a.lpn * LPN_BYTES, limit=512))


def _retry_note(s):
    if s.retries:
        print("(%u transient USB errors were retried successfully)" % s.retries, file=sys.stderr)


def cmd_dump_disk(s, a):
    st = s.status()
    if not st.step_ok_mask & (1 << 5):
        raise ToolError("FTL not open: run `ftl-open` first")
    total = st.ftl_pages
    count = a.count if a.count is not None else total - a.start
    if a.start + count > total:
        raise ToolError("range %d+%d exceeds the disk (%d pages)" % (a.start, count, total))
    done = 0
    if os.path.exists(a.output) and not a.overwrite:
        size = os.path.getsize(a.output)
        done = min(size // LPN_BYTES, count)
        print("resuming %s at page %d" % (a.output, a.start + done))
    mode = "r+b" if done else "wb"
    errlog = open(a.output + ".errors", "a")
    t0 = time.time()
    with open(a.output, mode) as f:
        f.seek(done * LPN_BYTES)
        f.truncate()
        while done < count:
            n = min(a.chunk, count - done, FTL_MAX_PAGES)
            lpn = a.start + done
            st, data = ftl_read(s, lpn, n, keep_ecc=a.keep_ecc)
            if st.job_lib_rc == 0x80000002:
                errlog.write("lpn %d..%d: ECC-marked page inside (first chunk lpn 0x%x)\n"
                             % (lpn, lpn + n - 1, st.job_detail[1]))
                errlog.flush()
            f.write(data)
            f.flush()
            done += n
            if done % (16 * a.chunk) == 0 or done == count:
                rate = done * LPN_BYTES / max(time.time() - t0, 1e-3) / 1048576.0
                print("  %d/%d pages (%.1f%%, %.2f MiB/s this session)" % (done, count, 100.0 * done / count, rate),
                      file=sys.stderr)
    print("wrote %s: lpn %d..%d" % (a.output, a.start, a.start + count - 1))
    _retry_note(s)


def cmd_mbr(s, a):
    _, lba0 = ftl_read(s, 0, 1)
    m = parse_mbr(lba0)
    print("MBR signature %s" % ("ok" if m.valid else "MISSING"))
    for p in m.partitions:
        print("  part %d: type 0x%02x active 0x%02x start lba %d size %d (%.1f MiB)" % (
            p.index, p.type, p.active, p.start, p.size, p.size * 4096 / 1048576.0))
    for p in m.partitions:
        if p.size == 0:
            continue
        _, blk = ftl_read(s, p.start, 1)
        if not is_stop_sign(blk):
            print("  part %d: first bytes %s" % (p.index, blk[:16].hex()))
            continue
        diroff = fw_dir_offset(blk)
        first = p.start + (diroff + LPN_BYTES) // LPN_BYTES
        _, d = ftl_read(s, first, 2)
        o = (diroff + LPN_BYTES) % LPN_BYTES
        ents = parse_fw_directory(d[o:o + 0x640])
        print("  part %d: firmware partition (STOP sign), dirOffset 0x%x, %d entries" % (p.index, diroff, len(ents)))
        for e in ents:
            lpn, off, ln = fw_file_location(p.start, e)
            print("    %-4s type 0x%08x devOffset 0x%08x len 0x%08x load 0x%08x -> lpn %d +0x%x" % (
                e.name, e.type, e.devoffset, e.length, e.loadaddr, lpn, off))


def cmd_get_struct(s, a):
    st = check(s.job(CMD_GET_STRUCT, a.id, a.input))
    n = st.job_detail[0]
    data = s.result(st, min(n, st.result_cap)) if n else b""
    print("GetStruct 0x%08x: %d bytes" % (a.id, n))
    if a.output:
        open(a.output, "wb").write(data)
        print("wrote %s" % a.output)
    else:
        print(hexdump(data, limit=512))


def cmd_free_vbs(s, a):
    st = check(s.job(CMD_GET_STRUCT, 0x1000300, 0))
    r = parse_ftl_free_ring(s.result(st, min(st.job_detail[0], 0x400)))
    print("FTL free VBs: %d (ring next %d, merges since last wear-level step %d): %s" % (
        r.count, r.next, r.swap_counter, " ".join(str(v) for v in r.free)))
    print("(VB n = physical block n on every bank before remap; erase-block/program-page re-check it on the device)")


def cmd_ring(s, a):
    st = s.status()
    r = parse_ring(s.read(st.ring_addr, st.ring_bytes))
    if not r.valid:
        print("ring not initialised (no bring-up step ran yet)")
        return
    print("ring: %d records logged" % r.total)
    for x in r.records:
        print("  %10d us  %-10s a=0x%08x b=0x%08x" % (x.t_us, x.name, x.a, x.b))


def cmd_watchdog(s, a):
    acc, secs, armed = s.cmd(CMD_WATCHDOG, a.seconds)
    print("NAND_WATCHDOG(%d): %s, armed=%d" % (secs, ACCEPT_NAMES.get(acc, acc), armed))
    if acc == 4:
        print("the S5L8720 watchdog enable/kick sequence is not established from the firmware;"
              " nothing was written to 0x3c800000")



# ------------------------------------------------------------------------------------------- write path (P2b)
def write_info(s):
    flags, stage_addr, stage_cap = s.cmd(CMD_WRITE_INFO)
    return Bunch(write_build=bool(flags & 1), snapshot_done=bool(flags & 2), rw_open=bool(flags & 4),
                 storage_writes=bool(flags & 8), session_end=bool(flags & 0x10), stage_addr=stage_addr,
                 stage_cap=stage_cap)


def require_destructive(a, wi, need_rw=False):
    if not wi.write_build:
        raise ToolError("the running image is a read-only build (use the rw build, 'Write path')")
    if not wi.get("session_end", False):
        raise ToolError("the running rw image predates WRITE_INFO bit 4 (staging-buffer cache fix + session-end job, "
                        "): uploaded page data could be lost in the D-cache. Rebuild/boot the current rw image")
    if not getattr(a, "i_understand_this_is_destructive", False):
        raise ToolError("refusing: add %s" % DESTRUCTIVE_FLAG)
    # The per-boot snapshot requirement is disabled (user request); the device no longer enforces it either.
    if need_rw and not wi.rw_open:
        raise ToolError("refusing: the write stack is not open (run `open-rw` first)")


def destructive_job(s, a, cmd, a0, a1, timeout=60.0, need_rw=False):
    wi = write_info(s)
    require_destructive(a, wi, need_rw)
    kind = CMD_TO_JOB[cmd]
    print("BEFORE:"); print(format_status(s.status()))
    st = s.job(cmd, a0, a1, timeout=a.timeout or timeout, d=token(kind, a0, a1))
    print("AFTER:"); print(format_status(st))
    if st.job_result == 6:
        raise ToolError("refused by the device gate: %s%s" % (GATE_NAMES.get(st.job_detail[0], st.job_detail[0]),
                        (" (%s)" % IN_USE_WHY.get(st.job_detail[1], st.job_detail[1])) if st.job_detail[0] == 5 else ""))
    return st


def stage_upload(s, wi, data, offset=0):
    # EP0 only: the device's stage_sync() cleans+invalidates the D-cache for CPU-copied (EP0) data; a bulk-OUT DMA
    # upload would be overwritten by that clean if older dirty lines exist (nand_poc.c stage_sync)
    if getattr(s.em, "bulk_write", False):
        raise ToolError("EMCORE_BULK_WRITE=1 is not supported for the staging buffer: unset it (EP0 writes only)")
    if offset + len(data) > wi.stage_cap:
        raise ToolError("staging buffer too small (%d > %d)" % (offset + len(data), wi.stage_cap))
    s._io(s.em.write, wi.stage_addr + offset, data)
    back = s.read(wi.stage_addr + offset, len(data))
    if back != data:
        raise ToolError("staging buffer verify failed (host read-back differs)")


def cmd_write_info(s, a):
    wi = write_info(s)
    print("write build: %s  snapshot done: %s  write stack open: %s  storage writes: %s  session-end job: %s" % (
        wi.write_build, wi.snapshot_done, wi.rw_open, wi.storage_writes, wi.session_end))
    print("staging buffer: 0x%08x (%d bytes = %d pages)" % (wi.stage_addr, wi.stage_cap, STAGE_PAGES))


def cmd_snapshot(s, a):
    wi = write_info(s)
    if not wi.write_build:
        raise ToolError("read-only build: snapshot is only needed/available in the rw build")
    os.makedirs(a.output, exist_ok=True)
    st = check(s.job(CMD_SNAPSHOT, timeout=a.timeout or 120.0, progress=True))
    pages, ctx_off, banks = st.job_detail[0], st.job_detail[1], st.job_detail[2]
    data = s.result(st, st.result_len)
    with open(os.path.join(a.output, "reserved.bin"), "wb") as f:
        f.write(data[:ctx_off])
    if st.result_len > ctx_off:
        with open(os.path.join(a.output, "contexts.bin"), "wb") as f:
            f.write(data[ctx_off:])
    print("snapshot: %d reserved pages (%d bytes) -> %s/reserved.bin%s" % (
        pages, ctx_off, a.output, ("; VSVFL(%d banks)+FTL contexts -> contexts.bin" % banks) if st.result_len > ctx_off else ""))
    print("rom read status: page0 rc %d, worst %d" % (st.job_detail[3] & 0xff, st.job_detail[3] >> 8))


def cmd_open_rw(s, a):
    st = destructive_job(s, a, CMD_OPEN_RW, 0, 0, timeout=900.0)
    check(st)
    print("write stack open: %d pages x %d bytes" % (st.job_detail[0], st.job_detail[1]))


def cmd_erase_block(s, a):
    a1 = a.block | (FORCE if a.force else 0)
    st = destructive_job(s, a, CMD_ERASE_BLOCK, a.bank, a1)
    check(st)
    print("erase bank %d block %d: lib rc 0x%08x (in-use check: %s)" % (
        a.bank, a.block, st.job_detail[2], IN_USE_WHY.get(st.job_detail[1], st.job_detail[1])))


def cmd_program_page(s, a):
    wi = write_info(s)
    data = open(a.input, "rb").read()
    if len(data) != LPN_BYTES:
        raise ToolError("page data must be exactly %d bytes" % LPN_BYTES)
    meta = bytes.fromhex(a.meta) if a.meta else b"\xff" * 12
    if len(meta) != 12:
        raise ToolError("meta must be 12 bytes (24 hex digits)")
    require_destructive(a, wi)
    stage_upload(s, wi, data + meta + b"\0" * 4)
    a1 = a.page | (FORCE if a.force else 0)
    st = destructive_job(s, a, CMD_PROGRAM_PAGE, a.bank, a1)
    check(st)
    print("program bank %d page %d: lib rc 0x%08x, failing CE %d" % (a.bank, a.page, st.job_detail[2], st.job_detail[3]))


def cmd_write_lpn(s, a):
    wi = write_info(s)
    data = open(a.input, "rb").read()
    if len(data) == 0 or len(data) % LPN_BYTES:
        raise ToolError("data must be a multiple of %d bytes" % LPN_BYTES)
    count = len(data) // LPN_BYTES
    require_destructive(a, wi, need_rw=True)
    done = 0
    while done < count:
        n = min(STAGE_PAGES, count - done)
        stage_upload(s, wi, data[done * LPN_BYTES:(done + n) * LPN_BYTES])
        check(destructive_job(s, a, CMD_FTL_WRITE, a.lpn + done, n, timeout=120.0, need_rw=True))
        done += n
        print("wrote lpn %d..%d" % (a.lpn, a.lpn + done - 1))


def cmd_flush(s, a):
    check(destructive_job(s, a, CMD_FTL_FLUSH, 0, 0, timeout=300.0, need_rw=True))
    print("FTL flushed")


def session_end(s, a, quiet=False):
    """End a write session the way the OS does (fw FUN_0802d8f8 == osos osos_and_sync): AND 0x74 (FTL_WearLevel,
    result ignored) then AND 0x71 with {1} (FTL_Flush(1): close every open log, store the context)."""
    wi = write_info(s)
    if wi.write_build and not wi.session_end:
        raise ToolError("the running rw image predates WRITE_INFO bit 4 (session-end job + staging cache fix): it "
                        "would run a plain flush(0) and may program stale staging data. Rebuild/boot the current rw image")
    if quiet:
        require_destructive(a, wi, need_rw=True)
        print("-> session end: FTL wear level (AND 0x74), then FTL flush(1) (AND 0x71)")
        st = s.job(CMD_FTL_FLUSH, FLUSH_WEARLEVEL, 1, timeout=a.timeout or 600.0,
                   d=token(JOB_FTL_FLUSH, FLUSH_WEARLEVEL, 1))
    else:
        st = destructive_job(s, a, CMD_FTL_FLUSH, FLUSH_WEARLEVEL, 1, timeout=600.0, need_rw=True)
    if st.job_result == 6:
        raise ToolError("session end refused by the device gate: %s" % GATE_NAMES.get(st.job_detail[0], st.job_detail[0]))
    check(st)
    print("session end ok: wear-level work steps %d, free VBs %d, flush rc 0x%08x" % (
        st.job_detail[0], st.job_detail[1], st.job_detail[2]))
    return st


def cmd_session_end(s, a):
    session_end(s, a)


def ftl_write_chunk(s, a, wi, lpn, data, upload=True):
    """One FTL_WRITE job (<= STAGE_PAGES pages), one line before it is queued and one after. upload=False reuses
    the staging buffer as it is (the caller proved its contents, e.g. zeros). Returns the status record."""
    n = len(data) // LPN_BYTES
    if not 0 < n <= STAGE_PAGES or len(data) % LPN_BYTES:
        raise ToolError("bad chunk size %d" % len(data))
    if upload:
        stage_upload(s, wi, data)
    print("-> FTL_WRITE lpn %d..%d (%d pages%s)" % (lpn, lpn + n - 1, n, "" if upload else ", staged zeros"))
    st = s.job(CMD_FTL_WRITE, lpn, n, timeout=a.timeout or 120.0, d=token(JOB_FTL_WRITE, lpn, n))
    if st.job_result == 6:
        print(format_status(st), file=sys.stderr)
        raise ToolError("write refused by the device gate: %s" % GATE_NAMES.get(st.job_detail[0], st.job_detail[0]))
    check(st)
    return st


def cmd_bootarea_write(s, a):
    wi = write_info(s)
    data = open(a.input, "rb").read()
    if len(data) != ROM_PAGE:
        raise ToolError("boot-area page data must be exactly 0x600 bytes")
    require_destructive(a, wi)
    if not a.force:
        raise ToolError("refusing: the boot area is bank 0 block 0 (what the ROM boots); add --force")
    stage_upload(s, wi, data)
    check(destructive_job(s, a, CMD_BOOTAREA_WR, a.page, FORCE))
    print("boot-area page %d written" % a.page)


def cmd_enable_storage_writes(s, a):
    st = destructive_job(s, a, CMD_STORAGE_WR, 1 if not a.disable else 0, 0, need_rw=not a.disable)
    check(st)
    print("storage-layer writes %s" % ("disabled" if a.disable else "enabled"))


def cmd_boot_rw(s, a):
    """Two-stage boot: upload a (too large for DFU) emCORE image into free SDRAM and execfirmware() it to 0x08000000."""
    data = open(a.image, "rb").read()
    lower, upper = s._io(s.em.getmallocpoolbounds).lower, s._io(s.em.getmallocpoolbounds).upper
    addr = (upper - len(data) - 0x100000) & ~0xfff          # top of the pool, below the running kernel's image region
    if addr < lower + 0x400000:
        raise ToolError("image too large for a two-stage load (%d bytes)" % len(data))
    print("uploading %d bytes to 0x%08x, then execfirmware(0x08000000)" % (len(data), addr))
    s._io(s.em.write, addr, data)
    if s.read(addr, 64) != data[:64]:
        raise ToolError("upload verify failed")
    try:
        s._io(s.em.execfirmware, 0x08000000, addr, len(data))
    except Exception as e:
        print("(device went away as expected: %s)" % e)
    print("second stage started; wait a few seconds, then run `status` again")


def auto_int(v):
    return int(v, 0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--settle", type=float, default=0.05, help="seconds to wait before each command (default 0.05)")
    ap.add_argument("--poll", type=float, default=0.05, help="status poll interval in seconds")
    ap.add_argument("--timeout", type=float, default=None, help="job timeout override in seconds")
    ap.add_argument("--ep0", action="store_true", help="read results over EP0 control transfers only (no bulk-IN); slower, robust")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    p = sub.add_parser("step"); p.add_argument("n", type=int); p.add_argument("--arg", type=auto_int, default=0)
    sub.add_parser("ids")
    p = sub.add_parser("read-reserved"); p.add_argument("page", type=auto_int); p.add_argument("-o", "--output")
    p.add_argument("--no-reset", action="store_true", help="skip the NAND RESET of bank 0 before the read")
    p = sub.add_parser("dump-reserved"); p.add_argument("-o", "--output", required=True)
    p.add_argument("--pages", type=auto_int, default=None, help="data pages after page 0 (default from IMG1 header)")
    p = sub.add_parser("read-page"); p.add_argument("bank", type=auto_int); p.add_argument("page", type=auto_int)
    p.add_argument("-o", "--output")
    p.add_argument("--uncached", action="store_true",
                   help="read through an uncached alias if the MMU map has one, else clean+invalidate after DMA")
    sub.add_parser("ftl-open")
    p = sub.add_parser("read-lpn"); p.add_argument("lpn", type=auto_int)
    p.add_argument("count", type=auto_int, nargs="?", default=1); p.add_argument("-o", "--output")
    p = sub.add_parser("dump-disk"); p.add_argument("-o", "--output", required=True)
    p.add_argument("--start", type=auto_int, default=0); p.add_argument("--count", type=auto_int, default=None)
    p.add_argument("--chunk", type=auto_int, default=FTL_MAX_PAGES, help="pages per job (<= 64)")
    p.add_argument("--overwrite", action="store_true", help="do not resume an existing output file")
    p.add_argument("--keep-ecc", action="store_true",
                   help="continue on ECC-marked pages (data kept, range logged to OUTPUT.errors)")
    sub.add_parser("mbr")
    p = sub.add_parser("get-struct"); p.add_argument("id", type=auto_int); p.add_argument("-o", "--output")
    p.add_argument("--input", type=auto_int, default=0, help="input word 0 (id 0x1000800: lpn)")
    sub.add_parser("ring")
    sub.add_parser("free-vbs")
    p = sub.add_parser("watchdog"); p.add_argument("seconds", type=auto_int, help="0 = disarm")
    # ---- write path (rw build only; every destructive command needs the flag and a snapshot in this boot)
    def destructive(parser):
        parser.add_argument(DESTRUCTIVE_FLAG, dest="i_understand_this_is_destructive", action="store_true")
        return parser
    sub.add_parser("write-info")
    p = sub.add_parser("snapshot"); p.add_argument("-o", "--output", required=True, help="directory for reserved.bin / contexts.bin")
    destructive(sub.add_parser("open-rw"))
    p = destructive(sub.add_parser("erase-block")); p.add_argument("bank", type=auto_int); p.add_argument("block", type=auto_int)
    p.add_argument("--force", action="store_true", help="also blocks the FTL reports as in use, and bank 0 block 0")
    p = destructive(sub.add_parser("program-page")); p.add_argument("bank", type=auto_int); p.add_argument("page", type=auto_int)
    p.add_argument("-i", "--input", required=True, help="4096-byte data file"); p.add_argument("--meta", help="12-byte meta as hex")
    p.add_argument("--force", action="store_true")
    p = destructive(sub.add_parser("write-lpn")); p.add_argument("lpn", type=auto_int); p.add_argument("-i", "--input", required=True)
    destructive(sub.add_parser("flush"))
    destructive(sub.add_parser("session-end"))
    p = destructive(sub.add_parser("bootarea-write")); p.add_argument("page", type=auto_int); p.add_argument("-i", "--input", required=True)
    p.add_argument("--force", action="store_true", help="required: the boot area is what the ROM boots")
    p = destructive(sub.add_parser("enable-storage-writes")); p.add_argument("--disable", action="store_true")
    p = sub.add_parser("boot-rw"); p.add_argument("image", help="emCORE/build/ipodnano4g/emcore-rw.bin (two-stage load, no DFU size limit)")
    a = ap.parse_args(argv)
    handlers = {"status": cmd_status, "step": cmd_step, "ids": cmd_ids, "read-reserved": cmd_read_reserved,
                "dump-reserved": cmd_dump_reserved, "read-page": cmd_read_page, "ftl-open": cmd_ftl_open,
                "read-lpn": cmd_read_lpn, "dump-disk": cmd_dump_disk, "mbr": cmd_mbr, "get-struct": cmd_get_struct,
                "ring": cmd_ring, "free-vbs": cmd_free_vbs, "watchdog": cmd_watchdog, "write-info": cmd_write_info, "snapshot": cmd_snapshot,
                "open-rw": cmd_open_rw, "erase-block": cmd_erase_block, "program-page": cmd_program_page,
                "write-lpn": cmd_write_lpn, "flush": cmd_flush, "session-end": cmd_session_end, "bootarea-write": cmd_bootarea_write,
                "enable-storage-writes": cmd_enable_storage_writes, "boot-rw": cmd_boot_rw}
    try:
        s = Session(settle=a.settle, poll=a.poll, ep0=a.ep0)
        handlers[a.cmd](s, a)
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
