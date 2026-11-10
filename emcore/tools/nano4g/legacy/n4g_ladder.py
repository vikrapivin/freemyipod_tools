#!/usr/bin/env python3
"""Primitive ladder for a BOOTED nano4g emCORE (round 14): verify ONE emCORE primitive per rung, with a
read-back check, before the next rung uses it. Nothing is assumed to work. n4g_memtest2.py rebooted the device although
it only combined malloc, WRITE_MEM, FLUSH CACHE and CREATE THREAD; the five later steps that used (almost) the same
primitives survived. The ladder splits the difference into single steps. NOTHING SURVIVES A RESET: every rung prints what it
is about to do before it does it, so the last line on the screen names the primitive that killed the device.

Rungs (run them in this order; each one is a separate process so a reboot costs only the current rung):
  0 read       READ_MEM: vectors at 0x22000000 vs the image file, the µs timer advances
  1 write      WRITE_MEM 4 / 56 / 64 / 264 B into free SRAM (0x22020100) and read back
  2 malloc     two malloc(0x1000): inside the pool, disjoint, writable, read back; free the second
  3 flush      FLUSH CACHE (cmd 20) acknowledges, device alive
  4 procinfo   GET PROCESS INFO: thread list, every stack guard word == 0xaffebeaf
  5 kthread    CREATE THREAD running KERNEL code boot_beacon(77, 0xabcd) with memtest2's parameters (prio 32, type 1,
               malloc'd stack): the beacon area must show id 77 / aux 0xabcd and the thread must be gone afterwards
  6 hret       first code from the HEAP: 'bx lr' in a malloc'd block, memtest2's geometry (code +0, stack +0x800/0x800)
  7 hstore     heap code 'str r1,[r0]; bx lr' with arg0 = block+0x400, arg1 = 0x600dc0de -> word must change
  8 hclean     heap code doing clean-entire-D-cache (c7,c10,0) + drain, then the store
  9 hcleaninv  heap code doing clean+invalidate-entire-D-cache (c7,c14,0) + drain, then the store  (= the memtest's op)
 10 mt-heap4k  the real memtest routine over 4 KiB of a second malloc'd block
 11 mt-tail4k  the memtest routine over 0x09f9a000..0x09f9b000 (first 4 KiB of the free tail below the framebuffer)
 12 mt-tail    the memtest routine over the whole tail 0x09f9a000..0x09fd4000 (= memtest2's first region)
 13 mt-heap    the memtest routine over a 16 MiB heap block (--mib)
 14 regs READ-ONLY register/state checks from the kernel audit : DMAC0 ch4 (a stalled, still-enabled channel
               makes every panic spin forever = faults hang, not reboot), VIC select/enable, the stub's leftover page-table
               entries for VA >= 0x40000000 (ROM/WDT/SRAM mappings a wild PC could hit), idle thread PC, the tail region read
               twice 10 s apart (anything writing there?), and, last, the watchdog registers
 15 cpuload    heap code: a pure countdown loop (--count iterations, no memory traffic): CPU load / supply only
 16 readsweep  heap code: read the tail region --passes times (SDRAM read load, no writes)
 17 writesweep-heap  heap code: write addr^key over a --size (64 KiB) malloc'd block --passes times, no cache op;
               snapshots .sram/.ibss/.sdram/.bss before and diffs after (did the writes corrupt other memory?)
               Round-14 result: 64 KiB x 16 KILLED the device (reads x16 of 232 KiB and pure CPU load survived) ->
               bisect: `17 --size 0x4000 --passes 1`, `17 --size 0x10000 --passes 1`, `17 --size 0x10000 --passes 4`,
               `17 --size 0x40000 --passes 1` (volume vs duration)
 18 writesweep-tail  heap code: the same write sweep over the whole tail region, no cache op
 19 alias      READ-ONLY-ish: 64-B marker at tail+0x100, look for it 1/2/4/8/16 MiB lower (SDRAM smaller than 32 MiB -> wraps)
 21 writeslow  heap code: the write sweep with --throttle spin iterations after every 32-B line and optional --value
               (constant data instead of addr^key: fewer data-line toggles). Round-14: throttle 64 at 64 KiB x16 STILL kills ->
               not rate. Next discriminators: `17 --size 0x100000 --passes 1 --no-snapshot` (1 MiB once: volume vs repetition),
               `21 --throttle 0 --value 0` (1 MiB of zeros: data toggling / I/O power), `17 --size 0x2000 --passes 128 --no-snapshot`
 22 sdram READ-ONLY : memory-controller regs vs stock, DLL lock value, CLKCON (CPU 266 vs 133 MHz), PMU
               reg 2 (0x80 = the stub skipped the SDRAM init this boot), PMU 4/0x33/0x30/0x31; I2C reads last
 20 icache     run heap routine A, overwrite with B in place, FLUSH CACHE, run again: A back = stale I-cache line, the
               emulator agent's erratum-411920 claim is real on this device; B = the invalidate works
Bisecting rung 12 (mt-tail killed the device, 4 KiB of the tail did not): --tail-start/--tail-end apply to every tail rung,
e.g. `python tools/n4g_ladder.py 18 --tail-end 0x09fb7000` (first half), then quarter it.
 udf (explicit only) heap code 'udf #0': the exception/panic path. Expect per : device HANGS (panic path
               spins on the stalled LCD DMA) -> only run this when nothing else is left to learn from this boot.
Usage: python tools/n4g_ladder.py <rung name or number> [--map M] [--image I] [--prio 32] [--stacksize 0x800] [--mib 16]
                                 [--count 0x8000000] [--passes 16]
       python tools/n4g_ladder.py all      # recommended order (ORDER below), stops at the first FAIL
Numbers are stable IDs; 'all' runs: 0 1 2 3 4 14 19 5 6 7 8 9 15 16 17 18 10 11 12 13 (load rungs before the memtests).
"""
import argparse
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
IMG_DIR = os.path.abspath(os.path.join(HERE, "..", "..", "..", "build", "ipodnano4g"))
DEFAULT_IMAGE = os.path.join(IMG_DIR, "emcore-nano4g.bin")
DEFAULT_MAP = os.path.join(IMG_DIR, "emcore-nano4g.map")
BLOB_PATH = os.path.join(HERE, "memtest", "memtest_arm.bin")
STUB_LEN = 2580
SRAM_FREE = 0x22020100          # beacon area is 0x22020000..0x22020040; .ibss ends 0x22004e80; L1 table at 0x2202c000
BEACON_BASE = 0x22020000
BEACON_MAGIC = 0x314e4342
USEC = 0x3c7000b4
POOL = (0x08000000, 0x09f80000)
TAIL = (0x09f9a000, 0x09fd4000)      # overridden by --tail-start/--tail-end (bisecting rung 12)
GUARD = 0xaffebeaf
PARAMS_WORDS = 14
NAMES = ["read", "write", "malloc", "flush", "procinfo", "kthread", "hret", "hstore", "hclean", "hcleaninv",
         "mt-heap4k", "mt-tail4k", "mt-tail", "mt-heap", "regs", "cpuload", "readsweep", "writesweep-heap", "writesweep-tail", "alias", "icache", "writeslow", "sdram"]
ORDER = ["read", "write", "malloc", "flush", "procinfo", "regs", "alias", "kthread", "hret", "hstore", "hclean", "hcleaninv",
         "cpuload", "readsweep", "writesweep-heap", "writesweep-tail", "mt-heap4k", "mt-tail4k", "mt-tail", "mt-heap"]
PROBES_PATH = os.path.join(HERE, "memtest", "loadprobes_arm.bin")   # from loadprobes_arm.S (arm-none-eabi-as)
PROBE_OFFS = {"cpuload": (0x00, 0x18), "readsweep": (0x18, 0x58), "writesweep": (0x58, 0x98), "writeslow": (0x98, 0x108)}
MIU = 0x3d700000
CLKCON = 0x3c500000
DMAC0 = 0x38200000
L1 = 0x2202c000
WDT = 0x3c800000

# ARM opcodes (little endian words)
BX_LR = 0xe12fff1e
STR_R1_R0 = 0xe5801000          # str r1, [r0]
MOV_R2_0 = 0xe3a02000           # mov r2, #0
MCR_CLEAN_ALL = 0xee072f1a      # mcr p15, 0, r2, c7, c10, 0
MCR_CLEANINV_ALL = 0xee072f1e   # mcr p15, 0, r2, c7, c14, 0
MCR_DRAIN = 0xee072f9a          # mcr p15, 0, r2, c7, c10, 4
UDF0 = 0xe7f000f0


def words(*ws):
    return struct.pack("<%dI" % len(ws), *ws)


def load_map(path):
    syms = {}
    for line in open(path):
        p = line.split()
        if len(p) == 3:
            syms.setdefault(p[2], int(p[0], 16))
    return syms


class Ladder:
    def __init__(self, a):
        import libemcore
        self.em = libemcore.Emcore()
        self.a = a
        self.syms = load_map(a.map)
        self.fails = 0
        self.eio = 0 # intermittent EP0 READ_MEM EIO under volume (device alive; )

    # ---- primitives, each used only after its rung passed ----
    def rd(self, addr, size):
        """READ_MEM in 1 KiB control transfers; a 4 KiB control read failed once with EIO on the host, so retry."""
        out = b""
        while size:
            n = min(size, 1024)
            for attempt in range(6):
                try:
                    out += self.em._readmem(addr, n)
                    break
                except Exception as e:  # noqa: BLE001
                    if attempt == 5:
                        raise
                    self.eio += 1
                    self.say("       (READ_MEM %08x/%u failed: %r; retry %u; EIO so far %u)" % (addr, n, e, attempt + 1, self.eio))
                    time.sleep(0.2 * (attempt + 1))
            addr += n
            size -= n
        return out

    def rw(self, addr, n=1):
        return struct.unpack("<%dI" % n, self.rd(addr, 4 * n))

    def wr(self, addr, data):
        for off in range(0, len(data), 64): # 64-B EP0 writes only (bulk OUT wedges this build)
            self.em._writemem(addr + off, data[off:off + 64])

    def say(self, s):
        print(s)
        sys.stdout.flush()

    def plan(self, s):
        self.say("[plan] " + s)

    def verdict(self, ok, s):
        self.say(("[PASS] " if ok else "[FAIL] ") + s)
        if not ok:
            self.fails += 1
        return ok

    def alive(self, what):
        time.sleep(0.5)
        try:
            t1 = self.rw(USEC)[0]
            time.sleep(0.05)
            t2 = self.rw(USEC)[0]
        except Exception as e:  # noqa: BLE001
            return self.verdict(False, "device does not answer after %s: %r" % (what, e))
        return self.verdict(t2 != t1, "device alive after %s (usec %u -> %u)" % (what, t1, t2))

    def create_thread(self, name_ptr, entry, stack, stacksize, prio, args):
        a = (list(args) + [0, 0, 0, 0])[:4]
        cmd = words(19, name_ptr, entry, stack, stacksize, 1, prio, 2, *a)
        return self.em.lib.monitorcommand(cmd, "III", ("threadptr", None, None)).threadptr & 0xffffffff

    def threads(self):
        ts = []
        for t in self.em.getprocinfo():
            d = t.__dict__ if hasattr(t, "__dict__") else dict(t)
            ts.append(d)
        return ts

    def thread_status(self, name):
        for d in self.threads():
            if str(d.get("name")) == name:
                return "%s (%s)" % (d.get("state"), d.get("block_type"))
        return "gone"

    def new_block(self, size=0x1000):
        p = self.em.malloc(size)
        if not p or not (POOL[0] <= p < POOL[1]):
            raise RuntimeError("malloc(%#x) returned %#x" % (size, p or 0))
        return p

    def console_tail(self):
        """USB console backlog; the map address of dbgconsendbuf is approximate on KNOWNGOOD, so print printable runs only."""
        buf = self.syms.get("dbgconsendbuf")
        if not buf:
            return "(no dbgconsendbuf in map)"
        data = self.rd(buf - 0x40, 4096 + 0x80)
        runs, cur = [], b""
        for c in data:
            if 32 <= c < 127 or c in (10,):
                cur += bytes([c])
            else:
                if len(cur) >= 4:
                    runs.append(cur.decode("ascii"))
                cur = b""
        if len(cur) >= 4:
            runs.append(cur.decode("ascii"))
        return "\n".join(runs[-12:]) if runs else "(empty)"

    # ---- integrity: snapshot .sram/.ibss/.sdram/.bss before a write test, diff afterwards ----
    SNAP_REGIONS = (("sram", 0x22000000, 15360), ("ibss", 0x22003c00, 0x22004e80 - 0x22003c00),
                    ("sdram", 0x09f80000, 82412), ("bss", 0x09f941ec, 0x09f99d00 - 0x09f941ec))

    def snapshot(self):
        return {n: self.rd(a, sz) for n, a, sz in self.SNAP_REGIONS}

    def symbolize(self, addr):
        best = None
        for name, a in self.syms.items():
            if a <= addr and (best is None or a > best[1]) and not name.startswith("."):
                best = (name, a)
        return "%s+%#x" % best if best and addr - best[1] < 0x2000 else "?"

    def diff_snapshot(self, before, after, label):
        total = 0
        for n, a, sz in self.SNAP_REGIONS:
            b, c = before[n], after[n]
            changed = [i for i in range(0, sz, 4) if b[i:i + 4] != c[i:i + 4]]
            total += len(changed)
            if changed:
                shown = ", ".join("%08x(%s)" % (a + i, self.symbolize(a + i)) for i in changed[:8])
                self.say("       %s: %u words changed during %s: %s%s" % (n, len(changed), label, shown, " ..." if len(changed) > 8 else ""))
        self.say("       (changes in .data/.bss variables such as tick counters are normal; changes in .sram/.sdram CODE or many"
                 " scattered words = the writes corrupted other memory)")
        return total

    # ---- a heap thread: code words at +0, name at +0x3c0, params at +0x400, stack at +0x800 ----
    def heap_thread(self, tag, code_words, args, flush=True, wait=2.0):
        blk = self.new_block()
        code = words(*code_words)
        self.plan("block %08x: %u B code at +0, name at +0x3c0, params at +0x400 (zeroed), stack +0x800 size %#x; prio %u"
                  % (blk, len(code), self.a.stacksize, self.a.prio))
        self.wr(blk, code)
        self.wr(blk + 0x3c0, tag.encode() + b"\0")
        self.wr(blk + 0x400, bytes(64))
        got = self.rd(blk, len(code))
        self.verdict(got == code, "code read back %s" % ("identical" if got == code else "DIFFERENT"))
        if flush:
            self.plan("FLUSH CACHE (cmd 20)")
            self.em.flushcaches()
        real_args = [blk + 0x400 if x == "params" else x for x in args]
        self.plan("CREATE THREAD '%s' entry %08x args %s  <-- the risky step" % (tag, blk, ["%#x" % x for x in real_args]))
        tp = self.create_thread(blk + 0x3c0, blk, blk + 0x800, self.a.stacksize, self.a.prio, real_args)
        self.say("       thread struct %08x" % tp)
        time.sleep(wait)
        return blk, tp

    # ---- rungs ----
    def r_read(self):
        self.plan("READ_MEM 256 B at 0x22000000 and compare with the image's .sram start; read the µs timer twice")
        img = open(self.a.image, "rb").read()
        sram = img[STUB_LEN + 4 + 4:]      # stub, u32 payload length, 4-B .inithead (branch to __start), then .sram
        dev = self.rd(0x22000000, 256)
        self.verdict(dev == sram[:256], ".sram head %s the image file (%s)" % (
            "matches" if dev == sram[:256] else "DIFFERS from", os.path.basename(self.a.image)))
        v = struct.unpack("<8I", dev[:32])
        self.verdict(all(x == 0xe59ff018 for x in v), "8 vectors are 'ldr pc,[pc,#24]' (%s)" % " ".join("%08x" % x for x in v))
        return self.alive("READ_MEM")

    def r_write(self):
        for n in (4, 56, 64, 264):
            pat = bytes((i * 7 + n) & 0xff for i in range(n))
            self.plan("WRITE_MEM %u B to %08x (free SRAM), read back, zero again" % (n, SRAM_FREE))
            self.wr(SRAM_FREE, pat)
            got = self.rd(SRAM_FREE, n)
            self.verdict(got == pat, "%u B read back %s" % (n, "identical" if got == pat else "DIFFERENT: %s" % got.hex()))
            self.wr(SRAM_FREE, bytes(n))
        return self.alive("WRITE_MEM")

    def r_malloc(self):
        self.plan("malloc(0x1000) twice (cmd 52, runs on the dbg thread)")
        p1 = self.em.malloc(0x1000)
        p2 = self.em.malloc(0x1000)
        self.say("       p1 %08x p2 %08x" % (p1, p2))
        self.verdict(all(POOL[0] <= p < POOL[1] for p in (p1, p2)), "both inside the pool %08x..%08x" % POOL)
        self.verdict(abs(p1 - p2) >= 0x1000, "blocks disjoint")
        self.verdict(p1 % 8 == 0 and p2 % 8 == 0, "8-byte aligned")
        pat = bytes((i * 13 + 5) & 0xff for i in range(0x1000))
        self.plan("fill p1 (4 KiB) through WRITE_MEM, read back whole block")
        self.wr(p1, pat)
        got = self.rd(p1, 0x1000)
        self.verdict(got == pat, "4 KiB read back %s" % ("identical" if got == pat else "DIFFERENT"))
        self.plan("free(p2) (cmd 56)")
        self.em.free(p2)
        return self.alive("malloc/free")

    def r_flush(self):
        self.plan("FLUSH CACHE (cmd 20): clean_dcache + invalidate_icache in the USB IRQ handler")
        self.em.flushcaches()
        return self.alive("FLUSH CACHE")

    def r_procinfo(self):
        self.plan("GET PROCESS INFO (freezes the scheduler while the host reads the thread structs)")
        ts = self.threads()
        ok = True
        for d in ts:
            st = d.get("stack", 0)
            guard = None
            if st not in (0, 0xffffffff):
                try:
                    guard = self.rw(st)[0]
                except Exception:  # noqa: BLE001
                    guard = None
            g = "stack=-1" if st == 0xffffffff else ("guard %08x %s" % (guard, "ok" if guard == GUARD else "BAD") if guard is not None else "guard ?")
            if guard is not None and guard != GUARD:
                ok = False
            self.say("       %-18s %-22s prio %3s load %3s %s" % (d.get("name"), d.get("state"), d.get("priority"), d.get("cpuload"), g))
        self.verdict(ok, "%u threads listed, all stack guard words intact" % len(ts))
        return self.alive("GET PROCESS INFO")

    def r_kthread(self):
        bb = self.syms.get("boot_beacon")
        if not bb:
            return self.verdict(False, "map has no boot_beacon")
        before = list(self.rw(BEACON_BASE, 4))
        stack = self.new_block()
        nm = self.new_block(64)
        self.wr(nm, b"ladder-k\0")
        self.plan("CREATE THREAD entry boot_beacon=%08x(77, 0xabcd), stack %08x size %#x, prio %u, type 1 (kernel code, memtest2's parameters)"
                  % (bb, stack, self.a.stacksize, self.a.prio))
        tp = self.create_thread(nm, bb, stack, self.a.stacksize, self.a.prio, [77, 0xabcd])
        self.say("       thread struct %08x" % tp)
        time.sleep(1.5)
        if not self.alive("kernel-code thread"):
            return False
        after = list(self.rw(BEACON_BASE, 4))
        self.verdict(after[0] == BEACON_MAGIC and after[1] == 77 and after[3] == 0xabcd and after[2] == (before[2] + 1 if before[0] == BEACON_MAGIC else 1),
                     "beacon area before %s after %s (expect id 77, aux 0xabcd, count+1)" % (["%x" % x for x in before], ["%x" % x for x in after]))
        self.verdict(self.thread_status("ladder-k") in ("gone",) or "DEFUNCT" in self.thread_status("ladder-k"),
                     "thread 'ladder-k' afterwards: %s" % self.thread_status("ladder-k"))
        return self.fails == 0

    def r_hret(self):
        blk, tp = self.heap_thread("ladder-ret", [BX_LR], [])
        if not self.alive("heap thread 'bx lr'"):
            return False
        return self.verdict("DEFUNCT" in self.thread_status("ladder-ret") or self.thread_status("ladder-ret") == "gone",
                            "thread afterwards: %s" % self.thread_status("ladder-ret"))

    def _store_rung(self, tag, prefix):
        blk, tp = self.heap_thread(tag, prefix + [STR_R1_R0, BX_LR], ["params", 0x600dc0de])
        if not self.alive("heap thread '%s'" % tag):
            return False
        v = self.rw(blk + 0x400)[0]
        self.verdict(v == 0x600dc0de, "params word = %08x (expect 600dc0de: code ran from the heap with the right args)" % v)
        return self.verdict("DEFUNCT" in self.thread_status(tag) or self.thread_status(tag) == "gone",
                            "thread afterwards: %s" % self.thread_status(tag)) and v == 0x600dc0de

    def r_hstore(self):
        return self._store_rung("ladder-st", [])

    def r_hclean(self):
        return self._store_rung("ladder-cl", [MOV_R2_0, MCR_CLEAN_ALL, MCR_DRAIN])

    def r_hcleaninv(self):
        return self._store_rung("ladder-ci", [MOV_R2_0, MCR_CLEANINV_ALL, MCR_DRAIN])

    def _run_blob(self, tag, what, blob, params_words, timeout, progress=None):
        """Upload a routine + param block into a fresh 4 KiB block, FLUSH CACHE, start it as a thread, poll done (+32)."""
        blk = self.new_block()
        params = blk + 0x400
        self.plan("%s (%u B) in block %08x, params +0x400 = %s, stack +0x800 size %#x, prio %u"
                  % (what, len(blob), blk, ["%#x" % x for x in params_words[:3]], self.a.stacksize, self.a.prio))
        self.wr(blk, blob)
        self.wr(blk + 0x3c0, tag.encode() + b"\0")
        self.wr(params, words(*(list(params_words) + [0] * (PARAMS_WORDS - len(params_words)))))
        got = self.rd(blk, len(blob))
        self.verdict(got == blob, "routine read back %s" % ("identical" if got == blob else "DIFFERENT"))
        self.plan("FLUSH CACHE")
        self.em.flushcaches()
        self.plan("CREATE THREAD '%s'  <-- the risky step" % tag)
        tp = self.create_thread(blk + 0x3c0, blk, blk + 0x800, self.a.stacksize, self.a.prio, [params])
        self.say("       thread struct %08x" % tp)
        t0 = time.time()
        p = None
        while time.time() - t0 < timeout:
            time.sleep(0.3)
            try:
                p = struct.unpack("<%dI" % PARAMS_WORDS, self.rd(params, 4 * PARAMS_WORDS))
            except Exception as e:  # noqa: BLE001
                return None, self.verdict(False, "device stopped answering while '%s' ran: %r" % (tag, e))
            if progress:
                sys.stdout.write("\r       " + progress(p) + "   ")
                sys.stdout.flush()
            if p[8]:
                break
        if progress:
            print()
        if not p or not p[8]:
            return p, self.verdict(False, "done flag never set within %u s; last params %s" % (timeout, p))
        self.say("       finished in %.1f s" % (time.time() - t0))
        return p, self.alive("thread '%s'" % tag)

    def _memtest(self, tag, base, size, timeout=120):
        blob = open(BLOB_PATH, "rb").read()
        p, ok = self._run_blob(tag, "memtest routine over %08x..%08x (%u KiB)" % (base, base + size, size >> 10), blob,
                               [base, size, 4], timeout,
                               lambda p: "pass %u/4 progress %6u KiB bad %u" % (min(p[13] + 1, 4), p[3] >> 10, p[4]))
        if p and p[8]:
            self.verdict(True, "bad words %u (per pattern %s)%s" % (
                p[4], p[9:13], "" if not p[4] else "; first @%08x exp %08x got %08x" % (p[5], p[6], p[7])))
        return ok

    def _probe(self, which):
        lo, hi = PROBE_OFFS[which]
        return open(PROBES_PATH, "rb").read()[lo:hi]

    def r_alias(self):
        """Is the SDRAM really 32 MiB, or does the tail (offset 31.6 MiB) wrap onto lower addresses? Marker at tail+0x100,
        FLUSH CACHE so it reaches the SDRAM, then look for it 1/2/4/8/16 MiB lower (free heap, never allocated this early)."""
        x = TAIL[0] + 0x100
        marker = bytes((i * 29 + 17) & 0xff for i in range(64))
        cands = [x - (m << 20) for m in (1, 2, 4, 8, 16)]
        self.plan("read %s before, write a 64-B marker at %08x, FLUSH CACHE, read the candidates again, restore zeros" % (
            ["%08x" % c for c in cands], x))
        before = {c: self.rd(c, 64) for c in cands}
        self.wr(x, marker)
        self.em.flushcaches()
        hit = []
        for c in cands:
            after = self.rd(c, 64)
            if after == marker and before[c] != marker:
                hit.append(c)
            self.say("       %08x: %s" % (c, "== MARKER (ALIAS, period %u MiB)" % ((x - c) >> 20) if after == marker else "unchanged" if after == before[c] else "changed otherwise"))
        self.wr(x, bytes(64))
        self.em.flushcaches()
        self.verdict(not hit, "no SDRAM aliasing seen" if not hit else "SDRAM ALIASES with period %u MiB: the 'tail' overlays live memory!" % ((x - hit[-1]) >> 20))
        return self.alive("alias check")

    def r_icache(self):
        """: the model claims the single 'invalidate entire I-cache' (ARM erratum 411920) fails
        depending on where the MCR sits in its cache line, and that FLUSH CACHE's invalidate_icache is such a case. Test:
        run routine A from a heap block (its line is now in the I-cache), overwrite it in place with routine B, FLUSH CACHE,
        run again. B = invalidate works; A = STALE I-CACHE, the erratum is real on this device. Third run: overwrite with C
        without any flush (informational: shows whether heap code is cached at all)."""
        blk = self.new_block()
        params = blk + 0x400
        self.wr(blk + 0x3c0, b"ladder-ic\0")
        results = []
        for step, (val, flush) in enumerate(((0xa, True), (0xb, True), (0xc, False))):
            code = words(0xe3a01000 | val, STR_R1_R0, BX_LR)     # mov r1,#val; str r1,[r0]; bx lr
            self.wr(blk, code)
            self.wr(params, bytes(4))
            if flush:
                self.plan("routine %X written at %08x, FLUSH CACHE, run it (expect result %X)" % (val, blk, val))
                self.em.flushcaches()
            else:
                self.plan("routine %X written at %08x WITHOUT flush, run it (stale B expected if heap code is cached)" % (val, blk))
            self.create_thread(blk + 0x3c0, blk, blk + 0x800, self.a.stacksize, self.a.prio, [params])
            time.sleep(1.0)
            if not self.alive("icache step %u" % step):
                return False
            got = self.rw(params)[0]
            results.append(got)
            self.say("       result word = %X" % got)
        self.verdict(results[0] == 0xa, "step 1 ran routine A (%X)" % results[0])
        self.verdict(results[1] == 0xb, "after overwrite + FLUSH CACHE the CPU ran %s (%X): %s" % (
            "B" if results[1] == 0xb else "A = STALE I-CACHE LINE", results[1],
            "invalidate works" if results[1] == 0xb else "erratum 411920 confirmed on this device "))
        self.say("       without flush: %X (%s)" % (results[2], "stale, heap code is cached" if results[2] != 0xc else "fresh: not cached or evicted"))
        return self.fails == 0

    def r_cpuload(self):
        p, ok = self._run_blob("ladder-cpu", "pure CPU countdown of %#x iterations" % self.a.count, self._probe("cpuload"),
                               [0, self.a.count, 0], 120)
        return ok

    def r_readsweep(self):
        size = TAIL[1] - TAIL[0]
        p, ok = self._run_blob("ladder-rd", "read sweep of the tail %08x..%08x x %u" % (TAIL[0], TAIL[1], self.a.passes),
                               self._probe("readsweep"), [TAIL[0], size, self.a.passes], 300)
        if p and p[8]:
            self.say("       word sum %08x" % p[3])
        return ok

    def r_writesweep_heap(self):
        size = self.a.size
        region = self.new_block(size)
        before = None
        if not self.a.no_snapshot:
            self.plan("snapshot .sram/.ibss/.sdram/.bss before the writes (about 125 KiB of reads; --no-snapshot skips)")
            before = self.snapshot()
        p, ok = self._run_blob("ladder-wrh", "write sweep (addr^key, no cache op) over heap block %08x (%u KiB) x %u = %u KiB written"
                               % (region, size >> 10, self.a.passes, (size * self.a.passes) >> 10),
                               self._probe("writesweep"), [region, size, self.a.passes], 300)
        if ok:
            got = struct.unpack("<%dI" % (size // 4), self.rd(region, size))
            bad = [i for i, w in enumerate(got) if w != ((region + 4 * i) ^ 0x5aa5f00f) & 0xffffffff]
            self.verdict(not bad, "block carries the pattern (%u of %u words wrong%s)" % (
                len(bad), size // 4, "" if not bad else "; first at %08x = %08x" % (region + 4 * bad[0], got[bad[0]])))
            if before:
                self.diff_snapshot(before, self.snapshot(), "the write sweep")
        return ok

    def r_writeslow(self):
        size = self.a.size
        region = self.new_block(size)
        mode, val = (0, 0) if self.a.value is None else (1, self.a.value & 0xffffffff)
        p, ok = self._run_blob("ladder-wsl", "write sweep (%s) over heap block %08x (%u KiB) x %u = %u KiB, throttle %u spins per 32-B line"
                               % ("addr^key" if not mode else "constant %08x" % val, region, size >> 10, self.a.passes,
                                  (size * self.a.passes) >> 10, self.a.throttle),
                               self._probe("writeslow"), [region, size, self.a.passes, 0, self.a.throttle, mode, val], 600)
        if ok:
            got = struct.unpack("<%dI" % (size // 4), self.rd(region, size))
            bad = [i for i, w in enumerate(got) if w != (val if mode else ((region + 4 * i) ^ 0x5aa5f00f) & 0xffffffff)]
            self.verdict(not bad, "block carries the pattern (%u of %u words wrong)" % (len(bad), size // 4))
        return ok

    def r_sdram(self):
        """READ-ONLY check from : did the stub's cold SDRAM init run, what are the clocks, are the controller
        timings the stock ones, is the DLL lock value sane. PMU I2C reads last (the I2C driver has no timeouts)."""
        self.plan("read memory-controller registers 0x3d700100..0x3d700150 and 0x3d700000/8, pad 0x3cf00384")
        exp = {0x100: 0x09101030, 0x108: 0x0002100b, 0x10c: 0x40, 0x110: 0x8040, 0x114: 0x8aac25, 0x118: 8, 0x11c: 0x19,
               0x120: 1, 0x124: 0x50d67e5, 0x148: 4, 0x14c: 0, 0x150: 0x10}
        regs = self.rw(MIU + 0x100, 0x54 // 4)
        for i, v in enumerate(regs):
            off = 0x100 + 4 * i
            e = exp.get(off)
            self.say("       MIU+%03x = %08x%s" % (off, v, "" if e is None else ("  ok" if v == e else "  != stock %08x" % e)))
        self.verdict(all(regs[(o - 0x100) // 4] == e for o, e in exp.items), "controller cfg/refresh/timing registers = stock values ")
        dll0, dll1 = regs[0x40 // 4], regs[0x44 // 4]
        lock = (dll1 >> 18) & 0xff
        self.say(" DLL 0x140=%08x 0x144=%08x lock value L=%u (setting (L-11)=%d; L<11 or near 255 = marginal)" % (dll0, dll1, lock, lock - 11))
        m0, m8 = self.rw(MIU)[0], self.rw(MIU + 8)[0]
        self.say("       MIU+000=%08x MIU+008=%08x pad 0x3cf00384=%08x 0x3e000008=%08x" % (m0, m8, self.rw(0x3cf00384)[0], self.rw(0x3e000008)[0]))
        self.plan("read clock registers CLKCON 0x3c500000/004/020/030/040")
        c = {o: self.rw(CLKCON + o)[0] for o in (0, 4, 0x20, 0x30, 0x40)}
        self.say("       CLKCON0=%08x CLKCON1=%08x +20=%08x +30=%08x +40=%08x" % (c[0], c[4], c[0x20], c[0x30], c[0x40]))
        self.say(" stub expectation 0x40001000 / 0x404040 (CPU 266 MHz); stock on USB power 0x40001001 / 0 (CPU 133 MHz)")
        self.plan("LAST: PMU registers via I2C (bus 0, slave 0xe6): reg 2 (0x80 = warm path, SDRAM init skipped this boot), 4, 0x33, 0x30, 0x31")
        vals = {}
        for r in (0x02, 0x04, 0x33, 0x30, 0x31):
            vals[r] = self.em.i2cread(0, 0xe6, r, 1)[0]
            self.say("       PMU reg %02x = %02x" % (r, vals[r]))
        self.verdict(not (vals[0x02] & 0x80), "PMU reg 2 bit7 = %u (%s)" % (
            (vals[0x02] >> 7) & 1, "cold SDRAM init ran" if not (vals[0x02] & 0x80) else "WARM PATH: the stub skipped the SDRAM init this boot "))
        return self.alive("sdram/clock/PMU reads")

    def r_writesweep_tail(self):
        size = TAIL[1] - TAIL[0]
        self.plan("snapshot .sram/.ibss/.sdram/.bss before the writes")
        before = self.snapshot()
        p, ok = self._run_blob("ladder-wrt", "write sweep (addr^key, no cache op) over the tail %08x..%08x x %u = %u KiB written"
                               % (TAIL[0], TAIL[1], self.a.passes, (size * self.a.passes) >> 10),
                               self._probe("writesweep"), [TAIL[0], size, self.a.passes], 300)
        if ok:
            self.diff_snapshot(before, self.snapshot(), "the write sweep")
        return ok

    def r_regs(self):
        self.plan("read DMAC0 channel registers (0x38200100.., ch4 = LCD)")
        for ch in range(8):
            src, dst, lli, ctl, cfg = self.rw(DMAC0 + 0x100 + 0x20 * ch, 5)
            flag = "  <-- enabled" if cfg & 1 else ""
            self.say("       ch%u SRC %08x DST %08x LLI %08x CONTROL %08x CONFIG %08x%s" % (ch, src, dst, lli, ctl, cfg, flag))
            if ch == 4:
                ch4 = cfg
        self.verdict(True, "ch4 CONFIG bit0 = %u (1 = still enabled and stalled: every panic spins at lcd.c:122 with IRQs off -> faults HANG this image)" % (ch4 & 1))
        self.plan("read VIC0/VIC1 IRQSTATUS, INTSELECT (FIQ select, must be 0), INTENABLE")
        for i, base in enumerate((0x38e00000, 0x38e01000)):
            st, fiq, raw, sel, en = self.rw(base, 5)
            self.say("       VIC%u IRQSTATUS %08x FIQSTATUS %08x RAW %08x INTSELECT %08x INTENABLE %08x" % (i, st, fiq, raw, sel, en))
            self.verdict(sel == 0, "VIC%u INTSELECT == 0 (no FIQ routed; the FIQ stack is never set up)" % i)
        self.plan("read the stub's page table entries for VA >= 0x40000000 (0x2202d000..0x22030000, leftover SRAM bytes)")
        ents = struct.unpack("<3072I", self.rd(L1 + 0x1000, 0x3000))
        kinds = {0: 0, 1: 0, 2: 0, 3: 0}
        hits = []
        for i, e in enumerate(ents):
            kinds[e & 3] += 1
            if (e & 3) == 2:
                pa = e >> 20
                if pa in range(0x200, 0x210) or pa in range(0x220, 0x223) or pa == 0x3c8:
                    hits.append((0x400 + i, e))
        self.say("       entries: fault %u coarse %u section %u fine %u" % (kinds[0], kinds[1], kinds[2], kinds[3]))
        self.say("       sections mapping ROM/SRAM/WDT: %s" % (", ".join("VA %03xxxxxx -> %08x" % (va, e) for va, e in hits[:16]) or "none"))
        e_aff = ents[0xaff - 0x400]
        self.say("       entry for VA 0xaff00000 (stack-fill word 0xaffebeaf as a PC): %08x (%s)" % (e_aff, ["fault", "coarse", "section", "fine"][e_aff & 3]))
        it = self.syms.get("idle_thread")
        if it:
            pc = self.rw(it + 60)[0]
            self.say(" idle_thread.regs[15] = %08x (%s; : should be the SRAM idle loop once it has been switched out)"
                     % (pc, "in SRAM" if 0x22000000 <= pc < 0x22005000 else "NOT in SRAM"))
        self.plan("read the tail region %08x..%08x twice, 10 s apart (changes = something writes there)" % TAIL)
        a1 = self.rd(TAIL[0], TAIL[1] - TAIL[0])
        time.sleep(10)
        a2 = self.rd(TAIL[0], TAIL[1] - TAIL[0])
        diff = [i for i in range(0, len(a1), 4) if a1[i:i + 4] != a2[i:i + 4]]
        self.verdict(not diff, "tail region unchanged over 10 s" if not diff else "tail region CHANGED: %u words, first at %08x" % (len(diff), TAIL[0] + diff[0]))
        nz = sum(1 for i in range(0, len(a1), 4) if a1[i:i + 4] != b"\0\0\0\0")
        self.say("       tail content: %u of %u words non-zero" % (nz, len(a1) // 4))
        self.plan("LAST read: watchdog registers 0x3c800000..0x3c80000c (reading only)")
        w = self.rw(WDT, 4)
        self.say("       WDT %s" % " ".join("%08x" % x for x in w))
        return self.alive("register reads")

    def r_mt_heap4k(self):
        region = self.new_block()
        return self._memtest("ladder-m4k", region, 0x1000)

    def r_mt_tail4k(self):
        return self._memtest("ladder-t4k", TAIL[0], 0x1000)

    def r_mt_tail(self):
        return self._memtest("ladder-tail", TAIL[0], TAIL[1] - TAIL[0])

    def r_mt_heap(self):
        mib, ptr = self.a.mib, None
        while mib >= 1 and ptr is None:
            try:
                ptr = self.em.malloc(mib << 20) or None
            except Exception:  # noqa: BLE001
                ptr = None
            if ptr is None:
                mib //= 2
        if ptr is None:
            return self.verdict(False, "malloc failed for every size")
        ok = self._memtest("ladder-heap", ptr, mib << 20, timeout=900)
        try:
            self.em.free(ptr)
        except Exception as e:  # noqa: BLE001
            self.say("       free failed: %r" % (e,))
        return ok

    def r_udf(self):
        self.plan("exception path: heap thread executing 'udf #0' -> undef handler -> panicf -> thread kill (expected)")
        self.say("       console backlog before:\n" + self.console_tail())
        blk, tp = self.heap_thread("ladder-udf", [UDF0, BX_LR], [])
        if not self.alive("heap thread 'udf'"):
            return False
        self.say("       thread afterwards: %s" % self.thread_status("ladder-udf"))
        self.say("       console backlog after:\n" + self.console_tail())
        return True

    def run(self, name):
        fn = {"read": self.r_read, "write": self.r_write, "malloc": self.r_malloc, "flush": self.r_flush,
              "procinfo": self.r_procinfo, "kthread": self.r_kthread, "hret": self.r_hret, "hstore": self.r_hstore,
              "hclean": self.r_hclean, "hcleaninv": self.r_hcleaninv, "mt-heap4k": self.r_mt_heap4k,
              "mt-tail4k": self.r_mt_tail4k, "mt-tail": self.r_mt_tail, "mt-heap": self.r_mt_heap, "udf": self.r_udf,
              "regs": self.r_regs, "alias": self.r_alias, "icache": self.r_icache, "writeslow": self.r_writeslow, "sdram": self.r_sdram, "cpuload": self.r_cpuload, "readsweep": self.r_readsweep,
              "writesweep-heap": self.r_writesweep_heap, "writesweep-tail": self.r_writesweep_tail}[name]
        self.say("===== rung %s =====" % name)
        self.fails = 0
        try:
            ok = fn()
        except Exception as e:  # noqa: BLE001
            ok = self.verdict(False, "exception on the host side: %r" % (e,))
        self.say("===== rung %s: %s%s =====" % (name, "PASS" if ok and self.fails == 0 else "FAIL",
                                                   " (%u READ_MEM retries)" % self.eio if self.eio else ""))
        return ok and self.fails == 0


def main():
    global TAIL
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rung", help="name or number (see list), or 'all' for 0..13")
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--prio", type=int, default=32)
    ap.add_argument("--stacksize", type=lambda x: int(x, 0), default=0x800)
    ap.add_argument("--mib", type=int, default=16)
    ap.add_argument("--count", type=lambda x: int(x, 0), default=0x8000000, help="cpuload iterations")
    ap.add_argument("--passes", type=int, default=16, help="read/write sweep passes")
    ap.add_argument("--size", type=lambda x: int(x, 0), default=0x10000, help="rung 17/21 heap block size in bytes")
    ap.add_argument("--throttle", type=lambda x: int(x, 0), default=64, help="rung 21: spin iterations after every 32-B line")
    ap.add_argument("--no-snapshot", action="store_true", help="rung 17: skip the before/after memory snapshot")
    ap.add_argument("--value", type=lambda x: int(x, 0), default=None, help="rung 21: write this constant instead of addr^key")
    ap.add_argument("--tail-start", type=lambda x: int(x, 0), default=TAIL[0])
    ap.add_argument("--tail-end", type=lambda x: int(x, 0), default=TAIL[1])
    a = ap.parse_args()
    TAIL = (a.tail_start, a.tail_end)
    if a.rung == "all":
        names = ORDER
    elif a.rung.isdigit():
        names = [NAMES[int(a.rung)]]
    elif a.rung in NAMES or a.rung == "udf":
        names = [a.rung]
    else:
        print("unknown rung %r; choose from %s" % (a.rung, NAMES + ["udf"])); return 1
    try:
        lad = Ladder(a)
    except Exception as e:  # noqa: BLE001
        print("cannot open the device: %r" % (e,)); return 1
    for n in names:
        if not lad.run(n):
            print("stopped at rung '%s'. If the device rebooted, this rung's [plan] line names the primitive that killed it." % n)
            return 2
        time.sleep(0.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
