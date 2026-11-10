#!/usr/bin/env python3
"""On-device SDRAM test for a BOOTED nano4g emCORE (round 13), USB-light version: the first memtest streamed every
pattern over USB and the bulk writes wedged the reads (a known USB-stack problem of this emCORE build). This one uploads a
264-byte ARM routine (tools/memtest/memtest_arm.S, assembled blob below) with 64-byte control transfers, starts it as an
emCORE system thread and polls a 56-byte parameter block with small control reads. All memory traffic is on the device.

Per region the routine writes 4 patterns (addr^key, zeros, ones, 0xa5a5a5a5), clean+invalidates the D-cache, reads back,
counts mismatches and records the first one. Regions: the free tail below the LCD framebuffer (0x09f9a000..0x09fd4000) and a
heap block from the device's own malloc (default 16 MiB, halved until it succeeds). Nothing the kernel owns is touched.
Usage: python tools/n4g_memtest2.py [--mib 16] [--out memtest2.txt] [--tail-start 0x09f9a000]
"""
import argparse
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

BLOB_PATH = os.path.join(HERE, "memtest", "memtest_arm.bin")
TAIL_START = 0x09f9a000
TAIL_END = 0x09fd4000
PARAMS_WORDS = 14          # base size npat progress bad_total first_addr first_exp first_got done pass0..3 passes_done
PATTERN_NAMES = ["addr^5aa5f00f", "zeros", "ones", "a5a5a5a5"]


class Dev:
    def __init__(self):
        import libemcore
        self.em = libemcore.Emcore()

    def write_small(self, addr, data):
        """64-byte EP0 control writes only (bulk OUT wedges this build; >64 B EP0 writes trip EPROTO)."""
        for off in range(0, len(data), 64):
            self.em._writemem(addr + off, data[off:off + 64])

    def read_small(self, addr, size):
        return self.em._readmem(addr, size)

    def create_thread(self, name_ptr, entry, stack, stacksize, prio, arg0):
        cmd = struct.pack("<12I", 19, name_ptr, entry, stack, stacksize, 1, prio, 2, arg0, 0, 0, 0)
        return self.em.lib.monitorcommand(cmd, "III", ("threadptr", None, None)).threadptr


def run_region(dev, blk, name, base, size, out, poll=0.5, timeout=900):
    code, params, stack = blk, blk + 0x400, blk + 0x800
    dev.write_small(params, struct.pack("<%dI" % PARAMS_WORDS, base, size, 4, *([0] * (PARAMS_WORDS - 3))))
    dev.write_small(blk + 0x3c0, b"memtest\0")
    dev.em.flushcaches()
    t0 = time.time()
    tp = dev.create_thread(blk + 0x3c0, code, stack, 0x800, 32, params)
    line = "== %s: %08x..%08x (%u KiB), thread %08x" % (name, base, base + size, size >> 10, tp & 0xffffffff)
    print(line); out.write(line + "\n")
    last = -1
    while True:
        time.sleep(poll)
        p = struct.unpack("<%dI" % PARAMS_WORDS, dev.read_small(params, 4 * PARAMS_WORDS))
        done, passes, progress = p[8], p[13], p[3]
        if (passes, progress >> 16) != last:
            last = (passes, progress >> 16)
            sys.stdout.write("\r   pass %u/4  %6u KiB read back  bad so far %u   " % (min(passes + 1, 4), progress >> 10, p[4]))
            sys.stdout.flush()
        if done:
            break
        if time.time() - t0 > timeout:
            line = "\n   TIMEOUT after %.0f s (done flag never set): thread hung or died; last state %s" % (time.time() - t0, p)
            print(line); out.write(line + "\n")
            return None
    print()
    for i, n in enumerate(PATTERN_NAMES):
        line = "   %-14s bad words %u" % (n, p[9 + i])
        print(line); out.write(line + "\n")
    if p[4]:
        line = "   first failure @%08x expected %08x got %08x (xor %08x)" % (p[5], p[6], p[7], p[6] ^ p[7])
        print(line); out.write(line + "\n")
    line = "   total bad words %u in %.1f s" % (p[4], time.time() - t0)
    print(line); out.write(line + "\n")
    return p[4]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mib", type=int, default=16)
    ap.add_argument("--out", default="memtest2.txt")
    ap.add_argument("--tail-start", type=lambda x: int(x, 0), default=TAIL_START)
    ap.add_argument("--no-heap", action="store_true")
    a = ap.parse_args()
    blob = open(BLOB_PATH, "rb").read()
    try:
        dev = Dev()
    except Exception as e:  # noqa: BLE001
        print("cannot open the device: %r" % (e,)); return 1
    out = open(a.out, "w")
    blk = dev.em.malloc(0x1000)
    if blk & 0xff:
        print("warning: code block not 256-aligned (%08x)" % blk)
    dev.write_small(blk, blob)
    line = "routine %u B uploaded at %08x (params +0x400, stack +0x800)" % (len(blob), blk)
    print(line); out.write(line + "\n")
    total = 0
    r = run_region(dev, blk, "tail", a.tail_start, TAIL_END - a.tail_start, out)
    if r is None:
        out.close(); return 2
    total += r
    if not a.no_heap:
        mib, ptr = a.mib, None
        while mib >= 1 and ptr is None:
            try:
                ptr = dev.em.malloc(mib << 20) or None
            except Exception:  # noqa: BLE001
                ptr = None
            if ptr is None:
                mib //= 2
        if ptr is None:
            print("heap block: malloc failed for every size")
        else:
            r = run_region(dev, blk, "heap", ptr, mib << 20, out)
            if r is None:
                out.close(); return 2
            total += r
            try:
                dev.em.free(ptr)
            except Exception as e:  # noqa: BLE001
                print("free failed: %r" % (e,))
    verdict = "SDRAM test: %s (%u bad words)" % ("CLEAN" if total == 0 else "ERRORS", total)
    print(verdict); out.write(verdict + "\n"); out.close()
    print("report written to %s" % a.out)
    return 0 if total == 0 else 3


if __name__ == "__main__":
    sys.exit(main())
