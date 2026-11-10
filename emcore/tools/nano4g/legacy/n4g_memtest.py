#!/usr/bin/env python3
"""SDRAM test over USB on a BOOTED nano4g emCORE (round 13): the pad96 experiment proved that moving the
data/bss addresses by ~100 bytes, with no code change, reboots the device. One explanation is marginal SDRAM at specific
addresses (the SRAM bootstub never issues the EMRS/second MRS the stock PreEfi does). This writes patterns
into memory the running image does not use and reads them back, forcing real DRAM traffic (regions >> the 16 KiB D-cache).

Regions:
  * a heap block obtained with the device's own malloc (default 16 MiB, halves on failure) -> freed afterwards
  * the free tail between the image's bss end and the LCD framebuffer (0x09f9a000..0x09fd4000)
Patterns: address^key, 0x00000000, 0xffffffff, 0xa5a5a5a5 (write the whole region, then read the whole region).
Read-only for the device state: it never touches anything the kernel owns. Safe to run repeatedly.
Usage: python tools/n4g_memtest.py [--mib 16] [--chunk 65536] [--quick]
"""
import argparse
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

TAIL_START = 0x09f9a000
TAIL_END = 0x09fd4000
PATTERNS = [("addr^5aa5f00f", None), ("zeros", 0x00000000), ("ones", 0xffffffff), ("a5a5a5a5", 0xa5a5a5a5)]


def pattern_bytes(base, size, fixed):
    if fixed is not None:
        return struct.pack("<I", fixed) * (size // 4)
    words = [((base + 4 * i) ^ 0x5aa5f00f) & 0xffffffff for i in range(size // 4)]
    return struct.pack("<%dI" % len(words), *words)


def test_region(em, name, base, size, chunk, patterns, out):
    bad_total = 0
    for pname, fixed in patterns:
        t0 = time.time()
        for off in range(0, size, chunk):
            n = min(chunk, size - off)
            em.write(base + off, pattern_bytes(base + off, n, fixed))
        t1 = time.time()
        bad = 0
        first = []
        hist = {}
        for off in range(0, size, chunk):
            n = min(chunk, size - off)
            got = em.read(base + off, n)
            exp = pattern_bytes(base + off, n, fixed)
            if got == exp:
                continue
            gw = struct.unpack("<%dI" % (n // 4), got)
            ew = struct.unpack("<%dI" % (n // 4), exp)
            for i, (g, e) in enumerate(zip(gw, ew)):
                if g != e:
                    bad += 1
                    a = base + off + 4 * i
                    hist[a >> 20] = hist.get(a >> 20, 0) + 1
                    if len(first) < 12:
                        first.append((a, e, g))
        t2 = time.time()
        line = "  %-10s %-12s %8u KiB  write %4.1fs read %4.1fs  bad words: %u" % (name, pname, size >> 10, t1 - t0, t2 - t1, bad)
        print(line); out and out.write(line + "\n")
        for a, e, g in first:
            line = "      %08x expected %08x got %08x (xor %08x)" % (a, e, g, e ^ g)
            print(line); out and out.write(line + "\n")
        if hist:
            line = "      per MiB: " + " ".join("%08x:%u" % (k << 20, v) for k, v in sorted(hist.items()))
            print(line); out and out.write(line + "\n")
        bad_total += bad
    return bad_total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mib", type=int, default=16, help="heap block size to test (halved until malloc succeeds)")
    ap.add_argument("--chunk", type=int, default=65536)
    ap.add_argument("--quick", action="store_true", help="only the address pattern")
    ap.add_argument("--out", default="memtest.txt")
    a = ap.parse_args()
    import libemcore
    try:
        em = libemcore.Emcore()
    except Exception as e:  # noqa: BLE001
        print("cannot open the device: %r" % (e,)); return 1
    out = open(a.out, "w")
    patterns = PATTERNS[:1] if a.quick else PATTERNS
    total = 0
    print("== free tail below the framebuffer")
    total += test_region(em, "tail", TAIL_START, TAIL_END - TAIL_START, a.chunk, patterns, out)
    mib = a.mib
    ptr = None
    while mib >= 1 and ptr is None:
        try:
            ptr = em.malloc(mib << 20)
            if ptr == 0:
                ptr = None
        except Exception:  # noqa: BLE001
            ptr = None
        if ptr is None:
            mib //= 2
    if ptr is None:
        print("malloc failed for every size"); out.close(); return 2
    print("== heap block %u MiB at %08x" % (mib, ptr))
    total += test_region(em, "heap", ptr, mib << 20, a.chunk, patterns, out)
    try:
        em.free(ptr)
    except Exception as e:  # noqa: BLE001
        print("free failed: %r" % (e,))
    verdict = "SDRAM test: %s (%u bad words)" % ("CLEAN" if total == 0 else "ERRORS", total)
    print(verdict); out.write(verdict + "\n"); out.close()
    print("report written to %s" % a.out)
    return 0 if total == 0 else 3


if __name__ == "__main__":
    sys.exit(main())
