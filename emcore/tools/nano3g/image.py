#!/usr/bin/env python3
"""Package the nano3g SRAM loader and kernel in an unsigned IMG1 "8702" container.

The body contains stage1, alignment padding, a kernel length word and the kernel bytes; the mask ROM's DFU
runs it at 0x22000000 (load address + entry offset 0). Header (docs/nano3g/02, bootrom mem:0x200006dc):

  +0x00 "8702", +0x04 "1.0", +0x07 format (4 = plain body, 3 = the signing step encrypts it)
  +0x08 entry offset 0, +0x0c body length n (16-byte aligned here)
  +0x14 = n + 0x80 (signature = the 0x80 bytes behind the body), +0x18 = certificate length
  +0x10 = +0x14 + +0x18, header 0x800 bytes; the ROM requires +0x10 >= n + 0x300, +0x14 >= n + 0x80,
  +0x14 & 15 == 0 and +0x08 <= n.

This is the layout of the format 4 WTF DFU template. The header value at +0x40, the signature and the
certificate area are left zero (unsigned, like the nano 5G image): the ROM checks them, so the image needs
the signing step or a loader that accepts it.
"""
import argparse
import struct
import subprocess
import sys

HEADER = 0x800
SIG_LEN = 0x80
CERT_LEN = 0x300
STAGE1_BASE = 0x22000000
KERNEL_BASE = 0x08000000
STAGE1_MAX = 0x1000            # stage1.ld ASSERT
BODY_ALIGN = 16                # AES block; the ROM rounds the body length up to it
DFU_MAXSIZE = 0x20000          # largest DFU file known to load on the 3G (Rockbox's DFU tool limit)


def align(n, a):
    return (n + a - 1) // a * a


def nm_symbols(elf, nm="arm-none-eabi-nm"):
    out = subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3:
            syms[p[2]] = int(p[0], 16)
    return syms


def build_payload(stage1, kernel, stage1_end=None):
    """stage1.bin + padding to _stage1_end + {u32 len, kernel padded to 4 bytes}."""
    if len(stage1) > STAGE1_MAX:
        raise ValueError("stage 1 is %d bytes, limit %d" % (len(stage1), STAGE1_MAX))
    end = align(len(stage1), 16)
    if stage1_end is not None and stage1_end != STAGE1_BASE + end:
        raise ValueError("stage1.elf _stage1_end 0x%x != 0x%x derived from stage1.bin" % (stage1_end, STAGE1_BASE + end))
    klen = align(len(kernel), 4)
    return stage1 + b"\0" * (end - len(stage1)) + struct.pack("<I", klen) + kernel + b"\0" * (klen - len(kernel))


def align_body(body):
    """Zero-pad the body to BODY_ALIGN bytes (stage 1 copies only the kernel length it was given)."""
    return body + b"\0" * (align(len(body), BODY_ALIGN) - len(body))


def build_img1(body, fmt=4, cert_len=CERT_LEN, version=b"1.0"):
    """IMG1 8702: 0x800-byte header, body, signature slot, certificate slot (both zero)."""
    if fmt not in (3, 4):
        raise ValueError("format must be 3 or 4")
    if len(body) % BODY_ALIGN:
        raise ValueError("body length 0x%x is not %d-byte aligned; pad it with align_body()" % (len(body), BODY_ALIGN))
    if cert_len < 0x300 - SIG_LEN:
        raise ValueError("certificate slot 0x%x too small: the ROM needs +0x10 >= body + 0x300" % cert_len)
    n = len(body)
    cert_off = n + SIG_LEN
    total = cert_off + cert_len
    if HEADER + total > DFU_MAXSIZE:
        raise ValueError("image is 0x%x bytes, DFU limit 0x%x (body %d bytes, max %d)" %
                         (HEADER + total, DFU_MAXSIZE, n, DFU_MAXSIZE - HEADER - SIG_LEN - cert_len))
    hdr = bytearray(HEADER)
    hdr[0:4] = b"8702"
    hdr[4:7] = version
    hdr[7] = fmt
    struct.pack_into("<IIIII", hdr, 8, 0, n, total, cert_off, cert_len)
    return bytes(hdr) + body + b"\0" * (SIG_LEN + cert_len)


def check_img1(img, body):
    """Re-reads the header with the ROM's rules (docs/nano3g/02) and checks the body placement."""
    if img[0:7] != b"87021.0" or img[7] not in (3, 4):
        raise ValueError("bad magic / format")
    entry, n, total, cert_off, cert_len = struct.unpack_from("<IIIII", img, 8)
    na = align(n, 16)
    if not (total >= na + 0x300 and cert_off >= na + SIG_LEN and cert_off % 16 == 0 and entry <= na
            and total == cert_off + cert_len and len(img) == HEADER + total):
        raise ValueError("header lengths violate the ROM rules")
    if img[HEADER:HEADER + n] != body:
        raise ValueError("body not at file+0x800")


def dfu_crc32(data):
    crc = 0xffffffff
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xedb88320 if crc & 1 else 0)
    return crc


def dfu_suffix(data, vid=0x05ac, pid=0x1223):
    """16-byte DFU suffix: bcdDevice, idProduct, idVendor, bcdDFU 0x0100, 'UFD', length 16, CRC-32."""
    s = struct.pack("<HHHH3sB", 0xffff, pid, vid, 0x0100, b"UFD", 16)
    return data + s + struct.pack("<I", dfu_crc32(data + s))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    ap.add_argument("--stage1", required=True)
    ap.add_argument("--payload", required=True, help="emcore.bin (raw kernel, linked for 0x08000000)")
    ap.add_argument("--stage1-elf")
    ap.add_argument("--kernel-elf")
    ap.add_argument("--raw", help="also write the raw payload (IMG1 body) here")
    ap.add_argument("--dfu-suffix", action="store_true")
    ap.add_argument("--format", type=int, choices=(3, 4), default=4,
                    help="IMG1 format byte: 4 plain body (default), 3 = the signing step encrypts the body")
    ap.add_argument("--cert-len", type=lambda v: int(v, 0), default=CERT_LEN,
                    help="certificate slot reserved for the signing step (default 0x300; Apple's chain is 0xbe3)")
    ap.add_argument("-o", "--output", required=True)
    a = ap.parse_args(argv)

    stage1 = open(a.stage1, "rb").read()
    kernel = open(a.payload, "rb").read()
    s1end = None
    if a.stage1_elf:
        s1end = nm_symbols(a.stage1_elf, a.nm).get("_stage1_end")
    if a.kernel_elf:
        k = nm_symbols(a.kernel_elf, a.nm)
        if "_inittailend" in k and k["_inittailend"] - KERNEL_BASE != len(kernel):
            print("warning: emcore.bin is %d bytes but _inittailend - 0x08000000 = %d" %
                  (len(kernel), k["_inittailend"] - KERNEL_BASE), file=sys.stderr)
    try:
        body = align_body(build_payload(stage1, kernel, s1end))
        img = build_img1(body, a.format, a.cert_len)
        check_img1(img, body)
        if a.dfu_suffix:
            img = dfu_suffix(img)
    except ValueError as e:
        print("n3g image: %s" % e, file=sys.stderr)
        return 1
    if a.raw:
        open(a.raw, "wb").write(body)
    open(a.output, "wb").write(img)
    hdr = struct.unpack_from("<IIIII", img, 8)
    print("stage1 %d B + payload %d B -> body %d B (0x%x), IMG1 8702 %d B, format %d, total 0x%x, "
          "cert offset 0x%x length 0x%x, UNSIGNED -> %s" %
          (len(stage1), len(kernel), len(body), len(body), len(img), img[7], hdr[2], hdr[3], hdr[4], a.output))

    return 0


if __name__ == "__main__":
    sys.exit(main())
