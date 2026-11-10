#!/usr/bin/env python3
"""Package the nano4g bootstub and kernel in an unsigned IMG1 8720 container.

S5L8720 uses a 0x600-byte header and version 2.0 (nano3g uses 0x800
and 1.0). validate_header/validate_image are at mem:0x200005c4 /
mem:0x200006f4.
The signature and certificate slots are zero, as in the nano3g packager.
"""
import argparse
from pathlib import Path
import struct
import sys

HEADER = 0x600
SIG_LEN = 0x80
CERT_LEN = 0x300
DATALEN_MAX = 0x24c00
BODY_MAX = 122880


def build_payload(stub, kernel):
    """Keep the working stub ABI: length word immediately after the stub.

    Unlike nano3g stage1, this stub has no padding before the length word.
    Only the end of the complete IMG1 body is padded to an AES block.
    """
    if not stub or len(stub) % 4 or not kernel or len(kernel) % 4:
        raise ValueError("bootstub and kernel must be nonempty and word-aligned")
    body = stub + struct.pack("<I", len(kernel)) + kernel
    return body.ljust((len(body) + 15) & ~15, b"\0")


def build_img1(body, fmt=4, cert_len=CERT_LEN, max_bytes=BODY_MAX):
    if fmt not in (3, 4):
        raise ValueError("format must be 3 or 4")
    if not body or len(body) % 16:
        raise ValueError("body must be nonempty and 16-byte aligned")
    if len(body) > max_bytes:
        raise ValueError("body is %d bytes, above the build cap of %d" % (len(body), max_bytes))
    if cert_len < 0x300 - SIG_LEN:
        raise ValueError("certificate slot must be at least 0x280 bytes")
    cert_off = len(body) + SIG_LEN
    datalen = cert_off + cert_len
    if datalen > DATALEN_MAX:
        raise ValueError("data length 0x%x exceeds the ROM limit 0x%x" % (datalen, DATALEN_MAX))
    hdr = bytearray(HEADER)
    hdr[:7] = b"87202.0"
    hdr[7] = fmt
    struct.pack_into("<5I", hdr, 8, 0, len(body), datalen, cert_off, cert_len)
    return bytes(hdr) + body + bytes(SIG_LEN + cert_len)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--stage1", required=True, type=Path, help="retained nano4g bootstub.bin")
    ap.add_argument("--payload", required=True, type=Path, help="raw emcore.bin kernel")
    ap.add_argument("--raw", type=Path, help="also write the raw IMG1 body")
    ap.add_argument("--format", type=int, choices=(3, 4), default=4,
                    help="4 = plaintext; 3 = a subsequent signing step must encrypt the body")
    ap.add_argument("--cert-len", type=lambda v: int(v, 0), default=CERT_LEN)
    ap.add_argument("--max-bytes", type=lambda v: int(v, 0), default=BODY_MAX)
    ap.add_argument("-o", "--output", required=True, type=Path)
    a = ap.parse_args(argv)
    try:
        stub, kernel = a.stage1.read_bytes(), a.payload.read_bytes()
        body = build_payload(stub, kernel)
        img = build_img1(body, a.format, a.cert_len, a.max_bytes)
        if a.raw:
            a.raw.write_bytes(body)
        a.output.write_bytes(img)
    except (OSError, ValueError) as e:
        print("n4g image: %s" % e, file=sys.stderr)
        return 1
    print("bootstub %d B + payload %d B -> body %d B, IMG1 8720 %d B, "
          "header 0x600, format %d, UNSIGNED -> %s" %
          (len(stub), len(kernel), len(body), len(img), a.format, a.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
