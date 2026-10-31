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


"""The boot flash (SPI NOR behind SPI0): what is on it, and a full dump. Read-only.

  n3g.py nor info [--show-values]     size, which 64 KiB regions hold data, the SysCfg tags, the file directory
  n3g.py nor dump -o FILE [--size N]  the whole chip (size found by address wrap unless given), with its SHA-256

Reads through emCORE's boot-flash read (monitor command 22) into the NAND worker's result buffer, then over USB.
The SysCfg block (offset 0) can hold unit-specific values (serial number and the like): info lists the tags and their
sizes only, unless --show-values; a dump contains everything.
"""
import argparse
import hashlib
import struct
import sys

from . import nand
from .nand import ToolError

SYSCFG_MAGIC = 0x53436667          # "SCfg"
DIR_OFFSET = 0x1000                # file directory: 256 entries of 16 bytes (name[8], offset, size), as emCORE reads it
MAX_SIZE = 16 << 20                # 24-bit addresses
CHUNK = 64 << 10


def nor_read(s, offset, size, progress=False):
    st = s.status()
    step = min(CHUNK, st.result_cap)
    out = bytearray()
    while len(out) < size:
        n = min(step, size - len(out))
        s.em.bootflashread(st.result_addr, offset + len(out), n)
        out += s.read(st.result_addr, n)
        if progress and len(out) % (1 << 20) == 0:
            print("  ... %d KiB" % (len(out) >> 10), file=sys.stderr)
    return bytes(out)


def find_size(s):
    """the chip repeats beyond its size (the address wraps): the first power of two from 64 KiB whose 8 KiB match
    offset 0 is the size; None when the start is blank (nothing to compare) or no wrap below 16 MiB"""
    head = nor_read(s, 0, 0x2000)
    if head == b"\xff" * len(head):
        return None, head
    size = CHUNK
    while size <= MAX_SIZE:
        if nor_read(s, size, len(head)) == head:
            return size, head
        size <<= 1
    return None, head


def tag_name(tag):
    b = struct.pack(">I", tag)
    return b.decode("ascii") if all(0x20 <= c < 0x7f for c in b) else "%08x" % tag


def syscfg(head, show_values):
    w = struct.unpack_from("<6I", head, 0)
    if w[0] != SYSCFG_MAGIC or w[1] > len(head) or w[5] * 0x14 + 0x18 != w[1]:
        return ["SysCfg: none at offset 0 (first word %08x)" % w[0]]
    lines = ["SysCfg: %d entries, %d bytes" % (w[5], w[1])]
    for i in range(w[5]):
        off = 0x18 + i * 0x14
        tag = struct.unpack_from("<I", head, off)[0]
        data = head[off + 4:off + 0x14]
        used = len(data.rstrip(b"\0"))
        line = "  %s  %2d bytes used" % (tag_name(tag), used)
        if show_values:
            text = data.split(b"\0", 1)[0]
            line += "  %s%s" % (data.hex(), "  \"%s\"" % text.decode("ascii") if text and all(
                0x20 <= c < 0x7f for c in text) else "")
        lines.append(line)
    return lines


def directory(head):
    lines = []
    for i in range(0x100):
        e = head[DIR_OFFSET + 16 * i:DIR_OFFSET + 16 * i + 16]
        if len(e) < 16 or e == b"\xff" * 16 or e == bytes(16):
            break
        name = e[:8]
        off, size = struct.unpack_from("<II", e, 8)
        shown = name.decode("ascii") if all(0x20 <= c < 0x7f for c in name) else name.hex()
        lines.append("  %-8s  offset 0x%06x  size 0x%06x (%d)" % (shown, off, size, size))
    return ["file directory at 0x%x: %s" % (DIR_OFFSET, "%d entries" % len(lines) if lines else "empty")] + lines


def regions(data):
    """64 KiB regions: '#' holds data, '.' blank (all 0xff)"""
    m = "".join("." if data[o:o + CHUNK] == b"\xff" * len(data[o:o + CHUNK]) else "#" for o in range(0, len(data), CHUNK))
    used = sum(len(data[o:o + CHUNK].rstrip(b"\xff")) > 0 for o in range(0, len(data), CHUNK))
    return ["64 KiB regions with data: %d of %d" % (used, len(m)), "  " + m]


def info(s, show_values=False):
    size, head = find_size(s)
    if size is None and head == b"\xff" * len(head):
        print("boot flash: the first 8 KiB are blank (0xff); size not found by wrap")
        size = 1 << 20
        print("checking the first %d KiB" % (size >> 10))
    elif size is None:
        print("boot flash: no address wrap below %d MiB" % (MAX_SIZE >> 20))
        size = MAX_SIZE
    else:
        print("boot flash size: %d KiB (the start repeats at 0x%x)" % (size >> 10, size))
    data = nor_read(s, 0, size, progress=size > (1 << 20))
    for line in regions(data) + syscfg(data, show_values) + directory(data):
        print(line)
    return data


def dump(s, path, size=None):
    if size is None:
        size, head = find_size(s)
        if size is None:
            raise ToolError("size not found by address wrap (blank start?): give --size")
    data = nor_read(s, 0, size, progress=True)
    with open(path, "wb") as f:
        f.write(data)
    print("wrote %s: %d bytes, sha256 %s" % (path, len(data), hashlib.sha256(data).hexdigest()))
    return data


def main(argv=None):
    ap = argparse.ArgumentParser(prog="n3g.py nor", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("info", help="size, used regions, SysCfg tags, file directory")
    p.add_argument("--show-values", action="store_true", help="also print the SysCfg values (unit-specific data)")
    p = sub.add_parser("dump", help="the whole chip to a file")
    p.add_argument("-o", "--output", required=True)
    p.add_argument("--size", type=lambda x: int(x, 0), help="bytes (default: found by address wrap)")
    for q in sub.choices.values():
        q.add_argument("--settle", type=float, default=0.02)
        q.add_argument("--poll", type=float, default=0.02)
    a = ap.parse_args(argv)
    s = None
    try:
        s = nand.make_dev_session(settle=a.settle, poll=a.poll)
        if a.cmd == "info":
            info(s, a.show_values)
        else:
            dump(s, a.output, a.size)
        return 0
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()


if __name__ == "__main__":
    sys.exit(main())
