"""M2 experiment primitives, expressed as source assembled at run time.

These are deliberately small, serialized experiments. Geometry describes the
test workload, not M2 architectural limits. Native driver programs are local to
target/ipodnano4g/nand/fmss/.
restore WRITE mem:0x080379e8, READ mem:0x08036688 (see source manifests).
No M1 ECC_MODE, parity-slot layout, status event 5 or channel GPIO is reused.
dma() keeps its historical FMC_DMA_CTRL=7 write (M1 R7 value) for comparability
with p06..p16; the original M2 READ never writes 0x038. Pass control=None to omit
the write. Every dma() waits WAIT2 and acknowledges FMC_STATUS bit 20: p16 shows
waits do not consume that flag, so an unacknowledged completion satisfies the
next WAIT2 immediately.
"""
from dataclasses import dataclass

from fmss.probe import assemble, probe, FULLMAP
from .protocol import Cmd, MUTATE, ToolError
from .nand import check


@dataclass(frozen=True)
class Geometry:
    ce: int = 0
    block: int = 3000
    pages: int = 128
    blocks: int = 4096
    strength: int = 8

    def __post_init__(self):
        if not (0 <= self.ce < 4 and 0 <= self.block < self.blocks and self.strength in (8, 16)):
            raise ValueError("invalid M2 experiment geometry")

    @property
    def row(self):
        return self.block * self.pages

    @property
    def parity(self):
        return 13 if self.strength == 8 else 26

    @property
    def spare(self):
        return 128 if self.strength == 8 else 218


def geometry(s):
    return getattr(s, "geometry", Geometry())


def wr(off, value):
    return "    wr [0x%x], #0x%x\n" % (off, value & 0xffffffff)


def wait(event, arg=0, mask=0):
    return "    wait %d, arg=0x%x, mask=0x%x\n" % (event, arg, mask)


def command(c):
    return wr(8, c) + wait(0) + wr(0x48, 2)


def ready():
    return command(0x70) + wr(4, 0xca) + wait(12, 0x40, 0x40) + wr(4, 0x30020) + wr(0x48, 0x1800000)


def address(row, column=0, cycles=5):
    value = row if cycles in (3, 4) else (row << 16) | column
    return wr(0x2c, cycles - 1) + wr(0xc, value) + wr(0x10, value >> 32) + wr(4, 1) + wait(1) + wr(0x48, 4)


def setup(ce=None):
    # Match READ/WRITE timing without selecting a chip for buffer-only tests.
    return wr(4, 0x33338e0) + wr(0x48, 0xff00ffe) + wr(0, 0x1077801 | (0 if ce is None else 2 << ce))


def dma(addr, slot=0, into=True, size=512, control=7, select=None):
    sel = (1 << slot if into else 0x100 << slot) if select is None else select
    return ((wr(0x38, control) if control is not None else '') + wr(0x30, size - 1) + wr(0x14, sel) + wr(0x34, addr) +
            wr(4, 0x2e0 if into else 0x100) + wait(2) + wr(0x48, 0x100000))


def ecc(slot=0, encode=True, strength=8, metalen=0, extra=0, acknowledge=True):
    kick = (2 if encode else 1) | (4 if strength == 16 else 0) | (metalen << 8) | extra
    return (wr(0x810, 0x1ff) + wr(0x1c, slot) + wr(0x804, kick) +
            (wait(4) + wr(0x810, 1) if acknowledge else ""))


def metadata(data):
    data = data[:10].ljust(12, b"\xff")
    return "".join(wr(0x60 + k, int.from_bytes(data[k:k + 4], "little")) for k in (0, 4, 8)) + wr(0x40, 0xc1)


def transfer(size, slot=0, parity=False, into=True):
    sel = (0x10 if parity else 1) << slot if into else (0x1000 if parity else 0x100) << slot
    return wr(0x30, size - 1) + wr(0x14, sel) + wr(4, 0x32 if parity and into else 0xe2 if into else 0x34 if parity else 0xe4) + wait(3) + wr(0x48, 8)


def store(off, addr, reg=0):
    return "    ld r%d, [0x%x]\n    movi r7, #0x%x\n    sti [r7], r%d\n" % (reg, off, addr, reg)


def read_page(g, row, dest, results, meta, raw=False, ce=None):
    ce = g.ce if ce is None else ce
    src = setup(ce) + command(0) + address(row) + command(0x30) + ready() + command(0)
    if raw:
        for pos in range(0, 4096 + g.spare, 512):
            n = min(512, 4096 + g.spare - pos)
            src += transfer(n) + dma(dest + pos, into=False)
    else:
        for sector in range(8):
            slot = sector & 1
            src += transfer(g.parity + (10 if sector == 0 else 0), slot, parity=True)
            src += transfer(512, slot) + ecc(slot, False, g.strength, 10 if sector == 0 else 0)
            src += store(0x80c, results + sector * 4) + dma(dest + sector * 512, slot, into=False)
            if sector == 0:
                src += wr(0x40, 0xc2)
                src += "meta_wait:\n    ld r0, [0x40], 2\n    bnz r0, meta_wait\n"
                src += "".join(store(0x60 + k, meta + k) for k in (0, 4, 8))
    return src


def write_page(g, row, source, meta, ce=None, raw=False, confirm=0x10):
    ce = g.ce if ce is None else ce
    src = setup(ce) + command(0x80) + address(row)
    if raw:
        for pos in range(0, 4096 + g.spare, 512):
            src += dma(source + pos) + transfer(min(512, 4096 + g.spare - pos), into=False)
    else:
        src += metadata(meta)
        for sector in range(8):
            slot = sector & 1
            src += dma(source + sector * 512, slot) + ecc(slot, True, g.strength, 10 if sector == 0 else 0)
            src += transfer(g.parity + (10 if sector == 0 else 0), slot, parity=True, into=False)
            src += transfer(512, slot, into=False)
    return src + command(confirm) + ready() + "    ld r5, [0x4c]\n"


def erase(g, block=None, ce=None, cycles=3):
    ce = g.ce if ce is None else ce
    row = (g.block if block is None else block) * g.pages
    return setup(ce) + command(0x60) + address(row, cycles=cycles) + command(0xd0) + ready() + "    ld r5, [0x4c]\n"


def run(s, t, body, *, scratch=None, mutate=False, pins=False, **kw):
    if pins and not getattr(s, "m2_pins_ready", False):
        check(s.job(Cmd.NAND_STEP, 2, 0))
        s.m2_pins_ready = True
    if mutate:
        g = geometry(s)
        print("PLAN: mutation enabled; campaign base CE%d block %d (exact targets in request journal)" %
              (g.ce, g.block), flush=True)
    flags = kw.pop("flags", 0) | (MUTATE if mutate else 0)
    src = t.prologue() + body + wr(0, 0) + "    movi r6, #0x600d600d\n    end\n" + t.trap()
    return probe(s, t, assemble(t, src), scratch=scratch, flags=flags, **kw)


def completed(rec):
    if rec["why"] != "END" or rec["r"][6] != 0x600d600d:
        raise ToolError("M2 prerequisite failed: %s, result %08x" % (rec["why"], rec["r"][6]))
    return rec
