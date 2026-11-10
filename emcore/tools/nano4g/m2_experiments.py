"""M2 replacements for experiments that used the M1 NAND/ECC datapath.

Each result retains its exact inputs and raw probe output. Sweep outcomes are
measurements to interpret, not assertions that an M1 rule also holds on M2.
"""
import hashlib
import random
import struct

from fmss.probe import FULLMAP, S_NO_PREFLIGHT, S_NO_CLEAN, S_NO_RESET
from . import sequences as Q
from .protocol import DISCOVERY, ToolError

REGISTRY = {}


def experiment(name):
    def register(fn):
        REGISTRY[name] = fn
        return fn
    return register


def pattern(size=4096, seed=17):
    return bytes((i * 29 + seed) & 255 for i in range(size))


def summary(name, records):
    counts = {}
    for rec in records.values():
        why = rec.get("why", "record")
        counts[why] = counts.get(why, 0) + 1
    return records, "%s: %d cases; %s" % (name, len(records), counts)


def buffer(s):
    return bytearray(0x8000)


def page(s, t, row, ce=None, raw=False, start=0):
    a, g = t.scratch_addr(s), Q.geometry(s)
    rec = Q.run(s, t, Q.read_page(g, row, a + 0x2000, a + 0x3100, a + 0x3000, raw, ce),
                scratch=buffer(s), pins=True, scratch_read=0x3400, start_value=start)
    data = rec["scratch"][0x2000:0x2000 + (4096 + g.spare if raw else 4096)]
    rec["page_sha256"] = hashlib.sha256(data).hexdigest()
    rec["row"], rec["ce"], rec["raw"] = row, g.ce if ce is None else ce, raw
    rec["data_hex"] = data.hex()
    return rec


@experiment("rawpage")
def rawpage(s, t):
    """Read the scratch block's first page raw and corrected; retain every chunk."""
    g = Q.geometry(s)
    return summary("rawpage", {str(raw): page(s, t, g.row, raw=raw) for raw in (True, False)})


@experiment("fullpage")
def fullpage(s, t):
    """Both halves of complete M2 pages: 8 sectors, ten metadata bytes per page."""
    out = {}
    for ce in (0, 1):
        for block in (1, 2, 10, 100, 500, 1000, 2000):
            for p in (0, 1, 64):
                out[f"CE{ce}/B{block}/P{p}"] = page(s, t, block * 128 + p, ce, True)
    return summary("fullpage", out)


def erase_block(s, t, block=None, ce=None, cycles=3, require_success=True):
    g = Q.geometry(s)
    print("PLAN: erase CE%d block %d, %d address cycles" %
          (g.ce if ce is None else ce, g.block if block is None else block, cycles), flush=True)
    rec = Q.completed(Q.run(s, t, Q.erase(g, block, ce, cycles), mutate=True, pins=True))
    if require_success and rec["r"][5] & 1:
        raise ToolError("NAND erase reported FAIL; stop before programming")
    return rec


def program_page(s, t, row, data=None, meta=bytes(range(10)), ce=None, raw=False):
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = buffer(s)
    payload = pattern(4096 + g.spare if raw else 4096) if data is None else data
    print("PLAN: program CE%d row 0x%x, %s data" % (g.ce if ce is None else ce, row,
          "raw" if raw else "ECC-encoded"), flush=True)
    mem[:len(payload)] = payload
    rec = Q.completed(Q.run(s, t, Q.write_page(g, row, a, meta, ce, raw), scratch=mem,
                           mutate=True, pins=True, scratch_read=0))
    rec["row"], rec["ce"] = row, g.ce if ce is None else ce
    if rec["r"][5] & 1:
        raise ToolError("NAND program FAIL on CE%d row %x" % (rec["ce"], row))
    return rec


@experiment("rawwrite")
def rawwrite(s, t):
    """M2 encoded and raw write paths, followed by exact raw/corrected readback."""
    g = Q.geometry(s)
    out = {"erase": erase_block(s, t)}
    for p, raw in enumerate((False, True)):
        out[f"program/{p}"] = program_page(s, t, g.row + p, raw=raw)
        out[f"read-raw/{p}"] = page(s, t, g.row + p, raw=True)
        out[f"read-ecc/{p}"] = page(s, t, g.row + p)
        if not raw:
            out[f"read-ecc/{p}"]["matches_input"] = bytes.fromhex(out[f"read-ecc/{p}"]["data_hex"]) == pattern()
        else:
            out[f"read-raw/{p}"]["matches_input"] = bytes.fromhex(out[f"read-raw/{p}"]["data_hex"]) == pattern(4096 + g.spare)
    out["erased-control"] = page(s, t, g.row + 2, raw=True)
    return summary("rawwrite", out)


@experiment("eraseseq")
def eraseseq(s, t):
    """Three/four-cycle erase, written/erased blocks, and CE0/CE1 controls."""
    g = Q.geometry(s)
    out = {}
    for ce in (g.ce, (g.ce + 1) % 4):
        for cycles in (3, 4):
            key = f"CE{ce}/cycles{cycles}"
            out[key + "/initial"] = erase_block(s, t, ce=ce)
            out[key + "/program"] = program_page(s, t, g.row, ce=ce)
            out[key + "/erase-written"] = erase_block(s, t, ce=ce, cycles=cycles, require_success=cycles == 3)
            out[key + "/readback"] = page(s, t, g.row, ce, True)
            out[key + "/erase-erased"] = erase_block(s, t, ce=ce, cycles=cycles, require_success=cycles == 3)
    return summary("eraseseq", out)


@experiment("pageorder")
def pageorder(s, t):
    """M2 MLC page-order matrix; each schedule starts with an erased block."""
    g = Q.geometry(s)
    out = {}
    for order in (tuple(range(8)), (5,), (4,), (4, 5), (1, 3)):
        key = ",".join(map(str, order))
        out[key + "/erase"] = erase_block(s, t)
        for p in order:
            out[key + f"/program{p}"] = program_page(s, t, g.row + p, data=pattern(seed=p))
        for p in order:
            rec = out[key + f"/read{p}"] = page(s, t, g.row + p)
            rec["matches_input"] = bytes.fromhex(rec["data_hex"]) == pattern(seed=p)
    return summary("pageorder", out)


@experiment("progtime")
def progtime(s, t):
    """Ready-inclusive program times for all 128 pages on both CEs."""
    g, out = Q.geometry(s), {}
    for ce, block in ((g.ce, g.block), (g.ce, g.block + 1), ((g.ce + 1) % 4, g.block)):
        key = f"CE{ce}/B{block}"
        out[key + "/erase"] = erase_block(s, t, block, ce)
        for p in range(128):
            out[key + f"/P{p}"] = program_page(s, t, block * 128 + p, data=pattern(seed=p), ce=ce)
    return summary("progtime", out)


@experiment("bufsize")
def bufsize(s, t):
    """DMA length boundary, both M2 data buffers, with red zones in destination RAM."""
    a, out = t.scratch_addr(s), {}
    for slot in (0, 1):
        for n in (256, 512, 513, 1024, 2048):
            mem = buffer(s)
            mem[:4096] = pattern()
            mem[0x2000:0x4000] = b"\xa5" * 0x2000
            body = Q.setup() + Q.dma(a, slot, size=n) + Q.dma(a + 0x2000, slot, False, n)
            rec = Q.run(s, t, body, scratch=mem, scratch_read=0x4000)
            rec["requested_bytes"], rec["slot"] = n, slot
            out[f"slot{slot}/n{n}"] = rec
    return summary("bufsize", out)


@experiment("dmaloop")
def dmaloop(s, t):
    """Both M2 data buffers and DMA selects; second register set tested separately."""
    a, out = t.scratch_addr(s), {}
    for slot in (0, 1):
        for ctrl in (0, 7):
            for select in (0x100 << slot, 0x1e0):
                mem = buffer(s)
                mem[:512] = pattern(512)
                body = Q.setup() + Q.dma(a, slot, control=ctrl) + Q.dma(a + 0x2000, slot, False, control=ctrl, select=select)
                out[f"slot{slot}/ctrl{ctrl}/sel{select:x}"] = Q.run(s, t, body, scratch=mem, scratch_read=0x2400)
    return summary("dmaloop", out)


def ecc_case(s, t, *, strength=8, errors=0, mode=0, metalen=10, slot=0, decode=True,
             data=None, params=(), flags=FULLMAP, error_kind="data", start=0):
    a = t.scratch_addr(s)
    original = pattern(512) if data is None else data
    damaged = bytearray(original)
    for k in range(errors):
        bit = (k * 37) % 4096
        damaged[bit // 8] ^= 1 << (bit & 7)
    mem = buffer(s)
    mem[:512], mem[0x400:0x600] = original, damaged
    body = Q.setup() + Q.dma(a, slot) + Q.metadata(bytes(range(10)))
    body += Q.ecc(slot, True, strength, metalen, mode)
    body += Q.store(0x810, a + 0x3000) + Q.store(0x80c, a + 0x3004)
    if decode:
        body += Q.dma(a + 0x400, slot) + Q.ecc(slot, False, strength, metalen, mode)
        body += Q.store(0x810, a + 0x3008) + Q.store(0x80c, a + 0x300c)
        body += Q.dma(a + 0x2000, slot, False)
    rec = Q.run(s, t, body, scratch=mem, scratch_read=0x3200, flags=flags, params=params, start_value=start)
    rec["inputs"] = dict(strength=strength, errors=errors, mode=mode, metalen=metalen, slot=slot, decode=decode)
    rec["ecc_words"] = list(struct.unpack_from("<4I", rec["scratch"], 0x3000))
    rec["corrected_matches_input"] = rec["scratch"][0x2000:0x2200] == original if decode else None
    return rec


@experiment("eccpar")
def eccpar(s, t):
    """M2 ECC encode/decode and full-window capture, both strengths and buffers."""
    return summary("eccpar", {f"bch{strength}/slot{slot}/errors{n}": ecc_case(s, t, strength=strength, slot=slot, errors=n)
                              for strength in (8, 16) for slot in (0, 1) for n in (0, 1, 2)})


@experiment("eccstr")
def eccstr(s, t):
    """M2 correction thresholds: every count through strength+4, both buffers."""
    return summary("eccstr", {f"bch{strength}/slot{slot}/errors{n}": ecc_case(s, t, strength=strength, slot=slot, errors=n)
                              for strength in (8, 16) for slot in (0, 1) for n in range(strength + 5)})


@experiment("eccstr2")
def eccstr2(s, t):
    """Single-program thresholds, plus zero/single-bit inputs for fitting the M2 code."""
    out, _ = eccstr(s, t)
    for strength in (8, 16):
        for bit in (-1, *range(16), *range(4080, 4096)):
            data = bytearray(512)
            if bit >= 0:
                data[bit // 8] ^= 1 << (bit & 7)
            out[f"bch{strength}/bit{bit}"] = ecc_case(s, t, strength=strength, data=data, decode=False)
    return summary("eccstr2", out)


@experiment("eccmode")
def eccmode(s, t):
    """M2 ECC_CTRL candidates: b2 strength, metadata lengths, higher unknown bits."""
    out = {}
    for strength in (8, 16):
        for length in (0, 3, 5, 10, 12, 31):
            for n in (0, strength, strength + 1):
                out[f"bch{strength}/len{length}/errors{n}"] = ecc_case(s, t, strength=strength, metalen=length, errors=n)
    return summary("eccmode", out)


@experiment("eccmode2")
def eccmode2(s, t):
    """Encode-only ECC_CTRL field sweep, with full register dumps per candidate."""
    out = {}
    for strength in (8, 16):
        for bit in (-1, *range(3, 8), *range(16, 32)):
            mode = 0 if bit < 0 else 1 << bit
            for name, data in (("zero", bytes(512)), ("ramp", pattern(512)), ("bit0", b"\x01" + bytes(511))):
                out[f"bch{strength}/bit{bit}/{name}"] = ecc_case(s, t, strength=strength, mode=mode, decode=False, data=data)
    return summary("eccmode2", out)


@experiment("eccmode3")
def eccmode3(s, t):
    """Metadata-length boundary and high length bits in M2 ECC_CTRL bits 15:8."""
    out = {}
    for strength in (8, 16):
        for length in (0, 3, 5, 10, 12, 31, 32, 63, 127, 255):
            for decode in (False, True):
                out[f"bch{strength}/len{length}/decode{decode}"] = ecc_case(s, t, strength=strength, metalen=length, decode=decode)
    return summary("eccmode3", out)


@experiment("eccregs")
def eccregs(s, t):
    """Candidate ECC area controls: capture encode/decode effects, restore per run."""
    out = {}
    for off in range(0x940, 0x954, 4):
        for value in (0, 0xffffffff, 0x55555555):
            out[f"{off:x}/{value:x}"] = ecc_case(s, t, errors=8, params=[(off, value)])
    return summary("eccregs", out)


@experiment("done5")
def done5(s, t):
    """M2 ECC_STATUS bits around encode/decode as ECC_CTRL fields vary."""
    out = {}
    for bit in range(3, 32):
        # M2 bits 15:8 specify metadata length; record the tested word exactly.
        for decode in (False, True):
            out[f"bit{bit}/decode{decode}"] = ecc_case(s, t, mode=1 << bit, decode=decode)
    return summary("done5", out)


@experiment("done5b")
def done5b(s, t):
    """Repeat high-bit encode/decode status comparison three times per condition."""
    return summary("done5b", {f"bit{bit}/decode{decode}/rep{rep}": ecc_case(s, t, mode=1 << bit, decode=decode)
                              for bit in range(21, 25) for decode in (False, True) for rep in range(3)})


@experiment("readev")
def readev(s, t):
    """Every WAIT event after each M2 read step, including masked ready event 12."""
    a, g, out = t.scratch_addr(s), Q.geometry(s), {}
    stages = [("command", Q.command(0)), ("address", Q.address(g.row)), ("confirm", Q.command(0x30)),
              ("ready", Q.ready() + Q.command(0)), ("parity", Q.transfer(g.parity + 10, parity=True)),
              ("data", Q.transfer(512)), ("ecc", Q.ecc(0, False, g.strength, 10)), ("dma", Q.dma(a, into=False))]
    src = Q.setup(g.ce)
    for name, step in stages:
        src += step
        for ev in range(16):
            rec = Q.run(s, t, src + Q.wait(ev, 0x40 if ev == 12 else 0, 0x40 if ev == 12 else 0),
                        pins=True, scratch=buffer(s))
            out[f"{name}/wait{ev}"] = rec
    return summary("readev", out)


@experiment("evclear")
def evclear(s, t):
    """M2 status/NSTAT read-to-clear question for command, DMA, ECC and latch."""
    a, g, out = t.scratch_addr(s), Q.geometry(s), {}
    states = [("command", Q.wr(8, 0x70), 0), ("DMA", Q.dma(a).replace(Q.wr(0x48, 0x100000), ""), 2),
              ("ECC", Q.ecc(acknowledge=False), 4), ("status", Q.command(0x70) + Q.wr(4, 0xca), 12)]
    for name, body, ev in states:
        for off in (None, 0x48, 0x4c):
            src = Q.setup(g.ce) + body + ("" if off is None else f"    ld r2, [0x{off:x}]\n")
            src += Q.wait(ev, 0x40 if ev == 12 else 0, 0x40 if ev == 12 else 0)
            out[f"{name}/{off}"] = Q.run(s, t, src, pins=True, scratch=buffer(s))
    return summary("evclear", out)


@experiment("access")
def access(s, t):
    """CPU and direct sequencer access masks across M2; action words excluded."""
    # Do not inherit M1's exclusion list: +804 starts ECC, +40 starts FIFO,
    # +c60 resets buffers, and +c6c changes the handshake on M2.
    skip = {0, 4, 8, 0x38, 0x40, 0x48, 0x60, 0x64, 0x68, 0x7c, 0x80,
            0x804, 0x810, 0x81c, 0xc00, 0xc04, 0xc0c, 0xc38, 0xc4c, 0xc58, 0xc5c, 0xc60, 0xc64, 0xc6c}
    skip |= set(range(0xc18, 0xc38, 4)) | set(range(0x400, 0x800, 4))
    out = {}
    for off in range(0, 0x1000, 4):
        if off in skip:
            continue
        baseline = Q.run(s, t, f"    ld r0, [0x{off:x}]\n", flags=FULLMAP)
        old = baseline["fullmap"][1][off // 4]
        for value in (0xffffffff, 0):
            for via in ("cpu", "sequencer"):
                body = (Q.wr(off, value) if via == "sequencer" else "") + f"    ld r0, [0x{off:x}]\n"
                # Restore CPU's value within the program, after the measured read.
                body += Q.wr(off, old)
                out[f"{off:03x}/{value:08x}/{via}"] = Q.run(s, t, body,
                    params=[(off, value)] if via == "cpu" else (), flags=FULLMAP)
    out["excluded_offsets"] = {"offsets": sorted(skip), "why": "action or unqualified channel words"}
    return summary("access", out)
