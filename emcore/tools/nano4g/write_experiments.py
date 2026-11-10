"""M2 flash/ECC follow-ups. All target rows derive from the selected geometry.

M1 Reed-Solomon mode values are NOT copied into ECC_CTRL: that word contains
M2's metadata length. RS-labelled entries are applicability tests and remain
incomplete until an M2 alternate-code selector is identified on hardware.
"""
import hashlib
import random

from fmss.probe import FULLMAP
from . import sequences as Q
from .m2_experiments import experiment, summary, buffer, pattern, page, program_page, erase_block, ecc_case


@experiment("writeseq")
def writeseq(s, t):
    """M2 page sequences on CE0/CE1, one program containing multiple pages."""
    g, a, out = Q.geometry(s), t.scratch_addr(s), {}
    for ces in ((g.ce,) * 4, (g.ce, (g.ce + 1) % 4)):
        for ce in sorted(set(ces)):
            out[f"{ces}/erase{ce}"] = erase_block(s, t, ce=ce)
        mem = buffer(s)
        mem[:4096] = pattern()
        body = "".join(Q.write_page(g, g.row + i, a, bytes(range(10)), ce) for i, ce in enumerate(ces))
        out[f"{ces}/program"] = Q.completed(Q.run(s, t, body, scratch=mem, mutate=True, pins=True))
        for i, ce in enumerate(ces):
            rec = out[f"{ces}/read{i}"] = page(s, t, g.row + i, ce)
            rec["matches_input"] = bytes.fromhex(rec["data_hex"]) == pattern()
    out["scope"] = {"pipeline": "serialized M2 baseline; overlapped/two-plane cases are not yet ported"}
    return summary("writeseq", out)


@experiment("writeseq2")
def writeseq2(s, t):
    """Eight sequential M2 pages and repeated programming with status capture."""
    g, a, out = Q.geometry(s), t.scratch_addr(s), {}
    out["erase"] = erase_block(s, t)
    mem = buffer(s)
    mem[:4096] = pattern()
    out["eight-pages"] = Q.completed(Q.run(s, t, "".join(Q.write_page(g, g.row + p, a, bytes(range(10)))
                                     for p in range(8)), scratch=mem, mutate=True, pins=True))
    for p in range(8):
        out[f"read{p}"] = page(s, t, g.row + p)
    # Do not reject FAIL here: it is the result under investigation.
    mem[:4096] = b"\xff" * 4096
    out["repeat-program"] = Q.run(s, t, Q.write_page(g, g.row + 7, a, bytes(10)), scratch=mem, mutate=True, pins=True)
    out["repeat-read"] = page(s, t, g.row + 7)
    return summary("writeseq2", out)


@experiment("ev5two")
def ev5two(s, t):
    """M2 status-ready polling across CEs after page programs, WAIT 5 versus 12."""
    g, a, out = Q.geometry(s), t.scratch_addr(s), {}
    mem = buffer(s)
    mem[:4096] = pattern()
    for ev, arg, mask in ((5, 0, 0), (12, 0x40, 0x40)):
        ces = (g.ce, (g.ce + 1) % 4)
        for ce in ces:
            out[f"wait{ev}/erase{ce}"] = erase_block(s, t, ce=ce)
        body = ""
        for ce in ces:
            body += Q.write_page(g, g.row, a, bytes(10), ce).removesuffix(Q.ready() + "    ld r5, [0x4c]\n")
        for ce in ces:
            body += Q.wr(0, 0x1077801 | 2 << ce) + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(ev, arg, mask)
        out[f"wait{ev}/program-poll"] = Q.run(s, t, body, scratch=mem, mutate=True, pins=True)
    return summary("ev5two", out)


@experiment("startregion")
def startregion(s, t):
    """Start-word pairs across user/reserved reads; mutate only declared scratch blocks."""
    g, out = Q.geometry(s), {}
    for ce in (g.ce, (g.ce + 1) % 4):
        for block in (0, 1, g.block, 3871, 3872, 4095):
            for start in (0xfff5, 0xffb5, 0xffd5, 0xff95):
                out[f"CE{ce}/B{block}/{start:x}"] = page(s, t, block * 128, ce, start=start)
    # Unlike M1, 'first erased reserved block' is not assumed disposable on M2.
    # The selected scratch block receives the program-start comparison.
    a, mem = t.scratch_addr(s), buffer(s)
    mem[:4096] = pattern()
    for start in (0xfff5, 0xffb5):
        out[f"scratch/{start:x}/erase"] = erase_block(s, t)
        out[f"scratch/{start:x}/write"] = Q.run(s, t, Q.write_page(g, g.row, a, bytes(10)),
                                                scratch=mem, mutate=True, pins=True, start_value=start)
        for rd in (0xfff5, 0xffb5):
            out[f"scratch/{start:x}/read{rd:x}"] = page(s, t, g.row, start=rd)
    return summary("startregion", out)


def read_variants(s, t, name, controls):
    g, a, out = Q.geometry(s), t.scratch_addr(s), {}
    out["erase"] = erase_block(s, t)
    out["program"] = program_page(s, t, g.row)
    for label, changes in controls:
        src = Q.read_page(g, g.row, a + 0x2000, a + 0x3100, a + 0x3000)
        for before, after in changes:
            src = src.replace(before, after)
        rec = out[label] = Q.run(s, t, src, scratch=buffer(s), scratch_read=0x3400, pins=True)
        rec["matches_input"] = rec["scratch"][0x2000:0x3000] == pattern()
    return summary(name, out)


@experiment("dmactl")
def dmactl(s, t):
    """Each DMA_CTRL bit and M2 drain select on a known written page."""
    return read_variants(s, t, "dmactl", [(f"ctrl{ctrl:x}/select{select:x}",
        [(Q.wr(0x38, 7), Q.wr(0x38, ctrl)), (Q.wr(0x14, 0x100), Q.wr(0x14, select))])
        for ctrl in (0, 7, *(1 << b for b in range(8))) for select in (0x100, 0x1e0, 0x102, 0x201)])


@experiment("fmcbits")
def fmcbits(s, t):
    """FMC_CTRL0 b9/b11/b24 and drain/DMA-control variants on a known page."""
    g = Q.geometry(s)
    normal = 0x1077801 | 2 << g.ce
    variants = [(f"CTRL0/{word:x}", [(Q.wr(0, normal), Q.wr(0, word))])
                for word in (normal, normal | 0x200, normal & ~0x800, normal & ~0x1000000)]
    variants += [(f"DMA/{word:x}", [(Q.wr(0x38, 7), Q.wr(0x38, word))]) for word in (0, 7, 0xff)]
    return read_variants(s, t, "fmcbits", variants)


@experiment("bufport")
def bufport(s, t):
    """Probe candidate FIFO views of M2 buffers; keep +60..68 separate from +80."""
    a, out = t.scratch_addr(s), {}
    for slot in (0, 1):
        for cmd in (0, 0x52, 0xc2):
            for port in (0x60, 0x64, 0x68, 0x80):
                mem = buffer(s)
                mem[:512] = pattern(512)
                body = Q.setup() + Q.dma(a, slot) + (Q.wr(0x40, cmd) if cmd else "")
                body += "".join(Q.store(port, a + 0x2000 + i * 4) for i in range(130))
                out[f"slot{slot}/cmd{cmd:x}/port{port:x}"] = Q.run(s, t, body, scratch=mem, scratch_read=0x2300)
    return summary("bufport", out)


@experiment("eccchunk")
def eccchunk(s, t):
    """Capture M2 engine-generated codewords from flash, including meta ordering."""
    g, out = Q.geometry(s), {}
    out["erase"] = erase_block(s, t)
    out["encode-write"] = program_page(s, t, g.row, meta=bytes(range(10)))
    out["raw-codewords"] = page(s, t, g.row, raw=True)
    out["decode"] = page(s, t, g.row)
    out["layout"] = {"sectors": 8, "metadata_bytes": 10, "parity_bytes": g.parity,
                     "first_codeword_bytes": 10 + g.parity + 512, "remaining_codeword_bytes": g.parity + 512}
    return summary("eccchunk", out)


@experiment("eccchunk2")
def eccchunk2(s, t):
    """Corrupt captured M2 codewords in metadata/parity/data, write raw, decode."""
    g, out = Q.geometry(s), {}
    out["initial-erase"] = erase_block(s, t)
    out["encode"] = program_page(s, t, g.row)
    rec = out["reference-raw"] = page(s, t, g.row, raw=True)
    Q.completed(rec)
    raw = bytes.fromhex(rec["data_hex"])
    for area, start, size in (("meta", 0, 10), ("parity", 10, g.parity), ("data", 10 + g.parity, 512)):
        for count in (0, 1, 2, g.strength, g.strength + 1):
            damaged = bytearray(raw)
            for k in range(count):
                bit = (37 * k) % (size * 8)
                damaged[start + bit // 8] ^= 1 << (bit & 7)
            key = f"{area}/errors{count}"
            out[key + "/erase"] = erase_block(s, t)
            out[key + "/raw-program"] = program_page(s, t, g.row, damaged, raw=True)
            out[key + "/decode"] = page(s, t, g.row)
    return summary("eccchunk2", out)


@experiment("miscorr")
def miscorr(s, t):
    """Beyond-strength M2 errors, 240 sectors per count, 12 sectors per upload."""
    a, out, rng = t.scratch_addr(s), {}, random.Random(0x4d32)
    for strength in (8, 16):
        for count in range(strength + 1, 65):
            for batch in range(20):
                mem, src, samples = buffer(s), Q.setup(), []
                for index in range(12):
                    rep, offset = batch * 12 + index, index * 1536
                    original = pattern(512, rep)
                    damaged = bytearray(original)
                    bits = rng.sample(range(4096), count)
                    for bit in bits:
                        damaged[bit // 8] ^= 1 << (bit & 7)
                    mem[offset:offset + 512] = original
                    mem[offset + 512:offset + 1024] = damaged
                    src += Q.dma(a + offset) + Q.ecc(strength=strength)
                    src += Q.dma(a + offset + 512) + Q.ecc(encode=False, strength=strength)
                    src += Q.store(0x80c, a + 0x6000 + index * 8)
                    src += Q.store(0x810, a + 0x6004 + index * 8)
                    src += Q.dma(a + offset + 1024, into=False)
                    samples.append({"sample": rep, "flipped_bits": bits})
                rec = Q.run(s, t, src, scratch=mem, scratch_read=0x6060)
                import struct
                for index, sample in enumerate(samples):
                    sample["ecc_result"], sample["ecc_status"] = struct.unpack_from("<II", rec["scratch"], 0x6000 + index * 8)
                    offset = index * 1536 + 1024
                    sample["corrected_matches_input"] = rec["scratch"][offset:offset + 512] == pattern(512, sample["sample"])
                rec["samples_ecc"] = samples
                out[f"bch{strength}/errors{count}/batch{batch}"] = rec
    return summary("miscorr", out)


@experiment("eccmode4")
def eccmode4(s, t):
    """M2 encode reproducibility and GF(2) input pairs for each known strength."""
    out = {}
    for strength in (8, 16):
        for byte in (0, 1, 2, 3):
            for rep in range(3):
                out[f"bch{strength}/input{byte}/rep{rep}"] = ecc_case(s, t, strength=strength,
                            data=bytes([byte]) + bytes(511), decode=False)
    return summary("eccmode4", out)


@experiment("seqfirst")
def seqfirst(s, t):
    """First M2 ECC operation, retained without a preflight ECC operation."""
    rec = ecc_case(s, t, data=bytes(512), decode=False)
    return rec, "First ECC encode; run immediately after a fresh boot for cold-state evidence"


@experiment("seqrestart")
def seqrestart(s, t):
    """Encode sequence across ECC reset/acknowledge/strength changes."""
    out = {}
    for off, value in ((0x81c, 1), (0x810, 0x1ff), (0x1c, 0), (0xc10, 1)):
        for rep in range(3):
            out[f"{off:x}/{rep}"] = ecc_case(s, t, decode=False, params=[(off, value)])
    return summary("seqrestart", out)


def alternate_code(s, t, name):
    # M1 b15 is not a usable M2 RS selector: M2 bits 15:8 are metadata length.
    # Sweep unassigned low/high control bits, preserving M2's encode bit.
    out = {}
    for bit in (*range(3, 8), *range(16, 25)):
        out[f"candidate-bit{bit}"] = ecc_case(s, t, mode=1 << bit, decode=False, data=bytes(512))
    out["applicability"] = {"status": "unresolved", "original_question": name,
        "reason": "M1 RS modes have no established M2 selector; no RS strength or flash claim follows from this sweep"}
    return summary(name, out)


for _name in ("b15ecc", "rsdecode", "rsresult", "rschip"):
    def _bind(name):
        return lambda s, t: alternate_code(s, t, name)
    experiment(_name)(_bind(_name))


@experiment("finale")
def finale(s, t):
    """M2 remaining low control bits, counter/IRQ and in-flight event follow-ups."""
    from .discovery import event_matrix, irq_matrix, fmc28b
    from .control_experiments import blk100b
    out = {"ecc-candidates": alternate_code(s, t, "finale")[0], "counter": fmc28b(s, t)[0],
           "window-timing": blk100b(s, t)[0], "interrupts": irq_matrix(s, t, "finale", range(16))[0],
           "inflight": event_matrix(s, t, "finale", ("ECC", "DMA"), (9,), range(64))[0]}
    return out, "M2 finale: ECC control candidates, counter, timing words, IRQs, in-flight conditions"
