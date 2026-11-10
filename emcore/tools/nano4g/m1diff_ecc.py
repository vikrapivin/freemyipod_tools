"""M1-versus-M2 experiments (m1diff_ecc): one family per comparison.

Each family settles the M2 side of the listed M1 findings; the decision rule is in
the key's crosswalk entry and the case conditions.

Construction facts used (settled on M2, not re-measured here): 0x804 ECC_CTRL bit0
decode, bit1 encode, bit2 BCH16, bits 15:8 metadata length; 0x810 status (bit1/bit2
done W1C, bit3 uncorrectable, bit5 >= 7 corrected, bit6 erased); 0x818 total<<11|n,
reset by 0x81c; 0x80c 8-slot ring (a read advances it); 0x820.. locations;
0x840..0x85c parity readout. Every unusual ECC word a case writes is read first and
written back with `st` on both the END path and the fault handler.
"""
import hashlib
import random

from . import followup_experiments as F, sequences as Q  # noqa: F401
from .protocol import DISCOVERY, MUTATE
from fmss import probe as P  # noqa: F401

M1 = 'M1-F116'
NOP_END = '    movi r6,#0xfa17fa17\n    end\n'


def sha(b):
    return hashlib.sha256(bytes(b)).hexdigest()


def handler(restore=''):
    """Fault vector that first writes back the saved words (same label as t.trap())."""
    return '\ntrap:\n' + restore + NOP_END


def page_layout(strength):
    plen = 13 if strength == 8 else 26
    return plen, (8 if strength == 8 else 7)


def build_page(sectors, strength, meta=F.BCH_META, meta_bytes=None):
    """Raw 4096+128 page: meta(10) | (par | data) per sector. sectors: (data, parity or None = host parity,
    bit errors as positions in this sector's on-flash bytes, meta included for sector 0)."""
    plen, _ = page_layout(strength)
    raw, info, off = bytearray(b'\xff' * (4096 + 128)), [], 10
    for k, (data, parity, bits) in enumerate(sectors):
        m = (meta if meta_bytes is None else meta_bytes) if k == 0 else b''
        if parity is None:
            parity = F.bch8_sector(data, m) if strength == 8 else F.bch16_parity(data + m)
        start = 0 if k == 0 else off
        clean = (m + parity + data)
        region = F.flip(clean, bits)
        raw[start:start + len(region)] = region
        off = (10 if k == 0 else off) + plen + 512
        info.append(dict(errors=len(bits), bits=list(bits), clean_data_sha256=sha(data),
                         written_data_sha256=sha(region[-512:]), parity=bytes(parity).hex()))
    return bytes(raw), info


def program(t, g, a, row, raw, conditions):
    mem = F.memory()
    mem[:len(raw)] = raw
    return F.Case('program/%d' % (row - g.row), F.finish(t, Q.write_page(g, row, a, F.BCH_META, raw=True)),
                  dict(flags=MUTATE, timeout_us=300000, scratch=mem, scratch_read=0),
                  dict(conditions, row=row, raw_sha256=sha(raw), check='r5 bit 0 = program FAIL'), pins=True)


def raw_read(t, g, a, row, raw, name=None):
    return F.Case(name or 'raw/%d' % (row - g.row), F.finish(t, Q.read_page(g, row, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
                  dict(scratch=F.memory(), scratch_read=0x3400, timeout_us=200000),
                  dict(row=row, expected_raw_sha256=sha(raw), layout='0x2000 raw page 4224 bytes'), pins=True)


def status_hook(cap, stride=12):
    return lambda sec, slot: (Q.store(0x810, cap + stride * sec) + Q.store(0x818, cap + stride * sec + 4)
                              + Q.store(0x80c, cap + stride * sec + 8))


# --- m2_m1_zero_chunk (M1-F116) --------------------------------------------------------------

ZERO_PAGES = (('zero', bytes(512), 'zero'), ('ffdata-zeropar', b'\xff' * 512, 'zero'),
              ('ffdata-validpar', b'\xff' * 512, 'host'))


def zero_pages(strength):
    plen, n = page_layout(strength)
    for label, data, par in ZERO_PAGES:
        sectors = [(data, bytes(plen) if par == 'zero' else None, ()) for _ in range(n)]
        raw, info = build_page(sectors, strength, meta_bytes=bytes(10))
        yield label, raw, info


def zero_chunk(s, t):
    """All-zero sector-0 chunk (meta, parity, data = 0) versus 0xff data with zero / valid parity, BCH8 and BCH16."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    pages = [(strength, label, raw, info) for strength in (8, 16) for label, raw, info in zero_pages(strength)]
    print('PLAN: m2_m1_zero_chunk erases CE%d block %d and programs pages 0..%d' % (g.ce, g.block, len(pages) - 1),
          flush=True)
    yield F.Case('erase', F.finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for page, (strength, label, raw, info) in enumerate(pages):
        yield program(t, g, a, g.row + page, raw, dict(strength=strength, page=label, sectors=info))
    cap = a + 0x3100
    for page, (strength, label, raw, info) in enumerate(pages):
        n = page_layout(strength)[1]
        body = Q.wr(0x81c, 1) + F.decode_loop(g, g.row + page, a + 0x2040, n, strength, status_hook(cap))
        yield F.Case('decode/bch%d/%s' % (strength, label), F.finish(t, body),
                     dict(scratch=F.memory(), scratch_read=0x3200, timeout_us=300000),
                     dict(strength=strength, page=label, row=g.row + page, sectors=info, findings=[M1],
                          decision='same as M1 if the all-zero chunk is uncorrectable (810 bit 3); differs if it decodes clean',
                          layout='0x2000 meta words, 0x2040 data; 0x3100 + 12s: 810, 818, 80c'), pins=True)
    for page, (strength, label, raw, info) in enumerate(pages):
        yield raw_read(t, g, a, g.row + page, raw, 'raw/bch%d/%s' % (strength, label))


# --- m2_m1_ecc_area (M1-F149) ------------------------------------------------------------------

AREA = tuple(range(0x900, 0xa00, 4))
AREA_SAVED = ((0x940, 1), (0x944, 2), (0x948, 4), (0x94c, 5), (0x950, 6))  # (word, register)
AREA_PATTERNS = ('zeros', 'ones', 'ramp')
AREA_WRITES = ('none', 'w0', 'wff')
AREA_DUMP = (0x810, 0x818) + tuple(range(0x820, 0x860, 4)) + AREA     # 0x80c skipped: a read advances the ring


def area_save(write):
    src = ''.join('    ld r%d,[0x%x]\n' % (reg, off) for off, reg in AREA_SAVED)
    if write != 'none':
        src += ''.join(Q.wr(off, 0 if write == 'w0' else 0xffffffff) for off, _ in AREA_SAVED)
    return src


def area_restore():
    return ''.join('    st [0x%x],r%d\n' % (off, reg) for off, reg in AREA_SAVED)


def dump(words, base):
    return ''.join(Q.store(off, base + 4 * i) for i, off in enumerate(words))


def ecc_area(s, t):
    """0x900..0x9fc (0x990) after BCH8/BCH16 encodes and decodes; 0x940..0x950 written 0 / ones first."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = F.memory()
    for k, p in enumerate(AREA_PATTERNS):
        mem[0x200 * k:0x200 * k + 512] = F.encode_data(p)
    for strength in (8, 16):
        for k, p in enumerate(AREA_PATTERNS):
            data = F.encode_data(p)
            expect = (F.bch8_sector(data) if strength == 8 else F.bch16_parity(data)).hex()
            for write in AREA_WRITES:
                body = (area_save(write) + Q.setup() + Q.dma(a + 0x200 * k, 0)
                        + Q.ecc(0, True, strength, acknowledge=False) + Q.wait(4)
                        + dump(AREA_DUMP, a + 0x2000) + Q.wr(0x810, 1) + area_restore())
                yield F.Case('encode/bch%d/%s/%s' % (strength, p, write), F.finish(t, body, handler=handler(area_restore())),
                             dict(scratch=mem, scratch_read=0x2200, timeout_us=100000),
                             dict(strength=strength, pattern=p, write=write, expected_parity=expect, findings=['M1-F149'],
                                  decision='same as M1 if 0x990 reads 0 after the encode and 0x940..0x950 change neither '
                                           'parity nor decode',
                                  layout='0x2000 + 4i: %s after WAIT4, before the ack' % ','.join('%x' % o for o in AREA_DUMP)))
    if g.strength != 8:
        return
    # Decodes need stored code words: sectors zero (with metadata), ones, ramp with two bit errors.
    pages = []
    for strength in (8, 16):
        plen = page_layout(strength)[0]
        sectors = [(F.encode_data('zeros'), None, ()), (F.encode_data('ones'), None, ()),
                   (F.encode_data('ramp'), None, (8 * plen + 100, 8 * plen + 2000))]
        pages.append((strength,) + build_page(sectors, strength))
    yield F.Case('erase', F.finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for page, (strength, raw, info) in enumerate(pages):
        yield program(t, g, a, g.row + page, raw, dict(strength=strength, sectors=info))
    stride = 4 * len(AREA_DUMP)
    cap = a + 0x3100
    for page, (strength, raw, info) in enumerate(pages):
        for write in AREA_WRITES:
            hook = lambda sec, slot: dump(AREA_DUMP, cap + stride * sec)
            body = area_save(write) + Q.wr(0x81c, 1) + F.decode_loop(g, g.row + page, a + 0x2040, 3, strength, hook)
            body += area_restore()
            yield F.Case('decode/bch%d/%s' % (strength, write), F.finish(t, body, handler=handler(area_restore())),
                         dict(scratch=F.memory(), scratch_read=0x3100 + 3 * stride, timeout_us=300000),
                         dict(strength=strength, write=write, row=g.row + page, sectors=info, findings=['M1-F149'],
                              layout='0x2000 meta, 0x2040 data (3 x 512); 0x3100 + %#x s + 4i: %s after WAIT4 of sector s'
                                     % (stride, ','.join('%x' % o for o in AREA_DUMP))), pins=True)



MISCORR_SECTORS = 240
MISCORR_RANGE = {8: (9, 64), 16: (17, 64)}
MISCORR_SEED = 0x4d31f168


def miscorr_pages():
    """(strength, raw, info) pages of random code words with random error counts beyond the radius."""
    rng = random.Random(MISCORR_SEED)
    out = []
    for strength in (8, 16):
        plen, per = page_layout(strength)
        lo, hi = MISCORR_RANGE[strength]
        for first in range(0, MISCORR_SECTORS, per):
            sectors = []
            for k in range(per):
                data = bytes(rng.getrandbits(8) for _ in range(512))
                size = 8 * ((10 if k == 0 else 0) + plen + 512)
                bits = sorted(rng.sample(range(size), rng.randint(lo, hi)))
                sectors.append((data, None, bits))
            out.append((strength,) + build_page(sectors, strength))
    return out


# --- m2_m1_ecc_irq_line (M1-F175) --------------------------------------------------------------

# Explained words excluded: 804 ctrl, 80c ring, 810 status, 818 count, 81c reset, 820..85c results/parity.
IRQ_WORDS = (0x800, 0x808, 0x814) + tuple(range(0x860, 0xa00, 4))
IRQ_VALUES = (0xffffffff, 0x1, 0xffff, 0xffff0000, 0x55555555, 0xaaaaaaaa)
VIC_RAW = (0x38e00008, 0x38e01008)
IRQ_SNAP = ('before', 'encode', 'decode')


def vic_snap(addr):
    """VIC0 / VIC1 raw status read by the sequencer (ldi), stored at addr, addr + 4."""
    return ''.join('    movi r7,#0x%x\n    ldi r0,[r7]\n    movi r7,#0x%x\n    sti [r7],r0\n' % (v, addr + 4 * k)
                   for k, v in enumerate(VIC_RAW))


def new_lines(base, reg, vic):
    """reg = (encode | decode snapshot) & ~before for one VIC (r3, r4 temporaries)."""
    return ('    movi r7,#0x%x\n    ldi r3,[r7]\n    movi r%d,#0xffffffff\n    sub r%d,r3\n' % (base + 4 * vic, reg, reg)
            + '    movi r7,#0x%x\n    ldi r3,[r7]\n    movi r7,#0x%x\n    ldi r4,[r7]\n    or r3,r4\n    and r%d,r3\n'
            % (base + 8 + 4 * vic, base + 16 + 4 * vic, reg))


def conditional_wait2(label):
    """wait2 N only for lines N (VIC N>>5 bit N&31) that rose; 0xd74 = N before each, 0xd78 = N after."""
    src = ''
    for n in range(64):
        reg = 1 if n < 32 else 2
        shift = '    shr r3,r3,#%d\n' % (n & 31) if n & 31 else ''
        src += ('    mov r3,r%d\n' % reg + shift + '    and r3,r3,#1\n    bz r3,%s_%d\n' % (label, n)
                + Q.wr(0xd74, n) + '    .insn 0x9,%d,0,0\n' % n + Q.wr(0xd78, n) + '%s_%d:\n' % (label, n))
    return src


def irq_restore(off):
    return ('    st [0x%x],r5\n' % off if off is not None else '') + Q.wr(0x44, 0)


def ecc_irq_line(s, t):
    """Candidate ECC enables (unexplained 0x800..0x9fc words, multi-bit values, with / without FMC 0x044 = ones):
    VIC raw status around a BCH8 encode and decode completion; wait2 N only for lines that rose."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    snap = a + 0x2000
    variants = [(None, None, 0), (None, None, 0xffffffff)]
    variants += [(off, v, 0) for off in IRQ_WORDS for v in IRQ_VALUES]
    variants += [(off, 0xffffffff, 0xffffffff) for off in IRQ_WORDS]
    for k, (off, value, en044) in enumerate(variants):
        body = Q.setup() + Q.wr(0xd74, 0xffffffff) + Q.wr(0xd78, 0xffffffff)
        if off is not None:
            body += '    ld r5,[0x%x]\n' % off + Q.wr(off, value)
        body += Q.wr(0x44, en044) + Q.dma(a, 0) + vic_snap(snap)
        body += Q.ecc(0, True, 8, acknowledge=False) + Q.wait(4) + F.delay(1000) + vic_snap(snap + 8) + Q.wr(0x810, 1)
        body += Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 1) + Q.wait(4) + F.delay(1000) + vic_snap(snap + 16)
        body += Q.wr(0x810, 1) + new_lines(snap, 1, 0) + new_lines(snap, 2, 1)
        body += conditional_wait2('w2l%d' % k) + irq_restore(off)
        label = 'none' if off is None else '%03x/%08x' % (off, value)
        yield F.Case('%s/en044-%x' % (label, en044), F.finish(t, body, handler=handler(irq_restore(off))),
                     dict(scratch=F.memory(), scratch_read=0x2020, start_value=0x07b5, flags=DISCOVERY, timeout_us=30000),
                     dict(candidate=None if off is None else hex(off), value=None if value is None else hex(value),
                          enable044=hex(en044), findings=['M1-F175'],
                          decision='same as M1 if an ECC enable raises source 43 (VIC1 line 11); differs if another '
                                   'line or none rises (compare with none/en044-*)',
                          observe='vic0_seen / vic1_seen; 0x2000 + 8k: VIC0, VIC1 raw %s; after-map 0xd74 = line '
                                  'of the last wait2 attempted (0xffffffff none), 0xd78 = last passed' % '/'.join(IRQ_SNAP),
                          reference='wait2 fault with start 0x07b5 ≈ 4.4 ms; VIC0 line 5 always asserted'))


# --- m2_m1_ecc_alt_code (M1-F133) --------------------------------------------------------------

RS_PRIM, RS_N = 0x409, 1023


def _rs_tables():
    exp, log, x = [0] * (2 * RS_N), [0] * (RS_N + 1), 1
    for i in range(RS_N):
        exp[i] = exp[i + RS_N] = x
        log[x] = i
        x <<= 1
        if x & 0x400:
            x ^= RS_PRIM
    return exp, log


def rs_parity(data, nsym):
    """M1 Reed-Solomon (GF(2^10)/0x409, roots a^0..a^(nsym-1), byte symbols): parity packed 10 bits LE."""
    exp, log = _rs_tables()
    mul = lambda p, q: exp[log[p] + log[q]] if p and q else 0
    g = [1]
    for i in range(nsym):
        new = [0] * (len(g) + 1)
        for k, c in enumerate(g):
            new[k + 1] ^= c
            new[k] ^= mul(c, exp[i])
        g = new
    reg = [0] * nsym
    for sym in data:
        fb = sym ^ reg[nsym - 1]
        reg = [0] + reg[:nsym - 1]
        if fb:
            for k in range(nsym):
                reg[k] ^= mul(fb, g[k])
    return sum(v << (10 * k) for k, v in enumerate(reg)).to_bytes((10 * nsym + 7) // 8, 'little')


ALT_BITS = (3, 4, 5, 6, 7) + tuple(range(16, 32))     # every ECC_CTRL bit outside op, BCH16 and metadata length
ALT_DUMP = tuple(range(0x840, 0x860, 4)) + tuple(range(0x900, 0x920, 4)) + (0x990, 0x810)
ALT_META_KICKS = (0x8002, 0xff02, 0x8006, 0xff06)


def alt_restore():
    # 0x804 mode/metadata bits written back without the op bits (no operation, p25).
    return '    and r5,r5,#0xfffffffc\n    st [0x804],r5\n'


def ecc_alt_code(s, t):
    """Encodes with ECC_CTRL bit pairs (bits 3..7, 16..31), all of them, and metadata length 0x80 / 0xff."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    mem = F.memory()
    for k, p in enumerate(AREA_PATTERNS):
        mem[0x200 * k:0x200 * k + 512] = F.encode_data(p)
    predictions = {}
    for p in AREA_PATTERNS:
        data = F.encode_data(p)
        predictions[p] = {'bch8': F.bch8_sector(data).hex(), 'bch16': F.bch16_parity(data).hex(),
                          **{'rs%d' % n: rs_parity(data, n).hex() for n in (8, 12, 16)}}
    words = []
    for s16 in (0, 4):
        for i, b0 in enumerate(ALT_BITS):
            for b1 in ALT_BITS[i + 1:]:
                words.append(('pair', 'ramp', 2 | s16 | 1 << b0 | 1 << b1))
        for p in AREA_PATTERNS:
            words.append(('all', p, 2 | s16 | sum(1 << b for b in ALT_BITS)))
            words += [('meta', p, 2 | s16 | m << 8) for m in (0x80, 0xff)]
    for kind, p, kick in words:
        k = AREA_PATTERNS.index(p)
        body = ('    ld r5,[0x804]\n' + Q.setup() + Q.metadata(F.BCH_META) + Q.dma(a + 0x200 * k, 0)
                + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, kick) + Q.wait(4)
                + dump(ALT_DUMP, a + 0x2000) + Q.wr(0x810, 1) + alt_restore())
        yield F.Case('%s/%s/%08x' % (kind, p, kick), F.finish(t, body, handler=handler(alt_restore())),
                     dict(scratch=mem, scratch_read=0x2100, start_value=0x07b5, flags=DISCOVERY, timeout_us=30000),
                     dict(kick=hex(kick), pattern=p, metalen=(kick >> 8) & 0xff, bch16=bool(kick & 4),
                          predictions=predictions[p], metadata=F.BCH_META.hex(), findings=['M1-F133'],
                          decision='same as M1 if some word yields RS parity; differs if every word yields BCH8/BCH16 '
                                   'parity or none (WAIT4 fault = no completion)',
                          layout='0x2000 + 4i: %s after WAIT4' % ','.join('%x' % o for o in ALT_DUMP)))
    # Metadata length 0x80 / 0xff stream on a scratch page (ecc_modes layout: metalen | 26-byte parity | data).
    for case in F.ecc_modes(s, t, ALT_META_KICKS):
        case.name = 'stream/' + case.name
        yield case


# --- m2_m1_timeout_steps ----------------------------------------------------------------------

STEP_N = (2, 3, 4, 5, 6)
STEP_COUNTS = (0x01, 0x07, 0x10, 0x40, 0xff)
ENGINE_HZ = 33.25e6


def timeout_steps(s, t):
    """Start-word bits 7:4 = n for n = 2..6 at five counts: unmet-WAIT3 fault time (m2_timeout_low construction)."""
    for n in STEP_N:
        for count in STEP_COUNTS:
            word = (count << 8) | (n << 4) | 0x5
            predicted = (count + 1) * (1 << (n + 3)) / ENGINE_HZ * 1e6
            yield F.Case('n%d/c%02x' % (n, count), F.finish(t, Q.wait(3)),
                         dict(start_value=word, flags=DISCOVERY, timeout_us=20000),
                         dict(start_word=hex(word), bits_7_4=n, count=count,
                              predicted_us_2_n_plus_3=round(predicted, 1),
                              measure='fault_at_us / elapsed_us',
                              reference='n >= 4 follows 2^(n+3) cycles per count step; count 0xff measured n=2 ≈ 62 us, '
                                        'n=3 ≈ 246 us (rule: 246 / 493 us)'))


SPECS = {
    'm2_m1_zero_chunk': (zero_chunk, 'eccstr,miscorr', 'All-zero sector-0 chunk (meta/parity/data 0) and 0xff data with zero/valid parity, BCH8/BCH16'),
    'm2_m1_ecc_area': (ecc_area, 'eccregs,eccpar', '0x900..0x9fc (0x990) after BCH8/BCH16 encodes and decodes; 0x940..0x950 written 0/ones first'),
    'm2_m1_ecc_irq_line': (ecc_irq_line, 'irq,eccregs', 'Unexplained ECC words x multi-bit values (and 0x044) around encode/decode: VIC raw, conditional wait2'),
    'm2_m1_ecc_alt_code': (ecc_alt_code, 'eccmode,b15ecc', 'ECC_CTRL bit pairs, all bits, metadata length 0x80/0xff: parity vs BCH8/BCH16/RS-8/12/16'),
    'm2_m1_timeout_steps': (timeout_steps, 'startbits,tmobits2', 'Start-word bits 7:4 n = 2..6 at counts 01/07/10/40/ff: WAIT3 fault time'),
}

# Writes to unexplained ECC words (0x800..0x9fc, incl. 0x940..0x950) could wedge the engine beyond the
# probe reset; run these after every other family.
RUN_LAST = ('m2_m1_ecc_area', 'm2_m1_ecc_irq_line')
