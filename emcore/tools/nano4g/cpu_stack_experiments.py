"""m2_cpu_stack: the CPU-only NAND driver (no FMSS sequencer program) against the sequencer driver.

Firmware: target/ipodnano4g/cpu_nand_stack.c, sub-ops 0x10..0x14 of Cmd.NAND_CPU (tools/nano4g/nand.py
cpu_stack). Every job resets the engine first, bounds every wait, releases the controller, clears
FMC_CTRL0 and runs the ordinary M2 reset; the idle state after that reset is part of every record that
touched the controller (reset_health below). ECC is either the hardware engine kicked by the CPU ('hw')
or the software BCH decoder/encoder ('sw', no ECC-engine access), selected per job.

Regions (records carry 'region' and 'format'):
  reserved  CE0 block 0, boot-ROM format (3 x [26 parity | 512 data], BCH16): READ ONLY
  main      other blocks outside the scratch blocks, Whimory format (meta[10] | par | data, 7 x par | data,
            BCH8 or BCH16 by the host-selected --bch): READ ONLY
  scratch   blocks 3000 (geometry block) and 3001: the erase/program targets this experiment picks
The firmware programs and erases any block of any CE; it rejects only malformed request words.

Phases (one session, one family; records land in results['m2_cpu_stack']):
  1 resident batch: sequencer erase of block 3001, engine-encoded sequencer program of its page 1,
    sequencer raw reads of reserved/0 and main/128
  2 CPU-stack jobs: raw (DMA, FIFO), hw-ECC and sw-ECC (DMA and FIFO raw paths) reads of the reserved,
    main and sequencer-written pages against the firmware sequencer readers (boot_read.s, read.s);
    hw/sw/host encodes of both formats without NAND access; malformed request words; scratch erase,
    hw- and sw-encoded programs of both formats, a raw program with injected bit errors (1..t+1 per
    sector) read back by both decoders against the known error locations; an erase/program of the
    sequencer-written block; the firmware reader of every CPU-written main page
  3 final info record

Simulation: BCH16 parity and the engine's correction are opt-in models (m2_data BCH16_PHYSICAL,
DataPath.correct_errors); checks that depend on them carry 'model' = 'bch16' / 'correction'.
"""
import dataclasses
import hashlib
import random

from . import sequences as Q
from .bch import bch8, bch16
from .nand import cpu_stack, check
from .protocol import Cmd, CPU_NAND, ToolError

NAME = 'm2_cpu_stack'
SCRATCH_BLOCKS = range(3000, 3002)
RESERVED_ROWS = (0, 1, 2, 100)
SEQ_META = b'SEQ-STACK0'
META_HW, META_SW, META_ERR, META_RANGE = b'CPU-HW-ECC', b'CPU-SW-ECC', bytes(range(0x70, 0x7a)), b'BLOCK-3001'


def main_rows(g):
    return (128, 129, 2000 * g.pages, 2999 * g.pages)


def seq_geometry(g):
    return dataclasses.replace(g, block=g.block + 1)


def page_rows(g):
    """(label, CE, row, format) of the read-only comparisons."""
    return ([('reserved/%d' % r, 0, r, 'boot') for r in RESERVED_ROWS] +
            [('main/%d' % r, g.ce, r, 'main') for r in main_rows(g)] +
            [('seq-page', g.ce, seq_geometry(g).row + 1, 'main')])


def pattern(seed, n):
    return bytes((i * (2 * seed + 37) + (i >> 9) * (seed + 3) + seed) & 255 for i in range(n))


def region(g, ce, row):
    block = row // g.pages
    if block == 0:
        return 'reserved' if ce == 0 else 'boot-block'
    return 'scratch' if block in SCRATCH_BLOCKS else 'main'


def parity(strength, message):
    return bch8(message) if strength == 8 else bch16(message)


def main_raw(g, data, meta):
    """Whimory raw page with host BCH parity."""
    p, out = g.parity, bytearray(b'\xff' * (4096 + g.spare))
    out[0:10] = meta
    for s in range(8):
        d = data[512 * s:512 * (s + 1)]
        o = 10 + s * (p + 512)
        out[o:o + p] = parity(g.strength, d + (meta if s == 0 else b''))
        out[o + p:o + p + 512] = d
    return bytes(out)


def boot_raw(g, data):
    """Boot-ROM raw page with host BCH16 parity; rest of the page erased."""
    out = bytearray(b'\xff' * (4096 + g.spare))
    for s in range(3):
        d = data[512 * s:512 * (s + 1)]
        out[538 * s:538 * s + 26] = bch16(d)
        out[538 * s + 26:538 * (s + 1)] = d
    return bytes(out)


def inject(g, raw, seed=7):
    """Bit errors per sector of a main raw page: (raw, {sector: sorted locations}, uncorrectable sectors).

    Locations use the engine's numbering: codeword byte b (data, metadata, parity) bit i -> 8b + i."""
    t, p = g.strength, g.parity
    plan = {0: [('meta', 1)], 1: [('data', 2)], 2: [('data', t // 2)], 3: [('data', t // 2), ('par', t // 2 - 1)],
            4: [('data', t)], 5: [('data', t + 1)], 6: [('par', 3)], 7: []}
    rnd, out, locations = random.Random(seed), bytearray(raw), {}
    for s, parts in plan.items():
        o = 10 + s * (p + 512)
        ml = 10 if s == 0 else 0
        chosen = set()
        for part, k in parts:
            span = {'data': (o + p, 0, 512), 'meta': (0, 512, 10), 'par': (o, 512 + ml, p)}[part]
            while k:
                byte, bit = rnd.randrange(span[2]), rnd.randrange(8)
                loc = (span[1] + byte) * 8 + bit
                if loc in chosen:
                    continue
                chosen.add(loc)
                out[span[0] + byte] ^= 1 << bit
                k -= 1
        locations[s] = sorted(chosen)
    uncorrectable = {s for s, l in locations.items() if len(l) > t}
    return bytes(out), locations, uncorrectable


def split_main(g, raw):
    """Data and metadata of a main raw page as stored (no correction)."""
    p = g.parity
    return b''.join(raw[10 + s * (p + 512) + p:10 + (s + 1) * (p + 512)] for s in range(8)), raw[:10]


def sha(b):
    return hashlib.sha256(bytes(b)).hexdigest()


def sector_view(rec):
    """Comparable per-sector outcome: (status, corrected bits)."""
    if not rec or not rec.get('sectors'):
        return None
    return [(next((f for f in ('erased', 'uncorrectable', 'ok') if f in e['flags']), 'none'), e['corrected'])
            for e in rec['sectors']]


# ------------------------------------------------------------------ sequencer phase (resident cases)
def phase1(g):
    from .followup_experiments import Case, finish, memory
    from .protocol import MUTATE
    sg = seq_geometry(g)

    def generate(view, t):
        a = t.scratch_addr(view)
        mutate = dict(flags=MUTATE, timeout_us=300000)
        yield Case('seq/erase', finish(t, Q.erase(sg)), dict(mutate), dict(block=sg.block, ce=sg.ce, region='scratch'),
                   pins=True)
        mem = memory()
        mem[:4096] = pattern(1, 4096)
        yield Case('seq/program', finish(t, Q.write_page(sg, sg.row + 1, a, SEQ_META)),
                   dict(mutate, scratch=mem, scratch_read=0),
                   dict(row=sg.row + 1, ce=sg.ce, region='scratch', format='main', data_sha256=sha(pattern(1, 4096)),
                        meta=SEQ_META.hex(), writer='sequencer, engine-encoded'), pins=True)
        for label, ce, row in (('reserved/0', 0, 0), ('main/128', g.ce, 128)):
            yield Case('seq/raw/' + label, finish(t, Q.read_page(g, row, a + 0x2000, a + 0x3100, a + 0x3000, raw=True, ce=ce)),
                       dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                       dict(row=row, ce=ce, region=region(g, ce, row), format='raw'), pins=True)
    return generate


def run_phase(s, t, generate, records, compact_cache, health):
    from .followup_experiments import run_cases
    from .batch import merge_health
    s.last_batch_records = s.last_batch_compact = s.last_batch_health = None
    recs, _ = run_cases(NAME, s, t, generate)
    records.update(recs)
    if getattr(s, 'last_batch_compact', None):
        compact_cache.update(s.last_batch_compact)
    if getattr(s, 'last_batch_health', None):
        merge_health(health, s.last_batch_health)
    return recs


def static_cases(view, t):
    """Every resident probe case of the sequencer phase (preflight static check)."""
    yield from phase1(Q.geometry(view))(view, t)


# ------------------------------------------------------------------ CPU-stack / firmware-reader jobs
class Runner:
    def __init__(self, s, g):
        self.s, self.g, self.records, self.jobs = s, g, {}, 0

    def stack(self, name, op, ce=0, row=0, ecc='raw', fmt='main', path='dma', data=None, meta=None,
              raw=None, word=None, **conditions):
        bch = self.g.strength
        try:
            job, out = cpu_stack(self.s, op, ce, row, ecc, fmt, path, bch, data, meta, raw, word)
        except ToolError:
            raise
        except Exception as exc:  # recorded, never replayed
            self.records[name] = {'case_error': type(exc).__name__, 'message': str(exc), 'conditions': conditions}
            raise
        self.jobs += 1
        rec = dict(job=job, conditions=dict(conditions, op=op, ce=ce, row=row, ecc=ecc, format=fmt, path=path, bch=bch))
        if op not in ('info', 'build'):
            rec['conditions'].setdefault('region', region(self.g, ce, row))
        if out is not None:
            out['controller'] = op != 'info' and not (op == 'build' and ecc == 'sw')
            for key in ('data', 'raw'):
                if out.get(key):
                    out[key + '_sha256'] = sha(out[key])
            rec.update(out)
        self.records[name] = rec
        return rec

    def fw_page(self, name, ce, row):
        st = self.s.job(Cmd.NAND_READ_PAGE, ce, row)
        raw = self.s.read(st.result_addr, 0x1040) if st.result_len else b''
        rec = dict(job=dict(result=st.job_result, lib_rc=st.job_lib_rc), reader='firmware nand_read_page (read.s)',
                   conditions=dict(ce=ce, row=row, region=region(self.g, ce, row), format='main'),
                   data=raw[:4096], meta=raw[0x1000:0x100a], sector_status=list(raw[0x1020:0x1028]))
        rec['data_sha256'] = sha(rec['data'])
        self.records[name] = rec
        return rec

    def fw_reserved(self, name, row):
        st = self.s.job(Cmd.NAND_READ_RESERVED, row, 0)
        raw = self.s.read(st.result_addr, 0x600) if st.result_len else b''
        rec = dict(job=dict(result=st.job_result, lib_rc=st.job_lib_rc), reader='firmware nand_read_boot_page (boot_read.s)',
                   conditions=dict(ce=0, row=row, region='reserved', format='boot'), data=raw)
        rec['data_sha256'] = sha(raw)
        self.records[name] = rec
        return rec


def idle_ok(rec):
    if not rec.get('controller'):
        return None
    return (rec['pre_reset_rc'] == 0 and rec['post_reset_rc'] == 0 and rec['post']['c08'] == 0
            and rec['post']['fmc_ctrl0'] == 0 and rec['cleanup']['fmc_ctrl0'] == 0)


def steps_ok(rec):
    return rec.get('rc') == 0 and rec.get('job', {}).get('result') == 0 and \
        not any(step['timeout'] for step in rec.get('steps', ()))


def cpu_stack_family(s, t):
    """CPU-only NAND driver (hw/sw ECC, raw DMA/FIFO, program/erase) vs the sequencer driver (one session)."""
    from .cpu_nand_experiments import seq_bytes, finish_family
    g = Q.geometry(s)
    sg = seq_geometry(g)
    records, compact_cache, health, checks = {}, {}, {}, []
    print('PLAN: %s: sequencer erase/program of CE%d block %d, then CPU-only jobs (writes: blocks %d..%d only)' %
          (NAME, sg.ce, sg.block, SCRATCH_BLOCKS[0], SCRATCH_BLOCKS[-1]), flush=True)
    run_phase(s, t, phase1(g), records, compact_cache, health)

    def check_eq(name, a, b, kind='paths', **info):
        status = 'skipped' if a is None or b is None else 'match' if a == b else 'mismatch'
        checks.append(dict(name=name, kind=kind, status=status, **info))
        return status

    def required(name, ok, **info):
        checks.append(dict(name=name, kind='required', status='match' if ok else 'mismatch', **info))

    if not getattr(s, 'probe_caps', 0) & CPU_NAND:
        return finish_family(s, records, compact_cache, health, checks,
                             '%s: kernel lacks CPU-direct NAND jobs (capability bit 30); only the sequencer phase ran' % NAME)
    r = Runner(s, g)
    for n in (1, 2, 4):
        check(s.job(Cmd.NAND_STEP, n, 0))
    info = r.stack('stack/info/initial', 'info')
    if info['job']['result'] != 0 or 'magic' not in info:
        records.update(r.records)
        return finish_family(s, records, compact_cache, health, checks,
                             '%s: kernel lacks the CPU-only NAND driver (Cmd.NAND_CPU op 0x10 refused, result %d); '
                             'only the sequencer phase ran' % (NAME, info['job']['result']))
    raw_len = 4096 + g.spare
    required('info strength/rawsize == host geometry', (info['strength'], info['rawsize']) == (g.strength, raw_len),
             strength=info['strength'], rawsize=info['rawsize'])
    bch16_model = dict(model='bch16')

    # ---- read-only comparisons: reserved region, main area, the sequencer-written scratch page
    for label, ce, row, fmt in page_rows(g):
        reg = region(g, ce, row)
        info = dict(region=reg, format=fmt, ce=ce, row=row)
        model = bch16_model if fmt == 'boot' or g.strength == 16 else {}
        dma = r.stack('stack/raw-dma/' + label, 'read', ce, row, 'raw', fmt, 'dma')
        fifo = r.stack('stack/raw-fifo/' + label, 'read', ce, row, 'raw', fmt, 'fifo')
        hw = r.stack('stack/hw/' + label, 'read', ce, row, 'hw', fmt)
        sw = r.stack('stack/sw/' + label, 'read', ce, row, 'sw', fmt, 'dma')
        swf = r.stack('stack/sw-fifo/' + label, 'read', ce, row, 'sw', fmt, 'fifo')
        for kind, rec in (('raw-dma', dma), ('raw-fifo', fifo), ('hw', hw), ('sw', sw), ('sw-fifo', swf)):
            required('completed %s %s' % (kind, label), steps_ok(rec), **info)
        check_eq('raw dma == fifo ' + label, dma.get('raw'), fifo.get('raw'), **info)
        seq = seq_bytes(records.get('seq/raw/' + label, {}), raw_len)
        if seq is not None:
            check_eq('raw cpu-dma == sequencer ' + label, dma.get('raw'), seq, **info)
        check_eq('sw raw (dma) == raw read ' + label, sw.get('raw'), dma.get('raw'), **info)
        check_eq('hw data == sw data ' + label, hw.get('data'), sw.get('data'), kind='required', **info, **model)
        check_eq('hw sectors == sw sectors ' + label, sector_view(hw), sector_view(sw), kind='required', **info, **model)
        check_eq('sw fifo == sw dma ' + label, (swf.get('data'), sector_view(swf)), (sw.get('data'), sector_view(sw)),
                 kind='required', **info)
        if fmt == 'boot':
            fw = r.fw_reserved('fw/' + label, row)
            if fw['job']['result'] == 0:
                check_eq('hw == firmware boot read ' + label, hw.get('data'), fw['data'], kind='required', **info)
                check_eq('sw == firmware boot read ' + label, sw.get('data'), fw['data'], kind='required', **info, **model)
        else:
            check_eq('hw meta == sw meta ' + label, hw.get('meta'), sw.get('meta'), kind='required', **info, **model)
            fw = r.fw_page('fw/' + label, ce, row)
            if fw['job']['result'] == 0:
                check_eq('hw == firmware read ' + label, (hw.get('data'), hw.get('meta')), (fw['data'], fw['meta']),
                         kind='required', **info)
                check_eq('sw == firmware read ' + label, (sw.get('data'), sw.get('meta')), (fw['data'], fw['meta']),
                         kind='required', **info, **model)
            if label == 'seq-page':
                check_eq('hw == sequencer-written page', (hw.get('data'), hw.get('meta')), (pattern(1, 4096), SEQ_META),
                         kind='required', **info)
                check_eq('sw == sequencer-written page', (sw.get('data'), sw.get('meta')), (pattern(1, 4096), SEQ_META),
                         kind='required', **info, **model)

    # ---- encodes without NAND access: hardware engine vs software vs host BCH
    for fmt, data, meta in (('main', pattern(2, 4096), META_HW), ('boot', pattern(3, 0x600), None)):
        host = main_raw(g, data, meta) if fmt == 'main' else boot_raw(g, data)
        model = bch16_model if fmt == 'boot' or g.strength == 16 else {}
        hw = r.stack('stack/build/%s/hw' % fmt, 'build', ecc='hw', fmt=fmt, data=data, meta=meta)
        sw = r.stack('stack/build/%s/sw' % fmt, 'build', ecc='sw', fmt=fmt, data=data, meta=meta)
        required('build %s hw completed' % fmt, steps_ok(hw), format=fmt)
        check_eq('build %s sw == host BCH' % fmt, sw.get('raw'), host, kind='required', format=fmt)
        check_eq('build %s hw == host BCH' % fmt, hw.get('raw'), host, kind='required', format=fmt, **model)

    # ---- malformed request words: rejected before any register access
    scratch0 = g.row
    for name, word in (('neg/bad-word-bits', 0x80011), ('neg/bad-ecc', 0x3011), ('neg/hw-fifo', 0x11011),
                       ('neg/build-raw', 0x14), ('neg/erase-ecc', 0x1013)):
        rec = r.stack(name, 'read', g.ce, 128, word=word)
        required('bad argument ' + name, rec['job']['result'] == 3 and 'magic' not in rec, job=rec['job'])

    # ---- scratch block: erase, verify, programs in every ECC mode and format, read back every way
    erase = r.stack('stack/erase', 'erase', g.ce, scratch0)
    required('erase completed', steps_ok(erase), nand_status=erase.get('nand_status'))
    for page in (0, 1, 2, 3, 4, 5, g.pages - 1):
        raw = r.stack('stack/erase-verify/raw/%d' % page, 'read', g.ce, scratch0 + page, 'raw')
        check_eq('erased page %d all 0xff' % page, raw.get('raw'), b'\xff' * raw_len, kind='required', region='scratch')
    for ecc in ('hw', 'sw'):
        rec = r.stack('stack/erase-verify/%s/0' % ecc, 'read', g.ce, scratch0, ecc)
        check_eq('erased page 0 %s: 8 erased sectors' % ecc, sector_view(rec), [('erased', 0)] * 8, kind='required',
                 region='scratch')

    def program_and_verify(label, page, fmt, ecc, data, meta=None, block_row=scratch0):
        """Program one page with ecc 'hw' or 'sw' encode; raw, hw-, sw- and firmware reads against it.

        Model tags: an engine-encoded image vs host BCH16, and any decode in the other ECC mode than
        the encode, depend on the physical BCH16 model (simulation only)."""
        row = block_row + page
        host = main_raw(g, data, meta) if fmt == 'main' else boot_raw(g, data)
        model = bch16_model if fmt == 'boot' or g.strength == 16 else {}
        enc_model = model if ecc == 'hw' else {}
        prog = r.stack('stack/program/' + label, 'program', g.ce, row, ecc, fmt, data=data, meta=meta,
                       region='scratch', data_sha256=sha(data), meta_hex=meta.hex() if meta else None)
        required('program %s completed' % label, steps_ok(prog), nand_status=prog.get('nand_status'))
        check_eq('program %s image == host BCH' % label, prog.get('raw'), host, kind='required', **enc_model)
        raw = r.stack('stack/raw/' + label, 'read', g.ce, row, 'raw', fmt)
        check_eq('raw %s == host BCH page' % label, raw.get('raw'), host, kind='required', **enc_model)
        for dec in ('hw', 'sw'):
            rec = r.stack('stack/%s/%s' % (dec, label), 'read', g.ce, row, dec, fmt)
            want = (data, meta) if fmt == 'main' else (data, b'')
            got = (rec.get('data'), rec.get('meta') if fmt == 'main' else b'')
            mixed = model if dec != ecc else {}
            check_eq('%s decode %s == written' % (dec, label), got, want, kind='required', **mixed)
            check_eq('%s decode %s clean' % (dec, label), sector_view(rec), [('ok', 0)] * (8 if fmt == 'main' else 3),
                     kind='required', **mixed)
        if fmt == 'boot' and g.ce == 0:
            fw = r.fw_reserved('fw/' + label, row)     # boot_read.s reads any CE0 page in the boot format
            check_eq('firmware boot read %s == written' % label, fw['data'] if fw['job']['result'] == 0 else
                     ('result', fw['job']['result']), data, kind='required', **(model if ecc == 'sw' else {}))
        if fmt == 'main':
            fw = r.fw_page('fw/' + label, g.ce, row)
            check_eq('firmware read %s == written' % label, (fw['data'], fw['meta']) if fw['job']['result'] == 0 else
                     ('result', fw['job']['result']), (data, meta), kind='required', **(model if ecc == 'sw' else {}))

    program_and_verify('main-hw', 0, 'main', 'hw', pattern(4, 4096), META_HW)
    program_and_verify('main-sw', 1, 'main', 'sw', pattern(5, 4096), META_SW)
    program_and_verify('boot-sw', 3, 'boot', 'sw', pattern(6, 0x600))
    program_and_verify('boot-hw', 4, 'boot', 'hw', pattern(7, 0x600))

    # Raw program with injected bit errors: both decoders against the known locations.
    clean = main_raw(g, pattern(8, 4096), META_ERR)
    bad, locations, uncorrectable = inject(g, clean)
    prog = r.stack('stack/program/raw-errors', 'program', g.ce, scratch0 + 2, 'raw', raw=bad, region='scratch',
                   injected={str(k): v for k, v in locations.items()}, raw_sha256=sha(bad))
    required('program raw-errors completed', steps_ok(prog))
    raw = r.stack('stack/raw/raw-errors', 'read', g.ce, scratch0 + 2, 'raw')
    check_eq('raw raw-errors == programmed image', raw.get('raw'), bad, kind='required', region='scratch')
    bad_data, _ = split_main(g, bad)
    want_data = b''.join(bad_data[512 * s:512 * (s + 1)] if s in uncorrectable else pattern(8, 4096)[512 * s:512 * (s + 1)]
                         for s in range(8))
    want_view = [('uncorrectable', 0) if s in uncorrectable else ('ok', len(locations[s])) for s in range(8)]
    hw_corr = dict(model='correction', **(bch16_model if g.strength == 16 else {}))
    results = {}
    for dec, path in (('hw', 'dma'), ('sw', 'dma'), ('sw', 'fifo')):
        rec = r.stack('stack/%s/raw-errors%s' % (dec, '-fifo' if path == 'fifo' else ''), 'read', g.ce, scratch0 + 2,
                      dec, 'main', path)
        results[dec + path] = rec
        m = hw_corr if dec == 'hw' else {}
        check_eq('%s %s raw-errors data == original (uncorrectable sectors as stored)' % (dec, path), rec.get('data'),
                 want_data, kind='required', **m)
        check_eq('%s %s raw-errors meta == original' % (dec, path), rec.get('meta'), META_ERR, kind='required', **m)
        check_eq('%s %s raw-errors sectors == injected' % (dec, path), sector_view(rec), want_view, kind='required', **m)
        # Location sets: the engine lists bits of one byte from the high bit down (p43: 4099 before 4098).
        check_eq('%s %s raw-errors locations == injected' % (dec, path),
                 [sorted(e['locations']) for e in rec.get('sectors', ())],
                 [[] if s in uncorrectable else sorted(locations[s]) for s in range(8)], kind='required', **m)
    check_eq('hw raw-errors locations == sw locations',
             [sorted(e['locations']) for e in results['hwdma'].get('sectors', ())],
             [sorted(e['locations']) for e in results['swdma'].get('sectors', ())], kind='required', **hw_corr)
    checks.append(dict(name='hw raw-errors location order', kind='observation', status='observation',
                       order=[e['locations'] for e in results['hwdma'].get('sectors', ())]))
    fw = r.fw_page('fw/raw-errors', g.ce, scratch0 + 2)
    checks.append(dict(name='firmware read raw-errors', kind='observation', status='observation',
                       result=fw['job'], sector_status=fw['sector_status']))

    # Raw program of a clean host image (ecc raw) round trip, then erase + program of the sequencer block.
    raw_img = main_raw(g, pattern(9, 4096), META_SW)
    prog = r.stack('stack/program/raw-clean', 'program', g.ce, scratch0 + 5, 'raw', raw=raw_img, region='scratch')
    required('program raw-clean completed', steps_ok(prog))
    back = r.stack('stack/raw/raw-clean', 'read', g.ce, scratch0 + 5, 'raw', path='fifo')
    check_eq('raw-clean fifo read == programmed image', back.get('raw'), raw_img, kind='required', region='scratch')
    erase = r.stack('stack/erase-seq-block', 'erase', g.ce, sg.row, region='scratch')
    required('erase block %d completed' % sg.block, steps_ok(erase))
    gone = r.stack('stack/raw/seq-page-after-erase', 'read', g.ce, sg.row + 1, 'raw')
    check_eq('sequencer page erased by the CPU stack', gone.get('raw'), b'\xff' * raw_len, kind='required', region='scratch')
    program_and_verify('seq-block-hw', 0, 'main', 'hw', pattern(10, 4096), META_RANGE, block_row=sg.row)

    r.stack('stack/info/final', 'info')
    records.update(r.records)

    cpu = {k: v for k, v in r.records.items() if v.get('controller')}
    failed = [k for k, v in cpu.items() if not idle_ok(v)]
    health['cases'] = health.get('cases', 0) + len(cpu)
    health['verified'] = health.get('verified', 0) + len(cpu) - len(failed)
    health.setdefault('initial_reset_failed', []).extend(k for k, v in cpu.items() if v['pre_reset_rc'])
    health.setdefault('cleanup_failed', []).extend(k for k in failed if cpu[k]['pre_reset_rc'] == 0)
    health['cpu_jobs'] = r.jobs
    health['cpu_idle_verified'] = len(cpu) - len(failed)
    records['timing_us'] = {k: v['elapsed_us'] for k, v in cpu.items()}
    records['sw_us'] = {k: v['sw_us'] for k, v in r.records.items() if v.get('sw_us')}
    counts = {}
    for c in checks:
        counts[c['status']] = counts.get(c['status'], 0) + 1
    bad = [c['name'] for c in checks if c['status'] == 'mismatch']
    summary = ('%s: %d CPU-stack jobs, %d sequencer cases; checks %s; idle after %d/%d controller jobs%s' %
               (NAME, r.jobs, len([k for k in records if k.startswith('seq/')]), counts, len(cpu) - len(failed),
                len(cpu), '; MISMATCH: ' + ', '.join(bad) if bad else ''))
    return finish_family(s, records, compact_cache, health, checks, summary)


def entry():
    return {'static_cases': static_cases, 'name': NAME, 'status': 'followup', 'reason': cpu_stack_family.__doc__,
            'question': cpu_stack_family.__doc__, 'original': 'cpu-only', 'fn': cpu_stack_family,
            'source': 'nano4g/cpu_stack_experiments.py', 'resident': True}
