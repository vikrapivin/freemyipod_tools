"""m2_cpu_nand: NAND driven by the CPU alone (no FMSS sequencer program), against the sequencer paths.

Firmware: target/ipodnano4g/cpu_nand.c (Cmd.NAND_CPU). Each CPU job replicates a source program's
register sequence (erase.s, program.s, read.s, boot_read.s, sequences.py) as CPU writes with bounded
polling of FMC_STATUS / ECC_STATUS instead of WAIT, records every step's polled register and time,
releases the controller, clears FMC_CTRL0 and runs the ordinary M2 reset; the idle state after that
reset is part of every record (reset_health below).

Regions (records carry 'region' and 'format'):
  reserved  block 0 (CE0 pages 0..N: the boot-ROM reserved region, 3 x [26 parity | 512 data] per
            0x600-byte ROM page, BCH16 decode 0x808 = 0x80 / 0x005 as boot_read.s); READ ONLY
  main      every other block outside the scratch blocks (Whimory page: meta[10] | par | data, then
            7 x par | data; BCH8 0xa01/0x001 or BCH16 0xa05/0x005); READ ONLY
  scratch   the campaign scratch block (geometry block, 3000): the erase/program target this
            experiment picks (the firmware erases and programs any block).

Phases (one session, one family; its records land in results['m2_cpu_nand']):
  1 resident batch: sequencer erase + engine-encoded program of the scratch page row+1; sequencer raw
    reads of the reserved, main and scratch pages
  2 CPU jobs: raw DMA / FIFO and CPU-kicked ECC reads of every page above, the firmware sequencer
    readers (NAND_READ_RESERVED boot_read.s, NAND_READ_PAGE read.s), CPU erase + erase
    verify, CPU program (host-built BCH parity) + read back, CPU ECC encodes vs host BCH
  3 resident batch: sequencer READ (decoded) and raw read of the CPU-written page
  4 CPU idle-state job
"""
import hashlib

from . import sequences as Q
from .bch import bch8, bch16
from .nand import cpu_nand, check
from .protocol import Cmd, MUTATE, CPU_NAND, ToolError

NAME = 'm2_cpu_nand'
SCRATCH_BLOCKS = range(3000, 3002)     # cpu_nand.h CPU_NAND_SCRATCH_FIRST / COUNT
RESERVED_ROWS = (0, 1, 2, 100)          # CE0 block 0
SEQ_META = b'SEQ-PAGE10'
CPU_META = bytes(range(0x40, 0x4a))
ENCODES = (('bch8/slot0', 0x002, 0), ('bch8/slot1', 0x002, 1), ('bch8/meta', 0xa02, 0),
           ('bch16/slot0', 0x006, 0), ('bch16/meta', 0xa06, 0))


def main_rows(g):
    return (128, 129, 2000 * g.pages, 2999 * g.pages)


def page_rows(g):
    """(label, CE, row) of every page read by both CPU and sequencer: reserved region, main area, scratch."""
    return ([('reserved/%d' % r, 0, r) for r in RESERVED_ROWS] + [('main/%d' % r, g.ce, r) for r in main_rows(g)]
            + [('seq-page', g.ce, g.row + 1)])


def seq_data():
    return bytes((i * 37 + 11) & 255 for i in range(4096))


def cpu_data():
    return bytes((i * 91 + (i >> 9) * 7 + 3) & 255 for i in range(4096))


def encode_data():
    return bytes((i * 53 + (i >> 5) * 7 + 101) & 255 for i in range(512))


def region(g, ce, row):
    block = row // g.pages
    if block == 0:
        return 'reserved' if ce == 0 else 'boot-block'
    return 'scratch' if block in SCRATCH_BLOCKS else 'main'


def page_format(g, reg):
    if reg in ('reserved', 'boot-block'):
        return 'rom'
    return 'main8' if g.strength == 8 else 'main16'


def parity(strength, message):
    return bch8(message) if strength == 8 else bch16(message)


def user_raw(g, data, meta):
    """Whimory raw page with host BCH parity (the stream the original WRITE produces)."""
    plen, out = g.parity, bytearray(b'\xff' * (4096 + g.spare))
    out[0:10] = meta
    out[10:10 + plen] = parity(g.strength, data[:512] + meta)
    out[10 + plen:10 + plen + 512] = data[:512]
    o = 10 + plen + 512
    for s in range(1, 8):
        d = data[512 * s:512 * (s + 1)]
        out[o:o + plen] = parity(g.strength, d)
        out[o + plen:o + plen + 512] = d
        o += plen + 512
    return bytes(out)


def split_raw(g, raw, fmt):
    """Host view of a raw page in the given format: data, metadata, per-sector parity status."""
    out, status, meta = bytearray(), [], b''
    if fmt == 'rom':
        for s in range(3):
            p, d = raw[538 * s:538 * s + 26], raw[538 * s + 26:538 * (s + 1)]
            out += d
            status.append('erased' if set(p + d) == {0xff} else 'match' if bch16(d) == p else 'differs')
        return bytes(out), meta, status
    plen, strength = (13, 8) if fmt == 'main8' else (26, 16)
    meta, o = raw[0:10], 10
    for s in range(8):
        p, d = raw[o:o + plen], raw[o + plen:o + plen + 512]
        o += plen + 512
        out += d
        message = d + (meta if s == 0 else b'')
        status.append('erased' if set(p + d) == {0xff} else 'match' if parity(strength, message) == p else 'differs')
    return bytes(out), bytes(meta), status


def parity_bytes(words, plen):
    """0x840.. words to parity bytes: word k holds bits 32k.. of the big-endian parity (M2-116)."""
    value = sum(w << 32 * k for k, w in enumerate(words))
    return (value & ((1 << 8 * plen) - 1)).to_bytes(plen, 'big')


def sha(b):
    return hashlib.sha256(bytes(b)).hexdigest()


# ------------------------------------------------------------------ sequencer phases (resident cases)
def phase1(g, raw_rows):
    from .followup_experiments import Case, finish, memory

    def generate(view, t):
        a = t.scratch_addr(view)
        mutate = dict(flags=MUTATE, timeout_us=300000)
        yield Case('seq/erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block, ce=g.ce, region='scratch'),
                   pins=True)
        mem = memory()
        mem[:4096] = seq_data()
        yield Case('seq/program', finish(t, Q.write_page(g, g.row + 1, a, SEQ_META)),
                   dict(mutate, scratch=mem, scratch_read=0),
                   dict(row=g.row + 1, ce=g.ce, region='scratch', format=page_format(g, 'scratch'),
                        data_sha256=sha(seq_data()), meta=SEQ_META.hex(), writer='sequencer, engine-encoded'), pins=True)
        for label, ce, row in raw_rows:
            reg = region(g, ce, row)
            yield Case('seq/raw/' + label, finish(t, Q.read_page(g, row, a + 0x2000, a + 0x3100, a + 0x3000, raw=True, ce=ce)),
                       dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                       dict(row=row, ce=ce, region=reg, format='raw', layout='0x2000 raw page'), pins=True)
    return generate


def phase3(g):
    from .followup_experiments import Case, finish, memory

    def generate(view, t):
        a = t.scratch_addr(view)
        yield Case('seq/decode/cpu-page', finish(t, Q.read_page(g, g.row, a + 0x2000, a + 0x3100, a + 0x3000)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                   dict(row=g.row, ce=g.ce, region='scratch', format=page_format(g, 'scratch'),
                        layout='0x2000 data (4096), 0x3000 meta words, 0x3100 ECC_RESULT x8', writer='CPU'), pins=True)
        yield Case('seq/raw/cpu-page', finish(t, Q.read_page(g, g.row, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                   dict(row=g.row, ce=g.ce, region='scratch', format='raw', writer='CPU'), pins=True)
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


def seq_bytes(rec, n, offset=0x2000):
    scratch = rec.get('scratch') if isinstance(rec, dict) else None
    if rec.get('why') != 'END' or not scratch:
        return None
    return bytes(scratch[offset:offset + n])


# ------------------------------------------------------------------ CPU / firmware-reader jobs
class Runner:
    def __init__(self, s, g):
        self.s, self.g, self.records, self.jobs = s, g, {}, 0
        self.raw_len = 4096 + g.spare

    def cpu(self, name, op, ce=0, row=0, param=0, slot=0, upload=None, **conditions):
        try:
            job, out = cpu_nand(self.s, op, ce, row, param, slot, upload)
        except ToolError:
            raise
        except Exception as exc:  # recorded, never replayed
            rec = {'case_error': type(exc).__name__, 'message': str(exc), 'conditions': conditions}
            self.records[name] = rec
            raise
        self.jobs += 1
        rec = dict(job=job, conditions=dict(conditions, op=op, ce=ce, row=row, param=param, slot=slot))
        if 'region' not in conditions and op not in ('state', 'encode'):
            rec['conditions']['region'] = region(self.g, ce, row)
        if out is not None:
            data = out.pop('data')
            rec.update(out)
            rec['data'] = data
            rec['data_sha256'] = sha(data)
        self.records[name] = rec
        return rec

    def fw_page(self, name, ce, row):
        st = self.s.job(Cmd.NAND_READ_PAGE, ce, row)
        raw = self.s.read(st.result_addr, 0x1040) if st.result_len else b''
        rec = dict(job=dict(result=st.job_result, lib_rc=st.job_lib_rc), reader='firmware nand_read_page (read.s)',
                   conditions=dict(ce=ce, row=row, region=region(self.g, ce, row), format=page_format(self.g, region(self.g, ce, row))),
                   data=raw[:4096], meta=raw[0x1000:0x100c], sector_status=list(raw[0x1020:0x1028]))
        rec['data_sha256'] = sha(rec['data'])
        self.records[name] = rec
        return rec

    def fw_reserved(self, name, row):
        st = self.s.job(Cmd.NAND_READ_RESERVED, row, 0)
        raw = self.s.read(st.result_addr, 0x600) if st.result_len else b''
        rec = dict(job=dict(result=st.job_result, lib_rc=st.job_lib_rc), reader='firmware nand_read_boot_page (boot_read.s)',
                   conditions=dict(ce=0, row=row, region='reserved', format='rom'), data=raw)
        rec['data_sha256'] = sha(raw)
        self.records[name] = rec
        return rec


def idle_ok(rec):
    if 'post' not in rec:
        return None
    return (rec['pre_reset_rc'] == 0 and rec['post_reset_rc'] == 0 and rec['post']['c08'] == 0
            and rec['post']['fmc_ctrl0'] == 0 and rec['cleanup']['fmc_ctrl0'] == 0)


def steps_ok(rec):
    return rec.get('rc') == 0 and not any(step['timeout'] for step in rec.get('steps', ()))


def cpu_nand_family(s, t):
    """CPU-direct erase/program/raw read (DMA, FIFO)/ECC vs the sequencer, reserved region vs main area (one session)."""
    g = Q.geometry(s)
    records, compact_cache, health = {}, {}, {}
    rows = page_rows(g)
    print('PLAN: %s: sequencer erase/program of CE%d block %d, then CPU-direct jobs (scratch blocks %d..%d only)' %
          (NAME, g.ce, g.block, SCRATCH_BLOCKS[0], SCRATCH_BLOCKS[-1]), flush=True)
    run_phase(s, t, phase1(g, rows), records, compact_cache, health)

    checks = []

    def check_eq(name, a, b, kind='paths', **info):
        status = 'skipped' if a is None or b is None else 'match' if a == b else 'mismatch'
        checks.append(dict(name=name, kind=kind, status=status, **info))
        return status

    caps = getattr(s, 'probe_caps', 0)
    if not caps & CPU_NAND:
        summary = '%s: kernel lacks CPU-direct NAND jobs (capability bit 30); only the sequencer phase ran' % NAME
        return finish_family(s, records, compact_cache, health, checks, summary)
    r = Runner(s, g)
    for n in (1, 2, 4):
        check(s.job(Cmd.NAND_STEP, n, 0))
    r.cpu('cpu/state/initial', 'state')

    # ---- read-only comparisons: reserved region, main area, sequencer-written scratch page
    for label, ce, row in rows:
        reg, fmt = region(g, ce, row), page_format(g, region(g, ce, row))
        dma = r.cpu('cpu/raw-dma/' + label, 'read_raw', ce, row, r.raw_len, format='raw')
        fifo = r.cpu('cpu/raw-fifo/' + label, 'read_raw', ce, row, r.raw_len | 1 << 16, format='raw', path='fifo')
        dec = r.cpu('cpu/ecc/' + label, 'read_ecc', ce, row, {'main8': 0, 'main16': 1, 'rom': 2}[fmt], format=fmt)
        seq = seq_bytes(records.get('seq/raw/' + label, {}), r.raw_len)
        info = dict(region=reg, format=fmt, ce=ce, row=row)
        check_eq('raw dma == fifo ' + label, dma['data'][:r.raw_len], fifo['data'][:r.raw_len], **info)
        check_eq('raw cpu-dma == sequencer ' + label, dma['data'][:r.raw_len], seq, **info)
        host_data, host_meta, host_status = split_raw(g, dma['data'][:r.raw_len], fmt)
        checks.append(dict(name='host parity ' + label, kind='observation', status='observation',
                           sectors=host_status, **info))
        check_eq('ecc cpu-decode data == raw data ' + label, dec['data'], host_data, **info)
        if fmt == 'rom':
            if ce == 0:
                fw = r.fw_reserved('fw/reserved/' + label, row)
                check_eq('ecc cpu-decode == firmware boot read ' + label, dec['data'], fw['data'], **info)
        else:
            meta = b''.join(w.to_bytes(4, 'little') for w in dec['meta_words'])[:10]
            check_eq('ecc cpu-decode meta == raw meta ' + label, meta, host_meta, **info)
            fw = r.fw_page('fw/page/' + label, ce, row)
            if fw['job']['result'] == 0:
                check_eq('ecc cpu-decode == firmware read ' + label, dec['data'], fw['data'], **info)
            if label == 'seq-page':
                check_eq('cpu-decode == sequencer-written data', dec['data'], seq_data(), kind='required', **info)
                check_eq('cpu-decode meta == sequencer-written meta', meta, SEQ_META, kind='required', **info)

    # ---- scratch block: CPU erase, verify, CPU program with host parity, read back every way
    fmt = page_format(g, 'scratch')
    erase = r.cpu('cpu/erase', 'erase', g.ce, g.row, region='scratch')
    checks.append(dict(name='cpu erase completed', kind='required', status='match' if steps_ok(erase) else 'mismatch',
                       nand_status=erase.get('nand_status')))
    for page in (0, 1, g.pages - 1):
        rec = r.cpu('cpu/erase-verify/%d' % page, 'read_raw', g.ce, g.row + page, r.raw_len, format='raw')
        check_eq('erased page %d all 0xff' % page, rec['data'][:r.raw_len], b'\xff' * r.raw_len, kind='required',
                 region='scratch')
    raw = user_raw(g, cpu_data(), CPU_META)
    upload = raw + b'\xff' * (0x1400 - len(raw))
    prog = r.cpu('cpu/program', 'program', g.ce, g.row, len(raw), upload=upload, region='scratch', format=fmt,
                 raw_sha256=sha(raw), data_sha256=sha(cpu_data()), meta=CPU_META.hex(), parity='host ' + fmt)
    checks.append(dict(name='cpu program completed', kind='required', status='match' if steps_ok(prog) else 'mismatch',
                       nand_status=prog.get('nand_status')))
    dma = r.cpu('cpu/raw-dma/cpu-page', 'read_raw', g.ce, g.row, r.raw_len, format='raw')
    fifo = r.cpu('cpu/raw-fifo/cpu-page', 'read_raw', g.ce, g.row, r.raw_len | 1 << 16, format='raw', path='fifo')
    dec = r.cpu('cpu/ecc/cpu-page', 'read_ecc', g.ce, g.row, 0 if g.strength == 8 else 1, format=fmt)
    fw = r.fw_page('fw/page/cpu-page', g.ce, g.row)
    check_eq('cpu raw-dma == programmed raw', dma['data'][:len(raw)], raw, kind='required', region='scratch')
    check_eq('cpu raw-fifo == programmed raw', fifo['data'][:len(raw)], raw, kind='required', region='scratch')
    check_eq('cpu decode == programmed data', dec['data'], cpu_data(), kind='required', region='scratch')
    meta = b''.join(w.to_bytes(4, 'little') for w in dec['meta_words'])[:10]
    check_eq('cpu decode meta == programmed meta', meta, CPU_META, kind='required', region='scratch')
    check_eq('cpu decode ring results all 0', [e['r80c'] for e in dec['ecc']], [0] * 8, kind='required', region='scratch')
    check_eq('firmware read == programmed data', fw['data'], cpu_data(), kind='required', region='scratch')

    # ---- CPU-kicked encodes vs host BCH
    data = encode_data()
    for label, kick, slot in ENCODES:
        strength, metalen = (16 if kick & 4 else 8), kick >> 8 & 0xff
        upload = data + b'\xff' * (0x1200 - 512) + CPU_META + b'\xff\xff'
        rec = r.cpu('cpu/encode/' + label, 'encode', 0, 0, kick, slot, upload=upload, strength=strength,
                    metalen=metalen, data_sha256=sha(data), meta=CPU_META.hex() if metalen else None)
        plen = 13 if strength == 8 else 26
        got = parity_bytes(rec['ecc'][0]['par'], plen) if rec.get('ecc') else None
        expected = parity(strength, data + (CPU_META if metalen else b''))
        rec['parity'] = got.hex() if got else None
        rec['host_parity'] = expected.hex()
        check_eq('encode %s parity == host BCH%d' % (label, strength), got, expected, kind='required',
                 strength=strength)

    # ---- sequencer READ of the CPU-written page, then the final idle state
    p3 = run_phase(s, t, phase3(g), records, compact_cache, health)
    seq = p3.get('seq/decode/cpu-page', {})
    check_eq('sequencer decode == cpu-programmed data', seq_bytes(seq, 4096), cpu_data(), kind='required',
             region='scratch')
    meta = seq_bytes(seq, 10, 0x3000)
    check_eq('sequencer decode meta == cpu-programmed meta', meta, CPU_META, kind='required', region='scratch')
    check_eq('sequencer raw == cpu-programmed raw', seq_bytes(p3.get('seq/raw/cpu-page', {}), len(raw)), raw,
             kind='required', region='scratch')
    r.cpu('cpu/state/final', 'state')

    records.update(r.records)
    # Idle after every CPU job: both resets succeeded, sequencer idle (C08 0), FMC_CTRL0 0.
    cpu = {k: v for k, v in r.records.items() if 'post' in v}
    failed = [k for k, v in cpu.items() if not idle_ok(v)]
    health['cases'] = health.get('cases', 0) + len(cpu)
    health['verified'] = health.get('verified', 0) + len(cpu) - len(failed)
    health.setdefault('initial_reset_failed', []).extend(k for k, v in cpu.items() if v['pre_reset_rc'])
    health.setdefault('cleanup_failed', []).extend(k for k in failed if cpu[k]['pre_reset_rc'] == 0)
    health['cpu_jobs'] = r.jobs
    health['cpu_idle_verified'] = len(cpu) - len(failed)
    timing = {k: v['elapsed_us'] for k, v in cpu.items()}
    counts = {}
    for c in checks:
        counts[c['status']] = counts.get(c['status'], 0) + 1
    bad = [c['name'] for c in checks if c['status'] == 'mismatch']
    summary = ('%s: %d CPU-direct jobs, %d sequencer cases; checks %s; idle after %d/%d CPU jobs%s' %
               (NAME, r.jobs, len([k for k in records if k.startswith('seq/')]), counts,
                len(cpu) - len(failed), len(cpu), '; MISMATCH: ' + ', '.join(bad) if bad else ''))
    records['timing_us'] = timing
    return finish_family(s, records, compact_cache, health, checks, summary)


def finish_family(s, records, compact_cache, health, checks, summary):
    bad = [c['name'] for c in checks if c['status'] == 'mismatch']
    records['checks'] = {'list': checks}
    if bad:
        records['checks']['assessment'] = {'status': 'mismatch', 'checks': bad}
    s.last_batch_records = records
    s.last_batch_compact = compact_cache
    s.last_batch_health = health
    return records, summary


def static_cases(view, t):
    """Every resident probe case of both sequencer phases (preflight static check)."""
    g = Q.geometry(view)
    yield from phase1(g, page_rows(g))(view, t)
    yield from phase3(g)(view, t)


def entry():
    return {'static_cases': static_cases, 'name': NAME, 'status': 'followup', 'reason': cpu_nand_family.__doc__, 'question': cpu_nand_family.__doc__,
            'original': 'cpu-direct', 'fn': cpu_nand_family, 'source': 'nano4g/cpu_nand_experiments.py',
            'resident': True}


SPECS = {}  # function family (CPU jobs between resident phases): registered through experiments.entries()
