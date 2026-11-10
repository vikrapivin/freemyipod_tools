"""Resident emCORE benchmark: one shared NAND API, M2 or CPU physical execution.

Both backends use hardware BCH. The firmware checks every returned byte against
known data/metadata; timer intervals exclude USB, pattern generation and checking.
"""
import statistics
import struct

from .cpu_nand_experiments import finish_family
from .nand import stack_prepare
from .protocol import Cmd, ToolError
from .sequences import geometry

NAME = 'm2_driver_compare'
PROFILE_NAME = 'm2_driver_profile'
PHASES = ('reset', 'lcd_dma_wait', 'setup', 'execution', 'cleanup')
HEADER = 'magic version bytes record_bytes count ce block pages repetitions strength failures previous_backend previous_ecc elapsed_us complete reserved'.split()
FIELDS = 'op backend writer row rc elapsed_us compared_bytes mismatch_bytes first_offset expected actual data_crc meta_crc status0 status1 meta0 meta1 meta2 erased reserved1 reserved2 reserved3 reserved4 reserved5'.split()
BACKENDS = ('m2', 'cpu')
OPS = {1: 'read', 2: 'write', 3: 'erase'}


def decode(raw):
    if len(raw) < 64:
        raise ToolError('Missing resident NAND comparison header; load the updated emCORE image')
    h = dict(zip(HEADER, struct.unpack_from('<16I', raw)))
    if h['magic'] != 0x3143424e or h['version'] not in (1, 2) or h['record_bytes'] != 96:
        raise ToolError('Unsupported NAND comparison ABI')
    if h['bytes'] != len(raw) or len(raw) != 64 + 96 * h['count']:
        raise ToolError('Truncated NAND comparison records')
    records = []
    for i in range(h['count']):
        r = dict(zip(FIELDS, struct.unpack_from('<24I', raw, 64 + 96 * i)))
        if r['op'] not in OPS or r['backend'] not in (0, 1) or r['writer'] not in (0, 1):
            raise ToolError('Malformed NAND comparison record')
        if r['rc'] & 0x80000000: r['rc'] -= 1 << 32
        r['sector_status'] = list(struct.pack('<2I', r.pop('status0'), r.pop('status1')))
        r['metadata'] = struct.pack('<3I', *(r.pop('meta%d' % k) for k in range(3)))[:10].hex()
        if h['version'] == 2:
            if r['erased'] & ~0x301:
                raise ToolError('Malformed NAND profile flags')
            r['profiled'] = bool(r['erased'] & 0x100)
            r['cpu_baseline'] = bool(r['erased'] & 0x200)
            r['erased'] &= 1
            r['phase_us'] = dict(zip(PHASES, (r['reserved%d' % k] for k in range(1, 6))))
            remainder = r['elapsed_us'] - sum(r['phase_us'].values())
            if remainder < 0 or (not r['profiled'] and any(r['phase_us'].values())):
                raise ToolError('Inconsistent exclusive NAND phase times')
            if r['profiled']: r['phase_us']['other'] = remainder
        r = {k: v for k, v in r.items() if not k.startswith('reserved')}
        records.append(r)
    return h, records


def checks_for(h, rows, job):
    checks = []
    def check(name, ok):
        checks.append(dict(name=name, kind='required', status='match' if ok else 'mismatch'))
    check('worker completed', job['result'] == 0 and job['lib_rc'] == 0)
    check('whole workload completed', h['complete'] == 1 and
          h['count'] == 6 + 2 * h['pages'] + 4 * h['pages'] * h['repetitions'])
    check('no recorded failures', h['failures'] == 0)
    if h.get('version') == 2:
        check('profile controls requested', h['repetitions'] % 4 == 0 and h['repetitions'] >= 4)
        controls = {(r.get('profiled'), r.get('cpu_baseline')) for r in rows
                    if r['op'] == 1 and r['backend'] == 1 and not r['erased']}
        check('all CPU read controls present', controls == {(True, True), (True, False), (False, True), (False, False)})
    for i, r in enumerate(rows):
        valid = r['rc'] in (0, 1) if r['op'] == 1 else r['rc'] == 0
        check('%d/%s/%s completed' % (i, BACKENDS[r['backend']], OPS[r['op']]), valid)
        if r['op'] == 1:
            check('%d/full data and metadata exact' % i, r['compared_bytes'] == 4106 and r['mismatch_bytes'] == 0)
        if h.get('version') == 2:
            phases = r.get('phase_us', {})
            check('%d/exclusive phase accounting' % i,
                  set(phases) == set(PHASES + (('other',) if r.get('profiled') else ())) and
                  all(isinstance(v, int) and v >= 0 for v in phases.values()) and
                  sum(phases.values()) == (r['elapsed_us'] if r.get('profiled') else 0))
            check('%d/CPU baseline scope' % i,
                  not r.get('cpu_baseline') or (r['op'] == 1 and r['backend'] == 1))
    return checks


def timing_key(r):
    backend = BACKENDS[r['backend']]
    if r.get('cpu_baseline'): backend += '-baseline'
    if 'profiled' in r: backend += '-profiled' if r['profiled'] else '-plain'
    return '%s/%s' % (backend, OPS[r['op']])


def phase_timings(rows):
    groups = {}
    for r in rows:
        if not r.get('profiled') or r['rc'] not in (0, 1) or r['mismatch_bytes']: continue
        if r['op'] != 1 and r['rc']: continue
        if r['op'] == 1 and r['erased']: continue
        g = groups.setdefault(timing_key(r), dict(count=0, total_us=0, phases_us={}))
        g['count'] += 1
        g['total_us'] += r['elapsed_us']
        for phase, duration in r['phase_us'].items():
            g['phases_us'][phase] = g['phases_us'].get(phase, 0) + duration
    return groups


def timings(rows):
    """Aggregate worker API durations; failed reads never count as throughput."""
    groups = {}
    for r in rows:
        if r['erased'] and r['op'] == 1: continue
        if r['rc'] not in ((0, 1) if r['op'] == 1 else (0,)) or r['mismatch_bytes']: continue
        key = timing_key(r)
        groups.setdefault(key, []).append(r['elapsed_us'])
    result = {}
    for key, values in groups.items():
        total = sum(values)
        item = dict(count=len(values), total_us=total, min_us=min(values),
                    median_us=statistics.median(values), max_us=max(values),
                    p95_us=sorted(values)[max(0, (95 * len(values) + 99) // 100 - 1)])
        if key.endswith('/erase'):
            item['blocks_per_second'] = len(values) * 1e6 / total if total else None
        else:
            item['MiB_per_second'] = len(values) * 4096 * 1e6 / (total * 1024 * 1024) if total else None
        result[key] = item
    return result


def run(s, t, profile=False):
    """Shared page/block driver: resident CPU-vs-M2 cross-read correctness and device read/write/erase speeds."""
    name = PROFILE_NAME if profile else NAME
    g = geometry(s)
    pages, repetitions = getattr(s, 'driver_compare_pages', 32), getattr(s, 'driver_compare_repetitions', 4)
    if not 1 <= pages <= g.pages or not 1 <= repetitions <= 255:
        raise ToolError('Comparison pages/repetitions exceed geometry or request encoding')
    if profile and repetitions % 4:
        raise ToolError('Profile needs read repetitions in multiples of four for CPU baseline/optimized and profiler controls')
    if g.block + 1 >= g.blocks:
        raise ToolError('Comparison needs two adjacent blocks within the chip')
    st = stack_prepare(s)
    count = 6 + 2 * pages + 4 * pages * repetitions
    if 64 + 96 * count > st.result_cap:
        raise ToolError('Comparison records exceed the device result buffer; reduce pages or repetitions')
    records, checks, all_rows = {}, [], []
    s.last_batch_records = records
    for ce in range(4):
        print('PLAN: resident driver comparison CE%d blocks %d,%d: %d pages/backend, %d read repetitions' %
              (ce, g.block, g.block + 1, pages, repetitions), flush=True)
        st = s.job(Cmd.NAND_COMPARE, ce, g.block, d=pages | repetitions << 16 | g.strength << 24 | int(profile) << 31, timeout=180)
        if not st.result_len:
            raise ToolError('Comparison returned no records: worker result %d, library rc 0x%x' %
                            (st.job_result, st.job_lib_rc))
        raw = s.read(st.result_addr, st.result_len)
        h, rows = decode(raw)
        if profile and h['version'] != 2: raise ToolError('Firmware did not return the requested profile')
        job = dict(result=st.job_result, lib_rc=st.job_lib_rc, cpu_clock_mhz=st.clk_mhz)
        label = 'ce%d' % ce
        rec = dict(header=h, job=job, operations=rows, timing=timings(rows),
                   timing_qualified=bool(h['complete'] and not h['failures']))
        records[label] = rec
        s.log(dict(type='batch-case-result', experiment=name, case=label, data=rec))
        checks.extend(dict(c, name=label + '/' + c['name']) for c in checks_for(h, rows, job))
        all_rows.extend(rows)
        records['checks'] = {'list': checks}
        if hasattr(s, 'checkpoint_cases'): s.checkpoint_cases(name, records)
        # An incomplete worker may have a timed-out controller. Never retry it.
        if not h['complete']: break
    checks.append(dict(name='all four CEs completed', kind='required', status='match' if
                       all(records.get('ce%d' % ce, {}).get('header', {}).get('complete') for ce in range(4)) else 'mismatch'))
    records['timing'] = dict(scope='Device API duration, including per-call setup/cleanup; excludes USB, '
                            'pattern generation, checksums and verification. Read/write MiB/s counts 4096-byte payloads. '
                            'Writes are single-page programs; reads are repeated sequential passes, not FTL throughput.',
                            measurements=timings(all_rows))
    if profile:
        records['timing']['phases'] = phase_timings(all_rows)
        records['timing']['scope'] += (' Profiled and plain timings are kept separate. CPU baseline retains the old diagnostic read path; '
            'CPU uses the optimized read path. Execution combines NAND, DMA and BCH; other includes shared policy and unclassified preparation.')
    bad = sum(c['status'] != 'match' for c in checks)
    records['timing']['qualified'] = bad == 0
    for key, v in records['timing']['measurements'].items():
        speed = v.get('MiB_per_second')
        if speed is not None:
            print('%s: %.3f MiB/s, median %.1f us, p95 %d us%s' %
                  (key, speed, v['median_us'], v['p95_us'],
                   '' if not bad else ' (correctness checks failed)'), flush=True)
    summary = '%s: %d operations, %d checks, %d mismatches' % (name, len(all_rows), len(checks), bad)
    result = finish_family(s, records, {}, {}, checks, summary)
    for label in ('checks', 'timing'):
        s.log(dict(type='batch-case-result', experiment=name, case=label, data=records[label]))
    return result


def entry():
    return dict(name=NAME, status='followup', reason=run.__doc__, question=run.__doc__, original='shared-driver',
                fn=run, resident=True, source='nano4g/driver_compare.py')


def run_profile(s, t):
    """Device CPU read optimization A/B, M2 control, and exclusive API phase timings; all bytes verified."""
    return run(s, t, profile=True)


def profile_entry():
    return dict(name=PROFILE_NAME, status='followup', reason=run_profile.__doc__, question=run_profile.__doc__,
                original='shared-driver-profile', fn=run_profile, resident=True, source='nano4g/driver_compare.py')
