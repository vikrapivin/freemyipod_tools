"""CPU-only raw NAND campaign; no FMSS program executes in this family.

No device map: the CPU's reads of the reserved region and the main area are
settled by m2_cpu_stack; this family measures the controller path itself.

Every command experiment is a wire-sequence hypothesis, with its conventional
command bytes recorded. Unsupported commands are observations, never silently
classified as supported by a successful controller transfer. Readback against
distinct source patterns is required. Defaults target blocks 3000/3001; the
sequence API itself accepts every physical address and command.
"""
from dataclasses import dataclass, field
import hashlib
import struct

from . import sequences as Q
from .nand import check
from .protocol import Cmd

NAME = 'm2_cpu_direct'
CTRL0 = 0x01077801
SPARES = (0, 32, 64, 96, 112, 128, 160, 192, 218, 224, 256, 384, 512)
REPEATS = 3


def pattern(seed, n):
    """Position-dependent words: no 256-byte repetition at column boundaries."""
    out = bytearray()
    x = (seed + 1) * 0x9e3779b9 & 0xffffffff
    for i in range((n + 3) // 4):
        x ^= x << 13 & 0xffffffff
        x ^= x >> 17
        x ^= x << 5 & 0xffffffff
        out += (x & 0xffffffff).to_bytes(4, 'little')
    return bytes(out[:n])


def sha(data):
    return hashlib.sha256(data).hexdigest()


def pack_map_steps(steps):
    """Preserve the exact step words without millions of retained Python dicts."""
    from .nand import CPU_NAND_STEP_KINDS
    kinds = {name: kind for kind, name in CPU_NAND_STEP_KINDS.items()}
    out = bytearray()
    for step in steps:
        kind = step['kind'] if isinstance(step['kind'], int) else kinds[step['kind']]
        tag = kind << 24 | step['index'] << 16 | step['arg'] | int(step['timeout']) << 31
        out += struct.pack('<3I', tag, step['value'], step['t_us'])
    return out.hex()


@dataclass
class Case:
    name: str
    ce: int
    row: int
    topic: str
    steps: list = field(default_factory=list)
    payload: bytes = b''
    checks: list = field(default_factory=list)
    conditions: dict = field(default_factory=dict)
    required: bool = False
    output_bytes: int = 0

    def cmd(self, command):
        self.steps.append((1, command, 0, 0))
        return self

    def addr(self, value, cycles):
        self.steps.append((2, cycles, value & 0xffffffff, value >> 32))
        return self

    def page_addr(self, row, column=0):
        return self.addr(row << 16 | column, 5)

    def output(self, data):
        self.steps.append((3, len(data), len(self.payload), 0))
        self.payload += bytes(data)
        return self

    def input(self, n, expected=None, label='readback', path=0):
        off = self.output_bytes
        self.steps.append((4, n, off, path))
        self.output_bytes += n
        if expected is not None:
            self.checks.append(dict(label=label, offset=off, size=n, expected=bytes(expected).hex()))
        return self

    def ready(self, operation=None, row=None, mask=0x40):
        idx = len(self.steps)
        self.steps.append((5, 50000, 0x70, mask))
        if operation:
            self.conditions.setdefault('busy_intervals', []).append(dict(
                operation=operation, confirm_index=idx - 1, ready_index=idx,
                row=self.row if row is None else row))
        return self

    def select(self, ce):
        self.steps.append((8, ce, 0, 0))
        return self

    def timing(self, ctrl):
        self.steps.append((6, ctrl, 0, 0))
        return self

    def reset(self):
        return self.cmd(0xff).ready()

    def erase(self, row=None):
        row = self.row if row is None else row
        return self.cmd(0x60).addr(row, 3).cmd(0xd0).ready('erase', row)

    def program(self, data, row=None, column=0, confirm=0x10):
        row = self.row if row is None else row
        return self.cmd(0x80).page_addr(row, column).output(data).cmd(confirm).ready('program', row)

    def read(self, n, row=None, column=0, expected=None, label='readback', path=0):
        row = self.row if row is None else row
        return self.cmd(0).page_addr(row, column).cmd(0x30).ready('read', row).cmd(0).input(n, expected, label, path)

    def describe(self):
        return dict(topic=self.topic, ce=self.ce, row=self.row, required=self.required,
                    steps=[list(s) for s in self.steps], payload_sha256=sha(self.payload),
                    output_bytes=self.output_bytes, checks=self.checks, **self.conditions)


def geometry_cases(g):
    row = g.row
    for spare in SPARES:
        n = 4096 + spare
        for repeat in range(2):
            data = pattern(0x100 + spare + repeat * 1000, n)
            c = Case('geometry/%d/%d' % (n, repeat), 0, row, 'physical_size',
                     conditions=dict(program_bytes=n, read_bytes=4608, repeat=repeat))
            c.reset().erase().program(data).read(4608)
            # Compare each byte, retain the complete measured raw stream.  Bytes
            # beyond the physical end may repeat or read FF: no assumed limit.
            c.conditions['programmed_hex'] = data.hex()
            yield c


def roundtrip_cases(g):
    n = 4096 + g.spare
    for ce in range(4):
        for repeat in range(REPEATS):
            data = pattern(0x200 + ce * 10 + repeat, n)
            c = Case('roundtrip/ce%d/%d' % (ce, repeat), ce, g.row, 'ce_roundtrip', required=True,
                     conditions=dict(repeat=repeat))
            c.reset().erase().read(n, expected=b'\xff' * n, label='erased')
            c.program(data).read(n, expected=data, label='programmed-dma')
            c.read(n, expected=data, label='programmed-fifo', path=1)
            yield c


def command_cases(g):
    """Wire hypotheses include ordinary controls and independent readbacks.

    Every trial starts with chip RESET and (when writing) fresh erases. Cache,
    copyback and two-plane trials verify both source and destination pages;
    complete multi-command transactions stay inside one CPU job.
    """
    n, row, nextblock = 4096 + g.spare, g.row, (g.block + 1) * g.pages
    a, b = pattern(0x301, n), pattern(0x302, n)
    ce = g.ce

    c = Case('commands/random-column-read', ce, row, 'random_column_read')
    c.reset().erase().program(a).read(64, expected=a[:64], label='ordinary-control')
    for col, size in ((17, 113), (511, 69), (2048, 193), (4096, g.spare)):
        c.cmd(0x05).addr(col, 2).cmd(0xe0).input(size, a[col:col + size], 'column-%d' % col)
    yield c

    c = Case('commands/random-column-write', ce, row, 'random_column_write')
    want = bytearray(b'\xff' * n)
    c.reset().erase().cmd(0x80).page_addr(row).output(a[:128])
    want[:128] = a[:128]
    for col, size in ((1024, 173), (4096, g.spare), (512, 91)):
        c.cmd(0x85).addr(col, 2).output(a[col:col + size])
        want[col:col + size] = a[col:col + size]
    c.cmd(0x10).ready('program').read(n, expected=want)
    yield c

    c = Case('commands/partial-page', ce, row, 'partial_page_program')
    c.reset().erase()
    want = bytearray(b'\xff' * n)
    for part, (col, size) in enumerate(((0, 256), (1024, 512), (4096, g.spare))):
        want[col:col + size] = a[col:col + size]
        c.program(a[col:col + size], column=col).read(n, expected=want, label='part-%d' % part)
    yield c

    for kind, second in (('more-zeroes', bytes(x & y for x, y in zip(a, b))),
                         ('attempt-zero-to-one', bytes(~x & 255 for x in a))):
        c = Case('commands/overwrite/' + kind, ce, row, 'overwrite_without_erase')
        c.reset().erase().program(a).read(n, expected=a, label='first-program')
        c.program(second).read(n, expected=bytes(x & y for x, y in zip(a, second)), label='bitwise-and-hypothesis')
        yield c

    for order in ((3, 1, 2, 0), (g.pages - 1, 0, g.pages // 2)):
        c = Case('commands/out-of-order/' + '-'.join(map(str, order)), ce, row, 'out_of_order_program',
                 conditions=dict(page_order=list(order)))
        c.reset().erase()
        for page in order:
            c.program(pattern(0x400 + page, n), row + page)
        for page in order:
            c.read(n, row + page, expected=pattern(0x400 + page, n), label='page-%d' % page)
        yield c

    c = Case('commands/cache-program', ce, row, 'cache_program', conditions=dict(command_variant='80/address/data/15 then 80/address/data/10'))
    c.reset().erase().program(a, confirm=0x15).program(b, row + 1)
    c.read(n, expected=a, label='cache-first').read(n, row + 1, expected=b, label='cache-final')
    yield c

    for variant in ('sequential-31', 'random-00-address-31'):
        c = Case('commands/cache-read/' + variant, ce, row, 'cache_read', conditions=dict(command_variant=variant))
        c.reset().erase().program(a).program(b, row + 1)
        c.read(n, expected=a, label='ordinary-control')
        if variant.startswith('random'):
            c.cmd(0).page_addr(row + 1)
        c.cmd(0x31).ready('cache-read', row + 1).cmd(0).input(n, a, 'cache-current-hypothesis')
        c.cmd(0x3f).ready('cache-read-final', row + 1).cmd(0).input(n, b, 'cache-next-hypothesis')
        c.read(n, row + 1, expected=b, label='ordinary-next-control')
        yield c

    for target, label in ((row + 2, 'same-block'), (nextblock + 2, 'next-block')):
        c = Case('commands/copyback/' + label, ce, row, 'copyback',
                 conditions=dict(source_row=row, destination_row=target, command_variant='00/address/35,85/address/10'))
        c.reset().erase()
        if target >= nextblock:
            c.erase(nextblock)
        c.program(a).cmd(0).page_addr(row).cmd(0x35).ready('copyback-read')
        c.cmd(0x85).page_addr(target).cmd(0x10).ready('copyback-program', target)
        c.read(n, expected=a, label='source-unchanged').read(n, target, expected=a, label='destination')
        yield c

    for second_cmd in (0x80, 0x81):
        c = Case('commands/two-plane-program/%02x' % second_cmd, ce, row, 'two_plane_program',
                 conditions=dict(plane_rows=[row, nextblock], command_variant='80/11,%02x/10' % second_cmd))
        c.reset().erase().erase(nextblock).cmd(0x80).page_addr(row).output(a).cmd(0x11).ready('plane-intermediate')
        c.cmd(second_cmd).page_addr(nextblock).output(b).cmd(0x10).ready('plane-program', nextblock)
        c.read(n, expected=a, label='plane-0').read(n, nextblock, expected=b, label='plane-1')
        yield c

    c = Case('commands/two-plane-erase', ce, row, 'two_plane_erase',
             conditions=dict(plane_rows=[row, nextblock], command_variant='60/row,60/row,D0'))
    c.reset().erase().erase(nextblock).program(a).program(b, nextblock)
    c.cmd(0x60).addr(row, 3).cmd(0x60).addr(nextblock, 3).cmd(0xd0).ready('two-plane-erase')
    c.read(n, expected=b'\xff' * n, label='plane-0-erased').read(n, nextblock, expected=b'\xff' * n, label='plane-1-erased')
    yield c

    for ce in range(4):
        c = Case('commands/status78/ce%d' % ce, ce, row, 'status78', conditions=dict(command_variant='78/three row bytes/read8'))
        c.reset().cmd(0x78).addr(row, 3).input(8, path=1)
        yield c
        c = Case('commands/parameter-page/ce%d' % ce, ce, row, 'parameter_page', conditions=dict(command_variant='EC/00, wait 1ms, read three 256-byte copies'))
        c.reset().cmd(0xec).addr(0, 1)
        # Parameter-data mode need not survive a 70h command. Wait without one.
        c.steps.append((7, 1000, 0, 0))
        c.input(768, path=1)
        yield c
        for address in (0x01, 0x02, 0x10, 0x30, 0x80, 0x90):
            c = Case('commands/get-features/ce%d/%02x' % (ce, address), ce, row, 'get_features',
                     conditions=dict(feature_address=address, command_variant='EE/address/read4'))
            c.reset().cmd(0xee).addr(address, 1)
            c.steps.append((7, 10, 0, 0))
            c.input(4, path=1)
            yield c


def busy_cases(g):
    """Program every physical page in order, covering both MLC page classes.

    Page pairing is not known for this ID. Preserve individual page indices and
    repeat timings; the analyser reports measured fast/slow populations without
    inventing a lower/upper lookup table from even/odd page numbers.
    """
    n = 4096 + g.spare
    for ce in range(4):
        for repeat in range(REPEATS):
            c = Case('busy/ce%d/%d/erase-filled' % (ce, repeat), ce, g.row, 'busy_erase', required=True,
                     conditions=dict(repeat=repeat, prior='previous workload or previous full-block pass'))
            c.reset().erase()
            yield c
            c = Case('busy/ce%d/%d/erase-blank' % (ce, repeat), ce, g.row, 'busy_erase', required=True,
                     conditions=dict(repeat=repeat, prior='blank verified by preceding erase'))
            c.reset().erase()
            yield c
            for page in range(g.pages):
                data = pattern(0x1000 + page + ce * 128 + repeat * 512, n)
                c = Case('busy/ce%d/%d/page%d' % (ce, repeat, page), ce, g.row + page, 'busy_page', required=True,
                         conditions=dict(repeat=repeat, page=page, mlc_class='unknown; complete sequential block covers both classes'))
                c.reset().program(data).read(n, expected=data)
                yield c


def timing_cases(g):
    n = 4096 + g.spare
    for setup in range(8):
        for hold in range(8):
            ctrl = (CTRL0 & ~0x77000) | setup << 12 | hold << 16
            for repeat in range(REPEATS):
                a, b = pattern(0x2000 + repeat, n), pattern(0x2100 + repeat, n)
                c = Case('timing/%d-%d/%d' % (setup, hold, repeat), g.ce, g.row, 'bus_timing',
                         conditions=dict(setup=setup, hold=hold, ctrl0=ctrl, repeat=repeat,
                                         cycle_ns=(setup + hold + 2) * 7.06))
                c.reset().erase().program(a).read(n, expected=a, label='reference-control')
                c.timing(ctrl).read(n, expected=a, label='candidate-read')
                c.program(b, g.row + 1).timing(CTRL0).read(n, g.row + 1, expected=b, label='candidate-write-reference-read')
                yield c


def parity_cases(g):
    """What the sequencer driver does, by the CPU alone: READ ID and RESET on every CE, and
    program.s's interleave (one page per CE programmed concurrently, then each polled)."""
    for ce in range(4):
        for address in (0x00, 0x20, 0x40):
            c = Case('parity/readid/ce%d/%02x' % (ce, address), ce, g.row, 'read_id',
                     conditions=dict(command_variant='FF, 90/%02x, read 8' % address))
            c.reset().cmd(0x90).addr(address, 1).input(8)
            yield c
    n, row = 4096 + g.spare, (g.block + 1) * g.pages
    data = {ce: pattern(0x400 + ce, n) for ce in range(4)}
    c = Case('parity/interleaved-program', 0, row, 'interleaved_program', required=True,
             conditions=dict(ces=[0, 1, 2, 3], block=g.block + 1))
    c.reset()                           # case CE0 first, as every trial
    for ce in range(4):
        c.select(ce).reset().erase(row)
    for ce in range(4):                 # all four programs in flight before any status poll
        c.select(ce).cmd(0x80).page_addr(row).output(data[ce]).cmd(0x10)
    for ce in range(4):
        c.select(ce).ready('program', row)
    for ce in range(4):
        c.select(ce).read(n, row, expected=data[ce], label='ce%d' % ce)
    yield c


def cases(g):
    yield from parity_cases(g)
    yield from geometry_cases(g)
    yield from roundtrip_cases(g)
    yield from command_cases(g)
    yield from busy_cases(g)
    yield from timing_cases(g)


def status_checks(c, rec):
    """Every program/erase READY is checked; a later read cannot hide FAIL."""
    out = []
    for interval in c.conditions.get('busy_intervals', []):
        operation, index = interval['operation'], interval['ready_index']
        if not ('program' in operation or 'erase' in operation or operation == 'plane-intermediate'):
            continue
        markers = [s for s in rec.get('steps', []) if s.get('index') == index and
                   s.get('kind') in (12, 'status', 'nand_status')]
        if not markers:
            markers = [s for s in rec.get('steps', []) if s.get('index') == index and s.get('kind') in (6, 'ready')]
        values = {s.get('value') for s in markers if not s.get('timeout')}
        status = next(iter(values)) if len(values) == 1 else None
        verdict = ('unavailable' if status is None or rec.get('rc') != 0 else
                   'match' if status & 0x41 == 0x40 else 'mismatch')
        out.append(dict(label='status-%s-%d' % (operation, index), status=verdict,
                        ready_index=index, nand_status=status))
    return out


def checks_for(c, rec):
    data = bytes(rec.get('data', b''))
    ok = rec.get('rc') == 0 and rec.get('job', {}).get('result') == 0
    out = status_checks(c, rec)
    for check_ in c.checks:
        start, n = check_['offset'], check_['size']
        expected = bytes.fromhex(check_['expected'])
        got = data[start:start + n]
        out.append(dict(label=check_['label'], status=('match' if got == expected else 'mismatch') if ok and len(got) == n else 'unavailable',
                        expected_sha256=sha(expected), actual_sha256=sha(got), bytes_compared=len(got)))
    return out


def cpu_direct_family(s, t):
    from fmss.probe import jsonable
    g = Q.geometry(s)
    records, health = {}, dict(cases=0, verified=0, initial_reset_failed=[], cleanup_failed=[])
    s.last_batch_records, s.last_batch_compact, s.last_batch_health = records, {}, health
    for n in (1, 2, 4):
        check(s.job(Cmd.NAND_STEP, n, 0))

    pending_logs = []

    def flush():
        if hasattr(s, 'log_many'):
            s.log_many(pending_logs)
        else:
            for record in pending_logs:
                s.log(record)
        pending_logs.clear()

    def save(name, rec):
        records[name] = rec
        health['cases'] += 1
        if rec.get('pre_reset_rc') == 0 and rec.get('post_reset_rc') == 0 and rec.get('post', {}).get('c08') == 0:
            health['verified'] += 1
        elif rec.get('pre_reset_rc'):
            health['initial_reset_failed'].append(name)
        else:
            health['cleanup_failed'].append(name)
        # Same durable schema as resident probes: interrupted maps rebuild using
        # `n4g.py fmss rebuild` without rerunning any completed/timeout job.
        if hasattr(s, 'log'):
            pending_logs.append(dict(type='batch-case-result', experiment=NAME, case=name, data=jsonable(rec)))

    print('PLAN: m2_cpu_direct: CPU command trials (writes on blocks %d/%d, READ ID on every CE)' %
          (g.block, g.block + 1), flush=True)
    records['manifest'] = dict(pages_per_block=g.pages, mutation_blocks=[g.block, g.block + 1])
    if hasattr(s, 'log'):
        s.log(dict(type='batch-case-result', experiment=NAME, case='manifest', data=records['manifest']))
    from .cpu_batch import Job, run

    def save_job(item, job, out):
        if item.count:
            rec = dict(out or {}, job=job, conditions=dict(topic='block_map', ce=item.ce, row=item.row,
                       count=item.count, block=item.row // g.pages, read_only=True))
            rec['page_records_hex'] = bytes(rec.pop('data', b'')).hex()
            rec['step_records_hex'] = pack_map_steps(rec.pop('steps', []))
            rec.pop('pages', None)
        else:
            rec = dict(out or {}, job=job, conditions=item.case.describe())
            rec['comparisons'] = checks_for(item.case, rec)
            if 'data' in rec:
                rec['data_sha256'] = sha(rec['data'])
        save(item.name, rec)

    run(s, (Job(c.name, c.ce, c.row, case=c) for c in cases(g)), save_job, flush=flush)
    bad = [k for k, r in records.items() if r.get('conditions', {}).get('required') and
           (r.get('rc') != 0 or any(x['status'] != 'match' for x in r.get('comparisons', ())))]
    summary = '%s: %d CPU jobs; %d required-control failures; resets %d/%d' % (
        NAME, health['cases'], len(bad), health['verified'], health['cases'])
    records['checks'] = {'required_failures': bad, 'list': [dict(name=n, kind='required', status='mismatch') for n in bad]}
    if bad:
        records['checks']['assessment'] = {'status': 'mismatch', 'checks': bad}
    return records, summary


def static_cases(view, t):
    """No sequencer blobs: use validate_plan() for CPU wire transactions."""
    return iter(())


def validate_plan(g=None):
    """Pure construction preflight; does not predict undocumented NAND behavior."""
    g = g or Q.Geometry()
    errors, count, names = [], 0, set()
    for c in cases(g):
        count += 1
        if c.name in names:
            errors.append(c.name + ': duplicate name')
        names.add(c.name)
        if not 0 <= c.ce < 4 or not 0 <= c.row < g.blocks * g.pages:
            errors.append(c.name + ': address outside chip')
        if len(c.steps) > 128 or len(c.payload) > 65536 or c.output_bytes > 65536:
            errors.append(c.name + ': sequence buffer capacity')
        for kind, a, b, d in c.steps:
            if kind == 3 and (not a or b + a > len(c.payload)):
                errors.append(c.name + ': payload slice')
            if kind == 4 and (not a or b + a > c.output_bytes):
                errors.append(c.name + ': output slice')
        for x in c.checks:
            if len(bytes.fromhex(x['expected'])) != x['size']:
                errors.append(c.name + ': expected length')
    return dict(family=NAME, cases=count, 
                problems=errors, validation='construction only; undocumented commands remain device observations')


def static_validate(view, t):
    from .cpu_direct import sequence_upload
    g = Q.geometry(view)
    report = validate_plan(g)
    for c in cases(g):
        sequence_upload(c.steps, CTRL0, c.payload)
    if report['problems']:
        raise ValueError('; '.join(report['problems']))
    return report


def preflight(s, t):
    """Native known-command smoke plus all-template construction validation."""
    from .cpu_batch import Job, run
    g = Q.geometry(s)
    report = static_validate(s, t)
    for n in (1, 2, 4):
        check(s.job(Cmd.NAND_STEP, n, 0))
    c = next(roundtrip_cases(g))
    captures = []
    run(s, [Job(c.name, c.ce, c.row, case=c), Job('map-control', c.ce, c.row, 2)],
        lambda item, job, out: captures.append((job, out)))
    job, out = captures[0]
    rec = dict(out or {}, job=job)
    tests = checks_for(c, rec)
    if rec.get('rc') or job.get('result') or any(x['status'] != 'match' for x in tests):
        raise ValueError('native CPU sequence control failed: %r' % tests)
    job, out = captures[1]
    if job.get('result') or not out or out.get('rc') or len(out.get('data', b'')) != 128:
        raise ValueError('native CPU map control failed')
    report['native_controls'] = 2
    report['native_transport'] = 'resident CPU batch (sequence + map)'
    report['device_only'] = 'Unsupported-command semantics, real busy times, spare capacity and complete physical map require device capture.'
    return report, 'CPU native sequence/map controls passed; all %d templates constructed; full map and undocumented chip behavior require hardware' % report['cases']


def entry():
    return dict(name=NAME, status='followup', reason=cpu_direct_family.__doc__ or __doc__.split('\n')[0],
                question='CPU-only NAND physical geometry, all-block map, commands and timing',
                original='cpu-only', fn=cpu_direct_family, source='nano4g/cpu_direct_experiments.py',
                resident=True, static_cases=static_cases, static_validate=static_validate, preflight=preflight)
