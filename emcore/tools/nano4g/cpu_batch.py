"""Resident CPU-direct sequence/map jobs, using the M2 batch arenas/worker.

One upload per bounded group; results drain with an exact cursor. A failed or
lost RUN is never replayed and its arenas are left owned by the worker.
"""
from dataclasses import dataclass
import hashlib
from itertools import islice
import struct
import time
import zlib

from .cpu_direct import sequence_upload, cpu_nand_sequence, cpu_nand_map, CTRL0
from .nand import check, parse_cpu_nand
from .protocol import Cmd, ToolError

MAGIC, OUTPUT = 0x31425443, 0x314f5443
COUNT = 128
CAPACITY = 2 * 1024 * 1024


@dataclass
class Job:
    name: str
    ce: int
    row: int
    count: int = 0                 # nonzero means a map job
    case: object = None           # otherwise a CPU sequence Case

    @property
    def timeout(self):
        if self.count:
            return max(120.0, self.count * 2.0)
        # READY and DELAY are explicit device intervals. Include transfer/reset
        # overhead without imposing a per-case host wait.
        return 10.0 + sum(a for kind, a, _, _ in self.case.steps if kind in (5, 7)) / 1e6


def pack(jobs):
    if not 1 <= len(jobs) <= COUNT:
        raise ToolError('CPU manifest needs 1..%d jobs' % COUNT)
    blob = bytearray(32 + 32 * len(jobs))
    shared = {}
    for i, job in enumerate(jobs):
        if not 0 <= job.ce < 8 or not 0 <= job.row <= 0xffffffff:
            raise ToolError('invalid CPU job address')
        if job.count:
            if not 1 <= job.count <= 256:
                raise ToolError('CPU map count must be 1..256')
            upload = b''
        else:
            upload = sequence_upload(job.case.steps, CTRL0, job.case.payload)
        if upload not in shared:
            blob.extend(bytes(-len(blob) & 3))
            shared[upload] = len(blob)
            blob.extend(upload)
        struct.pack_into('<8I', blob, 32 + 32 * i, i, (7 if job.count else 6) | job.ce << 8,
                         job.row, job.count, shared[upload], len(upload), 0, 0)
    blob.extend(bytes(-len(blob) & 3))
    struct.pack_into('<8I', blob, 0, MAGIC, 1, len(jobs), 32, len(blob), 0, 0, 0)
    return bytes(blob)


def parse_page(raw, jobs, first):
    if len(raw) < 32:
        raise ToolError('short CPU batch output')
    magic, version, total, start, next_, used, errors, elapsed = struct.unpack_from('<8I', raw)
    if (magic, version, total, start, used) != (OUTPUT, 1, len(jobs), first, len(raw)) or not first < next_ <= total:
        raise ToolError('invalid CPU batch header/cursor')
    pos, records, actual_errors = 32, [], 0
    for i in range(first, next_):
        if len(raw) - pos < 32:
            raise ToolError('truncated CPU batch record')
        ident, result, rc, head, data, d0, d1, reserved = struct.unpack_from('<8I', raw, pos)
        size = (32 + head + data + 3) & ~3
        if (ident != i or reserved or size > len(raw) - pos or data > 65536 or
                (not head and (data or not (result or rc)))):
            raise ToolError('invalid CPU batch record')
        out = None
        if head:
            if head < 0x304 or head > 0x304 + 512 * 12:
                raise ToolError('invalid CPU capture size')
            body = raw[pos + 32:pos + 32 + head]
            words = struct.unpack_from('<15I', body)
            job = jobs[i]
            if (words[2:6] != (7 if job.count else 6, job.ce, job.row, job.count) or
                    head != 0x304 + words[9] * 12 or words[14] > 8 or words[13] != data):
                raise ToolError('CPU capture does not match request or record bounds')
            out = parse_cpu_nand(body.ljust(0x2000, b'\0') + raw[pos + 32 + head:pos + 32 + head + data])
            if job.count:
                if data % 64 or data > job.count * 64:
                    raise ToolError('invalid CPU map table')
                out['map_rawsize'] = out['unused']
        status = dict(result=result, lib_rc=rc - (1 << 32) if rc & 0x80000000 else rc,
                      detail=[d0, d1, 0, 0], result_len=0x2000 + data if head else 0)
        records.append((jobs[i], status, out))
        actual_errors += bool(result or rc)
        pos += size
    if pos != len(raw) or errors != actual_errors:
        raise ToolError('CPU batch trailing bytes/error count mismatch')
    return next_, records, dict(cases=next_ - first, errors=errors, elapsed_us=elapsed, bytes=used)


def run(s, jobs, save, name='m2_cpu_direct', capacity=CAPACITY, flush=None):
    """Execute in order, calling save(job, status, capture) before the next RUN.

    The explicit --single-probes diagnostic path keeps the same journal schema.
    Map/sequence phase separation is owned by the caller.
    """
    if not getattr(s, 'batch_enabled', True):
        for job in jobs:
            if job.count:
                status, out = cpu_nand_map(s, job.ce, job.row, job.count, timeout=job.timeout)
            else:
                status, out = cpu_nand_sequence(s, job.case.steps, ce=job.ce, row=job.row,
                                                payload=job.case.payload, timeout=job.timeout)
            save(job, status, out)
            if flush:
                flush()
        return
    jobs = iter(jobs)
    timing = dict(upload_bytes=0, download_bytes=0, device_runs=0, worker_seconds=0,
                  upload_seconds=0, command_seconds=0, download_seconds=0, processing_seconds=0)
    started = time.monotonic()
    while group := list(islice(jobs, COUNT)):
        blob = pack(group)
        s.active_batch = dict(experiment=name, cases=[j.name for j in group])
        s.log(dict(type='cpu-batch-plan', experiment=name, cases=[j.name for j in group],
                   request=blob.hex(), sha256=hashlib.sha256(blob).hexdigest(), output_capacity=capacity))
        print('PLAN: %s CPU batch: %d jobs (%s through %s), %d input bytes' %
              (name, len(group), group[0].name, group[-1].name, len(blob)), flush=True)
        allocated, completed = False, True
        try:
            st = check(s.job(Cmd.NAND_BATCH_ALLOC, len(blob), capacity))
            addr, incap, outaddr, outcap = st.job_detail
            allocated = True
            if incap != len(blob) or outcap != capacity:
                raise ToolError('CPU batch allocation size mismatch')
            mark = time.monotonic()
            s.write_memory(addr, blob)
            timing['upload_seconds'] += time.monotonic() - mark
            timing['upload_bytes'] += len(blob)
            first = 0
            while first < len(group):
                completed = False
                mark = time.monotonic()
                st = s.job(Cmd.NAND_BATCH_RUN, len(blob), zlib.crc32(blob) & 0xffffffff,
                           d=first, timeout=30 + sum(j.timeout for j in group[first:]), poll_interval=0.25)
                completed = True
                check(st)
                timing['command_seconds'] += time.monotonic() - mark
                next_, used, actual_out, errors = st.job_detail
                if actual_out != outaddr or not 32 <= used <= capacity:
                    raise ToolError('CPU batch result outside allocated arena')
                mark = time.monotonic()
                raw = s.read(outaddr, used)
                timing['download_seconds'] += time.monotonic() - mark
                mark = time.monotonic()
                s.log(dict(type='cpu-batch-output', experiment=name, first=first, data=raw.hex(),
                           sha256=hashlib.sha256(raw).hexdigest()))
                parsed, records, stats = parse_page(raw, group, first)
                if parsed != next_ or errors != stats['errors']:
                    raise ToolError('CPU batch status/output mismatch')
                try:
                    for job, status, out in records:
                        save(job, status, out)
                finally:
                    if flush:
                        flush()
                s.log(dict(type='cpu-batch-page', experiment=name, first=first, next=next_, **stats))
                print('%s: collected %d/%d CPU jobs (%d bytes)' % (name, next_, len(group), used), flush=True)
                first = next_
                timing['processing_seconds'] += time.monotonic() - mark
                timing['device_runs'] += 1
                timing['download_bytes'] += used
                timing['worker_seconds'] += stats['elapsed_us'] / 1e6
        finally:
            if allocated and completed:
                check(s.job(Cmd.NAND_BATCH_FREE))
    timing['total_seconds'] = time.monotonic() - started
    s.log(dict(type='cpu-batch-timing', experiment=name, **timing))
    sink = getattr(s, 'family_timing', None)
    if isinstance(sink, dict):
        for key, value in timing.items():
            sink[key] = sink.get(key, 0) + value
