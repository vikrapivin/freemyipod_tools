"""Resident M2 experiment manifests: upload once, execute locally, fetch packed results.

The input deduplicates scratch initializers. Results retain snapshots and all
requested samples; scratch patches are lossless. Only a full result arena needs
a host drain/resume, using an exact next-case cursor without replay.
"""
from dataclasses import dataclass
import hashlib
import struct
import zlib
import time
from fmss import probe as P
from .protocol import Cmd, ToolError, MUTATE, BATCH2
from .nand import check
from .capture import compact
from . import fault_window as FW

MAGIC, OUTPUT = 0x3142544d, 0x314f544d
INPUT_MAX, OUTPUT_MAX = 24 * 1024 * 1024, 16 * 1024 * 1024
HEAD = 2136
DIAGNOSTICS = 196  # v3: stage + two 24-word reset records
DIAGNOSTIC_BYTES = {1: 0, 2: 68, 3: DIAGNOSTICS}


@dataclass
class Prepared:
    case: object
    request: bytes
    params: list
    flags: int
    scratch: object
    scratch_read: int
    sample_limit: int
    timeout_us: int


def prepare(case, t, timeout_floor=0):
    opts = dict(case.options)
    code = case.source if isinstance(case.source, bytes) else P.assemble(t, case.source)
    flags = opts.get('flags', 0) | opts.get('steps', 0) | t.probe_flags
    clean = opts.get('clean', True) and not flags & P.S_NO_CLEAN
    params = (list(t.clean_params) if clean else []) + list(opts.get('params', ()))
    late, late2 = opts.get('late', ()), opts.get('late2', ())
    timeout = max(opts.get('timeout_us', 100000) or 100000, timeout_floor)
    scratch = opts.get('scratch')
    read = opts.get('scratch_read', 256) if scratch is not None else 0
    arguments = dict(params=params, timeout_us=timeout, flags=flags, late=late,
                     late_us=opts.get('late_us', 0), start_value=opts.get('start_value', 0),
                     late2=late2, late2_us=opts.get('late2_us', 0))
    t.validate_probe(code, scratch=scratch, scratch_read=read, **arguments)
    request = P.request(t, code, **arguments)
    return Prepared(case, request, params, flags, scratch, read, t.max_samples, timeout or 100000)


def pack(prepared, extended=False):
    if not prepared or len(prepared) > 4096:
        raise ToolError('batch needs 1..4096 cases')
    payload = bytearray(32 + (40 if extended else 32) * len(prepared))
    action_table = 32 + 32 * len(prepared) if extended else 0
    shared = {}
    def add(data):
        data = bytes(data)
        if data not in shared:
            payload.extend(bytes((-len(payload)) & 3))
            shared[data] = len(payload)
            payload.extend(data)
        return shared[data]
    worst = 32
    for i, item in enumerate(prepared):
        nparams, prog_bytes = struct.unpack_from('<II', item.request, 16)
        compact = item.request[:44 + 8 * nparams] + item.request[0x400:0x400 + prog_bytes]
        req = add(compact)
        scratch = b'' if item.scratch is None else bytes(item.scratch)
        flags = int(item.case.pins)
        if extended and scratch:
            chunks = [scratch[k:k+64].ljust(64, b'\0') for k in range(0, len(scratch), 64)]
            runs = []
            for chunk in chunks:
                off = add(chunk)
                if runs and runs[-1][1] == off: runs[-1][0] += 1
                else: runs.append([1, off])
            packed = struct.pack('<2I', len(scratch), len(runs)) + b''.join(struct.pack('<2I', *r) for r in runs)
            initial, stored = add(packed), len(packed)
            flags |= 2
        else: initial, stored = add(scratch), len(scratch)
        actions = item.case.options.get('actions', ())
        if actions and not extended: raise ToolError('Resident dependencies require the updated kernel; reload first')
        if extended:
            action_data = b''.join(struct.pack('<4I', *a) for a in actions)
            struct.pack_into('<2I', payload, action_table + i * 8, add(action_data), len(actions))
        struct.pack_into('<8I', payload, 32 + i * 32, i, req, len(compact), initial, stored,
                         item.scratch_read, item.sample_limit, flags)
        worst += (32 + DIAGNOSTICS + HEAD + item.sample_limit * 40 +
                  (8192 if item.flags & P.FULLMAP else 0) + item.scratch_read + 3) & ~3
    payload.extend(bytes((-len(payload)) & 3))
    if len(payload) > INPUT_MAX:
        raise ToolError('experiment manifest exceeds 24 MiB input arena')
    struct.pack_into('<8I', payload, 0, MAGIC, 2 if extended else 1, len(prepared), 32, len(payload), action_table, 0, 0)
    return bytes(payload), min(OUTPUT_MAX, max(128 * 1024, worst), 26 * 1024 * 1024 - len(payload))


def parse_page(raw, t, prepared, first):
    if len(raw) < 32: raise ToolError('short batch output')
    magic, version, total, start, next_case, used, errors, elapsed = struct.unpack_from('<8I', raw)
    if magic != OUTPUT or version not in DIAGNOSTIC_BYTES or total != len(prepared) or start != first or \
            not first < next_case <= total or used != len(raw):
        raise ToolError('invalid batch output header/cursor')
    pos, records = 32, []
    for i in range(first, next_case):
        if len(raw) - pos < 32: raise ToolError('truncated batch record')
        ident, rc, size, head, samples, maps, scratch_bytes, encoding = struct.unpack_from('<8I', raw, pos)
        item = prepared[i]
        need = 32 + head + samples * 40 + maps + scratch_bytes + DIAGNOSTIC_BYTES[version]
        if ident != i or head != HEAD or samples > item.sample_limit or \
                maps != (8192 if item.flags & P.FULLMAP else 0) or encoding not in (0, 1) or \
                size != ((need + 3) & ~3) or size > len(raw) - pos:
            raise ToolError('invalid batch record layout')
        p = pos + 32
        if rc not in (0, 0xfffffffe):
            rec = {'case_error': 'ProbeRejected' if rc == 0xffffffff else 'DependencyError' if rc == 0xfffffffc else 'PrerequisiteError', 'lib_rc': rc}
        else:
            rec = P.parse_out(t, raw[p:p + head], raw[p + head:p + head + samples * 40])
            if struct.unpack_from('<I', raw, p + 4)[0] < t.need_version:
                raise ToolError('batch contains an obsolete probe capture')
            p += head + samples * 40
            if maps:
                rec['fullmap'] = [list(struct.unpack_from('<1024I', raw, p)),
                                  list(struct.unpack_from('<1024I', raw, p + 4096))]
                p += maps
            if encoding:
                if item.scratch is None or len(item.scratch) < item.scratch_read or scratch_bytes % 8:
                    raise ToolError('invalid scratch delta initializer')
                result = bytearray(item.scratch[:item.scratch_read])
                previous = -1
                for off, word in struct.iter_unpack('<II', raw[p:p + scratch_bytes]):
                    if off & 3 or off <= previous or off >= len(result):
                        raise ToolError('invalid scratch patch offset')
                    n = min(4, len(result) - off)
                    result[off:off + n] = struct.pack('<I', word)[:n]
                    previous = off
                rec['scratch'] = bytes(result)
            else:
                if scratch_bytes != item.scratch_read: raise ToolError('wrong scratch result size')
                rec['scratch'] = raw[p:p + scratch_bytes] if item.scratch is not None else None
            rec['worker'] = {'result': int(rc != 0), 'lib_rc': rc, 'batch_case': i}
            from .protocol import executed, STALL_MARK
            rec['execution'] = ('stall-safe' if rec.get('dropped') == STALL_MARK else
                                'sampled' if rec['polls'] else 'not-started')
            if not executed(rec) or not rec['pre_ok']:
                rec['execution_error'] = 'candidate-not-executed'
            elif rc:
                rec['execution_error'] = 'worker-cleanup-failed'
            rec['params_ok'] = all(rec['before'].get(o) == (v & 0xffffffff)
                                   for o, v in dict(item.params).items()
                                   if 0xc00 <= o < 0xd00 and o not in t.not_readback)
        if version >= 2:
            words = 8 if version == 2 else 24
            diag = struct.unpack_from('<%dI' % (1 + words * 2), raw,
                                      pos + need - DIAGNOSTIC_BYTES[version])
            names = ('method', 'status', 'irq', 'pc', 'after_status', 'after_irq', 'rc', 'elapsed_us')
            state_names = ('phase', 'control', 'status', 'bufrst', 'bufok', 'bufcfg', 'fmc_control', 'fmc_status')
            rec['diagnostics'] = {'stage': diag[0]}
            for index, name in enumerate(('initial', 'cleanup')):
                part = diag[1 + index * words:1 + (index + 1) * words]
                decoded = dict(zip(names, part[:8]))
                if decoded['method'] & 0x01000000:
                    decoded['recovery_status'] = {
                        'after_abort': (decoded['method'] >> 8) & 0xff,
                        'after_forced_reset': (decoded['method'] >> 16) & 0xff}
                if version == 3:
                    decoded['first_failure'] = dict(zip(state_names, part[8:16]))
                    decoded['final_state'] = dict(zip(state_names, part[16:24]))
                rec['diagnostics'][name] = decoded
        records.append((i, rec))
        pos += size
    if pos != len(raw): raise ToolError('trailing batch output')
    return next_case, records, {'cases': next_case - first, 'errors': errors, 'elapsed_us': elapsed, 'bytes': used}


def reset_health(prepared, records):
    """Per-family proof that each independent case started from a verified reset.

    A case is verified when its initial reset reached phase 6 with rc 0 and the
    END-only preflight then completed (pre_ok 1). Cases that request no reset
    or no preflight are counted separately. Failures are listed, never hidden.
    """
    out = {'cases': 0, 'verified': 0, 'skipped_by_request': 0, 'initial_reset_failed': [],
           'preflight_failed': [], 'cleanup_failed': [], 'recovery_methods': {}, 'longest_failure_run': 0}
    run_length = 0
    for i, rec in records:
        item = prepared[i]
        out['cases'] += 1
        if 'case_error' in rec:
            failed = rec['case_error'] in ('PrerequisiteError', 'DependencyError')
            run_length = run_length + 1 if failed else 0
            out['longest_failure_run'] = max(out['longest_failure_run'], run_length)
            continue
        diag = rec.get('diagnostics', {})
        initial, cleanup = diag.get('initial', {}), diag.get('cleanup', {})
        method = initial.get('method', 0)
        if method:
            key = '0x%x' % method
            out['recovery_methods'][key] = out['recovery_methods'].get(key, 0) + 1
        skipped = item.flags & (P.S_NO_RESET | P.S_NO_PREFLIGHT)
        reset_ok = skipped & P.S_NO_RESET or (not initial.get('rc') and initial.get('final_state', {}).get('phase', 6) == 6)
        preflight_ok = rec.get('pre_ok') == (2 if skipped & P.S_NO_PREFLIGHT else 1)
        bad = False
        if not reset_ok:
            out['initial_reset_failed'].append(item.case.name); bad = True
        elif not preflight_ok:
            out['preflight_failed'].append(item.case.name); bad = True
        if cleanup.get('rc') or rec.get('execution_error') == 'worker-cleanup-failed':
            out['cleanup_failed'].append(item.case.name)
        if skipped:
            out['skipped_by_request'] += 1
        elif not bad:
            out['verified'] += 1
        run_length = run_length + 1 if bad else 0
        out['longest_failure_run'] = max(out['longest_failure_run'], run_length)
    return out


def merge_health(total, part):
    for key, value in part.items():
        if isinstance(value, list): total.setdefault(key, []).extend(value)
        elif isinstance(value, dict):
            bucket = total.setdefault(key, {})
            for k, v in value.items(): bucket[k] = bucket.get(k, 0) + v
        elif key == 'longest_failure_run': total[key] = max(total.get(key, 0), value)
        else: total[key] = total.get(key, 0) + value
    return total


def run(s, t, name, cases):
    started = time.monotonic()
    timing = dict(prepare_seconds=0, upload_seconds=0, command_seconds=0,
                  download_seconds=0, processing_seconds=0, worker_seconds=0, probe_seconds=0)
    prepared, results = [], {}
    for case in cases:
        try:
            item = prepare(case, t, getattr(s, 'observation_min_us', 0))
            if item.flags & MUTATE and not s.probe_caps & MUTATE:
                raise ToolError('Mutation requires a reload: run tools/n4g.py reload first')
            prepared.append(item)
        except Exception as exc:
            results[case.name] = {'case_error': type(exc).__name__, 'message': str(exc)}
            s.log({'type': 'case-error', 'experiment': name, 'case': case.name, **results[case.name]})
    s.last_batch_records = results
    s.last_batch_compact = compact_cache = {}
    s.last_batch_health = health = {}
    if not prepared: return results, name + ': no cases compiled'
    # Calibrated short hardware timeouts; never changes the case identities.
    windows, window_reason = FW.plan(name, prepared, getattr(s, 'fault_margin', 0),
                                     getattr(s, 'fault_floor_us', 16000))
    if windows:
        short = sum(1 for d in windows.values() if d['role'] == 'short')
        print('PLAN: %s: %d cases use a calibrated %s fault window, %d long controls' %
              (name, short, next(d['start_word'] for d in windows.values() if d['role'] == 'short')
               if short else 'default', len(windows) - short), flush=True)
    s.log({'type': 'fault-window-plan', 'experiment': name, 'reason': window_reason,
           'cases': {prepared[i].case.name: d for i, d in windows.items()}})
    s.active_batch = {'experiment': name, 'cases': [x.case.name for x in prepared],
                      'checkpoint': any(x.case.conditions.get('checkpoint') for x in prepared)}
    blob, capacity = pack(prepared, extended=bool(s.probe_caps & BATCH2))
    crc = zlib.crc32(blob) & 0xffffffff
    s.active_batch['request_sha256'] = hashlib.sha256(blob).hexdigest()
    # Complete reproducible plan is durable before submission, not per-case USB traffic.
    s.log({'type': 'batch-plan', 'experiment': name, 'request': blob.hex(),
           'sha256': hashlib.sha256(blob).hexdigest(),
           'cases': [x.case.name for x in prepared], 'output_capacity': capacity})
    print('PLAN: resident experiment %s: %d cases, %d input bytes, up to %d result bytes' %
          (name, len(prepared), len(blob), capacity), flush=True)
    timing["prepare_seconds"] = time.monotonic() - started
    allocated, completed = False, True
    pages, downloaded = 0, 0
    try:
        st = check(s.job(Cmd.NAND_BATCH_ALLOC, len(blob), capacity))
        addr, incap, outaddr, outcap = st.job_detail
        allocated = True
        if incap != len(blob) or outcap != capacity: raise ToolError('batch allocation size mismatch')
        mark = time.monotonic()
        s.write_memory(addr, blob)
        timing["upload_seconds"] = time.monotonic() - mark
        first = 0
        while first < len(prepared):
            if first:
                print('PLAN: %s: resume resident manifest at case %d/%d after result collection (no re-upload)' %
                      (name, first + 1, len(prepared)), flush=True)
            timeout = 30 + sum(x.timeout_us + 200000 for x in prepared[first:]) / 1e6
            completed = False
            mark = time.monotonic()
            st = s.job(Cmd.NAND_BATCH_RUN, len(blob), crc, d=first, timeout=timeout, poll_interval=0.25)
            timing["command_seconds"] += time.monotonic() - mark
            completed = True
            check(st)
            next_case, used, actual_out, _ = st.job_detail
            if actual_out != outaddr or used < 32 or used > capacity:
                raise ToolError('batch result exceeds allocated arena')
            mark = time.monotonic()
            raw = s.read(outaddr, used)
            timing["download_seconds"] += time.monotonic() - mark
            mark = time.monotonic()
            s.log({'type': 'batch-output', 'experiment': name, 'first': first,
                   'sha256': hashlib.sha256(raw).hexdigest(), 'data': raw.hex()})
            next_parsed, records, stats = parse_page(raw, t, prepared, first)
            if next_parsed != next_case: raise ToolError('batch status/output cursor mismatch')
            timing["worker_seconds"] += stats["elapsed_us"] / 1e6
            timing["probe_seconds"] += sum(r.get("elapsed_us", 0) for _, r in records) / 1e6
            pages += 1; downloaded += used
            from .followup_experiments import annotate
            from types import SimpleNamespace
            logs = []
            annotation_sink = SimpleNamespace(log=logs.append)
            deviations = []
            for i, rec in records:
                case = prepared[i].case
                rec['observation_timeout_us'] = prepared[i].timeout_us
                if i in windows and 'case_error' not in rec:
                    rec['fault_window'] = FW.check(rec, windows[i])
                    if rec['fault_window'].get('prediction') == 'deviates': deviations.append(case.name)
                if 'case_error' not in rec: rec = annotate(case, annotation_sink, rec)
                results[case.name] = rec
                compact_cache[case.name] = data = compact(P.jsonable(rec))
                logs.append({'type': 'batch-case-result', 'experiment': name, 'case': case.name,
                       'batch_case': i, 'data': data})
            if deviations:
                print('WARNING %s: %d timeout faults deviate from the calibrated window (first: %s); '
                      'records retained' % (name, len(deviations), deviations[0]), flush=True)
            merge_health(health, reset_health(prepared, records))
            if hasattr(s, 'log_many'): s.log_many(logs)
            else:
                for log in logs: s.log(log)
            s.log({'type': 'batch-page', 'experiment': name, 'first': first, 'next': next_case, **stats})
            print('%s: collected %d/%d cases (%d bytes)' % (name, next_case, len(prepared), used), flush=True)
            timing["processing_seconds"] += time.monotonic() - mark
            first = next_case
    finally:
        # A timed-out/disconnected host must not free arenas under a running worker.
        if allocated and completed:
            check(s.job(Cmd.NAND_BATCH_FREE))
    timing['total_seconds'] = time.monotonic() - started
    timing['upload_bytes'], timing['download_bytes'] = len(blob), downloaded
    timing['device_runs'] = pages
    s.log({'type': 'batch-timing', 'experiment': name, **timing})
    sink = getattr(s, 'family_timing', None)
    if isinstance(sink, dict):
        sink.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in timing.items()})
    s.log({'type': 'reset-health', 'experiment': name, **health})
    failures = len(health.get('initial_reset_failed', ())) + len(health.get('preflight_failed', ()))
    print('RESET %s: %d/%d cases reset+preflight verified, %d skipped by request, %d failed, %d cleanup failures%s' %
          (name, health.get('verified', 0), health.get('cases', 0), health.get('skipped_by_request', 0), failures,
           len(health.get('cleanup_failed', ())), '; RESET FAILURES RECORDED' if failures else ''), flush=True)
    print('TIMING %s: prepare %.2fs, upload %.2fs, run/wait %.2fs (device %.2fs, programs %.2fs), download %.2fs, decode/journal %.2fs' %
          (name, timing['prepare_seconds'], timing['upload_seconds'], timing['command_seconds'],
           timing['worker_seconds'], timing['probe_seconds'], timing['download_seconds'], timing['processing_seconds']), flush=True)
    from .followup_experiments import summarize
    summary = summarize(name, results)
    return results, summary + '; %d device run(s), %d uploaded / %d downloaded bytes' % (pages, len(blob), downloaded)
