"""Small readable summaries; raw native requests/results remain in gzip JSONL."""
import gzip
import json
from pathlib import Path

# These are not hardware measurements when reset/preflight prevented execution.
# The raw output remains in the journal, including caller-provided scratch bytes.
INVALID_CAPTURE_FIELDS = ('before', 'after', 'fault', 'fullmap', 'r', 'scratch', 'samples')


def compact(value):
    if isinstance(value, dict):
        no_execution = (value.get('execution_error') == 'candidate-not-executed' or
                        ('pre_ok' in value and not value.get('pre_ok') and
                         not value.get('polls') and not value.get('nsamples') and value.get('dropped') != 0x5afe5afe))
        omitted = [k for k in INVALID_CAPTURE_FIELDS if k in value] if no_execution else []
        result = {k: compact(v) for k, v in value.items() if k not in omitted}
        if omitted:
            result['unexecuted_capture_omitted'] = omitted
        return result
    if isinstance(value, (tuple, list)):
        return [compact(v) for v in value]
    return value


def _encode(meta):
    """Same bytes as json.dumps(meta, separators=(',', ':')), one family at a time.

    The C encoder is used for every chunk (json.dump would use the pure-Python
    encoder); peak extra memory is bounded by the largest single entry.
    """
    yield '{'
    for index, (key, value) in enumerate(meta.items()):
        yield (',' if index else '') + json.dumps(key) + ':'
        if key == 'results' and isinstance(value, dict):
            yield '{'
            for i, (name, entry) in enumerate(value.items()):
                yield (',' if i else '') + json.dumps(name) + ':' + json.dumps(entry, separators=(',', ':'))
            yield '}'
        else:
            yield json.dumps(value, separators=(',', ':'))
    yield '}\n'


def save(path, meta, level=1):
    # JSONL is the durable case-by-case ledger; replace the readable checkpoint
    # atomically so an interrupted write cannot leave half a JSON document.
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    # Level 1 keeps checkpoints cheap; the final summary may use a higher level.
    # The JSON schema and text are unchanged.
    opener = (lambda: gzip.open(temporary, 'wt', compresslevel=level)) if path.suffix == '.gz' else (lambda: temporary.open('w'))
    with opener() as out:
        for chunk in _encode(meta):
            out.write(chunk)
    temporary.replace(path)


def rebuild(journal):
    """Readable summary from a raw journal alone (interrupted or unsaved run).

    Native case records come from batch-case-result entries, family summaries,
    errors, timing and reset health from family-result entries. Nothing is
    invented: families without a family-result are marked partial.
    """
    meta, open_families = None, {}
    with gzip.open(journal, 'rt') as source:
        for line in source:
            try:
                record = json.loads(line)
            except ValueError:
                break  # a truncated final line after an abrupt stop
            kind = record.get('type')
            if kind == 'session':
                meta = {k: v for k, v in record.items() if k != 'type'}
                meta['results'], meta['errors'] = {}, {}
            elif meta is None:
                continue
            elif kind in ('job-submit', 'job-accepted'):
                meta['pending_job'] = {k: v for k, v in record.items() if k != 'type'}
            elif kind == 'job-progress':
                meta['last_job_status'] = {k: v for k, v in record.items() if k != 'type'}
                pending = meta.get('pending_job', {})
                if record.get('state') == 3 and record.get('jobs_done') == pending.get('job_seq'):
                    pending['phase'] = 'done'
            elif kind == 'gate-checkpoint':
                checkpoint = {k: v for k, v in record.items() if k != 'type'}
                meta.setdefault('gate_checkpoints', []).append(checkpoint)
                meta.setdefault('pending_job', {})['checkpoint'] = checkpoint
            elif kind in ('gate-checkpoint-ack', 'gate-checkpoint-ack-returned'):
                meta.setdefault('pending_job', {})['phase'] = (
                    'checkpoint-ack-submitting' if kind == 'gate-checkpoint-ack' else 'checkpoint-ack-returned')
            elif kind == 'transport-lost' and record.get('pending_job'):
                meta['pending_job'] = record['pending_job']
            elif kind in ('batch-case-result', 'case-error'):
                name = record['experiment']
                entry = meta['results'].setdefault(name, {'summary': 'Rebuilt from journal; family did not finish',
                                                          'data_format': 'native-case-records', 'partial': True,
                                                          'data': {}})
                entry['data'][record['case']] = record['data'] if kind == 'batch-case-result' else {
                    k: record[k] for k in ('case_error', 'message') if k in record}
                open_families[name] = True
            elif kind == 'family-result':
                name = record['experiment']
                entry = meta['results'].setdefault(name, {'data': {}})
                entry.pop('partial', None)
                for key in ('summary', 'data_format', 'reset_health', 'partial'):
                    if key in record: entry[key] = record[key]
                if 'data' in record: entry['data'] = record['data']
                if record.get('errors'): meta['errors'][name] = record['errors']
                if record.get('timing'): meta.setdefault('timing', {}).setdefault('families', {})[name] = record['timing']
                open_families.pop(name, None)
            elif kind == 'experiment-error':
                meta['errors'][record['experiment']] = {k: v for k, v in record.items()
                                                        if k not in ('type', 'experiment')}
    if meta is None:
        raise ValueError('journal has no session record')
    meta['rebuilt_from_journal'] = str(journal)
    meta['complete'] = meta['attempt_finished'] = False
    if open_families:
        meta['partial_families'] = list(open_families)
    return meta


def journal_path(path):
    path = Path(path)
    if path.suffix == '.gz':
        path = path.with_suffix('')
    return path.with_suffix('.jsonl.gz')
