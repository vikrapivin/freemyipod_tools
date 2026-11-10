"""Calibrated M2 fault windows: shorter hardware timeouts where timing is not the question.

Measured basis (p16 `startbits`, start word 0xffb5 with every single-bit toggle,
WAIT1 timeout body, SDRAM and SRAM): the timeout fault arrives at 126,450 us for
0xffb5; clearing one bit of bits 15:8 shortens it linearly (0xfeb5 125,956 us,
0xfdb5 125,464, 0xfbb5 124,478, 0xf7b5 122,508, 0xefb5 118,564, 0xdfb5 110,682,
0xbfb5 94,904, 0x7fb5 63,378), i.e. (count + 1) x ~493.9 us. Other bits
outside 7:0 and 15:8 leave it unchanged. p18 `m2_timeout_window` measured
multi-bit counts 8..256 directly: fault_at = 492.747 x (count + 1) + 447.5 us
within 1.1 us. Every shortened case still records its predicted
fault time and the host checks the measured `fault_at_us` against it.

A case is shortened only when ALL of the following hold, so the same outcome
is expected and only the timeout-fault arrival moves earlier:
  * its family is in the p16 calibration table, is not a timing family, and
    the longest non-timeout event (END, or a fault that is not the ~126.5 ms
    timeout) of its eligible p16 cases times the margin fits inside the window;
  * the request uses the default start word, has no timed CPU writes, no NAND
    mutation, and its observation window would also see the long timeout;
  * the program contains only opcodes with known prompt behaviour; no pause,
    tmwait, evjmp, unknown/discovery opcode, and no WAIT2 condition that p16
    observed releasing late (8, 16, 19).
Every 64th eligible case per family keeps the hardware default as a long
control. Families without p16 evidence (new families) are never shortened.
"""
import hashlib
import json
import math
from pathlib import Path
import struct

DEFAULT_START = 0xffb5
HW_FAULT_US = 126450                # p16 startbits, 0xffb5
# p18 m2_timeout_window: 14 multi-bit counts 8..256, WAIT3 and illegal opcode,
# fault_at = 492.747 x (count + 1) + 447.5 us, largest residual 1.1 us.
STEP_US = 492.747                   # one count of start-word bits 15:8
OFFSET_US = 447.5
TIMEOUT_FAULT_US = (125000, 128000)  # observed timeout faults: 126,446..126,602 us
MIN_COUNT = 7                       # bits 15:11 cleared at most; each was measured alone
CONTROL_EVERY = 64
CALIBRATION = Path(__file__).with_name('fault_window_p16.json')
# Opcodes with prompt, timeout-independent completion on M2 (ISA confirm/core).
# 0x0f (jr) is excluded: p16 m2_opcode_states 0f/DMA/form2/c584 neither ended nor faulted.
FAST_OPS = frozenset({0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
                      0x0d, 0x0e, 0x10, 0x11, 0x13, 0x14, 0x17})
# The question of these families is a timeout, start word or timer; never shorten.
TIMING_FAMILIES = frozenset({'startbits', 'startdiff', 'ctrlfx', 'tmofield', 'tmobits', 'timeregs',
                             'tmwait', 'tmwait2', 'op15', 'c14', 'wait', 'clock', 'm2_nested_fault',
                             'm2_delayed_vector'})
MUTATE = 1 << 18


def start_word(window_us):
    count = min(255, max(MIN_COUNT, math.ceil((window_us - OFFSET_US) / STEP_US) - 1))
    return (count << 8) | (DEFAULT_START & 0xff), round((count + 1) * STEP_US + OFFSET_US)


def opcodes(code):
    for i in range(0, len(code) - 7, 8):
        word = struct.unpack_from('<I', code, i)[0]
        yield word >> 24, (word >> 16) & 0xff


def program_reason(code, late_wait2):
    for op, reg in opcodes(code):
        if op not in FAST_OPS:
            return 'opcode 0x%02x' % op
        if op == 0x09 and reg in late_wait2:
            return 'wait2 %d releases late' % reg
    return None


def request_fields(request):
    flags, timeout, nparams, prog = struct.unpack_from('<4I', request, 8)
    nlate, start = struct.unpack_from('<2I', request, 28)
    nlate2 = struct.unpack_from('<I', request, 40)[0]
    return dict(flags=flags, timeout_us=timeout, nlate=nlate + nlate2, start=start,
                code=request[0x400:0x400 + prog])


def case_reason(item, late_wait2):
    """Why this prepared case must keep the hardware timeout; None if it may be shortened."""
    f = request_fields(item.request)
    if f['start']:
        return 'explicit start word'
    if f['nlate']:
        return 'timed CPU writes'
    if f['flags'] & MUTATE:
        return 'NAND mutation'
    if (f['timeout_us'] or 100000) < TIMEOUT_FAULT_US[1]:
        return 'observation window shorter than the hardware timeout'
    return program_reason(f['code'], late_wait2)


def load(path=CALIBRATION):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def digest(prepared):
    """Identity of a family's planned requests and initializers (before any patch)."""
    h = hashlib.sha256()
    for item in prepared:
        h.update(item.case.name.encode() + b'\0' + item.request + b'\0' +
                 (b'' if item.scratch is None else bytes(item.scratch)))
    return h.hexdigest()


def family_window(table, name, margin, floor_us):
    """Shortened window for a family, or (None, reason)."""
    if not margin:
        return None, 'disabled'
    if table is None:
        return None, 'no calibration table'
    if name in TIMING_FAMILIES:
        return None, 'timing family'
    entry = table['families'].get(name)
    if entry is None:
        return None, 'no p16 evidence'
    if entry.get('blocked'):
        return None, entry['blocked']
    window = max(floor_us, margin * entry['max_event_us'])
    word, predicted = start_word(window)
    if predicted > HW_FAULT_US // 2:
        return None, 'no gain: needs %d us' % window
    return (word, predicted), None


def plan(name, prepared, margin, floor_us, table=None):
    """index -> per-case annotation; patches eligible requests in place."""
    table = load() if table is None else table
    window, reason = family_window(table, name, margin, floor_us)
    decisions = {}
    if window is None:
        return decisions, reason
    expected = table['families'][name].get('requests_sha256')
    if expected is not None and expected != digest(prepared):
        return decisions, 'family requests differ from the calibrated capture'
    word, predicted = window
    late = frozenset(table.get('late_wait2', ()))
    eligible = 0
    for index, item in enumerate(prepared):
        if case_reason(item, late) is not None:
            continue
        info = {'basis': 'p16 start-word calibration; margin %d, floor %d us' % (margin, floor_us)}
        if eligible % CONTROL_EVERY == 0:
            info.update(role='long-control', start_word='0x%x' % DEFAULT_START, predicted_fault_us=HW_FAULT_US)
        else:
            request = bytearray(item.request)
            struct.pack_into('<I', request, 32, word)
            item.request = bytes(request)
            info.update(role='short', start_word='0x%x' % word, predicted_fault_us=predicted)
        decisions[index] = info
        eligible += 1
    return decisions, None


def check(rec, info):
    """Attach the window and compare a timeout fault with its prediction."""
    out = dict(info)
    at = rec.get('fault_at_us') or 0
    if at:
        predicted = info['predicted_fault_us']
        out['measured_fault_us'] = at
        out['prediction'] = 'matches' if abs(at - predicted) <= max(500, predicted // 20) else 'deviates'
    return out


# ---- offline calibration from a physical capture --------------------------------------------------
def calibrate(capture, late_wait2=(8, 16, 19)):
    """Per-family evidence from one decoded capture (dict). Needs the host planner, no device."""
    import io
    from contextlib import redirect_stdout
    from types import SimpleNamespace
    from unittest.mock import patch
    from . import batch, experiments
    from .protocol import BATCH, BATCH2, MUTATE as M
    results = capture['results']
    s = SimpleNamespace(probe_caps=BATCH | BATCH2 | M, log=lambda _: None,
                        observation_min_us=capture.get('observation_min_us') or 0,
                        status=lambda: SimpleNamespace(result_addr=0x08400000, step_ok_mask=6))
    floor = s.observation_min_us
    late = frozenset(late_wait2)
    table = {'source': 'capture with %d families' % len(results), 'late_wait2': sorted(late),
             'hardware_fault_us': HW_FAULT_US, 'step_us': STEP_US, 'families': {}}

    class Stop(BaseException):
        pass

    def submit(session, t, name, cases):
        data = (results.get(name) or {}).get('data')
        if not isinstance(data, dict):
            raise Stop()
        entry = {'cases': 0, 'eligible': 0, 'eligible_timeout_faults': 0, 'eligible_fault_us': 0,
                 'max_event_us': 0}
        prepared = []
        for case in cases:
            entry['cases'] += 1
            try:
                item = batch.prepare(case, t, floor)
            except Exception:
                continue
            prepared.append(item)
            if case_reason(item, late) is not None:
                continue
            rec = data.get(case.name)
            if not isinstance(rec, dict) or 'why' not in rec:
                entry['blocked'] = 'eligible case %s has no executed record in the capture' % case.name
                continue
            entry['eligible'] += 1
            at, elapsed = rec.get('fault_at_us') or 0, rec.get('elapsed_us') or 0
            if TIMEOUT_FAULT_US[0] <= at <= TIMEOUT_FAULT_US[1]:
                entry['eligible_timeout_faults'] += 1
                entry['eligible_fault_us'] += at
            elif rec['why'] == 'TIMEOUT':
                entry['blocked'] = 'eligible case %s neither ended nor faulted' % case.name
            else:
                entry['max_event_us'] = max(entry['max_event_us'], elapsed)
        if not entry['eligible'] and 'blocked' not in entry:
            entry['blocked'] = 'no eligible case with evidence'
        if not entry['eligible_timeout_faults'] and 'blocked' not in entry:
            entry['blocked'] = 'no timeout faults to shorten'  # no gain; keep the start word
        entry['requests_sha256'] = digest(prepared)
        table['families'][name] = entry
        raise Stop()

    with patch.object(batch, 'run', side_effect=submit), redirect_stdout(io.StringIO()):
        for e in experiments.entries(True):
            try:
                e['fn'](s, experiments.NANO4G)
            except Stop:
                pass
    return table


def estimate(table, margin, floor_us):
    """Device seconds saved on the calibration capture's own timeout faults."""
    saved = 0.0
    for name, entry in table['families'].items():
        window, _ = family_window(table, name, margin, floor_us)
        if window and entry['eligible_timeout_faults']:
            shortened = entry['eligible_timeout_faults'] * (1 - 1.0 / CONTROL_EVERY)
            saved += shortened * (entry['eligible_fault_us'] / entry['eligible_timeout_faults'] - window[1]) / 1e6
    return saved


def main(argv=None):
    import argparse
    import gzip
    import hashlib
    ap = argparse.ArgumentParser(description='Regenerate the fault-window calibration from a capture')
    ap.add_argument('capture', type=Path)
    ap.add_argument('--output', type=Path, default=CALIBRATION)
    a = ap.parse_args(argv)
    raw = a.capture.read_bytes()
    raw = gzip.decompress(raw) if a.capture.suffix == '.gz' else raw
    table = calibrate(json.loads(raw))
    table['source'] = '%s sha256 %s' % (a.capture.name, hashlib.sha256(raw).hexdigest())
    for margin, floor in ((8, 16000), (4, 16000), (8, 4000)):
        table.setdefault('estimated_saving_s', {})['margin%d_floor%d' % (margin, floor)] = round(estimate(table, margin, floor), 1)
    a.output.write_text(json.dumps(table, indent=1, sort_keys=True) + '\n')
    print(json.dumps(table['estimated_saving_s']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
