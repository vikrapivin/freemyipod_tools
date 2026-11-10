"""Raw-encoding confirmation and discovery for M2; hypotheses scored against output.

The oracle below scores the encodings on their own. A match on a device confirms
only the stated operands/state; unknown instructions are never called NOPs merely
because a particular encoding continued. No firmware changes are required.
"""
from fmss import probe as P
from . import sequences as Q
from .protocol import DISCOVERY

KNOWN = frozenset((*range(8), *range(0x0a, 0x0f), 0x11, 0x13, 0x14, 0x17, 0x18, 0x19))
FOCUS = (8, 9, 0x0f, 0x10, 0x12, 0x15, 0x16)
MASK = 0xffffffff
SEEDS = [0x13570000 + 0x101 * r for r in range(8)]


def insn(op, reg=0, arg=0, imm=0):
    value = imm if isinstance(imm, str) else '0x%x' % imm
    return '    .insn 0x%x,%d,0x%x,%s\n' % (op, reg, arg, value)


def confirm_case(t, name, op, body, regs=None, expected=None, **options):
    from .followup_experiments import Case
    regs = list(SEEDS if regs is None else regs)
    options['params'] = [(0xc18 + 4 * i, v) for i, v in enumerate(regs)] + options.get('params', [])
    options['flags'] = options.get('flags', 0) | DISCOVERY | P.WAIT_END
    options.setdefault('timeout_us', 250000)
    want = dict(why='END', cs_irq=1, r={str(i): v for i, v in enumerate(regs)})
    for field, values in (expected or {}).items():
        if isinstance(values, dict): want.setdefault(field, {}).update(values)
        else: want[field] = values
    return Case(name, t.prologue() + body + '    end\n' + t.trap(), options,
                dict(opcode=op, mode='confirm', expected=want,
                     basis='explicit M1/common-ISA hypothesis plus first M2 observations; test each operand again'))


def confirmation(s, t):
    a = t.scratch_addr(s)
    yield confirm_case(t, '00/end-stops-before-poison', 0, insn(0) + '    movi r3,#0xbad\n')
    for dst in range(8):
        for value in (0, 1, 0x80000000, 0xffffffff):
            yield confirm_case(t, f'05/r{dst}/{value:x}', 5, insn(5, dst, 0, value),
                               expected={'r': {str(dst): value}})
        for src in (dst, (dst + 3) % 8):
            yield confirm_case(t, f'06/r{dst}/r{src}', 6, insn(6, dst, src),
                               expected={'r': {str(dst): SEEDS[src]}})
    operations = {0xa: lambda x,y: x & y, 0xb: lambda x,y: x | y,
                  0xc: lambda x,y: (x + y) & MASK, 0xd: lambda x,y: (x - y) & MASK,
                  0x13: lambda x,y: (x << y) & MASK if y < 32 else 0,
                  0x14: lambda x,y: x >> y if y < 32 else 0}
    for op, calculate in operations.items():
        for dst in (0, 3, 6, 7):
            for alias in (False, True):
                src = dst if alias else (dst + 1) % 8
                for form in ('register', 'immediate'):
                    for left, right in ((0, 0), (0xffffffff, 1), (0x80000001, 31), (0x55aa55aa, 33)):
                        regs = list(SEEDS)
                        if form == 'immediate' and right:
                            regs[src] = left
                            x, y, imm = left, right, right
                        else:
                            # Immediate field zero is the two-register form,
                            # even when reached from the immediate-form matrix.
                            regs[dst], regs[src] = left, right
                            x, y, imm = regs[dst], regs[src], 0
                        yield confirm_case(t, f'{op:02x}/{form}/r{dst}/alias{alias}/{left:x}/{right:x}', op,
                                           insn(op, dst, src, imm), regs,
                                           {'r': {str(dst): calculate(x, y)}})
    for off in (0xd00, 0xd60, 0xd7c):
        for value in (0, 0xa55ac33c, 0xffffffff):
            yield confirm_case(t, f'01/{off:x}/{value:x}', 1, insn(1, 0, off, value),
                               expected={'after': {hex(off): value}}, params=[(off, value ^ MASK)])
            regs = list(SEEDS); regs[3] = value
            yield confirm_case(t, f'02/{off:x}/{value:x}', 2, insn(2, 3, off), regs,
                               {'after': {hex(off): value}}, params=[(off, value ^ MASK)])
            for mask in (0, 0x00ff00ff, MASK):
                yield confirm_case(t, f'04/{off:x}/{value:x}/mask{mask:x}', 4, insn(4, 3, off, mask),
                                   expected={'r': {'3': value & mask}}, params=[(off, value)])
    pat = bytes(range(128))
    for delta in (0, 1, 2, 3, 4, 7):
        regs = list(SEEDS); regs[1] = a + delta
        value = int.from_bytes(pat[delta & ~3:(delta & ~3) + 4], 'little')
        yield confirm_case(t, f'03/offset{delta}', 3, insn(3, 3, 1), regs, {'r': {'3': value}},
                           scratch=pat, scratch_read=128)
        regs[3] = 0xa1b2c3d4
        changed = bytearray(pat); changed[delta & ~3:(delta & ~3) + 4] = bytes.fromhex('d4c3b2a1')
        yield confirm_case(t, f'11/offset{delta}', 0x11, insn(0x11, 3, 1), regs,
                           {'scratch': changed.hex()}, scratch=pat, scratch_read=128)
    for op in (0xe, 0x17):
        for value in (0, 1, 0x80000000, MASK):
            regs = list(SEEDS); regs[1] = value
            taken = bool(value) if op == 0xe else not value
            body = insn(op, 1, 0, 'taken') + '    movi r3,#0x11111111\n    end\ntaken:\n    movi r3,#0x22222222\n'
            yield confirm_case(t, f'{op:02x}/condition{value:x}', op, body, regs,
                               {'r': {'3': 0x22222222 if taken else 0x11111111}})
    for pointer in (0xd60, 0xd7c, 0x1d60):
        off = pointer & 0xfff
        regs = list(SEEDS); regs[1] = pointer
        yield confirm_case(t, f'18/{pointer:x}', 0x18, insn(0x18, 3, 1), regs,
                           {'r': {'3': 0x89abcdef}}, params=[(off, 0x89abcdef)])
        yield confirm_case(t, f'19/{pointer:x}', 0x19, insn(0x19, 3, 1), regs,
                           {'after': {hex(off): regs[3]}}, params=[(off, 0)])
    # Bare WAIT: baseline control and timeout/vector behavior, not state-independent meanings.
    for event in range(16):
        expected = {} if event == 11 else {'why': 'FAULT', 'cs_irq': 5, 'r': {'6': 0xfa17fa17}}
        yield confirm_case(t, f'07/idle/event{event}', 7, insn(7, event), expected=expected)


    regs = list(SEEDS); regs[1] = 0xc18
    yield confirm_case(t, '18/register-window', 0x18, insn(0x18, 3, 1), regs,
                       {'r': {'3': regs[0]}})
    from .followup_experiments import stimuli, wait
    for state, (action, event, ack, pins) in stimuli(s, t).items():
        for acknowledged in (False, True):
            # p09: all five immediate-ack controls reach END. An acknowledgement
            # issued before completion is not a valid negative event control.
            expected = {}
            body = Q.setup(Q.geometry(s).ce if pins else None) + action
            # Clear all ECC status flags for this negative control. The
            # single-bit acknowledge candidate remains in m2_waitcons.
            clear = Q.wr(0x810, 0x1ff) if state == 'ECC' else ack
            body += (clear if acknowledged else '') + wait(event)
            case = confirm_case(t, f'07/{state}/ack{acknowledged}', 7, body, expected=expected,
                                scratch=bytes((i * 17 + 3) & 255 for i in range(512)), scratch_read=512)
            case.pins = pins
            case.conditions['pending_state'] = state
            case.conditions['acknowledged_before_wait'] = acknowledged
            case.conditions['basis'] = 'p09 observed END; immediate acknowledgement may precede peripheral completion'
            yield case


def raw_case(s, t, op, form, policy, state='idle', action='', pins=False):
    from .followup_experiments import Case
    a = t.scratch_addr(s)
    # All register pointers land inside 8 KiB and have PRAM offsets in their low
    # twelve bits. This makes possible RAM/window effects distinguishable.
    base = (a + 0xfff) & ~0xfff
    regs = [base + 0xd40 + 4 * r for r in range(8)]
    forms = ((3, 1, 0), (3, 0xd60, 0xd64), (0, 0, MASK))
    reg, arg, imm = forms[form]
    pat = bytes((i * 7 + (i >> 8)) & 255 for i in range(8192))
    params = [(0xc18 + 4 * i, v) for i, v in enumerate(regs)]
    params += [(0xd40 + 4 * i, 0xc0de0000 + i) for i in range(16)]
    params += [(0xd70, 0), (0xd74, 0)]
    # Neither the fallthrough marker nor the vector handler clobbers GPRs.
    src = t.prologue(policy) + Q.setup(Q.geometry(s).ce if pins else None) + action
    src += insn(op, reg, arg, imm) + Q.wr(0xd70, 0x600d600d) + '    end\n'
    src += 'trap:\n' + Q.wr(0xd74, 0xfa17fa17) + '    end\n'
    return Case(f'{op:02x}/{state}/form{form}/c58{policy}', src,
                dict(params=params, scratch=pat, scratch_read=len(pat),
                     timeout_us=250000, flags=DISCOVERY | P.WAIT_END | P.F_SAMPLE_REGS),
                dict(opcode=op, mode='discover', operand_words=[(op << 24) | (reg << 16) | arg, imm],
                     seeds=regs, state=state, policy=policy, scratch_pattern='(i*7+(i>>8))&255',
                     marker_offsets={'fallthrough': '0xd70', 'vector': '0xd74'}), pins)


def scan(s, t):
    for op in range(256):
        if op in KNOWN: continue
        for policy in (0, 4):
            for form in range(3):
                yield raw_case(s, t, op, form, policy)


def state_scan(s, t):
    from .followup_experiments import stimuli
    # Repeat ambiguous M1 control opcodes with pending controller activity.
    for state, (action, _, _, pins) in stimuli(s, t).items():
        for op in FOCUS:
            for form in range(3):
                yield raw_case(s, t, op, form, 4, state, action, pins)


def assess(case, rec):
    c = case.conditions
    if c.get('mode') not in ('confirm', 'discover'): return None
    from .protocol import executed
    if not rec.get('pre_ok') or not executed(rec):
        return dict(status='inconclusive', reason='requested instruction was not sampled')
    if c['mode'] == 'confirm':
        differences = {}
        for field, wanted in c['expected'].items():
            actual = rec.get(field)
            if field == 'scratch' and isinstance(actual, (bytes, bytearray)): actual = actual.hex()
            if field == 'scratch_ranges':
                # Compare selected scratch windows only; the rest holds raw captures.
                raw = rec.get('scratch') or b''
                if isinstance(raw, str): raw = bytes.fromhex(raw)
                for key, value in wanted.items():
                    start, want = int(key, 0), bytes.fromhex(value)
                    got = raw[start:start + len(want)]
                    if got != want:
                        first = next((i for i, (x, y) in enumerate(zip(got, want)) if x != y), len(got))
                        differences[field + '/' + key] = dict(first_difference=first, length=len(want),
                                                              differing_bytes=sum(x != y for x, y in zip(got, want)))
                continue
            if isinstance(wanted, dict):
                for key, value in wanted.items():
                    got = actual[int(key)] if field == 'r' else actual.get(int(key, 0))
                    if got != value: differences[field + '/' + key] = dict(expected=value, actual=got)
            elif actual != wanted:
                differences[field] = dict(expected=wanted, actual=actual)
        return dict(status='mismatch' if differences else 'match', differences=differences,
                    scope='only this encoding, operands and starting state; evidence inherits enclosing session')
    regs = {str(i): {'before': value, 'after': rec['r'][i]}
            for i, value in enumerate(c['seeds']) if rec['r'][i] != value}
    raw = rec.get('scratch') or b''
    changed = [i for i, b in enumerate(raw) if b != ((i * 7 + (i >> 8)) & 255)]
    # Retain full raw windows in the record; this is just a searchable signature.
    return dict(status='observed', outcome=rec['why'], cs_irq=rec['cs_irq'],
                fallthrough=rec['after'].get(0xd70) == 0x600d600d,
                vector=rec['after'].get(0xd74) == 0xfa17fa17,
                changed_registers=regs, changed_scratch_bytes=len(changed),
                first_changed_scratch_offsets=changed[:64],
                control={hex(o): rec['after'].get(o) for o in (0xc08,0xc14,0xc38,0xc3c,0xc40,0xc44,0xc50,0xc58,0xc5c)},
                interpretation='signature only; no new opcode semantics assigned automatically')


def m1_control(s, t):
    """Explicit hypotheses for the eight device-discovered M1 instructions."""
    import struct
    from .followup_experiments import stimuli
    states = {'idle': ('', False)}
    states.update({k: (v[0], v[3]) for k, v in stimuli(s, t).items()})
    forms = ((0, 0, 0), (3, 1, 0xd64), (7, 0xd60, MASK))
    scratch = bytes((i * 17 + 3) & 255 for i in range(512))
    for op in (8, 0xff, 0x10, 0x15, 0x16):
        use_states = states if op == 0x16 else {k: states[k] for k in ('idle', 'DMA', 'ECC')}
        for state, (action, pins) in use_states.items():
            for form, (reg, arg, imm) in enumerate(forms):
                expected = {'cs_irq': 3} if op == 0x10 else {}
                if op == 0x16:
                    expected = {'why': 'FAULT' if state == 'idle' else 'END',
                                'cs_irq': 5 if state == 'idle' else 1, 'r': {'6': 0xfa17fa17}}
                body = Q.setup(Q.geometry(s).ce if pins else None) + action + insn(op, reg, arg, imm)
                case = confirm_case(t, f'{op:02x}/{state}/form{form}', op, body, expected=expected,
                                    scratch=scratch, scratch_read=512)
                case.pins = pins
                case.conditions['basis'] = 'M1 device-only opcode hypothesis; M2 may disagree'
                yield case
    for reg in (0, 1, 7):
        for imm in (0, 1, MASK):
            body = f'    movi r{reg},#destination\n' + insn(0xf, 3, reg, imm)
            body += '    movi r2,#0xbad\n    end\ndestination:\n    movi r2,#0x600d600d\n'
            case = confirm_case(t, f'0f/r{reg}/imm{imm:x}', 0xf, body, expected={'r': {'2': 0x600d600d}})
            code = P.assemble(t, case.source)
            target = next(imm for word, imm in struct.iter_unpack('<II', code) if word >> 24 == 5 and (word >> 16) & 255 == reg)
            case.conditions['expected']['r'][str(reg)] = target
            case.conditions['basis'] = 'M1 JR hypothesis: register target, no link register'
            yield case
    for form, (reg, arg, imm) in enumerate(forms):
        for release in ('none', 'advance-ack-start', 'advance-ack-resume'):
            expected = {'why': 'TIMEOUT', 'cs_irq': 8} if release == 'none' else {}
            case = confirm_case(t, f'12/form{form}/{release}', 0x12, insn(0x12, reg, arg, imm), expected=expected)
            if release != 'none':
                code = P.assemble(t, case.source)
                pc = next(i for i in range(0, len(code), 8) if code[i + 3] == 0x12)
                case.options.update(late=[(0xc38, pc + 8), (0xc0c, 8), (0xc00, 0xfff5 if release.endswith('start') else 1)], late_us=2000)
            case.conditions['basis'] = 'M1 pause/CPU-resume hypothesis; original M2 CPU driver also handles pause IRQ'
            yield case
    for number in (64, 255):
        for form, (_, arg, imm) in enumerate(forms):
            case = confirm_case(t, f'09/condition{number}/form{form}', 9, insn(9, number, arg, imm),
                                expected={'why': 'FAULT', 'cs_irq': 5, 'r': {'6': 0xfa17fa17}})
            case.conditions['basis'] = 'M1 WAIT2 numbers above 63 never pass; live VIC mapping is separately tested by w2vic'
            yield case


SPECS = {
    'm2_opcode_confirm': (confirmation, 'alu,branch,regmap,memory,indirect,wait', 'Expected-result checks of all 19 common ISA opcodes'),
    'm2_opcode_m1': (m1_control, 'op0f,op10,op12,op15,op16,w2vic', 'Retest all eight M1 device-discovered opcode meanings as explicit hypotheses'),
    'm2_opcode_scan': (scan, 'unknown,opsweep,fields0', 'All other opcode bytes, three operand forms, continue/vector policies'),
    'm2_opcode_states': (state_scan, 'opstate,op12wait,op16', 'Revisit seven ambiguous control opcodes with five pending stimuli'),
}


def candidate_summary(records):
    """Prioritize signatures for manual interpretation, without inventing meanings."""
    groups = {'continued without fault': set(), 'ended before marker': set(),
              'register or RAM effects': set(), 'pause IRQ': set(), 'timed out': set()}
    for record in records.values():
        a = record.get('assessment', {})
        c = record.get('case_conditions', {})
        if c.get('mode') != 'discover' or a.get('status') != 'observed': continue
        op = c['opcode']
        if a['outcome'] == 'END' and not a['cs_irq'] & 4:
            groups['continued without fault' if a['fallthrough'] else 'ended before marker'].add(op)
        if a['changed_registers'] or a['changed_scratch_bytes']:
            groups['register or RAM effects'].add(op)
        if a['cs_irq'] & 8: groups['pause IRQ'].add(op)
        if a['outcome'] == 'TIMEOUT': groups['timed out'].add(op)
    return '; '.join(label + ': ' + ','.join('%02x' % op for op in sorted(ops))
                     for label, ops in groups.items() if ops)
