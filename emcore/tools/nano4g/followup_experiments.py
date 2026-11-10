"""Additional M1/M2 comparisons, with independently journaled, bounded cases.

These extend the original registry without replacing its named experiments.
Every source is assembled on the host and runs through the existing v14 probe.
Unknown opcode/register meanings are observations, never prerequisites.
"""
from dataclasses import dataclass, field

from fmss import probe as P
from . import sequences as Q
from .protocol import DISCOVERY, MUTATE


@dataclass
class Case:
    name: str
    source: str
    options: dict = field(default_factory=dict)
    conditions: dict = field(default_factory=dict)
    pins: bool = False


def finish(t, body, policy=4, handler=None):
    return (t.prologue(policy) + body + Q.wr(0, 0) +
            '    movi r6,#0x600d600d\n    end\n' + (handler or t.trap()))


def memory():
    return bytearray((i * 37 + (i >> 9) * 53 + 11) & 255 for i in range(0x4000))


def capture(addr):
    # Each group is 16 bytes. C08/C0C/FMC_STATUS/C50 are raw observations.
    return ''.join(Q.store(off, addr + 4 * k) for k, off in enumerate((0xc08, 0xc0c, 0x48, 0xc50)))


def wait(n):
    return Q.wait(n, 0x40, 0x40) if n == 12 else Q.wait(n)


def stimuli(s, t):
    a = t.scratch_addr(s)
    return {
        'command': (Q.wr(8, 0x70), 0, Q.wr(0x48, 2), True),
        'address': (Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(4, 1), 1, Q.wr(0x48, 4), True),
        'DMA': (Q.dma(a).replace(Q.wait(2) + Q.wr(0x48, 0x100000), ''), 2, Q.wr(0x48, 0x100000), False),
        'ECC': (Q.dma(a) + Q.ecc(acknowledge=False), 4, Q.wr(0x810, 1), False),
        'status': (Q.command(0x70) + Q.wr(4, 0xca), 12, Q.wr(0x48, 0x1800000), True),
    }


def shifts(s, t):
    for op in (0x13, 0x14):
        for form in ('immediate', 'register'):
            for value in (1, 0x80000000, 0xffffffff):
                for count in (0, 1, 7, 8, 15, 16, 31, 32, 33, 63, 64, 255, 256, 0xffffffff):
                    # Immediate zero intentionally exercises the two-operand
                    # encoding; record its register inputs instead of calling
                    # it an immediate shift-by-zero measurement.
                    arg = value if form == 'immediate' else count
                    imm = count if form == 'immediate' else 0
                    body = ('    movi r1,#0x%x\n    movi r3,#0x%x\n' % (arg, value) +
                            '    .insn 0x%x,3,1,0x%x\n' % (op, imm))
                    yield Case(f'op{op:x}/{form}/v{value:x}/n{count:x}', finish(t, body),
                               conditions=dict(value=value, count=count, form=form, r1=arg, immediate=imm))


def pram_alias(s, t):
    for base in (0xd00, 0xd60, 0xd7c, 0xd80, 0xdfc):
        for high in (0, 0x1000, 0x2000, 0x10000, 0x100000):
            for low in range(4):
                pointer = base + high + low
                body = ('    movi r1,#0x%x\n    .insn 0x18,2,1,0\n'
                        '    movi r3,#0xc0ffee17\n    .insn 0x19,3,1,0\n'
                        '    .insn 0x18,4,1,0\n    ld r5,[0x%x]\n' % (pointer, base))
                yield Case(f'base{base:x}/high{high:x}/low{low}', finish(t, body),
                           dict(params=[(base, 0x12345678)], flags=DISCOVERY),
                           dict(pointer=pointer, direct_offset=base, read_before=2, indirect_after=4, direct_after=5))


def c58_bits(s, t):
    for path in ('cpu', 'sequencer'):
        for value in (0, 0xffffffff, *(1 << b for b in range(32))):
            body = (Q.wr(0xc58, value) if path == 'sequencer' else '')
            body += '    ld r2,[0xc58]\n'
            # Sample the mask separately from timeout behavior (already swept
            # by c58); this avoids fault entry modifying the observed word.
            src = (t.prologue() if path == 'sequencer' else '') + body + '    end\n' + t.trap()
            options = dict(params=[(0xc58, value)]) if path == 'cpu' else {}
            yield Case(f'{path}/{value:08x}', src, options)


def wait_consumption(s, t):
    a, g = t.scratch_addr(s), Q.geometry(s)
    for state, (action, event, ack, pins) in stimuli(s, t).items():
        variants = {'repeat': ((event, event), ''), 'triple': ((event,) * 3, ''),
                    'ack-between': ((event, event), ack), 'rearm-between': ((event, event), ack + action),
                    'event-15': ((event, 15), ''), '15-event': ((15, event), ''),
                    'event-11': ((event, 11), ''), '11-event': ((11, event), '')}
        for label, (events, between) in variants.items():
            for policy in (0, 4):
                body = Q.setup(g.ce if pins else None) + action
                for k, number in enumerate(events):
                    if k: body += between
                    body += Q.wr(0xd70, k + 1) + capture(a + 0x2000 + k * 32)
                    body += wait(number) + capture(a + 0x2010 + k * 32) + Q.wr(0xc0c, 4)
                yield Case(f'{state}/{label}/c58{policy}', finish(t, body, policy),
                           dict(scratch=memory(), scratch_read=0x2100, timeout_us=500000, flags=P.WAIT_END),
                           dict(events=events, completed_pairs_at='scratch+0x2000', stage_register='d70'), pins)


def status_order(s, t):
    g = Q.geometry(s)
    c, latch = Q.wr(8, 0x70), Q.wr(4, 0xca)
    cases = {'plain': c + latch, 'ack-before-latch': c + Q.wr(0x48, 2) + latch,
             'ack-after-latch': c + latch + Q.wr(0x48, 2),
             'wait-before-latch': c + wait(0) + latch,
             'wait-after-latch': c + latch + wait(0)}
    for n in (1, 20, 200):
        delay = '    movi r5,#0\n' * n
        cases[f'delay{n}-before'] = c + delay + latch
        cases[f'delay{n}-after'] = c + latch + delay
    for label, action in cases.items():
        for event in (5, 12, 15):
            body = Q.setup(g.ce) + action + '    ld r2,[0x48]\n' + wait(event)
            body += '    ld r3,[0x48]\n    ld r4,[0x4c]\n'
            yield Case(f'{label}/event{event}', finish(t, body),
                       dict(timeout_us=200000, flags=P.WAIT_END), dict(delay_unit='instructions, not microseconds'), True)


def ce_conditions(s, t):
    for ce in (None, *range(8)):
        for op, count in ((7, 16), (9, 32)):
            for number in range(count):
                body = Q.setup(ce) + '    .insn 0x%x,%d,0,0\n' % (op, number)
                yield Case(f'ce{ce}/op{op:x}/n{number}', finish(t, body),
                           dict(timeout_us=200000, flags=P.WAIT_END | DISCOVERY),
                           dict(ce=ce, populated='unknown; selection does not establish chip presence'), True)


def delayed_vector(s, t):
    a, g = t.scratch_addr(s), Q.geometry(s)
    dma = [(0, 0x1077801), (0x38, 7), (0x30, 511), (0x14, 1), (0x34, a), (4, 0x2e0)]
    command = [(0, 0x77001 | (2 << g.ce)), (8, 0x70)]
    for kind, writes in (('none', []), ('command', command), ('DMA', dma)):
        for policy in (0, 4):
            for delay in (0, 100, 2000):
                for vector in ('trap', 'alternate'):
                    src = t.prologue(policy) + '    wr [0xc4c],#%s\n' % vector
                    src += Q.setup() + Q.wr(0xd70, 1) + '    .insn 0x16,0,0,0\n'
                    src += Q.wr(0xd70, 2) + '    movi r6,#0x600d600d\n    end\n'
                    src += 'alternate:\n' + capture(a + 0x2000) + Q.wr(0, 0)
                    src += '    movi r6,#0xa17a17\n    end\n' + t.trap()
                    # delay=0 is a pending-state control. Delayed CPU writes
                    # are kept distinct from events issued by the sequencer.
                    if delay == 0 and writes:
                        src = src.replace(Q.wr(0xd70, 1), ''.join(Q.wr(o, v) for o, v in writes) + Q.wr(0xd70, 1))
                    options = dict(scratch=memory(), scratch_read=0x2100,
                                   flags=P.WAIT_END | DISCOVERY, timeout_us=200000)
                    if delay and writes: options.update(late=writes, late_us=delay)
                    yield Case(f'{kind}/c58{policy}/delay{delay}/{vector}', src, options,
                               dict(vector=vector, stimulus=kind, late_us=delay), kind == 'command')


def nested_fault(s, t):
    a = t.scratch_addr(s)
    for fault, instruction in (('wait', wait(1)), ('opcode', '    .insn 0x1a,0,0,0\n')):
        for pad in (0, 3, 9):
            for nested in (False, True):
                src = t.prologue(4) + Q.setup() + '    wr [0xc4c],#first\n'
                src += '    movi r5,#0\n' * pad + 'faulting:\n    movi r2,#faulting\n' + instruction
                src += '    movi r6,#0x600d600d\n    end\nfirst:\n'
                src += Q.store(0xc3c, a + 0x2000) + Q.store(0xc38, a + 0x2004) + capture(a + 0x2010)
                if nested:
                    src += '    wr [0xc4c],#second\n' + Q.wr(0xc0c, 4) + instruction
                src += '    movi r6,#0x11111111\n    end\nsecond:\n'
                src += Q.store(0xc3c, a + 0x2040) + Q.store(0xc38, a + 0x2044) + capture(a + 0x2050)
                src += '    movi r6,#0x22222222\n    end\n' + t.trap()
                yield Case(f'{fault}/pad{pad}/nested{nested}', src,
                           dict(scratch=memory(), scratch_read=0x2100, timeout_us=500000, flags=P.WAIT_END),
                           dict(first_handler='0x11111111', second_handler='0x22222222'))


def pause_state(s, t):
    a = t.scratch_addr(s)
    for op in (0x10, 0x12, 0x15, 0x16):
        for resume in ('none', 'control', 'advance-ack-control'):
            body = capture(a + 0x2000) + '    .insn 0x%x,0,0,0\n' % op + capture(a + 0x2020)
            src = finish(t, body)
            options = dict(scratch=memory(), scratch_read=0x2100, flags=P.WAIT_END | DISCOVERY, timeout_us=200000)
            if resume != 'none':
                code = P.assemble(t, src)
                pc = next(i for i in range(0, len(code), 8) if code[i + 3] == op)
                writes = [(0xc38, pc + 8), (0xc0c, 8)] if resume == 'advance-ack-control' else []
                options.update(late=writes + [(0xc00, 1)], late_us=2000)
            yield Case(f'op{op:x}/{resume}', src, options,
                       dict(resume_hypothesis=resume, before='scratch+0x2000', after='scratch+0x2020'))


def ecc_sequence(s, t):
    a = t.scratch_addr(s)
    actions = {'none': '', 'ecc-reset': Q.wr(0x81c, 1), 'ack': Q.wr(0x810, 0x1ff),
               'slot-select': Q.wr(0x1c, 1), 'control': Q.wr(0, 0) + Q.setup()}
    for strength in (8, 16):
        for slot in (0, 1):
            for label, action in actions.items():
                body = Q.setup() + Q.wr(0x810, 0x1ff)
                for rep in range(3):
                    if rep: body += action
                    # Do not hide the experiment behind Q.ecc's automatic
                    # acknowledgements. Each encode gets distinct input.
                    body += Q.dma(a + rep * 0x400, slot)
                    body += Q.wr(0x1c, slot) + Q.wr(0x804, 6 if strength == 16 else 2) + wait(4)
                    body += capture(a + 0x2000 + rep * 0x100)
                    for k, off in enumerate((0x80c, 0x810, 0x900, 0x904, 0x908, 0x90c, 0x910)):
                        body += Q.store(off, a + 0x2010 + rep * 0x100 + 4 * k)
                yield Case(f'bch{strength}/slot{slot}/{label}', finish(t, body),
                           dict(scratch=memory(), scratch_read=0x2300, flags=P.FULLMAP),
                           dict(strength=strength, slot=slot, encode_commands_in_one_program=3,
                                completion="WAIT4 may observe a retained flag; commands issued do not prove three completions",
                                candidate_words='0x900..0x910 are observations, not established parity ports'))


def fifo_access(s, t):
    a = t.scratch_addr(s)
    for slot in (0, 1):
        for control in (None, 0x100, 0x340):
            for writer in ('sequencer', 'cpu'):
                # Fill with a different pattern first; only 32 words are written
                # through the candidate port so CPU writes fit the v14 ABI.
                body = Q.setup() + Q.dma(a, slot) + Q.wr(0x14, 1 << slot)
                if control is not None: body += Q.wr(4, control)
                options = dict(scratch=memory(), scratch_read=0x2400, flags=P.FULLMAP)
                if writer == 'sequencer':
                    body += ''.join(Q.wr(0x80, 0x5a000000 + k) for k in range(32))
                else:
                    body += Q.wr(0xd70, 1) + 'gate:\n    ld r2,[0xd74]\n    bz r2,gate\n'
                    options.update(params=[(0xd74, 0)], late=[(0x80, 0x5a000000 + k) for k in range(32)] + [(0xd74, 1)], late_us=2000)
                body += Q.dma(a + 0x2000, slot, into=False)
                yield Case(f'slot{slot}/control{control}/{writer}', finish(t, body), options,
                           dict(words_written=32, expected_prefix='0x5a000000..0x5a00001f', cpu_stimulus='scheduled at 2000 us; d70 marks ready'))


ACK = {2: Q.wr(0x48, 0x100000), 4: Q.wr(0x810, 1)}


def overlap_body(a, slot, strength, mode, wait_order, acknowledge):
    """Second input DMA without its own WAIT2, then explicit waits.

    p16 used acknowledge=False: the explicit waits left FMC_STATUS bit 20
    (and ECC completion) set, so the first output DMA's WAIT2 passed at once
    and the second output DMA was programmed while the first was running.
    """
    other = slot if mode == 'same-slot' else 1 - slot
    body = Q.setup() + Q.dma(a, slot)
    body += Q.ecc(slot, strength=strength, acknowledge=mode == 'serial')
    body += Q.dma(a + 0x400, other).replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')
    body += capture(a + 0x3000)
    events = (2,) if mode == 'serial' else wait_order
    for event in events:
        body += wait(event) + (ACK[event] if acknowledge else '')
    body += capture(a + 0x3020) + Q.store(0x80c, a + 0x3040)
    body += Q.dma(a + 0x2000, slot, into=False) + Q.dma(a + 0x2200, 1 - slot, into=False)
    return body, other, events


def overlap_expected(slot, mode, other):
    """Output buffers after a correct run: input A at a, input B at a+0x400."""
    mem = memory()
    first, second = mem[:512], mem[0x400:0x600]
    buffers = {slot: first}
    buffers[other] = second  # same-slot: B replaces A
    return {'0x2000': bytes(buffers[slot]).hex(), '0x2200': bytes(buffers[1 - slot]).hex()} \
        if mode != 'same-slot' else {'0x2000': bytes(second).hex()}


def dma_ecc_overlap(s, t):
    """Corrected p16 family: every explicit WAIT is acknowledged before the next DMA."""
    a = t.scratch_addr(s)
    for strength in (8, 16):
        for slot in (0, 1):
            for mode in ('serial', 'different-slot', 'same-slot'):
                for wait_order in ((4, 2), (2, 4)):
                    body, other, events = overlap_body(a, slot, strength, mode, wait_order, True)
                    yield Case(f'bch{strength}/slot{slot}/{mode}/wait{wait_order}', finish(t, body),
                               dict(scratch=memory(), scratch_read=0x3100, flags=P.WAIT_END, timeout_us=500000),
                               dict(mode='confirm', expected={'why': 'END'} if mode == 'same-slot' else
                                    {'why': 'END', 'scratch_ranges': overlap_expected(slot, mode, other)},
                                    hazard='M2-80: DMA into the slot under its own ECC drops bursts (data not predicted)'
                                    if mode == 'same-slot' else None,
                                    strength=strength, overlap=mode, wait_order=events, acknowledged_waits=True,
                                    scope='buffer arbitration/data integrity; does not prove elapsed overlap on synchronous models'))


def dma_ack(s, t):
    """Discriminate the p16 payload loss: unacknowledged WAIT2 versus DMA itself.

    p16-serial is the exact p16 serial program. serial-ack adds only the two
    acknowledgements. ctl0 also clears FMC_DMA_CTRL once and never rewrites it,
    like the original M2 READ. plain-* repeat the contrast without ECC: one
    input DMA whose completion is left set, then two output DMAs of that slot.
    FMC_DMA_CTRL (0x038) and FMC_STATUS are stored at each stage.
    """
    a = t.scratch_addr(s)
    mem = memory()
    A = bytes(mem[:512])
    stamp = lambda k: Q.store(0x38, a + 0x3080 + 8 * k) + Q.store(0x48, a + 0x3084 + 8 * k)
    for slot in (0, 1):
        for variant in ('p16-serial', 'serial-ack', 'serial-ack-ctl0', 'plain-stale', 'plain-ack'):
            if 'serial' in variant:
                body, other, _ = overlap_body(a, slot, 8, 'serial', (2,), variant != 'p16-serial')
                if variant == 'serial-ack-ctl0':
                    body = Q.wr(0x38, 0) + body.replace(Q.wr(0x38, 7), '')
                expected = overlap_expected(slot, 'serial', other)
            else:
                body = Q.setup() + Q.dma(a, slot).replace(Q.wr(0x48, 0x100000), '' if variant == 'plain-stale' else Q.wr(0x48, 0x100000))
                body += stamp(1)
                body += Q.dma(a + 0x2000, slot, into=False) + stamp(2)
                body += Q.dma(a + 0x2200, slot, into=False)
                expected = {'0x2000': A.hex(), '0x2200': A.hex()}
            body = stamp(0) + body + stamp(3)
            conditions = dict(reference_ranges=expected, slot=slot, variant=variant,
                              stamps='0x3080+8k: FMC_DMA_CTRL, FMC_STATUS at k=0 start, 1 after input, 2 after first output, 3 end',
                              hypothesis='unacknowledged WAIT2 lets the next DMA be programmed during the previous one')
            if variant in ('serial-ack', 'serial-ack-ctl0', 'plain-ack'):
                conditions.update(mode='confirm', expected={'why': 'END', 'scratch_ranges': expected})
            else:
                conditions.update(prediction='first output truncated after a few bursts; 0x038 gains bits 6..8')
            yield Case(f'slot{slot}/{variant}', finish(t, body),
                       dict(scratch=memory(), scratch_read=0x3100, flags=P.WAIT_END, timeout_us=20000), conditions)


def dma_visibility(s, t):
    """P16 serial controls returned partial payloads: separate DMA and cleanup."""
    import struct
    a = t.scratch_addr(s)
    initial = memory()[:0x600]
    initial[0x400:0x600] = b'\xa5' * 512
    expected = bytearray(initial)
    expected[0x400:0x600] = initial[:512]
    checksum = sum(struct.unpack('<128I', initial[:512])) & 0xffffffff
    for slot in (0, 1):
        for strength in (0, 8, 16):
            for hold in ('none', 'input', 'output', 'both'):
                options = dict(scratch=initial, scratch_read=len(initial),
                               params=[(0xd74, 0), (0xd78, 0)], timeout_us=20000)
                body = Q.setup() + Q.dma(a, slot)
                if hold in ('input', 'both'):
                    body += 'input_gate:\n    ld r2,[0xd74]\n    bz r2,input_gate\n'
                    options.update(late=[(0xd74, 1)], late_us=2000)
                if strength:
                    body += Q.ecc(slot, strength=strength)
                body += Q.dma(a + 0x400, slot, into=False)
                # Freeze no state on the host: scheduled CPU writes execute
                # inside the resident probe, before normal cleanup.
                if hold in ('output', 'both'):
                    body += 'output_gate:\n    ld r2,[0xd78]\n    bz r2,output_gate\n'
                    if hold == 'both':
                        options.update(late2=[(0xd78, 1)], late2_us=4000)
                    else:
                        options.update(late=[(0xd78, 1)], late_us=2000)
                body += ('    movi r1,#0x%x\n    movi r3,#128\n    movi r4,#0\n'
                         'sum_words:\n    ldi r2,[r1]\n    .insn 0x0c,4,2,0\n'
                         '    add r1,r1,#4\n    sub r3,r3,#1\n    bnz r3,sum_words\n'
                         '    st [0xd30],r4\n' % (a + 0x400))
                yield Case(f'slot{slot}/bch{strength}/hold-{hold}', finish(t, body), options,
                           dict(mode='confirm', expected={'why': 'END', 'scratch': expected.hex(),
                                'after': {'0xd30': checksum}},
                                scope='full payload and pre-cleanup sequencer checksum; hold times are CPU scheduled',
                                strength=strength, slot=slot, hold=hold))


def timeout_window(s, t):
    """Multi-bit start-word counts: does (count + 1) x 494 us hold for combined bits?

    p16 startbits measured single-bit toggles of 0xffb5 only. fault
    windows use 0x20b5. Counts with several clear bits are the question here.
    """
    for count in (0xff, 0xfe, 0x80, 0x40, 0x20, 0x10, 0x07):
        word = (count << 8) | 0xb5
        for label, body in (('wait3', Q.wait(3)), ('illegal1a', '    .insn 0x1a,0,0,0\n')):
            yield Case(f'{word:04x}/{label}', finish(t, body),
                       dict(start_value=word, flags=DISCOVERY, timeout_us=300000),
                       dict(start_word=word, predicted_us=round((count + 1) * 493.9, 1),
                            basis='p16 startbits single-bit linear fit; combined counts are the test'))


BANK_WORDS = (0x00c, 0x010, 0x014, 0x018, 0x01c, 0x02c, 0x030, 0x034, 0x058, 0x05c)


def bank_regs(s, t):
    """Primary versus +400 register words: independent, mirrored or inert.

    Only storage-like words are written (no command, operation, W1C, FIFO or
    enable words). Complementary patterns in both write orders; every value is
    read back by the sequencer before the other base is touched again.
    A mirror shows cross-coupling in both orders; independence needs both
    distinct values retained; read-as-zero is its own outcome.
    """
    a = t.scratch_addr(s)
    clear = ''.join(Q.wr(off, 0) + Q.wr(off + 0x400, 0) for off in BANK_WORDS)
    for p0, p1 in ((0x55555555, 0xaaaaaaaa), (0xaaaaaaaa, 0x55555555)):
        for order in ('primary-first', 'second-first'):
            body, k = clear, 0
            for off in BANK_WORDS:
                writes = ((off, p0 ^ off), (off + 0x400, p1 ^ off))
                for o, v in (writes if order == 'primary-first' else writes[::-1]):
                    body += Q.wr(o, v)
                for o in (off, off + 0x400):
                    body += Q.store(o, a + 0x2000 + 4 * k); k += 1
            body += clear
            yield Case(f'seq/{p0:08x}/{order}', finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2100, flags=DISCOVERY | P.FULLMAP, timeout_us=20000),
                       dict(words=[hex(o) for o in BANK_WORDS], written={'primary': hex(p0), 'second': hex(p1)},
                            xor_offset=True, layout='0x2000+8i: [primary readback, +400 readback] per word',
                            order=order, writer='sequencer'))
    for order in ('primary-first', 'second-first'):
        p0, p1 = 0x55555555, 0xaaaaaaaa
        writes = []
        for off in BANK_WORDS:
            pair = [(off, p0 ^ off), (off + 0x400, p1 ^ off)]
            writes += pair if order == 'primary-first' else pair[::-1]
        body = clear + Q.wr(0xd70, 1) + 'gate:\n    ld r2,[0xd74]\n    bz r2,gate\n'
        for k, off in enumerate(BANK_WORDS):
            body += Q.store(off, a + 0x2000 + 8 * k) + Q.store(off + 0x400, a + 0x2004 + 8 * k)
        body += clear
        yield Case(f'cpu/{order}', finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2100, flags=DISCOVERY | P.FULLMAP, timeout_us=50000,
                        params=[(0xd74, 0)], late=writes + [(0xd74, 1)], late_us=2000),
                   dict(words=[hex(o) for o in BANK_WORDS], written={'primary': hex(p0), 'second': hex(p1)},
                        xor_offset=True, layout='0x2000+8i: [primary readback, +400 readback] per word',
                        order=order, writer='cpu (scheduled at 2000 us), sequencer readback'))


def bch8_sector(data, meta=b''):
    from .bch import bch8
    return bch8(data + meta)


# (label, data seed, data errors, metadata errors, parity errors, special)
BCH_PAGES = (
    (('d0', 1, 0, 0, 0), ('d1', 2, 1, 0, 0), ('d2', 3, 2, 0, 0), ('d3', 4, 3, 0, 0),
     ('d4', 5, 4, 0, 0), ('d5', 6, 5, 0, 0), ('d6', 7, 6, 0, 0), ('d7', 8, 7, 0, 0)),
    (('m1', 9, 0, 1, 0), ('d8', 10, 8, 0, 0), ('d9', 11, 9, 0, 0), ('p1', 12, 0, 0, 1),
     ('p8', 13, 0, 0, 8), ('p9', 14, 0, 0, 9), ('d4p4', 15, 4, 0, 4), ('d5p4', 16, 5, 0, 4)),
    (('m8', 17, 0, 8, 0), ('zeros', None, 0, 0, 0), ('ones', 'ff', 0, 0, 0), ('burst8', 18, 'burst', 0, 0),
     ('bit4095', 19, 'last', 0, 0), ('bit0', 20, 'first', 0, 0), ('d10', 21, 10, 0, 0), ('d16', 22, 16, 0, 0)),
    (('m4d4', 23, 4, 4, 0), ('d0b', 24, 0, 0, 0), ('d0c', 25, 0, 0, 0), ('d0d', 26, 0, 0, 0),
     ('d0e', 27, 0, 0, 0), ('d0f', 28, 0, 0, 0), ('d0g', 29, 0, 0, 0), ('d0h', 30, 0, 0, 0)),
)
BCH_META = bytes(range(0x30, 0x3a))


def flip(buf, bits):
    buf = bytearray(buf)
    for b in bits:
        buf[b // 8] ^= 0x80 >> (b % 8)
    return bytes(buf)


def bch_page(page):
    """Raw page (4096+128) with host BCH8 parity, then the specified bit errors."""
    raw, clean, info = bytearray(b'\xff' * (4096 + 128)), [], []
    for sector, (label, seed, derr, merr, perr) in enumerate(BCH_PAGES[page]):
        data = (bytes(512) if seed is None else b'\xff' * 512 if seed == 'ff'
                else bytes((i * 29 + seed * 7 + (i >> 8)) & 255 for i in range(512)))
        meta = BCH_META if sector == 0 else b''
        par = bch8_sector(data, meta)
        clean.append(data)
        if derr == 'burst': dbits = list(range(1000, 1008))
        elif derr == 'last': dbits = [4095]
        elif derr == 'first': dbits = [0]
        else: dbits = [(k * 977 + sector * 131 + page * 37) % 4096 for k in range(derr)]
        mbits = [(k * 23 + 5) % 80 for k in range(merr)]
        pbits = [(k * 13 + sector) % 104 for k in range(perr)]
        base = 0 if sector == 0 else 10 + 525 * sector
        if sector == 0:
            raw[0:10] = flip(meta, mbits)
            raw[10:23], raw[23:535] = flip(par, pbits), flip(data, dbits)
        else:
            raw[base:base + 13], raw[base + 13:base + 525] = flip(par, pbits), flip(data, dbits)
        info.append(dict(label=label, data_bits=dbits, meta_bits=mbits, parity_bits=pbits,
                         errors=len(dbits) + len(mbits) + len(pbits)))
    return bytes(raw), clean, info


def bch8_correct(s, t):
    """Decode host-computed BCH8 code words with known bit errors on the scratch block.

    Erase, raw-program four pages, read each with the normal ECC path (data,
    corrected metadata and ECC_RESULT per sector), raw-read each back to prove
    the written bits, and read one erased page. Destructive on the scratch
    block only (geometry block, pages 0..4).
    """
    import hashlib
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    print('PLAN: m2_bch8_correct erases CE%d block %d and programs pages 0..3' % (g.ce, g.block), flush=True)
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block, ce=g.ce), pins=True)
    for page in range(len(BCH_PAGES)):
        raw, _, info = bch_page(page)
        mem = memory()
        mem[:len(raw)] = raw
        yield Case(f'program{page}', finish(t, Q.write_page(g, g.row + page, a, BCH_META, raw=True)),
                   dict(mutate, scratch=mem, scratch_read=0), dict(row=g.row + page, raw_sha256=hashlib.sha256(raw).hexdigest(),
                   sectors=info, check='r5 bit 0 = program FAIL'), pins=True)
    for page in range(len(BCH_PAGES) + 1):
        conditions = dict(row=g.row + page, layout='0x2000 data (4096), 0x3000 meta words, 0x3100 ECC_RESULT x8')
        if page < len(BCH_PAGES):
            raw, clean, info = bch_page(page)
            conditions.update(sectors=info, clean_sha256=[hashlib.sha256(c).hexdigest() for c in clean],
                              meta=BCH_META.hex())
        else:
            conditions.update(sectors='erased page control')
        yield Case(f'decode{page}', finish(t, Q.read_page(g, g.row + page, a + 0x2000, a + 0x3100, a + 0x3000)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000), conditions, pins=True)
    for page in range(len(BCH_PAGES)):
        raw, _, _ = bch_page(page)
        yield Case(f'raw{page}', finish(t, Q.read_page(g, g.row + page, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                   dict(row=g.row + page, expected_raw_sha256=hashlib.sha256(raw).hexdigest(),
                        layout='0x2000 raw page 4224 bytes'), pins=True)


_LABEL = [0]


def delay(n):
    _LABEL[0] += 1
    return '    movi r3,#%d\ndl%d:\n    sub r3,r3,#1\n    bnz r3,dl%d\n' % (n, _LABEL[0], _LABEL[0])


STAGE_REGS = (0x80c, 0x810, 0x818, 0x048, 0xc14)


def stage(a, k):
    """Store ECC_RESULT, ECC_STATUS, 818, FMC_STATUS and C14 at a + 20k."""
    return ''.join(Q.store(off, a + 20 * k + 4 * i) for i, off in enumerate(STAGE_REGS))


# Error counts per sector for the timing page; sector 0 carries the metadata.
TIMING_ERRORS = (0, 1, 5, 8, 9, 0, 3, 16)
TIMING_STAGES = ('after-kick', 'after-wait4', 'after-ack', 'delay20', 'delay200', 'delay2000')


def timing_page():
    raw, clean, info = bytearray(b'\xff' * (4096 + 128)), [], []
    for sector, errs in enumerate(TIMING_ERRORS):
        data = bytes((i * 31 + sector * 11 + (i >> 7)) & 255 for i in range(512))
        meta = BCH_META if sector == 0 else b''
        par = bch8_sector(data, meta)
        bits = [(k * 977 + sector * 131) % 4096 for k in range(errs)]
        base = 0 if sector == 0 else 10 + 525 * sector
        if sector == 0:
            raw[0:10], raw[10:23], raw[23:535] = meta, par, flip(data, bits)
        else:
            raw[base:base + 13], raw[base + 13:base + 525] = par, flip(data, bits)
        clean.append(data)
        info.append(dict(errors=errs, data_bits=bits))
    return bytes(raw), clean, info


def read_page_stages(g, row, dest, cap, variant):
    """READ path as the original, with six status captures around every decode.

    variant: original (clear 1ff, kick, WAIT4, ack 1); ack-all (ack 1ff);
    no-clear (no 1ff before the kick); wait-twice (a second WAIT4 after the ack).
    """
    src = Q.setup(g.ce) + Q.command(0) + Q.address(row) + Q.command(0x30) + Q.ready() + Q.command(0)
    for sector in range(8):
        slot, metalen = sector & 1, (10 if sector == 0 else 0)
        src += Q.transfer(g.parity + metalen, slot, parity=True) + Q.transfer(512, slot)
        base = cap + sector * 20 * len(TIMING_STAGES)
        if variant != 'no-clear':
            src += Q.wr(0x810, 0x1ff)
        src += Q.wr(0x1c, slot) + Q.wr(0x804, 1 | (metalen << 8)) + stage(base, 0)
        src += Q.wait(4) + stage(base, 1)
        src += Q.wr(0x810, 0x1ff if variant == 'ack-all' else 1)
        if variant == 'wait-twice':
            src += Q.wait(4)
        src += stage(base, 2) + delay(20) + stage(base, 3) + delay(200) + stage(base, 4)
        src += delay(2000) + stage(base, 5)
        src += Q.dma(dest + sector * 512, slot, into=False)
    return src


def ecc_status_timing(s, t):
    """Where and when the BCH8 decode reports its corrected count / failure."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, clean, info = timing_page()
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, BCH_META, raw=True)), dict(mutate, scratch=mem, scratch_read=0),
               dict(raw_sha256=hashlib.sha256(raw).hexdigest(), sectors=info), pins=True)
    yield Case('raw', finish(t, Q.read_page(g, g.row, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
               dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
               dict(expected_raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    for variant in ('original', 'ack-all', 'no-clear', 'wait-twice'):
        yield Case(variant, finish(t, read_page_stages(g, g.row, a + 0x2000, a + 0x3200, variant)),
                   dict(scratch=memory(), scratch_read=0x3200 + 8 * 20 * len(TIMING_STAGES), timeout_us=300000),
                   dict(variant=variant, sectors=info, stages=TIMING_STAGES, regs=[hex(r) for r in STAGE_REGS],
                        layout='0x2000 data; 0x3200 + sector*120 + stage*20: 80c 810 818 048 c14',
                        clean_sha256=[hashlib.sha256(c).hexdigest() for c in clean]), pins=True)


ENCODE_DATA = ('zeros', 'ones', 'bit0', 'bit4095', 'ramp', 'seed1', 'seed2')


def encode_data(label):
    if label == 'zeros': return bytes(512)
    if label == 'ones': return b'\xff' * 512
    if label == 'bit0': return b'\x80' + bytes(511)
    if label == 'bit4095': return bytes(511) + b'\x01'
    if label == 'ramp': return bytes(i & 255 for i in range(512))
    seed = int(label[-1])
    return bytes((i * 53 + seed * 101 + (i >> 5) * 7) & 255 for i in range(512))


def encode_page(g, row, source, strength, meta):
    """Seven engine-encoded sectors written as meta | (par | data) x 7 (fits 4224 for BCH16)."""
    plen = 13 if strength == 8 else 26
    src = Q.setup(g.ce) + Q.command(0x80) + Q.address(row) + Q.metadata(meta)
    for sector in range(len(ENCODE_DATA)):
        slot, metalen = sector & 1, (10 if sector == 0 else 0)
        src += Q.dma(source + 512 * sector, slot) + Q.ecc(slot, True, strength, metalen)
        src += Q.transfer(plen + metalen, slot, parity=True, into=False) + Q.transfer(512, slot, into=False)
    return src + Q.command(0x10) + Q.ready() + '    ld r5, [0x4c]\n'


def bch16_encode(s, t):
    """Engine BCH16 (and BCH8 control) parity for known data, recovered by raw read."""
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = memory()
    for sector, label in enumerate(ENCODE_DATA):
        mem[512 * sector:512 * (sector + 1)] = encode_data(label)
    mutate = dict(flags=MUTATE, timeout_us=300000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    pages = (('bch8-control', 8), ('bch16', 16), ('bch16-repeat', 16))
    for page, (label, strength) in enumerate(pages):
        yield Case(f'program/{label}', finish(t, encode_page(g, g.row + page, a, strength, BCH_META)),
                   dict(mutate, scratch=mem, scratch_read=0),
                   dict(strength=strength, sectors=ENCODE_DATA, meta=BCH_META.hex(),
                        layout='meta[10] | par | data[512], then 6 x (par | data); par 13 or 26 bytes'), pins=True)
    for page, (label, strength) in enumerate(pages):
        yield Case(f'raw/{label}', finish(t, Q.read_page(g, g.row + page, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                   dict(strength=strength, sectors=ENCODE_DATA, layout='0x2000 raw page 4224 bytes'), pins=True)


POLLS = 48


def status_poll(a, n=POLLS, spacing=300):
    """Repeated 70h + latch; store FMC_STATUS, 04c and C14 without WAIT5/12, then release."""
    src = ''
    for k in range(n):
        src += Q.command(0x70) + Q.wr(4, 0xca) + delay(spacing)
        src += Q.store(0x4c, a + 12 * k) + Q.store(0x48, a + 12 * k + 4) + Q.store(0xc14, a + 12 * k + 8)
        src += Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000)
    return src


def status_busy(s, t):
    """Does the latched status byte follow the chip's busy -> ready transition?"""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mutate = dict(flags=MUTATE, timeout_us=500000)
    cap = a + 0x2000
    common = dict(scratch_read=0x2000 + 12 * POLLS, layout='0x2000 + 12k: 04c, 048, c14 per poll')
    yield Case('idle', finish(t, Q.setup(g.ce) + status_poll(cap)),
               dict(scratch=memory(), scratch_read=common['scratch_read'], timeout_us=200000),
               dict(common, operation='none'), pins=True)
    yield Case('erase', finish(t, Q.setup(g.ce) + Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0)
                          + status_poll(cap) + Q.ready()),
               dict(mutate, scratch=memory(), scratch_read=common['scratch_read']),
               dict(common, operation='erase block %d' % g.block), pins=True)
    mem = memory()
    prog = Q.write_page(g, g.row, a, bytes(10), raw=True)
    prog = prog[:prog.index(Q.command(0x10))] + Q.command(0x10)
    yield Case('program', finish(t, prog + status_poll(cap) + Q.ready()),
               dict(mutate, scratch=mem, scratch_read=common['scratch_read']),
               dict(common, operation='raw program row %d' % g.row), pins=True)
    yield Case('read', finish(t, Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30)
                         + status_poll(cap) + Q.ready()),
               dict(scratch=memory(), scratch_read=common['scratch_read'], timeout_us=200000),
               dict(common, operation='page read (00/30) row %d' % g.row), pins=True)
    yield Case('no-ce', finish(t, Q.setup(None) + status_poll(cap, 8)),
               dict(scratch=memory(), scratch_read=0x2000 + 12 * 8, timeout_us=200000),
               dict(operation='no chip enable', layout=common['layout']), pins=True)


def coded_page(errors, strength=8, meta_errors=0, parity_errors=()):
    """Raw page of host code words: meta | (par | data) per sector, known data-bit errors."""
    import sys
    plen = 13 if strength == 8 else 26
    g = None
    raw, clean = bytearray(b'\xff' * (4096 + 128)), []
    offset = 10
    for sector, errs in enumerate(errors):
        data = bytes((i * 37 + sector * 13 + (i >> 6)) & 255 for i in range(512))
        meta = BCH_META if sector == 0 else b''
        par = bch8_sector(data, meta) if strength == 8 else bch16_parity(data + meta)
        perr = parity_errors[sector] if parity_errors else 0
        dbits = [(k * 977 + sector * 131 + 7) % 4096 for k in range(errs)]
        pbits = [(k * 13 + sector) % (8 * plen) for k in range(perr)]
        if sector == 0:
            raw[0:10] = flip(meta, [(k * 23 + 5) % 80 for k in range(meta_errors)])
        raw[offset:offset + plen] = flip(par, pbits)
        raw[offset + plen:offset + plen + 512] = flip(data, dbits)
        offset += plen + 512
        clean.append(data)
    return bytes(raw), clean


def bch16_parity(message):
    from .bch import bch16
    return bch16(message)


def decode_loop(g, row, dest, sectors, strength, per_sector):
    """Original-style READ of `sectors` code words; per_sector(sector, slot) is inserted after the ack."""
    plen = 13 if strength == 8 else 26
    src = Q.setup(g.ce) + Q.command(0) + Q.address(row) + Q.command(0x30) + Q.ready() + Q.command(0)
    for sector in range(sectors):
        slot, metalen = sector & 1, (10 if sector == 0 else 0)
        src += Q.transfer(plen + metalen, slot, parity=True) + Q.transfer(512, slot)
        src += Q.wr(0x810, 0x1ff) + Q.wr(0x1c, slot) + Q.wr(0x804, (1 if strength == 8 else 5) | (metalen << 8))
        src += Q.wait(4) + per_sector(sector, slot) + Q.wr(0x810, 1)
        src += Q.dma(dest + sector * 512, slot, into=False)
        if sector == 0:
            _LABEL[0] += 1
            src += Q.wr(0x40, 0xc2) + '_meta%d:\n    ld r0, [0x40], 2\n    bnz r0, _meta%d\n' % (_LABEL[0], _LABEL[0])
            src += ''.join(Q.store(0x60 + k, dest - 0x40 + k) for k in (0, 4, 8))
    return src


RING_ERRORS = (1, 2, 3, 4, 5, 6, 7, 8)


def ecc_ring(s, t):
    """0x80c ring: length, read-pointer advance, reset, and the 0x810 bit-5 threshold."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, clean = coded_page(RING_ERRORS)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, BCH_META, raw=True)), dict(mutate, scratch=mem, scratch_read=0),
               dict(errors=RING_ERRORS, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    dest, cap = a + 0x2040, a + 0x3000

    def status(sector, slot):  # 810 and 818 per sector, no 80c read
        return Q.store(0x810, cap + 8 * sector) + Q.store(0x818, cap + 8 * sector + 4)

    def apple(sector, slot):
        return status(sector, slot) + Q.store(0x80c, cap + 0x80 + 4 * sector)

    def reads(n, base, between=''):
        return ''.join(Q.store(0x80c, base + 4 * k) + between for k in range(n))

    variants = {
        'burst24': ('', status, reads(24, cap + 0x100)),
        'reset-first': (Q.store(0x818, cap + 0x1f8) + Q.wr(0x81c, 1) + Q.store(0x818, cap + 0x1fc), status, reads(24, cap + 0x100)),
        'interleaved': ('', status, reads(24, cap + 0x100, Q.store(0x810, cap + 0x180) + Q.store(0x818, cap + 0x184))),
        'apple-reset': (Q.wr(0x81c, 1), apple, reads(16, cap + 0x100)),
        'apple-noreset': ('', apple, reads(16, cap + 0x100)),
        'write80c': ('', status, Q.wr(0x80c, 0) + reads(24, cap + 0x100)),
        'two-pages': ('', status, None),
    }
    for name, (pre, hook, post) in variants.items():
        body = pre + decode_loop(g, g.row, dest, 8, 8, hook)
        if post is None:
            body += decode_loop(g, g.row, dest, 8, 8, lambda sec, slot: '') + reads(24, cap + 0x100)
        else:
            body += post
        yield Case(name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x3200, timeout_us=300000),
                   dict(variant=name, errors=RING_ERRORS, clean_sha256=[hashlib.sha256(c).hexdigest() for c in clean],
                        layout='0x2000 meta words, 0x2040 data; 0x3000+8s: 810,818; 0x3080+4s: 80c per sector '
                               '(apple*); 0x3100+4k: 80c reads after the page; 0x31f8/0x31fc: 818 before/after 81c'),
                   pins=True)


BCH16_PAGES = (
    dict(errors=(0, 1, 8, 15, 16, 17, 24), meta_errors=3, parity_errors=()),
    dict(errors=(0, 12, 9, 0, 2, 20, 16), meta_errors=0, parity_errors=(0, 4, 8, 16, 0, 0, 1)),
)


def bch16_correct(s, t):
    """BCH16 decode of host code words written raw (seven sectors per page)."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mutate = dict(flags=MUTATE, timeout_us=300000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    for page, spec in enumerate(BCH16_PAGES):
        raw, clean = coded_page(spec['errors'], 16, spec['meta_errors'], spec['parity_errors'])
        mem = memory()
        mem[:len(raw)] = raw
        yield Case(f'program{page}', finish(t, Q.write_page(g, g.row + page, a, BCH_META, raw=True)),
                   dict(mutate, scratch=mem, scratch_read=0), dict(spec, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    cap = a + 0x3000
    for page, spec in enumerate(BCH16_PAGES):
        raw, clean = coded_page(spec['errors'], 16, spec['meta_errors'], spec['parity_errors'])
        hook = lambda sec, slot: (Q.store(0x810, cap + 12 * sec) + Q.store(0x818, cap + 12 * sec + 4)
                                  + Q.store(0x80c, cap + 12 * sec + 8))
        yield Case(f'decode{page}', finish(t, Q.wr(0x81c, 1) + decode_loop(g, g.row + page, a + 0x2040, 7, 16, hook)),
                   dict(scratch=memory(), scratch_read=0x3100, timeout_us=300000),
                   dict(spec, clean_sha256=[hashlib.sha256(c).hexdigest() for c in clean],
                        layout='0x2000 meta words, 0x2040 data (7 x 512); 0x3000+12s: 810, 818, 80c'), pins=True)
        yield Case(f'raw{page}', finish(t, Q.read_page(g, g.row + page, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                   dict(expected_raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)


def ecc_threshold(s, t):
    """BCH8 810 bit-5 threshold with 1..8 errors (non-overlapping layout), erased-page 810,
    and whether 0x81c clears 0x818. One 80c read per sector after 0x81c."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, clean = coded_page(RING_ERRORS)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, BCH_META, raw=True)), dict(mutate, scratch=mem, scratch_read=0),
               dict(errors=RING_ERRORS, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    cap = a + 0x3100
    hook = lambda sec, slot: (Q.store(0x810, cap + 12 * sec) + Q.store(0x818, cap + 12 * sec + 4)
                              + Q.store(0x80c, cap + 12 * sec + 8))
    for page, label in ((0, 'errors1to8'), (1, 'erased')):
        body = Q.wr(0x81c, 1) + decode_loop(g, g.row + page, a + 0x2040, 8, 8, hook)
        body += Q.store(0x818, cap + 0x80) + Q.wr(0x81c, 1) + Q.store(0x818, cap + 0x84) + Q.store(0x810, cap + 0x88)
        yield Case(label, finish(t, body), dict(scratch=memory(), scratch_read=0x3200, timeout_us=300000),
                   dict(errors=RING_ERRORS if page == 0 else 'erased page',
                        clean_sha256=[hashlib.sha256(c).hexdigest() for c in clean] if page == 0 else None,
                        layout='0x2000 meta, 0x2040 data; 0x3100+12s: 810 818 80c; 0x3180/84: 818 before/after 81c; 0x3188: 810 after 81c'),
                   pins=True)


WAIT4_ACKS = (0x0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x1ff, 'reset81c', 'no-kick')


def wait4_ack(s, t):
    """Which 0x810 write (or 0x81c) makes the next WAIT4 wait. Short start word 0x07b5
    (p18: fault at 4.4 ms) keeps a failing WAIT4 cheap."""
    a = t.scratch_addr(s)
    for ack in WAIT4_ACKS:
        body = Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff)
        if ack != 'no-kick':
            body += Q.wr(0x1c, 0) + Q.wr(0x804, 2) + Q.wait(4)
        body += Q.store(0x810, a + 0x2000)
        if ack == 'reset81c':
            body += Q.wr(0x81c, 1)
        elif ack != 'no-kick':
            body += Q.wr(0x810, ack)
        body += Q.store(0x810, a + 0x2004) + '    movi r2,#0x77770001\n' + Q.wait(4) + '    movi r2,#0x77770002\n'
        label = ack if isinstance(ack, str) else 'ack%03x' % ack
        yield Case(label, finish(t, body), dict(scratch=memory(), scratch_read=0x2010, start_value=0x07b5,
                                                flags=DISCOVERY, timeout_us=50000),
                   dict(ack=ack if isinstance(ack, str) else hex(ack), stages='r2 0x77770002 = second WAIT4 passed',
                        layout='0x2000: 810 after first WAIT4, 0x2004: 810 after the write'))


ECC_MODES = (0x002, 0x302, 0xa02, 0xc02, 0x006, 0xa06) + tuple(2 | (1 << b) for b in (3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23))


def ecc_modes(s, t, modes=None):
    """ECC_CTRL encode words: metadata length field and every other single bit.

    One page per mode: one engine-encoded sector written as metalen | 26-byte
    parity area | data, then raw read. Data = ramp; metadata = BCH_META.
    """
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = memory()
    mem[:512] = encode_data('ramp')
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    modes = ECC_MODES if modes is None else modes
    for page, kick in enumerate(modes):
        metalen = (kick >> 8) & 0xff
        body = Q.setup(g.ce) + Q.command(0x80) + Q.address(g.row + page) + Q.metadata(BCH_META)
        body += Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, kick) + Q.wait(4)
        body += Q.store(0x810, a + 0x3000) + Q.store(0x80c, a + 0x3004) + Q.store(0x818, a + 0x3008)
        body += Q.wr(0x810, 1)
        body += Q.transfer(26 + metalen, 0, parity=True, into=False) + Q.transfer(512, 0, into=False)
        body += Q.command(0x10) + Q.ready() + '    ld r5, [0x4c]\n'
        yield Case(f'program/{kick:04x}', finish(t, body), dict(mutate, scratch=mem, scratch_read=0x3010),
                   dict(kick=hex(kick), metalen=metalen, layout='page: metalen bytes | 26-byte parity area | 512 data; 0x3000: 810 80c 818'),
                   pins=True)
    for page, kick in enumerate(modes):
        yield Case(f'raw/{kick:04x}', finish(t, Q.read_page(g, g.row + page, a + 0x2000, a + 0x3100, a + 0x3000, raw=True)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=200000),
                   dict(kick=hex(kick), layout='0x2000 raw page'), pins=True)


ECC_MODES2 = tuple(2 | (1 << b) for b in range(24, 32)) + (0x003, 0x007, 0xd02, 0x1002, 0x2002)


def ecc_modes2(s, t):
    """ECC_CTRL bits 24..31, encode+decode words, metadata length 13, 16, 32."""
    yield from ecc_modes(s, t, ECC_MODES2)


WAIT4_DECODE_ACKS = (0x1, 0x2, 0x4, 0x40, 0x1ff)


def wait4_decode(s, t):
    """Decode-side WAIT4: decode an erased sector (810 0x55), write one 810 value, WAIT4 again."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    yield Case('erase', finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for ack in WAIT4_DECODE_ACKS:
        body = Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 5) + Q.command(0x30) + Q.ready() + Q.command(0)
        body += Q.transfer(g.parity + 10, 0, parity=True) + Q.transfer(512, 0)
        body += Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 0xa01) + Q.wait(4)
        body += Q.store(0x810, a + 0x2000) + Q.wr(0x810, ack) + Q.store(0x810, a + 0x2004)
        body += '    movi r2,#0x77770001\n' + Q.wait(4) + '    movi r2,#0x77770002\n'
        yield Case('ack%03x' % ack, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=0x07b5, flags=DISCOVERY, timeout_us=50000),
                   dict(ack=hex(ack), row=g.row + 5, stages='r2 0x77770002 = second WAIT4 passed'), pins=True)


BCH16_THRESHOLD = dict(errors=(4, 5, 6, 7, 8, 0, 12), meta_errors=0, parity_errors=())


def bch16_threshold(s, t):
    """810 bit-5 threshold for BCH16 (4..8 and 12 errors)."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    spec = BCH16_THRESHOLD
    raw, clean = coded_page(spec['errors'], 16)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=300000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, BCH_META, raw=True)), dict(mutate, scratch=mem, scratch_read=0),
               dict(spec, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    cap = a + 0x3000
    hook = lambda sec, slot: (Q.store(0x810, cap + 12 * sec) + Q.store(0x818, cap + 12 * sec + 4)
                              + Q.store(0x80c, cap + 12 * sec + 8))
    yield Case('decode', finish(t, Q.wr(0x81c, 1) + decode_loop(g, g.row, a + 0x2040, 7, 16, hook)),
               dict(scratch=memory(), scratch_read=0x3100, timeout_us=300000),
               dict(spec, clean_sha256=[hashlib.sha256(c).hexdigest() for c in clean],
                    layout='0x2000 meta, 0x2040 data (7 x 512); 0x3000+12s: 810 818 80c'), pins=True)


OPCOST = (('empty', ''), ('nop08', '    .insn 0x08,0,0,0\n'), ('movi', '    movi r1,#0x1234\n'),
          ('add', '    add r1,r1,#1\n'), ('ld-gpr', '    ld r1,[0xc18]\n'), ('ld-pram', '    ld r1,[0xd00]\n'),
          ('ld-fmc', '    ld r1,[0x48]\n'), ('ld-ecc', '    ld r1,[0x818]\n'), ('wr-pram', '    wr [0xd70],#1\n'),
          ('ldi-sdram', '    ldi r1,[r5]\n'), ('sti-sdram', '    sti [r5],r1\n'), ('nop2ff', '    .insn 0xff,0,0,0\n'))
OPCOST_N = (1000, 21000)


def opcost(s, t):
    """Per-instruction time: loops of N = 1000 and 21000 iterations, program in SDRAM and SRAM."""
    _LABEL[0] = 0
    a = t.scratch_addr(s)
    for label, ins in OPCOST:
        for n in OPCOST_N:
            for sram in (False, True):
                _LABEL[0] += 1
                body = '    movi r5,#0x%x\n    movi r4,#%d\nloop%d:\n%s    sub r4,r4,#1\n    bnz r4,loop%d\n' % (
                    a + 0x100, n, _LABEL[0], ins, _LABEL[0])
                flags = P.PROG_SRAM if sram else 0
                yield Case(f'{label}/n{n}/{"sram" if sram else "sdram"}', finish(t, body),
                           dict(scratch=memory(), scratch_read=0x200, flags=flags, timeout_us=200000),
                           dict(instruction=label, iterations=n, sram=sram,
                                method='(elapsed(21000) - elapsed(1000)) / 20000 = loop cost; minus empty = instruction'))


def fmc_misc(s, t):
    """FMC 018 and 05c on an encoded page read: data, ECC and FMC_STATUS/DMA_CTRL after."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    from .m2_experiments import pattern
    mem = memory()
    mem[:4096] = pattern()
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, bytes(range(10)))), dict(mutate, scratch=mem, scratch_read=0),
               dict(data='pattern()', meta='00..09'), pins=True)
    base = Q.read_page(g, g.row, a + 0x2000, a + 0x3100, a + 0x3000)
    last = Q.dma(a + 0x2000 + 7 * 512, 1, into=False)
    variants = {
        'baseline': base,
        'fmc018-before-data': base.replace(Q.transfer(512, 0), Q.wr(0x18, 1) + Q.transfer(512, 0))
                                  .replace(Q.transfer(512, 1), Q.wr(0x18, 1) + Q.transfer(512, 1)),
        'fmc05c-before-final-dma': base.replace(last, Q.wr(0x5c, 0x200) + last),
    }
    for name, src in variants.items():
        src += Q.store(0x48, a + 0x3200) + Q.store(0x38, a + 0x3204) + Q.store(0x18, a + 0x3208) + Q.store(0x5c, a + 0x320c)
        yield Case(name, finish(t, src), dict(scratch=memory(), scratch_read=0x3210, timeout_us=200000),
                   dict(variant=name, expected_sha256=hashlib.sha256(pattern()).hexdigest(),
                        layout='0x2000 data, 0x3000 meta, 0x3100 results; 0x3200: 048 038 018 05c after'), pins=True)


LAND = 0x200
JUMP_BITS = (0, 1, 2, 3) + tuple(range(4, 21))  # <= 1 MiB past the program: stays in SDRAM


def jump_program(t, kind, bit):
    """Jump to LAND + 2^bit (bits 0..2 unaligned, 3 mid-pad, 4.. aligned).

    Landing pads at LAND (marker 0x600d0000) and LAND + 2^j (0x600d0000 | j << 8)
    for j = 4..min(bit, 14) (assembler limit 32 KiB); `end` padding leaves d70
    untouched. Targets beyond the program fall outside it unless truncated.
    """
    target = LAND + (1 << bit)
    jump = ('    movi r1,#0x%x\n    .insn 0xf,0,1,0\n' % target if kind == 'jr'
            else '    movi r1,#1\n    .insn 0xe,1,0,0x%x\n' % target)
    src = t.prologue(c58=0) + '    movi r2,#0\n' + jump + '    wr [0xd70],#0xbad0bad0\n    end\n'

    def pad_to(src, offset):
        # Every instruction is 8 bytes; labels, blank lines and comments take none.
        size = 8 * sum(1 for line in src.splitlines()
                       if line.strip() and not line.strip().endswith(':') and not line.strip().startswith(';'))
        if size > offset:
            raise ValueError('jump program prefix exceeds landing offset')
        return src + '    end\n' * ((offset - size) // 8)

    src = pad_to(src, LAND) + '    wr [0xd70],#0x600d0000\n    end\n'
    for j in range(4, min(bit, 14) + 1):  # assembler limit: 4096 instructions
        src = pad_to(src, LAND + (1 << j)) + '    wr [0xd70],#0x%x\n    end\n' % (0x600d0000 | j << 8)
    return src + t.trap(), target


def jump_width(s, t):
    """JR and BNZ target width and alignment: which landing pad (if any) executes."""
    for kind in ('jr', 'bnz'):
        for bit in JUMP_BITS:
            src, target = jump_program(t, kind, bit)
            yield Case(f'{kind}/bit{bit:02d}', src,
                       dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                       dict(kind=kind, target=hex(target), land=hex(LAND),
                            markers='d70: 0x600d0000 land, 0x600d0j00 land+2^j, 0 = landed in zero padding (END), 0xbad0bad0 not taken'))


RB_WAITS = tuple(range(16))


def wait_rb(s, t):
    """Search for a WAIT event that follows the NAND ready/busy line.

    Start a block erase (busy > 1 ms), WAIT N, then latch the status byte: 04c
    0x80 means the WAIT released while the chip was busy, 0xe0 after it became
    ready. Start word 0x40b5 (32.5 ms fault) exceeds the erase time.
    """
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    for n in RB_WAITS:
        body = Q.setup(g.ce) + Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0)
        body += '    movi r2,#0x77770001\n' + Q.wait(n) + '    movi r2,#0x77770002\n'
        body += Q.store(0x48, a + 0x2000) + Q.store(0xc50, a + 0x2004)
        body += Q.command(0x70) + Q.wr(4, 0xca) + delay(20) + Q.store(0x4c, a + 0x2008)
        body += Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000) + Q.ready()
        yield Case(f'wait{n:02d}', finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=0x40b5, flags=DISCOVERY | MUTATE, timeout_us=100000),
                   dict(wait=n, layout='0x2000: 048, c50 after the WAIT; 0x2008: status byte (0x80 busy, 0xe0 ready)',
                        stages='r2 0x77770002 = WAIT passed'), pins=True)


NOP = '    .insn 0xff,0,0,0\n'  # p16-confirmed one-cycle nop2


def jr_program(t, gap, bit=5, decoy=6, pre=''):
    """movi r1 = LAND + 2^decoy long before; movi r1 = LAND + 2^bit; `gap` nops; JR r1."""
    target = LAND + (1 << bit)
    decoy_target = LAND + (1 << decoy) if decoy is not None else None
    src = t.prologue(c58=0) + '    movi r2,#0\n' + pre
    src += ('    movi r1,#0x%x\n' % decoy_target + NOP * 8) if decoy is not None else ''
    src += '    movi r1,#0x%x\n' % target + NOP * gap + '    .insn 0xf,0,1,0\n    wr [0xd70],#0xbad0bad0\n    end\n'

    def pad_to(src, offset):
        size = 8 * sum(1 for line in src.splitlines()
                       if line.strip() and not line.strip().endswith(':') and not line.strip().startswith(';'))
        return src + '    end\n' * ((offset - size) // 8)

    src = pad_to(src, LAND) + '    wr [0xd70],#0x600d0000\n    end\n'
    for j in range(4, min(max(bit, decoy or 0), 14) + 1):
        src = pad_to(src, LAND + (1 << j)) + '    wr [0xd70],#0x%x\n    end\n' % (0x600d0000 | j << 8)
    return src + t.trap(), target


def jr_hazard(s, t):
    """JR target-register hazard: 0..6 nops between `movi r1` and JR; decoy pad 2^6, target pad 2^5."""
    for gap in range(7):
        src, target = jr_program(t, gap)
        yield Case(f'gap{gap}', src, dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   dict(gap=gap, target=hex(target), decoy=hex(LAND + 64),
                        markers='0x600d0500 target, 0x600d0600 decoy (stale r1), 0xbad0bad0 fell through'))
    src, target = jr_program(t, 0, decoy=None)
    yield Case('gap0/no-decoy', src, dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
               dict(gap=0, target=hex(target), decoy=None))


def jr_width(s, t):
    """JR target width and alignment with six nops after the target write."""
    for bit in JUMP_BITS:
        src, target = jr_program(t, 6, bit, decoy=None)
        yield Case(f'jr/bit{bit:02d}', src, dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   dict(kind='jr', target=hex(target), land=hex(LAND), gap=6))


HAZARD_TESTS = {
    # name: (setup long before, producer, consumer, result register)
    'movi->add': ('    movi r1,#0x100\n', '    movi r1,#5\n', '    add r2,r1,#1\n', 'r2'),
    'movi->mov': ('    movi r1,#0x100\n', '    movi r1,#5\n', '    mov r2,r1\n', 'r2'),
    'add->add': ('    movi r1,#0x100\n', '    add r1,r1,#1\n', '    add r2,r1,#1\n', 'r2'),
    'ld->add': ('    movi r1,#0x100\n', '    ld r1,[0xd00]\n', '    add r2,r1,#1\n', 'r2'),
    'ldi->add': ('    movi r1,#0x100\n', '    ldi r1,[r5]\n', '    add r2,r1,#1\n', 'r2'),
    'movi->ldi-addr': ('    movi r3,#0\n', '    movi r5,#SCR104\n', '    ldi r2,[r5]\n', 'r2'),
    'movi->sti-data': ('    movi r1,#0x100\n', '    movi r1,#5\n', '    sti [r5],r1\n' + NOP * 8 + '    ldi r2,[r5]\n', 'r2'),
    'movi->bnz': ('    movi r1,#0\n', '    movi r1,#1\n', '    bnz r1,HTAKEN\n    movi r2,#0xf00\n    bz r3,HDONE\nHTAKEN:\n    movi r2,#0x7a\nHDONE:\n', 'r2'),
}


def alu_hazard(s, t):
    """Producer-to-consumer distance 0, 1, 2, 4 for register writes; result in PRAM 0xd40 + 4k."""
    a = t.scratch_addr(s)
    for name, (setup, producer, consumer, reg) in HAZARD_TESTS.items():
        body = '    movi r3,#0\n'
        for k, gap in enumerate((0, 1, 2, 4)):
            _LABEL[0] += 1
            c = consumer.replace('HTAKEN', 'ht%d' % _LABEL[0]).replace('HDONE', 'hd%d' % _LABEL[0])
            p_ = producer.replace('SCR104', '0x%x' % (a + 0x104))
            body += '    movi r5,#0x%x\n' % (a + 0x100) + '    movi r2,#0xeeee\n' + setup + NOP * 8
            body += p_ + NOP * gap + c + NOP * 8 + '    st [0x%x],%s\n' % (0xd40 + 4 * k, reg)
        yield Case(name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x200, params=[(0xd00, 0x1111)] + [(0xd40 + 4 * k, 0) for k in range(4)],
                        flags=DISCOVERY | P.FULLMAP, timeout_us=20000),
                   dict(test=name, gaps=(0, 1, 2, 4), result='PRAM 0xd40 + 4k for gap k', scratch_word_0x100=F_word(0x100),
                        scratch_word_0x104=F_word(0x104)))


def F_word(off):
    return hex(int.from_bytes(memory()[off:off + 4], 'little'))


def jr_cond(s, t):
    """JR as `jump to r[arg] if r[R] != 0`? R = 0..7, condition register zero or one.

    All registers are seeded 1 except the condition register; the target is in
    r7 (r6 when R = 7). Six nops separate the last register write from JR.
    """
    target = LAND + 32
    for R in range(8):
        treg = 6 if R == 7 else 7
        for value in (0, 1):
            src = t.prologue(c58=0) + ''.join('    movi r%d,#1\n' % i for i in range(8))
            src += '    movi r%d,#0x%x\n' % (treg, target) + '    movi r%d,#%d\n' % (R, value) + NOP * 6
            src += '    .insn 0xf,%d,%d,0\n    wr [0xd70],#0xbad0bad0\n    end\n' % (R, treg)
            src = pad_program(src, LAND) + '    wr [0xd70],#0x600d0000\n    end\n'
            src = pad_program(src, target) + '    wr [0xd70],#0x600d0500\n    end\n' + t.trap()
            yield Case(f'R{R}/cond{value}', src, dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                       dict(R=R, target_reg=treg, cond_value=value, target=hex(target),
                            markers='0x600d0500 jumped, 0xbad0bad0 fell through'))


def pad_program(src, offset):
    size = 8 * sum(1 for line in src.splitlines()
                   if line.strip() and not line.strip().endswith(':') and not line.strip().startswith(';'))
    if size > offset:
        raise ValueError('program prefix exceeds pad offset')
    return src + '    end\n' * ((offset - size) // 8)


def jr_width2(s, t):
    """JR width and alignment with r0 = 1 (JR R field 0) and six nops."""
    for bit in JUMP_BITS:
        # p25 inserted this line after the pads were laid out (8-byte shift);
        # it now precedes the layout.
        src, target = jr_program(t, 6, bit, decoy=None, pre='    movi r0,#1\n')
        yield Case(f'jr/bit{bit:02d}', src, dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   dict(kind='jr', target=hex(target), land=hex(LAND), gap=6, r0=1))


def wait_stimuli(a, g):
    """State preparations for the WAIT source sweep; no flag is acknowledged afterwards."""
    read_erased = Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 100) + Q.command(0x30) + Q.ready() + Q.command(0)
    load = read_erased + Q.transfer(g.parity + 10, 0, parity=True) + Q.transfer(512, 0)
    return {
        'idle': Q.setup(),
        'ecc-encode': Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 2) + delay(400),
        'ecc-decode-erased': load + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 0xa01) + delay(400),
        'ecc-uncorrectable': load + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 0xa01) + delay(400),
        'meta-latch': load + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 0xa01) + delay(400) + Q.wr(0x40, 0xc2) + delay(50),
        'dma-in-unacked': Q.setup() + Q.dma(a, 0).replace(Q.wr(0x48, 0x100000), '') + delay(400),
        'status-latch': Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca) + delay(100),
    }


def wait_sources(s, t):
    """WAIT 5..10, 12..15 (arg = mask = 0) after each prepared state; 0x07b5 faults at 4.4 ms."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    yield Case('erase', finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for name, prep in wait_stimuli(a, g).items():
        for n in (5, 6, 7, 8, 9, 10, 12, 13, 14, 15):
            body = prep + Q.store(0x48, a + 0x2000) + Q.store(0x810, a + 0x2004) + Q.store(0xc50, a + 0x2008)
            body += '    movi r2,#0x77770001\n' + Q.wait(n) + '    movi r2,#0x77770002\n'
            yield Case(f'{name}/wait{n:02d}', finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2010, start_value=0x07b5, flags=DISCOVERY, timeout_us=50000),
                       dict(stimulus=name, wait=n, layout='0x2000: 048 810 c50 before the WAIT',
                            stages='r2 0x77770002 = WAIT passed'), pins=True)


FLAG_REGS = (0x048, 0xc50, 0x810, 0x04c, 0xc08, 0x038, 0x040)


def flags_snapshot(a, k):
    return ''.join(Q.store(off, a + 0x2000 + 0x40 * k + 4 * i) for i, off in enumerate(FLAG_REGS))


def fmc_flags(s, t):
    """Which FMC_STATUS / C50 / 810 / 04c / C08 bits each operation sets, before any WAIT or ack."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    cmd = lambda c: Q.wr(8, c)
    addr = Q.wr(0x2c, 4) + Q.wr(0xc, (g.row + 100) << 16) + Q.wr(0x10, (g.row + 100) >> 16) + Q.wr(4, 1)
    opened = Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 100) + Q.command(0x30) + Q.ready() + Q.command(0)
    ops = {
        'none': Q.setup(g.ce),
        'command-70': Q.setup(g.ce) + cmd(0x70),
        'command-ff-reset': Q.setup(g.ce) + cmd(0xff),
        'address-5': Q.setup(g.ce) + cmd(0) + Q.wait(0) + Q.wr(0x48, 2) + addr,
        'read-30-busy': Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 100) + cmd(0x30),
        'data-read-512': opened + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(4, 0xe2),
        'parity-read': opened + Q.wr(0x30, g.parity + 9) + Q.wr(0x14, 0x10) + Q.wr(4, 0x32),
        'status-latch': Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca),
        'dma-in': Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(0x34, a) + Q.wr(4, 0x2e0),
        'dma-out': Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x1000) + Q.wr(4, 0x100),
        'ecc-encode': Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 2),
        'ecc-decode-erased': opened + Q.transfer(g.parity + 10, 0, parity=True) + Q.transfer(512, 0)
                             + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 0xa01),
        'fifo-52': Q.setup(g.ce) + Q.wr(0x40, 0x52),
        'meta-c1': Q.setup() + Q.wr(0x40, 0xc1),
    }
    yield Case('erase', finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for name, op in ops.items():
        body = flags_snapshot(a, 0) + op + flags_snapshot(a, 1) + delay(20) + flags_snapshot(a, 2)
        body += delay(2000) + flags_snapshot(a, 3)
        flags = DISCOVERY | (MUTATE if name == 'command-ff-reset' else 0)
        yield Case(name, finish(t, body), dict(scratch=memory(), scratch_read=0x2100, flags=flags, timeout_us=100000),
                   dict(operation=name, regs=[hex(r) for r in FLAG_REGS],
                        layout='0x2000 + 0x40k: before, right after, +20 loops, +2000 loops'), pins=True)


DMA_CUT_GAPS = (0, 2, 4, 8, 16, 32, 64, 128, 256)


def dma_cut(s, t):
    """A second DMA out started k nops after the first: bytes of the first that arrive (DMA rate)."""
    a = t.scratch_addr(s)
    for slot in (0, 1):
        for gap in DMA_CUT_GAPS:
            body = Q.setup() + Q.dma(a, slot) + Q.dma(a + 0x400, 1 - slot)
            body += Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100 << slot) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
            body += NOP * gap + Q.dma(a + 0x2200, 1 - slot, into=False) + Q.wait(2) + Q.wr(0x48, 0x100000)
            body += Q.store(0x38, a + 0x2400) + Q.store(0x48, a + 0x2404)
            yield Case(f'slot{slot}/gap{gap:03d}', finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2410, timeout_us=50000),
                       dict(slot=slot, gap_nops=gap, gap_ns=round(gap * 30.08, 1),
                            reference_ranges={'0x2000': bytes(memory()[:512]).hex(), '0x2200': bytes(memory()[0x400:0x600]).hex()},
                            layout='0x2000 first output (cut), 0x2200 second output, 0x2400: 038 048'))


def timeout_low(s, t):
    """Start-word bits 7:4 with count 0x07 (bits 0 and 2 set): fault time of an unmet WAIT3."""
    for n in range(16):
        word = 0x0700 | (n << 4) | 0x5
        yield Case(f'{word:04x}', finish(t, Q.wait(3)),
                   dict(start_value=word, flags=DISCOVERY, timeout_us=150000),
                   dict(start_word=hex(word), bits_7_4=n, reference='0x07b5 faults at 4,390 us (p18)'))


ECC_BLOCK = tuple(range(0x800, 0xa00, 4))


def ecc_regs(s, t):
    """Sequencer view of 0x800..0x9fc after encodes and decodes with known inputs."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = memory()
    mem[:512] = bytes(512)
    mem[0x200:0x400] = encode_data('ramp')
    dump = lambda: ''.join(Q.store(off, a + 0x2000 + off - 0x800) for off in ECC_BLOCK if off not in (0x80c,))
    load = (Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 100) + Q.command(0x30) + Q.ready() + Q.command(0)
            + Q.transfer(g.parity + 10, 0, parity=True) + Q.transfer(512, 0))
    cases = {
        'reset-only': Q.wr(0x81c, 1),
        'encode-zeros-bch8': Q.wr(0x81c, 1) + Q.setup() + Q.dma(a, 0) + Q.ecc(0, True, 8),
        'encode-ramp-bch8': Q.wr(0x81c, 1) + Q.setup() + Q.dma(a + 0x200, 0) + Q.ecc(0, True, 8),
        'encode-ramp-bch16': Q.wr(0x81c, 1) + Q.setup() + Q.dma(a + 0x200, 0) + Q.ecc(0, True, 16),
        'encode-ramp-bch8-meta10': Q.wr(0x81c, 1) + Q.setup() + Q.metadata(BCH_META) + Q.dma(a + 0x200, 0) + Q.ecc(0, True, 8, 10),
        'decode-erased': Q.wr(0x81c, 1) + load + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 0xa01) + Q.wait(4),
        'decode-uncorrectable': Q.wr(0x81c, 1) + load + Q.dma(a + 0x200, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0)
                                + Q.wr(0x804, 0xa01) + Q.wait(4),
    }
    yield Case('erase', finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for name, body in cases.items():
        yield Case(name, finish(t, body + dump()), dict(scratch=mem, scratch_read=0x2200, timeout_us=100000),
                   dict(state=name, layout='0x2000 + (off - 0x800): sequencer read of 0x800..0x9fc (80c skipped: read advances the ring)'),
                   pins=True)


def sampler(reg, n, base, spacing=0):
    """n samples of `reg` to SDRAM at base, one per 6.25 + spacing cycles (≈ 188 ns + 30 ns per nop)."""
    # Spacing above 8 nops uses a delay loop (2.25 cycles per iteration) to stay
    # within the assembler's 4096-instruction limit.
    gap = (lambda: delay(max(1, round(spacing / 2.25)))) if spacing > 8 else (lambda: NOP * spacing)
    return ''.join('    ld r1,[0x%x]\n    sti [r7],r1\n    add r7,r7,#4\n' % reg + gap() for _ in range(n))


TIMELINE_SAMPLES = 128


def timeline(s, t):
    """Register timelines during operations: DMA, ECC, NAND data/parity reads, status latch, command, address."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    opened = Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 100) + Q.command(0x30) + Q.ready() + Q.command(0)
    loaded = opened + Q.transfer(g.parity + 10, 0, parity=True) + Q.transfer(512, 0)
    dma_regs = lambda d, slot: (Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, (1 << slot) if d == 'in' else (0x100 << slot))
                                + Q.wr(0x34, a if d == 'in' else a + 0x1000))
    ops = {}
    for d, trig in (('in', 0x2e0), ('out', 0x100)):
        for slot in (0, 1):
            pre = Q.setup() + (Q.dma(a, slot) if d == 'out' else '') + dma_regs(d, slot)
            for reg in (0x038, 0x048):
                ops[f'dma-{d}/slot{slot}/r{reg:03x}'] = (pre, Q.wr(4, trig), reg)
    ops['ecc-encode-bch8/r810'] = (Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0), Q.wr(0x804, 2), 0x810)
    ops['ecc-encode-bch16/r810'] = (Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0), Q.wr(0x804, 6), 0x810)
    ops['ecc-encode-bch8/rc50'] = (Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0), Q.wr(0x804, 2), 0xc50)
    ops['ecc-decode-erased/r810'] = (loaded + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0), Q.wr(0x804, 0xa01), 0x810)
    ops['ecc-decode-uncorrectable/r810'] = (loaded + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0), Q.wr(0x804, 0xa01), 0x810)
    ops['nand-data-512/r048'] = (opened + Q.wr(0x30, 511) + Q.wr(0x14, 1), Q.wr(4, 0xe2), 0x048)
    ops['nand-parity-23/r048'] = (opened + Q.wr(0x30, g.parity + 9) + Q.wr(0x58, 0x10), Q.wr(4, 0x12), 0x048)
    ops['status-latch/r048'] = (Q.setup(g.ce) + Q.command(0x70), Q.wr(4, 0xca), 0x048)
    ops['command-70/r048'] = (Q.setup(g.ce), Q.wr(8, 0x70), 0x048)
    ops['address-5/r048'] = (Q.setup(g.ce) + Q.command(0) + Q.wr(0x2c, 4) + Q.wr(0xc, (g.row + 100) << 16) + Q.wr(0x10, 0),
                             Q.wr(4, 1), 0x048)
    yield Case('erase', finish(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000), dict(block=g.block), pins=True)
    for name, (pre, trig, reg) in ops.items():
        body = pre + '    movi r7,#0x%x\n' % (a + 0x2000) + trig + sampler(reg, TIMELINE_SAMPLES, a + 0x2000)
        yield Case(name, finish(t, body), dict(scratch=memory(), scratch_read=0x2000 + 4 * TIMELINE_SAMPLES, timeout_us=100000,
                                               flags=DISCOVERY),
                   dict(operation=name.rsplit('/', 1)[0], register=hex(reg), samples=TIMELINE_SAMPLES, sample_ns=188.0,
                        layout='0x2000 + 4k: register value k samples after the trigger write'), pins=True)


CPU_WRITES = ((0xc0c, 0xff), (0xc10, 1), (0xc48, 1), (0xc54, 1), (0xc60, 1), (0xc64, 1), (0xc68, 1), (0xc6c, 0x10))


def wait_cpu(s, t):
    """WAIT 6..10, 12..15 while the CPU writes one control word 1 ms after the WAIT starts (0x07b5)."""
    for off, value in CPU_WRITES:
        for n in (6, 7, 8, 9, 10, 12, 13, 14, 15):
            body = Q.setup() + '    movi r2,#0x77770001\n' + Q.wait(n) + '    movi r2,#0x77770002\n'
            yield Case(f'c{off:03x}/wait{n:02d}', finish(t, body),
                       dict(start_value=0x07b5, flags=DISCOVERY, timeout_us=50000, late=[(off, value)], late_us=1000),
                       dict(cpu_write=(hex(off), hex(value)), late_us=1000, wait=n,
                            stages='r2 0x77770002 = passed; fault_at ≈ 4390 us = not released'))


def status_wait(s, t):
    """WAIT5 / masked WAIT12 against a busy chip, and WAIT5 latch orderings."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    erase_start = Q.setup(g.ce) + Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0)
    after = Q.store(0x4c, a + 0x2000) + Q.store(0x48, a + 0x2004) + Q.store(0xc50, a + 0x2008)
    variants = {
        'erase/latch-once/wait5': erase_start + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(5),
        'erase/latch-once/wait12-40': erase_start + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(12, 0x40, 0x40),
        'erase/latch-once/wait12-00-40': erase_start + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(12, 0, 0x40),
        'erase/no-latch/wait12-40': erase_start + Q.command(0x70) + Q.wait(12, 0x40, 0x40),
        'idle/latch/wait5': Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(5),
        'idle/latch-twice/wait5': Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca) + Q.wr(4, 0xca) + Q.wait(5),
        'idle/ack-cmd-before-latch/wait5': Q.setup(g.ce) + Q.wr(8, 0x70) + Q.wait(0) + Q.wr(0x48, 2) + Q.wr(4, 0xca) + Q.wait(5),
        'idle/nop-before-latch/wait5': Q.setup(g.ce) + Q.command(0x70) + NOP + Q.wr(4, 0xca) + Q.wait(5),
        'idle/no-command/wait5': Q.setup(g.ce) + Q.wr(4, 0xca) + Q.wait(5),
        'no-ce/latch/wait12-40': Q.setup(None) + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(12, 0x40, 0x40),
    }
    for name, body in variants.items():
        body = '    movi r2,#0x77770001\n' + body + '    movi r2,#0x77770002\n' + after
        if name.startswith('erase'):
            body += Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000) + Q.ready()
        yield Case(name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=0x40b5,
                        flags=DISCOVERY | (MUTATE if name.startswith('erase') else 0), timeout_us=100000),
                   dict(variant=name, layout='0x2000: 04c 048 c50 after the WAIT', reference='erase ≈ 2 ms; fault 32.5 ms'),
                   pins=True)


def ecc_decode_regs(s, t):
    """0x820..0x85c after each decode of a page with 1..8 errors: syndromes or error locations?"""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, clean = coded_page(RING_ERRORS)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, BCH_META, raw=True)), dict(mutate, scratch=mem, scratch_read=0),
               dict(errors=RING_ERRORS, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    cap = a + 0x3100
    hook = lambda sec, slot: ''.join(Q.store(off, cap + 0x40 * sec + off - 0x820) for off in range(0x820, 0x860, 4))
    yield Case('decode', finish(t, Q.wr(0x81c, 1) + decode_loop(g, g.row, a + 0x2040, 8, 8, hook)),
               dict(scratch=memory(), scratch_read=0x3300, timeout_us=300000),
               dict(errors=RING_ERRORS, layout='0x3100 + 0x40s + (off - 0x820): 0x820..0x85c after decode s'), pins=True)


RETAIN_WORDS = ((0x00c, 0x12345678), (0x010, 0x9abcdef0), (0x014, 0x0101), (0x018, 0x5), (0x01c, 1), (0x02c, 3),
                (0x030, 0x1234), (0x034, 0x08440000), (0x038, 1), (0x058, 0x20), (0x05c, 0x200), (0x804, 0x304),
                (0xd70, 0x5a5a5a5a))


def reset_retention(s, t):
    """Write storage words, end; the next case's before-map shows what the harness reset/preflight keeps."""
    body = ''.join(Q.wr(off, v) for off, v in RETAIN_WORDS)
    for k in range(2):
        yield Case(f'set{k}', finish(t, body), dict(flags=DISCOVERY | P.FULLMAP, timeout_us=20000),
                   dict(written={hex(o): hex(v) for o, v in RETAIN_WORDS}))
        yield Case(f'check{k}', finish(t, ''), dict(flags=DISCOVERY | P.FULLMAP, timeout_us=20000),
                   dict(question='before-map of this case = values left after the reset that follows set%d' % k))


def decode_timeline(s, t):
    """810 sampled during each decode of a page with 1..8 errors: correction time vs error count."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, clean = coded_page(RING_ERRORS)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, BCH_META, raw=True)), dict(mutate, scratch=mem, scratch_read=0),
               dict(errors=RING_ERRORS, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    body = Q.wr(0x81c, 1) + Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
    for sector in range(8):
        slot, metalen = sector & 1, (10 if sector == 0 else 0)
        body += Q.transfer(g.parity + metalen, slot, parity=True) + Q.transfer(512, slot)
        body += Q.wr(0x810, 0x1ff) + Q.wr(0x1c, slot) + '    movi r7,#0x%x\n' % (a + 0x2000 + 0x100 * sector)
        body += Q.wr(0x804, 1 | (metalen << 8)) + sampler(0x810, 48, 0) + Q.wait(4) + Q.wr(0x810, 0x1ff)
        body += Q.dma(a + 0x3000 + sector * 512, slot, into=False)
    yield Case('decode', finish(t, body), dict(scratch=memory(), scratch_read=0x4000, timeout_us=300000),
               dict(errors=RING_ERRORS, samples=48, sample_ns=188.0,
                    layout='0x2000 + 0x100s + 4k: 810 sample k of sector s; 0x3000 data'), pins=True)


NAND_TIMING_FIELDS = ((12, 'f14_12'), (16, 'f18_16'), (28, 'f30_28'))
NAND_TIMING_SPACING = 36  # ≈ 1.27 us per sample, 160 samples ≈ 203 us (p28: 30 us was too short)


def nand_timing(s, t):
    """FMC_CTRL0 timing fields vs the duration and correctness of a 512-byte NAND data read."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    from .m2_experiments import pattern
    mem = memory()
    mem[:4096] = pattern()
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, bytes(10))), dict(mutate, scratch=mem, scratch_read=0),
               dict(data='pattern(), encoded'), pins=True)
    base = 0x1077801 | 2 << g.ce
    for shift, label in NAND_TIMING_FIELDS:
        for value in range(8):
            word = (base & ~(7 << shift)) | (value << shift)
            body = (Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
                    + Q.wr(0, word) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + '    movi r7,#0x%x\n' % (a + 0x2000)
                    + Q.wr(4, 0xe2) + sampler(0x048, 160, 0, NAND_TIMING_SPACING) + Q.wait(3) + Q.wr(0x48, 8)
                    + Q.dma(a + 0x3000, 0, into=False) + Q.wr(0, base))
            yield Case(f'{label}/{value}', finish(t, body),
                       dict(scratch=memory(), scratch_read=0x3200, timeout_us=100000, flags=DISCOVERY),
                       dict(field=label, value=value, ctrl0=hex(word),
                            layout='0x2000 + 4k: 048 sample k (≈ 1.27 us); 0x3000: first 512 data bytes (raw, sector 0 layout)'),
                       pins=True)


def fifo_read(s, t):
    """READ ID bytes via FIFO command 0x52 and ports 0x60/0x64 and 0x080."""
    a, g = t.scratch_addr(s), Q.geometry(s)
    readid = Q.setup(g.ce) + Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1) + Q.wr(0x48, 4)
    readid += Q.wr(0x30, 7) + Q.wr(0x14, 0x10) + Q.wr(4, 0xe2) + Q.wait(3) + Q.wr(0x48, 8)
    variants = {
        'latch52-ports60': readid + Q.wr(0x40, 0x52) + delay(20) + ''.join(Q.store(o, a + 0x2000 + o - 0x60) for o in (0x60, 0x64, 0x68)),
        'port080x4': readid + ''.join(Q.store(0x80, a + 0x2000 + 4 * k) for k in range(4)),
        'latch52-port080x4': readid + Q.wr(0x40, 0x52) + delay(20) + ''.join(Q.store(0x80, a + 0x2000 + 4 * k) for k in range(4)),
    }
    for name, body in variants.items():
        yield Case(name, finish(t, body), dict(scratch=memory(), scratch_read=0x2010, timeout_us=50000, flags=DISCOVERY),
                   dict(variant=name, expected_id='ad d5 14 b6 44 (Hynix)', layout='0x2000: words read'), pins=True)


def wait_cpu2(s, t):
    """WAIT 6..10, 12..15 with CPU writes to C64, C68, C6C at 1 ms, restored to 0 at 2.5 ms (run last)."""
    for off, value in ((0xc64, 1), (0xc68, 1), (0xc6c, 0x10)):
        for n in (6, 7, 8, 9, 10, 12, 13, 14, 15):
            body = Q.setup() + '    movi r2,#0x77770001\n' + Q.wait(n) + '    movi r2,#0x77770002\n'
            yield Case(f'c{off:03x}/wait{n:02d}', finish(t, body),
                       dict(start_value=0x07b5, flags=DISCOVERY, timeout_us=50000, late=[(off, value)], late_us=1000,
                            late2=[(off, 0)], late2_us=2500),
                       dict(cpu_write=(hex(off), hex(value)), restore_us=2500, wait=n))


def c60_hold(s, t):
    """p26: C60 = 1 written by the CPU during a WAIT left C08 = 1 that STOP could not clear.

    Variants restore C60 = 0 at 2.5 ms (before the 4.4 ms fault), after the fault
    (6 ms), or write C60 during a program without a WAIT. Run last.
    """
    wait6 = Q.setup() + '    movi r2,#0x77770001\n' + Q.wait(6) + '    movi r2,#0x77770002\n'
    loop = Q.setup() + '    movi r2,#0x77770001\n' + delay(30000) + '    movi r2,#0x77770002\n'
    cases = {
        'wait/restore-2500': (wait6, 2500),
        'wait/restore-6000': (wait6, 6000),
        'loop/restore-2500': (loop, 2500),
    }
    for name, (body, restore) in cases.items():
        yield Case(name, finish(t, body + Q.store(0xc08, t.scratch_addr(s) + 0x2000) + Q.store(0xc64, t.scratch_addr(s) + 0x2004)),
                   dict(start_value=0x07b5, flags=DISCOVERY | P.FULLMAP, timeout_us=50000, scratch=memory(), scratch_read=0x2010,
                        late=[(0xc60, 1)], late_us=1000, late2=[(0xc60, 0)], late2_us=restore),
                   dict(cpu_write='c60 = 1 at 1000 us', restore_us=restore,
                        question='does clearing C60 release the hold and allow cleanup?'))


def nand_write_timing(s, t):
    """FMC_CTRL0 fields vs the duration of a 512-byte NAND data-in transfer.

    80h + address, 512 bytes out from slot 0 with 048 sampled, then FFh reset
    instead of the 10h confirm: no page is programmed.
    """
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    base = 0x1077801 | 2 << g.ce
    for shift, label in NAND_TIMING_FIELDS:
        for value in range(8):
            word = (base & ~(7 << shift)) | (value << shift)
            body = (Q.setup(g.ce) + Q.dma(a, 0) + Q.command(0x80) + Q.address(g.row + 120)
                    + Q.wr(0, word) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100) + '    movi r7,#0x%x\n' % (a + 0x2000)
                    + Q.wr(4, 0xe4) + sampler(0x048, 160, 0, NAND_TIMING_SPACING) + Q.wait(3) + Q.wr(0x48, 8)
                    + Q.wr(0, base) + Q.wr(8, 0xff) + Q.wait(0) + Q.wr(0x48, 2) + delay(2000))
            yield Case(f'{label}/{value}', finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2000 + 4 * 160, timeout_us=100000, flags=DISCOVERY | MUTATE),
                       dict(field=label, value=value, ctrl0=hex(word), aborted_with='FFh reset (no 10h confirm)',
                            layout='0x2000 + 4k: 048 sample k (≈ 1.27 us)'), pins=True)


LOC_PAGES = (
    dict(strength=8, errors=(2, 0, 9, 4, 0, 1, 8, 7), meta_errors=3, parity_errors=(0, 3, 0, 4, 0, 0, 0, 1)),
    dict(strength=16, errors=(10, 9, 12, 17, 0, 1, 16), meta_errors=2, parity_errors=(0, 0, 4, 0, 2, 0, 0)),
)
LOC_STRIDE = 0x50  # 810, 818, then 0x820..0x85c per sector


def ecc_locations(s, t):
    """0x820..0x85c after BCH8/BCH16 decodes with metadata, parity, uncorrectable and clean sectors."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    mutate = dict(flags=MUTATE, timeout_us=300000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    for page, spec in enumerate(LOC_PAGES):
        raw, clean = coded_page(spec['errors'], spec['strength'], spec['meta_errors'], spec['parity_errors'])
        mem = memory()
        mem[:len(raw)] = raw
        yield Case(f'program{page}', finish(t, Q.write_page(g, g.row + page, a, BCH_META, raw=True)),
                   dict(mutate, scratch=mem, scratch_read=0), dict(spec, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    cap = a + 0x3100
    hook = lambda sec, slot: ''.join(Q.store(off, cap + LOC_STRIDE * sec + 4 * i)
                                     for i, off in enumerate((0x810, 0x818) + tuple(range(0x820, 0x860, 4))))
    for page, spec in enumerate(LOC_PAGES):
        n = len(spec['errors'])
        yield Case(f'decode{page}', finish(t, Q.wr(0x81c, 1) + decode_loop(g, g.row + page, a + 0x2040, n, spec['strength'], hook)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=300000),
                   dict(spec, layout='0x2040 data; 0x3100 + 0x50s: 810, 818, 0x820..0x85c after decode s'), pins=True)


WAIT12_ARGS = ((0x40, 0x40), (0, 0x40), (0x40, 0), (0, 0), (0xe0, 0xff), (0xe0, 0xe0), (0x80, 0x80), (0x20, 0x20),
               (0x01, 0x01), (0, 0x01), (0x60, 0x40), (0x40, 0x60), (0x41, 0x41), (0xff, 0xff))


def wait12_args(s, t):
    """WAIT12 operands against a ready chip (status 0xe0) and the no-CE echo (0x70): which arg/mask pairs pass."""
    g = Q.geometry(s)
    targets = [('ready', g.ce, WAIT12_ARGS), ('no-ce', None, ((0x40, 0x40), (0x70, 0xff), (0xe0, 0xff), (0x10, 0x10)))]
    for label, ce, pairs in targets:
        for arg, mask in pairs:
            body = (Q.setup(ce) + Q.command(0x70) + Q.wr(4, 0xca) + '    movi r2,#0x77770001\n' + Q.wait(12, arg, mask)
                    + '    movi r2,#0x77770002\n' + Q.store(0x4c, t.scratch_addr(s) + 0x2000))
            yield Case(f'{label}/arg{arg:02x}-mask{mask:02x}', finish(t, body),
                       dict(start_value=0x07b5, flags=DISCOVERY, timeout_us=20000, scratch=memory(), scratch_read=0x2010),
                       dict(arg=hex(arg), mask=hex(mask), chip=label, layout='0x2000: 04c after the WAIT'), pins=True)


CMD_TIMING_LOOPS = 128


def cmd_timing(s, t):
    """FMC_CTRL0 timing fields vs the duration of 128 command, address or status-read cycles (case elapsed time)."""
    _LABEL[0] = 0
    g = Q.geometry(s)
    base = 0x1077801 | 2 << g.ce
    ops = {
        'command': Q.command(0x70),
        'address': Q.address(g.row + 120),
        'status': Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(5) + Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000),
        'none': '',
    }
    for op, inner in ops.items():
        for shift, label in NAND_TIMING_FIELDS:
            for value in ((7,) if op == 'none' else (0, 3, 7)):
                if op == 'none' and shift != 12:
                    continue
                word = (base & ~(7 << shift)) | (value << shift)
                _LABEL[0] += 1
                body = (Q.setup(g.ce) + Q.wr(0, word) + '    movi r5,#%d\nct%d:\n' % (CMD_TIMING_LOOPS, _LABEL[0]) + inner
                        + '    sub r5,r5,#1\n    bnz r5,ct%d\n' % _LABEL[0] + Q.wr(0, base) + Q.command(0xff) + Q.ready())
                yield Case(f'{op}/{label}/{value}', finish(t, body), dict(timeout_us=100000, flags=DISCOVERY),
                           dict(op=op, field=label, value=value, ctrl0=hex(word), loops=CMD_TIMING_LOOPS,
                                measure='elapsed_us versus the none loop'), pins=True)


CTRL0_BITS = (0, 9, 10, 11, 15, 19, 20, 21, 22, 23, 25, 26, 27, 31)


def ctrl0_bits(s, t):
    """Single FMC_CTRL0 bits toggled for a 512-byte read of a programmed page: duration (048 samples) and data."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    from .m2_experiments import pattern
    mem = memory()
    mem[:4096] = pattern()
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, bytes(10))), dict(mutate, scratch=mem, scratch_read=0),
               dict(data='pattern(), encoded'), pins=True)
    base = 0x1077801 | 2 << g.ce
    for bit in (None,) + CTRL0_BITS:
        word = base if bit is None else base ^ (1 << bit)
        body = (Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
                + Q.wr(0, word) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + '    movi r7,#0x%x\n' % (a + 0x2000)
                + Q.wr(4, 0xe2) + sampler(0x048, 160, 0, NAND_TIMING_SPACING) + Q.wait(3) + Q.wr(0x48, 8)
                + Q.dma(a + 0x3000, 0, into=False) + Q.wr(0, base))
        yield Case('base' if bit is None else f'bit{bit:02d}', finish(t, body),
                   dict(scratch=memory(), scratch_read=0x3200, timeout_us=100000, flags=DISCOVERY),
                   dict(bit=bit, ctrl0=hex(word), layout='0x2000 + 4k: 048 sample k (≈ 1.27 us); 0x3000: first 512 raw bytes'),
                   pins=True)


TIMELINE2_PAGES = (
    dict(strength=8, errors=(0, 0, 1, 9, 16, 8, 0, 4), meta_errors=0, parity_errors=()),
    dict(strength=16, errors=(0, 1, 16, 17, 8, 0, 4), meta_errors=0, parity_errors=()),
)


def decode_timeline2(s, t):
    """810 sampled during BCH8/BCH16 decodes of clean, correctable and uncorrectable sectors."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    mutate = dict(flags=MUTATE, timeout_us=300000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    for page, spec in enumerate(TIMELINE2_PAGES):
        raw, clean = coded_page(spec['errors'], spec['strength'])
        mem = memory()
        mem[:len(raw)] = raw
        yield Case(f'program{page}', finish(t, Q.write_page(g, g.row + page, a, BCH_META, raw=True)),
                   dict(mutate, scratch=mem, scratch_read=0), dict(spec, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    for page, spec in enumerate(TIMELINE2_PAGES):
        plen, kick = (13, 1) if spec['strength'] == 8 else (26, 5)
        body = Q.wr(0x81c, 1) + Q.setup(g.ce) + Q.command(0) + Q.address(g.row + page) + Q.command(0x30) + Q.ready() + Q.command(0)
        for sector in range(len(spec['errors'])):
            slot, metalen = sector & 1, (10 if sector == 0 else 0)
            body += Q.transfer(plen + metalen, slot, parity=True) + Q.transfer(512, slot)
            body += Q.wr(0x810, 0x1ff) + Q.wr(0x1c, slot) + '    movi r7,#0x%x\n' % (a + 0x2000 + 0x100 * sector)
            body += Q.wr(0x804, kick | (metalen << 8)) + sampler(0x810, 64, 0) + Q.wait(4) + Q.wr(0x810, 0x1ff)
            body += Q.dma(a + 0x3000 + sector * 512, slot, into=False)
        yield Case(f'decode{page}', finish(t, body), dict(scratch=memory(), scratch_read=0x4000, timeout_us=300000),
                   dict(spec, samples=64, sample_ns=188.0,
                        layout='0x2000 + 0x100s + 4k: 810 sample k of sector s; 0x3000 data'), pins=True)


STATUS_POLL_SPACING = 660  # ≈ 20 us per 04c sample, 160 samples ≈ 3.2 ms (erase ≈ 2.27 ms)


def status_poll_erase(s, t):
    """04c sampled through an erase after one status latch; WAIT12 operands that the busy status 0x80 satisfies."""
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    erase_start = Q.setup(g.ce) + Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0)
    latch = Q.command(0x70) + Q.wr(4, 0xca)
    tail = Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000) + Q.ready()
    variants = {
        'erase/latch/sample04c': erase_start + latch + '    movi r7,#0x%x\n' % (a + 0x2000)
                                 + sampler(0x04c, 160, 0, STATUS_POLL_SPACING) + Q.wait(12, 0x40, 0x40),
        'erase/no-latch/sample04c': erase_start + Q.command(0x70) + '    movi r7,#0x%x\n' % (a + 0x2000)
                                    + sampler(0x04c, 160, 0, STATUS_POLL_SPACING) + Q.wr(4, 0xca) + Q.wait(12, 0x40, 0x40),
        'erase/latch/wait12-80-80': erase_start + latch + Q.wait(12, 0x80, 0x80),
        'erase/latch/wait12-20-20': erase_start + latch + Q.wait(12, 0x20, 0x20),
        'erase/latch/wait12-01-01': erase_start + latch + Q.wait(12, 0x01, 0x01),
        'erase/latch/wait5-then-wait12-80': erase_start + latch + Q.wait(5) + Q.wait(12, 0x80, 0x80),
    }
    for name, body in variants.items():
        body = ('    movi r2,#0x77770001\n' + body + '    movi r2,#0x77770002\n' + Q.store(0x4c, a + 0x2400)
                + Q.store(0x48, a + 0x2404) + tail)
        yield Case(name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2410, start_value=0x40b5, flags=DISCOVERY | MUTATE, timeout_us=100000),
                   dict(variant=name, layout='0x2000 + 4k: 04c sample k (≈ 20 us); 0x2400: 04c, 048 after the WAIT',
                        reference='erase ≈ 2.27 ms (p30); fault 32.5 ms'), pins=True)


LOC2_PAGES = (
    dict(strength=8, errors=(1, 0), meta_errors=1, parity_errors=(3, 2)),
    dict(strength=16, errors=(2, 0), meta_errors=2, parity_errors=(3, 5)),
)


def ecc_locations2(s, t):
    """Location numbering of parity errors in sector 0 (after the metadata), BCH8 and BCH16."""
    import hashlib
    _LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    mutate = dict(flags=MUTATE, timeout_us=300000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), dict(block=g.block), pins=True)
    for page, spec in enumerate(LOC2_PAGES):
        raw, clean = coded_page(spec['errors'], spec['strength'], spec['meta_errors'], spec['parity_errors'])
        mem = memory()
        mem[:len(raw)] = raw
        yield Case(f'program{page}', finish(t, Q.write_page(g, g.row + page, a, BCH_META, raw=True)),
                   dict(mutate, scratch=mem, scratch_read=0), dict(spec, raw_sha256=hashlib.sha256(raw).hexdigest()), pins=True)
    cap = a + 0x3100
    hook = lambda sec, slot: ''.join(Q.store(off, cap + LOC_STRIDE * sec + 4 * i)
                                     for i, off in enumerate((0x810, 0x818) + tuple(range(0x820, 0x860, 4))))
    for page, spec in enumerate(LOC2_PAGES):
        yield Case(f'decode{page}', finish(t, Q.wr(0x81c, 1) + decode_loop(g, g.row + page, a + 0x2040, 2, spec['strength'], hook)),
                   dict(scratch=memory(), scratch_read=0x3400, timeout_us=300000),
                   dict(spec, layout='0x2040 data; 0x3100 + 0x50s: 810, 818, 0x820..0x85c after decode s'), pins=True)


def bank_enable(s, t):
    """+400 storage words with each unassigned FMC_CTRL0 bit set: does any bit make the second set hold values?"""
    a = t.scratch_addr(s)
    base = 0x1077801 | 2
    for bit in (None,) + CTRL0_BITS[1:]:
        word = base if bit is None else base | (1 << bit)
        body = (Q.setup() + Q.wr(0, word) + Q.wr(0x40c, 0x12345678) + Q.wr(0x410, 0x9abcdef0) + Q.wr(0x434, 0x0f0f0f0f)
                + ''.join(Q.store(off, a + 0x2000 + 4 * i) for i, off in enumerate((0x40c, 0x410, 0x434, 0x400, 0x00c, 0x010, 0x034, 0x000)))
                + Q.wr(0, 0))
        yield Case('base' if bit is None else f'bit{bit:02d}', finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2020, timeout_us=20000, flags=DISCOVERY),
                   dict(bit=bit, ctrl0=hex(word), layout='0x2000: 40c 410 434 400 00c 010 034 000'))


RB_LOOPS = 16


def rb_timing(s, t):
    """FMC_CTRL0 30:28 (and 14:12 as reference) vs 16 full page-open cycles (00h, address, 30h, status ready)."""
    _LABEL[0] = 0
    g = Q.geometry(s)
    base = 0x1077801 | 2 << g.ce
    for shift, label, values in ((28, 'f30_28', (0, 3, 7)), (12, 'f14_12', (0, 7))):
        for value in values:
            word = (base & ~(7 << shift)) | (value << shift)
            _LABEL[0] += 1
            body = (Q.setup(g.ce) + Q.wr(0, word) + '    movi r5,#%d\nrb%d:\n' % (RB_LOOPS, _LABEL[0])
                    + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready()
                    + '    sub r5,r5,#1\n    bnz r5,rb%d\n' % _LABEL[0] + Q.wr(0, base))
            yield Case(f'{label}/{value}', finish(t, body), dict(timeout_us=100000, flags=DISCOVERY),
                       dict(field=label, value=value, ctrl0=hex(word), loops=RB_LOOPS, measure='elapsed_us'), pins=True)


def wait2_soft(s, t):
    """wait2 N (opcode 09): no raise (control), VIC line N raised by a CPU soft interrupt at 1 ms and held
    to 3 ms, or a 1 us pulse."""
    from .protocol import VICSOFT, vic_param
    for mode, clear_us in (('control', None), ('held', 3000), ('pulse', 1001)):
        for n in range(64):
            vic, bit = n >> 5, 1 << (n & 31)
            body = Q.setup() + '    movi r2,#0x77770001\n    .insn 0x9,%d,0,0\n    movi r2,#0x77770002\n' % n
            raise_ = {} if clear_us is None else dict(late=[(vic_param(vic, 0x18), bit)], late_us=1000,
                                                      late2=[(vic_param(vic, 0x1c), bit)], late2_us=clear_us)
            yield Case(f'{mode}/line{n:02d}', finish(t, body),
                       dict(start_value=0x10b5, flags=DISCOVERY | VICSOFT, timeout_us=20000, **raise_),
                       dict(line=n, vic=vic, bit=hex(bit), raise_us=1000, clear_us=clear_us,
                            note='lines the CPU has enabled are not raised (masked by the probe); vic*_seen shows raw',
                            reference='p16 w2vic: 5, 8, 16 pass in all states, 19 in DMA/ECC; fault at ≈ 8.8 ms'))


WAIT2_FMC_STATES = ('DMA', 'ECC', 'command', 'status')


def wait2_fmc(s, t):
    """Which interrupt line the NAND controller raises: VIC raw status after DMA, ECC, command and status
    operations with FMC 0x044 enables 0 / all, and wait2 0..63 after each operation with all enables."""
    from .discovery import states
    _LABEL[0] = 0
    actions = states(s, t)
    for state in WAIT2_FMC_STATES:
        for enable in (0, 0xffffffff):
            body = Q.setup() + Q.wr(0x44, enable) + actions[state] + delay(30000) + Q.wr(0x44, 0)
            yield Case(f'observe/{state}/en{enable:x}', finish(t, body), dict(flags=DISCOVERY, timeout_us=20000),
                       dict(state=state, enable=hex(enable), measure='vic0_seen / vic1_seen during ≈ 2 ms'))
    for state in WAIT2_FMC_STATES:
        for n in range(64):
            body = (Q.setup() + Q.wr(0x44, 0xffffffff) + actions[state] + '    movi r2,#0x77770001\n'
                    + '    .insn 0x9,%d,0,0\n    movi r2,#0x77770002\n' % n + Q.wr(0x44, 0))
            yield Case(f'wait2/{state}/line{n:02d}', finish(t, body),
                       dict(start_value=0x07b5, flags=DISCOVERY, timeout_us=20000),
                       dict(state=state, line=n, enable='0xffffffff', reference='fault ≈ 4.4 ms'))


def fmc_irq_bits(s, t):
    """FMC 0x044 single enable bits after a DMA, a command and a status latch: which bit raises VIC0 line 20."""
    from .discovery import states
    _LABEL[0] = 0
    actions = states(s, t)
    for state in ('DMA', 'command', 'status'):
        for bit in range(32):
            body = Q.setup() + Q.wr(0x44, 1 << bit) + actions[state] + delay(3000) + Q.store(0x48, t.scratch_addr(s) + 0x2000) + Q.wr(0x44, 0)
            yield Case(f'{state}/bit{bit:02d}', finish(t, body),
                       dict(flags=DISCOVERY, timeout_us=20000, scratch=memory(), scratch_read=0x2010),
                       dict(state=state, enable=hex(1 << bit), measure='vic0_seen bit 20; 0x2000: 048 after the operation'))


ECC_IRQ_CANDIDATES = (0x808, 0x814, 0x844, 0x860, 0x864, 0x868)


def ecc_irq(s, t):
    """ECC interrupt enable candidates (M1: 0x844, source 43): all-ones written before an encode;
    VIC raw status, wait2 43 and wait2 20 after it."""
    _LABEL[0] = 0
    a = t.scratch_addr(s)
    for off in (None,) + ECC_IRQ_CANDIDATES:
        pre = Q.setup() + Q.dma(a, 0) + ('' if off is None else Q.wr(off, 0xffffffff))
        restore = '' if off is None else Q.wr(off, 0x700 if off == 0x808 else 0)
        label = 'none' if off is None else f'{off:03x}'
        yield Case(f'observe/{label}', finish(t, pre + Q.ecc(acknowledge=False) + delay(3000) + restore),
                   dict(flags=DISCOVERY, timeout_us=20000), dict(candidate=label, measure='vic0_seen / vic1_seen'))
        for n in (20, 43):
            body = pre + Q.ecc(acknowledge=False) + '    movi r2,#0x77770001\n    .insn 0x9,%d,0,0\n    movi r2,#0x77770002\n' % n + restore
            yield Case(f'wait2-{n}/{label}', finish(t, body), dict(start_value=0x07b5, flags=DISCOVERY, timeout_us=20000),
                       dict(candidate=label, line=n))


def recovery(s, t):
    """One resident sequence: pause/capture/cleanup, control, explicit CPU resume."""
    import struct
    marker = 'movi r2,#0x7265636f\nend\n'
    control = {'mode': 'confirm', 'expected': {'why': 'END', 'r': {'2': 0x7265636f}}}
    yield Case('baseline', marker, conditions=control)
    for label, instruction in (('zero', '.insn 0x12,0,0,0'),
                               ('p06', '.insn 0x12,2,0xd60,0xffffffff')):
        source = (t.prologue(c58=0) + 'movi r6,#0\n' + instruction +
                  '\nmovi r6,#0x600d600d\nend\n' + t.trap())
        code = P.assemble(t, source)
        next_pc = next(i + 8 for i in range(0, len(code), 8)
                       if struct.unpack_from('<I', code, i)[0] >> 24 == 0x12)
        yield Case(label + '/pause-cleanup', source, {'timeout_us': 2000},
                   {'mode': 'confirm', 'expected': {'cs_irq': 8, 'r': {'6': 0}, 'after': {'0xc08': 10}},
                    'question': 'pause captured before cleanup; next control must execute'})
        yield Case(label + '/after-cleanup', marker, conditions=control)
        yield Case(label + '/cpu-resume', source,
                   {'timeout_us': 5000, 'late_us': 100,
                    'late': [(0xc38, next_pc), (0xc0c, 8), (0xc00, 0x4ffb5)]},
                   {'mode': 'confirm', 'expected': {'why': 'END', 'r': {'6': 0x600d600d}},
                    'question': 'original PC/ack/resume sequence; r6 marker and END expected'})
        yield Case(label + '/after-resume', marker, conditions=control)


def resetcheck(s, t):
    """P13 start=1 failure and an independent END after every cleanup."""
    marker = 'movi r2,#0x72657365\nend\n'
    expected = {'mode': 'confirm', 'expected': {'why': 'END', 'r': {'2': 0x72657365}}}
    yield Case('baseline', marker, conditions=expected)
    for label, body in (('wait3', Q.wait(3)), ('illegal1a', '.insn 0x1a,0,0,0\n'),
                        ('evjmp16', '.insn 0x16,0,0,0\n')):
        yield Case(label + '/start1', finish(t, body),
                   {'start_value': 1, 'timeout_us': 2000, 'flags': DISCOVERY},
                   {'question': 'capture timeout-disabled state, then reset; following control must execute',
                    'basis': 'p13 ctrlfx first three programs; shorter observation window'})
        yield Case(label + '/after-reset', marker, conditions=expected)


def status_recovery(s, t):
    """p07 latch sequence and read-order controls; never assume a new outcome."""
    a, g = t.scratch_addr(s), Q.geometry(s)
    marker = 'movi r2,#0x7265636f\nend\n'
    control = {'mode': 'confirm', 'expected': {'why': 'END', 'r': {'2': 0x7265636f}}}
    yield Case('baseline', marker, conditions=control)
    for order in ('before-wait', 'after-wait', 'no-status-read'):
        body = Q.setup(g.ce) + Q.wr(0x44, 0)
        for i in range(2):
            body += Q.command(0x70) + Q.wr(4, 0xca)
            status = Q.store(0x48, a + i * 16) + Q.store(0x4c, a + i * 16 + 4)
            body += (status if order == 'before-wait' else '') + Q.wait(5)
            body += status if order == 'after-wait' else ''
        body += Q.wr(0x44, 0)
        # First variant is the exact p07 ev5 program, including both latches.
        yield Case(order, finish(t, body), {'scratch': memory(), 'scratch_read': 64,
                   'timeout_us': 160000}, {'question': 'status read ordering and reset recovery',
                   'status_read_order': order}, pins=True)
        yield Case(order + '/after', marker, conditions=control)


SPECS = {
    'm2_resetcheck': (resetcheck, 'ctrlfx', 'Start=1 WAIT/illegal/evjmp cleanup, independent END after each reset'),
    'm2_status_recovery': (status_recovery, 'ev5', 'Status-latch failure, read-order controls and following execution'),
    'm2_recovery': (recovery, 'op12,startseq', 'Pause cleanup, subsequent execution and explicit CPU resume controls'),
    'm2_shifts': (shifts, 'alu', 'Full-width shift counts and immediate-zero encoding'),
    'm2_pram_alias': (pram_alias, 'pram,indirect', 'Parameter boundary, indirect high-bit alias and alignment'),
    'm2_c58_bits': (c58_bits, 'c58', 'CPU versus sequencer single-bit write/read masks'),
    'm2_waitcons': (wait_consumption, 'evcons', 'Repeated/ordered waits, acknowledge and rearm within one program'),
    'm2_status_order': (status_order, 'ev5t', 'Command acknowledgement and delay versus status-latch ordering'),
    'm2_ce_conditions': (ce_conditions, 'op09c', 'No CE and each CE selection with WAIT/opcode09 operand sweeps'),
    'm2_delayed_vector': (delayed_vector, 'op16b', 'Delayed CPU stimuli and alternate vector destinations'),
    'm2_nested_fault': (nested_fault, 'ctrlwords,c50x', 'Fault offsets and a second fault inside the handler'),
    'm2_pause_state': (pause_state, 'c08self', 'State across raw signal/pause/event opcodes and CPU resume candidates'),
    'm2_ecc_sequence': (ecc_sequence, 'seqrestart', 'Three encodes per program across reset/ack/slot changes, BCH8 and BCH16'),
    'm2_fifo_access': (fifo_access, 'bufport', 'CPU and sequencer writes to the candidate FIFO, then DMA readback'),
    'm2_dma_ecc_overlap': (dma_ecc_overlap, 'dmactl,writeseq2', 'Serial controls versus concurrent DMA/ECC on same/different slots'),
    'm2_timeout_window': (timeout_window, 'startbits,tmobits', 'Multi-bit start-word timeout counts versus the single-bit linear fit'),
    'm2_bank_regs': (bank_regs, 'chan1,chan1pins', 'Primary versus +400 storage words: independent, mirrored or inert, CPU and sequencer writers'),
    'm2_bch8_correct': (bch8_correct, 'eccstr,eccstr2,miscorr,rawwrite', 'Host BCH8 code words with 0..16 known bit errors: correction radius and reported counts'),
    'm2_ecc_status_timing': (ecc_status_timing, 'eccregs,done5,rsresult', 'BCH8 decode: when ECC_RESULT/810/818 report the corrected count and failure, four ack variants'),
    'm2_bch16_encode': (bch16_encode, 'eccpar,eccstr2', 'Engine BCH16 parity for known data (BCH8 control), recovered by raw read'),
    'm2_status_busy': (status_busy, 'ev5,latchrep,ev5ready', 'Latched status byte during erase/program/read busy, idle and no-CE'),
    'm2_ecc_ring': (ecc_ring, 'eccregs,rsresult', '0x80c result ring: length, read-pointer advance, reset; 0x810 bit-5 threshold with 1..8 errors'),
    'm2_bch16_correct': (bch16_correct, 'eccstr,eccstr2', 'BCH16 decode of host code words with 0..24 errors (seven sectors per page)'),
    'm2_ecc_threshold': (ecc_threshold, 'eccregs,done5', 'BCH8 810 bit-5 threshold (1..8 errors), erased-page status, 818 under 0x81c'),
    'm2_wait4_ack': (wait4_ack, 'evcons,done5b', 'Which 0x810 write or 0x81c re-arms WAIT4 after an encode'),
    'm2_ecc_modes': (ecc_modes, 'eccmode,eccmode2,eccmode3,b15ecc', 'ECC_CTRL encode words: metadata length and every other single bit, parity by raw read'),
    'm2_wait4_decode': (wait4_decode, 'evcons,done5b', 'Decode-side WAIT4: which 810 write re-arms it after an erased-sector decode'),
    'm2_bch16_threshold': (bch16_threshold, 'eccstr2', '810 bit-5 threshold for BCH16 (4..8, 12 errors)'),
    'm2_ecc_modes2': (ecc_modes2, 'eccmode4,b15ecc', 'ECC_CTRL bits 24..31, encode+decode words, metadata length 13/16/32'),
    'm2_opcost': (opcost, 'opcost,optime,clock', 'Per-instruction time from 1000 vs 21000-iteration loops, SDRAM and SRAM program'),
    'm2_fmc_misc': (fmc_misc, 'fmcbits', 'FMC 018 before data reads and 05c before the final DMA on a page read'),
    'm2_jump_width': (jump_width, 'branch,regjr,fwidth', 'JR and BNZ target width (bits 0..31) and alignment via landing pads'),
    'm2_wait_rb': (wait_rb, 'evsweep,ev5ready', 'WAIT 0..15 issued right after a block erase starts; status byte shows busy/ready at release'),
    'm2_jr_hazard': (jr_hazard, 'regjr,r8', 'JR target-register hazard: 0..6 nops between the target write and JR'),
    'm2_jr_width': (jr_width, 'regjr,fwidth', 'JR target width and alignment with the hazard avoided'),
    'm2_alu_hazard': (alu_hazard, 'alu,regsrc', 'Register-write hazards: movi/add/ld/ldi producers, add/mov/ldi/sti/bnz consumers at distance 0, 1, 2, 4'),
    'm2_jr_cond': (jr_cond, 'regjr,branch', 'JR as conditional jump: R field 0..7 with the condition register zero or one'),
    'm2_jr_width2': (jr_width2, 'regjr,fwidth', 'JR target width and alignment with r0 = 1'),
    'm2_wait_sources': (wait_sources, 'evsweep,evsweep2,evsweep3', 'WAIT 5..10, 12..15 after encode, decode (erased/uncorrectable), meta latch, unacknowledged DMA, status latch'),
    'm2_fmc_flags': (fmc_flags, 'fmcstat,fmcstat2,c50', 'FMC_STATUS/C50/810/04c/C08 right after each controller operation, before any ack'),
    'm2_dma_cut': (dma_cut, 'dmactl,dmaloop', 'Second DMA started 0..256 nops after the first: bytes delivered (DMA rate) and 038'),
    'm2_timeout_low': (timeout_low, 'startbits,tmobits2', 'Start-word bits 7:4 with count 0x07: WAIT3 fault time'),
    'm2_ecc_regs': (ecc_regs, 'eccregs,eccpar', 'Sequencer view of the ECC block 0x800..0x9fc after known encodes and decodes'),
    'm2_timeline': (timeline, 'dmactl,eccregs,fmcstat', 'Sampled register timelines (≈ 188 ns) during DMA, ECC, NAND reads, status latch, command, address'),
    'm2_wait_cpu': (wait_cpu, 'evsweep3,c58', 'WAIT 6..10, 12..15 with one CPU-written control word 1 ms into the wait'),
    'm2_status_wait': (status_wait, 'ev5,ev5ready,latchrep', 'WAIT5 and masked WAIT12 against a busy chip; WAIT5 latch orderings'),
    'm2_ecc_decode_regs': (ecc_decode_regs, 'eccregs', '0x820..0x85c after decodes with 1..8 errors'),
    'm2_reset_retention': (reset_retention, 'restore,ctrlfx', 'Storage words written in one case, read in the next before-map'),
    'm2_decode_timeline': (decode_timeline, 'eccregs,optime', '810 sampled during decodes with 1..8 errors: correction time vs error count'),
    'm2_nand_timing': (nand_timing, 'fmcbits,progtime', 'FMC_CTRL0 timing fields 14:12, 18:16, 30:28 vs 512-byte read duration and data'),
    'm2_fifo_read': (fifo_read, 'bufport,readid', 'READ ID bytes through FIFO latch 0x52 and ports 0x60..0x68 / 0x080'),
    'm2_wait_cpu2': (wait_cpu2, 'evsweep3', 'WAIT 6..10, 12..15 with CPU writes to C64/C68/C6C, restored (run last)'),
    'm2_c60_hold': (c60_hold, 'startseq', 'C60 written by the CPU while running: restore timing and cleanup (run last)'),
    'm2_nand_write_timing': (nand_write_timing, 'fmcbits,writeseq', 'FMC_CTRL0 timing fields vs 512-byte data-in duration (program aborted with FFh)'),
    'm2_ecc_locations': (ecc_locations, 'eccregs,eccstr2', '0x820..0x85c after BCH8/BCH16 decodes: metadata, parity, uncorrectable, clean sectors'),
    'm2_wait12_args': (wait12_args, 'ev5ready,status', 'WAIT12 arg/mask pairs against ready status 0xe0 and the no-CE echo 0x70'),
    'm2_cmd_timing': (cmd_timing, 'fmcbits,optime', 'FMC_CTRL0 timing fields vs 128 command, address and status cycles'),
    'm2_ctrl0_bits': (ctrl0_bits, 'fmcbits', 'Single FMC_CTRL0 bits toggled for a page read: duration and data'),
    'm2_decode_timeline2': (decode_timeline2, 'eccregs,optime', '810 timelines for clean, correctable and uncorrectable BCH8/BCH16 decodes'),
    'm2_status_poll': (status_poll_erase, 'ev5ready,status', '04c through an erase after one latch; WAIT12 operands satisfied by busy status'),
    'm2_ecc_locations2': (ecc_locations2, 'eccregs,eccstr2', 'Location numbering of sector-0 parity errors after metadata, BCH8/BCH16'),
    'm2_bank_enable': (bank_enable, 'bank,fmcbits', '+400 storage words with each unassigned FMC_CTRL0 bit set'),
    'm2_rb_timing': (rb_timing, 'fmcbits,progtime', 'FMC_CTRL0 30:28 vs 16 page-open cycles including ready'),
    'm2_wait2_soft': (wait2_soft, 'w2vic,irq', 'wait2 N with VIC line N raised by a CPU soft interrupt (held, 1 us pulse); needs the VICSOFT kernel'),
    'm2_wait2_fmc': (wait2_fmc, 'w2vic,irq', 'NAND controller interrupt line: VIC raw after operations with 0x044 enables, wait2 0..63'),
    'm2_fmc_irq_bits': (fmc_irq_bits, 'irq044,irq', 'FMC 0x044 single enable bits vs VIC0 line 20 after DMA, command, status latch'),
    'm2_ecc_irq': (ecc_irq, 'irq,eccregs', 'ECC interrupt enable candidates 0x808/0x814/0x844/0x860..0x868: VIC raw, wait2 43 and 20 after an encode'),
    'm2_dma_ack': (dma_ack, 'dmaloop,dmactl', 'p16 payload loss: exact p16 serial program versus acknowledged WAIT2, with and without ECC'),
    'm2_dma_visibility': (dma_visibility, 'dmaloop,dmactl', 'P16 partial payloads: plain DMA/BCH8/BCH16, CPU-timed holds and pre-cleanup checksum'),
}


from .opcode_experiments import SPECS as OPCODE_SPECS
SPECS.update(OPCODE_SPECS)
from .wedge_experiments import wedge_cases
SPECS['m2_wedge'] = (wedge_cases, 'regnum', 'Register/PC/load transitions; per-case collection only with --isolate-diagnostics')
from .register_experiments import SPECS as REGISTER_SPECS
SPECS.update(REGISTER_SPECS)
from .m1diff_isa import SPECS as M1DIFF_ISA_SPECS
from .m1diff_fmc import SPECS as M1DIFF_FMC_SPECS
from .m1diff_ecc import SPECS as M1DIFF_ECC_SPECS
from .open_experiments import SPECS as OPEN_SPECS
from .cold_experiments import SPECS as COLD_SPECS
from .m1diff_followup import SPECS as M1DIFF_FOLLOWUP_SPECS
from .round3_experiments import SPECS as ROUND3_SPECS
from .round4_experiments import SPECS as ROUND4_SPECS
from .round5_experiments import SPECS as ROUND5_SPECS
from .round6_experiments import SPECS as ROUND6_SPECS
for _specs in (M1DIFF_ISA_SPECS, M1DIFF_FMC_SPECS, M1DIFF_ECC_SPECS, OPEN_SPECS, COLD_SPECS, M1DIFF_FOLLOWUP_SPECS, ROUND3_SPECS, ROUND4_SPECS, ROUND5_SPECS, ROUND6_SPECS):
    SPECS.update(_specs)


def cases(name, s, t):
    yield from SPECS[name][0](s, t)


def execute(case, s, t):
    if case.pins:
        st = s.status()
        if not st.step_ok_mask & 2:
            Q.check(s.job(Q.Cmd.NAND_STEP, 1, 0))
        if not getattr(s, 'm2_pins_ready', False):
            Q.check(s.job(Q.Cmd.NAND_STEP, 2, 0))
            s.m2_pins_ready = True
    rec = P.probe(s, t, P.assemble(t, case.source), **case.options)
    return annotate(case, s, rec)


def annotate(case, s, rec):
    rec['case_conditions'] = case.conditions
    from .opcode_experiments import assess
    assessment = assess(case, rec)
    if assessment is not None:
        rec['assessment'] = assessment
        if hasattr(s, 'log'):
            s.log({'type': 'opcode-assessment', 'case': case.name, 'data': assessment})
    return rec


def run(name, s, t):
    if name in ('m2_r6_pc', 'm2_r6_vic1', 'm2_r6_gate5', 'm2_r6_gate_checkpoint'):
        from .round6_experiments import run_collected
        return run_collected(s, t, name, cases(name, s, t))
    return run_cases(name, s, t, lambda view, target: cases(name, view, target))


def isolates_cases(s, name):
    """Only an explicit host option splits diagnostic families into case jobs."""
    return (getattr(s, 'batch_enabled', True)
            and getattr(s, 'isolate_diagnostics', False)
            and name in ('m2_wedge', 'm2_regbus'))


def run_cases(name, s, t, generate):
    """Execute an explicit case generator with either resident or single transport."""
    from .protocol import BATCH
    if getattr(s, 'probe_caps', 0) & BATCH and getattr(s, 'batch_enabled', True):
        from .batch import run as run_batch
        # Cache status during generation: all sources share the stable probe
        # scratch allocation. No per-case USB reads just to obtain its address.
        from types import SimpleNamespace
        cached = s.status()
        view = SimpleNamespace(status=lambda: cached, geometry=Q.geometry(s))
        if isolates_cases(s, name):
            from .wedge_experiments import run_isolated
            return run_isolated(s, t, name, generate(view, t))
        return run_batch(s, t, name, generate(view, t))
    records, errors = {}, 0
    for case in generate(s, t):
        P.set_experiment(name + '/' + case.name)
        print('PLAN: case ' + name + '/' + case.name, flush=True)
        try:
            records[case.name] = execute(case, s, t)
        except Exception as exc:
            errors += 1
            rec = {'case_error': type(exc).__name__, 'message': str(exc), 'conditions': case.conditions}
            records[case.name] = rec
            if hasattr(s, 'log'): s.log({'type': 'case-error', 'experiment': name, 'case': case.name, **rec})
            print('RECORDED CASE ERROR: %s; continuing' % exc, flush=True)
    return records, summarize(name, records)


def summarize(name, records):
    errors = sum('case_error' in r for r in records.values())
    from collections import Counter
    checks = Counter(r['assessment']['status'] for r in records.values() if 'assessment' in r)
    summary = '%s: %d cases attempted, %d case errors (raw outcomes retained)' % (name, len(records), errors)
    if checks: summary += '; opcode checks ' + str(dict(checks))
    failed = sum('execution_error' in r for r in records.values())
    if failed: summary += '; %d execution/setup/cleanup failures' % failed
    from .opcode_experiments import candidate_summary
    candidates = candidate_summary(records)
    if candidates: summary += '\nCandidate signatures (not decoded meanings): ' + candidates
    return summary


def entries():
    for name, (_, original, question) in SPECS.items():
        yield {'name': name, 'status': 'followup', 'reason': question, 'question': question,
               'original': original, 'fn': lambda s, t, n=name: run(n, s, t),
               'source': 'nano4g/followup_experiments.py', 'resident': True}
