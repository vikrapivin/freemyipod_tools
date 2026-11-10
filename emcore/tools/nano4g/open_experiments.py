"""Experiments for the open points of the M2 book's chapter 17 that the M1 comparison
does not already cover. One family per open point.

Each case carries conditions `open_point` (chapter 17 text), `ch17` (table number) and
`decision` (how the outcome settles the point). Outcomes are discovery records: no M2
answer is assumed. Construction facts (settled on M2): start word count<<8|n<<4|5,
fault ~= 492.747 us * (count + 1) + 447.5 us for n = 0xb; C58 = 4 vectors a fault to
C4C, C58 = 0 continues; CS_IRQ is 0xc0c; C00: 8 STOP, 2 abort, 0x80000 clearing reset;
the probe captures the after-map (and FULLMAP) before its cleanup reset.

COVERED lists chapter-17 points already measured by existing families; NOT_MEASURABLE
gives the reason for points this harness cannot measure.
"""
from . import followup_experiments as F, sequences as Q  # noqa: F401
from .protocol import DISCOVERY, MUTATE, VICSOFT, vic_param
from fmss import probe as P  # noqa: F401

Case, finish, memory, delay, sampler, NOP = F.Case, F.finish, F.memory, F.delay, F.sampler, F.NOP
MASK = 0xffffffff
SHORT = 0x07b5                     # unmet WAIT faults at ~4.39 ms
MARK1, MARK2 = '    movi r2,#0x77770001\n', '    movi r2,#0x77770002\n'
SNAP_SKIP_800 = P.SNAP_SKIP[0x800]
HOLD = range(0xc60, 0xc70, 4)

POINTS = {
    1: 'extended destination/store roles, combinations, high source-field bits, non-zero reserved fields',
    2: 'exact PC width and the low-address bus map',
    3: 'opcode legality in operand/pending states; nop2 (ff); erased (all-ones) program memory',
    4: 'sig (10) on M2; tmwait (15) reload/decrement rule and operand forms; nop (08) operand forms',
    6: '0.25-cycle fractions of nop, branches and sti; the timeout prescaler phase',
    8: 'bit maps of C00, C3C, C5C, C14; complete C08 state-code set; C58 beyond the tested bits',
    9: 'sources of WAIT 6..10 and 13..15',
    10: 'source of WAIT11',
    11: 'the two WAIT5 faults seen after earlier programs; command/latch arm rule',
    12: 'evjmp with other operands and individual/multiple sources; other reset orderings',
    14: 'candidate +400 register set: absent or not enabled (other enable paths)',
    15: 'length rule for the cut point of a superseded DMA',
    16: 'FMC_DMA_CTRL (038) bits 6..12',
    17: '048 bit 25 and dropped bursts when DMA writes a buffer under its own ECC operation',
    18: 'roles of 018, 058, 05c; FMC_CTRL0 bits 30:28',
    19: 'bit map of the 004 operation words; access masks of 048 and 04c',
    21: 'ECC slot selector 01c beyond 0/1 and slot arbitration',
    22: 'unassigned bits of 810 and 80c; effect of 81c; CPU reads of 80c',
    25: 'which reset stage clears each storage word',
}

COVERED = {
    5: 'm2_shifts, m2_m1_shift_counts (counts 0..255, 2^8..2^31, wrap probes, both forms)',
    7: 'm2_wedge (per-case collection with --isolate-diagnostics); see NOT_MEASURABLE',
    13: 'm2_wait2_soft, m2_wait2_fmc, m2_fmc_irq_bits, m2_m1_wait2_vic1, m2_m1_ecc_irq_line, m2_m1_irq044_routing',
    20: 'm2_m1_ecc_alt_code (bit pairs, all bits, metadata length 0x80/0xff), m2_ecc_modes, m2_ecc_modes2',
    23: 'see NOT_MEASURABLE',
    24: 'm2_m1_poweron_values (first capture after a power cycle, FMSS untouched)',
}

NOT_MEASURABLE = {
    'ch17 #2 PC width above 2^20': (
        'A jump to 2^k (k >= 21) that does not wrap executes SDRAM beyond the probe result buffer '
        '(program at +0x400, scratch +0x38000..+0x40000), whose content the harness neither writes nor reads; '
        'a non-wrapping PC would run arbitrary words (possibly FMC command or DMA writes) with no bound. '
        'Only the wrapping outcome is safe, so the test cannot be decided. The low-address bus map is '
        'measured by m2_open_low_bus.'),
    'ch17 #7 final instruction of the regnum USB-loss case': (
        'The stall stops the worker and USB; nothing survives the reset that follows, and the probe only '
        'reports over USB after the run. The candidate programs are replayed by m2_wedge one case per job, '
        'which bounds but cannot name the instruction executing at the moment of the loss.'),
    'ch17 #13 device behind VIC0 line 5': (
        'Identifying it needs the CPU to gate or trigger peripherals during a run; the probe interface only '
        'allows CPU writes to FMSS offsets and to the VIC soft-interrupt registers, and line 5 is always '
        'raised (wait2 5 passes in every state).'),
    'ch17 #23 original WRITE/ERASE contracts': (
        'The harness runs only host-assembled programs and never contains Apple microcode bytes; the '
        'original M2 WRITE/ERASE programs and their CPU-side parameter blocks cannot be run as such. Their '
        'register sequences are already exercised by sequences.write_page/erase on the scratch block.'),
}


def cond(point, decision, **extra):
    return dict(open_point=POINTS[point], ch17='17-open-questions.md table row %d' % point, decision=decision, **extra)


def insn(op, reg=0, arg=0, imm=0):
    value = imm if isinstance(imm, str) else '0x%x' % (imm & MASK)
    return '    .insn 0x%x,%d,0x%x,%s\n' % (op, reg, arg, value)


def label(prefix):
    F._LABEL[0] += 1
    return '%s%d' % (prefix, F._LABEL[0])


def seeds():
    return [(0xc18 + 4 * i, 0x11111111 * (i + 1)) for i in range(8)]


def stores(regs, base):
    return ''.join(Q.store(off, base + 4 * k) for k, off in enumerate(regs))


def ctrl0_base(g):
    return 0x1077801 | 2 << g.ce


# ---- m2_open_reserved_fields (ch17 #1)
def reserved_fields(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    base_opts = dict(params=seeds() + [(0xd70, 0), (0xd74, 0), (0xd64, 0xcafe0064), (0xd68, 0xcafe0068)],
                     scratch=memory(), scratch_read=0x400, flags=DISCOVERY | P.WAIT_END, timeout_us=20000)
    cases = []
    for R, arg, imm in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (0x80, 0, 0), (0, 0x8000, 0), (0, 0, 0x80000000),
                        (0xff, 0xffff, MASK)):
        cases.append(('end/R%x-a%x-i%x' % (R, arg, imm), insn(0, R, arg, imm),
                      'END honoured = r6 keeps its seed 0x77777777; r6 0x600d600d = the fields made it continue'))
    for R in (1, 0x80, 0xff):
        cases.append(('wr/R%x' % R, insn(1, R, 0xd70, 0x5a5a0001), 'd70 = 0x5a5a0001 = R ignored'))
    for imm in (1, 0x80000000, MASK):
        cases.append(('st/i%x' % imm, insn(2, 3, 0xd70, imm), 'd70 = r3 seed 0x44444444 = imm ignored'))
    for imm in (1, 0x100, MASK):
        cases.append(('ldi/i%x' % imm, '    movi r5,#0x%x\n' % (a + 0x100) + insn(3, 2, 5, imm) + '    st [0xd70],r2\n',
                      'd70 = scratch word 0x100 (%s) = imm ignored; another scratch word = imm offsets' % F.F_word(0x100)))
    for arg in (1, 0x0a, 0x8000, 0xffff):
        cases.append(('movi/a%x' % arg, insn(5, 2, arg, 0x1234), 'r2 = 0x1234 = arg ignored'))
    for imm in (1, 0x10, MASK):
        cases.append(('mov/i%x' % imm, insn(6, 2, 3, imm), 'r2 = r3 seed = imm ignored'))
    for op, reg in ((0xe, 1), (0x17, 0)):
        for arg in (1, 0xff, 0xffff):
            taken = label('tk')
            cases.append(('%s/a%x' % ('bnz' if op == 0xe else 'bz', arg),
                          ('    movi r0,#0\n' if op == 0x17 else '') + insn(op, reg, arg, taken)
                          + '    movi r2,#0xbad\n%s:\n    st [0xd70],r2\n' % taken,
                          'd70 = r2 seed 0x33333333 = branch taken (arg ignored); 0xbad = not taken'))
    for imm in (1, 0x100, MASK):
        cases.append(('sti/i%x' % imm, '    movi r5,#0x%x\n' % (a + 0x100) + insn(0x11, 3, 5, imm),
                      'scratch 0x100 = 0x44444444 = imm ignored; another scratch word changed = imm offsets'))
    for imm in (1, 0x10, MASK):
        cases.append(('pld/i%x' % imm, '    movi r1,#0xd64\n' + insn(0x18, 2, 1, imm) + '    st [0xd70],r2\n',
                      'd70 = 0xcafe0064 = imm ignored'))
        cases.append(('pst/i%x' % imm, '    movi r1,#0xd64\n' + insn(0x19, 3, 1, imm),
                      'd64 = 0x44444444 and no other PRAM word changed = imm ignored'))
    for op, name in ((6, 'mov'), (0xc, 'add')):
        for arg in (0x103, 0x803, 0x8003, 0xff03):
            cases.append(('%s/src%x' % (name, arg), insn(op, 2, arg, 0),
                          'r2 = r3-based result = high source bits ignored; other value = selector extends above 8 bits'))
    cases += [
        ('combo/mov-r10-pc', insn(6, 10, 8, 0), 'every GPR = PC value: broadcast of the PC source'),
        ('combo/mov-r10-sel10', insn(6, 10, 10, 0), 'GPR values after a broadcast whose source is the destination selector'),
        ('combo/add-r10-sel10-imm5', insn(0xc, 10, 10, 5), 'GPR values: broadcast destination with selector-10 source, imm form'),
        ('combo/st-sel10-after-broadcast', insn(6, 10, 3, 0) + insn(2, 10, 0xd70, 0),
         'd70 = register-bus latch after a broadcast (M2-47 rule) or another value'),
    ]
    for name, body, expect in cases:
        yield Case(name, finish(t, body), dict(base_opts),
                   cond(1, 'compare with the zero form: %s' % expect, seeds='r0..r7 = 0x11111111*(n+1)',
                        instruction=body.strip()))


# ---- m2_open_low_bus (ch17 #2)
LOW_BUS_OFFSETS = (0x0, 0x4, 0x60, 0x100, 0x1000, 0x8000, 0xfffc)


def low_bus(s, t):
    for off in LOW_BUS_OFFSETS:
        body = ('    movi r1,#0x%x\n    ldi r2,[r1]\n    st [0xd70],r2\n' % (0x20000000 + off) +
                '    movi r1,#0x%x\n    ldi r3,[r1]\n    st [0xd74],r3\n' % off)
        yield Case('rom%04x' % off, finish(t, body),
                   dict(params=[(0xd70, 0), (0xd74, 0)], flags=DISCOVERY, timeout_us=20000),
                   cond(2, 'd74 == d70: low address mirrors the 64 KiB ROM at 0x20000000; different = another map; '
                           'a fault/hang after d70 = low address unmapped (ROM read first)',
                        rom_address=hex(0x20000000 + off), low_address=hex(off),
                        reference='address selector 8 reads e1a0f00e at 0x60 (M2-48)'))


# ---- m2_open_erased_ops (ch17 #3)
ERASED = insn(0xff, 0xff, 0xffff, MASK)


def erased_ops(s, t):
    F._LABEL[0] = 0
    from .discovery import states
    g = Q.geometry(s)
    actions = states(s, t)
    opts = dict(start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=20000)
    for k in (1, 8, 64):
        yield Case('erased%d' % k, finish(t, MARK1 + ERASED * k + MARK2), dict(opts),
                   cond(3, 'END with r6 0x600d600d and r2 0x77770002 = all-ones words execute as nop2; fault = illegal',
                        words=k))
    yield Case('nop2-zero', finish(t, MARK1 + insn(0xff) + MARK2), dict(opts),
               cond(3, 'control: zero-form nop2'))
    for state in ('command', 'address', 'DMA', 'ECC', 'status'):
        pins = state in ('command', 'address', 'status')
        body = Q.setup(g.ce if pins else None) + actions[state] + MARK1 + ERASED + MARK2 + '    ld r4,[0x48]\n'
        yield Case('pending-%s' % state, finish(t, body), dict(opts),
                   cond(3, 'pass = erased word legal with a pending %s; fault/vector = state-dependent legality' % state,
                        state=state), pins)
    for n in (1000, 21000):
        lp = label('er')
        body = '    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n    bnz r4,%s\n' % (n, lp, ERASED, lp)
        yield Case('cost/n%d' % n, finish(t, body), dict(timeout_us=200000),
                   cond(3, '(elapsed(21000) - elapsed(1000)) / 20000 minus the m2_opcost empty loop = cost of an '
                           'erased word; equal to nop2ff = same instruction', iterations=n))


# ---- m2_open_operand_forms (ch17 #4)
def operand_forms(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for R, arg, imm in ((0, 0, 0), (1, 0, 0), (0xff, 0, 0), (0, 0xffff, 0), (0, 0, 1), (0, 0, 0x80000000),
                        (0xff, 0xffff, MASK)):
        body = insn(0x10, R, arg, imm) + delay(3000) + '    ld r5,[0xc0c]\n'
        yield Case('sig/R%x-a%x-i%x' % (R, arg, imm), finish(t, body), dict(timeout_us=20000, flags=DISCOVERY),
                   cond(4, 'CS_IRQ bit 1 in samples (r[7]) and r5 = zero-form result; vic0/vic1_seen bits that appear '
                           'only with sig = its CPU interrupt line', form=(R, hex(arg), hex(imm))))
    body = insn(0x10) + delay(300) + Q.wr(0xc0c, 2) + '    ld r4,[0xc0c]\n' + insn(0x10) + delay(300) + '    ld r5,[0xc0c]\n'
    yield Case('sig/ack-resignal', finish(t, body), dict(timeout_us=20000, flags=DISCOVERY),
               cond(4, 'r4 bit1 clear and r5 bit1 set = W1C acknowledge and re-signal'))
    for n in (1000, 21000):
        lp = label('sg')
        body = '    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n    bnz r4,%s\n' % (n, lp, insn(0x10), lp)
        yield Case('sig/cost-n%d' % n, finish(t, body), dict(timeout_us=200000, flags=DISCOVERY),
                   cond(4, 'per-sig cost from the 1000/21000 difference minus the m2_opcost empty loop', iterations=n))
    tm = []
    for imm in (1, 0x10, 0x100, 0x1000, 0x10000):
        tm.append(('imm%x' % imm, insn(0x15, 0, 0, imm), SHORT, 4, ''))
    for R in (1, 0xff):
        tm.append(('R%x' % R, insn(0x15, R, 0, 0), SHORT, 4, ''))
    for arg in (1, 0xffff):
        tm.append(('arg%x' % arg, insn(0x15, 0, arg, 0), SHORT, 4, ''))
    for start in (0x01b5, 0x07b5, 0x40b5):
        tm.append(('start%04x' % start, insn(0x15), start, 4, ''))
    for c14 in (0x10, 0x1000):
        tm.append(('c14-%x' % c14, insn(0x15), 0xffb5, 4, Q.wr(0xc14, c14)))
    tm.append(('c58-0', insn(0x15), SHORT, 0, ''))
    for name, ins, start, policy, pre in tm:
        body = pre + Q.store(0xc14, a + 0x2000) + MARK1 + ins + MARK2 + Q.store(0xc14, a + 0x2004) + Q.store(0xc0c, a + 0x2008)
        yield Case('tmwait/' + name, finish(t, body, policy),
                   dict(start_value=start, scratch=memory(), scratch_read=0x2010, flags=DISCOVERY | P.WAIT_END,
                        timeout_us=200000),
                   cond(4, 'elapsed vs start-word count (0x01 ~1.4 ms, 0x07 ~4.4 ms, 0x40 ~32 ms, 0xff ~126 ms): '
                           'proportional = tmwait counts the timeout timer; constant = own reload; C14 preset '
                           'changes it = reload from C14', start_word=hex(start), c58=policy,
                        layout='0x2000: C14 before, after, CS_IRQ after; r2 0x77770002 = continued'))
    for R, arg, imm in ((0, 0, 0), (1, 0, 0), (8, 0, 0), (10, 0, 0), (0xff, 0, 0), (0, 2, 0), (0, 0x8000, 0),
                        (0, 0xffff, 0), (0, 0, 1), (0, 0, 0x80000000), (0xff, 0xffff, MASK)):
        body = MARK1 + insn(8, R, arg, imm) + MARK2
        yield Case('nop/R%x-a%x-i%x' % (R, arg, imm), finish(t, body),
                   dict(params=seeds(), start_value=SHORT, flags=DISCOVERY | P.WAIT_END | P.FULLMAP, timeout_us=20000),
                   cond(4, 'FULLMAP after and GPRs equal to the zero form (except r2/r6 markers) = fields ignored; '
                           'any change = the field is an operand', form=(R, hex(arg), hex(imm))))


# ---- m2_open_cycle_timing (ch17 #6)
FRACTION_N = 20000
FRACTION_COPIES = (1, 2, 3, 4, 5, 8)


def fraction_insn(kind):
    if kind == 'nop08': return insn(8)
    if kind == 'nop2ff': return NOP
    if kind == 'sti': return '    sti [r5],r1\n'
    if kind == 'movi': return '    movi r1,#0x1234\n'
    nxt = label('nx')
    reg = 0 if kind == 'bnz-untaken' else 6
    return '    bnz r%d,%s\n%s:\n' % (reg, nxt, nxt)


def cycle_timing(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    kinds = ('nop08', 'nop2ff', 'bnz-untaken', 'bnz-taken-next', 'sti', 'movi')
    for kind, k in [('empty', 0)] + [(kd, k) for kd in kinds for k in FRACTION_COPIES]:
        lp = label('cf')
        inner = ''.join(fraction_insn(kind) for _ in range(k))
        body = ('    movi r0,#0\n    movi r6,#1\n    movi r5,#0x%x\n    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n'
                '    bnz r4,%s\n' % (a + 0x100, FRACTION_N, lp, inner, lp))
        yield Case('fraction/%s/x%d' % (kind, k), finish(t, body),
                   dict(scratch=memory(), scratch_read=0x200, timeout_us=200000),
                   cond(6, 'slope of elapsed_us over copies / %d iterations = cost per instruction; a non-integer '
                           'slope in 33.25 MHz cycles (or copies that do not add linearly) = fractional/pairing cost'
                        % FRACTION_N, instruction=kind, copies=k, iterations=FRACTION_N))
    phases = [(0x00b5, d) for d in (0, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000)]
    phases += [(0x0075, d) for d in (0, 50, 100, 200, 300, 400)]
    for word, d in phases:
        body = (delay(d) if d else '') + MARK1 + Q.wait(3) + MARK2
        yield Case('phase/%04x/d%d' % (word, d), finish(t, body),
                   dict(start_value=word, flags=DISCOVERY | P.WAIT_END, timeout_us=50000),
                   cond(6, 'fault_at_us - wait entry (r2 sample t_us) constant = timer starts at the wait; fault_at_us '
                           'constant = runs from the start; a sawtooth with period = step 2^(n+3) cycles = free-running '
                           'prescaler, its phase from the offset', start_word=hex(word), delay_loops=d,
                        loop_ns=67.7))


# ---- m2_open_c08_states (ch17 #8)
def c08_states(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    self_read = '    ld r1,[0xc08]\n    st [0xd40],r1\n    ld r1,[0xc3c]\n    st [0xd44],r1\n' \
                '    ld r1,[0xc5c]\n    st [0xd48],r1\n    ld r1,[0xc14]\n    st [0xd4c],r1\n'
    states = {
        'loop': (delay(60000), 0xffb5, 4),
        'wait2-40': (insn(9, 40), 0xffb5, 4),
        'tmwait': (insn(0x15), 0xffb5, 4),
        'evjmp-idle-c58-0': (insn(0x16), 0xffb5, 0),
        'pause': (insn(0x12), 0xffb5, 4),
        'sig-loop': (insn(0x10) + delay(60000), 0xffb5, 4),
        'start1/wait3': (Q.wait(3), 1, 4),
        'fault-continue-loop': (Q.wait(3) + delay(60000), 0x01b5, 0),
        'handler-loop': (Q.wait(3), 0x01b5, 4),
        'ended': ('', 0xffb5, 4),
    }
    for n in (0, 1, 2, 3, 4, 6, 13, 15):
        states['wait%d' % n] = (Q.wait(n), 0xffb5, 4)
    for name, (body, start, policy) in states.items():
        handler = None
        if name == 'handler-loop':
            handler = t.trap().replace('trap:\n', 'trap:\n' + delay(60000))
        src = finish(t, Q.setup() + self_read + body, policy, handler)
        yield Case(name, src, dict(start_value=start, params=[(0xd40 + 4 * k, 0) for k in range(4)],
                                   flags=DISCOVERY, timeout_us=3000, scratch=memory(), scratch_read=16),
                   cond(8, 'after-map C08/C00/C14/C3C/C5C/C50 at the 3 ms observation (before cleanup) give the code '
                           'for this state; a code outside 0, 4, a, b is new; 0xd40..0xd4c = sequencer self-reads of '
                           'C08, C3C, C5C, C14 before it', state=name, start_word=hex(start), c58=policy,
                        observe_us=3000))


# ---- m2_open_wait_sources (ch17 #9)
WAITS = (6, 7, 8, 9, 10, 13, 14, 15)


def wait_sources(s, t):
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    opened = Q.setup(g.ce) + Q.command(0) + Q.address(g.row + 100) + Q.command(0x30) + Q.ready() + Q.command(0)
    readid = (Q.setup(g.ce) + Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1)
              + Q.wr(0x48, 4) + Q.wr(0x30, 7) + Q.wr(0x14, 0x10) + Q.wr(4, 0xe2) + Q.wait(3) + Q.wr(0x48, 8))
    unacked = lambda src: src.replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')
    states = {
        'dma-out-unacked': (Q.setup() + Q.dma(a, 0) + unacked(Q.dma(a + 0x1000, 0, into=False)), False),
        'data-read-unacked': (opened + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(4, 0xe2) + delay(400), True),
        'parity-read-unacked': (opened + Q.wr(0x30, g.parity + 9) + Q.wr(0x14, 0x10) + Q.wr(4, 0x32) + delay(400), True),
        'fifo-52': (readid + Q.wr(0x40, 0x52) + delay(50), True),
        'meta-c1': (Q.setup() + Q.metadata(bytes(range(10))), False),
        'irq044-dma': (Q.setup() + Q.wr(0x44, MASK) + unacked(Q.dma(a, 0)) + delay(100), False),
        'ecc-encode-slot1': (Q.setup() + Q.dma(a, 1) + Q.ecc(1, acknowledge=False) + delay(400), False),
        'ecc-encode-bch16': (Q.setup() + Q.dma(a, 0) + Q.ecc(0, strength=16, acknowledge=False) + delay(400), False),
        'ecc-reset-81c': (Q.setup() + Q.wr(0x81c, 1), False),
        'ce1-command': (Q.setup(1) + Q.wr(8, 0x70), True),
        'ce2-command': (Q.setup(2) + Q.wr(8, 0x70), True),
        'ce3-command': (Q.setup(3) + Q.wr(8, 0x70), True),
    }
    for name, (prep, pins) in states.items():
        for n in WAITS:
            body = prep + Q.store(0x48, a + 0x2000) + Q.store(0x810, a + 0x2004) + MARK1 + Q.wait(n) + MARK2 + Q.wr(0x44, 0)
            yield Case('%s/wait%02d' % (name, n), finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2010, start_value=SHORT, flags=DISCOVERY, timeout_us=50000),
                       cond(9, 'r2 0x77770002 before the 4.4 ms fault = WAIT %d released by %s' % (n, name),
                            stimulus=name, wait=n, layout='0x2000: 048, 810 before the WAIT'), pins)
    cpu = {'c40': [(0xc40, 1)], 'c44': [(0xc44, 1)], 'c50': [(0xc50, MASK)], 'c14': [(0xc14, 0)],
           'c5c': [(0xc5c, 1)], 'c3c': [(0xc3c, 0)]}
    restore = {'c40': [(0xc40, 0)], 'c44': [(0xc44, 0)], 'c5c': [(0xc5c, 0)]}
    for name, writes in cpu.items():
        for n in WAITS:
            opts = dict(start_value=SHORT, flags=DISCOVERY, timeout_us=50000, late=writes, late_us=1000)
            if name in restore:
                opts.update(late2=restore[name], late2_us=2500)
            yield Case('cpu-%s/wait%02d' % (name, n), finish(t, Q.setup() + MARK1 + Q.wait(n) + MARK2), opts,
                       cond(9, 'release right after the 1 ms CPU write (r2 sample t_us) = that word is the source',
                            cpu_write=[(hex(o), hex(v)) for o, v in writes], wait=n))
    for vic in (0, 1):
        for n in WAITS:
            opts = dict(start_value=SHORT, flags=DISCOVERY | VICSOFT, timeout_us=50000,
                        late=[(vic_param(vic, 0x18), MASK)], late_us=1000, late2=[(vic_param(vic, 0x1c), MASK)], late2_us=2500)
            yield Case('vicsoft%d/wait%02d' % (vic, n), finish(t, Q.setup() + MARK1 + Q.wait(n) + MARK2), opts,
                       cond(9, 'release between 1 and 2.5 ms = WAIT %d follows a VIC%d line (soft-raised, CPU-enabled '
                               'lines masked by the probe)' % (n, vic), vic=vic, wait=n))


# ---- m2_open_wait11 (ch17 #10)
def wait11(s, t):
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    unacked = lambda src: src.replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')
    after = Q.store(0xc08, a + 0x2000) + Q.store(0x48, a + 0x2004) + Q.store(0x810, a + 0x2008)
    variants = {
        'idle/c58-0': (Q.setup(), 0, SHORT, False),
        'idle/c58-4': (Q.setup(), 4, SHORT, False),
        'start1': (Q.setup(), 4, 1, False),
        'dma-in-flight': (Q.setup() + unacked(Q.dma(a, 0)), 4, SHORT, False),
        'ecc-running': (Q.setup() + Q.dma(a, 0) + Q.ecc(0, acknowledge=False), 4, SHORT, False),
        'command-pending': (Q.setup(g.ce) + Q.wr(8, 0x70), 4, SHORT, True),
        'status-latch-pending': (Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca), 4, SHORT, True),
        'all-acked': (Q.setup() + Q.wr(0x48, MASK) + Q.wr(0x810, 0x1ff) + Q.wr(0xc0c, 0xff), 4, SHORT, False),
        'c10-0': (Q.setup() + Q.wr(0xc10, 0), 4, SHORT, False),
        'irq044-all': (Q.setup() + Q.wr(0x44, MASK), 4, SHORT, False),
    }
    for name, (prep, policy, start, pins) in variants.items():
        body = prep + MARK1 + Q.wait(11) + MARK2 + after + Q.wr(0xc10, 1) + Q.wr(0x44, 0)
        yield Case(name, finish(t, body, policy),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=start, flags=DISCOVERY | P.WAIT_END,
                        timeout_us=20000),
                   cond(10, 'WAIT11 not passing at once in a state names its condition; passing everywhere keeps '
                            'it an always-true event', state=name, layout='0x2000: C08, 048, 810 after'), pins)
    for arg, mask in ((0x40, 0x40), (0xffff, MASK)):
        body = Q.setup() + MARK1 + Q.wait(11, arg, mask) + MARK2
        yield Case('form/a%x-m%x' % (arg, mask), finish(t, body), dict(start_value=SHORT, flags=DISCOVERY, timeout_us=20000),
                   cond(10, 'operands change the outcome = WAIT11 tests a masked condition', arg=hex(arg), mask=hex(mask)))
    for n in (1000, 21000):
        lp = label('w11')
        body = '    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n    bnz r4,%s\n' % (n, lp, Q.wait(11), lp)
        yield Case('cost/n%d' % n, finish(t, body), dict(timeout_us=200000, flags=DISCOVERY),
                   cond(10, 'per-WAIT11 cost; equal to a plain instruction = no synchronisation with another clock',
                        iterations=n))


# ---- m2_open_wait5_history (ch17 #11)
def wait5_history(s, t):
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    c, latch = Q.wr(8, 0x70), Q.wr(4, 0xca)
    targets = {'ack-before-latch': c + Q.wr(0x48, 2) + latch,
               'delay1-before': c + '    movi r5,#0\n' + latch}
    unacked = lambda src: src.replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')
    preds = {
        'none': None,
        'status-wait15-fault': Q.setup(g.ce) + c + latch + '    ld r2,[0x48]\n' + Q.wait(15) + '    ld r3,[0x48]\n',
        'status-wait12-pass': Q.setup(g.ce) + c + latch + Q.wait(12, 0x40, 0x40),
        'latch-no-wait': Q.setup(g.ce) + c + latch,
        'latch-twice': Q.setup(g.ce) + c + latch + latch,
        'command-unacked': Q.setup(g.ce) + c,
        'dma-unacked': Q.setup() + unacked(Q.dma(a, 0)),
        'ecc-unacked': Q.setup() + Q.dma(a, 0) + Q.ecc(0, acknowledge=False),
        'fifo-readid': Q.setup(g.ce) + Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(4, 1) + Q.wait(1)
                       + Q.wr(0x48, 4) + Q.wr(0x40, 0x52),
    }
    for pname, pbody in preds.items():
        if pbody is not None:
            yield Case('%s/pre' % pname, finish(t, pbody),
                       dict(scratch=memory(), scratch_read=16, timeout_us=200000, flags=P.WAIT_END),
                       cond(11, 'predecessor: leaves its state for the next case (only the probe reset between)',
                            predecessor=pname), True)
        for tname, action in targets.items():
            body = Q.setup(g.ce) + action + '    ld r2,[0x48]\n' + Q.wait(5) + '    ld r3,[0x48]\n    ld r4,[0x4c]\n'
            yield Case('%s/%s' % (pname, tname), finish(t, body), dict(timeout_us=200000, flags=P.WAIT_END),
                       cond(11, 'fault only after predecessor %s = its leftover state (surviving the probe reset) causes '
                                'the p16 WAIT5 fault; pass everywhere = cause is a longer history' % pname,
                            predecessor=pname, target=tname, reference='p16 status_order %s/event5' % tname), True)


# ---- m2_open_evjmp_forms (ch17 #12)
EVJMP_FORMS = ((0, 0, 0), (1, 0, 0), (2, 0, 0), (4, 0, 0), (0xff, 0, 0), (0, 1, 0), (0, 2, 0), (0, 0xffff, 0),
               (0, 0, 'alternate'), (0, 0, MASK))
RESET_ORDERS = (('stop', (8,)), ('abort', (2,)), ('reset', (0x80000,)), ('stop-abort', (8, 2)),
                ('stop-reset', (8, 0x80000)), ('abort-reset', (2, 0x80000)), ('reset-stop', (0x80000, 8)),
                ('abort-stop', (2, 8)))


def evjmp_forms(s, t):
    a, g = t.scratch_addr(s), Q.geometry(s)
    unacked = lambda src: src.replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')
    dma = unacked(Q.dma(a, 0))
    ecc = Q.dma(a, 0) + Q.ecc(0, acknowledge=False)
    stimuli = {'command': (Q.wr(8, 0x70), True), 'DMA': (dma, False), 'ECC': (ecc, False),
               'status': (Q.command(0x70) + Q.wr(4, 0xca), True), 'command+DMA': (dma + Q.wr(8, 0x70), True),
               'DMA+ECC': (ecc + unacked(Q.dma(a + 0x400, 1)), False)}
    handler = ('vector:\n    ld r1,[0x48]\n    st [0xd00],r1\n    ld r1,[0xc50]\n    st [0xd04],r1\n'
               '    ld r1,[0xc0c]\n    st [0xd08],r1\n    ld r1,[0x810]\n    st [0xd0c],r1\n' + Q.wr(0, 0) +
               '    movi r6,#0x0a17a17\n    end\nalternate:\n    ld r1,[0x48]\n    st [0xd00],r1\n' + Q.wr(0, 0) +
               '    movi r6,#0xa17e0000\n    end\n')
    for sname, (action, pins) in stimuli.items():
        for R, arg, imm in EVJMP_FORMS:
            src = (t.prologue(4) + '    wr [FMSS_C4C],#vector\n' + Q.setup(g.ce if pins else None) + action
                   + Q.wr(0xd70, 1) + insn(0x16, R, arg, imm) + Q.wr(0xd70, 2) + Q.wr(0, 0)
                   + '    movi r6,#0x600d600d\n    end\n' + handler + t.trap())
            yield Case('%s/R%x-a%x-i%s' % (sname, R, arg, imm if isinstance(imm, str) else '%x' % imm), src,
                       dict(params=[(0xd70, 0)] + [(0xd00 + 4 * k, 0) for k in range(4)], scratch=memory(), scratch_read=16,
                            start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                       cond(12, 'r6 0x0a17a17 = vector to C4C, 0xa17e0000 = jump to imm, 0x600d600d = fell through; '
                                'differences from the zero form name operand roles; d00..d0c = 048, C50, CS_IRQ, 810 '
                                'in the handler give the source(s) seen', stimulus=sname, form=(R, hex(arg), str(imm))),
                       pins)
    for name, writes in RESET_ORDERS:
        opts = dict(start_value=1, flags=DISCOVERY, timeout_us=5000, late=[(0xc00, writes[0])], late_us=1000)
        if len(writes) > 1:
            opts.update(late2=[(0xc00, writes[1])], late2_us=2000)
        yield Case('reset-order/%s' % name, finish(t, Q.setup() + insn(0x16)), opts,
                   cond(12, 'after-map C08/C00/CS_IRQ/PC after the CPU sequence on the blocked evjmp (start = 1, '
                            'C08 = b); 0 = idle, 3 = aborted; cleanup method in reset_health', order=name,
                        writes=[hex(w) for w in writes], at_us=(1000, 2000)[:len(writes)],
                        reference='STOP leaves b, abort gives 3, 0x80000 before STOP gives 0 (M2-62)'))


# ---- m2_open_bank_enable (ch17 #14)
BANK_ENABLES = (('c10', 0xc10, (0, 2, 3, 0xff), 1), ('018', 0x018, (1, 2, 0xff), 0), ('058', 0x058, (1, 0xff), 0),
                ('05c', 0x05c, (0x200, 0xffff), 0), ('01c', 0x01c, (2, 0xff), 0), ('c48', 0xc48, (1,), 0),
                ('c54', 0xc54, (1,), 0), ('400', 0x400, (0x1077803,), 0), ('438', 0x438, (7,), 0))


def bank_enable(s, t):
    a = t.scratch_addr(s)
    readback = (0x40c, 0x410, 0x434, 0x400, 0x438, 0x00c, 0x010, 0x034)
    for name, off, values, rest in BANK_ENABLES:
        for v in values:
            body = (Q.setup() + Q.wr(off, v) + Q.wr(0x40c, 0x12345678) + Q.wr(0x410, 0x9abcdef0) + Q.wr(0x434, 0x0f0f0f0f)
                    + stores(readback, a + 0x2000) + Q.wr(off, rest) + Q.wr(0x40c, 0) + Q.wr(0x410, 0) + Q.wr(0x434, 0)
                    + Q.wr(0, 0))
            yield Case('%s/%x' % (name, v), finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2020, timeout_us=20000, flags=DISCOVERY),
                       cond(14, 'any non-zero +400 readback = this write enables the second set; zeros everywhere keeps '
                                'it absent or enabled elsewhere', enable=(hex(off), hex(v)), restored=hex(rest),
                            layout='0x2000: ' + ' '.join('%03x' % r for r in readback)))


# ---- m2_open_dma_cut (ch17 #15)
def dma_cut(s, t):
    a = t.scratch_addr(s)
    mem = memory()
    pres = {'none': '', 'ecc8': Q.ecc(0, strength=8), 'ecc16': Q.ecc(0, strength=16)}
    for length in (64, 128, 256, 512):
        for pre, ecc in pres.items():
            for gap in (0, 16):
                body = Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1) + ecc
                body += Q.wr(0x38, 7) + Q.wr(0x30, length - 1) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
                body += NOP * gap + Q.dma(a + 0x2400, 1, into=False) + Q.wait(2) + Q.wr(0x48, 0x100000)
                body += Q.store(0x38, a + 0x2800) + Q.store(0x48, a + 0x2804)
                yield Case('out/len%d/%s/gap%d' % (length, pre, gap), finish(t, body),
                           dict(scratch=memory(), scratch_read=0x2810, timeout_us=50000),
                           cond(15, 'bytes of 0x2000 equal to the reference before the first unchanged byte = cut length; '
                                    'cut constant in length = fixed burst/FIFO depth, changing with the preceding ECC = '
                                    'engine-state dependent', length=length, preceding=pre, gap_nops=gap,
                                reference_ranges={'0x2000': bytes(mem[:length]).hex(), '0x2400': bytes(mem[0x400:0x600]).hex()},
                                layout='0x2000 first output (cut), 0x2400 second, 0x2800: 038 048'))
    for length in (64, 128, 256, 512):
        for gap in (0, 16):
            body = Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, length - 1) + Q.wr(0x14, 1) + Q.wr(0x34, a) + Q.wr(4, 0x2e0)
            body += NOP * gap + Q.dma(a + 0x400, 1) + Q.store(0x38, a + 0x2800) + Q.store(0x48, a + 0x2804)
            body += Q.wr(0x48, 0x100000) + Q.dma(a + 0x2000, 0, into=False) + Q.dma(a + 0x2400, 1, into=False)
            yield Case('in/len%d/gap%d' % (length, gap), finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2810, timeout_us=50000),
                       cond(15, 'prefix of 0x2000 equal to the first input = bytes the cut DMA in delivered to buffer 0',
                            length=length, gap_nops=gap, direction='in',
                            reference_ranges={'0x2000': bytes(mem[:512]).hex(), '0x2400': bytes(mem[0x400:0x600]).hex()},
                            layout='0x2000 buffer 0 read back, 0x2400 buffer 1, 0x2800: 038 048 after the second input'))


# ---- m2_open_dma_ctrl (ch17 #16)
def dma_ctrl(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    mem = memory()
    values = [v for b in range(6, 13) for v in ((1 << b) | 7, 1 << b)] + [0x1fc7, 0]
    for v in values:
        body = (Q.setup() + Q.wr(0x38, v) + Q.store(0x38, a + 0x2400) + Q.dma(a, 0, control=None)
                + Q.store(0x38, a + 0x2404) + Q.store(0x48, a + 0x2408) + Q.dma(a + 0x2000, 0, into=False, control=None)
                + Q.store(0x38, a + 0x240c) + Q.wr(0x38, 7))
        yield Case('value%04x' % v, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2410, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(16, 'readback mask/transform of each bit, DMA completion (no fault) and exact payload at 0x2000 '
                            'give the role of the written bit', written=hex(v), reference=bytes(mem[:512]).hex(),
                        layout='0x2400: 038 after write, after DMA in, 048, 038 after DMA out; 0x2000 payload'))
    for direction in ('in', 'out'):
        for length in (64, 256, 512):
            into = direction == 'in'
            pre = Q.setup() + ('' if into else Q.dma(a, 0))
            regs = (Q.wr(0x38, 7) + Q.wr(0x30, length - 1) + Q.wr(0x14, 1 if into else 0x100)
                    + Q.wr(0x34, a if into else a + 0x1000))
            body = (pre + regs + '    movi r7,#0x%x\n' % (a + 0x2000) + Q.wr(4, 0x2e0 if into else 0x100)
                    + sampler(0x38, 96, 0) + Q.wait(2) + Q.wr(0x48, 0x100000))
            yield Case('timeline/%s/len%d' % (direction, length), finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2000 + 4 * 96, timeout_us=20000, flags=DISCOVERY),
                       cond(16, 'bits 7..12 stepping in proportion to bytes moved (final count ~ length/unit) = progress '
                                'counter; same sequence for every length = state code', direction=direction,
                            length=length, samples=96, sample_ns=188.0, layout='0x2000 + 4k: 038 sample k'))


# ---- m2_open_ecc_collision (ch17 #17)
def ecc_collision(s, t):
    a = t.scratch_addr(s)
    mem = memory()
    A, B = bytes(mem[:512]), bytes(mem[0x400:0x600])
    par = {8: (F.bch8_sector(A).hex(), F.bch8_sector(B).hex()), 16: (F.bch16_parity(A).hex(), F.bch16_parity(B).hex())}
    out_regs = (0x48, 0x810) + tuple(range(0x840, 0x860, 4))
    for strength in (8, 16):
        kick = 6 if strength == 16 else 2
        for mode in ('serial', 'in', 'out'):
            for gap in ((0,) if mode == 'serial' else (0, 8, 32, 128)):
                body = Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, kick)
                if mode == 'serial':
                    body += Q.wait(4) + stores(out_regs, a + 0x2600) + Q.wr(0x810, 1) + Q.dma(a + 0x400, 0)
                else:
                    body += NOP * gap
                    body += Q.dma(a + 0x400, 0) if mode == 'in' else Q.dma(a + 0x2200, 0, into=False)
                    body += Q.wait(4) + stores(out_regs, a + 0x2600) + Q.wr(0x810, 1)
                body += Q.dma(a + 0x2000, 0, into=False) + Q.wr(0x48, 0x2000000)
                yield Case('bch%d/%s/gap%d' % (strength, mode, gap), finish(t, body),
                           dict(scratch=memory(), scratch_read=0x2640, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                           cond(17, 'in: 0x2000 = buffer after collision (runs of A = dropped DMA bursts, their sizes); '
                                    'out: 0x2200 vs A = whether the DMA side loses; parity 0x2608.. vs host parity of A '
                                    'or B = which data the ECC used; 048 bit 25 at 0x2600',
                                strength=strength, mode=mode, gap_nops=gap, host_parity_A=par[strength][0],
                                host_parity_B=par[strength][1],
                                reference_ranges={'A': A.hex(), 'B': B.hex()},
                                layout='0x2000 buffer 0 read back, 0x2200 out-mode output, 0x2600: 048 810 840..85c'))


# ---- m2_open_fmc_words (ch17 #18)
F30_VALUES = (0, 1, 3, 7)


def fmc_words(s, t):
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = memory()
    words = [(0x18, v) for v in (1, 2, 3, 0xff)] + [(0x58, v) for v in (1, 0x10, 0x100, 0xff)]
    words += [(0x5c, v) for v in (0x100, 0x200, 0x1ff, 0x3ff)]
    after = stores((0x18, 0x58, 0x5c, 0x38, 0x48), a + 0x2400)
    for off, v in [(None, 0)] + words:
        pre = '' if off is None else Q.wr(off, v)
        body = Q.setup() + pre + Q.dma(a, 0) + Q.dma(a + 0x2000, 0, into=False) + after + ('' if off is None else Q.wr(off, 0))
        yield Case('loop/%s' % ('base' if off is None else '%03x-%x' % (off, v)), finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2414, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(18, 'payload at 0x2000 different from the reference, a length change or a DMA fault = the word '
                            'acts on DMA; identical = no role in a buffer loopback', word=None if off is None else hex(off),
                        value=hex(v), reference=bytes(mem[:512]).hex(), layout='0x2400: 018 058 05c 038 048 after'))
    for sel in (1, 0x10, 0x100):
        body = (Q.setup() + Q.wr(0x58, sel) + Q.dma(a, 0, select=0) + Q.wr(0x58, 0) + Q.dma(a + 0x2000, 0, into=False)
                + after)
        yield Case('select058/%x' % sel, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2414, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(18, 'payload in buffer 0 with 014 = 0 and 058 = %x = 058 selects the DMA buffer' % sel,
                        select=hex(sel), reference=bytes(mem[:512]).hex()))
    base = ctrl0_base(g)
    readid_addr = Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1) + Q.wr(0x48, 4)
    r7 = '    movi r7,#0x%x\n' % (a + 0x2000)
    ops = {
        'cmd70': ('', r7 + Q.wr(8, 0x70), Q.wait(0) + Q.wr(0x48, 2)),
        'latch': (Q.command(0x70), r7 + Q.wr(4, 0xca), Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000)),
        'addr-to-data': (readid_addr + Q.wr(0x30, 7) + Q.wr(0x14, 0x10), r7 + Q.wr(4, 0xe2), Q.wait(3) + Q.wr(0x48, 8)),
        'ready-to-cmd': (Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready(), r7 + Q.wr(8, 0),
                         Q.wait(0) + Q.wr(0x48, 2)),
    }
    for op, (pre, trig, tail) in ops.items():
        for v in F30_VALUES:
            word = (base & ~(7 << 28)) | v << 28
            body = Q.setup(g.ce) + Q.wr(0, word) + pre + trig + sampler(0x48, 96, 0) + tail + Q.wr(0, base)
            yield Case('f30/%s/%d' % (op, v), finish(t, body),
                       dict(scratch=memory(), scratch_read=0x2000 + 4 * 96, timeout_us=50000, flags=DISCOVERY),
                       cond(18, 'completion-bit sample index shifting with f30:28 = it sets that delay (tWHR/tAR/tRR '
                                'class); identical timelines for all values keep it inert', op=op, f30_28=v, ctrl0=hex(word),
                            samples=96, sample_ns=188.0, layout='0x2000 + 4k: 048 sample k after the trigger'), True)


# ---- m2_open_op_words (ch17 #19)
def op_words(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    snap = lambda k: stores((0x48, 0xc50, 0x38, 0x4c, 0xc08, 0x810), a + 0x6000 + 0x20 * k)
    for bit in range(32):
        body = (Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, 15) + Q.wr(0x14, 1) + Q.wr(0x34, a + 0x5000) + snap(0)
                + Q.wr(4, 1 << bit) + snap(1) + delay(200) + snap(2) + Q.wr(0x48, 0x0ff00ffe))
        yield Case('fmctrl1/bit%02d' % bit, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x6060, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   cond(19, 'flags set by the single bit (048/C50/038 change, 16-byte DMA at scratch+0x5000) assign '
                            'it; no change = inert alone (no CE selected)', bit=bit, value=hex(1 << bit),
                        layout='0x6000 + 0x20k: 048 C50 038 04c C08 810 before, right after, +200 loops'))
    masks = {
        'w1c-ones-after-flags': Q.setup() + Q.dma(a, 0).replace(Q.wr(0x48, 0x100000), '')
                                + snap(0) + Q.wr(0x48, MASK) + snap(1),
        'write-zero-after-flags': Q.setup() + Q.dma(a, 0).replace(Q.wr(0x48, 0x100000), '') + snap(0) + Q.wr(0x48, 0) + snap(1),
        'write-ones-idle': Q.setup() + Q.wr(0x48, MASK) + snap(0) + Q.wr(0x48, MASK) + snap(1),
        '04c-ones': Q.setup() + snap(0) + Q.wr(0x4c, MASK) + snap(1) + Q.wr(0x4c, 0),
        '04c-zero': Q.setup() + snap(0) + Q.wr(0x4c, 0) + snap(1),
    }
    for name, body in masks.items():
        yield Case('mask/%s' % name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x6060, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   cond(19, 'bits cleared by ones = W1C, kept by zero = no write-0 effect, set by ones = writable',
                        case=name, layout='0x6000: before, 0x6020: after the write (048 C50 038 04c C08 810)'))


# ---- m2_open_ecc_slots (ch17 #21)
def ecc_slots(s, t):
    a = t.scratch_addr(s)
    mem = memory()
    A, B = bytes(mem[:512]), bytes(mem[0x400:0x600])
    out = (0x810, 0x01c) + tuple(range(0x840, 0x860, 4))
    load = Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1)
    ref = lambda st: dict(host_parity_slot0=(F.bch8_sector(A) if st == 8 else F.bch16_parity(A)).hex(),
                          host_parity_slot1=(F.bch8_sector(B) if st == 8 else F.bch16_parity(B)).hex())
    cases = [('sel%x' % v, v, 8) for v in (2, 3, 4, 7, 8, 0xff, 0x100, 0x80000000)] + [('sel2/bch16', 2, 16)]
    for name, v, st in cases:
        body = (load + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, v) + Q.wr(0x804, 6 if st == 16 else 2) + MARK1 + Q.wait(4)
                + MARK2 + stores(out, a + 0x2000) + Q.wr(0x810, 1) + Q.wr(0x1c, 0))
        yield Case(name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2030, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(21, 'parity equal to slot 0 or slot 1 host parity = selector decoded modulo 2 (or which bit); '
                            'neither / WAIT4 fault = no buffer behind that value', selector=hex(v), strength=st,
                        layout='0x2000: 810 01c 840..85c', **ref(st)))
    arb = {
        'back-to-back': (Q.wr(0x1c, 0) + Q.wr(0x804, 2) + Q.wr(0x1c, 1) + Q.wr(0x804, 2) + Q.wait(4)
                         + stores(out, a + 0x2000) + Q.wr(0x810, 1) + Q.wait(4) + stores(out, a + 0x2040) + Q.wr(0x810, 1)),
        'slot-change-mid': (Q.wr(0x1c, 0) + Q.wr(0x804, 2) + Q.wr(0x1c, 1) + Q.wait(4) + stores(out, a + 0x2000)
                            + Q.wr(0x810, 1)),
    }
    for name, tail in arb.items():
        body = load + Q.wr(0x810, 0x1ff) + tail + Q.wr(0x1c, 0)
        yield Case('arb/' + name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2070, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(21, 'which host parity each capture matches: second kick ignored, queued or replacing the first; '
                            'selector sampled at kick or at completion', case=name,
                        layout='0x2000 (and 0x2040): 810 01c 840..85c', **ref(8)))


# ---- m2_open_ecc_status_bits (ch17 #22)
ECC810_BITS = (4, 7, 8, 9, 10, 11, 12, 16, 24, 31)
ECC81C_VALUES = (1, 2, 4, 0x80, 0x100, MASK, 0)


def ecc_status_bits(s, t):
    import hashlib
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    encode = Q.dma(a, 0) + Q.wr(0x1c, 0) + Q.wr(0x804, 2) + Q.wait(4)
    for b in ECC810_BITS:
        body = (Q.setup() + Q.wr(0x810, 0x1ff) + Q.store(0x810, a + 0x2000) + Q.wr(0x810, 1 << b) + Q.store(0x810, a + 0x2004)
                + encode + Q.store(0x810, a + 0x2008) + Q.wr(0x810, 1 << b) + Q.store(0x810, a + 0x200c) + Q.wr(0x810, 0x1ff))
        yield Case('810/bit%02d' % b, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(22, 'bit set by the write = writable, set by the engine and cleared by the write = W1C status, '
                            'never set = unused', bit=b, layout='0x2000: 810 idle, after write, after encode, after write'))
    regs = (0x810, 0x818, 0x048, 0x80c)
    for slot, v in [(0, v) for v in ECC81C_VALUES] + [(1, 1)]:
        body = (Q.setup() + Q.dma(a, slot) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, slot) + Q.wr(0x804, 2) + Q.wait(4)
                + stores(regs, a + 0x2000) + Q.wr(0x81c, v) + stores(regs, a + 0x2010)
                + MARK1 + Q.wait(4) + MARK2 + Q.wr(0x810, 0x1ff))
        yield Case('81c/slot%d/%x' % (slot, v), finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2020, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond(22, 'changes between 0x2000 and 0x2010 = what this 81c value resets; WAIT4 fault after it = '
                            'completion flag cleared', value=hex(v), slot=slot,
                        layout='0x2000: 810 818 048 80c before, 0x2010 after 81c; r2 0x77770002 = WAIT4 still satisfied'))
    if g.strength != 8:
        return
    raw, clean = F.coded_page(F.RING_ERRORS)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('ring/erase', finish(t, Q.erase(g)), dict(mutate), cond(22, 'scratch block erase', block=g.block), True)
    yield Case('ring/program', finish(t, Q.write_page(g, g.row, a, F.BCH_META, raw=True)),
               dict(mutate, scratch=mem, scratch_read=0),
               cond(22, 'host code words with 1..8 errors per sector', errors=F.RING_ERRORS,
                    raw_sha256=hashlib.sha256(raw).hexdigest()), True)
    variants = {'cpu-pre0': (SNAP_SKIP_800, ''), 'cpu-pre1': (0, ''), 'cpu-pre2': (P.FULLMAP, ''),
                'cpu-pre2-reset': (P.FULLMAP, Q.wr(0x81c, 1))}
    for name, (flags, pre) in variants.items():
        body = pre + F.decode_loop(g, g.row, a + 0x2040, 8, 8, lambda sec, slot: '')
        body += ''.join(Q.store(0x80c, a + 0x3000 + 4 * k) for k in range(4))
        yield Case('ring/' + name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x3010, timeout_us=300000, flags=flags),
                   cond(22, 'first sequencer 80c read (0x3000) = result of sector k where k = CPU 80c reads before '
                            'the start (0 skip, 1 window, 2 window + FULLMAP) => CPU reads advance the ring; sector 0 '
                            'in all = they do not. After the run: after[0x80c] (CPU read 1) and fullmap[1][0x80c/4] '
                            '(CPU read 2) = sectors 4 and 5 if CPU reads advance, equal if not',
                        variant=name, errors=F.RING_ERRORS, layout='0x2040 data; 0x3000+4k: four sequencer 80c reads'),
                   True)


# ---- m2_open_reset_stages (ch17 #25)
STAGE_WORDS = ((0x00c, 0x12345678), (0x010, 0x9abcdef0), (0x014, 0x0101), (0x018, 0x5), (0x01c, 1), (0x02c, 3),
               (0x030, 0x1234), (0x034, 0x08440000), (0x038, 1), (0x044, 0), (0x058, 0x20), (0x05c, 0x200),
               (0x804, 0x304), (0xc40, 1), (0xc44, 1), (0xd00, 0x5a5a0d00), (0xd70, 0x5a5a0d70))
STAGE_ORDERS = (('none', ()), ('stop', (8,)), ('abort', (2,)), ('reset', (0x80000,)), ('stop-reset', (8, 0x80000)),
                ('abort-reset', (2, 0x80000)), ('stop-abort', (8, 2)), ('reset-stop', (0x80000, 8)))


def reset_stages(s, t):
    F._LABEL[0] = 0
    body = ''.join(Q.wr(off, v) for off, v in STAGE_WORDS)
    body += ''.join('    movi r%d,#0x%x\n' % (i, 0x5a5a0000 | i) for i in range(6)) + delay(60000)
    for name, writes in STAGE_ORDERS:
        opts = dict(flags=DISCOVERY | P.FULLMAP, timeout_us=6000)
        if writes:
            opts.update(late=[(0xc00, writes[0])], late_us=1000)
        if len(writes) > 1:
            opts.update(late2=[(0xc00, writes[1])], late2_us=2000)
        yield Case(name, finish(t, body + Q.wr(0xc40, 0) + Q.wr(0xc44, 0)), opts,
                   cond(25, 'words in the after-map/FULLMAP (captured before the probe cleanup) still holding the '
                            'written value were kept by this CPU sequence; zero/default = cleared by its last stage',
                        order=name, writes=[hex(w) for w in writes], written={hex(o): hex(v) for o, v in STAGE_WORDS},
                        gprs='r0..r5 = 0x5a5a000n', loop_ms=4.1))


SPECS = {
    'm2_open_reserved_fields': (reserved_fields, 'fields0,regsweep', 'ch17 #1: non-zero use-0 fields, high source bits, extended-role combinations'),
    'm2_open_low_bus': (low_bus, 'regaddr', 'ch17 #2: ldi of low bus addresses versus the ROM at 0x20000000 (run late)'),
    'm2_open_erased_ops': (erased_ops, 'unknown,opstate', 'ch17 #3: all-ones (erased) words, nop2 in pending states, cost'),
    'm2_open_operand_forms': (operand_forms, 'op10,op15,op08', 'ch17 #4: sig, tmwait (start-word/C14 scaling) and nop operand forms'),
    'm2_open_cycle_timing': (cycle_timing, 'opcost,tmobits', 'ch17 #6: 1..8 copies per loop for fractional costs; WAIT3 fault vs entry delay'),
    'm2_open_c08_states': (c08_states, 'c08self,ctrlwords', 'ch17 #8: C08/C3C/C5C/C14 observed at 3 ms in 18 engine states (run late)'),
    'm2_open_wait_sources': (wait_sources, 'evsweep3', 'ch17 #9: WAIT 6..10, 13..15 after new FMC/ECC states, CPU control words, VIC soft lines'),
    'm2_open_wait11': (wait11, 'evsweep', 'ch17 #10: WAIT11 in in-flight/pending states, operand forms, cost'),
    'm2_open_wait5_history': (wait5_history, 'ev5t', 'ch17 #11: p16 WAIT5 fault programs after each candidate predecessor'),
    'm2_open_evjmp_forms': (evjmp_forms, 'op16,op16b', 'ch17 #12: evjmp operand forms x single/multiple sources; CPU reset orderings (run late)'),
    'm2_open_bank_enable': (bank_enable, 'bank,chan1', 'ch17 #14: +400 words after candidate enable writes'),
    'm2_open_dma_cut': (dma_cut, 'dmactl,dmaloop', 'ch17 #15: superseded DMA cut length vs length, direction and preceding ECC'),
    'm2_open_dma_ctrl': (dma_ctrl, 'dmactl', 'ch17 #16: FMC_DMA_CTRL bits 6..12 written, and 038 timelines (run late)'),
    'm2_open_ecc_collision': (ecc_collision, 'dmactl,eccpar', 'ch17 #17: DMA in/out of the buffer under its own encode: bursts, parity, 048 bit 25'),
    'm2_open_fmc_words': (fmc_words, 'fmcbits', 'ch17 #18: 018/058/05c on DMA loopback; FMC_CTRL0 30:28 sampled timelines'),
    'm2_open_op_words': (op_words, 'fmcbits,fmcstat', 'ch17 #19: FMCTRL1 single bits; 048/04c write masks (run late)'),
    'm2_open_ecc_slots': (ecc_slots, 'eccregs', 'ch17 #21: 01c beyond 0/1 and two-slot arbitration (parity identifies the buffer)'),
    'm2_open_ecc_status_bits': (ecc_status_bits, 'eccregs,rsresult', 'ch17 #22: 810 unassigned bits, 81c values, CPU 80c read proxy'),
    'm2_open_reset_stages': (reset_stages, 'restore,ctrlfx', 'ch17 #25: CPU STOP/abort/clearing-reset orders while running (run late)'),
}

# Ordered least risky first; every family here may leave a state the probe cleanup
# reaches only through its fallbacks (TIMEOUT states, C00 sequences, DMA faults,
# unassigned operation bits, unmapped-looking bus addresses).
RUN_LAST = ('m2_open_c08_states', 'm2_open_dma_ctrl', 'm2_open_op_words', 'm2_open_evjmp_forms',
            'm2_open_reset_stages', 'm2_open_low_bus')
