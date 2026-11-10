"""Round 3 of the M2 open-point families: only what the p37 capture (experiments/m2-p37.json.gz) left open.

One family per open item (keys 1, 3, 4, 6, 8, 9, 10, 11, 14, 15, 16,
18, 22) and per unresolved crosswalk finding (M1-F151, M1-F164). Names m2_r3_<snake>. Every case carries
the conditions `item` (open-point key or finding), `open_point` (what remains after p37), `decision` (the
rule that turns the outcome into an answer) and, where an M1 finding is concerned, `m1`.

What the p37 records show and how each family follows from it:

- #4 tmwait: p37 ended tmwait after 943/3898/31986 us for start counts 1/7/0x40 (and 126103 for 0xff). These
  are 492.747 * count + ~450 us, i.e. exactly one prescaler step (492.7 us) before the WAIT timeout of the
  same count, not the timeout itself and not a constant. m2_r3_tmwait_rule measures tmwait and WAIT3 for
  counts 0..3, with the wait entered after the first lattice tick, and with step fields n = 7 and 9.
- #6 prescaler phase: in p37 phase/00b5 every WAIT3 entered before ~448 us faulted at ~448 us and every
  later one at ~940 us (0x0075: ~17 us, then ~47 us): the fault falls on a fixed tick lattice, so the
  "spreads" are quantisation. A free-running binary prescaler with phase phi at the start predicts the
  first tick at (2^(n+3) - phi mod 2^(n+3)) cycles; 448 us (n = 11) and 16 us (n = 7) both fit phi ~ 1500
  cycles. m2_r3_prescaler_phase measures count-0 faults for n = 4..15, two repeats, a start delayed by
  64 extra parameter writes (does phi move: prescaler running before the start) and the n = 1 step.
- #11 WAIT5: p37 reproduced both p16 faults after a fresh reset (048 before WAIT5 shows no latch flag for
  the one-instruction case), so no predecessor is involved; what remains is the arm rule. m2_r3_latch_gap
  sweeps the gap between an unwaited 70h and the latch in nop2 units with a 048 timeline after the latch.
- #15 DMA cut: every p37 'out' case faulted at the second wait2 (never completed, as M2-114) and the first
  output holds A and B bytes interleaved at the same offsets; the 'in' cases hold the full first input
  with its first 12..32 bytes replaced. No clean cut exists: m2_r3_dma_supersede rewrites one DMA register
  at a time while the first DMA is in flight to find which register the in-flight transfer follows.
- #16 038: p37 timelines pass through the same bits 9..12 sequence (2,4,8,a,e,10..1c) for 64/256/512 bytes
  in both directions, so it is a state sequence, not a byte counter. m2_r3_dma_ctrl_low measures the role
  of the untested low bits 0..2 (values 0..6) and lengths 4/16.
- #18: p37 select058 found the payload in buffer 0 for 058 = 1/0x10/0x100 with 014 = 0, but had no 058 = 0
  control, and 058 reads 0 / 05c 0x100 after sequencer writes. m2_r3_select058 adds the control, 058 = 2
  and 0x20 with distinct buffer contents, and a CPU write of 058/05c read by the sequencer.
- #22 ring: p37 cpu-pre0/1/2 first sequencer reads were sectors 1/2/4: one window snapshot read advances
  the ring once, FULLMAP twice more. m2_r3_ring_alias tests which other offsets (0x90c/0xa0c/0xb0c) are
  ring-advancing aliases of 0x80c and counts the FULLMAP advances alone.
- M1-F151: every p37 C14 read (WAIT3 timeouts for 10 start words, WAIT0 released 1..120 ms in) gave 0x7f.
  m2_r3_c14_reload reads C14 around WAIT0/2/3/4 passing at once and after 1.5 ms (CPU-released), start word
  0xefb5 (bits 15:8 = 0xef, 15:9 = 0x77, both distinct from 0x7f).
- M1-F164: with 0x340 every p37 read-back DMA of buffer 0 returned the 0xee filler (038 left at 0xa07..0x13c7
  mid-sequence), so 'changed 128 words' measured a failed read-back, not the buffer. m2_r3_fifo_readback
  makes the read-back work first (poll 038 idle, rewrite 038, second read-back) with and without writes.
- #1, #3, #8, #9, #10, #14: new operand/state combinations listed in each family docstring.

Conventions (as open_experiments): start word count<<8|n<<4|5; C58 = 4 vectors a fault to C4C, 0 continues;
0x044 = 0 at END and in the fault handler; C60..C6C never written; every changed word restored; NAND
writes only on the scratch geometry and only in cases with MUTATE.
"""
from . import followup_experiments as F, sequences as Q  # noqa: F401
from .protocol import DISCOVERY, MUTATE
from fmss import probe as P  # noqa: F401

Case, finish, memory, delay, sampler, NOP = F.Case, F.finish, F.memory, F.delay, F.sampler, F.NOP
MASK = 0xffffffff
SHORT = 0x07b5                     # unmet WAIT faults at ~4.39 ms
MARK1, MARK2 = '    movi r2,#0x77770001\n', '    movi r2,#0x77770002\n'
SNAP_SKIP_800 = P.SNAP_SKIP[0x800]
ACK2 = Q.wr(0x48, 0x100000)
STEP_US = 492.747                  # prescaler step for n >= 11 (M2-81)
CYCLE_MHZ = 33.25

ITEMS = {
    '1': 'extended destination/store roles outside the p37 set; high source-selector bits of other opcodes',
    '3': 'legality of undefined opcodes 1a..fe in pending operand states',
    '4': 'tmwait times 943/3898/31986 us fit neither the start-word timeout nor a constant',
    '6': 'timeout prescaler phase (p37 fault spreads 23/32 us and 426/508 us)',
    '8': 'C58 bits beyond the tested ones; effect of C48 on tmwait (ledger conflict, one 4391 us fault)',
    '9': 'no source found for WAIT 6/7/8/9/10/13/14/15',
    '10': 'WAIT11 never fails: its condition is not identified',
    '11': 'WAIT5 fault without predecessor (reproduced): arm rule of the status latch after an unwaited 70h',
    '14': '+400 set absent or enabled by a path outside the tested writes',
    '15': 'ambiguous superseded-DMA cuts: interleaved A/B bytes, second DMA never completes',
    '16': 'FMC_DMA_CTRL (038) role of the bits other than 6..12; the 038 sequence versus length',
    '18': 'role of 058 (buffer select with 014 = 0 lacks a 058 = 0 control); 058/05c readback',
    '22': '80c ring CPU-read proxy: FULLMAP advances the ring twice (alias of 0x80c?)',
    'F151': 'which waits load/reload C14 (start-word bits 15:8 versus 0x7f)',
    'F164': 'FMC_FIFO writes after 0x340: the read-back DMA failed after 0x340 in p37',
}

SKIPPED = {
    'open point 2 (PC width, low bus map)': (
        'PC width above 2^20 is NOT_MEASURABLE (open_experiments.NOT_MEASURABLE: a non-wrapping jump runs '
        'unwritten SDRAM without bound); the low-bus map was measured in p37 (6 of 7 offsets classified) and '
        'the point was not in the requested item list.'),
    'open point 12 (evjmp forms, reset orders)': 'settled in p37 (m2_open_evjmp_forms, 68/68 cases).',
    'open point 17 (ECC/DMA collision)': 'settled in p37 (m2_open_ecc_collision, 18/18 cases).',
    'open point 19 (004 operation bits, 048/04c masks)': 'settled in p37 (m2_open_op_words, 37/37 cases).',
    'open point 21 (ECC slot selector)': 'settled in p37 (m2_open_ecc_slots, 11/11 cases).',
    'open point 25 (reset stages)': 'settled in p37 (m2_open_reset_stages, 8/8 cases).',
    'open points 5, 13, 20, 24': 'covered by earlier families (open_experiments.COVERED).',
    'open points 7, 23': 'not measurable by this harness (open_experiments.NOT_MEASURABLE).',
    '#1 sti/jr with high address-selector bits': (
        'if the high selector bits read 0 (as p37 found for mov/add), sti stores to bus address 0 and jr jumps '
        'to offset 0 (program restart, unbounded loop); neither outcome is bounded. ldi/pld/pst cover the '
        'address-selector question with bounded targets.'),
    '#1 extended st roles, mov/movi/add destinations': 'settled (M2-43 258 destination cases, M2-47, M2-141).',
    '#3 erased (all-ones) words': 'settled in p37 (nop2, 1/8/64 words and all pending states).',
    '#4 sig/nop operand forms, consecutive tmwaits': 'settled in p37 and M2-150.',
    '#6 0.25-cycle fractions': 'settled in p37 (all six kinds linear, residual <= 1.1 us).',
    '#8 C08 state codes; C00/C3C/C5C/C14 storage masks': (
        'C08 codes measured in 18 states in p37; storage masks measured by m2_m1_window_matrix (p37). The '
        'functional bits of C00 beyond the start word (M2-153/157/158/201) have no further bounded stimulus.'),
    '#8 END with C58 bits 6/7': (
        'needs END while C58 holds an untested value; the word cannot be restored after END, and M2-148 '
        'already shows no C58 value 0..7/0x12/0x16 diverts an END.'),
    '#14 enables outside the FMSS window (clock gate, pin mux)': (
        'the probe allows CPU writes only to FMSS offsets and the VIC soft registers; m2_r3_bank_decode '
        'instead decides from access cost whether +0x40c is a decoded register slot or undecoded space.'),
    '#16 038 timeline maxima (in)': (
        'settled from the p37 records: the in/out sequences are identical for 64/256/512 bytes (state '
        'sequence, single-sample glitch values 0x3c7/0x847/0x1687 aside); m2_r3_dma_ctrl_low uses that '
        'sequence as its reference instead of repeating it.'),
    '#18 roles of 018 and 05c, FMC_CTRL0 30:28': (
        'no effect in loopback, page read, timing (M2-104, M2-127..139, M2-160, p37 fmc_words): no bounded '
        'observable is left; 018 stores bits 3:0, 05c reads 0x100 whatever is written (p37).'),
    '#22 unassigned 80c bits': 'p37 ring words carry only the error count in bits 16+; no stimulus sets other bits.',
    'other m1diff partially entries (F036, F041, F048, F050, F065, F076, F078, F080, F081, F088, F091, F122, '
    'F133, F174, F175)': 'not in the requested item list; their families are in the m1diff modules.',
}


def cond(item, decision, m1=None, **extra):
    c = dict(item=item, open_point=ITEMS[item], decision=decision, **extra)
    if m1:
        c['m1'] = list(m1)
    return c


def insn(op, reg=0, arg=0, imm=0):
    value = imm if isinstance(imm, str) else '0x%x' % (imm & MASK)
    return '    .insn 0x%x,%d,0x%x,%s\n' % (op, reg, arg, value)


def label(prefix):
    F._LABEL[0] += 1
    return '%s%d' % (prefix, F._LABEL[0])


def seeds():
    return [(0xc18 + 4 * i, 0x11111111 * (i + 1)) for i in range(8)]


SEEDS = [0x11111111 * (i + 1) for i in range(8)]


def stores(regs, base):
    return ''.join(Q.store(off, base + 4 * k) for k, off in enumerate(regs))


def loop(n, body, prefix='lp'):
    lp = label(prefix)
    return '    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n    bnz r4,%s\n' % (n, lp, body, lp)


def trap_with(t, extra):
    return t.trap().replace('trap:\n', 'trap:\n' + extra)


def start_word(count, n=0xb):
    return (count & 0xff) << 8 | (n & 0xf) << 4 | 5


def tick_us(n):
    return (1 << (min(n, 11) + 3)) / CYCLE_MHZ


# ---- m2_r3_reserved_fields (#1)
def reserved_fields(s, t):
    """High source-selector bits of and/or/sub/shl/shr, ldi/pld/pst address selectors, mov selectors
    0x108/0x10a/0x1ff (low byte decoded or zero), ld/ldi/pld/ALU-immediate to r10 (broadcast or discard)
    and ld/pld to r8 (jump or discard)."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    word100 = int(F.F_word(0x100), 16)
    opts = dict(params=seeds() + [(0xd64, 0xcafe0064), (0xd68, 0), (0xd6c, 0), (0xd70, 0x5a5a0d70), (0xd74, 0)],
                scratch=memory(), scratch_read=0x200, flags=DISCOVERY | P.WAIT_END, timeout_us=20000)
    r2, r3, r4 = SEEDS[2], SEEDS[3], SEEDS[4]
    cases = [
        ('mov/src108', insn(6, 3, 8, 0) + insn(6, 2, 0x108, 0), 'r2-pc',
         {'low byte (PC = r3 + 8)': 'pc', 'zero': 0}),
        ('mov/src10a', insn(6, 2, 0x10a, 0), 'r2', {'low byte (selector 10 = destination)': r2, 'zero': 0}),
        ('mov/src1ff', insn(6, 2, 0x1ff, 0), 'r2', {'zero': 0}),
        ('and/src104', insn(0xa, 2, 0x104, 0), 'r2', {'low byte': r2 & r4, 'zero': 0}),
        ('or/src104', insn(0xb, 2, 0x104, 0), 'r2', {'low byte': r2 | r4, 'zero': r2}),
        ('sub/src104', insn(0xd, 2, 0x104, 0), 'r2', {'low byte': (r2 - r4) & MASK, 'zero': r2}),
        ('shl/src104', insn(0x13, 2, 0x104, 0), 'r2', {'low byte (count >= 32)': 0, 'zero': r2}),
        ('shr/src104', insn(0x14, 2, 0x104, 0), 'r2', {'low byte (count >= 32)': 0, 'zero': r2}),
        # Every register holds a valid scratch address with its own marker word, so whichever register a
        # high-bit selector picks, the load stays inside scratch (p38/p40: USB lost in this family).
        ('ldi/sel105', ''.join('    movi r6,#0x%x\n    movi r7,#0x%x\n    sti [r7],r6\n' % (0x5e1000 + i, a + 0x140 + 4 * i)
                               for i in range(8))
         + ''.join('    movi r%d,#0x%x\n' % (i, a + 0x140 + 4 * i) for i in (0, 1, 2, 3, 4, 5, 7))
         + insn(3, 2, 0x105, 0), 'r2',
         dict({'low byte (r5)': 0x5e1005}, **{'register r%d' % i: 0x5e1000 + i for i in (0, 1, 2, 3, 4, 7)})),
        ('pld/sel101', '    movi r1,#0xd64\n' + insn(0x18, 2, 0x101, 0), 'r2',
         {'low byte (PRAM 0xd64)': 0xcafe0064}),
        ('pst/sel101', '    movi r1,#0xd68\n    movi r3,#0x11077801\n' + insn(0x19, 3, 0x101, 0)
         + '    ld r4,[0x0]\n    st [0xd6c],r4\n', 'd68/d6c',
         {'low byte (d68 written)': (0x11077801, None), 'zero (offset 0 = FMC_CTRL0 written)': (0, 0x11077801)}),
    ]
    # r10 destinations (p38: the run was lost here). Every value an r10 write can broadcast is the
    # offset of a landing pad inside this program, so a broadcast that also reaches the PC lands on
    # the pad (0xd74 = 0x7ad) instead of running SDRAM outside the program.
    keep = (0, 2, 3, 4, 5)
    r10 = [
        ('ld/r10', '    wr [0xd70],#{pad}\n', insn(4, 10, 0xd70, MASK), keep),
        ('pld/r10', '    wr [0xd70],#{pad}\n    movi r1,#0xd70\n', insn(0x18, 10, 1, 0), (0, 2, 3, 4)),
        ('ldi/r10', '    movi r1,#{pad}\n    movi r7,#0x%x\n    sti [r7],r1\n    movi r5,#0x%x\n' % (a + 0x100, a + 0x100),
         insn(3, 10, 5, 0), (0, 2, 3, 4)),
        ('sub/r10-imm1', '    movi r0,#{pad}\n    add r0,r0,#1\n', insn(0xd, 10, 0, 1), (2, 3, 4, 5)),
    ]
    for name, setup, ins, untouched in r10:
        pad, skip = label('pad'), label('skip')
        body = (setup.format(pad=pad) + ins + '    movi r6,#1\n    bnz r6,%s\n%s:\n    wr [0xd74],#0x7ad\n%s:\n'
                % (skip, pad, skip))
        cases.append((name, body, 'gprs', {'broadcast': 'equal', 'discard': {i: SEEDS[i] for i in untouched}}))
    for name, op, reg in (('ld/r8', 4, None), ('pld/r8', 0x18, 1)):
        tgt = label('tg')
        pre = '    wr [0xd70],#%s\n' % tgt + ('    movi r1,#0xd70\n' if reg else '')
        ins = insn(4, 8, 0xd70, MASK) if op == 4 else insn(0x18, 8, 1, 0)
        cases.append((name, pre + ins + '    movi r2,#0xbad\n%s:\n    st [0xd74],r2\n' % tgt, 'd74',
                      {'jump (PC destination)': r2, 'no jump (discard)': 0xbad}))
    for name, body, observe, predict in cases:
        yield Case(name, finish(t, body), dict(opts),
                   cond('1', 'the hypothesis whose prediction matches the observed value names the field role; '
                             'no match = another role (value recorded)', observe=observe,
                        predict={k: (v if isinstance(v, (str, dict)) else list(v) if isinstance(v, tuple) else v)
                                 for k, v in predict.items()},
                        seeds='r0..r7 = 0x11111111*(n+1)', instruction=body.strip()))


# ---- m2_r3_undefined_pending (#3)
UNDEFINED_OPS = (0x1a, 0x80, 0xfe)
PENDING = ('command', 'address', 'DMA', 'ECC', 'status')


def undefined_pending(s, t):
    """Undefined opcodes 1a/80/fe with a pending command, address, DMA, ECC or status flag (p37 scan: idle only)."""
    from .discovery import states
    F._LABEL[0] = 0
    g, actions = Q.geometry(s), states(s, t)
    for op in UNDEFINED_OPS:
        for state in PENDING:
            pins = state in ('command', 'address', 'status')
            body = Q.setup(g.ce if pins else None) + actions[state] + MARK1 + insn(op) + MARK2 + '    ld r4,[0x48]\n'
            yield Case('op%02x/%s' % (op, state), finish(t, body),
                       dict(start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                       cond('3', 'fault at the SHORT wait timeout (%.0f us) with r2 0x77770001 = unmet wait as when '
                                 'idle; r2 0x77770002 = the opcode passes on this pending state' % (
                                     STEP_US * 8 + 447.5), opcode=hex(op), state=state), pins)


# ---- m2_r3_tmwait_rule (#4)
TM_COUNTS = (0, 1, 2, 3)
LATE_LOOPS = 7000                  # ~474 us at 67.7 ns per delay loop: after the first lattice tick (~448 us)


def tmwait_rule(s, t):
    """tmwait and WAIT3 for counts 0..3 (n = 11), entered after the first tick, and for n = 7 / 9 (count 1)."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    tm = MARK1 + insn(0x15) + MARK2 + Q.store(0xc14, a + 0x2000)
    w3 = MARK1 + Q.wait(3) + MARK2
    plan = [('tmwait/c%d' % c, tm, start_word(c), 0) for c in TM_COUNTS]
    plan += [('wait3/c%d' % c, w3, start_word(c), 0) for c in TM_COUNTS]
    plan += [('tmwait/c0/late', tm, start_word(0), LATE_LOOPS), ('tmwait/c1/late', tm, start_word(1), LATE_LOOPS),
             ('wait3/c1/late', w3, start_word(1), LATE_LOOPS)]
    for n in (7, 9):
        plan += [('tmwait/n%d/c1' % n, tm, start_word(1, n), 0), ('wait3/n%d/c1' % n, w3, start_word(1, n), 0)]
    for name, body, start, d in plan:
        yield Case(name, finish(t, (delay(d) if d else '') + body),
                   dict(start_value=start, scratch=memory(), scratch_read=0x2004, flags=DISCOVERY | P.WAIT_END,
                        timeout_us=50000),
                   cond('4', 'tmwait end / WAIT3 fault on the tick lattice t_k = t_0 + k*step: one step per count = '
                             'counts prescaler ticks; WAIT - tmwait = one step at every count = the WAIT timeout is '
                             'one tick later; late entry moving the end by one step = counts ticks after entry, by '
                             '0 = counts from the start, by the entry delay = counts time',
                        start_word=hex(start), count=(start >> 8) & 0xff, n=(start >> 4) & 0xf,
                        kind=name.split('/')[0], entry_loops=d, loop_ns=67.7, step_us=round(tick_us((start >> 4) & 0xf), 3),
                        p37={'tmwait': {'0x01b5': 943, '0x07b5': 3898, '0x40b5': 31986, '0xffb5': 126103}}))


# ---- m2_r3_prescaler_phase (#6)
PHASE_N = tuple(range(4, 16))
PAD_PARAMS = [(0xd00 + 4 * (k % 32), 0) for k in range(64)]


def prescaler_phase(s, t):
    """Count-0 WAIT3 fault for every step field n (first prescaler tick), repeats, a delayed start, n = 1 step."""
    F._LABEL[0] = 0
    body = MARK1 + Q.wait(3) + MARK2
    plan = [('n%x' % n, start_word(0, n), []) for n in PHASE_N]
    plan += [('n7/rep1', start_word(0, 7), []), ('nb/rep1', start_word(0, 0xb), [])]
    plan += [('n9/pad64', start_word(0, 9), PAD_PARAMS), ('nb/pad64', start_word(0, 0xb), PAD_PARAMS)]
    plan += [('n1/c%02x' % c, start_word(c, 1), []) for c in (0x40, 0x80, 0xff)]
    for name, start, params in plan:
        n = (start >> 4) & 0xf
        yield Case(name, finish(t, body),
                   dict(start_value=start, params=list(params), flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   cond('6', 'count 0 faults at the first prescaler tick: a single phase phi (cycles) with fault = '
                             '(2^(n+3) - phi mod 2^(n+3)) / 33.25 MHz for every n = free-running binary prescaler; '
                             'pad64 moving the tick = the prescaler runs before the start (phase set earlier), '
                             'unchanged = it restarts at the start; n = 1 counts give its step',
                        start_word=hex(start), n=n, count=(start >> 8) & 0xff, extra_params=len(params),
                        p37={'00b5': 'faults at ~448 us for entry <= 406 us, ~940 us after 474 us',
                             '0075': 'faults at ~17 us for entry <= 13.5 us, ~47 us after 20 us'}))


# ---- m2_r3_control_words (#8)
C48_VALUES = (0, MASK, 0x55555555)
C58_BITS = (0x40, 0x80, 0xc0, 0x44, 0x84)


def control_words(s, t):
    """tmwait repeated 5x per C48 value (ledger conflict M2-179); C58 bits 6/7 alone and with bit 2 on an
    unmet WAIT3 (stored mask 0xd6; bits 6/7 never set in a test)."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for v in C48_VALUES:
        for rep in range(5):
            body = MARK1 + insn(0x15) + MARK2 + Q.wr(0xc48, 0)
            src = t.prologue(0) + body + Q.wr(0, 0) + '    movi r6,#0x600d600d\n    end\n' + trap_with(t, Q.wr(0xc48, 0))
            yield Case('c48/%x/rep%d' % (v, rep), src,
                       dict(start_value=SHORT, params=[(0xc48, v)], flags=P.WAIT_END, timeout_us=50000),
                       cond('8', 'all 5 repeats END at the C48 = 0 time = C48 has no effect on tmwait; a fault in '
                                 'some repeats only = intermittent (not C48); faults in all = C48 effect',
                            m1=('M1-F064',), c48=hex(v), rep=rep,
                            conflict='p36: one tmwait with C48 = 0xffffffff faulted at 4391 us, nine presets ended at '
                                     '3898..3900 us'))
    snap = stores((0xc5c, 0xc08, 0xc58, 0xc0c), a + 0x2000)
    for v in C58_BITS:
        body = MARK1 + Q.wait(3) + MARK2 + snap + Q.wr(0xc58, 4)
        handler = trap_with(t, stores((0xc5c, 0xc08, 0xc58, 0xc0c), a + 0x2010) + Q.wr(0xc58, 4))
        yield Case('c58/%02x' % v, finish(t, body, v, handler),
                   dict(start_value=SHORT, scratch=memory(), scratch_read=0x2020, flags=DISCOVERY | P.WAIT_END,
                        timeout_us=50000),
                   cond('8', 'trap (r6 0xfa17fa17) = timeout vectored, r2 0x77770002 END = continued, anything else = '
                             'bit 6/7 changes the fault path; C58 readback shows whether the bit is stored',
                        c58=hex(v), layout='0x2000: C5C C08 C58 C0C after continue; 0x2010: same in the handler'))


# ---- m2_r3_wait_sources (#9)
R3_WAITS = (6, 7, 8, 9, 10, 13, 14, 15)


def wait_sources(s, t):
    """WAIT 6..10/13..15 after stimuli p37 did not use: sig pending in CS_IRQ, ECC decode completion of an
    uncorrectable and of an erased (all 0xff) buffer, 0x044 = ones written by the CPU before the start."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mem = memory()
    mem[0x3000:0x3200] = b'\xff' * 0x200
    decode = Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 1) + Q.wait(4)
    stimuli = {
        'sig-pending': (Q.setup() + insn(0x10), [], False),
        'decode-uncorrectable': (Q.setup() + Q.dma(a, 0) + decode, [], False),
        'decode-erased': (Q.setup() + Q.dma(a + 0x3000, 0) + decode, [], False),
        'irq044-cpu-command': (Q.setup(g.ce) + Q.wr(8, 0x70), [(0x44, MASK)], True),
    }
    restore = Q.wr(0x44, 0) + Q.wr(0x810, 0x1ff)
    for name, (prep, params, pins) in stimuli.items():
        for n in R3_WAITS:
            body = prep + stores((0x48, 0x810, 0xc0c), a + 0x2000) + MARK1 + Q.wait(n) + MARK2 + restore
            yield Case('%s/wait%02d' % (name, n), finish(t, body, 4, trap_with(t, restore)),
                       dict(scratch=mem, scratch_read=0x2010, start_value=SHORT, params=list(params),
                            flags=DISCOVERY, timeout_us=50000),
                       cond('9', 'r2 0x77770002 before the 4.4 ms fault = WAIT %d released by %s; fault = not a source'
                                 % (n, name), stimulus=name, wait=n, layout='0x2000: 048, 810, C0C before the WAIT'),
                       pins)


# ---- m2_r3_wait11_busy (#10)
def wait11_busy(s, t):
    """WAIT11 repeated through NAND array busy periods (tR, FFh reset, block erase on the scratch block) and the
    cost of a passing WAIT0 (flag pending, not consumed) against WAIT11's 37.7 ns (p37)."""
    F._LABEL[0] = 0
    g = Q.geometry(s)
    w11 = Q.wait(11)
    variants = {
        'tR-busy': (Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + loop(1000, w11, 'w') +
                    Q.ready() + Q.command(0), 1000, 0),
        'reset-busy': (Q.setup(g.ce) + Q.command(0xff) + loop(1000, w11, 'w') + Q.ready(), 1000, 0),
        'erase-busy': (Q.setup(g.ce) + Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0)
                       + loop(30000, w11, 'w') + Q.ready() + '    ld r5,[0x4c]\n', 30000, MUTATE),
    }
    for name, (body, n, mutate) in variants.items():
        yield Case(name, finish(t, body), dict(start_value=SHORT, flags=DISCOVERY | mutate, timeout_us=200000),
                   cond('10', 'END = WAIT11 true throughout the busy period; a fault = WAIT11 follows the array '
                              'ready line (failing iteration = %d - r4)' % n, state=name, iterations=n,
                        block=g.block if mutate else None, ce=g.ce), True)
    for n in (1000, 21000):
        body = Q.setup(g.ce) + Q.wr(8, 0x70) + Q.wait(0) + loop(n, Q.wait(0), 'c') + Q.wr(0x48, 2)
        yield Case('cost/wait0-n%d' % n, finish(t, body), dict(start_value=SHORT, flags=DISCOVERY, timeout_us=200000),
                   cond('10', 'per-WAIT0 cost (passing) from the 1000/21000 difference minus the empty loop; equal to '
                              'WAIT11 (37.7 ns, p37) = WAIT11 is an ordinary wait whose condition is true',
                        iterations=n), True)


# ---- m2_r3_latch_gap (#11)
LATCH_GAPS = (0, 1, 2, 3, 4, 6, 8, 12, 16, 20)


def latch_gap(s, t):
    """Gap between an unwaited 70h and the status latch: nop2 x k, one nop (1.25 cycles), one movi (x3),
    an acknowledge with 0/20 nop2 after it; 048 timeline after the latch."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    c, latch = Q.wr(8, 0x70), Q.wr(4, 0xca)
    plan = [('nop2/%02d' % k, NOP * k) for k in LATCH_GAPS]
    plan += [('nop08/1', insn(8))] + [('movi/1/rep%d' % k, '    movi r5,#0\n') for k in range(3)]
    plan += [('ack/gap00', Q.wr(0x48, 2)), ('ack/gap20', Q.wr(0x48, 2) + NOP * 20)]
    tail = Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800002)
    for name, gap in plan:
        body = (Q.setup(g.ce) + '    movi r7,#0x%x\n' % (a + 0x2000) + c + gap + latch + sampler(0x48, 24, 0)
                + '    ld r2,[0x48]\n' + Q.wait(5) + '    ld r3,[0x48]\n    ld r4,[0x4c]\n' + tail)
        yield Case(name, finish(t, body), dict(scratch=memory(), scratch_read=0x2060, start_value=SHORT,
                                               timeout_us=50000),
                   cond('11', 'WAIT5 pass/fault per gap; 048 bits 23/24 (0x1800000) rising in the timeline = the '
                              'latch ran; a fault exactly where the latch flag never rises = the latch was refused '
                              '(arm window), a fault with the flag = WAIT5 condition', gap=name,
                        reference='p16/p37: gap 0 passes, movi or ack faults, 20/200 instructions pass (M2-68)',
                        layout='0x2000 + 4k: 048 sample k after the latch (24); r2 048 before WAIT5, r3 after, r4 04c'),
                   True)


# ---- m2_r3_bank_decode (#14)
DECODE_TARGETS = (0x00c, 0x40c, 0x60c)


def bank_decode(s, t):
    """Sequencer ld cost of a primary FMC word (00c), the +400 word (40c) and an unused offset (60c)."""
    F._LABEL[0] = 0
    for off in DECODE_TARGETS:
        for n in (1000, 21000):
            yield Case('ld%03x/n%d' % (off, n), finish(t, loop(n, '    ld r1,[0x%x]\n' % off, 'bd')),
                       dict(flags=DISCOVERY, timeout_us=200000),
                       cond('14', 'cost(40c) = cost(00c) != cost(60c) = decoded register slot (set present, not '
                                  'enabled); cost(40c) = cost(60c) != cost(00c) = undecoded (absent); all equal = '
                                  'timing cannot separate them', target=hex(off), iterations=n))


# ---- m2_r3_dma_supersede (#15)
SUPERSEDE = ('none', 'addr', 'count', 'select', 'ctrl', 'kick', 'all')


def dma_supersede(s, t):
    """One buffer-to-RAM DMA (A, buffer 0 -> 0x2000) in flight, then a single register rewritten (or the full
    second DMA without its wait); wait 2 continues on timeout (C58 = 0); 0x2000 and 0x2400 compared with A/B."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    rewrite = {'none': '', 'addr': Q.wr(0x34, a + 0x2400), 'count': Q.wr(0x30, 63), 'select': Q.wr(0x14, 0x200),
               'ctrl': Q.wr(0x38, 7), 'kick': Q.wr(4, 0x100),
               'all': Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x200) + Q.wr(0x34, a + 0x2400) + Q.wr(4, 0x100)}
    snap = lambda base: stores((0x30, 0x34, 0x14, 0x38, 0x48), base)  # noqa: E731
    for gap in (0, 16):
        for var in SUPERSEDE:
            body = Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1)
            body += Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
            body += NOP * gap + rewrite[var] + snap(a + 0x2800) + Q.wait(2) + snap(a + 0x2820) + ACK2 + delay(300)
            body += snap(a + 0x2840)
            yield Case('out/%s/gap%02d' % (var, gap), finish(t, body, 0),
                       dict(scratch=memory(), scratch_read=0x2860, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                       cond('15', 'per byte of 0x2000/0x2400: A (first source) / B (buffer 1) at the same offset / '
                                  'initial / other; a rewrite that changes the in-flight output relative to "none" = '
                                  'the transfer follows that register live', direction='out', rewrite=var, gap_nops=gap,
                            layout='0x2000 first destination, 0x2400 second; 0x2800/0x2820/0x2840: 030 034 014 038 048 '
                                   'after the rewrite, after wait 2, after the ack'))
    for var in ('none', 'addr', 'select', 'all'):
        rw = {'none': '', 'addr': Q.wr(0x34, a + 0x400), 'select': Q.wr(0x14, 2),
              'all': Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 2) + Q.wr(0x34, a + 0x400) + Q.wr(4, 0x2e0)}[var]
        body = Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(0x34, a) + Q.wr(4, 0x2e0)
        body += rw + snap(a + 0x2800) + Q.wait(2) + snap(a + 0x2820) + ACK2 + delay(300)
        body += Q.dma(a + 0x2000, 0, into=False) + Q.dma(a + 0x2400, 1, into=False) + snap(a + 0x2840)
        yield Case('in/%s/gap00' % var, finish(t, body, 0),
                   dict(scratch=memory(), scratch_read=0x2860, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('15', 'buffer 0 (read back at 0x2000) and buffer 1 (0x2400) against A (RAM 0) and B (RAM '
                              '0x400): which buffer/source the in-flight RAM-to-buffer DMA follows', direction='in',
                        rewrite=var, gap_nops=0, layout='as out; buffers read back by serial DMAs'))


# ---- m2_r3_dma_ctrl_low (#16)
P37_SEQUENCE = (0x1, 0x2, 0x4, 0x5, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe)   # (038 >> 9) states, p37 in


def dma_ctrl_low(s, t):
    """038 low bits 0..2 (values 0..6) on a 512-byte RAM-to-buffer DMA with a 038 timeline and payload check;
    038 = 7 at lengths 4 and 16 (in) and 4 (out) against the p37 length-independent sequence."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    plan = [('in/v%d/len512' % v, v, 512, True) for v in range(7)]
    plan += [('in/v7/len4', 7, 4, True), ('in/v7/len16', 7, 16, True), ('out/v7/len4', 7, 4, False)]
    for name, v, length, into in plan:
        r7 = '    movi r7,#0x%x\n' % (a + 0x2000)
        if into:
            body = (Q.setup() + Q.wr(0x38, v) + Q.wr(0x30, length - 1) + Q.wr(0x14, 1) + Q.wr(0x34, a) + r7
                    + Q.wr(4, 0x2e0) + sampler(0x38, 48, 0) + Q.wait(2) + stores((0x48, 0x38), a + 0x2200) + ACK2
                    + Q.wr(0x38, 7) + Q.dma(a + 0x2400, 0, into=False))
        else:
            body = (Q.setup() + Q.dma(a, 0) + Q.wr(0x38, v) + Q.wr(0x30, length - 1) + Q.wr(0x14, 0x100)
                    + Q.wr(0x34, a + 0x2400) + r7 + Q.wr(4, 0x100) + sampler(0x38, 48, 0) + Q.wait(2)
                    + stores((0x48, 0x38), a + 0x2200) + ACK2 + Q.wr(0x38, 7))
        yield Case(name, finish(t, body, 0),
                   dict(scratch=memory(), scratch_read=0x2600, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond('16', 'payload at 0x2400 equal to A (first %d bytes) = transfer works with this 038; '
                              'the (038 >> 9) sequence of the timeline equal to the p37 sequence for every value and '
                              'length = state sequence independent of length and low bits; a shorter/longer or '
                              'stepped sequence names the low bits as burst/width' % length,
                        value=v, length=length, direction='in' if into else 'out', samples=48, sample_ns=188.0,
                        p37_sequence=[hex(x) for x in P37_SEQUENCE],
                        layout='0x2000 + 4k: 038 sample k; 0x2200: 048 038 after wait 2; 0x2400 payload'))


# ---- m2_r3_select058 (#18)
SELECT058 = (0, 1, 2, 0x10, 0x20)


def select058(s, t):
    """DMA in of C (RAM 0x800) with 014 = 0 and 058 = 0/1/2/0x10/0x20 after buffers 0/1 were loaded with A/B;
    CPU-written 058/05c read by the sequencer."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for v in SELECT058:
        body = (Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1) + Q.wr(0x58, v) + Q.dma(a + 0x800, 0, select=0)
                + stores((0x48, 0x38), a + 0x2800) + ACK2 + Q.wr(0x58, 0) + Q.wr(0x38, 7)
                + Q.dma(a + 0x2000, 0, into=False) + Q.dma(a + 0x2400, 1, into=False))
        yield Case('in/058-%x' % v, finish(t, body, 0),
                   dict(scratch=memory(), scratch_read=0x2808, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('18', 'the buffer read back holding C (RAM 0x800) is the one this 058 selects with 014 = 0; '
                              'buffer 0 for 058 = 0 too = 014 = 0 defaults to buffer 0 (058 inert); none = no target',
                        value=hex(v), layout='0x2000 buffer 0, 0x2400 buffer 1 read back; 0x2800 048 038'))
    body = stores((0x58, 0x5c), a + 0x2800) + Q.wr(0x58, 0) + Q.wr(0x5c, 0x100)
    yield Case('cpu-write/058-05c', finish(t, body),
               dict(params=[(0x58, 0x10), (0x5c, 0x200)], scratch=memory(), scratch_read=0x2808, timeout_us=20000),
               cond('18', 'sequencer reads of CPU-written 058 = 0x10 / 05c = 0x200: the written values = storage seen '
                          'by both sides (p37 sequencer writes read 0 / 0x100); 0 / 0x100 = the words do not store',
                    layout='0x2800: 058 05c'))


# ---- m2_r3_ring_alias (#22)
RING_ALIASES = (None, 0x80c, 0x90c, 0xa0c, 0xb0c, 0x810)


def ring_alias(s, t):
    """Ring-advancing aliases of 0x80c: one sequencer read of 0x90c/0xa0c/0xb0c (0x80c positive, 0x810 negative
    control) after an 8-sector decode, then four 0x80c reads; FULLMAP alone (no window read of 0x800..)."""
    import hashlib
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, _ = F.coded_page(F.RING_ERRORS)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), cond('22', 'scratch block erase', block=g.block), True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, F.BCH_META, raw=True)),
               dict(mutate, scratch=mem, scratch_read=0),
               cond('22', 'host code words with 1..8 errors per sector', errors=F.RING_ERRORS,
                    raw_sha256=hashlib.sha256(raw).hexdigest()), True)
    reads = ''.join(Q.store(0x80c, a + 0x3004 + 4 * k) for k in range(4))
    plan = [('alias/%s' % ('none' if x is None else '%03x' % x), x, SNAP_SKIP_800) for x in RING_ALIASES]
    plan += [('fullmap', None, SNAP_SKIP_800 | P.FULLMAP)]
    for name, x, flags in plan:
        body = F.decode_loop(g, g.row, a + 0x2040, 8, 8, lambda sec, slot: '')
        body += (Q.store(x, a + 0x3000) if x is not None else '') + reads
        yield Case(name, finish(t, body), dict(scratch=memory(), scratch_read=0x3014, timeout_us=300000, flags=flags),
                   cond('22', 'first 0x80c read (0x3004) = result of sector k (errors k+1 in bits 16+): k = 1 after '
                              'the extra read = that offset advances the ring (alias); k = 0 = it does not; fullmap: '
                              'k = number of ring reads made by the FULLMAP pre-dump',
                        alias=None if x is None else hex(x), fullmap=bool(flags & P.FULLMAP), errors=F.RING_ERRORS,
                        p37='window read +1, window + FULLMAP +3',
                        layout='0x2040 data; 0x3000 extra read; 0x3004 + 4k: four 0x80c reads'), True)


# ---- m2_r3_c14_reload (M1-F151)
C14_START = 0xefb5                 # bits 15:8 = 0xef, 15:9 = 0x77, default 0x7f


def c14_reload(s, t):
    """C14 read by the sequencer before, right after and 1 ms after WAIT0/2/3/4 passing at once or released
    1.5 ms in by a CPU write; CPU read after END in the after-map."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    rd = lambda r: '    ld r%d,[0xc14]\n' % r  # noqa: E731
    post = '    ld r4,[0x48]\n    st [0xd00],r1\n    st [0xd04],r2\n    st [0xd08],r3\n    st [0xd0c],r4\n'
    dma_regs = Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(0x34, a)
    readid = (Q.setup(g.ce) + Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1)
              + Q.wr(0x48, 4) + Q.wr(0x30, 7) + Q.wr(0x14, 0x10))
    ms = lambda: delay(15000)  # noqa: E731
    plan = {
        'control': (Q.setup() + rd(1) + ms() + rd(2) + ms() + rd(3), None, False),
        'wait0/now': (Q.setup(g.ce) + rd(1) + Q.wr(8, 0x70) + Q.wait(0) + rd(2) + Q.wr(0x48, 2) + ms() + rd(3), None, True),
        'wait0/late': (Q.setup(g.ce) + rd(1) + Q.wait(0) + rd(2) + Q.wr(0x48, 2) + ms() + rd(3), (8, 0x70), True),
        'wait2/now': (Q.setup() + dma_regs + rd(1) + Q.wr(4, 0x2e0) + Q.wait(2) + rd(2) + ACK2 + ms() + rd(3), None, False),
        'wait2/late': (Q.setup() + dma_regs + rd(1) + Q.wait(2) + rd(2) + ACK2 + ms() + rd(3), (4, 0x2e0), False),
        'wait3/now': (readid + rd(1) + Q.wr(4, 0xe2) + Q.wait(3) + rd(2) + Q.wr(0x48, 8) + ms() + rd(3), None, True),
        'wait3/late': (readid + rd(1) + Q.wait(3) + rd(2) + Q.wr(0x48, 8) + ms() + rd(3), (4, 0xe2), True),
        'wait4/now': (Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + rd(1) + Q.wr(0x804, 2) + Q.wait(4)
                      + rd(2) + Q.wr(0x810, 1) + ms() + rd(3), None, False),
        'wait4/late': (Q.setup() + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + rd(1) + Q.wait(4) + rd(2)
                       + Q.wr(0x810, 1) + ms() + rd(3), (0x804, 2), False),
    }
    for name, (body, late, pins) in plan.items():
        opts = dict(start_value=C14_START, params=[(0xd00 + 4 * k, 0) for k in range(4)], flags=P.WAIT_END,
                    timeout_us=200000, scratch=memory(), scratch_read=16)
        if late:
            opts.update(late=[late], late_us=1500)
        yield Case(name, finish(t, body + post, 0), opts,
                   cond('F151', 'C14 = 0xef (start bits 15:8) or 0x77 (15:9) after a wait = that wait loads C14; 0x7f '
                                'everywhere = no tested wait loads it; a value falling between the right-after and the '
                                '1 ms read = it counts', m1=('M1-F151',), start_word=hex(C14_START),
                        wait=name.split('/')[0], release=('CPU write %s = %s at 1500 us' % (hex(late[0]), hex(late[1]))
                                                          if late else 'at once'),
                        layout='0xd00 C14 before, 0xd04 right after the wait, 0xd08 ~1 ms later, 0xd0c 048; '
                               'after[0xc14] = CPU read after END'), pins)


# ---- m2_r3_fifo_readback (M1-F164)
def fifo_readback(s, t):
    """FMCTRL1 0x340 (sunk, acknowledged), then a read-back of buffer 0 that works: wait for 038 bits 7..12 to
    clear, rewrite 038 = 7, or a second read-back after a delay; with 0 and 128 FIFO writes."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    poll = ('    movi r4,#2000\npl:\n    ld r1,[0x38],0x1f80\n    bnz r1,pw\n    movi r4,#1\npw:\n'
            '    sub r4,r4,#1\n    bnz r4,pl\n    movi r7,#0x%x\n    sti [r7],r1\n' % (a + 0x2c10))
    methods = {'poll': poll, 'rewrite038': Q.wr(0x38, 7), 'twice': ''}
    for method, fix in methods.items():
        for writes in (0, 128):
            body = Q.setup() + Q.dma(a, 0) + Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(0x34, a + 0x2800)
            body += Q.wr(4, 0x340) + Q.wait(2) + ACK2 + Q.store(0x38, a + 0x2c00) + fix + Q.store(0x38, a + 0x2c04)
            body += ''.join(Q.wr(0x80, 0x5a000000 + k) for k in range(writes))
            body += Q.dma(a + 0x2400, 0, into=False) + stores((0x38, 0x48), a + 0x2c08)
            if method == 'twice':
                body += delay(3000) + Q.dma(a + 0x2600, 0, into=False) + Q.store(0x38, a + 0x2c14)
            yield Case('%s/w%d' % (method, writes), finish(t, body, 0),
                       dict(scratch=memory(), scratch_read=0x2c20, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                       cond('F164', 'w0 read-back equal to A = the read-back works and 0x340 leaves buffer 0 unchanged; '
                                    'w128 equal to w0 = FIFO writes do not reach buffer 0 (differs from M1: 26 of 128 '
                                    'words changed); changed words holding 0x5a000000+k = writes land in buffer 0',
                            m1=('M1-F164',), method=method, writes=writes,
                            p37='every 0x340 read-back held the 0xee/initial fill (038 0xa07..0x13c7 left mid-sequence)',
                            layout='0x2400 buffer 0 read-back (0x2600 second for twice); 0x2800 sink; 0x2c00: 038 after '
                                   'ack, after the fix, 038 048 after the read-back, poll result 0x2c10, 038 0x2c14'))


SPECS = {
    'm2_r3_reserved_fields': (reserved_fields, 'fields0,regsweep', '#1: high selector bits of and/or/sub/shl/shr/ldi/pld/pst; r10/r8 destinations of ld/ldi/pld/sub (run late)'),
    'm2_r3_undefined_pending': (undefined_pending, 'unknown,opstate', '#3: undefined opcodes 1a/80/fe with pending command/address/DMA/ECC/status'),
    'm2_r3_tmwait_rule': (tmwait_rule, 'op15,tmobits', '#4: tmwait vs WAIT3 for counts 0..3, late entry, n = 7/9'),
    'm2_r3_prescaler_phase': (prescaler_phase, 'tmobits', '#6: count-0 first tick for n = 4..15, repeats, delayed start, n = 1 step'),
    'm2_r3_control_words': (control_words, 'ctrlwords', '#8: tmwait x5 per C48 value; C58 bits 6/7 on an unmet wait (run late)'),
    'm2_r3_wait_sources': (wait_sources, 'evsweep3', '#9: WAIT 6..10/13..15 after sig, ECC decode (uncorrectable/erased), CPU 0x044'),
    'm2_r3_wait11_busy': (wait11_busy, 'evsweep', '#10: WAIT11 through tR/reset/erase busy; passing WAIT0 cost'),
    'm2_r3_latch_gap': (latch_gap, 'ev5t', '#11: WAIT5 vs the gap between an unwaited 70h and the latch'),
    'm2_r3_bank_decode': (bank_decode, 'bank', '#14: ld cost of 00c / 40c / 60c'),
    'm2_r3_dma_supersede': (dma_supersede, 'dmactl,dmaloop', '#15: single-register rewrites during an in-flight DMA (run late)'),
    'm2_r3_dma_ctrl_low': (dma_ctrl_low, 'dmactl', '#16: 038 values 0..6 and short lengths, timelines (run late)'),
    'm2_r3_select058': (select058, 'fmcbits', '#18: 058 buffer select with 014 = 0 and its control; CPU-written 058/05c'),
    'm2_r3_ring_alias': (ring_alias, 'eccregs,rsresult', '#22: ring-advancing aliases of 0x80c; FULLMAP ring reads'),
    'm2_r3_c14_reload': (c14_reload, 'tmobits,startbits', 'M1-F151: C14 around WAIT0/2/3/4 passing at once and late'),
    'm2_r3_fifo_readback': (fifo_readback, 'fifo', 'M1-F164: FIFO writes after 0x340 with a working read-back (run late)'),
}

# Overlapping DMAs (P25: may wedge), 0x340 leaving the DMA mid-sequence, 038 values that fault the DMA,
# pst/ldi through unknown selectors and untested C58 bits come last, least risky first.
RUN_LAST = ('m2_r3_control_words', 'm2_r3_reserved_fields', 'm2_r3_dma_ctrl_low', 'm2_r3_fifo_readback',
            'm2_r3_dma_supersede')
