"""Round 4 of the M2 open-point families: one consolidated round over what chapter 17 still leaves open.

Sources:
chapter 19 (M1-F164 unresolved; M1-F059 has its own reboot tests and is skipped), the established facts of
the p38-p39 measurements (experiment key m2_r4_fifo_page_readback).
Names m2_r4_<snake>. Every case carries the conditions `item` (chapter-17 item or M1 ID), `key` (the
analyser point, one per family), `open_point` (what remains), `decision` (the rule that turns the outcome
into an answer) and, where an M1 finding is concerned, `m1`. The round rules classify each result.

Families and the question each settles:

- m2_r4_bus_latch (#1): does a register-form source selector with bits 8..15 set read 0 or the register-bus
  latch? Nine predecessors with distinct latch values x selectors 0x104/0x108/0x1ff; ALU forms; whether a
  discarded extended-destination write passes the bus (the extended `st` stores the latch, M2-47).
- m2_r4_undefined_scan (#3): every undefined opcode 1b..fd (80 done) with a pending command, address, DMA,
  ECC or status flag (only 1a/80/fe were run in pending states, M2-583).
- m2_r4_nop_operand (#4): `nop` (08) for every R = 0..255 and R = 8..15 with each pending flag: the stall
  map and whether a pending flag releases the stall (R as an event selector).
- m2_r4_c48_bits (#4): tmwait with C48 = each single bit, 0, ones, 0x55555555, 0xaaaaaaaa: which bits fault.
- m2_r4_c14_write (#4): program writes of C14 before tmwait (values, counts 1/16/255, repeats): bit-specific
  shift, rate change, a write-triggered phase change, or run-to-run spread (the 24 us at 0x1000, M2-584).
- m2_r4_prescaler_origin (#6): count-0 first instants with repeats, 30/60/90 extra parameter writes and
  without the per-case reset: which event sets the prescaler phase; n = 4..6 repeats.
- m2_r4_c08_running (#8): C08 read 3 ms into a loop of a single instruction kind (13 kinds x 3): whether the
  running codes 1/2/9 encode the executing instruction.
- m2_r4_c58_bits (#8): C58 bits 1 and 4 (alone and with bit 2) on the timeout paths of an unmet wait, a
  count-0 tmwait and an undefined opcode.
- m2_r4_c14_release (#8): C14 after a WAIT0 released by the CPU after 0..4 prescaler steps for counts
  3/0x10/0x40 and n = 9: whether 0x7f is a constant reload or a function of count/steps.
- m2_r4_latch_refusal (#11): which operations an unreleased status latch refuses (DMA in/out, data read,
  parity, FIFO read, ECC, second latch; command/address as known controls), each against a released control.
- m2_r4_status_bits (#19): FMC_STATUS bits 19:12 after data reads of 1..512 bytes, parity reads, DMAs.
- m2_r4_data_sel_alias (#19): the 0x550d readback (M2-83 p18): does a later 058 (or 018/05c) write alter
  014 bits 7:0? Both writers, both orders, the p18 order with CPU params and late writes.
- m2_r4_reset_steps (#26): FMC 00c..05c and ECC_CTRL after FMCTRL1 0x800, 0x80000000, both, 0x81c = 1 and
  combinations, written by the CPU and by the sequencer (C60/C64 handshake excluded: never written).
- m2_r4_ecc_814 (#23): 0x814 = each storage bit and 0x2f on an 8-sector decode of a host-coded page (scratch
  block, MUTATE) and on a buffer encode, against 814 = 0.
- m2_r4_ecc_metalen (#23): BCH8/BCH16 encode parity for metadata lengths 0..0xff against host BCH of the
  candidate message streams (ff/zero padding, 6-bit length, 32-byte cap).
- m2_r4_fifo_page_readback (M1-F164): buffer 0 after FMCTRL1 0x340 and 0/32/128 FIFO writes read without a
  DMA out: a page programmed from buffer 0 by a data-out transfer, raw-read in a fresh case (MUTATE).
- RUN_LAST: m2_r4_c08_running (TIMEOUT observation), m2_r4_dma_align (#20: unaligned DMA RAM addresses),
  m2_r4_data_sel_targets (#18: DMA in with 014 = 0 and non-source selects, parity buffers read back),
  m2_r4_dma_ctrl_count (#16: preset 038 bits 12:6 / 5:3 / 31:13 before a DMA), m2_r4_supersede_offset
  (#15: switch offset of an address/select rewrite versus gap, repeated), m2_r4_jr_selector (#1: jr with
  high target-selector bits; every register and offset 0 are labels, re-entry guarded).

Safety conventions (two earlier runs lost the device): every register used by ldi/sti as an address holds a
scratch address (movi only; the analyser-independent static check is in test_round4.py); no case uses an
ldi/sti/pld/pst address selector with bits 8..15 set; every value that can reach the PC is a label of the
program (jr_selector: all eight registers hold pads, offset 0 is the guarded program start); C60..C6C are
never written; changed control words (C48, C58, C14 is write-ignored) are restored at the END and in the
fault handler; NAND erases/programs only on the scratch geometry (block g.block) and only with MUTATE.
Start word count<<8|n<<4|5; C58 = 4 vectors a fault to C4C, 0 continues.
"""
from . import followup_experiments as F, sequences as Q  # noqa: F401
from .protocol import DISCOVERY, MUTATE
from fmss import probe as P

Case, finish, memory, delay, sampler, NOP = F.Case, F.finish, F.memory, F.delay, F.sampler, F.NOP
MASK = 0xffffffff
SHORT = 0x07b5                     # unmet WAIT faults at ~4.39 ms
START1 = 0x01b5                    # count 1: tmwait ends ~943 us, an unmet wait faults ~1433 us
START0 = 0x00b5                    # count 0: faults at the first instant (~448 us)
LONG = 0xffb5
MARK1, MARK2 = '    movi r2,#0x77770001\n', '    movi r2,#0x77770002\n'
ACK2 = Q.wr(0x48, 0x100000)
CLEAR = 0xff00ffe                  # set-up acknowledge word (every FMC flag)
STEP_US = 492.747
SCRATCH_BYTES = 0x8000

ITEMS = {
    '1a': ('1', 'why a high-bit source selector read 0x20 after a mov that wrote 0x20 (register-bus latch, one case)'),
    '1b': ('1', 'extended register role of jr with high target-selector bits'),
    '3': ('3', 'undefined opcodes other than 1a/80/fe in pending states'),
    '4a': ('4', 'nop (08) with R = 2, 4..6, 9, 11..254 and the stall mechanism for R >= 8'),
    '4b': ('4', 'which C48 bits make a tmwait fault'),
    '4c': ('4', 'why a C14 write of 0x1000 moves the tmwait end 24 us earlier'),
    '6': ('6', 'what sets the prescaler phase before the start; n = 4..6 first instants'),
    '8a': ('8', 'what the C08 values 1, 2 and 9 read while a program runs encode'),
    '8b': ('8', 'the effect of C58 bits 1 and 4 outside END diversion'),
    '8c': ('8', 'the meaning of C14 = 0x7f after a stalled wait'),
    '11': ('11', 'which operations other than a command and an address operation an unreleased latch refuses'),
    '15': ('15', 'what sets the offset at which a register rewrite takes effect; bytes matching neither input'),
    '16': ('16', '038 bits 5:3 and 31:13; why non-zero bits 12:6 keep a DMA from completing'),
    '18': ('18', 'where a DMA in with FMC_DATA_SEL = 0 writes; FMC_DATA_SEL values other than the source words'),
    '19a': ('19', 'FMC_STATUS bits 14..17 (bits 19:12 after transfers)'),
    '19b': ('19', 'why a CPU-written FMC_DATA_SEL value reads 0x550d where the sequencer-written one reads 0x5541'),
    '20': ('20', 'burst and DMA alignment'),
    '23a': ('23', 'the role of 0x814'),
    '23b': ('23', 'the parity of raw stream pages written with metadata length 0xff (M2-197)'),
    '26': ('26', 'which step of the complete reset outside C00 zeroes FMC 00c..05c and ECC_CTRL'),
    'F164': ('M1-F164', 'what data buffer 0 holds after FMCTRL1 0x340 and FIFO writes (no DMA-out read path)'),
}

SKIPPED = {
    '#2 PC width above 2^20, low bus map': (
        'NOT_MEASURABLE: a non-wrapping jump above 2^20 runs unbounded SDRAM; jump targets that are not program '
        'labels are excluded by the safety rule.'),
    '#7 final instruction of the USB losses': (
        'no output survives a loss and nothing survives a reset; the bus-hang candidate (ldi of an unmapped '
        'address) belongs to the separate M1-F059 reboot tests.'),
    '#9 sources of WAIT 6..10, 13..15': (
        '26 stimulus kinds already time out (M2-386, r3); the probe can write only FMSS offsets and the VIC soft '
        'lines, and every reachable FMC/ECC/DMA/status/IRQ state has been offered; no untested stimulus left.'),
    '#10 condition of WAIT11': (
        'passes in every reachable state including NAND array busy (M2-10, M2-69, M2-107, M2-387); the probe '
        'cannot create a state outside the tested set.'),
    '#11 why a latch 1..2 instructions after an unwaited 70h sets bit 3': (
        'the window is measured to single-instruction resolution (M2-593); no finer observable exists in the '
        'harness (048 sampling is 188 ns, coarser than the window).'),
    '#13 VIC0 line 5 device, VIC1 hardware lines, ECC interrupt': (
        'the probe cannot enable or assert peripheral interrupt lines outside FMSS; wait2 32..63 against VIC1 is '
        'the separate key m2_m1_wait2_vic1 (M1-F174).'),
    '#20 unmapped-bus DMA/ldi, cache coherency': (
        'unmapped addresses are M1-F059 (separate reboot tests, excluded here); coherency is not observable: the '
        'host reads scratch through the CPU after cache maintenance.'),
    '#21 channel-1 pins, second channel': (
        'pin-mux and clock-gate writes are outside the probe-writable window; +0x400 access cost cannot separate '
        'decoded from undecoded (M2-596).'),
    '#24 original WRITE/ERASE contracts': (
        'the harness runs only own programs (no Apple microcode bytes); the book programs read back exactly '
        '(M2-168, M2-194, M2-196, M2-339).'),
    '#26 C60/C64 handshake stage': (
        'C60..C6C are never written (held-engine loss, P26); m2_r4_reset_steps measures the other three stages, '
        'so the handshake is identified only by elimination.'),
    '#1 sti with high address-selector bits': (
        'if the selector reads 0 (as the ALU sources do) the store goes to bus address 0 (0xe1a0f00e words: CPU '
        'code region); not bounded by scratch.'),
    '#1 extended destinations combined with extended sources beyond the latch question': (
        'extended destinations other than r8/r10 discard (M2-43); m2_r4_bus_latch tests whether a discarded '
        'write reaches the latch, the only remaining observable.'),
    '#3 operand values outside the scanned set': 'M2-282: operand forms give the 1a outcome; no new state.',
    '#6 count + 2 instants mechanism': 'no observable beyond the fault/end times already measured (M2-584).',
    '#8 C00/C5C full bit maps': (
        'C00 writes start, stop or reset the engine and cannot be restored within a case; C5C is read-only '
        'status (1 in a handler); no bounded stimulus left.'),
    '#16 038 per-burst cycle cost': 'covered by m2_r4_dma_ctrl_count timings only as completion/no completion.',
    '#18 roles of 018, 05c, FMC_CTRL0 bit 0 and 30:28': (
        'no effect in loopback, page read or timing (M2-104, M2-127..139, M2-160, M2-405); no bounded observable '
        'left; 058/05c readback is measured by m2_r4_data_sel_alias.'),
    '#19 combined 004 operation bits': (
        'single bits measured (M2-411); 2^32 combinations are not a bounded experiment and unknown combinations '
        'may start unbounded transfers.'),
    '#23 810 never-set bits, 80c bits outside the count': 'no stimulus sets them (M2-420, M2-91).',
    'M1-F164 part (b): CPU 0x080 reads': (
        'the probe snapshot skips 0x080 (fmss_probe.c snapshot_skip) and CPU_NAND jobs reset the controller '
        'before running, so no CPU read of 0x080 sees the buffer state; sequencer reads return 0 (M2-124, '
        'M2-207). Part (a) is m2_r4_fifo_page_readback.'),
    'M1-F059 (DMA/ldi to an unmapped bus address)': 'covered by separate reboot tests (task instruction).',
}


def cond(key, decision, m1=None, **extra):
    item, text = ITEMS[key]
    c = dict(item=item, key=key, open_point=text, decision=decision, **extra)
    if m1:
        c['m1'] = list(m1)
    return c


def insn(op, reg=0, arg=0, imm=0):
    value = imm if isinstance(imm, str) else '0x%x' % (imm & MASK)
    return '    .insn 0x%x,%d,0x%x,%s\n' % (op, reg, arg, value)


def label(prefix):
    F._LABEL[0] += 1
    return '%s%d' % (prefix, F._LABEL[0])


SEEDS = [0x11111111 * (i + 1) for i in range(8)]


def seeds():
    return [(0xc18 + 4 * i, SEEDS[i]) for i in range(8)]


def stores(regs, base):
    return ''.join(Q.store(off, base + 4 * k) for k, off in enumerate(regs))


def loop(n, body, prefix='lp'):
    lp = label(prefix)
    return '    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n    bnz r4,%s\n' % (n, lp, body, lp)


def trap_with(t, extra):
    return t.trap().replace('trap:\n', 'trap:\n' + extra)


def start_word(count, n=0xb):
    return (count & 0xff) << 8 | (n & 0xf) << 4 | 5


PENDING = ('command', 'address', 'DMA', 'ECC', 'status')
PINNED = ('command', 'address', 'status')


# ---- m2_r4_bus_latch (#1)
LATCH_V = 0x1a2b3c4d
LATCH_D60 = 0x0d600d60
DISCARD_V = 0x5e5e1212
BUS_PREDS = (
    ('none', '', SEEDS[0]),                                   # prologue wr leaves r0
    ('movi', '    movi r3,#0x%x\n' % LATCH_V, LATCH_V),
    ('mov', insn(6, 3, 5, 0), SEEDS[5]),
    ('add', '    add r3,r3,#1\n', SEEDS[3] + 1),
    ('ld', '    ld r3,[0xd60]\n', LATCH_D60),
    ('st', '    st [0xd64],r4\n', SEEDS[4]),
    ('wr', '    wr [0xd68],#0x1234\n', SEEDS[0]),
    ('nop2', '    movi r3,#0x%x\n' % LATCH_V + NOP, None),
    ('mov-r8', insn(6, 3, 8, 0), 'r3'),
)
BUS_SELECTORS = (0x104, 0x108, 0x1ff)


def bus_latch(s, t):
    """mov r2 with source selector 0x104/0x108/0x1ff after nine predecessors whose latch values differ; add/or/sub
    with selector 0x104 after movi; discarded writes to r12 followed by the extended st (stores the latch)."""
    F._LABEL[0] = 0
    opts = dict(params=seeds() + [(0xd60, LATCH_D60), (0xd64, 0), (0xd68, 0), (0xd6c, 0)], flags=DISCOVERY,
                timeout_us=20000)
    decision = ('the hypothesis whose predicted r2 matches names what a high-bit source selector reads: '
                '"latch" in every predecessor = register-bus latch, "zero" = constant 0; mixed = latch only after '
                'some predecessors (named); no match = other (value recorded)')
    for pred, pre, latch in BUS_PREDS:
        for sel in BUS_SELECTORS:
            predict = {'zero': 0}
            if latch is None:
                predict.update({'latch (nop2 keeps the movi value)': LATCH_V, 'latch (nop2 leaves r0)': SEEDS[0]})
            else:
                predict['latch'] = latch
            if sel == 0x104:
                predict['low byte (r4)'] = SEEDS[4]
            body = pre + insn(6, 2, sel, 0)
            yield Case('mov/%s/sel%03x' % (pred, sel), finish(t, body), dict(opts),
                       cond('1a', decision, observe='r2', predict=predict, predecessor=pred, selector=hex(sel),
                            seeds='r0..r7 = 0x11111111*(n+1); 0xd60 = 0x%x' % LATCH_D60, instruction=body.strip()))
    r2 = SEEDS[2]
    for name, op, f in (('add', 0xc, lambda x: (r2 + x) & MASK), ('or', 0xb, lambda x: r2 | x),
                        ('sub', 0xd, lambda x: (r2 - x) & MASK)):
        body = '    movi r3,#0x%x\n' % LATCH_V + insn(op, 2, 0x104, 0)
        yield Case('%s/movi/sel104' % name, finish(t, body), dict(opts),
                   cond('1a', decision, observe='r2', predict={'latch': f(LATCH_V), 'zero': f(0), 'low byte (r4)':
                                                              f(SEEDS[4])},
                        predecessor='movi', selector='0x104', instruction=body.strip()))
    for name, w in (('movi-r12', insn(5, 12, 0, DISCARD_V)), ('mov-r12', insn(6, 12, 5, 0))):
        body = '    movi r3,#0x%x\n' % LATCH_V + w + insn(2, 9, 0xd6c, 0)
        passed = DISCARD_V if name == 'movi-r12' else SEEDS[5]
        yield Case('discard/%s' % name, finish(t, body), dict(opts),
                   cond('1a', 'd6c (extended st = latch) equal to the discarded value = a discarded extended-'
                              'destination write passes the register bus; equal to the movi value = it does not',
                        observe='d6c', predict={'passes the bus': passed, 'not on the bus': LATCH_V},
                        instruction=body.strip()))


# ---- m2_r4_undefined_scan (#3)
UNDEF_SCAN = tuple(op for op in range(0x1b, 0xfe) if op != 0x80)


def undefined_scan(s, t):
    """Every undefined opcode 1b..fd except 80 with a pending command, address, DMA, ECC or status flag."""
    from .discovery import states
    F._LABEL[0] = 0
    g, actions = Q.geometry(s), states(s, t)
    for op in UNDEF_SCAN:
        for state in PENDING:
            pins = state in PINNED
            body = Q.setup(g.ce if pins else None) + actions[state] + MARK1 + insn(op) + MARK2 + '    ld r4,[0x48]\n'
            yield Case('op%02x/%s' % (op, state), finish(t, body),
                       dict(start_value=START1, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                       cond('3', 'fault at the count-1 wait timeout with r2 0x77770001 = unmet wait as 1a/80/fe '
                                 '(M2-583); r2 0x77770002 END = the opcode passes on this pending state',
                            opcode=hex(op), state=state, start_word=hex(START1)), pins)


# ---- m2_r4_nop_operand (#4)
NOP_PENDING_R = tuple(range(8, 16))


def nop_operand(s, t):
    """nop (08) with every R = 0..255 at idle; R = 8..15 with each pending flag."""
    from .discovery import states
    F._LABEL[0] = 0
    g, actions = Q.geometry(s), states(s, t)
    decision = ('END with r2 0x77770002 = continues; fault with r2 0x77770001 = stalls (fault time against the '
                '4390 us wait and the 4226..4250 us nop stall of M2-377); a stall released only with a pending flag '
                '= R selects an event (R - 8 = event number if the released flag is that event)')
    for R in range(256):
        yield Case('r%02x' % R, finish(t, MARK1 + insn(8, R) + MARK2),
                   dict(start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   cond('4a', decision, R=R, state='idle'))
    for R in NOP_PENDING_R:
        for state in PENDING:
            pins = state in PINNED
            body = Q.setup(g.ce if pins else None) + actions[state] + MARK1 + insn(8, R) + MARK2 + '    ld r4,[0x48]\n'
            yield Case('r%02x/%s' % (R, state), finish(t, body),
                       dict(start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                       cond('4a', decision, R=R, state=state), pins)


# ---- m2_r4_c48_bits (#4)
C48_VALUES = (0, MASK, 0x55555555, 0xaaaaaaaa) + tuple(1 << b for b in range(32))


def c48_bits(s, t):
    """tmwait (count 1) with C48 = 0, ones, 0x55555555, 0xaaaaaaaa and each single bit (CPU-written)."""
    F._LABEL[0] = 0
    for v in C48_VALUES:
        body = MARK1 + insn(0x15) + MARK2 + Q.wr(0xc48, 0)
        src = t.prologue(4) + body + Q.wr(0, 0) + '    movi r6,#0x600d600d\n    end\n' + trap_with(t, Q.wr(0xc48, 0))
        yield Case('c48/%08x' % v, src, dict(start_value=START1, params=[(0xc48, v)], flags=P.WAIT_END,
                                             timeout_us=20000),
                   cond('4b', 'tmwait END at the C48 = 0 time = the value does not make it fault; trap at the wait '
                              'timeout = it does; the set of faulting single bits must predict 0x55555555, '
                              '0xaaaaaaaa and ones (any-bit OR rule), otherwise the rule is not a bit mask',
                        c48=hex(v), start_word=hex(START1)))


# ---- m2_r4_c14_write (#4)
C14_PLAN = ([(0xff, v, 0) for v in (None, 0, 0x10, 0x100, 0x800, 0x1000, 0x2000, 0xffff, MASK)]
            + [(0xff, v, r) for v in (None, 0x1000) for r in (1, 2)]
            + [(c, v, 0) for c in (0x10, 0x01) for v in (None, 0x1000)])


def c14_write(s, t):
    """tmwait after a program write of C14 (values at count 255, repeats; 0x1000 at counts 16 and 1)."""
    F._LABEL[0] = 0
    for count, v, rep in C14_PLAN:
        w = Q.wr(0xc14, v) if v is not None else ''
        body = MARK1 + w + insn(0x15) + MARK2 + '    ld r5,[0xc14]\n'
        yield Case('c%02x/%s/rep%d' % (count, 'none' if v is None else '%x' % v, rep), finish(t, body),
                   dict(start_value=start_word(count), flags=P.WAIT_END, timeout_us=200000),
                   cond('4c', 'tmwait end minus the no-write end (same count): within the repeat spread = no '
                              'effect; one value only = bit-specific; every value = a write-triggered phase change; '
                              'shift growing with the count = rate change',
                        count=count, c14=None if v is None else hex(v), rep=rep))


# ---- m2_r4_prescaler_origin (#6)
PAD = [(0xd00 + 4 * (k % 32), 0) for k in range(90)]


def prescaler_origin(s, t):
    """Count-0 WAIT3 first instant: three repeats for n = 4..7, 9, 0xb; no per-case reset; 30/60/90 extra
    parameter writes before the start."""
    F._LABEL[0] = 0
    body = MARK1 + Q.wait(3) + MARK2
    plan = [('n%x/rep%d' % (n, r), n, 0, 0) for n in (4, 5, 6, 7, 9, 0xb) for r in range(3)]
    plan += [('n%x/noreset/rep%d' % (n, r), n, 0, P.S_NO_RESET) for n in (7, 9, 0xb) for r in range(2)]
    plan += [('n%x/pad%02d' % (n, k), n, k, 0) for n in (9, 0xb) for k in (30, 60, 90)]
    for name, n, pads, steps in plan:
        opts = dict(start_value=start_word(0, n), params=PAD[:pads], flags=DISCOVERY | P.WAIT_END, timeout_us=20000)
        if steps:
            opts['flags'] = opts.get('flags', 0) | steps   # probe step flags travel in flags (batch and single-probe paths)
        yield Case(name, finish(t, body), opts,
                   cond('6', 'repeats equal = phase fixed by an event before the start; pads moving the instant '
                             'earlier linearly with the write count = prescaler free-running since before the '
                             'parameters (the reset); noreset different from reset = the per-case reset sets the '
                             'phase, equal = an earlier event (clock enable); n = 4..6 repeats give the spread',
                        n=n, pads=pads, reset=not steps, start_word=hex(start_word(0, n))))


# ---- m2_r4_c08_running (#8)
def c08_kinds(a):
    return {
        'nop2': ('', NOP), 'nop08': ('', insn(8)), 'movi': ('', '    movi r2,#0x1234\n'),
        'mov': ('', insn(6, 2, 3, 0)), 'add': ('', '    add r2,r2,#1\n'), 'ld': ('', '    ld r2,[0x48]\n'),
        'st': ('', '    st [0xd7c],r2\n'), 'wr': ('', '    wr [0xd7c],#0x1\n'),
        'ldi': ('    movi r5,#0x%x\n' % (a + 0x100), '    ldi r1,[r5]\n'),
        'sti': ('    movi r7,#0x%x\n' % (a + 0x100), '    sti [r7],r1\n'),
        'pld': ('    movi r1,#0xd7c\n', insn(0x18, 2, 1, 0)), 'wait11': ('', Q.wait(11)),
        'empty': ('', None),
    }


def c08_running(s, t):
    """C08 in the after-map 3 ms into a loop of 16 copies of one instruction kind, three runs per kind."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for kind, (pre, one) in c08_kinds(a).items():
        for rep in range(3):
            body = pre + (loop(30000, one * 16) if one else loop(400000, ''))
            yield Case('%s/rep%d' % (kind, rep), finish(t, body),
                       dict(start_value=LONG, params=[(0xd7c, 0)], flags=DISCOVERY, timeout_us=3000,
                            scratch=memory(), scratch_read=16),
                       cond('8a', 'one C08 value per kind in all three runs, differing between kinds = C08 encodes '
                                  'the executing instruction; values varying within a kind = sampling phase, not the '
                                  'instruction', kind=kind, rep=rep, observe_us=3000))


# ---- m2_r4_c58_bits (#8)
C58_VALUES = (0, 4, 0x02, 0x06, 0x10, 0x14, 0x12, 0x16)
C58_SITUATIONS = {'wait3': Q.wait(3), 'tmwait0': insn(0x15), 'op1a': insn(0x1a)}


def c58_bits(s, t):
    """C58 bits 1/4 alone and with bit 2 on an unmet WAIT3, a count-0 tmwait and undefined opcode 1a."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for sit, x in C58_SITUATIONS.items():
        for v in C58_VALUES:
            snap = stores((0xc5c, 0xc08, 0xc58, 0xc0c), a + 0x2000)
            body = MARK1 + x + MARK2 + snap + Q.wr(0xc58, 4)
            handler = trap_with(t, stores((0xc5c, 0xc08, 0xc58, 0xc0c), a + 0x2010) + Q.wr(0xc58, 4))
            yield Case('%s/c58-%02x' % (sit, v), finish(t, body, v, handler),
                       dict(start_value=START0, scratch=memory(), scratch_read=0x2020, flags=DISCOVERY | P.WAIT_END,
                            timeout_us=20000),
                       cond('8b', 'trap (r6 0xfa17fa17) iff bit 2 and END with r2 0x77770002 otherwise, for every bit 1/4 '
                                  'combination = bits 1/4 have no effect on this timeout path; any other outcome or a '
                                  'changed C5C/C08 names an effect', situation=sit, c58=hex(v),
                            layout='0x2000: C5C C08 C58 C0C after continue; 0x2010: same in the handler'))


# ---- m2_r4_c14_release (#8)
C14_REL = ([(c, 0xb, us) for c in (0x03, 0x10, 0x40) for us in (200, 700, 1200, 2200)]
           + [(0x10, 9, us) for us in (50, 300, 600)])


def c14_release(s, t):
    """C14 right after and 1 ms after a WAIT0 released by a CPU write of 70h after 0..4 prescaler steps."""
    F._LABEL[0] = 0
    g = Q.geometry(s)
    for count, n, us in C14_REL:
        body = (Q.setup(g.ce) + MARK1 + Q.wait(0) + MARK2 + '    ld r1,[0xc14]\n' + Q.wr(0x48, 2) + delay(15000)
                + '    ld r5,[0xc14]\n')
        yield Case('c%02x/n%x/t%04d' % (count, n, us), finish(t, body),
                   dict(start_value=start_word(count, n), late=[(8, 0x70)], late_us=us, flags=P.WAIT_END,
                        timeout_us=50000),
                   cond('8c', 'r1 (C14 right after the release) = start count for a release before the first step, '
                              '0x7f for every count, n and step number = a constant reload; count - steps = C14 '
                              'counts and keeps; other = recorded; r5 (1 ms later) equal to r1 = no counting after '
                              'the wait', count=count, n=n, release_us=us, start_word=hex(start_word(count, n))), True)


# ---- m2_r4_latch_refusal (#11)
def latch_ops(a):
    return {
        'command': Q.wr(8, 0x70) + Q.wait(0) + Q.wr(0x48, 2),
        'address': Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1) + Q.wr(0x48, 4),
        'dma-in': Q.dma(a, 0),
        'dma-out': Q.dma(a + 0x2400, 0, into=False),
        'read512': Q.transfer(512),
        'parity16': Q.transfer(16, parity=True),
        'fifo8': Q.wr(0x30, 7) + Q.wr(0x14, 0x10) + Q.wr(4, 0xe2) + Q.wait(3) + Q.wr(0x48, 8),
        'ecc': Q.ecc(0, True),
        'latch2': Q.wr(4, 0xca) + Q.wait(12, 0x40, 0x40) + Q.wr(0x48, 0x1800000),
    }


def latch_refusal(s, t):
    """After a status latch (70h waited, latch, wait 12, bit-23/24 ack) each operation with and without the
    FMCTRL1 0x30020 release; C58 = 0 so a refused operation's wait times out and the run continues."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    pre = Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(12, 0x40, 0x40) + Q.wr(0x48, 0x1800000)
    cleanup = Q.wr(4, 0x30020) + Q.wr(0x48, CLEAR) + Q.wr(0x810, 1)
    for op, x in latch_ops(a).items():
        for released in (True, False):
            body = (pre + (Q.wr(4, 0x30020) if released else '') + stores((0x48,), a + 0x2010) + MARK1 + x + MARK2
                    + stores((0x48, 0x810, 0x38, 0x4c), a + 0x2000) + cleanup)
            yield Case('%s/%s' % (op, 'released' if released else 'latched'), finish(t, body, 0),
                       dict(start_value=SHORT, scratch=memory(), scratch_read=0x2600, flags=P.WAIT_END,
                            timeout_us=50000),
                       cond('11', 'fault_at = 0 (no timeout) = the operation completed; a timeout in the latched case '
                                  'with the released control passing = refused by the unreleased latch; 048 bit 26 '
                                  'after a refusal names its other setters', operation=op, released=released,
                            layout='0x2000: 048 810 038 04c after the operation; 0x2010: 048 before; 0x2400 DMA out'),
                       True)


# ---- m2_r4_status_bits (#19)
STATUS_LENGTHS = (1, 2, 3, 4, 5, 8, 16, 31, 32, 33, 64, 128, 256, 384, 511, 512)
PARITY_LENGTHS = (13, 16, 23, 64)


def status_bits(s, t):
    """FMC_STATUS after data reads of 1..512 bytes and parity reads of an open page, and after DMAs."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    opened = Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
    plan = [('data/L%03d' % L, L, False) for L in STATUS_LENGTHS] + [('parity/L%02d' % L, L, True) for L in PARITY_LENGTHS]
    for name, L, parity in plan:
        x = (Q.wr(0x30, L - 1) + Q.wr(0x14, 0x10 if parity else 1) + Q.wr(4, 0x32 if parity else 0xe2) + Q.wait(3)
             + stores((0x48,), a + 0x2004) + Q.wr(0x48, 8) + stores((0x48,), a + 0x2008))
        yield Case(name, finish(t, opened + stores((0x48,), a + 0x2000) + x),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=SHORT, timeout_us=50000),
                   cond('19a', 'bits 19:12 of 048 after the transfer (0x2004) as a function of the length: one formula '
                               '(e.g. ((L - 1) >> k) & 0xf in bits 17:14) over every length = bits 14..17 count the '
                               'transfer; constant = flag', kind='parity' if parity else 'data', length=L,
                        layout='0x2000: 048 before, 0x2004 after wait 3, 0x2008 after the ack'), True)
    for name, into in (('dma-in', True), ('dma-out', False)):
        x = Q.dma(a + (0 if into else 0x2400), 0, into=into).replace(ACK2, stores((0x48,), a + 0x2004) + ACK2)
        yield Case(name, finish(t, opened + stores((0x48,), a + 0x2000) + x + stores((0x48,), a + 0x2008)),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=SHORT, timeout_us=50000),
                   cond('19a', 'bits 19:12 after a 512-byte DMA (restore to 0xc3 expected, M2-497)', kind=name,
                        length=512, layout='0x2000: 048 before, 0x2004 after wait 2, 0x2008 after the ack'), True)


# ---- m2_r4_data_sel_alias (#19)
BANK_WORDS = (0x00c, 0x010, 0x014, 0x018, 0x01c, 0x02c, 0x030, 0x034, 0x058, 0x05c)
ALIAS_SEQS = {
    '014-then-058': ((0x14, 0x5541), (0x58, 0x0d)),
    '058-then-014': ((0x58, 0x0d), (0x14, 0x5541)),
    '014-only': ((0x14, 0x5541),),
    '058-only': ((0x58, 0x0d),),
    '014-then-058ff': ((0x14, 0x5541), (0x58, 0xff)),
    '014ffff-then-058-0': ((0x14, 0xffff), (0x58, 0)),
    '014-then-018': ((0x14, 0x5541), (0x18, 0xd)),
    '014-then-05c': ((0x14, 0x5541), (0x5c, 0x5509)),
    'p18-order': tuple((off, 0x55555555 ^ off) for off in BANK_WORDS),   # 034 replaced by a scratch address
}
ALIAS_READ = (0x14, 0x18, 0x58, 0x5c)


def data_sel_alias(s, t):
    """014/018/058/05c read by the sequencer after write sequences by the sequencer, by CPU parameters and
    (p18 order) by CPU late writes."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    restore = Q.wr(0x14, 0) + Q.wr(0x58, 0)
    decision = ('014 read 0x5541 = no alias; 014 bits 7:0 equal to the later 058 (or 018/05c) value = a write of '
                'that word replaces 014 bits 7:0; the same outcome for both writers = not a CPU-path effect')
    for name, writes in ALIAS_SEQS.items():
        writes = tuple((o, a + 0x100 if o == 0x34 else v) for o, v in writes)   # 034: a scratch address
        rd = stores(ALIAS_READ, a + 0x2000)
        yield Case('seq/%s' % name, finish(t, ''.join(Q.wr(o, v) for o, v in writes) + rd + restore),
                   dict(scratch=memory(), scratch_read=0x2010, timeout_us=20000),
                   cond('19b', decision, writer='sequencer', sequence=name,
                        writes=[[hex(o), hex(v)] for o, v in writes], layout='0x2000: 014 018 058 05c'))
        yield Case('cpu/%s' % name, finish(t, rd + restore),
                   dict(params=list(writes), scratch=memory(), scratch_read=0x2010, timeout_us=20000),
                   cond('19b', decision, writer='cpu parameters', sequence=name,
                        writes=[[hex(o), hex(v)] for o, v in writes], layout='0x2000: 014 018 058 05c; before-map'))
    writes = tuple((o, a + 0x100 if o == 0x34 else v) for o, v in ALIAS_SEQS['p18-order'])
    yield Case('cpu-late/p18-order', finish(t, delay(30000) + stores(ALIAS_READ, a + 0x2000) + restore),
               dict(late=list(writes), late_us=1000, scratch=memory(), scratch_read=0x2010, timeout_us=50000),
               cond('19b', decision, writer='cpu late writes at 1000 us', sequence='p18-order',
                    writes=[[hex(o), hex(v)] for o, v in writes], layout='0x2000: 014 018 058 05c'))


# ---- m2_r4_reset_steps (#26)
def reset_words(a):
    return ((0x00c, 0x12345678), (0x010, 0x9abcdef0), (0x014, 0x0101), (0x018, 0x5), (0x01c, 1), (0x02c, 3),
            (0x030, 0x1ff), (0x034, a + 0x100), (0x038, 1), (0x058, 0x20), (0x05c, 0x200), (0x804, 0x304))


RESET_STEPS = {
    'none': (), 'ctrl1-800': ((4, 0x800),), 'ctrl1-80000000': ((4, 0x80000000),),
    'ctrl1-800-80000000': ((4, 0x800), (4, 0x80000000)), 'ctrl1-80000800': ((4, 0x80000800),),
    'ecc-81c': ((0x81c, 1),), 'ctrl1-800-ecc-81c': ((4, 0x800), (0x81c, 1)),
    'all': ((4, 0x800), (4, 0x80000000), (0x81c, 1)),
}


def reset_steps(s, t):
    """Storage words written, then one candidate reset stage, then read back (CPU parameters or program)."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    words = reset_words(a)
    rd = stores([o for o, _ in words], a + 0x2000)
    for name, steps in RESET_STEPS.items():
        for writer in ('cpu', 'seq'):
            if writer == 'cpu':
                body, opts = rd, dict(params=list(words) + list(steps))
            else:
                body, opts = ''.join(Q.wr(o, v) for o, v in words + tuple(steps)) + rd, {}
            opts.update(scratch=memory(), scratch_read=0x2040, timeout_us=20000)
            yield Case('%s/%s' % (writer, name), finish(t, body + Q.wr(0x38, 7)), opts,
                       cond('26', 'words still holding the written value after the stage were kept, zero or default '
                                  'were cleared by it; the stage that zeroes 00c..05c and ECC_CTRL in both writers is '
                                  'the clearing step; none = the C60/C64 handshake (inferred by elimination)',
                            writer=writer, stage=name, steps=[[hex(o), hex(v)] for o, v in steps],
                            written={hex(o): hex(v) for o, v in words},
                            layout='0x2000 + 4k: readback of the written words in order'))


# ---- m2_r4_ecc_814 (#23)
ERR814 = (0, 1, 4, 8, 9, 0, 2, 12)
V814 = (0, 1, 2, 4, 8, 0x20, 0x2f)


def ecc_814(s, t):
    """0x814 bits on an 8-sector BCH8 decode of a host-coded scratch page and on a buffer encode."""
    import hashlib
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    if g.strength != 8:
        return
    raw, _ = F.coded_page(ERR814)
    mem = memory()
    mem[:len(raw)] = raw
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), cond('23a', 'scratch block erase', block=g.block), True)
    yield Case('program', finish(t, Q.write_page(g, g.row, a, F.BCH_META, raw=True)),
               dict(mutate, scratch=mem, scratch_read=0),
               cond('23a', 'host code words with %s errors per sector' % '/'.join(map(str, ERR814)), errors=ERR814,
                    raw_sha256=hashlib.sha256(raw).hexdigest()), True)
    for v in V814:
        per = lambda sec, slot: stores((0x810, 0x80c, 0x818), a + 0x3100 + 16 * sec)  # noqa: E731
        body = Q.wr(0x814, v) + F.decode_loop(g, g.row, a + 0x2040, 8, 8, per) + Q.wr(0x814, 0)
        yield Case('decode/814-%02x' % v, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x3180, timeout_us=300000),
                   cond('23a', 'per-sector 810/80c/818 and the 4096 decoded bytes equal to 814 = 0 = this value has no '
                               'decode effect; differences name the effect', value=hex(v), errors=ERR814,
                        layout='0x2000 meta words, 0x2040 data, 0x3100 + 16s: 810 80c 818 of sector s'), True)
    for v in V814:
        body = (Q.setup() + Q.wr(0x814, v) + Q.dma(a, 0) + Q.ecc(0, True, 8, 0, acknowledge=False) + Q.wait(4)
                + stores(range(0x840, 0x860, 4), a + 0x2000) + stores((0x810, 0x814), a + 0x2020) + Q.wr(0x810, 1)
                + Q.wr(0x814, 0))
        yield Case('encode/814-%02x' % v, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2028, timeout_us=20000),
                   cond('23a', 'parity 0x840..0x85c equal to 814 = 0 (= host BCH8 of the 512 data bytes) = no encode '
                               'effect', value=hex(v), layout='0x2000: 840..85c; 0x2020: 810 814'))


# ---- m2_r4_ecc_metalen (#23)
META10 = bytes(range(0xa0, 0xaa))
METALENS = tuple(('bch8', L) for L in (0, 10, 12, 13, 16, 20, 32, 33, 64, 128, 255)) + tuple(
    ('bch16', L) for L in (10, 32, 255))


def ecc_metalen(s, t):
    """Encode of the 512 scratch bytes with the metadata registers holding META10 and lengths 0..0xff."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for code, L in METALENS:
        kick = 2 | (4 if code == 'bch16' else 0) | (L << 8)
        body = (Q.setup() + Q.metadata(META10) + Q.dma(a, 0) + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, kick)
                + Q.wait(4) + stores(range(0x840, 0x860, 4), a + 0x2000) + stores((0x810,), a + 0x2020)
                + Q.wr(0x810, 1))
        yield Case('%s/L%03d' % (code, L), finish(t, body), dict(scratch=memory(), scratch_read=0x2024, timeout_us=20000),
                   cond('23b', 'parity equal to host %s of data || stream(L) for one stream hypothesis (metadata '
                               'registers then 0xff, zeros or wrap; length L, L & 0x3f or min(L, 32)) names the raw '
                               'stream; none = unexplained' % code.upper(), code=code, metalen=L, kick=hex(kick),
                        meta=META10.hex(), layout='0x2000: 840..85c; 0x2020: 810'))


# ---- m2_r4_fifo_page_readback (M1-F164)
FIFO_TAG = 0xc3d40000
FIFO_VARIANTS = (('ctl/w0', False, 1, 0, False), ('ctl/w128', False, 1, 128, False),
                 ('x340/w0', True, 1, 0, False), ('x340/w32', True, 1, 32, False), ('x340/w128', True, 1, 128, False),
                 ('x340sel0/w128', True, 0, 128, False), ('x340drain/w128', True, 1, 128, True))


def fifo_memory():
    mem = memory()
    for k in range(128):
        mem[4 * k:4 * k + 4] = (FIFO_TAG | k).to_bytes(4, 'little')
    return mem


def fifo_page_readback(s, t):
    """Buffer 0 loaded with tagged words, optional FMCTRL1 0x340 (sink a + 0x2800) and 0/32/128 FIFO writes,
    then 80h, address, a 512-byte data-out transfer from buffer 0, 10h; each page raw-read in a later case."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mutate = dict(flags=MUTATE, timeout_us=200000)
    decision = ('page words 0..127: tagged = buffer 0 unchanged, 0x5a000000+k = written, 0 = 0x340 zeros, 0xff = '
                'nothing transferred; same as M1 if part (not all) of buffer 0 holds written values after 0x340, '
                'differs if it is unchanged or all 128 words hold written values; the ctl/w0 page must equal the tags')
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate),
               cond('F164', 'scratch block erase', m1=('M1-F164',), block=g.block), True)
    for page, (name, x340, sel, writes, drain) in enumerate(FIFO_VARIANTS):
        body = Q.setup(g.ce) + Q.dma(a, 0)
        if x340:
            body += (Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, sel) + Q.wr(0x34, a + 0x2800) + Q.wr(4, 0x340)
                     + Q.wait(2) + ACK2)
        body += stores((0x38, 0x48), a + 0x2c00)
        if drain:
            body += Q.dma(a + 0x3000, 0, into=False)
        body += ''.join(Q.wr(0x80, 0x5a000000 + k) for k in range(writes)) + stores((0x38, 0x48), a + 0x2c08)
        body += Q.command(0x80) + Q.address(g.row + page) + Q.transfer(512, 0, into=False)
        body += stores((0x38, 0x48), a + 0x2c10) + Q.command(0x10) + Q.ready() + '    ld r5,[0x4c]\n'
        yield Case('program/%s' % name, finish(t, body, 0),
                   dict(mutate, scratch=fifo_memory(), scratch_read=0x3200, start_value=SHORT),
                   cond('F164', decision, m1=('M1-F164',), variant=name, x340=x340, data_sel=sel, writes=writes,
                        drain=drain, row=g.row + page,
                        layout='0x2800 0x340 sink; 0x2c00: 038 048 after 0x340, 0x2c08 after the writes, 0x2c10 after '
                               'the data-out transfer; 0x3000 drain read-back; r5 = 04c after 10h'), True)
    reads = [(name, page) for page, (name, *_r) in enumerate(FIFO_VARIANTS)] + [('erased', len(FIFO_VARIANTS))]
    for name, page in reads:
        body = (Q.setup(g.ce) + Q.command(0) + Q.address(g.row + page) + Q.command(0x30) + Q.ready() + Q.command(0)
                + Q.transfer(512) + Q.dma(a + 0x2000, 0, into=False) + Q.transfer(512) + Q.dma(a + 0x2200, 0, into=False))
        yield Case('read/%s' % name, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2400, timeout_us=200000),
                   cond('F164', decision, m1=('M1-F164',), variant=name, row=g.row + page,
                        layout='0x2000 page bytes 0..1023 (raw)'), True)


# ---- m2_r4_dma_align (#20)
ALIGN_OFFSETS = (1, 2, 3, 4, 8, 16, 32)


def dma_align(s, t):
    """DMA in from RAM a + off (read back from an aligned address) and DMA out to a + 0x2400 + off."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for off in ALIGN_OFFSETS:
        yield Case('in/o%02d' % off, finish(t, Q.setup() + Q.dma(a + off, 0) + Q.dma(a + 0x2000, 0, into=False), 0),
                   dict(scratch=memory(), scratch_read=0x2700, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond('20', 'read-back equal to RAM[off..] = byte-exact source; RAM[off & ~3..] = word aligned; '
                              'RAM[off & ~(2^k - 1)..] = aligned to 2^k (burst)', direction='in', offset=off,
                        layout='0x2000 read-back of buffer 0'))
        yield Case('out/o%02d' % off, finish(t, Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x2400 + off, 0, into=False), 0),
                   dict(scratch=memory(), scratch_read=0x2700, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond('20', 'the 512 source bytes found at 0x2400 + off = byte exact; at an aligned-down address = '
                              'alignment granule; bytes changed outside the window = overrun', direction='out',
                        offset=off, layout='0x2400 + off destination'))


# ---- m2_r4_data_sel_targets (#18)
TARGET_SELECTS = (1, 0x10, 0, 3, 4, 8, 0x40, 0x80, 0x11, 0x100, 0x400, 0x4000, 0x8000)


def data_sel_targets(s, t):
    """Data buffers 0/1 and parity buffers 0/1 loaded with A/B/P0/P1, then a DMA in of C with FMC_DATA_SEL = v;
    all four buffers read back by DMA out."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for v in TARGET_SELECTS:
        body = (Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1) + Q.dma(a + 0x800, select=0x10)
                + Q.dma(a + 0xc00, select=0x20))
        body += (Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, v) + Q.wr(0x34, a + 0x1000) + Q.wr(4, 0x2e0) + Q.wait(2)
                 + stores((0x48, 0x38), a + 0x3000) + ACK2 + Q.wr(0x14, 0))
        body += (Q.dma(a + 0x2000, 0, into=False) + Q.dma(a + 0x2400, 1, into=False)
                 + Q.dma(a + 0x2800, select=0x1000, into=False) + Q.dma(a + 0x2c00, select=0x2000, into=False))
        yield Case('in/sel-%04x' % v, finish(t, body, 0),
                   dict(scratch=memory(), scratch_read=0x3008, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('18', 'the read-back holding C (RAM 0x1000; parity buffers hold its last 64 bytes) is the '
                              'target of this select; none = no target found; selects 1/0x10 are the controls',
                        select=hex(v), layout='0x2000 data 0, 0x2400 data 1, 0x2800 parity 0, 0x2c00 parity 1; '
                                              '0x3000: 048 038 after the DMA'))


# ---- m2_r4_dma_ctrl_count (#16)
CTRL_COUNT = ([('v7/k%03d' % k, k << 6 | 7) for k in (0, 1, 4, 8, 16, 64, 120, 127)]
              + [('v0/k%03d' % k, k << 6) for k in (0, 1, 4, 64)]
              + [('v3/k%03d' % k, k << 6 | 3) for k in (0, 2, 4, 8)]
              + [('w%08x' % w, w) for w in (0x0f, 0x17, 0x27, 0x3f, 0x2007, 0x80000007, 0xffffe007)])


def dma_ctrl_count(s, t):
    """A DMA out of buffer 0 (A) programmed after 038 = w (preset bits 12:6, bits 5:3 or 31:13), C58 = 0."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for name, w in CTRL_COUNT:
        body = (Q.setup() + Q.dma(a, 0) + Q.wr(0x38, w) + stores((0x38,), a + 0x2400) + Q.wr(0x30, 511)
                + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100) + Q.wait(2) + stores((0x38, 0x48), a + 0x2404)
                + ACK2 + delay(300) + stores((0x38,), a + 0x240c) + Q.wr(0x38, 7))
        yield Case(name, finish(t, body, 0),
                   dict(scratch=memory(), scratch_read=0x2410, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('16', 'completion (no fault, 048 bit 20) and the delivered A bytes per preset: completion only '
                              'for k a multiple of (v + 1) words, with 512 - 4k bytes delivered = bits 12:6 count words '
                              'from the preset; bits 5:3 / 31:13 readback and completion name their storage/effect',
                        value=hex(w), k=(w >> 6) & 0x7f, v=w & 7,
                        layout='0x2000 DMA target; 0x2400: 038 after the write, 0x2404 038 048 after wait 2, 0x240c 038 later'))


# ---- m2_r4_supersede_offset (#15)
SUPERSEDE_GAPS = (0, 2, 4, 8, 16, 32, 64)


def supersede_offset(s, t):
    """DMA out of buffer 0 (A) to 0x2000; after g nop2 one register rewritten (034 -> 0x2400 or 014 -> buffer 1)."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    rewrite = {'addr': Q.wr(0x34, a + 0x2400), 'select': Q.wr(0x14, 0x200)}
    for reg in ('addr', 'select'):
        for gap in SUPERSEDE_GAPS:
            for rep in range(2):
                body = (Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1) + Q.wr(0x38, 7) + Q.wr(0x30, 511)
                        + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100) + NOP * gap + rewrite[reg]
                        + Q.wait(2) + stores((0x48, 0x38), a + 0x2600) + ACK2 + delay(300))
                yield Case('%s/gap%02d/rep%d' % (reg, gap, rep), finish(t, body, 0),
                           dict(scratch=memory(), scratch_read=0x2608, start_value=SHORT, flags=P.WAIT_END,
                                timeout_us=50000),
                           cond('15', 'switch offset (first byte of 0x2000 not A; for addr the continuation at 0x2400 + '
                                      'offset) versus gap: linear in the gap with equal repeats = set by the transfer '
                                      'progress at the write; spread between repeats = arbitration; bytes matching '
                                      'neither input are classified initial/zero/other', register=reg, gap_nops=gap,
                                rep=rep, layout='0x2000 first target, 0x2400 second; 0x2600: 048 038'))


# ---- m2_r4_jr_selector (#1)
JR_SELECTORS = (5, 0x100, 0x101, 0x105, 0x1ff, 0x8005)


def jr_selector(s, t):
    """jr (condition r1) with target selector 5 (control) and with bits 8..15 set. Every GPR holds the offset of
    its own landing pad; offset 0 (selector reading 0) restarts the program, where a PRAM guard stops it."""
    F._LABEL[0] = 0
    for sel in JR_SELECTORS:
        pads = [label('pad') for _ in range(8)]
        restart, out = label('rs'), label('out')
        body = '    ld r1,[0xd78]\n    bnz r1,%s\n    wr [0xd78],#1\n' % restart
        body += ''.join('    movi r%d,#%s\n' % (i, pads[i]) for i in range(8))
        body += insn(0xf, 1, sel, 0) + '    wr [0xd74],#0x7f0\n    movi r6,#1\n    bnz r6,%s\n' % out
        for i, pad in enumerate(pads):
            body += '%s:\n    wr [0xd74],#0x%x\n    movi r6,#1\n    bnz r6,%s\n' % (pad, 0x7a0 + i, out)
        body += '%s:\n    wr [0xd74],#0x7af\n%s:\n' % (restart, out)
        yield Case('sel%04x' % sel, finish(t, body),
                   dict(params=[(0xd74, 0), (0xd78, 0)], flags=DISCOVERY, timeout_us=20000),
                   cond('1b', 'd74 = 0x7a0 + i = target from ri (0x7a5 for selector 5 = control; 0x7a0 = r0 as for '
                              'ldi); 0x7af = target 0 (program restart); 0x7f0 = no jump',
                        selector=hex(sel), condition='r1 (non-zero pad offset)'))


SPECS = {
    'm2_r4_bus_latch': (bus_latch, 'fields0,regsweep', '#1: high-bit source selectors after nine latch predecessors; discarded writes and the latch'),
    'm2_r4_undefined_scan': (undefined_scan, 'unknown,opstate', '#3: undefined opcodes 1b..fd with each pending flag'),
    'm2_r4_nop_operand': (nop_operand, 'op08', '#4: nop R = 0..255 idle; R = 8..15 with each pending flag'),
    'm2_r4_c48_bits': (c48_bits, 'ctrlwords,op15', '#4: tmwait with C48 single bits'),
    'm2_r4_c14_write': (c14_write, 'op15,tmobits', '#4: C14 program writes before tmwait (values, counts, repeats)'),
    'm2_r4_prescaler_origin': (prescaler_origin, 'tmobits', '#6: first-instant repeats, extra parameter writes, no reset'),
    'm2_r4_c08_running': (c08_running, 'c08self', '#8: C08 3 ms into single-instruction loops (TIMEOUT, run late)'),
    'm2_r4_c58_bits': (c58_bits, 'ctrlwords', '#8: C58 bits 1/4 on three timeout paths'),
    'm2_r4_c14_release': (c14_release, 'tmobits,startbits', '#8: C14 after a CPU-released WAIT0 after 0..4 steps'),
    'm2_r4_latch_refusal': (latch_refusal, 'ev5t', '#11: operations after an unreleased status latch'),
    'm2_r4_status_bits': (status_bits, 'fmcstat', '#19: FMC_STATUS 19:12 versus transfer length'),
    'm2_r4_data_sel_alias': (data_sel_alias, 'fmcbits', '#19: 014 low byte after 058/018/05c writes, both writers'),
    'm2_r4_reset_steps': (reset_steps, 'restore', '#26: FMCTRL1 0x800/0x80000000 and 0x81c stages on FMC/ECC words'),
    'm2_r4_ecc_814': (ecc_814, 'eccregs,rsresult', '#23: 0x814 bits on decode and encode (scratch page)'),
    'm2_r4_ecc_metalen': (ecc_metalen, 'eccpar', '#23: parity for metadata lengths 0..0xff'),
    'm2_r4_fifo_page_readback': (fifo_page_readback, 'fifo', 'M1-F164: buffer 0 after 0x340 + FIFO writes via page program'),
    'm2_r4_dma_align': (dma_align, 'dmaloop', '#20: unaligned DMA addresses (run late)'),
    'm2_r4_data_sel_targets': (data_sel_targets, 'dmactl,fmcbits', '#18: DMA in targets for FMC_DATA_SEL 0 and non-source values (run late)'),
    'm2_r4_dma_ctrl_count': (dma_ctrl_count, 'dmactl', '#16: preset 038 bits before a DMA (run late)'),
    'm2_r4_supersede_offset': (supersede_offset, 'dmactl,dmaloop', '#15: rewrite switch offset versus gap (run late)'),
    'm2_r4_jr_selector': (jr_selector, 'regsweep', '#1: jr with high target-selector bits, pads only (run last)'),
}

# Ordered least risky first: TIMEOUT observation, then DMAs that may stay pending/faulted (P25: overlapping DMAs
# may wedge), last the PC-from-selector cases (every target a label, restart guarded).
RUN_LAST = ('m2_r4_c08_running', 'm2_r4_dma_align', 'm2_r4_data_sel_targets', 'm2_r4_dma_ctrl_count',
            'm2_r4_supersede_offset', 'm2_r4_jr_selector')
