"""Round 5 of the M2 open-point families: the smallest safe measurements still open after round 4.

Sources: the p42 open items (the "unknown" entries and their
"smallest_measurement" proposals) and (items 1, 2, 3, 4, 6, 8, 15, 16,
18, 19, 23). Names m2_r5_<snake>. Every case carries the conditions `item` (chapter-17 item), `key` (the analyser
point, one per family), `open_point`, `decision` (the rule that turns the outcome into an answer) and, where the
outcome depends on a short observation window, `observation` (the run uses --observation-min-us 0).
The round rules classify each result.

Families and the question each settles:

- m2_r5_latch_clear (#1): movi (latch r0) or mov r3,r5 (latch r5), then ld / st / wr / nothing, then mov r2 with
  source selector 0x104: does ld/st/wr clear the high-bit source value to 0 or keep the earlier one?
- m2_r5_ext_dest (#1): mov r9, r11..r15 with source selector 8 (offset) and 10 (destination), full GPR dump and
  the extended-st latch: are the extended destinations discarded also with extended sources?
- m2_r5_sti_high (#1): sti with address selector 0x105 with every GPR holding the address of its own scratch
  marker (and after mov r2,r5): which register a high-bit sti address selector uses.
- m2_r5_rom_ldi (#2): ldi of 0x20000000 + k*0x1000 (k = 0..15) against fw/ipod4g_bootrom.bin: boot-ROM
  visibility inside 0x20000000..0x2000ffff.
- m2_r5_op1a_operands (#3): undefined opcode 0x1a with 64 seeded random R/arg/imm words against the zero form.
- m2_r5_nop_stall_c08 (#4): nop R = 8 / 0xff with start word 1 (timeout disabled), C08 read 3 ms into the stall
  against an unmet wait 3 and nop R = 0: is the stall a wait state (C08 = 4)?
- m2_r5_c48_count7 (#4): count-7 tmwait with C48 = 0..8: does it end at instant 8 - C48 with C14 = C48, and fault
  for C48 = 8 (compare rule)?
- m2_r5_tmwait_repeat (#4): count-255 tmwait with and without a C14 = 0x800 write, five runs each: frequency of
  the 13..24 us early ends.
- m2_r5_prescaler_step (#6): no-reset runs (n = 9, count 0) preceded by one CPU reset step (C00 = 0x80000,
  FMCTRL1 0x80000000, 0x81c = 1), against reset and no-step runs: which step sets the prescaler phase.
- m2_r5_select_tag (#15): FMC_DATA_SEL rewrite 16 / 32 nop2 into a DMA out with both buffers tagged per word,
  three runs each: the switch offset and the buffer-1 word (by tag) at the switch.
- m2_r5_overlap_tags (#15): the M2-394 out-then-out and in-then-in overlaps (gap 0 / 16) with tagged buffers:
  which input every word of the outputs comes from.
- m2_r5_dma_ctrl_sel (#16): DMA out after 038 = 0x1f, 0x2f, 0x37 (bits 5:3 = 3, 5, 6), DMA in after 0xf, control 7.
- m2_r5_out_select (#18): DMA out with FMC_DATA_SEL = 0, 1, 0x300, 0x400, 0x1100 (control 0x100), all four
  buffers tagged: which buffer each select sources.
- m2_r5_x340_window (#18): after FMCTRL1 0x340, 2000 delay loops (time-driven completion?) or a 014 = 0x100 write
  after 0 / 8 / 16 nop2 (stop at the write?), then a page program from buffer 0 (scratch block, MUTATE) and a raw
  read of each page.
- m2_r5_status_lengths (#19): FMC_STATUS bits 19:12 after data reads of 160..508 bytes of an open page.
- m2_r5_bch16_814 (#23): BCH16 buffer encode and buffer decode (host BCH16 code word with 12 data errors loaded by
  DMA into data/parity buffer 0) with 0x814 = 0x2f against 0x814 = 0.
- RUN_LAST: the TIMEOUT observations (nop_stall_c08, c08_long), the DMA families that may leave a DMA pending
  (P25), random operand words, extended destinations, ROM ldi and the high-bit sti.

Safety conventions (two earlier runs lost the device): every register used as an address, or pickable by a
high-bit selector (r0 in particular), holds a scratch address (CPU parameters and movi); the one exception is
m2_r5_rom_ldi, whose ldi address register holds 0x2000xxxx (read only, as M2-432); no write reaches r8/r10 and
no jr is used, so no value reaches the PC except program flow; DMA addresses are inside scratch; C60..C6C are
never written; changed control words (C48, 038, 814, 014) are restored at the END and, where a fault can come,
in the handler; NAND erases/programs only on the scratch geometry (block g.block) and only with MUTATE.
Start word count<<8|n<<4|5; C58 = 4 vectors a fault to C4C, 0 continues.
"""
import random

from . import followup_experiments as F, sequences as Q  # noqa: F401
from .protocol import DISCOVERY, MUTATE
from fmss import probe as P

Case, finish, memory, delay, NOP = F.Case, F.finish, F.memory, F.delay, F.NOP
MASK = 0xffffffff
SHORT = 0x07b5                     # unmet WAIT faults at ~4.39 ms
START1 = 0x01b5                    # count 1: an unmet wait faults ~1433 us
LONG = 0xffb5
MARK1, MARK2 = '    movi r2,#0x77770001\n', '    movi r2,#0x77770002\n'
ACK2 = Q.wr(0x48, 0x100000)
STEP_US = 492.747
SCRATCH_BYTES = 0x4000              # every address used lies in the initialised 16 KiB
SEED_BASE = 0x3000                  # GPR i seed = scratch + 0x3000 + 0x10 * i
SHORT_WINDOW = ('the outcome is read 3 ms into the run: valid only with --observation-min-us 0 (a 160 ms floor '
                'would read the after-map later)')

ITEMS = {
    '1a': ('1', 'whether ld, st or wr set the value a high-bit source selector reads to 0 or keep the earlier one'),
    '1b': ('1', 'extended destinations other than r8/r10 combined with extended sources 8 and 10'),
    '1c': ('1', 'sti with a high address-selector bit'),
    '2': ('2', 'boot ROM visibility at other addresses of 0x20000000..0x2000ffff'),
    '3': ('3', 'operand values of the undefined opcodes outside the scanned forms'),
    '4a': ('4', 'the condition behind the nop stall for R >= 8 (wait state or not)'),
    '4b': ('4', 'C48 bits 7:0 on a tmwait of a count other than 1'),
    '4c': ('4', 'the isolated 13..24 us early tmwait ends'),
    '6': ('6', 'which step of the per-case reset sets the prescaler phase'),
    '8': ('8', 'what the C08 values 1, 2 and 9 read while a program runs encode'),
    '15a': ('15', 'the select-switch spread at 16 and 32 nop2 and the fixed buffer-1 word at the switch'),
    '15b': ('15', 'bytes matching neither input when a complete second DMA is written over a running one'),
    '16': ('16', '038 bits 5:3 = 3, 5, 6 on a DMA out, bits 5:3 = 1 on a DMA in'),
    '18a': ('18', 'DMA-out FMC_DATA_SEL values other than the source words'),
    '18b': ('18', 'what starts and what stops the second transfer after FMCTRL1 0x340'),
    '19': ('19', 'FMC_STATUS bits 19:12 for lengths 129..255, 257..383, 385..510'),
    '23': ('23', 'whether 0x814 acts on BCH16'),
}

SKIPPED = {
    '#2 PC width above 2^20; fetch from invalid or unaligned targets': (
        'NOT_MEASURABLE safely: a non-wrapping jump above 2^20 or to an invalid target executes unbounded memory; '
        'every value that can reach the PC must be a program label.'),
    '#2 ldi outside 0x20000000..0x2000ffff': (
        'outside the boot-ROM window an address may be unmapped (bus hang, M1-F059 / M2-611); only the ROM window '
        'is read (m2_r5_rom_ldi).'),
    '#6 origin of the 0.25-cycle fractions; mechanism of the count + 2 / count + 1 instants': (
        'no observable beyond the fault/end times already measured (M2-584, M2-631).'),
    '#7 final instruction of the two USB losses': (
        'nothing survives a loss or a reset; the unmapped-ldi candidate is the separate reboot test m2_m1_bus_error.'),
    '#8 full bit maps of C00 and C5C': 'C00 writes start, stop or reset the engine; C5C has no bounded stimulus.',
    '#8 C14 = 0x7f constant or held value': 'not separable: C14 ignores writes (M2-177).',
    '#9 sources of WAIT 6..10, 13..15': 'no untested stimulus inside the probe-writable window (26 kinds time out).',
    '#10 condition of WAIT11': 'passes in every reachable state.',
    '#11 latch 1..2 instructions after an unwaited 70h sets bit 3': (
        'the window is measured to one instruction; 048 sampling (188 ns) is coarser.'),
    '#13 VIC0 line 5 device, VIC1 hardware lines, ECC interrupt': (
        'needs a non-FMSS interrupt source; wait2 32 + k against VIC1 is the separate key m2_m1_wait2_vic1.'),
    '#16 why non-zero bits 12:6 or bits 5:3 = 2/4/7 stop a DMA after 32 bytes; burst/word-count meaning': (
        'no further observable than completion and delivered bytes (m2_r5_dma_ctrl_sel measures the remaining '
        'bits 5:3 values).'),
    '#18 roles of 018, the 0x100 block, FMC_CTRL0 bit 0': 'no bounded observable left (M2-104, M2-135, M2-405).',
    '#19 combined 004 operation bits': 'not bounded: unknown combinations may start unbounded transfers.',
    '#19 other setters of bit 26': 'no candidate operation left after M2-641.',
    '#20 DMA to / ldi from an unmapped bus address; cache coherency': (
        'unmapped addresses are the separate reboot test m2_m1_bus_error (M1-F059); coherency is not observable '
        'by the host.'),
    '#21 M1 channel-1 pins; a second channel elsewhere': 'pin-mux and clock-gate writes are outside the probe window.',
    '#23 810 bits 13..15, 17..23, 25..30; 80c bits outside the count': 'no stimulus sets them (M2-420, M2-91).',
    '#23 BCH16 decode from a NAND page': (
        'the Hynix spare (128 B) cannot hold eight BCH16 code words; m2_r5_bch16_814 decodes a host BCH16 code word '
        'loaded into the buffers by DMA instead (no NAND write).'),
    '#24 original WRITE/ERASE contracts': 'the harness runs no Apple microcode.',
}


def cond(key, decision, **extra):
    item, text = ITEMS[key]
    return dict(item=item, key=key, open_point=text, decision=decision, **extra)


def insn(op, reg=0, arg=0, imm=0):
    return '    .insn 0x%x,%d,0x%x,0x%x\n' % (op, reg, arg, imm & MASK)


def start_word(count, n=0xb):
    return (count & 0xff) << 8 | (n & 0xf) << 4 | 5


def trap_with(t, extra):
    return t.trap().replace('trap:\n', 'trap:\n' + extra)


def stores(regs, base):
    return ''.join(Q.store(off, base + 4 * k) for k, off in enumerate(regs))


def loop(n, body):
    F._LABEL[0] += 1
    lp = 'lp%d' % F._LABEL[0]
    return '    movi r4,#%d\n%s:\n%s    sub r4,r4,#1\n    bnz r4,%s\n' % (n, lp, body, lp)


def seed_values(a):
    return [a + SEED_BASE + 0x10 * i for i in range(8)]


def seeds(a):
    """CPU parameters: every GPR holds a scratch address (also r0, the high-bit selector's register)."""
    return [(0xc18 + 4 * i, v) for i, v in enumerate(seed_values(a))]


def tagged(tags):
    """memory() with 128 words tag | k at each (offset, tag)."""
    mem = memory()
    for off, tag in tags:
        for k in range(128):
            mem[off + 4 * k:off + 4 * k + 4] = (tag | k).to_bytes(4, 'little')
    return mem


TAG_D0, TAG_D1, TAG_P0, TAG_P1 = 0x7a000000, 0x7b000000, 0x7c000000, 0x7d000000


# ---- m2_r5_latch_clear (#1)
LATCH_PREDS = ('movi', 'mov')
LATCH_MIDS = {'none': '', 'ld': '    ld r3,[0xd60]\n', 'st': '    st [0xd64],r4\n', 'wr': '    wr [0xd68],#0x1234\n'}
LATCH_D60 = 0x0d600d60


def latch_clear(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    sv = seed_values(a)
    pre = {'movi': ('    movi r3,#0x%x\n' % (a + 0x3100), sv[0], 'r0'), 'mov': (insn(6, 3, 5, 0), sv[5], 'r5')}
    for pred in LATCH_PREDS:
        x, kept, src = pre[pred]
        for mid, y in LATCH_MIDS.items():
            predict = {'kept (%s)' % src: kept, 'cleared': 0}
            if mid == 'ld':
                predict['loaded value'] = LATCH_D60
            if mid == 'st':
                predict['st source (r4)'] = sv[4]
            body = x + y + insn(6, 2, 0x104, 0)
            yield Case('%s/%s' % (pred, mid), finish(t, body),
                       dict(params=seeds(a) + [(0xd60, LATCH_D60), (0xd64, 0), (0xd68, 0)], flags=DISCOVERY,
                            timeout_us=20000),
                       cond('1a', 'r2 equal to the predecessor value = ld/st/wr keep it; 0 = they clear it; the '
                                  '"none" controls must read the predecessor value', observe='r2', predict=predict,
                            predecessor=pred, between=mid, selector='0x104', instruction=body.strip(),
                            seeds='GPR i = scratch + 0x%x + 0x10 i' % SEED_BASE))


# ---- m2_r5_ext_dest (#1)
EXT_DESTS = (9, 11, 12, 13, 14, 15)
EXT_SOURCES = (8, 10)


def ext_dest(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    latch = a + 0x3100
    for d in EXT_DESTS:
        for src in EXT_SOURCES:
            body = '    movi r3,#0x%x\n' % latch + insn(6, d, src, 0) + insn(2, 9, 0xd6c, 0)
            source = finish(t, body)
            code = P.assemble(t, source)
            idx = next(i for i in range(0, len(code), 8)
                       if code[i + 3] == 6 and code[i + 2] == d)
            yield Case('r%02d/src%02d' % (d, src), source,
                       dict(params=seeds(a) + [(0xd6c, 0)], flags=DISCOVERY, timeout_us=20000),
                       cond('1b', 'r0..r5, r7 equal to the seeds (r3 = the movi value) and END = the write is '
                                  'discarded; a changed GPR names a broadcast/alias; d6c (extended st = latch) equal to '
                                  'the movi value = the discarded write leaves the latch, other = recorded (source 8 '
                                  'relative to the mov offset)', dest=d, source=src, mov_offset=idx,
                            latch_before=latch, seeds=[hex(v) for v in seed_values(a)],
                            note='M2-43: other extended destinations discard for movi/mov/add; the sources reaching '
                                 'them are the instruction offset (8) or the destination selector (10)'))


# ---- m2_r5_sti_high (#1)
STI_CASES = (('sel005', 5, ''), ('sel105', 0x105, ''), ('sel105/after-mov', 0x105, insn(6, 2, 5, 0)))
MARKER = 0x5e000000


def sti_high(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    slots = [a + 0x2000 + 0x10 * i for i in range(8)]
    mem = memory()
    for i in range(8):
        mem[0x2000 + 0x10 * i:0x2004 + 0x10 * i] = (MARKER | i).to_bytes(4, 'little')
    for name, sel, pre in STI_CASES:
        body = ''.join('    movi r%d,#0x%x\n' % (i, v) for i, v in enumerate(slots)) + pre + insn(0x11, 1, sel, 0)
        yield Case(name, finish(t, body),
                   dict(params=[(0xc18 + 4 * i, v) for i, v in enumerate(slots)], scratch=mem, scratch_read=0x2080,
                        flags=DISCOVERY, timeout_us=20000),
                   cond('1c', 'the slot whose marker 0x5e00000i is replaced by r1 (= its own address + 0x10) names the '
                              'address register: slot 0 = r0 (as ldi, M2-615); slot 5 after mov r2,r5 = the last '
                              'source value; sel005 is the control (slot 5); no slot changed = store elsewhere',
                        selector=hex(sel), before=pre.strip() or None, value_register='r1', stored=slots[1],
                        layout='0x2000 + 0x10 i: marker of ri (ri = its address)'))


# ---- m2_r5_rom_ldi (#2)
ROM_BASE = 0x20000000
ROM_OFFSETS = tuple(k * 0x1000 for k in range(16))


def rom_ldi(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for off in ROM_OFFSETS:
        body = ('    movi r5,#0x%x\n    ldi r1,[r5]\n    movi r7,#0x%x\n    sti [r7],r1\n' % (ROM_BASE + off, a + 0x2000))
        yield Case('o%05x' % off, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2004, flags=DISCOVERY, timeout_us=20000),
                   cond('2', 'word equal to fw/ipod4g_bootrom.bin at the offset = the ROM is visible there; '
                             '0xe1a0f00e / other = not the ROM (M2-432)', address=hex(ROM_BASE + off), offset=off,
                        layout='0x2000 the ldi word'))


# ---- m2_r5_op1a_operands (#3)
def op1a_words(n=64, seed=0x1a5):
    rnd, out = random.Random(seed), []
    while len(out) < n:
        R, arg, imm = rnd.randrange(256), rnd.randrange(0x10000), rnd.getrandbits(32)
        if R in (8, 10) or 0xc00 <= arg & 0xfff < 0xc80:    # no PC/broadcast destination, no control offsets
            continue
        out.append((R, arg, imm))
    return out


def op1a_operands(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    plan = [('zero', (0, 0, 0))] + [('rnd%02d' % k, w) for k, w in enumerate(op1a_words())]
    for name, (R, arg, imm) in plan:
        body = MARK1 + insn(0x1a, R, arg, imm) + MARK2
        yield Case(name, finish(t, body),
                   dict(start_value=START1, params=seeds(a), flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   cond('3', 'fault at the count-1 wait timeout with r2 0x77770001 and the seeds unchanged = the '
                             'zero-form outcome (unmet wait); any other outcome names an operand effect',
                        R=R, arg=hex(arg), imm=hex(imm), start_word=hex(START1),
                        seeds=[hex(v) for v in seed_values(a)]))


# ---- m2_r5_nop_stall_c08 (#4)
NOP_STALL = (('r08', insn(8, 8)), ('rff', insn(8, 0xff)), ('r00', insn(8, 0)), ('wait3', Q.wait(3)))


def nop_stall_c08(s, t):
    F._LABEL[0] = 0
    for name, x in NOP_STALL:
        yield Case(name, finish(t, MARK1 + x + MARK2),
                   dict(start_value=1, flags=DISCOVERY, timeout_us=3000),
                   cond('4a', 'TIMEOUT with after-map C08 = 4 and r2 0x77770001 = the stall is a wait state (as the '
                              'wait3 control); another C08 value = a different state; r00 must END',
                        instruction=x.strip(), start_word='0x1', observe_us=3000, observation=SHORT_WINDOW))


# ---- m2_r5_c48_count7 (#4)
C48_COUNT7 = tuple(range(9))


def c48_count7(s, t):
    F._LABEL[0] = 0
    for v in C48_COUNT7:
        body = MARK1 + insn(0x15) + MARK2 + '    ld r5,[0xc14]\n' + Q.wr(0xc48, 0)
        src = t.prologue(4) + body + Q.wr(0, 0) + '    movi r6,#0x600d600d\n    end\n' + trap_with(t, Q.wr(0xc48, 0))
        yield Case('c48-%d' % v, src,
                   dict(start_value=start_word(7), params=[(0xc48, v)], flags=P.WAIT_END, timeout_us=20000),
                   cond('4b', 'compare rule: C48 = 1..7 end at instant 8 - C48 (450 + 492.747 (7 - C48) us) with r5 '
                              '(C14) = C48, C48 = 0 at instant 8 with C14 = 0, C48 = 8 faults at the wait timeout; any '
                              'other end/C14 contradicts it', c48=v, count=7, start_word=hex(start_word(7))))


# ---- m2_r5_tmwait_repeat (#4)
TMWAIT_VARIANTS = ('none', 'c14-800')
TMWAIT_REPS = 5


def tmwait_repeat(s, t):
    F._LABEL[0] = 0
    for v in TMWAIT_VARIANTS:
        for rep in range(TMWAIT_REPS):
            w = Q.wr(0xc14, 0x800) if v != 'none' else ''
            yield Case('%s/rep%d' % (v, rep), finish(t, MARK1 + w + insn(0x15) + MARK2 + '    ld r5,[0xc14]\n'),
                       dict(start_value=start_word(0xff), flags=P.WAIT_END, timeout_us=200000),
                       cond('4c', 'end times of five runs per variant: an early end (>= 10 us below the median) in '
                                  'both variants = run-to-run spread; only with 0x800 = a write effect',
                            variant=v, rep=rep, count=0xff))


# ---- m2_r5_prescaler_step (#6)
PRESCALER_STEPS = {'c00-80000': [(0xc00, 0x80000)], 'ctrl1-80000000': [(4, 0x80000000)], 'ecc-81c': [(0x81c, 1)]}


def prescaler_step(s, t):
    F._LABEL[0] = 0
    body = MARK1 + Q.wait(3) + MARK2
    plan = [('reset/rep%d' % r, [], 0) for r in range(2)] + [('noreset/none/rep%d' % r, [], P.S_NO_RESET)
                                                            for r in range(2)]
    plan += [('noreset/%s/rep%d' % (k, r), w, P.S_NO_RESET) for k, w in PRESCALER_STEPS.items() for r in range(2)]
    for name, writes, steps in plan:
        yield Case(name, finish(t, body),
                   dict(start_value=start_word(0, 9), params=list(writes), flags=DISCOVERY | P.WAIT_END | steps,
                        timeout_us=20000),
                   cond('6', 'count-0 n = 9 first instant: the step whose no-reset runs reproduce the reset instant '
                             '(78 us, M2-631, within 2 us) sets the prescaler phase; noreset/none moves (control)',
                        reset=not steps, step=name.split('/')[1] if steps else 'per-case reset',
                        writes=[[hex(o), hex(v)] for o, v in writes], start_word=hex(start_word(0, 9))))


# ---- m2_r5_c08_long (#8)
C08_LOOPS = {'nop2': ('', NOP, 730000), 'ld': ('', '    ld r2,[0x48]\n', 383000),
             'sti': (None, '    sti [r7],r1\n', 245000)}


def c08_long(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for kind, (pre, one, n) in C08_LOOPS.items():
        pre = '    movi r7,#0x%x\n' % (a + 0x100) if pre is None else pre
        for rep in range(3):
            yield Case('%s/rep%d' % (kind, rep), finish(t, pre + loop(n, one * 16)),
                       dict(start_value=LONG, flags=DISCOVERY, timeout_us=3000, scratch=memory(), scratch_read=16),
                       cond('8', 'TIMEOUT with one C08 value in all three runs of a kind, differing between kinds = '
                                 'C08 encodes the executing instruction; varying within a kind = sampling phase',
                            kind=kind, rep=rep, iterations=n, loop_ms=400, observe_us=3000,
                            observation=SHORT_WINDOW + '; the 400 ms loop is still running at a 160 ms floor too'))


# ---- m2_r5_select_tag (#15)
SELECT_GAPS = (16, 32)


def select_tag(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for gap in SELECT_GAPS:
        for rep in range(3):
            body = (Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1) + Q.wr(0x38, 7) + Q.wr(0x30, 511)
                    + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100) + NOP * gap + Q.wr(0x14, 0x200)
                    + Q.wait(2) + stores((0x48, 0x38), a + 0x2600) + ACK2 + delay(300) + Q.wr(0x14, 0))
            yield Case('gap%02d/rep%d' % (gap, rep), finish(t, body, 0),
                       dict(scratch=tagged([(0, TAG_D0), (0x400, TAG_D1)]), scratch_read=0x2608, start_value=SHORT,
                            flags=P.WAIT_END, timeout_us=50000),
                       cond('15a', 'output word k = 0x7a00000k (buffer 0) before the switch, 0x7b00000k (buffer 1) '
                                   'after; a 0x7b0000jj word with jj != k names the fixed buffer-1 word and its offset',
                            gap_nops=gap, rep=rep, tags='buffer 0 0x7a000000|k, buffer 1 0x7b000000|k',
                            layout='0x2000 DMA out target; 0x2600: 048 038'))


# ---- m2_r5_overlap_tags (#15)
OVERLAP_GAPS = (0, 16)


def overlap_tags(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    mem = tagged([(0, TAG_D0), (0x400, TAG_D1)])
    decision = ('each output word classified by tag (0x7a = first input/buffer 0, 0x7b = second/buffer 1, word '
                'index = source offset), initial scratch, zero or other: names the bytes matching neither input')
    for gap in OVERLAP_GAPS:
        body = Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1)
        body += Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
        body += NOP * gap + Q.dma(a + 0x2400, 1, into=False) + stores((0x38, 0x48), a + 0x2800) + Q.wr(0x14, 0)
        yield Case('out/gap%02d' % gap, finish(t, body, 0),
                   dict(scratch=mem, scratch_read=0x2810, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('15b', decision, direction='out', gap_nops=gap,
                        layout='0x2000 first DMA out target (P), 0x2400 second (Q), 0x2800: 038 048'))
    for gap in OVERLAP_GAPS:
        body = Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(0x34, a) + Q.wr(4, 0x2e0)
        body += NOP * gap + Q.dma(a + 0x400, 1) + stores((0x38, 0x48), a + 0x2800) + ACK2
        body += Q.dma(a + 0x2000, 0, into=False) + Q.dma(a + 0x2400, 1, into=False) + Q.wr(0x14, 0)
        yield Case('in/gap%02d' % gap, finish(t, body, 0),
                   dict(scratch=mem, scratch_read=0x2810, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('15b', decision, direction='in', gap_nops=gap,
                        layout='0x2000 buffer 0 read back, 0x2400 buffer 1, 0x2800: 038 048 after the second input'))


# ---- m2_r5_dma_ctrl_sel (#16)
CTRL_SEL = (('out/w07', 7), ('out/w1f', 0x1f), ('out/w2f', 0x2f), ('out/w37', 0x37), ('in/w0f', 0xf))


def dma_ctrl_sel(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for name, w in CTRL_SEL:
        if name.startswith('out'):
            body = (Q.setup() + Q.dma(a, 0) + Q.wr(0x38, w) + stores((0x38,), a + 0x2400) + Q.wr(0x30, 511)
                    + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100) + Q.wait(2)
                    + stores((0x38, 0x48), a + 0x2404) + ACK2 + delay(300) + stores((0x38,), a + 0x240c) + Q.wr(0x38, 7))
        else:
            x = Q.dma(a, 0, control=None).replace(ACK2, stores((0x38, 0x48), a + 0x2404) + ACK2)
            body = (Q.setup() + Q.wr(0x38, w) + stores((0x38,), a + 0x2400) + x + delay(300)
                    + stores((0x38,), a + 0x240c) + Q.wr(0x38, 7) + Q.dma(a + 0x2000, 0, into=False))
        yield Case(name, finish(t, body + Q.wr(0x38, 7), 0, trap_with(t, Q.wr(0x38, 7))),
                   dict(scratch=tagged([(0, TAG_D0)]), scratch_read=0x2410, start_value=SHORT, flags=P.WAIT_END,
                        timeout_us=50000),
                   cond('16', 'completion (no fault, 048 bit 20) and the words at 0x2000: exact tags = completes '
                              'exactly; first 32 bytes only = stops as bits 5:3 = 2/4/7; groups of four copies = as '
                              'bits 5:3 = 1 (M2-669); the in case is read back with 038 = 7',
                        value=hex(w), bits5_3=(w >> 3) & 7, direction=name.split('/')[0],
                        layout='0x2000 DMA target / read-back; 0x2400: 038 after the write, 0x2404 038 048 after wait '
                               '2, 0x240c 038 later'))


# ---- m2_r5_out_select (#18)
OUT_SELECTS = (0x100, 0, 1, 0x300, 0x400, 0x1100)


def out_select(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    mem = tagged([(0, TAG_D0), (0x400, TAG_D1), (0x800, TAG_P0), (0xc00, TAG_P1)])
    for v in OUT_SELECTS:
        body = (Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x400, 1) + Q.dma(a + 0x800, select=0x10)
                + Q.dma(a + 0xc00, select=0x20))
        body += (Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, v) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100) + Q.wait(2)
                 + stores((0x48, 0x38), a + 0x3000) + ACK2 + delay(300) + Q.wr(0x14, 0) + Q.wr(0x38, 7))
        yield Case('sel-%04x' % v, finish(t, body, 0, trap_with(t, Q.wr(0x14, 0) + Q.wr(0x38, 7))),
                   dict(scratch=mem, scratch_read=0x3008, start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                   cond('18a', 'words at 0x2000 by tag: 0x7a data 0, 0x7b data 1, 0x7c parity 0 (source words '
                               '112..127), 0x7d parity 1; initial scratch = nothing delivered; 0x100 is the control',
                        select=hex(v), layout='0x2000 DMA out target; 0x3000: 048 038 after wait 2'))


# ---- m2_r5_x340_window (#18)
X340_VARIANTS = ('delay2000', 'sel-gap00', 'sel-gap08', 'sel-gap16')
X340_SINK = 0x2800
X340_TAG = 0xc3d40000


def x340_window(s, t):
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    mutate = dict(flags=MUTATE, timeout_us=200000)
    mem = tagged([(0, X340_TAG)])
    decision = ('page words k: RAM S + 0x200 + 4k (S = 0x%x) = moved by the second transfer, tag 0xc3d4000k = buffer '
                'content before; delay2000 with all 128 RAM words = the transfer completes alone (time-driven); '
                'sel-gapNN with fewer RAM words growing with the gap = it stops at the 014 write' % X340_SINK)
    yield Case('erase', finish(t, Q.erase(g)), dict(mutate), cond('18b', 'scratch block erase', block=g.block), True)
    for page, v in enumerate(X340_VARIANTS):
        body = Q.setup(g.ce) + Q.dma(a, 0)
        body += (Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 1) + Q.wr(0x34, a + X340_SINK) + Q.wr(4, 0x340)
                 + Q.wait(2) + ACK2)
        if v == 'delay2000':
            body += delay(2000)
        else:
            body += NOP * int(v[-2:]) + Q.wr(0x14, 0x100) + delay(2000)
        body += stores((0x38, 0x48, 0x3c), a + 0x2c00)
        body += Q.command(0x80) + Q.address(g.row + page) + Q.transfer(512, 0, into=False)
        body += stores((0x38, 0x48, 0x3c), a + 0x2c10) + Q.command(0x10) + Q.ready() + '    ld r5,[0x4c]\n'
        yield Case('program/%s' % v, finish(t, body + Q.wr(0x14, 0), 0),
                   dict(mutate, scratch=mem, scratch_read=0x2c20, start_value=SHORT),
                   cond('18b', decision, variant=v, row=g.row + page, sink=hex(X340_SINK),
                        layout='0x2800 0x340 sink; 0x2c00: 038 048 03c before the data-out set-up, 0x2c10 after the '
                               'data-out transfer; r5 = 04c after 10h'), True)
    for page, v in enumerate(X340_VARIANTS):
        body = (Q.setup(g.ce) + Q.command(0) + Q.address(g.row + page) + Q.command(0x30) + Q.ready() + Q.command(0)
                + Q.transfer(512) + Q.dma(a + 0x2000, 0, into=False))
        yield Case('read/%s' % v, finish(t, body),
                   dict(scratch=memory(), scratch_read=0x2200, timeout_us=200000),
                   cond('18b', decision, variant=v, row=g.row + page, layout='0x2000 page bytes 0..511 (raw)'), True)


# ---- m2_r5_status_lengths (#19)
STATUS_LENGTHS = (160, 192, 224, 255, 257, 320, 383, 385, 448, 480, 508)


def status_lengths(s, t):
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    opened = Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
    for L in STATUS_LENGTHS:
        x = (Q.wr(0x30, L - 1) + Q.wr(0x14, 1) + Q.wr(4, 0xe2) + Q.wait(3) + stores((0x48,), a + 0x2004)
             + Q.wr(0x48, 8) + stores((0x48,), a + 0x2008))
        yield Case('data/L%03d' % L, finish(t, opened + stores((0x48,), a + 0x2000) + x),
                   dict(scratch=memory(), scratch_read=0x2010, start_value=SHORT, timeout_us=50000),
                   cond('19', 'bits 19:12 of 048 after wait 3 (0x2004) per length, with the round-4 values (0x41 up to '
                              '128, 0x14 at 256/384, 0x3c at 511/512, M2-642): the boundaries name the encoding',
                        length=L, layout='0x2000: 048 before, 0x2004 after wait 3, 0x2008 after the ack'), True)


# ---- m2_r5_bch16_814 (#23)
BCH16_ERRORS = tuple((k * 977 + 7) % 4096 for k in range(12))
V814_16 = (0, 0x2f)
DATA16 = 0x1000                      # erroneous data copy; clean data = memory()[0x1000:0x1200]
PAR16 = 0x1400                       # 512-byte parity source: parity at bytes 448..473 (parity buffer = last 64)


def bch16_memory():
    mem = memory()
    clean = bytes(mem[DATA16:DATA16 + 512])
    par = F.bch16_parity(clean)
    mem[DATA16:DATA16 + 512] = F.flip(clean, BCH16_ERRORS)
    mem[PAR16:PAR16 + 512] = (b'\xff' * 448 + par).ljust(512, b'\xff')
    return mem


def bch16_814(s, t):
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    for v in V814_16:
        body = (Q.setup() + Q.wr(0x814, v) + Q.dma(a, 0) + Q.ecc(0, True, 16, 0, acknowledge=False) + Q.wait(4)
                + stores(range(0x840, 0x860, 4), a + 0x2000) + stores((0x810, 0x814), a + 0x2020) + Q.wr(0x810, 1)
                + Q.wr(0x814, 0))
        yield Case('encode/814-%02x' % v, finish(t, body, 0, trap_with(t, Q.wr(0x814, 0))),
                   dict(scratch=memory(), scratch_read=0x2028, start_value=SHORT, flags=P.WAIT_END, timeout_us=20000),
                   cond('23', 'BCH16 parity 0x840..0x85c equal for 814 = 0 and 0x2f (and to host BCH16 of the 512 data '
                              'bytes) = 814 has no BCH16 encode effect', value=hex(v), code='bch16',
                        layout='0x2000: 840..85c; 0x2020: 810 814'))
    for v in V814_16:
        body = (Q.setup() + Q.wr(0x814, v) + Q.dma(a + DATA16, 0) + Q.dma(a + PAR16, select=0x10)
                + Q.wr(0x810, 0x1ff) + Q.wr(0x1c, 0) + Q.wr(0x804, 5) + Q.wait(4)
                + stores((0x810, 0x80c, 0x818, 0x814), a + 0x2000) + Q.wr(0x810, 1)
                + Q.dma(a + 0x2400, 0, into=False) + Q.wr(0x814, 0))
        yield Case('decode/814-%02x' % v, finish(t, body, 0, trap_with(t, Q.wr(0x814, 0))),
                   dict(scratch=bch16_memory(), scratch_read=0x2600, start_value=SHORT, flags=P.WAIT_END,
                        timeout_us=20000),
                   cond('23', 'buffer decode of a host BCH16 code word with 12 data errors: 810/80c/818 and the decoded '
                              '512 bytes equal for 814 = 0 and 0x2f = no BCH16 decode effect; decoded = clean shows '
                              'the buffer-loaded decode corrects', value=hex(v), code='bch16',
                        errors=list(BCH16_ERRORS),
                        layout='0x1000 data with errors, 0x1400 parity source (bytes 448..473); 0x2000: 810 80c 818 '
                               '814; 0x2400 decoded buffer 0'))


SPECS = {
    'm2_r5_latch_clear': (latch_clear, 'fields0,regsweep', '#1: ld/st/wr between a latch predecessor and a high-bit source'),
    'm2_r5_ext_dest': (ext_dest, 'regsweep', '#1: mov r9/r11..r15 with sources 8 and 10 (run late)'),
    'm2_r5_sti_high': (sti_high, 'regsweep', '#1: sti with selector 0x105, every GPR its own scratch marker (run last)'),
    'm2_r5_rom_ldi': (rom_ldi, 'unknown', '#2: ldi of 0x20000000 + k*0x1000 against the ROM image (run late)'),
    'm2_r5_op1a_operands': (op1a_operands, 'unknown,opstate', '#3: 0x1a with 64 random operand words (run late)'),
    'm2_r5_nop_stall_c08': (nop_stall_c08, 'op08,c08self', '#4: C08 inside the nop R >= 8 stall, timeout disabled (run late)'),
    'm2_r5_c48_count7': (c48_count7, 'ctrlwords,op15', '#4: count-7 tmwait with C48 = 0..8'),
    'm2_r5_tmwait_repeat': (tmwait_repeat, 'op15,tmobits', '#4: count-255 tmwait with/without C14 = 0x800, five runs'),
    'm2_r5_prescaler_step': (prescaler_step, 'tmobits', '#6: no-reset first instant after one CPU reset step'),
    'm2_r5_c08_long': (c08_long, 'c08self', '#8: C08 3 ms into 400 ms single-kind loops (run late)'),
    'm2_r5_select_tag': (select_tag, 'dmactl,dmaloop', '#15: select rewrite at 16/32 nop2 with tagged buffers (run late)'),
    'm2_r5_overlap_tags': (overlap_tags, 'dmactl,dmaloop', '#15: out-out / in-in overlaps with tagged buffers (run late)'),
    'm2_r5_dma_ctrl_sel': (dma_ctrl_sel, 'dmactl', '#16: 038 bits 5:3 = 3/5/6 out, 1 in (run late)'),
    'm2_r5_out_select': (out_select, 'dmactl,fmcbits', '#18: DMA out with non-source FMC_DATA_SEL (run late)'),
    'm2_r5_x340_window': (x340_window, 'fifo', '#18: what completes / stops the transfer after 0x340 (scratch page)'),
    'm2_r5_status_lengths': (status_lengths, 'fmcstat', '#19: FMC_STATUS 19:12 at more read lengths'),
    'm2_r5_bch16_814': (bch16_814, 'eccregs', '#23: 0x814 = 0x2f on BCH16 buffer encode and decode'),
}

# Least risky first: TIMEOUT observations (reset of a stalled/running engine), then DMAs that may stay pending or
# overlap (P25), random operand words, extended destinations, ROM reads, last the high-bit sti.
RUN_LAST = ('m2_r5_nop_stall_c08', 'm2_r5_c08_long', 'm2_r5_dma_ctrl_sel', 'm2_r5_out_select', 'm2_r5_select_tag',
            'm2_r5_overlap_tags', 'm2_r5_op1a_operands', 'm2_r5_ext_dest', 'm2_r5_rom_ldi', 'm2_r5_sti_high')
