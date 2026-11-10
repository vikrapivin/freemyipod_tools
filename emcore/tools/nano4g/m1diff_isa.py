"""M1-versus-M2 experiments (m1diff_isa): one family per comparison.

Each family settles the M2 side of the listed M1 findings; the decision rule is in
the key's crosswalk entry and the case conditions.

Construction facts used (settled on M2, not re-measured here): start word
count<<8|0xb5 with fault ~= 492.747 us * (count + 1) + 447.5 us (bits 7:4 = 0xb);
C58 = 4 vectors a timeout to C4C, C58 = 0 continues; C58 stores mask 0xd6;
pause (12) resumes with PC + 8, CS_IRQ bit 3 acknowledged and C00 = 0x4ffb5;
pld of 0x1d64 reads 0xd64; JR jumps iff r[R] != 0; the probe cleanup reset runs
after every case (C58 survives resets, the cleanup writes C58 = C4C = 0).
Unknown outcomes are discovery records; M1 predictions are kept as
`m1_prediction`, never as M2 expectations.
"""
import struct

from . import followup_experiments as F, sequences as Q  # noqa: F401
from .protocol import DISCOVERY
from fmss import probe as P  # noqa: F401

MASK = 0xffffffff
SEEDS = [0x11111111 * (n + 1) for n in range(8)]
SHORT = 0x07b5           # count 7: WAIT/illegal fault ~= 4.39 ms
F_KEEP_CTRL0, F_SAMPLE_REGS = 1, 4   # probe request flags (fmss.probe F_KEEP_CTRL0 / F_SAMPLE_REGS)


def cond(m1, decision, **extra):
    return dict(m1=list(m1), decision=decision, **extra)


def insn(op, reg=0, arg=0, imm=0):
    value = imm if isinstance(imm, str) else '0x%x' % (imm & MASK)
    return '    .insn 0x%x,%d,0x%x,%s\n' % (op, reg, arg, value)


def seed_params(values=SEEDS):
    return [(0xc18 + 4 * i, v) for i, v in enumerate(values)]


def label_offset(src, label):
    """Byte offset of `label:` in a source with one instruction or label per line."""
    count = 0
    for line in src.splitlines():
        s = line.split(';')[0].strip()
        if not s:
            continue
        if s.endswith(':'):
            if s[:-1] == label:
                return 8 * count
            continue
        count += 1
    raise ValueError('label %s not found' % label)


def pad_to(src, offset, filler='    movi r5,#0\n'):
    size = label_offset(src + 'padmark:\n', 'padmark')
    if size > offset or (offset - size) % 8:
        raise ValueError('program prefix exceeds pad offset')
    return src + filler * ((offset - size) // 8)


def shl(v, c): return (v << c) & MASK if c < 32 else 0
def shr(v, c): return v >> c if c < 32 else 0


# ---- m2_m1_shift_counts (M1-F040)
SHIFT_COUNTS = (tuple(range(256)) + tuple(1 << k for k in range(8, 32)) +
                (0xffffffff, 0x101, 0x11f, 0x1001f, 0x8000001f, 0xffffffe1))
SHIFT_VALUE = 0x80000001


def shift_counts(s, t):
    for k in range(0, len(SHIFT_COUNTS), 8):
        counts = SHIFT_COUNTS[k:k + 8]
        body = '    movi r0,#0x5a5a5a5a\n    movi r1,#0x%x\n' % SHIFT_VALUE
        m1, layout = {}, {}
        for i, c in enumerate(counts):
            off = 0xd00 + 16 * i
            if c:  # immediate zero is the two-register form: not an immediate shift
                body += '    shl r3,r1,#0x%x\n    st [0x%x],r3\n' % (c, off)
                body += '    shr r4,r1,#0x%x\n    st [0x%x],r4\n' % (c, off + 4)
                m1[hex(off)], m1[hex(off + 4)] = shl(SHIFT_VALUE, c), shr(SHIFT_VALUE, c)
            body += '    movi r2,#0x%x\n    mov r3,r1\n    shl r3,r2\n    st [0x%x],r3\n' % (c, off + 8)
            body += '    mov r4,r1\n    shr r4,r2\n    st [0x%x],r4\n' % (off + 12)
            m1[hex(off + 8)], m1[hex(off + 12)] = shl(SHIFT_VALUE, c), shr(SHIFT_VALUE, c)
            layout[hex(off)] = 'count 0x%x: shl#, shr#, shl reg, shr reg' % c
        yield F.Case('counts%03d' % (k // 8), F.finish(t, body, 0),
                     dict(params=[(o, 0xeeeeeeee) for o in range(0xd00, 0xd80, 4)], timeout_us=20000),
                     cond(['M1-F040'], 'same as M1 if every count >= 32 gives 0 in both forms and 1..31 the logical '
                          'shift; differs if any count >= 32 is non-zero (count modulo 32/256)',
                          counts=[hex(c) for c in counts], value=hex(SHIFT_VALUE), layout=layout,
                          unwritten='0xeeeeeeee (immediate count 0 has no immediate form)', m1_prediction={'after': m1}))


# ---- m2_m1_indirect_wrap (M1-F041)
PLD_OFFSETS = (0x0d64, 0x1d64, 0x2d64, 0x8d64, 0xfd64, 0x10d64, 0x80000d64, 0x0c18, 0x1c18, 0x0c30)
PST_OFFSETS = (0x1d68, 0x10d70, 0xfd6c)   # increasing distance from the established 12-bit read alias
WRAP_PRAM = {0xd64: 0xcafe0064, 0xd68: 0xcafe0068, 0xd6c: 0xcafe006c, 0xd70: 0xcafe0070}


def indirect_wrap(s, t):
    params = seed_params() + list(WRAP_PRAM.items())
    for off in PLD_OFFSETS:
        low = off & 0xfff
        m1 = SEEDS[(low - 0xc18) // 4] if 0xc18 <= low < 0xc38 else WRAP_PRAM.get(low)
        body = '    movi r1,#0x%x\n' % off + insn(0x18, 2, 1) + '    st [0xd00],r2\n'
        yield F.Case('pld/%x' % off, t.prologue(0) + body + '    end\n' + t.trap(),
                     dict(params=params, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                     cond(['M1-F041'], 'same as M1 if the word at (offset & 0xfff) is read (0xc18/0xc30 = r0/r6); '
                          'differs if another word, 0 or a fault', pointer=hex(off), stored_at='0xd00',
                          m1_prediction={'after': {'0xd00': m1}}))
    for off in PST_OFFSETS:
        body = '    movi r1,#0x%x\n' % off + insn(0x19, 3, 1)
        yield F.Case('pst/%x' % off, t.prologue(0) + body + '    end\n' + t.trap(),
                     dict(params=params, flags=DISCOVERY | P.WAIT_END | P.FULLMAP, timeout_us=20000),
                     cond(['M1-F041'], 'same as M1 if only (offset & 0xfff) changes to r3; differs if another '
                          'word changes (FULLMAP after) or nothing does', pointer=hex(off), value=hex(SEEDS[3]),
                          m1_prediction={'after': {hex(off & 0xfff): SEEDS[3]}},
                          risk='targets beyond 0xfff; ordered by distance from the measured 0x1d64 read alias'))




# ---- m2_m1_c58_end_divert (M1-F045, F046, F162)
C58_VALUES = (1, 2, 3, 4, 5, 6, 7, 0x12, 0x16)


def end_divert_source(t, a, program_c4c):
    """END chain: the program END, then up to three handler ENDs, each recorded; the fourth entry stops."""
    src = '    wr [FMSS_C10],#1\n    wr [CS_IRQ],#0xff\n'
    if program_c4c:
        src += '    wr [FMSS_C4C],#handler\n'
    src += '    movi r5,#0\n    movi r3,#0\n    movi r7,#0x%x\n' % a
    for off in (0xc5c, 0xc58, 0xc4c):
        src += '    ld r1,[0x%x]\n    sti [r7],r1\n    add r7,r7,#4\n' % off
    src += '    movi r2,#0xfa11fa11\n    end\n    movi r2,#0xbe40d000\n    end\n'
    src += 'handler:\n    and r4,r5,#0x100\n    bnz r4,again\n    add r5,r5,#1\n    st [0xd70],r5\n'
    for off in (0xc5c, 0xc0c, 0xc58, 0xc4c, 0xc08):
        src += '    ld r1,[0x%x]\n    sti [r7],r1\n    add r7,r7,#4\n' % off
    src += '    movi r2,#0x7ec70000\n    add r2,r5\n    sub r4,r5,#4\n    bz r4,stop\n    end\n'
    src += 'stop:\n    wr [FMSS_C58],#0\n    movi r5,#0x100\n    movi r6,#0x57095709\n    end\n'
    src += 'again:\n    add r3,r3,#1\n    st [0xd74],r3\n    end\n'
    return src + t.trap()


def c58_end_divert(s, t):
    a = t.scratch_addr(s)
    for order in ('cpu-c58-then-c4c', 'program-c4c', 'cpu-c4c-then-c58'):
        src = end_divert_source(t, a, order == 'program-c4c')
        handler = label_offset(src, 'handler')
        for value in (0,) + C58_VALUES:
            params = {'cpu-c58-then-c4c': [(0xc58, value), (0xc4c, handler)],
                      'program-c4c': [(0xc58, value)],
                      'cpu-c4c-then-c58': [(0xc4c, handler), (0xc58, value)]}[order]
            yield F.Case('%s/c58_%02x' % (order, value), src,
                         dict(params=params + [(0xd70, 0), (0xd74, 0)], scratch=bytes([0xee]) * 96, scratch_read=96,
                              flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                         cond(['M1-F045', 'M1-F046', 'M1-F162'],
                              'same as M1 if C58 = 6 diverts only the first END, 3 every END after the first and 2 none; '
                              'differs if no value diverts or another value/ordering does',
                              c58=hex(value), handler=hex(handler), order=order,
                              layout='scratch+0: C5C/C58/C4C before the END; +12+20k: C5C/C0C/C58/C4C/C08 at handler '
                                     'entry k; 0xd70 = entries (4th stops: C58 = 0, r6 0x57095709); 0xd74 = entries '
                                     'after stop; r2 0xfa11fa11 = no diversion, 0x7ec7000k = handler k',
                              series='three ENDs in one run replace three CPU starts: the v14 probe resets after '
                                     'every case; a diverted END that also sets CS_IRQ bit 0 ends the capture early'))




# ---- m2_m1_poweron_values (M1-F048, F078, F080)
def poweron_values(s, t):
    untouched = P.FULLMAP | P.S_NO_RESET | P.S_NO_PREFLIGHT | P.S_NO_CLEAN | F_KEEP_CTRL0
    decision = ('same as M1 if C14 reads 0xff at power-on and ignores CPU and program writes, and the soft reset '
                'leaves status words and C14 different from power-on; differs if C14 powers on at another value '
                '(e.g. 0x7f) or a write is retained')
    yield F.Case('first-untouched', '    end\n', dict(flags=untouched, timeout_us=20000),
                 cond(['M1-F048', 'M1-F078', 'M1-F080'], decision,
                      measures='fullmap[0]: the window as left by whatever ran before this case, with no probe reset, '
                               'no clean params, no FMC_CTRL0 clear and no preflight (only CS_IRQ is acknowledged). '
                               'As the first case after boot this is the state after the kernel boot (which already '
                               'resets the FMSS), not a true cold power-on; later in a campaign it is the previous '
                               'case\'s cleanup-reset state. FIFO ports 0x060..0x068/0x080 are not read'))
    yield F.Case('second-untouched', '    end\n', dict(flags=untouched, timeout_us=20000),
                 cond(['M1-F078', 'M1-F080'], decision,
                      measures='same flags as first-untouched: the state the probe cleanup reset leaves'))
    yield F.Case('after-reset', '    end\n', dict(flags=P.FULLMAP | F_KEEP_CTRL0, timeout_us=20000),
                 cond(['M1-F078', 'M1-F080'], decision, measures='ordinary probe reset + preflight, then fullmap'))
    body = ('    ld r1,[0xc14]\n    st [0xd04],r1\n    wr [0xc14],#0x33\n    ld r2,[0xc14]\n    st [0xd00],r2\n')
    yield F.Case('c14-writes', body + '    end\n', dict(params=[(0xc14, 0x55)], flags=P.FULLMAP, timeout_us=20000),
                 cond(['M1-F048'], decision, measures='before[0xc14] after a CPU write of 0x55; 0xd04 program read '
                      'before, 0xd00 after its write of 0x33'))




# ---- m2_m1_c50_bit0 (M1-F050)
def c50_bit0(s, t):
    g = Q.geometry(s)
    handler = ('    ld r2,[0xc50]\n    wr [0xc50],#1\n    ld r3,[0xc50]\n    wr [0x48],#2\n    ld r4,[0xc50]\n'
               '    st [0xd00],r1\n    st [0xd04],r2\n    st [0xd08],r3\n    st [0xd0c],r4\n'
               '    ld r5,[0x48]\n    st [0xd10],r5\n' + Q.wr(0, 0) + '    movi r6,#0x4a4d4a4d\n    end\n')
    for command in ('cmd', 'no-cmd', 'cmd-acked'):
        for path in ('timeout', 'inline'):
            body = Q.setup(g.ce) + {'cmd': Q.wr(8, 0x70), 'no-cmd': '',
                                    'cmd-acked': Q.command(0x70)}[command]
            body += '    ld r1,[0xc50]\n'
            src = t.prologue(4) + '    wr [FMSS_C4C],#handler\n' + body
            src += (Q.wait(3) + '    movi r6,#0xbad0bad0\n    end\n' if path == 'timeout' else '')
            src += 'handler:\n' + handler + t.trap()
            yield F.Case('%s/%s' % (command, path), src,
                         dict(start_value=SHORT, flags=P.WAIT_END, timeout_us=30000),
                         cond(['M1-F050'], 'same as M1 if C50 bit 0 is set in r1, r2 and r3 (timeout and a write of 1 '
                              'keep it) and clear in r4 (after the FMC_STATUS bit 1 acknowledgement); differs otherwise',
                              layout='0xd00..0xd0c: C50 after command, at handler entry, after wr 1, after ack; '
                                     '0xd10 FMC_STATUS after ack', path=path), pins=True)


# ---- m2_m1_c14_timer (M1-F051, F151)
def c14_timer(s, t):
    g = Q.geometry(s)
    starts = [0xffb5] + [0xffb5 & ~(1 << b) for b in range(8, 16)] + [0xff75]
    body = ('    ld r1,[0xc14]\n' + Q.wait(3) + '    ld r2,[0xc14]\n    wr [0xc14],#0x12\n    ld r3,[0xc14]\n'
            '    st [0xd00],r1\n    st [0xd04],r2\n    st [0xd08],r3\n')
    decision = ('same as M1 if the wait loads C14 with start-word bits 15:8, counts it down, leaves the reload value '
                'after the timeout and ignores the write; differs if another value (e.g. bits 15:9) or no count')
    for start in starts:
        count = (start >> 8) & 0xff
        yield F.Case('timeout/start%04x' % start, F.finish(t, body, 0),
                     dict(start_value=start, flags=P.WAIT_END, timeout_us=200000),
                     cond(['M1-F051', 'M1-F151'], decision, start=hex(start), bits15_8=hex(count),
                          layout='0xd00 C14 before the wait, 0xd04 after the timeout, 0xd08 after wr 0x12',
                          m1_prediction={'after': {'0xd04': count, '0xd08': count}}))
    for late_us in (1000, 10000, 40000, 80000, 120000):
        tr = (Q.setup(g.ce) + '    ld r1,[0xc14]\n' + Q.wait(0) + '    ld r2,[0xc14]\n    ld r4,[0x48]\n' +
              Q.wr(0x48, 2) + '    ld r3,[0xc14]\n    st [0xd00],r1\n    st [0xd04],r2\n    st [0xd08],r3\n'
              '    st [0xd0c],r4\n')
        yield F.Case('release/%dus' % late_us, F.finish(t, tr, 0),
                     dict(start_value=0xffb5, flags=P.WAIT_END, timeout_us=200000, late=[(8, 0x70)], late_us=late_us),
                     cond(['M1-F051'], decision, release='CPU FMC_CMD = 0x70 at %d us releases wait 0' % late_us,
                          layout='0xd00 C14 before, 0xd04 at release, 0xd08 after the ack, 0xd0c FMC_STATUS',
                          trajectory='C14 at the release time; the probe cannot sample C14 during a wait'), pins=True)


# ---- m2_m1_start_bits (M1-F054, F061, F068, F153)
def counter_program(t):
    body = '    ld r1,[0xd70]\n    add r1,r1,#1\n    st [0xd70],r1\n' + F.delay(200000) + '    movi r2,#0x600d0002\n'
    return F.finish(t, body, 0)


def start_bits(s, t):
    decision = ('same as M1 if bits 1, 3 and 16 hold the program at instruction 0 (C00 bit 18), bit 1 while running '
                'freezes at the PC (C08 = 3) and resumes there, bits 3/16 while running restart from 0 after release, '
                'a plain start while running is ignored and the other bits change nothing; differs otherwise')
    common = dict(params=[(0xd70, 0)], flags=P.WAIT_END, timeout_us=60000)
    m1 = ['M1-F054', 'M1-F061', 'M1-F068', 'M1-F153']
    note = ('loop ~13.5 ms (200000 x sub/bnz); 0xd70 counts passes through the program start; END time and the '
            'after snapshot (C00/C08/C38/C40/C44) give hold/freeze/restart')
    yield F.Case('control', counter_program(t), dict(common), cond(m1, decision, note=note))
    for b in (1, 3, 6) + tuple(range(16, 32)):
        word = 0xffb5 | (1 << b)
        yield F.Case('start/bit%02d' % b, counter_program(t), dict(common, start_value=word),
                     cond(m1, decision, start=hex(word), note=note))
        if b in (1, 3, 16):
            yield F.Case('start/bit%02d/release' % b, counter_program(t),
                         dict(common, start_value=word, late=[(0xc00, 0xffb5)], late_us=2000),
                         cond(m1, decision, start=hex(word), release='C00 = 0xffb5 at 2000 us', note=note))
    for word in (0xffb7, 0xffbd, 0x1ffb5):
        for release in (False, True):
            options = dict(common, late=[(0xc00, word)], late_us=1000)
            if release:
                options.update(late2=[(0xc00, 0xffb5)], late2_us=5000)
            yield F.Case('running/%05x/%s' % (word, 'release' if release else 'held'), counter_program(t), options,
                         cond(m1, decision, running_write=hex(word) + ' at 1000 us',
                              release='C00 = 0xffb5 at 5000 us' if release else None, note=note))
    yield F.Case('running/plain-start', counter_program(t), dict(common, late=[(0xc00, 0xffb5)], late_us=1000),
                 cond(m1, decision, running_write='0xffb5 at 1000 us', note=note))


# ---- m2_m1_window_matrix (M1-F055, F056, F103)
ACTION = (0x004, 0x008, 0x040, 0x060, 0x064, 0x068, 0x080, 0x804, 0x81c,
          0xc00, 0xc04, 0xc38, 0xc60, 0xc64, 0xc68, 0xc6c)
GPRS = tuple(range(0xc18, 0xc38, 4))
FIFO = (0x060, 0x064, 0x068, 0x080)


def excluded_writes():
    """Action words, every alias candidate (offset bits 8, 9, 10 within the same 2 KiB half), the GPRs.
    PRAM 0xd00..0xd7c stays writable: its 32 words are established as independent storage."""
    out = set(GPRS)
    for word in ACTION:
        for m in range(8):
            out.add(word ^ ((m & 1) << 8) ^ ((m >> 1 & 1) << 9) ^ ((m >> 2 & 1) << 10))
    return frozenset(o for o in out if not 0xd00 <= o < 0xd80)


EXCLUDED = excluded_writes()
PATTERNS = (('zero', lambda o: 0), ('ones', lambda o: MASK), ('a5', lambda o: 0xa5a5a5a5),
            ('unique', lambda o: 0x5a000000 | o << 12 | o))
RESTORE = Q.wr(0xc58, 0) + '    wr [FMSS_C4C],#trap\n' + Q.wr(0x44, 0) + Q.wr(0, 0) + '    wr [FMSS_C10],#1\n'


def window_reads(a, words, c10_reg=None):
    src = '    movi r7,#0x%x\n' % a
    for off in words:
        for k, alias in enumerate((off, off ^ 0x100, off ^ 0x200)):
            if alias in FIFO:
                src += '    movi r1,#0xeeeeeeee\n'
            elif k == 0 and off == 0xc10 and c10_reg:
                src += '    mov r1,%s\n' % c10_reg
            else:
                src += '    ld r1,[0x%x]\n' % alias
            src += '    sti [r7],r1\n    add r7,r7,#4\n'
    return src


def window_matrix(s, t):
    a = t.scratch_addr(s)
    for base in range(0, 0x1000, 0x100):
        words = [o for o in range(base, base + 0x100, 4) if o not in EXCLUDED]
        for label, value in PATTERNS:
            common = dict(scratch=bytes([0xee]) * 0x300, scratch_read=0x300,
                          flags=DISCOVERY | P.FULLMAP, timeout_us=50000)
            info = dict(block=hex(base), pattern=label, written=[hex(o) for o in words],
                        excluded=sorted(hex(o) for o in range(base, base + 0x100, 4) if o in EXCLUDED),
                        layout='scratch+12i: sequencer ld of word i, word^0x100, word^0x200 (FIFO ports not read: '
                               '0xeeeeeeee)')
            decision = ('same as M1 if C38 is read-only to the CPU outside a pause, C3C/C48 hold 32 bits, C54 12, '
                        'offset bit 9 aliases and 0x100..0x1fc mirrors four times with 0x940..0x950 read/write; '
                        'differs at any word whose mask or alias departs from M1 chapter 3.2')
            m1 = ['M1-F055', 'M1-F056', 'M1-F103']
            # CPU writer: fullmap[0] is the CPU read-back, scratch the sequencer view.
            src = '    ld r2,[0xc10]\n    wr [FMSS_C10],#1\n' + window_reads(a, words, 'r2') + RESTORE
            yield F.Case('cpu/%03x/%s' % (base, label), src + '    movi r6,#0x600d600d\n    end\n' + t.trap(),
                         dict(common, params=[(o, value(o)) for o in words]),
                         cond(m1, decision, writer='cpu', **info))
            # Sequencer writer: read back at once (scratch), CPU view in fullmap[1].
            src = '    wr [FMSS_C10],#1\n    movi r7,#0x%x\n' % a
            for off in words:
                src += '    wr [0x%x],#0x%x\n' % (off, value(off))
                for k, alias in enumerate((off, off ^ 0x100, off ^ 0x200)):
                    src += ('    movi r1,#0xeeeeeeee\n' if alias in FIFO else '    ld r1,[0x%x]\n' % alias)
                    if k == 0 and off == 0xc10:
                        src += '    wr [FMSS_C10],#1\n'
                    src += '    sti [r7],r1\n    add r7,r7,#4\n'
            src += RESTORE
            yield F.Case('seq/%03x/%s' % (base, label), src + '    movi r6,#0x600d600d\n    end\n' + t.trap(),
                         dict(common), cond(m1, decision, writer='sequencer', **info))


# ---- m2_m1_bus_error (M1-F059)
BUS_HOLE = 0x50000000


def bus_error(s, t):
    a = t.scratch_addr(s)
    handler = ('handler:\n    ld r1,[0xc3c]\n    st [0xd00],r1\n    ld r1,[0xc50]\n    st [0xd04],r1\n'
               '    ld r1,[0xc0c]\n    st [0xd08],r1\n    ld r1,[0x48]\n    st [0xd0c],r1\n' + Q.wr(0, 0) +
               '    movi r6,#0xfa17fa17\n    end\n' + t.trap())
    pre = t.prologue(4) + '    wr [FMSS_C4C],#handler\n'
    tail = '    movi r3,#0x3a4c3a4c\n' + Q.wr(0, 0) + '    movi r6,#0x600d600d\n    end\n'
    decision = ('same as M1 if the DMA never completes (wait 2 faults at the timeout) and the ldi completes without a '
                'fault; differs if the DMA completes or faults differently, or the ldi faults or hangs the bus')
    options = dict(start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=50000,
                   scratch=struct.pack('<I', 0xcafe0001), scratch_read=4)
    layout = '0xd00 C3C, 0xd04 C50, 0xd08 CS_IRQ, 0xd0c FMC_STATUS in the handler; r2 load, r3 marker'
    yield F.Case('ldi/control', pre + '    movi r1,#0x%x\n    ldi r2,[r1]\n' % a + tail + handler, dict(options),
                 cond(['M1-F059'], decision, address=hex(a), layout=layout))
    yield F.Case('ldi/hole', pre + '    movi r1,#0x%x\n    ldi r2,[r1]\n' % BUS_HOLE + tail + handler, dict(options),
                 cond(['M1-F059'], decision, address=hex(BUS_HOLE), layout=layout,
                      risk='system-bus read of an unmapped address'))
    yield F.Case('dma/hole', pre + Q.setup() + Q.dma(BUS_HOLE) + tail + handler, dict(options),
                 cond(['M1-F059'], decision, address=hex(BUS_HOLE), layout=layout,
                      risk='controller DMA from an unmapped address (riskiest, last)'))


# ---- m2_m1_ctrl_words (M1-F064)
CTRL_SETTINGS = (('none', None, None), ('c10', 0xc10, 0), ('c10', 0xc10, 1), ('c10', 0xc10, 0xff),
                 ('c48', 0xc48, 0), ('c48', 0xc48, MASK), ('c3c', 0xc3c, 0), ('c3c', 0xc3c, MASK),
                 ('c54', 0xc54, 0), ('c54', 0xc54, 0xfff))


def ctrl_words(s, t):
    a = t.scratch_addr(s)
    decision = ('same as M1 if C3C holds the faulting offset + 8 inside the handler (also nested) and reads 0 after '
                'the run, and none of C10/C3C/C48/C54 changes any time or result; differs otherwise')
    for offset in (0x28, 0x40, 0x70):
        for nested in (False, True):
            src = pad_to(t.prologue(4) + '    wr [FMSS_C4C],#first\n', offset) + Q.wait(3)
            src += '    movi r6,#0x600d600d\n    end\nfirst:\n    ld r1,[0xc3c]\n    st [0xd00],r1\n'
            src += '    ld r1,[0xc38]\n    st [0xd04],r1\n    ld r1,[0xc0c]\n    st [0xd08],r1\n'
            if nested:
                src += '    wr [FMSS_C4C],#second\n' + Q.wr(0xc0c, 4) + 'inner:\n' + Q.wait(3)
            src += '    movi r6,#0x11111111\n    end\nsecond:\n    ld r1,[0xc3c]\n    st [0xd10],r1\n'
            src += '    ld r1,[0xc38]\n    st [0xd14],r1\n    movi r6,#0x22222222\n    end\n' + t.trap()
            info = dict(wait_offset=hex(offset), m1_prediction={'after': {'0xd00': offset + 8}})
            if nested:
                inner = label_offset(src, 'inner')
                info.update(inner_wait_offset=hex(inner),
                            m1_prediction={'after': {'0xd00': offset + 8, '0xd10': inner + 8}})
            yield F.Case('c3c/wait%02x/nested%d' % (offset, nested), src,
                         dict(start_value=SHORT, flags=P.WAIT_END, timeout_us=50000),
                         cond(['M1-F064'], decision, layout='0xd00/0xd04/0xd08: C3C/C38/CS_IRQ in the first '
                              'handler, 0xd10/0xd14: C3C/C38 in the second; after[0xc3c] = CPU read after END', **info))
    workloads = {
        'wait': (Q.wait(3), 'fault_at_us of the unmet wait 3 (C58 = 0 continues)'),
        'tmwait': (insn(0x15), 'elapsed_us of tmwait'),
        'encode': (Q.setup() + Q.dma(a) + Q.ecc() + ''.join(Q.store(0x840 + 4 * k, a + 0x2000 + 4 * k)
                                                               for k in range(4))
                   + Q.store(0x810, a + 0x2010) + Q.dma(a + 0x1000, into=False),
                   'BCH8 parity 0x840..0x84c at scratch+0x2000, 810 at +0x2010, slot-0 buffer DMA-out at +0x1000'),
    }
    for work, (body, measure) in workloads.items():
        for label, off, value in CTRL_SETTINGS:
            prologue = ('    wr [CS_IRQ],#0xff\n    wr [FMSS_C4C],#trap\n    wr [FMSS_C58],#0x0\n'
                        if off == 0xc10 else t.prologue(0))
            src = prologue + body + Q.wr(0, 0) + '    movi r6,#0x600d600d\n    end\n' + t.trap()
            options = dict(start_value=SHORT, flags=P.WAIT_END, timeout_us=50000,
                           params=[] if off is None else [(off, value)])
            if work == 'encode':
                options.update(scratch=F.memory(), scratch_read=0x2020)
            yield F.Case('%s/%s/%s' % (work, label, 'none' if off is None else '%x' % value), src, options,
                         cond(['M1-F064'], decision, word=None if off is None else hex(off),
                              value=None if value is None else hex(value), measure=measure,
                              note='the C10 settings use a prologue without its C10 write'))


# ---- m2_m1_evjmp_rules (M1-F088)
def evjmp_rules(s, t):
    a, g = t.scratch_addr(s), Q.geometry(s)
    vector = ('vector:\n    ld r1,[0x48]\n    ld r2,[0xc50]\n    ld r3,[0xc0c]\n    ld r4,[0xc3c]\n'
              '    st [0xd00],r1\n    st [0xd04],r2\n    st [0xd08],r3\n    st [0xd0c],r4\n%s'
              '    ld r5,[0x48]\n    st [0xd10],r5\n' + Q.wr(0, 0) + '    movi r6,#0x0a17a17\n    end\n')
    kinds = (('none', '', None, '', False), ('command', '', 1000, Q.wr(0x48, 2), True),
             ('command', '', 100, Q.wr(0x48, 2), True), ('command', '', 5000, Q.wr(0x48, 2), True),
             ('DMA', Q.dma(a).replace(Q.wait(2) + Q.wr(0x48, 0x100000), ''), None, Q.wr(0x48, 0x100000), False))
    for kind, action, late_us, ack, pins in kinds:
        for policy in (0, 4):
            src = t.prologue(policy) + '    wr [FMSS_C4C],#vector\n' + Q.setup(g.ce if pins else None) + action
            src += Q.wr(0xd70, 1) + insn(0x16) + Q.wr(0xd70, 2) + '    movi r6,#0x600d600d\n    end\n'
            src += vector % ack + t.trap()
            options = dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, timeout_us=20000)
            if late_us:
                options.update(late=[(8, 0x70)], late_us=late_us)
            if kind == 'DMA':
                options.update(scratch=F.memory(), scratch_read=16)
            name = '%s%s/c58%d' % (kind, '' if late_us is None else '-%dus' % late_us, policy)
            yield F.Case(name, src, options,
                         cond(['M1-F088'], 'same as M1 if evjmp jumps to C4C once the event arrives, for both C58 '
                              'values, without a fault, and the flag is still set in the handler; differs otherwise',
                              stimulus=kind, late_us=late_us, c58=policy,
                              layout='0xd00 FMC_STATUS, 0xd04 C50, 0xd08 CS_IRQ, 0xd0c C3C at the vector before the '
                                     'ack, 0xd10 FMC_STATUS after it; r6 0x0a17a17 = vector, 0xd70 = 1 while waiting'),
                         pins)


# ---- m2_m1_wait2_fields (M1-F091)
WAIT2_FORMS = ((0, 0), (1, 0xd64), (0xd60, 0xffffffff), (0xffff, 0))


def wait2_fields(s, t):
    a = t.scratch_addr(s)
    dma = Q.dma(a).replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')
    decision = 'same as M1 if arg/imm never change the outcome; differs if any form changes it'
    cases = [('line05/idle', 5, '', form) for form in WAIT2_FORMS]
    cases += [('line20/dma-en', 20, Q.wr(0x44, MASK) + dma, form) for form in WAIT2_FORMS]
    cases += [('line20/dma-noen', 20, dma, (0, 0)), ('line20/idle-en', 20, Q.wr(0x44, MASK), (0, 0))]
    for label, line, action, (arg, imm) in cases:
        body = Q.setup() + action + '    movi r2,#0x77770001\n' + insn(9, line, arg, imm)
        body += '    movi r2,#0x77770002\n' + Q.wr(0x44, 0) + Q.wr(0x48, 0x100000)
        yield F.Case('%s/arg%x/imm%x' % (label, arg, imm), F.finish(t, body, 0),
                     dict(start_value=SHORT, flags=DISCOVERY | P.WAIT_END, timeout_us=20000,
                          scratch=F.memory(), scratch_read=16),
                     cond(['M1-F091'], decision, line=line, arg=hex(arg), imm=hex(imm),
                          measure='fault_at_us (0 = passed); fault ~= 4.4 ms', state=label.split('/')[1]))


# ---- m2_m1_pc_write (M1-F094, F095)
def pc_write(s, t):
    decision = ('same as M1 if program writes jump, the CPU write takes effect only at a pause and the resume works '
                'with or without the acknowledgement; differs otherwise')
    m1 = ['M1-F094', 'M1-F095']
    land = '    movi r2,#0xfa11fa11\n    end\nland:\n    movi r2,#0x1a4d1a4d\n    end\n'
    for label, body in (('wr', '    wr [FMSS_PC],#land\n'), ('st', '    movi r1,#land\n    st [FMSS_PC],r1\n')):
        src = t.prologue(0) + body + land + t.trap()
        yield F.Case('program/' + label, src, dict(flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                     cond(m1, decision, markers='r2 0x1a4d1a4d = jumped to land, 0xfa11fa11 = fall-through'))
    src = t.prologue(0) + land + t.trap()
    yield F.Case('cpu/idle', src, dict(params=[(0xc38, label_offset(src, 'land'))], flags=P.WAIT_END, timeout_us=20000),
                 cond(m1, decision, observe='before[0xc38] = CPU read after the write; r2 = where the start began',
                      written=hex(label_offset(src, 'land'))))
    src = t.prologue(0) + F.delay(200000) + land + t.trap()
    yield F.Case('cpu/running', src, dict(late=[(0xc38, label_offset(src, 'land'))], late_us=1000, flags=P.WAIT_END,
                                          timeout_us=60000),
                 cond(m1, decision, observe='END time (loop ~13.5 ms) and r2: jump at 1 ms or not; PC samples'))
    src = t.prologue(0) + Q.wait(3) + land + t.trap()
    yield F.Case('cpu/waiting', src, dict(late=[(0xc38, label_offset(src, 'land'))], late_us=1000, start_value=SHORT,
                                          flags=P.WAIT_END, timeout_us=30000),
                 cond(m1, decision, observe='r2 and fault_at_us (~4.4 ms) with a CPU PC write at 1 ms in wait 3'))
    src = (t.prologue(0) + '    movi r6,#0\n' + insn(0x12) + '    movi r6,#0x600d600d\n    end\n'
           'far:\n    movi r6,#0x1a4d1a4d\n    end\n' + t.trap())
    code = P.assemble(t, src)
    pc = next(i for i in range(0, len(code), 8) if code[i + 3] == 0x12)
    far = label_offset(src, 'far')
    variants = {
        'noack-ffb5': [(0xc38, pc + 8), (0xc00, 0xffb5)],
        'noack-4ffb5': [(0xc38, pc + 8), (0xc00, 0x4ffb5)],
        'ack-start-ffb5': [(0xc38, pc + 8), (0xc0c, 8), (0xc00, 0xffb5)],
        'ack-start-4ffb5': [(0xc38, pc + 8), (0xc0c, 8), (0xc00, 0x4ffb5)],
        'start-twice-ffb5': [(0xc38, pc + 8), (0xc00, 0xffb5), (0xc00, 0xffb5)],
        'redirect-ack-4ffb5': [(0xc38, far), (0xc0c, 8), (0xc00, 0x4ffb5)],
        'start-only-4ffb5': [(0xc0c, 8), (0xc00, 0x4ffb5)],
        'pc-only': [(0xc38, pc + 8)],
    }
    for label, writes in variants.items():
        c = cond(m1, decision, pause_pc=hex(pc), cpu_writes=[(hex(o), hex(v)) for o, v in writes],
                 markers='r6 0x600d600d = resumed after the pause, 0x1a4d1a4d = far, 0 = still paused')
        if label == 'ack-start-4ffb5':
            c.update(mode='confirm', expected={'why': 'END', 'r': {'6': 0x600d600d}},
                     basis='p15 m2_recovery cpu-resume (PC + 8, ack, 0x4ffb5); here 1 ms after the start')
        yield F.Case('pause/' + label, src, dict(late=writes, late_us=1000, flags=DISCOVERY | P.WAIT_END,
                                                 timeout_us=5000), c)


END_FIELDS = (('zero', 0, 0, 0), ('R1', 1, 0, 0), ('R7', 7, 0, 0), ('Rff', 0xff, 0, 0), ('arg1', 0, 1, 0),
              ('argd60', 0, 0xd60, 0), ('argffff', 0, 0xffff, 0), ('imm1', 0, 0, 1), ('immffffffff', 0, 0, MASK))


ZERO_VALUE = 0x0a5a0000   # no low-12-bit FMCTRL1 action bits, should an immediate replace an offset


SPECS = {
    'm2_m1_shift_counts': (shift_counts, 'alu', 'SHL/SHR with counts 0..255, 2^8..2^31 and wrap probes, immediate and register form'),
    'm2_m1_indirect_wrap': (indirect_wrap, 'indirect', 'pld/pst offsets above 0xfff: 12-bit wrap and r0..r7 reach'),
    'm2_m1_c58_end_divert': (c58_end_divert, 'oneshot,c58b1,c58b2', 'C58 values 1..7, 0x12, 0x16 diverting END to C4C, CPU/program C4C orders'),
    'm2_m1_poweron_values': (poweron_values, 'restore', 'Untouched first capture versus reset window; C14 CPU and program writes'),
    'm2_m1_c50_bit0': (c50_bit0, 'c50,c50x', 'C50 bit 0 with a pending command across a timeout, a write of 1 and the ack'),
    'm2_m1_c14_timer': (c14_timer, 'tmobits,startbits', 'C14 before/after an unmet wait per start-word bits 15:8; values at timed releases'),
    'm2_m1_start_bits': (start_bits, 'startbits', 'Start-word bits 1, 3, 6, 16..31 at start and while running: hold, pause, restart'),
    'm2_m1_window_matrix': (window_matrix, 'winmap,masks', 'Whole-window CPU/sequencer write/read-back matrix with bit 8/9 alias reads'),
    'm2_m1_bus_error': (bus_error, 'buserr', 'ldi and DMA to an unmapped bus address (run last)'),
    'm2_m1_ctrl_words': (ctrl_words, 'ctrlwords', 'C3C in (nested) handlers; C10/C3C/C48/C54 vs wait, tmwait and DMA+BCH8 encode'),
    'm2_m1_evjmp_rules': (evjmp_rules, 'op16,op16b', 'evjmp with a command/DMA event arriving during the wait, C58 0/4, flag in the handler'),
    'm2_m1_wait2_fields': (wait2_fields, 'op09,w2vic', 'wait2 5 and wait2 20 (after a DMA) with non-zero arg/imm'),
    'm2_m1_pc_write': (pc_write, 'r8,op12', 'PC writes by the program and the CPU (idle, running, waiting, pause resume orders)'),
}

# Window-wide writes (unknown words, 0xffffffff), pst beyond 0xfff and unmapped bus accesses could wedge
# the engine or the bus beyond the probe reset; the unmapped-address family is the very last.
RUN_LAST = ('m2_m1_indirect_wrap', 'm2_m1_window_matrix', 'm2_m1_bus_error')
