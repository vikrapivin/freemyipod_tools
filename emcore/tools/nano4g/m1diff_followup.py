"""M1-versus-M2 follow-ups (m1diff_followup) for the findings p36 left unresolved.

One family per open question; names m2_m1f_<snake>. Families and the p36 cause each answers:

- m2_m1f_dma_then_read (M1-F036): p36 m2_m1_dma_then_read put the same tail (`wait 2`, `wait 3`)
  after every variant. In 'serial-control' the DMA had already passed its own `wait 2` and been
  acknowledged, so the tail `wait 2` waited for a DMA that no longer existed and timed out (stall at
  the tail wait, pc 360 = 11 instructions after the action start; output 0x2000 was the exact page
  sector, the read completed: FMC_STATUS bit 3 set and `wait 3` passed). Here each variant carries its
  own waits, logged by stage name, and a serial raw-read reference gives the page bytes.
- m2_m1f_evjmp (M1-F088): p36 none/c584 "jumped" because C58 = 4 vectors the evjmp timeout fault to
  C4C, which the program had pointed at the evjmp handler (handler CS_IRQ = 4, reached at the 126.59 ms
  default fault window, FMC_STATUS without bit 1, C50 bits 0..5 clear: no pending flag from set-up).
  Here the handler stores CS_IRQ first (bit 2 = timeout fault vector, clear = evjmp departure), the
  window is short and pending/acknowledged/second-set controls are added.
- m2_m1f_indirect_reach (M1-F041): pld/pst reach of r1..r5/r7, pst into the register window, offset
  bits 17..30.
- m2_m1f_ecc_csirq (M1-F065/F076): CS_IRQ bits 5/6 (and VIC raw) around BCH8 encode AND decode
  completions with ECC enable candidates: every unexplained ECC word all ones, single bits of
  0x808/0x814/0x818/0x844, ECC_CTRL kick bits 3..7/16..31, ECC_STATUS 0x810 bits 9..31.
- m2_m1f_bank_channel (M1-F079/F170): +0x400 READ ID with a zero-filled log area and an explicit stage
  count (p36's "storage words not all 0" came from the 0xee filler of an unused third log slot of
  parity0/primary-only, not from a +0x400 word), sequencer view of 0x400..0x47c, C50 right after the
  +0x408 command, and the same READ ID through the primary set as positive control.
- m2_m1f_ctrl0_held_dma (M1-F081): a DMA programmed with FMC_CTRL0 bit 24 clear: output snapshots by
  the sequencer before bit 24 is set again, after it, and with the DMA address rewritten in between.
- m2_m1f_fifo_write (M1-F164): FMCTRL1 0x340 with its launched output transfer directed to a sink and
  acknowledged (WAIT2 + ack) before 128/32 writes to 0x080 and the read-back DMA.
- m2_m1f_two_plane (M1-F122): two-plane program 80h/11h/81h/10h and erase 60h/60h/D0h. Scratch
  geometry is block 3000 on the geometry CE; THIS FAMILY ALONE also uses block 3001 (same CE) as the
  second plane block (plane = block bit 0); stated in every case's conditions.

Conventions as m1diff_fmc: 0x044 = 0 at END and on the fault vector, C60..C6C never written, every
changed word restored, C58 = 0 cases continue after a timed-out wait (start word SHORT ≈ 4.4 ms).
"""
import struct

from . import followup_experiments as F, sequences as Q  # noqa: F401
from . import m1diff_fmc as MF
from fmss import probe as P  # noqa: F401
from .protocol import DISCOVERY, MUTATE

Case, delay = F.Case, F.delay
SHORT = MF.SHORT
WAIT0 = MF.WAIT0
fin, tagged, scratch, rand, sha, vic, Log, dma_nowait, open_page = (
    MF.fin, MF.tagged, MF.scratch, MF.rand, MF.sha, MF.vic, MF.Log, MF.dma_nowait, MF.open_page)
ACK2 = Q.wr(0x48, 0x100000)


def cond(m1, decision, **kw):
    return dict(m1=list(m1), decision=decision, **kw)


def copy_words(src, dst, n, step=4):
    """Sequencer copy of n SDRAM words src+step*k -> dst+4k (uses r1, r2, r7)."""
    return ''.join('    movi r1,#0x%x\n    ldi r2,[r1]\n    movi r7,#0x%x\n    sti [r7],r2\n'
                   % (src + step * k, dst + 4 * k) for k in range(n))


class Stages:
    """Named register logs: stage k at base + stride*k; names kept for the conditions."""

    def __init__(self, base, regs, stride=0x10, clear_irq=True):
        self.log, self.names, self.regs = Log(base, regs, stride, clear_irq), [], [hex(r) for r in regs]

    def __call__(self, name):
        self.names.append(name)
        return self.log()


# ------------------------------------------------------------ m2_m1f_dma_then_read (M1-F036)

def dma_then_read(s, t):
    """FMCTRL1 0x100 immediately replaced by 0xe2, with a working serial control (M1-F036)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F036',)
    decision = ('same as M1 if, with the serial control and the reference giving the page bytes, the immediate '
                'replacement loses the DMA (its wait 2 times out, 0x2000 not the sector) while the read completes; '
                'differs if the DMA completes or the read is lost instead')
    dma_out = Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
    read1 = Q.wr(0x30, 511) + Q.wr(0x14, 2) + Q.wr(4, 0xe2)
    final = lambda st: Q.dma(a + 0x2200, 1, into=False).replace(Q.wait(2), Q.wait(2) + st('final-wait2'))  # noqa

    def overlap(prefix):
        return lambda st: (prefix + Q.wait(2) + st('dma-wait2') + Q.wait(3) + st('read-wait3')
                           + Q.wr(0x48, 0x100008) + final(st))

    variants = {
        'reference': lambda st: (Q.dma(a + 0x2000, 0, into=False).replace(Q.wait(2), Q.wait(2) + st('dma-wait2'))
                                 + Q.transfer(512, 1) + st('read-wait3') + final(st)),
        'serial-control': lambda st: (dma_out + Q.wait(2) + st('dma-wait2') + ACK2 + read1 + Q.wait(3)
                                      + st('read-wait3') + Q.wr(0x48, 8) + final(st)),
        'immediate': overlap(dma_out + Q.wr(4, 0xe2)),
        'immediate-sel-slot1': overlap(dma_out + Q.wr(0x14, 2) + Q.wr(4, 0xe2)),
        'read-then-dma': overlap(read1 + Q.wr(0x34, a + 0x2000) + Q.wr(0x14, 0x100) + Q.wr(4, 0x100)),
    }
    for n in (1, 4, 16, 64):
        variants['gap%d' % n] = (lambda n: lambda st: overlap(dma_out + delay(n) + Q.wr(0x14, 2)
                                                              + Q.wr(4, 0xe2))(st))(n)
    for name, build in variants.items():
        for rep in range(2):
            st = Stages(a + 0x3000, (0x048, 0x038, 0xc0c))
            body = open_page(g, g.row) + Q.transfer(512, 0) + build(st)
            yield Case(f'{name}/rep{rep}', fin(t, body, 0),
                       dict(scratch=scratch(0x3100), scratch_read=0x3060, flags=WAIT0, start_value=SHORT,
                            timeout_us=100000),
                       cond(m1, decision, variant=name, rep=rep, row=g.row, stages=st.names, stage_regs=st.regs,
                            gap_loops=int(name[3:]) if name.startswith('gap') else None,
                            layout='0x2000 buffer-0 DMA output (page sector 0 expected; 0xee = lost); 0x2200 '
                                   'buffer-1 output (page bytes 512..1023); 0x3000+0x10k: 048 038 c0c per stage',
                            fix='p36 serial-control waited a second wait 2 for an already acknowledged DMA; '
                                'each variant now has only its own waits'),
                       pins=True)


# ------------------------------------------------------------------- m2_m1f_evjmp (M1-F088)

def evjmp(s, t):
    """evjmp: event during the wait, pending before, acknowledged before, second set, C58 0/4 (M1-F088)."""
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F088',)
    decision = ('same as M1 if evjmp departs to C4C when an event arrives (handler CS_IRQ bit 2 clear, before the '
                'fault window) for C58 0 and 4, the flag is still set in the handler, and the second-set command '
                'also departs; the none/acked controls must not depart (C58 = 0: fall through, d70 = 2; C58 = 4: '
                'handler with CS_IRQ bit 2 = timeout fault vector)')
    restore = Q.wr(0x400, 0) + Q.wr(0x404, 0)
    vector = ('vector:\n    ld r3,[0xc0c]\n    ld r1,[0x48]\n    ld r2,[0xc50]\n    ld r4,[0xc3c]\n'
              '    st [0xd08],r3\n    st [0xd00],r1\n    st [0xd04],r2\n    st [0xd0c],r4\n%s'
              '    ld r5,[0x48]\n    st [0xd10],r5\n' + restore + Q.wr(0, 0) + '    movi r6,#0x0a17a17\n    end\n')
    second = Q.wr(0x404, 0x33338e0) + Q.wr(0x448, 0xff00ffe) + Q.wr(0x400, 0x1077801 | 2 << g.ce)
    kinds = (
        ('none', '', None, '', False, {}),
        ('pending-before', Q.wr(8, 0x70) + delay(200), None, Q.wr(0x48, 2), True, {}),
        ('acked-before', Q.command(0x70), None, Q.wr(0x48, 2), True, {}),
        ('command-100us', '', 100, Q.wr(0x48, 2), True, dict(late=[(8, 0x70)])),
        ('command-1000us', '', 1000, Q.wr(0x48, 2), True, dict(late=[(8, 0x70)])),
        ('DMA', dma_nowait(a), None, ACK2, False, {}),
        ('second-set-1000us', second, 1000, Q.wr(0x448, 0xff00ffe), False, dict(late=[(0x408, 0x70)])),
    )
    for kind, action, late_us, ack, pins, late in kinds:
        for policy in (0, 4):
            src = t.prologue(policy) + '    wr [FMSS_C4C],#vector\n' + Q.setup(g.ce if pins else None) + action
            src += Q.wr(0xd70, 1) + '    .insn 0x16,0,0x0,0x0\n' + Q.wr(0xd70, 2) + restore + Q.wr(0, 0)
            src += '    movi r6,#0x600d600d\n    end\n' + vector % ack + '\ntrap:\n' + restore + Q.wr(0, 0)
            src += '    movi r6,#0xfa17fa17\n    end\n'
            options = dict(params=[(0xd70, 0)], flags=DISCOVERY | P.WAIT_END, start_value=SHORT, timeout_us=30000,
                           **late)
            if late_us:
                options['late_us'] = late_us
            if kind == 'DMA':
                options.update(scratch=F.memory(), scratch_read=16)
            yield Case(f'{kind}/c58{policy}', src, options,
                       cond(m1, decision, stimulus=kind, late_us=late_us, c58=policy, window='SHORT ≈ 4.4 ms',
                            control=kind in ('none', 'acked-before'),
                            layout='0xd08 CS_IRQ (first read in the handler), 0xd00 FMC_STATUS, 0xd04 C50, 0xd0c '
                                   'C3C, 0xd10 FMC_STATUS after the ack; r6 0x0a17a17 = handler, 0x600d600d = fell '
                                   'through; 0xd70 = 1 while in evjmp, 2 after',
                            fix='p36 none/c584: the C58 = 4 timeout fault vectored to C4C = handler (CS_IRQ 4 at '
                                '126.59 ms); CS_IRQ bit 2 in the handler now separates fault vector and departure'),
                       pins)


# ------------------------------------------------------- m2_m1f_indirect_reach (M1-F041)

SEEDS = [0x11111111 * (n + 1) for n in range(8)]
WRAP_PRAM = {0xd64: 0xcafe0064, 0xd68: 0xcafe0068, 0xd6c: 0xcafe006c, 0xd70: 0xcafe0070}
HIGH_BITS = tuple(range(17, 31))
PLD_OFFSETS = (0xc1c, 0xc20, 0xc24, 0xc28, 0xc2c, 0xc34, 0x1c1c, 0x10c28) + tuple(0xd64 | 1 << b for b in HIGH_BITS)
PST_OFFSETS = tuple(range(0xc18, 0xc38, 4)) + (0x1c28,) + tuple(0xd68 | 1 << b for b in HIGH_BITS)


def pld_prediction(off):
    low = off & 0xfff
    if 0xc18 <= low < 0xc38:
        n = (low - 0xc18) // 4
        return off if n == 1 else SEEDS[n]    # r1 holds the pointer itself
    return WRAP_PRAM.get(low)


def indirect_reach(s, t):
    """pld/pst through r1..r5/r7 window offsets, pst into the register window, offset bits 17..30 (M1-F041)."""
    params = [(0xc18 + 4 * i, v) for i, v in enumerate(SEEDS)] + list(WRAP_PRAM.items())
    dump = ''.join('    st [0x%x],r%d\n' % (0xd00 + 4 * i, i) for i in range(8))
    for off in PLD_OFFSETS:
        body = '    movi r1,#0x%x\n    .insn 0x18,2,0x1,0x0\n    st [0xd00],r2\n' % off
        yield Case('pld/%x' % off, t.prologue(0) + body + '    end\n' + t.trap(),
                   dict(params=params, flags=DISCOVERY | P.WAIT_END, timeout_us=20000),
                   cond(['M1-F041'], 'same as M1 if pld reads the word at (offset & 0xfff), r1 giving the '
                        'pointer itself; differs if another word, 0 or a fault', pointer=hex(off), stored_at='0xd00',
                        m1_prediction={'after': {'0xd00': pld_prediction(off)}}))
    for off in PST_OFFSETS:
        low = off & 0xfff
        body = '    movi r1,#0x%x\n    .insn 0x19,3,0x1,0x0\n' % off + dump
        regs = {i: SEEDS[i] for i in range(8)}
        regs[1] = off
        if 0xc18 <= low < 0xc38:
            regs[(low - 0xc18) // 4] = SEEDS[3]
        pram = {} if 0xc18 <= low < 0xc38 else {hex(low): SEEDS[3]}
        yield Case('pst/%x' % off, t.prologue(0) + body + '    end\n' + t.trap(),
                   dict(params=params, flags=DISCOVERY | P.WAIT_END | P.FULLMAP, timeout_us=20000),
                   cond(['M1-F041'], 'same as M1 if only (offset & 0xfff) changes to r3 (a register of r0..r7 '
                        'inside the window); differs if another word changes or nothing does', pointer=hex(off),
                        value=hex(SEEDS[3]), layout='0xd00+4i: r0..r7 after the pst (r1 = pointer)',
                        m1_prediction={'after': dict({hex(0xd00 + 4 * i): v for i, v in regs.items()}, **pram)},
                        risk='store targets beyond 0xfff and into the register window'))


# ------------------------------------------------------------ m2_m1f_ecc_csirq (M1-F065/F076)

CSIRQ_SNAP = (0xc0c, 0xc08, 0x810, 0x048)
ECC_WORDS = (0x800, 0x808, 0x814, 0x818, 0x844) + tuple(range(0x860, 0xa00, 4))
BIT_WORDS = (0x808, 0x814, 0x818, 0x844)
KICK_BITS = (3, 4, 5, 6, 7) + tuple(range(16, 32))
STATUS_BITS = tuple(range(9, 32))


def csirq_snap(a, k):
    base = a + 0x2000 + 0x20 * k
    return ''.join(Q.store(off, base + 4 * i) for i, off in enumerate(CSIRQ_SNAP)) + vic(base + 16)


def ecc_csirq(s, t):
    """CS_IRQ bits 5/6 and VIC raw around BCH8 encode and decode completions per ECC enable candidate."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    m1 = ('M1-F065', 'M1-F076')
    decision = ('same as M1 if a candidate sets CS_IRQ bit 5 or 6 after an encode or decode completion that the '
                'baseline does not; differs if no candidate does while every completion is seen (810 bit 1/2)')
    layout = ('0x2000+0x20k: c0c c08 810 048 VIC0raw VIC1raw; k=0 before the kick (CS_IRQ cleared with 0xff), '
              '1 after encode wait4, 2 after encode ack, 3 after decode wait4, 4 after decode ack, 5 after restore')
    variants = [('base/en0', None, None, 0, 0, 0x1ff), ('base/enffffffff', None, None, 0xffffffff, 0, 0x1ff)]
    variants += [('word/%03x/en0' % o, o, 0xffffffff, 0, 0, 0x1ff) for o in (0x808, 0x814, 0x818, 0x844, 0x800)]
    variants += [('word/%03x' % o, o, 0xffffffff, 0xffffffff, 0, 0x1ff) for o in ECC_WORDS]
    variants += [('bit/%03x/b%02d' % (o, b), o, 1 << b, 0xffffffff, 0, 0x1ff) for o in BIT_WORDS for b in range(32)]
    variants += [('kick/b%02d' % b, None, None, 0xffffffff, 1 << b, 0x1ff) for b in KICK_BITS]
    variants += [('st810/b%02d' % b, None, None, 0xffffffff, 0, 0x1ff | 1 << b) for b in STATUS_BITS]
    opts = dict(scratch=F.memory(), scratch_read=0x20c0, flags=WAIT0, start_value=SHORT, timeout_us=50000)
    for name, off, value, en044, extra, status in variants:
        save = '' if off is None else '    ld r5,[0x%x]\n' % off
        cand = '' if off is None else Q.wr(off, value)
        restore = ('' if off is None else '    st [0x%x],r5\n' % off) + Q.wr(0x810, 0x1ff) + Q.wr(0x44, 0)
        body = (Q.setup() + Q.dma(a, 0) + save + cand + Q.wr(0x44, en044) + Q.wr(0xc0c, 0xff) + csirq_snap(a, 0)
                + Q.wr(0x810, status) + Q.wr(0x1c, 0) + Q.wr(0x804, 2 | extra) + Q.wait(4) + delay(300)
                + csirq_snap(a, 1) + Q.wr(0x810, 0x1ff) + csirq_snap(a, 2)
                + Q.wr(0x810, status) + Q.wr(0x1c, 0) + Q.wr(0x804, 1 | extra) + Q.wait(4) + delay(300)
                + csirq_snap(a, 3) + Q.wr(0x810, 0x1ff) + csirq_snap(a, 4) + restore + csirq_snap(a, 5))
        yield Case(name, fin(t, body, 0, restore), dict(opts),
                   cond(m1, decision, candidate=None if off is None else hex(off),
                        value=None if value is None else hex(value), enable044=hex(en044), kick_extra=hex(extra),
                        status_write=hex(status), c58=0, layout=layout,
                        note='0x044 written after the preparing DMA (its FMC flag acknowledged) and CS_IRQ cleared '
                             'with 0xff before the kick; CS_IRQ bit 4 may still follow 0x044-routed FMC flags'))


# --------------------------------------------------------- m2_m1f_bank_channel (M1-F079/F170)

BANK_SNAP = (0x424, 0x448, 0xc50, 0x048, 0x44c, 0xc0c, 0x024)


def window_dump(base, dst):
    """Sequencer read of base..base+0x7c to dst (FIFO ports 0x060..0x068 of the primary set skipped)."""
    return ''.join(Q.store(base + 4 * k, dst + 4 * k) for k in range(32) if base or not 0x60 <= 4 * k <= 0x68)


def bank_channel(s, t):
    """+0x400 READ ID (clean log layout), C50 after the +0x408 command, primary READ ID control."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F079', 'M1-F170')
    decision = ('same as M1 if the +0x400 READ ID counts 8 bytes in +0x424, waits 6/7/9 pass and C50 bit 6 rises '
                'after the +0x408 command (bit 8 never); differs if every +0x400 word stays 0 and no wait passes '
                'while the primary control passes waits 0/1/3')
    mem = lambda: scratch(0x3300, fill=0xee, x3000=bytes(0x300))  # noqa: E731  zero log area
    opts = dict(flags=WAIT0 | P.FULLMAP, start_value=SHORT, timeout_us=100000, scratch_read=0x3280)
    layout = ('0x3000+0x20k: 424 448 c50 048 44c c0c 024 per named stage (area zero-filled); 0x3200: sequencer '
              'read of the 0x400..0x47c (or 0x000..0x07c) window at the end')
    for code in (0xe2, 0x1c2):
        for rep in range(2):
            st = Stages(a + 0x3000, BANK_SNAP, 0x20)
            body = (Q.setup() + Q.wr(0x404, 0x33338e0) + Q.wr(0x448, 0xff00ffe) + Q.wr(0x400, 0x1077801 | 2 << g.ce)
                    + Q.wr(0x408, 0x90) + st('after-cmd') + Q.wait(6) + st('after-wait6') + Q.wr(0x448, 2)
                    + Q.wr(0x42c, 0) + Q.wr(0x40c, 0) + Q.wr(0x410, 0) + Q.wr(0x404, 1) + Q.wait(7)
                    + st('after-wait7') + Q.wr(0x448, 4) + Q.wr(0x430, 7) + Q.wr(0x414, 0x10) + Q.wr(0x404, code)
                    + Q.wait(9) + st('after-wait9') + Q.wr(0x448, 8) + window_dump(0x400, a + 0x3200)
                    + Q.wr(0x400, 0) + Q.wr(0x404, 0))
            yield Case(f'readid400/ctrl1-{code:x}/rep{rep}', fin(t, body, 0, Q.wr(0x400, 0)),
                       dict(opts, scratch=mem()),
                       cond(m1, decision, set='+0x400', transfer_code=hex(code), stages=st.names,
                            stage_regs=[hex(r) for r in BANK_SNAP], layout=layout,
                            fix='p36 counted the 0xee filler of an unused log slot as a +0x400 storage word'))
    for code in (0xe2, 0x1c2):
        st = Stages(a + 0x3000, BANK_SNAP, 0x20)
        body = (Q.setup(g.ce) + Q.wr(8, 0x90) + st('after-cmd') + Q.wait(0) + st('after-wait0') + Q.wr(0x48, 2)
                + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1) + st('after-wait1')
                + Q.wr(0x48, 4) + Q.wr(0x30, 7) + Q.wr(0x14, 0x10) + Q.wr(4, code) + Q.wait(3) + st('after-wait3')
                + Q.wr(0x48, 8) + window_dump(0x000, a + 0x3200))
        yield Case(f'readid000/ctrl1-{code:x}', fin(t, body, 0), dict(opts, scratch=mem()),
                   cond(m1, decision, set='primary (control)', transfer_code=hex(code), stages=st.names,
                        stage_regs=[hex(r) for r in BANK_SNAP], layout=layout), pins=True)
    for cmd in (0x70, 0x90):
        st = Stages(a + 0x3000, BANK_SNAP, 0x20)
        body = (Q.setup() + Q.wr(0x404, 0x33338e0) + Q.wr(0x448, 0xff00ffe) + Q.wr(0x400, 0x1077801 | 2 << g.ce)
                + Q.wr(0x408, cmd))
        for k in range(8):
            body += st('c50-%d' % k) + delay(20)
        body += Q.wait(6) + st('after-wait6') + window_dump(0x400, a + 0x3200) + Q.wr(0x400, 0) + Q.wr(0x404, 0)
        yield Case(f'cmd400/{cmd:02x}', fin(t, body, 0, Q.wr(0x400, 0)), dict(opts, scratch=mem()),
                   cond(m1, decision, set='+0x400', command=hex(cmd), stages=st.names,
                        stage_regs=[hex(r) for r in BANK_SNAP], layout=layout,
                        sampling='C50 eight times ≈ 1.4 us apart right after the +0x408 write, then wait 6'))


# ------------------------------------------------------ m2_m1f_ctrl0_held_dma (M1-F081)

CTRL0 = 0x1077801
HELD_REGS = (0x048, 0x034, 0x038, 0x03c, 0xc0c)


def ctrl0_held_dma(s, t):
    """A DMA programmed with FMC_CTRL0 bit 24 clear: does it start when bit 24 is set again? (M1-F081)"""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    m1 = ('M1-F081',)
    decision = ('same as M1 if nothing moves while bit 24 is clear (snapshots before setting it unchanged, FMC_STATUS '
                'bit 20 clear); the remaining M2 question: a held DMA that starts on bit 24 (deferred, with the '
                'registers at that moment = retarget address) versus output from elsewhere')
    clear = CTRL0 & ~0x1000000
    A, B, C = tagged(0x24), tagged(0x35), tagged(0x46)
    layout = ('0x3000+0x10k: 048 034 038 03c c0c per stage; 0x3100+0x80j: sequencer copy of 0x2000[0..0x3c]+0x1fc '
              '(17 words) at snapshot j; 0x3300+0x80j: same of the retarget area 0x2800')

    def snap(j):
        return (copy_words(a + 0x2000, a + 0x3100 + 0x80 * j, 16) + copy_words(a + 0x21fc, a + 0x3140 + 0x80 * j, 1)
                + copy_words(a + 0x2800, a + 0x3300 + 0x80 * j, 16) + copy_words(a + 0x29fc, a + 0x3340 + 0x80 * j, 1))

    for direction in ('out', 'in'):
        modes = ('reset-bit', 'retarget', 'never') if direction == 'out' else ('reset-bit', 'retarget')
        for mode in modes:
            for rep in range(2):
                st = Stages(a + 0x3000, HELD_REGS)
                if direction == 'out':
                    body = Q.setup() + Q.dma(a, 0) + Q.wr(0, clear) + dma_nowait(a + 0x2000, 0, into=False)
                    retarget = a + 0x2800
                else:
                    body = Q.setup() + Q.dma(a + 0x400, 0) + Q.wr(0, clear) + dma_nowait(a, 0)
                    retarget = a + 0x200
                body += Q.wait(2) + st('held-wait2') + snap(0)
                if mode == 'retarget':
                    body += Q.wr(0x34, retarget) + st('retargeted')
                if mode == 'never':
                    body += delay(2000) + st('held-later') + snap(1) + ACK2 + st('acked')
                else:
                    body += Q.wr(0, CTRL0) + delay(200) + st('bit24-set') + snap(1)
                    body += Q.wait(2) + st('wait2-after-set') + ACK2
                    if direction == 'in':
                        body += Q.dma(a + 0x2000, 0, into=False).replace(Q.wait(2), Q.wait(2) + st('readback'))
                        body += snap(2)
                body += Q.wr(0, CTRL0)
                mem = scratch(0x3400, x0=A, x200=B, x400=C)
                yield Case(f'{direction}/{mode}/rep{rep}', fin(t, body, 0),
                           dict(scratch=mem, scratch_read=0x3400, flags=WAIT0, start_value=SHORT, timeout_us=50000),
                           cond(m1, decision, direction=direction, mode=mode, rep=rep, stages=st.names,
                                stage_regs=[hex(r) for r in HELD_REGS], layout=layout,
                                sources={'0x0': 'tag 24 (A)', '0x200': 'tag 35 (B)', '0x400': 'tag 46 (C)'},
                                plan=('out: buffer 0 filled from A with bit 24 set, then DMA out to 0x2000 with bit 24 '
                                      'clear; retarget rewrites FMC_DMA_ADDR to 0x2800 before bit 24' if direction == 'out'
                                      else 'in: buffer 0 filled from C, then DMA in from A with bit 24 clear; retarget '
                                           'rewrites the address to B; read-back DMA to 0x2000 shows A, B or C'),
                                reference={'A': sha(A), 'B': sha(B), 'C': sha(C)}))


# ----------------------------------------------------------- m2_m1f_fifo_write (M1-F164)

def fifo_write(s, t):
    """FMCTRL1 0x340 with its launched output sunk and acknowledged, then writes to 0x080 (M1-F164)."""
    a = t.scratch_addr(s)
    m1 = ('M1-F164',)
    decision = ('same as M1 if, after 0x340 (its own transfer acknowledged), the writes change part of data buffer '
                '0 (read back at 0x2400 against tag a0); differs if buffer 0 is unchanged or only the launched '
                'transfer changes memory')
    variants = (('none', None, 511, 1, 128), ('sink', 0x340, 511, 1, 128), ('sink', 0x340, 511, 1, 32),
                ('sink', 0x340, 511, 1, 0), ('len0', 0x340, 0, 1, 128), ('sel0', 0x340, 511, 0, 128))
    for label, control, dnum, sel, writes in variants:
        for rep in range(2):
            st = Stages(a + 0x2c00, (0x048, 0x038, 0x034, 0xc0c))
            body = Q.setup() + Q.dma(a, 0) + st('after-fill')
            body += Q.wr(0x38, 7) + Q.wr(0x30, dnum) + Q.wr(0x14, sel) + Q.wr(0x34, a + 0x2800)
            if control:
                body += Q.wr(4, control) + Q.wait(2) + st('ctrl-wait2') + ACK2 + st('ctrl-ack')
            body += ''.join(Q.wr(0x80, 0x5a000000 + k) for k in range(writes)) + st('after-writes')
            body += ''.join(Q.store(0x80, a + 0x2c80 + 4 * k) for k in range(4))
            body += Q.dma(a + 0x2400, 0, into=False).replace(Q.wait(2), Q.wait(2) + st('readback'))
            yield Case(f'{label}/w{writes}/rep{rep}', fin(t, body, 0),
                       dict(scratch=scratch(0x3000, x0=tagged(0xa0)), scratch_read=0x2d00, flags=WAIT0,
                            start_value=SHORT, timeout_us=50000),
                       cond(m1, decision, control=control, dnum=dnum, data_sel=hex(sel), writes=writes, rep=rep,
                            stages=st.names, stage_regs=['0x48', '0x38', '0x34', '0xc0c'],
                            reference={'0x2400 (unchanged buffer 0)': sha(tagged(0xa0))},
                            layout='0x2400 buffer 0 read back; 0x2800 sink of the 0x340-launched transfer (0xee = '
                                   'none); 0x2c00+0x10k: 048 038 034 c0c per stage; 0x2c80: four 0x080 reads after '
                                   'the writes; written values 0x5a000000+k',
                            fix='p36 confound: 0x340 launched an unacknowledged output with the programmed DMA '
                                'registers; now sunk to 0x2800 and acknowledged before the writes'))


# ----------------------------------------------------------- m2_m1f_two_plane (M1-F122)

def two_plane(s, t):
    """Two-plane program 80h/11h/81h/10h and erase 60h/60h/D0h on blocks 3000/3001 (M1-F122)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F122',)
    decision = ('same as M1 if the two-plane program reads back exactly on both blocks and the two-plane erase '
                'leaves both pages 0xff; differs if either form fails')
    b0, b1 = g.block, g.block + 1
    page = 4096 + g.spare
    r0, r1 = b0 * g.pages, b1 * g.pages
    A, B = rand(0x2201, page), rand(0x2202, page)
    mem = scratch(0x4900, x0=A, x1200=B)
    stat = a + 0x4800
    scope = ('scratch blocks: block %d on CE%d (campaign geometry) and, in this family only, block %d on the same '
             'CE as the second plane block (plane = block bit 0)' % (b0, g.ce, b1))
    layout = '0x2400 read-back block %d, 0x3600 read-back block %d (raw %d bytes); 0x4800+4k: 04c after step k' % (
        b0, b1, page)
    opts = dict(flags=MUTATE, timeout_us=400000, scratch=mem, scratch_read=0x4840)

    def data(src):
        return ''.join(Q.dma(src + pos) + Q.transfer(min(512, page - pos), into=False) for pos in range(0, page, 512))

    def readback(row0, row1):
        return (Q.read_page(g, row0, a + 0x2400, 0, 0, raw=True) + Q.read_page(g, row1, a + 0x3600, 0, 0, raw=True))

    yield Case('prepare/erase-single', fin(t, Q.erase(g, block=b0) + Q.store(0x4c, stat) + Q.erase(g, block=b1)
                                         + Q.store(0x4c, stat + 4) + readback(r0, r1)), dict(opts),
               cond(m1, decision, blocks=[b0, b1], ce=g.ce, scope=scope, layout=layout, expected='both 0xff',
                    role='control: single-plane erase of both blocks'), pins=True)
    body = (Q.setup(g.ce) + Q.command(0x80) + Q.address(r0) + data(a) + Q.command(0x11) + Q.ready()
            + Q.store(0x4c, stat) + Q.command(0x81) + Q.address(r1) + data(a + 0x1200) + Q.command(0x10) + Q.ready()
            + Q.store(0x4c, stat + 4) + readback(r0, r1))
    yield Case('program2p/page0', fin(t, body), dict(opts),
               cond(m1, decision, blocks=[b0, b1], ce=g.ce, rows=[r0, r1], scope=scope, layout=layout,
                    sequence='80h row%d data 11h ready, 81h row%d data 10h ready' % (r0, r1),
                    reference={'0x2400': sha(A), '0x3600': sha(B)}), pins=True)
    body = (Q.setup(g.ce) + Q.command(0x60) + Q.address(r0, cycles=3) + Q.command(0x60) + Q.address(r1, cycles=3)
            + Q.command(0xd0) + Q.ready() + Q.store(0x4c, stat) + readback(r0, r1))
    yield Case('erase2p', fin(t, body), dict(opts),
               cond(m1, decision, blocks=[b0, b1], ce=g.ce, scope=scope, layout=layout, expected='both 0xff',
                    sequence='60h row%d 60h row%d D0h ready' % (r0, r1),
                    note='follows program2p/page0: 0xff proves both blocks were erased by the one D0h'), pins=True)
    body = (Q.write_page(g, r1 + 1, a + 0x1200, b'', raw=True) + Q.store(0x4c, stat)
            + readback(r0 + 1, r1 + 1))
    yield Case('control/program-single-b%d' % b1, fin(t, body), dict(opts),
               cond(m1, decision, blocks=[b1], ce=g.ce, scope=scope, layout=layout, row=r1 + 1,
                    reference={'0x3600': sha(B)}, role='control: block %d accepts a single-plane program' % b1),
               pins=True)
    yield Case('cleanup/erase-single', fin(t, Q.erase(g, block=b0) + Q.store(0x4c, stat) + Q.erase(g, block=b1)
                                          + Q.store(0x4c, stat + 4)), dict(opts),
               cond(m1, decision, blocks=[b0, b1], ce=g.ce, scope=scope, layout=layout, role='cleanup'), pins=True)


def setup_bits(s, t):
    """M1-F159 remaining part: does the set-up word (FMCTRL1 0x33338e0) set FMC_STATUS bits 13/19 again
    once a data read has cleared them? 048 stored after the read, after the set-up word, after its ack."""
    a, g = t.scratch_addr(s), Q.geometry(s)
    opened = Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
    st = lambda k: Q.store(0x48, a + 0x2000 + 4 * k)
    decision = ('same as M1 if the set-up word sets bits 13 and 19 again after a read cleared them (0x2004 has '
                'bits 13/19, 0x2000 not); differs if they stay clear')
    variants = {
        'full-read/setup': opened + Q.transfer(512) + st(0) + Q.wr(4, 0x33338e0) + st(1) + Q.wr(0x48, 0xff00ffe) + st(2),
        'short-read/setup': opened + Q.transfer(23) + st(0) + Q.wr(4, 0x33338e0) + st(1) + Q.wr(0x48, 0xff00ffe) + st(2),
        'full-read/control': opened + Q.transfer(512) + st(0) + Q.wr(0x48, 0) + st(1) + st(2),
    }
    for name, body in variants.items():
        yield F.Case(name, F.finish(t, body), dict(scratch=F.memory(), scratch_read=0x2010, timeout_us=50000),
                     cond(['M1-F159'], decision, layout='0x2000: 048 after the read, after the set-up word '
                          '(control: after a no-op write), after the 0xff00ffe acknowledge'), pins=True)


LATCH_REARM_OPS = {
    'none': '',
    'ctrl1-20': Q.wr(4, 0x20),
    'status-clear': Q.wr(0x48, 0xff00ffe),
    'ce-toggle': 'CE',                      # replaced per geometry below
    'ecc-kick': Q.wr(0x810, 0x1ff) + Q.wr(0x804, 2) + Q.wait(4) + Q.wr(0x810, 0x1ff),
    'address': Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(4, 1) + Q.wait(1) + Q.wr(0x48, 4),
    'setup-word': Q.wr(4, 0x33338e0),
    'ctrl1-100e0': Q.wr(4, 0x100e0),
    'release-30020': Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000),
}


def latch_rearm(s, t):
    """M1-F121/F135: which operation re-arms event 5 for a SECOND status latch issued without the
    0x30020 release (every earlier second latch followed the release). 70h + latch + wait 5, ack only,
    one operation, then a second latch (0xca, no new 70h) + wait 5 at the short window."""
    g = Q.geometry(s)
    decision = ('M1: FMCTRL1 operations re-arm event 5 (setup word, 0x100e0, DMA, address), 0x20 / status clear / '
                'CE toggle / ECC kick do not. Per op: the second wait 5 passes (re-armed) or faults (not re-armed)')
    for op, body in LATCH_REARM_OPS.items():
        if body == 'CE':
            body = Q.wr(0, 0x1077801) + Q.wr(0, 0x1077801 | 2 << g.ce)
        src = (Q.setup(g.ce) + Q.command(0x70) + Q.wr(4, 0xca) + Q.wait(5) + Q.wr(0x48, 0x800000)
               + '    movi r2,#0x77770001\n' + body + Q.wr(4, 0xca) + Q.wait(5) + '    movi r2,#0x77770002\n'
               + Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000))
        yield F.Case(op, F.finish(t, src), dict(start_value=SHORT, timeout_us=20000),
                     cond(['M1-F121', 'M1-F135'], decision, operation=op,
                          layout='r2 = 0x77770002 if the second wait 5 passed'), pins=True)


SPECS = {
    'm2_m1f_dma_then_read': (dma_then_read, 'M1-F036', 'FMCTRL1 0x100 replaced by 0xe2: per-variant waits, serial control, reference, gaps (run late)'),
    'm2_m1f_evjmp': (evjmp, 'M1-F088', 'evjmp departure vs C58 = 4 fault vector (CS_IRQ bit 2 in handler); pending, acked, second-set'),
    'm2_m1f_indirect_reach': (indirect_reach, 'M1-F041', 'pld/pst of r1..r5/r7, pst into the register window, offset bits 17..30 (run last)'),
    'm2_m1f_ecc_csirq': (ecc_csirq, 'M1-F065,M1-F076', 'CS_IRQ bits 5/6 around ECC encode and decode per enable candidate (run last)'),
    'm2_m1f_bank_channel': (bank_channel, 'M1-F079,M1-F170', '+0x400 READ ID with clean logs, C50 after +0x408, primary control (run late)'),
    'm2_m1f_ctrl0_held_dma': (ctrl0_held_dma, 'M1-F081', 'DMA held by FMC_CTRL0 bit 24: snapshots before/after setting it, retarget'),
    'm2_m1f_fifo_write': (fifo_write, 'M1-F164', 'FMC_FIFO writes after 0x340 with its transfer sunk and acknowledged'),
    'm2_m1f_latch_rearm': (latch_rearm, 'M1-F121,M1-F135', 'Second status latch without the 0x30020 release: which operation re-arms event 5'),
    'm2_m1f_setup_bits': (setup_bits, 'M1-F159', 'Set-up word after a data read cleared FMC_STATUS bits 13/19'),
    'm2_m1f_two_plane': (two_plane, 'M1-F122', 'Two-plane program/erase on blocks 3000/3001 (block 3001 only here)'),
}

# Riskiest last: two overlapping FMC operations; the absent +0x400 set; writes to unexplained ECC words;
# pst beyond 0xfff and into the register window.
RUN_LAST = ('m2_m1f_dma_then_read', 'm2_m1f_bank_channel', 'm2_m1f_ecc_csirq', 'm2_m1f_indirect_reach')
