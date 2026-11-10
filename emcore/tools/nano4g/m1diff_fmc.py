"""M1-versus-M2 experiments (m1diff_fmc): one family per comparison.

Each family settles the M2 side of the listed M1 findings; the decision rule is in
the key's crosswalk entry and the case conditions.

Conventions of this module:
- every case ends with FMC 0x044 = 0 (also on the fault vector), except the two
  deliberate pending-enable cases of m2_m1_irq_end_pending;
- every status latch is released (FMCTRL1 0x30020 + FMC_STATUS 0x1800000);
- C60..C6C are never written;
- "C58 = 0" cases use prologue policy 0 with WAIT_END: a timed-out WAIT continues,
  CS_IRQ (0xc0c) is stored after the wait and cleared with 4;
- NAND erase/program only on block 3000 of the geometry CE (and of CE+1 mod 4).
Layouts are relative to the probe scratch address.
"""
import hashlib
import random
import struct

from . import followup_experiments as F, sequences as Q  # noqa: F401
from fmss import probe as P  # noqa: F401
from .protocol import DISCOVERY, MUTATE

Case, delay = F.Case, F.delay
SHORT = 0x07b5          # ≈ 4.4 ms fault window per unmet WAIT
WAIT0 = P.WAIT_END | DISCOVERY
RELEASE = Q.wr(4, 0x30020) + Q.wr(0x48, 0x1800000)
VIC0_RAW, VIC1_RAW = 0x38e00008, 0x38e01008


def trap(extra=''):
    return '\ntrap:\n' + extra + Q.wr(0x44, 0) + Q.wr(0, 0) + '    movi r6,#0xfa17fa17\n    end\n'


def fin(t, body, policy=4, extra=''):
    """F.finish with 0x044 cleared on END and on the fault vector."""
    return F.finish(t, body + Q.wr(0x44, 0), policy, trap(extra))


def tagged(tag, n=512):
    """n bytes; each word = tag << 24 | byte offset (identifies position and source)."""
    return b''.join(struct.pack('<I', (tag << 24) | i) for i in range(0, n, 4))


def scratch(size=0x4000, fill=0xee, **at):
    mem = bytearray([fill]) * size
    for off, data in at.items():
        o = int(off[1:], 16)
        mem[o:o + len(data)] = data
    return mem


def rand(seed, n):
    return random.Random(seed).randbytes(n)


def sha(data):
    return hashlib.sha256(bytes(data)).hexdigest()


def vic(addr):
    """Sequencer read of VIC0/VIC1 RAWINTR to addr, addr+4 (uses r1, r2, r7)."""
    out = ''
    for k, reg in enumerate((VIC0_RAW, VIC1_RAW)):
        out += '    movi r1,#0x%x\n    ldi r2,[r1]\n    movi r7,#0x%x\n    sti [r7],r2\n' % (reg, addr + 4 * k)
    return out


class Log:
    """Sequential register stores: entry k at base + stride*k."""

    def __init__(self, base, regs, stride=None, clear_irq=False):
        self.base, self.regs, self.k, self.clear = base, regs, 0, clear_irq
        self.stride = stride or 4 * len(regs)

    def __call__(self):
        addr = self.base + self.stride * self.k
        self.k += 1
        out = ''.join(Q.store(off, addr + 4 * i) for i, off in enumerate(self.regs))
        return out + (Q.wr(0xc0c, 4) if self.clear else '')


def dma_nowait(addr, slot=0, into=True, size=512, select=None):
    return Q.dma(addr, slot, into, size, select=select).replace(Q.wait(2) + Q.wr(0x48, 0x100000), '')


def open_page(g, row, ce=None):
    return (Q.setup(g.ce if ce is None else ce) + Q.command(0) + Q.address(row) + Q.command(0x30) + Q.ready()
            + Q.command(0))


def read512(g, row, dest, ce=None):
    return open_page(g, row, ce) + Q.transfer(512) + Q.dma(dest, 0, into=False)


def latch():
    return Q.command(0x70) + Q.wr(4, 0xca)


def other_ce(g):
    return (g.ce + 1) % 4


def cond(m1, decision, **kw):
    return dict(m1=list(m1), decision=decision, **kw)


# ---------------------------------------------------------------- m2_m1_fmc28

def fmc28(s, t):
    """FMC 0x028 as the FIFO read position (M1-F060/F145/F155)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F060', 'M1-F145', 'M1-F155')
    decision = ('same as M1 if 0x028 advances by 4 per 0x080 read wrapping at 512, by one per pld+sti iteration, '
                'is cleared by DMA and the set-up word and is otherwise constant; differs if constant/other count')
    readid = (Q.setup(g.ce) + Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1)
              + Q.wr(0x48, 4) + Q.wr(0x30, 7) + Q.wr(0x14, 0x10) + Q.wr(4, 0xe2) + Q.wait(3) + Q.wr(0x48, 8)
              + Q.wr(0x40, 0x52) + delay(20))
    fifo = a + 0x2000  # [0x080 value, 0x028 after] per read

    def reads(n, start=0):
        return ''.join(Q.store(0x80, fifo + 8 * (start + k)) + Q.store(0x28, fifo + 8 * (start + k) + 4)
                       for k in range(n))

    mark = lambda k: Q.store(0x28, a + 0x3000 + 4 * k)  # noqa: E731

    def loop(kind):
        src = {'pld-fifo': '    movi r2,#0x80\n', 'pld-048': '    movi r2,#0x48\n',
               'ldi-sti': '    movi r2,#0x%x\n' % a}[kind]
        insn = '    ldi r1,[r2]\n' if kind == 'ldi-sti' else '    .insn 0x18,1,2,0\n'
        return (src + '    movi r4,#0x%x\n    movi r5,#128\nloop128:\n' % (a + 0x2800) + insn
                + '    sti [r4],r1\n    add r4,r4,#4\n    sub r5,r5,#1\n    bnz r5,loop128\n')

    variants = {
        'setup-word': reads(4) + mark(0) + Q.wr(4, 0x33338e0) + mark(1),
        'fifo-1': reads(1),
        'fifo-4': reads(4),
        'fifo-130': reads(130),
        'dma': reads(4) + mark(0) + Q.dma(a, 0) + mark(1) + Q.dma(a + 0x1000, 0, into=False) + mark(2),
        'plain-32': reads(4) + mark(0) + '    movi r5,#0\n' * 32 + mark(1),
        'pld-fifo-128': mark(0) + loop('pld-fifo') + mark(1),
        'pld-048-128': mark(0) + loop('pld-048') + mark(1),
        'ldi-sti-128': mark(0) + loop('ldi-sti') + mark(1),
        'write-0x028': Q.wr(0x28, 0x5a5) + mark(0) + reads(4) + mark(1),
    }
    common = dict(scratch=F.memory(), scratch_read=0x3010, flags=DISCOVERY | P.FULLMAP, timeout_us=50000)
    layout = ('0x2000+8k: [0x080 read k, 0x028 after it]; 0x2800+4i: pld/ldi loop words; '
              '0x3000+4k: 0x028 at marks; CPU view in fullmap/before/after 0x028')
    for name, body in variants.items():
        yield Case('readid/' + name, fin(t, Q.store(0x28, a + 0x3008) + readid + mark(3) + body), dict(common),
                   cond(m1, decision, variant=name, layout=layout + '; 0x3008 before READ ID, 0x300c after the latch'),
                   pins=True)
    yield Case('noid/fifo-4', fin(t, Q.setup() + reads(4)), dict(common),
               cond(m1, decision, variant='no READ ID, no 0x52 latch', layout=layout), pins=False)


# ------------------------------------------------------ m2_m1_irq044_routing

def irq_ops(a, g):
    """(action leaving its flag pending, own event, acknowledge (+ release))."""
    return {
        'command': (Q.wr(8, 0x70), 0, Q.wr(0x48, 2)),
        'address': (Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1), 1, Q.wr(0x48, 4)),
        'data16': (Q.command(0x90) + Q.address(0, cycles=1) + Q.wr(0x30, 15) + Q.wr(0x14, 1) + Q.wr(4, 0xe2), 3,
                   Q.wr(0x48, 8)),
        'DMA': (dma_nowait(a), 2, Q.wr(0x48, 0x100000)),
        'status': (latch(), 5, RELEASE),
    }


IRQ_SNAP = (0xc0c, 0xc08, 0x048, 0xc50)


def irq_snap(a, k):
    base = a + 0x2000 + 0x20 * k
    return ''.join(Q.store(off, base + 4 * i) for i, off in enumerate(IRQ_SNAP)) + vic(base + 16)


def irq044_routing(s, t):
    """Single 0x044 bits per FMC operation: CS_IRQ, C50, VIC raw and wait2 20 (M1-F065/F076/F102/F146/F150/F161)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F065', 'M1-F076', 'M1-F102', 'M1-F146', 'M1-F150', 'M1-F161')
    decision = ('same as M1 if exactly 0x044 bits 0, 1, 2, 11, 12 route command, address, data, DMA, status into '
                'CS_IRQ bit 4 and VIC0 line 20, CS_IRQ bit 4 persists after the ack while line 20 follows the flag; '
                'differs on another bit, no CS_IRQ bit or another persistence')
    layout = ('0x2000+0x20k: c0c c08 048 c50 VIC0raw VIC1raw; k=0 after op, 1 after wait2 20, '
              '2 after own wait, 3 after ack, 4 after 0x044=0')
    opts = dict(scratch=F.memory(), scratch_read=0x20a0, flags=WAIT0, start_value=SHORT, timeout_us=50000)
    for op, (action, event, ack) in irq_ops(a, g).items():
        for bit in range(32):
            body = (Q.setup(g.ce) + Q.wr(0x44, 1 << bit) + action + delay(300) + irq_snap(a, 0)
                    + '    movi r4,#0x77770001\n    .insn 0x9,20,0,0\n    movi r4,#0x77770002\n' + irq_snap(a, 1)
                    + Q.wr(0xc0c, 4) + F.wait(event) + irq_snap(a, 2) + ack + irq_snap(a, 3)
                    + Q.wr(0x44, 0) + irq_snap(a, 4))
            yield Case(f'{op}/bit{bit:02d}', fin(t, body, 0, RELEASE if op == 'status' else ''), dict(opts),
                       cond(m1, decision, operation=op, enable=hex(1 << bit), own_event=event, c58=0,
                            layout=layout, stage='r4 0x77770002 = wait2 20 finished (passed or timed out)'),
                       pins=True)
    m1_ecc = 'M1: 0x844 bits 0, 2, 5 enable ECC sources; on M2 0x844 is an encode-parity word'
    for off in (None,) + F.ECC_IRQ_CANDIDATES:
        for enable in (0, 0xffffffff):
            label = 'none' if off is None else f'{off:03x}'
            pre = '' if off is None else Q.wr(off, 0xffffffff)
            restore = '' if off is None else Q.wr(off, 0x700 if off == 0x808 else 0)
            body = (Q.setup() + Q.wr(0x44, enable) + Q.dma(a, 0) + pre + Q.ecc(acknowledge=False) + delay(300)
                    + irq_snap(a, 0) + '    movi r4,#0x77770001\n    .insn 0x9,20,0,0\n    movi r4,#0x77770002\n'
                    + irq_snap(a, 1) + Q.wr(0xc0c, 4) + Q.wait(4) + irq_snap(a, 2) + Q.wr(0x810, 1) + irq_snap(a, 3)
                    + restore + Q.wr(0x44, 0) + irq_snap(a, 4))
            yield Case(f'ECC/{label}/en{enable:x}', fin(t, body, 0, restore), dict(opts),
                       cond(m1, decision, operation='ECC encode', candidate=label, enable=hex(enable), c58=0,
                            note=m1_ecc, layout=layout))


# ------------------------------------------------------------- m2_m1_blk100

BLK = tuple(range(0x100, 0x200, 4))
BLK_M1 = tuple(range(0x1b8, 0x1dc, 4))  # M1 writable words 0x1b8..0x1d8


def blk100(s, t):
    """Block 0x100..0x1fc: masks, power-on values, +0x200 copies, effect on timing (M1-F074/F172)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F074', 'M1-F172')
    decision = ('same as M1 if exactly 0x1b8..0x1d8 hold values (0x1b8 one bit, 0x1cc/0x1d0 24 bits) and no word '
                'changes a time or wait2; differs otherwise')
    opts = dict(scratch=F.memory(), scratch_read=0x2400, flags=DISCOVERY | P.FULLMAP, timeout_us=50000)
    body = ''.join(Q.store(w, a + 0x2000 + w - 0x100) for w in BLK)
    body += ''.join(Q.store(w + 0x200, a + 0x2100 + w - 0x100) for w in BLK)
    body += ''.join(Q.store(w, a + 0x2200 + w - 0x3a8) for w in range(0x3a8, 0x3b8, 4))
    yield Case('read-only', fin(t, body), dict(opts),
               cond(m1, decision, layout='0x2000: seq 0x100..0x1fc; 0x2100: 0x300..0x3fc; 0x2200: 0x3a8..0x3b4; '
                    'CPU view: fullmap'))
    for value in (0, 1, 0xffffff, 0xffffffff):
        body = ''
        for w in BLK:
            addr = a + 0x2000 + 4 * (w - 0x100)
            body += Q.store(w, addr, 0) + Q.wr(w, value) + Q.store(w, addr + 4, 1) + Q.store(w + 0x200, addr + 8, 1)
            body += '    st [0x%x],r0\n' % w + Q.store(w, addr + 12, 1)
        yield Case(f'write/{value:08x}', fin(t, body), dict(opts),
                   cond(m1, decision, value=hex(value), restored='each word rewritten with its original value',
                        layout='0x2000+16i (word 0x100+4i): original, read-back, +0x200 copy, after restore'))
    saved = a + 0x3000
    save = ''.join(Q.store(w, saved + 4 * i) for i, w in enumerate(BLK_M1))
    restore = ''.join('    movi r7,#0x%x\n    ldi r0,[r7]\n    st [0x%x],r0\n' % (saved + 4 * i, w)
                      for i, w in enumerate(BLK_M1))
    ops = {
        'dma32': (Q.setup() + '    movi r5,#32\ndmaloop:\n' + Q.dma(a, 0) + Q.dma(a + 0x2000, 0, into=False)
                  + '    sub r5,r5,#1\n    bnz r5,dmaloop\n', False),
        'ecc32': (Q.setup() + Q.dma(a, 0) + '    movi r5,#32\neccloop:\n' + Q.ecc(0, True, 8)
                  + '    sub r5,r5,#1\n    bnz r5,eccloop\n'
                  + ''.join(Q.store(off, a + 0x2200 + off - 0x840) for off in range(0x840, 0x850, 4)), False),
        'open16-read': (Q.setup(g.ce) + '    movi r5,#16\nopenloop:\n' + Q.command(0) + Q.address(g.row)
                        + Q.command(0x30) + Q.ready() + '    sub r5,r5,#1\n    bnz r5,openloop\n'
                        + Q.command(0) + Q.transfer(512) + Q.dma(a + 0x2000, 0, into=False), True),
        'wait3-unmet': (Q.setup() + '    .insn 0x9,5,0,0\n' + Q.wait(3), False),
    }
    for setting in ('untouched', '0', 'ffffff'):
        set_words = '' if setting == 'untouched' else ''.join(Q.wr(w, int(setting, 16)) for w in BLK_M1)
        for op, (code, pins) in ops.items():
            body = save + set_words + code + restore
            o = dict(opts, scratch_read=0x3040, start_value=SHORT, timeout_us=100000)
            yield Case(f'timing/{setting}/{op}', fin(t, body, 4, restore), o,
                       cond(m1, decision, words=[hex(w) for w in BLK_M1], setting=setting, operation=op,
                            measure='elapsed_us (op loops), fault_at_us (wait3-unmet), data at 0x2000/0x2200',
                            layout='0x3000: saved originals (restored at END and in the fault handler)'),
                       pins=pins)


# -------------------------------------------------------- m2_m1_bank_channel

BANK_SNAP = (0x424, 0x448, 0xc50, 0x048, 0x44c, 0xc0c)


def bank_channel(s, t):
    """+0x400 as a second NAND channel: READ ID, DMA loopback, parity buffer sharing (M1-F079/F082/F169/F170)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F079', 'M1-F082', 'M1-F169', 'M1-F170')
    decision = ('same as M1 if +0x424 counts the 8-byte transfer, events 6/7/9 and 8 and C50 bit 6 rise, data loops '
                'back intact through the set\'s own buffers and parity buffer 0 is shared; differs if the words stay 0 '
                'and no event or transfer occurs')
    layout = '0x3000+0x20k: 424 448 c50 048 44c c0c per stage'
    opts = dict(flags=WAIT0 | P.FULLMAP, start_value=SHORT, timeout_us=100000)
    pins_note = 'pins as configured by the boot (no remux); the pin-function variant needs CH1_PINS, refused on M2'
    for code in (0xe2, 0x1c2):
        log = Log(a + 0x3000, BANK_SNAP, 0x20, clear_irq=True)
        body = (Q.setup() + Q.wr(0x404, 0x33338e0) + Q.wr(0x448, 0xff00ffe) + Q.wr(0x400, 0x1077801 | 2 << g.ce)
                + Q.wr(0x408, 0x90) + Q.wait(6) + log() + Q.wr(0x448, 2)
                + Q.wr(0x42c, 0) + Q.wr(0x40c, 0) + Q.wr(0x410, 0) + Q.wr(0x404, 1) + Q.wait(7) + log() + Q.wr(0x448, 4)
                + Q.wr(0x430, 7) + Q.wr(0x414, 0x10) + Q.wr(0x404, code) + Q.wait(9) + log() + Q.wr(0x448, 8)
                + Q.store(0x460, a + 0x3100) + Q.store(0x464, a + 0x3104) + Q.store(0x424, a + 0x3108)
                + Q.wr(0x400, 0))
        yield Case(f'readid/ctrl1-{code:x}', fin(t, body, 0), dict(opts, scratch=F.memory(), scratch_read=0x3110),
                   cond(m1, decision, transfer_code=hex(code), pins=pins_note,
                        layout=layout + ' after wait 6, 7, 9; 0x3100: 460 464 424 at the end'))
    for slot in (0, 1):
        log = Log(a + 0x3000, BANK_SNAP, 0x20, clear_irq=True)
        mem = scratch(0x3200, x0=tagged(0xa1), x200=tagged(0xb2))
        body = (Q.setup() + Q.dma(a + 0x200, slot) + Q.wr(0x400, 0x1077801) + Q.wr(0x404, 0x33338e0)
                + Q.wr(0x438, 7) + Q.wr(0x430, 511) + Q.wr(0x414, 1 << slot) + Q.wr(0x434, a) + Q.wr(0x404, 0x2e0)
                + Q.wait(8) + log() + Q.wr(0x448, 0x100000)
                + Q.wr(0x414, 0x100 << slot) + Q.wr(0x434, a + 0x2000) + Q.wr(0x404, 0x100)
                + Q.wait(8) + log() + Q.wr(0x448, 0x100000) + Q.wr(0x400, 0)
                + Q.dma(a + 0x2200, slot, into=False) + log())
        yield Case(f'loopback/slot{slot}', fin(t, body, 0), dict(opts, scratch=mem, scratch_read=0x3080),
                   cond(m1, decision, slot=slot, pins=pins_note,
                        reference={'0x2000 (+0x400 loopback of 0x0)': sha(tagged(0xa1)),
                                   '0x2200 (primary buffer, tag b2)': sha(tagged(0xb2))},
                        layout='0x0 tag a1 into +0x400 slot, 0x200 tag b2 into primary slot first; '
                               '0x2000 +0x400 output, 0x2200 primary output (0xee = untouched); ' + layout))
    for second in (False, True):
        log = Log(a + 0x3000, BANK_SNAP, 0x20, clear_irq=True)
        mem = scratch(0x3200, x0=tagged(0xc3), x200=tagged(0xd4))
        body = Q.setup() + Q.dma(a, 0, select=0x10).replace(Q.wait(2), Q.wait(2) + log())
        if second:
            body += (Q.wr(0x400, 0x1077801) + Q.wr(0x438, 7) + Q.wr(0x430, 511) + Q.wr(0x414, 0x10)
                     + Q.wr(0x434, a + 0x200) + Q.wr(0x404, 0x2e0) + Q.wait(8) + log() + Q.wr(0x448, 0x100000)
                     + Q.wr(0x400, 0))
        body += Q.dma(a + 0x2000, 0, into=False, select=0x1000).replace(Q.wait(2), Q.wait(2) + log())
        yield Case('parity0/' + ('after-400-dma' if second else 'primary-only'), fin(t, body, 0),
                   dict(opts, scratch=mem, scratch_read=0x3080),
                   cond(m1, decision, second_set_dma=second,
                        layout='0x0 tag c3 -> primary parity buffer 0 (select 0x10); 0x200 tag d4 -> +0x400 parity '
                               'buffer 0; 0x2000 primary parity buffer 0 out (select 0x1000); ' + layout,
                        note='primary-only is the control: whether a parity-buffer DMA works at all on M2'))


# ------------------------------------------------------- m2_m1_ctrl0_dma_bit

def ctrl0_dma_bit(s, t):
    """FMC_CTRL0 bit 24 as the DMA enable (M1-F081)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F081',)
    decision = ('same as M1 if nothing moves and FMC_STATUS bit 20 never sets with bit 24 clear and the loopback is '
                'exact with it set; differs if data moves with bit 24 clear')
    base = 0x1077801
    word = lambda on, ce=None: (base | (0 if ce is None else 2 << ce)) & ~(0 if on else 0x1000000)  # noqa: E731
    src = tagged(0x24)
    for slot in (0, 1):
        for din, dout in (('set', 'set'), ('clear', 'clear'), ('clear', 'set'), ('set', 'clear')):
            log = Log(a + 0x3000, (0x048, 0x038, 0xc0c), 0x10, clear_irq=True)
            body = Q.setup()
            for direction, state, addr in (('in', din, a), ('out', dout, a + 0x2000)):
                body += Q.wr(0, word(state == 'set')) + dma_nowait(addr, slot, direction == 'in') + Q.wait(2)
                body += log() + Q.wr(0x48, 0x100000) + log()
            body += Q.wr(0, base)
            yield Case(f'loop/slot{slot}/in-{din}/out-{dout}', fin(t, body, 0),
                       dict(scratch=scratch(0x3100, x0=src), scratch_read=0x3040, flags=WAIT0, start_value=SHORT,
                            timeout_us=50000),
                       cond(m1, decision, slot=slot, bit24_in=din, bit24_out=dout, reference={'0x2000': sha(src)},
                            layout='0x2000 output (0xee = nothing moved); 0x3000+0x10k: 048 038 c0c after wait2 '
                                   'and after ack, in then out'))
    for state in ('set', 'clear'):
        log = Log(a + 0x5000, (0x048, 0x038, 0xc0c), 0x10, clear_irq=True)
        body = open_page(g, g.row)
        for pos in range(0, 4096 + g.spare, 512):
            body += Q.transfer(min(512, 4096 + g.spare - pos)) + Q.wr(0, word(state == 'set', g.ce))
            body += dma_nowait(a + 0x3000 + pos, 0, into=False) + Q.wait(2) + log() + Q.wr(0x48, 0x100000)
            body += Q.wr(0, word(True, g.ce))
        yield Case(f'page/bit24-{state}', fin(t, body, 0),
                   dict(scratch=scratch(0x5100), scratch_read=0x5100, flags=WAIT0, start_value=SHORT,
                        timeout_us=200000),
                   cond(m1, decision, bit24_during_dma=state, row=g.row,
                        layout='0x3000 raw page (DMA phases only with bit 24 as named); 0x5000+0x10k: 048 038 c0c'),
                   pins=True)


# --------------------------------------------------------- m2_m1_dma_length

DNUMS = (0, 15, 255, 511, 512, 1023, 2047, 0x1fff)


def dma_length(s, t):
    """DMA length versus FMC_DNUM (M1-F099)."""
    a = t.scratch_addr(s)
    m1 = ('M1-F099',)
    decision = 'same as M1 if every DMA moves exactly 512 bytes regardless of FMC_DNUM; differs otherwise'
    src = b''.join(tagged(0x40 + k) for k in range(16))  # 8 KiB, tag = 512-byte block
    for mode in ('both', 'in', 'out'):
        for n in DNUMS:
            if mode != 'both' and n == 511:
                continue
            din, dout = (n, n) if mode == 'both' else (n, 511) if mode == 'in' else (511, n)
            log = Log(a + 0x5000, (0x048, 0x038, 0xc0c), 0x10, clear_irq=True)
            body = (Q.setup() + Q.wr(0x38, 7) + Q.wr(0x30, din) + Q.wr(0x14, 1) + Q.wr(0x34, a) + Q.wr(4, 0x2e0)
                    + Q.wait(2) + log() + Q.wr(0x48, 0x100000)
                    + Q.wr(0x30, dout) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
                    + Q.wait(2) + log() + Q.wr(0x48, 0x100000))
            yield Case(f'{mode}/dnum{n:04x}', fin(t, body, 0),
                       dict(scratch=scratch(0x5100, x0=src), scratch_read=0x5040, flags=WAIT0, start_value=SHORT,
                            timeout_us=50000),
                       cond(m1, decision, dnum_in=din, dnum_out=dout,
                            layout='0x0..0x2000 source (word = (0x40+block)<<24 | offset); 0x2000..0x4000 output '
                                   'pre-filled 0xee; 0x5000+0x10k: 048 038 c0c after in, out'))



LATCH_RUNS = 40


# ------------------------------------------------------- m2_m1_parity_buffer

def parity_buffer(s, t):
    """Size of a parity buffer: a 512-byte DMA into it, read back by DMA and by programming 128 bytes (M1-F114)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F114',)
    decision = ('same as M1 if the buffer is 64 bytes and holds the ramp\'s last 64 bytes (wrap); differs if it '
                'holds more or another part')
    src = tagged(0x5a)
    yield Case('erase', fin(t, Q.erase(g)), dict(flags=MUTATE, timeout_us=200000),
               cond(m1, decision, block=g.block), pins=True)
    for k, select in enumerate((0x10, 0x1000)):
        log = Log(a + 0x3800, (0x048, 0x038, 0xc0c), 0x10, clear_irq=True)
        into = Q.dma(a, 0, select=select).replace(Q.wait(2), Q.wait(2) + log())
        body = Q.setup() + into + Q.dma(a + 0x2000, 0, into=False, select=0x1000).replace(Q.wait(2), Q.wait(2) + log())
        yield Case(f'dmaout/in-sel{select:x}', fin(t, body, 0),
                   dict(scratch=scratch(0x3900, x0=src), scratch_read=0x3840, flags=WAIT0, start_value=SHORT,
                        timeout_us=50000),
                   cond(m1, decision, select_in=hex(select), select_out='0x1000',
                        layout='0x2000: parity buffer 0 DMA out (0xee untouched); word = 0x5a<<24 | source offset'))
        log = Log(a + 0x3800, (0x048, 0x038, 0xc0c, 0x04c), 0x10, clear_irq=True)
        row = g.row + k
        body = (Q.setup(g.ce) + Q.command(0x80) + Q.address(row) + Q.dma(a, 0, select=select).replace(Q.wait(2), Q.wait(2) + log())
                + Q.wr(0x30, 127) + Q.wr(0x14, 0x1000) + Q.wr(4, 0x34) + Q.wait(3) + log() + Q.wr(0x48, 8)
                + Q.command(0x10) + Q.ready() + log() + read512(g, row, a + 0x3000))
        yield Case(f'program/in-sel{select:x}/page{k}', fin(t, body, 0, RELEASE),
                   dict(scratch=scratch(0x3900, x0=src), scratch_read=0x3840, flags=WAIT0 | MUTATE, start_value=SHORT,
                        timeout_us=200000),
                   cond(m1, decision, select_in=hex(select), row=row, transmitted=128,
                        layout='0x3000: raw first 512 bytes of the page (128 from the parity buffer, then 0xff); '
                               '0x3800+0x10k: 048 038 c0c 04c'),
                   pins=True)


# --------------------------------------------------- m2_m1_fmc_status_bit27

def fmc_status_bit27(s, t):
    """FMC_STATUS bit 27 after each sector transfer of programmed versus erased pages (M1-F023/F117)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F023', 'M1-F117')
    decision = ('same as M1 if bit 27 sets for every written sector and stays clear for erased ones; differs if it '
                'sets for erased data or does not track the written state')
    mutate = dict(flags=MUTATE, timeout_us=200000)
    yield Case('erase', fin(t, Q.erase(g)), dict(mutate), cond(m1, decision, block=g.block), pins=True)
    pages = {0: bytes(4096), 1: rand(0x27, 4096)}
    for p, data in pages.items():
        mem = scratch(0x1000, x0=data)
        yield Case(f'program{p}', fin(t, Q.write_page(g, g.row + p, a, F.BCH_META)),
                   dict(mutate, scratch=mem, scratch_read=0),
                   cond(m1, decision, page=p, data='zeros' if p == 0 else 'random seed 0x27', sha256=sha(data)),
                   pins=True)
    for p, state in ((0, 'zeros'), (1, 'random'), (2, 'erased')):
        log = Log(a + 0x2000, (0x048,))
        body = Q.wr(0x48, 0x0ff00ffe) + open_page(g, g.row + p) + log()
        for sector in range(8):
            slot = sector & 1
            body += Q.transfer(g.parity + (10 if sector == 0 else 0), slot, parity=True) + log()
            body += Q.transfer(512, slot) + log() + Q.wr(0x48, 0x0ff00ffe) + log()
        yield Case(f'read{p}/{state}', fin(t, body), dict(scratch=F.memory(), scratch_read=0x2080, timeout_us=100000),
                   cond(m1, decision, page=p, state=state,
                        layout='0x2000: 048 after open, then per sector: after parity, after data, after 0x0ff00ffe'),
                   pins=True)


# ---------------------------------------------------------- m2_m1_latch_arm

ARM_REGS = (0x048, 0x04c, 0xc0c, 0xc14)


def arm_intermediates(a, g):
    return {
        'setup-word': Q.wr(4, 0x33338e0),
        'ctrl1-100e0': Q.wr(4, 0x100e0),
        'data16': Q.transfer(16),
        'dma': Q.dma(a, 0),
        'address': Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(0x10, 0) + Q.wr(4, 1) + Q.wait(1) + Q.wr(0x48, 4),
        'ctrl1-20': Q.wr(4, 0x20),
        'status-clear': Q.wr(0x48, 0xff00ffe),
        'ce-toggle': Q.wr(0, 0x1077801) + Q.wr(0, 0x1077801 | 2 << g.ce),
        'ecc-kick': Q.ecc(0, True, 8),
    }


def latch_arm(s, t):
    """Which operations re-arm wait 5 after a status latch (M1-F063/F066/F087/F121/F135/F156)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F063', 'M1-F066', 'M1-F087', 'M1-F121', 'M1-F135', 'M1-F156')
    decision = ('same as M1 if (a) fails, the second latch in (b) fails, exactly the FMCTRL1 operations and address '
                'cycles re-arm, (d) sets no bit 23 and leaves 0x90, and busy-to-ready re-arms; differs if M2 passes '
                'cases M1 fails (e.g. every latch polls until ready) or vice versa')
    layout = '0x2000+0x10k: 048 04c c0c c14 per logged stage (c0c cleared with 4 after each)'

    def core(kind, log, between=''):
        if kind == 'a':
            return Q.wr(8, 0x70) + Q.wr(4, 0xca) + log() + Q.wait(5) + log() + RELEASE + Q.wr(0x48, 2) + log()
        return (latch() + Q.wait(5) + log() + RELEASE + log() + between + Q.wr(4, 0xca) + Q.wait(5) + log()
                + RELEASE + log())

    def tail(log):
        return Q.wr(8, 0x70) + Q.wait(0) + log() + Q.wr(0x48, 2)

    def opts(**kw):
        o = dict(scratch=F.memory(), scratch_read=0x3000, flags=WAIT0, start_value=SHORT, timeout_us=200000)
        o.update(kw)
        return o

    singles = {'a/no-wait0': ('a', ''), 'b/second-latch': ('b', '')}
    singles.update({f'c/{name}': ('b', code) for name, code in arm_intermediates(a, g).items()})
    for name, (kind, between) in singles.items():
        log = Log(a + 0x2000, ARM_REGS, clear_irq=True)
        body = Q.setup(g.ce) + core(kind, log, between) + tail(log)
        yield Case(name, fin(t, body, 0, RELEASE), opts(),
                   cond(m1, decision, form=name, between=between or None, layout=layout,
                        stages='(a): after latch, after wait5, after release; (b)/(c): after wait5, after release, '
                               'after second wait5, after release; last: next command wait0'),
                   pins=True)
    log = Log(a + 0x2000, ARM_REGS, clear_irq=True)
    body = Q.setup(g.ce) + Q.command(0x90) + Q.wr(4, 0xca) + Q.wait(5) + log() + RELEASE + log() + tail(log)
    yield Case('d/90h-latch', fin(t, body, 0, RELEASE), opts(),
               cond(m1, decision, form='(d) 90h then latch', layout=layout), pins=True)
    ces = (g.ce, other_ce(g))
    for order in (ces, ces[::-1]):
        log = Log(a + 0x2000, ARM_REGS, clear_irq=True)
        sel = lambda ce: Q.wr(0, 0x1077801 | 2 << ce)  # noqa: E731
        body = Q.setup(order[0])
        for ce in order:
            body += sel(ce) + Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0)
        for _ in range(2):
            for ce in order:
                body += sel(ce) + latch() + Q.wait(5) + log() + RELEASE
        body += sel(g.ce) + tail(log)
        yield Case(f'e/two-chip/ce{order[0]}-ce{order[1]}', fin(t, body, 0, RELEASE),
                   opts(flags=WAIT0 | MUTATE, start_value=0x20b5),
                   cond(m1, decision, form='(e) erase block %d on both CEs, then two rounds of polls' % g.block,
                        order=list(order), fault_window='0x20b5 ≈ 16.7 ms; erase ≈ 2.27 ms', layout=layout),
                   pins=True)
    for name, (kind, between) in singles.items():
        log = Log(a + 0x2000, ARM_REGS, clear_irq=True)
        body = Q.setup(g.ce) + ''.join(core(kind, log, between) for _ in range(LATCH_RUNS)) + tail(log)
        yield Case(f'f/{name}/x{LATCH_RUNS}', fin(t, body, 0, RELEASE), opts(timeout_us=1000000),
                   cond(m1, decision, form='(f) %s repeated %d times without reset' % (name, LATCH_RUNS),
                        layout=layout), pins=True)




# ----------------------------------------------- m2_m1_program_erase_matrix

PAGE = 4096 + 128


def program_erase_matrix(s, t):
    """Raw program/erase read-back on CE0 and CE1 and two CEs in one run (M1-F122)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F122',)
    decision = 'same as M1 if every page reads back exactly and erases leave 0xff; differs if any form fails'
    if g.strength != 8:
        return
    ce0, ce1 = g.ce, other_ce(g)
    src = {ce0: a, ce1: a + 0x1200}
    dest = {ce0: a + 0x2400, ce1: a + 0x3600}
    stat = a + 0x4800
    data = {ce0: rand(0x122, PAGE), ce1: rand(0x123, PAGE)}
    mem = scratch(0x4900, x0=data[ce0], x1200=data[ce1])
    mutate = dict(flags=MUTATE, timeout_us=300000, scratch=mem, scratch_read=0x4840)
    layout = '0x2400 read-back CE%d, 0x3600 read-back CE%d (raw 4224 bytes); 0x4800: 04c after each op' % (ce0, ce1)
    for ce in (ce0, ce1):
        yield Case(f'erase/ce{ce}', fin(t, Q.erase(g, ce=ce) + Q.store(0x4c, stat)), dict(mutate),
                   cond(m1, decision, ce=ce, block=g.block, layout=layout), pins=True)
    refs = {'0x%x' % (dest[ce] - a): sha(data[ce]) for ce in (ce0, ce1)}
    for ce in (ce0, ce1):
        body = (Q.write_page(g, g.row, src[ce], b'', ce=ce, raw=True) + Q.store(0x4c, stat)
                + Q.read_page(g, g.row, dest[ce], 0, 0, raw=True, ce=ce))
        yield Case(f'program/ce{ce}/page0', fin(t, body), dict(mutate),
                   cond(m1, decision, ce=ce, row=g.row, reference={'0x%x' % (dest[ce] - a): sha(data[ce])},
                        layout=layout), pins=True)
    body = ''
    for k, ce in enumerate((ce0, ce1)):
        body += Q.write_page(g, g.row + 1, src[ce], b'', ce=ce, raw=True) + Q.store(0x4c, stat + 4 * k)
    for ce in (ce0, ce1):
        body += Q.read_page(g, g.row + 1, dest[ce], 0, 0, raw=True, ce=ce)
    yield Case('program/two-ce/page1', fin(t, body), dict(mutate),
               cond(m1, decision, ces=[ce0, ce1], row=g.row + 1, reference=refs, layout=layout), pins=True)
    for ce in (ce1, ce0):
        body = (Q.erase(g, ce=ce) + Q.store(0x4c, stat) + Q.read_page(g, g.row, dest[ce0], 0, 0, raw=True, ce=ce)
                + Q.read_page(g, g.row + 1, dest[ce1], 0, 0, raw=True, ce=ce))
        yield Case(f'erase-check/ce{ce}', fin(t, body), dict(mutate),
                   cond(m1, decision, ce=ce, expected='pages 0 and 1 all 0xff',
                        layout='0x2400 page 0, 0x3600 page 1 of CE%d; 0x4800: 04c after the erase' % ce), pins=True)


# ---------------------------------------------------- m2_m1_status_datasel

def ready_sel(sel):
    return Q.command(0x70) + Q.wr(0x14, sel) + Q.wr(4, 0xca) + Q.wait(12, 0x40, 0x40)


def status_datasel(s, t):
    """Status latch with FMC_DATA_SEL 0x100 versus 1, idle and through multi-page programs (M1-F123/F128/F129)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F123', 'M1-F128', 'M1-F129')
    decision = ('same as M1 if DATA_SEL 0x100 returns 0x70 and leaves alternate pages erased while DATA_SEL 1 writes '
                'every page; differs if DATA_SEL 0x100 returns the real status byte')
    for ce in (g.ce, None):
        for sel in (1, 0x100):
            body = (Q.setup(ce) + Q.command(0x70) + Q.wr(0x14, sel) + Q.wr(4, 0xca) + Q.wait(5)
                    + Q.store(0x4c, a + 0x2000) + Q.store(0x48, a + 0x2004) + Q.store(0xc0c, a + 0x2008) + RELEASE)
            yield Case(f'idle/ce{ce}/sel{sel:x}', fin(t, body, 0, RELEASE),
                       dict(scratch=F.memory(), scratch_read=0x2010, flags=WAIT0, start_value=SHORT, timeout_us=50000),
                       cond(m1, decision, ce=ce, data_sel=hex(sel), layout='0x2000: 04c 048 c0c after wait5'),
                       pins=ce is not None)
    srcs = b''.join(tagged(0x80 + p) for p in range(8))
    for sel in (0x100, 1):
        body = Q.erase(g)
        for p in range(8):
            body += (Q.command(0x80) + Q.address(g.row + p) + Q.dma(a + 512 * p) + Q.transfer(512, into=False)
                     + Q.command(0x10) + ready_sel(sel) + Q.store(0x4c, a + 0x3000 + 4 * p) + RELEASE)
        for p in range(8):
            body += (Q.command(0) + Q.address(g.row + p) + Q.command(0x30) + ready_sel(1) + RELEASE + Q.command(0)
                     + Q.transfer(512) + Q.dma(a + 0x2000 + 512 * p, 0, into=False))
        yield Case(f'multipage/sel{sel:x}', fin(t, body, 4, RELEASE),
                   dict(scratch=scratch(0x3100, x0=srcs), scratch_read=0x3020, flags=MUTATE, timeout_us=300000),
                   cond(m1, decision, data_sel=hex(sel), pages=8, block=g.block, dma_ctrl=7,
                        reference={'0x%x' % (0x2000 + 512 * p): sha(tagged(0x80 + p)) for p in range(8)},
                        layout='0x2000+512p: first 512 bytes of page p (0xff = not written); 0x3000+4p: 04c per poll'),
                   pins=True)




# --------------------------------------------------- m2_m1_fmc_status_low

def fmc_status_low(s, t):
    """FMC_STATUS bits 12, 13, 18, 19 after reset, W1C, reads, DMA, latch and set-up word (M1-F159)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F159',)
    decision = ('same as M1 if the four bits are set after reset, survive writing 1, a 16-byte read clears 13/19, a '
                '512-byte read clears all four and DMA/latch/set-up word set 13/19 again; differs otherwise')
    opened = Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
    steps = {
        'w1c-c3000': Q.wr(0x48, 0xc3000),
        'read16': opened + Q.transfer(16),
        'read512': opened + Q.transfer(512),
        'dma-data': Q.dma(a, 0),
        'dma-parity': Q.dma(a, 0, select=0x10),
        'latch': latch() + Q.wait(5),
        'setup-word': Q.wr(4, 0x33338e0),
    }
    opts = dict(scratch=F.memory(), scratch_read=0x2080, flags=WAIT0, start_value=SHORT, timeout_us=100000)
    log = Log(a + 0x2000, (0x048,))
    order = ('w1c-c3000', 'setup', 'read16', 'read512', 'dma-data', 'read512', 'dma-parity', 'read512', 'latch',
             'release', 'read512', 'setup-word')
    body = log()
    for step in order:
        body += {'setup': Q.setup(g.ce), 'release': RELEASE}.get(step) or steps[step]
        body += log()
    yield Case('series', fin(t, body, 0, RELEASE), dict(opts),
               cond(m1, decision, order=('reset',) + order, layout='0x2000+4k: 048 after each step (k=0 reset)'),
               pins=True)
    for name, step in steps.items():
        log = Log(a + 0x2000, (0x048,))
        body = log() + Q.setup(g.ce) + log()
        if name in ('dma-data', 'dma-parity', 'latch', 'setup-word'):
            body += steps['read512'] + log()
        body += step + log() + (RELEASE + log() if name == 'latch' else '')
        yield Case(name, fin(t, body, 0, RELEASE), dict(opts),
                   cond(m1, decision, step=name,
                        layout='0x2000+4k: 048 at reset, after setup, [after a clearing 512-byte read], after the '
                               'step[, after release]'),
                   pins=True)


# --------------------------------------------------------- m2_m1_event8

def event8(s, t):
    """wait 8 after a page read on CE0 through the first register set (M1-F163)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F163',)
    decision = 'same as M1 if wait 8 passes after the page read; differs if it faults'
    data = rand(0x163, PAGE)
    yield Case('prepare', fin(t, Q.erase(g) + Q.write_page(g, g.row, a, b'', raw=True)),
               dict(flags=MUTATE, timeout_us=200000, scratch=scratch(0x1200, x0=data), scratch_read=0),
               cond(m1, decision, step='erase and raw program of page 0'), pins=True)

    def read(kind):
        ready = (latch() + Q.wait(5) + RELEASE) if kind == 'wait5' else Q.ready()
        return (Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + ready + Q.command(0)
                + Q.transfer(512) + Q.dma(a + 0x2000, 0, into=False))

    snap = ''.join(Q.store(off, a + 0x2200 + 4 * i) for i, off in enumerate((0xc0c, 0xc08, 0x048, 0xc50)))
    wait8 = '    movi r4,#0x77770001\n' + Q.wait(8) + '    movi r4,#0x77770002\n' + snap
    variants = (('read-wait5/wait8-c58-0', read('wait5'), 0), ('read-wait5/wait8-c58-4', read('wait5'), 4),
                ('read-ready12/wait8-c58-4', read('ready12'), 4), ('idle/wait8-c58-4', Q.setup(), 4))
    for name, prefix, policy in variants:
        yield Case(name, fin(t, prefix + wait8, policy, RELEASE),
                   dict(scratch=F.memory(), scratch_read=0x2210, flags=WAIT0 if policy == 0 else DISCOVERY,
                        start_value=SHORT, timeout_us=50000),
                   cond(m1, decision, variant=name, c58=policy, reference={'0x2000': sha(data[:512])},
                        layout='0x2000 sector data; 0x2200: c0c c08 048 c50 after wait 8',
                        not_tested='wait 8 right after power-on: the campaign cannot run before the first FMSS use'),
                   pins=name != 'idle/wait8-c58-4')


# ------------------------------------------------------- m2_m1_fifo_port

def fifo_port(s, t):
    """FMC_FIFO 0x080 reads of DMA-filled buffers and 128 writes after FMCTRL1 0x340 (M1-F164)."""
    F._LABEL[0] = 0
    a = t.scratch_addr(s)
    m1 = ('M1-F164',)
    decision = 'same as M1 if all reads return 0 and the writes change part of data buffer 0; differs otherwise'
    mem = scratch(0x3000, x0=tagged(0xa0), x200=tagged(0xb1), x400=tagged(0xc2))
    log = lambda base: Log(base, (0x048, 0x038, 0xc0c), 0x10, clear_irq=True)  # noqa: E731
    prep = (Q.setup() + Q.dma(a, 0) + Q.dma(a + 0x200, 1)
            + Q.dma(a + 0x400, 0, select=0x10).replace(Q.wait(2), Q.wait(2) + log(a + 0x2c00)()))
    opts = dict(scratch=mem, scratch_read=0x2d00, flags=WAIT0, start_value=SHORT, timeout_us=50000)
    for buf, sel in (('data0', 0x100), ('data1', 0x200), ('parity0', 0x1000)):
        for control in (None, 0x340):
            body = prep + Q.wr(0x14, sel) + (Q.wr(4, control) if control else '')
            body += ''.join(Q.store(0x80, a + 0x2000 + 4 * k) for k in range(130)) + Q.store(0x28, a + 0x2c10)
            yield Case(f'read/{buf}/ctrl{control or 0:x}', fin(t, body, 0), dict(opts),
                       cond(m1, decision, buffer=buf, data_sel=hex(sel), control=control, reads=130,
                            layout='0x2000+4k: 0x080 read k; 0x2c00: 048 038 c0c after the parity DMA; 0x2c10: 028; '
                                   'buffers: data0 tag a0, data1 tag b1, parity0 tag c2'))
    for control in (None, 0x340):
        body = prep + Q.wr(0x14, 1) + (Q.wr(4, control) if control else '')
        body += ''.join(Q.wr(0x80, 0x5a000000 + k) for k in range(128))
        body += Q.dma(a + 0x2400, 0, into=False)
        yield Case(f'write128/ctrl{control or 0:x}', fin(t, body, 0), dict(opts),
                   cond(m1, decision, control=control, writes=128, reference={'0x2400 (unchanged)': sha(tagged(0xa0))},
                        layout='0x2400: data buffer 0 after 128 writes 0x5a000000+k (was tag a0)'))


# ------------------------------------------------- m2_m1_irq_end_pending

def irq_end_pending(s, t):
    """END with an enabled FMC flag pending (M1-F167). Run last: M1 lost USB."""
    m1 = ('M1-F167',)
    decision = ('same as M1 if the pending enabled flag stops the device answering USB while the acknowledged case '
                'does not; differs if USB keeps working with the flag pending')
    marker = 'movi r2,#0x72657365\nend\n'
    scope = ('the probe resets the engine after END before returning: the pending window is END..cleanup; '
             'observable: batch/USB return, vic0_seen bit 20, following marker cases')
    variants = (('control/acked', 1, True), ('pending/en00000001', 1, False), ('pending/enffffffff', 0xffffffff, False))
    for name, enable, acked in variants:
        body = Q.setup() + Q.wr(0x44, enable) + Q.wr(8, 0x70) + Q.wait(0)
        body += Q.wr(0x48, 2) + Q.wr(0x44, 0) if acked else ''
        # Deliberately no 0x044 clear on the pending variants (that is the test).
        yield Case(name, F.finish(t, body, 4, trap()), dict(flags=DISCOVERY, timeout_us=20000),
                   cond(m1, decision, enable=hex(enable), acknowledged=acked, scope=scope))
        for k in range(5):
            yield Case(f'{name}/after{k}', marker, dict(timeout_us=20000),
                       dict(m1=list(m1), decision=decision, mode='confirm',
                            expected={'why': 'END', 'r': {'2': 0x72657365}}, question='device still runs and answers'))


# ------------------------------------------------- m2_m1_dma_then_read

def dma_then_read(s, t):
    """FMCTRL1 0x100 immediately replaced by 0xe2 (M1-F036)."""
    F._LABEL[0] = 0
    a, g = t.scratch_addr(s), Q.geometry(s)
    m1 = ('M1-F036',)
    decision = ('same as M1 if the DMA is lost (wait 2 times out, no output) while the read completes; differs if the '
                'DMA completes or the read is lost instead')
    dma_out = Q.wr(0x38, 7) + Q.wr(0x30, 511) + Q.wr(0x14, 0x100) + Q.wr(0x34, a + 0x2000) + Q.wr(4, 0x100)
    read1 = Q.wr(0x30, 511) + Q.wr(0x14, 2) + Q.wr(4, 0xe2)
    variants = {
        'immediate': dma_out + Q.wr(4, 0xe2),
        'immediate-sel-slot1': dma_out + Q.wr(0x14, 2) + Q.wr(4, 0xe2),
        'serial-control': dma_out + Q.wait(2) + Q.wr(0x48, 0x100000) + read1,
        'read-then-dma': read1 + Q.wr(0x34, a + 0x2000) + Q.wr(0x14, 0x100) + Q.wr(4, 0x100),
    }
    for name, action in variants.items():
        log = Log(a + 0x3000, (0x048, 0x038, 0xc0c), 0x10, clear_irq=True)
        body = (open_page(g, g.row) + Q.transfer(512, 0) + action + '    movi r4,#0x77770001\n'
                + Q.wait(2) + log() + Q.wait(3) + log() + Q.wr(0x48, 0x100008) + log()
                + Q.dma(a + 0x2200, 1, into=False).replace(Q.wait(2), Q.wait(2) + log()) + log())
        yield Case(name, fin(t, body, 0), dict(scratch=scratch(0x3100), scratch_read=0x3060, flags=WAIT0,
                                              start_value=SHORT, timeout_us=100000),
                   cond(m1, decision, variant=name, row=g.row,
                        layout='0x2000 DMA of buffer 0 (sector 0; 0xee = lost); 0x2200 buffer 1 (the second read); '
                               '0x3000+0x10k: 048 038 c0c after wait2, wait3, ack, final wait2, end',
                        reference='serial-control gives the page bytes for both outputs'),
                   pins=True)


# --------------------------------------------------- m2_m1_wait2_vic1

VIC1_LINE = 22   # VIC1 raw bit seen by the probe in p35 (vic1_seen 0x400000), source unidentified


def wait2_vic1(s, t):
    """wait2 32+k against VIC1 hardware line k: wait2 issued as soon as the sequencer sees the raw bit (M1-F174)."""
    a = t.scratch_addr(s)
    m1 = ('M1-F174',)
    decision = ('same as M1 if wait2 32+k passes only while VIC1 line k is asserted by hardware; differs if it never '
                'passes or passes on the software interrupt (soft case: m2_wait2_soft)')
    bit = 1 << VIC1_LINE
    for n in (32 + VIC1_LINE, VIC1_LINE, 31 + VIC1_LINE, 33 + VIC1_LINE):
        for rep in range(6):
            body = ('    movi r5,#100000\n    movi r1,#0x%x\npoll:\n    ldi r2,[r1]\n    and r3,r2,#0x%x\n'
                    '    bnz r3,found\n    sub r5,r5,#1\n    bnz r5,poll\nfound:\n' % (VIC1_RAW, bit)
                    + '    movi r7,#0x%x\n    sti [r7],r2\n    movi r7,#0x%x\n    sti [r7],r5\n' % (a + 0x2000, a + 0x2004)
                    + '    movi r4,#0x77770001\n    .insn 0x9,%d,0,0\n    movi r4,#0x77770002\n' % n
                    + '    ldi r2,[r1]\n    movi r7,#0x%x\n    sti [r7],r2\n' % (a + 0x2008)
                    + Q.store(0xc0c, a + 0x200c))
            yield Case(f'gated/line{n:02d}/rep{rep}', fin(t, body, 0),
                       dict(scratch=F.memory(), scratch_read=0x2010, flags=WAIT0, start_value=SHORT,
                            timeout_us=100000),
                       cond(m1, decision, line=n, vic1_bit=VIC1_LINE, gate='sequencer polls VIC1 RAWINTR bit %d '
                            '(≤ 100000 reads), then wait2 N' % VIC1_LINE,
                            layout='0x2000: VIC1 raw at the gate, polls left (0 = never seen), VIC1 raw after '
                                   'wait2, c0c; r4 0x77770002 = wait2 finished',
                            controls='N = 22 (VIC0 numbering), 53/55 (neighbours)',
                            not_tested='a CPU-prepared VIC1 peripheral source: the probe offers only VIC soft lines'))
    for rep in range(4):
        body = '    movi r4,#0x77770001\n    .insn 0x9,%d,0,0\n    movi r4,#0x77770002\n' % (32 + VIC1_LINE)
        yield Case(f'free/line{32 + VIC1_LINE}/rep{rep}', fin(t, body, 0),
                   dict(flags=WAIT0, start_value=0x40b5, timeout_us=100000),
                   cond(m1, decision, line=32 + VIC1_LINE, window='0x40b5 ≈ 32 ms',
                        measure='vic1_seen bit %d during the window versus why/fault_at_us' % VIC1_LINE))


SPECS = {
    'm2_m1_fmc28': (fmc28, 'M1-F060,M1-F145,M1-F155', 'FMC 0x028 as the FIFO read position: 0x080 reads, pld/sti, DMA, set-up word'),
    'm2_m1_irq044_routing': (irq044_routing, 'M1-F065,M1-F076,M1-F102,M1-F146,M1-F150,M1-F161',
                             'Single 0x044 bits per FMC operation: CS_IRQ/C50/VIC raw before and after ack, wait2 20; ECC candidates'),
    'm2_m1_blk100': (blk100, 'M1-F074,M1-F172', 'Block 0x100..0x1fc: masks, power-on values, +0x200 copies, effect on timing (run late)'),
    'm2_m1_bank_channel': (bank_channel, 'M1-F079,M1-F082,M1-F169,M1-F170',
                           '+0x400 as a second channel: READ ID, waits 6..9, DMA loopback, parity buffer sharing (run late)'),
    'm2_m1_ctrl0_dma_bit': (ctrl0_dma_bit, 'M1-F081', 'FMC_CTRL0 bit 24 clear/set for DMA in/out and page-read DMA'),
    'm2_m1_dma_length': (dma_length, 'M1-F099', 'DMA length versus FMC_DNUM 0..0x1fff, in, out and both'),
    'm2_m1_parity_buffer': (parity_buffer, 'M1-F114', '512-byte DMA into parity buffer 0: DMA out and 128 bytes programmed'),
    'm2_m1_fmc_status_bit27': (fmc_status_bit27, 'M1-F023,M1-F117', 'FMC_STATUS bit 27 per sector transfer: zero, random, erased pages'),
    'm2_m1_latch_arm': (latch_arm, 'M1-F063,M1-F066,M1-F087,M1-F121,M1-F135,M1-F156',
                        'Status-latch arm rule: second latch, intermediates, 90h, two-chip erase polls, 40-fold series'),
    'm2_m1_program_erase_matrix': (program_erase_matrix, 'M1-F122', 'Raw program/erase read-back on CE0, CE1 and both in one run'),
    'm2_m1_status_datasel': (status_datasel, 'M1-F123,M1-F128,M1-F129', 'Status latch with DATA_SEL 0x100 vs 1, idle and 8-page programs'),
    'm2_m1_fmc_status_low': (fmc_status_low, 'M1-F159', 'FMC_STATUS bits 12/13/18/19 after reset, W1C, reads, DMA, latch, set-up word'),
    'm2_m1_event8': (event8, 'M1-F163', 'wait 8 after a one-sector page read on CE0'),
    'm2_m1_fifo_port': (fifo_port, 'M1-F164', 'FMC_FIFO 0x080: 130 reads per buffer, 128 writes after 0x340'),
    'm2_m1_irq_end_pending': (irq_end_pending, 'M1-F167', 'END with an enabled FMC flag pending, then marker cases (run last)'),
    'm2_m1_dma_then_read': (dma_then_read, 'M1-F036', 'FMCTRL1 0x100 immediately replaced by 0xe2 (run late)'),
    'm2_m1_wait2_vic1': (wait2_vic1, 'M1-F174', 'wait2 32+k gated on VIC1 raw line 22 as seen by the sequencer'),
}

# Ordered last by the campaign; riskiest last. dma_then_read overlaps two FMC operations;
# bank_channel drives the absent +0x400 set; blk100 writes every unknown 0x100-block word;
# irq_end_pending ends with an enabled flag pending (M1 lost USB).
RUN_LAST = ('m2_m1_dma_then_read', 'm2_m1_bank_channel', 'm2_m1_blk100', 'm2_m1_irq_end_pending')
# Not implemented: m2_m1_write_dmalist needs Apple's original WRITE program; the harness
# assembles only its own programs (no original microcode in emCORE).
