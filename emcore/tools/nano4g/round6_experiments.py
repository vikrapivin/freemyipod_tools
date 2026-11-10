"""Round 6: CPU timelines, hardware IRQ stimuli, distant PC pads and original programs.

All register meanings under investigation remain observations. Sampler slots are
explicit in every case. A missed pulse does not establish an absent interrupt;
original Apple programs are assembled without editing their instruction stream.
"""
from pathlib import Path
import hashlib
import struct
import sys

from . import followup_experiments as F, sequences as Q, protocol as U
from .round5_experiments import loop, stores

Case, finish, memory, NOP = F.Case, F.finish, F.memory, F.NOP
ROOT = Path(__file__).resolve().parents[3]
GOOD = 0x600d600d
SOURCES = (0xc08, 0x48, 0x38, 0x34, 0x4c, 0xc1c, 0xc0c, 0xc50)
RAW0, RAW1 = U.vic_param(0, 8), U.vic_param(1, 8)
ITEMS = {'c08': '8', 'read_status': '19', 'latch': '11', 'dma': '15/16',
         'vic1': '13', 'gate5': '13', 'gate_checkpoint': '13', 'pc': '2', 'apple': '24'}


def conditions(key, decision, **kw):
    return dict(key=key, item=ITEMS[key], open_point=key, decision=decision, **kw)


def options(sources=SOURCES, **kw):
    params = U.sampler(sources, changes=True, pc=True)
    params += kw.pop('params', [])
    flags = U.SAMPLER | U.DISCOVERY | kw.pop('flags', 0)
    return dict(flags=flags, params=params, timeout_us=kw.pop('timeout_us', 20000),
                start_value=kw.pop('start_value', 0x07b5), **kw)


def sampled(key, name, source, sources=SOURCES, pins=False, **kw):
    cond = kw.pop('conditions', {})
    return Case(name, source, options(sources, **kw),
                conditions(key, 'classify only observed CPU samples with time and PC; an unsampled transition is unresolved',
                           sources=list(sources), **cond), pins)


def c08(s, t):
    a = t.scratch_addr(s)
    for kind, one in [('nop2', NOP), ('alu', '    add r2,r2,#1\n'),
                      ('window', '    ld r2,[0x48]\n'), ('sdram', '    ldi r2,[r7]\n'),
                      ('wait', Q.wait(1))]:
        for rep in range(3):
            F._LABEL[0] = 0
            body = '    movi r7,#0x%x\n' % a + loop(10000, one * 16)
            yield sampled('c08', '%s/rep%d' % (kind, rep), finish(t, body),
                          scratch=memory(), scratch_read=16, timeout_us=3000,
                          conditions=dict(kind=kind, rep=rep, observation='--observation-min-us 0; CPU samples, not only after-map'))


def read_status(s, t):
    g = Q.geometry(s)
    for n in (1, 32, 128, 129, 255, 256, 257, 383, 384, 385, 511, 512):
        F._LABEL[0] = 0
        body = Q.setup(g.ce) + Q.command(0) + Q.address(g.row) + Q.command(0x30) + Q.ready() + Q.command(0)
        # Repeated transfers supply independent chances to sample the short operation.
        body += loop(4, Q.transfer(n))
        yield sampled('read_status', 'bytes%d' % n, finish(t, body), pins=True,
                      conditions=dict(length=n, repetitions=4, status_field='048 bits19:12'))


def latch(s, t):
    g = Q.geometry(s)
    for gap in (0, 1, 2, 3, 8, 32):
        for acknowledged in (False, True):
            F._LABEL[0] = 0
            action = (Q.wr(8, 0x70) + NOP * gap + Q.wr(4, 0xca)
                      + F.delay(100) + (Q.wr(0x48, 0x800008) if acknowledged else '')
                      + Q.wr(4, 0x30020) + Q.wr(0x48, 0x180000a))
            yield sampled('latch', 'gap%d/ack%d' % (gap, acknowledged),
                          finish(t, Q.setup(g.ce) + loop(16, action)), pins=True,
                          conditions=dict(gap_nops=gap, acknowledge=acknowledged, repetitions=16,
                                          resolution='CPU microsecond timestamps cannot order sub-sample edges'))


def dma(s, t):
    a = t.scratch_addr(s)
    for direction in ('in', 'out'):
        for ctrl in (7, 0xf, 0x17, 0x27, 0x3f):
            for n in (32, 128, 512):
                F._LABEL[0] = 0
                body = Q.setup() + Q.dma(a, 0)
                body += Q.dma(a + (0x2000 if direction == 'out' else 0), into=direction == 'in',
                              size=n, control=ctrl)
                if direction == 'in':
                    body += Q.dma(a + 0x2000, into=False)
                yield sampled('dma', '%s/ctrl%x/bytes%d' % (direction, ctrl, n), finish(t, body),
                              scratch=memory(), scratch_read=0x2200,
                              conditions=dict(direction=direction, control=ctrl, length=n,
                                              payload_offset=0x2000, reference_offset=0))


def vic1(s, t):
    """EIC low/high level stimuli; IRQ52 3de block controls are discovery candidates.

    Restore/ack are performed before END by sequencer stores as well as late CPU
    writes, so an early WAIT release cannot skip the cleanup phase.
    """
    for line in (32, 33, 52):
        variants = ('off', 'low', 'high') if line != 52 else ('off', 'apple-init', 'control0', 'control1', 'control3')
        for variant in variants:
            for event in (line - 1, line, line + 1):
                sources = (RAW0, RAW1, 0xc08, 0xc0c, 0xc1c, 0x397000a0, 0x397000a4, 0x3de0000c)
                early, late, clear = [], [], []
                body = ''
                if line in (32, 33):
                    group, bit = (1, 1) if line == 32 else (0, 4)
                    reg = 4 * group
                    early = [(U.eic_param(U.EIC_CLEAR, 0xc0 + reg), bit),
                             (U.eic_param(U.EIC_SET, 0xe0 + reg), bit),
                             (U.eic_param(U.EIC_SET if variant == 'high' else U.EIC_CLEAR, 0x80 + reg), bit),
                             (U.eic_param(U.EIC_WRITE, 0xa0 + reg), bit)]
                    late = [(U.eic_param(U.EIC_SET, 0xc0 + reg), bit)] if variant != 'off' else []
                    clear = [(U.eic_param(U.EIC_CLEAR, 0xc0 + reg), bit)]
                else:
                    # Save absolute controls in SDRAM and restore in the program's
                    # success/fault paths; these are candidates, never a claimed IRQ.
                    a = t.scratch_addr(s)
                    for i, addr in enumerate((0x3de00000, 0x3de00004, 0x3de00010)):
                        early += [(0xc18 + 4 * i, addr)]
                        body += '    ldi r3,[r%d]\n    movi r7,#0x%x\n    sti [r7],r3\n' % (i, a + 0x3000 + 4*i)
                    late = [] if variant == 'off' else [(0x3de00004, 1), (0x3de00000, 0x1703),
                                                        (0x3de00010, 0 if variant == 'apple-init' else int(variant[-1]))]
                    clear = [(0x3de0000c, 0xffffffff)]
                tail = ''
                if line == 52:
                    for i, addr in enumerate((0x3de00000, 0x3de00004, 0x3de00010)):
                        tail += '    movi r7,#0x%x\n    ldi r3,[r7]\n    movi r7,#0x%x\n    sti [r7],r3\n' % (a + 0x3000+4*i, addr)
                body += '    movi r1,#0\n    .insn 9,%d,0,0\n    movi r1,#1\n' % event
                body += F.delay(10000) + tail
                handler = t.trap().replace('trap:\n', 'trap:\n' + tail)
                yield sampled('vic1', 'line%d/%s/wait%d' % (line, variant, event),
                              finish(t, body, handler=handler), sources=sources, flags=U.EIC,
                              params=early, late=late or clear, late_us=100, late2=clear if late else [], late2_us=300 if late else 0,
                              scratch=memory(), scratch_read=0x3010, timeout_us=20000,
                              conditions=dict(line=line, stimulus=variant, wait2=event,
                                              requirement='raw VIC1 target bit absent before then asserted; compare off control',
                                              stimulus_basis='fw_restore0802dc98 init; IRQ52 ack0802dc8c' if line == 52 else 'EIC level polarity and enable'))


def gate5(s, t):
    for gate in (None, *range(160)):
        F._LABEL[0] = 0
        body = '    movi r1,#0\n' + loop(100000, '    .insn 9,5,0,0\n    add r1,r1,#1\n')
        yield sampled('gate5', 'baseline' if gate is None else 'gate%d' % gate, finish(t, body),
                      sources=(RAW0, RAW1, 0xc08, 0xc1c), flags=U.GATEWIN,
                      params=U.gate_window([] if gate is None else [gate], 100, 1000),
                      timeout_us=3000, conditions=dict(gate=gate, observation='--observation-min-us 0',
                                                      requirement='enabled before; compare VIC0 bit5 before and after restore',
                                                      window_sampling='none: restore clocks before any peripheral or SDRAM read'))


def gate_checkpoint(s, t):
    """P50 tail first, then entry-sensitive gate3, then gate109; USB checkpoint per case."""
    original = {c.conditions['gate']: c for c in gate5(s, t)}
    for gate in (None, *range(110, 160), 3, 109):
        case = original[gate]
        case.options['params'].append((U.cfg_param(U.CFG_GATE, 8), 1))
        case.options['timeout_us'] = 160000
        case.conditions.update(key='gate_checkpoint', open_point='gate_checkpoint', checkpoint=True,
                               observation='candidate starts only after automatic USB checkpoint ACK',
                               rationale='remaining P50 gates, then P51 entry-sensitive gate3, then P50 gate109')
        yield case


def pc(s, t):
    for form in ('bnz', 'jr'):
        for bit in range(16, 25):
            for delta in (0, 0x100):
                target = (1 << bit) + delta
                body = '    movi r0,#1\n    movi r1,#0x%x\n' % target
                body += ('    bnz r0,0x%x\n' % target if form == 'bnz' else '    .insn 0xf,0,1,0\n')
                # Prefix markers distinguish fallthrough from a fetched pad.
                src = t.prologue() + body + '    movi r2,#0xbad00000\n    end\n' + t.trap()
                yield Case('%s/bit%d/delta%x' % (form, bit, delta), src,
                           dict(flags=U.PADS | U.DISCOVERY, params=U.pads(target + 0x10000), timeout_us=20000),
                           conditions('pc', 'r2 pad tag decodes actual landing offset; zero r2/FAULT does not prove a width',
                                      form=form, bit=bit, target=target, span=target+0x10000,
                                      expected_tag=U.PAD_TAG | target >> 4))


def original(name):
    """Return preserved Apple source; assemble locally only for its identity hash."""
    from fmss.target import NANO4G
    from fmss.probe import assemble
    text = (Path(__file__).with_name('programs') / (name + '.s')).read_text()
    code = assemble(NANO4G, text)
    return text, hashlib.sha256(code).hexdigest()


def apple(s, t):
    a, g = t.scratch_addr(s), Q.geometry(s)
    erase_src, erase_sha = original('ERASE')
    write_src, write_sha = original('WRITE')
    for block in (3000, 3001):
        row = block * g.pages
        mem = memory()
        struct.pack_into('<4I', mem, 0x1000, g.ce, row, 0, 0)
        common = dict(flags=U.MUTATE, scratch=mem, scratch_read=0x3200, timeout_us=200000)
        params = [(0xd0c, a+0x1004), (0xd10, a+0x1000), (0xd18, 1), (0xd4c, 0x77000)]
        yield Case('block%d/erase' % block, erase_src, dict(common, params=params),
                   conditions('apple', 'END, result fault bits clear, pointers advanced once; separate raw read proves erase',
                              operation='erase', block=block, ce=g.ce, row=row, original_sha256=erase_sha,
                              expected_after={0xd0c:a+0x1008, 0xd10:a+0x1004, 0xd18:0}), True)
        yield Case('block%d/erased-read' % block,
                   finish(t, Q.read_page(g, row, a+0x2000, a+0x3100, a+0x3140, raw=True)),
                   dict(scratch=memory(), scratch_read=0x3200, timeout_us=200000),
                   conditions('apple', 'all raw page bytes FF after original erase', operation='erased-read',
                              block=block, bytes=4096+g.spare, offset=0x2000), True)
        mem = memory()
        struct.pack_into('<4I', mem, 0x1000, 0x00801000 | 1 << g.ce, row, 0, 0)
        # WRITE fetches one lookahead pointer before seeing the zero descriptor.
        # It never starts a DMA from this third entry.
        struct.pack_into('<3I', mem, 0x1020, a, a+0x800, a)
        mem[0x1040:0x104c] = bytes(range(12))
        params = [(0xd08, 8), (0xd10, a+0x1000), (0xd1c, a+0x1040), (0xd20, a+0x1020),
                  (0xd28, 2), (0xd2c, 2 | (4 if g.strength == 16 else 0)),
                  (0xd30, 0xa02 | (4 if g.strength == 16 else 0)), (0xd34, g.parity+9),
                  (0xd38, g.parity-1), (0xd48, 0x77000), (0xd78, 0)]
        yield Case('block%d/write' % block, write_src, dict(common, scratch=mem, params=params),
                   conditions('apple', 'END and result fault bits clear; decoded read must match all data and metadata',
                              operation='write', block=block, ce=g.ce, row=row, original_sha256=write_sha,
                              expected_after={0xd10:a+0x1010, 0xd20:a+0x102c, 0xd1c:a+0x104c}), True)
        yield Case('block%d/readback' % block,
                   finish(t, Q.read_page(g, row, a+0x2000, a+0x3100, a+0x3140)),
                   dict(scratch=memory(), scratch_read=0x3200, timeout_us=200000),
                   conditions('apple', '4096 data bytes equal input and metadata bytes0..9 equal 00..09',
                              operation='readback', block=block, offset=0x2000), True)


SPECS = {
    'm2_r6_c08': (c08, 'c08self', '#8 CPU C08 timeline'),
    'm2_r6_read_status': (read_status, 'fmcstat', '#19 CPU FMC_STATUS timeline during read'),
    'm2_r6_latch': (latch, 'ev5t', '#11 CPU bit3/bit23 latch timing'),
    'm2_r6_dma': (dma, 'dmactl', '#15/16 CPU DMA progress and delivered bytes'),
    'm2_r6_vic1': (vic1, 'w2vic', '#13 hardware VIC1 lines32/33/52 against wait2'),
    'm2_r6_gate5': (gate5, 'w2vic', '#13 all clock gates versus VIC0 line5'),
    'm2_r6_gate_checkpoint': (gate_checkpoint, 'w2vic', '#13 remaining gates and entry-sensitive cases with USB evidence before start'),
    'm2_r6_pc': (pc, 'branches', '#2 distant tagged END landing pads'),
    'm2_r6_apple': (apple, 'writeseq', '#24 unchanged original WRITE/ERASE blocks3000/3001'),
}
RUN_LAST = ('m2_r6_pc', 'm2_r6_vic1', 'm2_r6_gate5', 'm2_r6_gate_checkpoint')


def run_collected(s, t, name, generated=None):
    """Collect evidence over USB before submitting the next potentially losing case.

    There is no recovery replay: transport loss propagates after retaining all
    completed records and the last submitted case in the durable journal.
    """
    from . import batch as B
    from .capture import compact
    from fmss import probe as P
    records, compact_records, health, timing = {}, {}, {}, {}
    for case in SPECS[name][0](s, t) if generated is None else generated:
        print('PLAN: case %s/%s; collect USB evidence before next case' % (name, case.name), flush=True)
        s.last_batch_records, s.last_batch_compact, s.last_batch_health = {}, {}, {}
        try:
            if getattr(s, 'batch_enabled', True) and getattr(s, 'probe_caps', 0) & U.BATCH:
                B.run(s, t, name, [case])
            else:
                P.set_experiment(name + '/' + case.name)
                r = F.execute(case, s, t)
                s.last_batch_records[case.name] = r
                s.last_batch_compact[case.name] = compact(P.jsonable(r))
                s.log(dict(type='batch-case-result', experiment=name, case=case.name,
                           data=s.last_batch_compact[case.name]))
        finally:
            records.update(getattr(s, 'last_batch_records', None) or {})
            compact_records.update(getattr(s, 'last_batch_compact', None) or {})
            B.merge_health(health, getattr(s, 'last_batch_health', None) or {})
            for k, v in (getattr(s, 'family_timing', None) or {}).items():
                if isinstance(v, (int, float)):
                    timing[k] = timing.get(k, 0) + v
            s.last_batch_records, s.last_batch_compact, s.last_batch_health = records, compact_records, health
    if isinstance(getattr(s, 'family_timing', None), dict):
        s.family_timing.update(timing)
    s.log(dict(type='reset-health', experiment=name, **health))
    return records, F.summarize(name, records)


def run_gate5(s, t, generated=None):
    return run_collected(s, t, 'm2_r6_gate5', generated)
