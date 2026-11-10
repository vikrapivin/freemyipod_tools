"""Resident cases whose inputs depend on earlier captures.

The worker saves and copies real bytes locally; placeholder instruction words
are patched before execution. No host-supplied value stands in for a reading.
"""
import struct
from . import sequences as Q
from . import m2_experiments as M
from .followup_experiments import Case
from .resident_plan import Planner
from fmss import probe as P

OUT, SCRATCH, PROGRAM = 0x20000, 0x38000, 0x400
R0 = OUT + 88 + 1024 + 512 + 0x18
AFTER_MAP = OUT + 2136 + 40960 + 4096
MARKER = 0xabadc0de
NAMES = {'access', 'rwmask', 'eccchunk2', 'straps'}


def finish(t, body):
    return t.prologue() + body + Q.wr(0, 0) + '    movi r6,#0x600d600d\n    end\n' + t.trap()


def cases(name, s, t):
    if name in ('access', 'rwmask'):
        excluded = {0, 4, 8, 0x38, 0x40, 0x48, 0x60, 0x64, 0x68, 0x80, 0x804, 0x810, 0x81c,
                    0xc00, 0xc04, 0xc0c, 0xc38, 0xc4c, 0xc58, 0xc5c, 0xc60, 0xc64, 0xc6c}
        if name == 'access':
            excluded |= {0x7c} | set(range(0xc18, 0xc38, 4)) | set(range(0x400, 0x800, 4))
            offsets, flags, vias = range(0x1000)[::4], P.FULLMAP, ('cpu', 'sequencer')
        else:
            offsets = [b + k for b in t.windows for k in range(0, 256, 4)]
            flags, vias = 0, ('cpu',)
        for off in offsets:
            if off in excluded: continue
            yield Case(f'{off:03x}/baseline', finish(t, f'    ld r0,[0x{off:x}]\n'), dict(flags=flags))
            first = True
            for value in (0xffffffff, 0):
                for via in vias:
                    body = (Q.wr(off, value) if via == 'sequencer' else '') + f'    ld r0,[0x{off:x}]\n' + Q.wr(off, MARKER)
                    code = P.assemble(t, finish(t, body))
                    patch = code.index(struct.pack('<I', MARKER))
                    actions = [(1, AFTER_MAP + off if flags else R0, 0, 4)] if first else []
                    actions.append((2, 0, PROGRAM + patch, 4))
                    yield Case(f'{off:03x}/{value:08x}/{via}', code,
                               dict(params=[(off, value)] if via == 'cpu' else (), flags=flags, actions=actions),
                               dict(restore='device-captured baseline', offset=off, value=value, via=via))
                    first = False
    elif name == 'eccchunk2':
        p, g = Planner(s, t), Q.geometry(s)
        def emit(fn, *args, **kw):
            start = len(p.cases); p.function(fn, (s, t, *args), kw)
            return p.cases[start:]
        yield from emit(M.erase_block)
        yield from emit(M.program_page, g.row)
        yield from emit(M.page, g.row, raw=True)
        first = True
        for area, start, size in (('meta', 0, 10), ('parity', 10, g.parity), ('data', 10 + g.parity, 512)):
            for count in (0, 1, 2, g.strength, g.strength + 1):
                erase = emit(M.erase_block)[0]
                if first: erase.options['actions'] = [(1, SCRATCH + 0x2000, 0, 4096 + g.spare)]
                first = False
                yield erase
                write = emit(M.program_page, g.row, data=bytes(4096 + g.spare), raw=True)[0]
                actions = [(2, 0, SCRATCH, 4096 + g.spare)]
                for k in range(count):
                    bit = (37 * k) % (size * 8)
                    actions.append((3, 1 << (bit & 7), SCRATCH + start + bit // 8, 1))
                write.options['actions'] = actions
                write.conditions = dict(area=area, flipped_bits=count, source='captured raw codeword')
                yield write
                yield from emit(M.page, g.row)
    elif name == 'straps':
        a = t.scratch_addr(s)
        body = ''
        for port in range(16):
            for i, off in enumerate((0, 4)):
                addr = 0x3cf00000 + port * 32 + off
                body += f'    movi r0,#0x{addr:x}\n    ldi r1,[r0]\n    movi r2,#0x{a + (port * 2 + i) * 4:x}\n    sti [r2],r1\n'
        yield Case('GPIO-PCON-PDAT', finish(t, body), dict(scratch=bytes(128), scratch_read=128))
    else: raise KeyError(name)
