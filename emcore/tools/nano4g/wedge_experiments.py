"""Register transitions, optionally collected individually to locate a lost job."""
import struct
from dataclasses import replace
from fmss import probe as P
from .protocol import DISCOVERY


def wedge_cases(s, t):
    from .followup_experiments import Case
    from .isa_experiments import regnum_cases
    candidates = list(regnum_cases(s, t))
    options = dict(flags=P.WAIT_END | DISCOVERY, timeout_us=20000)
    # Distinguish a PC read from a constant/zero/register alias at two sites.
    for padding in (0, 4):
        source = t.prologue(c58=0) + 'movi r7,#0\n' * padding
        source += '.insn 6,2,8,0\nmovi r6,#0x600d600d\nend\n' + t.trap()
        candidates.append(Case('read-r8/pc-site-%d' % padding, source, options,
                               dict(question='does r8 track the instruction address?', padding=padding)))
    # Known register loads are controls for the r9 load and the system bus.
    base = t.scratch_addr(s)
    for reg in range(8):
        source = t.prologue(c58=0) + 'movi r%d,#0x%x\nldi r2,[r%d]\nend\n' % (reg, base, reg)
        candidates.append(Case('mapped-load/r%d' % reg, source + t.trap(),
            dict(options, scratch=struct.pack('<I', 0xcafe1234), scratch_read=4),
            dict(mode='confirm', expected={'why': 'END', 'r': {'2': 0xcafe1234}}, question='mapped indirect-load control')))
    yield Case('000/baseline', 'movi r6,#0x600d600d\nend\n', options,
               dict(mode='confirm', expected={'why': 'END', 'r': {'6': 0x600d600d}}))
    for index, case in enumerate(candidates):
        ident = '%03d' % (index + 1)
        yield replace(case, name=ident + '/candidate/' + case.name)
        yield Case(ident + '/following-END', 'movi r6,#0x600d600d\nend\n', options,
                   dict(mode='confirm', expected={'why': 'END', 'r': {'6': 0x600d600d}}, question='independent execution after preceding candidate'))


def run_isolated(s, t, name, cases):
    """One candidate per native RUN; durable output before submitting the next.

    Selected only with --isolate-diagnostics; default campaigns batch these
    families too. No poll can identify the last fast case reliably
    if a later instruction stalls the CPU before its next USB response.
    """
    from . import batch as B
    collected = {}
    for index, case in enumerate(cases):
        s.isolation_active = {'index': index, 'case': case.name,
                              'scope': 'one outstanding native request; setup, candidate or cleanup may be responsible'}
        s.log({'type': 'isolation-start', 'experiment': name, 'index': index, 'case': case.name})
        print('PLAN: isolate %s case %d: %s' % (name, index, case.name), flush=True)
        if hasattr(s, 'checkpoint_cases'):
            s.checkpoint_cases(name, collected)
        s.last_batch_records = None
        completed = False
        try:
            records, _ = B.run(s, t, name, [case])
            collected.update(records)
            completed = True
        finally:
            if s.last_batch_records:
                collected.update(s.last_batch_records)
            s.last_batch_records = collected
            if completed:
                s.isolation_active = None
            if hasattr(s, 'checkpoint_cases'):
                s.checkpoint_cases(name, collected)
        s.log({'type': 'isolation-complete', 'experiment': name, 'index': index, 'case': case.name})
    from .followup_experiments import summarize
    return collected, summarize(name, collected) + '; each transition collected before the next RUN'
