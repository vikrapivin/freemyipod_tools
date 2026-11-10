"""M2 operand probes with program-local PC targets and private RAM addresses."""
import struct
from .m2_experiments import experiment
from .followup_experiments import Case, run_cases
from .protocol import DISCOVERY
from fmss import probe as P


def regnum_cases(s, t):
    seed = ''.join('    movi r%d,#0x%x\n' % (i, 0x10101010 * (i + 1)) for i in range(8) if i != 6)
    seed += '    movi r6,#0\n'
    bodies = [('mov r1,r9', '.insn 6,1,9,0'), ('mov r1,r15', '.insn 6,1,15,0'),
              ('mov r1,r255', '.insn 6,1,255,0'), ('add r1,r9,#1', '.insn 0xc,1,9,1'),
              ('st [0xd70],r9', '.insn 2,9,0xd70,0'), ('st [0xd70],r14', '.insn 2,14,0xd70,0')]
    options = dict(params=[(0xd70, 0)], timeout_us=20000, flags=P.WAIT_END | DISCOVERY)
    for label, body in bodies:
        yield Case(label, t.prologue(c58=0) + seed + body + '\nmovi r6,#0x600d600d\nend\n' + t.trap(), options)
    for number in (8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 255):
        # M1 r8 is PC. The old 0x5eed0008 immediate escaped the program.
        # Keep every destination test, with distinct landing/fallthrough markers.
        source = t.prologue(c58=0) + seed + '.insn 5,%d,0,landing\n' % number
        source += 'movi r6,#0xbad0bad0\nend\nlanding:\nmovi r6,#0x600d600d\nend\n' + t.trap()
        yield Case('movi r%d,#landing' % number, source, options,
                   dict(question='destination register or PC alias', register=number,
                        landing=0x600d600d, fallthrough=0xbad0bad0))
    base = t.scratch_addr(s)
    # Any alias to a GPR now supplies a private, mapped address. An unknown
    # extended source can still have another meaning; this is not an ISA claim.
    pointers = ''.join('movi r%d,#0x%x\n' % (i, base + 4*i) for i in range(8))
    source = t.prologue(c58=0) + pointers + '.insn 3,1,9,0\nmovi r6,#0x600d600d\nend\n' + t.trap()
    yield Case('ldi r1,[r9] (all GPRs point into scratch)', source,
               dict(options, scratch=struct.pack('<8I', *(0xcafe0000+i for i in range(8))), scratch_read=32))
    source = t.prologue(c58=0) + seed + '.insn 0x0e,9,0,skip\nmovi r6,#0xbad0bad0\nend\nskip:\nmovi r6,#0x600d600d\nend\n' + t.trap()
    yield Case('bnz r9 taken? (r0 != 0)', source, options)


@experiment('regnum')
def regnum(s, t):
    return run_cases('regnum', s, t, regnum_cases)


def r8_cases(s, t):
    """M1 chapter 4.3 roles; code targets are local, fallback load addresses mapped.

    Direct LDI r8 still deliberately probes a low system-bus address and is
    unqualified on M2. It is distinct from reading the PC with MOV.
    """
    base = t.scratch_addr(s)
    tail = 'movi r6,#0xbad0bad0\nend\nlab:\nmovi r6,#0x11111111\nend\n'
    bodies = [
        ('movi r8,#lab', '.insn 5,8,0,lab\n'),
        ('movi r8,#alternate (local target)', '.insn 5,8,0,alternate\nalternate:\n.insn 5,8,0,lab\n'),
        ('mov r8,r1 (r1=lab)', 'movi r1,#lab\n.insn 6,8,1,0\n'),
        ('mov r1,r8', '.insn 6,1,8,0\n'),
        ('movi r10,#0x12345678', '.insn 5,10,0,0x12345678\n'),
        ('mov r10,r2', '.insn 6,10,2,0\n'),
        ('add r10,r2,#1', '.insn 0xc,10,2,1\n'),
        ('movi r26', '.insn 5,26,0,0x26262626\n'),
        ('movi r24,#lab', '.insn 5,24,0,lab\n'),
        ('ldi r1,[r8] (low-address bus applicability)', '.insn 3,1,8,0\n'),
        ('ldi r1,[r10] (mapped r0 fallback)', '.insn 3,1,10,0\n')]
    for label, body in bodies:
        seed = ''.join('movi r%d,#0x%x\n' % (i, base if i == 0 and label.startswith('ldi') else
                                             0x10101010 * (i + 1)) for i in range(8) if i != 6)
        seed += 'movi r6,#0\n'
        yield Case(label, t.prologue(c58=0) + seed + body + tail + t.trap(),
            dict(flags=P.WAIT_END | P.F_SAMPLE_REGS | DISCOVERY, timeout_us=20000,
                 scratch=struct.pack('<I',0xcafe1234), scratch_read=4),
            dict(basis='M1 chapter 4.3; M2 qualification remains per operand role'))


@experiment('r8')
def r8(s, t):
    return run_cases('r8', s, t, r8_cases)
