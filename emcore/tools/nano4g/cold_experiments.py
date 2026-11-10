"""Cold-boot captures: meaningful only as the first FMSS work after a power cycle and DFU.

The kernel does not touch the FMSS at boot (nand_poc_init allocates buffers only;
the NAND routines reset the engine on demand). The first probe opens the clock gates
and, with every probe start/end step disabled, captures the engine as the boot ROM
left it. `n4g.py fmss explore --cold` runs these families alone and records how many
worker jobs ran before them in this boot (0 for a valid cold capture).
"""
from . import followup_experiments as F
from fmss import probe as P

KEEP_CTRL0 = 1 << 0
UNTOUCHED = P.FULLMAP | P.S_NO_RESET | P.S_NO_PREFLIGHT | P.S_NO_CLEAN | KEEP_CTRL0
MARK = 0x636f6c64  # 'cold'


def cold_boot(s, t):
    """Window at power-on; the very first start; state after it; an ordinary reset."""
    common = dict(timeout_us=20000)
    yield F.Case('power-on-window', '    end\n', dict(common, flags=UNTOUCHED),
                 dict(measures='fullmap[0] = the 0x000..0xffc window before any reset, preflight or start in this boot '
                               '(FIFO ports not read); fullmap[1] after the first-ever start of an END program',
                      m1=['M1-F048', 'M1-F078', 'M1-F080'],
                      decision='M1: C14 powers on at 0xff and the soft reset differs from power-on; compare every word '
                               'with the after-reset case'))
    yield F.Case('first-program', '    movi r2,#0x%x\n    st [0xd00],r2\n    end\n' % MARK,
                 dict(common, flags=UNTOUCHED),
                 dict(measures='the first program with a non-END instruction 0 after power-on, started once, no reset: '
                               'r2 / 0xd00 = marker if it ran', m1=['M1-F049'],
                      decision='M1 needs a second start after power-on; M2 after the ordinary reset runs on the first'))
    yield F.Case('after-first-program', '    end\n', dict(common, flags=UNTOUCHED),
                 dict(measures='window left by the first program and its untouched end (no cleanup)'))
    yield F.Case('after-reset', '    end\n', dict(common, flags=P.FULLMAP | KEEP_CTRL0),
                 dict(measures='ordinary probe reset and preflight, then the window: soft-reset values for comparison'))


def reboot_hole(kind):
    """One unmapped-bus access after a control (p36: the combined family lost USB; which access is unknown).

    Run alone with --single-probes so the control's record reaches the host before the access; a lost USB
    after the control is the result (the access hangs the system bus), and the device then needs a power cycle."""
    from . import m1diff_isa as I

    def family(s, t):
        from dataclasses import replace
        from .protocol import STALL_SAFE
        cases = {c.name: c for c in I.bus_error(s, t)}
        control, hole = cases['ldi/control'], cases['%s/hole' % kind]
        safe = lambda c, name: replace(c, name=name, options=dict(c.options, flags=c.options['flags'] | STALL_SAFE),
                                       conditions=dict(c.conditions, stall_safe='CPU reads no FMSS register during the '
                                                       'run; clock gates 5/9 cycled before the first read'))
        yield control
        yield safe(control, 'ldi/control/stall-safe')   # the mode itself on a normal program
        yield safe(hole, '%s/hole/stall-safe' % kind)   # device alive afterwards = the stall is recoverable
        yield hole                                       # ordinary probe: reproduces the p36 loss if it stalls
    family.__doc__ = 'ldi control, then the %s access to 0x%08x' % (kind, I.BUS_HOLE)
    return family


SPECS = {'m2_reboot_ldi_hole': (reboot_hole('ldi'), 'access', 'Control, then ldi from an unmapped address '
                                                              '(reboot test: run alone with --single-probes, then power cycle)'),
         'm2_reboot_dma_hole': (reboot_hole('dma'), 'access', 'Control, then a DMA from an unmapped address '
                                                              '(reboot test: run alone with --single-probes, then power cycle)'),
         'm2_cold_boot': (cold_boot, 'restore,ctrlfx', 'Power-on window, first-ever start, soft-reset comparison '
                                                        '(valid only as the first FMSS work after a power cycle; explore --cold)')}
COLD = ('m2_cold_boot', 'm2_m1_poweron_values')
# Families that end in a power cycle; never part of --all (p36: m2_m1_bus_error lost USB for the rest of the run).
REBOOT_ONLY = ('m2_reboot_ldi_hole', 'm2_reboot_dma_hole', 'm2_m1_bus_error')
