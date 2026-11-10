"""Nano4g monitor numbers and explicit experiment capabilities.

These checks catch accidental misuse of the runner, not hostile microcode.
Raw system-memory stores and unknown instructions are not a sandbox.
"""
from enum import IntEnum
import struct

from nano3g.session import ToolError


class Cmd(IntEnum):
    NAND_STATUS = 120
    NAND_STEP = 121
    NAND_READ_RESERVED = 122
    NAND_READ_PAGE = 123
    NAND_FMSS_PROBE = 137  # 128..136 remain reserved; legacy writes are unsupported
    NAND_FMSS_INFO = 138
    NAND_BATCH_ALLOC = 139
    NAND_BATCH_RUN = 140
    NAND_BATCH_FREE = 141
    NAND_COMPARE = 143  # resident comparison through the shared NAND API
    NAND_CPU = 142  # CPU-direct NAND operation, no sequencer program (target/ipodnano4g/cpu_nand.h)
    NAND_FMSS_NAND = 144  # one page program or one block erase through program.s / erase.s


MUTATE = 1 << 18       # selected experiment may program/erase NAND
BULK_USB = 1 << 22    # RX armed before command ACK; OUT endpoints initially NAK
RESET_PHASES = 1 << 24  # peripheral cleanup and batch v3 reset-phase snapshots
COMMITTED_PROGRESS = 1 << 25  # cache-cleaned batch prefix visible while worker runs
STATE_B_RESET = 1 << 26  # bounded abort / forced sequencer reset for p13 state b
STATE_B_RESET_HELD = 1 << 27  # p14: always reset while held; intermediate status bytes
RECOVERY = 1 << 23    # pause cleanup and batch output v2 diagnostic trailer
BATCH2 = 1 << 21      # compressed resident inputs and on-device dependencies
BATCH = 1 << 20        # resident manifest executor; probe output remains v14
DISCOVERY = 1 << 19    # test unqualified opcodes/register sets
RELOADED = 1 << 29     # capability: kernel entered through EXECFIRMWARE (n4g.py reload)
STALL_SAFE = 1 << 31   # no FMSS access during the run; clock-gate cycle before the first read (no late writes)
STALL_MARK = 0x5afe5afe  # out.dropped of a stall-safe run (it has no polls by design)


def executed(rec):
    """The candidate ran: sampled polls, or a stall-safe run (no polls by design)."""
    return bool(rec.get('polls')) or rec.get('dropped') == STALL_MARK
VICSOFT = 1 << 28      # params may raise/clear VIC soft interrupt lines (CPU-enabled lines masked)
CPU_NAND = 1 << 30     # capability: CPU-direct NAND jobs (Cmd.NAND_CPU)


# Round 6 (target/ipodnano4g/fmss_probe.h): request options whose settings travel as cfg_param() params.
SAMPLER = 1 << 20      # CPU samples up to 8 host-chosen sources every poll into the sample buffer (us timestamps)
GATEWIN = 1 << 21      # one IRQ-off clock-gate window during the run (gates restored exactly)
EIC = 1 << 22          # params may set/clear/write GPIO interrupt controller words 0x39700080..0x397000fc
PADS = 1 << 23         # program runs from a fresh SDRAM region filled with tagged END pads
CAP_SAMPLER = 1 << 12  # capability only: SAMPLER
CAP_STIMULI = 1 << 13  # capability only: GATEWIN, EIC, PADS
CFG_SMP, CFG_GATE, CFG_PADS = 1, 2, 3
EIC_BASE = 0x39700000
EIC_SET, EIC_CLEAR, EIC_WRITE = 0, 1, 2
GATE_MARK = 0xfeed0000  # sample pc tags 1: before, 2: gated, 3: restored
PAD_TAG = 0xfa000000    # pad unit at region offset x: movi r2,#(PAD_TAG | x >> 4); end
PAD_MAX = 0x01100000
PWRCON_WORDS = (0x3c500048, 0x3c50004c, 0x3c500058, 0x3c500068, 0x3c50006c)


def cfg_param(ident, k):
    """Probe parameter offset carrying round-6 setting k of option ident (CFG_SMP/CFG_GATE/CFG_PADS)."""
    return 0x40000000 | ident << 8 | k << 2


def eic_param(op, reg):
    """Probe parameter offset: EIC word 0x39700000 + reg, op EIC_SET / EIC_CLEAR / EIC_WRITE."""
    return 0x90000000 | op << 8 | reg


def is_cfg(off):
    return off & 0xffff0003 == 0x40000000 and 1 <= (off >> 8) & 0xff <= 3


def is_eic(off):
    return off & 0xfffffc03 == 0x90000000 and (off >> 8) & 3 < 3 and off & 0xff >= 0x80


def sampler_source_ok(src, flags=0):
    """Sources the firmware sampler reads (fmss_probe.c smp_source_ok)."""
    return isinstance(src, int) and 0 <= src <= 0xfffffffc and not src & 3


def sampler(sources, period_us=0, from_us=0, changes=False, pc=False):
    """Params for SAMPLER: up to 8 sources, minimum period, first sample time, change-only mode."""
    if not 1 <= len(sources) <= 8:
        raise ToolError("the sampler takes 1..8 sources")
    mode = int(bool(changes)) | int(bool(pc)) << 1
    return ([(cfg_param(CFG_SMP, k), v) for k, v in enumerate(sources)]
            + [(cfg_param(CFG_SMP, 8), period_us), (cfg_param(CFG_SMP, 9), from_us), (cfg_param(CFG_SMP, 10), mode)])


def gate_window(gates, at_us, iterations, word=0xc1c, checkpoint=False):
    """Params for GATEWIN: emCORE gate numbers (32 * PWRCON index + bit) off for `iterations` CPU delay iterations; sample `word` at either end."""
    masks = [0] * 5
    for g in gates:
        if not isinstance(g, int) or not 0 <= g < 160:
            raise ToolError("clock gate index must be 0..159")
        masks[g >> 5] |= 1 << (g & 31)
    return ([(cfg_param(CFG_GATE, k), m) for k, m in enumerate(masks)]
            + [(cfg_param(CFG_GATE, 5), at_us), (cfg_param(CFG_GATE, 6), iterations), (cfg_param(CFG_GATE, 7), word)]
            + ([(cfg_param(CFG_GATE, 8), 1)] if checkpoint else []))


def pads(span):
    return [(cfg_param(CFG_PADS, 0), span)]


def check_round6(code, params, flags):
    """The firmware's probe_config rules (a refused request returns -1 on the device)."""
    cfg = [(o, v) for o, v in params if is_cfg(o)]
    have = {(o >> 8) & 0xff for o, _ in cfg}
    for ident, flag, name in ((CFG_SMP, SAMPLER, 'SAMPLER'), (CFG_GATE, GATEWIN, 'GATEWIN'), (CFG_PADS, PADS, 'PADS')):
        if (ident in have) != bool(flags & flag):
            raise ToolError("%s settings and flag disagree" % name)
    src = [v for o, v in cfg if (o >> 8) & 0xff == CFG_SMP and (o >> 2) & 0x3f < 8]
    if [(o >> 2) & 0x3f for o, _ in cfg if (o >> 8) & 0xff == CFG_SMP and (o >> 2) & 0x3f < 8] != list(range(len(src))):
        raise ToolError("sampler sources must be settings 0..n-1 in order")
    if any((o >> 2) & 0x3f > {CFG_SMP: 10, CFG_GATE: 8, CFG_PADS: 0}[(o >> 8) & 0xff] for o, _ in cfg):
        raise ToolError("unknown round-6 setting")
    if flags & SAMPLER and (not src or flags & (STALL_SAFE | 4) or not all(sampler_source_ok(v, flags) for v in src)):
        raise ToolError("invalid sampler request")
    if flags & GATEWIN:
        g = {(o >> 2) & 0x3f: v for o, v in cfg if (o >> 8) & 0xff == CFG_GATE}
        if (g.get(8, 0) not in (0, 1) or not 0 < g.get(6, 0) <= 0xffffffff or flags & STALL_SAFE
                or not (sampler_source_ok(g.get(7, 0xc1c), flags) and g.get(7, 0xc1c) <= 0xffc)):
            raise ToolError("invalid gate window")
    if flags & PADS:
        span = next(v for o, v in cfg if (o >> 8) & 0xff == CFG_PADS)
        if span < len(code) + 16 or span > PAD_MAX or span & 15 or flags & (1 << 15):
            raise ToolError("invalid pad region")


def vic_param(vic, reg):
    """Probe parameter offset for VIC `vic` SOFTINT (0x18) or SOFTINTCLEAR (0x1c)."""
    return 0x80000000 | vic << 12 | reg


def write_ok(off, value, flags=0):
    if 0x1000 <= off < 0x40000000 and not off & 3:
        return True  # absolute system-memory parameter write
    if flags & VICSOFT and off in {vic_param(v, r) for v in (0, 1) for r in (0x18, 0x1c)}:
        return True
    if is_cfg(off):
        return bool(flags & (SAMPLER | GATEWIN | PADS))
    if is_eic(off):
        return bool(flags & EIC)
    if off < 0 or off > 0xffc or off & 3:
        return False
    return True


def validate_probe(code, params=(), timeout_us=100000, flags=0, late=(), late_us=0,
                   start_value=0, late2=(), late2_us=0, scratch=None, scratch_read=256):
    if not code or len(code) % 8 or len(code) > 0x1fc00:
        raise ToolError("M2 program must contain 8-byte instructions and fit before output")
    if flags & ((1 << 1) | (1 << 16)):
        raise ToolError("M2 has four snapshot windows; M1 channel pin switching is not implemented")
    if flags & (1 << 15) and len(code) > 4096:
        raise ToolError("SRAM program exceeds 4096 bytes")
    if not 0 <= timeout_us <= 2000000:
        raise ToolError("probe timeout must be 0..2000000 us")
    if not 0 <= scratch_read <= 0x8000 or (scratch is not None and len(scratch) > 0x8000):
        raise ToolError("probe scratch access exceeds 32 KiB")
    if flags & STALL_SAFE and (late or late2):
        raise ToolError("stall-safe cases cannot schedule late writes")
    if len(params) + len(late) + len(late2) > 96 or (late2 and not late):
        raise ToolError("invalid late-write batches or too many parameter writes")
    for off, value in (*params, *late, *late2):
        if not write_ok(off, value, flags):
            raise ToolError("M2 parameter write refused: 0x%x = 0x%x" % (off, value))
    if any(is_cfg(o) for o, _ in (*late, *late2)):
        raise ToolError("round-6 settings belong to the parameters before the start")
    check_round6(code, params, flags)
    for word, value in struct.iter_unpack("<II", code):
        op, off = word >> 24, word & 0xffff
        # Field-width probes deliberately set upper offset bits. Apply command
        # checks to the possible 12-bit alias as well; no alias is assumed true.
        checked = off & 0xfff if flags & DISCOVERY else off
        if op in (1, 2) and (checked > 0xffc or checked & 3):
            raise ToolError("M2 program contains a malformed register write")

