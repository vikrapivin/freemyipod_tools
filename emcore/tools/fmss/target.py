# Copyright (C) 2026 Vikotr Krapivin
#
# This file is part of emCORE.
#
# emCORE is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# emCORE is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with emCORE. If not, see <http://www.gnu.org/licenses/>.


"""Target profiles of the FMSS probe: everything the experiments take from the model they run on.

A Target holds the assembler mode, the probe job's layout, the register windows, the recorded power-on values, the NAND
set-up sequences (as methods: they emit FMSS source for the model's controller) and capability flags. The
experiments in fmss/experiments are written against a Target; one that needs a capability the Target lacks is skipped.
NANO3G is complete. NANO4G and NANO5G are skeletons: fields set to None are "to be measured / filled" and the
helpers that need them raise NotImplementedError.
"""
from dataclasses import dataclass, field
from typing import Optional

# ---- nano 3G tables (CPU view of one unit right after a power cycle and emCORE's start)
# Non-zero words (every other word of the 0x000, 0x400, 0x800 and 0xc00..0xc5c windows and all parameter RAM read 0).
# Counters and run state are left out (FMC_28, its 0x428 twin, C04, C14, C38).
_3G_POWER_ON = {0x020: 0x7f167f, 0x038: 0x7, 0x048: 0xc3000, 0x04c: 0xff, 0x420: 0x7f167f, 0x438: 0x7, 0x448: 0xc3000,
                0x44c: 0x1, 0x800: 0x20060808, 0x814: 0xe00000, 0x840: 0x10, 0xc50: 0xffff1800}
_3G_POWER_ON_SEQ = {0x1bc: ~0, 0x1c0: ~0, 0x1c4: ~0, 0x1c8: ~0, 0x1cc: 0x8060, 0x1d0: 0x3010d0, 0x1d4: ~0, 0x1d8: ~0,
                    0x9c0: 1}                             # sequencer-only view (pld): block 0x100 and 0x9c0
_3G_POWER_ON_SEQ.update({o: ~0 for o in range(0x1dc, 0x200, 4)})
_3G_RW_WORDS = ([0x000, 0x00c, 0x010, 0x014, 0x018, 0x020, 0x024, 0x02c, 0x030, 0x034, 0x044] +
                list(range(0x060, 0x07c, 4)) + [0x804, 0x808, 0x814, 0x844, 0xc10, 0xc3c, 0xc48, 0xc54])
_3G_RW_WORDS += [0x400 + o for o in _3G_RW_WORDS if o < 0x080] + [0x404, 0x438, 0x47c]
# writable words to their power-on values, write-1-to-clear status words acknowledged (FMC_STATUS of both sets,
# ECC_DONE; 0x050 / 0x450 tried the same way), the unknown ECC-area words a soft reset left set, parameter RAM zeroed
_3G_RESTORE = [(o, _3G_POWER_ON.get(o, 0)) for o in _3G_RW_WORDS] + \
    [(0x048, 0xffffffff), (0x448, 0xffffffff), (0x840, 0x1ff), (0x050, 0xffffffff), (0x450, 0xffffffff),
     (0x980, 0), (0x984, 0), (0x9a4, 0)]
_3G_RESTORE_PRAM = [(0xd00 + 4 * i, 0) for i in range(32)]
# opcodes with a known meaning (opsweep leaves these out)
_3G_KNOWN_OPS = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x11, 0x13, 0x14, 0x17,
                 0x18, 0x19}
# CPU-written words that survive a sequencer reset and a kernel reload; with C58 = 1 a start begins at the fault vector
# (FMSS_C4C) instead of 0 (device: after a run that left C58 = 1 every later program lost its first instructions)
_3G_CLEAN_PARAMS = [(0xc58, 0), (0xc4c, 0), (0xc5c, 0)]
_3G_NOT_READBACK = {0xc0c, 0x048, 0x840}                  # write-1-to-clear: a write does not read back
# registers whose write starts something (a transfer, a command, the ECC engine, the sequencer) or pops a FIFO
_3G_RW_SKIP = {0x004, 0x008, 0x038, 0x07c, 0x080, 0x80c, 0xc00}
# the sequencer control window other than the start word, CS_IRQ, PC and r0..r7
_3G_CTRL_SKIP = {0xc00, 0xc04, 0xc0c, 0xc38} | set(range(0xc18, 0xc38, 4))
# words the power-on diff leaves out (counters, run state)
_3G_DIFF_SKIP = {0x028, 0x428, 0xc00, 0xc04, 0xc0c, 0xc14, 0xc38} | set(range(0xc18, 0xc38, 4))
# FMC register names (assembler symbols) moved to the second FMC register set by ch_regs
_3G_FMC_NAMES = {"FMC_CTRL0": 0x000, "FMCTRL1": 0x004, "FMC_CMD": 0x008, "FMC_ADDR0": 0x00c, "FMC_ADDR1": 0x010,
                 "FMC_DATA_SEL": 0x014, "FMC_ANUM": 0x02c, "FMC_DNUM": 0x030, "FMC_STATUS": 0x048, "FMC_NSTAT": 0x04c,
                 "FMC_FIFO": 0x080, "FMC_DMA_ADDR": 0x034}
# WAIT event numbers (3G: 0..15 are the controller's flags; 11 is set at idle)
_3G_EVENTS = {"cmd": 0, "addr": 1, "dma": 2, "data": 3, "ecc": 4, "status": 5, "set1_cmd": 6, "set1_dma": 8, "idle": 11}


@dataclass
class Target:
    name: str
    asm_flags: tuple = ()                 # fmssasm mode flags: ("-3",) for the nano 3G, none for the 4G/5G names
    host: str = "nano3g"                  # package of the host transport library (session, nand job checks, Cmd)
    cmd_step: Optional[str] = None        # monitor command names (protocol.Cmd members) of the NAND step / probe jobs
    cmd_probe: Optional[str] = None
    step_ok_bit: Optional[int] = None     # NAND layer step that must have run before a probe
    # probe job layout (device side: emCORE's fmss probe job)
    probe_prog: Optional[int] = None      # offsets inside the job's result buffer
    probe_out: Optional[int] = None
    probe_scratch: Optional[int] = None
    scratch_bytes: Optional[int] = None
    req_magic: Optional[int] = None
    out_magic: Optional[int] = None
    req_version: Optional[int] = None     # request header version
    need_version: Optional[int] = None    # probe version the running emCORE must report
    max_params: Optional[int] = None
    max_samples: Optional[int] = None
    head_fmt: Optional[str] = None        # output header and sample record layouts
    head_bytes: Optional[int] = None
    sample_fmt: Optional[str] = None
    windows: tuple = ()                   # register windows the probe snapshots (offsets from the FMC base)
    # register knowledge
    known_ops: frozenset = frozenset()
    nand_regs: dict = field(default_factory=dict)     # registers a probe may not write without --allow-nand
    clean_params: Optional[list] = None
    not_readback: frozenset = frozenset()
    ctrl_skip: frozenset = frozenset()
    diff_skip: frozenset = frozenset()
    rw_skip: frozenset = frozenset()
    fmc_names: Optional[dict] = None
    power_on: Optional[dict] = None
    power_on_seq: Optional[dict] = None
    rw_words: Optional[list] = None
    restore: Optional[list] = None
    restore_pram: Optional[list] = None
    # NAND set-up
    ce_sel: Optional[dict] = None         # FMC_CTRL0 word per chip enable (None: no chip)
    ce_base: Optional[int] = None         # word with no chip enable; chip k adds 1 << (k + 1)
    ce_dma: Optional[int] = None          # bit that lets a DMA run (FMC_CTRL0 b24)
    fmc_init: Optional[tuple] = None      # (FMCTRL1, FMC_STATUS) values a NAND sequence starts with
    scratch_block: Optional[int] = None   # block of CE0 the destructive experiments erase
    rows_per_block: Optional[int] = None
    events: Optional[dict] = None
    # capabilities
    has_pld_pst: bool = False             # pld / pst (indirect register access) exist
    has_second_fmc_set: bool = False      # a second FMC register set at +0x400
    has_ch1_pins: bool = False            # the probe job can switch FMC channel 1's pins (flag CH1_PINS)
    has_fullmap: bool = False             # the probe job can dump the whole window (flag FULLMAP)
    has_win400: bool = False              # the probe job can snapshot the +0x400 window
    has_sram_prog: bool = False           # the job can run the program from internal SRAM
    has_start_value: bool = False         # the FMSS_CTRL start word can be chosen
    has_nand_seq: bool = False            # the NAND set-up sequences below are defined (CE words, FMC_CTRL0 layout)

    # ---- guards
    def need(self, *fields):
        for f in fields:
            if getattr(self, f) is None:
                raise NotImplementedError("target %s: %s is to be measured / filled" % (self.name, f))

    def has(self, cap):
        return bool(getattr(self, "has_" + cap))

    # ---- derived values
    @property
    def scratch_row(self):
        self.need("scratch_block", "rows_per_block")
        return self.scratch_block * self.rows_per_block

    def ce_word(self, k):
        """FMC_CTRL0 word selecting chip enable k"""
        self.need("ce_base")
        return self.ce_base | 1 << (k + 1)

    def ev(self, name):
        self.need("events")
        return self.events[name]

    def expand(self, text):
        """Replace the target tokens of a source text: the chip-select words and the FMC set-up values"""
        if "@" not in text:
            return text
        self.need("ce_sel", "ce_dma", "fmc_init")
        for tok, val in (("@CE_NONE@", self.ce_sel[None]), ("@CE0@", self.ce_sel[0]),
                         ("@CE_DMA@", self.ce_sel[None] | self.ce_dma), ("@CE0_DMA@", self.ce_sel[0] | self.ce_dma)):
            text = text.replace(tok, "0x%x" % val)
        return text.replace("@FMCTRL1_INIT@", "0x%08x" % self.fmc_init[0]).replace("@STATUS_CLEAR@",
                                                                                      "0x%08x" % self.fmc_init[1])

    # ---- FMSS source helpers
    def trap(self):
        return "\ntrap:\n    movi r6, #0xfa17fa17\n    end\n"

    def prologue(self, c58=4):
        """C58 is written every time: it keeps its value across resets (device). 4 = a timeout vectors to the fault
        handler, as in the original programs"""
        return "    wr   [CS_IRQ], #0xff\n    wr   [FMSS_C4C], #trap\n    wr   [FMSS_C58], #0x%x\n" % c58

    def nand_prologue(self, ce=0):
        self.need("ce_sel", "fmc_init")
        return (self.prologue(c58=0) + "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"
                "    wr   [FMC_DATA_SEL], #0x1\n    wr   [FMC_CTRL0], #0x%x\n" % self.ce_sel[ce])

    def cmd(self, c):
        return "    wr   [FMC_CMD], #0x%x\n    wait %d\n    wr   [FMC_STATUS], #0x2\n" % (c, self.ev("cmd"))

    @property
    def erase(self):
        """block erase of the scratch block on CE0 (after nand_prologue)"""
        return (self.cmd(0x60) + "    wr   [FMC_ANUM], #0x2\n    wr   [FMC_ADDR0], #0x%x\n    wr   [FMCTRL1], #0x1\n"
                "    wait %d\n    wr   [FMC_STATUS], #0x4\n" % (self.scratch_row, self.ev("addr")) + self.cmd(0xd0))

    def ch_regs(self, text, base):
        """FMC register names of nand_prologue-style source moved to the FMC register set at +base (0 or 0x400)"""
        self.need("fmc_names")
        for n, o in sorted(self.fmc_names.items(), key=lambda x: -len(x[0])):
            text = text.replace("[%s]" % n, "[0x%x]" % (base + o))
        return text

    def read_id(self, ce_word, base):
        self.need("fmc_init")
        e = self.events
        src = ("    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n    wr   [FMC_DATA_SEL], #0x1\n"
               "    wr   [FMC_CTRL0], #0x%x\n" % ce_word +
               "    wr   [FMC_CMD], #0x90\n    wait %d\n    wr   [FMC_STATUS], #0x2\n"
               "    wr   [FMC_ANUM], #0x0\n    wr   [FMC_ADDR0], #0x0\n    wr   [FMCTRL1], #0x1\n    wait %d\n"
               "    wr   [FMC_STATUS], #0x4\n    wr   [FMC_DNUM], #0x7\n    wr   [FMC_DATA_SEL], #0x1\n"
               "    wr   [FMCTRL1], #0x1c2\n    wait %d\n    wr   [FMC_STATUS], #0x8\n    wr   [FMC_DATA_SEL], #0x100\n"
               "    wr   [FMCTRL1], #0x340\n    ld   r2, [FMC_FIFO]\n    ld   r3, [FMC_FIFO]\n    ld   r4, [FMC_STATUS]\n"
               "    wr   [FMC_CTRL0], #@CE_NONE@\n" % (e["cmd"], e["addr"], e["data"]))
        return self.ch_regs(src, base)

    def scratch_addr(self, s):
        self.need("probe_scratch")
        return s.status().result_addr + self.probe_scratch

    def page_read_steps(self, s):
        """(name, cumulative source) after each step of a one-sector page read of CE0"""
        a, e, row = self.scratch_addr(s), self.events, self.scratch_row
        steps = [("address sent", self.cmd(0x00) + "    wr   [FMC_ANUM], #0x4\n    wr   [FMC_ADDR0], #0x%x\n"
                  "    wr   [FMC_ADDR1], #0x%x\n    wr   [FMCTRL1], #0x1\n" % ((row & 0xffff) << 16, row >> 16)),
                 ("30h sent (tR running)", "    wait %d\n    wr   [FMC_STATUS], #0x4\n    wr   [FMC_CMD], #0x30\n" % e["addr"]),
                 ("ready, data mode", "    wait %d\n    wr   [FMC_STATUS], #0x2\n" % e["cmd"] + self.cmd(0x70) +
                  "    wr   [FMCTRL1], #0xca\n    wait %d\n    wr   [FMC_STATUS], #0x800000\n    wr   [FMCTRL1], #0x20\n"
                  "    wr   [FMCTRL1], #0x100e0\n" % e["status"] + self.cmd(0x00)),
                 ("parity read started", "    wr   [FMC_DNUM], #0xf\n    wr   [FMC_DATA_SEL], #0x10\n    wr   [FMC_24], #0x0\n"
                  "    wr   [FMCTRL1], #0x32\n"),
                 ("data read started", "    wait %d\n    wr   [FMC_STATUS], #0x8\n    wr   [FMC_DNUM], #0x1ff\n"
                  "    wr   [FMC_DATA_SEL], #0x1\n    wr   [FMCTRL1], #0xe2\n" % e["data"]),
                 ("ECC decode started", "    wait %d\n    wr   [FMC_STATUS], #0x8\n    wr   [ECC_DONE], #0x1ff\n"
                  "    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1101\n" % e["data"]),
                 ("DMA started", "    wait %d\n    wr   [ECC_DONE], #0x1\n    wr   [FMC_DATA_SEL], #0x100\n"
                  "    wr   [FMC_CTRL0], #@CE0_DMA@\n"
                  "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x100\n" % (e["ecc"], a)),
                 ("all done", "    wait %d\n    wr   [FMC_STATUS], #0x100000\n" % e["dma"])]
        acc, outl = self.nand_prologue(0), []
        for name, body in steps:
            acc += body
            outl.append((name, acc))
        return outl

    def state_setup(self):
        return "    wr   [FMCTRL1], #@FMCTRL1_INIT@\n    wr   [FMC_STATUS], #@STATUS_CLEAR@\n"

    def state_actions(self, s):
        """(label, source) that leave exactly one kind of flag pending (after a soft reset)"""
        a, e = self.scratch_addr(s), self.events
        cmdp = "    wr   [FMC_CTRL0], #@CE0@\n    wr   [FMC_CMD], #0x70\n    wait %d\n" % e["cmd"]
        dma = ("    wr   [FMC_CTRL0], #@CE_DMA@\n    wr   [FMC_DNUM], #0x1ff\n    wr   [FMC_DATA_SEL], #0x1\n"
               "    wr   [FMC_DMA_ADDR], #0x%x\n    wr   [FMCTRL1], #0x2e0\n    wait %d\n    wr   [FMC_CTRL0], #@CE_NONE@\n" % (a, e["dma"]))
        ecc = "    wr   [ECC_DONE], #0x1ff\n    wr   [ECC_MODE], #0x180\n    wr   [ECC_KICK], #0x1101\n    wait %d\n" % e["ecc"]
        stat = "    wr   [FMC_CTRL0], #@CE0@\n" + self.cmd(0x70) + "    wr   [FMCTRL1], #0xca\n    wait %d\n" % e["status"]
        s1cmd = ("    wr   [0x404], #@FMCTRL1_INIT@\n    wr   [0x448], #@STATUS_CLEAR@\n    wr   [0x400], #@CE0@\n"
                 "    wr   [0x408], #0x70\n    wait %d\n" % e["set1_cmd"])
        s1dma = ("    wr   [0x404], #@FMCTRL1_INIT@\n    wr   [0x448], #@STATUS_CLEAR@\n    wr   [0x400], #@CE_DMA@\n"
                 "    wr   [0x430], #0x1ff\n    wr   [0x414], #0x1\n    wr   [0x434], #0x%x\n    wr   [0x404], #0x2e0\n"
                 "    wait %d\n    wr   [0x400], #@CE_NONE@\n" % (a, e["set1_dma"]))
        return [("nothing pending", ""), ("command flag (b1)", cmdp), ("DMA flag (b20)", dma), ("ECC_DONE b0", ecc),
                ("status flag (b23)", stat), ("all four", cmdp + dma + ecc + stat.replace(self.cmd(0x70), self.cmd(0x70))),
                ("second set: command flag", s1cmd), ("second set: DMA flag", s1dma)]

    def restore_seq(self):
        """source that rewrites the sequencer-only words of the 0x100 block to their power-on values"""
        self.need("power_on_seq")
        return "".join("    wr   [0x%03x], #0x%x\n" % (o, v & 0xffffffff) for o, v in sorted(self.power_on_seq.items())
                       if 0x100 <= o < 0x200)


NANO3G = Target(
    name="nano3g", asm_flags=("-3",), host="nano3g", cmd_step="NAND_STEP", cmd_probe="NAND_FMSS_PROBE", step_ok_bit=1,
    probe_prog=0x400, probe_out=0x20000, probe_scratch=0x38000, scratch_bytes=0x8000,
    req_magic=0x51525046, out_magic=0x4f525046, req_version=4, need_version=12, max_params=96, max_samples=1024,
    head_fmt="<22I", head_bytes=88, sample_fmt="<10I", windows=(0x000, 0x400, 0x800, 0xc00, 0xd00),
    known_ops=frozenset(_3G_KNOWN_OPS), nand_regs={0x000: "FMC_CTRL0", 0x008: "FMC_CMD"},
    clean_params=_3G_CLEAN_PARAMS, not_readback=frozenset(_3G_NOT_READBACK), ctrl_skip=frozenset(_3G_CTRL_SKIP),
    diff_skip=frozenset(_3G_DIFF_SKIP), rw_skip=frozenset(_3G_RW_SKIP), fmc_names=_3G_FMC_NAMES,
    power_on=_3G_POWER_ON, power_on_seq=_3G_POWER_ON_SEQ, rw_words=_3G_RW_WORDS, restore=_3G_RESTORE,
    restore_pram=_3G_RESTORE_PRAM,
    ce_sel={0: 0x43803, 1: 0x43805, None: 0x43801},       # FMC_CTRL0: 0x801 | safe timing (4, 3) | CE bit
    ce_base=0x43801, ce_dma=0x1000000, fmc_init=(0x0ff3f8e0, 0x0ff00ffe), scratch_block=3000, rows_per_block=128,
    events=_3G_EVENTS,
    has_pld_pst=True, has_second_fmc_set=True, has_ch1_pins=True, has_fullmap=True, has_win400=True, has_sram_prog=True,
    has_start_value=True,
    has_nand_seq=True)

# The 4G (S5L8720) and 5G (S5L8730) FMSS engines: same instruction encoding and assembler, no pld / pst, ECC_RESULT at
# 0x80c; the FMC, ECC, sequencer and parameter RAM windows sit at +0x000, +0x800, +0xc00 and +0xd00 of the FMC base
# (0x38a00000); the sequencer start word is 0xffb5 on the 4G. The device-side probe job does not exist there yet:
# the layout, the power-on tables and the NAND sequences are to be measured / filled.
NANO4G = Target(
    name="nano4g", asm_flags=(), windows=(0x000, 0x800, 0xc00, 0xd00),
    nand_regs={0x000: "FMC_CTRL0", 0x008: "FMC_CMD"},
    has_pld_pst=False, has_second_fmc_set=False, has_win400=False, has_sram_prog=False, has_start_value=False,
    has_nand_seq=False)

NANO5G = Target(
    name="nano5g", asm_flags=(), windows=(0x000, 0x800, 0xc00, 0xd00),
    nand_regs={0x000: "FMC_CTRL0", 0x008: "FMC_CMD"},
    has_pld_pst=False, has_second_fmc_set=False, has_win400=False, has_sram_prog=False, has_start_value=False,
    has_nand_seq=False)

TARGETS = {t.name: t for t in (NANO3G, NANO4G, NANO5G)}
