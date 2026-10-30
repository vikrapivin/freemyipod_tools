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


"""FMSS sequencer probe for the nano 3G (`n3g.py fmss`): the target-independent tool lives in tools/fmss; this module
binds it to the NANO3G profile and keeps the names the tool had before it was split."""
import sys

from fmss import cli as _cli
from fmss import probe as _p
from fmss.experiments import EXPERIMENTS as _EXPERIMENTS
from fmss.experiments.registry import RESULTS as _RESULTS   # noqa: F401
from fmss.probe import (F_KEEP_CTRL0, F_SAMPLE_REGS, F_WIN400, PROG_SRAM, SNAP_NONE, SNAP_SKIP, S_END_NOCLEAN,  # noqa: F401
                        S_END_RESET, S_NO_400, S_NO_CLEAN, S_NO_PREFLIGHT, S_NO_RESET, WAIT_END, WHY, answer, changes,
                        describe, jsonable)
from fmss.target import NANO3G as T
from .session import ToolError                                   # noqa: F401

PROBE_PROG, PROBE_OUT, PROBE_SCRATCH, SCRATCH_BYTES = T.probe_prog, T.probe_out, T.probe_scratch, T.scratch_bytes
REQ_MAGIC, OUT_MAGIC, MAXPARAMS, MAXSAMPLES, NEED_VERSION = T.req_magic, T.out_magic, T.max_params, T.max_samples, \
    T.need_version
WINDOWS, HEAD, HEAD_BYTES, SAMPLE = T.windows, T.head_fmt, T.head_bytes, T.sample_fmt
KNOWN_OPS, NAND_REGS, CLEAN_PARAMS, NOT_READBACK = T.known_ops, T.nand_regs, T.clean_params, T.not_readback
CE_SEL, POWER_ON, POWER_ON_SEQ, RESTORE, RESTORE_PRAM = T.ce_sel, T.power_on, T.power_on_seq, T.restore, T.restore_pram
EXPERIMENTS = [(ex.name, ex.bind(T)) for ex in _EXPERIMENTS]


def assemble(text):
    """3G FMSS source text -> program bytes"""
    return _p.assemble(T, text)


def lint(code, allow_nand=False, allow_indirect=False, allow_unknown=False):
    return _p.lint(T, code, allow_nand, allow_indirect, allow_unknown)


def request(code, *a, **kw):
    return _p.request(T, code, *a, **kw)


def parse_out(head_and_windows, samples_raw):
    return _p.parse_out(T, head_and_windows, samples_raw)


def probe(s, code, *a, **kw):
    return _p.probe(s, T, code, *a, **kw)


def scratch_addr(s):
    return T.scratch_addr(s)


def explore(s, only=None, json_path=None):
    return _cli.explore(s, only, json_path, T)


def main(argv=None):
    return _cli.main(argv, T)


if __name__ == "__main__":
    sys.exit(main())
