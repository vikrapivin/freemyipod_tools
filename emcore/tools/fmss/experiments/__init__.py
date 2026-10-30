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

"""The experiment battery. ORDER is the order explore runs them in (it matters: later ones read earlier results and the
device state the earlier ones leave behind)."""
from . import isa, control, events, nand, sweeps, state, access, startword, fields, followup, probes2, probes3, probes4          # noqa: F401  (registering the experiments)
from .registry import REGISTRY, RESULTS, Experiment            # noqa: F401

ORDER = ("state", "restore", "regmap", "clock", "pram", "alu", "memory", "indirect", "resume", "control", "wait",
         "c58", "op15", "op12", "c58ill", "startseq", "startseq2", "startseq3", "oneshot", "c50", "holdout",
         "op12wait", "ev11", "ev5", "c50x", "pending", "c14", "op12ack", "op10", "unknown", "opsweep", "optime",
         "ctrlbits", "fmcmap", "evsweep", "rwmask", "op0f", "ctrlhold", "evclear", "timeregs", "ev5rule", "c08",
         "chan1", "readev", "regnum", "branch", "waitf", "opcost", "op12rel", "pause", "fmc28", "blk100", "irqline",
         "irqen", "ev5t", "r8", "ev8", "evcons", "dmaloop", "op0916", "op16", "opstate", "op16b", "op09", "op09b", "op09c",
         "chan1pins", "pcwrite", "op12cont", "ctrlfx", "op09d", "fwidth", "irqsrc", "eccpar", "bufsize", "eccstr", "fields0", "tmofield", "eccstr2", "ev5en", "tmobits", "eccmode", "eccchunk", "rawpage", "rawwrite", "writeseq", "op09nand", "eraseseq", "eccchunk2", "eccmode2", "tmobits2", "ev5ready", "pageorder", "op09w33", "ev5arm", "writeseq2", "eccmode3", "ev5two", "b15ecc", "eccmode4", "finale", "startdiff", "startbits", "rsdecode", "vic20", "fmc28b",
         "seqfirst", "w2state", "c50b8",
         "c58b1", "evsweep2", "c50fault", "ctrlwords", "c08self", "latchrep", "irq044", "fmcbits", "fmcstat",
         "eccregs", "seqrestart", "done5", "progtime", "startregion", "bufport", "c58b2", "stprev", "tmwait2", "done5b", "rsresult",
         "rschip", "miscorr", "fullpage", "dmactl", "fmcstat2", "ch1buf", "evsweep3", "blk100b", "straps", "irqop", "w2vic", "regsweep",
         "access")                      # access last: it writes every word of the window

if sorted(ORDER) != sorted(REGISTRY):
    raise ImportError("experiments registered but not ordered: %s; ordered but missing: %s" % (
        sorted(set(REGISTRY) - set(ORDER)), sorted(set(ORDER) - set(REGISTRY))))
EXPERIMENTS = [REGISTRY[n] for n in ORDER]
