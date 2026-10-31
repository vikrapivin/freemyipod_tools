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

"""Controller state: the recorded power-on values and the soft reset."""
from ..probe import (probe, assemble, snapshot, idle_events, power_on_diff, F_WIN400)
from .registry import experiment


@experiment("state", needs=("pld_pst", "win400"))
def ex_state(s, t):
    """Record the state as it is (run first after a power cycle: the power-on values)"""
    st = snapshot(s, t)
    st["idle_events"] = ev = idle_events(s, t)
    r = st["run"]
    d = power_on_diff(t, st)
    return st, "events passing in an idle program: %s (power-on: 11); %d words differ from the recorded power-on " \
        "values%s\n  CS_IRQ after an END-only run %08x, interrupt line %d/%d; FMC 0x044 %08x, ECC 0x844 %08x, ECC_DONE %08x, " \
        "C10 %02x, 0x404 %08x" % (" ".join(map(str, ev)), len(d), "".join("\n  0x%03x = %08x" % kv for kv in sorted(d.items())), r["cs_irq"], r["irq_seen"], r["irq_end"], r["after"][0x044], r["after"][0x844],
                                  r["after"][0x840], r["after"][0xc10], r["after"][0x404])


@experiment("restore", needs=("pld_pst", "win400", "second_fmc_set"))
def ex_restore(s, t):
    """Soft reset instead of a power cycle: every writable FMC / ECC / sequencer word to its power-on value, status
    words acknowledged, the 0x100 block rewritten by the sequencer, parameter RAM zeroed; then every word compared with
    the power-on values (what is left can only be cleared by a power cycle, or by a reset not found yet)"""
    before = snapshot(s, t)
    w1 = probe(s, t, assemble(t, "    end\n"), params=t.restore, timeout_us=20000, quiet=True, flags=F_WIN400)
    w2 = probe(s, t, assemble(t, t.prologue(c58=0) + t.restore_seq() + "    end\n" + t.trap()), params=t.restore_pram, timeout_us=20000,
               quiet=True)
    after = snapshot(s, t)
    ev = idle_events(s, t)
    d0, d1 = power_on_diff(t, before), power_on_diff(t, after)
    return {"before": before, "write": w1, "write_seq": w2, "after": after, "idle_events": ev}, (
        "differ from power-on before: %d words, after: %d words%s\n  CS_IRQ after an END-only run %08x -> %08x, "
        "interrupt line %d -> %d\n  events passing in an idle program afterwards: %s (power-on: 11)" % (
            len(d0), len(d1), "".join(
            "\n  0x%03x = %08x (power-on %08x)" % (o, v, t.power_on.get(o, t.power_on_seq.get(o, 0)) & 0xffffffff)
            for o, v in sorted(d1.items())), before["run"]["cs_irq"], after["run"]["cs_irq"],
            before["run"]["irq_end"], after["run"]["irq_end"], " ".join(map(str, ev))))
