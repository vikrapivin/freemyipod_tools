"""Resident execution for the existing opcode sweep name."""
from dataclasses import replace
from fmss.experiments.sweeps import opsweep_cases, opsweep_summary, sweep_seeds, scratch_pattern
from .followup_experiments import Case, run_cases
from .protocol import DISCOVERY


def opsweep(s, t):
    t = replace(t, asm_flags=("-3",), probe_flags=t.probe_flags | DISCOVERY)
    context = {}
    def generate(view, target):
        seeds, pat = sweep_seeds(view, target), scratch_pattern(target)
        context.update(seeds=seeds, pat=pat)
        for key, src, options in opsweep_cases(target, seeds, pat):
            yield Case(key, src, options)
    records, transport = run_cases("opsweep", s, t, generate)
    records, report = opsweep_summary(records, t, context['seeds'], context['pat'])
    return records, transport + "\n" + report
