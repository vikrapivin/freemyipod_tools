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


"""FMSS sequencer probe: run small programs on the NAND sequencer and record what the CPU can see.

The device job (monitor command 139) runs one program and records r0..r7 (0xc18..0xc34, r6 = SEQ_RESULT), the PC
(0xc38) and CS_IRQ while it runs (a sample whenever one of them changes, with the time), and the register windows
0x000 / 0x400 / 0x800 / 0xc00 / 0xd00 before and after. Then the sequencer is reset and parked, and the NAND layer
must be initialised again (step 3) before the next NAND job.

  n3g.py fmss run PROG.s [--param OFF=VAL ...] [--timeout-us N] [--json FILE]
  n3g.py fmss explore [--only NAME,...] [--json FILE]      the experiment battery (questions about the ISA)

Safety (defaults; each has a bypass): FMC_CTRL0 is 0 at the start (no chip enable selected) and a program may not
write FMC_CTRL0 or FMC_CMD (--allow-nand), use pst (indirect register store, --allow-indirect) or opcodes without a
known meaning (--allow-unknown). With no chip enable selected nothing reaches the flash. The explore battery uses
pst and unknown opcodes only with registers it set itself, and never selects a chip.

An experiment that needs something the target does not have (pld / pst, a second FMC register set, ...) is skipped
with a note.
"""
import argparse
import importlib
import json
import sys

from nano3g.session import ToolError

from . import probe as P
from .experiments import EXPERIMENTS, ORDER
from .experiments.registry import RESULTS
from .target import NANO3G


def explore(s, only=None, json_path=None, target=NANO3G):
    names = list(ORDER)
    unknown = [n for n in (only or []) if n not in names]
    if unknown:
        raise ToolError("unknown experiment(s) %s; this copy of the tool has: %s" % (", ".join(unknown), ", ".join(names)))
    results = {}
    for ex in EXPERIMENTS:
        name, fn = ex.name, ex.fn
        if only and name not in only:
            continue
        missing = ex.missing(target)
        if missing:
            print("\n### %s: skipped, target %s lacks: %s" % (name, target.name, ", ".join(missing)))
            continue
        P.set_experiment(name)
        print("\n### %s: %s" % (name, fn.__doc__))
        try:
            data, summary = fn(s, target)
        except ToolError as e:
            data, summary = None, "ERROR: %s" % e
        except NotImplementedError as e:               # a Target field or helper still to be filled
            data, summary = None, "SKIPPED: %s" % e
        except Exception as e:                         # noqa: BLE001  a bug in one experiment must not stop the rest
            import traceback
            data, summary = None, "ERROR (tool bug, please report): %r\n%s" % (e, traceback.format_exc())
        print(summary)
        results[name] = RESULTS[name] = {"question": fn.__doc__, "summary": summary, "data": P.jsonable(data)}
        if json_path:                                  # after every experiment: a hang later loses nothing
            with open(json_path, "w") as f:
                json.dump(results, f, indent=1)
        first = data if isinstance(data, dict) and "pre_ok" in data else None
        if first is not None and not first["pre_ok"]:
            print("\nThe engine did not run the END-only preflight program: stopping (nothing else can answer). "
                  "Preflight: CS_IRQ %08x, FMSS_CTRL %08x after %d us." % (first["pre_cs_irq"], first["pre_c00"],
                                                                        first["pre_us"]))
            break
    print("\n(the NAND layer was reset by the probes: run `n3g.py nand step 3` before NAND jobs)")
    if json_path:
        with open(json_path, "w") as f:
            json.dump(results, f, indent=1)
        print("wrote %s" % json_path)
    return results


def parse_param(text):
    o, v = text.split("=", 1)
    return int(o, 0), int(v, 0)


def main(argv=None, target=NANO3G, prog="n3g.py fmss"):
    ap = argparse.ArgumentParser(prog=prog, description=__doc__.split("\n")[0])
    ap.add_argument("--settle", type=float, default=0.02)
    ap.add_argument("--poll", type=float, default=0.05)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run", help="run one program (%s FMSS source) and show what the CPU saw" % (
        "3G" if target is NANO3G else target.name))
    p.add_argument("source")
    p.add_argument("--param", action="append", type=parse_param, default=[], metavar="OFF=VAL",
                   help="register write before the start (FMC offset), e.g. --param 0xd04=4")
    p.add_argument("--timeout-us", type=int, default=100000)
    p.add_argument("--keep-ctrl0", action="store_true", help="do not clear FMC_CTRL0 before the start")
    p.add_argument("--win400", action="store_true", help="also read the 0x400 window")
    p.add_argument("--sample-regs", action="store_true", help="also read r0..r7 while the program runs")
    p.add_argument("--allow-nand", action="store_true", help="allow writes to FMC_CTRL0 / FMC_CMD (can reach the flash)")
    p.add_argument("--allow-indirect", action="store_true", help="allow pst")
    p.add_argument("--allow-unknown", action="store_true", help="allow opcodes without a known meaning")
    p.add_argument("--json")
    p = sub.add_parser("explore", help="run the experiment battery")
    p.add_argument("--only", type=lambda t: t.split(","), default=None, help=",".join(ORDER))
    p.add_argument("--list", action="store_true", help="list the experiments of this copy and exit")
    p.add_argument("--json")
    a = ap.parse_args(argv)
    if a.cmd == "explore" and a.list:
        for ex in EXPERIMENTS:
            missing = ex.missing(target)
            print("%-10s %s%s" % (ex.name, ex.doc.split("\n")[0],
                                  "  [target %s lacks: %s]" % (target.name, ", ".join(missing)) if missing else ""))
        return 0
    s = None
    try:
        s = importlib.import_module(target.host + ".nand").make_dev_session(settle=a.settle, poll=a.poll)
        if a.cmd == "explore":
            explore(s, a.only, a.json, target)
            return 0
        code = P.assemble(target, open(a.source).read())
        P.lint(target, code, a.allow_nand, a.allow_indirect, a.allow_unknown)
        flags = (P.F_KEEP_CTRL0 if a.keep_ctrl0 else 0) | (P.F_WIN400 if a.win400 else 0) | \
            (P.F_SAMPLE_REGS if a.sample_regs else 0)
        rec = P.probe(s, target, code, a.param, a.timeout_us, flags)
        print(P.describe(rec))
        if a.json:
            with open(a.json, "w") as f:
                json.dump(P.jsonable(rec), f, indent=1)
        return 0
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()
