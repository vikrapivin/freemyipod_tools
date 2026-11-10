#!/usr/bin/env python3
"""Nano4g image packaging, emCORE M2 experiments and NAND bring-up."""
import sys
from pathlib import Path

# The script directory holds the nano4g and fmss packages. Isolated Python
# does not put that directory on sys.path.
_TOOLS = str(Path(__file__).resolve().parent)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["image"]:
        from nano4g.image import main as command
        return command(argv[1:])
    if argv[:1] == ["reload"]:
        from nano4g.reload import main as command
        return command(argv[1:])
    from nano4g.cli import main as command
    return command(argv)

if __name__ == "__main__":
    raise SystemExit(main())
