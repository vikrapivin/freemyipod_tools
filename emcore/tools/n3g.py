#!/usr/bin/env python3
"""Nano3g build and device tools (NAND worker, benchmark, FMSS probe, boot flash)."""
import argparse
import importlib
import sys

COMMANDS = {'image': 'image', 'nand': 'nand', 'bench': 'bench', 'collect': 'collect', 'write-test': 'writetest', 'reload': 'reload', 'nor': 'nor'}

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=tuple(COMMANDS))
    argv = sys.argv[1:] if argv is None else argv
    args = parser.parse_args(argv[:1])
    rest = argv[1:]
    return importlib.import_module("nano3g." + COMMANDS[args.command]).main(rest)

if __name__ == "__main__":
    raise SystemExit(main())
