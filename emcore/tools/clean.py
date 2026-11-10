#!/usr/bin/env python3
"""Remove generated emCORE build products and host Python bytecode caches."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def main():
    build = ROOT / "build"
    if build.is_symlink():
        build.unlink()
    elif build.exists():
        shutil.rmtree(build)
    for cache in (ROOT / "tools").rglob("__pycache__"):
        if cache.is_symlink():
            cache.unlink()
        elif cache.is_dir():
            shutil.rmtree(cache)


if __name__ == "__main__":
    main()
