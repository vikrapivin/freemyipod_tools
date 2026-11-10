"""Reload the normal emcore.bin using the nano3g EXECFIRMWARE workflow."""
import argparse
from pathlib import Path
import sys
import time

from nano3g.reload import check_image, reload as reload_kernel
from .nand import make_dev_session, ToolError
from .protocol import RELOADED

BUILD = Path(__file__).resolve().parents[2] / "build/ipodnano4g"


def wait_back(seconds, settle, poll, usb="auto"):
    start = time.monotonic()
    last = None
    while time.monotonic() - start < seconds:
        time.sleep(0.5)
        s = None
        try:
            s = make_dev_session(settle=settle, poll=poll, usb=usb)
            info, status = s.em.getversioninfo(), s.status()
            if not s.probe_caps & RELOADED:
                raise ToolError("device answered, but did not enter through EXECFIRMWARE")
            return s, info, status, time.monotonic() - start
        except Exception as exc:
            last = exc
            if s is not None:
                s.close()
    raise ToolError("emCORE did not return after reload (%s); recover through DFU" % last)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--kernel", type=Path, default=BUILD / "emcore.bin")
    p.add_argument("--wait", type=float, default=20)
    p.add_argument("--settle", type=float, default=0.02)
    p.add_argument("--poll", type=float, default=0.05)
    p.add_argument("--usb", choices=("auto", "bulk", "control"), default="auto")
    a = p.parse_args(argv)
    s = None
    try:
        data = a.kernel.read_bytes()
        check_image(data)
        s = make_dev_session(settle=a.settle, poll=a.poll, usb=a.usb)
        print("PLAN: upload and verify the normal kernel, then EXECFIRMWARE and reconnect", flush=True)
        s, _ = reload_kernel(s, data, verify=True, wait=a.wait,
                             reconnect=lambda: wait_back(a.wait, a.settle, a.poll, a.usb))
        return 0
    except (OSError, ToolError) as exc:
        print("n4g reload: %s" % exc, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            s.close()


if __name__ == "__main__":
    raise SystemExit(main())
