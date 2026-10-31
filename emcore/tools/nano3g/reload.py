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

"""Replace the running emCORE with a new build, without going through DFU again.

  n3g.py reload [--kernel build/ipodnano3g/emcore.bin] [--no-verify] [--wait S]

The kernel image (emcore.bin, linked at 0x08000000) is written to the NAND result buffer, read back and compared
(--no-verify skips that), then the monitor's EXECFIRMWARE (command 24) shuts emCORE down (user threads, USB,
interrupts), copies the image to 0x08000000 with MMU and caches off and jumps to it: the kernel starts exactly as
stage 1 starts it at boot. Clocks, SDRAM and the translation table from the first boot stay in place; stage 1 does not
run again. The new emCORE re-enumerates on USB; the tool waits for it to answer.

Use it on an idle emCORE (no NAND job running). If the new kernel does not come back, restart the iPod into DFU and
load the image the normal way; nothing is written to the flash by a reload.
"""
import argparse
import os
import struct
import sys
import time

from . import nand
from .nand import ToolError

KERNEL_BASE = 0x08000000


def default_kernel():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "build",
                                         "ipodnano3g", "emcore.bin"))


def check_image(img):
    if len(img) < 0x100 or len(img) & 3:
        raise ToolError("kernel image of %d bytes: not an emcore.bin" % len(img))
    w0 = struct.unpack_from("<I", img, 0)[0]
    if w0 >> 24 != 0xea:
        raise ToolError("first word %08x is not a branch: not an emcore.bin linked at 0x%08x" % (w0, KERNEL_BASE))


def wait_back(seconds, settle, poll):
    """Reconnect until the monitor and the NAND status answer"""
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < seconds:
        time.sleep(0.5)
        try:
            s = nand.make_dev_session(settle=settle, poll=poll)
        except Exception as e:                     # noqa: BLE001  not enumerated yet
            last = e
            continue
        try:
            info = s.em.getversioninfo()
            st = s.status()
            return s, info, st, time.monotonic() - t0
        except Exception as e:                     # noqa: BLE001
            last = e
            s.close()
    raise ToolError("emCORE did not answer within %.0f s after the jump (%s). Restart the iPod into DFU and load the "
                    "image the normal way." % (seconds, last))


def reload(s, img, verify=True, wait=20.0, settle=0.02, poll=0.05, execfw=None, reconnect=None):
    """execfw(dest, src, size) sends EXECFIRMWARE; reconnect() returns (session, info, status, seconds) once the new
    emCORE answers. Defaults: libemcore over USB."""
    check_image(img)
    st = s.status()
    if st.busy:
        raise ToolError("a NAND job is running: wait for it, then reload")
    if len(img) > st.result_cap:
        raise ToolError("kernel of %d bytes does not fit the %d-byte transfer buffer" % (len(img), st.result_cap))
    src = st.result_addr
    print("uploading %d bytes to 0x%08x" % (len(img), src))
    s.write_memory(src, img)
    if verify:
        back = s.read(src, len(img))
        if back != img:
            bad = next(i for i in range(len(img)) if back[i] != img[i])
            raise ToolError("read-back differs at byte 0x%x: nothing was started" % bad)
        print("read back: identical")
    print("EXECFIRMWARE: copy to 0x%08x and jump (emCORE disconnects now)" % KERNEL_BASE)
    try:
        if execfw:
            execfw(KERNEL_BASE, src, len(img))
        else:
            s.em.execfirmware(KERNEL_BASE, src, len(img), timeout=2000)
    except Exception:                              # noqa: BLE001  no reply is expected
        pass
    try:
        s.close()
    except Exception:                              # noqa: BLE001  the device is gone
        pass
    s2, info, st2, dt = reconnect() if reconnect else wait_back(wait, settle, poll)
    print("emCORE answered again after %.1f s; NAND status record v%d, result buffer 0x%08x" % (
        dt, st2.version, st2.result_addr))
    return s2, info


def main(argv=None):
    ap = argparse.ArgumentParser(prog="n3g.py reload", description=__doc__.split("\n")[0])
    ap.add_argument("--kernel", default=default_kernel(), help="emcore.bin of the new build (default: %(default)s)")
    ap.add_argument("--no-verify", action="store_true", help="skip reading the uploaded image back")
    ap.add_argument("--wait", type=float, default=20.0, help="seconds to wait for the new emCORE (default 20)")
    ap.add_argument("--settle", type=float, default=0.02)
    ap.add_argument("--poll", type=float, default=0.05)
    a = ap.parse_args(argv)
    s = None
    try:
        with open(a.kernel, "rb") as f:
            img = f.read()
        print("kernel %s: %d bytes" % (a.kernel, len(img)))
        s = nand.make_dev_session(settle=a.settle, poll=a.poll)
        s, _ = reload(s, img, not a.no_verify, a.wait, a.settle, a.poll)
        return 0
    except (ToolError, OSError) as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:                      # noqa: BLE001
                pass


if __name__ == "__main__":
    sys.exit(main())
