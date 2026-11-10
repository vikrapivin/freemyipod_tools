#!/usr/bin/env python3
"""Modern py3 Apple DFU sender for iPod Nano 4G (bootrom DFU PID 0x1225).

Sends a (pwnage-signed) IMG1/DFU image via standard DFU protocol.
A valid pwnage 2.0 image is accepted by the bootrom normally -> NO exploit needed.
"""
import sys, struct, time
import usb.core, usb.util

APPLE_VID = 0x05AC
N4G_DFU_PID = 0x1225
TMO = 3000

def crc32(data):
    c = 0xFFFFFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if (c & 1) else 0)
    return c & 0xFFFFFFFF

def find_dfu():
    return usb.core.find(idVendor=APPLE_VID, idProduct=N4G_DFU_PID)

def send(dev, image):
    if dev.is_kernel_driver_active(0):
        try: dev.detach_kernel_driver(0)
        except Exception: pass
    try: dev.set_configuration()
    except Exception: pass
    try: usb.util.claim_interface(dev, 0)
    except Exception: pass

    data = image + struct.pack("<I", crc32(image))
    n = (len(data) + 2047) // 2048
    print("[*] Sending %d bytes in %d x 2048 chunks" % (len(data), n))
    for i in range(n):
        chunk = data[2048 * i: 2048 * (i + 1)]
        dev.ctrl_transfer(0x21, 1, i, 0, chunk, TMO)
        deadline = time.monotonic() + 10
        while True:
            if time.monotonic() >= deadline:
                raise TimeoutError("DFU download-status deadline")
            st = dev.ctrl_transfer(0xa1, 3, 0, 0, 6, TMO)
            if st[0]:
                raise RuntimeError("DFU error status %d" % st[0])
            if st[4] == 0x05:  # DNLOAD-IDLE
                break
        if (i + 1) % 25 == 0 or i == n - 1:
            print("[*]  chunk %d/%d" % (i + 1, n))
    # zero-length DNLOAD + MANIFEST
    dev.ctrl_transfer(0x21, 1, n, 0, b"", TMO)
    # wait for MANIFEST (state 2 = MANIFEST COMPLETE)
    for _ in range(1000):
        try:
            st = dev.ctrl_transfer(0xa1, 3, 0, 0, 6, TMO)
            if st[4] == 0x02:
                print("[+] MANIFEST COMPLETE - bootrom is booting the image")
                return True
        except Exception:
            # device re-enumerates on manifest -> that's success
            print("[+] device re-enumerated during manifest (boot started)")
            return True
        time.sleep(0.01)
    print("[!] manifest timeout (may still be rebooting)")
    return False

def main():
    if len(sys.argv) < 2:
        print("usage: send_dfu.py <image.dfu>"); return 2
    img = open(sys.argv[1], "rb").read()
    print("[*] image: %s (%d bytes)" % (sys.argv[1], len(img)))
    print("[*] waiting for DFU device (Apple 05ac / PID 0x1225)...")
    dev = None
    for _ in range(60):  # up to 60s
        dev = find_dfu()
        if dev is not None:
            break
        time.sleep(1)
    if dev is None:
        print("[!] no DFU device found. Put iPod in DFU mode (Menu+Center twice).")
        return 1

    print("PLAN: send nano4g DFU image; stop on an error without replay", flush=True)
    try:
        return 0 if send(dev, img) else 2
    except Exception as exc:
        print("STOP: DFU transfer failed: %r" % exc)
        return 2

if __name__ == "__main__":
    sys.exit(main())
