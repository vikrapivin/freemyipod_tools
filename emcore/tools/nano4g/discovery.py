"""M2 event/control discovery. All opcode and IRQ meanings are hypotheses.

M1 event 5 and opcode 09 are tested alongside the M2 WAIT 12 masked-status
control. They are never substituted for WAIT 12 in a prerequisite sequence.
"""
import struct

from fmss.probe import FULLMAP, assemble, probe, S_NO_PREFLIGHT, S_NO_CLEAN, S_END_NOCLEAN, WAIT_END
from . import sequences as Q
from .m2_experiments import experiment, summary, buffer
from .protocol import DISCOVERY, MUTATE
from .followup_experiments import Case, run_cases


def states(s, t):
    a, g = t.scratch_addr(s), Q.geometry(s)
    return {
        "idle": "",
        "command": Q.wr(8, 0x70),
        "address": Q.command(0x90) + Q.wr(0x2c, 0) + Q.wr(0xc, 0) + Q.wr(4, 1),
        "data": Q.command(0x90) + Q.address(0, cycles=1) + Q.wr(0x30, 4) + Q.wr(0x14, 0x10) + Q.wr(4, 0xe2),
        "DMA": Q.dma(a).replace(Q.wait(2) + Q.wr(0x48, 0x100000), ""),
        "ECC": Q.dma(a) + Q.ecc(acknowledge=False),
        "status": Q.command(0x70) + Q.wr(4, 0xca),
        "read": Q.command(0) + Q.address(g.row) + Q.command(0x30),
        "erase": Q.command(0x60) + Q.address(g.row, cycles=3) + Q.command(0xd0),
        "program": Q.command(0x80) + Q.address(g.row) + Q.dma(a) + Q.transfer(512, into=False) + Q.command(0x10),
        "meta-encode": Q.metadata(bytes(range(10))),
        "meta-decode": Q.wr(0x40, 0xc2),
    }


def event_cases(s, t, name, state_names, ops=(7,), numbers=range(16), masks=((0, 0),), reverse=False):
    g = Q.geometry(s)
    actions = states(s, t)
    for state in state_names:
        for op in ops:
            for ns in (list(numbers), list(reversed(list(numbers)))) if reverse else (list(numbers),):
                for n in ns:
                    for arg, mask in masks:
                        # Capture raw VIC words and status before the candidate,
                        # without assigning either M1 interrupt line to M2.
                        capture = "    movi r1,#0x38e00008\n    ldi r2,[r1]\n    movi r1,#0x38e01008\n    ldi r3,[r1]\n"
                        candidate = "    .insn 0x%x,%d,0x%x,0x%x\n" % (op, n, arg, mask)
                        body = Q.setup(g.ce) + actions[state] + capture + candidate + "    ld r4,[0x48]\n    ld r5,[0x4c]\n"
                        key = f"{state}/op{op:02x}/n{n}/arg{arg:x}/mask{mask:x}/order{ns[0]}"
                        src = t.prologue() + body + Q.wr(0, 0) + "    movi r6, #0x600d600d\n    end\n" + t.trap()
                        yield Case(key, src, dict(scratch=buffer(s), flags=DISCOVERY |
                                   (MUTATE if state in ("erase", "program") else 0), timeout_us=20000), pins=True)


def event_matrix(s, t, name, *spec):
    return run_cases(name, s, t, lambda view, target: event_cases(view, target, name, *spec))


# Each matrix retains the original question's states and operand ranges while
# using M2 prerequisites. Busy timings require a physical device.
EVENT_PLANS = {
    "ev11": (("idle", "erase"), (7,), (11, 12), ((0, 0), (0x40, 0x40))),
    "evsweep": (("idle", "command", "address", "data", "DMA", "ECC", "status", "erase"), (7,), range(16)),
    "op12wait": (("idle", "command", "DMA", "ECC", "status"), (0x12, 7), range(16)),
    "evcons": (("command", "address", "DMA", "ECC", "status"), (7,), range(16)),
    "op16": (("idle", "command", "address", "data", "DMA", "ECC", "status"), (0x16, 9), (0,)),
    "opstate": (("idle", "command", "address", "data", "DMA", "ECC", "status"), (8, 9, 0x10, 0x12, 0x15, 0x16, 0xff), (0,)),
    "op09": (("idle", "command", "DMA", "ECC", "status"), (9, 0x1a, 0x20, 7), range(16)),
    "op09b": (("idle", "status", "erase"), (9, 7), range(16), ((0, 0), (0x40, 0x40), (0, 0x40))),
    "op09c": (("idle", "status"), (9, 7), range(32)),
    "op09nand": (("erase", "program", "read", "idle"), (9,), range(256)),
    "op09w33": (("idle", "erase", "program", "read"), (9,), (1, 4, 5, 32, 33, 37, 65, 69, 97, 133, 161, 165)),
    "w2state": (("idle", "command", "address", "DMA", "ECC", "status", "read"), (9,), range(256)),
    "evsweep2": (("erase", "program", "read", "ECC", "DMA", "idle"), (7, 9), range(256)),
    "evsweep3": (("meta-encode", "meta-decode", "ECC", "program", "DMA"), (7, 9), range(256)),
    "w2vic": (("idle", "command", "DMA", "ECC"), (9,), range(256)),
}


def bind_events(name, spec):
    def run(s, t):
        return event_matrix(s, t, name, *spec)
    run.__doc__ = "M2 state/opcode matrix; see EVENT_PLANS[%r] for exact cases." % name
    return run


for _name, _spec in EVENT_PLANS.items():
    experiment(_name)(bind_events(_name, _spec))


def latch_matrix(s, t, name, reps, mutations=False, rearm=False, enables=False):
    g, out = Q.geometry(s), {}
    actions = {"none": "", "reset-chip": Q.command(0xff), "setup": Q.wr(4, 0x33338e0),
               "clear-status": Q.wr(0x48, 0xff00ffe), "deselect": Q.wr(0, 0) + Q.setup(g.ce),
               "mode": Q.wr(4, 0x100e0), "DMA": Q.dma(t.scratch_addr(s)), "ECC": Q.ecc()}
    if not rearm:
        actions = {"none": ""}
    if mutations:
        actions["erase"] = Q.erase(g)
    for label, action in actions.items():
        for enable in (0, 1, 2, 0xffff) if enables else (0,):
            for event, arg, mask in ((5, 0, 0), (12, 0x40, 0x40)):
                for rep in range(reps):
                    # Both first and second latch are retained in one record.
                    body = Q.setup(g.ce) + Q.wr(0x44, enable)
                    for i in range(2):
                        body += (action if i else "") + Q.command(0x70) + Q.wr(4, 0xca)
                        body += Q.store(0x48, t.scratch_addr(s) + i * 16)
                        body += Q.store(0x4c, t.scratch_addr(s) + i * 16 + 4)
                        body += Q.wait(event, arg, mask)
                    body += Q.wr(0x44, 0)
                    key = f"{label}/en{enable:x}/wait{event}/rep{rep}"
                    out[key] = Q.run(s, t, body, scratch=buffer(s), pins=True, mutate=label == "erase")
    return summary(name, out)


LATCH_PLANS = {"ev5": (1, True, False, False), "ev5rule": (1, False, True, False),
               "ev5t": (3, False, False, False), "ev5en": (3, False, False, True),
               "ev5ready": (1, True, True, False), "ev5arm": (1, False, True, False),
               "latchrep": (40, False, True, True)}
for _name, _spec in LATCH_PLANS.items():
    def _bind(name, spec):
        def run(s, t):
            return latch_matrix(s, t, name, *spec)
        return run
    experiment(_name)(_bind(_name, _spec))


def irq_matrix(s, t, name, bits=range(32), reps=1, wait_event=False):
    out, g = {}, Q.geometry(s)
    for state in ("command", "address", "data", "status", "DMA", "ECC"):
        for bit in (-1, *bits):
            for rep in range(reps):
                en = 0 if bit < 0 else 1 << bit
                # +844 is an M1 candidate only; +818 is also swept, not asserted
                # to be the M2 enable register. Preserve and restore each value.
                for off in (0x44, 0x818, 0x844) if state == "ECC" else (0x44,):
                    body = Q.setup(g.ce) + "    ld r4,[0x%x]\n" % off + Q.wr(off, en) + states(s, t)[state]
                    body += "    movi r1,#0x38e00008\n    ldi r2,[r1]\n    movi r1,#0x38e01008\n    ldi r3,[r1]\n"
                    body += "    ld r5,[0xc0c]\n" + (Q.wait(15) if wait_event else "")
                    body += "    st [0x%x],r4\n" % off
                    out[f"{state}/off{off:x}/bit{bit}/rep{rep}"] = Q.run(s, t, body, pins=True, scratch=buffer(s))
    return summary(name, out)


for _name, _bits, _reps, _wait in (("irqsrc", range(16), 1, False), ("irqen", range(32), 1, False),
        ("irq044", range(32), 3, False), ("vic20", (0, 1, 2, 11, 12), 3, False),
        ("irqop", (0, 1, 2, 11, 12), 1, True)):
    def _bind(name, bits, reps, wait):
        return lambda s, t: irq_matrix(s, t, name, bits, reps, wait)
    experiment(_name)(_bind(_name, _bits, _reps, _wait))


@experiment("restore")
def restore(s, t):
    """M2 reset effects. Does not write unmeasured M1 power-on values."""
    out = {"before": Q.run(s, t, "", flags=FULLMAP | S_NO_PREFLIGHT | S_NO_CLEAN)}
    out["after-reset"] = Q.run(s, t, "", flags=FULLMAP)
    out["note"] = {"scope": "M2 native reset comparison; a power-on baseline requires a separate cold boot"}
    return summary("restore", out)


@experiment("straps")
def straps(s, t):
    """S5L8720 GPIO/PHY raw capture; no M1 boot-device decoding."""
    out = {}
    for port in range(16):
        for name, offset in (("PCON", 0), ("PDAT", 4)):
            addr = 0x3cf00000 + port * 32 + offset
            out[f"{name}{port:x}"] = {"address": addr, "value": struct.unpack("<I", s.read(addr, 4))[0]}
    return out, "S5L8720 GPIO capture; strap meanings not inferred from M1"


@experiment("fmcstat")
def fmcstat(s, t):
    """FMC_STATUS write-one-to-clear bits and which M2 operation sets them."""
    out, g = {}, Q.geometry(s)
    for bit in (12, 13, 18, 19):
        for state in ("idle", "command", "address", "data", "status", "DMA", "ECC", "meta-encode"):
            body = Q.setup(g.ce) + Q.wr(0x48, 1 << bit) + "    ld r2,[0x48]\n" + states(s, t)[state]
            body += "    ld r3,[0x48]\n"
            out[f"bit{bit}/{state}"] = Q.run(s, t, body, scratch=buffer(s), pins=True)
    return summary("fmcstat", out)


@experiment("fmcstat2")
def fmcstat2(s, t):
    """M2 flags after a short transfer followed by each type of operation."""
    out, g = {}, Q.geometry(s)
    for state in ("idle", "command", "address", "data", "status", "DMA", "ECC", "meta-encode"):
        body = Q.setup(g.ce) + states(s, t)["data"] + Q.wait(3) + "    ld r2,[0x48]\n"
        body += states(s, t)[state] + "    ld r3,[0x48]\n"
        out[state] = Q.run(s, t, body, scratch=buffer(s), pins=True)
    return summary("fmcstat2", out)


@experiment("fmc28b")
def fmc28b(s, t):
    """Counter before/after operations with and without the M2 setup word."""
    out, g = {}, Q.geometry(s)
    for init in (False, True):
        for state in ("idle", "command", "data", "DMA", "ECC"):
            body = (Q.setup(g.ce) if init else "") + "    ld r2,[0x28]\n" + states(s, t)[state] + "    ld r3,[0x28]\n"
            out[f"init{init}/{state}"] = Q.run(s, t, body, scratch=buffer(s), pins=True)
    return summary("fmc28b", out)
