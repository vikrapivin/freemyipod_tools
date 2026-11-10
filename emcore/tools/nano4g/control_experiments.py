"""M2 control, start-word, register-mask and channel applicability experiments."""
import struct

from fmss.probe import assemble, probe, FULLMAP, PROG_SRAM, WAIT_END
from . import sequences as Q
from .m2_experiments import experiment, summary, buffer
from .discovery import states, event_matrix
from .protocol import DISCOVERY


@experiment("rwmask")
def rwmask(s, t):
    """CPU readback masks in the four M2 snapshot windows; exclude actions."""
    excluded = {0, 4, 8, 0x38, 0x40, 0x48, 0x60, 0x64, 0x68, 0x80, 0x804, 0x810, 0x81c,
                0xc00, 0xc04, 0xc0c, 0xc38, 0xc4c, 0xc58, 0xc5c, 0xc60, 0xc64, 0xc6c}
    out = {}
    for base in t.windows:
        for off in range(base, base + 0x100, 4):
            if off in excluded:
                continue
            old = Q.run(s, t, f"    ld r0,[0x{off:x}]\n")["r"][0]
            for value in (0xffffffff, 0):
                out[f"{off:x}/{value:x}"] = Q.run(s, t, f"    ld r0,[0x{off:x}]\n" + Q.wr(off, old), params=[(off, value)])
    return summary("rwmask", out)


@experiment("blk100")
def blk100(s, t):
    """Direct M2 reads/writes of 0x100..1fc and the possible +0x200 mirror."""
    out = {}
    for off in range(0x100, 0x200, 4):
        for value in (0xffffffff, 0):
            body = f"    ld r1,[0x{off:x}]\n" + Q.wr(off, value)
            body += f"    ld r2,[0x{off:x}]\n    ld r3,[0x{off + 0x200:x}]\n    st [0x{off:x}],r1\n"
            out[f"{off:x}/{value:x}"] = Q.run(s, t, body, flags=FULLMAP)
    return summary("blk100", out)


@experiment("blk100b")
def blk100b(s, t):
    """Candidate timing words under M2 DMA/ECC workload, restored in each run."""
    out = {}
    for off in (*range(0x1b8, 0x1d4, 4), *range(0x3a8, 0x3b8, 4)):
        for value in (0, 1, 0xffffff):
            for op in ("DMA", "ECC", "data"):
                body = Q.setup(Q.geometry(s).ce) + f"    ld r4,[0x{off:x}]\n" + Q.wr(off, value)
                body += states(s, t)[op] * (8 if op in ("DMA", "ECC") else 1)
                body += f"    st [0x{off:x}],r4\n"
                out[f"{off:x}/{value:x}/{op}"] = Q.run(s, t, body, scratch=buffer(s), pins=True, flags=FULLMAP)
    return summary("blk100b", out)


@experiment("op12rel")
def op12rel(s, t):
    """CPU release candidates for raw opcode 12 with known scratch operands."""
    out = {}
    src = t.prologue() + "    .insn 0x12,2,0xd60,0xffffffff\n    movi r6,#0x600d600d\n    end\n" + t.trap()
    for off in (0xc00, 0xc08, 0xc0c, 0xc10, 0xc14, 0xc38, 0xc40, 0xc44, 0xc50, 0xc58, 0xd60):
        for value in (0, 1, 8, 0xff, 0xffb5):
            out[f"{off:x}/{value:x}"] = probe(s, t, assemble(t, src), params=[(0xd60, 0x1234)],
                late=[(off, value)], late_us=2000, timeout_us=20000)
    return summary("op12rel", out)


def start_matrix(s, t, name, words, bodies=None, sram=False):
    out, a, g = {}, t.scratch_addr(s), Q.geometry(s)
    bodies = bodies or {"end": "", "loop": "    movi r0,#100\nloop:\n    sub r0,r0,#1\n    bnz r0,loop\n",
                      "memory": "    movi r1,#0x%x\n    ldi r2,[r1]\n    sti [r1],r2\n" % a,
                      "timeout": Q.wait(1), "unknown": "    .insn 0x1a,0,0,0\n",
                      "DMA-ECC": Q.setup() + Q.dma(a) + Q.ecc()}
    for word in words:
        for label, body in bodies.items():
            for flag in (0, PROG_SRAM) if sram else (0,):
                out[f"{word:08x}/{label}/sram{bool(flag)}"] = Q.run(s, t, body, start_value=word,
                    scratch=buffer(s), flags=FULLMAP | flag, timeout_us=20000)
    return summary(name, out)


@experiment("startdiff")
def startdiff(s, t):
    """Observed M2 bootloader/restore start words and bit-six comparison pairs."""
    return start_matrix(s, t, "startdiff", (0xfff5, 0xffb5, 0xffd5, 0xff95))


@experiment("startbits")
def startbits(s, t):
    """M2 start word: repeated baseline and every single-bit toggle, SDRAM/SRAM."""
    return start_matrix(s, t, "startbits", (0xffb5, 0xffb5, 0xffb5, *(0xffb5 ^ (1 << b) for b in range(32))), sram=True)


@experiment("ctrlfx")
def ctrlfx(s, t):
    """Start bits on timeout, unknown opcode, event jump and a pending command."""
    return start_matrix(s, t, "ctrlfx", (1, 0xffb5, 0xfff5, *(1 | 1 << b for b in (2, *range(4, 16)))),
        {"timeout": Q.wait(3), "opcode": "    .insn 0x1a,0,0,0\n", "evjmp": "    .insn 0x16,0,0,0\n"})


@experiment("irqline")
def irqline(s, t):
    """Entire VIC0/VIC1 raw status around sequencer states; do not presume a line."""
    return start_matrix(s, t, "irqline", (0xffb5, 0xfff5),
        {"end": "", "pause": "    .insn 0x12,0,0,0\n", "signal": "    .insn 0x10,0,0,0\n", "fault": Q.wait(1)})


@experiment("c50fault")
def c50fault(s, t):
    """Read C50 before a fault, inside the vector, and after acknowledgement."""
    out = {}
    for fault, body in (("timeout", Q.wait(1)), ("opcode", "    .insn 0x1a,0,0,0\n")):
        for pending in (False, True):
            for c58 in (0, 4):
                src = t.prologue(c58) + (Q.setup(Q.geometry(s).ce) + Q.wr(8, 0x70) if pending else "")
                src += "    ld r1,[0xc50]\n" + body + "    ld r2,[0xc50]\n    end\n"
                src += "trap:\n    ld r2,[0xc50]\n    wr [0xc50],#1\n    ld r3,[0xc50]\n    end\n"
                out[f"{fault}/pending{pending}/c58{c58}"] = probe(s, t, assemble(t, src), flags=WAIT_END)
    return summary("c50fault", out)


@experiment("c50b8")
def c50b8(s, t):
    """C50 around resets and operation classes, then ten END-only runs."""
    out = {}
    for state in ("idle", "command", "DMA", "ECC", *(["idle"] * 10)):
        i = len(out)
        out[f"{i}/{state}"] = Q.run(s, t, Q.setup(Q.geometry(s).ce) + "    ld r2,[0xc50]\n" +
                                   states(s, t)[state] + "    ld r3,[0xc50]\n", scratch=buffer(s), pins=True)
    return summary("c50b8", out)


@experiment("c50x")
def c50x(s, t):
    """Fault sources: timeout, unknown opcode, and misaligned scratch access."""
    out = {}
    a = t.scratch_addr(s)
    for label, body in (("timeout", Q.wait(1)), ("opcode", "    .insn 0x1a,0,0,0\n"),
                        ("unaligned", f"    movi r1,#0x{a+1:x}\n    ldi r2,[r1]\n")):
        out[label] = Q.run(s, t, body, scratch=buffer(s), flags=FULLMAP | WAIT_END)
    return summary("c50x", out)


@experiment("ctrlwords")
def ctrlwords(s, t):
    """Unknown control words with timeout, opcode and M2 DMA/ECC operations."""
    out = {}
    for value in (0, 0xffffffff):
        for state in ("idle", "DMA", "ECC"):
            for off in (0xc10, 0xc48, 0xc54, 0xc3c):
                out[f"{off:x}/{value:x}/{state}"] = Q.run(s, t, Q.setup() + Q.wr(off, value) + states(s, t)[state],
                                                         scratch=buffer(s), flags=FULLMAP)
    return summary("ctrlwords", out)


@experiment("c08self")
def c08self(s, t):
    """Read C08 immediately after initiating each operation, before CPU sampling."""
    out = {}
    for state in ("idle", "command", "DMA", "ECC", "status"):
        out[state] = Q.run(s, t, Q.setup(Q.geometry(s).ce) + states(s, t)[state] + "    ld r2,[0xc08]\n",
                           scratch=buffer(s), pins=True)
    return summary("c08self", out)


@experiment("op16b")
def op16b(s, t):
    """Raw 16 with varied C58 policy and a pending command/DMA/ECC condition."""
    out = {}
    for c58 in (0, 1, 2, 4, 8):
        for state in ("idle", "command", "DMA", "ECC"):
            src = t.prologue(c58) + Q.setup(Q.geometry(s).ce) + states(s, t)[state]
            src += "    .insn 0x16,0,0,0\n    movi r6,#0x600d600d\n    end\n" + t.trap()
            out[f"c58{c58}/{state}"] = probe(s, t, assemble(t, src), scratch=buffer(s), flags=WAIT_END)
    return summary("op16b", out)


def channel(s, t, name, dma=False, events=False):
    """Probe candidate +400 registers; no unverified GPIO remux is attempted."""
    out, a = {}, t.scratch_addr(s)
    for base in (0, 0x400):
        for ce in range(4):
            body = Q.setup(ce)
            if dma:
                body += Q.dma(a) + Q.dma(a + 0x2000, into=False)
            else:
                body += Q.command(0x90) + Q.address(0, cycles=1) + Q.transfer(5, parity=True) + Q.wr(0x40, 0x52)
                body += "    ld r2,[0x60]\n"
            if base:
                import re
                body = re.sub(r"\[0x([0-9a-f]+)\]", lambda m: "[0x%x]" % (int(m[1], 16) + (base if int(m[1], 16) < 0x400 else 0)), body)
                # This is an applicability probe. No guessed event remapping:
                # inspect full raw state after bounded WAIT, not M1 event 6/8.
            body += Q.wr(base, 0)
            out[f"base{base:x}/CE{ce}"] = Q.run(s, t, body, scratch=buffer(s), pins=True,
                                               flags=DISCOVERY | FULLMAP, timeout_us=20000)
    out["applicability"] = {"pin_switch": "unimplemented; M1 GPIO mux is not valid evidence for S5L8720",
                            "scope": "candidate +400 register-set presence; no second-channel existence assumed"}
    return summary(name, out)


for _name, _dma in (("chan1", False), ("chan1pins", False), ("ev8", False), ("op09d", False), ("ch1buf", True)):
    def _bind(name, dma):
        return lambda s, t: channel(s, t, name, dma)
    experiment(_name)(_bind(_name, _dma))
