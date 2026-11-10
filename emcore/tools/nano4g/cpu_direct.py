"""Generic CPU NAND byte sequences and compact, read-only physical page maps.

No sequencer program is involved. Timing records are CPU microsecond samples;
command completion to ready is an upper bound that includes status-poll overhead.
"""
import struct

from .nand import cpu_nand

SEQ_MAGIC = 0x31514553
SEQ_MAX = 128
SEQ_BYTES = 0x10000
SEQ_PAYLOAD = 0x1000
CTRL0 = 0x01077801
KINDS = {'cmd': 1, 'addr': 2, 'out': 3, 'in': 4, 'ready': 5, 'timing': 6, 'delay': 7, 'select': 8}


def sequence_upload(steps, ctrl0=CTRL0, payload=b''):
    """Pack four-word (kind, a, b, c) descriptors. Kinds may be names or integers.

    cmd: byte; addr: length, low four bytes, high four bytes; out: length,
    payload offset, 0; in: length, output offset, path (0 DMA, 1 FIFO);
    ready: timeout us, status command (0x100 means already sent), status mask;
    timing: full CTRL0 excluding CE; delay: microseconds. Unused words are 0.
    """
    steps, payload = list(steps), bytes(payload)
    if not 1 <= len(steps) <= SEQ_MAX or len(payload) > SEQ_BYTES:
        raise ValueError('sequence exceeds instruction/payload buffer or has no steps')
    buf = bytearray(SEQ_PAYLOAD + len(payload))
    struct.pack_into('<4I', buf, 0, SEQ_MAGIC, len(steps), ctrl0, len(payload))
    for i, step in enumerate(steps):
        if len(step) != 4:
            raise ValueError('each sequence descriptor needs four words')
        kind, a, b, c = step
        kind = KINDS[kind] if isinstance(kind, str) else kind
        struct.pack_into('<4I', buf, 16 + 16 * i, kind, a, b, c)
    buf[SEQ_PAYLOAD:] = payload
    return bytes(buf)


def cpu_nand_sequence(s, steps, ce=0, row=0, ctrl0=CTRL0, payload=b'', timeout=10.0):
    """Execute exactly one uploaded sequence; return (worker status, CPUN record).

    A command's `begin`/`done` records delimit the CPU issue/completion interval.
    READY leaves the NAND in status mode; explicitly send the next command.
    Every READY has a reserved `status` marker (kind 12) holding its NAND
    status byte. FAIL is recorded, not converted to an aborted sequence.
    """
    upload = sequence_upload(steps, ctrl0, payload)
    return cpu_nand(s, 'sequence', ce, row, upload=upload, timeout=timeout)


def cpu_nand_map(s, ce, row, count, timeout=30.0):
    """Read every requested page (1..256) and retain a 64-byte record per page.

    Marker bytes are raw column 4096..4103, not proof of factory provenance.
    Parity masks mean exact agreement only; damaged/unknown formats may have 0.
    """
    job, out = cpu_nand(s, 'map', ce, row, count, timeout=timeout)
    if out is not None:
        data = out['data']
        if len(data) % 64:
            raise ValueError('truncated CPU map record')
        pages = []
        for offset in range(0, len(data), 64):
            r, rc, flags, crc = struct.unpack_from('<4I', data, offset)
            masks, = struct.unpack_from('<I', data, offset + 56)
            pages.append(dict(row=r, rc=rc - (1 << 32) if rc & 0x80000000 else rc,
                              flags=flags, crc32=crc, prefix=data[offset + 16:offset + 48].hex(),
                              marker=data[offset + 48:offset + 56].hex(),
                              boot_mask=masks & 7, main8_mask=masks >> 8 & 255,
                              main16_mask=masks >> 16 & 255, tail=data[offset + 60:offset + 64].hex()))
        out['pages'], out['map_rawsize'] = pages, out['unused']
    return job, out
