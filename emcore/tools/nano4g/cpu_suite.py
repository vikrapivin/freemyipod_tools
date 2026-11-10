"""On-device CPU NAND read/write suite.

One command, from emCORE/: ``python3 tools/n4g.py cpu-suite``.

Bank 1. The row is chosen at run time, above 1024, and outside block 0.
The FMSS reader is ``read-page`` (Rockbox read.s). The CPU path is
``cpu-nand`` with hardware ECC and BCH8. The two reads of the chosen page
have to match before anything is erased. Erase clears the whole block, so
every page of that block is written to the host first. The CPU writer then
programs one pattern. Both readers have to return it. The CPU writer
programs the saved pages back, and the FMSS reader checks the block.

The monitor has no FMSS program or erase command. Restore uses the CPU
writer. The backup file remains the pre-erase copy when that restore misses.
"""
import json
import os
import random
import struct
from datetime import datetime, timezone
from pathlib import Path

PAGE = 4096
META = 10
CHUNKS = 8
ROW_MIN = 1025
BANK = 1
BCH = 8
ERASED = b'\xfe' * CHUNKS
CLEAN = b'\x00' * CHUNKS
BLANK_DATA = b'\xff' * PAGE
BLANK_META = b'\xff' * META
PATTERN = bytes((i * 7 + 3) & 0xff for i in range(PAGE))
PATTERN_META = bytes.fromhex('00112233445566778899')
MAGIC = b'N4BK'
VERSION = 1
HEADER = struct.Struct('<4s9I')
RECORD_HEAD = struct.Struct('<B8s')


class SuiteError(Exception):
    pass


def default_backup_dir():
    return Path(__file__).resolve().parents[3] / 'nand_device_suite'


def classify(secstat):
    """'erased', 'programmed', 'uncorrectable', or 'mixed' from eight status bytes."""
    raw = bytes(secstat)
    if len(raw) != CHUNKS:
        raise SuiteError('sector status is %d bytes' % len(raw))
    if raw == ERASED:
        return 'erased'
    if any(b == 0xff for b in raw):
        return 'uncorrectable'
    if any(b == 0xfe for b in raw):
        return 'mixed'
    return 'programmed'


class Page(object):
    def __init__(self, data, meta, secstat):
        self.data = bytes(data)
        self.meta = bytes(meta)
        self.secstat = bytes(secstat)
        if len(self.data) != PAGE or len(self.meta) != META or len(self.secstat) != CHUNKS:
            raise SuiteError('page is %d data, %d meta, %d status bytes' % (
                len(self.data), len(self.meta), len(self.secstat)))
        self.kind = classify(self.secstat)

    def programmable(self):
        return self.kind == 'programmed'


def choose_row(rng, pages_per_block, blocks):
    """A row above 1024 and outside block 0. The value is not a constant."""
    if pages_per_block < 1 or blocks < 1:
        raise SuiteError('NAND geometry is empty')
    span = pages_per_block * blocks
    start = max(ROW_MIN, pages_per_block)
    if start >= span:
        raise SuiteError('geometry has no page above row 1024 outside block 0')
    return rng.randrange(start, span)


def require_geometry(geo):
    if geo.page_bytes != PAGE:
        raise SuiteError('page size is %d bytes' % geo.page_bytes)
    if geo.banks <= BANK:
        raise SuiteError('this suite uses bank 1; the chip reports %d banks' % geo.banks)
    if geo.ecc_bits != BCH:
        raise SuiteError(
            'the FMSS reader is on BCH%d. Reload the kernel so FIL init selects BCH8, '
            'then run cpu-suite again' % geo.ecc_bits)
    if not geo.chip_id:
        raise SuiteError('chip id is empty; NAND init did not finish')
    choose_row(random.Random(0), geo.pages_per_block, geo.blocks)


def accept_saved(page, row):
    if page.kind == 'erased':
        if page.data != BLANK_DATA or page.meta != BLANK_META:
            raise SuiteError('row %d is marked erased and still holds data' % row)
        return
    if page.kind != 'programmed':
        raise SuiteError('row %d is %s; this block stays untouched' % (row, page.kind))


def readers_agree(fmss, cpu):
    return (fmss.kind == cpu.kind and fmss.kind in ('erased', 'programmed')
            and fmss.data == cpu.data and fmss.meta == cpu.meta)


def pack_backup(geo, ce, block, row, pages):
    if len(pages) != geo.pages_per_block:
        raise SuiteError('backup has %d pages; the block has %d' % (len(pages), geo.pages_per_block))
    blob = bytearray(HEADER.pack(
        MAGIC, VERSION, ce, block, geo.pages_per_block, row, PAGE, META, geo.chip_id, BCH))
    for page in pages:
        kind = 1 if page.programmable() else 0
        blob += RECORD_HEAD.pack(kind, page.secstat)
        blob += page.data + page.meta
    return bytes(blob)


def load_backup(path):
    blob = Path(path).read_bytes()
    if len(blob) < HEADER.size:
        raise SuiteError('backup file is too short')
    magic, version, ce, block, ppb, row, page_bytes, meta_bytes, chip_id, bch = HEADER.unpack_from(blob)
    if magic != MAGIC or version != VERSION:
        raise SuiteError('backup file is not a cpu-suite image')
    if page_bytes != PAGE or meta_bytes != META or bch != BCH or ppb < 1:
        raise SuiteError('backup geometry does not match this suite')
    if not 0 <= ce < 8 or block < 0 or row < 0:
        raise SuiteError('backup addresses are invalid')
    record = RECORD_HEAD.size + PAGE + META
    body = blob[HEADER.size:]
    if len(body) != ppb * record:
        raise SuiteError('backup length does not match %d pages' % ppb)
    pages = []
    for i in range(ppb):
        off = i * record
        kind, secstat = RECORD_HEAD.unpack_from(body, off)
        data = body[off + RECORD_HEAD.size:off + RECORD_HEAD.size + PAGE]
        meta = body[off + RECORD_HEAD.size + PAGE:off + record]
        page = Page(data, meta, secstat)
        if kind not in (0, 1) or (kind == 1) != page.programmable() or (kind == 0 and page.kind != 'erased'):
            raise SuiteError('backup page %d does not match its saved status' % i)
        pages.append(page)
    header = dict(ce=ce, block=block, pages_per_block=ppb, row=row, chip_id=chip_id, bch=bch)
    return header, pages


def _atomic_write(path, blob):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.partial')
    try:
        with tmp.open('wb') as out:
            out.write(blob)
            out.flush()
            os.fsync(out.fileno())
        os.replace(tmp, path)
        fd = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except Exception:
        if tmp.exists():
            tmp.unlink()
        raise


def backup_paths(directory, ce, block, row, stamp):
    directory = Path(directory)
    stem = 'ce%d-block%d-row%d-%s' % (ce, block, row, stamp)
    return directory / (stem + '.n4bk'), directory / (stem + '.json')


def write_report(path, report):
    _atomic_write(path, (json.dumps(report, indent=1) + '\n').encode())


def _stamp(now):
    when = now or datetime.now(timezone.utc)
    return when.strftime('%Y%m%dT%H%M%SZ')


def _check_live(dev, geo):
    live = dev.geometry()
    if (live.pages_per_block, live.blocks, live.banks, live.page_bytes, live.chip_id) != (
            geo.pages_per_block, geo.blocks, geo.banks, geo.page_bytes, geo.chip_id):
        raise SuiteError('NAND geometry changed during the suite')
    require_geometry(live)
    return live


def _read_block(dev, geo, ce, block, agreed, agreed_row):
    pages = []
    base = block * geo.pages_per_block
    for index in range(geo.pages_per_block):
        row = base + index
        if index == 0 or index + 1 == geo.pages_per_block or index % 32 == 0:
            print('cpu-suite: FMSS read page %d/%d of the block' % (index + 1, geo.pages_per_block), flush=True)
        page = dev.fmss_read(ce, row)
        accept_saved(page, row)
        if row == agreed_row and (page.data != agreed.data or page.meta != agreed.meta):
            raise SuiteError('row %d changed between the two reads' % row)
        pages.append(page)
    return pages


def _expect_erased(page, row):
    if page.kind != 'erased' or page.data != BLANK_DATA or page.meta != BLANK_META:
        raise SuiteError('row %d did not read back erased' % row)


def _expect_pattern(page, row):
    if page.data != PATTERN or page.meta != PATTERN_META or page.secstat != CLEAN:
        raise SuiteError('row %d did not read back the programmed pattern' % row)


def restore_pages(dev, geo, ce, block, pages):
    """Erase the block and program every saved programmed page. Returns FMSS mismatches."""
    if len(pages) != geo.pages_per_block:
        raise SuiteError('restore image has %d pages' % len(pages))
    saved = [(i, page) for i, page in enumerate(pages) if page.programmable()]
    print('PLAN: CPU erase CE%d block %d, then program %d saved pages' % (ce, block, len(saved)), flush=True)
    dev.erase(ce, block)
    base = block * geo.pages_per_block
    for index, page in saved:
        dev.program(ce, base + index, page.data, page.meta)
    missed = []
    for index, page in enumerate(pages):
        got = dev.fmss_read(ce, base + index)
        if page.programmable():
            if got.data != page.data or got.meta != page.meta or got.secstat != CLEAN:
                missed.append(index)
        elif got.kind != 'erased' or got.data != BLANK_DATA or got.meta != BLANK_META:
            missed.append(index)
    return missed


def _result(ok, phase, message, **fields):
    out = dict(passed=ok, phase=phase, message=message)
    out.update(fields)
    return out


SCAN_PAGES = 128


def run_one_page(dev, rng):
    """Read one erased page with both drivers, program it from the CPU, and read it back with both.

    The block is not erased. A page that already holds data is left alone.
    """
    dev.prepare()
    geo = dev.geometry()
    require_geometry(geo)
    span = geo.pages_per_block * geo.blocks
    start = choose_row(rng, geo.pages_per_block, geo.blocks)
    ce = BANK
    print('cpu-suite: chip %08x, looking for one erased page from row %d' % (geo.chip_id, start), flush=True)
    row = None
    for step in range(SCAN_PAGES):
        candidate = start + step
        if candidate >= span:
            break
        fmss = dev.fmss_read(ce, candidate)
        if fmss.kind != 'erased':
            continue
        _check_live(dev, geo)
        cpu = dev.cpu_read(ce, candidate)
        if not readers_agree(fmss, cpu):
            block, page = divmod(candidate, geo.pages_per_block)
            return _result(False, 'agree',
                           'FMSS and CPU reads of row %d differ. Nothing was programmed.' % candidate,
                           ce=ce, block=block, page=page, row=candidate, backup=None,
                           chip_id=geo.chip_id, bch=BCH)
        row = candidate
        break
    if row is None:
        return _result(False, 'read',
                       'No erased page in the %d rows from %d. Nothing was programmed.' % (SCAN_PAGES, start),
                       ce=ce, block=start // geo.pages_per_block, page=start % geo.pages_per_block,
                       row=start, backup=None, chip_id=geo.chip_id, bch=BCH)
    block, page = divmod(row, geo.pages_per_block)
    print('cpu-suite: both readers see row %d erased' % row, flush=True)
    print('PLAN: CPU program pattern at CE%d row %d' % (ce, row), flush=True)
    dev.program(ce, row, PATTERN, PATTERN_META)
    try:
        _expect_pattern(dev.fmss_read(ce, row), row)
        _expect_pattern(dev.cpu_read(ce, row), row)
    except SuiteError as exc:
        return _result(False, 'pattern', str(exc),
                       ce=ce, block=block, page=page, row=row, backup=None,
                       chip_id=geo.chip_id, bch=BCH)
    return _result(True, 'page',
                   'PASS CE%d block %d page %d row %d. Both readers returned the pattern. '
                   'The block was not erased.' % (ce, block, page, row),
                   ce=ce, block=block, page=page, row=row, backup=None,
                   chip_id=geo.chip_id, bch=BCH)


def run_read(dev, rng):
    """Compare one page with both readers. Does not erase or program."""
    dev.prepare()
    geo = dev.geometry()
    require_geometry(geo)
    row = choose_row(rng, geo.pages_per_block, geo.blocks)
    block, page = divmod(row, geo.pages_per_block)
    ce = BANK
    print('cpu-suite: chip %08x, CE%d block %d page %d row %d' % (
        geo.chip_id, ce, block, page, row), flush=True)
    fmss = dev.fmss_read(ce, row)
    _check_live(dev, geo)
    cpu = dev.cpu_read(ce, row)
    common = dict(ce=ce, block=block, page=page, row=row, backup=None, chip_id=geo.chip_id, bch=BCH)
    if not readers_agree(fmss, cpu):
        return _result(False, 'agree',
                       'FMSS and CPU reads of row %d differ. Nothing was erased.' % row, **common)
    return _result(True, 'read',
                   'PASS CE%d block %d page %d row %d. Both readers agree (%s). Nothing was erased.' % (
                       ce, block, page, row, fmss.kind), **common)


def run_suite(dev, rng, backup_dir, now=None):
    """Compare, back up, program one pattern, and restore. Returns a result dict."""
    dev.prepare()
    geo = dev.geometry()
    require_geometry(geo)
    row = choose_row(rng, geo.pages_per_block, geo.blocks)
    block, page = divmod(row, geo.pages_per_block)
    ce = BANK
    print('cpu-suite: chip %08x, CE%d block %d page %d row %d' % (
        geo.chip_id, ce, block, page, row), flush=True)
    fmss = dev.fmss_read(ce, row)
    _check_live(dev, geo)
    cpu = dev.cpu_read(ce, row)
    if not readers_agree(fmss, cpu):
        return _result(False, 'agree',
                       'FMSS and CPU reads of row %d differ. Nothing was erased.' % row,
                       ce=ce, block=block, page=page, row=row, backup=None)
    print('cpu-suite: FMSS and CPU agree on row %d (%s)' % (row, fmss.kind), flush=True)
    try:
        saved = _read_block(dev, geo, ce, block, fmss, row)
        _check_live(dev, geo)
    except SuiteError as exc:
        return _result(False, 'backup', str(exc) + ' Nothing was erased.',
                       ce=ce, block=block, page=page, row=row, backup=None)
    stamp = _stamp(now)
    image_path, report_path = backup_paths(backup_dir, ce, block, row, stamp)
    _atomic_write(image_path, pack_backup(geo, ce, block, row, saved))
    common = dict(ce=ce, block=block, page=page, row=row, backup=str(image_path),
                  report=str(report_path), chip_id=geo.chip_id, bch=BCH,
                  programmed_pages=sum(page.programmable() for page in saved))
    print('cpu-suite: backup %d pages -> %s' % (len(saved), image_path), flush=True)
    write_report(report_path, _result(False, 'backup-committed',
                                      'Backup is on disk. Erase has not finished.', **common))
    pattern_error = None
    try:
        print('PLAN: CPU erase CE%d block %d (row %d)' % (ce, block, block * geo.pages_per_block), flush=True)
        dev.erase(ce, block)
        _expect_erased(dev.fmss_read(ce, row), row)
        _expect_erased(dev.cpu_read(ce, row), row)
        print('PLAN: CPU program pattern at CE%d row %d' % (ce, row), flush=True)
        dev.program(ce, row, PATTERN, PATTERN_META)
        _expect_pattern(dev.fmss_read(ce, row), row)
        _expect_pattern(dev.cpu_read(ce, row), row)
        print('cpu-suite: both readers returned the pattern at row %d' % row, flush=True)
    except SuiteError as exc:
        pattern_error = str(exc)
    try:
        missed = restore_pages(dev, geo, ce, block, saved)
    except SuiteError as exc:
        message = ('The CPU writer did not restore CE%d block %d (%s). The backup file is the '
                   'pre-erase copy. python3 tools/n4g.py cpu-suite --restore %s' % (ce, block, exc, image_path))
        if pattern_error:
            message = pattern_error + ' ' + message
        report = _result(False, 'restore', message, restored=False, pattern_ok=pattern_error is None, **common)
        write_report(report_path, report)
        return report
    restored = not missed
    if pattern_error or missed:
        parts = []
        if pattern_error:
            parts.append(pattern_error)
        if missed:
            shown = ', '.join(str(i) for i in missed[:8])
            parts.append('FMSS check missed %d restored pages (%s)' % (len(missed), shown))
        elif pattern_error:
            parts.append('The block was restored and the FMSS reader matches the backup.')
        report = _result(False, 'verify' if pattern_error is None else 'pattern', ' '.join(parts),
                         restored=restored, pattern_ok=pattern_error is None, **common)
        write_report(report_path, report)
        return report
    try:
        cpu_back = dev.cpu_read(ce, row)
    except SuiteError as exc:
        report = _result(False, 'verify',
                         'The FMSS reader matches the backup. The CPU read of restored row %d failed: %s' % (row, exc),
                         restored=True, pattern_ok=True, **common)
        write_report(report_path, report)
        return report
    fmss_back = saved[page]
    # A page written again comes back clean, including one whose saved status had corrections.
    cpu_ok = (cpu_back.data == fmss_back.data and cpu_back.meta == fmss_back.meta and (
        cpu_back.secstat == CLEAN if fmss_back.programmable() else cpu_back.kind == 'erased'))
    if not cpu_ok:
        report = _result(False, 'verify',
                         'The FMSS reader matches the backup. The CPU read of restored row %d does not.' % row,
                         restored=True, pattern_ok=True, **common)
        write_report(report_path, report)
        return report
    report = _result(True, 'done',
                     'PASS CE%d block %d page %d row %d. Both readers saw the pattern. '
                     'The block matches the backup. Backup kept at %s' % (ce, block, page, row, image_path),
                     restored=True, pattern_ok=True, **common)
    write_report(report_path, report)
    return report


def restore_saved(dev, path):
    """Program a suite backup back and check it with the FMSS reader."""
    header, pages = load_backup(path)
    dev.prepare()
    geo = dev.geometry()
    require_geometry(geo)
    if header['chip_id'] != geo.chip_id:
        raise SuiteError('backup chip %08x does not match the running chip %08x' % (
            header['chip_id'], geo.chip_id))
    if header['pages_per_block'] != geo.pages_per_block:
        raise SuiteError('backup has %d pages per block; the chip has %d' % (
            header['pages_per_block'], geo.pages_per_block))
    if header['block'] >= geo.blocks:
        raise SuiteError('backup block %d is outside the chip' % header['block'])
    print('cpu-suite: restore CE%d block %d from %s' % (header['ce'], header['block'], path), flush=True)
    missed = restore_pages(dev, geo, header['ce'], header['block'], pages)
    if missed:
        raise SuiteError('FMSS check missed %d pages after restore. The backup file is still %s' % (
            len(missed), path))
    return header


class SessionDevice(object):
    """USB adapter. FMSS reads use read-page. CPU reads and writes use cpu_nand."""

    def __init__(self, session):
        self.session = session

    def prepare(self):
        from .nand import stack_prepare
        from .protocol import CPU_NAND, MUTATE
        if not self.session.probe_caps & CPU_NAND:
            raise SuiteError('this kernel has no CPU NAND command. python3 tools/n4g.py reload')
        if not self.session.probe_caps & MUTATE:
            raise SuiteError('writes are off on this kernel. python3 tools/n4g.py reload')
        print('cpu-suite: preparing gates, pins and NAND init', flush=True)
        stack_prepare(self.session)

    def geometry(self):
        from types import SimpleNamespace
        st = self.session.status()
        geom = st.geom
        return SimpleNamespace(pages_per_block=geom[0] or 0, banks=geom[1] or 0, blocks=geom[3] or 0,
                              page_bytes=geom[4] or 0, spare=geom[5] or 0, ecc_bits=geom[8] or 0,
                              chip_id=geom[11] or 0)

    def fmss_read(self, ce, row):
        from .nand import check
        from .protocol import Cmd, ToolError
        try:
            st = check(self.session.job(Cmd.NAND_READ_PAGE, ce, row), (0, 1))
            raw = self.session.read(st.result_addr, st.result_len)
        except ToolError as exc:
            raise SuiteError('FMSS read of CE%d row %d failed: %s' % (ce, row, exc)) from exc
        if len(raw) < 0x1028:
            raise SuiteError('FMSS read of CE%d row %d returned %d bytes' % (ce, row, len(raw)))
        return Page(raw[:PAGE], raw[0x1000:0x1000 + META], raw[0x1020:0x1020 + CHUNKS])

    def _stack(self, op, ce, row, data=None, meta=None):
        from .nand import cpu_stack
        from .protocol import ToolError
        try:
            job, out = cpu_stack(self.session, op, ce, row, 'hw' if op != 'erase' else 'raw', 'main', 'dma', BCH,
                                 data, meta, timeout=30.0)
        except ToolError as exc:
            raise SuiteError('CPU %s of CE%d row %d failed: %s' % (op, ce, row, exc)) from exc
        if job['result'] or out is None or out['rc']:
            rc = None if out is None else out['rc']
            raise SuiteError('CPU %s of CE%d row %d returned job %s rc %s' % (op, ce, row, job['result'], rc))
        return out

    def cpu_read(self, ce, row):
        out = self._stack('read', ce, row)
        if out.get('nsectors') != CHUNKS or len(out.get('data') or b'') < PAGE or len(out.get('meta') or b'') < META:
            raise SuiteError('CPU read of CE%d row %d returned a short page' % (ce, row))
        secstat = bytearray(CHUNKS)
        for index, sector in enumerate(out['sectors'][:CHUNKS]):
            flags = sector['flags']
            if 'uncorrectable' in flags:
                secstat[index] = 0xff
            elif 'erased' in flags:
                secstat[index] = 0xfe
            elif 'ok' in flags:
                if sector['corrected'] > 0xfd:
                    raise SuiteError('CPU sector %d correction count is %d' % (index, sector['corrected']))
                secstat[index] = sector['corrected']
            else:
                raise SuiteError('CPU sector %d of row %d has no status' % (index, row))
        return Page(out['data'][:PAGE], out['meta'][:META], bytes(secstat))

    def erase(self, ce, block):
        geo = self.geometry()
        self._stack('erase', ce, block * geo.pages_per_block)

    def program(self, ce, row, data, meta):
        self._stack('program', ce, row, data, meta)


def command(session, backup_dir=None, restore=None, rng=None, now=None, read_only=False, one_page=False):
    directory = Path(backup_dir) if backup_dir else default_backup_dir()
    try:
        dev = SessionDevice(session)
        if restore:
            restore_saved(dev, restore)
            print('cpu-suite: PASS restore %s' % restore, flush=True)
            return 0
        rng = rng or random.SystemRandom()
        if one_page:
            report = run_one_page(dev, rng)
        elif read_only:
            report = run_read(dev, rng)
        else:
            report = run_suite(dev, rng, directory, now)
    except SuiteError as exc:
        print('cpu-suite: FAIL %s' % exc, flush=True)
        return 1
    print('cpu-suite: %s%s' % ('' if report['passed'] else 'FAIL ', report['message']), flush=True)
    return 0 if report['passed'] else 1
