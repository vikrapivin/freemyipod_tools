"""The working nano4g status ABI on the current emCORE transport. No command replay."""
import hashlib
import json
import os
import struct

from nano3g.session import Session as Transport, ToolError
from .legacy.n4g_nand import parse_status, format_status, STATUS_BYTES
from .protocol import Cmd, MUTATE, BULK_USB


class TransportUnavailable(ToolError):
    """No completion was received; replaying the outstanding command is forbidden."""
    pass


def check(st, ok_lib=(0,)):
    if st.job_result or st.job_lib_rc not in ok_lib:
        raise ToolError(format_status(st))
    return st


class Session(Transport):
    def __init__(self, usb="auto", **kw):
        super().__init__(**kw)
        self.journal = None
        self.probe_index = 0
        self.em.bulk_write = False
        if self.em.lib.dev.hwtypeid != 0x47344e49:
            self.close()
            raise ToolError("Expected an iPod nano 4G running emCORE")
        magic, self.probe_version, self.probe_caps = self.cmd(Cmd.NAND_FMSS_INFO)
        if magic != 0x4d325052 or self.probe_version != 14:
            self.close()
            raise ToolError("Expected the M2 probe v14 image")
        try:
            self.configure_transport(usb)
        except Exception:
            self.close()
            raise

    def configure_transport(self, mode):
        if mode not in ('auto', 'bulk', 'control'):
            raise ToolError('USB mode must be auto, bulk or control')
        capable = bool(self.probe_caps & BULK_USB)
        endpoints = bool(self.em.lib.dev.bulkin and self.em.lib.dev.bulkout)
        if mode == 'bulk' and not (capable and endpoints):
            raise ToolError('Bulk USB requires the updated kernel and both debug endpoints; reload using --usb auto')
        if mode == 'auto' and capable and not endpoints:
            raise ToolError('Kernel advertises bulk USB but the debug endpoints are absent')
        self.usb_transport = 'bulk' if mode != 'control' and capable and endpoints else 'control'
        self.em.lib.bulkin = self.usb_transport == 'bulk'
        self.em.lib.bulkout = bool(self.em.lib.dev.bulkout)
        self.em.bulk_write = self.usb_transport == 'bulk'

    def read(self, addr, size):
        if self.usb_transport == 'bulk' and size >= 2048:
            chunks = []
            if addr & 63:
                n = min(64 - (addr & 63), size)
                chunks.append(self.em._readmem(addr, n)); addr += n; size -= n
            while size >= 64:
                n = min(size & ~63, 256 * 1024)
                data = bytes(self.em._readmem_bulk(addr, n))
                if len(data) != n: raise ToolError('Short bulk read; transfer not retried')
                chunks.append(data); addr += n; size -= n
            if size: chunks.append(self.em._readmem(addr, size))
            return b''.join(chunks)
        return b"".join(self.em._readmem(addr + off, min(512, size - off))
                        for off in range(0, size, 512))

    def write_memory(self, addr, data):
        data = bytes(data)
        offset = 0
        if self.usb_transport == 'bulk' and len(data) >= 64:
            if addr & 63:
                n = min(64 - (addr & 63), len(data))
                self.em._writemem(addr, data[:n]); offset += n; addr += n
            while len(data) - offset >= 64:
                n = min((len(data) - offset) & ~63, 256 * 1024)
                self.em._writemem_bulk(addr, data[offset:offset + n])
                offset += n; addr += n
        # Short alignment edges and explicit control mode retain the known EP0 geometry.
        while offset < len(data):
            n = min(64, len(data) - offset)
            self.em._writemem(addr, data[offset:offset + n])
            offset += n; addr += n

    def status(self):
        addr, size, result = self.cmd(Cmd.NAND_STATUS)
        if size != STATUS_BYTES:
            raise ToolError("Wrong nano4g status ABI: 0x%x" % size)
        st = parse_status(self.read(addr, size))
        if st.result_addr != result or st.result_cap < 0x40000:
            raise ToolError("The running image lacks the 256 KiB probe buffer")
        return st

    def job(self, c, a=0, b=0, timeout=10.0, d=0, poll_interval=None, **kw):
        try:
            return self._job(c, a, b, timeout, d, poll_interval, **kw)
        except OSError as exc:
            self.transport_unavailable = True
            self.log({'type': 'transport-lost', 'command': int(c), 'message': str(exc),
                      'last_status': getattr(self, 'last_job_status', None),
                      'pending_job': getattr(self, 'pending_job', None)})
            raise TransportUnavailable('USB communication lost during command %d; completion unknown: %s' % (c, exc)) from exc

    def firmware_identity(self):
        magic = 0x3149464e
        addr, size, tag = self.cmd(Cmd.NAND_STATUS, magic)
        if tag != magic:
            return {'supported': False}
        if size != 40:
            raise ToolError('Wrong running-code identity size')
        v = struct.unpack('<10I', self.read(addr, size))
        if v[:3] != (magic, 1, size):
            raise ToolError('Wrong running-code identity ABI')
        regions, digest = [], hashlib.sha256()
        for address, count in ((v[3], v[4]), (v[5], v[6])):
            if not 0 < count <= 0x54000:
                raise ToolError('Invalid running-code region size')
            data = self.read(address, count)
            digest.update(data)
            regions.append(dict(address=address, bytes=count, sha256=hashlib.sha256(data).hexdigest()))
        return dict(supported=True, scope='running immutable SRAM and SDRAM code/rodata; not whole image',
                    sha256=digest.hexdigest(), regions=regions, sctlr=v[7], ttbr=v[8], reloaded=v[9])

    def gate_checkpoint(self):
        magic = 0x31504347
        addr, size, tag = self.cmd(Cmd.NAND_STATUS, magic)
        if tag != magic or size != 92:
            raise ToolError('This checkpoint experiment requires the GCP1 firmware')
        v = struct.unpack('<23I', self.read(addr, size))
        if v[4] == 0:
            return None
        if v[:3] != (magic, 1, size):
            raise ToolError('Wrong gate checkpoint ABI')
        return dict(token=v[3], phase=v[4], job_seq=v[5], case_index=v[6], iterations=v[7],
                    masks=list(v[8:13]), pwrcon=list(v[13:18]), vic_raw=list(v[18:20]),
                    c08=v[20], pc=v[21], sctlr=v[22])

    def collect_gate_checkpoint(self, seq):
        record = self.gate_checkpoint()
        if not record or record['phase'] != 1 or record['job_seq'] != seq:
            return
        batch = self.pending_job.get('probe') or self.pending_job.get('batch') or self.active_batch
        index = record['case_index']
        record.update(experiment=batch['experiment'], case=batch['cases'][index],
                      stage='before candidate start; pulse not yet attempted')
        self.gate_checkpoints = getattr(self, 'gate_checkpoints', [])
        if any(r['token'] == record['token'] for r in self.gate_checkpoints):
            raise ToolError('Checkpoint ACK did not advance the worker; no replay')
        self.gate_checkpoints.append(record)
        self.log({'type': 'gate-checkpoint', **record})
        self.pending_job['checkpoint'] = record
        if hasattr(self, 'checkpoint_diagnostics'):
            self.checkpoint_diagnostics()
        # Automatic transport acknowledgement, after durable host storage.
        self.pending_job['phase'] = 'checkpoint-ack-submitting'
        self.log({'type': 'gate-checkpoint-ack', 'token': record['token'], 'job_seq': seq})
        self.cmd(Cmd.NAND_STATUS, 0x31504347, record['token'])
        self.pending_job['phase'] = 'checkpoint-ack-returned'
        self.log({'type': 'gate-checkpoint-ack-returned', 'token': record['token'], 'job_seq': seq})

    def worker_diagnostics(self):
        """Read-only optional ABI; old kernels return their ordinary status."""
        magic = 0x3144574e
        addr, size, tag = self.cmd(Cmd.NAND_STATUS, magic)
        if tag != magic:
            return {'supported': False}
        fields = ('magic version size jobs_started state job_kind worker_state worker_block '
                  'worker_pc worker_sp stack_guard wake_signalled wake_waiter current_thread '
                  'current_state scheduler_frozen vic0_enable vic0_raw vic1_enable vic1_raw '
                  'timer_b_con timer_b_cmd timer_b_count unhandled_irq_last unhandled_irq_count '
                  'direct_dispatches queued_worker_state queued_current_thread queued_vic0_enable').split()
        if size != len(fields) * 4:
            raise ToolError('Wrong worker diagnostic size: %d' % size)
        result = dict(zip(fields, struct.unpack('<%dI' % len(fields), self.read(addr, size))))
        if (result['magic'], result['version'], result['size']) != (magic, 1, size):
            raise ToolError('Wrong worker diagnostic ABI')
        return result

    def observe_job(self, st):
        fields = ('busy', 'state', 'jobs_started', 'jobs_done', 'job_kind', 'job_arg',
                  'job_result', 'job_lib_rc', 'progress_done', 'progress_total', 'job_detail', 'now_us')
        status = {name: getattr(st, name) for name in fields}
        batch = getattr(self, 'active_batch', None)
        if st.job_kind == 18 and batch:
            status['experiment'] = batch['experiment']
            index = st.progress_done
            status['last_seen_case'] = batch['cases'][index] if index < len(batch['cases']) else None
            status['progress_scope'] = 'last status observation; later cases may have started before communication was lost'
        self.last_job_status = status
        key = tuple(status[name] if not isinstance(status[name], list) else tuple(status[name])
                    for name in fields if name != 'now_us')
        if key != getattr(self, '_progress_key', None):
            self._progress_key = key
            self.log({'type': 'job-progress', **status})

    def _job(self, c, a=0, b=0, timeout=10.0, d=0, poll_interval=None, **kw):
        self.pending_job = dict(command=int(c), args=[a, b, d], phase='before-submit')
        if int(c) == int(Cmd.NAND_BATCH_RUN) and getattr(self, 'active_batch', None):
            self.pending_job['batch'] = dict(self.active_batch)
        if int(c) == int(Cmd.NAND_FMSS_PROBE) and getattr(self, 'active_probe', None):
            self.pending_job['probe'] = dict(self.active_probe)
        self.sleep(self.settle)
        before = self.status()
        self.observe_job(before)
        if before.busy:
            raise ToolError("A NAND job is already running")
        print("PLAN: command %d args 0x%x 0x%x" % (c, a, b), flush=True)
        self.pending_job['phase'] = 'submitting'
        self.log({'type': 'job-submit', **self.pending_job})
        accepted, seq, _ = self.cmd(c, a, b, d)
        self.pending_job.update(phase='reply-received', accept=accepted, job_seq=seq)
        self.log({'type': 'job-accepted', **self.pending_job})
        if accepted:
            raise ToolError("Command refused (%d); no retry" % accepted)
        start = self.now()
        while self.now() - start < timeout:
            self.sleep(self.poll if poll_interval is None else poll_interval)
            st = self.status()
            self.observe_job(st)
            if st.jobs_done == seq and st.state == 3:
                self.pending_job['phase'] = 'done'
                return st
            if st.jobs_started != seq:
                raise ToolError("Job sequence changed; device reset or another client is active")
            context = self.pending_job.get('batch') or self.pending_job.get('probe') or {}
            if context.get('checkpoint'):
                self.collect_gate_checkpoint(seq)
        diagnostic = None
        if st.state == 1:
            # Preserve the accepted job; no replay, reset or FREE on timeout.
            try:
                diagnostic = self.worker_diagnostics()
            except Exception as exc:
                diagnostic = {'error': str(exc)}
            self.last_job_status['worker_diagnostics'] = diagnostic
            self.log({'type': 'worker-timeout', **self.last_job_status})
        raise ToolError("Job timed out; completion not received. Last status:\n" + format_status(st)
                        + ('\nWorker diagnostics: ' + json.dumps(diagnostic) if diagnostic is not None else ''))

    def log(self, record):
        if self.journal:
            self.journal.write(json.dumps(record, separators=(',', ':')) + "\n")
            self.journal.flush()
            os.fsync(self.journal.fileno())

    def log_many(self, records):
        """One durable flush per collected batch; raw output was saved first."""
        if self.journal:
            for record in records:
                self.journal.write(json.dumps(record, separators=(',', ':')) + "\n")
            self.journal.flush()
            os.fsync(self.journal.fileno())

    def plan_probe(self, name, blob, scratch):
        flags = struct.unpack_from("<I", blob, 8)[0]
        if flags & MUTATE and not self.probe_caps & MUTATE:
            raise ToolError("Mutation requires a reload: run tools/n4g.py reload first")
        self.probe_index += 1
        count = struct.unpack_from('<I', blob, 16)[0]
        params = struct.iter_unpack('<II', blob[44:44+8*count])
        self.active_probe = dict(experiment=name, cases=[name],
                                 request_sha256=hashlib.sha256(blob).hexdigest(),
                                 checkpoint=any(off == 0x40000220 and val == 1 for off, val in params))
        # The complete request is retained, including dynamically chosen DMA addresses.
        self.log({"type": "plan", "probe": self.probe_index, "experiment": name,
                  "request": blob.hex(), "sha256": hashlib.sha256(blob).hexdigest(),
                  "scratch": None if scratch is None else scratch.hex()})
        print("PLAN: %s probe %d, %d bytes" % (name, self.probe_index, len(blob)), flush=True)

    def record_probe(self, rec):
        from fmss.probe import jsonable
        self.log({"type": "result", "probe": self.probe_index, "data": jsonable(rec)})
        if not rec["pre_ok"]:
            print("OBSERVATION: END-only preflight failed; recorded, continuing campaign", flush=True)

    def check_probe_job(self, st):
        # -2 occurs after output initialization, including after snapshots when
        # cleanup fails. -1 rejects the request BEFORE initialization: do not
        # mistake the previous output buffer for a new observation.
        if st.job_result and not (st.job_result == 1 and
                st.job_lib_rc in (-2, 0xfffffffe) and st.result_len >= 0x40000):
            check(st)
        elif not st.job_result:
            check(st)
        result = {"result": st.job_result, "lib_rc": st.job_lib_rc,
                  "sequence": st.jobs_done}
        if st.job_result:
            print("OBSERVATION: worker returned %08x; retaining its FMSS capture" %
                  (st.job_lib_rc & 0xffffffff), flush=True)
        return result


make_dev_session = Session


# ---- CPU-direct NAND jobs (Cmd.NAND_CPU, target/ipodnano4g/cpu_nand.h): no sequencer program
CPU_NAND_OPS = {'state': 0, 'erase': 1, 'program': 2, 'read_raw': 3, 'read_ecc': 4, 'encode': 5, 'sequence': 6, 'map': 7}
CPU_NAND_FORMATS = {'main8': 0, 'main16': 1, 'rom': 2}
CPU_NAND_DATA, CPU_NAND_IN, CPU_NAND_IN_META = 0x2000, 0x4000, 0x1200
CPU_NAND_STEP_KINDS = {1: 'cmd', 2: 'addr', 3: 'xfer', 4: 'dma', 5: 'latch', 6: 'ready', 7: 'ecc', 8: 'meta', 9: 'fifo', 10: 'begin', 11: 'done', 12: 'status'}
_CPU_HEAD = ('magic', 'version', 'op', 'ce', 'row', 'param', 'rc', 'unused', 'elapsed_us', 'nsteps', 'dropped',
             'nand_status', 'ctrl0', 'bytes', 'nsectors')


def parse_cpu_nand(raw):
    """Output record of one CPU-direct job (layout: struct cpu_nand_out)."""
    w = struct.unpack_from('<%dI' % (0x84 // 4), raw, 0)
    if w[0] != 0x4e555043 or w[1] != 1:
        raise ToolError('CPU NAND output magic/version mismatch: %08x v%d' % (w[0], w[1]))
    rec = dict(zip(_CPU_HEAD, w[:15]))
    rec['rc'] = w[6] - (1 << 32) if w[6] & 0x80000000 else w[6]
    rec['meta_words'] = list(w[15:18])
    rec['pre_reset_rc'], rec['pre'] = w[18], dict(zip(('c08', 'fmc_status', 'ecc_status'), w[19:22]))
    rec['cleanup'] = dict(zip(('fmc_ctrl0', 'fmctrl1', 'fmc_status'), w[22:25]))
    rec['post_reset_rc'] = w[25]
    rec['post'] = dict(zip(('c08', 'c0c', 'fmc_ctrl0', 'fmctrl1', 'fmc_status', 'ecc_status', 'ecc_818'), w[26:33]))
    ecc = []
    for s in range(rec['nsectors']):
        e = struct.unpack_from('<20I', raw, 0x84 + 80 * s)
        ecc.append(dict(r810=e[0], r80c=e[1], r818=e[2], t_us=e[3], loc=list(e[4:12]), par=list(e[12:19]),
                        r810_ack=e[19]))
    rec['ecc'] = ecc
    steps = []
    for k in range(min(rec['nsteps'], 512)):
        tag, value, t_us = struct.unpack_from('<3I', raw, 0x304 + 12 * k)
        steps.append(dict(kind=CPU_NAND_STEP_KINDS.get(tag >> 24 & 0x7f, tag >> 24 & 0x7f), index=tag >> 16 & 0xff,
                          arg=tag & 0xffff, timeout=bool(tag >> 31), value=value, t_us=t_us))
    rec['steps'] = steps
    rec['data'] = bytes(raw[CPU_NAND_DATA:CPU_NAND_DATA + rec['bytes']]) if len(raw) > CPU_NAND_DATA else b''
    return rec


def cpu_nand(s, op, ce=0, row=0, param=0, slot=0, upload=None, timeout=10.0):
    """Run one CPU-direct job; returns (worker status fields, parsed output or None). Never retried."""
    from .protocol import CPU_NAND
    if not getattr(s, 'probe_caps', 0) & CPU_NAND:
        raise ToolError('This kernel lacks CPU-direct NAND jobs; reload the rebuilt image: python3 tools/n4g.py reload')
    if op in ('erase', 'program') and not s.probe_caps & MUTATE:
        raise ToolError('CPU erase/program requires the write-enabled kernel: python3 tools/n4g.py reload')
    if upload is not None:
        st = s.status()
        s.write_memory(st.result_addr + CPU_NAND_IN, bytes(upload))
    word = CPU_NAND_OPS[op] | (ce & 0xf) << 8 | (slot & 0xf) << 12
    print('PLAN: CPU NAND %s CE%d row %d param 0x%x' % (op, ce, row, param), flush=True)
    st = s.job(Cmd.NAND_CPU, word, row, d=param, timeout=timeout)
    job = dict(result=st.job_result, lib_rc=st.job_lib_rc - (1 << 32) if st.job_lib_rc & 0x80000000 else st.job_lib_rc,
               detail=list(st.job_detail), result_len=st.result_len)
    out = parse_cpu_nand(s.read(st.result_addr, st.result_len)) if st.result_len else None
    return job, out


# ---- CPU-only NAND driver (sub-ops 0x10..0x14 of Cmd.NAND_CPU, target/ipodnano4g/cpu_nand_stack.h)
STACK_OPS = {'info': 0x10, 'read': 0x11, 'program': 0x12, 'erase': 0x13, 'build': 0x14}
STACK_ECC = {'raw': 0, 'hw': 1, 'sw': 2}
STACK_FORMATS = {'main': 0, 'boot': 1}
STACK_PATHS = {'dma': 0, 'fifo': 1}
STACK_BCH = {'auto': 0, 8: 1, 16: 2, '8': 1, '16': 2}
STACK_MAGIC, STACK_VERSION = 0x53555043, 1
STACK_STEPS, STACK_DATA, STACK_IN, STACK_IN_META, STACK_RAW, STACK_BYTES = 0x400, 0x2000, 0x4000, 0x1200, 0x6000, 0x7200
STACK_SECTOR_FLAGS = ((1, 'ok'), (2, 'uncorrectable'), (4, 'erased'), (8, 'hw'), (16, 'sw'))
_STACK_HEAD = ('magic', 'version', 'op', 'word', 'ce', 'row', 'unused0', 'unused1',
               'rc', 'unused2', 'strength', 'rawsize', 'elapsed_us', 'nsteps', 'dropped', 'nand_status',
               'ctrl0', 'bytes', 'raw_bytes', 'ecc', 'fmt', 'path', 'max_steps')
_STACK_REP = ('nsectors', 'corrected_total', 'max_corrected', 'uncorrectable_mask', 'erased_mask', 'sw_us')


def stack_word(op, ce=0, ecc='raw', fmt='main', path='dma', bch='auto'):
    """Request word 1 of a CPU-stack job."""
    return (STACK_OPS[op] | (ce & 0xf) << 8 | STACK_ECC[ecc] << 12 | STACK_FORMATS[fmt] << 14
            | STACK_PATHS[path] << 16 | STACK_BCH[bch] << 17)



def unpack_locations(words, count):
    value = sum(w << 32 * k for k, w in enumerate(words))
    return [(value >> 13 * i) & 0x1fff for i in range(count)]


def parse_cpu_stack(raw):
    """Output record of one CPU-stack job (layout: struct cns_out)."""
    w = struct.unpack_from('<%dI' % (0xa0 // 4), raw, 0)
    if w[0] != STACK_MAGIC or w[1] != STACK_VERSION:
        raise ToolError('CPU stack output magic/version mismatch: %08x v%d' % (w[0], w[1]))
    rec = dict(zip(_STACK_HEAD, w[:23]))
    rec['rc'] = w[8] - (1 << 32) if w[8] & 0x80000000 else w[8]
    rec['pre_reset_rc'], rec['pre'] = w[24], dict(zip(('c08', 'fmc_status', 'ecc_status'), w[25:28]))
    rec['cleanup'] = dict(zip(('fmc_ctrl0', 'fmctrl1', 'fmc_status'), w[28:31]))
    rec['post_reset_rc'] = w[31]
    rec['post'] = dict(zip(('c08', 'c0c', 'fmc_ctrl0', 'fmctrl1', 'fmc_status', 'ecc_status', 'ecc_818'), w[32:39]))
    r = struct.unpack_from('<9I', raw, 0xa0)
    rec.update(zip(_STACK_REP, r[:6]))
    rec['meta_words'] = list(r[6:9])
    rec['meta'] = b''.join(v.to_bytes(4, 'little') for v in r[6:9])[:10]
    sectors = []
    for s in range(min(rec['nsectors'], 8)):
        e = struct.unpack_from('<13I', raw, 0xc4 + 52 * s)
        sectors.append(dict(flags=[name for bit, name in STACK_SECTOR_FLAGS if e[0] & bit], corrected=e[1],
                            r810=e[2], r80c=e[3], r818=e[4], t_us=e[5],
                            locations=unpack_locations(e[6:13], e[1] if e[0] & 1 else 0)))
    rec['sectors'] = sectors
    steps = []
    for k in range(min(rec['nsteps'], 256)) if len(raw) > STACK_STEPS else ():
        tag, value, t_us = struct.unpack_from('<3I', raw, STACK_STEPS + 12 * k)
        steps.append(dict(kind=CPU_NAND_STEP_KINDS.get(tag >> 24 & 0x7f, tag >> 24 & 0x7f), index=tag >> 16 & 0xff,
                          arg=tag & 0xffff, timeout=bool(tag >> 31), value=value, t_us=t_us))
    rec['steps'] = steps
    rec['data'] = bytes(raw[STACK_DATA:STACK_DATA + rec['bytes']]) if len(raw) >= STACK_BYTES else b''
    rec['raw'] = bytes(raw[STACK_RAW:STACK_RAW + rec['raw_bytes']]) if len(raw) >= STACK_BYTES else b''
    return rec


def stack_upload(fmt='main', data=None, meta=None, raw=None):
    """Input image at STACK_IN: raw page (ecc raw) or data + metadata (hw/sw encode)."""
    buf = bytearray(b'\xff' * 0x1400)
    if raw is not None:
        if len(raw) > 0x1200:
            raise ToolError('raw page image exceeds 4608 bytes')
        buf[:len(raw)] = raw
        return bytes(buf)
    size = 0x600 if fmt == 'boot' else 4096
    data = bytes(data or b'')
    if len(data) > size:
        raise ToolError('%s-format data is %d bytes at most' % (fmt, size))
    buf[:len(data)] = data
    meta = b'\xff' * 10 if meta is None else bytes(meta)
    if len(meta) != 10:
        raise ToolError('metadata must be 10 bytes')
    buf[STACK_IN_META:STACK_IN_META + 10] = meta
    return bytes(buf)


def cpu_stack(s, op, ce=0, row=0, ecc='raw', fmt='main', path='dma', bch='auto',
              data=None, meta=None, raw=None, word=None, timeout=20.0, header_only=False):
    """Run one CPU-stack job; returns (worker status fields, parsed output or None). Never retried.

    word overrides request word 1 (argument-check tests)."""
    from .protocol import CPU_NAND
    if not getattr(s, 'probe_caps', 0) & CPU_NAND:
        raise ToolError('This kernel lacks CPU-direct NAND jobs; reload the rebuilt image: python3 tools/n4g.py reload')
    if op == 'program' or (op == 'build' and (data is not None or raw is not None)):
        st = s.status()
        s.write_memory(st.result_addr + STACK_IN, stack_upload(fmt, data, meta, raw))
    w1 = stack_word(op, ce, ecc, fmt, path, bch) if word is None else word
    print('PLAN: CPU stack %s CE%d row %d ecc %s format %s path %s bch %s' %
          (op, ce, row, ecc, fmt, path, bch), flush=True)
    st = s.job(Cmd.NAND_CPU, w1, row, d=0, timeout=timeout)
    job = dict(result=st.job_result, lib_rc=st.job_lib_rc - (1 << 32) if st.job_lib_rc & 0x80000000 else st.job_lib_rc,
               detail=list(st.job_detail), result_len=st.result_len)
    # header_only: record and per-sector report (no step log, data or raw page): device-wide scans
    size = min(st.result_len, STACK_STEPS) if header_only else st.result_len
    out = parse_cpu_stack(s.read(st.result_addr, size)) if st.result_len else None
    return job, out


def cpu_stack_info(s, bch='auto'):
    """The stack's info record, or None on a kernel without the CPU-only driver (no NAND access)."""
    job, out = cpu_stack(s, 'info', bch=bch)
    return out if job['result'] == 0 and out else None


def stack_prepare(s):
    """Run the read-only prerequisite steps (gates, pins, FIL) that have not succeeded yet."""
    st = s.status()
    for n in (1, 2, 4):
        if not st.step_ok_mask >> n & 1:
            check(s.job(Cmd.NAND_STEP, n, 0))
    return s.status()
