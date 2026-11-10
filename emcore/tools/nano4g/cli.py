"""Host interface for the nano4g M2 probe. Physical results are saved per probe."""
import argparse
from datetime import datetime, timezone
import hashlib
import gzip
import json
from pathlib import Path
import sys
import time

from fmss import probe as P
from fmss.target import NANO4G as T
from .experiments import entries, resolve, SMOKE, campaign_order
from .nand import make_dev_session, check, format_status
from .protocol import Cmd, ToolError
from .capture import compact, save, rebuild


def parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 epilog="Other commands: n4g.py image --help; n4g.py reload --help")
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    step = sub.add_parser("step")
    step.add_argument("number", type=int, choices=(1, 2, 3, 4))
    page = sub.add_parser("read-page")
    page.add_argument("bank", type=int)
    page.add_argument("row", type=lambda x: int(x, 0))
    page.add_argument("-o", "--output", required=True)
    fmss = sub.add_parser("fmss")
    commands = fmss.add_subparsers(dest="action", required=True)
    ex = commands.add_parser("explore")
    ex.add_argument("--list", action="store_true")
    ex.add_argument("--only", default=",".join(SMOKE))
    ex.add_argument("--exclude", default="", help="comma-separated families to omit from the selected campaign")
    execution = ex.add_mutually_exclusive_group()
    execution.add_argument("--single-probes", action="store_true", help="use the diagnostic one-case-per-USB-job path for every family")
    execution.add_argument("--isolate-diagnostics", action="store_true", help="collect m2_wedge and m2_regbus one case at a time; other families remain resident")
    ex.add_argument("--opcodes", action="store_true", help="recheck known ISA, M1 opcode hypotheses and the full opcode-byte sweep")
    ex.add_argument("--registers", action="store_true", help="verify M1 register-role predictions on M2 in resident batches, with PC bus probes last")
    ex.add_argument("--followups", action="store_true", help="run the additional M1/M2 comparison matrices")
    ex.add_argument("--all", action="store_true", help="run the complete migration registry, including flash writes")
    ex.add_argument("--new", action="store_true",
                    help="only families that can produce new information: never captured, settling an unresolved "
                         "M1/M2 finding, or deciding an open ledger conflict (nano4g/campaign_status.py)")
    ex.add_argument("--cold", action="store_true",
                    help="power-on captures only (m2_cold_boot, m2_m1_poweron_values): run first after a power cycle and DFU")
    ex.add_argument('--observation-min-us', type=int, help='minimum observation window; full/opcode/followup campaigns default to 160000 us, longer than the measured idle fault timeout; 0 preserves original windows')
    ex.add_argument('--fault-margin', type=int, help='shorten the ~126.5 ms hardware timeout for calibrated cases to margin x the '
                    'longest non-timeout event p16 observed in that family (see nano4g/fault_window.py); campaigns default to 8, '
                    'other runs to 0; 0 keeps the hardware default everywhere')
    ex.add_argument('--fault-floor-us', type=int, default=16000, help='shortest calibrated fault window (default 16000 us)')
    ex.add_argument('--checkpoint-seconds', type=float, default=300,
                    help='rewrite the readable summary at most this often (default 300 s; 0 = after every family); '
                         'the raw journal is appended and flushed after every family regardless')
    ex.add_argument('--reconnect-seconds', type=float, default=120,
                    help='after a USB loss, wait this long for emCORE to answer again and continue with the next '
                         'family (the lost family is never replayed; 0 = record the rest as not submitted)')
    ex.add_argument("--from", dest="start_at", help="begin the selected campaign at this experiment (e.g. op12)")
    ex.add_argument("--scratch-block", type=int, default=3000, help="scratch block for NAND workloads (default: 3000)")
    ex.add_argument('--driver-pages', type=int, default=32, help='driver comparison: pages per physical backend')
    ex.add_argument('--driver-repetitions', type=int, default=4, help='driver comparison: sequential read passes')
    ex.add_argument("--scratch-ce", type=int, default=0, help="chip enable for NAND workloads (default: 0)")
    ex.add_argument("--bch", type=int, choices=(8, 16), default=8, help="ECC test configuration, not detected chip geometry (default: 8)")
    ex.add_argument("--json", help="summary path (.json.gz recommended); adjacent .jsonl.gz preserves raw requests/results")
    ex.add_argument("--image", help="exact unsigned image used on this boot, for provenance")
    run = commands.add_parser("run")
    run.add_argument("source")
    run.add_argument("--json")
    run.add_argument("--image")
    run.add_argument("--param", action="append", default=[])
    run.add_argument("--timeout-us", type=int, default=100000)
    run.add_argument("--allow-nand", action="store_true")
    run.add_argument("--allow-unknown", action="store_true")
    run.add_argument("--allow-write", action="store_true", help="permit NAND program/erase commands in custom microcode")
    run.add_argument("--discovery", action="store_true", help="permit exploratory opcodes/register accesses")
    rb = commands.add_parser("rebuild", help="rebuild a readable summary from a raw .jsonl.gz journal (no device)")
    rb.add_argument("journal")
    rb.add_argument("--json", required=True)
    cn = sub.add_parser("cpu-nand", help="CPU-only NAND driver: no FMSS sequencer program (cpu_nand_stack.c)",
                        description="Read, program or erase one page/block of any CE through the CPU-only driver.")
    cn.add_argument("action", choices=("info", "read", "write", "erase"))
    cn.add_argument("--ce", type=int, default=0)
    cn.add_argument("--block", type=lambda x: int(x, 0))
    cn.add_argument("--page", type=lambda x: int(x, 0), default=0)
    cn.add_argument("--ecc", choices=("hw", "sw", "raw"), default="hw",
                    help="hw: the ECC engine kicked by the CPU; sw: software BCH, no ECC engine; raw: no ECC (default hw)")
    cn.add_argument("--format", choices=("main", "boot"), default="main",
                    help="main: Whimory page (meta[10], BCH8/BCH16); boot: boot-ROM reserved-region page (3 x BCH16)")
    cn.add_argument("--path", choices=("dma", "fifo"), default="dma", help="raw transfer path for --ecc raw/sw reads")
    cn.add_argument("--bch", choices=("auto", "8", "16"), default="auto",
                    help="main-format code; auto = the firmware's current choice (BCH8 until a BCH16 page decoded)")
    cn.add_argument("--out", help="read: data (4096 / 0x600 bytes), or the raw page for --ecc raw")
    cn.add_argument("--raw-out", help="read: also save the raw page (--ecc raw/sw)")
    cn.add_argument("--in", dest="input", help="write: data (--ecc hw/sw; shorter input is padded with 0xff) or the "
                                               "raw page image (--ecc raw)")
    cn.add_argument("--meta", help="write, main format: 10 metadata bytes as hex (default ff x 10)")
    cn.add_argument("--json", help="save the parsed job record (without data bytes) as JSON")
    suite = sub.add_parser("cpu-suite",
                           help="on-device CPU NAND read/write test: bank 1, one page chosen at run time")
    suite.add_argument("--backup-dir", help="directory for the block backup (default: nand_device_suite/)")
    suite.add_argument("--restore", help="program this cpu-suite backup back and check it with the FMSS reader")
    for command in (sub.choices['status'], step, page, ex, run, cn, suite):
        command.add_argument('--usb', choices=('auto', 'bulk', 'control'), default='auto',
                             help='bulk on updated kernels; control is the slow diagnostic path')
    return ap


def finish_timing(meta, started, checkpoint_seconds):
    """Campaign wall time and per-phase totals; host clock, recorded in the summary."""
    timing = meta.setdefault('timing', {'families': {}})
    timing['finished_utc'] = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    timing['wall_seconds'] = round(time.monotonic() - started, 3)
    if checkpoint_seconds is not None:
        timing['checkpoint_seconds'] = round(checkpoint_seconds, 3)
    totals = {}
    for family in timing.get('families', {}).values():
        for key, value in family.items():
            if key.endswith('_seconds') and isinstance(value, (int, float)):
                totals[key] = round(totals.get(key, 0) + value, 3)
    timing['family_totals'] = totals
    if totals:
        print('TIMING campaign: wall %.1fs; plan %.1fs, prepare %.1fs, upload %.1fs, run/wait %.1fs '
              '(device worker %.1fs, programs %.1fs), download %.1fs, decode %.1fs, report %.1fs, store %.1fs, '
              'checkpoints %.1fs' % (timing['wall_seconds'], *(totals.get(k, 0) for k in (
                  'plan_seconds', 'prepare_seconds', 'upload_seconds', 'command_seconds', 'worker_seconds',
                  'probe_seconds', 'download_seconds', 'processing_seconds', 'report_seconds', 'store_seconds',
                  'checkpoint_seconds'))), flush=True)


def preflight_notice(image, selected=None):
    """Report a preflight JSON file for this image, when one is present."""
    report = Path(__file__).resolve().parents[2] / 'build/m2-preflight.json'
    try:
        r = json.loads(report.read_text())
    except (OSError, ValueError):
        print('PREFLIGHT: no report (%s)' % report, flush=True)
        return
    sha = hashlib.sha256(Path(image).read_bytes()).hexdigest() if image else None
    wanted = {e['name'] for e in selected or ()}
    covered = set(r.get('families', {}))
    scoped = bool(wanted) and wanted <= covered and not r.get('problems')
    stale = (sha and r.get('image_sha256') != sha) or not (r.get('complete') or scoped)
    print('PREFLIGHT: %d problems, %d device-only cases%s' % (
        len(r.get('problems', ())), sum(len(v) for v in r.get('unmodelled', {}).values()),
        '; STALE (other image or partial run)' if stale else ''), flush=True)
    for name, native in r.get('function_family_native', {}).items():
        if wanted and name not in wanted:
            continue
        profile = native.get('model_profile', 'measured')
        if profile != 'measured':
            print('PREFLIGHT: %s uses %s; optional NAND support and physical timing remain device-dependent' %
                  (name, profile), flush=True)


RECONNECT_ATTRS = ('geometry', 'batch_enabled', 'isolate_diagnostics', 'observation_min_us', 'fault_margin',
                   'fault_floor_us', 'journal', 'checkpoint_cases', 'engine_held',
                   'checkpoint_diagnostics', 'gate_checkpoints')


def reconnect(old, a, meta):
    """USB came back after a loss: a new session with the same campaign settings, once the worker is idle.

    The family whose command was lost is not replayed. The kernel must report
    the same capabilities; otherwise the old session stays unavailable."""
    deadline = time.monotonic() + a.reconnect_seconds
    last = None
    while time.monotonic() < deadline:
        time.sleep(2)
        try:
            new = make_dev_session(usb=a.usb)
            if new.probe_caps != old.probe_caps:
                raise ToolError('capabilities changed after reconnect (%#x -> %#x): kernel restarted'
                                % (old.probe_caps, new.probe_caps))
            while new.status().busy and time.monotonic() < deadline:
                time.sleep(1)
            if new.status().busy:
                raise ToolError('worker still busy')
            for name in RECONNECT_ATTRS:
                if hasattr(old, name):
                    setattr(new, name, getattr(old, name))
            meta.setdefault('reconnects', []).append({'at_utc': datetime.now(timezone.utc).isoformat(), 'ok': True})
            new.log({'type': 'transport-reconnected'})
            print('RECONNECTED: continuing with the next experiment', flush=True)
            return new
        except Exception as exc:  # device absent or not answering yet
            last = exc
    meta.setdefault('reconnects', []).append({'at_utc': datetime.now(timezone.utc).isoformat(), 'ok': False,
                                              'message': str(last)})
    print('RECONNECT FAILED: %s' % last, flush=True)
    old.reconnect_failed = True
    return old


def mutating_families(s, selected, flag=None):
    """Follow-up families whose generated cases request NAND mutation (or another probe flag)."""
    from types import SimpleNamespace
    from . import followup_experiments as F, sequences as Q
    from .protocol import MUTATE
    from .experiments import NANO4G
    flag = MUTATE if flag is None else flag
    names = [e['name'] for e in selected
             if e.get('source') == 'nano4g/followup_experiments.py' and e['name'] in F.SPECS]
    if not names or not hasattr(s, 'status'):
        return []
    status = s.status()
    view = SimpleNamespace(status=lambda: status, geometry=Q.geometry(s))
    out = []
    for e in selected:
        if e.get('source') != 'nano4g/followup_experiments.py' or e['name'] not in F.SPECS:
            continue
        try:
            if any(c.options.get('flags', 0) & flag for c in F.cases(e['name'], view, NANO4G)):
                out.append(e['name'])
        except Exception:
            continue
    return out


def cpu_nand_command(s, a):
    """n4g.py cpu-nand info|read|write|erase: one CPU-only driver job (no sequencer program)."""
    from .nand import cpu_stack, cpu_stack_info, stack_prepare
    from .protocol import MUTATE
    if a.action != 'info' and a.block is None:
        raise ToolError('--block is required')
    if not 0 <= a.ce < 8 or a.page < 0:
        raise ToolError('invalid --ce or --page')
    if a.action in ('write', 'erase') and not s.probe_caps & MUTATE:
        raise ToolError('CPU program/erase requires the write-enabled kernel: python3 tools/n4g.py reload')
    if a.action == 'read' and a.ecc == 'hw' and a.path == 'fifo':
        raise ToolError('--ecc hw decodes on the DMA path; use --path fifo with --ecc sw or raw')
    if a.action == 'erase' and a.page:
        raise ToolError('erase acts on a whole block; omit --page')
    if a.action != 'info':
        stack_prepare(s)
    info = cpu_stack_info(s, a.bch)
    if info is None:
        raise ToolError('This kernel lacks the CPU-only NAND driver; load the rebuilt image: python3 tools/n4g.py reload')
    if a.action == 'info':
        print('CPU stack v%d: strength %s, raw page %d bytes, %d step records' % (
            info['version'], info['strength'] or 'unknown (run step 4)', info['rawsize'], info['max_steps']))
        return 0
    pages = s.status().geom[0] or 128      # pages per block from the FIL step
    row = a.block * pages + a.page
    data = meta = raw = None
    op = {'read': 'read', 'write': 'program', 'erase': 'erase'}[a.action]
    ecc = 'raw' if op == 'erase' else a.ecc
    if op == 'program':
        if not a.input:
            raise ToolError('write needs --in')
        blob = Path(a.input).read_bytes()
        if ecc == 'raw':
            if len(blob) > info['rawsize']:
                raise ToolError('raw image is %d bytes; the raw page is %d' % (len(blob), info['rawsize']))
            raw = blob
        else:
            data = blob
            if a.meta is not None:
                if a.format != 'main':
                    raise ToolError('--meta applies to the main format only')
                meta = bytes.fromhex(a.meta)
                if len(meta) != 10:
                    raise ToolError('--meta must be 10 bytes (20 hex digits)')
    job, out = cpu_stack(s, op, a.ce, row, ecc, 'main' if op == 'erase' else a.format,
                         a.path if op == 'read' else 'dma', a.bch, data, meta, raw)
    if job['result'] and out is None:
        raise ToolError('job result %d (lib rc %d), nothing recorded' % (job['result'], job['lib_rc']))
    print('CPU stack %s CE%d block %d page %d (row %d): rc %d, %d us, chip status 0x%02x, idle after reset: %s' % (
        a.action, a.ce, a.block, a.page, row, out['rc'], out['elapsed_us'], out['nand_status'],
        'yes' if out['post_reset_rc'] == 0 and out['post']['c08'] == 0 and out['post']['fmc_ctrl0'] == 0 else 'NO'))
    for i, e in enumerate(out['sectors']):
        status = next((f for f in ('erased', 'uncorrectable', 'ok') if f in e['flags']), 'none')
        print('  sector %d: %s%s%s' % (i, status, ', %d bits corrected' % e['corrected'] if e['corrected'] else '',
                                     ' at %s' % e['locations'] if e['locations'] else ''))
    if op == 'read' and a.format == 'main' and ecc != 'raw':
        print('  meta %s' % out['meta'].hex())
    if out['sw_us']:
        print('  software BCH %d us' % out['sw_us'])
    if a.out and op == 'read':
        Path(a.out).write_bytes(out['raw'] if ecc == 'raw' else out['data'])
        print('Saved ' + a.out)
    if a.raw_out and out['raw']:
        Path(a.raw_out).write_bytes(out['raw'])
        print('Saved ' + a.raw_out)
    if a.json:
        rec = {k: (v.hex() if isinstance(v, bytes) and k == 'meta' else v) for k, v in out.items()
               if not isinstance(v, bytes) or k == 'meta'}
        rec.update(job=job, data_sha256=hashlib.sha256(out['data']).hexdigest() if out['data'] else None,
                   raw_sha256=hashlib.sha256(out['raw']).hexdigest() if out['raw'] else None)
        Path(a.json).write_text(json.dumps(rec, indent=1) + '\n')
    if out['rc']:
        return 1
    return 2 if out['uncorrectable_mask'] else 0


def main(argv=None):
    a = parser().parse_args(argv)
    if a.command == "fmss" and a.action == "explore" and a.list:
        for e in entries(include_followups=True):
            print("%-12s %-17s %s" % (e["name"], e["status"], e["reason"]))
        return 0
    if a.command == "fmss" and a.action == "rebuild":
        out = Path(a.json)
        if out.exists():
            print("ERROR: refusing to overwrite " + str(out), file=sys.stderr)
            return 1
        save(out, rebuild(a.journal), level=6)
        print("Rebuilt " + str(out))
        return 0
    campaign_started = time.monotonic()
    s = None
    journal = None
    meta, path = None, None
    try:
        selected = None
        if a.command == "fmss" and a.action == "explore":
            if a.registers and (a.all or a.followups or a.opcodes):
                raise ToolError('--registers selects its complete comparison campaign; use it without --all/--followups/--opcodes')
            if a.observation_min_us is not None and not 0 <= a.observation_min_us <= 2000000:
                raise ToolError('--observation-min-us must be 0..2000000')
            if (a.fault_margin is not None and not 0 <= a.fault_margin <= 1000) or not 4000 <= a.fault_floor_us <= 63000:
                raise ToolError('--fault-margin must be 0..1000 and --fault-floor-us 4000..63000')
            if a.cold:
                from .cold_experiments import COLD
                a.only = ','.join(COLD)
            selected = resolve([e["name"] for e in entries(include_followups=True)
                                if a.all or (a.opcodes and e["name"].startswith("m2_opcode_"))
                                or (a.registers and (e['name']=='regnum' or e['name'].startswith('m2_reg')))
                                or (a.followups and e["status"] == "followup")]
                               if a.all or a.followups or a.opcodes or a.registers or a.new else a.only.split(","))
            if a.new:
                from .campaign_status import shipped_focus
                from .cold_experiments import REBOOT_ONLY, COLD
                pool = [e for e in entries(include_followups=True) if e['name'] not in REBOOT_ONLY + COLD]
                why = shipped_focus([e['name'] for e in pool])
                selected = campaign_order([e for e in pool if e['name'] in why])
                for e in selected:
                    print('NEW: %-28s %s' % (e['name'], why[e['name']]), flush=True)
            if a.all:
                from .cold_experiments import REBOOT_ONLY
                selected = campaign_order([e for e in selected if e['name'] not in REBOOT_ONLY])
            if a.exclude:
                omitted = {e['name'] for e in resolve(a.exclude.split(','))}
                selected = [e for e in selected if e['name'] not in omitted]
                print('EXCLUDED: ' + ', '.join(sorted(omitted)), flush=True)
            if a.all or a.new:
                preflight_notice(a.image, selected)
            if a.start_at:
                names = [e["name"] for e in selected]
                if a.start_at not in names:
                    raise ToolError("--from experiment is not in the selected campaign: " + a.start_at)
                selected = selected[names.index(a.start_at):]
        if a.command == "fmss":
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            from .capture import journal_path
            path = Path(a.json) if a.json else Path("experiments") / ("m2-" + stamp + ".json.gz")
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists():
                raise ToolError("Refusing to overwrite an existing result: " + str(path))
            journal = gzip.open(journal_path(path), "xt", compresslevel=6)
            image_sha = hashlib.sha256(Path(a.image).read_bytes()).hexdigest() if a.image else None
            meta = {"target": "nano4g", "soc": "S5L8720", "evidence": "device",
                    "started_utc": stamp, "image_sha256": image_sha,
                    "image_identity": "host-supplied" if image_sha else "not recorded",
                    "probe_version": T.need_version, "capture_format": "compact-v1",
                    "journal": str(journal_path(path)), "results": {}, "errors": {}, "complete": False}
            if selected is not None:
                from .sequences import Geometry
                geometry = Geometry(ce=a.scratch_ce, block=a.scratch_block, strength=a.bch)
                meta["geometry"] = vars(geometry)
                meta["geometry_origin"] = "host-selected NAND workload parameters; not detected geometry or M2 architectural limits"
                meta["migration_scope"] = {e["name"]: e["reason"] for e in selected}
                if a.registers:
                    from .register_experiments import M1_REFERENCE
                    meta['comparison_baseline'] = M1_REFERENCE
                    meta['comparison_scope'] = 'M1 register-role predictions plus explicit M2 source-10 candidate; each case records its basis; physical results qualify only tested operands/states'
            save(path, meta)
        s = make_dev_session(usb=a.usb)
        transport = getattr(s, 'usb_transport', 'model')
        print('USB transport: ' + transport + (' (64-byte writes / 512-byte reads; reload updated kernel for bulk)' if transport == 'control' else ''), flush=True)
        if meta is not None: meta['usb_transport'] = transport
        if selected is not None and a.usb != 'control' and transport == 'control':
            raise ToolError('Bulk experiment transfers require the updated kernel. Run: python3 tools/n4g.py reload. Use --usb control only for an explicit slow-path comparison.')
        if selected is not None:
            s.geometry = geometry
            s.driver_compare_pages = a.driver_pages
            s.driver_compare_repetitions = a.driver_repetitions
            s.batch_enabled = not a.single_probes
            s.isolate_diagnostics = a.isolate_diagnostics
            s.observation_min_us = (a.observation_min_us if a.observation_min_us is not None else
                                    160000 if a.all or a.new or a.opcodes or a.followups else 0)
            if a.single_probes and s.observation_min_us:
                raise ToolError('Observation-window override requires resident execution; use --observation-min-us 0 with --single-probes')
            meta['observation_min_us'] = s.observation_min_us
            s.fault_margin = (a.fault_margin if a.fault_margin is not None else
                              8 if (a.all or a.new or a.opcodes or a.followups) and not a.single_probes else 0)
            s.fault_floor_us = a.fault_floor_us
            if s.fault_margin and a.single_probes:
                raise ToolError('Calibrated fault windows require resident execution; use --fault-margin 0 with --single-probes')
            from .fault_window import load as load_windows
            calibration = load_windows() if s.fault_margin else None
            if s.fault_margin and calibration is None:
                raise ToolError('fault-window calibration table missing; use --fault-margin 0')
            meta['fault_window'] = {'margin': s.fault_margin, 'floor_us': s.fault_floor_us,
                                    'calibration': calibration['source'] if calibration else None,
                                    'late_wait2': calibration['late_wait2'] if calibration else None,
                                    'scope': 'per-case record fault_window marks every shortened case and long control'}
            from .protocol import BATCH, BATCH2, RECOVERY, RESET_PHASES, STATE_B_RESET_HELD, MUTATE
            if s.batch_enabled and not s.probe_caps & BATCH2:
                raise ToolError('Resident experiments require the updated kernel. Run: python3 tools/n4g.py reload. Use --single-probes only to explicitly request the slower diagnostic path.')
            if s.batch_enabled and transport != 'model' and not s.probe_caps & RESET_PHASES:
                raise ToolError('This kernel predates peripheral recovery and reset-phase diagnostics. Run: python3 tools/n4g.py reload')
            if transport != 'model' and not s.probe_caps & STATE_B_RESET_HELD:
                raise ToolError('This kernel lacks the p14 reset-while-held correction. Reload the rebuilt image before experiments: python3 tools/n4g.py reload')
            if a.all and not s.probe_caps & MUTATE:
                raise ToolError('--all includes writes; reload the normal kernel first so those cases can execute: python3 tools/n4g.py reload')
            if selected and not s.probe_caps & MUTATE:
                # p28/p29: families with NAND writes ran on a DFU-entered kernel
                # and every erase/program case was refused. Refuse up front.
                needs = mutating_families(s, selected)
                if needs:
                    raise ToolError('%s erase/program NAND, but this kernel refuses writes after DFU '
                                    '(image before the p35 gate removal). Run: python3 tools/n4g.py reload, then re-run.' % ', '.join(needs))
            from .protocol import VICSOFT, STALL_SAFE
            if selected and transport != 'model' and not s.probe_caps & STALL_SAFE:
                needs = mutating_families(s, selected, STALL_SAFE)
                if needs:
                    raise ToolError('%s use stall-safe cases; this kernel predates that probe option. '
                                    'Load the rebuilt image: python3 tools/n4g.py reload.' % ', '.join(needs))
            if selected and transport != 'model' and not s.probe_caps & VICSOFT:
                needs = mutating_families(s, selected, VICSOFT)
                if needs:
                    raise ToolError('%s raise VIC soft interrupts; this kernel predates that probe option. '
                                    'Load the rebuilt image: python3 tools/n4g.py reload.' % ', '.join(needs))
            from .protocol import SAMPLER, GATEWIN, EIC, PADS, CAP_SAMPLER, CAP_STIMULI
            for flag, cap, what in ((SAMPLER, CAP_SAMPLER, 'use the CPU sampler'),
                                    (GATEWIN | EIC | PADS, CAP_STIMULI, 'use the gate-window, EIC or pad options')):
                if selected and transport != 'model' and not s.probe_caps & cap:
                    needs = mutating_families(s, selected, flag)
                    if needs:
                        raise ToolError('%s %s; this kernel predates them. Load the rebuilt image: '
                                        'python3 tools/n4g.py reload.' % (', '.join(needs), what))
            from .followup_experiments import isolates_cases
            meta['execution_mode'] = ('resident batches with explicit diagnostic isolation' if a.isolate_diagnostics else 'resident batches') if s.batch_enabled else 'single probes (explicit override)'
            meta['experiment_transport'] = {e['name']: ('individual native requests (diagnostic)' if isolates_cases(s, e['name']) else 'resident batch') if s.batch_enabled else 'single probes' for e in selected}
            print('Execution: ' + meta['execution_mode'], flush=True)


        if a.command == "status":
            print(format_status(s.status()))
            from .protocol import MUTATE, RELOADED
            print("M2 probe v%d, %s, %s" % (s.probe_version,
                  "writes enabled" if s.probe_caps & MUTATE else "read-only (older kernel after DFU)",
                  "entered by reload" if s.probe_caps & RELOADED else "entered by DFU"))
        elif a.command == "step":
            print(format_status(check(s.job(Cmd.NAND_STEP, a.number, 0))))
        elif a.command == "cpu-nand":
            return cpu_nand_command(s, a)
        elif a.command == "cpu-suite":
            from .cpu_suite import command
            return command(s, a.backup_dir, a.restore)
        elif a.command == "read-page":
            if not 0 <= a.bank < 8 or a.row < 0:
                raise ToolError("Invalid bank or row")
            st = check(s.job(Cmd.NAND_READ_PAGE, a.bank, a.row), (0, 1))
            Path(a.output).write_bytes(s.read(st.result_addr, st.result_len))
        else:
            s.journal = journal
            timing = meta.setdefault('timing', {'clock': 'host time.monotonic seconds; device times are '
                                               'worker/probe microsecond counters', 'families': {}})
            timing['setup_seconds'] = round(time.monotonic() - campaign_started, 4)
            last_save = [time.monotonic()]
            checkpoint_total = [0.0]
            def checkpoint(force=False, level=1):
                now = time.monotonic()
                if not force and now - last_save[0] < a.checkpoint_seconds:
                    return 0.0
                save(path, meta, level=level)
                last_save[0] = time.monotonic()
                checkpoint_total[0] += last_save[0] - now
                return last_save[0] - now
            def checkpoint_cases(name, records):
                meta['results'][name] = {'summary': 'Isolation in progress', 'partial': True,
                    'data_format': 'native-case-records', 'data': compact(P.jsonable(records)),
                    'outstanding_request': getattr(s, 'isolation_active', None)}
                # Explicit diagnostic isolation keeps its per-case readable checkpoint.
                checkpoint(force=True)
            s.checkpoint_cases = checkpoint_cases
            def checkpoint_diagnostics():
                meta['gate_checkpoints'] = s.gate_checkpoints
                meta['pending_job'] = s.pending_job
                checkpoint(force=True)
            s.checkpoint_diagnostics = checkpoint_diagnostics
            if a.cold:
                st = s.status()
                meta['cold_boot'] = {'jobs_before': st.jobs_started, 'step_ok_mask': st.step_ok_mask,
                                     'valid': st.jobs_started == 0}
                print('COLD: %d worker jobs ran before this capture in this boot%s' % (
                    st.jobs_started, '' if st.jobs_started == 0 else ' -- NOT a power-on capture; power-cycle, DFU, '
                    'then run --cold before anything else'), flush=True)
            meta["reported_probe_version"] = s.probe_version
            meta["reported_probe_caps"] = s.probe_caps
            meta['running_firmware'] = s.firmware_identity()
            s.log({"type": "session", **meta})
            if selected is None:
                code = P.assemble(T, Path(a.source).read_text())
                P.lint(T, code, a.allow_nand, False, a.allow_unknown)
                params = [tuple(int(v, 0) for v in p.split("=")) for p in a.param]
                from .protocol import MUTATE, DISCOVERY
                flags = (MUTATE if a.allow_write else 0) | (DISCOVERY if a.discovery else 0)
                data = P.probe(s, T, code, params=params, timeout_us=a.timeout_us, flags=flags)
                meta["results"]["custom"] = P.jsonable(data)
                print(P.describe(data))
            else:
                for e in selected:
                    if getattr(s, 'transport_unavailable', False) and a.reconnect_seconds and \
                            not getattr(s, 'reconnect_failed', False):
                        s = reconnect(s, a, meta)
                    if getattr(s, 'transport_unavailable', False):
                        meta.setdefault('not_submitted', []).append(e['name'])
                        continue
                    if getattr(s, 'engine_held', None):
                        # p26/p27: a held engine survives the reload; every further
                        # case would fail its initial reset. Record, do not submit.
                        meta["errors"][e["name"]] = {"error_type": "EngineHeld", "message": s.engine_held}
                        s.log({"type": "experiment-not-submitted", "experiment": e["name"], "reason": s.engine_held})
                        print("NOT SUBMITTED %s: %s" % (e["name"], s.engine_held), flush=True)
                        continue
                    P.set_experiment(e["name"])
                    print("PLAN: experiment " + e["name"], flush=True)
                    s.last_batch_records = None
                    s.last_batch_compact = None
                    s.last_batch_health = None
                    s.family_timing = family_timing = {}
                    family_started = time.monotonic()
                    try:
                        data, summary = e["fn"](s, T)
                        family_timing['experiment_seconds'] = round(time.monotonic() - family_started, 4)
                        store_started = time.monotonic()
                        from fmss.experiments.registry import RESULTS
                        # Only optime reads another family's report (opsweep).
                        # Retaining every report would duplicate all captures in memory.
                        RESULTS[e["name"]] = {"data": data if e["name"] == "opsweep" else None, "summary": summary}
                        # Keep every native capture, including setup probes and
                        # duplicate legacy report labels, without duplicating
                        # all snapshots into both native and report views.
                        captures = s.last_batch_records if s.last_batch_records is not None else data
                        cache = s.last_batch_compact if s.last_batch_records is not None else None
                        stored = ({k: cache[k] if k in cache else compact(P.jsonable(v)) for k, v in captures.items()}
                                  if cache is not None and isinstance(captures, dict) else compact(P.jsonable(captures)))
                        meta["results"][e["name"]] = {"summary": summary, "data": stored,
                            "data_format": 'native-case-records' if s.last_batch_records is not None else 'legacy-report'}
                        if s.last_batch_health:
                            meta["results"][e["name"]]["reset_health"] = s.last_batch_health
                            health = s.last_batch_health
                            if health.get('cases') and not health.get('verified') and \
                                    len(health.get('initial_reset_failed', ())) == health['cases']:
                                s.engine_held = ('every initial reset of %s failed; the FMSS engine is held. '
                                                 'Reboot the device (SoC reset) and reload before re-running.' % e["name"])
                                print("ENGINE HELD: " + s.engine_held, flush=True)
                        if isinstance(captures, dict):
                            case_errors = {k: v for k, v in captures.items()
                                           if isinstance(v, dict) and ("case_error" in v or 'execution_error' in v or
                                               v.get('assessment', {}).get('status') in ('mismatch', 'inconclusive'))}
                            if case_errors:
                                meta["errors"][e["name"]] = {"cases": {k: {f: v[f] for f in
                                    ('case_error','execution_error','message','assessment') if f in v}
                                    for k, v in case_errors.items()}}
                        print(summary, flush=True)
                        family_timing['store_seconds'] = round(time.monotonic() - store_started, 4)
                    except Exception as exc:
                        # Failure is evidence about this entry, not a reason to
                        # abandon independent M1/M2 comparisons. Never replay
                        # the failed command; attempt the next experiment once.
                        error = {"error_type": type(exc).__name__, "message": str(exc)}
                        if getattr(s, 'pending_job', None):
                            error['pending_job'] = s.pending_job
                        if getattr(s, 'isolation_active', None):
                            error['outstanding_request'] = s.isolation_active
                        if isinstance(exc, OSError):
                            s.transport_unavailable = True
                        if getattr(s, 'last_job_status', None) is not None:
                            error['last_job_status'] = s.last_job_status
                        if s.last_batch_records:
                            meta['results'][e['name']] = {
                                'summary': 'Partial resident capture; experiment did not finish',
                                'data_format': 'native-case-records', 'partial': True,
                                'data': compact(P.jsonable(s.last_batch_records))}
                        meta["errors"][e["name"]] = error
                        s.log({"type": "experiment-error", "experiment": e["name"], **error})
                        action = ('remaining entries recorded as not submitted; USB is unavailable'
                                  if getattr(s, 'transport_unavailable', False) else 'continuing')
                        print("RECORDED ERROR %s: %s; %s" % (e["name"], exc, action), flush=True)
                    family_timing['family_seconds'] = round(time.monotonic() - family_started, 4)
                    timing['families'][e['name']] = family_timing
                    # Durable, append-only family record: the summary can be
                    # rebuilt from the journal alone (fmss rebuild).
                    entry = meta['results'].get(e['name'], {})
                    family_record = {'type': 'family-result', 'experiment': e['name'], 'timing': family_timing,
                                     'errors': meta['errors'].get(e['name'])}
                    for key in ('summary', 'data_format', 'reset_health', 'partial'):
                        if key in entry: family_record[key] = entry[key]
                    if entry and entry.get('data_format') != 'native-case-records':
                        family_record['data'] = entry.get('data')
                    s.log(family_record)
                    family_timing['checkpoint_seconds'] = round(checkpoint(), 4)
            meta["attempt_finished"] = not getattr(s, 'transport_unavailable', False)
            meta["complete"] = not meta["errors"]
            finish_timing(meta, campaign_started, checkpoint_total[0])
            save_started = time.monotonic()
            save(path, meta, level=6)
            print("Saved %s (final write %.1fs)" % (path, time.monotonic() - save_started))
            if meta["errors"]:
                return 2
        return 0
    except KeyboardInterrupt:
        if meta is not None and path is not None:
            meta['interrupted'] = True
            finish_timing(meta, campaign_started, None)
            meta['complete'] = meta['attempt_finished'] = False
            if s is not None and getattr(s, 'last_job_status', None) is not None:
                meta['last_job_status'] = s.last_job_status
            save(path, meta)
            print('Interrupted; saved collected results to ' + str(path), flush=True)
        return 130
    except (ToolError, OSError, ValueError, RuntimeError) as exc:
        if meta is not None and path is not None:
            meta["error"] = str(exc)
            save(path, meta)
        print("ERROR: %s" % exc, file=sys.stderr)
        return 1
    finally:
        if journal:
            journal.close()
        if s:
            s.close()
