"""Extract concrete probe inputs without fabricating device results.

This small AST evaluator defers result-only calculations. A deferred value may
never enter a probe request or control a loop containing probes. The original
Python experiment is replayed against the actual captures for its report.
"""
import ast
import builtins
import copy
import inspect
import textwrap
import time
from types import SimpleNamespace
from .followup_experiments import Case, run_cases
from . import sequences as Q
from .protocol import ToolError, MUTATE
from fmss import probe as P


class Pending(Exception): pass
class PlanError(ToolError): pass
class Returned(Exception):
    def __init__(self, value): self.value = value
class Continued(Exception): pass
class Broken(Exception): pass


class Unknown:
    def __init__(self): self.fields = {}
    def __getitem__(self, key):
        try: return self.fields[key]
        except (KeyError, TypeError): return Unknown()
    def __setitem__(self, key, value):
        try: self.fields[key] = value
        except TypeError: pass
    def get(self, key, default=None): return self[key]
    def pop(self, key, default=None): return self[key]
    def __getattr__(self, key): raise Pending()
    def fail(self, *a, **kw): raise Pending()
    __str__ = __repr__ = __bool__ = __iter__ = __len__ = __index__ = __format__ = __hash__ = fail
    __add__ = __radd__ = __sub__ = __rsub__ = __mul__ = __rmul__ = fail
    __truediv__ = __floordiv__ = __mod__ = __rmod__ = __and__ = __or__ = fail
    __xor__ = __lshift__ = __rshift__ = __invert__ = __neg__ = fail
    __eq__ = __ne__ = __lt__ = __le__ = __gt__ = __ge__ = fail


def unknown(value):
    if isinstance(value, Unknown): return True
    if isinstance(value, (tuple, list)): return any(unknown(x) for x in value)
    if isinstance(value, dict): return any(unknown(x) for x in value.values())
    return False


class Calls(ast.NodeTransformer):
    def visit_Call(self, node):
        node = self.generic_visit(node)
        return ast.copy_location(ast.Call(ast.Name('_plan_call', ast.Load()),
                                         [node.func, *node.args], node.keywords), node)


class Planner:
    def __init__(self, session, target):
        self.s, self.t = session, target
        self.cases, self.stack = [], []
        self.cache = {}
        self.expr_cache, self.aug_cache = {}, {}
        self.pins = False

    def probe(self, s, t, code, **options):
        if unknown(code) or unknown(options):
            raise PlanError('device result required to construct a probe at ' + '/'.join(self.stack))
        options = dict(options)
        pins = options.pop('_pins', False) or self.pins
        # Freeze mutable initializers now, just as an immediate upload did.
        if options.get('scratch') is not None: options['scratch'] = bytes(options['scratch'])
        options['flags'] = options.get('flags', 0) | t.probe_flags
        options.pop('quiet', None)
        self.cases.append(Case('%04d/%s' % (len(self.cases), self.stack[-1]), bytes(code), copy.deepcopy(options), pins=pins))
        return Unknown()

    def call(self, fn, *args, **kw):
        if getattr(fn, '__self__', None) is self.s and getattr(fn, '__name__', '') == 'job':
            from .protocol import Cmd
            if args[:2] == (Cmd.NAND_STEP, 2): self.pins = True; return Unknown()
            raise PlanError('unhandled device command in resident plan')
        if fn is P.probe: return self.probe(*args, **kw)
        if fn is Q.run:
            s, t, body = args
            options = dict(kw)
            mutate, pins = options.pop('mutate', False), options.pop('pins', False)
            if unknown(body): raise PlanError('device-dependent source at ' + '/'.join(self.stack))
            src = t.prologue() + body + Q.wr(0, 0) + '    movi r6, #0x600d600d\n    end\n' + t.trap()
            options['flags'] = options.get('flags', 0) | (MUTATE if mutate else 0)
            return self.probe(s, t, P.assemble(t, src), _pins=pins, **options)
        if fn is run_cases:
            name, s, t, generate = args
            for case in generate(s, t):
                self.probe(s, t, P.assemble(t, case.source), _pins=case.pins, **case.options)
            return Unknown(), Unknown()
        if inspect.isgeneratorfunction(fn): return fn(*args, **kw)
        if fn is Q.completed: return args[0]  # evaluated against real output during report replay
        if fn is print: return None
        if isinstance(fn, Unknown): return Unknown()
        module = getattr(fn, '__module__', '') or ''
        if inspect.isfunction(fn) and (fn in (P.fmc_window_dump, P.soft_reset) or module.startswith('fmss.experiments.') or module in
                ('nano4g.m2_experiments', 'nano4g.control_experiments', 'nano4g.write_experiments',
                 'nano4g.discovery', 'nano4g.experiments')):
            return self.function(fn, args, kw)
        if any(isinstance(x, Unknown) for x in (*args, *kw.values())) and getattr(fn, '__name__', '') not in ('append', 'insert', 'setdefault'):
            return Unknown()
        try: return fn(*args, **kw)
        except Pending: return Unknown()
        except TypeError as exc:
            if 'Unknown' in str(exc): return Unknown()
            raise

    def evaluate(self, node, env):
        # eval uses one namespace so comprehensions see local loop variables.
        original_node = node
        if node not in self.expr_cache:
            transformed = Calls().visit(copy.deepcopy(node))
            expression = ast.fix_missing_locations(ast.Expression(transformed))
            self.expr_cache[node] = (compile(expression, '<resident-plan>', 'eval'), transformed)
        compiled, node = self.expr_cache[original_node]
        env['_plan_call'] = self.call
        try: return eval(compiled, env)
        except Pending:
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name) and sub.func.id == '_plan_call':
                    called = self.evaluate(sub.args[0], env)
                    if called is P.probe or called is Q.run or self.contains_probe(called):
                        raise PlanError('device-dependent probe inputs at ' + '/'.join(self.stack))
            return Unknown()

    def assign(self, target, value, env):
        if isinstance(target, ast.Name): env[target.id] = value
        elif isinstance(target, (ast.Tuple, ast.List)):
            values = [Unknown() for _ in target.elts] if isinstance(value, Unknown) else value
            for t, v in zip(target.elts, values): self.assign(t, v, env)
        elif isinstance(target, ast.Subscript):
            obj, key = self.evaluate(target.value, env), self.evaluate(target.slice, env)
            if not isinstance(key, Unknown): obj[key] = value
        elif isinstance(target, ast.Attribute):
            obj = self.evaluate(target.value, env)
            if not isinstance(obj, Unknown): setattr(obj, target.attr, value)
        else: raise PlanError('unsupported assignment ' + type(target).__name__)

    def block(self, nodes, env):
        for node in nodes:
            if isinstance(node, ast.Assign):
                value = self.evaluate(node.value, env)
                for t in node.targets: self.assign(t, value, env)
            elif isinstance(node, ast.AnnAssign): self.assign(node.target, self.evaluate(node.value, env), env)
            elif isinstance(node, ast.AugAssign):
                if node not in self.aug_cache:
                    load = copy.deepcopy(node.target)
                    for n in ast.walk(load):
                        if hasattr(n, 'ctx'): n.ctx = ast.Load()
                    self.aug_cache[node] = ast.BinOp(load, node.op, node.value)
                self.assign(node.target, self.evaluate(self.aug_cache[node], env), env)
            elif isinstance(node, ast.Expr):
                if isinstance(node.value, ast.Call) and self.evaluate(node.value.func, env) is print: continue
                self.evaluate(node.value, env)
            elif isinstance(node, ast.Return): raise Returned(self.evaluate(node.value, env) if node.value else None)
            elif isinstance(node, ast.For):
                it = self.evaluate(node.iter, env)
                if isinstance(it, Unknown):
                    raise PlanError('device-dependent iteration at ' + '/'.join(self.stack))
                for value in it:
                    self.assign(node.target, value, env)
                    try: self.block(node.body, env)
                    except Continued: continue
                    except Broken: break
                else: self.block(node.orelse, env)
            elif isinstance(node, ast.If):
                test = self.evaluate(node.test, env)
                if isinstance(test, Unknown):
                    # Output-only branches are deferred, never used to choose
                    # inputs. Any assigned local becomes unavailable as well.
                    for branch in (node.body, node.orelse):
                        for sub in ast.walk(ast.Module(body=branch, type_ignores=[])):
                            if isinstance(sub, ast.Call):
                                try: called = self.evaluate(sub.func, env)
                                except (NameError, Pending, AttributeError): called = None
                                if called is P.probe or called is Q.run or self.contains_probe(called):
                                    raise PlanError('device-dependent probe branch at ' + '/'.join(self.stack))
                            if isinstance(sub, ast.Name) and isinstance(sub.ctx, ast.Store): env[sub.id] = Unknown()
                else: self.block(node.body if test else node.orelse, env)
            elif isinstance(node, ast.Continue): raise Continued()
            elif isinstance(node, ast.Break): raise Broken()
            elif isinstance(node, (ast.Import, ast.ImportFrom)):
                exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), '<resident-plan>', 'exec'), env)
            elif isinstance(node, ast.FunctionDef):
                def local(*a, _node=node, **kw):
                    signature = inspect.signature(self.local_signature(_node, env))
                    bound = signature.bind(*a, **kw); bound.apply_defaults()
                    inner = dict(env); inner.update(bound.arguments)
                    try: self.block(_node.body, inner)
                    except Returned as r: return r.value
                local._plan_has_probe = False
                for call in (n for n in ast.walk(node) if isinstance(n, ast.Call)):
                    try: called = self.evaluate(call.func, env)
                    except (NameError, Pending, AttributeError): continue
                    local._plan_has_probe |= self.contains_probe(called)
                env[node.name] = local
            elif isinstance(node, ast.Try):
                try: self.block(node.body, env)
                except (Returned, Continued, Broken, PlanError): raise
                except Exception as exc:
                    for handler in node.handlers:
                        if handler.type is None or isinstance(exc, self.evaluate(handler.type, env)):
                            if handler.name: env[handler.name] = exc
                            self.block(handler.body, env); break
                    else: raise
                else: self.block(node.orelse, env)
                finally: self.block(node.finalbody, env)
            elif isinstance(node, ast.Raise):
                # Concrete failures still abort planning. Unknown-result guards
                # were deferred by the If handler above.
                raise self.evaluate(node.exc, env)
            elif not isinstance(node, ast.Pass): raise PlanError('unsupported statement ' + type(node).__name__)

    def local_signature(self, node, env):
        stub = copy.deepcopy(node); stub.body = [ast.Pass()]; stub.decorator_list = []
        scope = dict(env)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[stub], type_ignores=[])), '<resident-plan>', 'exec'), scope)
        return scope[node.name]

    def contains_probe(self, fn, seen=None):
        if isinstance(fn, Unknown): return False
        if fn is P.probe or fn is Q.run: return True
        if getattr(fn, '_plan_has_probe', False): return True
        if not inspect.isfunction(fn): return False
        if not (getattr(fn, '__module__', '') or '').startswith(('nano4g.', 'fmss.')): return False
        seen = set() if seen is None else seen
        if fn in seen: return False
        seen.add(fn)
        try: tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
        except (OSError, TypeError): return False
        scope = dict(fn.__globals__); scope.update(inspect.getclosurevars(fn).nonlocals)
        def resolve(node):
            if isinstance(node, ast.Name): return scope.get(node.id)
            if isinstance(node, ast.Attribute): return getattr(resolve(node.value), node.attr, None)
        return any(self.contains_probe(resolve(n.func), seen) for n in ast.walk(tree) if isinstance(n, ast.Call))

    def function(self, fn, args, kw):
        if fn not in self.cache:
            tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
            self.cache[fn] = tree.body[0]
        node = self.cache[fn]
        env = dict(fn.__globals__)
        env.update(inspect.getclosurevars(fn).nonlocals)
        bound = inspect.signature(fn).bind(*args, **kw); bound.apply_defaults(); env.update(bound.arguments)
        self.stack.append(fn.__name__)
        try:
            if isinstance(node, ast.FunctionDef):
                try: self.block(node.body, env)
                except Returned as r: return r.value
            else:
                # Registry binders return lambdas; inspect's source includes
                # the surrounding return statement.
                lam = next(x for x in ast.walk(node) if isinstance(x, ast.Lambda))
                return self.evaluate(lam.body, env)
        finally: self.stack.pop()

    def plan(self, fn):
        self.function(fn, (self.s, self.t), {})
        return self.cases


class PlanSession:
    def __init__(self, status, geometry):
        self.cached = status
        self.geometry = geometry
        self.m2_pins_ready = True
    def status(self): return self.cached
    def job(self, *a, **kw): raise PlanError('planning must not issue device commands')
    def read(self, *a, **kw): raise PlanError('planning must not read device memory')
    def write_memory(self, *a, **kw): raise PlanError('planning must not write device memory')


def compile_experiment(name, fn, view, target):
    from .resident_special import NAMES, cases
    if name in NAMES: return list(cases(name, view, target))
    return Planner(view, target).plan(fn)


def replay(fn, view, target, cases, records):
    """Run the original report code against real captures; no device methods exist."""
    import sys
    from contextlib import redirect_stdout
    import io
    from . import batch as B
    index = 0
    original = P.probe
    def captured(s, t, code, **options):
        nonlocal index
        if index >= len(cases): raise PlanError('report requested a probe absent from the resident manifest')
        case = cases[index]
        floor = getattr(view, 'observation_min_us', 0)
        wanted = B.prepare(Case('report', code, options), t, floor)
        planned = B.prepare(case, target, floor)
        if wanted.request != planned.request or wanted.scratch != planned.scratch or wanted.scratch_read != planned.scratch_read:
            raise PlanError('report probe differs from resident manifest at case %d' % index)
        index += 1
        rec = copy.deepcopy(records[case.name])
        if 'case_error' in rec: raise ToolError('captured case failed: ' + rec['case_error'])
        return rec
    patched = []
    for name, module in list(sys.modules.items()):
        if module is None or not name.startswith(('fmss.', 'nano4g.')): continue
        for key, value in list(vars(module).items()):
            if value is original:
                patched.append((module, key, value)); setattr(module, key, captured)
    class ReplaySession(PlanSession):
        def job(self, cmd, number, *a, **kw):
            from .protocol import Cmd
            if cmd != Cmd.NAND_STEP or number != 2: raise PlanError('unexpected report device command')
            # Every pins-marked case already ran this prerequisite in the worker.
            return SimpleNamespace(job_result=0, job_lib_rc=0)
    s = ReplaySession(view.cached, view.geometry)
    try:
        with redirect_stdout(io.StringIO()): result = fn(s, target)
        return result, index
    finally:
        for module, key, value in patched: setattr(module, key, value)


def resident(name, fn):
    """All legacy registry entries use the same resident transport by default."""
    def run(s, t):
        from .protocol import BATCH2
        from . import batch as B
        from .resident_special import NAMES
        if not getattr(s, 'batch_enabled', True): return fn(s, t)
        if not getattr(s, 'probe_caps', 0) & BATCH2:
            raise ToolError('Resident experiment support is missing: reload the updated emcore.bin (or explicitly use --single-probes)')
        print('PLAN: compiling complete resident experiment ' + name, flush=True)
        view = PlanSession(s.status(), Q.geometry(s))
        view.observation_min_us = getattr(s, 'observation_min_us', 0)
        plan_started = time.monotonic()
        planned = compile_experiment(name, fn, view, t)
        plan_seconds = time.monotonic() - plan_started
        s.log({'type': 'compile-timing', 'experiment': name, 'seconds': plan_seconds})
        if isinstance(getattr(s, 'family_timing', None), dict): s.family_timing['plan_seconds'] = round(plan_seconds, 4)
        print('TIMING %s: manifest generation %.2fs' % (name, plan_seconds), flush=True)
        records, transport = B.run(s, t, name, planned)
        if any('execution_error' in r for r in records.values()):
            return records, transport + '\nIncomplete execution: retaining every native case; legacy interpretation suppressed.'
        if name in NAMES:
            if name == 'straps' and 'case_error' not in records['GPIO-PCON-PDAT']:
                import struct
                raw = records['GPIO-PCON-PDAT']['scratch']
                values = {f'{label}{port:x}': {'address': 0x3cf00000 + port * 32 + off,
                          'value': struct.unpack_from('<I', raw, (port * 2 + i) * 4)[0]}
                          for port in range(16) for i, (label, off) in enumerate((('PCON', 0), ('PDAT', 4)))}
                return values, transport
            return records, transport
        report_started = time.monotonic()
        try:
            (data, summary), consumed = replay(fn, view, t, planned, records)
            report_seconds = time.monotonic() - report_started
            s.log({'type': 'report-timing', 'experiment': name, 'seconds': report_seconds})
            if isinstance(getattr(s, 'family_timing', None), dict): s.family_timing['report_seconds'] = round(report_seconds, 4)
            print('TIMING %s: original report %.2fs' % (name, report_seconds), flush=True)
            return data, transport + '\n' + str(summary) + (f'\nAll {len(planned)} cases captured; report consumed {consumed}.' if consumed != len(planned) else '')
        except Exception as exc:
            s.log({'type': 'report-error', 'experiment': name, 'message': str(exc)})
            return records, transport + '\nOriginal report could not finish: ' + str(exc) + '; all raw case records retained.'
    run.resident_original = fn
    return run
