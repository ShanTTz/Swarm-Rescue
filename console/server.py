# -*- coding: utf-8 -*-
"""Swarm-Rescue 可视化控制台后端。

只依赖 Python 标准库。通过子进程驱动 scripts/ 下既有脚本：
  run_blue.py / play_replay.py / summarize_scores.py / run_tests.py /
  check_submission.py / smoke_algorithm_upgrade.py / json_to_map / bomb_editor。
不改动任何仿真与算法逻辑。

启动：python console/server.py --port 8765 [--open-browser]
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import uuid
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONSOLE_DIR = Path(__file__).resolve().parent
LOG_DIR = CONSOLE_DIR / 'logs'
LOCAL_PY = ROOT / '.venv' / 'Scripts' / 'python.exe'
VERIFIED_PY = Path('C:/Users/25923/Documents/Codex/2026-09-16/x20/swarm-eod-main/.venv/Scripts/python.exe')
LAYOUTS = ['historic', 'stratified', 'edge_pairs', 'stress_far_wall_five_pairs']

CREATE_NO_WINDOW = 0x08000000
CREATE_NEW_CONSOLE = 0x00000010

JOBS = {}
JOBS_LOCK = threading.Lock()
WINDOWS = []  # 弹窗进程记录 [{pid,label,cmd,started}]


def find_python():
    if LOCAL_PY.is_file():
        return str(LOCAL_PY)
    if VERIFIED_PY.is_file():
        return str(VERIFIED_PY)
    return shutil.which('python') or 'python'


PY = find_python()


def base_env(extra=None):
    env = dict(os.environ)
    env['PYTHONPATH'] = str(ROOT / 'src')
    env['PYTHONIOENCODING'] = 'utf-8'
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    if extra:
        env.update(extra)
    return env


def safe_relpath(p):
    """把请求里的相对路径约束在项目根内，返回绝对 Path 或抛 ValueError。"""
    q = Path(str(p))
    if q.is_absolute():
        rp = q.resolve()
    else:
        rp = (ROOT / q).resolve()
    root = ROOT.resolve()
    if root != rp and root not in rp.parents:
        raise ValueError('路径必须在项目目录内: %s' % p)
    return rp


# ---------------------------------------------------------------- 地图元数据
MAP_IMPORT_RE = re.compile(r'from swarm_rescue\.maps\.(map_\w+) import (\w+)')


def registered_maps():
    lp = ROOT / 'src' / 'swarm_rescue' / 'launcher.py'
    try:
        txt = lp.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return []
    rows, seen = [], set()
    for m in MAP_IMPORT_RE.finditer(txt):
        module, cls = m.group(1), m.group(2)
        if cls in seen:
            continue
        seen.add(cls)
        row = {'module': module, 'cls': cls, 'registered': True,
               'size': None, 'steps': None, 'walltime': None}
        mf = ROOT / 'src' / 'swarm_rescue' / 'maps' / (module + '.py')
        if mf.is_file():
            mt = mf.read_text(encoding='utf-8', errors='replace')
            s = re.search(r'_size_area\s*=\s*\(([^)]*)\)', mt)
            if s:
                row['size'] = [x.strip() for x in s.group(1).split(',')[:2]]
            t = re.search(r'_max_timestep_limit\s*=\s*([\d_]+)', mt)
            row['steps'] = int(t.group(1).replace('_', '')) if t else None
            w = re.search(r'_max_walltime_limit\s*=\s*([\d.]+)', mt)
            row['walltime'] = float(w.group(1)) if w else None
        preview = ROOT / 'map_data' / (module + '.png')
        row['preview'] = ('/api/mapfile/' + module + '.png') if preview.is_file() else None
        editor_json = ROOT / 'map_data' / (module + '.json')
        row['editor_json'] = module + '.json' if editor_json.is_file() else None
        rows.append(row)
    return rows


def module_and_class(name):
    """复现 json_to_map 的命名规则。"""
    normalized = re.sub(r'[^0-9a-zA-Z]+', '_', name.strip()).strip('_').lower()
    if not normalized or not re.search(r'[0-9a-z]', normalized, re.I):
        raise ValueError('地图名必须包含字母或数字')
    module = 'map_' + normalized
    cls = 'Map' + ''.join(p.capitalize() for p in normalized.split('_') if p)
    return module, cls


def bombs_catalog():
    rows = []
    for folder in ('fixtures/bombs', 'map_data'):
        d = ROOT / folder
        if not d.is_dir():
            continue
        for f in sorted(d.glob('*.json')):
            if f.name.startswith('map_'):
                continue
            try:
                data = json.loads(f.read_text(encoding='utf-8-sig'))
                rows.append({'path': f.relative_to(ROOT).as_posix(), 'name': f.name,
                             'map_name': data.get('map_name', '?'),
                             'count': len(data.get('bombs', []))})
            except Exception:
                rows.append({'path': f.relative_to(ROOT).as_posix(), 'name': f.name,
                             'map_name': '(解析失败)', 'count': 0})
    return rows


# ---------------------------------------------------------------- 作业管理
def append_line(job, line):
    job['lines'].append(line.rstrip('\n'))
    if len(job['lines']) > 8000:
        del job['lines'][:2000]
        job['dropped'] += 2000


def finalize_run_result(job):
    """run 作业结束后解析 RESULT DIRECTORY 与 summary.json。"""
    batch = None
    for line in job['lines']:
        m = re.search(r'RESULT DIRECTORY:\s*(\S+)', line)
        if m:
            batch = m.group(1)
    if not batch:
        return
    d = ROOT / batch
    result = {'batch': batch, 'rows': []}
    summary = d / 'summary.json'
    if summary.is_file():
        try:
            for row in json.loads(summary.read_text(encoding='utf-8-sig')):
                result['rows'].append({
                    'map': row.get('map'), 'layout': row.get('layout'), 'seed': row.get('seed'),
                    'disposed': row.get('disposed'), 'score': row.get('Round Score'),
                    'health': row.get('Health Return Score'), 'time_score': row.get('Time Score'),
                    'crashed': row.get('Crashed'), 'exit': row.get('exit_code'),
                    'dir': row.get('output'), 'replays': row.get('replays', [])})
        except Exception as exc:
            result['error'] = 'summary.json 解析失败: %s' % exc
    job['result'] = result


def _reader(job, proc):
    try:
        for line in proc.stdout:
            append_line(job, line)
    except Exception as exc:
        append_line(job, '[reader-error] %r' % exc)
    code = proc.wait()
    job['exit'] = code
    job['finished'] = datetime.now().isoformat(timespec='seconds')
    if job['type'] == 'run' and code == 0:
        finalize_run_result(job)
    job['status'] = 'done' if code == 0 else 'failed'
    append_line(job, '[控制台] 进程退出，退出码=%s，状态=%s' % (code, job['status']))


def start_job(jtype, label, argv, exclusive=False):
    with JOBS_LOCK:
        if exclusive:
            for j in JOBS.values():
                if j['type'] == jtype and j['status'] == 'running':
                    return None, '已有同类任务在运行：%s（先停止或等待完成）' % j['label']
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        jid = uuid.uuid4().hex[:8]
        job = {'id': jid, 'type': jtype, 'label': label, 'status': 'running', 'exit': None,
               'started': datetime.now().isoformat(timespec='seconds'), 'finished': None,
               'lines': [], 'dropped': 0, 'result': None,
               'argv': [str(a) for a in argv]}
        try:
            proc = subprocess.Popen(argv, cwd=str(ROOT), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                                    errors='replace', env=base_env(),
                                    creationflags=CREATE_NO_WINDOW)
        except Exception as exc:
            return None, '启动失败: %r' % exc
        job['pid'] = proc.pid
        JOBS[jid] = job
    threading.Thread(target=_reader, args=(job, proc), daemon=True).start()
    return job, None


def spawn_window(label, argv):
    try:
        proc = subprocess.Popen(argv, cwd=str(ROOT), env=base_env(),
                                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, creationflags=CREATE_NEW_CONSOLE)
    except Exception as exc:
        return None, '启动失败: %r' % exc
    WINDOWS.append({'pid': proc.pid, 'label': label, 'cmd': [str(a) for a in argv],
                    'started': datetime.now().isoformat(timespec='seconds')})
    del WINDOWS[:-30]
    return proc.pid, None


# ---------------------------------------------------------------- 数据扫描
def trim_metric(row):
    return {'map': row.get('map'), 'layout': row.get('layout'), 'seed': row.get('seed'),
            'disposed': row.get('disposed'),
            'score': row.get('Round Score'), 'health': row.get('Health Return Score'),
            'time_score': row.get('Time Score'), 'crashed': row.get('Crashed'),
            'exit': row.get('exit_code'), 'dir': row.get('output'),
            'replays': row.get('replays', []),
            'steps': row.get('Elapsed Time Step'), 'realtime': row.get('Real Time Elapsed')}


def list_results(limit=60):
    out = []
    rdir = ROOT / 'results'
    if not rdir.is_dir():
        return out
    dirs = [d for d in rdir.iterdir() if d.is_dir()]
    dirs.sort(key=lambda d: d.stat().st_mtime, reverse=True)
    for d in dirs[:limit]:
        cases = []
        summary = d / 'summary.json'
        if summary.is_file():
            try:
                cases = [trim_metric(r) for r in json.loads(summary.read_text(encoding='utf-8-sig'))]
            except Exception:
                cases = []
        if not cases:
            for c in sorted(d.iterdir()):
                mj = c / 'metrics.json'
                if c.is_dir() and mj.is_file():
                    try:
                        cases.append(trim_metric(json.loads(mj.read_text(encoding='utf-8-sig'))))
                    except Exception:
                        pass
        out.append({'dir': d.name, 'mtime': datetime.fromtimestamp(d.stat().st_mtime)
                    .strftime('%m-%d %H:%M'), 'cases': cases})
    return out


def list_replays():
    rows = []
    idx = ROOT / 'replays' / 'index.json'
    if idx.is_file():
        try:
            for r in json.loads(idx.read_text(encoding='utf-8-sig')):
                rows.append({'version': r.get('version'), 'map': r.get('map'),
                             'layout': r.get('layout'), 'seed': r.get('seed', 12345),
                             'score': r.get('score'), 'disposed': r.get('disposed'),
                             'path': r.get('path'), 'source': 'bundled',
                             'exists': (ROOT / r.get('path', '')).is_file()})
        except Exception:
            pass
    rdir = ROOT / 'results'
    if rdir.is_dir():
        dirs = sorted([d for d in rdir.iterdir() if d.is_dir()],
                      key=lambda d: d.stat().st_mtime, reverse=True)[:40]
        for d in dirs:
            for mj in d.glob('*/metrics.json'):
                try:
                    row = json.loads(mj.read_text(encoding='utf-8-sig'))
                except Exception:
                    continue
                for rp in row.get('replays', []) or []:
                    rows.append({'version': row.get('version', '?'), 'map': row.get('map'),
                                 'layout': row.get('layout'), 'seed': row.get('seed'),
                                 'score': row.get('Round Score'), 'disposed': row.get('disposed'),
                                 'path': rp, 'source': 'results',
                                 'exists': (ROOT / rp).is_file()})
    return rows


def register_map_in_launcher(cls, module):
    lp = ROOT / 'src' / 'swarm_rescue' / 'launcher.py'
    lines = lp.read_text(encoding='utf-8').splitlines(keepends=True)
    newline = 'from swarm_rescue.maps.%s import %s\n' % (module, cls)
    if any(l.strip() == newline.strip() for l in lines):
        return '已存在导入，跳过'
    last = max((i for i, l in enumerate(lines) if MAP_IMPORT_RE.search(l)), default=None)
    if last is None:
        raise RuntimeError('launcher.py 中未找到地图导入区，请手工添加: ' + newline.strip())
    bak = LOG_DIR / 'backups'
    bak.mkdir(parents=True, exist_ok=True)
    (bak / ('launcher.py.%s.bak' % datetime.now().strftime('%Y%m%d_%H%M%S'))).write_text(
        ''.join(lines), encoding='utf-8')
    lines.insert(last + 1, newline)
    lp.write_text(''.join(lines), encoding='utf-8')
    return '已在 launcher.py 注册 ' + cls + '（修改前已自动备份到 console/logs/backups/）'


# ---------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = 'SwarmConsole/1.0'

    def log_message(self, fmt, *args):
        pass

    # -- 基础 ------------------------------------------------------------
    def _send(self, code, body, ctype='application/json; charset=utf-8'):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _ok(self, obj):
        self._send(200, obj)

    def _err(self, code, msg):
        self._send(code, {'error': msg})

    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        if n <= 0 or n > 30 * 1024 * 1024:
            raise ValueError('请求体为空或过大')
        return json.loads(self.rfile.read(n).decode('utf-8'))

    # -- GET -------------------------------------------------------------
    def do_GET(self):
        path = self.path.split('?')[0]
        try:
            if path in ('/', '/index.html'):
                f = CONSOLE_DIR / 'index.html'
                self._send(200, f.read_bytes(), 'text/html; charset=utf-8')
            elif path == '/api/bootstrap':
                self._ok({'python': PY, 'root': str(ROOT), 'maps': registered_maps(),
                          'layouts': LAYOUTS, 'bombs': bombs_catalog(),
                          'jobs': self._jobs_view(), 'windows': WINDOWS,
                          'results_count': len([d for d in (ROOT / 'results').glob('*') if d.is_dir()])})
            elif path == '/api/jobs':
                self._ok({'jobs': self._jobs_view(), 'windows': WINDOWS})
            elif path == '/api/log':
                self._api_log()
            elif path == '/api/replays':
                self._ok({'rows': list_replays()})
            elif path == '/api/results':
                self._ok({'batches': list_results()})
            elif path.startswith('/api/mapfile/'):
                self._api_mapfile(path)
            elif path.startswith('/api/mapjson/'):
                self._api_mapjson(path)
            else:
                self._err(404, '未知路径 ' + path)
        except Exception as exc:
            self._err(500, '服务器内部错误: %r' % exc)

    def _jobs_view(self):
        with JOBS_LOCK:
            return [{k: j[k] for k in ('id', 'type', 'label', 'status', 'exit', 'started',
                                       'finished', 'pid', 'dropped', 'result')}
                    for j in JOBS.values()]

    def _api_log(self):
        from urllib.parse import parse_qs, urlparse
        q = parse_qs(urlparse(self.path).query)
        jid = (q.get('id') or [''])[0]
        off = int((q.get('offset') or ['0'])[0])
        job = JOBS.get(jid)
        if not job:
            self._err(404, '作业不存在')
            return
        lines = job['lines']
        chunk = lines[off:off + 800]
        self._ok({'id': jid, 'lines': chunk, 'next': off + len(chunk), 'total': len(lines),
                  'status': job['status'], 'exit': job['exit'], 'result': job['result'],
                  'dropped': job['dropped']})

    def _api_mapfile(self, path):
        name = path[len('/api/mapfile/'):]
        if not re.fullmatch(r'map_[0-9A-Za-z_]+\.(png|json)', name):
            self._err(400, '非法文件名')
            return
        f = ROOT / 'map_data' / name
        if not f.is_file():
            self._err(404, '文件不存在')
            return
        ctype = 'image/png' if name.endswith('.png') else 'application/json; charset=utf-8'
        self._send(200, f.read_bytes(), ctype)

    def _api_mapjson(self, path):
        name = path[len('/api/mapjson/'):]
        if not re.fullmatch(r'map_[0-9A-Za-z_]+\.json', name):
            self._err(400, '非法文件名')
            return
        f = ROOT / 'map_data' / name
        if not f.is_file():
            f = ROOT / 'templates' / name  # 模板回退（如 templates/map_09.json）
        if not f.is_file():
            self._err(404, '无编辑器JSON')
            return
        self._send(200, f.read_bytes(), 'application/json; charset=utf-8')

    # -- POST ------------------------------------------------------------
    def do_POST(self):
        path = self.path.split('?')[0]
        try:
            body = self._body()
        except Exception as exc:
            self._err(400, '请求体解析失败: %r' % exc)
            return
        try:
            if path == '/api/run':
                self._api_run(body)
            elif path == '/api/stop':
                self._api_stop(body)
            elif path == '/api/tool':
                self._api_tool(body)
            elif path == '/api/replay':
                self._api_replay(body)
            elif path == '/api/map/add':
                self._api_map_add(body)
            elif path == '/api/bombs/editor':
                self._api_bombs_editor(body)
            elif path == '/api/bombs/validate':
                self._api_bombs_validate(body)
            elif path == '/api/bombs/upload':
                self._api_bombs_upload(body)
            elif path == '/api/openfolder':
                self._api_openfolder(body)
            else:
                self._err(404, '未知接口 ' + path)
        except ValueError as exc:
            self._err(400, str(exc))
        except Exception as exc:
            self._err(500, '服务器内部错误: %r' % exc)

    def _api_run(self, body):
        map_name = str(body.get('map', '')).strip()
        layout = str(body.get('layout', 'historic')).strip()
        seed = int(body.get('seed', 12345))
        headless = bool(body.get('headless', False))
        suite = bool(body.get('suite', False))
        bombs = str(body.get('bombs') or '').strip()
        if not (0 <= seed < 2 ** 32):
            self._err(400, 'seed 必须在 [0, 2^32) 内')
            return
        regs = {m['cls'] for m in registered_maps()}
        if not suite and map_name not in regs:
            self._err(400, '地图 %s 未注册（先在“地图”页新增注册）' % map_name)
            return
        if bombs:
            bp = safe_relpath(bombs)
            if not bp.is_file():
                self._err(400, '炸弹文件不存在: %s' % bombs)
                return
            try:
                data = json.loads(bp.read_text(encoding='utf-8-sig'))
                if data.get('map_name') != map_name:
                    raise ValueError('炸弹JSON的 map_name=%s 与所选地图 %s 不一致'
                                     % (data.get('map_name'), map_name))
                if len(data.get('bombs', [])) != 10:
                    raise ValueError('炸弹JSON需要恰好10枚，当前 %d 枚' % len(data.get('bombs', [])))
            except ValueError:
                raise
            except Exception as exc:
                raise ValueError('炸弹JSON解析失败: %r' % exc)
        argv = [PY, 'scripts/run_blue.py', '--map', map_name, '--layout', layout,
                '--seed', str(seed)]
        if bombs:
            argv += ['--bombs', bombs]
        if headless:
            argv.append('--headless')
        if suite:
            argv.append('--suite')
        label = ('批量15局 ' if suite else '运行 ') + (suite and 'Map01-05' or
                 ('%s/%s/seed=%s%s' % (map_name, layout, seed, ' 自定义炸弹' if bombs else '')) + (' 无头' if headless else ' 窗口'))
        job, err = start_job('run', label, argv, exclusive=True)
        if err:
            self._err(409, err)
            return
        self._ok({'job': job['id'], 'label': label})

    def _api_stop(self, body):
        jid = str(body.get('id', ''))
        job = JOBS.get(jid)
        if not job:
            self._err(404, '作业不存在')
            return
        if job['status'] != 'running':
            self._ok({'stopped': False, 'msg': '作业已结束'})
            return
        r = subprocess.run(['taskkill', '/PID', str(job['pid']), '/T', '/F'],
                           capture_output=True, text=True)
        self._ok({'stopped': r.returncode == 0, 'msg': (r.stdout or r.stderr).strip()})

    def _api_tool(self, body):
        action = str(body.get('action', ''))
        table = {
            'test': ('相关回归测试(约180项)', [PY, 'scripts/run_tests.py']),
            'test_full': ('全量测试(384项)', [PY, 'scripts/run_tests.py', '--full']),
            'check': ('提交检查 Check', [PY, 'scripts/check_submission.py']),
            'smoke': ('冒烟 Smoke(120步)', [PY, 'scripts/smoke_algorithm_upgrade.py',
                                            '--algorithm', 'v13', '--planner', 'astar', '--steps', '120']),
        }
        if action == 'walleditor':
            pid, err = spawn_window('墙体编辑器', [PY, '-m', 'swarm_rescue.tools.map_wall_editor'])
            if err:
                self._err(500, err)
                return
            self._ok({'msg': '墙体编辑器已打开（独立窗口，在本窗口内打开/保存编辑器JSON）'})
            return
        if action == 'scores':
            p = str(body.get('path') or '').strip()
            if not p:
                self._err(400, '请填写要复算的目录或CSV路径')
                return
            fp = safe_relpath(p)
            if not fp.exists():
                self._err(400, '路径不存在: %s' % p)
                return
            job, err = start_job('scores', '复算分数 ' + p,
                                 [PY, 'scripts/summarize_scores.py', str(fp)])
        elif action in table:
            label, argv = table[action]
            job, err = start_job(action, label, argv)
        else:
            self._err(400, '未知工具: ' + action)
            return
        if err:
            self._err(409, err)
            return
        self._ok({'job': job['id'], 'label': job['label']})

    def _api_replay(self, body):
        p = str(body.get('path') or '').strip()
        if not p:
            self._err(400, '缺少回放文件路径')
            return
        fp = safe_relpath(p)
        if not fp.is_file() or fp.suffix.lower() != '.npz':
            self._err(400, '回放文件不存在或不是 .npz: %s' % p)
            return
        argv = [PY, 'scripts/play_replay.py', '--file', str(fp)]
        pid, err = spawn_window('回放 ' + fp.name, argv)
        if err:
            self._err(500, err)
            return
        self._ok({'pid': pid, 'msg': '已打开回放窗口（空格播放/暂停，Q退出）'})

    def _api_map_add(self, body):
        name = str(body.get('name', '')).strip()
        raw = body.get('json')
        overwrite = bool(body.get('overwrite', False))
        if not name:
            self._err(400, '请填写地图名（如 11）')
            return
        module, cls = module_and_class(name)
        steps = []
        try:
            data = json.loads(raw) if isinstance(raw, str) else raw
            if not isinstance(data, dict) or 'canvas' not in data or 'elements' not in data:
                raise ValueError('JSON 需包含 canvas 与 elements 字段')
        except Exception as exc:
            self._err(400, '地图JSON解析失败: %r' % exc)
            return
        mapjson = ROOT / 'map_data' / (module + '.json')
        map_py = ROOT / 'src' / 'swarm_rescue' / 'maps' / (module + '.py')
        walls_py = ROOT / 'src' / 'swarm_rescue' / 'maps' / ('walls_' + module[4:] + '.py')
        walls_py2 = ROOT / 'src' / 'swarm_rescue' / 'maps' / (module.replace('map_', 'walls_', 1) + '.py')
        conflict = sorted({p for p in (mapjson, map_py, walls_py, walls_py2) if p.is_file()})
        if conflict and not overwrite:
            self._err(409, {'conflict': [p.relative_to(ROOT).as_posix() for p in conflict],
                             'msg': '同名地图文件已存在，勾选覆盖后重试'})
            return
        mapjson.parent.mkdir(parents=True, exist_ok=True)
        mapjson.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        steps.append('已保存 ' + mapjson.relative_to(ROOT).as_posix())
        cmd = [PY, '-m', 'swarm_rescue.tools.json_to_map', str(mapjson),
               '--name', name, '--maps-dir', 'src/swarm_rescue/maps']
        r = subprocess.run(cmd, cwd=str(ROOT), env=base_env(), capture_output=True,
                           text=True, encoding='utf-8', errors='replace', timeout=180,
                           creationflags=CREATE_NO_WINDOW)
        steps.append('$ python -m swarm_rescue.tools.json_to_map --name %s\n%s%s'
                     % (name, r.stdout.strip(), ('\n' + r.stderr.strip()) if r.stderr.strip() else ''))
        if r.returncode != 0:
            self._err(500, {'steps': steps, 'msg': 'json_to_map 生成失败'})
            return
        steps.append('已生成 %s 与 walls 文件' % map_py.name)
        try:
            steps.append(register_map_in_launcher(cls, module))
        except Exception as exc:
            self._err(500, {'steps': steps, 'msg': str(exc)})
            return
        cfg = ROOT / 'config' / ('rescue_%s.yml' % cls)
        tpl = (ROOT / 'templates' / 'rescue_Map09.yml').read_text(encoding='utf-8')
        cfg.write_text(tpl.replace('Map09', cls), encoding='utf-8')
        steps.append('已写配置 ' + cfg.relative_to(ROOT).as_posix() + ' (map_name=%s)' % cls)
        chk = subprocess.run([PY, '-c', 'import swarm_rescue.maps.%s as m; print(m.%s)' % (module, cls)],
                             cwd=str(ROOT), env=base_env(), capture_output=True, text=True,
                             encoding='utf-8', errors='replace', timeout=120,
                             creationflags=CREATE_NO_WINDOW)
        if chk.returncode == 0:
            steps.append('导入验证通过: ' + chk.stdout.strip())
        else:
            self._err(500, {'steps': steps, 'msg': '导入验证失败: ' + chk.stderr.strip()[-500:]})
            return
        self._ok({'steps': steps, 'cls': cls, 'module': module,
                  'msg': '地图 %s 已注册。下一步：生成炸弹布局后即可运行。' % cls})

    def _api_bombs_editor(self, body):
        map_name = str(body.get('map', '')).strip()
        out = str(body.get('output') or '').strip()
        regs = {m['cls'] for m in registered_maps()}
        if map_name not in regs:
            self._err(400, '地图 %s 未注册' % map_name)
            return
        if not out:
            out = 'fixtures/bombs/%s_custom.json' % map_name
        op = safe_relpath(out)
        op.parent.mkdir(parents=True, exist_ok=True)
        argv = [PY, '-m', 'swarm_rescue.tools.bomb_editor', map_name, '--output', str(op)]
        pid, err = spawn_window('炸弹编辑器 ' + map_name, argv)
        if err:
            self._err(500, err)
            return
        self._ok({'pid': pid, 'output': op.relative_to(ROOT).as_posix(),
                  'msg': '炸弹编辑器已打开：左键添加、右键删除、J导出、K清空、I导入、Q退出。放满10枚再导出。'})

    def _api_bombs_validate(self, body):
        p = str(body.get('path') or '').strip()
        fp = safe_relpath(p)
        if not fp.is_file():
            self._err(400, '文件不存在: %s' % p)
            return
        try:
            data = json.loads(fp.read_text(encoding='utf-8-sig'))
        except Exception as exc:
            self._err(400, 'JSON解析失败: %r' % exc)
            return
        self._ok({'map_name': data.get('map_name'), 'count': len(data.get('bombs', [])),
                  'zones': len(data.get('zones_config', []) or []),
                  'ok': data.get('map_name') == str(body.get('map', data.get('map_name')))
                        and len(data.get('bombs', [])) == 10})

    def _api_bombs_upload(self, body):
        name = str(body.get('name', '')).strip()
        raw = body.get('json')
        overwrite = bool(body.get('overwrite', False))
        if not re.fullmatch(r'[0-9A-Za-z_\-]{1,60}\.json', name):
            self._err(400, '文件名仅允许字母数字_-且以 .json 结尾')
            return
        try:
            data = json.loads(raw)
            if 'map_name' not in data or 'bombs' not in data:
                raise ValueError('缺少 map_name / bombs 字段')
        except Exception as exc:
            self._err(400, '炸弹JSON解析失败: %r' % exc)
            return
        dst = ROOT / 'fixtures' / 'bombs' / name
        if dst.exists() and not overwrite:
            self._err(409, {'msg': '同名文件已存在', 'exists': True})
            return
        dst.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        self._ok({'path': dst.relative_to(ROOT).as_posix(),
                  'map_name': data.get('map_name'), 'count': len(data.get('bombs', []))})

    def _api_openfolder(self, body):
        p = str(body.get('path') or '').strip()
        fp = safe_relpath(p) if p else ROOT
        if not fp.exists():
            self._err(400, '路径不存在')
            return
        subprocess.Popen(['explorer', str(fp)])
        self._ok({'ok': True})


def main():
    global PY
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--no-browser', action='store_true', help='不自动打开浏览器')
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    httpd = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    httpd.daemon_threads = True
    url = 'http://127.0.0.1:%d/' % args.port
    print('Swarm-Rescue 控制台已启动: %s' % url)
    print('Python: %s' % PY)
    print('项目根: %s' % ROOT)
    print('按 Ctrl+C 停止服务。')
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n已停止。')


if __name__ == '__main__':
    main()
