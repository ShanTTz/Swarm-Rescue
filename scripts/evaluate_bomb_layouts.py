"""Serial paired evaluation with independent bomb-layout and simulation seeds.

Run with the project virtual environment. Controllers receive only normal
sensor inputs; bomb files are passed to the simulator, never to the solution.
"""
import argparse
import csv
import hashlib
import json
import os
import re
from collections import Counter
from pathlib import Path
import subprocess
import sys
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_digest(repo):
    hashes = {p.relative_to(repo).as_posix(): digest(p)
              for p in sorted((repo / 'src').rglob('*.py'))}
    return hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()


def evaluate(repo, name, task, bombs, seed, output, python):
    out = output / f"{task['map']}_{name}_{task['scenario']}_seed{seed}"
    backend = 'cpu' if os.environ.get('SWARM_EVAL_CPU_RAYS') == '1' else 'shader'
    v13_defaults = {'SWARM_RESCUE_ALGORITHM': 'v13', 'SWARM_RESCUE_PLANNER': 'astar',
                    'SWARM_RESCUE_MOTION': 'balanced', 'SWARM_RESCUE_TRAJECTORY': '1',
                    'SWARM_RESCUE_AUCTION': '1', 'SWARM_RESCUE_REGION_SEARCH': '0',
                    'SWARM_RESCUE_ROUTE_BIDS': '0', 'SWARM_RESCUE_DIAGNOSTICS': '0',
                    'SWARM_RESCUE_V13_VACANCY': '1', 'SWARM_RESCUE_V13_PORTAL': '0',
                    'SWARM_RESCUE_V13_BLOCK_HISTORY': '1', 'SWARM_RESCUE_V13_CLAIM_GUARD': '0',
                    'SWARM_RESCUE_V13_VACANCY_POLICY': 'hard', 'SWARM_RESCUE_V13_ENDGAME': '0',
                    'SWARM_RESCUE_V13_DISPOSAL_VACANCY': '0',
                    'SWARM_RESCUE_V13_PORTAL_PROGRESS_GATE': '1',
                    'SWARM_RESCUE_V13_VACANCY_RADIUS': '18', 'SWARM_RESCUE_V13_CORNER_GUARD': '0', 'SWARM_RESCUE_V13_PEER_SWEEP': '0', 'SWARM_RESCUE_V13_PEER_SOFT': '0', 'SWARM_RESCUE_V13_BOMB_CENTER': '1'}
    identity = {'version': name, 'map': task['map'], 'layout': task['scenario'],
                'seed_placement': 'after_solution_import_v1',
                'sensor_backend': backend,
                'algorithm_options': {key: os.environ.get(key, v13_defaults.get(key, '0')
                    if os.environ.get('SWARM_RESCUE_ALGORITHM', 'v13') == 'v13' else '0') for key in
                    ('SWARM_RESCUE_REGION_SEARCH', 'SWARM_RESCUE_ROUTE_BIDS', 'SWARM_RESCUE_ALGORITHM',
                     'SWARM_RESCUE_TRAJECTORY', 'SWARM_RESCUE_AUCTION', 'SWARM_RESCUE_PLANNER', 'SWARM_RESCUE_MOTION', 'SWARM_RESCUE_DIAGNOSTICS',
                     'SWARM_RESCUE_V13_VACANCY', 'SWARM_RESCUE_V13_PORTAL',
                     'SWARM_RESCUE_V13_BLOCK_HISTORY', 'SWARM_RESCUE_V13_CLAIM_GUARD',
                     'SWARM_RESCUE_V13_VACANCY_POLICY', 'SWARM_RESCUE_V13_ENDGAME',
                     'SWARM_RESCUE_V13_DISPOSAL_VACANCY', 'SWARM_RESCUE_V13_PORTAL_PROGRESS_GATE',
                     'SWARM_RESCUE_V13_VACANCY_RADIUS', 'SWARM_RESCUE_V13_CORNER_GUARD', 'SWARM_RESCUE_V13_PEER_SWEEP', 'SWARM_RESCUE_V13_PEER_SOFT', 'SWARM_RESCUE_V13_BOMB_CENTER')},
                'seed': seed, 'bombs_sha256': digest(bombs),
                'source_sha256': source_digest(repo), 'repo': str(repo)}
    marker = out / 'metrics.json'
    if marker.exists():
        previous = json.loads(marker.read_text(encoding='utf-8'))
        if all(previous.get(k) == v for k, v in identity.items()):
            return previous
        raise RuntimeError(f'Inputs changed for {out}; choose a fresh output directory.')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'bombs_input.json').write_bytes(bombs.read_bytes())
    config = out / 'eval.yml'
    config.write_text(
        'stat_saving_enabled: true\nstate_recording_enabled: true\n'
        'video_capture_enabled: false\nevaluation_plan:\n'
        f"  - map_name: {task['map']}\n    team_mode: rescue\n"
        '    nb_rounds: 1\n    config_weight: 1\n    number_drones: 10\n'
        '    zones_config: []\n', encoding='utf-8')
    environment = dict(os.environ, PYTHONPATH=str(repo / 'src'),
                       PYTHONHASHSEED=str(seed), PYTHONIOENCODING='utf-8',
                       SWARM_RESCUE_RESULT_PATH=str(out))
    backend_setup = (
        "from swarm_rescue.simulation.gui_map.playground import Playground; "
        "_original_init=Playground.__init__; "
        "Playground.__init__=lambda self,*a,**kw:_original_init(self,*a,**dict(kw,use_shaders=False)); "
    ) if backend == 'cpu' else ''
    program = (
        'import random,numpy as np,runpy,sys; '
        f'random.seed({seed}); np.random.seed({seed}); '
        + backend_setup +
        'import inspect; from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode; '
        "print('EVALUATED_SOLUTION',inspect.getfile(drone_class_for_mode('rescue')),flush=True); "
        f'random.seed({seed}); np.random.seed({seed}); '
        f"sys.argv=['swarm_rescue.launcher','--headless','--competition',"
        f"'--stop_at_first_crash','--config',{str(config)!r},"
        f"'--bombs-file',{str(out / 'bombs_input.json')!r}]; "
        "runpy.run_module('swarm_rescue.launcher',run_name='__main__')")
    print(f"START {name} {task['map']} {task['scenario']} seed={seed}", flush=True)
    started = time.monotonic()
    with (out / 'run.log').open('w', encoding='utf-8') as log:
        try:
            code = subprocess.run([str(python), '-u', '-c', program], cwd=repo,
                                  env=environment, stdout=log,
                                  stderr=subprocess.STDOUT, timeout=900).returncode
        except subprocess.TimeoutExpired:
            code = -999
    result = dict(identity, exit_code=code, duration_seconds=round(time.monotonic() - started, 2),
                  output=str(out))
    stats = out / 'team006_stats.csv'
    if stats.exists():
        with stats.open(encoding='utf-8-sig', newline='') as stream:
            matches = [row for row in csv.DictReader(stream) if row.get('Map') == task['map']]
        if matches:
            result.update(matches[-1])
    result['full_disposal'] = (code == 0 and float(result.get('Crashed', 1)) == 0
        and float(result.get('Rescued Percent', 0)) >= 100
        and float(result.get('Percent Drones Destroyed', result.get('Crashed', 100))) == 0)
    if 'Rescued Percent' in result:
        result['disposed_count'] = round(float(result['Rescued Percent'])/10)
        result['destroyed_count'] = round(float(result['Percent Drones Destroyed'])/10)
        result['all_bombs_home'] = code == 0 and result['disposed_count'] == 10
        result['completion_timestep'] = int(result['Rescue All Time Step']) if result['all_bombs_home'] else None
        result['score_60_20_20'] = round(.6*float(result['Rescued Percent'])
            + .2*float(result['Health Return Score']) + .2*float(result['Time Score']), 2)
    log_text = (out / 'run.log').read_text(encoding='utf-8', errors='replace')
    limits = re.findall(r'walltime elapsed: (\d+)s/(\d+)s, elapse timestep: (\d+)/(\d+) steps', log_text)
    if limits:
        elapsed_wall, wall_limit, elapsed_steps, step_limit = map(int, limits[-1])
        result['walltime_limit'] = wall_limit
        result['timestep_limit'] = step_limit
        result['termination'] = ('process_error' if code else 'full_disposal' if result['full_disposal']
            else 'timestep_limit' if elapsed_steps >= step_limit else 'walltime_limit' if elapsed_wall >= wall_limit
            else 'other')
    events = []
    for line in log_text.splitlines():
        if line.startswith('TASK_EVENT '):
            try:
                events.append(json.loads(line[11:]))
            except ValueError:
                continue
    result['task_event_counts'] = dict(Counter(event.get('event') for event in events))
    marker.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f"DONE full={result['full_disposal']} disposed={result.get('Rescued Percent')} "
          f"health={result.get('Health Return Score')}", flush=True)
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, default=root)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--layouts', type=Path, required=True, help='Generated manifest.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[12345])
    parser.add_argument('--python', type=Path, default=Path(sys.executable))
    args = parser.parse_args()
    manifest = args.layouts.resolve()
    tasks = json.loads(manifest.read_text(encoding='utf-8'))
    if not tasks or not args.seeds or any(seed < 0 or seed >= 2**32 for seed in args.seeds):
        parser.error('Provide at least one layout and valid NumPy seeds in [0, 2**32).')
    cases = [(task['map'], task['scenario']) for task in tasks]
    if len(cases) != len(set(cases)):
        parser.error('Manifest map/scenario names must be unique.')
    rows = []
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for task in tasks:
        bomb_path = Path(task['path'])
        if not bomb_path.is_absolute():
            bomb_path = manifest.parent / bomb_path
        for seed in args.seeds:
            for name, repo in [('baseline', args.baseline), ('candidate', args.candidate)]:
                rows.append(evaluate(repo.resolve(), name, task, bomb_path, seed,
                                     output, args.python.resolve()))
                (output / 'summary.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    pairs = [rows[i:i+2] for i in range(0, len(rows), 2)]
    regressions = [{'map': a['map'], 'layout': a['layout'], 'seed': a['seed']}
                   for a, b in pairs if a['full_disposal'] and not b['full_disposal']]
    gate = {'cases_per_version': len(pairs),
            'baseline_full': sum(a['full_disposal'] for a, _ in pairs),
            'candidate_full': sum(b['full_disposal'] for _, b in pairs),
            'candidate_passes_all_cases': all(b['full_disposal'] for _, b in pairs),
            'full_disposal_regressions': regressions}
    (output / 'gate.json').write_text(json.dumps(gate, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(gate, ensure_ascii=False, indent=2))
    return 0 if gate['candidate_passes_all_cases'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
