"""Portable serial competition runner, with fixed seeds and saved scores/replays."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULTS = {
    'SWARM_RESCUE_ALGORITHM': 'v13', 'SWARM_RESCUE_PLANNER': 'astar',
    'SWARM_RESCUE_MOTION': 'balanced', 'SWARM_RESCUE_REGION_SEARCH': '0',
    'SWARM_RESCUE_ROUTE_BIDS': '0', 'SWARM_RESCUE_TRAJECTORY': '1',
    'SWARM_RESCUE_AUCTION': '1', 'SWARM_RESCUE_DIAGNOSTICS': '1',
    'SWARM_RESCUE_V13_BOMB_CENTER': '1', 'SWARM_RESCUE_V13_VACANCY': '1',
    'SWARM_RESCUE_V13_VACANCY_POLICY': 'hard', 'SWARM_RESCUE_V13_VACANCY_RADIUS': '18',
    'SWARM_RESCUE_V13_BLOCK_HISTORY': '1', 'SWARM_RESCUE_V13_CLAIM_GUARD': '0',
    'SWARM_RESCUE_V13_PORTAL': '0', 'SWARM_RESCUE_V13_ENDGAME': '0',
    'SWARM_RESCUE_V13_DISPOSAL_VACANCY': '0', 'SWARM_RESCUE_V13_CORNER_GUARD': '0',
    'SWARM_RESCUE_V13_PEER_SWEEP': '0', 'SWARM_RESCUE_V13_PEER_SOFT': '0',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hash():
    data = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted((ROOT / 'src').rglob('*.py'))}
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def run_case(map_name, layout, bombs, seed, batch, headless):
    data = json.loads(bombs.read_text(encoding='utf-8'))
    if data.get('map_name') != map_name or len(data.get('bombs', [])) != 10:
        raise ValueError('Bomb file must match the map and contain 10 bombs for this team benchmark.')
    zones = data.get('zones_config', [])
    out = batch / f'{map_name}_{layout}_seed{seed}'
    out.mkdir()
    (out / 'bombs_input.json').write_bytes(bombs.read_bytes())
    config = out / 'eval.yml'
    config.write_text('stat_saving_enabled: true\nstate_recording_enabled: true\nvideo_capture_enabled: false\n'
                      'evaluation_plan:\n' + f'  - map_name: {map_name}\n    team_mode: rescue\n'
                      '    nb_rounds: 1\n    config_weight: 1\n    number_drones: 10\n'
                      f'    zones_config: {json.dumps(zones)}\n', encoding='utf-8')
    env = dict(os.environ)
    for key in list(env):
        if key.startswith(('SWARM_RESCUE_', 'SWARM_EVAL_')):
            del env[key]
    env.update(DEFAULTS, PYTHONPATH=str(ROOT / 'src'), PYTHONHASHSEED=str(seed), PYTHONIOENCODING='utf-8',
               SWARM_RESCUE_RESULT_PATH=str(out))
    args = ['swarm_rescue.launcher', '--competition', '--stop_at_first_crash', '--config', str(config),
            '--bombs-file', str(out / 'bombs_input.json')]
    if headless:
        args.append('--headless')
    program = ('import random,numpy as np,runpy,sys,inspect; '
               'from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode; '
               "cls=drone_class_for_mode('rescue'); print('CONTROLLER',inspect.getfile(cls),flush=True); "
               f'random.seed({seed}); np.random.seed({seed}); sys.argv={args!r}; '
               "runpy.run_module('swarm_rescue.launcher',run_name='__main__')")
    identity = {'version': 'v13.3', 'map': map_name, 'layout': layout, 'seed': seed,
                'source_sha256': source_hash(), 'bombs_sha256': digest(bombs), 'options': DEFAULTS,
                'sensor_backend': 'shader_requested', 'zones': zones}
    print(f'RUN {map_name} / {layout} / seed={seed}', flush=True)
    with (out / 'run.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen([sys.executable, '-u', '-c', program], cwd=ROOT, env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, encoding='utf-8', errors='replace')
        try:
            for line in process.stdout:
                log.write(line)
                if not line.startswith('TASK_EVENT '):
                    print(line, end='', flush=True)
            code = process.wait()
        except KeyboardInterrupt:
            process.terminate()
            process.wait()
            raise
    result = dict(identity, exit_code=code, output=out.relative_to(ROOT).as_posix())
    stats = out / 'team006_stats.csv'
    if stats.exists():
        with stats.open(encoding='utf-8-sig', newline='') as stream:
            matches = [row for row in csv.DictReader(stream) if row.get('Map') == map_name]
        if matches:
            result.update(matches[-1])
    result['full_disposal'] = code == 0 and float(result.get('Crashed', 1)) == 0 and float(result.get('Rescued Percent', 0)) == 100
    result['disposed'] = round(float(result.get('Rescued Percent', 0)) / 10)
    result['replays'] = [p.relative_to(ROOT).as_posix() for p in (out / 'recordings').glob('*.npz')]
    (out / 'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    if 'Round Score' not in result or code != 0:
        raise RuntimeError(f'Run failed or produced no score. Inspect {out / "run.log"}')
    print(f"RESULT: {result['disposed']}/10, score={result['Round Score']}, health={result['Health Return Score']}%", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default='Map01')
    parser.add_argument('--layout', default='historic', choices=['historic', 'stratified', 'edge_pairs', 'stress_far_wall_five_pairs'])
    parser.add_argument('--bombs', type=Path, help='Custom bomb JSON; supports registered new maps.')
    parser.add_argument('--seed', type=int, default=12345)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--suite', action='store_true', help='Run all 15 Map01-05 / three-layout cases serially.')
    args = parser.parse_args()
    if not 0 <= args.seed < 2**32:
        parser.error('Seed must be in [0, 2**32).')
    tasks = [(f'Map{i:02d}', layout, ROOT / f'fixtures/bombs/Map{i:02d}_{layout}.json')
             for i in range(1, 6) for layout in ('historic', 'stratified', 'edge_pairs')] if args.suite else [
             (args.map, 'custom' if args.bombs else args.layout,
              (args.bombs if args.bombs.is_absolute() else ROOT / args.bombs) if args.bombs else
              ROOT / (f'map_data/bombs_{args.map}_challenge.json' if args.map in ('Map06', 'Map07', 'Map08')
                      else f'fixtures/bombs/{args.map}_{args.layout}.json'))]
    for _, _, path in tasks:
        if not path.is_file():
            parser.error(f'Bomb file not found: {path}')
    batch = ROOT / 'results' / ('v13_3_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    batch.mkdir(parents=True)
    locked_hash = source_hash()
    rows = []
    for map_name, layout, bombs in tasks:
        if source_hash() != locked_hash:
            raise RuntimeError('Source changed during the batch; start a new batch after edits.')
        rows.append(run_case(map_name, layout, bombs, args.seed, batch, args.headless))
        (batch / 'summary.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f"ALL DONE: {sum(row['full_disposal'] for row in rows)}/{len(rows)} full-disposal rounds")
    print('RESULT DIRECTORY:', batch)


if __name__ == '__main__':
    main()
