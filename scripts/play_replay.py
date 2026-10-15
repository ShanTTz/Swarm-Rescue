"""Select a version/layout/seed recording, or open an explicit NPZ file."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', choices=['v13_3', 'v13_2', 'v13_1', 'v4'], default='v13_3')
    parser.add_argument('--map', default='Map01')
    parser.add_argument('--layout', default='historic')
    parser.add_argument('--seed', type=int, default=12345)
    parser.add_argument('--file', type=Path, help='Direct path to any recorded NPZ.')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    if args.file and not args.list:
        path = args.file if args.file.is_absolute() else ROOT/args.file
    else:
        index = ROOT/'replays/index.json'
        if not index.is_file():
            parser.error('No bundled replay index. Use --file with a generated recording.')
        rows = json.loads(index.read_text(encoding='utf-8'))
        if args.list:
            for row in rows:
                print(row['version'], row['map'], row['layout'], f"seed={row.get('seed', 12345)}",
                      f"{row['disposed']}/10", row['score'], row['path'])
            return 0
        matches = [r for r in rows if (r['version'], r['map'], r['layout'], int(r.get('seed', 12345)))
                   == (args.version, args.map, args.layout, args.seed)]
        if not matches:
            parser.error('No bundled recording for this version/map/layout/seed. Use --list or --file.')
        path = ROOT/matches[0]['path']
    if not path.is_file():
        parser.error(f'Replay not found: {path}')
    env = dict(os.environ, PYTHONPATH=str(ROOT/'src'))
    return subprocess.run([sys.executable, '-m', 'swarm_rescue.launcher', '--replay', str(path)],
                          cwd=ROOT, env=env).returncode

if __name__ == '__main__':
    raise SystemExit(main())
