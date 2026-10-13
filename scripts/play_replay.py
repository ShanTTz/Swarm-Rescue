"""Select bundled recordings or open any newly generated NPZ replay."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', choices=['v13_1', 'v4'], default='v13_1')
    parser.add_argument('--map', default='Map01')
    parser.add_argument('--layout', default='historic')
    parser.add_argument('--file', type=Path, help='Direct path to any recorded NPZ.')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    rows = json.loads((ROOT / 'replays/index.json').read_text(encoding='utf-8'))
    if args.list:
        for row in rows:
            print(row['version'], row['map'], row['layout'], f"{row['disposed']}/10", row['score'], row['path'])
        return 0
    if args.file:
        path = args.file if args.file.is_absolute() else ROOT / args.file
    else:
        matches = [r for r in rows if (r['version'], r['map'], r['layout']) == (args.version, args.map, args.layout)]
        if not matches:
            parser.error('This combination has no bundled replay. Use --list or --file; v13.1 has eight saved cases.')
        path = ROOT / matches[0]['path']
    if not path.is_file():
        parser.error(f'Replay not found: {path}')
    env = dict(os.environ, PYTHONPATH=str(ROOT / 'src'))
    return subprocess.run([sys.executable, '-m', 'swarm_rescue.launcher', '--replay', str(path)], cwd=ROOT, env=env).returncode


if __name__ == '__main__':
    raise SystemExit(main())
