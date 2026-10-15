"""Run the core controller checks or the full suite with local temp files."""
import argparse
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
CORE = ('test_algorithm_upgrade', 'test_advanced_upgrade', 'test_v13_upgrade',
        'test_return_navigation', 'test_task_progress', 'test_carrier_recovery',
        'test_bomb_tracking', 'test_negative_observation', 'test_peer_yield',
        'test_online_navigation', 'test_shared_frontier', 'test_mapping_during_rescue',
        'test_online_mapping', 'test_ground_truth_in_control', 'test_score_manager',
        'test_state_recording', 'test_disposal_repair', 'test_carrier_traffic')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full', action='store_true', help='Run every test, including engine and editor checks.')
    args = parser.parse_args()
    os.chdir(ROOT)
    sys.path.insert(0, str(ROOT/'src'))
    task_temp = ROOT/'.test_cache/tmp'
    task_temp.mkdir(parents=True, exist_ok=True)
    tempfile.tempdir = str(task_temp)
    base = ROOT/'.test_cache'/f'pytest_{time.time_ns()}_{os.getpid()}'
    if ROOT not in base.resolve().parents:
        raise ValueError('Test output must stay in this project.')
    import pytest
    selected = ['tests'] if args.full else [f'tests/{name}.py' for name in CORE]
    return pytest.main(selected+['-q', '-p', 'no:cacheprovider', '-W', 'ignore', '--basetemp', str(base)])

if __name__ == '__main__':
    raise SystemExit(main())
