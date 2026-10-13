"""Read original blue CSV statistics and verify the 60/20/20 formula."""
import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path, help='A stats CSV or a directory containing *_stats.csv files.')
    args = parser.parse_args()
    path = args.path
    if not path.exists():
        parser.error(f'Path not found: {path}')
    files = [path] if path.is_file() else sorted(path.rglob('*_stats.csv'))
    rows = []
    for file in files:
        with file.open(encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream):
                if not {'Rescued Percent', 'Health Return Score', 'Time Score', 'Round Score'} <= set(row):
                    continue
                row['file'] = str(file)
                rows.append(row)
    if not rows:
        parser.error('No blue-team stats CSV found. Enable stat_saving_enabled or use run_blue.py.')
    print('MAP | DISPOSAL% | HEALTH% | TIME% | SCORE | VERIFIED | END STEPS')
    for row in rows:
        computed = .6*float(row['Rescued Percent']) + .2*float(row['Health Return Score']) + .2*float(row['Time Score'])
        ok = abs(computed-float(row['Round Score'])) <= .021
        if not ok:
            raise ValueError('Score formula mismatch: ' + row['file'])
        print(row['Map'], row['Rescued Percent'], row['Health Return Score'], row['Time Score'],
              row['Round Score'], 'OK', row['Elapsed Time Step'], sep=' | ')
    weights = [float(row.get('Config Weight', 1)) for row in rows]
    score = sum(float(row['Round Score'])*w for row, w in zip(rows, weights))/sum(weights)
    full = sum(float(row['Rescued Percent']) == 100 and float(row.get('Crashed', 1)) == 0 for row in rows)
    print(f'FULL DISPOSAL: {full}/{len(rows)}; CONFIG-WEIGHTED SCORE: {score:.2f}')
    print('Archive folders should be summarized per version/batch; do not pool different versions as one score.')


if __name__ == '__main__':
    main()
