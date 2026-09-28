"""Refresh reports from actual saved runs, without repeating numerical integration."""

from pathlib import Path
import json
import pandas as pd
from dbf_stability.analysis import load_result
from run_mission_matrix import summarize


def main():
    root = Path(__file__).resolve().parents[1]
    out = root / 'outputs/mission_convergence'
    rows = []
    for path in sorted(out.glob('case_*/summary.json')):
        r = load_result(path.parent)
        rows.append(
            {
                'cable.segments': r.config['cable']['segments'],
                'simulation.max_step_s': r.config['simulation']['max_step_s'],
                **r.summary,
            }
        )
    table = pd.json_normalize(rows)
    table.to_csv(out / 'sweep.csv', index=False)
    (out / 'sweep.json').write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding='utf8')
    summarize(table, out)
    edge = root / 'outputs/edge_cases'
    rows = [
        {'scenario': p.parent.name, **json.loads(p.read_text(encoding='utf8'))}
        for p in sorted(edge.glob('*/summary.json'))
    ]
    (edge / 'summary.json').write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding='utf8')


if __name__ == '__main__':
    main()
