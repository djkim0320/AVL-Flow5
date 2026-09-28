"""Regression checks using the independently audited, real H1 saved trajectory."""
from pathlib import Path
import json
import shutil
import numpy as np
import pytest
from dbf_stability.analysis import load_result
from dbf_stability.plots import history_figure, replay_figure

RUN = Path(__file__).resolve().parents[1] / 'test_models/H1_reference/runs/20260917_031449/mission'


@pytest.fixture
def audited_run(tmp_path):
    names = ('states.npz', 'inputs.json', 'timeseries.csv', 'summary.json',
             'events.json', 'cad_collision_audit.json')
    if not all((RUN / n).exists() for n in names):
        pytest.skip('Requires delivered H1 states and independent CAD audit')
    for name in names:
        shutil.copy2(RUN / name, tmp_path / name)
    return tmp_path


def test_real_cad_intersection_invalidates_completed_integration(audited_run):
    original = load_result(audited_run, apply_geometry_audit=False)
    result = load_result(audited_run)
    assert original.summary['status'] == 'completed'
    assert result.summary['integration_status'] == 'completed'
    assert result.summary['status'] == 'geometry_invalid'
    assert result.summary['physical_validation'] == 'invalid_geometry'
    assert np.array_equal(original.states, result.states)  # No cosmetic trajectory repair.
    report = json.loads((audited_run / 'cad_collision_audit.json').read_text(encoding='utf8'))
    pairs = {(h['moving'].split('_span')[0], h['fixed']) for row in report['rows'] for h in row['hits']}
    assert ('cable', 'guide_floor') in pairs
    assert ('sensor', 'guide_floor') in pairs
    assert ('sensor', 'fuselage_H1_approx_with_rear_cutout') in pairs
    for figure in (history_figure(result), replay_figure(result)):
        assert any('INVALID TRAJECTORY' in a.text for a in figure.layout.annotations)


def test_changed_inputs_cannot_reuse_old_geometry_audit(audited_run):
    with (audited_run / 'inputs.json').open('a', encoding='utf8') as f:
        f.write('\n')
    with pytest.raises(ValueError, match='stale'):
        load_result(audited_run)
    # An auditor must still be able to read raw states to perform a fresh check.
    assert len(load_result(audited_run, apply_geometry_audit=False).time) > 0
