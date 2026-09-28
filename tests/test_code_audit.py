"""Regressions found by the 2026-09-22 structural/physics audit."""

import json
from pathlib import Path
import numpy as np
import pytest
from dbf_stability import changed, solve_trim, simulate, analyze_stability, run_sweep
from dbf_stability import AeroDatabase, build_aero_database
from dbf_stability.model import CoupledModel
from dbf_stability.verification import kinetic_work_audit


@pytest.mark.parametrize(
    'path,value',
    [
        ('cable.EA_N', np.nan),
        ('simulation.rtol', np.inf),
        ('sensor.cd', np.nan),
        ('flight.wind_ned_m_s', [0, np.inf, 0]),
        ('sensor.area_m2', 0),
        ('simulation.jacobian_workers', 1.5),
        ('simulation.jacobian_step_factor', 0),
        ('flight.controls', {'elevator_delta_deg': [[1, 0], [0, 5]]}),
        ('aero.alpha_deg', [0, 0]),
        ('bay.capture_speed_m_s', 0),
    ],
)
def test_invalid_case_is_rejected(cfg, path, value):
    with pytest.raises(ValueError):
        changed(cfg, **{path: value})


def test_stowed_phase_does_not_play_mission_winch_schedule(cfg, aero):
    with __import__('contextlib').closing(CoupledModel(cfg, aero, phase='stowed')) as m:
        for t in [0, 3, 5, 8, 100]:
            assert m.length(t) == (cfg['winch']['stowed_length_m'], 0.0)


def test_nonzero_yaw_acceleration_is_not_successful_trim(cfg, aero):
    c = changed(cfg, **{'aircraft.thrust_point_m': [0, 0.05, 0]})
    with pytest.raises(RuntimeError, match='TRIM_INFEASIBLE'):
        solve_trim(c, aero, mode='aircraft_only')


def test_aircraft_only_trim_ignores_unused_sensor_offset(cfg, aero):
    c = changed(cfg, **{'sensor.tow_point_m': [0.04, 0.02, 0]})
    assert solve_trim(c, aero, mode='aircraft_only')['residual_norm'] < 1e-5


def test_nested_case_uses_project_root_and_explicit_external_root(cfg, tmp_path):
    from dbf_stability import load_case
    import yaml

    root = Path(cfg['_root'])
    nested = load_case(root / 'test_models/H1_reference/runs/recovery_repair_08/stop_slow/case.yaml')
    assert Path(nested['_root']) == root
    p = tmp_path / 'case.yaml'
    p.write_text(yaml.safe_dump({k: v for k, v in cfg.items() if not k.startswith('_')}), encoding='utf8')
    external = load_case(p, root=root)
    assert Path(external['_root']) == root


def test_shielded_full_tow_trim_uses_same_local_airflow_as_rhs(cfg, aero):
    c = changed(
        cfg,
        **{
            'flight.cable_bay_shielding': True,
            'flight.internal_cable_flow_factor': 0.0,
            'flight.local_flow_factor': 0.8,
        },
    )
    t = solve_trim(c, aero)
    m = CoupledModel(c, aero, t, phase='deployed', fixed_length=t['length_m'])
    try:
        dy = m.rhs(0, t['state'])
        res = np.r_[dy[3:6], dy[10:13], dy[16:19], dy[23:26], dy[26:].reshape(m.n, 6)[:, 3:].ravel()]
        assert max(abs(res)) < 1e-5
    finally:
        m.close()


@pytest.mark.parametrize('duration', [0, -1, np.nan, np.inf])
def test_invalid_simulation_interval_is_rejected(cfg, aero, duration):
    with pytest.raises(ValueError):
        simulate(cfg, aero, duration=duration)


def test_initial_underground_state_stops_before_advancing(cfg, aero):
    t = solve_trim(cfg, aero, mode='aircraft_only')
    state = t['state'].copy()
    state[2] = 1.0
    r = simulate(cfg, aero, t, phase='aircraft_only', initial_state=state, duration=0.01)
    assert r.summary['status'] == 'ground_contact' and r.summary['duration_s'] == 0
    np.testing.assert_array_equal(r.states[0], state)


def test_zero_quaternion_is_rejected(cfg, aero):
    t = solve_trim(cfg, aero, mode='aircraft_only')
    state = t['state'].copy()
    state[6:10] = 0
    with pytest.raises(ValueError, match='quaternion'):
        simulate(cfg, aero, t, phase='aircraft_only', initial_state=state, duration=0.01)


def test_modes_reject_non_equilibrium_input(cfg, aero):
    t = solve_trim(cfg, aero, mode='aircraft_only')
    t['state'][5] += 0.2
    with pytest.raises(ValueError, match='equilibrium'):
        analyze_stability(cfg, aero, t)


def test_incompatible_aerodynamic_geometry_rejected(cfg, aero, tmp_path):
    p = tmp_path / 'other.avl'
    p.write_text('a different geometry')
    c = changed(cfg, **{'aero.geometry': str(p)})
    with pytest.raises(ValueError, match='AERO_MISMATCH'):
        CoupledModel(c, aero)


def test_corrupt_aero_table_and_nonfinite_query_rejected(aero, tmp_path):
    with np.load(aero.path, allow_pickle=False) as d:
        data = {k: d[k].copy() for k in d.files}
    data['rates'].flat[0] = np.nan
    p = tmp_path / 'bad.npz'
    np.savez_compressed(p, **data)
    with pytest.raises(ValueError, match='rates'):
        AeroDatabase(p)
    with pytest.raises(ValueError, match='AERO_DOMAIN'):
        aero.evaluate(0, 0, 0, [0, 0, 0], 0)


def test_interlocked_capture_energy_audit_uses_actual_capture_time(cfg, aero, tmp_path):
    c = changed(
        cfg,
        **{
            'winch.door_capture_interlock': True,
            'winch.door_capture_hold_s': 0.2,
            'winch.door_schedule': [[0, 0], [0.4, 140], [10, 140], [10.8, 0], [12, 0]],
        },
    )
    t = solve_trim(c, aero, mode='stowed')
    r = simulate(c, aero, t, start_time=11, initial_state=t['state'], initial_capture_time=10.9, duration=0.002)
    report = kinetic_work_audit(r, aero, tmp_path / 'energy')
    assert np.isfinite(report['max_residual_J'])
    r.summary.pop('trim')
    with pytest.raises(ValueError, match='actual trim'):
        kinetic_work_audit(r, aero, tmp_path / 'bad')


def test_parallel_sweep_avoids_nested_pools_and_preserves_existing_runs(cfg, aero, tmp_path):
    c = changed(cfg, **{'simulation.duration_s': 0.001, 'simulation.jacobian_workers': 4})
    run_sweep(c, aero, [{}, {'simulation.jacobian_workers': 3}], tmp_path, workers=2, phase='aircraft_only')
    for p in tmp_path.glob('case_*/inputs.json'):
        assert json.loads(p.read_text('utf8'))['simulation']['jacobian_workers'] == 1
    before = (tmp_path / 'case_000/summary.json').read_bytes()
    with pytest.raises(FileExistsError):
        run_sweep(c, aero, [{}], tmp_path, workers=2)
    assert before == (tmp_path / 'case_000/summary.json').read_bytes()
    assert c['simulation']['jacobian_workers'] == 4


@pytest.mark.avl
def test_real_avl_database_small_parallel_rebuild_and_no_overwrite(cfg, tmp_path):
    c = changed(cfg, **{'aero.alpha_deg': [-2, 2], 'aero.beta_deg': [-2, 2], 'aero.elevator_deg': [-2, 2]})
    p = tmp_path / 'actual.npz'
    db = build_aero_database(c, p, workers=2)
    db.assert_compatible(c)
    assert db.metadata['solver'] == 'AVL 3.52' and np.isfinite(db.evaluate(0, 0, 0, [0, 0, 0], 16)).all()
    before = p.read_bytes()
    with pytest.raises(FileExistsError):
        build_aero_database(c, p, workers=2)
    assert p.read_bytes() == before
