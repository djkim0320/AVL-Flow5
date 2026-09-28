"""Check actual reel commands used by both solvers for the requested mission."""
from pathlib import Path
import numpy as np
import pytest
from dbf_stability import load_case, AeroDatabase
from dbf_stability.model import CoupledModel, schedule

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('backend',['flow5','avl'])
def test_reel_holds_one_and_half_span_for_sixty_seconds(backend):
    c=load_case(ROOT/f'examples/h1_normal_r3_{backend}_span_hold.yaml')
    run='flow5_connection_01' if backend=='flow5' else 'normal_r3_01'
    db=AeroDatabase(ROOT/f'test_models/H1_reference/runs/{run}/aerodynamics/aero_database.npz')
    m=CoupledModel(c,db);p=c['mission_profile']
    assert c['cable']['length_m']==pytest.approx(1.5*db.refs[2])
    assert c['cable']['length_m']/c['cable']['segments']==pytest.approx(.15)
    assert p['recovery_start_s']-p['deployment_complete_s']==pytest.approx(60.)
    for t in np.linspace(p['deployment_complete_s'],p['recovery_start_s'],121)[:-1]:
        assert m.length(t)==pytest.approx((2.7,0.))
        assert m.door_state(t)==pytest.approx((140.,0.))
    assert m.length(p['recovery_start_s']+.001)[1]<0
    assert m.length(p['retrieval_complete_s'])[0]==pytest.approx(c['winch']['stowed_length_m'])
    assert m.door_state(p['end_s'])==(140.,0.)  # Capture condition is not bypassed.
    rows=np.asarray(c['winch']['length_schedule'])
    assert np.all(np.diff(rows[:,0])>1e-10)
    assert rows[-1,0]==pytest.approx(c['simulation']['duration_s'])
    # All commanded lengths stay physical, including at schedule interpolation.
    for t in np.linspace(0,p['end_s'],1001):
        length,_=m.length(t)
        assert c['winch']['stowed_length_m']-1e-12<=length<=2.7+1e-12


def test_longer_line_does_not_increase_reel_speed_or_change_final_approach():
    old=load_case(ROOT/'examples/h1_normal_r3_flow5.yaml')
    new=load_case(ROOT/'examples/h1_normal_r3_flow5_span_hold.yaml')
    def nonzero_speeds(c):
        rows=np.asarray(c['winch']['length_schedule'])
        change=np.diff(rows[:,1]);dt=np.diff(rows[:,0]);keep=abs(change)>1e-12
        return change[keep]/dt[keep]
    np.testing.assert_allclose(nonzero_speeds(new),nonzero_speeds(old),rtol=1e-10,atol=1e-10)
    # Recovery starts after a true plateau, not 60 seconds after initial release.
    assert new['winch']['recovery_start_s']>new['winch']['release_s']+60
    shift=new['mission_profile']['slow_approach_start_s']-5.5
    # Compare inside interpolation intervals; derivative sides at floating-point
    # knots are ambiguous, whereas physical length remains continuous.
    for t in np.linspace(5.513,8.987,31):
        assert schedule(new['winch']['length_schedule'],t+shift)==pytest.approx(schedule(old['winch']['length_schedule'],t),abs=1e-10)
