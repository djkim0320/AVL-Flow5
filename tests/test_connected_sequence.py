import json
import numpy as np
import pytest
from dbf_stability import changed,solve_trim,simulate,resume_simulation
from dbf_stability.model import CoupledModel
from dbf_stability.avl import parse_output


def interlocked(cfg):
    return changed(cfg,**{'winch.door_capture_interlock':True,'winch.door_capture_hold_s':.2,
        'winch.door_schedule':[[0,0],[.4,140],[10,140],[10.8,0],[12,0]]})


def test_failed_capture_holds_actual_door_open(cfg,aero):
    m=CoupledModel(interlocked(cfg),aero)
    assert m.door_state(.2)==pytest.approx((70,350))
    assert m.door_state(11)==(140,0)
    assert m.door_state(30)==(140,0)


@pytest.mark.parametrize('capture_time,close_start',[(9,10),(11,11.2)])
def test_capture_delays_closing_without_angle_jump(cfg,aero,capture_time,close_start):
    m=CoupledModel(interlocked(cfg),aero);m.captured=True;m.capture_time=capture_time
    assert m.door_state(close_start-1e-8)[0]==pytest.approx(140)
    assert m.door_state(close_start+.4)[0]==pytest.approx(70)
    assert m.door_state(close_start+.8)[0]==pytest.approx(0)
    assert close_start in m.door_transition_times()


def test_partial_result_and_strict_restart_keep_capture_state(cfg,aero,tmp_path):
    c=changed(interlocked(cfg),**{'simulation.maximum_rhs_evaluations':35,'simulation.max_step_s':.001})
    trim=solve_trim(c,aero,mode='stowed')
    first=simulate(c,aero,trim,start_time=11,duration=.1,initial_state=trim['state'],output=tmp_path/'first',initial_capture_time=10.9)
    assert first.summary['status']=='evaluation_limit'
    assert 11<first.time[-1]<11.1
    with np.load(tmp_path/'first/accepted_checkpoint.npz') as d:
        assert float(d['time'])==first.time[-1]
        assert np.array_equal(d['state'],first.states[-1])
        assert json.loads(str(d['metadata']))['capture_time_s']==10.9
    c['simulation'].pop('maximum_rhs_evaluations')
    second=resume_simulation(c,aero,tmp_path/'first/accepted_checkpoint.npz',duration=.01,output=tmp_path/'second')
    assert second.summary['status']=='completed'
    assert second.table.captured.all()
    assert second.table.capture_N.iloc[0]==pytest.approx(first.table.capture_N.iloc[-1])
    assert np.array_equal(second.states[0],first.states[-1])
    other=changed(c,**{'sensor.mass_kg':c['sensor']['mass_kg']+.01})
    with pytest.raises(ValueError,match='changed'):
        resume_simulation(other,aero,tmp_path/'first/accepted_checkpoint.npz',duration=.01,output=tmp_path/'bad')


def test_avl_spiral_ratio_does_not_overwrite_yaw_derivative():
    assert parse_output(' Clb = -.08 Cnb = .0409\n Clb Cnr / Clr Cnb = 2.519')['Cnb']==.0409
