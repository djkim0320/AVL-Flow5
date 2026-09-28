"""Actual conical-stop impacts: equivalent acceleration and damper energy."""
import copy,json
from pathlib import Path
import numpy as np
import pytest
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import rotation
from dbf_stability.config import validate

ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'test_models/H1_reference/runs'


def data():
    c=load_case(RUNS/'recovery_repair_08/stop_slow/case.yaml')
    explicit_modeling_manifest(c,'h1_round_x_capture_stop_v2.yaml')
    a=AeroDatabase(RUNS/'span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    trim=json.loads((RUNS/'recovery_repair_08/stop_slow/trim.json').read_text())
    with np.load(ROOT/'tests/data/capture_stop_contact.npz',allow_pickle=False) as d:t,y=d['time'].copy(),d['states'].copy()
    return c,a,trim,t,y


def test_bounded_engine_preserves_actual_impact_forces_and_derivatives():
    c,a,tr,t,y=data();new=copy.deepcopy(c);new['collision']['feature_engine']='numba_bounded'
    first,second=CoupledModel(c,a,tr),CoupledModel(new,a,tr)
    try:
        for time,state in zip(t,y):
            for axis,amount in [(13,-1e-7),(14,0.),(15,1e-7)]:
                yy=state.copy();yy[axis]+=amount
                d0,f0=first.rhs(time,yy,True);d1,f1=second.rhs(time,yy,True)
                np.testing.assert_allclose(d0,d1,rtol=1e-11,atol=1e-8)
                np.testing.assert_allclose(f0['contact_N'],f1['contact_N'],rtol=1e-12,atol=1e-10)
    finally:
        for m in (first,second):
            if m._mesh_contacts:m._mesh_contacts.close()


def test_stop_damper_dissipates_and_preserves_total_force_and_moment():
    c,a,tr,t,y=data();c['collision']['feature_engine']='numba_bounded'
    damped=copy.deepcopy(c);damped['collision']['part_materials']={'capture_front_stop':dict(stiffness_N_m=20000.,damping_Ns_m=240.)}
    validate(damped);first,second=CoupledModel(c,a,tr),CoupledModel(damped,a,tr)
    state=y[1];time=t[1]
    try:
        d0=first.rhs(time,state);d1=second.rhs(time,state);difference=d1-d0
        force=np.zeros(3);moment=np.zeros(3);power=0.;origin=state[:3]
        for offset,mass,inertia in [(0,first.ma,first.Ia),(13,first.ms,first.Is)]:
            f=mass*difference[offset+3:offset+6];r=rotation(state[offset+6:offset+10])
            torque=inertia@difference[offset+10:offset+13];force+=f
            moment+=np.cross(state[offset:offset+3]-origin,f)+r@torque
            power+=state[offset+3:offset+6]@f+state[offset+10:offset+13]@torque
        for body,delta in zip(state[26:].reshape(-1,6),difference[26:].reshape(-1,6)):
            f=first.mn*delta[3:6];force+=f;moment+=np.cross(body[:3]-origin,f);power+=body[3:6]@f
        np.testing.assert_allclose(force,0.,atol=1e-10)
        np.testing.assert_allclose(moment,0.,atol=1e-10)
        assert power<-.01
        invalid=copy.deepcopy(damped);invalid['collision']['part_materials']['capture_front_stop']['damping_Ns_m']=-1
        with pytest.raises(ValueError,match='damping'):validate(invalid)
    finally:
        for m in (first,second):
            if m._mesh_contacts:m._mesh_contacts.close()

from conftest import explicit_modeling_manifest
