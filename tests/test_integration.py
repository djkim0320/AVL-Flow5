from pathlib import Path
import numpy as np
import pytest
from dbf_stability import solve_trim,analyze_stability,simulate,changed
from dbf_stability.avl import run_avl,parse_output
from dbf_stability.model import CoupledModel


@pytest.mark.avl
def test_real_avl_standard_axes_and_rate_derivatives(cfg,tmp_path):
    root=Path(cfg['_root']);geom=root/cfg['aero']['geometry'];exe=root/cfg['aero']['executable']
    d=run_avl(exe,geom,tmp_path/'base',alpha_deg=4,elevator_index=3)
    assert d['CZtot']<0 and d['Cmq']<0 and d['Clp']<0
    q=.0001
    p=run_avl(exe,geom,tmp_path/'plus',alpha_deg=4,elevator_index=3,rates=(0,q,0))
    m=run_avl(exe,geom,tmp_path/'minus',alpha_deg=4,elevator_index=3,rates=(0,-q,0))
    assert (p['Cmtot']-m['Cmtot'])/(2*q)==pytest.approx(d['Cmq'],abs=.08)
    direct=parse_output((tmp_path/'base/forces.txt').read_text())
    assert direct['Cmtot']==d['Cmtot']


def test_coupled_trim_and_unforced_equilibrium(cfg,aero):
    t=solve_trim(cfg,aero)
    m=CoupledModel(cfg,aero,t,phase='deployed',fixed_length=t['length_m'])
    dy=m.rhs(0,t['state'])
    assert np.linalg.norm(dy[[3,4,5,10,11,12,16,17,18,23,24,25]])<1e-5
    sim=simulate(cfg,aero,t,phase='deployed',duration=.15)
    assert sim.summary['status']=='completed'
    assert sim.summary['max_pitch_change_deg']<1e-5


def test_internal_force_momentum_balance(cfg,aero):
    t=solve_trim(cfg,aero)
    c=changed(cfg,**{'flight.g_m_s2':0.,'cable.cd_normal':0.,'cable.cd_tangent':0.})
    m=CoupledModel(c,aero,t,phase='deployed',fixed_length=t['length_m'])
    # Explicit zero-external-load fixture tests mechanics, never used by product runs.
    m.aircraft_loads=lambda time,a:(np.zeros(3),np.zeros(3),{})
    m.sensor_loads=lambda time,a,s:(np.zeros(3),np.zeros(3),1.)
    y=t['state'].copy();dy=m.rhs(0,y)
    momentum=m.ma*dy[3:6]+m.ms*dy[16:19]+m.mn*dy[26:].reshape(m.n,6)[:,3:].sum(axis=0)
    assert np.linalg.norm(momentum)<1e-10
    from dbf_stability.math3d import rotation
    angular=np.cross(y[:3],m.ma*dy[3:6])+rotation(y[6:10])@m.Ia@dy[10:13]
    angular+=np.cross(y[13:16],m.ms*dy[16:19])+rotation(y[19:23])@m.Is@dy[23:26]
    angular+=np.cross(y[26:].reshape(m.n,6)[:,:3],m.mn*dy[26:].reshape(m.n,6)[:,3:]).sum(axis=0)
    assert np.linalg.norm(angular)<1e-9


def test_payout_does_not_duplicate_mass(cfg,aero):
    t=solve_trim(cfg,aero,mode='stowed');m=CoupledModel(cfg,aero,t)
    masses=[m.rhs(time,m.initial(),True)[1]['total_mass_kg'] for time in [0,1,4,7,10]]
    assert np.ptp(masses)==0
    assert masses[0]==pytest.approx(cfg['aircraft']['mass_kg']+cfg['sensor']['mass_kg']+cfg['cable']['density_kg_m']*cfg['cable']['length_m'])


def test_local_linearization_predicts_rhs(cfg,aero):
    t=solve_trim(cfg,aero,mode='aircraft_only');r=analyze_stability(cfg,aero,t)
    m=CoupledModel(cfg,aero,t,phase='aircraft_only')
    perturb=np.zeros(12);perturb[5]=1e-5
    y=t['state'].copy();y[5]+=perturb[5]
    delta=m.rhs(0,y)-m.rhs(0,t['state'])
    reduced=np.r_[delta[:6],np.zeros(3),delta[10:13]]
    assert np.linalg.norm(reduced-r['matrix']@perturb)<1e-8


def test_aero_outside_bounds_is_not_extrapolated(aero):
    with pytest.raises(ValueError):aero.evaluate(np.deg2rad(80),0,0,np.zeros(3),16)


def test_mass_file_axes_and_inertia_products(cfg,tmp_path):
    from dbf_stability.avl import write_mass_file
    c=changed(cfg,**{'aircraft.inertia_kgm2':[[.18,.01,.02],[.01,.15,.03],[.02,.03,.3]]})
    rows=write_mass_file(c,tmp_path/'plane.mass',True).read_text().splitlines()[6:]
    values=np.array([list(map(float,row.split('#')[0].split())) for row in rows])
    assert values[:,0].sum()==pytest.approx(2.1078)
    assert values[0,1]==pytest.approx(.085)
    assert values[0,7:]==pytest.approx([.01,-.02,.03])


def test_cable_uniform_strain_and_rate_independent_of_mesh(cfg,aero):
    # Mechanical isolated chain, equal strain and strain-rate on all material spans.
    measured=[]
    for count in [10,20,40]:
        c=changed(cfg,**{'cable.segments':count})
        m=CoupledModel(c,aero,phase='deployed',fixed_length=c['cable']['length_m'])
        y=m.initial();y[6:10]=[1,0,0,0];y[19:23]=[1,0,0,0]
        length=c['cable']['length_m'];anchor=y[:3]+m.attach
        nose=anchor-np.array([1.05*length,0,0]);y[13:16]=nose-m.sensor_attach
        y[3:6]=[16,0,0];y[16:19]=[15.9,0,0]
        nodes=y[26:].reshape(count,6)
        fractions=(np.arange(count)+.5)/count
        nodes[:,:3]=nose+np.outer(fractions,[1.05*length,0,0])
        nodes[:,3:]=np.outer(1-fractions,[-.1,0,0])+[16,0,0]
        measured.append(m.rhs(0,y,True)[1]['tension_N'])
    assert measured==pytest.approx([60*.05+.03*.1]*3,rel=1e-10)


def test_stored_spool_internal_angular_momentum(cfg,aero):
    from dbf_stability.math3d import rotation
    c=changed(cfg,**{'flight.g_m_s2':0.,'cable.cd_normal':0.,'cable.cd_tangent':0.})
    m=CoupledModel(c,aero,phase='stowed')
    m.aircraft_loads=lambda t,a:(np.zeros(3),np.zeros(3),{})
    m.sensor_loads=lambda t,a,s:(np.zeros(3),np.zeros(3),0.)
    y=m.initial();nodes=y[26:].reshape(m.n,6)
    nodes[:,:3]+=[.001,.002,.003];nodes[:,3:]+=[.01,-.03,.04]
    dy=m.rhs(0,y);dn=dy[26:].reshape(m.n,6)
    linear=m.ma*dy[3:6]+m.ms*dy[16:19]+m.mn*dn[:,3:].sum(axis=0)
    angular=np.cross(y[:3],m.ma*dy[3:6])+rotation(y[6:10])@m.Ia@dy[10:13]
    angular+=np.cross(y[13:16],m.ms*dy[16:19])+rotation(y[19:23])@m.Is@dy[23:26]
    angular+=np.cross(nodes[:,:3],m.mn*dn[:,3:]).sum(axis=0)
    assert np.linalg.norm(linear)<1e-10
    assert np.linalg.norm(angular)<1e-9


@pytest.mark.parametrize('phase', ['stowed','deployed'])
def test_grouped_jacobian_matches_directional_difference(cfg,aero,phase):
    t=solve_trim(cfg,aero,mode=phase)
    m=CoupledModel(cfg,aero,t,phase=phase,fixed_length=t.get('length_m'))
    m.active_override=m.active_count(m.length(0)[0])
    y=t['state'].copy()
    if phase=='stowed':
        y[13]-=.005  # Keep the Jacobian check away from the slack/taut kink.
    direction=np.random.default_rng(413).normal(size=len(y))
    epsilon=1e-7
    numerical=(m.rhs(0,y+epsilon*direction)-m.rhs(0,y-epsilon*direction))/(2*epsilon)
    predicted=m.jacobian()(0,y)@direction
    assert np.linalg.norm(numerical-predicted)/np.linalg.norm(numerical)<2e-4


def test_ground_event_stops_propagation(cfg,aero):
    trim=solve_trim(cfg,aero,mode='aircraft_only');state=trim['state'].copy()
    state[2]=-.01;state[5]=.5
    result=simulate(cfg,aero,trim,phase='aircraft_only',initial_state=state,duration=.1)
    assert result.summary['status']=='ground_contact'
    assert result.time[-1]<.1
    assert result.table.altitude_m.min()>-1e-8


def test_coupled_linear_response_predicts_small_perturbation(cfg,aero):
    from scipy.linalg import expm
    from scipy.spatial.transform import Rotation
    from dbf_stability.math3d import rotation
    from threadpoolctl import threadpool_limits
    trim=solve_trim(cfg,aero);linear=analyze_stability(cfg,aero,trim)
    state=trim['state'].copy();state[17]+=1e-5
    c=changed(cfg,**{'simulation.rtol':1e-9,'simulation.atol':1e-11})
    result=simulate(c,aero,trim,phase='deployed',initial_state=state,duration=.1)
    reference=trim['state'].copy()
    reference[:3]+=reference[3:6]*.1;reference[13:16]+=reference[16:19]*.1
    reference[26:].reshape(-1,6)[:,:3]+=reference[26:].reshape(-1,6)[:,3:]*.1
    def reduced(y,base):
        pieces=[]
        for off in [0,13]:
            pieces.extend([y[off:off+6]-base[off:off+6],Rotation.from_matrix(rotation(base[off+6:off+10]).T@rotation(y[off+6:off+10])).as_rotvec(),y[off+10:off+13]-base[off+10:off+13]])
        return np.r_[*pieces,y[26:]-base[26:]]
    with threadpool_limits(limits=1):
        predicted=expm(linear['matrix']*.1)@reduced(state,trim['state'])
    actual=reduced(result.states[-1],reference)
    assert np.linalg.norm(actual-predicted)/np.linalg.norm(predicted)<.005


def test_capture_records_force_jump_without_velocity_reset(cfg,aero):
    trim=solve_trim(cfg,aero,mode='stowed');state=trim['state'].copy()
    state[13]-=.02
    result=simulate(cfg,aero,trim,phase='mission',initial_state=state,start_time=11,duration=.0001)
    assert result.summary['captured']
    assert result.time[0]==result.time[1]
    assert np.array_equal(result.states[0],result.states[1])
    assert result.table.capture_N.iloc[0]==0
    assert result.table.capture_N.iloc[1]>3.9


def test_sweep_preserves_failure_without_stale_result(cfg,aero,tmp_path):
    from dbf_stability import run_sweep
    from dbf_stability.analysis import load_result
    c=changed(cfg,**{'simulation.duration_s':.01})
    table=run_sweep(c,aero,[{'sensor.mass_kg':-1},{}],tmp_path,workers=2)
    assert table.status.tolist()==['failed','completed']
    with pytest.raises(ValueError):load_result(tmp_path/'case_000')
    assert load_result(tmp_path/'case_001').summary['status']=='completed'
