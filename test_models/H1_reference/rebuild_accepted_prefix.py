"""Recover real accepted states and recompute diagnostics; never invent motion."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,hashlib,json,sys,time
import numpy as np
import pandas as pd
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'solver_variants/fast_analytic_flow'))
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import euler
from dbf_stability.analysis import SimulationResult

def main():
    p=argparse.ArgumentParser();p.add_argument('branch',type=Path);args=p.parse_args();branch=args.branch.resolve()
    evidence=json.loads((branch/'branch_source.json').read_text(encoding='utf8'));source=Path(evidence['source_parent']);last=Path(evidence['history_file'])
    if hashlib.sha256(last.read_bytes()).hexdigest()!=evidence['history_sha256']:raise ValueError('History source changed')
    rows=[];sources=[]
    for path in sorted((source/'mission/accepted_history').glob('part_*.npz')):
        if path.name>last.name:break
        with np.load(path,allow_pickle=False) as d:
            meta=json.loads(str(d['metadata']))
            for t,y,captured in zip(d['time'],d['states'],d['captured']):
                if t>evidence['start_s']:break
                if rows and t==rows[-1][0]:
                    if not np.array_equal(y,rows[-1][1]):raise ValueError('Equal time with different accepted states')
                    if bool(captured)==rows[-1][2] and meta['active_nodes']==rows[-1][3]:continue
                rows.append((float(t),y.copy(),bool(captured),meta['active_nodes']))
        sources.append(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    assert rows[-1][0]==evidence['start_s']
    assert hashlib.sha256(rows[-1][1].tobytes()).hexdigest()==evidence['initial_state_sha256']
    c=load_case(source/'case.yaml');a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    m=CoupledModel(c,a,meta['trim']);details=[];minimum=c['collision']['skin_m'];started=time.perf_counter()
    for index,(t,y,captured,k) in enumerate(rows):
        m.captured=captured;m.capture_time=evidence['capture_time_s'] if captured else None;m.active_override=k
        _,d=m.rhs(t,y,True);gap=m.clearance_metric(t,y)+c['collision']['minimum_gap_m']
        if gap<c['collision']['minimum_gap_m']-1e-10:raise ValueError(f'Accepted state fails recomputed clearance at {t}')
        minimum=min(minimum,gap);d['minimum_mesh_gap_m']=gap;details.append(d)
        if index%500==0:print(f'Recomputed accepted diagnostics {index}/{len(rows)}',flush=True)
    times=np.array([r[0] for r in rows]);states=np.array([r[1] for r in rows]);frame=pd.DataFrame(details);frame.insert(0,'time_s',times)
    for index,name in enumerate(('roll_deg','pitch_deg','yaw_deg')):
        frame[name]=[np.rad2deg(euler(y[6:10])[index]) for y in states]
        frame['sensor_'+name]=[np.rad2deg(euler(y[19:23])[index]) for y in states]
    frame['altitude_m']=-states[:,2];frame['sensor_altitude_m']=-states[:,15]
    for out,key in [('contact_impulse_Ns','contact_N'),('capture_impulse_Ns','capture_N'),('winch_work_J','winch_power_W')]:
        v=frame[key].values;frame[out]=np.r_[0.,np.cumsum(np.diff(times)*(v[1:]+v[:-1])/2)]
    target=load_case(branch/'case.yaml')['mission_profile']['end_s']
    summary=dict(status='recorded_prefix',message='Real accepted states recovered from immutable chunks; diagnostics recomputed with verified equivalent force implementation.',phase='mission',
        start_time_s=float(times[0]),end_time_s=float(times[-1]),duration_s=float(times[-1]-times[0]),requested_duration_s=target-float(times[0]),
        captured=True,capture_status='captured',capture_time_s=evidence['capture_time_s'],final_door_deg=float(frame.door_deg.iloc[-1]),
        max_tension_N=float(frame.tension_N.max()),max_contact_N=float(frame.contact_N.max()),max_capture_N=float(frame.capture_N.max()),
        max_penetration_m=float(frame.penetration_m.max()),contact_impulse_Ns=float(frame.contact_impulse_Ns.iloc[-1]),capture_impulse_Ns=float(frame.capture_impulse_Ns.iloc[-1]),
        max_pitch_change_deg=float(abs(frame.pitch_deg-frame.pitch_deg.iloc[0]).max()),runtime_s=time.perf_counter()-started,
        runtime_scope='diagnostic reconstruction only; original integration runtime is preserved in source accepted_progress.json',
        event_sample_format=2,numerically_converged=False,physical_validation='unvalidated_assumption_case',geometry_validity='not_audited',
        mesh_contact_enabled=True,clearance_checks=len(rows),minimum_checked_mesh_gap_m=float(minimum),
        clearance_checks_scope='full-mesh clearance recomputed at recovered actual accepted endpoints/midpoints; source used travel bounds',
        assumptions=c['provenance'],aero_metadata=a.metadata,branch_source=evidence,accepted_history_sources=sources,
        scope='Actual recovery prefix from accepted histories; no interpolation, position/velocity reset or fabricated state. Independent CAD audit pending.')
    events=[dict(time_s=float(times[0]),event='recorded_prefix_start'),dict(time_s=evidence['capture_time_s'],event='capture_engaged',method='compliant latch; velocities continuous')]
    output=branch/'prefix';result=SimulationResult(times,states,frame,summary,events,c);result.save(output)
    m._mesh_contacts.close();print(json.dumps(dict(path=str(output),states=len(rows),end_s=float(times[-1]))),flush=True)

if __name__=='__main__':main()
