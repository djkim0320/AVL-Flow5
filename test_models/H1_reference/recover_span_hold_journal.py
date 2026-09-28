"""Rebuild diagnostics of durable accepted states; never interpolate new poses."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import json
import hashlib
import numpy as np
import pandas as pd
from dbf_stability import AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.analysis import SimulationResult
from dbf_stability.math3d import euler

HERE=Path(__file__).resolve().parent
OUT=HERE/'runs/span_hold_01'


def init_worker():
    global MODEL
    c=json.loads((OUT/'mission/inputs.json').read_text(encoding='utf8'))
    a=AeroDatabase(HERE/'runs/flow5_connection_01/aerodynamics/aero_database.npz')
    trim=json.loads((OUT/'trim.json').read_text())
    MODEL=CoupledModel(c,a,trim,phase='mission')


def recover_chunk(path):
    with np.load(path) as d:
        times=d['time'].copy();states=d['states'].copy();flags=d['captured'].copy()
        meta=json.loads(str(d['metadata']))
    MODEL.active_override=meta['active_nodes']
    rows=[]
    for t,y,captured in zip(times,states,flags):
        MODEL.captured=bool(captured);MODEL.capture_time=meta['capture_time_s'] if captured else None
        _,row=MODEL.rhs(t,y,True)
        for index,name in enumerate(('roll_deg','pitch_deg','yaw_deg')):
            row[name]=float(np.rad2deg(euler(y[6:10])[index]))
            row['sensor_'+name]=float(np.rad2deg(euler(y[19:23])[index]))
        row.update(time_s=float(t),altitude_m=float(-y[2]),sensor_altitude_m=float(-y[15]))
        rows.append(row)
    return times,states,rows,meta


def finish_summary(time,frame,base,case):
    for field,force in [('contact_impulse_Ns','contact_N'),('capture_impulse_Ns','capture_N'),('winch_work_J','winch_power_W')]:
        values=frame[force].to_numpy()
        frame[field]=np.r_[0.,np.cumsum(np.diff(time)*(values[1:]+values[:-1])/2)]
    base.update(duration_s=float(time[-1]-time[0]),start_time_s=float(time[0]),end_time_s=float(time[-1]),
        max_tension_N=float(frame.tension_N.max()),max_contact_N=float(frame.contact_N.max()),
        max_capture_N=float(frame.capture_N.max()),max_penetration_m=float(frame.penetration_m.max()),
        contact_impulse_Ns=float(frame.contact_impulse_Ns.iloc[-1]),capture_impulse_Ns=float(frame.capture_impulse_Ns.iloc[-1]),
        max_pitch_change_deg=float(abs(frame.pitch_deg-frame.pitch_deg.iloc[0]).max()),
        minimum_recorded_mesh_gap_m=float(frame.minimum_mesh_gap_m.min()),final_door_deg=float(frame.door_deg.iloc[-1]))
    base['violations']=[]
    for column,section,key in [('tension_N','cable','limit_N'),('torque_Nm','winch','limit_torque_Nm'),('winch_power_W','winch','limit_power_W'),('contact_N','bay','limit_contact_N'),('penetration_m','bay','limit_penetration_m')]:
        limit=case[section].get(key)
        peak=float(frame[column].abs().max() if column=='winch_power_W' else frame[column].max())
        if limit is not None and peak>limit:base['violations'].append(dict(quantity=column,limit=limit,peak=peak))
    return base


if __name__=='__main__':
    chunks=sorted((OUT/'mission/accepted_history').glob('part_*.npz'))
    times=[];states=[];rows=[];identities=[]
    with ProcessPoolExecutor(max_workers=4,initializer=init_worker) as pool:
        for i,(t,y,d,meta) in enumerate(pool.map(recover_chunk,chunks)):
            times.append(t);states.append(y);rows.extend(d);identities.append(meta['fingerprints'])
            if i%10==0:print(f'Recovered {i+1}/{len(chunks)} durable chunks',flush=True)
    assert all(identity==identities[0] for identity in identities)
    t=np.concatenate(times);y=np.vstack(states);assert np.all(np.diff(t)>=0)
    with np.load(OUT/'mission/accepted_checkpoint.tmp') as d:
        assert float(d['time'])==t[-1] and np.array_equal(d['state'],y[-1])
    c=json.loads((OUT/'mission/inputs.json').read_text(encoding='utf8'))
    a=AeroDatabase(HERE/'runs/flow5_connection_01/aerodynamics/aero_database.npz')
    frame=pd.DataFrame(rows)
    base=dict(status='interrupted_io',message='Recovered all durable accepted journal rows up to the complete temporary checkpoint.',
        phase='mission',requested_duration_s=c['simulation']['duration_s'],captured=False,capture_status='not_captured',capture_time_s=None,
        door_capture_interlock=True,door_status='open_or_moving',runtime_s=None,event_sample_format=2,
        numerically_converged=False,physical_validation='unvalidated_assumption_case',geometry_validity='not_audited',
        mesh_contact_enabled=True,clearance_checks=None,minimum_checked_mesh_gap_m=None,
        assumptions=c['provenance'],aero_metadata=a.metadata,
        limitations=['Recovered accepted-state journal; unrecorded integration check counts and runtime unavailable.',
            'Contact loads and cable discretization unvalidated; no aircraft autopilot.'],
        scope='Stored accepted states only; diagnostics reevaluated with each original chunk topology. No reconstructed or forced motion.',
        journal_source=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in chunks],
        fingerprints=identities[0])
    summary=finish_summary(t,frame,base,c)
    events=[dict(time_s=float(t[i]),event=str(frame.phase.iloc[i])) for i in range(len(t)) if i==0 or frame.phase.iloc[i]!=frame.phase.iloc[i-1]]
    target=OUT/'recovered';target.mkdir(exist_ok=False)
    SimulationResult(t,y,frame,summary,events,c).save(target)
    print(json.dumps(dict(end_time_s=float(t[-1]),accepted_rows=len(t))),flush=True)
