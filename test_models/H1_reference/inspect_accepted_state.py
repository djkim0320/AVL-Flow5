"""Read an immutable accepted-history chunk, never the live checkpoint."""
from pathlib import Path
import argparse,json
import numpy as np
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import rotation,attitude_error
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('parent',type=Path);args=p.parse_args()
c=load_case(args.parent/'case.yaml');source=sorted((args.parent/'mission/accepted_history').glob('part_*.npz'))[-1]
with np.load(source,allow_pickle=False) as d:t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
m=CoupledModel(c,a,meta['trim']);m.captured=meta['captured'];m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes']
_,diag=m.rhs(t,y,True);r=rotation(y[6:10]);position=r.T@(y[13:16]-y[:3]);nodes=(y[26:].reshape(m.n,6)[:m.active_override,:3]-y[:3])@r
contacts={}
for h in m.contact_geometry(t,y):
 key=h['moving']+'/'+h['fixed'];row=contacts.setdefault(key,dict(count=0,minimum_gap_m=1.));row['count']+=1;row['minimum_gap_m']=min(row['minimum_gap_m'],h['gap'])
print(json.dumps(dict(time_s=t,captured=m.captured,capture_time_s=m.capture_time,position_error_mm=float(np.linalg.norm(position-c['bay']['stowed_center_m'])*1000),
 relative_angle_deg=float(np.rad2deg(np.linalg.norm(attitude_error(y[6:10],y[19:23])))),nodes_local_m=nodes.tolist(),contacts=contacts,
 diagnostics={k:v for k,v in diag.items() if np.isscalar(v)}),indent=2),flush=True)
m._mesh_contacts.close()
