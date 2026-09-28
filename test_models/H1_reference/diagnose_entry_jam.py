"""Inspect the first recovery obstruction, using saved physical states."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'): os.environ[key]='1'
from pathlib import Path
import json, sys
import numpy as np
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]
RUN=HERE/'runs/underbody_mission_02'
sys.path.insert(0,str(RUN/'solver'))
from dbf_stability import AeroDatabase
from dbf_stability.analysis import load_result
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import rotation
from dbf_stability.contact_batch import sensor_contact_loads
from dbf_stability.collision import read_binary_stl

def main():
    out=HERE/'runs/entry_redesign_diagnosis';out.mkdir(exist_ok=True)
    r=load_result(RUN/'complete');c=r.config
    source=json.loads((RUN/'source.json').read_text())
    trim=json.loads((RUN/'trim.json').read_text())
    m=CoupledModel(c,AeroDatabase(source['aero_path']),trim)
    records=[]
    for request in [69.,69.8,70.,70.05,70.1,70.15,70.2,70.3,70.5,71.]:
        i=int(np.argmin(abs(r.time-request)));t=float(r.time[i]);y=r.states[i]
        hits=m.contact_geometry(t,y);_,d=m.rhs(t,y,True)
        pairs=[];ra=rotation(y[6:10]);rs=rotation(y[19:23])
        for moving,fixed in sorted(set((h['moving'],h['fixed']) for h in hits)):
            group=[h for h in hits if (h['moving'],h['fixed'])==(moving,fixed)]
            batch=sensor_contact_loads(group,y[:13],y[13:26],ra,rs,c['collision']['skin_m'],c['collision'].get('part_materials',{}),c['collision'],c['bay']['friction'],0.,m._mesh_contacts.hinge)
            closest=min(group,key=lambda h:h.get('geometry_gap',h['gap']))
            pairs.append(dict(moving=moving,fixed=fixed,gap_mm=1000*closest.get('geometry_gap',closest['gap']),point_m=np.asarray(closest['point']).tolist(),normal=np.asarray(closest['normal']).tolist(),contact_N=0. if batch is None else batch['contact_N']))
        records.append(dict(time_s=t,center_m=(ra.T@(y[13:16]-y[:3])).tolist(),pairs=pairs))
    (out/'contacts.json').write_text(json.dumps(records,indent=2))
    print(json.dumps(records),flush=True)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection,PolyCollection
    fig,axes=plt.subplots(2,2,figsize=(14,10),sharex=True,sharey=True)
    meshdir=ROOT/c['collision']['mesh_directory']
    for ax,request in zip(axes.flat,[69.8,70.1,70.3,71.]):
        for path in meshdir.glob('*_FRD_m.stl'):
            name=path.name.removesuffix('_FRD_m.stl')
            if not name.startswith(('fuselage','guide','rear_exit','capture_')):continue
            v,f=read_binary_stl(path);lines=[]
            for tri in v[f]:
                points=[]
                for a,b in zip(tri,np.roll(tri,-1,axis=0)):
                    if abs(a[1])<1e-8:points.append(a)
                    if a[1]*b[1]<0:points.append(a-a[1]*(b-a)/(b[1]-a[1]))
                if len(points)>1:lines.append(np.asarray(points)[:,[0,2]]*1000)
            if lines:ax.add_collection(LineCollection(lines,colors='#647782',linewidths=1))
        i=int(np.argmin(abs(r.time-request)));y=r.states[i];ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3])
        for part in m._mesh_contacts.sensor:
            v=part['vertices']@(ra.T@rs).T+center
            ax.add_collection(PolyCollection(v[part['faces']][:,:,[0,2]]*1000,facecolors='#dc982b',edgecolors='none',alpha=.18))
        k=m.active_count(m.length(float(r.time[i]))[0]);nodes=(y[26:].reshape(m.n,6)[:k,:3]-y[:3])@ra
        chain=np.vstack([center+ra.T@rs@m.sensor_attach,nodes,m.attach]);ax.plot(chain[:,0]*1000,chain[:,2]*1000,color='#a42c32')
        ax.set_title(f'{r.time[i]:.4f} s');ax.set_xlim(-820,-360);ax.set_ylim(180,-60);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('Body x [mm]');ax.set_ylabel('Body z, downward [mm]')
    fig.tight_layout();fig.savefig(out/'entry_sections.png',dpi=150);m._mesh_contacts.close()

if __name__=='__main__':main()
