"""Static same-pose geometry comparison; explicitly not a new trajectory."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from dbf_stability import changed
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation
from dbf_stability.door import door_hinge, door_reference_shift
from dbf_stability.collision import MeshContacts
from extract_h1 import HERE, read_glb


def main():
    result=load_result(HERE/'runs/contact_corrected_06/baseline/mission')
    data=json.loads((HERE/'diagnostics/door_angles/geometry_comparison.json').read_text(encoding='utf8'))
    row=min((r for r in data['100']['rows'] if r['time_s']>=5.3 and r['sensor_door_gap_m'] is not None),key=lambda r:r['sensor_door_gap_m'])
    i=int(np.argmin(abs(result.time-row['time_s'])));y=result.states[i]
    ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3]);att=ra.T@rs
    raw=read_glb(HERE/'meshes/H1_A_temporary_assembly.glb',None)
    parts={k:(v*[1,-1,-1],f) for k,(v,f) in raw.items()}
    fig,axs=plt.subplots(1,2,figsize=(12,4),sharex=True,sharey=True)
    records=[]
    for ax,angle,modified in zip(axs,[100,140],[False,True]):
        c=result.config if not modified else changed(result.config,**{'bay.door_hinge_offset_m':[-.003,0,.003],'bay.door_closed_offset_m':[-.003,0,0]})
        b=c['bay'];h=door_hinge(b)
        for name,(v,f) in parts.items():
            if name.startswith('sensor_fin') or name=='sensor_body':
                v=(v-np.array(b['stowed_center_m']))@att.T+center;color='#dfa020';zorder=3
            elif name=='rear_door_100deg':
                v=(v+door_reference_shift(b)-h)@Rotation.from_euler('y',angle-100,degrees=True).as_matrix().T+h;color='#2d87ba';zorder=2
            elif name in ('guide_floor','guide_ceiling','rear_exit_frame'):
                color='#74808b';zorder=1
            else:continue
            ax.add_collection(PolyCollection(v[f][:,:,[0,2]]*[1,-1],facecolors=color,edgecolors='none',zorder=zorder))
        ax.scatter(h[0],-h[2],s=25,color='#142b3e',zorder=5)
        world=MeshContacts(c);world.obstacles=[o for o in world.obstacles if o['door']]
        hits=world.contacts(center,att,np.empty((0,3)),angle,distance=.15);world.close()
        gap=min(h.get('geometry_gap',h['gap']) for h in hits)
        records.append({'angle_deg':angle,'modified_mount':modified,'sensor_door_gap_m':float(gap)})
        ax.set_title(f"{'Revised mount' if modified else 'Original mount'} / {angle} deg\nSensor-door gap: {gap*1000:.2f} mm",fontsize=12)
        ax.set(xlim=(-.85,-.39),ylim=(-.135,.035),xlabel='Forward x / m',aspect='equal')
        ax.grid(alpha=.15);ax.set_axisbelow(True)
    axs[0].set_ylabel('Up / m')
    fig.suptitle(f'Same recorded sensor pose at {result.time[i]:.2f} s | geometry comparison, NOT recalculated motion',fontsize=12)
    fig.text(.5,.02,'Orange: sensor and fins   Blue: door   Gray: container frame   Black dot: hinge',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.045,1,.92))
    output=HERE/'diagnostics/door_angles';fig.savefig(output/'same_pose_comparison.png',dpi=170);plt.close(fig)
    (output/'same_pose_revised_mount.json').write_text(json.dumps({'saved_sensor_time_s':float(result.time[i]),'new_motion_prediction':False,'comparison':records},indent=2),encoding='utf8')
    print(json.dumps(records,indent=2))


if __name__=='__main__':main()
