"""Cross-section at the previous stopping pose, not a new motion prediction."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from dbf_stability import load_case
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation
from dbf_stability.collision import MeshContacts

HERE=Path(__file__).resolve().parent


def main():
    r=load_result(HERE/'runs/door_angle_01/angle_140/mission');y=r.states[-1]
    ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3]);att=ra.T@rs
    fig,axs=plt.subplots(1,3,figsize=(14,4.6),sharex=True,sharey=True);rows=[]
    for height,ax in zip((56,70,80),axs):
        c=r.config if height==56 else load_case(HERE.parents[1]/f'examples/h1_opening_{height}.yaml')
        b=c['bay'];world=MeshContacts(c)
        for part in world.sensor:
            v=part['vertices']@att.T+center
            ax.add_collection(PolyCollection(v[part['faces']][:,:,[0,2]]*[1000,-1000],facecolors='#e8a12b',edgecolors='none',zorder=3))
        # Centre-plane section of walls/frame: lateral rails are outside this plane.
        x,front=b['exit_x_m'],b['front_x_m'];floor,ceil=b['floor_z_m'],b['ceiling_z_m'];t=b['wall_thickness_m']
        boxes=[([x,ceil-t],[front,ceil]),([x,floor],[front,floor+t]),
               ([x-.002,ceil-.004],[x+.002,ceil]),([x-.002,floor],[x+.002,floor+.004])]
        for lo,hi in boxes:ax.add_patch(Rectangle((lo[0]*1000,-hi[1]*1000),(hi[0]-lo[0])*1000,(hi[1]-lo[1])*1000,color='#667986',zorder=2))
        world.obstacles=[o for o in world.obstacles if o['name'] in ('guide_ceiling','rear_exit_frame_ceiling')]
        hits=world.contacts(center,att,np.empty((0,3)),140,distance=.10)
        gap=min(h.get('geometry_gap',h['gap']) for h in hits);world.close()
        rows.append({'opening_height_mm':height,'upper_sensor_clearance_mm':gap*1000})
        ax.set_title(f'Opening height {height} mm\nUpper clearance {gap*1000:.2f} mm',fontsize=12)
        ax.set(xlim=(-710,-505),ylim=(-85,35),xlabel='Forward x / mm',aspect='equal')
        ax.grid(alpha=.15);ax.set_axisbelow(True)
    axs[0].set_ylabel('Up / mm')
    fig.suptitle(f'Same sensor pose at {r.time[-1]:.3f} s | centre-plane wall section | NOT recalculated motion',fontsize=12)
    fig.text(.5,.035,'Orange: sensor + fins    Gray: guide and opening frame    Width remains 56 mm; floor position unchanged',ha='center')
    fig.tight_layout(rect=(0,.07,1,.90))
    out=HERE/'diagnostics/opening';out.mkdir(parents=True,exist_ok=True)
    fig.savefig(out/'same_pose_comparison.png',dpi=170);plt.close(fig)
    report={'source_run':str(HERE/'runs/door_angle_01/angle_140/mission'),'source_time_s':float(r.time[-1]),'new_motion_prediction':False,'comparison':rows}
    (out/'same_pose_comparison.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
