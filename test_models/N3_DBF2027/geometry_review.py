"""Inspectable geometry checks and drawings for the new conceptual airframe."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go

HERE=Path(__file__).resolve().parent
parts=json.loads((HERE/'cad_parts.json').read_text('utf8'))
data=np.load(HERE/'cad_meshes.npz');dims=json.loads((HERE/'dimensions.json').read_text('utf8'))
fig=plt.figure(figsize=(12,7),facecolor='#f1f5f9');ax=fig.add_subplot(projection='3d')
ax.set_facecolor('#f1f5f9');plot=go.Figure()
triangles=[];colors=[]
for i,p in enumerate(parts):
    v=data[f'v{i}']*[1,1,-1];f=data[f'f{i}'];color=p['color']
    triangles.extend(v[f[:,::-1]]);colors.extend([color]*len(f))
    rgb='rgb('+','.join(str(round(255*x)) for x in color[:3])+')'
    plot.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],color=rgb,name=p['name'],hovertemplate=p['name']+'<extra></extra>',showlegend=False))
surface=Poly3DCollection(triangles,facecolors=colors,linewidths=0,shade=True,lightsource=matplotlib.colors.LightSource(azdeg=100,altdeg=45))
surface.set_edgecolors('none');ax.add_collection3d(surface)
ax.set(xlim=(-1.02,.54),ylim=(-.92,.92),zlim=(-.31,.40),xlabel='Forward [m]',ylabel='Right [m]',zlabel='Up [m]')
ax.set_box_aspect((1.56,1.84,.71));ax.set_proj_type('ortho');ax.view_init(elev=22,azim=135)
ax.set_title('N3 | New 1.80 m DBF airframe\nCommon CAD / AVL lifting-surface definition',loc='left',pad=22,fontweight='bold')
fig.text(.10,.035,'270° underbody door · internal winch · raised tail boom · conventional electric tractor layout',fontsize=10,color='#334155')
fig.savefig(HERE/'N3_geometry.png',dpi=170,bbox_inches='tight');plt.close(fig)
plot.update_layout(title='N3 새 기체 · 1.80 m / 후방 화물칸 / 270° 문',paper_bgcolor='#f1f5f9',margin=dict(l=0,r=0,t=50,b=0),
    scene=dict(aspectmode='data',xaxis_title='전방 m',yaxis_title='우측 m',zaxis_title='상방 m',camera=dict(eye=dict(x=1.4,y=1.7,z=.85))))
plot.write_html(HERE/'N3_geometry.html',include_plotlyjs=True)

cg=np.array(dims['cg_aru_m'])
# Reservation boxes, not sensor CAD parts or modeled contacts. Check their
# eight corners against the constant superellipse cavity and top of the floor.
checks=[]
for name,center,size in [('M2_container',[.54,0,-.015],[.280,.130,.130]),('M3_sensor',[.71,0,-.037],[.220,.090,.100])]:
    center=np.array(center);size=np.array(size)
    corners=center+np.array([[a,b,c] for a in (-.5,.5) for b in (-.5,.5) for c in (-.5,.5)])*size
    q=(abs(corners[:,1])/.106)**(2/.65)+(abs(corners[:,2])/.109)**(2/.65)
    inside=bool((q<=1).all() and corners[:,0].min()>=.28 and corners[:,0].max()<=.91 and corners[:,2].min()>=-.0885)
    checks.append(dict(reservation=name,center_aru_m=center.tolist(),size_m=size.tolist(),inside_shell_and_above_floor=inside,
        maximum_superellipse_measure=float(q.max()),floor_clearance_m=float(corners[:,2].min()+.0885)))
audit=dict(CAD_valid_solids_checked_by_design_script=40,CAD_triangle_count=sum(len(data[f'f{i}']) for i in range(len(parts))),
    span_m=float(np.ptp(np.concatenate([data[f'v{i}'] for i in range(len(parts))])[:,1])),
    span_limit_m=1.8288,span_margin_m=1.8288-dims['span_m'],reserve_boxes=checks,
    limitations=['Envelope fit is not a deployment/capture collision simulation.','Point payload has no sensor attitude/aerodynamics.','Mass/inertia budget is assumed.'])
(HERE/'geometry_checks.json').write_text(json.dumps(audit,indent=2,ensure_ascii=False),encoding='utf8')
print(json.dumps(audit,ensure_ascii=True),flush=True)
