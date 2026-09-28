"""Replay the actual saved coupled states with this H1 CAD, not generic aircraft lines."""
from pathlib import Path
import json
import argparse
import os
from html import escape
from urllib.parse import quote
import numpy as np
import plotly.graph_objects as go
from plotly.utils import PlotlyJSONEncoder
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation, attitude_error
from dbf_stability.door import door_hinge, door_reference_shift
from extract_h1 import HERE,read_glb
from geometry_paths import mesh_directory

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run', type=Path)
    parser.add_argument('--diagnose', action='store_true')
    args=parser.parse_args()
    out=args.run
    if out is None:
        latest=json.loads((HERE/'latest_run.json').read_text(encoding='utf8'))
        out=Path(latest['mission_path']) if latest.get('mission_path') else Path(latest['path'])/'mission'
    r=load_result(out);c=r.config
    invalid=r.summary.get('geometry_validity')=='invalid'
    audit_path=out/'cad_collision_audit.json'
    audit=json.loads(audit_path.read_text(encoding='utf8')) if audit_path.exists() else None
    audited={row['index']:row['hits'] for row in audit['rows']} if audit else {}
    no_overlap=bool(audit and not audit['intersecting_samples'] and not audit.get('cad_query_error_samples'))
    diagnosis_path=out/'door_diagnosis.json'
    limiting=json.loads(diagnosis_path.read_text(encoding='utf8')).get('nearest_end_pair') if diagnosis_path.exists() and r.summary['status']=='contact_clearance_limit' else None
    parts=read_glb(mesh_directory(c)/'H1_A_temporary_assembly.glb',None)
    parts={k:(v*[1,-1,-1],f) for k,(v,f) in parts.items()}
    sensor_names=[n for n in parts if n=='sensor_body' or n.startswith('sensor_fin_')];sensor_parts=[parts[n] for n in sensor_names]
    vertices=[];faces=[];count=0
    for v,f in sensor_parts:vertices.append(v);faces.append(f+count);count+=len(v)
    sensor_local=np.vstack(vertices)-np.array(c['bay']['stowed_center_m']);sensor_faces=np.vstack(faces)
    door,door_faces=parts['rear_door_100deg'];b=c['bay'];hinge=door_hinge(b)
    door=door+door_reference_shift(b)
    data=[]
    def coords(v):return dict(x=v[:,0],y=v[:,1],z=-v[:,2])
    def mesh(v,f,name,color,opacity=1):
        return go.Mesh3d(**coords(v),i=f[:,0],j=f[:,1],k=f[:,2],name=name,color=color,opacity=opacity,showlegend=False)
    def wire(chain):
        # Render the actual diameter; a fixed 5-pixel line obscures sub-mm gaps.
        vertices=[];faces=[];sides=12;radius=c['cable']['diameter_m']/2
        angles=np.arange(sides)*2*np.pi/sides
        for a,z in zip(chain[:-1],chain[1:]):
            direction=z-a;length=np.linalg.norm(direction)
            if length<1e-10:continue
            direction/=length
            reference=np.array([1.,0,0]) if abs(direction[0])<.9 else np.array([0.,1,0])
            u=np.cross(direction,reference);u/=np.linalg.norm(u);v=np.cross(direction,u)
            ring=radius*(np.cos(angles)[:,None]*u+np.sin(angles)[:,None]*v)
            offset=len(vertices);vertices.extend(a+ring);vertices.extend(z+ring)
            for j in range(sides):
                k=(j+1)%sides
                faces.extend([[offset+j,offset+k,offset+sides+j],[offset+k,offset+sides+k,offset+sides+j]])
            for j in range(1,sides-1):
                faces.extend([[offset,offset+j+1,offset+j],[offset+sides,offset+sides+j,offset+sides+j+1]])
        return mesh(np.asarray(vertices).reshape(-1,3),np.asarray(faces,dtype=int).reshape(-1,3),'와이어 (실제 직경)','#bf3d37')
    for name,(v,f) in parts.items():
        if name in sensor_names or name in ('rear_door_100deg','sensor_tow_point','aircraft_tow_point'):continue
        if args.diagnose and ('wing' in name or 'tail' in name or 'rudder' in name or 'fin_CAD' in name or 'aileron' in name or 'elevator' in name):continue
        color='#249f78' if name=='recovery_funnel' else '#b94d61' if name=='capture_front_stop' else '#4c959c' if name.startswith('fuselage') else '#bcc6bd' if 'wing' in name or 'tail' in name else '#687782'
        data.append(mesh(v,f,name,color,.18 if name.startswith('fuselage') else .3 if name=='recovery_funnel' else .55 if name.startswith('guide') else .8 if name=='capture_front_stop' else 1))
    ids=list(range(len(data),len(data)+5))
    def state(i):
        y=r.states[i];ra=rotation(y[6:10]);rs=rotation(y[19:23]);a=y[:3]
        center=ra.T@(y[13:16]-a);sensor=sensor_local@(ra.T@rs).T+center
        angle=np.deg2rad(float(r.table.door_deg.iloc[i])-100);co,si=np.cos(angle),np.sin(angle)
        rot=np.array([[co,0,si],[0,1,0],[-si,0,co]]);door_now=(door-hinge)@rot.T+hinge
        n=c['cable']['segments'];nodes=(y[26:].reshape(n,6)[:,:3]-a)@ra
        nose=ra.T@(y[13:16]+rs@np.array(c['sensor']['tow_point_m'])-a)
        k=int(r.table.active_nodes.iloc[i]);cable=np.vstack([nose,nodes[:k],c['aircraft']['tow_point_m']])
        return sensor,door_now,cable
    def collision_marks(i):
        hits=audited.get(i,[])
        points=np.array([h['point_frd_m'] for h in hits]).reshape(-1,3)
        return go.Scatter3d(**coords(points),mode='markers',name='CAD overlap',marker=dict(color='#e00030',size=7,symbol='x'),
                            text=[f"{h['moving']} / {h['fixed']}<br>{h['overlap_mm3']:.3f} mm³ overlap" for h in hits],hoverinfo='text',showlegend=False)
    def collision_labels(i):
        labels={'guide_floor':'컨테이너 바닥', 'fuselage_H1_approx_with_rear_cutout':'동체 외피', 'winch_drum':'윈치'}
        annotations=[dict(x=h['point_frd_m'][0],y=h['point_frd_m'][1],z=-h['point_frd_m'][2],
                     text=f"{'센서' if h['moving']=='sensor' else '와이어'} ↔ {labels.get(h['fixed'],h['fixed'])} 겹침",
                     showarrow=True,arrowcolor='#b42318',arrowhead=2,ax=100,ay=-65 if j%2 else 65,
                            bgcolor='white',font=dict(color='#b42318',size=13)) for j,h in enumerate(audited.get(i,[]))]
        if i==len(r.time)-1 and limiting:
            p=limiting['point_frd_m']
            names={'sensor_fin_3':'센서 상부 핀','sensor_body':'센서 몸체','rear_exit_frame_ceiling':'출구 상단',
                   'rear_exit_frame_floor':'출구 하단','rear_door_100deg':'도어','winch_drum':'윈치',
                   'guide_ceiling':'컨테이너 천장','guide_floor':'컨테이너 바닥'}
            moving=names.get(limiting['moving'],limiting['moving']);fixed=names.get(limiting['fixed'],limiting['fixed'])
            annotations.append(dict(x=p[0],y=p[1],z=-p[2],text=f"{moving} ↔ {fixed}<br>간격 {limiting['gap_m']*1000:.3f}mm에서 중단",
                showarrow=True,arrowcolor='#a35413',arrowhead=2,ax=70,ay=-80,bgcolor='#fff4e5',font=dict(color='#713808',size=13)))
        return annotations
    def label_trace(i):
        labels=collision_labels(i)
        x=[];y=[];z=[];text=[]
        for a in labels:
            x.extend([a['x'],a['x']-.04,None]);y.extend([a['y'],a['y'],None]);z.extend([a['z'],a['z']+.075,None])
            text.extend(['',a['text'],''])
        return go.Scatter3d(x=x,y=y,z=z,text=text,mode='lines+markers+text',textposition='top left',
            textfont=dict(size=12,color='#713808'),marker=dict(size=3,color='#a35413'),
            line=dict(color='#a35413',width=2),showlegend=False,hoverinfo='text')
    start=len(r.time)-1 if args.diagnose else 0
    s,d,line=state(start)
    data += [mesh(s,sensor_faces,'Sensor','#e8a12b'),mesh(d,door_faces,'Door','#2d87ba'),wire(line),collision_marks(start),label_trace(start)]
    # Contact output is deliberately dense and nonuniform. Choose frames by
    # physical time, not row number; each displayed pose is still a saved state.
    from replay_sampling import replay_indices
    idx=replay_indices(r);frames=[]
    static=np.vstack([v for v,_ in parts.values()]);lower=static.min(0);upper=static.max(0)
    for i in idx:
        s,d,line=state(i);hits=audited.get(int(i))
        visible=np.vstack([s,d,line]);lower=np.minimum(lower,visible.min(0));upper=np.maximum(upper,visible.max(0))
        label=f"{r.time[i]:.2f}초 · 줄 {r.table.length_m.iloc[i]:.2f}m · CAD 겹침 {len(hits)}곳" if hits is not None else f"{r.time[i]:.2f}초 — 이 시각 CAD 미검사"
        profile=c.get('mission_profile')
        if profile:
            t=float(r.time[i]);full=profile['deployment_complete_s'];recovery=profile['recovery_start_s']
            stage='전개 중' if t<full else f'전개 유지 {t-full:.1f}/{profile["hold_duration_s"]:g}초' if t<recovery else '회수 중' if t<profile['retrieval_complete_s'] else '회수 명령 종료'
            label+=' · '+stage
        if i==len(r.time)-1 and limiting:label+=' · '+collision_labels(int(i))[-1]['text'].replace('<br>',' · ')
        traces=[go.Mesh3d(**coords(s)),go.Mesh3d(**coords(d)),wire(line),collision_marks(int(i)),label_trace(int(i))]
        frames.append(dict(time=float(r.time[i]),status=label,
            data=[{k:v for k,v in trace.to_plotly_json().items() if k in ('x','y','z','i','j','k','text')} for trace in traces]))
    close_visibility=[not any(word in (trace.name or '') for word in ('wing','tail','rudder','fin_CAD','aileron','elevator')) for trace in data]
    inside_visibility=[visible and not (trace.name or '').startswith(('fuselage','guide_side_','capture_ramp_side_-1','capture_ramp_top')) for visible,trace in zip(close_visibility,data)]
    full_ranges=[[-2.7,.65],[-1,1],[-.8,.45]]
    # Determine one range for the entire saved trajectory, never per frame.
    lo=np.array([lower[0],lower[1],-upper[2]])-.08
    hi=np.array([upper[0],upper[1],-lower[2]])+.08
    full_ranges=[[min(pair[0],float(a)),max(pair[1],float(b))] for pair,a,b in zip(full_ranges,lo,hi)]
    close_ranges=[[-.9,-.23],[-.10,.10],[-.22,.08]]
    full_camera=dict(eye=dict(x=-1.1,y=-1.4,z=.8),projection=dict(type='perspective'))
    close_camera=dict(eye=dict(x=.05,y=-1.8,z=.55),projection=dict(type='orthographic'))
    def view_layout(ranges,camera):
        spans=np.ptp(np.asarray(ranges),axis=1);ratios=2*spans/max(spans)
        out={'scene.camera':camera,'scene.aspectmode':'manual','scene.aspectratio':dict(zip('xyz',map(float,ratios)))}
        ticks=(.5,.5,.25) if spans[0]>1 else (.1,.05,.05)
        for axis,limits,dtick in zip('xyz',ranges,ticks):
            out[f'scene.{axis}axis.range']=limits;out[f'scene.{axis}axis.autorange']=False
            out[f'scene.{axis}axis.dtick']=dtick
        return out
    views={'full':dict(visible=[True]*len(data),layout=view_layout(full_ranges,full_camera)),
        'close':dict(visible=close_visibility,layout=view_layout(close_ranges,close_camera)),
        'inside':dict(visible=inside_visibility,layout=view_layout(close_ranges,close_camera))}
    initial_view='full' if c.get('mission_profile') else 'inside'
    if r.time[0]>=c['winch']['recovery_start_s']:initial_view='inside'
    initial=views[initial_view]['layout']
    for trace,visible in zip(data,views[initial_view]['visible']):trace.visible=visible
    fig=go.Figure(data=data)
    fig.update_layout(title=dict(text=('물리적으로 무효인 궤적' if invalid else '기체 기준 고정 축 · 실제 계산 재생'),x=.02,y=.98,font=dict(size=17)),
        height=630,margin=dict(t=80,b=25,l=20,r=20),uirevision='h1-replay-fixed-view',showlegend=False,
        scene=dict(xaxis=dict(title='CG forward x / m',range=initial['scene.xaxis.range'],autorange=False,tickmode='linear',tick0=0,dtick=initial['scene.xaxis.dtick']),
            yaxis=dict(title='Right y / m',range=initial['scene.yaxis.range'],autorange=False,tickmode='linear',tick0=0,dtick=initial['scene.yaxis.dtick']),
            zaxis=dict(title='Up / m',range=initial['scene.zaxis.range'],autorange=False,tickmode='linear',tick0=0,dtick=initial['scene.zaxis.dtick']),
            aspectmode='manual',aspectratio=initial['scene.aspectratio'],camera=initial['scene.camera'],uirevision='h1-camera'))
    capture_label='성공' if r.summary['captured'] else '미도달' if r.time[-1]<=c['winch']['recovery_start_s'] else '안 됨'
    if r.summary['status']=='partial_snapshot' and not r.summary['captured']:capture_label='진행 중 · 아직 미감지'
    if invalid:
        fig.add_annotation(x=.5,y=1.04,xref='paper',yref='paper',showarrow=False,
                           text='빨간 ×: CAD가 실제로 겹친 위치 · 충돌 계산 수정 전 결과 · 설계 하중으로 사용 불가',
                           bgcolor='#fff0ec',font=dict(color='#b42318',size=13))
    elif no_overlap:
        fig.add_annotation(x=.5,y=1.04,xref='paper',yref='paper',showarrow=False,
             text=f"실제 CAD {audit['samples']}시각 검사 · 겹침 0건 · 포획 {capture_label} · 실물/충격하중 미검증",
             bgcolor='#eaf4f1',font=dict(color='#216757',size=13))
    payload=dict(frames=frames,traces=ids,views=views,start=len(frames)-1 if args.diagnose else 0)
    controller=(HERE/'replay_controller.js').read_text(encoding='utf8').replace('__REPLAY_PAYLOAD__',json.dumps(payload,cls=PlotlyJSONEncoder,ensure_ascii=False))
    page=fig.to_html(include_plotlyjs=True,auto_play=False,div_id='h1-replay',post_script=controller,config={'responsive':True})
    display_name='고익기 R3' if c['name'].startswith('R3_') else c['name']
    page=page.replace('<head>',f'<head><title>{escape(display_name)} · 전개·회수 계산 재생</title>',1)
    controls=f'''<section class="replay-controls" aria-label="재생 제어">
      <div><button id="replay-play">계산 결과 재생</button><button id="replay-pause">일시정지</button>
      <label for="replay-speed">재생 속도</label> <select id="replay-speed"><option value="0.25">0.25배</option><option value="0.5">0.5배</option><option value="1" selected>1배</option><option value="5">5배</option><option value="10">10배</option></select>
      <span class="view-buttons"><button data-replay-view="full" aria-pressed="{str(initial_view=='full').lower()}">기체 전체</button><button data-replay-view="close" aria-pressed="false">컨테이너 확대</button><button data-replay-view="inside" aria-pressed="{str(initial_view=='inside').lower()}">내부 보기 (외피·측벽 숨김)</button></span></div>
      <p id="replay-readout"></p><label for="replay-position">재생 위치</label>
      <input id="replay-position" type="range" min="0" max="{len(frames)-1}" step="1" value="{payload['start']}">
      <small>축과 배율은 고정됩니다. 해석 시각에 맞춰 재생하며, 시점은 마우스나 보기 버튼으로 바꿀 수 있습니다.</small>
      </section><style>.replay-controls{{margin:16px 28px;font:14px/1.6 system-ui,sans-serif}}.replay-controls button{{padding:8px 14px;background:#fff;border:1px solid #a8b7c0;border-radius:5px;cursor:pointer;margin:0 5px 5px 0}}.replay-controls button[aria-pressed=true]{{background:#e8f3f4;border-color:#247782}}.replay-controls button:disabled{{opacity:.55;cursor:default}}.view-buttons{{margin-left:28px}}#replay-position{{width:100%;accent-color:#247782}}#replay-readout{{min-height:24px;margin:8px 0;font-weight:600}}.replay-controls small{{color:#526773}}</style>''' 
    aero_solver=escape(r.summary['aero_metadata']['solver'].replace('flow5 flow5','flow5'))
    is_flow5='flow5' in aero_solver.lower()
    avl_raw=Path(r.summary['aero_metadata']['raw_directory'])/('job_000/stdout.txt' if is_flow5 else 'case_0000/derivatives.txt')
    avl_record=quote(Path(os.path.relpath(avl_raw,out)).as_posix(),safe='/')
    status=r.summary['status']
    stop_labels={'contact_clearance_limit':'관통 방지를 위한 최소 거리 한계',
        'numerical_failure':'운동 계산 수치 오류','contact_domain_exceeded':'입력 형상의 간섭',
        'aero_domain_exceeded':'공력표 적용 범위 이탈','geometry_invalid':'저장 궤적의 CAD 관통',
        'numerical_stagnation':'계산 진행 정체','runtime_limit':'설정한 계산 시간 한도','evaluation_limit':'설정한 계산 횟수 한도'}
    stop_labels['interrupted_io']='저장 중단 전 기록 (별도 구간에서 재개)'
    stop_labels['ground_contact']='기체 또는 센서가 지면에 도달'
    status_text=('요청 시간까지 계산 완료' if status=='completed' else
        '계산 중단: '+stop_labels.get(status,status))
    if status=='partial_snapshot':
        status_text='계산된 구간 미리보기 · 전체 임무 계산 중'
    status_text+=' · 포획 '+capture_label
    requested_end=r.summary['start_time_s']+r.summary['requested_duration_s']
    status_color='#216757' if status=='completed' and no_overlap and r.summary['captured'] else '#8a4b08'
    limit_names={'tension_N':'줄 장력','torque_Nm':'윈치 토크','winch_power_W':'윈치 출력'}
    exceeded=[limit_names.get(v['quantity'],v['quantity']) for v in r.summary.get('violations',[])]
    limit_note=(f'<span style="color:#8a4b08">입력한 한계 초과: {escape(", ".join(exceeded))}</span><br>'
                if exceeded else '')
    door_note=''
    if 'door_hinge_offset_m' in b:
        hinge_note=('힌지 핀·브래킷·받침 형상을 반영했습니다. 구동기 응답과 체결 강도는 검증하지 않았습니다.'
                    if 'door_hinge_pin' in parts else '힌지·구동장치의 상세 구조는 포함하지 않았습니다.')
        door_note=(f"<strong>문 최대 개방각 {max(v for _,v in c['winch']['door_schedule']):g}°</strong> · "
                   '이 계산에 입력한 문판·힌지 위치를 적용한 임시 형상입니다. '
                   +hinge_note+'<br>')
    if not r.summary['captured'] and r.time[-1]>c['winch']['recovery_start_s'] and r.table.door_deg.iloc[-1]==0:
        door_note+='문은 포획과 연동하지 않은 지정 이력에 따라 닫혔습니다.<br>'
    if r.summary.get('door_status')=='held_open_capture_required':
        door_note+='포획 조건을 충족하지 못해 문 닫힘을 보류하고 열린 상태를 유지했습니다.<br>'
    if r.summary.get('scope'):
        door_note+=escape(r.summary['scope'])+'<br>'
    if c.get('mission_profile'):
        p=c['mission_profile']
        door_note+=f"전개 길이 {p['deployed_length_m']:.2f}m (윙스팬 {p['wingspan_m']:.2f}m × {p['wire_span_ratio']:g}). 전개 완료 {p['deployment_complete_s']:.2f}초 → {p['hold_duration_s']:g}초 유지 → 회수 시작 {p['recovery_start_s']:.2f}초.<br>"
    solver_note=('flow5 VLM2: 날개·꼬리날개 공력이며 동체 공력은 포함하지 않았습니다. ' if is_flow5 else '')
    provenance=(f'<section aria-label="해석 출처" style="font:14px/1.6 system-ui,sans-serif;'
        f'margin:16px 20px 0;padding:12px 16px;background:#f2f5f6;border-radius:8px">'
        f'<strong style="color:{status_color}">{escape(status_text)} · '
        f'{r.time[0]:.2f}–{r.time[-1]:.2f}초 (요청 종료 {requested_end:.2f}초)</strong><br>'
        f'{limit_note}{door_note}출구 폭 {2*b["half_width_m"]*1000:g}mm · 높이 {(b["floor_z_m"]-b["ceiling_z_m"])*1000:g}mm. 주황색은 센서, 파란색은 도어입니다.<br>'
        f'<strong>기체 양력면 공력: {aero_solver} 실제 계산 결과</strong><br>'
        'Python은 저장된 공력에 센서·와이어·접촉력을 더해 운동을 계산합니다. '
        f'{solver_note}<a href="{avl_record}" target="_blank" rel="noopener">이 공력표의 해석기 원본 기록</a><br>'
        '<span style="color:#65502c">센서·줄 공력, 추가 항력, 후류는 가정값이며 '
        '실물 시험으로 검증하지 않았습니다.</span></section>')
    page=page.replace('<body>','<body>'+provenance+controls,1)
    (out/('H1_collision_diagnosis.html' if args.diagnose else 'H1_replay.html')).write_text(page,encoding='utf8')
    if args.diagnose:
        return
    y=r.states[-1];ra=rotation(y[6:10]);local=ra.T@(y[13:16]-y[:3]);err=np.linalg.norm(local-np.array(b['stowed_center_m']))
    angle=float(np.rad2deg(np.linalg.norm(attitude_error(y[6:10],y[19:23]))))
    finding={'final_position_error_m':float(err),'capture_position_limit_m':b['capture_radius_m'],
       'final_relative_speed_m_s':float(r.table.sensor_relative_speed_m_s.iloc[-1]),'capture_speed_limit_m_s':b['capture_speed_m_s'],
       'final_relative_angle_deg':angle,'capture_angle_limit_deg':b['capture_angle_deg'],
       'final_sensor_CG_frd_m':local.tolist(),'bay_floor_frd_z_m':b['floor_z_m'],
       'captured':r.summary['captured'],'geometry_validity':r.summary.get('geometry_validity'),
       'note':'Recorded endpoint. Axial exposure does not prove in-bay capture. Use the captured flag, actual pose, and independent CAD audit together.'}
    (out/'capture_diagnosis.json').write_text(json.dumps(finding,indent=2),encoding='utf8')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cg=np.array(c['aircraft']['cg_m'])*[-1,1,-1]
    x=cg[0]-r.table.sensor_local_x_m.values;z=cg[2]-r.table.sensor_local_z_m.values
    fig,axs=plt.subplots(2,1,figsize=(10,7),gridspec_kw={'height_ratios':[1.15,1]})
    sc=axs[0].scatter(x,z,c=r.time,s=10,cmap='viridis');fig.colorbar(sc,ax=axs[0],label='Recorded time / s')
    shape_record_path=mesh_directory(c).parent/'geometry.json'
    shape_record=json.loads(shape_record_path.read_text(encoding='utf8')) if shape_record_path.exists() else {}
    if 'stations_old_frd_m' in shape_record:
        stations=np.asarray(shape_record['stations_old_frd_m']);shift=np.asarray(shape_record['cg_shift_m'])
        for column in (2,3):
            axs[0].plot(cg[0]-(stations[:,0]-shift[0]),cg[2]-(stations[:,column]-shift[2]),color='#555',lw=4)
    else:
        for zz in [b['floor_z_m'],b['ceiling_z_m']]:
            axs[0].plot([cg[0]-b['front_x_m'],cg[0]-b['exit_x_m']],[cg[2]-zz]*2,color='#555',lw=4)
    target=cg-np.array(b['stowed_center_m'])*[1,-1,1]
    axs[0].scatter([target[0]],[target[2]],marker='x',s=100,color='red',label='Capture target')
    axs[0].set(xlabel='Original aft x / m',ylabel='Original up z / m',title=f"Computed sensor CG trajectory | captured={r.summary['captured']}")
    axs[0].legend(fontsize=8)
    if r.summary.get('mesh_contact_enabled'):
        skin=1000*c['collision']['skin_m'];minimum=1000*c['collision']['minimum_gap_m']
        axs[1].plot(r.time,1000*r.table.minimum_mesh_gap_m,
                    label=f'Mesh gap (capped at query skin: {skin:g} mm)',color='#24736b')
        axs[1].axhline(minimum,color='#a85113',ls='--',label=f'Stop threshold: {minimum:g} mm')
        axs[1].axhline(0,color='#aa382a',lw=.8)
        axs[1].set(xlabel='Time / s',ylabel='Geometric gap / mm')
    else:
        axs[1].plot(r.time,1000*r.table.penetration_m,label='Contact penetration / mm',color='#aa382a')
        axs[1].axhline(2,color='#555',ls='--',label='Assumed 2 mm limit')
        axs[1].set(xlabel='Time / s',ylabel='Penetration / mm')
    axs[1].legend();fig.tight_layout()
    fig.savefig(out/'trajectory_and_contact.png',dpi=160);plt.close(fig)
    print(json.dumps(finding,indent=2))

if __name__=='__main__':main()
