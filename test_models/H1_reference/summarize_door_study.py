"""Compare complete saved trajectories, including actual sensor/door separation."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse
import json
from html import escape
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dbf_stability.analysis import load_result
from dbf_stability.collision import MeshContacts
from dbf_stability.math3d import rotation


def inspect_run(path):
    path=Path(path);r=load_result(path);world=MeshContacts(r.config)
    y=r.states[-1];ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3])
    posed_sensor=np.vstack([part['vertices'] for part in world.sensor])@(ra.T@rs).T
    projected_height=float(np.ptp(posed_sensor[:,2]))
    nose=center+ra.T@rs@np.array(r.config['sensor']['tow_point_m']);k=int(r.table.active_nodes.iloc[-1])
    chain=np.vstack([nose,(y[26:].reshape(-1,6)[:k,:3]-y[:3])@ra,r.config['aircraft']['tow_point_m']])
    hits=world.contacts(center,ra.T@rs,chain,float(r.table.door_deg.iloc[-1]),distance=.02)
    if hits:
        # Several weighted feature witnesses share a pair's global minimum.
        # Place the annotation at the closest feature, not an arbitrary tie.
        hit=min(hits,key=lambda h:(h.get('geometry_gap',h['gap']),h['gap']))
        nearest={'moving':hit['moving'],'fixed':hit['fixed'],'gap_m':float(hit.get('geometry_gap',hit['gap'])),
                 'point_frd_m':np.asarray(hit['point']).tolist()}
    else:nearest=None
    world.obstacles=[o for o in world.obstacles if o['door']];rows=[]
    for i,y in enumerate(r.states):
        ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3])
        hits=world.contacts(center,ra.T@rs,np.empty((0,3)),float(r.table.door_deg.iloc[i]),distance=.12)
        hit=min(hits,key=lambda h:h.get('geometry_gap',h['gap'])) if hits else None
        rows.append({'time_s':float(r.time[i]),'sensor_door_gap_m':float(hit.get('geometry_gap',hit['gap'])) if hit else None,
                     'part':hit['moving'] if hit else None})
    world.close();pd.DataFrame(rows).to_csv(path/'sensor_door_clearance.csv',index=False)
    recovery=[x for x in rows if x['time_s']>=r.config['winch']['recovery_start_s'] and x['sensor_door_gap_m'] is not None]
    skin=r.config['collision']['skin_m'];engaged=[x for x in recovery if x['sensor_door_gap_m']<skin]
    audit=json.loads((path/'cad_collision_audit.json').read_text(encoding='utf8'))
    result={key:r.summary[key] for key in ('status','message','duration_s','captured','max_tension_N','max_contact_N',
          'contact_impulse_Ns','max_pitch_change_deg','violations','geometry_validity','runtime_s')}
    result.update(door_angle_deg=max(v for _,v in r.config['winch']['door_schedule']),
        min_recovery_sensor_door_gap_m=min((x['sensor_door_gap_m'] for x in recovery),default=None),
        first_recovery_door_contact_skin_entry=engaged[0] if engaged else None,
        recovery_door_contact_sample_count=len(engaged),nearest_end_pair=nearest,
        final_sensor_projected_height_m=projected_height,
        opening_height_m=r.config['bay']['floor_z_m']-r.config['bay']['ceiling_z_m'],
        cad_samples=audit['samples'],cad_overlap_samples=audit['intersecting_samples'],cad_error_samples=audit['cad_query_error_samples'])
    (path/'door_diagnosis.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf8')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--angles',type=int,nargs='+',default=[100,140,160]);args=p.parse_args()
    paths=[args.root/f'angle_{a}'/'mission' for a in args.angles]
    input_sets=[]
    for path in paths:
        c=load_result(path).config
        shared={key:c[key] for key in ('aircraft','sensor','cable','bay','flight','collision','aero','simulation')}
        shared['winch']={key:value for key,value in c['winch'].items() if key!='door_schedule'}
        input_sets.append(shared)
    same_inputs=all(c==input_sets[0] for c in input_sets)
    if not same_inputs:raise ValueError('This comparison requires identical non-angle inputs')
    with ProcessPoolExecutor(max_workers=min(3,len(paths))) as pool:rows=list(pool.map(inspect_run,paths))
    report={'case_start_s':0.,'requested_end_s':7.,'door_closed_offset_frd_m':[-.003,0,0],'door_hinge_offset_frd_m':[-.003,0,.003],
            'same_non_door_inputs':same_inputs,'numerically_converged':False,'physical_validation':'unvalidated_assumption_case','runs':rows}
    (args.root/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    fig=make_subplots(rows=2,cols=1,shared_xaxes=True,subplot_titles=['줄 장력','센서–도어 간격 (120mm 이내인 저장 시각)'])
    table=[]
    for row,path in zip(rows,paths):
        a=row['door_angle_deg'];r=load_result(path);gap=pd.read_csv(path/'sensor_door_clearance.csv')
        fig.add_trace(go.Scatter(x=r.time,y=r.table.tension_N,name=f'{a:g}°',legendgroup=str(a)),row=1,col=1)
        fig.add_trace(go.Scatter(x=gap.time_s,y=gap.sensor_door_gap_m*1000,name=f'{a:g}°',legendgroup=str(a),showlegend=False),row=2,col=1)
        status='7초 계산 완료' if row['status']=='completed' and row['duration_s']>=7-1e-8 else f"{row['duration_s']:.3f}초 중단"
        if row['status']=='numerical_failure':status=f"수치 오류 · {row['duration_s']:.3f}초까지 저장"
        contact='있음' if row['recovery_door_contact_sample_count'] else '없음 (저장 시각 기준)'
        table.append(f"<tr><td><a href='angle_{a:g}/mission/H1_replay.html'>{a:g}° 재생</a></td><td>{status}</td>"
            f"<td>{contact}</td><td>{row['max_tension_N']:.2f} N</td><td>{row['max_contact_N']:.2f} N</td><td>{'포획' if row['captured'] else '미포획'}</td>"
            f"<td>{row['cad_samples']} / {row['cad_overlap_samples']} / {row['cad_error_samples']}</td></tr>")
    fig.add_hline(y=8,line_dash='dash',line_color='#ad4c22',row=1,col=1,annotation_text='입력 장력 한계 8N')
    fig.add_hline(y=.8,line_dash='dash',line_color='#ad4c22',row=2,col=1,annotation_text='수치 접촉 범위 0.8mm')
    fig.update_yaxes(title_text='N',row=1,col=1);fig.update_yaxes(title_text='mm',row=2,col=1);fig.update_xaxes(title_text='시간 / 초',row=2,col=1)
    fig.update_layout(height=680,template='plotly_white',margin=dict(t=60,b=45,l=60,r=30))
    html='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H1 도어 개방각 비교</title><style>body{font:16px/1.6 system-ui,sans-serif;color:#23313b;max-width:1160px;margin:32px auto;padding:0 24px}h1{font-size:26px}table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:12px;border-bottom:1px solid #d7dfe4}th{background:#f2f5f7}a{color:#086b85}aside{padding:16px;background:#fff4e5;border-radius:8px}</style>
<h1>H1 도어 개방각 비교</h1><p>같은 센서·윈치·기체 조건으로 0초부터 각각 다시 계산했습니다. 모든 비교안에서 문판은 프레임 바깥으로 3mm, 힌지는 기존 위치에서 뒤·아래로 3mm 옮겼습니다.</p>
<aside>회수 성공은 포획 여부와 함께 확인해야 합니다. 계산 중단 이후의 움직임은 재생하지 않습니다. 수치 오류 사례의 하중은 마지막 저장 시각까지의 값이며, 짧게 계산됐다는 이유로 더 좋은 설계로 판정하지 않습니다. 도어 명령은 비교를 위해 7초까지 열어 두도록 설정했습니다.</aside>
<table><thead><tr><th>개방각</th><th>운동 계산</th><th>회수 중 센서–도어 접촉</th><th>최대 장력</th><th>최대 수치 접촉력</th><th>포획</th><th>CAD 검사 시각 / 겹침 / 오류</th></tr></thead><tbody>'''+''.join(table)+'''</tbody></table>
<p>접촉력은 실측 물성이 아닌 가정 모델의 값입니다. 표의 하중은 각 궤적의 마지막 저장 시각까지 계산한 최대값입니다.</p>
<p>수정한 도어의 별도 CAD 검사: 0~180°의 181개 자세에서 고정 부품과의 겹침·검사 오류 0건. 저장 자세 검사는 연속 시간 전체의 무간섭 증명이 아닙니다.</p>'''+fig.to_html(full_html=False,include_plotlyjs=True)+'''
<p>기체 양력면은 실제 AVL 3.52 공력표를 사용했습니다. 센서·줄 공력과 후류는 가정값이며, 도어 각도에 따른 후류 변화는 계산하지 않았습니다. flow5는 연결하지 않았습니다. 접촉 하중은 실험으로 검증하지 않았고, 변경한 조건의 시간·공간 분할 수렴도 아직 확인하지 않았습니다.</p>
<p><a href="../../aerodynamics/connection_check/report.html">실제 AVL 실행 근거</a> · <a href="../../diagnostics/door_angles/offset_swing.json">도어 CAD 회전 검사</a></p></html>'''
    (args.root/'comparison.html').write_text(html,encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
