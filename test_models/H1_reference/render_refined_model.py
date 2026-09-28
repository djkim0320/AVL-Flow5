"""Inspect the delivered R2 CAD meshes, at fixed metric scales."""
from pathlib import Path
import json
import argparse
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dbf_stability import load_case
from dbf_stability.door import door_hinge,door_reference_shift
from scipy.spatial.transform import Rotation
from extract_h1 import read_glb

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,default=ROOT/'examples/h1_normal_r3.yaml');args=p.parse_args()
    c=load_case(args.case);out=(ROOT/c['collision']['mesh_directory']).parent
    geometry=json.loads((out/'geometry.json').read_text(encoding='utf8'))
    parts=read_glb(out/'meshes/H1_A_temporary_assembly.glb',None)
    parts={name:(v*[1,-1,-1],f) for name,(v,f) in parts.items()}
    h=door_hinge(c['bay']);v,f=parts['rear_door_100deg']
    closed_door=(v+door_reference_shift(c['bay'])-h)@Rotation.from_euler('y',-100,degrees=True).as_matrix().T+h
    parts['rear_door_100deg']=((v+door_reference_shift(c['bay'])-h)@Rotation.from_euler('y',40,degrees=True).as_matrix().T+h,f)
    fig=make_subplots(rows=1,cols=2,specs=[[{'type':'scene'},{'type':'scene'}]],subplot_titles=['기체 전체 · 문 닫힘 · 실제 크기 비율','회수부 내부 · 문 140° 개방'])
    for name,(v,f) in parts.items():
        if name in ('sensor_tow_point','aircraft_tow_point'):continue
        color='#eda924' if name.startswith('sensor') else '#247cbb' if name.startswith('rear_door') else '#b98ed1' if name.startswith('capture_') else '#3f899b' if name.startswith('fuselage') else '#283c49' if name.startswith(('landing_','propulsion_')) else '#dae3e0'
        mesh=dict(x=v[:,0],y=v[:,1],z=-v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],color=color,name=name,showlegend=False,
                  opacity=.25 if name.startswith('guide') else 1)
        exterior={**mesh}
        if name=='rear_door_100deg':
            exterior.update(x=closed_door[:,0],y=closed_door[:,1],z=-closed_door[:,2])
        fig.add_trace(go.Mesh3d(**exterior),row=1,col=1)
        if not any(k in name for k in ('wing','aileron','tail','elevator','fin_CAD','rudder','fuselage','guide_side')):
            if name not in ('capture_ramp_side_-1','capture_ramp_top'):
                fig.add_trace(go.Mesh3d(**mesh),row=1,col=2)
    def scene(ranges,eye):
        spans=np.ptp(ranges,axis=1);ratio=2*spans/max(spans)
        return dict(xaxis=dict(range=ranges[0],autorange=False,title='전방 / m'),yaxis=dict(range=ranges[1],autorange=False,title='우측 / m'),
                    zaxis=dict(range=ranges[2],autorange=False,title='상방 / m'),aspectmode='manual',aspectratio=dict(zip('xyz',ratio)),camera=dict(eye=eye,projection=dict(type='orthographic')))
    fig.update_layout(height=650,margin=dict(l=0,r=0,t=55,b=0),paper_bgcolor='#f8fafb',
        scene=scene([[-1.15,.65],[-.95,.95],[-.29,.40]],dict(x=1.3,y=-1.8,z=.9)),
        scene2=scene([[-.76,-.2],[-.09,.09],[-.16,.10]],dict(x=.25,y=-1.8,z=.75)))
    mass=json.loads((out/'mass_properties.json').read_text(encoding='utf8'))
    door=json.loads((out/'door_swing.json').read_text(encoding='utf8'))
    pre=json.loads((out/'preflight.json').read_text(encoding='utf8'))
    intro=f'''<section style="max-width:1200px;margin:24px auto;font:16px/1.7 system-ui,sans-serif;color:#20333d">
    <h1>고익기 시험 모델 · 기체와 회수장치</h1><p>날개폭 1.8m · 매끈한 동체와 테이퍼 주익 · 전방 프로펠러 · 삼륜 착륙장치 · {len(geometry['parts'])}개 CAD 부품</p>
    <p>센서 수납부를 동체 안에 넣고 후방 출구를 배치했습니다. 출구 56 × 70mm · 문 최대 개방각 140° · 회수 가이드 50 × 50mm.</p>
    <p>도어 0–180°의 {door['sampled_angles']}개 위치에서 간섭 {door['intersecting_angles']}건. 정렬된 센서의 41개 검사 위치에서 최소 여유 {pre['minimum_gap_m']*1000:.2f}mm.</p>
    <p>기체 질량 {mass['aircraft_mass_kg']:.3f}kg, 센서 0.040kg, 줄 0.00225kg. 질량·재료 물성은 가정값이며 실제 부품 측정값으로 바꿔야 합니다.</p>
    <p>이 화면은 형상 검토용입니다. 전개·회수 성패와 하중은 별도의 운동 해석 결과에서 확인합니다.</p></section>'''
    if (out/'operating_cad/H1_door_closed_assembly.step').exists():
        intro=intro.replace('</section>','<p><a href="operating_cad/H1_door_closed_assembly.step">문 닫힌 조립 STEP</a> · <a href="operating_cad/H1_door_140_assembly.step">문 열린 조립 STEP</a></p></section>')
    page=fig.to_html(include_plotlyjs=True,full_html=True).replace('<body>','<body>'+intro)
    page=page.replace('<head>','<head><title>고익기 R3 · 기체와 회수장치</title>',1)
    (out/'model_overview.html').write_text(page,encoding='utf8')


if __name__=='__main__':main()
