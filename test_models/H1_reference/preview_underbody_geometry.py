"""Fixed-scale kinematic CAD preview; explicitly separate from flight results."""
from pathlib import Path
import argparse, json
import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation
from dbf_stability import load_case
from dbf_stability.door import door_hinge, door_reference_shift
from dbf_stability.collision import read_binary_stl


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();c=load_case(args.case);b=c['bay'];h=door_hinge(b)
    directory=Path(c['_root'])/c['collision']['mesh_directory']
    audit=json.loads(args.audit.read_text(encoding='utf8'))
    fig=go.Figure();door=None
    for file in directory.glob('*_FRD_m.stl'):
        name=file.name.removesuffix('_FRD_m.stl')
        if name in ('sensor_tow_point','aircraft_tow_point'):continue
        v,f=read_binary_stl(file)
        if name=='rear_door_100deg':door=(v,f);continue
        if v[:,0].max()<h[0]-.13 or v[:,0].min()>h[0]+.26:continue
        if any(s in name for s in ('wing','tail','rudder','elevator','aileron','vertical_fin','landing','propulsion')):continue
        color='#deab38' if name.startswith('sensor') else '#006c9b' if name.startswith('door_') else '#b95564' if name=='capture_front_stop' else '#778b94'
        opacity=.14 if name.startswith('fuselage') else .25 if name.startswith('guide') else 1
        fig.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=-v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],
            color=color,opacity=opacity,name=name,hoverinfo='name',showlegend=False))
    v,f=door;index=len(fig.data)
    def posed(angle):return Rotation.from_euler('y',angle-100,degrees=True).apply(v+door_reference_shift(b)-h)+h
    q=posed(270)
    fig.add_trace(go.Mesh3d(x=q[:,0],y=q[:,1],z=-q[:,2],i=f[:,0],j=f[:,1],k=f[:,2],color='#007da8',opacity=1,name='270° 회전 문',hoverinfo='name'))
    angles=list(range(0,271,3));frames=[]
    for angle in angles:
        q=posed(angle)
        frames.append(go.Frame(name=str(angle),traces=[index],data=[go.Mesh3d(x=q[:,0],y=q[:,1],z=-q[:,2])]))
    fig.frames=frames
    fig.update_layout(height=650,margin=dict(l=0,r=0,t=20,b=100),template='plotly_white',uirevision='fixed-camera',
        scene=dict(xaxis=dict(title='전방 x (m)',range=[h[0]-.13,h[0]+.26],autorange=False),
                   yaxis=dict(title='우측 y (m)',range=[-.09,.09],autorange=False),
                   zaxis=dict(title='위쪽 -z (m)',range=[-h[2]-.11,-b['ceiling_z_m']+.035],autorange=False),
                   aspectmode='manual',aspectratio=dict(x=1.8,y=.83,z=1.1),
                   camera=dict(eye=dict(x=-1.5,y=-2.3,z=.85),projection=dict(type='orthographic'))),
        updatemenus=[dict(type='buttons',direction='left',x=0,y=-.13,buttons=[
            dict(label='열기 0 → 270°',method='animate',args=[[str(v) for v in angles],dict(mode='immediate',frame=dict(duration=30,redraw=True),transition=dict(duration=0))]),
            dict(label='닫기 270 → 0°',method='animate',args=[[str(v) for v in reversed(angles)],dict(mode='immediate',frame=dict(duration=30,redraw=True),transition=dict(duration=0))]),
            dict(label='정지',method='animate',args=[[None],dict(mode='immediate',frame=dict(duration=0,redraw=False))])])],
        sliders=[dict(active=len(angles)-1,x=0,len=1,y=-.02,currentvalue=dict(prefix='개방각 '),steps=[
            dict(label=str(a),method='animate',args=[[str(a)],dict(mode='immediate',frame=dict(duration=0,redraw=True),transition=dict(duration=0))]) for a in angles])])
    doc=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>동체 아래로 접히는 270° 문</title>
<style>body{{font-family:system-ui,sans-serif;margin:26px auto;max-width:1150px;padding:0 20px;color:#20343f;line-height:1.7}}a{{color:#066e9b}}.note{{padding:14px 18px;background:#eef4f6}}</style>
<h1>동체 아래로 접히는 270° 문</h1><p>문판을 70mm에서 102mm로 연장하고, 힌지를 출구 바닥 아래 30mm에 배치했습니다. 파란색은 문과 힌지 부품, 노란색은 수납된 센서입니다.</p>
<div class="note">기구 형상 확인용 재생입니다. 센서와 기체는 고정해 두었으며, 비행·와이어 운동 계산은 별도입니다. 열고 닫아도 축 범위와 화면 비율은 유지됩니다.</div>
<p>1° 간격 {audit['samples']}개 자세 검사: 겹침 {audit['intersecting_samples']}개 · 계산 오류 {audit['query_error_samples']}개. 270°에서 동체와 최소 11.19mm, 힌지 핀·받침과 0.20mm 여유. 연속 충돌 증명이나 제작 공차 검증은 아닙니다.</p>
{fig.to_html(full_html=False,include_plotlyjs=True,auto_play=False)}
<p><a href="../underbody_recovery_04/progress.html">포획·문 닫힘 해석</a> · <a href="../../geometry_variants/normal_r3_underbody_270_v2/H1_underbody_270_assembly.step">270° 조립 STEP</a> · <a href="audit.json">각도별 검사 기록</a></p></html>'''
    args.out.write_text(doc,encoding='utf8');print(args.out)


if __name__=='__main__':main()
