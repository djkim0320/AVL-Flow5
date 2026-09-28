"""Compare actual-AVL airframe responses with a one-degree pitch disturbance."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dbf_stability import load_case,AeroDatabase,solve_trim,simulate
from dbf_stability.math3d import quaternion

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'runs/normal_r3_01/airframe_response'


def calculate(name):
    new=name=='normal_r3'
    c=load_case(ROOT/('examples/h1_normal_r3.yaml' if new else 'examples/h1_opening_70.yaml'))
    a=AeroDatabase(HERE/('runs/normal_r3_01/aerodynamics/aero_database.npz' if new else 'aerodynamics/aero_database.npz'))
    trim=solve_trim(c,a,mode='aircraft_only')
    state=trim['state'].copy()
    state[6:10]=quaternion(pitch=trim['alpha_rad']+np.deg2rad(1))
    c['simulation']['sample_dt_s']=.02
    target=OUT/name
    target.mkdir(parents=True,exist_ok=False)
    r=simulate(c,a,trim,phase='aircraft_only',initial_state=state,duration=20,output=target)
    data=dict(name=name,status=r.summary['status'],duration_s=r.summary['duration_s'],
        trim_pitch_deg=trim['alpha_deg'],initial_pitch_offset_deg=1.,autopilot=False,
        final_pitch_offset_deg=float(r.table.pitch_deg.iloc[-1]-trim['alpha_deg']),
        final_pitch_rate_deg_s=float(np.rad2deg(r.states[-1,11])),
        aircraft_only=True,sensor_forces_included=False)
    (target/'response.json').write_text(json.dumps(data,indent=2),encoding='utf8')
    return data


def main():
    from dbf_stability.analysis import load_result
    with ProcessPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(calculate,['previous','normal_r3']))
    fig=make_subplots(rows=2,cols=1,shared_xaxes=True,subplot_titles=['트림에서 벗어난 피치 각도','피치 각속도'])
    for row,color,label in zip(results,['#a8a6a0','#237e8d'],['이전 H1','새 고익기 R3']):
        r=load_result(OUT/row['name'])
        fig.add_trace(go.Scatter(x=r.time,y=r.table.pitch_deg-row['trim_pitch_deg'],name=label,legendgroup=label,line=dict(color=color)),row=1,col=1)
        fig.add_trace(go.Scatter(x=r.time,y=np.rad2deg(r.states[:,11]),name=label,legendgroup=label,showlegend=False,line=dict(color=color)),row=2,col=1)
    fig.update_yaxes(title='피치 변화 / °',row=1,col=1)
    fig.update_yaxes(title='각속도 / °/s',row=2,col=1)
    fig.update_xaxes(title='시간 / s',row=2,col=1)
    fig.update_layout(template='plotly_white',height=800)
    heading='<section style="font:16px/1.7 system-ui;margin:24px 5%"><h1>기체 단독 · 1° 피치 교란 응답</h1><p>각 기체의 실제 AVL 공력표로 20초를 계산했습니다. 초기 자세만 트림보다 1° 올리고 엘리베이터·추력은 각 기체의 트림값으로 유지했습니다. 자동조종기는 없습니다.</p><p>센서와 줄의 힘을 제외한 기체 단독 비교입니다. 센서 전개·회수 안정성이나 비행 안전을 판정하는 그래프가 아닙니다.</p></section>'
    (OUT/'response.html').write_text(fig.to_html(include_plotlyjs=True).replace('<body>','<body>'+heading),encoding='utf8')
    (OUT/'comparison.json').write_text(json.dumps(results,indent=2),encoding='utf8')
    print(json.dumps(results,indent=2),flush=True)


if __name__=='__main__':main()
