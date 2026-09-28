"""Plot actual AVL strip forces and table coefficients for the R3 airframe."""
from pathlib import Path
import json,re
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

HERE=Path(__file__).resolve().parent
OUT=HERE/'runs/normal_r3_01/aerodynamics'


def main():
    cols=['strip','x_le_m','y_le_m','z_le_m','chord_m','area_m2','c_cl_m','induced_angle','cl_normal','cl','cd','cd_viscous','cm_quarter','cm_le','cp_x_c']
    surface=None;rows=[]
    for line in (OUT/'spanwise/strip_forces.txt').read_text().splitlines():
        match=re.match(r'\s*Surface #\s+\d+\s+(.*)',line)
        if match:surface=match.group(1).strip()
        fields=line.split()
        if surface and len(fields)==len(cols) and fields[0].isdigit():
            values=list(map(float,fields));rows.append(dict(surface=surface,**dict(zip(cols,values))))
    data=pd.DataFrame(rows)
    if len(data)!=132:raise ValueError(f'Unexpected AVL strip count {len(data)}')
    data['lift_per_span_N_m']=.5*1.225*20**2*data.c_cl_m
    data.to_csv(OUT/'spanwise/strip_forces.csv',index=False)
    fig=make_subplots(rows=2,cols=2,subplot_titles=['수납 트림 · 주익 분포 양력','수납 트림 · 수평꼬리 분포 양력','새 기체 양력 계수','새 기체 피치 모멘트 계수'])
    for col,part in [(1,'Main_wing'),(2,'Horizontal_tail')]:
        table=data[data.surface.str.contains(part)].sort_values('y_le_m')
        fig.add_trace(go.Scatter(x=table.y_le_m,y=table.lift_per_span_N_m,name=part,mode='lines+markers',line=dict(color='#217b8b'),showlegend=False),row=1,col=col)
        fig.update_xaxes(title='우측 방향 위치 / m',row=1,col=col)
        fig.update_yaxes(title='분포 양력 / N/m',row=1,col=col)
    with np.load(OUT/'aero_database.npz',allow_pickle=False) as archive:
        alpha=archive['alpha'];ib=int(np.flatnonzero(archive['beta']==0)[0]);coeff=archive['coeff']
        for elevator,color in [(0,'#758793'),(5,'#1e778a')]:
            ie=int(np.flatnonzero(archive['elevator']==elevator)[0]);c=coeff[:,ib,ie]
            lift=np.sin(np.deg2rad(alpha))*c[:,0]-np.cos(np.deg2rad(alpha))*c[:,2]
            for col,value in [(1,lift),(2,c[:,4])]:
                fig.add_trace(go.Scatter(x=alpha,y=value,name=f'엘리베이터 {elevator}°',legendgroup=str(elevator),showlegend=col==1,mode='lines+markers',line=dict(color=color)),row=2,col=col)
                fig.update_xaxes(title='기체 받음각 / °',row=2,col=col)
    fig.update_yaxes(title='CL',row=2,col=1);fig.update_yaxes(title='Cm',row=2,col=2)
    fig.update_layout(height=840,template='plotly_white',margin=dict(l=75,r=35,t=75,b=60),legend=dict(orientation='h',y=-.12))
    trim=json.loads((OUT/'trim_comparison.json').read_text(encoding='utf8'))['normal_r3']['trim']
    intro=f'''<section style="margin:24px 5%;font:16px/1.7 system-ui;color:#233d48"><h1>새 고익기의 실제 AVL 공력</h1>
    <p>AVL 3.52 · 315개 공력 조건 · 주익·꼬리·등가 단면적 동체. 아래 분포 양력은 AVL의 FS 출력 132개 구간을 읽어 그렸습니다.</p>
    <p>분포 양력 계산점: 20m/s, 받음각 {trim['alpha_deg']:.3f}°, 엘리베이터 {trim['elevator_deg']:.3f}°. 수평꼬리의 음의 값은 아래쪽 힘입니다.</p>
    <p><a href="spanwise/strip_forces.txt">AVL 분포 원본</a> · <a href="spanwise/surface_forces.txt">양력면별 원본</a> · <a href="spanwise/strip_forces.csv">분포 CSV</a></p>
    <p>이 결과는 준정상 공력 계산이며, 실속·도어 주변 박리·프로펠러 후류를 계산한 결과가 아닙니다. 기체 추가 항력과 센서·줄 공력은 별도 가정값입니다.</p></section>'''
    (OUT/'aero_report.html').write_text(fig.to_html(include_plotlyjs=True).replace('<body>','<body>'+intro),encoding='utf8')
    print(f'Parsed {len(data)} actual AVL strips',flush=True)


if __name__=='__main__':main()
