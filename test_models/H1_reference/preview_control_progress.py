"""Read only immutable accepted chunks; never open the live checkpoint writer."""
from pathlib import Path
import argparse
import json
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def main():
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    args=p.parse_args()
    out=args.run
    c=json.loads((out/'mission/inputs.json').read_text(encoding='utf8'))
    source=json.loads((out/'branch_source.json').read_text(encoding='utf8'))
    chunks=sorted((out/'mission/accepted_history').glob('part_*.npz'))
    times=[];states=[]
    for path in chunks:
        with np.load(path,allow_pickle=False) as data:
            times.append(data['time'].copy());states.append(data['states'].copy())
    t=np.concatenate(times);y=np.concatenate(states)
    if c['flight'].get('gust'):
        raise ValueError('Preview speed calculation requires still air or constant wind')
    speed=np.linalg.norm(y[:,3:6]-np.array(c['flight']['wind_ned_m_s']),axis=1)
    baseline=pd.read_csv(Path(source['baseline'])/'timeseries.csv',usecols=['time_s','altitude_m','airspeed_m_s'])
    fig=make_subplots(rows=2,cols=1,shared_xaxes=True,subplot_titles=('기체 고도','대기 속도'))
    stride=max(1,len(t)//5000)
    indices=np.unique(np.r_[np.arange(0,len(t),stride),len(t)-1,np.argmax(y[:,2])])
    old=baseline.iloc[::max(1,len(baseline)//5000)]
    for row,key,values,target in ((1,'altitude_m',-y[:,2],100.),(2,'airspeed_m_s',speed,25.)):
        fig.add_trace(go.Scatter(x=old.time_s,y=old[key],name='제어 없음',legendgroup='off',showlegend=row==1,line=dict(color='#a65d47')),row=row,col=1)
        fig.add_trace(go.Scatter(x=t[indices],y=values[indices],name='제어 적용 · 계산된 구간',legendgroup='on',showlegend=row==1,line=dict(color='#1769aa')),row=row,col=1)
        fig.add_hline(y=target,line_dash='dot',line_color='#60756e',row=row,col=1)
    fig.update_yaxes(title_text='고도 (m)',row=1,col=1)
    fig.update_yaxes(title_text='속도 (m/s)',row=2,col=1)
    fig.update_xaxes(title_text='미션 경과 시간 (s)',row=2,col=1)
    fig.update_layout(template='plotly_white',height=780,margin=dict(t=50,b=50),legend=dict(orientation='h'))
    info=dict(time_s=float(t[-1]),altitude_m=float(-y[-1,2]),airspeed_m_s=float(speed[-1]),
        sink_rate_m_s=float(y[-1,5]),minimum_altitude_m=float(-y[:,2].max()),
        chunks=len(chunks),last_immutable_chunk=chunks[-1].name,status='accepted_partial_snapshot')
    (out/'progress_snapshot.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf8')
    page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>고도·속도 제어 · 계산된 구간</title>
<style>body{{font-family:system-ui,sans-serif;max-width:1100px;padding:28px;margin:auto;color:#24343c;line-height:1.7}}.note{{padding:16px;background:#eef3f6}}a{{color:#1769aa}}</style>
<h1>고도·속도 제어 · 중간 결과</h1><p><strong>{t[-1]:.2f}초까지 저장된 상태</strong> · 고도 {-y[-1,2]:.2f}m · 속도 {speed[-1]:.2f}m/s · 하강률 {y[-1,5]:.5f}m/s</p>
<p>목표 100m·25m/s. 전개 완료 {t[0]:.3f}초의 같은 상태에서 제어를 켰습니다. 실제 flow5 공력과 센서·와이어·접촉 운동을 사용했습니다.</p>
<div class="note">그래프는 저장된 구간의 스냅샷입니다. 파란 선이 끝난 뒤의 운동은 아직 표시하지 않습니다. 최종 회수·포획 및 별도 CAD 검사는 아직 판정하지 않았습니다. 센서 오차와 구동기 지연은 없는 가정 사례입니다.</div>
<p id="pipeline-status" role="status">계산·검사가 끝나면 이곳에 최종 결과 링크가 표시됩니다. 그래프 자체는 자동 갱신되지 않습니다.</p>
{fig.to_html(full_html=False,include_plotlyjs=True)}<p><a href="progress_snapshot.json">이 화면의 수치·기록 범위</a></p>
<script>
let timer;
async function checkCompletion() {{
  try {{
    const response=await fetch('pipeline.json',{{cache:'no-store'}});
    if(!response.ok)return;
    const status=await response.json();
    const element=document.getElementById('pipeline-status');
    if(status.stage==='completed') {{
      clearInterval(timer);
      element.replaceChildren(document.createTextNode('계산과 결과 정리가 끝났습니다. '));
      const link=document.createElement('a');link.href='report.html';link.textContent='최종 고도·회수·포획 결과 보기';element.append(link);
    }} else if(status.stage==='failed') {{
      clearInterval(timer);element.textContent='처리가 중단됐습니다: '+status.message;
    }}
  }} catch(error) {{ /* Keep the verified snapshot if the local server is unavailable. */ }}
}}
timer=setInterval(checkCompletion,15000);checkCompletion();
</script></html>'''
    (out/'progress.html').write_text(page,encoding='utf8')
    print(json.dumps(info,ensure_ascii=False))


if __name__=='__main__':main()
