"""Branch the actual deployed state into an explicitly controlled experiment.

Reuses the exact real flow5 database and physical inputs of the uncontrolled run.
This changes the controls, so it is an initial-state experiment, not a strict resume.
"""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
from pathlib import Path
import argparse
import copy
import hashlib
import html
import json
import subprocess
import sys
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dbf_stability import load_case, AeroDatabase, simulate
from dbf_stability.analysis import load_result
from dbf_stability.plots import history_figure

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE/'runs/span_hold_100m_25ms_01'


def record(out, stage, **details):
    value = dict(stage=stage, **details)
    temp = out/'pipeline.tmp'
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf8')
    temp.replace(out/'pipeline.json')
    print(json.dumps(value, ensure_ascii=False), flush=True)


def metrics(result, lower, upper):
    t = result.table
    mask = (t.time_s >= lower) & (t.time_s <= upper)
    d = t.loc[mask]
    return dict(start_time_s=float(d.time_s.iloc[0]), end_time_s=float(d.time_s.iloc[-1]),
        final_altitude_m=float(d.altitude_m.iloc[-1]), min_altitude_m=float(d.altitude_m.min()),
        max_altitude_m=float(d.altitude_m.max()), final_speed_m_s=float(d.airspeed_m_s.iloc[-1]),
        final_sink_rate_m_s=float(result.states[np.flatnonzero(mask)[-1], 5]),
        max_tension_N=float(d.tension_N.max()), max_contact_N=float(d.contact_N.max()),
        elevator_range_deg=[float(d.elevator_deg.min()), float(d.elevator_deg.max())],
        thrust_range_N=[float(d.thrust_N.min()), float(d.thrust_N.max())])


def report(out, baseline, result):
    c = result.config['flight']['controller']
    common_end = min(baseline.time[-1], result.time[-1])
    start = result.time[0]
    comparison = dict(common_window_s=[float(start), float(common_end)],
        baseline=metrics(baseline,start,common_end), controlled=metrics(result,start,common_end),
        controlled_whole=metrics(result,start,result.time[-1]),
        status=result.summary['status'], captured=result.summary['captured'],
        geometry_validity=result.summary.get('geometry_validity','not_audited'),
        numerically_converged=False,
        elevator_saturated=bool(result.table.elevator_saturated.any()),
        thrust_saturated=bool(result.table.thrust_saturated.any()))
    (out/'comparison.json').write_text(json.dumps(comparison,ensure_ascii=False,indent=2),encoding='utf8')
    fig = make_subplots(rows=4,cols=1,shared_xaxes=True,
        subplot_titles=('기체 고도','대기 속도','승강타','추력'))
    for label,r,color in [('제어 없음',baseline,'#9b5a48'),('고도·속도 제어',result,'#1769aa')]:
        t=r.table
        # Plot saved samples; retain command extrema in the full CSV.
        stride=max(1,len(t)//10000)
        t=t.iloc[::stride]
        for row,key in enumerate(('altitude_m','airspeed_m_s','elevator_deg','thrust_N'),1):
            fig.add_trace(go.Scatter(x=t.time_s,y=t[key],name=label,legendgroup=label,
                showlegend=row==1,line=dict(color=color)),row=row,col=1)
    for row,target in ((1,c['altitude_m']),(2,c['airspeed_m_s'])):
        fig.add_hline(y=target,line_dash='dot',line_color='#536159',row=row,col=1)
    for row,unit in enumerate(('m','m/s','deg','N'),1):
        fig.update_yaxes(title_text=unit,row=row,col=1)
    fig.update_xaxes(title_text='미션 경과 시간 (s)',row=4,col=1)
    fig.update_layout(template='plotly_white',height=1000,margin=dict(t=50,b=50),
        legend=dict(orientation='h'),title='같은 flow5 공력·같은 전개 상태에서 비교')
    rows=''.join(f'<tr><td>{label}</td><td>{comparison[key]["final_altitude_m"]:.2f} m</td>'
        f'<td>{comparison[key]["final_speed_m_s"]:.2f} m/s</td></tr>'
        for label,key in [('제어 없음','baseline'),('제어 적용','controlled')])
    s=result.summary
    document=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>고도·속도 제어 시험</title>
<style>body{{font-family:system-ui,sans-serif;max-width:1100px;margin:40px auto;padding:0 20px;color:#24323b;line-height:1.7}}table{{border-collapse:collapse}}td,th{{padding:8px 22px;border-bottom:1px solid #ccc}}a{{color:#1769aa}}.note{{background:#f1f4f5;padding:16px}}</style>
<h1>고도·속도 제어 시험</h1><p>목표: {c['altitude_m']:g} m · {c['airspeed_m_s']:g} m/s. 전개 완료 {start:.3f}초의 기존 기체·센서·와이어 상태에서 제어를 켰습니다.</p>
<p>계산 구간: {start:.3f}–{result.time[-1]:.3f}초 · 종료 상태: {html.escape(s['status'])} · 센서 포획: {'완료' if s['captured'] else '미완료'}.</p>
<p>{html.escape(s.get('message',''))}</p><p><a href="mission/H1_replay.html">실제 계산 3차원 재생</a> · <a href="mission/timeseries.csv">전체 시계열</a> · <a href="comparison.json">비교 수치</a> · <a href="branch_source.json">초기 상태·공력 출처</a></p>
<p>두 계산이 모두 존재하는 {common_end:.3f}초 시점 비교입니다. 그 이후의 무제어 궤적은 추정해서 그리지 않았습니다.</p>
<table><tr><th>조건</th><th>고도</th><th>속도</th></tr>{rows}</table>
<div class="note">승강타 한계 ±7.5° · 추력 0–15 N. 고도·하강률→피치, 피치·피치율→승강타, 속도→추력으로 연결한 PD/P 시험입니다. 센서 오차·서보 지연·모터 응답은 포함하지 않았습니다. 추력은 배터리 전력값이 아닙니다. 횡방향 제어는 없습니다. CAD 검사: {html.escape(s.get('geometry_validity','not_audited'))}. 분할·시간 간격 수렴 및 실기 시험으로 검증하지 않은 가정 사례입니다.</div>
{fig.to_html(full_html=False,include_plotlyjs=True)}<p>구조 참고: <a href="https://docs.px4.io/main/en/flight_stack/controller_diagrams#fixed-wing-position-controller">PX4 고정익 제어 설명</a>. 이 시험은 PX4 또는 TECS 구현이 아닙니다.</p></html>'''
    (out/'report.html').write_text(document,encoding='utf8')
    return comparison


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--case',type=Path,default=ROOT/'examples/h1_normal_r3_flow5_controlled.yaml')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--until',type=float)
    p.add_argument('--workers',type=int,default=2)
    args=p.parse_args()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    try:
        c=load_case(args.case)
        baseline=load_result(BASE/'mission')
        # No silent changes to the plant, wind, geometry, or cable schedule.
        for key in ('aircraft','sensor','cable','bay','winch','collision','aero','mission_profile'):
            if c[key]!=baseline.config[key]:
                raise ValueError(f'Controlled experiment changes baseline {key}')
        oldflight=copy.deepcopy(baseline.config['flight']);newflight=copy.deepcopy(c['flight'])
        newflight.pop('controller')
        if oldflight!=newflight:
            raise ValueError('Controlled experiment changes baseline flight conditions')
        start=c['flight']['controller']['enable_from_s']
        index=int(np.argmin(abs(baseline.time-start)))
        if abs(baseline.time[index]-start)>1e-10:
            raise ValueError('Control activation must match an exactly saved baseline state')
        if 'captured' in baseline.table and baseline.table.captured.iloc[index]:
            raise ValueError('Expected uncaptured deployed initial state')
        end=args.until if args.until is not None else c['mission_profile']['end_s']
        if end<=start:raise ValueError('End time must follow activation')
        state=baseline.states[index].copy()
        a=AeroDatabase(BASE/'aerodynamics/aero_database.npz')
        trim=json.loads((BASE/'trim.json').read_text(encoding='utf8'))
        source=dict(experiment='new controlled branch, not strict checkpoint resume',
            baseline=str(BASE/'mission'),time_s=float(start),row=index,
            states_sha256=hashlib.sha256((BASE/'mission/states.npz').read_bytes()).hexdigest(),
            initial_state_sha256=hashlib.sha256(state.tobytes()).hexdigest(),
            aero_database=str(a.path),aero_sha256=hashlib.sha256(Path(a.path).read_bytes()).hexdigest(),
            solver=a.metadata['solver'],controller=c['flight']['controller'])
        (out/'branch_source.json').write_text(json.dumps(source,ensure_ascii=False,indent=2),encoding='utf8')
        (out/'source_versions.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest()
            for f in (ROOT/'src/dbf_stability').glob('*.py')},indent=2),encoding='utf8')
        (out/'trim.json').write_text(json.dumps(trim,indent=2),encoding='utf8')
        record(out,'simulation',start_time_s=float(start),end_time_s=end,solver=a.metadata['solver'])
        r=simulate(c,a,trim,phase='mission',initial_state=state,start_time=start,duration=end-start,output=out/'mission')
        np.testing.assert_array_equal(r.states[0],state)
        r.summary.update(controller=c['flight']['controller'],branch_source=source,
            scope=f'고도 100m·속도 25m/s 제어 시험. 전개 완료 {start:.3f}초의 기존 상태에서 이어 계산했습니다. 표시되는 하중·충격량은 이 구간의 값입니다. 완전한 상태 측정과 즉각적인 조종면·추력 응답을 가정했습니다.')
        r.summary['limitations'].append('Explicit longitudinal PD/P; ideal measurements and instantaneous bounded actuators; no integral or lateral feedback')
        r.save(out/'mission')
        history_figure(r).write_html(out/'mission/history.html',include_plotlyjs=True)
        record(out,'cad_audit',calculation_status=r.summary['status'],end_time_s=float(r.time[-1]))
        subprocess.run([sys.executable,str(HERE/'audit_trajectory_collisions.py'),str(out/'mission'),
            '--workers',str(args.workers),'--time-step','.1'],cwd=ROOT,check=True)
        subprocess.run([sys.executable,str(HERE/'visualize_run.py'),'--run',str(out/'mission')],cwd=ROOT,check=True)
        checked=load_result(out/'mission')
        comparison=report(out,baseline,checked)
        record(out,'completed',calculation_status=checked.summary['status'],end_time_s=float(checked.time[-1]),
            captured=checked.summary['captured'],metrics=comparison['controlled_whole'])
    except Exception as exc:
        record(out,'failed',message=str(exc));raise


if __name__=='__main__':main()
