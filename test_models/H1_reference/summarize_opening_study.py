"""Make the opening comparison only from completed, independently audited runs."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse
import hashlib
import json
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from dbf_stability.analysis import load_result
from dbf_stability.collision import MeshContacts
from dbf_stability.math3d import rotation
from summarize_door_study import inspect_run

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def upper_clearance(path):
    path=Path(path);r=load_result(path);world=MeshContacts(r.config)
    world.obstacles=[o for o in world.obstacles if o['name'] in ('guide_ceiling','rear_exit_frame_ceiling')]
    rows=[]
    for i,y in enumerate(r.states):
        if r.time[i]<r.config['winch']['recovery_start_s']:continue
        ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3])
        hits=world.contacts(center,ra.T@rs,np.empty((0,3)),float(r.table.door_deg.iloc[i]),distance=.20)
        h=min(hits,key=lambda h:(h.get('geometry_gap',h['gap']),h['gap'])) if hits else None
        rows.append({'time_s':float(r.time[i]),'gap_m':float(h.get('geometry_gap',h['gap'])) if h else None,
                     'sensor_part':h['moving'] if h else None,'fixed_part':h['fixed'] if h else None})
    world.close()
    near=[row for row in rows if row['gap_m'] is not None]
    return {'rows':rows,'minimum_gap_m':min((row['gap_m'] for row in near),default=None),
        'contact_skin_samples':sum(row['gap_m']<r.config['collision']['skin_m'] for row in near)}


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);args=p.parse_args();args.root=args.root.resolve()
    baseline=HERE/'runs/door_angle_01/angle_140/mission';new=args.root/'opening_70/mission'
    runs=[load_result(path) for path in (baseline,new)]
    diag=[json.loads((baseline/'door_diagnosis.json').read_text(encoding='utf8')),inspect_run(new)]
    with ProcessPoolExecutor(max_workers=2) as pool:upper=list(pool.map(upper_clearance,(baseline,new)))
    pd.DataFrame(upper[1]['rows']).to_csv(new/'sensor_upper_clearance.csv',index=False)
    versions=json.loads((args.root/'source_versions.json').read_text(encoding='utf8'))
    version_matches={key:hashlib.sha256((ROOT/key).read_bytes()).hexdigest()==digest for key,digest in versions.items()}
    if not all(version_matches.values()):raise ValueError('Sources changed during the run')
    old_versions=json.loads((HERE/'runs/door_angle_01/source_versions.json').read_text(encoding='utf8'))
    same_physics=all(old_versions[key]==value for key,value in versions.items() if key.startswith('src'))
    if not same_physics:raise ValueError('Baseline physics differs from revised run')
    rejected=json.loads((HERE/'geometry_variants/opening_80/door_swing.json').read_text(encoding='utf8'))
    report={'same_core_physics_as_56mm':same_physics,'source_hash_checks':version_matches,
        'new_run_from_s':float(runs[1].time[0]),'requested_end_s':7.,'geometry_height_mm':[56,70],
        'opening_80_rejected_door_intersecting_angles':rejected['intersecting_angles'],
        'numerically_converged':False,'physical_validation':'unvalidated_assumption_case',
        'runs':[dict(d,upper_sensor_clearance={k:v for k,v in u.items() if k!='rows'}) for d,u in zip(diag,upper)]}
    r=runs[1];y=r.states[-1];ra=rotation(y[6:10]);rs=rotation(y[19:23]);world=MeshContacts(r.config)
    sensor=np.vstack([part['vertices'] for part in world.sensor])@(ra.T@rs).T+ra.T@(y[13:16]-y[:3]);world.close()
    report['final_sensor_bounds_frd_m']=[sensor.min(0).tolist(),sensor.max(0).tolist()]
    report['sensor_completely_forward_of_exit_at_end']=bool(sensor[:,0].min()>r.config['bay']['exit_x_m'])
    (args.root/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    def status(d):
        reason={'contact_clearance_limit':'최소 간격 한계','numerical_failure':'수치 오류'}.get(d['status'],d['status'])
        return '7초 계산 완료' if d['status']=='completed' and d['duration_s']>=7-1e-8 else f"{d['duration_s']:.3f}초 중단 · {reason}"
    fig=go.Figure();table=[]
    for h,r,d,u,url in zip((56,70),runs,diag,upper,('../door_angle_01/angle_140/mission/H1_replay.html','opening_70/mission/H1_replay.html')):
        fig.add_trace(go.Scatter(x=r.time,y=r.table.tension_N,name=f'높이 {h}mm'))
        gap=u['minimum_gap_m'];gap_text='검사 범위 밖' if gap is None else f'{gap*1000:.3f}mm'
        table.append(f"<tr><td><a href='{url}'>{h}mm 재생</a></td><td>{status(d)}</td><td>{gap_text}</td><td>{d['max_tension_N']:.2f}N</td><td>{'포획' if d['captured'] else '미포획'}</td><td>{d['cad_samples']} / {d['cad_overlap_samples']} / {d['cad_error_samples']}</td></tr>")
    fig.add_hline(y=8,line_dash='dash',line_color='#ad4c22',annotation_text='입력 장력 한계 8N')
    fig.update_layout(template='plotly_white',height=370,xaxis_title='시간 / 초',yaxis_title='줄 장력 / N',margin=dict(t=35,b=45,l=60,r=30))
    latest=diag[1];nearest=latest['nearest_end_pair']
    names={'sensor_body':'센서 몸체','winch_drum':'윈치','sensor_fin_3':'센서 상부 핀','rear_exit_frame_ceiling':'출구 상단'}
    end_note=(f"마지막 근접 부품: {names.get(nearest['moving'],nearest['moving'])} / {names.get(nearest['fixed'],nearest['fixed'])}, 간격 {nearest['gap_m']*1000:.3f}mm." if nearest else '마지막 상태에서 20mm 이내의 근접 부품 없음.')
    passage_note=('센서 전체가 출구를 지나 내부로 들어왔다. ' if report['sensor_completely_forward_of_exit_at_end'] else '센서 전체의 출구 통과는 확인되지 않았다. ')
    if nearest and nearest['fixed']=='winch_drum':passage_note+='이번 계산은 내부 윈치에 접근하면서 중단됐으며, 포획하지 못했다. '
    if upper[1]['contact_skin_samples']:
        passage_note+=f"회수 중 상단 최소 간격은 {upper[1]['minimum_gap_m']*1000:.3f}mm이고, 저장된 {upper[1]['contact_skin_samples']}개 시각이 수치 접촉 범위 0.8mm 안에 들었다. 상부가 완전히 무접촉인 결과는 아니다."
    html='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>H1 출구 높이 변경 결과</title><style>body{font:16px/1.65 system-ui,sans-serif;color:#23313b;max-width:1160px;margin:28px auto;padding:0 24px}h1{font-size:27px}table{width:100%;border-collapse:collapse}th,td{padding:12px;text-align:left;border-bottom:1px solid #d7dfe4}th{background:#f2f5f7}a{color:#086b85}aside{padding:16px;background:#fff4e5;border-radius:8px}img{width:100%;height:auto}small{color:#596772}</style>
<h1>출구 높이 56 → 70mm</h1><p>바닥을 유지하고 상단을 14mm 올렸습니다. 내부 천장·측벽·출구 프레임·동체 절개부와 문판을 함께 바꿨습니다. 출구 폭 56mm, 문 개방각 140°와 윈치 명령은 같습니다.</p>
<p>기존에 멈춘 센서 자세를 그대로 놓으면 상단 간격이 0.02mm에서 6.74mm로 늘어납니다. 아래 그림은 같은 자세의 형상 비교이며, 새 운동 계산은 별도 재생에서 확인합니다.</p>
<img src="../../diagnostics/opening/same_pose_comparison.png" alt="기존 센서 자세에서 높이 56, 70, 80mm 출구의 상단 여유 비교">
<aside>80mm 안은 늘어난 문판이 개방 중 꼬리날개와 겹쳐 제외했습니다. 70mm 문판은 0~180°의 181개 CAD 자세에서 고정 부품과의 겹침·검사 오류가 0건이었습니다.</aside>
<h2>처음부터 다시 계산한 운동</h2><table><thead><tr><th>출구 높이</th><th>계산 결과</th><th>회수 중 센서–상단 최소 간격</th><th>최대 장력</th><th>포획</th><th>CAD 검사 시각 / 겹침 / 오류</th></tr></thead><tbody>'''+''.join(table)+'''</tbody></table><p>'''+end_note+'''</p>
<p><small>하중은 각 계산의 마지막 저장 시각까지의 최대값입니다. 수치 오류·중단 사례를 짧게 계산됐다는 이유로 좋은 설계로 판정하지 않습니다. CAD 검사는 저장 자세의 검사이며 연속 시간 전체의 증명이 아닙니다.</small></p>'''+fig.to_html(full_html=False,include_plotlyjs=True)+'''
<p><a href="opening_70/mission/H1_replay.html">70mm 실제 계산 재생</a> · <a href="../../geometry_variants/opening_70/operating_cad/H1_door_140_assembly.step">70mm 수정 STEP 조립체</a> · <a href="../../aerodynamics/connection_check/report.html">실제 AVL 실행 근거</a></p>
<p>기체 양력면은 실제 AVL 3.52 공력표를 사용했습니다. 구멍·문의 유동 변화, 센서·줄 공력과 후류는 검증하지 않았으며 flow5는 연결하지 않았습니다. 형상 비교를 위해 가정한 기체 질량·CG·관성은 유지했습니다. 접촉 하중과 변경안의 수치 수렴성은 아직 검증하지 않았습니다.</p></html>'''
    html=html.replace('<h2>처음부터 다시 계산한 운동</h2>','<h2>처음부터 다시 계산한 운동</h2><p>'+passage_note+'</p>')
    (args.root/'comparison.html').write_text(html,encoding='utf8')
    md_rows=[]
    for h,d,u in zip((56,70),diag,upper):
        gap=u['minimum_gap_m'];gap_text='범위 밖' if gap is None else f'{gap*1000:.3f}mm'
        md_rows.append(f"| {h}mm | {status(d)} | {gap_text} | {d['max_tension_N']:.3f}N | {'포획' if d['captured'] else '미포획'} | {d['cad_samples']} / {d['cad_overlap_samples']} / {d['cad_error_samples']} |")
    report_md=f'''# H1 출구 높이 변경 결과

출구를 **폭 56mm × 높이 70mm**로 키웠다. 바닥·견인점은 유지하고 상단을 14mm 올렸으며 내부 천장·측벽·프레임·동체 절개부·문판을 함께 바꿨다. 문 개방각은 140°로 유지했다. 재생 화면, 접촉 계산, 실제 STEP 검사는 같은 형상 폴더를 사용한다.

기존 56mm 출구에서 멈춘 자세를 그대로 놓으면 센서–상단 간격은 **0.020 → 6.738mm**로 늘어난다. 이는 해당 자세의 간섭 제거를 확인한 결과이며 새 회수 궤적의 성공을 뜻하지 않는다. [같은 자세의 단면 비교](diagnostics/opening/same_pose_comparison.png)를 제공한다.

80mm 안도 만들었지만, 커진 문판이 개방 중 수직·수평 꼬리날개와 겹쳤다. 0~180°의 181개 자세 중 17개에서 교차가 발생해 운동 계산에서 제외했다. 70mm 안은 같은 181개 문 자세에서 겹침·CAD 오류가 0건이었다. 이는 표본 자세 검사이며 연속 회전 전체의 증명은 아니다.

## 실제 계산 결과

{passage_note}

센서·줄·기체·윈치·조종 입력과 핵심 운동 코드를 유지하고, 70mm 안을 0초부터 7초까지 요청해 계산했다. 기존에 멈춘 상태에서 이어 붙이거나 위치·속도를 보정하지 않았다.

| 출구 높이 | 저장된 계산 결과 | 회수 중 센서–상단 최소 간격 | 최대 장력 | 포획 | CAD 검사 시각 / 겹침 / 오류 |
|---|---|---:|---:|---|---|
{chr(10).join(md_rows)}

70mm 안의 {end_note}

최소 간격과 최대 하중은 저장 시각 기준이다. CAD 검사는 모든 저장 시각의 센서 전체와 실제 직경의 와이어를 대상으로 하며, 윈치 안의 줄은 의도적으로 제외한다. 계산이 중단되거나 수치 오류가 난 뒤의 미수렴 상태를 결과로 쓰지 않는다. 접촉력은 실측 물성이 아닌 수치 장벽 모델의 값이다.

## 파일과 검증 범위

- [70mm 실제 운동 재생](runs/opening_01/opening_70/mission/H1_replay.html), [56/70mm 비교](runs/opening_01/comparison.html)
- [70mm STEP 조립체](geometry_variants/opening_70/operating_cad/H1_door_140_assembly.step): 140° 열린 도어와 변경 통로. STEP 재입력에서 유효한 솔리드 24개를 확인했다.
- [70mm 입력](../../examples/h1_opening_70.yaml), [CAD 생성 기록](geometry_variants/opening_70/geometry.json), [도어 회전 검사](geometry_variants/opening_70/door_swing.json)
- [실제 AVL 연결 근거](AERO_CONNECTION.md), [원시 비교 결과](runs/opening_01/comparison.json)

자동 검사 62개가 통과했다(`outputs/validation/opening_tests.xml`). 새 검사는 실제 STL의 천장·측벽·프레임 치수와 변경 문판의 각도별 위치를 확인한다. 기존 56mm 계산과 핵심 운동 코드의 해시가 같고, 새 실행 중 계산 코드가 바뀌지 않았음을 확인했다.

기체 양력면은 기존의 **실제 AVL 3.52 공력표**를 사용했다. 구멍·문의 유동 변화를 AVL로 새로 해석한 것은 아니다. flow5는 연결하지 않았다. 형상 영향 비교를 위해 가정한 질량·CG·관성을 유지했으며 재료 증감 질량은 반영하지 않았다. 센서·줄 공력, 후류, 접촉 물성, 구조 강도와 변경안의 시간·줄 분할 수렴은 미검증이다.

## 재실행

PowerShell에서 `D:/DBF/안정성 툴`을 현재 폴더로 지정한다. 기존 출력과 원본 CAD를 보존한다. CAD 생성기는 기존 형상 폴더를 덮어쓰지 않는다.

```powershell
.venv/Scripts/python test_models/H1_reference/run_opening_study.py --output outputs/h1_opening_new --heights 70 --workers 1
.venv/Scripts/python test_models/H1_reference/audit_trajectory_collisions.py outputs/h1_opening_new/opening_70/mission --stride 1 --workers 6
.venv/Scripts/python test_models/H1_reference/visualize_run.py --run outputs/h1_opening_new/opening_70/mission
```

형상 생성과 문 회전 검사, 독립 CAD 검사는 여러 프로세스를 사용한다. 운동 비교 대상은 문 간섭 검사를 통과한 70mm 한 안이다. `run_opening_study.py`는 문 검사 실패·오류 또는 검사 후 CAD 변경을 발견하면 계산을 시작하지 않는다. 형상 재생성은 프로젝트 안의 복사본만 읽는 `build_opening_variants.py`로 수행한다.
'''
    (HERE/'OPENING_CHANGE_RESULTS.md').write_text(report_md,encoding='utf8')
    latest_info={'path':str(new.parent.relative_to(ROOT)),'checks':{'mission':{k:latest[k] for k in ('status','duration_s','captured','geometry_validity','max_tension_N','max_contact_N')}},
        'normal_mission_complete':latest['status']=='completed' and latest['duration_s']>=7-1e-8 and latest['captured'] and not latest['cad_overlap_samples'] and not latest['cad_error_samples'],
        'numerically_converged':False,'selected_case_note':'70 mm opening; actual outcome is recorded in checks, not a manufacturing release.',
        'report':'test_models/H1_reference/OPENING_CHANGE_RESULTS.md','comparison':str((args.root/'comparison.html').relative_to(ROOT))}
    (HERE/'latest_run.json').write_text(json.dumps(latest_info,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k!='source_hash_checks'},indent=2))


if __name__=='__main__':main()
