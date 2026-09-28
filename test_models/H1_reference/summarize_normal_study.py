"""Publish recorded R3 analysis and explicitly scoped saved-state CAD audits."""
from pathlib import Path
import hashlib
import json
from html import escape
import numpy as np
import pandas as pd
from dbf_stability.analysis import load_result
from dbf_stability import AeroDatabase
import plotly.graph_objects as go
from plotly.subplots import make_subplots

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'runs/normal_r3_01'
GEOMETRY=HERE/'geometry_variants/normal_r3'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    versions=json.loads((OUT/'source_versions.json').read_text(encoding='utf8'))
    for relative,expected in versions.items():
        if digest(ROOT/relative)!=expected:raise ValueError(f'Solver changed after launch: {relative}')
    runs={name:load_result(OUT/name/'mission') for name in ('nominal','time_refined')}
    from capture_check import capture_readiness
    capture={name:capture_readiness(r) for name,r in runs.items()}
    for name,data in capture.items():
        (OUT/name/'mission/capture_readiness.json').write_text(json.dumps(data,indent=2),encoding='utf8')
    audits={}
    for name,r in runs.items():
        folder=OUT/name/'mission'
        a=json.loads((folder/'cad_collision_audit.json').read_text(encoding='utf8'))
        if not a['samples']:raise ValueError('Missing sampled CAD audit')
        if a['source_states_sha256']!=digest(folder/'states.npz') or a['source_inputs_sha256']!=digest(folder/'inputs.json'):raise ValueError('Stale CAD audit')
        audits[name]={k:a[k] for k in ('samples','saved_states','sample_selection','every_saved_state_checked','intersecting_samples','cad_query_error_samples','geometry_validity')}
    metrics=('max_tension_N','max_pitch_change_deg','contact_impulse_Ns','max_contact_N')
    sanity={}
    for name,r in runs.items():
        expected=r.config['aircraft']['mass_kg']+r.config['sensor']['mass_kg']+r.config['cable']['length_m']*r.config['cable']['density_kg_m']
        error=float(abs(r.table.total_mass_kg-expected).max())
        finite=bool(np.isfinite(r.states).all())
        if not finite or error>1e-10:raise ValueError(f'Invalid finite-state/mass bookkeeping: {name}')
        sanity[name]=dict(all_states_finite=finite,total_mass_kg=expected,max_mass_error_kg=error)
    differences={key:100*abs(runs['nominal'].summary[key]-runs['time_refined'].summary[key])/max(abs(runs['time_refined'].summary[key]),1e-12) for key in metrics}
    comparison=json.loads((OUT/'aerodynamics/trim_comparison.json').read_text(encoding='utf8'))
    equilibrium=json.loads((OUT/'equilibrium/comparison.json').read_text(encoding='utf8'))
    door=json.loads((GEOMETRY/'door_swing.json').read_text(encoding='utf8'))
    shape=json.loads((GEOMETRY/'geometry.json').read_text(encoding='utf8'))
    result=runs['nominal'];s=result.summary
    report=dict(geometry=shape,door_sweep={k:door[k] for k in ('sampled_angles','intersecting_angles','error_angles')},
        trim_comparison=comparison,equilibrium=equilibrium,
        missions={name:{k:r.summary[k] for k in ('status','duration_s','captured',*metrics,'violations','runtime_s')} for name,r in runs.items()},
        saved_state_cad=audits,capture_conditions=capture,numerical_sanity=sanity,time_refinement_change_percent=differences,
        full_mission_completed=(all(r.summary['status']=='completed' and r.summary['captured'] for r in runs.values())
            and all(not a['intersecting_samples'] and not a['cad_query_error_samples'] for a in audits.values())),
        time_refinement_within_5_percent=(all(r.summary['status']=='completed' for r in runs.values())
            and all(differences[k]<=5 for k in metrics)),
        cable_spatial_convergence_verified=False,physical_validation='Unvalidated assumed test aircraft, not team aircraft or a safety approval',
        actual_avl_grid_cases=315,aero_sha256=digest(OUT/'aerodynamics/aero_database.npz'))
    (OUT/'comparison.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
    normal=comparison['normal_r3'];old=comparison['previous'];trim=normal['trim']
    rows=[('기체 질량 / kg',old['mass_kg'],normal['mass_kg']),('수납 트림 받음각 / °',old['trim']['alpha_deg'],trim['alpha_deg']),
          ('수납 트림 엘리베이터 / °',old['trim']['elevator_deg'],trim['elevator_deg']),('수납 트림 추력 / N',old['trim']['thrust_N'],trim['thrust_N']),
          ('기체 CG 기준 −Cmα/CLα / % MAC',100*old['actual_AVL']['static_margin_fraction'],100*normal['actual_AVL']['static_margin_fraction'])]
    table='| 항목 | 이전 H1 | 새 고익기 R3 |\n|---|---:|---:|\n'+'\n'.join(f'| {k} | {a:.4f} | {b:.4f} |' for k,a,b in rows)
    mission_rows='| 조건 | 계산 종료 / s | 포획 | 최대 장력 / N | 최대 피치 변화 / ° | 상태 |\n|---|---:|---|---:|---:|---|\n'
    for name,r in runs.items():
        q=r.summary
        mission_rows+=f"| {name} | {q['duration_s']:.6f} | {'성공' if q['captured'] else '실패'} | {q['max_tension_N']:.4f} | {q['max_pitch_change_deg']:.4f} | {q['status']} |\n"
    motion='요청한 전 과정을 계산하고 센서를 포획했다.' if report['full_mission_completed'] else '전 과정의 회수·포획 성공 조건을 충족하지 못했다. 아래 상태와 실제 종료 시각을 확인해야 한다.'
    capture_table='| 계산 | 포획 조건에 가장 가까운 저장 시각 / s | 위치 오차 / mm | 상대속도 / m/s | 상대각도 / ° |\n|---|---:|---:|---:|---:|\n'
    for name,data in capture.items():
        if data['status']=='evaluated':
            best=data['closest_combined_conditions']
            capture_table+=f"| {name} | {best['time_s']:.4f} | {best['position_error_mm']:.4f} | {best['relative_speed_m_s']:.4f} | {best['relative_angle_deg']:.4f} |\n"
    text=f'''# 고익기 시험 모델 R3 — 형상 변경과 재해석

기체 외형을 새로 만들고 실제 AVL 3.52 공력으로 다시 계산했다. {motion}

## 형상

날개폭 1.8 m, 주익 면적 0.555 m², 동체 길이 1.56 m인 고익기 시험 설계다. 테이퍼 주익에 상반각 4°와 끝단 비틀림 1°를 주었고 수평·수직 꼬리, 전방 프로펠러, 삼륜 착륙장치를 구성했다. 예전 동체 아래의 긴 절개를 없애고 센서 수납부를 닫힌 동체 안에 넣었다. 출구는 56 × 70 mm이며 그 위로 꼬리붐이 연결된다.

CAD는 {shape['solids']}개 유효 솔리드다. 도어는 최대 140°로 열리며 0–180°의 181개 정적 위치에서 고정 부품과 겹치지 않았다. 정렬된 센서·직선 줄 경로는 41개 검사 위치에서 최소 약 2 mm 여유를 보였다. 이 정적 경로 검사가 실제 운동의 성공을 뜻하지는 않는다.

윈치를 앞쪽으로 옮기고 별도 줄 안내구와 50 × 50 mm 포획 가이드를 추가했다. 회수 끝부분의 줄 속도를 낮춘 명령을 입력했으며 기체를 안정화하는 자동조종기는 넣지 않았다. 부품별 가정 질량을 CAD에 배분해 무게중심과 관성을 다시 계산했다. 기체 2.300 kg에 센서 0.040 kg과 줄 0.00225 kg을 별도로 더한다.

## 실제 AVL 계산

새 CAD와 같은 주익·꼬리 평면형, 상반각, 비틀림으로 315개 조건을 실제 실행했다. 동체는 외부 CAD의 단면적을 같은 면적의 원형 단면으로 바꾼 AVL BODY 모델이다. 별도 8개 계산으로 격자 세분화와 동체 포함 여부를 비교했고 이전·새 기체의 수납 트림점도 직접 다시 실행했다.

20 m/s, 공기 밀도 1.225 kg/m³에서 계산한 비교다. 형상과 가정 질량이 함께 달라진 비교이므로 한 변수의 개선 효과로 해석하면 안 된다.

{table}

표의 −Cmα/CLα는 AVL 모멘트 기준점인 센서·줄 제외 기체 CG에서 구했다. 센서·줄을 수납한 전체 무게중심 기준의 정적 여유와는 구분해야 한다.

새 트림점에서 공력표 보간과 직접 AVL 실행의 차이는 축력 {abs(normal['interpolation_minus_direct']['Fx_N']):.4f} N, 피치 모멘트 {abs(normal['interpolation_minus_direct']['My_Nm']):.5f} N·m다. 음의 기체 받음각은 주익 장착각과 캠버를 포함한 트림 결과다.

기체 단독 트림 주위 선형화에서는 증가하는 모드가 발견되지 않았다. 위치·방향의 중립 모드는 제외한 결과다. 센서가 완전히 전개된 결합계의 비접촉 평형은 줄 분할 10·20·40에서 찾지 못했으므로, 해당 계의 고유값이나 안정 판정을 내리지 않았다.

## 전개·회수 계산

{mission_rows}

포획하려면 위치 오차 6mm, 상대속도 0.15m/s, 상대각도 8° 조건을 동시에 충족해야 한다. 가장 가까웠던 저장 상태를 비교했다. 이는 적분 중의 연속 포획 이벤트를 대신하는 판정이 아니다.

{capture_table}

기준 계산과 최대 시간 간격·접촉 중 이동거리 상한을 절반으로 줄인 계산을 처음부터 각각 실행했다. 짧은 접촉을 놓치지 않도록 접촉 구간의 적분 시각과 중간 시각도 저장했다. STEP 검사는 0.01초 간격의 자세, 각 구간의 최대 접촉력·장력 및 최소 메시 거리 자세, 이벤트 주변과 모든 재생 프레임을 포함한다. 기준 {audits['nominal']['samples']}개 / 저장 {audits['nominal']['saved_states']}개, 세분화 {audits['time_refined']['samples']}개 / 저장 {audits['time_refined']['saved_states']}개 상태를 검사했다. 이는 모든 적분 시각이나 연속 시간에 대한 CAD 무간섭 증명은 아니다. 상세 결과는 `runs/normal_r3_01/comparison.json`에 있다.

최대 장력 변화 {differences['max_tension_N']:.2f}%, 피치 변화 {differences['max_pitch_change_deg']:.2f}%, 접촉 충격량 변화 {differences['contact_impulse_Ns']:.2f}%, 최대 접촉력 변화 {differences['max_contact_N']:.2f}%다. 계산이 도중에 끝났다면 이 차이를 전 과정 수렴으로 해석할 수 없다. 줄 분할 수에 대한 전 과정 수렴은 아직 확인하지 않았다.

## 확인 범위

센서·줄 항력, 기체 추가 항력, 질량·재료·마찰 계수, 후류는 실측하지 않은 입력이다. 표의 접촉력은 모든 접촉점의 수직 반력 크기를 합한 값이고, 접촉 충격량은 그 합을 시간에 대해 적분한 값이다. 개별 부품의 최대 하중이나 방향을 가진 총 충격량과는 구분해야 한다. 접촉력은 수치적 관통 방지 모델의 값이며 부품 충격하중의 정확도를 검증한 값이 아니다. 도어 관성은 닫힌 상태로 고정하고 문과 윈치를 지정 이력으로 구동한다. 기체 양력면과 등가 동체는 AVL로 계산했지만 열린 출구의 박리, 프로펠러 후류, 구조 강도는 계산하지 않았다. flow5는 연결하지 않았다.

이 모델은 팀 기체의 제작 승인이나 비행 안전 판정에 쓸 수 있는 검증 모델이 아니다. [MIT AVL 설명서](https://web.mit.edu/drela/Public/web/avl/avl_doc.txt)도 동체 근사와 작은 각도·준정상 유동의 적용 범위를 명시한다.

## 파일과 재실행

- 형상: `geometry_variants/normal_r3/model_overview.html`
- 조립 STEP: `geometry_variants/normal_r3/operating_cad/H1_door_140_assembly.step`
- 문 닫힌 조립 STEP: `geometry_variants/normal_r3/operating_cad/H1_door_closed_assembly.step`
- 입력: `examples/h1_normal_r3.yaml` (프로젝트 루트 기준)
- 실제 공력과 원본: `runs/normal_r3_01/aerodynamics/`
- 실제 AVL 분포 양력·공력 곡선: `runs/normal_r3_01/aerodynamics/aero_report.html`
- 기체 단독 1° 교란 응답: `runs/normal_r3_01/airframe_response/response.html`
- 기준 계산·재생: `runs/normal_r3_01/nominal/mission/`
- 더 작은 시간 간격: `runs/normal_r3_01/time_refined/mission/`
- 전체 수치 비교: `runs/normal_r3_01/comparison.json`

프로젝트 루트에서 가상환경 Python으로 `test_models/H1_reference/run_refined_study.py`를 실행한다. `--case examples/h1_normal_r3.yaml --output test_models/H1_reference/runs/새_실행명`을 지정하고 `screen`, `database`, `equilibrium`, `mission --time-refined` 순서로 진행한다. 이전 출력은 덮어쓰지 않는다. 원본 모델링 프로젝트를 다시 읽을 필요는 없다.
'''
    (HERE/'NORMAL_AIRCRAFT_RESULTS.md').write_text(text,encoding='utf8')
    html_rows=''.join(f'<tr><th>{escape(k)}</th><td>{a:.4f}</td><td>{b:.4f}</td></tr>' for k,a,b in rows)
    status_labels={'completed':'요청 시간 완료','contact_clearance_limit':'최소 간격 한계로 중단',
        'numerical_failure':'수치 계산 중단','geometry_invalid':'CAD 관통 확인',
        'aero_domain_exceeded':'공력표 범위 이탈','ground_contact':'지면 도달'}
    html_missions=''
    for name,r in runs.items():
        q=r.summary;ad=audits[name]
        html_missions+=f'''<tr><th>{'기준' if name=='nominal' else '시간 세분화'}</th>
        <td>{q['duration_s']:.3f}초 · {escape(status_labels.get(q['status'],q['status']))}</td>
        <td>{'성공' if q['captured'] else '미포획'}</td><td>{q['max_tension_N']:.3f} N</td>
        <td>{q['max_pitch_change_deg']:.3f}°</td><td>{ad['samples']} / {ad['intersecting_samples']} / {ad['cad_query_error_samples']}</td></tr>'''
    html=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>고익기 R3 재해석</title><style>body{{max-width:1100px;margin:40px auto;font:16px/1.7 system-ui;color:#20333d;background:#f7fafb;padding:0 20px}}a{{color:#126378}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:12px;border-bottom:1px solid #d7e2e8;text-align:left}}.notice{{padding:18px;background:#fff1da;border-radius:8px}}iframe{{width:100%;height:800px;border:0}}</style>
    <h1>고익기 R3 · 외형을 새로 만든 시험 기체</h1><p>실제 AVL 3.52 · 315개 공력 조건 · 40개 CAD 부품</p>
    <p><a href="../../geometry_variants/normal_r3/model_overview.html">새 기체 3D 형상</a> · <a href="nominal/mission/H1_replay.html">실제 계산 재생</a> · <a href="aerodynamics/aero_report.html">AVL 분포 양력·공력 곡선</a> · <a href="airframe_response/response.html">1° 교란 응답</a> · <a href="comparison.json">전체 수치와 검증 기록</a> · <a href="aerodynamics/direct_trim/normal_r3/stability.txt">AVL 트림점 원본</a></p>
    <p class="notice">{escape(motion)} 기준 계산: {s['duration_s']:.3f} / 11초, 포획 {'성공' if s['captured'] else '안 됨'}. 실물·충격하중은 미검증입니다.</p>
    <table><tr><th>20 m/s 수납 트림</th><th>이전 H1</th><th>새 고익기 R3</th></tr>{html_rows}</table>
    <p>−Cmα/CLα는 센서·줄을 제외한 기체 CG 기준의 값입니다. 전체 수납 무게중심 기준의 정적 여유와 구분해야 합니다.</p>
    <p>기체 단독에서는 증가하는 선형 모드가 발견되지 않았습니다. 전개된 센서 결합계의 비접촉 평형은 찾지 못해 안정 판정을 보류했습니다.</p>
    <h2>전개·회수 결과</h2><table><tr><th>계산</th><th>종료 시각과 상태</th><th>포획</th><th>최대 장력</th><th>최대 피치 변화</th><th>CAD 검사 / 겹침 / 오류</th></tr>{html_missions}</table>
    <p>CAD는 0.01초 간격과 각 구간의 최대 하중·최소 간격, 이벤트 주변, 모든 재생 프레임을 검사했습니다. 연속 시간 전체의 무간섭 증명은 아닙니다.</p>
    <p>시간 세분화에 따른 변화: 최대 장력 {differences['max_tension_N']:.2f}%, 피치 {differences['max_pitch_change_deg']:.2f}%, 접촉 충격량 {differences['contact_impulse_Ns']:.2f}%, 최대 접촉력 {differences['max_contact_N']:.2f}%. 도중에 중단된 경우 전 과정 수렴을 판정할 수 없습니다. 줄 분할 수렴과 실측 충격 검증도 남아 있습니다.</p>
    <iframe title="고익기 3D 형상" src="../../geometry_variants/normal_r3/model_overview.html"></iframe></html>'''
    (OUT/'report.html').write_text(html,encoding='utf8')
    latest=dict(path=str(OUT/'nominal'),case=str(ROOT/'examples/h1_normal_r3.yaml'),report=str(OUT/'report.html'),
        normal_mission_complete=report['full_mission_completed'],numerically_converged=False)
    (HERE/'latest_run.json').write_text(json.dumps(latest,indent=2),encoding='utf8')
    print(json.dumps({k:report[k] for k in ('missions','saved_state_cad','time_refinement_change_percent','full_mission_completed')},indent=2),flush=True)


if __name__=='__main__':main()
