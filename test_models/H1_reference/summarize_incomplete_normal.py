"""Report R3 without inventing missing histories or a convergence result."""
from pathlib import Path
import hashlib
from html import escape
import json
import numpy as np
from dbf_stability.analysis import load_result
from capture_check import capture_readiness

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / 'runs/normal_r3_01'
GEOMETRY = HERE / 'geometry_variants/normal_r3'


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    for relative, expected in read(OUT / 'source_versions.json').items():
        if digest(ROOT / relative) != expected:
            raise ValueError(f'Solver changed after launch: {relative}')
    cases = {}
    for name in ('nominal', 'time_refined'):
        folder = OUT / name / 'mission'
        summary = read(folder / 'summary.json')
        if summary['status'] == 'running':
            raise ValueError(f'{name} is still running; finalize or record cancellation first')
        data = dict(summary=summary, trajectory_available=False, cad_audit=None,
                    capture_conditions=None, numerical_sanity=None)
        if summary['status'] not in ('cancelled', 'failed'):
            result = load_result(folder)
            audit = read(folder / 'cad_collision_audit.json')
            if (audit['source_states_sha256'] != digest(folder / 'states.npz') or
                    audit['source_inputs_sha256'] != digest(folder / 'inputs.json')):
                raise ValueError('Stale CAD audit')
            if not audit['samples']:
                raise ValueError('Empty CAD audit')
            expected = (result.config['aircraft']['mass_kg'] + result.config['sensor']['mass_kg'] +
                        result.config['cable']['length_m'] * result.config['cable']['density_kg_m'])
            mass_error = float(abs(result.table.total_mass_kg - expected).max())
            finite = bool(np.isfinite(result.states).all())
            if not finite or mass_error > 1e-10:
                raise ValueError('Invalid states or total-mass bookkeeping')
            data.update(summary=result.summary, trajectory_available=True,
                        capture_conditions=capture_readiness(result),
                        numerical_sanity=dict(all_states_finite=finite, total_mass_kg=expected,
                                              max_mass_error_kg=mass_error),
                        cad_audit={k: audit[k] for k in ('samples', 'saved_states', 'sample_selection',
                            'every_saved_state_checked', 'intersecting_samples', 'cad_query_error_samples',
                            'geometry_validity', 'first_sampled_intersection')})
            (folder / 'capture_readiness.json').write_text(json.dumps(data['capture_conditions'], indent=2), encoding='utf8')
        cases[name] = data
    trim_comparison = read(OUT / 'aerodynamics/trim_comparison.json')
    response = read(OUT / 'airframe_response/comparison.json')
    equilibrium = read(OUT / 'equilibrium/comparison.json')
    report = dict(geometry=read(GEOMETRY / 'geometry.json'),
                  trim_comparison=trim_comparison, airframe_response=response, equilibrium=equilibrium,
                  missions=cases, actual_avl_grid_cases=315,
                  aero_sha256=digest(OUT / 'aerodynamics/aero_database.npz'),
                  full_mission_verified=False, time_convergence_verified=False,
                  cable_spatial_convergence_verified=False,
                  physical_validation='Assumed test aircraft; no team-aircraft or impact validation',
                  note='Unavailable histories and metrics remain absent. Trial-state progress is not accepted trajectory time.')
    (OUT / 'comparison.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf8')
    normal = trim_comparison['normal_r3']
    old = trim_comparison['previous']
    rows = [('기체 질량 / kg', old['mass_kg'], normal['mass_kg'])]
    for key, label in [('alpha_deg', '수납 트림 받음각 / °'), ('elevator_deg', '수납 트림 엘리베이터 / °'),
                       ('thrust_N', '수납 트림 추력 / N')]:
        rows.append((label, old['trim'][key], normal['trim'][key]))
    table = '| 항목 | 이전 H1 | 새 고익기 R3 |\n|---|---:|---:|\n' + '\n'.join(
        f'| {label} | {a:.4f} | {b:.4f} |' for label, a, b in rows)
    paragraphs = []
    replay_links = []
    for name, data in cases.items():
        label = '기준 계산' if name == 'nominal' else '시간 간격을 줄인 계산'
        q = data['summary']
        if not data['trajectory_available']:
            checkpoint = q.get('accepted_checkpoint_time_s')
            paragraphs.append(f"{label}: 중단. {q.get('reason', '')} "
                f"마지막으로 저장된 중간 상태는 {checkpoint}초다. 전체 시간 이력을 저장하지 못해 "
                '이 계산의 최대 장력·충격량·포획 여부는 확인할 수 없다.')
        else:
            ad = data['cad_audit']
            state_label={'completed':'요청 시간 완료','geometry_invalid':'CAD 관통 확인',
                         'contact_clearance_limit':'최소 간격 한계로 중단','numerical_failure':'수치 계산 중단'}.get(q['status'],q['status'])
            paragraphs.append(f"{label}: {q['duration_s']:.3f} / 11초, {state_label}, "
                f"포획 {'기록됨' if q['captured'] else '안 됨'}. 최대 장력 {q['max_tension_N']:.4f} N, "
                f"최대 피치 변화 {q['max_pitch_change_deg']:.4f}°, 접촉 반력 합의 최대값 {q['max_contact_N']:.4f} N, "
                f"그 합의 적분 {q['contact_impulse_Ns']:.5f} N·s. "
                f"저장 {ad['saved_states']}개 중 {ad['samples']}개 자세의 STEP 검사: 겹침 {ad['intersecting_samples']}개, "
                f"검사 오류 {ad['cad_query_error_samples']}개. 이는 연속 시간 전체의 무간섭 증명이 아니다.")
            cap = data['capture_conditions']
            if cap['status'] == 'evaluated':
                best = cap['closest_position']
                paragraphs.append(f"{label}에서 포획 위치에 가장 가까웠던 저장 시각은 {best['time_s']:.4f}초다. "
                    f"위치 오차 {best['position_error_mm']:.3f} mm, 상대속도 {best['relative_speed_m_s']:.4f} m/s, "
                    f"상대각도 {best['relative_angle_deg']:.3f}°다. 포획 허용치는 각각 6 mm, 0.15 m/s, 8°다.")
                last = cap['final_state']
                paragraphs.append(f"마지막 저장 상태의 위치 오차는 {last['position_error_mm']:.3f} mm, "
                    f"상대속도는 {last['relative_speed_m_s']:.5f} m/s다. 기체에 대한 센서의 롤·피치·요는 "
                    + ', '.join(f'{v:.3f}°' for v in last['relative_euler_xyz_deg']) + '다.')
            replay = OUT / name / 'mission/H1_replay.html'
            if replay.exists():
                replay_links.append((f"{q['duration_s']:g}초 전개·회수 재생", f'{name}/mission/H1_replay.html'))
    explanation = '\n\n'.join(paragraphs)
    text = f'''# 고익기 R3 — 새 형상과 재해석 결과

일반적인 고익기 형상으로 다시 만들고 실제 AVL 3.52로 공력을 계산했다. 기체 형상·공력표·기체 단독 응답을 제공한다. 전개·회수·포획의 전 과정 수렴과 센서 결합계 안정성은 확인하지 못했다.

## 형상과 CAD

날개폭 1.8 m, 주익 면적 0.555 m², 동체·꼬리붐 길이 1.56 m다. 매끈한 동체, 테이퍼 주익과 바깥쪽 상반각 4°, 수평·수직 꼬리, 전방 프로펠러, 삼륜 착륙장치를 구성했다. 센서 수납부는 닫힌 동체 안에 넣고 후방 출구 56 × 70 mm만 열었다. 문은 140°로 열린다. 기체 가정 질량 2.300 kg, 센서 0.040 kg, 줄 0.00225 kg을 각각 반영했다.

40개 유효 CAD 솔리드로 구성했고 문을 닫거나 연 조립 STEP을 재반입해 확인했다. 문 0–180°의 181개 정적 자세는 겹침 0개다. 정렬된 센서·직선 줄 경로의 41개 검사 위치는 최소 약 2 mm 여유가 있다. 정적 간섭 검사와 실제 회수 성공은 구분한다.

## 실제 AVL와 기체 응답

새 형상으로 315개 공력 조건을 실제 실행했다. 원본 입력, 명령, 출력과 실행 버전을 보존했다. 날개·꼬리의 평면형과 상반각·비틀림은 CAD와 AVL에 함께 반영했다. 동체는 CAD 단면과 같은 면적의 원형 단면을 쓰는 AVL BODY 근사이며 추가 항력 계수 0.035는 가정값이다. 별도 8개 계산으로 두 조건에서 격자와 BODY 포함 여부를 비교했다. 이 비교만으로 전체 공력 수렴을 보증하지는 않는다.

20 m/s, 밀도 1.225 kg/m³에서의 수납 트림이다. 형상과 질량을 함께 바꾼 비교다.

{table}

받음각은 기체축 기준이며 주익의 장착각(뿌리 2°, 끝 1°)과 익형 캠버를 포함한 트림 결과다.

보간 공력과 트림점 직접 AVL 실행의 차이는 축력 {abs(normal['interpolation_minus_direct']['Fx_N']):.4f} N, 피치 모멘트 {abs(normal['interpolation_minus_direct']['My_Nm']):.5f} N·m다. 기체 CG 기준 −Cmα/CLα는 {100*normal['actual_AVL']['static_margin_fraction']:.3f}% MAC이며 센서·줄을 수납한 전체 CG 기준 정적 여유와는 다르다.

기체 단독 선형화에서 증가하는 모드는 발견되지 않았다. 자동조종기 없이 피치를 1° 교란한 20초 응답에서 새 기체의 마지막 피치 편차는 0.05276°다. 센서·줄 힘을 제외한 시험이다. 전개된 결합계 평형은 줄 분할 10·20·40에서 찾지 못해 해당 고유값·안정 판정을 제시하지 않는다.

## 전개·회수 재계산

{explanation}

기준과 세분화 계산의 전 과정 비교를 완료하지 못했으므로 5% 수렴 판정을 내리지 않는다. 줄 분할 수에 대한 과도응답 수렴도 확인하지 못했다. 접촉 반력 합과 그 시간 적분은 개별 부품의 충격하중이나 방향을 가진 총 충격량을 뜻하지 않는다.

## 확인 범위와 파일

센서·줄 항력, 질량·관성, 접촉 강성·마찰과 후류는 가정 입력이다. 도어 관성은 닫힌 상태로 고정했고 문·윈치는 지정 이력으로 구동했다. 이 실행의 문 닫힘은 포획과 연동하지 않은 10.6초 명령이므로 포획 실패 상태에서도 닫혔다. 열린 출구 박리, 프로펠러 후류, 부품 강도와 실측 충격은 검증하지 않았다. flow5는 연결하지 않았다. [MIT AVL 설명서](https://web.mit.edu/drela/Public/web/avl/avl_doc.txt)의 동체·작은 각도·준정상 근사 범위를 고려해야 한다.

- [새 기체 3D 형상](geometry_variants/normal_r3/model_overview.html)
- [문 닫힌 조립 STEP](geometry_variants/normal_r3/operating_cad/H1_door_closed_assembly.step)
- [문 열린 조립 STEP](geometry_variants/normal_r3/operating_cad/H1_door_140_assembly.step)
- [실제 AVL 분포 양력·공력 곡선](runs/normal_r3_01/aerodynamics/aero_report.html)
- [기체 단독 1° 교란 응답](runs/normal_r3_01/airframe_response/response.html)
- [전체 수치와 계산 상태](runs/normal_r3_01/comparison.json)

프로젝트 입력은 `examples/h1_normal_r3.yaml`이다. 가상환경 Python으로 `test_models/H1_reference/run_refined_study.py`에 `--case examples/h1_normal_r3.yaml --output test_models/H1_reference/runs/새_실행명`을 지정한다. 단계는 `screen`, `database`, `equilibrium`, `mission --time-refined`다. 이전 출력은 덮어쓰지 않는다. 타 모델링 프로젝트를 읽지 않고 이 프로젝트에 저장된 파일만 사용한다.
'''
    (HERE / 'NORMAL_AIRCRAFT_RESULTS.md').write_text(text, encoding='utf8')
    links = [('새 기체 3D 형상', '../../geometry_variants/normal_r3/model_overview.html'),
             ('문 닫힌 STEP', '../../geometry_variants/normal_r3/operating_cad/H1_door_closed_assembly.step'),
             ('AVL 공력 결과', 'aerodynamics/aero_report.html'), ('1° 교란 응답', 'airframe_response/response.html'),
             ('수치·검증 기록', 'comparison.json'), ('AVL 원본 출력', 'aerodynamics/direct_trim/normal_r3/stability.txt')] + replay_links
    html_rows = ''.join(f'<tr><th>{escape(label)}</th><td>{a:.4f}</td><td>{b:.4f}</td></tr>' for label, a, b in rows)
    html = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>고익기 R3 형상과 재해석</title>
<style>body{{max-width:1120px;margin:36px auto;padding:0 24px;font:16px/1.7 system-ui;color:#20333d;background:#f7fafb}}a{{color:#126378}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:12px;border-bottom:1px solid #d7e2e8;text-align:left}}.notice{{padding:18px;background:#fff1da;border-radius:8px}}iframe{{width:100%;height:780px;border:0}}</style>
<h1>고익기 R3 · 새 형상과 실제 AVL 재해석</h1><p>날개폭 1.8 m · 매끈한 동체 · 테이퍼 주익 · 전방 프로펠러 · 삼륜 착륙장치</p>
<nav>{' · '.join(f'<a href="{url}">{label}</a>' for label,url in links)}</nav>
<p class="notice">형상·공력·기체 단독 응답은 계산했습니다. 회수·포획의 전 과정 수렴과 센서 결합계 안정성은 확인하지 못했습니다.</p>
<table><tr><th>20 m/s 수납 트림</th><th>이전 H1</th><th>새 고익기 R3</th></tr>{html_rows}</table>
<p>실제 AVL 3.52의 315개 공력 조건을 사용했습니다. 기체 단독 1° 피치 교란은 20초 뒤 편차 0.053°로 줄었습니다. 실물 시험으로 검증한 결과는 아닙니다.</p>
<h2>전개·회수 계산의 상태</h2>{''.join('<p>'+escape(p)+'</p>' for p in paragraphs)}
<p>문은 포획과 연동하지 않은 10.6초 폐문 명령을 따랐습니다. 시간·줄 분할 수렴 판정은 보류했습니다. 가정 질량·항력·접촉 물성을 사용했으며 열린 출구의 박리, 프로펠러 후류와 부품 충격하중은 검증하지 않았습니다.</p>
<iframe title="새 고익기 3D 형상" src="../../geometry_variants/normal_r3/model_overview.html"></iframe></html>'''
    (OUT / 'report.html').write_text(html, encoding='utf8')
    for name, data in reversed(list(cases.items())):
        if data['trajectory_available'] and (OUT / name / 'mission/H1_replay.html').exists():
            latest = dict(path=str(OUT / name), case=str(ROOT / 'examples/h1_normal_r3.yaml'),
                          report=str(OUT / 'report.html'), normal_mission_complete=False,
                          numerically_converged=False)
            (HERE / 'latest_run.json').write_text(json.dumps(latest, indent=2), encoding='utf8')
            break
    print(json.dumps(dict(report=str(OUT / 'report.html'), trajectory_cases=[k for k,v in cases.items() if v['trajectory_available']]), indent=2))


if __name__ == '__main__':
    main()
