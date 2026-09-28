# flow5 연결과 도어·저장 상태 연동

`examples/h1_normal_r3_flow5.yaml`을 선택하면 실제 flow5 7.57이 공력표를 만든다. 생성된 공력표는 기존 `solve_trim`, `analyze_stability`, `simulate`, `run_sweep`에서 그대로 사용한다. 실행이 실패하거나 필요한 공력값이 빠지면 오류로 종료한다.

## 이번에 연결한 범위

- R3 날개·꼬리날개의 형상, 비틀림, 에어포일과 조종면 구간을 flow5 XML로 전달한다. 조종면 경계에서 양력면을 나누고 에어포일 후연을 회전한다. AVL의 선형화된 조종면 법선 처리와는 차이가 있다.
- 54개 받음각·옆미끄럼각·승강타 조합과 에일러론·러더 중앙차분을 합해 **270조건을 실제 실행**했다. 각 프로세스는 별도 폴더를 사용한다.
- flow5 7.57의 힘·모멘트 출력을 기체 FRD축으로 변환한다. 특히 출력의 `CY`는 기체 y축 값이고, `CL`, 항력과 모멘트는 바람축 값이다. 파일의 `CX` 열은 실제로 총 항력 값이므로 기체 x축 힘으로 직접 쓰지 않는다.
- 회전율 미계수는 flow5가 출력하는 종·횡방향 블록을 사용한다. 출력되지 않는 교차항은 0으로 두는 **명시적 모델 가정**이며, `rate_derivative_closure: classical_longitudinal_lateral`을 지정해야 실행된다. 큰 옆미끄럼이나 비대칭 조종 상태에서 이 가정은 미검증이다.
- 도어 닫힘 명령은 포획 후 0.2초가 지나야 실행한다. 늦게 포획하면 원래 닫힘 이력을 시간 이동하므로 문 각도가 순간적으로 바뀌지 않는다. 포획 조건과 속도는 바꾸지 않는다.
- 정상 수락된 적분 상태를 주기적으로 저장한다. 수치 계산 정체, 시간 한도, 계산 횟수 한도로 중단돼도 수락된 부분 궤적과 체크포인트를 남긴다. Newton 반복 중의 시험 상태는 재개 상태로 사용하지 않는다.

**flow5 자동 연결은 현재 VLM2 양력면 해석 범위다. 동체 공력·도어 후류·센서 후류를 flow5로 계산한 결과가 아니다.** 충돌 계산에는 기존 전체 기체 CAD가 그대로 들어간다. 센서·줄 공력과 추가 항력은 출처가 표시된 기존 가정값이다.

## 실행

프로젝트 폴더에서 실행한다. 결과 폴더는 기존 결과와 다른 이름을 사용한다.

```powershell
.venv/Scripts/python scripts/fetch_flow5.py
.venv/Scripts/python -m dbf_stability.cli build-aero --case examples/h1_normal_r3_flow5.yaml --aero outputs/flow5_new/aero_database.npz --workers 4
.venv/Scripts/python -m dbf_stability.cli trim --case examples/h1_normal_r3_flow5.yaml --aero outputs/flow5_new/aero_database.npz --phase stowed
.venv/Scripts/python -m dbf_stability.cli simulate --case examples/h1_normal_r3_flow5.yaml --aero outputs/flow5_new/aero_database.npz --duration 3 --output outputs/flow5_new/deployment
```

실행된 예시는 `test_models/H1_reference/runs/flow5_connection_01`에 있다. 원본 `script.xml`, `plane.xml`, `polar.xml`, 에어포일, 실행 로그, 계산점 출력과 flow5에서 열 수 있는 `dbf.fl5`를 모두 보존한다. 다운로드는 [공식 배포](https://github.com/techwinder/flow5/releases/tag/v7.57)를 사용한다.

## 재개와 부분 결과

```powershell
.venv/Scripts/python -m dbf_stability.cli resume --case examples/h1_normal_r3_flow5.yaml --aero outputs/flow5_new/aero_database.npz --checkpoint outputs/flow5_new/deployment/accepted_checkpoint.npz --duration 1 --output outputs/flow5_new/continuation
```

재개 시 물리 입력, 형상, 공력표와 계산 소스 해시를 검사한다. 계산 허용 오차·시간 한도 등 `simulation` 설정은 조정할 수 있다. 재개 결과의 최대값과 충격량은 **재개 구간에만 해당**하며 이전 구간과 합산한 값이 아니다. 이전 형식의 체크포인트에는 포획 상태가 없으므로 자동 재개를 거부한다.

`accepted_progress.json`은 마지막 수락 시각이다. 기존 `progress.json`과 `debug_last_trial.npz`는 적분기의 시험 상태를 포함할 수 있다. `accepted_history/part_*.npz`에는 각 수락 단계와 중간 보간 상태가 저장된다. 강제 종료 후에는 체크포인트로 재개할 수 있지만, 저널 자체를 완성된 임무 결과로 표시하지 않는다.

## 검증과 남은 일

실제 flow5 출력의 부호·축 변환·조종면 반응, 도어 연동, 재개 상태 연속성을 검사했다. AVL과는 동체를 제외한 11조건을 직접 비교했다. 이 비교에서 두 해석기가 완전히 일치하지 않으며, 일치율을 정확도나 수렴성으로 해석하지 않는다.

2026-09-17 실행 기록은 다음과 같다. [실행된 노트북](../notebooks/05_flow5_and_sequence_connection.ipynb)과 [브라우저 결과](../test_models/H1_reference/runs/flow5_connection_01/connection_notebook.html)에 표·그래프·재생 링크를 모았다.

| 검사 | 확인한 결과 |
|---|---|
| 전체 자동 검사 | 93개 통과. `outputs/validation/all_connected_tests.xml` |
| 수납 트림 | 받음각 −1.32693°, 승강타 2.93029°, 추력 4.82192 N |
| 기체 단독 +1° 피치 교란 | 20초 계산 완료. 센서 결합 안정성 검사가 아님 |
| flow5를 적용한 센서 전개 | 0–3초 완료. 최대 장력 2.38792 N, 초기 대비 최대 피치 변화 1.70975°. 하중·수렴 미검증 |
| 중단 후 재개 | 600초 실행 한도로 임무 시각 2.21494초에서 중단. 같은 물리 입력·소스·상태로 3초까지 재개 완료 |
| 전개 궤적 STEP 검사 | 선택한 496시각에서 겹침 0건, 형상 조회 오류 0건. 연속 무충돌 증명은 아님 |
| 도어 연동 (AVL) | 기존 상태에서 9.70–11초 재계산. 포획 실패 시 140° 개방 유지. 250시각 STEP 검사 겹침 0건 |
| 도어 연동 상태 재개 | 11–11.02초 계산. 위치·속도·자세·포획 상태 연속, 도어 각도 차이 0° |

전개 결과의 `flow5_deployment/mission`은 처음 중단된 구간, `continuation`은 재개 구간, `combined`는 상태 변경 없이 두 구간을 이어 놓은 결과다. `combined/summary.json`에는 원본 경로·해시·중단 상태를 남겼다. 최대값과 누적 충격량은 연결한 전체 0–3초에서 다시 집계했다. 회수·포획까지 flow5로 실행한 결과는 아직 아니다.

R3의 센서 정렬과 포획 성공, 내부 가이드 접촉이 있는 견인 평형, 결합 고유모드, 10→20→40구간 수렴은 이번 연결만으로 해결되지 않았다. 기존 R3 해석과 소스는 별도로 보존했다.

원본 근거: [자동 실행과 미계수 출력에 관한 개발자 설명](https://github.com/techwinder/flow5/discussions/1), [7.57 좌표·힘·모멘트 구현](https://github.com/techwinder/flow5/blob/v7.57/flow5-lib/objects3d/analysis3d/aeroforces.cpp), [실제 출력 열 정의](https://github.com/techwinder/flow5/blob/v7.57/flow5-lib/objects3d/analysis3d/planeopp.cpp).


## 복합 공력표 · 구현 기준

`aero.backend: hybrid`와 다음 조합을 사용한다. 빌드 진입점은 기존 `build_aero_database(config, output, workers)`이며, 별도 호출은 `dbf_stability.hybrid.compose_tables(sources, composition, output)`이다. `sources`에는 `avl`, `flow5` 키로 원본 npz 경로나 `AeroDatabase`를 전달한다.

```yaml
aero:
  backend: hybrid
  hybrid:
    coeff: flow5
    controls: flow5
    rates: avl
```

- `coeff`는 FRD `[CX,CY,CZ,Cl,Cm,Cn]`, `controls`는 도당 에일러론·러더 미계수, `rates`는 `[pb/2V,qc/2V,rb/2V]`당 미계수다. 승강타 효과는 `coeff`의 승강타 격자에 들어간다.
- 세 출처가 같으면 단일 해석기를 요구한다. `rates: flow5`에는 `rate_derivative_closure: classical_longitudinal_lateral`이 필요하다. AVL 회전율을 쓸 때 flow5 하위 실행에만 이 설정을 주고, 그 회전율 블록은 버린다. 원래 설정은 바꾸지 않는다.
- 두 원본의 형상·참조 파일 해시와 격자는 같아야 한다. 기준 면적·시위·날개폭은 두 값 중 큰 값 기준 상대 1e-5 이내, 기준점은 절대 1e-9 m 이내여야 한다. AVL 원본 파일과 실제 실행 폴더의 `plane.avl` 헤더를 모두 CG와 대조한다.
- 새 UI 등록은 CG를 AVL 기준점으로 기록한다. 기존 단일 해석용 등록 파일과 표는 그대로 보존한다. 기존 운동 모델은 공력 블록을 합한 힘·모멘트에 기준점 이동을 적용하므로, 정적 모멘트만 이동한다고 해석해서는 안 된다. 회전 중심까지 맞추기 위해 복합 계산은 두 원본 모두 CG에서 계산하도록 제한한다.
- 두 원본은 출력 폴더 아래 `avl/`, `flow5/`에 보존한다. 원본 npz 해시, 실행 경로·버전, 선택·제외 블록, 계산 속도, 비교 결과가 최종 메타데이터에 들어간다. 기존 출력 파일과 하위 실행 폴더가 있으면 덮어쓰지 않고 오류를 낸다.
- `consistency_report`는 각 각도의 0°에 가장 가까운 점에서 CLα, Cmα, CYβ, Clβ, Cnβ를 비교한다. 내부점에서는 양쪽 이웃의 차분, 경계에서는 한쪽 차분을 쓴다. 사용한 점과 간격을 기록한다. 상대 차이는 `abs(a-b)/max(abs(a),abs(b))`다. 두 기울기가 모두 1e-10 이하면 상대값은 null이고 절대 차이를 기록한다. 상대 20% 초과는 경고다.
- flow5 속도는 `flow5_speed_m_s`로 저장한다. 새 flow5·복합 표는 현재 비행 속도와 다르면 재사용을 거부한다. 오래된 단일 flow5 표의 속도 검사는 UI 실행 요청 기록을 사용한다.
- 작업자 수 1은 순차 실행, 2 이상은 두 해석기에 분배한다. 각 flow5 작업의 스레드는 1개다. AVL의 기존 메모리 예산 검사도 유지한다.

두 해석기의 차이가 작아도 실제 기체의 정확도를 입증하지 않는다. 동체·문·후류·점성 효과의 추가 검증과 수치 수렴 검사는 별도다. 서로 다른 격자·기준점의 표를 보간하거나 이동해 결합하는 선택 항목 B5는 구현하지 않았다.

### 브라우저 연결 검증에서 확인한 트림 제한

2026-09-28 생성한 직사각 양력면 시험 기체에서 기본 조합(flow5 정적·조종 + AVL 회전율)의 공력표 생성은 완료됐다. 그러나 기존 대칭 직선비행 트림은 작은 횡방향 잔차를 제거하지 못해 중단됐다. 옆미끄럼각 0°를 포함한 27조건 실행 `9184201583ba45d69cf9bd1f39d0d0d0`의 선가속도 잔차는 4.22382e-5 m/s², 각가속도 잔차는 2.30845e-4 rad/s²로 각각의 허용값 1e-5를 넘었다. β=0° 원본에도 작은 CY·Cl·Cn이 남아 있다. 공력 조합 성공과 전체 평형 성립은 별도 조건이다.

실패한 표의 계수를 0으로 수정하거나 트림 허용치를 늘리지 않았다. UI에서 정적 계수 출처를 AVL로 **직접 선택한 별도 실행** `0728c28bf7b147a8b1a622431bd81a46`은 트림·42개 모드·0.02초 응답을 완료했다. 조종 미계수는 flow5, 회전율은 AVL이다. 기본 조합이나 자동 재시도 동작은 바꾸지 않았다. 이 결과는 연결 검사이며, 시험 기체에는 불안정 모드가 검출됐다. 기본 조합의 전체 비행 계산을 항상 성공시킨다는 의미가 아니다. 공력표 생성 뒤 트림이 실패해도 출처·비교 결과와 원본 파일을 결과 카드에서 확인할 수 있다.
