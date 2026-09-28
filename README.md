# DBF 센서 전개·회수 안정성 툴

UI는 등록한 기체만 해석합니다. CAD를 불러온 뒤 부품·CG·공력 면을 정의하고 새 공력표를 계산하세요. [사용설명서](docs/사용설명서.md) · [복합 공력표 구현 기록](ui/design/hybrid-aero-20260928/IMPLEMENTATION.md)

## GitHub에서 받아 실행하기

Windows와 Python 3.11 이상, Node.js/npm이 필요합니다. 저장소에는 코드·테스트·입력 예제·문서를 담았습니다. 가상환경, 설치된 AVL/flow5, 생성 CAD, 과거 해석 결과, 사용자 프로젝트와 등록 데이터는 포함하지 않습니다.

```powershell
git clone https://github.com/djkim0320/AVL-Flow5.git
cd AVL-Flow5
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -e ".[contact,dev]"
npm.cmd --prefix ui ci
.venv\Scripts\python.exe scripts/fetch_avl.py
.venv\Scripts\python.exe scripts/fetch_flow5.py
.\Start_DBF_UI.bat
```

해석 화면에서 **AVL + flow5 복합**을 선택하고 정적 계수·조종·회전율 출처를 지정할 수 있습니다. 실제 해석기 결과만 사용하며 형상·격자·CG 기준점·계산 속도가 맞지 않으면 중단합니다. 기본 flow5 정적 조합은 시험 기체에서 대칭 트림 잔차를 넘긴 사례가 있으므로 [확인된 제한](docs/FLOW5_CONNECTION.md#브라우저-연결-검증에서-확인한-트림-제한)을 확인하세요.

새로 받은 저장소에서 실행할 수 있는 검사:

```powershell
.venv\Scripts\python.exe -m unittest discover -s ui/tests
npm.cmd --prefix ui run check
npm.cmd --prefix ui test
.venv\Scripts\python.exe -m pytest -q tests/test_hybrid_compose.py tests/test_ui_generated_pipeline.py
```

마지막 검사는 설치된 실제 AVL·flow5를 사용합니다. 전체 `tests/` 중 과거 임무·접촉 회귀 검사는 제외된 로컬 계산 기록을 필요로 합니다. 아래의 과거 보고서·생성 CAD·`outputs/` 링크도 개발 PC의 보관 자료를 가리키므로 GitHub 소스만으로는 열리지 않습니다. 배포 범위와 검증 내역은 [저장소 안내](docs/REPOSITORY.md)를 참고하세요.

## 연구용 시제품과 과거 실행 기록

실제 MIT AVL 3.52 또는 flow5 7.57 공력 데이터를 사용해 기체·센서·견인선의 운동을 계산하는 연구용 시제품이다. Windows용 Python 패키지와 Jupyter 노트북을 제공한다. **예시 입력은 팀 기체의 설계값이 아니다.**

수납·전개 트림, 고정 길이 결합 안정성, 후방 문·줄 전개·회수·포획, 병렬 설계 비교를 실행할 수 있다. **현재 전개·회수의 5% 수렴 기준은 미달이며, 계획의 전체 완료 조건을 충족한 상태는 아니다.** 정량적인 검증 범위와 남은 항목은 [검증 보고서](docs/VALIDATION.md)에 기록한다.

## 바로 확인할 파일

**2026-09-22 코드 점검:** [발견한 오류·수정 내용·시험 기록](docs/CODE_AUDIT_20260922.md). 수납 중 줄 길이 변경, 불완전한 트림 판정, 형상과 공력표 불일치, 중첩 병렬 실행, 편집기 단위·좌표·복원 문제를 수정했다. 기존 임무 결과를 새 코드로 재계산한 것으로 취급하지 않는다.

**3D 배치 편집기:** [실행·사용법](ui/README.md). 기체를 불러오고 윈치 위치·질점 질량·줄 길이를 설정한다. 기체 정의에서 부품·CG·물성·공력 면을 등록한 뒤 전개 비행 또는 회수를 해석한다. 로컬 서버 실행 후 [편집기](http://127.0.0.1:8767/)에서 사용한다.

보관된 모델링 사례에는 **와이어 2.7m(윙스팬 1.5배) 전개 → 60초 유지 → 회수**가 있다. [임무 일정·계산 결과](test_models/H1_reference/runs/span_hold_01/report.html), [입력·실행 안내](docs/SPAN_HOLD_MISSION.md)를 참고한다. 현재 flow5 입력은 `examples/h1_normal_r3_flow5_span_hold.yaml`이다.

추가한 [flow5 연결과 재개 안내](docs/FLOW5_CONNECTION.md)에는 실제 해석기 선택, 도어 포획 연동, 중단 후 재개 방법을 설명한다. `examples/h1_normal_r3_connected.yaml`은 AVL용 도어 연동 사례, `examples/h1_normal_r3_flow5.yaml`은 flow5용 사례다. flow5는 현재 동체를 제외한 VLM2 양력면 해석이며, 전체 임무의 포획 성공이나 수렴성을 입증한 상태는 아니다.

보관된 형상 중 하나는 **R3 고익기 시험 모델**이다. 닫힌 동체, 테이퍼 주익, 상반각, 꼬리붐, 전방 프로펠러와 삼륜 착륙장치를 새로 만들고 질량·관성·AVL 입력을 함께 바꿨다. [3D 형상](test_models/H1_reference/geometry_variants/normal_r3/model_overview.html), [실제 AVL 315개 조건과 분포 양력](test_models/H1_reference/runs/normal_r3_01/aerodynamics/aero_report.html), [기체 단독 교란 응답](test_models/H1_reference/runs/normal_r3_01/airframe_response/response.html)을 열어 볼 수 있다. 이전 H1 사례와 공력표를 섞어 쓰면 안 된다.

R3의 계산 상태와 한계는 [새 기체 재해석 결과](test_models/H1_reference/NORMAL_AIRCRAFT_RESULTS.md)에 기록한다. 아래 네 노트북과 기존 `outputs/mission_*`는 앞서 만든 기준 사례이며, R3의 회수·포획 검증 결과가 아니다.

| 파일 | 내용 |
|---|---|
| [01 기체 검증](notebooks/01_aircraft_validation.ipynb) | 실제 AVL 공력, 단독·수납·전개 트림 비교 |
| [02 전개 후 안정성](notebooks/02_deployed_stability.ipynb) | 전체 결합 고유값과 작은 센서 교란 응답 |
| [03 전개·회수·포획](notebooks/03_deploy_recover_capture.ipynb) | 공개한 조종 이력, 접촉과 포획, 3차원 재생 |
| [04 설계 민감도](notebooks/04_sensitivity_and_convergence.ipynb) | 다중 프로세스 계산과 수렴 기록 |
| [05 flow5·도어 연결](notebooks/05_flow5_and_sequence_connection.ipynb) / [결과 보기](test_models/H1_reference/runs/flow5_connection_01/connection_notebook.html) | 실제 flow5 270조건, AVL 비교, 기체 응답, 3초 전개와 도어 연동 |
| [모델 설명](docs/MODEL.md) | 식·좌표·가정·적용 범위 |
| [H1 형상 접촉 모델](docs/CONTACT_MODEL.md) | 동체·컨테이너·문과 센서·와이어의 충돌 반력, 관통 방지, 재계산 방법 |
| [H1 도어 개방각 비교](test_models/H1_reference/DOOR_CHANGE_RESULTS.md) | 100°·140°·160° 및 수정 힌지의 실제 재계산, STEP, 남은 접촉·포획 실패 |
| [H1 출구 높이 변경](test_models/H1_reference/OPENING_CHANGE_RESULTS.md) | 56→70mm 확대 형상과 실제 회수 재계산, 수정 STEP, 80mm 안의 도어 간섭 |
| [CATIA 인수 안내](docs/CATIA_HANDOFF.md) | 팀에 요청할 CAD·질량·물성 자료 |
| [입력 양식](docs/intake.csv) | 항목별 값·단위·출처·가정/측정/설계 구분 |

노트북을 실행하지 않고 읽으려면 `outputs/notebook_html`의 HTML을 연다. 전 과정 결과의 `history.html`은 확대 가능한 그래프, `replay.html`은 기체 기준 좌표로 보는 3차원 재생이다.

[40구간 전 과정 그래프](outputs/mission_40/history.html)와 [3차원 재생](outputs/mission_40/replay.html)은 포획한 사례다. 같은 입력의 10·20구간에서는 포획하지 못했으므로, 이 성공을 수렴한 예측으로 해석해서는 안 된다. [전체 비교](outputs/mission_convergence/convergence.json)에 결과를 모두 보존했다.

## 설치와 실행

프로젝트 폴더에서 PowerShell을 연다. 기존 `.venv`가 있으면 활성화 없이 아래 Python 경로를 그대로 사용해도 된다. 다른 PC에는 Python 3.11 이상을 설치하고 환경을 새로 만든다.

```powershell
py -3.11 -m venv .venv
.venv/Scripts/python -m pip install -e '.[dev]'
.venv/Scripts/python -m ipykernel install --prefix .venv --name dbf-stability --display-name 'DBF Python'
```

같은 직접 의존성 버전이 필요하면 `pip install -r requirements-lock.txt`를 먼저 실행한다. AVL 파일을 다시 받아야 할 때는 `scripts/fetch_avl.py`를 실행한다. 공식 다운로드의 실행파일 해시가 달라지면 자동 교체하지 않고 중단한다.

제공한 `vendor/avl/avl.exe`는 공식 Windows 실행파일이다. 공식 원본 소스도 `vendor/avl/source.tgz`에 보존한다. 배포 정보와 해시는 [외부 구성요소](docs/THIRD_PARTY.md)를 참고한다. 실행파일이 없거나 AVL 실행이 실패하면 대체 공력값을 만들지 않고 오류를 낸다.

```powershell
# 공식 예제를 축소한 입력 형상·질량 파일 생성
.venv/Scripts/python scripts/prepare_reference.py
# 실제 AVL을 여러 프로세스로 실행해 공력 표 생성
.venv/Scripts/python -m dbf_stability.cli build-aero --workers 6
# 단위·물리·실제 AVL 연동 검사
.venv/Scripts/python -m pytest -q
# 트림·고유값·고정 길이 수렴 확인
.venv/Scripts/python scripts/run_validation.py
# 전개·회수·포획과 10/20/40구간·시간 간격 비교
.venv/Scripts/python scripts/run_mission_matrix.py
# 돌풍·중량·견인점·줄 길이 비교
.venv/Scripts/python scripts/run_scenarios.py
# 완성된 전 과정 상태에서 출발하는 급정지·빠른 회수·정렬 불량 등
.venv/Scripts/python scripts/run_edge_cases.py
```

모든 전 과정 계산을 병렬로 실행하면 CPU를 오래 사용할 수 있다. 구간 수와 접촉 횟수에 따라 실행 시간이 달라진다. 각 작업 폴더의 `progress.json`에서 진행 시각과 실행 시간을 확인한다. `workers`를 PC 사양에 맞게 조정한다. 각 작업은 별도 폴더를 사용한다.

```powershell
$env:JUPYTER_PATH = "$PWD/.venv/share/jupyter"
.venv/Scripts/python -m jupyterlab notebooks
# 네 노트북을 처음부터 실행하고 HTML도 저장
.venv/Scripts/python scripts/execute_notebooks.py
```

## Python에서 사용

```python
from dbf_stability import (
    load_case, AeroDatabase, build_aero_database, solve_trim,
    analyze_stability, simulate, run_sweep,
)
case = load_case('examples/reference.yaml')
aero = AeroDatabase('outputs/aero_database.npz')
trim = solve_trim(case, aero, mode='deployed')
modes = analyze_stability(case, aero, trim)
result = simulate(case, aero, trim, phase='deployed', duration=1,
                  output='outputs/my_tow')
```

Windows에서 `build_aero_database`와 `run_sweep`를 Python 스크립트에서 호출할 때는 `if __name__ == '__main__':` 안에 넣는다. 노트북에서는 패키지의 작업 함수를 사용하므로 그대로 호출할 수 있다.

## 입력과 결과

`examples/reference.yaml`은 트림 조종면·추력을 유지하는 기본 사례다. `examples/mission_scheduled.yaml`은 수납/전개 트림 차이를 바탕으로 **명시적인 시간별 조종 입력**을 넣은 사례다. 이 두 사례에는 자동조종이 없다. 후방 유동과 센서 공력·접촉 물성은 가정값이며 `provenance`에 표시한다.

`examples/h1_normal_r3_flow5_controlled.yaml`은 사용자가 요청한 **고도·속도 피드백 제어 시험**이다. 기존 100m·25m/s 무제어 계산의 전개 완료 상태(4.135815755초)에서 시작한다. 같은 실제 flow5 데이터와 기체·센서·케이블·접촉 모델을 사용한다. 바뀐 제어 조건으로 시작하는 새 실험이며, 입력이 같은 체크포인트 재개와 구분한다.

후속 회수 입구 형상은 `examples/h1_recovery_funnel_v2.yaml`이다. 입구를 80×120mm로 확장하고 수직 턱을 경사 통로로 교체했다. 270° 도어, 질량·무게중심·관성 변경과 실제 flow5 트림 확인을 포함한다. 기존 전체 임무의 회수 중단 원인, 형상 파일, 확인된 구간과 재실행 방법은 [회수 입구 재설계 기록](docs/RECOVERY_FUNNEL.md)에 있다. `funnel_recovery_02`는 저장된 69.8초 자세에서 시작한 회수 비교이며, `funnel_mission_01`은 0초부터 새로 계산하는 별도 실행이다. 진행 중인 실행을 완료 결과로 사용하지 않는다.

```powershell
.venv/Scripts/python test_models/H1_reference/run_controlled_mission.py --out test_models/H1_reference/runs/my_control_test --workers 2
```

고도와 상승률로 피치 목표를 정하고, 피치·피치율로 승강타를, 속도 오차로 추력을 조절한다. 1초에 걸쳐 제어를 켜며 승강타 ±7.5°, 추력 0–15N으로 제한한다. 게인과 제한값은 YAML에서 바꾼다. `--until 20`으로 20초까지의 시험을 실행할 수 있다. 저장된 실제 초기 상태와 공력 파일이 없으면 실행을 중단한다.

이 PD/P 제어에는 적분 보상, 횡방향 제어, 센서 오차, 서보 지연, 모터 응답이 없다. 정상상태 오차가 남을 수 있으며 추력값을 배터리 전력으로 해석해서는 안 된다. 구성 참고는 [PX4 고정익 제어 설명](https://docs.px4.io/main/en/flight_stack/controller_diagrams#fixed-wing-position-controller)이며, PX4나 TECS의 구현은 아니다. 제어를 켠 설정의 고유값 분석은 아직 지원하지 않는다. 시간응답으로 평가한다.

결과 폴더에는 초기 상태·공력 해시를 보존한 `branch_source.json`, 무제어와 같은 시간 구간을 비교하는 `comparison.json`·`report.html`, 실제 계산 상태의 `mission/H1_replay.html`을 저장한다. 시계열에는 목표 고도·속도, 피치 목표, 실제 승강타·추력 명령, 제한값 도달 여부도 포함한다. 출력 폴더는 매 실행마다 새 경로를 사용한다.

문·줄 이력은 YAML의 `[시각, 값]` 행 또는 `time_s,value` 열을 가진 CSV로 입력한다. 질량·관성·항력·한계와 출처는 YAML에 둔다. 공력 입력 격자를 바꿨으면 AVL 데이터도 다시 생성해야 한다. 입력이 바뀌어도 기존 데이터를 자동으로 수정하지 않는다.

| 출력 | 내용 |
|---|---|
| `inputs.json` | 해당 실행의 입력 전체 |
| `timeseries.csv` | 자세·고도·장력·토크·접촉/포획 하중·충격량 |
| `states.npz` | 전 시각의 강체·케이블 상태, 3차원 재생과 후속 계산용 |
| `summary.json`, `events.json` | 완료/범위 이탈/수치 실패/지면 도달, 포획 여부와 한계 초과 |
| `history.html`, `replay.html` | 대화형 이력과 기체·센서·줄·문 재생 |
| `avl_runs_*/case_*` | 실제 AVL 입력·명령·출력·오류 원문 |

`completed`는 요청한 시간까지 적분했다는 뜻이다. `captured`는 포획 구속이 작동했다는 뜻이다. 두 값 모두 실물 안전성 판정이 아니다. 허용치가 없는 하중은 합격으로 표시하지 않는다. 수렴 검증 없이 개별 실행의 `numerically_converged`를 참으로 바꾸지 않는다.

## 현재 범위

- 트림은 무풍·대칭·수평 직선비행을 지원한다. 비대칭 트림과 선회 트림은 지원하지 않는다.
- 견인선은 고정 물질 셀의 구속을 바꾸는 분할 근사다. 줄 감김 형상·로터 관성·매듭·걸림·파단은 포함하지 않는다.
- 윈치는 이상적인 지정 속도를 따른다. 토크 한계 초과를 보고하지만 모터가 포화되어 실제 속도가 느려지는 동역학은 아직 없다.
- 기본 사례는 몸체·핀 표본점과 단순 벽으로 접촉을 계산한다. H1의 `collision.enabled: true` 사례는 센서와 기체의 실제 메시, 견인선 캡슐로 접촉을 계산하고 저장된 선택 자세를 STEP으로 별도 검사한다. 부품 응력·파손 해석은 포함하지 않는다.
- AVL 격자 밖의 공력은 외삽하지 않는다. 큰 센서 자세, 후류와 노출 비율, 실물 포획 물성은 추가 검증이 필요하다.
- 실제 기체에 대한 검증과 전 과정의 구동·열·탄성 에너지 수지 폐합은 아직 완료 조건으로 남아 있다.

팀 자료를 받으면 먼저 좌표·CG·관성 기준을 확인하고 AVL 형상과 접촉 치수를 교체한다. 이후 실제 물성으로 수렴 검사를 다시 수행하고 시험값과 비교한다.

AVL·flow5 블록 조합은 [복합 공력표 사용법](docs/사용설명서.md#avl--flow5-복합-공력표)과 [구현 기준](docs/FLOW5_CONNECTION.md#복합-공력표--구현-기준)을 참고하세요.
