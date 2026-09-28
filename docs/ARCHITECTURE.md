# 코드 구조

2026-09-28 구조 정리 이후의 모듈 지도다. 물리 모델의 내용은 [MODEL.md](MODEL.md), 배포 범위는 [REPOSITORY.md](REPOSITORY.md)를 본다.

## 패키지

| 패키지 | 역할 | 의존 방향 |
|---|---|---|
| `src/dbf_stability/` | 공력표 생성·트림·안정성·결합 운동 적분 | 다른 패키지에 의존하지 않음 |
| `src/dbf_studio/` | 로컬 서버, 모델 등록, 해석 작업 실행 | `dbf_stability`를 사용 |
| `ui/` | 배치 편집기 화면(HTML·JS·CSS), 템플릿, 사용자 자료(`ui/data/`) | 서버 API만 호출 |
| `tests/`, `tests/studio/`, `ui/tests/` | 계산부·서버·화면 검사 | |

두 Python 패키지는 `pip install -e .`로 함께 설치된다. `ui/` 폴더 위치는 `dbf_studio/paths.py`가 저장소 루트에서 계산하며, 코드 어디에도 `sys.path`를 손대는 곳이 없다.

## dbf_stability 의존 그래프

```
config ── control
aero_inputs, solver_resources, compute_resources, math3d, door, point_mass, cable_flow, contact_batch, routing
hybrid_rules ─┐
aero_database ─┴─ avl ─┐
                flow5 ─┼─ hybrid ─ aero (backend 분기: build_aero_database)
checkpoint ─ contact_integration
collision ─ analytic_contact, contact_features(numba), contact_bounded(numba)
model ─ collision, control, cable_flow, door, routing, parallel_rhs
analysis ─ model, aero_database, checkpoint, contact_integration, stability_quality
verification, plots, cli ─ analysis
```

- `aero_database.AeroDatabase`는 저장된 공력표 형식과 호환성 검사만 담당한다. AVL·flow5·복합 생성기는 모두 이 클래스를 반환한다.
- `hybrid_rules`는 복합 공력표의 블록 구성 규칙과 AVL 기준점 검사다. `AeroDatabase.assert_compatible`과 `hybrid` 양쪽이 쓰므로 둘보다 아래에 둔다.
- `resume_simulation`은 `analysis`에 있다. `checkpoint`는 파일 형식만 담당하므로 `analysis`를 알지 못한다.
- 함수 안에서 import하는 곳은 세 가지뿐이다. 선택 의존성(`pybullet`, `numba`, `cadquery`)과 `parallel_rhs`의 작업자 프로세스 초기화다.

## dbf_studio

| 모듈 | 내용 |
|---|---|
| `paths` | `ROOT`, `UI`, `DATA`, `TEMPLATES` |
| `server` | `ThreadingHTTPServer` 핸들러. `GET_ROUTES`/`POST_ROUTES` 표로 경로를 찾고, 입력 오류(`ValueError` 등)는 400, 그 밖의 예외는 500과 서버 로그로 나눈다. 정적 파일은 `ui/` 아래만 낸다 |
| `analysis_bridge` | `catalog()`는 등록 모델의 기본값·입력 범위(`fields`)·저장 공력표 목록을 돌려준다. `prepare()`는 배치와 설정을 검증해 `Prepared`(request.json)로 얼린다. 단계별 함수 `_load_request → _check_choices → _apply_numeric_settings → _check_scene_objects → _select_aero_table → _apply_simulation_settings → _map_scene_to_body → _build_schedule → _record_provenance` 순서다 |
| `analysis_jobs` | 작업 폴더와 `job.json`, 워커 프로세스(`python -m dbf_studio.analysis_worker`)와 Windows Job 정리 |
| `analysis_worker` | 별도 프로세스에서 공력표·트림·고유값·시간 적분을 실행하고 결과 파일을 쓴다 |
| `model_registry` | 등록 모델 저장·조회·CAD 서명 대조, `register_definition()` |
| `aircraft_definition` | 기체 정의(부품 역할·CG·관성·공력 면)의 검증과 AVL/flow5 입력 생성. 다른 studio 모듈을 import하지 않는 잎 모듈 |
| `mechanisms`, `mission_sequence`, `assembly` | 문·가이드 형상, 임무 단계 표, 조립 부품 검증 |

숫자 입력의 허용 범위는 `analysis_bridge.FIELDS`·`SETTING_RANGES` 한 곳에 있고, 화면은 `catalog().fields`를 받아 `<input min max step>`으로 적용한다.

## ui (JavaScript)

| 모듈 | 내용 |
|---|---|
| `app.js` | 편집기 상태(배치된 모델, 선택, 되돌리기)와 화면 연결 |
| `scene.js` | Three.js 씬·카메라·렌더러·기즈모·격자·시점 맞춤(`fitView`) |
| `flow-indicator.js` | 공기 흐름 화살표와 라벨 |
| `wire.js` | 윈치–질점 줄 미리보기와 교차 표시 |
| `model-io.js` | STL·GLB·STEP 읽기, 자산 검증 |
| `project-store.js` | IndexedDB 초안, 서버 사본, 파일 내려받기 |
| `analysis-ui.js`, `analysis-settings.js`, `analysis-options.js`, `analysis-scenarios.js`, `aero-ranges.js`, `controller-settings.js`, `result-selection.js`, `analysis-quality.js` | 해석 화면 |
| `aircraft-definition-ui.js`, `aircraft-model.js`, `aircraft-assembly.js`, `point-mass.js`, `coordinates.js` | 기체 정의·좌표·조립 |
| `analysis-replay.js` | 결과 3D 재생 페이지 |

## 형식과 검사

```powershell
.venv\Scripts\python.exe -m ruff format src tests scripts
.venv\Scripts\python.exe -m ruff check --select F src tests scripts
npm.cmd --prefix ui run format
npm.cmd --prefix ui run check
npm.cmd --prefix ui test
.venv\Scripts\python.exe -m pytest -q
```

줄 길이는 120자, 한 줄에 한 문장이다. `tests/studio/`는 서버·등록·`prepare` 검사이고 합성 기체만 쓴다. 서버 라우팅은 `tests/studio/test_server_routes.py`가 실제 HTTP로 확인한다.
