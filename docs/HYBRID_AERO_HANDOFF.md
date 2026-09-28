# 구현 인계 계획 · ① H1 내장 제거 → ② AVL + flow5 복합 공력표

작성 2026-09-28, 개정 2026-09-28. 이 문서만 읽고 작업할 수 있도록 현재 구조, 설계, 작업 순서, 완료 조건을 적었다. 결정할 항목은 마지막 절에 기본값과 함께 두었다. 기본값으로 진행하고, 바꾸면 이 문서도 고친다.

**개정 이유.** H1은 프로그램 안에 기본 모델, 기본 공력표, 윈치 형상, 테스트 기준으로 박혀 있다. 이 상태에서는 기능과 테스트가 H1에서 돌아가는지에 맞춰진다. "H1에서 통과하면 된다"가 목표를 대신하는 보상 해킹이 생기기 쉽다. 그래서 **Part A에서 H1을 프로그램에서 분리하고**, Part B의 복합 공력표는 **어떤 기체에도 H1 없이** 동작하도록 설계한다.

## 붙여 넣을 작업 지시

> `docs/HYBRID_AERO_HANDOFF.md`를 끝까지 읽고 Part A → Part B 순서로 구현하라. 각 단계의 완료 조건과 테스트를 통과한 뒤 다음 단계로 간다. 프로그램 코드(`ui/`, `src/dbf_stability/`)에 특정 기체 이름·파일·수치를 새로 넣지 않는다. H1 파일을 테스트 기대값의 근거로 쓰지 않는다. 실패를 다른 값으로 대체하지 않는다. 오류로 멈춘다. UI 문구는 한국어로 쓴다. 수정 전 파일은 `ui/design/<작업명>-<날짜>/before/`에 복사하고, 끝나면 같은 폴더에 `IMPLEMENTATION.md`를 남긴다.

---

# Part A · H1 내장 제거 (먼저 한다)

## A1. 목표와 경계

- **남기는 것 (모델링 자료):** `test_models/H1_reference/`의 CAD, 메시, AVL·flow5 입력, 실행 기록, 보고서와 `examples/h1_*.yaml`. 노트북과 CLI에서 파일 경로를 명시해 쓰는 것은 그대로 둔다. H1을 UI에서 해석하고 싶으면 다른 기체와 **같은 절차**(기체 불러오기 → 기체 정의 또는 AVL 파일 등록)를 거친다.
- **없애는 것 (프로그램 내장):** 코드 안의 모델 ID `'h1'`, 내장 공력표 경로, 내장 형상(`/api/reference`), H1 전용 분기, H1 값을 기본 템플릿으로 내주는 기능, H1에 기대는 UI 테스트.
- **완료 판정 기준:** 아래 검색 결과가 0건이어야 한다. 대상은 `ui/`의 `.py`, `.js`, `.html`이다. `ui/design/`, `ui/data/`, `ui/output/`은 기록이므로 제외한다. 2026-09-28 기준으로 13개 파일에서 66건이 나온다. `rg`는 설치되어 있지 않으므로 PowerShell을 쓴다.

```powershell
Get-ChildItem ui -Recurse -File -Include *.py,*.js,*.html | Where-Object { $_.FullName -notmatch '\\(design|data|output|node_modules)\\' } | Select-String -CaseSensitive -Pattern "'h1'|`"h1`"|\bH1\b|h1_recovery|H1_reference|builtin|/api/reference|getReference|DATABASES"
```

## A2. 제거 목록 (현재 코드 근거)

| # | 위치 | 현재 | 바꿀 내용 |
|---|---|---|---|
| 1 | `ui/model_registry.py:13, 31-36` | `CASE`, `load('h1')`가 H1 YAML을 읽어 `builtin=True` 모델을 만든다 | `CASE` 삭제. `load(model_id)`는 등록 폴더의 32자리 ID만 받는다. `builtin` 키를 없앤다 |
| 2 | `ui/model_registry.py:47-51` | `list_models()` 첫 행이 항상 H1 | 등록 모델만 반환한다. 없으면 빈 목록 |
| 3 | `ui/model_registry.py:54-82` | `match_aircraft(project, reference, id)`에 H1 분기(69, 76행) | `reference` 인자를 없앤다. 모든 모델을 저장된 `signatures`·`definition_sha256`으로만 대조한다 |
| 4 | `ui/analysis_bridge.py:22-25` | `CASE`, `DATABASES`(H1 AVL·flow5 공력표 경로) | 삭제. 해석기 목록은 `BACKENDS=('avl','flow5')` 상수로 둔다 |
| 5 | `ui/analysis_bridge.py:61-94` `catalog(model_id='h1')` | 기본 ID가 H1. `aero_job='builtin'`, `databases[*].available`이 H1 전용 | 기본값 없이 ID를 필수로 받는다. `aero_job` 기본값은 등록 모델의 최근 저장 표, 없으면 `'new'`. `databases`에는 실행파일 존재 여부만 둔다 |
| 6 | `ui/analysis_bridge.py:107-205` `prepare(project, settings, reference)` | 109행 기본 `'h1'`. 176-181행 H1 형상 비교. 184·193·197행 `'builtin'` 분기. 189·197행 flow5 25 m/s 가정 | `reference` 인자 삭제. 형상은 `profile['signatures']`로만 비교한다. `aero_job`은 `'new'` 또는 저장 실행 ID만 받는다. flow5 속도는 **저장 표의 요청 속도**와 비교한다 |
| 7 | `ui/analysis_bridge.py:340-389` 메시 준비 | `profile['builtin']` 분기: H1 STL 폴더 복사, 원뿔 멈춤장치 CAD, 문 170° 재구성 | 등록 모델용 일반 경로(360-375행)만 남긴다 |
| 8 | `ui/server.py:38-66` `build_reference()`, `149` `/api/reference`, `235` `server.reference` | H1 형상을 서버 시작 때 만든다 | 모두 삭제. 호출하던 곳(`/api/models/match`, `prepare`)에서 인자를 뺀다 |
| 9 | `ui/server.py:154-157` | `/api/models/template`, `/api/analysis/catalog`의 `model` 기본값 `'h1'` | `model` 없으면 400 오류. 템플릿은 A3의 중립 템플릿을 반환한다 |
| 10 | `ui/server.py:21`, `ui/analysis_worker.py:56` | CPU 친화도 코드를 `test_models/H1_reference/compute_resources.py`에서 import | `compute_resources.py`를 `src/dbf_stability/` 또는 `ui/`로 옮긴다. 기존 `solver_resources.py`와 겹치면 합친다. H1 폴더의 사본은 모델링 스크립트용으로 남겨도 된다 |
| 11 | `ui/app.js:269-270` `getReference()`, `addWinch()` | 윈치 형상과 기본 위치를 `/api/reference`의 H1 윈치에서 가져온다. `origin==='reference'` 분기 | 코드로 만든 **일반 윈치 표시 형상**을 쓴다(A3). 위치는 중심선 규칙(`onCenterline`) 하나로 정한다. `reference` 변수와 분기를 삭제한다 |
| 12 | `ui/aircraft-model.js` `reorientAsset` | `origin==='reference'`를 `file_origin`으로 바꾸는 H1 호환 코드 | 삭제(원점 값은 그대로 유지) |
| 13 | `ui/analysis-ui.js:102, 165` | 모델 ID 기본값 `'h1'` | 등록 모델 목록의 첫 항목. 목록이 비면 A3의 빈 상태를 표시한다 |
| 14 | `ui/analysis-ui.js:111` `aero-source` | `'builtin'` 문구와 "저장 표: 25 m/s" | 삭제. 저장 표는 해당 실행의 해석기·속도·격자를 표시한다 |
| 15 | `ui/analysis-options.js:4` | H1이면 "H1 검증용 저장 공력표" 행 추가 | 삭제 |
| 16 | `ui/analysis-settings.js` "선택 모델 설정 내려받기" | 선택 모델(기본 H1)의 설정을 예시로 준다 | 중립 템플릿을 내려받는다(A3). 문구를 "빈 설정 템플릿 내려받기"로 바꾼다 |
| 17 | `ui/index.html`, `ui/app.js` 문구 | "H1 모델을 불러오지 못했습니다" 등 | 해당 기능이 없어지므로 삭제 |
| 18 | `ui/tests/*` | `build_reference()`, `DATABASES['avl']`, 모델 `'h1'`, N3 실데이터에 의존 | A4 참고 |
| 19 | `src/dbf_stability/collision.py:356, 415` | 매니페스트가 없을 때 `rear_door_100deg` 이름으로 문을 찾는 H1 기본값 | 매니페스트를 필수로 하고 없으면 오류. 이 모듈을 쓰는 기존 H1 노트북·CLI 예제가 깨지면, 그 예제 YAML에 매니페스트를 명시해 고친다(모델링 쪽 수정) |

## A3. 새로 만들 것

- **일반 윈치 표시 형상:** 드럼 원통과 받침판 두 부품. 부품 이름 `winch_drum`, `winch_base`(`analysis_bridge.py:371`이 `winch_drum`을 스풀로 인식한다). 그룹 원점이 줄 출구다. 치수는 "표시용"이라고 명시하고 해석 입력에 쓰지 않는다.
- **중립 설정 템플릿:** `ui/templates/model_physics_template.yaml`. 모든 물리값은 `null`, 각 항목에 단위·좌표계·출처 칸을 둔다. 스키마 검증(`dbf_stability.config.validate`)은 **값을 채운 뒤** 통과해야 한다. 빈 템플릿이 그대로 등록되면 안 된다. 테스트로 확인한다.
- **등록 모델이 없을 때 해석 화면:** "등록된 해석 모델이 없습니다. 배치 화면에서 기체를 불러온 뒤 기체 정의로 등록하세요." + 기체 정의 바로가기. 입력 확인·실행 버튼은 비활성화한다.
- **기존 프로젝트·결과 호환:**
  - `model_id: 'h1'`이 든 저장 프로젝트·자동 저장을 열면 오류로 멈추지 않는다. 배치(기체·윈치·질점)는 복원하고, 해석 조건만 "내장 H1 모델은 제거되었습니다. 이 기체를 등록한 뒤 다시 선택하세요."로 안내한다.
  - 과거 `ui/data/analysis/*` 결과와 보고서는 읽기 전용으로 계속 열린다. 해당 실행의 공력표를 새 해석의 "저장 공력표"로 고를 수는 없다(등록 모델이 없으므로).

## A4. 테스트 정리 원칙

- UI 테스트는 **테스트 안에서 만든 합성 기체**를 쓴다. 예: 직사각 주익·수평·수직 꼬리 상자 메시와 최소 AVL 형상(`ui/tests/fixtures/`). 기대값은 입력에서 해석적으로 나오는 값(좌표 변환, 서명 일치·불일치, 거부 조건)만 쓴다.
- 실제 해석기를 돌리는 테스트는 `@avl`/`@slow` 표식으로 분리한다. 판정은 "실행이 끝나고 형식·단위·해시가 맞다"로 한다. **특정 기체의 수치 결과를 정답으로 고정하지 않는다.**
- 지금 H1·N3 실데이터에 기대는 테스트는 목적을 보고 합성 기체로 옮긴다. 옮길 수 없는 것(실제 H1 결과의 회귀 확인)은 `test_models/H1_reference/`의 모델링 검증 스크립트로 옮기고 UI 테스트 목록에서 뺀다.
  - `test_flight_recovery.py`, `test_analysis_bridge.py`, `test_point_mass.py`, `test_model_connection.py`, `test_registration.py`, `test_aircraft_definition.py`
- 새 회귀 테스트:
  - (a) 등록 모델 0개에서 catalog·prepare가 명확한 오류를 낸다.
  - (b) 형상이 1 µm 다른 기체는 어떤 모델과도 연결되지 않는다.
  - (c) `model_id:'h1'` 프로젝트를 열면 배치는 복원되고 해석 조건은 안내된다.
  - (d) 빈 템플릿은 등록이 거부된다.
  - (e) 일반 윈치를 추가하면 CAD Y가 중심선에 놓인다.

## A5. 완료 조건

- A1의 검색 결과 0건.
- `npm.cmd run check`, `npm.cmd test`, `python -m unittest discover -s ui/tests`, `pytest -q` 통과.
- 브라우저: 등록 모델이 없는 새 브라우저에서 기체 STEP 불러오기 → 윈치·질점 → 기체 정의 등록 → 새 공력 계산 → 트림·고유값까지 끝난다(합성 기체 또는 팀 기체). H1 파일을 불러와도 같은 절차를 거쳐야만 해석이 된다.
- `ui/README.md`, `docs/사용설명서.md`, 루트 `README.md`에서 "내장 H1", "H1 예제", "H1 검증용 저장 공력표" 설명을 없앤다. H1은 `test_models/H1_reference/` 안내로만 남긴다.

---

# Part B · AVL + flow5 복합 공력표 (Part A 후)

## B1. 왜 필요한가

- 지금은 해석마다 공력 해석기를 **하나만** 고른다(`settings.backend` = `avl` | `flow5`). 공력표 한 개가 정적 계수, 회전율 미계수, 조종 미계수를 모두 담는다.
- flow5 7.57은 회전율 미계수 18개 중 종·횡 블록만 출력한다. 나머지 교차항은 0으로 두고 `rate_derivative_closure: classical_longitudinal_lateral`로 명시해야 실행된다(`src/dbf_stability/flow5.py:223-234, 267-268`). 큰 옆미끄럼이나 비대칭 조종에서 이 가정은 검증되지 않았다.
- AVL은 18개 회전율 미계수를 모두 준다. 조종면은 선형화된 법선 회전으로 처리한다. flow5는 조종면을 실제 기하 회전으로 처리한다.
- 목표: **정적 계수와 조종 미계수는 flow5, 회전율 미계수는 AVL**에서 가져와 한 공력표로 만든다. 블록별 출처는 설정으로 바꿀 수 있게 한다. 두 해석기를 섞는 것 자체가 모델링 가정이다. 출처와 한계를 메타데이터·UI에 표시한다. 두 해석기의 불일치를 정확도로 해석하지 않는다.

## B2. 현재 구조 (코드 근거)

| 항목 | 위치 | 내용 |
|---|---|---|
| 공력표 형식 | `src/dbf_stability/avl.py:156`, `flow5.py:308` | npz: `alpha, beta, elevator`(1차원, 증가, 도 단위), `coeff[...,6]`, `rates[...,6,3]`, `controls[...,6,2]`, `refs=[S,c,b]`, `metadata`(JSON) |
| 계수 순서·축 | 메타 `axes='FRD body'` | `[CX,CY,CZ,Cl,Cm,Cn]` 기체축. 모멘트 무차원 길이 `[b,c,b]`(`model.py:204`) |
| 회전율 단위 | 메타 `rate_unit='pb/2V,qc/2V,rb/2V'` | `rates[...,i,j]` = ∂C_i/∂(p̂,q̂,r̂)_j |
| 조종 미계수 | AVL `avl.py:146-148`, flow5 `flow5.py:293-296` | 두 쪽 모두 **도(°)당**. 열 `[aileron, rudder]` |
| 로더·검사 | `avl.py:160-194` `AeroDatabase` | 형상·축·단위 검사. `assert_compatible`는 `backend in metadata['solver'].lower()`, 형상 해시, 조종면 인덱스를 확인. 격자 밖은 `bounds_error=True`로 거부 |
| 실행 중 모멘트 기준 | `model.py:205-207` | **정적** 모멘트만 표의 `moment_reference_frd_m`에서 CG로 옮긴다. **회전율·조종 미계수는 옮기지 않는다.** 한 표의 모든 블록은 같은 기준점 값이어야 한다 |
| 빌드 진입점 | `avl.py:115-120` | `config['aero']['backend']`로 빌더 선택 |
| flow5 제한 | `flow5.py:266-276` | VLM2만, \|δe\| ≤ 10°, 속도는 `flight.speed_m_s` 한 값. 메타에 속도 값이 없다 → 복합 표는 `flow5_speed_m_s`를 기록한다 |
| AVL 기준점 | `avl.py:60-81` | `.avl` 파일의 `Xref Yref Zref`를 그대로 쓴다. 메타 `moment_reference_frd_m`은 설정값(`aero.moment_reference_frd_m`)을 적을 뿐, 파일과 일치하는지 확인하지 않는다 → B3.2-4의 검사를 추가한다 |
| UI | `ui/analysis_bridge.py` `prepare`·`catalog`, `ui/analysis_worker.py:62-65`, `ui/analysis-ui.js` 해석기 선택, `ui/analysis-options.js` `tableOptions` | Part A 이후 기준으로 연결한다 |

> 과거 기록 참고: H1의 저장 AVL 표와 flow5 표는 격자(α −6…12 / −4…8, δe ±20 / ±8)와 모멘트 기준점(6.6 mm 차이)이 달랐다. 서로 다른 시점·설정의 표를 섞으면 이런 불일치가 흔하다는 예로만 적는다. **Part B는 H1 표를 쓰지 않는다.**

## B3. 설계

### B3.1 조합 규칙

```yaml
aero:
  backend: hybrid
  hybrid:
    coeff: flow5      # 정적 계수 [CX..Cn]
    controls: flow5   # 에일러론·러더 미계수
    rates: avl        # 회전율 미계수 18개
```

- 값은 `avl` | `flow5`. 세 블록이 모두 같으면 거부한다(`단일 해석기를 선택하세요`).
- `rates: flow5`이면 기존처럼 `rate_derivative_closure` 확인을 요구한다.
- 결과 표의 `moment_reference_frd_m`, `refs`는 `coeff` 출처 값을 쓴다.

### B3.2 불변 조건 (어기면 오류)

1. 두 원본의 `geometry_sha256`, `input_files_sha256`이 같다(같은 등록 모델의 같은 AVL 형상에서 만든 것).
2. `refs` 상대 차이 ≤ 1e-5.
3. 격자: 두 해석기에 **같은 격자**를 넣어 계산하고 `np.array_equal`로 확인한다. 보간 조합은 B5(선택)에서만 허용한다.
4. 모멘트·회전 기준점: 두 해석기 모두 등록 모델의 CG를 쓴다. AVL은 실행 폴더에 복사한 `plane.avl`의 `Xref Yref Zref`를 읽어 CG와 ≤ 1e-9 m로 일치하는지 확인한다. 다르면 오류: "AVL 형상의 Xref·Yref·Zref가 CG와 다릅니다. 기체 정의에서 다시 등록하세요." 기체 정의로 등록한 모델은 AVL 파일을 생성할 때 CG를 기준점으로 쓰는지 확인하고, 아니면 생성 코드를 고친다.
5. 결과는 기존 `AeroDatabase`로 그대로 읽혀야 한다. 키, 모양, 단위, 메타 키는 기존 표와 같다.
6. flow5가 참여하면 `flow5_speed_m_s`를 기록한다. 해석 속도가 다르면 거부한다.

### B3.3 메타데이터

```json
{
  "solver": "hybrid: flow5 v7.57 (coeff, controls) + AVL 3.52 (rates)",
  "backend": "hybrid",
  "axes": "FRD body", "angle_unit": "degrees", "rate_unit": "pb/2V,qc/2V,rb/2V",
  "moment_reference_frd_m": [..], "geometry_sha256": "..", "input_files_sha256": "..",
  "composition": {"coeff": "flow5", "controls": "flow5", "rates": "avl"},
  "sources": {"avl": {"raw_directory": "..", "npz_sha256": "..", "solver": "..", "moment_reference_frd_m": [..]},
              "flow5": {"raw_directory": "..", "npz_sha256": "..", "solver": "..", "moment_reference_frd_m": [..], "speed_m_s": ..}},
  "flow5_speed_m_s": ..,
  "discarded": ["flow5 rates"],
  "consistency": {"CL_alpha_per_deg": {"avl": .., "flow5": .., "relative_difference": ..}, "Cm_alpha_per_deg": {..}, "CY_beta_per_deg": {..}, "Cl_beta_per_deg": {..}, "Cn_beta_per_deg": {..}},
  "limitations": ["정적 계수·조종 미계수와 회전율 미계수는 서로 다른 해석기의 결과입니다.", "..."]
}
```

- `solver`에 `hybrid`를 넣는다. 그래야 기존 `assert_compatible` 검사를 그대로 통과한다. `control_indices`는 controls 출처가 AVL일 때만 넣는다.
- `consistency`: 격자 중앙(α·β·δe가 0에 가장 가까운 점)에서 중앙차분으로 구한다. **경고만 하고 차단하지 않는다.** 기준은 상대 20%이며 결정 항목 B4에서 바꿀 수 있다. 결과 카드와 `report.html`에 표로 표시한다.

## B4. 작업 순서

### B-1단계 · 순수 조합 함수 (해석기 실행 없음)

- `src/dbf_stability/hybrid.py`: `compose_tables(sources, composition, output) -> AeroDatabase`, `consistency_report(avl_npz, flow5_npz)`.
- `tests/test_hybrid_compose.py`는 **합성 npz**를 테스트 안에서 만든다. 격자 2×2×2, 해석적으로 정한 값을 쓴다. 다음을 확인한다.
  - 블록별 값이 지정 출처와 `np.array_equal`로 같다.
  - 해시, refs, 격자, 기준점 불일치를 각각 거부한다.
  - 세 블록 출처가 모두 같으면 거부한다.
  - 결과가 `AeroDatabase`로 읽히고 `backend='hybrid'`로 `assert_compatible`을 통과한다.
  - `consistency_report`가 합성 기울기를 정확히 복원한다.
- 완료 조건: 해당 테스트와 기존 전체 테스트 통과.

### B-2단계 · 새 계산 빌더와 UI 연결

- `hybrid.build_hybrid_database(config, output, workers)`:
  1. `output.parent/'avl'`, `output.parent/'flow5'`에서 두 해석기를 **같은 격자·같은 기준점**으로 실행한다. 기존 `build_aero_database`를 `backend`만 바꾼 설정 사본으로 부른다. 작업자 수는 나눠 쓴다(12 CPU 상한 유지).
  2. flow5 하위 실행은 회전율을 쓰지 않는다. 설정 사본에만 `rate_derivative_closure`를 넣고, 메타 `discarded`에 기록한다.
  3. B3.2-4 기준점 검사 → `compose_tables`.
- `avl.build_aero_database`: `backend=='hybrid'`이면 `hybrid.build_hybrid_database`로 보낸다.
- `ui/analysis_bridge.py`:
  - `BACKENDS`에 `hybrid`를 추가한다.
  - flow5 전용 검사(대칭 수평면, 기울어진 수직면, 속도)를 flow5가 참여하는 조합에도 적용한다.
  - 회전율 근사 확인은 `rates: flow5`일 때만 요구한다.
  - 새 계산 시 두 실행파일이 모두 있어야 한다. hybrid에서는 \|δe\| > 10 격자를 거부한다.
  - 저장된 hybrid 실행은 같은 등록 모델의 `tables`에 올린다.
- UI:
  - 해석기 선택에 "AVL + flow5 복합"을 추가한다.
  - 공력표 상세에 블록별 출처 선택 3개를 둔다(기본 flow5 / flow5 / AVL). 설정 키는 `aero_hybrid`이고 `catalog.defaults`에 넣어 저장·복원되게 한다.
  - `aero-source` 문구: "정적 계수·조종: flow5 (VLM2, 동체 제외, 계산 속도 V) · 회전율: AVL · 서로 다른 해석기 조합".
  - hybrid에서는 승강타 슬라이더 범위를 ±10°로 제한한다.
  - 결과 카드의 `analysis-quality`에 `consistency` 경고를 표시한다.
  - 기체 정의의 "flow5 종·횡 분리 근사 동의"는 `rates: avl`이면 요구하지 않는다(서버 규칙과 일치).
- 테스트:
  - UI 브리지 테스트는 Part A의 **합성 등록 모델**로 hybrid 설정을 검증하고, 속도·δe·실행파일 누락 거부를 확인한다.
  - JS `tableOptions`에 hybrid 필터 테스트를 추가한다.
  - `@avl @slow` 실제 실행 1건: 합성 최소 AVL 형상, α{0,4}·β{0,4}·δe{0,4}, `workers=2`. 판정은 실행 완료와 메타·해시 형식뿐이다. 수치를 정답으로 고정하지 않는다.
- 완료 조건: 전체 테스트 통과. 브라우저에서 합성 또는 팀 기체로 hybrid 선택 → 입력 확인 → 작은 격자 실제 실행 → 결과 카드에 출처·일치도 표시.

### B-3단계 · 문서

- `docs/FLOW5_CONNECTION.md`, `docs/MODEL.md`에 "복합 공력표" 절(조합 규칙, 불변 조건, 한계)을 쓴다.
- `ui/README.md`, `docs/사용설명서.md`에 해석기 선택지를 추가하고, 루트 `README.md`에는 한 줄 링크를 둔다.

## B5. (선택, 결정 B3 후) 서로 다른 격자·기준점의 저장 표 조합

B-2단계로 필요가 해결되면 하지 않는다. 할 경우:

- 목표 격자는 `coeff` 출처 격자다. 다른 블록은 `RegularGridInterpolator(linear, bounds_error=True)`로 계산한다. 외삽은 금지한다.
- 기준점 이동:
  - 모멘트 팔 항 `C_M,B = C_M,A − (d × C_F)/[b,c,b]`, `d = r_B − r_A`. 모든 블록에 적용한다.
  - 회전율 블록에는 회전 중심 이동에 따른 α·β 변화 항을 더한다.
  - **부호는 합성 최소 AVL 형상을 Xref 두 곳에서 직접 계산한 결과로 확정한다.** 18개 성분이 상대 2% 또는 절대 1e-3 이내여야 한다.
- 메타 `interpolated`, `transfers`에 기록한다.

---

## 공통 규칙

- 기존 `backend: avl`·`flow5` 단독 계산의 수치, 메타, 오류 문구는 Part A의 H1 관련 부분 말고는 바꾸지 않는다.
- 외삽하지 않는다. 대체값을 만들지 않는다. 불일치는 한국어 사유와 함께 오류로 멈춘다.
- 원본 표·실행 기록은 읽기만 한다. 결과 폴더는 새로 만든다(`FileExistsError` 관례).
- `ui/*.js`는 기존 촘촘한 한 줄 스타일로 쓴다. 색은 `ui/theme.css` 토큰만 쓴다.
- Python 서버 코드를 바꾸면 재시작해야 반영된다. `GET /api/analysis/jobs`로 진행 중 해석이 없는지 확인하고, 서버 프로세스를 멈춘 뒤 `.venv\Scripts\python.exe ui\launch.py --no-browser`로 다시 띄운다.
- 사용자 데이터(`ui/data/projects`, `ui/data/analysis`의 기존 실행, `outputs/`, `test_models/`)를 지우거나 덮어쓰지 않는다.

## 결정할 것 (기본값으로 진행)

| # | 질문 | 기본값 |
|---|---|---|
| A1 | H1 관련 UI 테스트 중 합성 기체로 옮길 수 없는 것 | `test_models/H1_reference/`의 모델링 검증 스크립트로 옮기고 UI 테스트에서 뺀다 |
| A2 | `collision.py`의 H1 이름 기본값(표 A2 #19) | 매니페스트 필수로 바꾸고, 깨지는 H1 예제 YAML에 매니페스트를 명시한다 |
| A3 | `model_id:'h1'`이 든 옛 프로젝트 | 배치는 복원, 해석 조건만 안내(오류로 멈추지 않음) |
| B1 | 복합 기본 조합 | 정적 계수 flow5, 조종 미계수 flow5, 회전율 AVL |
| B2 | 두 해석기 불일치 | 경고만(상대 20% 초과 시 표시) |
| B3 | B5(저장 표 보간 조합) 진행 | 하지 않음. B-2단계 결과를 보고 다시 판단 |

## 검증 명령

```powershell
# 프로젝트 폴더
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe -m pytest -q -m "avl and slow" tests/test_hybrid_compose.py
.venv\Scripts\python.exe -m unittest discover -s ui/tests
Get-ChildItem ui -Recurse -File -Include *.py,*.js,*.html | Where-Object { $_.FullName -notmatch '\\(design|data|output|node_modules)\\' } | Select-String -CaseSensitive -Pattern "'h1'|`"h1`"|\bH1\b|h1_recovery|H1_reference|builtin|/api/reference|getReference|DATABASES"
```

```powershell
# ui 폴더
npm.cmd run check
npm.cmd test
```


## 구현 중 확인한 정정 사항

- 현재 환경에는 `rg`가 설치되어 있다.
- `model.py`는 정적·회전율·조종 계수를 합친 뒤 모멘트 팔을 적용한다. 정적 계수에만 적용된다는 B2 설명은 현재 코드와 다르다. 복합 표의 회전 중심 일치는 계획대로 양쪽 CG 기준 계산으로 보장한다.
- `/api/models/template`은 모델이 없는 상태에서도 쓰는 중립 템플릿이므로 모델 ID 없이 제공한다. `/api/analysis/catalog`에는 모델 ID가 반드시 필요하다.
- 2점 격자는 내부점이 없으므로 중앙차분을 할 수 없다. 경계에서는 한쪽 차분을 쓰고 메타데이터에 구분해서 남긴다.
