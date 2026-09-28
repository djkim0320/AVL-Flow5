# H1-A 임시 센서 전개·회수 시험 모델

R3 전체 결과는 [새 기체 재해석 보고서](NORMAL_AIRCRAFT_RESULTS.md)와 [결과 화면](runs/normal_r3_01/report.html)에 정리했다. 회수 계산의 포획 실패와 미확인 수렴 조건도 함께 기록한다.

**새 형상: R3 고익기.** [3D 모델](geometry_variants/normal_r3/model_overview.html)과 [조립 STEP](geometry_variants/normal_r3/operating_cad/H1_door_140_assembly.step)을 제공한다. 동체·날개·꼬리·착륙장치·프로펠러를 새로 만들고 내부 회수장치를 배치했다. 입력은 `../../examples/h1_normal_r3.yaml`, 전용 공력표는 `runs/normal_r3_01/aerodynamics/aero_database.npz`다. [실제 AVL 공력](runs/normal_r3_01/aerodynamics/aero_report.html)과 [기체 단독 교란 응답](runs/normal_r3_01/airframe_response/response.html)을 확인할 수 있다. 아래 내용은 이전 H1의 변경 이력이다.

복사해 둔 He 51 계열 H1의 **A안**을 안정성 툴에 연결한 임시 모델이다. A/B의 원본 메시를 각각 확인했으며 A안 주익 면적은 0.55㎡, B안은 0.60㎡다. 이번 실제 AVL 계산과 운동 계산은 A안만 사용한다. 기체 본체, 센서와 견인선의 질량을 따로 계산한다.

**[출구 높이 변경 결과](OPENING_CHANGE_RESULTS.md):** 기본 실행은 폭 56mm × 높이 70mm, 도어 140° 입력을 사용한다. 상단을 14mm 올리고 내부 천장·측벽·프레임·문판·동체 절개부를 함께 수정했다. 기존에 걸렸던 자세의 상단 여유는 6.738mm다. 실제 회수 결과는 [새 궤적 비교](runs/opening_01/comparison.html)에 별도로 표시한다. 80mm 안은 도어–꼬리날개 간섭으로 제외했다. [70mm STEP](geometry_variants/opening_70/operating_cad/H1_door_140_assembly.step)을 제공하며 제조 승인안은 아니다.

**[이전 도어·힌지 수정 결과](DOOR_CHANGE_RESULTS.md):** 56mm 출구에서 도어를 140°·160°로 넓힌 시험이다. 140°에서도 짧은 도어 접촉이 남았고, 6.09초에 상부 핀–출구 상단의 간격 한계로 중단됐다. 160°는 출구 접근 중 수치 오류로 계산이 끝났다. 두 경우 모두 미포획이며 최적 설계로 판정하지 않았다. [이전 궤적 비교](runs/door_angle_01/comparison.html)를 보존했다.

**[이전 100° 배치의 관통 수정 결과](CONTACT_FIX_RESULTS.md):** 전체 센서·와이어의 접촉 반력과 관통 방지 검사를 추가했다. 처음부터 다시 계산한 두 궤적의 저장 상태 각 638개에서 CAD 겹침·검사 오류가 모두 0건이었다. 다만 두 계산 모두 약 6.17초에 와이어–출구 상단의 거리 한계로 중단됐고 포획하지 못했다. 정상 회수 7초 시연은 미완료다.

**[수정 전 실행의 오류 정정](RESULTS.md):** 기존 7초 궤적은 실제 CAD 관통으로 무효다. 그 장력·접촉력 등을 설계 결과로 사용하면 안 된다. CAD의 별도 지정 직선 경로 여유 4.02mm도 실제 운동 궤적의 검증이 아니다.

## 열어 볼 파일

| 파일 | 내용 |
|---|---|
| `overview.png` | 기체와 후방 통로 배치 |
| `cad/H1_A_temporary_assembly.step` | 실제 CadQuery/OpenCASCADE로 생성한 부품별 CAD 조립체, mm, 기체 CG 기준 FRD |
| `cad/*.step` | 센서 몸체·핀, 문·가이드·출구·포획 패드·윈치와 근사 기체 부품 |
| `meshes/H1_A_temporary_assembly.glb` | 위 임시 모델의 3D 표시용 조립체 |
| `meshes/*_FRD_m.stl` | 부품별 메시, 정점 단위 m, CG 기준 FRD |
| `meshes/H1_A_original_lifting_surfaces.glb` | 원본 H1-A 양력면·조종면 메시를 CG 기준으로 변환한 비교용 파일 |
| `meshes/H1_A_original_context_CG.glb` | 원본 H1-A 전체 형상, 비교용이며 새 통로는 없음 |
| `geometry/h1_a.avl`, `h1_*.dat` | H1 양력면과 익형 입력 |
| `geometry/h1_a.mass`, `h1_stowed.mass` | 기체 단독 / 센서·줄을 추가한 수납 질량 파일 |
| `../../examples/h1_reference.yaml` | 기존 로더로 바로 읽는 새 사례 |
| `schedules/door.csv`, `length.csv` | 문 각도·풀린 줄 길이 이력 |
| `dimensions_mass_provenance.csv`, `mass_budget.csv` | 치수·질량·CG·관성·출처 및 가정 구분 |
| `clearance_path.csv`, `door_sweep.csv` | 지정한 이동 경로와 문 회전의 CAD 거리 검사 |
| `aerodynamics/aero_database.npz` | H1 입력으로 새로 계산한 실제 AVL 공력표 |
| [공력 해석기 연결 확인](AERO_CONNECTION.md) | 실제 AVL 재실행·보간값 비교와 센서·후류 가정의 범위 |
| `runs/` | 실행별 트림, 원시 상태, CSV, 하중 그래프, 오류 기록 |

STEP은 파라메트릭 스케치 이력 대신 B-rep 솔리드를 담는다. 치수 편집과 재생성에는 아래 Python 스크립트를 사용한다. 원본 H1 `.blend`와 `.stp`는 변경하지 않았다.

## 원본에서 가져온 부분과 새로 만든 부분

원본은 `D:/DBF/모델링/H1_비행기_모델`이다. `extract_h1.py`가 GLB의 **DBF A | H1**, **DBF B | H1** 장면만 읽는다. 같은 GLB에 담긴 다른 장면은 선택하지 않는다. 파일명만 보고 형상을 추정하지 않았으며, 선택한 장면·부품별 경계·해시는 `source_audit/`에 남겼다.

주익 폭 1.80m, A안 메시 단면 시위 약 0.305574m, 붙임각 2°, 실제 단면의 캠버·두께를 입력에 반영했다. 수평미익 폭 0.70m, 시위 약 0.200000m와 위치도 메시에서 가져왔다. 에일러론은 좌우 각각 0.3478m, 엘리베이터는 각각 0.2780m 구간이다. 조종면 힌지는 약 75% 시위로 근사했다. 수직미익은 고정핀과 러더의 경계를 합쳐 높이 0.30m, 루트 시위 0.27m, 팁 시위 0.13m로 재구성하고 작은 힌지 틈을 닫았다. 수직미익 익형은 H1 수평미익에서 추출한 대칭 단면을 사용했다.

AVL에는 H1 양력면만 넣었다. 동체·착륙장치·후방 통로의 유동은 풀지 않으며 `profile_cd=0.035`는 가정이다. 형상 표시용 CAD의 주익·미익은 추출 단면을 로프트한 근사 솔리드다. 원본 메시를 그대로 보존한 양력면 GLB/STL은 별도로 제공한다.

임시 동체는 H1 단면표의 폭·윗선·어깨·아랫선을 사용한 직선 구간 로프트이며 외피 두께는 1.8mm로 가정했다. 곡면과 하부 모서리는 근사했으며 기존 프레임·장비를 그대로 재현하지 않았다. 센서·핀·출구·문·가이드·포획부·윈치는 이번 시험을 위해 새로 배치했다. 원래 센서 외형과 윈치 위치를 그대로 사용하지 않는다.

### 후방 통로 변경

기수 원점의 뒤쪽 x=0.91~1.14m에 폭 56mm, 높이 56mm의 가이드를 배치했다. 출구는 x=1.14m이고 센서 CG 수납 위치는 (0.99, 0, −0.02)m다. 센서는 길이 120mm, 몸체 지름 28mm, 핀 전체 폭 46mm다. 센서 CG는 뒤쪽 핀 질량 때문에 몸체 기하 중심에서 뒤로 3.8mm 이동한다. 접촉점과 견인점도 이 CG 기준으로 보정했다.

H1 후방 동체는 폭과 높이가 줄어들므로, 이 센서는 원래의 닫힌 외피를 그대로 통과할 수 없다. 임시 CAD에는 x=0.885~1.60m, y=±31mm, z=−62~22mm의 직사각 통로를 빼서 **후방 하부 외피를 절개**했다. 가이드와 출구는 이 안에 배치한다. 절개 후 강도·미익 지지 구조는 설계하지 않았다. 제조용 변경안으로 승인한 형상이 아니다.

문은 출구 하단의 y축을 중심으로 100° 아래로 열린다. 포획 위치는 수납 CG와 같으며, 허용 위치 오차 6mm·상대속도 0.15m/s·각도 8°는 가정이다. CAD 포획 패드는 위치 표시용이고, 툴의 포획력은 선형·회전 스프링으로 계산한다. 문은 회수 명령 종료 후에도 열린 상태로 둔다. 실제 포획 여부를 확인하지 않고 자동으로 닫지 않는다.

## 좌표와 단위

### 재생 화면의 배율

재생 중에는 센서·문·와이어의 좌표만 바꾼다. 축 범위와 3D 비율은 보기별로 고정하며, 장면 설정을 프레임마다 다시 적용하지 않는다. 마우스로 돌린 시점도 유지한다. 전체 보기·컨테이너 확대·내부 보기 버튼을 누를 때만 정해 둔 범위와 시점으로 전환한다. 내부 보기는 외피와 측벽을 숨겨 내부를 보여 주며, 이 표시 선택이 충돌 계산에서 부품을 제외한다는 뜻은 아니다.

`replay_controller.js`가 재생·일시정지·위치 이동을 처리한다. 물체만 갱신할 때는 [Plotly.restyle](https://plotly.com/javascript/plotlyjs-function-reference/#plotlyrestyle)을 사용한다. 보기 버튼에서는 `Plotly.newPlot`으로 선택한 장면을 다시 만들어 이전 확대 상태가 다음 프레임까지 남지 않게 했다. 축 범위·실제 크기 비율·눈금 간격은 보기마다 고정한다.

원본 H1/AVL의 기수 원점 좌표는 x 뒤쪽, y 우측, z 위쪽이다. 운동 계산은 FRD(x 전방, y 우측, z 하방)다. 가정한 기체 CG의 원본 좌표는 **(0.514013158, 0, 0.015657895)m**다.

원본 점 `(x, y, z)`를 기체 CG 기준으로 바꾸면 `(0.514013158 − x, y, 0.015657895 − z)`다. YAML의 `aircraft.cg_m`와 `aero.moment_reference_frd_m`만은 기수 원점에 대한 FRD 좌표 `(-0.514013158, 0, -0.015657895)`를 쓴다. 견인점·출구·센서 수납 위치는 CG 기준이다. 센서 접촉점과 센서 견인점은 센서 CG 기준이다.

STEP은 mm, STL의 정점 값은 m다. STL에는 단위 메타데이터가 없으므로 가져올 때 m를 지정한다. GLB는 m이며 표시 좌표를 `(FRD x, −FRD z, FRD y)`로 변환해 위쪽을 +Y로 두었다. 이는 기체 FRD를 일반 3D 뷰어에 표시하기 위한 회전이며, 해석 입력의 좌표를 바꾸지는 않는다.

## 질량과 물성

기체 2.28kg에는 가정한 배터리·모터·전자장비·윈치·문·가이드·포획부가 포함된다. 센서 0.04kg와 줄 0.00225kg은 이 질량에 포함하지 않는다. 모든 상태에서 총질량은 **2.32225kg**다. 기체는 부품별 상자 질량과 평행축 정리, 센서는 36g 원통과 1g 핀 4개의 질량으로 관성을 계산했다. 재료 밀도나 메시 체적에서 실물 질량을 추정한 값이 아니다.

기체 CG 기준 FRD 관성 텐서(kg·m²)는 다음과 같다. 행렬의 비대각 성분은 텐서 성분이며, AVL 질량 파일의 곱관성 부호로 쓸 때 변환한다.

```text
[[ 0.157778405, 0, -0.005098730],
 [ 0, 0.295347037, 0],
 [-0.005098730, 0, 0.425811049]]
```

센서 CG 기준 관성은 `diag(0.00000492533, 0.00005116107, 0.00005116107)` kg·m²다. 센서 공력·줄 인장 강성 60N·감쇠·접촉·포획·윈치 허용치는 모두 미측정 가정이다. `provenance`와 출처표에 이 상태를 표시했다.

## 재생성과 실행

PowerShell에서 `D:/DBF/안정성 툴`을 현재 폴더로 지정한다. 기존 `.venv`의 CadQuery 2.8.0, NumPy, SciPy, PyYAML, Matplotlib, Plotly와 실제 `vendor/avl/avl.exe`를 사용한다.

```powershell
.venv/Scripts/python test_models/H1_reference/extract_h1.py
.venv/Scripts/python test_models/H1_reference/build_inputs.py
.venv/Scripts/python test_models/H1_reference/build_cad.py
.venv/Scripts/python test_models/H1_reference/run_checks.py --workers 6
.venv/Scripts/python test_models/H1_reference/audit_delivery.py
```

`build_inputs.py`의 치수·질량 상수와 가정값을 수정해 재생성한다. `parameters.json`은 CAD가 읽는 생성 결과다. `build_inputs.py`를 다시 실행하면 이 파일과 H1 사례 YAML을 갱신한다. 해석 형상이나 익형을 바꿨으면 공력표도 다시 계산해야 한다.

`run_checks.py`는 서로 다른 폴더에서 실제 AVL **7×3×5=105조건**을 6개 프로세스로 계산한다. 받음각 −6~12°, 옆미끄럼각 −8~8°, 엘리베이터 −20~20° 격자이며, 넓은 입력 범위 자체가 실속 이후 해석의 정확성을 보장하지 않는다. 조종면 순서는 1 에일러론, 2 엘리베이터, 3 러더다. 기본 확인은 기체 단독 0.5초, 수납 0.25초, 전개 후 센서 횡속도 0.02m/s 교란 0.5초, 문 개방·방출 0.85초다.

완전한 7초 명령 이력을 시험하려면 다음을 실행한다. `--reuse-aero`는 이 H1 공력표를 재사용한다는 뜻이며 기존 Vanilla 공력표와는 무관하다.

```powershell
.venv/Scripts/python test_models/H1_reference/run_checks.py --reuse-aero --modes mission --mission-duration 7
.venv/Scripts/python test_models/H1_reference/visualize_run.py
.venv/Scripts/python test_models/H1_reference/audit_delivery.py
```

각 실행은 새 `runs/날짜_시각` 폴더를 만든다. `checks.json`에서 완료 여부, `trim.json`에서 트림 수치, `mission/progress.json`에서 진행 시간을 확인한다. 실패하면 `failure.log` 또는 `summary.json`의 원인을 읽는다. 초기 익형 순서 오류로 채택하지 않은 계산은 `diagnostics/README.md`에서 구분했다.

## 확인 범위

`cad_validation.json`은 실제 CAD 솔리드 유효성과 STEP 재열기를 기록한다. `clearance_summary.json`은 문을 연 상태에서 센서 CG를 x=0.99~1.72m로 이동시키는 74개 CAD 위치와 수납 상태의 문 회전 간섭을 기록한다. 이 지정 경로는 문을 연 채 역방향으로 움직일 때도 같은 여유를 갖지만, 실제 줄이 센서를 그 경로로 유도한다는 증거는 아니다. 접촉 구는 실제 핀 끝보다 1.5mm 더 두껍게 취급한다.

운동 계산의 `completed`는 요청 시간까지 적분했다는 뜻이고 `captured`는 포획 구속이 작동했다는 뜻이다. **전개·회수 공간 분할 수렴과 줄 출입을 포함한 전체 에너지 수지는 이 제작으로 검증되지 않았다.** `numerically_converged`는 false로 유지한다. 동체 후류·출구 유동, 실물 센서 공력·물성, 구조 강도와 제작 공차도 별도 확인 대상이다.

## 외부 형식 근거

[MIT AVL User Primer](https://web.mit.edu/drela/Public/web/avl/avl_doc.txt)의 형상 좌표·조종면·입력 형식을 확인했다. 공개 Primer 표기는 3.40이며 실제 사용 실행파일은 출력에서 확인한 3.52다. [CadQuery 입출력 문서](https://cadquery.readthedocs.io/en/stable/importexport.html)는 STEP 및 조립체 내보내기 형식의 근거다. 모델 치수의 출처는 외부 He 51 실측 자료가 아니라 현재 프로젝트에 복사된 H1 파일이다.
