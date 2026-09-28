# N3 신규 DBF 기체

2026-09-26에 AIAA 공식 페이지가 연결한 **2026–27 DBF 규정**을 확인하고 새로 만든 해석용 기체다. 공식 PDF 파일명에는 `Draft`가 들어 있으므로 최종판과 추가 Q&A가 나오면 다시 대조해야 한다.

- [공식 규정 페이지](https://aiaa.org/get-involved/university-students/dbf/competition-information/rules-faq-qa/)
- [확인한 규정 PDF](https://aiaa.org/wp-content/uploads/2026/09/DBF-2027-Rules-Draft.pdf)
- [로컬 결과 보고서](http://127.0.0.1:8767/data/reports/N3_DBF2027/report.html)

## 사용할 파일

| 파일 | 용도 |
|---|---|
| `N3_aircraft_open.step` | mm 단위 CAD 조립체, 문 270° 고정 |
| `N3_aircraft_open.glb` | m 단위, Y-up 시각화 모델 |
| `N3_ready.dbf.json` | DBF Studio에서 **프로젝트 열기**로 불러오는 최종 배치 |
| `report.html`, `N3_geometry.html` | 결과 보고서와 회전 가능한 새 형상 |
| `design.py`, `geometry_definition.json` | CAD·공력 모델이 공유하는 형상 정의 |
| `N3_extra.yaml`, `aero/extra/` | 최종 기본 해석 입력, AVL·에어포일·질량 파일 |
| `mass_budget.csv`, `dimensions.json` | 질량·관성·치수 가정 |
| `runs/final/summary.json` | 실행 ID, 평형, 수렴 비교와 직접 실행 검증 |
| `sources/` | 원문 PDF, 확인일, SHA-256, 텍스트와 확인용 페이지 이미지 |

DBF Studio를 실행한 뒤 `N3_ready.dbf.json`을 열면 새 기체, 줄 출구, 120 g 질점, 3 m 줄이 함께 나타난다. 센서 CAD 입력은 추가하지 않았다. 화면에서 질점은 길이 확인을 위해 출구 아래에 표시되며, 실제 견인 평형 위치는 해석으로 따로 구한다.

2026-09-27 개선에서는 최종 CAD 등록본으로 AVL 105조건을 다시 계산해 이 배치 파일의 기본 공력표로 연결했다. 새 표의 계수·미계수 배열은 최초 AVL 표와 정확히 일치하며, 형상과 참조 에어포일의 해시도 저장한다. 현재 기본 연결 정보는 `ui_default_aero.json`, 최초 두 해석기의 비교 기록은 `runs/final/`에 있다. [협업 개선 기록](http://127.0.0.1:8767/data/reports/claude_collab_20260926/report.html)에서 후속 검사와 남은 문제를 확인할 수 있다.

## 설계와 좌표

날개폭 1.80 m, 면적 0.5552 m², MAC 0.31365754 m이다. 주익은 NACA 2412, 꼬리날개는 NACA 0012로 구성했다. 외측 주익에 비틀림을 주고, 꼬리날개 설치각은 실제 AVL 설계 비교를 거쳐 +1°로 정했다. 후방 화물칸 위로 꼬리 붐을 올렸고 문은 동체 아래로 접었다. 상부 차단 플러그와 측면 RC 스위치는 별개다.

설계 원점 좌표는 ARU(후방·우측·상방), 계산과 배치 저장 좌표는 CG 기준 FRD(전방·우측·하방)다. STEP는 CG 기준 FRD를 mm로 내보냈다. GLB는 같은 꼭짓점을 Y-up으로 변환했다.

기체 2.695 kg은 부품별 **가정 질량**이다. 추진 전지 66.6 Wh, 수신기·서보용 별도 전지 4.62 Wh를 가정했으며 실제 상용 부품은 선정하지 않았다. 수신기와 별도 전지 합계에 80 g를 배정했다. 질점과 줄 질량은 기체 질량에 중복 포함하지 않는다. M2 비교에서는 상자 70 g를 별도로 가정했다.

센서 220×90×100 mm와 상자 280×130×130 mm의 설치 공간을 확인했다. 움직이는 센서의 접촉이나 포획을 해석한 것은 아니다. 실물 CG 표시, 구조 강도, 배선·퓨즈·차단·BEC 비활성화, 센서 조명과 안정성, 상자 낙하·비행 시험은 완료하지 않았다.

## 계산과 검증 범위

AVL 3.52와 flow5 7.57 실행파일을 실제로 실행했다. 이전 기체 공력표를 대체값으로 쓰지 않았다. 독립 작업은 최대 12개 CPU에 배정하고 작업별 폴더를 분리했다. 원본 입력과 출력은 각 UI 실행 폴더와 `runs/`에 남겼다.

384·864·1944·3456 패널을 비교했고, flow5 중립 조건은 6144·9600 패널도 확인했다. flow5 항력과 요 감쇠는 더 촘촘한 격자에서도 수렴하지 않았다. 보고서에 실패 항목과 실제 변화율을 표시했다. **모든 항목이 수렴했다거나 실기 안전성을 검증했다는 판정은 없다.**

각 해석기의 공력표에서 트림을 구한 뒤 그 조건을 실제 해석기에 다시 입력했다. flow5는 승강타 보간 오차가 커서 운용점 주변 표를 추가 계산했다. 최초 표와 결과는 `runs/before_interpolation_refinement/`에 보존했다.

flow5의 β=0° 조건에 측력계수 약 10⁻⁷의 수치 잔차가 남는다. 원본 계수를 0으로 바꾸지 않았다. N3에는 병진 가속도 허용치 10⁻⁵ m/s², 각가속도 허용치 5×10⁻⁵ rad/s²를 명시하고 실제 잔차를 저장한다. 일반 코드의 기본 허용치는 기존 10⁻⁵를 유지한다. 엄격한 기준에서 실패했던 실행도 남겼다.

CD₀=0.030은 동체·마찰 등을 보완하는 가정이며 0.020/0.040 민감도를 함께 계산했다. 열린 문 주변 박리, 동체 후류, 프로펠러 후류와 실속은 이번 VLM 결과로 검증하지 않는다. 점질량 센서에는 공력·회전·접촉이 없다. flow5의 출력에 없는 교차 회전율 미계수는 기존 연결부의 명시적 근사이므로 해당 안정성 결과도 검증 전 상태다.

## 다시 실행하기

프로젝트 루트 `D:\DBF\안정성 툴`에서 다음 명령을 실행한다. `--out`은 기존 결과를 덮어쓰지 않도록 새 폴더 이름으로 지정한다.

```powershell
.venv/Scripts/python.exe test_models/N3_DBF2027/design.py
.venv/Scripts/python.exe test_models/N3_DBF2027/analysis_campaign.py mesh --out runs/mesh_new --meshes coarse medium fine extra --workers 12
.venv/Scripts/python.exe test_models/N3_DBF2027/analysis_campaign.py database --out runs/avl_new --mesh extra --backend avl --workers 12
.venv/Scripts/python.exe test_models/N3_DBF2027/analysis_campaign.py database --out runs/flow5_new --mesh extra --backend flow5 --workers 12
```

기본 YAML 표는 넓은 범위의 105조건이다. UI 최종 flow5 표의 세밀한 각도 목록은 최종 `request.json` 또는 `N3_ready.dbf.json`의 `analysis.aero_grid`에 저장했다. 같은 목록으로 재현하려면 해당 값을 입력한다.

CAD나 공력 형상을 바꾸면 UI에서 새 모델로 등록해야 한다. 등록 파일을 직접 수정하지 않는다. 이번 전원 부품 위치 수정 전후 공력 파일과 질량·관성의 일치 확인은 `cad_revision_audit.json`에 기록했다.

60초 견인은 이전 시험 설정이었다. 현재 UI의 전개 비행·회수 조건은 문이 열린 완전 전개 상태에서 시작하며 포획과 문 닫힘을 계산하지 않는다.
