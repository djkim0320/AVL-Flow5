# H1 출구 높이 변경 결과

출구를 **폭 56mm × 높이 70mm**로 키웠다. 바닥·견인점은 유지하고 상단을 14mm 올렸으며 내부 천장·측벽·프레임·동체 절개부·문판을 함께 바꿨다. 문 개방각은 140°로 유지했다. 재생 화면, 접촉 계산, 실제 STEP 검사는 같은 형상 폴더를 사용한다.

기존 56mm 출구에서 멈춘 자세를 그대로 놓으면 센서–상단 간격은 **0.020 → 6.738mm**로 늘어난다. 이는 해당 자세의 간섭 제거를 확인한 결과이며 새 회수 궤적의 성공을 뜻하지 않는다. [같은 자세의 단면 비교](diagnostics/opening/same_pose_comparison.png)를 제공한다.

80mm 안도 만들었지만, 커진 문판이 개방 중 수직·수평 꼬리날개와 겹쳤다. 0~180°의 181개 자세 중 17개에서 교차가 발생해 운동 계산에서 제외했다. 70mm 안은 같은 181개 문 자세에서 겹침·CAD 오류가 0건이었다. 이는 표본 자세 검사이며 연속 회전 전체의 증명은 아니다.

## 실제 계산 결과

센서 전체가 출구를 지나 내부로 들어왔다. 이번 계산은 내부 윈치에 접근하면서 중단됐으며, 포획하지 못했다. 회수 중 상단 최소 간격은 0.717mm이고, 저장된 1개 시각이 수치 접촉 범위 0.8mm 안에 들었다. 상부가 완전히 무접촉인 결과는 아니다.

센서·줄·기체·윈치·조종 입력과 핵심 운동 코드를 유지하고, 70mm 안을 0초부터 7초까지 요청해 계산했다. 기존에 멈춘 상태에서 이어 붙이거나 위치·속도를 보정하지 않았다.

| 출구 높이 | 저장된 계산 결과 | 회수 중 센서–상단 최소 간격 | 최대 장력 | 포획 | CAD 검사 시각 / 겹침 / 오류 |
|---|---|---:|---:|---|---|
| 56mm | 6.089초 중단 · 최소 간격 한계 | 0.020mm | 6.498N | 미포획 | 629 / 0 / 0 |
| 70mm | 6.171초 중단 · 최소 간격 한계 | 0.717mm | 6.569N | 미포획 | 638 / 0 / 0 |

70mm 안의 마지막 근접 부품: 센서 몸체 / 윈치, 간격 0.019mm.

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
