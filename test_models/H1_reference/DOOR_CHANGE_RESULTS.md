# H1 도어 개방각·힌지 수정 결과

기존 100°에서 140°·160°로 문을 더 열고, 같은 실제 AVL 공력표를 사용해 각 조건을 0초부터 다시 계산했다. **개방각을 늘리는 것만으로 회수·포획이 해결되지는 않았다.** 새 운동 궤적에서도 도어 접촉이 잠깐 남았고, 출구 진입 과정에서 계산이 중단되거나 실패했다.

이 기록은 높이 56mm 출구의 140° 시험 입력이다. 후속 기본 입력은 [높이 70mm 출구 변경안](OPENING_CHANGE_RESULTS.md)으로 옮겼다. 이 개방각 비교를 제작 승인이나 최적 각도 판정으로 해석하지 않는다. 기존 입력과 궤적은 비교용으로 보존했다.

## 수정한 형상

- 닫힌 문판을 기존 위치에서 뒤로 3mm 옮겨 프레임 바깥면에 배치했다.
- 회전축을 기존 출구 하단 중심에서 뒤·아래로 각각 3mm 옮겼다.
- 0.4초에 지정 개방각에 도달하고 7초까지 유지하도록 명령했다. 회수 성공 후 문 닫힘을 보여 주는 시연이 아니다.

기존 도어는 회전축 근처에서 프레임·가이드 바닥과도 겹쳤다. 힌지만 옮긴 첫 수정안은 1~2°에서 상단 프레임과 겹쳐 사용하지 않았다. 문판까지 옮긴 수정안은 **0~180°의 181개 CAD 자세에서 고정 부품과의 겹침·검사 오류가 0건**이었다. 이는 표본 자세 검사이며 연속 회전 전체의 증명은 아니다.

접촉 계산, 문 표면의 속도, 재생 화면, STEP 검사에 같은 회전축·문판 변환을 적용했다. 형상·입력 검사를 포함한 자동 검사 60개가 통과했다. 원본 STL·STEP은 변경하지 않았으며, [140° 수정 조립체](cad_variants/door_140/H1_door_140_assembly.step)를 별도로 저장했다. STEP 재입력에서도 솔리드 24개와 형상 유효성을 확인했다.

## 확인된 운동 결과

| 수정 배치의 개방각 | 저장된 마지막 시각 | 결과 | 저장 시각의 최대 장력 | 회수 중 도어 접촉 범위 진입 |
|---|---:|---|---:|---|
| 100° | 6.045094초 | 몸체–출구 상단의 간격 한계로 중단, 미포획 | 19.316N | 5.84초부터 17개 표본, 최소 간격 0.269mm |
| 140° | 6.088935초 | 상부 핀–출구 상단의 간격 한계로 중단, 미포획 | 6.498N | 5.95초부터 2개 표본, 최소 간격 0.724mm |
| 160° | 6.027887초 | 그 이후 적분 중 수치 오류, 미포획 | 4.531N | 5.96초부터 2개 표본, 최소 간격 0.540mm |

도어 접촉은 실측 충돌이 아니라 **0.8mm 이내에서 반력이 작용하는 가정 모델**의 판정이다. 160° 수치 오류 직전의 미수렴 시험 상태는 결과로 채택하지 않았다. 표의 값은 마지막으로 저장한 시각까지의 값이라, 160°의 장력이 작다는 이유로 더 좋은 설계로 판정할 수 없다.

140°의 마지막 실제 형상 간격은 0.020mm였다. 센서는 약 13.45° 기울어져 있고, 세로 투영 높이는 62.27mm였다. 출구 높이는 56mm이므로 그 자세 그대로 평행 이동해 들어갈 수 없다. 진입 중 센서를 정렬하는 가이드·견인 경로도 검토해야 한다.

위 세 경우는 같은 수정 힌지·문판 배치로, 개방각만 다르다. 100°→140°에서 저장 최대 장력은 **19.32N→6.50N**으로 줄었지만 최대 접촉 법선력 합은 **289.67N→634.49N**으로 늘었다. 두 궤적 모두 출구 상단 접근으로 중단됐다. 접촉력은 실측 충격 하중이 아니며, 장력 감소만으로 정상 회수를 판정하지 않는다.

이전 원래 배치의 100° 최대 장력은 44.37N이었다. 이 값과 새 140°의 6.50N을 비교하면 **힌지·문판 위치와 개방각을 함께 바꾼 비교**가 되므로, 각도만의 효과로 해석하지 않는다.

## 형상 비교와 새 궤적의 구분

[동일 자세 비교 그림](diagnostics/door_angles/same_pose_comparison.png)은 이전 궤적의 6.14초 센서 자세를 고정한 그림이다. 그 자세에서는 기존 문 간격 0.133mm가 수정한 140° 문에서 31.02mm로 늘었다. **31.02mm는 새 운동 궤적 전체의 최소 간격이 아니다.** 다시 적분하면 센서 경로가 바뀌며, 실제 새 궤적에서는 위 표처럼 짧은 도어 접촉이 남았다.

100°·140°·160° 저장 자세 각각 625개·629개·622개, **총 1,876개**를 별도 OpenCASCADE 교차 검사로 확인했다. 세 경우 모두 CAD 관통과 검사 오류는 0건이었다. 센서·줄의 저장 자세 검사와 별도 도어 회전 검사를 구분해야 하며, 관통 0건이 포획 성공이나 실물 안전성 검증을 뜻하지 않는다.

## 열어 볼 결과와 재실행

- [개방각 비교](runs/door_angle_01/comparison.html), [수치 기록](runs/door_angle_01/comparison.json).
- [140° 실제 계산 재생](runs/door_angle_01/angle_140/mission/H1_replay.html), [160° 중단 전 저장 궤적](runs/door_angle_01/angle_160/mission/H1_replay.html).
- [문 회전 CAD 검사](diagnostics/door_angles/offset_swing.json), [140° 궤적 CAD 검사](runs/door_angle_01/angle_140/mission/cad_collision_audit.json), [160° 궤적 CAD 검사](runs/door_angle_01/angle_160/mission/cad_collision_audit.json).
- 입력은 `../../examples/h1_door_100.yaml`, `h1_door_140.yaml`, `h1_door_160.yaml`에 있다. 검사 기록은 `../../outputs/validation/door_tests.xml`이다.

프로젝트 폴더에서 실행한다. 새 출력 폴더 이름을 사용한다.

```powershell
.venv/Scripts/python test_models/H1_reference/run_door_angle_study.py --output outputs/door_comparison_new --workers 3
foreach ($doorAngle in 100,140,160) {
    .venv/Scripts/python test_models/H1_reference/audit_trajectory_collisions.py "outputs/door_comparison_new/angle_$doorAngle/mission" --stride 1 --workers 6
}
.venv/Scripts/python test_models/H1_reference/summarize_door_study.py outputs/door_comparison_new
foreach ($doorAngle in 100,140,160) {
    .venv/Scripts/python test_models/H1_reference/visualize_run.py --run "outputs/door_comparison_new/angle_$doorAngle/mission"
}
```

기체 양력면은 실제 AVL 3.52의 105조건 공력표를 재사용했다. 문 배치 변경은 그 양력면 형상을 변경하지 않으며, 개방각별 도어 공력·후류 변화는 별도로 해석하지 않았다. 센서·줄 공력은 가정값이고 flow5는 연결하지 않았다. 힌지·구동장치·씰의 상세 구조 및 도어 자체의 관성 반력도 포함하지 않는다. 변경한 조건의 시간 간격·줄 분할 수렴과 실물 검증은 아직 수행하지 않았다.
