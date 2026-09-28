# 동체 아래로 접히는 270° 도어

현재 형상은 `examples/h1_underbody_270_v2.yaml`이다. 이전 140° 문과 완료된 회수 결과는 그대로 보존했다.

## 형상 변경

출구는 56×70mm로 유지했다. 문판을 아래로 32mm 연장해 102×56×2mm로 만들고, 힌지축을 출구 바닥에서 아래로 30mm, 뒤로 3mm 옮겼다. 문판의 상단은 기존 출구 상단과 일치한다. 닫힘 0°에서 바깥으로 돌아 270°가 되면 문판이 동체 아래에서 전방을 향한다.

직경 1mm 핀, 1.4mm 구멍, 양쪽 브래킷과 접힘 받침을 STEP/STL에 추가했다. 브래킷·받침과 동체가 만나는 곳은 장착부다. 이 장착부의 구조 강도나 접착·체결 방식은 검증하지 않았다. 열린 상태에서 문판과 받침의 명목 간격은 0.2mm이며, 서보와 잠금장치의 응답은 아직 모델에 없다.

- 조립 STEP: `test_models/H1_reference/geometry_variants/normal_r3_underbody_270_v2/H1_underbody_270_assembly.step`
- 형상과 질량 기록: 같은 폴더의 `geometry.json`
- 문 회전 검사: `test_models/H1_reference/runs/door_underbody_03/audit.json`
- 형상 재생: 같은 검사 폴더의 `preview.html`

STEP은 mm, STL/GLB는 m이다. 계산 좌표는 기체 무게중심 기준 전방–우측–하방이다. 내부 `rear_door_100deg` 파일은 기존 변환 규약을 따르는 기준 형상이다. 최종 개방각이 100°라는 뜻은 아니다. 배포용 270° 조립 STEP에는 실제 접힌 위치를 적용했다.

## 검사 결과

OpenCASCADE로 0~270°를 1° 간격으로 검사했다. 문판과 고정 기체 부품, 수납된 센서 사이의 체적 겹침은 271개 자세 모두 0개, 형상 계산 오류도 0개였다. 270°에서 동체와 최소 거리는 11.1895mm, 핀 및 받침과의 간격은 약 0.2mm다. 이 결과는 표본 각도의 간섭 검사이며 연속 회전 전체나 제작 공차를 보증하지 않는다. 추가로 0.2° 간격 1,351개 자세의 CAD 거리와 자세 사이 최대 이동 거리를 비교했다. 0.6~270° 구간에는 양의 간격을 확인했지만, 0°에서는 문이 외피에 맞닿으므로 닫힘 끝점을 포함한 전 구간의 양의 간격 증명은 통과로 표시하지 않았다. 기록은 같은 폴더의 `continuous_clearance.json`이다.

관련 형상·질량·포획 이벤트 테스트 31개, 기존 적분·제어·연결·충돌 검사 테스트 33개, flow5 입출력·축 변환 테스트 3개를 통과했다. 기록은 `outputs/validation/underbody_door_tests.xml`, `underbody_integration_tests.xml`, `underbody_flow5_tests.xml`에 있다.

## 질량과 공력

문판은 기존 모델의 가정 밀도를 유지했다. 문 질량은 1.758g에서 2.543g으로 늘었다. 핀·브래킷·받침에는 합계 10g의 가정 질량을 배정했다. 전체 증가량은 10.784g이며 새 기체 질량은 2.330784kg이다. 센서와 줄 질량은 이 값에 중복 포함하지 않는다. 부품의 CAD 무게중심과 관성으로 기체 질량·무게중심·관성을 다시 계산하고, 모든 형상과 견인점 좌표를 같은 기준으로 옮겼다.

주익과 꼬리날개는 같지만, 기존 실제 flow5 7.57, VLM2, 25m/s 공력표에서 승강타 중간 각도의 보간 오차를 확인했다. -8°, 0°, 8° 격자점에서는 실제 재실행과 정확히 일치했고, 받음각 -2.065°, 승강타 3.6607°에서는 직접 계산과 보간값의 피칭 모멘트 차이가 0.6267Nm였다. 기존 공력표를 사용한 `underbody_mission_01`은 2.688529초에서 중단하고 기록을 보존했다. `underbody_recovery_02`의 궤적도 공력 수정 전 비교 기록으로만 남긴다.

새 공력표는 `runs/underbody_aero_refined_01`에 실제 flow5 실행으로 생성했다. 받음각 12개, 옆미끄럼각 3개, 승강타 17개와 좌우 조종면 미분용 조건을 합쳐 3,060개를 계산했다. 승강타 트림 주변은 0.25° 간격이다. 원본 입력·출력과 별도 트림 검증 기록을 보존한다. 추가 문판·브래킷의 국소 유동, 항력, 힌지 공력 모멘트를 새로 해석한 것은 아니다. 문 질량은 기체 운동 모델에서 닫힌 자세로 고정되므로 개폐에 따른 관성 변화와 서보 반력은 포함하지 않는다.

기존 공력표에서 얻은 수납 트림(받음각 -2.06501°, 승강타 3.66070°, 추력 7.60944N)은 보간 오차 발견으로 대체 대상이다. 이전 형상과의 계산 기록은 `runs/door_underbody_03/trim_comparison.json`에 있다. 고정 길이 전개 트림은 두 형상 모두 평형 계산이 수렴하지 않았다. 이를 물리적으로 평형이 없다는 판정으로 사용하거나 고유값 안정성 결과로 대체하지 않는다.

새 공력표에서 수납 트림은 받음각 -2.064135°, 승강타 3.175254°, 추력 7.564440N이다. 같은 새 공력표를 쓴 이전 140° 형상은 -2.065318°, 3.126989°, 7.564286N이다. 질량·무게중심 변경에 따른 승강타 차이는 약 +0.0483°다. 문 주변 항력 감소량을 추정한 비교가 아니다. 기록은 `runs/door_underbody_03/refined_trim_comparison.json`에 있다.

새 트림의 받음각을 실제 출력 정밀도인 -2.064°로 맞춰 공력표에 없는 승강타 각도에서 flow5를 별도 실행했다. 보간값과 직접 계산의 최대 힘 차이는 0.00301N, 피칭 모멘트 차이는 0.000651Nm이다. 검증점에서 정한 허용치 0.05N·0.02Nm를 만족했다. 이 한 점의 일치가 모든 비행 조건의 공력 정확도나 전체 운동 계산의 수치 수렴을 보증하지는 않는다. `trim_validation.json`은 사용한 공력표의 SHA-256에 연결되어 있으며, 실행기는 검증이 없거나 다른 공력표일 때 시작하지 않는다.

## 전개·회수 재계산

`run_underbody_mission.py`는 수정된 형상의 수납 트림에서 0초부터 계산한다. 고도 100m, 속도 25m/s, 줄 길이 2.7m, 전개 유지 60초와 기존 종방향 제어를 사용한다. 실제 포획 조건을 만족하면 적분을 그 상태에서 끝내고, 0.3초의 안정화 구간을 거쳐 내부 줄 정리 명령을 적용한다. 연결 시 위치·속도·자세·줄 상태는 그대로 유지한다. 포획이 없으면 문 닫힘 인터록이 유지된다.

새 실행 폴더에 계산 코드를 복사해 실행 도중 코드가 달라지지 않게 했다. `pipeline.json`과 각 구간의 `accepted_progress.json`에서 진행 상황을 확인한다. 실행 중인 `accepted_checkpoint.npz`는 열지 않는다. 중간 자세가 필요하면 `accepted_history`의 완료된 조각 파일만 읽는다.

최종 성공 여부는 실행 폴더의 `pipeline.json`과 `complete/summary.json`, `complete/cad_collision_audit.json`으로 확인한다. 형상 재생 화면은 기구 동작만 보여주므로 비행 해석 완료의 근거로 쓰지 않는다.

`underbody_recovery_03`은 새 공력표를 사용해 기존 저장 상태 70.811992초부터 다시 적분하는 회수 후반 비교다. 시작할 때 센서는 이미 가이드 안에 있으므로 출구로 접근·진입하는 구간이나 전개·60초 견인 전체를 검증한 실행이 아니다. 처음 상태만 기존 기록에서 가져오고, 이후에는 수정 질량·관성·270° 문과 새 공력표로 계산한다.

이 실행은 포획·안정화를 마친 뒤 긴 실제 시간 공백으로 실행 시간 제한에 걸렸다. `underbody_recovery_04`는 71.3748328825초의 최종 저장 상태를 공력표·형상·물리 입력·소스 코드 일치 검사 후 이어 계산했다. 재개 지점과 단계 연결 지점의 상태 배열은 완전히 일치한다. 로그의 누적 실행 시간에는 이 시간 공백이 포함되므로 실제 CPU 연산 시간으로 읽으면 안 된다.

최종 결과는 `runs/underbody_recovery_04/complete`에 있다. 70.811992~73.161254초 구간을 계산했고, 71.049529초에 포획된 뒤 줄 정리와 문 닫힘까지 완료했다. 최종 문 각도는 0°, 고도는 99.526127m, 속도는 24.964342m/s다. 최대 장력 2.931478N, 최대 접촉력 9.959800N, 최대 윈치 요구 토크 0.043972Nm와 출력 0.321154W를 기록했다. 입력된 장력·윈치 한계 초과는 없었다. 접촉력은 측정되지 않은 접촉 물성에 의존하는 시제품 예측값이다.

10,337개 저장 상태 중 재생 프레임·이벤트 주변·하중 극값 등을 포함한 91개 자세를 실제 STEP 형상으로 별도 검사했다. 체적 겹침과 CAD 계산 오류는 모두 0개였다. 모든 저장 상태 또는 연속 궤적 전체의 CAD 검사라고 확대 해석하지 않는다. `verification.json`에 데이터 파일 지문, 질량 보존, 유한값, 연결 상태 일치 및 CAD 검사 결과를 기록했다. 재생은 `H1_replay.html`, 시계열은 `timeseries.csv`, 결과 그래프는 `history.html`이다.

```powershell
.\.venv\Scripts\python.exe test_models/H1_reference/run_underbody_mission.py --aero test_models/H1_reference/runs/underbody_aero_refined_01/aero_database.npz --out test_models/H1_reference/runs/underbody_mission_NEW
```

출력 폴더는 새 이름을 사용해야 한다. 표본 CAD 간섭 검사 외에 시간 간격·케이블 분할 수렴, 실측 접촉 물성, 서보와 비행 시험 비교가 남아 있다.

## 전체 임무 실행: underbody_mission_02

2026-09-19에 `runs/underbody_mission_02`를 새 수납 트림의 0초에서 시작했다. 회수 후반 비교에서 확인한 느린 최종 접근 명령만 가져왔으며, 이전 비행 상태는 가져오지 않았다. 완전 전개는 4.135816초, 회수 시작은 64.135816초로 명령했고 두 시각 사이 60초 동안 줄 길이는 2.7m다. 포획 후에는 실제 포획 시각을 기준으로 내부 줄 정리와 문 닫힘 명령을 계산한다.

```powershell
.\.venv\Scripts\python.exe test_models/H1_reference/run_underbody_mission.py --aero test_models/H1_reference/runs/underbody_aero_refined_01/aero_database.npz --command-reference test_models/H1_reference/runs/underbody_recovery_03/case.yaml --out test_models/H1_reference/runs/underbody_mission_NEW
.\.venv\Scripts\python.exe test_models/H1_reference/monitor_underbody_mission.py test_models/H1_reference/runs/underbody_mission_NEW
.\.venv\Scripts\python.exe test_models/H1_reference/finish_underbody_mission.py test_models/H1_reference/runs/underbody_mission_NEW
```

세 명령은 같은 실행 폴더를 대상으로 각각 별도 프로세스에서 실행한다. 이미 실행 중인 폴더에 계산기를 다시 시작하거나 완료 처리 작업을 중복 실행하지 않는다. `underbody_mission_02`는 이후 회수 중 71.114932초에 최소 간격 한도로 중단됐고, 완료 처리 작업도 멈췄다. 전개와 60초 견인은 수행했지만 포획과 문 닫힘은 완료하지 못했다. 원인과 후속 입구 재설계는 [RECOVERY_FUNNEL.md](RECOVERY_FUNNEL.md)에 기록했다.

`finish_underbody_mission.py`는 시간 한도로 끝난 계산만 `continue_underbody_mission.py`로 이어간다. 물리 입력·공력표·형상·계산 코드·포획 상태와 모든 위치·속도·자세가 일치해야 한다. 수치 실패, 공력 범위 이탈, 간섭 중단은 자동으로 재시도하거나 성공으로 바꾸지 않는다. 재개한 실행은 별도 폴더에 보존한다. 이후 계산·STEP 검사·전체 임무 조건 검증을 통과하면 최초 실행 폴더에 `report.html`을 생성하고 진행 화면에 링크를 표시한다.

검증기는 0초 시작, 2.7m 전개와 60초 유지의 실제 기록, 이후 포획과 문 닫힘을 확인한다. 회수 후반만 있는 기존 결과를 전체 임무 완료로 처리하지 않는다. 상태·입력·CAD·공력 파일의 지문도 대조한다. 기존 완료 체크포인트로 정상 재개 검사, 물리 입력 변경 거부, 상태 불일치 거부를 확인했으며, 기록은 `outputs/validation/full_mission_restart.json`에 있다.

진행 중에 별도로 검사한 0~2.256638초의 47개 전개 자세에서는 STEP 체적 겹침과 CAD 오류가 모두 0건이었다. 기록은 `runs/underbody_mission_02/deployment_geometry_01`이다. 약 0.05초 간격으로 고른 실제 저장 자세의 중간 검사이며, 이후 견인·회수나 전체 연속 궤적을 검증한 결과는 아니다.
