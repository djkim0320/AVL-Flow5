# UI 테스트의 특정 기체 의존성 분리

2026-09-28부터 일반 UI 검사는 `ui/tests/fixtures/aircraft.py`에서 생성한 기체를 사용한다. H1·N3의 저장된 공력표나 등록 상태가 UI 테스트의 통과 조건이 되지 않는다.

`verify_ui_registration.py`는 H1 형상에 종속된 기존 검사 중 CG 원점 이동, 접촉 매니페스트, 가이드 구멍과 벽, 수납 관성 회전, 움직이는 문이 없는 등록을 명시적인 모델링 검사로 옮긴 것이다. 모델은 임시 폴더에 등록하며, 공력표가 필요한 검사는 같은 등록 형상으로 실제 AVL을 실행한다. 과거의 다른 형상 표를 빌려 쓰지 않는다.

```powershell
.venv\Scripts\python.exe test_models/H1_reference/verify_ui_registration.py
```

질점 운동·고유모드·회수 중 줄 구간 변경·질량 보존 검사는 `tests/test_ui_generated_pipeline.py`로 옮겼다. 실제 해석기를 실행하며 `avl`, `slow` 표식으로 구분한다. 좌표 변환·등록 서명·빈 템플릿·오류 조건 검사는 일반 UI 테스트에 남는다.

기존 UI 테스트 원문은 `ui/design/hybrid-aero-20260928/modeling-regressions/*.txt`에 보관했다. 삭제된 내장 API를 호출하는 이 사본 자체를 실행 가능한 검사로 간주하지 않는다. 기존 N3의 특정 등록 목록과 S27 로컬 산출물의 특정 시위 범위는 범용 UI의 합격 기준에서 제외했다. 새 생성 기체로 같은 등록 대조·단면 추출 기능을 검사한다. 이 문서는 과거 기체의 모든 수치 회귀 검사를 재실행했다는 뜻이 아니다.
