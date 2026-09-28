# 외부 구성요소와 재현 정보

flow5는 [공식 7.57 Windows 배포본](https://github.com/techwinder/flow5/releases/tag/v7.57)을 수정 없이 별도 프로세스로 실행한다. 프로젝트 내부 `vendor/flow5`에 실행파일, 전체 원본 소스 ZIP과 GPL 고지를 보존한다. `scripts/fetch_flow5.py`가 배포 ZIP과 실행파일 해시를 검사하며, `installation.json`에 원본 주소와 해시를 기록한다. 자동 연결 범위와 좌표 변환은 [flow5 연결 안내](FLOW5_CONNECTION.md)에 설명한다.

공력 계산은 [MIT AVL 공식 배포](https://web.mit.edu/drela/Public/web/avl/)의 AVL 3.52 Windows 실행파일을 별도 프로세스로 호출한다.

- 실행파일: `vendor/avl/avl.exe`
- SHA-256: `443520d255408491222a8df9060bd000f78da95a68845a2f6efdbc67e203f07a`
- 원본 소스 묶음: `vendor/avl/source.tgz`
- 동봉 소스: `vendor/avl/AVL3.52rel09032025`
- GPL v2 원문: `vendor/avl/COPYING-GPL-2.0.txt`
- 원본 예제: 동봉 소스의 `runs/vanilla.avl`, `runs/sd7037.dat`
- 이 프로젝트의 변환: 전체 선형 치수 0.17배, 면적 0.17²배. 원본 제목과 출처를 유지한다.

AVL 저작권과 GPL v2 또는 이후 버전 고지는 원본 소스 헤더에 있다. AVL을 수정한 실행파일은 만들지 않았다. 이 저장 폴더를 재배포할 때 AVL 원본 소스·고지와 라이선스 조건을 함께 확인한다. 패키지 의존성은 `pyproject.toml`, 실제 실행 환경의 버전은 `requirements-lock.txt`에 기록한다.

공력 데이터의 메타데이터에는 사용한 실행파일과 형상의 해시, 원시 실행 폴더가 들어 있다. 원시 폴더에는 입력 형상·명령·실제 출력·표준 출력·오류 출력을 보존한다. `.mass`의 관성곱 순서는 동봉 `src/amass.f`에서 확인한 `Ixy Ixz Iyz`를 사용한다. 좌표는 FRD에서 AVL 형상축(후방·우측·상방)으로 변환한다.
