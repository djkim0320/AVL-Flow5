"""Readable HTML of retained, actual AVL outputs and interpolation comparisons."""
from pathlib import Path
from html import escape
import json

HERE=Path(__file__).resolve().parent
root=HERE/'aerodynamics/connection_check'
comparison=json.loads((root/'comparison.json').read_text(encoding='utf8'))
trim=comparison['stowed_trim']
rows=''.join(f'<tr><th>{k}</th><td>{trim["direct_coefficients"][k]:.8f}</td>'
             f'<td>{trim["cached_coefficients"][k]:.8f}</td></tr>' for k in ('CX','CZ','Cm'))
raw=escape((root/'stowed_trim/forces.txt').read_text(encoding='utf8'))
log=escape((root/'stowed_trim/stdout.txt').read_text(encoding='utf8'))
commands=escape((root/'stowed_trim/commands.txt').read_text(encoding='utf8'))
cases=len(list((HERE/'aerodynamics/avl_runs_cdac0fce').glob('case_*')))
page=f'''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>H1 · 실제 AVL 공력 계산 확인</title>
<style>
body{{margin:0;background:#f3f5f6;color:#172d35;font:16px/1.7 system-ui,sans-serif}}
main{{max-width:1000px;margin:32px auto;padding:0 24px}}h1{{font-size:28px;margin:8px 0}}
section{{background:white;border:1px solid #d9e2e6;border-radius:12px;padding:24px;margin:20px 0}}
h2{{font-size:20px;margin:0 0 12px}}p{{margin:12px 0}}.label{{color:#24736b;font-weight:700}}
table{{border-collapse:collapse;width:100%}}th,td{{text-align:left;border-bottom:1px solid #d9e2e6;padding:10px}}
td{{font-variant-numeric:tabular-nums}}pre{{overflow:auto;background:#edf2f4;padding:16px;font:13px/1.6 Consolas,monospace}}
summary{{cursor:pointer;font-weight:600}}a{{color:#165faf}}
</style><main>
<span class="label">MIT AVL {escape(trim['solver_version'])} · 실제 실행 기록</span>
<h1>기체 공력은 AVL에서 계산합니다</h1>
<p>H1-A 양력면 형상으로 계산한 {cases}조건의 공력표를 Python 운동 계산에 연결했습니다.
Python은 이 표를 읽고 센서·와이어·접촉력을 더해 시간에 따른 움직임을 계산합니다.</p>
<section><h2>연결을 다시 확인한 결과</h2>
<p>두 조건을 AVL 실행파일로 새로 계산했습니다. 받음각·옆미끄럼각·엘리베이터가 모두 0°인
격자점에서는 여섯 힘·모멘트 계수가 저장 공력표와 출력 정밀도에서 일치했습니다.</p>
<p>아래는 수납 트림점(받음각 −1.635911°, 엘리베이터 3.166695°)의 직접 계산과 보간값 비교입니다.</p>
<table><thead><tr><th>계수</th><th>AVL 직접 계산</th><th>공력표 보간</th></tr></thead><tbody>{rows}</tbody></table>
<p>20m/s에서 축방향 힘 차이는 약 0.25N, 피치 모멘트 차이는 약 0.029N·m입니다.
이 비교만으로 전체 운용 범위의 보간 정확도를 검증한 것은 아닙니다.</p></section>
<section><h2>아직 가정으로 남은 부분</h2>
<p>센서·줄 공력, 추가 항력, 후류·노출 정도는 가정 모델입니다.
후방 문 주변 박리와 센서–기체 유동 간섭을 해석한 결과가 아닙니다. flow5는 아직 연결하지 않았습니다.</p></section>
<section id="raw"><h2>AVL 공력 출력 원본</h2>
<p>보존된 <code>stowed_trim/forces.txt</code>의 내용입니다. 출력 숫자를 다시 만들거나 수정하지 않았습니다.</p>
<pre>{raw}</pre><details><summary>실행 로그와 버전 확인</summary><pre>{log}</pre></details>
<details><summary>AVL에 전달한 명령</summary><pre>{commands}</pre></details></section>
</main></html>'''
(root/'report.html').write_text(page,encoding='utf8')
print(root/'report.html')
