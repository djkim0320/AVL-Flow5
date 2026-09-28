"""An explicit progress/result page for the altitude and airspeed experiment."""
from pathlib import Path
from html import escape
import json


def render(out):
    out = Path(out)
    c = json.loads((out/'inputs.json').read_text(encoding='utf8'))
    f, p = c['flight'], c['mission_profile']
    links = '<a href="../span_hold_01/combined/H1_replay.html">기존 40m · 20m/s 재생</a>'
    for name, label in [('H1_replay.html', '새 조건 3D 재생'), ('history.html', '고도·장력·자세 그래프')]:
        if (out/'mission'/name).exists():
            links += f' <a class="primary" href="mission/{name}">{label}</a>'
    result = ''
    if (out/'comparison.json').exists():
        rows = json.loads((out/'comparison.json').read_text(encoding='utf8'))
        body = ''
        for r in rows:
            body += f'<tr><td>{escape(r["label"])}</td><td>{r["end_time_s"]:.2f}s</td><td>{r["final_altitude_m"]:.2f}m</td><td>{r["max_tension_N"]:.3f}N</td><td>{r["observed_command_hold_s"]:.2f}s</td><td>{"시작함" if r["recovery_started"] else "미도달"}</td><td>{"성공" if r["captured"] else "미완료"}</td></tr>'
        result = '<h2>계산된 구간 비교</h2><div class="table"><table><tr><th>조건</th><th>도달 시각</th><th>최종 기체 고도</th><th>최대 장력</th><th>전개 유지</th><th>회수</th><th>포획</th></tr>'+body+'</table></div><p>각 계산이 도달한 구간의 값입니다. 계산 길이가 다르면 최대값의 비교 범위도 다릅니다.</p>'
    audit = out/'mission/cad_collision_audit.json'
    if audit.exists():
        a = json.loads(audit.read_text())
        result += f'<p>CAD 검사 {a["samples"]}시각 · 겹침 {a["intersecting_samples"]}건 · 조회 오류 {a["cad_query_error_samples"]}건. 검사 시각 사이의 연속 무충돌을 입증하지는 않습니다.</p>'
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>고도·속도 증가 시험</title>
<style>body{font:16px/1.65 'Malgun Gothic',sans-serif;background:#f4f7f8;color:#24353e;margin:0}main{max-width:1060px;margin:28px auto;background:white;padding:30px;border-radius:10px}h1{font-size:28px}h2{font-size:21px}a{color:#176374;display:inline-block;margin:8px 10px 8px 0}.primary{background:#176374;color:white;padding:8px 16px;border-radius:5px}.status{background:#eaf3f4;padding:16px;border-left:4px solid #287782}table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid #dce4e6;padding:10px;text-align:left;white-space:nowrap}.table{overflow:auto}small{color:#586c76}@media(max-width:700px){main{margin:0;padding:18px}}</style>
<main><h1>시작 고도 __ALT__m · 속도 __SPEED__m/s</h1>
<p>기존 40m · 20m/s에서 고도와 속도를 높인 비교 계산입니다. 와이어 2.7m를 전개하고 60초 유지한 뒤 회수하는 일정은 같습니다.</p>
<div class="status" id="status" role="status">진행 상태 확인 중</div>
<p id="trim"></p><p>__LINKS__</p>__RESULT__
<h2>동일하게 유지한 조건</h2><p>전개 완료 __FULL__초 → 60초 유지 → 회수 시작 __RECOVERY__초 → 계산 종료 __END__초. 새 속도에서 수납 트림을 다시 구하고 조종면·추력을 유지합니다. 시작 고도를 높였으며, 고도를 일정하게 유지하는 제어는 추가하지 않았습니다.</p>
<p>실제 flow5를 __SPEED__m/s 조건에서 실행합니다. 공력 계산은 4개 작업으로 나눠 수행합니다. 기존 센서·와이어·접촉 물성과 일정은 유지하며, 공기 밀도는 __RHO__kg/m³로 일정하게 둡니다. 시작 고도는 모델 지면에서 잰 값입니다.</p>
<small>임시 R3 모델입니다. 센서·줄 공력과 접촉 물성은 가정값이며, 수치 수렴 및 실물 검증은 완료하지 않았습니다.</small></main>
<script>
const stages={aerodynamics:'flow5 공력 계산 중',trim:'새 속도 트림 계산 중',simulation:'결합 운동 계산 중',cad_audit:'CAD 간섭 검사 중',replay:'3D 재생 준비 중',completed:'결과 정리 완료',failed:'작업 중단'};
const states={completed:'요청 시각까지 계산 완료',running:'계산 중',ground_contact:'지면 도달로 중단',aero_domain_exceeded:'공력표 범위 이탈로 중단',contact_clearance_limit:'최소 형상 간격에서 중단',numerical_failure:'수치 계산 중단',numerical_stagnation:'수치 계산 정체',runtime_limit:'실행 시간 한도에서 중단'};
const initiallyComplete=__COMPLETE__;
async function json(path){const response=await fetch(path,{cache:'no-store'});if(response.status===404)return null;if(!response.ok)throw Error('상태 파일 읽기 오류');return response.json();}
async function update(){try{
  const pipeline=await json('pipeline.json');
  const summary=await json('mission/summary.json');
  const progress=summary?.status==='running'?await json('mission/accepted_progress.json'):null;
  let text=stages[pipeline.stage]||pipeline.stage;
  if(summary)text+=' · '+(states[summary.status]||summary.status)+' · '+Number(progress?.time_s??summary.end_time_s??0).toFixed(2)+' / __END__초';
  if(pipeline.message)text+=' · '+pipeline.message;
  document.getElementById('status').textContent=text;
  const trim=await json('trim.json');
  if(trim)document.getElementById('trim').textContent='새 속도 트림: 받음각 '+trim.alpha_deg.toFixed(3)+'° · 승강타 '+trim.elevator_deg.toFixed(3)+'° · 추력 '+trim.thrust_N.toFixed(3)+'N';
  if(pipeline.stage==='completed'&&!initiallyComplete){location.reload();return;}
  if(!['completed','failed'].includes(pipeline.stage))setTimeout(update,5000);
}catch(error){document.getElementById('status').textContent='진행 상태를 불러오지 못했습니다. 잠시 후 다시 확인합니다.';setTimeout(update,10000);}}
update();
</script></html>'''
    for key, value in dict(ALT=f'{f["altitude_m"]:g}', SPEED=f'{f["speed_m_s"]:g}', RHO=f'{f["rho_kg_m3"]:g}',
        FULL=f'{p["deployment_complete_s"]:.2f}', RECOVERY=f'{p["recovery_start_s"]:.2f}', END=f'{p["end_s"]:.2f}',
        LINKS=links, RESULT=result, COMPLETE=json.dumps((out/'pipeline.json').exists() and json.loads((out/'pipeline.json').read_text(encoding='utf8'))['stage']=='completed')).items():
        page = page.replace('__'+key+'__', str(value))
    (out/'report.html').write_text(page, encoding='utf8')
