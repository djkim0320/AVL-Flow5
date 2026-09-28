"""Show commanded timing separately from the actual simulation's reach/status."""
from pathlib import Path
from html import escape
import json
import plotly.graph_objects as go
from dbf_stability import load_case

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent/'runs/span_hold_01'


def main():
    c=load_case(ROOT/'examples/h1_normal_r3_flow5_span_hold.yaml');p=c['mission_profile']
    times,lengths=zip(*c['winch']['length_schedule'])
    fig=go.Figure(go.Scatter(x=times,y=lengths,mode='lines',name='윈치 길이 명령',line=dict(color='#287782',width=3)))
    fig.add_vrect(x0=p['deployment_complete_s'],x1=p['recovery_start_s'],fillcolor='#daf0ee',opacity=.65,line_width=0,layer='below',annotation_text='완전 전개 후 60초 유지',annotation_position='top left')
    fig.update_layout(template='plotly_white',height=390,margin=dict(l=60,r=30,t=45,b=55),font=dict(family='Malgun Gothic, sans-serif'),
        xaxis_title='임무 시각 (s)',yaxis_title='풀린 줄 길이 (m)',xaxis_range=[0,p['end_s']],yaxis_range=[0,2.9])
    run='combined' if (OUT/'combined/summary.json').exists() else 'continuation' if (OUT/'continuation/summary.json').exists() else 'mission'
    summary_path=OUT/run/'summary.json'
    summary=json.loads(summary_path.read_text(encoding='utf8')) if summary_path.exists() else {'status':'not_started'}
    labels={'completed':'계산 완료','running':'계산 중','not_started':'계산 전','runtime_limit':'실행 시간 한도에서 중단',
            'aero_domain_exceeded':'공력표 범위 이탈로 중단','numerical_failure':'수치 계산 중단','numerical_stagnation':'수치 계산 정체',
            'contact_clearance_limit':'최소 형상 간격에서 중단','ground_contact':'지면 도달로 중단'}
    end=summary.get('end_time_s',0.)
    progress=OUT/run/'accepted_progress.json'
    if summary['status']=='running' and progress.exists():end=json.loads(progress.read_text())['time_s']
    status=f"{labels.get(summary['status'],summary['status'])} · 실제 계산 {end:.2f} / {p['end_s']:.2f}초"
    links=''
    if (OUT/run/'H1_replay.html').exists():links+=f'<a href="{run}/H1_replay.html">실제 계산 3D 재생</a> · '
    elif (OUT/'preview/H1_replay.html').exists():
        preview=json.loads((OUT/'preview/summary.json').read_text(encoding='utf8'))
        links+=f'<a href="preview/H1_replay.html" style="display:inline-block;padding:10px 18px;background:#176374;color:white;border-radius:6px">▶ 계산된 구간 재생 (0–{preview["end_time_s"]:.2f}초)</a> · '
    if (OUT/run/'history.html').exists():links+=f'<a href="{run}/history.html">장력·자세 그래프</a> · '
    links+='<a href="mission_plan.json">임무 설정 기록</a>'
    metrics=''
    if 'max_tension_N' in summary and run=='combined':
        metrics=f"<p>계산된 구간의 최대 장력 {summary['max_tension_N']:.3f} N · 최대 피치 변화 {summary['max_pitch_change_deg']:.3f}° · 포획 {'완료' if summary['captured'] else '미완료'}</p>"
    audit_path=OUT/run/'cad_collision_audit.json'
    if audit_path.exists():
        audit=json.loads(audit_path.read_text())
        metrics+=f"<p>CAD 검사 {audit['samples']}시각 · 겹침 {audit['intersecting_samples']}건 · 조회 오류 {audit['cad_query_error_samples']}건. 선택한 시각의 검사이며 연속 무충돌을 입증하지는 않습니다.</p>"
    rows=[('문 개방', '0.00–0.40초'),('센서 전개',f"0.60–{p['deployment_complete_s']:.2f}초"),
        ('2.70m 유지',f"{p['deployment_complete_s']:.2f}–{p['recovery_start_s']:.2f}초 (60초)"),
        ('와이어 회수',f"{p['recovery_start_s']:.2f}–{p['retrieval_complete_s']:.2f}초"),
        ('도어 닫힘', '회수 후 포획 성공 조건을 충족할 때만'),('계산 종료',f"{p['end_s']:.2f}초")]
    page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>2.7m 전개 · 60초 유지 · 회수</title>
<style>body{{font:16px/1.7 'Malgun Gothic',sans-serif;color:#24353e;background:#f5f7f8;margin:0}}main{{max-width:1050px;margin:30px auto;padding:28px 34px;background:white;border-radius:10px}}h1{{font-size:29px;line-height:1.4;margin-top:0}}h2{{font-size:21px;margin-top:30px}}a{{color:#176374}}.status{{padding:14px 18px;background:#edf3f4;border-left:4px solid #287782}}table{{border-collapse:collapse;width:100%;max-width:750px}}td,th{{padding:9px 14px;border-bottom:1px solid #d9e1e4;text-align:left}}small{{color:#53656f}}@media(max-width:700px){{main{{margin:0;padding:18px}}}}</style>
<main><h1>2.7m 전개 → 60초 유지 → 회수</h1>
<p>윙스팬 <b>1.8m × 1.5 = 2.7m</b>. 완전 전개된 시점부터 60초를 유지합니다. 기존 전개·회수 속도와 마지막 저속 진입 구간을 유지했습니다.</p>
<p>{links}</p><div class="status" id="mission-status" role="status">{escape(status)}</div>{metrics}
<h2>입력한 임무 일정</h2><p>아래 선은 윈치 명령입니다. 실제 계산이 도달한 구간과 포획 여부는 위 상태 및 재생 화면에서 확인합니다.</p>
{fig.to_html(full_html=False,include_plotlyjs=True)}
<table><thead><tr><th>단계</th><th>명령 시각 / 조건</th></tr></thead><tbody>{''.join(f'<tr><td>{a}</td><td>{b}</td></tr>' for a,b in rows)}</tbody></table>
<h2>계산 조건</h2><p>줄은 18구간으로 나눠 기존 0.15m 간격을 유지했습니다. 줄 질량은 4.05g이며 기체와 중복 계산하지 않습니다. 기체 형상이 같으므로 앞서 실제 flow5로 계산한 270조건 공력표를 사용합니다.</p>
<p>2.7m는 윈치 가이드에서 센서 견인점까지 풀린 줄의 무변형 길이이며, 동체 내부 경로도 포함합니다. 도어는 포획되지 않으면 열린 상태를 유지합니다. 조종면·추력은 수납 트림 값을 유지합니다.</p>
<small>임시 R3 모델의 계산입니다. flow5는 날개·꼬리날개 공력에 적용했으며 센서·줄 공력과 접촉 물성은 가정값입니다. 수치 수렴 및 실물 검증은 완료하지 않았습니다.</small></main></html>'''
    if summary['status']=='running':
        live='''<script>
const statusLabels=__LABELS__;
const timer=setInterval(async()=>{
  try {
    const state=await (await fetch('__RUN__/summary.json',{cache:'no-store'})).json();
    const running=state.status==='running';
    const progress=running?await (await fetch('__RUN__/accepted_progress.json',{cache:'no-store'})).json():null;
    const time=running?progress.time_s:state.end_time_s;
    document.getElementById('mission-status').textContent=(statusLabels[state.status]||state.status)+' · 실제 계산 '+Number(time||0).toFixed(2)+' / __END__초';
    if(!running){
      const post=await (await fetch('postprocess.json',{cache:'no-store'})).json();
      if(post.stage==='completed'){clearInterval(timer);location.reload();}
      else if(post.stage==='failed'){document.getElementById('mission-status').textContent+=' · 결과 정리 중 오류: '+post.message;clearInterval(timer);}
      else document.getElementById('mission-status').textContent+=' · CAD 검사·재생 준비 중';
    }
  }catch(error){document.getElementById('mission-status').textContent='진행 상태를 불러오지 못했습니다. 저장된 결과 파일을 확인해 주세요.';clearInterval(timer);}
},5000);
</script>'''.replace('__LABELS__',json.dumps(labels,ensure_ascii=False)).replace('__END__',f"{p['end_s']:.2f}").replace('__RUN__',run)
        page=page.replace('</html>',live+'</html>')
    (OUT/'report.html').write_text(page,encoding='utf8')
    print(status)


if __name__=='__main__':main()
