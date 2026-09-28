"""Compare genuine recovery trajectories and their independent CAD audits."""
from pathlib import Path
from html import escape
import argparse,hashlib,json,os
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dbf_stability.analysis import load_result
from capture_check import capture_readiness

HERE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory',type=Path)
    parser.add_argument('--variants',nargs='+',default=['original_slow','round_nominal'])
    parser.add_argument('--extra',action='append',default=[],help='name=relative-or-absolute-case-directory')
    args=parser.parse_args();out=args.directory.resolve()
    names={'baseline':'기존 회수 · 수정 전','original_slow':'기존 센서 · 느린 회수','round_nominal':'둥근 앞코·X자 핀 · 기존 속도','round_slow':'둥근 앞코·X자 핀 · 느린 회수','stop_slow':'원뿔형 정지부 · 느린 회수','stop_nominal':'원뿔형 정지부 · 기존 속도','damped_80':'정지부 · 감쇠 80 Ns/m','damped_240':'정지부 · 감쇠 240 Ns/m'}
    names['analytic_80']='곡면 접촉 · 정지부 · 감쇠 80 Ns/m'
    names['shielded_cleanup']='내부 와이어 공력 수정 · 포획 후 줄 정리'
    paths=[('baseline',HERE/'runs/control_100m_25ms_02/mission')]
    for name in args.variants:
        pipeline=json.loads((out/name/'pipeline.json').read_text(encoding='utf8'))
        paths.append((name,out/name/pipeline.get('result_directory','mission')))
    for entry in args.extra:
        name,directory=entry.split('=',1);parent=(out/Path(directory)).resolve()
        pipeline=json.loads((parent/'pipeline.json').read_text(encoding='utf8'))
        paths.append((name,parent/pipeline.get('result_directory','mission')))
    fig=make_subplots(rows=3,cols=2,subplot_titles=['줄 장력','접촉력 · 수치 모델값','수납 위치까지 거리','기체 고도','센서 상대속도','문 개방각'],vertical_spacing=.12)
    records=[];rows=[];links=[]
    colors=['#8b969e','#14758e','#c57612','#7146a8','#bb3f59','#397138']
    labels={'completed':'계산 완료','contact_clearance_limit':'최소 거리 도달','numerical_stagnation':'수치 계산 정체','runtime_limit':'시간 한도 도달','geometry_invalid':'CAD 겹침 발견','numerical_failure':'수치 계산 실패'}
    for index,(name,path) in enumerate(paths):
        color=colors[index%len(colors)]
        r=load_result(path);audit_path=path/'cad_collision_audit.json'
        audit=json.loads(audit_path.read_text(encoding='utf8')) if audit_path.exists() else None
        start=max(69.4,float(r.time[0]));frame=r.table.loc[r.table.time_s>=start].copy()
        center=np.asarray(r.config['bay']['stowed_center_m'])
        local=frame[['sensor_local_x_m','sensor_local_y_m','sensor_local_z_m']].to_numpy()
        frame['capture_distance_mm']=np.linalg.norm(local-center,axis=1)*1000
        success=bool(r.summary['status']=='completed' and r.summary['captured'] and abs(r.summary['final_door_deg'])<1e-6 and audit and audit['geometry_validity']=='no_overlap_at_checked_samples' and not audit['cad_query_error_samples'])
        item=dict(name=name,label=names.get(name,name),path=str(path),status=r.summary['status'],
            capture_and_closure_with_sampled_clearance=success,captured=r.summary['captured'],capture_time_s=r.summary['capture_time_s'],
            start_s=float(frame.time_s.iloc[0]),end_s=float(frame.time_s.iloc[-1]),max_tension_N=float(frame.tension_N.max()),
            max_contact_N=float(frame.contact_N.max()),final_altitude_m=float(frame.altitude_m.iloc[-1]),
            final_speed_m_s=float(frame.airspeed_m_s.iloc[-1]),final_door_deg=float(frame.door_deg.iloc[-1]),
            capture_readiness=capture_readiness(r),numerically_converged=r.summary['numerically_converged'],
            contact_part_materials=r.config.get('collision',{}).get('part_materials',{}),
            violations=r.summary.get('violations',[]),
            cable_bay_shielding=r.config['flight'].get('cable_bay_shielding',False),
            internal_cable_flow_factor=r.config['flight'].get('internal_cable_flow_factor'),
            post_capture_cleanup=r.config.get('mission_profile',{}).get('post_capture_reel_speed_m_s'),
            analytic_conical_stop=r.config.get('collision',{}).get('analytic_conical_stop'),
            source_states_sha256=hashlib.sha256((path/'states.npz').read_bytes()).hexdigest(),
            source_inputs_sha256=hashlib.sha256((path/'inputs.json').read_bytes()).hexdigest(),
            audit={k:audit[k] for k in ['samples','saved_states','intersecting_samples','cad_query_error_samples','geometry_validity','continuous_collision_detection']} if audit else None)
        records.append(item)
        for row,col,key,unit in [(1,1,'tension_N','N'),(1,2,'contact_N','N'),(2,1,'capture_distance_mm','mm'),(2,2,'altitude_m','m'),(3,1,'sensor_relative_speed_m_s','m/s'),(3,2,'door_deg','°')]:
            fig.add_trace(go.Scatter(x=frame.time_s,y=frame[key],name=item['label'],legendgroup=name,showlegend=row==1 and col==1,line=dict(color=color,width=2)),row=row,col=col)
            fig.update_yaxes(title_text=unit,row=row,col=col);fig.update_xaxes(title_text='시뮬레이션 시간 / s',row=row,col=col)
        result='포획·문 닫힘 확인' if success else labels.get(r.summary['status'],r.summary['status'])+' · 포획 '+('됨' if r.summary['captured'] else '안 됨')
        audit_text=f"{audit['samples']}시각 / 겹침 {audit['intersecting_samples']} / 오류 {audit['cad_query_error_samples']}" if audit else '검사 안 됨'
        rows.append(f"<tr><th>{escape(item['label'])}</th><td>{escape(result)}</td><td>{item['max_tension_N']:.2f}</td><td>{item['max_contact_N']:.2f}</td><td>{item['start_s']:.3f}–{item['end_s']:.3f}</td><td>{audit_text}</td></tr>")
        if (path/'H1_replay.html').exists():
            relative=Path(os.path.relpath(path/'H1_replay.html',out)).as_posix()
            links.append(f'<a href="{escape(relative,quote=True)}">{escape(item["label"])} 재생</a>')
    passed=[r for r in records if r['name']!='baseline' and r['capture_and_closure_with_sampled_clearance']]
    headline='회수·포획·문 닫힘까지 계산했습니다.' if passed else '수정안 계산 결과 · 회수 성공 여부 확인'
    conclusion=(' / '.join(r['label'] for r in passed)+'에서 포획과 문 닫힘을 확인했습니다. 검사한 CAD 자세에서 겹침이 없었습니다.') if passed else '현재 결과에서 포획·문 닫힘·CAD 검사를 모두 통과한 수정안은 없습니다.'
    fig.update_layout(template='plotly_white',height=1080,legend=dict(orientation='h',y=1.08),margin=dict(l=55,r=20,t=100,b=40))
    html=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>센서 회수 수정 결과</title>
<style>body{{font-family:system-ui,'Malgun Gothic',sans-serif;max-width:1150px;margin:auto;padding:28px 20px;color:#24343c;line-height:1.7}}h1{{font-size:28px}}.note{{background:#eef4f5;padding:18px;border-radius:10px}}.links{{display:flex;gap:20px;flex-wrap:wrap;margin:24px 0}}a{{color:#075b9d}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{padding:12px;border-bottom:1px solid #dbe2e6;text-align:left}}.table{{overflow-x:auto}}</style>
<h1>{headline}</h1><p><strong>{escape(conclusion)}</strong></p>
<p>출구에서 센서가 걸렸다가 튀는 문제를 줄이기 위해 회수 속도와 센서 형상을 비교했습니다. 수납 위치를 지나치는 문제에는 물리적인 정지부를 추가했습니다. 접촉 시작점에서 가까운 표면을 빠뜨려 힘이 끊기던 수치 오류도 수정했습니다.</p>
<div class="links">{' '.join(links)}</div>
<div class="note"><strong>저장 자세에서 시작한 회수 구간 시험</strong>입니다. 앞코 수정안은 기존 계산의 69.4초, 정지부 추가안은 앞코 수정안의 70.812초 자세에서 시작합니다. 정지부는 20g 추가 질량·무게중심·관성을 반영했습니다. 감쇠값 80·240 Ns/m는 실측하지 않은 완충부 가정입니다. 새 형상을 처음부터 전개한 결과가 아닙니다. 기체 공력은 실제 flow5 7.57의 25m/s 공력표를 사용했습니다. 센서 공력·후류·접촉 물성은 가정값이며, 아래 접촉력은 수치 모델값입니다. 시간·케이블 분할 수렴성과 실물 안전성은 입증하지 않았습니다.</div>
<p>최대값은 표에 적힌 각 구간의 값입니다. 시작 시각과 상태가 다른 정지부 추가안의 최대값을 같은 조건의 성능 비교로 읽으면 안 됩니다. 각 선은 해당 계산이 끝난 곳에서 멈춥니다. CAD 검사는 모든 연속 시각의 무관통을 보증하지 않습니다.</p>
<p>곡면 접촉안은 반구·원뿔의 해석식과 256방향 접촉력 분배를 사용합니다. 적용 가능한 곡면 간격은 CAD와 대조했고, 다른 부품과 와이어의 메시 검사를 유지합니다. 기존 삼각형 메시와 접촉력 분배 방식이 달라 하중이 완전히 같은 모델은 아닙니다. 포획 시각이 가깝다는 사실만으로 충격 하중의 정확도를 입증하지는 않습니다.</p>
<p>내부 와이어 공력 수정안은 통과 구멍을 22mm로 넓히고, 동체 안의 공기가 기체와 함께 움직인다는 가정을 적용합니다. 각 줄 구간의 외부 노출 비율을 반영합니다. 포획 후 저장 상태를 그대로 이어받아, 남은 내부 줄을 최대 0.25m/s로 정리하고 문을 닫는 명령을 별도로 계산합니다. 수정 전 저속 회수 명령과 결과를 같은 운용 조건으로 보아서는 안 됩니다.</p>
<div class="table"><table><thead><tr><th>조건</th><th>결과</th><th>최대 장력 N</th><th>최대 접촉력 N</th><th>계산 구간 s</th><th>CAD 검사</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>
{fig.to_html(full_html=False,include_plotlyjs=True)}
<p><a href="analysis.json">수치·입력 출처·파일 검증값</a></p></html>'''
    (out/'analysis.json').write_text(json.dumps(dict(scope='Recovery-only experiments from recorded poses; per-case start times and mass changes differ',records=records),ensure_ascii=False,indent=2),encoding='utf8')
    (out/'report.html').write_text(html,encoding='utf8')
    print(json.dumps(dict(report=str(out/'report.html'),passed=[r['name'] for r in passed]),ensure_ascii=False))


if __name__=='__main__':main()
