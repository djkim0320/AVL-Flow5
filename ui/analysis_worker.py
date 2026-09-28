"""Real solver entry point; receives frozen, validated server inputs only."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json
import sys
import time
import traceback
import html
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from analysis_bridge import ROOT, initial_from_scene
from analysis_jobs import write_json
from dbf_stability import AeroDatabase,build_aero_database,solve_trim,analyze_stability,simulate


def stage(folder,name,message,**extra):
    p=folder/'job.json';r=json.loads(p.read_text('utf8'));r.update(state='running',stage=name,message=message,**extra);write_json(p,r)


def replay_data(result,project,prepared,folder):
    from dbf_stability.math3d import rotation
    from dbf_stability.door import door_hinge
    frames=[];c=result.config;n=c['cable']['segments']
    indices=np.linspace(0,len(result.time)-1,min(600,len(result.time))).astype(int)
    if prepared['settings']['task']=='sequence':
        # Keep both sides of each phase transition even in a long hold replay.
        boundaries=[int(np.searchsorted(result.time,t)) for t in prepared['schedule'].values()]
        indices=np.r_[indices,[i for k in boundaries for i in (k-1,k) if 0<=i<len(result.time)]]
    indices=np.unique(indices)
    for i in indices:
        y=result.states[i];r=rotation(y[6:10]);rs=rotation(y[19:23]);k=int(result.table.active_nodes.iloc[i])
        pos=(y[13:16]-y[:3])@r;rot=r.T@rs
        nose=pos+rot@np.array(c['sensor']['tow_point_m'])
        line=np.vstack([nose,(y[26:].reshape(n,6)[:k,:3]-y[:3])@r,c['aircraft']['tow_point_m']])
        frames.append(dict(t=float(result.time[i]),sensor=pos.tolist(),quaternion=Rotation.from_matrix(rot).as_quat().tolist(),
                           aircraft_quaternion=Rotation.from_matrix(r).as_quat().tolist(),
                           altitude_m=float(-y[2]),cable=line.tolist(),door=float(result.table.door_deg.iloc[i]),
                           length_m=float(result.table.length_m.iloc[i]),
                           stage=str(result.table.mission_stage.iloc[i]) if 'mission_stage' in result.table else str(result.table.phase.iloc[i])))
    write_json(folder/'replay.json',dict(frames=frames,phase=result.summary['phase'],mapping=prepared['mapping'],project_file='replay_project.json',
                  model_name=prepared['profile']['name'],model_id=prepared['profile']['id'],
                  hinge=door_hinge(c['bay']).tolist(),status=result.summary['status'],
                  note=('질점 운동 계산 재생 · 구는 위치 표시용 · 형상 접촉/센서 공력 제외' if c['sensor'].get('model')=='point_mass' else '실제 계산 상태 재생 · 기체 CG 추적/지면축 고정 · 형상 간섭 감사/수렴 검증 미완료')))


def report(folder,prepared,summary,modes=None):
    esc=lambda x:html.escape(str(x))
    rows=''.join(f'<tr><th>{esc(k)}</th><td>{esc(v)}</td></tr>' for k,v in summary.items() if not isinstance(v,(dict,list)))
    mode_table=modes.to_html(index=False,escape=True,float_format=lambda x:f'{x:.5g}') if modes is not None else ''
    text=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DBF 해석 결과</title><style>body{{font:15px/1.65 'Segoe UI','Malgun Gothic',sans-serif;color:#0b1c30;background:#f8f9ff;margin:32px auto;padding:0 20px;max-width:1000px}}table{{border-collapse:collapse;width:100%;background:white}}td,th{{padding:8px;border:1px solid #dce3ed;text-align:left}}a{{color:#006194}}.note{{padding:16px;background:#fff4df}}.scroll{{overflow:auto}}iframe{{width:100%;height:820px;border:0}}h1{{font-size:24px}}</style>
<h1>{esc(prepared['profile']['name'])} · 해석 결과</h1><p>{esc(prepared['settings']['task'])} · {esc(prepared['phase'])} · {esc(summary.get('solver',''))}</p>
<p class="note">계산 결과는 가정 물성을 사용한 시험 모델의 예측입니다. 계산 완료는 수치 수렴이나 실제 기체의 안전성 판정이 아닙니다.</p>
<p><a href="request.json">실행 조건</a> · <a href="project.json">실행 당시 배치</a> · <a href="solver.log">계산 로그</a></p><table>{rows}</table>
<p>{esc(prepared['mapping']['sensor_start'])}</p><div class="scroll">{mode_table}</div>'''
    quality=summary.get('aero_quality')
    if quality:
        text+='<h2>복합 공력표 · 출처와 차이</h2><p>'+esc(quality['composition'])+' · flow5 '+esc(quality['flow5_speed_m_s'])+' m/s</p>'
        text+='<table><tr><th>기울기 / °</th><th>AVL</th><th>flow5</th><th>상대 차이</th></tr>'
        for key,row in quality['consistency'].items():
            difference='두 기울기 모두 0 근처' if row['relative_difference'] is None else f"{100*row['relative_difference']:.1f}%"
            if row['warning']:difference+=' · 20% 초과, 확인 필요'
            text+=f"<tr><th>{esc(key)}</th><td>{row['avl']:.6g}</td><td>{row['flow5']:.6g}</td><td>{esc(difference)}</td></tr>"
        text+='</table><p>차이의 분모는 두 기울기의 절댓값 중 큰 값입니다. 0°에 가장 가까운 계산점에서 비교하며, 경계에서는 한쪽 차분을 씁니다. 시험 정확도 판정이 아닙니다.</p>'
        text+='<ul>'+''.join('<li>'+esc(note)+'</li>' for note in quality['limitations'])+'</ul>'
        text+='<p><a href="aero_metadata.json">원본 경로·해시·선택하지 않은 블록·비교 조건</a></p>'
    if (folder/'history.html').is_file():text+='<p><a href="history.html">응답 그래프 열기</a> · <a href="simulation/timeseries.csv">시계열 CSV</a></p><iframe title="응답 그래프" src="history.html"></iframe>'
    if (folder/'replay.json').is_file():text+=f'<p><a href="/analysis-replay.html?job={folder.name}">실제 형상으로 3D 결과 재생</a></p>'
    (folder/'report.html').write_text(text+'</html>',encoding='utf8')


def run(folder):
    from dbf_stability.compute_resources import apply_affinity
    apply_affinity()
    prepared=json.loads((folder/'request.json').read_text('utf8'));c=prepared['config'];s=prepared['settings'];task=s['task'];phase=prepared['phase']
    stage(folder,'aero','실제 공력표를 준비합니다.')
    if s['rebuild'] or task=='aero':
        db=build_aero_database(c,folder/'aero_database.npz',workers=s['workers'])
    else:db=AeroDatabase(prepared['aero_path'])
    db.assert_compatible(c)
    write_json(folder/'aero_metadata.json',db.metadata)
    summary={'solver':db.metadata['solver'],'task':task,'phase':phase,'model_name':prepared['profile']['name'],
             'model_id':prepared['profile']['id'],'numerically_converged':False,'physical_validation':'unvalidated_assumption_case'}
    if db.metadata.get('backend')=='hybrid':summary['aero_quality']={key:db.metadata[key] for key in ('backend','composition','sources','flow5_speed_m_s','consistency','limitations','discarded')}
    if 'aero_quality' in summary:
        record=json.loads((folder/'job.json').read_text('utf8'))
        record['result']={'solver':summary['solver'],'aero_quality':summary['aero_quality'],'numerically_converged':False}
        write_json(folder/'job.json',record)
    if c['sensor'].get('model')=='point_mass':
        summary.update(payload_model='point_mass',payload_mass_kg=c['sensor']['mass_kg'],
                       sensor_aerodynamics='not_modelled',geometry_contact='not_modelled')
    modes=None;finished='completed'
    if task=='aero':
        summary.update(conditions=int(np.prod([len(a) for a in db.axes])),status='completed')
    else:
        scene_start=task=='response' and s['start']=='scene' and phase=='deployed'
        trim_phase='aircraft_only' if scene_start else 'stowed' if phase=='mission' else 'deployed' if phase=='recovery' else phase
        stage(folder,'trim','힘·모멘트 평형을 계산합니다.')
        equilibrium_config=copy.deepcopy(c)
        if task in ('flight','recovery','sequence'):
            equilibrium_config['flight']['controller']['enabled']=False
            equilibrium_config['flight']['gust']=None
            equilibrium_config['flight']['controls']={}
        trim=solve_trim(equilibrium_config,db,mode=trim_phase)
        trim_record={k:v for k,v in trim.items() if k!='state'}
        write_json(folder/'trim.json',trim_record);np.save(folder/'trim_state.npy',trim['state'])
        summary.update(trim_record)
        if task in ('stability','flight'):
            stage(folder,'stability','평형 주위의 고유값과 감쇠를 계산합니다.')
            result=analyze_stability(equilibrium_config,db,trim);modes=result['modes'];modes.to_csv(folder/'modes.csv',index=False)
            np.save(folder/'state_matrix.npy',result['matrix'])
            summary.update({k:v for k,v in result.items() if k not in ('matrix','eigenvalues','modes','trim')})
            summary['mode_count']=len(modes)
            write_json(folder/'stability_quality.json',{k:v for k,v in summary.items()
                       if k in ('unstable','stability_status','max_real_eigenvalue_1_s','growth_time_s',
                                'near_neutral_modes','eigenvalue_tolerance_1_s','linearization_verified','linearization_step','note')})
        if task in ('response','mission','flight','recovery','sequence'):
            stage(folder,'simulation','운동 방정식을 적분합니다. 수용된 시간만 진행률에 반영합니다.')
            initial=None
            if scene_start:
                initial=initial_from_scene(prepared,trim)
                trim={**trim,'mode':'deployed','length_m':c['cable']['length_m'],'state':initial}
            elif task=='recovery':
                initial=trim['state'].copy()
            elif task in ('response','flight'):
                initial=trim['state'].copy()
                from dbf_stability.math3d import rotation
                q=(Rotation.from_matrix(rotation(initial[6:10]))*Rotation.from_euler('y',s['pitch_delta'],degrees=True)).as_quat()
                initial[6:10]=np.r_[q[3],q[:3]]
                if phase=='deployed' and c['sensor'].get('model')!='point_mass':
                    q=(Rotation.from_matrix(rotation(initial[19:23]))*Rotation.from_euler('z',s['sensor_yaw_delta'],degrees=True)).as_quat()
                    initial[19:23]=np.r_[q[3],q[:3]]
            result=simulate(c,db,trim=trim,phase=phase,initial_state=initial,duration=c['simulation']['duration_s'],output=folder/'simulation')
            if task=='sequence':
                from mission_sequence import stage_at
                result.table['mission_stage']=[stage_at(t,prepared['schedule']) for t in result.time]
                result.table.to_csv(folder/'simulation/timeseries.csv',index=False)
                summary['sequence_schedule']=prepared['schedule']
            summary.update(result.summary);summary['solver']=db.metadata['solver']
            if task=='flight':summary['stability_scope']='uncontrolled fixed-length equilibrium; controls and gusts apply to time response only'
            if task in ('recovery','sequence'):
                final_length=float(result.table.length_m.iloc[-1])
                summary.update(recovery_target_m=s['recovery_length'],final_length_m=final_length,
                               recovery_completed=result.summary['status']=='completed' and abs(final_length-s['recovery_length'])<1e-8)
            finished='completed' if result.summary['status']=='completed' else 'partial'
            stage(folder,'report','계산한 구간의 그래프와 3D 재생을 저장합니다.')
            from dbf_stability.plots import history_figure
            history_figure(result).write_html(folder/'history.html',include_plotlyjs=True)
            replay_data(result,json.loads((folder/'project.json').read_text('utf8')),prepared,folder)
        summary.setdefault('status','completed')
    write_json(folder/'result.json',summary);report(folder,prepared,summary,modes)
    r=json.loads((folder/'job.json').read_text('utf8'));r.update(state=finished,stage='done',finished=time.time(),
        message='계산이 완료됐습니다. 결과와 적용 범위를 확인하세요.' if finished=='completed' else '요청 시간 전에 계산이 멈췄습니다. 계산된 구간과 중단 원인을 확인하세요.',
        result={k:v for k,v in summary.items() if not isinstance(v,(dict,list)) or k=='aero_quality'})
    write_json(folder/'job.json',r)


if __name__=='__main__':
    folder=Path(sys.argv[1]).resolve()
    if sys.stdin.readline().strip()!='go':raise SystemExit('Missing process gate')
    try:run(folder)
    except Exception as exc:
        traceback.print_exc()
        message=str(exc)
        if message.startswith('TRIM_INFEASIBLE'):
            message='평형 조건을 충족하지 못했습니다. 공력표 범위와 견인선·포획부 접촉을 확인하세요. 비접촉 평형을 구하지 못한 경우 고유값은 계산하지 않습니다. '+message
        r=json.loads((folder/'job.json').read_text('utf8'));r.update(state='failed',finished=time.time(),message=message)
        write_json(folder/'job.json',r)
        raise SystemExit(1)
