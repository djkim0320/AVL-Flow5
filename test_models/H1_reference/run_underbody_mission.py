"""Fresh 270-degree mission; capture-triggered cleanup without state resets."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse, copy, hashlib, json, shutil, subprocess, sys
import numpy as np
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
from compute_resources import apply_affinity


def main():
    resources=apply_affinity()
    p=argparse.ArgumentParser()
    p.add_argument('--case',type=Path,default=ROOT/'examples/h1_underbody_270_v2.yaml')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--aero',type=Path,required=True,help='Explicit real-solver database; coarse legacy data are not an implicit default')
    p.add_argument('--trim-validation',type=Path,help='Independent holdout for changed geometry/CG')
    p.add_argument('--audit',type=Path,default=HERE/'runs/door_underbody_03/audit.json')
    p.add_argument('--recovery-reference',type=Path,help='Explicit changed-hardware experiment from first saved reference state; not a full mission')
    p.add_argument('--recovery-start',type=float,help='Select the nearest actual saved reference time, before entry contact')
    p.add_argument('--command-reference',type=Path,help='Copy only the previously verified winch/door histories; initial state still comes from time-zero trim')
    p.add_argument('--jacobian-workers',type=int,default=len(resources['logical_processors'])-1)
    args=p.parse_args()
    if not 1<=args.jacobian_workers<len(resources['logical_processors']):
        raise ValueError('Reserve one logical processor for ordered integration')
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    # Immutable per-run solver, including current tested capture event boundary.
    archive=out/'solver';shutil.copytree(ROOT/'src/dbf_stability',archive/'dbf_stability',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    sys.path.insert(0,str(archive))
    from dbf_stability import load_case,AeroDatabase,solve_trim,simulate
    from resume_recovery_case import join
    from run_captured_cleanup import cleanup_schedule
    def record(stage,**kw):
        value=dict(stage=stage,**kw);tmp=out/'pipeline.tmp'
        tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');tmp.replace(out/'pipeline.json')
        print(json.dumps(value),flush=True)
    page=r'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>270° 도어 · 해석 진행</title>
<style>body{font-family:system-ui,sans-serif;max-width:1000px;margin:40px auto;padding:20px;line-height:1.8;color:#23353e}a{color:#136e9a}pre{white-space:pre-wrap;background:#eef3f5;padding:20px}</style>
<h1>270° 도어 · 전개와 회수 해석</h1><p>100m · 25m/s · 와이어 2.7m · 전개 유지 60초. 실제 flow5 공력 데이터와 수정한 CAD를 사용합니다.</p>
<p id="delivery"></p><script>async function delivery(){try{let c=await(await fetch('continuation.json',{cache:'no-store'})).json();if(c.progress_href){location.replace(c.progress_href);return}let r=await(await fetch('completion_report.json',{cache:'no-store'})).json();if(r.report_href){let a=document.createElement('a');a.href=r.report_href;a.textContent='전체 해석 결과 보기';document.querySelector('#delivery').replaceChildren(a)}}catch(e){}}delivery();setInterval(delivery,5000)</script>
<p>형상: 102×56×2mm 문판, 출구 바닥 아래 30mm 힌지, 핀·브래킷·받침 추가. 문 움직임은 지정한 각도를 따릅니다. 서보 응답과 문 주변 유동은 검증하지 않았습니다.</p>
<pre id="metrics">저장된 비행 상태 확인 중</pre>
<script>async function metrics(){try{let r=await fetch('live_metrics.json',{cache:'no-store'});if(!r.ok)return;let d=await r.json();let phases={stowed:'수납',internal:'출구 통과',payout:'줄 전개',tow:'전개 유지',recovery:'회수',captured:'포획'};document.querySelector('#metrics').textContent=(phases[d.phase]||d.phase)+' · '+d.time_s.toFixed(3)+'초\n고도 '+d.altitude_m.toFixed(3)+'m · 속도 '+d.airspeed_m_s.toFixed(3)+'m/s\n줄 '+d.length_m.toFixed(3)+'m / 2.700m · 장력 '+d.tension_N.toFixed(3)+'N\n문 '+d.door_deg.toFixed(1)+'° · 접촉력 '+d.contact_N.toFixed(3)+'N\n최근 저장 상태의 값이며, 전체 구간의 최댓값은 완료 후 별도 집계합니다.'}catch(e){}}metrics();setInterval(metrics,5000)</script>
<pre id="status">상태 확인 중</pre><p id="links"></p><p>진행 중인 시각은 적분기가 계산을 받아들여 저장한 상태만 표시합니다. 완료 표시는 계산과 CAD 검사를 마친 뒤 나타납니다.</p>
<script>async function refresh(){try{let p=await(await fetch('pipeline.json',{cache:'no-store'})).json();let a={};if(p.live_directory){let r=await fetch(p.live_directory+'/accepted_progress.json',{cache:'no-store'});if(r.ok){a=await r.json();let modified=Date.parse(r.headers.get('Last-Modified'));a.stale=Number.isFinite(modified)&&Date.now()-modified>90000}};let names={trim:'트림 계산',deployment:'문 개방·전개·견인·회수',settling:'포획 후 안정화',cleanup:'내부 줄 정리·문 닫힘',audit:'저장 궤적 CAD 검사',completed:'완료',partial:'계산 중단 · 결과 확인 필요',failed:'처리 오류'};document.querySelector('#status').textContent=(a.stale?'최근 저장 갱신이 멈췄습니다. 계산 프로세스 확인이 필요합니다.':(names[p.stage]||p.stage))+'\n'+(a.time_s!==undefined?'계산된 시각: '+a.time_s.toFixed(3)+'초\n':'')+(a.captured?'포획 감지됨\n':'')+(p.message||'');if(p.result_directory && ['completed','partial'].includes(p.stage)){document.querySelector('#links').innerHTML='<a href="'+p.result_directory+'/H1_replay.html">실제 계산 재생</a> · <a href="'+p.result_directory+'/history.html">결과 그래프</a> · <a href="'+p.result_directory+'/summary.json">결과 수치</a>'}}catch(e){document.querySelector('#status').textContent='진행 기록을 읽는 중입니다.'}}refresh();setInterval(refresh,5000)</script></html>'''
    (out/'progress.html').write_text(page,encoding='utf8')
    (out/'continuation.json').write_text(json.dumps({'progress_href':None}),encoding='utf8')
    (out/'completion_report.json').write_text(json.dumps({'report_href':None}),encoding='utf8')
    try:
        c=load_case(args.case)
        geometry_note=(f"형상: 입구 {c['bay']['half_width_m']*2000:.0f}×{(c['bay']['floor_z_m']-c['bay']['ceiling_z_m'])*1000:.0f}mm, "
                       f"문판 {c['bay']['door_length_m']*1000:.0f}×{c['bay']['half_width_m']*2000:.0f}mm, 270° 접힘. "
                       "문 각도는 지정한 이력을 따르며 서보 응답과 국소 유동은 검증하지 않았습니다.")
        page=page.replace('형상: 102×56×2mm 문판, 출구 바닥 아래 30mm 힌지, 핀·브래킷·받침 추가. 문 움직임은 지정한 각도를 따릅니다. 서보 응답과 문 주변 유동은 검증하지 않았습니다.',geometry_note)
        (out/'progress.html').write_text(page,encoding='utf8')
        audit=json.loads(args.audit.read_text(encoding='utf8'))
        if audit['intersecting_samples'] or audit['query_error_samples'] or audit['angle_step_deg']>1:
            raise ValueError('270 degree CAD sweep must pass at <=1 degree spacing')
        if audit['source_case_sha256']!=hashlib.sha256(args.case.read_bytes()).hexdigest():
            raise ValueError('Stale door audit')
        cad=(ROOT/c['collision']['mesh_directory']).parent/'cad'
        if audit['cad_sha256']!={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in cad.glob('*.step')}:
            raise ValueError('CAD changed since door audit')
        c['collision']['mesh_directory']=str((ROOT/c['collision']['mesh_directory']).resolve())
        c['simulation']['maximum_runtime_s']=7200.
        c['simulation']['jacobian_workers']=args.jacobian_workers
        c['simulation']['jacobian_method']='grouped'
        c['provenance']['collision']['source']+=' Cable mesh forces share load across nearby triangle witnesses with compact continuous weights; avoids a jumping single closest-face normal in a polygonal bore. Complete CAD gap guards and contact constants retained. This numerical contact regularisation is not calibrated against impact tests.'
        (out/'integration_resources.json').write_text(json.dumps(dict(
            logical_processors=resources['logical_processors'],reserved_logical_processors=resources['reserved_logical_processors'],
            jacobian_workers=args.jacobian_workers,main_integrator_processes=1,blas_threads_per_process=1,
            method='Process-parallel independent RHS columns; ordered Radau steps and events'),indent=2),encoding='utf8')
        (out/'case.yaml').write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
        a=AeroDatabase(args.aero)
        check_path=args.trim_validation or args.aero.parent/'trim_validation.json'
        validation=json.loads(check_path.read_text(encoding='utf8'))
        if not validation['passed']:
            raise ValueError('Independent actual flow5 trim check must pass')
        if validation.get('aero_sha256')!=hashlib.sha256(args.aero.read_bytes()).hexdigest():
            raise ValueError('Trim validation is not bound to this database')
        if validation.get('case_sha256') and validation['case_sha256']!=hashlib.sha256(args.case.read_bytes()).hexdigest():
            raise ValueError('Trim validation belongs to another mechanical case')
        if a.metadata['geometry_sha256']!=hashlib.sha256((ROOT/c['aero']['geometry']).read_bytes()).hexdigest():
            raise ValueError('Aerodynamic database geometry differs from this case')
        for key,axis in zip(('alpha_deg','beta_deg','elevator_deg'),a.axes):
            c['aero'][key]=axis.tolist()
        c['provenance']['aero']['source']='Refined actual flow5 7.57 VLM2 lifting-surface database, independent trim holdout passed. See source.aero_path and trim_validation. Door/bracket local flow and drag remain unresolved.'
        (out/'case.yaml').write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
        start=0.;initial=None
        scope='수정된 270° 문 형상의 새 수납 트림에서 0초부터 계산.'
        source=dict(initial_condition='New stowed trim; fresh time-zero integration',aero_path=str(a.path),
            aero_sha256=hashlib.sha256(Path(a.path).read_bytes()).hexdigest(),
            trim_validation=str(check_path.resolve()),trim_validation_sha256=hashlib.sha256(check_path.read_bytes()).hexdigest(),
            door_audit=str(args.audit.resolve()),case_sha256=hashlib.sha256(args.case.read_bytes()).hexdigest(),
            solver_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (archive/'dbf_stability').glob('*.py')})
        if args.command_reference:
            if args.recovery_reference:raise ValueError('Command-only and state reference cannot be combined')
            ref=yaml.safe_load(args.command_reference.read_text(encoding='utf8'))
            c['winch']=copy.deepcopy(ref['winch'])
            c['mission_profile']=copy.deepcopy(ref['mission_profile'])
            rows=np.asarray(c['winch']['length_schedule'])
            full=np.flatnonzero(rows[:,1]==c['cable']['length_m'])
            if len(full)<2 or not np.isclose(rows[full[-1],0]-rows[full[0],0],60.):
                raise ValueError('Command reference must retain 60 second full payout hold')
            if max(v for _,v in c['winch']['door_schedule'])!=270.:raise ValueError('270 degree door required')
            c['mission_profile']['retrieval_complete_s']=float(rows[np.flatnonzero((rows[:,0]>c['winch']['recovery_start_s']) & (rows[:,1]<=c['winch']['stowed_length_m']+1e-12))[0],0])
            c['simulation']['duration_s']=c['mission_profile']['end_s']
            c['provenance']['winch']['source']+=' Uses the verified slow final approach command history; no saved mechanical state imported.'
            source.update(command_reference=str(args.command_reference.resolve()),
                command_reference_sha256=hashlib.sha256(args.command_reference.read_bytes()).hexdigest())
            (out/'case.yaml').write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
        if args.recovery_reference:
            from dbf_stability.analysis import load_result
            from run_capture_stop import map_reference
            from dbf_stability.model import schedule
            previous=load_result(args.recovery_reference)
            index=0 if args.recovery_start is None else int(np.argmin(abs(previous.time-args.recovery_start)))
            if bool(previous.table.captured.iloc[index]):raise ValueError('Reference must start before capture')
            start=float(previous.time[index]);initial,shifts=map_reference(previous.states[index],previous.config,c)
            # Retain the actual approach commands from the reference, before its
            # manual post-capture cleanup. Never schedule cleanup before new capture.
            previous_winch=previous.summary.get('command_change_source',{}).get('old_winch',previous.config['winch'])
            c['winch']=copy.deepcopy(previous_winch)
            old_angle=max(v for _,v in c['winch']['door_schedule'])
            c['winch']['door_schedule']=[[t,v*270/old_angle] for t,v in c['winch']['door_schedule']]
            c['mission_profile']['end_s']=max(c['winch']['length_schedule'][-1][0],c['winch']['door_schedule'][-1][0])
            c['simulation']['duration_s']=c['mission_profile']['end_s']
            np.testing.assert_allclose(schedule(c['winch']['length_schedule'],start),
                schedule(previous.config['winch']['length_schedule'],start),atol=1e-12,rtol=0)
            scope=f'{start:.3f}초의 실제 저장 회수 자세에 수정 도어·추가 질량을 적용한 별도 비교 계산. 새 형상의 전개 구간은 포함하지 않음.'
            source.update(initial_condition='Changed-hardware recovery comparison from actual saved state',
                reference=str(args.recovery_reference.resolve()),start_time_s=start,cg_mapping_m=shifts,
                source_row=index,source_state_sha256=hashlib.sha256(previous.states[index].tobytes()).hexdigest(),
                approach_commands='Reference pre-cleanup old_winch; door angle scaled to 270 degrees',
                reference_states_sha256=hashlib.sha256((args.recovery_reference/'states.npz').read_bytes()).hexdigest(),
                initial_state_sha256=hashlib.sha256(initial.tobytes()).hexdigest())
            (out/'progress.html').write_text(page.replace('<h1>270° 도어 · 전개와 회수 해석</h1>',
                '<h1>270° 도어 · 회수 구간 비교</h1><p>'+scope+'</p>'),encoding='utf8')
            (out/'case.yaml').write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
        (out/'source.json').write_text(json.dumps(source,indent=2),encoding='utf8')
        record('trim');trim=solve_trim(c,a,mode='stowed')
        if initial is not None:
            from dbf_stability.model import CoupledModel
            probe=CoupledModel(c,a,trim)
            gap=probe.clearance_metric(start,initial)+c['collision']['minimum_gap_m']
            if probe._mesh_contacts:probe._mesh_contacts.close()
            if gap<=c['collision']['minimum_gap_m']:raise ValueError(f'Changed-geometry initial pose is obstructed: {gap} m')
        (out/'trim.json').write_text(json.dumps({k:v for k,v in trim.items() if k!='state'},indent=2),encoding='utf8')
        record('deployment',live_directory='mission')
        first=simulate(c,a,trim,phase='mission',output=out/'mission',stop_on_capture=True,
            initial_state=initial,start_time=start,duration=c['mission_profile']['end_s']-start)
        combined=first;paths=[out/'mission']
        if first.summary['status']=='capture_event':
            t=float(first.time[-1]);capture_time=first.summary['capture_time_s']
            record('settling',live_directory='settling',capture_time_s=capture_time)
            settle=simulate(c,a,trim,phase='mission',initial_state=first.states[-1],start_time=t,duration=.3,
                            output=out/'settling',initial_capture_time=capture_time)
            paths.append(out/'settling');combined=join(first,settle,paths)
            if settle.summary['status']=='completed':
                cleanup=copy.deepcopy(c);end,_=cleanup_schedule(cleanup,float(settle.time[-1]),.25,.2)
                (out/'cleanup_case.yaml').write_text(yaml.safe_dump({k:v for k,v in cleanup.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
                record('cleanup',live_directory='cleanup',capture_time_s=capture_time)
                last=simulate(cleanup,a,trim,phase='mission',initial_state=settle.states[-1],start_time=float(settle.time[-1]),
                    duration=end-float(settle.time[-1]),output=out/'cleanup',initial_capture_time=capture_time)
                paths.append(out/'cleanup');combined=join(combined,last,paths)
        combined.summary.update(scope=scope+' 포획을 실제로 감지한 경우에만 0.3초 후 내부 줄 정리 명령으로 전환. 모든 단계의 연결 상태는 정확히 동일합니다.',
            requested_duration_s=combined.config['mission_profile']['end_s']-start,source=source,
            door_kinematics='Prescribed hinge; updated closed-pose mass properties. Door inertia variation, actuator dynamics and local door drag not resolved.')
        combined.save(out/'complete')
        from dbf_stability.plots import history_figure
        history_figure(combined).write_html(out/'complete/history.html',include_plotlyjs=True)
        record('audit',result_directory='complete',status=combined.summary['status'])
        subprocess.run([sys.executable,str(HERE/'audit_trajectory_collisions.py'),str(out/'complete'),'--workers','2','--time-step','.2'],cwd=ROOT,check=True)
        subprocess.run([sys.executable,str(HERE/'visualize_run.py'),'--run',str(out/'complete')],cwd=ROOT,check=True)
        checked=json.loads((out/'complete/cad_collision_audit.json').read_text(encoding='utf8'))
        success=combined.summary['status']=='completed' and combined.summary['captured'] and abs(combined.summary['final_door_deg'])<1e-6 and not checked['intersecting_samples'] and not checked['cad_query_error_samples']
        record('completed' if success else 'partial',result_directory='complete',status=combined.summary['status'],
            captured=combined.summary['captured'],end_time_s=float(combined.time[-1]),
            final_door_deg=combined.summary['final_door_deg'],message=combined.summary['message'])
    except Exception as exc:
        record('failed',message=str(exc));raise


if __name__=='__main__':main()
