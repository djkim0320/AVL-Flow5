"""Strict full-mission continuation through capture, settling and door closure.

Only an exhausted wall-time budget is resumable. Geometry, aerodynamic data,
mechanical state, capture state and the archived solver must match exactly.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,copy,hashlib,json,shutil,subprocess,sys
import numpy as np
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
from compute_resources import apply_affinity


def read_checkpoint(path,config,aero,result):
    from dbf_stability.checkpoint import fingerprints
    with np.load(path,allow_pickle=False) as data:
        meta=json.loads(str(data['metadata']));t=float(data['time']);state=data['state'].copy()
    if meta.get('format')!=1 or meta['fingerprints']!=fingerprints(config,aero):
        raise ValueError('Checkpoint physics, solver or aerodynamic identity mismatch')
    if t!=float(result.time[-1]) or not np.array_equal(state,result.states[-1]):
        raise ValueError('Checkpoint does not match the completed source result')
    if meta['captured']!=result.summary['captured'] or meta['capture_time_s']!=result.summary['capture_time_s']:
        raise ValueError('Checkpoint capture state mismatch')
    return t,state,meta


def main():
    apply_affinity()
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--budget',type=float,default=7200.)
    args=p.parse_args();source=args.source.resolve();out=args.out.resolve()
    if args.budget<=0:raise ValueError('Positive runtime budget required')
    pipeline=json.loads((source/'pipeline.json').read_text(encoding='utf8'))
    if pipeline['stage']!='partial' or pipeline['status']!='runtime_limit':
        raise ValueError('Source pipeline must have ended at its runtime limit')
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(source/'solver',out/'solver',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    sys.path.insert(0,str(out/'solver'))
    from dbf_stability import AeroDatabase,simulate
    from dbf_stability.analysis import load_result
    from dbf_stability.plots import history_figure
    from resume_recovery_case import join
    from run_captured_cleanup import cleanup_schedule
    result=load_result(source/'complete');c=copy.deepcopy(result.config)
    aero=AeroDatabase(result.summary['source']['aero_path'])
    if hashlib.sha256(Path(aero.path).read_bytes()).hexdigest()!=result.summary['source']['aero_sha256']:
        raise ValueError('Aerodynamic database changed')
    c['simulation']['maximum_runtime_s']=args.budget
    paths=[Path(row['path']) for row in result.summary.get('continuation_parts',[])] or [source/'mission']
    checkpoint=paths[-1]/'accepted_checkpoint.npz'
    t,state,meta=read_checkpoint(checkpoint,c,aero,result)
    preserved={key:copy.deepcopy(result.summary[key]) for key in ('source','scope','door_kinematics')}
    (out/'source.json').write_text(json.dumps(preserved['source'],indent=2),encoding='utf8')
    original=yaml.safe_load((source/'case.yaml').read_text(encoding='utf8'))
    stage=('mission' if not meta['captured'] else
           'settling' if c['winch']==original['winch'] else 'cleanup')
    provenance=dict(parent=str(source),start_s=t,checkpoint=str(checkpoint),
        checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        reason='Runtime-only interruption; complete physical/code fingerprints and all state entries verified.')
    (out/'continuation_source.json').write_text(json.dumps(provenance,indent=2),encoding='utf8')
    shutil.copy2(source/'case.yaml',out/'case.yaml')
    shutil.copy2(source/'progress.html',out/'progress.html')
    (out/'continuation.json').write_text(json.dumps({'progress_href':None}),encoding='utf8')
    (out/'completion_report.json').write_text(json.dumps({'report_href':None}),encoding='utf8')
    def record(stage_name,**kw):
        value=dict(stage=stage_name,**kw);tmp=out/'pipeline.tmp'
        tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');tmp.replace(out/'pipeline.json')
        print(json.dumps(value),flush=True)
    number=0
    try:
        while True:
            target=(meta['capture_time_s']+.3 if stage=='settling' else c['mission_profile']['end_s'])
            if target<=t+1e-10:
                if stage!='settling':raise ValueError('No remaining integration interval')
                stage='cleanup';cleanup_schedule(c,t,.25,.2);continue
            directory=out/f'{stage}_{number:02d}';number+=1
            record('deployment' if stage=='mission' else stage,live_directory=directory.name,
                   message=f'{t:.6f}초 저장 상태에서 이어 계산합니다.')
            next_result=simulate(c,aero,trim={**meta['trim'],'state':state},phase='mission',
                initial_state=state,start_time=t,duration=target-t,output=directory,
                initial_capture_time=meta['capture_time_s'] if meta['captured'] else None,
                stop_on_capture=stage=='mission')
            paths.append(directory);result=join(result,next_result,paths)
            result.summary.update(**preserved,continuation_source=provenance,continuation_stage=stage,
                requested_duration_s=c['mission_profile']['end_s']-float(result.time[0]))
            result.save(out/'complete')
            t,state,meta=read_checkpoint(directory/'accepted_checkpoint.npz',c,aero,next_result)
            status=next_result.summary['status']
            if status=='runtime_limit':continue
            if stage=='mission' and status=='capture_event':stage='settling';continue
            if stage=='settling' and status=='completed':
                stage='cleanup';cleanup_schedule(c,t,.25,.2);continue
            break
        history_figure(result).write_html(out/'complete/history.html',include_plotlyjs=True)
        record('audit',result_directory='complete',status=result.summary['status'])
        subprocess.run([sys.executable,str(HERE/'audit_trajectory_collisions.py'),str(out/'complete'),'--workers','2','--time-step','.2'],cwd=ROOT,check=True)
        subprocess.run([sys.executable,str(HERE/'visualize_run.py'),'--run',str(out/'complete')],cwd=ROOT,check=True)
        audit=json.loads((out/'complete/cad_collision_audit.json').read_text(encoding='utf8'))
        passed=result.summary['status']=='completed' and result.summary['captured'] and abs(result.summary['final_door_deg'])<1e-6 and not audit['intersecting_samples'] and not audit['cad_query_error_samples']
        record('completed' if passed else 'partial',result_directory='complete',status=result.summary['status'],
            captured=result.summary['captured'],end_time_s=float(result.time[-1]),final_door_deg=result.summary['final_door_deg'],message=result.summary['message'])
    except Exception as exc:
        record('failed',message=str(exc));raise


if __name__=='__main__':main()
