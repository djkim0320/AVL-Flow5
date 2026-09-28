"""Resume an interrupted captured window with identical physics and saved state."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,copy,hashlib,json,shutil,subprocess,sys
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();source=args.source.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(source/'solver',out/'solver',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    sys.path.insert(0,str(out/'solver'))
    from dbf_stability import AeroDatabase,resume_simulation
    from dbf_stability.analysis import load_result
    from dbf_stability.plots import history_figure
    from resume_recovery_case import join
    first=load_result(source/'complete')
    if first.summary['status']!='runtime_limit' or not first.summary['captured']:
        raise ValueError('This continuation requires a captured runtime-limited result')
    c=copy.deepcopy(first.config);c['simulation']['maximum_runtime_s']=3600.
    aero=AeroDatabase(first.summary['source']['aero_path'])
    if hashlib.sha256(Path(aero.path).read_bytes()).hexdigest()!=first.summary['source']['aero_sha256']:
        raise ValueError('Aerodynamic database changed')
    checkpoint=Path(first.summary['continuation_parts'][-1]['path'])/'accepted_checkpoint.npz'
    # The source process has ended; its final checkpoint is no longer live.
    with np.load(checkpoint,allow_pickle=False) as d:
        np.testing.assert_array_equal(d['state'],first.states[-1])
        assert float(d['time'])==float(first.time[-1])
    end=c['mission_profile']['end_s'];start=float(first.time[-1])
    def record(stage,**kw):
        value=dict(stage=stage,**kw);tmp=out/'pipeline.tmp'
        tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');tmp.replace(out/'pipeline.json')
        print(json.dumps(value),flush=True)
    (out/'progress.html').write_text((source/'progress.html').read_text(encoding='utf8'),encoding='utf8')
    provenance=dict(parent=str(source),checkpoint=str(checkpoint),
        checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),start_s=start,end_s=end,
        reason='Continue the saved runtime-limited state after an extended wall-time gap; no physical inputs changed.')
    (out/'continuation_source.json').write_text(json.dumps(provenance,indent=2),encoding='utf8')
    record('cleanup',live_directory='continuation',message=f'{start:.6f}초 저장 상태에서 이어 계산합니다. 물리 입력과 계산 코드가 같은지 확인했습니다.')
    try:
        second=resume_simulation(c,aero,checkpoint,duration=end-start,output=out/'continuation')
        paths=[Path(v['path']) for v in first.summary['continuation_parts']]+[out/'continuation']
        result=join(first,second,paths)
        result.summary.update(source=first.summary['source'],door_kinematics=first.summary['door_kinematics'],
            scope=first.summary['scope']+' 실행 시간 한도에서 저장된 상태를 물리 입력·소스 일치 검사 후 그대로 이어 계산했습니다.',
            resume_provenance=provenance,
            runtime_note='Reported wall runtime includes the extended pause in the source run; it is not active CPU time.')
        result.save(out/'complete')
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
