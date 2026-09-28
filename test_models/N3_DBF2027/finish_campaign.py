"""Sequential real-solver stages, each capped at the same 12-CPU allocation."""
from pathlib import Path
import json,subprocess,sys,time
from connect_ui import api,HERE,ROOT,start

def wait_job(backend,task='flight'):
    job=json.loads((HERE/f'ui_{backend}_{task}_job.json').read_text('utf8'));last=None
    while True:
        state=api('/api/analysis/jobs/'+job['id'])
        if state['stage']!=last:print(backend,task,state['state'],state['stage'],flush=True);last=state['stage']
        if state['state'] in ('completed','partial'):
            print(backend,task,state['state'],'elapsed',state['elapsed_s'],flush=True);return
        if state['state'] in ('failed','cancelled','interrupted'):raise RuntimeError(state)
        time.sleep(3)

def run(script,*args):
    print('Stage',script,*args,flush=True)
    subprocess.run([sys.executable,str(HERE/script),*args],cwd=ROOT,check=True)

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf8')
    wait_job('avl')
    start('flow5','flight');wait_job('flow5')
    if not (HERE/'runs/flow5_refinement_01/results.json').is_file():run('refine_flow5.py')
    run('postprocess.py','holdout','--backend','avl')
    run('postprocess.py','holdout','--backend','flow5')
    run('postprocess.py','cases')
    run('postprocess.py','compare')
    start('flow5','recovery');wait_job('flow5','recovery')
    run('connect_ui.py','update','--backend','flow5')
    (HERE/'runs/final/pipeline_complete.json').write_text(json.dumps({'completed':True,'time':time.time()}),encoding='utf8')
