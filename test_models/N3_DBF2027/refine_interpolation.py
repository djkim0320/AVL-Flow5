"""Refine the real flow5 table where the independent trim check found error."""
import json,shutil,sys,time
from connect_ui import HERE,start
from finish_campaign import wait_job,run

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf8')
    shutil.copytree(HERE/'runs/final',HERE/'runs/before_interpolation_refinement')
    for task in ('flight','recovery'):
        shutil.copy2(HERE/f'ui_flow5_{task}_job.json',HERE/f'ui_flow5_{task}_before_refinement.json')
    grid=dict(alpha_deg=[-4,-2,-1,-.5,0,.5,1,2,4,6,8],beta_deg=[-4,0,4],elevator_deg=[-8,-4,-1,0,.5,1,1.5,2,4,8])
    start('flow5','flight',grid);wait_job('flow5')
    run('postprocess.py','holdout','--backend','flow5')
    run('postprocess.py','cases');run('postprocess.py','compare')
    start('flow5','recovery');wait_job('flow5','recovery')
    run('connect_ui.py','update','--backend','flow5')
    (HERE/'runs/final/pipeline_complete.json').write_text(json.dumps({'completed':True,'time':time.time(),'flow5_grid_refined':True}),encoding='utf8')
