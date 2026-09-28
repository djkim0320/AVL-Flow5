"""Finish CAD checks and replay as each independent R3 mission ends."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import json
import subprocess
import sys
import time
import os
import numpy as np
from dbf_stability.analysis import load_result
from dbf_stability import AeroDatabase
from dbf_stability.model import CoupledModel

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'runs/normal_r3_01'


def execute(script,*arguments):
    environment={**os.environ,'PYTHONIOENCODING':'utf-8'}
    subprocess.run([sys.executable,str(HERE/script),*map(str,arguments)],cwd=ROOT,
        check=True,env=environment,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))


def finish(name):
    folder=OUT/name/'mission'
    while True:
        if (folder/'failure.log').exists():raise RuntimeError(f'{name}: mission process failed; see failure.log')
        try:
            summary=json.loads((folder/'summary.json').read_text(encoding='utf8'))
        except (FileNotFoundError,json.JSONDecodeError):
            time.sleep(3);continue
        if summary.get('status')=='running':time.sleep(3);continue
        if summary.get('status') in ('cancelled','failed'):
            print(f'{name}: no saved trajectory; retaining recorded failure/cancellation',flush=True)
            return name
        if not (folder/'inputs.json').exists():time.sleep(1);continue
        break
    print(f'{name}: {summary["status"]} at {summary["duration_s"]:.6f} s; starting sampled STEP checks',flush=True)
    execute('audit_trajectory_collisions.py',folder,'--time-step',.01,'--workers',8)
    r=load_result(folder)
    model=CoupledModel(r.config,AeroDatabase(OUT/'aerodynamics/aero_database.npz'))
    model.active_override=int(r.table.active_nodes.iloc[-1])
    hits=model.contact_geometry(float(r.time[-1]),r.states[-1],distance=.02)
    if hits:
        hit=min(hits,key=lambda h:(h.get('geometry_gap',h['gap']),h['gap']))
        nearest=dict(moving=hit['moving'],fixed=hit['fixed'],gap_m=float(hit.get('geometry_gap',hit['gap'])),point_frd_m=hit['point'].tolist())
    else:nearest=None
    (folder/'door_diagnosis.json').write_text(json.dumps(dict(nearest_end_pair=nearest,status=r.summary['status'],
        end_time_s=float(r.time[-1]),scope='Final saved state only'),indent=2),encoding='utf8')
    if model._mesh_contacts is not None:model._mesh_contacts.close()
    execute('visualize_run.py','--run',folder)
    print(f'{name}: CAD audit and fixed-scale replay saved',flush=True)
    return name


def main():
    status=OUT/'finalization.json'
    status.write_text(json.dumps(dict(status='running')),encoding='utf8')
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            for future in as_completed([pool.submit(finish,name) for name in ('nominal','time_refined')]):future.result()
        summaries=[json.loads((OUT/name/'mission/summary.json').read_text(encoding='utf8'))
                   for name in ('nominal','time_refined')]
        incomplete=any(q.get('status') in ('cancelled','failed') for q in summaries)
        execute('summarize_incomplete_normal.py' if incomplete else 'summarize_normal_study.py')
    except Exception as exc:
        status.write_text(json.dumps(dict(status='failed',error=str(exc)),indent=2),encoding='utf8');raise
    status.write_text(json.dumps(dict(status='completed',report=str(OUT/'report.html')),indent=2),encoding='utf8')
    print('R3 report, replay and CAD checks completed.',flush=True)


if __name__=='__main__':main()
