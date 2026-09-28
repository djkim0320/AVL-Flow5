"""Run the requested 1.5-span / 60-second hold with actual flow5 aerodynamics."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'): os.environ[key]='1'
from pathlib import Path
import hashlib
import json
from dbf_stability import load_case, AeroDatabase, solve_trim, simulate
from dbf_stability.plots import history_figure

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent

if __name__=='__main__':
    out=HERE/'runs/span_hold_01'
    c=load_case(ROOT/'examples/h1_normal_r3_flow5_span_hold.yaml')
    a=AeroDatabase(HERE/'runs/flow5_connection_01/aerodynamics/aero_database.npz')
    (out/'source_versions.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/dbf_stability').glob('*.py')},indent=2),encoding='utf8')
    trim=solve_trim(c,a,mode='stowed')
    (out/'trim.json').write_text(json.dumps({k:v for k,v in trim.items() if k!='state'},indent=2),encoding='utf8')
    r=simulate(c,a,trim,phase='mission',output=out/'mission')
    r.summary['scope']='윙스팬 1.5배의 줄을 전개한 뒤 60초 유지하고 회수하는 명령을 입력한 실제 flow5 결합 계산. 실제 도달 시각과 포획 여부는 계산 상태에 따른다.'
    r.save(out/'mission')
    history_figure(r).write_html(out/'mission/history.html',include_plotlyjs=True)
    print(json.dumps({k:v for k,v in r.summary.items() if k not in ('assumptions','aero_metadata','collision_metadata')},ensure_ascii=False),flush=True)
