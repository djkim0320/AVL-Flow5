"""Retarget the existing reel command while preserving its segment velocities."""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import numpy as np
import yaml
from dbf_stability import load_case, AeroDatabase
from dbf_stability.config import validate

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def retarget(case, wingspan, ratio=1.5, hold=60.):
    if not all(np.isfinite(v) and v > 0 for v in (wingspan, ratio, hold)):
        raise ValueError('Positive finite span, length ratio and hold time required')
    c = copy.deepcopy(case)
    w = c['winch']
    rows = np.asarray(w['length_schedule'], float)
    old_peak = float(rows[:,1].max())
    start = float(w['release_s'])
    old_full = float(rows[np.isclose(rows[:,1], old_peak, rtol=0, atol=1e-10),0][0])
    old_recovery = float(w['recovery_start_s'])
    old_end = float(rows[(rows[:,0] > old_recovery) & np.isclose(rows[:,1], w['stowed_length_m'], rtol=0, atol=1e-10),0][0])
    approach_length = .42
    approach = np.flatnonzero((rows[:,0]>old_recovery) & np.isclose(rows[:,1],approach_length,rtol=0,atol=1e-9))
    if len(approach) != 1:
        raise ValueError('R3 command must contain the 0.42 m slow approach point')
    old_approach = float(rows[approach[0],0])
    target = float(wingspan*ratio)
    if target <= max(w['stowed_length_m'], approach_length):
        raise ValueError('Target length must exceed the internal approach length')
    payout_scale = (target-w['stowed_length_m'])/(old_peak-w['stowed_length_m'])
    recovery_scale = (target-approach_length)/(old_peak-approach_length)
    full = start+(old_full-start)*payout_scale
    recovery = full+hold
    approach_time = recovery+(old_approach-old_recovery)*recovery_scale
    retrieved = approach_time+old_end-old_approach
    end = retrieved+case['simulation']['duration_s']-old_end
    changed = []
    for t,l in rows:
        if t <= start:
            nt,nl = t,l
        elif t <= old_full:
            nt,nl = start+(t-start)*payout_scale,w['stowed_length_m']+(l-w['stowed_length_m'])*payout_scale
        elif t < old_recovery:
            # Only the plateau boundaries are needed; no fictitious motion.
            continue
        elif t <= old_approach:
            nt,nl = recovery+(t-old_recovery)*recovery_scale,approach_length+(l-approach_length)*recovery_scale
        else:
            nt,nl = approach_time+t-old_approach,l
        if changed and abs(nt-changed[-1][0]) < 1e-10:
            continue
        changed.append([float(nt),float(np.clip(nl,w['stowed_length_m'],target))])
    # Exact plateau endpoints avoid first-hit timing ambiguity.
    for row in changed:
        if abs(row[0]-full)<1e-10 or abs(row[0]-recovery)<1e-10: row[1]=target
    old_cell = case['cable']['length_m']/case['cable']['segments']
    c['cable'].update(length_m=target,segments=math.ceil(target/old_cell-1e-10))
    w.update(length_schedule=changed,recovery_start_s=recovery)
    w['door_schedule']=[[float(t if t<old_recovery else t+retrieved-old_end),a] for t,a in w['door_schedule']]
    c['bay']['release_push_until_s']=full
    c['simulation']['duration_s']=end
    c['name']=f"R3_{c['aero'].get('backend','avl')}_{ratio:g}span_{hold:g}s_hold"
    c['mission_profile']=dict(wingspan_m=wingspan,wire_span_ratio=ratio,deployed_length_m=target,
        length_reference='Unstretched paid-out cable from winch fairlead to sensor tow point; includes the internal bay path.',
        deployment_complete_s=full,hold_duration_s=hold,recovery_start_s=recovery,
        slow_approach_start_s=approach_time,retrieval_complete_s=retrieved,end_s=end,
        speed_policy='Original piecewise reel velocities retained by scaling length and time together; original final approach retained.')
    c['provenance']['cable']['source']=f'{target:g} m = {ratio:g} times {wingspan:g} m reference span, requested by user; {c["cable"]["segments"]} cells retain approximately {old_cell:g} m spacing. Linear density, drag and elastic properties remain unmeasured assumptions.'
    c['provenance']['winch']['source']+=f' User mission: full payout {target:g} m, then hold exactly {hold:g} s before recovery. Existing payout/recovery speeds and final approach retained; door remains capture-interlocked.'
    validate(c)
    return c


def main():
    p=argparse.ArgumentParser();p.add_argument('--ratio',type=float,default=1.5);p.add_argument('--hold',type=float,default=60.)
    args=p.parse_args()
    aero_path=HERE/'runs/flow5_connection_01/aerodynamics/aero_database.npz'
    db=AeroDatabase(aero_path)
    records=[]
    for backend,source in [('flow5','h1_normal_r3_flow5.yaml'),('avl','h1_normal_r3_connected.yaml')]:
        case=load_case(ROOT/'examples'/source)
        geometry=ROOT/case['aero']['geometry']
        if hashlib.sha256(geometry.read_bytes()).hexdigest()!=db.metadata['geometry_sha256']:
            raise ValueError('Aircraft geometry changed: rebuild the aero database before deriving span or reusing forces')
        c=retarget(case,float(db.refs[2]),args.ratio,args.hold)
        target=ROOT/'examples'/f'h1_normal_r3_{backend}_span_hold.yaml'
        c.pop('_root',None);c.pop('_source',None)
        target.write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True),encoding='utf8')
        load_case(target)
        records.append(dict(case=str(target),**c['mission_profile']))
    output=HERE/'runs/span_hold_01';output.mkdir(exist_ok=True)
    record=dict(missions=records,aero_reused=str(aero_path),aero_sha256=hashlib.sha256(aero_path.read_bytes()).hexdigest(),
        reason_aero_reused='Aircraft geometry and aero grid unchanged; cable mass, length and commands change only in coupled dynamics.')
    (output/'mission_plan.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
