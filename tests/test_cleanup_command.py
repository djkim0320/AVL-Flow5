"""Post-latch reel command preserves the tow mission and prior input history."""
from pathlib import Path
import copy,sys
import numpy as np
from dbf_stability import load_case
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'test_models/H1_reference'
sys.path.insert(0,str(HERE))
from run_captured_cleanup import cleanup_schedule
from dbf_stability.model import schedule

def test_cleanup_preserves_prior_history_and_finishes_before_door_closes():
    c=load_case(HERE/'runs/recovery_repair_13/shielded_22/case.yaml');original=copy.deepcopy(c)
    start=71.3477050691505;end,old=cleanup_schedule(c,start,.25,.2)
    times=np.linspace(0,start,1001)
    for key in ('length_schedule','door_schedule'):
        a=np.array(old[key]);b=np.array(c['winch'][key])
        np.testing.assert_allclose(np.interp(times,a[:,0],a[:,1]),np.interp(times,b[:,0],b[:,1]),atol=1e-12,rtol=0)
    line=np.array([row for row in c['winch']['length_schedule'] if row[0]>=start])
    speeds=np.diff(line[:,1])/np.diff(line[:,0])
    assert np.all(speeds<=0) and np.max(abs(speeds))<=.25+1e-12
    assert line[-1,1]==old['length_schedule'][-1][1]
    assert c['mission_profile']['hold_duration_s']==60 and c['mission_profile']['deployed_length_m']==2.7
    assert c['winch']['door_capture_interlock']
    assert c['winch']['door_schedule'][-3][0]>c['mission_profile']['retrieval_complete_s']
    assert end==c['mission_profile']['end_s'] and c['winch']['door_schedule'][-1][1]==0
    for section in ('aircraft','sensor','cable','bay','flight','collision','aero'):assert c[section]==original[section]


def test_late_capture_retains_actual_final_braking_command():
    import json
    fixture=json.loads((ROOT/'tests/data/late_capture_cleanup.json').read_text(encoding='utf8'))
    c=load_case(ROOT/'examples/h1_recovery_funnel_v2.yaml')
    c['winch']=fixture['winch'];c['mission_profile']=fixture['mission_profile'];c['provenance']['winch']=fixture['winch_provenance']
    original=copy.deepcopy(c);start=fixture['start_s']
    end,old=cleanup_schedule(c,start,.25,.2)
    finish=c['mission_profile']['retrieval_complete_s']
    assert c['mission_profile']['post_capture_cleanup_mode']=='retained_final_recovery'
    assert 0<schedule(old['length_schedule'],start)[0]-old['length_schedule'][-1][1]<.0002
    for t in np.linspace(0,finish,1001):
        np.testing.assert_allclose(schedule(c['winch']['length_schedule'],t),schedule(old['length_schedule'],t),atol=1e-12,rtol=0)
    assert finish>start and c['winch']['door_schedule'][-3][0]==finish+.2
    assert c['winch']['door_schedule'][-1]==(end,0.)
    for section in ('aircraft','sensor','cable','bay','flight','collision','aero'):assert c[section]==original[section]


def test_already_reeled_line_only_schedules_door():
    c=load_case(HERE/'runs/recovery_repair_13/shielded_22/case.yaml')
    start=c['winch']['length_schedule'][-1][0]+1
    end,_=cleanup_schedule(c,start)
    assert c['mission_profile']['post_capture_cleanup_mode']=='already_reeled'
    assert c['mission_profile']['retrieval_complete_s']==start
    assert np.isclose(end-start,1.3)
    assert all(length==c['winch']['stowed_length_m'] for t,length in c['winch']['length_schedule'] if t>=start)
