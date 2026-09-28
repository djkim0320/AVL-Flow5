"""Verify the delivered repaired trajectory against saved evidence."""
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as ET
import numpy as np
from dbf_stability.analysis import load_result
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
out=HERE/'runs/recovery_repair_16/captured_cleanup/complete';r=load_result(out)
audit=json.loads((out/'cad_collision_audit.json').read_text(encoding='utf8'))
assert r.summary['status']=='completed' and r.summary['captured'] and r.summary['final_door_deg']==0.
assert r.summary['geometry_validity']=='no_overlap_at_checked_samples'
assert audit['intersecting_samples']==0 and audit['cad_query_error_samples']==0
assert not r.summary['violations']
assert np.isfinite(r.states).all() and np.isfinite(r.table.select_dtypes(include=[np.number]).to_numpy()).all()
assert np.all(np.diff(r.time)>=0)
assert r.summary['junction']['all_components_exactly_equal']
mass=r.config['aircraft']['mass_kg']+r.config['sensor']['mass_kg']+r.config['cable']['density_kg_m']*r.config['cable']['length_m']
np.testing.assert_allclose(r.table.total_mass_kg,mass,atol=1e-12,rtol=0)
tests=[]
for name in ['fast_shielded_tests.xml','widened_bore_geometry_tests.xml','cleanup_command_tests.xml']:
    path=ROOT/'outputs/validation'/name;tree=ET.parse(path)
    cases=tree.findall('.//testcase');assert all(c.find('failure') is None and c.find('error') is None for c in cases)
    tests.extend((c.attrib.get('classname'),c.attrib['name']) for c in cases)
assert len(tests)==len(set(tests))==44
last=r.table.iloc[-1]
result=dict(status='completed_recovery_window',start_s=float(r.time[0]),end_s=float(r.time[-1]),captured=True,
    capture_time_s=r.summary['capture_time_s'],final_door_deg=0.,final_altitude_m=float(last.altitude_m),final_airspeed_m_s=float(last.airspeed_m_s),
    max_tension_N=r.summary['max_tension_N'],max_contact_N_unvalidated=r.summary['max_contact_N'],
    max_winch_torque_Nm=float(r.table.torque_Nm.abs().max()),max_winch_power_W=float(r.table.winch_power_W.abs().max()),
    specified_limit_violations=[],total_mass_kg=mass,states=len(r.time),cad_samples=audit['samples'],cad_overlap_samples=0,cad_query_errors=0,
    all_replay_frames_in_cad_audit=True,continuous_CAD_proof=False,tests_passed=44,
    numerically_converged=False,physical_validation='unvalidated_assumption_case',
    scope='Recovery window from 70.812 s recorded state; exact-state join to a changed post-capture reel command. Not a full deployment-to-recovery reintegration.',
    files={name:hashlib.sha256((out/name).read_bytes()).hexdigest() for name in ['states.npz','inputs.json','cad_collision_audit.json']})
(out/'repair_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result),flush=True)
