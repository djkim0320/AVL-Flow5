"""Hardware initial-state experiment; preserve physical towpoint placement.

The rounded-sensor source is a completed, independently CAD-audited prefix.
Adding a stop changes aircraft mass: this is not an unchanged-input resume.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import yaml
from dbf_stability import load_case,AeroDatabase,simulate,solve_trim
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation,cross,point_state
from dbf_stability.model import CoupledModel
from run_recovery_repair import slowed_retrieval,HERE,ROOT,AERO


def map_reference(y,old,new):
    y=y.copy();shifts={}
    for name,start in (('aircraft',0),('sensor',13)):
        delta=np.asarray(new[name].get('cg_shift_from_r3_body_m',[0.,0.,0.]))-np.asarray(old[name].get('cg_shift_from_r3_body_m',[0.,0.,0.]))
        rigid=y[start:start+13];r=rotation(rigid[6:10]);offset=r@delta
        p0,v0=point_state(rigid,old[name]['tow_point_m'])
        rigid[:3]+=offset;rigid[3:6]+=cross(r@rigid[10:13],offset)
        p1,v1=point_state(rigid,new[name]['tow_point_m'])
        np.testing.assert_allclose(p0,p1,atol=1e-12,rtol=0)
        np.testing.assert_allclose(v0,v1,atol=1e-12,rtol=0)
        shifts[name]=delta.tolist()
    return y,shifts


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--source',type=Path,default=HERE/'runs/recovery_repair_04/round_nominal/mission')
    parser.add_argument('--slow',action='store_true')
    parser.add_argument('--damping',type=float)
    parser.add_argument('--engine',choices=['numba','numba_bounded'],default='numba')
    parser.add_argument('--analytic-stop',action='store_true')
    parser.add_argument('--case',type=Path,default=ROOT/'examples/h1_round_x_capture_stop_v2.yaml')
    args=parser.parse_args()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    def record(stage,**kw):
        data=dict(stage=stage,**kw);p=out/'pipeline.tmp'
        p.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8');p.replace(out/'pipeline.json')
        print(json.dumps(data,ensure_ascii=False),flush=True)
    try:
        previous=load_result(args.source)
        if previous.summary['geometry_validity']!='sampled_clear':
            # Audit uses an explicit evidence label; reject an invalid/unaudited source.
            audit=json.loads((args.source/'cad_collision_audit.json').read_text(encoding='utf8'))
            assert audit['intersecting_samples']==0 and audit['cad_query_error_samples']==0
        start=float(previous.time[-1]);old=previous.states[-1].copy()
        c=load_case(args.case)
        if args.slow:slowed_retrieval(c,start)
        c['collision']['feature_engine']=args.engine
        if args.damping is not None:
            c['collision']['part_materials']={'capture_front_stop':dict(stiffness_N_m=20000.,damping_Ns_m=args.damping)}
            c['provenance']['collision']['source']+=f' Explicit conical-stop damper assumption: {args.damping:g} Ns/m, same 20000 N/m barrier stiffness and 0.8 mm numerical skin. Unmeasured effective pad response; other surfaces unchanged. No strength or shock qualification.'
        if args.analytic_stop:
            from dbf_stability.analytic_contact import sphere_cone_contacts
            from analytic_stop_probe import cone_spec
            spec=cone_spec(c,256);cad=(ROOT/c['collision']['mesh_directory']).parent/'cad'
            spec['cad_sha256']={name:hashlib.sha256((cad/name).read_bytes()).hexdigest() for name in ('sensor_body.step','capture_front_stop.step')}
            c['collision']['analytic_conical_stop']=spec
            c['provenance']['collision']['source']+=' Analytic spherical-cap/conical-seat support planes, 256-angle force quadrature inside the finite analytic domain. Exact curved clearance is added to all existing mesh checks; other surfaces, cable and finite edges retain full mesh contact. Unmeasured force law; CAD distance and angular quadrature checked separately.'
        c['simulation']['maximum_runtime_s']=1800.
        c['provenance']['aero']['source']='Actual flow5 7.57 25 m/s R3 lifting-surface database reused: wing and tail surfaces unchanged. Added capture-stop mass, CG and inertia included; moments translated from original aerodynamic reference. Sensor aerodynamic coefficients and contact material remain unvalidated assumptions.'
        c['name']='R3_conical_stop_'+('slow' if args.slow else 'nominal')
        c['collision']['mesh_directory']=str((ROOT/c['collision']['mesh_directory']).resolve())
        case=out/'case.yaml'
        case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
        c=load_case(case);y,shifts=map_reference(old,previous.config,c)
        for key in ('capture_radius_m','capture_speed_m_s','capture_angle_deg'):
            assert c['bay'][key]==previous.config['bay'][key]
        assert c['collision']['minimum_gap_m']==previous.config['collision']['minimum_gap_m']
        a=AeroDatabase(AERO);trim=solve_trim(c,a,mode='stowed');trim.pop('state')
        model=CoupledModel(c,a,trim,phase='mission')
        gap=model.clearance_metric(start,y)+c['collision']['minimum_gap_m']
        if model._mesh_contacts:model._mesh_contacts.close()
        assert gap>c['collision']['minimum_gap_m'],gap
        source=dict(source=str(args.source.resolve()),time_s=start,aircraft_mass_kg=c['aircraft']['mass_kg'],
            stop_damping_Ns_m=args.damping,contact_feature_engine=args.engine,
            analytic_stop=args.analytic_stop,
            initial_minimum_gap_m=gap,reference_shifts_body_m=shifts,towpoint_position_and_velocity_preserved=True,
            source_state_sha256=hashlib.sha256(old.tobytes()).hexdigest(),initial_state_sha256=hashlib.sha256(y.tobytes()).hexdigest(),
            aero_sha256=hashlib.sha256(AERO.read_bytes()).hexdigest(),
            scope='New 20 g capture-stop hardware experiment from the rounded-sensor saved recovery state. New mass, CG, inertia and stowed trim. Not a strict resume or a full deployment flight.')
        (out/'branch_source.json').write_text(json.dumps(source,ensure_ascii=False,indent=2),encoding='utf8')
        (out/'trim.json').write_text(json.dumps(trim,indent=2),encoding='utf8')
        record('simulation',start_time_s=start,end_time_s=c['mission_profile']['end_s'],initial_gap_m=gap)
        result=simulate(c,a,trim,phase='mission',initial_state=y,start_time=start,
            duration=c['mission_profile']['end_s']-start,output=out/'mission')
        result.summary.update(branch_source=source,scope=f'정지부 추가 회수 시험: {start:.3f}초 저장 자세에서 시작. 20g 추가 질량·CG·관성을 반영하고, 기존 견인점 위치·속도를 보존했습니다. 전개부터 다시 계산한 결과는 아닙니다.')
        result.save(out/'mission')
        record('calculated',status=result.summary['status'],captured=result.summary['captured'],end_time_s=float(result.time[-1]),
            max_tension_N=result.summary['max_tension_N'],max_contact_N=result.summary['max_contact_N'],final_door_deg=result.summary['final_door_deg'])
    except Exception as exc:record('failed',message=str(exc));raise


if __name__=='__main__':main()
