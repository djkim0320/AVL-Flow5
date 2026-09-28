"""Refresh portable tables, source hashes, exact meshes and a delivery inventory."""
from pathlib import Path
import csv, hashlib, json
import numpy as np
from extract_h1 import HERE, SOURCE, read_glb
from build_cad import stl, glb

def main():
    p=json.loads((HERE/'parameters.json').read_text(encoding='utf8'));cg=np.array(p['aircraft_cg_aru_m'])
    parts=read_glb(SOURCE/'GLB/DBF_A_H1.glb','DBF A | H1');exact=[];allparts=[]
    prefixes=('03_Main wing','Main wing outer','Wing tip','Aileron ','04_Tail','Tail tip','Horizontal fixed stabilizer','Elevator','05_Vertical','Rudder')
    for name,(v,f) in parts.items():
        v=(v-cg)*[-1,1,-1];color=[.4,.62,.68,1]
        allparts.append((name,v,f,color))
        if name.startswith(prefixes) and 'cover' not in name.lower():
            safe='source_'+''.join(c if c.isalnum() else '_' for c in name)
            stl(HERE/'meshes'/f'{safe}_FRD_m.stl',v,f);exact.append((name,v,f,color))
    glb(HERE/'meshes/H1_A_original_lifting_surfaces.glb',exact)
    glb(HERE/'meshes/H1_A_original_context_CG.glb',allparts)
    # Native source GLBs have additional scenes; above reader selects only H1-A.
    cv=HERE/'cad_validation.json'
    if cv.exists():
        d=json.loads(cv.read_text(encoding='utf8'));d['preserved_source_mesh_parts']=[r[0] for r in exact]
        cv.write_text(json.dumps(d,indent=2),encoding='utf8')
    rows=[]
    def row(item,value,unit,frame,status,source):
        rows.append(dict(item=item,value=json.dumps(value,ensure_ascii=False),unit=unit,frame=frame,status=status,source=source))
    audit=json.loads((HERE/'source_audit/geometry_measurements.json').read_text(encoding='utf8'))
    for variant,area in [('A',.55),('B',.60)]:
        s=audit['variants'][variant]['wing_section'];t=audit['variants'][variant]['tail_section']
        row(f'{variant}.wing.area_nominal',area,'m2','planform','source_document','copied H1 README')
        row(f'{variant}.wing.span',1.8,'m','ARU','source_mesh','GLB tip extrema')
        for k in ['chord_m','leading_edge_m','incidence_deg']:
            row(f'{variant}.wing.{k}',s[k],'deg' if k=='incidence_deg' else 'm','ARU','source_mesh' if k!='incidence_deg' else 'source_document','GLB section / copied H1 README')
        row(f'{variant}.tail.chord',t['chord_m'],'m','ARU','source_mesh','GLB central tail section')
        row(f'{variant}.tail.span',.70,'m','ARU','source_mesh','GLB tail tips')
    for item,val in [('fin_root_chord',.27),('fin_tip_chord',.13),('fin_height',.30),('aileron_span_each',.3478),('elevator_span_each',.278)]:
        row(item,val,'m','ARU','reconstructed_from_source_mesh','GLB fin/rudder/controls extrema, gaps closed for AVL')
    row('aircraft.mass',p['aircraft_mass_kg'],'kg','aircraft only','assumption','mass_budget.csv: excludes sensor and all line mass')
    row('aircraft.cg',p['aircraft_cg_aru_m'],'m','nose ARU','calculated_from_assumptions','box component budget')
    row('aircraft.inertia',p['aircraft_inertia_frd_kgm2'],'kg m2','CG FRD tensor','calculated_from_assumptions','box inertia plus parallel axis')
    row('sensor.mass',p['sensor_mass_kg'],'kg','sensor only','assumption','36 g cylinder + 4 x 1 g fins')
    row('sensor.cg_offset',p['sensor_geometric_cg_offset_frd_m'],'m','geometric centre FRD','calculated_from_assumptions','cylinder / fin mass budget')
    row('sensor.inertia',p['sensor_inertia_frd_kgm2'],'kg m2','sensor CG FRD tensor','calculated_from_assumptions','solid cylinder + rectangular fin inertia')
    row('cable.total_mass',p['cable_mass_kg'],'kg','separate material cells','assumption','1.5 m x .0015 kg/m')
    row('system.total_mass',p['total_mass_kg'],'kg','all phases','calculated_from_assumptions','aircraft + sensor + line exactly once')
    for item,key in [('bay_front','bay_front_aru_m'),('bay_exit','bay_exit_aru_m'),('bay_width','bay_width_m'),('bay_floor','bay_floor_aru_z_m'),('bay_ceiling','bay_ceiling_aru_z_m')]:
        row(item,p[key],'m','nose ARU','new_temporary_design','build_inputs.py / parameters.json')
    with (HERE/'dimensions_mass_provenance.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    hashes=json.loads((HERE/'source_audit/source_hashes.json').read_text(encoding='utf8'))
    checks={f:hashlib.sha256((SOURCE/f).read_bytes()).hexdigest()==h for f,h in hashes.items()}
    assert all(checks.values())
    avlhash={str(f.relative_to(HERE)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((HERE/'geometry').glob('*')) if f.is_file()}
    assert json.loads((HERE/'aerodynamics/input_hashes.json').read_text(encoding='utf8'))==avlhash
    from dbf_stability import AeroDatabase
    db=AeroDatabase(HERE/'aerodynamics/aero_database.npz')
    first=Path(db.metadata['raw_directory'])/'case_0000'
    used={}
    for f in (HERE/'geometry').glob('*'):
        if f.name=='h1_stowed.mass':continue
        raw=first/('plane.avl' if f.suffix=='.avl' else 'plane.mass' if f.name=='h1_a.mass' else f.name)
        used[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()==hashlib.sha256(raw.read_bytes()).hexdigest()
    assert all(used.values()),'Final input files differ from actual AVL run copies'
    runs={}
    for r in sorted((HERE/'runs').glob('*/checks.json')):runs[r.parent.name]=json.loads(r.read_text(encoding='utf8'))
    report={'source_unchanged_sha256_checks':checks,'chosen_variant':'A','original_lifting_mesh_parts':len(exact),'AVL_inputs_match_actual_run':used,
       'mass_accounting_total_kg':p['total_mass_kg'],'cad':json.loads(cv.read_text(encoding='utf8')),
       'clearance':json.loads((HERE/'clearance_summary.json').read_text(encoding='utf8')),'runs':runs,
       'full_deployment_recovery_mesh_convergence':'not established','complete_energy_balance':'not established'}
    (HERE/'delivery_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    manifest=[]
    for f in sorted(HERE.rglob('*')):
        if f.is_file() and '__pycache__' not in f.parts and f.name!='file_manifest.csv':
            manifest.append({'path':str(f.relative_to(HERE)),'bytes':f.stat().st_size})
    with (HERE/'file_manifest.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=['path','bytes']);w.writeheader();w.writerows(manifest)
    print(json.dumps({'source_unchanged':checks,'exact_lifting_parts':len(exact),'files':len(manifest)},indent=2))

if __name__=='__main__':main()
