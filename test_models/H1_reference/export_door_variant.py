"""Export a door variant from this project's copied CAD, preserving raw assets."""
from pathlib import Path
import argparse
import json
import hashlib
import numpy as np
import cadquery as cq
from dbf_stability import load_case
from dbf_stability.door import door_hinge, door_reference_shift
from geometry_paths import cad_directory

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();c=load_case(args.case);b=c['bay'];angle=max(v for _,v in c['winch']['door_schedule'])
    args.output.mkdir(parents=True,exist_ok=True)
    shapes={f.stem:cq.importers.importStep(str(f)).val() for f in cad_directory(c).glob('*.step') if f.stem!='H1_A_temporary_assembly'}
    original=shapes.pop('rear_door_100deg');h=1000*door_hinge(b)
    reference=original.translate(tuple(1000*door_reference_shift(b)))
    def posed(a):return reference.rotate(tuple(h),tuple(h+[0,1,0]),a-100)
    closed,opened=posed(0),posed(angle)
    cq.exporters.export(closed,str(args.output/'door_closed.step'))
    cq.exporters.export(opened,str(args.output/f'door_{angle:g}deg.step'))
    shapes[f'rear_door_{angle:g}deg']=opened
    assembly=cq.Assembly(name=f'H1_door_{angle:g}_FRD_mm')
    for name,shape in shapes.items():
        assembly.add(shape,name=name,color=cq.Color(.96,.55,.12) if name.startswith('rear_door') else cq.Color(.55,.62,.66))
    out=args.output/f'H1_door_{angle:g}_assembly.step';assembly.export(str(out))
    restored=cq.importers.importStep(str(out)).val()
    closed_assembly=cq.Assembly(name='H1_door_closed_FRD_mm')
    for name,shape in shapes.items():
        closed_assembly.add(closed if name.startswith('rear_door_') else shape,name=name,
            color=cq.Color(.96,.55,.12) if name.startswith('rear_door') else cq.Color(.55,.62,.66))
    closed_out=args.output/'H1_door_closed_assembly.step'
    closed_assembly.export(str(closed_out))
    closed_restored=cq.importers.importStep(str(closed_out)).val()
    info={'case':str(args.case.resolve()),'case_sha256':hashlib.sha256(args.case.read_bytes()).hexdigest(),
          'units':'mm; aircraft CG, forward-right-down','door_angle_deg':angle,
          'door_hinge_offset_m':b['door_hinge_offset_m'],'door_closed_offset_m':b['door_closed_offset_m'],
          'hinge_frd_m':(h/1000).tolist(),'valid_solids':restored.isValid(),
          'expected_solid_count':sum(len(s.Solids()) for s in shapes.values()),'roundtrip_solid_count':len(restored.Solids()),
          'closed_assembly_valid':closed_restored.isValid(),'closed_assembly_solid_count':len(closed_restored.Solids()),
          'note':'Door leaf geometry only; ideal prescribed hinge. No manufactured hinge/actuator bracket or seal design.'}
    if not info['valid_solids'] or info['expected_solid_count']!=info['roundtrip_solid_count']:raise ValueError(info)
    if not info['closed_assembly_valid'] or info['expected_solid_count']!=info['closed_assembly_solid_count']:raise ValueError(info)
    (args.output/'validation.json').write_text(json.dumps(info,indent=2),encoding='utf8')
    print(json.dumps(info,indent=2))


if __name__=='__main__':main()
