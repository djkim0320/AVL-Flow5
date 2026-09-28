"""Extended rear leaf and lowered hinge for a 270 degree belly fold.

STEP mm, STL m, FRD at updated aircraft CG. Preserves source geometries.
Pin/cheek joints are clearance fits; brackets attach to shell intentionally.
"""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse, copy, hashlib, json
from pathlib import Path
import numpy as np
import cadquery as cq
import yaml
from dbf_stability import load_case
from dbf_stability.door import door_hinge, door_reference_shift
from dbf_stability.collision import read_binary_stl
from build_cad import stl, glb

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def pa(r):
    return np.eye(3)*(r@r)-np.outer(r, r)


def properties(shape, mass):
    center = np.asarray(shape.Center().toTuple())*.001
    inertia = np.asarray(cq.Shape.matrixOfInertia(shape))*mass/shape.Volume()*1e-6
    return center, inertia


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--drop-mm', type=float, default=30.)
    p.add_argument('--version', default='v1')
    args = p.parse_args()
    if not args.version.isidentifier() or not 25 <= args.drop_mm <= 50:
        raise ValueError('Invalid version or hinge drop')
    c = load_case(ROOT/'examples/h1_round_x_capture_stop_v3_shielded.yaml')
    old = copy.deepcopy(c)
    source = (ROOT/c['collision']['mesh_directory']).parent
    out = HERE/f'geometry_variants/normal_r3_underbody_270_{args.version}'
    out.mkdir(exist_ok=False)
    cad, meshes = out/'cad', out/'meshes'
    cad.mkdir(); meshes.mkdir()
    shapes = {f.stem: cq.importers.importStep(str(f)).val()
              for f in (source/'cad').glob('*.step') if f.stem != 'H1_A_temporary_assembly'}
    b = c['bay']; old_h = 1000*door_hinge(b)
    old_closed = shapes['rear_door_100deg'].translate(tuple(1000*door_reference_shift(b))).rotate(
        tuple(old_h), tuple(old_h+[0, 1, 0]), -100)
    # The existing 70 mm aperture stays covered. Extend the same 2 mm leaf below it.
    # Hinge lies 2 mm above the new lower edge; a 1.4 mm bore accepts a 1 mm pin.
    b['door_hinge_offset_m'] = [-.003, 0., args.drop_mm*.001]
    b['door_closed_offset_m'] = [-.003, 0., (args.drop_mm+2)*.001]
    b['door_length_m'] = .070+(args.drop_mm+2)*.001
    hinge = 1000*door_hinge(b)
    closed = cq.Solid.makeBox(2., 56., b['door_length_m']*1000,
        cq.Vector(hinge[0]-1., -28., b['ceiling_z_m']*1000))
    bore = cq.Solid.makeCylinder(.7, 60., cq.Vector(hinge[0], -30., hinge[2]), cq.Vector(0, 1, 0))
    closed = closed.cut(bore).clean()
    reference = closed.rotate(tuple(hinge), tuple(hinge+[0, 1, 0]), 100).translate(
        tuple(-1000*door_reference_shift(b)))
    shapes['rear_door_100deg'] = reference
    hardware = {'door_hinge_pin': cq.Solid.makeCylinder(.5, 64.,
                  cq.Vector(hinge[0], -32., hinge[2]), cq.Vector(0, 1, 0))}
    for sign in (-1, 1):
        y = 29. if sign == 1 else -31.
        cheek = cq.Solid.makeBox(22., 2., args.drop_mm-6.,
            cq.Vector(hinge[0]-4., y, b['floor_z_m']*1000+10.))
        hole = cq.Solid.makeCylinder(.7, 4., cq.Vector(hinge[0], y-1., hinge[2]), cq.Vector(0, 1, 0))
        hardware[f'door_hinge_cheek_{sign:+d}'] = cheek.cut(hole).clean()
        # Upper stop: 0.2 mm nominal gap so sampled CAD audit distinguishes
        # working clearance from a collision. Real pad/latch compliance is not solved.
        ypad = 22. if sign == 1 else -26.
        hardware[f'door_fold_stop_{sign:+d}'] = cq.Solid.makeBox(6., 4., args.drop_mm-1.2-12.,
            cq.Vector(hinge[0]+78., ypad, b['floor_z_m']*1000+12.))
    for name, shape in hardware.items():
        if not shape.isValid() or len(shape.Solids()) != 1:
            raise ValueError(f'Invalid hardware: {name}')
    # Keep the existing assumed material density for the leaf, including the hole.
    budget = json.loads((HERE/'geometry_variants/normal_r3/mass_properties.json').read_text(encoding='utf8'))
    old_mass = next(r['mass_kg'] for r in budget['records'] if r['part'] == 'rear_door_100deg')
    new_mass = old_mass*closed.Volume()/old_closed.Volume()
    old_center, old_inertia = properties(old_closed, old_mass)
    new_center, new_inertia = properties(closed, new_mass)
    # Declared 10 g hardware budget, volume-weighted CAD inertia; not measured.
    total_volume = sum(s.Volume() for s in hardware.values())
    changes = [(new_mass, new_center, new_inertia), (-old_mass, old_center, -old_inertia)]
    records = []
    for name, shape in hardware.items():
        mass = .010*shape.Volume()/total_volume
        center, inertia = properties(shape, mass)
        changes.append((mass, center, inertia))
        records.append(dict(part=name, mass_kg=mass, center_old_frd_m=center.tolist(),
                            inertia_at_part_kgm2=inertia.tolist()))
    mass = c['aircraft']['mass_kg']+sum(q[0] for q in changes)
    shift = sum(m*r for m, r, _ in changes)/mass
    inertia_origin = np.array(c['aircraft']['inertia_kgm2'])+sum(I+m*pa(r) for m, r, I in changes)
    c['aircraft']['mass_kg'] = float(mass)
    c['aircraft']['inertia_kgm2'] = (inertia_origin-mass*pa(shift)).tolist()
    c['aircraft']['cg_m'] = (np.array(c['aircraft']['cg_m'])+shift).tolist()
    c['aircraft']['cg_shift_from_r3_body_m'] = (np.array(c['aircraft']['cg_shift_from_r3_body_m'])+shift).tolist()
    for key in ('tow_point_m', 'thrust_point_m'):
        c['aircraft'][key] = (np.array(c['aircraft'][key])-shift).tolist()
    b['stowed_center_m'] = (np.array(b['stowed_center_m'])-shift).tolist()
    for key in ('exit_x_m', 'front_x_m'): b[key] -= float(shift[0])
    for key in ('floor_z_m', 'ceiling_z_m'): b[key] -= float(shift[2])
    shapes.update(hardware)
    assembly = cq.Assembly(name='R3_underbody_270_reference_FRD_mm')
    opened_assembly = cq.Assembly(name='R3_underbody_270_open_FRD_mm')
    parts = []
    for name, shape in shapes.items():
        shape = shape.translate(tuple(-1000*shift))
        cq.exporters.export(shape, str(cad/f'{name}.step'))
        if name == 'rear_door_100deg' or name in hardware:
            vertices, faces = shape.tessellate(.08, .12)
            v = np.array([p.toTuple() for p in vertices])*.001; f = np.asarray(faces, int)
            tri = v[f]; area = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)
            f = f[area > 1e-14]
        else:
            v, f = read_binary_stl(source/'meshes'/f'{name}_FRD_m.stl'); v -= shift
        stl(meshes/f'{name}_FRD_m.stl', v, f)
        color = [.10, .48, .72, 1] if name.startswith(('rear_door', 'door_')) else [.88, .28, .32, 1] if name=='capture_front_stop' else [.91, .63, .17, 1] if name.startswith('sensor') else [.35, .55, .58, .3] if name.startswith('fuselage') else [.55, .60, .65, 1]
        assembly.add(shape, name=name, color=cq.Color(*color)); parts.append((name, v, f, color))
        if name == 'rear_door_100deg':
            h = 1000*door_hinge(b)
            shape = shape.translate(tuple(1000*door_reference_shift(b))).rotate(tuple(h), tuple(h+[0, 1, 0]), 170)
            cq.exporters.export(shape, str(out/'door_270deg.step'))
        opened_assembly.add(shape, name=name, color=cq.Color(*color))
    assembly.export(str(cad/'H1_A_temporary_assembly.step'))
    opened_assembly.export(str(out/'H1_underbody_270_assembly.step'))
    restored = cq.importers.importStep(str(cad/'H1_A_temporary_assembly.step')).val()
    if not restored.isValid() or len(restored.Solids()) != len(shapes): raise ValueError('STEP roundtrip failed')
    glb(meshes/'H1_A_temporary_assembly.glb', parts)
    c['collision']['mesh_directory'] = meshes.relative_to(ROOT).as_posix()
    c['collision']['concave_parts'] += [n for n in hardware if 'cheek' in n]
    c['collision']['part_materials'] = {'capture_front_stop': dict(stiffness_N_m=20000., damping_Ns_m=80.)}
    from analytic_stop_probe import cone_spec
    # The cone itself is unchanged; translate its verified support definition.
    spec = cone_spec(old, 256)
    spec['back_x_m'] -= float(shift[0]); spec['front_x_m'] -= float(shift[0])
    spec['center_yz_m'] = (np.asarray(spec['center_yz_m'])-shift[1:]).tolist()
    spec['cad_sha256'] = {name: hashlib.sha256((cad/name).read_bytes()).hexdigest() for name in ('sensor_body.step', 'capture_front_stop.step')}
    c['collision']['analytic_conical_stop'] = spec
    c['winch']['door_schedule'] = [[t, float(v)*270/140] for t, v in c['winch']['door_schedule']]
    c['name'] = 'R3_270deg_underbody_extended_door'
    c['provenance']['aircraft']['source'] += ' Replaced door with extended leaf at original assumed effective density; added 10 g assumed pin, brackets and stops. Closed-pose CAD mass/CG/inertia recomputed. Door mass remains frozen closed in the coupled rigid-aircraft model; no servo dynamics.'
    c['provenance']['bay']['source'] = f'56x70 mm aperture retained. {b["door_length_m"]*1000:g}x56x2 mm extended door, hinge {args.drop_mm:g} mm below aperture floor and 3 mm aft. 270 degree prescribed fold; 1 mm pin in 1.4 mm bore, lateral cheeks, 0.2 mm upper-stop gap. Ideal commanded holding, not a designed actuator/latch. Convex leaf envelope conservatively fills only pin bore for sensor/cable contacts.'
    c['provenance']['aero']['source'] = 'Actual flow5 7.57 25 m/s R3 lifting-surface database reused; wing and tail unchanged. Updated mass/CG/inertia and original aerodynamic moment reference. Extended door/bracket local flow and drag are not resolved by this database.'
    c['provenance']['flight']['source'] = '100 m, 25 m/s still-air reference; new geometry. Existing longitudinal controller and assumptions retained.'
    case = ROOT/f'examples/h1_underbody_270_{args.version}.yaml'
    case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')}, sort_keys=False, allow_unicode=True), encoding='utf8')
    load_case(case)
    record = dict(case=str(case), source=str(source), hinge_drop_mm=args.drop_mm,
        old_door_mass_kg=old_mass, new_door_mass_kg=new_mass, hardware_mass_kg=.010,
        mass_increment_kg=mass-old['aircraft']['mass_kg'], aircraft_mass_kg=mass,
        aircraft_cg_shift_m=shift.tolist(), new_door_center_old_frd_m=new_center.tolist(),
        old_door_center_old_frd_m=old_center.tolist(), hardware_records=records,
        solids=len(restored.Solids()), roundtrip_valid=restored.isValid(),
        door_mass_pose='frozen closed; prescribed hinge, no actuator dynamics',
        builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    source_record = json.loads((source/'geometry.json').read_text(encoding='utf8'))
    for key in ('cone_length_mm', 'cone_slope', 'mouth_diameter_mm', 'cable_bore_diameter_mm'):
        record[key] = source_record[key]
    (out/'geometry.json').write_text(json.dumps(record, indent=2), encoding='utf8')
    print(json.dumps(record), flush=True)


if __name__ == '__main__': main()
