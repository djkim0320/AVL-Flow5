"""flow5 7.57 XML subprocess adapter; all aerodynamic values come from flow5.

The lifting-surface exporter supports the R3 AVL SECTION/AFILE/CONTROL subset.
BODY is deliberately excluded in the VLM2 comparison. Native flow5 projects can
be run with run_script. Original input, output and executable hashes are kept.
"""

from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from itertools import product
import hashlib
import json
import os
import subprocess
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from .aero_database import AeroDatabase
from .aero_inputs import input_fingerprint


def element(parent, name, value=None):
    node = ET.SubElement(parent, name)
    if value is not None:
        node.text = str(value).lower() if isinstance(value, bool) else str(value)
    return node


def xml_file(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def read_lifting_surfaces(path):
    lines = [s.split('#')[0].strip() for s in Path(path).read_text().splitlines()]
    lines = [s for s in lines if s]
    surfaces = []
    i = 5
    if float(lines[1]) != 0 or not np.allclose(list(map(float, lines[2].split())), [0, 0, 0]):
        raise ValueError('flow5 conversion requires Mach 0 and explicit full geometry without AVL symmetry flags')
    refs = np.array(list(map(float, lines[3].split())))
    while i < len(lines):
        key = lines[i].split()[0]
        if key == 'BODY':
            if any(line.split()[0] == 'SURFACE' for line in lines[i + 1 :]):
                raise ValueError('flow5 conversion requires all lifting surfaces before BODY')
            break
        if key == 'SURFACE':
            mesh = list(map(float, lines[i + 2].split()))
            if len(mesh) != 2:
                raise ValueError('flow5 conversion requires SECTION-level span counts; SURFACE Nspan is unsupported')
            if not np.isfinite(mesh).all() or mesh[0] < 1 or not mesh[0].is_integer() or mesh[1] != 1.0:
                raise ValueError('flow5 conversion requires positive integer Nchord and cosine Cspace=1')
            s = dict(name=lines[i + 1], nx=int(mesh[0]), duplicate=False, sections=[])
            surfaces.append(s)
            i += 3
        elif key == 'YDUPLICATE':
            if float(lines[i + 1]) != 0:
                raise ValueError('Only centre-plane YDUPLICATE supported')
            s['duplicate'] = True
            i += 2
        elif key == 'SECTION':
            v = list(map(float, lines[i + 1].split()))
            if len(v) != 7 or not np.isfinite(v).all() or v[5] < 1 or not v[5].is_integer() or v[6] != 1.0:
                raise ValueError('flow5 conversion requires positive integer SECTION Nspan and cosine Sspace=1')
            sec = dict(x=v[0], y=v[1], z=v[2], chord=v[3], incidence=v[4], ny=int(v[5]), controls={})
            s['sections'].append(sec)
            i += 2
        elif key == 'AFILE':
            sec['foil'] = str((Path(path).parent / lines[i + 1]).resolve())
            i += 2
        elif key == 'CONTROL':
            v = lines[i + 1].split()
            sec['controls'][v[0]] = list(map(float, v[1:]))
            i += 2
            axis = list(map(float, v[3:6]))
            expected = [0, 1, 0] if s['duplicate'] else [0, 0, 1]
            if (
                v[0] not in ('aileron', 'elevator', 'rudder')
                or len(v) != 7
                or not (np.allclose(axis, [0, 0, 0]) or np.allclose(axis, expected))
            ):
                raise ValueError(
                    'flow5 conversion requires named aileron/elevator/rudder controls with automatic or positive span-axis hinges'
                )
        elif i == 5 and len(lines[i].split()) == 1:
            if float(lines[i]) != 0:
                raise ValueError('Use explicit aircraft.profile_cd instead of AVL header CDp for flow5')
            i += 1
        else:
            raise ValueError(f'Unsupported AVL geometry statement for flow5: {lines[i]}')
    if not surfaces:
        raise ValueError('No lifting surfaces')
    for surface in surfaces:
        if len(surface['sections']) < 2 or any('foil' not in section for section in surface['sections']):
            raise ValueError('flow5 conversion requires at least two sections with AFILE per surface')
        for first, last in zip(surface['sections'][:-1], surface['sections'][1:]):
            for name in set(first['controls']) & set(last['controls']):
                if not np.allclose(first['controls'][name], last['controls'][name]):
                    raise ValueError('flow5 conversion requires matching control definitions across each strip')
    return refs, surfaces


def export_lifting_plane(geometry, directory, controls):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    refs, surfaces = read_lifting_surfaces(geometry)
    root = ET.Element('xflplane', version='1.0')
    plane = element(root, 'Plane')
    element(plane, 'Name', 'DBF_R3_lifting_surfaces')
    foils = []
    pieces = []
    # Native mirrored strips retain exact left/right symmetry. Control boundaries
    # become separate strips, avoiding a fictitious blend across fixed surfaces.
    for surface in surfaces:
        for j, (first, last) in enumerate(zip(surface['sections'][:-1], surface['sections'][1:])):
            pair = surface['duplicate']
            vertical = not pair
            positions = np.array([[v['x'], v['y'], v['z']] for v in (first, last)])
            common = set(first['controls']) & set(last['controls'])
            names = {}
            angles = {}
            for side in (-1, 1):
                delta = 0.0
                hinge = 0.75
                for name in common:
                    gain, hinge, *axis, duplicate = first['controls'][name]
                    delta += (
                        controls.get(name, 0.0)
                        * gain
                        * (duplicate if side < 0 and pair else 1.0)
                        * (-1 if vertical else 1)
                    )
                angles[side] = delta
                names[side] = []
                for k, v in enumerate((first, last)):
                    foil = np.loadtxt(v['foil'], skiprows=1)
                    le = int(np.argmin(foil[:, 0]))
                    upper = foil[: le + 1][::-1]
                    lower = foil[le:]
                    hinge_z = (
                        np.interp(hinge, upper[:, 0], upper[:, 1]) + np.interp(hinge, lower[:, 0], lower[:, 1])
                    ) / 2
                    moving = foil[:, 0] > hinge
                    theta = np.deg2rad(delta)
                    rot = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
                    foil[moving] = (foil[moving] - [hinge, hinge_z]) @ rot.T + [hinge, hinge_z]
                    name = f'foil_{len(pieces):02d}_{side:+d}_{k}'
                    path = directory / (name + '.dat')
                    np.savetxt(path, foil, header=name, comments='', fmt='%.10f')
                    foils.append(path)
                    names[side].append(name)
            # flow5 twists the native inner endpoint about y and the outer about
            # the dihedral-rotated span axis; adjust their quarter-chord pivots.
            pos = positions.copy()
            if pair:
                for _ in range(5):
                    phi = np.arctan2(pos[1, 2] - pos[0, 2], pos[1, 1] - pos[0, 1])
                    for k, v in enumerate((first, last)):
                        th = np.deg2rad(v['incidence'])
                        ph = phi if k else 0.0
                        shift = (
                            0.25
                            * v['chord']
                            * np.array([1 - np.cos(th), -np.sin(th) * np.sin(ph), np.sin(th) * np.cos(ph)])
                        )
                        pos[k] = positions[k] - shift
                span0 = pos[0, 1]
                span1 = span0 + np.linalg.norm((pos[1] - pos[0])[[1, 2]])
                translation = [pos[0, 0], 0, pos[0, 2]]
                dihedral = np.rad2deg(phi)
            else:
                if abs(positions[1, 1] - positions[0, 1]) > 1e-9:
                    raise ValueError('Single surface must be vertical')
                span0 = 0.0
                span1 = positions[1, 2] - positions[0, 2]
                translation = positions[0]
                dihedral = 0.0
            wing = element(plane, 'wing')
            label = f'{surface["name"]}_{j}'
            for key, value in dict(
                Name=label,
                Type='OTHERWING' if pieces else 'MAINWING',
                Position=', '.join(map(str, translation)),
                Rx_angle=0 if pair else -90,
                Ry_angle=0,
                symmetric=False,
                Two_Sided=pair,
                Closed_Inner_Side=False,
                AutoInertia=False,
                Tip_Strips=0,
            ).items():
                element(wing, key, value)
            sections = element(wing, 'Sections')
            for k, v in enumerate((first, last)):
                sec = element(sections, 'Section')
                for key, value in dict(
                    y_position=span1 if k else span0,
                    Chord=v['chord'],
                    xOffset=pos[k, 0] - pos[0, 0],
                    Dihedral=dihedral,
                    Twist=v['incidence'],
                    x_number_of_panels=surface['nx'],
                    y_number_of_panels=first['ny'],
                    x_panel_distribution='COSINE',
                    y_panel_distribution='COSINE',
                    Left_Side_FoilName=names[-1][k],
                    Right_Side_FoilName=names[1][k],
                ).items():
                    element(sec, key, value)
            pieces.append(dict(name=label, control_deg=angles, le_aru_m=positions.tolist()))
    xml_file(directory / 'plane.xml', root)
    (directory / 'geometry_manifest.json').write_text(
        json.dumps(
            dict(
                source=str(geometry),
                source_sha256=hashlib.sha256(Path(geometry).read_bytes()).hexdigest(),
                refs=refs.tolist(),
                pieces=pieces,
                body_included=False,
                control_method='physical trailing-edge rotation, not AVL linearized normals',
            ),
            indent=2,
        )
    )
    return refs, foils


def write_polar(config, directory, refs):
    root = ET.Element('xflPlanePolar', version='1.0')
    p = element(root, 'Polar')
    for k, v in dict(
        Polar_Name='T8_DBF',
        Plane_Name='DBF_R3_lifting_surfaces',
        Type='T8POLAR',
        Method='VLM2',
        Thin_Surfaces=True,
        Use_plane_inertia=False,
        Include_Fuse_Moments=False,
        Ground_Effect=False,
    ).items():
        element(p, k, v)
    r = element(p, 'Reference_Dimensions')
    for k, v in dict(
        Reference_Dimensions='CUSTOM',
        Reference_Area=refs[0],
        Reference_Chord_Length=refs[1],
        Reference_Span_Length=refs[2],
    ).items():
        element(r, k, v)
    f = element(p, 'Fluid')
    element(f, 'Density', config['flight']['rho_kg_m3'])
    element(f, 'Viscosity', 1.5e-5)
    v = element(p, 'Viscous_Analysis')
    element(v, 'Is_Viscous_Analysis', False)
    i = element(p, 'Inertia')
    a = config['aircraft']
    tensor = np.asarray(a['inertia_kgm2'])
    for k, v in dict(
        Mass=a['mass_kg'],
        CoG=', '.join(map(str, np.array(a['cg_m']) * [-1, 1, -1])),
        CoG_Ixx=tensor[0, 0],
        CoG_Iyy=tensor[1, 1],
        CoG_Izz=tensor[2, 2],
        CoG_Ixz=-tensor[0, 2],
    ).items():
        element(i, k, v)
    w = element(p, 'Wake')
    for k, v in dict(FlatPanelWake=True, NX=5, ProgressionFactor=1.1, LengthFactor=30).items():
        element(w, k, v)
    xml_file(Path(directory) / 'polar.xml', root)


def write_script(directory, foils, points, threads=2):
    directory = Path(directory).resolve()
    root = ET.Element('xflscript', version='1.0')
    m = element(root, 'Metadata')
    for k, v in dict(
        make_project_file=True, project_file_name='dbf.fl5', polar_text_output_format='csv', Double_Precision=True
    ).items():
        element(m, k, v)
    d = element(m, 'Directories')
    for k, v in dict(
        output_dir=directory / 'output',
        plane_definition_xml_dir=directory,
        plane_analysis_xml_dir=directory,
        foil_files_dir=directory,
        recursive_scan=False,
    ).items():
        element(d, k, str(v).replace('\\', '/'))
    mt = element(m, 'MultiThreading')
    element(mt, 'Allow_Multithreading', True)
    element(mt, 'max_threads', threads)
    p = element(root, 'Plane_analysis')
    out = element(p, 'Plane_Analysis_Output')
    for k, v in dict(
        make_polars_text_file=True,
        make_oppoints=True,
        make_oppoints_text_file=True,
        Compute_derivatives=True,
        export_stl_mesh=False,
    ).items():
        element(out, k, v)
    f = element(p, 'Foil_Dat_Files')
    for path in foils:
        element(f, 'Foil_File_Name', path.name)
    f = element(p, 'Plane_Definition_Files')
    element(f, 'Plane_File_Name', 'plane.xml')
    f = element(p, 'Plane_Analysis_Files')
    element(f, 'Analysis_File_Name', 'polar.xml')
    f = element(p, 'Plane_Analysis_Data')
    for point in points:
        element(f, 'T8_Range', ', '.join(map(str, point)))
    xml_file(directory / 'script.xml', root)
    return directory / 'script.xml'


def run_script(executable, script, timeout=600):
    executable = Path(executable).resolve()
    script = Path(script).resolve()
    if not executable.is_file():
        raise FileNotFoundError(executable)
    env = os.environ.copy()
    env['OMP_NUM_THREADS'] = '2'
    env['MKL_NUM_THREADS'] = '2'
    # Script mode does not need user interaction; do not open desktop windows.
    flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
    startup = None
    if os.name == 'nt':
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = subprocess.SW_HIDE
    command = [str(executable), '-p', '-s', str(script)]
    version = (
        subprocess.run([str(executable), '--version'], capture_output=True, timeout=30, creationflags=flags)
        .stdout.decode('ascii', errors='replace')
        .strip()
    )
    if '7.57' not in version:
        raise ValueError(f'Adapter verified for flow5 7.57, found {version}')
    with (script.parent / 'stdout.txt').open('wb') as out, (script.parent / 'stderr.txt').open('wb') as err:
        try:
            result = subprocess.run(
                command,
                cwd=script.parent,
                env=env,
                stdout=out,
                stderr=err,
                timeout=timeout,
                creationflags=flags,
                startupinfo=startup,
            )
        except subprocess.TimeoutExpired:
            (script.parent / 'execution.json').write_text(
                json.dumps(dict(command=command, status='timeout', timeout_s=timeout))
            )
            raise
    manifest = dict(
        command=command,
        returncode=result.returncode,
        solver=version,
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
    )
    (script.parent / 'execution.json').write_text(json.dumps(manifest, indent=2))
    if result.returncode:
        raise RuntimeError(f'flow5 failed ({result.returncode}); inspect {script.parent}')
    return manifest


def parse_operating_point(path):
    """Read native 7.57 operating-point tables (space separated even with .csv)."""
    lines = Path(path).read_text(encoding='utf-8-sig').splitlines()
    data = {}
    for i, line in enumerate(lines[:-1]):
        keys = line.replace(',', ' ').split()
        if keys and keys[0] in ('α', 'CL', 'CXu', 'Cyb'):
            values = list(map(float, lines[i + 1].replace(',', ' ').split()))
            if len(values) != len(keys):
                raise ValueError(f'Malformed flow5 table in {path}')
            data.update(zip(keys, values))
    required = (
        'α',
        'β',
        'VInf(m/s)',
        'CL',
        'CY',
        'CD_inviscid',
        'CD_viscous',
        'Cl',
        'Cm_inviscid',
        'Cm_viscous',
        'Cn_inviscid',
        'Cn_viscous',
        'CXq',
        'CZq',
        'Cmq',
        'Cyp',
        'Clp',
        'Cnp',
        'Cyr',
        'Clr',
        'Cnr',
    )
    if any(k not in data or not np.isfinite(data[k]) for k in required):
        raise ValueError(f'Incomplete/nonfinite flow5 point, including rate derivatives: {path}')
    return data


def to_body_coefficients(data, refs):
    a, b = np.deg2rad([data['α'], data['β']])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    if abs(cb) < 0.1:
        raise ValueError('flow5 sideslip outside conversion range')
    # Export CY is still geometric/body y, while CL/CD and moments use true wind axes.
    cd = data['CD_inviscid'] + data['CD_viscous']
    cy = data['CY']
    cl = data['CL']
    axial = (cd + sb * cy) / cb
    force = np.array([-(ca * axial - sa * cl), cy, -(sa * axial + ca * cl)])
    wind_to_geom = np.array([[ca * cb, ca * sb, -sa], [-sb, cb, 0], [sa * cb, sa * sb, ca]])
    lengths = np.array([refs[2], refs[1], refs[2]])
    moment_wind = (
        np.array([data['Cl'], data['Cm_inviscid'] + data['Cm_viscous'], data['Cn_inviscid'] + data['Cn_viscous']])
        * lengths
    )
    moment = np.array([-1.0, 1.0, -1.0]) * (wind_to_geom @ moment_wind) / lengths
    return np.r_[force, moment]


def to_body_rate_derivatives(data, refs):
    # flow5 exports the classical decoupled longitudinal/lateral derivative set.
    # The absent cross-blocks are an explicit model approximation, not solver data.
    derivative = np.zeros((6, 3))
    derivative[[0, 2, 4], 1] = [data[k] for k in ('CXq', 'CZq', 'Cmq')]
    derivative[[1, 3, 5], 0] = [data[k] for k in ('Cyp', 'Clp', 'Cnp')]
    derivative[[1, 3, 5], 2] = [data[k] for k in ('Cyr', 'Clr', 'Cnr')]
    a = np.deg2rad(data['α'])
    ca, sa = np.cos(a), np.sin(a)
    stab_to_body = np.array([[ca, 0, -sa], [0, 1, 0], [sa, 0, ca]])
    derivative[:3] = stab_to_body @ derivative[:3] @ stab_to_body.T
    derivative[3:] = stab_to_body @ derivative[3:] @ stab_to_body.T
    return derivative


def _database_job(args):
    config, directory, elevator, lateral_name, lateral_value, points = args
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    cfg = config['aero']
    root = Path(config['_root'])
    fingerprint = input_fingerprint(root / cfg['geometry'])
    controls = dict(elevator=elevator, aileron=0.0, rudder=0.0)
    if lateral_name:
        controls[lateral_name] = lateral_value
    refs, foils = export_lifting_plane(root / cfg['geometry'], directory, controls)
    if input_fingerprint(root / cfg['geometry']) != fingerprint:
        raise ValueError('AERO_MISMATCH: aerodynamic inputs changed during flow5 export')
    write_polar(config, directory, refs)
    script = write_script(directory, foils, points, threads=cfg.get('threads_per_worker', 2))
    manifest = run_script(root / cfg['flow5_executable'], script, timeout=cfg.get('timeout_s', 600))
    files = list((directory / 'output/dbf/DBF_R3_lifting_surfaces/polar').glob('*.csv'))
    rows = {}
    for file in files:
        row = parse_operating_point(file)
        key = (row['α'], row['β'], row['VInf(m/s)'])
        if key in rows:
            raise ValueError(f'Duplicate flow5 point: {key}')
        rows[key] = row
    if set(rows) != set(map(tuple, points)):
        raise ValueError(f'flow5 output point grid incomplete: {directory}')
    log = (directory / 'stdout.txt').read_text(encoding='utf8', errors='replace')
    if (
        'Panel analysis completed successfully' not in log
        or 'Non-dim' not in log
        and 'Longitudinal derivatives' not in log
    ):
        raise RuntimeError(f'flow5 analysis/derivatives not confirmed: {directory}')
    return dict(
        elevator=elevator,
        lateral_name=lateral_name,
        lateral_value=lateral_value,
        rows=rows,
        refs=refs,
        manifest=manifest,
        directory=str(directory),
        input_files_sha256=fingerprint,
    )


def build_flow5_database(config, output=None, workers=None):
    root = Path(config['_root'])
    cfg = config['aero']
    fingerprint = input_fingerprint(root / cfg['geometry'])
    if cfg.get('flow5_method', 'VLM2') != 'VLM2':
        raise ValueError('This automatic exporter supports VLM2 lifting surfaces only')
    if cfg.get('rate_derivative_closure') != 'classical_longitudinal_lateral':
        raise ValueError(
            'Explicit aero.rate_derivative_closure=classical_longitudinal_lateral required; flow5 does not export all 18 rate entries'
        )
    axes = [np.asarray(cfg[k], float) for k in ('alpha_deg', 'beta_deg', 'elevator_deg')]
    if any(len(v) < 2 or np.any(np.diff(v) <= 0) for v in axes):
        raise ValueError('Increasing alpha/beta/elevator grids required')
    if max(abs(axes[2])) > 10:
        raise ValueError('Initial flow5 geometric flap exporter limited to +/-10 degrees')
    output = Path(output or root / 'outputs/flow5_aero_database.npz').resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    batch = output.parent / ('flow5_runs_' + uuid.uuid4().hex[:8])
    points = [(float(a), float(b), float(config['flight']['speed_m_s'])) for a, b in product(*axes[:2])]
    step = float(cfg.get('lateral_difference_deg', 0.5))
    if not 0 < step <= 2:
        raise ValueError('Lateral finite difference step must be (0,2] degrees')
    jobs = []
    for e in axes[2]:
        for name, value in [(None, 0), ('aileron', -step), ('aileron', step), ('rudder', -step), ('rudder', step)]:
            jobs.append((config, str(batch / f'job_{len(jobs):03d}'), float(e), name, value, points))
    with ProcessPoolExecutor(max_workers=workers or min(4, max(1, (os.cpu_count() or 2) // 2))) as pool:
        rows = list(pool.map(_database_job, jobs))
    if input_fingerprint(root / cfg['geometry']) != fingerprint or any(
        r['input_files_sha256'] != fingerprint for r in rows
    ):
        raise ValueError('AERO_MISMATCH: aerodynamic input files changed during database construction')
    lookup = {(r['elevator'], r['lateral_name'], r['lateral_value']): r for r in rows}
    refs = rows[0]['refs']
    shape = tuple(map(len, axes))
    coeff = np.empty((*shape, 6))
    rates = np.empty((*shape, 6, 3))
    controls = np.empty((*shape, 6, 2))
    for i, a in enumerate(axes[0]):
        for j, b in enumerate(axes[1]):
            key = (float(a), float(b), float(config['flight']['speed_m_s']))
            for k, e in enumerate(axes[2]):
                point = lookup[(e, None, 0)]['rows'][key]
                coeff[i, j, k] = to_body_coefficients(point, refs)
                rates[i, j, k] = to_body_rate_derivatives(point, refs)
                for column, name in enumerate(('aileron', 'rudder')):
                    plus = to_body_coefficients(lookup[(e, name, step)]['rows'][key], refs)
                    minus = to_body_coefficients(lookup[(e, name, -step)]['rows'][key], refs)
                    controls[i, j, k, :, column] = (plus - minus) / (2 * step)
    metadata = dict(
        solver=rows[0]['manifest']['solver'],
        axes='FRD body',
        angle_unit='degrees',
        rate_unit='pb/2V,qc/2V,rb/2V',
        moment_reference_frd_m=config['aircraft']['cg_m'],
        raw_directory=str(batch),
        body_included=False,
        method='VLM2',
        condition_count=int(np.prod(shape)),
        actual_solver_points=len(jobs) * len(points),
        lateral_difference_deg=step,
        geometry_sha256=hashlib.sha256((root / cfg['geometry']).read_bytes()).hexdigest(),
        input_files_sha256=fingerprint,
        flow5_speed_m_s=float(config['flight']['speed_m_s']),
        executable_sha256=rows[0]['manifest']['executable_sha256'],
        rate_derivative_closure='classical_longitudinal_lateral',
        limitations=[
            'No fuselage aerodynamics in this VLM2 comparison; full CAD retained for collisions.',
            'Geometric flap rotations and independent span strips differ from AVL linearized control normals.',
            'Unexported cross-block rate derivatives assumed zero in stability axes; not validated at finite sideslip/asymmetric controls.',
            'Inviscid solver; aircraft.profile_cd is the explicit additional drag assumption.',
        ],
    )
    np.savez_compressed(
        output,
        alpha=axes[0],
        beta=axes[1],
        elevator=axes[2],
        refs=refs,
        coeff=coeff,
        rates=rates,
        controls=controls,
        metadata=json.dumps(metadata),
    )
    (output.parent / 'flow5_manifest.json').write_text(json.dumps(metadata, indent=2))
    return AeroDatabase(output)
