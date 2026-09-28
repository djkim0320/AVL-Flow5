"""Actual AVL subprocess adapter. Missing solver/results are errors, never fabricated."""
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from itertools import product
import hashlib
import json
import os
import re
import shutil
import subprocess
import uuid
import numpy as np
from scipy.interpolate import RegularGridInterpolator
from .aero_inputs import input_fingerprint
from .solver_resources import available_memory_bytes, avl_worker_budget

COEFF = ("CX", "CY", "CZ", "Cl", "Cm", "Cn")


def write_mass_file(config, path, stowed=False):
    """AVL geometric axes: aft/right/up. Products are -tensor off-diagonals.

    Column order verified against AVL 3.52 src/amass.f (not the conflicting
    order in one paragraph of the primer). Deployed cable is not a rigid mass.
    """
    a, s, cable = config['aircraft'], config['sensor'], config['cable']
    transform = np.diag([-1., 1., -1.])
    entries = [(a['mass_kg'], a['cg_m'], a['inertia_kgm2'], 'aircraft only')]
    if stowed:
        from .math3d import rotation
        orientation=rotation(config['bay'].get('stowed_quaternion_wxyz',[1.,0.,0.,0.]))
        sensor_inertia=np.zeros((3,3)) if s.get('model')=='point_mass' else orientation@np.asarray(s['inertia_kgm2'])@orientation.T
        entries += [(s['mass_kg'], np.array(a['cg_m'])+config['bay']['stowed_center_m'], sensor_inertia, 'stowed sensor'),
                    (cable['density_kg_m']*cable['length_m'], np.array(a['cg_m'])+a['tow_point_m'], np.zeros((3,3)), 'stored cable point approximation')]
    lines = ['Lunit = 1 m', 'Munit = 1 kg', 'Tunit = 1 s',
             f"g = {config['flight']['g_m_s2']}", f"rho = {config['flight']['rho_kg_m3']}",
             '# mass x y z Ixx Iyy Izz Ixy Ixz Iyz']
    for mass, cg, inertia, name in entries:
        j = transform@np.asarray(inertia)@transform
        numbers = [mass, *(transform@cg), *np.diag(j), -j[0,1], -j[0,2], -j[1,2]]
        lines.append(' '.join(f'{x:.12g}' for x in numbers)+' # '+name)
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines)+'\n', encoding='ascii')
    return path


def parse_output(text):
    # ST ends with "Clb Cnr / Clr Cnb = ...". That diagnostic ratio is not Cnb.
    text='\n'.join(line for line in text.splitlines() if not re.search(r'Clb\s+Cnr\s*/\s*Clr\s+Cnb',line))
    return {k: float(v.replace("D", "E")) for k, v in re.findall(
        r"([A-Za-z][A-Za-z0-9_']*)\s*=\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[EeDd][-+]?\d+)?)", text)}


def run_avl(executable, geometry, output_dir, alpha_deg=0., beta_deg=0., elevator_deg=0., elevator_index=1, rates=(0., 0., 0.), timeout_s=60.):
    if not np.isfinite(timeout_s) or timeout_s<=0:
        raise ValueError('AVL timeout_s must be finite and positive')
    exe, geom = Path(executable).resolve(), Path(geometry).resolve()
    if not exe.is_file() or not geom.is_file():
        raise FileNotFoundError(f"AVL executable or geometry missing: {exe}, {geom}")
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=False)
    # Copy referenced airfoil/body files with their geometry directory hierarchy.
    shutil.copy2(geom, out/"plane.avl")
    mass_file=geom.with_suffix('.mass')
    if mass_file.is_file():
        shutil.copy2(mass_file,out/'plane.mass')
    lines = geom.read_text(encoding="ascii").splitlines()
    for i, line in enumerate(lines):
        if line.strip().split()[:1] in (["AFILE"], ["BFILE"]):
            ref = lines[i+1].strip()
            source = (geom.parent/ref).resolve()
            if not source.is_relative_to(geom.parent):
                raise ValueError("Geometry reference must remain in its input directory")
            target = out/ref
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    mass_command='MASS plane.mass\n' if mass_file.is_file() else ''
    commands = f"PLOP\nG\n\nLOAD plane.avl\n{mass_command}OPER\nA A {alpha_deg}\nB B {beta_deg}\n"
    if elevator_index:
        commands += f"D{elevator_index} D{elevator_index} {elevator_deg}\n"
    commands += f"R R {rates[0]}\nP P {rates[1]}\nY Y {rates[2]}\nX\nFT\nforces.txt\nSB\nderivatives.txt\nST\nstability.txt\n\nQUIT\n"
    (out/"commands.txt").write_text(commands, encoding="ascii")
    try:
        proc = subprocess.run([str(exe)], input=commands, text=True, capture_output=True, cwd=out, timeout=float(timeout_s),
                              creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    except subprocess.TimeoutExpired as exc:
        for name,value in [('stdout.txt',exc.stdout),('stderr.txt',exc.stderr)]:
            text=value.decode('utf8',errors='replace') if isinstance(value,bytes) else value or ''
            (out/name).write_text(text,encoding='utf8')
        (out/'failure.json').write_text(json.dumps(dict(status='timeout',timeout_s=float(timeout_s))),encoding='utf8')
        raise
    (out/"stdout.txt").write_text(proc.stdout, encoding="utf-8")
    (out/"stderr.txt").write_text(proc.stderr, encoding="utf-8")
    if proc.returncode or not (out/"derivatives.txt").exists():
        raise RuntimeError(f"AVL failed; inspect {out}")
    raw = (out/"derivatives.txt").read_text()
    if "Standard axis orientation" not in raw:
        raise ValueError("Expected AVL standard FRD output axes")
    d = parse_output(raw)
    required = [x+"tot" for x in COEFF]+[x+r for x in COEFF for r in "pqr"]
    if any(x not in d or not np.isfinite(d[x]) for x in required):
        raise ValueError(f"Incomplete/nonfinite AVL output: {out}")
    d["raw_dir"] = str(out)
    version=re.search(r'Athena Vortex Lattice\s+Program\s+Version\s+([\d.]+)',proc.stdout)
    if version is None or version.group(1)!='3.52':
        raise ValueError('This adapter was verified for AVL 3.52; solver version differs or is missing')
    d['solver_version']=version.group(1)
    return d


def _job(args):
    return run_avl(*args)


def build_aero_database(config, output=None, workers=None):
    backend=config['aero'].get('backend','avl').lower()
    if backend=='hybrid':
        from .hybrid import build_hybrid_database
        return build_hybrid_database(config,output,workers)
    if backend=='flow5':
        from .flow5 import build_flow5_database
        return build_flow5_database(config,output,workers)
    if backend!='avl':raise ValueError(f'Unknown aerodynamic backend: {backend}')
    root = Path(config["_root"])
    cfg = config["aero"]
    exe, geom = root/cfg["executable"], root/cfg["geometry"]
    fingerprint = input_fingerprint(geom)
    axes = [cfg[k] for k in ("alpha_deg", "beta_deg", "elevator_deg")]
    if any(len(x)<2 or np.any(np.diff(x)<=0) for x in axes):
        raise ValueError("Aero axes must have >=2 strictly increasing values")
    output = Path(output or root/"outputs/aero_database.npz").resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():raise FileExistsError(output)
    batch = output.parent/("avl_runs_"+uuid.uuid4().hex[:8])
    jobs = [(exe, geom, batch/f"case_{i:04d}", *v, cfg["elevator_index"], (0.,0.,0.), cfg.get('timeout_s',60.)) for i, v in enumerate(product(*axes))]
    requested = workers if workers is not None else min(8, max(1, (os.cpu_count() or 2)-1))
    execution = avl_worker_budget(requested, len(jobs), available_memory_bytes())
    workers = execution['effective_workers']
    batch.mkdir(parents=True, exist_ok=False)
    (batch/'execution.json').write_text(json.dumps(execution, indent=2), encoding='utf8')
    print(f'AVL: {workers} concurrent workers (requested {requested}; memory budget applied)', flush=True)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(_job, jobs))
    if input_fingerprint(geom)!=fingerprint or any(input_fingerprint(Path(d['raw_dir'])/'plane.avl')!=fingerprint for d in results):
        raise ValueError('AERO_MISMATCH: aerodynamic input files changed during database construction')
    shape = tuple(map(len, axes))
    coeff = np.array([[d[k+"tot"] for k in COEFF] for d in results]).reshape(*shape, 6)
    rates = np.array([[[d[k+r] for r in "pqr"] for k in COEFF] for d in results]).reshape(*shape, 6, 3)
    # Control slopes in AVL SB are per degree, unlike angular stability derivatives.
    # None means the aircraft definition explicitly has no actuator on this channel.
    controls = np.array([[[0. if j is None else d[k+f"d{j:02d}"] for j in cfg["lateral_control_indices"]] for k in COEFF] for d in results]).reshape(*shape, 6, 2)
    refs = [results[0][k] for k in ("Sref", "Cref", "Bref")]
    metadata = {"solver": "AVL "+results[0]['solver_version'], "axes": "FRD body", "angle_unit": "degrees", "rate_unit": "pb/2V,qc/2V,rb/2V",
                "executable_sha256": hashlib.sha256(exe.read_bytes()).hexdigest(), "geometry_sha256": hashlib.sha256(geom.read_bytes()).hexdigest(),
                "raw_directory": str(batch), "source": "https://web.mit.edu/drela/Public/web/avl/", "provenance": config["provenance"]["aero"],
                "moment_reference_frd_m": cfg["moment_reference_frd_m"],
                "input_files_sha256":fingerprint, "execution":execution,
                "control_indices":{"elevator":cfg['elevator_index'],"lateral":list(cfg['lateral_control_indices'])}}
    np.savez_compressed(output, alpha=axes[0], beta=axes[1], elevator=axes[2], coeff=coeff, rates=rates, controls=controls, refs=refs, metadata=json.dumps(metadata))
    return AeroDatabase(output)


class AeroDatabase:
    def __init__(self, path):
        self.path = str(Path(path).resolve())
        with np.load(path, allow_pickle=False) as d:
            self.axes = tuple(d[k].copy() for k in ("alpha", "beta", "elevator"))
            self.refs = d["refs"].copy()
            self.metadata = json.loads(str(d["metadata"]))
            if self.refs.shape!=(3,) or not np.isfinite(self.refs).all() or np.any(self.refs<=0):
                raise ValueError('Invalid aerodynamic reference dimensions')
            if any(x.ndim!=1 or len(x)<2 or not np.isfinite(x).all() or np.any(np.diff(x)<=0) for x in self.axes):
                raise ValueError('Invalid aerodynamic interpolation axes')
            for key,tail in [('coeff',(6,)),('rates',(6,3)),('controls',(6,2))]:
                if d[key].shape!=tuple(map(len,self.axes))+tail or not np.isfinite(d[key]).all():
                    raise ValueError(f'Invalid/nonfinite aerodynamic {key} table')
            for key,value in [('axes','FRD body'),('angle_unit','degrees'),('rate_unit','pb/2V,qc/2V,rb/2V')]:
                if self.metadata.get(key)!=value:raise ValueError(f'Unsupported aerodynamic {key}')
            ref=np.asarray(self.metadata.get('moment_reference_frd_m'),float)
            if ref.shape!=(3,) or not np.isfinite(ref).all():raise ValueError('Missing aerodynamic moment reference')
            self.coeff = RegularGridInterpolator(self.axes, d["coeff"], bounds_error=True)
            self.rates = RegularGridInterpolator(self.axes, d["rates"], bounds_error=True)
            self.controls = RegularGridInterpolator(self.axes, d["controls"], bounds_error=True)

    def assert_compatible(self,config):
        geometry=Path(config['_root'])/config['aero']['geometry']
        if hashlib.sha256(geometry.read_bytes()).hexdigest()!=self.metadata.get('geometry_sha256'):
            raise ValueError('AERO_MISMATCH: case geometry differs from the actual aerodynamic database')
        backend=config['aero'].get('backend','avl').lower()
        actual=self.metadata.get('backend') or self.metadata.get('solver','').lower().split(':')[0].split(' ')[0]
        if backend != actual:
            raise ValueError('AERO_MISMATCH: configured solver differs from loaded aerodynamic database')
        if backend=='hybrid':
            from .hybrid import check_reference,validate_composition
            composition=validate_composition(config['aero'].get('hybrid'),config['aero'].get('rate_derivative_closure'))
            if composition!=self.metadata.get('composition'):raise ValueError('복합 공력표의 블록 출처가 설정과 다릅니다. 새로 계산하세요.')
            check_reference(config)
            if not np.allclose(self.metadata['moment_reference_frd_m'],config['aircraft']['cg_m'],rtol=0,atol=1e-9):
                raise ValueError('복합 공력표의 기준점이 현재 CG와 다릅니다.')
        if backend in ('hybrid','flow5') and 'flow5_speed_m_s' in self.metadata:
            if abs(self.metadata['flow5_speed_m_s']-config['flight']['speed_m_s'])>1e-8:
                raise ValueError('flow5 공력표와 현재 속도가 다릅니다. 새로 계산하세요.')
        fingerprints=self.metadata.get('input_files_sha256')
        if fingerprints is not None and input_fingerprint(geometry)!=fingerprints:
            raise ValueError('AERO_MISMATCH: referenced airfoil/body input differs from the aerodynamic database')
        controls=self.metadata.get('control_indices')
        if controls is not None and controls!={'elevator':config['aero']['elevator_index'],'lateral':list(config['aero']['lateral_control_indices'])}:
            raise ValueError('AERO_MISMATCH: control indices differ from the aerodynamic database')

    def evaluate(self, alpha, beta, elevator, omega, speed, lateral=(0., 0.)):
        if not np.isfinite([alpha,beta,elevator,speed]).all() or speed<=0:
            raise ValueError('AERO_DOMAIN: finite angles and positive speed required')
        if np.shape(omega)!=(3,) or np.shape(lateral)!=(2,) or not np.isfinite(omega).all() or not np.isfinite(lateral).all():
            raise ValueError('AERO_DOMAIN: invalid rates or lateral controls')
        point = np.array([[np.rad2deg(alpha), np.rad2deg(beta), elevator]])
        c = self.coeff(point)[0]
        _, chord, span = self.refs
        normalized = np.asarray(omega)*np.array([span, chord, span])/(2*speed)
        if np.any(np.abs(normalized)>np.array([.10, .03, .25])):
            raise ValueError("AERO_DOMAIN: angular rate exceeds the configured small-rate model envelope")
        return c+self.rates(point)[0]@normalized+self.controls(point)[0]@np.asarray(lateral)
