"""Interpolated aerodynamic coefficient tables and their compatibility checks."""

from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.interpolate import RegularGridInterpolator
from .aero_inputs import input_fingerprint
from .hybrid_rules import check_reference, validate_composition


class AeroDatabase:
    def __init__(self, path):
        self.path = str(Path(path).resolve())
        with np.load(path, allow_pickle=False) as d:
            self.axes = tuple(d[k].copy() for k in ("alpha", "beta", "elevator"))
            self.refs = d["refs"].copy()
            self.metadata = json.loads(str(d["metadata"]))
            if self.refs.shape != (3,) or not np.isfinite(self.refs).all() or np.any(self.refs <= 0):
                raise ValueError('Invalid aerodynamic reference dimensions')
            if any(x.ndim != 1 or len(x) < 2 or not np.isfinite(x).all() or np.any(np.diff(x) <= 0) for x in self.axes):
                raise ValueError('Invalid aerodynamic interpolation axes')
            for key, tail in [('coeff', (6,)), ('rates', (6, 3)), ('controls', (6, 2))]:
                if d[key].shape != tuple(map(len, self.axes)) + tail or not np.isfinite(d[key]).all():
                    raise ValueError(f'Invalid/nonfinite aerodynamic {key} table')
            for key, value in [('axes', 'FRD body'), ('angle_unit', 'degrees'), ('rate_unit', 'pb/2V,qc/2V,rb/2V')]:
                if self.metadata.get(key) != value:
                    raise ValueError(f'Unsupported aerodynamic {key}')
            ref = np.asarray(self.metadata.get('moment_reference_frd_m'), float)
            if ref.shape != (3,) or not np.isfinite(ref).all():
                raise ValueError('Missing aerodynamic moment reference')
            self.coeff = RegularGridInterpolator(self.axes, d["coeff"], bounds_error=True)
            self.rates = RegularGridInterpolator(self.axes, d["rates"], bounds_error=True)
            self.controls = RegularGridInterpolator(self.axes, d["controls"], bounds_error=True)

    def assert_compatible(self, config):
        geometry = Path(config['_root']) / config['aero']['geometry']
        if hashlib.sha256(geometry.read_bytes()).hexdigest() != self.metadata.get('geometry_sha256'):
            raise ValueError('AERO_MISMATCH: case geometry differs from the actual aerodynamic database')
        backend = config['aero'].get('backend', 'avl').lower()
        actual = self.metadata.get('backend') or self.metadata.get('solver', '').lower().split(':')[0].split(' ')[0]
        if backend != actual:
            raise ValueError('AERO_MISMATCH: configured solver differs from loaded aerodynamic database')
        if backend == 'hybrid':
            composition = validate_composition(
                config['aero'].get('hybrid'), config['aero'].get('rate_derivative_closure')
            )
            if composition != self.metadata.get('composition'):
                raise ValueError('복합 공력표의 블록 출처가 설정과 다릅니다. 새로 계산하세요.')
            check_reference(config)
            if not np.allclose(self.metadata['moment_reference_frd_m'], config['aircraft']['cg_m'], rtol=0, atol=1e-9):
                raise ValueError('복합 공력표의 기준점이 현재 CG와 다릅니다.')
        if backend in ('hybrid', 'flow5') and 'flow5_speed_m_s' in self.metadata:
            if abs(self.metadata['flow5_speed_m_s'] - config['flight']['speed_m_s']) > 1e-8:
                raise ValueError('flow5 공력표와 현재 속도가 다릅니다. 새로 계산하세요.')
        fingerprints = self.metadata.get('input_files_sha256')
        if fingerprints is not None and input_fingerprint(geometry) != fingerprints:
            raise ValueError('AERO_MISMATCH: referenced airfoil/body input differs from the aerodynamic database')
        controls = self.metadata.get('control_indices')
        if controls is not None and controls != {
            'elevator': config['aero']['elevator_index'],
            'lateral': list(config['aero']['lateral_control_indices']),
        }:
            raise ValueError('AERO_MISMATCH: control indices differ from the aerodynamic database')

    def evaluate(self, alpha, beta, elevator, omega, speed, lateral=(0.0, 0.0)):
        if not np.isfinite([alpha, beta, elevator, speed]).all() or speed <= 0:
            raise ValueError('AERO_DOMAIN: finite angles and positive speed required')
        if (
            np.shape(omega) != (3,)
            or np.shape(lateral) != (2,)
            or not np.isfinite(omega).all()
            or not np.isfinite(lateral).all()
        ):
            raise ValueError('AERO_DOMAIN: invalid rates or lateral controls')
        point = np.array([[np.rad2deg(alpha), np.rad2deg(beta), elevator]])
        c = self.coeff(point)[0]
        _, chord, span = self.refs
        normalized = np.asarray(omega) * np.array([span, chord, span]) / (2 * speed)
        if np.any(np.abs(normalized) > np.array([0.10, 0.03, 0.25])):
            raise ValueError("AERO_DOMAIN: angular rate exceeds the configured small-rate model envelope")
        return c + self.rates(point)[0] @ normalized + self.controls(point)[0] @ np.asarray(lateral)
