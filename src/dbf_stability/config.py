from pathlib import Path
import copy
import numbers
import numpy as np
import yaml
import pandas as pd
from .control import validate_controller


def load_case(path, *, root=None):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(cfg, dict):
        raise ValueError('Case must be a mapping')
    project = root or cfg.get('_root')
    if project is None:
        project = next(
            (
                parent
                for parent in path.parents
                if (parent / 'src/dbf_stability').is_dir() and (parent / 'pyproject.toml').is_file()
            ),
            path.parent.parent,
        )
    cfg["_root"] = str(Path(project).resolve())
    cfg["_source"] = str(path)
    # CSV history files have exactly time_s,value columns, relative to case file.
    for key in ("door_schedule", "length_schedule"):
        history = cfg.get("winch", {}).get(key)
        if isinstance(history, str):
            table = pd.read_csv(path.parent / history)
            if list(table.columns) != ["time_s", "value"]:
                raise ValueError(f"{history}: expected time_s,value headers")
            cfg["winch"][key] = table.to_numpy(float).tolist()
    validate(cfg)
    return cfg


REQUIRED_SECTIONS = ("aircraft", "sensor", "cable", "bay", "winch", "flight", "simulation", "provenance", "aero")
CONTROL_FIELDS = {'elevator_delta_deg', 'thrust_delta_N', 'aileron_deg', 'rudder_deg'}


def validate(c):
    """Reject a case that any later stage could misread. Every check raises ValueError with the field path."""
    if not isinstance(c, dict):
        raise ValueError('Case must be a mapping')
    _finite_tree(c)
    for key in REQUIRED_SECTIONS:
        if key not in c:
            raise ValueError(f"Missing required section: {key}")
    point_mass = c['sensor'].get('model') == 'point_mass'
    _validate_bodies(c, point_mass)
    _validate_cable(c)
    _validate_bay(c)
    _validate_winch(c)
    _validate_flight(c)
    _validate_simulation(c)
    _validate_aero(c)
    for section in ("aircraft", "sensor", "cable", "bay", "winch", "flight", "aero"):
        if section not in c["provenance"]:
            raise ValueError(f"Missing provenance: {section}")
    _validate_collision(c.get('collision', {}))
    return c


def _finite_tree(value, path='case'):
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_tree(item, f'{path}.{key}')
    elif isinstance(value, (list, tuple, np.ndarray)):
        for i, item in enumerate(value):
            _finite_tree(item, f'{path}[{i}]')
    elif isinstance(value, numbers.Real) and not np.isfinite(value):
        raise ValueError(f'{path} must be finite')


def _vector3(c, section, key):
    value = np.asarray(c[section][key], float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f'{section}.{key} must be a finite 3-vector')


def _positive(c, section, keys):
    for key in keys:
        if c[section][key] <= 0:
            raise ValueError(f'{section}.{key} must be positive')


def _nonnegative(c, section, keys):
    for key in keys:
        if not np.isfinite(c[section][key]) or c[section][key] < 0:
            raise ValueError(f'{section}.{key} must be finite and nonnegative')


def _validate_bodies(c, point_mass):
    """Aircraft and sensor mass properties, attachment points and drag inputs."""
    if point_mass and set(c['sensor']) != {'model', 'mass_kg', 'tow_point_m'}:
        raise ValueError('Point mass accepts only model, mass_kg and zero tow_point_m')
    if point_mass and np.any(np.asarray(c['sensor']['tow_point_m']) != 0):
        raise ValueError('Point mass attachment must be at its center')
    for name in ("aircraft", "sensor"):
        b = c[name]
        if not np.isfinite(b['mass_kg']) or b["mass_kg"] <= 0:
            raise ValueError(f"{name}.mass_kg must be positive")
        if name == 'sensor' and point_mass:
            continue
        I = np.asarray(b["inertia_kgm2"], float)
        if (
            I.shape != (3, 3)
            or not np.isfinite(I).all()
            or not np.allclose(I, I.T)
            or np.min(np.linalg.eigvalsh(I)) <= 0
        ):
            raise ValueError(f"Invalid {name} inertia tensor")
    for key in ('cg_m', 'tow_point_m', 'thrust_point_m'):
        _vector3(c, 'aircraft', key)
    _positive(c, 'aircraft', ['max_thrust_N'])
    _vector3(c, 'sensor', 'tow_point_m')
    if not point_mass:
        _vector3(c, 'sensor', 'rate_damping')
        _positive(c, 'sensor', ['length_m', 'span_m', 'area_m2'])
    for section, key in (("aircraft", "profile_cd"), ("sensor", "cd")):
        if section == 'sensor' and point_mass:
            continue
        if key not in c[section] or c[section][key] < 0:
            raise ValueError(f"Explicit nonnegative {section}.{key} required")
    if 'mesh_radius_m' in c['sensor'] and (
        not np.isfinite(c['sensor']['mesh_radius_m']) or c['sensor']['mesh_radius_m'] < 0
    ):
        raise ValueError('sensor.mesh_radius_m must be finite and nonnegative')


def _validate_cable(c):
    cable = c['cable']
    for key in ("length_m", "diameter_m", "density_kg_m", "EA_N", "damping_engagement_strain"):
        if cable[key] <= 0:
            raise ValueError(f"cable.{key} must be positive")
    if isinstance(cable['segments'], bool) or cable["segments"] < 2 or int(cable['segments']) != cable['segments']:
        raise ValueError("At least 2 material cable cells required")
    _nonnegative(c, 'cable', ['damping_Ns_m', 'spool_k_N_m', 'spool_c_Ns_m'])
    for key in ("cd_normal", "cd_tangent"):
        if key not in cable or cable[key] < 0:
            raise ValueError(f"Explicit nonnegative cable.{key} required")
    guides = np.asarray(cable.get('guide_points_body_m', []), float)
    if guides.size and (guides.ndim != 2 or guides.shape[1] != 3 or not np.isfinite(guides).all() or len(guides) > 16):
        raise ValueError('Guide positions require at most 16 finite body-frame 3-vectors')


def _validate_bay(c):
    """Stowage bay geometry, contact and latch parameters."""
    bay = c['bay']
    _vector3(c, 'bay', 'stowed_center_m')
    _positive(
        c,
        'bay',
        [
            'half_width_m',
            'door_length_m',
            'door_thickness_m',
            'wall_thickness_m',
            'contact_radius_m',
            'capture_radius_m',
            'capture_speed_m_s',
            'capture_angle_deg',
        ],
    )
    if bay['front_x_m'] <= bay['exit_x_m'] or bay['floor_z_m'] <= bay['ceiling_z_m']:
        raise ValueError('Bay front/exit or floor/ceiling order is invalid in FRD')
    _nonnegative(
        c,
        'bay',
        [
            'contact_k_N_m',
            'contact_c_Ns_m',
            'friction',
            'latch_k_N_m',
            'latch_c_Ns_m',
            'latch_kr_Nm_rad',
            'latch_cr_Nms_rad',
        ],
    )
    for key in ('door_hinge_offset_m', 'door_closed_offset_m'):
        offset = np.asarray(bay.get(key, [0.0, 0.0, 0.0]), float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError(f'bay.{key} must be a finite FRD 3-vector in metres')
    q = np.asarray(bay.get('stowed_quaternion_wxyz', [1.0, 0.0, 0.0, 0.0]), float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-6:
        raise ValueError('Stored sensor orientation must be a unit scalar-first quaternion')
    if 'door_mesh_radius_m' in bay and (not np.isfinite(bay['door_mesh_radius_m']) or bay['door_mesh_radius_m'] < 0):
        raise ValueError('bay.door_mesh_radius_m must be finite and nonnegative')
    push_until = bay.get('release_push_until_s')
    if push_until is not None and (not np.isfinite(push_until) or push_until < c['winch']['release_s']):
        raise ValueError('bay.release_push_until_s must be finite and after release')


def _validate_winch(c):
    """Winch geometry and the door/length command histories."""
    winch = c['winch']
    _positive(c, 'winch', ['radius_m'])
    if not 0 < winch["stowed_length_m"] <= c["cable"]["length_m"]:
        raise ValueError("Invalid stowed cable length")
    for name in ("door_schedule", "length_schedule"):
        a = np.asarray(winch[name], float)
        if a.ndim != 2 or a.shape[1] != 2 or len(a) < 2 or not np.isfinite(a).all() or np.any(np.diff(a[:, 0]) <= 0):
            raise ValueError(f"winch.{name}: increasing time,value rows required")
    if 'capture_enabled' in winch and not isinstance(winch['capture_enabled'], bool):
        raise ValueError('winch.capture_enabled must be boolean')
    if winch.get('door_capture_interlock', False):
        if not winch.get('capture_enabled', True):
            raise ValueError('Door capture interlock requires capture_enabled')
        rows = np.asarray(winch['door_schedule'], float)
        closing = np.flatnonzero(np.diff(rows[:, 1]) < 0)
        if (
            not len(closing)
            or rows[closing[0], 0] < winch['recovery_start_s']
            or np.any(np.diff(rows[closing[0] :, 1]) > 0)
        ):
            raise ValueError('Interlocked door requires one final monotone closing sequence after recovery starts')
        hold = winch.get('door_capture_hold_s', 0.0)
        if not np.isfinite(hold) or hold < 0:
            raise ValueError('winch.door_capture_hold_s must be finite and nonnegative')
    lengths = np.array(winch["length_schedule"])[:, 1]
    if np.min(lengths) < winch["stowed_length_m"] or np.max(lengths) > c["cable"]["length_m"]:
        raise ValueError("Commanded cable length outside physical bounds")


def _validate_flight(c):
    """Atmosphere, wind, gust, open-loop control tables and the closed-loop controller."""
    flight = c['flight']
    _vector3(c, 'flight', 'wind_ned_m_s')
    if flight['g_m_s2'] < 0 or flight['local_flow_factor'] < 0:
        raise ValueError('Gravity and local flow factor must be nonnegative')
    controls = flight.get('controls', {})
    if not isinstance(controls, dict) or set(controls) - CONTROL_FIELDS:
        raise ValueError(
            'Unsupported flight.controls field; use elevator_delta_deg, thrust_delta_N, aileron_deg, rudder_deg'
        )
    gust = flight.get('gust')
    if gust is not None:
        if not isinstance(gust, dict) or set(gust) != {'start_s', 'duration_s', 'velocity_ned_m_s'}:
            raise ValueError('Gust requires start_s, duration_s and velocity_ned_m_s')
        if gust['start_s'] < 0 or gust['duration_s'] <= 0 or np.asarray(gust['velocity_ned_m_s']).shape != (3,):
            raise ValueError('Invalid gust time or velocity vector')
    for name, rows in controls.items():
        table = np.asarray(rows, float)
        if table.ndim != 2 or table.shape[1] != 2 or len(table) < 2 or np.any(np.diff(table[:, 0]) <= 0):
            raise ValueError(f'flight.controls.{name} requires increasing time,value rows')
    if flight["speed_m_s"] <= 1 or flight["rho_kg_m3"] <= 0:
        raise ValueError("Invalid flight speed/density")
    if flight.get('cable_bay_shielding', False):
        factor = flight.get('internal_cable_flow_factor')
        if factor is None or not np.isfinite(factor) or not 0 <= factor <= 1:
            raise ValueError('Shielded cable requires an explicit internal flow factor in [0,1]')
    validate_controller(flight, c['aircraft'], c['aero'])


def _validate_simulation(c):
    sim = c['simulation']
    value = sim.get('jacobian_workers', 1)
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < 1:
        raise ValueError('simulation.jacobian_workers must be a positive integer')
    if sim.get('jacobian_step_factor', 1.0) <= 0:
        raise ValueError('simulation.jacobian_step_factor must be positive')
    for key in ("duration_s", "sample_dt_s", "max_step_s", "rtol", "atol"):
        if sim[key] <= 0:
            raise ValueError(f"simulation.{key} must be positive")
    for key in (
        'contact_max_step_s',
        'maximum_runtime_s',
        'stagnation_window_s',
        'stagnation_min_advance_s',
        'checkpoint_interval_s',
        'maximum_rhs_evaluations',
    ):
        value = sim.get(key)
        if value is not None and (not np.isfinite(value) or value <= 0):
            raise ValueError(f'simulation.{key} must be finite and positive')


def _validate_aero(c):
    for key in ('alpha_deg', 'beta_deg', 'elevator_deg'):
        grid = np.asarray(c['aero'][key], float)
        if grid.ndim != 1 or grid.size < 2 or np.any(np.diff(grid) <= 0):
            raise ValueError(f'aero.{key} must be strictly increasing')


def _validate_collision(contact):
    """Optional mesh/analytic contact block; only checked when present."""
    analytic = contact.get('analytic_conical_stop')
    if analytic:
        for key, length in [('nose_center_sensor_m', 3), ('center_yz_m', 2)]:
            if np.asarray(analytic[key]).shape != (length,) or not np.isfinite(analytic[key]).all():
                raise ValueError('Invalid analytic conical stop reference')
        if (
            not analytic['nose_radius_m'] > 0
            or not analytic['mouth_radius_m'] > analytic['bore_radius_m'] > 0
            or not analytic['front_x_m'] > analytic['back_x_m']
            or not 16 <= analytic['quadrature_order'] <= 2048
            or int(analytic['quadrature_order']) != analytic['quadrature_order']
        ):
            raise ValueError('Invalid analytic conical stop geometry or quadrature')
    for name, material in contact.get('part_materials', {}).items():
        for key in ('stiffness_N_m', 'damping_Ns_m'):
            if (
                key not in material
                or not np.isfinite(material[key])
                or material[key] < 0
                or (key == 'stiffness_N_m' and material[key] == 0)
            ):
                raise ValueError(
                    f'collision.part_materials.{name}.{key} must be explicit, finite and physically admissible'
                )
    if contact.get('enabled'):
        for key in (
            'skin_m',
            'minimum_gap_m',
            'stiffness_N_m',
            'damping_Ns_m',
            'maximum_surface_travel_m',
            'cable_contact_regularization_m',
        ):
            if not np.isfinite(contact[key]) or contact[key] <= 0:
                raise ValueError(f'collision.{key} must be finite and positive')
        if contact['minimum_gap_m'] >= contact['skin_m']:
            raise ValueError('Collision safety gap must be smaller than contact skin')
        for key in ('maximum_free_surface_travel_m', 'clearance_query_m'):
            if key in contact and (not np.isfinite(contact[key]) or contact[key] <= 0):
                raise ValueError(f'collision.{key} must be finite and positive')


def changed(c, **changes):
    out = copy.deepcopy(c)
    for path, value in changes.items():
        dest = out
        keys = path.split(".")
        for key in keys[:-1]:
            dest = dest[key]
        dest[keys[-1]] = value
    validate(out)
    return out
