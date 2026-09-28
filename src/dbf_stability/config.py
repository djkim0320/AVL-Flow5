from pathlib import Path
import copy
import numbers
import numpy as np
import yaml


def load_case(path, *, root=None):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(cfg,dict):raise ValueError('Case must be a mapping')
    project=root or cfg.get('_root')
    if project is None:
        project=next((parent for parent in path.parents if (parent/'src/dbf_stability').is_dir() and (parent/'pyproject.toml').is_file()),path.parent.parent)
    cfg["_root"] = str(Path(project).resolve())
    cfg["_source"] = str(path)
    # CSV history files have exactly time_s,value columns, relative to case file.
    for key in ("door_schedule", "length_schedule"):
        history = cfg.get("winch", {}).get(key)
        if isinstance(history, str):
            import pandas as pd
            table = pd.read_csv(path.parent/history)
            if list(table.columns) != ["time_s", "value"]:
                raise ValueError(f"{history}: expected time_s,value headers")
            cfg["winch"][key] = table.to_numpy(float).tolist()
    validate(cfg)
    return cfg


def validate(c):
    if not isinstance(c,dict):
        raise ValueError('Case must be a mapping')
    def finite_tree(value,path='case'):
        if isinstance(value,dict):
            for key,item in value.items():finite_tree(item,f'{path}.{key}')
        elif isinstance(value,(list,tuple,np.ndarray)):
            for i,item in enumerate(value):finite_tree(item,f'{path}[{i}]')
        elif isinstance(value,numbers.Real) and not np.isfinite(value):
            raise ValueError(f'{path} must be finite')
    finite_tree(c)
    for key in ("aircraft", "sensor", "cable", "bay", "winch", "flight", "simulation", "provenance", "aero"):
        if key not in c:
            raise ValueError(f"Missing required section: {key}")
    point_mass = c['sensor'].get('model') == 'point_mass'
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
        if I.shape != (3, 3) or not np.isfinite(I).all() or not np.allclose(I, I.T) or np.min(np.linalg.eigvalsh(I)) <= 0:
            raise ValueError(f"Invalid {name} inertia tensor")
    for key in ("length_m", "diameter_m", "density_kg_m", "EA_N", "damping_engagement_strain"):
        if c["cable"][key] <= 0:
            raise ValueError(f"cable.{key} must be positive")
    if isinstance(c['cable']['segments'],bool) or c["cable"]["segments"] < 2 or int(c['cable']['segments'])!=c['cable']['segments']:
        raise ValueError("At least 2 material cable cells required")
    for section,keys in [('aircraft',['cg_m','tow_point_m','thrust_point_m']),
                         ('sensor',['tow_point_m','rate_damping']),('bay',['stowed_center_m']),
                         ('flight',['wind_ned_m_s'])]:
        for key in keys:
            if section == 'sensor' and point_mass and key == 'rate_damping':
                continue
            value=np.asarray(c[section][key],float)
            if value.shape!=(3,) or not np.isfinite(value).all():
                raise ValueError(f'{section}.{key} must be a finite 3-vector')
    for section,keys in [('sensor',['length_m','span_m','area_m2']),('aircraft',['max_thrust_N']),
                         ('winch',['radius_m']),('bay',['half_width_m','door_length_m','door_thickness_m',
                         'wall_thickness_m','contact_radius_m','capture_radius_m','capture_speed_m_s','capture_angle_deg'])]:
        for key in keys:
            if section == 'sensor' and point_mass:
                continue
            if c[section][key]<=0:raise ValueError(f'{section}.{key} must be positive')
    if c['flight']['g_m_s2']<0 or c['flight']['local_flow_factor']<0:
        raise ValueError('Gravity and local flow factor must be nonnegative')
    if c['bay']['front_x_m']<=c['bay']['exit_x_m'] or c['bay']['floor_z_m']<=c['bay']['ceiling_z_m']:
        raise ValueError('Bay front/exit or floor/ceiling order is invalid in FRD')
    for key in ('alpha_deg','beta_deg','elevator_deg'):
        grid=np.asarray(c['aero'][key],float)
        if grid.ndim!=1 or grid.size<2 or np.any(np.diff(grid)<=0):
            raise ValueError(f'aero.{key} must be strictly increasing')
    controls=c['flight'].get('controls',{})
    if not isinstance(controls,dict) or set(controls)-{'elevator_delta_deg','thrust_delta_N','aileron_deg','rudder_deg'}:
        raise ValueError('Unsupported flight.controls field; use elevator_delta_deg, thrust_delta_N, aileron_deg, rudder_deg')
    gust=c['flight'].get('gust')
    if gust is not None:
        if not isinstance(gust,dict) or set(gust)!={'start_s','duration_s','velocity_ned_m_s'}:
            raise ValueError('Gust requires start_s, duration_s and velocity_ned_m_s')
        if gust['start_s']<0 or gust['duration_s']<=0 or np.asarray(gust['velocity_ned_m_s']).shape!=(3,):
            raise ValueError('Invalid gust time or velocity vector')
    for name,rows in controls.items():
        table=np.asarray(rows,float)
        if table.ndim!=2 or table.shape[1]!=2 or len(table)<2 or np.any(np.diff(table[:,0])<=0):
            raise ValueError(f'flight.controls.{name} requires increasing time,value rows')
    for key in ('jacobian_workers',):
        value=c['simulation'].get(key,1)
        if isinstance(value,bool) or not isinstance(value,numbers.Integral) or value<1:
            raise ValueError(f'simulation.{key} must be a positive integer')
    if c['simulation'].get('jacobian_step_factor',1.)<=0:
        raise ValueError('simulation.jacobian_step_factor must be positive')
    for section,keys in [('cable',['damping_Ns_m','spool_k_N_m','spool_c_Ns_m']),
                         ('bay',['contact_k_N_m','contact_c_Ns_m','friction','latch_k_N_m','latch_c_Ns_m','latch_kr_Nm_rad','latch_cr_Nms_rad'])]:
        for key in keys:
            if not np.isfinite(c[section][key]) or c[section][key]<0:
                raise ValueError(f'{section}.{key} must be finite and nonnegative')
    for section, key in (("aircraft", "profile_cd"), ("sensor", "cd"), ("cable", "cd_normal"), ("cable", "cd_tangent")):
        if section == 'sensor' and point_mass:
            continue
        if key not in c[section] or c[section][key] < 0:
            raise ValueError(f"Explicit nonnegative {section}.{key} required")
    if not 0 < c["winch"]["stowed_length_m"] <= c["cable"]["length_m"]:
        raise ValueError("Invalid stowed cable length")
    for name in ("door_schedule", "length_schedule"):
        a = np.asarray(c["winch"][name], float)
        if a.ndim != 2 or a.shape[1] != 2 or len(a)<2 or not np.isfinite(a).all() or np.any(np.diff(a[:, 0]) <= 0):
            raise ValueError(f"winch.{name}: increasing time,value rows required")
    if c['winch'].get('door_capture_interlock',False):
        rows=np.asarray(c['winch']['door_schedule'],float)
        closing=np.flatnonzero(np.diff(rows[:,1])<0)
        if not len(closing) or rows[closing[0],0]<c['winch']['recovery_start_s'] or np.any(np.diff(rows[closing[0]:,1])>0):
            raise ValueError('Interlocked door requires one final monotone closing sequence after recovery starts')
        hold=c['winch'].get('door_capture_hold_s',0.)
        if not np.isfinite(hold) or hold<0:
            raise ValueError('winch.door_capture_hold_s must be finite and nonnegative')
    lengths = np.array(c["winch"]["length_schedule"])[:, 1]
    if np.min(lengths) < c["winch"]["stowed_length_m"] or np.max(lengths) > c["cable"]["length_m"]:
        raise ValueError("Commanded cable length outside physical bounds")
    for section in ("aircraft", "sensor", "cable", "bay", "winch", "flight", "aero"):
        if section not in c["provenance"]:
            raise ValueError(f"Missing provenance: {section}")
    for key in ("duration_s", "sample_dt_s", "max_step_s", "rtol", "atol"):
        if c["simulation"][key] <= 0:
            raise ValueError(f"simulation.{key} must be positive")
    if 'contact_max_step_s' in c['simulation']:
        value=c['simulation']['contact_max_step_s']
        if not np.isfinite(value) or value<=0:
            raise ValueError('simulation.contact_max_step_s must be finite and positive')
    for key in ('maximum_runtime_s','stagnation_window_s','stagnation_min_advance_s','checkpoint_interval_s','maximum_rhs_evaluations'):
        value=c['simulation'].get(key)
        if value is not None and (not np.isfinite(value) or value<=0):
            raise ValueError(f'simulation.{key} must be finite and positive')
    if c["flight"]["speed_m_s"] <= 1 or c["flight"]["rho_kg_m3"] <= 0:
        raise ValueError("Invalid flight speed/density")
    if c['flight'].get('cable_bay_shielding',False):
        factor=c['flight'].get('internal_cable_flow_factor')
        if factor is None or not np.isfinite(factor) or not 0<=factor<=1:
            raise ValueError('Shielded cable requires an explicit internal flow factor in [0,1]')
    from .control import validate_controller
    validate_controller(c['flight'], c['aircraft'], c['aero'])
    for key in ('door_hinge_offset_m','door_closed_offset_m'):
        offset=np.asarray(c['bay'].get(key,[0.,0.,0.]),float)
        if offset.shape!=(3,) or not np.isfinite(offset).all():
            raise ValueError(f'bay.{key} must be a finite FRD 3-vector in metres')
    q=np.asarray(c['bay'].get('stowed_quaternion_wxyz',[1.,0.,0.,0.]),float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6:
        raise ValueError('Stored sensor orientation must be a unit scalar-first quaternion')
    guides=np.asarray(c['cable'].get('guide_points_body_m',[]),float)
    if guides.size and (guides.ndim!=2 or guides.shape[1]!=3 or not np.isfinite(guides).all() or len(guides)>16):
        raise ValueError('Guide positions require at most 16 finite body-frame 3-vectors')
    for section,key in [('bay','door_mesh_radius_m'),('sensor','mesh_radius_m')]:
        if key in c[section] and (not np.isfinite(c[section][key]) or c[section][key]<0):
            raise ValueError(section+'.'+key+' must be finite and nonnegative')
    push_until=c['bay'].get('release_push_until_s')
    if push_until is not None and (not np.isfinite(push_until) or push_until<c['winch']['release_s']):
        raise ValueError('bay.release_push_until_s must be finite and after release')
    contact=c.get('collision',{})
    analytic=contact.get('analytic_conical_stop')
    if analytic:
        for key,length in [('nose_center_sensor_m',3),('center_yz_m',2)]:
            if np.asarray(analytic[key]).shape!=(length,) or not np.isfinite(analytic[key]).all():
                raise ValueError('Invalid analytic conical stop reference')
        if (not analytic['nose_radius_m']>0 or not analytic['mouth_radius_m']>analytic['bore_radius_m']>0
                or not analytic['front_x_m']>analytic['back_x_m']
                or not 16<=analytic['quadrature_order']<=2048 or int(analytic['quadrature_order'])!=analytic['quadrature_order']):
            raise ValueError('Invalid analytic conical stop geometry or quadrature')
    for name,material in contact.get('part_materials',{}).items():
        for key in ('stiffness_N_m','damping_Ns_m'):
            if key not in material or not np.isfinite(material[key]) or material[key]<0 or (key=='stiffness_N_m' and material[key]==0):
                raise ValueError(f'collision.part_materials.{name}.{key} must be explicit, finite and physically admissible')
    if contact.get('enabled'):
        for key in ('skin_m','minimum_gap_m','stiffness_N_m','damping_Ns_m','maximum_surface_travel_m','cable_contact_regularization_m'):
            if not np.isfinite(contact[key]) or contact[key]<=0:
                raise ValueError(f'collision.{key} must be finite and positive')
        if contact['minimum_gap_m']>=contact['skin_m']:
            raise ValueError('Collision safety gap must be smaller than contact skin')
        for key in ('maximum_free_surface_travel_m','clearance_query_m'):
            if key in contact and (not np.isfinite(contact[key]) or contact[key]<=0):
                raise ValueError(f'collision.{key} must be finite and positive')
    return c


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
