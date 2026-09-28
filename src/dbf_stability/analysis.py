from pathlib import Path
from dataclasses import dataclass
from contextlib import ExitStack, closing
from concurrent.futures import ProcessPoolExecutor
import copy
import json
import hashlib
import os
import time
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.integrate import solve_ivp
from .model import CoupledModel
from .math3d import rotation, quaternion, euler
from .avl import AeroDatabase
from .config import changed


@dataclass
class SimulationResult:
    time: np.ndarray
    states: np.ndarray
    table: pd.DataFrame
    summary: dict
    events: list
    config: dict

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.table.to_csv(directory/"timeseries.csv", index=False)
        np.savez_compressed(directory/"states.npz", time=self.time, states=self.states)
        (directory/"summary.json").write_text(json.dumps(self.summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf8")
        (directory/"events.json").write_text(json.dumps(self.events, ensure_ascii=False, indent=2), encoding="utf8")
        (directory/"inputs.json").write_text(json.dumps(self.config, ensure_ascii=False, indent=2), encoding="utf8")
        return directory


def load_result(directory, *, apply_geometry_audit=True):
    directory=Path(directory)
    summary=json.loads((directory/'summary.json').read_text(encoding='utf8'))
    if summary.get('status') in ('running','failed','cancelled'):
        raise ValueError('This run has no current completed/partial trajectory; old files must not be reused')
    with np.load(directory/'states.npz',allow_pickle=False) as d:
        times,states=d['time'].copy(),d['states'].copy()
    table=pd.read_csv(directory/'timeseries.csv')
    if len(table)!=len(times) or not np.allclose(table.time_s,times):
        raise ValueError('Result files disagree on sample times')
    audit_path = directory / 'cad_collision_audit.json'
    if apply_geometry_audit and audit_path.exists():
        audit = json.loads(audit_path.read_text(encoding='utf8'))
        for name in ('states', 'inputs'):
            file = directory / ('states.npz' if name == 'states' else 'inputs.json')
            expected = audit.get(f'source_{name}_sha256')
            if expected is None or hashlib.sha256(file.read_bytes()).hexdigest() != expected:
                raise ValueError('CAD collision audit is stale or lacks input hashes; rerun the audit')
        summary['geometry_validity'] = audit['geometry_validity']
        if audit['geometry_validity'] == 'invalid':
            summary['integration_status'] = summary.get('integration_status', summary['status'])
            summary['status'] = 'geometry_invalid'
            summary['physical_validation'] = 'invalid_geometry'
            summary['message'] = 'Saved trajectory intersects delivered CAD; not valid for design loads or recovery evaluation.'
            summary['first_sampled_collision'] = audit['first_sampled_intersection']
        # Absence of sampled overlap is explicitly not a continuous-contact proof.
    else:
        summary.setdefault('geometry_validity', 'not_audited')
    return SimulationResult(times,states,table,summary,
                            json.loads((directory/'events.json').read_text(encoding='utf8')),
                            json.loads((directory/'inputs.json').read_text(encoding='utf8')))


def solve_trim(config, aero, mode="deployed", length=None):
    # Equilibrium is an open-loop operating point. Resolve feedback direction
    # only against the resulting trim, never the optimizer's placeholder state.
    if (config['flight'].get('controller') or {}).get('enabled'):
        config=copy.deepcopy(config)
        config['flight']['controller']['enabled']=False
    with ExitStack() as resources:
        return _solve_trim(config,aero,mode,length,resources)


def _solve_trim(config,aero,mode,length,resources):
    """Symmetric straight-level trim, with all cable/sensor equilibrium unknowns.

    Lateral offsets require asymmetric trim and are explicitly rejected in v0.1.
    Longitudinal/vertical attachment sweeps and full 3D perturbations are supported.
    """
    if mode not in ("deployed", "stowed", "aircraft_only"):
        raise ValueError("Unknown trim mode")
    for name in (("aircraft", "sensor") if mode=='deployed' else ()):
        # CAD-derived symmetric mass properties can carry sub-micrometre kernel noise.
        if abs(config[name]["tow_point_m"][1]) > 1e-7:
            raise ValueError("Asymmetric trim not implemented: nonzero lateral tow offset")
    if np.linalg.norm(config["flight"]["wind_ned_m_s"]) > 1e-10:
        raise ValueError("Trim requires zero base wind; use gusts in transient cases")
    if mode != "deployed":
        model = resources.enter_context(closing(CoupledModel(config, aero, phase="aircraft_only")))
        extra_mass = config["sensor"]["mass_kg"]+config["cable"]["density_kg_m"]*config["cable"]["length_m"] if mode=="stowed" else 0.
        def residual(u):
            model.trim = {"alpha_rad": u[0], "elevator_deg": u[1], "thrust_N": u[2]}
            y = model.initial()
            f, m, _ = model.aircraft_loads(0., y[:13])
            f += np.array([0, 0, (model.ma+extra_mass)*config["flight"]["g_m_s2"]])
            r = rotation(y[6:10])
            if mode=="stowed":
                sensor_weight = r.T@np.array([0, 0, config["sensor"]["mass_kg"]*config["flight"]["g_m_s2"]])
                cable_weight = r.T@np.array([0, 0, (extra_mass-config["sensor"]["mass_kg"])*config["flight"]["g_m_s2"]])
                # Paid-out cells in the stowed guide route retain their actual
                # lever arms. Only the cells still on the reel sit at the feed.
                cable_center=r.T@(y[26:].reshape(model.n,6)[:,:3].mean(0)-y[:3])
                m += np.cross(config["bay"]["stowed_center_m"], sensor_weight)+np.cross(cable_center, cable_weight)
            return np.r_[f,m/np.array([aero.refs[2],aero.refs[1],aero.refs[2]])]/((model.ma+extra_mass)*9.81)
        bounds = ([np.deg2rad(aero.axes[0][0]+.1), aero.axes[2][0]+.1, 0], [np.deg2rad(aero.axes[0][-1]-.1), aero.axes[2][-1]-.1, config["aircraft"]["max_thrust_N"]])
        seed=np.clip([.06,0,2],*bounds)
        fit = least_squares(residual, seed, bounds=bounds, xtol=1e-10, ftol=1e-10, gtol=1e-10)
        if np.max(np.abs(fit.fun))>1e-5:
            raise RuntimeError(f"TRIM_INFEASIBLE: residual {fit.fun}")
        answer = {"mode": mode, "alpha_rad": float(fit.x[0]), "alpha_deg": float(np.rad2deg(fit.x[0])), "elevator_deg": float(fit.x[1]), "thrust_N": float(fit.x[2]), "residual_norm": float(np.linalg.norm(fit.fun))}
        full = resources.enter_context(closing(CoupledModel(config, aero, answer, phase=mode)))
        answer["state"] = full.initial()
        return answer

    length = config["cable"]["length_m"] if length is None else length
    model = resources.enter_context(closing(CoupledModel(config, aero, phase="deployed", fixed_length=length)))
    # Partial-spool equilibrium is supported via the same support forces as simulation.
    n, k = model.n, model.active_count(length)
    bare = solve_trim(config, aero, mode="stowed")
    if abs(length-config["cable"]["length_m"]) < 1e-9:
        return _full_length_trim(config, aero, model, bare, length)
    if model.point_mass:
        raise ValueError('Point-mass trim uses the specified full cable length; partial payout requires time response')
    u0 = [bare["alpha_rad"], bare["elevator_deg"], bare["thrust_N"]+1.]
    s = config["sensor"]
    drag = .5*config["flight"]["rho_kg_m3"]*config["flight"]["speed_m_s"]**2*s["area_m2"]*s["cd"]
    sensor_pitch = s["mass_kg"]*9.81*s["tow_point_m"][0]/max(1e-8, .5*config["flight"]["rho_kg_m3"]*config["flight"]["speed_m_s"]**2*s["area_m2"]*(s["length_m"]*(-s["cm_alpha"])+s["tow_point_m"][0]*s["cz_alpha"]))
    lift = .5*config["flight"]["rho_kg_m3"]*config["flight"]["speed_m_s"]**2*s["area_m2"]*s["cz_alpha"]*sensor_pitch
    direction = np.array([-drag, s["mass_kg"]*9.81-lift]); tension_seed=np.linalg.norm(direction); direction /= tension_seed
    anchor = np.array(config["aircraft"]["tow_point_m"])[[0, 2]]
    direction = (rotation(quaternion(pitch=bare["alpha_rad"])).T@np.array([direction[0],0,direction[1]]))[[0,2]]
    stretch = 1+tension_seed/config["cable"]["EA_N"]
    sensor_relative_rotation=rotation(quaternion(pitch=sensor_pitch-bare["alpha_rad"]))
    sensor_pos = anchor+length*stretch*direction-(sensor_relative_rotation@np.array(s["tow_point_m"]))[[0, 2]]
    u0 += [*sensor_pos, sensor_pitch]
    for j in range(n):
        p = anchor+max(0., length-(j+.5)*model.h)*stretch*direction
        u0.extend(p)

    def state(u):
        model.trim = {"alpha_rad": u[0], "elevator_deg": u[1], "thrust_N": u[2]}
        y = model.initial()
        r = rotation(y[6:10])
        y[13:16] = y[:3]+r@np.array([u[3], 0., u[4]])
        y[19:23] = quaternion(pitch=u[5])
        nodes = y[26:].reshape(n, 6)
        for j in range(n):
            nodes[j, :3] = y[:3]+r@np.array([u[6+2*j], 0, u[7+2*j]])
        return y

    def residual(u):
        dy = model.rhs(0., state(u))
        return np.r_[dy[[3, 5]], dy[11]*.1, dy[[16, 18]], dy[24]*.02,
                     dy[26:].reshape(n, 6)[:, [3, 5]].ravel()]/9.81

    lo, hi = np.full(len(u0), -np.inf), np.full(len(u0), np.inf)
    lo[:3] = [np.deg2rad(aero.axes[0][0]+.1), aero.axes[2][0]+.1, 0]
    hi[:3] = [np.deg2rad(aero.axes[0][-1]-.1), aero.axes[2][-1]-.1, config["aircraft"]["max_thrust_N"]]
    lo[5], hi[5] = -.8, .8
    fit = least_squares(residual, np.clip(u0,lo,hi), bounds=(lo, hi), x_scale="jac", max_nfev=500, xtol=1e-10, ftol=1e-10, gtol=1e-9)
    y = state(fit.x)
    dy=model.rhs(0.,y)
    error = np.max(np.abs(np.r_[dy[[3,4,5,10,11,12,16,17,18,23,24,25]],dy[26:].reshape(n,6)[:,3:].ravel()]))/9.81
    if error > 2e-4:
        raise RuntimeError(f"TRIM_INFEASIBLE: coupled equilibrium residual {error:.5g}; {fit.message}")
    return {"mode": mode, "alpha_rad": float(fit.x[0]), "alpha_deg": float(np.rad2deg(fit.x[0])),
            "elevator_deg": float(fit.x[1]), "thrust_N": float(fit.x[2]), "length_m": length,
            "residual_norm": float(np.linalg.norm(fit.fun)), "state": y}


def _full_length_trim(config, aero, model, bare, length):
    """Static shooting along cable balances every bead before aircraft trim.

    Eliminating the cable coordinates avoids an ill-conditioned many-coordinate
    least-squares problem as the spatial resolution is increased.
    """
    n, h, cable = model.n, model.h, config['cable']
    rests=np.r_[h/2,np.full(n-1,h),h/2]
    vel=np.array([config['flight']['speed_m_s'],0.,0.])
    if config['flight'].get('cable_bay_shielding',False):
        vel*=config['flight']['local_flow_factor']
    g=np.array([0.,0.,config['flight']['g_m_s2']])
    def make_state(u):
        model.trim={'alpha_rad':u[0],'elevator_deg':u[1],'thrust_N':u[2]}
        y=model.initial();y[19:23]=quaternion(pitch=0. if model.point_mass else u[3]);y[13:16]=0.
        fs,_,_=model.sensor_loads(0,y[:13],y[13:26]);external=fs+model.ms*g
        rs=rotation(y[19:23]);nose=rs@model.sensor_attach
        drags=np.zeros((n,3))
        for iteration in range(60):
            points=[nose.copy()];load=external.copy()
            for j,rest in enumerate(rests):
                tension=np.linalg.norm(load)
                direction=-load/max(tension,1e-12)
                points.append(points[-1]+rest*(1+tension/cable['EA_N'])*direction)
                if j<n:load+=model.mn*g+drags[j]
            points=np.array(points)
            updated=[]
            for j in range(n):
                direction=points[j+2]-points[j];direction/=np.linalg.norm(direction)
                vt=np.dot(vel,direction)*direction;vn=vel-vt
                updated.append(-.5*config['flight']['rho_kg_m3']*cable['diameter_m']*h*(cable['cd_normal']*np.linalg.norm(vn)*vn+cable['cd_tangent']*np.linalg.norm(vt)*vt))
            updated=np.array(updated)
            if np.max(abs(updated-drags))<1e-12:break
            drags=.5*drags+.5*updated
        anchor=y[:3]+rotation(y[6:10])@model.attach
        shift=anchor-points[-1]
        y[13:16]=shift
        y[26:].reshape(n,6)[:,:3]=points[1:-1]+shift
        return y
    def residual(u):
        y=make_state(u);dy=model.rhs(0,y)
        return (np.r_[dy[[3,5]],dy[11]*.1] if model.point_mass else np.r_[dy[[3,5]],dy[11]*.1,dy[24]*.02])/9.81
    low=[np.deg2rad(aero.axes[0][0]+.1),aero.axes[2][0]+.1,0,-.8]
    high=[np.deg2rad(aero.axes[0][-1]-.1),aero.axes[2][-1]-.1,config['aircraft']['max_thrust_N'],.8]
    if model.point_mass:
        low, high = low[:3], high[:3]
    seed=np.clip([bare['alpha_rad'],bare['elevator_deg'],bare['thrust_N']+1,.4][:len(low)],low,high)
    fit=least_squares(residual,seed,bounds=(low,high),xtol=1e-11,ftol=1e-11,gtol=1e-10)
    y=make_state(fit.x);dy=model.rhs(0,y)
    linear=np.r_[dy[3:6],dy[16:19],dy[26:].reshape(n,6)[:,3:].ravel()]
    angular=np.r_[dy[10:13],dy[23:26]]
    checks=np.r_[linear,angular]
    # Keep translational and rotational residuals in their own units. Defaults
    # preserve the former strict threshold; any relaxation must be an explicit
    # case input and remains bounded by the stability equilibrium requirement.
    lt=config['simulation'].get('trim_linear_tolerance_m_s2',1e-5)
    at=config['simulation'].get('trim_angular_tolerance_rad_s2',1e-5)
    if not np.isfinite([lt,at]).all() or min(lt,at)<=0 or max(lt,at)>2e-3:
        raise ValueError('Trim acceleration tolerances must be positive and <= 2e-3 in their declared units')
    le=float(np.max(abs(linear)));ae=float(np.max(abs(angular)))
    if le>lt or ae>at:
        raise RuntimeError(f'TRIM_INFEASIBLE: linear residual {le:g} m/s^2 (limit {lt:g}); angular {ae:g} rad/s^2 (limit {at:g})')
    return {'mode':'deployed','alpha_rad':float(fit.x[0]),'alpha_deg':float(np.rad2deg(fit.x[0])),
            'elevator_deg':float(fit.x[1]),'thrust_N':float(fit.x[2]),'length_m':length,'residual_norm':float(np.linalg.norm(checks)),
            'max_linear_acceleration_m_s2':le,'max_angular_acceleration_rad_s2':ae,
            'trim_linear_tolerance_m_s2':float(lt),'trim_angular_tolerance_rad_s2':float(at),'state':y}


def analyze_stability(config, aero, trim=None, perturbation=1e-5):
    if not np.isfinite(perturbation) or perturbation<=0:
        raise ValueError('Stability perturbation must be finite and positive')
    if (config['flight'].get('controller') or {}).get('enabled'):
        raise ValueError('Scheduled feedback requires time-response analysis; disable controller for open-loop eigenmodes')
    trim = trim or solve_trim(config, aero)
    if trim["mode"] not in ("deployed", "aircraft_only"):
        raise ValueError("Linear modes supported for noncontact deployed or aircraft-only trim")
    with closing(CoupledModel(config,aero,trim,phase=trim['mode'],fixed_length=trim.get('length_m'))) as model:
        return _analyze_stability(model,trim,perturbation)


def _analyze_stability(model,trim,perturbation):
    y = np.array(trim["state"], float)
    count = len(y) if model.sensor_enabled else 13
    # Remove quaternion radial degrees by expressing attitude perturbations as rotation vectors.
    from scipy.spatial.transform import Rotation
    blocks = [(0, 13)] if count==13 or model.point_mass else [(0, 13), (13, 26)]
    reduced = count-len(blocks)-(7 if model.point_mass and count>13 else 0)
    def expand(z):
        yp = y.copy(); at = 0
        for start, end in blocks:
            yp[start:start+6] += z[at:at+6]
            mat = rotation(y[start+6:start+10])@Rotation.from_rotvec(z[at+6:at+9]).as_matrix()
            quat_xyzw = Rotation.from_matrix(mat).as_quat()
            yp[start+6:start+10] = np.r_[quat_xyzw[3], quat_xyzw[:3]]
            yp[start+10:start+13] += z[at+9:at+12]; at += 12
        if model.point_mass and count>13:
            yp[13:19] += z[at:at+6]; at += 6
        if count>13:
            yp[26:] += z[at:]
        return yp
    base = model.rhs(0, y)
    accelerations=np.r_[base[3:6],base[10:13]]
    if model.sensor_enabled:
        accelerations=np.r_[accelerations,base[16:19],base[23:26],base[26:].reshape(model.n,6)[:,3:].ravel()]
    if np.max(abs(accelerations))>2e-3 or np.max(abs(y[10:13]))>1e-8 or (model.sensor_enabled and np.max(abs(y[23:26]))>1e-8):
        raise ValueError('Stability analysis requires a steady nonrotating equilibrium')
    if model.mesh_contact_enabled and any(h['gap']<model.c['collision']['skin_m'] for h in model.contact_geometry(0,y)):
        raise ValueError('Stability analysis requires noncontact equilibrium')
    def dynamics(z):
        yp = expand(z); dy = model.rhs(0, yp)-base; parts=[]; at=0
        for start, end in blocks:
            # Trim angular rates are zero. Local attitude-error derivative is delta omega.
            parts.extend([dy[start:start+6], yp[start+10:start+13]-y[start+10:start+13], dy[start+10:start+13]])
            at += 12
        if model.point_mass and count>13:
            parts.append(dy[13:19])
        if count>13:
            parts.append(dy[26:])
        return np.concatenate(parts)
    matrix = np.column_stack([(dynamics(np.eye(reduced)[i]*perturbation)-dynamics(-np.eye(reduced)[i]*perturbation))/(2*perturbation) for i in range(reduced)])
    eig = np.linalg.eigvals(matrix)
    from .stability_quality import describe_spectrum
    return {"matrix": matrix, "eigenvalues": eig, "trim": trim,
            "linearization_step": float(perturbation), **describe_spectrum(eig)}


def simulate(config, aero, trim=None, phase="mission", initial_state=None, duration=None, output=None, start_time=0., initial_capture_time=None, stop_on_capture=False):
    interval=config['simulation']['duration_s'] if duration is None else duration
    if not np.isfinite([start_time,interval]).all() or start_time<0 or interval<=0:
        raise ValueError('Simulation requires a finite nonnegative start and positive duration')
    trim = trim or solve_trim(config, aero, mode="deployed" if phase in ('deployed','recovery') else "stowed" if phase=="mission" else phase)
    with closing(CoupledModel(config, aero, trim, phase=phase, fixed_length=trim.get("length_m") if phase=="deployed" else None)) as model:
        return _simulate(config,aero,trim,model,phase,initial_state,duration,output,start_time,initial_capture_time,stop_on_capture)


def _simulate(config,aero,trim,model,phase,initial_state,duration,output,start_time,initial_capture_time,stop_on_capture):
    capture_enabled=phase=='mission' and config['winch'].get('capture_enabled',True)
    started = time.perf_counter()
    if initial_capture_time is not None:
        if not capture_enabled or not np.isfinite(initial_capture_time) or not 0<=initial_capture_time<=start_time:
            raise ValueError('Initial capture time must precede a mission restart')
        model.captured=True;model.capture_time=float(initial_capture_time)
    y = np.array(initial_state if initial_state is not None else trim["state"], float)
    if y.shape!=(model.size,) or not np.isfinite(y).all() or any(abs(np.linalg.norm(y[i:i+4])-1)>1e-5 for i in (6,19)):
        raise ValueError('Invalid initial state size, finite values or normalized quaternion')
    if initial_state is None and phase=="deployed":
        angles=euler(y[19:23])+np.deg2rad(config['flight'].get('initial_sensor_angles_delta_deg',[0,0,0]))
        y[19:23]=quaternion(*angles)
        y[16:19] += np.asarray(config['flight'].get('initial_sensor_velocity_delta_m_s',[0,0,0]))
    end = float(start_time+(duration if duration is not None else config["simulation"]["duration_s"]))
    sample = config["simulation"]["sample_dt_s"]
    # Integrate separately across schedule corners and bead activation crossings.
    breaks = [float(start_time), end]
    controller = config['flight'].get('controller') or {}
    if controller.get('enabled'):
        breaks += [controller['enable_from_s'], controller['enable_from_s']+controller['engage_ramp_s']]
    for rows in config['flight'].get('controls',{}).values():
        breaks += [row[0] for row in rows]
    gust=config['flight'].get('gust')
    if gust:
        breaks += [gust['start_s'],gust['start_s']+gust['duration_s']]
    if phase in ('mission','recovery'):
        w = config["winch"]
        breaks += [w["release_s"], w["recovery_start_s"]]
        if 'release_push_until_s' in config['bay']:
            breaks.append(config['bay']['release_push_until_s'])
        for key in ("length_schedule", "door_schedule"):
            breaks += [r[0] for r in w[key]]
        for (t0,l0),(t1,l1) in zip(w["length_schedule"][:-1], w["length_schedule"][1:]):
            if l1!=l0:
                for j in range(model.n):
                    edge = (j+.5)*model.h
                    f = (edge-l0)/(l1-l0)
                    if 0<f<1:
                        breaks.append(t0+f*(t1-t0))
    breaks = sorted(set(t for t in breaks if start_time<=t<=end))
    ts, ys, details, events = [float(start_time)], [y.copy()], [], []
    status, message = "completed", ""
    last_phase = None
    def engage_capture(t, state):
        nonlocal last_phase
        model.captured=True
        model.capture_time=float(t)
        events.append({'time_s':float(t),'event':'capture_engaged','method':'compliant latch; velocities continuous'})
        # Record both sides of the force jump. Position/velocity stay continuous;
        # omitting this right-hand limit aliases the maximum latch force.
        _,right=model.rhs(t,state,True)
        ts.append(float(t));ys.append(state.copy());details.append(right)
        progress.accept(t,state);progress.flush()
        last_phase='captured'
    if output:
        if (Path(output)/'accepted_history').exists() or (Path(output)/'states.npz').exists():
            raise FileExistsError('Simulation output already contains a trajectory; use a new directory')
        Path(output).mkdir(parents=True, exist_ok=True)
        (Path(output)/'summary.json').write_text(json.dumps({'status':'running','start_time_s':start_time,'requested_end_s':end}),encoding='utf8')
        (Path(output)/'inputs.json').write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf8')
    from .checkpoint import AcceptedProgress
    progress=AcceptedProgress(model,output,start_time,end)
    progress.accept(start_time,y);progress.flush()
    evaluations = 0
    clearance_checks = 0
    minimum_checked_gap = config.get('collision',{}).get('skin_m')
    last_progress = time.perf_counter()
    def rhs_with_progress(t, state):
        nonlocal evaluations, last_progress
        evaluations += 1
        progress.check()
        now = time.perf_counter()
        if output and now-last_progress>10:
            (Path(output)/'progress.json').write_text(json.dumps({'time_s':float(t),'requested_s':end,'runtime_s':now-started,'rhs_evaluations':evaluations}),encoding='utf8')
            np.savez(Path(output)/'debug_last_trial.npz',time=float(t),state=state,active_nodes=model.active_override)
            last_progress = now
        return model.rhs(t, state)
    for start, stop in zip(breaks[:-1], breaks[1:]):
        # A fixed topology on each open interval avoids switching back and forth
        # at floating-point event times during implicit Newton iterations.
        model.active_override=model.active_count(model.length((start+stop)/2)[0])
        current = start
        while current < stop-1e-10:
            def capture(t,z):
                if not capture_enabled or model.captured or t<config["winch"]["recovery_start_s"]:
                    return 1.
                return model.capture_metric(t,z)
            capture.terminal=True; capture.direction=-1
            def ground(t,z):
                return min(-z[2],-z[15]) if model.sensor_enabled else -z[2]
            ground.terminal=True;ground.direction=-1
            if ground(current,y)<=0:
                status='ground_contact';message='Initial state reaches ground altitude; propagation stopped.'
                events.append({'time_s':float(current),'event':'ground_contact'});break
            if capture_enabled and not model.captured and current>=config["winch"]["recovery_start_s"] and model.capture_metric(current,y)<0:
                if stop_on_capture and ground(current,y)<=0:
                    status='ground_contact';message='Initial capture pose reaches ground altitude.';break
                if stop_on_capture and model.mesh_contact_enabled and model.clearance_metric(current,y)<=0:
                    status='contact_domain_exceeded';message='Initial capture pose violates minimum geometric clearance.';break
                engage_capture(current,y)
                if stop_on_capture:
                    status='capture_event';message='Stopped at actual capture for an explicit command update.';break
            local_stop=min([stop]+[t for t in model.door_transition_times() if current+1e-10<t<stop])
            try:
                local_sensor=rotation(y[6:10]).T@(y[13:16]-y[:3])
                contact_step=config['simulation'].get('contact_max_step_s',.003) if not model.point_mass and phase in ('mission','stowed') and abs(local_sensor[0]-config['bay']['exit_x_m'])<.8 else np.inf
                from .contact_integration import checked_radau
                sol=checked_radau(rhs_with_progress,(current,local_stop),y,model,[capture,ground],
                                      min(config['simulation']['max_step_s'],contact_step),
                                      config['simulation']['rtol'],config['simulation']['atol'],model.jacobian(),on_accepted=progress.accept)
                if model.mesh_contact_enabled:
                    clearance_checks+=sol.clearance_checks
                    minimum_checked_gap=min(minimum_checked_gap,sol.minimum_checked_gap_m)
            except ValueError as exc:
                status="contact_domain_exceeded" if 'CONTACT_DOMAIN' in str(exc) else "aero_domain_exceeded" if "bounds" in str(exc) or "AERO_DOMAIN" in str(exc) else 'control_domain_exceeded' if 'CONTROL_DOMAIN' in str(exc) else "numerical_failure"
                message=str(exc); break
            new_t=np.arange(np.floor(current/sample+1)*sample,sol.t[-1]+1e-9,sample)
            if not len(new_t) or abs(new_t[-1]-sol.t[-1])>1e-8:
                new_t=np.r_[new_t,sol.t[-1]]
            # Fixed output intervals can miss brief impact peaks entirely.
            # Preserve checked contact steps and their midpoints for force and
            # impulse reporting, while keeping the real integrated trajectory.
            contact_samples=getattr(sol,'contact_sample_times',np.empty(0))
            new_t=np.unique(np.r_[new_t,sol.t[-1],contact_samples[contact_samples>current+1e-12]])
            new_t=new_t[(new_t>current+1e-12)&(new_t<=sol.t[-1])]
            for t,z in zip(new_t,sol.sol(new_t).T):
                _,d=model.rhs(float(t),z,True)
                if d["phase"]!=last_phase:
                    events.append({"time_s":float(t),"event":d["phase"]});last_phase=d["phase"]
                ts.append(float(t));ys.append(z.copy());details.append(d)
            y=sol.y[:,-1];current=float(sol.t[-1])
            progress.flush()
            if not sol.success:
                status=sol.failure_status or 'numerical_failure';message=sol.message;break
            if sol.t_events[0].size:
                engage_capture(current,y)
            if sol.t_events[1].size:
                status='ground_contact';message='Aircraft or sensor reached ground altitude; propagation stopped.'
                events.append({'time_s':current,'event':'ground_contact'});break
            if model.mesh_contact_enabled and sol.t_events[2].size:
                status='contact_clearance_limit';message='Minimum geometric separation reached; propagation stopped before CAD penetration.'
                events.append({'time_s':current,'event':'contact_clearance_limit'});break
            if stop_on_capture and sol.t_events[0].size:
                status='capture_event';message='Stopped at actual capture for an explicit command update.';break
            if current>=stop-1e-10:
                break
        if status!="completed":
            break
        if output:
            (Path(output)/"progress.json").write_text(json.dumps({"time_s":float(current),"requested_s":end,"runtime_s":time.perf_counter()-started}),encoding="utf8")
            progress.accept(current,y);progress.flush()
    progress.flush()
    times=np.array(ts); states=np.array(ys)
    # The initial row must use the initial (not eventual captured) state.
    with closing(CoupledModel(config,aero,trim,phase=phase,fixed_length=model.fixed_length)) as initial_model:
        initial_model.captured=initial_capture_time is not None;initial_model.capture_time=initial_capture_time
        _,d0=initial_model.rhs(start_time,states[0],True)
    frame=pd.DataFrame([d0]+details)
    frame.insert(0,"time_s",times)
    for idx,name in enumerate(("roll_deg","pitch_deg","yaw_deg")):
        frame[name]=[np.rad2deg(euler(z[6:10])[idx]) for z in states]
        if not model.point_mass:
            frame["sensor_"+name]=[np.rad2deg(euler(z[19:23])[idx]) for z in states]
    frame["altitude_m"]=-states[:,2]
    frame["sensor_altitude_m"]=-states[:,15]
    frame["contact_impulse_Ns"]=np.r_[0.,np.cumsum(np.diff(times)*(frame.contact_N.values[1:]+frame.contact_N.values[:-1])/2)]
    frame["capture_impulse_Ns"]=np.r_[0.,np.cumsum(np.diff(times)*(frame.capture_N.values[1:]+frame.capture_N.values[:-1])/2)]
    frame["winch_work_J"]=np.r_[0.,np.cumsum(np.diff(times)*(frame.winch_power_W.values[1:]+frame.winch_power_W.values[:-1])/2)]
    violations=[]
    for column,section,key in (("tension_N","cable","limit_N"),("torque_Nm","winch","limit_torque_Nm"),("winch_power_W","winch","limit_power_W"),("contact_N","bay","limit_contact_N"),("penetration_m","bay","limit_penetration_m")):
        limit=config[section].get(key)
        peak=frame[column].abs().max() if column=='winch_power_W' else frame[column].max()
        if limit is not None and peak>limit:
            violations.append({"quantity":column,"limit":limit,"peak":float(peak)})
    summary={"status":status,"message":message,"phase":phase,"trim":{k:v for k,v in trim.items() if k!='state'},"duration_s":float(times[-1]-start_time),"requested_duration_s":end-start_time,"start_time_s":start_time,"end_time_s":float(times[-1]),
             "captured":bool(model.captured),"capture_status":"not_applicable" if not capture_enabled else "captured" if model.captured else "not_captured",
             "capture_time_s":model.capture_time,"final_door_deg":float(frame.door_deg.iloc[-1]),
             "door_capture_interlock":bool(config['winch'].get('door_capture_interlock',False)),
             "door_status":"held_open_capture_required" if frame.door_interlock_active.iloc[-1] and not model.captured else "closed" if abs(frame.door_deg.iloc[-1])<1e-6 else "open_or_moving",
             "violations":violations,"max_tension_N":float(frame.tension_N.max()),"max_contact_N":float(frame.contact_N.max()),
             "contact_impulse_Ns":float(frame.contact_impulse_Ns.iloc[-1]),"max_penetration_m":float(frame.penetration_m.max()),
             "max_capture_N":float(frame.capture_N.max()),"capture_impulse_Ns":float(frame.capture_impulse_Ns.iloc[-1]),
             "max_pitch_change_deg":float(np.max(np.abs(frame.pitch_deg-frame.pitch_deg.iloc[0]))),
             "runtime_s":time.perf_counter()-started,"event_sample_format":2,"numerically_converged":False,"physical_validation":"unvalidated_assumption_case",
             "geometry_validity":"not_audited","mesh_contact_enabled":model.mesh_contact_enabled,
             "clearance_checks":clearance_checks,"minimum_checked_mesh_gap_m":minimum_checked_gap,
             "assumptions":config["provenance"],"aero_metadata":aero.metadata,
             "limitations":["rear wake/exposure prescribed, not resolved CFD","bead release needs mesh convergence",
                            "CAD mesh separation barrier; impact loads uncalibrated; cable/winch winding and cable self-contact not modelled" if model.mesh_contact_enabled else "compliant discrete-point contact; no structural failure prediction",
                            "explicit longitudinal PD/P control; perfect state and instantaneous bounded actuators; no lateral control" if controller.get('enabled') else "no pilot/autopilot stabilisation"]}
    summary.update(controller_enabled=bool(controller.get('enabled')),
                   min_altitude_m=float(frame.altitude_m.min()), max_altitude_m=float(frame.altitude_m.max()),
                   final_altitude_m=float(frame.altitude_m.iloc[-1]),
                   min_airspeed_m_s=float(frame.airspeed_m_s.min()), max_airspeed_m_s=float(frame.airspeed_m_s.max()))
    if controller.get('enabled'):
        summary.update(controller_resolved=model.controller,
            max_altitude_error_m=float(frame.altitude_error_m.abs().max()),
            max_airspeed_error_m_s=float((frame.airspeed_m_s-controller['airspeed_m_s']).abs().max()))
        # Time-weighted duty, not a sample count: event boundaries add samples.
        for key in ('elevator_saturated','thrust_saturated'):
            values=frame[key].to_numpy(dtype=float)
            duty=np.sum(np.diff(times)*(values[1:]+values[:-1])*.5)/(times[-1]-times[0]) if times[-1]>times[0] else 0.
            summary[key+'_time_fraction']=float(duty)
    if model.mesh_contact_enabled and model._mesh_contacts is not None:
        from importlib.metadata import version
        summary['collision_metadata']={'query_engine':'pybullet','version':version('pybullet'),
            'mesh_sha256':model._mesh_contacts.hashes,'parameters':config['collision'],
            'integrator':'Radau with relative surface-travel bound and midpoint/endpoint clearance checks',
            'scope':'all delivered aircraft CAD parts and full sensor; capsules for every exposed cable span; '
                    + ('legacy drum-centre cable feed excluded' if config['winch'].get('line_attaches_to_drum_center',True)
                       else 'drum and separate fairlead included in cable collision checks')}
    if model.point_mass:
        summary.update(payload_model='point_mass',geometry_validity='not_modelled',
                       sensor_aerodynamics='not_modelled',payload_rotational_dofs=0,
                       contact_evaluation='not_modelled; capture is a translational compliant latch only' if capture_enabled else 'not_modelled; no capture')
        summary['limitations'][2]='Point payload has no aerodynamic load, attitude or CAD contact; marker size is visual only'
    result=SimulationResult(times,states,frame,summary,events,copy.deepcopy(config))
    if output:
        result.save(output)
    return result


def _sweep_job(args):
    from threadpoolctl import threadpool_limits
    cfg, aero_path, variant, directory, phase = args
    with threadpool_limits(limits=1):
        Path(directory).mkdir(parents=True,exist_ok=True)
        (Path(directory)/'summary.json').write_text(json.dumps({'status':'running','variant':variant}),encoding='utf8')
        try:
            case=changed(cfg,**variant)
            result=simulate(case,AeroDatabase(aero_path),phase=phase,output=directory)
            return {**variant,**result.summary}
        except Exception as exc:
            failure={**variant,"status":"failed","message":str(exc)}
            (Path(directory)/'summary.json').write_text(json.dumps(failure,indent=2),encoding='utf8')
            return failure


def run_sweep(config,aero,variants,output,workers=None,phase="deployed"):
    workers=workers if workers is not None else min(8,max(1,(os.cpu_count() or 2)-1))
    if isinstance(workers,bool) or not isinstance(workers,int) or workers<1:
        raise ValueError('Sweep workers must be a positive integer')
    config=copy.deepcopy(config)
    variants=[dict(v) for v in variants]
    requested_variants=copy.deepcopy(variants)
    if workers>1:
        # Parallelize cases, not nested pools of cases x Jacobian columns.
        config['simulation']['jacobian_workers']=1
        for v in variants:
            if 'simulation.jacobian_workers' in v:v['simulation.jacobian_workers']=1
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    if any((output/f'case_{i:03d}').exists() for i in range(len(variants))):
        raise FileExistsError('Sweep case directories already exist; use a new output directory')
    (output/'sweep_execution.json').write_text(json.dumps({'case_workers':workers,'jacobian_policy':'serial within parallel cases' if workers>1 else 'configured',
        'requested_variants':requested_variants},indent=2),encoding='utf8')
    jobs=[(config,aero.path,v,str(output/f"case_{i:03d}"),phase) for i,v in enumerate(variants)]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows=list(pool.map(_sweep_job,jobs))
    (output/"sweep.json").write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding="utf8")
    table=pd.json_normalize(rows);table.to_csv(output/"sweep.csv",index=False)
    return table
