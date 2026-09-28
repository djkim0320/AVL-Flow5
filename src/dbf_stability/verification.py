"""Executed numerical-convergence reports; failure never becomes an accuracy claim."""
from pathlib import Path
import json
from contextlib import closing
import numpy as np
from . import changed, solve_trim, simulate, run_sweep


def kinetic_work_audit(result, aero, output):
    """Audit integrated force work against kinetic energy, including stored cells.

    This checks quadrature/integration and mass bookkeeping. Internal force work
    includes elastic storage and dissipation; it is NOT a closed actuator/thermal
    energy budget and must not be presented as such.
    """
    import pandas as pd
    from .model import CoupledModel
    c=result.config;phase=result.summary['phase']
    trim=result.summary.get('trim')
    if trim is None:
        raise ValueError('Trajectory lacks actual trim commands. Restore them from its source checkpoint before auditing; do not reconstruct a different trim.')
    fixed_length=float(result.table.length_m.iloc[0]) if phase=='deployed' else None
    with closing(CoupledModel(c,aero,trim,phase=phase,fixed_length=fixed_length)) as model:
        return _kinetic_work_audit(result,model,output)


def _kinetic_work_audit(result,model,output):
    import pandas as pd
    capture_time=result.summary.get('capture_time_s')
    if result.table.captured.any() and capture_time is None:
        raise ValueError('Captured trajectory lacks capture_time_s; cannot reproduce door interlock forces')
    records=[]
    for i,(t,y) in enumerate(zip(result.time,result.states)):
        model.captured=bool(result.table.captured.iloc[i])
        model.capture_time=capture_time if model.captured else None
        model.active_override=int(result.table.active_nodes.iloc[i])
        dy=model.rhs(t,y);nodes=y[26:].reshape(model.n,6);dn=dy[26:].reshape(model.n,6)
        k=model.active_override
        kinetic=.5*model.ma*np.dot(y[3:6],y[3:6])+.5*y[10:13]@model.Ia@y[10:13]
        power=model.ma*np.dot(y[3:6],dy[3:6])+y[10:13]@model.Ia@dy[10:13]
        stored=.5*model.mn*np.sum(nodes[k:,3:]**2);deployed=.5*model.mn*np.sum(nodes[:k,3:]**2)
        if model.sensor_enabled:
            kinetic+=.5*model.ms*np.dot(y[16:19],y[16:19])+stored+deployed
            power+=model.ms*np.dot(y[16:19],dy[16:19])+model.mn*np.sum(nodes[:,3:]*dn[:,3:])
            if not model.point_mass:
                kinetic+=.5*y[23:26]@model.Is@y[23:26]
                power+=y[23:26]@model.Is@dy[23:26]
        records.append({'time_s':t,'kinetic_J':kinetic,'force_work_rate_W':power,'stored_cable_kinetic_J':stored,'deployed_cable_kinetic_J':deployed})
    table=pd.DataFrame(records)
    table['integrated_force_work_J']=np.r_[0,np.cumsum(np.diff(table.time_s)*(table.force_work_rate_W.values[1:]+table.force_work_rate_W.values[:-1])/2)]
    table['kinetic_work_residual_J']=table.kinetic_J-table.kinetic_J.iloc[0]-table.integrated_force_work_J
    scale=max(float(np.max(abs(table.kinetic_J-table.kinetic_J.iloc[0]))),1e-6)
    report={'scope':'kinetic-energy / resultant-force-work quadrature consistency, including stored and deployed cable mass',
            'max_residual_J':float(abs(table.kinetic_work_residual_J).max()),'relative_to_max_kinetic_change':float(abs(table.kinetic_work_residual_J).max()/scale),
            'full_actuator_elastic_thermal_energy_closure':False,'winch_work_J':float(result.table.winch_work_J.iloc[-1]),
            'note':'Cable-cell transfer does not create/delete mass or velocity. Elastic constraint energy at topology switches still requires a conservative formulation.'}
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    table.to_csv(output/'kinetic_work_audit.csv',index=False)
    (output/'kinetic_work_audit.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    return report


def convergence_study(config,aero,output,workers=3,duration=1.0):
    base=changed(config,**{'simulation.duration_s':duration,'flight.initial_sensor_angles_delta_deg':[0,.1,0]})
    variants=[{'cable.segments':n,'simulation.max_step_s':dt} for n,dt in [(10,.02),(20,.02),(40,.02),(40,.01)]]
    table=run_sweep(base,aero,variants,Path(output)/'runs',workers=workers,phase='deployed')
    metrics=['max_tension_N','max_pitch_change_deg','contact_impulse_Ns','max_contact_N']
    checks=[]
    for metric in metrics:
        if metric not in table or table['status'].ne('completed').any():
            checks.append({'metric':metric,'status':'incomplete'});continue
        values=table[metric].astype(float).to_numpy()
        if np.max(abs(values))<1e-7:
            checks.append({'metric':metric,'status':'not_excited','values':values.tolist()});continue
        rel=abs(np.diff(values))/np.maximum(abs(values[1:]),1e-7)
        checks.append({'metric':metric,'status':'pass' if np.max(rel)<=.05 else 'fail','relative_changes':rel.tolist(),'values':values.tolist()})
    report={'case':'fixed-length tow, sensor pitch perturbed by 0.1 deg; contact is not excited','target_relative_change':.05,'checks':checks,
            'physical_validation':False,'deployment_contact_convergence':'not_established_by_this_equilibrium_check'}
    Path(output).mkdir(parents=True,exist_ok=True)
    (Path(output)/'convergence.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    return report
