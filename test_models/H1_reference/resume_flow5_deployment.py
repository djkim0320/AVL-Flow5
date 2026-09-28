"""Continue the saved real-flow5 deployment without changing physical inputs."""
import os
for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
from pathlib import Path
import json
import numpy as np
from dbf_stability import AeroDatabase, resume_simulation
from dbf_stability.plots import history_figure

HERE = Path(__file__).resolve().parent
RUN = HERE/'runs/flow5_connection_01'

if __name__ == '__main__':
    original = RUN/'flow5_deployment/mission'
    output = RUN/'flow5_deployment/continuation'
    case = json.loads((original/'inputs.json').read_text(encoding='utf8'))
    case['simulation']['maximum_runtime_s'] = 1200
    aero = AeroDatabase(RUN/'aerodynamics/aero_database.npz')
    with np.load(original/'accepted_checkpoint.npz') as checkpoint:
        start = float(checkpoint['time'])
        state = checkpoint['state'].copy()
    result = resume_simulation(case, aero, original/'accepted_checkpoint.npz',
                               duration=3.-start, output=output)
    history_figure(result).write_html(output/'history.html', include_plotlyjs=True)
    check = dict(status=result.summary['status'],
                 state_continuous=bool(np.array_equal(state, result.states[0])),
                 start_time_s=start, end_time_s=float(result.time[-1]))
    (output/'restart_check.json').write_text(json.dumps(check, indent=2), encoding='utf8')
    results = json.loads((RUN/'connection_results.json').read_text(encoding='utf8'))
    results['flow5_deployment_continuation'] = result.summary
    (RUN/'connection_results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(check), flush=True)
