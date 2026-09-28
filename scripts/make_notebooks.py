from pathlib import Path
import nbformat as nb

ROOT = Path(__file__).resolve().parents[1]

SETUP = '''from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dbf_stability import *
from dbf_stability.analysis import load_result
ROOT = Path.cwd() if (Path.cwd() / "examples").exists() else Path.cwd().parent
c = load_case(ROOT / "examples/reference.yaml")
aero = AeroDatabase(ROOT / "outputs/aero_database.npz")
plt.rcParams.update({"figure.figsize": (10, 4), "axes.grid": True, "grid.alpha": 0.2})
pd.set_option("display.max_columns", 12)
print("Actual solver:", aero.metadata["solver"])
print("Raw solver files:", aero.metadata["raw_directory"])
'''


def write(name, title, description, parts):
    cells = [
        nb.v4.new_markdown_cell('# ' + title + '\n\n' + description),
        nb.v4.new_markdown_cell(
            '## 실행 준비\n\n프로젝트 전용 환경에서 실행합니다. 먼저 README의 `build-aero` 명령으로 실제 AVL 데이터를 생성해야 합니다. 예시 입력은 팀 기체의 설계값이 아닙니다.'
        ),
        nb.v4.new_code_cell(SETUP),
    ]
    for kind, body in parts:
        cells.append(nb.v4.new_markdown_cell(body) if kind == 'md' else nb.v4.new_code_cell(body))
    n = nb.v4.new_notebook(
        cells=cells,
        metadata={
            'kernelspec': {'display_name': 'DBF Python', 'language': 'python', 'name': 'dbf-stability'},
            'language_info': {'name': 'python', 'version': '3.11'},
        },
    )
    nb.validate(n)
    nb.write(n, ROOT / 'notebooks' / name)


write(
    '01_aircraft_validation.ipynb',
    '기체 단독 공력·트림 검증',
    '실제 AVL 결과와 기체 운동 모델을 연결합니다. 수납 중량을 추가했을 때 필요한 받음각·조종면·추력의 변화를 비교합니다.',
    [
        ('md', '## 입력과 출처'),
        ('code', "pd.DataFrame(c['provenance']).T"),
        ('md', '## 트림 비교'),
        (
            'code',
            "trims = {mode: solve_trim(c, aero, mode=mode) for mode in ['aircraft_only', 'stowed', 'deployed']}\ncomparison = pd.DataFrame([{k:v for k,v in tr.items() if k != 'state'} for tr in trims.values()]).set_index('mode')\ncomparison[['alpha_deg','elevator_deg','thrust_N','residual_norm']]",
        ),
        (
            'code',
            "fig, axes = plt.subplots(1, 3, figsize=(12, 4))\nfor ax, key, label in zip(axes, ['alpha_deg','elevator_deg','thrust_N'], ['Angle of attack (deg)','Elevator (deg)','Thrust (N)']):\n    comparison[key].plot.bar(ax=ax, color='#2563a6', rot=25)\n    ax.set_ylabel(label); ax.set_xlabel('')\nfig.suptitle('Actual AVL-backed trim; assumed reference aircraft')\nfig.tight_layout(); plt.show()",
        ),
        ('md', '## 평형 유지 확인'),
        (
            'code',
            "result = simulate(c, aero, trims['aircraft_only'], phase='aircraft_only', duration=0.5)\nassert result.summary['status'] == 'completed'\nassert result.summary['max_pitch_change_deg'] < 1e-5\nprint('Pitch drift (deg):', result.summary['max_pitch_change_deg'])",
        ),
        (
            'md',
            '## 해석 범위\n\n이 비교는 지정한 공력·질량·관성 가정에서의 계산입니다. 공력 보간 범위를 벗어난 조건은 거부합니다. 후방 개구부의 박리·프로펠러 후류와 실제 제작 오차는 검증하지 않았습니다. [모델 설명](../docs/MODEL.md)과 [검증 결과](../outputs/validation/convergence/convergence.json)를 함께 확인하세요.',
        ),
    ],
)

write(
    '02_deployed_stability.ipynb',
    '센서 전개 후 결합 안정성',
    '고정 길이 견인 상태의 기체·센서·케이블을 함께 선형화하고 작은 교란 응답과 비교합니다. 전개·접촉 중의 안정성을 이 고유값으로 판정하지 않습니다.',
    [
        ('md', '## 평형과 고유값'),
        (
            'code',
            "trim = solve_trim(c, aero)\nstability = analyze_stability(c, aero, trim)\nstability['modes'].head(16)",
        ),
        (
            'code',
            "modes = stability['modes']\nfig, axes = plt.subplots(1,2,figsize=(12,4))\nfor ax in axes:\n    ax.scatter(modes.real_1_s, modes.imag_rad_s, marker='x', color='#2563a6')\n    ax.axvline(0,color='#444444',linewidth=1)\n    ax.set(xlabel='Real part (1/s)',ylabel='Imaginary part (rad/s)')\naxes[0].set_title('All coupled eigenvalues')\naxes[1].set(xlim=(-5,1),ylim=(-8,8),title='Slow modes: positive real part means growth')\nfig.tight_layout();plt.show()\nprint('Local growing mode present:', stability['unstable'])",
        ),
        ('md', '## 센서 피치에 작은 교란 적용'),
        (
            'code',
            "perturbed = changed(c, **{'flight.initial_sensor_angles_delta_deg':[0,0.1,0]})\nresponse = simulate(perturbed, aero, trim, phase='deployed', duration=1.0)\nfig, ax = plt.subplots()\nfor key, color in [('pitch_deg','#2563a6'),('sensor_pitch_deg','#c57a22')]:\n    ax.plot(response.time, response.table[key]-response.table[key].iloc[0], label=key, color=color)\nax.set(xlabel='Time (s)',ylabel='Angle change (deg)',title='Response to 0.1 degree sensor pitch perturbation')\nax.legend(); plt.show()\nresponse.summary['status']",
        ),
        (
            'md',
            '## 판독\n\n실수부가 양수인 고유값은 해당 평형에서 교란이 커지는 모드를 뜻합니다. 0 부근의 운동학적 모드와 구분해 읽어야 합니다. 짧은 시간 그래프에서 변화가 작아도 느린 발산 모드가 없다는 뜻은 아닙니다.',
        ),
    ],
)

write(
    '03_deploy_recover_capture.ipynb',
    '후방 전개·회수·포획',
    '문 개방부터 줄 전개, 견인, 회수, 출구 접촉과 포획까지 저장된 실제 시뮬레이션을 확인합니다. 시간 절약을 위해 기본값은 저장 결과를 읽으며 재계산 옵션을 제공합니다.',
    [
        (
            'md',
            '## 입력과 재계산 선택\n\n`RUN_FULL = True`로 바꾸면 전 과정을 다시 계산합니다. 기본값은 입력·출력·원시 상태가 함께 보존된 결과를 읽습니다. 조종 입력은 공개한 시간 이력이며 자동조종이 아닙니다.',
        ),
        (
            'code',
            "RUN_FULL = False\nmission_config = changed(load_case(ROOT / 'examples/mission_scheduled.yaml'), **{'cable.segments':40})\nif RUN_FULL:\n    mission = simulate(mission_config, aero, phase='mission', output=ROOT/'outputs/mission_40')\nelse:\n    mission = load_result(ROOT/'outputs/mission_40')\nprint('Case:', mission.config['name'])\nprint('Applied control histories:', mission.config['flight']['controls'])\npd.Series({k:mission.summary[k] for k in ['status','duration_s','capture_status','max_tension_N','max_contact_N','max_capture_N','max_pitch_change_deg','physical_validation']})",
        ),
        (
            'md',
            '## 분할 수에 따른 포획 결과\n\n40구간에서는 포획했지만 10·20구간에서는 포획하지 못했습니다. 아래 재생은 40구간 결과이며, 이 차이가 남아 있으므로 수렴한 예측으로 취급하지 않습니다.',
        ),
        (
            'code',
            "mesh = pd.read_csv(ROOT/'outputs/mission_convergence/sweep.csv')\nmesh[['cable.segments','capture_status','max_tension_N','contact_impulse_Ns','max_capture_N']]",
        ),
        ('md', '## 전 과정 이력'),
        (
            'code',
            "fig, axes = plt.subplots(3,1,figsize=(11,9),sharex=True)\nfor key in ['pitch_deg','sensor_pitch_deg']:\n    axes[0].plot(mission.time,mission.table[key],label=key)\nfor key in ['tension_N','contact_N','capture_N']:\n    axes[1].plot(mission.time,mission.table[key],label=key)\nfor key in ['sensor_local_x_m','sensor_local_z_m']:\n    axes[2].plot(mission.time,mission.table[key],label=key)\nfor ax,label in zip(axes,['Angle (deg)','Force (N)','Position (m)']):\n    ax.set_ylabel(label); ax.legend()\naxes[2].set_xlabel('Time (s)');fig.tight_layout();plt.show()",
        ),
        ('md', '## 사건과 한계 초과'),
        (
            'code',
            "display(pd.DataFrame(mission.events))\nprint('Limit violations:', mission.summary['violations'])\nprint('Convergence established:', mission.summary['numerically_converged'])",
        ),
        ('md', '## 3차원 재생'),
        (
            'code',
            "from dbf_stability.plots import export_plots\nexport_plots(mission, ROOT/'outputs/mission_40')\nprint('Open:', ROOT/'outputs/mission_40/replay.html')",
        ),
        (
            'md',
            '## 판독\n\n포획 구속 작동 여부와 임무의 적합성은 별개입니다. 자세 변화·고도·장력·접촉 침투량·한계 초과를 함께 확인해야 합니다. 접촉 물성이 가정값이므로 최대 충격력을 실물 강도 판정에 바로 사용하지 마세요. 저장 결과의 `inputs.json`이 해당 실행에 사용된 입력입니다.',
        ),
    ],
)

write(
    '04_sensitivity_and_convergence.ipynb',
    '설계 민감도와 수치 수렴',
    '병렬 실행으로 센서 중량·견인점·줄 길이의 영향을 비교하고 공간 분할과 시간 간격의 민감도를 확인합니다. 시험 검증과 수치 수렴을 구분합니다.',
    [
        ('md', '## 설계 조합과 병렬 계산'),
        (
            'code',
            "variants = [\n    {'sensor.mass_kg':0.08}, {'sensor.mass_kg':0.10}, {'sensor.mass_kg':0.12},\n    {'aircraft.tow_point_m':[-0.16,0,0.03]},\n    {'cable.length_m':3.0,'winch.length_schedule':[[0,.03],[.6,.03],[4,3],[5,3],[10,.03],[11,.03]]}\n]\nbase = changed(c, **{'simulation.duration_s':0.3, 'flight.initial_sensor_angles_delta_deg':[0,.1,0]})\nsweep = run_sweep(base,aero,variants,ROOT/'outputs/notebook_sweep',workers=3)\nsweep[['status','max_tension_N','max_pitch_change_deg']]",
        ),
        (
            'code',
            "fig, ax = plt.subplots()\nlabels=['Sensor 80 g','Sensor 100 g','Sensor 120 g','Tow x=-0.16 m','Cable 3.0 m']\nax.bar(labels,sweep['max_tension_N'],color='#2563a6')\nax.set(ylabel='Peak tension (N)',title='0.1 degree perturbation, 0.3 s response — illustrative cases')\nax.tick_params(axis='x',labelrotation=20);fig.tight_layout();plt.show()",
        ),
        ('md', '## 수치 수렴 기록'),
        (
            'code',
            "report = json.loads((ROOT/'outputs/validation/convergence/convergence.json').read_text())\nprint(report['case'])\npd.DataFrame(report['checks'])",
        ),
        ('md', '## 전 과정 수렴 검사'),
        (
            'code',
            "full = json.loads((ROOT/'outputs/mission_convergence/convergence.json').read_text())\nprint('All checks passed:', full['all_numerical_checks_pass'])\nprint('Same capture outcome:', full['same_capture_outcome'])\npd.DataFrame(full['checks'])",
        ),
        ('md', '## 회수 구간 시간 간격 비교'),
        (
            'code',
            "recovery_time = json.loads((ROOT/'outputs/recovery_time_convergence/convergence.json').read_text())\nprint(recovery_time['scope'])\npd.DataFrame(recovery_time['checks'])",
        ),
        (
            'md',
            '## 판독\n\n고정 길이 견인에서 통과한 검사는 전개·접촉·포획의 수렴성을 대신하지 않습니다. 접촉이 발생하지 않은 값은 `not_excited`로 표시하며 통과로 세지 않습니다. 전체 임무의 수렴성은 `scripts/run_mission_matrix.py`가 만드는 별도 기록에서 확인합니다.',
        ),
    ],
)
print('Created four notebooks')
