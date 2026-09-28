from pathlib import Path
from dbf_stability import *


def main():
    root = Path(__file__).resolve().parents[1]
    c = load_case(root / 'examples/reference.yaml')
    a = AeroDatabase(root / 'outputs/aero_database.npz')
    variants = [
        {},
        {'flight.gust': {'start_s': 0.2, 'duration_s': 0.5, 'velocity_ned_m_s': [0, 1, 0]}},
        {'sensor.mass_kg': 0.08},
        {'sensor.mass_kg': 0.12},
        {'aircraft.tow_point_m': [-0.16, 0, 0.03]},
        {
            'cable.length_m': 3.0,
            'winch.length_schedule': [[0, 0.03], [0.6, 0.03], [4, 3], [5, 3], [10, 0.03], [11, 0.03]],
        },
        {'winch.limit_torque_Nm': 0.001},
        {'flight.initial_sensor_angles_delta_deg': [0, 0, 2.0]},
    ]
    base = changed(c, **{'simulation.duration_s': 1.0})
    table = run_sweep(base, a, variants, root / 'outputs/scenarios', workers=4)
    print(table[['status', 'max_tension_N', 'max_pitch_change_deg']].to_string(index=False), flush=True)


if __name__ == '__main__':
    main()
