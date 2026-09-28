from pathlib import Path
import json
from dbf_stability import *
from dbf_stability.plots import export_plots


def main():
    root=Path(__file__).resolve().parents[1]
    c=load_case(root/'examples/reference.yaml');a=AeroDatabase(root/'outputs/aero_database.npz')
    variants=[
      {},
      {'flight.gust':{'start_s':.2,'duration_s':.5,'velocity_ned_m_s':[0,1,0]}},
      {'sensor.mass_kg':.08},
      {'sensor.mass_kg':.12},
      {'aircraft.tow_point_m':[-.16,0,.03]},
      {'cable.length_m':3.,'winch.length_schedule':[[0,.03],[.6,.03],[4,3],[5,3],[10,.03],[11,.03]]},
      {'winch.limit_torque_Nm':.001},
      {'flight.initial_sensor_angles_delta_deg':[0,0,2.]},
    ]
    base=changed(c,**{'simulation.duration_s':1.})
    table=run_sweep(base,a,variants,root/'outputs/scenarios',workers=4)
    print(table[['status','max_tension_N','max_pitch_change_deg']].to_string(index=False),flush=True)


if __name__=='__main__':main()
