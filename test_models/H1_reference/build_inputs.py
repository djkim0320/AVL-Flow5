"""Reproducible H1-A temporary test inputs. No edits to package source or H1 originals."""
from pathlib import Path
import csv, json, hashlib, shutil, math
import numpy as np
import yaml
from extract_h1 import HERE, SOURCE
from dbf_stability import load_case
from dbf_stability.avl import write_mass_file

ROOT=HERE.parents[1]

def dump(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    audit=json.loads((HERE/'source_audit/geometry_measurements.json').read_text())
    geom=HERE/'geometry';geom.mkdir(exist_ok=True)
    # Lumped box masses are assumptions, not mesh-volume-derived aircraft masses.
    masses=[
        ('wing',.50,[.50,0,.113],[.305556,1.8,.037]),
        ('fuselage',.40,[.65,0,-.005],[1.5,.18,.20]),
        ('battery',.60,[.30,0,-.001],[.20,.10,.07]),
        ('propulsion',.25,[.08,0,0],[.12,.08,.08]),
        ('tail_group',.14,[1.28,0,.07],[.27,.70,.30]),
        ('landing_gear',.12,[.50,0,-.18],[.30,.40,.15]),
        ('electronics',.10,[.61,0,-.03],[.10,.08,.03]),
        ('winch_motor_drum',.10,[.90,0,-.02],[.055,.045,.045]),
        ('bay_door_guides_capture',.07,[1.025,0,-.02],[.23,.06,.06]),
    ]
    mass=sum(r[1] for r in masses)
    cg=sum(m*np.array(p) for _,m,p,d in masses)/mass
    tensor=np.zeros((3,3));rows=[]
    for name,m,p,d in masses:
        d=np.array(d);offset=np.array(p)-cg
        local=np.diag(m/12*(sum(d*d)-d*d))
        tensor+=local+m*((offset@offset)*np.eye(3)-np.outer(offset,offset))
        rows.append(dict(part=name,mass_kg=m,x_aru_m=p[0],y_aru_m=p[1],z_aru_m=p[2],
                         length_m=d[0],width_m=d[1],height_m=d[2],status='assumption',source='temporary lumped box mass budget'))
    T=np.diag([-1.,1.,-1.]);ifr=T@tensor@T
    def frd(p):return (T@(np.array(p)-cg)).tolist()
    cfg=yaml.safe_load((ROOT/'examples/reference.yaml').read_text(encoding='utf-8'))
    cfg['name']='H1_A_mesh_contact_rear_sensor_assumption_case'
    cfg['collision']=dict(enabled=True,mesh_directory='test_models/H1_reference/meshes',
        skin_m=.0008,minimum_gap_m=.00002,stiffness_N_m=20000.,damping_Ns_m=8.,
        maximum_surface_travel_m=.0002,maximum_free_surface_travel_m=.002,
        clearance_query_m=.01,cable_contact_regularization_m=.00016,
        model='near_rigid_barrier_unvalidated_impact_loads')
    cfg['aircraft'].update(mass_kg=mass,inertia_kgm2=ifr.tolist(),cg_m=(T@cg).tolist(),
        tow_point_m=frd([.90,0,-.02]),thrust_point_m=frd([.08,0,0]),profile_cd=.035,max_thrust_N=30.)
    # Sensor body and four fins: explicit mass budget; sensor local axes FRD.
    sensor_parts=[('sensor_body',.036,[0,0,0],[.12,.028,.028])]
    for k,(y,z) in enumerate([(.0185,0),(-.0185,0),(0,.0185),(0,-.0185)]):
        sensor_parts.append((f'sensor_fin_{k}',.001,[-.038,y,z],[.03,.009,.002] if y else [.03,.002,.009]))
    sm=sum(x[1] for x in sensor_parts); scg=sum(m*np.array(p) for n,m,p,d in sensor_parts)/sm
    si=np.zeros((3,3))
    for name,m,p,d in sensor_parts:
        d=np.array(d);o=np.array(p)-scg
        # Cylinder body axial/transverse inertia; fins rectangular plates.
        il=np.diag([m*.014**2/2,m*(3*.014**2+.12**2)/12,m*(3*.014**2+.12**2)/12]) if name=='sensor_body' else np.diag(m/12*(sum(d*d)-d*d))
        si+=il+m*((o@o)*np.eye(3)-np.outer(o,o))
    # Translate geometry by -scg so stored centre is the actual assumed sensor CG.
    cps=[]
    for x in [-.06,0,.06]:
        for angle in np.linspace(0,2*np.pi,12,endpoint=False):
            cps.append((np.array([x,.014*np.cos(angle),.014*np.sin(angle)])-scg).tolist())
    for x in [-.053,-.023]:
        for p in [[x,.023,0],[x,-.023,0],[x,0,.023],[x,0,-.023]]:cps.append((np.array(p)-scg).tolist())
    tow=(np.array([.06,0,0])-scg).tolist()
    cfg['sensor'].update(mass_kg=sm,inertia_kgm2=si.tolist(),length_m=.12,span_m=.046,
        area_m2=.0015,tow_point_m=tow,cd=.6,cy_beta=2.,cz_alpha=2.,cm_alpha=-.35,cn_beta=.35,
        rate_damping=[1.,3.,3.],contact_points_m=cps)
    stowed_aru=np.array([.99,0,-.02])
    # Body geometric centre x is 0.9862 m; nose x=0.9262 m.
    natural_stowed=float(np.linalg.norm(np.array(frd(stowed_aru))+np.array(tow)-np.array(cfg['aircraft']['tow_point_m'])))
    cfg['cable'].update(length_m=1.5,segments=10,density_kg_m=.0015,diameter_m=.001,
        EA_N=60.,damping_Ns_m=.03,limit_N=8.)
    cfg['bay'].update(stowed_center_m=frd(stowed_aru),exit_x_m=frd([1.14,0,0])[0],front_x_m=frd([.91,0,0])[0],
        half_width_m=.028,floor_z_m=frd([0,0,-.048])[2],ceiling_z_m=frd([0,0,.008])[2],
        ramp_slope=0.,lip_depth_m=0.,door_length_m=.056,door_thickness_m=.002,wall_thickness_m=.002,
        contact_radius_m=.0015,release_push_N=.12,contact_k_N_m=800.,contact_c_Ns_m=2.,friction=.10,
        latch_k_N_m=500.,latch_c_Ns_m=5.,latch_kr_Nm_rad=.10,latch_cr_Nms_rad=.01,
        capture_radius_m=.006,capture_speed_m_s=.15,capture_angle_deg=8.,limit_contact_N=None,limit_penetration_m=.002)
    cfg['winch'].update(stowed_length_m=natural_stowed,radius_m=.015,limit_torque_Nm=.15,limit_power_W=5.,
        release_s=.6,recovery_start_s=3.,
        door_schedule='../test_models/H1_reference/schedules/door.csv',
        length_schedule='../test_models/H1_reference/schedules/length.csv')
    cfg['flight'].update(speed_m_s=20.,altitude_m=40.,local_flow_factor=1.,controls={})
    cfg['simulation'].update(duration_s=7.,sample_dt_s=.01,max_step_s=.01)
    sec=audit['variants']['A']['wing_section'];tail=audit['variants']['A']['tail_section']
    # Origin stays at original H1 nose. Moment reference explicitly uses aircraft CG.
    cfg['aero'].update(geometry='test_models/H1_reference/geometry/h1_a.avl',elevator_index=2,
        lateral_control_indices=[1,3],moment_reference_frd_m=(T@cg).tolist(),
        alpha_deg=[-6.,-3.,0.,3.,6.,9.,12.],beta_deg=[-8.,0.,8.],elevator_deg=[-20.,-10.,0.,10.,20.])
    sources={
      'aircraft':('assumption','H1 A copied GLB lifting surfaces; lumped box masses, CG, inertia, thrust and profile drag assumed.'),
      'sensor':('assumption','New 120 mm cylinder, 46 mm fin span, 40 g lumped mass. Aerodynamic/contact properties unmeasured.'),
      'cable':('assumption','1.5 m, 1.5 g/m, 1 mm line; EA and damping illustrative; no material tests.'),
      'bay':('assumption','New 56 x 56 mm rear guide, outlet at original x=1.14 m. Requires cutout through original aft lower shell.'),
      'winch':('assumption','Repositioned ideal actuator; prescribed schedules, no real motor or reel inertia model.'),
      'flight':('assumption','20 m/s, 40 m, still air, density 1.225 kg/m3. Local flow factor 1.0 is unvalidated.'),
      'aero':('calculated','Fresh actual AVL 3.52 on H1-A extracted lifting surfaces, no fuselage body; original 2-degree wing incidence; https://web.mit.edu/drela/Public/web/avl/')}
    cfg['provenance']={k:dict(kind=v[0],source=v[1]) for k,v in sources.items()}
    cfg['provenance']['collision']=dict(kind='assumption',source='Delivered CAD mesh contact with an 0.8 mm numerical skin and near-rigid barrier; stiffness/damping are numerical regularisation, not measured impact properties.')
    for name,rows2 in [('door',[[0,0],[.4,100],[6.5,100],[7,100]]),
                       ('length',[[0,natural_stowed],[.6,natural_stowed],[2.5,1.5],[3.,1.5],[6.5,natural_stowed],[7,natural_stowed]])]:
        p=HERE/'schedules'/f'{name}.csv';p.parent.mkdir(exist_ok=True)
        with p.open('w',newline='',encoding='utf8') as f:
            w=csv.writer(f);w.writerow(['time_s','value']);w.writerows(rows2)
    target=ROOT/'examples/h1_reference.yaml'
    target.write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True),encoding='utf8')
    case=load_case(target)
    for typ in ['wing','tail']:
        shutil.copy2(HERE/f'source_audit/A_{typ}_section.dat',geom/f'h1_{typ}.dat')
    # Symmetric NACA0012 for fin, taken from the actual H1 horizontal tail section.
    shutil.copy2(geom/'h1_tail.dat',geom/'h1_fin.dat')
    def section(x,y,z,chord,inc,foil,control=None,spanels=6):
        s=f'SECTION\n{x:.9f} {y:.9f} {z:.9f} {chord:.9f} {inc:.6f} {spanels} 1\nAFILE\n{foil}\n'
        if control:s+='CONTROL\n'+control+'\n'
        return s
    lines=f'H1 A extracted lifting surfaces - temporary sensor trial\n0.0\n0 0 0\n0.55 {sec["chord_m"]:.9f} 1.8\n{cg[0]:.9f} {cg[1]:.9f} {cg[2]:.9f}\n0.0\n'
    wx,_,wz=sec['leading_edge_m'];wc=sec['chord_m']
    lines+='SURFACE\nH1_Wing\n12 1\nYDUPLICATE\n0\n'
    for y,control,n in [(0,None,16),(.5316,'aileron 1 .75 0 1 0 -1',10),(.8794,'aileron 1 .75 0 1 0 -1',2),(.9,None,2)]:
        lines+=section(wx,y,wz,wc,2,'h1_wing.dat',control,n)
    lines+='SURFACE\nH1_Horizontal_tail\n10 1\nYDUPLICATE\n0\n'
    tx,_,tz=tail['leading_edge_m'];tc=tail['chord_m']
    for y,control,n in [(0,None,2),(.0365,'elevator 1 .75 0 1 0 1',12),(.3145,'elevator 1 .75 0 1 0 1',3),(.35,None,2)]:
        lines+=section(tx,y,tz,tc,0,'h1_tail.dat',control,n)
    lines+='SURFACE\nH1_Vertical_fin\n10 1\n'
    # Root 0.27 m, tip 0.13 m reconstructed by combining fin/rudder extrema;
    # 1 mm root gap and hinge clearance are closed for the vortex sheet.
    lines+=section(1.1525,0,.03,.27,0,'h1_fin.dat','rudder 1 .75 0 0 1 1',16)
    lines+=section(1.1875,0,.33,.13,0,'h1_fin.dat','rudder 1 .75 0 0 1 1',2)
    (geom/'h1_a.avl').write_text(lines,encoding='ascii')
    write_mass_file(case,geom/'h1_a.mass');write_mass_file(case,geom/'h1_stowed.mass',stowed=True)
    with (HERE/'mass_budget.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    params={'variant':'A','source':str(SOURCE),'aru_to_frd':T.tolist(),'aircraft_cg_aru_m':cg.tolist(),
      'aircraft_mass_kg':mass,'aircraft_inertia_frd_kgm2':ifr.tolist(),'sensor_mass_kg':sm,
      'sensor_geometric_cg_offset_frd_m':scg.tolist(),'sensor_inertia_frd_kgm2':si.tolist(),
      'cable_mass_kg':.0015*1.5,'total_mass_kg':mass+sm+.0015*1.5,
      'sensor_stowed_cg_aru_m':stowed_aru.tolist(),'wing_section':sec,'tail_section':tail,
      'bay_front_aru_m':.91,'bay_exit_aru_m':1.14,'bay_width_m':.056,'bay_floor_aru_z_m':-.048,
      'bay_ceiling_aru_z_m':.008,'tunnel_box_aru_m':[[.885,-.031,-.062],[1.60,.031,.022]],
      'door_angle_deg':100.,'source_body_sections':json.loads((SOURCE/'body_sections.json').read_text()),
      'sensor_parts':sensor_parts,'assumptions':'All new hardware, material/mass/aero/contact data are unvalidated assumptions.'}
    dump(HERE/'parameters.json',params)
    dump(HERE/'source_audit/input_summary.json',{'variant':'A','mass_accounting':{'aircraft_only':mass,'sensor':sm,'line':.00225,'total':mass+sm+.00225},
        'aircraft_cg_aru_m':cg.tolist(),'aircraft_cg_frd_from_nose_m':(T@cg).tolist(),
        'inertia_eigenvalues':np.linalg.eigvalsh(ifr).tolist(),'stowed_line_length_m':natural_stowed,
        'surface_preservation':'Source GLB H1 scene only. Actual wing and horizontal-tail sections extracted. Fin planform reconstructed.',
        'body_aero':'No AVL BODY. Fuselage profile drag .035 is an assumption, not a computed H1 body aerodynamic result.'})
    print(json.dumps({'yaml':str(target),'cg_aru':cg.tolist(),'aircraft_mass_kg':mass,'total_mass_kg':mass+sm+.00225,'avl_grid_cases':7*3*5},indent=2))

if __name__=='__main__':main()
