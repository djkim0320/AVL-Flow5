"""Explicit assembly-to-solver adapter for registered CAD and physics models.

The registered UI aircraft origin is already the physical CG frame. The case's
aircraft.cg_m is its location in the aerodynamic geometry file, not an offset
to subtract from UI coordinates a second time.
"""
from pathlib import Path
import copy
import hashlib
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from dbf_stability import load_case, AeroDatabase
from dbf_stability.config import validate
import model_registry
import mechanisms

BACKENDS = ('avl','flow5','hybrid')
ANALYSIS_DIRECTORY = ROOT/'ui/data/analysis'
FIELDS = {
    'speed': ('flight.speed_m_s', 1.01, 100), 'altitude': ('flight.altitude_m', 1, 10000),
    'rho': ('flight.rho_kg_m3', .05, 2),
    'aircraft_mass': ('aircraft.mass_kg', .01, 100), 'sensor_mass': ('sensor.mass_kg', .0001, 10),
    'profile_cd': ('aircraft.profile_cd', 0, 3), 'sensor_cd': ('sensor.cd', 0, 5),
    'density': ('cable.density_kg_m', .000001, 1), 'EA': ('cable.EA_N', .001, 1e7),
    'damping': ('cable.damping_Ns_m', 0, 1e4), 'cd_normal': ('cable.cd_normal', 0, 10),
    'cd_tangent': ('cable.cd_tangent', 0, 10), 'segments': ('cable.segments', 2, 80),
    'max_step': ('simulation.max_step_s', .00001, .1),
    'runtime_limit': ('simulation.maximum_runtime_s', 10, 86400),
}


def number(value, label, lo, hi, integer=False):
    if isinstance(value, bool) or value is None or value == '':
        raise ValueError(f'{label}: 숫자를 입력하세요.')
    try: v = float(value)
    except (TypeError, ValueError): raise ValueError(f'{label}: 숫자를 입력하세요.')
    if not np.isfinite(v) or not lo <= v <= hi or (integer and v != int(v)):
        raise ValueError(f'{label}: {lo}~{hi} 범위의 '+('정수를' if integer else '값을')+' 입력하세요.')
    return int(v) if integer else v


def mesh_signature(item):
    h = hashlib.sha256()
    for p in item['parts']:
        h.update(p['name'].encode('utf8'))
        coordinates=np.asarray(p['positions'], dtype='<f8').copy()
        coordinates[coordinates==0]=0.0 # Browser JSON writes both signed zeros as 0.
        h.update(coordinates.tobytes())
        h.update(np.asarray(p['indices'], dtype='<i8').tobytes())
    return h.hexdigest()


def catalog(model_id):
    profile=model_registry.load(model_id);c=profile['config']
    defaults = {k: c[path.split('.')[0]][path.split('.')[1]] for k,(path,_,_) in FIELDS.items()
                if path.split('.')[1] in c[path.split('.')[0]]}
    defaults.update(backend='avl', task='sequence', phase='stowed', start='equilibrium', sequence_ui_version=1, preflight=2.,
                    duration=2., workers=worker_limit(os_cpu_count()), rebuild=False,
                    controller=False, hold=60., payout=.75, recovery=.3, recovery_length=.1,
                    pitch_delta=1., sensor_yaw_delta=0.,
                    aircraft_inertia=c['aircraft']['inertia_kgm2'])
    if c['sensor'].get('model')!='point_mass':defaults['sensor_inertia']=c['sensor']['inertia_kgm2']
    defaults.update(model_id=model_id,aero_job='new',mission_start='registered',aero_hybrid=dict(coeff='flow5',controls='flow5',rates='avl'),
                    mechanism=mechanisms.defaults(c),aero_grid={k:c['aero'][k] for k in ('alpha_deg','beta_deg','elevator_deg')},
                    gust=c['flight'].get('gust'),controls=c['flight'].get('controls',{}),
                    controller_parameters=c['flight'].get('controller',{}))
    databases={backend:dict(executable_available=(ROOT/c['aero']['flow5_executable' if backend=='flow5' else 'executable']).is_file()) for backend in ('avl','flow5')}
    databases['hybrid']=dict(executable_available=all(row['executable_available'] for row in databases.values()))
    tables=[]
    for p in sorted(ANALYSIS_DIRECTORY.glob('*/aero_metadata.json'),key=lambda p:p.stat().st_mtime,reverse=True):
        request_path=p.parent/'request.json'
        if not request_path.is_file():continue
        request=json.loads(request_path.read_text('utf8'))
        if request['settings'].get('model_id')==model_id and (p.parent/'aero_database.npz').is_file():
            meta=json.loads(p.read_text('utf8'))
            tables.append(dict(id=p.parent.name,backend=request['settings']['backend'],speed=request['settings']['speed'],
                               grid=request['config']['aero'],solver=meta['solver'],composition=meta.get('composition')))
    compatible=[t for t in tables if t['backend']==defaults['backend']]
    if compatible:defaults['aero_job']=compatible[0]['id']
    return dict(model=profile['name'], definition_managed=bool(profile.get('definition_sha256')), defaults=defaults, databases=databases,tables=tables,
                sensor_attachment=c['sensor']['tow_point_m'], aircraft_cg=c['aircraft']['cg_m'],
                provenance=c['provenance'], max_workers=worker_limit(os_cpu_count()),
                scope='등록된 기체 공력과 질점·줄의 결합 운동을 계산합니다. 센서 형상·공력·회전·CAD 접촉은 제외합니다.')


def os_cpu_count():
    import os
    return os.cpu_count() or 1


def worker_limit(cpu_count):
    """Keep four logical CPUs free when possible, with a 12-worker ceiling."""
    return min(12,max(1,int(cpu_count)-4))


def prepare(project, settings):
    """Validate and freeze inputs. Never substitute a different model on failure."""
    profile=model_registry.load(settings.get('model_id'));c=copy.deepcopy(profile['config']);s=copy.deepcopy(settings)
    from aircraft_definition import digest,effective
    definition=effective(project.get('aircraft_definition'))
    if definition is not None or profile.get('definition_sha256'):
        if definition is None or digest(definition)!=profile.get('definition_sha256'):
            raise ValueError('기체 정의가 등록 모델과 다릅니다. 부품·CG·공력 면 변경을 확인하고 새 버전으로 등록하세요.')
    point_mass=project.get('schema')=='dbf-assembly/2'
    defaults=catalog(profile['id'])['defaults']
    for key in ('model_id','aero_job','mission_start','mechanism','aero_grid','gust','controls','controller_parameters','recovery_length','aero_hybrid','preflight','sequence_ui_version'):
        s.setdefault(key,copy.deepcopy(defaults[key]))
    missing=set(defaults)-set(s)
    if point_mass:
        missing-={'sensor_mass','sensor_cd','sensor_inertia','sensor_yaw_delta'}
    if missing: raise ValueError('해석 조건이 빠졌습니다: '+', '.join(sorted(missing)))
    if s['task'] not in ('aero','trim','stability','response','mission','flight','recovery','sequence'): raise ValueError('해석 종류를 확인하세요.')
    if s['task'] in ('flight','recovery','sequence'):
        if not point_mass:raise ValueError('전개 비행·회수는 질점 모델을 사용하세요.')
        s['phase']='stowed' if s['task']=='sequence' else 'recovery' if s['task']=='recovery' else 'deployed'
        s['start']='equilibrium'
    if s['phase'] not in ('aircraft_only','stowed','deployed','recovery') or s['phase']=='recovery' and s['task']!='recovery': raise ValueError('비행 상태를 확인하세요.')
    if s['start'] not in ('equilibrium','scene'): raise ValueError('초기 상태를 확인하세요.')
    if s['backend'] not in BACKENDS: raise ValueError('AVL, flow5 또는 복합 해석기를 선택하세요.')
    if s['backend']=='hybrid':
        from dbf_stability.hybrid import validate_composition,check_reference
        c['aero']['hybrid']=validate_composition(s['aero_hybrid'],c['aero'].get('rate_derivative_closure'))
        check_reference(c)
    uses_flow=s['backend'] in ('flow5','hybrid')
    flow_rates=s['backend']=='flow5' or s['backend']=='hybrid' and s['aero_hybrid']['rates']=='flow5'
    if profile.get('definition_sha256') and flow_rates and c['aero'].get('rate_derivative_closure')!='classical_longitudinal_lateral':
        raise ValueError('flow5를 사용하려면 기체 정의의 공력 면에서 회전율 미계수 근사를 확인한 뒤 새 버전으로 등록하세요. AVL은 이 근사 없이 실행할 수 있습니다.')
    if profile.get('definition_sha256') and uses_flow:
        for surface in profile['aircraft_definition']['surfaces']:
            if surface['axis']=='y' and not surface['mirror']:
                raise ValueError('현재 flow5 자동 연결은 좌우 대칭 수평면과 수직면을 지원합니다. 비대칭 수평면은 AVL을 사용하세요.')
            if surface['axis']=='z' and np.ptp([section['le_m'][1] for section in surface['sections']])>1e-9:
                raise ValueError('현재 flow5 자동 연결은 기울어진 수직면을 지원하지 않습니다. 해당 형상은 AVL을 사용하세요.')
    if s['mission_start'] not in ('registered','scene'):raise ValueError('임무 초기 상태를 확인하세요.')
    for key in ('controller','rebuild'):
        if not isinstance(s[key],bool):raise ValueError(f'{key}: 선택값이 올바르지 않습니다.')
    for key,(path,lo,hi) in FIELDS.items():
        if point_mass and path.startswith('sensor.'):
            continue
        v=number(s[key],key,lo,hi,key=='segments'); a,b=path.split('.'); c[a][b]=v;s[key]=v
    inactive=({'hold','payout','recovery'} if s['task']=='flight' else
              {'hold','payout','duration','pitch_delta'} if s['task']=='recovery' else
              {'duration','pitch_delta'} if s['task']=='sequence' else set())
    for key,lo,hi in [('workers',1,worker_limit(os_cpu_count())),('duration',.01,600),('hold',0,600),
                      ('payout',.001,5),('recovery',.001,5),('pitch_delta',-10,10),('sensor_yaw_delta',-30,30)]:
        if key in inactive or point_mass and key=='sensor_yaw_delta':continue
        s[key]=number(s[key],key,lo,hi,key=='workers')
    if s['task']=='sequence':s['preflight']=number(s['preflight'],'전개 전 비행 시간',.01,600)
    for role in ('aircraft','sensor'):
        if point_mass and role=='sensor':continue
        inertia=np.asarray(s[role+'_inertia'],float)
        if inertia.shape!=(3,3):raise ValueError(f'{role}: 관성은 3×3 행렬이어야 합니다.')
        c[role]['inertia_kgm2']=inertia.tolist()
    if profile.get('definition_sha256'):
        # A single source of truth for UI-defined aircraft physical properties.
        if abs(c['aircraft']['mass_kg']-profile['config']['aircraft']['mass_kg'])>1e-10 or not np.allclose(c['aircraft']['inertia_kgm2'],profile['config']['aircraft']['inertia_kgm2'],rtol=0,atol=1e-10):
            raise ValueError('질량·관성이 기체 정의와 다릅니다. 기체 정의에서 수정하고 새 버전으로 등록하세요.')
        c['aircraft']['mass_kg']=profile['config']['aircraft']['mass_kg']
        c['aircraft']['inertia_kgm2']=copy.deepcopy(profile['config']['aircraft']['inertia_kgm2'])
        s['aircraft_mass']=c['aircraft']['mass_kg'];s['aircraft_inertia']=c['aircraft']['inertia_kgm2']
    task=s['task']; phase='mission' if task in ('mission','sequence') else s['phase']
    if task=='response' and s['start']=='scene' and phase!='deployed':
        raise ValueError('화면 배치 시작은 전개 상태 시간응답에서 선택하세요.')
    coupled=phase!='aircraft_only' and task!='aero'
    required=['aircraft']+(['sensor','winch'] if coupled else [])
    objects=project['objects']
    for role in required:
        if not objects.get(role):raise ValueError(f'{role}: 모델을 먼저 배치하세요.')
        if point_mass and role=='sensor':
            if objects[role].get('kind')!='point_mass':raise ValueError('센서 대신 질점을 추가하세요.')
            continue
        if point_mass and role=='winch':continue
        expected=profile['signatures'][role]
        if mesh_signature(objects[role])!=expected:
            raise ValueError(f'{role}: 선택한 등록 모델의 CAD와 다릅니다. 현재 기체를 등록한 뒤 같은 모델을 선택하세요.')
    c['aero']['backend']=s['backend'];dbpath=None
    build=s['rebuild'] or task=='aero' or s['aero_job']=='new'
    if s['aero_job']!='new':
        import re
        if not re.fullmatch('[0-9a-f]{32}',s['aero_job']):raise ValueError('공력표 ID를 확인하세요.')
        directory=ANALYSIS_DIRECTORY/s['aero_job']; previous=json.loads((directory/'request.json').read_text('utf8'))
        if previous['settings'].get('model_id')!=profile['id']:raise ValueError('다른 등록 모델의 공력표입니다.')
        if uses_flow and abs(previous['settings']['speed']-s['speed'])>1e-8 and not build:
            raise ValueError('flow5 공력표와 현재 속도가 다릅니다. 새로 계산하세요.')
        dbpath=directory/'aero_database.npz'
    if not build:
        if not dbpath.is_file():raise ValueError('공력표가 없습니다. 공력 새 계산을 선택하세요.')
        db=AeroDatabase(dbpath);db.assert_compatible(c)
        for k,axis in zip(('alpha_deg','beta_deg','elevator_deg'),db.axes):c['aero'][k]=axis.tolist()
    else:
        s['rebuild']=True
        for key in ('alpha_deg','beta_deg','elevator_deg'):c['aero'][key]=s['aero_grid'][key]
        if np.prod([len(c['aero'][key]) for key in ('alpha_deg','beta_deg','elevator_deg')])>10000:
            raise ValueError('공력 조건은 한 실행에서 10,000개 이하여야 합니다.')
        required_exes=('executable','flow5_executable') if s['backend']=='hybrid' else ('flow5_executable' if s['backend']=='flow5' else 'executable',)
        for key in required_exes:
            exe=ROOT/c['aero'][key]
            if not exe.is_file():raise ValueError('해석기 실행파일이 없습니다: '+str(exe))
        if uses_flow and max(abs(np.asarray(c['aero']['elevator_deg'],float)))>10:
            raise ValueError('flow5가 포함된 공력 계산의 승강타 범위는 ±10° 이내여야 합니다.')
    # Keep process count inside the user's 12-CPU affinity budget.
    c['aero']['threads_per_worker']=1
    c['simulation']['jacobian_workers']=max(1,s['workers']-1)
    # Recovery duration is derived from line length and speed below, not the
    # hidden flight-duration field. This provisional value is never integrated.
    duration=s['duration'] if task not in ('recovery','sequence') else .01
    c['simulation']['sample_dt_s']=min(.02,duration/10)
    c['simulation']['checkpoint_interval_s']=2.
    c['simulation']['duration_s']=duration
    c['flight']['controller']=copy.deepcopy(s['controller_parameters'])
    c['flight']['controller']['enabled']=s['controller']
    c['flight']['controls']=copy.deepcopy(s['controls']);c['flight']['gust']=copy.deepcopy(s['gust'])
    c['flight']['controller']['altitude_m']=s['altitude'];c['flight']['controller']['airspeed_m_s']=s['speed']
    if task in ('stability','trim') and s['controller']:
        raise ValueError('트림·고유값 계산에서는 제어를 꺼 주세요. 제어 응답은 시간응답으로 평가합니다.')
    if task=='stability' and phase=='stowed':raise ValueError('고유값 해석은 기체 단독 또는 비접촉 전개 평형을 선택하세요.')
    mapping={'aircraft_origin':'registered physical CG frame','sensor_start':'equilibrium','door':'registered geometry',
             'aircraft_cg_local_m':profile['aircraft_cg_local_m'],'sensor_cg_local_m':profile['sensor_cg_local_m'],
             'door_parts':profile['door_parts'],'door_reference_deg':profile['door_reference_deg']}
    if coupled:
        c['cable']['diameter_m']=number(project['cable']['diameter_m'],'줄 직경',.00005,.02)
    m=mechanisms.defaults(c) if point_mass or task in ('flight','recovery') else mechanisms.apply(c,s)
    if point_mass:
        from dbf_stability.point_mass import configure
        mass=number(objects['sensor']['mass_kg'],'질점 질량',.0001,10) if objects.get('sensor') else c['sensor']['mass_kg']
        configure(c,mass)
        mapping.update(payload_model='point_mass',sensor_cg_local_m=[0.,0.,0.],
                       contact_scope='not_modelled',sensor_start='point-mass equilibrium')
        for key in ('sensor_mass','sensor_cd','sensor_inertia','sensor_yaw_delta'):
            s.pop(key,None)
        s['payload_mass_kg']=mass
    if coupled:
        attachment=project['cable'].get('attachment')
        if not attachment:raise ValueError('센서 표면에 줄을 먼저 연결하세요.')
        a=objects['aircraft'];sensor=objects['sensor'];winch=objects['winch']
        r=Rotation.from_quat(a['quaternion']).inv(); origin=np.array(a['position'])
        cg=np.array(profile['aircraft_cg_local_m']);sensor_cg=np.zeros(3) if point_mass else np.array(profile['sensor_cg_local_m'])
        feed=r.apply(np.array(winch['position'])-origin)-cg
        pos=r.apply(np.array(sensor['position'])+Rotation.from_quat(sensor['quaternion']).apply(sensor_cg)-origin)-cg
        orientation=Rotation.identity() if point_mass else r*Rotation.from_quat(sensor['quaternion'])
        wr=r*Rotation.from_quat(winch['quaternion'])
        c['aircraft']['tow_point_m']=feed.tolist()
        c['sensor']['tow_point_m']=(np.array(attachment['local_point_m'])-sensor_cg).tolist()
        c['cable']['length_m']=number(project['cable']['length_m'],'줄 길이',.001,100)
        c['cable']['diameter_m']=number(project['cable']['diameter_m'],'줄 직경',.00005,.02)
        if point_mass:
            # Geometry-free retrieval uses a declared ideal latch near the feed,
            # not a hidden stow/CG offset inherited from the former sensor CAD.
            distance=min(.02,c['cable']['length_m']*.05)
            c['bay']['stowed_center_m']=(feed+[0.,0.,distance]).tolist()
            mapping['point_mass_latch_offset_m']=distance
        if task=='mission' and s['mission_start']=='scene':
            c['bay']['stowed_center_m']=pos.tolist();q=orientation.as_quat();c['bay']['stowed_quaternion_wxyz']=np.r_[q[3],q[:3]].tolist()
        from dbf_stability.math3d import rotation
        from dbf_stability.routing import path_points
        stowed=np.asarray(c['bay']['stowed_center_m'])+rotation(c['bay'].get('stowed_quaternion_wxyz',[1,0,0,0]))@np.array(c['sensor']['tow_point_m'])
        stored=float(np.linalg.norm(np.diff(path_points(c,stowed,feed),axis=0),axis=1).sum())
        if not (0<stored<c['cable']['length_m'] if point_mass else .001<=stored<c['cable']['length_m']):
            raise ValueError('수납 위치에서 필요한 줄 길이가 전개 길이보다 작아야 합니다. 윈치·연결점·줄 길이를 확인하세요.')
        c['winch']['stowed_length_m']=stored
        mapping.update(sensor_position_body_m=pos.tolist(),sensor_quaternion_body_xyzw=orientation.as_quat().tolist(),
                       winch_position_body_m=feed.tolist(),winch_quaternion_body_xyzw=wr.as_quat().tolist())
        equilibrium=phase in ('deployed','recovery') and not (task=='response' and s['start']=='scene')
        if equilibrium and c['cable'].get('guide_points_body_m'):raise ValueError('가이드 접촉이 있는 견인은 비접촉 고유값 대신 화면 배치 시간응답을 선택하세요.')
        if equilibrium and any(abs(c[k]['tow_point_m'][1])>1e-7 for k in ('aircraft','sensor')):
            raise ValueError('현재 견인 트림은 좌우 대칭만 지원합니다. 윈치 출구와 센서 연결점의 Y를 중심선에 맞추거나, 화면 배치 시간응답을 선택하세요.')
        if task=='response' and s['start']=='scene':
            if phase!='deployed':raise ValueError('화면 배치 시작은 전개 상태 시간응답에서 선택하세요.')
            distance=np.linalg.norm(np.diff(path_points(c,pos+orientation.apply(c['sensor']['tow_point_m']),feed),axis=0),axis=1).sum()
            if distance>c['cable']['length_m']*1.05:raise ValueError('배치 거리보다 줄이 5% 이상 짧습니다. 줄 길이를 늘리거나 배치를 수정하세요.')
            mapping['sensor_start']='scene pose; cable material cells follow the initial guide route, shared translational velocity'
        if task=='mission':mapping['sensor_start']=('ideal point-mass latch below winch; no CAD contact' if point_mass else 'scene pose as initial latch/capture target' if s['mission_start']=='scene' else 'registered stowed cradle')
    # Preserve the registered guide/capture geometry, and derive a smooth schedule
    # from the edited feed point, attachment and requested total line length.
    length=c['cable']['length_m']; stored=c['winch']['stowed_length_m']
    if task not in ('flight','recovery','sequence'):
        release=m['release_s'];deploy_end=release+(length-stored)/s['payout'];recover_start=deploy_end+s['hold']
        recover_end=recover_start+(length-stored)/s['recovery'];end=recover_end+.5+m['close_time_s']+.7
        rows=[[0.,stored]]
        if release>0:rows.append([release,stored])
        for start,stop,a,b in [(release,deploy_end,stored,length),(recover_start,recover_end,length,stored)]:
            if rows[-1][0]!=start:rows.append([start,a])
            for u in np.linspace(0,1,21)[1:]:
                rows.append([float(stop if u==1 else start+u*(stop-start)),float(b if u==1 else a+(b-a)*(3*u*u-2*u*u*u))])
        rows.append([end,stored])
        c['winch'].update(length_schedule=rows,recovery_start_s=recover_start,
                         door_schedule=[[0.,m['initial_deg']],[m['open_time_s'],m['open_deg']],[recover_end+.5,m['open_deg']],[recover_end+.5+m['close_time_s'],0.],[end,0.]])
        c['bay']['release_push_until_s']=deploy_end
        if task=='mission':c['simulation']['duration_s']=end
    c['flight']['controller']['enable_from_s']=deploy_end if task=='mission' else 0.
    if task=='sequence':
        # One continuous airborne trajectory, with no fresh trim/state reset at
        # deployment or recovery. The imported door remains in its CAD pose.
        target=number(s['recovery_length'],'회수 후 남길 줄',stored,length)
        if target>=length:raise ValueError('회수 후 남길 줄은 전개 길이보다 짧아야 합니다.')
        s['recovery_length']=target
        release=s['preflight'];deploy_end=release+(length-stored)/s['payout']
        recover_start=deploy_end+s['hold'];end=recover_start+(length-target)/s['recovery']
        if end>600:raise ValueError('전체 계산 시간은 600초 이하여야 합니다. 시간·속도·줄 길이를 확인하세요.')
        rows=[[0.,stored],[release,stored]]
        for start,stop,a,b in [(release,deploy_end,stored,length),(recover_start,end,length,target)]:
            if rows[-1][0]!=start:rows.append([start,a])
            rows.extend([[float(stop if u==1 else start+u*(stop-start)),float(b if u==1 else a+(b-a)*(3*u*u-2*u*u*u))] for u in np.linspace(0,1,21)[1:]])
        c['simulation'].update(duration_s=end,sample_dt_s=min(.02,s['preflight']/10,(deploy_end-release)/20,(end-recover_start)/20))
        c['winch'].update(length_schedule=rows,release_s=release,recovery_start_s=recover_start,
                          capture_enabled=False,door_capture_interlock=False,
                          door_schedule=[[0.,profile['door_reference_deg']],[end,profile['door_reference_deg']]])
        c['bay']['release_push_until_s']=release
        mapping['sensor_start']='stowed at winch; airborne trim; continuous payout, hold and recovery; no capture'
    if task in ('flight','recovery'):
        # Both scenarios begin at the full-length flight equilibrium. The door
        # remains in the imported pose, with no release, capture or door cycle.
        end=duration;rows=[[0.,length],[end,length]]
        if task=='recovery':
            target=number(s['recovery_length'],'회수 후 남길 줄',stored,length)
            if target>=length:raise ValueError('회수 후 남길 줄은 전개 길이보다 짧아야 합니다.')
            s['recovery_length']=target
            end=(length-target)/s['recovery']
            if not .01<=end<=600:raise ValueError('회수 시간은 0.01~600초가 되도록 속도와 남길 줄 길이를 설정하세요.')
            rows=[[float(u*end),float(length if u==0 else target if u==1 else length+(target-length)*(3*u*u-2*u*u*u))] for u in np.linspace(0,1,21)]
        deploy_end=recover_start=0.
        c['simulation']['duration_s']=end
        c['simulation']['sample_dt_s']=min(.02,end/10)
        c['winch'].update(length_schedule=rows,release_s=0.,recovery_start_s=0.,door_capture_interlock=False,
                          door_schedule=[[0.,profile['door_reference_deg']],[end,profile['door_reference_deg']]])
        c['bay']['release_push_until_s']=0.
        mapping['sensor_start']='full-length flight equilibrium; fixed open door; no payout, capture or closing'
    c['provenance']['ui']={'kind':'design','source':'UI request snapshot; registered CAD signature checked; model mass/CG and wake assumptions remain unvalidated',
                           'mapping':mapping,'settings':s}
    c['provenance']['winch']['source']+=(' UI: continuous airborne stowed/payout/hold/recovery sequence; fixed door; no capture; declared mean winch speeds.' if task=='sequence' else ' UI: full-length equilibrium; fixed door; constant length or recovery-only piecewise-linear sampling of smoothstep at declared mean speed; no capture.' if task in ('flight','recovery') else ' UI overrides: smoothstep payout/recovery at declared mean speeds; selected full-length hold.')
    validate(c)
    warnings=(['센서는 질점입니다. 센서 공력·회전·기체/문/가이드 접촉은 계산하지 않습니다. 줄의 질량·공력·탄성·감쇠는 계산합니다.',
               '그림은 줄 길이 미리보기입니다. 트림은 평형을 새로 구하며 전개·회수의 이상적 고정점은 윈치 하방 min(20 mm, 줄 길이의 5%)입니다.',
               '기체 공력은 선택한 실제 AVL/flow5 공력표를 사용합니다. 물성과 수치 수렴 검증은 별도입니다.'] if point_mass else
              ['등록 모델의 물성과 후류 가정을 사용합니다. 시험 검증·수치 수렴 판정은 별도입니다.',
               '윈치 이동·가이드 추가에 따른 질량·CG·관성 변화는 자동 추정하지 않습니다. 이를 포함한 값을 입력하세요.',
               '화면 배치 임무는 센서의 현재 위치·자세를 시작 고정점과 회수 포획 목표로 사용합니다. 트림은 평형을 새로 구합니다.'])
    if task in ('flight','recovery'):
        warnings[1]='완전 전개 상태의 평형에서 시작합니다. 문은 불러온 각도를 유지하며 전개 과정·포획·문 닫힘은 계산하지 않습니다.'
    if task=='sequence':
        warnings[1]='입력한 고도·속도로 비행 중인 수납 상태에서 시작해 전개·유지 비행·회수를 연속 계산합니다. 이륙·포획·문 닫힘은 계산하지 않습니다. 전개·회수 속도는 평균값이며 최대값은 약 1.5배입니다.'
    return dict(config=c,settings=s,mapping=mapping,profile=profile,aero_path=str(dbpath) if dbpath else None,phase=phase,
                duration_s=c['simulation']['duration_s'],schedule=dict(release_s=c['winch']['release_s'],deployed_s=deploy_end,recovery_s=recover_start,end_s=end),
                warnings=warnings)


def stage_meshes(prepared, project, directory):
    """Stage identical collision/replay geometry in physical CG frames."""
    import shutil
    from dbf_stability.collision import read_binary_stl
    from dbf_stability.door import door_hinge,door_reference_shift
    c=prepared['config']; m=prepared['mapping']
    profile=prepared['profile'];replay=copy.deepcopy(project)
    for role,cg in [('aircraft',profile['aircraft_cg_local_m']),('sensor',profile['sensor_cg_local_m'])]:
        if not replay['objects'].get(role):continue
        if replay['objects'][role].get('kind')=='point_mass':continue
        for part in replay['objects'][role]['parts']:
            part['positions']=(np.asarray(part['positions']).reshape(-1,3)-cg).ravel().tolist()
    coupled='winch_position_body_m' in m and c['sensor'].get('model')!='point_mass'
    target=Path(directory)/'meshes'
    if coupled:
        target.mkdir();manifest={}
        door_radius=0.;sensor_radius=0.
        for role,item in replay['objects'].items():
            if not item:continue
            for i,part in enumerate(item['parts']):
                name=f'u_{role}_{i}';v=np.asarray(part['positions']).reshape(-1,3);f=np.asarray(part['indices']).reshape(-1,3)
                door=role=='aircraft' and part['name'] in profile['door_parts']
                if door:door_radius=max(door_radius,float(np.linalg.norm((v-door_hinge(c['bay']))[:,[0,2]],axis=1).max()))
                if role=='sensor':sensor_radius=max(sensor_radius,float(np.linalg.norm(v,axis=1).max()))
                if role=='winch':v=Rotation.from_quat(m['winch_quaternion_body_xyzw']).apply(v)+m['winch_position_body_m']
                manifest[name]=dict(role='sensor' if role=='sensor' else 'door' if door else 'fixed',
                                    concave=role!='sensor',reference_deg=profile['door_reference_deg'],
                                    spool=role=='winch' and part['name']=='winch_drum')
                mechanisms.write_stl(target/(name+'_FRD_m.stl'),v,f)
        c['collision']['parts_manifest']=manifest
        c['bay']['door_mesh_radius_m']=door_radius
        c['sensor']['mesh_radius_m']=sensor_radius
        for name,(v,f) in mechanisms.guide_meshes(c,prepared['settings']['mechanism']):
            mechanisms.write_stl(target/(name+'_FRD_m.stl'),v,f)
            manifest[name]=dict(role='fixed',concave=True,spool=False)
            replay['objects']['aircraft']['parts'].append(dict(name=name,positions=v.ravel().tolist(),indices=f.ravel().tolist(),color='#f59735'))
        c['collision']['mesh_directory']=str(target.resolve())
    (Path(directory)/'replay_project.json').write_text(json.dumps(replay,ensure_ascii=False,allow_nan=False),encoding='utf8')


def initial_from_scene(prepared, trim):
    from dbf_stability.math3d import rotation
    c=prepared['config'];m=prepared['mapping'];y=trim['state'].copy();r=rotation(y[6:10])
    y[13:16]=y[:3]+r@np.array(m['sensor_position_body_m']);y[16:19]=y[3:6]
    q=(Rotation.from_matrix(r)*Rotation.from_quat(m['sensor_quaternion_body_xyzw'])).as_quat()
    y[19:23]=np.r_[q[3],q[:3]];y[23:26]=0
    from dbf_stability.routing import cable_initial_points
    nodes=y[26:].reshape(c['cable']['segments'],6)
    nodes[:,:3]=cable_initial_points(c,y[:13],y[13:26],len(nodes),c['cable']['length_m']);nodes[:,3:]=y[3:6]
    return y
