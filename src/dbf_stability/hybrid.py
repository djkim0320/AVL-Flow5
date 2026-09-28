"""Strict block composition of independently computed AVL and flow5 tables.

No resampling, reference transfers or replacement coefficients. Differences
between solvers are reported, not treated as experimental accuracy estimates.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import copy
import hashlib
import json
import numpy as np
from .avl import AeroDatabase
from .compute_resources import worker_limit

DEFAULT_COMPOSITION = dict(coeff='flow5', controls='flow5', rates='avl')
BLOCKS = ('coeff', 'controls', 'rates')
CLOSURE = 'classical_longitudinal_lateral'


def validate_composition(composition, closure=None, *, check_closure=True):
    if not isinstance(composition, dict) or set(composition)!=set(BLOCKS) or any(v not in ('avl','flow5') for v in composition.values()):
        raise ValueError('복합 공력표의 coeff·controls·rates 출처를 avl 또는 flow5로 지정하세요.')
    if len(set(composition.values()))==1:raise ValueError('세 블록의 출처가 같습니다. 단일 해석기를 선택하세요.')
    if check_closure and composition['rates']=='flow5' and closure!=CLOSURE:
        raise ValueError('flow5 회전율 미계수에는 종·횡 분리 근사 동의가 필요합니다.')
    return dict(composition)


def avl_reference_frd(path):
    lines=[s.split('#')[0].split('!')[0].strip() for s in Path(path).read_text(encoding='ascii').splitlines()]
    lines=[s for s in lines if s]
    try:point=np.asarray([float(v) for v in lines[4].split()],float)
    except (ValueError,IndexError) as exc:raise ValueError('AVL 형상의 기준점 헤더를 읽을 수 없습니다.') from exc
    if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('AVL Xref·Yref·Zref는 유한한 세 좌표여야 합니다.')
    return point*np.array([-1.,1.,-1.])


def check_reference(config, path=None):
    cg=np.asarray(config['aircraft']['cg_m'],float)
    declared=np.asarray(config['aero']['moment_reference_frd_m'],float)
    actual=avl_reference_frd(path or Path(config['_root'])/config['aero']['geometry'])
    if cg.shape!=(3,) or not np.isfinite(cg).all() or declared.shape!=(3,) or not np.allclose(declared,cg,rtol=0,atol=1e-9) or not np.allclose(actual,cg,rtol=0,atol=1e-9):
        raise ValueError('AVL 형상의 Xref·Yref·Zref가 CG와 다릅니다. 기체 정의에서 다시 등록하세요.')


def _tables(sources):
    if set(sources)!={'avl','flow5'}:raise ValueError('복합 공력표에는 AVL과 flow5 원본 두 개가 필요합니다.')
    dbs={name:AeroDatabase(path.path if isinstance(path,AeroDatabase) else path) for name,path in sources.items()}
    for name,db in dbs.items():
        if not db.metadata.get('solver','').lower().startswith(name):raise ValueError('원본 해석기 출처가 다릅니다: '+name)
        for key in ('geometry_sha256','input_files_sha256'):
            if not db.metadata.get(key):raise ValueError('원본 공력표의 해시가 없습니다: '+key)
    a,b=dbs['avl'],dbs['flow5']
    for key in ('geometry_sha256','input_files_sha256'):
        if a.metadata[key]!=b.metadata[key]:raise ValueError('원본 공력표의 형상·입력 해시가 다릅니다: '+key)
    if np.any(abs(a.refs-b.refs)>1e-5*np.maximum(abs(a.refs),abs(b.refs))):raise ValueError('원본 공력표의 기준 면적·시위·날개폭이 다릅니다.')
    if any(not np.array_equal(x,y) for x,y in zip(a.axes,b.axes)):raise ValueError('원본 공력표의 격자가 다릅니다. 같은 격자로 다시 계산하세요.')
    if not np.allclose(a.metadata['moment_reference_frd_m'],b.metadata['moment_reference_frd_m'],rtol=0,atol=1e-9):
        raise ValueError('원본 공력표의 모멘트·회전 기준점이 다릅니다.')
    return dbs


def _consistency(dbs):
    anchor=tuple(int(np.argmin(abs(x))) for x in dbs['avl'].axes)
    report={}
    for label,dimension,column in [('CL_alpha_per_deg',0,None),('Cm_alpha_per_deg',0,4),('CY_beta_per_deg',1,1),('Cl_beta_per_deg',1,3),('Cn_beta_per_deg',1,5)]:
        grid=dbs['avl'].axes[dimension];center=anchor[dimension]
        low=max(0,center-1);high=min(len(grid)-1,center+1)
        values={}
        for name,db in dbs.items():
            pair=[]
            for i in (low,high):
                index=list(anchor);index[dimension]=i;coeff=db.coeff.values[tuple(index)]
                alpha=np.deg2rad(db.axes[0][index[0]])
                pair.append(float(coeff[column]) if column is not None else float(coeff[0]*np.sin(alpha)-coeff[2]*np.cos(alpha)))
            values[name]=(pair[1]-pair[0])/float(grid[high]-grid[low])
        absolute=abs(values['avl']-values['flow5']);scale=max(abs(v) for v in values.values())
        relative=absolute/scale if scale>1e-10 else None
        report[label]={**values,'absolute_difference':absolute,'relative_difference':relative,
                       'warning':relative is not None and relative>.2,
                       'method':'centered_secant' if 0<center<len(grid)-1 else 'boundary_secant',
                       'sample_angles_deg':[float(x[i]) for x,i in zip(dbs['avl'].axes,anchor)],
                       'difference_interval_deg':[float(grid[low]),float(grid[high])]}
    return report


def consistency_report(avl_npz, flow5_npz):
    return _consistency(_tables(dict(avl=avl_npz,flow5=flow5_npz)))


def compose_tables(sources, composition, output):
    composition=validate_composition(composition,check_closure=False)
    output=Path(output).resolve()
    if output.exists():raise FileExistsError(output)
    dbs=_tables(sources);a=dbs[composition['coeff']]
    flow=dbs['flow5'].metadata
    if composition['rates']=='flow5' and flow.get('rate_derivative_closure')!=CLOSURE:
        raise ValueError('flow5 원본의 회전율 근사 정보가 없습니다.')
    speed=flow.get('flow5_speed_m_s')
    if not isinstance(speed,(int,float)) or isinstance(speed,bool) or not np.isfinite(speed) or speed<=0:
        raise ValueError('flow5 원본 공력표의 계산 속도가 없습니다. 새로 계산하세요.')
    records={name:dict(raw_directory=db.metadata.get('raw_directory'),npz_sha256=hashlib.sha256(Path(db.path).read_bytes()).hexdigest(),
                      npz_path=db.path,solver=db.metadata['solver'],moment_reference_frd_m=db.metadata['moment_reference_frd_m']) for name,db in dbs.items()}
    records['flow5']['speed_m_s']=speed
    limitations=['정적·조종 계수와 회전율 미계수는 서로 다른 해석기의 결과를 조합합니다. 시험으로 검증한 정확도가 아닙니다.',
                 'flow5는 VLM2 양력면 모델이며 동체 공력은 포함하지 않습니다. 추가 항력은 입력한 가정값입니다.',
                 'flow5의 실제 조종면 회전과 AVL의 선형화된 조종면 처리는 서로 다릅니다.',
                 '격자와 기준점이 같은 원본만 결합하며, 격자 밖으로 외삽하지 않습니다.']
    if composition['rates']=='flow5':limitations.append('flow5의 종·횡 교차 회전율 미계수는 0으로 가정합니다. 큰 옆미끄럼에서 미검증입니다.')
    metadata={k:a.metadata[k] for k in ('axes','angle_unit','rate_unit','geometry_sha256','input_files_sha256','moment_reference_frd_m')}
    metadata.update(solver='hybrid: '+' + '.join(db.metadata['solver']+' ('+', '.join(k for k in BLOCKS if composition[k]==name)+')' for name,db in dbs.items()),
                    backend='hybrid',composition=composition,sources=records,flow5_speed_m_s=speed,
                    discarded=[name+' '+key for name in dbs for key in BLOCKS if composition[key]!=name],
                    consistency=_consistency(dbs),consistency_warning_threshold=.2,limitations=limitations)
    if composition['controls']=='avl':metadata['control_indices']=dbs['avl'].metadata['control_indices']
    if composition['rates']=='flow5':metadata['rate_derivative_closure']=CLOSURE
    output.parent.mkdir(parents=True,exist_ok=True)
    arrays={key:getattr(dbs[composition[key]],key).values for key in BLOCKS}
    # Exclusive creation also prevents races between independent callers.
    with output.open('xb') as stream:
        np.savez_compressed(stream,alpha=a.axes[0],beta=a.axes[1],elevator=a.axes[2],refs=a.refs,
                            **arrays,metadata=json.dumps(metadata,ensure_ascii=False,allow_nan=False))
    return AeroDatabase(output)


def build_hybrid_database(config, output=None, workers=None):
    from .avl import build_aero_database
    composition=validate_composition(config['aero'].get('hybrid'),config['aero'].get('rate_derivative_closure'))
    check_reference(config)
    output=Path(output or Path(config['_root'])/'outputs/hybrid_aero_database.npz').resolve()
    if output.exists():raise FileExistsError(output)
    if max(abs(np.asarray(config['aero']['elevator_deg'],float)))>10:raise ValueError('복합 공력표의 승강타 범위는 ±10° 이내여야 합니다.')
    maximum=worker_limit();requested=maximum if workers is None else workers
    if isinstance(requested,bool) or not isinstance(requested,int) or not 1<=requested<=maximum:
        raise ValueError(f'계산 자원은 1~{maximum}개 CPU로 지정하세요.')
    for key in ('executable','flow5_executable'):
        if not (Path(config['_root'])/config['aero'][key]).is_file():raise FileNotFoundError(config['aero'][key])
    children={}
    for name in ('avl','flow5'):
        child=copy.deepcopy(config);child['aero']['backend']=name;child['aero']['threads_per_worker']=1
        if name=='flow5' and composition['rates']!='flow5':child['aero']['rate_derivative_closure']=CLOSURE
        children[name]=child
        if (output.parent/name).exists():raise FileExistsError(output.parent/name)
    counts=dict(avl=max(1,requested//2),flow5=max(1,requested-requested//2))
    def run(name):return build_aero_database(children[name],output.parent/name/'aero_database.npz',counts[name])
    if requested==1:dbs={name:run(name) for name in children}
    else:
        # Each subprocess builder owns its process pool; split, never multiply the CPU budget.
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures={name:pool.submit(run,name) for name in children}
            dbs={name:future.result() for name,future in futures.items()}
    for name,db in dbs.items():db.assert_compatible(children[name])
    copied=list(Path(dbs['avl'].metadata['raw_directory']).glob('case_*/plane.avl'))
    if not copied:raise ValueError('실제 실행에 사용한 AVL 형상 사본이 없습니다.')
    for path in copied:check_reference(config,path)
    return compose_tables(dbs,composition,output)
