"""Analytic block selection tests plus separately marked real solver execution."""
from pathlib import Path
import copy,hashlib,json,sys
import numpy as np
import pytest
from dbf_stability.avl import AeroDatabase,build_aero_database
from dbf_stability.aero_inputs import input_fingerprint
from dbf_stability.hybrid import compose_tables,consistency_report,DEFAULT_COMPOSITION,check_reference

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def tables(tmp_path):
    geometry=tmp_path/'plane.avl';geometry.write_text('Analytic\n0\n0 0 0\n1 1 1\n0 0 0\n0\n')
    axes=[np.array([0.,4.])]*3;shape=(2,2,2)
    sources={}
    for source,factor in [('avl',1.),('flow5',1.1)]:
        coeff=np.zeros((*shape,6))
        for i,alpha in enumerate(axes[0]):
            for j,beta in enumerate(axes[1]):
                lift=factor*.1*alpha
                coeff[i,j,:,0]=lift*np.sin(np.deg2rad(alpha));coeff[i,j,:,2]=-lift*np.cos(np.deg2rad(alpha))
                coeff[i,j,:,4]=factor*.2*alpha
                for column,slope in [(1,.3),(3,.4),(5,.5)]:coeff[i,j,:,column]=factor*slope*beta
        meta=dict(solver='AVL 3.52' if source=='avl' else 'flow5 v7.57',axes='FRD body',angle_unit='degrees',rate_unit='pb/2V,qc/2V,rb/2V',
            geometry_sha256=hashlib.sha256(geometry.read_bytes()).hexdigest(),input_files_sha256=input_fingerprint(geometry),moment_reference_frd_m=[0.,0.,0.],
            raw_directory=str(tmp_path/source),flow5_speed_m_s=20.,rate_derivative_closure='classical_longitudinal_lateral',
            control_indices=dict(elevator=1,lateral=[2,3]))
        path=tmp_path/(source+'.npz')
        np.savez(path,alpha=axes[0],beta=axes[1],elevator=axes[2],refs=[1.,1.,1.],coeff=coeff,rates=np.full((*shape,6,3),factor),controls=np.full((*shape,6,2),10*factor),metadata=json.dumps(meta))
        sources[source]=path
    cfg=dict(_root=str(tmp_path),aero=dict(backend='hybrid',geometry='plane.avl',hybrid=copy.deepcopy(DEFAULT_COMPOSITION),moment_reference_frd_m=[0,0,0],elevator_index=1,lateral_control_indices=[2,3]),aircraft=dict(cg_m=[0,0,0]),flight=dict(speed_m_s=20.))
    return sources,cfg

def rewrite(path,**changes):
    with np.load(path,allow_pickle=False) as data:arrays={k:data[k].copy() for k in data.files}
    meta=json.loads(str(arrays['metadata']))
    for key,value in changes.items():
        if key in arrays:arrays[key]=value
        else:meta[key]=value
    arrays['metadata']=json.dumps(meta);np.savez(path,**arrays)

def test_exact_blocks_compatibility_and_no_extrapolation(tables,tmp_path):
    sources,c=tables;db=compose_tables(sources,DEFAULT_COMPOSITION,tmp_path/'composed.npz');db.assert_compatible(c)
    for block,source in DEFAULT_COMPOSITION.items():
        with np.load(sources[source]) as raw:np.testing.assert_array_equal(getattr(db,block).values,raw[block])
    for name,record in db.metadata['sources'].items():assert record['npz_sha256']==hashlib.sha256(sources[name].read_bytes()).hexdigest()
    assert db.metadata['discarded']==['avl coeff','avl controls','flow5 rates']
    with pytest.raises(FileExistsError):compose_tables(sources,DEFAULT_COMPOSITION,tmp_path/'composed.npz')
    with pytest.raises(ValueError):db.evaluate(np.deg2rad(5),0,0,[0,0,0],20)
    for backend in ('avl','flow5'):
        other=copy.deepcopy(c);other['aero']['backend']=backend
        with pytest.raises(ValueError,match='solver'):db.assert_compatible(other)
    other=copy.deepcopy(c);other['flight']['speed_m_s']=21
    with pytest.raises(ValueError,match='속도'):db.assert_compatible(other)

@pytest.mark.parametrize('changes',[
    dict(geometry_sha256='different'),dict(input_files_sha256={'geometry':'different'}),dict(refs=[1,1,1.01]),
    dict(alpha=[0,3]),dict(beta=[0,3]),dict(elevator=[0,3]),dict(moment_reference_frd_m=[0,0,.0001]),dict(flow5_speed_m_s=None)])
def test_reject_incompatible_raw_tables(tables,tmp_path,changes):
    sources,_=tables;rewrite(sources['flow5'],**changes)
    with pytest.raises(ValueError):compose_tables(sources,DEFAULT_COMPOSITION,tmp_path/'invalid.npz')
    assert not (tmp_path/'invalid.npz').exists()

def test_composition_and_reference_validation(tables,tmp_path):
    sources,c=tables
    for composition in (dict(coeff='avl',controls='avl',rates='avl'),dict(coeff='unknown',controls='flow5',rates='avl'),{'coeff':'avl'}):
        with pytest.raises(ValueError):compose_tables(sources,composition,tmp_path/'invalid.npz')
    check_reference(c);c['aircraft']['cg_m']=[.01,0,0]
    with pytest.raises(ValueError,match='CG'):check_reference(c)

def test_consistency_recovers_known_slopes(tables):
    report=consistency_report(*tables[0].values())
    for key,slope in [('CL_alpha_per_deg',.1),('Cm_alpha_per_deg',.2),('CY_beta_per_deg',.3),('Cl_beta_per_deg',.4),('Cn_beta_per_deg',.5)]:
        row=report[key];assert row['avl']==pytest.approx(slope);assert row['flow5']==pytest.approx(slope*1.1)
        assert row['relative_difference']==pytest.approx(.1/1.1);assert not row['warning'];assert row['method']=='boundary_secant'

def test_warning_and_near_zero_are_finite(tables,tmp_path):
    sources,_=tables
    with np.load(sources['flow5']) as raw:coeff=raw['coeff'].copy()
    coeff[...,4]*=2;coeff[...,1]=0
    rewrite(sources['flow5'],coeff=coeff)
    report=consistency_report(*sources.values());assert report['Cm_alpha_per_deg']['warning'];assert report['CY_beta_per_deg']['relative_difference']==1
    for path in sources.values():
        with np.load(path) as raw:coeff=raw['coeff'].copy()
        coeff[...,3]=0;rewrite(path,coeff=coeff)
    row=consistency_report(*sources.values())['Cl_beta_per_deg'];assert row['relative_difference'] is None;assert not row['warning']

@pytest.mark.avl
@pytest.mark.slow
def test_real_hybrid_generated_aircraft(tmp_path):
    sys.path[:0]=[str(ROOT/'ui/tests'),str(ROOT/'ui')]
    from fixtures.aircraft import definition_fixture
    from aircraft_definition import export_aero,generic_case
    p=definition_fixture();a=export_aero(p['aircraft_definition'],p['objects']['aircraft'])
    for file in a['files']:(tmp_path/file['name']).write_text(file['text'],encoding='ascii')
    c=generic_case(p['aircraft_definition'],p,a);c['_root']=str(ROOT)
    c['aero'].update(backend='hybrid',hybrid=copy.deepcopy(DEFAULT_COMPOSITION),geometry=str(tmp_path/'aircraft.avl'),
        flow5_executable='vendor/flow5/bin/flow5_v7.57_win64/flow5.exe',alpha_deg=[0.,4.],beta_deg=[0.,4.],elevator_deg=[0.,4.])
    for key in ('executable','flow5_executable'):
        if not (ROOT/c['aero'][key]).is_file():pytest.skip('Actual solver installation required: '+key)
    unchanged=copy.deepcopy(c)
    db=build_aero_database(c,tmp_path/'result/aero_database.npz',workers=2);db.assert_compatible(c)
    assert c==unchanged;assert db.metadata['backend']=='hybrid';assert db.metadata['flow5_speed_m_s']==c['flight']['speed_m_s']
    assert db.coeff.values.shape==(2,2,2,6)
    for name,record in db.metadata['sources'].items():
        assert len(record['npz_sha256'])==64;assert Path(record['raw_directory']).is_dir()
    check_reference(c,next((tmp_path/'result/avl').glob('avl_runs_*/case_*/plane.avl')))
