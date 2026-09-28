"""Real generated aircraft exercises point-mass invariants, not aircraft snapshots."""
from pathlib import Path
import sys,copy
import numpy as np
import pytest
from dbf_stability import build_aero_database,solve_trim,analyze_stability,simulate
from dbf_stability.model import CoupledModel

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.avl
@pytest.mark.slow
def test_registered_point_mass_modes_and_recovery(tmp_path):
    sys.path[:0]=[str(ROOT/'ui/tests'),str(ROOT/'ui')]
    import model_registry
    from fixtures.aircraft import definition_fixture
    from aircraft_definition import register_definition
    from analysis_bridge import catalog,prepare
    original=model_registry.DIRECTORY
    try:
        model_registry.DIRECTORY=tmp_path/'models';p=definition_fixture();record=register_definition(p)
        settings=catalog(record['id'])['defaults'];length=p['cable']['length_m'];target=.75*length
        settings.update(task='recovery',recovery_length=target,recovery=1.,segments=4,workers=2,
                        aero_grid=dict(alpha_deg=[-4.,4.,12.],beta_deg=[-4.,4.],elevator_deg=[-8.,0.,8.]))
        prepared=prepare(p,settings);c=prepared['config']
        if not (ROOT/c['aero']['executable']).is_file():pytest.skip('Actual AVL required')
        db=build_aero_database(c,tmp_path/'aero/aero_database.npz',workers=2);db.assert_compatible(c)
        trim=solve_trim(c,db,mode='deployed');modes=analyze_stability(c,db,trim)
        assert modes['matrix'].shape==(18+6*4,18+6*4)
        model=CoupledModel(c,db,trim,phase='deployed',fixed_length=length)
        try:
            y=trim['state'];rhs,diag=model.rhs(0,y,True)
            assert np.max(abs(rhs[[3,4,5,10,11,12,16,17,18]]))<1e-5
            assert diag['total_mass_kg']==pytest.approx(c['aircraft']['mass_kg']+c['sensor']['mass_kg']+c['cable']['density_kg_m']*length)
            changed=y.copy();changed[19:23]=[0,1,0,0];changed[23:26]=[10,20,30]
            np.testing.assert_allclose(model.rhs(0,changed),rhs,atol=1e-12)
            np.testing.assert_array_equal(rhs[19:26],np.zeros(7))
        finally:model.close()
        result=simulate(c,db,trim,phase='recovery',duration=prepared['duration_s'])
        assert result.summary['status']=='completed'
        assert result.summary['capture_status']=='not_applicable'
        np.testing.assert_array_equal(result.states[0],trim['state'])
        assert result.table.length_m.iloc[-1]==pytest.approx(target)
        assert np.ptp(result.table.total_mass_kg)<1e-12
        assert result.table.active_nodes.iloc[0]>result.table.active_nodes.iloc[-1]
        assert not any(e['event'] in ('payout','stowed','capture_engaged') for e in result.events)
        # Use the same real AVL table for the full airborne sequence. Short
        # durations exercise every material-cell boundary without a long hold.
        settings.update(task='sequence',preflight=.15,hold=.3,payout=2.,recovery=2.,recovery_length=.02)
        prepared=prepare(p,settings);c=prepared['config'];schedule=prepared['schedule']
        stowed_trim=solve_trim(c,db,mode='stowed')
        result=simulate(c,db,stowed_trim,phase=prepared['phase'],duration=prepared['duration_s'])
        assert result.summary['status']=='completed',result.summary
        assert result.summary['capture_status']=='not_applicable'
        np.testing.assert_array_equal(result.states[0],stowed_trim['state'])
        assert result.table.length_m.iloc[0]==pytest.approx(.02)
        assert result.table.length_m.max()==pytest.approx(length)
        assert result.table.length_m.iloc[-1]==pytest.approx(.02)
        assert np.ptp(result.table.total_mass_kg)<1e-12
        assert result.table.active_nodes.max()==c['cable']['segments']
        assert result.table.active_nodes.iloc[-1]==0
        assert {'stowed','payout','tow','recovery'}<=set(result.table.phase)
        assert not any(e['event']=='capture_engaged' for e in result.events)
        # Stage boundaries are stored once, not two unrelated reset trajectories.
        for boundary in (schedule['release_s'],schedule['deployed_s'],schedule['recovery_s']):
            assert np.count_nonzero(np.isclose(result.time,boundary,rtol=0,atol=1e-10))==1
        from mission_sequence import stage_at
        assert [stage_at(t,schedule) for t in (0,schedule['release_s'],schedule['deployed_s'],schedule['recovery_s'])]==['비행 시작','전개','전개 후 비행','회수']
        # The UI switch must yield a real controlled response and exportable
        # tracking metrics, not just a configuration that passes validation.
        settings.update(controller=True)
        controlled=prepare(p,settings)
        cc=controlled['config'];before=copy.deepcopy(cc['flight']['controller'])
        controlled_trim=solve_trim(cc,db,mode='stowed')
        assert cc['flight']['controller']==before  # equilibrium must not turn off the user's control
        np.testing.assert_allclose(controlled_trim['state'],stowed_trim['state'])
        from dbf_stability.control import resolve_controller
        resolved=resolve_controller(cc,db,controlled_trim)
        assert resolved['elevator_pitch_sign'] in (-1,1)
        bad=copy.deepcopy(cc);bad['flight']['controller']['elevator_pitch_sign']=-resolved['elevator_pitch_sign']
        with pytest.raises(ValueError,match='direction opposes'):resolve_controller(bad,db,controlled_trim)
        initial=controlled_trim['state'].copy();initial[2]+=.1
        controlled_result=simulate(cc,db,controlled_trim,phase='mission',initial_state=initial,duration=.1,output=tmp_path/'controlled')
        assert controlled_result.summary['status']=='completed'
        assert controlled_result.summary['controller_enabled']
        assert controlled_result.summary['max_altitude_error_m']>=.1-1e-9
        assert controlled_result.table.controller_active.iloc[-1]
        assert 0<=controlled_result.summary['thrust_saturated_time_fraction']<=1
        from dbf_stability.plots import history_figure
        assert len(history_figure(controlled_result).data)==15
        from analysis_worker import replay_data
        replay_data(controlled_result,p,controlled,tmp_path/'controlled')
        import json
        replay=json.loads((tmp_path/'controlled/replay.json').read_text('utf8'))
        assert replay['controller_enabled'] and replay['model_id']==record['id']
        assert replay['frames'][-1]['speed_m_s']==pytest.approx(controlled_result.table.airspeed_m_s.iloc[-1])
    finally:model_registry.DIRECTORY=original
