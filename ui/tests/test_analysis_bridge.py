"""Analytic assembly mapping and rejection checks."""
import copy
import sys
import unittest
from pathlib import Path
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import validate_project
from fixtures.aircraft import RegisteredCase
from analysis_bridge import catalog,prepare,stage_meshes,ROOT
from dbf_stability.collision import read_binary_stl


class AnalysisBridgeChecks(RegisteredCase):
    def setUp(self):self.p=copy.deepcopy(self.project);self.s=copy.deepcopy(self.settings)

    def test_real_reference_and_surface_are_valid(self):
        validate_project(self.p);self.s['phase']='deployed'
        r=prepare(self.p,self.s)
        self.assertEqual(r['config']['aero']['backend'],'avl')
        self.assertEqual(r['config']['cable']['length_m'],self.project['cable']['length_m'])

    def test_global_rigid_transform_does_not_change_body_mapping(self):
        self.s.update(task='response',phase='deployed',start='scene')
        a=prepare(self.p,self.s)
        r=Rotation.from_euler('xyz',[.2,-.1,.8]);offset=np.array([2,-4,1])
        for item in self.p['objects'].values():
            item['position']=(r.apply(item['position'])+offset).tolist()
            item['quaternion']=(r*Rotation.from_quat(item['quaternion'])).as_quat().tolist()
        b=prepare(self.p,self.s)
        for key in ('sensor_position_body_m','winch_position_body_m'):
            np.testing.assert_allclose(a['mapping'][key],b['mapping'][key],atol=1e-12)


    def test_modified_geometry_cannot_borrow_other_aero(self):
        self.p['objects']['aircraft']['parts'][0]['positions'][0]+=.001
        with self.assertRaisesRegex(ValueError,'등록 모델의 CAD'):prepare(self.p,self.s)

    def test_browser_signed_zero_serialization_keeps_geometry_identity(self):
        for item in self.p['objects'].values():
            for part in item.get('parts',[]):part['positions']=[0.0 if v==0 else v for v in part['positions']]
        prepare(self.p,self.s)

    def test_asymmetric_trim_is_rejected_before_execution(self):
        self.s['phase']='deployed';self.p['objects']['winch']['position'][1]=.01
        with self.assertRaisesRegex(ValueError,'좌우 대칭'):prepare(self.p,self.s)

    def test_controller_modes_invalid_inertia_and_missing_values(self):
        for changes in ({'controller':True,'task':'stability'},{'aircraft_inertia':[[0,0,0],[0,1,0],[0,0,1]]},{'EA':''},{'workers':13}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):prepare(self.p,{**self.s,**changes})

    def test_control_switch_has_complete_settings_for_legacy_projects(self):
        self.s.update(task='sequence',controller=True,controller_parameters={'enabled':False})
        result=prepare(self.p,self.s);c=result['config']['flight']['controller']
        self.assertTrue(c['enabled'])
        self.assertEqual(c['type'],'longitudinal_pd')
        self.assertEqual(c['elevator_pitch_sign'],'from_aero')
        self.assertEqual(c['enable_from_s'],0.)
        self.assertEqual(c['altitude_m'],self.s['altitude'])
        self.assertEqual(c['airspeed_m_s'],self.s['speed'])
        self.assertIn('controller',result['config']['provenance'])

    def test_explicit_bad_controller_is_not_replaced_by_defaults(self):
        self.s.update(controller=True,controller_parameters={'type':'longitudinal_pd','pitch_kp':0.})
        with self.assertRaises(ValueError):prepare(self.p,self.s)

    def test_scene_start_cannot_silently_become_aircraft_perturbation(self):
        self.s.update(task='response',phase='aircraft_only',start='scene')
        with self.assertRaisesRegex(ValueError,'전개 상태'):prepare(self.p,self.s)

