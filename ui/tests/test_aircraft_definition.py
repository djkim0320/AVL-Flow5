"""Physical invariants and the real CAD-to-solver path, without mocked aero."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ui'),str(ROOT/'src')]
from aircraft_definition import (digest,validate_draft,mass_properties,export_aero,
                                extract_sections,generic_case,register_definition)
from dbf_stability.config import validate




from fixtures.aircraft import definition_fixture

class DefinitionTests(unittest.TestCase):
    def test_cg_parallel_axis_and_no_sensor_mass(self):
        d=definition_fixture()['aircraft_definition'];d['mass']['mode']='components';d['parts']=[]
        for i,(m,x) in enumerate([(2.,0.),(1.,3.)]):
            d['parts'].append(dict(index=i,name=str(i),mass_kg=m,cg_m=[x,0.,0.],inertia_kgm2=(np.eye(3)*.2).tolist()))
        result=mass_properties(d)
        self.assertEqual(result['mass_kg'],3.)
        np.testing.assert_allclose(result['cg_m'],[1,0,0])
        np.testing.assert_allclose(result['inertia_kgm2'],np.diag([.4,6.4,6.4]))

    def test_incomplete_draft_can_save_but_not_analyze(self):
        p=definition_fixture();p['aircraft_definition']['mass']['cg_m'][0]=None
        validate_draft(p['aircraft_definition'],p['objects']['aircraft'])
        with self.assertRaises(ValueError):mass_properties(p['aircraft_definition'])
        p['aircraft_definition']['mass']['cg_m'][0]=0
        p['aircraft_definition']['mass']['inertia_kgm2']=np.diag([1.,1.,3.]).tolist()
        with self.assertRaises(ValueError):mass_properties(p['aircraft_definition'])

    def test_mesh_intersection_measures_chord_and_does_not_invent_missing_geometry(self):
        p=definition_fixture();r=extract_sections(p['objects']['aircraft'],dict(parts=[0],axis='y',stations_m=[.1,.7]))
        self.assertEqual(len(r['sections']),2)
        self.assertAlmostEqual(r['sections'][0]['chord_m'],.3)
        self.assertAlmostEqual(r['sections'][0]['le_m'][1],.1)
        with self.assertRaises(ValueError):extract_sections(p['objects']['aircraft'],dict(parts=[0],axis='y',stations_m=[-1.,1.]))


    def test_generator_and_flow5_input_agree_on_coordinates(self):
        from dbf_stability.flow5 import read_lifting_surfaces
        p=definition_fixture();a=export_aero(p['aircraft_definition'],p['objects']['aircraft'])
        self.assertEqual(a['controls'],dict(elevator=2,aileron=1,rudder=3))
        self.assertIn('0.8 0.3 -0.0',a['files'][0]['text'])
        with tempfile.TemporaryDirectory() as tmp:
            for f in a['files']:(Path(tmp)/f['name']).write_text(f['text'],encoding='ascii')
            refs,surfaces=read_lifting_surfaces(Path(tmp)/'aircraft.avl')
            np.testing.assert_allclose(refs,[.48,.3,1.6]);self.assertEqual(len(surfaces),3)
        c=generic_case(p['aircraft_definition'],p,a);validate(c)
        self.assertFalse(c['collision']['enabled']);self.assertEqual(c['aircraft']['mass_kg'],2.)

    def test_one_cad_wing_can_have_distinct_fixed_and_controlled_spans(self):
        p=definition_fixture();d=p['aircraft_definition'];wing=d['surfaces'][0]
        outer=copy.deepcopy(wing);outer['sections'][0]['le_m'][1]=.4
        wing['sections'][-1]['le_m'][1]=.4;wing['control']='none'
        d['surfaces'].append(outer)
        self.assertIsNotNone(export_aero(d,p['objects']['aircraft'])['controls']['aileron'])
        outer['sections'][0]['le_m'][1]=.3
        with self.assertRaisesRegex(ValueError,'겹칩니다'):export_aero(d,p['objects']['aircraft'])

    def test_definition_changes_require_registration_and_mapping_uses_cg(self):
        import model_registry
        from analysis_bridge import catalog,prepare
        p=definition_fixture();original=model_registry.DIRECTORY
        with tempfile.TemporaryDirectory() as tmp:
            try:
                model_registry.DIRECTORY=Path(tmp)
                result=register_definition(p);model=model_registry.load(result['id'])
                self.assertEqual(model['definition_sha256'],digest(p['aircraft_definition']))
                self.assertTrue(model_registry.match_aircraft(p,result['id'])['matches_selected'])
                settings=catalog(result['id'])['defaults'];settings.update(task='flight',phase='deployed')
                r=prepare(p,settings)
                np.testing.assert_allclose(r['config']['aircraft']['tow_point_m'],[-.015,0.,.02])
                wrong=copy.deepcopy(settings);wrong['aircraft_mass']+=1
                with self.assertRaisesRegex(ValueError,'질량·관성'):prepare(p,wrong)
                flow=copy.deepcopy(settings);flow['backend']='flow5'
                with self.assertRaisesRegex(ValueError,'근사'):prepare(p,flow)
                p['aircraft_definition']['mass']['cg_m'][0]+=.01
                self.assertFalse(model_registry.match_aircraft(p,result['id'])['matches_selected'])
                with self.assertRaisesRegex(ValueError,'기체 정의'):prepare(p,settings)
            finally:model_registry.DIRECTORY=original



if __name__=='__main__':unittest.main()
