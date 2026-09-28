"""File components must never apply a second pose to the solver mesh."""
import copy
import sys
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ui'),str(ROOT/'src')]
from fixtures.aircraft import definition_fixture
from server import validate_project
from aircraft_definition import export_aero
from model_registry import signature as mesh_signature


class AssemblyTests(unittest.TestCase):
    def assembly(self):
        p=definition_fixture();a=p['objects']['aircraft']
        a['components']=[dict(id=str(i),name=part['name']+'.step',parts=[i],position=[i*4.,1.,2.],
                              quaternion=[0.,0.,0.,1.],locked=False,source=dict(origin='file_origin'))
                         for i,part in enumerate(a['parts'])]
        return p

    def test_saved_assembly_does_not_double_transform_baked_mesh_or_aero(self):
        p=self.assembly();plain=copy.deepcopy(p);del plain['objects']['aircraft']['components']
        validate_project(p)
        self.assertEqual(mesh_signature(p['objects']['aircraft']),mesh_signature(plain['objects']['aircraft']))
        self.assertEqual(export_aero(p['aircraft_definition'],p['objects']['aircraft']),
                         export_aero(plain['aircraft_definition'],plain['objects']['aircraft']))

    def test_invalid_component_metadata_is_rejected_at_save(self):
        for mutate in [lambda c:c[1].update(parts=[0]),lambda c:c[0].update(parts=[True]),
                       lambda c:c[0].update(quaternion=[0,0,0,2]),lambda c:c[0].update(position=[float('nan'),0,0]),
                       lambda c:c.pop(),lambda c:c[0].update(locked='yes')]:
            p=self.assembly();mutate(p['objects']['aircraft']['components'])
            with self.assertRaises(ValueError):validate_project(p)

    def test_baked_component_movement_changes_the_registered_geometry_signature(self):
        p=self.assembly();a=p['objects']['aircraft'];before=mesh_signature(a)
        a['parts'][0]['positions'][0]+=.001
        self.assertNotEqual(before,mesh_signature(a))

if __name__=='__main__':unittest.main()
