"""Project validation on generated meshes, independent of browser evidence."""

import copy, json, unittest
from tests.studio.fixtures.aircraft import definition_fixture
from dbf_studio.server import validate_project


class ProjectChecks(unittest.TestCase):
    def setUp(self):
        self.project = definition_fixture()

    def test_roundtrip(self):
        self.assertEqual(validate_project(json.loads(json.dumps(self.project))), self.project)

    def test_invalid_dynamics_pose_and_topology(self):
        edits = [
            lambda p: p['cable'].__setitem__('length_m', 0),
            lambda p: p['cable'].__setitem__('diameter_m', 0.03),
            lambda p: p['objects']['sensor']['quaternion'].__setitem__(3, 2),
            lambda p: p['objects']['sensor']['position'].__setitem__(0, float('inf')),
            lambda p: p['objects']['aircraft']['parts'][0]['indices'].__setitem__(0, 999999999),
            lambda p: p['objects'].__setitem__('winch', None),
        ]
        for edit in edits:
            p = copy.deepcopy(self.project)
            edit(p)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_project(p)
