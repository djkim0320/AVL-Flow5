"""Registration matching uses generated meshes and physical definitions."""

import copy
from tests.studio.fixtures.aircraft import RegisteredCase
from dbf_studio.model_registry import match_aircraft


class ModelConnectionChecks(RegisteredCase):
    def test_matching_and_wrong_selection(self):
        self.assertTrue(match_aircraft(self.project, self.model_id)['matches_selected'])
        result = match_aircraft(self.project, '0' * 32)
        self.assertEqual(result['status'], 'wrong_selection')
        self.assertEqual([x['id'] for x in result['matches']], [self.model_id])

    def test_geometry_and_definition_changes_cannot_borrow_aero(self):
        for edit in (
            lambda p: p['objects']['aircraft']['parts'][0]['positions'].__setitem__(0, 1e-6),
            lambda p: p['aircraft_definition']['mass'].__setitem__('mass_kg', 3.0),
            lambda p: p.pop('aircraft_definition'),
        ):
            p = copy.deepcopy(self.project)
            edit(p)
            self.assertFalse(match_aircraft(p, self.model_id)['matches_selected'])

    def test_pose_payload_mass_and_filename_do_not_change_identity(self):
        p = copy.deepcopy(self.project)
        p['objects']['aircraft']['position'] = [1, 2, 3]
        p['objects']['aircraft']['source']['name'] = 'renamed.step'
        p['objects']['sensor']['mass_kg'] = 0.6
        self.assertTrue(match_aircraft(p, self.model_id)['matches_selected'])
