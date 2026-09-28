"""The simplified UI drives the real aircraft table and geometry-free dynamics."""

import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from dbf_studio.server import validate_project
from dbf_studio.analysis_bridge import prepare, stage_meshes
from dbf_stability.avl import write_mass_file


from tests.studio.fixtures.aircraft import RegisteredCase


class PointMassChecks(RegisteredCase):
    def setUp(self):
        self.prepared = prepare(validate_project(self.project), self.settings)

    def test_no_sensor_geometry_or_inertia_reaches_solver(self):
        c = self.prepared['config']
        self.assertEqual(set(c['sensor']), {'model', 'mass_kg', 'tow_point_m'})
        self.assertEqual(c['sensor']['mass_kg'], 0.12)
        self.assertFalse(c['collision']['enabled'])
        self.assertNotIn('sensor_inertia', self.prepared['settings'])
        with tempfile.TemporaryDirectory() as folder:
            prepared = copy.deepcopy(self.prepared)
            stage_meshes(prepared, self.project, folder)
            replay = json.loads((Path(folder) / 'replay_project.json').read_text('utf8'))
            self.assertNotIn('parts', replay['objects']['sensor'])
            self.assertFalse((Path(folder) / 'meshes').exists())
            write_mass_file(c, Path(folder) / 'point.mass', stowed=True)
            sensor_line = next(
                line for line in (Path(folder) / 'point.mass').read_text().splitlines() if '# stowed sensor' in line
            )
            self.assertTrue(all(float(v) == 0 for v in sensor_line.split('#')[0].split()[4:]))

    def test_reject_mesh_mass_and_inconsistent_length(self):
        for key, value in [('mass_kg', 0), ('mass_kg', True), ('mass_kg', float('nan')), ('parts', [])]:
            p = copy.deepcopy(self.project)
            p['objects']['sensor'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_project(p)
        p = copy.deepcopy(self.project)
        p['cable']['length_m'] = 3
        with self.assertRaisesRegex(ValueError, '미리보기'):
            validate_project(p)

    def test_mapping_under_global_rotation(self):
        p = copy.deepcopy(self.project)
        r = Rotation.from_euler('xyz', [0.2, 0.3, 0.1])
        shift = np.array([3, 2, 1])
        for role, obj in p['objects'].items():
            obj['position'] = (r.apply(obj['position']) + shift).tolist()
            if role != 'sensor':
                obj['quaternion'] = r.as_quat().tolist()
        actual = prepare(validate_project(p), self.settings)
        for key in ('sensor_position_body_m', 'winch_position_body_m'):
            np.testing.assert_allclose(actual['mapping'][key], self.prepared['mapping'][key], atol=1e-12)

    def test_short_line_does_not_require_old_sensor_bay_offset(self):
        p = copy.deepcopy(self.project)
        p['cable']['length_m'] = 0.1
        p['objects']['sensor']['position'] = (np.array(p['objects']['winch']['position']) + [0, 0, 0.1]).tolist()
        actual = prepare(validate_project(p), self.settings)
        self.assertAlmostEqual(actual['config']['winch']['stowed_length_m'], 0.005)


if __name__ == '__main__':
    unittest.main()
