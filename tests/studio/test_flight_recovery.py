"""Scenario schedules derived from analytic fixture inputs."""

import copy
import unittest
import numpy as np

from dbf_studio.server import validate_project
from dbf_studio.analysis_bridge import prepare, worker_limit


from tests.studio.fixtures.aircraft import RegisteredCase


class FlightRecoveryChecks(RegisteredCase):
    def prepared(self, **changes):
        return prepare(self.project, {**self.settings, **changes})

    def test_flight_uses_full_length_with_fixed_imported_door(self):
        r = self.prepared()
        c = r['config']
        self.assertEqual(r['phase'], 'deployed')
        self.assertEqual(c['winch']['length_schedule'], [[0.0, 2.4], [0.1, 2.4]])
        self.assertEqual({a for _, a in c['winch']['door_schedule']}, {r['profile']['door_reference_deg']})
        self.assertFalse(c['winch']['door_capture_interlock'])

    def test_recovery_ignores_legacy_payout_and_hold(self):
        a = self.prepared(task='recovery', recovery_length=1.8, recovery=1.0)
        b = self.prepared(
            task='recovery', recovery_length=1.8, recovery=1.0, payout=0.1, hold=100, duration=200, start='scene'
        )
        self.assertEqual(a['phase'], 'recovery')
        self.assertEqual(a['config']['winch'], b['config']['winch'])
        self.assertAlmostEqual(a['duration_s'], 0.6)
        rows = np.array(a['config']['winch']['length_schedule'])
        self.assertEqual(rows[0, 1], 2.4)
        self.assertEqual(rows[-1, 1], 1.8)
        self.assertTrue(np.all(np.diff(rows[:, 1]) < 0))

    def test_invalid_recovery_end_rejected(self):
        for length in (0, 2.4, 3.0, float('nan')):
            with self.subTest(length=length), self.assertRaises(ValueError):
                self.prepared(task='recovery', recovery_length=length)

    def test_only_visible_scenario_fields_control_the_schedule(self):
        flight = self.prepared(recovery=None, recovery_length=None, hold=None, payout=None)
        self.assertEqual(flight['duration_s'], 0.1)
        recovery = self.prepared(
            task='recovery', duration=None, pitch_delta=None, hold=None, payout=None, recovery_length=1.8, recovery=1.0
        )
        self.assertAlmostEqual(recovery['duration_s'], 0.6)
        with self.assertRaisesRegex(ValueError, 'recovery:'):
            self.prepared(task='recovery', recovery=None)
        with self.assertRaisesRegex(ValueError, 'duration:'):
            self.prepared(duration=None)

    def test_cpu_budget_reserves_four_where_available(self):
        for cpus, expected in [(4, 1), (7, 3), (10, 6), (14, 10), (16, 12), (32, 12)]:
            with self.subTest(cpus=cpus):
                self.assertEqual(worker_limit(cpus), expected)

    def test_sequence_has_four_ordered_phases_and_fixed_door(self):
        r = self.prepared(
            task='sequence',
            preflight=2.0,
            payout=1.0,
            hold=60.0,
            recovery=1.0,
            recovery_length=0.1,
            duration=None,
            pitch_delta=None,
        )
        c = r['config']
        rows = np.array(c['winch']['length_schedule'])
        schedule = r['schedule']
        self.assertEqual(r['phase'], 'mission')
        self.assertEqual(rows[0, 1], 0.02)
        self.assertEqual(rows[-1, 1], 0.1)
        self.assertTrue(np.all(np.diff(rows[:, 0]) > 0))
        self.assertAlmostEqual(schedule['deployed_s'], 4.38)
        self.assertAlmostEqual(schedule['recovery_s'], 64.38)
        self.assertAlmostEqual(r['duration_s'], 66.68)
        self.assertEqual(c['winch']['release_s'], 2.0)
        self.assertFalse(c['winch']['capture_enabled'])
        self.assertEqual({v for _, v in c['winch']['door_schedule']}, {r['profile']['door_reference_deg']})
        for times, sign in [((2, 4.38), 1), ((64.38, 66.68), -1)]:
            selection = rows[(rows[:, 0] >= times[0] - 1e-9) & (rows[:, 0] <= times[1] + 1e-9)]
            self.assertTrue(np.all(sign * np.diff(selection[:, 1]) > 0))

    def test_sequence_zero_hold_and_invalid_inputs(self):
        r = self.prepared(task='sequence', hold=0.0)
        self.assertEqual(r['schedule']['deployed_s'], r['schedule']['recovery_s'])
        self.assertTrue(np.all(np.diff(np.array(r['config']['winch']['length_schedule'])[:, 0]) > 0))
        for changes in [
            dict(preflight=0),
            dict(preflight=None),
            dict(payout=0),
            dict(hold=-1),
            dict(recovery_length=2.4),
            dict(hold=600),
            dict(recovery_length=0.001),
        ]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.prepared(task='sequence', **changes)

    def test_stowed_editor_position_is_accepted(self):
        project = copy.deepcopy(self.project)
        project['objects']['sensor']['position'] = (
            np.array(project['objects']['winch']['position']) + [0, 0, 0.02]
        ).tolist()
        validate_project(project)
        self.assertEqual(
            prepare(project, {**self.settings, 'task': 'sequence'})['config']['winch']['stowed_length_m'], 0.02
        )


if __name__ == '__main__':
    unittest.main()
