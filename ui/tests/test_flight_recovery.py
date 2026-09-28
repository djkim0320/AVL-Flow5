"""Scenario schedules derived from analytic fixture inputs."""
import copy
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import validate_project
from analysis_bridge import catalog, prepare, worker_limit
from dbf_stability import AeroDatabase, solve_trim, simulate
from dbf_stability.model import CoupledModel


from fixtures.aircraft import RegisteredCase

class FlightRecoveryChecks(RegisteredCase):

    def prepared(self,**changes):
        return prepare(self.project,{**self.settings,**changes})

    def test_flight_uses_full_length_with_fixed_imported_door(self):
        r=self.prepared();c=r['config']
        self.assertEqual(r['phase'],'deployed')
        self.assertEqual(c['winch']['length_schedule'],[[0.,2.4],[.1,2.4]])
        self.assertEqual({a for _,a in c['winch']['door_schedule']},{r['profile']['door_reference_deg']})
        self.assertFalse(c['winch']['door_capture_interlock'])

    def test_recovery_ignores_legacy_payout_and_hold(self):
        a=self.prepared(task='recovery',recovery_length=1.8,recovery=1.)
        b=self.prepared(task='recovery',recovery_length=1.8,recovery=1.,payout=.1,hold=100,duration=200,start='scene')
        self.assertEqual(a['phase'],'recovery')
        self.assertEqual(a['config']['winch'],b['config']['winch'])
        self.assertAlmostEqual(a['duration_s'],.6)
        rows=np.array(a['config']['winch']['length_schedule'])
        self.assertEqual(rows[0,1],2.4);self.assertEqual(rows[-1,1],1.8)
        self.assertTrue(np.all(np.diff(rows[:,1])<0))

    def test_invalid_recovery_end_rejected(self):
        for length in (0,2.4,3.,float('nan')):
            with self.subTest(length=length),self.assertRaises(ValueError):self.prepared(task='recovery',recovery_length=length)

    def test_only_visible_scenario_fields_control_the_schedule(self):
        flight=self.prepared(recovery=None,recovery_length=None,hold=None,payout=None)
        self.assertEqual(flight['duration_s'],.1)
        recovery=self.prepared(task='recovery',duration=None,pitch_delta=None,hold=None,payout=None,recovery_length=1.8,recovery=1.)
        self.assertAlmostEqual(recovery['duration_s'],.6)
        with self.assertRaisesRegex(ValueError,'recovery:'):
            self.prepared(task='recovery',recovery=None)
        with self.assertRaisesRegex(ValueError,'duration:'):
            self.prepared(duration=None)

    def test_cpu_budget_reserves_four_where_available(self):
        for cpus,expected in [(4,1),(7,3),(10,6),(14,10),(16,12),(32,12)]:
            with self.subTest(cpus=cpus):self.assertEqual(worker_limit(cpus),expected)



if __name__=='__main__':unittest.main()
