import copy, json, tempfile, uuid
from pathlib import Path
from tests.studio.fixtures.aircraft import RegisteredCase
from dbf_studio import model_registry, analysis_bridge
from dbf_studio.analysis_bridge import prepare, catalog
from dbf_stability.hybrid import DEFAULT_COMPOSITION


class HybridBridgeChecks(RegisteredCase):
    def setUp(self):
        self.s = copy.deepcopy(self.settings)
        self.s.update(backend='hybrid', aero_hybrid=copy.deepcopy(DEFAULT_COMPOSITION))

    def test_default_hybrid_does_not_require_unused_flow_rate_closure(self):
        result = prepare(self.project, self.s)
        self.assertEqual(result['config']['aero']['hybrid'], DEFAULT_COMPOSITION)
        self.assertNotIn('rate_derivative_closure', result['config']['aero'])
        self.assertIn('hybrid', catalog(self.model_id)['databases'])

    def test_reject_flow_rates_without_confirmation_and_all_same(self):
        for composition in (
            dict(coeff='avl', controls='flow5', rates='flow5'),
            dict(coeff='avl', controls='avl', rates='avl'),
        ):
            with self.assertRaises(ValueError):
                prepare(self.project, {**self.s, 'aero_hybrid': composition})

    def test_elevator_limit_and_missing_executable(self):
        grid = copy.deepcopy(self.s['aero_grid'])
        grid['elevator_deg'] = [-11, 0, 8]
        with self.assertRaisesRegex(ValueError, '10'):
            prepare(self.project, {**self.s, 'aero_grid': grid})
        path = model_registry.DIRECTORY / self.model_id / 'model.json'
        original = path.read_text('utf8')
        try:
            for key in ('executable', 'flow5_executable'):
                model = json.loads(original)
                model['config']['aero'][key] = str(model_registry.DIRECTORY / 'not-installed.exe')
                path.write_text(json.dumps(model), encoding='utf8')
                with self.assertRaisesRegex(ValueError, '실행파일'):
                    prepare(self.project, self.s)
        finally:
            path.write_text(original, encoding='utf8')

    def test_cached_flow_speed_comes_from_request_not_a_constant(self):
        old = analysis_bridge.ANALYSIS_DIRECTORY
        with tempfile.TemporaryDirectory() as tmp:
            try:
                analysis_bridge.ANALYSIS_DIRECTORY = Path(tmp)
                run = Path(tmp) / uuid.uuid4().hex
                run.mkdir()
                request = {'settings': {**self.s, 'speed': 17.0}}
                (run / 'request.json').write_text(json.dumps(request), encoding='utf8')
                # Speed validation precedes even opening the table: no fabricated npz.
                with self.assertRaisesRegex(ValueError, '속도'):
                    prepare(self.project, {**self.s, 'aero_job': run.name, 'speed': 21.0, 'rebuild': False})
            finally:
                analysis_bridge.ANALYSIS_DIRECTORY = old
