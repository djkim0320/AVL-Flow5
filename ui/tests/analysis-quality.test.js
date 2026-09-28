import test from 'node:test';
import assert from 'node:assert/strict';
import { resultQuality, hybridQuality } from '../analysis-quality.js';
test('old no-unstable result cannot imply a verified stable aircraft', () => {
  const q = resultQuality({ unstable: false, numerically_converged: false });
  assert.match(q.stability, /추가 확인/);
  assert.match(q.convergence, /미확인/);
});
test('near neutral, growing and damped spectra remain distinct', () => {
  assert.match(resultQuality({ stability_status: 'near_neutral' }).stability, /판정 보류/);
  assert.match(resultQuality({ stability_status: 'unstable_detected' }).stability, /불안정/);
  assert.match(resultQuality({ stability_status: 'damped' }).stability, /실물 검증 전/);
});

test('hybrid source and discrepancy warning are visible without claiming accuracy', () => {
  const q = hybridQuality({
    backend: 'hybrid',
    composition: { coeff: 'flow5', controls: 'flow5', rates: 'avl' },
    flow5_speed_m_s: 20,
    consistency: { Cm_alpha_per_deg: { avl: -0.1, flow5: -0.15, relative_difference: 1 / 3, warning: true } }
  });
  assert.match(q.source, /회전율: avl/);
  assert.match(q.note, /20%/);
  assert.equal(q.rows[0].difference, '33.3%');
  assert.equal(hybridQuality({ backend: 'avl' }), null);
});
