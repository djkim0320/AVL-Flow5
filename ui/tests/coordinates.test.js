import test from 'node:test';
import assert from 'node:assert/strict';
import { Group } from 'three';
import { finiteInput, importUnit, editPoseComponent } from '../coordinates.js';

test('STEP kernel millimetres are not scaled as metres', () => {
  assert.equal(importUnit('step', 'm'), 'mm');
  assert.equal(importUnit('stp', 'cm'), 'mm');
  assert.equal(importUnit('stl', 'm'), 'm');
  assert.equal(importUnit('glb', 'm'), 'm');
});
test('empty and nonfinite coordinates are rejected', () => {
  for (const raw of ['', ' ', 'NaN', 'Infinity']) assert.throws(() => finiteInput(raw));
  assert.equal(finiteInput('-1.25'), -1.25);
});
test('editing one position keeps every other coordinate and rotation exact', () => {
  const g = new Group();
  g.position.set(0.123456789, -0.2222222, 0.3333333);
  g.rotation.set(0.1234567, 0.8765432, -0.7654321);
  const q = g.quaternion.toArray();
  editPoseComponent(g, 'py', '85');
  assert.equal(g.position.x, 0.123456789);
  assert.equal(g.position.z, 0.3333333);
  assert.equal(g.position.y, 0.085);
  assert.deepEqual(g.quaternion.toArray(), q);
});
test('editing one rotation keeps unedited full-precision angles and position', () => {
  const g = new Group();
  g.position.set(0.123456789, 2, 3);
  g.rotation.set(0.1234567, 0.3456789, 0.7654321);
  editPoseComponent(g, 'rx', '30');
  assert.ok(Math.abs(g.rotation.y - 0.3456789) < 1e-14);
  assert.ok(Math.abs(g.rotation.z - 0.7654321) < 1e-14);
  assert.equal(g.position.x, 0.123456789);
});
