import test from 'node:test';
import assert from 'node:assert/strict';
import { resultSelection } from '../result-selection.js';

function setup() {
  const waiting = [],
    visible = [],
    errors = [],
    pending = [];
  const selection = resultSelection({
    read: id => new Promise((resolve, reject) => waiting.push({ id, resolve, reject })),
    render: value => visible.push(value),
    error: e => errors.push(e.message),
    pending: id => pending.push(id)
  });
  return { selection, waiting, visible, errors, pending };
}

test('a slow prior selection cannot replace the most recently chosen run', async () => {
  const s = setup(),
    first = s.selection.select('old'),
    second = s.selection.select('new');
  s.waiting[1].resolve({ id: 'new', state: 'completed' });
  await second;
  s.waiting[0].resolve({ id: 'old', state: 'running' });
  await first;
  assert.deepEqual(s.visible, [{ id: 'new', state: 'completed' }]);
  assert.equal(s.selection.id, 'new');
});

test('a late poll error cannot overwrite a new selection or restart old polling', async () => {
  const s = setup();
  await s.selection.select('old', { id: 'old', state: 'running' });
  const poll = s.selection.refresh();
  await s.selection.select('new', { id: 'new', state: 'completed' });
  s.waiting[0].reject(Error('old run disconnected'));
  await poll;
  assert.deepEqual(s.errors, []);
  assert.equal(s.visible.at(-1).id, 'new');
});

test('returning to the same run still rejects a poll from an earlier selection', async () => {
  const s = setup();
  await s.selection.select('A', { id: 'A', state: 'running' });
  const old = s.selection.refresh();
  const cancellationTicket = s.selection.capture();
  await s.selection.select('B', { id: 'B' });
  await s.selection.select('A', { id: 'A', state: 'completed' });
  s.waiting[0].resolve({ id: 'A', state: 'running' });
  await old;
  assert.equal(cancellationTicket(), false);
  assert.equal(s.visible.at(-1).state, 'completed');
});

test('current lookup errors are visible and the current run can be retried', async () => {
  const s = setup(),
    first = s.selection.select('A');
  s.waiting[0].reject(Error('connection'));
  await first;
  assert.deepEqual(s.errors, ['connection']);
  const retry = s.selection.refresh();
  s.waiting[1].resolve({ id: 'A', state: 'completed' });
  await retry;
  assert.equal(s.visible.at(-1).state, 'completed');
});
