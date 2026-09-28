import test from 'node:test';
import assert from 'node:assert/strict';
import {tableOptions,workerOptions,modelChoice} from '../analysis-options.js';

const catalog={defaults:{model_id:'generated-model',aero_job:'new'},tables:[{id:'avl-table',backend:'avl',speed:20},{id:'flow5-table',backend:'flow5',speed:20}]};
test('a missing saved table is retained as an error choice, never replaced by new',()=>{
  const result=tableOptions(catalog,'avl','missing-id');
  assert.equal(result.value,'missing-id');assert.equal(result.missing,true);
  assert.equal(result.rows.find(r=>r.id==='missing-id').missing,true);
  assert.equal(tableOptions(catalog,'avl','avl-table').missing,false);
  assert.equal(tableOptions(catalog,'flow5','avl-table').missing,true);
});
test('explicit new calculation remains possible',()=>{
  assert.equal(tableOptions(catalog,'avl','new').value,'new');
  assert.equal(tableOptions(catalog,'avl','new').missing,false);
});
test('hybrid tables are filtered by backend without substituting a saved choice',()=>{
 const value={...catalog,tables:[...catalog.tables,{id:'combined-table',backend:'hybrid',speed:20}]};
 assert.deepEqual(tableOptions(value,'hybrid','new').rows.map(r=>r.id),['new','combined-table']);
 assert.equal(tableOptions(value,'hybrid','avl-table').missing,true);
});
test('CPU options include non-power-of-two defaults and preserve unsupported saved requests visibly',()=>{
  for(const maximum of [1,3,5,6,7,9,10,11,12]){
    const result=workerOptions(maximum,maximum);
    assert.equal(result.missing,false);assert.ok(result.rows.some(r=>r.value===maximum));
    assert.equal(workerOptions(maximum,maximum+1).missing,true);
  }
});

import {readFileSync} from 'node:fs';
test('empty registry and obsolete saved model need registration, never automatic substitution',()=>{
 const saved=JSON.parse(readFileSync(new URL('./fixtures/legacy-selection.json',import.meta.url),'utf8'));
 assert.equal(modelChoice([],null).id,null);
 assert.equal(modelChoice([{id:'registered'}],saved).id,null);
 assert.match(modelChoice([{id:'registered'}],saved).message,/배치는 복원/);
 assert.equal(modelChoice([{id:'registered'}],null).id,'registered');
});
