import test from 'node:test';
import assert from 'node:assert/strict';
import {uniformSamples,checkedSamples} from '../aero-ranges.js';
import {currentScenario} from '../analysis-scenarios.js';

test('slider grid contains endpoints and evenly spaced interior points',()=>{
  assert.deepEqual(uniformSamples(-4,12,5),[-4,0,4,8,12]);
  assert.deepEqual(uniformSamples(-5,5,3),[-5,0,5]);
  const samples=uniformSamples(-1.3,4.7,8);
  assert.equal(samples.length,8);assert.equal(samples[0],-1.3);assert.equal(samples.at(-1),4.7);
  for(let i=1;i<samples.length;i++)assert.ok(Math.abs(samples[i]-samples[i-1]-6/7)<1e-10);
  for(const args of [[3,3,5],[4,-4,5],[-4,12,1],[-4,12,2.5],[NaN,12,5]])assert.throws(()=>uniformSamples(...args));
});
test('legacy nonuniform samples survive without inferred replacement',()=>{
  const original=[-4,-1.5,0,3,12],loaded=checkedSamples(original);
  assert.deepEqual(loaded,original);loaded[0]=-5;assert.equal(original[0],-4);
  for(const values of [[1],[1,1],[2,1],[NaN,3],null])assert.throws(()=>checkedSamples(values));
});
test('old analysis choices migrate to continuous sequence without changing physical inputs',()=>{
  for(const task of ['aero','trim','stability','response','flight','mission','recovery']){
    const previous={task,phase:'aircraft_only',start:'scene',speed:25,workers:12};
    assert.deepEqual(currentScenario(previous),{...previous,sequence_ui_version:1,task:'sequence',phase:'stowed',start:'equilibrium'});
    assert.equal(previous.phase,'aircraft_only');
  }
  assert.equal(currentScenario({task:'recovery',sequence_ui_version:1}).task,'recovery');
  assert.equal(currentScenario({task:'flight',sequence_ui_version:1}).task,'flight');
  assert.equal('hold' in currentScenario({task:'flight',hold:null,payout:null}),false);
});
