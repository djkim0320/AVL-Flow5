import test from 'node:test';
import assert from 'node:assert/strict';
import {restoreController} from '../controller-settings.js';

test('old disabled-only project gains visible settings without enabling control',()=>{
  const defaults={enabled:false,type:'longitudinal_pd',pitch_kp:.8,elevator_limits_deg:[-8,8]};
  const actual=restoreController({enabled:false},defaults);
  assert.deepEqual(actual,defaults);actual.elevator_limits_deg[0]=-1;
  assert.equal(defaults.elevator_limits_deg[0],-8);
});
test('explicit tuning and malformed input are preserved for validation',()=>{
  for(const value of [{pitch_kp:0,type:'bad'},{enabled:true,pitch_kp:.3},null,[]])
    assert.deepEqual(restoreController(value,{pitch_kp:.8}),value);
});
