import test from 'node:test';
import assert from 'node:assert/strict';
import {pointAsset,previewPosition,migrateProject} from '../point-mass.js';

test('point payload contains mass, never a sensor mesh',()=>{
  assert.equal(pointAsset(.12).mass_kg,.12);assert.equal('parts' in pointAsset(.12),false);
  for(const invalid of [0,-1,NaN,Infinity,11])assert.throws(()=>pointAsset(invalid));
});
test('length preview follows the aircraft vertical axis and winch',()=>{
  assert.deepEqual(previewPosition([1,2,3],[0,0,0,1],2.7),[1,2,5.7]);
  const p=previewPosition([1,2,3],[0,1,0,0],2);assert.deepEqual(p,[1,2,1]);
  assert.throws(()=>previewPosition([0,0,0],[0,0,0,1],0));
});
test('old sensor mesh is removed, mass preserved, source untouched',()=>{
  const old={schema:'dbf-assembly/1',objects:{sensor:{parts:[{name:'old'}]},winch:{position:[0,0,0]}},cable:{length_m:2},analysis:{sensor_mass:.2}};
  const out=migrateProject(old);assert.equal(out.schema,'dbf-assembly/2');assert.equal(out.objects.sensor.mass_kg,.2);
  assert.equal('parts' in out.objects.sensor,false);assert.deepEqual(out.cable.attachment,{kind:'point_mass',local_point_m:[0,0,0]});
  assert.equal(old.objects.sensor.parts[0].name,'old');
  delete old.analysis;assert.equal(migrateProject(old).objects.sensor,null);
});
