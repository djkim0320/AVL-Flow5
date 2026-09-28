import test from 'node:test';
import assert from 'node:assert/strict';
import {Matrix4,Quaternion,Vector3} from 'three';
import {appendComponents,moveComponent,removeComponent,reviseDefinition,validateComponents,importBatch} from '../aircraft-assembly.js';
import {newDefinition,reorientAsset} from '../aircraft-model.js';

const file=(name='wing.step',x=0)=>({source:{name,origin:'file_origin'},parts:[{name:'solid',positions:[x,0,0,x+2,0,0,x,2,0],indices:[0,1,2]}]});
const assembly=()=>appendComponents(null,[file(),file('elevator.step',10)]);
const near=(a,b)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<1e-11,`${a} != ${b}`));

test('append preserves CAD origins and existing geometry; names retain file identity',()=>{
 const old=file(),a=appendComponents(old,[file('elevator.step',10)]);
 assert.equal(a.parts[0],old.parts[0]);assert.deepEqual(a.parts[1].positions,[10,0,0,12,0,0,10,2,0]);
 assert.equal(a.parts[1].name,'elevator.step / solid');assert.deepEqual(a.components.map(c=>c.parts),[[0],[1]]);
 assert.equal(old.components,undefined);validateComponents(a);
});

test('component rigid motion is baked once and leaves other files and undo source unchanged',()=>{
 const a=assembly(),before=structuredClone(a),c=a.components[1],q=[0,0,Math.SQRT1_2,Math.SQRT1_2];
 const moved=moveComponent(a,c.id,[21,1,0],q).asset;
 near(moved.parts[1].positions,[22,0,0,22,2,0,20,0,0]);assert.equal(moved.parts[0],a.parts[0]);assert.deepEqual(a,before);
 // Rendering uses inverse(handle) baked positions followed by handle, not a second transform.
 const pose=new Matrix4().compose(new Vector3(...moved.components[1].position),new Quaternion(...q),new Vector3(1,1,1));
 near(new Vector3(...moved.parts[1].positions.slice(0,3)).applyMatrix4(pose.clone().invert()).applyMatrix4(pose).toArray(),[22,0,0]);
 const restored=moveComponent(moved,c.id,c.position,c.quaternion).asset;near(restored.parts[1].positions,a.parts[1].positions);
});

test('locked files cannot move and corrupt memberships and poses fail explicitly',()=>{
 const a=assembly(),id=a.components[1].id;a.components[1].locked=true;
 assert.throws(()=>moveComponent(a,id,[1,2,3],[0,0,0,1]),/고정/);
 for(const edit of [v=>v.components[1].parts=[0],v=>v.components[1].parts=[2],v=>v.components[1].quaternion=[0,0,0,2],v=>v.components.pop(),v=>v.components[1].position[0]=NaN]){
  const b=assembly();edit(b);assert.throws(()=>validateComponents(b));
 }
});

test('removing a file remaps surviving parts, surfaces and definition roles',()=>{
 const a=assembly(),d=newDefinition(a);d.parts[1].role='control';d.surfaces=[{name:'tail',parts:[1],sections:[{test:true}]}];d.mass.mass_kg=2;d.reviewed=true;
 const r=removeComponent(a,a.components[0].id),next=reviseDefinition(d,r.asset,{...r,topology:true});
 assert.deepEqual(r.asset.components[0].parts,[0]);assert.equal(next.parts[0].role,'control');assert.equal(next.parts[0].index,0);
 assert.deepEqual(next.surfaces[0].parts,[0]);assert.deepEqual(next.surfaces[0].sections,[{test:true}]);assert.equal(next.mass.mass_kg,null);assert.equal(next.reviewed,false);
 assert.equal(removeComponent(r.asset,r.asset.components[0].id).asset,null);
});

test('component CG and inertia follow the motion; affected aero sections require rebuilding',()=>{
 const a=assembly(),d=newDefinition(a),c=a.components[1];d.mass.mode='components';
 d.parts[1].cg_m=[11,1,0];d.parts[1].inertia_kgm2=[[1,0,0],[0,2,0],[0,0,3]];
 d.surfaces=[{parts:[0],sections:[{unchanged:true}]},{parts:[1],sections:[{stale:true}]}];d.reviewed=true;
 const r=moveComponent(a,c.id,[21,1,0],[0,0,Math.SQRT1_2,Math.SQRT1_2]),n=reviseDefinition(d,r.asset,r);
 near(n.parts[1].cg_m,[21,1,0]);near(n.parts[1].inertia_kgm2.flat(),[2,0,0,0,1,0,0,0,3]);
 assert.deepEqual(n.surfaces[0].sections,[{unchanged:true}]);assert.deepEqual(n.surfaces[1].sections,[]);assert.equal(n.reviewed,false);
 assert.deepEqual(d.parts[1].cg_m,[11,1,0]);
});

test('total physical values become unknown after geometry edits, never silently reused',()=>{
 const a=assembly(),d=newDefinition(a);d.mass={mode:'total',mass_kg:2,cg_m:[0,0,0],inertia_kgm2:[[1,0,0],[0,1,0],[0,0,1]]};
 const n=reviseDefinition(d,a);assert.equal(n.mass.mass_kg,2);assert.deepEqual(n.mass.cg_m,[null,null,null]);assert.equal(n.mass.inertia_kgm2[0][0],null);
 const appended=appendComponents(a,[file('fin.step')]),added=reviseDefinition(d,appended,{topology:true});
 assert.equal(added.parts[2].role,'unassigned');assert.equal(added.parts[2].mass_kg,null);assert.equal(added.mass.mass_kg,null);
});

test('whole-aircraft frame correction preserves component handles and applies to subsequent imports',()=>{
 const a=assembly(),rotated=reorientAsset(a,'z',1),c=rotated.components[1];near(c.position,[-1,11,0]);
 near(new Vector3(1,0,0).applyQuaternion(new Quaternion(...c.quaternion)).toArray(),[0,1,0]);
 const appended=appendComponents(rotated,[file('new.step',10)]);near(appended.parts[2].positions,rotated.parts[1].positions);
 const back=reorientAsset(rotated,'z',-1);near(back.components[1].position,a.components[1].position);near(back.parts[1].positions,a.parts[1].positions);
});

test('parallel conversion preserves order, is bounded, and waits for all errors before rejecting the batch',async()=>{
 let active=0,peak=0,done=[];
 const read=async f=>{peak=Math.max(peak,++active);await new Promise(r=>setTimeout(r,f.delay));active--;done.push(f.name);if(f.bad)throw Error('invalid STEP');return f.name;};
 const files=[{name:'a',delay:20},{name:'b',delay:1},{name:'c',delay:1}];
 assert.deepEqual(await importBatch(files,read),['a','b','c']);assert.equal(peak,2);
 done=[];await assert.rejects(importBatch([{...files[0],bad:true},files[1],files[2]],read),/invalid STEP/);assert.equal(done.length,3);assert.equal(active,0);
});
