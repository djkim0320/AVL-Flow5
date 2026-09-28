// Mesh coordinates are always baked in aircraft-local SI/FRD. Component poses
// are editing handles only; solvers, signatures and exports see the same mesh.
import {Matrix4,Quaternion,Vector3} from 'three';
import {newDefinition,blankTensor,partBounds} from './aircraft-model.js';

const uuid=()=>crypto.randomUUID();
const poseMatrix=c=>new Matrix4().compose(new Vector3(...c.position),new Quaternion(...c.quaternion),new Vector3(1,1,1));
export function validateComponents(asset){
  if(asset.components===undefined)return;
  if(!Array.isArray(asset.components)||!asset.components.length)throw new Error('조립체 파일 목록이 비어 있습니다.');
  const used=new Set(),ids=new Set();
  for(const c of asset.components){
    if(typeof c.id!=='string'||!c.id||ids.has(c.id)||typeof c.name!=='string'||!c.name||!c.source||typeof c.source.origin!=='string'||typeof c.locked!=='boolean')throw new Error('조립체 파일 정보가 올바르지 않습니다.');
    ids.add(c.id);
    if(!Array.isArray(c.position)||c.position.length!==3||!c.position.every(Number.isFinite)||!Array.isArray(c.quaternion)||c.quaternion.length!==4||!c.quaternion.every(Number.isFinite)||Math.abs(Math.hypot(...c.quaternion)-1)>1e-5)throw new Error('부품 위치·회전이 올바르지 않습니다.');
    if(!Array.isArray(c.parts)||!c.parts.length)throw new Error('파일에 속한 부품이 없습니다.');
    for(const i of c.parts){if(!Number.isInteger(i)||i<0||i>=asset.parts.length||used.has(i))throw new Error('조립체 부품 번호가 중복되거나 잘못됐습니다.');used.add(i);}
  }
  if(used.size!==asset.parts.length)throw new Error('파일에 연결되지 않은 조립체 부품이 있습니다.');
}
export function componentsOf(asset){
  if(asset.components){validateComponents(asset);return asset.components;}
  return [{id:uuid(),name:asset.source.name||'기체 파일',source:structuredClone(asset.source),
    parts:asset.parts.map((_,i)=>i),position:partBounds(asset,asset.parts.map((_,i)=>i)).center,
    quaternion:[0,0,0,1],locked:false}];
}
function finish(asset,parts,components){
  const {id,...rest}=asset;
  const result={...rest,parts,components,source:{...asset.source,origin:'assembly_origin',
    name:components.length===1?components[0].name:`${components[0].name} 외 ${components.length-1}개 파일`}};
  validateComponents(result);return result;
}
export function appendComponents(base,imports){
  if(!imports.length)throw new Error('추가할 파일을 선택하세요.');
  let parts=base?[...base.parts]:[],components=base?[...componentsOf(base)]:[];
  // New files use the assembly's established forward-axis correction.
  const correction=base?.source.orientation_matrix;
  for(const asset of imports){
    const start=parts.length;
    const incoming=asset.parts.map(p=>({...p,name:`${asset.source.name} / ${p.name}`,positions:correction?p.positions.map((_,i,a)=>{const j=i-i%3,r=correction[i%3];return r[0]*a[j]+r[1]*a[j+1]+r[2]*a[j+2];}):p.positions}));
    parts.push(...incoming);
    const indices=incoming.map((_,i)=>start+i);
    components.push({id:uuid(),name:asset.source.name,source:structuredClone(asset.source),parts:indices,
      position:partBounds({parts},indices).center,quaternion:[0,0,0,1],locked:false});
  }
  return finish(base||{source:{units:'m',axes:'FRD',origin:'assembly_origin'}},parts,components);
}
export function moveComponent(asset,id,position,quaternion){
  validateComponents(asset);
  const c=asset.components.find(c=>c.id===id);if(!c)throw new Error('선택한 파일을 찾을 수 없습니다.');
  if(c.locked)throw new Error('고정된 파일은 위치 고정을 해제한 뒤 이동하세요.');
  const next={...c,position:[...position],quaternion:[...quaternion]};
  const components=asset.components.map(c=>c.id===id?next:c);
  validateComponents({...asset,components});
  const delta=poseMatrix(next).multiply(poseMatrix(c).invert()),set=new Set(c.parts),v=new Vector3();
  const parts=asset.parts.map((p,i)=>{if(!set.has(i))return p;const positions=[...p.positions];for(let j=0;j<positions.length;j+=3){v.fromArray(p.positions,j).applyMatrix4(delta).toArray(positions,j);}return {...p,positions};});
  return {asset:finish(asset,parts,components),delta,affected:c.parts};
}
export function removeComponent(asset,id){
  const c=asset.components.find(c=>c.id===id);if(!c)throw new Error('선택한 파일을 찾을 수 없습니다.');
  const removed=new Set(c.parts),map=new Map(),parts=[];
  asset.parts.forEach((p,i)=>{if(!removed.has(i)){map.set(i,parts.length);parts.push(p);}});
  const components=asset.components.filter(c=>c.id!==id).map(c=>({...c,parts:c.parts.map(i=>map.get(i))}));
  return {asset:parts.length?finish(asset,parts,components):null,map,affected:c.parts};
}
export function reviseDefinition(definition,asset,{map=null,affected=[],delta=null,topology=false}={}){
  if(!definition||!asset)return null;
  const fresh=newDefinition(asset),d=structuredClone(definition),set=new Set(affected);
  map??=new Map(d.parts.map((_,i)=>[i,i]));
  for(const old of d.parts){const i=map.get(old.index);if(i===undefined)continue;fresh.parts[i]={...old,index:i};
    if(delta&&set.has(old.index)){
      const part=fresh.parts[i];
      if(part.cg_m?.every(Number.isFinite))part.cg_m=new Vector3(...part.cg_m).applyMatrix4(delta).toArray();
      if(part.inertia_kgm2?.flat().every(Number.isFinite)){
        const e=delta.elements,r=[[e[0],e[4],e[8]],[e[1],e[5],e[9]],[e[2],e[6],e[10]]],j=part.inertia_kgm2;
        part.inertia_kgm2=r.map(a=>r.map(b=>a.reduce((sum,x,k)=>sum+x*b.reduce((s,y,l)=>s+j[k][l]*y,0),0)));
      }
    }
  }
  d.parts=fresh.parts;
  d.surfaces=d.surfaces.map(s=>{const changed=s.parts.some(i=>set.has(i)||!map.has(i));return {...s,parts:s.parts.filter(i=>map.has(i)).map(i=>map.get(i)),sections:changed?[]:s.sections};}).filter(s=>s.parts.length);
  if(d.mass.mode==='total'){
    if(topology)d.mass.mass_kg=null;
    d.mass.cg_m=[null,null,null];d.mass.inertia_kgm2=blankTensor();
  }
  d.reviewed=false;
  return d;
}

export async function importBatch(files,read,workers=2){
  const results=new Array(files.length),failures=[];let cursor=0;
  await Promise.all(Array.from({length:Math.min(workers,files.length)},async()=>{
    while(cursor<files.length){const i=cursor++;try{results[i]=await read(files[i]);}catch(e){failures.push(`${files[i].name}: ${e.message}`);}}
  }));
  if(failures.length)throw new Error('파일을 추가하지 않았습니다. '+failures.join(' / '));
  return results;
}
