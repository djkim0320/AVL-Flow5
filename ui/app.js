import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import {importUnit,finiteInput,editPoseComponent} from './coordinates.js';
import {pointAsset,previewPosition,migrateProject} from './point-mass.js';
import {initAnalysis} from './analysis-ui.js';
import {initAircraftDefinition} from './aircraft-definition-ui.js';
import {localFromWorld,worldFromLocal,onCenterline,quarterTurn,applyMatrix,reorientAsset,isBlankDefinition} from './aircraft-model.js';

const $ = id => document.getElementById(id);
const names = {aircraft:'기체',sensor:'질점',winch:'윈치'};
const colors = {aircraft:'#b3c9d2',sensor:'#d69c43',winch:'#577b8c'};
const objects = {aircraft:null,sensor:null,winch:null};
let selected=null, pendingRole=null, drag=null, lastPointer=null;
let cable={diameter_m:.001,length_m:2.7,attachment:null};
let undoStack=[], redoStack=[], busy=false, savingTimer=null, draftDB=null, dirty=false;
let analysisUI=null;
let aircraftDefinition=null,definitionUI=null,cgCallback=null;
let recentModelFile=null;
const viewport=$('viewport'), ray=new THREE.Raycaster(), mouse=new THREE.Vector2();
// The canvas is transparent; the viewport's CSS gradient (theme.css) is the scene background.
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(38,1,.001,10000);camera.up.set(0,0,-1);camera.position.set(-2.1,-2,-1.1);
const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setClearColor(0x000000,0);
renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.domElement.setAttribute('aria-hidden','true');viewport.prepend(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.target.set(-.2,0,0);orbit.enableDamping=false;
orbit.mouseButtons={LEFT:THREE.MOUSE.ROTATE,MIDDLE:THREE.MOUSE.PAN,RIGHT:THREE.MOUSE.PAN};
orbit.screenSpacePanning=true;
// Middle-button dragging moves the camera target, not browser auto-scroll.
renderer.domElement.addEventListener('mousedown',event=>{if(event.button===1)event.preventDefault();});
const transform=new TransformControls(camera,renderer.domElement);transform.setSize(.82);scene.add(transform.getHelper());
scene.add(new THREE.HemisphereLight(0xffffff,0x60788e,2.3));
for (const [position,intensity] of [[[2,-3,-5],3],[[-3,2,-1],1.4]]) { const l=new THREE.DirectionalLight(0xffffff,intensity);l.position.fromArray(position);scene.add(l); }
// 10 cm minor / 1 m major floor grid, recoloured from CSS tokens when the theme changes.
const grid=new THREE.Group();grid.rotation.x=Math.PI/2;grid.position.z=.35;scene.add(grid);
function applySceneTheme(){
  const css=getComputedStyle(document.documentElement),color=name=>new THREE.Color(css.getPropertyValue(name).trim());
  while(grid.children.length){const child=grid.children[0];grid.remove(child);disposeTree(child);}
  for(const [divisions,token,opacity] of [[100,'--grid-minor',.7],[10,'--grid-major',.9]]){const g=new THREE.GridHelper(10,divisions,color(token),color(token));g.material.transparent=true;g.material.opacity=opacity;g.material.depthWrite=false;grid.add(g);}
}
applySceneTheme();
const originAxes=new THREE.AxesHelper(.18);scene.add(originAxes);
const wireGroup=new THREE.Group();scene.add(wireGroup);
const wireColor='#4c9aff',wireWarning='#ff5a52';
const startMarker=new THREE.Mesh(new THREE.SphereGeometry(.004,16,10),new THREE.MeshBasicMaterial({color:'#4c9aff',depthTest:false}));
const endMarker=new THREE.Mesh(new THREE.SphereGeometry(.0035,16,10),new THREE.MeshBasicMaterial({color:'#2fcf7c',depthTest:false}));
startMarker.renderOrder=10;endMarker.renderOrder=10;scene.add(startMarker,endMarker);startMarker.visible=endMarker.visible=false;
const cgHandle=new THREE.Group();cgHandle.visible=false;scene.add(cgHandle);
const cgLabel=document.createElement('div');cgLabel.id='cg-label';cgLabel.textContent='CG · 편집 중';cgLabel.hidden=true;viewport.append(cgLabel);
const cgBall=new THREE.Mesh(new THREE.SphereGeometry(.018,20,12),new THREE.MeshBasicMaterial({color:'#ff64d5',depthTest:false}));cgBall.renderOrder=20;cgHandle.add(cgBall);
const cgAxes=new THREE.AxesHelper(.09);cgAxes.material.depthTest=false;cgAxes.renderOrder=20;cgHandle.add(cgAxes);
const aeroOverlay=new THREE.Group();scene.add(aeroOverlay);
// Relative wind drawn in the aircraft (CAD) frame: the analysis flies along CAD +X, so air arrives from +X toward −X.
const flowGroup=new THREE.Group();flowGroup.visible=false;scene.add(flowGroup);
const flowLabel=document.createElement('div');flowLabel.id='flow-label';flowLabel.hidden=true;viewport.append(flowLabel);
const flowLabelLocal=new THREE.Vector3();let flowAsset=null;
function updateFlow(){
  const a=objects.aircraft,on=!!a&&$('show-flow').checked;flowGroup.visible=on;if(!on){flowLabel.hidden=true;return;}
  if(flowAsset!==a.asset){
    flowAsset=a.asset;while(flowGroup.children.length){const o=flowGroup.children[0];flowGroup.remove(o);disposeTree(o);}
    const box=new THREE.Box3(),v=new THREE.Vector3();for(const p of a.asset.parts)for(let i=0;i<p.positions.length;i+=3)box.expandByPoint(v.set(p.positions[i],p.positions[i+1],p.positions[i+2]));
    const size=box.getSize(new THREE.Vector3()),c=box.getCenter(new THREE.Vector3()),L=Math.max(size.x,size.y,size.z,.05),h=Math.max(size.z,L*.15),len=L*.28,start=box.max.x+L*.45;
    for(const y of [-.32,0,.32])for(const z of [-.2,.2])flowGroup.add(new THREE.ArrowHelper(new THREE.Vector3(-1,0,0),new THREE.Vector3(start,c.y+y*size.y,c.z+z*h),len,0x3cc9d8,len*.26,len*.13));
    flowLabelLocal.set(start-len/2,c.y,c.z-h*.45);
  }
  let speed=null;try{speed=analysisUI?.snapshot()?.speed;}catch{}
  flowLabel.textContent=`공기 흐름 · 기체 +X → −X${Number.isFinite(speed)?` · ${speed} m/s`:''}`;
}
function highlightParts(indices){objects.aircraft?.group.children.forEach(m=>{if(m.material?.emissive)m.material.emissive.set(indices.includes(m.userData.part)?'#285faf':'#000000');});render();}
function editCG(point,callback){
  transform.detach();cgCallback=callback||null;cgHandle.visible=!!point;
  transform.setSpace(point?'local':'world');
  if(point){$('move').setAttribute('aria-pressed','true');$('rotate').setAttribute('aria-pressed','false');}
  if(point&&objects.aircraft){objects.aircraft.group.updateMatrixWorld(true);cgHandle.position.copy(objects.aircraft.group.localToWorld(new THREE.Vector3().fromArray(point)));cgHandle.quaternion.copy(objects.aircraft.group.quaternion);transform.setMode('translate');transform.setSpace('local');transform.attach(cgHandle);status('분홍색 CG의 축을 드래그하세요. 좌표는 CAD 원점 기준으로 저장합니다.');}
  render();
}
function showAeroSurfaces(surfaces){
  while(aeroOverlay.children.length){const o=aeroOverlay.children[0];aeroOverlay.remove(o);disposeTree(o);}
  const g=objects.aircraft?.group;if(!g)return;aeroOverlay.position.copy(g.position);aeroOverlay.quaternion.copy(g.quaternion);
  for(const surface of surfaces){const sections=surface.sections.filter(s=>s.le_m.every(Number.isFinite)&&Number.isFinite(s.chord_m));
    for(const side of (surface.mirror?[1,-1]:[1])){const points=[];
      for(const sec of sections){const le=new THREE.Vector3(...sec.le_m),te=le.clone();te.x-=sec.chord_m;le.y*=side;te.y*=side;points.push(le,te);}
      for(let i=0;i<sections.length-1;i++)points.push(points[i*2],points[(i+1)*2],points[i*2+1],points[(i+1)*2+1]);
      if(points.length){const line=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(points),new THREE.LineBasicMaterial({color:'#f6c453',depthTest:false}));line.renderOrder=18;aeroOverlay.add(line);}
    }
  }render();
}

let selectionBox=null,wireSignature='';

function status(message){$('status').textContent=message;}
function error(message){$('error').textContent=message;$('error').hidden=!message;if(document.body.dataset.view==='analysis')analysisUI?.showError(message);}
function setBusy(value,message='모델을 불러오는 중…'){busy=value;$('busy').hidden=!value;$('busy-text').textContent=message;viewport.setAttribute('aria-busy',String(value));document.querySelectorAll('button,input,select').forEach(b=>{if(value){b.dataset.wasDisabled=String(b.disabled);b.disabled=true;}else if(b.dataset.wasDisabled){b.disabled=b.dataset.wasDisabled==='true';delete b.dataset.wasDisabled;}});if(!value){refresh();analysisUI?.refreshState();}}
async function task(fn,message){if(busy)return;error('');setBusy(true,message);try{await fn();}catch(e){error(e.message || '처리하지 못했습니다. 다시 시도해 주세요.');}finally{setBusy(false);}}
function render(){if(selectionBox && selected && objects[selected])selectionBox.setFromObject(objects[selected].group);if(typeof cgHandle!=='undefined'){cgLabel.hidden=!cgHandle.visible;if(cgHandle.visible){const p=cgHandle.position.clone().project(camera);cgLabel.style.left=`${(p.x+1)*viewport.clientWidth/2+12}px`;cgLabel.style.top=`${(1-p.y)*viewport.clientHeight/2+12}px`;}}
  if(typeof flowGroup!=='undefined'&&flowGroup.visible&&objects.aircraft){const g=objects.aircraft.group;flowGroup.position.copy(g.position);flowGroup.quaternion.copy(g.quaternion);flowGroup.updateMatrixWorld(true);const p=flowGroup.localToWorld(flowLabelLocal.clone()).project(camera),inside=p.z<1&&Math.abs(p.x)<1.1&&Math.abs(p.y)<1.1;flowLabel.hidden=!inside;if(inside){flowLabel.style.left=`${(p.x+1)*viewport.clientWidth/2}px`;flowLabel.style.top=`${(1-p.y)*viewport.clientHeight/2}px`;}}
  renderer.render(scene,camera);}
const observer=new ResizeObserver(()=>{const {width,height}=viewport.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/Math.max(height,1);camera.updateProjectionMatrix();render();});observer.observe(viewport);
orbit.addEventListener('change',render);
transform.addEventListener('change',render);
transform.addEventListener('dragging-changed',event=>{orbit.enabled=!event.value;if(!event.value){if(transform.object===cgHandle&&cgCallback){cgCallback(localFromWorld(cgHandle.position.toArray(),objects.aircraft.group.position.toArray(),objects.aircraft.group.quaternion.toArray()));status('CG 위치를 변경했습니다. 새 버전 등록 후 해석에 적용됩니다.');}else{remember();status('위치를 변경했습니다.');}}});
transform.addEventListener('objectChange',()=>{refreshInspector();updateWire();render();});

function disposeTree(root){root.traverse(o=>{o.geometry?.dispose();const mats=Array.isArray(o.material)?o.material:[o.material];mats.forEach(m=>m?.dispose());});}
function validateAsset(asset){
  if(asset.kind==='point_mass'){pointAsset(asset.mass_kg);if('parts' in asset)throw new Error('질점에는 센서 형상을 저장하지 않습니다.');return asset;}
  if(!Array.isArray(asset.parts)||!asset.parts.length)throw new Error('표시할 표면이 없습니다. 모델을 다시 내보내 주세요.');
  let triangles=0;
  for(const p of asset.parts){
    if(!Array.isArray(p.positions)||!p.positions.length||p.positions.length%3||!p.positions.every(Number.isFinite))throw new Error('모델 좌표가 올바르지 않습니다.');
    if(!Array.isArray(p.indices)||!p.indices.length||p.indices.length%3||!p.indices.every(v=>Number.isInteger(v)&&v>=0&&v<p.positions.length/3))throw new Error('모델 표면 데이터가 올바르지 않습니다.');
    triangles+=p.indices.length/3;
  }
  if(triangles>1000000)throw new Error('삼각형이 100만 개를 넘습니다. 표시용 메시를 줄여 다시 불러와 주세요.');
  return asset;
}
function install(role,asset,position,quaternion=[0,0,0,1]){
  validateAsset(asset);
  if(selected===role)transform.detach();
  if(objects[role]){scene.remove(objects[role].group);disposeTree(objects[role].group);}
  const group=new THREE.Group();group.name=role;group.userData.role=role;
  asset.id ||= crypto.randomUUID();
  if(asset.kind==='point_mass'){const marker=new THREE.Mesh(new THREE.SphereGeometry(.025,16,12),new THREE.MeshStandardMaterial({color:colors.sensor,roughness:.65}));marker.userData={role};group.add(marker);}
  for(const [i,part] of (asset.parts||[]).entries()){
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(part.positions,3));geometry.setIndex(part.indices);geometry.computeVertexNormals();geometry.computeBoundingSphere();
    const material=new THREE.MeshStandardMaterial({color:part.color||colors[role],roughness:.65,metalness:.12,side:THREE.DoubleSide});
    const mesh=new THREE.Mesh(geometry,material);mesh.name=part.name;mesh.userData={role,part:i};group.add(mesh);
  }
  group.position.fromArray(position);group.quaternion.fromArray(quaternion).normalize();scene.add(group);group.updateMatrixWorld(true);
  objects[role]={asset,group};applyXray();
}
function remove(role){if(!objects[role])return;if(selected===role){selected=null;transform.detach();}scene.remove(objects[role].group);disposeTree(objects[role].group);objects[role]=null;}
function select(role){
  selected=objects[role]?role:null;transform.detach();
  if(selectionBox){scene.remove(selectionBox);selectionBox.dispose();selectionBox=null;}
  if(selected){
    selectionBox=new THREE.BoxHelper(objects[selected].group,0x4c9aff);selectionBox.material.depthTest=false;selectionBox.material.transparent=true;selectionBox.material.opacity=.65;scene.add(selectionBox);
    // Aircraft orientation is changed only by the 90° CAD turns (the analysis frame), never by a display rotation.
    if(selected!=='sensor'&&(selected!=='aircraft'||!$('aircraft-lock').checked&&transform.getMode()!=='rotate'))transform.attach(objects[selected].group);
  }
  refresh();render();
}
function refresh(){
  const count=Object.values(objects).filter(Boolean).length;$('object-count').textContent=`${count}개`;$('object-empty').hidden=count>0;$('empty-scene').hidden=count>0;
  const list=$('object-list');list.replaceChildren();
  for(const [role,entry] of Object.entries(objects))if(entry){
    const b=document.createElement('button');b.className='object-item';b.setAttribute('aria-pressed',String(selected===role));b.dataset.role=role;b.disabled=busy;
    const swatch=document.createElement('span');swatch.className='swatch';swatch.style.background=colors[role];
    const text=document.createElement('span');text.textContent=names[role];const small=document.createElement('small');small.textContent=entry.asset.source?.name||names[role];text.append(small);b.append(swatch,text);b.onclick=()=>select(role);list.append(b);
  }
  $('add-winch').disabled=busy||!!objects.winch;
  $('add-point').disabled=busy||!objects.winch||!!objects.sensor;
  $('point-mass').disabled=busy;
  $('connection-state').textContent=objects.sensor&&objects.winch?'자동 연결됨':'질점 대기';
  $('connection-state').dataset.state=objects.sensor&&objects.winch?'connected':'unconnected';
  const step=!objects.aircraft?1:!objects.winch?2:!objects.sensor?3:4;
  document.querySelectorAll('[data-step]').forEach(item=>{const n=Number(item.dataset.step);item.classList.toggle('done',n<step);if(n===step)item.setAttribute('aria-current','step');else item.removeAttribute('aria-current');});
  $('undo').disabled=busy||undoStack.length<2;$('redo').disabled=busy||!redoStack.length;
  refreshInspector();updateWire();updateFlow();
}
function refreshInspector(){
  $('selection-kind').dataset.role=selected||'';
  const entry=objects[selected];$('selection-kind').textContent=entry?names[selected]:'선택 없음';
  // Role first; the source name is secondary.
  $('selection-name').textContent=entry?names[selected]:'모델을 선택하세요.';
  if(entry?.asset.source?.name){const from=document.createElement('small');from.textContent=(selected==='winch'?'형상 · ':selected==='aircraft'?'파일 · ':'')+entry.asset.source.name;$('selection-name').append(from);}
  if(selected==='aircraft'&&entry){const turns=entry.asset.source?.orientation_turns||[];$('orient-state').textContent=turns.length?`보정됨 · ${turns.join(' → ')}`:'보정 없음 · 파일 방향 그대로';$('orient-state').dataset.corrected=String(!!turns.length);
    const q=entry.group.quaternion;$('reset-aircraft-pose').hidden=Math.abs(q.w)>1-1e-9;}
  $('transform-fields').disabled=busy||!entry||selected==='sensor'||(selected==='aircraft'&&$('aircraft-lock').checked);
  if(!entry){
    for(const id of ['px','py','pz','rx','ry','rz'])$(id).value='';
    $('selection-size').textContent='모델을 선택하면 외곽 크기가 표시됩니다.';
    return;
  }
  const g=entry.group,e=new THREE.Euler().setFromQuaternion(g.quaternion,'XYZ');
  ['px','py','pz'].forEach((id,i)=>{if(document.activeElement!==$(id))$(id).value=(g.position.getComponent(i)*1000).toFixed(2);});
  ['rx','ry','rz'].forEach((id,i)=>{if(document.activeElement!==$(id))$(id).value=THREE.MathUtils.radToDeg([e.x,e.y,e.z][i]).toFixed(2);});
  if(selected==='sensor'){$('selection-size').textContent=`질량 ${entry.asset.mass_kg} kg · 점의 크기는 표시용입니다.`;return;}
  const bounds=new THREE.Box3().setFromObject(g).getSize(new THREE.Vector3());
  $('selection-size').textContent=`외곽 크기 ${bounds.toArray().map(v=>(v*1000).toFixed(1)).join(' × ')} mm${selected==='winch'?' · 파란 점이 줄 출구입니다.':''}`;
}
function applyXray(){objects.aircraft?.group.traverse(o=>{if(o.isMesh){o.material.transparent=$('xray').checked;o.material.opacity=$('xray').checked?.17:1;o.material.depthWrite=!$('xray').checked;}});render();}
function snapshot(){syncPoint();return {aircraft_definition:structuredClone(aircraftDefinition),objects:Object.fromEntries(Object.entries(objects).map(([k,v])=>[k,v?{asset:v.asset.kind==='point_mass'?{...v.asset}:v.asset,position:v.group.position.toArray(),quaternion:v.group.quaternion.toArray()}:null])),cable:structuredClone(cable),selected,locked:$('aircraft-lock').checked};}
function signature(s){return JSON.stringify({...s,objects:Object.fromEntries(Object.entries(s.objects).map(([k,v])=>[k,v?{asset:v.asset.id,mass_kg:v.asset.mass_kg,position:v.position,quaternion:v.quaternion}:null]))});}
function remember(){const s=snapshot();if(undoStack.length&&signature(undoStack.at(-1))===signature(s))return;undoStack.push(s);if(undoStack.length>40)undoStack.shift();redoStack=[];dirty=true;queueDraft();refresh();render();}
function restore(s,replaceAssets=false){
  aircraftDefinition=structuredClone(s.aircraft_definition||null);editCG(null);
  transform.detach();for(const role of Object.keys(objects)){const v=s.objects[role];if(!v)remove(role);else if(!replaceAssets&&objects[role]?.asset.id===v.asset.id){objects[role].group.position.fromArray(v.position);objects[role].group.quaternion.fromArray(v.quaternion);if(v.asset.kind==='point_mass')objects[role].asset={...v.asset};}else install(role,v.asset,v.position,v.quaternion);}
  cable=structuredClone(s.cable);$('aircraft-lock').checked=s.locked??true;
  $('wire-diameter').value=cable.diameter_m*1000;$('wire-length').value=cable.length_m;
  if(objects.sensor)$('point-mass').value=objects.sensor.asset.mass_kg;
  select(s.selected);refresh();definitionUI?.refresh();render();
}
function undo(){if(undoStack.length<2)return;redoStack.push(undoStack.pop());restore(undoStack.at(-1));queueDraft();status('이전 배치로 되돌렸습니다.');}
function redo(){if(!redoStack.length)return;const s=redoStack.pop();undoStack.push(s);restore(s);queueDraft();status('배치를 다시 적용했습니다.');}
function worldAttachment(){if(!cable.attachment||!objects.sensor)return null;objects.sensor.group.updateMatrixWorld(true);return objects.sensor.group.localToWorld(new THREE.Vector3().fromArray(cable.attachment.local_point_m));}
function winchPoint(){objects.winch?.group.updateMatrixWorld(true);return objects.winch?.group.localToWorld(new THREE.Vector3());}
function clearWire(){while(wireGroup.children.length){const child=wireGroup.children[0];wireGroup.remove(child);disposeTree(child);}}
function updateWire(){
  syncPoint();
  const a=winchPoint(),b=worldAttachment();startMarker.visible=!!a;if(a)startMarker.position.copy(a);endMarker.visible=!!b;if(b)endMarker.position.copy(b);
  if(!a||!b){clearWire();wireSignature='';$('wire-measure').textContent='연결된 줄 없음';$('path-status').className='path-status';$('path-status').textContent='윈치와 질점을 추가하면 줄을 자동 연결합니다.';return;}
  const sig=JSON.stringify([a.toArray(),b.toArray(),cable.diameter_m,cable.length_m,$('xray').checked,objects.aircraft?.group.matrixWorld.elements]);
  if(sig===wireSignature)return;wireSignature=sig;clearWire();
  const distance=a.distanceTo(b),direction=b.clone().sub(a).normalize();let crossings=[];
  if(distance>1e-8&&objects.aircraft){
    objects.aircraft.group.updateMatrixWorld(true);const probe=new THREE.Raycaster(a,direction,1e-5,Math.max(distance-1e-5,1e-5));
    crossings=probe.intersectObject(objects.aircraft.group,true).filter((h,i,arr)=>!i||Math.abs(h.distance-arr[i-1].distance)>1e-5);
  }
  const short=cable.length_m+1e-9<distance;const warning=crossings.length>0||short;
  if(distance>1e-8){
    const tube=new THREE.Mesh(new THREE.CylinderGeometry(cable.diameter_m/2,cable.diameter_m/2,distance,12),new THREE.MeshStandardMaterial({color:warning?wireWarning:wireColor,roughness:.55}));
    tube.position.copy(a).add(b).multiplyScalar(.5);tube.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),direction);wireGroup.add(tube);
    const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([a,b]),new THREE.LineBasicMaterial({color:warning?wireWarning:wireColor}));wireGroup.add(line);
  }
  for(const h of crossings){const mark=new THREE.Mesh(new THREE.SphereGeometry(.003,10,8),new THREE.MeshBasicMaterial({color:wireWarning,depthTest:false}));mark.position.copy(h.point);mark.renderOrder=8;wireGroup.add(mark);}
  $('wire-measure').textContent=`직선 거리 ${(distance*1000).toFixed(1)} mm`;
  $('path-status').className=`path-status ${warning?'warning':'clear'}`;
  $('path-status').textContent=short?`줄 길이가 직선 거리 ${distance.toFixed(3)} m보다 짧습니다. 길이나 배치를 수정하세요.`:crossings.length?`길이 미리보기 직선이 기체와 ${crossings.length}곳에서 겹칩니다. 실제 줄 경로나 형상 접촉을 계산한 결과는 아닙니다.`:'직선 줄과 기체 표면의 교차가 없습니다. 그림은 초기 길이 확인용이며 질점 해석에 형상 접촉은 포함되지 않습니다.';
}
function pointerRay(event){const rect=renderer.domElement.getBoundingClientRect();mouse.set((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1);ray.setFromCamera(mouse,camera);}
function pick(event,role=null){pointerRay(event);scene.updateMatrixWorld(true);let targets=Object.values(objects).filter(Boolean).map(v=>v.group);if(role)targets=objects[role]?[objects[role].group]:[];
  const hits=ray.intersectObjects(targets,true);return hits.find(h=>h.object.isMesh && (!$('xray').checked||role||h.object.userData.role!=='aircraft'))||null;
}
function syncPoint(){
  cable.attachment=objects.sensor&&objects.winch?{kind:'point_mass',local_point_m:[0,0,0]}:null;
  if(!cable.attachment)return;
  objects.sensor.group.position.fromArray(previewPosition(objects.winch.group.position.toArray(),objects.aircraft?.group.quaternion.toArray()||[0,0,0,1],cable.length_m));
  objects.sensor.group.quaternion.identity();objects.sensor.group.updateMatrixWorld(true);
}
function addPoint(){
  if(!objects.winch)throw new Error('윈치를 먼저 추가하세요.');
  if(objects.sensor)return;
  readBasicInputs();
  install('sensor',pointAsset(finiteInput($('point-mass').value)),[0,0,0]);syncPoint();select('sensor');remember();view();status('질점을 추가하고 줄을 자동 연결했습니다. 질점 위치는 줄 길이로 표시합니다.');
}

renderer.domElement.addEventListener('pointerdown',event=>{
  lastPointer={x:event.clientX,y:event.clientY};
  if(event.button!==0||busy||transform.axis)return;
  if(definitionUI?.isActive()){const h=pick(event,'aircraft');if(h){definitionUI.pick(h.object.userData.part,event.shiftKey);event.stopImmediatePropagation();}return;}
  const hit=pick(event);if(!hit)return;const role=hit.object.userData.role;select(role);
  if(role==='sensor'||role==='aircraft'&&$('aircraft-lock').checked)return;
  if(transform.getMode()==='rotate')return;
  const plane=new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()),objects[role].group.position);
  pointerRay(event);const point=ray.ray.intersectPlane(plane,new THREE.Vector3());if(!point)return;
  drag={role,plane,offset:objects[role].group.position.clone().sub(point),start:objects[role].group.position.clone(),x:event.clientX,y:event.clientY,moved:false,id:event.pointerId};
  event.stopImmediatePropagation();orbit.enabled=false;renderer.domElement.setPointerCapture(event.pointerId);
},true);
renderer.domElement.addEventListener('pointermove',event=>{
  if(drag){pointerRay(event);const point=ray.ray.intersectPlane(drag.plane,new THREE.Vector3());if(point&&Math.hypot(event.clientX-drag.x,event.clientY-drag.y)>2){drag.moved=true;objects[drag.role].group.position.copy(point).add(drag.offset);refreshInspector();updateWire();render();}event.stopImmediatePropagation();return;}

},true);
function finishDrag(event,cancel=false){if(!drag)return;const d=drag;drag=null;if(cancel)objects[d.role].group.position.copy(d.start);orbit.enabled=true;if(renderer.domElement.hasPointerCapture(d.id))renderer.domElement.releasePointerCapture(d.id);event?.stopImmediatePropagation();if(d.moved&&!cancel)remember();else {refresh();render();}}
renderer.domElement.addEventListener('pointerup',e=>finishDrag(e),true);renderer.domElement.addEventListener('pointercancel',e=>finishDrag(e,true),true);
renderer.domElement.addEventListener('contextmenu',event=>event.preventDefault());

function view(kind='all'){
  const entries=kind==='focus'&&selected?[objects[selected]]:Object.values(objects).filter(Boolean);if(!entries.length)return;
  const bounds=new THREE.Box3();entries.forEach(o=>bounds.expandByObject(o.group));if(kind!=='focus'&&flowGroup.visible)bounds.expandByObject(flowGroup);const center=bounds.getCenter(new THREE.Vector3()),size=bounds.getSize(new THREE.Vector3());
  const radius=Math.max(size.length()/2,.015),distance=radius/Math.sin(THREE.MathUtils.degToRad(camera.fov/2))*1.2/Math.min(camera.aspect,1);
  const direction=kind==='side'?new THREE.Vector3(0,-1,0):kind==='top'?new THREE.Vector3(0,0,-1):kind==='rear'?new THREE.Vector3(-1,0,0):new THREE.Vector3(-1.4,-1.2,-.8).normalize();
  camera.up.set(0,0,-1);if(kind==='top')camera.up.set(1,0,0);
  camera.position.copy(center).addScaledVector(direction,distance);orbit.target.copy(center);camera.near=Math.max(.0001,distance/10000);camera.far=Math.max(100,distance*100);camera.updateProjectionMatrix();orbit.update();render();
}
async function addWinch(){
  if(objects.winch)return;if(!objects.aircraft)throw new Error('기체를 먼저 불러오세요.');
  const drum=new THREE.CylinderGeometry(.025,.025,.05,24).translate(0,0,.025),base=new THREE.BoxGeometry(.075,.065,.008).translate(0,0,.054);
  const asset={parts:[geometryPart(drum,'winch_drum'),geometryPart(base,'winch_base')],source:{name:'윈치 표시 · 치수는 해석에 사용하지 않음',units:'m',axes:'FRD',origin:'feed_point'}};drum.dispose();base.dispose();
  const group=objects.aircraft.group;group.updateMatrixWorld(true);
  const cgY=aircraftDefinition?.mass?.mode==='total'&&Number.isFinite(aircraftDefinition.mass.cg_m?.[1])?aircraftDefinition.mass.cg_m[1]:0;
  const position=onCenterline(new THREE.Box3().setFromObject(group).getCenter(new THREE.Vector3()).toArray(),group.position.toArray(),group.quaternion.toArray(),cgY);
  install('winch',asset,position,group.quaternion.toArray());select('winch');remember();status('윈치를 기체 중심선에 추가했습니다. 파란 점이 줄 출구입니다. 윈치 모양과 치수는 표시용입니다.');
}

const turnQuaternion=m=>new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().set(m[0][0],m[0][1],m[0][2],0,m[1][0],m[1][1],m[1][2],0,m[2][0],m[2][1],m[2][2],0,0,0,0,1));
// Keep the winch fixed to the airframe while the aircraft frame changes (CAD turn or pose reset).
function carryWinch(fromQ,toQ,localTurn=null){
  const a=objects.aircraft.group,w=objects.winch?.group;if(!w)return;
  const o=a.position.toArray();let local=localFromWorld(w.position.toArray(),o,fromQ.toArray());let ql=fromQ.clone().invert().multiply(w.quaternion);
  if(localTurn){local=applyMatrix(localTurn,local);ql=turnQuaternion(localTurn).multiply(ql);}
  w.position.fromArray(worldFromLocal(local,o,toQ.toArray()));w.quaternion.copy(toQ.clone().multiply(ql).normalize());w.updateMatrixWorld(true);
}
function reorientAircraft(axis,sign){
  const a=objects.aircraft;if(!a)return;
  if(aircraftDefinition&&!isBlankDefinition(aircraftDefinition))throw new Error('기체 정의에 입력한 좌표·단면은 현재 방향 기준입니다. 방향 보정은 기체 정의를 입력하기 전에 하세요. 되돌리기(Ctrl+Z)로 입력 전 상태로 돌아갈 수 있습니다.');
  const q=a.group.quaternion.clone(),m=quarterTurn(axis,sign);
  carryWinch(q,q,m);aircraftDefinition=null;editCG(null);showAeroSurfaces([]);
  install('aircraft',reorientAsset(a.asset,axis,sign),a.group.position.toArray(),q.toArray());wireSignature='';
  select('aircraft');remember();definitionUI?.refresh();
  status(`기체 방향을 ${axis.toUpperCase()}축 ${sign<0?'−':'+'}90° 보정했습니다. 청록색 공기 흐름 화살표가 기체 코 쪽에서 오는지 확인하세요.`);
}
function resetAircraftPose(){
  const a=objects.aircraft;if(!a)return;const from=a.group.quaternion.clone(),to=new THREE.Quaternion();
  carryWinch(from,to);a.group.quaternion.copy(to);a.group.updateMatrixWorld(true);wireSignature='';select('aircraft');remember();
  status('기체의 화면 표시 회전을 0으로 되돌렸습니다. 윈치는 기체와 함께 움직였습니다.');
}

function geometryPart(geometry,name,matrix=new THREE.Matrix4()){
  const g=geometry.clone().applyMatrix4(matrix),p=g.getAttribute('position');if(!p)throw new Error('모델에 표면 좌표가 없습니다.');
  const positions=Array.from(p.array),indices=g.index?Array.from(g.index.array):Array.from({length:p.count},(_,i)=>i);g.dispose();return {name,positions,indices};
}
async function loadModel(file,role){
  if(file.size>64*1024**2)throw new Error('64 MB 이하의 파일을 선택해 주세요.');
  const extension=file.name.split('.').at(-1).toLowerCase(),buffer=await file.arrayBuffer();let parts;
  if(extension==='stl'){try{const geometry=new STLLoader().parse(buffer);parts=[geometryPart(geometry,file.name)];geometry.dispose();}catch(e){throw new Error('STL 표면을 읽지 못했습니다. 파일이 손상되지 않았는지 확인한 뒤 다시 내보내 주세요.');}}
  else if(extension==='glb'){
    const manager=new THREE.LoadingManager();manager.setURLModifier(url=>{if(!url.startsWith('blob:')&&!url.startsWith('data:'))throw new Error('외부 파일을 참조하지 않는 단일 GLB로 내보내 주세요.');return url;});
    const gltf=await new GLTFLoader(manager).parseAsync(buffer,'');gltf.scene.updateMatrixWorld(true);parts=[];
    gltf.scene.traverse(o=>{if(o.isMesh){if(o.isSkinnedMesh)throw new Error('변형 메시를 적용한 정적 GLB로 내보내 주세요.');parts.push(geometryPart(o.geometry,o.name||file.name,o.matrixWorld));}});disposeTree(gltf.scene);
  }else if(extension==='step'||extension==='stp'){
    $('busy-text').textContent='STEP 형상을 변환하는 중…';const response=await fetch('/api/import-step?name='+encodeURIComponent(file.name),{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:buffer});const value=await response.json();if(!response.ok)throw new Error(value.error);parts=value.parts;
  }else throw new Error('STEP, STL, GLB 중 하나를 선택해 주세요.');
  const unit=importUnit(extension,$('import-unit').value),scale={mm:.001,cm:.01,m:1}[unit],axes=$('import-axes').value;
  for(const part of parts)for(let i=0;i<part.positions.length;i+=3){let [x,y,z]=part.positions.slice(i,i+3);if(axes==='zup')[x,y,z]=[x,-y,-z];if(axes==='aru')[x,y,z]=[-x,y,-z];if(axes==='yup')[x,y,z]=[x,z,-y];part.positions.splice(i,3,x*scale,y*scale,z*scale);}
  const bounds=new THREE.Box3();for(const p of parts)for(let i=0;i<p.positions.length;i+=3)bounds.expandByPoint(new THREE.Vector3().fromArray(p.positions,i));
  const center=new THREE.Vector3();
  const sha=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',buffer)),b=>b.toString(16).padStart(2,'0')).join('');
  const asset=validateAsset({parts,source:{name:file.name,units:unit,axes,sha256:sha,source_origin_shift_m:center.toArray(),origin:'file_origin'}});
  // A different airframe must not inherit the previous winch/payload placement.
  if(role==='aircraft'){definitionUI?.close();aircraftDefinition=null;remove('sensor');remove('winch');cable.attachment=null;}
  install(role,asset,[0,0,0]);select(role);remember();view();status(`${file.name}을 불러왔습니다. 치수를 확인하고, 청록색 공기 흐름 화살표가 기체 코 쪽에서 오는지 확인하세요. 아니면 기체를 선택해 방향을 보정하세요.`);$('scene-badge').textContent='사용자 모델 · 배치 편집';
  recentModelFile=file;$('recent-model-file').hidden=false;$('recent-model-file').textContent=file.name+' · 다시 끌어넣기';
}

function project({includeAnalysis=true}={}){readBasicInputs();syncPoint();return {schema:'dbf-assembly/2',units:'m',axes:'FRD',aircraft_definition:structuredClone(aircraftDefinition),saved_at:new Date().toISOString(),objects:Object.fromEntries(Object.entries(objects).map(([role,v])=>[role,v?{...v.asset,position:v.group.position.toArray(),quaternion:v.group.quaternion.toArray()}:null])),cable:structuredClone(cable),view:{selected,aircraft_locked:$('aircraft-lock').checked},...(includeAnalysis?{analysis:analysisUI?.snapshot()}:{})};}
async function postProject(value){const response=await fetch('/api/projects',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value)});const result=await response.json();if(!response.ok)throw new Error(result.error);return result;}
function downloadJSON(value,name){const blob=new Blob([JSON.stringify(value,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),10000);}
async function applyProject(value){
  const legacy=value.schema==='dbf-assembly/1';value=migrateProject(value);
  if(value.schema!=='dbf-assembly/2'||value.units!=='m'||value.axes!=='FRD')throw new Error('이 편집기에서 저장한 DBF 배치 JSON을 선택해 주세요.');
  const staged={};for(const role of Object.keys(objects)){const o=value.objects[role];if(o){validateAsset(o);for(const key of ['position','quaternion'])if(!Array.isArray(o[key])||!o[key].every(Number.isFinite))throw new Error('저장된 위치 값이 올바르지 않습니다.');staged[role]={asset:{...o,id:crypto.randomUUID()},position:o.position,quaternion:o.quaternion};}else staged[role]=null;}
  restore({objects:staged,aircraft_definition:value.aircraft_definition,cable:value.cable,selected:value.view?.selected,locked:value.view?.aircraft_locked},true);await analysisUI?.restore(value.analysis);remember();view();if(legacy)status(value.migration_note);
}
function openDraftDB(){return new Promise((resolve,reject)=>{const request=indexedDB.open('dbf-assembly-editor',1);request.onupgradeneeded=()=>request.result.createObjectStore('projects');request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);});}
function queueDraft(){clearTimeout(savingTimer);$('save-state').textContent='배치 변경됨';$('save-state').title='';savingTimer=setTimeout(async()=>{let value;try{value=project();}catch(e){$('save-state').textContent='입력 확인 필요 · 저장되지 않음';$('save-state').title=e.message;return;}try{draftDB ||= await openDraftDB();const tx=draftDB.transaction('projects','readwrite');tx.objectStore('projects').put(value,'draft');tx.oncomplete=()=>{$('save-state').textContent='이 브라우저에 자동 저장됨';dirty=false;};tx.onerror=()=>{$('save-state').textContent='자동 저장 실패 · 파일 저장을 사용하세요';};}catch(e){$('save-state').textContent='파일 저장을 사용하세요';}},400);}

$('empty-load').onclick=()=>$('load-aircraft').click();
$('add-winch').onclick=()=>task(addWinch,'윈치를 추가하는 중…');
for(const role of ['aircraft'])$('load-'+role).onclick=()=>{pendingRole=role;$('model-file').click();};
$('model-file').onchange=e=>{const file=e.target.files[0],role=pendingRole;e.target.value='';if(file)task(()=>loadModel(file,role));};
$('recent-model-file').addEventListener('dragstart',e=>{
  if(busy||!recentModelFile){e.preventDefault();return;}
  e.dataTransfer.setData('application/x-dbf-recent-model','aircraft');e.dataTransfer.effectAllowed='copy';
});
$('recent-model-file').onclick=()=>{if(!busy&&recentModelFile)task(()=>loadModel(recentModelFile,'aircraft'));};
const isModelDrag=e=>Array.from(e.dataTransfer?.types||[]).some(t=>t==='Files'||t==='application/x-dbf-recent-model');
viewport.addEventListener('dragover',e=>{if(!isModelDrag(e))return;e.preventDefault();e.dataTransfer.dropEffect=busy?'none':'copy';viewport.classList.toggle('file-drag-over',!busy);});
viewport.addEventListener('dragleave',e=>{if(!viewport.contains(e.relatedTarget))viewport.classList.remove('file-drag-over');});
viewport.addEventListener('drop',e=>{
  if(!isModelDrag(e))return;e.preventDefault();viewport.classList.remove('file-drag-over');
  if(busy){error('파일을 불러오는 중입니다. 완료 후 다시 놓아 주세요.');return;}
  const files=Array.from(e.dataTransfer.files||[]);
  if(!files.length&&e.dataTransfer.getData('application/x-dbf-recent-model')==='aircraft'&&recentModelFile)files.push(recentModelFile);
  if(files.length!==1){error('기체 파일을 하나씩 놓아 주세요.');return;}
  task(()=>loadModel(files[0],'aircraft'));
});
// Keep files dropped outside the canvas from replacing the editor page.
window.addEventListener('dragover',e=>{if(isModelDrag(e))e.preventDefault();});
window.addEventListener('drop',e=>{if(isModelDrag(e))e.preventDefault();});
$('add-point').onclick=()=>task(async()=>addPoint(),'질점을 추가하는 중…');
function readBasicInputs(){
  const mass=pointAsset(finiteInput($('point-mass').value)).mass_kg;
  const length=finiteInput($('wire-length').value),diameter=finiteInput($('wire-diameter').value)/1000;
  if(length<=0||length>100||diameter<.00005||diameter>.02)throw new Error('줄 길이는 0 초과~100 m, 직경은 0.05~20 mm로 입력하세요.');
  cable.length_m=length;cable.diameter_m=diameter;
  if(objects.sensor)objects.sensor.asset={...objects.sensor.asset,mass_kg:mass};
}
function editBasics(){try{readBasicInputs();error('');remember();}catch(e){error(e.message);}}
$('point-mass').oninput=$('point-mass').onchange=editBasics;
$('undo').onclick=undo;$('redo').onclick=redo;
$('aircraft-lock').onchange=()=>{select(selected);remember();};$('xray').onchange=()=>{applyXray();wireSignature='';updateWire();};
$('show-flow').onchange=()=>{updateFlow();render();};
document.querySelectorAll('[data-turn]').forEach(b=>b.onclick=()=>{if(busy)return;error('');try{reorientAircraft(b.dataset.turn,Number(b.dataset.sign));}catch(e){error(e.message);}});
$('reset-aircraft-pose').onclick=()=>{if(!busy)resetAircraftPose();};
for(const id of ['px','py','pz','rx','ry','rz'])$(id).oninput=$(id).onchange=()=>{if(!selected||!objects[selected]||selected==='sensor')return;error('');try{editPoseComponent(objects[selected].group,id,$(id).value);remember();}catch(e){error(e.message);}};
for(const id of ['wire-diameter','wire-length'])$(id).oninput=$(id).onchange=editBasics;
for(const [id,kind] of [['view-all','all'],['view-side','side'],['view-top','top'],['view-rear','rear'],['focus','focus']])$(id).onclick=()=>view(kind);
function setMode(mode){
  if(cgHandle.visible&&cgCallback){
    if(mode==='rotate'){status('CG는 위치만 설정합니다. 분홍색 축을 드래그하세요.');return;}
    transform.setMode('translate');transform.attach(cgHandle);render();return;
  }
  transform.setMode(mode);$('move').setAttribute('aria-pressed',String(mode==='translate'));$('rotate').setAttribute('aria-pressed',String(mode==='rotate'));select(selected);
  if(mode==='rotate'&&selected==='aircraft')status('기체 방향은 오른쪽 선택 속성의 해석 기준 방향에서 90° 단위로 바꿉니다.');
}
$('move').onclick=()=>setMode('translate');$('rotate').onclick=()=>setMode('rotate');
$('save-project').onclick=()=>task(async()=>{const value=project();await postProject(value);downloadJSON(value,'DBF_센서배치.dbf-scene.json');dirty=false;$('save-state').textContent='프로젝트 파일 저장됨';status('모델과 배치를 프로젝트 파일로 저장했습니다.');},'배치를 저장하는 중…');
$('save-project-step').onclick=()=>$('save-project').click();
$('open-project').onclick=()=>$('project-file').click();
$('project-file').onchange=e=>{const file=e.target.files[0];e.target.value='';if(file)task(async()=>{if(file.size>64*1024**2)throw new Error('64 MB 이하 프로젝트를 선택하세요.');const value=migrateProject(JSON.parse(await file.text()));await postProject(value);await applyProject(value);status(value.migration_note||'저장한 기체·윈치·질점을 다시 불러왔습니다.');},'프로젝트를 여는 중…');};
$('export-setup').onclick=()=>{error('');if(!objects.aircraft||!objects.sensor||!objects.winch||!cable.attachment){error('기체와 윈치를 배치하고 질점을 추가하세요.');return;}scene.updateMatrixWorld(true);const aircraftInverse=objects.aircraft.group.matrixWorld.clone().invert(),sensorMatrix=aircraftInverse.clone().multiply(objects.sensor.group.matrixWorld),p=new THREE.Vector3(),q=new THREE.Quaternion(),scale=new THREE.Vector3();sensorMatrix.decompose(p,q,scale);const start=winchPoint().applyMatrix4(aircraftInverse),end=worldAttachment().applyMatrix4(aircraftInverse);
  downloadJSON({schema:'dbf-assembly-setup/2',units:'SI',axes:'FRD',aircraft_definition:structuredClone(aircraftDefinition),analysis_ready:false,aircraft_origin:'imported model origin; CG must be specified separately',sensor:{kind:'point_mass',mass_kg:objects.sensor.asset.mass_kg,preview_position_aircraft_m:p.toArray()},winch:{feed_point_aircraft_m:start.toArray()},cable:{...cable,straight_distance_m:start.distanceTo(end)},sources:Object.fromEntries(Object.entries(objects).map(([k,v])=>[k,v.asset.source])),required_analysis_inputs:['aircraft mass, CG and inertia','aerodynamic database matching the aircraft','cable mass, stiffness, damping and drag','deployment/recovery commands; point-mass model excludes CAD contact']},'DBF_해석용배치.setup.json');status('좌표·연결점·줄 설정을 내보냈습니다. 해석 물성은 별도로 지정해야 합니다.');
};
window.addEventListener('keydown',e=>{if($('analysis-dialog')?.open)return;if(busy||/INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName))return;if(e.key==='Escape'){if(drag)finishDrag(null,true);transform.reset();select(selected);status('현재 조작을 취소했습니다.');}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){e.preventDefault();e.shiftKey?redo():undo();}else if(!e.ctrlKey&&!e.metaKey){if(e.key.toLowerCase()==='g')setMode('translate');if(e.key.toLowerCase()==='r')setMode('rotate');if(e.key.toLowerCase()==='f')view('focus');}});
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});
// Read-only diagnostic surface for reproducible local UI checks.
window.dbfEditor={snapshot:()=>project(),getScreenPoint:(role,local=[0,0,0])=>{const v=objects[role]?.group.localToWorld(new THREE.Vector3().fromArray(local));if(!v)return null;v.project(camera);const r=renderer.domElement.getBoundingClientRect();return {x:r.left+(v.x+1)*r.width/2,y:r.top+(1-v.y)*r.height/2};}};
definitionUI=initAircraftDefinition({getAircraft:()=>objects.aircraft?.asset,getDefinition:()=>aircraftDefinition,setDefinition:v=>{aircraftDefinition=v;},getProject:()=>project({includeAnalysis:false}),highlight:highlightParts,editCG,showSurfaces:showAeroSurfaces,onSave:remember,onError:error,onRegistered:async id=>{await analysisUI.useModel(id);queueDraft();await analysisUI.open();}});
$('open-aircraft-definition').onclick=()=>definitionUI.open();
analysisUI=initAnalysis({getProject:()=>project({includeAnalysis:false}),openDefinition:()=>definitionUI.open(),onChange:()=>{dirty=true;queueDraft();updateFlow();}});
$('open-analysis').onclick=$('open-analysis-panel').onclick=()=>analysisUI.open();
function syncThemeButton(){const light=document.documentElement.dataset.theme==='light',label=light?'어두운 화면으로 전환':'밝은 화면으로 전환';$('theme-toggle').setAttribute('aria-label',label);$('theme-toggle').title=label;}
$('theme-toggle').onclick=()=>{const next=document.documentElement.dataset.theme==='light'?'dark':'light';document.documentElement.dataset.theme=next;try{localStorage.setItem('dbf-theme',next);}catch{}syncThemeButton();applySceneTheme();render();};
syncThemeButton();
undoStack=[snapshot()];refresh();render();
try{
  draftDB=await openDraftDB();
  const tx=draftDB.transaction('projects','readwrite'),store=tx.objectStore('projects'),req=store.get('draft');
  req.onsuccess=async()=>{if(!req.result)return;try{
    const previous=req.result;
    if(previous.schema==='dbf-assembly/1')store.put(previous,'legacy-draft-before-point-mass');
    await applyProject(previous);
    if(previous.schema!=='dbf-assembly/1')status('이 브라우저에 저장한 질점 배치를 복원했습니다.');
  }catch(e){error('이전 자동 저장을 열지 못했습니다: '+e.message);}};
}catch(e){$('save-state').textContent='파일 저장 사용';}
