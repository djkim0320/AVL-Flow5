import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id),job=new URL(location.href).searchParams.get('job');
if(!/^[0-9a-f]{32}$/.test(job||''))throw new Error('올바른 해석 결과 링크로 여세요.');
const base='/analysis-files/'+job+'/';$('report').href=base+'report.html';
try{
  const read=async name=>{const r=await fetch(base+name);if(!r.ok)throw new Error('결과 파일을 읽지 못했습니다: '+name);return r.json();};
  const data=await read('replay.json'),project=await read(data.project_file||'project.json');
  // Transparent canvas over the themed CSS gradient (theme.css), matching the editor.
  const scene=new THREE.Scene();const host=$('scene'),css=getComputedStyle(document.documentElement),token=name=>new THREE.Color(css.getPropertyValue(name).trim());
  const camera=new THREE.PerspectiveCamera(38,1,.001,10000);camera.up.set(0,0,-1);
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setClearColor(0x000000,0);renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.append(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xffffff,0x60788e,2));const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(1,-3,-5);scene.add(light);
  const vehicle=new THREE.Group();scene.add(vehicle);
  const groups={},hinge=new THREE.Vector3().fromArray(data.hinge),doors=[];
  for(const [role,item] of Object.entries(project.objects)){
    if(!item||data.phase==='aircraft_only'&&role!=='aircraft')continue;
    const group=new THREE.Group();groups[role]=group;vehicle.add(group);
    if(item.kind==='point_mass'){
      group.add(new THREE.Mesh(new THREE.SphereGeometry(.025,16,12),new THREE.MeshStandardMaterial({color:'#d69c43'})));
      continue;
    }
    for(const part of item.parts){const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(part.positions,3));geometry.setIndex(part.indices);geometry.computeVertexNormals();
      const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:part.color||(role==='sensor'?'#d69c43':'#b3c9d2'),roughness:.7,side:THREE.DoubleSide}));
      if(role==='aircraft'&&(data.mapping.door_parts||['rear_door_100deg']).includes(part.name)){geometry.translate(-hinge.x,-hinge.y,-hinge.z);const door=new THREE.Group();door.position.copy(hinge);door.add(mesh);group.add(door);doors.push(door);}else group.add(mesh);
    }
    if(role==='winch'){group.position.fromArray(data.mapping.winch_position_body_m);group.quaternion.fromArray(data.mapping.winch_quaternion_body_xyzw);}
  }
  const line=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:'#4c9aff'}));vehicle.add(line);line.visible=data.phase!=='aircraft_only';
  const frames=data.frames;let at=0,playing=false,last=0,clock=frames[0].t;
  function draw(index){at=index;const f=frames[index];$('frame').value=index;$('time').textContent=f.t.toFixed(3)+' s · '+f.altitude_m.toFixed(2)+' m';
    vehicle.quaternion.fromArray(f.aircraft_quaternion);
    if(groups.sensor){groups.sensor.position.fromArray(f.sensor);groups.sensor.quaternion.fromArray(f.quaternion);}
    for(const door of doors)door.rotation.y=THREE.MathUtils.degToRad(f.door-(data.mapping.door_reference_deg??270));
    if(line.visible){line.geometry.dispose();line.geometry=new THREE.BufferGeometry().setFromPoints(f.cable.map(p=>new THREE.Vector3().fromArray(p)));}
    renderer.render(scene,camera);
  }
  const bounds=new THREE.Box3();if(groups.aircraft)bounds.setFromObject(groups.aircraft);
  if(data.phase!=='aircraft_only')for(const f of frames){bounds.expandByPoint(new THREE.Vector3().fromArray(f.sensor));for(const p of f.cable)bounds.expandByPoint(new THREE.Vector3().fromArray(p));}
  bounds.expandByScalar(.15);const center=new THREE.Vector3(),extent=2*Math.max(bounds.min.length(),bounds.max.length());
  const grid=new THREE.GridHelper(extent*1.5,20,token('--grid-major'),token('--grid-minor'));grid.rotation.x=Math.PI/2;grid.position.z=extent*.3;scene.add(grid);
  camera.position.copy(center).add(new THREE.Vector3(-.8,-1,-.6).normalize().multiplyScalar(extent*1.5));
  const orbit=new OrbitControls(camera,renderer.domElement);
  orbit.mouseButtons={LEFT:THREE.MOUSE.ROTATE,MIDDLE:THREE.MOUSE.PAN,RIGHT:THREE.MOUSE.PAN};
  orbit.screenSpacePanning=true;
  renderer.domElement.addEventListener('mousedown',event=>{if(event.button===1)event.preventDefault();});
  orbit.target.copy(center);orbit.update();orbit.addEventListener('change',()=>renderer.render(scene,camera));
  new ResizeObserver(()=>{const r=host.getBoundingClientRect();renderer.setSize(r.width,r.height);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();renderer.render(scene,camera);}).observe(host);
  $('frame').max=frames.length-1;$('frame').disabled=false;$('play').disabled=false;
  $('message').textContent=`${data.note} · 상태: ${data.status} · ${frames.length}개 실제 상태 표본 (표본 사이 보간 없음)`;
  $('frame').oninput=()=>{playing=false;$('play').textContent='재생';draw(Number($('frame').value));clock=frames[at].t;};
  $('play').onclick=()=>{playing=!playing;if(playing&&at===frames.length-1){draw(0);clock=frames[0].t;}last=performance.now();$('play').textContent=playing?'일시 정지':'재생';};
  function animate(now){if(playing){clock+=(now-last)/1000*Number($('rate').value);let next=at;while(next<frames.length-1&&frames[next+1].t<=clock)next++;if(next!==at)draw(next);if(next===frames.length-1){playing=false;$('play').textContent='다시 재생';}}last=now;requestAnimationFrame(animate);}draw(0);requestAnimationFrame(animate);
}catch(e){$('message').textContent=e.message;}
