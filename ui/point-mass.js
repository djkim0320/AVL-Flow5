import * as THREE from 'three';

export function pointAsset(mass){
  if(!Number.isFinite(mass)||mass<.0001||mass>10)throw new Error('질점 질량은 0.0001~10 kg으로 입력하세요.');
  return {kind:'point_mass',mass_kg:mass,source:{name:'센서 역할 질점',origin:'point_mass'}};
}

export function previewPosition(winch,aircraftQuaternion,length){
  if(!Number.isFinite(length)||length<=0||length>100)throw new Error('줄 길이는 0보다 크고 100 m 이하여야 합니다.');
  return new THREE.Vector3(0,0,length).applyQuaternion(new THREE.Quaternion().fromArray(aircraftQuaternion)).add(new THREE.Vector3().fromArray(winch)).toArray();
}

// Old files remain on disk. Opening one discards its sensor mesh and surface
// attachment, retaining its explicitly saved payload mass and aircraft/winch.
export function migrateProject(value){
  if(value.schema==='dbf-assembly/2')return value;
  if(value.schema!=='dbf-assembly/1')throw new Error('지원하지 않는 프로젝트 형식입니다.');
  const out=structuredClone(value);out.schema='dbf-assembly/2';
  if(out.objects.sensor){
    const mass=out.analysis?.sensor_mass;
    out.objects.sensor=Number.isFinite(mass)?{...pointAsset(mass),position:[0,0,0],quaternion:[0,0,0,1]}:null;
    out.migration_note=Number.isFinite(mass)?'이전 센서 형상을 질점으로 바꿨습니다. 저장된 질량을 확인하세요.':'기체와 윈치를 복원했습니다. 이전 파일에 질량이 없어 질점은 아직 추가하지 않았습니다.';
  }
  const linked=out.objects.sensor&&out.objects.winch;
  out.cable.attachment=linked?{kind:'point_mass',local_point_m:[0,0,0]}:null;
  if(linked)out.objects.sensor.position=previewPosition(out.objects.winch.position,out.objects.aircraft?.quaternion||[0,0,0,1],out.cable.length_m);
  if(out.analysis){out.analysis.mission_start='registered';out.analysis.start='equilibrium';out.analysis.sensor_yaw_delta=0;}
  return out;
}
