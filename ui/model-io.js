// Reading STL / GLB / STEP files into display assets (metres, FRD) and validating asset records.
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { importUnit } from './coordinates.js';
import { pointAsset } from './point-mass.js';
import { validateComponents } from './aircraft-assembly.js';
import { disposeTree } from './scene.js';

export const MAX_FILE_BYTES = 64 * 1024 ** 2;

export function geometryPart(geometry, name, matrix = new THREE.Matrix4()) {
  const g = geometry.clone().applyMatrix4(matrix),
    p = g.getAttribute('position');
  if (!p) throw new Error('모델에 표면 좌표가 없습니다.');
  const positions = Array.from(p.array),
    indices = g.index ? Array.from(g.index.array) : Array.from({ length: p.count }, (_, i) => i);
  g.dispose();
  return { name, positions, indices };
}

export function validateAsset(asset) {
  if (asset.kind === 'point_mass') {
    pointAsset(asset.mass_kg);
    if ('parts' in asset) throw new Error('질점에는 센서 형상을 저장하지 않습니다.');
    return asset;
  }
  if (!Array.isArray(asset.parts) || !asset.parts.length)
    throw new Error('표시할 표면이 없습니다. 모델을 다시 내보내 주세요.');
  let triangles = 0;
  for (const p of asset.parts) {
    if (
      !Array.isArray(p.positions) ||
      !p.positions.length ||
      p.positions.length % 3 ||
      !p.positions.every(Number.isFinite)
    )
      throw new Error('모델 좌표가 올바르지 않습니다.');
    if (
      !Array.isArray(p.indices) ||
      !p.indices.length ||
      p.indices.length % 3 ||
      !p.indices.every(v => Number.isInteger(v) && v >= 0 && v < p.positions.length / 3)
    )
      throw new Error('모델 표면 데이터가 올바르지 않습니다.');
    triangles += p.indices.length / 3;
  }
  if (triangles > 1000000) throw new Error('삼각형이 100만 개를 넘습니다. 표시용 메시를 줄여 다시 불러와 주세요.');
  validateComponents(asset);
  return asset;
}

// `unit` and `axes` are the import options chosen on the page; `onStage` reports long steps (STEP conversion).
export async function readModel(file, { unit: chosenUnit, axes, onStage }) {
  if (file.size > MAX_FILE_BYTES) throw new Error('64 MB 이하의 파일을 선택해 주세요.');
  const extension = file.name.split('.').at(-1).toLowerCase(),
    buffer = await file.arrayBuffer();
  let parts;
  if (extension === 'stl') {
    try {
      const geometry = new STLLoader().parse(buffer);
      parts = [geometryPart(geometry, file.name)];
      geometry.dispose();
    } catch (e) {
      throw new Error('STL 표면을 읽지 못했습니다. 파일이 손상되지 않았는지 확인한 뒤 다시 내보내 주세요.');
    }
  } else if (extension === 'glb') {
    const manager = new THREE.LoadingManager();
    manager.setURLModifier(url => {
      if (!url.startsWith('blob:') && !url.startsWith('data:'))
        throw new Error('외부 파일을 참조하지 않는 단일 GLB로 내보내 주세요.');
      return url;
    });
    const gltf = await new GLTFLoader(manager).parseAsync(buffer, '');
    gltf.scene.updateMatrixWorld(true);
    parts = [];
    gltf.scene.traverse(o => {
      if (o.isMesh) {
        if (o.isSkinnedMesh) throw new Error('변형 메시를 적용한 정적 GLB로 내보내 주세요.');
        parts.push(geometryPart(o.geometry, o.name || file.name, o.matrixWorld));
      }
    });
    disposeTree(gltf.scene);
  } else if (extension === 'step' || extension === 'stp') {
    onStage?.('STEP 형상을 변환하는 중…');
    const response = await fetch('/api/import-step?name=' + encodeURIComponent(file.name), {
      method: 'POST',
      headers: { 'Content-Type': 'application/octet-stream' },
      body: buffer
    });
    const value = await response.json();
    if (!response.ok) throw new Error(value.error);
    parts = value.parts;
  } else throw new Error('STEP, STL, GLB 중 하나를 선택해 주세요.');
  const unit = importUnit(extension, chosenUnit),
    scale = { mm: 0.001, cm: 0.01, m: 1 }[unit];
  for (const part of parts)
    for (let i = 0; i < part.positions.length; i += 3) {
      let [x, y, z] = part.positions.slice(i, i + 3);
      if (axes === 'zup') [x, y, z] = [x, -y, -z];
      if (axes === 'aru') [x, y, z] = [-x, y, -z];
      if (axes === 'yup') [x, y, z] = [x, z, -y];
      part.positions.splice(i, 3, x * scale, y * scale, z * scale);
    }
  const sha = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', buffer)), b =>
    b.toString(16).padStart(2, '0')
  ).join('');
  return validateAsset({
    parts,
    source: {
      name: file.name,
      units: unit,
      axes,
      sha256: sha,
      source_origin_shift_m: [0, 0, 0],
      origin: 'file_origin'
    }
  });
}
