import { Euler, MathUtils } from 'three';

export function importUnit(extension, selected) {
  // OpenCASCADE returns STEP geometry in mm regardless of its source units.
  const unit = ['step', 'stp'].includes(extension) ? 'mm' : selected;
  if (!['mm', 'cm', 'm'].includes(unit)) throw new Error('파일 단위를 확인하세요.');
  return unit;
}

export function finiteInput(raw) {
  if (String(raw).trim() === '' || !Number.isFinite(Number(raw)))
    throw new Error('빈칸 없이 유효한 숫자를 입력하세요.');
  return Number(raw);
}

export function editPoseComponent(group, id, raw) {
  const value = finiteInput(raw),
    index = 'xyz'.indexOf(id[1]);
  if (index < 0 || !['p', 'r'].includes(id[0])) throw new Error('알 수 없는 좌표입니다.');
  if (id[0] === 'p') group.position.setComponent(index, value / 1000);
  else {
    const angles = new Euler().setFromQuaternion(group.quaternion, 'XYZ').toArray().slice(0, 3);
    angles[index] = MathUtils.degToRad(value);
    group.rotation.set(...angles, 'XYZ');
  }
}
