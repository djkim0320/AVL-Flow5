// Keep saved (including nonuniform) samples intact until a slider is moved.
export const axes = [
  { key: 'alpha_deg', label: '받음각', min: -20, max: 25 },
  { key: 'beta_deg', label: '옆미끄럼각', min: -20, max: 20 },
  { key: 'elevator_deg', label: '승강타각', min: -30, max: 30 }
];

export function uniformSamples(min, max, count) {
  if (![min, max, count].every(Number.isFinite) || min >= max || !Number.isInteger(count) || count < 2 || count > 10000)
    throw new Error('계산 범위의 최솟값·최댓값·계산점 수를 확인하세요.');
  return Array.from({ length: count }, (_, i) =>
    i === 0 ? min : i === count - 1 ? max : Number((min + ((max - min) * i) / (count - 1)).toPrecision(12))
  );
}

export function checkedSamples(values) {
  if (
    !Array.isArray(values) ||
    values.length < 2 ||
    values.some((v, i) => !Number.isFinite(v) || (i > 0 && v <= values[i - 1]))
  )
    throw new Error('공력 계산점은 작은 값부터 큰 값 순서로 2개 이상 필요합니다.');
  return [...values];
}

export function initAeroRanges(host) {
  const saved = {};
  let elevatorLimit = null;
  host.innerHTML =
    axes
      .map(
        ({ key, label, min, max }) => `<section class="aero-range" aria-labelledby="range-${key}">
    <h4 id="range-${key}">${label}<output data-summary="${key}"></output></h4>
    ${[
      ['min', '최소', min, max, 0.1],
      ['max', '최대', min, max, 0.1],
      ['count', '계산점 수', 2, 21, 1]
    ]
      .map(
        ([part, text, lo, hi, step]) => `
    <label class="range-control" for="${key}-${part}"><span>${text}</span><input id="${key}-${part}" name="aero_grid.${key}.${part}" type="range" min="${lo}" max="${hi}" step="${step}" aria-label="${label} ${text}"><output for="${key}-${part}"></output></label>`
      )
      .join('')}
    <p class="range-samples hint" data-samples="${key}"></p></section>`
      )
      .join('') + '<p class="hint" data-range-state></p>';
  const input = (key, part) => host.querySelector(`[name="aero_grid.${key}.${part}"]`);
  function render(key) {
    const values = saved[key];
    host.querySelector(`[data-summary="${key}"]`).textContent =
      `${values[0]}° ~ ${values.at(-1)}° · ${values.length}점`;
    for (const part of ['min', 'max', 'count']) {
      const e = input(key, part),
        value = part === 'min' ? values[0] : part === 'max' ? values.at(-1) : values.length;
      const text = value + (part === 'count' ? '점' : '°');
      host.querySelector(`output[for="${e.id}"]`).textContent = text;
      e.setAttribute('aria-valuetext', text);
    }
    const preview =
      values.length <= 21 ? values.map(v => Number(v.toFixed(3))).join(' · ') : `${values.length}개 계산점`;
    host.querySelector(`[data-samples="${key}"]`).textContent = preview;
  }
  host.addEventListener('input', e => {
    if (e.target.type !== 'range') return;
    const [, key, part] = e.target.name.split('.');
    let min = part === 'min' ? Number(input(key, 'min').value) : saved[key][0];
    let max = part === 'max' ? Number(input(key, 'max').value) : saved[key].at(-1);
    if (min >= max) {
      if (part === 'min') min = Number((max - 0.1).toFixed(10));
      else max = Number((min + 0.1).toFixed(10));
    }
    input(key, 'min').value = min;
    input(key, 'max').value = max;
    saved[key] = uniformSamples(min, max, Number(input(key, 'count').value));
    render(key);
  });
  return {
    read: () => {
      if (elevatorLimit && saved.elevator_deg.some(v => Math.abs(v) > elevatorLimit))
        throw new Error('flow5가 포함된 계산은 승강타를 ±10° 이내로 설정하세요.');
      return Object.fromEntries(axes.map(({ key }) => [key, checkedSamples(saved[key])]));
    },
    backend(backend) {
      elevatorLimit = backend === 'avl' ? null : 10;
      for (const part of ['min', 'max']) {
        const e = input('elevator_deg', part);
        e.min = elevatorLimit ? -10 : Math.min(-30, saved.elevator_deg?.[0] ?? -30);
        e.max = elevatorLimit ? 10 : Math.max(30, saved.elevator_deg?.at(-1) ?? 30);
        e.setCustomValidity(
          elevatorLimit && saved.elevator_deg?.some(v => Math.abs(v) > 10)
            ? '저장된 범위가 ±10°를 넘습니다. 승강타 범위를 직접 수정하세요.'
            : ''
        );
      }
    },
    write(grid) {
      for (const axis of axes) {
        const values = checkedSamples(grid[axis.key]);
        saved[axis.key] = values;
        for (const part of ['min', 'max']) {
          const e = input(axis.key, part);
          e.min = Math.min(axis.min, Math.floor(values[0]));
          e.max = Math.max(axis.max, Math.ceil(values.at(-1)));
          e.value = part === 'min' ? values[0] : values.at(-1);
        }
        const count = input(axis.key, 'count');
        count.max = Math.max(21, values.length);
        count.value = values.length;
        render(axis.key);
      }
    },
    enable(enabled) {
      host.querySelectorAll('input').forEach(e => (e.disabled = !enabled));
      host.querySelector('[data-range-state]').textContent = enabled
        ? '슬라이더를 움직이면 해당 각도의 계산점을 같은 간격으로 만듭니다.'
        : '저장된 공력표를 사용합니다. 범위를 바꾸려면 ‘실제 해석기로 다시 계산’을 선택하세요.';
    }
  };
}
