import { newDefinition, roleLabels, partBounds, defaultStations, parseFoil } from './aircraft-model.js';
const escape = s =>
  String(s ?? '').replace(
    /[&<>"']/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]
  );
const num = (id, label, value, unit = '', scale = 1) =>
  `<label>${label}${unit ? ` · ${unit}` : ''}<input id="${id}" type="number" step="any" value="${value == null ? '' : value * scale}"></label>`;
const xyz = (prefix, label, value) =>
  `<fieldset class="definition-vector"><legend>${label} · mm / CAD 원점</legend><div class="definition-grid">${['X', 'Y', 'Z'].map((a, i) => num(`${prefix}-${i}`, a, value?.[i], '', 1000)).join('')}</div></fieldset>`;
const tensor = (prefix, a) =>
  `<fieldset><legend>CG 기준 관성 · kg·m² / 기체축</legend><div class="definition-grid">${[
    [0, 0, 'Ixx'],
    [1, 1, 'Iyy'],
    [2, 2, 'Izz'],
    [0, 1, 'Ixy'],
    [0, 2, 'Ixz'],
    [1, 2, 'Iyz']
  ]
    .map(([i, j, l]) => num(`${prefix}-${i}-${j}`, l, a[i][j]))
    .join(
      ''
    )}</div><p class="hint">대각선 밖 값은 관성 행렬 성분입니다. CAD의 관성곱과 부호 규약을 확인하세요.</p></fieldset>`;

export function initAircraftDefinition({
  getAircraft,
  getDefinition,
  setDefinition,
  getProject,
  highlight,
  editCG,
  showSurfaces,
  onRegistered,
  onError,
  onSave
}) {
  const panel = document.createElement('section');
  panel.id = 'aircraft-definition-panel';
  panel.hidden = true;
  panel.setAttribute('aria-label', '기체 정의');
  document.querySelector('.inspector').prepend(panel);
  // A fresh template stays in `draft` until the first edit, so merely opening the
  // panel does not attach an (empty) definition to the project or its model match.
  let active = false,
    tab = 'parts',
    selected = [],
    massPart = 0,
    surfaceIndex = 0,
    requesting = false,
    epoch = 0,
    draft = null;
  const $ = id => panel.querySelector('#' + id);
  const read = id => {
    const s = $(id).value.trim();
    return s === '' ? null : Number(s);
  };
  const d = () => getDefinition() || draft;
  function commit() {
    const value = d();
    draft = null;
    setDefinition(value);
    return value;
  }
  function changed() {
    const value = commit();
    value.reviewed = false;
    if ($('definition-reviewed')) $('definition-reviewed').checked = false;
    onSave();
  }
  function bindNumber(id, setter, scale = 1) {
    $(id).oninput = $(id).onchange = () => {
      setter(read(id) === null ? null : read(id) / scale);
      changed();
    };
  }
  function bindVector(prefix, value) {
    for (let i = 0; i < 3; i++) bindNumber(`${prefix}-${i}`, v => (value[i] = v), 1000);
  }
  function bindTensor(prefix, value) {
    for (const [i, j] of [
      [0, 0],
      [1, 1],
      [2, 2],
      [0, 1],
      [0, 2],
      [1, 2]
    ])
      bindNumber(`${prefix}-${i}-${j}`, v => {
        value[i][j] = v;
        value[j][i] = v;
      });
  }
  async function request(path, extra = {}) {
    const response = await fetch('/api/aircraft/' + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ project: getProject(), ...extra })
    });
    const v = await response.json();
    if (!response.ok) throw new Error(v.error);
    return v;
  }
  async function action(fn) {
    if (requesting) return;
    if (draft) {
      commit();
      onSave();
    }
    requesting = true;
    const version = epoch;
    $('definition-status').textContent = '처리 중…';
    panel.querySelectorAll('button,input,select,textarea').forEach(e => (e.disabled = true));
    try {
      await fn(() => version === epoch);
    } catch (e) {
      onError(e.message);
      if (version === epoch) $('definition-status').textContent = e.message;
    } finally {
      requesting = false;
      if (version === epoch) {
        panel.querySelectorAll('button,input,select,textarea').forEach(e => (e.disabled = false));
      }
    }
  }
  function syncSelection() {
    selected = selected.filter(i => i < d().parts.length);
    massPart = Math.min(massPart, Math.max(0, d().parts.length - 1));
    surfaceIndex = Math.min(surfaceIndex, Math.max(0, d().surfaces.length - 1));
  }
  function open(indices) {
    if (!getAircraft()) {
      onError('기체를 먼저 불러오세요.');
      return;
    }
    if (!d()) draft = newDefinition(getAircraft());
    if (Array.isArray(indices)) {
      selected = [...indices];
      tab = 'parts';
    }
    syncSelection();
    active = true;
    panel.hidden = false;
    document.body.classList.add('defining-aircraft');
    render();
  }
  function close() {
    active = false;
    epoch++;
    draft = null;
    panel.hidden = true;
    document.body.classList.remove('defining-aircraft');
    highlight([]);
    editCG(null);
    showSurfaces([]);
  }
  function render() {
    if (!active || !d()) return;
    panel.innerHTML = `<div class="definition-heading"><h2>기체 정의</h2><button type="button" id="definition-close" aria-label="기체 정의 닫기">닫기</button></div><p class="hint">한 번 등록하면 윈치·질점·줄 조건을 바꿔 반복 해석할 수 있습니다. 수정 중인 정의는 새 버전 등록 후 해석에 적용됩니다.</p>
    <nav class="definition-tabs" aria-label="기체 정의 단계">${[
      ['parts', '부품'],
      ['mass', '질량·CG'],
      ['aero', '공력 면']
    ]
      .map(([id, name]) => `<button type="button" data-tab="${id}" aria-pressed="${tab === id}">${name}</button>`)
      .join(
        ''
      )}</nav><div id="definition-content"></div><p id="definition-status" role="status" aria-live="polite"></p>`;
    $('definition-close').onclick = close;
    panel.querySelectorAll('[data-tab]').forEach(
      b =>
        (b.onclick = () => {
          tab = b.dataset.tab;
          editCG(null);
          render();
        })
    );
    if (tab === 'parts') parts();
    else if (tab === 'mass') mass();
    else aero();
    highlight(selected);
    showSurfaces(tab === 'aero' ? d().surfaces : []);
  }
  function parts() {
    $('definition-content').innerHTML =
      `<p>3D에서 부품을 클릭하세요. Shift를 누르면 여러 부품을 선택합니다.</p><label>부품 목록<select id="definition-parts" multiple size="8">${d()
        .parts.map(
          p =>
            `<option value="${p.index}" ${selected.includes(p.index) ? 'selected' : ''}>${p.index + 1}. ${escape(p.name)} · ${roleLabels[p.role]}</option>`
        )
        .join(
          ''
        )}</select></label><div class="definition-inline"><label>선택 부품 역할<select id="definition-role">${Object.entries(
        roleLabels
      )
        .map(([k, v]) => `<option value="${k}">${v}</option>`)
        .join(
          ''
        )}</select></label><button type="button" id="definition-assign">역할 적용</button></div><label>부품 이름<input id="definition-part-name" value="${selected.length === 1 ? escape(d().parts[selected[0]].name) : ''}" ${selected.length !== 1 ? 'disabled' : ''}></label><p class="hint">공력 제외와 질량 제외는 다릅니다. 장비 질량도 질량·CG에서 포함하세요.</p><button type="button" id="definition-to-mass">질량·CG 입력</button>`;
    $('definition-parts').onchange = () => {
      selected = [...$('definition-parts').selectedOptions].map(o => Number(o.value));
      highlight(selected);
      parts();
    };
    if (selected.length) $('definition-role').value = d().parts[selected[0]].role;
    $('definition-assign').onclick = () => {
      if (!selected.length) {
        onError('부품을 먼저 선택하세요.');
        return;
      }
      for (const i of selected) d().parts[i].role = $('definition-role').value;
      changed();
      parts();
    };
    $('definition-part-name').onchange = () => {
      if (selected.length === 1) {
        d().parts[selected[0]].name = $('definition-part-name').value;
        changed();
        parts();
      }
    };
    $('definition-to-mass').onclick = () => {
      tab = 'mass';
      render();
    };
  }
  function mass() {
    const m = d().mass,
      components = m.mode === 'components';
    massPart = Math.min(massPart, d().parts.length - 1);
    const target = components ? d().parts[massPart] : m;
    $('definition-content').innerHTML =
      `<label>입력 방식<select id="definition-mass-mode"><option value="total">기체 전체 값 입력</option><option value="components">부품별 값 합산</option></select></label><p class="hint">기체 값에 배터리·모터·윈치는 포함하고, 줄과 센서 질점은 제외하세요. 형상만으로 밀도나 실제 관성을 추정하지 않습니다.</p>
    ${
      components
        ? `<label>물성을 입력할 부품<select id="definition-mass-part">${d()
            .parts.map(p => `<option value="${p.index}">${p.index + 1}. ${escape(p.name)}</option>`)
            .join(
              ''
            )}</select></label><label class="check"><input id="definition-mass-excluded" type="checkbox" ${target.mass_excluded ? 'checked' : ''}>질량 합산에서 제외 · 중복 CAD/표시용 부품</label>`
        : ''
    }
    ${num('definition-mass', '질량', target.mass_kg, 'kg')}${xyz('definition-cg', components ? '부품 CG' : '기체 CG', target.cg_m)}
    <div class="definition-inline"><button type="button" id="definition-cg-place">CG를 3D에서 이동</button><button type="button" id="definition-cg-center">형상 중앙에 CG 초안 배치</button></div><p class="hint">형상 중앙은 CG 측정값이 아닙니다. 분홍색 CG 표시를 드래그하거나 위 좌표를 입력하세요.</p>
    ${tensor('definition-inertia', target.inertia_kgm2)}${components ? '<button type="button" id="definition-mass-sum">전체 질량·CG·관성 계산</button><pre id="definition-mass-result"></pre>' : ''}`;
    $('definition-mass-mode').value = m.mode;
    $('definition-mass-mode').onchange = () => {
      m.mode = $('definition-mass-mode').value;
      changed();
      editCG(null);
      mass();
    };
    if (components) {
      $('definition-mass-part').value = String(massPart);
      $('definition-mass-part').onchange = () => {
        massPart = Number($('definition-mass-part').value);
        highlight([massPart]);
        editCG(null);
        mass();
      };
      $('definition-mass-excluded').onchange = () => {
        target.mass_excluded = $('definition-mass-excluded').checked;
        changed();
      };
    }
    bindNumber('definition-mass', v => (target.mass_kg = v));
    bindVector('definition-cg', target.cg_m);
    bindTensor('definition-inertia', target.inertia_kgm2);
    const place = () => {
      if (!target.cg_m.every(v => Number.isFinite(v))) {
        onError('CG 좌표를 입력하거나 형상 중앙에 초안을 배치하세요.');
        return;
      }
      editCG(target.cg_m, point => {
        target.cg_m.splice(0, 3, ...point);
        changed();
        for (let i = 0; i < 3; i++)
          if ($('definition-cg-' + i)) $('definition-cg-' + i).value = (point[i] * 1000).toFixed(3);
      });
    };
    $('definition-cg-place').onclick = place;
    $('definition-cg-center').onclick = () => {
      target.cg_m = partBounds(getAircraft(), components ? [massPart] : d().parts.map(p => p.index)).center;
      changed();
      mass();
      $('definition-cg-place').click();
    };
    for (let i = 0; i < 3; i++)
      $('definition-cg-' + i).addEventListener('change', () => {
        if (target.cg_m.every(v => Number.isFinite(v))) place();
      });
    if (components)
      $('definition-mass-sum').onclick = () =>
        action(async current => {
          const r = await request('mass');
          if (!current()) return;
          $('definition-mass-result').textContent =
            `질량 ${r.mass_kg.toFixed(4)} kg\nCG ${r.cg_m.map(x => (x * 1000).toFixed(2)).join(', ')} mm\n관성 ${JSON.stringify(r.inertia_kgm2)}`;
          $('definition-status').textContent = '부품별 관성을 평행축 정리로 합산했습니다.';
        });
  }
  function aero() {
    surfaceIndex = Math.min(surfaceIndex, Math.max(0, d().surfaces.length - 1));
    $('definition-content').innerHTML =
      `<p class="hint">양력면을 단면으로 정의합니다. 동체·문·장비의 압력 분포는 계산하지 않습니다. 각 단면과 조종면 범위를 확인하세요.</p><label>공력 면<select id="definition-surface">${d()
        .surfaces.map((s, i) => `<option value="${i}">${escape(s.name)}</option>`)
        .join(
          ''
        )}</select></label><div class="definition-inline"><button type="button" id="definition-surface-add">선택 부품으로 면 추가</button><button type="button" id="definition-surface-remove">이 면 삭제</button></div><div id="definition-surface-editor"></div>
    <details open><summary>기준 치수·추진</summary><div class="definition-grid">${num('definition-area', '날개 면적', d().references.area_m2, 'm²')}${num('definition-chord', '평균 시위', d().references.chord_m, 'm')}${num('definition-span', '날개폭', d().references.span_m, 'm')}</div>${num('definition-cd', '추가 항력계수', d().physical.profile_cd)}${num('definition-thrust', '최대 추력', d().physical.max_thrust_N, 'N')}${xyz('definition-thrust-point', '추력 작용점', d().physical.thrust_point_m)}<label class="check"><input id="definition-flow5" type="checkbox" ${d().physical.flow5_closure_confirmed ? 'checked' : ''}>flow5의 종·횡 분리 근사 사용에 동의</label><p class="hint">flow5에서 출력하지 않는 종·횡 교차 회전율 미계수는 0으로 가정합니다. 큰 옆미끄럼·비대칭 조종 조건의 검증은 별도입니다. AVL 또는 복합 공력표의 회전율 출처가 AVL이면 이 동의가 필요하지 않습니다.</p></details>
    <details open><summary>확인·등록</summary><label>기체 이름<input id="definition-name" maxlength="100" value="${escape(d().name)}"></label><label>자료 구분<select id="definition-source-kind"><option value="assumption">가정값 포함</option><option value="design">설계값</option><option value="measured">측정값</option></select></label><label>형상·물성 출처와 가정<textarea id="definition-source" rows="3">${escape(d().source)}</textarea></label><button type="button" id="definition-preview">해석 입력 확인</button><p id="definition-preview-result" role="status"></p><label class="check"><input id="definition-reviewed" type="checkbox" ${d().reviewed ? 'checked' : ''}>단면·조종면·물성·좌표를 확인했습니다.</label><button type="button" id="definition-register" class="primary">해석 모델 등록</button><p class="hint">새 버전으로 등록하며 첫 공력표는 실제 AVL 또는 flow5로 계산합니다. 줄 물성·비행 조건은 해석 화면에서 확인하세요.</p></details>`;
    $('definition-surface').value = String(surfaceIndex);
    $('definition-surface').onchange = () => {
      surfaceIndex = Number($('definition-surface').value);
      surfaceEditor();
    };
    $('definition-surface-add').onclick = () => {
      if (!selected.length) {
        onError('부품 탭이나 3D에서 면에 포함할 부품을 선택하세요.');
        return;
      }
      const axis = selected.some(i => d().parts[i].role === 'vertical_tail') ? 'z' : 'y';
      d().surfaces.push({
        name: `공력 면 ${d().surfaces.length + 1}`,
        parts: [...selected],
        axis,
        mirror: false,
        control: 'none',
        hinge_fraction: 0.75,
        sections: []
      });
      surfaceIndex = d().surfaces.length - 1;
      changed();
      aero();
    };
    $('definition-surface-remove').onclick = () => {
      if (!d().surfaces.length) return;
      d().surfaces.splice(surfaceIndex, 1);
      changed();
      aero();
      showSurfaces(d().surfaces);
    };
    for (const [id, key] of [
      ['area', 'area_m2'],
      ['chord', 'chord_m'],
      ['span', 'span_m']
    ])
      bindNumber('definition-' + id, v => (d().references[key] = v));
    bindNumber('definition-cd', v => (d().physical.profile_cd = v));
    bindNumber('definition-thrust', v => (d().physical.max_thrust_N = v));
    bindVector('definition-thrust-point', d().physical.thrust_point_m);
    $('definition-flow5').onchange = () => {
      d().physical.flow5_closure_confirmed = $('definition-flow5').checked;
      changed();
    };
    $('definition-name').oninput = $('definition-name').onchange = () => {
      d().name = $('definition-name').value;
      changed();
    };
    $('definition-source-kind').value = d().source_kind;
    $('definition-source-kind').onchange = () => {
      d().source_kind = $('definition-source-kind').value;
      changed();
    };
    $('definition-source').oninput = $('definition-source').onchange = () => {
      d().source = $('definition-source').value;
      changed();
    };
    $('definition-reviewed').onchange = () => {
      commit().reviewed = $('definition-reviewed').checked;
      onSave();
    };
    $('definition-preview').onclick = () =>
      action(async current => {
        const r = await request('preview');
        if (!current()) return;
        $('definition-preview-result').textContent =
          `${r.files.length}개 해석 파일 · 질량 ${r.mass.mass_kg.toFixed(3)} kg · CG ${r.mass.cg_m.map(x => (x * 1000).toFixed(2)).join(', ')} mm. ${r.limitations.join(' ')}`;
        $('definition-status').textContent = '입력 확인 완료. 실제 해석은 등록 후 실행하세요.';
      });
    $('definition-register').onclick = () =>
      action(async current => {
        const r = await request('register');
        if (!current()) return;
        await onRegistered(r.id);
        if (current()) {
          $('definition-status').textContent = r.name + ' 등록 완료';
          close();
        }
      });
    surfaceEditor();
  }
  function surfaceEditor() {
    const s = d().surfaces[surfaceIndex],
      box = $('definition-surface-editor');
    if (!s) {
      box.innerHTML = '<p class="hint">부품을 선택한 뒤 공력 면을 추가하세요.</p>';
      return;
    }
    box.innerHTML = `<label>면 이름<input id="surface-name" value="${escape(s.name)}"></label><p class="hint">연결 부품: ${s.parts.map(i => escape(d().parts[i].name)).join(', ')}</p><div class="definition-inline"><label>단면 절단 축<select id="surface-axis"><option value="y">Y · 수평 날개</option><option value="z">Z · 수직 날개</option></select></label><label class="check"><input id="surface-mirror" type="checkbox" ${s.mirror ? 'checked' : ''}>오른쪽 반쪽을 좌우 대칭 복제</label></div><div class="definition-inline"><label>조종면<select id="surface-control"><option value="none">없음</option><option value="elevator">승강타</option><option value="aileron">에일러론</option><option value="rudder">방향타</option></select></label>${num('surface-hinge', '힌지 x/c', s.hinge_fraction)}</div><p class="hint">조종면은 이 면의 전체 span에 적용합니다. 부분 조종면은 면을 구간별로 정의하세요.</p><label>단면 위치 · m · 쉼표로 구분<input id="surface-stations" placeholder="예: 0.02, 0.4, 0.78"></label><div class="definition-inline"><button type="button" id="surface-stations-propose">절단 위치 제안</button><button type="button" id="surface-extract">CAD에서 단면 추출</button></div><button type="button" id="surface-section-add">단면 직접 추가</button><div id="surface-sections"></div>`;
    $('surface-name').onchange = () => {
      s.name = $('surface-name').value;
      changed();
    };
    $('surface-axis').value = s.axis;
    $('surface-axis').onchange = () => {
      s.axis = $('surface-axis').value;
      if (s.axis === 'z') s.mirror = false;
      changed();
      surfaceEditor();
    };
    $('surface-mirror').onchange = () => {
      s.mirror = $('surface-mirror').checked;
      changed();
    };
    $('surface-control').value = s.control;
    $('surface-control').onchange = () => {
      s.control = $('surface-control').value;
      changed();
    };
    bindNumber('surface-hinge', v => (s.hinge_fraction = v));
    $('surface-stations-propose').onclick = () => {
      try {
        $('surface-stations').value = defaultStations(getAircraft(), s.parts, s.axis, s.mirror)
          .map(x => +x.toFixed(6))
          .join(', ');
      } catch (e) {
        onError(e.message);
      }
    };
    $('surface-extract').onclick = () =>
      action(async current => {
        const raw = $('surface-stations').value.trim();
        if (!raw) throw new Error('절단 위치를 입력하거나 제안 버튼을 누르세요.');
        const stations = raw.split(',').map(x => Number(x.trim()));
        const r = await request('sections', { request: { parts: s.parts, axis: s.axis, stations_m: stations } });
        if (!current()) return;
        s.sections = r.sections;
        changed();
        sections();
        showSurfaces(d().surfaces);
        $('definition-status').textContent = r.note;
      });
    $('surface-section-add').onclick = () => {
      s.sections.push({
        le_m: [null, null, null],
        chord_m: null,
        incidence_deg: 0,
        span_panels: 8,
        foil: { kind: 'naca4', code: '' }
      });
      changed();
      sections();
    };
    selected = [...s.parts];
    highlight(selected);
    sections();
  }
  function sections() {
    const s = d().surfaces[surfaceIndex];
    $('surface-sections').innerHTML = s.sections
      .map(
        (sec, i) =>
          `<details class="definition-section"><summary>단면 ${i + 1} · ${sec.le_m?.[s.axis === 'y' ? 1 : 2]?.toFixed(3) ?? '미입력'} m</summary>${xyz(`sec-${i}-le`, '앞전 위치', sec.le_m)}<div class="definition-grid">${num(`sec-${i}-chord`, '시위', sec.chord_m, 'm')}${num(`sec-${i}-angle`, '설치각', sec.incidence_deg, '°')}${num(`sec-${i}-panels`, '다음 구간 분할', sec.span_panels)}</div><label>에어포일<select id="sec-${i}-foil"><option value="naca4">NACA 4자리</option><option value="coordinates">CAD 단면 / DAT 좌표</option></select></label><label>NACA 번호<input id="sec-${i}-naca" maxlength="4" value="${escape(sec.foil.code || '')}" placeholder="예: 2412"></label><label>에어포일 DAT<input id="sec-${i}-file" type="file" accept=".dat,.txt"></label><p class="hint">${escape(sec.foil.source || '')} ${sec.foil.points ? sec.foil.points.length + '개 좌표' : ''}</p><button type="button" id="sec-${i}-remove">단면 삭제</button></details>`
      )
      .join('');
    s.sections.forEach((sec, i) => {
      bindVector(`sec-${i}-le`, sec.le_m);
      bindNumber(`sec-${i}-chord`, v => (sec.chord_m = v));
      bindNumber(`sec-${i}-angle`, v => (sec.incidence_deg = v));
      bindNumber(`sec-${i}-panels`, v => (sec.span_panels = v));
      $(`sec-${i}-foil`).value = sec.foil.kind;
      $(`sec-${i}-foil`).onchange = () => {
        sec.foil.kind = $(`sec-${i}-foil`).value;
        changed();
      };
      $(`sec-${i}-naca`).oninput = $(`sec-${i}-naca`).onchange = () => {
        sec.foil = { kind: 'naca4', code: $(`sec-${i}-naca`).value };
        changed();
        $(`sec-${i}-foil`).value = 'naca4';
      };
      $(`sec-${i}-file`).onchange = async e => {
        const file = e.target.files[0];
        if (!file) return;
        try {
          sec.foil = { ...parseFoil(await file.text()), source: file.name };
          changed();
          sections();
        } catch (e) {
          onError(e.message);
        }
      };
      $(`sec-${i}-remove`).onclick = () => {
        s.sections.splice(i, 1);
        changed();
        sections();
        showSurfaces(d().surfaces);
      };
    });
    $('surface-sections').onchange = () => showSurfaces(d().surfaces);
  }
  return {
    open,
    close,
    isActive: () => active,
    pick: (index, add) => {
      if (!active || requesting) return;
      selected = add ? (selected.includes(index) ? selected.filter(i => i !== index) : [...selected, index]) : [index];
      highlight(selected);
      if (tab === 'parts') parts();
    },
    refresh: () => {
      if (active) {
        epoch++;
        if (!getAircraft()) close();
        else {
          if (!getDefinition()) draft = newDefinition(getAircraft());
          syncSelection();
          render();
        }
      }
    }
  };
}
