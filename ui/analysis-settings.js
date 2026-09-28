// Registration and advanced solver inputs share the frozen analysis snapshot.
import {initAeroRanges} from './aero-ranges.js';
import {tableOptions,workerOptions} from './analysis-options.js';
const number=(name,label)=>`<label>${label}<input name="${name}" type="number" step="any"></label>`;
const json=(name,label,rows=3)=>`<label>${label}<textarea name="${name}" rows="${rows}" spellcheck="false"></textarea></label>`;


export function initSettings({form,request,getProject,selectModel,showError,openDefinition}){
  const top=document.createElement('fieldset');top.innerHTML=`<legend>해석 모델</legend><label>등록 모델<select name="model_id"></select></label><p class="hint">모델을 바꾸면 해석 조건도 해당 모델의 등록값으로 바뀝니다.</p>  <details id="model-registration"><summary>현재 CAD를 새 해석 모델로 등록</summary><p class="hint">기체 파일을 불러오는 것과 공력 모델을 연결하는 것은 별도입니다. 이 기체의 AVL 형상·참조 에어포일과 질량·무게중심·관성 자료를 함께 등록하세요. STEP만으로 공력이나 관성을 자동 생성하지 않습니다.</p>
  <label>모델 이름<input name="reg_name" maxlength="100"></label>
  <label>물성·접촉 설정 · YAML 또는 JSON<input name="reg_case" type="file" accept=".json,.yaml,.yml"></label>
  <button type="button" id="model-template">빈 설정 템플릿 내려받기</button><p class="hint">내려받은 빈 설정의 null 값을 채우세요. 새 기체의 질량·관성·CG와 공력 기준치를 직접 수정하세요. 센서 형상과 접촉 설정은 사용하지 않습니다.</p>
  <label>AVL 형상 및 참조 파일 · 함께 선택<input name="reg_aero" type="file" accept=".avl,.dat,.txt,.mass" multiple></label>
  <label>하위 폴더가 있는 경우 · 공력 폴더 선택<input name="reg_folder" type="file" webkitdirectory multiple></label>
  ${json('reg_aircraft_cg','기체 CAD 원점에서 본 CG · [x, y, z] m',1)}
  <p class="hint">CAD를 불러온 뒤의 SI/FRD 좌표입니다. 설정 파일의 aircraft.cg_m는 별도로 AVL 형상 기준 CG를 뜻합니다. 센서는 질점으로 계산하며 기체·문과의 형상 접촉은 계산하지 않습니다.</p>
  ${json('reg_doors','문으로 움직일 기체 부품 이름 · JSON 배열',2)}${number('reg_angle','CAD에 저장된 문 각도 · °')}
  <p class="hint" id="registration-parts"></p><p class="hint">문은 설정한 힌지의 기체 Y축으로 회전합니다. 문이 없으면 부품 이름에 []를 입력하세요.</p>
  <label class="check"><input name="reg_confirm" type="checkbox">이 CAD와 공력 형상이 같은 기체이며, 좌표·물성·출처를 확인했습니다.</label>
  <button type="button" id="model-register">현재 CAD와 물성 등록</button><p id="registration-status" role="status"></p></details>`;
  form.querySelector('#analysis-setup').before(top);
  const define=document.createElement('button');define.type='button';define.className='define-button';define.textContent='기체 정의에서 부품·CG·공력 면 편집';define.onclick=openDefinition;top.querySelector('details').before(define);
  top.querySelector('summary').textContent='기존 AVL 파일로 등록 · 고급 입력';
  // Point-mass missions use registered stow/release settings; geometry-contact
  // controls are intentionally absent from this interface.
  const aero=document.createElement('details');aero.innerHTML=`<summary>공력표 선택과 새 계산 범위</summary><label>공력표<select name="aero_job"></select></label>
  <p class="hint">선택한 등록 모델에서 실제로 계산한 표만 표시합니다. flow5 표는 속도도 같아야 합니다. 새 계산은 아래 모든 조합을 사용합니다.</p>
  <div id="hybrid-sources" hidden><p class="hint">블록별 출처를 고르세요. 서로 다른 해석기를 조합하는 가정이며, 시험 검증은 별도입니다.</p>${[['coeff','정적 계수 출처'],['controls','조종 미계수 출처'],['rates','회전율 미계수 출처']].map(([key,label])=>`<label>${label}<select name="hybrid_${key}"><option value="flow5">flow5</option><option value="avl">AVL</option></select></label>`).join('')}</div><div class="aero-ranges"></div>`;
  form.querySelector('#aero-source').after(aero);
  const ranges=initAeroRanges(aero.querySelector('.aero-ranges'));
  const inputs=document.createElement('fieldset');inputs.innerHTML=`<legend>돌풍·조종 입력</legend><details><summary>돌풍·조종 입력·제어기 세부 설정</summary><p class="hint">등록 설정의 이력을 사용합니다. 배열의 시간은 초, 각도는 도입니다. 자세한 형식은 <a href="/analysis-help.html" target="_blank">입력 안내</a>를 확인하세요.</p>
  ${json('gust','돌풍 설정 · 없으면 null',4)}${json('controls','승강타·추력 입력 이력',4)}${json('controller_parameters','제어기 이득과 제한값 · 활성화는 위 체크박스',6)}</details>`;
  form.append(inputs);
  const field=name=>form.elements.namedItem(name);
  const rememberedTables=new Map();let tableContext=null,workerMaximum=1;
  const parse=name=>{try{return JSON.parse(field(name).value);}catch{throw new Error(field(name).closest('label').childNodes[0].textContent+': JSON 형식을 확인하세요.');}};
  function read(key,base){
    if(key==='aero_hybrid')return Object.fromEntries(['coeff','controls','rates'].map(k=>[k,field('hybrid_'+k).value]));
    if(key==='mechanism')return structuredClone(base);
    if(['sensor_mass','sensor_cd','sensor_inertia','sensor_yaw_delta','mission_start'].includes(key))return key==='mission_start'?'registered':structuredClone(base);
    if(key==='aero_grid')return ranges.read();
    if(!field(key))return structuredClone(base);
    if(base===null||typeof base==='object')return parse(key);
    const e=field(key);
    if(e.disabled&&e.closest('[data-scenario]'))return e.value.trim()?Number(e.value):null;
    return e.type==='checkbox'?e.checked:typeof base==='number'?readNumber(key):e.value;
  }
  function readNumber(name){const e=field(name);if(!e.value.trim()||!Number.isFinite(Number(e.value)))throw new Error((e.closest('label')?.childNodes[0]?.textContent.trim()||name)+': 숫자를 입력하세요.');return Number(e.value);}
  function write(key,value){
    if(key==='aero_hybrid'){for(const k of ['coeff','controls','rates'])field('hybrid_'+k).value=value[k];return;}
    if(key==='aero_grid'){ranges.write(value);return;}
    if(key==='mechanism')return;
    if(key==='workers'){setWorkers(value);return;}
    const e=field(key);if(!e)return;if(e.type==='checkbox')e.checked=!!value;else e.value=value===null||typeof value==='object'?JSON.stringify(value,null,e.rows<=2?0:2):value;
  }
  async function refreshModels(){const rows=await request('/api/models');field('model_id').replaceChildren(...rows.map(r=>new Option(r.name,r.id)));return rows;}
  function tables(catalog,backend,selected){
    if(tableContext&&field('aero_job').value)rememberedTables.set(tableContext,field('aero_job').value);
    tableContext=catalog.defaults.model_id+':'+backend;
    const choice=tableOptions(catalog,backend,selected??rememberedTables.get(tableContext));
    field('aero_job').replaceChildren(...choice.rows.map(r=>{const option=new Option(r.label,r.id);option.dataset.missing=String(!!r.missing);return option;}));
    field('aero_job').value=choice.value;validateTable();
  }
  function validateTable(){field('aero_job').setCustomValidity(field('aero_job').selectedOptions[0]?.dataset.missing==='true'?'저장된 공력표를 찾을 수 없습니다. 사용할 표를 선택하거나 새 계산을 명시적으로 선택하세요.':'');}
  field('aero_job').addEventListener('change',validateTable);
  function setWorkers(value){const options=workerOptions(workerMaximum,value);field('workers').replaceChildren(...options.rows.map(r=>new Option(r.label,r.value)));field('workers').value=String(value);field('workers').setCustomValidity(options.missing?`사용 가능한 계산 자원은 최대 ${workerMaximum}개 CPU입니다.`:'');}
  field('workers').addEventListener('change',()=>setWorkers(Number(field('workers').value)));
  field('model_id').onchange=async()=>{showError('');try{await selectModel(field('model_id').value);}catch(e){showError(e.message);}};
  top.querySelector('details').addEventListener('toggle',()=>{if(top.querySelector('details').open){const parts=getProject().objects.aircraft?.parts||[];document.getElementById('registration-parts').textContent='현재 기체 부품: '+parts.map(p=>p.name).join(', ');}});
  document.getElementById('model-template').onclick=async()=>{try{const value=await request('/api/models/template');const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(value,null,2)],{type:'application/json'}));a.download='model-physics-reference.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}catch(e){showError(e.message);}};
  document.getElementById('model-register').onclick=async()=>{
    const button=document.getElementById('model-register'),status=document.getElementById('registration-status');showError('');button.disabled=true;
    try{
      const caseFile=field('reg_case').files[0];if(!caseFile)throw new Error('물성·접촉 설정 파일을 선택하세요.');
      const files=await Promise.all([...field('reg_aero').files,...field('reg_folder').files].map(async f=>({name:f.webkitRelativePath?f.webkitRelativePath.split('/').slice(1).join('/'):f.name,text:await f.text()})));
      const result=await request('/api/models/register',{project:getProject(),registration:{name:field('reg_name').value,case_text:await caseFile.text(),files,
        aircraft_cg_local_m:parse('reg_aircraft_cg'),sensor_cg_local_m:[0,0,0],door_parts:parse('reg_doors'),door_reference_deg:readNumber('reg_angle'),source_confirmed:field('reg_confirm').checked}});
      await refreshModels();await selectModel(result.id);status.textContent=result.name+' 등록 완료. 입력한 물성으로 조건을 불러왔습니다. 첫 공력표는 실제 해석기로 계산합니다.';
    }catch(e){showError(e.message);status.textContent='등록하지 못했습니다.';}finally{button.disabled=false;}
  };
  function showRegistration(name){const details=document.getElementById('model-registration');details.open=true;if(!field('reg_name').value)field('reg_name').value=(name||'').replace(/\.[^.]+$/,'');details.scrollIntoView({block:'start'});field('reg_name').focus();}
  return {read,write,refreshModels,tables,ranges,showRegistration,resources:maximum=>{workerMaximum=maximum;}};
}
