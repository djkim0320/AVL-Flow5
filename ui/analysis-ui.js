import {modelChoice} from './analysis-options.js';
import {initSettings} from './analysis-settings.js';
import {currentScenario} from './analysis-scenarios.js';
import {resultQuality,hybridQuality} from './analysis-quality.js';
import {resultSelection} from './result-selection.js';
const $=id=>document.getElementById(id);
const titles={queued:'시작 대기',running:'계산 중',completed:'계산 완료',partial:'일부 구간 계산',failed:'계산 실패',cancelled:'중단됨',interrupted:'서버 중단'};
const stages={starting:'시작',aero:'공력 준비',trim:'트림',stability:'안정성',simulation:'시간 적분',report:'결과 작성',done:'완료'};
const tasks={sequence:'비행 → 전개 → 유지 → 회수',flight:'전개 비행 · 단독',recovery:'회수 · 단독',aero:'공력표',trim:'트림',stability:'안정성',response:'시간응답',mission:'전개·회수'};
const phases={aircraft_only:'기체 단독',deployed:'완전 전개',recovery:'전개 상태에서 회수',stowed:'질점 고정',mission:'전체 임무'};
async function request(url,value){const response=await fetch(url,value===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(value)});const data=await response.json();if(!response.ok)throw new Error(data.error||'요청을 처리하지 못했습니다.');return data;}

export function initAnalysis({getProject,onChange,openDefinition}){
  const dialog=document.createElement('dialog');dialog.id='analysis-dialog';dialog.setAttribute('aria-labelledby','analysis-title');
  // Full-page analysis view (non-modal dialog under the top bar): section index · form · run/results.
  dialog.innerHTML=`<div class="analysis-shell"><nav class="analysis-nav" aria-label="해석 입력 구역">
  <div class="analysis-nav-head"><h2 id="analysis-title">해석 설정</h2><p>조건을 확인하고 실행합니다. 실행하는 순간의 배치로 계산합니다.</p></div>
  <div class="readiness"><h3>현재 배치</h3><ul id="analysis-readiness"></ul></div>
  <h3 class="nav-title">입력 항목</h3><ol id="analysis-sections" class="section-nav"></ol>
  <button type="button" id="analysis-close" class="back-button"><svg class="icon mirror" aria-hidden="true"><use href="#i-arrow"/></svg>배치 화면으로<kbd>Esc</kbd></button></nav>
  <form id="analysis-form"><p id="analysis-model" class="analysis-note">해석 모델을 확인하는 중…</p>
  <fieldset id="analysis-setup"><legend>해석 종류와 공력</legend>
  <div class="analysis-grid"><label>해석 조건<select name="task"><option value="sequence">비행 시작 → 전개 → 전개 후 비행 → 회수</option><option value="flight">전개 비행 · 안정성 단독 해석</option><option value="recovery">회수만 단독 해석</option></select></label>
  <label>공력 해석기<select name="backend"><option value="avl">MIT AVL</option><option value="flow5">flow5</option><option value="hybrid">AVL + flow5 복합</option></select></label>
  <label>계산 자원<select name="workers"><option value="12">최대 12개 CPU</option><option value="8">최대 8개 CPU</option><option value="4">최대 4개 CPU</option><option value="2">최대 2개 CPU</option><option value="1">1개 CPU</option></select></label></div>
  <label class="check"><input type="checkbox" name="rebuild">저장된 공력표 대신 실제 해석기로 다시 계산</label><p id="aero-source" class="analysis-note"></p></fieldset>
  <fieldset><legend>비행 조건</legend><div class="analysis-grid">
  <label>속도 · m/s<input name="speed" type="number" step="any" required></label><label>고도 · m<input name="altitude" type="number" step="any" required></label><label>공기 밀도 · kg/m³<input name="rho" type="number" step="any" required></label></div><p class="flow-note"><span class="flow-arrow" aria-hidden="true">→</span><span>공기는 기체 CAD의 <b>+X(앞)</b>에서 <b>−X(뒤)</b>로, 위 속도로 흐릅니다. 받음각·승강타는 트림에서 계산합니다. 방향은 배치 화면의 청록색 화살표로 확인하고, 틀리면 기체를 선택해 90° 단위로 보정하세요.</span></p><p class="hint">밀도는 독립 입력입니다. 고도를 바꿔도 자동으로 변경하지 않습니다.</p></fieldset>
  <fieldset><legend>질량과 관성</legend><div class="analysis-grid"><label>기체 질량 · kg<input name="aircraft_mass" type="number" step="any" required></label><p class="hint">질점 질량은 배치 화면에서 설정합니다.</p></div>
  <details><summary>관성·항력과 기준 좌표</summary><p class="hint">CG 기준 FRD 관성 행렬입니다. 질량 변경에 따른 관성은 자동 추정하지 않습니다. 기체는 등록 모델의 CG를 사용합니다. 질점의 관성과 공력은 입력하지 않습니다.</p>
  <label>기체 관성 · kg·m²<textarea name="aircraft_inertia" rows="4" spellcheck="false" required></textarea></label>
  <div class="analysis-grid"><label>기체 추가 항력계수<input name="profile_cd" type="number" step="any" required></label></div></details></fieldset>
  <fieldset><legend>줄 물성</legend><p class="hint">질점 질량·윈치 위치·줄 길이와 직경은 배치 화면에서 가져옵니다. 질점은 형상과 회전이 없으며 중력·줄 장력만 받습니다.</p><div class="analysis-grid"><label>선밀도 · kg/m<input name="density" type="number" step="any" required></label><label>축강성 EA · N<input name="EA" type="number" step="any" required></label><label>감쇠 · N·s/m<input name="damping" type="number" step="any" required></label><label>줄 분할 수<input name="segments" type="number" step="1" required></label></div>
  <details><summary>줄 항력</summary><div class="analysis-grid"><label>수직 항력계수<input name="cd_normal" type="number" step="any" required></label><label>접선 항력계수<input name="cd_tangent" type="number" step="any" required></label></div></details></fieldset>
  <fieldset><legend>비행 순서와 시간</legend><div class="analysis-grid">
  <label data-scenario="sequence">전개 전 비행 시간 · s<input name="preflight" type="number" step="any" min="0.01" max="600" required></label>
  <label data-scenario="sequence">전개 평균 속도 · m/s<input name="payout" type="number" step="any" min="0.001" max="5" required></label>
  <label data-scenario="sequence">전개 후 비행 시간 · s<input name="hold" type="number" step="any" min="0" max="600" required></label>
  <label data-scenario="flight">비행 계산 시간 · s<input name="duration" type="number" step="any" min="0.01" max="600" required></label><label data-scenario="flight">기체 피치 교란 · °<input name="pitch_delta" type="number" step="any" min="-10" max="10" required></label>
  <label data-scenario="sequence recovery">회수 평균 속도 · m/s<input name="recovery" type="number" step="any" min="0.001" max="5" required></label><label data-scenario="sequence recovery">회수 후 남길 줄 · m<input name="recovery_length" type="number" step="any" min="0.001" required></label></div>
  <p id="analysis-scope" class="analysis-note"></p><details><summary>적분·제어 설정</summary><div class="analysis-grid"><label>최대 적분 간격 · s<input name="max_step" type="number" step="any" required></label><label>적분 제한 시간 · s<input name="runtime_limit" type="number" step="1" required></label></div>
  <label class="check"><input name="controller" type="checkbox">고도·속도 제어 사용 · 시간응답에만 적용</label></details></fieldset>
  </form>
  <aside class="analysis-run" aria-label="해석 실행과 결과"><div class="analysis-actions"><button type="button" id="analysis-validate">입력 확인</button><button type="submit" form="analysis-form" class="primary" id="analysis-run">해석 실행</button></div>
  <p class="run-note">실행 버튼을 누른 순간의 배치와 조건으로 계산합니다. 이후 편집 내용은 진행 중인 계산에 반영되지 않습니다.</p>
  <section id="analysis-connection" class="analysis-connection" role="status"><strong id="connection-title">기체와 공력 모델 연결 확인 중</strong><p id="connection-message"></p><button type="button" id="connection-action" hidden></button></section>
  <p id="analysis-error" class="analysis-error" role="alert" hidden></p><p id="analysis-validation" role="status"></p>
  <section class="analysis-results" aria-label="해석 진행 및 결과"><h3>진행 상황과 결과</h3><div id="analysis-job"><p>아직 선택한 해석이 없습니다. 가운데에서 조건을 확인한 뒤 실행하거나, 아래 최근 해석을 선택하세요.</p></div><button id="analysis-cancel" hidden>현재 해석 중단</button><div id="analysis-files" class="analysis-links"></div><h3>최근 해석</h3><div id="analysis-history"></div><p class="analysis-note">계산 완료는 안전성 합격 판정이 아닙니다. 질점 가정·기체 공력·물성과 수치 수렴을 결과에서 확인하세요.</p></section></aside></div>`;
  document.body.append(dialog);
  const form=$('analysis-form');form.inert=true;let catalog=null,pending=null,current=null,timer=null,activityTimer=null,submitting=false,cancelling=false,modelLoading=false,activeId=null,inputRevision=0,connection=null,connectionPending=false,connectionRevision=0;const refreshedTables=new Set();
  const field=name=>form.elements.namedItem(name);
  const defineAircraft=()=>{closeView();openDefinition?.();};
  const advanced=initSettings({form,request,getProject,selectModel,showError,openDefinition:defineAircraft});
  const selection=resultSelection({
    read:id=>request('/api/analysis/jobs/'+id),render:renderJob,
    pending:id=>{current=id;clearTimeout(timer);timer=null;syncButtons();$('analysis-files').replaceChildren();$('analysis-job').textContent=`해석 ${id.slice(0,8)} 결과를 불러오는 중…`;},
    error:e=>{showError('진행 상황을 읽지 못했습니다: '+e.message);clearTimeout(timer);timer=setTimeout(poll,5000);},
  });
  function syncButtons(){
    $('analysis-run').disabled=submitting||modelLoading||!catalog||connectionPending||!connection?.matches_selected||!!activeId;
    $('analysis-validate').disabled=submitting||modelLoading||!catalog||connectionPending||!connection?.matches_selected;
    $('analysis-cancel').hidden=!activeId;$('analysis-cancel').disabled=cancelling;
    $('analysis-cancel').textContent=activeId?`진행 중 해석 중단 · ${activeId.slice(0,8)}`:'현재 해석 중단';
    if(activeId&&activeId!==current){if(!activityTimer)activityTimer=setTimeout(()=>{activityTimer=null;history().catch(e=>{showError(e.message);syncButtons();});},2000);}
    else{clearTimeout(activityTimer);activityTimer=null;}
  }
  syncButtons();
  // Section index: follows the form scroll and flags sections with invalid required inputs.
  const sections=[...form.querySelectorAll(':scope > fieldset')];
  const navLinks=sections.map((section,i)=>{section.id||=`analysis-section-${i+1}`;const li=document.createElement('li'),a=document.createElement('a');a.href='#'+section.id;a.textContent=section.querySelector('legend').textContent;a.onclick=e=>{e.preventDefault();section.scrollIntoView({block:'start',behavior:'smooth'});};li.append(a);$('analysis-sections').append(li);return a;});
  function trackSection(){const top=form.getBoundingClientRect().top+32;let active=0;sections.forEach((s,i)=>{if(s.getBoundingClientRect().top<=top)active=i;});if(form.scrollTop+form.clientHeight>=form.scrollHeight-4)active=sections.length-1;navLinks.forEach((a,i)=>a.setAttribute('aria-current',i===active?'location':'false'));}
  function markInvalid(){sections.forEach((s,i)=>navLinks[i].classList.toggle('invalid',!!s.querySelector(':invalid')));}
  form.addEventListener('invalid',e=>{const details=e.target.closest('details');if(details)details.open=true;showError((e.target.closest('label')?.childNodes[0]?.textContent.trim()||'입력')+': '+e.target.validationMessage);},true);
  form.addEventListener('scroll',trackSection,{passive:true});
  function readiness(){
    let p;try{p=getProject();}catch{return;}
    const o=p.objects||{},rows=[['기체',o.aircraft,'배치됨','없음'],['질점',o.sensor,o.sensor?`${o.sensor.mass_kg} kg`:'추가됨','없음'],['윈치',o.winch,'배치됨','없음'],['줄 연결',p.cable?.attachment,'자동 연결됨','미연결']];
    const items=rows.map(([label,ok,yes,no])=>{const li=document.createElement('li'),state=document.createElement('span');li.dataset.ok=String(!!ok);li.textContent=label;state.textContent=ok?yes:no;li.append(state);return li;});
    if(p.cable?.attachment){const li=document.createElement('li');li.className='readiness-cable';li.textContent=`줄 ${p.cable.length_m} m · 직경 ${+(p.cable.diameter_m*1000).toFixed(3)} mm`;items.push(li);}
    $('analysis-readiness').replaceChildren(...items);
  }
  function setView(analysis){document.body.dataset.view=analysis?'analysis':'editor';const layout=document.querySelector('.layout');if(layout)layout.inert=analysis;for(const tab of document.querySelectorAll('[data-view-tab]'))tab.setAttribute('aria-current',String(tab.dataset.viewTab===document.body.dataset.view));}
  function closeView(event){if(dialog.open)dialog.close();setView(false);if(event?.currentTarget?.id!=='view-editor')$('open-analysis')?.focus();}
  dialog.addEventListener('close',()=>setView(false));
  dialog.addEventListener('keydown',e=>{if(e.key==='Escape'&&e.target.tagName!=='SELECT'){e.preventDefault();closeView();}});
  document.getElementById('view-editor')?.addEventListener('click',closeView);
  function showError(message){$('analysis-error').textContent=message;$('analysis-error').hidden=!message;if(message)$('analysis-error').scrollIntoView({block:'nearest'});}
  async function checkConnection(){
    const revision=++connectionRevision,model=field('model_id').value;
    connection=null;connectionPending=true;syncButtons();
    $('analysis-connection').dataset.state='checking';$('connection-title').textContent='기체와 공력 모델 연결 확인 중';$('connection-message').textContent='';$('connection-action').hidden=true;
    try{
      const result=await request('/api/models/match',{project:getProject(),model_id:model});
      if(revision!==connectionRevision)return;connection=result;
      $('analysis-connection').dataset.state=result.matches_selected?'ready':'blocked';
      $('connection-title').textContent=result.matches_selected?'기체 형상 연결됨':result.status==='no_aircraft'?'기체를 먼저 배치하세요':result.status==='wrong_selection'?'현재 기체와 다른 공력 모델을 선택했습니다':'현재 기체의 해석 자료가 등록되지 않았습니다';
      $('connection-message').textContent=result.matches_selected?`${result.aircraft_name} ↔ ${result.selected.name}. 물성과 비행 조건은 입력 확인에서 검사합니다.`:result.status==='no_aircraft'?'배치 화면에서 기체 파일을 불러오세요.':result.status==='wrong_selection'?`${result.aircraft_name}에 맞는 등록 모델: ${result.matches.map(row=>row.name).join(', ')}. 등록 모델에서 선택하세요.`:`${result.aircraft_name}의 3D 형상은 불러왔지만 공력 모델은 아직 없습니다. 현재 선택한 ${result.selected?.name||'모델'}의 공력을 사용할 수 없습니다. 기체 정의에서 부품·단면·CG·관성을 지정하고 등록하세요. 수정한 정의도 새 버전 등록이 필요합니다.`;
      const action=$('connection-action');action.hidden=result.matches_selected||result.status==='no_aircraft';action.textContent=result.status==='wrong_selection'?'맞는 등록 모델 선택':'기체 정의 · 부품 / CG / 공력 면';action.onclick=()=>{if(result.status==='wrong_selection'){field('model_id').scrollIntoView({block:'center'});field('model_id').focus();}else defineAircraft();};
    }catch(e){if(revision!==connectionRevision)return;$('analysis-connection').dataset.state='blocked';$('connection-title').textContent='기체 연결을 확인하지 못했습니다';$('connection-message').textContent=e.message;const action=$('connection-action');action.hidden=false;action.textContent='연결 다시 확인';action.onclick=checkConnection;}
    finally{if(revision===connectionRevision){connectionPending=false;syncButtons();}}
  }
  function writeSettings(value){for(const [key,v] of Object.entries(currentScenario(value)))advanced.write(key,v);advanced.tables(catalog,field('backend').value,value.aero_job||field('aero_job').value);update();}
  async function restore(value){
    inputRevision++;
    if(!value){++modelRevision;++connectionRevision;connection=null;connectionPending=false;catalog=null;pending=null;modelLoading=false;form.inert=true;syncButtons();return;}
    if(!catalog&&!modelLoading&&!dialog.open){pending=structuredClone(value);return;}
    if(!catalog||modelLoading||value.model_id&&value.model_id!==catalog.defaults.model_id){const rows=await advanced.refreshModels(),choice=modelChoice(rows,value);if(!choice.id){emptyModel(choice.message,value);return;}await selectModel(choice.id,value);return;}
    ++modelRevision;writeSettings(value);await checkConnection();
  }
  function snapshot(){if(modelLoading)throw new Error('해석 모델을 불러오는 중입니다. 완료 후 저장하거나 실행하세요.');if(!catalog)return pending||null;const out={};for(const [key,base] of Object.entries(catalog.defaults))out[key]=advanced.read(key,base);for(const key of ['sensor_mass','sensor_cd','sensor_inertia','sensor_yaw_delta'])delete out[key];return currentScenario(out);}
  function update(){if(!catalog)return;const task=field('task').value,b=field('backend').value,db=catalog.databases[b];
    for(const element of form.querySelectorAll('[data-scenario]')){element.hidden=!element.dataset.scenario.split(' ').includes(task);for(const input of element.querySelectorAll('input'))input.disabled=element.hidden;}
    const selected=field('aero_job').value;
    advanced.ranges.enable(field('rebuild').checked||selected==='new');advanced.ranges.backend(b);$('hybrid-sources').hidden=b!=='hybrid';
    let gridDescription='입력한 공력 범위의 모든 조합';try{const grid=advanced.read('aero_grid',{});gridDescription=Object.values(grid).reduce((n,a)=>n*a.length,1)+'조건';}catch{}
    const saved=catalog.tables.find(t=>t.id===selected);
    $('aero-source').textContent=field('rebuild').checked||selected==='new'?`실제 해석기로 ${gridDescription}을 새 계산합니다. 원본 입출력과 실행 버전을 보존합니다.`:saved?`${saved.solver} · ${saved.speed} m/s · 저장 공력표 ${selected.slice(0,8)} · 형상·해석기·속도를 검사합니다.`:'선택한 저장 공력표를 찾을 수 없습니다. 표를 선택하거나 새로 계산하세요.';
    if(b==='hybrid'){const c=advanced.read('aero_hybrid',{});$('aero-source').textContent+=` 정적 계수: ${c.coeff} · 조종: ${c.controls} · 회전율: ${c.rates}. flow5 VLM2 · 동체 제외 · 계산 속도 ${saved&&selected!=='new'&&!field('rebuild').checked?saved.speed:field('speed').value} m/s. 서로 다른 해석기 조합입니다.`;}
    const length=getProject().cable?.length_m,target=Number(field('recovery_length').value),speed=Number(field('recovery').value),seconds=(length-target)/speed;
    $('analysis-scope').textContent=task==='recovery'?`완전히 전개된 비행 평형에서 회수를 시작합니다. ${Number.isFinite(seconds)&&seconds>0?`예상 회수 시간 ${seconds.toFixed(2)}초. `:''}시작과 끝에서 천천히 움직이며 최대 속도는 평균의 약 1.5배입니다. 문은 현재 열린 상태를 유지하고, 지정한 줄 길이에서 계산을 마칩니다. 포획·문 닫힘은 계산하지 않습니다.`:'줄이 완전히 전개된 비행 평형, 고정 길이에서의 안정성, 지정 시간의 움직임을 함께 계산합니다. 고유값에는 제어를 적용하지 않으며, 선택한 제어·돌풍·조종 입력은 시간응답에만 적용합니다.';
    if(task==='sequence'){
      const before=Number(field('preflight').value),hold=Number(field('hold').value),deploy=(length-Math.min(.02,length*.05))/Number(field('payout').value),end=before+deploy+hold+seconds;
      const valid=Number.isFinite(end)&&before>0&&hold>=0&&deploy>0&&seconds>0;
      $('analysis-scope').textContent=`비행 시작 → 전개 → 전개 후 비행 → 회수. ${valid?`전개 시작 ${before.toFixed(2)}초 · 전개 완료 ${(before+deploy).toFixed(2)}초 · 회수 시작 ${(before+deploy+hold).toFixed(2)}초 · 종료 ${end.toFixed(2)}초. `:''}입력한 고도·속도로 비행 중인 수납 상태에서 시작하며, 구간 사이 위치·속도는 이어집니다. 전개·회수 속도는 평균값이며 최대값은 약 1.5배입니다. 이륙·포획·문 닫힘은 계산하지 않습니다.`;
    }
  }
  function files(job){const box=$('analysis-files');box.replaceChildren();for(const [label,url] of Object.entries(job.files||{})){const a=document.createElement('a');a.href=url;a.textContent=label;a.target='_blank';a.rel='noopener';box.append(a);}}
  function renderJob(job){current=job.id;localStorage.setItem('dbf-analysis-job',current);const box=$('analysis-job');box.replaceChildren();box.dataset.state=job.state;const heading=document.createElement('h4');heading.textContent=['queued','running'].includes(job.state)?`${titles[job.state]} · ${stages[job.stage]||job.stage}`:titles[job.state]||job.state;const message=document.createElement('p');message.textContent=job.message;box.append(heading,message);const timing=document.createElement('p');timing.textContent=`${job.backend.toUpperCase()} · ${tasks[job.task]||job.task} · ${phases[job.phase]||job.phase} · 경과 ${Math.round(job.elapsed_s)}초 · 실행 ${job.id.slice(0,8)}`;box.append(timing);
    if(job.progress){const p=document.createElement('p');p.textContent=`수용된 계산 시간 ${job.progress.time_s.toFixed(3)} / ${job.progress.requested_end_s.toFixed(3)}초`;box.append(p);const meter=document.createElement('progress');meter.max=job.progress.requested_end_s;meter.value=job.progress.time_s;box.append(meter);}
    if(job.result){const dl=document.createElement('dl');dl.className='analysis-summary';const labels={alpha_deg:'트림 받음각 · °',elevator_deg:'승강타 · °',thrust_N:'추력 · N',max_real_eigenvalue_1_s:'가장 큰 성장률 · 1/s',growth_time_s:'진폭이 e배 커지는 시간 · s',mode_count:'모드 수',max_tension_N:'최대 장력 · N',max_pitch_change_deg:'최대 피치 변화 · °',captured:'포획 여부',final_length_m:'마지막 줄 길이 · m',recovery_completed:'회수 목표 길이 도달',status:'계산 상태'};for(const [key,label] of Object.entries(labels)){if(job.result[key]==null||job.phase==='aircraft_only'&&['max_tension_N','captured'].includes(key)||job.task!=='mission'&&key==='captured')continue;const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=label;const v=job.result[key];dd.textContent=typeof v==='number'?(Number.isInteger(v)?String(v):v.toPrecision(5)):typeof v==='boolean'?(v?'예':'아니오'):v;dl.append(dt,dd);}box.append(dl);}
    if(job.result){const quality=resultQuality(job.result),panel=document.createElement('section');panel.className='analysis-quality';panel.dataset.stability=job.result.stability_status||(job.result.unstable===true?'unstable_detected':'');panel.setAttribute('aria-label','결과 해석 범위');const label=document.createElement('strong');label.textContent=quality.stability||'계산 결과의 검증 상태';const convergence=document.createElement('p');convergence.textContent=quality.convergence;const note=document.createElement('p');note.textContent=quality.note;panel.append(label,convergence,note);const aero=hybridQuality(job.result.aero_quality);if(aero){const source=document.createElement('p'),comparison=document.createElement('p'),table=document.createElement('table');source.textContent=aero.source;comparison.textContent=aero.note;const head=document.createElement('tr');for(const text of ['기울기 / °','AVL','flow5','차이']){const th=document.createElement('th');th.textContent=text;head.append(th);}table.append(head);for(const row of aero.rows){const tr=document.createElement('tr');for(const value of [row.name,row.avl.toPrecision(4),row.flow5.toPrecision(4),row.difference+(row.warning?' · 확인 필요':'')]){const td=document.createElement('td');td.textContent=value;tr.append(td);}table.append(tr);}panel.append(source,comparison,table);}box.append(panel);}
    const running=['queued','running'].includes(job.state);if(running)activeId=job.id;else if(activeId===job.id)activeId=null;$('analysis-cancel').hidden=!running;syncButtons();files(job);for(const button of $('analysis-history').querySelectorAll('[data-job]'))button.setAttribute('aria-pressed',String(button.dataset.job===job.id));const historyButton=$('analysis-history').querySelector(`[data-job="${job.id}"]`);if(historyButton){historyButton.textContent=`${job.id.slice(0,8)} · ${job.backend} · ${tasks[job.task]||job.task} · ${titles[job.state]||job.state}`;historyButton.dataset.state=job.state;}clearTimeout(timer);timer=null;if(running)timer=setTimeout(poll,2000);
    refreshAeroTables(job);
  }
  function refreshAeroTables(job){
    if(!['running','queued'].includes(job.state)&&job.files?.['새 공력표']&&catalog&&!refreshedTables.has(job.id)){
      refreshedTables.add(job.id);const model=catalog.defaults.model_id,revision=modelRevision;
      request('/api/analysis/catalog?model='+encodeURIComponent(model)).then(next=>{if(modelRevision!==revision||catalog?.defaults.model_id!==model)return;catalog.tables=next.tables;advanced.tables(catalog,field('backend').value,field('aero_job').value);update();}).catch(e=>{refreshedTables.delete(job.id);if(modelRevision===revision)showError('새 공력표 목록을 읽지 못했습니다: '+e.message);});
    }
  }
  function poll(){return selection.refresh();}
  let historyRevision=0;
  async function history(){
    const revision=++historyRevision,isCurrent=selection.capture(),previousActive=activeId;
    const jobs=await request('/api/analysis/jobs');if(revision!==historyRevision)return;
    const box=$('analysis-history');box.replaceChildren();
    for(const job of jobs){const b=document.createElement('button');b.type='button';b.textContent=`${job.id.slice(0,8)} · ${job.backend} · ${tasks[job.task]||job.task} · ${titles[job.state]||job.state}`;b.onclick=()=>selection.select(job.id);b.dataset.job=job.id;b.dataset.state=job.state;b.setAttribute('aria-pressed',String(job.id===current));box.append(b);}
    activeId=jobs.find(j=>['queued','running'].includes(j.state))?.id||null;syncButtons();
    if(previousActive&&previousActive!==activeId){const finished=jobs.find(j=>j.id===previousActive);if(finished)refreshAeroTables(finished);}
    if(!current&&isCurrent()){const remembered=jobs.find(j=>j.id===localStorage.getItem('dbf-analysis-job'));const chosen=jobs.find(j=>j.id===activeId)||remembered;if(chosen)await selection.select(chosen.id,chosen);}
  }
  async function submit(validateOnly){showError('');$('analysis-validation').textContent='';submitting=true;syncButtons();const revision=inputRevision;try{if(!catalog||modelLoading)throw new Error('등록 모델을 불러온 뒤 실행하세요.');const settings=snapshot();onChange?.();const project=getProject();project.analysis=settings;const payload={project,settings},isCurrent=selection.capture();const result=await request(validateOnly?'/api/analysis/validate':'/api/analysis/jobs',payload);if(validateOnly){$('analysis-validation').textContent=revision!==inputRevision?'검사 중 입력이 변경됐습니다. 현재 조건으로 다시 확인하세요.':'입력을 확인했습니다. '+(settings.task==='recovery'?`회수 시간 ${result.duration_s.toFixed(2)}초. `:'')+result.warnings.join(' ');$('analysis-validation').scrollIntoView({block:'nearest'});}else{activeId=result.id;if(isCurrent())await selection.select(result.id,result);await history();}}catch(e){if(!validateOnly||revision===inputRevision)showError(e.message);}finally{submitting=false;syncButtons();}}
  form.onsubmit=e=>{e.preventDefault();if(connectionPending||!connection?.matches_selected){showError($('connection-message').textContent||'기체와 공력 모델의 연결을 확인하세요.');return;}submit(false);};$('analysis-validate').onclick=()=>{if(form.reportValidity())submit(true);};
  form.addEventListener('change',e=>{inputRevision++;if(e.target.name==='model_id')return;if(e.target.name==='backend'&&catalog)advanced.tables(catalog,field('backend').value);update();$('analysis-validation').textContent='';onChange?.();});
  form.addEventListener('input',e=>{inputRevision++;markInvalid();if(e.target.name?.startsWith('aero_grid.')||['preflight','payout','hold','recovery','recovery_length'].includes(e.target.name))update();$('analysis-validation').textContent='';if(catalog&&!e.target.name?.startsWith('reg_'))onChange?.();});
  $('analysis-close').onclick=closeView;$('analysis-cancel').onclick=async()=>{const id=activeId,isCurrent=selection.capture();if(!id)return;cancelling=true;syncButtons();try{const result=await request('/api/analysis/cancel/'+id,{});if(isCurrent()&&current===id)await selection.select(id,result);await history();}catch(e){showError(e.message);}finally{cancelling=false;syncButtons();}};
  let modelRevision=0;
  async function selectModel(id,saved=null){
    const previousCatalog=catalog;let previous=null;
    if(previousCatalog&&!saved){try{previous={...snapshot(),model_id:previousCatalog.defaults.model_id};}catch(e){field('model_id').value=previousCatalog.defaults.model_id;throw e;}}
    inputRevision++;const revision=++modelRevision;++connectionRevision;connection=null;connectionPending=false;form.inert=true;modelLoading=true;catalog=null;pending=saved||{model_id:id};syncButtons();
    try{
      const next=await request('/api/analysis/catalog?model='+encodeURIComponent(id));if(revision!==modelRevision)return;
      catalog=next;for(const section of form.querySelectorAll(':scope > fieldset'))section.disabled=false;advanced.resources(catalog.max_workers);advanced.tables(catalog,catalog.defaults.backend,catalog.defaults.aero_job);writeSettings(catalog.defaults);if(saved)writeSettings(saved);
      $('analysis-model').textContent=catalog.model+' · '+catalog.scope;
      for(const key of ['aircraft_mass','aircraft_inertia']){field(key).readOnly=!!catalog.definition_managed;field(key).title=catalog.definition_managed?'기체 정의에서 수정한 후 새 버전으로 등록하세요.':'';}
      form.inert=false;modelLoading=false;pending=null;syncButtons();markInvalid();trackSection();onChange?.();await checkConnection();
    }catch(e){
      if(revision!==modelRevision)return;
      modelLoading=false;form.inert=false;
      if(previousCatalog&&previous){catalog=previousCatalog;pending=null;advanced.resources(catalog.max_workers);writeSettings(previous);await checkConnection();}
      else{catalog=null;pending=saved||{model_id:id};}
      syncButtons();
      throw e;
    }
  }
  function emptyModel(message,saved=null){
    catalog=null;pending=saved;modelLoading=false;connection=null;connectionPending=false;form.inert=false;
    [...form.querySelectorAll(':scope > fieldset')].forEach((s,i)=>s.disabled=i>0);
    field('model_id').prepend(new Option('등록 모델을 선택하세요','',true,true));
    $('analysis-model').textContent=message;$('connection-title').textContent='등록 모델 선택 필요';$('connection-message').textContent=message;
    const action=$('connection-action');action.hidden=false;action.textContent='기체 정의 · 부품 / CG / 공력 면';action.onclick=defineAircraft;syncButtons();
  }
  async function open(){if(!dialog.open)dialog.show();setView(true);readiness();trackSection();showError('');try{if(!catalog){const rows=await advanced.refreshModels(),saved=pending,choice=modelChoice(rows,saved);if(choice.id)await selectModel(choice.id,saved);else emptyModel(choice.message,saved);}else await checkConnection();await history();}catch(e){showError(e.message);}}
  return {open,snapshot,restore,showError,useModel:async id=>{await advanced.refreshModels();await selectModel(id);},refreshState:()=>{update();syncButtons();}};
}
