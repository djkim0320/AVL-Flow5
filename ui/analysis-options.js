// Preserve missing saved choices: only an explicit user choice may replace them.
export function tableOptions(catalog,backend,selected){
  const rows=[{id:'new',label:'실제 해석기로 새 계산'}];
  for(const r of catalog.tables||[])if(r.backend===backend)rows.push({id:r.id,label:`${r.backend} · ${r.speed} m/s · ${r.id.slice(0,8)}`});
  const value=selected??catalog.defaults.aero_job;
  const missing=!rows.some(r=>r.id===value);
  if(missing)rows.push({id:value,label:`찾을 수 없는 저장 공력표 · ${String(value).slice(0,8)}`,missing:true});
  return {rows,value,missing};
}

export function workerOptions(maximum,selected){
  if(!Number.isInteger(maximum)||maximum<1)throw new Error('사용 가능한 CPU 수를 확인하지 못했습니다.');
  const rows=Array.from({length:maximum},(_,i)=>({value:i+1,label:`${i+1}개 CPU`})).reverse();
  const missing=!Number.isInteger(Number(selected))||Number(selected)<1||Number(selected)>maximum;
  if(missing)rows.push({value:selected,label:`${selected}개 CPU · 현재 최대 ${maximum}개`});
  return {rows,value:selected,missing};
}

// Missing IDs preserve the user's choice and require explicit registration/selection.
export function modelChoice(rows,saved){
  if(saved?.model_id&&!rows.some(r=>r.id===saved.model_id))return {id:null,message:'이전에 선택한 해석 모델을 사용할 수 없습니다. 배치는 복원했습니다. 이 기체를 등록한 뒤 다시 선택하세요.'};
  if(!rows.length)return {id:null,message:'등록된 해석 모델이 없습니다. 배치 화면에서 기체를 불러온 뒤 기체 정의로 등록하세요.'};
  return {id:saved?.model_id||rows[0].id,message:''};
}
