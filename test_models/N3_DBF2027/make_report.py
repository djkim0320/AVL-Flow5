"""Build the N3 report from saved solver outputs (never invented results)."""
from pathlib import Path
import html,json,shutil,sys
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];FINAL=HERE/'runs/final'

def read(p):return json.loads(Path(p).read_text('utf8'))
def table(d):return d.to_html(index=False,border=0,float_format=lambda x:f'{x:.5g}',escape=True)
def job(b):return ROOT/'ui/data/analysis'/read(HERE/f'ui_{b}_flight_job.json')['id']

def main():
    dims=read(HERE/'dimensions.json');cases=pd.DataFrame(read(FINAL/'mission_cases.json'))
    nominal=cases[(cases.speed_m_s==20)&(cases.profile_cd==.03)]
    aero=read(FINAL/'mesh_assessment.json');h={b:read(FINAL/f'{b}_holdout.json') for b in ('avl','flow5')}
    results={b:read(job(b)/'result.json') for b in ('avl','flow5')}
    counts={}
    for b in results:
        with np.load(job(b)/'aero_database.npz') as d:counts[b]=int(np.prod([len(d[k]) for k in ('alpha','beta','elevator')]))
    native=read(job('flow5')/'aero_metadata.json')['actual_solver_points']
    quality=read(FINAL/'quality_checks.json')
    polar=pd.read_csv(FINAL/'polars.csv');fig=make_subplots(rows=1,cols=3,subplot_titles=['양력계수','유도항력계수','피칭 모멘트계수'])
    for b,color in [('avl','#1260a2'),('flow5','#cc622e')]:
        d=polar[polar.backend==b]
        for i,key in enumerate(['CL','CDi','Cm'],1):
            fig.add_trace(go.Scatter(x=d.alpha_deg,y=d[key],mode='lines+markers',name=b.upper(),legendgroup=b,showlegend=i==1,line_color=color),row=1,col=i)
            fig.update_xaxes(title_text='받음각 °',row=1,col=i)
    fig.update_layout(height=390,template='plotly_white',margin=dict(l=45,r=25,t=60,b=50),legend=dict(orientation='h'))
    plots=fig.to_html(full_html=False,include_plotlyjs=True)
    mesh=pd.read_csv(HERE/'runs/mesh_02/mesh_results.csv');mesh=mesh[(mesh.alpha==0)&(mesh.beta==0)&(mesh.elevator==0)].copy()
    sizes={'coarse':384,'medium':864,'fine':1944,'extra':3456,'ultra':6144,'finest':9600}
    ref=read(HERE/'runs/flow5_refinement_01/results.json')
    for r in ref:
        v=r['rows'][0]
        mesh=pd.concat([mesh,pd.DataFrame([dict(backend=r['backend'],mesh=r['mesh'],CL=v['CL'],CDi=v['CDi'],Cm=v['coeff'][4],Cmq=v['rates'][4][1],Cnr=v['rates'][5][2])])],ignore_index=True)
    mesh['panels']=mesh.mesh.map(sizes);mesh=mesh.sort_values(['backend','panels'])
    mf=make_subplots(rows=1,cols=3,subplot_titles=['격자별 양력','격자별 피치 감쇠','격자별 요 감쇠'])
    for b,color in [('avl','#1260a2'),('flow5','#cc622e')]:
        d=mesh[mesh.backend==b]
        for i,key in enumerate(['CL','Cmq','Cnr'],1):mf.add_trace(go.Scatter(x=d.panels,y=d[key],mode='lines+markers',name=b.upper(),legendgroup=b,showlegend=i==1,line_color=color),row=1,col=i)
    mf.update_xaxes(title_text='패널 수');mf.update_layout(height=350,template='plotly_white',margin=dict(l=45,r=25,t=60,b=50))
    meshplot=mf.to_html(full_html=False,include_plotlyjs=False)
    rules_url=dims['rules_url'];official='https://aiaa.org/get-involved/university-students/dbf/competition-information/rules-faq-qa/'
    rows=[
        ['날개폭 ≤ 1.8288 m (p10)','1.800 m, 여유 28.8 mm','CAD 치수 확인'],
        ['출구→센서 앞끝 ≥ 1.5×날개폭 (p11)','요구 2.700 m / 설정 줄 3.000 m','아래 평형 거리와 실물 앞끝 확인 필요'],
        ['센서 길이 ≥ 152.4 mm, 내부 수납 (p10–11)','220×90×100 mm 공간 예약','실물 센서·자세 안정성 미검증'],
        ['직육면체 운반 상자 (p11–12)','280×130×130 mm 공간 예약','고정·표시·낙하 시험 필요'],
        ['장치 내부 설치·임무 간 유지 (p11)','내부 윈치 / 후방 화물칸 / 270° 문','작동·포획 기구 검증 필요'],
        ['전동·상용 프로펠러·수동 조종 (p13)','고정익 전방 추진 / 13인치 외형 예약 / 자동제어 OFF','실제 부품·수동 비행 시험 필요'],
        ['상부 차단 플러그, 프로펠러 면과 ≥152.4 mm (p14–15)','상부 배치 / 최소 간격 228 mm','치수 확인, 배선 시험 필요'],
        ['추진 전지 ≤100 Wh, 별도 Rx 전지 (p15–17)','추진 66.6 Wh / Rx 4.62 Wh 설계 가정','상용 팩·라벨·퓨즈·BEC 차단 확인 필요'],
        ['55 lb 미만·하중·CG·비행 검증 (p13,27)','아래 질량 가정과 CG 계산','제작 후 측정·표시·하중 시험 필요'],
        ['조명 3모드·도전성 견인선·낙하 시험 (p10,22)','전선 포함 줄 물성 가정, 설치 공간 예약','전기·낙하 기능 미검증'],
    ]
    rules=table(pd.DataFrame(rows,columns=['규정 항목','N3 반영','판정 범위']))
    cols=['backend','mission','total_mass_kg','alpha_deg','elevator_deg','thrust_N','propulsive_power_W','max_real_1_s']
    trims=nominal[cols].rename(columns={'backend':'해석기','mission':'조건','total_mass_kg':'총질량 kg','alpha_deg':'받음각 °','elevator_deg':'승강타 °','thrust_N':'추력 N','propulsive_power_W':'추력×속도 W','max_real_1_s':'최대 실수부 1/s'})
    m3=nominal[nominal.mission=='M3'];distance=float(m3.conservative_exit_to_front_tip_m.min())
    detail=cases[['backend','mission','speed_m_s','profile_cd','alpha_deg','elevator_deg','thrust_N','propulsive_power_W']]
    holdout=[]
    for b,v in h.items():
        err=np.array(v['absolute_error']);rate=np.array(v['rate_absolute_error'])
        holdout.append(dict(solver=b,max_force_error_N=max(abs(np.array(v['force_error_N']))),pitch_moment_error_Nm=v['moment_error_Nm'][1],
            delta_Cm=err[4],delta_Cmq=rate[4,1],delta_Cnr=rate[5,2]))
    recovery_job=read(HERE/'ui_flow5_recovery_job.json');recovery=read(ROOT/'ui/data/analysis'/recovery_job['id']/'result.json')
    recoveryline=f"회수 계산 {recovery.get('status')} · 계산 {recovery.get('duration_s',0):.3f} s · 최종 줄 {recovery.get('final_length_m',float('nan')):.3f} m · 최대 장력 {recovery.get('max_tension_N',float('nan')):.3f} N."
    sm=h['avl']['stability']['static_margin_percent_MAC'];rates=' / '.join(f"{b.upper()} {results[b]['alpha_deg']:.3f}°, {results[b]['elevator_deg']:.3f}°, {results[b]['thrust_N']:.3f} N" for b in results)
    links=''.join(f'<a href="/analysis-files/{job(b).name}/report.html">{b.upper()} 원본 결과</a> <a href="/analysis-replay.html?job={job(b).name}">{b.upper()} 비행 재생</a> ' for b in results)
    style='''body{font:15px/1.7 "Segoe UI","Malgun Gothic",sans-serif;color:#173047;background:#eef3f7;margin:0}main{max-width:1170px;margin:auto;padding:38px 28px 70px}h1{font-size:32px;margin:8px 0}h2{font-size:23px;margin-top:34px}h3{font-size:18px}a{color:#075d95;margin-right:12px}p{max-width:1050px}table{border-collapse:collapse;background:white;width:100%;font-size:13px}th,td{padding:10px;border-bottom:1px solid #d7e2ec;text-align:left}th{background:#e1eaf1}.scroll{overflow:auto}.note{padding:16px 20px;background:#fff2d6;border-left:4px solid #bc781b}.cards{display:flex;gap:14px;flex-wrap:wrap}.card{background:#fff;padding:20px 25px;border-radius:12px;flex:1}.card b{font-size:28px;display:block}iframe{border:0;width:100%;height:590px;background:white;border-radius:12px}small{color:#506478}.tag{font-size:13px;color:#506478}code{overflow-wrap:anywhere}@media(max-width:700px){main{padding:20px 14px}h1{font-size:25px}iframe{height:410px}}'''
    text=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>N3 DBF 신규 기체 공력해석</title><style>{style}</style><main>
<div class="tag">DBF STUDIO · 새 형상 / 실제 AVL 3.52 · flow5 7.57 / 2026-09-26</div>
<h1>N3 기체를 새로 설계하고 공력을 다시 계산했다</h1>
<p>고익 주익, 후방 화물칸, 높인 꼬리 붐, 동체 아래로 접힌 문과 내부 윈치를 새로 구성했다. CAD와 AVL 입력은 같은 단면·설치각을 사용한다. 센서는 요청대로 120 g 질점으로 유지했다.</p>
<div class="cards"><div class="card">날개폭<b>1.80 m</b>면적 0.5552 m² / AR 5.836</div><div class="card">기체 질량 가정<b>2.695 kg</b>질점·줄 포함 M3 2.8195 kg</div><div class="card">새 공력표<b>{counts['avl']} + {counts['flow5']}</b>flow5는 조종면 차분까지 실제 {native}점</div></div>
<p class="note">실행 완료와 실제 기체 검증은 다르다. 무제어 결합 모델에서 양의 실수부를 가진 고유값이 나타났고, flow5의 항력·요 감쇠계수는 격자에 민감하다. 현재 flow5 표는 교차 비교용이며 설계 확정의 근거로 쓰지 않는다. 동체·열린 문·후류·프로펠러 후류·실속·센서 자세·포획 충격은 이번 공력 모델의 검증 범위에 들어가지 않는다.</p>
<h2>새 기체</h2><iframe title="N3 3차원 CAD" src="N3_geometry.html"></iframe>
<p>STEP는 mm, GLB는 m·Y-up으로 저장했다. 계산 내부는 m·FRD(전방·우측·하방)이며 기준 원점은 기체 CG다. 40개 CAD 솔리드의 유효성과 1.80 m 외곽 폭을 확인했다. 상자·센서 공간은 동체 내벽과 바닥 기준으로 확인했으며 움직이는 센서의 간섭 검증을 뜻하지 않는다.</p>
<p><a href="N3_ready.dbf.json" download>새 기체 배치 파일</a><a href="geometry_checks.json">치수 검사</a><a href="dimensions.json">설계 치수·가정</a><a href="cad_revision_audit.json">최종 CAD와 공력 입력 일치 확인</a></p>
<h2>규정 확인</h2><p><a href="{official}">AIAA 공식 규정 페이지</a>에서 연결한 <a href="{rules_url}">2026–27 규정 PDF</a>를 확인했다. 파일명은 Draft이며, 아래 판정은 확인일에 공개된 그 문서 기준이다. 최종판·추가 Q&amp;A가 나오면 다시 대조해야 한다.</p>
<div class="scroll">{rules}</div>
<p>60초 견인은 경쟁 규정이 아닌 기존 시험 설정이다. M3는 5분 창 안에서 전개·조명 전환·회수를 수행하며 착륙 전에 센서를 수납한다. 질점 모델은 센서의 수평 회전 금지와 조명 기능을 증명하지 못한다. [PDF p10,21–22]</p>
<h2>20 m/s 평형과 안정성</h2><p>고도 100 m, 밀도 1.2133 kg/m³, 추가 항력계수 CD₀=0.030 가정이다. M1은 줄을 내부에 감은 무탑재 상태, M2는 센서 120 g와 상자 70 g를 화물칸 중심에 고정한 상태, M3는 줄 3 m를 전개한 질점 견인 상태다. M1·M2는 추가 질량에 맞춰 CG와 관성을 다시 합산했다.</p>
<div class="scroll">{table(trims)}</div><p>추력×속도는 유효 추진 동력이며 배터리 전력·항속시간이 아니다. 프로펠러·모터 효율은 아직 입력하지 않았다. 수치 오차 범위를 벗어난 양의 고유값 실수부는 해당 무제어 평형에서 작은 교란이 성장하는 모드가 있다는 뜻이다.</p>
<p>AVL 직접 실행의 정적 여유는 {sm:.2f}% MAC다. 양의 정적 여유만으로 모든 동적 모드가 안정하다고 판정하지 않았다. M3 트림은 {rates} (받음각, 승강타, 추력)다.</p>
<p>선형화 교란 크기를 10⁻⁴→10⁻⁵→10⁻⁶로 바꿔도 AVL의 지배적인 양의 실수부는 약 +0.00672 s⁻¹로 유지됐다. flow5의 0 근처 모드는 이 간격에 따라 불안정·중립 분류가 달라졌고, 격자 문제도 남아 있으므로 안정하다는 판정을 확정하지 않았다.</p><details><summary>선형화 간격 확인</summary>{table(pd.DataFrame(quality['eigenvalue_step_sensitivity']))}</details>
<p>견인 평형에서 출구–질점 거리를 계산하고 220 mm 센서의 앞끝 오프셋 110 mm를 보수적으로 빼면 최소 {distance:.4f} m다. 2.70 m 기준과 비교할 수 있지만 실물 센서 공력·줄의 측정 물성으로 다시 확인해야 한다.</p>
<h2>새 공력 곡선</h2><p>받음각 −4°~8°, 옆미끄럼각 −4°~4°, 승강타 −8°~8°에서 AVL {counts['avl']}조건, flow5 {counts['flow5']}조건을 새로 계산했다. flow5는 직접 실행과의 보간 오차를 확인한 뒤 운용점 주변 각도 간격을 0.5°까지 줄였다. 아래는 β=0°, 승강타=0°다. 유도항력 곡선에 동체·마찰 항력을 몰래 포함하지 않았다.</p>{plots}
<h2>격자 수렴과 해석기 차이</h2><p>384→864→1944→3456 패널을 비교하고, flow5 중립 조건은 6144·9600 패널까지 추가 확인했다. 전체 표는 3456 패널이다. 1944→3456에서 AVL 양력 최대 변화는 {aero['max_relative_percent']['avl']['CL']:.3f}%, 유도항력 {aero['max_relative_percent']['avl']['CDi']:.3f}%다. flow5 요 감쇠 Cnr는 최대 {aero['max_relative_percent']['flow5']['Cnr']:.2f}% 변했다. 모든 항목이 5% 이내라는 판정을 내리지 않았다. 0에 가까운 Cm의 상대 변화율은 절대 차이와 함께 읽어야 한다.</p>{meshplot}
<p>flow5의 6144→9600 패널 추가 비교에서도 유도항력 변화는 {aero['flow5_6144_to_9600_change_percent']['CDi']:.2f}%, 요 감쇠 변화는 {aero['flow5_6144_to_9600_change_percent']['Cnr']:.2f}%였다. 패널 수를 늘린 것만으로 문제가 해결되지 않았다. 현재 분할 면·접합부 표현을 포함한 flow5 VLM2 모델의 수렴 문제는 남아 있다.</p>
<details><summary>격자별 실제 수치 보기</summary><div class="scroll">{table(mesh[['backend','mesh','panels','CL','CDi','Cm','Cmq','Cnr']])}</div></details>
<p><a href="mesh_assessment.json">수렴 판정 근거</a> AVL의 선형 조종면 처리와 flow5의 실제 후연 회전·분할 면 처리가 달라 두 결과를 동일한 정답으로 취급하지 않았다. <a href="https://flow5.tech/docs/flow5_doc/Validation/Stability.html">flow5 안정미계수 검증 문서</a></p>
<h2>보간 검증과 항력 민감도</h2><p>표에서 얻은 M3 평형 조건을 두 해석기에 직접 다시 입력해 표 보간값과 비교했다. 아래 오차는 표 값−직접 실행값이다.</p><div class="scroll">{table(pd.DataFrame(holdout))}</div>
<p>flow5 피칭 모멘트 보간 오차는 세분화 전 0.07814 N·m에서 세분화 후 {h['flow5']['moment_error_Nm'][1]:.5f} N·m로 줄었다. 이것은 보간 오차 개선이며 앞서 확인한 공간 격자 수렴 문제의 해결을 뜻하지 않는다.</p>
<p>flow5의 대칭 조건에서도 측력계수 약 10⁻⁷ 수준의 잔차가 남았다. 계수를 0으로 바꾸지 않았다. N3 입력에 병진 가속도 허용치 10⁻⁵ m/s², 각가속도 허용치 5×10⁻⁵ rad/s²를 명시했다. 실제 M3 각가속도 잔차는 {results['flow5'].get('max_angular_acceleration_rad_s2',float('nan')):.6g} rad/s²다. 이전 각가속도 기준 10⁻⁵에서는 실패했으며 그 실행도 보존했다. 이 수치 허용은 실물 검증을 뜻하지 않는다.</p>
<p>CD₀=0.020/0.030/0.040으로 필요한 추력을 비교했다. AVL의 15·20·25 m/s 계산은 동일한 비점성·Mach 0 공력계수를 사용한 동압·트림 민감도이며, Reynolds 수에 따른 실속·항력 변화는 검증하지 않았다. flow5 표와 비교 사례의 실제 계산 속도는 20 m/s다.</p><details><summary>속도·탑재·항력 가정별 결과</summary><div class="scroll">{table(detail)}</div></details>
<h2>UI 연결과 시간응답</h2><p>{links}<a href="/analysis-replay.html?job={recovery_job['id']}">새 기체 회수 재생</a></p><p>{recoveryline} 문은 270°로 고정하고 자동제어를 껐다. 회수는 완전 전개 평형에서 시작하며 포획·문 닫힘을 계산하지 않는다.</p>
<p>최종 배치 파일은 전원 부품 위치를 보완한 CAD와 flow5 표에 연결했다. AVL 계산 당시 CAD와 최종 CAD의 공력 입력 파일·질량·관성이 동일함을 해시와 값으로 확인했다. 비공력 부품 위치만 달라진 기록은 CAD 비교 파일에 남겼다.</p>
<h2>사용 범위와 남은 검증</h2><p>두 해석기는 이번 설정에서 날개·꼬리날개의 비점성 공력을 계산한다. 열린 동체와 문, 접합부 박리, 프로펠러 후류, 마찰 항력, 실속을 충분히 해석했다는 뜻은 아니다. <a href="https://web.mit.edu/drela/Public/web/avl/avl_doc.txt">AVL 공식 설명</a><a href="https://flow5.tech/docs/flow5_doc/Analysis/Moments.html">flow5 동체·모멘트 설명</a></p>
<p>질량·관성, CD₀, 줄 강성·항력·감쇠는 가정이다. CATIA 실제 형상, 부품별 질량, 모터·프로펠러 시험값, 줄 물성, 센서 안정성 시험과 풍동·비행 데이터를 확보한 뒤 정량 검증해야 한다. 현 상태는 규정 조건을 반영한 새로운 해석용 시제품이며 제작 승인이나 비행 안전 인증이 아니다.</p>
<p><small>계산 기록: 원본 명령·형상·에어포일·출력·해석기 버전/해시·입력 스냅샷 보존. AVL 60초 고정 제한은 설정값을 적용하도록 수정했다. 실제 프로세스 시간 제한과 평형·선형화 회귀 검사 4개를 통과했다. 새 N3 결과에서는 엄격한 허용치 실패 재현, 명시한 허용치 통과, 공력계수 무수정, 질량 보존도 확인했다.</small></p><a href="quality_checks.json">추가 검사 기록</a>
</main></html>'''
    promotion=read(HERE/'ui_default_aero.json') if (HERE/'ui_default_aero.json').exists() else None
    if promotion:
        banner='<p class="note"><a href="'+html.escape(promotion['report_url'],quote=True)+'">Claude 협업 후 개선 기록</a> · 최종 CAD용 AVL 105조건을 다시 계산해 배치 파일의 기본 공력표로 연결했다. 아래 비교 표는 최초 계산 기록을 유지한다.</p>'
        text=text.replace('<h2>새 기체</h2>',banner+'<h2>새 기체</h2>',1)
        text=text.replace('최종 배치 파일은 전원 부품 위치를 보완한 CAD와 flow5 표에 연결했다.','초기 배치 파일은 전원 부품 위치를 보완한 CAD와 flow5 표에 연결했고, 후속 개선에서 최종 CAD용 AVL 표를 새로 계산해 기본값을 바꿨다.')
    (HERE/'report.html').write_text(text,encoding='utf8')
    dest=ROOT/'ui/data/reports/N3_DBF2027';dest.mkdir(parents=True,exist_ok=True)
    for name in ['report.html','N3_geometry.html','N3_geometry.png','N3_ready.dbf.json','geometry_checks.json','dimensions.json','cad_revision_audit.json']:
        shutil.copy2(HERE/name,dest/name)
    shutil.copy2(FINAL/'mesh_assessment.json',dest/'mesh_assessment.json')
    shutil.copy2(FINAL/'quality_checks.json',dest/'quality_checks.json')
    # Compact machine-readable overview for future work, with source files intact.
    summary=dict(design=dims,mission_cases=read(FINAL/'mission_cases.json'),mesh_assessment=aero,holdouts=h,
        jobs={b:job(b).name for b in results},recovery_job=recovery_job['id'],all_aerodynamics_physically_validated=False)
    if promotion:summary['current_ui_default']=promotion
    (FINAL/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf8')
    print('http://127.0.0.1:8767/data/reports/N3_DBF2027/report.html',flush=True)

if __name__=='__main__':main()
