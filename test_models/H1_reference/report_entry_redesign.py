"""Show actual geometry at one identical saved pose, not a synthetic trajectory."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json
import numpy as np
from dbf_stability import load_case
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation
from dbf_stability.collision import read_binary_stl
from dbf_stability.door import door_box
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]

if __name__=='__main__':
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.collections import LineCollection,PolyCollection
    font_manager.fontManager.addfont('C:/Windows/Fonts/malgun.ttf');plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
    out=HERE/'runs/entry_redesign_report';out.mkdir(exist_ok=True)
    result=load_result(HERE/'runs/underbody_mission_02/complete');old=result.config
    new=load_case(ROOT/'examples/h1_recovery_funnel_v2.yaml')
    idx=np.argmin(abs(result.time-70.2));y=result.states[idx];ra=rotation(y[6:10]);rs=rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3])
    fig,axs=plt.subplots(1,2,figsize=(13,5.6),sharex=True,sharey=True)
    shift=np.asarray(new['aircraft']['cg_shift_from_r3_body_m'])-old['aircraft']['cg_shift_from_r3_body_m']
    for ax,c,offset,title in zip(axs,[old,new],[np.zeros(3),shift],['기존 · 코가 수직 턱에 걸림','변경 · 낮고 넓은 입구에서 경사 통로로']):
        meshdir=ROOT/c['collision']['mesh_directory']
        for path in meshdir.glob('*_FRD_m.stl'):
            name=path.name.removesuffix('_FRD_m.stl')
            if not name.startswith(('fuselage','guide','rear_exit','capture_','recovery_funnel')):continue
            v,f=read_binary_stl(path);v+=offset;lines=[]
            for tri in v[f]:
                points=[]
                for a,b in zip(tri,np.roll(tri,-1,axis=0)):
                    if abs(a[1])<1e-8:points.append(a)
                    if a[1]*b[1]<0:points.append(a-a[1]*(b-a)/(b[1]-a[1]))
                if len(points)>1:lines.append(np.asarray(points)[:,[0,2]]*1000)
            if lines:ax.add_collection(LineCollection(lines,colors='#188b70' if name=='recovery_funnel' else '#74828c',linewidths=1.6))
        sensor_dir=ROOT/old['collision']['mesh_directory']
        for path in sensor_dir.glob('sensor*_FRD_m.stl'):
            if 'tow_point' in path.name:continue
            v,f=read_binary_stl(path);v=(v-old['bay']['stowed_center_m'])@(ra.T@rs).T+center
            ax.add_collection(PolyCollection(v[f][:,:,[0,2]]*1000,facecolors='#dc982b',edgecolors='none',alpha=.20))
        dc,axes,half=door_box(c['bay'],270);points=np.array([[-half[0],0,-half[2]],[half[0],0,-half[2]],[half[0],0,half[2]],[-half[0],0,half[2]]])@axes.T+dc+offset
        ax.fill(points[:,0]*1000,points[:,2]*1000,color='#227cac',label='270° 접힌 도어')
        ax.set_title(title,pad=14,fontsize=13);ax.set_xlim(-820,-360);ax.set_ylim(175,-50);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('기체 x [mm]  ← 후방 · 전방 →')
    axs[0].set_ylabel('기체 z [mm] · 아래쪽이 양수')
    axs[0].annotate('처음 막히는 곳',xy=(-630,72),xytext=(-610,140),arrowprops=dict(arrowstyle='->',color='#a14325'),color='#a14325')
    axs[1].annotate('하단 40mm 확장\n수직 턱 제거',xy=(-628,109),xytext=(-775,153),arrowprops=dict(arrowstyle='->',color='#188b70'),color='#188b70')
    fig.suptitle('동일한 70.2초 센서 자세에서 실제 CAD 단면 비교',fontsize=16,y=.98)
    fig.text(.5,.025,'형상 비교 그림입니다. 변경 후 센서 운동을 계산한 결과가 아니며, 회수 해석은 별도로 진행합니다.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.06,1,.93));fig.savefig(out/'comparison.png',dpi=170)
    record=json.loads((ROOT/new['collision']['mesh_directory']).parent.joinpath('geometry.json').read_text())
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>회수 입구 형상 수정</title>
<style>body{font-family:system-ui,sans-serif;max-width:1100px;margin:35px auto;padding:24px;background:#f6f8f9;color:#213b43;line-height:1.75}h1{font-size:28px}img{width:100%;background:white;border-radius:14px}.cards{display:flex;gap:16px;flex-wrap:wrap}.cards p{flex:1;min-width:190px;background:white;padding:18px;border-radius:12px}.cards strong{display:block;font-size:24px;color:#177765}a{color:#126f9a}#state{padding:20px;background:#e6f0ed;border-radius:12px}</style>
<h1>회수 입구의 수직 턱을 없앴습니다</h1><p>기존 형상에서는 와이어가 출구 바닥에 닿아 꺾인 채 감겼습니다. 센서의 코는 턱에 막혔고, 장력이 쌓인 뒤 갑자기 들어가며 핀이 충돌했습니다.</p>
<div class="cards"><p>출구 크기<strong>80 × 120 mm</strong>기존 56 × 70 mm</p><p>바닥 확장<strong>아래로 40 mm</strong>50 × 50 mm 내부 통로까지 경사 연결</p><p>도어<strong>152 × 80 mm · 270°</strong>문판·힌지 지지부도 함께 변경</p></div>
<img src="comparison.png" alt="같은 센서 자세에서 기존 수직 턱과 새 경사 통로를 비교한 실제 CAD 단면"><p>위 그림은 같은 자세를 놓고 비교한 형상 검사입니다. 변경 후 운동을 재생한 영상이 아닙니다.</p>
<p>도어 0–270°를 1° 간격으로 검사한 271개 자세에서 CAD 겹침과 질의 오류가 없었습니다. 통로 내부로 돌출한 동체 잔여물도 제거했습니다. 가정한 재료 물성으로 질량·무게중심·관성을 갱신했고, flow5 직접 실행으로 변경된 트림의 공력 보간 오차를 확인했습니다.</p>
<p><a href="../funnel_recovery_02/entry_preview_02/H1_replay.html">센서가 새 입구를 통과하는 실제 계산 재생 (69.80–73.06초)</a></p><p id="state">회수 계산 상태를 불러오는 중입니다.</p><p id="links"></p>
<p>계산에는 논리 프로세서 16개 중 12개까지 사용하고 4개는 남깁니다. CAD 검사는 메모리 여유에 따라 최대 10개 작업을 동시에 처리합니다. 입구 통과 구간의 67개 자세 검사는 작업자 7개로 마쳤습니다. 시간 적분 자체가 12개로 나뉘는 것은 아닙니다.</p>
<p><a href="../funnel_mission_01/progress.html">수납부터 다시 계산한 전체 임무 진행</a> · <a href="../funnel_recovery_02/progress.html">회수 구간 비교 진행</a> · <a href="../funnel_door_v2/audit.json">도어 CAD 검사</a> · <a href="../funnel_aero_v2/trim_validation.json">실제 flow5 실행·비교 기록</a> · <a href="../../geometry_variants/normal_r3_recovery_funnel_v2/H1_recovery_funnel_open.step">수정 STEP 모델</a></p>
<p>수정 기체의 건조 질량은 MASSkg입니다. 입구 주변 유동과 접촉 물성, 구조 강도는 아직 시험으로 검증하지 않았습니다. 힘·장력 한도와 포획 조건은 그대로 유지했습니다.</p>
<script>async function update(){try{let p=await(await fetch('../funnel_recovery_02/pipeline.json',{cache:'no-store'})).json();if(p.result_directory){let s=await(await fetch('../funnel_recovery_02/'+p.result_directory+'/summary.json',{cache:'no-store'})).json();document.querySelector('#state').textContent='회수 계산: '+({audit:'CAD 검사 중',completed:'완료',partial:'중단 · 결과 확인 필요',failed:'오류'}[p.stage]||p.stage)+' · 포획 '+(s.captured?'감지':'미완료')+' · 최종 문 각도 '+s.final_door_deg+'°';if(['completed','partial'].includes(p.stage))document.querySelector('#links').innerHTML='<a href="../funnel_recovery_02/'+p.result_directory+'/H1_replay.html">실제 계산 궤적 재생</a>'}else if(p.live_directory){let r=await(await fetch('../funnel_recovery_02/'+p.live_directory+'/accepted_progress.json',{cache:'no-store'})).json();document.querySelector('#state').textContent='회수 구간 계산 중 · '+r.time_s.toFixed(3)+'초까지 저장됨. '+(r.captured?'포획 감지: '+r.capture_time_s.toFixed(3)+'초. 내부 줄 정리와 문 닫힘을 확인하고 있습니다.':'포획·문 닫힘 완료는 아직 확인하지 않았습니다.')+' 센서가 처음 걸리기 전인 69.8초 저장 자세에서 시작한 변경 형상 시험이며, 수납부터 시작한 전체 임무는 별도로 계산합니다.'}}catch(e){document.querySelector('#state').textContent='진행 기록을 아직 읽지 못했습니다.'}}update();setInterval(update,5000)</script></html>'''
    # Retain the inspected partial replay, but follow the actual finalized-state
    # cleanup continuation for live status and the eventual complete result.
    page=page.replace('funnel_mission_01','funnel_mission_02').replace('수납부터 다시 계산한 전체 임무 진행','수납·전개·60초 견인·회수 전 과정 계산 상태')
    page=page.replace('funnel_recovery_02','funnel_recovery_03').replace(
        'funnel_recovery_03/entry_preview_02','funnel_recovery_02/entry_preview_02')
    verified=HERE/'runs/funnel_recovery_03/complete/verification.json'
    if verified.exists():
        v=json.loads(verified.read_text(encoding='utf8'))
        if v['status']!='verified_completed_window':raise ValueError('Unexpected recovery verification scope')
        page=page.replace(
            '<a href="../funnel_recovery_02/entry_preview_02/H1_replay.html">센서가 새 입구를 통과하는 실제 계산 재생 (69.80–73.06초)</a>',
            '<a href="../funnel_recovery_03/complete/H1_replay.html">회수 → 포획 → 문 닫힘 실제 계산 재생</a>')
        evidence=(f"<p>회수 구간 {v['start_s']:.2f}–{v['end_s']:.2f}초 계산을 마쳤습니다. "
                  f"포획 {v['capture_time_s']:.3f}초, 최종 문 각도 {v['final_door_deg']:.0f}°, "
                  f"최대 장력 {v['max_tension_N']:.3f}N. 검사한 CAD 자세 {v['CAD_checked_poses']}개에서 "
                  "겹침과 질의 오류가 없었습니다. 수납부터 시작한 전체 임무 결과와는 구분합니다. "
                  '<a href="../funnel_recovery_03/complete/verification.json">검증 기록</a></p>')
        page=page.replace('<p id="state">',evidence+'<p id="state">')
    (out/'report.html').write_text(page.replace('MASS',f"{new['aircraft']['mass_kg']:.3f}"),encoding='utf8')
    # Correct the already-running progress page without altering its physical inputs.
    progress=HERE/'runs/funnel_recovery_02/progress.html'
    if progress.exists():
        text=progress.read_text(encoding='utf8').replace('형상: 102×56×2mm 문판, 출구 바닥 아래 30mm 힌지, 핀·브래킷·받침 추가.','형상: 입구 80×120mm, 경사 통로와 152×80×2mm 문판, 270° 접힘.')
        progress.write_text(text,encoding='utf8')
    print(out/'report.html')
