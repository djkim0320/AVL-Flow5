"""Finish one already-running full mission and its runtime-only continuations.

This is a finite post-processing job, not a scheduler. Physical/numerical failures
are never retried as successes. Runtime limits continue unchanged; the diagnosed
short-line command error may use the verified completed settling checkpoint.
"""
from pathlib import Path
import argparse,datetime,html,json,os,subprocess,sys,time
from compute_resources import apply_affinity
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def atomic_json(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');temporary.replace(path)


def completed_report(origin,current):
    v=json.loads((current/'complete/verification.json').read_text(encoding='utf8'))
    if v['status']!='verified_completed_full_mission':raise ValueError('A short window cannot satisfy the full mission')
    relative=Path(os.path.relpath(current,origin)).as_posix()
    prefix='' if relative=='.' else relative+'/'
    rows=[('계산 범위',f"{v['start_s']:.3f}–{v['end_s']:.3f}초"),
          ('와이어 완전 전개',f"{v['mission_checks']['full_payout_m']:.3f}m · 날개폭의 1.5배"),
          ('완전 전개 유지',f"{v['mission_checks']['full_payout_hold_s']:.3f}초"),
          ('포획 시각',f"{v['capture_time_s']:.3f}초"),('최종 문 각도',f"{v['final_door_deg']:.1f}°"),
          ('고도 범위',f"{v['altitude_range_m'][0]:.3f}–{v['altitude_range_m'][1]:.3f}m"),
          ('속도 범위',f"{v['airspeed_range_m_s'][0]:.3f}–{v['airspeed_range_m_s'][1]:.3f}m/s"),
          ('최대 장력',f"{v['max_tension_N']:.3f}N"),('최대 접촉력',f"{v['max_contact_N']:.3f}N · 접촉 물성 미검증"),
          ('최대 윈치 토크',f"{v['max_winch_torque_Nm']:.4f}N·m"),('최대 윈치 기계 출력',f"{v['max_winch_power_W']:.3f}W"),
          ('CAD 검사',f"검사한 {v['CAD_checked_poses']}개 자세에서 겹침 0건"),
          ('설정 한도', '초과 항목 있음' if v['configured_limit_violations'] else '입력된 한도 내')]
    cells=''.join('<tr><th>'+html.escape(k)+'</th><td>'+html.escape(value)+'</td></tr>' for k,value in rows)
    links=' · '.join(f'<a href="{html.escape(prefix+"complete/"+file)}">{label}</a>' for file,label in
        [('H1_replay.html','계산 궤적 재생'),('history.html','결과 그래프'),('verification.json','검증 기록'),('timeseries.csv','시계열 CSV')])
    page=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>270° 도어 전체 해석 결과</title>
<style>body{{font-family:system-ui,sans-serif;max-width:960px;margin:32px auto;padding:24px;color:#23353e;line-height:1.8}}table{{border-collapse:collapse;width:100%}}th,td{{border-bottom:1px solid #dae3e7;padding:10px;text-align:left}}a{{color:#136e9a}}p{{max-width:80ch}}</style>
<h1>270° 도어 · 전체 임무 해석</h1><p>수납부터 전개, 60초 견인, 회수, 포획, 문 닫힘까지 계산했습니다. 실제 flow5 공력표와 6자유도 결합 운동 모델을 사용했습니다.</p>
<p>{links}</p><table>{cells}</table>
<p>가정한 시험 기체의 계산 결과입니다. 견인선 분할 수와 시간 간격에 따른 수렴성, 실제 비행·접촉 시험과의 일치는 아직 검증하지 않았습니다. 문 주변 유동, 서보 응답, 파손·응력 해석은 포함하지 않습니다. CAD 검사는 선택된 자세를 대상으로 합니다.</p></html>'''
    (origin/'report.html').write_text(page,encoding='utf8')


def main():
    apply_affinity()
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
    origin=args.run.resolve();current=origin;continuation=None;monitor=None;logs=[]
    flags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0
    for filename,value in [('continuation.json',{'progress_href':None}),('completion_report.json',{'report_href':None})]:
        if not (origin/filename).exists():atomic_json(origin/filename,value)
    atomic_json(origin/'completion_job.json',dict(pid=os.getpid(),started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    previous=None
    try:
        while True:
            pipeline=json.loads((current/'pipeline.json').read_text(encoding='utf8'))
            state=dict(status='running',current_run=str(current),pipeline=pipeline,
                       progress_href=Path(os.path.relpath(current/'progress.html',origin)).as_posix())
            if state!=previous:atomic_json(origin/'completion_status.json',state);previous=state
            stage=pipeline['stage']
            if stage not in ('completed','partial','failed'):
                if continuation is not None and continuation.poll() is not None:
                    raise RuntimeError('Continuation process ended before a final pipeline status')
                time.sleep(10);continue
            if continuation is not None and continuation.wait(timeout=30)!=0:raise RuntimeError('Continuation process failed')
            if stage=='completed':
                subprocess.run([sys.executable,str(HERE/'verify_underbody_delivery.py'),str(current)],cwd=ROOT,check=True)
                completed_report(origin,current)
                atomic_json(origin/'completion_status.json',dict(status='completed',current_run=str(current),report_href='report.html'))
                atomic_json(origin/'completion_report.json',dict(report_href='report.html'))
                if current!=origin:
                    atomic_json(current/'completion_report.json',dict(report_href=Path(os.path.relpath(origin/'report.html',current)).as_posix()))
                print(json.dumps(dict(status='completed',report=str(origin/'report.html'))),flush=True);break
            short_line_error=(stage=='failed' and pipeline.get('message')=='Not enough line for the requested acceleration and braking')
            if not short_line_error and (stage!='partial' or pipeline.get('status')!='runtime_limit'):
                atomic_json(origin/'completion_status.json',dict(status='stopped',current_run=str(current),pipeline=pipeline,
                    progress_href=Path(os.path.relpath(current/'progress.html',origin)).as_posix(),
                    message='물리 조건 또는 계산 오류로 중단됐습니다. 시간 한도 이외의 중단은 자동 재개하지 않습니다.'))
                print(json.dumps(dict(status='stopped',pipeline=pipeline)),flush=True);break
            index=1
            suffix='cleanup' if short_line_error else 'resume'
            while (origin.parent/f'{origin.name}_{suffix}_{index:02d}').exists():index+=1
            target=origin.parent/f'{origin.name}_{suffix}_{index:02d}'
            log=(origin/f'continuation_{index:02d}.log').open('w',encoding='utf8');logs.append(log)
            helper='finish_short_line_cleanup.py' if short_line_error else 'continue_underbody_mission.py'
            continuation=subprocess.Popen([sys.executable,str(HERE/helper),str(current),'--out',str(target)],
                cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=flags)
            atomic_json(current/'continuation.json',dict(progress_href=Path(os.path.relpath(target/'progress.html',current)).as_posix()))
            # The child publishes its initial pipeline only after validating the
            # archived checkpoint. Never assume an output directory means success.
            while not (target/'pipeline.json').exists():
                if continuation.poll() is not None:raise RuntimeError('Continuation initialization failed; see continuation log')
                time.sleep(1)
            current=target
            monitor_log=(origin/f'continuation_monitor_{index:02d}.log').open('w',encoding='utf8');logs.append(monitor_log)
            monitor=subprocess.Popen([sys.executable,str(HERE/'monitor_underbody_mission.py'),str(current)],cwd=ROOT,
                stdout=monitor_log,stderr=subprocess.STDOUT,creationflags=flags)
    except Exception as exc:
        atomic_json(origin/'completion_status.json',dict(status='failed',current_run=str(current),message=str(exc)));raise
    finally:
        for log in logs:log.close()


if __name__=='__main__':main()
