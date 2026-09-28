"""Finite postprocessing job for the already running mission, not a scheduler."""
from pathlib import Path
import argparse
import ctypes
from ctypes import wintypes
import json
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=HERE/'runs/span_hold_01'


def status(stage,**details):
    data=dict(stage=stage,**details)
    temporary=OUT/'postprocess.tmp'
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
    temporary.replace(OUT/'postprocess.json')
    print(json.dumps(data,ensure_ascii=False),flush=True)


def execute(script,*args):
    subprocess.run([sys.executable,str(HERE/script),*map(str,args)],cwd=ROOT,check=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True);args=p.parse_args()
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.OpenProcess.argtypes=[wintypes.DWORD,wintypes.BOOL,wintypes.DWORD];kernel.OpenProcess.restype=wintypes.HANDLE
    kernel.WaitForSingleObject.argtypes=[wintypes.HANDLE,wintypes.DWORD];kernel.WaitForSingleObject.restype=wintypes.DWORD
    kernel.CloseHandle.argtypes=[wintypes.HANDLE]
    handle=kernel.OpenProcess(0x00100000,False,args.pid)  # SYNCHRONIZE, read/wait only
    status('waiting_for_calculation',worker_pid=args.pid)
    if handle:
        try:
            # Wait for process termination before opening any live checkpoint.
            started=time.monotonic()
            while True:
                code=kernel.WaitForSingleObject(handle,30000)
                if code==0:break
                if code!=258:raise OSError(ctypes.get_last_error(),'Could not wait for calculation process')
                if time.monotonic()-started>3*3600:raise TimeoutError('Calculation did not exit within the bounded three-hour wait')
        finally:kernel.CloseHandle(handle)
    summary=json.loads((OUT/'continuation/summary.json').read_text(encoding='utf8'))
    if summary['status']=='running' or not (OUT/'continuation/states.npz').exists():
        raise RuntimeError('Calculation process exited without a completed/partial trajectory; retained journals require recovery')
    status('assembling_saved_states',calculation_status=summary['status'])
    execute('assemble_span_hold_mission.py')
    status('checking_cad',calculation_status=summary['status'])
    execute('audit_trajectory_collisions.py',OUT/'combined','--workers','6','--time-step','.1')
    status('rendering_results',calculation_status=summary['status'])
    execute('visualize_run.py','--run',OUT/'combined')
    execute('render_span_hold_report.py')
    latest=dict(path=str(OUT),mission_path=str(OUT/'combined'),case=str(ROOT/'examples/h1_normal_r3_flow5_span_hold.yaml'),
        report=str(OUT/'report.html'),normal_mission_complete=bool(summary['status']=='completed' and summary.get('captured')),
        numerically_converged=False)
    (HERE/'latest_run.json').write_text(json.dumps(latest,ensure_ascii=False,indent=2),encoding='utf8')
    status('completed',calculation_status=summary['status'],end_time_s=summary['end_time_s'],report=str(OUT/'report.html'))


if __name__=='__main__':
    try:main()
    except Exception as exc:
        status('failed',message=str(exc))
        raise
