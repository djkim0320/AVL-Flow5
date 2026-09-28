"""Persistent job records, one isolated solver process tree at a time."""
from pathlib import Path
import json
import os
import re
import subprocess
import sys
import threading
import time
import uuid

HERE=Path(__file__).resolve().parent
TERMINAL={'completed','partial','failed','cancelled','interrupted'}


def write_json(path,value):
    path=Path(path);temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf8')
    os.replace(temp,path)


def stop_summary(folder, state, message):
    path=Path(folder)/'simulation/summary.json'
    if path.is_file():
        value=json.loads(path.read_text('utf8'))
        if value.get('status')=='running':
            value.update(status='cancelled' if state=='cancelled' else 'failed',message=message)
            write_json(path,value)


class ProcessTree:
    """Windows job closes descendants on cancel or UI server termination."""
    def __init__(self,process):
        import ctypes
        from ctypes import wintypes as w
        self.kernel=ctypes.WinDLL('kernel32',use_last_error=True);k=self.kernel
        class Basic(ctypes.Structure):
            _fields_=[('per_process',ctypes.c_int64),('per_job',ctypes.c_int64),('flags',w.DWORD),
                      ('min_ws',ctypes.c_size_t),('max_ws',ctypes.c_size_t),('active',w.DWORD),
                      ('affinity',ctypes.c_size_t),('priority',w.DWORD),('scheduling',w.DWORD)]
        class IO(ctypes.Structure):_fields_=[(n,ctypes.c_uint64) for n in ('read','write','other','rb','wb','ob')]
        class Extended(ctypes.Structure):
            _fields_=[('basic',Basic),('io',IO),('process_memory',ctypes.c_size_t),('job_memory',ctypes.c_size_t),
                      ('peak_process',ctypes.c_size_t),('peak_job',ctypes.c_size_t)]
        k.CreateJobObjectW.argtypes=[ctypes.c_void_p,w.LPCWSTR];k.CreateJobObjectW.restype=w.HANDLE
        k.SetInformationJobObject.argtypes=[w.HANDLE,ctypes.c_int,ctypes.c_void_p,w.DWORD]
        k.AssignProcessToJobObject.argtypes=[w.HANDLE,w.HANDLE];k.CloseHandle.argtypes=[w.HANDLE]
        self.handle=k.CreateJobObjectW(None,None)
        if not self.handle:raise ctypes.WinError(ctypes.get_last_error())
        info=Extended();info.basic.flags=0x2000 # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not k.SetInformationJobObject(self.handle,9,ctypes.byref(info),ctypes.sizeof(info)) or not k.AssignProcessToJobObject(self.handle,int(process._handle)):
            error=ctypes.WinError(ctypes.get_last_error());self.close();raise error

    def close(self):
        if self.handle:self.kernel.CloseHandle(self.handle);self.handle=None


class JobManager:
    def __init__(self,root=None):
        self.root=Path(root or HERE/'data/analysis');self.root.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock();self.live={}
        for p in self.root.glob('*/job.json'):
            value=json.loads(p.read_text('utf8'))
            if value['state'] not in TERMINAL:
                value.update(state='interrupted',message='서버 재시작으로 중단됐습니다. 새 계산을 시작하세요.',finished=time.time())
                write_json(p,value)
                stop_summary(p.parent,'interrupted',value['message'])

    def folder(self,job_id):
        if not re.fullmatch('[0-9a-f]{32}',job_id):raise ValueError('잘못된 해석 ID입니다.')
        p=self.root/job_id
        if not (p/'job.json').is_file():raise FileNotFoundError('해석 기록이 없습니다.')
        return p

    def start(self,project,prepared):
        from analysis_bridge import stage_meshes
        with self.lock:
            if any(p.poll() is None for p,_,_ in self.live.values()):
                raise ValueError('이미 해석 중입니다. 완료하거나 중단한 뒤 새 계산을 시작하세요.')
            job_id=uuid.uuid4().hex;folder=self.root/job_id;folder.mkdir()
            write_json(folder/'project.json',project)
            stage_meshes(prepared,project,folder)
            write_json(folder/'request.json',prepared)
            record=dict(id=job_id,state='queued',stage='starting',created=time.time(),task=prepared['settings']['task'],
                        phase=prepared['phase'],backend=prepared['settings']['backend'],duration_s=prepared['duration_s'],
                        message='해석 프로세스를 시작합니다.',warnings=prepared['warnings'])
            write_json(folder/'job.json',record)
            log=(folder/'solver.log').open('wb')
            p=None;tree=None
            try:
                # Gate input ensures no worker can spawn before it is assigned to its job.
                p=subprocess.Popen([sys.executable,str(HERE/'analysis_worker.py'),str(folder)],cwd=HERE.parent,
                                   stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
                tree=ProcessTree(p);self.live[job_id]=(p,tree,log)
                p.stdin.write(b'go\n');p.stdin.close()
                threading.Thread(target=self._watch,args=(job_id,),daemon=True).start()
            except Exception as exc:
                if tree:tree.close()
                if p and p.poll() is None:p.kill();p.wait()
                log.close();record.update(state='failed',message=str(exc));write_json(folder/'job.json',record)
                raise
            return self.status(job_id)

    def _watch(self,job_id):
        p,tree,log=self.live[job_id];code=p.wait()
        with self.lock:
            tree.close();log.close()
            path=self.root/job_id/'job.json';r=json.loads(path.read_text('utf8'))
            if r['state'] not in TERMINAL:
                r.update(state='failed',message=f'해석 프로세스가 종료됐습니다 ({code}). 로그를 확인하세요.',finished=time.time())
                write_json(path,r)
            if r['state'] in ('failed','cancelled','interrupted'):stop_summary(path.parent,r['state'],r['message'])

    def status(self,job_id):
        folder=self.folder(job_id);r=json.loads((folder/'job.json').read_text('utf8'))
        r['elapsed_s']=max(0,(r.get('finished') or time.time())-r['created'])
        progress=folder/'simulation/accepted_progress.json'
        if progress.is_file():
            try:r['progress']=json.loads(progress.read_text('utf8'))
            except json.JSONDecodeError:pass # A read can overlap the solver's progress write.
        r['files']={}
        for name,label in [('report.html','결과 보고서'),('solver.log','계산 로그'),('request.json','해석 입력'),
                           ('project.json','실행 당시 배치'),('trim.json','트림'),('modes.csv','고유모드 CSV'),
                           ('stability_quality.json','안정성 판정 근거'),
                           ('aero_database.npz','새 공력표'),('aero_metadata.json','공력 출처·비교'),('simulation/timeseries.csv','시계열 CSV'),
                           ('simulation/accepted_checkpoint.npz','마지막 수용 상태'),
                           ('simulation/summary.json','응답 요약'),('history.html','응답 그래프')]:
            if (folder/name).is_file():r['files'][label]=f'/analysis-files/{job_id}/{name}'
        if (folder/'replay.json').is_file():r['files']['3D 결과 재생']=f'/analysis-replay.html?job={job_id}'
        return r

    def list(self):
        folders=sorted(self.root.glob('*/job.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:30]
        return [self.status(p.parent.name) for p in folders]

    def cancel(self,job_id):
        with self.lock:
            folder=self.folder(job_id);r=json.loads((folder/'job.json').read_text('utf8'))
            live=self.live.get(job_id)
            if live and live[0].poll() is None:
                live[1].close();live[0].wait(timeout=15)
                # All descendants have stopped before terminal state is written.
                r=json.loads((folder/'job.json').read_text('utf8'))
                r.update(state='cancelled',message='사용자가 해석을 중단했습니다. 마지막 수용 상태와 로그는 보존됩니다.',finished=time.time())
                write_json(folder/'job.json',r)
                stop_summary(folder,'cancelled',r['message'])
            return self.status(job_id)

    def close(self):
        for job_id in list(self.live):self.cancel(job_id)
