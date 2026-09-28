import json,sys
import numpy as np
from postprocess import HERE,ROOT,folder,AeroDatabase,solve_trim,save

def trace(frame,event,arg):
    if frame.f_code.co_name=='_full_length_trim' and event=='exception':
        local=frame.f_locals
        record={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in local.items() if k in ('error','dy','y','u0')}
        fit=local.get('fit')
        if fit is not None:record.update(fit_x=fit.x.tolist(),fit_fun=fit.fun.tolist(),message=str(fit.message))
        save(HERE/'runs/final/flow5_trim_diagnostic.json',record)
    return trace

if __name__=='__main__':
    d=folder('flow5');c=json.loads((d/'request.json').read_text('utf8'))['config'];db=AeroDatabase(d/'aero_database.npz')
    sys.settrace(trace)
    try:solve_trim(c,db)
    except RuntimeError as e:print(e)
    finally:sys.settrace(None)
    r=json.loads((HERE/'runs/final/flow5_trim_diagnostic.json').read_text('utf8'));dy=np.array(r['dy'])
    print('fit',r['fit_x'],r['fit_fun'],r['message']);print('aircraft accelerations',dy[3:6],dy[10:13]);print('sensor acceleration',dy[16:19]);print('max cable acceleration',abs(dy[26:].reshape(-1,6)[:,3:]).max())
