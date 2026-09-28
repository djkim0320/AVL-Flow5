"""Select saved physical states densely in moving phases, sparsely in long holds."""
import numpy as np


def replay_indices(result):
    t=result.time
    p=result.config.get('mission_profile')
    if p:
        limits=sorted(set([float(t[0]),float(t[-1])]+[float(p[k]) for k in
            ('deployment_complete_s','recovery_start_s','retrieval_complete_s') if t[0]<p[k]<t[-1]]))
        targets=[]
        for a,b in zip(limits[:-1],limits[1:]):
            hold=p['deployment_complete_s']<=a<p['recovery_start_s']
            targets.extend(np.arange(a,b,.5 if hold else .05))
        targets.extend(limits)
    else:
        targets=np.linspace(t[0],t[-1],min(180,len(t)))
    right=np.minimum(np.searchsorted(t,targets),len(t)-1)
    left=np.maximum(right-1,0)
    idx=np.where(abs(t[right]-targets)<abs(t[left]-targets),right,left)
    return np.unique(np.r_[idx,0,len(t)-1,int(result.table.contact_N.argmax()),int(result.table.tension_N.argmax())])
