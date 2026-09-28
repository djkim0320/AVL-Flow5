"""Declared rear-opening exposure of the material around each cable node."""
import numpy as np

def outside_fraction(x0,x1,exit_x):
    """Exact length fraction of a straight span behind the rear opening plane."""
    a=np.asarray(x0);b=np.asarray(x1);delta=b-a
    crossing=np.divide(exit_x-a,delta,out=np.zeros_like(delta,dtype=float),where=abs(delta)>1e-15)
    return np.where(delta>1e-15,np.clip(crossing,0,1),
        np.where(delta< -1e-15,1-np.clip(crossing,0,1),(a<exit_x).astype(float)))

def nodal_exposure(chain,aircraft_position,body_rotation,exit_x):
    """Length-weight the two half-spans around each material node."""
    local=(chain-aircraft_position)@body_rotation
    middle=(local[:-1]+local[1:])/2
    node=local[1:-1];left=middle[:-1];right=middle[1:]
    l0=np.linalg.norm(node-left,axis=1);l1=np.linalg.norm(right-node,axis=1)
    return (l0*outside_fraction(left[:,0],node[:,0],exit_x)+l1*outside_fraction(node[:,0],right[:,0],exit_x))/np.maximum(l0+l1,1e-15)

def drag(velocity,tangent,density,diameter,length,cd_normal,cd_tangent):
    axial=(velocity*tangent).sum(axis=1)[:,None]*tangent;normal=velocity-axial
    return -.5*density*diameter*length*(cd_normal*np.linalg.norm(normal,axis=1)[:,None]*normal+cd_tangent*np.linalg.norm(axial,axis=1)[:,None]*axial)
