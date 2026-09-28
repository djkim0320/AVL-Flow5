"""Opt-in historical CAD checks, outside generic UI acceptance tests.

Run from the repository root with .venv/Scripts/python.exe and this path.
Uses a temporary registration and output directory; never edits saved runs.
"""
from pathlib import Path
import copy
import json
import sys
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'ui'),str(ROOT/'src')]
import model_registry
from server import validate_project
from analysis_bridge import catalog,prepare,stage_meshes
from dbf_stability import load_case,build_aero_database
from dbf_stability.avl import write_mass_file
from dbf_stability.collision import MeshContacts,read_binary_stl
from dbf_stability.door import door_hinge,door_reference_shift
from dbf_stability.math3d import rotation
from dbf_stability.model import CoupledModel


def historical_geometry(c):
    origins={'aircraft':np.zeros(3),'sensor':np.array(c['bay']['stowed_center_m']),
             'winch':np.array(c['aircraft']['tow_point_m'])}
    groups={k:dict(parts=[],source=dict(name='H1 historical CAD',units='m',axes='FRD',origin='file'),
                   position=origins[k].tolist(),quaternion=[0,0,0,1]) for k in origins}
    for path in sorted((ROOT/c['collision']['mesh_directory']).glob('*_FRD_m.stl')):
        name=path.name.removesuffix('_FRD_m.stl')
        if name in ('sensor_tow_point','aircraft_tow_point'):continue
        role='sensor' if name=='sensor_body' or name.startswith('sensor_fin_') else 'winch' if name.startswith('winch_') else 'aircraft'
        vertices,faces=read_binary_stl(path)
        if name=='rear_door_100deg':
            hinge=door_hinge(c['bay'])
            vertices=(vertices+door_reference_shift(c['bay'])-hinge)@Rotation.from_euler('y',170,degrees=True).as_matrix().T+hinge
        groups[role]['parts'].append(dict(name=name,positions=(vertices-origins[role]).ravel().tolist(),indices=faces.ravel().tolist()))
    groups['sensor']['position']=[-1.03,0,.08]
    return groups


class HistoricalRegistrationChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.case=load_case(ROOT/'examples/h1_recovery_funnel_v2.yaml')
        cls.temp=tempfile.TemporaryDirectory();cls.previous=model_registry.DIRECTORY
        model_registry.DIRECTORY=Path(cls.temp.name)/'models'

    @classmethod
    def tearDownClass(cls):
        model_registry.DIRECTORY=cls.previous;cls.temp.cleanup()

    def setUp(self):
        self.project=dict(schema='dbf-assembly/1',units='m',axes='FRD',
            objects=historical_geometry(self.case),cable=dict(length_m=2.7,diameter_m=.001,attachment=None))
        part=self.project['objects']['sensor']['parts'][0]
        vertices=np.array(part['positions']).reshape(-1,3);faces=np.array(part['indices']).reshape(-1,3);hits=[]
        for face,triangle in enumerate(vertices[faces]):
            matrix=np.vstack([np.ones(3),triangle[:,1],triangle[:,2]])
            if abs(np.linalg.det(matrix))<1e-15:continue
            bary=np.linalg.solve(matrix,[1,0,0])
            if bary.min()>=-1e-10:hits.append((float((bary@triangle)[0]),face,bary,bary@triangle))
        _,face,bary,point=max(hits,key=lambda x:x[0])
        self.project['cable']['attachment']=dict(part=0,face=face,barycentric=bary.tolist(),local_point_m=point.tolist())
        validate_project(self.project)
        c=copy.deepcopy(self.case);c['collision']['analytic_conical_stop']={};c['collision']['part_materials']={}
        geometry=ROOT/c['aero']['geometry'];files=[dict(name=geometry.name,text=geometry.read_text('ascii'))]
        lines=files[0]['text'].splitlines()
        for i,line in enumerate(lines):
            if line.strip().split()[:1] in (['AFILE'],['BFILE']):
                name=lines[i+1].strip()
                if not any(f['name']==name for f in files):files.append(dict(name=name,text=(geometry.parent/name).read_text('ascii')))
        self.value=dict(name='H1 explicit historical registration',case_text=json.dumps(c),source_confirmed=True,
            aircraft_cg_local_m=[0,0,0],sensor_cg_local_m=[0,0,0],door_parts=['rear_door_100deg'],door_reference_deg=270,files=files)

    def register(self):
        record=model_registry.register(self.project,self.value)
        settings=catalog(record['id'])['defaults'];settings.update(task='response',phase='deployed',start='scene',workers=2)
        return settings

    def test_cad_cg_offset_and_explicit_manifest(self):
        original=copy.deepcopy(self.project)
        for role,offset in [('aircraft',np.array([1.,.2,-.1])),('sensor',np.array([.2,-.4,.1]))]:
            for part in self.project['objects'][role]['parts']:
                part['positions']=(np.array(part['positions']).reshape(-1,3)+offset).ravel().tolist()
            self.project['objects'][role]['position']=(np.array(self.project['objects'][role]['position'])-offset).tolist()
            self.value[role+'_cg_local_m']=offset.tolist()
            if role=='sensor':self.project['cable']['attachment']['local_point_m']=(np.array(self.project['cable']['attachment']['local_point_m'])+offset).tolist()
        prepared=prepare(self.project,self.register())
        np.testing.assert_allclose(prepared['mapping']['sensor_position_body_m'],original['objects']['sensor']['position'],atol=1e-12)
        with tempfile.TemporaryDirectory() as folder:
            stage_meshes(prepared,self.project,folder);contacts=MeshContacts(prepared['config'])
            try:
                self.assertEqual(len(contacts.sensor),5)
                self.assertEqual(sum(p['door'] for p in contacts.obstacles),1)
                replay=json.loads((Path(folder)/'replay_project.json').read_text('utf8'))
                np.testing.assert_allclose(replay['objects']['sensor']['parts'][0]['positions'],original['objects']['sensor']['parts'][0]['positions'],atol=1e-12)
            finally:contacts.close()

    def test_generated_guide_hole_and_wall(self):
        settings=self.register();settings['mechanism']['guide_points_m']=[[-2.,0,0]]
        self.project['cable']['length_m']=5.;prepared=prepare(self.project,settings)
        with tempfile.TemporaryDirectory() as folder:
            stage_meshes(prepared,self.project,folder);contacts=MeshContacts(prepared['config'])
            try:
                ring=next(p for p in contacts.obstacles if p['name']=='route_ring_1')
                center=ring['vertices'].mean(0);_,_,v=np.linalg.svd(ring['vertices']-center)
                chain=np.array([center-.05*v[-1],center+.05*v[-1]])
                clear=contacts.contacts(np.array([-4.,0,0]),np.eye(3),chain,270)
                self.assertFalse(any(h['fixed']=='route_ring_1' for h in clear))
                offset=(settings['mechanism']['guide_inner_radius_m']+.0002)*v[0]
                hit=contacts.contacts(np.array([-4.,0,0]),np.eye(3),chain+offset,270)
                self.assertTrue(any(h['fixed']=='route_ring_1' for h in hit))
            finally:contacts.close()

    def test_stowed_mass_file_rotates_inertia(self):
        c=copy.deepcopy(self.case);inertia=np.asarray(c['sensor']['inertia_kgm2'])
        q=Rotation.from_euler('y',90,degrees=True).as_quat();c['bay']['stowed_quaternion_wxyz']=np.r_[q[3],q[:3]].tolist()
        with tempfile.TemporaryDirectory() as folder:
            rows=write_mass_file(c,Path(folder)/'case.mass',stowed=True).read_text().splitlines()
            row=next(r for r in rows if '# stowed sensor' in r)
            values=list(map(float,row.split('#')[0].split()))
            np.testing.assert_allclose(values[4:7],np.diag(inertia)[[2,1,0]],atol=1e-14)

    def test_no_moving_door_uses_registered_manifest(self):
        from dbf_stability.contact_integration import separated_from_entire_door_sweep
        self.value['door_parts']=[];prepared=prepare(self.project,self.register())
        with tempfile.TemporaryDirectory() as folder:
            stage_meshes(prepared,self.project,folder)
            c=prepared['config'];c['aero'].update(alpha_deg=[0.,4.],beta_deg=[-4.,4.],elevator_deg=[-4.,4.])
            db=build_aero_database(c,Path(folder)/'aero/aero_database.npz',workers=2)
            model=CoupledModel(prepared['config'],db,phase='deployed')
            try:
                y=model.initial();model.contact_geometry(0,y)
                self.assertTrue(separated_from_entire_door_sweep(model,0,y,.01))
            finally:model.close()


if __name__=='__main__':unittest.main()
