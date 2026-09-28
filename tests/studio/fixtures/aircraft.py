"""Analytic boxes and lifting surfaces. Not a product aircraft or aero substitute."""

import copy, json, tempfile, unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
from dbf_studio import model_registry
from dbf_studio.aircraft_definition import export_aero, generic_case
from dbf_studio.model_registry import register_definition
from dbf_studio.analysis_bridge import catalog


def definition_fixture():
    # Rectangular, symmetric test wing; tail and fin are independently defined.
    vertices = np.array([[x, y, z] for x in [0.0, -0.3] for y in [0.0, 0.8] for z in [-0.015, 0.015]])
    faces = [
        [0, 1, 3],
        [0, 3, 2],
        [4, 6, 7],
        [4, 7, 5],
        [0, 4, 5],
        [0, 5, 1],
        [2, 3, 7],
        [2, 7, 6],
        [0, 2, 6],
        [0, 6, 4],
        [1, 5, 7],
        [1, 7, 3],
    ]
    parts = [
        dict(name=f'CAD {i}', positions=vertices.ravel().tolist(), indices=np.array(faces).ravel().tolist())
        for i in range(3)
    ]
    aircraft = dict(
        parts=parts,
        position=[0.0, 0.0, 0.0],
        quaternion=[0.0, 0.0, 0.0, 1.0],
        source=dict(origin='file_origin', name='generic-test.step'),
    )

    def sec(x, y, z, c):
        return dict(le_m=[x, y, z], chord_m=c, incidence_deg=0.0, span_panels=8, foil=dict(kind='naca4', code='0012'))

    d = dict(
        schema='dbf-aircraft/1',
        name='Generic generated validation aircraft',
        parts=[
            dict(index=i, name=p['name'], role=['main_wing', 'horizontal_tail', 'vertical_tail'][i])
            for i, p in enumerate(parts)
        ],
        mass=dict(
            mode='total',
            mass_kg=2.0,
            cg_m=[-0.085, 0.0, 0.0],
            inertia_kgm2=[[0.18, 0.0, 0.0], [0.0, 0.15, 0.0], [0.0, 0.0, 0.3]],
        ),
        references=dict(area_m2=0.48, chord_m=0.3, span_m=1.6),
        physical=dict(profile_cd=0.03, max_thrust_N=25.0, thrust_point_m=[-0.085, 0.0, 0.0]),
        source='Analytic validation fixture; not a team aircraft',
        source_kind='assumption',
        reviewed=True,
        surfaces=[
            dict(
                name='Wing',
                parts=[0],
                axis='y',
                mirror=True,
                control='aileron',
                hinge_fraction=0.75,
                sections=[sec(0, 0, 0, 0.3), sec(0, 0.8, 0, 0.3)],
            ),
            dict(
                name='Tail',
                parts=[1],
                axis='y',
                mirror=True,
                control='elevator',
                hinge_fraction=0.7,
                sections=[sec(-0.8, 0, 0, 0.18), sec(-0.8, 0.3, 0, 0.18)],
            ),
            dict(
                name='Fin',
                parts=[2],
                axis='z',
                mirror=False,
                control='rudder',
                hinge_fraction=0.7,
                sections=[sec(-0.8, 0, 0, 0.18), sec(-0.8, 0, -0.25, 0.12)],
            ),
        ],
    )
    project = dict(
        schema='dbf-assembly/2',
        units='m',
        axes='FRD',
        aircraft_definition=d,
        objects=dict(
            aircraft=aircraft,
            winch={**copy.deepcopy(aircraft), 'position': [-0.1, 0.0, 0.02]},
            sensor=dict(kind='point_mass', mass_kg=0.12, position=[-0.1, 0.0, 2.42], quaternion=[0, 0, 0, 1]),
        ),
        cable=dict(length_m=2.4, diameter_m=0.001, attachment=dict(kind='point_mass', local_point_m=[0, 0, 0])),
    )
    return project


def registration(project):
    d = project['aircraft_definition']
    a = export_aero(d, project['objects']['aircraft'])
    return dict(
        name='Analytic test aircraft',
        case_text=json.dumps(generic_case(d, project, a)),
        files=a['files'],
        aircraft_cg_local_m=d['mass']['cg_m'],
        sensor_cg_local_m=[0, 0, 0],
        door_parts=[],
        door_reference_deg=0,
        source_confirmed=True,
    )


class RegisteredCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.old = model_registry.DIRECTORY
        model_registry.DIRECTORY = Path(cls.temp.name) / 'models'
        cls.project = definition_fixture()
        cls.model_id = register_definition(cls.project)['id']
        cls.settings = catalog(cls.model_id)['defaults']
        cls.settings.update(
            task='flight',
            phase='deployed',
            segments=4,
            workers=2,
            duration=0.1,
            controls={},
            gust=None,
            controller=False,
        )

    @classmethod
    def tearDownClass(cls):
        model_registry.DIRECTORY = cls.old
        cls.temp.cleanup()
