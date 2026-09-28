"""Registration must not fabricate physical inputs or assume a product model."""
import copy,json,tempfile
from pathlib import Path
import numpy as np
import yaml
from fixtures.aircraft import RegisteredCase,registration,ROOT
import model_registry
from analysis_bridge import catalog,prepare
from dbf_stability.avl import write_mass_file
from dbf_stability.routing import sample_path

class RegistrationChecks(RegisteredCase):
    def test_registration_and_sources(self):
        p=copy.deepcopy(self.project);value=registration(p);p.pop('aircraft_definition')
        saved=model_registry.register(p,value);r=prepare(p,catalog(saved['id'])['defaults'])
        self.assertEqual(r['profile']['door_parts'],[])
        self.assertEqual(r['config']['sensor']['model'],'point_mass')
        value['source_confirmed']=False
        with self.assertRaisesRegex(ValueError,'출처'):model_registry.register(p,value)
        value=registration(self.project);value['files'][0]['text']+='\nAFILE\nmissing.dat\n'
        with self.assertRaisesRegex(ValueError,'참조 파일 누락'):model_registry.register(p,value)

    def test_empty_registry_and_template_are_explicit_errors(self):
        original=model_registry.DIRECTORY
        with tempfile.TemporaryDirectory() as folder:
            try:
                model_registry.DIRECTORY=Path(folder)
                self.assertEqual(model_registry.list_models(),[])
                for model_id in (None,'0'*32):
                    with self.assertRaises(ValueError):catalog(model_id)
                    with self.assertRaises(ValueError):prepare(self.project,{'model_id':model_id})
            finally:model_registry.DIRECTORY=original
        value=registration(self.project)
        template=yaml.safe_load((ROOT/'ui/templates/model_physics_template.yaml').read_text('utf8'))
        value['case_text']=json.dumps(template)
        with self.assertRaisesRegex(ValueError,'빈 설정'):model_registry.register(self.project,value)
        template['template_unfilled']=False;value['case_text']=json.dumps(template)
        with self.assertRaisesRegex(ValueError,'빈 설정'):model_registry.register(self.project,value)
        filled=json.loads(registration(self.project)['case_text']);filled['template_unfilled']=False;filled['field_help']=template['field_help']
        value['case_text']=json.dumps(filled)
        self.assertIn('id',model_registry.register(self.project,value))

    def test_guide_sampling_and_invalid_time_inputs(self):
        np.testing.assert_allclose(sample_path(np.array([[0,0,0],[1,0,0],[1,1,0]]),[.5,1,1.5]),[[.5,0,0],[1,0,0],[1,.5,0]])
        for changes in ({'gust':{'start_s':0,'duration_s':-1,'velocity_ned_m_s':[0,1,0]}},{'controls':{'typo':[[0,0],[1,1]]}}):
            with self.assertRaises(ValueError):prepare(self.project,{**self.settings,**changes})

    def test_point_mass_has_zero_rotational_inertia_in_mass_file(self):
        c=model_registry.load(self.model_id)['config']
        with tempfile.TemporaryDirectory() as folder:
            rows=write_mass_file(c,Path(folder)/'case.mass',stowed=True).read_text().splitlines()
            row=next(r for r in rows if '# stowed sensor' in r)
            self.assertEqual(list(map(float,row.split('#')[0].split()[4:])),[0.]*6)
