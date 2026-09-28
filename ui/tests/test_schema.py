import copy,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import validate_project

class SchemaChecks(unittest.TestCase):
    def setUp(self):
        # Exact geometric fixture for schema rejection, never a product model.
        sensor={'position':[0,0,0],'quaternion':[0,0,0,1],
                'parts':[{'positions':[0,0,0,1,0,0,0,1,0],'indices':[0,1,2]}],
                'source':{'origin':'file_origin'}}
        self.project={'schema':'dbf-assembly/1','units':'m','axes':'FRD','objects':{'sensor':sensor,'winch':copy.deepcopy(sensor)},
                      'cable':{'diameter_m':.001,'length_m':2.7,'attachment':None}}
    def test_valid_surface_and_empty_attachment_rejection(self):
        validate_project(self.project)
        for invalid in ({},False,[],{'part':True}):
            self.project['cable']['attachment']=invalid
            with self.assertRaises(ValueError):validate_project(self.project)
    def test_nonfinite_index_and_missing_origin(self):
        p=copy.deepcopy(self.project);p['objects']['sensor']['parts'][0]['indices'][0]=float('nan')
        with self.assertRaises(ValueError):validate_project(p)
        p=copy.deepcopy(self.project);p['objects']['sensor'].pop('source')
        with self.assertRaises(ValueError):validate_project(p)

if __name__=='__main__':unittest.main()
