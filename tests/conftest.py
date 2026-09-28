from pathlib import Path
import pytest
from dbf_stability import load_case,AeroDatabase

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def cfg():return load_case(ROOT/'examples/reference.yaml')

@pytest.fixture(scope='session')
def aero():
    path=ROOT/'outputs/aero_database.npz'
    if not path.exists():pytest.skip('Run dbf build-aero with actual AVL first')
    return AeroDatabase(path)


def explicit_modeling_manifest(config, example):
    """Upgrade only the in-memory archived regression input from a named modeling example."""
    import copy
    supplied=load_case(ROOT/'examples'/example)
    assert (ROOT/config['collision']['mesh_directory']).resolve()==(ROOT/supplied['collision']['mesh_directory']).resolve()
    config['collision']['parts_manifest']=copy.deepcopy(supplied['collision']['parts_manifest'])
    return config
