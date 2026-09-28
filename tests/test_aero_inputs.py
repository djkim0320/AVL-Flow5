from pathlib import Path
import copy, shutil
import pytest
from dbf_stability import load_case, build_aero_database
from dbf_stability.aero_inputs import input_fingerprint

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.avl
def test_real_table_rejects_airfoil_edit_and_control_remapping(tmp_path):
    cfg = load_case(ROOT / 'test_models/N3_DBF2027/N3_coarse.yaml')
    source = Path(cfg['aero']['geometry'])
    dest = tmp_path / 'inputs'
    shutil.copytree(source.parent, dest)
    geometry = dest / source.name
    cfg['aero'].update(geometry=str(geometry), alpha_deg=[-1.0, 1.0], beta_deg=[-1.0, 1.0], elevator_deg=[-1.0, 1.0])
    db = build_aero_database(cfg, tmp_path / 'actual_avl.npz', workers=4)
    db.assert_compatible(cfg)
    changed = copy.deepcopy(cfg)
    changed['aero']['elevator_index'] = 3
    with pytest.raises(ValueError, match='control indices'):
        db.assert_compatible(changed)
    foil = dest / 'n3_2412.dat'
    foil.write_text(foil.read_text() + '\n')
    with pytest.raises(ValueError, match='airfoil/body'):
        db.assert_compatible(cfg)
    assert db.metadata['input_files_sha256'] != input_fingerprint(geometry)


def test_reference_cannot_escape_input_directory(tmp_path):
    geometry = tmp_path / 'plane.avl'
    geometry.write_text('AFILE\n../other.dat\n')
    with pytest.raises(ValueError, match='remain'):
        input_fingerprint(geometry)
