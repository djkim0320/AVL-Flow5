"""Reject meshes that the flow5 adapter would silently redistribute."""

from pathlib import Path
import pytest
from dbf_stability.flow5 import read_lifting_surfaces

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY = ROOT / 'test_models/N3_DBF2027/aero/extra/N3.avl'


@pytest.mark.parametrize(
    'old,new,reason',
    [
        ('24 1\n', '24 0\n', 'Cspace'),
        ('24 1\n', '24 1 36 1\n', 'SURFACE Nspan'),
        ('1.5 12 1\n', '1.5 12 -1\n', 'Sspace'),
        ('1.5 12 1\n', '1.5\n', 'SECTION Nspan'),
        ('1.5 12 1\n', '1.5 12.5 1\n', 'SECTION Nspan'),
    ],
)
def test_unsupported_mesh_is_rejected_before_export(tmp_path, old, new, reason):
    source = GEOMETRY.read_text('ascii')
    assert old in source
    changed = tmp_path / 'changed.avl'
    changed.write_text(source.replace(old, new, 1), encoding='ascii')
    with pytest.raises(ValueError, match=reason):
        read_lifting_surfaces(changed)


def test_n3_supported_mesh_keeps_its_panel_counts():
    _, surfaces = read_lifting_surfaces(GEOMETRY)
    assert [s['nx'] for s in surfaces] == [24, 24, 24]
    assert {section['ny'] for s in surfaces for section in s['sections']} == {12}
