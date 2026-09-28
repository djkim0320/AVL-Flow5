import pytest

cq = pytest.importorskip('cadquery')

from dbf_studio.server import convert_step


def test_assembly_names_follow_solids(tmp_path):
    assembly = cq.Assembly(name='root')
    group = cq.Assembly(name='group', loc=cq.Location(cq.Vector(0, 0, 50)))
    group.add(cq.Workplane().box(10, 10, 10).val(), name='Wing', loc=cq.Location(cq.Vector(100, 0, 0)))
    assembly.add(cq.Workplane().box(20, 5, 5).val(), name='Fuselage')
    assembly.add(group)
    path = tmp_path / 'named.step'
    assembly.export(str(path))
    parts = convert_step(path)['parts']
    assert sorted(p['name'] for p in parts) == ['Fuselage', 'Wing']
    wing = next(p for p in parts if p['name'] == 'Wing')
    assert min(wing['positions'][0::3]) == pytest.approx(95)
    assert min(wing['positions'][2::3]) == pytest.approx(45)


def test_plain_step_keeps_numbered_names(tmp_path):
    path = tmp_path / 'plain.step'
    cq.exporters.export(cq.Workplane().box(1, 1, 1), str(path))
    assert [p['name'] for p in convert_step(path)['parts']] == ['부품 1']
