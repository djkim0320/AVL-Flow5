"""Composition and reference-point rules shared by AeroDatabase validation and the hybrid builder."""

from pathlib import Path
import numpy as np

DEFAULT_COMPOSITION = dict(coeff='flow5', controls='flow5', rates='avl')

BLOCKS = ('coeff', 'controls', 'rates')

CLOSURE = 'classical_longitudinal_lateral'


def validate_composition(composition, closure=None, *, check_closure=True):
    if (
        not isinstance(composition, dict)
        or set(composition) != set(BLOCKS)
        or any(v not in ('avl', 'flow5') for v in composition.values())
    ):
        raise ValueError('복합 공력표의 coeff·controls·rates 출처를 avl 또는 flow5로 지정하세요.')
    if len(set(composition.values())) == 1:
        raise ValueError('세 블록의 출처가 같습니다. 단일 해석기를 선택하세요.')
    if check_closure and composition['rates'] == 'flow5' and closure != CLOSURE:
        raise ValueError('flow5 회전율 미계수에는 종·횡 분리 근사 동의가 필요합니다.')
    return dict(composition)


def avl_reference_frd(path):
    lines = [s.split('#')[0].split('!')[0].strip() for s in Path(path).read_text(encoding='ascii').splitlines()]
    lines = [s for s in lines if s]
    try:
        point = np.asarray([float(v) for v in lines[4].split()], float)
    except (ValueError, IndexError) as exc:
        raise ValueError('AVL 형상의 기준점 헤더를 읽을 수 없습니다.') from exc
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('AVL Xref·Yref·Zref는 유한한 세 좌표여야 합니다.')
    return point * np.array([-1.0, 1.0, -1.0])


def check_reference(config, path=None):
    cg = np.asarray(config['aircraft']['cg_m'], float)
    declared = np.asarray(config['aero']['moment_reference_frd_m'], float)
    actual = avl_reference_frd(path or Path(config['_root']) / config['aero']['geometry'])
    if (
        cg.shape != (3,)
        or not np.isfinite(cg).all()
        or declared.shape != (3,)
        or not np.allclose(declared, cg, rtol=0, atol=1e-9)
        or not np.allclose(actual, cg, rtol=0, atol=1e-9)
    ):
        raise ValueError('AVL 형상의 Xref·Yref·Zref가 CG와 다릅니다. 기체 정의에서 다시 등록하세요.')
