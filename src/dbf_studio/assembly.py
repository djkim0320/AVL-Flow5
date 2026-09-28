"""Optional file-component metadata; mesh coordinates remain baked SI/FRD."""

import numpy as np


def validate_components(asset):
    if 'components' not in asset:
        return
    components = asset['components']
    if not isinstance(components, list) or not components:
        raise ValueError('조립체 파일 목록이 비어 있습니다.')
    ids = set()
    used = set()
    for c in components:
        if (
            not isinstance(c, dict)
            or not isinstance(c.get('id'), str)
            or not c['id']
            or c['id'] in ids
            or not isinstance(c.get('name'), str)
            or not c['name']
            or not isinstance(c.get('locked'), bool)
            or not isinstance(c.get('source'), dict)
            or not isinstance(c['source'].get('origin'), str)
        ):
            raise ValueError('조립체 파일 정보가 올바르지 않습니다.')
        ids.add(c['id'])
        for name, n in [('position', 3), ('quaternion', 4)]:
            a = np.asarray(c.get(name), float)
            if a.shape != (n,) or not np.isfinite(a).all():
                raise ValueError('조립체 위치·회전이 올바르지 않습니다.')
        if abs(np.linalg.norm(c['quaternion']) - 1) > 1e-5:
            raise ValueError('조립체 회전이 정규화되지 않았습니다.')
        if not isinstance(c.get('parts'), list) or not c['parts']:
            raise ValueError('파일에 속한 부품이 없습니다.')
        for i in c['parts']:
            if type(i) is not int or not 0 <= i < len(asset['parts']) or i in used:
                raise ValueError('조립체 부품 번호가 중복되거나 잘못됐습니다.')
            used.add(i)
    if len(used) != len(asset['parts']):
        raise ValueError('파일에 연결되지 않은 부품이 있습니다.')
