"""Compiled evaluation of the existing complete mesh-feature contact formula.

No fastmath, feature sampling, changed skin, or replacement collider. Feature
ordering, edge regularisation and exponential weights match collision.py.
"""

import numpy as np
from numba import njit
from .math3d import BOX_SIGNS, BOX_EDGES


@njit(cache=True)
def _clip(x, lo, hi):
    return min(max(x, lo), hi)


@njit(cache=True)
def _closest(points, vertices, faces):
    out = np.empty_like(points)
    for i in range(len(points)):
        px, py, pz = points[i]
        best = np.inf
        qx_best = qy_best = qz_best = 0.0
        for face in faces:
            ax, ay, az = vertices[face[0]]
            bx, by, bz = vertices[face[1]]
            cx, cy, cz = vertices[face[2]]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            norm = np.sqrt(nx * nx + ny * ny + nz * nz)
            nx /= norm
            ny /= norm
            nz /= norm
            distance = (px - ax) * nx + (py - ay) * ny + (pz - az) * nz
            qx, qy, qz = px - distance * nx, py - distance * ny, pz - distance * nz
            dx, dy, dz = qx - ax, qy - ay, qz - az
            uu = ux * ux + uy * uy + uz * uz
            uv = ux * vx + uy * vy + uz * vz
            vv = vx * vx + vy * vy + vz * vz
            den = uu * vv - uv * uv
            du = dx * ux + dy * uy + dz * uz
            dv = dx * vx + dy * vy + dz * vz
            s = (vv * du - uv * dv) / den
            t = (uu * dv - uv * du) / den
            if s >= 0 and t >= 0 and s + t <= 1:
                distance = (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2
                if distance < best:
                    best = distance
                    qx_best = qx
                    qy_best = qy
                    qz_best = qz
            for j in range(3):
                sx, sy, sz = vertices[face[j]]
                ex, ey, ez = vertices[face[(j + 1) % 3]]
                dx, dy, dz = ex - sx, ey - sy, ez - sz
                k = _clip(((px - sx) * dx + (py - sy) * dy + (pz - sz) * dz) / (dx * dx + dy * dy + dz * dz), 0.0, 1.0)
                qx, qy, qz = sx + k * dx, sy + k * dy, sz + k * dz
                distance = (px - qx) ** 2 + (py - qy) ** 2 + (pz - qz) ** 2
                if distance < best:
                    best = distance
                    qx_best = qx
                    qy_best = qy
                    qz_best = qz
        out[i, 0] = qx_best
        out[i, 1] = qy_best
        out[i, 2] = qz_best
    return out


@njit(cache=True)
def _edge_features(va, ea, vb, eb, pa, pb, factors, offset):
    count = len(ea) * len(eb)
    for i in range(len(ea)):
        ax, ay, az = va[ea[i, 0]]
        endx, endy, endz = va[ea[i, 1]]
        ux, uy, uz = endx - ax, endy - ay, endz - az
        a = ux * ux + uy * uy + uz * uz
        for j in range(len(eb)):
            bx, by, bz = vb[eb[j, 0]]
            endx, endy, endz = vb[eb[j, 1]]
            vx, vy, vz = endx - bx, endy - by, endz - bz
            wx, wy, wz = ax - bx, ay - by, az - bz
            b = ux * vx + uy * vy + uz * vz
            c = vx * vx + vy * vy + vz * vz
            d = ux * wx + uy * wy + uz * wz
            e = vx * wx + vy * wy + vz * wz
            den = max(a * c - b * b, 0.0)
            relative = den / (a * c)
            if relative > 1e-14:
                t = (b * e - c * d) / den
                s = (a * e - b * d) / den
            else:
                t = 0.5
                s = _clip(e / c, 0.0, 1.0)
            for case in range(5):
                k = offset + case * count + i * len(eb) + j
                factor = 1.0
                if case == 0:
                    ta = _clip(t, 0.0, 1.0)
                    sb = _clip(s, 0.0, 1.0)
                    factor = relative / (relative + 1e-8)
                elif case == 1:
                    ta = 0.0
                    sb = _clip(e / c, 0.0, 1.0)
                elif case == 2:
                    ta = 1.0
                    sb = _clip((e + b) / c, 0.0, 1.0)
                elif case == 3:
                    ta = _clip(-d / a, 0.0, 1.0)
                    sb = 0.0
                else:
                    ta = _clip((b - d) / a, 0.0, 1.0)
                    sb = 1.0
                pa[k, 0] = ax + ta * ux
                pa[k, 1] = ay + ta * uy
                pa[k, 2] = az + ta * uz
                pb[k, 0] = bx + sb * vx
                pb[k, 1] = by + sb * vy
                pb[k, 2] = bz + sb * vz
                factors[k] = factor


@njit(cache=True)
def _weights(pa, pb, factors, skin):
    gaps = np.empty(len(pa))
    minimum = np.inf
    for i in range(len(pa)):
        dx, dy, dz = pa[i, 0] - pb[i, 0], pa[i, 1] - pb[i, 1], pa[i, 2] - pb[i, 2]
        gaps[i] = np.sqrt(dx * dx + dy * dy + dz * dz)
        minimum = min(minimum, gaps[i])
    if minimum >= skin:
        return np.empty((0, 11))
    weights = np.empty(len(pa))
    total = 0.0
    for i in range(len(pa)):
        weights[i] = factors[i] * np.exp(-_clip((gaps[i] - minimum) / (skin * 0.2), 0.0, 700.0))
        total += weights[i]
    count = 0
    for i in range(len(pa)):
        weights[i] /= total
        if gaps[i] < skin and weights[i] > 1e-10:
            count += 1
    out = np.empty((count, 11))
    at = 0
    for i in range(len(pa)):
        if gaps[i] < skin and weights[i] > 1e-10:
            out[at, :3] = pa[i]
            out[at, 3:6] = pb[i]
            out[at, 6:9] = (pa[i] - pb[i]) / max(gaps[i], 1e-15)
            out[at, 9] = gaps[i]
            out[at, 10] = weights[i]
            at += 1
    return out


@njit(cache=True)
def _mesh_box(va, faces, edges, cb, ab, hb, signs, box_edges, skin):
    vb = (signs * hb) @ ab.T + cb
    offset = len(va) + len(vb)
    count = offset + 5 * len(edges) * len(box_edges)
    pa = np.empty((count, 3))
    pb = np.empty((count, 3))
    factors = np.ones(count)
    pa[: len(va)] = va
    for i in range(len(va)):
        local = (va[i] - cb) @ ab
        for j in range(3):
            local[j] = _clip(local[j], -hb[j], hb[j])
        pb[i] = local @ ab.T + cb
    pa[len(va) : offset] = _closest(vb, va, faces)
    pb[len(va) : offset] = vb
    _edge_features(va, edges, vb, box_edges, pa, pb, factors, offset)
    return _weights(pa, pb, factors, skin)


@njit(cache=True)
def _mesh_mesh(va, fa, ea, vb, fb, eb, skin):
    offset = len(va) + len(vb)
    count = offset + 5 * len(ea) * len(eb)
    pa = np.empty((count, 3))
    pb = np.empty((count, 3))
    factors = np.ones(count)
    pa[: len(va)] = va
    pb[: len(va)] = _closest(va, vb, fb)
    pa[len(va) : offset] = _closest(vb, va, fa)
    pb[len(va) : offset] = vb
    _edge_features(va, ea, vb, eb, pa, pb, factors, offset)
    return _weights(pa, pb, factors, skin)


def _merge(features):
    rows = {}
    for row in features:
        x, y, normal, gap, weight = row[:3], row[3:6], row[6:9], float(row[9]), float(row[10])
        key = tuple(np.round(row[:6], 11))
        if key in rows:
            rows[key][-1] += weight
        else:
            rows[key] = [x, y, normal, gap, weight]
    return list(rows.values())


def mesh_box_manifold(va, faces, edges, cb, ab, hb, skin):
    return _merge(
        _mesh_box(va, faces, edges, np.asarray(cb), np.asarray(ab), np.asarray(hb), BOX_SIGNS, BOX_EDGES, skin)
    )


def mesh_mesh_manifold(va, fa, ea, vb, fb, eb, skin):
    return _merge(_mesh_mesh(va, fa, ea, vb, fb, eb, skin))
