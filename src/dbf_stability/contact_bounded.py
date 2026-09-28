"""Memory-bounded evaluation of complete contact-feature weights.

All possible contact witnesses remain. Features whose axis-aligned distance
bound excludes contact and whose combined unnormalised exponential weights
are <=1e-24 (+ N*exp(-700) from exponent clipping) are omitted. No fastmath.
"""

import numpy as np
from numba import njit
from .contact_features import _weights, _clip, _merge


@njit(cache=True)
def _pairs(va, ea, vb, eb, cut):
    result = np.empty((len(ea) * len(eb), 2), np.int64)
    count = 0
    alo = np.minimum(va[ea[:, 0]], va[ea[:, 1]])
    ahi = np.maximum(va[ea[:, 0]], va[ea[:, 1]])
    blo = np.minimum(vb[eb[:, 0]], vb[eb[:, 1]])
    bhi = np.maximum(vb[eb[:, 0]], vb[eb[:, 1]])
    for i in range(len(ea)):
        for j in range(len(eb)):
            distance2 = 0.0
            for axis in range(3):
                separation = max(alo[i, axis] - bhi[j, axis], blo[j, axis] - ahi[i, axis], 0.0)
                distance2 += separation * separation
                if distance2 > cut * cut:
                    break
            if distance2 <= cut * cut:
                result[count, 0] = i
                result[count, 1] = j
                count += 1
    return result[:count]


@njit(cache=True)
def _closest_bounded(points, vertices, faces, cut):
    out = np.empty_like(points)
    indices = np.empty(len(points), np.int64)
    count = 0
    lo = np.empty((len(faces), 3))
    hi = np.empty_like(lo)
    for fi in range(len(faces)):
        for axis in range(3):
            x = vertices[faces[fi, 0], axis]
            y = vertices[faces[fi, 1], axis]
            z = vertices[faces[fi, 2], axis]
            lo[fi, axis] = min(x, y, z)
            hi[fi, axis] = max(x, y, z)
    for i in range(len(points)):
        px, py, pz = points[i]
        best = cut * cut
        found = False
        qx_best = qy_best = qz_best = 0.0
        for fi in range(len(faces)):
            lower = 0.0
            for axis in range(3):
                separation = max(lo[fi, axis] - points[i, axis], points[i, axis] - hi[fi, axis], 0.0)
                lower += separation * separation
            if lower > best:
                continue
            face = faces[fi]
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
                    found = True
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
                    found = True
        if found:
            indices[count] = i
            out[count, 0] = qx_best
            out[count, 1] = qy_best
            out[count, 2] = qz_best
            count += 1
    return indices[:count], out[:count]


@njit(cache=True)
def _selected_edges(va, ea, vb, eb, pairs, pa, pb, factors, offset):
    count = len(pairs)
    for pair in range(count):
        i, j = pairs[pair]
        ax, ay, az = va[ea[i, 0]]
        endx, endy, endz = va[ea[i, 1]]
        ux, uy, uz = endx - ax, endy - ay, endz - az
        a = ux * ux + uy * uy + uz * uz
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
            k = offset + case * count + pair
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
def _mesh_mesh_bounded(va, fa, ea, vb, fb, eb, skin):
    # exp(-700) clipping leaves another <=N*exp(-700) numerical tail.
    total = len(va) + len(vb) + 5 * len(ea) * len(eb)
    cut = skin + 0.2 * skin * np.log(max(total, 1) / 1e-24)
    pairs = _pairs(va, ea, vb, eb, cut)
    ia, ca = _closest_bounded(va, vb, fb, cut)
    ib, cb = _closest_bounded(vb, va, fa, cut)
    offset = len(ia) + len(ib)
    count = offset + 5 * len(pairs)
    if count == 0:
        return np.empty((0, 11))
    pa = np.empty((count, 3))
    pb = np.empty((count, 3))
    factors = np.ones(count)
    pa[: len(ia)] = va[ia]
    pb[: len(ia)] = ca
    pa[len(ia) : offset] = cb
    pb[len(ia) : offset] = vb[ib]
    _selected_edges(va, ea, vb, eb, pairs, pa, pb, factors, offset)
    return _weights(pa, pb, factors, skin)


def bounded_features(va, fa, ea, vb, fb, eb, skin):
    return _merge(_mesh_mesh_bounded(va, fa, ea, vb, fb, eb, skin))
