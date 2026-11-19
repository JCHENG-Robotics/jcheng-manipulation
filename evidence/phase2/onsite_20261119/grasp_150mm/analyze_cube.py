#!/usr/bin/env python3
"""Robust XY geometry of the selected cube cluster from object_segment_rgbd."""
import json
import numpy as np


def min_area_rect(xy):
    """Return (centre, size, yaw) of the minimum-area rectangle (rotating calipers)."""
    pts = np.asarray(xy, dtype=float)
    hull = convex_hull(pts)
    best = None
    n = len(hull)
    for i in range(n):
        edge = hull[(i + 1) % n] - hull[i]
        norm = np.hypot(*edge)
        if norm < 1e-9:
            continue
        u = edge / norm
        v = np.array([-u[1], u[0]])
        pu = hull @ u
        pv = hull @ v
        size = (pu.max() - pu.min(), pv.max() - pv.min())
        area = size[0] * size[1]
        if best is None or area < best[0]:
            uc = 0.5 * (pu.max() + pu.min())
            vc = 0.5 * (pv.max() + pv.min())
            centre = uc * u + vc * v
            best = (area, centre, size, np.arctan2(u[1], u[0]))
    return best


def convex_hull(points):
    pts = sorted(map(tuple, points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


data = np.load('/tmp/view3.npz')
pts = data['points']
print('cluster points:', len(pts))

# full cluster XY projection
area, centre, size, yaw = min_area_rect(pts[:, :2])
print('full-cluster min-area rect: centre=(%.4f, %.4f) size=%.1f x %.1f mm yaw=%.1f deg'
      % (centre[0], centre[1], size[0] * 1000, size[1] * 1000, np.degrees(yaw)))

# top slab (highest 5 mm)
zmax = pts[:, 2].max()
top = pts[pts[:, 2] >= zmax - 0.005]
area_t, centre_t, size_t, yaw_t = min_area_rect(top[:, :2])
print('top-slab  min-area rect: centre=(%.4f, %.4f) size=%.1f x %.1f mm yaw=%.1f deg'
      % (centre_t[0], centre_t[1], size_t[0] * 1000, size_t[1] * 1000, np.degrees(yaw_t)))
print('z: min=%.4f max=%.4f' % (pts[:, 2].min(), zmax))

# 3 mm slab too
for slab in (0.002, 0.003, 0.004):
    sel = pts[pts[:, 2] >= zmax - slab]
    _, c, s, y = min_area_rect(sel[:, :2])
    print('top %.0fmm slab: n=%d centre=(%.4f, %.4f) size=%.1f x %.1f mm yaw=%.1f'
          % (slab * 1000, len(sel), c[0], c[1], s[0] * 1000, s[1] * 1000, np.degrees(y)))

# side faces: points in the upper 25 mm (sides + top)
side = pts[pts[:, 2] >= pts[:, 2].max() - 0.030]
_, c, s, y = min_area_rect(side[:, :2])
print('upper30mm slab: n=%d centre=(%.4f, %.4f) size=%.1f x %.1f mm yaw=%.1f'
      % (len(side), c[0], c[1], s[0] * 1000, s[1] * 1000, np.degrees(y)))

# robust median centre
print('xy median: (%.4f, %.4f)' % (np.median(pts[:, 0]), np.median(pts[:, 1])))
