#!/usr/bin/env python3
"""RGB top-face-only measurement of the wooden cube."""
import json
import math

import cv2
import numpy as np

seg = json.load(open('/tmp/seg_view3.json'))
cal = json.load(open('/tmp/hand_eye.json'))
K = np.array(cal['intrinsics']['refined_k'])
T = np.array(seg['hand_eye']['T_base_cloud'])
R, t = T[:3, :3], T[:3, 3]
pts_base = np.load('/tmp/view3.npz')['points']

img = cv2.imread('/tmp/view3.jpg')
cam = (pts_base - t) @ R
uv = np.column_stack([K[0, 0] * cam[:, 0] / cam[:, 2] + K[0, 2],
                      K[1, 1] * cam[:, 1] / cam[:, 2] + K[1, 2]])
u0, u1 = max(0, int(uv[:, 0].min()) - 30), min(img.shape[1], int(uv[:, 0].max()) + 30)
v0, v1 = max(0, int(uv[:, 1].min()) - 30), min(img.shape[0], int(uv[:, 1].max()) + 30)
crop = img[v0:v1, u0:u1]
hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
wood = ((h > 5) & (h < 35) & (s > 35) & (v > 50)).astype(np.uint8)
wood = cv2.morphologyEx(wood, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
# keep the largest component
n, labels, stats, _ = cv2.connectedComponentsWithStats(wood, 8)
big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
wood = (labels == big).astype(np.uint8)
n_total = int(wood.sum())

vals = v[wood > 0]
thr, _ = cv2.threshold(vals.reshape(-1, 1), 0, 255,
                       cv2.THRESH_BINARY + cv2.THRESH_OTSU)
print('wood px=%d otsu V=%.0f' % (n_total, thr))
top = ((wood > 0) & (v >= thr)).astype(np.uint8)
top = cv2.morphologyEx(top, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
n, labels, stats, cents = cv2.connectedComponentsWithStats(top, 8)
big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
top = (labels == big).astype(np.uint8)
print('top-face px=%d (%.0f%% of wood)' % (int(top.sum()), 100.0 * top.sum() / n_total))
cnts, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
rect = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
(cu, cv), (w, h_), ang = rect
cu_f, cv_f = cu + u0, cv + v0
print('top-face rect: centre=(%.1f, %.1f) px size=%.1f x %.1f px angle=%.1f'
      % (cu_f, cv_f, w, h_, ang))

z_top = float(pts_base[:, 2].max())
def back(u, vv):
    ray = np.array([(u - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], 1.0])
    rb = R @ ray
    return t + (z_top - t[2]) / rb[2] * rb

c = back(cu_f, cv_f)
print('top-face centre base_link (z=%.4f): (%.4f, %.4f)' % (z_top, c[0], c[1]))
box = cv2.boxPoints(rect) + np.array([u0, v0])
corners = np.array([back(bu, bv) for bu, bv in box])
d = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
print('RGB top-face edges: %.1f, %.1f, %.1f, %.1f mm' % tuple(x * 1000 for x in d))
yaw = math.degrees(math.atan2(corners[1][1] - corners[0][1], corners[1][0] - corners[0][0]))
print('RGB edge yaw (base_link): %.1f deg' % yaw)

vis = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
cv2.drawContours(vis, [np.int32((np.array(cv2.boxPoints(rect), np.float32)) * 4)], -1, (0, 0, 255), 2)
cv2.imwrite('/tmp/cube_top_rect.png', vis)
