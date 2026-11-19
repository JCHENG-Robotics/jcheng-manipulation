#!/usr/bin/env python3
"""RGB top-face measurement from the adopted-low pose (near-nadir at ~7 cm).

The depth camera is blind this close, so the cube is measured in RGB:
colour-segment the wooden top face, fit its min-area rectangle, and
back-project onto the known top plane.
"""
import json
import math

import cv2
import numpy as np

seg = json.load(open('/tmp/seg_low.json'))
cal = json.load(open('/tmp/hand_eye.json'))
K = np.array(cal['intrinsics']['refined_k'])
T = np.array(seg['hand_eye']['T_base_cloud'])
R, t = T[:3, :3], T[:3, 3]
z_top = float(__import__('sys').argv[1]) if len(__import__('sys').argv) > 1 else -0.0396
print('camera at', np.round(t, 4), 'optical +z in base', np.round(R[:, 2], 3), 'top plane z', z_top)

img = cv2.imread('/tmp/low_pose.jpg')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
wood = ((h > 5) & (h < 35) & (s > 35) & (v > 50)).astype(np.uint8)
wood = cv2.morphologyEx(wood, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
n, labels, stats, cents = cv2.connectedComponentsWithStats(wood, 8)
big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
wood = (labels == big).astype(np.uint8)
print('wood px', int(wood.sum()), 'bbox', stats[big, :4])
vals = v[wood > 0]
thr, _ = cv2.threshold(vals.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
top = ((wood > 0) & (v >= thr)).astype(np.uint8)
top = cv2.morphologyEx(top, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
n, labels, stats, _ = cv2.connectedComponentsWithStats(top, 8)
big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
top = (labels == big).astype(np.uint8)
print('otsu V=%.0f  top-face px=%d' % (thr, int(top.sum())))
cnts, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
rect = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
(cu, cv), (w, hh), ang = rect
print('top-face rect: centre=(%.1f, %.1f) px  size=%.1f x %.1f px  angle=%.1f' % (cu, cv, w, hh, ang))

def back(u, vv):
    ray = np.array([(u - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], 1.0])
    rb = R @ ray
    return t + (z_top - t[2]) / rb[2] * rb

c = back(cu, cv)
print('cube centre base_link: (%.4f, %.4f, %.4f)' % (c[0], c[1], c[2]))
box = cv2.boxPoints(rect)
corners = np.array([back(bu, bv) for bu, bv in box])
d = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
print('RGB metric edges on the top plane: %.1f, %.1f, %.1f, %.1f mm' % tuple(x * 1000 for x in d))
yaw = math.degrees(math.atan2(corners[1][1] - corners[0][1], corners[1][0] - corners[0][0]))
print('edge yaw in base_link: %.1f deg' % yaw)
vis = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
cv2.drawContours(vis, [np.int32(np.array(cv2.boxPoints(rect)) * 2)], -1, (0, 0, 255), 2)
cv2.imwrite('/tmp/low_top_rect.png', vis)
