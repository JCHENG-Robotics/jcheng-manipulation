#!/usr/bin/env python3
"""Host-side AprilTag probe: image + link4 FK (json) -> marker pose in base_link."""
import json
import sys

import cv2
import numpy as np

img_path = sys.argv[1]
fk_path = sys.argv[2]
size = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0734
marker_id = int(sys.argv[4]) if len(sys.argv) > 4 else 0

cal = json.load(open('/tmp/hand_eye.json'))
K = np.array(cal['intrinsics']['refined_k'])
D = np.array(cal['intrinsics']['refined_d'])
Tlc = np.array(cal['hand_eye']['methods']['TSAI']['T_link4_camera'])
fk = json.load(open(fk_path))
T = np.eye(4)
T[:3, :3] = np.array(fk['R'])
T[:3, 3] = np.array(fk['t'])
Tb = T @ Tlc

img = cv2.imread(img_path)
g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
params = cv2.aruco.DetectorParameters_create()
corners, ids, _ = cv2.aruco.detectMarkers(g, d, parameters=params)
if ids is None or marker_id not in ids.ravel():
    print('TAG_NOT_FOUND', '' if ids is None else ids.ravel().tolist())
    sys.exit(1)
idx = int(np.where(ids.ravel() == marker_id)[0][0])
c = corners[idx][0].astype(np.float64)
rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx:idx+1], size, K, D)
rvec, tvec = rvecs[0].ravel(), tvecs[0].ravel()
half = size / 2.0
obj = np.array([[-half, half, 0], [half, half, 0],
                [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
proj, _ = cv2.projectPoints(obj, rvec.reshape(3,1), tvec.reshape(3,1), K, D)
err = float(np.linalg.norm(proj.reshape(-1, 2) - c, axis=1).mean())
Rcm, _ = cv2.Rodrigues(rvec.reshape(3,1))
Tcm = np.eye(4)
Tcm[:3, :3] = Rcm
Tcm[:3, 3] = tvec
Tbm = Tb @ Tcm
p = Tbm[:3, 3]
sp = float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]))
print(json.dumps({
    'centre': [round(float(v), 5) for v in p],
    'dist_m': round(float(np.linalg.norm(tvec)), 4),
    'px_side': round(sp, 1),
    'reproj_px': round(err, 2),
}))
