import sys, numpy as np
sys.path.insert(0,'/repo_tools/phase2')
import depth_plane_fit as d
from sensor_msgs_py import point_cloud2
import rclpy
rclpy.init()
node=d.CloudCapture('/depth_cam/depth_registered/points')
msg=node.wait_for_cloud(15.0)
if msg is None:
    print('no cloud'); sys.exit(1)
pts=np.array([[p[0],p[1],p[2]] for p in point_cloud2.read_points(msg, skip_nans=True)],dtype=float)
print('points',len(pts))
np.save('/tmp/raw_cloud.npy',pts)
rng=np.linalg.norm(pts,axis=1)
print('range min/med/max %.3f/%.3f/%.3f'%(rng.min(),np.median(rng),rng.max()))
for lo,hi in ((0.03,0.10),(0.10,0.20),(0.20,0.30),(0.30,0.50),(0.50,1.0)):
    print('range %.2f-%.2f: %d'%(lo,hi,((rng>=lo)&(rng<hi)).sum()))
