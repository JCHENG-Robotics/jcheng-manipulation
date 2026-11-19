import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k = ArmKinematics(tip_link='end_effector_link')
cur = [0.0, 0.134, -1.4074, -1.198, -0.0922]
target_z = 0.1109
best=[]
for j2 in np.arange(0.134, 0.30, 0.004):
    for j3 in np.arange(-1.44, -1.30, 0.004):
        for j4 in np.arange(-1.34, -1.10, 0.004):
            f = k.forward([0.0, j2, j3, j4, -0.0922])
            x,y,z = f.p.x(), f.p.y(), f.p.z()
            cost = abs(z-target_z)*2 + abs(x-0.2373) + abs(y+0.0021) + 0.02*(abs(j2-cur[1])+abs(j3-cur[2])+abs(j4-cur[3]))
            best.append((cost, z, x, y, j2, j3, j4))
best.sort()
for b in best[:6]:
    print('cost=%.5f z=%.4f x=%.4f y=%.4f j2=%.3f j3=%.3f j4=%.3f' % b)
