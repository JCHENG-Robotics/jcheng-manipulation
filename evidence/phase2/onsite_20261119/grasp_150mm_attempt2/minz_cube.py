import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k = ArmKinematics(tip_link='end_effector_link')
lim = {'j1':1.8012,'j2':(-0.6618,1.2985),'j3':(-1.8012,0.0),'j4':(-1.5917,1.5917),'j5':(-1.5917,1.5499)}
best=[]
for j2 in np.arange(-0.66, 0.05, 0.01):
    for j3 in np.arange(-1.80, -1.30, 0.01):
        for j4 in np.arange(-1.20, 0.20, 0.01):
            f = k.forward([0.0, j2, j3, j4, 0.0])
            x,y,z = f.p.x(), f.p.y(), f.p.z()
            if abs(x-0.2305) < 0.005 and abs(y-0.0085) < 0.010:
                best.append((z, x, y, j2, j3, j4))
best.sort()
print('lowest TCP at x~0.2305:')
for b in best[:6]:
    print('z=%.4f x=%.4f y=%.4f j2=%.3f j3=%.3f j4=%.3f' % b)
# also min z at x 0.21 and 0.22
for xt in (0.205, 0.215, 0.225):
    cand=[]
    for j2 in np.arange(-0.66, 0.30, 0.01):
        for j3 in np.arange(-1.80, -1.20, 0.01):
            for j4 in np.arange(-1.40, 0.20, 0.01):
                f=k.forward([0.0,j2,j3,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
                if abs(x-xt)<0.005 and abs(y-0.0085)<0.010:
                    cand.append((z,x,j2,j3,j4))
    cand.sort()
    if cand: print('x~%.3f min z=%.4f (x=%.4f j2=%.3f j3=%.3f j4=%.3f)'%cand[0])
