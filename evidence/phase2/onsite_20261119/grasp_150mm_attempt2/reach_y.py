import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k = ArmKinematics(tip_link='end_effector_link')
best=[]
for j1 in np.arange(-0.15, 0.151, 0.01):
    for j2 in np.arange(-0.6618, -0.30, 0.01):
        for j3 in np.arange(-1.8012, -1.45, 0.01):
            for j4 in np.arange(-1.10, -0.20, 0.01):
                f=k.forward([j1,j2,j3,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
                if abs(x-0.2305)<0.004 and abs(y-0.0085)<0.004:
                    best.append((z,x,y,j1,j2,j3,j4))
best.sort()
print('poses within 4mm of (0.2305, 0.0085):',len(best))
for b in best[:8]:
    print('z=%.4f x=%.4f y=%.4f j1=%.3f j2=%.3f j3=%.3f j4=%.3f'%b)
