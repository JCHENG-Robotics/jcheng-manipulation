import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k=ArmKinematics(tip_link='end_effector_link')
target=np.array([0.2157,-0.0003,0.1013])
best=[]
for j1 in np.arange(-0.06,0.07,0.01):
  for j2 in np.arange(0.05,0.35,0.01):
    for j3 in np.arange(-1.60,-1.15,0.01):
      for j4 in np.arange(-1.45,-1.00,0.01):
        f=k.forward([j1,j2,j3,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
        if abs(x-target[0])<0.006 and abs(y-target[1])<0.006:
            cost=abs(z-target[2])*3+abs(x-target[0])+abs(y-target[1])
            best.append((cost,z,x,y,j1,j2,j3,j4))
best.sort()
for b in best[:5]: print('cost=%.4f z=%.4f x=%.4f y=%.4f j=(%.3f,%.3f,%.3f,%.3f)'%b)
