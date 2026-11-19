import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k=ArmKinematics(tip_link='end_effector_link')
tcp=np.array([0.2422,-0.0347,-0.0269]); top=-0.0357; cube_h=0.0394; floor=-0.0795
bottom_at_grasp=top-cube_h
target_bottom=floor+0.150
target_z=target_bottom+(tcp[2]-bottom_at_grasp)
print('grasp TCP',tcp,'bottom %.4f -> target TCP z %.4f'%(bottom_at_grasp,target_z))
best=[]
for j1 in np.arange(-0.35,0.05,0.01):
  for j2 in np.arange(0.05,0.60,0.01):
    for j3 in np.arange(-1.65,-1.10,0.01):
      for j4 in np.arange(-1.55,-0.85,0.01):
        f=k.forward([j1,j2,j3,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
        if abs(x-0.2422)<0.008 and abs(y+0.0347)<0.008:
            cost=abs(z-target_z)*3+abs(x-0.2422)+abs(y+0.0347)
            best.append((cost,z,x,y,j1,j2,j3,j4))
best.sort()
for b in best[:5]: print('cost=%.4f z=%.4f x=%.4f y=%.4f j=(%.3f,%.3f,%.3f,%.3f)'%b)
