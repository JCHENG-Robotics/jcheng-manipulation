import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k=ArmKinematics(tip_link='end_effector_link')
grasp_tcp=np.array([0.2287,-0.0242,-0.0402]); top=-0.0369; cube_h=0.0394; floor=-0.0795
bottom=top-cube_h; target_bottom=floor+0.150; target_z=target_bottom+(grasp_tcp[2]-bottom)
print('grasp TCP',grasp_tcp,'bottom %.4f target TCP z %.4f'%(bottom,target_z))
best=[]
for j1 in np.arange(-0.30,0.05,0.01):
  for j2 in np.arange(0.0,0.45,0.01):
    for j3 in np.arange(-1.70,-1.20,0.01):
      for j4 in np.arange(-1.45,-0.90,0.01):
        f=k.forward([j1,j2,j3,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
        if abs(x-0.2287)<0.008 and abs(y+0.0242)<0.008:
            cost=abs(z-target_z)*3+abs(x-0.2287)+abs(y+0.0242)
            best.append((cost,z,x,y,j1,j2,j3,j4))
best.sort()
for b in best[:5]: print('cost=%.4f z=%.4f x=%.4f y=%.4f j=(%.3f,%.3f,%.3f,%.3f)'%b)
