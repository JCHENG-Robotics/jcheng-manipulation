import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k=ArmKinematics(tip_link='end_effector_link')
cube=np.array([0.2282,-0.0242,-0.0369])
best=[]
for j1 in np.arange(-0.30,-0.02,0.005):
    for j4 in np.arange(-0.60,-0.25,0.005):
        f=k.forward([j1,-0.6618,-1.8012,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
        cost=abs(x-cube[0])*1.0+abs(y-cube[1])*3.0+abs(z-cube[2])*1.0
        best.append((cost,x,y,z,j1,j4))
best.sort()
for b in best[:5]:
    print('cost=%.4f TCP=(%.4f,%.4f,%.4f) dx=%.1f dy=%.1f dz=%.1f j1=%.3f j4=%.3f'%(b[0],b[1],b[2],b[3],(b[1]-cube[0])*1000,(b[2]-cube[1])*1000,(b[3]-cube[2])*1000,b[4],b[5]))
