import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k=ArmKinematics(tip_link='end_effector_link')
target=np.array([0.2157,-0.0001,-0.0403])
best=[]
for j1 in np.arange(-0.10,0.10,0.004):
    for j4 in np.arange(-0.75,-0.25,0.004):
        f=k.forward([j1,-0.6618,-1.8012,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
        cost=abs(x-target[0])+abs(y-target[1])*3+abs(z-target[2])
        best.append((cost,x,y,z,j1,j4))
best.sort()
for b in best[:5]:
    print('cost=%.4f TCP=(%.4f,%.4f,%.4f) dx=%.1f dy=%.1f dz=%.1f j1=%.3f j4=%.3f'%(b[0],b[1],b[2],b[3],(b[1]-target[0])*1000,(b[2]-target[1])*1000,(b[3]-target[2])*1000,b[4],b[5]))
