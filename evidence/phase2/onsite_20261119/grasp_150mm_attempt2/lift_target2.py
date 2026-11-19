import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
k = ArmKinematics(tip_link='end_effector_link')
# grasp pose that just planned: TCP (0.2278, 0.0083, -0.0406)
gz = -0.0406
floor = -0.0795            # physical floor (depth plane + 215 mm ruler)
cube_h = 0.0394
bottom_at_grasp = gz - (cube_h - 0.0010)   # TCP sits ~1 mm below the top
target_bottom = floor + 0.150
target_z = target_bottom + (gz - bottom_at_grasp)
print('target TCP z = %.4f (object bottom at %.4f, floor %.4f)' % (target_z, target_bottom, floor))
best=[]
for j1 in np.arange(-0.05, 0.151, 0.005):
    for j2 in np.arange(0.05, 0.40, 0.005):
        for j3 in np.arange(-1.55, -1.20, 0.005):
            for j4 in np.arange(-1.40, -0.95, 0.005):
                f=k.forward([j1,j2,j3,j4,0.0]); x,y,z=f.p.x(),f.p.y(),f.p.z()
                if abs(x-0.2278) < 0.010 and abs(y-0.0083) < 0.010:
                    cost = abs(z-target_z)*3 + abs(x-0.2278) + abs(y-0.0083) + 0.01*(abs(j1-0.05)+abs(j2-0.15)+abs(j3+1.40)+abs(j4+1.20))
                    best.append((cost,z,x,y,j1,j2,j3,j4))
best.sort()
for b in best[:5]:
    print('cost=%.4f z=%.4f x=%.4f y=%.4f j1=%.3f j2=%.3f j3=%.3f j4=%.3f'%b)
