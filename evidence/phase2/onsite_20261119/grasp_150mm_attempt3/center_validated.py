import numpy as np, json
from armpi_grasp_planner.reachability import ArmKinematics
cal=json.load(open('/tmp/hand_eye.json')); Tlc=np.array(cal['hand_eye']['methods']['TSAI']['T_link4_camera']); K=np.array(cal['intrinsics']['refined_k'])
cube=np.array([0.2303,0.0151,-0.0396]); k=ArmKinematics(tip_link='link4')
best=[]
for j1 in np.arange(-0.4,0.9,0.05):
  for j2 in np.arange(0.15,1.05,0.05):
    for j3 in np.arange(-1.68,-0.95,0.05):     # validated-ish region
      for j4 in np.arange(-1.5917,-1.05,0.05):
        f=k.forward([j1,j2,j3,j4]); R=np.array([[f.M[i,j] for j in range(3)] for i in range(3)]); t=np.array([f.p.x(),f.p.y(),f.p.z()])
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;Tb=T@Tlc;Rc,tc=Tb[:3,:3],Tb[:3,3]
        d=cube-tc; dist=np.linalg.norm(d)
        if dist<0.20 or dist>0.45: continue
        pc=Rc.T@d
        if pc[2]<=0.02: continue
        u=K[0,0]*pc[0]/pc[2]+K[0,2]; v=K[1,1]*pc[1]/pc[2]+K[1,2]
        best.append((abs(u-320)+abs(v-200),u,v,dist,j1,j2,j3,j4))
best.sort()
for b in best[:5]:
    print('score=%.1f u=%.1f v=%.1f dist=%.3f j=(%.2f,%.2f,%.2f,%.2f)'%b)
