import numpy as np, json
from armpi_grasp_planner.reachability import ArmKinematics
cal=json.load(open('/tmp/hand_eye.json')); Tlc=np.array(cal['hand_eye']['methods']['TSAI']['T_link4_camera']); K=np.array(cal['intrinsics']['refined_k'])
k4=ArmKinematics(tip_link='link4'); k5=ArmKinematics(tip_link='end_effector_link')
cands=[(-0.200,0.270,-1.580,-0.950),(-0.200,0.210,-1.480,-1.040),(-0.200,0.140,-1.360,-1.150),(-0.200,0.130,-1.340,-1.170),(-0.200,0.200,-1.460,-1.060),
       (-0.10,0.30,-1.55,-0.95),(-0.10,0.10,-1.30,-1.20),(-0.05,0.20,-1.40,-1.05)]
for j in cands:
    f=k4.forward(list(j)); R=np.array([[f.M[i,ii] for ii in range(3)] for i in range(3)]); t=np.array([f.p.x(),f.p.y(),f.p.z()])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;Tb=T@Tlc;Rc,tc=Tb[:3,:3],Tb[:3,3]
    e=k5.forward(list(j)+[0.0]); tcp=np.array([e.p.x(),e.p.y(),e.p.z()])
    cube=tcp+np.array([0,0,-0.02]); pc=Rc.T@(cube-tc)
    if pc[2]<=0: print(j,'behind'); continue
    u=K[0,0]*pc[0]/pc[2]+K[0,2]; v=K[1,1]*pc[1]/pc[2]+K[1,2]
    print('j=%s tcp=(%.3f,%.3f,%.3f) cube proj=(%.0f,%.0f) dist=%.3f'%(tuple(round(x,2) for x in j),tcp[0],tcp[1],tcp[2],u,v,np.linalg.norm(cube-tc)))
