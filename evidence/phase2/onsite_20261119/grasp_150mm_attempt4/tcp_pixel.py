import numpy as np, json
from armpi_grasp_planner.reachability import ArmKinematics
cal=json.load(open('/tmp/hand_eye.json')); Tlc=np.array(cal['hand_eye']['methods']['TSAI']['T_link4_camera']); K=np.array(cal['intrinsics']['refined_k'])
k4=ArmKinematics(tip_link='link4'); k5=ArmKinematics(tip_link='end_effector_link')
for name,j in (('view',[0.504,0.685,-1.652,-1.559]),('pre',[0.030,-0.560,-1.750,-0.545]),('grasp',[0.030,-0.6618,-1.8012,-0.495]),('adopted',[0.0,-0.6618,-1.8012,-0.6704])):
    f4=k4.forward(j); R4=np.array([[f4.M[i,ii] for ii in range(3)] for i in range(3)]); t4=np.array([f4.p.x(),f4.p.y(),f4.p.z()])
    T=np.eye(4);T[:3,:3]=R4;T[:3,3]=t4;Tb=T@Tlc;Rc,tc=Tb[:3,:3],Tb[:3,3]
    e=k5.forward(j+[0.0]); tcp=np.array([e.p.x(),e.p.y(),e.p.z()])
    pc=Rc.T@(tcp-tc)
    u=K[0,0]*pc[0]/pc[2]+K[0,2]; v=K[1,1]*pc[1]/pc[2]+K[1,2]
    print('%-8s TCP proj (%.1f, %.1f) dist %.3f'%(name,u,v,np.linalg.norm(tcp-tc)))
