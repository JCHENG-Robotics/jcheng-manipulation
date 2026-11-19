import numpy as np, json
from armpi_grasp_planner.reachability import ArmKinematics
cal=json.load(open('/tmp/hand_eye.json')); Tlc=np.array(cal['hand_eye']['methods']['TSAI']['T_link4_camera']); K=np.array(cal['intrinsics']['refined_k'])
k=ArmKinematics(tip_link='link4')
for name,j in (('grasp',[-0.201,-0.662,-1.601,-0.592]),('lift',[-0.195,0.280,-1.571,-0.945])):
    f=k.forward(j); R=np.array([[f.M[i,ii] for ii in range(3)] for i in range(3)]); t=np.array([f.p.x(),f.p.y(),f.p.z()])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;Tb=T@Tlc;Rc,tc=Tb[:3,:3],Tb[:3,3]
    # TCP from the end-effector FK
    e=ArmKinematics(tip_link='end_effector_link').forward(j+[0.0]); tcp=np.array([e.p.x(),e.p.y(),e.p.z()])
    cube=tcp+np.array([0,0,-0.02])
    pc=Rc.T@(cube-tc); u=K[0,0]*pc[0]/pc[2]+K[0,2]; v=K[1,1]*pc[1]/pc[2]+K[1,2]
    print('%s cam=(%.3f,%.3f,%.3f) TCP=(%.3f,%.3f,%.3f) cube(held) proj=(%.0f,%.0f) dist=%.3f'%(name,tc[0],tc[1],tc[2],tcp[0],tcp[1],tcp[2],u,v,np.linalg.norm(cube-tc)))

# floor position of the cube at the lift pose
for name,j in (('lift',[-0.195,0.280,-1.571,-0.945]),):
    f=k.forward(j); R=np.array([[f.M[i,ii] for ii in range(3)] for i in range(3)]); t=np.array([f.p.x(),f.p.y(),f.p.z()])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;Tb=T@Tlc;Rc,tc=Tb[:3,:3],Tb[:3,3]
    for label,p in (('floor_pos',np.array([0.242,-0.033,-0.075])),('held_pos',np.array([0.243,-0.035,0.10]))):
        pc=Rc.T@(p-tc)
        if pc[2]<=0: print(label,'behind camera'); continue
        u=K[0,0]*pc[0]/pc[2]+K[0,2]; v=K[1,1]*pc[1]/pc[2]+K[1,2]
        print('%s proj=(%.0f,%.0f) dist=%.3f'%(label,u,v,np.linalg.norm(p-tc)))
