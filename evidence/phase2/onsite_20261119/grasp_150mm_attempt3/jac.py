import numpy as np, json
from armpi_grasp_planner.reachability import ArmKinematics
cal=json.load(open('/tmp/hand_eye.json')); Tlc=np.array(cal['hand_eye']['methods']['TSAI']['T_link4_camera']); K=np.array(cal['intrinsics']['refined_k'])
k=ArmKinematics(tip_link='link4')
base=np.array([0.044243,0.492286,-1.790792,-1.485349])
def proj(j, cube):
    f=k.forward(list(j)); R=np.array([[f.M[i,ii] for ii in range(3)] for i in range(3)]); t=np.array([f.p.x(),f.p.y(),f.p.z()])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t;Tb=T@Tlc;Rc,tc=Tb[:3,:3],Tb[:3,3]
    pc=Rc.T@(cube-tc)
    return np.array([K[0,0]*pc[0]/pc[2]+K[0,2], K[1,1]*pc[1]/pc[2]+K[1,2]])
cube=np.array([0.2303,0.0151,-0.0396])
p0=proj(base,cube); print('model p0',np.round(p0,1))
J=np.zeros((2,2))
eps=0.01
for i,idx in enumerate((0,3)):
    jp=base.copy(); jp[idx]+=eps
    J[:,i]=(proj(jp,cube)-p0)/eps
print('J (du/dj1, du/dj4; dv/dj1, dv/dj4):\n',np.round(J,1))
# observed offset: want the image to move by (-85, +35)
target=np.array([-85.0,35.0])
delta=np.linalg.solve(J,target)
print('required djoints (j1,j4):',np.round(delta,4),'rad -> j=',np.round(base+np.array([delta[0],0,0,delta[1]]),4))
for s in (0.5,0.75,1.0):
    d=delta*s; jn=base+np.array([d[0],0,0,d[1]]); print('scale %.2f -> j=%s model proj=%s'%(s,np.round(jn,3),np.round(proj(jn,cube),1)))
