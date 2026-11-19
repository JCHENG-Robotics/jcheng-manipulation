import sys, json
import numpy as np
from armpi_grasp_planner.reachability import ArmKinematics
pulses=[float(v) for v in sys.argv[1].split(',')]
scale=0.00418879
j=[(500.0-p)*scale if i==2 else (p-500.0)*scale for i,p in enumerate(pulses)]
k=ArmKinematics(tip_link='link4'); f=k.forward(j[:4])
R=[[f.M[i,ii] for ii in range(3)] for i in range(3)]
print(json.dumps({'R':R,'t':[f.p.x(),f.p.y(),f.p.z()],'j':j}))
