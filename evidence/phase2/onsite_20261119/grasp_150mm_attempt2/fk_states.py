from armpi_grasp_planner.reachability import ArmKinematics
for j in ([0.0,-0.2,-1.2,-0.6,0.0],[0.0,0.2,-1.6,-1.0,0.0],[0.0,0.4,-1.0,-1.2,0.0]):
    tcp=ArmKinematics(tip_link='end_effector_link').forward(j)
    g=ArmKinematics(tip_link='gripper_link').forward(j)
    print(j,'TCP=(%.4f,%.4f,%.4f)'%(tcp.p.x(),tcp.p.y(),tcp.p.z()),'gripper=(%.4f,%.4f,%.4f)'%(g.p.x(),g.p.y(),g.p.z()))
