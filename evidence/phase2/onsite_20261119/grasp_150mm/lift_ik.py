#!/usr/bin/env python3
"""IK for a TCP translation at the current orientation, with MoveIt.

Prints the current TCP, the requested target, and IK solutions for several
seeds so the caller can pick one and feed it to real_moveit_execute.py.
"""
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState

DZ = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03
DX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
DY = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

rclpy.init()
node = rclpy.create_node('lift_ik')
js = None
def cb(msg):
    global js
    js = msg
sub = node.create_subscription(JointState, '/joint_states', cb, 10)
while js is None:
    rclpy.spin_once(node, timeout_sec=1.0)
print('joint_states:', dict(zip(js.name, [round(v, 4) for v in js.position])))
names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
jv = [js.position[js.name.index(n)] for n in names]
r = js.position[js.name.index('r_joint')] if 'r_joint' in js.name else -2.22

fk_cli = node.create_client(GetPositionFK, '/compute_fk')
fk_cli.wait_for_service(timeout_sec=10.0)
req = GetPositionFK.Request()
req.header.frame_id = 'base_link'
req.fk_link_names = ['end_effector_link']
req.robot_state = RobotState()
req.robot_state.joint_state = js
future = fk_cli.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
fk = future.result()
p = fk.pose_stamped[0].pose
print('current TCP: %.4f %.4f %.4f  q=(%.4f %.4f %.4f %.4f)'
      % (p.position.x, p.position.y, p.position.z,
         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

target = Pose()
target.position.x = p.position.x + DX
target.position.y = p.position.y + DY
target.position.z = p.position.z + DZ
target.orientation = p.orientation
print('target TCP: %.4f %.4f %.4f' % (target.position.x, target.position.y, target.position.z))

ik_cli = node.create_client(GetPositionIK, '/compute_ik')
ik_cli.wait_for_service(timeout_sec=10.0)
seeds = [jv,
         [0.0, -0.5, -1.7, -0.5, 0.0],
         [0.0, -0.5, -1.6, -0.7, 0.0],
         [0.0, -0.3, -1.5, -0.8, 0.0],
         [0.0, 0.0, -1.0, -1.0, 0.0],
         [0.0, 0.3, -1.2, -0.6, 0.0]]
for seed in seeds:
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'arm'
    req.ik_request.avoid_collisions = True
    req.ik_request.ik_link_name = 'end_effector_link'
    req.ik_request.pose_stamped.header.frame_id = 'base_link'
    req.ik_request.pose_stamped.pose = target
    req.ik_request.robot_state.joint_state.name = names + ['r_joint']
    req.ik_request.robot_state.joint_state.position = list(seed) + [r]
    req.ik_request.timeout.sec = 1
    fut = ik_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=6.0)
    out = fut.result()
    if out is None:
        print('seed', [round(x, 3) for x in seed], 'no response')
        continue
    code = out.error_code.val
    sol = {n: v for n, v in zip(out.solution.joint_state.name,
                                out.solution.joint_state.position)}
    print('seed', [round(x, 3) for x in seed], 'code', code,
          'sol', [round(sol.get(n, float('nan')), 4) for n in names])
rclpy.shutdown()
