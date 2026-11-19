#!/usr/bin/env python3
"""Report the planning-scene collision objects and whether the current state is valid."""
import rclpy
from moveit_msgs.msg import PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetStateValidity, GetPlanningScene
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('scene_probe')
js = None
def cb(msg):
    global js
    js = msg
node.create_subscription(JointState, '/joint_states', cb, 10)
while js is None:
    rclpy.spin_once(node, timeout_sec=1.0)
names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
vals = [js.position[js.name.index(n)] for n in names]
print('current', dict(zip(names, [round(v, 4) for v in vals])))

rs = RobotState()
rs.joint_state.name = names
rs.joint_state.position = vals

cli = node.create_client(GetStateValidity, '/check_state_validity')
cli.wait_for_service(timeout_sec=10.0)
req = GetStateValidity.Request()
req.robot_state = rs
req.group_name = 'arm'
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
res = fut.result()
print('state valid:', res.valid if res else None, 'contacts:', len(res.contacts) if res else None)
if res:
    for c in res.contacts[:5]:
        print('  contact', c.contact_body_1, c.contact_body_2, round(c.position.x, 4))

msk = PlanningSceneComponents()
msk.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                  | PlanningSceneComponents.WORLD_OBJECT_NAMES
                  | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
cli2 = node.create_client(GetPlanningScene, '/get_planning_scene')
cli2.wait_for_service(timeout_sec=10.0)
req2 = GetPlanningScene.Request()
req2.components = msk
fut2 = cli2.call_async(req2)
rclpy.spin_until_future_complete(node, fut2, timeout_sec=10.0)
sc = fut2.result().scene if fut2.result() else None
if sc:
    print('world objects:', [o.id for o in sc.world.collision_objects])
    for o in sc.world.collision_objects:
        prim = o.primitives[0] if o.primitives else None
        p = o.primitive_poses[0] if o.primitive_poses else None
        print(' ', o.id, 'frame', o.header.frame_id, 'obj_pose', [round(o.pose.position.x,4), round(o.pose.position.y,4), round(o.pose.position.z,4)],
              'type', prim.type if prim else None,
              'dims', [round(d, 4) for d in prim.dimensions] if prim else None,
              'pose', [round(p.position.x, 4), round(p.position.y, 4), round(p.position.z, 4)] if p else None)
    print('attached:', [a.id for a in sc.robot_state.attached_collision_objects])
rclpy.shutdown()
