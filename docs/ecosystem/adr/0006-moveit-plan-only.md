# ADR-0006：MoveIt 只规划，执行归仲裁器

- 状态：提议
- 日期：2026-11-23

## 背景

v1.0.0 的 MoveIt 路径可以通过 `/move_action` 或 `/execute_trajectory` 直接执行，
最终到达厂商 `/arm_controller/follow_joint_trajectory`。驱动端只按
`servo_controller.yaml` 的 0..1000 pulse 钳位，不读取 MoveIt 的
`joint_limits.yaml`。因此手写 action 客户端可以把 joint2 命令到 pulse 1000，
而实测机械高 stop 是 795。

`armpi_safety` 只被两个 CLI 工具强制调用；RViz、任意 action 客户端、直接
`FollowJointTrajectory` 都不在环。

## 决策

- MoveIt 在目标架构中**只规划**：`planning_options.plan_only=true` 或
  `/plan_kinematic_path`，返回轨迹。
- 目标架构**不启动** MoveIt 的 controller 执行链路（`arm_controller`/
  `gripper_controller` 不作为运动入口）。
- 轨迹转换为 `jcheng_interfaces/JointTrajectory`，通过 `ExecuteMotion` 提交。
- 执行由 `jcheng_motion` 执行器完成：插值、限幅、总线流式设定点、总线确认。
- 安全节点独立做位置/速度/加速度/工作空间/桌面校验，不依赖规划器结论。
- 未来若需要自碰撞模型，放在规划侧与执行侧双重检查；执行侧不能假设规划侧正确。

## 后果

正面：

- 规划器被替换/被误调用也不能直接驱动硬件。
- 执行侧有独立的限幅与完成判定。
- 规划可以在非特权平面运行。

负面：

- 需要自己实现轨迹插值与流式执行，替代 MoveIt 的 controller 执行。
- 需要维护 `moveit_msgs` <-> `jcheng_interfaces` 转换。
- 原有依赖 MoveIt 执行的脚本需要适配或退役。

## 替代方案

1. **保留 MoveIt 执行，但只允许仲裁器调用 `/move_action`。** 否决：MoveIt
   仍会直发控制器；其他 action 客户端仍可绕过；驱动端不读 MoveIt 限位。
2. **给 MoveIt 控制器加安全插件。** 否决：厂商控制器不可修改；插件边界仍
   无法阻止直接发 `/follow_joint_trajectory`。
3. **只靠 `armpi_safety` 否决。** 否决：它是建议性服务，不是拦截器。

## 证据

- `config/moveit_controllers.yaml:12-34`：MoveIt -> FollowJointTrajectory。
- `joint_trajectory_action_controller.py:146-174`：按时间发送后直接 succeed，
  不读实际位置。
- `joint_position_controller.py:43-48`：只按 0..1000 pulse 钳位。
- `tools/phase2/real_moveit_execute.py:298-308`：只有该工具强制调用安全服务。
