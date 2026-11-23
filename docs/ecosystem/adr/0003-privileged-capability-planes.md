# ADR-0003：特权/能力平面与 DDS 分域

- 状态：提议
- 日期：2026-11-23

## 背景

ROS 2/DDS 默认没有发布者鉴权。v1.0.0 的局域网内任何节点都可以直接发布
`/servo_controller`、`/ros_robot_controller/bus_servo/set_position`、
`/ros_robot_controller/bus_servo/set_state`、`/ros_robot_controller/set_motor_speeds`、
`/arm_controller/follow_joint_trajectory`、`/cmd_vel`、`/grasp`。
网关的 HTTP token 只保护 HTTP，不保护 ROS 话题。

`SetBusServoState` 还能改 ID、offset、角度限位、力矩并写 EEPROM，能直接破坏
“EEPROM 限位 − 20 pulse”的安全前提。

## 决策

把运行时分为两个平面，并用 DDS 域隔离：

- **特权运动平面（域 41）**：`jcheng_safety`、`jcheng_motion` 执行器、
  `jcheng_guard`、`jcheng_telemetry`。
- **能力平面（域 42）**：`jcheng_motion` 前端、`jcheng_authority`、
  `jcheng_gateway`、规划、遥操作、学习、应用。
- **厂商域（域 0）**：厂商节点保持不变；只有 `jcheng_hardware` 双上下文加入。
- **仿真域（域 43）**：Mock/CI。

配套：

- 能力容器无串口设备、无 host 网络、只读根、独立用户。
- 运行时 `jcheng_guard` 监控硬件命令话题的发布者白名单；违规锁存停止。
- v1.0 目标加入 SROS2 签名权限；未完成则明确标注 enforcement level B。
- 运行时**不暴露** `SetBusServoState`；维护写操作走独立审批流程。

## 后果

正面：

- 能力模块默认看不到硬件话题；误配置不会直接变成运动。
- 即使某个模块被替换，也无法加入域 0。
- 安全边界从“代码约定”变成“网络/进程事实”。

负面：

- 多域增加部署复杂度与发现配置。
- 双上下文进程（硬件桥、仲裁器）需要仔细测试。
- SROS2 证书/权限运维成本。

## 替代方案

1. **单域 + 代码评审。** 否决：无法防止旧脚本、第三方包、误配置。
2. **只用 HTTP 网关，禁止 ROS 模块。** 否决：遥操作/学习需要 ROS 本地性；
   且网关本身仍要连 ROS。
3. **在厂商侧改驱动加鉴权。** 否决：厂商代码只读，不可修改；升级不可控。

## 证据

- `docs/hardware/network_access.md:19-28`：DDS 无认证，交换机上任意主机可发布。
- `tools/net/armpi-lockdown.nft`：只做 IP/端口过滤，不区分 ROS 发布者。
- `ros_robot_controller_node.py:178-215`：`SetBusServoState` 无守卫。
- `gateway/node.py:134`：`safety_stop` 初值 false。
