# JER-59 ArmPi Ultra Sprint 01 — 只读审计与实机集成计划（待 Board 审批）

范围声明：只读审计。未修改代码、未执行 git 写操作、未向任何硬件发送命令。
标记：**[V]** 已在本机验证；**[U]** 未验证（无证据）。
注：仅 Shuhua 一人执行；S03/S05/S06 的专职 Agent 未部署，以下为基于仓库证据的桌面版草案，需实机/厂商资料后复核。

## 1. Current State Report (S01)
- [V] ROS 2 Jazzy（/opt/ros/jazzy），已装 moveit、moveit_resources_panda_*、controller_manager、joint_trajectory_controller、gz_ros2_control。当前 shell 未 source，`ros2` 不在 PATH（需 source 后才能做运行时检查）。
- [V] 仓库包：arm_basics（学习节点/TF）、arm_description（simple_arm 2-DOF + gazebo）、arm_interfaces、arm_moveit_demo（Panda，move group `panda_arm`，plan→execute，已含 execute 失败即停的修复）。
- [V] 仓库内**没有任何 ArmPi Ultra 资产**（grep armpi 仅命中 phase0 文档）：无 URDF/mesh/MoveIt config/驱动。
- [V] 本机 /dev 下无 ttyUSB/ttyACM/video 设备（开发机，非 Pi/ROCK 5T）。实机侧状态全部 [U]。
- [V] Panda Demo 仅证明 MoveIt 流程在仿真/fake controller 下可用，**不能**作为 ArmPi 实机依据。
- 已知缺陷（沿用 JER-39，已再次确认）：
  - P1 `display.launch.py`：模块级使用未定义 `xacro_file_path`（导入即 NameError），且读取不存在的 `simple_arm.urdf`。
  - P1 `simple_arm.xacro:123` 硬编码绝对路径 `/home/jerichocheng/...`。
  - P2 joint2 与 joint1 轴平行（仅学习用）。P3 gazebo bridge 用 `ignition.msgs.Clock`（Jazzy 应为 `gz.msgs.Clock`）。
- 工作区：docs/phase0/ 与 `__pycache__` 为未跟踪文件；未提交。

## 2. Hardware Interface Matrix (S02)
未取得真实接口证据，不假设存在 ros2_control / FollowJointTrajectory / 闭环反馈。

| 项目 | 已知 | 状态 | 取证方法（只读） |
|---|---|---|---|
| 机械臂总线/协议 | 上位机可驱动 | [U] | 阅读厂商 SDK 源码/文档；抓取串口配置（不发送运动命令） |
| 舵机型号/ID/分辨率/限位 | 无 | [U] | 厂商规格书、SDK 常量表 |
| 关节映射（舵机ID↔关节、方向、零位偏移、脉宽↔角度） | 无 | [U] | SDK 源码 |
| 位置反馈（读回角度/负载/温度/电压） | 无 | [U] | SDK 是否有 read API；若仅开环则 ros2_control 须以“命令回显”实现并明确标注非闭环 |
| 控制周期/插值 | 无 | [U] | SDK 的 duration 参数含义（舵机内部插值 vs 主机插值） |
| 异常处理（堵转、过温、掉电、超限） | 无 | [U] | SDK/舵机手册 |
| 夹爪 | 无 | [U] | 行程、力限制、是否可读回 |
| 相机 | 无 | [U] | 型号、接口、安装位姿、内参、标定 |
| 上位机运行位置 | Pi 5 / ROCK 5T 之一 | [U] | 需 Board 告知主控板 |

## 3. ROS 2 Integration Architecture（草案）
分层，硬件细节隔离在最底层，其余可先用仿真/mock 验证：
1. `armpi_ultra_description`（URDF/xacro、TF、limits，参数标 provisional）。
2. `armpi_ultra_hardware`：ros2_control `SystemInterface` 插件，封装厂商驱动；先实现 **mock/fake 模式**（`mock_components`），再接真机。反馈能力由 S02 结论决定（有读回→闭环 state；无→明确标注 estimated state）。
3. `joint_trajectory_controller` + `joint_state_broadcaster`（暴露 FollowJointTrajectory，MoveIt 通过它执行）。
4. `armpi_ultra_moveit_config`（5-DoF 规划组 + 夹爪组）。
5. 底盘：独立 `armpi_base_driver`（`/cmd_vel`→电机，`/odom`），与机械臂分离；本轮无 Nav2。
6. 安全层：软件限位、速度缩放上限、急停话题/看门狗、命令超时归零。
运行拓扑 [U]：Pi 5 与 ROCK 5T 分工待 Board 决定（建议：单一机载主机跑 ros2_control，另一块做视觉，经 DDS 通信）。

## 4. URDF / TF Repair Plan (S03，桌面草案)
1. 新建独立包，不动 Panda 与 simple_arm 基线。
2. 来源：厂商 URDF/CAD（[U] 需 Board 提供）；无则由 CAD 测量建模，所有尺寸标 provisional。
3. 必查项：5 个旋转关节轴/零位/限位（须与舵机限位及机械干涉一致）、夹爪（mimic 或单关节 + TCP 帧 `tool0`）、`camera_link` 与 `camera_optical_frame`（REP-103：optical z 前、x 右、y 下）、惯量用于仿真即可、碰撞体简化。
4. 修复 simple_arm 的 P1 launch/绝对路径问题（可选，需 Board 批准）。
5. 验收：`xacro`→`check_urdf`、`view_frames` 单树无环、RViz 关节扫动与实物对照（实物对照需 Board 批准的人工操作）。

## 5. MoveIt 2 Implementation Plan (S04)
- 5-DoF 臂无法实现任意 6D 位姿 → 采用**约束抓取**：目标位姿降为（x,y,z,yaw）+ 固定/限定俯仰（如竖直向下或一个小范围），使用 IK 对 5-DoF 可解的位姿子空间；规划器用 `PositionConstraint` + `OrientationConstraint`（自由 yaw/roll 容差），或自定义 IK/pose generator 枚举抓取候选后调用 plan。
- IK：默认 KDL 对 5-DoF 易失败，评估 pick_ik 或 analytical 解 [U 需实测]。
- 流程：预抓取（上方）→ 直线下降（cartesian path）→ 夹爪闭合 → 抬升。沿用现有 demo 的“失败即停”。
- 阶段：A 仿真（mock hardware）验证规划；B 实机限速（≤20% 速度缩放）空载；C 带物抓取。
- 依赖：S03 模型与 S02 控制接口；MoveIt 配置包用 Setup Assistant 生成后手工校对。

## 6. Safety & QA Test Matrix (S05/S06)
| 级别 | 测试 | 通过标准 | 前置/审批 |
|---|---|---|---|
| L1 仿真 | xacro/check_urdf；TF 树；MoveIt plan/execute（mock）；约束抓取可达性统计；失败路径（规划失败/执行失败停止） | 全通过，无 NameError/绝对路径 | 无硬件 |
| L2 接口 | 驱动单测（映射、限位裁剪、超时归零）；ros2_control 加载 mock；话题/动作类型检查；底盘 `/odom` 回放 | 与 Hardware Matrix 一致 | 需 S02 证据 |
| L3 实机 | 臂：上电空载、单关节小幅（≤5°）、轨迹跟踪误差、急停、断连行为；底盘：架空轮子测速→编码器→odom 一致性→地面低速（≤0.1 m/s）；供电压降/过温 | 分项通过 | **每一步需 Board 批准**，人在现场、有物理断电 |
- 底盘安全 [U]：编码器有无、供电（电机与计算共电会导致掉电重启）、急停、cmd_vel 超时。
- 厂商上位机测试结果不计入 L3。

## 7. Prioritized Task Backlog（待批准后）
1. P0 获取厂商 SDK/文档/URDF（Board）；确认 ROS 主控板。
2. P0 S02 驱动源码只读分析 → 填满接口矩阵。
3. P1 `armpi_ultra_description` + display launch（含 mock）。
4. P1 `armpi_ultra_hardware` mock 版 + controllers。
5. P1 MoveIt config + 约束抓取原型（仿真）。
6. P2 安全层（限位/看门狗/急停）。
7. P2 底盘驱动与 odom（只读协议调研先行）。
8. P2 修复 simple_arm 缺陷 / gz.msgs。
9. P3 相机标定与 optical frame 校验。
10. P3 L3 实机测试脚本与检查单。

## 8. Board Decision Requests
1. 提供 ArmPi Ultra 厂商资料（SDK/源码/URDF/CAD/规格书）的路径或下载许可；以及上位机与硬件所在主机（Pi 5 vs ROCK 5T）的访问方式。
2. 批准下一轮代码修改范围：新建 `armpi_ultra_description` / `armpi_ultra_hardware`（mock 优先）— 是/否。
3. 是否纳入 simple_arm 的 P1 缺陷修复。
4. 批准“只读接口取证”是否允许在实机主机上以只读方式运行（列出串口/读取传感器反馈，**不发运动命令**）；任何实机运动测试需另行审批。
5. 是否部署 ROS Engineer / QA / Hardware Agent（当前未部署，S03/S05/S06 由我代做草案）。
6. 对 5-DoF 约束抓取接受“固定俯仰角”这一限制。

审计完成，暂停，等待 Board 审批。
