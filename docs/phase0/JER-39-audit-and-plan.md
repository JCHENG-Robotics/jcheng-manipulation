# JER-39 Phase 0 只读审计与实施方案（待 Board 审批）

状态：只读审计完成，**未修改任何代码，未执行任何 git 写操作**。

## 1. 阻塞/前提问题
- `MASTER_PLAN_V3_PRIVATE.md`、`ROADMAP_PUBLIC.md` 在仓库及全盘搜索中均**不存在**（仅有 `ROADMAP.md`，为 8 周学习路线）。
- 仓库中**没有任何 ArmPi Ultra 资产**（无 URDF/Xacro/mesh/MoveIt config）。`grep -i armpi` 无命中。
- 因此“模型审计”目前无对象。需要 Board 提供：(a) 两份计划文件；(b) ArmPi Ultra 官方 URDF/mesh/资料来源（Hiwonder 提供的包或 CAD）。

## 2. 环境（已验证）
- ROS 2 Jazzy 在 /opt/ros/jazzy；已装 moveit、moveit_resources_panda_description、gz_ros2_control、ros_gz。
- `xacro simple_arm.xacro` 成功；`check_urdf` 解析通过（base_link→link1→link2）。

## 3. 现有仓库结构
`ros2_ws/src`: arm_basics（学习节点/TF）、arm_description（simple_arm 2-DOF 示例 + gazebo launch）、arm_interfaces、arm_moveit_demo（Panda 基线，move group `panda_arm`）。

## 4. 缺陷清单（针对现有 arm_description，按优先级）
已确认缺陷：
- P1 `display.launch.py`：模块级引用未定义变量 `xacro_file_path`（导入即 NameError）；且读取不存在的 `simple_arm.urdf`（实际仅有 .xacro）。RViz 流程无法启动。
- P1 `simple_arm.xacro`：gazebo 插件 `<parameters>` 硬编码绝对路径 `/home/jerichocheng/...install/...`，不可移植。
- P2 joint2 轴为 z 轴，与 joint1 平行，运动学上冗余（仅学习用，非 ArmPi）。
- P2 `arm_description/CMakeLists.txt` 需确认是否安装 urdf/launch/config 目录（审计中未见完整安装规则，实施前确认）。
- P3 gazebo.launch.py 的 clock bridge 使用 `ignition.msgs.Clock`，Jazzy 应为 `gz.msgs.Clock`。
待验证参数（provisional）：全部关节限位/速度/力矩、惯量（圆柱近似）、质量。
未知硬件约束：ArmPi Ultra 真实自由度、舵机限位与分辨率、相机型号与安装位姿、夹爪行程——均需资料或实机。

## 5. 最小修改方案（批准后实施）
1. 新建独立包 `armpi_ultra_description`（不动 Panda 与 simple_arm 学习基线）：xacro + 命名空间化参数，所有限位/相机位姿/动力学标注 `provisional`（xacro 注释 + 参数文件 `provisional: true` 段）。
2. `display.launch.py`：xacro 处理 + robot_state_publisher + joint_state_publisher_gui + rviz2 配置，路径用 `get_package_share_directory`。
3. 仅修复 simple_arm 的 P1 launch 缺陷（可选，需 Board 确认是否纳入）。
4. 不含底盘、Nav2、高级算法；MoveIt/Gazebo 留给后续阶段。

## 6. QA 验收命令（独立 QA 执行）
```
xacro <pkg>/urdf/armpi_ultra.urdf.xacro > /tmp/a.urdf && check_urdf /tmp/a.urdf
colcon build --packages-select armpi_ultra_description
ros2 launch armpi_ultra_description display.launch.py
ros2 run tf2_tools view_frames   # 检查 TF 树单根、无环
ros2 topic echo /joint_states --once
```
检查项：模型加载、TF 连贯、关节类型（revolute/prismatic/fixed 与资料一致）、基础启动。

## 7. 清单
- 已完成：环境与仓库只读审计、缺陷识别、方案草案。
- 未完成：ArmPi 模型（缺资产）、实施、QA。
- 待实机验证：关节限位、相机安装位姿、动力学参数、夹爪行程。

## 8. 请 Board 决策
1. 提供 `MASTER_PLAN_V3_PRIVATE.md`、`ROADMAP_PUBLIC.md` 与 ArmPi Ultra URDF/mesh 来源。
2. 批准第 5 节方案（含是否修复 simple_arm P1 缺陷）。
