# PROJECT_STATUS_AUDIT — 模块真实完成度、验证证据与不确定事项

> 审计基线：`MASTER_PLAN_V3_PRIVATE.md`、`ROADMAP_PUBLIC.md`（USB `/media/jerichocheng/CC98-6092`，2026-10-26）
> 辅助基线：`ARMPI_DEBUG_STUDIO_PRD_V1.md`、`ArmPiUltra/docs/g0/`、仓库 README/ROADMAP
> 状态标签：
> `[V]` 本轮已由文件/哈希/命令/日志验证；`[S]` 仅来自项目自述或历史报告（未重跑）；`[U]` 未知/无证据。
> 任务状态：**已实现且已验证 / 已实现未验证 / 部分实现 / 仅有计划 / 阻塞 / 无法确认**。

## 0. 执行摘要

| 结论 | 状态 |
|---|---|
| Panda baseline 冻结 | **已完成**：`panda-baseline-v1` tag 指向 `435f8ec`，已推送 GitHub。但 Phase II Week 1 的完整可靠性工作仍停留在未合并分支，**不在冻结基线和 tag 内** |
| ArmPi URDF / joint / limits / TF | **部分实现**：xacro 可展开、`check_urdf` 通过、joint1–5 为 revolute、TF 单根；但限位是占位值、camera optical frame 缺失、陈旧 URDF 仍随包安装 |
| 5-DoF 约束规划 / MoveIt 2 | **部分实现**：MoveIt 配置存在，mock hardware 下关节空间轨迹 3 次执行成功；TRAC-IK 插件缺失、IK 未实例化、无约束位姿规划 |
| 相机安装 / 标定 | **未实现**：只有厂商 `camera_connet_link`，无 `camera_link` / `camera_optical_frame`，无标定数据 |
| ROS 2 控制接口 / 硬件集成 | **未实现（ArmPi）**：无 ros2_control 硬件插件、无串口/实机证据；厂商栈仅完成源码审计 |
| 视觉定位 / 抓取 | **未开始**：无 AprilTag / 手眼 / 抓取代码；厂商 demo 不计为本人成果 |
| 实验 / 测试 / 误差预算 | **初步**：M2.2 是 2-DOF `simple_arm` Gazebo 调参，不是 ArmPi/Error Budget v1；Studio 只有 Mock 测试；无重复性/回差/容差实验 |
| README / 技术文档 | **不一致**：旧 8 周文档、V3 计划在 USB、PRD/G0 在本地、Studio 文档在 M70q、M2.2 无文档 |

**总判断**：项目已从"学习/Panda 仿真"进入"ArmPi 模型与 MoveIt 仿真"的门槛，但**尚未建立可信的真实 ArmPi 基线**。MASTER_PLAN 的 Phase 0 只完成了一部分；Phase 1 之后（硬件基线、标定、视觉抓取、误差预算）均未开始或仅停留在计划。

> **审计边界**：并行会话 `sess_83d860c9` 在审计期间（11:20–11:24）新建了 `/home/jerichocheng/Projects_M70q/armpi_ultra_ws_clean`（28M/248 文件/12 包，无 Git）。它属于"厂商精简/重构"的另一条任务线，**不在本文件的任务级核对范围内**；会话仍在 running，内容可能继续变化。其最终成果需单独审计。

## 1. 规划基线（实际读取内容）

### 1.1 MASTER_PLAN 的关键要求

| 章节 | 要求 | 对当前项目的含义 |
|---|---|---|
| §2 | Must-have：真实控制、模型/TF、自研标定、视觉抓取、Error Budget、容差包络、干预、对比、报告 | 当前 0 项完成 |
| §3 | ArmPi 视为 5-DoF；禁止任意 6D 位姿；约束抓取族 | 尚未实现约束抓取 |
| §4 | MoveIt2 规划 + 解析/运动学服务；position-only IK 只能做可达性检查 | 当前 MoveIt 配置 `position_only_ik: true`，且 IK 插件缺失 |
| §5 | Phase 0：冻结 Panda；修 joint1 fixed、限位/effort 占位、命名/空格、gripper mimic、camera optical frame、安装变换 | 部分完成，见 §3.1/§3.2 |
| §6 | 硬件到达后 2 周：环境、关节名、限位、反馈、camera、gripper、安全、vendor baseline；输出 `docs/hardware/*` | 未开始 |
| §7 | 相机装在 `link4`，静态变换是 `link4 → camera`，不是 gripper；TF 链 `link4 → camera_mount → camera_link → camera_optical_frame` | 厂商模型只有 `camera_connet_link`，缺 optical frame |
| §8–§12 | 标定 A/B、视觉抓取、Error Budget、重复性/回差、视觉相对位移、容差包络 | 未开始 |
| §17 | 安全层：限速、限位、超时、看门狗、停止、碰撞、断电 | 仅 Studio Mock 层有部分安全语义；实机无 |

### 1.2 ROADMAP_PUBLIC 的里程碑

Milestone A 硬件基线 → B 标定 → C 真实视觉抓取 → D Error Budget → E 可靠性干预。当前**连 Milestone A 的"真实硬件基线"都未完成**。

### 1.3 PRD/G0 的 G 阶段

G0 Discovery 完成文档；G1 Safe Debug MVP 需要"有人看护的实机验收通过"。当前 Studio 只有 G1 Mock 切片，**G1 门禁未通过**。

## 2. 模块状态总表

| # | 模块 | 状态 | 已验证证据 | 主要缺口 |
|---|---|---|---|---|
| 1 | Panda baseline 冻结 | **已实现且已验证**（tag）/ 分支工作未合并 | `panda-baseline-v1` → `435f8ec`；GitHub 有 tag；`main` 51 commits；README/demo/weekly-log | Phase II 分支未合并；ROADMAP 有 4 个已完成但未勾选项 |
| 2 | ArmPi URDF / joints / limits / TF | **部分实现** | xacro 展开 15 links/14 joints；`check_urdf` 通过；单根 `base_link`；joint1–5 revolute；gripper 5 mimic `[V]` | 限位 ±2.09/effort 1000 占位；无 provisional 标记；无 optical frame；陈旧 URDF `joint1/r_joint fixed`；joint3 软件范围冲突；initial_positions 不符 |
| 3 | 5-DoF 约束规划 / MoveIt 2 | **部分实现** | MoveIt config；mock hardware；2026-11-14 13:53–13:58 三条 arm 轨迹 `SUCCEEDED` `[V]` | TRAC-IK 插件缺失；IK 未实例化；`position_only_ik: true`；无约束位姿/候选搜索；gripper 规划因缺加速度限制失败 |
| 4 | 相机安装 / 标定 | **未实现** | 厂商 `camera_connet_joint`（link4→camera_connet_link，xyz/rpy 可见）`[V]` | 无 `camera_link`/`camera_optical_frame`；无内参/手眼/标定数据；安装变换未实测 |
| 5 | ROS 2 控制 / 硬件集成 | **未实现（ArmPi）** | 厂商 SDK/Driver 源码审计（G0）`[S]`；本机无 `/dev/rrc`、`ttyACM`、`video` `[V]` | 无 ros2_control 硬件插件；无串口/反馈；SD Docker 不可读；无实机连接 |
| 6 | 视觉定位 / 抓取 | **仅有计划** | 无代码/数据 | 无 AprilTag、手眼、抓取候选、MoveIt 集成 |
| 7 | 实验 / 测试 / 误差预算 | **部分实现（初步）** | M2.2 四个 bag + 分析 `[V]`；Studio 55 events/1 command `[V]`；41 测试自述 `[S]` | 2-DOF simple_arm、Gazebo，不是 ArmPi；无 Error Budget v1/容差/重复性/回差 |
| 8 | README / 技术文档 | **部分实现且不一致** | 旧 README/ROADMAP、PRD、G0、Studio README、M2.2 报告均存在 | 无统一状态；无 `docs/hardware/*`；V3 计划不在仓库；旧文档误导 |

## 3. 逐模块证据与判断

### 3.1 Panda baseline 冻结

**已确认 `[V]`：**

- `panda-baseline-v1` 是 annotated tag：`b116577` → commit `435f8ec`；GitHub 上同时存在 tag object 和 peeled commit。
- `v0.1.0`：`464026f` → `eb08b9b`。
- `main` 共 51 个提交，最后为 2026-11-08 `Merge JER-17`。
- 仓库有 66 个跟踪文件；README、`docs/demo.gif`、`docs/demo.mp4`、`docs/weekly-log/week01–08.md` 均存在。
- 未发现 tag 之后的 Panda 新功能提交；M2.2 是另一条 `simple_arm` 实验线。

**但 `[V]` 也确认：**

- `origin/phase2/week01-execution-reliability`（`e596c5a`）有 **4 个未合并提交**，10 文件 483+/161-，包含：
  - pick/place 位姿参数化 + launch 覆盖 + 7 元素校验；
  - typed controller action-readiness gate（`panda_arm_controller/follow_joint_trajectory`、`panda_hand_controller/gripper_cmd`）；
  - plan/execute 失败退出；
  - `CHANGELOG.md`、`docs/evolution/phase2/week01.md`、`docs/learning-roadmap.md`、ROADMAP 重构。
- 分支文档自述："Remaining: Run default, planning-failure, and execution-failure regressions in a ROS 2 Jazzy + MoveIt 2 environment."
- `origin/agent/JER-18-remediation`（`1bbb136`）是 phase2 代码的子集，未合并。
- `main` 只包含 `616ac80` 的 13 行"执行失败即停"修复。

**判断：** Panda 冻结动作本身完成；Phase II Week 1 的成果是"实现完成、运行回归未完成、未进入冻结基线"。这是一个**需要 Board 决定**的孤儿成果，不应被无声丢弃，也不应自动合并。

**文档矛盾 `[V]`：** `ROADMAP.md` 仍有 4 个未勾选项（pick_place_demo、单 launch、README、demo video），这 4 项实际均已完成。

### 3.2 ArmPi URDF / 关节 / 限位 / TF

**模型展开验证 `[V]`：**

```text
xacro ~/armpi_sim_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro
→ 15 links, 14 joints
→ check_urdf: Successfully Parsed XML
→ root: base_link（单根）
```

关节表（xacro 展开结果）：

| 关节 | 类型 | 父→子 | 轴 | 限位 (rad) | effort | velocity |
|---|---|---|---|---|---|---:|
| joint1 | revolute | base_link→link1 | 0 0 1 | -2.09…2.09 | 1000 | 10 |
| joint2 | revolute | link1→link2 | -0.0010551 -0.99998 0.0068419 | -2.09…2.09 | 1000 | 10 |
| joint3 | revolute | link2→link3 | 0.000275 -1 -0.00032517 | -2.09…2.09 | 1000 | 10 |
| joint4 | revolute | link3→link4 | 0.00040702 -1 -0.000213 | -2.09…2.09 | 1000 | 10 |
| joint5 | revolute | link4→link5 | -0.012028 0.0031146 -0.99992 | -2.09…2.09 | 1000 | 10 |
| gripper_joint | fixed | link5→gripper_link | — | — | — | — |
| end_effector_joint | fixed | link5→end_effector_link | — | — | — | — |
| camera_connet_joint | fixed | link4→camera_connet_link | — | — | — | — |
| r_joint | revolute | gripper_link→r_link | 0.99996 -0.004699 -0.0070434 | -1.57…1.57 | 1000 | 10 |
| l_joint / l_in_joint | revolute | gripper_link→… | ≈ +x | -1.57…1.57 | 1000 | 10 |
| l_out_joint | revolute | l_in_link→l_out_link | ≈ +x | -1.57…1.57 | 1000 | 10 |
| r_in_joint | revolute | gripper_link→r_in_link | ≈ -x | -1.57…1.57 | 1000 | 10 |
| r_out_joint | revolute | r_in_link→r_out_link | ≈ -x | -1.57…1.57 | 1000 | 10 |

mimic：`l_joint`、`l_in_joint`、`l_out_joint`、`r_in_joint`、`r_out_joint` 均 mimic `r_joint`（multiplier ±1）。

**已满足的 Phase 0 项：**

- RViz/MoveIt 能加载模型 `[V]`（2026-11-14 运行日志）。
- TF 树单根、无环 `[V]`（`check_urdf` + root/leaf 检查）。
- xacro 中 joint1–joint5 均为 revolute `[V]`。
- gripper 有 mimic 链 `[V]`。

**未满足/风险项：**

1. **限位是占位值**：所有臂关节 ±2.09，effort 1000，velocity 10；没有 `provisional` 标记。真实机械限位未测。
2. **joint3 软件范围冲突**：厂商 `servo_controller.yaml` 为 `min=1000, max=500`（flipped），换算范围 **-2.094…0.000**；URDF 却允许 **+2.09**。MoveIt 可能规划出会被厂商层静默裁剪的正角度。
3. **r_joint 冲突**：URDF ±1.57；软件范围 **-2.932…+1.257**。MoveIt 初始 `r_joint=1.57` 已超出软件上限 1.257。
4. **camera optical frame 缺失**：只有 `camera_connet_link`；无 `camera_link`、`camera_optical_frame`，不满足 REP-103 与 MASTER_PLAN §7。
5. **陈旧 URDF**：同包内 `urdf/armpi_ultra.urdf`（`00dc26…`）把 `joint1`、`r_joint` 定义为 **fixed**，且无 mimic 链。当前 launch 都用 xacro，所以运行未受影响；但该文件被 `setup.py` 安装，属于危险遗留物。
6. **initial_positions 与真实启动姿态不符**：见 `CHANGE_RECONCILIATION.md` §3.9；r_joint 方向/量级均错。
7. **没有 `docs/hardware/*`**：MASTER_PLAN §6 要求的 7 份硬件文档均不存在。

**判断：部分实现。** "模型能加载"不等于"模型可用于实机规划"。

### 3.3 5-DoF 约束运动规划与 MoveIt 2

**已确认 `[V]`：**

- MoveIt config：`armpi_ultra_moveit_config`，group `arm` 包含 joint1–joint5；group `gripper` 包含 r_joint 及 mimic 关节；`end_effector` parent_link=`link5`。
- `ros2_control` 使用 `mock_components/GenericSystem`，6 个主动关节（joint1–5 + r_joint）。
- 2026-11-14 13:53–13:58 运行：
  - `move_group`、`ros2_control_node`、`rviz2` 启动；
  - `arm_controller` 与 `gripper_controller` 激活；
  - 三次 arm 轨迹：`Goal request accepted!` → `Controller 'arm_controller' successfully finished` → `SUCCEEDED`；
  - 日志显示 `Motion plan was computed successfully`。

**未满足 `[V]`：**

- `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` **加载失败**；`/opt/ros/jazzy/share` 中没有 `trac_ik` 包；`Kinematics solver could not be instantiated for joint group arm`。
- 因此 IK 未实例化；当前成功的轨迹是**关节空间目标**，不是笛卡尔/约束位姿目标。
- `kinematics.yaml` 设置 `position_only_ik: true`，与 MASTER_PLAN §4（position-only 仅用于可达性）冲突；不能作为最终抓取 IK。
- 没有 `PositionConstraint` + `OrientationConstraint`、没有抓取候选生成、没有 approach pitch / wrist rotation 搜索。
- gripper 规划一次失败：`No acceleration limit was defined for joint l_in_joint!`（`joint_limits.yaml` 未给 gripper 加速度）。
- 没有碰撞对象/世界、没有相机、没有抓取/放置序列。

**判断：部分实现。** 5-DoF 的"运动学可规划"尚未证明；只证明了"固定关节空间目标可在 mock hardware 执行"。

### 3.4 相机安装坐标系与标定方案

**已确认 `[V]`：**

- 厂商 xacro 中 `camera_connet_joint`：parent `link4`，child `camera_connet_link`，`xyz="-0.040352 -0.019122 0.048005"`，`rpy="0 -1.5614 0"`。
- 这是相机支架的静态变换，位于 joint5 之前，符合 MASTER_PLAN §7 的"link4 前"几何。

**未满足：**

- 无 `camera_link` / `camera_optical_frame`；无 `camera_mount` 命名。
- 无相机内参、无手目标定、无标定数据/验证。
- 无相机话题验证（本机无 `/dev/video*`；未连接深度相机）。
- 无 `docs/hardware/camera_mount.md`。

**判断：仅有厂商几何，未实现本项目标定方案。**

### 3.5 ROS 2 控制接口与硬件集成

**已确认 `[V]`：**

- 本机 `/dev/rrc`、`/dev/ttyACM*`、`/dev/ttyUSB*`、`/dev/video*` 均不存在。
- SD 卡 rootfs 中没有 `ros2_ws`/项目代码；`/var/lib/docker` 与 `/root` 为 root-only，无法审计容器内 `/home/ubuntu/ros2_ws`。
- 机器人 SSH 端点不可达；Tailscale 无机器人节点。
- 仓库没有 ArmPi 驱动/hardware 插件；Studio `hardware.py` 是只读 stub，`hardware_motion=false`。

**厂商侧 `[S]`（G0 源码审计，未实机验证）：**

- `/controller_manager/joint_states` 是最后下发脉冲的换算，不是实测反馈。
- `bus_servo_set_position()` 无 ACK，命令最多到 `Dispatched`。
- SDK 读取 `queue.get(block=True)` 无超时，可能冻结单线程 Driver。
- 厂商 MoveIt config 使用 `mock_components`；真实入口是开环 `FollowJointTrajectory`。
- 已知 Driver 缺陷：`get_torque_state` 调不存在的方法、`pub_button_data` 未导入枚举、`/joint_controller` 回调类型错误、轨迹 Action 不校验实际位置。

**判断：ArmPi 的 ROS 2 控制/硬件集成未实现。** 仅完成厂商源码审计；真实硬件基线被"无法访问 Docker/机器人"阻塞。

### 3.6 视觉定位与抓取流程

- 无 AprilTag/fiducial 检测、无 `link4→camera` TF 使用、无目标位姿生成、无约束抓取候选、无 IK、无 MoveIt 执行、无抓取验证。
- 厂商 `software/hand2cam_tf_matrix_software`、`app/calibration.py`、`app/grasp.py` 存在，但属于 vendor baseline，不能计为本人实现。
- Studio G4/Visual Workbench 未实现。
- MASTER_PLAN §8 的 Phase 2 里程碑"Real Level A"未开始。

**判断：仅有计划。**

### 3.7 实验、测试与误差预算

#### 3.7.1 M2.2（2-DOF `simple_arm` Gazebo）

- 4 个 mcap：`trajectory_test_01`（19:31:26，46.2s）、`1p5s`（19:42:23，21.1s）、`settling`（19:46:02，23.3s）、`gain_0p2/settling`（20:59:25，133.2s）`[V]`。
- 分析与图：`analysis/`、`analysis_1p5s/`、`analysis_settling/`、`m2_2_system_identification.png` `[V]`。
- 重算统计见 `CHANGE_RECONCILIATION.md` §4.1。
- 结论边界：
  - 模型是 **2-DOF simple_arm**，不是 ArmPi Ultra；
  - 环境是 Gazebo + mock/position controller，不是实机；
  - gain 0.2 的 settling 末端误差为 0，`>1 mrad` 最晚 0.2s；
  - 这些是**控制器调参/系统辨识的初步数据**，不是 MASTER_PLAN §9 的 Error Budget v1。
- rework1：9 tests / 0 failures（自述 `[S]`），但**未运行 Gazebo/ROS**；bags 早于 rework1，不能互相证明。

#### 3.7.2 Studio G1 Mock

- 28 py + 16 ts/tsx；4,178 + 2,895 行 `[V]`。
- 37 个测试函数 + 2 parametrize；README 声称 41 用例；未重跑 `[S]`。
- `data/studio.db`：55 events / 1 command `[V]`，与一次 E2E 运行一致。
- 无 Playwright 文件 `[V]`；`g1-verification.md` 的 9 项 browser PASS 不可复现。
- E7 ROS gateway、watchdog/faults/gateway/realtime 模块缺失 `[V]`。
- PRD G1 的"实机验收"未发生。

#### 3.7.3 Panda demo

- README 描述 open → pick → close → place；`docs/demo.gif`/`demo.mp4` 存在；weekly-log week08 有调试记录。
- 本轮未重跑；按项目记录为"仿真已验证" `[S]`。

#### 3.7.4 缺失的实验能力

- 无真实 ArmPi 重复性、回差/迟滞、标定误差、视觉相对位移误差、抓取执行误差、夹爪容差包络。
- 无 paired randomized trials、无 `pair_id`/`trial_order`/失败编码。
- 无 `docs/hardware/*`、无实验数据目录规范。

**判断：只有初步仿真调参和 Mock 测试；距 Error Budget v1 很远。**

### 3.8 README、技术文档与开发记录

| 文档 | 位置 | 状态 |
|---|---|---|
| `README.md` | S01/S02 仓库 | 8 周学习项目、WSL2；未提 ArmPi/M2.2/Studio/Phase 0 |
| `ROADMAP.md` | S01/S02 仓库 | 旧 8 周路线；4 个已完成项未勾选；与 V3 计划无关 |
| `MASTER_PLAN_V3_PRIVATE.md` | USB only | 私有计划；单副本；未进仓库（符合保密要求） |
| `ROADMAP_PUBLIC.md` | USB only | 公开路线；未进仓库 |
| `ARMPI_DEBUG_STUDIO_PRD_V1.md` | S07 + 微信文件 | Proposed；未进仓库 |
| `docs/g0/README + 01–07` | S07 | Analysis complete / 待 Board 评审；未进仓库 |
| Studio `README.md` / `docs/g1-verification.md` | S02 | 记录 G1 Mock；未提交；Playwright 无文件 |
| M2.2 报告 | S03 | 只在 worktree；未进仓库/文档 |
| `docs/hardware/*` | — | **全部缺失**（MASTER_PLAN §6 要求 7 份） |

**判断：文档碎片化严重，且存在多处"文档说未做、实际已做"或"文档说完成、实际未验证"。**

## 4. 任务状态分类

### 4.1 已实现且已验证（限仿真/Mock）

| 任务 | 验证级别 | 证据 |
|---|---|---|
| Panda 8 周仿真抓取 demo | 仿真 | README、demo.gif/mp4、weekly-log、tag |
| ArmPi 模型在 RViz/MoveIt 加载 | 仿真 | `check_urdf`、2026-11-14 运行日志 |
| ArmPi 关节空间轨迹执行 | 仿真（mock hardware） | 3 次 `SUCCEEDED` |
| M2.2 gain/settling 数据采集 | 仿真（2-DOF Gazebo） | 4 个 mcap + CSV/PNG |
| Studio Mock 端到端 | Mock | 41 测试自述、55 events/1 command |

### 4.2 已实现但未验证

| 任务 | 证据 | 缺什么 |
|---|---|---|
| Studio G1 Mock 切片 | 代码/测试/DB | 独立重跑、Playwright 文件、E7、PRD G1 实机验收 |
| M2.2 Gate 2 rework1 | 4 文件 + helper + 测试 + 离线报告 | Gazebo/ROS 运行时回归；bags 早于 rework1 |
| Phase II Week 1（Panda） | 未合并分支 4 commits + 文档 | 合并、运行回归、是否纳入冻结基线 |
| ArmPi 模型修正（joint1/rev） | xacro | 限位/effort 实测、optical frame、陈旧 URDF 清理 |

### 4.3 部分实现

| 任务 | 已完成 | 未完成 |
|---|---|---|
| ArmPi Phase 0 | 模型加载、TF、joint 类型、mimic | 限位、optical frame、provisional、约束规划、5-DoF 文档 |
| 5-DoF MoveIt | 配置、mock 执行 | IK 插件、约束位姿、候选搜索、gripper 限位 |
| Studio G1 | E1–E6 大部分（Mock） | E7 gateway、watchdog/faults 模块、E8 Playwright、E9 实机 |
| M2.2 | 调参/数据/离线测试 | 与 ArmPi 无关；未进主文档；未运行时验证 rework1 |

### 4.4 仅有计划

- 5-DoF 约束抓取（位置 + approach pitch + wrist rotation + offset）。
- 自研手眼标定 + 厂商 baseline 对比。
- 真实视觉抓取（AprilTag → link4/camera TF → IK → MoveIt → 验证）。
- Error Budget v1、重复性/回差、视觉相对位移、夹爪容差包络。
- 移动底盘 / odom / Nav2。
- `docs/hardware/*`。

### 4.5 阻塞

| 阻塞项 | 原因 |
|---|---|
| 真实 ArmPi 硬件基线 | 机器人 SSH 不可达；SD Docker/`root` 不可读；无 `/dev/rrc` |
| G0 文档的 D-10 反馈真实性测试 | 需 Board 批准 + 实机 + 人工看护 |
| 厂商 Docker 工作区审计 | root-only，当前用户无权限 |
| 实机运动/标定/抓取 | 需 Board 授权，且模型/限位/反馈尚未验证 |
| 合并执行 | 本轮只审计，未获批准 |

### 4.6 无法确认

- 真实关节限位、effort、回差、重复性。
- `joint_states` 的真实语义（G0 判定为命令回显；未实机复核）。
- 串口/驱动在真实系统上的行为、OOM 根因。
- 相机型号、内参、实际安装变换。
- 底盘/编码器/急停状态。
- Paperclip 任务 DB 的完整任务状态（未查询）。

## 5. 实际已完成但未被正式记录的工作

1. **Studio G1 Mock 全栈切片**（S02，未跟踪）：后端 4,178 行 + 前端 2,895 行 + 测试 + DB。文档仍停在 G0"分析完成"。
2. **M2.2 Gate 2 rework1**（S03，未提交）：可移植 launch/xacro、校验 helper、watchdog、离线测试。仓库/README 完全未提。
3. **ArmPi 5-DoF MoveIt 仿真工作区**（S06，非 Git）：模型加载 + 三条关节空间轨迹成功。无任何正式文档。
4. **`docs/phase0` 两份审计**（S02，未跟踪）：JER-39、JER-59；未进仓库。
5. **M2.2 四个 bag 与源快照**（S08/S09）：验证了 11-10 的控制器调参；未写入 README/ROADMAP。
6. **Phase II Week 1 分支**（S01/S02 refs）：4 commits + CHANGELOG/evolution 文档；未合并、未进 README。
7. **Panda baseline tag**：已在 Git/GitHub，但 README/ROADMAP 没有"已冻结"说明。
8. **本地 M2.2 运行时变体**（S01 未提交 3 文件）：是 bags 的真实源码；没有 commit/tag。

## 6. 状态矛盾清单

| # | 声称 | 实际 | 影响 |
|---|---|---|---|
| 1 | G0 README："不含实现代码" | S02 已有 G1 Mock 代码 | 会误判"Studio 未开始" |
| 2 | ZCode memory（11-14）："no backend/frontend/tests" | 代码 11-14 14:05–14:23 落盘 | memory 过期 |
| 3 | JER-39："仓库没有任何 ArmPi 资产" | 仓库仍没有；S06 有模型/MoveIt | 会误判"模型未做" |
| 4 | `ROADMAP.md` 4 个 `[ ]` | 4 项均已完成 | 文档状态失真 |
| 5 | `armpi-debug-studio/README` "41 tests" | 37 个测试函数 + 2 parametrize；未重跑 | 数字未复核 |
| 6 | `g1-verification` "Playwright PASS" | 无 Playwright 文件 | 浏览器验收不可复现 |
| 7 | M2.2 "implemented/9 tests pass" | 离线测试；bags 早于 rework1 | 不能称为"运行时验证" |
| 8 | Panda "baseline frozen" | tag 存在；phase2 分支未合并 | 冻结基线可能漏掉完整可靠性工作 |
| 9 | `initial_positions.yaml` | 与厂商启动姿态不符；r_joint 超限 | 规划起点失真 |
| 10 | URDF 限位 ±2.09 | joint3 软件范围 -2.094…0 | MoveIt 规划可能被静默裁剪 |
| 11 | `Projects_M70q` 是主工作区 | 0 文件、`.git` 不可用 | 任何以此为准的会话都会误判 |
| 12 | `README` WSL2 | 当前原生 Ubuntu 24.04 | 环境说明过期 |

## 7. 不确定事项（必须实测/批准）

1. 6 个舵机的真实 offset、限位、方向；`joint_states` 是否真的只是命令回显。
2. 真实 ArmPi 的 5-DoF 可达工作空间与关节耦合；joint3 正向是否物理禁区。
3. 相机型号、内参、`link4→camera` 实际变换；深度图/点云是否可用。
4. 夹爪 `r_joint` 行程、方向、力限；mimic 参数是否与实物一致。
5. 厂商 Driver 的 OOM/DDS 反序列化异常根因；容器内 `ros2_ws` 的真实内容。
6. 底盘/编码器/急停/供电；`CHASSIS_TYPE` 是否与实际一致。
7. Phase II 分支是否应作为 Panda 最终基线；还是按 MASTER_PLAN 直接冻结在 `main`。
8. Paperclip 中 JER-* 任务的真实状态（DB 未查询）。
9. S17 `armpi_ultra_ws_clean` 的最终内容、正确性与可复现性（并行会话仍在写入；未审计）。

## 8. 结论与建议焦点

1. **不要继续在 ArmPi 模型上叠加 MoveIt 功能**，先把 Phase 0 的 4 个硬缺口补齐：joint3/r_joint 限位统一、camera optical frame、provisional 标记、陈旧 URDF 清理。
2. **不要声称 5-DoF 规划已实现**：IK 插件缺失，当前只有关节空间轨迹。下一步应先装/选 IK（TRAC-IK 或 pick_ik），再做约束位姿可达性统计。
3. **不要把 M2.2 当作项目误差预算**：它是 2-DOF Gazebo 调参。Error Budget v1 需要 ArmPi 实机、重复性/回差/标定/视觉数据。
4. **Studio 是真实成果，但只是 G1 Mock**：先补 E7 只读 gateway 与 Playwright 文件，再谈 G1 验收。
5. **Phase II 分支需要一个明确的处置**（合并/归档/放弃），否则"Panda 冻结"叙事不完整。
6. **主工作区必须先统一**，否则上述成果会继续分散在 6 个位置（含进行中的 S17）。
