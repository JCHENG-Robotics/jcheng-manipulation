# JCHENG Manipulation: A ROS 2-Based Robotic Manipulation System with Quantitative Reliability Evaluation

[English](README.md) | **简体中文**

JCHENG Manipulation 是一个围绕 **ArmPi Ultra** 五自由度机械臂的个人机器人项目：修正模型与 TF、约束运动规划、标定、真实视觉抓取，以及量化的误差预算。

厂商技术栈被视为**只读基线**。本仓库包含项目特定的代码、模型、实验与证据，**不**复制 16 GB 的厂商源码树。

> 历史说明：本仓库最初是一个为期 8 周的 ROS 2 / Panda 学习项目。相关记录保留在
> [`docs/archive/README_8WEEK.md`](docs/archive/README_8WEEK.md) 和
> [`docs/weekly-log/`](docs/weekly-log/)。旧路线图位于
> [`docs/archive/ROADMAP_8WEEK.md`](docs/archive/ROADMAP_8WEEK.md)。

## 状态（2026-11-16）

状态约定：**已实现 / 已验证 / 进行中 / 计划中**。厂商功能单独标注。`已验证` 表示 `evidence/` 中存在命令、日志或数据文件；它**不**代表已完成硬件验证。

| 领域 | 状态 | 证据 / 限制 |
|---|---|---|
| Panda 学习演示 | 已实现，仿真验证 | `panda-baseline-v1` tag；`docs/archive/README_8WEEK.md`；每周日志 |
| ArmPi URDF / TF | 已实现；模型 + 实机 TF 验证 | `xacro` + `check_urdf`；实机 `base_link → rgb_camera_link` / `depth_camera_link`；**关节限位仍为暂定值** |
| ArmPi MoveIt 2 | 已实现，Mock 验证；实机 `reset_servo` 验证 | KDL IK；关节空间规划 `error_code=1`；6D 约束的 Pilz `PTP`（Mock）；Humble Docker MoveIt 完成一次受监督实机运动 |
| ArmPi 约束抓取 | 已实现，Mock 验证 | 5-DoF 抓取搜索：80 个关节空间候选 → 40 个 IK 成功 → 26 个无碰撞 → 3 条 Pilz `PTP` 规划；仅规划 |
| Fiducial 目标定位 | 已实现，合成验证 | AprilTag 36h11；5/5 合成位姿，最大 1.82 mm / 1.49°；ROS TF 检查 0.44 mm / 0.03° |
| Phase 0 验收 | **完成（Mock 范围）** | [`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md) |
| 真实 ArmPi 硬件 | **已采集基线，部分验证** | 2026-11-17 实机 Pi/ROS/舵机/TF/相机基线；关节限位和 IMU 待完成 |
| 相机 / 标定 | **相机可用；手眼标定已验证** | Aurora 930 RGB/depth 实时；14 姿态棋盘手眼（读取舵机实测位置）：板位姿一致性 2.94 mm / 0.39°，留出基姿态 3.29 mm / 0.43°；光轴与静态 TF 差约 89.3° |
| 视觉抓取 | 进行中 | 感知、抓取搜索和已验证的相机标定；尚无实机抓取 |
| 误差预算 v1 | 计划中 | M2.2 是 2-DOF Gazebo 控制器研究，不是项目误差预算 |
| ArmPi Debug Studio | G1-Mock + 只读硬件 | 46 个后端测试通过；前端可构建；E7 网关已实机验证（只读）；硬件运动仍禁用 |
| M2.2 控制器配置 | 已实现，仿真验证 | default / 0.1 / 0.2 轨迹在 Gazebo 中均成功（2026-11-15） |

详细审计（来源、冲突、决策）见
[`docs/audit/2026-11-15/`](docs/audit/2026-11-15/)。

## 仓库结构

```text
armpi-debug-studio/        # G1-Mock 控制台（FastAPI + React）；不做硬件运动
docs/
├── armpi/                 # PRD + G0 厂商审计
├── audit/2026-11-15/      # 源码清单、变更核对、状态、合并计划
├── hardware/              # 硬件基线（2026-11-17 实机；关节限位/IMU 待完成）
├── phase0/                # Phase 0 验收、5-DoF 约束、历史审计
├── phase2/                # 抓取候选搜索；标定/视觉抓取准备
├── archive/               # 旧 README/ROADMAP、过期 URDF、Phase II 分支说明
└── weekly-log/            # 8 周学习记录
evidence/
├── m2_2_v2/               # M2.2 bag（已 gitignore）、CSV、报告、快照
├── armpi_moveit/          # 模型检查、MoveIt 冒烟测试和 Phase 0 日志
├── armpi_grasp/           # 抓取候选搜索证据（Mock，仅规划）
├── fiducial/              # 合成 fiducial 检测 + ROS TF 证据
└── phase1/                # 实机 Pi/ROS/舵机/TF/相机基线（2026-11-17）
ros2_ws/src/
├── arm_basics/            # 学习节点（历史遗留）
├── arm_description/       # 2-DOF simple_arm + M2.2 控制器配置
├── arm_interfaces/        # 自定义 action
├── arm_moveit_demo/       # Panda 抓取放置演示
├── armpi_fiducial_localizer/  # AprilTag/ArUco 目标定位
├── armpi_grasp_planner/   # 5-DoF 候选生成 + MoveIt 筛选
├── armpi_ultra_description/   # 5-DoF ArmPi xacro + 夹爪 + 相机坐标系
└── armpi_ultra_moveit_config/ # 5-DoF MoveIt 配置（Mock + 实机 launch）
tools/
├── humble_moveit/         # 实机使用的 Humble Docker MoveIt/RViz
└── phase1/                # 实机基线辅助工具
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
```

Panda 演示还需要归档的 8 周 README 中描述的 MoveIt/Panda underlay（单独的 `~/ws_moveit` 工作区）。

## 运行

### ArmPi 模型与 MoveIt（Mock 硬件）

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# 模型检查
xacro ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro > /tmp/armpi.urdf
check_urdf /tmp/armpi.urdf

# 仅模型 RViz（不启动 MoveIt）；无头检查用 use_rviz:=false
ros2 launch armpi_ultra_description display.launch.py

# MoveIt 演示：mock_components，无真实硬件
# 无头运行加 use_rviz:=false
ros2 launch armpi_ultra_moveit_config demo.launch.py \
  ros2_control_hardware_type:=mock_components \
  use_sim_time:=false use_gazebo:=false use_real:=false

# 在 gripper_link 上做 6D 约束目标（仅规划，不执行）
python3 tools/phase0/constrained_plan_check.py \
  --pipeline pilz_industrial_motion_planner --planner PTP
```

预期：`move_group` 加载成功，KDL IK 初始化，`arm_controller` 和
`gripper_controller` 处于 active 状态。关节空间冒烟测试见
`evidence/armpi_moveit/MOVEIT_SMOKE_20261115.md`；约束目标检查返回
`error_code=1 (SUCCESS)`，且轨迹点数量非零。详见
[`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md)。

> **不要把这当作硬件验证。** `mock_components` 是仿真系统；`joint_states`
> 语义、限位、相机和夹爪均未测量。

### 5-DoF 抓取候选搜索（Mock，仅规划）

保持上面的无头 MoveIt Mock 运行，另开终端：

```bash
# 正例：FK 精确的笛卡尔候选
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-joints 0.3 0.9 -1.8 -1.7 0.4 --max-plan 3 --json-out /tmp/grasp_cartesian.json

# 实用搜索：目标点周围的关节空间采样
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.1382 0.0225 0.0896 --sampler joint \
  --max-plan 3 --json-out /tmp/grasp_joint.json

# 负例：不可达点（预期退出码 1）
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.60 0.0 0.30 --sampler joint --json-out /tmp/grasp_unreachable.json
```

流程为：策略检查 → 全位姿 IK → FK 校验 → 碰撞检查 → 仅规划的 Pilz `PTP`。
不会执行任何轨迹。2026-11-16 的关节采样结果为 80 个候选 → 40 个 IK 成功 →
26 个无碰撞 → 3 条规划成功。详见
[`docs/phase2/GRASP_CANDIDATE_SEARCH.md`](docs/phase2/GRASP_CANDIDATE_SEARCH.md)
和 [`evidence/armpi_grasp/20261116/`](evidence/armpi_grasp/20261116/)。

### Fiducial 目标定位（合成相机）

```bash
# 合成精度检查：5 个已知位姿，输出 JSON + 叠加图
ros2 run armpi_fiducial_localizer fiducial_synthetic_check \
  --output-dir evidence/fiducial/20261116

# 节点级检查：合成图像流（无真实相机）
ros2 run armpi_fiducial_localizer fiducial_localizer --ros-args \
  -p image_topic:=/image_raw -p camera_info_topic:=/camera_info \
  -p marker_size_m:=0.06 -p output_frame:=base_link

ros2 run armpi_fiducial_localizer synthetic_marker_publisher \
  --tvec 0 0 0.4 --rvec 3.141592653589793 0 0 --marker-size 0.06
```

AprilTag 36h11 检测器默认在相机光学坐标系发布 `target_pose`，设置
`output_frame:=base_link` 后经 TF 转换。合成结果：5/5 检测，最大
1.82 mm / 1.49°；ROS TF 检查：0.44 mm / 0.03°。详见
[`docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md`](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md)
和 [`evidence/fiducial/20261116/`](evidence/fiducial/20261116/)。

### M2.2 控制器配置（Gazebo，2-DOF `simple_arm`）

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch arm_description gazebo.launch.py controller_profile:=gain_0p2
```

配置：`default`、`gain_0p1`、`gain_0p2`。2026-11-15 的回归测试向
`/arm_controller/follow_joint_trajectory` 发送 `joint1=0.2, joint2=0.2`；
三者均返回 `error_code=0`。详见
[`evidence/m2_2_v2/rework1_runtime/REWORK1_RUNTIME_20261115.md`](evidence/m2_2_v2/rework1_runtime/REWORK1_RUNTIME_20261115.md)。

### ArmPi Debug Studio（Mock 与只读硬件）

```bash
cd armpi-debug-studio/backend
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev]'
pytest
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000

# 另一个终端
cd armpi-debug-studio/frontend
npm install && npm run dev
```

Mock 模式界面必须显示 `SIMULATION`。使用 `ARMPI_MODE=hardware` 并在机器人上
运行只读网关时，界面显示 `LIVE` 电池与相机数据；关节位置是命令回显。硬件运动
仍按设计禁用。网关已于 2026-11-17 实机验证；详见
[`armpi-debug-studio/docs/e7-gateway-verification.md`](armpi-debug-studio/docs/e7-gateway-verification.md)
和 [`armpi-debug-studio/README.md`](armpi-debug-studio/README.md)。

## 模型注意事项（重要）

- **关节限位是暂定值。** `joint3` 上限为 `0.0`（厂商软件映射），而不是
  `+2.09`。`r_joint` 上限为 `1.257`，取 URDF 与厂商软件范围的交集。
  机械限位和所有 `effort="1000"` 数值均未测量；每个机械臂限位都带有
  `PROVISIONAL` 注释。
- **相机坐标系是标称值。** `camera_connet_link` 是厂商安装座；
  `camera_link` 和 `camera_optical_frame` 以单位变换/REP-103 旋转添加。
  真实的 `link4 → camera` 外参未知。
- **IK 默认是 KDL 全位姿。** 未安装 TRAC-IK。
  `position_only_ik: false` 是仓库中的默认值，因此 6D 目标不会被静默降级为
  位置目标。仅在“机械臂能否到达该点？”检查中使用
  `config/kinematics_reachability.yaml`。详见
  [`docs/phase0/ARMPI_5DOF_CONSTRAINTS.md`](docs/phase0/ARMPI_5DOF_CONSTRAINTS.md)。
- **约束目标使用 Pilz `PTP`。** 即使 `/compute_ik` 能求解，OMPL 的目标采样器
  在 KDL 下仍无法处理该 6D 目标；`LIN` 失败，因为固定姿态的直线会离开
  5-DoF 可达集。细节见 Phase 0 验收文档。
- **抓取搜索基于可达集。** 在固定工具点处，可达姿态构成一个很薄的 5-DoF
  流形，因此大多数笛卡尔姿态邻域都会 IK 失败。对目标点请使用关节空间采样
  （`--sampler joint`）；结果中的 `target_offset_m` 给出候选与目标点的残余偏差。
  TCP 偏置为标称值，所有结果均为仅规划。
- **Fiducial 定位仅完成合成验证。** AprilTag 检测器使用打印边长
  （`marker_size_m`，默认 0.06 m）和相机 `CameraInfo`。真实内参、
  `link4 → camera` 外参、光照和镜头畸变均未测量；不要把合成精度当作真实
  相机计量结果。
- **已移除过期 URDF。** 厂商预生成的 `armpi_ultra.urdf`（joint1 和 r_joint
  为 `fixed`）已归档到 `docs/archive/armpi_ultra.urdf.stale`；只使用 xacro。
- **无硬件。** 没有 `/dev/rrc`、没有相机、没有底盘、没有真实运动。

## 证据与可复现性

- `evidence/m2_2_v2/` — 四个 MCAP bag、分析 CSV/PNG、源码快照、Gate 2 报告。
  `*.mcap` 已 gitignore；请保留本地/远程备份。
- `evidence/armpi_moveit/` — 展开后的 URDF、`check_urdf`、MoveIt 日志和
  Phase 0 约束规划证据。
- `evidence/armpi_grasp/` — 抓取候选搜索 JSON 与无头 MoveIt 日志
  （Mock，仅规划）。
- `evidence/fiducial/` — 合成标记精度 JSON/叠加图与 ROS 定位器/TF 集成日志
  （无真实相机）。
- `evidence/README.md` 和 `evidence/m2_2_v2/MANIFEST.md` 说明了哪些内容被跟踪。

永远不要把 Mock/Gazebo 结果当作硬件验证。

## 文档

- [技术路线图](ROADMAP.md) — 包含当前状态的公开技术路线图。
- [Phase 0 验收](docs/phase0/PHASE0_ACCEPTANCE.md) — 检查清单、证据和剩余缺口。
- [5-DoF 约束](docs/phase0/ARMPI_5DOF_CONSTRAINTS.md) — 任务空间限制与抓取策略。
- [Phase 2 抓取搜索](docs/phase2/GRASP_CANDIDATE_SEARCH.md) — 候选生成、筛选流程、Mock 结果与限制。
- [Phase 2 Fiducial 定位](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md) — AprilTag 检测、位姿约定、合成精度与 ROS 集成。
- [2026-11-15 审计](docs/audit/2026-11-15/README.md) — 来源、冲突、状态、合并计划、待决事项。
- [Debug Studio PRD](docs/armpi/ARMPI_DEBUG_STUDIO_PRD_V1.md)
- [G0 厂商审计](docs/armpi/g0/README.md)
- [硬件基线](docs/hardware/README.md)
- [Phase 0 审计](docs/phase0/)
- [8 周学习日志](docs/weekly-log/)

`MASTER_PLAN_V3_PRIVATE.md` 有意**不**放入这个公开仓库。

## 许可证

见 [LICENSE](LICENSE)。
