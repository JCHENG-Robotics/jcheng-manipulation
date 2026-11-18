# JCHENG Manipulation: A ROS 2-Based Robotic Manipulation System with Quantitative Reliability Evaluation

[English](README.md) | **简体中文**

JCHENG Manipulation 是一个围绕 **ArmPi Ultra** 五自由度机械臂的个人机器人项目：修正模型与 TF、约束运动规划、标定、真实视觉抓取，以及量化的误差预算。

厂商技术栈被视为**只读基线**。本仓库包含项目特定的代码、模型、实验与证据，**不**复制 16 GB 的厂商源码树。

> 历史说明：本仓库最初是一个为期 8 周的 ROS 2 / Panda 学习项目。相关记录保留在
> [`docs/archive/README_8WEEK.md`](docs/archive/README_8WEEK.md) 和
> [`docs/weekly-log/`](docs/weekly-log/)。旧路线图位于
> [`docs/archive/ROADMAP_8WEEK.md`](docs/archive/ROADMAP_8WEEK.md)。

## 状态（2026-11-17）

状态约定：**已实现 / 已验证 / 进行中 / 计划中**。厂商功能单独标注。`已验证` 表示 `evidence/` 中存在命令、日志或数据文件；它**不**代表已完成硬件验证。

| 领域 | 状态 | 证据 / 限制 |
|---|---|---|
| Panda 学习演示 | 已实现，仿真验证 | `panda-baseline-v1` tag；`docs/archive/README_8WEEK.md`；每周日志 |
| ArmPi URDF / TF | 已实现；模型 + 实机 TF 验证 | `xacro` + `check_urdf`；实机 `base_link → rgb_camera_link` / `depth_camera_link`；**关节限位仍为暂定值** |
| ArmPi MoveIt 2 | 已实现，Mock 验证；实机 `reset_servo` 验证 | KDL IK；关节空间规划 `error_code=1`；6D 约束的 Pilz `PTP`（Mock）；Humble Docker MoveIt 完成一次受监督实机运动 |
| ArmPi 约束抓取 | 已实现，Mock 验证 | 5-DoF 抓取搜索：80 个关节空间候选 → 40 个 IK 成功 → 26 个无碰撞 → 3 条 Pilz `PTP` 规划；仅规划 |
| Fiducial 目标定位 | 已实现，合成 + 真实背景半合成验证 | AprilTag 36h11；5/5 合成位姿（最大 1.82 mm / 1.49°）；真实背景 15/15 半合成位姿（最大 1.69 mm / 0.53°）；8 帧真实图像 0 误检 |
| 相机 frame 契约 | 已实现，已验证 | `camera_optical_calibrated` = 实测 body→optical 适配（89.798°，5.4/10.1/−27.4 mm）；模型 `camera_optical_frame` 仅用于 Mock |
| Phase 0 验收 | **完成（Mock 范围）** | [`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md) |
| 真实 ArmPi 硬件 | **已采集基线，部分验证** | 2026-11-17 实机 Pi/ROS/舵机/TF/相机基线；关节限位和 IMU 待完成 |
| 相机 / 标定 | **相机可用；手眼标定已验证；已离线选姿** | Aurora 930 RGB/depth 实时；RGB 跨主机实测约 12 Hz（容器内订阅会低估到约 8 Hz）；14 姿态棋盘手眼（读取舵机实测位置）：板位姿一致性 2.94 mm / 0.39°，留出基姿态 3.29 mm / 0.43°，留一验证 3.41 mm / 0.44°（std）；实测光轴 frame 已发布；从 14,176 个评估位姿中离线选出 24 个多样位姿（尚未实机执行） |
| 视觉抓取 | 进行中 — Mock 链路已验证 | 感知、抓取搜索、已验证的相机标定与 frame；fiducial → 抓取桥接 + 安全否决通过合成 Mock E2E；尚无实机抓取 |
| 安全层 | 已实现，Mock 验证 | 状态看门狗、关节/速度/加速度/起始状态/时长/工作空间检查；`ValidateTrajectory` 否决服务；限位仍为暂定值，无急停集成 |
| 测量协议 | 已实现，dry-run/simulate 验证 | 重复性/回差/视觉位移/夹爪包络的计划与分析器；尚无实机数据 |
| 误差预算框架 | 已实现，demo 验证；项目预算**仍不完整** | [`docs/phase3/ERROR_BUDGET.md`](docs/phase3/ERROR_BUDGET.md)；已测量手眼 + fiducial 项，RSS 3.699 mm / 0.682°；重复性/回差/执行/夹爪/夹具位姿仍未测量 |
| 配对试验框架 | 已实现，仿真验证 | [`docs/phase3/PAIRED_TRIALS.md`](docs/phase3/PAIRED_TRIALS.md)；24 对随机计划、种子化 bootstrap/sign/McNemar 分析；尚无实机试验 |
| 主机/DDS 基准 | 已实现，只读实机快照 | [`docs/phase3/HOST_BENCHMARK.md`](docs/phase3/HOST_BENCHMARK.md)；笔记本 + Pi 主机指标、20 Hz 状态、跨主机约 12 Hz 相机、0.24 ms ping |
| ArmPi Debug Studio | G1-Mock + 只读硬件 + G2/G3/G4 离线 | 74 个后端 + 12 个网关测试通过；10 个 Playwright Mock E2E；E7 网关已实机验证并常驻（systemd）；G2 轨迹分析、G3 实验记录、G4 只读可视化工作台（可选 `/tf`）；硬件运动仍禁用 |
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
├── phase3/                # 误差预算、配对试验、主机/DDS 基准
├── archive/               # 旧 README/ROADMAP、过期 URDF、Phase II 分支说明
└── weekly-log/            # 8 周学习记录
evidence/
├── m2_2_v2/               # M2.2 bag（已 gitignore）、CSV、报告、快照
├── armpi_moveit/          # 模型检查、MoveIt 冒烟测试和 Phase 0 日志
├── armpi_grasp/           # 抓取候选搜索证据（Mock，仅规划）
├── fiducial/              # 合成 fiducial 检测 + ROS TF 证据
├── phase1/                # 实机 Pi/ROS/舵机/TF/相机基线（2026-11-17）
├── phase2/                # frame、关节限位、桥接 E2E、安全层、测量
└── phase3/                # 误差预算、配对试验、主机/DDS 基准证据
ros2_ws/src/
├── arm_basics/            # 学习节点（历史遗留）
├── arm_description/       # 2-DOF simple_arm + M2.2 控制器配置
├── arm_interfaces/        # 自定义 action + 轨迹校验服务
├── arm_moveit_demo/       # Panda 抓取放置演示
├── armpi_fiducial_localizer/  # AprilTag/ArUco 目标定位
├── armpi_grasp_planner/   # 5-DoF 候选生成 + 桥接 + Mock E2E launch
├── armpi_safety/          # 状态看门狗 + 轨迹否决
├── armpi_analysis/        # 误差预算、配对试验、主机/DDS 基准
├── armpi_ultra_description/   # 5-DoF ArmPi xacro + 夹爪 + 相机坐标系
└── armpi_ultra_moveit_config/ # 5-DoF MoveIt 配置（Mock + 实机 launch）
tools/
├── humble_moveit/         # 实机使用的 Humble Docker MoveIt/RViz
├── phase1/                # 实机基线辅助工具
├── phase2/                # 标定、fiducial、桥接 E2E 与测量工具
└── phase4/                # 只读主机/DDS 基准采集
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

### Fiducial 目标定位（真实相机 + 合成检查）

```bash
# 合成精度检查：5 个已知位姿，输出 JSON + 叠加图
ros2 run armpi_fiducial_localizer fiducial_synthetic_check \
  --output-dir evidence/fiducial/20261116

# 真实相机：定位器 + 实测光学 frame 发布器
ros2 launch armpi_fiducial_localizer fiducial_localizer.launch.py \
  image_topic:=/depth_cam/rgb/image_raw \
  camera_info_topic:=/depth_cam/rgb/camera_info \
  marker_size_m:=0.06 output_frame:=base_link

# 快速节点检查：合成图像流（无真实相机）
ros2 run armpi_fiducial_localizer synthetic_marker_publisher \
  --tvec 0 0 0.4 --rvec 3.141592653589793 0 0 --marker-size 0.06
```

AprilTag 36h11 检测器默认在 `camera_optical_calibrated`（实测光学 frame）发布
`target_pose`，设置 `output_frame:=base_link` 后经 TF 转换；使用
`solvePnPRefineLM` 消除正对相机的 IPPE 二义性。结果：合成 5/5，最大
1.82 mm / 1.49°；真实背景半合成 15/15，最大 1.69 mm / 0.53°；8 帧真实图像
0 误检。详见
[`docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md`](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md)、
[`docs/hardware/camera_frames.md`](docs/hardware/camera_frames.md)、
[`evidence/fiducial/20261116/`](evidence/fiducial/20261116/) 和
[`evidence/phase2/real_background_20261117/`](evidence/phase2/real_background_20261117/)。

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

### ArmPi Debug Studio（Mock、只读硬件与 G2/G3/G4 离线分析）

```bash
cd armpi-debug-studio/backend
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev]'
pytest
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000

# 另一个终端
cd armpi-debug-studio/frontend
npm install && npm run dev
npm run test:e2e          # 构建并运行 10 个 Playwright Mock 测试
```

Mock 模式界面必须显示 `SIMULATION`。使用 `ARMPI_MODE=hardware` 并在机器人上
运行只读网关时，界面显示 `LIVE` 电池与相机数据；关节位置是命令回显。硬件运动
仍按设计禁用。网关已于 2026-11-17 实机验证，并已作为宿主机 systemd 服务常驻；
详见
[`armpi-debug-studio/docs/e7-gateway-verification.md`](armpi-debug-studio/docs/e7-gateway-verification.md)、
[`armpi-debug-studio/ros2_gateway/deploy/README.md`](armpi-debug-studio/ros2_gateway/deploy/README.md)
和 [`armpi-debug-studio/README.md`](armpi-debug-studio/README.md)。关节限位仍为
provisional；只读实测与关力矩测量工具见
[`docs/phase2/JOINT_LIMITS.md`](docs/phase2/JOINT_LIMITS.md)。

G2 Trajectory Inspector 已于 2026-11-18 实现为离线 JSON 数据集分析：计划/命令/
反馈三流分离、逐关节误差与采样质量、CSV 可复算，以及仅在浏览器内播放的 Replay；
没有硬件轨迹数据，示例明确标记为 simulated。详见
[`armpi-debug-studio/docs/g2-trajectory-inspector.md`](armpi-debug-studio/docs/g2-trajectory-inspector.md)。

G3 Experiment Lab 同日实现为离线实验记录：不可变实验定义、带随机化计划的追加式
配对 trial、失败代码、G2 数据集/rosbag2 路径引用、CSV/JSON 导出，以及与 Phase 3
一致的 bootstrap/符号检验/McNemar 分析。示例 16 对全部标记 simulated，二元端点
样本量不足并在界面中明确警告，不能据此宣称成功率显著改善。详见
[`armpi-debug-studio/docs/g3-experiment-lab.md`](armpi-debug-studio/docs/g3-experiment-lab.md)。

G4 Visual Workbench 实现为只读可视化：解析项目 URDF 做前向运动学，可选使用网关
`/tf` + `/tf_static` 快照，显示 link/joint 轴、已验证手眼的
`camera_optical_calibrated` 光学帧、相机画面与标定/TF/URDF provisional 警示；相机
引用只保存元数据，不保存图像字节。`/tf` 目前只有无 ROS 测试，尚未在真实机器人 TF
图上验证；没有实时 TF 时回退到 URDF FK。渲染层不进入安全/运动链。详见
[`armpi-debug-studio/docs/g4-visual-workbench.md`](armpi-debug-studio/docs/g4-visual-workbench.md)。

### 误差预算、配对试验与主机基准（离线/只读）

```bash
# 误差预算：仓库内的部分项目预算 + 合成 demo
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_baseline.json \
  --markdown-out /tmp/error_budget.md
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget --demo

# 配对试验：随机计划、合成仿真、分析
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  plan --pairs 24 --seed 7 --json-out /tmp/paired_plan.json
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  simulate --pairs 24 --seed 7 --jsonl-out /tmp/paired.jsonl
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  analyze --input /tmp/paired.jsonl --markdown-out /tmp/paired.md

# 只读主机/DDS 基准（笔记本 + 机器人；不运动）
tools/phase4/run_benchmark.sh evidence/phase3/session4_20261117/with_camera
```

误差预算**仍不完整**：重复性、回差、抓取执行、夹爪放置和夹具位姿尚未测量。
配对试验分析只在合成数据上验证。基准为只读，不向机器人发送命令。详见
[`docs/phase3/`](docs/phase3/README.md) 和
[`evidence/phase3/session4_20261117/`](evidence/phase3/session4_20261117/)。

## 模型注意事项（重要）

- **关节限位是暂定值。** `joint3` 上限为 `0.0`（厂商软件映射），而不是
  `+2.09`。`r_joint` 上限为 `1.257`，取 URDF 与厂商软件范围的交集。
  机械限位和所有 `effort="1000"` 数值均未测量；每个机械臂限位都带有
  `PROVISIONAL` 注释。
- **相机 frame 分为模型与实机两套。** `camera_connet_link` 是厂商安装座；
  模型中的 `camera_link` / `camera_optical_frame` 是 Mock 用的名义 frame
  （单位变换/REP-103）。实机上 `rgb_camera_link` 是 body frame，实测光学
  frame 是 `camera_optical_calibrated`，由已验证的手眼和全精度实机 TF 组合
  得到；模型 frame 不会在实机上发布。详见
  [`docs/hardware/camera_frames.md`](docs/hardware/camera_frames.md)。
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
- `evidence/fiducial/` — 合成标记精度 JSON/叠加图与 ROS 定位器/TF 集成日志。
- `evidence/calibration/20261117/` — 棋盘手眼数据集、逐方法验证和离线
  24 位姿选择。
- `evidence/phase2/real_background_20261117/` — 真实背景负对照 + 半合成位姿精度。
- `evidence/phase2/frames_20261117/` — 全精度 body→optical frame 契约。
- `evidence/phase2/session3_20261117/` — 合成标记 → 抓取 → 安全 E2E、安全服务负对照、测量 dry-run/simulate 报告。
- `evidence/phase3/session4_20261117/` — 部分误差预算、配对试验仿真、只读笔记本/机器人主机/DDS 基准。
- `evidence/README.md` 和 `evidence/m2_2_v2/MANIFEST.md` 说明了哪些内容被跟踪。

永远不要把 Mock/Gazebo 结果当作硬件验证。

## 文档

- [技术路线图](ROADMAP.md) — 包含当前状态的公开技术路线图。
- [Phase 0 验收](docs/phase0/PHASE0_ACCEPTANCE.md) — 检查清单、证据和剩余缺口。
- [5-DoF 约束](docs/phase0/ARMPI_5DOF_CONSTRAINTS.md) — 任务空间限制与抓取策略。
- [Phase 2 抓取搜索](docs/phase2/GRASP_CANDIDATE_SEARCH.md) — 候选生成、筛选流程、Mock 结果与限制。
- [Phase 2 Fiducial 定位](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md) — AprilTag 检测、位姿约定、真实背景精度与 frame 契约。
- [Phase 2 Fiducial → 抓取桥接](docs/phase2/FIDUCIAL_GRASP_BRIDGE.md) — `target_pose` → TF → 抓取搜索 → 安全否决，Mock E2E 结果。
- [Phase 2 安全层](docs/phase2/SAFETY_LAYER.md) — 看门狗、限位、工作空间与轨迹否决服务。
- [Phase 3 测量协议](docs/phase2/MEASUREMENT_PROTOCOL.md) — 重复性、回差、视觉位移与夹爪包络。
- [Phase 3 误差预算](docs/phase3/ERROR_BUDGET.md) — 误差源 schema、RSS/最坏情况、部分项目预算与缺失测量。
- [Phase 3 配对试验](docs/phase3/PAIRED_TRIALS.md) — 配对记录器、随机化与精确配对分析。
- [Phase 3 主机/DDS 基准](docs/phase3/HOST_BENCHMARK.md) — 只读笔记本/Pi 指标、topic 速率/抖动/延迟与相机速率修正。
- [相机 frame 契约](docs/hardware/camera_frames.md) — body/optical frame 与 `camera_optical_calibrated`。
- [Vendor 标定基线](docs/hardware/vendor_calibration_baseline.md) — vendor 矩阵解码与 Baseline A 状态。
- [Phase 2 关节限位](docs/phase2/JOINT_LIMITS.md) — 舵机只读证据、关力矩测量流程与 CycloneDDS 注意事项。
- [2026-11-15 审计](docs/audit/2026-11-15/README.md) — 来源、冲突、状态、合并计划、待决事项。
- [Debug Studio PRD](docs/armpi/ARMPI_DEBUG_STUDIO_PRD_V1.md)
- [G0 厂商审计](docs/armpi/g0/README.md)
- [硬件基线](docs/hardware/README.md)
- [Phase 0 审计](docs/phase0/)
- [8 周学习日志](docs/weekly-log/)

`MASTER_PLAN_V3_PRIVATE.md` 有意**不**放入这个公开仓库。

## 许可证

见 [LICENSE](LICENSE)。
