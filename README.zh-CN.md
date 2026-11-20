# JCHENG Manipulation

**面向五自由度 ArmPi Ultra 的实测视觉操作：修正模型、自主标定、实机抓取执行与量化可靠性预算。**

[English](README.md) | **简体中文**

> 本项目为应用机器人方向的项目制硕士毕业课题。本仓库是该工作的完整、可追溯记录：修正后的五自由度模型与 TF 树、约束 MoveIt 2 规划、经独立标记验证的自主手眼标定、实机抓取—抬升—放置、安全否决，以及基于实测的失败模式分类。厂商固件与驱动仅作为**只读基线**，不复制进本仓库。

> 历史说明：本仓库最初是一个为期 8 周的 ROS 2 / Panda 学习项目。该记录归档于
> [`docs/archive/README_8WEEK.md`](docs/archive/README_8WEEK.md) 和
> [`docs/weekly-log/`](docs/weekly-log/)；它不属于本毕业课题的结论。早期 Panda
> 与 M2.2 产物仅为可追溯性保留。

**目录：** [项目概览](#项目概览) · [摘要](#摘要) · [目标与贡献](#目标与贡献) ·
[关键结果](#关键结果实机实测) · [系统架构](#系统架构) · [仓库结构](#仓库结构) ·
[项目状态](#项目状态2026-11-20) · [交付清单](#毕业项目交付清单) ·
[快速开始](#快速开始) · [实验方法](#实验方法) ·
[失败分类](#失败模式分类实测) · [安全包络](#安全与运行包络) ·
[局限性](#局限性与有效性威胁) · [证据索引](#可复现性与证据索引) ·
[文档](#文档) · [结论](#结论) · [路线图](#路线图) · [许可证](#许可证)

<p align="center">
  <img src="evidence/phase2/onsite_20261118/first_grasp/operator_grasp_20261118.jpg" width="420" alt="ArmPi Ultra 夹住地面上的 39.4 mm 木块">
  <br>
  <em>首次实机抓取（2026-11-18）：机械臂夹住地面上的 39.4 mm 木块。
  全自主抓取—抬升—放置于 2026-11-19 完成。</em>
</p>

## 项目概览

| 项目 | 摘要 |
|---|---|
| **平台** | HiWonder ArmPi Ultra：五自由度机械臂、两指夹爪、麦克纳姆底盘、Raspberry Pi 5、Aurora 930 RGB-D 相机 |
| **软件** | 机器人端 ROS 2 Humble；工作站端 ROS 2 Jazzy；MoveIt 2 + Pilz `PTP` + KDL；FastAPI/React 调试台 |
| **核心问题** | 当每一个变换、限位和失效模式都被实测而非假设时，低成本五自由度机械臂加腕部相机能在桌面上实现多精确的操作？ |
| **代表性结果** | 对无标记 39.4 mm 木块完成自主抓取—抬升—放置；固定 Tag 普查把低构型误差测成**常数**，随后**预注册的 24 对实机试验**把抓取成功率从 **12/24 提升到 24/24**（McNemar p = 0.00049），双成功对子的放置误差配对改善 **2.19 mm**（p = 0.0063） |
| **证据** | 20+ 个证据包：会话日志、标定数据集、舵机总线记录、Studio 测试报告与机器可读 JSON |
| **状态** | 2026-11-20（实机结果截至 2026-11-19）。实机运动均为受监督执行；正式普查后关节映射在作业包络内为 `verified: true`；机器人没有急停或抱闸 |

## 摘要

本项目研究：当每一个变换、限位和失效模式都被实测而非假设时，低成本五自由度机械臂加腕部 RGB-D 相机能否完成可重复的桌面视觉抓取。ArmPi Ultra 被建模为受约束的五自由度机械臂；相机通过自主实现的棋盘格手眼流程标定到 `link4`（14 个位姿，板位姿一致性 2.94 mm / 0.39°，留出基位姿 3.29 mm / 0.43°）。无标记 RGB-D 分割、关节空间抓取搜索、MoveIt/Pilz `PTP` 执行与独立轨迹否决构成抓取链路。对无标记 39.4 mm 木块进行了 5 轮实机抓取—抬升—放置试验，3 轮成功。第 3、4 轮失败并暴露出低抓取构型下系统性的 −8.0/−15.3/−5.8 mm FK/手眼误差；在该构型下用一次 AprilTag 观测得到修正量后，第 5 轮成功夹住木块、抬升约 148 mm 并释放。随后固定 Tag 普查证明该误差是**常向量**，关节偏置与 `link4` 偏置模型被拟合后**否决**；在 27.9° 偏航下常数模型仍是最近的模型（相差 5.6 mm，夹持关键轴 y 差 3.3 mm），拟合模型相差 14–19 mm。**预注册的 24 对实机试验**把成功率从 12/24 提升到 24/24（McNemar p = 0.00049），配对放置改善 2.19 mm。五个关节已正式手推普查（止挡为姿态相关的自碰撞限位），重复性与回差实测为 0.449 mm RMS 与 1.117 mm，46 次偏移网格测得夹持容差 ±9 mm；夹爪重测将行程从 0.84 rad 修正为 2.22 rad。本项目的贡献是实测失败模式分类、一项预注册的实机可靠性实验与明确的证据链；误差预算仍有三项未测，2° 旋转门限仍是暂定值。

## 目标与贡献

**项目目标**

1. 重建可信的五自由度模型：关节类型、限位、TF 与相机坐标系均对实机核验。
2. 建立带独立验证的自主标定流程，而不是依赖厂商默认值。
3. 在约束规划与独立安全否决下执行实机视觉抓取。
4. 量化各项误差来源，并识别主导项。
5. 让系统可观测、可复现：每条结论背后都有命令、日志或数据文件。

**主要贡献**

1. **实测模型与坐标系契约。** 修正后的 xacro 模型、实机 TF 核验，以及供 fiducial 定位器使用的实测 body→optical 变换 `camera_optical_calibrated`。
2. **带独立检查的自主标定。** 棋盘格手眼、桌面下俯重标定，以及暴露 4.87% 桌面纵向偏差的实体 AprilTag 尺度测试。
3. **完整的实机抓取链路。** RGB-D 分割 → 约束抓取搜索 → Pilz `PTP` 规划 → 安全否决 → 实测舵机执行 → 验证，并在实机上端到端运行。
4. **实测低构型误差与被检验模型。** 固定 Tag 普查证明低抓取构型下的 FK/手眼误差是**常向量 (+11.6, +4.0, +8.5) mm**（散布 0.40–0.99 mm），抓取验证修正量为 **(+13.7, +3.9, +8.9) mm**；同一模型在其验证体积内为 2–3 mm。关节偏置与 `link4` 偏置模型虽能拟合 Tag 数据（1.4–2.3 mm RMS），但相对四次成功抓取所用的修正量偏差 **10.6–15.0 mm**，被**否决**。
5. **预注册的配对实机试验。** 24 对匹配试验比较原始 FK/手眼目标与常数修正：**12/24 vs 24/24**（McNemar p = 0.00049），双成功对子的放置误差配对改善 **2.19 mm**（95% CI −3.96…−0.83，p = 0.0063），4 项协议偏差已记录并量化。
6. **实测误差预算。** 30 次重复性 **0.449 mm RMS / 1.017 mm max**，回差 **1.117 mm**，46 次抓取容差网格给出 **±9 mm**（保守值 6 mm）；预算更新为平移 **RSS 3.995 mm**、旋转 **1.103°**；手眼项占平移 RSS 的 68%，三项集成项仍未测。
7. **安全的关节限位普查。** 以 EEPROM 为界的采纳限位、通电探测到机械止挡，以及规避厂商节点负寄存器崩溃的工具链。
8. **可靠性工具。** 误差预算框架、配对试验记录器与运行器、测量运行器、只读主机/DDS 基准，以及 G1–G6 调试台（后端 92 + 网关 38 个测试）。
9. **证据优先的文档。** 每条量化结论都链接到带时间戳的证据包；Mock、仿真与实机结果严格区分。

## 关键结果（实机实测）

| 结果 | 实测值 | 证据 |
|---|---|---|
| 手眼标定，14 位姿 | 板位姿一致性 **2.94 mm / 0.39°**；留出基位姿 **3.29 mm / 0.43°**；留一标准差 **3.41 mm / 0.44°** | [`evidence/calibration/20261117/`](evidence/calibration/20261117/) |
| 桌面下俯重标定，15 位姿 | 一致性 **2.7 mm / 0.42°**；留出 **2.2 mm / 0.29°**；留一 **3.1 mm / 0.45°** | [`evidence/calibration/20261118/table_down/`](evidence/calibration/20261118/table_down/) |
| 标定位姿选择 | 从 14,176 个评估位姿 / 3,093 个可行位姿中选出 **24 个**；视角跨度 33.5°；已规划、未执行 | [`evidence/calibration/20261117/pose_selection/`](evidence/calibration/20261117/pose_selection/) |
| 独立 Tag 尺度 | 横向 **100.67 mm vs 100.0 mm（0.67%）**；桌面纵向 **104.87 mm vs 100.0 mm（4.87%）**；夹角 **92.3°** | [`evidence/phase2/onsite_20261118/tag_displacement_validation/`](evidence/phase2/onsite_20261118/tag_displacement_validation/) |
| 深度桌面平面 | **147,626 个内点，RMS 0.7 mm**；桌面 z **−0.0787 m**（深度）vs **−0.0784 m**（215 mm 直尺） | [`evidence/phase2/onsite_20261118/depth_plane_20261118.md`](evidence/phase2/onsite_20261118/depth_plane_20261118.md) |
| 实机抓取试验（第一组） | 39.4 mm 木块的抓取—抬升—放置 **3 / 5** 成功（可行性、非配对） | [`grasp_150mm/`](evidence/phase2/onsite_20261119/grasp_150mm/)、[`attempt2/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt2/)、[`attempt5/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt5/) |
| 最佳自主抬升 | TCP **+0.0993 m**，物体离地 **约 148 mm**；木块 **39.2 × 39.8 mm** | [`attempt5/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt5/) |
| 低构型模型误差 | 固定 Tag：**常数 (+11.6, +4.0, +8.5) mm**，位形散布 **0.40–0.99 mm**；抓取验证修正量 **(+13.7, +3.9, +8.9) mm**，放置残差 **1.3 mm** | [`low_config_error/`](evidence/calibration/20261119/low_config_error/) |
| 低构型模型选择 | 关节偏置 / `link4` 3-DoF / `link4` 6-DoF 都能拟合 Tag 数据（1.4–2.3 mm RMS），但相对已验证修正量偏差 **10.6–15.0 mm**；**支持常数偏置模型** | [`model_selection.md`](evidence/calibration/20261119/low_config_error/model_selection.md) |
| 包络的偏航延伸 | 在 `j1 = 27.9°` 处所需修正为 **(9.14, 7.18, 8.75) mm**；常数偏置相差 **5.6 mm**（夹持关键 y 分量 **3.3 mm**），构型相关模型相差 **14–19 mm** 且 y 反号 | [`yaw_envelope/`](evidence/phase3/onsite_20261119/yaw_envelope/) |
| 配对实机试验，24 对 | **baseline 12/24 vs intervention 24/24**（McNemar **p = 0.00049**）；放置误差 **3.31 mm** vs **4.97 mm**；配对改善 **2.19 mm**（95% CI −3.96…−0.83，**p = 0.0063**） | [`evidence/phase3/onsite_20261119/`](evidence/phase3/onsite_20261119/README.md) |
| 机械臂重复性，30 次 | 径向 **RMS 0.449 mm / 最大 1.017 mm**；关节重复到 0–1 脉冲；Tag 旋转最大 **0.867°** | [`repeatability/`](evidence/phase3/onsite_20261119/repeatability/) |
| 机械臂回差，joint4 ±0.1 rad | 回差 **1.117 mm**（−0.971, −0.064, −0.549 mm）；joint4 落点相差 10 脉冲 | [`backlash/`](evidence/phase3/onsite_20261119/backlash/) |
| 夹爪容差包络，46 次抓取 | 开口方向通过区间 **[-6, +12] mm**（以 +3 mm 为中心 **±9 mm**，即修正量的 y 残余）；`x` 与 `z` 到 **±9 mm** 均无失败；±15 与 −12 均 0/2 | [`envelope/`](evidence/phase3/onsite_20261119/envelope/) |
| 抓取工作半径 | 指垫半径 **205–210 mm**；`joint1` 偏航范围 **±103.2°（206°）** | [`evidence/phase2/onsite_20261119/reach/`](evidence/phase2/onsite_20261119/reach/) |
| 视野外搜索 + 底盘往返 | **+16° `joint1` 偏航**找到 **0.469 m** 处木块；闭环底盘移动 **0.325 m**；抓取并抬升 150 mm；回位残差 **9.4 mm**（地标校验）；放置相对机器人 **1.1 mm** | [`evidence/phase2/onsite_20261119/search_and_return/`](evidence/phase2/onsite_20261119/search_and_return/) |
| 最低可达位置 | 物理最低点离地 **10 mm**；名义 TCP 离地 **15 mm**；机械极限为 joint2 322 / joint3 939 | [`evidence/phase2/onsite_20261119/lowest_reach/`](evidence/phase2/onsite_20261119/lowest_reach/) |
| 采纳关节限位 | 舵机 EEPROM **减 20 脉冲**，普查后 joint2 上限收紧到 **775**；五个关节 2026-11-19 全部手推实测；`mapping_verified=true`（作业包络） | [`docs/phase2/JOINT_LIMITS.md`](docs/phase2/JOINT_LIMITS.md) |
| 夹爪行程 | **2.22 rad**（张开 −2.22，闭合 0.0）；回差 **约 34 counts**；39.4 mm 木块在 −0.63 夹住 | [`evidence/phase2/onsite_20261118/gripper_stops_20261118/`](evidence/phase2/onsite_20261118/gripper_stops_20261118/) |
| 相机 / DDS | 跨主机 RGB **约 12 Hz**（容器内订阅低估到约 8 Hz）；深度 **8.2 Hz**；ping **0.24 ms** | [`docs/phase3/HOST_BENCHMARK.md`](docs/phase3/HOST_BENCHMARK.md) |
| Studio G5 跟踪 | **712 命令 / 711 实测**样本；最大 **0.008321 rad**，RMSE **0.003962 rad** | [`evidence/studio_g5/20261118/README.md`](evidence/studio_g5/20261118/README.md) |
| 误差预算（2026-11-19） | 平移 **RSS 3.995 mm**（通过）/ 最坏 **7.114 mm**（未通过）vs 实测容差 **6 mm**；旋转 **RSS 1.103° / 最坏 1.827°** vs 暂定 2°（均通过）；手眼项占平移 RSS 68% | [`error_budget_envelope.md`](evidence/phase3/onsite_20261119/error_budget_envelope.md) |

## 系统架构

```text
┌────────────────────────────── 工作站（笔记本）──────────────────────────────────┐
│  MoveIt 2（Humble 容器）     armpi_safety 否决      armpi_analysis                │
│  Pilz PTP / KDL IK           轨迹检查               误差预算、配对试验             │
│  Debug Studio（FastAPI+React）G1–G6、只读网关、G5 记录器                          │
└───────────────────────────────────────┬───────────────────────────────────────────┘
                                        │ CycloneDDS，直连以太网
                                        │ 10.42.0.1 ↔ 10.42.0.218
┌───────────────────────────────────────┴───────────────────────────────────────────┐
│ Raspberry Pi 5 — 厂商 Humble 核心栈                                                │
│ ros_robot_controller · servo_manager · camera · TF · start_node.service            │
└───────────────┬───────────────────────────────┬───────────────────────────┬────────┘
                │ 6 路总线舵机                  │ 4 路电机通道              │ USB
        机械臂关节 2–6 + 夹爪 1           麦克纳姆底盘（开环）        Aurora 930 RGB-D
```

抓取链路为：

```text
RGB-D 帧
  → 地面/桌面平面拟合
  → 物体掩码 + 体素聚类 + 顶面切片位姿
  → TF 变换到 base_link
  → 约束抓取搜索（位置 + 俯仰 + 腕部滚转 + 接近偏置）
  → 全位姿 KDL IK → FK 校验 → 碰撞检查
  → Pilz PTP 规划（仅规划）
  → /armpi_safety/validate_trajectory
  → 实测舵机执行
  → 视角位姿验证（物体已不在原地）
```

## 仓库结构

```text
armpi-debug-studio/        # G1–G6 控制台（FastAPI + React）+ 只读/受监督网关
docs/
├── armpi/                 # PRD + G0 厂商审计
├── audit/2026-11-15/      # 源码清单、核对与决策
├── hardware/              # 硬件基线、坐标系契约、网络访问、限位
├── phase0/                # Phase 0 验收、5-DoF 约束、历史审计
├── phase2/                # 标定、抓取搜索、fiducial 桥接、安全、限位
├── phase3/                # 误差预算、配对试验、主机/DDS 基准
├── archive/               # 旧 README/ROADMAP、过期 URDF、8 周记录
└── weekly-log/            # 8 周学习记录
evidence/
├── phase1/                # 实机 Pi/ROS/舵机/TF/相机基线（2026-11-17）
├── calibration/           # 棋盘手眼（20261117）+ 桌面下俯（20261118）
├── fiducial/              # 合成 + 真实背景 AprilTag 验证
├── armpi_moveit/          # 模型检查、MoveIt 冒烟、Phase 0 日志
├── armpi_grasp/           # Mock 抓取候选搜索（仅规划）
├── phase2/onsite_20261118 # 现场安全、限位、夹爪、首次抓取
├── phase2/onsite_20261119 # 第 1–5 轮抓取、最低可达、偏航包络、控制板探测
├── phase3/                # 误差预算、配对试验仿真、基准
└── studio_g*/             # G2–G6 测试与实机证据
ros2_ws/src/
├── armpi_ultra_description/    # 5-DoF xacro + 夹爪 + 相机坐标系
├── armpi_ultra_moveit_config/  # MoveIt 配置（Mock + 实机 launch）
├── armpi_fiducial_localizer/   # AprilTag/ArUco 目标定位
├── armpi_grasp_planner/        # 5-DoF 候选生成 + 桥接 + Mock E2E
├── armpi_safety/               # 状态看门狗 + 轨迹否决
├── armpi_analysis/             # 误差预算、配对试验、主机基准
├── arm_basics/                 # 历史学习节点
├── arm_description/            # 2-DOF simple_arm + M2.2 控制器配置
├── arm_interfaces/             # 自定义 action + 轨迹校验服务
└── arm_moveit_demo/            # Panda 抓取放置演示
tools/
├── humble_moveit/         # 实机使用的 Humble Docker MoveIt/RViz
├── phase1/                # 实机基线辅助工具
├── phase2/                # 标定、分割、抓取、限位、安全工具
└── phase4/                # 只读主机/DDS 基准采集
```

## 项目状态（2026-11-20）

实机结果截至 2026-11-19；本文档于 2026-11-20 与路线图、变更日志和验收报告同步。状态约定：**已实现 / 已验证 / 进行中 / 计划中**。`已验证` 表示 `evidence/` 中存在命令、日志或数据文件；它本身不代表完成硬件验证。厂商功能单独标注。

| 工作流 | 状态 | 证据 / 限制 |
|---|---|---|
| 机器人模型与 TF | **已实现；实机 TF 已验证** | 修正 xacro；`check_urdf`；实机 `base_link → rgb_camera_link` / `depth_camera_link`；实测光学坐标系 |
| ArmPi MoveIt 2 | **已实现；Mock + 实机执行验证** | KDL 全位姿 IK；关节空间规划 `error_code=1`；6D 约束 Pilz `PTP`；实机使用 Humble Docker MoveIt |
| 手眼标定 | **已验证** | 14 位姿棋盘：2.94 mm / 0.39°；留出 3.29 mm / 0.43°；桌面下俯 15 位姿重拟合 2.7 mm / 0.42° |
| Fiducial 定位 | **已验证（合成 + 真实背景 + 实体 Tag）** | 合成 5/5（最大 1.82 mm / 1.49°）；半合成 15/15（最大 1.69 mm / 0.53°）；8 帧真实图像 0 误检；实体 Tag 横向尺度 0.67% |
| 约束抓取搜索 | **已实现；Mock + 实机使用** | Mock：80 候选 → 40 IK → 26 无碰撞 → 3 条规划；每轮实机抓取均使用该搜索 |
| 实机视觉抓取 | **已验证：第一组 3/5；加修正后 24/24** | 第 1、2、5 轮建立链路；第 3–4 轮暴露低构型误差；随后 24 对试验测得 12/24（原始目标）vs 24/24（修正后） |
| 关节限位 | **2026-11-19 正式普查；`mapping_verified=true`** | 五个关节全部手推实测（释放扭矩）；止挡为姿态相关的自碰撞限位；joint2 上限 810→775 |
| 无头抓取工具链 | **已验证** | `grasp_150mm` 工具、`moveit_execute`、`pulse_move`；每条机械臂规划均经安全否决 |
| 安全层 | **已实现；Mock 验证 + 实机否决** | 看门狗、关节/速度/加速度/起始状态/时长/工作空间检查；桌面平面间隙；无急停或抱闸 |
| 测量协议 | **已实现；重复性、回差与夹爪包络已实测** | 30 次重复性 0.449 mm RMS / 1.017 mm max；回差 1.117 mm；46 次容差网格（开口轴 ±9 mm）；视觉位移仍仅仿真 |
| 误差预算 | **框架已实现；部分完成（三项待测）** | 重复性、回差与容差已测；平移 RSS 3.995 mm 通过实测 6 mm 容差，最坏 7.114 mm 未通过；旋转 1.103° / 1.827° 通过暂定 2°；抓取执行、夹爪误差源与夹具位姿未测 |
| 配对试验 | **已实现；首次实机试验完成（2026-11-19）** | 24 对 / 48 次：baseline 12/24 vs intervention 24/24，McNemar p = 0.00049；12 个双成功对子中干预放置误差低 2.19 mm（95% CI −3.96…−0.83，p = 0.0063）；记录 4 项偏差 |
| 主机/DDS 基准 | **已实现；只读实机快照** | 笔记本 + Pi 指标；状态 20 Hz；相机约 12 Hz；ping 0.24 ms |
| Debug Studio | **G1–G6 已实现；测试通过** | 后端 92 + 网关 38；Mock 10 + 硬件 3 个 Playwright；G5 运动采集；G6 受监督路径无运动验证 |
| 控制板 | **已探测（2026-11-19）** — 电池、6 路总线舵机、4 路开环电机占空比、蜂鸣器与 LED1/2 为真实功能；IMU、编码器、急停、板载手柄/SBUS 不存在；KEY1/2 已拆除 | [`evidence/phase2/onsite_20261119/board_capabilities/`](evidence/phase2/onsite_20261119/board_capabilities/) |
| 移动底盘 | **仅开环；计划扩展** | 4 路电机占空比，无编码器/IMU/里程计；0.5 s 看门狗；任何底盘运动都会使桌面标定失效 |

历史学习路线：Panda 抓取放置演示（`arm_moveit_demo`）与 M2.2 双自由度控制器
配置（`arm_description`）属于归档的学习产物，证据在
[`evidence/m2_2_v2/`](evidence/m2_2_v2/) 与 [`docs/archive/`](docs/archive/)；
它们不属于本毕业课题的结果。

## 毕业项目交付清单

- [x] 问题定义、范围与平台约束
- [x] 修正模型、TF 树与实测相机坐标系
- [x] 自主标定与独立验证
- [x] 约束规划与抓取候选搜索
- [x] 安全否决下的实机执行
- [x] 带逐轮证据的量化结果
- [x] 失败模式分类与批判性分析
- [x] 可复现命令与测试套件
- [ ] 完整误差预算——重复性、回差与容差（±9 mm）已测；末端放置误差 3.31 mm；执行、夹爪放置与夹具位姿仍待测
- [x] 具备统计功效的配对实机试验（2026-11-19）：24 对，McNemar p = 0.00049，配对放置改善 2.19 mm（p = 0.0063）
- [x] 低构型模型标定（2026-11-19）——关节偏置假设被否决，实测包络内支持常数偏置
- [x] 正式机械限位测量与 `mapping_verified=true`（2026-11-19；止挡与姿态相关，按作业包络认定）

## 快速开始

### 环境要求

- 工作站 ROS 2 Jazzy；机器人 ROS 2 Humble + CycloneDDS。
- Humble MoveIt/RViz 的 Docker 容器（`tools/humble_moveit/`）。
- Python 3.10+、Node 20+（调试台）。
- 实机运行还需：直连以太网、厂商 `start_node.service`，以及
  [`docs/hardware/network_access.md`](docs/hardware/network_access.md) 中的网络锁定。

### 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install

# 纯 Python CI：误差预算、配对试验、基准、测量、分割
python -m pytest -q \
  ros2_ws/src/armpi_analysis/test/test_error_budget.py \
  ros2_ws/src/armpi_analysis/test/test_paired_trials.py \
  ros2_ws/src/armpi_analysis/test/test_host_benchmark.py \
  tools/phase2/tests/test_grasp_joint_fit.py \
  tools/phase2/tests/test_low_config_model_fit.py \
  tools/phase2/tests/test_measurement_protocol.py \
  tools/phase2/tests/test_object_segment.py \
  tools/phase3/tests \
  tools/phase4/tests

# 调试台：后端 92、网关 38、Mock Playwright 10
cd armpi-debug-studio/backend && pytest
cd ../ros2_gateway && pytest
cd ../frontend && npm run test:e2e
```

### Mock 链路（无硬件）

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# 模型 + MoveIt Mock
xacro ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro > /tmp/armpi.urdf
check_urdf /tmp/armpi.urdf
ros2 launch armpi_ultra_moveit_config demo.launch.py \
  ros2_control_hardware_type:=mock_components \
  use_sim_time:=false use_gazebo:=false use_real:=false

# 6D 约束目标（仅规划）
python3 tools/phase0/constrained_plan_check.py \
  --pipeline pilz_industrial_motion_planner --planner PTP

# fiducial → 抓取 → 安全 E2E，隔离域 77
tools/phase2/run_mock_visual_grasp_e2e.sh evidence/phase2/session3_20261117
```

### 实机运行（仅受监督）

实机流程刻意不提供一条可直接复制执行的命令。必需步骤见：

- [`docs/hardware/network_access.md`](docs/hardware/network_access.md) — 有线链路、防火墙、网关绑定。
- [`tools/humble_moveit/README.md`](tools/humble_moveit/README.md) — Humble MoveIt/RViz 容器与 CycloneDDS。
- [`docs/phase2/JOINT_LIMITS.md`](docs/phase2/JOINT_LIMITS.md) — 采纳限位、测量与恢复规则。
- [`docs/phase2/SAFETY_LAYER.md`](docs/phase2/SAFETY_LAYER.md) — 否决服务与运行包络。
- [`docs/hardware/camera_frames.md`](docs/hardware/camera_frames.md) — body 与 optical 坐标系、`camera_optical_calibrated`。

2026-11-19 的抓取使用 `tools/phase2/real_moveit_execute.py`、
`tools/phase2/object_segment_rgbd.py`、`tools/phase2/depth_plane_fit.py`
以及 Studio G5/G6 路径；详见
[`evidence/phase2/onsite_20261119/`](evidence/phase2/onsite_20261119/)。

### 离线分析（不运动）

```bash
# 误差预算：仓库内部分预算 + 合成 demo
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_baseline.json \
  --markdown-out /tmp/error_budget.md

# 配对试验：计划、合成运行、分析
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  plan --pairs 24 --seed 7 --json-out /tmp/paired_plan.json
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  analyze --input /tmp/paired.jsonl --markdown-out /tmp/paired.md

# 只读主机/DDS 基准
tools/phase4/run_benchmark.sh evidence/phase3/session4_20261117/with_camera
```

## 实验方法

### 标定

相机安装在 `link4` 上、位于最终腕部旋转之前，因此标定的是 `link4 → camera`，而不是直接的夹爪→相机变换。棋盘采集使用舵机实测位置而非命令回显。验证采用三种独立程序：板位姿一致性、留出基位姿与留一交叉验证。桌面下俯采集针对实际工作视角重新拟合变换，实体 AprilTag 提供独立尺度检查。厂商手眼矩阵对当前安装已过期，**不**作为 Baseline A。

### 抓取

物体不使用标记：地面平面 RANSAC、体素聚类与顶面切片拟合给出位置、尺寸和偏航。约束抓取搜索在五自由度可达集内采样，筛选全位姿 IK、校验 FK、检查碰撞，并用 Pilz `PTP` 规划。独立安全节点在执行前否决轨迹。执行以舵机实测完成判定，而不是命令回显。只有物体在已验证视角下从地面消失，才判定抓取成功；安装在运动臂上的相机离被夹物体太近，不能作为可靠的夹持传感器。

### 关节限位与安全

限位按舵机 EEPROM 减 20 脉冲（0.0838 rad）采纳。2026-11-19 的正式普查在竖直位姿逐个关节释放扭矩手推实测（joint2 另在低位作业位姿测了一次）：止挡是**自碰撞限位、随姿态变化**（joint2 低位 795 / 竖直 873；joint3 939 / 923），因此静态限位按作业包络认定，其余姿态由 MoveIt 的自碰撞检查兜底；joint2 上限收紧到低位手推止挡（775 脉冲）。夹爪在发现原始张开值只是卡滞而非止挡后重新测量。厂商节点在读到负舵机寄存器时会因 `uint16` 断言崩溃，因此所有测量运动单调推进，并带自由下落中止保护。

### 模型与规划注意事项

- 仓库默认使用 KDL 全位姿 IK（`position_only_ik: false`）；未安装 TRAC-IK。
  仅位置可达性检查使用单独配置。
- 约束目标使用 Pilz `PTP`：OMPL 的目标采样器在 KDL 下无法求解该 6D 目标，
  而 `LIN` 会离开五自由度可达集。
- 厂商预生成的 URDF 已归档；只使用 xacro。
- 厂商手眼矩阵对当前安装已过期，不作为 Baseline A。

### 可靠性分析

误差预算框架区分已测与未测来源，报告 RSS、最坏情况、主导项与 Monte Carlo 区间。配对试验记录器采用追加式 JSONL、带种子的对内随机化与精确配对检验；预注册的 24 对试验是它首次实机使用。运动项用**固定 AprilTag** 测量（重复性、回差、偏航延伸）；**容差**来自 46 次偏移网格，且每次放置都经过木块高度/尺寸与朝向门控，确保测的是夹持容差而不是几何。主机/DDS 基准为只读，测量两端主机的 topic 速率、抖动与消息年龄。除显式受监督的运行器外，没有测量工具会命令抓取。

## 失败模式分类（实测）

| # | 发现 | 后果 | 状态 |
|---|---|---|---|
| 1 | 夹爪“张开”值 −0.8378 rad 是卡滞，不是连杆止挡 | 手指只张开约 60%，木块可能卡住 | **已修复**：行程重测为 2.22 rad |
| 2 | FK/手眼误差与构型相关：低抓取位姿为 −8.0/−15.3/−5.8 mm，桌面下俯为亚毫米 | 第 3–4 轮在木块旁闭合 | **已缓解**：单点 Tag 修正；根因待解 |
| 3 | 深度相机近端截止 0.176 m；低构型下物体距相机 72 mm | “物体位于视野中心”门控物理上不可行 | **已记录**；目标改由桌面下俯视角提供 |
| 4 | 用距被夹物体 0.10 m 的相机判断夹持 | 第 3–4 轮出现错误的“已夹住”证据 | **协议已修复**：在视角位姿检查地面原位 |
| 5 | 地面可达边界使 TCP 沿手指方向差 5.9 mm | 抓取位于工作空间边缘 | **已知**；使用深关节族 |
| 6 | 负舵机寄存器使厂商 `get_state` 回调崩溃 | 限位测量期间 4 次节点崩溃 | **已缓解**：单调测量、自由下落中止、绝不读负值 |
| 7 | 机器人无 RTC、无 NTP；`/joint_states` 到达即过期 | MoveIt 在运动前中止 | **操作修复**：机器人时钟比工作站快 1–2 s |
| 8 | AprilTag 尺寸错误（估计 73 mm，实际 60 mm） | Tag 被放到地面以下 59 mm | **已修复**：用直尺实测尺寸 |
| 9 | `armpi_safety` 拒绝仅夹爪轨迹（“FK failed: joint1”） | MoveIt 无法 `--execute` 夹爪动作 | **未解决**；夹爪走 `/gripper_controller` |
| 10 | 底盘运动使桌面标定失效；无里程计/IMU | 任何底盘移动后必须重标定 | **已知**；移动扩展单列 |

## 安全与运行包络

- **没有急停、没有抱闸。** 断电会使机械臂下落；厂商栈约 89 s 自动恢复。所有运动均受监督。
- **采纳限位是可指令限位，不是机械保证。** 2026-11-19 普查后 `mapping_verified=true`，但止挡是姿态相关的自碰撞限位；后端与 Studio 仍默认关闭运动。
- **绝不读取或指令负舵机寄存器。** 它会使厂商节点崩溃；恢复时的节点重启本身就会移动机械臂。
- **MoveIt 执行需要时钟领先。** 跨主机执行前，将机器人时钟设为比工作站快 1–2 s。
- **`/joint_states` 是命令回显。** 实测真值是 `/controller_manager/servo_states`；寄存器读数用 `get_state` 服务。
- **低构型对深度相机是盲区。** 近端截止 0.176 m；不要从低视角规划抓取。
- **入站只走有线链路。** 默认拒绝的 nftables，仅允许 `eth0`，SSH 仅密钥，网关绑定 `10.42.0.218`。
- **没有自主运行。** G6 路径为单关节、步长受限，并要求显式确认短语。

## 局限性与有效性威胁

- **单一平台、单一物体、单一会话。** 24 对试验给出的是该夹具、该木块、该电池状态下的效应，不是总体成功率估计；连续误差比较以 baseline 也成功的 12 对为条件。
- **低构型修正是有包络的。** 固定 Tag 普查证明它在实测包络内是常数（散布 0.40–0.99 mm）；关节偏置与 `link4` 偏置模型把修正量预测偏差 10.6–15.0 mm 并被否决。偏航延伸实验把包络测到 `j1 ≈ 0.5 rad`：该处所需修正 (9.14, 7.18, 8.75) mm，常数差 5.6 mm，拟合模型差 14–19 mm 且 y 反号。
- **误差预算不完整。** 重复性（0.449 mm RMS）、回差（1.117 mm）与容差（±9 mm）已测；抓取执行、夹爪放置与夹具位姿仍未测，2° 旋转门限仍是暂定值。
- **48° 斜视下绝对深度有偏。** RGB 与深度在该视角相差约 40–50 mm；桌面下俯与深度平面估计才是可靠参考。
- **手眼验证是板相对的。** 它证明一致性而非绝对计量；Tag 尺度检查是独立的物理锚点。
- **关节限位已正式普查但依赖姿态。** 五个关节在 2026-11-19 释放扭矩手推实测；止挡是自碰撞限位，`mapping_verified=true` 仅覆盖作业包络。
- **无力/力矩传感、无急停、无抱闸。** 接触丰富操作与自主恢复不在范围内。
- **Mock 与 Gazebo 结果从不作为硬件验证。** `mock_components` 与合成标记只验证软件路径。

## 可复现性与证据索引

本 README 中的每条量化结论都可追溯到以下证据包之一：

| 领域 | 证据 |
|---|---|
| 硬件基线 | [`evidence/phase1/20261117/`](evidence/phase1/20261117/)、[`docs/hardware/README.md`](docs/hardware/README.md) |
| 模型 / MoveIt | [`evidence/armpi_moveit/`](evidence/armpi_moveit/)、[`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md) |
| 标定 | [`evidence/calibration/20261117/`](evidence/calibration/20261117/)、[`evidence/calibration/20261118/table_down/`](evidence/calibration/20261118/table_down/) |
| Fiducial | [`evidence/fiducial/20261116/`](evidence/fiducial/20261116/)、[`evidence/phase2/real_background_20261117/`](evidence/phase2/real_background_20261117/) |
| 抓取搜索 | [`evidence/armpi_grasp/20261116/`](evidence/armpi_grasp/20261116/) |
| 现场安全 / 限位 | [`evidence/phase2/onsite_20261118/`](evidence/phase2/onsite_20261118/) |
| 实机抓取 / 可达 | [`evidence/phase2/onsite_20261119/`](evidence/phase2/onsite_20261119/) |
| 可靠性分析 | [`evidence/phase3/onsite_20261119/`](evidence/phase3/onsite_20261119/README.md)（配对试验、重复性、回差、容差网格、偏航包络、误差预算）；框架基线 [`evidence/phase3/session4_20261117/`](evidence/phase3/session4_20261117/) |
| 关节限位 | [`evidence/phase2/onsite_20261119/joint_limits/`](evidence/phase2/onsite_20261119/joint_limits/README.md) |
| Studio G1–G6 | [`evidence/studio_gateway/`](evidence/studio_gateway/)、[`studio_g2/`](evidence/studio_g2/)、[`studio_g3/`](evidence/studio_g3/)、[`studio_g4/`](evidence/studio_g4/)、[`studio_g5/`](evidence/studio_g5/)、[`studio_g6/`](evidence/studio_g6/) |
| 安全加固 | [`evidence/security/20261118/`](evidence/security/20261118/) |

仓库保留原始日志与机器可读 JSON；当图像是主要证据时也会保留。`*.mcap` bag 已 gitignore，需单独备份。`evidence/README.md` 说明了跟踪范围。

## 文档

- [技术路线图](ROADMAP.md) — 公开技术路线图与后续步骤。
- [变更日志](CHANGELOG.md) — 按会话记录的时间顺序变更。
- [验收报告](docs/phase0/PHASE0_ACCEPTANCE.md) — Phase 0 检查清单、当前毕业项目验收结论与剩余缺口。
- [5-DoF 约束](docs/phase0/ARMPI_5DOF_CONSTRAINTS.md) — 任务空间限制与抓取策略。
- [标定](docs/phase2/CALIBRATION.md) — 棋盘手眼流程与验证结果。
- [标定位姿选择](docs/phase2/CALIBRATION_POSES.md) — 离线 24 位姿选择。
- [抓取候选搜索](docs/phase2/GRASP_CANDIDATE_SEARCH.md) — 候选生成与筛选。
- [Fiducial 定位](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md) — AprilTag 位姿约定与验证。
- [Fiducial → 抓取桥接](docs/phase2/FIDUCIAL_GRASP_BRIDGE.md) — TF、搜索与安全否决。
- [安全层](docs/phase2/SAFETY_LAYER.md) — 看门狗、限位与否决。
- [测量协议](docs/phase2/MEASUREMENT_PROTOCOL.md) — 重复性、回差、视觉位移、夹爪包络。
- [关节限位](docs/phase2/JOINT_LIMITS.md) — 测量、EEPROM 边界与恢复规则。
- [误差预算](docs/phase3/ERROR_BUDGET.md) — schema、RSS 与缺失项。
- [配对试验](docs/phase3/PAIRED_TRIALS.md) — 记录器、随机化与分析；设计预注册在
  [PAIRED_TRIAL_DESIGN.md](docs/phase3/PAIRED_TRIAL_DESIGN.md)。
- [主机/DDS 基准](docs/phase3/HOST_BENCHMARK.md) — 只读指标与相机速率修正。
- [相机坐标系契约](docs/hardware/camera_frames.md) — body 与 optical。
- [网络访问](docs/hardware/network_access.md) — 有线链路与安全锁定。
- [厂商标定基线](docs/hardware/vendor_calibration_baseline.md) — 矩阵解码与 Baseline A 状态。
- [2026-11-15 审计](docs/audit/2026-11-15/README.md) — 来源、冲突、状态与决策。
- [Debug Studio PRD](docs/armpi/ARMPI_DEBUG_STUDIO_PRD_V1.md)
- [G0 厂商审计](docs/armpi/g0/README.md)
- [8 周学习日志](docs/weekly-log/)

`MASTER_PLAN_V3_PRIVATE.md` 有意**不**放入这个公开仓库。

## 结论

本工作支持一个范围有限但有意义的结论：在该平台上，借助实测标定、约束规划与安全否决，可以对无标记 39.4 mm 木块完成自主抓取—抬升—放置；主要残余误差来自运动学，而不是感知。第 1–2 轮建立了完整链路；第 3–4 轮定位了低构型模型误差；固定 Tag 普查证明它在实测包络内是常数，并用抓取验证到 1.3 mm；随后又用同一批数据否决了关节偏置与 `link4` 偏置模型。预注册的 24 对实机试验随后完成：抓取成功率从 12/24 提升到 24/24（McNemar p = 0.00049），双成功对子的放置误差配对改善 2.19 mm（p = 0.0063）。同一批证据也说明了尚未解决的部分：修正是常数量而非构型函数，包络之外未验证；误差预算仍不完整；单会话结果不足以支撑总体可靠性结论。

## 路线图

**下一步（最高优先级）。** 在实测包络边界（`j1 ≈ 0.3–0.6 rad`）用一次偏航位形**抓取**检验常数修正。偏航延伸实验已表明常数模型在该处比拟合模型更接近实测；此前两次偏航抓取失败只是因为方形木块在 20–30° 方位角下以约 50 mm 对角线面对约 40 mm 的开口，因此必须先把木块转到正对夹爪。随后补齐误差预算剩余三项（抓取执行、夹爪放置、夹具位姿），并用主导剩余项作为干预再跑一轮配对试验。

**随后。** 为 `armpi_safety` 增加仅夹爪轨迹支持；把物体集从 39.4 mm 木块扩展到更多形状；补齐误差预算剩余三项。可选移动底盘只在固定基座链路稳定后推进。

## 许可证

见 [LICENSE](LICENSE)。
