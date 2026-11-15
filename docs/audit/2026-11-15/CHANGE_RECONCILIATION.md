# CHANGE_RECONCILIATION — 逐文件差异、独有成果与冲突对照

> 对应来源编号见 `SOURCE_INVENTORY.md`。所有比较基于 SHA-256 内容哈希与 Git 共同祖先，**不依据文件修改时间**。
> 本轮未修改任何被审计文件；未执行 Git 写操作。

## 0. 方法

1. 对 S01 与 S02 的 66 个跟踪文件分别生成 `sha256sum` 清单，逐路径比较。
2. 对 S03 的 73 个非构建文件生成清单，与 S01/S05 比较。
3. 对 M2.2 相关文件使用三路比较：`HEAD` blob（435f8ec） / S01 工作区 / S03 工作区。
4. 对非 Git 工作区（S06/S07）按路径和哈希比较厂商原件。
5. 对证据文件（S08/S09）做 24 文件逐一哈希比较，并验证 `SHA256SUMS.txt` 与 mcap。
6. 对未合并分支（`phase2`、`agent/JER-18`）用 `git log main..branch`、`git diff main...branch` 比较。

## 1. 完全相同的内容（可去重）

| 对象 | 范围 | 结果 |
|---|---|---|
| S01 vs S02 跟踪树 | 66 个文件 | **64 个完全相同**；仅 `gazebo.launch.py`、`simple_arm.xacro` 两个文件不同（S01 有未提交修改） |
| S05 vs S03 | 25 个文件 | **25/25 完全相同**；S05 是 S03 的真子集，无独有内容 |
| S08 vs S09 | 24 个文件 | **24/24 完全相同**；仅目录名 `pre_sync_backup/` ↔ `config_snapshot/` 不同 |
| `armpi_ultra.xacro` | S06 vs 厂商 A 版 | 相同（`158224…`） |
| `armpi_ultra.urdf` | S06 vs 厂商预生成 URDF | 相同（`00dc26…`，**陈旧版本**） |
| 14 个 STL | S06 vs 厂商 A 版 | 全部相同 |
| `simulations.zip` | `ArmPiUltra/3 功能包文件/` vs `Downloads/` | 相同（`37599f…`） |
| `arm_controllers_gain_0p2.yaml` | S01 未跟踪 vs S03 rework1 | 相同（`5ceea2…`） |
| `arm_controllers.yaml` | S01 / S03 / S08 快照 | 相同（`c85bf8…`） |

> 结论：S05 可以直接清理（需批准）；S09 与 S08 只需保留一份；S06 的 URDF 不是有效基线，见 §3.7。

## 2. 单侧新增或修改的成果

### 2.1 仅存在于 S02（M70q）的成果

| 成果 | 路径 | 规模 | 说明 |
|---|---|---|---|
| ArmPi Debug Studio G1 Mock 切片 | `armpi-debug-studio/` | 28 个 Python 文件（4,178 行）+ 16 个 TS/TSX（2,895 行）+ 配置/文档；含 `.venv`、`node_modules`、`dist`，总 177M | 未跟踪；不在 GitHub；R9000P 无副本 |
| G0/阶段审计 | `docs/phase0/` | 2 份（JER-39、JER-59） | 未跟踪；不在 R9000P |

Studio 关键事实：

- `README.md` 声明 "G1 Mock-first vertical slice"；Mock 端到端可运行，Hardware 只读占位，`hardware_motion=false`。
- 测试：37 个 `test_*` 函数 + 2 处 `parametrize`；README 声称 41 个用例；`docs/g1-verification.md` 声称 `41 passed`。
- 真实 HTTP E2E 证据：`data/studio.db` 有 **55 条 events、1 条 command**（只读查询确认）。
- Playwright：`g1-verification.md` 声称 9 项 PASS，但仓库中**没有 Playwright 测试文件/配置**（`find` 无结果）。
- E7 ROS 2 gateway **未实现**；`safety/watchdog.py`、`domain/faults.py`、`adapters/gateway.py`、`realtime/state.py` **不存在**。
- PRD §14 的 G1 验收要求"有人看护的实机验收通过"——**未发生**，因此 Studio 只能算 "G1 Mock 切片"，不是 PRD 意义的 G1 通过。

### 2.2 仅存在于 S03（M70q worktree）的成果

| 成果 | 路径 | 说明 |
|---|---|---|
| Gate 2 rework1 实现 | 4 个修改文件 + `arm_description/controller_config.py` + `test/test_controller_config.py` + 0.1/0.2 gain YAML | 未提交；`m2-2-v2` 无任何提交 |
| 离线测试证据 | `evidence/m2_2_v2/gate2/20261112T014542Z-rework1/implementation-report.md` | 自述 9 tests / 0 failures；`colcon build/test` 通过；**未启动 Gazebo/ROS** |
| 早期 Gate 2 报告 | `evidence/m2_2_v2/gate2/20261111T0547Z/implementation-report.md` | 与 rework1 的 gain 哈希不同，说明中间有过一轮返工 |

rework1 的关键设计：

- 唯一 gain 叶为 `/gz_ros_control.ros__parameters.position_proportional_gain`；
- 校验 helper 安装为 `arm_description.controller_config` Python 包；
- 严格校验 V2/default schema、YAML 特性、路径（支持 `~`，拒绝环境变量/控制字符/空白）；
- 分阶段 watchdog（60s）与 `on_exit_shutdown:=true`；
- Xacro 默认值改为 `$(find arm_description)/config/arm_controllers.yaml`（可移植）；
- 新增离线回归：schema、路径、YAML 错误、事件链、source/install launch 载入、模型检查。

### 2.3 仅存在于 S01（R9000P）的成果

| 成果 | 路径/哈希 | 说明 |
|---|---|---|
| M2.2 本地运行时变体 | `gazebo.launch.py` `ee85…`；`simple_arm.xacro` `b3be…`；`arm_controllers_gain_0p2.yaml` `5ceea…` | **正是 2026-11-10 四个 mcap 的源码快照**（S08/S09 的 `SHA256SUMS.txt` 逐一对应） |
| 本地未跟踪 gain profile | `config/arm_controllers_gain_0p2.yaml` | 与 S03 rework1 的 0.2 profile 相同 |
| 本地实测 bags | S08/S09 | 见 §4 |

### 2.4 仅存在于 S06（R9000P 仿真工作区）的成果

| 成果 | 说明 |
|---|---|
| `armpi_ultra_description` + `armpi_ultra_moveit_config` | 从厂商 `3 功能包文件/simulations/` 复制；MoveIt config 由厂商 Setup Assistant 生成（timestamp 1751454082 = 2025-07-02） |
| `joint_limits.yaml` 修改 | 仅对 joint1–5 启用 `max_acceleration: 1.0`；gripper 关节仍为 `false` |
| 运行证据 | `~/.ros/log` 2026-11-14 13:53–13:58：`move_group`、`ros2_control_node`、`rviz2` 启动；3 条 arm 轨迹 `Goal reached, success!` / `SUCCEEDED` |

### 2.5 仅存在于 S07（R9000P 厂商目录）的文档

| 文档 | 说明 |
|---|---|
| `ARMPI_DEBUG_STUDIO_PRD_V1.md` | PRD v1.0，2026-11-13；Proposed / 待 Board 审批 |
| `docs/g0/README.md` + 01–07 | G0 只读审计交付；状态 `Analysis complete / 待 Board 评审`；无 `Hardware validated` 结论 |

### 2.6 仅存在于 S13（USB）的计划

| 文档 | 说明 |
|---|---|
| `MASTER_PLAN_V3_PRIVATE.md` | 18,608B，10-26；私有计划；禁止原样进公开仓库 |
| `ROADMAP_PUBLIC.md` | 4,656B，10-26；公开技术路线 |

### 2.7 未合并分支上的成果（不在任何工作树）
| 分支 | 独有提交 | 文件/行数 | 内容 |
|---|---|---|---|
| `origin/phase2/week01-execution-reliability` | `b73f804`、`a2c51bf`、`fdca2e0`、`e596c5a` | 10 文件，483+/161- | pick/place 参数化 + launch 覆盖；typed controller action readiness gate；plan/execute 失败退出；CHANGELOG；`docs/evolution/phase2/week01.md`；`docs/learning-roadmap.md`；ROADMAP 重构 |
| `origin/agent/JER-18-remediation` | `8007c7a`、`1bbb136` | 6 文件，189+/38- | 与 phase2 的 `b73f804`/`a2c51bf` 同源的代码；phase2 是其超集 |

> phase2 分支文档明确写："Remaining: Run default, planning-failure, and execution-failure regressions in a ROS 2 Jazzy + MoveIt 2 environment." 因此它是**实现完成、运行回归未完成**，且未进入 `main`/`panda-baseline-v1`。

### 2.8 审计期间新建：S17 `armpi_ultra_ws_clean`

- 并行会话 `sess_83d860c9` 在 2026-11-15 11:20–11:24 创建 `/home/jerichocheng/Projects_M70q/armpi_ultra_ws_clean`：28M、248 文件、12 个包、无 `.git`。
- 含 `docs/removed_files.tsv`（删除/排除清单）、`docs/original/`（6 个原始文件）、`tests/test_mecanum_watchdog.py`，并修改了 `kinematics/transform.py`、`controller/mecanum.py`、`ros_robot_controller_node.py`、`bringup/core_check.py`。
- 该目录**不在本轮文件级比较范围内**，且会话仍在写入；不能把它当作"已完成/已验证"的成果。它可能成为后续厂商精简基线，需等会话结束、输出报告后再审计。

## 3. 同一文件的不同修改（真实差异）

### 3.1 `ros2_ws/src/arm_description/launch/gazebo.launch.py`

| 版本 | SHA-256 | 语义 |
|---|---|---|
| HEAD `435f8ec`（S01/S02 跟踪版本） | `29f6a1c7a671fb22b0b34592edf11a9f647e7d64cac7e21bf7ecf68252e7151f` | `xacro.process_file()` 直接展开；无 launch 参数；插件路径由 xacro 内绝对路径决定 |
| S01 本地工作区 | `ee85a1e46781c67112144828d5f546132f2c35f00439bdbed8281cd917dc159a` | 增加 `controller_config` LaunchArgument + `Command([xacro ...])`；默认值仍是 xacro 内绝对路径；无 watchdog/失败关闭 |
| S03 rework1 | `61cb84159e68cf4fe880b3edf585aabdee201462d0e422e708adf3efa54dc3b4` | 从安装的 `arm_description.controller_config` 导入校验 helper；严格校验；分阶段 60s watchdog；`gz_sim.launch.py` 显式 `on_exit_shutdown:=true`；source/install 均测过 |

**判定：真实冲突（同一功能的三种实现）。** S01 版本是 11-10 实测所用；S03 是 11-12 的强化版但未运行。不能按时间简单覆盖。

### 3.2 `ros2_ws/src/arm_description/urdf/simple_arm.xacro`

| 版本 | SHA-256 | 语义 |
|---|---|---|
| HEAD（S01/S02） | `8de7c33f71290c71b4b72fa16ca2188578a9594b2e7afac7ca718e1c4830dd9f` | 插件 `<parameters>` 写死 `/home/jerichocheng/projects/.../install/.../arm_controllers.yaml` |
| S01 本地工作区 | `b3be05fa77d4cdcc8e9e13e4b67cec109f8231b1321b7b6fb014cba089022b35` | 增加 `xacro:arg controller_config`，默认值仍是本地绝对路径 |
| S03 rework1 | `a4f89f14aee8248bfc216e6b9929f1ae819d4b236e290c270aeb8111501314e1` | 默认值 `$(find arm_description)/config/arm_controllers.yaml`；可移植；配合 helper 校验 |

**判定：真实冲突；S03 明显更适合作为长期基线。** 但 S01 版本是实测过的，需保留为历史分支/补丁。

### 3.3 `CMakeLists.txt` 与 `package.xml`

| 文件 | HEAD | S03 rework1 | 判定 |
|---|---|---|---|
| `arm_description/CMakeLists.txt` | `4a2519…` | `895292…` | 仅 S03 新增 `ament_cmake_python`、安装 Python 包、pytest |
| `arm_description/package.xml` | `f8b497…` | `b66ed7…` | 仅 S03 新增依赖（launch/launch_ros/yaml/gz_ros2_control/...）与 test_depend |

**判定：单侧新增，无本地对应修改；应采用 S03 版本**（需先确认依赖在 R9000P 可用）。

### 3.4 `arm_controllers_gain_0p2.yaml`

- S01 未跟踪文件 = `5ceea231…`
- S03 rework1 = `5ceea231…`
- S08/S09 快照 = `5ceea231…`

**判定：完全相同，单副本保留即可。** 注意早期 Gate 2 报告中的 0.2 哈希是 `097c2000…`，已被 rework1 取代。

### 3.5 `arm_moveit_demo/src/pick_place_demo.cpp`（Panda 线）

| 版本 | 来源 | 变化 | 状态 |
|---|---|---|---|
| `main` `616ac80` | 2026-11-04 | 13 行：plan/execute 失败即停 | 已合并进 `main` 与 tag |
| `phase2` `b73f804` + `a2c51bf` + `fdca2e0` | 2026-10-10 | 186 行：参数化位姿、launch 覆盖、typed action readiness gate、失败退出 | **未合并** |
| `agent/JER-18` `8007c7a` + `1bbb136` | 2026-10-10 | 150 行：与 phase2 同源代码的子集 | **未合并**，被 phase2 覆盖 |

**判定：同一任务的三条实现线。** `main` 的修复较窄；`phase2` 功能更完整但未做运行回归；`agent/JER-18` 冗余。不能自动合并，需 Board 选择"保持冻结基线"或"把 phase2 作为 Panda 线的最后一次性合并"。

### 3.6 厂商 `armpi_ultra.xacro` 两个版本

| 版本 | SHA-256 | 时间 | 差异 |
|---|---|---|---|
| A：`3 功能包文件/simulations/...`（S06 使用） | `158224…` | 2025-07-02 | `base_link` collision 用 `box 1.0 1.0 0.025` |
| B：`ROS2/Armpi_ultra-main/.../simulations/...` | `7ce1d0…` | 2026-04-02 | `base_link` collision 用 `base_link.STL`；缩进不同 |

**判定：B 更新，差异仅 base_link collision。** S06 使用 A；建议以 B 为参考重新生成，但不要在未验证时直接替换已跑通的 A。

### 3.7 预生成 `armpi_ultra.urdf` vs `armpi_ultra.xacro`（模型内部冲突）

| 项 | `armpi_ultra.urdf`（`00dc26…`） | `armpi_ultra.xacro` 展开结果 |
|---|---|---|
| `joint1` | **fixed** | **revolute**，axis `0 0 1`，±2.09 |
| `r_joint` | **fixed** | **revolute**，±1.57 |
| gripper 联动 | 无有效运动链 | `r_joint` + 5 个 mimic 关节 |
| 用途 | 被 `setup.py` 一并安装；`launch` 实际用 xacro | MoveIt demo 与 display 均用 xacro |

**判定：陈旧 URDF 是危险遗留物。** 它物理上存在于包内，任何按文件名取 URDF 的工具都会拿到错误模型（4 个臂关节 + 固定夹爪）。必须在合并时删除/重新生成/加显著标注。

### 3.8 `joint_limits.yaml`（S06 vs 厂商）

- S06 修改：仅 joint1–joint5 `has_acceleration_limits: true` / `max_acceleration: 1.0`。
- 未修改：gripper 的 `l_in_joint`、`l_joint`、`l_out_joint`、`r_in_joint`、`r_joint`、`r_out_joint` 仍为 `false`。
- 运行后果：`move_group` 日志明确报 `No acceleration limit was defined for joint l_in_joint!`，gripper 规划失败；arm 规划随后成功。
- 另：该文件没有 `max_position/min_position`；位置限位完全取自 URDF 的 ±2.09。

**判定：部分修复。** 合并时应补齐 gripper 加速度限制，并把位置限位显式化（至少与软件范围一致），否则同类失败会重复。

### 3.9 `initial_positions.yaml` vs 厂商启动姿态

| 关节 | MoveIt 初始 (rad) | 厂商启动 pulse | 换算 (rad) | 差异 |
|---|---:|---:|---:|---|
| joint1 | 0.000 | 500 | 0.000 | 0 |
| joint2 | 0.7309 | 600 | 0.419 | +0.312 |
| joint3 | -1.6127 | 800 | -1.257 | -0.356 |
| joint4 | -1.6591 | 110 | -1.634 | -0.025 |
| joint5 | 0.000 | 500 | 0.000 | 0 |
| r_joint | 1.5700 | 210 | -2.052 | 方向/量级均不符；1.57 超出软件上限 1.257 |

**判定：仿真初始状态与真实启动姿态不一致。** 若把 `initial_positions.yaml` 当作"当前状态"喂给 MoveIt，规划起点是假的；接实机前必须修正或改为从真实状态读取。

### 3.10 文档状态冲突

| 文档/记录 | 声明 | 实际 |
|---|---|---|
| `docs/g0/README.md` | "Analysis complete / 待 Board 评审"；"不含实现代码" | S02 已有 G1 Mock 代码 |
| ZCode memory（11-14） | "There is no backend, frontend, ROS 2 gateway, Mock adapter, tests, or CI code" | 代码在 11-14 14:05–14:23 落盘；memory 写于此前 |
| `docs/phase0/JER-39` | "仓库中没有任何 ArmPi Ultra 资产" | 仓库内仍没有；但 S06 已有 ArmPi 模型/MoveIt |
| `README.md` | 8 周学习项目、WSL2 | 当前为原生 Ubuntu 24.04；项目已进入 ArmPi 阶段 |
| `ROADMAP.md` | 4 个未勾选项（pick_place_demo、launch、README、demo video） | 这 4 项均已完成（README/demo/weekly-log 可证） |
| `armpi-debug-studio/README.md` | "41 backend tests" | 静态 37 个测试函数 + 2 处 parametrize；未重跑 |
| `g1-verification.md` | Playwright 9 项 PASS | 仓库内无 Playwright 文件，不可复现 |

## 4. 证据链一致性核对

### 4.1 M2.2 bags 与源码快照

`S08/S09/evidence/SHA256SUMS.txt` 中与源码相关的哈希：

```text
b3be05fa…  src/arm_description/urdf/simple_arm.xacro
ee85a1e4…  src/arm_description/launch/gazebo.launch.py
c85bf801…  src/arm_description/config/arm_controllers.yaml
5ceea231…  src/arm_description/config/arm_controllers_gain_0p2.yaml
```

这 4 个哈希与 **S01 当前工作区**逐一相同，说明：

- 2026-11-10 的四个 mcap 是用 S01 当前未提交代码录制的；
- S01 的未提交修改不是垃圾，而是**实测过的 M2.2 配置**；
- S03 的 rework1（`61cb…`/`a4f8…`）**没有被这些 bags 验证**。

bags 时间（metadata `starting_time`）：

| bag | 开始时间（本地 +8） | 时长 | bag 消息数 | 分析 CSV 行数 |
|---|---|---|---:|---:|
| `trajectory_test_01` | 2026-11-10 19:31:26 | 46.2s | 9,247 | 302 |
| `trajectory_test_1p5s` | 2026-11-10 19:42:23 | 21.1s | 4,217 | 151 |
| `trajectory_settling_test` | 2026-11-10 19:46:02 | 23.3s | 4,658 | 1,148 |
| `gain_0p2/settling_test` | 2026-11-10 20:59:25 | 133.2s | 26,640 | 未纳入本次 CSV 统计 |

> 四个 bag 均早于 rework1 报告（2026-11-12 UTC），因此 rework1 没有对应的运行时证据。

重算的误差统计（仅用于说明"是什么"，不作为项目误差预算）：

| 数据 | joint | max abs error (mrad) | RMS (mrad) | final (mrad) |
|---|---|---:|---:|---:|
| `trajectory_test_01` | joint1 | 9.094 | 8.773 | 8.170 |
| `trajectory_test_01` | joint2 | 3.031 | 2.924 | 2.723 |
| `1p5s` | joint1 | 18.175 | 17.092 | 18.175 |
| `1p5s` | joint2 | 6.058 | 5.697 | 6.058 |
| `settling` | joint1 | 9.078 | 0.615 | 0.000 |
| `settling` | joint2 | 3.026 | 0.205 | 0.000 |

settling 数据中最后一次 `|error| > 1 mrad`：joint1 在 0.200s，joint2 在 0.100s；末端误差为 0。

**边界**：这是 Gazebo + `mock_components` 的 2-DOF `simple_arm`，不是 ArmPi Ultra，也不是实机；不能当作 `MASTER_PLAN` §9 的 Error Budget v1。

### 4.2 Studio 证据

| 证据 | 结果 |
|---|---|
| 代码 | 28 py + 16 ts/tsx；4,178 + 2,895 行 |
| 后端测试 | 37 个测试函数 + 2 parametrize；README 声称 41；**本次未重跑** |
| HTTP E2E | `data/studio.db` 55 events / 1 command；`data/studio.db-wal` 最后修改 11-14 14:43 |
| 前端构建 | `dist/` 存在；`g1-verification` 声称 `tsc -b && vite build` 成功 |
| Playwright | 无测试文件；仅文档声称 |
| 硬件 | `hardware_motion=false`；无 rclpy import；无串口；未连接机器人 |

### 4.3 ArmPi 仿真运行

`~/.ros/log/move_group_57490_1794635606118.log`：

- `move_group` 11-14 13:53 启动；
- TRAC-IK 加载失败：`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin ... does not exist`；
- `Kinematics solver could not be instantiated for joint group arm`；
- 13:53–13:58 三次 arm 轨迹 `arm_controller started execution` → `Goal reached, success!`；
- 13:55 一次 gripper 规划失败：`No acceleration limit was defined for joint l_in_joint!`。

`ros2_control_node` 日志确认两个控制器激活成功；`joint_state_broadcaster` 正常。模型确实在 RViz/MoveIt 中跑起来了，但**不是 5-DoF 约束规划**，而是关节空间目标。

## 5. 冲突清单与建议保留版本

| 文件/对象 | 冲突方 | 类型 | 建议保留 | 理由 |
|---|---|---|---|---|
| `gazebo.launch.py` | HEAD / S01 / S03 | 三路真实冲突 | **S03 rework1** + 保留 S01 为历史补丁 | S03 可移植、有校验/看门狗；S01 有 runtime 证据但路径硬编码 |
| `simple_arm.xacro` | HEAD / S01 / S03 | 三路真实冲突 | **S03 rework1** + 保留 S01 | 同上 |
| `CMakeLists.txt` / `package.xml` | HEAD vs S03 | 单侧新增 | S03 | 无本地对应修改 |
| `arm_controllers_gain_0p2.yaml` | S01 = S03 | 相同 | 任一 | 哈希相同 |
| `pick_place_demo.cpp` | main vs phase2 vs JER-18 | 同一任务多实现 | **暂不合并**；Board 决定 | phase2 未做运行回归；main 已冻结 tag |
| `armpi_ultra.xacro` | 厂商 A vs B | 上游版本差 | **B 为参考**；S06 的 A 保持可运行 | B 仅 base_link collision 不同；未验证前不替换 |
| `armpi_ultra.urdf` | 陈旧 URDF vs xacro | 内部矛盾 | **xacro 为唯一基线**；URDF 删除/重生成 | URDF 的 joint1/r_joint 是 fixed |
| `joint_limits.yaml` | S06 部分修复 | 不完整 | 补齐 gripper 加速度 + 显式位置限位 | 已导致一次规划失败 |
| `initial_positions.yaml` | MoveIt vs 厂商启动 | 数值冲突 | 以真实状态/实测为准；仿真里标注 provisional | r_joint 1.57 超软件上限且方向不符 |
| `README.md` / `ROADMAP.md` | 旧 8 周文档 vs V3 计划 | 文档冲突 | 保留旧文档为 archive，新增 V3 状态文档 | 旧文档仍声称 WSL2/学习项目 |
| `Projects_M70q` 空骨架 | 与真实仓库 | 假副本 | 清理/隔离 | 0 文件，`.git` 不可用 |
| `S05` 残缺 M2.2 副本 | 与 S03 | 假副本 | 清理 | 25 文件全为 S03 子集，无 `ros2_ws` |

## 6. 可去重/可清理候选（均需批准，本轮未执行）

| 对象 | 建议 | 前提 |
|---|---|---|
| S04 `Projects_M70q/ros2-industrial-arm-learning` | 隔离/删除 | 确认没有任何未来会话依赖该路径 |
| S05 `Projects_M70q/...-m2_2-v2` | 删除 | 已确认 25 文件全部是 S03 子集 |
| S09 `~/robotics_evidence/m2_2` | 保留一份 | 与 S08 完全相同；建议保留 S08 为原件，S09 作异地备份 |
| S10 `robotics-migration-backup` | 归档/删除 | 仅 build/install/log，无源码；建议先打包保留 |
| S11 v0.1.0 tar | 保留 | 唯一 v0.1.0 全仓库快照；可压缩归档 |
| `ArmPiUltra/ROS2/Armpi_ultra-main.zip` | 保留但不要进 Git | 921MB 原始厂商包 |
| `Downloads/simulations.zip` 与 `ArmPiUltra/3 功能包文件/simulations.zip` | 保留一份 | 哈希相同 |

## 7. 结论

1. **64/66 个跟踪文件一致**，主仓库没有大规模分叉；真正的分叉集中在 M2.2 的两个文件、Panda 的未合并分支、以及模型/配置的内部矛盾。
2. **M2.2 有两个"正确版本"**：S01 是被 bags 实测的旧版本；S03 是更工程化但未运行的新版本。二者不能靠时间戳决定。
3. **Panda 的 Phase II 分支是"被遗忘的完成品"**：代码+文档完整，但未合并、未运行回归、不在 baseline tag 内。
4. **Studio G1 Mock 是真实实现**，但文档仍停留在 G0；它只存在于 M70q，未提交。
5. **模型层面最危险的不是缺失，而是陈旧 URDF 与限位冲突**：它们会让 MoveIt 以错误模型规划。
