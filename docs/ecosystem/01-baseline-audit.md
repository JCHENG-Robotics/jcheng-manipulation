# v1.0.0 基线审计

> 审计对象：`/home/jerichocheng/jcheng/jcheng-manipulation`
> 审计提交：`a49cce5`（`v1.0.0` 之后 5 个文档提交）
> 发布标签：`v1.0.0` -> `b3411e4`（2026-11-23）
> 审计方式：只读代码与文档检查；未连接机器人，未发送运动命令。

## 1. 发布快照

| 项 | 值 | 证据 |
|---|---|---|
| 根 `VERSION` | `1.0.0` | `VERSION:1` |
| `v1.0.0` 标签对象 | `6436e2b`（annotated tag） | `git rev-parse v1.0.0` |
| `v1.0.0` 指向的提交 | `b3411e4`，2026-11-23 10:46:54 +0800 | `git rev-parse v1.0.0^{commit}` |
| 标签后提交 | 5 个文档提交（`799544e`…`a49cce5`） | `git log --oneline v1.0.0..HEAD` |
| 清单 `source_commit` | `40a623e`（标签提交的父提交） | `release/manifest.json:76` |
| 冻结基线 | `v0.9.0-ff` -> `30799df`，2026-11-21 | `release/manifest.json:27-31` |
| 组件版本 | `armpi_analysis` 0.1.0；`armpi_fiducial_localizer` 0.1.0；`armpi_grasp_planner` 0.1.0；`armpi_safety` 0.1.0；`armpi_ultra_moveit_config` 0.3.0；`jcheng_core` 0.1.0；Studio backend/frontend/gateway 1.0.0 | `release/manifest.json:15-25` |
| 清单校验和 | 8 个 SHA-256；本次抽检 `VERSION`、`safety_limits.yaml`、`joint_mapping.yaml`、`takeover.yaml`、`requirements/ci.txt`、`jcheng_core/pyproject.toml`、`ros-ci.yml` 一致 | `release/manifest.json:2-13` |
| 验证计数 | analysis 34；backend 148；gateway 93；safety/planner/fiducial 59；tools 242 passed / 7 skipped | `release/manifest.json:78-86` |
| 构建证据 | 7 包、118 tests、0 fail、1 skipped（xmllint schema 拉取） | `evidence/ros_colcon_20261123/` |
| 已知限制 | 无 e-stop/brake/自碰撞/独立编码器/里程计；fast CLI 设计上绕过 safety；真实 safety 节点是手动进程；8766 单元默认禁用；后端默认 localhost 无 token | `release/manifest.json:47-53`、`RELEASE.md:113-131` |

**注意：** `release/manifest.json` 的 `source_commit` 指向 `40a623e`，而发布标签在
`b3411e4`。这是“清单不能记录自身提交”的常见做法；生态锁文件必须同时记录
**标签提交**与**清单内容哈希**，不能只记录 `source_commit`。

## 2. ROS 2 包清单

`ros2_ws/src` 下共 10 个包；CI 只构建其中 7 个，另外 3 个是归档学习包。

| 包 | 版本 | 构建 | 角色 | 关键接口 / 节点 | CI |
|---|---|---|---|---|---|
| `arm_interfaces` | 0.0.0 | ament_cmake | 自定义 action/srv | `ArmMove.action`、`ValidateTrajectory.srv` | 是 |
| `armpi_analysis` | 0.1.0 | ament_python | 误差预算、配对试验、主机基准（纯 Python） | `error_budget`、`paired_trials`、`host_benchmark` | 是 |
| `armpi_fiducial_localizer` | 0.1.0 | ament_python | AprilTag/ArUco 定位 | `fiducial_localizer`、`camera_frame_publisher`、`synthetic_marker_publisher` | 是 |
| `armpi_grasp_planner` | 0.1.0 | ament_python | 5-DoF 候选生成、IK/FK/碰撞筛选、目标桥 | `grasp_candidate_check`、`grasp_target_bridge` | 是 |
| `armpi_safety` | 0.1.0 | ament_python | 轨迹否决与状态看门狗 | `safety_supervisor` | 是 |
| `armpi_ultra_description` | 0.0.0 | ament_python | 5-DoF xacro、夹爪、相机坐标系 | 无节点 | 是 |
| `armpi_ultra_moveit_config` | 0.3.0 | ament_cmake | MoveIt 配置、规划/执行 launch | `move_group`、`real_moveit` | 是 |
| `arm_basics` | 0.0.0 | ament_cmake | 8 周学习包 | `arm_move_server/client`、TF demo | 否 |
| `arm_description` | 0.0.1 | ament_cmake | M2.2 双自由度归档 | Gazebo/display | 否 |
| `arm_moveit_demo` | 0.0.0 | ament_cmake | Panda 归档演示 | `pick_place_demo` | 否 |

关键依赖链：

```text
armpi_safety
  └── armpi_grasp_planner.reachability
        └── armpi_ultra_description (xacro)
              └── PyKDL / urdf_parser_py / xacro
```

`armpi_safety` 的 FK 工作空间检查失败时**拒绝所有轨迹**（`workspace.required: true`，
`policy.py:526-546`）。这使安全节点隐式依赖规划包与描述包，是拆分时必须解耦的
耦合点。

## 3. `arm_interfaces` 契约

### 3.1 `ArmMove.action`（归档，不进入生产）

| 段 | 字段 | 类型 |
|---|---|---|
| Goal | `target_angle` | `float64` |
| Result | `success` / `final_angle` / `message` | `bool` / `float64` / `string` |
| Feedback | `current_angle` | `float64` |

- 唯一 server/client 在 `arm_basics`（`arm_move_server.cpp:18-23`、
  `arm_move_client.cpp:14`），是纯软件仿真，不发布硬件命令。
- 代码按“度”判断 `abs(angle) > 180`（`arm_move_server.cpp:39`），action 文件
  未声明单位。
- 结论：**历史学习产物，不迁移。**

### 3.2 `ValidateTrajectory.srv`（生产安全接口）

| 段 | 字段 | 类型 |
|---|---|---|
| Request | `trajectory` | `moveit_msgs/RobotTrajectory` |
| Request | `max_start_state_distance_rad` | `float64` |
| Response | `approved` | `bool` |
| Response | `code` | `string` |
| Response | `violations` | `string[]` |
| Response | `report_json` | `string` |

- 唯一 server：`armpi_safety`，服务名 `/armpi_safety/validate_trajectory`
  （`node.py:96-100`）。
- 调用方：`armpi_grasp_planner`（可选）、`real_moveit_execute.py`（默认强制）、
  `execute_plan_json.py`（强制）。
- 结论：语义保留，载荷在新契约中改为 `JointTrajectory`，由适配器转换
  `moveit_msgs`。

## 4. Studio、网关与 API

### 4.1 结构

```text
armpi-debug-studio/
├── backend/            FastAPI；148 tests；租约/安全/预览 token/事件存储
├── ros2_gateway/       rclpy 网关；93 tests；8765 只读 / 8766 pose-motion
├── frontend/           React + TS + Vite；Playwright 10 mock + 3 token
├── configs/            joint_mapping.yaml、takeover.yaml
└── tools/
```

### 4.2 后端 API（节选）

| 方法 | 路径 | 作用 |
|---|---|---|
| GET | `/api/health`、`/api/state`、`/api/connection` | 健康、状态、连接证据 |
| GET | `/api/peripherals`、`/api/peripherals/*/snapshot` | 电池、主相机、D435i 辅助画面 |
| GET | `/api/safety`、`/api/diagnostics` | 安全检查、诊断 |
| POST | `/api/control/{acquire,heartbeat,release}` | 单一控制租约 |
| POST | `/api/motion/{preview,execute,stop}` | 预览 token、执行、软件停止 |
| GET | `/api/events`、`/api/events/export.csv` | 只追加审计事件 |
| GET/POST | `/api/trajectories*` | G2 离线轨迹 |
| GET/POST | `/api/experiments*` | G3 配对试验 |
| GET/POST | `/api/visual*` | G4 只读可视化 |
| WS | `/api/ws/{state,events}` | 10 Hz 状态、事件流 |

后端的安全不变量（`armpi-debug-studio/README.md:484-497`）：

- 后端不打开串口、不导入 `rclpy`。
- 只读网关只订阅，不发布运动。
- 硬件模式默认 `hardware_motion=false`。
- 执行需要租约 + 单次预览 token + 全部安全检查。
- 状态机不产生 `Acknowledged`；`Dispatched -> Completed` 非法。
- 事件只追加；存储失败锁存并禁止运动。
- UI 只写“停止请求”，不写“急停”。

### 4.3 网关

- 两个实例：8765 只读；8766 pose/motion，安装但默认禁用
  （`RELEASE.md:160-166`）。
- 运动发布者：`servo_controller`（`node.py:82,142-144`），通过 `MotionGuard`
  和 `MotionSafety` 校验。
- 底盘：`/ros_robot_controller/set_motor_speeds`（`chassis.py:26`），
  `/chassis/pulse` 限制 duty ≤10、时长 ≤2 s。
- 实测：`GetBusServoState`（`/ros_robot_controller/bus_servo/get_state`），
  总线读数用于完成判定；`/joint_states` 与 `/controller_manager/servo_states`
  是命令缓存。
- `/safety_stop` 自 2026-11-22 起被网关消费（`node.py:209-214,294-316`），
  上升沿锁存 `MotionGuard` 并发布保持命令。
- RMW：网关默认 Fast DDS；厂商节点 CycloneDDS。**话题可互通，服务不可互通**；
  任何服务调用必须使用厂商 `.zshrc` 环境。

## 5. 实机运动写入路径（官方清单）

`docs/phase2/MOTION_WRITE_PATHS.md` 是仓库内的权威清单，共六条：

| # | 路径 | 入口 | 守卫 | 限幅 / 验证 |
|---|---|---|---|---|
| 1 | MoveIt CLI | `tools/phase2/real_moveit_execute.py`、`execute_plan_json.py` | `/armpi_safety/validate_trajectory`（必需） | 1.2 rad/s、0.5 rad/s²、起点、工作空间、桌面、时长/点数 |
| 2 | Fast CLI | `tools/phase2/fast_motion.py`、`tools/phase3/fast_motion_e2e.py` | **无 `armpi_safety`**；仅 S 曲线、joint2 边界、总线稳定判据 | 4.0 rad/s、20 rad/s²；总线 settle |
| 3 | Studio G6 step | 后端 → 网关 `/motion/dispatch` | 后端租约/预览 token；网关 motion policy（单关节步长、时长、总线 ±8 脉冲） | 限幅来自 `joint_mapping.yaml` |
| 4 | Studio G7 pose | 后端 → 网关 pose scope | 后端 pose gate（FK 工作空间/桌面/低姿态 yaw）；网关只做关节/step/速度/总线确认 | `takeover.yaml` 0.5 rad/s；仅 TCP，无自碰撞 |
| 5 | Studio G8 task/fast | 后端 `/api/task/plan` + `/api/motion/execute` | 网关 profile policy：峰值速度/加速度、段长、点数、总时长、起点容差、总线确认、底盘上限 | `task.fast` 4.0/20；`max_points 400`；`max_total_duration_s 6.0` |
| 6 | Chassis pulse | 网关 `/chassis/pulse` | 网关 chassis policy：duty、轮速、时长、有限值、方向符号 | duty ≤10、≤2.0 s；开环、无里程计 |

**第七个写面：** `docs/phase2/MOTION_WRITE_PATHS.md:26` 之外还有脚本直写
`/ros_robot_controller/bus_servo/set_position`（`pulse_move.py`、`servo_hold.py`、
`joint_limit_survey.py`、`torque_probe.py` 等）。这些是研究/标定工具，不是
生产路径，但同样能移动机械臂；新生态必须把它们归入“受监督工具”，不能默认可运行。

## 6. 安全机制与缺口

### 6.1 已实现

| 机制 | 位置 | 覆盖 |
|---|---|---|
| 状态新鲜度 0.5 s、缺关节、非有限、越界 | `armpi_safety/policy.py:236-317` | MoveIt CLI |
| 位置限位 | `safety_limits.yaml:38-69` | MoveIt CLI |
| 速度/加速度上限（1.2/0.5） | `policy.py:640-696` | MoveIt CLI |
| 起点一致性 0.08 rad | `policy.py` | MoveIt CLI |
| 时长/点数（30 s / 5000） | `safety_limits.yaml:79-84` | MoveIt CLI |
| 工作空间包围盒 | `safety_limits.yaml:86-96` | MoveIt CLI |
| 桌面平面间隙 10 mm | `safety_limits.yaml:98-111` | MoveIt CLI |
| 停止锁存 `STOP_LATCHED` | `policy.py:297-344` | MoveIt CLI + 网关 |
| `/safety_stop` 瞬态本地 Bool | `node.py:80-86` | 仅 Studio 网关消费 |
| 网关总线完成判定 | `gateway/node.py:830-930` | G6/G7/G8 |
| 网关 `/safety_stop` 锁存 | `gateway/node.py:294-316` | G6/G7/G8、底盘 |

### 6.2 关键缺口（G1–G16）

| ID | 缺口 | 证据 | 影响 |
|---|---|---|---|
| G1 | DDS 无认证，任意节点可发布运动话题 | `docs/hardware/network_access.md:19-28`、`tools/net/armpi-lockdown.nft` | 绕过全部 HTTP 鉴权与租约 |
| G2 | 网关在安全节点缺失时 **fail-open** | `gateway/node.py:134,209-214` 初值 false，无“等待安全节点”联锁 | 不启动 safety 也能运动 |
| G3 | 网关限位完全来自 `--motion-config` 文件 | `gateway/main.py:78-83,131-142`；`motion.py:363-392` 只做结构校验 | 配置文件可放宽；pose 的 `max_step_rad=3.61` 是全行程跨度上限，不是小步限幅 |
| G4 | 后端不知道 `/safety_stop` 与真实 `stop_state` | `hardware.py:765` 固定 `None`；`supervisor.py:302-312` 检查恒通过 | UI“安全就绪”不反映网关锁存 |
| G5 | `/safety_stop` 不中止在飞 dense profile/底盘脉冲 | `node.py:714-721` 只检查 state/command_id；`node.py:784-803` 循环中不再检查 | 停止只阻止新命令 |
| G6 | `ValidateTrajectory` 可被调用方放宽起点；速率可伪造 | `policy.py:491-496` 覆盖 0.08；`policy.py:698-750` 显式速率优先 | 校验可被绕过 |
| G7 | 安全配置缺关节时回退 ±2.09 rad；无机器可读 `mapping_version` | `policy.py:20-29,79-86`；`safety_limits.yaml:3,12-13` | 配置漂移/放宽 |
| G8 | `SetBusServoState` 可改 ID/offset/角度限位/力矩/EEPROM | `ros_robot_controller_node.py:178-215`；SDK `:471-501` | 安全模型前提可被持久修改 |
| G9 | MoveIt action 直接暴露；驱动只钳 0..1000 pulse | `joint_trajectory_action_controller.py:146-156`；`joint_position_controller.py:43-48` | 可命令 joint2 到 1000（实测高 stop 795） |
| G10 | Fast CLI 无 `--vmax` 硬上限且绕过 safety | `fast_motion.py:462-467`；`RELEASE.md:113-116` | 4.0/20 可被放大 |
| G11 | `/grasp`、手柄直发 `/servo_controller` | `grasp.py:94-152`；`joystick_control.py:155-309` | 无守卫运动 |
| G12 | safety 读的 `/joint_states` 是命令缓存重发 | `robot_description.launch.py:35-44`（joint_state_publisher 重发）；`controller_manager.py:97-116` | 看门狗看不到真实总线停摆 |
| G13 | dispatch 超时不锁存 `UNKNOWN` | `hardware.py:1269-1286` 仅轮询路径锁存 | 可能重复下发 |
| G14 | stop 不需要租约，且只是 hold | `motion_ops.py:445-452`；`node.py:932-971` | 任何已认证操作员都能停；不是急停 |
| G15 | 厂商容器重启 `startup_check` 自动运动 | `armpi-debug-studio/README.md:620-624` | 软件层无法否决 |
| G16 | FK/桌面检查只针对 TCP 单点 | `policy.py:605-633` | 连杆/夹爪碰撞不在模型 |

### 6.3 结论：安全边界是分层的，不是统一的

v1.0.0 的安全是“每个入口各自实现一套”：

- MoveIt CLI：`armpi_safety` 否决 + 工具侧强制调用。
- Studio G6/G7/G8：后端 gate + 网关 `MotionGuard` + 总线确认。
- Fast CLI：仅自身 S 曲线、joint2 边界与总线 settle。
- 底盘：网关 chassis policy；与机械臂无交叉互锁。
- 脚本/手柄/`/grasp`：无统一守卫。

这解释了为什么“新增一个模块”不能简单复用现有安全代码：复用任何一条路径都会
继承该路径的缺口。新系统必须重建一个**统一入口 + 统一校验 + 统一停止**，
而不是把六套策略拼在一起。

### 6.4 关键运行事实

1. **`armpi_safety` 运行时不读取 `profiles.step/pose/fast`。**
   `SafetyConfig.from_dict` 只使用 `profiles.moveit`（`policy.py:68-75`）；
   四个命名 profile 是文档/一致性权威，不是单一运行时权威。
2. **`/safety_stop` 只有 Studio 网关消费。**
   Fast CLI、`pulse_move.py`、`servo_hold.py`、raw-bus 脚本都不订阅
   （`MOTION_WRITE_PATHS.md:57-64,79-84`）。
3. **Fast CLI 设计上绕过 `armpi_safety`。**
   4.0/20 的轨迹如果走 safety 会被 1.2/0.5 拒绝；它直接发
   `/arm_controller/follow_joint_trajectory`。
4. **`/joint_states` 与 `/controller_manager/joint_states` 并存，且都是缓存。**
   `joint_state_publisher` 以 20 Hz 重发 `controller_manager/joint_states`；
   safety 的超时判据因此不能证明总线仍在响应。
5. **完成判定依赖总线读数，但状态流仍可能被误当实测。**
   只有 `GetBusServoState` 是实测；新契约必须强制 `truth` 与 `verification`。
6. **无急停、抱闸、独立编码器、自碰撞模型。**
   软件停止是保持/置零，不是安全停。
7. **厂商容器重启会移动机械臂。**
   `startup_check` 触发初始化运动；软件层无法否决。
8. **真实 safety 节点是手动进程，不是开机服务。**
   `RELEASE.md:150-153`、`docs/operations/RUNBOOK.md:47-59`。

## 7. 依赖与运行环境

### 7.1 软件栈

| 层 | 版本/约束 | 证据 |
|---|---|---|
| 机器人 ROS | Humble | `ros-ci.yml:30` |
| 工作站 ROS | Jazzy | `tools/phase4/run_benchmark.sh:30` 等 |
| RMW | CycloneDDS（厂商/实机）；网关默认 Fast DDS | `run_humble.sh:30-32`；网关 README |
| Python | 3.10–3.12；backend `<3.13`；CI 3.12 | `jcheng_core/pyproject.toml:9`；`backend/pyproject.toml` |
| 核心 Python 依赖 | FastAPI 0.163.0、uvicorn 0.54.0、pydantic 2.14.3、PyYAML 6.0.3、numpy 1.26.4 | `backend/pyproject.toml` |
| 视觉/规划 | OpenCV、NumPy、PyKDL、urdf_parser_py、xacro、MoveIt/Pilz | `package.xml`、CI apt 列表 |
| 厂商消息 | `servo_controller_msgs`、`ros_robot_controller_msgs` | 网关/工具导入 |
| 基础镜像 | `ros:humble-ros-base@sha256:dfc94c85…` | `ros-ci.yml:30` |

### 7.2 私有与外部依赖

- `armpi_ultra_ws_clean/` 是 gitignored 的厂商工作区（约 29 MB）；
  `tools/vendor_baseline_manifest.py` 用 SHA-256 锁定，不发布。
- 厂商网格资产不在 MIT 下再许可（`THIRD_PARTY_NOTICES.md:3-20`）。
- 外部系统：Aurora 930、RealSense D435i（UVC + 深度）、Pi 5、ArmPi 控制板。

### 7.3 硬编码布局（拆分阻塞项）

| 类型 | 示例 | 位置 |
|---|---|---|
| 绝对路径 | `/home/pi/docker/tmp/armpi_gateway_deploy/token` | `tools/studio/start_studio.sh:32` |
| 绝对路径 | `/home/ubuntu/shared/safety_ws/install/setup.zsh` | `docs/operations/RUNBOOK.md:53-54` |
| 相对跨仓库路径 | backend 通过 `../../ros2_ws/src/armpi_ultra_description/...` 找 URDF | `backend/app/config.py:14-25` |
| 主机/IP | `10.42.0.218`、`10.42.0.1`、网卡名 | `tools/studio/start_studio.sh:26`、`cyclonedds-laptop.xml:3` |
| 端口 | 8765、8766、8000 | `gateway.env.example:10`、`pose.env.example:5`、`backend/app/config.py:150` |
| 布局假设 | `PYTHONPATH=packages/jcheng_core:ros2_ws/src/...` | `scripts/release_check.sh:28,40` |

这些不是简单的“路径替换”：它们说明当前代码假设**单一工作区布局**。新仓库必须
显式引入 `robot profile`、配置注入与包安装，而不是复制相对路径。

## 8. 配置重复与漂移面

同一物理常量至少出现在：

| 常量 | 位置 |
|---|---|
| 关节限位 | xacro、`armpi_ultra_moveit_config/config/joint_limits.yaml`、`safety_limits.yaml`、`joint_mapping.yaml`、`takeover.yaml`、`vendor_servo_map.py` |
| 伺服 ID | `vendor_servo_map.py`、`bus_pulses.py`、网关 `motion.py`、`trajectory_capture.py`、后端 `takeover.py`/`task.py`、`chessboard_capture.py` |
| 低构型修正常量 | `low_config_model_fit.py`、`execution_fixture_run.py`、`fast_motion_e2e.py`、`gripper_envelope_run.py`、`paired_trial_session.py` |
| 视角/快速运动默认值 | `tools/phase3/*` 多个运行器 |

`tools/verify_config_consistency.py` 能检查已知文件，但它本身也硬编码了文件列表
（`verify_config_consistency.py:29-38`）。新增路径不会被自动覆盖。
这正是新系统要改为“单一 robot profile + 生成 + 哈希”的原因。

## 9. 拆分可行性判断

### 9.1 可以较干净地进入新仓库

| 资产 | 原因 |
|---|---|
| `arm_interfaces` 的 `ValidateTrajectory` 语义 | 纯接口，可重建为 `ValidateMotion` |
| `armpi_safety/policy.py` 的检查逻辑 | 纯 Python，无 ROS 依赖，适合移植 + 扩展 |
| 网关的 `MotionGuard`/总线确认逻辑 | 已有明确的安全与验证语义 |
| Studio 的 API/状态机/事件存储 | 边界清晰，但需把租约/安全权威上移 |
| 纯 Python 分析/统计 | 无硬件依赖 |

### 9.2 不能直接迁移或必须重写

| 资产 | 原因 |
|---|---|
| `release/manifest.json`、证据目录 | 属于冻结基线；迁移会破坏可复现性 |
| 厂商工作区 | 私有、不可发布、哈希锁定 |
| 硬编码绝对路径的工具 | 依赖单一主机布局 |
| Fast CLI | 绕过安全层；必须重写为受控 profile |
| 双网关 8765/8766 | 端口/进程模型需重新设计 |
| 跨仓库相对路径 | 拆分后失效 |

## 10. 审计结论

1. **基线是可信的、可复现的、有证据的。** 校验和抽检一致，测试计数与清单一致。
2. **但它不是“单一安全权威”。** 六条写入路径、四套策略、一个设计上的绕过。
3. **`armpi_safety` 的覆盖范围被高估。** 它只否决 MoveIt CLI；step/pose/fast
   的限幅在网关和文档里，不在安全节点里。
4. **`/safety_stop` 不是全系统停止。** 只有 Studio 网关消费。
5. **实测与缓存混用是系统性风险。** 新契约必须把 `truth` 与 `verification`
   作为一等字段。
6. **配置重复是安全缺陷来源。** 单一 robot profile 是必要修复。
7. **拆分的主要成本在布局假设，不在 ROS 代码本身。** 接口与策略可移植；
   路径、配置、发布清单必须重建。
8. **冻结基线应保持只读。** 新生态通过契约和适配器复用语义，不复制仓库。

## 11. 需要确认的审计结论

1. 是否认可“六条写入路径 + 脚本直写”是当前完整清单？
2. 是否认可“`armpi_safety` 只覆盖 MoveIt CLI”这一事实认定？
3. 是否认可把 Fast CLI 视为“必须重写的受控 profile”，而不是可复用组件？
4. 是否认可基线仓库不迁移、只做参考与语义复用？
