# 统一接口契约

> 契约仓库：`jcheng-interfaces`
> 原则：**契约里不提供任何可以直接驱动硬件的原语。**
> 能力模块能表达“意图”，不能表达“寄存器写入”。

## 1. 契约分层

| 层 | 载体 | 消费者 | 版本策略 |
|---|---|---|---|
| ROS IDL | `jcheng_interfaces` 包 | ROS 节点、网关、测试工具 | 契约哈希 + SemVer |
| HTTP/OpenAPI | `schemas/openapi/jcheng-gateway.v1.yaml` | 应用、前端、外部工具 | URL 主版本 `/v1` |
| WebSocket | `schemas/ws/jcheng-ws.v1.json` | 实时 UI | 协议版本字段 |
| Python | `jcheng_contracts` | 网关、工具、测试 | 跟随契约版本 |
| TypeScript | `typescript/src`（生成） | 前端 | 跟随 OpenAPI |
| 配置 | `robot-profile`、`safety-profile`、`capability-manifest` | 系统、部署、CI | Schema 版本 + profile 版本 |
| 证据 | `evidence-record`、`acceptance-report` | 验收、发布 | Schema 版本 |

所有层共享同一个 `contract_hash`。任何一层改变结构，哈希必须改变。

## 2. 设计规则

1. **单一运动入口。** 实机运动只能通过 `jcheng_interfaces/action/ExecuteMotion`。
2. **SI 单位。** 关节用 `rad`、`rad/s`、`rad/s²`；直线用 `m`；时间用 `s`。
   伺服原始脉冲只存在于机器人配置与硬件适配器，不进入契约。
3. **显式坐标系。** 位姿必须声明 `frame_id`；默认 `base_link`。TCP 语义由
   `tip_link` 字段显式给出，不能靠隐式约定。
4. **显式真值。** 状态必须标注 `truth=measured|commanded`；完成必须标注
   `verification=bus|timed|open_loop`。命令缓存不得冒充实测。
5. **显式版本。** 每个租约申请与运动命令都携带 `ContractVersion`。
6. **幂等。** 每个运动命令有 `idempotency_key`；重放返回同一结果，不重复运动。
7. **失败关闭。** 缺字段、未知枚举、版本不匹配、状态过期一律拒绝，不做默认补全。
8. **不可表达危险原语。** 契约没有“写任意舵机寄存器”“设置任意电机占空比”
   “直接发布厂商话题”的消息类型。
9. **审计可重建。** 每个结果带 `evidence_id`，可关联到输入、校验报告和实测读数。
10. **时间诚实。** 控制决策使用本机单调时钟；跨主机时间戳只用于记录，
    不用于小于 10 ms 的先后判断。

## 3. ROS IDL 目录

```text
jcheng_interfaces/
├── msg/
│   ├── ContractVersion.msg
│   ├── CapabilityManifest.msg
│   ├── SafetyState.msg
│   ├── ControlLease.msg
│   ├── MotionCommand.msg
│   ├── MotionFeedback.msg
│   ├── MotionResult.msg
│   ├── RobotState.msg
│   ├── JointTrajectory.msg
│   ├── JointTrajectoryPoint.msg
│   ├── BasePulse.msg
│   ├── GripperCommand.msg
│   └── TeleopCommand.msg
├── srv/
│   ├── AcquireLease.srv
│   ├── RenewLease.srv
│   ├── ReleaseLease.srv
│   ├── RevokeLease.srv
│   ├── ValidateMotion.srv
│   ├── GetSafetyState.srv
│   ├── ResetSafety.srv
│   ├── ArmSystem.srv
│   ├── DisarmSystem.srv
│   └── GetCapabilities.srv
└── action/
    └── ExecuteMotion.action
```

## 4. 核心消息

### 4.1 `ContractVersion.msg`

```text
uint16 major
uint16 minor
uint16 patch
string contract_hash
```

`contract_hash` 是 `sha256:` 前缀的十六进制字符串。仲裁器要求**精确匹配**；
`major`/`minor` 用于人类可读的错误说明，不作为放宽依据。

### 4.2 `MotionCommand.msg`

```text
std_msgs/Header header

string intent_id
string idempotency_key
string lease_id
string source                 # 模块名，必须与 CapabilityManifest.name 一致
string kind                   # 见下表
string profile                # moveit|step|pose|fast|base|gripper
ContractVersion contract

# 关节轨迹载荷（kind=JOINT_TRAJECTORY）
JointTrajectory trajectory

# 单步/目标关节载荷（kind=JOINT_STEP|HOME）
string[] joint_names
float64[] target_positions
float64 max_velocity_rad_s
float64 max_acceleration_rad_s2
float64 duration_s

# 位姿载荷（kind=POSE）
geometry_msgs/Pose target_pose
string tip_link
string frame_id

# 底盘载荷（kind=BASE_PULSE）
BasePulse base_pulse

# 夹爪载荷（kind=GRIPPER）
GripperCommand gripper

# 执行与验证要求
bool verify_bus              # 默认 true；false 只在允许的 profile 中使用
bool require_measured_state  # 默认 true
string expected_profile_hash
```

`kind` 与载荷的合法组合：

| kind | 必填载荷 | 允许 profile | 说明 |
|---|---|---|---|
| `JOINT_TRAJECTORY` | `trajectory` | `moveit`、`fast` | 密集轨迹；每段或整条校验 |
| `JOINT_STEP` | 关节目标、速度/加速度、时长 | `step`、`pose` | 单步或小步序列 |
| `POSE` | `target_pose`、`tip_link`、`frame_id` | `pose` | 由系统做 IK，不接受客户端 IK 结果 |
| `HOME` | 无 | `pose`、`step` | 仅允许配置中定义的 home 位姿 |
| `BASE_PULSE` | `base_pulse` | `base` | 开环；必须绑定标定失效规则 |
| `GRIPPER` | `gripper` | `gripper` | 弧度，不接受原始寄存器 |

**拒绝规则：** 载荷与 `kind` 不匹配、出现多余载荷、`profile` 与 `kind` 不匹配、
`source` 与已登记能力不符，均返回 `INVALID_COMMAND` 并记录审计事件。

### 4.3 `JointTrajectory.msg` / `JointTrajectoryPoint.msg`

```text
std_msgs/Header header
string[] joint_names
JointTrajectoryPoint[] points
```

```text
float64[] positions
float64[] velocities
float64[] accelerations
builtin_interfaces/Duration time_from_start
```

规则：

- `joint_names` 必须按机器人配置的规范顺序（当前为 `joint1..joint5`），
  或显式重排；重复/缺失/未知关节拒绝。
- `positions`、`velocities`、`accelerations` 长度必须等于关节数。
- `time_from_start` 严格递增；首点时间必须为 0。
- 空轨迹、单点轨迹、时间回退、NaN/Inf 一律拒绝。
- 点数与时长上限来自安全 profile（当前基线：5000 点 / 30 s）。

### 4.4 `BasePulse.msg`

```text
float64[] duty                 # 4 通道，-1.0..1.0
float64 duration_s             # 0.05..2.0
bool require_stopped_after     # 默认 true
```

规则：

- 长度必须等于底盘通道数（当前 4）。
- `duty` 是归一化值；绝对值不得超过 profile 的 `max_abs_duty_normalized`。
  当前基线 G8.1 的厂商原始 duty 上限为 10（满量程 100），对应归一化 `0.10`；
  硬件适配器负责 `归一化 <-> 厂商原始 duty` 的换算。
- `duration_s` 不得超过 profile 上限（当前 2.0 s）。
- 任何底盘运动必须使桌面标定失效；仲裁器在结果中返回
  `calibration_invalidated=true`，后续抓取任务必须重新定位。

### 4.5 `GripperCommand.msg`

```text
float64 position_rad           # 0.0=闭合，2.22=全开（以 profile 为准）
float64 max_velocity_rad_s
float64 duration_s
bool verify_bus
```

规则：

- 契约只表达弧度；原始寄存器（170/700 等）由硬件适配器按 robot profile 映射。
- **符号约定：** 契约使用 `0.0=闭合、2.22=全开` 的正向行程。基线 URDF 的
  `r_joint` 使用 `-2.22..0.0`，两者符号相反；适配器负责转换，契约与 UI 不暴露
  URDF 负号。
- 必须有总线读数确认；仅按时间完成不算成功。
- 夹持判定需要 `held` 证据（堵转寄存器范围）由硬件适配器返回，不由客户端声明。

### 4.6 `ControlLease.msg`

```text
string lease_id
string holder                 # 操作员/模块稳定标识
string scope                  # arm|gripper|base|full
string mode                   # exclusive
uint8 priority
bool deadman_required
builtin_interfaces/Time issued_at
builtin_interfaces/Time expires_at
ContractVersion contract
```

优先级建议（可在 profile 调整）：

| 优先级 | 主体 | 说明 |
|---|---|---|
| 100 | 现场操作员 | 可抢占应用与自动模块 |
| 60 | 应用任务 | 可抢占遥操作/学习 |
| 40 | 遥操作 | 人机直连，高于自动学习 |
| 20 | 学习/自动策略 | 最低，可被随时抢占 |
| 10 | 诊断/标定 | 默认不授予，需单独审批 |

`scope=full` 等价于同时占用 `arm`、`gripper`、`base`，用于整机任务；
`arm` 与 `gripper` 可以分开授予，但同一执行器永远只有一个独占租约。

### 4.7 `SafetyState.msg`

```text
std_msgs/Header header
uint32 seq
string state                  # DISARMED|ARMED_IDLE|MOTION_ACTIVE|STOP_LATCHED|FAULT
bool stop_latched
string[] violation_codes
string[] violations
string config_hash
string profile_hash
builtin_interfaces/Time last_valid_state_at
```

规则：

- `stop_latched=true` 时仲裁器拒绝一切新的运动命令。
- `last_valid_state_at` 超过 state timeout（当前 0.5 s）即进入 `STOP_LATCHED`。
- `config_hash`/`profile_hash` 与锁文件不一致时，安全节点拒绝进入 `ARMED`。
- 状态机与复位条件见 `04-system-architecture.md`。

### 4.8 `MotionResult.msg`

```text
string intent_id
string outcome                # COMPLETED|REJECTED|ABORTED|TIMEOUT|UNKNOWN
string reason_code
string message
SafetyState safety
float64[] measured_positions
bool bus_verified
bool calibration_invalidated
builtin_interfaces/Time completed_at
string evidence_id
```

`outcome` 语义：

- `COMPLETED`：只有总线读数确认（或明确允许的 `open_loop` 且满足其条件）才可返回。
- `REJECTED`：校验/租约/版本/权限失败；没有发出任何运动命令。
- `ABORTED`：执行中出现停止、租约撤销或安全违规；已尽力停止。
- `TIMEOUT`：执行超时；必须锁存，不能当作成功。
- `UNKNOWN`：命令可能已发出但无法确认；必须锁存 `MOTION_OUTCOME_UNKNOWN`，
  需要总线复核与人工复位。

### 4.9 `TeleopCommand.msg`（预留，遥操作流）

```text
std_msgs/Header header
uint64 seq
string lease_id
string source
ContractVersion contract
string mode                    # JOINT_VELOCITY|CARTESIAN_VELOCITY
float64[] joint_velocity_rad_s
geometry_msgs/Twist cartesian_velocity
bool deadman
```

规则：

- 只在持有 `motion_source` 租约时接受；`scope` 必须覆盖命令轴。
- `seq` 严格递增；`header.stamp` 在 200 ms 窗口内；命令间隔超过
  `deadman_ms` 立即停止。
- 仲裁器按固定周期（建议 20–50 Hz）采样并转换为内部 `JOINT_STEP`/`POSE`
  意图，仍走同一安全校验；**不直接发布硬件命令**。
- 速度上限由 profile 裁剪；超限返回 `VELOCITY_LIMIT` 并计数，不静默截断。
- v0.1 先定义契约，首个 `jcheng-teleop` 试点时启用；启用不改变其他消息。

## 5. 服务

### 5.1 租约

```text
# AcquireLease.srv
string holder
string scope
string mode
uint8 priority
uint32 ttl_ms
bool deadman_required
ContractVersion contract
---
bool granted
ControlLease lease
string reason_code
string message
```

`RenewLease.srv`、`ReleaseLease.srv`、`RevokeLease.srv` 均以 `lease_id` 为主键。
`RevokeLease` 只允许操作员/系统管理员调用；撤销后仲裁器必须在下一次控制周期停止。

### 5.2 校验

```text
# ValidateMotion.srv
MotionCommand command
---
bool approved
string reason_code
string[] violations
string normalized_profile
string validation_id
builtin_interfaces/Time expires_at
```

`ValidateMotion` 是**纯校验**：不占用执行器、不发布命令、不产生运动。它给 UI 做预览，
也给执行前的二次校验复用。`validation_id` 与命令哈希绑定，单次使用，默认 30 s 过期。

### 5.3 安全与使能

```text
# ArmSystem.srv
string operator
string checklist_hash
string confirm_phrase
ContractVersion contract
string profile_hash
---
bool armed
SafetyState safety
string message

# DisarmSystem.srv
string operator
string reason
---
bool disarmed
SafetyState safety

# ResetSafety.srv
string operator
string reason
string confirm_phrase
ContractVersion contract
---
bool accepted
SafetyState safety
string message
```

使能（arming）不是“上电”，而是允许系统接受运动命令。默认 `DISARMED`；
每次进程重启、配置变化、停止锁存、总线未知都回到 `DISARMED` 或 `STOP_LATCHED`。

### 5.4 能力与状态

```text
# GetCapabilities.srv
---
CapabilityManifest[] capabilities
ContractVersion[] supported_contracts

# GetSafetyState.srv
---
SafetyState state
```

## 6. Action

```text
# ExecuteMotion.action
MotionCommand command
---
MotionResult result
---
MotionFeedback feedback
```

`MotionFeedback.msg`：

```text
string intent_id
string phase                  # VALIDATING|QUEUED|EXECUTING|SETTLING|VERIFYING
float32 progress
float64[] measured_positions
bool bus_verified
builtin_interfaces/Time stamp
```

执行语义：

1. 客户端持有有效租约，提交 goal。
2. 仲裁器做契约/租约/能力/幂等检查。
3. 安全节点做状态/限位/轨迹/工作空间校验。
4. 通过后由 `jcheng_motion` 调用允许的执行后端。
5. 执行中持续反馈；停止、撤租、超时立即中止。
6. 完成判定必须满足 `verify_bus` 与 profile 要求。
7. 结果与证据写入审计存储；重复 `idempotency_key` 返回缓存结果。

取消语义：action cancel 触发“尽力停止”，结果必须是 `ABORTED` 或 `UNKNOWN`，
不得返回 `COMPLETED`。

## 7. HTTP / WebSocket 契约

### 7.1 HTTP v1

所有路径以 `/v1` 开头；请求/响应 `Content-Type: application/json`；
错误统一为：

```json
{
  "error": {
    "code": "SAFETY_STOP_LATCHED",
    "message": "安全停止已锁存",
    "details": {},
    "request_id": "..."
  }
}
```

| 方法 | 路径 | 作用 | 认证 |
|---|---|---|---|
| GET | `/v1/health` | 存活/版本/能力摘要 | 可选 |
| GET | `/v1/contract` | 契约版本与哈希 | 是 |
| GET | `/v1/state` | 最新 `RobotState` | 是 |
| GET | `/v1/safety` | `SafetyState` | 是 |
| POST | `/v1/safety/arm` | 使能（检查清单 + 确认短语） | 操作员 |
| POST | `/v1/safety/disarm` | 解除使能 | 操作员 |
| POST | `/v1/safety/reset` | 清除停止锁存 | 操作员 |
| POST | `/v1/leases` | 申请租约 | 是 |
| POST | `/v1/leases/{id}/renew` | 续租 | 持有者 |
| DELETE | `/v1/leases/{id}` | 释放 | 持有者 |
| POST | `/v1/leases/{id}/revoke` | 撤销 | 管理员 |
| POST | `/v1/motion/validate` | 纯校验/预览 | 是 |
| POST | `/v1/motion/execute` | 执行（`Idempotency-Key` 必需） | 租约持有者 |
| POST | `/v1/motion/stop` | 软件停止请求 | 操作员 |
| GET | `/v1/motion/{intent_id}` | 查询结果 | 是 |
| GET | `/v1/capabilities` | 已登记能力 | 是 |
| GET | `/v1/events` | 审计事件（分页） | 是 |
| WS | `/v1/ws/state` | 状态/安全/租约推送 | 是 |
| WS | `/v1/ws/events` | 事件流 | 是 |

**禁止的端点（契约层面永久不提供）：**

- 原始舵机寄存器读写。
- 任意电机占空比/持续时间的直写（底盘只允许 profile 边界内的 `BasePulse`）。
- 直接发布厂商话题。
- 绕过租约的“调试执行”。

### 7.2 WebSocket 信封

```json
{
  "v": 1,
  "type": "state",
  "id": "evt-...",
  "ts": "2026-11-23T10:00:00.000Z",
  "payload": {}
}
```

类型：`hello`、`state`、`safety`、`lease`、`motion`、`event`、`ping`、`pong`。
服务端每 5 s 心跳；客户端 15 s 无消息视为断线；断线不自动撤销任务租约，
但任务租约 TTL 到期后必须停止。

## 8. 配置与证据 Schema

### 8.1 `capability-manifest.v1.schema.json`

见 `02-org-and-repositories.md` 的 `capability.yaml` 示例。关键约束：

- `hardware_access` 只能是 `none`；未来若有例外，必须是独立的安全评审项。
- `publishes`/`subscribes` 必须是命名空间内的意图/状态话题，不能匹配硬件话题
  黑名单：`/servo_controller*`、`/ros_robot_controller*`、`/move_action*`、
  `/joint_states`（只读订阅除外）、`/controller_manager*`。
- `scopes` 必须是 `arm|gripper|base|full` 的子集。
- `deadman_ms` 对 `class=motion_source` 必填，且不得大于 500 ms。

### 8.2 `robot-profile.v1.schema.json`

至少包含：

```yaml
schema_version: 1
robot: armpi_ultra
profile_version: 1.0.0
baseline_ref: {repo: jcheng-manipulation, tag: v1.0.0, commit: b3411e4}
joints:                       # 示例只列 joint1；实际必须包含 joint1..joint5
  - name: joint1
    servo_id: 6
    lower_rad: -1.8012
    upper_rad: 1.8012
    max_velocity_rad_s: 1.2
    max_acceleration_rad_s2: 0.5
gripper:
  servo_id: 1
  open_rad: 2.22              # 契约正向；URDF r_joint 为 -2.22..0.0
  closed_rad: 0.0
  measured_open_pulses: [170, 167]
  measured_closed_pulses: [700, 640]
workspace:
  frame: base_link
  tip_link: end_effector_link
  lower: [-0.05, -0.30, -0.10]
  upper: [0.45, 0.30, 0.55]
table_plane:
  normal: [0.010305373291790987, -0.06301369005824492, 0.9979594551616625]
  offset: 0.07728284112861183
  clearance_m: 0.01
profiles:
  moveit: {max_velocity_rad_s: 1.2, max_acceleration_rad_s2: 0.5}
  step: {max_step_rad: 0.35, min_duration_s: 0.5, max_duration_s: 10.0}
  pose: {max_velocity_rad_s: 0.5}
  fast: {max_velocity_rad_s: 4.0, max_acceleration_rad_s2: 20.0, requires_bus_verification: true}
  base:
    max_abs_duty_normalized: 0.10   # 厂商原始 duty 10 / 满量程 100
    vendor_duty_scale: 100
    max_duration_s: 2.0
source_hashes:
  safety_limits: "sha256:..."
  joint_mapping: "sha256:..."
  xacro: "sha256:..."
```

这是**唯一权威**。安全 YAML、MoveIt 限位、网关映射、应用预设必须由
`tools/render_configs.py` 生成；CI 检查生成物与源一致。

### 8.2.1 设备描述与 `robot-profile.v1` 的边界

`jcheng-devices` v0.1.0 引入 `device-profile.v1` 与 `capability-profile.v1`
两个**仓库内** Schema，用于设备注册、能力匹配与仿真配置。它们不属于本节定义的
契约层：

- 每机器人的 `robot-profile.v1` 仍是关节限位、伺服映射、工作空间与工作台平面的
  运行时唯一权威；
- 设备描述只保存型号、类别、关节角色、能力与验证标记，不包含 `servo_id`、
  `direction`、实测脉冲、命名动作 profile 或 `source_hashes`；
- 重叠的关节限位必须由兼容性检查核对，不得复制成第二份运行时配置；
- 提升为跨仓库契约前，必须先在本仓库完成提案、评审与版本化。

### 8.3 `evidence-record.v1.schema.json`

```json
{
  "schema_version": 1,
  "evidence_id": "ev-20261123-...",
  "kind": "motion|safety|acceptance|replay",
  "created_at": "2026-11-23T10:00:00Z",
  "operator": "name-or-id",
  "git": {"repo": "...", "commit": "...", "tag": "..."},
  "contract_hash": "sha256:...",
  "profile_hash": "sha256:...",
  "environment": {"host": "...", "ros_distro": "humble", "domains": [41, 42]},
  "request": {},
  "validation": {},
  "observations": [],
  "artifacts": [{"path": "logs/...", "sha256": "..."}],
  "result": "pass|fail|inconclusive",
  "limitations": []
}
```

证据包必须能回答：谁、用什么版本、在什么环境、对什么请求、做了什么校验、
观察到了什么、为什么判定通过/失败。

## 9. 与现有 `arm_interfaces` 的关系

| 现有 | 新契约 | 处理 |
|---|---|---|
| `arm_interfaces/action/ArmMove` | 无直接对应 | 历史学习产物；不迁移；如需要，用 `JOINT_STEP` 表达 |
| `arm_interfaces/srv/ValidateTrajectory` | `ValidateMotion` | 语义保留，载荷改为 `JointTrajectory`；`moveit_msgs` 由适配器转换 |
| `moveit_msgs/RobotTrajectory` | `JointTrajectory` | 在 `jcheng_motion` 的 MoveIt 后端转换 |
| `std_msgs/Bool /safety_stop` | `SafetyState` + 兼容镜像 | 迁移期双向镜像；新模块只读新契约 |
| 厂商 `ServosPosition` | 不进入契约 | 仅 `jcheng_hardware` 内部使用 |

新契约**不**为了兼容旧消息而保留字段；兼容由系统层的适配器承担。

## 10. 契约测试

`jcheng-interfaces` CI 必须包含：

1. `colcon build` 在 Humble 与 Jazzy 容器中通过。
2. 每个 msg/srv/action 可生成 Python/C++ 类型。
3. `interface_manifest.json` 可重建且哈希一致。
4. 与上一个发布标签的 diff 符合 SemVer 规则。
5. JSON Schema 全部通过 meta-schema；OpenAPI 通过 lint。
6. ROS IDL 与 JSON Schema 字段一一对应（`test_schema_parity.py`）。
7. 错误码注册表无重复、无删除、语义未改。
8. 契约中不存在硬件原语（静态检查关键词与字段类型）。
9. `jcheng_contracts` 在 Python 3.10/3.11/3.12 导入并通过示例校验。
10. 示例请求/响应可被双方绑定解析。

## 11. 需要确认的决策

1. 是否接受“单一 `MotionCommand` 信封 + kind 校验”，而不是每种运动一个 action？
2. 是否接受“契约中不出现伺服原始脉冲/任意占空比”？
3. 夹爪契约用弧度（推荐）还是归一化 0–1？
4. `POSE` 的 `frame_id` 默认 `base_link`、`tip_link` 必填，是否同意？
5. 是否同意遥操作流契约按推荐方案在 v0.1 定义（预留），到 `jcheng-teleop`
   试点时才启用？
