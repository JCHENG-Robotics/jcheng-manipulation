# 06 · 架构决策与设计（G1）

> 证据等级：`Proposed`（本文件全部是设计决策，尚未实现）
> 对应 PRD：§4 信息架构、§7 系统架构、§7.1 数据契约、§8 NFR
> 基线：PRD v1.0；本文件不扩大范围

---

## 1. 架构决策记录（ADR）

### ADR-001 · 复用厂商 SDK 与 ROS 2 Driver，不重写协议

- **决策**：串口协议、CRC、报文解析、舵机读写全部复用；新系统只做“校验 + 编排 + 记录 + 展示”。
- **理由**：PRD §1.4、§2 原则 4；协议细节无公开文档，重写风险高。
- **后果**：受厂商缺陷约束（见 03 文档 §5）；修补必须单独审批。
- **状态**：Accepted（由任务约束强制）

### ADR-002 · 后端与 ROS 网关分进程

```text
Browser ──HTTP/WS──> FastAPI Backend ──localhost RPC──> ROS 2 Gateway ──ROS 2──> Vendor Driver ──serial──> STM32
```

- **决策**：G1 起就把 ROS 访问隔离到独立进程 `armpi_debug_gateway`；FastAPI 不直接 import `rclpy`，更不 import `Board`。
- **理由**：
  1. SDK 读取可能永久阻塞（02 文档 §5），不能让它拖垮 Web API；
  2. SR-10 单一串口所有者：只有网关进程持有 ROS 写权限；
  3. PRD §7 明确“FastAPI 只允许通过唯一 ROS 2 控制网关写入”；
  4. 网关崩溃可被后端检测并锁存，Web 仍可展示诊断。
- **传输**：Unix domain socket 上的 HTTP/JSON（`/run/armpi-debug/gateway.sock`），仅本机；或 `127.0.0.1:8765` + 共享 token。前端永远看不到该端口。
- **后果**：多一个进程和部署步骤；换来故障隔离与清晰边界。
- **状态**：Proposed（推荐）

### ADR-003 · 反馈真实性分级

- **决策**：所有位置数据必须带 `truth`：
  - `commanded`：最后下发目标（现有 `joint_states`）
  - `measured_on_demand`：通过 `GetBusServoState` 读回（语义待 D-10 确认）
  - `simulated`：Mock
  - `unknown`：无数据/超时
- **理由**：PRD §7.2“SDK 写入成功不等于关节达到目标”；现有 `joint_states` 是命令回显。
- **后果**：UI 不能显示单一“反馈角”；必须显示“目标 / 读回 / 年龄 / 来源”。
- **状态**：Proposed

### ADR-004 · 禁止浏览器直连 rosbridge 做控制

- **决策**：不使用厂商 `bringup.launch.py` 中的 `rosbridge_websocket` 作为控制通道；它最多作为只读调试参考，且默认不启动。
- **理由**：rosbridge 会绕过所有后端校验、审计、租约与限流；SR-02/SR-07 不允许。
- **后果**：前端只认 FastAPI 的 REST/WS。
- **状态**：Proposed（安全红线）

### ADR-005 · Mock Mode 不依赖 ROS/Gazebo

- **决策**：Mock 是后端内的纯 Python 适配器，带确定性场景与故障注入；不启动 ROS 2、不启动 Gazebo。
- **理由**：NFR-05 要求无硬件可开发；Gazebo 启动慢、依赖重，且会把“模拟物理”误当成“真实反馈”。
- **后果**：Mock 只模拟接口行为与可控故障，不模拟刚体动力学；UI 明确标注 `SIMULATION`。
- **状态**：Proposed

### ADR-006 · 持久化分层

- **决策**：
  - 事件/命令/租约/配置：SQLite（WAL），append-only 事件表；
  - 高频采样：内存环形缓冲（G1 不落盘）；
  - 导出：CSV + JSON 元数据；
  - rosbag2：G2 仅保存路径引用，不自行实现录制。
- **理由**：PRD §7、§9；G1 只需要可审计、可复算的最小集合。
- **状态**：Proposed

### ADR-007 · 前端技术栈与设计系统

- **决策**：
  - Vite + React 18 + TypeScript（strict）
  - Tailwind CSS + CSS variables（设计令牌）
  - Radix Primitives（Dialog/Tooltip/Tabs/Select）+ lucide-react 图标
  - Zustand（实时状态）+ TanStack Query（REST 缓存/重试）
  - zod（WS/REST 运行时校验，防 NFR-04）
  - Vitest + Testing Library + Playwright
  - 图表：G1 只用 SVG sparkline；G2 引入 uPlot（时间序列）；G4 才引入 Three.js
- **理由**：高频调试界面需要密度、键盘操作与稳定渲染；避免重量级 UI 框架和过早 3D。
- **状态**：Proposed

### ADR-008 · 停止语义四态

- **决策**：`StopRequested → StopDispatched → StopObserved → StopUnknown`；UI 永不使用“急停”。
- **理由**：SR-06；SDK 无 ACK。
- **状态**：Proposed（安全红线）

### ADR-009 · 部署与网络

- **决策**：
  - G1 默认 `127.0.0.1` 绑定；局域网访问需显式配置 + token；
  - 网关与后端同机（机器人主控）；
  - 前端静态文件由后端托管，开发期用 Vite proxy；
  - 不依赖云服务。
- **理由**：SR-07、NFR-09；部署位置待基准测试（PRD §15）。
- **状态**：Proposed

### ADR-010 · G1 范围冻结

- **决策**：G1 只做 Dashboard / Joint Control / Diagnostics / Event Timeline / Settings + Mock + 只读 Hardware Discovery。Trajectory / Experiment / 3D / Camera / Replay 不进入 G1 代码路径。
- **理由**：PRD §2 范围冻结；避免“页面很多、证据很少”。
- **状态**：Accepted（由任务约束强制）

---

## 2. 系统组件

```text
┌──────────────────────────────────────────────────────────────┐
│ Browser (React + TS)                                         │
│  AppShell / Pages / Components / WS client / zod validation  │
└───────────────┬──────────────────────────────────────────────┘
                │ HTTP + WebSocket (same origin, 127.0.0.1)
┌───────────────▼──────────────────────────────────────────────┐
│ FastAPI Backend                                              │
│  API routers │ Session/Lease │ SafetySupervisor │ Audit      │
│  StateCache  │ EventStore    │ MockAdapter      │ WS hub     │
└───────────────┬──────────────────────────────────────────────┘
                │ local HTTP/JSON over UDS (token)
┌───────────────▼──────────────────────────────────────────────┐
│ armpi_debug_gateway (ROS 2 node, rclpy)                      │
│  CapabilityProbe │ StateReader │ CommandDispatcher │ Watchdog│
└───────────────┬──────────────────────────────────────────────┘
                │ ROS 2 topics/services/actions
┌───────────────▼──────────────────────────────────────────────┐
│ Vendor: ros_robot_controller + servo_controller + kinematics │
└───────────────┬──────────────────────────────────────────────┘
                │ serial /dev/rrc @ 1,000,000
┌───────────────▼──────────────────────────────────────────────┐
│ STM32 + 6 bus servos                                          │
└──────────────────────────────────────────────────────────────┘
```

### 2.1 职责边界

| 层 | 可以做 | 不可以做 |
|---|---|---|
| Frontend | 展示、预校验、收集确认、发起请求 | 直接连串口/ROS/rosbridge；决定安全 |
| Backend | 会话、租约、校验、审计、持久化、Mock | 直接开串口；绕过网关写硬件 |
| Gateway | ROS 读写、能力探测、超时、返回证据 | 业务规则、用户权限、实验逻辑 |
| Vendor Driver | 串口协议、舵机读写 | 安全门禁、审计 |

### 2.2 状态采集与命令链路分离

- **采集链路**：Gateway 定时/按需读取 → Backend 缓存 → WS 推送。可丢帧、可降采样。
- **命令链路**：Frontend → Backend 校验 → Gateway 下发 → 事件落库。不可丢、不可重放。
- **两条链路不共享内存队列**；命令结果必须落事件表。

---

## 3. 数据契约

所有时间戳携带时钟来源：`wall_time`（ISO8601）、`monotonic_ms`、`ros_time_ns`（可空）。

### 3.1 RobotState

```jsonc
{
  "robot_id": "armpi-ultra-01",
  "mode": "hardware",                  // hardware | mock | replay
  "connection_state": "ReadOnly",      // 见 §4.1
  "source_label": "LIVE",              // LIVE | SIMULATION | REPLAY
  "control_owner": null,               // session_id | null
  "received_at": "2026-11-13T10:00:00.123Z",
  "monotonic_ms": 123456,
  "ros_time_ns": null,
  "joints": [
    {
      "name": "joint1",
      "servo_id": 6,
      "target_rad": 0.0,
      "target_raw": 500,
      "position_rad": null,
      "raw_position": null,
      "truth": "commanded",            // commanded | measured_on_demand | simulated | unknown
      "validity": "valid",             // valid | stale | unavailable | invalid
      "sample_age_ms": null,
      "source": "controller_manager/joint_states"
    }
  ],
  "faults": [],
  "mapping_version": "0.1.0-provisional",
  "limit_version": "0.1.0-provisional",
  "capabilities": {
    "hardware_motion": false,
    "measured_position": "unknown",    // true | false | unknown
    "temperature": true,
    "voltage": true,
    "torque_state": false,
    "trajectory_action": true,
    "camera": false
  }
}
```

**规则**：

- 未知数据用 `null` + `validity`/`reason`，不填 0。
- `truth=commanded` 时，`position_rad` 为 `null`，另给 `last_command_rad`。
- `capabilities` 必须来自探测，不得硬编码为 true。

### 3.2 ConnectionReport

```jsonc
{
  "checked_at": "...",
  "links": [
    {"name": "serial_device", "status": "ok|missing|busy|unknown",
     "evidence": "/dev/rrc exists", "detail": null},
    {"name": "driver_node", "status": "online|offline|unknown",
     "evidence": "ROS graph: /ros_robot_controller", "detail": null},
    {"name": "gateway", "status": "online|degraded|offline",
     "evidence": "heartbeat age 120 ms", "detail": null},
    {"name": "feedback", "status": "fresh|stale|unavailable|unknown",
     "evidence": "last measured read 340 ms ago", "detail": null}
  ],
  "motion_blockers": [
    {"code": "MAPPING_UNVERIFIED", "message": "关节映射尚未实机验证",
     "remediation": "完成 G0 V-01…V-08 并更新 joint_mapping.yaml"}
  ]
}
```

### 3.3 MotionCommand / ValidationResult

```jsonc
{
  "command_id": "cmd_01H...",
  "session_id": "sess_...",
  "lease_id": "lease_...",
  "source": "web",
  "joint_targets": [{"name": "joint1", "target_rad": 0.1}],
  "duration_s": 1.5,
  "mapping_version": "0.1.0-provisional",
  "confirmation_token": "ct_...",
  "issued_at": "..."
}
```

```jsonc
{
  "valid": false,
  "errors": [
    {"code": "OUT_OF_RANGE", "joint": "joint3",
     "message": "target 0.20 rad exceeds verified max 0.00 rad",
     "details": {"min": -2.0944, "max": 0.0}}
  ],
  "warnings": [
    {"code": "MAPPING_PROVISIONAL", "message": "映射未验证，仅允许受限测试"}
  ],
  "preview": {
    "from_rad": 0.0, "to_rad": 0.1, "delta_rad": 0.1,
    "estimated_duration_s": 1.5,
    "raw_target": 524
  },
  "confirmation_token": "ct_...",   // 仅 valid 时下发，TTL 10 s，单次
  "expires_at": "..."
}
```

### 3.4 CommandEvent

```jsonc
{
  "event_id": "evt_...",
  "command_id": "cmd_...",
  "stage": "Dispatched",
  "timestamp": "...",
  "source": "backend|gateway|driver",
  "result": "ok|error|unknown",
  "error_code": null,
  "details": {"ros_publish_returned": true},
  "trace_id": "..."
}
```

### 3.5 Fault

```jsonc
{
  "fault_id": "flt_...",
  "severity": "Info|Warning|Fault|Critical",
  "code": "FEEDBACK_TIMEOUT",
  "message": "...",
  "source": "gateway|backend|driver|frontend",
  "first_seen": "...",
  "last_seen": "...",
  "latched": true,
  "context": {},
  "remediation": ["检查舵机接线", "确认 /dev/rrc 未被占用"]
}
```

### 3.6 ControlLease

```jsonc
{
  "lease_id": "lease_...",
  "session_id": "sess_...",
  "owner_label": "jericho@laptop",
  "acquired_at": "...",
  "expires_at": "...",
  "heartbeat_interval_ms": 5000,
  "ttl_ms": 15000,
  "state": "Active"
}
```

### 3.7 ExperimentTrial（G3，仅先定字段）

沿用 PRD §7.1；G1 不实现，但事件表保留 `experiment_id/trial_id/pair_id` 可空列，避免后期迁移。

---

## 4. 状态机

### 4.1 连接状态

| 状态 | 进入条件 | 允许运动 |
|---|---|---|
| `Disconnected` | 无网关/无驱动 | ❌ |
| `Connecting` | 正在探测，≤ 5 s | ❌ |
| `ReadOnly` | 驱动在线，但映射/限位/控制权未满足 | ❌ |
| `Ready` | 全部 G1 门禁通过（§05 文档 §6） | ✅ |
| `Executing` | 有命令处于 `Dispatched/Observed` | ❌（新命令） |
| `Degraded` | 部分能力缺失（如反馈不可用） | ❌ |
| `Fault` | 锁存故障 | ❌ |
| `EStopOrPowerCutReported` | **仅有可信硬件信号时才可进入** | ❌ |
| `Replay` | 回放模式 | ❌ |
| `Simulation` | Mock 模式 | 仅模拟 |

**本栈没有急停信号** → 永远不进入 `EStopOrPowerCutReported`；UI 显示“急停能力 Unknown”。

转移规则：

- `Ready → Fault`：任一锁存故障。
- `Ready → ReadOnly`：反馈过期、租约丢失、映射版本变化。
- `Fault → ReadOnly`：人工确认 + 重新体检；不自动回 `Ready`。
- `Disconnected → *`：重连后只到 `ReadOnly`，需重新申请控制权。

### 4.2 命令状态

见 05 文档 §4。补充：

```text
Requested ──valid──> Validated ──dispatch──> Dispatched
    │                    │                       │
    └──invalid──> Failed  └──expired──> Failed   ├──observed within tol──> Observed ──stable N──> Completed
                                                  └──timeout/no evidence──> Unknown
```

- `Acknowledged` 保留但永不产生。
- `Unknown` 是终态，不允许自动重试。
- 每个转移写一条 `CommandEvent`。

### 4.3 租约

```text
None ──acquire──> Active ──heartbeat──> Active
                    │                      │
                    ├──ttl expired──> Expired
                    ├──release──────> Released
                    └──disconnect───> Revoked
```

- 同一时刻只有一个 `Active`。
- 租约过期/撤销后，后端撤销所有运动许可；不自动重新获取。
- 浏览器断线 → 心跳停止 → TTL 后 `Revoked`。

### 4.4 故障等级

`Info < Warning < Fault < Critical`。
`Fault/Critical` 默认 `latched=true`；清除需要：
1. 操作者点击“重新体检”；
2. 所有门禁重新通过；
3. 重新申请租约。

---

## 5. 前端信息架构

### 5.1 路由（G1 只暴露已实现页面）

| 路由 | 页面 | 状态 |
|---|---|---|
| `/` | Dashboard | G1 |
| `/joints` | Joint Control | G1 |
| `/diagnostics` | Diagnostics | G1 |
| `/events` | Event Timeline | G1 |
| `/settings` | Settings | G1 |
| `/trajectory` | Trajectory Inspector | G2，**G1 不注册路由** |
| `/experiments` | Experiment Lab | G3，**G1 不注册路由** |

> 未实现功能只在 Settings 的 “Roadmap” 文本区列出，**不放置可点击的假按钮**。

### 5.2 AppShell 布局

```text
┌──────────────────────────────────────────────────────────────────────┐
│ TopBar: Robot · LIVE/SIM/REPLAY · Connection · Owner · FeedbackAge   │
│         · FaultLevel · ReadOnly/Controllable · [停止请求]             │
├──────────┬───────────────────────────────────────────────────────────┤
│ LeftNav  │ Page content                                              │
│ 200px    │                                                           │
│          │                                                           │
├──────────┴───────────────────────────────────────────────────────────┤
│ StatusBar: last event · clock source · build · mapping version        │
└──────────────────────────────────────────────────────────────────────┘
```

TopBar 规则：

- `source_label` 用高对比徽章，始终可见；`SIMULATION` 用青色 + 斜纹背景，`REPLAY` 用紫色。
- 反馈年龄：显示 `age` + 来源（`commanded` / `measured`）；无数据显示 `Unavailable`。
- 停止按钮文案：**“停止请求”**，Tooltip 固定包含“软件请求，非急停；物理断电是最终防护”。
- 故障等级：文字 + 图标 + 计数，不仅靠颜色。

### 5.3 Dashboard

| 区块 | 内容 | 数据来源 |
|---|---|---|
| Connection Chain | serial → driver → gateway → feedback，四段状态与证据 | `ConnectionReport` |
| Safety Gate | 12 项门禁清单，通过/阻止/未知 + 原因 + 修复建议 | `SafetyReport` |
| Joint Snapshot | 6 行：关节、ID、目标、读回、truth、年龄 | `RobotState` |
| Active Faults | 按严重度排序，含 first/last seen、remediation | `Fault[]` |
| Latest Events | 最近 20 条命令/状态事件 | `EventStore` |
| Control | 当前持有者、租约剩余、申请/释放按钮（按状态启用） | `ControlLease` |

禁止：把“网关在线”显示成“机器人就绪”。

### 5.4 Joint Control

交互流程（严格）：

```text
选择关节 → 输入目标(rad) + 时长(s)
        → 本地预校验（即时反馈，不调后端）
        → [预览] 调用 /api/motion/preview
        → 展示 from/to/delta/raw/预计时长/校验结果
        → [确认执行] 弹窗二次确认（显示 command_id 前 8 位）
        → 调用 /api/motion/execute
        → 展示状态轨迹：Requested → Validated → Dispatched → …
```

约束：

- 滑块只更新本地预览，**不发送命令**；输入框失焦也不发送。
- `Execute` 在无租约/门禁未通过/预览过期时禁用，并显示原因。
- 单关节与多关节（安全姿态）分 Tab；多关节在 G1 仅允许“读取当前目标并微调”，不做自动插值。
- 每个关节行显示：名称、舵机 ID、目标 rad、读回 rad（或 `Unavailable`）、原始脉冲、限位来源/验证状态、年龄。
- 停止按钮始终可见；停止结果按四态展示。

### 5.5 Diagnostics

| 区块 | 内容 |
|---|---|
| ROS Graph | 期望节点/话题/服务 vs 实际发现；缺失项标红并给启动命令 |
| Serial | `/dev/rrc` 存在性、权限、可能的占用进程（只读提示） |
| SDK/Driver Errors | 已知缺陷 B-01…B-12 的检测结果（如 `get_torque_state` 不可用） |
| Feedback Probe | 手动触发单次 6 舵机读取；显示每舵机耗时、结果、超时、错配风险 |
| Environment | ROS_DOMAIN_ID、主机名、网关版本、后端版本、mapping version |
| Fault Injection | **仅 Mock 模式**：超时、断线、空反馈、NaN、舵机缺失、节点退出 |

### 5.6 Event Timeline

- 虚拟化表格：时间、severity、stage、source、command_id（短）、关节、结果。
- 右侧详情：完整 JSON、错误码、trace_id、复制按钮。
- 过滤：severity、stage、source、command_id、时间范围、关节。
- 导出：CSV / JSON（G1）；导出内容含 schema version。
- 故障发生时“冻结视图”按钮：暂停自动滚动，不丢缓冲。

### 5.7 Settings

| 区块 | G1 行为 |
|---|---|
| Connection | gateway URL、mode（mock/hardware）、ROS_DOMAIN_ID（只读显示）、测试连接 |
| Display | 单位（rad/deg）、小数位、时间显示（本地/UTC）、密度 |
| Joint Mapping | 只读表格 + `verified` 徽章 + “如何验证”链接；编辑禁用并说明需 Board 批准 |
| Safety | 反馈年龄阈值、命令时长上限、步长上限；修改需二次确认并审计 |
| Logging | 事件保留天数、导出路径（服务端）、隐私脱敏开关 |
| Roadmap | 文本列出 G2–G4，明确“未实现” |

危险配置变更（阈值、模式切换）：
- 前端二次确认；
- 后端写审计事件；
- 不允在 `Executing` 状态修改。

---

## 6. 组件体系

### 6.1 基础组件

| 组件 | 职责 | 关键 props |
|---|---|---|
| `StatusChip` | 状态文字 + 图标 + 形状 + 颜色 | `tone`, `label`, `icon`, `detail` |
| `SourceBadge` | LIVE / SIMULATION / REPLAY | `source` |
| `MetricValue` | 数值 + 单位 + 来源 + 年龄 | `value`, `unit`, `truth`, `ageMs`, `precision` |
| `DataAge` | 年龄与新鲜度 | `ageMs`, `thresholds` |
| `UnitLabel` | 统一单位排版 | `unit` |
| `FaultBanner` | 故障摘要与展开 | `fault`, `onOpen` |
| `SafetyGateList` | 门禁清单 | `items` |
| `JointRow` | 关节行 | `joint`, `mapping`, `selected` |
| `CommandStateTrack` | 命令状态轨迹 | `events` |
| `EventTable` | 虚拟化事件表 | `rows`, `filters` |
| `ConfirmDialog` | 二次确认 | `title`, `diff`, `token`, `onConfirm` |
| `Unavailable` | 统一“不可用”占位 | `reason`, `remediation` |
| `CopyableError` | 错误码/详情复制 | `code`, `message`, `details` |
| `Sparkline` | G1 轻量趋势 | `points`, `unit` |
| `MockBadge` | 模拟数据标识 | — |
| `StopButton` | 停止请求四态 | `state`, `onRequest` |

### 6.2 组合组件

- `ConnectionChain`
- `JointTable`
- `CommandPanel`
- `LeasePanel`
- `ProbePanel`
- `FaultList`
- `SettingsSection`

### 6.3 设计令牌

```css
--bg: #0B0F14;
--panel: #11161D;
--panel-2: #161D26;
--border: #1F2937;
--text: #E6EDF3;
--text-muted: #8B98A9;
--accent: #3B82F6;
--ok: #22C55E;
--warn: #F59E0B;
--fault: #EF4444;
--unknown: #A855F7;
--sim: #06B6D4;
--replay: #8B5CF6;
--radius: 6px;
--row-h: 32px;
--font-ui: Inter, system-ui, sans-serif;
--font-mono: "JetBrains Mono", ui-monospace, monospace;
```

规则：

- 数值一律等宽字体；单位用小号、弱化颜色。
- 状态不能只靠颜色：`StatusChip` 同时有图标与文字。
- 表格行高 32 px，支持键盘上下选择。
- 所有时间显示 `HH:mm:ss.SSS`，并附时区/时钟来源 Tooltip。
- 错误信息可复制（代码块 + Copy 按钮）。
- 动效仅用于状态变化提示，尊重 `prefers-reduced-motion`。

---

## 7. 后端模块与 API

### 7.1 模块

```text
backend/app/
├── main.py                 # FastAPI 装配
├── api/
│   ├── health.py
│   ├── robot.py            # state, discovery, capabilities
│   ├── control.py          # lease
│   ├── motion.py           # preview, execute, stop
│   ├── events.py           # list, detail, export
│   ├── settings.py
│   └── mock.py             # 仅 mock 模式挂载
├── domain/
│   ├── enums.py
│   ├── models.py           # pydantic
│   └── state_machine.py
├── safety/
│   ├── supervisor.py       # 门禁编排
│   ├── validation.py       # 数值/单位/限位
│   ├── lease.py
│   └── watchdog.py
├── adapters/
│   ├── base.py             # RobotAdapter Protocol
│   ├── mock.py
│   └── gateway.py          # 调本地网关
├── persistence/
│   ├── db.py               # SQLite WAL
│   ├── events.py
│   └── commands.py
├── realtime/
│   ├── hub.py              # WS 广播
│   └── sampling.py         # 降采样/丢帧策略
└── config.py
```

### 7.2 REST（G1）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/health` | 后端/网关/DB 健康 |
| GET | `/api/robot/state` | 当前快照 |
| POST | `/api/robot/discovery` | 触发一次只读体检（限流） |
| GET | `/api/robot/capabilities` | 能力矩阵 |
| POST | `/api/control/lease` | 申请控制权 |
| POST | `/api/control/heartbeat` | 续租 |
| DELETE | `/api/control/lease` | 释放 |
| POST | `/api/motion/preview` | 校验 + 预览 + confirmation_token |
| POST | `/api/motion/execute` | 执行（需 token + lease + Idempotency-Key） |
| POST | `/api/motion/stop` | 停止请求 |
| GET | `/api/events` | 过滤/分页 |
| GET | `/api/events/{id}` | 详情 |
| GET | `/api/events/export` | CSV/JSON |
| GET/PUT | `/api/settings` | 非危险配置 |
| POST | `/api/mock/faults` | Mock 故障注入（仅 mock） |

### 7.3 WebSocket

| 通道 | 频率 | 内容 |
|---|---|---|
| `/ws/state` | 5–10 Hz | `RobotState` 快照 + `seq` + `degraded` |
| `/ws/events` | 事件驱动 | `CommandEvent` / `Fault` |

规则：

- 消息带 `schema_version` 与 `seq`；前端 zod 校验失败时丢弃并显示“数据契约错误”，不崩溃。
- WS 断开后前端进入 `Disconnected` 展示；不自动重发运动命令。
- 事件通道与状态通道分离；事件不丢（落库），状态可丢帧。

### 7.4 错误格式

```json
{
  "code": "MOTION_BLOCKED",
  "message": "控制权租约已过期",
  "details": {"lease_id": "...", "expired_at": "..."},
  "trace_id": "..."
}
```

HTTP 状态：400 校验失败、403 无控制权、409 状态冲突、423 锁存故障、503 网关不可用。

### 7.5 幂等与防重

- `POST /api/motion/execute` 必须带 `Idempotency-Key`；同 key 在 10 s 内返回同一结果。
- `confirmation_token` 单次使用，绑定 `payload_hash + session_id + lease_id`，TTL 10 s。
- 单飞：同一时刻只允许一个 `Dispatched/Observed` 命令。

---

## 8. Mock 与 Hardware 边界

| 能力 | Mock | Hardware G1 | UI 标识 |
|---|---|---|---|
| 连接 | 即时成功/可注入失败 | 只读探测；失败显示 `Disconnected` | `SIMULATION` / `LIVE` |
| 状态 | 合成状态，含噪声/延迟 | 命令回显 + 按需读回 | `truth=simulated/commanded/measured` |
| 运动 | 模拟执行，可注入超时 | **默认禁用**，能力为 false | 禁用 + 原因 |
| 停止 | 模拟四态 | 仅 `StopDispatched` 起 | 四态文案 |
| 故障注入 | 完整 | 不可用 | 仅 Mock 显示按钮 |
| 数据导出 | 可 | 可 | 元数据含 `mode` |
| 3D/相机 | 不模拟 | 不支持 | 不显示 |

**Hardware Mode 的“未实现”表达**：

- `capabilities.hardware_motion=false`；
- 所有 `Execute` 按钮禁用，Tooltip：“Hardware motion not implemented — G1 只读发现阶段”；
- 不显示伪造的 `Ready`；最多到 `ReadOnly`；
- 若未来启用，必须先在后端配置显式开关 + 审计。

---

## 9. 部署与运行（规划，不含实现）

```text
开发（无硬件）:
  backend:  uvicorn app.main:app --port 8000        MODE=mock
  frontend: npm run dev                             VITE_API_BASE=http://127.0.0.1:8000

硬件（机器人主控）:
  gateway:  python -m armpi_debug_gateway            (rclpy, /run/armpi-debug/gateway.sock)
  backend:  uvicorn app.main:app --host 127.0.0.1    MODE=hardware
  frontend: 由 backend 静态托管，或开发机 Vite proxy

前置:
  ros2 launch ros_robot_controller ros_robot_controller.launch.py
  ros2 launch servo_controller servo_controller.launch.py
  # 不启动 bringup.launch.py（会启动 startup_check 自动运动）
```

环境变量：

| 变量 | 默认 | 说明 |
|---|---|---|
| `ARMPI_MODE` | `mock` | `mock` / `hardware` |
| `GATEWAY_SOCKET` | `/run/armpi-debug/gateway.sock` | 网关地址 |
| `GATEWAY_TOKEN` | 随机 | 本机共享密钥 |
| `DB_PATH` | `./data/studio.db` | SQLite |
| `BIND_HOST` | `127.0.0.1` | 不默认暴露局域网 |
| `FEEDBACK_MAX_AGE_MS` | `500` | 门禁阈值 |

---

## 10. 测试与质量门禁（摘要）

| 层 | 内容 |
|---|---|
| 单元 | 映射换算、限位校验、状态机、指标计算 |
| 契约 | pydantic/zod schema、WS 消息、CSV 字段 |
| 集成 | Mock 故障注入：超时/断线/空反馈/NaN/重复请求/租约争用 |
| E2E | Playwright：默认只读、拒绝路径、预览→确认、停止四态 |
| 硬件 | 单独目录，永不 CI 自动启动；人工看护 |

**负向测试必须覆盖**：超限、NaN/Inf、未知关节、无租约、租约过期、预览过期、重复执行、WS 断线、网关超时、DB 写失败。

---

## 11. 已知限制（设计层面）

1. 无真实反馈时，Joint Control 只能提高“命令已下发”的证据，不能证明运动。
2. 网关与后端分进程增加部署复杂度；G1 先本机，远程访问另评。
3. SQLite 不适合高频原始采样；G2 需要独立采样文件或 rosbag2。
4. Mock 不模拟动力学，不能用于控制算法验证。
5. 现有 Driver 缺陷未修补前，Diagnostics 的部分探测可能触发阻塞；必须限流 + 超时 + 熔断。
