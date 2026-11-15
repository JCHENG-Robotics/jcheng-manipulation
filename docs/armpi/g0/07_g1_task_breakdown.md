# 07 · G1 任务分解与待确认问题

> 证据等级：`Proposed`
> 对应 PRD：§2 阶段门槛、§11 测试门禁、§14 G1 验收清单
> 原则：先契约与门禁，后页面；先 Mock，后只读硬件；实机运动单独授权

---

## 1. 工作流与顺序

```text
E0 基线与仓库
  └─ E1 领域模型/数据契约
       ├─ E2 安全核心（校验/门禁/租约/看门狗）
       │    └─ E4 后端 API + WS + 持久化
       │         └─ E6 前端页面
       ├─ E3 Mock 适配器 + 故障注入
       │    └─ E4
       └─ E7 硬件只读网关
            └─ E9 受控实机验证（需 Board）
E5 前端壳与设计系统（可与 E2/E4 并行）
E8 QA 贯穿 E1–E7
```

**关键路径**：E1 → E2 → E3 → E4 → E6。E7 不阻塞前端。

---

## 2. 任务包

### E0 · 基线与仓库

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E0-1 | 确认代码位置与仓库策略 | ADR / 目录 | Board 批准（Q-01） |
| E0-2 | 记录厂商栈版本与哈希 | `vendor_versions.md` | SDK SHA、包版本、udev 规则 |
| E0-3 | 许可证与来源清单 | `LICENSES.md` | 厂商代码不复制、不混编 |
| E0-4 | 开发环境 | `README` / `.env.example` | Node/Python/ROS 版本锁定 |
| E0-5 | CI 骨架 | lint + unit | 不依赖硬件；Mock 可跑 |

### E1 · 领域模型与契约

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E1-1 | 枚举：连接/命令/租约/故障 | `domain/enums.py` | 与 PRD 术语一致 |
| E1-2 | Pydantic 模型 | `domain/models.py` | 未知值可为 null；禁止默认 0 |
| E1-3 | 前向/反向 zod schema | `frontend/src/api/schema.ts` | 与后端契约测试通过 |
| E1-4 | 状态机 | `state_machine.py` + 单测 | 非法转移抛错；`Acknowledged` 不可达 |
| E1-5 | 关节映射加载器 | `configs/joint_mapping.yaml` | 启动校验；`mapping_hash` |
| E1-6 | 事件 schema 版本 | `schema_version=1` | 导出含版本 |

### E2 · 安全核心

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E2-1 | 数值校验 | `safety/validation.py` | 超限/NaN/Inf/未知关节/单位错误全部拒绝 |
| E2-2 | 门禁编排 | `safety/supervisor.py` | 12 项检查可单测；返回结构化 blockers |
| E2-3 | 租约 | `safety/lease.py` | 单持有者、TTL、心跳、撤销 |
| E2-4 | 预览 token | `safety/tokens.py` | 单次、TTL、payload 绑定 |
| E2-5 | 幂等 | `api/motion.py` | 同 Idempotency-Key 返回同结果 |
| E2-6 | 看门狗 | `safety/watchdog.py` | 租约过期/网关失联 → 撤销许可 + 锁存 |
| E2-7 | 故障锁存 | `domain/faults.py` | 只能人工 re-arm |
| E2-8 | 停止四态 | `api/motion.py` | 无证据 → `StopUnknown` |

### E3 · Mock 适配器

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E3-1 | 纯 Python 状态模型 | `adapters/mock.py` | 无 ROS 依赖 |
| E3-2 | 命令模拟 | — | 可配置延迟/失败/超时 |
| E3-3 | 故障注入 API | `POST /api/mock/faults` | 超时/断线/空反馈/NaN/舵机缺失 |
| E3-4 | 确定性种子 | `seed` 参数 | 同种子同结果 |
| E3-5 | 数据标识 | 所有状态 `truth=simulated` | UI 显示 `SIMULATION` |
| E3-6 | 独立启动 | `ARMPI_MODE=mock` | 无 ROS、无串口、无硬件 |

### E4 · 后端 API 与持久化

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E4-1 | FastAPI 装配 | `main.py` | `/api/health` |
| E4-2 | 状态缓存 | `realtime/state.py` | 快照 + seq |
| E4-3 | WS hub | `/ws/state`, `/ws/events` | 断线不崩；契约错误可观测 |
| E4-4 | SQLite | `persistence/` | WAL；写入失败 → Fault |
| E4-5 | 事件查询/导出 | `/api/events` | CSV 字段稳定、可复算 |
| E4-6 | 网关客户端 | `adapters/gateway.py` | 超时、重试上限、熔断 |
| E4-7 | 配置 | `config.py` | 默认 localhost；危险项审计 |

### E5 · 前端壳与设计系统

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E5-1 | Vite/TS strict | 工程 | `tsc --noEmit` 通过 |
| E5-2 | 设计令牌 | `styles/tokens.css` | 深色、等宽数值、密度 |
| E5-3 | AppShell | TopBar/LeftNav/StatusBar | 所有全局字段可见 |
| E5-4 | 基础组件 | `components/` | Storybook 或独立页 |
| E5-5 | API/WS 客户端 | `api/` | zod 校验；失败降级 |
| E5-6 | 状态存储 | `stores/` | 单一快照源；无乐观运动成功 |

### E6 · 页面

| ID | 页面 | 关键验收 |
|---|---|---|
| E6-1 | Dashboard | 四段链路、门禁、关节快照、故障、最近事件 |
| E6-2 | Joint Control | 预览→确认；无租约禁用并解释；停止四态 |
| E6-3 | Diagnostics | ROS graph、串口、缺陷检测、单次探针、仅 Mock 故障注入 |
| E6-4 | Event Timeline | 过滤、详情、复制、导出、冻结视图 |
| E6-5 | Settings | 连接/显示/映射只读/安全阈值/日志/Roadmap |

**每个页面必须定义空态、加载态、错误态、不可用态。**

### E7 · 硬件只读网关

| ID | 任务 | 交付 | 验收 |
|---|---|---|---|
| E7-1 | rclpy 节点 | `armpi_debug_gateway` | 启动/关闭干净 |
| E7-2 | 能力探测 | `/probe` | 节点/话题/服务存在性 |
| E7-3 | 状态读取 | `/state` | 命令回显 + 按需读回；带 `truth` |
| E7-4 | 超时/熔断 | — | 单次探针 ≤ 300 ms；失败锁存 |
| E7-5 | 本地 RPC | UDS + token | 仅本机；后端唯一客户端 |
| E7-6 | 运动写接口 | `/dispatch`, `/stop` | **G1 默认禁用**，返回 `NOT_IMPLEMENTED` |
| E7-7 | 启动副作用 | 文档 | 记录 Driver 启动写操作 |

### E8 · QA

| ID | 任务 | 验收 |
|---|---|---|
| E8-1 | 单元测试 | 映射/校验/状态机/指标 ≥ 覆盖关键分支 |
| E8-2 | 契约测试 | 后端模型 ↔ 前端 schema |
| E8-3 | Mock 集成 | 故障注入全通过 |
| E8-4 | E2E | Playwright 覆盖 5 页面关键路径 |
| E8-5 | 负向测试 | 见 §3 |
| E8-6 | 可复现性 | 一条命令启动 Mock；README 步骤 |
| E8-7 | 缺陷登记 | 已知限制清单 |

### E9 · 受控实机验证（需 Board 授权）

| ID | 任务 | 前置 |
|---|---|---|
| E9-1 | 只读体检 | E7 完成；物理隔离 |
| E9-2 | V-01…V-08 映射验证 | Board 书面批准；人工看护 |
| E9-3 | 单关节受限测试 | `hardware_motion` 临时开启；单次确认 |
| E9-4 | 故障复现（断线/超时） | 可立即断电 |
| E9-5 | 验收报告 | 证据、回滚、残余风险 |

---

## 3. 测试计划

### 3.1 单元

| ID | 用例 | 期望 |
|---|---|---|
| T-MAP-01 | joint3 正角度 +0.1 | 拒绝（超出 0.0 上限） |
| T-MAP-02 | raw 转换边界 0/1000 | rad 精确匹配公式 |
| T-MAP-03 | 反向关节正/反算 | 往返误差 < 1e-9 |
| T-MAP-04 | 未知关节名 | 拒绝 + 事件 |
| T-MAP-05 | 映射版本不匹配 | 拒绝执行 |
| T-SAFE-01 | NaN / +Inf / -Inf | 拒绝 |
| T-SAFE-02 | 步长 > max_step | 拒绝 |
| T-SAFE-03 | duration < min / > max | 拒绝 |
| T-SAFE-04 | 无租约 | 403 |
| T-SAFE-05 | 租约过期 | 403 + 撤销许可 |
| T-SAFE-06 | 预览 token 重用 | 拒绝 |
| T-SAFE-07 | token 绑定不同 payload | 拒绝 |
| T-SAFE-08 | 重复 Idempotency-Key | 同结果 |
| T-STATE-01 | `Dispatched → Completed` 跳跃 | 拒绝（必须经过 Observed） |
| T-STATE-02 | `Acknowledged` 产生 | 断言永不发生 |
| T-STATE-03 | 超时无证据 | `Unknown` |
| T-STATE-04 | 故障后自动恢复 | 不允许 |
| T-EVT-01 | 每次命令至少 3 条事件 | Requested/Validated/最终 |
| T-EVT-02 | 导出 CSV 可被脚本复算 | 字段/单位/版本完整 |
| T-EVT-03 | 事件写入失败 | 进入 Fault，不静默丢 |

### 3.2 Mock 集成

| ID | 场景 | 期望 |
|---|---|---|
| T-MOCK-01 | 正常连接 | `Simulation`，状态流动 |
| T-MOCK-02 | 注入超时 | 命令 `Unknown`，故障 `Fault` |
| T-MOCK-03 | 注入断线 | `Disconnected`，租约撤销 |
| T-MOCK-04 | 注入空反馈 | `Unavailable`，不显示 0 |
| T-MOCK-05 | 注入 NaN | 校验拒绝，事件记录 |
| T-MOCK-06 | 注入舵机缺失 | 探针失败，门禁阻止 |
| T-MOCK-07 | 两个客户端争租约 | 仅一个成功 |
| T-MOCK-08 | 浏览器断线 | TTL 后租约撤销；重连不自动执行 |

### 3.3 E2E（Playwright）

| ID | 路径 | 期望 |
|---|---|---|
| T-UI-01 | 首次打开 | 默认只读；`SIMULATION` 可见 |
| T-UI-02 | Dashboard | 四段链路与证据可见 |
| T-UI-03 | 无租约点执行 | 禁用 + 原因 |
| T-UI-04 | 预览非法目标 | 错误码/详情可复制 |
| T-UI-05 | 预览合法 → 确认 | 状态轨迹出现；按钮防重复 |
| T-UI-06 | 停止 | 四态文案；无“急停”字样 |
| T-UI-07 | 反馈缺失 | 显示 `Unavailable`，非绿色 |
| T-UI-08 | WS 断开 | 顶部 `Disconnected`；不崩溃 |
| T-UI-09 | 事件导出 | 下载文件含版本与时间 |
| T-UI-10 | Settings 映射 | 只读 + `provisional` 徽章 |

### 3.4 硬件（人工、非 CI）

| ID | 场景 | 期望 |
|---|---|---|
| T-HW-01 | 只读体检 | 节点/设备/服务状态正确 |
| T-HW-02 | 单次位置探针 | 6 舵机结果/耗时/失败原因 |
| T-HW-03 | 拔 USB | `Fault`；不冻死 Web |
| T-HW-04 | 关闭 Driver | `ReadOnly/Disconnected`；不自动恢复 |
| T-HW-05 | 厂商 GUI 占用串口 | 明确提示占用 |
| T-HW-06 | 停止请求 | 最多 `StopDispatched/Unknown`，不谎称停止 |

---

## 4. G1 验收清单（对应 PRD §14）

- [ ] 只读 Dashboard 与诊断（Mock + Hardware 只读）
- [ ] 唯一控制权与运动门禁（Mock 可完整演示）
- [ ] 安全关节命令预览/确认（Mock 可完整演示；Hardware 默认禁用）
- [ ] 状态反馈、过期提示与异常状态（`truth` 分级）
- [ ] 命令/事件日志与导出
- [ ] 模拟故障注入通过
- [ ] Hardware Mode 明确标注“运动未实现”
- [ ] 有人看护的实机验收（需 Board 另行批准，不计入纯软件 G1）

---

## 5. 待确认问题（Board）

### 5.1 阻断项

| ID | 问题 | 建议默认 | 为什么阻断 |
|---|---|---|---|
| Q-01 | 代码放哪里？新仓库 / 现有 `ros2-industrial-arm-learning` 子目录 / `ArmPiUltra` 下 | **新仓库 `armpi-debug-studio`**，厂商代码只读引用 | 决定工程结构、CI、许可证边界 |
| Q-02 | G1 是否允许受控实机运动？ | **先只读**；完成 V-01…V-08 后，再批准“单关节受限测试” | 决定 E9 是否存在、`hardware_motion` 默认值 |
| Q-04 | 是否允许为验证 D-10 做“扭矩关闭 + 手动转动 + 读位置”测试？ | **允许**，但需机械固定、人工看护、可断电 | 决定“反馈”能否成为一等能力 |
| Q-08 | `r_joint`（夹爪）是否进入 G1？ | **不进入**；G1 只做 5 个臂关节 | 夹爪方向/范围/语义全部 Unknown |

### 5.2 非阻断项

| ID | 问题 | 建议默认 |
|---|---|---|
| Q-03 | 是否允许修补厂商 Driver（B-01…B-12）？ | G1 只包装不修补；G2 提交 patch + 回归测试 |
| Q-05 | 部署拓扑：机器人与 UI 同机还是分机？ | G1 同机（机器人主控），避免网络变量 |
| Q-06 | 局域网访问是否需要认证？ | G1 仅 localhost；局域网需 token + 明确开关 |
| Q-07 | 关节映射实机验证由谁执行、何时？ | Board 指定；G1 代码只准备工具与记录格式 |
| Q-09 | G2 是否使用厂商 MoveIt 配置？ | 先离线轨迹记录，再评估；不预设 |
| Q-10 | 哪份 SDK 是 canonical？ | ROS 2 Driver 内嵌 809 行版本 |
| Q-11 | 事件日志保留多久、是否脱敏路径/IP？ | 30 天；默认脱敏主机名与绝对路径 |
| Q-12 | UI 语言？ | 中文界面 + 英文技术标识（joint1、Command ID） |

---

## 6. 风险登记

| ID | 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|---|
| R-01 | 反馈其实是命令回显 | 中 | 高 | 先做 D-10；在验证前不把反馈用于安全判断 |
| R-02 | Driver 被一次读取冻结 | 高 | 高 | 网关隔离、限流、超时、熔断；推动修补 |
| R-03 | 厂商层静默裁剪掩盖非法目标 | 高 | 中 | 后端前置拒绝 + 审计 |
| R-04 | 串口被厂商 GUI 占用 | 中 | 高 | 启动探测 + 文档 + 进程提示 |
| R-05 | 映射/限位未验证导致越界 | 中 | 高 | 默认拒绝；受限测试；物理断电 |
| R-06 | MoveIt 初始姿态与真实不一致 | 中 | 中 | G2 前统一限位来源；不把 MoveIt 状态当真实 |
| R-07 | 页面很多但证据不足 | 中 | 中 | G1 范围冻结；每页必须有数据来源 |
| R-08 | Mock 被误认真实 | 低 | 高 | 全局 `SIMULATION` 徽章 + `truth` 字段 + 导出元数据 |

---

## 7. 下一步建议

1. **Board 先裁决 Q-01、Q-02、Q-04、Q-08**（其余可并行）。
2. 批准后按 E0 → E1 → E2 → E3 实现；**先让 Mock 端到端跑通**，再碰硬件。
3. 在 E7 之前不写任何硬件写路径；`/dispatch` 先返回 `NOT_IMPLEMENTED`。
4. 实机只读体检与 D-10 验证单独排期，必须有物理隔离与人工看护。
5. 每次交付附：变更摘要、运行说明、测试输出、已知限制、下一步建议。
