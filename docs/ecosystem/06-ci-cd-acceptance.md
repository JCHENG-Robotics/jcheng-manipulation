# CI/CD 与端到端验收

> 目标：让“通过”有可执行定义，让“不能绕过保护层”在 CI 与验收中可验证。

## 1. 原则

1. **分层测试。** 契约、单元、Mock 集成、Mock E2E、回放、实机，各自独立门槛。
2. **复用工作流。** 组织级 `.github` 提供唯一 CI 实现，仓库只声明参数。
3. **最小权限。** PR 工作流不接触机器人网络、不持有长期令牌；发布与实机用受保护环境。
4. **确定性。** 固定镜像、固定依赖、固定提交；失败可复现。
5. **证据化。** 每次实机验收产出 `evidence-record` 与验收报告。
6. **失败关闭。** 安全检查失败、契约不匹配、证据缺失都阻止发布。

## 2. 测试层级

| 层级 | 名称 | 环境 | 是否接触实机 | 典型耗时 | 触发 |
|---|---|---|---|---|---|
| L0 | 契约/静态 | GitHub runner | 否 | < 2 min | 每次 PR |
| L1 | 单元 | runner / 容器 | 否 | < 5 min | 每次 PR |
| L2 | Mock 集成 | 域 43 | 否 | < 10 min | 每次 PR |
| L3 | Mock E2E | 域 43 全栈 | 否 | < 15 min | PR + nightly |
| L4 | 回放 | 域 43 + 录制数据 | 否 | < 15 min | nightly + 发布 |
| L5 | 实机 | 域 0/41/42 | 是 | 30–90 min | 受监督、手动 |

### 2.1 各层内容

- **L0**：IDL 构建、Schema 校验、契约哈希、SemVer diff、硬件原语扫描、
  能力清单白名单、Action pin、许可证/依赖审查。
- **L1**：安全策略、状态机、租约、限流、错误码、配置生成、锁文件校验。
- **L2**：单进程/单域集成：仲裁器 + 安全 + Mock 硬件桥；服务/action 往返。
- **L3**：全栈 Mock：网关 HTTP/WS -> 租约 -> 仲裁 -> 安全 -> Mock 厂商；
  场景化故障注入。
- **L4**：用 v1.0.0 `evidence/` 中的轨迹/命令/状态回放，比较校验决定与结果；
  不驱动真实硬件。
- **L5**：只读预检、单关节、夹爪、停止、G8 复验；受监督并记录证据。

## 3. 组织复用工作流

位于 `JCHENG-Robotics/.github/.github/workflows/`，用 `@v1` 引用。

| 工作流 | 输入 | 主要步骤 | 输出 | 状态（2026-11-27） |
|---|---|---|---|---|
| `reusable-python.yml` | `python-version`、`working-directory`、`test-command` | setup-python、缓存、ruff、pytest、coverage | 测试报告、覆盖率 | 骨架 |
| `reusable-ros-build.yml` | `distro`、`image`、`packages`、`test-command` | 容器、apt 依赖、colcon build/test、test-result | colcon 日志、测试结果 | 骨架 |
| `reusable-contract-check.yml` | `base-ref`、`interfaces-path`、`run-tests` | manifest、Schema、硬件原语、pytest、VERSION-bump guard | 检查日志 | 已实现 |
| `reusable-integration-mock.yml` | `scenario-set`、`system-ref` | 启动 Mock 栈、运行场景、收集指标 | E2E 报告、日志 | 骨架 |
| `reusable-security.yml` | `forbid-license` | secret scan、Action pin check、LICENSE hold | 检查日志 | 已实现（CodeQL/dependency review 待接） |
| `reusable-release.yml` | `version`、`lockfile`、`artifacts`、`draft` | 版本/tag 校验、SHA256SUMS、draft release | 发布附件 | 已实现（SBOM/签名待 M8） |

**2026-11-27 实现边界：** 三个“已实现”工作流只使用 `actions/checkout`
（完整 SHA）与 shell，不引入新的第三方 Action；它们在本机对
`jcheng-interfaces` 与 `.github` 跑通了相同命令。`reusable-release` 默认只创建
draft，发布仍需人工点击；SBOM、签名与 dependency review 不在本轮声称范围内。

要求：

- 第三方 Action 用完整 commit SHA；复用工作流自身用组织标签 `v1`。
- 工作流默认 `permissions: contents: read`；发布作业显式提升。
- 所有工作流设置 `timeout-minutes`；禁止无限等待。
- 失败时上传日志；日志中不得包含令牌。

## 4. 各仓库 CI

### 4.1 `.github`

- `org-health.yml`：每周检查仓库规则、分支保护、secret scanning、Action pin 过期。
- 复用工作流的“自检”仓库：用示例仓库跑通 Python/ROS/Contract 三条。

### 4.2 `jcheng-interfaces`

必需检查：

- `ros-colcon`（Humble + Jazzy 矩阵）。
- `contract`（manifest、Schema、parity、SemVer diff）。
- `python-contracts`（3.10/3.11/3.12）。
- `security`。

发布：

- 标签触发 `release`：源码包、manifest、Schema bundle、变更说明。

### 4.3 `jcheng-system`

必需检查：

- `python-unit`（safety/authority/motion/gateway）。
- `ros-colcon`（Humble；系统包全量）。
- `contract`（使用锁文件中的 interfaces 提交）。
- `integration-mock`（S1–S17 的 L2/L3 子集）。
- `security`。
- `lock`（锁文件可解析、提交可达、哈希匹配）。

Nightly：

- 按锁文件检出全部仓库，构建 + L3 + L4。
- 生成 `e2e-report.json`；失败自动开 issue 并附日志。

### 4.4 能力/应用仓库

- `contract`：握手、拒绝路径、能力清单。
- `no-hardware`：静态扫描禁止话题/设备/import。
- `integration-mock`：在系统 Mock 栈中运行自身场景。
- `security`。
- 首次接入必须有一次 L5 证据（不要求在 CI 中运行）。

## 5. E2E 场景目录

场景定义位于 `jcheng-system/e2e/scenarios/<id>.yaml`，包含前置、步骤、期望、
清理与指标。首批：

| ID | 场景 | 期望 | 层级 |
|---|---|---|---|
| S1 | 契约握手成功 | 接受；返回契约版本/哈希 | L2 |
| S2 | 契约 major/hash 不匹配 | 拒绝 `CONTRACT_MISMATCH`；无运动 | L2 |
| S3 | 租约申请/续租/释放 | 状态一致；审计完整 | L2 |
| S4 | 租约争用 | 第二持有者被拒；高优先级可抢占 | L2/L3 |
| S5 | 租约过期在飞 | 中止；停止；`LEASE_EXPIRED` | L3 |
| S6 | 安全停止锁存 | 新命令全拒；复位后可重新使能 | L3/L5 |
| S7 | 状态过期/缺关节 | 拒绝；锁存；不执行 | L2 |
| S8 | 越限/工作空间/桌面违规 | 否决并返回具体 code | L2/L3 |
| S9 | 未授权发布者 | guard 锁存；命令拒绝 | L3 |
| S10 | 无租约执行 | 拒绝 `NO_LEASE`；图形中无硬件发布 | L2/L3 |
| S11 | 幂等重放 | 返回同一结果；不重复运动 | L2 |
| S12 | 总线读失败 | `UNKNOWN` 锁存；不可自动清除 | L3/L5 |
| S13 | v1.0.0 轨迹回放 | 校验决定一致；结果在容差内 | L4 |
| S14 | 底盘脉冲与标定失效 | 结果标记 `calibration_invalidated`；后续抓取拒绝 | L3/L5 |
| S15 | 网关认证/限流/重连 | 401/429；租约到期停止；重连恢复只读 | L3 |
| S16 | 配置/profile 哈希不匹配 | 拒绝使能 | L2/L5 |
| S17 | MoveIt 只规划 | 调用 `/move_action` 带 `plan_only=true`；无执行 | L2/L5 |

### 5.1 S9 的关键断言

1. 启动 Mock 厂商和一个“违规发布者”，向 `/ros_robot_controller/bus_servo/set_position`
   发布一条命令。
2. `jcheng_guard` 在 100 ms 内检测到非白名单发布者并锁存停止。
3. 仲裁器拒绝后续命令；安全状态为 `STOP_LATCHED`。
4. 证据包包含违规发布者的节点名/GID/时间戳。

**注意：** guard 是检测层，不是预防层；单条命令可能在检测前发出。真正的预防是
域隔离、容器权限与 SROS2。S9 验证的是“违规不会持续/不会被当作正常”，
不能替代第 1/3/4 层。

### 5.2 S10 的关键断言

1. 能力模块只加入域 42，未申请租约。
2. 尝试 `ExecuteMotion` -> `NO_LEASE`。
3. Mock 厂商话题的发布者集合只包含 `jcheng_hardware`。
4. 图形审计报告为零非授权发布者。

### 5.3 S13 回放数据

来源（只读引用，不复制）：

- `evidence/g8_acceptance_20261123/` 的轨迹/命令/结果。
- `evidence/studio_g8_fast/`、`evidence/studio_g81/` 的快速路径与底盘脉冲。
- `evidence/phase3/...` 的重复性与放置数据（用于容差判定）。

回放比较：

- 对同一轨迹，安全校验的 `approved/violations` 必须一致（允许时间戳字段差异）。
- 执行结果在 Mock 中为模拟值；只验证决策与状态机，不比较物理落点。
- 物理落点比较只在 L5 进行。

## 6. 实机验收协议（L5）

### 6.1 前置检查

- 物理：工作区清空、机械臂有支撑、无人员接触、电源/电池正常。
- 链路：有线 10.42.0.x、DDS 发现、时间同步（偏移在阈值内）。
- 栈：厂商节点已运行（不重启容器）；`jcheng_hardware` 只读；安全节点
  `STOP_LATCHED=false`；总线实测与缓存已对齐。
- 版本：锁文件、契约哈希、profile 哈希与部署一致。
- 证据：录屏/日志目录就绪，操作员与监督人确认。

### 6.2 步骤

1. **只读**：读取关节/夹爪/电池/相机；确认 `truth=measured`。
2. **使能**：检查清单 + 确认短语；确认 `ARMED_IDLE`。
3. **单关节小步**：0.05 rad，`profile=step`；总线 ±8 脉冲确认。
4. **停止**：触发软件停止；测量锁存与保持延迟；确认新命令被拒。
5. **复位/重新使能**：确认需要重新使能。
6. **夹爪**：开/合各一次；总线确认；不夹物体。
7. **租约争用**：两个客户端；确认抢占/拒绝。
8. **G8 复验**：定位 -> 抓取 -> 抬升 -> 放回；每阶段总线确认。
9. **异常**：可选注入过期状态/总线错误（仅在 Mock 或受控条件）。
10. **回滚演练**：停止新系统，恢复旧网关只读；确认可回到基线路径。

### 6.3 通过标准

- 所有运动命令 `COMPLETED` 且 `bus_verified=true`（底盘为显式 `open_loop`）。
- 无 `UNKNOWN`；若有，必须停止并判定失败。
- 软件停止锁存传播 ≤100 ms（日志时间戳测量）。
- G8 落点在基线容差内（基线快路径 2.27 mm；建议门槛 ≤5 mm，需确认）。
- 安全状态全程无未解释违规。
- 证据包完整，可被 `evidence-record` Schema 校验。

### 6.4 中止条件

- 非预期运动、异常噪声、碰撞风险。
- 总线读数与命令不一致且无法解释。
- 安全锁存无法复位。
- 操作员或监督人要求停止。
- 任何人员进入工作区。

## 7. 证据包格式

目录：

```text
evidence/<date>_<session>/
├── README.md                 # 目的、版本、操作员、结论、限制
├── acceptance-report.json    # 机器可读，符合 schema
├── commands.jsonl            # 请求/校验/结果
├── safety.jsonl              # 安全状态变化
├── bus_reads.jsonl           # 总线实测
├── metrics.json              # 延迟/成功率/落点
├── logs/                     # 进程日志（脱敏）
└── checksums.sha256          # 所有文件
```

`acceptance-report.json` 必须包含：

- 锁文件中的契约/系统/模块版本。
- 操作员与监督人。
- 场景 ID、步骤、结果。
- 指标与门槛。
- 失败/偏差与处置。
- 残余风险与限制。

CI 只验证证据包结构与校验和，不实时控制实机。

## 8. 验收指标（建议门槛）

| ID | 指标 | 门槛 | 层级 |
|---|---|---|---|
| G1 | 非授权硬件发布者 | 0 | L3/L5 |
| G2 | 契约不匹配拒绝率 | 100% | L2 |
| G3 | 无租约/越权拒绝率 | 100% | L2/L3 |
| G4 | 越限/工作空间/桌面否决率 | 100% | L2/L3 |
| G5 | 软件停止锁存传播 | ≤100 ms | L3/L5 |
| G6 | 无总线确认的 `COMPLETED` | 0 | L2/L5 |
| G7 | 回放校验决定一致率 | 100% | L4 |
| G8 | G8 落点 | ≤5 mm（待定） | L5 |
| G9 | `UNKNOWN` 结果 | 0（验收运行） | L5 |
| G10 | 锁文件重建一致 | 100% | L0/L4 |
| G11 | 安全/仲裁/运动分支覆盖率 | ≥90% | L1 |
| G12 | 能力模块静态无硬件访问 | 100% | L0 |

**注意：** G5/G8 是软件与任务指标，不是功能安全认证；对外表述必须保留
“受监督、非安全认证”的限定。

## 9. 安全与供应链

- PR 工作流：`permissions: contents: read`；不使用机器人环境密钥。
- 发布：受保护环境 `release`；标签触发；签名标签。
- 实机：受保护环境 `robot-lab`；手动触发；操作员审批；不存长期令牌。
- 依赖：Dependabot + dependency review；新依赖需说明用途与许可证。
- Action：完整 SHA；组织复用工作流 `@v1`。
- 产物：SBOM（CycloneDX）、SHA-256、GitHub attestation（可选）。
- 日志：脱敏；禁止令牌、私钥、站点 IP。

## 10. 失败处理

- **Flaky 测试**：标记并隔离；连续 3 次失败自动开 issue；不得用重试掩盖
  安全相关失败。
- **Nightly 失败**：阻断发布；保留产物 30 天。
- **实机失败**：立即停止；证据包标记 `fail`；未完成根因分析前不得重跑同一场景。
- **回滚**：恢复上一锁文件与部署；旧网关只读；记录回滚证据。

## 11. 需要确认的决策

1. 是否接受 L0–L5 六级测试与 S1–S17 场景集？
2. 是否接受“实机验收不在 CI 中实时控制，CI 只校验证据包”？
3. G5（≤100 ms）与 G8（≤5 mm）门槛是否合适？
4. 覆盖率门槛 90% 是否过高/过低？
5. Nightly 是否只在 GitHub 托管 runner 上跑 Mock，不接机器人网络？
