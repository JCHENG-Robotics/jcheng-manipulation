# 组织、仓库职责与初始化方案

> 组织名：`JCHENG-Robotics`（GitHub Organization）
> 首批仓库：`.github`、`jcheng-interfaces`、`jcheng-system`
> 本文件只定义结构与初始化步骤，**不执行创建**。

## 1. 组织原则

1. **契约先行。** 任何跨仓库调用先进入 `jcheng-interfaces`，再进入实现。
2. **单一权威。** 安全、控制权、硬件写入只有一套实现，位于 `jcheng-system`。
3. **独立版本。** 每个仓库可独立打标签；组合关系由锁文件表达。
4. **最小权限。** 能力仓库没有硬件设备、厂商话题和特权 DDS 域的访问权。
5. **可复现证据。** 每次发布绑定提交、契约哈希、机器人配置哈希和验收证据。
6. **基线只读。** `jcheng-manipulation v1.0.0` 不被修改、不被隐式依赖。

## 2. 仓库分层与职责

```text
L0  .github                 治理、模板、复用 CI、组织级策略
L1  jcheng-interfaces       契约：ROS IDL + HTTP/WS/配置/证据 Schema
L2  jcheng-system           特权运行时：安全、仲裁、运动、硬件桥、网关、部署、锁
L3  jcheng-manipulation     v1.0.0 冻结实验基线（参考，不参与运行时）
    jcheng-teleop           遥操作能力（意图源）
    jcheng-learning         数据/训练/策略推理（意图源）
    jcheng-mobile           移动底盘能力（意图源）
    jcheng-devices          离线设备库：设备/能力描述与校验（不参与运行时）
    jcheng-scenarios        SCEN 声明式场景库：场景 Profile 与离线校验（不参与运行时）
L4  jcheng-applications     任务编排与操作界面（网关客户端）
```

依赖方向严格单向：

```text
jcheng-applications ──HTTP/WS──> jcheng-system ──ROS──> 厂商节点
        │                              │
        └──────────┐          ┌────────┘
                   ▼          ▼
              jcheng-interfaces
                   ▲
        ┌──────────┴──────────┐
  jcheng-teleop        jcheng-learning
  jcheng-mobile        (能力层只依赖契约与系统客户端库)
```

`jcheng-devices` 是离线设备库，不在运行时依赖图中；它只读取声明式设备与能力
描述文件，不打开设备、不加入 DDS 域，也不构成控制通道。设备 I/O 与运动仍归
`jcheng-system`，每机器人的 `robot-profile.v1` 仍是运行时权威。

`jcheng-scenarios` 是 SCEN 层声明式场景库，不在运行时依赖图中。它记录场景注册表、
环境/语义/任务/能力/运行/评测 Profile，并提供仓库内 JSON Schema 校验器；不加入
DDS 域、不启动 ROS 2 节点、不打开设备、不执行任务、不发布控制或运动意图。场景
约束不能替代 `jcheng-system` 的安全机制。applications、simulation、benchmark 与
设备库只读取固定版本的声明文件；跨仓库消费前必须走 `jcheng-interfaces` 评审。

禁止的方向：

- `jcheng-system` 不在运行时导入任何 L3/L4 仓库的代码。
- L3/L4 不发布厂商话题，不打开串口，不加入硬件 DDS 域。
- `jcheng-interfaces` 不依赖系统或能力仓库。
- `.github` 不含产品代码。

## 3. 首批仓库

### 3.1 `JCHENG-Robotics/.github`

**职责：** 组织主页、社区健康文件、问题/PR 模板、复用工作流、复合 Action、
组织级策略文档。**不包含**产品代码、机器人配置、密钥。

```text
.github/
├── README.md                       # 仓库说明
├── LICENSE                         # MIT
├── profile/
│   └── README.md                   # 组织主页
├── CONTRIBUTING.md                 # 贡献流程（中文为主）
├── CODE_OF_CONDUCT.md
├── SECURITY.md                     # 漏洞报告、密钥泄露流程
├── SUPPORT.md
├── GOVERNANCE.md                   # 角色、评审、发布权限
├── CODEOWNERS.template             # 各仓库复制后生效；组织级不会自动应用
├── PULL_REQUEST_TEMPLATE.md
├── ISSUE_TEMPLATE/
│   ├── bug.yml
│   ├── feature.yml
│   ├── design.yml                  # 架构/契约变更
│   └── config.yml
├── .github/
│   └── workflows/
│       ├── reusable-python.yml
│       ├── reusable-ros-build.yml
│       ├── reusable-contract-check.yml
│       ├── reusable-integration-mock.yml
│       ├── reusable-security.yml
│       ├── reusable-release.yml
│       └── org-health.yml
├── actions/
│   ├── setup-colcon/action.yml
│   ├── contract-manifest/action.yml
│   └── evidence-check/action.yml
└── docs/
    ├── repo-taxonomy.md
    ├── branch-protection.md
    ├── release-train.md
    ├── security-baseline.md
    └── hardware-workflow.md
```

复用工作流用组织标签 `v1` 引用，例如：

```yaml
uses: JCHENG-Robotics/.github/.github/workflows/reusable-ros-build.yml@v1
```

`v1` 标签只增不改；破坏性变更发 `v2`，旧标签保留。

### 3.2 `JCHENG-Robotics/jcheng-interfaces`

**职责：** 唯一跨仓库契约。产出：

- 一个 ROS 2 包 `jcheng_interfaces`（msg/srv/action）。
- JSON Schema、OpenAPI、WebSocket 协议。
- 纯 Python 契约绑定 `jcheng_contracts`。
- TypeScript 类型（由 Schema 生成，前端使用）。
- 契约哈希、兼容性检查与文档。

**不包含：** 任何节点、任何硬件访问、任何厂商依赖、任何业务逻辑。

```text
jcheng-interfaces/
├── ros2_ws/
│   └── src/
│       └── jcheng_interfaces/
│           ├── package.xml
│           ├── CMakeLists.txt
│           ├── msg/
│           │   ├── ContractVersion.msg
│           │   ├── CapabilityManifest.msg
│           │   ├── SafetyState.msg
│           │   ├── ControlLease.msg
│           │   ├── MotionCommand.msg
│           │   ├── MotionFeedback.msg
│           │   ├── MotionResult.msg
│           │   ├── RobotState.msg
│           │   ├── JointTrajectory.msg
│           │   ├── JointTrajectoryPoint.msg
│           │   ├── BasePulse.msg
│           │   ├── GripperCommand.msg
│           │   └── TeleopCommand.msg
│           ├── srv/
│           │   ├── AcquireLease.srv
│           │   ├── RenewLease.srv
│           │   ├── ReleaseLease.srv
│           │   ├── RevokeLease.srv
│           │   ├── ValidateMotion.srv
│           │   ├── GetSafetyState.srv
│           │   ├── ResetSafety.srv
│           │   ├── ArmSystem.srv
│           │   ├── DisarmSystem.srv
│           │   └── GetCapabilities.srv
│           └── action/
│               └── ExecuteMotion.action
├── schemas/
│   ├── openapi/
│   │   ├── jcheng-gateway.v1.yaml
│   │   └── jcheng-gateway.v1.d.ts
│   ├── ws/
│   │   └── jcheng-ws.v1.json
│   ├── capability-manifest.v1.schema.json
│   ├── evidence-record.v1.schema.json
│   ├── acceptance-report.v1.schema.json
│   ├── lockfile.v1.schema.json
│   ├── robot-profile.v1.schema.json
│   ├── safety-profile.v1.schema.json
│   └── error-codes.v1.json
├── python/
│   └── jcheng_contracts/
│       ├── __init__.py
│       ├── version.py
│       ├── models.py
│       ├── errors.py
│       ├── manifest.py
│       └── validate.py
├── typescript/
│   └── src/                        # 由 OpenAPI/Schema 生成
├── tools/
│   ├── generate_manifest.py
│   ├── check_compat.py
│   ├── generate_bindings.py
│   └── render_contract_docs.py
├── tests/
│   ├── test_idl_build.py
│   ├── test_schema_parity.py
│   ├── test_manifest_hash.py
│   ├── test_compat_rules.py
│   └── test_error_codes.py
├── docs/
│   ├── CONTRACT.md
│   ├── VERSIONING.md
│   ├── MIGRATION.md
│   └── EXAMPLES.md
├── interface_manifest.json         # 生成并提交
├── VERSION
├── CHANGELOG.md
├── RELEASE.md
├── LICENSE
└── README.md
```

**为什么只用一个 ROS 包：** msg/srv/action 的类型哈希、版本和兼容性是一个整体；
拆成多个包会让“哪个组合是兼容的”变成组合爆炸。等契约稳定且构建时间成为问题时
再评估拆分。

### 3.3 `JCHENG-Robotics/jcheng-system`

**职责：** 唯一特权运行时与系统集成点。

```text
jcheng-system/
├── ros2_ws/
│   └── src/
│       ├── jcheng_bringup/          # launch、参数、启动顺序、域配置
│       ├── jcheng_safety/           # 安全监督、否决、锁存、看门狗
│       ├── jcheng_authority/        # 租约、优先级、抢占、TTL、撤销
│       ├── jcheng_motion/           # 仲裁器、意图路由、执行后端、总线确认
│       ├── jcheng_hardware/         # 厂商话题桥、唯一硬件命令发布者
│       ├── jcheng_gateway/          # HTTP/WS、认证、限流、OpenAPI 实现
│       ├── jcheng_telemetry/        # 状态缓存、实测/命令区分、证据记录
│       ├── jcheng_guard/            # 发布者白名单、域/配置审计、违规锁存
│       └── jcheng_testkit/          # Mock 硬件、场景运行器、回放工具
├── configs/
│   ├── robots/
│   │   └── armpi_ultra/
│   │       ├── profile.yaml         # 单一来源：限位/映射/工作空间/标定
│   │       ├── safety.yaml          # 由 profile 生成
│   │       ├── moveit_joint_limits.yaml
│   │       ├── mapping.yaml
│   │       └── dds/
│   │           ├── hardware.xml     # 域 41
│   │           ├── capability.xml   # 域 42
│   │           └── simulation.xml   # 域 43
│   ├── profiles/
│   │   ├── moveit.yaml
│   │   ├── step.yaml
│   │   ├── pose.yaml
│   │   ├── fast.yaml
│   │   └── base.yaml
│   └── sites/
│       └── example.yaml             # 站点差异；真实站点不入库
├── deploy/
│   ├── systemd/                     # 机器人端服务单元
│   ├── docker/                      # 运行时镜像与 compose
│   ├── pi/                          # Pi 5 部署脚本（不重启厂商容器）
│   └── rollback/
├── locks/
│   ├── ecosystem.lock.yaml
│   ├── ecosystem.repos
│   └── constraints.txt
├── e2e/
│   ├── scenarios/                   # S1–S17 场景定义
│   ├── fixtures/                    # 录制轨迹、合成状态、故障注入
│   ├── mock/                        # 纯 Mock 组合
│   ├── replay/                      # v1.0.0 录制数据回放
│   └── hardware/                    # 受监督实机验收脚本与证据模板
├── tools/
│   ├── verify_lock.py
│   ├── render_configs.py
│   ├── check_no_bypass.py
│   ├── evidence_bundle.py
│   └── compatibility_matrix.py
├── docs/
│   ├── ARCHITECTURE.md
│   ├── SAFETY.md
│   ├── AUTHORITY.md
│   ├── OPERATIONS.md
│   ├── ACCEPTANCE.md
│   ├── MIGRATION.md
│   └── adr/
├── tests/
├── VERSION
├── CHANGELOG.md
├── RELEASE.md
├── LICENSE
└── README.md
```

**命名注意：** 新系统包统一使用 `jcheng_` 前缀，避免与冻结基线的 `armpi_*`
包在同一工作区发生覆盖冲突。禁止在同一 colcon 工作区同时安装
`armpi_safety` 与 `jcheng_safety` 的同名可执行文件。

## 4. 未来仓库与接入点

| 仓库 | 角色 | 允许发布 | 禁止 |
|---|---|---|---|
| `jcheng-teleop` | 遥操作设备/界面，产生高频运动意图 | 意图话题、自身状态 | 厂商话题、`/move_action`、串口 |
| `jcheng-learning` | 数据集、训练、策略推理 | 数据集、模型、技能意图 | 直接运动命令、硬件设备 |
| `jcheng-mobile` | 底盘能力封装、未来的定位/导航 | 底盘意图 | 电机驱动话题、里程计伪造 |
| `jcheng-applications` | 任务编排、Studio/操作界面 | 网关 API 调用 | ROS 硬件话题、rclpy 直连 |
| `jcheng-devices` | 离线设备库：设备注册、型号/能力描述、离线校验 | 描述文件、校验结果、注册摘要 | 设备访问、DDS 域、运动命令、独立控制通道 |
| `jcheng-scenarios` | SCEN 声明式场景库：注册表、环境/语义/任务/能力/运行/评测 Profile、离线校验 | 声明文件、校验结果 | 运行时执行、DDS 域、设备访问、控制/运动意图 |

### 4.1 新模块接入清单

每个模块仓库必须包含：

```text
module/
├── capability.yaml          # 名称、版本、契约范围、作用域、速率、硬件访问=none
├── docs/INTEGRATION.md      # 如何被系统发现、如何申请租约、失败语义
├── tests/
│   ├── test_contract.py     # 契约握手与拒绝路径
│   └── test_no_hardware.py  # 静态扫描：无硬件话题/设备访问
├── .github/workflows/ci.yml # 调用组织复用工作流
├── VERSION
├── CHANGELOG.md
└── LICENSE
```

`capability.yaml` 示例：

```yaml
schema_version: 1
name: jcheng-teleop
version: 0.1.0
contract:
  interfaces: ">=1.0,<2.0"
  contract_hash: "sha256:..."
  system_api: ">=1.0,<2.0"
safety:
  class: motion_source
  scopes: [arm, gripper]
  requires_lease: true
  max_rate_hz: 50
  deadman_ms: 300
hardware_access: none
runtime:
  ros_domains: [42]
  publishes:
    - /jcheng/intent/teleop
  subscribes:
    - /jcheng/state
    - /jcheng/safety/state
    - /jcheng/authority/lease
```

系统在启动时校验该清单；清单声明 `hardware_access: none` 的模块如果出现在
硬件域或硬件命令话题上，`jcheng_guard` 必须锁存停止。

### 4.2 接入顺序建议

1. `jcheng-applications`：只读网关 + 任务编排，最容易证明“无硬件权限”。
2. `jcheng-teleop`：高频意图、死区、看门狗，验证租约与实时性。
3. `jcheng-mobile`：开环底盘、标定失效规则，验证不同作用域租约。
4. `jcheng-learning`：离线为主，最后接入推理节点，验证长任务与模型版本。

## 5. `jcheng-manipulation` 的处置

| 事项 | 决定 |
|---|---|
| `v1.0.0` 标签与提交 | 保持不变；不重建、不移动 |
| 代码是否迁入新仓库 | 本阶段不迁移；首批系统不依赖其运行时代码 |
| 证据目录 | 保留在原仓库；新系统验收报告引用其证据 URL，不复制 |
| 仓库是否转移到组织 | **暂不转移**；待生态 v1.0 稳定后单独批准；转移前验证标签/提交/清单哈希 |
| 未来能力演进 | 如需 v2，另开仓库或分支，不覆盖 `v1.0.0`；须重新走契约与安全评审 |
| 共享纯 Python 库 `jcheng_core` | 首批不提取；如多个新仓库需要，再建 `jcheng-common`（不属于首批） |

## 6. 仓库设置基线

### 6.1 可见性与许可证

| 仓库 | 可见性 | 许可证 | 说明 |
|---|---|---|---|
| `.github` | Public | MIT | 治理与复用 CI 需要被引用 |
| `jcheng-interfaces` | Public | MIT | 契约应可被外部工具和未来成员使用 |
| `jcheng-system` | Public | MIT | 站点差异、密钥、真实 IP 不入库 |
| 能力/应用仓库 | Public（建议） | MIT | 数据集与模型可另行许可 |
| 厂商代码 | Private | 厂商许可 | 不复制、不发布 |

如需 Apache-2.0 或私有仓库，必须在创建前决定；创建后再改会牵连 CI 与外部引用。

### 6.2 分支与标签保护

`main` 规则集：

- 禁止直接 push、force-push、删除。
- 必须通过 PR，至少 1 个批准；契约/安全目录需要该仓库的 `CODEOWNERS` 批准。
- 必需检查按仓库类型配置（见 `06-ci-cd-acceptance.md`）：
  `jcheng-interfaces` 为 `contract`/`ros-colcon`/`python-contracts`/`security`；
  `jcheng-system` 额外要求 `integration-mock` 与 `lock`。
- 要求对话解决、线性历史（squash 或 rebase）。
- 发布标签：受保护、签名、不可移动。

**注意：** GitHub 不会把 `.github` 仓库里的 `CODEOWNERS` 自动应用到其他仓库；
每个仓库必须在根目录或 `.github/` 下有自己的 `CODEOWNERS`。

### 6.3 环境与密钥

| 环境 | 用途 | 保护 |
|---|---|---|
| `release` | 签名、发布附件 | 必需评审、仅标签触发 |
| `robot-lab` | 受监督实机验收 | 必需评审、手动触发、不存长期令牌 |
| `nightly` | 跨仓库 Mock E2E | 只读令牌、无机器人网络 |

- 组织级密钥只用于发布；机器人令牌放在 `robot-lab` 环境，定期轮换。
- 禁止把网关令牌、SSH 私钥、站点 IP 写入仓库或前端构建产物。
- 所有第三方 Action 用完整 commit SHA 固定；Dependabot 负责升级 PR。

### 6.4 安全功能

- 启用 secret scanning、push protection、Dependabot alerts、dependency review。
- 启用 CodeQL（Python/TypeScript）。
- 发布附件包含 SBOM 与 SHA-256 校验和。
- 安全报告流程写入 `SECURITY.md`；密钥泄露按“先轮换、后清理历史”处理。

## 7. 初始化清单（M1 执行，需批准）

### 7.1 组织

- [ ] 创建/确认 `JCHENG-Robotics` 组织，开启成员 2FA 要求。
- [ ] 设置基础权限为 `read`；仓库创建权限仅限所有者。
- [ ] 配置组织级 ruleset 模板与安全功能。
- [ ] 建立 `@JCHENG-Robotics/maintainers` 团队（或等价角色）。

### 7.2 `.github` 仓库

- [ ] 创建 Public 仓库，MIT LICENSE。
- [ ] 提交 `profile/README.md`、社区健康文件、模板、`CODEOWNERS.template`。
- [ ] 提交 6 个可复用工作流、1 个组织健康检查与 3 个复合 Action。
- [ ] 用空仓库 PR 验证复用工作流可被调用。
- [ ] 打 `v1` 标签（不可移动）。

### 7.3 `jcheng-interfaces` 仓库

- [ ] 创建 Public 仓库，MIT LICENSE。
- [ ] 建立 ROS 包骨架（M1 仅 `ContractVersion`；M2 再补核心 IDL）。
- [ ] 接入组织 `reusable-ros-build.yml` 与 `reusable-contract-check.yml`。
- [ ] 保护 `main`，设置必需检查。
- [ ] 发布 `v0.1.0` 前完成契约评审。

### 7.4 `jcheng-system` 仓库

- [ ] 创建 Public 仓库，MIT LICENSE。
- [ ] 建立包骨架、配置目录、锁文件 Schema、E2E 目录。
- [ ] 接入组织工作流；配置 `robot-lab` 与 `release` 环境。
- [ ] 保护 `main`；添加安全目录 CODEOWNERS。
- [ ] 首个发布必须是纯 Mock，不含实机启用。

### 7.5 初始化证据

- [ ] 记录仓库创建时间、初始提交、规则集截图/API 输出。
- [ ] 记录复用工作流首次运行 URL。
- [ ] 写入 `jcheng-system/docs/evidence/org-init-2026xxxx/README.md`。
- [ ] 验证 `jcheng-manipulation` 的 `v1.0.0` 标签、提交与清单哈希未变化。

## 8. 需要确认的决策

1. 组织名是否确定为 `JCHENG-Robotics`？
2. 三个首批仓库是否都公开、都使用 MIT？
3. 是否接受“新仓库全新历史、基线仓库暂不转移”？
4. 是否同意 `jcheng_core` 暂不提取，未来需要时再建 `jcheng-common`？
5. 是否同意 `jcheng-system` 的 `main` 必需检查中包含 `integration-mock`
   （会要求系统 Mock 先可用；接口仓库不要求）？
