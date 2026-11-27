# 版本管理、依赖锁定与发布

> 目标：让 5 个以上仓库可以独立演进，同时在任何时刻都能重建一个**确定**的系统组合。

## 1. 四个版本轴

| 版本轴 | 载体 | 变更频率 | 谁批准 |
|---|---|---|---|
| 契约版本 | `jcheng-interfaces/VERSION` + `interface_manifest.json` | 低 | 接口评审 + 系统负责人 |
| 系统版本 | `jcheng-system/VERSION` | 中 | 系统负责人 + 安全评审 |
| 模块版本 | 各能力仓库 `VERSION` | 高 | 模块负责人 |
| 机器人配置版本 | `configs/robots/<robot>/profile.yaml` 的 `profile_version` | 很低 | 安全评审 + 实机证据 |

生态发布版本（release train）不是第五个代码版本，而是一个**锁文件快照**，例如
`2026.12.0`。它只引用上述四个轴的确定值。

## 2. SemVer 规则

### 2.1 契约仓库（`jcheng-interfaces`）

`0.y.z` 为初始开发期；`1.0.0` 起进入稳定期。

| 变更 | 版本位 | 说明 |
|---|---|---|
| 修改字段类型、单位、坐标系、语义；删除/重命名字段或消息；修改 action/srv 结构 | **major** | 即使字段数量不变，也视为破坏 |
| 新增消息、字段、服务、action、错误码；新增可选能力 | **minor** | 源码兼容，但 ROS 2 类型哈希会变化 |
| 仅文档、注释、生成器修复，不改变规范化契约 | **patch** | 契约哈希必须不变 |

**关键事实：ROS 2 的 message type hash 对任何字段变化敏感。** 因此“minor 新增字段”
在源码层是兼容的，在二进制/DDS 层**不是**兼容的。所有消费者必须按锁文件重建；
运行时用契约哈希做硬校验，而不是只比较 SemVer。

### 2.2 系统仓库（`jcheng-system`）

- **major**：不兼容的 API/租约/安全语义变化；删除 profile；改变停止/复位语义。
- **minor**：新增能力、新增 profile、新增可选配置、内部重构。
- **patch**：缺陷修复、文档、不影响契约的性能改进。

系统 `major` 必须声明支持的契约 `major` 范围，例如 `interfaces: ">=1.0,<2.0"`。

### 2.3 能力仓库

能力仓库独立 SemVer，但必须声明：

```yaml
contract:
  interfaces: ">=1.2,<2.0"
  contract_hash: "sha256:..."   # 构建时实际使用的契约
system_api: ">=1.0,<2.0"
```

能力模块的 `major` 变化不要求系统升级；系统升级也不要求所有模块同时升级，
但必须满足各自声明的范围。

## 3. 契约哈希（contract hash）

### 3.1 计算范围

契约哈希覆盖以下内容的**规范化表示**：

- `jcheng_interfaces` 的 `.msg` / `.srv` / `.action`：消息名、字段名、字段类型、
  数组边界、常量名与值、字段顺序。
- JSON Schema：排序键后的规范化 JSON；`$id`、`title`、`description` 之外的
  结构字段全部参与。
- OpenAPI：路径、方法、请求/响应 Schema、错误码。
- WebSocket 协议：消息类型、字段、版本字段。
- 错误码注册表 `error-codes.v1.json`。

**不参与**：注释、空白、文件顺序、生成代码、README。

### 3.2 产物

`jcheng-interfaces` 在 CI 中生成并提交 `interface_manifest.json`：

```json
{
  "schema_version": 1,
  "package": "jcheng_interfaces",
  "version": "1.0.0",
  "contract_hash": "sha256:...",
  "files": {
    "msg/MotionCommand.msg": "sha256:...",
    "srv/AcquireLease.srv": "sha256:...",
    "action/ExecuteMotion.action": "sha256:..."
  },
  "types": {
    "jcheng_interfaces/msg/MotionCommand": {
      "ros_type_hash": "..."
    }
  }
}
```

CI 必须验证：manifest 与源码一致、类型哈希可生成、与上一个发布标签的 diff 符合
SemVer 规则。

### 3.3 运行时握手

每条租约申请与每个 `ExecuteMotion` goal 都携带：

```text
ContractVersion:
  uint16 major
  uint16 minor
  uint16 patch
  string contract_hash
```

仲裁器规则：

- `major` 不同：拒绝 `CONTRACT_MAJOR_MISMATCH`。
- `contract_hash` 不同：拒绝 `CONTRACT_HASH_MISMATCH`，并在结果中返回期望哈希。
- `major`、`minor` 相同且哈希相同：接受。
- `patch` 不同但哈希相同：接受（说明只是文档/构建修复）。

这比“版本范围协商”更严格，但换来可证明的组合一致性；锁文件负责让所有组件
在发布时对齐。

## 4. 锁文件

### 4.1 位置与职责

- 系统仓库：`jcheng-system/locks/ecosystem.lock.yaml`，每个系统发布一份。
- 模块仓库：只声明兼容范围，不复制锁文件。
- 生态发布：`jcheng-system` 的 release 附件中包含完整锁文件、SBOM 与验收报告。

### 4.2 Schema 示例

```yaml
schema_version: 1
release: "2026.12.0"
channel: rc            # dev | rc | stable
created_at: "2026-12-01T08:00:00Z"

toolchain:
  ros_distro: humble
  ros_image: "ros:humble-ros-base@sha256:dfc94c85..."
  python: "3.11"
  colcon: "0.2.x"
  os: ubuntu-22.04

repos:
  jcheng-interfaces:
    url: "https://github.com/JCHENG-Robotics/jcheng-interfaces"
    version: "1.0.0"
    tag: "v1.0.0"
    commit: "<40-hex>"
    contract_hash: "sha256:..."
  jcheng-system:
    url: "https://github.com/JCHENG-Robotics/jcheng-system"
    version: "0.3.0"
    tag: "v0.3.0"
    commit: "<40-hex>"

modules:
  jcheng-teleop:
    url: "https://github.com/JCHENG-Robotics/jcheng-teleop"
    version: "0.3.0"
    tag: null                    # dev/rc 允许为空；stable 必须非空
    commit: "<40-hex>"
    layer: L3
    interfaces: ">=1.0,<2.0"
    contract_hash: "sha256:..."
    system_api: ">=1.0,<2.0"
    capability_sha256: "sha256:..."
  jcheng-applications:
    url: "https://github.com/JCHENG-Robotics/jcheng-applications"
    version: "0.1.0"
    tag: null
    commit: "<40-hex>"
    layer: L4
    interfaces: ">=1.0,<2.0"
    contract_hash: "sha256:..."
    system_api: ">=1.0,<2.0"
    capability_sha256: "sha256:..."

robot:
  name: armpi_ultra
  profile_version: "1.0.0"
  profile_sha256: "sha256:..."
  vendor_manifest_sha256: "sha256:..."

verification:
  ci_run: "<url>"
  e2e_report: "e2e/reports/2026-12-01.json"
  hardware_evidence: "evidence/2026-12-01/README.md"
```

### 4.2.1 模块收录判据

`modules` 不是“所有仓库的清单”，而是**运行时组合**。一个仓库进入 `modules`
必须同时满足：

1. 在运行时依赖图中（L3 能力或 L4 应用），能提交运动意图或消费运行时状态；
2. 有符合 `capability-manifest.v1` 的 `capability.yaml`；
3. 有可解析、不可变的 `commit`；
4. 不在 `repos:` 主体中（`jcheng-interfaces` 与 `jcheng-system` 已占位）。

因此 `jcheng-simulation`（域 43 离线被控对象）、`jcheng-ops`
（仅 HTTP 健康）、`jcheng-benchmark`、`jcheng-research` 与冻结基线
`jcheng-manipulation` 不进入 `modules`。`jcheng-learning` 在出现运行时推理
节点之前也不进入。

### 4.2.2 模块字段与粒度

下面的取值来自 2026-11-27 的实际状态，用来说明字段；`4.2` 的 1.0 示例仍是
长期目标形态。
```yaml
modules:
  jcheng-teleop:
    url: "https://github.com/JCHENG-Robotics/jcheng-teleop"
    version: "0.3.0"
    tag: null                    # dev/rc 允许为空；stable 必须非空
    commit: "<40-hex>"
    layer: L3
    interfaces: ">=0.2,<0.3"
    contract_hash: "sha256:..."
    system_api: ">=0.1,<0.2"
    capability_sha256: "sha256:<capability.yaml 规范化哈希>"
```

- `commit` 是权威标识；`version` 与 `tag` 不得替代提交。
- `capability_sha256` 钉住声明的权限面（`safety.class`、`scopes`、
  `runtime.ros_domains`），使“代码不变但权限声明变化”也触发锁文件更新。
- `contract_hash` 只在模块清单声明该字段时写入，不由锁文件臆造。

### 4.3 验证规则

`jcheng-system/tools/verify_lock.py` 必须检查：

1. 每个 `commit` 可从声明的 `url` 取得，且 `tag` 指向同一提交。
2. `jcheng-interfaces` 的 `interface_manifest.json` 哈希等于锁文件中的
   `contract_hash`。
3. 模块声明的 `interfaces` 范围包含锁文件中的契约版本。
4. 机器人配置文件的 SHA-256 匹配，且 `profile_version` 与内容一致。
5. 锁文件自身通过 `lockfile.v1.schema.json` 校验。
6. 工作区干净；任何未提交改动都使发布失败（与当前 `release_check.sh` 一致）。
7. 每个 module 的 `commit` 可从 `url` 取得；`tag` 非空时指向同一提交，
   且不得是浮动 ref。
8. `capability.yaml` 的规范化 SHA-256 等于 `capability_sha256`。
9. `interfaces` 范围包含锁内契约版本；`contract_hash` 与锁内哈希完全一致。
10. `modules` 中不得出现 L0/L1/L2 仓库；`jcheng-system` 只在 `repos`。
11. `channel: stable` 要求每个 module 的 `tag` 非空且为签名标签；
    `channel: dev|rc` 允许 `tag: null`。

### 4.4 依赖获取方式

- **不使用 Git submodule。** 避免嵌套仓库、凭据传播和“忘记更新子模块”的失败模式。
- 使用 `vcstool` 风格的 `.repos` 文件，由锁文件生成：

```bash
vcs import --input locks/ecosystem.repos ws/src
vcs export --exact ws/src > /tmp/actual.repos
diff <(normalize locks/ecosystem.repos) <(normalize /tmp/actual.repos)
```

- CI 在检出后再次验证实际提交哈希，防止 `vcs` 缓存或标签移动。
- 禁止 `main`、`master`、`latest` 等浮动 ref 进入锁文件。

## 5. Python 与系统依赖

| 层 | 策略 |
|---|---|
| 纯 Python 库（`jcheng_contracts`；未来的 `jcheng-common`） | `pyproject.toml` 声明范围；发布 wheel；CI 在 3.10/3.11/3.12 测试 |
| 应用（gateway/backend/系统工具） | 精确 pin；`pip-compile --generate-hashes` 生成 `constraints.txt` |
| ROS 包 | `package.xml` 声明依赖；colcon 在固定镜像内构建 |
| 系统库 | 第一阶段：固定镜像摘要 + 显式 apt 列表；第二阶段：发布带 SBOM 的运行时镜像 |
| 厂商代码 | 不发布、不复制；仅用私有 `vendor_baseline_manifest.json` 的哈希锁定 |

**诚实说明：** 当前 `ros-ci.yml` 的 apt 安装未逐包钉版本，因此“可复现”目前是
“固定镜像 + 显式包列表 + 提交锁定”，不是 apt 层级的逐字节可复现。第二阶段
通过发布运行时镜像和 SBOM 补齐；在补齐前不得宣称更高等级。

## 6. 机器人配置版本

机器人配置是代码之外的“数据契约”，必须独立版本化：

```yaml
schema_version: 1
robot: armpi_ultra
profile_version: 1.0.0
baseline:
  repo: jcheng-manipulation
  tag: v1.0.0
  commit: b3411e4
  source_hashes:
    safety_limits: "sha256:..."
    joint_mapping: "sha256:..."
    xacro: "sha256:..."
```

规则：

- 任何限位、映射、工作空间、标定常量、伺服 ID、夹爪范围的变化都必须提升
  `profile_version`，并附实机证据。
- 配置由 `tools/render_configs.py` 生成安全、MoveIt、网关、应用所需文件；
  生成物禁止手工编辑，CI 检查生成物与源一致。
- 配置哈希进入锁文件；运行时握手返回 `profile_hash`，不匹配时安全节点拒绝
  进入 `ARMED`。

## 7. 发布流程

### 7.1 契约发布

1. PR 修改 IDL/Schema，更新 `CHANGELOG.md` 与 `VERSION`。
2. CI 运行 IDL 构建、Schema 校验、契约 diff、生成 manifest。
3. 评审通过后打签名标签 `vX.Y.Z`。
4. 发布附件：源码包、`interface_manifest.json`、Schema bundle、变更说明。

### 7.2 系统发布

1. 更新 `locks/ecosystem.lock.yaml` 指向新的契约提交。
2. 在 Mock 与回放 E2E 上验证。
3. 受监督实机验收（若涉及运动路径）。
4. 更新 `VERSION`、`CHANGELOG.md`、`RELEASE.md`。
5. 打签名标签；发布源码包、锁文件、SBOM、校验和、验收报告。
6. 可选：发布 `ghcr.io/jcheng-robotics/jcheng-runtime:<version>` 镜像。

### 7.3 模块发布

1. 声明并验证契约范围。
2. 通过模块 CI 与系统 Mock E2E。
3. 在系统锁文件中登记（由系统发布统一收录）。
4. 打标签；不要求每个模块都有独立硬件验收，但首次接入必须有一次受监督验收。

### 7.4 渠道

| 渠道 | 用途 | 签名 | 允许实机 |
|---|---|---|---|
| `dev` | 每日构建、接口试验 | 否 | 否 |
| `rc` | 候选发布、验收 | 是 | 仅受监督验收 |
| `stable` | 生产基线 | 是 | 是，按运行手册 |

标签不可移动；禁止 force-push 已发布标签。`main` 受保护，发布从标签构建，
不从分支头构建。

## 8. 兼容性与弃用

- 契约 `major` 的兼容窗口：HTTP/WS 的**只读** API 至少支持当前与上一个 `major`
  一个发布周期；ROS IDL 的运动写入路径只接受精确 `contract_hash`，不保留双版本。
  需要跨 major 读取时，通过网关的版本化 HTTP 适配层实现，不把两套 IDL 装进
  同一个运动平面。
- 弃用流程：minor 中标记 `deprecated`（文档 + 运行时 warning），下一个 major 删除。
- 错误码只增不改语义；删除错误码属于 major。
- 配置字段：新增可选字段为 minor；修改默认值若改变安全行为，视为 major。

## 9. 回滚

- 每个 `stable` 发布保留：锁文件、源码标签、运行时镜像摘要、机器人配置、证据包。
- 回滚单位是**锁文件快照**，不是单个仓库。
- 机器人端回滚必须同时回滚网关、安全配置与系统服务单元；禁止只回滚 UI。
- `jcheng-manipulation v1.0.0` 是最终兜底参考，但不得作为新系统的运行时依赖。

## 10. 需要确认的决策

1. 契约哈希不匹配时**硬拒绝**（而不是降级警告），是否同意？
2. 首批是否发布运行时容器镜像，还是先只做源码构建？
3. 生态发布采用事件驱动（有契约变化才发）还是固定月度节奏？
4. 机器人配置变更是否要求至少一次受监督实机证据才能提升版本？
