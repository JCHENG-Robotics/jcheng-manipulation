# v0.2.0 生态对齐包

> 状态：**审计完成；文档修正与 2026-11-27 验收快照就绪；4 项决策待确认。**
> 方法：12 个仓库先 `git fetch --prune`，再读取 `origin/main`、标签与 GitHub API；
> 不以工作区分支的过期内容为依据。
> 边界：只读审计 + 文档；不改运行时代码、不迁移 `jcheng-perception`、不改
> `jcheng-manipulation v1.0.0`、不连接机器人、不发送运动命令。
> 结论：契约 v0.2.0 已在 **7 个消费仓库**落地；`jcheng-perception` 是唯一未迁移的
> L3 消费方；组织级复用 CI 仍是 M1 骨架；锁文件已按 v1 字段表达首批 3 个模块
> （分支待合并）。

## 1. 这份对齐包解决什么问题

R5 停止确认契约（`StopResponse.confirmation`）以 v0.2.0 发布后，生态从“单契约
v0.1.0 + Mock 系统”进入了“多仓库各自锁定契约”的阶段。设计文档 02–07 描述的是
三仓库、单版本的初始形态；本文回答三个由此产生的问题：

1. 14 个仓库里，谁真正消费了契约，谁没有，证据在哪个文件？
2. `locks/ecosystem.lock.yaml` 要怎样表达“模块组合”，才能支撑 M8 的 lockfile freeze？
3. v0.2.0 之后哪些“现行声明”已经与事实不符，哪些只是历史记录、不能改？

本文不重新定义仓库职责；仓库分层以
`JCHENG-Robotics/.github` 的 `docs/repo-taxonomy.md` 为准。

## 2. 核实结果（2026-11-27）

### 2.1 契约采用矩阵

判定依据：`capability.yaml` 的 `contract:` 块、`jcheng-system/configs/contract.yaml`
与 `locks/ecosystem.lock.yaml`、发布标签。

| 仓库 | 层 | `origin/main` | 模块版本 | 契约声明 | `contract_hash` | 结论 |
|---|---|---|---|---|---|---|
| `.github` | L0 | `16ef3f7` | — | — | — | 治理，不消费 |
| `jcheng-interfaces` | L1 | `f27114a` | `0.2.0` | 发布源（`v0.2.0` → `b03a054`） | `afca22…` | 契约源 |
| `jcheng-system` | L2 | `9cd1c8f` | `0.3.0-dev0` | `0.2.0` / `v0.2.0` / `b03a054` | `afca22…` | 已 pin（PR #34） |
| `jcheng-manipulation` | L3 冻结 | `d63f6d1` | `1.0.0` | — | — | 冻结基线，不消费 |
| `jcheng-mobile` | L3 | `0972e4b` | `0.1.0` | `>=0.2,<0.3` | `afca22…` | 已采用 |
| `jcheng-perception` | L3 | `b8cd2db` | `0.1.0` | `>=0.1,<0.2`（`ROADMAP.md:85`） | 无 `contract` 块 | **未迁移** |
| `jcheng-learning` | L3 | `25eb184` | `0.2.0` | `>=0.2,<0.3` | `afca22…` | 已采用（offline） |
| `jcheng-simulation` | SIM | `40c250d` | `0.1.0` | `>=0.2,<0.3` | `afca22…` | 已采用（域 43 偏差已声明） |
| `jcheng-teleop` | L3 | `eca8223` | `0.3.0` | `>=0.2,<0.3` | `afca22…` | 已采用（`class: offline`） |
| `jcheng-studio` | L4 | `1eade61` | `0.7.0` | `>=0.2,<0.3` | `afca22…` | 已采用 |
| `jcheng-applications` | L4 | `0bd57de` | `0.1.0` | `>=0.2,<0.3` | `afca22…` | 已采用 |
| `jcheng-ops` | OPS | `9196466` | `0.1.0` | — | — | 仅 HTTP 健康，不适用 |
| `jcheng-benchmark` | BENCH | `bfaeac9` | `0.1.0` | — | — | 离线，不适用 |
| `jcheng-research` | RESEARCH | `58858ec` | `0.1.0` | — | — | 调研，不适用 |

计数口径：`jcheng-interfaces` 是发布源；运行时/消费仓库共 8 个
（system、mobile、perception、learning、simulation、teleop、studio、applications），
其中 **7 个已采用 v0.2.0，1 个（perception）未迁移**。

`jcheng-mobile` 的模块版本仍是 `0.1.0`，但契约已声明 `>=0.2,<0.3`；**模块版本与
契约版本是两个轴**，不能用 `VERSION` 判断契约采用状态。

perception 的情况不同：它的代码没有导入 `jcheng_interfaces`，其依赖是声明性的
（消息包尚未晋升到契约仓库），因此处理方式不是“赶快追版本”，见 §7.1。

### 2.2 已核实的事实

1. `v0.2.0` 标签已签名并通过 GitHub 验证（`verified=true, reason=valid`），
   契约哈希 `sha256:afca22a42f7163e0351d185d5a8577b0b51f2f3da4b92278f161a4a7d2c35b1c`。
2. `jcheng-system` 的 `configs/contract.yaml` 与
   `locks/ecosystem.lock.yaml` 均指向 `0.2.0 / v0.2.0 / b03a054`（PR #34 → `9cd1c8f`）。
3. `jcheng-perception` 仍是早期清单格式：没有 `schema_version`、没有
   `contract:` 块、没有 `safety.class`，不符合 `capability-manifest.v1`。
4. 全部 12 个产品仓库各自维护一份 `.github/workflows/ci.yml`，
   **没有仓库引用** `.github` 的 `reusable-*` 工作流。
5. `.github` 的 7 个复用工作流中，**6 个仍是 M1 骨架**
   （`reusable-python`、`reusable-ros-build`、`reusable-contract-check`、
   `reusable-integration-mock`、`reusable-security`、`reusable-release`），
   只有 `org-health.yml` 是实际执行的。
6. 工作区已从 `/home/jerichocheng/jcheng` 移到
   `/home/jerichocheng/01_Projects/jcheng-robotics`；旧路径的兼容符号链接
   **当前不存在**。

## 3. 锁文件 `modules` 拓扑（设计提案）

### 3.1 进锁判据

一个仓库进入 `modules` 必须同时满足：

1. 在运行时依赖图中（L3 能力或 L4 应用），能提交运动意图或消费运行时状态；
2. 有符合 `capability-manifest.v1` 的 `capability.yaml`；
3. 有可解析、不可变的 `commit`；
4. 不在 `repos:` 主体中（`jcheng-interfaces` 与 `jcheng-system` 已占位）。

按此判据：

| 处置 | 仓库 | 理由 |
|---|---|---|
| 首批纳入 | `jcheng-teleop`、`jcheng-applications`、`jcheng-studio` | 与 ArmPi 运行时组合相关、且已采用 v0.2.0 契约 |
| 迁移后纳入 | `jcheng-perception` | 先满足 3.1 第 2 条 |
| 暂不纳入 | `jcheng-mobile` | 移动平台能力，尚未进入 ArmPi 运行时组合 |
| 暂不纳入 | `jcheng-learning` | 当前离线，无运行时节点；M3 出现推理节点后再纳入 |
| 不纳入 | `jcheng-simulation` | 域 43 离线被控对象，不在运行时组合中 |
| 不纳入 | `jcheng-ops`、`jcheng-benchmark`、`jcheng-research` | 仅 HTTP / 离线 / 调研 |
| 不纳入 | `jcheng-manipulation` | 冻结基线，不是运行时依赖 |

### 3.2 字段与粒度（两阶段）

**约束：`lockfile.v1.schema.json` 计入契约哈希。** 它位于
`jcheng-interfaces/schemas/`，修改会改变所有消费者运行时握手的哈希；
因此富字段必须与一次契约发布（v0.3.0）一起做。

**第一阶段（当前 v1，已实现）** 只允许三个字段：

```yaml
modules:
  jcheng-teleop:
    version: "0.3.0"
    commit: "eca822399f03c5ca6228306ffc931d764e766941"
    interfaces: ">=0.2,<0.3"
```

`verify_lock.py` 校验提交格式与范围；提供 `--module-root` 时，从本地检出的
pinned commit 读取 `VERSION` 与 `capability.yaml`，核对版本、契约哈希与范围。
v1 没有 `url`/`tag`，所以远程可达性检查留到第二阶段。

**第二阶段（契约 v0.3.0 / lockfile v2）** 增加
`url`、`tag`、`layer`、`contract_hash`、`system_api`、`capability_sha256`：

```yaml
modules:
  jcheng-teleop:
    url: "https://github.com/JCHENG-Robotics/jcheng-teleop"
    version: "0.3.0"
    tag: null                    # dev/rc 允许为空；stable 必须非空
    commit: "<40-hex>"
    layer: L3
    interfaces: ">=0.2,<0.3"
    contract_hash: "sha256:afca22..."
    system_api: ">=0.1,<0.2"
    capability_sha256: "sha256:<capability.yaml 规范化哈希>"
```

规则（第二阶段字段）：

- **`commit` 是权威标识**；`version` 与 `tag` 只是可读性与发布语义，不得替代提交。
- **禁止浮动 ref**（`main`、`latest` 等）进入锁文件；`tag` 非空时必须指向同一提交。
- **`capability_sha256`** 钉住“声明的权限面”（`safety.class`、`scopes`、
  `runtime.ros_domains`）。代码未变但权限声明变化，也必须改变锁文件。
- `contract_hash` 只在模块的 `capability.yaml` 声明了该字段时写入；
  未声明（如 perception 当前状态）不得由锁文件臆造。

### 3.3 验证规则（接 `05` §4.3）

第一阶段已实现：

7. `interfaces` 范围包含锁内契约版本。
8. `modules` 中不得出现 L0/L1/L2 仓库；`jcheng-system` 只在 `repos`。
9. 提供 `--module-root` 时，pinned commit 必须存在，且该提交的 `VERSION`、
   `capability.yaml` 的 `contract_hash` 与范围必须与锁一致，`system_api`
   范围必须包含 `configs/api.yaml` 的 `api_version`。

第二阶段（需要 lockfile v2）：

10. 每个 module 的 `commit` 可从声明的 `url` 取得；`tag` 非空时指向同一提交。
11. `capability.yaml` 的规范化 SHA-256 等于 `capability_sha256`。
12. `channel: stable` 要求每个 module 的 `tag` 非空且为签名标签。

### 3.4 发布标识

锁文件原先写 `release: "2026.11.0-m3"`，与 `05` §4.2 的 `2026.12.0` 约定不一致，
且把里程碑号放进了发布标识。本轮已改为：

- `release: "2026.11.1"`，通道仍用 `channel: dev`；
- 里程碑号只出现在 `CHANGELOG.md`、执行记录与 `verification.e2e_report` 中；
- 契约升级（0.1→0.2）属于需要重建的组合变化，发布标识至少提升 `PATCH`，
  并在 `CHANGELOG.md` 说明原因。

## 4. 漂移登记

### 4.1 现行声明（需修）

| 位置 | 现状 | 应改为 | 处置 |
|---|---|---|---|
| `jcheng-system/docs/OPERATIONS.md:56` | “`--remote` 会检查 interfaces v0.1.0 标签提交” | 锁文件声明的标签（当前 v0.2.0） | 本对齐分支修正 |
| `.github/ACCEPTANCE.md` §2–§3 | 契约仍写 v0.1.0 | v0.2.0 + 采用矩阵 + 两项失败门禁 | 本对齐分支修正 |
| `.github/ROADMAP.md`（修改前第 3、12、74–75 行） | 契约与依赖仍写 v0.1.0 / 旧哈希 | v0.2.0 / `afca22…` | 本对齐分支修正 |
| `.github/docs/repo-taxonomy.md`（修改前第 13、15、26–29 行） | teleop 写 v0.1.0“仅准入”，applications 写“只读传输” | teleop v0.3.0 离线核心；applications 受监督 mock 网关意图 | 本对齐分支修正 |
| 工作区 `AGENTS.md` §5 | 契约 v0.1.0 / 旧哈希 | v0.2.0 / `afca22…` | 待确认（运行指令，不擅自改） |
| 工作区 `AGENTS.md` §3 | applications/teleop/learning 仍描述为“无 HTTP/WS 客户端”等旧状态 | 按各仓 `capability.yaml` 与 ACCEPTANCE 更新 | 待确认 |
| 工作区 `AGENTS.md` §2/§12、bootstrap `README.md`、`GIT-WORKSPACE-POLICY.md`、`scripts/git_preflight.sh` | 仍以 `~/jcheng` 为根 | `/home/jerichocheng/01_Projects/jcheng-robotics`（或 `JCHENG_ROOT`） | 待确认 |

### 4.2 可执行常量（代码，单独 PR）

| 位置 | 现状 | 影响 |
|---|---|---|
| `jcheng-system/tools/teleop_sim_probe.py:17` | `CONTRACT` 硬编码旧哈希 | 探针与 v0.2.0 运行时握手会失败 |
| `jcheng-system/tools/teleop_stream_probe.py:22` | 同上 | 同上 |
| `jcheng-teleop/ros2_ws/src/jcheng_teleop_ros/jcheng_teleop_ros/publisher.py:45` | `CONTRACT_HASH` 旧哈希 | test-only 发布器广播错误哈希 |

这些是代码改动，需走正常 PR 与 CI；本文只登记，不修改。

### 4.3 历史记录（不改）

以下内容记录的是当时事实，修改会破坏证据链：

- `jcheng-applications/evidence/20261126_*` 中的旧哈希；
- `jcheng-interfaces/CHANGELOG.md` 的 v0.1.0 条目；
- `jcheng-studio/evidence/20261126_gateway_ws/integration.txt`；
- `jcheng-research/ROADMAP.md` 与 `experiments/20261125-vp-vla-armpi/design.md`；
- `jcheng-learning/docs/m1/04-DATA-CONTRACT.md` 中的 JSON 样例；
- `jcheng-system` 的 `docs/M3-SCOPE.md`、`M4-SCOPE.md`、`M5-SCOPE.md`，
  以及 `ACCEPTANCE.md` 的 M3 小节（“interfaces v0.1.0”是当时的验收结果）。

### 4.4 能力清单缺口

`jcheng-perception` 是唯一不满足“每个能力仓库都有
`capability-manifest.v1` + `contract_hash`”的 L3 仓库。它的 `capability.yaml`
是早期状态清单（`runtime_roles`、`verification`），没有 `contract` 块，
也没有 `safety.class`。这不是笔误，而是仓库准入时尚未迁移；处理方式见 §7.1。

## 5. 组织级 CI 复用偏差

设计 `06-ci-cd-acceptance.md` §3 规定复用工作流位于
`JCHENG-Robotics/.github/.github/workflows/`，仓库以 `@v1` 引用。现实是：

- 6/7 复用工作流仍是 M1 骨架（文件内写明 “M2 will add…”）；
- 12/12 产品仓库各自维护 `ci.yml`，0 个引用复用工作流；
- 2026-11-27 的 merge-queue 引导必须逐个仓库改 11 个 PR，正是重复 CI 的
  直接成本。

影响：

1. 契约哈希、硬件原语扫描、SemVer diff 没有组织级唯一实现，靠各仓自行维护；
2. 新增仓库的准入检查无法由组织统一执行；
3. 同一门禁在不同仓库可能已经漂移，且没有对比机制。

## 6. 2026-11-27 组织验收快照

新增 `acceptance/ecosystem-2026-11-27.json`，相对 11-26 的变化：

| 场景 | 层级 | 结果 | 说明 |
|---|---|---|---|
| `contract-v0.2.0-rollout` | L0–L1 | pass | 7/8 消费仓库已采用；perception 未迁移 |
| `org-merge-queue` | L0 | pass | 11 个仓库启用 merge queue；break-glass 有书面记录 |
| `org-ci-reuse` | L0 | **fail** | 6/7 复用工作流仍是骨架；0/12 仓库引用 |
| `workspace-path-consistency` | L0 | **fail** | 旧根路径 `~/jcheng` 已不存在，多处文档/脚本未更新 |

总体结果仍为 **inconclusive**：M5.2 S7/S9、M6 S14/G8、mobile 运动、perception
可重复性等前缀限制没有变化；本次新增的是契约与治理状态，不是实机能力。

## 7. 待确认决策

### 7.1 `jcheng-perception` 的契约处置

核实结果：perception 的代码没有导入 `jcheng_interfaces`，其 `ROADMAP.md` 也写明
“perception 消息包是临时包，未来才晋升到契约仓库”。也就是说，它当前**不是**
运行时契约消费者，`>=0.1,<0.2` 只是一个声明性依赖。

- **A**：现在就把清单迁移到 `capability-manifest.v1` 并声明 `>=0.2,<0.3`；
  需要把 schema 的 `contract_hash` 改为条件必填。适合它已经开始消费契约类型的
  情况，当前证据不支持。
- **B（推荐）**：显式声明“尚未消费契约运行时类型”，把 `ROADMAP.md` 的依赖改为
  “待消息包晋升后再锁定”，不进入锁文件 `modules`；晋升时再迁移清单。
  这样既不虚报采用，也不强迫一个没有运行时契约的仓库追版本。
- **C**：保持现状只修文档、不记录豁免；这是最弱的选项，下一次审计仍会把它
  当成未完成项。

无论选哪个，perception 都不应在本轮进入 `modules`。

### 7.2 组织级 CI 复用策略

- **A（推荐）**：先填实 3 个高风险工作流（`reusable-contract-check`、
  `reusable-security`、`reusable-release`），仓库逐步引用；其余保留仓库自有。
- **B**：7 个全部填实并迁移 12 个仓库，一次性统一。
- **C**：明确放弃复用，删除骨架，把 `06` 改为“仓库自有 CI + 组织审计”。

### 7.3 锁文件首批 `modules` 与发布标识

- 首批纳入 `teleop 0.3.0`、`applications 0.1.0`、`studio 0.7.0`（ArmPi 组合）；
  `mobile` 暂不纳入，等移动平台进入运行时组合；
- 第一阶段只用 v1 允许的 `version`/`commit`/`interfaces`；
- `channel: dev` 允许无 tag，`stable` 必须签名标签（第二阶段字段）；
- `release` 已改为 `2026.11.1`，不再嵌入里程碑号。

### 7.4 `system_api` 语义

已采用 **A**（2026-11-27）：`system_api` 定义为系统 HTTP/WS 公共 API 版本，
与系统 `VERSION` 解耦。

- `jcheng-system/configs/api.yaml` 提供 `api_version: "0.1.0"` 与路径；
- `/v1` 是 HTTP 路径主版本，`api_version` 是能力仓库声明范围所用的语义版本；
- `verify_lock.py --module-root` 会验证每个模块 `capability.yaml` 的
  `system_api` 范围包含该值。

备选 B（删除字段）不再采用。

## 8. 实施顺序

| 阶段 | 内容 | 风险 | 回滚 |
|---|---|---|---|
| P0 | 本文、`05` 规则、验收快照、`OPERATIONS.md` 修正 | 无运行时影响 | 文档 revert |
| P1 | 锁文件 `modules`（v1 字段）+ `verify_lock` 扩展 + 首批 3 模块；富字段留待契约 v0.3.0 | 中；只影响发布校验 | 恢复旧锁文件 |
| P2 | 填实 3 个复用工作流并试点 2 个仓库 | 中；CI 变更 | 仓库改回自有 `ci.yml` |
| P3 | perception 清单迁移（按 7.1 决定） | 中；能力仓库声明变化 | 恢复旧清单 |
| P4 | 3 处工具常量更新 + 测试 | 低；代码 PR | revert |

任何阶段都不需要连接机器人；涉及实机运动的 M6/M7 收口仍按原规则单独授权。
