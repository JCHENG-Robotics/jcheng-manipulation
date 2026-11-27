# JCHENG Robotics 生态架构设计（已批准；M1–M6 离线与 S17 完成；v0.2.0 对齐审计完成）

> 状态：**M5.2 核心实机通过；M6 S13 21/21、S17 真实 MoveIt plan-only 通过；R5 契约 v0.2.0 已在 7 个消费仓库落地；v0.2.0 对齐包与 2026-11-27 验收快照已准备；G8/底盘待授权。**
> 基线：`jcheng-manipulation` 的 `v1.0.0`（提交 `b3411e4`，2026-11-23），未改动。
> 本文档集描述目标架构、仓库初始化方案、目录结构、迁移风险与实施顺序；
> M1–M6 已按批准范围实施，**未迁移现有代码、未修改 `v1.0.0`。**
> M6 已合并（PR #8 → `427214b`）；S17 于 2026-11-25 在机器人上完成真实
> `move_group` plan-only；后续实机运动仍需单独授权。

## 1. 这份文档集解决什么问题

`jcheng-manipulation` 已经是一个可复现、带完整证据链的单仓库实验基线，但它同时承担了
研究记录、ROS 包、Studio 后端、网关、部署脚本和实机运行器的职责。继续在这个仓库里
增加遥操作、学习、移动底盘和应用层，会产生三个结构性问题：

1. **安全边界被稀释。** 当前至少存在六条实机运动写入路径，其中 `profiles.fast`
   被明确记录为“设计上绕过 `armpi_safety`”。新模块如果延续这种模式，保护层将无法审计。
2. **版本不可独立演进。** 研究基线要求冻结，而生态模块要求持续迭代；同一个仓库无法同时
   满足“可复现证据”和“独立发布”两个目标。
3. **契约没有单一来源。** 关节限位、伺服映射、工作空间、姿态常量在多个文件中重复，
   现有 `tools/verify_config_consistency.py` 只是事后检查，不是单一权威。

本设计提出一个**契约优先、特权运动平面唯一、模块独立版本化**的多仓库生态。

### 1.1 实施进度（2026-11-23）

| 阶段 | 范围 | 状态 |
|---|---|---|
| M1 | 组织与三仓库、分支保护、双评审人 | 完成 |
| M2 | 契约审计、interfaces v0.1.0、CI、PR #3、签名标签 | 完成；GitHub `verified=true, reason=valid` |
| M3 | Mock 系统、9 包、S1–S17、锁文件、CI、PR #2 | 完成；`main` `40a74ba`，分支已删除 |
| M4 | 只读遥测、只读桥、只读网关、域配置、CI | 完成并合并（PR #4 → `3d8842d`）；实机只读 30 分钟通过 |
| M5.1 | ROS 服务/action、租约、安全、停止锁存（Mock） | 完成并合并（PR #5 → `5e4254f`） |
| M5.2 | 硬件桥、单关节/夹爪实机运动 | 核心通过并合并（PR #7 → `d765ae3`）：单关节、夹爪开/合、停止 0.266 ms、停止后拒绝、无租约/越限；S7/S9 残余 |
| M6 | S13 回放、S17 只规划、S14 底盘、G8 | 离线部分已合并（PR #8 → `427214b`）；S13 21/21、校验 mean 0.0166 ms；S17 真实 MoveIt plan-only `SUCCESS`、8 点轨迹、无执行路径；G8/底盘实机待授权 |
| A1 | v0.2.0 生态对齐：采用矩阵、锁文件 `modules` 拓扑、漂移登记、组织验收快照 | 审计完成（2026-11-27）；锁文件 `modules` v1 已实现（待合并）；4 项决策待确认 |

任何实机运动仍需单独、明确的现场授权。M5.2 夹爪 UNKNOWN 后未自动重试。

## 2. 目标与非目标

### 2.1 目标

- 建立 GitHub 组织 `JCHENG-Robotics` 的仓库分层与治理规则。
- 第一批只创建三个仓库：`.github`、`jcheng-interfaces`、`jcheng-system`。
- 定义统一接口契约、版本管理、依赖锁定、模块集成、CI 与端到端验收。
- 为 `jcheng-teleop`、`jcheng-learning`、`jcheng-mobile`、`jcheng-applications`
  预留无需修改系统层即可接入的扩展点。
- 保证所有实机运动命令经过统一的安全校验与控制权仲裁，任何新模块不能绕过保护层。

### 2.2 非目标（本阶段明确不做）

- 不增加任何机器人功能（不新增抓取、导航、学习算法）。
- 不迁移 `jcheng-manipulation` 的代码、证据或 Git 历史。
- 不修改、重建或移动 `v1.0.0` 标签；不改变发布清单与校验和。
- 不创建 GitHub 仓库、不推送代码、不配置组织权限（需批准后执行）。
- 不声称达到功能安全认证；平台仍然没有急停、抱闸、独立编码器和自碰撞模型。

**说明：** “不创建 GitHub 仓库、不推送代码”是设计阶段的边界；M1–M3 已按批准
范围解除，其余非目标（不增加机器人功能、不迁移代码、不改 `v1.0.0`、不声称认证）
仍然有效。

## 3. 文档索引

| 文档 | 内容 | 评审关注点 |
|---|---|---|
| [00-executive-summary.md](00-executive-summary.md) | 一页结论、关键决策、批准项 | 是否同意总体方向与边界 |
| [01-baseline-audit.md](01-baseline-audit.md) | v1.0.0 的包、接口、API、控制路径、安全与依赖审计 | 事实是否完整、有无遗漏的写入路径 |
| [02-org-and-repositories.md](02-org-and-repositories.md) | 组织、仓库职责、目录结构、初始化清单 | 三个首批仓库的边界是否正确 |
| [03-interface-contracts.md](03-interface-contracts.md) | ROS IDL、HTTP/WS、配置与证据契约 | 契约是否足够表达安全与仲裁 |
| [04-system-architecture.md](04-system-architecture.md) | 特权运动平面、控制权、安全状态机、DDS 拓扑 | “不能绕过”的 enforcement 是否成立 |
| [05-versioning-lock-release.md](05-versioning-lock-release.md) | SemVer、契约哈希、锁文件、发布列车 | 独立模块如何可复现地组合 |
| [06-ci-cd-acceptance.md](06-ci-cd-acceptance.md) | CI 矩阵、E2E 分级、验收场景与证据 | 什么算“通过”，如何防止回归 |
| [07-migration-and-risks.md](07-migration-and-risks.md) | 迁移原则、阶段计划、风险登记、回滚 | 实施顺序与停止条件 |
| [08-m1-execution.md](08-m1-execution.md) | M1 完成记录、证据、分支保护验证与治理待决项 | M1 验收与 M2 入口 |
| [09-m2-execution.md](09-m2-execution.md) | M2 完成记录：审计、v0.1 契约、CI、PR 合并与 GitHub Verified 标签 | M2 验收与 M3 入口 |
| [10-m3-execution.md](10-m3-execution.md) | M3 完成记录：Mock 核心、9 包、S1–S17、锁文件、PR #2 合并与 main CI | M3 验收与 M4 入口 |
| [11-m4-execution.md](11-m4-execution.md) | M4 只读接入：10 包、只读桥/网关/域配置、CI 与边界 | M4 离线验收与实机授权 |
| [12-m5-execution.md](12-m5-execution.md) | M5.1 Mock ROS 传输：服务/action、租约、安全、停止锁存 | M5.1 验收与 M5.2 授权 |
| [13-m5.2-execution.md](13-m5.2-execution.md) | M5.2 实机：核心通过；夹爪完成语义修复、停止与拒绝；S7/S9 残余 | 修复分支合并与 S7/S9 |
| [14-m6-execution.md](14-m6-execution.md) | M6：S13 21/21；S17 真实 MoveIt plan-only 通过；G8/底盘计划 | M6 离线验收、S17 实机只规划与实机授权 |
| [15-v020-alignment.md](15-v020-alignment.md) | v0.2.0 采用矩阵、锁文件 `modules` 拓扑、漂移登记、2026-11-27 验收快照 | 契约采用是否完整、锁文件模块范围与 4 项决策 |
| [adr/](adr/) | 关键架构决策记录 | 决策依据与替代方案 |

## 4. 基线承诺

本设计对 `jcheng-manipulation` 的承诺是**只读引用**：

- `v1.0.0` 标签、`b3411e4` 提交、`release/manifest.json`、证据目录和校验和保持不变。
- 新仓库不通过源码路径、Python `sys.path`、ROS 工作区 overlay 或 Git submodule
  依赖 `jcheng-manipulation` 的运行时代码。
- 需要复用的数值（限位、映射、标定常量）以**带来源哈希的机器人配置**方式复制，
  并由一致性检查证明与基线一致；不是隐式引用。
- `jcheng-manipulation` 在生态中的角色是**参考基线与证据来源**，不是首批系统的运行时依赖。
- 未来是否把仓库转移到组织、是否开启 v2 能力仓库，是独立的、需再次批准的决策。

## 5. 评审方式

建议按以下顺序评审，每份文档末尾都有“需要确认的决策”：

1. 先看 `00-executive-summary.md` 的决策表，确认方向。
2. 再看 `01-baseline-audit.md`，确认审计事实没有遗漏，尤其是实机写入路径。
3. 重点评审 `04-system-architecture.md` 的“六层防绕过”和 `03` 的契约字段。
4. 最后评审 `05`–`07` 的版本、CI、阶段与风险。

批准后已进入 M1–M5.2；M1–M3 为 Mock，M4 实机只读通过，M5.1 已合并。
M5.2 核心实机通过；M6 S13 21/21、S17 真实 MoveIt plan-only 通过；
R5 契约 v0.2.0 已发布并在 7 个消费仓库落地；v0.2.0 对齐包见
[15-v020-alignment.md](15-v020-alignment.md)。G8/底盘实机待授权。

## 6. 术语

| 术语 | 含义 |
|---|---|
| 特权运动平面（privileged motion plane） | 唯一持有硬件命令发布权限、执行安全校验与仲裁的进程集合 |
| 能力平面（capability plane） | 遥操作、学习、移动、应用等模块所在的非特权平面 |
| 运动意图（motion intent） | 能力模块提交给仲裁器的结构化请求，不是舵机/电机命令 |
| 控制租约（control lease） | 在限定范围、优先级和 TTL 内允许提交运动意图的授权 |
| 契约哈希（contract hash） | 对 IDL 与 JSON Schema 规范化后计算的 SHA-256，用于运行时握手 |
| 机器人配置（robot profile） | 关节限位、伺服映射、工作空间、标定常量与安全 profile 的单一来源 |
| 锁文件（lockfile） | 固定各仓库提交、契约版本、工具链与镜像摘要的发布清单 |
| `MOTION_OUTCOME_UNKNOWN` | 命令已发出但无法用总线读数确认结果的锁存状态，必须人工/总线复核后清除 |

## 7. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-11-23 | 初稿，基于 `v1.0.0` 基线审计 |
| 2026-11-23 | 同步实施状态：M1–M3 完成；M2 标签 GitHub Verified；M3 PR #2 合并、分支删除 |
| 2026-11-23 | M4 只读接入离线完成；CI 全绿；实机 30 分钟只读待单独授权 |
| 2026-11-23 | M4 实机只读 30 分钟通过：1799/1799 measured、writes=0；旧网关缓存与总线差异 387 pulses；修复分支待合并 |
| 2026-11-24 | M4 修复合并（PR #4 → `3d8842d`）；M5.1 Mock ROS 传输完成，CI `m5-mock` 全绿；M5.2 实机运动待授权 |
| 2026-11-24 | M5.1 合并（PR #5 → `5e4254f`）；M5.2 代码/首次记录合并（PR #6 → `c4f46b2`）；单关节通过；夹爪开 183/170、闭 620/640 均超差，两次 UNKNOWN；未完成 |
| 2026-11-24 | M5.2 第二次失败记录分支 `m5.2/second-attempt-docs` `3c3e172`，待 PR |
| 2026-11-24 | M5.2 第三次（内侧目标 194→207）仍 UNKNOWN；夹爪机械卡滞，停止运动与重试，需机械维护 |
| 2026-11-24 | 根因修正：夹爪应采用基线“定时完成 + 总线证据”，而非 ±8 精确容差；修复后闭 183→645、开 647→183 通过；停止 0.266 ms；S7/S9 残余 |
| 2026-11-24 | M6 离线开始：S13 G8 回放 21/21 decision 一致；`m6-replay` CI 全绿；S17/G8/底盘实机待授权 |
| 2026-11-24 | M6 已合并（PR #8 → `427214b`，main CI 绿）；S13 21/21、S17 plan-only 离线通过；S14/G8 实机待授权 |
| 2026-11-24 | M6 S17 离线：`jcheng_moveit` plan-only goal 构建/校验与正反测试通过；真实 MoveIt/G8/底盘待授权 |
| 2026-11-24 | M6 校验延迟：21 条回放 mean 0.0166 ms、p95 0.0352 ms；`m6/metrics` 后合并为 PR #11 → `78c613d` |
| 2026-11-25 | S17 真实 MoveIt plan-only 通过：MoveIt 2.5.10、domain 42、`SUCCESS`、8 点轨迹、`plan_only=true`；控制器 action 0 个服务端、无命令 topic；未执行运动 |
| 2026-11-27 | R5 停止确认契约：interfaces v0.2.0 签名标签 GitHub 验证通过；system 增加 `confirmation`/`physical_stop`，pin v0.2.0（PR #34）；未执行运动 |
| 2026-11-27 | v0.2.0 对齐审计：7/8 消费仓库已采用；perception 未迁移；锁文件 `modules` 拓扑与 2026-11-27 验收快照就绪 |
