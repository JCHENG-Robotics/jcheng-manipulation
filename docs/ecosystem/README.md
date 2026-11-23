# JCHENG Robotics 生态架构设计（评审稿）

> 状态：**设计评审中，未批准，未实施。**
> 基线：`jcheng-manipulation` 的 `v1.0.0`（提交 `b3411e4`，2026-11-23）。
> 本文档集只描述目标架构、仓库初始化方案、目录结构、迁移风险与实施顺序；
> **不迁移现有代码，不修改 `v1.0.0`，不创建 GitHub 组织仓库，不向实机发送任何命令。**

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
| [09-m2-execution.md](09-m2-execution.md) | M2 完成记录：审计、v0.1 契约、CI、PR 合并与 v0.1.0 标签 | M2 验收与 M3 入口 |
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

批准后进入 M1（组织与 `.github` 初始化）。M1 不会触碰实机，也不会迁移代码。

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
