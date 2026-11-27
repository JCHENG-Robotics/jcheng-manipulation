# SCEN 层：声明式场景库

> 状态：**v0.1.0 准入**，2026-11-27。`jcheng-scenarios` 已创建并推送；
> 仓库门禁与 CI 通过。SCEN 层位于运行时依赖图之外，不参与启动、控制或运动。
> 本文是生态设计记录中的层定义；仓库内细节以其 `docs/SCEN_LAYER.md` 为准。

## 1. 分类与边界

`jcheng-scenarios` 属于 **SCEN** 层，与 BENCH、RESEARCH 一样是离线知识层，
不属于 L0–L4 运行时分层：

| 属性 | 值 |
|---|---|
| 运行时节点 | 无 |
| DDS 域 | 无 |
| 硬件访问 | `none` |
| 运动权限 | `none` |
| 控制通道 | `none` |
| 输入 / 输出 | YAML/JSON 文件进；经过校验的声明与报告出 |
| 运行时依赖 | 无；不被任何运行时仓库导入 |

SCEN 不是 L4：它不是操作台，也不调用系统网关。它不是 BENCH：它声明评测指标，
但由 `jcheng-benchmark` 执行。它不是能力仓库：没有算法、设备或运动路径。

## 2. 职责

| 仓库 | 负责 | 不负责 |
|---|---|---|
| `jcheng-scenarios` | 场景 ID 与版本；环境、语义、任务、能力需求、运行约束、评测声明；离线结构校验 | 运行时执行、运动、设备控制、安全执行、仿真被控对象、基准报告、感知算法 |
| `jcheng-devices` | 设备能力、限制与兼容性描述（离线） | 场景任务目标、运行时运动、安全权威 |
| `jcheng-applications` | 解析任务规范、确定性规划、经网关的失败关闭意图序列 | 场景编写、运动执行、安全策略 |
| `jcheng-perception` | 通用感知与场景理解；advisory 场景状态 | 场景验收、运动权威、矿石品位或经济价值结论 |
| `jcheng-simulation` | 域 43 仿真被控对象；模拟真值 | 把场景声明当测量、加入域 0/41/42 |
| `jcheng-benchmark` | 在声明的数据源上执行评测并生成带 provenance 的报告 | 修改场景阈值、编造测量、给出安全验收 |
| `jcheng-system` | 运行时安全、租约、仲裁、限位、停止锁存、硬件控制与唯一 `ExecuteMotion` | 把场景文件当权威、把安全责任下放给场景 |
| `jcheng-interfaces` | 跨仓库契约、Schema 与契约哈希 | 在提案获批前把仓库内 SCEN 文档当契约 |

场景是**意图与上下文的声明**，不是权威。系统层仍是唯一运动来源。

## 3. v0.1.0 内容

| 内容 | 说明 |
|---|---|
| 场景注册表 | `registry.yaml`：唯一 ID、版本、路径、交付/验证状态 |
| 场景 Profile | 顶层身份、provenance、消费方与边界声明 |
| 环境 Profile | 坐标系、工位、区域、光照、表面、物理约束 |
| 语义 Profile | 类别、可观测属性、关系、禁止推断 |
| 任务规范 | 目标、前置条件、完成条件、失败条件、范围外事项 |
| 能力需求 | 抽象设备/算法/接口需求；不绑定设备型号 |
| 运行约束 | 业务、环境、风险与安全边界 |
| 评测规范 | 指标、测试层级、数据源类别、报告归属 |
| 离线校验 | 8 份仓库内 JSON Schema、严格 YAML 加载器、CLI 与测试 |
| 参考场景 | `ore_sorting`；所有参数为设计声明，所有验证状态为 `Unverified` |

仓库：<https://github.com/JCHENG-Robotics/jcheng-scenarios>，标签 `v0.1.0`。

## 4. 依赖方向与消费方

```text
                只读固定版本
 jcheng-applications ─────────────┐
 jcheng-simulation ───────────────┼──▶ jcheng-scenarios
 jcheng-benchmark ────────────────┘         │
                                            │ 能力词汇（版本化）
                                            ▼
                                    jcheng-devices
                                            │
                                  两者不互相 import
                                            │
 jcheng-perception ── advisory 场景状态 ────┘（先契约评审）

 jcheng-system ◀── 仅经网关的运动意图 ── applications / 系统客户端
       ▲
       └── 安全权威；绝不下放给 SCEN
```

| 消费方 | 消费内容 | 集成状态 | 前置条件 |
|---|---|---|---|
| `jcheng-devices` | `capability-requirements.yaml` | Not started | 共享能力词汇提案与设备 Profile Schema |
| `jcheng-applications` | `task.yaml`、`semantics.yaml` | Not started | 任务规范契约或受控本地适配器；只经网关执行 |
| `jcheng-simulation` | `environment.yaml` | Not started | 环境到仿真资产映射；仅域 43 |
| `jcheng-benchmark` | `evaluation.yaml` | Not started | 指标与报告映射；不得修改阈值 |
| `jcheng-perception` | 可选消费 `semantics.yaml` 的类别名 | Not started | Scene State 契约提案；感知保持 advisory |
| `jcheng-system` | 不消费 | Not applicable | 安全权威，不是场景消费方 |

## 5. 循环依赖防护

- 没有仓库 import SCEN 代码；消费方用自己的读取器解析固定版本文件。
- 场景能力需求使用抽象 dotted capability 名称，不写设备 ID。
- 设备 Profile 用同一词汇声明能力，不把场景 ID 当权威。
- 未来匹配器是离线工具，不是运行时依赖；归属在设备库一侧。
- 跨仓库消费必须固定场景版本；浮动读取 `main` 不能作为验收证据。

## 6. 变更与准入

新增场景或 Schema 版本必须满足：

- [ ] 职责不与现有仓库重叠。
- [ ] 每个参数有 `design_target` / `measured` / `derived` / `assumed` 依据；
      测量值链接证据。
- [ ] 每个指标有阈值状态；`not_set` 是允许且诚实的状态。
- [ ] 不声明品位、成分、质量、价值等禁止推断。
- [ ] 仓库门禁与测试通过。
- [ ] 跨仓库消费变更已有获批契约提案或书面本地适配器。
- [ ] 任何硬件声明逐次授权并链接证据；场景文件本身不声称硬件验证。

## 7. 矿石场景资产

现有矿石研究、分割管线、Scene State Schema 与证据仍在 `jcheng-perception`。
v0.1.0 **没有移动任何文件**：`jcheng-scenarios` 只新增场景声明，并以路径引用
perception 的设计文档与证据。归属与迁移建议见
[`jcheng-scenarios/docs/ASSET_AUDIT_20261127.md`](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/ASSET_AUDIT_20261127.md)。
perception 的 `ORE_SCENE_UNDERSTANDING.md` 中“不新建独立仓库”是 S0 阶段的模块
非目标；本仓库不吸收该模块，该表述的上下文澄清是 perception 的后续文档变更。

## 8. 路线

| 版本 | 范围 |
|---|---|
| v0.1.x | 场景注册、Schema、声明式配置与离线校验（当前） |
| v0.2.x | 领域语义建模、任务规范与场景约束 |
| v0.3.x | 场景能力需求与设备能力匹配、离线兼容性验证 |
| v0.4.x | 与 Applications、Simulation、Benchmark 的受控集成 |
| v0.5.x | 跨设备、跨场景任务复用与实机实验验证 |
| v1.0.0 | 稳定契约、可重复验证、可持续扩展的场景规范体系 |

## 9. 相关文档

- [`jcheng-scenarios` 架构](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/ARCHITECTURE.md)
- [`jcheng-scenarios` SCEN 层](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/SCEN_LAYER.md)
- [`jcheng-scenarios` 场景模型](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/SCENARIO_MODEL.md)
- [`jcheng-scenarios` 能力匹配](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/CAPABILITY_MATCHING.md)
- [`jcheng-scenarios` 评测模型](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/EVALUATION_MODEL.md)
- [`jcheng-scenarios` 资产审计](https://github.com/JCHENG-Robotics/jcheng-scenarios/blob/main/docs/ASSET_AUDIT_20261127.md)
- [仓库分类与依赖方向](02-org-and-repositories.md)
- [接口契约](03-interface-contracts.md)
- [`jcheng-perception` 矿石场景路线](https://github.com/JCHENG-Robotics/jcheng-perception/blob/main/docs/ORE_SCENE_ROADMAP.md)
