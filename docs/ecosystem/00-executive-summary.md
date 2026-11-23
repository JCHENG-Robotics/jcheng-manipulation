# 执行摘要与关键决策

> 评审对象：JCHENG Robotics 生态初始化设计
> 基线：`jcheng-manipulation` `v1.0.0` / `b3411e4`（2026-11-23）

## 1. 结论

建议把当前的单仓库项目演进为**五层、三首批仓库**的 GitHub 生态：

```text
L0 治理层      JCHENG-Robotics/.github
L1 契约层      JCHENG-Robotics/jcheng-interfaces
L2 系统层      JCHENG-Robotics/jcheng-system
L3 能力层      jcheng-manipulation(冻结基线) · jcheng-teleop · jcheng-learning · jcheng-mobile
L4 应用层      JCHENG-Robotics/jcheng-applications
```

核心不是“把代码拆开”，而是先建立三条不可协商的边界：

1. **契约边界。** `jcheng-interfaces` 是唯一跨仓库编译期契约；ROS IDL、HTTP/WS Schema、
   配置 Schema 与证据 Schema 在同一个版本号下发布。
2. **运动边界。** 只有 `jcheng-system` 的特权运动平面可以发布硬件命令。所有能力模块
   只能提交“运动意图”，经 `jcheng_motion_arbiter` 做控制权仲裁、经 `jcheng_safety`
   做安全校验后被代执行。
3. **可复现边界。** 每个系统发布用锁文件固定各仓库提交、契约哈希、镜像摘要和机器人配置
   哈希；运行时不接受契约哈希不匹配的客户端。

`jcheng-manipulation` 的 `v1.0.0` 保持只读：不迁移、不改标签、不做运行时依赖。
首批三个仓库从空历史开始，现有仓库暂时留在原位置。

## 2. 关键决策

| ID | 决策 | 理由 | 替代方案与否决原因 |
|---|---|---|---|
| D1 | 第一批只建 `.github`、`jcheng-interfaces`、`jcheng-system` | 先把治理、契约、系统层立住；能力仓库没有稳定契约前不建 | 一次性建 7 个仓库：会产生无契约的空壳与版本漂移 |
| D2 | 接口仓库只产出一个 ROS 包 `jcheng_interfaces` | msg/srv/action 原子版本化，避免多包版本组合爆炸 | 拆成 `jcheng_msgs`/`jcheng_srvs`/`jcheng_actions`：构建与版本成本高，收益不足 |
| D3 | 全部运动走单一入口 `ExecuteMotion` action | 一个入口才能集中做租约、限位、审计与幂等 | 保留多条 topic/service：无法证明“无绕过” |
| D4 | 控制权用“租约”表达，而不是隐式 UI 状态 | 可过期、可撤销、可优先级抢占、可审计 | 沿用 Studio 单一 lease：只覆盖一个后端，不能覆盖多模块 |
| D5 | 安全节点只做校验、状态机与锁存，不执行运动 | 职责单一，避免“裁判兼运动员” | 让安全节点代执行：故障域扩大，审计困难 |
| D6 | 规划与执行分离 | 规划可在非特权平面运行；执行必须特权化 | 让模块直接调 `/move_action`：绕过安全层 |
| D7 | DDS 分域：厂商 0 / 硬件 41 / 能力 42 / 仿真 43 | 能力模块默认看不到厂商命令话题 | 单域 + 约定：无法防止误配置或恶意发布 |
| D8 | 版本 = SemVer + 契约哈希；锁文件钉提交 | ROS IDL 类型哈希对任何字段变化敏感，语义版本不足以表达兼容性 | 只用 tag：可被移动；只用版本号：无法发现未升级的 IDL 改动 |
| D9 | 机器人配置单一来源，派生安全/MoveIt/后端配置 | 消除当前多文件重复导致的限位漂移风险 | 继续多文件 + 事后检查：已发生过 joint2 事件 |
| D10 | 新仓库使用全新 Git 历史，不用 `filter-repo` | 保留基线证据链，避免重写历史与校验和失效 | 从基线切分历史：会改变提交哈希与发布证据 |
| D11 | `jcheng-manipulation` 在生态 v1 中仅作参考基线 | 冻结承诺；避免系统层被研究代码反向耦合 | 直接复用其运行器：破坏“不可修改基线” |
| D12 | Studio 拆为“系统网关 + 应用层 UI” | 安全与租约必须由系统层唯一持有，不能停留在某个 UI 后端 | 保留 Studio 后端作为第二权威：双权威冲突 |

## 3. 现状中最需要解决的问题

审计（见 `01-baseline-audit.md`）确认了四个结构性问题：

### 3.1 六条实机写入路径，保护覆盖不一致

仓库内的权威清单（`docs/phase2/MOTION_WRITE_PATHS.md`）列出六条路径：
MoveIt CLI、Fast CLI、Studio G6 step、Studio G7 pose、Studio G8 task/fast、
Chassis pulse；此外还有 `pulse_move.py`、`servo_hold.py` 等脚本直写总线。

`armpi_safety` 只否决 MoveIt CLI。Studio G6/G7/G8 依赖网关自己的策略；
Fast CLI 两者都不经过，被 `RELEASE.md` 明确记录为“设计上绕过 `armpi_safety`”；
底盘由网关策略保护；脚本直写没有统一守卫。`/safety_stop` 目前只有 Studio
网关消费。**这在新生态中不可接受。**

### 3.2 安全配置多处重复

同一组关节限位、伺服 ID、夹爪范围、低构型修正常量出现在 xacro、MoveIt 配置、
`armpi_safety`、Studio 映射、网关运动模块和多个 phase2/3 工具中。现有
`tools/verify_config_consistency.py` 是事后检查，且只覆盖已知文件。新系统必须让
**一个 robot profile 生成所有派生配置**。

### 3.3 安全语义与硬件事实不一致

- `/joint_states` 与 `/controller_manager/servo_states` 是厂商**命令缓存**；
  safety 节点读到的 `/joint_states` 还是 `joint_state_publisher` 对缓存的 20 Hz
  重发。真正的位置来自 `GetBusServoState`。
- 网关在安全节点未启动时默认 `safety_stop=false`，没有“等待安全节点”的联锁，
  属于 **fail-open**。
- Studio 后端完全不知道 `/safety_stop`，`stop_state` 在硬件快照中固定为 `None`，
  因此“安全就绪”检查在硬件上恒通过。
- `/safety_stop` 只阻止新命令，不中止在飞的密集轨迹或底盘脉冲；stop 只是机械臂
  hold，不归零底盘。

新契约必须把 `truth=measured|commanded`、`verification=bus|timed|open_loop`
作为一等字段；安全状态必须由新鲜的心跳维持，缺失即锁存；停止必须中止在飞执行
并使底盘归零。

### 3.4 平台没有急停、抱闸、编码器和自碰撞模型

软件层可以做到“统一校验、锁存、审计、最小权限”，但不能替代硬件安全。
新生态必须把这一限制写进验收口径，禁止用“已通过安全校验”暗示功能安全认证。
厂商容器重启时的 `startup_check` 会移动机械臂，这是软件层无法完全否决的残余路径，
必须用“启动前置条件 + 物理监督 + 证据”管理。

## 4. 目标系统的一句话描述

> 能力模块在非特权 DDS 域里提交带租约的运动意图；系统层的仲裁器验证控制权与契约，
> 安全监督器验证状态、限位、轨迹与工作空间；只有硬件桥可以发布厂商命令；执行结果
> 只有在总线读数确认后才算完成；所有决定写入不可变证据包。

## 5. 实施阶段（批准后才执行）

| 阶段 | 内容 | 是否接触实机 | 出口条件 |
|---|---|---|---|
| M0 | 本设计评审 | 否 | 本文件集批准 |
| M1 | 组织与 `.github` 初始化 | 否 | 模板、规则集、复用工作流可用 |
| M2 | `jcheng-interfaces` v0.1.0 | 否 | IDL/Schema/契约哈希 CI 通过 |
| M3 | `jcheng-system` 纯 Mock 骨架 | 否 | 租约、安全、仲裁、网关 Mock E2E 全绿 |
| M4 | 机器人只读接入 | 只读 | 图、时钟、遥测、安全状态可见 |
| M5 | 单关节/夹爪受监督执行 | 是，受监督 | 租约、安全否决、总线确认、停止锁存实测 |
| M6 | 基线轨迹回放与 G8 复验 | 是，受监督 | 回放一致性与 G8 证据 |
| M7 | 首个外部模块试点（遥操作或应用） | 是，受监督 | 证明该模块无硬件发布权限 |
| M8 | 生态 v1.0 发布 | 视验收 | 锁文件、SBOM、验收报告、回滚演练 |

详细出口条件与风险见 `07-migration-and-risks.md`。

## 6. 需要批准的事项

请明确回复以下五项：

1. **是否同意五层分层与三个首批仓库。**
2. **是否同意“单一运动入口 + 租约 + 安全否决”作为不可协商的安全边界。**
3. **是否同意 `jcheng-manipulation` 仅作参考基线、不做运行时依赖、暂不转移仓库。**
4. **是否同意新仓库使用全新 Git 历史，并从 M1 开始分阶段实施。**
5. **是否同意首批仓库采用 MIT 许可证、公开可见、`main` 受保护**（如需私有或
   Apache-2.0，请在批复中说明）。

未获批准前，不创建仓库、不推送、不写代码、不接触实机。
