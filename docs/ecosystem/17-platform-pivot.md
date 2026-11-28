# 平台转向：冻结 ArmPi Ultra，转向 PiPER X 与 AR5 双臂

> 状态：**提议（待评审）**，2026-11-28。
> 本文是生态设计记录，定义平台迁移的边界、契约决策与阶段顺序。它不修改任何
> 契约、代码、部署单元或证据，也不授权任何实机运动。
> 基线：`jcheng-manipulation` `v1.0.0`（提交 `b3411e4`，2026-11-23）只读引用；
> 契约 pin `v0.2.0`（提交 `b03a054`，`sha256:afca22a4…c35b1c`）。

## 1. 决策摘要

| ID | 决策 | 值 | 理由 |
|---|---|---|---|
| D1 | ArmPi Ultra 平台状态 | `Frozen`，不 `Archived` | 只读演示与回滚仍需可用；证据与部署记录保留 |
| D2 | 部署形态 | 先固定单元，后移动 | 去掉底盘、导航与移动标定；先把运动链做对 |
| D3 | PiPER X 角色 | 生产执行臂；示教 → 录制 → 回放 → 受监督执行 | leader 不进控制回路，安全论证最简 |
| D4 | AR5 角色 | 第二阶段双臂；身份门禁通过后才写后端 | 本地零资产，厂商/控制器/SDK 未确认 |
| D5 | 契约变更 | `v0.3.0`：robot-profile v2、`chain_id`、夹爪范围去常量 | v1 是舵机形状，无法表达 CAN/EtherCAT/双臂 |
| D6 | 证据规则 | 按平台重做；ArmPi 的 `Verified` 不迁移 | 不同硬件、不同接口、不同标定 |

**关键区分：冻结的是平台，不是仓库。** `jcheng-system`、`jcheng-perception`、
`jcheng-applications` 等继续演进；`jcheng-manipulation` 保持既有的 `Frozen`。
平台冻结只改变 ArmPi 的部署、运动权威与验收状态。

## 2. 范围

### 2.1 在范围内

- ArmPi Ultra 平台的冻结记录、只读保留与运动权威撤销。
- `robot-profile` v2：总线无关、n-DoF、单夹爪类型化、多链 workspace/tip。
- 双臂寻址：`chain_id` 与租约作用域扩展。
- 夹爪范围从契约常量改为 profile 驱动。
- PiPER X 只读接入、受监督执行与示教回放。
- AR5 身份门禁、只读接入与双臂协调。
- 固定单元的 ops / perception 部署 profile。

### 2.2 明确不做

- 不修改 `jcheng-manipulation` 的代码、标签、清单或证据。
- 不删除、不移动 ArmPi 的任何证据或部署记录。
- 不把 leader 臂接入控制回路，也不让 teleop 成为默认运动源。
- 不在 v1 profile 中伪造 `servo_id` 或脉冲。
- 不做移动底盘、Nav2、`/cmd_vel` 集成（P7，另行立项）。
- 不做实时主从遥操作（P3.5，另行安全案例）。
- 不做 AR5 力矩控制；v1 只做位置。
- 不声称功能安全认证；平台仍无急停、抱闸、独立编码器。

## 3. 现状评估

### 3.1 可复用资产

| 层 | 资产 | 状态 |
|---|---|---|
| L1 | `jcheng-interfaces/docs/proposals/piper-x-robot-profile.md` | v2 profile 形状已写，提案未实现 |
| L2 | `jcheng-system/configs/robots/piper_x/profile.yaml` | simulation-only mock，可加载 |
| L2 | `jcheng-system/tests/test_teleop.py` | 2-DoF 与 6-DoF 走同一仲裁器，证明 DOF 非耦合 |
| L3 | `jcheng-simulation/docs/robots/piper_x/` | MIT URDF 资产管线，618 文件校验通过 |
| L3 | `jcheng-learning/src/jcheng_learning/profiles/piper_x.yaml` | 6 关节 + 夹爪 adapter |
| L3 | `jcheng-teleop/examples/piper_x_demo/` | profile 驱动，关节数不写死 |
| L3 | `jcheng-devices/registry/` | `agilex.piper-x`（candidate）；`rokae.ar5`/`ar5l`/`ar5r`（planned） |
| L4 | `jcheng-studio`、`jcheng-applications` | 只走 gateway，基本与臂型号无关 |

结论：PiPER X 的迁移是**补后端**，不是建平台。AR5 才是空白。

### 3.2 耦合面

| # | 位置 | 卡点 | 迁移动作 |
|---|---|---|---|
| A | `jcheng-interfaces/schemas/robot-profile.v1.schema.json` | 每关节必须 `servo_id`；夹爪必须 `measured_*_pulses`；`profiles.base` 必填；单 robot/单 workspace/单 tip | 新增 v2，v1 保持可加载 |
| A | `jcheng-interfaces/schemas/messages.v1.schema.json` | `GripperCommand.position_rad` 上限写死 `2.22`；`MotionCommand` 无 `chain_id`；`ControlLease.scope` 单值 | 范围来自 profile；加链地址 |
| B | `jcheng-system/ros2_ws/src/jcheng_vendor/jcheng_vendor/executor.py:41` | `HardwareBridge` 是脉冲端口（`send_step→pulse`、`read_bus→pulses`、`tolerance_pulses`） | 新 joint-space 端口 |
| B | `jcheng-system/ros2_ws/src/jcheng_vendor/jcheng_vendor/bridge.py:28` | 直接 import `GetBusServoState` / `/servo_controller`，Hiwonder 专用 | 新 PiPER / AR5 backend |
| B | `jcheng-system/ros2_ws/src/jcheng_hardware/jcheng_hardware/readonly.py` | 0..1000 pulse、必须恰好一个夹爪、`read_order` 覆盖全部舵机 | 新 mapping kind |
| C | `jcheng-system/ros2_ws/src/jcheng_system_core/jcheng_system_core/safety.py:512` | `frame_id != "base_link"` 直接拒绝 | 参数化到 profile |
| C | `jcheng-system/ros2_ws/src/jcheng_system_core/jcheng_system_core/safety.py:522` | base pulse 固定 4 通道 | 与 D2 一并出局 |
| C | `jcheng-system/ros2_ws/src/jcheng_moveit/jcheng_moveit/plan_only.py:13` | `group="arm"`、`frame="base_link"`、`tip="end_effector_link"` 写死 | 每链参数化 |
| D | `jcheng-applications/src/jcheng_applications/model.py:464` | 夹爪 `[0.0, 2.22]` rad | profile 驱动 |
| D | `jcheng-perception/config/wrist_tf.yaml` | `bus_servo/get_state`、numeric servo IDs、`link4` 手眼 | CAN joint source + 新手眼 |
| D | `jcheng-perception/config/cameras.yaml` | 要求 external 与 wrist 两种 role 同时存在；融合路径假设单腕 | 双腕需融合侧扩展 |
| D | `jcheng-ops/robot/armpi-ops-health` | 8765/8766、`/dev/rrc`、ArmPi unit 名与 host | 新固定单元部署 profile |
| E | `jcheng-system/scripts/check_m4_readonly.sh` | 断言 `read_order == [1..6]` | 参数化 |
| E | `jcheng-telemetry` / `jcheng_vendor` / `jcheng_runtime` 默认路径 | 默认指 `configs/robots/armpi_ultra` | 去默认，显式传入 |

### 3.3 结论

生态的脊柱——单一运动权威、租约、deadman、停止锁存、guard、证据、HTTP/WS
网关、契约版本机制——与臂型号无关。迁移成本集中在三层：

1. **契约形状**（robot-profile v1、夹爪上限、无链地址）；
2. **执行器端口**（脉冲语义、单臂、Hiwonder 专用 bridge）；
3. **部署与标定**（wrist TF、相机 role、ops 探针、默认路径）。

换平台 = 换后端 + 升一次契约 + 重做标定与证据。

## 4. 目标运行形态

### 4.1 固定单元拓扑

```text
固定单元（无底盘）
├── PiPER X follower   ← 生产执行臂，唯一运动被控对象
├── PiPER X leader     ← 示教输入，只读，永不接收命令
├── D435i 外部相机     ← 固定基座标定
├── （可选）腕部相机   ← PiPER link6 手眼，后置
└── 主机               ← Orin 或工作站；域 0 vendor / 41 特权 / 42 能力
```

移动化（P7）不在本设计内。`jcheng-mobile` 保持 `Experimental` 休眠，不做
挂载、导航或底盘标定。

### 4.2 示教 → 回放 → 受监督执行

```text
leader (PiPER) ──只读──▶ 示教捕获（operator 侧，复用 teleop capture core）
                              │  joint trajectory + provenance + seal
                              ▼
                        离线回放（SafetySupervisor，复用 S13 判定）
                              │  决策一致率 100%
                              ▼
              ExecuteMotion ─▶ 仲裁器 ─▶ 租约 ─▶ follower (PiPER)
```

- leader 只读：不使能、不发运动命令；捕获进程在 operator 侧，不属于 system。
- 回放复用 `jcheng-system/tools/replay_g8.py` 的判定模式，不直接执行运动。
- 执行走 `JOINT_TRAJECTORY`，每帧由 CAN 读数确认；停止中止在飞帧。
- 生产 recipe 生成同一种 `JOINT_TRAJECTORY`，与示教回放共用执行器。

### 4.3 为什么不是实时主从

实时主从要求 leader 进入控制回路，带来三个额外问题：控制环频率与 CAN 周期、
deadman/看门狗调参、以及双权威风险。生产执行臂不需要这些：轨迹在离线阶段
已经过安全判定，执行阶段只需租约、限位、停止与逐帧确认。

如果未来要做实时遥操作，单独立 P3.5：批准 `intent_source` 能力类、实现流式
执行器、给出独立安全案例与证据。它不阻塞 P0–P3。

## 5. 契约决策

### 5.1 robot-profile v2

| v1 要求 | PiPER / AR5 现实 | v2 处置 |
|---|---|---|
| `joints[].servo_id` 必填 | CAN 关节索引 / EtherCAT 从站，无舵机 ID | 改为 `index`，`servo_id` 仅在 `kind: bus_servo` 下要求 |
| `gripper.servo_id` + `measured_*_pulses` 必填 | 平行棱柱夹爪，行程 0.1 m，无脉冲 | 夹爪 `type: bus_servo | parallel_prismatic`，按类型要求字段 |
| `profiles.base` 必填 | 固定单元无底盘 | `base` 改为可选；`BASE_PULSE` 在无 base 的 profile 上拒绝 |
| 单 `robot` / 单 `workspace` / 单 `tip_link` | 双臂需要每链 workspace 与 tip | 引入 `chains[]`（见 5.2），每链独立 workspace/tip |
| `source_hashes` | URDF / SRDF / 标定文件 | 保留；必含 URDF 与生成物哈希 |

v1 profile 保持可加载、不改语义。v2 通过 `schema_version: 2` 与 `kind` 区分。
PiPER X 的具体取值以 `jcheng-simulation/docs/robots/piper_x/joint_map.yaml`
为来源；J6 采用保守的 ±2.880 rad，直到固件/总线读数确认。

### 5.2 双臂寻址

两个选项：

| 选项 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| 前缀（最小改动） | 一个 profile、14 个关节、名字 `ar5l_axis1` | 不改 IDL，今天可跑 | 租约无法分臂；停止/恢复粒度粗；MoveIt 分组需另做名字映射 |
| `chain_id`（推荐） | `MotionCommand.chain_id` + `RobotState` 分组 + 租约 scope `arm:left` | 左右独立租约、独立停止、独立遥操作；与 MoveIt 分组一致 | 一次 IDL 变更；需要向后兼容空值语义 |

推荐 `chain_id`：AR5 双臂是这个生态第一次真正的多链机器人，一次做到位比
两次改契约便宜。空 `chain_id` 表示"默认链"，PiPER 单臂继续使用空值。

### 5.3 夹爪

- `GripperCommand.position_rad` 的 `2.22` 上限从 JSON Schema 移除，改为
  profile 的 `gripper.lower/upper` 校验。v1 消费方仍按 profile 得到 2.22，
  行为不变。
- PiPER 夹爪登记为独立 end-effector profile（暂定 `agilex.agx-gripper`），
  行程 0–0.1 m，mimic 关节不入契约。
- `jcheng-applications` 的 `[0.0, 2.22]` 常量删除，改由 profile 读取。

### 5.4 `intent_source`

`intent_source`（teleop P-006）**不阻塞本迁移**。示教捕获是离线数据路径，
不是运行时能力类。该提案保留，等 P3.5 实时遥操作时再批准。

### 5.5 版本与发布

- 新标签 `v0.3.0`；重生成 `interface_manifest.json`；更新 `configs/contract.yaml`
  与 `locks/ecosystem.lock.yaml`。
- 7 个仓库的 `capability.yaml` 当前 pin `interfaces: ">=0.2,<0.3"`：
  applications、devices、learning、mobile、scenarios、studio、teleop；
  加上 `jcheng-system` 的 `configs/contract.yaml` 与 `locks/ecosystem.lock.yaml`，
  共 8 个消费边界，全部改为 `>=0.3,<0.4`。
- `jcheng-simulation` 当前 pin `>=0.1,<0.2`，落后两个 minor；本次一并评估，
  不默认跟随。
- 契约变更前不得合并任何依赖 v2 的运行时代码。

## 6. 阶段计划

| 阶段 | 内容 | 出口证据 | 估算 |
|---|---|---|---|
| P0 | ArmPi 平台冻结 | freeze record；只读演示可用；无新 ArmPi 功能 PR | 1 周 |
| P1 | 契约 v0.3.0 | v1/v2 均可加载；全 gate 绿；mock 跑 Piper v2 profile | 2–3 周 |
| P2 | PiPER X 只读 | 30 min 只读、零写、实测与 SDK 一致 | 1–2 周 |
| P3 | PiPER X 生产执行 | 一个抓放循环、逐帧确认、停止 ≤100 ms、leader 零下发 | 3–5 周 |
| P4 | AR5 身份门禁 | 6 项事实有出处或明确"未知" | 1 周 |
| P5 | AR5 只读 + 双臂 | 双臂只读；plan-only 无碰撞；单臂 L5 | 4–8 周（gated） |
| P6 | 固定单元收尾 | 新 lock、每平台独立证据、release notes | 2–3 周 |
| P7 | 移动化 | 单独立项 | deferred |

### P0 平台冻结

- 停止 M6 S14 与完整 G8；在 M6 范围文档中记为**平台退役，非 deferred**。
- `jcheng-system/deploy/armpi-ultra/` 增加 `Frozen` 说明；保留 unit 文件与
  镜像摘要 `armpi-ultra:flatten-20261125`。
- 撤销运动权威：停止 `jcheng-gateway-vendor`，仅保留只读 `/v1`。
- `jcheng-ops` 的 ArmPi 探针标 `legacy`，不再安装新实例。
- 写 freeze record：已验证项、未验证项、契约 pin、镜像摘要、回滚步骤。

**回滚**：恢复只读网关即可；不重启 vendor 容器（启动检查会动臂）。

### P1 契约 v0.3.0

- `robot-profile.v2.schema.json`；`chain_id`；夹爪范围去常量。
- v1 加载器不变；新增 v2 加载分支与一致性测试。
- 生成 Piper X 的 v2 runtime profile（`simulation-only` 起步）。
- 更新 7 个 `capability.yaml` 边界 + `jcheng-system` 的 contract/lock + manifest。

**出口**：`python3 -m pytest -q` 全绿；`jcheng-interfaces` 的 schema/manifest
检查通过；mock 用 v2 profile 跑 S1–S12；无实机依赖。

**回滚**：不发 `v0.3.0` 标签；消费仓库边界保持在 `>=0.2,<0.3`。

### P2 PiPER X 只读

- 新 backend `jcheng_vendor_piper`（只读）：关节反馈、状态、错误码。
- Orin 双 candleLight 通道独立；不假设共享 CAN 总线。
- 新部署 profile：主机、域、CycloneDDS、unit 名。
- 夹爪资产登记（`agx_gripper`），只做描述，不控制。

**出口**：30 min 只读；`writes=0`；实测与 SDK 读数一致；通道拓扑（哪个 serial
对应左/右）记录在案。

**回滚**：停 unit；不动 vendor 容器；不发送任何命令。

### P3 PiPER X 生产执行

- 3a 示教捕获：leader 只读适配器（operator 侧）；先验证 SDK 能在不使能、
  不发运动命令的前提下读反馈。
- 3b 离线回放：复用 S13 判定；决策一致率 100%；关节空间限位。
- 3c 受监督执行：轨迹执行器逐帧下发；每点 CAN 确认；停止中止在飞帧；
  `MOTION_OUTCOME_UNKNOWN` 锁存。

**出口**：一个完整抓放循环；每阶段总线确认；停止 ≤100 ms（实测）；leader
全程零下发（静态 + 运行时证据）；证据等级 L5。

**回滚**：软件停止 + 解除使能 + 停执行器；总线复核实际位置；记录证据。

### P4 AR5 身份门禁

纯文档，可与 P1 并行。六项：

1. 厂商与型号（珞石 AR5 还是其他厂商）。
2. 控制器拓扑（一控双腕 / 两控）。
3. SDK 与许可（Rokae ROS 2 包、依赖使用条款）。
4. EtherCAT 主站与实时性（主机、RT 内核、周期）。
5. URDF / 关节符号 / 限位。
6. 夹爪型号、行程、控制接口。

任一项未知：AR5 保持 `planned`，不写 P5 代码，不修改 `rokae.ar5*` 记录。

### P5 AR5 只读 + 双臂

- 只读 backend；双臂反馈；每链 workspace/tip。
- MoveIt plan-only 双臂；SRDF 的 disable-collision 与自碰撞模型。
- 单臂受监督 → 顺序动作 → 协调动作。

**出口**：双臂只读零写；plan-only 无碰撞；单臂 L5；双臂 supervised 证据。

### P6 固定单元收尾

- ops 新部署 profile；studio 端点；applications 夹爪/recipe；scenarios；
  benchmark；perception 固定基座标定与（可选）腕相机。
- 新 lock、release notes、证据索引。

## 7. ArmPi 冻结清单

### 7.1 平台动作

| 动作 | 目标状态 | 备注 |
|---|---|---|
| 停止 M6 S14 / 完整 G8 | `Not started`，标注平台退役 | 不写"待授权" |
| 运动权威 | 撤销；仅保留只读 `/v1` | 不重启 vendor 容器 |
| `deploy/armpi-ultra/` | `Frozen` 说明 | 文件与镜像摘要保留 |
| `jcheng-ops` ArmPi 探针 | `legacy` | 不删代码，不装新实例 |
| 只读演示 | 保留 | `/v1` 只读；无 `ExecuteMotion` |

### 7.2 冻结记录必填

- 已验证：M4 只读（1799/1799 measured，writes=0）、M5.2 单关节/夹爪/停止
  0.266 ms、M6 S13 21/21、S17 真实 MoveIt plan-only。
- 未验证：S14 底盘、完整 G8、S7/S9 残余、teleop 硬件、M7 外部模块试点。
- 契约 pin：`v0.2.0` / `b03a054` / `sha256:afca22a4…`。
- 镜像：`armpi-ultra:flatten-20261125`。
- 回滚：只读网关恢复步骤；不重启 vendor 容器。
- 证据索引：既有 `evidence/` 与 `e2e/reports/` 路径不变。

### 7.3 不动的东西

`jcheng-manipulation` 的标签、提交、清单、证据；ArmPi 的 `evidence/`、
`e2e/reports/`、`deploy/armpi-ultra/` 文件；`jcheng-mobile` 的代码。

## 8. 风险登记（增量）

| ID | 风险 | 可能性 | 影响 | 缓解 |
|---|---|---|---|---|
| R31 | 两套 vendor stack 同时在线，双权威重演 | 中 | 高 | 一次只接一个运动权威；切换记录发布者集合 |
| R32 | leader 被误接成运动源 | 中 | 高 | 只读适配器；静态 + 运行时零下发证明；不批准 `intent_source` |
| R33 | 在 v1 profile 中伪造 servo_id/脉冲 | 中 | 高 | v2 schema；review 拒绝伪造；测试断言 |
| R34 | AR5 厂商/型号判断错误 | 中 | 高 | P4 身份门禁；未确认前不改登记、不写代码 |
| R35 | EtherCAT 非 RT 内核，周期不达标 | 高 | 中 | 不承诺周期；先只读；必要时换主机 |
| R36 | pyAgxArm LGPL / Rokae 许可不清 | 中 | 中 | 依赖使用，不复制代码；发布前法务评审 |
| R37 | ArmPi 证据被当作新平台验证 | 中 | 高 | 每平台独立 evidence 目录；状态词表 |
| R38 | PiPER 夹爪方向/限位未实测 | 高 | 中 | 先总线读数确认；保守软限位；不照抄 URDF |
| R39 | 固定单元标定失效 | 中 | 中 | 标定文件哈希；mount 变更即失效 |
| R40 | ArmPi 仍通电可运动 | 低 | 高 | 撤销运动权威；只读网关；物理断电作为最终手段 |

R31/R32/R33 是本次转向的**停止条件**：发现任一情况，立即停实机并回到上一阶段。

## 9. 验收与证据规则

- 状态词表沿用 `README.md`：交付 `Complete / In progress / Planned / Not started`；
  验证 `Verified / Partially verified / Unverified / Inconclusive`；
  证据 `L0–L5`；仓库状态 `Experimental / Deployed / Frozen / Archived`。
- 每个平台独立证据目录；ArmPi 的 `Verified` 不迁移。
- `Verified` 必须有证据路径与日期；mock/仿真结果不得写成实机验证。
- 只有 P3 的实机循环可以标 `Verified`；P2/P5 只读只能标 `Partially verified`。
- README 与 `AGENTS.md` 的状态行只在 freeze record 与 P1 出口证据齐备后更新。

## 10. 需要确认的决策

- [ ] 接受 D1：ArmPi 平台 `Frozen`，不 `Archived`，仓库不冻。
- [ ] 接受 D2：先固定单元，移动化推迟到 P7。
- [ ] 接受 D3：PiPER X 为生产执行臂；leader 只读，不进控制回路。
- [ ] 接受 D4：AR5 先过身份门禁，未确认前不写后端。
- [ ] 接受 D5：契约一次升到 `v0.3.0`，含 `chain_id` 与夹爪范围去常量。
- [ ] 接受 D6：证据按平台重做，ArmPi `Verified` 不迁移。
- [ ] 指定 P3 的实机操作员与监督人。

## 11. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-11-28 | 初稿：冻结 ArmPi，PiPER X 生产臂，AR5 双臂后置；契约 v0.3.0 |
