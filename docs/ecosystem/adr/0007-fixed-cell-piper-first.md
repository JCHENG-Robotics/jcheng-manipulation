# ADR-0007：固定单元优先，PiPER X 生产臂，leader 只读

- 状态：已接受
- 日期：2026-11-28
- 批准：2026-11-28，负责人
- 决策者：JCHENG Robotics 架构评审

## 背景

ArmPi Ultra 是 5-DoF 总线舵机移动平台。生态在它上面完成了 M1–M5.2：
只读遥测 1799/1799 `measured`、单关节与夹爪受监督执行、停止 0.266 ms；
M6 的 S13 回放 21/21、S17 真实 MoveIt plan-only 通过；S14 底盘与完整 G8
从未开始。`jcheng-manipulation v1.0.0` 是它的冻结基线。

2026-11-28，负责人决定冻结 ArmPi Ultra，转向 AgileX PiPER X（6-DoF，CAN）
与 AR5（7-DoF，EtherCAT）双臂。三个拓扑选择在同一天确定，并约束后续所有
阶段：

1. 先做固定单元，移动化推迟；
2. PiPER X 作为生产执行臂；
3. PiPER leader 只读，不进入控制回路。

这三项难以回退：它们决定部署形态、契约形状与安全案例的范围。如果不写成
ADR，后续里程碑会反复重新讨论，或在实现中无意违背。

## 决策

### 1. 固定单元优先

P0–P6 不包含底盘、Nav2、`/cmd_vel` 或移动标定。`jcheng-mobile` 保持
`Experimental` 休眠；移动化作为 P7 单独立项。固定单元的标定、ops 与
perception 部署按无底盘形态设计。

### 2. PiPER X 作为生产执行臂

主路径是**示教 → 录制 → 离线回放 → 受监督执行**，不是实时主从闭环。
leader 用于示教采集，follower 在租约下执行录制的 `JOINT_TRAJECTORY`。
回放复用 S13 的 `SafetySupervisor` 判定；执行走 `ExecuteMotion`，
逐帧由 CAN 读数确认。生产 recipe 生成同一种轨迹，与回放共用执行器。

### 3. leader 只读

leader 臂永不使能、永不接收运动命令。示教捕获在 operator 侧运行，
不属于 system，也不发布运动意图。follower 是唯一被控对象，只能通过
`ExecuteMotion` 到达。`intent_source`（teleop P-006）保持提案状态，
等 P3.5 实时遥操作另行批准。

### 配套边界：AR5 身份门禁

AR5 接入以六项事实为前置：厂商与型号、控制器拓扑、SDK 与许可、
EtherCAT 主站与实时性、URDF/关节符号/限位、夹爪接口。任一项未知，
AR5 保持 `planned`，不写后端、不改 `rokae.ar5*` 登记。

## 后果

正面：

- 去掉底盘与导航，少一层标定、看门狗和移动安全论证。
- leader 不进控制回路，没有实时闭环、deadman 调参和双权威风险；
  安全案例只需覆盖 follower 的租约、限位、停止与逐帧确认。
- 复用已有资产：S13 回放判定、teleop capture core、PiPER 的 MIT URDF、
  仿真目录、learning/teleop adapter。
- 固定单元的基座标定稳定，外部相机与手眼不需要随底盘复标。
- PiPER 的 CAN 接口比 AR5 的 EtherCAT 简单，可以先落地再处理双臂。

负面：

- 失去移动演示能力；`jcheng-mobile` 的 ICP/Nav2 工作暂时没有消费方。
- 示教-回放不适合高度动态的环境；场景变化需要重新示教。
- PiPER X 的 payload、repeatability、夹爪方向与限位均未实测；生产节拍
  不能从 URDF 或厂商页推断。
- 契约仍要升到 `v0.3.0`：robot-profile v2、`chain_id`、夹爪范围去常量；
  这不是零成本迁移。
- 如果 AR5 身份长期未确认，双臂目标会整体后移，PiPER 单臂成为事实上限。

## 替代方案

1. **保留 ArmPi 移动平台，继续 M6 S14/G8。** 否决：平台已决定退役，
   S14/G8 的证据不会迁移到新硬件；继续投入没有验收价值。
2. **以实时主从作为主路径。** 否决：需要 `intent_source`、流式执行器、
   deadman 与频率调参；对固定单元的生产执行没有必要性，且显著扩大安全案例。
3. **让 leader 直接驱动 follower，或把 leader 注册为 motion source。**
   否决：绕过租约、限位与停止锁存，制造第二个权威。
4. **先 AR5 后 PiPER。** 否决：AR5 本地零资产、SDK/控制器未确认；
   PiPER 有硬件、MIT 模型和现成 adapter，能先产生实机证据。
5. **双臂用扁平 14 关节 profile，不引入 `chain_id`。** 保留为备选：
   如果 `chain_id` 的契约变更成本超出预算，可用前缀命名先落地；代价是
   租约无法分臂、停止粒度粗。

## 证据

- 设计记录：[`17-platform-pivot.md`](../17-platform-pivot.md)。
- `jcheng-interfaces/schemas/robot-profile.v1.schema.json`：`servo_id` 必填、
  夹爪脉冲必填、`profiles.base` 必填。
- `jcheng-system/ros2_ws/src/jcheng_vendor/jcheng_vendor/executor.py:41`：
  `HardwareBridge` 是脉冲端口，无法表达 CAN/EtherCAT/多链。
- `jcheng-system/ros2_ws/src/jcheng_moveit/jcheng_moveit/plan_only.py:13`：
  `group="arm"`、`tip="end_effector_link"` 写死。
- `jcheng-devices/registry/devices/agilex.piper-x.yaml`：`status: candidate`，
  限位来自仿真 profile，未实测。
- `jcheng-devices/registry/devices/rokae.ar5.yaml`、`ar5l.yaml`、`ar5r.yaml`：
  vendor-public 事实，`data_status: missing`，无本地 SDK/URDF/标定。
- `jcheng-simulation/docs/robots/piper_x/`：MIT 资产管线，618 文件校验通过；
  仅编目，未接入 launch。
- `jcheng-system/ROADMAP.md`：M4/M5.2/M6 的已验证数字与 S14/G8 的未开始状态。
