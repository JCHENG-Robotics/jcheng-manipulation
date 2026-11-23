# M2 执行记录

> 状态：**M2.1 审计已签字；M2.2 v0.1 契约与 M2.3 CI 已实现，等待 PR 评审/合并与 v0.1.0 标签。**
> 日期：2026-11-23
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`），未改动。

## 1. 已完成

### M2.1 契约审计

- 审计 `arm_interfaces`、Studio API、gateway、安全执行路径。
- 产出字段级盘点、重复/冲突矩阵、兼容映射、破坏性变更、弃用计划、
  安全不变量与 v0.1 建议清单。
- 审计 PR #2 由 `JerichoSwaine` 批准并合并（`b2cfcf1`）；
  `docs/CONTRACT_AUDIT.md` 已标记签字。

### M2.2 v0.1 契约

| 项 | 内容 |
|---|---|
| ROS 包 | `ros2_ws/src/jcheng_interfaces` |
| 消息 | 14 个：`ContractVersion`、`CapabilityManifest`、`SafetyState`、`ControlLease`、`MotionCommand`、`MotionFeedback`、`MotionResult`、`RobotState`、`JointTrajectory`、`JointTrajectoryPoint`、`BasePulse`、`GripperCommand`、`TeleopCommand`、`ValidationReport` |
| 服务 | 10 个：租约 4 个、校验 1 个、安全/使能 4 个、能力 1 个 |
| Action | `ExecuteMotion` |
| Schema | 7 个 JSON Schema + OpenAPI `/v1` + WebSocket `/v1` + 错误码注册表/映射 |
| Python | `jcheng_contracts`：版本、错误码、模型、manifest、校验 |
| Manifest | `interface_manifest.json`；契约哈希 `sha256:0b8d72ce…f24b42` |
| 测试 | 15 项：manifest、Schema、错误码、IDL/Schema parity、OpenAPI、硬件原语 |

### M2.3 CI

- `m1-gate`：仓库结构/审计签字/无运行时代码/无 LICENSE。
- `m2-contract`：Humble 容器 `colcon build` + manifest + Schema + pytest +
  `ros2 interface show`。
- 运行 `45134699521`：`m1-gate` success，`m2-contract` success。
- `jcheng-interfaces` 的 `main` 保护已要求 `m1-gate` + `m2-contract`。

## 2. 分支与提交

| 项 | 值 |
|---|---|
| 仓库 | `JCHENG-Robotics/jcheng-interfaces` |
| 分支 | `m2/interfaces-v0.1` |
| 提交 | `b50fc3e`（v0.1 实现）、`7608500`（CI 修复） |
| CI | https://github.com/JCHENG-Robotics/jcheng-interfaces/actions/runs/45134699521 |
| PR | https://github.com/JCHENG-Robotics/jcheng-interfaces/compare/main...m2/interfaces-v0.1?expand=1 |

## 3. 关键设计决定

1. `operator` 改为 `operator_id`，避免 C++ 保留字。
2. 契约不提供原始 pulse/寄存器/EEPROM/任意 duty；`BasePulse.duty` 归一化。
3. 夹爪契约正向 0–2.22 rad；URDF 负号由适配器转换。
4. `ValidateMotion` 返回结构化 `ValidationReport`；不允许调用方放宽起点。
5. `TeleopCommand` 在 v0.1 预留，试点时启用。
6. 错误码统一注册表；旧码通过 `error-code-mapping.v1.json` 映射。
7. `ContractVersion.contract_hash` 是运行时握手依据；manifest 逐文件哈希可重建。

## 4. 剩余步骤

1. 由 `JerichoCheng` 作者打开 PR，`JerichoSwaine` 批准，合并到 `main`。
2. 合并后创建 v0.1.0 标签（签名标签；需确认签名方式）。
3. 可选：生成 TypeScript 绑定与 ROS type hash（M2.3 后续）；`contract_hash`
   仍是运行时握手的权威，不影响 M3 使用 Python/ROS。
4. 组织主页 PR 仍待打开；不阻塞 M2。

## 5. 未做

- 没有运行节点、网关、硬件桥或部署单元。
- 没有连接机器人、没有发送运动命令。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。
- 没有添加 `LICENSE`；许可证审查仍为 pending。
- `interface_manifest.json` 的 `ros_type_hashes` 仍为空；这是 v0.1 已知限制，
  后续补充。TypeScript 绑定同样未生成。

## 6. 验收状态

| 项 | 状态 |
|---|---|
| 审计签字 | 完成 |
| 14 msg / 10 srv / 1 action 构建 | Jazzy 本地 + Humble CI 通过 |
| JSON Schema/OpenAPI/WS | 通过 |
| 契约哈希可重建 | 通过 |
| 15 项测试 | 通过 |
| `m1-gate` + `m2-contract` | 通过 |
| PR 合并 + v0.1.0 标签 | 待完成 |
