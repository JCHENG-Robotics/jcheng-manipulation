# M2 执行记录

> 状态：**M2 已完成（2026-11-23）。**
> PR #3 已由 `JerichoSwaine` 批准并合并；`v0.1.0` 已打 SSH 签名标签，
> 并已在 GitHub 显示 **Verified**。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`），未改动。

## 1. 已完成

### M2.1 契约审计

- 审计 `arm_interfaces`、Studio API、gateway、安全执行路径。
- 字段级盘点、重复/冲突矩阵、兼容映射、破坏性变更、弃用计划、
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
| Schema | 7 个 JSON Schema + OpenAPI `/v1` + WebSocket `/v1` + 52 个错误码/映射 |
| Python | `jcheng_contracts`：版本、错误码、模型、manifest、校验 |
| Manifest | `interface_manifest.json`；契约哈希 `sha256:0b8d72ced7f54d444d6851d8b99a51a78e2bed3f687324f51a15f76557f24b42` |
| 测试 | 15 项：manifest、Schema、错误码、IDL/Schema parity、OpenAPI、硬件原语 |

### M2.3 CI

- `m1-gate`：仓库结构/审计签字/无运行时代码/无 LICENSE。
- `m2-contract`：Humble 容器 `colcon build` + manifest + Schema + pytest +
  `ros2 interface show`。
- 运行 `45135205900`：`m1-gate` success，`m2-contract` success。
- `jcheng-interfaces` 的 `main` 保护已要求 `m1-gate` + `m2-contract`。

## 2. 合并与标签

| 项 | 值 |
|---|---|
| 仓库 | `JCHENG-Robotics/jcheng-interfaces` |
| PR | [#3](https://github.com/JCHENG-Robotics/jcheng-interfaces/pull/3)，作者 `JerichoCheng`，批准/合并 `JerichoSwaine` |
| 合并提交 | `33f1293bcdcc5c866fbd655232e26a1f3a756006` |
| Tag | `v0.1.0` |
| Tag object | `8bc17cd3debfd1badc451ec75a48c144a872ea2d` |
| Tag target | `33f1293`（与 `main` 一致） |
| Tag 类型 | SSH 签名 annotated tag |
| 本地验证 | `Good "git" signature for chengkewei77@gmail.com`；ED25519 fingerprint `SHA256:88tnes5V23WwLptWGXBaitgw4ti0Hb7aiqeT3wJHNlA` |
| GitHub 验证 | `verified=true, reason=valid`（2026-11-23T08:43:48Z） |
| CI | https://github.com/JCHENG-Robotics/jcheng-interfaces/actions/runs/45135205900 |

**说明：** 2026-11-23 已把 `~/.ssh/id_ed25519.pub` 注册为 GitHub **Signing Key**；
`v0.1.0` 无需重打，GitHub 已重新校验为 Verified。`unknown_key` 曾只表示
签名公钥未注册，不代表标签未签名。

## 3. 关键设计决定

1. `operator` 改为 `operator_id`，避免 C++ 保留字。
2. 契约不提供原始 pulse/寄存器/EEPROM/任意 duty；`BasePulse.duty` 归一化。
3. 夹爪契约正向 0–2.22 rad；URDF 负号由适配器转换。
4. `ValidateMotion` 返回结构化 `ValidationReport`；不允许调用方放宽起点。
5. `TeleopCommand` 在 v0.1 预留，试点时启用。
6. 错误码统一注册表；旧码通过 `error-code-mapping.v1.json` 映射。
7. `ContractVersion.contract_hash` 是运行时握手依据；manifest 逐文件哈希可重建。

## 4. 后续

1. 生成 TypeScript 绑定。
2. 补充 `ros_type_hashes`；`contract_hash` 仍是运行时权威。
3. 许可证与第三方依赖审查（pending）；正式采用前必须完成。

组织主页中英文 profile 已合并（PR #2，`16bddf2`），无待办。

## 5. 未做

- 没有运行节点、网关、硬件桥或部署单元。
- 没有连接机器人、没有发送运动命令。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。
- 没有添加 `LICENSE`；许可证审查仍为 pending。

## 6. 验收状态

| 项 | 状态 |
|---|---|
| 审计签字 | 完成 |
| 14 msg / 10 srv / 1 action 构建 | Jazzy 本地 + Humble CI 通过 |
| JSON Schema/OpenAPI/WS | 通过 |
| 契约哈希可重建 | 通过 |
| 15 项测试 | 通过 |
| `m1-gate` + `m2-contract` | 通过 |
| PR 合并 | 完成（`33f1293`） |
| v0.1.0 标签 | 完成（SSH 签名；GitHub Verified `valid`） |
