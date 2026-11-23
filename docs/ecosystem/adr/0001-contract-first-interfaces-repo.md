# ADR-0001：契约优先，单独 `jcheng-interfaces` 仓库

- 状态：提议
- 日期：2026-11-23
- 决策者：JCHENG Robotics 架构评审

## 背景

当前 `arm_interfaces` 只有一个 action 和一个 srv，且与 `armpi_safety`、
`armpi_grasp_planner` 同仓。跨仓库调用没有稳定契约：消息定义、HTTP API、
配置格式、错误码各自演化。ROS 2 的 message type hash 对字段变化敏感，
仅靠 SemVer 无法发现未升级的 IDL。

## 决策

建立独立的 `jcheng-interfaces` 仓库，作为唯一跨仓库编译期契约：

- 一个 ROS 包 `jcheng_interfaces`（msg/srv/action 原子版本化）。
- JSON Schema、OpenAPI、WebSocket 协议、配置与证据 Schema。
- 纯 Python `jcheng_contracts` 与生成的 TypeScript 类型。
- 契约哈希 `contract_hash` + `interface_manifest.json`。
- 兼容性 CI 与 SemVer 规则。

## 后果

正面：

- 所有消费者按同一契约构建，运行时可用哈希握手。
- 契约变更可独立评审、独立发布。
- 能力仓库接入不需要读系统源码。

负面：

- 任何 IDL 变更都要求消费者重建（ROS 2 类型哈希）。
- 多一个仓库与发布流程。
- 需要维护 Schema 与 IDL 的字段对应测试。

## 替代方案

1. **把契约放在 `jcheng-system`。** 否决：能力仓库会依赖系统仓库，方向错误。
2. **每个能力仓库自带消息。** 否决：组合爆炸，无法统一仲裁。
3. **只用 HTTP/JSON，不用 ROS IDL。** 否决：ROS 节点仍需要类型安全的本地契约；
   ROS IDL 与 JSON 可以并存。

## 证据

- `arm_interfaces` 的现状：仅 `ArmMove.action`、`ValidateTrajectory.srv`。
- `ros-ci.yml` 构建 7 个包，接口包与实现包同仓。
- ROS 2 类型哈希随字段变化；`release/manifest.json` 未包含接口包版本。
