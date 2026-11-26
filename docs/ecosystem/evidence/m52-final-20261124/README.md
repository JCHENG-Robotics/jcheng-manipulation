# M5.2 最终实机验收（2026-11-24）

> 结论：**M5.2 核心功能验收通过**；夹爪采用基线语义（定时完成 + 总线证据），
> 手臂仍使用 ±8 pulse 精确容差。S7 状态过期硬件注入与 S9 运行时发布者检测
> 仍未单独实机验证。

## 1. 根因与修复

- 基线 `armpi_debug_gateway` 对纯夹爪命令的完成定义是**插值时间完成**，
  并用 `GetBusServoState` 读数作为 `verification=bus` 证据；夹爪会接触/停位，
  不要求精确 pulse。
- M5.2 初版错误地把手臂的 ±8 pulse 精确容差也用于夹爪，导致三次误判
  `UNKNOWN`（170→183、194→207、640→620/619）。
- 修复：手臂保持 ±8；夹爪改为定时完成 + 总线读数 + 方向/最小位移 ≥5 pulse。
  没有放宽手臂容差。

## 2. 夹爪开/合（通过）

| 动作 | 起点 | 目标 | 实测 | 结果 |
|---|---:|---:|---:|---|
| 闭合 | 183 | 700 | 645 | `COMPLETED`，`bus_verified=true` |
| 打开 | 647 | 170 | 183 | `COMPLETED`，`bus_verified=true` |

消息分别为：

```text
gripper timed completion with bus evidence: start=183 target=700 measured=645
gripper timed completion with bus evidence: start=647 target=170 measured=183
```

## 3. 停止传播（通过）

- joint1 +0.05 rad，duration 5 s；1 s 后 `/safety_stop=true`。
- action：`ABORTED`、`SAFETY_STOP_LATCHED`、`stop_latched=true`。
- 节点 hold 延迟：**0.266 ms**（≤100 ms）。
- 客户端 stop→result：**48.4 ms**。

## 4. 停止运行时后拒绝（通过）

- 停止 `jcheng_vendor_runtime` 后，`/jcheng/motion/execute` action server
  不存在；客户端 `wait_for_server` 超时，`rejected=true`。
- 没有 fail-open 路径。

## 5. 无租约/越限（通过，纯校验）

- 无租约：`approved=false`、`NO_LEASE`、`control lease is not valid`。
- 越限 joint1 2.0 rad：`approved=false`、`JOINT_LIMIT`、`joint1 outside limits`。
- 未执行 action，无运动。

## 6. 最终状态

- 最终总线：`183, 501, 108, 834, 590, 498`（servo 1..6）。
- 运行时已停止；机械臂保持支撑姿态。
- 厂商运动容器未重启。

## 7. 残余

- S7（状态过期注入）与 S9（运行时未授权发布者检测）未在实机单独验证；
  M3 Mock 场景覆盖，但 M5.2 的运行时 guard 仍是静态允许列表。
- 旧网关/辅助相机为内存稳定暂时停止，测试后未全部恢复。
- M5.2 代码修复在 `m5.2/final-acceptance` 分支，主分支尚未合并。

## 8. 文件

| 文件 | 内容 |
|---|---|
| `m52-run5-result.json` | 夹爪闭/开结果 |
| `m52-stop-result.json` | 停止 action 结果 |
| `m52-stop-final.json` | 停止后总线/安全状态 |
| `m52-after-stop.json` | 停止运行时后 action server 不可用 |
| `m52-run6-reject.json` | 无租约/越限拒绝 |
| `m52-run5.jsonl` | 夹爪/停止证据 |
| `m52-run5.log` / `m52-run6.log` | 节点日志 |
| `SHA256SUMS` | 文件哈希 |
