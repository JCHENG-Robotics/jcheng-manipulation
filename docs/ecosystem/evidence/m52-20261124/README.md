# M5.2 受监督实机证据（2026-11-24）

> 结论：**单关节通过；夹爪开动作未通过总线容差，已锁存停止。M5.2 未完成。**
> 没有自动重置或重试；没有继续发送运动命令。
> 运行代码：`jcheng-system` 分支 `m5/vendor-motion`，提交 `83a50a0`。

## 1. 前置与安全状态

- 用户授权：M5.2 实机单关节与夹爪测试；工作区已清空、机械臂已支撑、监督人就位。
- 机器人：`pi@10.42.0.218`，容器 `ArmPiUltra`，ROS Humble，域 0。
- 前置检查：`GetSafetyState=DISARMED`、`ArmSystem=ARMED_IDLE`、租约 granted、
  `ValidateMotion=approved`。
- 发现机器人内存/负载异常：旧网关与辅助相机占用约 7 GB，SSH 多次失去响应。
  停止了 `armpi-gateway.service` 与 `armpi-ext-cam.service` 以释放内存；
  厂商运动栈未重启。测试结束后重启了只读 `armpi-gateway.service`，
  保持 `armpi-ext-cam.service` 停止。

## 2. 结果

### 2.1 单关节 joint1 +0.05 rad

- 目标：joint1 0.05 rad（512 pulse），duration 2.0 s，profile step。
- 结果：`COMPLETED`，`bus_verified=true`，`safety=ARMED_IDLE`。
- 总线实测：joint1 = 505 pulse（目标 512，差 7 pulses，容差 8）。
- 实际关节位移约 0.0377 rad（从 496 到 505 pulse），在容差内。

### 2.2 夹爪开

- 目标：`gripper.position_rad=2.22`（170 pulse），duration 2.0 s。
- 结果：`UNKNOWN`，`reason_code=MOTION_OUTCOME_UNKNOWN`，
  `safety=STOP_LATCHED`。
- 总线实测：servo1 = 183 pulse；与目标 170 差 **13 pulses**，超过 ±8 容差。
- 安全动作：机械臂保持，夹爪停在 183；闭合命令
  `m52-gripper-close-001` 被 `SAFETY_STOP_LATCHED` 拒绝。

最终总线（servo 1..6）：

```text
183, 500, 110, 831, 597, 507
```

对应：gripper 183，joint5 500，joint4 110，joint3 831，joint2 597，
joint1 507。

## 3. 关键发现

- 单关节闭环验证链路可用：action → `/servo_controller` → 总线复核 →
  `COMPLETED`。
- 夹爪全开目标 170 在当前机械状态下不可达/存在卡滞，实际停在 183；
  这可能是机械开限位或夹爪连杆卡滞，不应通过放宽容差来“通过”。
- M5.2 验收因此**未完成**；需要人工检查夹爪开限位/连杆，确认安全的
  可达开度（例如约 183 pulse / 2.16 rad），再单独授权重试。
- 旧网关的只读入口已恢复；辅助相机仍停止，避免再次内存耗尽。

## 4. 安全声明

- 夹爪 `UNKNOWN` 后没有自动重试、没有重置、没有继续运动。
- 没有重启厂商容器；没有写 EEPROM/ID/offset/限位。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。
- 下一次实机动作需要人工检查夹爪并获得新的明确授权。

## 5. 文件

| 文件 | 内容 |
|---|---|
| `m52-preflight.json` | 使能/租约/校验结果 |
| `m52-step-result.json` | 单关节结果 |
| `m52-gripper-result.json` | 夹爪开 UNKNOWN、闭合拒绝 |
| `m52-final-state.json` | 最终安全状态与总线 |
| `m52-run.jsonl` | 动作证据（2 条 motion record） |
| `m52-run.log` | vendor 节点日志 |
| `m52-smoke.log` | 前置启动日志 |
| `SHA256SUMS` | 文件哈希 |

M5.2 代码 CI：https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45306784954
（`m5-vendor` 等全部通过）。
