# M4 实机只读验收证据（2026-11-23）

> 结论：**通过（只读）**。30 分钟运行期间所有样本为 `truth=measured`、
> `verification=bus`，`writes=0`；没有发送任何运动命令，没有重启厂商容器。
> 运行代码：`jcheng-system` 分支 `m4/readonly-fixes`，提交 `4f64df2`。

## 1. 运行环境

| 项 | 值 |
|---|---|
| 机器人 | `pi@10.42.0.218`（Raspberry Pi 5，Debian 12） |
| 容器 | `ArmPiUltra`（ROS 2 Humble） |
| 厂商栈 | `bringup.launch.py` 已运行；**未重启** |
| 域 | 0 |
| RMW | `rmw_cyclonedds_cpp` |
| DDS | `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml` |
| 运行方式 | 容器内直接运行 `jcheng_telemetry_node`；localhost 只读网关 8767 |
| 开始 | 2026-11-23T09:47:11.922373Z（17:47 +08） |
| 结束 | 2026-11-23T10:17:09.923254Z（18:17 +08） |
| 时长 | 1798.0 s（29 min 58 s） |

## 2. 结果

`run30.summary.json`：

- `ok=true`
- `samples=1799`
- `measured_samples=1799`
- `commanded_samples=0`
- `truth_counts={"measured": 1799}`
- `writes=0`
- `max_gap_s=1.016`（证据采样 1 Hz；总线读取 4 Hz）
- preflight `ok=true`

运行期间 `/v1/health`：

- `status=ok`
- `mode=readonly`
- `writes=0`
- `state.truth=measured`
- `state.verification=bus`
- `safety.state=DISARMED`
- `stop_latched=false`

位置稳定性（30 分钟）：

| 关节 | rad 跨度 | 约 pulses | 结论 |
|---|---:|---:|---|
| joint1 | 0.012566 | 3 | 仅抖动 |
| joint2 | 0.033510 | 8 | 仅抖动 |
| joint3 | 0.012566 | 3 | 仅抖动 |
| joint4 | 0.012566 | 3 | 仅抖动 |
| joint5 | 0.004189 | 1 | 仅抖动 |
| r_joint（夹爪） | 0.012566 | 3 | 仅抖动 |

## 3. 总线真值与旧网关对比

同一时刻（2026-11-23T09:48:38Z）的独立 `GetBusServoState` 只读读数：

| servo | 独立读数 | 新节点换算 | 旧网关 `measured.raw_position` |
|---|---:|---:|---:|
| 1（夹爪） | 215 | 215 | 500 |
| 2（joint5） | 500 | 500 | 500 |
| 3（joint4） | 113 | 113 | 500 |
| 4（joint3） | 829 | 829 | 500 |
| 5（joint2） | 596 | 596 | 500 |
| 6（joint1） | 502 | 502 | 700 |

**关键发现：** 旧网关 `/state` 的 `measured` 字段实际来自
`/controller_manager/servo_states (command cache)`，`truth=commanded`。
它与真实总线最大相差 **387 pulses**（joint4）。这不是 M4 节点读错：
独立只读读数与新节点逐项一致。该结果直接验证了基线审计中的 G12
（命令缓存被当成实测）。

因此 M4 验收采用**独立 `GetBusServoState` 读数**作为对比基准；旧网关
不提供总线真值，不能作为对比真值。

电池：新节点 7.665 V，旧网关 7.665 V，差 0 mV。

## 4. 域隔离与发布者

域 42（能力域）只能看到：

```text
/parameter_events
/rosout
```

`domain42-result.txt`：`DOMAIN_ISOLATION_OK`。没有看到
`/servo_controller`、`/ros_robot_controller/bus_servo/set_position`、
`/ros_robot_controller/set_motor_speeds`、`/joint_controller` 或 `/cmd_vel`。

发布者检查（`publisher_check.txt`）：

- `/jcheng/telemetry/state`：唯一发布者 `jcheng_telemetry`
- `/servo_controller`：发布者 `kinematics`（厂商）
- `/ros_robot_controller/bus_servo/set_position`：发布者 `servo_manager`（厂商）

**M4 节点没有发布任何厂商命令话题，也没有调用任何写服务。**

## 5. 相机/TF 限制

- `/depth_cam/rgb/image_raw` 当前 publisher count = 0；相机驱动未运行，
  因此 `/v1/cameras` 没有数据。这是机器人栈状态，不是 M4 写路径问题。
- `/tf` 有 `robot_state_publisher` 发布，健康正常。
- `/tf_static` 使用 TRANSIENT_LOCAL QoS；M4 节点当前用 VOLATILE 订阅，
  未收到静态 TF。后续应改为 TRANSIENT_LOCAL，或通过 `/tf` 覆盖。

## 6. 安全声明

- 运行期间没有发送任何运动命令。
- 没有调用 `SetBusServoState`、`SetPWMServoState`、`set_motor_speeds`、
  `/servo_controller` 发布、`/grasp`、`/move_action` 或 `/cmd_vel`。
- 没有打开串口/设备；没有重启厂商容器。
- 旧网关 token 未写入证据；新网关 token 通过环境变量传递，未进入 argv。
- 首次烟测使用的本地只读 token 已轮换。

## 7. 文件

| 文件 | 内容 |
|---|---|
| `run30.summary.json` | 30 分钟汇总 |
| `run30.jsonl` | 1799 条证据样本（未压缩） |
| `run30.jsonl.gz` | 同上，gzip |
| `run30.log` | 节点日志 |
| `compare_20261123.json` | 独立总线读数、新节点状态、旧网关缓存对比 |
| `publisher_check.txt` | 发布者检查 |
| `domain42-topics.txt` / `domain42-result.txt` | 域 42 隔离证据 |
| `SHA256SUMS` | 文件哈希 |

`SHA256SUMS` 记录未压缩 JSONL 的哈希；可用 `gzip -dc run30.jsonl.gz | sha256sum`
复核。
