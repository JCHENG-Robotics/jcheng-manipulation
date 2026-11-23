# M4 执行记录（只读接入）

> 状态：**M4 只读实机验收通过（2026-11-23）；修复分支待合并。**
> PR #3 已合并为 `f98152e`；30 分钟实机运行使用修复分支
> `m4/readonly-fixes` 的 `4f64df2`，该修复尚未合并到 `main`。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`）未改动；`v0.1.0` 标签未改动。
> 约束：**只读接入；没有发送任何运动命令，没有重启厂商容器。**

## 1. 交付物

| 组件 | 内容 |
|---|---|
| `jcheng_hardware/readonly.py` | 只读总线桥：舵机 ID 1..6 严格递增、0–1000 脉冲校验、无 publish/write API |
| `jcheng_telemetry` | 遥测缓存、`measured/commanded` 区分、只读安全视图、只读 HTTP 网关、预检、证据、短时 Mock 探针 |
| `jcheng_bringup/readonly.launch.py` | 只启动 telemetry 节点；不启动厂商容器或运动节点 |
| `configs/domains.yaml` | 域 0/41/42/43、进程归属、能力域隔离、只读策略 |
| `configs/readonly.yaml` | 订阅/发布/调用允许列表；命令话题与服务禁止列表 |
| `configs/robots/armpi_ultra/readonly_mapping.yaml` | 只读脉冲→弧度映射；来源哈希；读取顺序 1..6 |
| `configs/dds/*.xml` | CycloneDDS 单播域模板 |
| `scripts/check_m3_gate.sh` | 冻结 M3 包集的 Mock-only 门禁 |
| `scripts/check_m4_readonly.sh` | 只读门禁：无命令发布者、无写服务、无设备路径 |
| CI `m4-readonly` | 构建 interfaces + 10 个系统包、pytest、短时探针、只读门禁 |

## 2. 只读边界

- 只发布 `/jcheng/telemetry/state` 与 `/jcheng/telemetry/safety`。
- 只调用 `/ros_robot_controller/bus_servo/get_state`（`GetBusServoState`）；
  只设置 `get_position=1`；不读 offset/EEPROM/力矩，不写寄存器。
- 不打开 `/dev/tty*`、不导入 `serial`、不访问串口设备。
- `/controller_manager/joint_states` 标记 `truth=commanded`、
  `verification=timed`；只有新的总线样本标记 `truth=measured`、
  `verification=bus`。
- 总线读取默认 **4 Hz**；1 Hz 会让 `measured` 在 0.5 s 新鲜度阈值下
  每个采样点都过期（实机烟测发现）。
- 只读网关只有 GET；非 GET 在任何路由前返回 403；没有 motion/lease/
  arm/reset 路由。
- 只读安全视图只会是 `DISARMED` 或 `STOP_LATCHED`，不是控制权威。
- M4 遥测运行在厂商域 0；域 41/42 配置已准备，域 0→41/42 桥接留到 M5。

## 3. 实机只读验收（2026-11-23）

### 3.1 环境与结果

| 项 | 值 |
|---|---|
| 机器人 | `pi@10.42.0.218`，Raspberry Pi 5，Debian 12 |
| 容器 | `ArmPiUltra`（ROS 2 Humble）；厂商栈已运行，未重启 |
| 域 / RMW | 0 / `rmw_cyclonedds_cpp` |
| DDS | `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml` |
| 运行代码 | `m4/readonly-fixes` `4f64df2` |
| 时间 | 2026-11-23T09:47:11Z–10:17:09Z（17:47–18:17 +08） |
| 时长 | 1798.0 s（29 min 58 s） |
| 样本 | **1799 / 1799 `measured`，0 `commanded`** |
| 写入 | **`writes=0`** |
| 汇总 | **`ok=true`** |
| preflight | `ok=true` |
| 安全视图 | `DISARMED`，`stop_latched=false` |
| 位置变化 | 最大 8 pulses（joint2），其余 1–3 pulses；仅抖动 |

证据包：[`evidence/m4-readonly-20261123/`](evidence/m4-readonly-20261123/)。

### 3.2 总线真值 vs 旧网关

同一时刻的独立 `GetBusServoState` 读数与新节点逐项一致：

| servo | 独立读数 | 新节点 | 旧网关 `measured.raw_position` |
|---|---:|---:|---:|
| 1（夹爪） | 215 | 215 | 500 |
| 2（joint5） | 500 | 500 | 500 |
| 3（joint4） | 113 | 113 | 500 |
| 4（joint3） | 829 | 829 | 500 |
| 5（joint2） | 596 | 596 | 500 |
| 6（joint1） | 502 | 502 | 700 |

**关键发现：** 旧网关 `/state` 的 `measured` 来自
`/controller_manager/servo_states (command cache)`，标记 `truth=commanded`；
与真实总线最大相差 **387 pulses**（joint4）。这不是 M4 读错：独立只读
`GetBusServoState` 与新节点完全一致，直接验证了基线审计 G12
（命令缓存被当成实测）。因此 M4 以独立总线读数为对比基准；旧网关不提供
总线真值。

电池：新节点 7.665 V，旧网关 7.665 V，差 0 mV。

### 3.3 域隔离与发布者

域 42 只能看到 `/parameter_events` 与 `/rosout`；没有
`/servo_controller`、`/ros_robot_controller/bus_servo/set_position`、
`/ros_robot_controller/set_motor_speeds`、`/joint_controller` 或 `/cmd_vel`。

发布者检查：

- `/jcheng/telemetry/state`：唯一发布者 `jcheng_telemetry`
- `/servo_controller`：发布者 `kinematics`（厂商）
- `/ros_robot_controller/bus_servo/set_position`：发布者 `servo_manager`（厂商）

M4 节点没有发布任何厂商命令话题，也没有调用任何写服务。

### 3.4 相机/TF 限制

- `/depth_cam/rgb/image_raw` 当前 publisher count = 0；相机驱动未运行，
  所以只读相机元数据为空。这是机器人栈状态，不是 M4 写路径问题。
- `/tf` 由 `robot_state_publisher` 发布，健康正常。
- `/tf_static` 使用 TRANSIENT_LOCAL；M4 当前以 VOLATILE 订阅，未收到静态 TF。
  后续应改为 TRANSIENT_LOCAL，或用 `/tf` 覆盖。

## 4. 离线验证

| 项 | 结果 |
|---|---|
| 本地 pytest | 28 项（27 pass / 1 conditional skip） |
| 本地构建 | Jazzy 11 包（interfaces + 10 系统包）通过 |
| 短时只读探针 | `writes=0`、`measured_samples>0`、preflight `ok=true` |
| `m1-gate` / `m3-mock` / `m4-readonly` | 通过 |
| M4 PR #3 CI | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45146255614 |
| M4 修复分支 CI（代码 `4f64df2`） | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45149828614（通过） |
| M4 修复分支 CI（最终文档提交） | 2026-11-23 两个 run（45154996046、45155929815）均卡在 runner 的 `apt-get`；待 GitHub Actions 网络恢复后重跑 |
| `main` 保护 | `m1-gate` + `m3-mock` + `m4-readonly`，strict，管理员受限 |

## 5. 未做 / 待办

- **修复分支 `m4/readonly-fixes` 尚未合并到 `main`**；`main` 仍是 1 Hz
  总线默认值和 token-via-argv 版本，不能直接用于实机验收。
- 相机驱动未运行；`/tf_static` 的 QoS 需修正。
- 没有容器镜像/部署单元；没有 SROS2 权限。
- 没有运动 service/action、`ExecuteMotion`、租约服务或安全服务。
- 没有 `LICENSE`；许可证审查仍为 pending。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。

## 6. 后续

1. 打开并合并 `m4/readonly-fixes` 的 PR（4 Hz、token env、关闭守卫），
   使 `main` 与实际验收代码一致。代码提交 `4f64df2` 的 CI 已绿；最终文档
   提交的 CI 因 GitHub runner `apt-get` 卡住，需在 Actions 恢复后重跑。
2. 合并后再讨论 `jcheng-system v0.2.0` 标签。
3. 修正相机/TF 只读元数据（可选）：启用相机或改用 `/ext_cam`；
   `/tf_static` 改 TRANSIENT_LOCAL。
4. M5 才接入运动服务/action、租约、安全服务与受监督单关节/夹爪。
5. 任何实机运动仍需单独、明确的现场授权。
