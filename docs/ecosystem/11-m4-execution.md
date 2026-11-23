# M4 执行记录（只读接入，离线部分）

> 状态：**M4 离线实现完成（2026-11-23）；CI 全绿；实机 30 分钟只读验收待单独授权。**
> 分支：`jcheng-system` 的 `m4/readonly-integration`，最终提交 `58c8695`。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`）未改动；`v0.1.0` 标签未改动。
> 约束：**没有连接机器人、没有打开串口、没有发布厂商命令、没有发送任何运动命令。**

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
- 只读网关只有 GET；非 GET 在任何路由前返回 403；没有 motion/lease/
  arm/reset 路由。
- 只读安全视图只会是 `DISARMED` 或 `STOP_LATCHED`，不是控制权威。
- M4 遥测运行在厂商域 0；域 41/42 配置已准备，域 0→41/42 桥接留到 M5。

## 3. 验证结果

| 项 | 结果 |
|---|---|
| 本地 pytest | 28 项（27 pass / 1 conditional skip） |
| 本地构建 | Jazzy 11 包（interfaces + 10 系统包）通过 |
| 短时只读探针 | `writes=0`、`measured_samples>0`、preflight `ok=true` |
| `m1-gate` | 通过 |
| `m3-mock` | 通过（M3 未回归） |
| `m4-readonly` | 通过 |
| CI run | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45144116055 |
| `main` 保护 | `m1-gate` + `m3-mock` + `m4-readonly`，strict，管理员受限 |
| 实机 | **未连接、未发送运动命令** |

## 4. 未做

- 没有在机器人上运行；没有 30 分钟只读验收。
- 没有总线读数与旧网关的实机对比；没有能力域运行时隔离验证。
- 没有容器镜像/部署单元；没有 SROS2 权限。
- 没有运动 service/action、`ExecuteMotion`、租约服务或安全服务。
- 没有 `LICENSE`；许可证审查仍为 pending。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。

## 5. 后续

1. `JerichoSwaine`/`JerichoCheng` 评审并合并 M4 PR。
2. 单独授权后执行实机只读：≥30 分钟、`truth=measured`、总线脉冲与旧网关
   同刻对比、能力域看不到厂商命令话题、证据 `writes=0`。
3. 实机只读通过后再讨论 `jcheng-system v0.2.0` 标签。
4. M5 才接入运动服务/action、租约、安全服务与受监督单关节/夹爪。
5. 任何实机运动仍需单独、明确的现场授权。
