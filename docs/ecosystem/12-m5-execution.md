# M5 执行记录（受监督单关节/夹爪；M5.1 Mock 部分）

> 状态：**M5.1 Mock ROS 传输完成（2026-11-24）；CI 全绿；M5.2 硬件执行待单独授权。**
> 分支：`jcheng-system` 的 `m5/supervised-motion`，提交 `4741356`。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`）未改动；`v0.1.0` 标签未改动。
> 约束：**M5.1 只使用 Mock 硬件；没有连接机器人、没有发布厂商命令、没有发送运动。**

## 1. 交付物

| 组件 | 内容 |
|---|---|
| `jcheng_runtime` | 单一 ROS 运行时：租约、安全、校验、能力、`ExecuteMotion` action |
| 服务 | `/jcheng/authority/*_lease`、`/jcheng/safety/{get_state,validate_motion,arm,disarm,reset}`、`/jcheng/capabilities` |
| Action | `/jcheng/motion/execute`（`jcheng_interfaces/action/ExecuteMotion`） |
| 停止 | `/safety_stop`（`std_msgs/Bool`，true → `STOP_LATCHED`，拒绝新命令） |
| 测试 | `tests_ros/test_runtime.py`：5 项服务/action/停止测试 |
| 门禁 | `scripts/check_m5_mock.sh`：Mock-only、无厂商命令路径 |
| CI | `m5-mock`：Humble 构建 11 包 + `pytest` + `tests_ros` + 门禁 |

## 2. M5.1 边界

- 默认 `DISARMED`；使能需要检查清单与确认短语。
- 所有运动必须携带有效租约；无租约/过期/抢占/撤销拒绝。
- `ValidateMotion` 只校验，不执行、不发布命令。
- `ExecuteMotion` 只操作 `MockStack`；完成来自 Mock 总线确认。
- `/safety_stop=true` 立即锁存并拒绝新请求。
- 没有 `servo_controller_msgs`/`ros_robot_controller_msgs` 导入、
  没有 `/servo_controller` 发布、没有串口/设备路径。
- M5.2 才接入 `/servo_controller` 硬件桥、单关节 0.05 rad、夹爪开/合；
  必须获得单独明确授权。

## 3. 验证结果

| 项 | 结果 |
|---|---|
| `pytest`（默认） | 28 项（27 pass / 1 conditional skip） |
| `pytest tests_ros` | 5 项通过 |
| 本地构建 | Jazzy 12 包（interfaces + 11 系统包）通过 |
| `m1-gate` / `m3-mock` / `m4-readonly` / `m5-mock` | 全部通过 |
| CI run | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45302412932 |
| `main` 保护 | `m1-gate` + `m3-mock` + `m4-readonly` + `m5-mock`，strict，管理员受限 |
| 实机 | **未连接、未发送运动命令** |

## 4. 未做

- 没有 M5.2 硬件桥；没有 `/servo_controller` 发布。
- 没有单关节/夹爪实机运动；没有停止传播实测。
- 没有域 0→41/42 桥接；M5.1 仅在进程内 Mock。
- 没有 `LICENSE`；许可证审查仍为 pending。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。

## 5. 后续

1. 评审并合并 `m5/supervised-motion`（M5.1）。
2. M5.2：在单独授权下接入硬件桥，先做单关节 0.05 rad 与夹爪开/合；
   清空工作区、支撑机械臂、操作员与监督人就位；旧网关保持只读。
3. 通过后再讨论 `jcheng-system v0.3.0` 标签。
4. 任何实机运动仍需单独、明确的现场授权。
