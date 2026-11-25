# M6 执行记录（基线回放与 G8 复验）

> 状态：**M6：S13 回放 21/21；S17 真实 MoveIt plan-only 通过（2026-11-25）；G8/底盘实机待授权。**
> 分支：`jcheng-system` 的 `m6/replay-planning`，提交 `8e319b1`；
> PR #8 已合并为 `427214b`，`main` CI 绿；`m6/metrics` 已合并为 PR #11 → `78c613d`。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`）只读引用；证据不复制、不修改。
> 约束：S13 只回放 decision，不执行运动；S17 只规划；G8/底盘实机需单独明确授权。

## 1. S13 决策回放

`tools/replay_g8.py` 将 v1.0.0 的 joint targets 转为
`JOINT_TRAJECTORY`/`GRIPPER`，通过 `SafetySupervisor.validate` 重新判定，
比较基线 `preview.valid` 与新的 `approved`。

| 基线 | 命令数 | 一致 |
|---|---:|---:|
| `evidence/g8_acceptance_20261123/g8_run.json` | 11 | **11/11** |
| `evidence/studio_g8_fast/20261121/run2.json` | 10 | **10/10** |

- 允许时间戳差异；不比较物理落点。
- profile 限位为 4 位小数，基线精确值最大差 5.9e-5 rad；回放仅在边界
  做 ≤1e-3 rad 归一化并记录，**不修改运行时安全校验**。
- `tests/test_replay.py`：合成 valid/invalid 用例通过。
- 校验延迟（21 条命令）：mean **0.0166 ms**，p95/max **0.0352 ms**。
- 证据：[`evidence/m6-s13-20261124/s13-replay.json`](evidence/m6-s13-20261124/s13-replay.json)。

## 2. M6 其余部分

### S17 MoveIt 只规划（真实 MoveIt，2026-11-25 通过）

离线契约部分：

- `jcheng_moveit/plan_only.py`：构造 MoveGroup/GetMotionPlan 形状的 goal，
  强制 `planning_options.plan_only=true`；拒绝含 `execute`/`trajectory` 的 goal。
- `tests/test_moveit_plan_only.py`：正反用例通过；`check_m6_moveit.sh` 通过。

真实 MoveIt 部分（机器人容器 `ArmPiUltra`）：

- 安装 MoveIt 2.5.10 arm64；配置包 `armpi_ultra_moveit_config` 0.3.0 构建成功，
  `move_group` 仅加载 `ompl_interface/OMPLPlanner`。
- 机器人 Wi-Fi 在强制门户后；apt 经笔记本 `10.42.0.1:3128` 临时 HTTP 代理安装，
  完成后恢复原始 apt 源和 DNS。该网络细节不影响 S17 结论。
- 厂商配置要求 TRAC-IK，但 Ubuntu 22.04 arm64 没有可用包；本地副本改用
  `kdl_kinematics_plugin/KDLKinematicsPlugin`（保留 `kinematics.yaml.trac-ik`）。
- 启动：`move_group.launch.py allow_trajectory_execution:=false`，domain 42，
  CycloneDDS；未启动 controller manager / robot_state_publisher。
- Goal：group `arm`，start `joint1..5 = 0.0, 0.7309, -1.6127, -1.6591, 0.0`，
  target 仅 `joint1=0.10`（0.01 rad 容差），`plan_only=true`，OMPL RRTConnect。
- 结果：`error_code=SUCCESS(1)`，规划 **0.050016 s**，返回 **8 点** 轨迹。
- 无执行路径：`allow_trajectory_execution=False`；`/arm_controller` 和
  `/gripper_controller` 的 FollowJointTrajectory action 各 **0 个服务端**；
  无 `arm_controller`/`servo_controller`/`joint_trajectory` 命令 topic；未发送
  `/execute_trajectory` goal。MoveIt 日志中的 `executing..` 是 action 请求处理，
  不是轨迹执行。
- 证据：[`evidence/m6-s17-20261125/`](evidence/m6-s17-20261125/)。
- 未验证：轨迹执行、控制器桥接、带障碍物的规划、pose/Cartesian goal、多目标；
  真实响应尚未接入 `jcheng_runtime` 契约。

### S14 底盘脉冲

- Mock S14 已通过：`calibration_invalidated=true`，后续抓取拒绝。
- 实机底盘脉冲需清空工作区、确认标定失效、单独授权；本轮未做。

### G8 全流程

- 定位 -> 抓取 -> 抬升 -> 放回；每阶段总线确认；无 `UNKNOWN`。
- 落点门槛：基线快路径 2.27 mm；建议 ≤5 mm（需确认）。
- 需要相机、MoveIt、旧网关只读回滚、操作员/监督人；单独授权。
- 不修改 v1.0.0 的 G8 运行器/证据。

### 性能指标

- 校验延迟、执行延迟：待测。
- 停止延迟：M5.2 已测 hold 0.266 ms、客户端 48.4 ms。

## 3. 环境与依赖

- M5.2 修复已合并到 `main`（PR #7 → `d765ae3`）；M6 分支基于该修复。
- M6（`m6/replay-planning`）已合并为 `427214b`；`main` CI 绿。
- 校验延迟指标 `m6/metrics` 已合并为 PR #11 → `78c613d`。
- S17 真实 MoveIt 在机器人上完成；只规划，无运动命令，无控制器执行。
- `main` 保护包含 `m6-replay`。
- 未做 S14 底盘脉冲或 G8 全流程；任何实机运动仍需单独授权。

## 4. 后续

1. M6 S13/S17 离线部分已合并（`427214b`）；S17 真实 MoveIt plan-only 已通过。
2. 将真实 plan-only 响应接入契约/运行时，并补充 pose/Cartesian goal 测试。
3. 单独授权后做 S14 底盘脉冲与 G8 全流程，记录落点/总线证据。
4. M6 全部通过后再讨论 `v0.3.0`/M7；任何实机运动仍需单独授权。
