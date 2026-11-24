# M6 执行记录（基线回放与 G8 复验）

> 状态：**M6 离线开始：S13 回放 21/21 decision 一致；S17/G8/底盘实机待授权。**
> 分支：`jcheng-system` 的 `m6/replay-planning`，提交 `eba0e73`。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`）只读引用；证据不复制、不修改。
> 约束：S13 只回放 decision，不执行运动；G8/底盘实机需单独明确授权。

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
- 证据：[`evidence/m6-s13-20261124/s13-replay.json`](evidence/m6-s13-20261124/s13-replay.json)。

## 2. M6 其余部分

### S17 MoveIt 只规划

- 目标：`/plan_kinematic_path` 或 `/move_action` + `planning_options.plan_only=true`；
  不执行。
- 尚未接入真实 MoveIt/move_group；需要录制 goal 或实际 move_group 验证。
- MoveIt 执行路径必须关闭或无硬件发布权限。

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

- M6 分支基于 `m5.2/final-acceptance`（`822b05d`），包含夹爪完成语义
  修复；`main` 仍是 `c4f46b2` 旧语义，需先合并 M5.2 修复。
- CI `m6-replay` 全绿；`main` 保护新增 `m6-replay`。
- 未连接机器人；没有运动。

## 4. 后续

1. 合并 M5.2 修复与 M6 S13 分支。
2. 补 S17 MoveIt plan-only 验证。
3. 单独授权后做 S14 底盘脉冲与 G8 全流程，记录落点/总线证据。
4. M6 全部通过后再讨论 `v0.3.0`/M7；任何实机运动仍需单独授权。
