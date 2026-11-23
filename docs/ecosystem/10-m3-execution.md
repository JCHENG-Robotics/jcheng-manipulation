# M3 执行记录

> 状态：**M3 已完成并合并（2026-11-23）。**
> PR #2 已由 `JerichoSwaine` 批准、`JerichoCheng` 合并；`main` 为 `40a74ba`，
> `main` CI 通过；`m3/system-mock` 分支已删除。
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`），未改动。
> 约束：纯 Mock；不连接机器人、不写硬件、不实现 ROS 传输。

## 1. 交付物

| 组件 | 内容 |
|---|---|
| `jcheng_system_core` | 纯 Python Mock 核心：配置、安全状态机、租约、仲裁、Mock 硬件、guard、HTTP 网关、证据、S1–S17 |
| `jcheng_safety` | 安全诊断 CLI |
| `jcheng_authority` | 租约诊断 CLI |
| `jcheng_motion` | 仲裁诊断 CLI |
| `jcheng_hardware` | Mock 硬件状态 CLI |
| `jcheng_gateway` | HTTP 网关入口 |
| `jcheng_guard` | 发布者白名单诊断 |
| `jcheng_testkit` | 场景运行器 |
| `jcheng_bringup` | Mock launch |
| `locks/ecosystem.lock.yaml` | interfaces v0.1.0、契约哈希、profile 哈希 |
| `tools/verify_lock.py` | 锁文件/契约/profile/远程 tag 校验 |

## 2. 验收结果

| 项 | 结果 |
|---|---|
| 包构建 | Jazzy 本地 9 包通过；Humble CI 通过 |
| pytest | 16 项通过 |
| S1–S17 | **15 pass / 2 skipped / 0 failed** |
| 必选场景 | S1–S12、S14、S15、S16 全部 pass |
| 跳过 | S13（M6 回放）、S17（M4/M6 MoveIt） |
| 非仲裁器发布者 | 0（S1/S10 断言） |
| 锁文件 | `LOCK OK`，含远程 tag 与证据校验 |
| `m1-gate` | 通过（Mock-only、无硬件访问、无 LICENSE） |
| `m3-mock` | 通过（Humble 构建 + 场景 + 锁文件） |
| `main` CI | 通过（run 45139635986） |

## 3. 分支与 CI

| 项 | 值 |
|---|---|
| 仓库 | `JCHENG-Robotics/jcheng-system` |
| 分支 | `m3/system-mock`（PR head `90c3904`；已合并并删除） |
| 合并后 `main` | `40a74ba955ea96bad288b41e38f58e7bbfb94803` |
| CI（PR） | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45138929506 |
| CI（`main`） | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45139635986 |
| `main` 保护 | `m1-gate` + `m3-mock`，1 个 code-owner 批准，管理员受限 |
| PR | [#2](https://github.com/JCHENG-Robotics/jcheng-system/pull/2)，批准 `JerichoSwaine`，合并 `JerichoCheng` |

**关于 compare 页面：** `main...m3/system-mock` 显示 “There isn't anything to
compare” 是预期结果。PR #2 已合并、分支已删除，`main` 已包含 M3；不是漏提交。

## 4. 关键设计决定

1. M3 核心是纯 Python；ROS 包只提供诊断 CLI、gateway/testkit 入口与 bringup。
2. `rclpy`/`rclcpp` 在 M3 仓库门禁中禁止；ROS service/action 传输留到 M4。
3. 只有 `MockHardwareBridge` 发布 Mock 命令；guard 允许列表仅
   `jcheng_hardware`。
4. 安全状态机默认 `DISARMED`；状态过期、停止锁存、无租约、错误契约/
   profile、guard 违规、总线失败均拒绝或锁存。
5. 底盘脉冲是 `open_loop`，结果标记 `calibration_invalidated=true`。
6. 契约哈希固定为 interfaces v0.1.0：
   `sha256:0b8d72ced7f54d444d6851d8b99a51a78e2bed3f687324f51a15f76557f24b42`。

## 5. 未做

- 没有 ROS service/action 传输、域 41/42/43、SROS2。
- 没有真实 MoveIt、总线、底盘、相机或厂商节点。
- 没有连接机器人、没有真实运动。
- 没有 `LICENSE`；许可证审查仍为 pending。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。

## 6. 后续

1. 可打 `jcheng-system v0.1.0` 标签（Mock 版本；尚未打，需单独确认）。
2. M4：只读机器人接入；域/容器/SROS2/guard 运行时；**不运动**。
3. M5 起任何真实运动仍需单独、明确的现场授权。

## 7. 其他待办

无。M2 `v0.1.0` 标签已验证；组织主页 PR #2 已合并（`16bddf2`）。
