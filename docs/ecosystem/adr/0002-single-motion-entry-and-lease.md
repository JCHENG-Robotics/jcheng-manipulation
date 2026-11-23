# ADR-0002：单一运动入口 + 租约仲裁

- 状态：提议
- 日期：2026-11-23

## 背景

v1.0.0 有六条实机写入路径：MoveIt CLI、Fast CLI、Studio G6、G7、G8、
Chassis pulse；此外还有脚本直写总线。`armpi_safety` 只否决 MoveIt CLI；
Studio 网关有自己的策略；Fast CLI 绕过两者。后端租约只存在于进程内，
网关没有租约概念，持有共享 token 可直接命令网关。

## 决策

所有实机运动只能通过 `jcheng_interfaces/action/ExecuteMotion` 提交到
`jcheng_motion` 仲裁器。仲裁器：

1. 校验契约、能力、租约、幂等、限流。
2. 调用 `jcheng_safety` 做状态/限位/轨迹/工作空间校验。
3. 由唯一执行器发布硬件命令。
4. 结果必须总线确认；未知则锁存。

控制权用租约表达：作用域 `arm|gripper|base|full`、优先级、TTL、
deadman、抢占、撤销。默认 `DISARMED`，需检查清单使能。

## 后果

正面：

- 一个入口可集中审计、限流、停止与复现。
- 租约可跨 UI/模块/进程，不依赖某个后端。
- 新模块没有第二条路。

负面：

- 仲裁器是关键路径，需要高可用与低延迟。
- 旧路径必须逐步退役，迁移期有双轨成本。
- 高频遥操作需要额外流式契约与 deadman。

## 替代方案

1. **保留多条路径，仅加文档约束。** 否决：无法证明“不能绕过”。
2. **让 `armpi_safety` 订阅所有命令话题并否决。** 否决：它只能观察，不能阻止
   已发布命令；且无法覆盖 MoveIt 直发控制器。
3. **只在网关做租约。** 否决：网关不是唯一入口，且当前网关没有租约概念。

## 证据

- `docs/phase2/MOTION_WRITE_PATHS.md` 六条路径与守卫表。
- `armpi_safety/policy.py` 只读 `profiles.moveit`。
- `backend/app/safety/lease.py` 进程内租约；`gateway/motion.py` 无 lease 字段。
- `gateway/server.py` 共享 token 可直接 `/motion/dispatch`。
