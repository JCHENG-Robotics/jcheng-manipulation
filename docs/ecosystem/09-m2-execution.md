# M2 执行记录

> 状态：**M2.1 契约审计稿已提交，等待签字；尚未定义 IDL。**
> 日期：2026-11-23
> 基线：`jcheng-manipulation v1.0.0`（`b3411e4`），未改动。

## 1. 本轮完成

- 审计 `arm_interfaces`：`ArmMove.action`、`ValidateTrajectory.srv` 的字段、
  生产者/消费者、单位与安全语义。
- 审计 Studio 后端 API：路由、请求/响应模型、租约、预览 token、幂等、
  状态机、安全 gate。
- 审计 gateway：`/state` 的三层位置语义、`/motion/dispatch`、
  `/motion/status`、`/chassis/pulse`、`/safety_stop`、错误码族。
- 审计安全执行路径：MoveIt CLI、Fast CLI、G6/G7/G8、底盘、脚本直写、
  厂商直发。
- 产出重复/冲突矩阵、兼容映射、破坏性变更、弃用计划、安全不变量、
  v0.1 建议清单与开放问题。

## 2. 分支与门禁

| 项 | 值 |
|---|---|
| 仓库 | `JCHENG-Robotics/jcheng-interfaces` |
| 分支 | `m2/contract-audit` |
| 提交 | `a0a5574` |
| 审计文档 | `docs/CONTRACT_AUDIT.md`（507 行） |
| M2 范围 | `docs/M2-SCOPE.md` |
| M1 门禁 | 通过（无 IDL、无 package、无 LICENSE） |
| CI | `m1-gate` 通过（run 45132889388） |

对比/建 PR：
https://github.com/JCHENG-Robotics/jcheng-interfaces/compare/main...m2/contract-audit?expand=1

## 3. 审计的核心结论

1. **`ArmMove` 不进入新契约。** 归档学习产物，单位未声明，仅 `arm_basics`
   自用；如需要单步运动，用 `JOINT_STEP`（rad）。
2. **`ValidateTrajectory` 语义保留，接口重做。** 新 `ValidateMotion` 不允许
   调用方放宽起点门限；显式速率必须与位置差分交叉校验；报告结构化。
3. **Studio `/api/*` 不逐字保留。** 新契约为 `/v1/*`；旧后端迁移期只做
   客户端/适配器。
4. **gateway `/motion/*` 不是外部契约。** 它是系统层硬件桥内部接口；
   对外不暴露。
5. **新契约必须更严格。** 总线实测、fail-closed、停止中止在飞执行、
   禁止原始 pulse/任意 duty/EEPROM 写入。
6. **重复/冲突主要集中在单位、限位、真值、完成判定与停止语义。**
   单一 robot profile + 命名 profile + 哈希是修复路径。

## 4. 签字门禁

审计稿第 13 节需要：

- 接口评审：`JerichoCheng`
- 系统/安全评审：`JerichoCheng` / `JerichoSwaine`
- Owner 批准：`JerichoCheng`

签字前：

- 不创建 `.msg/.srv/.action`。
- 不写最终 JSON Schema。
- 不发布 `jcheng_interfaces` v0.1。

## 5. 签字后的 M2.2

1. 按审计第 11 节建立 `jcheng_interfaces` ROS 包与类型。
2. 建立 JSON Schema、OpenAPI `/v1`、WebSocket `/v1`。
3. 建立 `jcheng_contracts` Python 包与 `interface_manifest.json`。
4. CI：Humble/Jazzy、Schema parity、契约哈希、SemVer diff、硬件原语扫描。
5. 记录与 `arm_interfaces`/Studio/gateway 的适配映射。

## 6. 未做

- 没有 IDL、Schema、ROS 包或运行时代码。
- 没有修改 `jcheng-manipulation v1.0.0` 的标签、证据或实验历史。
- 没有连接机器人、没有发送运动命令。
- 组织主页 PR 仍待打开；不影响 M2。

## 7. 需要确认

审计第 12 节的 10 个开放问题，重点：

1. `ValidationReport` 结构化字段是否足够替代 `report_json`？
2. `ArmMove` 是否立即弃用？
3. 夹爪契约用正向 0–2.22 rad？
4. 底盘 duty 用归一化 -1..1？
5. 遥操作流是否在 v0.1 预留？
6. HTTP 新 API 用 `/v1` 干净路径？
