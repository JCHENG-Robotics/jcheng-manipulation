# ArmPi Debug Studio — G0 Discovery 交付包

> 状态：**Analysis complete / 待 Board 评审**
> 日期：2026-11-13
> 基线：`ARMPI_DEBUG_STUDIO_PRD_V1.md`（v1.0, 2026-11-13）
> 范围：只读源码审计与设计，不含实现代码、不含实机运动

## 证据等级

本目录所有结论标注证据等级：

| 等级 | 含义 |
|---|---|
| `Verified in source` | 已在厂商源码中逐行确认，附路径/行号 |
| `Hardware validated` | 已在实机验证（本阶段 **无** 此类结论） |
| `Proposed` | 设计建议，尚未实现 |
| `Unknown` | 现有文件无法确认，必须实测或问厂商 |

**本阶段不存在任何 `Hardware validated` 结论。** 所有实机行为均为 `Unknown`。

## 文档清单

| 文档 | 对应 PRD 交付物 | 内容 |
|---|---|---|
| [01_vendor_file_inventory.md](01_vendor_file_inventory.md) | 输入文件盘点 | 完成项目需要哪些厂商文件、哪些不要、各自作用 |
| [02_sdk_api_audit.md](02_sdk_api_audit.md) | `sdk_api_audit.md` | SDK 真实 API、返回值、异常、阻塞行为、能力边界 |
| [03_driver_interface_inventory.md](03_driver_interface_inventory.md) | `driver_interface_inventory.md` | ROS 2 节点/Topic/Service/Action 清单与数据真实性 |
| [04_joint_mapping_risks.md](04_joint_mapping_risks.md) | `joint_mapping_risks.md` | 关节↔舵机映射、单位、限位冲突与验证计划 |
| [05_safety_gap_analysis.md](05_safety_gap_analysis.md) | `safety_gap_analysis.md` | SR-01…SR-10 逐条差距、故障场景、UI 语义红线 |
| [06_architecture_decision_record.md](06_architecture_decision_record.md) | `architecture_decision_record.md` | 架构、数据契约、状态机、页面/组件体系、Mock 边界 |
| [07_g1_task_breakdown.md](07_g1_task_breakdown.md) | `G1_task_breakdown.md` | G1 任务分解、测试计划、待确认问题 |

## 五个必须先知道的结论

1. **`/controller_manager/joint_states` 不是传感器反馈。**
   它由 `servo_manager` 缓存的“最后下发脉冲”换算而来（`controller_manager.py:97-114`）。真实位置只能通过 `GetBusServoState` 服务按需单舵机读取。任何界面把 `joint_states` 标成 “feedback/实测” 都是错误陈述。

2. **运动命令没有 ACK。**
   `bus_servo_set_position()` 只做一次串口写（`ros_robot_controller_sdk.py:514-519`）。因此命令状态最多到 `Dispatched`，之后只能靠“独立位置读取 + 时间/容差判定”进入 `Observed/Completed`，否则必须是 `Unknown`。

3. **SDK 读取可能无限阻塞。**
   `bus_servo_read_and_unpack()` 使用 `queue.get(block=True)` 且无超时（`ros_robot_controller_sdk.py:523-529`）。服务回调阻塞会卡住整个 Driver 节点（默认单线程 executor）。这是 SR-03 的阻断项，不能靠“前端超时”掩盖。

4. **厂商 Driver 存在若干确定性缺陷**（详见 03 文档）：
   - `get_torque_state` 调用不存在的方法 → `AttributeError`；
   - `pub_button_data` 使用未导入的 `PacketReportKeyEvents` → `NameError`；
   - `/joint_controller` 回调参数类型错误 → 该控制入口不可用；
   - Trajectory Action 不发 feedback，且结束即 `succeed()`，不校验实际位置；
   - `enable_reception` 方法名与布尔属性同名，语义随消息到达而改变。

5. **现有栈没有 MoveIt 实机链路，也没有 ros2_control 实机插件。**
   厂商 MoveIt 配置使用 `mock_components/GenericSystem`（假硬件）。真实可用的轨迹入口只有 `servo_controller` 自带的 `FollowJointTrajectory` ActionServer，且为开环。是否复用、是否修补，需 Board 决策。

## 关键文件根路径

```
VENDOR = /home/jerichocheng/ArmPiUltra/ROS2/Armpi_ultra-main/armpi_ultra
PRD    = /home/jerichocheng/ArmPiUltra/ARMPI_DEBUG_STUDIO_PRD_V1.md
```

下文所有相对路径均以 `VENDOR` 为根，除非另行标注。

## 本阶段明确没做的事

- 未运行任何 ROS 2 节点、未打开串口、未发送任何运动命令。
- 未修改厂商源码、未安装依赖、未创建 Git 提交。
- 未把任何“源码存在”当成“实机可用”。
