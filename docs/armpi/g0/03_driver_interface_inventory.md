# 03 · ROS 2 Driver 接口清单（D-01 / D-05 / D-06）

> 证据等级：`Verified in source`
> 审计范围：`VENDOR/ros2_ws/src/driver/` 与 `VENDOR/ros2_ws/src/simulations/armpi_ultra_description/`
> 对应 PRD：D-01、D-05、D-06、FR-01、FR-05、FR-06

---

## 1. 包与节点

| 包 | 可执行文件 | 节点名 | 作用 |
|---|---|---|---|
| `ros_robot_controller` | `ros_robot_controller` | `/ros_robot_controller` | 串口 SDK 的 ROS 2 封装 |
| `servo_controller` | `servo_controller` | `/controller_manager` | 关节名/单位/限位、轨迹 Action、`joint_states` |
| `servo_controller` | `grasp` | `/grasp` | 抓取动作序列（非 G1） |
| `servo_controller`（内部类） | — | `/servo_manager` | 发布脉冲、缓存“最后命令” |
| `kinematics` | `search_kinematics_solutions` | `/kinematics` | 逆解/正解、当前位姿 |
| `sdk` | `check_servo_connection` | `/check_servo_connection` | 启动时检查 6 个舵机 ID（默认未启用） |

标准硬件启动顺序来自 `driver/sdk/launch/armpi_ultra.launch.py`：
`ros_robot_controller` → `servo_controller`（含 `grasp`）→ `kinematics` → 底盘（本项目不启动）。

---

## 2. `/ros_robot_controller` 接口

### 2.1 发布（私有命名空间 `~/`）

| 话题 | 类型 | 内容 | 频率/条件 |
|---|---|---|---|
| `~/imu_raw` | `sensor_msgs/Imu` | 加速度（g→m/s²）、角速度（°/s→rad/s）；orientation 恒为 0 | 50 Hz 轮询，仅有数据时发布 |
| `~/joy` | `sensor_msgs/Joy` | 手柄 axes/buttons | 同上 |
| `~/sbus` | `ros_robot_controller_msgs/Sbus` | 16 通道 | 同上 |
| `~/button` | `ros_robot_controller_msgs/ButtonState` | 按键 id/state | 事件触发（存在 NameError，见 §5） |
| `~/battery` | `std_msgs/UInt16` | SYS 包电压值 | 有数据时 |

> `imu_raw` 的 `orientation` 四元数恒为 0，协方差却写成有效值。下游不得把该 orientation 当作姿态。

### 2.2 订阅

| 话题 | 类型 | 语义 |
|---|---|---|
| `~/set_led` | `LedState` | LED |
| `~/set_buzzer` | `BuzzerState` | 蜂鸣器 |
| `~/set_oled` | `OLEDState` | OLED 文本 |
| `~/set_motor_speeds` | `MotorsSpeedControl` | 底盘电机占空比 |
| `~/enable_reception` | `std_msgs/Bool` | 开关 SDK 接收线程 |
| `~/bus_servo/set_position` | `ros_robot_controller_msgs/ServosPosition` | **原始脉冲**，无单位字段 |
| `~/bus_servo/set_state` | `SetBusServoState` | 舵机参数：ID/偏移/限位/扭矩/停止 |
| `~/pwm_servo/set_state` | `SetPWMServoState` | PWM 舵机 |
| `~/set_conveyor_speed` | `ConveyorSpeed` | 传送带 |
| `~/set_stepper_subdivision` | `StepperSubdivision` | 步进细分 |

### 2.3 服务

| 服务 | 类型 | 语义 | 备注 |
|---|---|---|---|
| `~/bus_servo/get_state` | `GetBusServoState` | 按 `cmd[]` 逐项读取 | **唯一的真实位置读取入口**；无超时；`success` 恒为 True |
| `~/pwm_servo/get_state` | `GetPWMServoState` | PWM 舵机读取 | 不使用 |
| `~/move_stepper` | `MoveStepper` | 步进移动 | 不使用 |
| `~/init_finish` | `std_srvs/Trigger` | 启动完成握手 | 无实际状态检查 |

`GetBusServoState` 请求字段：`id, get_id, get_position, get_offset, get_voltage, get_temperature, get_position_limit, get_voltage_limit, get_max_temperature_limit, get_torque_state`。
响应：`success: bool` + `state: BusServoState[]`。

**重要**：`success` 在 L248 无条件置 True，即使所有读取返回 `None`。它表示“回调执行完”，**不表示数据有效**。

---

## 3. `/controller_manager` 接口

### 3.1 发布

| 话题 | 类型 | 数据来源 | 真实性 |
|---|---|---|---|
| `/controller_manager/joint_states` | `sensor_msgs/JointState` | `ServoManager.servos[*].position`（最后下发脉冲） | **命令回显** |
| `/controller_manager/servo_states` | `servo_controller_msgs/ServoStateList` | 同上 | **命令回显** |

证据：`controller_manager.py:97-114`；`servo_controller.py:99-111` 在每次 `set_position` 时更新缓存（L106）。

发布频率：20 Hz（`time.sleep(0.02)`，L116）。

### 3.2 订阅

| 话题 | 类型 | 语义 | 状态 |
|---|---|---|---|
| `/servo_controller` | `servo_controller_msgs/ServosPosition` | 支持 `position_unit = pulse / rad / deg` | 可用（高层入口） |
| `/joint_controller` | `sensor_msgs/JointState` | 以 rad 下发单关节 | **不可用**（参数类型错误，见 §5） |

`/servo_controller` 的消息结构：

```
float64 duration
string  position_unit      # "pulse" | "rad" | "deg"
servo_controller_msgs/ServoPosition[] position   # uint16 id, float32 position
```

`servo_controller` 会把 `duration` 裁剪到 `[0.02, 30]` 秒、`position` 裁剪到 `[0, 1000]` 脉冲（`servo_controller.py:100-106`）。**裁剪是静默的，不报错、不记录**——后端必须在此之前拒绝非法值。

### 3.3 服务

| 服务 | 类型 | 语义 |
|---|---|---|
| `~/init_finish` | `Trigger` | 恒返回 success=True |

### 3.4 Action

| Action | 类型 | 语义 |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 5 关节开环轨迹 |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | `r_joint` |

实现事实（`joint_trajectory_action_controller.py`）：

- 按段调用 `servo_manager.set_position(duration, ...)`（L156），每段一次脉冲命令。
- **不发布 feedback**（没有 `publish_feedback` 调用），但日志打印 “Feedback received” 是误导性文本（L158-159）。
- 段结束后不读取实际位置，不检查 `goal_tolerance`，直接 `goal_handle.succeed()`（L173）。
- 取消只在段间检查（L162-165）；段内不可中断。
- 无 preemption 处理；无路径容差；无执行时间校验。
- 首点 `positions` 为空 → `INVALID_GOAL`（L101-108）；关节名不匹配 → abort（L75-83）。

**结论**：该 Action 的 `succeeded` 只代表“所有段的脉冲命令已按时间发完”，**不代表机械臂到达目标**。G2 轨迹分析必须把这一点写进指标说明。

---

## 4. `/servo_manager` 与 `/kinematics`

### 4.1 `/servo_manager`（内部节点）

| 方向 | 名称 | 类型 |
|---|---|---|
| Pub | `/ros_robot_controller/bus_servo/set_position` | `ros_robot_controller_msgs/ServosPosition` |
| Client | `/ros_robot_controller/bus_servo/get_state` | `GetBusServoState` |

`ServoManager.__init__` 调用 `client.wait_for_service()` **无超时**（L42）。Driver 不在线时，`servo_controller` 会永久卡在启动阶段。

### 4.2 `/kinematics`

| 方向 | 名称 | 类型 |
|---|---|---|
| Pub | `/servo_controller` | `ServosPosition` |
| Sub | `/controller_manager/servo_states` | `ServoStateList` |
| Client | `/controller_manager/init_finish` | `Trigger`（无超时等待，L38-39） |
| Srv | `~/set_link` / `~/get_link` | `SetLink` / `GetLink` |
| Srv | `~/set_joint_range` / `~/get_joint_range` | `SetJointRange` / `GetJointRange` |
| Srv | `~/set_pose_target` / `~/set_pose_target_smooth` | `SetRobotPose` |
| Srv | `~/get_current_pose` | `GetRobotPose` |
| Srv | `~/set_joint_value_target` | `SetJointValue` |
| Srv | `~/init_finish` | `Trigger` |

`GetRobotPose` / 逆解的“当前位姿”来自 `/controller_manager/servo_states`，即 **命令回显**。因此它是“模型上的当前命令位姿”，不是实测位姿。G2/G4 使用前必须重命名或标注。

### 4.3 `/grasp`

订阅 `/grasp`（`Grasp`），调用 `/kinematics/set_pose_target`，向 `/servo_controller` 发脉冲。属于厂商业务层，G1 不启用。

---

## 5. 已确认缺陷（会影响 Debug Studio）

| ID | 位置 | 缺陷 | 触发条件 | 影响 | 证据 |
|---|---|---|---|---|---|
| B-01 | `ros_robot_controller_node.py:243` | 调用 `self.board.bus_servo_read_torque(i.id)`，SDK 只有 `bus_servo_read_torque_state` | `GetBusServoState` 请求 `get_torque_state=1` | `AttributeError`，服务无响应 | SDK L556 |
| B-02 | `ros_robot_controller_node.py:291-298` | 使用未导入的 `PacketReportKeyEvents` | 任意按键事件 | `NameError`，发布线程退出，后续 IMU/电池等停止发布 | import 仅 L16 |
| B-03 | `controller_manager.py:91-95` | `set_position(servo_id, pulse)` 与签名 `(duration, positions)` 不符 | 任何 `/joint_controller` 消息 | `TypeError`，该入口不可用 | `servo_controller.py:99` |
| B-04 | `joint_trajectory_action_controller.py:173` | 不校验实际位置即 `succeed()` | 任意轨迹 | “执行成功”是假阳性 | — |
| B-05 | `joint_trajectory_action_controller.py` | 无 `publish_feedback` | 任意轨迹 | MoveIt 无进度反馈 | grep 无调用 |
| B-06 | `ros_robot_controller_node.py:59,82,93` | `enable_reception` 方法/属性同名 | 收到 Bool 后 | 发布线程开关语义漂移 | — |
| B-07 | `ros_robot_controller_node.py:40` | `Board()` 构造即开串口，无重试 | 设备缺失/被占用 | 节点直接退出 | SDK L139-143 |
| B-08 | `servo_controller.py:42`、`controller_manager.py:59`、`kinematics:39`、`grasp:19` | `wait_for_service()` 无超时 | 上游节点缺失 | 启动永久阻塞 | — |
| B-09 | `ros_robot_controller_node.py:248` | `response.success = True` 无条件 | 读取全部失败 | 假阳性成功 | — |
| B-10 | `joint_position_controller.py:43-55` | 超限目标静默裁剪 | rad 超限 | 无拒绝、无日志，违反 FR-02 | — |
| B-11 | `ros_robot_controller_node.py:69-70` | 启动即写 PWM 偏移=0、底盘电机=0 | 每次启动 | 未确认的硬件写操作 | — |
| B-12 | `ros_robot_controller/package.xml` | 未声明 `rclpy`、`python3-serial`、消息依赖 | 干净环境构建 | 依赖不可复现 | — |

> 这些缺陷 **不在本阶段修补**。G1 的处理方式是：绕开不可用入口（B-01/B-03）、把假阳性标成 `Unknown`（B-04/B-09）、把无超时等待纳入看门狗（B-08）。

---

## 6. 启动依赖链与故障表现

```text
ros_robot_controller
  └─ 打开 /dev/rrc（失败 → 进程退出）
       ▲
       │ GetBusServoState 服务（无超时等待）
servo_manager
       ▲
       │ /ros_robot_controller/init_finish
controller_manager
       ▲
       │ /controller_manager/init_finish
kinematics
       ▲
       │ /kinematics/set_pose_target
grasp
```

| 故障 | 当前表现 | Debug Studio 必须显示 |
|---|---|---|
| `/dev/rrc` 不存在 | Driver 启动即退出 | `Disconnected` + “串口设备缺失” + 规则安装步骤 |
| 串口被厂商 GUI 占用 | `SerialException`（Permission/Resource busy） | `Fault` + “串口被其他进程占用” + `fuser` 提示 |
| Driver 崩溃 | `servo_manager` 永久等待 | `Degraded`（上游依赖阻塞），不是 `Connecting` |
| 舵机缺失 | `get_state` 永久阻塞 | `Fault` + “位置读取超时（SDK 无超时）” |
| 反馈停止 | `joint_states` 仍在发布（命令回显） | 必须显示“反馈年龄不可计算”，而不是“正常” |

---

## 7. MoveIt 与 ros2_control 现状（D-05）

| 项 | 结论 | 证据 |
|---|---|---|
| 是否存在 `ros2_control` 实机插件？ | **否**。厂商 MoveIt 配置的 `armpi_ultra.ros2_control.xacro` 使用 `mock_components/GenericSystem` | `/home/jerichocheng/ArmPiUltra/3 功能包文件/simulations/armpi_ultra_moveit_config/config/armpi_ultra.ros2_control.xacro` |
| 是否有轨迹 Action？ | **有**，但由 `servo_controller` 自带，非 `ros2_control` | `joint_trajectory_action_controller.py:51-55` |
| MoveIt 能否直接驱动真实臂？ | **可能**：`moveit_controllers.yaml` 的 Action 名与真实 Action 一致；但 MoveIt 需要 `/joint_states` 作为当前状态，而现有 `/joint_states` 是命令回显，且厂商启动文件未显式 remap。需实机验证。 | `moveit_controllers.yaml`、`move_group.launch.py` |
| 是否使用 `JointTrajectoryController`？ | 仅假硬件配置；真实栈没有 | `ros2_controllers.yaml` |
| URDF 与真实关节是否一致？ | 5 关节名称一致；`gripper.urdf.xacro` 中 `r_joint` 为 revolute（±1.57），但同目录预生成的 `armpi_ultra.urdf` 把它写成 `fixed`（旧文件）。真实 `r_joint` 是舵机 ID 1，软件范围约 `[-2.93, +1.26]` rad，与 URDF 不一致 | `gripper.urdf.xacro:44-60`、`armpi_ultra.urdf:431` |

**决策建议**：G1 不接 MoveIt；G2 先做离线轨迹记录，再评估“MoveIt + 真实 Action”的可行性。不要在 G1 同时引入 MoveIt 和 Web 控制。

---

## 8. 串口所有权（D-06）

| 所有者 | 路径 | 行为 |
|---|---|---|
| ROS Driver | `ros_robot_controller_node.py:40` | `Board()` → `/dev/rrc` |
| 厂商 GUI | `software/armpi_ultra_control/servo_controller.py:4` | 模块导入即 `Board()` |
| 厂商标定工具 | `software/servo_tool/servo_tool/*` | 直接 `Board()` |
| `hw_interfaces.py` | `driver/sdk/sdk/hw_interfaces.py:14-17` | 直接 `Board()`（部分厂商 App 使用） |

**结论**：至少 4 条路径可能同时打开 `/dev/rrc`。Linux 串口默认不共享，第二个进程会 `Permission denied / Device or resource busy`。G1 必须实现“串口所有者探测”，并在 UI 明确提示关闭厂商 GUI/标定工具。

---

## 9. D-01 交付结论

已确认的接口全部在上文；未确认项：

| 项 | 状态 | 原因 |
|---|---|---|
| `joint_states` 是否为真实反馈 | **否（源码确认是命令回显）** | `controller_manager.py:97-114` |
| 真实反馈频率/精度 | `Unknown` | 无轮询、无实测 |
| 硬件限位是否已写入舵机 | `Unknown` | `servo_controller.yaml` 只是软件范围；舵机 EEPROM 限位需读取确认 |
| 固件版本 | `Unknown` | SDK 无 API |
| 急停/看门狗能力 | `Unknown` | 需固件审计 + 实机 |
| 断线后的驱动行为 | `Unknown` | 静态代码只能推断为“阻塞/线程死亡” |
| 真实轨迹执行能力 | `Partial` | 有开环 Action，但无反馈/无校验 |
