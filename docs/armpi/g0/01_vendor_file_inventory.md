# 01 · 厂商文件清单与复用边界

> 证据等级：`Verified in source`（路径与职责均来自源码）
> 根路径：`VENDOR = /home/jerichocheng/ArmPiUltra/ROS2/Armpi_ultra-main/armpi_ultra`

本文件回答两个问题：**完成 ArmPi Debug Studio 需要哪些厂商文件？哪些文件不能碰、也不需要？**

---

## 1. 控制链路必需（G1 直接复用）

按数据流从下到上排列。

| # | 文件 | 职责 | 复用方式 | 证据 |
|---|---|---|---|---|
| 1 | `ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py` | 串口协议、CRC8、总线舵机读写、系统/按键/IMU 上报 | **原样复用**；不拷贝、不改写、不另起协议实现 | 809 行，`Board` 类 118-703 |
| 2 | `ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py` | 把 SDK 暴露为 ROS 2 Topic/Service | **复用**；缺陷以“已知限制”登记，修补需 Board 批准 | 365 行，接口在 50-73 |
| 3 | `ros2_ws/src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py` | 启动 Driver | 复用 | — |
| 4 | `ros2_ws/src/driver/servo_controller/servo_controller/controller_manager.py` | 关节名↔舵机 ID、单位换算、`joint_states` 发布、`FollowJointTrajectory` Action | **复用但不得信任其反馈语义** | 126 行 |
| 5 | `ros2_ws/src/driver/servo_controller/servo_controller/joint_position_controller.py` | pulse↔rad 线性换算、方向、限位裁剪 | 复用；其换算常量作为“临时映射”证据 | 55 行 |
| 6 | `ros2_ws/src/driver/servo_controller/servo_controller/joint_trajectory_action_controller.py` | 开环轨迹 ActionServer | 复用；G2 前必须审计其完成语义 | 175 行 |
| 7 | `ros2_ws/src/driver/servo_controller/servo_controller/servo_controller.py` | `ServoManager`：脉冲下发、初始位置缓存 | 复用 | 120 行 |
| 8 | `ros2_ws/src/driver/servo_controller/servo_controller/bus_servo_control.py` | 构造 `ServosPosition` 消息的便捷函数 | 参考/复用 | 35 行 |
| 9 | `ros2_ws/src/driver/servo_controller/config/servo_controller.yaml` | **唯一现存的关节↔舵机映射与限位来源** | 作为 `provisional` 映射输入，不直接当作已验证配置 | 见 04 文档 |
| 10 | `ros2_ws/src/driver/servo_controller/launch/servo_controller.launch.py` | 启动 `servo_controller` + `grasp` | 复用 | — |
| 11 | `ros2_ws/src/driver/ros_robot_controller_msgs/` | `BusServoState`、`GetBusServoCmd`、`ServosPosition`、`SetBusServoState` 等消息/服务 | 复用 | 19 个 msg + 3 个 srv |
| 12 | `ros2_ws/src/driver/servo_controller_msgs/` | `ServosPosition`（含 `position_unit`）、`ServoStateList`、`Grasp` | 复用 | 6 个 msg |
| 13 | `ros2_ws/src/driver/kinematics/` + `kinematics_msgs/` | 逆解服务、当前位姿、关节范围 | **G1 可选**；G2/G4 位姿功能需要 | 见 03 文档 |
| 14 | `ros2_ws/src/driver/sdk/launch/armpi_ultra.launch.py` | 硬件总启动：Driver + Servo + Kinematics + 底盘 | 作为“厂商标准启动顺序”参考；我们的 Gateway 不应启动底盘 | — |
| 15 | `ros2_ws/src/peripherals/scripts/99-ttyACM0.rules` | 创建 `/dev/rrc` 符号链接 | **必须确认已安装**；文件本身有语法错误（见 §5） | 2 行 |

> **单串口所有者原则（SR-10）**：以上 1-2-4-5-6-7 共同构成唯一硬件写路径。任何新代码不得 import `Board` 直接开串口。

---

## 2. 诊断与标定参考（只读，不进入运行时）

| 文件 | 用途 | 使用方式 |
|---|---|---|
| `software/servo_tool/servo_tool/main.py`、`software/servo_tool/config/bus_servo_control.py` | 厂商舵机调试工具：ID、偏移、角度/电压/温度限位 | 只读参考；用于设计“限位与偏移”诊断字段，以及实机标定流程 |
| `software/armpi_ultra_control/servo_controller.py` | 厂商标定/控制 GUI 直接 `Board()` 打开串口 | **证据**：证明存在第二串口所有者（D-06） |
| `software/armpi_ultra_control/main.py` | GUI 主程序，含舵机读写 | 只读参考 |
| `software/Servo_upper_computer/ros_robot_controller_sdk.py` | 旧版 SDK（621 行） | 版本差异证据（D-02） |
| `software/hand2cam_tf_matrix_software/calibration/ros_robot_controller_sdk.py` | 又一份旧版 SDK（614 行） | 版本差异证据（D-02） |
| `STM32/ArmPiUltra/` | 控制板固件源码 | **仅在 Board 批准后**做只读协议/看门狗审计（D-07） |
| `1.ArmPi Ultra使用入门.pdf` | 接线、供电、上手流程 | 只读参考；PDF 中的接口描述需与源码交叉核对 |

---

## 3. 后期阶段可选（G2–G4）

| 文件 | 阶段 | 说明 |
|---|---|---|
| `ros2_ws/src/simulations/armpi_ultra_description/urdf/armpi_ultra.xacro` | G4 / 3D | 5 关节 + 夹爪模型；包含 `gripper.urdf.xacro`，其中 `r_joint` 为 revolute ±1.57。注意同目录预生成的 `armpi_ultra.urdf` 是旧版本，把 `r_joint` 写成 `fixed`，不可作为基线 |
| `ros2_ws/src/simulations/armpi_ultra_description/meshes/*.STL` | G4 | 3D 渲染资源 |
| `ros2_ws/src/simulations/armpi_ultra_description/launch/robot_description.launch.py` | G4 | RViz/TF 参考；其中 `joint_state_publisher` 的 `source_list` 是 `controller_manager/joint_states`（命令回显） |
| `/home/jerichocheng/ArmPiUltra/3 功能包文件/simulations/armpi_ultra_moveit_config/` | G2 | 厂商 MoveIt 配置；**使用 `mock_components/GenericSystem`，不是实机插件**；`moveit_controllers.yaml` 指向的 Action 名与真实 `servo_controller` 一致，可作为集成起点 |
| `ros2_ws/src/peripherals/launch/depth_camera.launch.py` + `launch/include/usb_cam.launch.py` | G4 | 相机；话题 `/depth_cam/rgb/image_compressed`、`/depth_cam/rgb/camera_info` |
| `ros2_ws/src/simulations/robot_gazebo/`（在 `3 功能包文件/`） | 可选 | Gazebo 仿真；**Mock Mode 不依赖 Gazebo**，避免把 mock 与仿真器耦合 |
| `ros2_ws/src/app/config/calibration.yaml`、`transform.yaml` | G4 | 视觉/手眼标定；与关节调试无关 |

---

## 4. 明确不进入本项目（Out of Scope）

以下目录体量大、与本项目目标无关，**不复制、不修改、不打包**：

- `software/ultralytics/`、`software/yolov8/`、`software/yolov11/`
- `software/roLabelImg/`、`software/collect_picture/`、`software/lab_tool/`
- `software/calibration/`、`software/hand2cam_tf_matrix_software/`（仅其中的 SDK 副本作为证据）
- `ros2_ws/src/app/`（颜色分拣、垃圾识别、标签堆叠等业务）
- `ros2_ws/src/large_models/`、`large_models_msgs/`、`yolov8_detect/`
- `ros2_ws/src/peripherals/`（除 udev 规则与 G4 相机 launch）
- `ros2_ws/src/driver/controller/`（麦轮/坦克底盘）
- `ros2_ws/src/example/`、`bringup/voice/`
- `ros2_ws/src/interfaces/`（视觉 ROI/颜色服务，G4 以前无关）

> 理由：PRD §1.4 明确不在 MVP 中加入 AI/视觉/底盘；复制这些目录会扩大攻击面与构建时间，且与“可解释调试”无直接关系。

---

## 5. 必须核实的部署文件

### 5.1 `/dev/rrc` 从哪来

- SDK 默认设备路径：`Board(device="/dev/rrc")`（SDK:134）。
- `peripherals/scripts/99-ttyACM0.rules` 第 2 行创建 `SYMLINK+="rrc"`。
- `driver/ros_robot_controller/scripts/99-ttyACM0.rules` **没有** SYMLINK，只改权限。
- `peripherals` 规则第 3 行存在语法错误：`ENV{ID_MM_PORT_IGNORE}="1"1"`，多了一个 `1"`。

**结论**：`/dev/rrc` 是否存在取决于镜像里安装了哪一份规则；两份规则不一致。G1 必须把“设备节点存在/可打开”作为独立诊断项，而不是假定存在。

### 5.2 波特率与串口参数

- 1,000,000 baud（SDK 默认）。
- `serial.Serial(None, baudrate, timeout=5)`，8N1（pyserial 默认），RTS/DTR 置低。
- 无流控配置。

---

## 6. SDK 版本对照（D-02）

| 副本 | 行数 | SHA-256（前 16） | 差异要点 |
|---|---|---|---|
| `ros2_ws/src/driver/ros_robot_controller/.../ros_robot_controller_sdk.py` | 809 | `369b11dd3b73a9e3` | **最新**：含步进电机、传送带、`set_motor_duty/pwm` |
| `software/Servo_upper_computer/ros_robot_controller_sdk.py` | 621 | `224e58597aee8ab0` | 无步进/传送带；有 `set_RGB`、`set_motor_type` |
| `software/hand2cam_tf_matrix_software/calibration/ros_robot_controller_sdk.py` | 614 | `a510df07bfbf7ee1` | 更旧；接口集合不同 |

**决策建议**：以 ROS 2 Driver 内嵌的 809 行版本为唯一基线；不引入另外两份，避免“同名不同行为”。

---

## 7. 本项目需要新建的文件（尚未创建）

```text
armpi-debug-studio/            # 位置待 Board 决定，见 07 文档 Q-01
├── backend/
│   ├── app/main.py            # FastAPI 入口
│   ├── app/adapters/base.py   # RobotAdapter 协议
│   ├── app/adapters/mock.py   # Mock 适配器（无 ROS）
│   ├── app/adapters/ros2.py   # ROS 2 适配器（G1 只读优先）
│   ├── app/safety/            # 门禁、校验、租约、看门狗
│   ├── app/domain/            # 状态机、数据模型、错误码
│   ├── app/persistence/       # SQLite 事件/命令/配置
│   └── tests/
├── frontend/
│   ├── src/app/               # 路由、AppShell
│   ├── src/pages/             # Dashboard/JointControl/Diagnostics/Timeline/Settings
│   ├── src/components/        # 设计系统组件
│   ├── src/stores/            # 实时状态
│   └── src/api/               # REST/WS 客户端 + zod 校验
├── ros2_ws/src/armpi_debug_gateway/   # 仅在需要独立进程时创建（ADR-002）
├── configs/
│   ├── joint_mapping.yaml     # versioned, verified: false
│   └── safety_limits.yaml
├── docs/                      # 本目录迁入
└── tests/{unit,integration,simulation,hardware}
```

**本阶段只创建了 `docs/g0/`，没有创建上述代码目录。**
