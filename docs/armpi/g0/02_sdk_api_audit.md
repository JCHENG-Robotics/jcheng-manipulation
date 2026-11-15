# 02 · SDK 审计（`ros_robot_controller_sdk.py`）

> 证据等级：`Verified in source`（逐行）
> 审计对象：`VENDOR/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`
> 行数：809 · SHA-256：`369b11dd3b73a9e31fd88057a0a9c39e153ab2d978606be37e77280cd22a25bb`
> 对应 PRD：D-02、D-03、D-10、SR-03

---

## 1. 定位

这是厂商提供的 **Python 串口 SDK**，负责 STM32 控制板的二进制协议。它不包含任何 ROS 2 依赖，也不理解“关节”“弧度”“轨迹”。

- 直接依赖：`pyserial`、`struct`、`queue`、`threading`。
- 被谁使用：`ros_robot_controller_node.py`（ROS 2 Driver）、`sdk/sdk/hw_interfaces.py`（另一条直连路径）、厂商 GUI `software/armpi_ultra_control/servo_controller.py`。
- **本项目决策：只通过 ROS 2 Driver 间接使用，不在后端 import `Board`。**（SR-10）

---

## 2. 传输层与帧格式

| 项 | 值 | 证据 |
|---|---|---|
| 设备 | `/dev/rrc` | `Board.__init__` L134 |
| 波特率 | `1_000_000` | L134 |
| 串口超时 | `timeout=5` 秒（仅作用于 `port.read()`） | L134 |
| 数据位/校验/停止位 | pyserial 默认 8N1 | 未显式设置 |
| RTS/DTR | 打开前置低 | L140-141 |

帧格式（`buf_write` L361-366）：

```
0xAA 0x55 | func(1B) | len(1B) | data(lenB) | crc8(1B)
CRC8 覆盖: func + len + data
```

> 注意：文件头注释 L12 写的是 `0xAA 0x55 Length Function ID Data Checksum`，与实际字节顺序 **相反**。以代码为准。

CRC8 使用厂商查表（L84-108）。接收状态机在 `recv_task`（L650-703），只接受 `func < 14` 的帧；CRC 失败打印“校验失败”并丢弃。

### 功能号

| 值 | 功能 | 本项目是否使用 |
|---|---|---|
| 0 | SYS（电池等上报） | 诊断（电压） |
| 5 | 总线舵机 | **核心** |
| 4 | PWM 舵机 | 否（ArmPi Ultra 用总线舵机） |
| 1/2/10/11 | LED/蜂鸣器/OLED/RGB | 可选提示 |
| 3/12/13 | 电机/传送带/步进 | 否（配件） |
| 6/7/8/9 | 按键/IMU/手柄/SBUS | 诊断可选 |

---

## 3. 生命周期与线程模型

1. `Board.__init__` **在构造函数里直接打开串口**（L139-143）。设备不存在 → `serial.SerialException` 立即抛出，Driver 节点启动失败。无重试、无降级。
2. 构造后 `time.sleep(0.5)`，然后启动 **daemon** 接收线程（L170-171）。
3. `enable_recv` 默认 `False`（L135）。接收线程在 `False` 时只 `sleep(0.01)`，**不读取串口**（L701）。
4. 读取类 API 依赖接收线程解析回包。因此：**未调用 `enable_reception(True)` 时，任何 `bus_servo_read_*` 都会永久阻塞**（见 §5）。
5. `recv_task` 是 `while True`，`self.port.close()`（L702）是**不可达代码**；节点退出时不会优雅关闭串口。
6. 读事务用 `self.servo_read_lock` 串行化（L146, L459, L524）；**写事务没有锁**。

---

## 4. 真实 API 清单

### 4.1 总线舵机——写（无返回值、无 ACK）

| 方法 | 参数 | 行为 | 证据 |
|---|---|---|---|
| `bus_servo_set_position(duration, positions)` | `duration: float` 秒；`positions: Iterable[(id, pulse)]` | 转 ms 后下发；`pulse` 不裁剪 | L514-519 |
| `bus_servo_stop(servo_id)` | `servo_id: Iterable[int]`（**不是单个 int**） | 子命令 `0x03` | L509-512 |
| `bus_servo_enable_torque(servo_id, enable)` | `enable` 真值 | `0x0B`/`0x0C`；写完 `sleep(0.02)` | L471-477 |
| `bus_servo_set_id(now, new)` | — | `0x10`；危险，会改舵机 ID | L479-482 |
| `bus_servo_set_offset(servo_id, offset)` | `offset: int` | `0x20`；不保存 | L484-487 |
| `bus_servo_save_offset(servo_id)` | — | `0x24`；写 EEPROM | L489-492 |
| `bus_servo_set_angle_limit(servo_id, [min,max])` | 0–1000 | `0x30`；写舵机限位 | L494-497 |
| `bus_servo_set_vin_limit(servo_id, [min,max])` | 单位未知（见 §9） | `0x34` | L499-502 |
| `bus_servo_set_temp_limit(servo_id, limit)` | 摄氏度 | `0x38` | L504-507 |

**关键结论**：这些方法都只调用 `buf_write`，**没有等待任何确认帧**。返回 `None` 仅表示“字节已写入串口”，不表示舵机收到、开始运动或到达目标。

### 4.2 总线舵机——读（阻塞）

统一入口 `bus_servo_read_and_unpack(servo_id, cmd, unpack)`（L523-529）：

```python
with self.servo_read_lock:
    self.buf_write(PACKET_FUNC_BUS_SERVO, [cmd, servo_id])
    data = self.bus_servo_queue.get(block=True)   # ← 无超时
    servo_id, cmd, success, *info = struct.unpack(unpack, data)
    if success == 0:
        return info
```

| 方法 | 子命令 | 解包格式 | 返回 |
|---|---|---|---|
| `bus_servo_read_id(servo_id=254)` | `0x12` | `<BBbB` | `[id]` |
| `bus_servo_read_offset(id)` | `0x22` | `<BBbb` | `[offset]` |
| `bus_servo_read_position(id)` | `0x05` | `<BBbh` | `[position]`（有符号） |
| `bus_servo_read_vin(id)` | `0x07` | `<BBbH` | `[voltage]` |
| `bus_servo_read_temp(id)` | `0x09` | `<BBbB` | `[temperature]` |
| `bus_servo_read_temp_limit(id)` | `0x3A` | `<BBbB` | `[limit]` |
| `bus_servo_read_angle_limit(id)` | `0x32` | `<BBb2H` | `[min, max]` |
| `bus_servo_read_vin_limit(id)` | `0x36` | `<BBb2H` | `[min, max]` |
| `bus_servo_read_torque_state(id)` | `0x0D` | `<BBbb` | `[state]` |

失败语义：`success != 0` → 返回 `None`；调用方必须显式处理 `None`，否则会 `TypeError`。

### 4.3 系统/人机接口（非核心）

| 方法 | 返回 | 备注 |
|---|---|---|
| `get_battery()` | `int` 或 `None` | 仅当 `enable_recv=True`；解析 SYS 包 `data[0]==0x04` 为 `<H` |
| `get_button()` | `(key_id, 0/1)` 或 `None` | 只映射 CLICK/PRESSED；其他事件返回 `None`；非法事件字节会抛 `ValueError` |
| `get_imu()` | `(ax,ay,az,gx,gy,gz)` 或 `None` | 加速度单位 g，角速度 °/s（Driver 再换算） |
| `get_gamepad()` | `(axes, buttons)` 或 `None` | 16 通道 |
| `get_sbus()` | 16 通道列表 或 `None` | 信号丢失时返回固定 0.5 |

### 4.4 其他执行器（本项目不用）

`set_led`、`set_buzzer`、`set_oled_text`、`set_motor_speed/duty/pwm`、`pwm_servo_*`、`set_conveyor_speed`、`set_stepper_motor_*`。

> **SDK 缺陷**：`set_stepper_motor_position` 定义了两次（L419-428 与 L558-646）。后者覆盖前者；前者是死代码。后者会在未初始化时阻塞轮询最多 30 秒，并直接 `print` 到 stdout。本项目不使用步进电机，但需知道该 API 不适合放进 Web 请求路径。

---

## 5. 阻塞、超时与队列行为（SR-03 核心）

### 5.1 无超时的阻塞读取

```python
data = self.bus_servo_queue.get(block=True)   # SDK L526
```

- 串口 `timeout=5` **不能**终止这个等待：超过 5 秒没有字节时，`port.read()` 返回空，接收线程继续循环；队列仍然为空。
- 如果舵机不存在、掉线、ID 错、总线冲突，调用线程会 **永久阻塞**。
- Driver 的 `get_bus_servo_state` 服务回调运行在该等待中。Driver 用 `rclpy.spin(node)`（单线程执行器），因此 **一次读失败会冻结整个 Driver 节点**（所有 Topic/Service 停止响应）。

### 5.2 `enable_reception` 是读操作的前置条件

接收线程只在 `self.enable_recv` 为真时读取串口（L652）。若未启用：

- 电池/按键/IMU 等 `get_*` 返回 `None`；
- `bus_servo_read_*` 永久阻塞（没有解析线程把回包放进队列）。

Driver 在构造时已调用 `self.board.enable_reception()`（node L41），所以正常路径可用；但任何直接使用 `Board()` 的新代码都必须显式启用。

### 5.3 队列容量与“错配回包”

- `bus_servo_queue`、`pwm_servo_queue`、`sys_queue` 等 `maxsize=1`（L150-157）。
- 接收线程用 `put_nowait`，**队列满时静默丢弃**（L203-207）。
- `bus_servo_read_and_unpack` **不校验回包中的 servo_id / cmd 是否与请求一致**，只返回 `info`。

后果：在连续快速读取多个舵机时，一次丢弃 + 一次旧回包，可能把 **A 舵机的位置返回给 B 舵机**，而调用方无法察觉。这是诊断功能必须规避的设计缺陷。

### 5.4 `enable_reception` 的命名陷阱

Driver 节点里同时存在：

- 方法 `enable_reception(self, msg)`（node L93）
- 属性 `self.enable_reception`（第一次收到 Bool 消息后被赋值，node L95）

在收到消息前，`if self.enable_reception:` 判断的是“绑定方法”，恒为真；收到消息后才变成布尔值。发布线程的行为因此在运行中改变。**这不是可靠的启用状态。**

---

## 6. 反馈真实性（D-10）

| 问题 | 结论 | 证据 |
|---|---|---|
| `bus_servo_read_position` 是编码器读数还是命令回显？ | **无法从源码确认**。协议子命令 `0x05` 由 STM32 固件实现；SDK 只负责收发。需要固件审计或实机独立核验。 | SDK L537-539 |
| ROS Driver 是否周期读取真实位置？ | **否**。Driver 只在 `GetBusServoState` 服务被调用时读取，且逐个舵机阻塞读。 | node L204-250 |
| `/controller_manager/joint_states` 是什么？ | **最后下发脉冲的缓存换算**，不是测量值。 | `controller_manager.py:97-114` |
| `/controller_manager/servo_states` 是什么？ | 同上，命令回显。 | 同上 |
| 是否有时间戳/采样年龄？ | 没有。SDK 返回值不带时间；Driver 也不打时间戳。 | SDK 全部读 API |
| 是否有丢样/采样率概念？ | 没有。读取是请求-响应，不是流。 | — |

**因此**：G1 的任何“反馈”必须标注来源为 `commanded` 或 `measured_on_demand`，并附带 `read_at` 与 `age_ms`。在 `bus_servo_read_position` 的真实语义被实机确认前，界面不得写“实际角度/编码器反馈”。

---

## 7. 能力矩阵

| 能力 | SDK 是否提供 | 备注 |
|---|---|---|
| 单/多舵机位置命令（带时长） | ✅ | 开环，无 ACK |
| 位置读取（单舵机） | ✅ | 阻塞、无超时、可能错配 |
| 温度/电压/扭矩状态读取 | ✅ | 逐个舵机 |
| 角度/电压/温度限位读写 | ✅ | 写 EEPROM |
| 偏移读写/保存 | ✅ | 标定用 |
| 扭矩使能/关闭 | ✅ | 无确认 |
| 停止命令 | ✅ | 无确认 |
| 电池电压 | ✅ | 依赖 `enable_recv` |
| 固件版本 | ❌ | 无 API |
| 舵机型号/序列号 | ❌ | 无 API |
| 舵机错误码/故障字 | ❌ | 只有每次请求的 `success` 字节 |
| 多舵机同步读 | ❌ | 只能逐个读 |
| 写命令 ACK | ❌ | 串口写成功即返回 |
| 轨迹/插值 | ❌ | 只有“duration + 目标脉冲” |
| 看门狗/超时停止 | ❌ | 无 |
| 硬件急停状态 | ❌ | 无信号 |
| 碰撞/力矩保护 | ❌ | 舵机内部限流未知 |
| 多圈/绝对编码器 | ❌ | 脉冲范围 0–1000 |

---

## 8. 异常与错误处理现状

| 场景 | 当前行为 | 风险 |
|---|---|---|
| 设备不存在 | `serial.SerialException`（构造时） | 进程退出；无诊断对象 |
| 串口拔出 | `port.read()` 抛 `SerialException`（接收线程） | 线程死亡，`enable_recv` 仍为真，读请求永久阻塞 |
| 舵机不响应 | `queue.get` 永久阻塞 | Driver 冻结 |
| 回包 CRC 错 | 打印“校验失败”，丢弃 | 调用方继续等，可能永久阻塞 |
| 回包长度/格式错 | `struct.error` 抛到调用线程 | 服务回调异常；客户端可能收不到响应 |
| `success != 0` | 返回 `None` | 调用方若直接索引 → `TypeError` |
| 非法按键事件 | `ValueError` | 发布线程退出 |

**没有**：重连、退避、错误码、结构化日志、指标计数。

---

## 9. 单位与数值边界（未确认项）

| 量 | 源码线索 | 状态 |
|---|---|---|
| 位置脉冲 | 0–1000（`ServoPosition.msg` 为 uint16；`servo_controller` 裁剪 0–1000） | `Verified in source`（应用层）；舵机物理范围 `Unknown` |
| 位置读取 | `<h` 有符号 16 位 | 单位应为脉冲，`Unknown` |
| 电压 | `<H`；`servo_tool` 设置时乘 100 | 单位 `Unknown`（可能 10 mV / mV） |
| 温度 | `<B`；`set_temp_limit` 传摄氏度 | 单位 `Unknown` |
| 电池 | `get_battery` 注释写 “mAh”，但字段是 `<H` 电压包 | **注释错误**；单位 `Unknown` |
| 时长 | 秒（float），内部转 ms，uint16 | `Verified in source`；上限 65.535 s（uint16 ms），`servo_controller` 再裁剪到 30 s |

---

## 10. 后端包装规则（Proposed，G1 必须遵守）

1. **禁止**在 FastAPI 进程内 `import Board` 或打开 `/dev/rrc`。唯一写路径是 ROS 2 Driver。
2. 所有读取必须带 **客户端侧超时**（建议 ≤ 300 ms），超时后标记 `Unknown`，不得重试到阻塞。
3. 读取结果必须与请求的 `servo_id` 关联校验；若无法校验，UI 标注“无法确认采样归属”。
4. 任何写命令最多记到 `Dispatched`；`Completed` 只能由独立位置读取 + 容差 + 时间窗支撑。
5. 串口/驱动故障必须表现为 `Fault/Degraded`，不得用 0 或上一次值填充。
6. 若必须修补 Driver/SDK（如加超时），单独提 patch + 风险评估，经 Board 批准；不得在 Debug Studio 仓库里悄悄改厂商代码。

---

## 11. 待实测清单（D-03）

| ID | 问题 | 方法 | 通过标准 |
|---|---|---|---|
| S-01 | `bus_servo_read_position` 是真实位置还是回显？ | 断电手动转动舵机后读取；对比命令值 | 读数随手动转动变化 |
| S-02 | 单次读取耗时分布 | 100 次读取计时 | p50/p95/p99 记录 |
| S-03 | 6 舵机轮询可达频率 | 循环读 6 个 ID | 记录成功率与错配率 |
| S-04 | 丢包/错配率 | 连续读并校验回包 servo_id | 0 错配才可用于反馈 |
| S-05 | 位置重复精度 | 同目标 30 次，读回 | 记录 σ |
| S-06 | 回差 | 正/反方向逼近同一点 | 记录差值 |
| S-07 | 电压/温度单位 | 与万用表/室温对比 | 确认换算 |
| S-08 | 断线行为 | 拔 USB，观察 Driver/服务 | 明确故障表现 |
| S-09 | 舵机缺失行为 | 断开一个舵机，读它 | 是否会永久阻塞 |
| S-10 | 固件版本 | 固件源码/串口协议审计 | 得到可记录版本号 |

> 以上全部需要 **物理隔离、低速、人工看护、可立即断电**，且先通过 Board 的运动授权。
