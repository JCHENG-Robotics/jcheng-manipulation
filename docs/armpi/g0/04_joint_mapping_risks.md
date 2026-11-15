# 04 · 关节映射与限位风险（D-04 / FR-02）

> 证据等级：`Verified in source`（映射来源与换算公式）；限位真实性 `Unknown`
> 对应 PRD：FR-02、FR-03、SR-01、SR-05、D-04、D-10

---

## 1. 现有映射的唯一来源

`VENDOR/ros2_ws/src/driver/servo_controller/config/servo_controller.yaml`

| 关节 | 舵机 ID | init | min | max | 备注 |
|---|---:|---:|---:|---:|---|
| `joint1` | 6 | 500 | 0 | 1000 | 底座旋转 |
| `joint2` | 5 | 500 | 0 | 1000 | 大臂 |
| `joint3` | 4 | 500 | 1000 | 500 | **反向**（min > max） |
| `joint4` | 3 | 500 | 0 | 1000 | 小臂 |
| `joint5` | 2 | 500 | 0 | 1000 | 腕部 |
| `r_joint` | 1 | 700 | 0 | 1000 | 夹爪 |

启动自检使用的“初始姿态”脉冲（`bringup/startup_check.py:49`）：

```
(6,500) (5,600) (4,800) (3,110) (2,500) (1,210)
```

---

## 2. 单位换算

`joint_position_controller.py:11-14`：

```python
RADIANS_PER_ENCODER_TICK = 240 / 360 * (2π) / 1000   # 0.00418879 rad/tick
ENCODER_TICKS_PER_RADIAN = 1 / RADIANS_PER_ENCODER_TICK  # 238.732 ticks/rad
ENCODER_RESOLUTION = 1000
```

即：**1000 tick = 240°**。这是对所有关节统一的线性假设。

换算：

```
raw → rad:  flipped ? (init - raw) * r : (raw - init) * r
rad → raw:  flipped ? init - rad / r : init + rad / r
```

`flipped = min > max`（`joint_position_controller.py:25`）。

### 2.1 由配置推导的软件范围

| 关节 | 方向 | raw 范围 | 由公式得到的 rad 范围 |
|---|---|---|---|
| joint1 | + | 0–1000 | −2.094 … +2.094 |
| joint2 | + | 0–1000 | −2.094 … +2.094 |
| joint3 | − | 500–1000 | −2.094 … 0.000 |
| joint4 | + | 0–1000 | −2.094 … +2.094 |
| joint5 | + | 0–1000 | −2.094 … +2.094 |
| r_joint | + | 0–1000 | −2.932 … +1.257 |

> 这些是 **软件配置范围**，不是机械限位。PRD 要求区分 `provisional` 与 `hardware_verified`；当前全部是 `provisional`。

---

## 3. 五处已经确认的不一致

### 3.1 URDF 与软件范围不一致

`armpi_ultra.xacro` 给 joint1–joint5 的限位全部是 `±2.09 rad`（L52, 81, 110, 139, 168）。

| 关节 | URDF | 软件映射 | 冲突 |
|---|---|---|---|
| joint3 | −2.09 … +2.09 | −2.094 … **0.000** | URDF 允许正角度，实际会被静默裁剪到 0 |
| r_joint | −1.57 … +1.57（`gripper.urdf.xacro:44-60`） | **−2.932 … +1.257** | 下界比真实窄、上界比真实宽 |

**后果**：MoveIt/RViz 以 URDF 为准规划，可能生成软件层无法执行或会被裁剪的目标。G2 接入 MoveIt 前必须统一限位来源。

### 3.2 MoveIt 初始姿态不是真实初始姿态

`armpi_ultra_moveit_config/config/initial_positions.yaml`：

| 关节 | MoveIt 初始 (rad) | 换算 pulse | 与 servo init 的差 |
|---|---:|---:|---:|
| joint1 | 0.000 | 500 | 0 |
| joint2 | 0.731 | 674 | +174 |
| joint3 | −1.613 | 885 | +385 |
| joint4 | −1.659 | 104 | −396 |
| joint5 | 0.000 | 500 | 0 |
| r_joint | 1.570 | **1075 → 裁剪 1000** | +375 |

**后果**：

- 该姿态与厂商启动姿态（§1）差异很大，尤其 `r_joint` 方向相反；
- `r_joint = 1.57 rad` 已超出软件上限 `1.257 rad`，会被 `servo_controller` 裁剪；
- 若把该文件当作“当前状态”喂给 move_group，规划起点是假的。

### 3.3 标定偏移没有被应用

SDK 支持读写舵机偏移（`bus_servo_read_offset`、`bus_servo_set_offset`），但 `JointPositionController` 的换算 **完全不使用 offset**。

若任一舵机 EEPROM 中的 offset 非 0：

- `raw → rad` 的结果与真实机械角度存在固定偏差；
- 不同关节偏差不同；
- 现有 ROS 栈无法察觉。

**必须实测**：读取 6 个舵机的 offset，记录到 `joint_mapping.yaml`。当前 `Unknown`。

### 3.4 超限是静默裁剪，不是拒绝

`pos_rad_to_pulse()` 在 L43-48 把超限角度裁剪到 `min_angle/max_angle`，`ServoManager.set_position()` 在 L105 再把脉冲裁剪到 0–1000。两层都不报错、不记录。

这直接违反 FR-02 的验收要求：“超限目标均被拒绝并写入日志”。

**后端必须在调用 ROS 之前拒绝**，不能依赖厂商层裁剪。

### 3.5 映射没有版本、没有验证状态

`servo_controller.yaml` 只有数字，没有：

- `version`
- `verified: true/false`
- `source`（厂商默认 / 实测）
- `calibration_version`
- `verified_at` / `verified_by`

因此无法满足 PRD “映射可版本化”和“实验可追溯”。

---

## 4. 其它风险

| ID | 风险 | 等级 | 说明 |
|---|---|---|---|
| M-01 | 240°/1000 tick 的统一假设 | 高 | 不同舵机可能不同；必须逐关节实测 |
| M-02 | 舵机 ID 顺序与关节顺序相反（6→1） | 中 | 容易在日志/导出中错位；后端必须以关节名为主键 |
| M-03 | `r_joint` 夹爪方向未验证 | 高 | 初始 700、home 210、MoveIt 1.57 三者语义不一致；夹爪开合方向未知 |
| M-04 | 舵机 EEPROM 限位未知 | 高 | `bus_servo_read_angle_limit` 可读；需逐舵机记录 |
| M-05 | 机械限位未知 | 高 | 软件 0–1000 不等于机械可动范围；PRD 要求实机测量 |
| M-06 | 反向关节只靠 `min>max` 表达 | 中 | 配置可读性差，易被误改；建议显式 `direction: -1` |
| M-07 | 无零位标记 | 高 | `init` 是“上电默认”，不一定是机械零位；需定义并记录 zero 参考 |
| M-08 | 断电后位置丢失 | 中 | 总线舵机通常为相对/绝对？`0x05` 语义 Unknown；上电位置是否保持未验证 |
| M-09 | 夹爪 `r_joint` 是连续旋转还是有限角度 | 未知 | 0–1000 脉冲，但机械结构可能限制 |

---

## 5. 建议的映射文件（Proposed）

`configs/joint_mapping.yaml`，由后端加载并计算 `mapping_hash`：

```yaml
schema_version: 1
mapping_version: 0.1.0-provisional
robot_id: armpi-ultra-5dof
verified: false
verified_at: null
verified_by: null
source: "vendor servo_controller.yaml + joint_position_controller.py"
units:
  raw: pulse
  raw_min: 0
  raw_max: 1000
  rad_per_tick: 0.00418879
joints:
  - name: joint1
    servo_id: 6
    zero_raw: 500
    direction: 1
    raw_min: 0
    raw_max: 1000
    rad_min: -2.0944
    rad_max: 2.0944
    limit_source: software_config
    limit_verified: false
    offset_raw: null
    offset_verified: false
    notes: "vendor default; mechanical limit not measured"
  # ... joint2..joint5, r_joint
safety:
  max_step_rad: 0.05
  min_duration_s: 0.5
  max_duration_s: 5.0
  feedback_max_age_ms: 500
```

后端启动时校验：

- 所有 `servo_id` 唯一且在 1–6；
- `raw_min < raw_max`（方向由 `direction` 表达，不再用 min>max）；
- `rad_min < rad_max`；
- `verified: false` 时只允许“受限测试模式”，具体门槛由 Board 批准。

---

## 6. 验证计划（需 Board 运动授权）

> 原则：物理隔离、低速、单关节、人工看护、可立即断电；不得自动探测机械极限。

| 步骤 | 动作 | 证据 |
|---|---|---|
| V-01 | 读取 6 个舵机的 `id / offset / angle_limit / vin_limit / temp_limit`，只读 | 原始返回值记录到标定日志 |
| V-02 | 比较 EEPROM 限位与 `servo_controller.yaml` | 差异表 |
| V-03 | 扭矩关闭、机械固定，手动小幅转动单关节，同时读取位置 | 判断 `0x05` 是否为真实传感器（D-10） |
| V-04 | 单关节、最小步长（建议 ≤ 10 pulse）、长时长（≥ 1 s），正反各一次 | 方向、回差、重复性 |
| V-05 | 在保守范围内找到“可重复停止点”，不外推为机械极限 | 软限位候选 |
| V-06 | 记录零位参考与 `init` 的关系 | zero 定义 |
| V-07 | 重复 30 次，统计 σ 与回差 | 写回 `joint_mapping.yaml` 的 `verified` 字段 |
| V-08 | 夹爪单独验证开/合方向与安全范围 | `r_joint` 语义 |

**V-03 是关键**：如果手动转动时读数不变，则 `bus_servo_read_position` 是命令回显，整个“反馈”概念需要重新定义（见 02 文档 §6）。

---

## 7. D-04 结论

| 问题 | 结论 |
|---|---|
| 关节↔舵机 ID 映射是否存在？ | ✅ 存在，来自 `servo_controller.yaml` |
| 是否可信？ | ⚠️ 仅 `provisional`；无版本、无标定、无限位验证 |
| 单位换算是否统一？ | ⚠️ 统一按 240°/1000 tick 线性假设，未逐关节验证 |
| 是否有反向关节？ | ✅ `joint3`，通过 `min>max` 表达 |
| 是否有夹爪？ | ✅ `r_joint`，ID 1；方向与范围未验证 |
| 是否可直接用于运动？ | ❌ 必须先完成 V-01…V-08，并由 Board 批准受限测试 |
