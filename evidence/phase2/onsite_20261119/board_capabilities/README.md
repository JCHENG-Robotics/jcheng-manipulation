# Controller board capability probe — 2026-11-19

> Status: **live probe + vendor-firmware cross-check.** The robot was powered,
> idle and connected over the wired link (`10.42.0.218`). No joint, motor,
> gripper or base motion was commanded. The only writes were one buzzer beep,
> two LED flashes and one OLED text publish (all fire-and-forget, no ack).
> Battery before/after: **7812 mV / 7812 mV**.

## What the board is

| Item | Value | Evidence |
|---|---|---|
| MCU board | STM32 (vendor "扩展板"/ROS controller), vendor firmware | `[S]` `ArmPiUltra/STM32` |
| Host link | CH340 USB-serial `1a86:7523` → `/dev/rrc` → `/dev/ttyUSB0`, 1 Mbaud | `[V]` `lsusb`, `udevadm`, SDK `Board(baudrate=1000000)` |
| USB topology | port `1-2` = controller; port `1-1.1` reserved for `wonderechopro` | `[S]` `/etc/udev/rules.d/99-rrc.rules` |
| ROS node | `/ros_robot_controller` (one node, 1 publisher per report topic) | `[V]` `ros2 topic info` |
| Firmware source | STM32 source in the vendor tree is **older than the flash**: it registers only LED/BUS_SERVO/BUZZER/SYS, yet the flashed board accepts motor-duty packets | `[S]` + earlier live test |

The vendor firmware snapshot describes this hardware:

- **Battery ADC**: PC3 / ADC1_IN13, 11:1 divider, internal VREFINT reference,
  report every 100 ms; default low-voltage alarm **6400 mV** (beeps every 10 s).
- **Bus servos**: 6-channel UART bus, position / VIN / temperature / offset /
  angle-limit / VIN-limit / temperature-limit / torque on-off / ID write-read.
- **LEDs ×2**: `LED` PB9, `LED1` PB15. In the source snapshot LED1 blinks
  (10 ms on / 50 ms off) on **every valid received packet** — an activity light.
- **Buzzer ×1**: PB8 / TIM4_CH3, 50 % duty PWM, 0–20 kHz.
- **Keys ×2**: KEY1 PD2, KEY2 PB3, rising-edge interrupts; the driver reports
  click / press / long-press / repeat / release / double / triple click.
- **SPI flash W25Q64**: stores the `"Hiwonder"` magic, servo offsets and arm
  action groups (action-group erase/write is supported over the packet link).
- **Spare GPIO**: IO18 PB4, IO19 PB5 (no ROS interface).
- **I2C1**: initialized and mutex-guarded, no user in this source snapshot.
- **UARTs**: USART1 1 Mbaud + DMA (host link via CH340), USART2 115200 on
  PA2/PA3 with `BUS_EN` PA1 (bus-servo power), USART3 9600.
- The packet enum defines `MOTOR`, `PWM_SERVO`, `IMU`, `GAMEPAD`, `SBUS`,
  `OLED`, `RGB`, `CONVEYOR`, `STEPPER`, but this snapshot only *implements* LED,
  bus servo, buzzer and SYS.

## Live results

| Capability | Status | Evidence |
|---|---|---|
| Battery voltage | **works** — `7812` mV, 10.05 Hz, jitter 81–106 ms | `/ros_robot_controller/battery` |
| Bus servos (arm ×5 + gripper) | **works** — normal arm operation, `get_state` register reads | previous sessions + `ros2 topic info` |
| Chassis motor duty (4 channels) | **works, open-loop** — no encoder readback (verified 2026-11-18) | memory `armpi-chassis-link` |
| Buzzer | **works** — 2 kHz beep ×2 heard by the operator | publish + operator confirmation |
| LED 1 / 2 | **works** — both flashed ×3, seen by the operator | publish + operator confirmation |
| OLED text | command accepted; **no display/ack** — source has no OLED driver | publish log |
| IMU (accel + gyro) | **absent** — 0 messages in 10 s (250 expected publish cycles) | `ros2 topic echo` |
| Gamepad (`/joy`) | **absent on the board path** — 0 messages; no `/dev/input/js*`; the real gamepad is a USB dongle read by `joy_node`/`joystick_control` | `lsusb`, `ls /dev/input` |
| SBUS RC input | **absent** — 0 messages, no receiver | `ros2 topic echo` |
| Keys KEY1/KEY2 | present in firmware; **removed by the operator** — not testable on this unit | `[S]` + operator |
| PWM servo outputs | **not probed** — `get_state` blocks forever when no report arrives (`get(block=True)`, shared callback group), and sending positions could move an attached servo | `[S]` SDK |
| Stepper / conveyor | protocol only, accessories not in this kit | `[S]` |
| Encoders / odometry | **none** — motor duty is write-only | `[S]` SDK + 2026-11-18 test |
| Current sensing / e-stop | **none** | `[S]` |

## Practical conclusions

- The only usable **sensor** on the controller board is the battery voltage ADC.
  There is **no IMU and no wheel encoder**, so no odometry or inertial state can
  come from this board.
- The only usable **actuators** are the 4 open-loop motor duty channels and the
  6 bus servos (arm + gripper).
- Usable **indicators/interaction**: buzzer and two LEDs (both confirmed), and
  two keys — the keys are physically removed on this unit, so they are not
  usable. The OLED and PWM-servo headers exist only in the protocol/source,
  not in this build.
- `ros_robot_controller_node` also publishes `/sbus` and `/joy` topics with a
  valid publisher count; both stay silent on this robot. Do not wait on them.

## Commands used

```bash
# battery
ros2 topic echo --once /ros_robot_controller/battery
ros2 topic hz /ros_robot_controller/battery --window 10

# report topics (all silent except battery)
ros2 topic echo /ros_robot_controller/imu_raw        # 10 s, 0 messages
ros2 topic echo --once /ros_robot_controller/joy     # timeout
ros2 topic echo --once /ros_robot_controller/sbus    # timeout

# fire-and-forget writes (no ack)
ros2 topic pub --once /ros_robot_controller/set_buzzer \
  ros_robot_controller_msgs/msg/BuzzerState "{freq: 2000, on_time: 0.15, off_time: 0.15, repeat: 2}"
ros2 topic pub --once /ros_robot_controller/set_led \
  ros_robot_controller_msgs/msg/LedState "{id: 1, on_time: 0.4, off_time: 0.4, repeat: 3}"
ros2 topic pub --once /ros_robot_controller/set_led \
  ros_robot_controller_msgs/msg/LedState "{id: 2, on_time: 0.4, off_time: 0.4, repeat: 3}"
ros2 topic pub --once /ros_robot_controller/set_oled \
  ros_robot_controller_msgs/msg/OLEDState "{index: 0, text: TEST}"
```

## Open items

1. Forward the vendor STM32 source snapshot to the flashed firmware version, or
   stop treating `ArmPiUltra/STM32` as the board's ground truth — it lacks the
   motor handling that the live board demonstrably has.
2. `ros2 topic echo /ros_robot_controller/button` cannot be exercised on this
   unit until the KEY1/KEY2 switches are reinstalled.
