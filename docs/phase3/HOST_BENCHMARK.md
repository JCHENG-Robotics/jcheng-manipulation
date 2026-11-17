# Host and ROS/DDS Benchmark

> Status: **Implemented and measured read-only on 2026-11-17 (Session 4).**
> One laptop and one Raspberry Pi 5 snapshot; no load sweep, no motion and no
> real-time claim.

## What is measured

`armpi_analysis/host_benchmark.py` has two modes.

| Mode | Metrics |
|---|---|
| `host` | CPU model/cores/load/utilization (1 s `/proc/stat` sample), memory, root disk, thermal zone, container/cgroup limit, kernel, Python, ROS/RMW/domain environment |
| `ros` | received-message count and rate, inter-arrival mean/jitter/p95, header age, publisher count, topic type, graph node/topic counts |

The `ros` mode uses a best-effort sensor-data QoS profile by default, so it can
receive from both best-effort camera publishers and reliable state publishers.
It never publishes and never commands the robot. `tools/phase4/run_benchmark.sh`
collects the laptop host, the Pi host, the Pi container, the laptop ROS side
and a ping RTT in one read-only run.

## Setup

| Item | Laptop | Robot |
|---|---|---|
| Host | Lenovo Legion R9000P, Ubuntu 24.04 | Raspberry Pi 5, Debian 12 |
| CPU | AMD Ryzen 7 5800H, 16 logical cores | 4 cores, aarch64 |
| Memory | 27.9 GB | 8.05 GB |
| ROS / RMW | Jazzy host, Fast DDS | Humble container `ArmPiUltra`, CycloneDDS on `eth0` |
| Link | USB Ethernet `enx00e04ca373c7` | `eth0 10.42.0.218/24` |
| Domain | 0 | 0 |

The laptop and robot use different RMW implementations. The cross-host numbers
therefore include a Fast DDS ↔ CycloneDDS interop path; the robot-container
numbers are the local CycloneDDS path.

## Results

### Hosts

| Metric | Laptop | Raspberry Pi 5 |
|---|---:|---:|
| CPU utilization (1 s) | 4.4–4.7% | 1.8% idle, 10.7% with camera |
| Load average (1 min) | 1.34–1.89 | 0.47–0.65 |
| Memory used | 33.9% of 27.9 GB | 8.7–10.1% of 8.05 GB |
| Root disk used | 14.3% | **84.7%** (6.6 GB free) |
| Temperature | not exposed | 46.9–48.0 °C |
| Ping RTT avg / max | — | 0.239–0.254 ms / 0.281–0.298 ms, 0% loss |

The Pi root filesystem is the operational risk: 84.7% used and only 6.6 GB
free. The disk-cleanup work package (R14) was not part of Session 4.

### ROS topics, no camera running

| Topic | Robot container | Laptop (cross-host) |
|---|---|---|
| `/joint_states` | 20.00 Hz, jitter 0.164 ms, p95 50.27 ms | 20.00 Hz, jitter 0.299 ms, p95 50.37 ms |
| `/controller_manager/joint_states` | 48.80 Hz, jitter 0.127 ms | 48.53 Hz, jitter 0.465 ms |
| `/ros_robot_controller/battery` | 9.99 Hz, jitter 3.39 ms | 9.99 Hz, jitter 4.47 ms |
| Header age, `/joint_states` | 0.54 ms | 4.34 ms |

“Jitter” is the standard deviation of inter-arrival times; header age is the
receiver wall clock minus `header.stamp`.

### ROS topics, camera running

The camera launch was started only for this measurement and stopped afterwards.

| Topic | Robot container | Laptop (cross-host) |
|---|---|---|
| `/depth_cam/rgb/image_raw` | 8.33 Hz, interval 120.0 ms, jitter **104.1 ms**, p95 **313.2 ms**, age 24.6 ms | 11.83 Hz, interval 84.5 ms, jitter 20.8 ms, p95 121.5 ms, age 34.7 ms |
| `/joint_states` | 20.00 Hz, jitter 1.449 ms | 20.00 Hz, jitter 0.599 ms |
| `/ros_robot_controller/battery` | 9.78 Hz, jitter 21.25 ms | 10.00 Hz, jitter 5.05 ms |

Cross-checks:

```text
ros2 topic hz /depth_cam/rgb/image_raw  (laptop, cross-host)  ~12.0 Hz
ros2 topic hz /depth_cam/rgb/image_raw  (container, local)    ~7.0 Hz
host_benchmark --mode ros               (container, local)     8.33 Hz
host_benchmark --mode ros               (laptop, cross-host)   11.83 Hz
```

The camera publishes at about **12 Hz**, not 8 Hz. The earlier project note of
8.0 Hz (and depth 8.2 Hz) came from an in-container subscriber and undercounts
because local DDS delivery drops/queues large images. The cross-host laptop
subscriber and `ros2 topic hz` agree at ~12 Hz. Use the cross-host number when
a camera rate is needed, and state the measurement location.

## Findings

1. **Use the cross-host camera rate.** In-container image subscription is not a
   reliable FPS measurement: 8.33 Hz with 104 ms jitter and a 313 ms p95 versus
   11.83–12.0 Hz cross-host. The camera is not an 8 Hz sensor.
2. **State topics are stable.** `/joint_states` is 20 Hz with sub-millisecond
   jitter locally and ~0.3–0.6 ms cross-host; the controller-manager stream is
   ~48.8 Hz. Cross-host header age is ~4.3 ms for state.
3. **The graph has a malformed participant.** `get_node_names()` in the robot
   container fails with `Failed to get node names: empty node name returned by
   the RMW layer`. Topic discovery and subscriptions still work; the laptop
   sees 6 nodes. The camera and static-TF logs also repeat `serdata.cpp`
   errors (`string data is not null-terminated` / `invalid data size`). This is
   an open environment defect, not fixed in Session 4.
4. **Camera cost is visible on the Pi.** Host CPU utilization rose from ~1.8%
   to ~10.7% when the camera ran; laptop CPU stayed under 5%. The laptop is not
   the bottleneck for these topics.
5. **Disk remains tight.** 84.7% used / 6.6 GB free. Long bags or calibration
   captures need cleanup first (R14, not done here).

## Reproduce

```bash
# Full read-only collection (laptop + Pi host + Pi container + laptop ROS)
tools/phase4/run_benchmark.sh evidence/phase3/session4_20261117/with_camera

# Laptop host only
python3 ros2_ws/src/armpi_analysis/armpi_analysis/host_benchmark.py \
  --mode host --json-out /tmp/host.json

# One topic from the laptop (Jazzy sourced)
source /opt/ros/jazzy/setup.bash
python3 ros2_ws/src/armpi_analysis/armpi_analysis/host_benchmark.py \
  --mode ros --topics /joint_states --duration-s 10 --json-out /tmp/ros.json
```

Raw reports:
[`evidence/phase3/session4_20261117/no_camera/`](../../evidence/phase3/session4_20261117/no_camera/)
and
[`with_camera/`](../../evidence/phase3/session4_20261117/with_camera/).

## Limitations

- One snapshot per host; no repeated runs, no CPU/memory load sweep, no thermal
  steady state.
- Laptop Fast DDS and robot CycloneDDS are different RMW stacks; latency is not
  a pure network measure.
- Header age is contaminated by clock offset across hosts. The local
  container values are the cleaner latency estimate.
- Topic rate is the received rate, not the sensor exposure rate; an overloaded
  subscriber can undercount.
- The benchmark is read-only but it does subscribe to live topics; it does not
  guarantee that the camera or controllers remain untouched by other clients.
