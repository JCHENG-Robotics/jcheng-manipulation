# Step 2 — physical power-cut behavior

Date: 2026-11-18 09:12–09:14 (robot local time, UTC+8).

## Procedure

1. Workspace cleared; the arm was stationary at the boot pose
   (`joint1=+0.8378`, other arm joints 0).
2. Operator cut the main power at the battery/switch for a few seconds.
3. Operator restored power; the Pi rebooted automatically.

## Observations

| Observation | Result |
|---|---|
| Arm behavior at power loss | **Immediate drop/sag** — no brake, no holding torque |
| Operator support | Arm was supported/attended; no damage reported |
| Network outage | Ping UP at 09:12:15, back at 09:13:44 → **~89 s** including boot |
| Service recovery | `start_node.service` active, container `ArmPiUltra` up |
| ROS recovery | All six core nodes and both action servers returned |
| Pose after boot | `joint1=+0.8378`, `r_joint=-0.8378` (vendor init re-homed the arm) |
| Rate after recovery | `/joint_states` 20.001 Hz, 0.050 s interval |
| Battery | **7704 mV** |

## Conclusions

- The only hardware-level stop is **main power removal**. There is no
  electronic e-stop input, no brake, and no documented firmware watchdog that
  holds the arm.
- Power loss is a **gravity event**: joints 2–5 fall immediately. Never run
  unattended motion, and keep the arm supported or the workspace clear whenever
  the power can be interrupted.
- Recovery is automatic, but the vendor init moves `joint1` to `+0.8378`
  (48°) at boot. Keep that swept volume clear.

## Caveats

- The laptop ping monitor also showed intermittent packet loss after recovery
  (UP/DOWN flapping); this is the known SSH/DDS-load behavior, not repeated
  power cycling. Only the 09:12:15 → 09:13:44 transition is the power event.
- A hard power cut is a filesystem-risk event. The root filesystem was 87% full;
  no corruption was observed, but back up before repeated tests.
