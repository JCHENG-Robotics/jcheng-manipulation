# ArmPi Ultra platform freeze — 2026-11-28

Status: **Frozen** for the ArmPi Ultra platform. This record does not stop any
service on the robot and does not send motion. The on-robot runtime change
(stop the vendor motion gateway, keep the read-only gateway) is listed in §6
and needs explicit per-session authorization.

Scope: the platform, not the repositories. `jcheng-system`,
`jcheng-perception`, `jcheng-applications` and the other product repositories
keep evolving for the new fixed cell. `jcheng-manipulation v1.0.0` stays
`Frozen` and unchanged.

Decision record: [`17-platform-pivot.md`](../../17-platform-pivot.md) and
[`ADR-0007`](../../adr/0007-fixed-cell-piper-first.md), approved 2026-11-28.

## 1. What stays verified

These results are hardware results on the ArmPi Ultra. They stay valid for
this platform and are **not** transferred to PiPER X or AR5.

| Item | Result | Evidence |
|---|---|---|
| M4 read-only telemetry | 30 min, 1799/1799 samples `measured`, `writes=0` | [`../m4-readonly-20261123/`](../m4-readonly-20261123/README.md) |
| M5.2 supervised joint | `joint1` 505 vs 512 pulses (within the 8-pulse tolerance) | [`../m52-final-20261124/`](../m52-final-20261124/) |
| M5.2 gripper | close 183→645, open 647→183, timer-completion + bus evidence | [`../m52-final-20261124/`](../m52-final-20261124/) |
| M5.2 stop | 0.266 ms software stop; in-flight command did not continue | [`../m52-final-20261124/`](../m52-final-20261124/) |
| M6 S13 replay | 21/21 decisions consistent; validation mean 0.0166 ms, p95 0.0352 ms | [`../m6-s13-20261124/`](../m6-s13-20261124/) |
| M6 S17 plan-only | real `move_group` `SUCCESS`, 8-point plan, `plan_only=true`, 0 controller servers | [`../m6-s17-20261125/`](../m6-s17-20261125/) |

Claim level: these are **Partially verified** for the platform as a whole.
M4/M5.2/S13/S17 scopes passed; the platform was never validated end to end.

## 2. What was never verified

These stay `Not started` or `Unverified`. They are retired with the platform,
not deferred.

| Item | State |
|---|---|
| M6 S14 chassis pulse on hardware | Not started |
| Full G8 (locate → grasp → lift → place) | Not started |
| S7 stale-state injection, S9 runtime publisher | Unverified |
| Teleoperation with hardware streaming | Unverified |
| M7 first external module pilot | Not started |
| Functional safety (e-stop, brake, encoder, self-collision) | Not claimed |

## 3. Pins and image

| Item | Value |
|---|---|
| Contract | `v0.2.0`, commit `b03a054a59f70c24741fce37ebe1bb03978d9021` |
| Contract hash | `sha256:afca22a42f7163e0351d185d5a8577b0b51f2f3da4b92278f161a4a7d2c35b1c` |
| Runtime image tag | `armpi-ultra:flatten-20261125` |
| Runtime image id | `sha256:7e823327a18754cd795978191b07ffa94ab6ea1beb0cd466df6d7f81455948b1` |
| Runtime image digest | `ghcr.io/jcheng-robotics/armpi-ultra@sha256:d4e6f3901f1c43e08f4018ff7f7b55dec21b2955dba8b98404a440a1f89df15f` |
| Baseline | `jcheng-manipulation` `v1.0.0`, `b3411e4`, unchanged |

Image record: `jcheng-system/deploy/armpi-ultra/container/image.lock`
(2026-11-25, 7.1 GB, linux/arm64).

## 4. Platform state

| Surface | State | Note |
|---|---|---|
| Repo state, ArmPi platform | `Frozen` | This record |
| `jcheng-system/deploy/armpi-ultra/` | `Frozen` banner | Files and image lock retained |
| Motion authority | To be revoked on the robot (§6) | Not done by this record |
| Read-only `/v1` gateway | Retained | Demo and rollback surface |
| `jcheng-ops` ArmPi probes | `legacy` | No new installs |
| `jcheng-mobile` | `Experimental`, dormant | No chassis work in P0–P6 |

## 5. Evidence index

The ArmPi evidence set is unchanged:

- `jcheng-design/docs/ecosystem/evidence/m4-readonly-20261123/`
- `m52-20261124/`, `m52-retry-20261124/`, `m52-run3-20261124/`,
  `m52-final-20261124/`
- `m6-s13-20261124/`, `m6-s17-20261125/`
- `jcheng-system/e2e/reports/` (M3–M6, replay, concurrency, guard race)

No file is deleted, moved or rewritten by the freeze.

## 6. On-robot actions (pending authorization)

These change the live robot. Each needs a separate, explicit, per-session
operator authorization. Do not run them from CI or unattended.

1. Confirm the current `/v1` read-only health and that no motion session is
   active.
2. Stop `jcheng-gateway-vendor.service` (the M5.2 motion authority). Keep
   `jcheng-system-readonly.service` running.
3. Do **not** restart the `ArmPiUltra` vendor container: its startup check
   moves the arm.
4. Record the publisher set before and after the stop; verify no
   `/servo_controller` publisher remains.
5. If the arm is to be left powered, leave only the read-only gateway; physical
   power-down is the final state once the demo is no longer needed.

## 7. Rollback

1. Software stop and disarm if a motion session was enabled.
2. Start only `jcheng-system-readonly.service`; do not start the motion
   gateway.
3. Probe the application layer (`/v1/health`, TCP banner), not ping.
4. Read the bus for the actual positions and record them.
5. Mark the evidence entry `rollback` with reason, version and positions.

The vendor container is never restarted as part of a rollback.

## 8. Open items

- P3 operator and supervisor are not assigned (the only unchecked decision in
  `17-platform-pivot.md`).
- The P0 documentation changes in `jcheng-system` (`deploy/armpi-ultra` banner
  and the M6 scope retirement note) are tracked separately.
- This record and the design documents are on a local branch; push and PR are
  pending owner confirmation.
