# Third grasp — adopted home, centring gate, one-step lift (2026-11-19)

> Status: **FAILED (operator-confirmed).** The robot ran the full sequence — the
> grasp, the close, the 150 mm lift, the lower, the release and the return to
> the adopted pose — but **the cube was never picked up**. The operator watched
> the cube stay on the floor. The two visual "confirmations" recorded during
> the run were wrong; the reasons are in §3 and are the main lesson of this
> round.

Operator requirements for this round:

1. home = **adopted** (start and finish);
2. before planning, the collected data must show the object at the **visual
   centre** of the lens;
3. do not move the robot about unnecessarily;
4. attempt the third grasp.

## 1. Why the centring gate could not be met (measured, not assumed)

| Fact | Value |
|---|---|
| Depth near cutoff | **0.176 m** (raw cloud at the adopted pose: 3,067 points, none closer than 0.176 m) |
| Camera–cube distance at adopted | **0.072 m** → depth blind |
| Centred (>0.20 m) requirement | the link4 mount centres the cube only at ≤0.10 m |
| Model at the table-down view | projection (447.9, 314.1) vs observed (448, 314) — **0.2 px** |
| Model at the centring poses | same cube read **(0.2502, −0.0354)** vs **(0.2303, 0.0151)** — 20/50 mm apart |
| Base move to centre at the view pose | 85 mm away → cube at 0.27 m radius, outside the floor reach |

The operator chose **proceed without centring** and, later, **a small chassis
correction**.

## 2. Sequence and measurements

### 2.1 Adopted home

`joint1 0, joint2 −0.6618, joint3 −1.8012, joint4 −0.6704, joint5 0`;
registers at the end **500 / 344 / 927 / 338 / 498** (reference 496/339/925/334/500).

### 2.2 Data collection (view pose) and the gate result

View `0.504, 0.685, −1.652, −1.559, 0`:

| Step | Cube in base_link | RGB centroid | Offset from (320, 200) |
|---|---|---|---|
| first collection | (0.2632, −0.0258) | (527.8, 311.1) | +207.8, +111.1 px |
| after chassis pulses | (0.2282, −0.0242) | (499.5, 357.0) | +179.5, +157.0 px |

Gate result: **not centred**; the operator accepted this.

### 2.3 Cube displacement during the centring experiments

The open gripper contacted the cube during the centring moves and pushed it
from **(0.2303, 0.0151)** to **(0.2632, −0.0258)** — 50 mm.

### 2.4 Chassis correction (six pulses, `motor_pulse.py`)

Duty 10 % per axis component; signs `x = [+,−,−,+]`, `y = [+,+,+,+]`; every
pulse ended with an echoed zero.

| Pulse | Command | Cube after (base_link) |
|---|---|---|
| 1 | forward 0.30 s `10,−10,−10,10` | (0.2520, −0.0280) |
| 2 | left 0.45 s `10,10,10,10` | (0.2477, −0.0421) |
| 3 | right 0.20 s `−10,−10,−10,−10` | (0.2468, −0.0404) (deadband: 1.7 mm) |
| 4 | right 0.45 s `−10,−10,−10,−10` | (0.2474, −0.0214) |
| 5 | forward 0.25 s `10,−10,−10,10` | (0.2419, −0.0326) |
| 6 | forward 0.40 s `10,−10,−10,10` | (0.2282, −0.0242) |

Lateral travel ~31–42 mm/s with a deadband below ~0.3 s; forward ~28–50 mm/s.

### 2.5 Grasp 3a — planner reach solution (failed)

| Item | Value |
|---|---|
| Target | (0.2419, −0.0326, −0.0357) |
| TCP | **(0.2422, −0.0347, −0.0269)**, offset 9.1 mm, **8.8 mm above the top** |
| Joints | `−0.201, −0.662, −1.601, −0.592, 0` |
| Result | the pads closed above the cube; the cube was pushed to (0.2482, −0.0240) |

### 2.6 Grasp 3b — proven deep family (executed, but the cube was not held)

| Item | Value |
|---|---|
| Joints | **`−0.150, −0.6618, −1.8012, −0.410, 0`** |
| TCP | (0.2287, −0.0242, −0.0402) — dx 0.5 mm, dy 0.0 mm, dz −3.3 mm |
| Close | `−2.22 → −0.63`, `status=4` |
| Lift | TCP (0.2291, −0.0238, 0.1079), **+148.1 mm** |
| Release / return | `status=4`; adopted registers 500/344/927/338/498 |
| **Actual outcome** | **the cube stayed on the floor** (operator-confirmed) |

## 3. Why the run's two "confirmations" were wrong

1. **The 15 mm diagnostic lift.** The cube's RGB centroid moved only 7.6 px,
   which was read as "held" versus an estimated ~60 px for an unheld cube.
   That estimate assumed a pure camera translation; the diagnostic pose also
   rotated the camera (j2 +0.07, j3 +0.06, j4 −0.10 rad), and at these poses the
   FK/hand-eye projection is off by tens of pixels. The 7.6 px was the
   projection change of a cube still on the floor.
2. **The wood at the bottom of the 150 mm lift image.** It was read as the held
   cube. The model projected the floor cube to v = 554 (outside the frame), but
   the model is off by 50–90 px in this joint region — the wood was the cube
   **still on the floor**, visible at the bottom edge.

Lesson: with the camera mounted ~0.10 m from the TCP, a held cube is always at
the frame's bottom edge and inside the depth near-cutoff, so a held-vs-floor
judgement from this camera needs a projection model that is *not* trusted at
those poses. A reliable check must make the **floor spot** visible (search the
redundant lift configurations for one where the cube's original floor position
projects into the frame) or use an independent sensor.

## 4. Open problems

1. `joint3 ≈ −1.6` reach solutions are not trustworthy for a floor grasp.
2. A failed close pushes the cube away (6 mm here); re-measure before retrying.
3. The held-vs-floor discrimination needs the floor spot in view (§3).
4. The chassis lateral axis has a ~0.3 s deadband and ~10 % scatter.
5. A local hand-eye re-calibration over the low/centring poses would make the
   centring gate feasible; the table-down-only calibration does not extrapolate.

## 5. Files

| File | Content |
|---|---|
| `a3_measure_view.jpg` | first data collection (gate: 65 %/56 % off centre) |
| `a3_grip.jpg`, `a3_lift.jpg` | grasp 3a at the grasp and at the lift |
| `a3b_grip.jpg`, `a3b_lift15.jpg` | grasp 3b and the 15 mm diagnostic |
| `a3b_lift150.jpg` | grasp 3b at 150 mm — the wood at the bottom is the floor cube |
| `a3_final_adopted.jpg` | after release, from the adopted pose |
| `seg_a3r2.json` | final measurement used for grasp 3b |
| `seg_a3_lift.json` | lift-pose segmentation (0 clusters) |
| `grasp3.json`, `grasp3b.json`, `grasp3c.json` | reach searches at the three cube positions |
| `center_view.py`, `center_validated.py` | centring searches and the distance constraint |
| `deep_fit.py` | fit of the proven deep family to the measured cube |
| `lift3.py`, `lift3b.py`, `lift_view.py` | lift targets and held-cube projections |
| `jac.py` | image Jacobian used for the centring corrections |
| `capture_cloud.py`, `proj_lift.py` | raw-cloud range check and projection checks |
