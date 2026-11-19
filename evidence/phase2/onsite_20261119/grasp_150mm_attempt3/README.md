# Third grasp — adopted home, centring gate, one-step lift (2026-11-19)

> Status: **PASS after one retry.** The first grasp of this round missed (the
> planner's reach solution closed the pads above the cube); the retry with the
> proven deep joint family held, lifted the TCP **+148.1 mm** in a single
> motion, was lowered back and released, and the arm returned to the
> **adopted** pose. The cube was displaced during the centring experiments and
> again by the failed close; both are recorded below.

Operator requirements for this round:

1. home = **adopted** (start and finish);
2. before planning, the collected data must show the object at the **visual
   centre** of the lens;
3. do not move the robot about unnecessarily;
4. attempt the third grasp.

## 1. Why the centring gate could not be met (measured, not assumed)

The gate was implemented and measured, then the physics was checked:

| Fact | Value |
|---|---|
| Depth near cutoff | **0.176 m** (raw cloud at the adopted pose: 3,067 points, none closer than 0.176 m) |
| Camera–cube distance at adopted | **0.072 m** → depth blind |
| Camera–cube distance needed for depth | ≥ 0.18 m |
| Centred (>0.20 m) requirement | camera must look at the cube, which the link4 mount only does at ≤0.10 m |
| Model validation at the table-down view | projection (447.9, 314.1) vs observed (448, 314) — **0.2 px** |
| Model at the centring poses | same cube read **(0.2502, −0.0354)** vs **(0.2303, 0.0151)** — 20/50 mm apart |
| Base move to centre at the view pose | 85 mm away from the robot → cube at 0.27 m radius, **outside the floor reach** |

The operator chose **proceed without centring** (validated view-pose recipe)
and, when the cube ended up outside the reach, **a small chassis correction**.

## 2. Sequence and measurements

### 2.1 Adopted home

Target `joint1 0, joint2 −0.6618, joint3 −1.8012, joint4 −0.6704, joint5 0`
(measured registers at the end: **500 / 344 / 927 / 338 / 498**, reference
496/339/925/334/500).

### 2.2 Data collection (view pose) and the gate result

View pose `0.504, 0.685, −1.652, −1.559, 0`; segmentation + RGB gate:

| Step | Cube in base_link | RGB centroid | Offset from (320, 200) |
|---|---|---|---|
| first collection | (0.2632, −0.0258) | (527.8, 311.1) | +207.8, +111.1 px |
| after chassis pulses | (0.2282, −0.0242) | (499.5, 357.0) | +179.5, +157.0 px |

Gate result: **not centred** (the first collection is 65 % / 56 % of the
half-frame). The operator accepted this because the validated measurement
chain requires this pose.

### 2.3 Cube displacement during the centring experiments

The open gripper contacted the cube during the centring moves at 0.20–0.21 m
and pushed it from **(0.2303, 0.0151)** to **(0.2632, −0.0258)** — 50 mm. This
is an operator-visible error of this session and is why the chassis correction
was needed.

### 2.4 Chassis correction (five pulses, `motor_pulse.py`)

Duty 10 % per axis component, calibrated signs `x = [+,−,−,+]`,
`y = [+,+,+,+]`; each pulse ended with an echoed zero:

| Pulse | Command | Cube after (base_link) |
|---|---|---|
| 1 | forward 0.30 s `10,−10,−10,10` | (0.2520, −0.0280) |
| 2 | left 0.45 s `10,10,10,10` | (0.2477, −0.0421) |
| 3 | right 0.20 s `−10,−10,−10,−10` | (0.2468, −0.0404) (deadband: 1.7 mm) |
| 4 | right 0.45 s `−10,−10,−10,−10` | (0.2474, −0.0214) |
| 5 | forward 0.25 s `10,−10,−10,10` | (0.2419, −0.0326) |
| 6 | forward 0.40 s `10,−10,−10,10` (after the failed close) | (0.2282, −0.0242) |

Lateral travel is ~31–42 mm/s and has a deadband below ~0.3 s; forward is
~28–50 mm/s.

### 2.5 Grasp 3a — planner reach solution (MISS)

| Item | Value |
|---|---|
| Target | (0.2419, −0.0326, −0.0357) |
| Plan | 5 PLAN_OK; TCP **(0.2422, −0.0347, −0.0269)**, offset 9.1 mm |
| Joints | `−0.201, −0.662, −1.601, −0.592, 0` |
| Problem | TCP **8.8 mm above the cube top**; the sampled solution sits in a configuration where the FK/hand-eye extrapolation is worst |
| Close | `r_joint −2.22 → −0.63`, `status=4` |
| Lift | TCP **(0.2428, −0.0349, 0.1220)**, `status=4` |
| Result | **cube not held** — at the lift the expected held-cube window (x 330–385, v 370–400) is floor colour (HSV S ≈ 8–11) while a held cube reads S ≈ 56; the cube stayed on the floor |
| Side effect | closing pushed the cube to (0.2482, −0.0240) |

### 2.6 Grasp 3b — proven deep configuration (PASS)

The first two grasps used `joint3 = −1.8012, joint4 ≈ −0.40`; that family is
the one whose model was physically verified. After one more forward pulse the
cube was at (0.2282, −0.0242, top −0.0369, 39.4 mm tall, 40.5 mm above the
floor):

| Item | Value |
|---|---|
| Joints | **`−0.150, −0.6618, −1.8012, −0.410, 0`** |
| TCP | **(0.2287, −0.0242, −0.0402)** — dx **0.5 mm**, dy **0.0 mm**, dz **−3.3 mm** (below the top, deeper than 3a) |
| Close | `r_joint −2.22 → −0.63`, `status=4` |
| Grip check | 15 mm diagnostic lift: the cube's RGB centroid moved **7.6 px**; an unheld cube would move ≈ 60 px at 0.10 m — **held** |
| One-step lift | `−0.150, 0.050, −1.220, −1.410, 0` → TCP **(0.2291, −0.0238, 0.1079)** |
| TCP travel | **+148.1 mm** |
| Object bottom | **≈ +151 mm above the floor** (−0.0795 m) |
| Lift image | cube visible between the fingers (`a3b_lift150.jpg`) |
| Lower / release | same grasp joints, `status=4`; `r_joint −2.22` `SUCCEEDED` |
| Return | adopted pose, `status=4`; registers **500 / 344 / 927 / 338 / 498** |
| Final cube | on the floor in front of the adopted camera (`a3_final_adopted.jpg`) |

## 3. What this round established

- The **centring gate is physically incompatible** with this camera mount and
  the 0.176 m depth cutoff: the only poses that centre the cube are ≤0.10 m
  from it (blind), and the FK/hand-eye model is 15–50 mm off in those poses.
  The gate is recorded as a measured limitation, not a silent omission.
- The model is excellent at the table-down view (0.2 px) and in the proven
  deep grasp family, and poor in the `joint3 ≈ −1.6, joint4 ≈ −0.6`
  configurations the reach planner prefers at this cube position.
- **Use the proven deep family** (`joint3 = −1.8012`, `joint4 ≈ −0.40`) when
  the cube is inside the reach; the reach planner's flatter solutions can close
  above the cube.
- A 15 mm diagnostic lift (image displacement of the cube's centroid) is a
  reliable, cheap grip check; the held cube at 150 mm is at the bottom edge of
  this camera's frame, so the lift itself is not self-evident on the RGB.

## 4. Open problems

1. `joint3 ≈ −1.6` reach solutions are not trustworthy for a floor grasp;
   restrict the grasp search to the deep family, or verify the grip before
   lifting.
2. The cube drifts away on a failed close; a missed grasp should be followed
   by a re-measurement before the next attempt.
3. The chassis lateral axis has a ~0.3 s deadband and ~10 % step-to-step
   scatter; plan the centring in 2–3 verified pulses.
4. A local hand-eye re-calibration over the low/centring poses would make the
   centring gate feasible; the current table-down-only calibration does not
   extrapolate there.

## 5. Files

| File | Content |
|---|---|
| `a3_measure_view.jpg` | first data collection (gate: 65 %/56 % off centre) |
| `a3_grip.jpg`, `a3_lift.jpg` | failed grasp 3a at the grasp and at the lift (no wood at the expected window) |
| `a3b_grip.jpg`, `a3b_lift15.jpg` | retry: grasp pose and the 15 mm grip check |
| `a3b_lift150.jpg` | retry at 150 mm — cube visible between the fingers |
| `a3_final_adopted.jpg` | after release, from the adopted pose |
| `seg_a3r2.json` | final measurement used for grasp 3b (clipped top face, 34.8 mm) |
| `seg_a3_lift.json` | lift-pose segmentation (0 clusters — the held cube is inside the 0.176 m cutoff) |
| `grasp3.json`, `grasp3b.json`, `grasp3c.json` | reach searches at the three cube positions |
| `center_view.py`, `center_validated.py` | centring pose searches and the distance constraint |
| `deep_fit.py` | fit of the proven deep family to the measured cube |
| `lift3.py`, `lift3b.py`, `lift_view.py` | one-step lift targets and held-cube projections |
| `jac.py` | image Jacobian used for the centring corrections |
| `capture_cloud.py`, `proj_lift.py` | raw-cloud range check and projection checks |
