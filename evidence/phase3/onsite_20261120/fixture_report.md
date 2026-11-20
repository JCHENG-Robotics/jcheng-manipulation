# Object-fixture pose error (2026-11-20)

Cube lifted and re-placed on the new floor mark 20 times; top-face point measured at the fixed view pose after every placement.

| Quantity | Value |
|---|---:|
| Placements | 20 |
| Radial RMS about mean | 5.057 mm |
| Radial max | 14.815 mm |
| Mean position | (+0.2321, -0.0146, -0.0501) m |
| Per-axis SD | (5.112, 0.884, 0.362) mm |
| Per-axis range | (22.990, 3.783, 1.076) mm |

The fixture is a radial yellow tape strip with no x-direction stop; the x scatter is the operator placing the cube at different distances along the strip. The raw capture also holds three placements off the strip (|y - median| > 8 mm); they are stored in discarded_fixture_off_tape.jsonl and are not part of this report.

The measurement uses the same RGB-D segmenter as the grasp trials, so it is a task-level bound and includes the perception noise that the budget already carries separately.
