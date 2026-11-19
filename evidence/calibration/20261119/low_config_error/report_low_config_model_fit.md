| Model | Params | Residual RMS (mm) | Residual max (mm) | LOO RMS (mm) | LOO max (mm) | Implied TCP correction (mm) |
|---|---:|---:|---:|---:|---:|---|
| `none` | 0 | 6.096 | 12.379 | 6.774 | 13.755 | (+0.0, +0.0, +0.0) |
| `link4_translation` | 3 | 2.280 | 3.565 | 2.740 | 4.183 | (+13.0, +0.8, +9.0) |
| `link4_se3` | 6 | 1.438 | 2.598 | 1.833 | 3.249 | (+4.1, +0.7, +7.4) |
| `joint_offsets4` | 4 | 1.663 | 2.766 | 2.278 | 4.062 | (-0.0, -1.5, +7.3) |
