# Controller tuning and validation

Tested October 17, 2026 in the Windows AutoDRIVE IROS 2026 practice simulator using Direct3D 11 and ROS 2 Humble in Docker. The provided bridge is unchanged.

## Measured progression

| Lookahead | Base throttle | Three consecutive flying laps (seconds) |
|---|---|---|
| 0.50 m | 0.15 | 24.3073, 23.8755, 24.1753 |
| 0.65 m | 0.15 | 16.6295, 16.8654, 16.8524 |
| 0.80 m | 0.15 | 15.2315, 14.9344, 15.1285 |
| **0.80 m** | **0.18** | **12.3093, 12.1873, 12.2903** (submission recording) |

All listed laps had zero recorded collisions. The submitted controller's three-lap median is 12.2903s, approximately 49.2% below the original three-lap median of 24.1753s.

## Why it improved

The original controller reduced throttle for front clearance during about 82% of a typical lap and nearly stopped at tight corners. Increasing lookahead lets steering react earlier to the wall's orientation. This improved the path and corner speed before increasing base throttle from 0.15 to 0.18.

The controller uses median LiDAR returns near +90 and +45 degrees to estimate left-wall angle and perpendicular distance. It projects that distance 0.8m ahead and applies PD steering around a target clearance of 0.8m. Steering demand and front clearance reduce throttle. Throttle is a normalized command, not a speed target.

## Submission evidence

The fresh recording uses the rebuilt, installed controller. Its first four completed laps were 13.8150s (standing start), 12.3093s, 12.1873s, and 12.2903s, all without collisions. The continuous video excerpt contains the three complete flying laps, with no internal cuts or speed changes. The simulator's Autonomous display and collision counter are visible.

The same source previously completed a clean 13-lap validation segment; its fastest clean lap in that segment was **11.9742s**. That historical best is separate from the new recording's lap times. Selected completed-lap evidence is in `validation/clean_laps.csv`.

## Checks and safeguards

The packaged source and the installed source used for recording have identical SHA-256 hashes:

`bfed9d48be11430a015143ed0526376e9014299df15ca1e254c3873b510524dc`

Focused checks passed for wall geometry at -20, 0 and +20 degrees; valid clear-front readings; below-minimum front returns; all-NaN scans; and infinite front readings. Run them in a sourced ROS 2 environment with `python3 tests/check_controller.py` from the repository root.

Stale or invalid scans request zero commands. Finite nonnegative front readings below the sensor minimum also request a stop. The two-ray wall model is approximate at corners, and zero throttle is not instantaneous braking. These results demonstrate the required clean laps on the practice track, not guaranteed collision avoidance on every run or track.
