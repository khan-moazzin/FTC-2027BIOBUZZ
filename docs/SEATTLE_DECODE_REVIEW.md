# Seattle Solvers DECODE review

Reference: [FTC-23511/Decode-2026](https://github.com/FTC-23511/Decode-2026/tree/02a453104b0b208a885571b5fa93b0465c054b78), commit `02a4531`. Reviewed 2026-10-03. This report distinguishes inspected code from measured robot performance; we have not benchmarked their hardware.

| Source | Useful technique | BioBuzz adaptation |
| --- | --- | --- |
| `globals/MathFunctions.java`, `VirtualGoalSolver` | Iteratively shift the target by launch delay and flight time; include tangential velocity from the turret offset | Predict the actual moving HIVE cell in 3D. Keep explicit units and an invalid-solution result. |
| `commandbase/subsystems/Turret.java` | Position servo target plus angular velocity times response lag | Use our two analog sensors at 1:1 ratio, calibrate lag, and retain internal servo control. |
| `globals/Robot.java`, `getShotSolution` | Compute one solution shared by shooter and turret per loop | Immutable, timestamped snapshot after sensor acquisition and localization. |
| `globals/Robot.java` | Manual Lynx bulk caches, cached voltage reads | One cache clear before acquisition; reset the voltage poll timer after each refresh. |
| Hardware initialization | Motor/servo write caching | Use SolversLib wrappers, with explicit zero writes on shutdown. |
| Profiler integration | Named execution stages and exported CSV | Bounded samples, decimated telemetry, stage and loop percentiles; export outside active control. |
| `Launcher.java` | Interpolated wheel-speed and flight-time characterization | Guided trials for our hardware, target height and game piece. Do not copy DECODE constants. |

## Corrections needed when porting

- `FullAim` is deprecated. Port the virtual-goal solver rather than this command.
- Their line-of-sight feedforward expression has the opposite sign from the derivative of `atan2(goalY-robotY, goalX-robotX)`: target (100,0), robot velocity (0,10) produces approximately -0.1 rad/s, while their expression returns +0.1. Use relative target-minus-launcher velocity and test finite differences.
- The voltage polling timer is not reset after refresh, so throttling ceases after its first interval.
- Their fixed DECODE goal/backboard and one-dimensional shot map cannot represent a tilting BIOBUZZ target. Reject shots outside calibrated height/distance coverage.
- Do not copy their command-position readiness check as measured turret feedback. Use both calibrated analog channels and time-aligned history.
- Keep swerve-specific controls, hard-coded hub assumptions, custom SDK copies, and PhotonCore concurrency out of our mecanum integration.
- Keep telemetry and logs out of tight measurement loops; runtime improvement must be verified with our profiler rather than inferred from their architecture.

## Attribution

The aiming implementation adapts the iterative virtual-target and positional velocity-lead techniques from Seattle Solvers. Their repository ships the FIRST BSD-3-Clause-Clear license; retained in `docs/licenses/Seattle-DECODE-LICENSE.txt`. Pedro-derived fusion code retains its own source header and license. SolversLib is consumed as a pinned dependency, not copied wholesale.
