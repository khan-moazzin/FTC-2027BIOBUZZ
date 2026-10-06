# Seattle Solvers DECODE — complete source audit

> Historical integration record. Competition shooting now uses the [stable-HIVE physics solver](PHYSICS_SHOOTING.md); references below to active moving-target prediction or shot-map aiming describe the earlier implementation.


Reviewed 2026-10-04 at [FTC-23511/Decode-2026, 02a453104b0b208a885571b5fa93b0465c054b78](https://github.com/FTC-23511/Decode-2026/tree/02a453104b0b208a885571b5fa93b0465c054b78).

## Scope and evidence

All **272 tracked text files, 38,845 lines** are covered, including comments, tests, autonomous variants, vendored libraries, SDK scaffolding, build files and documentation. The 61 team-namespace files account for 12,410 lines; the vendored SolversLib accounts for 19,978 lines. Coverage is by full source reading or full verified differences against already reviewed source, as recorded per file in [SEATTLE_AUDIT_COVERAGE.json](SEATTLE_AUDIT_COVERAGE.json). The manifest records SHA-256 hashes and reviewed line counts. Nine binary assets/archives, including a PDF, were inventoried rather than claimed as source-line review. This is an audit of this commit, not other branches, repository history, or untracked files.

Static review does not prove match performance or exhaustively discover every defect. Neither Seattle's robot nor ours was benchmarked on hardware. This report supersedes the earlier selective review.

## Useful techniques and disposition

Paths in the following tables are beneath Seattle's `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` unless a library package is named.

| Source / technique | BioBuzz decision |
| --- | --- |
| `globals/MathFunctions.java`, `VirtualGoalSolver`: iterate flight time and a velocity-offset virtual target | Already adapted in `MovingShotSolver`: predict the actual moving HIVE cell in 3D, including launch delay and launcher tangential velocity. Use measured distance/height shot data; reject extrapolation. |
| `commandbase/subsystems/Turret.java`: angular velocity multiplied by servo response lag | Already adapted in `Turret`: positional Axon servos retain their internal controller; our calibrated dual analog feedback measures the **1:1 turret angle**. Preserve timestamped unwrapping and capture-time camera geometry. |
| `globals/Robot.java`: one shared shot solution per loop | Already integrated: shooter, turret, hood and flywheel consume one coherent solution after acquisition/fusion. |
| `Robot.java`: manual bulk reads and cached voltage | Bulk caching already integrated. Added a corrected monotonic 5 Hz voltage cache and voltage-based flywheel control/characterization. No fabricated 12 V fallback on invalid measurements. |
| `Launcher.java`: empirical launcher/flight-time lookup tables | Already represented by `ShotMap` and guided trials. Both BioBuzz wheels must independently meet tolerance for a dwell period. |
| `dev/nullftc/profiler`: named stages and CSV export | Added bounded read/commands/write/work and start-to-start timing. Last 512 loops export after TeleOp actuator shutdown. Period includes telemetry and SDK scheduling gaps; work measures only our acquisition-through-output portion. |
| OctoQuad driver and drive localizer: status/CRC checking and reusable read blocks | Keep SDK 12 Pinpoint and Pedro 3. Added cached device-status and finite pose/velocity checks around their one read per loop. Bad data cannot become a fresh aiming state. |
| Command requirements/defaults, motor/servo write caching | Already integrated through pinned SolversLib 0.3.6. Continue explicit cancellation and output shutdown before scheduler reset. |
| Camera ROI, decimation, exposure control, disabling live preview | Useful pipeline tuning concepts. Our Limelight processes vision externally; copying their OpenCV processor would add a competing pipeline. Measure image blur, detection quality and end-to-end delay using our vision wizard. |
| Intake current + distance + dwell + startup grace | Useful when the corresponding sensor exists. BioBuzz has no configured intake distance sensor or measured jam-current threshold, so this is not enabled by assumption. |
| Auto-to-TeleOp pose handoff | Useful future addition once a competition autonomous mode exists. Require alliance, age and operator confirmation; never reuse an unqualified static end pose. |
| Rumble on readiness / intake-full state | Prefer one event on each state transition. No new continuous rumble or unmeasured full-intake detector was introduced. |

New runtime behavior: if Pinpoint loses readiness or returns nonfinite data after a healthy sample, fusion clears its history, exposes zero velocity and latches the fault. Automated feeding, aiming assistance and path/hold control are inhibited. Manual driving becomes robot-relative. A verified `setPose` or OpMode restart is required before automation can recover; changing the field-forward trim does not clear the fault. This avoids treating a potentially reset device coordinate frame as continuous motion. Freshness has a 250 ms control deadline. It does not detect every possible frozen-but-plausible sensor failure.

Flywheel gains retain explicit units: existing configurations use duty-cycle gains (`flywheelGainsInVolts=false`). Tune 3 now fits applied volts (`power * measured battery voltage`) versus each wheel's RPM and exports `true`, with S in volts, V in volts/RPM and P in volts/RPM error. Do not flip the flag on old coefficients. Invalid/stale voltage inhibits voltage-mode readiness and output. The fitted P starting value still requires loaded-response verification in Tune 5.

## Team-code findings that affect a port

| Source | Finding and consequence |
| --- | --- |
| `MathFunctions.VirtualGoalSolver` / aim math | Their line-of-sight angular feedforward sign is opposite the derivative of target-relative `atan2`. At target (100,0), robot velocity (0,10) requires about -0.1 rad/s. BioBuzz uses relative target-minus-launcher velocity, subtracts chassis yaw rate, and tests the sign. |
| `Robot.getVoltage` | After the first polling interval the timer is never reset; subsequent calls read hardware every loop. Their invalid-voltage fallback hides missing data. Our cache resets every poll and invalidates bad readings. |
| `Robot.exportProfiler` | A daemon thread accesses shared profiler state; the next run can replace that state before export finishes. Our bounded, same-thread export runs after shutdown. |
| `Robot` initialization | Singleton/static state, hard-coded hub assumptions and PhotonCore parallel-command settings are coupled to their robot. They are not portable performance guarantees. |
| `Turret` | The getter for target refers to a PID setpoint that is not the actual servo target; raw angle subtraction also needs seam handling. Use measured feedback, reachable equivalents and explicit overflow. |
| `Launcher`, `ClearLaunch` | Dynamic hood compensation during flywheel acceleration depends on their measured projectile model. The non-precise launch path can feed without readiness. Neither behavior should bypass our dual-wheel gate. Their hood-angle convention also differs from ours. |
| `Launcher.periodic` | The voltage-scaling lines are commented out; their presence is not evidence of active compensation. The commented `battery / nominal` multiplier is not a replacement for dividing requested volts by measured supply voltage. |
| `Robot.getShotSolution` | Driver-intent velocity blending is not a physical velocity measurement. Keep measured velocity in the moving-target model unless a predictive blend is experimentally justified. |
| `Intake` | Startup grace and current/distance dwell are useful, but state updates are conditional on mode/direction; a cached full flag can become stale in TeleOp. |
| Drive / `DriveTo` | Failed OctoQuad reads can leave old pose/velocity available without a freshness contract. DriveTo saves old settings at construction rather than initialization, compares an integration override against the wrong reference, and does not explicitly stop on end. |
| Autonomous routines | Several repeat counts are captured when commands are constructed, before init-loop edits. A profiler end is called without a matching start. Static end poses lack age/alliance validity. `Pose2d.mirror` DOES mutate, so the foreach mirror calls are valid. |
| `SetIntake`, command construction | Waiting for artifacts can be unbounded unless a parent supplies a timeout. Hardware actions need requirements propagated through their groups. |
| TeleOp / event telemetry | Repeated rumble/stop-rumble calls and persistent static zone overrides are avoidable. Preserve XY when changing driver-forward heading. |
| `Camera` | Default camera construction is disabled in the inspected robot setup. Repeated `getDetections` calls can reuse the same frame; averaging three poses and assigning the newest timestamp misrepresents capture time. Linear heading averaging fails across the angle seam. |
| Camera history/fusion | Inserting the camera pose into the odometry history before looking up that timestamp can compare the observation with itself. Fixed scalar correction is not a covariance filter; translation-only replay omits heading effects. BioBuzz retains bounded delayed SE(2) replay and independent red/blue HIVE states. |
| Custom AprilTag processor | ROI principal-point adjustment is useful, but native temporary objects/cleanup, ignored solvePnP success, and orientation conventions need care. Do not assume its outputs share Limelight coordinates. |
| Legacy math | `Double.MIN_VALUE` used as a maximum initializer fails for all-negative coordinates. An older launcher conversion uses a drivetrain maximum velocity for a projectile limit. Avoid unused legacy alternatives. |
| Tuners / tests | TransferMotorTuner can flip the sign every loop; SingleTurretServo's timer does not reset. Some tests only check positive examples or finite output. Explicit tuner periodic calls are not automatically duplicate updates: their scheduler conditionally suppresses them, which is itself undesirable team/library coupling. |

## Vendored library and infrastructure review

Seattle vendors a modified SolversLib, PhotonCore, drivers and profiler. These findings describe the inspected copies unless explicitly verified against our dependency. We do not import the whole tree.

- **Command lifecycle:** the vendored scheduler imports team globals/profiling; `reset` drops the singleton without ending active commands. Callback/Uninterruptible wrappers separately schedule children without dependable cancellation propagation; Lambda conversion loses requirements; Deferred relies on caller-declared requirements; Retry can end a child twice; Select evaluates a selector twice. Perpetual does not restart a completed sequence. Follow explicit ownership and cancellation rules.
- **Verified in published SolversLib 0.3.6 too:** `ParallelRaceGroup.end` never calls `end(false)` on a finished winner. A `.withTimeout(...)` composition can therefore omit normal cleanup. Current BioBuzz production code does not use race/timeout wrappers. Do not add them around output-owning commands without fixing/testing that lifecycle. `Motor.stopMotor()` bypasses cached `lastPower`; our mechanisms already stop with raw-mode `set(0)` so a same-power restart remains writable. The flywheel's own readiness dwell now also resets on stop.
- **Controllers:** cascade error derivatives are not measured mechanism velocity; output limits still need application-level enforcement. Do not copy gains or assume profile clocks are interchangeable. A mecanum trajectory command mixes frame assumptions and lacks complete drivetrain lifecycle ownership. Pedro remains the sole drivetrain controller.
- **Input/geometry:** ToggleButtonReader can toggle on repeated reads in one edge cycle; slew-filter getters also mutate state. Snapshot inputs once if adopting those APIs. SDK Pose2d conversions mishandle degree arguments in the vendored copy; always make units explicit. Nonfinite angles can trap a while-loop normalizer.
- **Analog/motor feedback:** wrapped-angle subtraction creates false velocity spikes at the seam. Corrected motor velocity depends on position sampling/sign assumptions and can preserve stale estimates. MotorGroup's leader feedback does not verify both flywheels. Servo command position is not measured position. Some CRServo optimized modes interpret `set(0)` as angle zero rather than zero power; our positional turret does not use that mode.
- **Swerve/kinematics:** wheel reversal/cosine compensation are swerve-specific. An ignored immutable `Vector2d.scale` return defeats one desaturation path. Mecanum wheel normalization uses the signed maximum instead of maximum absolute magnitude; odometry-wheel normalization ignores the center wheel. These are reasons to retain Pedro, not to add a second drive stack.
- **Pure pursuit/P2P:** blocking follow loops, timeout/reset behavior, heading/seam conventions, zero-distance divisions and profile time units need corrections. `PathMotionProfile` converts nanoseconds in the wrong direction. Spline parameterization has a useful iteration cap; trajectory interpolation still needs finite/degenerate-input guards. None replaces our existing Pedro implementation.
- **Utilities:** Debouncer documents seconds but compares nanoseconds with `debounce * 1e6` (1,000 times too short). GenericDebouncer resets its timer when input is equal rather than changed. Timing.Rate resets on every query, so fast polling can prevent it from ever reaching its interval. BioBuzz's readiness and voltage cache use explicit nanosecond intervals.
- **Custom drivers:** reuse buffers and check status/CRC, but avoid unchecked narrowing, duplicated bulk-scope lengths and unit conversion mistakes. OctoQuad's CRC helper indexes modulo 256 (not an active problem for its short current packets). Keep SDK 12 drivers; do not assume a defect in Seattle's copy is present in SDK 12.
- **PhotonCore:** reflective replacement of SDK devices, immediate acknowledgements, unbounded busy-waits and hard-coded hub state are not a portable optimization. Sensor wrappers can return cached data without health/age contracts. Retain standard SDK bulk caching and measure the result.
- **Profiler:** wall-clock millisecond timing and an unbounded entry list lose precision and grow through a run. The adaptation uses monotonic nanoseconds, bounded storage, rolling percentiles and no per-cycle file writes.
- **Build/SDK/resources:** their SDK 11.1 / Gradle 8.9 / AGP 8.7 scaffolding should not replace our SDK 12 / Pedro 3 build. Hardware XML is specific to their swerve/OctoQuad robot. Their CI test workflow is useful as a future model; their SDK activity/resources/release notes contain no additional BioBuzz control algorithm to transplant.

## Validation and remaining physical work

See [VALIDATION.md](VALIDATION.md) for actual build/test results. New regression coverage checks health loss/recovery, nonfinite velocity, voltage polling after the first interval, bad/stale voltage, supply compensation and bounded chronological timing export. Existing tests cover 3D fitting, red/blue separation, turret seams, delayed fusion and aiming.

On the robot, verify Pinpoint fault/recovery and robot-relative fallback, rerun Tune 3 then loaded Tune 5, and compare loop CSVs under identical driving/camera conditions. Tune gains, turret response lag, camera exposure/COR, shot maps and SWYFT scale using our hardware. No Seattle tuning constants or measured loop-time claims were adopted.

## Attribution

The existing aiming/turret adaptation credits Seattle's virtual-target and positional velocity-lead techniques in source. Their repository's FIRST BSD-3-Clause-Clear notice is retained in [docs/licenses/Seattle-DECODE-LICENSE.txt](licenses/Seattle-DECODE-LICENSE.txt). The additional health, voltage and profiler code is our implementation of the reviewed techniques and corrections. SolversLib remains a pinned dependency, not a copied fork.
