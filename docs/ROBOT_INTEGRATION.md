# Operating and calibrating the integrated robot

This branch starts at upstream d54f419 and uses SolversLib 0.3.6, its Pedro command adapter, Pedro 3.0.0, REV Hub 3.0.0, AutoTune 1.0.0, and FTC SDK 12.0.0. Ivy and the old standalone webcam/static-botpose vision implementations are removed.

## Runtime structure

Robot owns devices and one lifecycle. Each loop clears hub caches, reads turret/flywheel/Pinpoint, processes new vision frames, runs Solvers commands, then updates Pedro and writes mechanism outputs. BufferedFusionLocalizer.update is intentionally a no-op; Drive.read performs the one hardware update. Standalone Pedro tools use a raw Pinpoint localizer.

Packages:
- config: VisionConfig, MechanismConfig, ShotConfig; Constants holds Pedro setup and alliance.
- field/math: official geometry, rigid camera transforms, numerical fitting.
- vision: Limelight adapter, capture-time turret history, HIVE estimators, pose fitting.
- localization: bounded delayed pose filter with covariance and innovation rejection.
- control/subsystems: measured mechanism state, aiming, shot interpolation, readiness and command ownership.
- calibration: separate guided OpModes and complete Java configuration export.

The vision estimator uses per-tag 3D camera-space translations, explicitly converts optical (right, down, forward) to (forward, left, up), and fits robot XY/yaw and one HIVE angle. It requires at least two separated tag IDs from the same HIVE. It rejects degenerate fits; it does not silently fall back to static field poses. Camera roll/pitch/yaw and turret COR are part of the transform.

Red and blue have separate angle, angular velocity, covariance, and freshness. Covariance grows with range, camera/turret motion, HIVE angular motion during exposure, and time since capture. Tag correlation is retained conservatively. The raw robot model assumes a level floor: do not interpret this as a full robot roll/pitch estimator.

Fusion uses capture time, full pose covariance, a Mahalanobis innovation gate, and replay of odometry after correction. It is bounded to two seconds / 512 entries including inserted corrections. Frames older than the latest accepted correction are rejected so they cannot erase that correction. Reset clears history, covariance and time state. Fresh duplicate camera frames are not fused twice.

Moving-shot aiming uses Seattle's iterative virtual-goal approach, extended with cell height and HIVE motion. Robot angular velocity around the turret and turret velocity around an offset muzzle affect launch velocity. Relative line-of-sight feedforward has the corrected sign. Flight interpolation is restricted to measured distance/height triangles. Prediction assumes constant velocities over flight and clamps HIVE angle at the mechanical endpoints; uncertainty can inhibit shooting.

## Driver controls

| Controller | Control | Action |
|---|---|---|
| G1 | left stick / right stick X | translation / yaw; manual yaw wins over assistance |
| G1 | B | robot/field drive toggle |
| G1 | Back | driver-heading trim |
| G1 | LT / LB | collect / reverse; reverse wins |
| G1 | held RB | chassis aiming assistance when a valid prepared solution exists |
| G2 | LT | prepare and track |
| G2 | RT | prepare and feed only when all readiness conditions pass |
| G2 | B | cancel, latched until triggers are released |
| G2 | A / X / Y | automatic raised cell / audience cell / scoring cell |
| G1 during INIT | dpad left/right | blue/red alliance |

Preparation owns the shared intake path. An intake trigger cannot bypass the shooting gate while preparing. Releasing preparation restores ordinary intake control. There is no separate feeder hardware specified in this robot; gated feed uses the two intake motors.

At startup, put the turret away from the shared endpoints of a full revolution (forward is suitable). A 1:1 absolute encoder cannot distinguish the two travel endpoints there; automatic turret output remains inhibited until its position is unambiguous.

Both flywheel RPMs must individually meet tolerance continuously for the dwell time. A changing target inside the tolerance does not needlessly restart the dwell. Any invalid/zero target, wheel error, stale HIVE, bad turret feedback, unreachable target, uncalibrated mechanism, hood travel violation, or poor pose certainty inhibits feed. Large target changes reset readiness.

## Calibration workflow

Generated files are in /sdcard/FIRST/biobuzz-calibration on the Control Hub. Copy exported source into the config package and rebuild. Exports include every public configuration field; no manual merging of isolated vision fragments is required. Save a copy of each previous export before another run.

1. **Tune 0 â€” Drive and Pinpoint verification.** Verify axis signs with limited power and surveyed distances. Run registered Pedro AutoTune Mecanum, Pinpoint CUSTOM for SWYFT pods, then Foresight. Copy the resulting Pedro configuration into Constants. Those hardware values remain marked TUNER until measured.
2. **Tune 1 â€” Turret feedback and lag.** Physically mark forward, record center, move CCW to identify each analog sign, record both safe software limits, then sweep back and forth. The regression fits PWM angle scale and angular-velocity lag. The two encoders are read directly at 1:1; commanded servo position is never substituted for feedback. Use the Axon programmer to establish correct position mode and compatible travel before sweeping. The tuner rejects a negative PWM-to-CCW mapping rather than quietly reversing a coupled mechanism.
3. **Tune 2 â€” Complete BIOBUZZ Vision Wizard.** Requires the copied turret export. Enter surveyed robot XY/heading, known red/blue HIVE angles, and physically measured turret-axis height. At each static robot/HIVE placement record slow bidirectional turret sweeps. Repeat at three or more robot positions/headings and both HIVE endpoints. The fit estimates COR XY, camera XYZ, roll/pitch/yaw and residual latency. COR Z is fixed by physical measurement to remove an otherwise unobservable vertical-offset ambiguity. A subset of observations is held out for validation. Rank-deficient or high-error fits do not enable calibration. Observations are exported to CSV for review. One complete VisionConfig.java is exported.
4. **Tune 3 â€” Flywheel feedforward.** Empty shooter, hold RB through staged characterization. It fits each wheel's static and velocity coefficients separately. The proportional coefficient is an initial value derived from measured velocity gain, not a measured optimal controller. Validate settling and loaded recovery in Tune 5 before competition.
5. **Tune 4 â€” Hood angle mapping.** Record two well-separated servo positions and physically measured launch angles. Adjust the safe hood limits in MechanismConfig to the actual mechanism before operating outside them. This is a linear mapping; validate intermediate positions if the linkage is nonlinear.
6. **Tune 5 â€” Shooter flight map.** Record successful shots across distance and height. Enter measured flight time (video is appropriate), RPM and hood angle. Three non-collinear samples are the minimum for interpolation, not sufficient coverage for a full field. Measure turretToMuzzle and transferDelay in ShotConfig; they cannot be recovered from tag data. The tuner exports the complete shot table.

Vision noise thresholds, exposure duration, HIVE acceleration process noise, readiness tolerances and gate thresholds are initial policy values, not measured robot constants. Validate them against field motion and replay logs. If impact-time HIVE uncertainty exceeds maxHiveSigma, feeding remains inhibited even with a good current pose. The wizard does not claim to measure projectile flight, camera exposure settings, or all future HIVE dynamics from static tag captures.

The tuning OpModes require operator acknowledgement and physical measurements where the sensors cannot observe the quantity. Robot movement, field surveying, physical stop discovery and projectile success cannot be automated on the desktop.

## Verification and remaining hardware work

Desktop tests cover 3D transform inversion, field fixtures, synthetic pose/HIVE recovery, dual-encoder zero/sign/wrap, bounded delayed fusion and resets, old-frame rejection, feedforward direction, shot-map extrapolation, calibration observability, source export completeness, and synthetic recovery of camera geometry/latency.

Run :TeamCode:assembleDebug and :TeamCode:testDebugUnitTest. Loop telemetry reports rolling p50/p95/max work time and start-to-start period; no on-robot performance numbers are claimed. Telemetry updates at 10 Hz. Measure loaded loop timing with the camera before increasing pipeline rate.

No robot is connected during this implementation. Physical calibration flags remain false in committed defaults. No claimed match readiness, physical accuracy, or successful shot is inferred from compiling or simulation.


## Seattle audit follow-up

Pinpoint must report READY and finite pose/velocity after its normal Pedro bulk update. Startup calibration may finish normally. A later device fault invalidates the fusion history and latches automation off until a verified `Drive.setPose(...)` or OpMode restart. Manual control falls back to robot-relative drive; shooting, yaw assist and path/hold control are blocked. The back-button driver-forward trim does not reset this fault. Diagnose the device and re-establish field pose before continuing automation. A 250 ms freshness deadline also rejects an old state; no unchanged-position heuristic is used because a stationary robot is valid.

Tune 3 now records applied **volts**, not duty cycle, and exports `MechanismConfig.flywheelGainsInVolts=true`. Rerun the tuner to migrate existing gains; never just flip this flag. S has units volts, V has volts/RPM, and P has volts/RPM of error. Controller output is divided by cached measured battery voltage and clamped to [0,1]. Legacy duty-cycle configurations retain `false`. Voltage is read at 5 Hz; invalid or stale voltage inhibits voltage-mode output and readiness. The P estimate remains only a starting point for Tune 5 loaded testing.

`TeleopMain` saves the latest 512 completed loops to `/sdcard/FIRST/biobuzz-logs/teleop-loop.csv` after shutdown. Each row contains read, command, write and total work milliseconds. Start-to-start period also includes telemetry and SDK scheduling time. The final period is blank because there is no next loop. The file replaces the previous run; copy it before another run. Export is best effort within the SDK stop lifecycle, so inspect the Robot Controller log if the file is missing. No background exporter survives into the next OpMode.

SolversLib 0.3.6 `ParallelRaceGroup` omits normal cleanup of a finished child. Do not introduce `withTimeout`/race compositions around hardware-owning commands without a tested lifecycle fix. Current production code uses neither. Keep explicit `cancelAll`, mechanism stops and then scheduler reset. Raw-mode motor `set(0)` also preserves SolversLib's write cache across a same-power restart.


## Simplified source layout

Editable code stays in `Robot.java`, `Constants.java`, `config/`, `subsystems/` and `opmodes/`. Reusable support moved into `lib/`, including Pedro tuning infrastructure. See [CODE_STRUCTURE.md](CODE_STRUCTURE.md). Driver Station names, configuration class packages, hardware names and exported configuration filenames are unchanged.

Vision commits a HIVE update only after the matching robot-pose correction passes its innovation gate. Hood readiness starts on an output write and resets after cumulative target movement. Shot-map tuning rejects requests outside configured RPM/hood travel instead of recording silently clamped values. Changing the turret zero invalidates the prior vision calibration, so rerun the vision wizard after Tune 1. Invalid/empty calibration datasets now return a failed fit rather than crashing.

## Full robot logging

`TeleopMain` now also records uniquely named AdvantageScope-compatible CSV sessions with calibration metadata and completion summaries, and publishes matching live data through FTC Dashboard. This runs alongside the older last-512-cycle timing CSV. See [LOGGING.md](LOGGING.md) for connection steps, field meanings, storage limits and hardware validation.
