# Integration validation — 2026-10-04

Base: khan-moazzin/FTC-2027BIOBUZZ main at d54f419. A final fetch found no newer upstream commits. Work is isolated on codex/solverslib-biobuzz-integration.

Verified locally:

- TeamCode assembleDebug: PASS, including the final turret startup change.
- TeamCode testDebugUnitTest: 21 tests, zero failures, zero errors.
- TeamCode lintDebug: zero reported errors, 17 warnings. Lint also emitted a Kotlin 2.4 metadata / 2.2 reader incompatibility diagnostic; dependency analysis should not be treated as complete.
- git diff --check: no whitespace errors.

Tests exercise field/CAD fixtures, camera transforms, red/blue HIVE independence, synthetic 3D pose recovery, capture-time interpolation, covariance/history reset and bounds, delayed corrections, out-of-order rejection, dual encoder calibration, full-turn ambiguity, flywheel dwell logic, motion lead and angular feedforward, shot coverage, calibration rank checks, export completeness, and synthetic camera/COR/latency recovery.

Build environment: Android Studio bundled JBR 25, Gradle 9.1.0, AGP 8.13.2. Java 8 source/target deprecation and Gradle 10 compatibility warnings remain in the pinned upstream toolchain. The environment returned HTTP 403 from the ordinary Maven Central endpoints; a local Gradle init script added Google's Maven Central mirror for verification. That machine-specific workaround is not a project dependency change.

No connected robot or field was available. Encoder wiring/direction, safe mechanical travel, camera pipeline/tag size, odometry scale, physical field survey, launch geometry, controller gains and shot trials must be measured using the calibration workflow. The committed configuration flags remain uncalibrated. Loop timing is instrumented; no robot-loop benchmark or match readiness is claimed.

The vision wizard automates an observable extrinsic/COR/latency fit and noise estimate with held-out checks. It cannot infer a physical forward mark, pivot height, projectile success or flight time from the available tag measurements. Those guided inputs remain explicit. The flywheel tuner's P gain is a starting value derived from measured feedforward, and requires loaded-response validation.

See [operating and calibration instructions](ROBOT_INTEGRATION.md) and the [Seattle DECODE review](SEATTLE_DECODE_REVIEW.md).


## Complete Seattle audit follow-up — 2026-10-05

- Audited Seattle commit `02a453104b0b208a885571b5fa93b0465c054b78`: 272 text files, 38,845 lines, plus nine inventoried binary files. Full-source/verified-diff coverage and hashes are recorded in `SEATTLE_AUDIT_COVERAGE.json`.
- Added Pinpoint status/finite-velocity gates with explicit recovery, voltage-based flywheel characterization/control, readiness reset on stop, and bounded stage/period CSV profiling.
- `:TeamCode:assembleDebug`: PASS with the final production changes.
- `:TeamCode:testDebugUnitTest`: **27 tests, zero failures/errors/skips**. Additional regressions cover odometry fault/recovery, invalid velocity, voltage polling/freshness/compensation, chronological bounded profiling including telemetry gaps, and finite-difference angular feedforward across all quadrants.
- `:TeamCode:lintDebug`: completed, **zero reported errors, 18 warnings**. One additional warning is the already-present local `Robot.java` KITKAT annotation, which was preserved and excluded from these commits. The Kotlin metadata compatibility diagnostics described above remain; lint dependency analysis is limited.
- `git diff --check`: PASS. A fresh fetch showed no new origin/main commit beyond `d54f419`.

Build logs: `build/seattle-audit-validation.log` and `build/seattle-audit-final-checks.log` (local generated files, not committed). Source/API checks confirmed Pedro 3 performs the Pinpoint read and SDK 12 `getDeviceStatus()` reads the cached result without another I2C transaction. The race-command cleanup issue was also verified against the actual cached SolversLib 0.3.6 binary; current production code does not use that composition.

Hardware validation is still required. In particular, test Pinpoint disconnect/recovery and robot-relative fallback; rerun Tune 3 before enabling voltage-based gains, verify loaded flywheel recovery with Tune 5, and collect the loop CSV with the camera and drivetrain active. No physical tuning values or on-robot performance results were invented.


## Source simplification and defect fixes — 2026-10-05

Debug build and all 32 unit tests pass after migrating support packages under `lib`. Package/path and import checks pass. Lint reports no errors and 14 warnings (12 dependency native-alignment warnings and two unused SDK resource placeholders); no team Java warnings remain. The Kotlin metadata reader limitation remains as described above. See [CODE_REVIEW_FIXES.md](CODE_REVIEW_FIXES.md) for corrected behavior and [CODE_STRUCTURE.md](CODE_STRUCTURE.md) for the new editing layout. Hardware calibration flags remain unchanged/uncalibrated in the committed configuration.

## AdvantageScope logging — 2026-10-05

Added per-cycle full-robot snapshots, bounded asynchronous CSV writing, live FTC Dashboard packets, unique run files, config/completion JSON, per-HIVE vision diagnostics and explicit shot-readiness blockers. Loop timing remains available in the full log and the legacy export. See [logging setup and field semantics](LOGGING.md).

- `:TeamCode:testDebugUnitTest`: 40 tests, zero failures/errors/skips (8 new logging tests).
- `:TeamCode:assembleDebug`: passed.
- `:TeamCode:lintDebug`: completed, zero reported errors and 16 warnings: 12 native-library alignment warnings, 2 SDK placeholder resources and 2 suggestions to use Android allocatable-space APIs. Logging deliberately checks current usable bytes and does not reclaim other cached storage. The existing Kotlin dependency-metadata mismatch still limits dependency lint analysis.
- Synthetic CSV successfully decoded with the actual upstream AdvantageScope `CSVDecoder` pinned at `abb616bdd575dfa27f5794ed6be427ffd0209897`, using a stub log receiver. Values, timestamps, field names and escaped messages round-trip. This validates parser compatibility, not the desktop UI.
- Hardware/GUI checks remaining: connect live, inspect a real recorded run in AdvantageScope, compare loaded loop periods with logging on/off, check free-space/size-limit reporting, and confirm shutdown completion on the Robot Controller. No measured overhead or field performance is claimed.

## Merge of confirmed main hardware — 2026-10-05

- Integrated main through `bea78b3`; details in [MAIN_INTEGRATION.md](MAIN_INTEGRATION.md).
- `:TeamCode:testDebugUnitTest`: 45 tests, zero failures/errors/skips, including single-motor readiness, reverse/feed arbitration, optional stationary-shot gating, hood travel settling and incomplete trial-data checks.
- `:TeamCode:assembleDebug`: passed.
- `:TeamCode:lintDebug`: completed, zero reported errors; existing dependency/resource/usable-space warnings and Kotlin metadata limitation remain.
- `git diff --check`: passed. No unresolved Git conflicts remain.
- Physical calibration remains required, including new Tune 6 indexer endpoints before Tune 5 feeding. Preserved trial data lacks height and flight time and does not enable the predictive shot map.

## Stable-HIVE physics update (2026-10-05)

- 55 desktop unit tests passed, zero failures/errors/skips. New checks cover analytic gravity, numerical drag, tilted aperture crossings, distinct-frame stability dwell, loss of frames, independent stability gates, synthetic parameter recovery, held-out calibration rejection, bounded stable-target solutions and invalidated model rejection.
- Debug APK assembly passed. Lint completed with 0 errors and the existing 16 warnings; the local Kotlin metadata compatibility diagnostics remain in its output. This does not establish hardware timing or readiness.
- Offline synthetic demo: 99 search samples and 357 validation points; maximum nominal model miss 0.02245 m. These numbers describe synthetic inputs only. Demo exports remain disabled. The normal generator correctly refuses the current uncalibrated robot configuration.
- The first positive trajectory fixture was rejected under its initial uncertainty settings; the fixture now explicitly models a synthetic repeatable launcher. Production uncertainty defaults were not reduced to make the test pass.
- Competition shooting now requires stable-HIVE evidence and a generated physics model. Prior sections describing active moving-HIVE prediction/map coverage are historical. See [Physics shooting](PHYSICS_SHOOTING.md) for measurement requirements and limitations.

## Empirical-map shooting restoration (2026-10-06)

- Independent distance-to-hood-degree and distance-to-flywheel-percent maps are again the primary competition setpoints. Each map interpolates and endpoint-clamps independently.
- A valid physics model can only add a motion-dependent correction bounded by the active empirical zone; missing or invalid physics falls back to the calibrated maps. Stable-HIVE and readiness gates remain in force.
- Chassis aim assist was removed. G1 right-stick X is always the driver's drivetrain-yaw command; turret tracking remains independent.
- `:TeamCode:assembleDebug` passed and all 57 unit tests passed with zero failures/errors/skips. `:TeamCode:lintDebug` completed with zero errors and the existing 16 warnings; Kotlin dependency-metadata diagnostics still limit dependency analysis. Physical defaults remain uncalibrated, so this does not establish shot accuracy or robot readiness.

## Pedro competition autonomous (2026-10-06)

- Added the ten supplied Pedro Path Generator segments as one continuous blue route with tested
  red mirroring, reusable intake/shoot actions, a mode factory and loop-driven `AutoMain`.
- Autonomous shooting uses the normal readiness gate and bounded 4.0/4.5-second windows. The route
  never uses Ivy, a blocking loop, a background hardware thread or a second Follower.
- Debug assembly and all 59 unit tests passed. Path continuity, finite endpoints and red/blue
  symmetry are covered on desktop. Actual completion time, path tracking, intake timing and shot
  success still require the tuned robot; the requested shooting windows alone total 16.5 seconds.
