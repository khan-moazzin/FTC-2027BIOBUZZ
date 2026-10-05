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
