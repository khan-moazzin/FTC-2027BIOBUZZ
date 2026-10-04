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
