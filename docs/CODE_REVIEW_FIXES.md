# Code cleanup and fixes — 2026-10-05

The review addressed team-code runtime boundaries, calibration, command/hardware lifecycle, package consistency and build/lint findings. This is not a claim that every possible robot or dependency defect has been proven absent.

## Layout

- Top-level team code now contains only Robot, Constants, config, subsystems, opmodes and lib.
- Reusable control, vision, localization, geometry/math, calibration support and Pedro infrastructure moved into lib. All production/test imports were migrated; package-to-path consistency was checked.
- Guided calibration OpModes moved to opmodes/tuning. Driver Station names and generated config packages stay unchanged.
- The root README is now a short project entry point; the original SDK documentation/release history is preserved in docs/FTC_SDK_README.md.

## Behavior corrected

| Problem | Correction |
| --- | --- |
| A rejected camera pose could still update the HIVE used for aiming | Filter a candidate HIVE state, gate the robot correction, then commit the HIVE update only on acceptance. |
| Hood could appear settled before its first output write or during cumulative small moves | Start settling on output writes and accumulate small setpoint changes against a reference. |
| Unreachable turret targets across the angle seam could select the farther stop | Evaluate reachable equivalents and choose the smallest miss, then shortest travel. Readiness compares the measured reachable angle rather than wrapping a long physical move into a short error. |
| A partial robot or tuner initialization failure could skip cleanup | Run cleanup and retain the original exception, including any cleanup failure as suppressed context. |
| Empty/underdetermined calibration could index an empty Jacobian | Return an invalid fit with infinite RMS; also reject nonfinite initial parameters. |
| Shot tuner could record RPM/hood values different from clamped outputs | Require the requested shot to fit configured RPM and hood limits before spinning, feeding or recording. |
| Turret retuning could preserve an obsolete vision-calibrated flag | Clear that flag when capturing a new turret zero. Initialize the tuner's PWM from the configured position and unwrap sweep samples around the expected physical angle. |
| Vision wizard could collect observations with missing turret history | Require valid capture-time feedback before recording a frame. |
| Flywheel characterization could oversample without a bound on loop rate | Limit sample collection to 50 Hz within the fixed step durations. |
| Future HIVE timestamps and invalid readiness noise/tolerance values could be treated as usable | Reject future freshness, nonfinite/negative process noise and invalid dwell/tolerance values. |
| Intake raw-power control depended on the prior SDK motor mode | Explicitly select RUN_WITHOUT_ENCODER. |
| Driver pose access could depend on follower update order | Read the prepared fused localizer pose directly. |
| Mecanum tuner could energize after STOP during INIT | Check STOP before output and zero power in finally. Add explicit locale to internal strings. |
| Unnecessary local KITKAT annotation generated a warning | Remove the redundant annotation/imports; the project's minimum SDK is already newer. |

## Verification

- Debug APK build passed.
- 32 unit tests passed, zero failures/errors/skips.
- Regression tests cover first-write/cumulative hood settling, independent HIVE candidates, future timestamps, empty fits, turret stop selection and invalid readiness settings, alongside the existing geometry, fusion, calibration, aiming and profiling tests.
- Lint reports zero errors and 14 warnings: 12 native-library alignment warnings from dependencies and two unused SDK resource placeholders. No team Java warnings remain.
- The pinned lint reader still emits Kotlin 2.4-versus-2.2 metadata diagnostics. This limits dependency analysis; changing or suppressing the pinned SDK/toolchain was not used to conceal it.
- No robot was attached. Mechanical limits, encoder wiring, physical calibration, loaded shooter behavior and field performance remain on-robot checks. A lib location means reusable support, not hardware certification.

Local logs: build/restructure-validation.log and build/restructure-final.log. See CODE_STRUCTURE.md for editing guidance.
