# BIOBUZZ 2026–27 — FTC Team 26282

Background, decision history, verified API details and environment
troubleshooting live in `docs/PROJECT-NOTES.md` — read it when you need the
reasoning behind a decision or hit an environment problem.

FTC DECODE-successor season code. Mecanum drivebase, Pinpoint odometry, turret
shooter. Written by one student programmer who also codes FRC Team 5817.

## Ground rules

- **Correctness over speed.** This code runs on a competition robot. Verify API
  signatures against source rather than assuming them.
- **Minimal, targeted changes.** No new abstractions unless the task requires it.
- **Never invent tuning numbers.** If a value must come from measurement or a
  tuner, mark it as a placeholder and say so. Do not write a plausible-looking
  constant and leave it unmarked.
- **Push back.** If something is poorly designed, inconsistent, or risky, say so
  directly instead of implementing it silently.

## Build environment

- Android Studio Quail 4 (2026.1.4) or newer. Ladybug **cannot** open this project.
- AGP 8.13.2 / Gradle 9.1.0, FTC SDK 12.0.0, Pedro Pathing 3.0, SolversLib 0.3.6.
- Terminal Gradle needs `JAVA_HOME` pointed at Studio's bundled JBR:
  `/Applications/Android Studio.app/Contents/jbr/Contents/Home`
- `dev.frozenmilk.sinister:Sloth` arrives transitively from Pedro and resolves
  **only** from `https://repo.dairy.foundation/releases/`. That line must stay in
  `build.dependencies.gradle`. Never pin its version by hand.
- No Panels or FtcDashboard in dependencies — no `@Configurable`, no live
  constant tuning, plain FTC telemetry only.
- Decline Android Studio's AGP upgrade prompt and its "migrate to Daemon
  toolchain" prompt. Both break a pinned FTC build.

## Current architecture

See docs/ROBOT_INTEGRATION.md for runtime, controls, units and calibration.
See docs/FIELD_GEOMETRY.md for official geometry provenance.
Historical decisions in docs/PROJECT-NOTES.md and docs/PEDRO_SETUP.md may describe the pre-SolversLib code; the integration guide supersedes them.

SolversLib SubsystemBase and CommandScheduler own requirements and defaults.
Robot reads sensors and fuses observations before commands, then applies outputs.
Drive.read captures localization exactly once. BufferedFusionLocalizer.update is intentionally a no-op for the prepared Pedro state.
Always cancel commands and stop mechanisms before resetting the scheduler.

VisionConfig owns all camera, turret encoder and extrinsic calibration. MechanismConfig owns flywheel/hood settings; ShotConfig owns measured shot data. Constants owns Pedro configuration and alliance.
All angles are radians internally; field distances are inches. Both Axon analog feedback channels measure 1:1 turret position. Never substitute commanded PWM for feedback.
Explicit Axon PWM limits are 500–2500 microseconds; FTC SDK defaults are 600–2400, not 500–2500.
Camera is a turret-mounted Limelight, using per-tag camera-space observations. Moving tags cannot use a static botpose map.
Red and blue HIVE estimates remain independent. No single-tag or uncalibrated fallback may enable automatic feeding.

Hardware names: fl, bl, fr, br, pinpoint, intake1, intake2, turret1, turret2, hood, flywheel1, flywheel2, limelight, turretEncoder1, turretEncoder2.
The last two analog names are configurable in VisionConfig. Verify them on the robot.
SWYFT odometry requires CUSTOM Pinpoint scalar tuning.

Windows JBR: C:/Program Files/Android/Android Studio/jbr.
Build: gradlew.bat :TeamCode:assembleDebug :TeamCode:testDebugUnitTest.
Do not fabricate physical tuning values or report simulation as hardware validation.
