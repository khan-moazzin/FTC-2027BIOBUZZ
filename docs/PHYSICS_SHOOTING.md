# Stable-HIVE physics shooting

Competition shooting now uses `StableShotSolver`. There is no moving-HIVE shooting mode or empirical-map fallback. `MovingShotSolver` and `ShotMap` remain tested library references; only the line-of-sight feedforward helper is reused by the active solver.

## What comes from 4414

Team 4414 describes offline trajectory search, error-tolerant shot selection, a second-order distance/radial-velocity polynomial and tangential robot-velocity compensation in its [2026 technical binder](https://2026.team4414.com/) and [team explanation](https://www.chiefdelphi.com/t/team-4414-hightide-2026-tech-binder-ripcurrent/519602/116). No public 2026 shooter source was found. This repository contains an original adaptation of that published method, not copied 4414 code or tuning constants.

Our model uses gravity and optional quadratic drag in still air: acceleration is gravity minus `dragPerMeter * |velocity| * velocity`. Zero drag uses analytic trajectories; nonzero drag uses RK4 with a 5 ms step. There is no Magnus lift, rim collision or bounce model. Opening clearance is approximated by a circular plane, with the projectile radius and an extra margin subtracted from its radius.

## Stable target, independent robot motion

Each alliance has its own stability gate. By default, at least three distinct accepted vision frames must span 0.35 s, stay within 2 degrees of an endpoint and below 2 degrees/s. A frame gap above 0.25 s, stale/uncertain observations, motion or switching endpoints resets the evidence. These thresholds are configurable in `PhysicsShotConfig`.

Only the raised cell is eligible. The solver freezes the current estimated target position; it never projects HIVE motion to impact. Stability is observational evidence, not a guarantee the HIVE cannot start moving after release. Vision still tracks moving HIVEs for localization.

Robot translation and rotational muzzle velocity remain compensated. The release point includes measured velocity over transfer delay. Radial velocity enters the polynomial; tangential velocity adjusts the launch vector. `MechanismConfig.stationaryShotsOnly=true` additionally restricts robot motion if desired. Constant robot velocity over release/flight is an approximation requiring validation.

## Calibration and generation

1. Finish the existing vision/turret, hood, flywheel and indexer calibrations. Preserve the confirmed single-motor and servo hardware settings. Measure `ShotConfig.turretToMuzzle` in inches and `transferDelay` in seconds.
2. Run **Tune 5 - Physics shot calibration** with a stationary robot and a stable raised HIVE. X selects a field and the stick adjusts it. Hold RB to prepare; RT requests manual feeding through mechanism readiness. A records a confirmed successful measured shot. This supervised calibration mode does not run the competition vision stability gate.
3. Record at least nine varied shots. Each row is horizontal distance inches, height difference inches, RPM, hood angle radians and flight time seconds. Measure flight time from release to arrival using video; do not substitute a guessed value. Include RPM variation of at least 300 and varied distances/angles. The preserved reference trial lacks measured height and flight time and is insufficient by itself.
4. Y archives samples in `ShotConfig` and fits `PhysicsShotConfig.speedPerRpm` and `launchAngleOffset`. Every third sample is held out; the fit rejects excessive training or validation error. Copy exported configs into `config/`. A failed fit does not enable shooting. This fit assumes a linear RPM-to-exit-speed relation; systematic residuals require investigating the model.
5. Measure opening radius, projectile radius, aperture center offset and outward unit normal. `apertureOffset` is relative to `Field.cell` in the neutral scoring-cell frame, in meters; the audience cell mirrors Y. The normal defaults to neutral +Z as a placeholder. Confirm the physical opening geometry against the field before setting `geometryVerified=true`. Set uncertainty bounds from repeated measurements, not the demo. Drag is a configurable input, not automatically identified by Tune 5.
6. Choose the measured distance/radial-speed domain and physical launch limits, then run `./tools/generate-shot-model.ps1` from PowerShell. It compiles the current configs, derives endpoint target/muzzle height, intersects angle limits with hood travel, searches 99 operating points and validates the fitted model on 357 grid points including held-out midpoints. If rejected, inspect measurements, model assumptions and domain; do not reduce uncertainty merely to make it pass.
7. Inspect `build/generated-physics/report.txt` and `validation.csv`. Copy the generated `PhysicsShotConfig.java` into the source config folder, rebuild and validate stationary shots before robot-motion trials. Changing physical model settings or hood calibration invalidates the generated fingerprint and requires regeneration.

`./tools/generate-shot-model.ps1 -Demo` exercises the pipeline with synthetic inputs and writes to `build/physics-demo`. Its exported calibration/model flags remain false. It must never be treated as robot calibration.

## Runtime checks and limits

Polynomials use six normalized terms: `1, d, v, d², d*v, v²`. They approximate exit speed, angle and flight time within the generated domain. The runtime checks actual tilted-aperture intersections for 27 combinations of speed, elevation and yaw error. Any missing intersection or insufficient clearance rejects the shot. Actual nominal intersection time is logged. The sampled errors are checks at selected points, not a mathematical guarantee over all possible perturbations.

Model generation uses a horizontal-plane training approximation with projected clearance; runtime applies the actual plane check and can reject a generated-domain shot. Near-endpoint height changes must remain within `heightTolerance`. A single quadratic may need a narrower domain to meet validation limits.

The HIVE covariance gate is separate from launch-error checks; localization errors are not fully propagated into trajectory clearance. Nonzero drag also makes runtime checks more expensive. Measure Control Hub loop timings with camera, driving and logging active before competition; no on-robot timing or accuracy claim follows from desktop tests.

AdvantageScope records both stability gates, solver rejection reason, fixed target, trajectory checks, clearance and mechanism readiness. Metadata includes the physical model config (schema 3).
