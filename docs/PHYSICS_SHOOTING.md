# Empirical shot maps with bounded physics correction

Competition shooting uses `StableShotSolver`, but the hood and flywheel settings come from measured shot maps. Physics is optional: when its calibration and generated model are valid, it may correct the mapped setpoint for robot motion and check aperture clearance. Missing or invalid physics calibration falls back to the empirical maps; it does not disable a calibrated map shot.

## Primary empirical maps

`ShotConfig` contains two independent maps:

- `hoodMap`: horizontal distance in inches to hood degrees above the calibrated lowest position.
- `flywheelMap`: horizontal distance in inches to flywheel percent from 0 to 100.

The two maps do not need matching distance keys. Each is linearly interpolated between its own adjacent measured points. A request outside a map's measured range clamps to that map's nearest endpoint and is logged as clamped. The flywheel percentage is converted to motor RPM using `MechanismConfig.maxRpm` only after lookup.

Adjacent points form the active empirical distance zone. A physics correction cannot command a hood or flywheel value outside that zone's two measured values. At an exact point, outside the measured range, or with a one-point map, the bounds collapse to the measured value, so physics cannot alter it. This keeps trial-derived settings authoritative.

The committed 48-inch, 10.61-degree, 62.1-percent point preserves the previous confirmed trial, but `ShotConfig.calibrated` remains false. It is not enough to claim a competition-ready map; copy only physically confirmed Tune 5 exports into the source configuration and rebuild.

## Stable target and turret tracking

Each alliance has its own HIVE stability gate. By default, at least three distinct accepted vision frames must span 0.35 seconds, remain within 2 degrees of an endpoint and below 2 degrees per second. Stale or uncertain observations, a large frame gap, motion, or switching endpoints resets the evidence. Only the raised cell is eligible.

The turret aims from the field pose and measured turret feedback. The drivetrain has no chassis aim assist: G1 right-stick X is always the driver's yaw command. Robot translation, chassis rotation and turret rotation are still included in measured muzzle velocity for the optional shot correction.

## Optional physics correction

The physics model is based on an original adaptation of Team 4414's published offline-search and polynomial approach. Gravity and optional quadratic drag are modeled; Magnus lift, rim collision and bounce are not.

When `PhysicsShotConfig` is fully calibrated, geometry-verified, generated, fingerprint-matched and inside its measured distance, height and radial-speed domain, runtime evaluates the polynomial twice at the same distance:

1. once with measured radial muzzle velocity;
2. once with zero radial velocity.

Only the difference is applied to the empirical RPM and hood settings. Tangential velocity adjusts turret yaw. The corrected values remain inside the active empirical zone. The runtime then checks 27 speed/elevation/yaw error combinations against the tilted aperture; insufficient clearance rejects that shot. If the physics model is unavailable, changed, or outside its domain, the uncorrected map shot remains available.

## Tune 5 workflow

1. Complete turret/vision, flywheel, hood and indexer calibration. Measure `ShotConfig.turretToMuzzle` in inches and `transferDelay` in seconds.
2. Run **Tune 5 - Shot map calibration** with a stationary robot and stable raised HIVE. X selects a field and the stick adjusts it. Flywheel is entered as 0–100 percent and hood as degrees above its lowest calibrated position. Hold RB to prepare; RT requests feeding through mechanism readiness. Press A only after a confirmed successful shot.
3. Record successful shots across the distances the robot will use. Press Y to export independent sorted hood and flywheel maps. Duplicate distance entries are replaced by the latest recorded value. Copy the exported `ShotConfig` into `config/`, inspect it, rebuild and revalidate.
4. Height difference and measured flight time are optional for map shooting. To enable physics correction, collect at least nine varied shots with accurate height, flight time, RPM range and hood angle. Tune 5 uses held-out samples to fit speed-per-RPM and launch-angle offset; a failed fit does not invalidate the exported maps.
5. For physics correction, also measure aperture/projectile geometry and uncertainty, set `geometryVerified`, and run `./tools/generate-shot-model.ps1`. Inspect its report and validation CSV before copying the generated config. Changing calibrated physical inputs invalidates the model fingerprint.

`./tools/generate-shot-model.ps1 -Demo` is synthetic pipeline verification only. Its flags remain disabled and its values are not robot calibration.

## Runtime and logging limits

The readiness gate still requires calibrated vision/maps, healthy localization, reachable turret angle, flywheel speed, settled hood, calibrated indexer and allowed robot motion. A valid solution never bypasses those mechanism checks.

AdvantageScope schema 4 records whether a map endpoint was clamped, whether physics correction was active, the final RPM and 0–100 flywheel percentage, stability gates, rejection reason, trajectory-check count and clearance. Desktop tests verify software behavior only; real shot accuracy, uncertainty bounds and loop timing require robot and field trials.
