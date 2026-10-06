# Code — Understand the field. Predict the shot. Understand every decision.

Our software connects a moving camera, a moving HIVE and a moving robot. Three parts of the code make that possible: vision that accounts for changing geometry, empirical shooting maps with bounded motion correction, and logging that connects what the robot saw to what it calculated and commanded.

Suggested format: one code page with three substantial sections. Each section can expand into its own technical page later.

## 1. Advanced vision — A moving camera observing a moving field

### Solving for the robot and the HIVE together

Our Limelight rotates with the turret, while the AprilTags on the HIVE move as the HIVE tilts. Both motions change what the camera sees.

We use a shared field model containing the HIVE pivots, tag placements and scoring-cell geometry. With at least two suitably separated tags from the same HIVE, our solver fits robot X, Y, heading and HIVE tilt together. It checks how well that solution explains the observed tag positions and rejects observations with insufficient geometry or excessive error.

The calculations use three-dimensional tag observations and camera transforms. Robot localization assumes a level floor, estimating planar position and heading.

### Reconstructing the camera at capture time

The camera transform includes its mounting position and orientation, the turret center of rotation and measured turret angle. Our 1:1 turret uses Axon analog feedback to measure that rotation directly.

An image describes the past. We keep timestamped turret angles and odometry so we can reconstruct the camera pose when the image was captured. An accepted vision correction updates that historical robot pose; the system then replays the motion that followed to recover the current pose.

### Knowing how much to trust an observation

Confidence changes with tag geometry, distance, motion and observation age. Our system represents this uncertainty with covariance and checks proposed corrections against the predicted robot state before accepting them.

Red and blue HIVEs have independent angle, angular velocity, uncertainty and update histories. Seeing one does not make the other appear fresh. A HIVE estimate is committed only after its associated robot-pose correction passes the acceptance check, keeping localization and aiming consistent.

A guided vision wizard fits camera mounting geometry, turret center-of-rotation offsets and residual image latency from surveyed placements and turret sweeps. It checks the fit against held-out observations and exports a complete vision configuration.

**Suggested visual:** a robot–turret–camera–HIVE geometry diagram, paired with a capture-to-arrival timeline.

## 2. Advanced shot calculation — Measured maps for a stable HIVE

### Make successful trials authoritative

Our primary shot settings come from two independent trial-derived maps. One maps horizontal distance to hood degrees; the other maps distance to a 0–100 flywheel percentage. Their distance keys do not need to match. Each map interpolates inside its own measured zones and clamps to the nearest endpoint outside them.

This makes physical testing—not an idealized projectile model—the source of the stationary shot. The percentage is converted to measured motor RPM only at the control boundary, keeping calibration and debugging readable.

### Use physics only as a bounded correction

Robot motion can still change the required release. When a fully calibrated physics model is available, the solver compares its moving and stationary predictions at the same distance and applies only that difference to the mapped setting. The result is clamped between the two measured values defining the active distance zone. Physics cannot replace the empirical stationary setting.

The optional model adapts Team 4414's published offline-search and polynomial approach. It models gravity and optional quadratic drag, and its generated fingerprint is invalidated by changed physical settings. If the model is missing, changed or outside its calibrated domain, the empirical map remains available without correction.

### Wait for stability, then check the actual opening

Competition shooting requires distinct fresh observations showing low HIVE motion near an endpoint for a continuous dwell. Red and blue maintain separate stability histories. Only the raised cell is eligible. The selected target remains fixed during the shot calculation; moving-HIVE shots are deferred.

Robot translation, chassis rotation and turret rotation contribute to muzzle velocity. Radial velocity can adjust the bounded mechanism targets, while tangential compensation adjusts turret aim. When physics correction is active, the runtime solver rechecks 27 combinations of speed, elevation and yaw error against the actual tilted aperture plane before accepting a shot. The drivetrain remains entirely under driver yaw control; there is no chassis aim assist.

### Connect the calculation to readiness

A valid trajectory does not immediately deploy the indexer. Localization, turret alignment, hood settling and flywheel speed must pass their checks, followed by an overall readiness dwell. The single flywheel and 1:1 Axon-feedback turret retain their calibrated controllers.

The optional model is a point-mass and aperture-clearance approximation. It does not simulate spin lift or rim collisions, and desktop tests do not establish shot accuracy. See the [shot-map and physics-correction guide](PHYSICS_SHOOTING.md) for calibration and limitations.

**Suggested visual:** independent hood/flywheel interpolation curves, a bounded motion correction, and the stable-HIVE/readiness gates on a shared timeline.

## 3. Logging and AdvantageScope — Seeing inside the robot

### One timeline for the whole system

A missed shot can begin with a camera observation, a delayed estimate or a mechanism that has not reached its target. Our logging system puts those signals on a shared timeline so we can investigate how they relate.

Each control-cycle snapshot records robot pose and uncertainty, separate red and blue HIVE states, tag observations, shot solutions, driver inputs and mechanism targets and measurements. Every run gets a uniquely named CSV file and a companion record of its calibration settings.

### Understanding why the robot waited

We record more than the final command. Vision logs each HIVE's latest fit result, fitting error and acceptance decision. The shooter records solution validity, its rejection reason and individual readiness blockers, including turret alignment, flywheel speed and hood settling.

This lets us trace a decision from measurement to action. If feeding is inhibited, we can compare the driver's request with the shot solution and the readiness checks that were evaluated in that cycle.

### Comparing predictions with response

The aiming logs include the fixed target position, muzzle position and velocity, flight time, clearance and trajectory-check count. Mechanism logs show turret target and measured angle, commanded servo position, flywheel speed and motor command, and indexer position commands.

Overlaying these signals helps us investigate tracking lag, flywheel recovery and changes in the calculated shot. The independent HIVE histories show whether target motion or confidence changed at the same time.

### Live views and recorded analysis

The same snapshot fields are available live through FTC Dashboard's AdvantageScope connection and afterward through AdvantageScope-compatible CSV files. We can compare time-series plots, inspect state changes and revisit a recorded interval during debugging.

Numeric validity channels distinguish unavailable measurements from real zeros. Frame sequence numbers and capture times identify repeated observations, while logging counters reveal dropped samples. Recorded data can be reviewed in AdvantageScope; the logger does not rerun the robot's control code.

### Keeping logging within limits

The control loop copies cached state into a bounded queue. A separate worker formats the CSV, writes files and publishes live telemetry at a limited rate. If that worker falls behind, the queue drops snapshots and reports the loss instead of waiting for storage to catch up.

File-size and free-space limits bound storage use, and shutdown attempts to drain the queue after actuator shutdown. Loop-stage timings and the previous full loop period remain part of the log, alongside the time spent collecting the previous snapshot. These measurements let us evaluate logging overhead on the robot.

**Suggested visual:** an AdvantageScope graph of flywheel target and measured RPM, turret error and shot readiness around one feed request, with a second panel showing vision acceptance and HIVE uncertainty.

**Validation status:** file formatting and failure handling are covered by desktop tests. Live connection, on-robot overhead and shooting behavior still require hardware trials.

---

**Explore the source:** [FTC-2027BIOBUZZ](https://github.com/khan-moazzin/FTC-2027BIOBUZZ/tree/main)

**Development status:** these sections describe implemented software. Accuracy, shot success and timing claims require calibrated robot trials. Built with the FTC SDK, SolversLib and Pedro Pathing, with turret aiming techniques adapted from Seattle Solvers and a physics-shot method inspired by Team 4414.
