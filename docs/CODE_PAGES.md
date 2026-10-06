# Code — Understand the field. Predict the shot. Understand every decision.

Our software connects a moving camera, a moving HIVE and a moving robot. Three parts of the code make that possible: vision that accounts for changing geometry, physics calculations for a stable scoring target, and logging that connects what the robot saw to what it calculated and commanded.

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

## 2. Advanced shot calculation — Physics for a stable HIVE

### Search offline, calculate on the robot

Our shooter adapts Team 4414's published approach: simulate trajectories, select shots that tolerate launch errors, then fit a compact polynomial. This is our implementation of their method; their 2026 shooter source was not publicly available when we researched it.

The simulator models gravity and optional quadratic air drag. It searches launch angles and speeds across a configured distance and radial robot-velocity range. Candidate shots must descend into the target with clearance under configured speed and angle errors. A second-order polynomial represents exit speed, launch angle and flight time. A separate validation grid checks the fit before export.

### Measure the launcher, then generate its model

We record distance, height difference, RPM, hood angle and flight time from measured shots. The calibration fits an RPM-to-exit-speed coefficient and an angle correction, reserving every third sample for validation. Opening geometry, projectile size and uncertainty settings require physical measurements too.

The generated model is bounded by the calibrated operating range. Changed physical settings invalidate its fingerprint, and requests outside its domain are rejected. There is no empirical shot-map fallback.

### Wait for stability, then check the actual opening

Competition shooting requires distinct fresh observations showing low HIVE motion near an endpoint for a continuous dwell. Red and blue maintain separate stability histories. Only the raised cell is eligible. The selected target remains fixed during the shot calculation; moving-HIVE shots are deferred.

Robot motion compensation remains separate. Translation, chassis rotation and turret rotation contribute to muzzle velocity. Radial velocity enters the polynomial, while tangential compensation adjusts the launch vector. The runtime solver rechecks 27 combinations of speed, elevation and yaw error against the actual tilted aperture plane before accepting a shot.

### Connect the calculation to readiness

A valid trajectory does not immediately deploy the indexer. Localization, turret alignment, hood settling and flywheel speed must pass their checks, followed by an overall readiness dwell. The single flywheel and 1:1 Axon-feedback turret retain their calibrated controllers.

The model is a point-mass and aperture-clearance approximation. It does not simulate spin lift or rim collisions, and desktop tests do not establish shot accuracy. See the [physics shooting guide](PHYSICS_SHOOTING.md) for calibration and limitations.

**Suggested visual:** a family of simulated arcs, the chosen error-tolerant shot, and the stable-HIVE/readiness gates on a shared timeline.

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
