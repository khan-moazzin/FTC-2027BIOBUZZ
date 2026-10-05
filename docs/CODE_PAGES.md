# Code — Understand the field. Predict the shot. Understand every decision.

Our software connects a moving camera, a moving HIVE and a moving robot. Three parts of the code make that possible: vision that accounts for changing geometry, shot calculations that predict the moment of impact, and logging that connects what the robot saw to what it calculated and commanded.

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

## 2. Advanced shot calculation — Aim for where the cell will be

### Predicting the moment of impact

Between requesting a shot and reaching the HIVE, the projectile passes through a launch delay and a flight interval. During that time, the scoring cell can move.

Our solver predicts the selected cell's future position, calculates a shot and uses its flight time to update that prediction. It repeats until the flight-time estimate converges. This couples target motion, target height, turret direction, hood angle and flywheel speed in one solution.

The prediction assumes constant velocities over the shot interval and respects the HIVE's angular limits. Increasing prediction uncertainty can prevent a shot from being released.

### Accounting for motion at the muzzle

The muzzle can move even when the robot's center stays in place. Chassis rotation and turret rotation create tangential velocity because the launch point is offset from their axes.

Our calculation includes chassis translation, chassis rotation and turret rotation when estimating muzzle velocity. That velocity contributes to the projectile's motion, so it changes the required aim offset.

### Combining prediction with measured shots

Our shot map stores horizontal distance, target height difference, flywheel RPM, hood angle and measured flight time. The solver interpolates within regions covered by recorded samples and rejects requests outside that coverage.

The turret tracks the resulting angle using two analog feedback channels, calibrated travel limits and a velocity-based lead that compensates for measured response lag. Each flywheel has separate feedback and control coefficients.

### Releasing only when the whole system is ready

A calculated solution is one condition for firing. The target estimate must also be fresh enough, localization must be healthy, the turret must be aligned, the hood must have settled, and both flywheels must remain within speed tolerance for a continuous dwell period.

These checks connect prediction to the physical state of the robot. The shooter can prepare its mechanisms while waiting for the conditions needed to feed a projectile.

Our predictive aiming adapts Seattle Solvers' iterative virtual-target and velocity-lead techniques, extended for BIOBUZZ geometry, moving cells and our robot's feedback. Attribution remains in the codebase.

**Suggested visual:** current cell position, predicted impact position and compensated aim direction, with launch delay and flight time labeled.

## 3. Logging and AdvantageScope — Seeing inside the robot

### One timeline for the whole system

A missed shot can begin with a camera observation, a delayed estimate or a mechanism that has not reached its target. Our logging system puts those signals on a shared timeline so we can investigate how they relate.

Each control-cycle snapshot records robot pose and uncertainty, separate red and blue HIVE states, tag observations, shot predictions, driver inputs and mechanism targets and measurements. Every run gets a uniquely named CSV file and a companion record of its calibration settings.

### Understanding why the robot waited

We record more than the final command. Vision logs each HIVE's latest fit result, fitting error and acceptance decision. The shooter records solution validity, its rejection reason and individual readiness blockers, including turret alignment, flywheel speed and hood settling.

This lets us trace a decision from measurement to action. If feeding is inhibited, we can compare the driver's request with the shot solution and the readiness checks that were evaluated in that cycle.

### Comparing predictions with response

The aiming logs include the predicted cell position, muzzle position and velocity, flight time and solver iterations. Mechanism logs show turret target and measured angle, commanded servo position, both flywheel speeds and motor commands.

Overlaying these signals helps us investigate tracking lag, flywheel recovery and changes in the predicted shot. The independent HIVE histories show whether target motion or confidence changed at the same time.

### Live views and recorded analysis

The same snapshot fields are available live through FTC Dashboard's AdvantageScope connection and afterward through AdvantageScope-compatible CSV files. We can compare time-series plots, inspect state changes and revisit a recorded interval during debugging.

Numeric validity channels distinguish unavailable measurements from real zeros. Frame sequence numbers and capture times identify repeated observations, while logging counters reveal dropped samples. Recorded data can be reviewed in AdvantageScope; the logger does not rerun the robot's control code.

### Keeping logging within limits

The control loop copies cached state into a bounded queue. A separate worker formats the CSV, writes files and publishes live telemetry at a limited rate. If that worker falls behind, the queue drops snapshots and reports the loss instead of waiting for storage to catch up.

File-size and free-space limits bound storage use, and shutdown attempts to drain the queue after actuator shutdown. Loop-stage timings and the previous full loop period remain part of the log, alongside the time spent collecting the previous snapshot. These measurements let us evaluate logging overhead on the robot.

**Suggested visual:** an AdvantageScope graph of flywheel target and measured RPM, turret error and shot readiness around one feed request, with a second panel showing vision acceptance and HIVE uncertainty.

**Validation status:** file formatting and failure handling are covered by desktop tests. Live connection, on-robot overhead and shooting behavior still require hardware trials.

---

**Explore the source:** [FTC-2027BIOBUZZ](https://github.com/khan-moazzin/FTC-2027BIOBUZZ/tree/codex/solverslib-biobuzz-integration)

**Development status:** these sections describe implemented software. Accuracy, shot success and timing claims require calibrated robot trials. Built with the FTC SDK, SolversLib and Pedro Pathing, with aiming techniques adapted from Seattle Solvers.
