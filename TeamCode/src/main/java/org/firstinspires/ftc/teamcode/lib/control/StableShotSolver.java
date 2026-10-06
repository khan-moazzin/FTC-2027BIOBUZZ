package org.firstinspires.ftc.teamcode.lib.control;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.config.PhysicsShotConfig;
import org.firstinspires.ftc.teamcode.config.ShotConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.math.Vec3;
import org.firstinspires.ftc.teamcode.lib.vision.HiveState;

/** Stable-HIVE empirical-map solver with optional bounded physics motion correction. */
public final class StableShotSolver {
  private static Vec3 mirror(double[] value, Field.Cell cell) {
    return new Vec3(value[0], cell == Field.Cell.SCORING ? value[1] : -value[1], value[2]);
  }

  public static ShotSolution solve(
      Pose robot,
      Velocity velocity,
      double turret,
      double turretRate,
      HiveState hive,
      Field.Hive alliance,
      Field.Cell cell,
      long now,
      VisionConfig vision,
      ShotConfig shots,
      PhysicsShotConfig physics,
      boolean stable) {
    ShotSolution solution = new ShotSolution();
    if (!stable
        || !hive.fresh(now, vision.hiveMaxAge)
        || Math.abs(hive.rate()) > physics.stableRate
        || Math.abs(Math.abs(hive.angle(now)) - Field.MAX_ANGLE)
            > physics.stableAngleTolerance) {
      solution.reason = "Waiting for stable HIVE";
      return solution;
    }
    if (cell != Field.raised(hive.angle(now))) {
      solution.reason = "Select raised stable cell";
      return solution;
    }
    if (!ShotMap.ready(shots)) {
      solution.reason = "Empirical hood/flywheel maps not calibrated";
      return solution;
    }
    if (!finite(
            turret,
            turretRate,
            robot.x(),
            robot.y(),
            robot.heading(),
            velocity.vx,
            velocity.vy,
            velocity.omega,
            shots.transferDelay)
        || shots.transferDelay < 0
        || shots.transferDelay > 1) {
      solution.reason = "Invalid motion/release delay";
      return solution;
    }

    Vec3 robotToMuzzle =
        new Vec3(vision.robotToTurret)
            .plus(new Vec3(shots.turretToMuzzle).rotateZ(turret))
            .rotateZ(robot.heading());
    Vec3 muzzle = new Vec3(robot.x(), robot.y(), 0).plus(robotToMuzzle);
    Vec3 turretArm = new Vec3(shots.turretToMuzzle).rotateZ(turret + robot.heading());
    Vec3 platformVelocity =
        new Vec3(
            velocity.vx - velocity.omega * robotToMuzzle.y - turretRate * turretArm.y,
            velocity.vy + velocity.omega * robotToMuzzle.x + turretRate * turretArm.x,
            0);

    boolean physicsReady = physicsReady(physics);
    double hiveAngle = hive.angle(now);
    Vec3 goal = Field.cell(alliance, cell, hiveAngle);
    if (physicsReady)
      goal =
          goal.plus(
              mirror(physics.apertureOffset, cell).rotateX(hiveAngle).times(1.0 / .0254));

    Vec3 release = muzzle.plus(platformVelocity.times(shots.transferDelay));
    Vec3 delta = goal.minus(release);
    double distance = Math.hypot(delta.x, delta.y);
    ShotMap.Interpolation hood = ShotMap.interpolate(shots.hoodMap, distance);
    ShotMap.Interpolation flywheel = ShotMap.interpolate(shots.flywheelMap, distance);
    if (hood == null || flywheel == null || flywheel.value <= 0 || flywheel.value > 100) {
      solution.reason = "Invalid empirical hood/flywheel map";
      return solution;
    }

    solution.muzzle = muzzle;
    solution.predictedTarget = goal;
    solution.distance = distance;
    solution.height = delta.z;
    double flywheelPercent = flywheel.value;
    solution.rpm = flywheelPercent / 100 * MechanismConfig.maxRpm;
    solution.hood = Math.toRadians(hood.value);
    solution.mapClamped = hood.clamped || flywheel.clamped;
    solution.impactVariance = hive.variance(now, vision.hiveAccelerationNoise);
    if (!Double.isFinite(solution.impactVariance)
        || Math.sqrt(solution.impactVariance) > vision.maxHiveSigma) {
      solution.reason = "HIVE uncertain";
      return solution;
    }

    double bearing = Math.atan2(delta.y, delta.x);
    double radialInches =
        platformVelocity.x * Math.cos(bearing) + platformVelocity.y * Math.sin(bearing);
    double tangentInches =
        -platformVelocity.x * Math.sin(bearing) + platformVelocity.y * Math.cos(bearing);
    double launchYaw = bearing;

    double distanceMeters = distance * .0254;
    double radialMeters = radialInches * .0254;
    boolean insidePhysicsDomain =
        physicsReady
            && distanceMeters >= physics.minDistance
            && distanceMeters <= physics.maxDistance
            && Math.abs(radialMeters) <= physics.maxRadialSpeed
            && Math.abs(muzzle.z * .0254 - physics.muzzleHeight) <= physics.heightTolerance
            && Math.abs(goal.z * .0254 - physics.targetHeight) <= physics.heightTolerance;
    if (insidePhysicsDomain) {
      double[] moving = ShotPolynomial.basis(distanceMeters, radialMeters, physics);
      double[] stationary = ShotPolynomial.basis(distanceMeters, 0, physics);
      double movingSpeed = ShotPolynomial.evaluate(physics.speedCoefficients, moving);
      double stationarySpeed = ShotPolynomial.evaluate(physics.speedCoefficients, stationary);
      double movingAngle = ShotPolynomial.evaluate(physics.angleCoefficients, moving);
      double stationaryAngle = ShotPolynomial.evaluate(physics.angleCoefficients, stationary);
      double flight = ShotPolynomial.evaluate(physics.flightCoefficients, moving);
      if (finite(movingSpeed, stationarySpeed, movingAngle, stationaryAngle, flight)
          && movingSpeed > 0
          && stationarySpeed > 0
          && flight > 0
          && flight <= physics.maxFlightSeconds) {
        flywheelPercent =
            clamp(
                flywheelPercent
                    + (movingSpeed - stationarySpeed)
                        / physics.speedPerRpm
                        / MechanismConfig.maxRpm
                        * 100,
                flywheel.zoneMin,
                flywheel.zoneMax);
        solution.rpm = flywheelPercent / 100 * MechanismConfig.maxRpm;
        solution.hood =
            clamp(
                solution.hood + movingAngle - stationaryAngle,
                Math.toRadians(hood.zoneMin),
                Math.toRadians(hood.zoneMax));
        solution.flight = flight;
        solution.physicsCorrected = true;

        double exitSpeed = solution.rpm * physics.speedPerRpm;
        double elevation = solution.hood + physics.launchAngleOffset;
        double horizontalSpeed = exitSpeed * Math.cos(elevation);
        launchYaw = bearing + Math.atan2(-tangentInches * .0254, horizontalSpeed);
        if (!validatePhysics(
            solution,
            release.times(.0254),
            goal.times(.0254),
            mirror(physics.apertureNormal, cell).rotateX(hiveAngle),
            platformVelocity.times(.0254),
            launchYaw,
            elevation,
            exitSpeed,
            physics)) return solution;
      }
    }

    solution.launchVelocity = platformVelocity;
    solution.angle =
        Angles.wrap(launchYaw - robot.heading() - velocity.omega * shots.transferDelay);
    solution.angularVelocity =
        MovingShotSolver.lineOfSightRate(
            delta.x, delta.y, -platformVelocity.x, -platformVelocity.y, velocity.omega);
    solution.valid =
        finite(solution.angle, solution.angularVelocity, solution.rpm, solution.hood)
            && solution.rpm > 0
            && solution.rpm <= MechanismConfig.maxRpm;
    solution.reason =
        solution.valid
            ? solution.physicsCorrected
                ? "Stable HIVE empirical maps + bounded physics correction"
                : solution.mapClamped
                    ? "Stable HIVE empirical maps (distance clamped)"
                    : "Stable HIVE empirical maps"
            : "Mechanism setpoint invalid";
    return solution;
  }

  private static boolean validatePhysics(
      ShotSolution solution,
      Vec3 release,
      Vec3 goal,
      Vec3 normal,
      Vec3 platformVelocity,
      double launchYaw,
      double elevation,
      double exitSpeed,
      PhysicsShotConfig physics) {
    if (exitSpeed > physics.maxExitSpeed
        || elevation < physics.minLaunchAngle
        || elevation > physics.maxLaunchAngle) {
      solution.reason = "Physics limits rejected mapped shot";
      return false;
    }
    double clear = physics.openingRadius - physics.projectileRadius - physics.clearanceMargin;
    double worst = 0;
    Ballistics.Crossing nominal = null;
    for (int speedError = -1; speedError <= 1; speedError++)
      for (int angleError = -1; angleError <= 1; angleError++)
        for (int yawError = -1; yawError <= 1; yawError++) {
          double speed =
              exitSpeed
                  * (1 + speedError * physics.speedSigmaFraction * physics.errorMultiplier);
          double angle = elevation + angleError * physics.angleSigma * physics.errorMultiplier;
          double yaw = launchYaw + yawError * physics.angleSigma * physics.errorMultiplier;
          Vec3 projectile =
              new Vec3(
                      speed * Math.cos(angle) * Math.cos(yaw),
                      speed * Math.cos(angle) * Math.sin(yaw),
                      speed * Math.sin(angle))
                  .plus(platformVelocity);
          Ballistics.Crossing hit =
              Ballistics.cross(
                  release,
                  projectile,
                  goal,
                  normal,
                  physics.dragPerMeter,
                  physics.maxFlightSeconds);
          solution.iterations++;
          if (hit == null || hit.miss > clear) {
            solution.reason = "Physics clearance rejected mapped shot";
            return false;
          }
          worst = Math.max(worst, hit.miss);
          if (speedError == 0 && angleError == 0 && yawError == 0) nominal = hit;
        }
    solution.flight = nominal.time;
    solution.clearance = clear - worst;
    return true;
  }

  private static boolean physicsReady(PhysicsShotConfig physics) {
    return physics.calibrated
        && physics.modelGenerated
        && physics.geometryVerified
        && ShotPolynomial.physicalInputsValid(physics)
        && ShotPolynomial.signature(physics).equals(physics.modelSignature);
  }

  private static boolean finite(double... values) {
    for (double value : values) if (!Double.isFinite(value)) return false;
    return true;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
