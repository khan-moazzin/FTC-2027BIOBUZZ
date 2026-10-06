package org.firstinspires.ftc.teamcode.lib.control;

import com.pedropathing.math.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.math.Vec3;
import org.firstinspires.ftc.teamcode.lib.vision.HiveState;

/**
 * Original adaptation of 4414's published physics/precomputed-polynomial method; no target-motion
 * mode.
 */
public final class StableShotSolver {
  private static Vec3 mirror(double[] v, Field.Cell cell) {
    return new Vec3(v[0], cell == Field.Cell.SCORING ? v[1] : -v[1], v[2]);
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
      PhysicsShotConfig c,
      boolean stable) {
    ShotSolution s = new ShotSolution();
    if (!stable
        || !hive.fresh(now, vision.hiveMaxAge)
        || Math.abs(hive.rate()) > c.stableRate
        || Math.abs(Math.abs(hive.angle(now)) - Field.MAX_ANGLE) > c.stableAngleTolerance) {
      s.reason = "Waiting for stable HIVE";
      return s;
    }
    if (cell != Field.raised(hive.angle(now))) {
      s.reason = "Select raised stable cell";
      return s;
    }
    if (!c.calibrated
        || !c.modelGenerated
        || !c.geometryVerified
        || !ShotPolynomial.physicalInputsValid(c)
        || !ShotPolynomial.signature(c).equals(c.modelSignature)) {
      s.reason = "Physics calibration/model missing or changed";
      return s;
    }
    if (!Double.isFinite(turret)
        || !Double.isFinite(turretRate)
        || !Double.isFinite(robot.x())
        || !Double.isFinite(robot.y())
        || !Double.isFinite(robot.heading())
        || !Double.isFinite(velocity.vx)
        || !Double.isFinite(velocity.vy)
        || !Double.isFinite(velocity.omega)
        || !Double.isFinite(shots.transferDelay)
        || shots.transferDelay < 0
        || shots.transferDelay > 1) {
      s.reason = "Invalid motion/release delay";
      return s;
    }
    Vec3 offset =
        new Vec3(vision.robotToTurret)
            .plus(new Vec3(shots.turretToMuzzle).rotateZ(turret))
            .rotateZ(robot.heading());
    Vec3 muzzle = new Vec3(robot.x(), robot.y(), 0).plus(offset).times(.0254);
    Vec3 arm = new Vec3(shots.turretToMuzzle).rotateZ(turret + robot.heading()).times(.0254);
    Vec3 platform =
        new Vec3(
            velocity.vx * .0254 - velocity.omega * offset.y * .0254 - turretRate * arm.y,
            velocity.vy * .0254 + velocity.omega * offset.x * .0254 + turretRate * arm.x,
            0);
    // Freeze the observed target. No HIVE angle extrapolation to future impact time.
    double angle = hive.angle(now);
    Vec3 goal =
        Field.cell(alliance, cell, angle)
            .times(.0254)
            .plus(mirror(c.apertureOffset, cell).rotateX(angle));
    Vec3 normal = mirror(c.apertureNormal, cell).rotateX(angle);
    Vec3 release = muzzle.plus(platform.times(shots.transferDelay)), delta = goal.minus(release);
    double distance = Math.hypot(delta.x, delta.y), bearing = Math.atan2(delta.y, delta.x);
    double radial = platform.x * Math.cos(bearing) + platform.y * Math.sin(bearing);
    double tangent = -platform.x * Math.sin(bearing) + platform.y * Math.cos(bearing);
    s.muzzle = muzzle.times(1 / .0254);
    s.predictedTarget = goal.times(1 / .0254);
    s.launchVelocity = platform.times(1 / .0254);
    s.distance = distance / .0254;
    s.height = delta.z / .0254;
    s.impactVariance = hive.variance(now, vision.hiveAccelerationNoise);
    if (!Double.isFinite(s.impactVariance) || Math.sqrt(s.impactVariance) > vision.maxHiveSigma) {
      s.reason = "HIVE uncertain";
      return s;
    }
    if (distance < c.minDistance
        || distance > c.maxDistance
        || Math.abs(radial) > c.maxRadialSpeed
        || Math.abs(muzzle.z - c.muzzleHeight) > c.heightTolerance
        || Math.abs(goal.z - c.targetHeight) > c.heightTolerance) {
      s.reason = "Outside generated physics domain";
      return s;
    }
    double[] basis = ShotPolynomial.basis(distance, radial, c);
    double speed = ShotPolynomial.evaluate(c.speedCoefficients, basis),
        elevation = ShotPolynomial.evaluate(c.angleCoefficients, basis);
    double flight = ShotPolynomial.evaluate(c.flightCoefficients, basis);
    double horizontal = speed * Math.cos(elevation), vertical = speed * Math.sin(elevation);
    double launchYaw = bearing + Math.atan2(-tangent, horizontal);
    double launchElevation = Math.atan2(vertical, Math.hypot(horizontal, tangent));
    double exitSpeed = Math.sqrt(speed * speed + tangent * tangent);
    if (!Double.isFinite(speed)
        || speed <= 0
        || !Double.isFinite(elevation)
        || !Double.isFinite(flight)
        || flight <= 0
        || flight > c.maxFlightSeconds
        || exitSpeed > c.maxExitSpeed
        || launchElevation < c.minLaunchAngle
        || launchElevation > c.maxLaunchAngle) {
      s.reason = "Invalid polynomial / exit limits";
      return s;
    }
    s.rpm = exitSpeed / c.speedPerRpm;
    s.hood = launchElevation - c.launchAngleOffset;
    s.angle = Angles.wrap(launchYaw - robot.heading() - velocity.omega * shots.transferDelay);
    s.angularVelocity =
        MovingShotSolver.lineOfSightRate(
            delta.x, delta.y, -platform.x, -platform.y, velocity.omega);
    double clear = c.openingRadius - c.projectileRadius - c.clearanceMargin;
    double worst = 0;
    Ballistics.Crossing nominal = null;
    for (int sv = -1; sv <= 1; sv++)
      for (int av = -1; av <= 1; av++)
        for (int yv = -1; yv <= 1; yv++) {
          double v = exitSpeed * (1 + sv * c.speedSigmaFraction * c.errorMultiplier);
          double a = launchElevation + av * c.angleSigma * c.errorMultiplier;
          double yaw = launchYaw + yv * c.angleSigma * c.errorMultiplier;
          Vec3 launch =
              new Vec3(
                      v * Math.cos(a) * Math.cos(yaw),
                      v * Math.cos(a) * Math.sin(yaw),
                      v * Math.sin(a))
                  .plus(platform);
          Ballistics.Crossing hit =
              Ballistics.cross(release, launch, goal, normal, c.dragPerMeter, c.maxFlightSeconds);
          s.iterations++;
          if (hit == null || hit.miss > clear) {
            s.reason = "Trajectory clearance / uncertainty rejected";
            return s;
          }
          worst = Math.max(worst, hit.miss);
          if (sv == 0 && av == 0 && yv == 0) nominal = hit;
        }
    s.flight = nominal.time;
    s.clearance = clear - worst;
    s.valid =
        Double.isFinite(s.angle)
            && Double.isFinite(s.angularVelocity)
            && s.rpm <= MechanismConfig.maxRpm;
    s.reason = s.valid ? "Stable HIVE physics" : "Mechanism speed limit";
    return s;
  }
}
