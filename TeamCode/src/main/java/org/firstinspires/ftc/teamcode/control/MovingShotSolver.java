/*
 * Virtual-goal iteration adapted from FTC-23511/Decode-2026 MathFunctions.VirtualGoalSolver,
 * commit 02a453104b0b208a885571b5fa93b0465c054b78. BSD-3-Clause-Clear notice:
 * docs/licenses/Seattle-DECODE-LICENSE.txt. Modified for moving 3D BIOBUZZ cells.
 */
package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.math.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Vec3;
import org.firstinspires.ftc.teamcode.vision.HiveState;

public final class MovingShotSolver {
  public static final class Solution {
    public boolean valid;
    public String reason = "No solution";
    public double angle, angularVelocity, rpm, hood, flight, distance, height;
  }

  public static double lineOfSightRate(
      double rx, double ry, double relativeVx, double relativeVy, double robotOmega) {
    double d = rx * rx + ry * ry;
    return d < 1e-6 ? Double.NaN : (rx * relativeVy - ry * relativeVx) / d - robotOmega;
  }

  public static Solution solve(
      Pose robot,
      Velocity velocity,
      double turret,
      double turretRate,
      HiveState hive,
      Field.Hive alliance,
      Field.Cell cell,
      long now,
      VisionConfig vision,
      ShotConfig shots) {
    Solution s = new Solution();
    if (!hive.fresh(now, vision.hiveMaxAge)
        || Math.sqrt(hive.variance(now, vision.hiveAccelerationNoise)) > vision.maxHiveSigma) {
      s.reason = "HIVE stale/uncertain";
      return s;
    }
    if (!Double.isFinite(turret) || !shots.calibrated) {
      s.reason = "Shot/turret calibration missing";
      return s;
    }
    Vec3 offset =
        new Vec3(vision.robotToTurret)
            .plus(new Vec3(shots.turretToMuzzle).rotateZ(turret))
            .rotateZ(robot.heading());
    Vec3 muzzle = new Vec3(robot.x(), robot.y(), 0).plus(offset);
    Vec3 arm = new Vec3(shots.turretToMuzzle).rotateZ(turret + robot.heading());
    Vec3 launchVelocity =
        new Vec3(
            velocity.vx - velocity.omega * offset.y - turretRate * arm.y,
            velocity.vy + velocity.omega * offset.x + turretRate * arm.x,
            0);
    double tof = .4;
    Vec3 delta = null, goalVelocity = null;
    boolean converged = false;
    for (int i = 0; i < 12; i++) {
      double t = shots.transferDelay + tof;
      double a = hive.angle(now + (long) (t * 1e9));
      Vec3 goal = Field.cell(alliance, cell, a);
      goalVelocity =
          Field.cellVelocity(cell, a, Math.abs(a) >= Field.MAX_ANGLE - 1e-6 ? 0 : hive.rate());
      delta = goal.minus(muzzle).minus(launchVelocity.times(t));
      s.distance = Math.hypot(delta.x, delta.y);
      s.height = delta.z;
      double[] setting = ShotMap.lookup(shots, s.distance, s.height);
      if (setting == null) {
        s.reason = "Outside measured shot map";
        return s;
      }
      s.rpm = setting[0];
      s.hood = setting[1];
      s.flight = setting[2];
      if (Math.abs(tof - s.flight) < .001) {
        converged = true;
        break;
      }
      tof = s.flight;
    }
    if (Math.sqrt(
            hive.variance(
                now + (long) ((shots.transferDelay + s.flight) * 1e9),
                vision.hiveAccelerationNoise))
        > vision.maxHiveSigma) {
      s.reason = "Impact HIVE prediction uncertain";
      return s;
    }
    if (!converged) {
      s.reason = "Shot iteration did not converge";
      return s;
    }
    s.angle = Angles.wrap(Math.atan2(delta.y, delta.x) - robot.heading());
    Vec3 relative = goalVelocity.minus(launchVelocity);
    s.angularVelocity = lineOfSightRate(delta.x, delta.y, relative.x, relative.y, velocity.omega);
    s.valid =
        Double.isFinite(s.angle)
            && Double.isFinite(s.angularVelocity)
            && Double.isFinite(s.rpm)
            && Double.isFinite(s.hood);
    s.reason = s.valid ? "Tracking" : "Invalid motion";
    return s;
  }
}
