package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.planners.ShooterConstants;

/** Aiming math. Degrees, CCW positive, 0 = robot forward. */
public final class Aiming {

    private Aiming() {}

    /** Horizontal robot-to-goal distance, inches. Feeds the shot map. */
    public static double distanceTo(Pose robot, double goalX, double goalY) {
        return Math.hypot(goalX - robot.x(), goalY - robot.y());
    }

    /** Robot-relative bearing to the goal, from pose alone. Pass to Turret.setAngle(). */
    public static double bearingTo(Pose robot, double goalX, double goalY) {
        double fieldBearing = Math.toDegrees(Math.atan2(goalY - robot.y(), goalX - robot.x()));
        return normalize(fieldBearing - Math.toDegrees(robot.heading()));
    }

    /** Field pose of the turret rotation axis from the calibrated robot-frame offset. */
    public static Pose turretAxisPose(Pose robot) {
        double cos = Math.cos(robot.heading());
        double sin = Math.sin(robot.heading());
        double x = robot.x()
                + cos * ShooterConstants.TURRET_FORWARD_IN
                - sin * ShooterConstants.TURRET_LEFT_IN;
        double y = robot.y()
                + sin * ShooterConstants.TURRET_FORWARD_IN
                + cos * ShooterConstants.TURRET_LEFT_IN;
        return new Pose(x, y, robot.heading());
    }

    public static double distanceFromTurret(Pose robot, double goalX, double goalY) {
        return distanceTo(turretAxisPose(robot), goalX, goalY);
    }

    public static double bearingFromTurret(Pose robot, double goalX, double goalY) {
        return bearingTo(turretAxisPose(robot), goalX, goalY);
    }

    /** Measured bearing, drift-free. tx is +right and turret angle is +CCW, hence the subtraction. */
    public static double bearingFromLimelight(double turretAngle, double tx) {
        return normalize(turretAngle - tx);
    }

    /** To [-180, 180], preserving the sign of an exact 180-degree result. */
    public static double normalize(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }
}
