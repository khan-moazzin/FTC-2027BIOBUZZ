package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/** Aiming math. Degrees, CCW positive, 0 = robot forward. Goal x/y come from BiobuzzVision.getHivePivot(). */
public final class Aiming {

    private Aiming() {}

    /** Horizontal robot-to-goal distance, inches. Feeds the shot map. */
    public static double distanceTo(Pose robot, double goalX, double goalY) {
        return Math.hypot(goalX - robot.x(), goalY - robot.y());
    }

    /** Robot-relative bearing to the goal, from pose alone. Pass to Turret.splitAim(). */
    public static double bearingTo(Pose robot, double goalX, double goalY) {
        double fieldBearing = Math.toDegrees(Math.atan2(goalY - robot.y(), goalX - robot.x()));
        return normalize(fieldBearing - Math.toDegrees(robot.heading()));
    }

    /** Measured bearing, drift-free. tx is +right and turret angle is +CCW, hence the subtraction. */
    public static double bearingFromLimelight(double turretAngle, double tx) {
        return normalize(turretAngle - tx);
    }

    /** To (-180, 180]. */
    public static double normalize(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d <= -180.0) d += 360.0;
        return d;
    }
}
