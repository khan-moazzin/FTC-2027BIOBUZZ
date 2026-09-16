package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Turret {

    private final Servo left;
    private final Servo right;

    private double position;

    public Turret(HardwareMap hw) {
        left = hw.get(Servo.class, "turret1");
        right = hw.get(Servo.class, "turret2");

        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        position = TURRET_CENTER;
        apply(position);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command goTo(double target) {
        return Commands.instant(() -> setPosition(target)).requiring(this);
    }

    public Command aimAt(double degrees) {
        return Commands.instant(() -> setAngle(degrees)).requiring(this);
    }

    public Command center() {
        return goTo(TURRET_CENTER);
    }

    // -----------------------------------------------------
    // ANGLE CONTROL
    // -----------------------------------------------------
    /** 0 = forward, CCW positive. Returns false if unreachable; then parks at the nearest limit. */
    public boolean setAngle(double degrees) {
        double normalized = normalize(degrees);
        double low = minAngle();
        double high = maxAngle();

        for (double offset : WRAP_OFFSETS) {
            double candidate = normalized + offset;
            if (candidate >= low && candidate <= high) {
                setPosition(angleToServo(candidate));
                return true;
            }
        }

        setPosition(angleToServo(
                Math.abs(normalized - low) < Math.abs(normalized - high) ? low : high));
        return false;
    }

    /**Gated Shooting**/
    public boolean canReach(double degrees) {
        double normalized = normalize(degrees);
        for (double offset : WRAP_OFFSETS) {
            double candidate = normalized + offset;
            if (candidate >= minAngle() && candidate <= maxAngle()) return true;
        }
        return false;
    }

    public double getAngle() {
        return servoToAngle(position);
    }

    public static double[] splitAim(double targetDegrees) {
        double total = normalize(targetDegrees);
        double low = minAngle();
        double high = maxAngle();

        if (total >= low && total <= high) return new double[] {0.0, total};

        double turret = (total > high) ? high : low;
        return new double[] {total - turret, turret};
    }

    public static double minAngle() {
        return servoToAngle(TURRET_MIN);
    }

    public static double maxAngle() {
        return servoToAngle(TURRET_MAX);
    }

    // -----------------------------------------------------
    // SERVO-UNIT CONTROL
    // -----------------------------------------------------
    public void setPosition(double target) {
        position = clamp(target, TURRET_MIN, TURRET_MAX);
        apply(position);
    }

    public double getPosition() {
        return position;
    }

    // -----------------------------------------------------
    // HELPERS
    // -----------------------------------------------------
    private static final double[] WRAP_OFFSETS = {0.0, -360.0, 360.0};

    /** Configure during bringu0 */
    private static double degreesPerServoUnit() {
        return TURRET_RANGE_DEGREES / (TURRET_MAX - TURRET_MIN);
    }

    private static double angleToServo(double degrees) {
        return TURRET_CENTER + degrees / degreesPerServoUnit();
    }

    private static double servoToAngle(double servoPosition) {
        return (servoPosition - TURRET_CENTER) * degreesPerServoUnit();
    }

    /** To (-180, 180]. */
    private static double normalize(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d <= -180.0) d += 360.0;
        return d;
    }

    private void apply(double p) {
        left.setPosition(p);
        right.setPosition(p);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
