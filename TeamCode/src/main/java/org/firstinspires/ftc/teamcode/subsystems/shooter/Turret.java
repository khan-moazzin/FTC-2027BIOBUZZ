package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.*;

public class Turret {

    private static final double COMMAND_EPSILON_DEGREES = 0.50;

    private final ServoImplEx turret1;
    private final ServoImplEx turret2;

    private double position;
    private long estimatedReadyNanos;

    public Turret(HardwareMap hw) {
        turret1 = hw.get(ServoImplEx.class, "turret1");
        turret2 = hw.get(ServoImplEx.class, "turret2");

        PwmControl.PwmRange axonRange =
                new PwmControl.PwmRange(AXON_PWM_MIN_US, AXON_PWM_MAX_US);
        turret1.setPwmRange(axonRange);
        turret2.setPwmRange(axonRange);

        turret1.setDirection(Servo.Direction.FORWARD);
        turret2.setDirection(Servo.Direction.REVERSE);

        position = TURRET_CENTER_POSITION;
        apply(position);
        estimatedReadyNanos = System.nanoTime() + secondsToNanos(
                180.0 / 60.0 * TURRET_SECONDS_PER_60_DEGREES
                        + SERVO_SETTLE_MARGIN_SECONDS);
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
        return goTo(TURRET_CENTER_POSITION);
    }

    // -----------------------------------------------------
    // ANGLE CONTROL
    // -----------------------------------------------------
    /**
     * 0 = forward and CCW is positive. Returns false and holds the previous target
     * if the request is not finite. An unreachable bearing parks at the nearer
     * configured limit and returns false.
     */
    public boolean setAngle(double degrees) {
        if (!Double.isFinite(degrees)) return false;

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

    public double getAngle() {
        return servoToAngle(position);
    }

    public static double minAngle() {
        return TURRET_MIN_DEGREES;
    }

    public static double maxAngle() {
        return TURRET_MAX_DEGREES;
    }

    // -----------------------------------------------------
    // SERVO-UNIT CONTROL
    // -----------------------------------------------------
    public void setPosition(double target) {
        if (!Double.isFinite(target)) return;

        double next = clamp(target, minPosition(), maxPosition());
        double travelDegrees = Math.abs(servoToAngle(next) - servoToAngle(position));
        if (travelDegrees >= COMMAND_EPSILON_DEGREES) {
            long travelNanos = secondsToNanos(
                    travelDegrees / 60.0 * TURRET_SECONDS_PER_60_DEGREES
                            + SERVO_SETTLE_MARGIN_SECONDS);
            estimatedReadyNanos = Math.max(
                    estimatedReadyNanos,
                    System.nanoTime() + travelNanos);
        }
        position = next;
        apply(position);
    }

    public double getPosition() {
        return position;
    }

    /** Open-loop estimate only; no analog feedback is used. */
    public boolean isSettled() {
        return System.nanoTime() >= estimatedReadyNanos;
    }

    // -----------------------------------------------------
    // HELPERS
    // -----------------------------------------------------
    private static final double[] WRAP_OFFSETS = {0.0, -360.0, 360.0};

    private static double minPosition() {
        return angleToServo(TURRET_MIN_DEGREES);
    }

    private static double maxPosition() {
        return angleToServo(TURRET_MAX_DEGREES);
    }

    private static double angleToServo(double degrees) {
        return TURRET_CENTER_POSITION + degrees / TURRET_DEGREES_PER_SERVO_UNIT;
    }

    private static double servoToAngle(double servoPosition) {
        return (servoPosition - TURRET_CENTER_POSITION) * TURRET_DEGREES_PER_SERVO_UNIT;
    }

    /** To [-180, 180], preserving the sign of an exact 180-degree request. */
    private static double normalize(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }

    private void apply(double p) {
        turret1.setPosition(p);
        turret2.setPosition(p);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static long secondsToNanos(double seconds) {
        return (long) (Math.max(0.0, seconds) * 1_000_000_000.0);
    }
}
