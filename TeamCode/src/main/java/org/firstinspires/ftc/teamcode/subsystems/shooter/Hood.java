package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.*;

/** Axon MINI MK2 with a 30:173 reduction. */
public class Hood {

    private static final double COMMAND_EPSILON_DEGREES = 0.25;

    private final ServoImplEx hood;

    private double position;
    private double targetDegrees;
    private long estimatedReadyNanos;

    public Hood(HardwareMap hw) {
        hood = hw.get(ServoImplEx.class, "hood");
        hood.setPwmRange(new PwmControl.PwmRange(AXON_PWM_MIN_US, AXON_PWM_MAX_US));
        hood.setDirection(ServoImplEx.Direction.FORWARD);

        targetDegrees = HOOD_STOW_DEGREES;
        position = angleToPosition(targetDegrees);
        hood.setPosition(position);
        estimatedReadyNanos = System.nanoTime() + secondsToNanos(
                (HOOD_MAX_POSITION - HOOD_MIN_POSITION) * 6.0
                        * HOOD_SECONDS_PER_60_DEGREES
                        + SERVO_SETTLE_MARGIN_SECONDS);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command goTo(double degrees) {
        return Commands.instant(() -> setAngle(degrees)).requiring(this);
    }

    public Command stow() {
        return goTo(HOOD_STOW_DEGREES);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    public void setAngle(double degrees) {
        if (!Double.isFinite(degrees)) return;

        double nextDegrees = clamp(degrees, 0.0, HOOD_MAX_DEGREES);
        double next = angleToPosition(nextDegrees);
        double servoTravelDegrees = Math.abs(next - position) * 360.0;
        if (Math.abs(nextDegrees - targetDegrees) >= COMMAND_EPSILON_DEGREES) {
            long travelNanos = secondsToNanos(
                    servoTravelDegrees / 60.0 * HOOD_SECONDS_PER_60_DEGREES
                            + SERVO_SETTLE_MARGIN_SECONDS);
            estimatedReadyNanos = Math.max(
                    estimatedReadyNanos,
                    System.nanoTime() + travelNanos);
        }
        targetDegrees = nextDegrees;
        position = next;
        hood.setPosition(position);
    }

    public double getAngle() {
        return targetDegrees;
    }

    public double getPosition() {
        return position;
    }

    /** Open-loop estimate only; no analog feedback is used. */
    public boolean isSettled() {
        return System.nanoTime() >= estimatedReadyNanos;
    }

    private static double angleToPosition(double degrees) {
        double servoDegrees = degrees * HOOD_GEAR_TEETH / HOOD_SERVO_GEAR_TEETH;
        return HOOD_MIN_POSITION + servoDegrees / 360.0;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static long secondsToNanos(double seconds) {
        return (long) (Math.max(0.0, seconds) * 1_000_000_000.0);
    }
}
