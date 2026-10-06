package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.ShooterConstants.*;

/** Axon MINI MK2 with a 30:173 reduction. */
public class Hood {

    private static final double COMMAND_EPSILON = 0.005;

    private final ServoImplEx hood;

    private double position;
    private long estimatedReadyNanos;

    public Hood(HardwareMap hw) {
        hood = hw.get(ServoImplEx.class, "hood");
        hood.setPwmRange(new PwmControl.PwmRange(AXON_PWM_MIN_US, AXON_PWM_MAX_US));
        hood.setDirection(ServoImplEx.Direction.FORWARD);

        position = HOOD_STOW;
        hood.setPosition(position);
        estimatedReadyNanos = System.nanoTime() + secondsToNanos(
                HOOD_SECONDS_PER_60_DEGREES + SERVO_SETTLE_MARGIN_SECONDS);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command goTo(double target) {
        return Commands.instant(() -> setPosition(target)).requiring(this);
    }

    public Command stow() {
        return goTo(HOOD_STOW);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    public void setPosition(double target) {
        if (!Double.isFinite(target)) return;

        double next = clamp(target, HOOD_MIN, HOOD_MAX);
        double servoTravelDegrees = Math.abs(next - position) * 360.0;
        if (Math.abs(next - position) >= COMMAND_EPSILON) {
            long travelNanos = secondsToNanos(
                    servoTravelDegrees / 60.0 * HOOD_SECONDS_PER_60_DEGREES
                            + SERVO_SETTLE_MARGIN_SECONDS);
            estimatedReadyNanos = Math.max(
                    estimatedReadyNanos,
                    System.nanoTime() + travelNanos);
        }
        position = next;
        hood.setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    /** Open-loop estimate only; no analog feedback is used. */
    public boolean isSettled() {
        return System.nanoTime() >= estimatedReadyNanos;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static long secondsToNanos(double seconds) {
        return (long) (Math.max(0.0, seconds) * 1_000_000_000.0);
    }
}
