package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.*;

public class Flywheel {

    private final DcMotorEx shooter;
    private double targetPercent = 0.0;

    public Flywheel(HardwareMap hw) {
        shooter = hw.get(DcMotorEx.class, "flywheel");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command spin(double percent) {
        return Command.build()
                .setStart(() -> setSpeed(percent))
                .setEnd(end -> setSpeed(0.0))
                .requiring(this);
    }

    public Command stop() {
        return Commands.instant(() -> setSpeed(0.0)).requiring(this);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    public void setSpeed(double percent) {
        if (!Double.isFinite(percent)) percent = 0.0;

        targetPercent = clamp(percent, 0.0, FLYWHEEL_MAX_PERCENT);
        double ticksPerSecond = percentToRpm(targetPercent)
                / 60.0 * FLYWHEEL_TICKS_PER_REV;
        shooter.setVelocity(ticksPerSecond);
    }

    /** False whenever the flywheel is off or outside its allowed speed error. */
    public boolean atSpeed() {
        if (targetPercent <= 0.0) return false;
        double percent = getSpeedPercent();
        return Double.isFinite(percent)
                && Math.abs(percent - targetPercent) <= FLYWHEEL_TOLERANCE_PERCENT;
    }

    public double getRpm() {
        return ticksPerSecondToRpm(shooter.getVelocity());
    }

    public double getSpeedPercent() {
        return getRpm() / FLYWHEEL_MAX_RPM * FLYWHEEL_MAX_PERCENT;
    }

    public double getTargetPercent() {
        return targetPercent;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private static double percentToRpm(double percent) {
        return percent / FLYWHEEL_MAX_PERCENT * FLYWHEEL_MAX_RPM;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
