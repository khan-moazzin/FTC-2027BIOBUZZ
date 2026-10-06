package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.ShooterConstants.*;

public class Flywheel {

    private final DcMotorEx shooter;
    private double targetRpm = 0.0;

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
    public Command spin(double rpm) {
        return Command.build()
                .setStart(() -> setTargetRpm(rpm))
                .setEnd(end -> setTargetRpm(0.0))
                .requiring(this);
    }

    public Command stop() {
        return Commands.instant(() -> setTargetRpm(0.0)).requiring(this);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    public void setTargetRpm(double rpm) {
        if (!Double.isFinite(rpm)) rpm = 0.0;

        targetRpm = clamp(rpm, 0.0, FLYWHEEL_MAX_RPM);
        double ticksPerSecond = targetRpm / 60.0 * FLYWHEEL_TICKS_PER_REV;
        shooter.setVelocity(ticksPerSecond);
    }

    /** Gate for readyToShoot later. False whenever the wheel is commanded off. */
    public boolean atSpeed() {
        if (targetRpm <= 0.0) return false;
        double rpm = getRpm();
        return Double.isFinite(rpm) && Math.abs(rpm - targetRpm) <= FLYWHEEL_TOLERANCE_RPM;
    }

    public double getRpm() {
        return ticksPerSecondToRpm(shooter.getVelocity());
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
