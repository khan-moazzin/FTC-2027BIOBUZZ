package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Flywheel {

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private double targetRpm = 0.0;

    public Flywheel(HardwareMap hw) {
        leftShooter = hw.get(DcMotorEx.class, "flywheel1");
        rightShooter = hw.get(DcMotorEx.class, "flywheel2");

        leftShooter.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);


        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        targetRpm = Math.max(0.0, rpm);
        double ticksPerSecond = targetRpm / 60.0 * FLYWHEEL_TICKS_PER_REV;
        leftShooter.setVelocity(ticksPerSecond);
        rightShooter.setVelocity(ticksPerSecond);
    }

    /** Gate for readyToShoot later. False whenever the wheel is commanded off. */
    public boolean atSpeed() {
        if (targetRpm <= 0.0) return false;
        double leftRpm = getLeftRpm();
        double rightRpm = getRightRpm();
        return Double.isFinite(leftRpm)
                && Double.isFinite(rightRpm)
                && Math.abs(leftRpm - targetRpm) <= FLYWHEEL_TOLERANCE
                && Math.abs(rightRpm - targetRpm) <= FLYWHEEL_TOLERANCE;
    }

    public double getRpm() {
        return (getLeftRpm() + getRightRpm()) / 2.0;
    }

    public double getLeftRpm() {
        return ticksPerSecondToRpm(leftShooter.getVelocity());
    }

    public double getRightRpm() {
        return ticksPerSecondToRpm(rightShooter.getVelocity());
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond / FLYWHEEL_TICKS_PER_REV * 60.0;
    }
}
