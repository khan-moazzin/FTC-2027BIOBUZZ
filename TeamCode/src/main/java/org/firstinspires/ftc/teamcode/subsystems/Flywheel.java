package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Flywheel {

    private final DcMotorEx left;
    private final DcMotorEx right;
    private double targetRpm = 0.0;

    public Flywheel(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, "flywheel1");
        right = hw.get(DcMotorEx.class, "flywheel2");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);


        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        targetRpm = Math.max(0.0, rpm);
        double ticksPerSecond = targetRpm / 60.0 * FLYWHEEL_TICKS_PER_REV;
        left.setVelocity(ticksPerSecond);
        right.setVelocity(ticksPerSecond);
    }

    /** Gate for readyToShoot later. False whenever the wheel is commanded off. */
    public boolean atSpeed() {
        if (targetRpm <= 0.0) return false;
        return Math.abs(getRpm() - targetRpm) <= FLYWHEEL_TOLERANCE;
    }

    public double getRpm() {
        double avgTicksPerSecond = (left.getVelocity() + right.getVelocity()) / 2.0;
        return avgTicksPerSecond / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }
}