package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Intake {

    private final DcMotor motor;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotor.class, "intake");
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------

    public Command intake() {
        return Command.build()
                .setStart(() -> motor.setPower(INTAKE))
                .setEnd(end -> motor.setPower(INTAKE_IDLE))
                .requiring(this);
    }

    public Command outtake() {
        return Command.build()
                .setStart(() -> motor.setPower(OUTTAKE))
                .setEnd(end -> motor.setPower(INTAKE_IDLE))
                .requiring(this);
    }

    public Command stop() {
        return Commands.instant(() -> motor.setPower(INTAKE_IDLE)).requiring(this);
    }

    // -----------------------------------------------------
    // STATE
    // -----------------------------------------------------
    public double getPower() {
        return motor.getPower();
    }
}