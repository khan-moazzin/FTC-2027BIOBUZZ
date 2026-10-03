package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Intake {

    private final DcMotor intake1;
    private final DcMotor intake2;

    public Intake(HardwareMap hw) {
        intake1 = hw.get(DcMotor.class, "intake1");
        intake2 = hw.get(DcMotor.class, "intake2");

        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command intake() {
        return Command.build()
                .setStart(() -> set(INTAKE))
                .setEnd(end -> set(INTAKE_IDLE))
                .requiring(this);
    }

    public Command outtake() {
        return Command.build()
                .setStart(() -> set(OUTTAKE))
                .setEnd(end -> set(INTAKE_IDLE))
                .requiring(this);
    }

    // -----------------------------------------------------
    // STATE
    // -----------------------------------------------------
    private void set(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public double getPower() {
        return intake1.getPower();
    }
}
