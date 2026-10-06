package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
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
                .setExecute(() -> setPower(INTAKE))
                .setEnd(end -> setPower(INTAKE_IDLE))
                .requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    public Command outtake() {
        return Command.build()
                .setExecute(() -> setPower(OUTTAKE))
                .setEnd(end -> setPower(INTAKE_IDLE))
                .requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    // -----------------------------------------------------
    // STATE
    // -----------------------------------------------------
    public void setPower(double power) {
        if (!Double.isFinite(power)) power = INTAKE_IDLE;
        power = Math.max(-1.0, Math.min(1.0, power));
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public double getPower() {
        return intake1.getPower();
    }
}
