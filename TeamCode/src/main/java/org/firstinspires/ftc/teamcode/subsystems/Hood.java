package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.*;

/** One servo, model TBD. Closes its own position loop, so no PID here.
 *  If it is not a 500-2500us servo, setPosition() will not reach full travel and it
 *  needs ServoImplEx.setPwmRange. */

public class Hood {

    private final Servo hood;

    private double position;

    public Hood(HardwareMap hw) {
        hood = hw.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);

        position = HOOD_STOW;
        hood.setPosition(position);
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
        position = clamp(target, HOOD_MIN, HOOD_MAX);
        hood.setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}