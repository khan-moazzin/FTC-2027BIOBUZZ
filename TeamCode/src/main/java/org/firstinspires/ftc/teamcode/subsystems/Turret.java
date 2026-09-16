package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * VERIFY the two servos are set to opposite directions below,
 * which is correct if they are mounted facing each other. If they are mounted
 * the same way, set both FORWARD or they will fight and stall.
 */

public class Turret {

    private final Servo left;
    private final Servo right;

    private double position;

    public Turret(HardwareMap hw) {
        left = hw.get(Servo.class, "turret1");
        right = hw.get(Servo.class, "turret2");

        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        position = TURRET_CENTER;
        apply(position);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command goTo(double target) {
        return Commands.instant(() -> setPosition(target)).requiring(this);
    }

    public Command center() {
        return goTo(TURRET_CENTER);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    public void setPosition(double target) {
        position = clamp(target, TURRET_MIN, TURRET_MAX);
        apply(position);
    }

    public double getPosition() {
        return position;
    }

    private void apply(double p) {
        left.setPosition(p);
        right.setPosition(p);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}