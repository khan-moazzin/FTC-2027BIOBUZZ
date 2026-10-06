package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.*;

public class Indexer {

    private final ServoImplEx indexer;
    private double position;

    public Indexer(HardwareMap hw) {
        indexer = hw.get(ServoImplEx.class, "indexer");
        indexer.setPwmRange(new PwmControl.PwmRange(AXON_PWM_MIN_US, AXON_PWM_MAX_US));
        indexer.setDirection(ServoImplEx.Direction.FORWARD);

        position = INDEXER_RETRACTED;
        indexer.setPosition(position);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command deploy() {
        return Command.build()
                .setStart(() -> setPosition(INDEXER_DEPLOYED))
                .setEnd(end -> setPosition(INDEXER_RETRACTED))
                .requiring(this);
    }

    public Command retract() {
        return Commands.instant(() -> setPosition(INDEXER_RETRACTED)).requiring(this);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    public void setPosition(double target) {
        if (!Double.isFinite(target)) return;

        position = Math.max(0.0, Math.min(1.0, target));
        indexer.setPosition(position);
    }

    public double getPosition() {
        return position;
    }
}
