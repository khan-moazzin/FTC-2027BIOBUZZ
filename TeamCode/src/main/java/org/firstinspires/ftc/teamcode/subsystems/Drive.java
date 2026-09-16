package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;


public class Drive {

    private final Follower follower;

    private double headingOffset = 0.0;
    private boolean robotOriented = false;

    public Drive(HardwareMap hw) {
        follower = Constants.createFollower(hw);
    }

    public void update() {
        follower.update();
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    // -----------------------------------------------------
    // COMMANDS
    // -----------------------------------------------------
    public Command teleopDrive(DoubleSupplier axial, DoubleSupplier lateral, DoubleSupplier yaw) {
        return Commands.infinite(() ->
                        drive(axial.getAsDouble(), lateral.getAsDouble(), yaw.getAsDouble()))
                .requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    // -----------------------------------------------------
    // DIRECT CONTROL
    // -----------------------------------------------------
    /** Heading comes from the Pinpoint through the localizer. */
    public void drive(double axial, double lateral, double yaw) {
        if (robotOriented) {
            follower.manual(axial, lateral, yaw);
        } else {
            follower.manual(ManualDrive.fieldCentric(
                    axial, lateral, yaw, follower.pose().heading(), -headingOffset));
        }
    }

    public void resetHeading() {
        headingOffset = follower.pose().heading();
    }

    public void toggleRobotOriented() {
        robotOriented = !robotOriented;
    }

    public boolean isRobotOriented() {
        return robotOriented;
    }
}