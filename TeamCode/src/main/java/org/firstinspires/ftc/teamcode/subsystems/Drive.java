package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;


public class Drive {

    private final FusionLocalizer localizer;
    private final Follower follower;

    private double headingOffset = 0.0;

    public Drive(HardwareMap hw) {
        localizer = Constants.createLocalizer(hw);
        follower = Constants.createFollower(hw, localizer);
    }

    public void addVisionMeasurement(Pose pose, long timestampNanos) {
        localizer.addMeasurement(pose, timestampNanos);
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
    /** Field-oriented drive. Heading comes from the Pinpoint through the localizer. */
    public void drive(double axial, double lateral, double yaw) {
        follower.manual(ManualDrive.fieldCentric(
                axial, lateral, yaw, follower.pose().heading(), -driverForward()));
    }

    private double driverForward() {
        return Math.toRadians(Constants.driverForwardDegrees()) + headingOffset;
    }

    public void resetHeading() {
        headingOffset = follower.pose().heading() - Math.toRadians(Constants.driverForwardDegrees());
    }

    public void clearHeadingTrim() {
        headingOffset = 0.0;
    }

}
