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

    /** Manual trim on top of the alliance heading, for when localization is lost. */
    private double headingOffset = 0.0;
    private boolean robotOriented = false;
    private boolean fieldOriented = false;

    public Drive(HardwareMap hw) {
        localizer = Constants.createLocalizer(hw);
        follower = Constants.createFollower(hw, localizer);
    }

    /** Vision pose in, timestamp in System.nanoTime() minus camera latency. */
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
    /** Heading comes from the Pinpoint through the localizer. */
    public void drive(double axial, double lateral, double yaw) {
        if (robotOriented) {
            follower.manual(axial, lateral, yaw);
        } else {
            follower.manual(ManualDrive.fieldCentric(
                    axial, lateral, yaw, follower.pose().heading(), -driverForward()));
        }
    }

    /** Driver forward in field radians: alliance heading plus manual trim. */
    private double driverForward() {
        return Math.toRadians(Constants.driverForwardDegrees()) + headingOffset;
    }

    /** Declares the robot's current facing to be driver-forward. Escape hatch if pose drifts. */
    public void resetHeading() {
        headingOffset = follower.pose().heading() - Math.toRadians(Constants.driverForwardDegrees());
    }

    /** Clears the manual trim, back to pure alliance-relative. */
    public void clearHeadingTrim() {
        headingOffset = 0.0;
    }

    public void toggleFieldOriented(){
        fieldOriented =! fieldOriented;
    }


    public void toggleRobotOriented() {
        robotOriented = !robotOriented;
    }

    public boolean isRobotOriented() {
        return robotOriented;
    }
}