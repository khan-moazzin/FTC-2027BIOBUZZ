package org.firstinspires.ftc.teamcode.pedro.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Constants;

/** Remove @Disabled after confirming drivetrain directions and Pinpoint tuning. */
@Disabled
@TeleOp(name = "Pedro 3 - Field Centric Demo", group = "Pedro Examples")
public class Pedro3TeleOp extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap)
                .withLogger(log -> telemetry.addData("Pedro", log.toString()));
        // Heading zero is the robot's forward direction at initialization.
        follower.setPose(Pose.zero());
        telemetry.addLine("Hold left bumper for robot-centric controls. Sticks drive/turn.");
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            follower.manual(forward, lateral, turn);
        } else {
            follower.manual(ManualDrive.fieldCentric(
                    forward, lateral, turn, follower.pose().heading()));
        }
        follower.update();
        telemetry.addData("Drive mode", gamepad1.left_bumper ? "Robot centric" : "Field centric");
        telemetry.addData("Pose", follower.pose());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
    }
}
