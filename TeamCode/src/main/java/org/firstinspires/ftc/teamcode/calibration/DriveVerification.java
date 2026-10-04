package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.Angles;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "Tune 0 - Drive and Pinpoint verification", group = "Calibration")
public final class DriveVerification extends GuidedOpMode {
  private Drive drive;

  protected void setup() {
    drive = new Drive(hardwareMap);
  }

  protected void tick(long now) {
    drive.read(now);
    if (a) drive.setPose(Pose.zero());
    telemetry.addLine(
        "A zeroes. Push exactly 48 inches forward/left and rotate 360 degrees; compare pose.");
    telemetry.addLine(
        "For automated coefficients run Pedro AutoTune: Mecanum, Pinpoint CUSTOM (SWYFT), then"
            + " Foresight.");
    telemetry.addLine(
        "Hold RB for 25% robot-centric motion. Verify each axis/sign before path tuning.");
    if (gamepad1.right_bumper)
      drive
          .getFollower()
          .manual(
              -.25 * Angles.deadband(gamepad1.left_stick_y),
              -.25 * Angles.deadband(gamepad1.left_stick_x),
              -.25 * Angles.deadband(gamepad1.right_stick_x));
    else drive.getFollower().stop();
    drive.update();
    telemetry.addData("Pose (inches/radians)", drive.getPose());
    telemetry.addData("Velocity", drive.localizer().velocity());
  }

  protected void shutdown() {
    if (drive != null) drive.stop();
  }
}
