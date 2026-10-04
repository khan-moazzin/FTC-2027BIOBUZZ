package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.*;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.Angles;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleopMain", group = "Competition")
public final class TeleOp extends OpMode {
  private Robot robot;
  private boolean lastBack, lastB;
  private long telemetryAt;

  public void init() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    robot = new Robot();
    robot.init(hardwareMap, telemetry);
    robot.drive.setDefaultCommand(
        robot.drive.teleopDrive(
            () -> -Angles.deadband(gamepad1.left_stick_y),
            () -> -Angles.deadband(gamepad1.left_stick_x),
            () ->
                robot.shooter.yaw(
                    -.6 * Angles.deadband(gamepad1.right_stick_x), gamepad1.right_bumper)));
    robot.shooter.setDefaultCommand(robot.shooter.driverControl(gamepad1, gamepad2));
  }

  public void init_loop() {
    robot.read(System.nanoTime());
    if (gamepad1.dpad_left) Constants.ALLIANCE = Constants.Alliance.BLUE;
    if (gamepad1.dpad_right) Constants.ALLIANCE = Constants.Alliance.RED;
    telemetry.addData("Alliance (dpad left/right)", Constants.ALLIANCE);
    telemetry.addData("Initial pose", robot.drive.getPose());
    telemetry.addData("Vision", robot.vision.status);
    telemetry.update();
  }

  public void loop() {
    long now = System.nanoTime();
    robot.read(now);
    if (gamepad1.back && !lastBack) robot.drive.resetHeading();
    lastBack = gamepad1.back;
    if (gamepad1.b && !lastB) robot.drive.toggleRobotOriented();
    lastB = gamepad1.b;
    CommandScheduler.getInstance().run();
    robot.write();
    if (now - telemetryAt > 100000000L) {
      robot.sendTelemetry();
      telemetry.update();
      telemetryAt = now;
    }
  }

  public void stop() {
    if (robot != null) robot.close();
  }
}
