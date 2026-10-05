package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.*;
import java.io.*;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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
    telemetry.addData("Odometry", robot.drive.localizer().healthStatus);
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
      telemetry.addData("Odometry", robot.drive.localizer().healthStatus);
      telemetry.addData("Flywheel volts", robot.flywheel.getVoltage());
      telemetry.update();
      telemetryAt = now;
    }
  }

  public void stop() {
    if (robot == null) return;
    try {
      robot.close();
    } finally {
      // Bounded export after actuator shutdown; overwrites the previous TeleOp trace.
      try {
        File directory = new File(AppUtil.FIRST_FOLDER, "biobuzz-logs");
        if (!directory.isDirectory() && !directory.mkdirs())
          throw new IOException("Cannot create " + directory);
        try (Writer writer =
            new OutputStreamWriter(
                new FileOutputStream(new File(directory, "teleop-loop.csv")), "UTF-8")) {
          robot.timing.writeCsv(writer);
        }
      } catch (IOException e) {
        RobotLog.ee("BioBuzz", e, "Could not export loop timing");
      }
    }
  }
}
