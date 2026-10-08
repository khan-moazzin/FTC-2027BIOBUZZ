package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "AutoMain", group = "Competition")
public final class AutoMain extends OpMode {
  private Robot robot;
  private AutoModeFactory.Mode mode = AutoModeFactory.Mode.THREE_TIP;
  private Command routine;
  private long telemetryAt;

  @Override
  public void init() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    robot = new Robot();
    robot.init(hardwareMap, telemetry);
    robot.driver1 = gamepad1;
    robot.driver2 = gamepad2;
  }

  @Override
  public void init_loop() {
    long now = System.nanoTime();
    robot.read(now);
    if (gamepad1.dpad_left) Constants.ALLIANCE = Constants.Alliance.BLUE;
    if (gamepad1.dpad_right) Constants.ALLIANCE = Constants.Alliance.RED;
    if (gamepad1.dpad_up) mode = AutoModeFactory.Mode.THREE_TIP;
    if (gamepad1.dpad_down) mode = AutoModeFactory.Mode.DO_NOTHING;
    telemetry.addData("Alliance (dpad left/right)", Constants.ALLIANCE);
    telemetry.addData("Auto (dpad up/down)", mode);
    telemetry.addData("Vision", robot.vision.status);
    telemetry.addData("Odometry", robot.drive.localizer().healthStatus);
    telemetry.addData("Logging", robot.logging.status());
    robot.logging.capture(robot, now, gamepad1, gamepad2);
    telemetry.update();
  }

  @Override
  public void start() {
    routine = AutoModeFactory.create(mode, robot, Constants.ALLIANCE);
    robot.logging.mode("AUTO");
    CommandScheduler.getInstance().schedule(routine);
  }

  @Override
  public void loop() {
    long now = System.nanoTime();
    robot.read(now);
    CommandScheduler.getInstance().run();
    robot.write();
    if (now - telemetryAt > 100000000L) {
      robot.sendTelemetry();
      telemetry.addData("Auto", mode);
      telemetry.addData("Alliance", Constants.ALLIANCE);
      telemetry.update();
      telemetryAt = now;
    }
  }

  @Override
  public void stop() {
    if (robot != null) robot.close();
  }
}
