package org.firstinspires.ftc.teamcode.lib.pedro.examples;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.seattlesolvers.solverslib.command.*;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Disabled
@Autonomous(name = "Pedro 3 - Solvers Path Demo", group = "Pedro Examples")
public final class Pedro3Auto extends OpMode {
  private Drive drive;
  private Command routine;

  public void init() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().reset();
    drive = new Drive(hardwareMap);
    DemoPaths p = new DemoPaths(false);
    drive.setPose(p.start);
    routine =
        new SequentialCommandGroup(
            drive.follow(p.outbound), new WaitCommand(500), drive.follow(p.home));
  }

  public void start() {
    CommandScheduler.getInstance().schedule(routine);
  }

  public void loop() {
    drive.read(System.nanoTime());
    CommandScheduler.getInstance().run();
    drive.update();
  }

  public void stop() {
    CommandScheduler.getInstance().cancelAll();
    if (drive != null) drive.stop();
    CommandScheduler.getInstance().reset();
  }
}
