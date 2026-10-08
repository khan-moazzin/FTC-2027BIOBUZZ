package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.autos.modes.ThreeTip;
import org.firstinspires.ftc.teamcode.opmodes.autos.paths.ThreeTipPath;

/** Creates autonomous routines from the selection finalized during INIT. */
public final class AutoModeFactory {
  private AutoModeFactory() {}

  public enum Mode {
    THREE_TIP,
    DO_NOTHING
  }

  public static Command create(Mode mode, Robot robot, Constants.Alliance alliance) {
    switch (mode) {
      case THREE_TIP:
        ThreeTipPath paths = new ThreeTipPath(alliance);
        robot.drive.setPose(paths.start);
        return new ThreeTip(robot, paths);
      case DO_NOTHING:
      default:
        return new InstantCommand();
    }
  }
}
