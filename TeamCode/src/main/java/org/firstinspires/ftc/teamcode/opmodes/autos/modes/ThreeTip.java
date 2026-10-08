package org.firstinspires.ftc.teamcode.opmodes.autos.modes;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.autos.paths.ThreeTipPath;
import org.firstinspires.ftc.teamcode.opmodes.autos.actions.SetIntakeAction;
import org.firstinspires.ftc.teamcode.opmodes.autos.actions.ShootAction;

/** Full preload, collection, shooting and park routine from the Pedro-generated route. */
public final class ThreeTip extends SequentialCommandGroup {
  public ThreeTip(Robot robot, ThreeTipPath paths) {
    super(commands(robot, paths));
  }

  private static Command[] commands(Robot robot, ThreeTipPath paths) {
    return new Command[] {
      new ShootAction(robot, 4.0),
      new SetIntakeAction(robot.intake, true),
      robot.drive.follow(paths.intake1),
      new SetIntakeAction(robot.intake, false),
      robot.drive.follow(paths.intake2Prep),
      new SetIntakeAction(robot.intake, true),
      robot.drive.follow(paths.intake2Flower),
      new ShootAction(robot, 4.5),
      new SetIntakeAction(robot.intake, true),
      robot.drive.follow(paths.intake3Prep),
      robot.drive.follow(paths.intake3),
      new SetIntakeAction(robot.intake, false),
      robot.drive.follow(paths.shoot1),
      new ShootAction(robot, 4.0),
      new SetIntakeAction(robot.intake, true),
      robot.drive.follow(paths.intake4Flower),
      robot.drive.follow(paths.intake4),
      robot.drive.follow(paths.shoot2),
      new ShootAction(robot, 4.0),
      new SetIntakeAction(robot.intake, false),
      robot.drive.follow(paths.park, 1.0)
    };
  }
}
