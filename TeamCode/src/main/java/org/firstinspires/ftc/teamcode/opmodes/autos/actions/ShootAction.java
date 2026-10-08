package org.firstinspires.ftc.teamcode.opmodes.autos.actions;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot;

/** Readiness-gated autonomous shooting request with a bounded total window. */
public final class ShootAction extends CommandBase {
  private final Robot robot;
  private final long durationNanos;
  private long started;

  public ShootAction(Robot robot, double durationSeconds) {
    if (!Double.isFinite(durationSeconds) || durationSeconds <= 0)
      throw new IllegalArgumentException("Shot duration must be positive and finite");
    this.robot = robot;
    durationNanos = (long) (durationSeconds * 1e9);
    addRequirements(
        robot.shooter,
        robot.intake,
        robot.indexer,
        robot.turret,
        robot.hood,
        robot.flywheel);
  }

  @Override
  public void initialize() {
    started = System.nanoTime();
  }

  @Override
  public void execute() {
    robot.shooter.runAutoShot(System.nanoTime());
  }

  @Override
  public boolean isFinished() {
    return System.nanoTime() - started >= durationNanos;
  }

  @Override
  public void end(boolean interrupted) {
    robot.shooter.stopAutoShot(System.nanoTime());
  }
}
