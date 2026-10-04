package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.*;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.vision.HiveState;

public final class Shooter extends SubsystemBase {
  private final Robot robot;
  private final ShotConfig config = new ShotConfig();
  private Field.Cell selection;
  private boolean cancelled;
  private long lastTime;
  public MovingShotSolver.Solution solution = new MovingShotSolver.Solution();
  public String status = "Idle";
  public boolean ready;
  private double overflow;

  public Shooter(Robot r) {
    robot = r;
  }

  public Command driverControl(Gamepad g1, Gamepad g2) {
    return new RunCommand(
        () -> control(g1, g2, System.nanoTime()),
        this,
        robot.intake,
        robot.turret,
        robot.hood,
        robot.flywheel);
  }

  private void control(Gamepad g1, Gamepad g2, long now) {
    double dt = lastTime == 0 ? .02 : Math.min(.1, (now - lastTime) * 1e-9);
    lastTime = now;
    if (g2.a) selection = null;
    if (g2.x) selection = Field.Cell.AUDIENCE;
    if (g2.y) selection = Field.Cell.SCORING;
    if (g2.b) cancelled = true;
    else if (g2.left_trigger < .1 && g2.right_trigger < .1) cancelled = false;
    boolean prepare = !cancelled && (g2.left_trigger > .5 || g2.right_trigger > .5);
    ready = false;
    overflow = 0;
    if (prepare) {
      Field.Hive alliance =
          Constants.ALLIANCE == Constants.Alliance.RED ? Field.Hive.RED : Field.Hive.BLUE;
      HiveState hive = robot.vision.hive(alliance);
      solution =
          MovingShotSolver.solve(
              robot.drive.getPose(),
              robot.drive.localizer().velocity(),
              robot.turret.feedback.angle,
              robot.turret.feedback.velocity,
              hive,
              alliance,
              selection == null ? Field.raised(hive.angle(now)) : selection,
              now,
              robot.visionConfig,
              config);
      if (solution.valid) {
        overflow = robot.turret.aim(solution.angle, solution.angularVelocity, dt);
        robot.flywheel.setTargetRpm(solution.rpm);
        robot.hood.setAngle(solution.hood);
        double hoodPosition =
            MechanismConfig.hoodZero + solution.hood / MechanismConfig.hoodRadiansPerUnit;
        ready =
            robot.visionConfig.calibrated
                && solution.rpm <= MechanismConfig.maxRpm
                && robot.drive.localizer().positionSigma() <= robot.visionConfig.maxPoseSigma
                && Double.isFinite(overflow)
                && Math.abs(overflow) < MechanismConfig.turretTolerance
                && robot.turret.ready()
                && robot.flywheel.atSpeed()
                && robot.hood.ready(now)
                && hoodPosition >= MechanismConfig.hoodMin
                && hoodPosition <= MechanismConfig.hoodMax;
        status = ready ? "Ready" : "Waiting for mechanisms/pose";
      } else {
        robot.flywheel.setTargetRpm(0);
        robot.turret.hold();
        status = solution.reason;
      }
    } else {
      robot.flywheel.setTargetRpm(0);
      robot.turret.hold();
      robot.hood.stow();
      status = cancelled ? "Cancelled: release triggers" : "Idle";
      solution.valid = false;
    }
    // Reverse always wins. Feeding cannot bypass shot readiness.
    robot.intake.set(
        g1.left_bumper
            ? -.9
            : (prepare
                ? (g2.right_trigger > .5 && ready ? 1 : 0)
                : Intake.requested(g1.left_trigger > .5, false)));
  }

  public double yaw(double manual, boolean assist) {
    if (Math.abs(manual) > .001 || !assist || !solution.valid) return manual;
    double error = Math.abs(overflow) > MechanismConfig.turretTolerance ? overflow : solution.angle;
    return Double.isFinite(error)
        ? Angles.clamp(
            error * MechanismConfig.assistP, -MechanismConfig.assistMax, MechanismConfig.assistMax)
        : 0;
  }
}
