package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.*;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.vision.HiveState;

public final class Shooter extends SubsystemBase {
  private final Robot robot;
  private final ShotConfig config = new ShotConfig();
  public final PhysicsShotConfig physics = new PhysicsShotConfig();
  private final StableHiveGate redGate = new StableHiveGate(), blueGate = new StableHiveGate();
  public boolean redStable, blueStable;
  private Field.Cell selection;
  private boolean cancelled;
  private long lastTime;
  private final Readiness shotGate = new Readiness();
  public boolean dwellReady;
  public ShotSolution solution = new ShotSolution();
  public String status = "Idle";
  public boolean ready, preparing, feedRequested, readinessEvaluated;
  // Bitmask: calibration, RPM range, pose, reachability, turret, flywheels, hood settle, hood
  // range.
  public int readinessBlockers = 1023;

  public String selectedCell() {
    return selection == null ? "AUTO_RAISED" : selection.name();
  }

  public boolean cancelled() {
    return cancelled;
  }

  public double overflow() {
    return overflow;
  }

  private double overflow;

  public Shooter(Robot r) {
    robot = r;
  }

  public Command driverControl(Gamepad g1, Gamepad g2) {
    return new RunCommand(
        () -> control(g1, g2, System.nanoTime()),
        this,
        robot.intake,
        robot.indexer,
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
    redStable = stable(robot.vision.red, redGate, now);
    blueStable = stable(robot.vision.blue, blueGate, now);
    boolean prepare = !cancelled && (g2.left_trigger > .5 || g2.right_trigger > .5);
    preparing = prepare;
    feedRequested = g2.right_trigger > .5;
    readinessBlockers = 1023;
    readinessEvaluated = false;
    ready = false;
    dwellReady = false;
    overflow = 0;
    if (prepare && !robot.drive.localizer().healthy(now)) {
      solution.valid = false;
      robot.flywheel.setTargetRpm(0);
      robot.turret.hold();
      robot.hood.stow();
      status = robot.drive.localizer().healthStatus;
    } else if (prepare) {
      Field.Hive alliance =
          Constants.ALLIANCE == Constants.Alliance.RED ? Field.Hive.RED : Field.Hive.BLUE;
      HiveState hive = robot.vision.hive(alliance);
      solution =
          StableShotSolver.solve(
              robot.drive.getPose(),
              robot.drive.localizer().velocity(),
              robot.turret.feedback.angle,
              robot.turret.feedback.velocity,
              hive,
              alliance,
              selection == null ? Field.raised(hive.angle(now)) : selection,
              now,
              robot.visionConfig,
              config,
              physics,
              alliance == Field.Hive.RED ? redStable : blueStable);
      if (solution.valid) {
        overflow = robot.turret.aim(solution.angle, solution.angularVelocity, dt);
        robot.flywheel.setTargetRpm(solution.rpm);
        robot.hood.setAngle(solution.hood);
        double hoodPosition =
            MechanismConfig.hoodZero + solution.hood / MechanismConfig.hoodRadiansPerUnit;
        readinessEvaluated = true;
        readinessBlockers =
            (robot.visionConfig.calibrated ? 0 : 1)
                | (solution.rpm <= MechanismConfig.maxRpm ? 0 : 2)
                | (robot.drive.localizer().positionSigma() <= robot.visionConfig.maxPoseSigma
                    ? 0
                    : 4)
                | (Double.isFinite(overflow) && Math.abs(overflow) < MechanismConfig.turretTolerance
                    ? 0
                    : 8)
                | (robot.turret.ready() ? 0 : 16)
                | (robot.flywheel.atSpeed() ? 0 : 32)
                | (robot.hood.ready(now) ? 0 : 64)
                | (MechanismConfig.indexerCalibrated ? 0 : 256)
                | (Readiness.motionAllowed(
                        MechanismConfig.stationaryShotsOnly,
                        robot.drive.localizer().velocity().vx,
                        robot.drive.localizer().velocity().vy,
                        robot.drive.localizer().velocity().omega,
                        MechanismConfig.maxShotTranslation,
                        MechanismConfig.maxShotRotation)
                    ? 0
                    : 512)
                | (hoodPosition >= MechanismConfig.hoodMin
                        && hoodPosition <= MechanismConfig.hoodMax
                    ? 0
                    : 128);
        dwellReady = shotGate.update(readinessBlockers == 0, now, MechanismConfig.shotReadySeconds);
        ready = readinessBlockers == 0 && dwellReady;
        status =
            ready
                ? "Ready"
                : readinessBlockers == 0 ? "Readiness dwell" : "Waiting for mechanisms/pose";
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
    if (!prepare || !solution.valid || readinessBlockers != 0) shotGate.update(false, now, 0);
    // Reverse retracts the indexer even when a valid shooting request is held.
    boolean feed = Readiness.feed(prepare, feedRequested, ready, g1.left_bumper);
    robot.indexer.feed(feed);
    robot.intake.set(
        g1.left_bumper
            ? -.9
            : prepare ? (feed ? 1 : 0) : Intake.requested(g1.left_trigger > .5, false));
  }

  private boolean stable(HiveState hive, StableHiveGate gate, long now) {
    double variance = hive.variance(now, robot.visionConfig.hiveAccelerationNoise);
    boolean fresh =
        hive.fresh(now, robot.visionConfig.hiveMaxAge)
            && Double.isFinite(variance)
            && Math.sqrt(variance) <= robot.visionConfig.maxHiveSigma;
    return gate.update(hive.angle(now), hive.rate(), fresh, hive.observationTime(), now, physics);
  }

  public double yaw(double manual, boolean assist) {
    if (Math.abs(manual) > .001
        || !assist
        || !solution.valid
        || !robot.drive.localizer().healthy(System.nanoTime())) return manual;
    double error = Math.abs(overflow) > MechanismConfig.turretTolerance ? overflow : solution.angle;
    return Double.isFinite(error)
        ? Angles.clamp(
            error * MechanismConfig.assistP, -MechanismConfig.assistMax, MechanismConfig.assistMax)
        : 0;
  }
}
