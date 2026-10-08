package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.control.LoopTiming;
import org.firstinspires.ftc.teamcode.lib.vision.BiobuzzVision;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;

public final class Robot implements AutoCloseable {
  public Shooter shooter;
  public Drive drive;
  public Intake intake;
  public Kickup kickup;
  public Indexer indexer;
  public Turret turret;
  public Hood hood;
  public Flywheel flywheel;
  public BiobuzzVision vision;
  public final VisionConfig visionConfig = new VisionConfig();
  public final LoopTiming timing = new LoopTiming();
  public org.firstinspires.ftc.teamcode.lib.logging.RobotLogging logging;
  public com.qualcomm.robotcore.hardware.Gamepad driver1, driver2;
  private long cycleTime;
  private List<LynxModule> hubs;
  private Telemetry telemetry;

  public void init(HardwareMap hw, Telemetry t) {
    try {
      telemetry = t;
      hubs = hw.getAll(LynxModule.class);
      for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      drive = new Drive(hw);
      intake = new Intake(hw);
      kickup = new Kickup(hw);
      indexer = new Indexer(hw);
      turret = new Turret(hw, visionConfig);
      hood = new Hood(hw);
      flywheel = new Flywheel(hw);
      vision = new BiobuzzVision(hw, visionConfig);
      shooter = new Shooter(this);
      logging = new org.firstinspires.ftc.teamcode.lib.logging.RobotLogging(this);
    } catch (RuntimeException failure) {
      try {
        close();
      } catch (RuntimeException cleanup) {
        failure.addSuppressed(cleanup);
      }
      throw failure;
    }
  }

  public void read(long now) {
    cycleTime = now;
    timing.start(now);
    for (LynxModule h : hubs) h.clearBulkCache();
    turret.read(now);
    flywheel.read(now);
    drive.read(now);
    vision.update(
        now,
        turret.feedback.healthy ? turret.feedback.angle : Double.NaN,
        turret.feedback.velocity,
        drive.localizer());
    timing.readDone();
  }

  public void write() {
    long started = System.nanoTime();
    drive.update();
    indexer.write();
    intake.write();
    kickup.write();
    turret.write();
    hood.write();
    flywheel.write();
    timing.finish(started);
    if (logging != null) logging.capture(this, cycleTime, driver1, driver2);
  }

  public void sendTelemetry() {
    telemetry.addData("Loop", timing.summary());
    if (logging != null) {
      telemetry.addData("Logging", logging.status());
      telemetry.addData("Log file", logging.path());
    }
    telemetry.addData("Shooter", shooter.status);
    telemetry.addData("Vision", vision.status);
    telemetry.addData("Pose", drive.getPose());
    telemetry.addData("Turret feedback", turret.feedback.healthy);
    telemetry.addData("Flywheel RPM", flywheel.getRpm());
    telemetry.addData(
        "Kickup",
        MechanismConfig.kickupCalibrated ? (kickup.isUp() ? "UP" : "DOWN") : "UNCALIBRATED");
  }

  public void close() {
    RuntimeException failure = null;
    Runnable[] stops = {
      () -> CommandScheduler.getInstance().cancelAll(),
      () -> {
        if (intake != null) intake.stop();
      },
      () -> {
        if (kickup != null) kickup.stop();
      },
      () -> {
        if (indexer != null) indexer.retract();
      },
      () -> {
        if (flywheel != null) flywheel.stop();
      },
      () -> {
        if (drive != null) drive.stop();
      },
      () -> {
        if (turret != null) turret.hold();
      },
      () -> {
        if (hood != null) hood.stow();
      },
      () -> {
        if (vision != null) vision.close();
      },
      () -> CommandScheduler.getInstance().reset(),
      () -> {
        if (logging != null) {
          logging.mode("STOP");
          if (shooter != null) {
            shooter.ready = shooter.preparing = shooter.feedRequested = false;
            shooter.solution.valid = false;
            shooter.readinessBlockers = 1023;
            shooter.readinessEvaluated = shooter.dwellReady = false;
            shooter.status = "Stopped";
          }
          if (shooter != null) logging.capture(this, System.nanoTime(), driver1, driver2);
          logging.close();
        }
      }
    };
    for (Runnable action : stops)
      try {
        action.run();
      } catch (RuntimeException e) {
        if (failure == null) failure = e;
        else failure.addSuppressed(e);
      }
    if (hubs != null)
      for (LynxModule h : hubs)
        try {
          h.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
          if (failure == null) failure = e;
          else failure.addSuppressed(e);
        }
    if (failure != null) throw failure;
  }
}
