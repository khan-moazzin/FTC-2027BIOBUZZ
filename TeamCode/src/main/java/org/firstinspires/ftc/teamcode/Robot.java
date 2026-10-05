package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.control.LoopTiming;
import org.firstinspires.ftc.teamcode.lib.vision.BiobuzzVision;
import org.firstinspires.ftc.teamcode.subsystems.*;

public final class Robot implements AutoCloseable {
  public Shooter shooter;
  public Drive drive;
  public Intake intake;
  public Turret turret;
  public Hood hood;
  public Flywheel flywheel;
  public BiobuzzVision vision;
  public final VisionConfig visionConfig = new VisionConfig();
  public final LoopTiming timing = new LoopTiming();
  private List<LynxModule> hubs;
  private Telemetry telemetry;

  public void init(HardwareMap hw, Telemetry t) {
    try {
      telemetry = t;
      hubs = hw.getAll(LynxModule.class);
      for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      drive = new Drive(hw);
      intake = new Intake(hw);
      turret = new Turret(hw, visionConfig);
      hood = new Hood(hw);
      flywheel = new Flywheel(hw);
      vision = new BiobuzzVision(hw, visionConfig);
      shooter = new Shooter(this);
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
    intake.write();
    turret.write();
    hood.write();
    flywheel.write();
    timing.finish(started);
  }

  public void sendTelemetry() {
    telemetry.addData("Loop", timing.summary());
    telemetry.addData("Shooter", shooter.status);
    telemetry.addData("Vision", vision.status);
    telemetry.addData("Pose", drive.getPose());
    telemetry.addData("Turret feedback", turret.feedback.healthy);
    telemetry.addData(
        "Flywheel RPM L/R", "%.0f / %.0f", flywheel.getLeftRpm(), flywheel.getRightRpm());
  }

  public void close() {
    RuntimeException failure = null;
    Runnable[] stops = {
      () -> CommandScheduler.getInstance().cancelAll(),
      () -> {
        if (intake != null) intake.stop();
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
      () -> CommandScheduler.getInstance().reset()
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
