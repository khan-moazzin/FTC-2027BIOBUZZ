package org.firstinspires.ftc.teamcode.lib.logging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.gson.GsonBuilder;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.*;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.util.*;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.vision.*;

/** One session per Robot instance. Snapshot cached state only, after commands and writes. */
public final class RobotLogging implements AutoCloseable {
  private AsyncCsvLog log;
  private String path = "Disabled", error = "";
  private final long epoch = System.nanoTime();
  private long sequence;
  private double captureMs;
  private String mode = "INIT";

  public RobotLogging(Robot robot) {
    if (!LoggingConfig.enabled) return;
    FileOutputStream output = null;
    try {
      File directory = new File(AppUtil.FIRST_FOLDER, "biobuzz-logs");
      if (!directory.isDirectory() && !directory.mkdirs())
        throw new IOException("Cannot create log directory");
      if (directory.getUsableSpace() < LoggingConfig.minimumFreeBytes)
        throw new IOException("Low free storage");
      File file = File.createTempFile("run-" + System.currentTimeMillis() + "-", ".csv", directory);
      path = file.getAbsolutePath();
      Map<String, Object> metadata = new LinkedHashMap<>();
      metadata.put("schema", 4);
      metadata.put("createdUnixMs", System.currentTimeMillis());
      metadata.put("timebase", "monotonic seconds since logger INIT; snapshot time is cycle start");
      metadata.put("poseUnits", "Pedro inches and radians");
      metadata.put("VisionConfig", config(robot.visionConfig));
      metadata.put("MechanismConfig", config(new MechanismConfig()));
      metadata.put("ShotConfig", config(new ShotConfig()));
      metadata.put("PhysicsShotConfig", config(robot.shooter.physics));
      metadata.put("LoggingConfig", config(new LoggingConfig()));
      try (Writer writer =
          new OutputStreamWriter(new FileOutputStream(path + ".json"), StandardCharsets.UTF_8)) {
        new GsonBuilder()
            .serializeSpecialFloatingPointValues()
            .setPrettyPrinting()
            .create()
            .toJson(metadata, writer);
      }
      output = new FileOutputStream(file);
      final long reserve = LoggingConfig.minimumFreeBytes;
      final FileOutputStream fileOutput = output;
      OutputStream guarded =
          new BufferedOutputStream(
              new FilterOutputStream(fileOutput) {
                private long checked;

                @Override
                public void write(byte[] bytes, int offset, int length) throws IOException {
                  if (checked <= 0) {
                    if (directory.getUsableSpace() < reserve)
                      throw new IOException("Low free storage");
                    checked = 1024 * 1024;
                  }
                  out.write(bytes, offset, length);
                  checked -= length;
                }
              });
      log =
          new AsyncCsvLog(
              guarded,
              epoch,
              LoggingConfig.queueCapacity,
              LoggingConfig.maxFileBytes,
              LoggingConfig.live
                  ? frame -> {
                    TelemetryPacket packet = new TelemetryPacket();
                    for (Map.Entry<String, Object> entry : frame.values.entrySet())
                      packet.put(entry.getKey(), entry.getValue());
                    packet.put("Logging/CaptureTime_s", (frame.time - epoch) * 1e-9);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                  }
                  : null,
              Math.max(1, LoggingConfig.livePeriodMs) * 1000000L);
    } catch (IOException | RuntimeException | IllegalAccessException e) {
      error = "Logging unavailable: " + e;
      if (output != null)
        try {
          output.close();
        } catch (IOException ignored) {
        }
    }
  }

  private static Map<String, Object> config(Object config) throws IllegalAccessException {
    Map<String, Object> out = new TreeMap<>();
    for (Field f : config.getClass().getFields()) out.put(f.getName(), f.get(config));
    return out;
  }

  public void mode(String value) {
    mode = value;
  }

  public String status() {
    return !error.isEmpty()
        ? error
        : log == null
            ? "Disabled"
            : log.status()
                + "; live "
                + log.liveStatus()
                + "; dropped "
                + log.dropped()
                + "; file lost "
                + log.fileDropped();
  }

  public String path() {
    return path;
  }

  public void capture(Robot r, long now, Gamepad g1, Gamepad g2) {
    if (log == null || !error.isEmpty()) return;
    long started = System.nanoTime();
    try {
      Map<String, Object> v = new LinkedHashMap<>();
      v.put("Session/Mode", mode);
      v.put("Session/Alliance", Constants.ALLIANCE.name());
      v.put("Logging/Sequence", sequence++);
      v.put("Logging/Dropped", log.dropped());
      v.put("Logging/FileDropped", log.fileDropped());
      v.put("Logging/Written", log.written());
      v.put("Logging/Queued", log.queued());
      v.put("Logging/Bytes", log.bytes());
      v.put("Logging/Status", log.status());
      v.put("Logging/LiveStatus", log.liveStatus());
      n(v, "Logging/PreviousCapture_ms", captureMs);
      pose(v, "Localization/Pose", r.drive.getPose());
      pose(v, "Localization/RawPose", r.drive.localizer().rawPose());
      n(v, "Localization/Vx_in_s", r.drive.localizer().velocity().vx);
      n(v, "Localization/Vy_in_s", r.drive.localizer().velocity().vy);
      n(v, "Localization/Omega_rad_s", r.drive.localizer().velocity().omega);
      n(v, "Localization/PositionSigma_in", r.drive.localizer().positionSigma());
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          n(v, "Localization/Covariance/" + i + j, r.drive.localizer().covariance(i, j));
      v.put("Localization/Healthy", r.drive.localizer().healthy(now));
      v.put("Localization/Health", r.drive.localizer().healthStatus);
      v.put("Localization/Fusion", r.drive.localizer().status);
      v.put("Drive/RobotOriented", r.drive.isRobotOriented());
      v.put("Drive/Following", r.drive.getFollower().following());
      v.put("Drive/Holding", r.drive.getFollower().holding());
      v.put("Vision/Status", r.vision.status);
      v.put("Vision/Source", r.vision.source.status);
      v.put("Vision/FrameSequence", r.vision.frameSequence);
      LimelightSource.Frame f = r.vision.lastFrame;
      n(v, "Vision/CaptureTime_s", f == null ? Double.NaN : (f.timestamp - epoch) * 1e-9);
      n(v, "Vision/FrameAge_s", f == null ? Double.NaN : (now - f.timestamp) * 1e-9);
      for (int id = 30; id <= 45; id++) {
        TagObservation found = null;
        if (f != null) for (TagObservation tag : f.tags) if (tag.id == id) found = tag;
        String prefix = "Vision/Tags/" + id;
        v.put(prefix + "/Present", found != null);
        n(v, prefix + "/X_in", found == null ? Double.NaN : found.camera.x);
        n(v, prefix + "/Y_in", found == null ? Double.NaN : found.camera.y);
        n(v, prefix + "/Z_in", found == null ? Double.NaN : found.camera.z);
      }
      hive(v, "Red", r.vision.red, r.vision.redDiagnostic, r, now);
      hive(v, "Blue", r.vision.blue, r.vision.blueDiagnostic, r, now);
      v.put("Shot/Status", r.shooter.status);
      v.put("Shot/Model", "Stable-HIVE empirical maps + bounded physics correction");
      v.put("HIVE/Red/Stable", r.shooter.redStable);
      v.put("HIVE/Blue/Stable", r.shooter.blueStable);
      n(v, "Shot/Clearance_m", r.shooter.solution.clearance);
      v.put("Shot/Reason", r.shooter.solution.reason);
      v.put("Shot/SelectedCell", r.shooter.selectedCell());
      v.put("Shot/Valid", r.shooter.solution.valid);
      v.put("Shot/MapClamped", r.shooter.solution.mapClamped);
      v.put("Shot/PhysicsCorrected", r.shooter.solution.physicsCorrected);
      v.put("Shot/Ready", r.shooter.ready);
      v.put("Shot/Preparing", r.shooter.preparing);
      v.put("Shot/FeedRequested", r.shooter.feedRequested);
      v.put("Shot/Cancelled", r.shooter.cancelled());
      v.put("Shot/ReadinessEvaluated", r.shooter.readinessEvaluated);
      v.put("Shot/ReadinessBlockers", r.shooter.readinessBlockers);
      String[] gates = {
        "Calibration",
        "RpmRange",
        "Pose",
        "Reachable",
        "Turret",
        "Flywheels",
        "HoodSettled",
        "HoodRange",
        "IndexerCalibrated",
        "RobotMotion"
      };
      for (int i = 0; i < gates.length; i++)
        v.put("Shot/Blocked/" + gates[i], (r.shooter.readinessBlockers & (1 << i)) != 0);
      n(v, "Shot/TrajectoryChecks", r.shooter.solution.iterations);
      n(v, "Shot/HiveVariance_rad2", r.shooter.solution.impactVariance);
      vector(v, "Shot/Target_in", r.shooter.solution.predictedTarget);
      vector(v, "Shot/Muzzle_in", r.shooter.solution.muzzle);
      vector(v, "Shot/LaunchVelocity_in_s", r.shooter.solution.launchVelocity);
      n(v, "Drive/CommandForward", r.drive.commandForward);
      n(v, "Drive/CommandStrafe", r.drive.commandStrafe);
      n(v, "Drive/CommandYaw", r.drive.commandYaw);
      n(v, "Shot/Angle_rad", r.shooter.solution.angle);
      n(v, "Shot/AngularVelocity_rad_s", r.shooter.solution.angularVelocity);
      n(v, "Shot/Rpm", r.shooter.solution.rpm);
      n(v, "Shot/FlywheelPercent", r.shooter.solution.rpm / MechanismConfig.maxRpm * 100);
      n(v, "Shot/Hood_rad", r.shooter.solution.hood);
      n(v, "Shot/Flight_s", r.shooter.solution.flight);
      n(v, "Shot/Distance_in", r.shooter.solution.distance);
      n(v, "Shot/Height_in", r.shooter.solution.height);
      n(v, "Shot/TurretOverflow_rad", r.shooter.overflow());
      n(v, "Turret/Measured_rad", r.turret.feedback.angle);
      n(v, "Turret/Velocity_rad_s", r.turret.feedback.velocity);
      n(v, "Turret/Disagreement_rad", r.turret.feedback.disagreement);
      n(v, "Turret/Left_V", r.turret.leftVoltage);
      n(v, "Turret/Right_V", r.turret.rightVoltage);
      n(v, "Turret/Target_rad", r.turret.getTargetAngle());
      n(v, "Turret/Command_rad", r.turret.getCommandAngle());
      n(v, "Turret/ServoPosition", r.turret.getPosition());
      v.put("Turret/Healthy", r.turret.feedback.healthy);
      v.put("Turret/Ready", r.turret.ready());
      n(v, "Flywheel/Target_rpm", r.flywheel.getTargetRpm());
      n(v, "Flywheel/Measured_rpm", r.flywheel.getRpm());
      n(v, "Flywheel/Duty", r.flywheel.duty());
      n(v, "Indexer/CommandPosition", r.indexer.getPosition());
      v.put("Indexer/Calibrated", MechanismConfig.indexerCalibrated);
      v.put("Shot/DwellReady", r.shooter.dwellReady);
      n(v, "Battery/Voltage_V", r.flywheel.getVoltage());
      v.put("Flywheel/Ready", r.flywheel.atSpeed());
      n(v, "Hood/CommandPosition", r.hood.getPosition());
      v.put("Hood/Settled", r.hood.ready(now));
      n(v, "Intake/CommandPower", r.intake.getPower());
      n(v, "Loop/Read_ms", r.timing.readMs);
      n(v, "Loop/Commands_ms", r.timing.commandMs);
      n(v, "Loop/Write_ms", r.timing.writeMs);
      n(v, "Loop/Work_ms", r.timing.workMs);
      n(v, "Loop/PreviousPeriod_ms", r.timing.periodMs);
      gamepad(v, "Driver1", g1);
      gamepad(v, "Driver2", g2);
      log.offer(new LogFrame(now, v));
    } catch (RuntimeException e) {
      error = "Snapshot error: " + e;
    }
    captureMs = (System.nanoTime() - started) * 1e-6;
  }

  private static void vector(
      Map<String, Object> v, String key, org.firstinspires.ftc.teamcode.lib.math.Vec3 p) {
    n(v, key + "/X", p == null ? Double.NaN : p.x);
    n(v, key + "/Y", p == null ? Double.NaN : p.y);
    n(v, key + "/Z", p == null ? Double.NaN : p.z);
  }

  private static void n(Map<String, Object> v, String key, double x) {
    LogFrame.number(v, key, x);
  }

  private static void pose(Map<String, Object> v, String key, Pose p) {
    n(v, key + " x", p.x());
    n(v, key + " y", p.y());
    n(v, key + " heading", p.heading());
  }

  private static void hive(
      Map<String, Object> v,
      String name,
      HiveState h,
      BiobuzzVision.Diagnostic d,
      Robot r,
      long now) {
    String k = "HIVE/" + name;
    n(v, k + "/Angle_rad", h.angle(now));
    n(v, k + "/Rate_rad_s", h.rate());
    n(v, k + "/Age_s", h.age(now));
    n(v, k + "/Variance_rad2", h.variance(now, r.visionConfig.hiveAccelerationNoise));
    v.put(k + "/Fresh", h.fresh(now, r.visionConfig.hiveMaxAge));
    v.put(k + "/Fit/Result", d.result);
    v.put(k + "/Fit/Accepted", d.accepted);
    v.put(k + "/Fit/Tags", d.tagCount);
    n(v, k + "/Fit/Rms_in", d.rms);
    n(v, k + "/Fit/Innovation", d.innovation);
    n(v, k + "/Fit/X_in", d.x);
    n(v, k + "/Fit/Y_in", d.y);
    n(v, k + "/Fit/Heading_rad", d.heading);
    n(v, k + "/Fit/Angle_rad", d.angle);
  }

  private static void gamepad(Map<String, Object> v, String k, Gamepad g) {
    v.put(k + "/Present", g != null);
    n(v, k + "/LeftX", g == null ? 0 : g.left_stick_x);
    n(v, k + "/LeftY", g == null ? 0 : g.left_stick_y);
    n(v, k + "/RightX", g == null ? 0 : g.right_stick_x);
    n(v, k + "/RightY", g == null ? 0 : g.right_stick_y);
    n(v, k + "/LeftTrigger", g == null ? 0 : g.left_trigger);
    n(v, k + "/RightTrigger", g == null ? 0 : g.right_trigger);
    v.put(k + "/A", g != null && g.a);
    v.put(k + "/B", g != null && g.b);
    v.put(k + "/X", g != null && g.x);
    v.put(k + "/Y", g != null && g.y);
    v.put(k + "/LeftBumper", g != null && g.left_bumper);
    v.put(k + "/RightBumper", g != null && g.right_bumper);
    v.put(k + "/Back", g != null && g.back);
    v.put(k + "/Start", g != null && g.start);
    v.put(k + "/DpadUp", g != null && g.dpad_up);
    v.put(k + "/DpadDown", g != null && g.dpad_down);
    v.put(k + "/DpadLeft", g != null && g.dpad_left);
    v.put(k + "/DpadRight", g != null && g.dpad_right);
  }

  @Override
  public void close() {
    if (log == null) return;
    log.close();
    Map<String, Object> summary = new LinkedHashMap<>();
    summary.put("status", status());
    summary.put("written", log.written());
    summary.put("queueDropped", log.dropped());
    summary.put("fileDropped", log.fileDropped());
    summary.put("bytes", log.bytes());
    try (Writer writer =
        new OutputStreamWriter(
            new FileOutputStream(path + ".summary.json"), StandardCharsets.UTF_8)) {
      new GsonBuilder().setPrettyPrinting().create().toJson(summary, writer);
    } catch (IOException | RuntimeException e) {
      error = "Summary error: " + e;
    }
  }
}
