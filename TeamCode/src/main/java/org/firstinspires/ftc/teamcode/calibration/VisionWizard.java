package org.firstinspires.ftc.teamcode.calibration;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.*;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.*;

@TeleOp(name = "Tune 2 - Complete BIOBUZZ Vision Wizard", group = "Calibration")
public final class VisionWizard extends GuidedOpMode {
  private final VisionConfig c = new VisionConfig();
  private Turret turret;
  private LimelightSource source;
  private final AngleHistory history = new AngleHistory(30000, 600000000000L);
  private final List<VisionCalibration.Sample> samples = new ArrayList<>();
  private final double[] known = {72, 24, 90, -30, 30, 0};
  private final String[] labels = {
    "robot X inches",
    "robot Y inches",
    "robot heading degrees",
    "red HIVE angle degrees",
    "blue HIVE angle degrees",
    "turret pivot height inches"
  };
  private int selected;
  private boolean recording;
  private long last, frameAt;
  private double commanded;

  protected void setup() {
    turret = new Turret(hardwareMap, c);
    source = new LimelightSource(hardwareMap, c);
    known[5] = c.robotToTurret[2];
    c.extraLatency = 0;
  }

  protected void tick(long now) throws Exception {
    double dt = last == 0 ? .02 : Math.min(.1, (now - last) * 1e-9);
    last = now;
    turret.read(now);
    history.add(now, turret.feedback.healthy ? turret.feedback.angle : Double.NaN);
    if (!c.turretCalibrated) {
      telemetry.addLine("First run Tune 1, copy VisionConfig.java and rebuild.");
      return;
    }
    if (x && !recording) selected = (selected + 1) % known.length;
    if (!recording)
      known[selected] +=
          -gamepad1.left_stick_y * dt * (selected == 2 || selected == 3 || selected == 4 ? 20 : 5);
    telemetry.addData("X selects; left stick adjusts", labels[selected] + " = " + known[selected]);
    telemetry.addLine(
        "Measure robot pose and pivot height. Keep robot and both HIVEs stationary while"
            + " recording.");
    telemetry.addLine(
        "A starts/stops a capture. Hold RB and right stick X to sweep turret slowly back AND"
            + " forth.");
    telemetry.addLine(
        "Between captures: move robot to 3+ surveyed poses/headings; record both HIVE end"
            + " positions.");
    telemetry.addLine(
        "Y fits camera XYZ, roll/pitch/yaw, turret COR XY, residual latency; exports COMPLETE"
            + " VisionConfig.");
    if (a) {
      recording = !recording;
      commanded = turret.feedback.angle;
    }
    if (b) recording = false;
    if (recording && gamepad1.right_bumper) {
      commanded += gamepad1.right_stick_x * dt * .4;
      turret.aim(commanded, 0, dt);
      turret.write();
    }
    LimelightSource.Frame frame = source.poll(now);
    if (recording && frame != null && now - frameAt > 80000000L && samples.size() < 1200) {
      Pose pose = new Pose(known[0], known[1], Math.toRadians(known[2]));
      for (TagObservation t : frame.tags)
        samples.add(
            new VisionCalibration.Sample(
                t,
                pose,
                Math.toRadians(known[Field.hive(t.id) == Field.Hive.RED ? 3 : 4]),
                frame.timestamp));
      frameAt = now;
    }
    telemetry.addData("Recording / observations", recording + " / " + samples.size());
    telemetry.addData("Camera", source.status);
    if (y && !recording) {
      if (samples.size() < 100 || known[5] <= 0) {
        result = "Need 100+ observations and measured positive pivot height";
        return;
      }
      StringBuilder csv =
          new StringBuilder(
              "timestamp_ns,id,robot_x,robot_y,heading,hive_angle,camera_x,camera_y,camera_z\n");
      for (VisionCalibration.Sample sample : samples)
        csv.append(sample.time)
            .append(',')
            .append(sample.tag.id)
            .append(',')
            .append(sample.robot.x())
            .append(',')
            .append(sample.robot.y())
            .append(',')
            .append(sample.robot.heading())
            .append(',')
            .append(sample.hiveAngle)
            .append(',')
            .append(sample.tag.camera.x)
            .append(',')
            .append(sample.tag.camera.y)
            .append(',')
            .append(sample.tag.camera.z)
            .append('\n');
      ConfigExport.saveText("vision-observations.csv", csv.toString());
      ConfigExport.saveText("vision-turret-history.csv", history.csv());
      List<VisionCalibration.Sample> training = new ArrayList<>(), test = new ArrayList<>();
      for (int i = 0; i < samples.size(); i++) (i % 5 == 0 ? test : training).add(samples.get(i));
      c.robotToTurret[2] = known[5];
      LeastSquares.Fit fit = VisionCalibration.fit(training, history, c);
      if (!fit.valid || fit.rms > c.maxFitRms || fit.parameters[8] < 0 || fit.parameters[8] > .15) {
        result = "Fit rejected: need diverse poses, angles, and sweeps; RMS=" + fit.rms;
        return;
      }
      VisionConfig fitted = VisionCalibration.parameters(c, fit.parameters);
      double error = 0;
      for (VisionCalibration.Sample s : test) {
        double a = history.at(s.time - (long) (fitted.extraLatency * 1e9));
        Vec3 e =
            CameraGeometry.toField(s.tag.camera, a, s.robot, fitted)
                .minus(Field.tag(s.tag.id, s.hiveAngle));
        error += e.dot(e);
      }
      double rms = Math.sqrt(error / (test.size() * 3));
      if (!Double.isFinite(rms) || rms > c.maxFitRms) {
        result = "Held-out validation failed: " + rms;
        return;
      }
      c.robotToTurret = fitted.robotToTurret;
      c.turretToCamera = fitted.turretToCamera;
      c.cameraRotation = fitted.cameraRotation;
      c.extraLatency = fitted.extraLatency;
      c.positionSigma = Math.max(.25, 2 * rms);
      c.angleSigma = Math.max(Math.toRadians(1), c.positionSigma / Field.TAG_RADIUS);
      c.calibrated = true;
      export(c);
      result += "; held-out RMS " + rms + " inches";
    }
  }

  protected void shutdown() {
    if (source != null) source.close();
    if (turret != null) turret.hold();
  }
}
