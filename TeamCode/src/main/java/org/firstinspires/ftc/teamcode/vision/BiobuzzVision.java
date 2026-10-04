package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.localization.BufferedFusionLocalizer;
import org.firstinspires.ftc.teamcode.math.LeastSquares;

public final class BiobuzzVision implements AutoCloseable {
  public final HiveState red = new HiveState(), blue = new HiveState();
  public final AngleHistory turretHistory = new AngleHistory();
  public final LimelightSource source;
  private final VisionConfig c;
  public String status = "Uncalibrated";

  public BiobuzzVision(HardwareMap hw, VisionConfig c) {
    this.c = c;
    source = new LimelightSource(hw, c);
  }

  public HiveState hive(Field.Hive h) {
    return h == Field.Hive.RED ? red : blue;
  }

  public void update(
      long now, double turret, double turretRate, BufferedFusionLocalizer localizer) {
    turretHistory.add(now, turret);
    LimelightSource.Frame frame = source.poll(now);
    if (frame == null) return;
    if (!c.calibrated || !c.turretCalibrated) {
      status = "Run vision/turret calibration";
      return;
    }
    double captureTurret = turretHistory.at(frame.timestamp);
    Pose prior = localizer.sample(frame.timestamp);
    if (!Double.isFinite(captureTurret) || prior == null) {
      status = "No capture-time history";
      return;
    }
    for (Field.Hive h : Field.Hive.values()) {
      List<TagObservation> tags = new ArrayList<>();
      for (TagObservation t : frame.tags) if (Field.hive(t.id) == h) tags.add(t);
      HiveState state = hive(h);
      double seed = state.fresh(now, c.hiveMaxAge) ? state.angle(frame.timestamp) : 0;
      LeastSquares.Fit fit = VisionPoseSolver.solve(tags, captureTurret, prior, seed, c);
      if (fit == null) {
        status = "Insufficient geometry / fit rejected";
        continue;
      }
      // Tag errors within a frame share range/extrinsic error: do not divide by tag count.
      double range = 0;
      for (TagObservation t : tags) range = Math.max(range, t.camera.norm());
      double sigma =
          c.positionSigma * (1 + range * range / (72 * 72))
              + Math.abs(turretRate) * range * c.exposureSeconds;
      double noise = sigma * sigma * Math.max(1, tags.size());
      double angleVar = Math.max(c.angleSigma * c.angleSigma, fit.covariance[3][3] * noise);
      if (!state.update(fit.parameters[3], angleVar, frame.timestamp, c.hiveAccelerationNoise)) {
        status = "HIVE innovation rejected";
        continue;
      }
      // Moving target/exposure uncertainty propagated through the geometric roll sensitivity.
      double motion = state.rate() * c.exposureSeconds, extra = motion * motion;
      double[][] r = new double[3][3];
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
          double ji = fit.covariance[i][3] / fit.covariance[3][3],
              jj = fit.covariance[j][3] / fit.covariance[3][3];
          r[i][j] = fit.covariance[i][j] * noise + ji * jj * extra;
        }
      r[0][0] += .01;
      r[1][1] += .01;
      r[2][2] += 1e-5;
      boolean ok =
          localizer.addMeasurement(
              new Pose(fit.parameters[0], fit.parameters[1], fit.parameters[2]),
              frame.timestamp,
              new Matrix(r),
              c.innovationGate);
      status = h + ": " + (ok ? "fused" : localizer.status);
    }
  }

  @Override
  public void close() {
    source.close();
  }
}
