package org.firstinspires.ftc.teamcode.lib.calibration;

import java.util.*;
import org.firstinspires.ftc.teamcode.config.PhysicsShotConfig;
import org.firstinspires.ftc.teamcode.lib.control.Ballistics;
import org.firstinspires.ftc.teamcode.lib.math.*;

/**
 * Fit exit-speed/RPM ratio and launch-angle bias from stationary measured shots with flight times.
 */
public final class PhysicsCalibration {
  public static final class Result {
    public boolean valid;
    public String reason;
    public double validationRms, validationMax;
  }

  public static Result fit(PhysicsShotConfig c, double[][] samples) {
    Result result = new Result();
    result.reason = "Need 9+ measured shots, varied RPM and angles";
    c.calibrated = false;
    c.modelGenerated = false;
    c.modelSignature = "";
    if (samples == null
        || samples.length < 9
        || !Double.isFinite(c.dragPerMeter)
        || c.dragPerMeter < 0
        || !Double.isFinite(c.calibrationMaxError)
        || c.calibrationMaxError <= 0) return result;
    List<double[]> train = new ArrayList<>(), test = new ArrayList<>();
    double minRpm = Double.POSITIVE_INFINITY, maxRpm = 0;
    for (int i = 0; i < samples.length; i++) {
      double[] s = samples[i];
      if (s == null || s.length != 5) return result;
      for (double x : s) if (!Double.isFinite(x)) return result;
      if (s[0] <= 0 || s[2] <= 0 || s[3] <= 0 || s[3] >= Math.PI / 2 || s[4] <= 0 || s[4] > 3)
        return result;
      (i % 3 == 2 ? test : train).add(s);
      minRpm = Math.min(minRpm, s[2]);
      maxRpm = Math.max(maxRpm, s[2]);
    }
    if (maxRpm - minRpm < 300) {
      result.reason = "Use a wider RPM range (at least 300 RPM)";
      return result;
    }
    LeastSquares.Fit fit =
        LeastSquares.fit(
            p -> {
              double[] residual = new double[train.size() * 2];
              for (int i = 0; i < train.size(); i++) {
                double[] s = train.get(i);
                Vec3 end = predict(s, p[0], p[1], c.dragPerMeter);
                residual[2 * i] = end.x - s[0] * .0254;
                residual[2 * i + 1] = end.z - s[1] * .0254;
              }
              return residual;
            },
            new double[] {c.speedPerRpm > 0 ? c.speedPerRpm : .002, c.launchAngleOffset},
            20);
    if (!fit.valid
        || fit.parameters[0] <= 0
        || fit.parameters[0] > .02
        || Math.abs(fit.parameters[1]) > Math.toRadians(20)
        || fit.rms > c.calibrationMaxError) {
      result.reason = "Physics fit rejected; check measurements/angle frame/drag";
      return result;
    }
    double sum = 0, max = 0;
    for (double[] s : test) {
      Vec3 end = predict(s, fit.parameters[0], fit.parameters[1], c.dragPerMeter);
      double error = Math.hypot(end.x - s[0] * .0254, end.z - s[1] * .0254);
      if (!Double.isFinite(error)) {
        result.reason = "Nonfinite validation";
        return result;
      }
      sum += error * error;
      max = Math.max(max, error);
    }
    result.validationRms = Math.sqrt(sum / test.size());
    result.validationMax = max;
    if (max > c.calibrationMaxError) {
      result.reason = "Held-out shot error exceeds limit";
      return result;
    }
    c.speedPerRpm = fit.parameters[0];
    c.launchAngleOffset = fit.parameters[1];
    c.calibrated = true;
    result.valid = true;
    result.reason = "Physics fit accepted; verify geometry and generate runtime polynomial";
    return result;
  }

  private static Vec3 predict(double[] s, double ratio, double offset, double drag) {
    double v = s[2] * ratio, a = s[3] + offset;
    return Ballistics.atTime(new Vec3(v * Math.cos(a), 0, v * Math.sin(a)), s[4], drag);
  }
}
