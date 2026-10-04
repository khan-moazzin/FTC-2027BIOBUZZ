package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.ShotConfig;

/** Barycentric interpolation inside measured triangles; never extrapolates outside coverage. */
public final class ShotMap {
  public static double[] lookup(ShotConfig c, double distance, double height) {
    if (!c.calibrated || !Double.isFinite(distance) || !Double.isFinite(height)) return null;
    double[] best = null;
    double area = Double.POSITIVE_INFINITY;
    for (int i = 0; i < c.samples.length; i++)
      for (int j = i + 1; j < c.samples.length; j++)
        for (int k = j + 1; k < c.samples.length; k++) {
          double[] a = c.samples[i], b = c.samples[j], d = c.samples[k];
          if (!valid(a) || !valid(b) || !valid(d)) continue;
          double det = (b[1] - d[1]) * (a[0] - d[0]) + (d[0] - b[0]) * (a[1] - d[1]);
          if (Math.abs(det) < 1e-6) continue;
          double u = ((b[1] - d[1]) * (distance - d[0]) + (d[0] - b[0]) * (height - d[1])) / det;
          double v = ((d[1] - a[1]) * (distance - d[0]) + (a[0] - d[0]) * (height - d[1])) / det,
              w = 1 - u - v;
          if (u < -1e-8 || v < -1e-8 || w < -1e-8 || Math.abs(det) >= area) continue;
          best =
              new double[] {
                u * a[2] + v * b[2] + w * d[2],
                u * a[3] + v * b[3] + w * d[3],
                u * a[4] + v * b[4] + w * d[4]
              };
          area = Math.abs(det);
        }
    return best;
  }

  public static boolean valid(double[] s) {
    if (s.length != 5) return false;
    for (double v : s) if (!Double.isFinite(v)) return false;
    return s[0] > 0 && s[2] > 0 && s[4] > 0;
  }
}
