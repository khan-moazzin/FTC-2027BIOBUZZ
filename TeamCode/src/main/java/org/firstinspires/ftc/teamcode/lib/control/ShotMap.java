package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.config.ShotConfig;

/** Independent one-dimensional mechanism maps plus the archived physics-calibration sample lookup. */
public final class ShotMap {
  public static final class Interpolation {
    public final double value, zoneMin, zoneMax;
    public final boolean clamped;

    private Interpolation(double value, double a, double b, boolean clamped) {
      this.value = value;
      zoneMin = Math.min(a, b);
      zoneMax = Math.max(a, b);
      this.clamped = clamped;
    }
  }

  /** Linearly interpolates distance/value rows and clamps outside their measured distance range. */
  public static Interpolation interpolate(double[][] map, double distance) {
    if (!validMap(map) || !Double.isFinite(distance) || distance < 0) return null;
    double[] lower = null, upper = null;
    for (double[] point : map) {
      if (point[0] <= distance && (lower == null || point[0] > lower[0])) lower = point;
      if (point[0] >= distance && (upper == null || point[0] < upper[0])) upper = point;
    }
    if (lower == null) {
      upper = nearest(map, true);
      return new Interpolation(upper[1], upper[1], upper[1], true);
    }
    if (upper == null) {
      lower = nearest(map, false);
      return new Interpolation(lower[1], lower[1], lower[1], true);
    }
    if (lower[0] == upper[0])
      return new Interpolation(lower[1], lower[1], lower[1], false);
    double t = (distance - lower[0]) / (upper[0] - lower[0]);
    return new Interpolation(
        lower[1] + t * (upper[1] - lower[1]), lower[1], upper[1], false);
  }

  public static boolean ready(ShotConfig c) {
    return c != null && c.calibrated && validMap(c.hoodMap) && validMap(c.flywheelMap);
  }

  public static boolean validMap(double[][] map) {
    if (map == null || map.length == 0) return false;
    for (int i = 0; i < map.length; i++) {
      double[] point = map[i];
      if (point == null
          || point.length != 2
          || !Double.isFinite(point[0])
          || point[0] < 0
          || !Double.isFinite(point[1])
          || point[1] < 0) return false;
      for (int j = 0; j < i; j++) if (map[j][0] == point[0]) return false;
    }
    return true;
  }

  private static double[] nearest(double[][] map, boolean first) {
    double[] selected = map[0];
    for (double[] point : map)
      if (first ? point[0] < selected[0] : point[0] > selected[0]) selected = point;
    return selected;
  }

  /** Legacy 2D lookup retained for physics calibration and offline analysis. */
  public static double[] lookup(ShotConfig c, double distance, double height) {
    if (!c.calibrated
        || c.samples == null
        || !Double.isFinite(distance)
        || !Double.isFinite(height)) return null;
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
    if (s == null || s.length != 5) return false;
    for (double v : s) if (!Double.isFinite(v)) return false;
    return s[0] > 0 && s[2] > 0 && s[4] > 0;
  }
}
