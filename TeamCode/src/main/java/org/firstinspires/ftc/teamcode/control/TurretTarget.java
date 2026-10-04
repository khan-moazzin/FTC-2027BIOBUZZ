package org.firstinspires.ftc.teamcode.control;

public final class TurretTarget {
  public static double measured(
      double wrapped, double previous, double low, double high, double tolerance) {
    return Angles.clamp(choose(wrapped, previous, low - tolerance, high + tolerance), low, high);
  }

  public static double choose(double wrapped, double previous, double low, double high) {
    double best = Angles.clamp(wrapped, low, high), distance = Double.POSITIVE_INFINITY;
    for (int k = -2; k <= 2; k++) {
      double candidate = wrapped + k * 2 * Math.PI;
      if (candidate >= low && candidate <= high && Math.abs(candidate - previous) < distance) {
        best = candidate;
        distance = Math.abs(candidate - previous);
      }
    }
    return best;
  }
}
