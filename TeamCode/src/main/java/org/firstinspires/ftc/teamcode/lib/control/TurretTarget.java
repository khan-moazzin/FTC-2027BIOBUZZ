package org.firstinspires.ftc.teamcode.lib.control;

public final class TurretTarget {
  public static boolean ambiguous(double wrapped, double low, double high, double tolerance) {
    int candidates = 0;
    for (int k = -2; k <= 2; k++) {
      double a = wrapped + k * 2 * Math.PI;
      if (a >= low - tolerance && a <= high + tolerance) candidates++;
    }
    return candidates != 1;
  }

  public static double measured(
      double wrapped, double previous, double low, double high, double tolerance) {
    return Angles.clamp(choose(wrapped, previous, low - tolerance, high + tolerance), low, high);
  }

  public static double choose(double wrapped, double previous, double low, double high) {
    double best = Double.NaN, error = Double.POSITIVE_INFINITY, distance = Double.POSITIVE_INFINITY;
    for (int k = -2; k <= 2; k++) {
      double candidate = wrapped + k * 2 * Math.PI;
      double reachable = Angles.clamp(candidate, low, high);
      double miss = Math.abs(candidate - reachable), travel = Math.abs(reachable - previous);
      if (miss < error - 1e-9 || (Math.abs(miss - error) <= 1e-9 && travel < distance)) {
        best = reachable;
        error = miss;
        distance = travel;
      }
    }
    return best;
  }
}
