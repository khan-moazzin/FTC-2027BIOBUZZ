package org.firstinspires.ftc.teamcode.lib.control;

public final class Readiness {
  private long since = -1;

  public boolean update(boolean good, long now, double seconds) {
    if (!good || !Double.isFinite(seconds) || seconds < 0) {
      since = -1;
      return false;
    }
    if (since < 0 || now < since) since = now;
    return now - since >= seconds * 1e9;
  }

  public static boolean speed(double rpm, double target, double tolerance) {
    return Double.isFinite(rpm)
        && Double.isFinite(target)
        && target > 0
        && Double.isFinite(tolerance)
        && tolerance >= 0
        && Math.abs(rpm - target) <= tolerance;
  }

  public static boolean motionAllowed(
      boolean stationaryOnly,
      double vx,
      double vy,
      double omega,
      double maxTranslation,
      double maxRotation) {
    if (!Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(omega)) return false;
    return !stationaryOnly
        || (Double.isFinite(maxTranslation)
            && maxTranslation >= 0
            && Double.isFinite(maxRotation)
            && maxRotation >= 0
            && Math.hypot(vx, vy) <= maxTranslation
            && Math.abs(omega) <= maxRotation);
  }

  public static boolean feed(boolean prepare, boolean request, boolean ready, boolean reverse) {
    return prepare && request && ready && !reverse;
  }

  public static boolean wheels(double a, double b, double target, double tolerance) {
    return Double.isFinite(target)
        && target > 0
        && Double.isFinite(tolerance)
        && tolerance >= 0
        && Double.isFinite(a)
        && Double.isFinite(b)
        && Math.abs(a - target) <= tolerance
        && Math.abs(b - target) <= tolerance;
  }
}
