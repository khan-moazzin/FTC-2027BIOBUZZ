package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.control.Angles;
import org.firstinspires.ftc.teamcode.field.Field;

/** Independent angle/rate Kalman state. Seconds are monotonic, angles radians. */
public final class HiveState {
  private double angle, rate, p = 1, cross, q = 4;
  private long time = -1;

  public double angle(long t) {
    return Angles.clamp(angle + rate * age(t), -Field.MAX_ANGLE, Field.MAX_ANGLE);
  }

  public double rate() {
    return rate;
  }

  public double age(long t) {
    return time < 0 ? Double.POSITIVE_INFINITY : Math.max(0, (t - time) * 1e-9);
  }

  public double variance(long t, double acceleration) {
    double dt = age(t);
    return p + 2 * dt * cross + dt * dt * q + acceleration * dt * dt * dt * dt / 4;
  }

  public boolean fresh(long t, double maxAge) {
    return time >= 0 && age(t) <= maxAge;
  }

  public boolean update(double measurement, double variance, long t, double acceleration) {
    if (!Double.isFinite(measurement)
        || !Double.isFinite(variance)
        || variance <= 0
        || Math.abs(measurement) > Field.MAX_ANGLE + .04
        || t <= time) return false;
    if (time < 0) {
      angle = measurement;
      p = variance;
      time = t;
      return true;
    }
    double dt = (t - time) * 1e-9,
        a = angle(t),
        pp = variance(t, acceleration),
        pc = cross + dt * q + acceleration * dt * dt * dt / 2,
        pq = q + acceleration * dt * dt;
    double residual = measurement - a;
    if (residual * residual > 16 * (pp + variance)) return false;
    double k = pp / (pp + variance), kr = pc / (pp + variance);
    angle = a + k * residual;
    rate += kr * residual;
    p = (1 - k) * pp;
    cross = (1 - k) * pc;
    q = Math.max(1e-8, pq - kr * pc);
    time = t;
    return true;
  }
}
