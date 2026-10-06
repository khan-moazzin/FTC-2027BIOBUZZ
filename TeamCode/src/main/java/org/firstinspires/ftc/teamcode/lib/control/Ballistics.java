package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.lib.math.Vec3;

/** SI point-mass model: gravity and optional quadratic drag, still air, no spin lift. */
public final class Ballistics {
  public static final double G = 9.80665, DT = .005;

  public static final class Crossing {
    public final Vec3 position;
    public final double time, miss;

    Crossing(Vec3 p, double t, Vec3 center) {
      position = p;
      time = t;
      miss = p.minus(center).norm();
    }
  }

  private static void derivative(double[] y, double drag, double[] out) {
    double speed = Math.sqrt(y[3] * y[3] + y[4] * y[4] + y[5] * y[5]);
    out[0] = y[3];
    out[1] = y[4];
    out[2] = y[5];
    out[3] = -drag * speed * y[3];
    out[4] = -drag * speed * y[4];
    out[5] = -G - drag * speed * y[5];
  }

  private static void step(double[] y, double dt, double drag, double[][] k, double[] temp) {
    derivative(y, drag, k[0]);
    for (int i = 0; i < 6; i++) temp[i] = y[i] + dt * k[0][i] / 2;
    derivative(temp, drag, k[1]);
    for (int i = 0; i < 6; i++) temp[i] = y[i] + dt * k[1][i] / 2;
    derivative(temp, drag, k[2]);
    for (int i = 0; i < 6; i++) temp[i] = y[i] + dt * k[2][i];
    derivative(temp, drag, k[3]);
    for (int i = 0; i < 6; i++) y[i] += dt * (k[0][i] + 2 * k[1][i] + 2 * k[2][i] + k[3][i]) / 6;
  }

  public static Vec3 atTime(Vec3 velocity, double time, double drag) {
    if (!velocity.finite()
        || !Double.isFinite(time)
        || time < 0
        || time > 3
        || !Double.isFinite(drag)
        || drag < 0) return new Vec3(Double.NaN, 0, 0);
    if (drag == 0) return velocity.times(time).plus(new Vec3(0, 0, -.5 * G * time * time));
    double[] y = {0, 0, 0, velocity.x, velocity.y, velocity.z};
    double[][] k = new double[4][6];
    double[] temp = new double[6];
    for (double t = 0; t < time; t += DT) step(y, Math.min(DT, time - t), drag, k, temp);
    return new Vec3(y[0], y[1], y[2]);
  }

  /** First front-to-back crossing of aperture plane. Normal points out of the receiving opening. */
  public static Crossing cross(
      Vec3 start, Vec3 velocity, Vec3 center, Vec3 normal, double drag, double maxTime) {
    if (!start.finite()
        || !velocity.finite()
        || !center.finite()
        || !normal.finite()
        || Math.abs(normal.norm() - 1) > 1e-6
        || !Double.isFinite(drag)
        || drag < 0
        || !Double.isFinite(maxTime)
        || maxTime <= 0
        || maxTime > 3) return null;
    if (drag == 0) {
      double a = -.5 * G * normal.z, b = velocity.dot(normal), c = start.minus(center).dot(normal);
      double[] roots;
      if (Math.abs(a) < 1e-12) roots = new double[] {b < 0 ? -c / b : Double.NaN};
      else {
        double disc = b * b - 4 * a * c;
        if (disc < 0) return null;
        roots = new double[] {(-b - Math.sqrt(disc)) / (2 * a), (-b + Math.sqrt(disc)) / (2 * a)};
      }
      double time = Double.POSITIVE_INFINITY;
      for (double t : roots)
        if (t > 0 && t <= maxTime && b + 2 * a * t < 0) time = Math.min(time, t);
      if (!Double.isFinite(time)) return null;
      return new Crossing(start.plus(atTime(velocity, time, 0)), time, center);
    }
    double[] y = {start.x, start.y, start.z, velocity.x, velocity.y, velocity.z};
    double[][] k = new double[4][6];
    double[] temp = new double[6];
    Vec3 previous = start;
    double previousSide = start.minus(center).dot(normal);
    for (double t = 0; t < maxTime; t += DT) {
      double dt = Math.min(DT, maxTime - t);
      step(y, dt, drag, k, temp);
      Vec3 next = new Vec3(y[0], y[1], y[2]);
      double side = next.minus(center).dot(normal);
      if (previousSide > 0 && side <= 0) {
        double f = previousSide / (previousSide - side);
        return new Crossing(previous.plus(next.minus(previous).times(f)), t + dt * f, center);
      }
      previous = next;
      previousSide = side;
    }
    return null;
  }

  /** Ascending/descending height at a specified downrange distance, used only by offline search. */
  public static double[] atDistance(
      double speed, double angle, double radial, double distance, double drag, double maxTime) {
    double vx = speed * Math.cos(angle) + radial, vz = speed * Math.sin(angle);
    if (vx <= 0 || distance <= 0) return null;
    if (drag == 0) {
      double t = distance / vx;
      return t > maxTime ? null : new double[] {vz * t - .5 * G * t * t, t, vz - G * t};
    }
    double[] y = {0, 0, 0, vx, 0, vz};
    double[][] k = new double[4][6];
    double[] temp = new double[6];
    for (double t = 0; t < maxTime; t += DT) {
      double px = y[0], pz = y[2], pvz = y[5], dt = Math.min(DT, maxTime - t);
      step(y, dt, drag, k, temp);
      if (y[0] >= distance) {
        double f = (distance - px) / (y[0] - px);
        return new double[] {pz + f * (y[2] - pz), t + dt * f, pvz + f * (y[5] - pvz)};
      }
    }
    return null;
  }
}
