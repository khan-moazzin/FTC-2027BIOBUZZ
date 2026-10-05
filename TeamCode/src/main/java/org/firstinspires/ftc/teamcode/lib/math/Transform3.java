package org.firstinspires.ftc.teamcode.lib.math;

/** Right handed inches, radians. R = Rz(yaw) Ry(pitch) Rx(roll). */
public final class Transform3 {
  public final Vec3 translation;
  private final double[][] r;

  public Transform3(Vec3 t, double roll, double pitch, double yaw) {
    translation = t;
    double c = Math.cos(yaw),
        s = Math.sin(yaw),
        b = Math.cos(pitch),
        d = Math.sin(pitch),
        e = Math.cos(roll),
        f = Math.sin(roll);
    r =
        new double[][] {
          {c * b, c * d * f - s * e, c * d * e + s * f},
          {s * b, s * d * f + c * e, s * d * e - c * f},
          {-d, b * f, b * e}
        };
  }

  public Vec3 rotate(Vec3 p) {
    double[] v = p.array(), o = new double[3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) o[i] += r[i][j] * v[j];
    return new Vec3(o);
  }

  public Vec3 apply(Vec3 p) {
    return rotate(p).plus(translation);
  }

  public Vec3 inverse(Vec3 p) {
    double[] v = p.minus(translation).array(), o = new double[3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) o[i] += r[j][i] * v[j];
    return new Vec3(o);
  }
}
