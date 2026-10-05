package org.firstinspires.ftc.teamcode.lib.math;

public final class Vec3 {
  public final double x, y, z;

  public Vec3(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public Vec3(double[] p) {
    this(p[0], p[1], p[2]);
  }

  public Vec3 plus(Vec3 b) {
    return new Vec3(x + b.x, y + b.y, z + b.z);
  }

  public Vec3 minus(Vec3 b) {
    return new Vec3(x - b.x, y - b.y, z - b.z);
  }

  public Vec3 times(double s) {
    return new Vec3(x * s, y * s, z * s);
  }

  public double norm() {
    return Math.sqrt(x * x + y * y + z * z);
  }

  public double dot(Vec3 b) {
    return x * b.x + y * b.y + z * b.z;
  }

  public boolean finite() {
    return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
  }

  public double[] array() {
    return new double[] {x, y, z};
  }

  public Vec3 rotateX(double a) {
    return new Vec3(x, Math.cos(a) * y - Math.sin(a) * z, Math.sin(a) * y + Math.cos(a) * z);
  }

  public Vec3 rotateZ(double a) {
    return new Vec3(Math.cos(a) * x - Math.sin(a) * y, Math.sin(a) * x + Math.cos(a) * y, z);
  }
}
