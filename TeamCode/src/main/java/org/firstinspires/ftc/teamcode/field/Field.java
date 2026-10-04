package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.math.Vec3;

/** BIOBUZZ v26-27.2 CAD; see docs/FIELD_GEOMETRY.md. Pedro inches, +Y scoring. */
public final class Field {
  private Field() {}

  public enum Hive {
    RED,
    BLUE
  }

  public enum Cell {
    AUDIENCE,
    SCORING
  }

  public static final double TAG_SIZE = 3.25, MAX_ANGLE = Math.PI / 6;
  public static final double PIVOT_Z = 43.9497, TAG_RADIUS = 14.2690594803, TAG_Z = -1.48816784224;
  public static final double CELL_RADIUS = TAG_RADIUS + 7.1874, CELL_Z = TAG_Z + 5.622;

  public static Vec3 pivot(Hive h) {
    return new Vec3(h == Hive.RED ? 59.2583590151 : 84.7583590151, 72, PIVOT_Z);
  }

  public static Hive hive(int id) {
    return id >= 30 && id <= 37 ? Hive.RED : id >= 38 && id <= 45 ? Hive.BLUE : null;
  }

  public static Cell side(int id) {
    if (hive(id) == null) throw new IllegalArgumentException("Unknown tag");
    return id < 34 || id >= 42 ? Cell.SCORING : Cell.AUDIENCE;
  }

  public static Vec3 neutralTag(int id) {
    int first = id < 34 ? 30 : id < 38 ? 34 : id < 42 ? 38 : 42;
    double x = new double[] {-6.5, -2.75, 2.75, 6.5}[id - first];
    boolean scoring = side(id) == Cell.SCORING;
    return new Vec3(scoring ? -x : x, scoring ? TAG_RADIUS : -TAG_RADIUS, TAG_Z);
  }

  public static Vec3 tag(int id, double angle) {
    return pivot(hive(id)).plus(neutralTag(id).rotateX(angle));
  }

  public static Vec3 cell(Hive h, Cell c, double angle) {
    return pivot(h)
        .plus(new Vec3(0, c == Cell.SCORING ? CELL_RADIUS : -CELL_RADIUS, CELL_Z).rotateX(angle));
  }

  public static Vec3 cellVelocity(Cell c, double angle, double omega) {
    Vec3 r = new Vec3(0, c == Cell.SCORING ? CELL_RADIUS : -CELL_RADIUS, CELL_Z).rotateX(angle);
    return new Vec3(0, -r.z * omega, r.y * omega);
  }

  public static Cell raised(double angle) {
    return angle >= 0 ? Cell.SCORING : Cell.AUDIENCE;
  }
}
