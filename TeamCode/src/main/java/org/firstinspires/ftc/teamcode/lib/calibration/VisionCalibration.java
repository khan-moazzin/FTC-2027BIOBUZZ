package org.firstinspires.ftc.teamcode.lib.calibration;

import com.pedropathing.math.Pose;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.math.*;
import org.firstinspires.ftc.teamcode.lib.vision.*;

/**
 * Batch camera extrinsics and delay fit; turret axis Z is physically measured to fix the Z gauge.
 */
public final class VisionCalibration {
  public static final class Sample {
    public final TagObservation tag;
    public final Pose robot;
    public final double hiveAngle;
    public final long time;

    public Sample(TagObservation t, Pose p, double a, long time) {
      tag = t;
      robot = p;
      hiveAngle = a;
      this.time = time;
    }
  }

  public static LeastSquares.Fit fit(List<Sample> samples, AngleHistory angles, VisionConfig seed) {
    double[] initial = {
      seed.robotToTurret[0],
      seed.robotToTurret[1],
      seed.turretToCamera[0],
      seed.turretToCamera[1],
      seed.turretToCamera[2],
      seed.cameraRotation[0],
      seed.cameraRotation[1],
      seed.cameraRotation[2],
      seed.extraLatency
    };
    return LeastSquares.fit(
        p -> {
          VisionConfig c = parameters(seed, p);
          double[] residual = new double[samples.size() * 3];
          int i = 0;
          for (Sample s : samples) {
            double turret = angles.at(s.time - (long) (p[8] * 1e9));
            Vec3 e =
                Double.isFinite(turret)
                    ? CameraGeometry.toField(s.tag.camera, turret, s.robot, c)
                        .minus(Field.tag(s.tag.id, s.hiveAngle))
                    : new Vec3(1000, 1000, 1000);
            residual[i++] = e.x;
            residual[i++] = e.y;
            residual[i++] = e.z;
          }
          return residual;
        },
        initial,
        30);
  }

  public static VisionConfig parameters(VisionConfig seed, double[] p) {
    VisionConfig c = new VisionConfig();
    c.robotToTurret = new double[] {p[0], p[1], seed.robotToTurret[2]};
    c.turretToCamera = new double[] {p[2], p[3], p[4]};
    c.cameraRotation = new double[] {p[5], p[6], p[7]};
    c.extraLatency = p[8];
    return c;
  }
}
