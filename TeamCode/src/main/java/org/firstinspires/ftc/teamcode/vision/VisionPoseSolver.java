package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.*;

/** Fits robot x/y/yaw and this HIVE's roll to 3D tag centers. At least two separated IDs. */
public final class VisionPoseSolver {
  public static LeastSquares.Fit solve(
      List<TagObservation> tags, double turret, Pose prior, double initialAngle, VisionConfig c) {
    if (tags.size() < 2) return null;
    double heading = prior.heading(), span = 0;
    for (int i = 0; i < tags.size(); i++)
      for (int j = i + 1; j < tags.size(); j++) {
        TagObservation a = tags.get(i), b = tags.get(j);
        if (Field.side(a.id) != Field.side(b.id)) continue;
        Vec3 expected = Field.neutralTag(b.id).minus(Field.neutralTag(a.id));
        if (Math.abs(expected.x) > span) {
          Vec3 observed =
              CameraGeometry.toRobot(b.camera, turret, c)
                  .minus(CameraGeometry.toRobot(a.camera, turret, c));
          heading = Math.atan2(0, expected.x) - Math.atan2(observed.y, observed.x);
          span = Math.abs(expected.x);
        }
      }
    if (span < 1) return null;
    // The same-side tag baseline gives yaw independently of robot XY and HIVE angle.
    // Within +/-30 degrees the tag-height/roll mapping is monotonic, so one seed suffices.
    LeastSquares.Fit f =
        LeastSquares.fit(
            p -> {
              double[] r = new double[tags.size() * 3];
              int i = 0;
              Pose robot = new Pose(p[0], p[1], p[2]);
              for (TagObservation t : tags) {
                Vec3 delta =
                    CameraGeometry.toField(t.camera, turret, robot, c).minus(Field.tag(t.id, p[3]));
                r[i++] = delta.x;
                r[i++] = delta.y;
                r[i++] = delta.z;
              }
              return r;
            },
            new double[] {prior.x(), prior.y(), heading, initialAngle},
            8);
    return f.valid && Math.abs(f.parameters[3]) <= Field.MAX_ANGLE + .04 && f.rms <= c.maxFitRms
        ? f
        : null;
  }
}
