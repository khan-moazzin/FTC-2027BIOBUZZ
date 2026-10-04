package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import com.pedropathing.math.Pose;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.*;
import org.firstinspires.ftc.teamcode.vision.*;
import org.junit.Test;

public class GeometryTest {
  @Test
  public void cameraRoundTripAllRotations() {
    VisionConfig c = new VisionConfig();
    c.robotToTurret = new double[] {3, -2, 10};
    c.turretToCamera = new double[] {4, 1, 2};
    c.cameraRotation = new double[] {.1, -.4, .2};
    Pose robot = new Pose(30, 40, .8);
    Vec3 p = new Vec3(50, 10, 20);
    Vec3 q = CameraGeometry.toCamera(CameraGeometry.toField(p, -1.2, robot, c), -1.2, robot, c);
    assertEquals(p.x, q.x, 1e-9);
    assertEquals(p.y, q.y, 1e-9);
    assertEquals(p.z, q.z, 1e-9);
  }

  @Test
  public void officialCadBlueScoringFixture() {
    Vec3 p = Field.tag(42, Math.PI / 6);
    assertEquals(91.2583590151, p.x, 1e-7);
    assertEquals(85.1014519191, p.y, 1e-6);
    assertEquals(49.7954385835, p.z, 1e-6);
  }

  @Test
  public void recoverRobotAndIndependentHiveAngle() {
    VisionConfig c = new VisionConfig();
    c.robotToTurret = new double[] {2, 1, 12};
    c.turretToCamera = new double[] {3, -1, 2};
    c.cameraRotation = new double[] {.1, -.2, .05};
    Pose actual = new Pose(48, 25, .7);
    for (int first : new int[] {30, 34, 38, 42})
      for (double angle : new double[] {-.52, 0, .52}) {
        List<TagObservation> tags = new ArrayList<>();
        for (int id = first; id < first + 4; id++)
          tags.add(
              new TagObservation(id, CameraGeometry.toCamera(Field.tag(id, angle), .4, actual, c)));
        LeastSquares.Fit f = VisionPoseSolver.solve(tags, .4, new Pose(50, 28, .75), 0, c);
        assertNotNull(f);
        assertEquals(actual.x(), f.parameters[0], 1e-5);
        assertEquals(actual.y(), f.parameters[1], 1e-5);
        assertEquals(actual.heading(), f.parameters[2], 1e-5);
        assertEquals(angle, f.parameters[3], 1e-5);
      }
  }

  @Test
  public void historyDoesNotBridgeBadFeedback() {
    AngleHistory h = new AngleHistory();
    h.add(100, 3.1);
    h.add(200, -3.1);
    assertEquals(Math.PI, h.at(150), 1e-9);
    h.add(300, Double.NaN);
    assertTrue(Double.isNaN(h.at(250)));
    assertTrue(Double.isNaN(h.at(99)));
  }

  @Test
  public void redBlueRemainIndependent() {
    HiveState red = new HiveState(), blue = new HiveState();
    red.update(.3, .01, 1000000000L, 1);
    blue.update(-.3, .01, 1000000000L, 1);
    red.update(.4, .01, 1100000000L, 1);
    assertTrue(red.angle(1100000000L) > 0);
    assertEquals(-.3, blue.angle(1100000000L), 1e-9);
    assertTrue(red.variance(1600000000L, 1) > red.variance(1100000000L, 1));
  }
}
