package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.opmodes.autos.paths.ThreeTipPath;
import org.junit.Test;

public class ThreeTipPathsTest {
  @Test
  public void generatedPathsAreContinuousAndFinite() {
    ThreeTipPath paths = new ThreeTipPath(Constants.Alliance.BLUE);
    Pose expectedStart = paths.start;
    for (Path path : ordered(paths)) {
      Pose actualStart = path.get(0);
      Pose end = path.endPose();
      assertEquals(expectedStart.x(), actualStart.x(), 1e-8);
      assertEquals(expectedStart.y(), actualStart.y(), 1e-8);
      assertTrue(finite(actualStart));
      assertTrue(finite(end));
      expectedStart = end;
    }
    assertEquals(128.5244, expectedStart.x(), 1e-8);
    assertEquals(48.5305, expectedStart.y(), 1e-8);
  }

  @Test
  public void redRouteMirrorsTheBlueRouteAcrossFieldCenter() {
    ThreeTipPath blue = new ThreeTipPath(Constants.Alliance.BLUE);
    ThreeTipPath red = new ThreeTipPath(Constants.Alliance.RED);
    assertEquals(144 - blue.start.x(), red.start.x(), 1e-8);
    assertEquals(blue.start.y(), red.start.y(), 1e-8);
    Path[] bluePaths = ordered(blue), redPaths = ordered(red);
    assertEquals(bluePaths.length, redPaths.length);
    for (int i = 0; i < bluePaths.length; i++) {
      assertEquals(144 - bluePaths[i].endPose().x(), redPaths[i].endPose().x(), 1e-8);
      assertEquals(bluePaths[i].endPose().y(), redPaths[i].endPose().y(), 1e-8);
    }
  }

  private static Path[] ordered(ThreeTipPath paths) {
    return new Path[] {
      paths.intake1,
      paths.intake2Prep,
      paths.intake2Flower,
      paths.intake3Prep,
      paths.intake3,
      paths.shoot1,
      paths.intake4Flower,
      paths.intake4,
      paths.shoot2,
      paths.park
    };
  }

  private static boolean finite(Pose pose) {
    return Double.isFinite(pose.x())
        && Double.isFinite(pose.y())
        && Double.isFinite(pose.heading());
  }
}
