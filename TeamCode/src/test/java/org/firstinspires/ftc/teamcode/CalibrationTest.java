package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.calibration.ConfigExport;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.math.LeastSquares;
import org.junit.Test;

public class CalibrationTest {
  @Test
  public void rankDeficientFitIsRejected() {
    LeastSquares.Fit f =
        LeastSquares.fit(
            p -> new double[] {p[0] + p[1] - 2, p[0] + p[1] - 2}, new double[] {0, 0}, 5);
    assertFalse(f.valid);
  }

  @Test
  public void exporterIncludesAllConfigurationFields() throws Exception {
    VisionConfig c = new VisionConfig();
    c.robotToTurret[2] = 13.5;
    String s = ConfigExport.source(c);
    for (java.lang.reflect.Field f : VisionConfig.class.getFields())
      assertTrue(s.contains(" " + f.getName() + " = "));
    assertTrue(s.contains("13.5"));
    assertTrue(ConfigExport.source(new ShotConfig()).contains("double[][] samples"));
  }

  @Test
  public void recoverExtrinsicsAndDelayFromSyntheticSweep() {
    org.firstinspires.ftc.teamcode.vision.AngleHistory history =
        new org.firstinspires.ftc.teamcode.vision.AngleHistory(10000, 100000000000L);
    for (int i = 0; i <= 4000; i++) {
      double t = i * .005;
      history.add(1000000000L + (long) (t * 1e9), .7 * Math.sin(2 * t) + .1 * Math.sin(5 * t));
    }
    VisionConfig truth = new VisionConfig();
    truth.robotToTurret = new double[] {2, -1, 12};
    truth.turretToCamera = new double[] {4, 1, 3};
    truth.cameraRotation = new double[] {.08, -.25, .12};
    truth.extraLatency = .03;
    java.util.List<org.firstinspires.ftc.teamcode.calibration.VisionCalibration.Sample> data =
        new java.util.ArrayList<>();
    for (int i = 0; i < 80; i++) {
      long t = 1500000000L + i * 180000000L;
      com.pedropathing.math.Pose pose =
          new com.pedropathing.math.Pose(30 + (i / 20) * 10, 20 + (i / 20) * 8, .3 + (i / 20) * .4);
      double angle = (i / 20) % 2 == 0 ? .4 : -.4;
      for (int id = 42; id < 46; id++) {
        org.firstinspires.ftc.teamcode.math.Vec3 point =
            org.firstinspires.ftc.teamcode.vision.CameraGeometry.toCamera(
                org.firstinspires.ftc.teamcode.field.Field.tag(id, angle),
                history.at(t - 30000000L),
                pose,
                truth);
        data.add(
            new org.firstinspires.ftc.teamcode.calibration.VisionCalibration.Sample(
                new org.firstinspires.ftc.teamcode.vision.TagObservation(id, point),
                pose,
                angle,
                t));
      }
    }
    VisionConfig seed = new VisionConfig();
    seed.robotToTurret[2] = 12;
    LeastSquares.Fit fit =
        org.firstinspires.ftc.teamcode.calibration.VisionCalibration.fit(data, history, seed);
    assertTrue(fit.valid);
    assertEquals(0, fit.rms, 1e-5);
    assertEquals(.03, fit.parameters[8], 1e-5);
    assertEquals(2, fit.parameters[0], 1e-5);
    assertEquals(3, fit.parameters[4], 1e-5);
  }
}
