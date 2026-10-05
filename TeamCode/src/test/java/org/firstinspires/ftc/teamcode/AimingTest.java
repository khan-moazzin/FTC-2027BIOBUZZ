package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.junit.Test;

public class AimingTest {
  @Test
  public void angularFeedforwardMatchesFiniteDifferenceAcrossQuadrants() {
    double dt = 1e-6;
    for (double x : new double[] {-100, 100})
      for (double y : new double[] {-60, 60}) {
        double vx = 12, vy = -7, omega = .3;
        double before = Math.atan2(y, x);
        double after = Math.atan2(y + vy * dt, x + vx * dt);
        double measured = Angles.wrap(after - before) / dt - omega;
        assertEquals(measured, MovingShotSolver.lineOfSightRate(x, y, vx, vy, omega), 1e-7);
      }
  }

  @Test
  public void angularFeedforwardUsesRelativeVelocity() {
    assertEquals(-.1, MovingShotSolver.lineOfSightRate(100, 0, 0, -10, 0), 1e-10);
    assertEquals(.1, MovingShotSolver.lineOfSightRate(100, 0, 0, 10, 0), 1e-10);
    assertEquals(-.6, MovingShotSolver.lineOfSightRate(100, 0, 0, -10, .5), 1e-10);
  }

  @Test
  public void shotMapRejectsExtrapolation() {
    ShotConfig c = new ShotConfig();
    c.calibrated = true;
    c.samples =
        new double[][] {{20, 10, 2000, .4, .3}, {60, 10, 4000, .6, .6}, {20, 40, 3000, .8, .4}};
    double[] result = ShotMap.lookup(c, 30, 15);
    assertNotNull(result);
    assertEquals(2666.6666667, result[0], 1e-5);
    assertNull(ShotMap.lookup(c, 100, 10));
    c.calibrated = false;
    assertNull(ShotMap.lookup(c, 30, 15));
  }

  @Test
  public void turretSelectsReachableEquivalentAcrossSeam() {
    assertEquals(
        Math.toRadians(-170),
        TurretTarget.choose(Math.toRadians(-170), Math.toRadians(170), -Math.PI, Math.PI),
        1e-9);
    assertEquals(.5, TurretTarget.choose(1, 0, -.5, .5), 0);
  }

  @Test
  public void signedFeedbackUsesPhysicalZero() {
    VisionConfig c = new VisionConfig();
    c.turretCalibrated = true;
    c.leftZero = c.rightZero = Math.PI;
    c.leftSign = 1;
    c.rightSign = -1;
    TurretFeedback f = new TurretFeedback();
    f.update(1.65 + .1, 1.65 - .1, 1, c);
    assertTrue(f.healthy);
    assertEquals(.1 / 3.3 * 2 * Math.PI, f.angle, 1e-9);
  }

  @Test
  public void feedbackNoiseAtTravelSeamDoesNotCommandAFullTurn() {
    assertEquals(
        Math.PI,
        TurretTarget.measured(-Math.PI + .001, Math.PI - .01, -Math.PI, Math.PI, .02),
        1e-9);
  }

  @Test
  public void movingRobotLeadsBehindStationaryCell() {
    VisionConfig vision = new VisionConfig();
    vision.hiveAccelerationNoise = 0;
    vision.maxHiveSigma = 1;
    ShotConfig shots = new ShotConfig();
    shots.calibrated = true;
    shots.samples =
        new double[][] {
          {1, 0, 3000, .5, .5},
          {140, 0, 3000, .5, .5},
          {1, 100, 3000, .5, .5},
          {140, 100, 3000, .5, .5}
        };
    org.firstinspires.ftc.teamcode.lib.vision.HiveState hive =
        new org.firstinspires.ftc.teamcode.lib.vision.HiveState();
    for (int i = 0; i < 50; i++) hive.update(0, .0001, 1000000000L + i * 20000000L, 0);
    com.pedropathing.math.Pose pose = new com.pedropathing.math.Pose(20, 85, 0);
    MovingShotSolver.Solution stationary =
        MovingShotSolver.solve(
            pose,
            com.pedropathing.math.Velocity.zero(),
            0,
            0,
            hive,
            org.firstinspires.ftc.teamcode.lib.field.Field.Hive.BLUE,
            org.firstinspires.ftc.teamcode.lib.field.Field.Cell.SCORING,
            1980000000L,
            vision,
            shots);
    MovingShotSolver.Solution moving =
        MovingShotSolver.solve(
            pose,
            new com.pedropathing.math.Velocity(0, 10, 0),
            0,
            0,
            hive,
            org.firstinspires.ftc.teamcode.lib.field.Field.Hive.BLUE,
            org.firstinspires.ftc.teamcode.lib.field.Field.Cell.SCORING,
            1980000000L,
            vision,
            shots);
    assertTrue(stationary.reason, stationary.valid);
    assertTrue(moving.reason, moving.valid);
    assertTrue(moving.angle < stationary.angle);
    assertTrue(moving.angularVelocity < 0);
  }

  @Test
  public void fullTurnEndpointsRequireAnUnambiguousStartupPosition() {
    assertTrue(TurretTarget.ambiguous(Math.PI - .001, -Math.PI, Math.PI, .02));
    assertFalse(TurretTarget.ambiguous(0, -Math.PI, Math.PI, .02));
  }
}
