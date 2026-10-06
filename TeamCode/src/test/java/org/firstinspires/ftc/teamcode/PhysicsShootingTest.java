package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import com.pedropathing.math.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.calibration.PhysicsCalibration;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.math.Vec3;
import org.firstinspires.ftc.teamcode.lib.vision.HiveState;
import org.junit.Test;

public class PhysicsShootingTest {
  @Test
  public void gravityTrajectoryMatchesAnalyticAndCrossesOnlyInbound() {
    Vec3 v = new Vec3(5, 0, 5);
    Vec3 p = Ballistics.atTime(v, .5, 0);
    assertEquals(2.5, p.x, 1e-10);
    assertEquals(2.5 - .5 * Ballistics.G * .25, p.z, 1e-10);
    double flight = 10 / Ballistics.G;
    Ballistics.Crossing hit =
        Ballistics.cross(new Vec3(0, 0, 0), v, new Vec3(5 * flight, 0, 0), new Vec3(0, 0, 1), 0, 2);
    assertNotNull(hit);
    assertEquals(flight, hit.time, 1e-10);
    assertEquals(0, hit.miss, 1e-10);
    assertNull(Ballistics.cross(new Vec3(0, 0, 0), v, new Vec3(0, 0, 3), new Vec3(0, 0, 1), 0, 2));
  }

  @Test
  public void quadraticDragReducesTravelAndRkMatchesZeroDragLimit() {
    Vec3 v = new Vec3(8, 1, 6);
    assertEquals(Ballistics.atTime(v, .4, 0).x, Ballistics.atTime(v, .4, 1e-9).x, 1e-7);
    assertTrue(Ballistics.atTime(v, .4, .1).x < Ballistics.atTime(v, .4, 0).x);
    assertFalse(Ballistics.atTime(v, 10, 0).finite());
  }

  @Test
  public void tiltedApertureCrossingUsesPlaneAndProjectileClearanceGeometry() {
    Vec3 v = new Vec3(4, 0, 6),
        center = Ballistics.atTime(v, 1, 0),
        normal = new Vec3(0, -.5, Math.sqrt(.75));
    Ballistics.Crossing hit = Ballistics.cross(new Vec3(0, 0, 0), v, center, normal, 0, 2);
    assertNotNull(hit);
    assertEquals(1, hit.time, 1e-8);
    assertEquals(0, hit.miss, 1e-8);
    assertNull(Ballistics.cross(new Vec3(0, 0, 0), v, center, new Vec3(0, 0, 2), 0, 2));
  }

  @Test
  public void stabilityNeedsFreshDistinctFramesAndResetsOnMotionStalenessOrSideChange() {
    PhysicsShotConfig c = new PhysicsShotConfig();
    StableHiveGate g = new StableHiveGate();
    long t = 1000000000L;
    double a = Field.MAX_ANGLE;
    assertFalse(g.update(a, 0, true, t, t, c));
    assertFalse(g.update(a, 0, true, t, t + 100000000, c));
    assertFalse(g.update(a, 0, true, t + 200000000, t + 200000000, c));
    assertTrue(g.update(a, 0, true, t + 400000000, t + 400000000, c));
    assertFalse(g.update(a, .1, true, t + 420000000, t + 420000000, c));
    assertFalse(g.update(a, 0, true, t + 440000000, t + 440000000, c));
    assertFalse(g.update(a, 0, false, t + 440000000, t + 900000000, c));
    assertFalse(g.update(-a, 0, true, t + 920000000, t + 920000000, c));
    assertFalse(g.update(0, 0, true, t + 940000000, t + 940000000, c));
  }

  @Test
  public void missedFrameGapAndBadGateSettingsCannotAccumulateDwell() {
    PhysicsShotConfig c = new PhysicsShotConfig();
    StableHiveGate g = new StableHiveGate();
    g.update(Field.MAX_ANGLE, 0, true, 1000000000L, 1000000000L, c);
    assertFalse(g.update(Field.MAX_ANGLE, 0, true, 2000000000L, 2000000000L, c));
    c.stableRate = Double.NaN;
    assertFalse(g.update(Field.MAX_ANGLE, 0, true, 2200000000L, 2200000000L, c));
  }

  @Test
  public void readyGateExpiresWithoutNewFramesAndAlliancesStayIndependent() {
    PhysicsShotConfig c = new PhysicsShotConfig();
    StableHiveGate red = new StableHiveGate(), blue = new StableHiveGate();
    double a = Field.MAX_ANGLE;
    red.update(a, 0, true, 1000000000L, 1000000000L, c);
    red.update(a, 0, true, 1200000000L, 1200000000L, c);
    assertTrue(red.update(a, 0, true, 1400000000L, 1400000000L, c));
    assertFalse(blue.update(a, 0, true, 1400000000L, 1400000000L, c));
    assertFalse(red.update(a, 0, true, 1400000000L, 1700000000L, c));
    assertFalse(red.update(a, 0, true, 1720000000L, 1720000000L, c));
  }

  private static double[][] measuredShots() {
    double[][] samples = new double[12][5];
    for (int i = 0; i < samples.length; i++) {
      double rpm = 2200 + 150 * i, angle = .7 + .02 * i, time = .35 + .01 * i;
      double v = rpm * .002, a = angle + .03;
      Vec3 p = Ballistics.atTime(new Vec3(v * Math.cos(a), 0, v * Math.sin(a)), time, 0);
      samples[i] = new double[] {p.x / .0254, p.z / .0254, rpm, angle, time};
    }
    return samples;
  }

  @Test
  public void calibrationRecoversSyntheticSpeedAndAngleWithoutEnablingGeneratedModel() {
    PhysicsShotConfig c = new PhysicsShotConfig();
    PhysicsCalibration.Result fit = PhysicsCalibration.fit(c, measuredShots());
    assertTrue(fit.reason, fit.valid);
    assertEquals(.002, c.speedPerRpm, 1e-6);
    assertEquals(.03, c.launchAngleOffset, 1e-4);
    assertFalse(c.modelGenerated);
    assertFalse(c.geometryVerified);
  }

  @Test
  public void badHeldOutMeasurementsAndMissingFlightTimeRejectCalibration() {
    PhysicsShotConfig c = new PhysicsShotConfig();
    double[][] samples = measuredShots();
    samples[2][0] += 20;
    assertFalse(PhysicsCalibration.fit(c, samples).valid);
    assertFalse(c.calibrated);
    samples = measuredShots();
    samples[0][4] = 0;
    assertFalse(PhysicsCalibration.fit(c, samples).valid);
  }

  private static PhysicsShotConfig model() {
    PhysicsShotConfig c = new PhysicsShotConfig();
    c.calibrated = c.geometryVerified = c.modelGenerated = true;
    c.speedPerRpm = .002;
    c.openingRadius = .3;
    c.projectileRadius = .04;
    // Synthetic repeatable launcher; these are not measured robot uncertainties.
    c.speedSigmaFraction = .005;
    c.angleSigma = Math.toRadians(.15);
    c.muzzleHeight = .4;
    c.targetHeight = Field.cell(Field.Hive.BLUE, Field.Cell.SCORING, Field.MAX_ANGLE).z * .0254;
    c.minDistance = 1.9;
    c.maxDistance = 2.1;
    c.maxRadialSpeed = .5;
    double angle = Math.toRadians(60), height = c.targetHeight - c.muzzleHeight;
    double speed =
        Math.sqrt(
            Ballistics.G * 4 / (2 * Math.pow(Math.cos(angle), 2) * (2 * Math.tan(angle) - height)));
    c.speedCoefficients = new double[] {speed, 0, 0, 0, 0, 0};
    c.angleCoefficients = new double[] {angle, 0, 0, 0, 0, 0};
    c.flightCoefficients = new double[] {2 / (speed * Math.cos(angle)), 0, 0, 0, 0, 0};
    c.modelSignature = ShotPolynomial.signature(c);
    return c;
  }

  private static ShotSolution solve(PhysicsShotConfig c, boolean stable, Field.Cell cell) {
    PhysicsShotConfig nominal = model();
    double flywheelPercent =
        nominal.speedCoefficients[0] / nominal.speedPerRpm / MechanismConfig.maxRpm * 100;
    double hoodDegrees =
        Math.toDegrees(nominal.angleCoefficients[0] - nominal.launchAngleOffset);
    return solve(
        c,
        stable,
        cell,
        Velocity.zero(),
        new double[][] {{70, flywheelPercent}, {90, flywheelPercent}},
        new double[][] {{70, hoodDegrees}, {90, hoodDegrees}});
  }

  private static ShotSolution solve(
      PhysicsShotConfig c,
      boolean stable,
      Field.Cell cell,
      Velocity velocity,
      double[][] flywheelMap,
      double[][] hoodMap) {
    VisionConfig vision = new VisionConfig();
    vision.robotToTurret[2] = c.muzzleHeight / .0254;
    vision.hiveAccelerationNoise = 0;
    ShotConfig shots = new ShotConfig();
    shots.calibrated = true;
    shots.flywheelMap = flywheelMap;
    shots.hoodMap = hoodMap;
    HiveState hive = new HiveState();
    long now = 2000000000L;
    hive.update(Field.MAX_ANGLE, .000001, now, 0);
    Vec3 goal = Field.cell(Field.Hive.BLUE, Field.Cell.SCORING, Field.MAX_ANGLE);
    return StableShotSolver.solve(
        new Pose(goal.x - 2 / .0254, goal.y, 0),
        velocity,
        0,
        0,
        hive,
        Field.Hive.BLUE,
        cell,
        now,
        vision,
        shots,
        c,
        stable);
  }

  @Test
  public void stableHybridSolverUsesMapsAndValidatesWithPhysics() {
    PhysicsShotConfig c = model();
    ShotSolution shot = solve(c, true, Field.Cell.SCORING);
    assertTrue(shot.reason, shot.valid);
    assertTrue(shot.physicsCorrected);
    assertTrue(shot.clearance > 0);
    assertEquals(27, shot.iterations);
    assertFalse(solve(c, false, Field.Cell.SCORING).valid);
    assertFalse(solve(c, true, Field.Cell.AUDIENCE).valid);
  }

  @Test
  public void changedOrMissingPhysicsFallsBackToEmpiricalMaps() {
    PhysicsShotConfig c = model();
    c.speedPerRpm *= 1.1;
    assertTrue(solve(c, true, Field.Cell.SCORING).valid);
    assertFalse(solve(c, true, Field.Cell.SCORING).physicsCorrected);
    c = model();
    c.modelGenerated = false;
    assertTrue(solve(c, true, Field.Cell.SCORING).valid);
    c = model();
    c.speedCoefficients[0] = Double.NaN;
    assertTrue(solve(c, true, Field.Cell.SCORING).valid);
    c = model();
    c.openingRadius = .045;
    c.modelSignature = ShotPolynomial.signature(c);
    ShotSolution fallback = solve(c, true, Field.Cell.SCORING);
    assertTrue(fallback.reason, fallback.valid);
    assertFalse(fallback.physicsCorrected);
    assertEquals(0, fallback.iterations);
  }

  @Test
  public void physicsMotionCorrectionCannotLeaveTheEmpiricalDistanceZone() {
    PhysicsShotConfig c = model();
    double basePercent =
        c.speedCoefficients[0] / c.speedPerRpm / MechanismConfig.maxRpm * 100;
    double baseHood = Math.toDegrees(c.angleCoefficients[0] - c.launchAngleOffset);
    c.speedCoefficients[2] = .5;
    c.angleCoefficients[2] = Math.toRadians(5);
    c.openingRadius = .8;
    c.modelSignature = ShotPolynomial.signature(c);
    ShotSolution shot =
        solve(
            c,
            true,
            Field.Cell.SCORING,
            new Velocity(10, 0, 0),
            new double[][] {{70, basePercent - 1}, {90, basePercent + 1}},
            new double[][] {{70, baseHood - 1}, {90, baseHood + 1}});
    assertTrue(shot.reason, shot.valid);
    assertTrue(shot.physicsCorrected);
    assertEquals((basePercent + 1) / 100 * MechanismConfig.maxRpm, shot.rpm, 1e-8);
    assertEquals(Math.toRadians(baseHood + 1), shot.hood, 1e-8);
  }
}
